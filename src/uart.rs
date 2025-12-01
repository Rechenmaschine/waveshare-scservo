use crate::{Instruction, ProtocolError};
use device_driver::RegisterInterface;
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};

const BROADCAST_ID: u8 = 0xFE;
const HEADER_BYTE: u8 = 0xFF;

pub struct UartBusInterface<I> {
    interface: I,
    id: Option<u8>,
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VersionInformation{
    pub firmware_major_version: u8,
    pub firmware_minor_version: u8,
    pub servo_major_version: u8,
    pub servo_minor_version: u8,
}


// blocking impl
impl<I> UartBusInterface<I>
where
    I: BlockingRead + BlockingWrite,
{
    pub fn new(interface: I) -> Self {
        UartBusInterface { interface, id: None }
    }

    pub fn ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }

        let mut response = [];
        self.transfer(id, Instruction::Ping, &[], &mut response)?;
        Ok(())
    }

    pub fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.transfer(id, Instruction::Reset, &[], &mut response)?;
        Ok(())
    }

    pub fn action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.transfer(id, Instruction::RegAction, &[], &mut response)?;
        Ok(())
    }

    pub fn reg_write(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        if data.len() + 1 > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1..1 + data.len()].copy_from_slice(data);

        let mut response = [];
        self.transfer(id, Instruction::RegWrite, &params[..1 + data.len()], &mut response)?;
        Ok(())
    }

    pub fn sync_write(
        &mut self,
        address: u8,
        data_len: u8,
        payload: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        if 2 + payload.len() > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1] = data_len;
        params[2..2 + payload.len()].copy_from_slice(payload);

        let mut response = [];
        self.transfer(BROADCAST_ID, Instruction::SyncWrite, &params[..2 + payload.len()], &mut response)?;
        Ok(())
    }

    pub fn sync_read(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
        output: &mut [u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        if 2 + ids.len() > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1] = data_len;
        params[2..2 + ids.len()].copy_from_slice(ids);

        let param_slice = &params[..2 + ids.len()];
        let length = (param_slice.len() + 2) as u8;
        let id = BROADCAST_ID;
        let instruction = Instruction::SyncRead;

        let checksum = Self::calculate_checksum(id, length, instruction as u8, param_slice);
        let header = [HEADER_BYTE, HEADER_BYTE, id, length, instruction as u8];

        self.interface
            .write_all(&header)
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(param_slice)
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(&[checksum])
            .map_err(ProtocolError::Serial)?;

        if output.len() < ids.len() * data_len as usize {
            return Err(ProtocolError::InvalidLength);
        }

        for (i, &expected_id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            let end = start + data_len as usize;
            self.read_response(expected_id, &mut output[start..end])?;
        }

        Ok(())
    }

    fn calculate_checksum(id: u8, length: u8, instruction: u8, params: &[u8]) -> u8 {
        let mut sum: u32 = id as u32 + length as u32 + instruction as u32;
        for &p in params {
            sum += p as u32;
        }
        !(sum as u8)
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ProtocolError<I::Error>> {
        self.interface.read_exact(buf).map_err(|e| match e {
            embedded_io::ReadExactError::UnexpectedEof => ProtocolError::InvalidLength,
            embedded_io::ReadExactError::Other(e) => ProtocolError::Serial(e),
        })
    }

    fn read_byte(&mut self) -> Result<u8, ProtocolError<I::Error>> {
        let mut buf = [0u8; 1];
        self.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    pub fn set_busid(&mut self, new_id: u8) {
        self.id = Some(new_id);
    }
    pub fn clear_busid(&mut self) {
        self.id = None;
    }

    fn read_response(
        &mut self,
        expected_id: u8,
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        // 1. Scan for Header 0xFF 0xFF
        loop {
            let b = self.read_byte()?;
            if b == HEADER_BYTE {
                let b2 = self.read_byte()?;
                if b2 == HEADER_BYTE {
                    break;
                }
            }
        }

        // 2. Read ID, Length
        let received_id = self.read_byte()?;
        let length = self.read_byte()?;

        if received_id != expected_id {
            return Err(ProtocolError::InvalidId);
        }

        // Length = N + 2.
        // Body = Error(1) + Params(N) + Checksum(1).
        // So Body Length = Length.
        let body_len = length as usize;
        if body_len < 2 {
            return Err(ProtocolError::InvalidLength);
        }

        let mut body_buf = [0u8; 256];
        let body = &mut body_buf[..body_len];
        self.read_exact(body)?;

        let error = body[0];
        let received_checksum = body[body_len - 1];
        let response_params = &body[1..body_len - 1];

        // Verify Checksum
        // Checksum = ~(ID + Length + Error + Params...)
        let mut sum: u32 = received_id as u32 + length as u32;
        for &b in &body[..body_len - 1] {
            sum += b as u32;
        }
        let calculated_checksum = !(sum as u8);

        if calculated_checksum != received_checksum {
            return Err(ProtocolError::Checksum);
        }

        if error != 0 {
            return Err(ProtocolError::ServoError(error));
        }

        // Copy params
        let copy_len = response_buf.len().min(response_params.len());
        response_buf[..copy_len].copy_from_slice(&response_params[..copy_len]);

        Ok(response_params.len())
    }

    /// Generic transfer function
    /// Sends instruction and parameters, and reads response if applicable.
    /// Returns the number of bytes read into `response_buf`.
    pub fn transfer(
        &mut self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        let length = (params.len() + 2) as u8;
        let checksum = Self::calculate_checksum(id, length, instruction as u8, params);
        let header = [HEADER_BYTE, HEADER_BYTE, id, length, instruction as u8];

        self.interface
            .write_all(&header)
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(params)
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(&[checksum])
            .map_err(ProtocolError::Serial)?;

        // If broadcast ID (0xFE), usually no response, except PING (0x01) is not allowed for broadcast.
        if id == BROADCAST_ID {
            return Ok(0);
        }

        self.read_response(id, response_buf)
    }
}

impl<I> RegisterInterface for UartBusInterface<I>
where
    I: BlockingRead + BlockingWrite,
{
    type Error = ProtocolError<I::Error>;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // todo does this really need to be this long? We want to avoid allocation.
        let mut params = [0u8; 256];
        if data.len() + 1 > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1..1 + data.len()].copy_from_slice(data);

        let mut response = [];
        self.transfer(self.id.unwrap(), Instruction::Write, &params[..1 + data.len()], &mut response)?;
        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let len = data.len() as u8;
        let params = [address, len];
        self.transfer(self.id.unwrap(), Instruction::Read, &params, data)?;
        Ok(())
    }
}
