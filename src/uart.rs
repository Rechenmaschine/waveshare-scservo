use crate::{Instruction, ProtocolError};
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

type BlockingExactError<E> = embedded_io::ReadExactError<E>;
type AsyncExactError<E> = embedded_io_async::ReadExactError<E>;

const HEADER_BYTE: u8 = 0xFF;

pub struct UartBusInterface<I> {
    pub(crate) interface: I,
    pub(crate) id: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VersionInformation {
    pub firmware_major: u8,
    pub firmware_minor: u8,
    pub servo_major: u8,
    pub servo_minor: u8,
}

impl<I> UartBusInterface<I> {
    pub fn new(interface: I) -> Self {
        UartBusInterface {
            interface,
            id: None,
        }
    }

    pub fn set_busid(&mut self, new_id: u8) {
        self.id = Some(new_id);
    }

    pub fn clear_busid(&mut self) {
        self.id = None;
    }

    fn calculate_checksum(id: u8, length: u8, instruction: u8, params: &[u8]) -> u8 {
        let mut sum = u32::from(id) + u32::from(length) + u32::from(instruction);
        for &p in params {
            sum += u32::from(p);
        }
        #[allow(clippy::cast_possible_truncation)]
        !(sum as u8)
    }

    fn copy_reg_write_params<E>(
        params: &mut [u8; 256],
        address: u8,
        data: &[u8],
    ) -> Result<usize, ProtocolError<E>> {
        if data.len() + 1 > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1..=data.len()].copy_from_slice(data);
        Ok(data.len() + 1)
    }
}

impl<I> UartBusInterface<I>
where
    I: BlockingRead + BlockingWrite,
{
    pub fn blocking_ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }

        let mut response = [];
        self.blocking_transfer(id, Instruction::Ping, &[], &mut response)?;
        Ok(())
    }

    pub fn blocking_reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.blocking_transfer(id, Instruction::Reset, &[], &mut response)?;
        Ok(())
    }

    pub fn blocking_action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.blocking_transfer(id, Instruction::RegAction, &[], &mut response)?;
        Ok(())
    }

    pub fn blocking_reg_write(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.blocking_transfer(id, Instruction::RegWrite, &params[..len], &mut response)?;
        Ok(())
    }

    pub fn blocking_sync_write(
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
        self.blocking_transfer(
            crate::BROADCAST_ID,
            Instruction::SyncWrite,
            &params[..2 + payload.len()],
            &mut response,
        )?;
        Ok(())
    }

    pub fn blocking_send_sync_read_request(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        if 2 + ids.len() > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1] = data_len;
        params[2..2 + ids.len()].copy_from_slice(ids);

        let param_slice = &params[..2 + ids.len()];
        let length =
            u8::try_from(param_slice.len() + 2).map_err(|_| ProtocolError::InvalidLength)?;
        let id = crate::BROADCAST_ID;
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
        Ok(())
    }

    pub fn blocking_sync_read(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
        output: &mut [u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        self.blocking_send_sync_read_request(address, data_len, ids)?;

        if output.len() < ids.len() * data_len as usize {
            return Err(ProtocolError::InvalidLength);
        }

        for (i, &expected_id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            let end = start + data_len as usize;
            self.blocking_read_response(expected_id, &mut output[start..end])?;
        }

        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ProtocolError<I::Error>> {
        self.interface.read_exact(buf).map_err(|e| match e {
            BlockingExactError::UnexpectedEof => ProtocolError::InvalidLength,
            BlockingExactError::Other(e) => ProtocolError::Serial(e),
        })
    }

    fn read_byte(&mut self) -> Result<u8, ProtocolError<I::Error>> {
        let mut buf = [0u8; 1];
        self.read_exact(&mut buf)?;
        Ok(buf[0])
    }

    pub fn blocking_read_response(
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
        let mut sum = u32::from(received_id) + u32::from(length);
        for &b in &body[..body_len - 1] {
            sum += u32::from(b);
        }
        #[allow(clippy::cast_possible_truncation)]
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

    pub fn blocking_transfer(
        &mut self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        let length = u8::try_from(params.len() + 2).map_err(|_| ProtocolError::InvalidLength)?;
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

        if id == crate::BROADCAST_ID {
            return Ok(0);
        }

        self.blocking_read_response(id, response_buf)
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
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.blocking_transfer(
            self.id.unwrap(),
            Instruction::Write,
            &params[..len],
            &mut response,
        )?;
        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let len = u8::try_from(data.len()).map_err(|_| ProtocolError::InvalidLength)?;
        let params = [address, len];
        self.blocking_transfer(self.id.unwrap(), Instruction::Read, &params, data)?;
        Ok(())
    }
}

impl<I> UartBusInterface<I>
where
    I: AsyncRead + AsyncWrite,
{
    async fn read_exact_async(&mut self, buf: &mut [u8]) -> Result<(), ProtocolError<I::Error>> {
        self.interface.read_exact(buf).await.map_err(|e| match e {
            AsyncExactError::UnexpectedEof => ProtocolError::InvalidLength,
            AsyncExactError::Other(e) => ProtocolError::Serial(e),
        })
    }

    async fn read_byte_async(&mut self) -> Result<u8, ProtocolError<I::Error>> {
        let mut buf = [0u8; 1];
        self.read_exact_async(&mut buf).await?;
        Ok(buf[0])
    }

    pub async fn ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }

        let mut response = [];
        self.transfer_async(id, Instruction::Ping, &[], &mut response)
            .await?;
        Ok(())
    }

    pub async fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.transfer_async(id, Instruction::Reset, &[], &mut response)
            .await?;
        Ok(())
    }

    pub async fn action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut response = [];
        self.transfer_async(id, Instruction::RegAction, &[], &mut response)
            .await?;
        Ok(())
    }

    pub async fn reg_write(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.transfer_async(id, Instruction::RegWrite, &params[..len], &mut response)
            .await?;
        Ok(())
    }

    pub async fn sync_write(
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
        self.transfer_async(
            crate::BROADCAST_ID,
            Instruction::SyncWrite,
            &params[..2 + payload.len()],
            &mut response,
        )
        .await?;
        Ok(())
    }

    pub async fn send_sync_read_request(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut params = [0u8; 256];
        if 2 + ids.len() > params.len() {
            return Err(ProtocolError::InvalidLength);
        }
        params[0] = address;
        params[1] = data_len;
        params[2..2 + ids.len()].copy_from_slice(ids);

        let param_slice = &params[..2 + ids.len()];
        let length =
            u8::try_from(param_slice.len() + 2).map_err(|_| ProtocolError::InvalidLength)?;
        let id = crate::BROADCAST_ID;
        let instruction = Instruction::SyncRead;

        let checksum = Self::calculate_checksum(id, length, instruction as u8, param_slice);
        let header = [HEADER_BYTE, HEADER_BYTE, id, length, instruction as u8];

        self.interface
            .write_all(&header)
            .await
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(param_slice)
            .await
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(&[checksum])
            .await
            .map_err(ProtocolError::Serial)?;
        Ok(())
    }

    pub async fn sync_read(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
        output: &mut [u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        self.send_sync_read_request(address, data_len, ids).await?;

        if output.len() < ids.len() * data_len as usize {
            return Err(ProtocolError::InvalidLength);
        }

        for (i, &expected_id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            let end = start + data_len as usize;
            self.read_response_async(expected_id, &mut output[start..end])
                .await?;
        }

        Ok(())
    }

    pub async fn read_response_async(
        &mut self,
        expected_id: u8,
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        // 1. Scan for Header 0xFF 0xFF
        loop {
            let b = self.read_byte_async().await?;
            if b == HEADER_BYTE {
                let b2 = self.read_byte_async().await?;
                if b2 == HEADER_BYTE {
                    break;
                }
            }
        }

        // 2. Read ID, Length
        let received_id = self.read_byte_async().await?;
        let length = self.read_byte_async().await?;

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
        self.read_exact_async(body).await?;

        let error = body[0];
        let received_checksum = body[body_len - 1];
        let response_params = &body[1..body_len - 1];

        // Verify Checksum
        // Checksum = ~(ID + Length + Error + Params...)
        let mut sum = u32::from(received_id) + u32::from(length);
        for &b in &body[..body_len - 1] {
            sum += u32::from(b);
        }
        #[allow(clippy::cast_possible_truncation)]
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

    pub async fn transfer_async(
        &mut self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        let length = u8::try_from(params.len() + 2).map_err(|_| ProtocolError::InvalidLength)?;
        let checksum = Self::calculate_checksum(id, length, instruction as u8, params);
        let header = [HEADER_BYTE, HEADER_BYTE, id, length, instruction as u8];

        self.interface
            .write_all(&header)
            .await
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(params)
            .await
            .map_err(ProtocolError::Serial)?;
        self.interface
            .write_all(&[checksum])
            .await
            .map_err(ProtocolError::Serial)?;

        if id == crate::BROADCAST_ID {
            return Ok(0);
        }

        self.read_response_async(id, response_buf).await
    }
}

impl<I> AsyncRegisterInterface for UartBusInterface<I>
where
    I: AsyncRead + AsyncWrite,
{
    type Error = ProtocolError<I::Error>;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.transfer_async(
            self.id.unwrap(),
            Instruction::Write,
            &params[..len],
            &mut response,
        )
        .await?;
        Ok(())
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let len = u8::try_from(data.len()).map_err(|_| ProtocolError::InvalidLength)?;
        let params = [address, len];
        self.transfer_async(self.id.unwrap(), Instruction::Read, &params, data)
            .await?;
        Ok(())
    }
}
