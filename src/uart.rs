use crate::error::ProtocolError;
use crate::types::Instruction;
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

type BlockingExactError<E> = embedded_io::ReadExactError<E>;
type AsyncExactError<E> = embedded_io_async::ReadExactError<E>;

const HEADER_BYTE: u8 = 0xFF;

pub struct UartBusInterface<I> {
    pub(crate) interface: I,
    pub(crate) id: Option<u8>,
    response_status_level: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Firmware and servo version bytes from the standard control table.
pub struct VersionInformation {
    /// Firmware major version (control-table address 0).
    pub firmware_major: u8,
    /// Firmware minor version (control-table address 1).
    pub firmware_minor: u8,
    /// Servo major/model version (control-table address 3).
    pub servo_major: u8,
    /// Servo minor/version value (control-table address 4).
    pub servo_minor: u8,
}

impl<I> UartBusInterface<I> {
    pub fn new(interface: I) -> Self {
        UartBusInterface {
            interface,
            id: None,
            response_status_level: true,
        }
    }

    pub fn set_response_status_level(&mut self, enabled: bool) {
        self.response_status_level = enabled;
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

    fn validate_id<E>(id: u8, allow_broadcast: bool) -> Result<(), ProtocolError<E>> {
        if id <= crate::MAX_SERVO_ID || (allow_broadcast && id == crate::BROADCAST_ID) {
            Ok(())
        } else {
            Err(ProtocolError::InvalidId)
        }
    }

    fn validate_sync_write<E>(data_len: u8, payload: &[u8]) -> Result<(), ProtocolError<E>> {
        if data_len == 0 {
            return Err(ProtocolError::InvalidLength);
        }

        let entry_len = usize::from(data_len) + 1;
        if payload.is_empty() || !payload.len().is_multiple_of(entry_len) {
            return Err(ProtocolError::InvalidLength);
        }

        for entry in payload.chunks_exact(entry_len) {
            Self::validate_id(entry[0], false)?;
        }
        Ok(())
    }

    fn validate_sync_read<E>(data_len: u8, ids: &[u8]) -> Result<(), ProtocolError<E>> {
        if data_len == 0 || ids.is_empty() {
            return Err(ProtocolError::InvalidLength);
        }
        for &id in ids {
            Self::validate_id(id, false)?;
        }
        Ok(())
    }

    fn copy_reg_write_params<E>(
        params: &mut [u8; 256],
        address: u8,
        data: &[u8],
    ) -> Result<usize, ProtocolError<E>> {
        if data.is_empty() || data.len() >= params.len() {
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
        Self::validate_id(id, false)?;

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
        Self::validate_sync_write(data_len, payload)?;
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
        Self::validate_sync_read(data_len, ids)?;
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
        self.interface.flush().map_err(ProtocolError::Serial)?;
        Ok(())
    }

    pub fn blocking_sync_read(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
        output: &mut [u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        Self::validate_sync_read(data_len, ids)?;
        let required_len = ids
            .len()
            .checked_mul(usize::from(data_len))
            .ok_or(ProtocolError::InvalidLength)?;
        if output.len() < required_len {
            return Err(ProtocolError::InvalidLength);
        }

        self.blocking_send_sync_read_request(address, data_len, ids)?;

        for (i, &expected_id) in ids.iter().enumerate() {
            let start = i * usize::from(data_len);
            let end = start + usize::from(data_len);
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
        Self::validate_id(expected_id, false)?;

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

        if response_params.len() != response_buf.len() {
            return Err(ProtocolError::InvalidLength);
        }

        // Checksum validation
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

        // Consume and validate the complete packet before reporting an ID
        // mismatch, so a stray response cannot poison the next transaction.
        if received_id != expected_id {
            return Err(ProtocolError::InvalidId);
        }

        if error != 0 {
            return Err(ProtocolError::ServoError(error));
        }

        // Copy params
        response_buf.copy_from_slice(response_params);

        Ok(response_params.len())
    }

    pub(crate) fn blocking_transfer(
        &mut self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID && !response_buf.is_empty() {
            return Err(ProtocolError::InvalidId);
        }
        Self::validate_id(id, true)?;
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
        self.interface.flush().map_err(ProtocolError::Serial)?;

        if id == crate::BROADCAST_ID
            || (!self.response_status_level
                && !matches!(instruction, Instruction::Ping | Instruction::Read))
        {
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
        let id = self.id.ok_or(ProtocolError::InvalidId)?;
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.blocking_transfer(id, Instruction::Write, &params[..len], &mut response)?;
        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let id = self.id.ok_or(ProtocolError::InvalidId)?;
        let len = u8::try_from(data.len()).map_err(|_| ProtocolError::InvalidLength)?;
        let params = [address, len];
        self.blocking_transfer(id, Instruction::Read, &params, data)?;
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
        Self::validate_id(id, false)?;

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
        Self::validate_sync_write(data_len, payload)?;
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
        Self::validate_sync_read(data_len, ids)?;
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
        self.interface
            .flush()
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
        Self::validate_sync_read(data_len, ids)?;
        let required_len = ids
            .len()
            .checked_mul(usize::from(data_len))
            .ok_or(ProtocolError::InvalidLength)?;
        if output.len() < required_len {
            return Err(ProtocolError::InvalidLength);
        }

        self.send_sync_read_request(address, data_len, ids).await?;

        for (i, &expected_id) in ids.iter().enumerate() {
            let start = i * usize::from(data_len);
            let end = start + usize::from(data_len);
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
        Self::validate_id(expected_id, false)?;

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

        if response_params.len() != response_buf.len() {
            return Err(ProtocolError::InvalidLength);
        }

        // Checksum validation
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

        // Consume and validate the complete packet before reporting an ID
        // mismatch, so a stray response cannot poison the next transaction.
        if received_id != expected_id {
            return Err(ProtocolError::InvalidId);
        }

        if error != 0 {
            return Err(ProtocolError::ServoError(error));
        }

        // Copy params
        response_buf.copy_from_slice(response_params);

        Ok(response_params.len())
    }

    pub(crate) async fn transfer_async(
        &mut self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
        response_buf: &mut [u8],
    ) -> Result<usize, ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID && !response_buf.is_empty() {
            return Err(ProtocolError::InvalidId);
        }
        Self::validate_id(id, true)?;
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
        self.interface
            .flush()
            .await
            .map_err(ProtocolError::Serial)?;

        if id == crate::BROADCAST_ID
            || (!self.response_status_level
                && !matches!(instruction, Instruction::Ping | Instruction::Read))
        {
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
        let id = self.id.ok_or(ProtocolError::InvalidId)?;
        let mut params = [0u8; 256];
        let len = Self::copy_reg_write_params(&mut params, address, data)?;

        let mut response = [];
        self.transfer_async(id, Instruction::Write, &params[..len], &mut response)
            .await?;
        Ok(())
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let id = self.id.ok_or(ProtocolError::InvalidId)?;
        let len = u8::try_from(data.len()).map_err(|_| ProtocolError::InvalidLength)?;
        let params = [address, len];
        self.transfer_async(id, Instruction::Read, &params, data)
            .await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_io::ErrorType;
    use std::{vec, vec::Vec};

    struct ScriptedInterface {
        rx: Vec<u8>,
        tx: Vec<u8>,
        flushes: usize,
    }

    impl ErrorType for ScriptedInterface {
        type Error = embedded_io::ErrorKind;
    }

    impl BlockingRead for ScriptedInterface {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.rx.is_empty() {
                return Ok(0);
            }
            let count = buf.len().min(self.rx.len());
            buf[..count].copy_from_slice(&self.rx[..count]);
            self.rx.drain(..count);
            Ok(count)
        }
    }

    impl BlockingWrite for ScriptedInterface {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.flushes += 1;
            Ok(())
        }
    }

    #[test]
    fn documented_read_packet_round_trips() {
        let interface = ScriptedInterface {
            // Waveshare manual: response to reading address 0x38 from ID 1.
            rx: vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD],
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let mut response = [0u8; 2];

        let length = uart
            .blocking_transfer(1, Instruction::Read, &[0x38, 0x02], &mut response)
            .unwrap();

        assert_eq!(length, 2);
        assert_eq!(response, [0x18, 0x05]);
        assert_eq!(
            uart.interface.tx,
            [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
        assert_eq!(uart.interface.flushes, 1);
    }

    #[test]
    fn documented_sync_write_packet_round_trips() {
        let interface = ScriptedInterface {
            rx: Vec::new(),
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let payload = [
            1, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 2, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 3, 0x00,
            0x08, 0x00, 0x00, 0xE8, 0x03, 4, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
        ];

        uart.blocking_sync_write(0x2A, 6, &payload).unwrap();

        assert_eq!(
            uart.interface.tx,
            [
                0xFF, 0xFF, 0xFE, 0x20, 0x83, 0x2A, 0x06, 1, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 2,
                0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 3, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 4, 0x00,
                0x08, 0x00, 0x00, 0xE8, 0x03, 0x58,
            ]
        );
        assert_eq!(uart.interface.flushes, 1);
    }

    #[test]
    fn response_length_must_match_destination() {
        let interface = ScriptedInterface {
            rx: vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x18, 0xE3],
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let mut response = [0u8; 2];

        assert!(matches!(
            uart.blocking_read_response(1, &mut response),
            Err(ProtocolError::InvalidLength)
        ));
    }

    #[test]
    fn mismatched_response_is_consumed_before_error() {
        let interface = ScriptedInterface {
            rx: vec![0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB],
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let mut response = [];

        assert!(matches!(
            uart.blocking_read_response(1, &mut response),
            Err(ProtocolError::InvalidId)
        ));
        assert!(uart.interface.rx.is_empty());
    }

    #[test]
    fn sync_write_rejects_misaligned_payload_without_transmitting() {
        let interface = ScriptedInterface {
            rx: Vec::new(),
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);

        assert!(matches!(
            uart.blocking_sync_write(0x2A, 6, &[1, 0]),
            Err(ProtocolError::InvalidLength)
        ));
        assert!(uart.interface.tx.is_empty());
    }

    #[test]
    fn register_access_without_bus_id_is_an_error() {
        let interface = ScriptedInterface {
            rx: Vec::new(),
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let mut data = [0u8; 1];

        assert!(matches!(
            RegisterInterface::read_register(&mut uart, 0x38, 8, &mut data),
            Err(ProtocolError::InvalidId)
        ));
    }

    #[test]
    fn broadcast_transfer_cannot_claim_a_response() {
        let interface = ScriptedInterface {
            rx: Vec::new(),
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        let mut response = [0u8; 1];

        assert!(matches!(
            uart.blocking_transfer(crate::BROADCAST_ID, Instruction::Read, &[], &mut response),
            Err(ProtocolError::InvalidId)
        ));
        assert!(uart.interface.tx.is_empty());
    }

    #[test]
    fn response_level_zero_skips_write_ack() {
        let interface = ScriptedInterface {
            rx: Vec::new(),
            tx: Vec::new(),
            flushes: 0,
        };
        let mut uart = UartBusInterface::new(interface);
        uart.set_response_status_level(false);

        uart.blocking_transfer(1, Instruction::Write, &[0x28, 1], &mut [])
            .unwrap();

        assert_eq!(uart.interface.tx, [0xFF, 0xFF, 1, 4, 0x03, 0x28, 1, 0xCE]);
        assert_eq!(uart.interface.flushes, 1);
    }
}
