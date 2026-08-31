//! SMS/STS series servo bus (STS3215, etc.).
//!
//! Magnetic encoder servos using little-endian wire format.

use crate::error::ProtocolError;
use crate::registers::sts_device::{OperatingMode, SmsStsDevice, TorqueMode as SmsTorqueMode};
use crate::series::{ServoMode, ServoTelemetry};
use crate::types::{
    ScsStatus, SmsPositionMove, SmsPositionMoveEx, SmsServoState, SmsSpeedCommand,
    SmsStsOperatingMode, SmsTorqueLimitCommand, SyncWriteData, TorqueModeCommand,
};
use crate::uart::UartBusInterface;
use crate::{TorqueMode, VersionInformation, registers};
use crate::{decode_current, decode_load, decode_speed};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

const BIT_15_SIGN: u16 = 0x8000;
const BIT_15_VALUE: u16 = 0x7FFF;

/// SMS_STS-specific register addresses
mod addr {
    /// Torque switch register address (0x28)
    pub const TORQUE_SWITCH: u8 = 0x28;
    /// Acceleration register address (0x29)
    pub const ACCELERATION: u8 = 0x29;
    /// Torque limit register address (0x30, runtime limit)
    pub const TORQUE_LIMIT: u8 = 0x30;
}

/// Servo bus for SMS_STS series (STS3215, etc.).
///
/// Uses little-endian wire format.
pub struct SmsStsBus<I> {
    device: SmsStsDevice<UartBusInterface<I>>,
}

struct BusIdGuard<'a, I> {
    device: &'a mut SmsStsDevice<UartBusInterface<I>>,
}

impl<'a, I> BusIdGuard<'a, I> {
    fn new(device: &'a mut SmsStsDevice<UartBusInterface<I>>, id: u8) -> Self {
        device.interface.set_busid(id);
        Self { device }
    }
}

impl<I> Drop for BusIdGuard<'_, I> {
    fn drop(&mut self) {
        self.device.interface.clear_busid();
    }
}

impl<I> core::ops::Deref for BusIdGuard<'_, I> {
    type Target = SmsStsDevice<UartBusInterface<I>>;
    fn deref(&self) -> &Self::Target {
        self.device
    }
}

impl<I> core::ops::DerefMut for BusIdGuard<'_, I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.device
    }
}

fn with_blocking_eeprom<I, T, F>(
    device: &mut SmsStsDevice<UartBusInterface<I>>,
    operation: F,
) -> Result<T, ProtocolError<I::Error>>
where
    I: BlockingRead + BlockingWrite,
    F: FnOnce(&mut SmsStsDevice<UartBusInterface<I>>) -> Result<T, ProtocolError<I::Error>>,
{
    device.blocking_unlock_eeprom()?;
    let operation_result = operation(device);
    let lock_result = device.blocking_lock_eeprom();

    match (operation_result, lock_result) {
        (Err(operation_error), _) => Err(operation_error),
        (Ok(value), Ok(())) => Ok(value),
        (Ok(_), Err(lock_error)) => Err(lock_error),
    }
}

fn encode_position_payload(position: i16, _time: u16, speed: u16) -> Option<[u8; 6]> {
    let p = encode_signed_position(position)?.to_le_bytes();
    let s = speed.to_le_bytes();
    Some([p[0], p[1], 0, 0, s[0], s[1]])
}

fn fill_sync_position_payload<E>(
    moves: &[SmsPositionMove],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 6;
    let mut offset = 0;
    for m in moves {
        let p = encode_signed_position(m.position).ok_or(ProtocolError::InvalidSetting)?;
        if offset + 7 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = m.id;
        let p = p.to_le_bytes();
        let s = m.speed.to_le_bytes();
        payload[offset + 1] = p[0];
        payload[offset + 2] = p[1];
        payload[offset + 3] = 0;
        payload[offset + 4] = 0;
        payload[offset + 5] = s[0];
        payload[offset + 6] = s[1];
        offset += 7;
    }
    Ok((data_len, offset))
}

fn fill_sync_position_ex_payload<E>(
    moves: &[SmsPositionMoveEx],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 7; // Includes acceleration byte
    let mut offset = 0;
    for m in moves {
        if m.acceleration > 254 {
            return Err(ProtocolError::InvalidSetting);
        }
        let p = encode_signed_position(m.position).ok_or(ProtocolError::InvalidSetting)?;
        if offset + 8 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = m.id;
        payload[offset + 1] = m.acceleration;
        let p = p.to_le_bytes();
        let s = m.speed.to_le_bytes();
        payload[offset + 2] = p[0];
        payload[offset + 3] = p[1];
        payload[offset + 4] = 0;
        payload[offset + 5] = 0;
        payload[offset + 6] = s[0];
        payload[offset + 7] = s[1];
        offset += 8;
    }
    Ok((data_len, offset))
}

fn encode_signed_speed(speed: i16) -> Option<u16> {
    let magnitude = speed.unsigned_abs();
    if magnitude > BIT_15_VALUE {
        return None;
    }
    if speed < 0 {
        Some(BIT_15_SIGN | magnitude)
    } else {
        Some(magnitude)
    }
}

fn from_register_operating_mode(mode: OperatingMode) -> Option<SmsStsOperatingMode> {
    match mode {
        OperatingMode::Position => Some(SmsStsOperatingMode::Position),
        OperatingMode::Wheel => Some(SmsStsOperatingMode::Wheel),
        OperatingMode::PwmOpenLoop => Some(SmsStsOperatingMode::PwmOpenLoop),
        OperatingMode::Step => Some(SmsStsOperatingMode::Step),
        OperatingMode::Unknown(_) => None,
    }
}

fn to_register_operating_mode(mode: SmsStsOperatingMode) -> OperatingMode {
    match mode {
        SmsStsOperatingMode::Position => OperatingMode::Position,
        SmsStsOperatingMode::Wheel => OperatingMode::Wheel,
        SmsStsOperatingMode::PwmOpenLoop => OperatingMode::PwmOpenLoop,
        SmsStsOperatingMode::Step => OperatingMode::Step,
    }
}

fn fill_sync_speed_payload<E>(
    commands: &[SmsSpeedCommand],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    // SMS_STS wheel mode speed control writes 7 bytes starting at ACCELERATION (0x29):
    // - ACC (1 byte at 0x29)
    // - Position (2 bytes at 0x2A, ignored in wheel mode, set to 0)
    // - Time (2 bytes at 0x2C, ignored in wheel mode, set to 0)
    // - Speed (2 bytes at 0x2E)
    // This matches the C reference implementation SyncWritePosEx.
    let data_len: u8 = 7;
    let mut offset = 0;
    for cmd in commands {
        if cmd.acceleration > 254 {
            return Err(ProtocolError::InvalidSetting);
        }
        let s = encode_signed_speed(cmd.speed).ok_or(ProtocolError::InvalidSetting)?;
        if offset + 8 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = cmd.id;
        payload[offset + 1] = cmd.acceleration;
        // Position = 0 (2 bytes, ignored in wheel mode)
        payload[offset + 2] = 0;
        payload[offset + 3] = 0;
        // Time = 0 (2 bytes, ignored in wheel mode)
        payload[offset + 4] = 0;
        payload[offset + 5] = 0;
        // Speed (2 bytes, little-endian, signed with bit 15)
        let s = s.to_le_bytes();
        payload[offset + 6] = s[0];
        payload[offset + 7] = s[1];
        offset += 8;
    }
    Ok((data_len, offset))
}

fn fill_sync_torque_mode_payload<E>(
    commands: &[TorqueModeCommand],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 1; // Just the mode byte
    let mut offset = 0;
    for cmd in commands {
        if offset + 2 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = cmd.id;
        payload[offset + 1] = cmd.mode.into();
        offset += 2;
    }
    Ok((data_len, offset))
}

fn fill_sync_torque_limit_payload<E>(
    commands: &[SmsTorqueLimitCommand],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 2; // Torque limit is 2 bytes
    let mut offset = 0;
    for cmd in commands {
        if cmd.limit > crate::MAX_TORQUE_VALUE {
            return Err(ProtocolError::InvalidSetting);
        }
        if offset + 3 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = cmd.id;
        let limit_bytes = cmd.limit.to_le_bytes();
        payload[offset + 1] = limit_bytes[0];
        payload[offset + 2] = limit_bytes[1];
        offset += 3;
    }
    Ok((data_len, offset))
}

fn parse_state_chunk(id: u8, chunk: &[u8]) -> SmsServoState {
    let position_raw = u16::from_le_bytes([chunk[0], chunk[1]]);
    let speed_raw = u16::from_le_bytes([chunk[2], chunk[3]]);
    let load_raw = u16::from_le_bytes([chunk[4], chunk[5]]);
    let voltage_raw = chunk[6];
    let temp_raw = chunk[7];

    SmsServoState {
        id,
        position_raw: decode_signed_position(position_raw),
        speed_raw: decode_speed(speed_raw),
        load_raw: decode_load(load_raw),
        voltage_raw,
        temperature_raw: temp_raw,
    }
}

/// Encode signed position to hardware format (bit 15 = sign) for SMS_STS.
///
/// The hardware protocol uses bit 15 of a 16-bit value as a sign flag. The
/// target-position register accepts magnitudes up to 32767.
///
/// **Hardware encoding:**
/// - Positive: `0x0000` to `0x7FFF` (0 to 32767)
/// - Negative: `0x8001` to `0xFFFF` (-1 to -32767, bit 15 set)
fn encode_signed_position(position: i16) -> Option<u16> {
    let magnitude = position.unsigned_abs();
    if magnitude > BIT_15_VALUE {
        return None;
    }
    if position < 0 {
        Some(BIT_15_SIGN | magnitude)
    } else {
        Some(magnitude)
    }
}

fn encode_offset(offset: i16) -> Option<u16> {
    let magnitude = offset.unsigned_abs();
    if magnitude > 4095 {
        return None;
    }
    if offset >= 0 {
        if magnitude < 2048 {
            Some(magnitude)
        } else {
            Some(0x1000 | (magnitude - 2048))
        }
    } else if magnitude < 2048 {
        Some(0x0800 | magnitude)
    } else {
        Some(0x1800 | (magnitude - 2048))
    }
}

fn decode_offset(raw: u16) -> Option<i16> {
    if raw > 0x1FFF {
        return None;
    }
    let magnitude = raw & 0x07FF;
    #[allow(clippy::cast_possible_wrap)]
    Some(match raw & 0x1800 {
        0x0000 => magnitude as i16,
        0x0800 => -(magnitude as i16),
        0x1000 => (magnitude + 2048) as i16,
        _ => -((magnitude + 2048) as i16),
    })
}

/// Decode signed position from hardware format (bit 15 = sign) for SMS_STS.
///
/// Converts the hardware's bit-15-sign encoding back to a standard signed i16.
#[allow(clippy::cast_possible_wrap)]
fn decode_signed_position(pos_raw: u16) -> i16 {
    if pos_raw & BIT_15_SIGN != 0 {
        -((pos_raw & BIT_15_VALUE) as i16)
    } else {
        pos_raw as i16
    }
}

impl<I> SmsStsBus<I> {
    pub fn new(interface: I) -> Self {
        let uart = UartBusInterface::new(interface);
        let device = SmsStsDevice::new(uart);
        SmsStsBus { device }
    }

    pub fn inner_mut(&mut self) -> &mut SmsStsDevice<UartBusInterface<I>> {
        &mut self.device
    }

    pub fn set_response_status_level(&mut self, enabled: bool) {
        self.device.interface.set_response_status_level(enabled);
    }
}

impl<I> SmsStsBus<I>
where
    I: BlockingRead + BlockingWrite,
{
    pub fn blocking_read_version(
        &mut self,
        id: u8,
    ) -> Result<VersionInformation, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(VersionInformation {
            firmware_major: device.fw_major_version().read()?.version_number(),
            firmware_minor: device.fw_minor_version().read()?.version_number(),
            servo_major: device.servo_major_version().read()?.version_number(),
            servo_minor: device.servo_minor_version().read()?.version_number(),
        })
    }

    pub fn blocking_ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }
        self.device.interface.blocking_ping(id)
    }

    pub fn blocking_reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_reset(id)
    }

    pub fn blocking_set_id(
        &mut self,
        current_id: u8,
        new_id: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        if current_id > crate::MAX_SERVO_ID || new_id > crate::MAX_SERVO_ID {
            return Err(ProtocolError::InvalidId);
        }
        let mut device = BusIdGuard::new(&mut self.device, current_id);
        device.blocking_unlock_eeprom()?;
        if let Err(error) = device.id().write(|w| w.set_id(new_id)) {
            let _ = device.blocking_lock_eeprom();
            return Err(error);
        }
        // The servo starts answering at its new ID immediately after the ID
        // register is written, so the lock command must use that ID too.
        device.interface.set_busid(new_id);
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    pub fn blocking_set_torque_mode(
        &mut self,
        id: u8,
        mode: TorqueMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        // Convert shared TorqueMode to SMS_STS TorqueMode
        let sms_mode = match mode {
            TorqueMode::Disable => SmsTorqueMode::Disable,
            TorqueMode::Enable => SmsTorqueMode::Enable,
            TorqueMode::Free => SmsTorqueMode::Free,
        };
        device.torque_switch().write(|w| w.set_mode(sms_mode))?;
        Ok(())
    }

    pub fn blocking_set_angle_limits(
        &mut self,
        id: u8,
        min: u16,
        max: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if (min != 0 || max != 0)
            && (min >= max || max > crate::sms_sts_constants::SMS_STS_MAX_POSITION_STEPS)
        {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device.minimum_angle().write(|w| w.set_angle(min))?;
            device.maximum_angle().write(|w| w.set_angle(max))?;
            Ok(())
        })
    }

    pub fn blocking_set_voltage_limits(
        &mut self,
        id: u8,
        min_voltage: u8,
        max_voltage: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        if min_voltage > 254 || max_voltage > 254 || min_voltage > max_voltage {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device
                .minimum_input_voltage()
                .write(|w| w.set_voltage(min_voltage))?;
            device
                .maximum_input_voltage()
                .write(|w| w.set_voltage(max_voltage))?;
            Ok(())
        })
    }

    pub fn blocking_set_max_temperature_limit(
        &mut self,
        id: u8,
        max_temp: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        if max_temp > 100 {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device
                .maximum_temperature()
                .write(|w| w.set_temperature(max_temp))?;
            Ok(())
        })
    }

    pub fn blocking_set_max_torque(
        &mut self,
        id: u8,
        max_torque: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if max_torque > crate::MAX_TORQUE_VALUE {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device
                .maximum_torque()
                .write(|w| w.set_torque(max_torque))?;
            Ok(())
        })
    }

    pub fn blocking_set_pid_coefficients(
        &mut self,
        id: u8,
        kp: u8,
        kd: u8,
        ki: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        if kp > 254 || kd > 254 || ki > 254 {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device.p_coefficient().write(|w| w.set_coefficient(kp))?;
            device.d_coefficient().write(|w| w.set_coefficient(kd))?;
            device.i_coefficient().write(|w| w.set_coefficient(ki))?;
            Ok(())
        })
    }

    pub fn blocking_write_position(
        &mut self,
        id: u8,
        position: i16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let position = encode_signed_position(position).ok_or(ProtocolError::InvalidSetting)?;
        let mut device = BusIdGuard::new(&mut self.device, id);
        device
            .target_position()
            .write(|w| w.set_position(position))?;
        Ok(())
    }

    pub fn blocking_read_position(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read()?.position();
        Ok(decode_signed_position(position_raw))
    }

    pub fn blocking_read_speed(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_speed(device.current_speed().read()?.speed()))
    }

    pub fn blocking_read_voltage(&mut self, id: u8) -> Result<u8, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_voltage().read()?.voltage())
    }

    pub fn blocking_read_temperature(&mut self, id: u8) -> Result<u8, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_temperature().read()?.temperature())
    }

    pub fn blocking_read_load(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_load(device.current_load().read()?.load()))
    }

    pub fn blocking_read_current(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_current(device.current_current().read()?.current()))
    }

    pub fn blocking_read_status(&mut self, id: u8) -> Result<ScsStatus, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let status = device.servo_status().read()?;
        Ok(ScsStatus {
            id,
            voltage_error: status.voltage(),
            temperature_error: status.temperature(),
            overload_error: status.overload(),
            magnetic_error: status.magnetic(),
            current_error: status.current(),
        })
    }

    pub fn blocking_is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read()?.flag())
    }

    pub fn blocking_read_mode(&mut self, id: u8) -> Result<ServoMode, ProtocolError<I::Error>> {
        match self.blocking_read_operating_mode(id)? {
            SmsStsOperatingMode::Wheel => Ok(ServoMode::Wheel),
            SmsStsOperatingMode::Position => Ok(ServoMode::Position),
            SmsStsOperatingMode::PwmOpenLoop | SmsStsOperatingMode::Step => {
                Err(ProtocolError::InvalidSetting)
            }
        }
    }

    pub fn blocking_read_operating_mode(
        &mut self,
        id: u8,
    ) -> Result<SmsStsOperatingMode, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        from_register_operating_mode(device.mode().read()?.mode())
            .ok_or(ProtocolError::InvalidSetting)
    }

    pub fn blocking_set_sms_sts_operating_mode(
        &mut self,
        id: u8,
        mode: SmsStsOperatingMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let mode = to_register_operating_mode(mode);
        with_blocking_eeprom(&mut device, |device| {
            device.mode().write(|w| w.set_mode(mode))?;
            Ok(())
        })
    }

    pub fn blocking_set_operating_mode(
        &mut self,
        id: u8,
        mode: ServoMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.blocking_set_sms_sts_operating_mode(
            id,
            match mode {
                ServoMode::Wheel => SmsStsOperatingMode::Wheel,
                ServoMode::Position => SmsStsOperatingMode::Position,
            },
        )
    }

    pub fn blocking_calibrate(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device
            .torque_switch()
            .write(|w| w.set_mode(SmsTorqueMode::Calibration))?;
        Ok(())
    }

    pub fn blocking_read_offset(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let offset_raw = device.offset().read()?.offset();
        decode_offset(offset_raw).ok_or(ProtocolError::InvalidSetting)
    }

    pub fn blocking_set_offset(
        &mut self,
        id: u8,
        offset: i16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let encoded = encode_offset(offset).ok_or(ProtocolError::InvalidSetting)?;
        with_blocking_eeprom(&mut device, |device| {
            device.offset().write(|w| w.set_offset(encoded))?;
            Ok(())
        })
    }

    pub fn blocking_action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_action(id)
    }

    pub fn blocking_reg_write_position(
        &mut self,
        id: u8,
        position: i16,
        _time: u16,
        speed: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let data =
            encode_position_payload(position, 0, speed).ok_or(ProtocolError::InvalidSetting)?;
        self.device
            .interface
            .blocking_reg_write(id, registers::addr::TARGET_POSITION, &data)
    }

    pub fn blocking_sync_write_position<const SIZE: usize>(
        &mut self,
        moves: &[SmsPositionMove; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_payload(moves, &mut payload)?;
        self.device.interface.blocking_sync_write(
            registers::addr::TARGET_POSITION,
            data_len,
            &payload[..offset],
        )
    }

    pub fn blocking_sync_write_position_ex<const SIZE: usize>(
        &mut self,
        moves: &[SmsPositionMoveEx; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_ex_payload(moves, &mut payload)?;
        self.device
            .interface
            .blocking_sync_write(addr::ACCELERATION, data_len, &payload[..offset])
    }

    pub fn blocking_sync_write_speed<const SIZE: usize>(
        &mut self,
        commands: &[SmsSpeedCommand; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_speed_payload(commands, &mut payload)?;
        self.device
            .interface
            .blocking_sync_write(addr::ACCELERATION, data_len, &payload[..offset])
    }

    pub fn blocking_sync_write_torque_mode<const SIZE: usize>(
        &mut self,
        commands: &[TorqueModeCommand; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_torque_mode_payload(commands, &mut payload)?;
        self.device
            .interface
            .blocking_sync_write(addr::TORQUE_SWITCH, data_len, &payload[..offset])
    }

    pub fn blocking_sync_write_torque_limit<const SIZE: usize>(
        &mut self,
        commands: &[SmsTorqueLimitCommand; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_torque_limit_payload(commands, &mut payload)?;
        self.device
            .interface
            .blocking_sync_write(addr::TORQUE_LIMIT, data_len, &payload[..offset])
    }

    pub fn blocking_sync_write_raw<const DATA_LEN: usize, const SIZE: usize>(
        &mut self,
        address: u8,
        commands: &[SyncWriteData<DATA_LEN>; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let mut offset = 0;

        for cmd in commands {
            if offset + 1 + DATA_LEN > payload.len() {
                return Err(ProtocolError::InvalidLength);
            }
            payload[offset] = cmd.id;
            payload[offset + 1..offset + 1 + DATA_LEN].copy_from_slice(&cmd.data);
            offset += 1 + DATA_LEN;
        }

        let data_len = u8::try_from(DATA_LEN).map_err(|_| ProtocolError::InvalidLength)?;
        self.device
            .interface
            .blocking_sync_write(address, data_len, &payload[..offset])
    }

    pub fn blocking_sync_read_state<const SIZE: usize>(
        &mut self,
        ids: &[u8; SIZE],
    ) -> Result<[SmsServoState; SIZE], ProtocolError<I::Error>> {
        let address = registers::addr::CURRENT_POSITION;
        let data_len = 8u8;
        let mut output = [0u8; 256];
        let total_len = ids
            .len()
            .checked_mul(usize::from(data_len))
            .ok_or(ProtocolError::InvalidLength)?;
        if total_len > output.len() {
            return Err(ProtocolError::InvalidLength);
        }

        self.device.interface.blocking_sync_read(
            address,
            data_len,
            ids,
            &mut output[..total_len],
        )?;

        let mut states = [SmsServoState::default(); SIZE];
        for (i, &id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            states[i] = parse_state_chunk(id, &output[start..start + data_len as usize]);
        }
        Ok(states)
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn blocking_read_state(
        &mut self,
        id: u8,
    ) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read()?.position();
        let speed_raw = device.current_speed().read()?.speed();
        let load_raw = device.current_load().read()?.load();
        let voltage_raw = device.current_voltage().read()?.voltage();
        let temp_raw = device.current_temperature().read()?.temperature();
        let moving = device.move_flag().read()?.flag();
        let current = decode_current(device.current_current().read()?.current());

        Ok(ServoTelemetry {
            position_raw: decode_signed_position(position_raw),
            speed_raw: decode_speed(speed_raw),
            load_raw: decode_load(load_raw),
            voltage_raw,
            temperature_raw: temp_raw,
            current_raw: Some(current),
            moving,
        })
    }
}

impl<I> SmsStsBus<I>
where
    I: AsyncRead + AsyncWrite,
{
    pub async fn read_version(
        &mut self,
        id: u8,
    ) -> Result<VersionInformation, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(VersionInformation {
            firmware_major: device
                .fw_major_version()
                .read_async()
                .await?
                .version_number(),
            firmware_minor: device
                .fw_minor_version()
                .read_async()
                .await?
                .version_number(),
            servo_major: device
                .servo_major_version()
                .read_async()
                .await?
                .version_number(),
            servo_minor: device
                .servo_minor_version()
                .read_async()
                .await?
                .version_number(),
        })
    }

    pub async fn ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }
        self.device.interface.ping(id).await
    }

    pub async fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.reset(id).await
    }

    pub async fn write_position(
        &mut self,
        id: u8,
        position: i16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let position = encode_signed_position(position).ok_or(ProtocolError::InvalidSetting)?;
        let mut device = BusIdGuard::new(&mut self.device, id);
        device
            .target_position()
            .write_async(|w| w.set_position(position))
            .await?;
        Ok(())
    }

    pub async fn read_position(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read_async().await?.position();
        Ok(decode_signed_position(position_raw))
    }

    pub async fn is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read_async().await?.flag())
    }

    pub async fn read_operating_mode(
        &mut self,
        id: u8,
    ) -> Result<SmsStsOperatingMode, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        from_register_operating_mode(device.mode().read_async().await?.mode())
            .ok_or(ProtocolError::InvalidSetting)
    }

    pub async fn set_sms_sts_operating_mode(
        &mut self,
        id: u8,
        mode: SmsStsOperatingMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let mode = to_register_operating_mode(mode);
        device.unlock_eeprom().await?;
        let operation_result = device.mode().write_async(|w| w.set_mode(mode)).await;
        let lock_result = device.lock_eeprom().await;
        match (operation_result, lock_result) {
            (Err(operation_error), _) => Err(operation_error),
            (Ok(()), Ok(())) => Ok(()),
            (Ok(()), Err(lock_error)) => Err(lock_error),
        }
    }

    pub async fn set_operating_mode(
        &mut self,
        id: u8,
        mode: ServoMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.set_sms_sts_operating_mode(
            id,
            match mode {
                ServoMode::Wheel => SmsStsOperatingMode::Wheel,
                ServoMode::Position => SmsStsOperatingMode::Position,
            },
        )
        .await
    }

    pub async fn read_offset(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let offset_raw = device.offset().read_async().await?.offset();
        decode_offset(offset_raw).ok_or(ProtocolError::InvalidSetting)
    }

    pub async fn set_offset(&mut self, id: u8, offset: i16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let encoded = encode_offset(offset).ok_or(ProtocolError::InvalidSetting)?;
        device.unlock_eeprom().await?;
        let operation_result = device.offset().write_async(|w| w.set_offset(encoded)).await;
        let lock_result = device.lock_eeprom().await;
        match (operation_result, lock_result) {
            (Err(operation_error), _) => Err(operation_error),
            (Ok(()), Ok(())) => Ok(()),
            (Ok(()), Err(lock_error)) => Err(lock_error),
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub async fn read_state(&mut self, id: u8) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read_async().await?.position();
        let speed_raw = device.current_speed().read_async().await?.speed();
        let load_raw = device.current_load().read_async().await?.load();
        let voltage_raw = device.current_voltage().read_async().await?.voltage();
        let temp_raw = device
            .current_temperature()
            .read_async()
            .await?
            .temperature();
        let moving = device.move_flag().read_async().await?.flag();
        let current = decode_current(device.current_current().read_async().await?.current());

        Ok(ServoTelemetry {
            position_raw: decode_signed_position(position_raw),
            speed_raw: decode_speed(speed_raw),
            load_raw: decode_load(load_raw),
            voltage_raw,
            temperature_raw: temp_raw,
            current_raw: Some(current),
            moving,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockInterface;

    #[test]
    fn test_sms_sts_bus_creation() {
        let _bus = SmsStsBus::new(MockInterface { inner: () });
    }

    #[test]
    fn test_signed_position_encoding() {
        // Test positive values (default range: 0-4095)
        assert_eq!(encode_signed_position(0), Some(0x0000));
        assert_eq!(encode_signed_position(100), Some(100));
        assert_eq!(encode_signed_position(4095), Some(4095));

        // Test negative values (bit 15 should be set for sign)
        // These would be used with offset calibration
        assert_eq!(encode_signed_position(-1), Some(0x8001));
        assert_eq!(encode_signed_position(-100), Some(0x8064));
        assert_eq!(encode_signed_position(-2048), Some(0x8800)); // Max negative for centered offset
        assert_eq!(encode_signed_position(4095), Some(4095));
        assert_eq!(encode_signed_position(-4095), Some(0x8FFF));
        assert_eq!(encode_signed_position(i16::MAX), Some(32767));
        assert_eq!(encode_signed_position(-32767), Some(0xFFFF));
        assert_eq!(encode_signed_position(-32768), None);

        // Test round-trip encoding/decoding
        // The protocol supports signed magnitudes through 32767.
        for val in [-4095, -2048, -1000, -1, 0, 1, 1000, 2047, 4095] {
            let encoded = encode_signed_position(val).unwrap();
            let decoded = decode_signed_position(encoded);
            assert_eq!(decoded, val, "Round-trip failed for {val}");
        }

        // Bit 15 is the sign flag
        assert_eq!(encode_signed_position(-1).unwrap() & 0x8000, 0x8000); // Sign bit set
        assert_eq!(encode_signed_position(1).unwrap() & 0x8000, 0x0000); // Sign bit clear
        assert_eq!(encode_signed_position(i16::MIN), None);
        assert_eq!(encode_signed_speed(i16::MIN), None);
    }

    #[test]
    fn test_position_commands_zero_goal_time() {
        assert_eq!(
            encode_position_payload(123, u16::MAX, 456),
            Some([123, 0, 0, 0, 200, 1])
        );

        let moves = [SmsPositionMove {
            id: 1,
            position: -123,
            time: u16::MAX,
            speed: 456,
        }];
        let mut payload = [0u8; 256];
        let (data_len, payload_len) =
            fill_sync_position_payload::<()>(&moves, &mut payload).unwrap();
        assert_eq!(data_len, 6);
        assert_eq!(&payload[..payload_len], &[1, 123, 128, 0, 0, 200, 1]);

        let moves = [SmsPositionMoveEx {
            id: 2,
            acceleration: 7,
            position: 123,
            time: u16::MAX,
            speed: 456,
        }];
        let mut payload = [0u8; 256];
        let (data_len, payload_len) =
            fill_sync_position_ex_payload::<()>(&moves, &mut payload).unwrap();
        assert_eq!(data_len, 7);
        assert_eq!(&payload[..payload_len], &[2, 7, 123, 0, 0, 0, 200, 1]);
    }

    #[test]
    fn test_offset_encoding_round_trips() {
        for value in [-4095, -2048, -2047, -1, 0, 1, 2047, 2048, 4095] {
            let raw = encode_offset(value).unwrap();
            assert_eq!(
                decode_offset(raw),
                Some(value),
                "Round-trip failed for {value}"
            );
        }
        assert_eq!(encode_offset(-4096), None);
        assert_eq!(encode_offset(4096), None);
        assert_eq!(decode_offset(0x2000), None);
    }

    #[test]
    fn test_operating_mode_mapping() {
        assert_eq!(
            from_register_operating_mode(OperatingMode::Position),
            Some(SmsStsOperatingMode::Position)
        );
        assert_eq!(
            from_register_operating_mode(OperatingMode::Wheel),
            Some(SmsStsOperatingMode::Wheel)
        );
        assert_eq!(
            from_register_operating_mode(OperatingMode::PwmOpenLoop),
            Some(SmsStsOperatingMode::PwmOpenLoop)
        );
        assert_eq!(
            from_register_operating_mode(OperatingMode::Step),
            Some(SmsStsOperatingMode::Step)
        );
        assert_eq!(
            from_register_operating_mode(OperatingMode::Unknown(4)),
            None
        );

        assert_eq!(
            to_register_operating_mode(SmsStsOperatingMode::PwmOpenLoop),
            OperatingMode::PwmOpenLoop
        );
        assert_eq!(
            to_register_operating_mode(SmsStsOperatingMode::Step),
            OperatingMode::Step
        );
    }
}
