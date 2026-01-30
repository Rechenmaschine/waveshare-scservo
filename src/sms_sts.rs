//! SMS/STS series servo bus (STS3215, etc.).
//!
//! Magnetic encoder servos using little-endian wire format.

use crate::error::ProtocolError;
use crate::registers::sts_device::{OperatingMode, SmsStsDevice, TorqueMode as SmsTorqueMode};
use crate::series::{ServoMode, ServoTelemetry};
use crate::types::{
    ScsStatus, SmsPositionMove, SmsPositionMoveEx, SmsServoState, SmsSpeedCommand,
    SmsTorqueLimitCommand, SyncWriteData, TorqueModeCommand,
};
use crate::uart::UartBusInterface;
use crate::{
    registers, TorqueMode, VersionInformation,
};
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

fn encode_position_payload(position: i16, time: u16, speed: u16) -> [u8; 6] {
    let p = encode_signed_position(position).to_le_bytes();
    let t = time.to_le_bytes();
    let s = speed.to_le_bytes();
    [p[0], p[1], t[0], t[1], s[0], s[1]]
}

fn fill_sync_position_payload<E>(
    moves: &[SmsPositionMove],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 6;
    let mut offset = 0;
    for m in moves {
        if offset + 7 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = m.id;
        let p = encode_signed_position(m.position).to_le_bytes();
        let t = m.time.to_le_bytes();
        let s = m.speed.to_le_bytes();
        payload[offset + 1] = p[0];
        payload[offset + 2] = p[1];
        payload[offset + 3] = t[0];
        payload[offset + 4] = t[1];
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
        if offset + 8 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = m.id;
        payload[offset + 1] = m.acceleration;
        let p = encode_signed_position(m.position).to_le_bytes();
        let t = m.time.to_le_bytes();
        let s = m.speed.to_le_bytes();
        payload[offset + 2] = p[0];
        payload[offset + 3] = p[1];
        payload[offset + 4] = t[0];
        payload[offset + 5] = t[1];
        payload[offset + 6] = s[0];
        payload[offset + 7] = s[1];
        offset += 8;
    }
    Ok((data_len, offset))
}

fn encode_signed_speed(speed: i16) -> u16 {
    if speed < 0 {
        BIT_15_SIGN | ((-speed) as u16 & BIT_15_VALUE)
    } else {
        speed as u16
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
        let s = encode_signed_speed(cmd.speed).to_le_bytes();
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
/// The SMS_STS uses a 12-bit magnetic encoder providing 4096 positions (0-4095).
/// The hardware protocol uses bit 15 of a 16-bit value as a sign flag to support
/// offset-adjusted coordinate systems.
///
/// **Hardware encoding:**
/// - Positive: `0x0000` to `0x0FFF` (0 to 4095)
/// - Negative: `0x8001` to `0x8FFF` (-1 to -4095, bit 15 set)
///
/// **Position range depends on offset:**
/// - offset=0: Valid range 0 to 4095 (default)
/// - offset=2048: Valid range -2048 to +2047 (centered)
/// - offset=4095: Valid range -4095 to 0 (inverted)
///
/// The total span is always 4096 positions (12-bit encoder limitation).
fn encode_signed_position(position: i16) -> u16 {
    if position < 0 {
        BIT_15_SIGN | ((-position) as u16 & BIT_15_VALUE)
    } else {
        position as u16
    }
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
    /// Create a new SMS_STS servo bus.
    pub fn new(interface: I) -> Self {
        let uart = UartBusInterface::new(interface);
        let device = SmsStsDevice::new(uart);
        SmsStsBus { device }
    }

    /// Get mutable access to the underlying device.
    pub fn inner_mut(&mut self) -> &mut SmsStsDevice<UartBusInterface<I>> {
        &mut self.device
    }
}

impl<I> SmsStsBus<I>
where
    I: BlockingRead + BlockingWrite,
{
    /// Read version information.
    pub fn blocking_read_version(&mut self, id: u8) -> Result<VersionInformation, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(VersionInformation {
            firmware_major: device.fw_major_version().read()?.version_number(),
            firmware_minor: device.fw_minor_version().read()?.version_number(),
            servo_major: device.servo_major_version().read()?.version_number(),
            servo_minor: device.servo_minor_version().read()?.version_number(),
        })
    }

    /// Ping a servo.
    pub fn blocking_ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }
        self.device.interface.blocking_ping(id)
    }

    /// Reset a servo to factory defaults.
    pub fn blocking_reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_reset(id)
    }

    /// Change a servo's ID.
    pub fn blocking_set_id(&mut self, current_id: u8, new_id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, current_id);
        device.blocking_unlock_eeprom()?;
        device.id().write(|w| w.set_id(new_id))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set torque mode.
    pub fn blocking_set_torque_mode(&mut self, id: u8, mode: TorqueMode) -> Result<(), ProtocolError<I::Error>> {
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

    /// Set angle limits.
    ///
    /// **Units:** `min`, `max` = `steps` (0-4095)
    pub fn blocking_set_angle_limits(&mut self, id: u8, min: u16, max: u16) -> Result<(), ProtocolError<I::Error>> {
        if min > max {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.minimum_angle().write(|w| w.set_angle(min))?;
        device.maximum_angle().write(|w| w.set_angle(max))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set voltage limits.
    ///
    /// **Units:** `min_voltage`, `max_voltage` = `0.1V` units (120 = 12.0V)
    pub fn blocking_set_voltage_limits(&mut self, id: u8, min_voltage: u8, max_voltage: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.minimum_input_voltage().write(|w| w.set_voltage(min_voltage))?;
        device.maximum_input_voltage().write(|w| w.set_voltage(max_voltage))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set maximum temperature limit.
    ///
    /// **Units:** `max_temp` = `°C`
    pub fn blocking_set_max_temperature_limit(&mut self, id: u8, max_temp: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.maximum_temperature().write(|w| w.set_temperature(max_temp))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set maximum torque.
    ///
    /// **Units:** `max_torque` = `0.1%` units (500 = 50.0%, max 1000 = 100%)
    pub fn blocking_set_max_torque(&mut self, id: u8, max_torque: u16) -> Result<(), ProtocolError<I::Error>> {
        if max_torque > crate::MAX_TORQUE_VALUE {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.maximum_torque().write(|w| w.set_torque(max_torque))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set PID coefficients.
    ///
    /// **Units:** `kp`, `kd`, `ki` = unitless (0-254)
    pub fn blocking_set_pid_coefficients(&mut self, id: u8, kp: u8, kd: u8, ki: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.p_coefficient().write(|w| w.set_coefficient(kp))?;
        device.d_coefficient().write(|w| w.set_coefficient(kd))?;
        device.i_coefficient().write(|w| w.set_coefficient(ki))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Write target position.
    ///
    /// Requires torque enabled first.
    ///
    /// **Units:** `position` = `steps` (12-bit encoder: 0-4095)
    ///
    /// **See also:** [`blocking_sync_write_position`](Self::blocking_sync_write_position),
    /// [`blocking_reg_write_position`](Self::blocking_reg_write_position) + [`blocking_action`](Self::blocking_action)
    pub fn blocking_write_position(&mut self, id: u8, position: i16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.target_position().write(|w| w.set_position(encode_signed_position(position)))?;
        Ok(())
    }

    /// Read current position in steps.
    ///
    /// **Units:** Returns `steps` (12-bit encoder: 0-4095)
    pub fn blocking_read_position(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read()?.position();
        Ok(decode_signed_position(position_raw))
    }

    /// Read current speed in steps/s.
    ///
    /// **Units:** Returns `steps/second` (signed, negative = CCW)
    pub fn blocking_read_speed(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_speed(device.current_speed().read()?.speed()))
    }

    /// Read current voltage.
    ///
    /// **Units:** Returns `0.1V` units (120 = 12.0V). Convert with `(voltage as f32) * 0.1`
    pub fn blocking_read_voltage(&mut self, id: u8) -> Result<u8, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_voltage().read()?.voltage())
    }

    /// Read current temperature.
    ///
    /// **Units:** Returns `°C`
    pub fn blocking_read_temperature(&mut self, id: u8) -> Result<u8, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_temperature().read()?.temperature())
    }

    /// Read current load.
    ///
    /// **Units:** Returns `0.1%` units (500 = 50.0%). Convert with `(load as f32) * 0.1`
    pub fn blocking_read_load(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_load(device.current_load().read()?.load()))
    }

    /// Read current draw.
    ///
    /// **Units:** Returns `mA` (likely)
    pub fn blocking_read_current(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_current(device.current_current().read()?.current()))
    }

    /// Read servo status flags.
    pub fn blocking_read_status(&mut self, id: u8) -> Result<ScsStatus, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let status = device.servo_status().read()?;
        Ok(ScsStatus {
            id,
            voltage_error: status.voltage(),
            temperature_error: status.temperature(),
            overload_error: status.overload(),
        })
    }

    /// Check if servo is moving.
    pub fn blocking_is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read()?.flag())
    }

    /// Read servo mode (position or wheel).
    pub fn blocking_read_mode(&mut self, id: u8) -> Result<ServoMode, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        match device.mode().read()?.mode() {
            OperatingMode::Wheel => Ok(ServoMode::Wheel),
            _ => Ok(ServoMode::Position),
        }
    }

    /// Set servo operating mode.
    pub fn blocking_set_operating_mode(&mut self, id: u8, mode: ServoMode) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        match mode {
            ServoMode::Wheel => device.mode().write(|w| w.set_mode(OperatingMode::Wheel))?,
            ServoMode::Position => device.mode().write(|w| w.set_mode(OperatingMode::Position))?,
        }
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Calibrate offset (SMS_STS specific).
    ///
    /// Sets the servo to calibration mode. The current position becomes the new zero point.
    /// Use [`blocking_read_offset`](Self::blocking_read_offset) to read the calculated offset value,
    /// or [`blocking_set_offset`](Self::blocking_set_offset) to manually set an offset.
    pub fn blocking_calibrate(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.torque_switch().write(|w| w.set_mode(SmsTorqueMode::Calibration))?;
        Ok(())
    }

    /// Read position offset.
    ///
    /// The offset shifts the coordinate system. The servo's 12-bit encoder provides 4096
    /// positions (0-4095). With an offset, you can center this range around any point.
    ///
    /// **Examples:**
    /// - offset=0: positions 0 to 4095
    /// - offset=2048: positions -2048 to +2047 (centered)
    /// - offset=4095: positions -4095 to 0
    ///
    /// **Units:** `offset` = `steps` (signed)
    pub fn blocking_read_offset(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let offset_raw = device.offset().read()?.offset();
        Ok(offset_raw as i16)
    }

    /// Set position offset.
    ///
    /// Manually sets the position offset value. This shifts the coordinate system.
    /// The servo's 12-bit encoder provides 4096 positions (0-4095). With an offset,
    /// you can center this range around any point.
    ///
    /// **Examples:**
    /// - offset=0: positions 0 to 4095 (default)
    /// - offset=2048: positions -2048 to +2047 (centered)
    /// - offset=4095: positions -4095 to 0
    ///
    /// Alternatively, use [`blocking_calibrate`](Self::blocking_calibrate) to automatically
    /// calibrate the offset to the current position.
    ///
    /// **Units:** `offset` = `steps` (signed)
    pub fn blocking_set_offset(&mut self, id: u8, offset: i16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.offset().write(|w| w.set_offset(offset as u16))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Trigger registered actions.
    pub fn blocking_action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_action(id)
    }

    /// Queue position command for deferred execution.
    ///
    /// **Units:**
    /// - `position` = `steps` (12-bit encoder: 0-4095)
    /// - `time` = `milliseconds`
    /// - `speed` = `steps/second`
    ///
    /// **See also:** [`blocking_action`](Self::blocking_action)
    pub fn blocking_reg_write_position(&mut self, id: u8, position: i16, time: u16, speed: u16) -> Result<(), ProtocolError<I::Error>> {
        let data = encode_position_payload(position, time, speed);
        self.device.interface.blocking_reg_write(id, registers::addr::TARGET_POSITION, &data)
    }

    /// Move multiple servos simultaneously.
    pub fn blocking_sync_write_position<const SIZE: usize>(&mut self, moves: &[SmsPositionMove; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_payload(moves, &mut payload)?;
        self.device.interface.blocking_sync_write(registers::addr::TARGET_POSITION, data_len, &payload[..offset])
    }

    /// Move multiple servos simultaneously with acceleration control.
    pub fn blocking_sync_write_position_ex<const SIZE: usize>(&mut self, moves: &[SmsPositionMoveEx; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_ex_payload(moves, &mut payload)?;
        self.device.interface.blocking_sync_write(addr::ACCELERATION, data_len, &payload[..offset])
    }

    /// Set speed for multiple servos in wheel mode simultaneously.
    ///
    /// Writes 7 bytes starting at ACCELERATION register (0x29):
    /// - Acceleration (1 byte)
    /// - Position (2 bytes, set to 0, ignored in wheel mode)
    /// - Time (2 bytes, set to 0, ignored in wheel mode)
    /// - Speed (2 bytes, signed with bit 15 = direction)
    ///
    /// This follows the SMS_STS protocol specification where speed control
    /// uses the same register layout as position control (SyncWritePosEx in C reference).
    pub fn blocking_sync_write_speed<const SIZE: usize>(&mut self, commands: &[SmsSpeedCommand; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_speed_payload(commands, &mut payload)?;
        self.device.interface.blocking_sync_write(addr::ACCELERATION, data_len, &payload[..offset])
    }

    /// Set torque mode for multiple servos simultaneously.
    ///
    /// Writes torque mode (Enable/Disable/Free/Calibration) in a single broadcast packet.
    pub fn blocking_sync_write_torque_mode<const SIZE: usize>(&mut self, commands: &[TorqueModeCommand; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_torque_mode_payload(commands, &mut payload)?;
        self.device.interface.blocking_sync_write(addr::TORQUE_SWITCH, data_len, &payload[..offset])
    }

    /// Set runtime torque limit for multiple servos simultaneously.
    ///
    /// **Units:** `limit` = `0.1%` units (500 = 50.0%, max 1000 = 100%)
    pub fn blocking_sync_write_torque_limit<const SIZE: usize>(&mut self, commands: &[SmsTorqueLimitCommand; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_torque_limit_payload(commands, &mut payload)?;
        self.device.interface.blocking_sync_write(addr::TORQUE_LIMIT, data_len, &payload[..offset])
    }

    /// Generic sync write to any register address.
    ///
    /// Writes fixed-size data to the same register address on multiple servos simultaneously.
    /// The `DATA_LEN` const generic specifies how many bytes to write per servo.
    ///
    /// # Example
    /// ```ignore
    /// use waveshare_scservo::SyncWriteData;
    ///
    /// // Write 2 bytes to register 0x30 for servos 1 and 2
    /// let commands = [
    ///     SyncWriteData { id: 1, data: [0x12, 0x34] },  // Little-endian: 0x3412
    ///     SyncWriteData { id: 2, data: [0x56, 0x78] },  // Little-endian: 0x7856
    /// ];
    /// bus.blocking_sync_write(0x30, &commands)?;
    /// ```
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

        self.device.interface.blocking_sync_write(address, DATA_LEN as u8, &payload[..offset])
    }

    /// Sync read state from multiple servos.
    pub fn blocking_sync_read_state<const SIZE: usize>(&mut self, ids: &[u8; SIZE]) -> Result<[SmsServoState; SIZE], ProtocolError<I::Error>> {
        let address = registers::addr::CURRENT_POSITION;
        let data_len = 8u8;
        let mut output = [0u8; 256];
        let total_len = ids.len() * data_len as usize;

        self.device.interface.blocking_sync_read(address, data_len, ids, &mut output[..total_len])?;

        let mut states = [SmsServoState::default(); SIZE];
        for (i, &id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            states[i] = parse_state_chunk(id, &output[start..start + data_len as usize]);
        }
        Ok(states)
    }

    /// Read full telemetry.
    #[allow(clippy::cast_possible_truncation)]
    pub fn blocking_read_state(&mut self, id: u8) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
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
    /// Read version information.
    pub async fn read_version(&mut self, id: u8) -> Result<VersionInformation, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(VersionInformation {
            firmware_major: device.fw_major_version().read_async().await?.version_number(),
            firmware_minor: device.fw_minor_version().read_async().await?.version_number(),
            servo_major: device.servo_major_version().read_async().await?.version_number(),
            servo_minor: device.servo_minor_version().read_async().await?.version_number(),
        })
    }

    /// Ping a servo.
    pub async fn ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        if id == crate::BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }
        self.device.interface.ping(id).await
    }

    /// Reset a servo.
    pub async fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.reset(id).await
    }

    /// Set target position.
    ///
    /// **Units:** `position` = steps (12-bit encoder: 0-4095 by default, or offset-adjusted signed)
    pub async fn write_position(&mut self, id: u8, position: i16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.target_position().write_async(|w| w.set_position(encode_signed_position(position))).await?;
        Ok(())
    }

    /// Get current position.
    ///
    /// **Units:** Returns `steps` (12-bit encoder: 0-4095)
    pub async fn read_position(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read_async().await?.position();
        Ok(decode_signed_position(position_raw))
    }

    /// Check if servo is moving.
    pub async fn is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read_async().await?.flag())
    }

    /// Set servo operating mode.
    pub async fn set_operating_mode(&mut self, id: u8, mode: ServoMode) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.unlock_eeprom().await?;
        match mode {
            ServoMode::Wheel => device.mode().write_async(|w| w.set_mode(OperatingMode::Wheel)).await?,
            ServoMode::Position => device.mode().write_async(|w| w.set_mode(OperatingMode::Position)).await?,
        }
        device.lock_eeprom().await?;
        Ok(())
    }

    /// Read position offset.
    ///
    /// The offset shifts the coordinate system. Valid offsets are constrained by the
    /// 4096-position encoder range (e.g., offset=2048 gives range -2048 to +2047).
    ///
    /// **Units:** `offset` = `steps` (signed)
    pub async fn read_offset(&mut self, id: u8) -> Result<i16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let offset_raw = device.offset().read_async().await?.offset();
        Ok(offset_raw as i16)
    }

    /// Set position offset.
    ///
    /// The offset shifts the coordinate system. Valid offsets are constrained by the
    /// 4096-position encoder range (e.g., offset=2048 gives range -2048 to +2047).
    ///
    /// **Units:** `offset` = `steps` (signed)
    pub async fn set_offset(&mut self, id: u8, offset: i16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.unlock_eeprom().await?;
        device.offset().write_async(|w| w.set_offset(offset as u16)).await?;
        device.lock_eeprom().await?;
        Ok(())
    }

    /// Read full telemetry.
    #[allow(clippy::cast_possible_truncation)]
    pub async fn read_state(&mut self, id: u8) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read_async().await?.position();
        let speed_raw = device.current_speed().read_async().await?.speed();
        let load_raw = device.current_load().read_async().await?.load();
        let voltage_raw = device.current_voltage().read_async().await?.voltage();
        let temp_raw = device.current_temperature().read_async().await?.temperature();
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
        assert_eq!(encode_signed_position(0), 0x0000);
        assert_eq!(encode_signed_position(100), 100);
        assert_eq!(encode_signed_position(4095), 4095);

        // Test negative values (bit 15 should be set for sign)
        // These would be used with offset calibration
        assert_eq!(encode_signed_position(-1), 0x8001);
        assert_eq!(encode_signed_position(-100), 0x8064);
        assert_eq!(encode_signed_position(-2048), 0x8800); // Max negative for centered offset

        // Test round-trip encoding/decoding
        // Note: Valid range depends on offset, but encoding works for full i16 range
        for val in [-4095, -2048, -1000, -1, 0, 1, 1000, 2047, 4095] {
            let encoded = encode_signed_position(val);
            let decoded = decode_signed_position(encoded);
            assert_eq!(decoded, val, "Round-trip failed for {val}");
        }

        // Verify bit 15 is used as sign flag
        assert_eq!(encode_signed_position(-1) & 0x8000, 0x8000); // Sign bit set
        assert_eq!(encode_signed_position(1) & 0x8000, 0x0000); // Sign bit clear
    }
}
