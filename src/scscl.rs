//! SCSCL series servo bus (SC09, SC15, etc.).
//!
//! Potentiometer-based servos using big-endian wire format.

use crate::error::ProtocolError;
use crate::registers::sc_device::ScsclDevice;
use crate::series::{ServoMode, ServoTelemetry};
use crate::types::{ScsPositionMove, ScsServoState, ScsStatus};
use crate::uart::UartBusInterface;
use crate::{
    registers, Instruction, TorqueMode, VersionInformation, BROADCAST_ID, MAX_TORQUE_VALUE,
    TORQUE_UNIT, VOLTAGE_UNIT,
};
use crate::{decode_current, decode_load, decode_speed, encode_signed_pwm};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

/// Servo bus for SCSCL series (SC09, SC15, etc.).
///
/// Uses big-endian wire format.
pub struct ScsclBus<I> {
    device: ScsclDevice<UartBusInterface<I>>,
}

struct BusIdGuard<'a, I> {
    device: &'a mut ScsclDevice<UartBusInterface<I>>,
}

impl<'a, I> BusIdGuard<'a, I> {
    fn new(device: &'a mut ScsclDevice<UartBusInterface<I>>, id: u8) -> Self {
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
    type Target = ScsclDevice<UartBusInterface<I>>;
    fn deref(&self) -> &Self::Target {
        self.device
    }
}

impl<I> core::ops::DerefMut for BusIdGuard<'_, I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.device
    }
}

fn encode_position_payload(position: u16, time: u16, speed: u16) -> [u8; 6] {
    let p = position.to_be_bytes();
    let t = time.to_be_bytes();
    let s = speed.to_be_bytes();
    [p[0], p[1], t[0], t[1], s[0], s[1]]
}

fn fill_sync_position_payload<E>(
    moves: &[ScsPositionMove],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 6;
    let mut offset = 0;
    for m in moves {
        if offset + 7 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = m.id;
        let p = m.position.to_be_bytes();
        let t = m.time.to_be_bytes();
        let s = m.speed.to_be_bytes();
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

fn parse_state_chunk(id: u8, chunk: &[u8]) -> ScsServoState {
    let position = u16::from_be_bytes([chunk[0], chunk[1]]);
    let speed_raw = u16::from_be_bytes([chunk[2], chunk[3]]);
    let load_raw = u16::from_be_bytes([chunk[4], chunk[5]]);
    let voltage_raw = chunk[6];
    let temp_raw = chunk[7];

    ScsServoState {
        id,
        position,
        speed: decode_speed(speed_raw),
        load: decode_load(load_raw),
        voltage: f32::from(voltage_raw) * VOLTAGE_UNIT,
        temperature: f32::from(temp_raw),
    }
}

impl<I> ScsclBus<I> {
    /// Create a new SCSCL servo bus.
    pub fn new(interface: I) -> Self {
        let uart = UartBusInterface::new(interface);
        let device = ScsclDevice::new(uart);
        ScsclBus { device }
    }

    /// Get mutable access to the underlying device.
    pub fn inner_mut(&mut self) -> &mut ScsclDevice<UartBusInterface<I>> {
        &mut self.device
    }
}

impl<I> ScsclBus<I>
where
    I: BlockingRead + BlockingWrite,
{
    /// Read version information.
    pub fn blocking_version(&mut self, id: u8) -> Result<VersionInformation, ProtocolError<I::Error>> {
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
        if id == BROADCAST_ID {
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
        self.device.interface.blocking_transfer(id, Instruction::Write, &[0x28, mode as u8], &mut [])?;
        Ok(())
    }

    /// Set angle limits.
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
    pub fn blocking_set_voltage_limits(&mut self, id: u8, min_volts: f32, max_volts: f32) -> Result<(), ProtocolError<I::Error>> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let min_val = (min_volts / VOLTAGE_UNIT) as u8;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let max_val = (max_volts / VOLTAGE_UNIT) as u8;
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.minimum_input_voltage().write(|w| w.set_voltage(min_val))?;
        device.maximum_input_voltage().write(|w| w.set_voltage(max_val))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set maximum temperature limit.
    pub fn blocking_set_max_temperature_limit(&mut self, id: u8, max_temp: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.maximum_temperature().write(|w| w.set_temperature(max_temp))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set maximum torque.
    pub fn blocking_set_max_torque(&mut self, id: u8, max_torque_percent: f32) -> Result<(), ProtocolError<I::Error>> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let val = (max_torque_percent / TORQUE_UNIT) as u16;
        if val > MAX_TORQUE_VALUE {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.maximum_torque().write(|w| w.set_torque(val))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set PID coefficients.
    pub fn blocking_set_pid_coefficients(&mut self, id: u8, kp: u8, kd: u8, ki: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.blocking_unlock_eeprom()?;
        device.p_coefficient().write(|w| w.set_coefficient(kp))?;
        device.d_coefficient().write(|w| w.set_coefficient(kd))?;
        device.i_coefficient().write(|w| w.set_coefficient(ki))?;
        device.blocking_lock_eeprom()?;
        Ok(())
    }

    /// Set target position.
    pub fn blocking_set_target_position(&mut self, id: u8, steps: u16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.target_position().write(|w| w.set_position(steps))?;
        Ok(())
    }

    /// Get current position.
    pub fn blocking_current_position_steps(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_position().read()?.position())
    }

    /// Get current speed.
    pub fn blocking_current_speed(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_speed(device.current_speed().read()?.speed()))
    }

    /// Get current voltage.
    pub fn blocking_current_voltage(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(f32::from(device.current_voltage().read()?.voltage()) * VOLTAGE_UNIT)
    }

    /// Get current temperature.
    pub fn blocking_current_temperature(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(f32::from(device.current_temperature().read()?.temperature()))
    }

    /// Get current load.
    pub fn blocking_current_load(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_load(device.current_load().read()?.load()))
    }

    /// Read current draw in mA.
    pub fn blocking_read_current(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(decode_current(device.current_current().read()?.current()))
    }

    /// Read servo status flags.
    pub fn blocking_read_status(&mut self, id: u8) -> Result<ScsStatus, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let status = device.servo_status().read()?;
        Ok(ScsStatus {
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

    /// Read servo mode (position or wheel/PWM).
    ///
    /// SCSCL uses angle limits = 0 to indicate wheel/PWM mode.
    pub fn blocking_read_mode(&mut self, id: u8) -> Result<ServoMode, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let min = device.minimum_angle().read()?.angle();
        let max = device.maximum_angle().read()?.angle();
        if min == 0 && max == 0 {
            Ok(ServoMode::Wheel)
        } else {
            Ok(ServoMode::Position)
        }
    }

    /// Enable PWM mode (sets angle limits to 0).
    pub fn blocking_pwm_mode(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.blocking_set_angle_limits(id, 0, 0)
    }

    /// Write PWM value (SCSCL-specific).
    ///
    /// Positive = CW, negative = CCW.
    pub fn blocking_write_pwm(&mut self, id: u8, pwm: i16) -> Result<(), ProtocolError<I::Error>> {
        let encoded = encode_signed_pwm(pwm);
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.operation_time().write(|w| w.set_time(encoded))?;
        Ok(())
    }

    /// Trigger registered actions.
    pub fn blocking_action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_action(id)
    }

    /// Set target position with time/speed (deferred execution).
    pub fn blocking_reg_write_position(&mut self, id: u8, position: u16, time: u16, speed: u16) -> Result<(), ProtocolError<I::Error>> {
        let data = encode_position_payload(position, time, speed);
        self.device.interface.blocking_reg_write(id, registers::addr::TARGET_POSITION, &data)
    }

    /// Move multiple servos simultaneously.
    pub fn blocking_sync_write_position<const SIZE: usize>(&mut self, moves: &[ScsPositionMove; SIZE]) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_payload(moves, &mut payload)?;
        self.device.interface.blocking_sync_write(registers::addr::TARGET_POSITION, data_len, &payload[..offset])
    }

    /// Sync read state from multiple servos.
    pub fn blocking_sync_read_state<const SIZE: usize>(&mut self, ids: &[u8; SIZE]) -> Result<[ScsServoState; SIZE], ProtocolError<I::Error>> {
        let address = registers::addr::CURRENT_POSITION;
        let data_len = 8u8;
        let mut output = [0u8; 256];
        let total_len = ids.len() * data_len as usize;

        self.device.interface.blocking_sync_read(address, data_len, ids, &mut output[..total_len])?;

        let mut states = [ScsServoState::default(); SIZE];
        for (i, &id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            states[i] = parse_state_chunk(id, &output[start..start + data_len as usize]);
        }
        Ok(states)
    }

    /// Read full telemetry.
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    pub fn blocking_feedback(&mut self, id: u8) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read()?.position();
        let speed_raw = device.current_speed().read()?.speed();
        let load_raw = device.current_load().read()?.load();
        let voltage_raw = device.current_voltage().read()?.voltage();
        let temp_raw = device.current_temperature().read()?.temperature();
        let moving = device.move_flag().read()?.flag();
        let current = device.current_current().read().map(|c| decode_current(c.current())).ok();

        Ok(ServoTelemetry {
            position: position_raw as i16,
            speed: decode_speed(speed_raw) as i16,
            load: decode_load(load_raw),
            voltage: f32::from(voltage_raw) * VOLTAGE_UNIT,
            temperature: f32::from(temp_raw),
            current,
            moving,
        })
    }
}

impl<I> ScsclBus<I>
where
    I: AsyncRead + AsyncWrite,
{
    /// Read version information.
    pub async fn version(&mut self, id: u8) -> Result<VersionInformation, ProtocolError<I::Error>> {
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
        if id == BROADCAST_ID {
            return Err(ProtocolError::InvalidId);
        }
        self.device.interface.ping(id).await
    }

    /// Reset a servo.
    pub async fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.reset(id).await
    }

    /// Set target position.
    pub async fn set_target_position(&mut self, id: u8, steps: u16) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.target_position().write_async(|w| w.set_position(steps)).await?;
        Ok(())
    }

    /// Get current position.
    pub async fn current_position_steps(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_position().read_async().await?.position())
    }

    /// Enable PWM mode.
    pub async fn pwm_mode(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.unlock_eeprom().await?;
        device.minimum_angle().write_async(|w| w.set_angle(0)).await?;
        device.maximum_angle().write_async(|w| w.set_angle(0)).await?;
        device.lock_eeprom().await?;
        Ok(())
    }

    /// Write PWM value.
    pub async fn write_pwm(&mut self, id: u8, pwm: i16) -> Result<(), ProtocolError<I::Error>> {
        let encoded = encode_signed_pwm(pwm);
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.operation_time().write_async(|w| w.set_time(encoded)).await?;
        Ok(())
    }

    /// Read full telemetry.
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    pub async fn feedback(&mut self, id: u8) -> Result<ServoTelemetry, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        let position_raw = device.current_position().read_async().await?.position();
        let speed_raw = device.current_speed().read_async().await?.speed();
        let load_raw = device.current_load().read_async().await?.load();
        let voltage_raw = device.current_voltage().read_async().await?.voltage();
        let temp_raw = device.current_temperature().read_async().await?.temperature();
        let moving = device.move_flag().read_async().await?.flag();
        let current = match device.current_current().read_async().await {
            Ok(c) => Some(decode_current(c.current())),
            Err(_) => None,
        };

        Ok(ServoTelemetry {
            position: position_raw as i16,
            speed: decode_speed(speed_raw) as i16,
            load: decode_load(load_raw),
            voltage: f32::from(voltage_raw) * VOLTAGE_UNIT,
            temperature: f32::from(temp_raw),
            current,
            moving,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockInterface;

    #[test]
    fn test_scscl_bus_creation() {
        let _bus = ScsclBus::new(MockInterface { inner: () });
    }
}
