//! SCSCL series servo bus (SC09, SC15, etc.).
//!
//! Potentiometer-based servos using big-endian wire format.

use crate::error::ProtocolError;
use crate::registers::sc_device::ScsclDevice;
use crate::series::{ServoMode, ServoTelemetry};
use crate::types::{
    ScsStatus, ScsclMotorCommand, ScsclPositionMove, ScsclServoState, SyncWriteData,
    TorqueModeCommand,
};
use crate::uart::UartBusInterface;
use crate::{TorqueMode, VersionInformation, registers, scscl_constants};
use crate::{decode_current, decode_load, decode_speed, encode_signed_pwm};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

/// SCSCL-specific register addresses
mod addr {
    /// Torque switch register address (0x28)
    pub const TORQUE_SWITCH: u8 = 0x28;

    /// Goal time register address (0x2C)
    ///
    /// This register has dual purposes depending on the servo's operating mode:
    /// - **Position mode:** Movement duration in milliseconds (how long to take reaching target position)
    /// - **Wheel mode:** Signed PWM motor output (bit 10 = sign, range typically -1000 to 1000)
    ///
    /// This is standard behavior for SCSCL servos (see SCSCL.h: SCSCL_GOAL_TIME_L = 44 = 0x2C).
    /// The servo firmware interprets this register differently based on whether angle limits
    /// are set (position mode) or cleared to 0 (wheel/PWM mode).
    pub const GOAL_TIME: u8 = 0x2C;
}

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

fn with_blocking_eeprom<I, T, F>(
    device: &mut ScsclDevice<UartBusInterface<I>>,
    operation: F,
) -> Result<T, ProtocolError<I::Error>>
where
    I: BlockingRead + BlockingWrite,
    F: FnOnce(&mut ScsclDevice<UartBusInterface<I>>) -> Result<T, ProtocolError<I::Error>>,
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

fn encode_position_payload(position: u16, time: u16, speed: u16) -> [u8; 6] {
    let p = position.to_be_bytes();
    let t = time.to_be_bytes();
    let s = speed.to_be_bytes();
    [p[0], p[1], t[0], t[1], s[0], s[1]]
}

fn fill_sync_position_payload<E>(
    moves: &[ScsclPositionMove],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 6;
    let mut offset = 0;
    for m in moves {
        if m.position > scscl_constants::SCSCL_MAX_POSITION_STEPS {
            return Err(ProtocolError::InvalidSetting);
        }
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

fn fill_sync_motor_payload<E>(
    commands: &[ScsclMotorCommand],
    payload: &mut [u8; 256],
) -> Result<(u8, usize), ProtocolError<E>> {
    let data_len: u8 = 2; // Motor output is 2 bytes (signed PWM with bit 10 as sign)
    let mut offset = 0;
    for cmd in commands {
        if offset + 3 > payload.len() {
            return Err(ProtocolError::InvalidLength);
        }
        payload[offset] = cmd.id;
        let encoded = encode_signed_pwm(cmd.output)
            .ok_or(ProtocolError::InvalidSetting)?
            .to_be_bytes();
        payload[offset + 1] = encoded[0];
        payload[offset + 2] = encoded[1];
        offset += 3;
    }
    Ok((data_len, offset))
}

fn parse_state_chunk(id: u8, chunk: &[u8]) -> ScsclServoState {
    let position = u16::from_be_bytes([chunk[0], chunk[1]]);
    let speed_raw = u16::from_be_bytes([chunk[2], chunk[3]]);
    let load_raw = u16::from_be_bytes([chunk[4], chunk[5]]);
    let voltage_raw = chunk[6];
    let temp_raw = chunk[7];

    ScsclServoState {
        id,
        position_raw: position,
        speed_raw: decode_speed(speed_raw),
        load_raw: decode_load(load_raw),
        voltage_raw,
        temperature_raw: temp_raw,
    }
}

impl<I> ScsclBus<I> {
    pub fn new(interface: I) -> Self {
        let uart = UartBusInterface::new(interface);
        let device = ScsclDevice::new(uart);
        ScsclBus { device }
    }

    pub fn inner_mut(&mut self) -> &mut ScsclDevice<UartBusInterface<I>> {
        &mut self.device
    }

    pub fn set_response_status_level(&mut self, enabled: bool) {
        self.device.interface.set_response_status_level(enabled);
    }
}

impl<I> ScsclBus<I>
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
        device.torque_switch().write(|w| w.set_mode(mode))?;
        Ok(())
    }

    pub fn blocking_set_angle_limits(
        &mut self,
        id: u8,
        min: u16,
        max: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if (min != 0 || max != 0) && (min >= max || max > scscl_constants::SCSCL_MAX_POSITION_STEPS)
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
        if kp > 254 || kd > 254 || ki != 0 {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        with_blocking_eeprom(&mut device, |device| {
            device.p_coefficient().write(|w| w.set_coefficient(kp))?;
            device.d_coefficient().write(|w| w.set_coefficient(kd))?;
            Ok(())
        })
    }

    pub fn blocking_write_position(
        &mut self,
        id: u8,
        steps: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if steps > scscl_constants::SCSCL_MAX_POSITION_STEPS {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.target_position().write(|w| w.set_position(steps))?;
        Ok(())
    }

    pub fn blocking_read_position(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_position().read()?.position())
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
            magnetic_error: false,
            current_error: false,
        })
    }

    pub fn blocking_is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read()?.flag())
    }

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

    pub fn blocking_set_operating_mode(
        &mut self,
        id: u8,
        mode: ServoMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        match mode {
            ServoMode::Wheel => self.blocking_set_angle_limits(id, 0, 0),
            ServoMode::Position => {
                self.blocking_set_angle_limits(id, 0, scscl_constants::SCSCL_MAX_POSITION_STEPS)
            }
        }
    }

    pub fn blocking_write_motor(
        &mut self,
        id: u8,
        output: i16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let encoded = encode_signed_pwm(output).ok_or(ProtocolError::InvalidSetting)?;
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.goal_time().write(|w| w.set_time(encoded))?;
        Ok(())
    }

    pub fn blocking_action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.blocking_action(id)
    }

    pub fn blocking_reg_write_position(
        &mut self,
        id: u8,
        position: u16,
        time: u16,
        speed: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if position > scscl_constants::SCSCL_MAX_POSITION_STEPS {
            return Err(ProtocolError::InvalidSetting);
        }
        let data = encode_position_payload(position, time, speed);
        self.device
            .interface
            .blocking_reg_write(id, registers::addr::TARGET_POSITION, &data)
    }

    pub fn blocking_sync_write_position<const SIZE: usize>(
        &mut self,
        moves: &[ScsclPositionMove; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_position_payload(moves, &mut payload)?;
        self.device.interface.blocking_sync_write(
            registers::addr::TARGET_POSITION,
            data_len,
            &payload[..offset],
        )
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

    pub fn blocking_sync_write_motor<const SIZE: usize>(
        &mut self,
        commands: &[ScsclMotorCommand; SIZE],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let (data_len, offset) = fill_sync_motor_payload(commands, &mut payload)?;
        self.device
            .interface
            .blocking_sync_write(addr::GOAL_TIME, data_len, &payload[..offset])
    }

    pub fn blocking_sync_write<const DATA_LEN: usize, const SIZE: usize>(
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
    ) -> Result<[ScsclServoState; SIZE], ProtocolError<I::Error>> {
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

        let mut states = [ScsclServoState::default(); SIZE];
        for (i, &id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            states[i] = parse_state_chunk(id, &output[start..start + data_len as usize]);
        }
        Ok(states)
    }

    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
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
        Ok(ServoTelemetry {
            position_raw: position_raw as i16,
            speed_raw: decode_speed(speed_raw),
            load_raw: decode_load(load_raw),
            voltage_raw,
            temperature_raw: temp_raw,
            current_raw: None,
            moving,
        })
    }
}

impl<I> ScsclBus<I>
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
        steps: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if steps > scscl_constants::SCSCL_MAX_POSITION_STEPS {
            return Err(ProtocolError::InvalidSetting);
        }
        let mut device = BusIdGuard::new(&mut self.device, id);
        device
            .target_position()
            .write_async(|w| w.set_position(steps))
            .await?;
        Ok(())
    }

    pub async fn read_position(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.current_position().read_async().await?.position())
    }

    pub async fn is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        Ok(device.move_flag().read_async().await?.flag())
    }

    pub async fn set_operating_mode(
        &mut self,
        id: u8,
        mode: ServoMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut device = BusIdGuard::new(&mut self.device, id);
        device.unlock_eeprom().await?;
        let operation_result = match mode {
            ServoMode::Wheel => {
                async {
                    device
                        .minimum_angle()
                        .write_async(|w| w.set_angle(0))
                        .await?;
                    device.maximum_angle().write_async(|w| w.set_angle(0)).await
                }
                .await
            }
            ServoMode::Position => {
                async {
                    device
                        .minimum_angle()
                        .write_async(|w| w.set_angle(0))
                        .await?;
                    device
                        .maximum_angle()
                        .write_async(|w| w.set_angle(scscl_constants::SCSCL_MAX_POSITION_STEPS))
                        .await
                }
                .await
            }
        };
        let lock_result = device.lock_eeprom().await;
        match (operation_result, lock_result) {
            (Err(operation_error), _) => Err(operation_error),
            (Ok(()), Ok(())) => Ok(()),
            (Ok(()), Err(lock_error)) => Err(lock_error),
        }
    }

    pub async fn write_motor(
        &mut self,
        id: u8,
        output: i16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let encoded = encode_signed_pwm(output).ok_or(ProtocolError::InvalidSetting)?;
        let mut device = BusIdGuard::new(&mut self.device, id);
        device
            .goal_time()
            .write_async(|w| w.set_time(encoded))
            .await?;
        Ok(())
    }

    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
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
        Ok(ServoTelemetry {
            position_raw: position_raw as i16,
            speed_raw: decode_speed(speed_raw),
            load_raw: decode_load(load_raw),
            voltage_raw,
            temperature_raw: temp_raw,
            current_raw: None,
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
