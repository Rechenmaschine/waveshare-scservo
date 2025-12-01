#![no_std]
use crate::registers::{BaudRate, SclInternal, TorqueMode};
use crate::uart::{UartBusInterface, VersionInformation};
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};

mod mock;
mod registers;
mod uart;

/// Resolution of the servo in steps (0-1023)
pub const RESOLUTION_STEPS: u16 = 1024;
/// Maximum effective angle in degrees
pub const MAX_ANGLE_DEGREES: f32 = 220.0;
/// Minimum resolution angle (degrees per step)
pub const DEGREES_PER_STEP: f32 = 0.21484375;
/// No-load speed in steps per second
pub const NO_LOAD_SPEED_STEPS_PER_SEC: u16 = 1500;
/// No-load speed in RPM
pub const NO_LOAD_SPEED_RPM: u16 = 54;

/// Maximum position value (steps)
pub const MAX_POSITION_STEPS: u16 = 1023;
/// Maximum torque value (0.1%)
pub const MAX_TORQUE_VALUE: u16 = 1000;

/// Voltage scaling factor (0.1V per unit)
const VOLTAGE_UNIT: f32 = 0.1;
/// Load/Torque scaling factor (0.1% per unit)
const TORQUE_UNIT: f32 = 0.1;
/// Protection time unit (40ms per unit)
const PROTECTION_TIME_UNIT_MS: u16 = 40;

const BIT_15_SIGN: u16 = 0x8000;
const BIT_15_VALUE: u16 = 0x7FFF;
const BIT_14_SIGN: u16 = 0x4000;
const BIT_14_VALUE: u16 = 0x3FFF;

pub const fn degrees_to_steps(degrees: f32) -> u16 {
    (degrees / DEGREES_PER_STEP) as u16
}

pub const fn steps_to_degrees(steps: u16) -> f32 {
    steps as f32 * DEGREES_PER_STEP
}

#[derive(Debug)]
pub enum ProtocolError<E> {
    Serial(E),
    Checksum,
    Timeout,
    InvalidHeader,
    InvalidId,
    InvalidLength,
    /// The requested setting is invalid (e.g., unsupported baudrate)
    InvalidSetting,
    ServoError(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Instruction {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    RegAction = 0x05,
    Reset = 0x06,
    SyncRead = 0x82,
    SyncWrite = 0x83,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsPositionMove {
    pub id: u8,
    pub position: u16,
    pub time: u16,
    pub speed: u16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsStatus {
    pub voltage_error: bool,
    pub temperature_error: bool,
    pub overload_error: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsServoState {
    pub id: u8,
    pub position: u16,
    pub speed: f32,
    pub load: f32,
    pub voltage: f32,
    pub temperature: f32,
}

pub struct SCLBus<I> {
    device: SclInternal<UartBusInterface<I>>,
}

impl<I> SCLBus<I>
where
    I: BlockingRead + BlockingWrite,
{
    pub fn new(interface: I) -> Self {
        let uart_interface = UartBusInterface::new(interface);
        let device = SclInternal::new(uart_interface);
        SCLBus { device }
    }

    /// Unsafe access to the inner device abstraction. This may break the bus ID handling!
    pub fn inner_mut(&mut self) -> &mut SclInternal<UartBusInterface<I>> {
        &mut self.device
    }

    fn transaction<F, R>(&mut self, id: u8, f: F) -> Result<R, ProtocolError<I::Error>>
    where
        F: FnOnce(&mut SclInternal<UartBusInterface<I>>) -> Result<R, ProtocolError<I::Error>>,
    {
        self.device.interface.set_busid(id);
        let r = f(&mut self.device)?;
        self.device.interface.clear_busid();
        Ok(r)
    }

    pub fn version(&mut self, id: u8) -> Result<VersionInformation, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let servo_major = device.servo_major_version().read()?.version_number();
            let servo_minor = device.servo_minor_version().read()?.version_number();
            let firmware_major = device.fw_major_version().read()?.version_number();
            let firmware_minor = device.fw_minor_version().read()?.version_number();

            Ok(VersionInformation {
                firmware_major_version: firmware_major,
                firmware_minor_version: firmware_minor,
                servo_major_version: servo_major,
                servo_minor_version: servo_minor,
            })
        })
    }

    pub fn set_id(&mut self, current_id: u8, new_id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.transaction(current_id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.id().write(|w| w.set_id(new_id))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_baudrate(
        &mut self,
        id: u8,
        baudrate: BaudRate,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.baudrate().write(|w| w.set_baudrate(baudrate))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn reset(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.reset(id)
    }

    pub fn ping(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.ping(id)
    }

    pub fn set_torque_mode(
        &mut self,
        id: u8,
        mode: TorqueMode,
    ) -> Result<(), ProtocolError<I::Error>> {
        if matches!(mode, TorqueMode::Unknown(_)) {
            return Err(ProtocolError::InvalidSetting);
        }
        self.transaction(id, |device| {
            device.torque_switch().write(|w| w.set_mode(mode))?;
            Ok(())
        })
    }

    pub fn set_angle_limits(
        &mut self,
        id: u8,
        min_angle_steps: u16,
        max_angle_steps: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if min_angle_steps > MAX_POSITION_STEPS || max_angle_steps > MAX_POSITION_STEPS || min_angle_steps > max_angle_steps {
            return Err(ProtocolError::InvalidSetting);
        }
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device
                .minimum_angle()
                .write(|w| w.set_angle(min_angle_steps))?;
            device
                .maximum_angle()
                .write(|w| w.set_angle(max_angle_steps))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_voltage_limits(
        &mut self,
        id: u8,
        min_volts: f32,
        max_volts: f32,
    ) -> Result<(), ProtocolError<I::Error>> {
        let min_val = (min_volts / VOLTAGE_UNIT) as u8;
        let max_val = (max_volts / VOLTAGE_UNIT) as u8;

        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device
                .minimum_input_voltage()
                .write(|w| w.set_voltage(min_val))?;
            device
                .maximum_input_voltage()
                .write(|w| w.set_voltage(max_val))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_max_temperature_limit(
        &mut self,
        id: u8,
        max_temp_celsius: f32,
    ) -> Result<(), ProtocolError<I::Error>> {
        let val = max_temp_celsius as u8;
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device
                .maximum_temperature()
                .write(|w| w.set_temperature(val))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_max_torque(
        &mut self,
        id: u8,
        max_torque_percent: f32,
    ) -> Result<(), ProtocolError<I::Error>> {
        let val = (max_torque_percent / TORQUE_UNIT) as u16;
        if val > MAX_TORQUE_VALUE {
            return Err(ProtocolError::InvalidSetting);
        }
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.maximum_torque().write(|w| w.set_torque(val))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_pid_coefficients(
        &mut self,
        id: u8,
        kp: u8,
        kd: u8,
        ki: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.p_coefficient().write(|w| w.set_coefficient(kp))?;
            device.d_coefficient().write(|w| w.set_coefficient(kd))?;
            device.i_coefficient().write(|w| w.set_coefficient(ki))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_protection_config(
        &mut self,
        id: u8,
        protection_torque_percent: u8,
        protection_time_ms: u16,
        overload_torque_percent: u8,
    ) -> Result<(), ProtocolError<I::Error>> {
        let time_val = (protection_time_ms / PROTECTION_TIME_UNIT_MS).min(254) as u8;
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device
                .protection_torque()
                .write(|w| w.set_torque(protection_torque_percent))?;
            device.protection_time().write(|w| w.set_time(time_val))?;
            device
                .overload_torque()
                .write(|w| w.set_torque(overload_torque_percent))?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_alarm_led(
        &mut self,
        id: u8,
        voltage: bool,
        temperature: bool,
        overload: bool,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.led_alarm_condition().write(|w| {
                w.set_voltage(voltage);
                w.set_temperature(temperature);
                w.set_overload(overload);
            })?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn set_alarm_shutdown(
        &mut self,
        id: u8,
        voltage: bool,
        temperature: bool,
        overload: bool,
    ) -> Result<(), ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            device.lock_flag().write(|w| w.set_locked(false))?;
            device.unloading_conditions().write(|w| {
                w.set_voltage(voltage);
                w.set_temperature(temperature);
                w.set_overload(overload);
            })?;
            device.lock_flag().write(|w| w.set_locked(true))?;
            Ok(())
        })
    }

    pub fn read_status(&mut self, id: u8) -> Result<ScsStatus, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let status = device.servo_status().read()?;
            Ok(ScsStatus {
                voltage_error: status.voltage(),
                temperature_error: status.temperature(),
                overload_error: status.overload(),
            })
        })
    }

    pub fn is_moving(&mut self, id: u8) -> Result<bool, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let moving = device.move_flag().read()?.flag();
            Ok(moving)
        })
    }

    /// Set the target position of the servo.
    ///
    /// # Arguments
    /// * `id` - The ID of the servo.
    /// * `steps` - The target position in steps (0-1023).
    pub fn set_target_position(
        &mut self,
        id: u8,
        steps: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if steps > MAX_POSITION_STEPS {
            return Err(ProtocolError::InvalidSetting);
        }
        self.transaction(id, |device| {
            device.target_position().write(|w| w.set_position(steps))?;
            Ok(())
        })
    }

    /// Get the current position of the servo.
    ///
    /// Returns the position in steps (0-1023).
    pub fn current_position_steps(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let pos = device.current_position().read()?.position();
            Ok(pos)
        })
    }

    /// Get the current speed of the servo.
    ///
    /// Returns the speed in steps/s.
    /// Positive values indicate forward rotation, negative values indicate reverse rotation.
    pub fn current_speed(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let speed_raw = device.current_speed().read()?.speed();
            let speed = if speed_raw & BIT_15_SIGN != 0 {
                -1.0 * (speed_raw & BIT_15_VALUE) as f32
            } else {
                speed_raw as f32
            };
            Ok(speed)
        })
    }

    /// Get the current input voltage of the servo.
    ///
    /// Returns the voltage in Volts.
    pub fn current_voltage(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let voltage = device.current_voltage().read()?.voltage();
            Ok(voltage as f32 * VOLTAGE_UNIT)
        })
    }

    /// Get the current temperature of the servo.
    ///
    /// Returns the temperature in degrees Celsius.
    pub fn current_temperature(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let temp = device.current_temperature().read()?.temperature();
            Ok(temp as f32)
        })
    }

    /// Get the current load of the servo.
    ///
    /// Returns the load as a percentage of maximum torque (0.0 - 100.0).
    /// Positive values indicate forward load, negative values indicate reverse load.
    pub fn current_load(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let load_raw = device.current_load().read()?.load();

            let load = if load_raw & BIT_14_SIGN != 0 {
                // Negative load
                -1.0 * ((load_raw & BIT_14_VALUE) as f32) * TORQUE_UNIT
            } else {
                // Positive load
                (load_raw as f32) * TORQUE_UNIT
            };
            Ok(load)
        })
    }

    /// Trigger the action for registered instructions.
    ///
    /// This command is used to execute instructions that were sent with `reg_write`.
    pub fn action(&mut self, id: u8) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.action(id)
    }

    /// Write to a register asynchronously.
    ///
    /// The instruction is registered but not executed until an `action` command is received.
    pub fn reg_write_raw(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.reg_write(id, address, data)
    }

    /// Set the target position, time, and speed asynchronously.
    ///
    /// The instruction is registered but not executed until an `action` command is received.
    ///
    /// # Arguments
    /// * `id` - The ID of the servo.
    /// * `position` - Target position in steps.
    /// * `time` - Movement time in ms (0 means use speed).
    /// * `speed` - Movement speed in steps/s (0 means use time).
    pub fn reg_write_position(
        &mut self,
        id: u8,
        position: u16,
        time: u16,
        speed: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut data = [0u8; 6];
        let p = position.to_le_bytes();
        let t = time.to_le_bytes();
        let s = speed.to_le_bytes();
        data[0] = p[0];
        data[1] = p[1];
        data[2] = t[0];
        data[3] = t[1];
        data[4] = s[0];
        data[5] = s[1];

        self.device.interface.reg_write(id, crate::registers::TARGET_POSITION_ADDR, &data)
    }

    /// Write to multiple servos simultaneously.
    ///
    /// # Arguments
    /// * `address` - The starting register address.
    /// * `data_len` - The length of data per servo.
    /// * `payload` - The concatenated data for all servos (ID + Data).
    pub fn sync_write_raw(
        &mut self,
        address: u8,
        data_len: u8,
        payload: &[u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        self.device.interface.sync_write(address, data_len, payload)
    }

    /// Move multiple servos simultaneously.
    ///
    /// # Arguments
    /// * `moves` - A slice of `ScsPositionMove` structs defining the movement for each servo.
    pub fn sync_write_position(
        &mut self,
        moves: &[ScsPositionMove],
    ) -> Result<(), ProtocolError<I::Error>> {
        let mut payload = [0u8; 256];
        let data_len = 6;
        let mut offset = 0;

        for m in moves {
            if offset + 7 > payload.len() {
                return Err(ProtocolError::InvalidLength);
            }
            payload[offset] = m.id;
            let p = m.position.to_le_bytes();
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

        self.device
            .interface
            .sync_write(crate::registers::TARGET_POSITION_ADDR, data_len, &payload[..offset])
    }

    /// Read from multiple servos simultaneously.
    ///
    /// # Arguments
    /// * `address` - The starting register address.
    /// * `data_len` - The length of data to read per servo.
    /// * `ids` - The IDs of the servos to read from.
    /// * `output` - Buffer to store the read data.
    pub fn sync_read_raw(
        &mut self,
        address: u8,
        data_len: u8,
        ids: &[u8],
        output: &mut [u8],
    ) -> Result<(), ProtocolError<I::Error>> {
        self.device
            .interface
            .sync_read(address, data_len, ids, output)
    }

    /// Read the state of multiple servos simultaneously.
    ///
    /// Reads position, speed, load, voltage, and temperature.
    ///
    /// # Arguments
    /// * `ids` - The IDs of the servos to read from.
    /// * `states` - Buffer to store the parsed state for each servo. Must be at least as long as `ids`.
    pub fn sync_read_state(
        &mut self,
        ids: &[u8],
        states: &mut [ScsServoState],
    ) -> Result<(), ProtocolError<I::Error>> {
        if states.len() < ids.len() {
            return Err(ProtocolError::InvalidLength);
        }

        let address = crate::registers::CURRENT_POSITION_ADDR;
        let data_len = 8;
        let mut output = [0u8; 256];
        let total_len = ids.len() * data_len as usize;
        if total_len > output.len() {
            return Err(ProtocolError::InvalidLength);
        }

        self.device
            .interface
            .sync_read(address, data_len, ids, &mut output[..total_len])?;

        for (i, &id) in ids.iter().enumerate() {
            let start = i * data_len as usize;
            let chunk = &output[start..start + data_len as usize];

            let position = u16::from_le_bytes([chunk[0], chunk[1]]);
            let speed_raw = u16::from_le_bytes([chunk[2], chunk[3]]);
            let load_raw = u16::from_le_bytes([chunk[4], chunk[5]]);
            let voltage_raw = chunk[6];
            let temp_raw = chunk[7];

            let speed = if speed_raw & BIT_15_SIGN != 0 {
                -1.0 * (speed_raw & BIT_15_VALUE) as f32
            } else {
                speed_raw as f32
            };

            let load = if load_raw & BIT_14_SIGN != 0 {
                -1.0 * ((load_raw & BIT_14_VALUE) as f32) * TORQUE_UNIT
            } else {
                (load_raw as f32) * TORQUE_UNIT
            };

            states[i] = ScsServoState {
                id,
                position,
                speed,
                load,
                voltage: voltage_raw as f32 * VOLTAGE_UNIT,
                temperature: temp_raw as f32,
            };
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockInterface;

    #[test]
    fn test_device_creation() {
        let mut bus = SCLBus::new(MockInterface { inner: () });
        const ID: u8 = 1;

        bus.set_angle_limits(ID, degrees_to_steps(0.0), degrees_to_steps(180.0))
            .unwrap();

        bus.set_target_position(ID, degrees_to_steps(200.0)).unwrap();
        bus.set_torque_mode(ID, TorqueMode::Enable).unwrap();

        const NEW_ID: u8 = 2;
        bus.set_id(ID, NEW_ID).unwrap();

        bus.inner_mut().read_all_registers(|a,b,c| {
            
        }).expect("TODO: panic message");


    }
}
