use crate::registers::{BaudRate, SclInternal, TorqueMode};
use crate::uart::{UartBusInterface, VersionInformation};
use device_driver::RegisterInterface;
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};

mod mock;
mod registers;
mod uart;

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

    pub fn configure_torque(
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

    pub fn set_target_position(
        &mut self,
        id: u8,
        steps: u16,
    ) -> Result<(), ProtocolError<I::Error>> {
        if steps > 1023 {
            return Err(ProtocolError::InvalidSetting);
        }
        self.transaction(id, |device| {
            device
                .target_position()
                .write(|w| w.set_position(steps))?;
            Ok(())
        })
    }

    pub fn current_position_steps(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let pos = device.current_position().read()?.position();
            Ok(pos)
        })
    }

    pub fn current_speed(&mut self, id: u8) -> Result<u16, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let speed = device.current_speed().read()?.speed();
            Ok(speed)
        })
    }

    pub fn current_voltage(&mut self, id: u8) -> Result<u8, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let voltage = device.current_voltage().read()?.voltage();
            Ok(voltage)
        })
    }

    pub fn current_load(&mut self, id: u8) -> Result<f32, ProtocolError<I::Error>> {
        self.transaction(id, |device| {
            let load_raw = device.current_load().read()?.load();

            let load = if load_raw & 0x4000 != 0 {
                // Negative load
                -1.0 * ((load_raw & 0x3FFF) as f32) * 0.1
            } else {
                // Positive load
                (load_raw as f32) * 0.1
            };
            Ok(load)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockInterface;

    #[test]
    fn test_device_creation() {
        let mut bus = SCLBus::new(MockInterface { inner: () });
        // Example usage


        let x = bus.current_voltage(12).expect("ah");

    }
}
