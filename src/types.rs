//! Shared data types for SCServo protocol.

/// SCServo protocol instruction types.
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

/// Position move command for sync writes.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsPositionMove {
    pub id: u8,
    pub position: u16,
    pub time: u16,
    pub speed: u16,
}

/// Servo status flags.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsStatus {
    pub voltage_error: bool,
    pub temperature_error: bool,
    pub overload_error: bool,
}

/// Servo state from sync read.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsServoState {
    pub id: u8,
    pub position: u16,
    pub speed: f32,
    pub load: f32,
    pub voltage: f32,
    pub temperature: f32,
}
