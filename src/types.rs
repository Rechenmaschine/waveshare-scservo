/// SCServo protocol instruction types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub(crate) enum Instruction {
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
///
/// Used with [`blocking_sync_write_position`](crate::ScsclBus::blocking_sync_write_position)
/// to move multiple servos simultaneously.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsPositionMove {
    /// Servo ID.
    pub id: u8,
    /// Target position in steps.
    pub position: u16,
    /// Movement time in milliseconds (0 = use speed parameter).
    pub time: u16,
    /// Movement speed in steps/second.
    pub speed: u16,
}

/// Servo status flags.
///
/// Returned by [`blocking_read_status`](crate::ScsclBus::blocking_read_status).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsStatus {
    /// Servo ID.
    pub id: u8,
    /// Voltage is outside configured limits.
    pub voltage_error: bool,
    /// Temperature exceeds configured limit.
    pub temperature_error: bool,
    /// Load exceeds configured torque limit.
    pub overload_error: bool,
}

/// Servo state from sync read.
///
/// Returned by [`blocking_sync_read_state`](crate::ScsclBus::blocking_sync_read_state)
/// for reading multiple servos at once.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsServoState {
    /// Servo ID.
    pub id: u8,
    /// Current position in steps.
    pub position: u16,
    /// Current speed in steps/second (signed, negative = CCW).
    pub speed: f32,
    /// Current load as percentage (signed, negative = CCW).
    pub load: f32,
    /// Supply voltage in volts.
    pub voltage: f32,
    /// Temperature in degrees Celsius.
    pub temperature: f32,
}
