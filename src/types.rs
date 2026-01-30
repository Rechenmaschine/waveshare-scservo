//! Type definitions for SCServo protocol.
//!
//! # Position Type Differences
//!
//! **Important:** SCSCL and SMS_STS have different position representations:
//!
//! - **SCSCL**: Unsigned `u16` (0-1023 steps, potentiometer-based)
//!   - Use [`ScsclPositionMove`] and [`ScsclServoState`]
//!   - Physical range: 220° (0.21° per step)
//!
//! - **SMS_STS**: Signed `i16` (12-bit magnetic encoder, 4096 positions)
//!   - Use [`SmsPositionMove`] and [`SmsServoState`]
//!   - Physical range: 0-4095 steps (360°, 0.088° per step)
//!   - With offset: Can represent as signed coordinates (e.g., -2048 to +2047)
//!   - **Note:** Total span is always 4096 positions, offset just shifts the coordinate system
//!   - Hardware uses bit 15 as sign flag (abstracted away by this library)

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

/// Position move command for SCSCL series sync writes.
///
/// Used with [`ScsclBus::blocking_sync_write_position`](crate::ScsclBus::blocking_sync_write_position).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsclPositionMove {
    /// Servo ID.
    pub id: u8,
    /// Target position in `steps` (0-1023 for SCSCL).
    pub position: u16,
    /// Movement time in `milliseconds` (0 = use speed parameter).
    pub time: u16,
    /// Movement speed in `steps/second`.
    pub speed: u16,
}

/// Position move command for SMS_STS series sync writes.
///
/// Used with [`SmsStsBus::blocking_sync_write_position`](crate::SmsStsBus::blocking_sync_write_position).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsPositionMove {
    /// Servo ID.
    pub id: u8,
    /// Target position in `steps` (12-bit encoder: 0-4095, or offset-adjusted signed range).
    ///
    /// **Default (no offset):** 0 to 4095
    ///
    /// **With offset:** Can use signed coordinates, but span is always 4096 positions.
    /// Example: offset=2048 gives range -2048 to +2047.
    pub position: i16,
    /// Movement time in `milliseconds` (0 = use speed parameter).
    pub time: u16,
    /// Movement speed in `steps/second`.
    pub speed: u16,
}

/// Position move command with acceleration for SMS_STS series.
///
/// Used with [`SmsStsBus::blocking_sync_write_position_ex`](crate::SmsStsBus::blocking_sync_write_position_ex).
/// Writes to ACCELERATION register (0x29) + position/time/speed.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsPositionMoveEx {
    /// Servo ID.
    pub id: u8,
    /// Acceleration value (0-254).
    pub acceleration: u8,
    /// Target position in `steps` (12-bit encoder: 0-4095, or offset-adjusted signed range).
    pub position: i16,
    /// Movement time in `milliseconds` (0 = use speed parameter).
    pub time: u16,
    /// Movement speed in `steps/second`.
    pub speed: u16,
}

/// Speed command for wheel mode (SMS_STS series).
///
/// Used with [`SmsStsBus::blocking_sync_write_speed`](crate::SmsStsBus::blocking_sync_write_speed).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsSpeedCommand {
    /// Servo ID.
    pub id: u8,
    /// Speed in `steps/second` (signed: positive = CW, negative = CCW).
    pub speed: i16,
    /// Acceleration value (0-254).
    pub acceleration: u8,
}

/// Torque mode command for sync writes.
///
/// Used with [`SmsStsBus::blocking_sync_write_torque_mode`](crate::SmsStsBus::blocking_sync_write_torque_mode)
/// and [`ScsclBus::blocking_sync_write_torque_mode`](crate::ScsclBus::blocking_sync_write_torque_mode).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TorqueModeCommand {
    /// Servo ID.
    pub id: u8,
    /// Torque mode (Enable/Disable/Free/Calibration).
    pub mode: crate::TorqueMode,
}

/// Motor output command for SCSCL wheel mode sync writes.
///
/// Used with [`ScsclBus::blocking_sync_write_motor`](crate::ScsclBus::blocking_sync_write_motor).
#[cfg(feature = "scscl")]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsclMotorCommand {
    /// Servo ID.
    pub id: u8,
    /// Motor output (signed PWM: positive = CW, negative = CCW).
    /// Range depends on servo model, typically -1000 to 1000.
    pub output: i16,
}

/// Torque limit command for SMS_STS runtime torque limiting.
///
/// Used with [`SmsStsBus::blocking_sync_write_torque_limit`](crate::SmsStsBus::blocking_sync_write_torque_limit).
#[cfg(feature = "sms_sts")]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsTorqueLimitCommand {
    /// Servo ID.
    pub id: u8,
    /// Torque limit in `0.1%` units (500 = 50.0%, max 1000 = 100%).
    pub limit: u16,
}

/// Generic sync write command with fixed-size data.
///
/// Used with [`SmsStsBus::blocking_sync_write`](crate::SmsStsBus::blocking_sync_write_raw)
/// and [`ScsclBus::blocking_sync_write`](crate::ScsclBus::blocking_sync_write).
///
/// The `DATA_LEN` const generic specifies how many bytes to write per servo.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyncWriteData<const DATA_LEN: usize> {
    /// Servo ID.
    pub id: u8,
    /// Data bytes to write (length determined by const generic).
    pub data: [u8; DATA_LEN],
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

/// SCSCL servo state from sync read.
///
/// Returned by [`ScsclBus::blocking_sync_read_state`](crate::ScsclBus::blocking_sync_read_state).
///
/// Raw values stored in native hardware units. Use getter methods for converted values:
/// - [`load()`](Self::load) - Returns load as percentage
/// - [`voltage()`](Self::voltage) - Returns voltage in volts
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScsclServoState {
    /// Servo ID.
    pub id: u8,
    /// Current position in `steps` (0-1023 for SCSCL).
    pub position_raw: u16,
    /// Current speed in `steps/second` (signed, negative = CCW).
    pub speed_raw: i16,
    /// Current load in `0.1%` units (signed, negative = CCW).
    /// Range: -1000 to +1000 representing -100% to +100%.
    pub load_raw: i16,
    /// Supply voltage in `0.1V` units (0-255 = 0.0V to 25.5V).
    pub voltage_raw: u8,
    /// Temperature in `°C` (direct value).
    pub temperature_raw: u8,
}

impl ScsclServoState {
    /// Get position in steps (0-1023 for SCSCL).
    #[inline]
    pub fn position(&self) -> u16 {
        self.position_raw
    }

    /// Get speed in steps/second (signed, negative = CCW).
    #[inline]
    pub fn speed(&self) -> i16 {
        self.speed_raw
    }

    /// Get load as percentage (-100.0 to +100.0, negative = CCW).
    #[inline]
    pub fn load(&self) -> f32 {
        self.load_raw as f32 * 0.1
    }

    /// Get voltage in volts.
    #[inline]
    pub fn voltage(&self) -> f32 {
        self.voltage_raw as f32 * 0.1
    }

    /// Get temperature in degrees Celsius.
    #[inline]
    pub fn temperature(&self) -> u8 {
        self.temperature_raw
    }
}

/// SMS_STS servo state from sync read.
///
/// Returned by [`SmsStsBus::blocking_sync_read_state`](crate::SmsStsBus::blocking_sync_read_state).
///
/// Raw values stored in native hardware units. Use getter methods for converted values:
/// - [`load()`](Self::load) - Returns load as percentage
/// - [`voltage()`](Self::voltage) - Returns voltage in volts
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsServoState {
    /// Servo ID.
    pub id: u8,
    /// Current position in `steps` (12-bit encoder: 0-4095).
    pub position_raw: i16,
    /// Current speed in `steps/second` (signed, negative = CCW).
    pub speed_raw: i16,
    /// Current load in `0.1%` units (signed, negative = CCW).
    /// Range: -1000 to +1000 representing -100% to +100%.
    pub load_raw: i16,
    /// Supply voltage in `0.1V` units (0-255 = 0.0V to 25.5V).
    pub voltage_raw: u8,
    /// Temperature in `°C` (direct value).
    pub temperature_raw: u8,
}

impl SmsServoState {
    /// Get position in steps (12-bit encoder: 0-4095).
    #[inline]
    pub fn position(&self) -> i16 {
        self.position_raw
    }

    /// Get speed in steps/second (signed, negative = CCW).
    #[inline]
    pub fn speed(&self) -> i16 {
        self.speed_raw
    }

    /// Get load as percentage (-100.0 to +100.0, negative = CCW).
    #[inline]
    pub fn load(&self) -> f32 {
        self.load_raw as f32 * 0.1
    }

    /// Get voltage in volts.
    #[inline]
    pub fn voltage(&self) -> f32 {
        self.voltage_raw as f32 * 0.1
    }

    /// Get temperature in degrees Celsius.
    #[inline]
    pub fn temperature(&self) -> u8 {
        self.temperature_raw
    }
}

