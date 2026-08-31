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

/// Position move command for SCSCL sync writes.
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

/// Position move command for SMS_STS sync writes.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsPositionMove {
    /// Servo ID.
    pub id: u8,
    /// Target position in signed protocol steps (`-32767..=32767`).
    /// The effective mechanical range depends on the servo model and mode.
    pub position: i16,
    /// Retained for source compatibility. SMS/ST position commands write zero
    /// to address `0x2C`; PWM output uses a raw register write in PWM mode.
    pub time: u16,
    /// Movement speed in `steps/second`.
    pub speed: u16,
}

/// SMS_STS position move command with acceleration.
///
/// Writes the seven-byte layout beginning at `ACCELERATION` (`0x29`).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsPositionMoveEx {
    /// Servo ID.
    pub id: u8,
    /// Acceleration value (0-254).
    pub acceleration: u8,
    /// Target position in signed protocol steps (`-32767..=32767`).
    /// The effective mechanical range depends on the servo model and mode.
    pub position: i16,
    /// Retained for source compatibility. SMS/ST position commands write zero
    /// to address `0x2C`; PWM output uses a raw register write in PWM mode.
    pub time: u16,
    /// Movement speed in `steps/second`.
    pub speed: u16,
}

/// Operating modes supported by SMS/ST servos.
#[cfg(feature = "sms_sts")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SmsStsOperatingMode {
    /// Closed-loop position control.
    Position,
    /// Continuous-rotation wheel/motor control.
    Wheel,
    /// Open-loop PWM control.
    PwmOpenLoop,
    /// Step control.
    Step,
}

/// SMS_STS wheel-mode speed command.
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
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TorqueModeCommand {
    /// Servo ID.
    pub id: u8,
    /// Torque mode (Enable/Disable/Free-or-damping, raw values 1/0/2).
    pub mode: crate::TorqueMode,
}

/// SCSCL wheel-mode motor output command.
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

/// SMS_STS runtime torque-limit command.
#[cfg(feature = "sms_sts")]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsTorqueLimitCommand {
    /// Servo ID.
    pub id: u8,
    /// Torque limit in `0.1%` units (500 = 50.0%, max 1000 = 100%).
    pub limit: u16,
}

/// Generic sync-write command with fixed-size data.
///
/// `DATA_LEN` specifies the number of bytes written per servo.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyncWriteData<const DATA_LEN: usize> {
    /// Servo ID.
    pub id: u8,
    /// Data bytes to write (length determined by const generic).
    pub data: [u8; DATA_LEN],
}

/// Servo status flags returned by the series-specific status methods.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::struct_excessive_bools)]
pub struct ScsStatus {
    /// Servo ID.
    pub id: u8,
    /// Voltage is outside configured limits.
    pub voltage_error: bool,
    /// Temperature exceeds configured limit.
    pub temperature_error: bool,
    /// Load exceeds configured torque limit.
    pub overload_error: bool,
    /// Magnetic encoder fault; always false for SCSCL.
    pub magnetic_error: bool,
    /// Over-current fault; always false for SCSCL.
    pub current_error: bool,
}

/// SCSCL servo state from a sync read.
///
/// Values are stored in native hardware units; getters provide converted load and voltage.
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
    #[must_use]
    pub fn position(&self) -> u16 {
        self.position_raw
    }

    /// Get speed in steps/second (signed, negative = CCW).
    #[inline]
    #[must_use]
    pub fn speed(&self) -> i16 {
        self.speed_raw
    }

    /// Get load as a percentage (-100.0 to +100.0).
    #[inline]
    #[must_use]
    pub fn load(&self) -> f32 {
        f32::from(self.load_raw) * 0.1
    }

    /// Get voltage in volts.
    #[inline]
    #[must_use]
    pub fn voltage(&self) -> f32 {
        f32::from(self.voltage_raw) * 0.1
    }

    /// Get temperature in degrees Celsius.
    #[inline]
    #[must_use]
    pub fn temperature(&self) -> u8 {
        self.temperature_raw
    }
}

/// SMS_STS servo state from a sync read.
///
/// Values are stored in native hardware units; getters provide converted load and voltage.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SmsServoState {
    /// Servo ID.
    pub id: u8,
    /// Current position in signed protocol steps.
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
    /// Get position in signed protocol steps.
    #[inline]
    #[must_use]
    pub fn position(&self) -> i16 {
        self.position_raw
    }

    /// Get speed in steps/second (signed, negative = CCW).
    #[inline]
    #[must_use]
    pub fn speed(&self) -> i16 {
        self.speed_raw
    }

    /// Get load as a percentage (-100.0 to +100.0).
    #[inline]
    #[must_use]
    pub fn load(&self) -> f32 {
        f32::from(self.load_raw) * 0.1
    }

    /// Get voltage in volts.
    #[inline]
    #[must_use]
    pub fn voltage(&self) -> f32 {
        f32::from(self.voltage_raw) * 0.1
    }

    /// Get temperature in degrees Celsius.
    #[inline]
    #[must_use]
    pub fn temperature(&self) -> u8 {
        self.temperature_raw
    }
}
