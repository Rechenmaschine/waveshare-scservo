//! Common types shared across servo series.

/// Servo mode (position vs wheel/motor).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServoMode {
    /// Position control mode (servo mode).
    Position,
    /// Wheel/motor mode (continuous rotation).
    Wheel,
}

/// Telemetry data from a servo.
///
/// Returned by `blocking_read_state()` methods on servo buses.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ServoTelemetry {
    /// Current position in steps.
    /// For SCSCL: 0-1023 (unsigned)
    /// For SMS_STS: signed with bit 15 as sign
    pub position: i16,
    /// Current speed in steps/s (signed, negative = CCW).
    pub speed: i16,
    /// Current load as percentage (signed, negative = CCW torque).
    pub load: f32,
    /// Supply voltage in volts.
    pub voltage: f32,
    /// Temperature in Celsius.
    pub temperature: f32,
    /// Current draw in mA (only available on some servos).
    pub current: Option<f32>,
    /// Whether the servo is currently moving.
    pub moving: bool,
}

