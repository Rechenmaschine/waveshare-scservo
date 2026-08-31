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

/// Servo telemetry in native hardware units.
///
/// Getters provide converted load, voltage, and current values.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ServoTelemetry {
    /// Current position in steps; unsigned for SCSCL and signed for SMS_STS.
    pub position_raw: i16,
    /// Current speed in `steps/second` (signed, negative = CCW).
    pub speed_raw: i16,
    /// Current load in signed `0.1%` units (`-1000..=1000`).
    pub load_raw: i16,
    /// Supply voltage in `0.1V` units (`0..=255`).
    pub voltage_raw: u8,
    /// Temperature in °C.
    pub temperature_raw: u8,
    /// Current draw in device-specific raw units (only available on some servos).
    pub current_raw: Option<i16>,
    /// Whether the servo is currently moving.
    pub moving: bool,
}

impl ServoTelemetry {
    /// Get position in steps.
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

    /// Get current in amperes, treating raw counts as milliamperes.
    ///
    /// SMS/ST uses 6.5 mA per count; use
    /// [`current_with_milliamps_per_count`](Self::current_with_milliamps_per_count)
    /// for that scale.
    #[inline]
    #[must_use]
    pub fn current(&self) -> Option<f32> {
        self.current_with_milliamps_per_count(1.0)
    }

    /// Get current in amperes using the supplied milliampere-per-count scale.
    #[inline]
    #[must_use]
    pub fn current_with_milliamps_per_count(&self, milliamps_per_count: f32) -> Option<f32> {
        self.current_raw
            .map(|c| f32::from(c) * milliamps_per_count * 0.001)
    }
}
