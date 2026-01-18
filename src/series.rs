//! Servo series marker traits and types.
//!
//! This module provides compile-time differentiation between servo series:
//! - [`SCSCL`]: Potentiometer-based servos (SC09, SC15, etc.) - big-endian wire format
//! - [`SMS_STS`]: Magnetic encoder servos (STS3215, SMS_STS, etc.) - little-endian wire format

use core::marker::PhantomData;

/// Marker trait for servo series.
///
/// This trait provides compile-time information about the capabilities
/// and characteristics of different servo series.
pub trait ServoSeries: private::Sealed {
    /// Whether the servo supports negative position values.
    /// - SCSCL: false (0-1023 unsigned)
    /// - SMS_STS: true (bit 15 = sign)
    const SUPPORTS_NEGATIVE_POSITION: bool;

    /// Whether the servo has a dedicated acceleration register.
    /// - SCSCL: false
    /// - SMS_STS: true (ACC register at 0x29)
    const SUPPORTS_ACCELERATION: bool;

    /// Whether the servo has a native wheel/motor mode register.
    /// - SCSCL: false (emulated via angle limits = 0)
    /// - SMS_STS: true (MODE register at 0x21)
    const NATIVE_WHEEL_MODE: bool;

    /// EEPROM lock register address (differs between series).
    /// - SCSCL: 0x30
    /// - SMS_STS: 0x37
    const LOCK_ADDRESS: u8;
}

/// SCSCL series marker (SC09, SC15, etc.).
///
/// Potentiometer-based servos with:
/// - Big-endian wire format
/// - 0-1023 position range (unsigned)
/// - PWM mode via angle limits = 0
/// - LOCK at 0x30
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SCSCL;

/// SMS/STS series marker (STS3215, SMS_STS, etc.).
///
/// Magnetic encoder servos with:
/// - Little-endian wire format
/// - Signed position support
/// - Native wheel mode register
/// - Acceleration parameter
/// - LOCK at 0x37
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub struct SMS_STS;

impl ServoSeries for SCSCL {
    const SUPPORTS_NEGATIVE_POSITION: bool = false;
    const SUPPORTS_ACCELERATION: bool = false;
    const NATIVE_WHEEL_MODE: bool = false;
    const LOCK_ADDRESS: u8 = 0x30;
}

impl ServoSeries for SMS_STS {
    const SUPPORTS_NEGATIVE_POSITION: bool = true;
    const SUPPORTS_ACCELERATION: bool = true;
    const NATIVE_WHEEL_MODE: bool = true;
    const LOCK_ADDRESS: u8 = 0x37;
}

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
/// Returned by the [`feedback()`](crate::ServoBus::blocking_feedback) method.
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

/// Generic servo bus with compile-time series selection.
///
/// This struct wraps the underlying register interface and provides
/// a high-level API for controlling servos.
///
/// # Type Parameters
/// - `I`: The I/O interface type (must implement embedded-io traits)
/// - `S`: The servo series marker ([`SCSCL`] or [`SMS_STS`])
pub struct ServoBus<I, S: ServoSeries> {
    pub(crate) interface: I,
    pub(crate) _series: PhantomData<S>,
}

impl<I, S: ServoSeries> ServoBus<I, S> {
    /// Create a new servo bus with the given interface.
    pub fn new(interface: I) -> Self {
        Self {
            interface,
            _series: PhantomData,
        }
    }

    /// Get mutable access to the underlying interface.
    pub fn inner_mut(&mut self) -> &mut I {
        &mut self.interface
    }

    /// Consume the bus and return the underlying interface.
    pub fn into_inner(self) -> I {
        self.interface
    }
}

// Sealed trait pattern to prevent external implementations
mod private {
    pub trait Sealed {}
    impl Sealed for super::SCSCL {}
    impl Sealed for super::SMS_STS {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::assertions_on_constants)]
    fn test_series_constants() {
        // SCSCL characteristics
        assert!(!SCSCL::SUPPORTS_NEGATIVE_POSITION);
        assert!(!SCSCL::SUPPORTS_ACCELERATION);
        assert!(!SCSCL::NATIVE_WHEEL_MODE);
        assert_eq!(SCSCL::LOCK_ADDRESS, 0x30);

        // SMS_STS characteristics
        assert!(SMS_STS::SUPPORTS_NEGATIVE_POSITION);
        assert!(SMS_STS::SUPPORTS_ACCELERATION);
        assert!(SMS_STS::NATIVE_WHEEL_MODE);
        assert_eq!(SMS_STS::LOCK_ADDRESS, 0x37);
    }
}
