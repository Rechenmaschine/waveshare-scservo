#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::pedantic)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::doc_markdown)]

//! Rust driver for Waveshare SCServo motors.
//!
//! Supports SCSCL (`ScsclBus`) and SMS/STS (`SmsStsBus`) servo series.

#[cfg(test)]
extern crate std;

pub mod error;
pub mod registers;
pub mod series;
mod types;
mod uart;

#[cfg(feature = "scscl")]
mod scscl;

#[cfg(feature = "sms_sts")]
mod sms_sts;

#[cfg(test)]
mod mock;

pub use error::ProtocolError;
pub use registers::{BaudRate, TorqueMode};
pub use series::{ServoMode, ServoTelemetry};
pub use types::{
    ScsStatus, ScsclPositionMove, ScsclServoState, SmsPositionMove, SmsPositionMoveEx,
    SmsServoState, SmsSpeedCommand, SyncWriteData, TorqueModeCommand,
};

#[cfg(feature = "scscl")]
pub use types::ScsclMotorCommand;

#[cfg(feature = "sms_sts")]
pub use types::{SmsStsOperatingMode, SmsTorqueLimitCommand};
pub use uart::{UartBusInterface, VersionInformation};

#[cfg(feature = "scscl")]
pub use scscl::ScsclBus;

#[cfg(feature = "sms_sts")]
pub use sms_sts::SmsStsBus;

/// Broadcast ID (0xFE) - commands sent to this ID reach all servos
pub const BROADCAST_ID: u8 = 0xFE;
/// Highest valid unicast servo ID.
pub const MAX_SERVO_ID: u8 = 0xFD;
/// Default servo ID
pub const DEFAULT_ID: u8 = 1;

/// Maximum torque value (0.1% units)
pub const MAX_TORQUE_VALUE: u16 = 1000;

/// SCSCL series-specific constants
#[cfg(feature = "scscl")]
pub mod scscl_constants {
    /// Resolution of the servo in steps (0-1023)
    pub const SCSCL_RESOLUTION_STEPS: u16 = 1024;
    /// Nominal maximum angle in degrees for some 220° SCSCL models.
    ///
    /// Published mechanical travel differs between SCSCL models, including SC09
    /// and SC15 variants.
    pub const SCSCL_MAX_ANGLE_DEGREES: f32 = 220.0;
    /// Nominal degrees per step corresponding to [`SCSCL_MAX_ANGLE_DEGREES`].
    ///
    /// This is an approximate model calibration, not a protocol invariant.
    pub const SCSCL_DEGREES_PER_STEP: f32 = 0.214_843_75;
    /// No-load speed in steps per second
    pub const SCSCL_NO_LOAD_SPEED_STEPS_PER_SEC: u16 = 1500;
    /// No-load speed in RPM
    pub const SCSCL_NO_LOAD_SPEED_RPM: u16 = 54;
    /// Maximum position value (steps)
    pub const SCSCL_MAX_POSITION_STEPS: u16 = 1023;

    /// Convert degrees to steps for SCSCL series.
    #[must_use]
    pub const fn scscl_degrees_to_steps(degrees: f32) -> u16 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            let steps = (degrees / SCSCL_DEGREES_PER_STEP) as u16;
            if steps > SCSCL_MAX_POSITION_STEPS {
                SCSCL_MAX_POSITION_STEPS
            } else {
                steps
            }
        }
    }

    /// Convert steps to degrees for SCSCL series.
    #[must_use]
    pub const fn scscl_steps_to_degrees(steps: u16) -> f32 {
        steps as f32 * SCSCL_DEGREES_PER_STEP
    }
}

/// SMS_STS series-specific constants
#[cfg(feature = "sms_sts")]
pub mod sms_sts_constants {
    /// Resolution of the servo (12-bit magnetic encoder)
    ///
    /// The encoder provides 4096 distinct positions across 360°.
    pub const SMS_STS_RESOLUTION_STEPS: u16 = 4096;
    /// Maximum effective angle in degrees (full rotation)
    pub const SMS_STS_MAX_ANGLE_DEGREES: f32 = 360.0;
    /// Angular resolution (degrees per step)
    pub const SMS_STS_DEGREES_PER_STEP: f32 = 0.087_890_625;
    /// Maximum position value (12-bit encoder: 0-4095)
    ///
    /// With offset calibration, positions can be represented as signed coordinates,
    /// but the total span is always 4096 positions.
    pub const SMS_STS_MAX_POSITION_STEPS: u16 = 4095;

    /// Convert degrees to steps for SMS_STS series.
    #[must_use]
    pub const fn sms_sts_degrees_to_steps(degrees: f32) -> u16 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            let steps = (degrees / SMS_STS_DEGREES_PER_STEP) as u16;
            if steps > SMS_STS_MAX_POSITION_STEPS {
                SMS_STS_MAX_POSITION_STEPS
            } else {
                steps
            }
        }
    }

    /// Convert steps to degrees for SMS_STS series.
    #[must_use]
    pub const fn sms_sts_steps_to_degrees(steps: u16) -> f32 {
        steps as f32 * SMS_STS_DEGREES_PER_STEP
    }
}

#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
const BIT_15_SIGN: u16 = 0x8000;
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
const BIT_15_VALUE: u16 = 0x7FFF;
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
const BIT_10_SIGN: u16 = 0x0400;
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
const BIT_10_VALUE: u16 = 0x03FF;

/// Decode signed speed (bit 15 = sign).
///
/// Returns: i16 in steps/second (native hardware unit).
#[allow(clippy::cast_possible_wrap)]
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
pub(crate) fn decode_speed(speed_raw: u16) -> i16 {
    if speed_raw & BIT_15_SIGN != 0 {
        -((speed_raw & BIT_15_VALUE) as i16)
    } else {
        speed_raw as i16
    }
}

/// Decode signed load (bit 10 = sign).
///
/// Returns: i16 in 0.1% units (0-1000 = 0-100%, native hardware unit).
/// To convert to percentage: `(load as f32) * 0.1`
#[allow(clippy::cast_possible_wrap)]
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
pub(crate) fn decode_load(load_raw: u16) -> i16 {
    if load_raw & BIT_10_SIGN != 0 {
        -((load_raw & BIT_10_VALUE) as i16)
    } else {
        load_raw as i16
    }
}

/// Decode signed current (bit 15 = sign).
///
/// Returns: i16 in native device-specific units.
#[allow(clippy::cast_possible_wrap)]
#[cfg(any(feature = "scscl", feature = "sms_sts", test))]
pub(crate) fn decode_current(current_raw: u16) -> i16 {
    if current_raw & BIT_15_SIGN != 0 {
        -((current_raw & BIT_15_VALUE) as i16)
    } else {
        current_raw as i16
    }
}

/// Encode signed PWM (bit 10 = sign).
#[cfg(feature = "scscl")]
pub(crate) fn encode_signed_pwm(pwm: i16) -> Option<u16> {
    let magnitude = pwm.unsigned_abs();
    if magnitude > BIT_10_VALUE {
        None
    } else if pwm < 0 {
        Some(magnitude | BIT_10_SIGN)
    } else {
        Some(magnitude)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_speed() {
        assert_eq!(decode_speed(100), 100);
        assert_eq!(decode_speed(0x8064), -100);
        assert_eq!(decode_speed(0), 0);
    }

    #[test]
    fn test_decode_load() {
        assert_eq!(decode_load(100), 100);
        assert_eq!(decode_load(0x0464), -100);
        assert_eq!(decode_load(0), 0);
    }

    #[test]
    fn test_decode_current() {
        assert_eq!(decode_current(100), 100);
        assert_eq!(decode_current(0x8064), -100);
        assert_eq!(decode_current(0), 0);
    }

    #[test]
    fn test_current_scaling() {
        let telemetry = ServoTelemetry {
            position_raw: 0,
            speed_raw: 0,
            load_raw: 0,
            voltage_raw: 0,
            temperature_raw: 0,
            current_raw: Some(100),
            moving: false,
        };
        assert_eq!(telemetry.current(), Some(0.1));
        assert!((telemetry.current_with_milliamps_per_count(6.5).unwrap() - 0.65).abs() < 1e-6);
    }

    #[cfg(feature = "scscl")]
    #[test]
    fn test_encode_pwm() {
        assert_eq!(encode_signed_pwm(100), Some(100));
        assert_eq!(encode_signed_pwm(-100), Some(0x0464));
        assert_eq!(encode_signed_pwm(1023), Some(1023));
        assert_eq!(encode_signed_pwm(-1023), Some(0x07FF));
        assert_eq!(encode_signed_pwm(1024), None);
        assert_eq!(encode_signed_pwm(i16::MIN), None);
    }

    #[cfg(feature = "scscl")]
    #[test]
    fn test_degrees_conversion_scscl() {
        let steps = scscl_constants::scscl_degrees_to_steps(110.0);
        let degrees = scscl_constants::scscl_steps_to_degrees(steps);
        assert!((degrees - 110.0).abs() < 1.0);
        assert_eq!(
            scscl_constants::scscl_degrees_to_steps(scscl_constants::SCSCL_MAX_ANGLE_DEGREES),
            scscl_constants::SCSCL_MAX_POSITION_STEPS
        );
    }

    #[cfg(feature = "sms_sts")]
    #[test]
    fn test_degrees_conversion_sms_sts() {
        let steps = sms_sts_constants::sms_sts_degrees_to_steps(180.0);
        let degrees = sms_sts_constants::sms_sts_steps_to_degrees(steps);
        assert!((degrees - 180.0).abs() < 1.0);
        assert_eq!(
            sms_sts_constants::sms_sts_degrees_to_steps(
                sms_sts_constants::SMS_STS_MAX_ANGLE_DEGREES
            ),
            sms_sts_constants::SMS_STS_MAX_POSITION_STEPS
        );
    }
}
