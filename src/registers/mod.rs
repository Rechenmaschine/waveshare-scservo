//! Register definitions for SCServo motors.
//!
//! This module provides register definitions for different servo series:
//! - [`sc_device`]: SCSCL/SC series (potentiometer, big-endian)
//! - [`sts_device`]: SMS/STS series (magnetic encoder, little-endian)

#[cfg(feature = "scscl")]
pub mod sc_device;

#[cfg(feature = "sms_sts")]
pub mod sts_device;

#[cfg(feature = "scscl")]
pub use sc_device::ScsclDevice;

#[cfg(feature = "sms_sts")]
pub use sts_device::SmsStsDevice;

/// Register addresses used for raw protocol operations (sync read/write).
pub mod addr {
    pub const TARGET_POSITION: u8 = 0x2A;
    pub const CURRENT_POSITION: u8 = 0x38;
}

/// Baud rate setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BaudRate {
    Baud1000000 = 0,
    Baud500000 = 1,
    Baud250000 = 2,
    Baud128000 = 3,
    Baud115200 = 4,
    Baud76800 = 5,
    Baud57600 = 6,
    Baud38400 = 7,
    Baud19200 = 8,
    Baud14400 = 9,
    Baud9600 = 10,
    Baud4800 = 11,
}

impl BaudRate {
    #[must_use]
    pub const fn to_bps(self) -> u32 {
        match self {
            BaudRate::Baud1000000 => 1_000_000,
            BaudRate::Baud500000 => 500_000,
            BaudRate::Baud250000 => 250_000,
            BaudRate::Baud128000 => 128_000,
            BaudRate::Baud115200 => 115_200,
            BaudRate::Baud76800 => 76_800,
            BaudRate::Baud57600 => 57_600,
            BaudRate::Baud38400 => 38_400,
            BaudRate::Baud19200 => 19_200,
            BaudRate::Baud14400 => 14_400,
            BaudRate::Baud9600 => 9_600,
            BaudRate::Baud4800 => 4_800,
        }
    }

    #[must_use]
    pub const fn from_raw(value: u8) -> Option<Self> {
        match value {
            0 => Some(BaudRate::Baud1000000),
            1 => Some(BaudRate::Baud500000),
            2 => Some(BaudRate::Baud250000),
            3 => Some(BaudRate::Baud128000),
            4 => Some(BaudRate::Baud115200),
            5 => Some(BaudRate::Baud76800),
            6 => Some(BaudRate::Baud57600),
            7 => Some(BaudRate::Baud38400),
            8 => Some(BaudRate::Baud19200),
            9 => Some(BaudRate::Baud14400),
            10 => Some(BaudRate::Baud9600),
            11 => Some(BaudRate::Baud4800),
            _ => None,
        }
    }
}

impl From<BaudRate> for u8 {
    fn from(br: BaudRate) -> Self {
        br as u8
    }
}

impl TryFrom<u8> for BaudRate {
    type Error = u8;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::from_raw(value).ok_or(value)
    }
}

/// Torque/enable mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TorqueMode {
    Disable = 0,
    Enable = 1,
    Free = 2,
}

impl TorqueMode {
    #[must_use]
    pub const fn from_raw(value: u8) -> Option<Self> {
        match value {
            0 => Some(TorqueMode::Disable),
            1 => Some(TorqueMode::Enable),
            2 => Some(TorqueMode::Free),
            _ => None,
        }
    }
}

impl From<TorqueMode> for u8 {
    fn from(mode: TorqueMode) -> Self {
        mode as u8
    }
}

impl TryFrom<u8> for TorqueMode {
    type Error = u8;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::from_raw(value).ok_or(value)
    }
}
