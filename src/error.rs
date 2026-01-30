//! Error types for the SCServo protocol.

use core::fmt;

/// Protocol error type for SCServo communication.
#[derive(Debug)]
pub enum ProtocolError<E> {
    /// Underlying serial/I/O error.
    Serial(E),
    /// Checksum mismatch in received packet.
    Checksum,
    /// Communication timeout (no response received).
    Timeout,
    /// Invalid protocol header received.
    InvalidHeader,
    /// Invalid servo ID in response.
    InvalidId,
    /// Invalid packet or buffer length.
    InvalidLength,
    /// Invalid setting value (out of range or unsupported).
    InvalidSetting,
    /// Servo reported an error (error code in value).
    ServoError(u8),
}

impl<E> From<E> for ProtocolError<E> {
    fn from(err: E) -> Self {
        ProtocolError::Serial(err)
    }
}

impl<E: fmt::Debug> fmt::Display for ProtocolError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProtocolError::Serial(e) => write!(f, "Serial error: {e:?}"),
            ProtocolError::Checksum => write!(f, "Checksum mismatch"),
            ProtocolError::Timeout => write!(f, "Communication timeout"),
            ProtocolError::InvalidHeader => write!(f, "Invalid protocol header"),
            ProtocolError::InvalidId => write!(f, "Invalid servo ID"),
            ProtocolError::InvalidLength => write!(f, "Invalid packet length"),
            ProtocolError::InvalidSetting => write!(f, "Invalid setting value"),
            ProtocolError::ServoError(code) => write!(f, "Servo error: 0x{code:02X}"),
        }
    }
}
