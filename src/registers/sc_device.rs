//! SCSCL series register definitions (SC09, SC15, etc.).
//!
//! These servos use big-endian wire format and potentiometer-based position sensing.

use super::{BaudRate as SharedBaudRate, TorqueMode as SharedTorqueMode};

device_driver::create_device!(
    device_name: ScsclDevice,
    dsl: {
        config {
            type DefaultRegisterAccess = RO;
            type DefaultFieldAccess = RW;
            type DefaultBufferAccess = RW;
            type DefaultByteOrder = BE;
            type DefaultBitOrder = LSB0;
            type RegisterAddressType = u8;
            type NameWordBoundaries = [
                Underscore, Hyphen, Space, LowerUpper,
            ];
            type DefmtFeature = "defmt";
        }

        /// Firmware major version (EPROM, read-only).
        register FW_MAJOR_VERSION {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Firmware minor version (EPROM, read-only).
        register FW_MINOR_VERSION {
            const ADDRESS = 0x01;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Endianness indicator (`1` = big-endian; EPROM, read-only).
        register ENDIANNESS {
            const ADDRESS = 0x02;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 1;

            endian: uint = 0..8,
        },

        /// Servo major version (EPROM, read-only).
        register SERVO_MAJOR_VERSION {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Servo minor version (EPROM, read-only).
        register SERVO_MINOR_VERSION {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Servo ID (EPROM, `0..=253`, default `1`; `0xFE` is broadcast).
        register ID {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            id: uint = 0..8,
        },

        /// Baud-rate setting (EPROM, values `0..=11`, default `0`).
        register BAUDRATE {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            baudrate: uint as try SharedBaudRate = 0..8,
        },

        /// Response status level (EPROM; `0` = READ/PING only, `1` = all commands).
        register RESPONSE_STATUS_LEVEL {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            level: bool = 0,
        },

        /// Minimum angle limit in steps (EPROM, `0..=1023`, default `20`; `0` selects wheel/PWM mode).
        register MINIMUM_ANGLE {
            const ADDRESS = 0x09;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 20;

            angle: uint = 0..16,
        },

        /// Maximum angle limit in steps (EPROM, `0..=1023`, default `1003`; `0` selects wheel/PWM mode).
        register MAXIMUM_ANGLE {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1003;

            angle: uint = 0..16,
        },

        /// Maximum temperature limit in °C (EPROM, `0..=100`, default `70`).
        register MAXIMUM_TEMPERATURE {
            const ADDRESS = 0x0D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 70;

            temperature: uint = 0..8,
        },

        /// Maximum input voltage in `0.1V` units (EPROM, `0..=254`).
        register MAXIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            voltage: uint = 0..8,
        },

        /// Minimum input voltage in `0.1V` units (EPROM, `0..=254`, default `40`/4.0V).
        register MINIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 40;

            voltage: uint = 0..8,
        },

        /// Maximum torque in `0.1%` units (EPROM, `0..=1000`, default `1000`/100%).
        register MAXIMUM_TORQUE {
            const ADDRESS = 0x10;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            torque: uint = 0..16,
        },

        /// Phase (special-function byte, EPROM).
        register PHASE {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            phase: uint = 0..8,
        },

        /// Unloading/protection conditions (EPROM).
        register UNLOADING_CONDITIONS {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// LED alarm conditions (EPROM).
        register LED_ALARM_CONDITION {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// P (proportional) coefficient (EPROM, `0..=254`).
        register P_COEFFICIENT {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// D (derivative) coefficient (EPROM, `0..=254`).
        register D_COEFFICIENT {
            const ADDRESS = 0x16;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// Minimum starting force in `0.1%` units (EPROM, `0..=1000`).
        register MINIMUM_STARTING_FORCE {
            const ADDRESS = 0x18;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            force: uint = 0..16,
        },

        /// Clockwise insensitive zone in steps (EPROM, `0..=16`, default `1`).
        register CLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            zone: uint = 0..8,
        },

        /// Anticlockwise insensitive zone in steps (EPROM, `0..=16`, default `1`).
        register ANTICLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            zone: uint = 0..8,
        },

        /// Protection torque in `1%` units (EPROM, `0..=254`, default `20`).
        register PROTECTION_TORQUE {
            const ADDRESS = 0x25;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 20;

            torque: uint = 0..8,
        },

        /// Protection time in 10 ms units (EPROM, `0..=254`, default `200`/2 s).
        register PROTECTION_TIME {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 200;

            time: uint = 0..8,
        },

        /// Overload torque threshold in `1%` units (EPROM, `0..=254`, default `80`).
        register OVERLOAD_TORQUE {
            const ADDRESS = 0x27;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            torque: uint = 0..8,
        },

        /// Torque switch (SRAM; `0` = free, `1` = enabled, `2` = damping).
        register TORQUE_SWITCH {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            mode: uint as try SharedTorqueMode = 0..8,
        },

        /// Target position in steps (SRAM, `0..=1023`).
        register TARGET_POSITION {
            const ADDRESS = 0x2A;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Goal time in milliseconds (`0..=9999`) in position mode; signed PWM output
        /// in wheel mode (bit 10 is the sign bit, typically `-1000..=1000`; SRAM).
        register GOAL_TIME {
            const ADDRESS = 0x2C;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            time: uint = 0..16,
        },

        /// Goal speed in steps/s (`0..=1000`) for position moves (SRAM); wheel mode
        /// uses `GOAL_TIME` for PWM.
        register GOAL_SPEED {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// EEPROM lock flag (SRAM; `0` = unlocked, `1` = locked).
        register LOCK_FLAG {
            const ADDRESS = 0x30;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            locked: bool = 0,
        },

        /// Current position in steps (SRAM, read-only).
        register CURRENT_POSITION {
            const ADDRESS = 0x38;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Current speed in signed steps/s (bit 15 is the sign bit; SRAM, read-only).
        register CURRENT_SPEED {
            const ADDRESS = 0x3A;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// Current load in signed `0.1%` units (bit 10 is the sign bit; SRAM, read-only).
        register CURRENT_LOAD {
            const ADDRESS = 0x3C;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            load: uint = 0..16,
        },

        /// Current voltage in `0.1V` units (SRAM, read-only).
        register CURRENT_VOLTAGE {
            const ADDRESS = 0x3E;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            voltage: uint = 0..8,
        },

        /// Current temperature in °C (SRAM, read-only).
        register CURRENT_TEMPERATURE {
            const ADDRESS = 0x3F;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            temperature: uint = 0..8,
        },

        /// Asynchronous-write flag (SRAM, read-only).
        register ASYNCHRONOUS_WRITE_FLAG {
            const ADDRESS = 0x40;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            flag: bool = 0,
        },

        /// Servo status error flags (SRAM, read-only).
        register SERVO_STATUS {
            const ADDRESS = 0x41;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// Move flag (SRAM, read-only; `1` = moving, `0` = stopped).
        register MOVE_FLAG {
            const ADDRESS = 0x42;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            flag: bool = 0,
        },

        /// Current draw as a signed raw value (bit 15 is sign; optional register at `0x45`).
        register CURRENT_CURRENT {
            const ADDRESS = 0x45;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            current: uint = 0..16,
        },

    }
);

impl<I> ScsclDevice<I>
where
    I: device_driver::RegisterInterface<AddressType = u8>,
{
    /// Lock EEPROM (prevent writes from being saved).
    pub fn blocking_lock_eeprom(&mut self) -> Result<(), I::Error> {
        self.lock_flag().write(|w| w.set_locked(true))
    }

    /// Unlock EEPROM (allow writes to be saved).
    pub fn blocking_unlock_eeprom(&mut self) -> Result<(), I::Error> {
        self.lock_flag().write(|w| w.set_locked(false))
    }
}

impl<I> ScsclDevice<I>
where
    I: device_driver::AsyncRegisterInterface<AddressType = u8>,
{
    /// Lock EEPROM (prevent writes from being saved).
    pub async fn lock_eeprom(&mut self) -> Result<(), I::Error> {
        self.lock_flag().write_async(|w| w.set_locked(true)).await
    }

    /// Unlock EEPROM (allow writes to be saved).
    pub async fn unlock_eeprom(&mut self) -> Result<(), I::Error> {
        self.lock_flag().write_async(|w| w.set_locked(false)).await
    }
}
