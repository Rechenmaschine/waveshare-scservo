//! SMS/STS series register definitions (STS3215, SMS_STS, etc.).
//!
//! These servos use little-endian wire format and magnetic encoder position sensing.

use super::BaudRate as SharedBaudRate;

device_driver::create_device!(
    device_name: SmsStsDevice,
    dsl: {
        config {
            type DefaultRegisterAccess = RO;
            type DefaultFieldAccess = RW;
            type DefaultBufferAccess = RW;
            type DefaultByteOrder = LE;
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

        /// Endianness indicator (`0` = little-endian; EPROM, read-only).
        register ENDIANNESS {
            const ADDRESS = 0x02;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            endian: uint = 0..8,
        },

        /// Servo major/model version (EPROM, read-only).
        register SERVO_MAJOR_VERSION {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Servo minor/version value (EPROM, read-only).
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

        /// Minimum angle limit in steps (EPROM, `0..=4094`, default `0`).
        register MINIMUM_ANGLE {
            const ADDRESS = 0x09;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            angle: uint = 0..16,
        },

        /// Maximum angle limit in steps (EPROM, `1..=4095`, default `4095`).
        register MAXIMUM_ANGLE {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 4095;

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

        /// Maximum input voltage in `0.1V` units (EPROM, `0..=254`; `0` may disable
        /// voltage feedback when the minimum is also `0`).
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
            magnetic: bool = 1,
            temperature: bool = 2,
            current: bool = 3,
            overload: bool = 5,
        },

        /// LED alarm conditions (EPROM).
        register LED_ALARM_CONDITION {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            voltage: bool = 0,
            magnetic: bool = 1,
            temperature: bool = 2,
            current: bool = 3,
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

        /// I (integral) coefficient (EPROM, `0..=254`, default `0`).
        register I_COEFFICIENT {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// Minimum starting force in `0.1%` units (EPROM, `0..=254`).
        register MINIMUM_STARTING_FORCE {
            const ADDRESS = 0x18;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            force: uint = 0..8,
        },

        /// Position-loop integral limit (EPROM, `0..=254`).
        register INTEGRAL_LIMIT {
            const ADDRESS = 0x19;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            limit: uint = 0..8,
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

        /// Protection current in 6.5 mA units (EPROM, `0..=2047`).
        register PROTECTION_CURRENT {
            const ADDRESS = 0x1C;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 511;

            current: uint = 0..16,
        },

        /// Angular resolution multiplier (EPROM, `1..=128`).
        register ANGLE_RESOLUTION {
            const ADDRESS = 0x1E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            resolution: uint = 0..8,
        },

        /// Signed middle offset (SMS_STS-specific, EPROM).
        register OFFSET {
            const ADDRESS = 0x1F;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            offset: uint = 0..16,
        },

        /// Operating mode (SMS_STS-specific, EPROM; `0` position, `1` wheel,
        /// `2` PWM open-loop, `3` step).
        register MODE {
            const ADDRESS = 0x21;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            mode: uint as enum OperatingMode {
                Position = 0,
                Wheel = 1,
                PwmOpenLoop = 2,
                Step = 3,
                Unknown = catch_all,
            } = 0..8,
        },

        /// Holding torque after overload protection in `1%` units (EPROM, `0..=254`, default `20`).
        register HOLDING_TORQUE {
            const ADDRESS = 0x22;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 20;

            torque: uint = 0..8,
        },

        /// Overload protection time in 10 ms units (EPROM, `0..=254`, default `200`/2 s).
        register PROTECTION_TIME {
            const ADDRESS = 0x23;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 200;

            time: uint = 0..8,
        },

        /// Overload torque threshold in `1%` units (EPROM, `0..=254`, default `80`).
        register OVERLOAD_TORQUE {
            const ADDRESS = 0x24;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            torque: uint = 0..8,
        },

        /// Velocity-loop proportional coefficient (EPROM, `0..=254`).
        register VELOCITY_LOOP_P_COEFFICIENT {
            const ADDRESS = 0x25;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// Over-current protection time in 10 ms units (EPROM, `0..=254`, default `200`/2 s).
        register OVERCURRENT_PROTECTION_TIME {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 200;

            time: uint = 0..8,
        },

        /// Velocity-loop integral coefficient (EPROM, `0..=254`).
        register VELOCITY_LOOP_I_COEFFICIENT {
            const ADDRESS = 0x27;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// Torque switch (SRAM; `0` = free, `1` = enabled, `2` = damping,
        /// `128` = calibration).
        register TORQUE_SWITCH {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            mode: uint as enum TorqueMode {
                Disable = 0,
                Enable = 1,
                Free = 2,
                Calibration = 128,
                Unknown = catch_all,
            } = 0..8,
        },

        /// Acceleration parameter (SMS_STS-specific, SRAM).
        register ACCELERATION {
            const ADDRESS = 0x29;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            acceleration: uint = 0..8,
        },

        /// Target position in signed steps (bit 15 is sign; SRAM, `-32767..=32767`).
        register TARGET_POSITION {
            const ADDRESS = 0x2A;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// PWM magnitude in open-loop mode (bit 10 is sign); position and wheel
        /// commands write zero here and use `GOAL_SPEED` (SRAM).
        register GOAL_TIME {
            const ADDRESS = 0x2C;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            time: uint = 0..16,
        },

        /// Goal speed in steps/s (SRAM); position mode uses a maximum speed and
        /// wheel mode uses signed speed with bit 15 as the sign bit.
        register GOAL_SPEED {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// Runtime torque limit in `0.1%` units (SMS_STS-specific, SRAM).
        register TORQUE_LIMIT {
            const ADDRESS = 0x30;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            torque: uint = 0..16,
        },

        /// EEPROM lock flag (SRAM; `0` = unlocked, `1` = locked).
        register LOCK_FLAG {
            const ADDRESS = 0x37;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            locked: bool = 0,
        },

        /// Current position in signed steps (bit 15 is sign; SRAM, read-only).
        register CURRENT_POSITION {
            const ADDRESS = 0x38;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Current speed in signed steps/s (bit 15 is sign; SRAM, read-only).
        register CURRENT_SPEED {
            const ADDRESS = 0x3A;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// Current load in signed `0.1%` units (bit 10 is sign; SRAM, read-only).
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
            magnetic: bool = 1,
            temperature: bool = 2,
            current: bool = 3,
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

        /// Current target position in signed steps (bit 15 is sign; SRAM, read-only).
        register TARGET_POSITION_READBACK {
            const ADDRESS = 0x43;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Current draw in signed 6.5 mA units (bit 15 is sign; SRAM, read-only, `0x45`).
        register CURRENT_CURRENT {
            const ADDRESS = 0x45;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            current: uint = 0..16,
        },
    }
);

impl<I> SmsStsDevice<I>
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

impl<I> SmsStsDevice<I>
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
