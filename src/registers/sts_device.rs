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

        /// Firmware major version number
        /// - Min: 0, Max: 253
        /// - Initial: 0
        /// - Storage: EPROM
        register FW_MAJOR_VERSION {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            version_number: uint = 0..8,
        },

        /// Firmware minor version number
        /// - Min: 0, Max: 253
        /// - Initial: 5
        /// - Storage: EPROM
        register FW_MINOR_VERSION {
            const ADDRESS = 0x01;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 5;

            version_number: uint = 0..8,
        },

        /// Servo major version number
        /// - Min: 0, Max: 253
        /// - Initial: 5
        /// - Storage: EPROM
        register SERVO_MAJOR_VERSION {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 5;

            version_number: uint = 0..8,
        },

        /// Servo minor version number
        /// - Min: 0, Max: 253
        /// - Initial: 15
        /// - Storage: EPROM
        register SERVO_MINOR_VERSION {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            version_number: uint = 0..8,
        },

        /// Servo ID (unique on bus)
        /// - Min: 0, Max: 253
        /// - Initial: 1
        /// - Storage: EPROM
        /// ID 254 (0xFE) is broadcast
        register ID {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            id: uint = 0..8,
        },

        /// Baudrate setting
        /// - Min: 0, Max: 11
        /// - Initial: 0
        /// - Storage: EPROM
        register BAUDRATE {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            baudrate: uint as try SharedBaudRate = 0..8,
        },

        /// Return delay in 2µs units
        /// - Min: 0, Max: 254
        /// - Initial: 0
        /// - Storage: EPROM
        register RETURN_DELAY {
            const ADDRESS = 0x07;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            delay: uint = 0..8,
        },

        /// Response status level
        /// - 0: Only read/PING return responses
        /// - 1: All commands return responses
        /// - Storage: EPROM
        register RESPONSE_STATUS_LEVEL {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            level: bool = 0,
        },

        /// Minimum angle limit (steps)
        /// - Min: 0, Max: 4095
        /// - Initial: 0
        /// - Storage: EPROM
        register MINIMUM_ANGLE {
            const ADDRESS = 0x09;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            angle: uint = 0..16,
        },

        /// Maximum angle limit (steps)
        /// - Min: 0, Max: 4095
        /// - Initial: 4095
        /// - Storage: EPROM
        register MAXIMUM_ANGLE {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 4095;

            angle: uint = 0..16,
        },

        /// Maximum temperature limit (°C)
        /// - Min: 0, Max: 100
        /// - Initial: 80
        /// - Storage: EPROM
        register MAXIMUM_TEMPERATURE {
            const ADDRESS = 0x0D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            temperature: uint = 0..8,
        },

        /// Maximum input voltage (0.1V units)
        /// - Min: 0, Max: 254
        /// - Initial: 140 (14.0V)
        /// - Storage: EPROM
        register MAXIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 140;

            voltage: uint = 0..8,
        },

        /// Minimum input voltage (0.1V units)
        /// - Min: 0, Max: 254
        /// - Initial: 50 (5.0V)
        /// - Storage: EPROM
        register MINIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 50;

            voltage: uint = 0..8,
        },

        /// Maximum torque (0.1% units)
        /// - Min: 0, Max: 1000
        /// - Initial: 1000 (100%)
        /// - Storage: EPROM
        register MAXIMUM_TORQUE {
            const ADDRESS = 0x10;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            torque: uint = 0..16,
        },

        /// Phase (special function byte)
        /// - Storage: EPROM
        register PHASE {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            phase: uint = 0..8,
        },

        /// Unloading/protection conditions
        /// - Storage: EPROM
        register UNLOADING_CONDITIONS {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 36;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// LED alarm conditions
        /// - Storage: EPROM
        register LED_ALARM_CONDITION {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 37;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// P (proportional) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 15
        /// - Storage: EPROM
        register P_COEFFICIENT {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            coefficient: uint = 0..8,
        },

        /// D (derivative) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 15
        /// - Storage: EPROM
        register D_COEFFICIENT {
            const ADDRESS = 0x16;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            coefficient: uint = 0..8,
        },

        /// I (integral) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 0
        /// - Storage: EPROM
        register I_COEFFICIENT {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            coefficient: uint = 0..8,
        },

        /// Minimum starting force (0.1% units)
        /// - Min: 0, Max: 1000
        /// - Initial: 30
        /// - Storage: EPROM
        register MINIMUM_STARTING_FORCE {
            const ADDRESS = 0x18;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 30;

            force: uint = 0..16,
        },

        /// Clockwise insensitive zone (steps)
        /// - Min: 0, Max: 32
        /// - Initial: 0
        /// - Storage: EPROM
        register CLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            zone: uint = 0..8,
        },

        /// Anticlockwise insensitive zone (steps)
        /// - Min: 0, Max: 32
        /// - Initial: 0
        /// - Storage: EPROM
        register ANTICLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            zone: uint = 0..8,
        },

        /// Hysteresis loop (steps)
        /// - Min: 0, Max: 32
        /// - Initial: 0
        /// - Storage: EPROM
        register HYSTERESIS_LOOP {
            const ADDRESS = 0x1C;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            hysteresis_loop: uint = 0..8,
        },

        /// Middle offset (2 bytes, signed)
        /// - SMS_STS specific
        /// - Storage: EPROM
        register OFFSET {
            const ADDRESS = 0x1F;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            offset: uint = 0..16,
        },

        /// Operating mode
        /// - 0: Position servo mode
        /// - 1: Wheel/motor mode (continuous rotation)
        /// - SMS_STS specific
        /// - Storage: EPROM
        register MODE {
            const ADDRESS = 0x21;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            mode: uint as enum OperatingMode {
                Position = 0,
                Wheel = 1,
                Unknown = catch_all,
            } = 0..8,
        },

        /// Protection torque (1.0% units)
        /// - Min: 0, Max: 100
        /// - Initial: 20
        /// - Storage: EPROM
        register PROTECTION_TORQUE {
            const ADDRESS = 0x25;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 20;

            torque: uint = 0..8,
        },

        /// Protection time (40ms units)
        /// - Min: 0, Max: 254
        /// - Initial: 100 (4 seconds)
        /// - Storage: EPROM
        register PROTECTION_TIME {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 100;

            time: uint = 0..8,
        },

        /// Overload torque threshold (1.0% units)
        /// - Min: 0, Max: 100
        /// - Initial: 80
        /// - Storage: EPROM
        register OVERLOAD_TORQUE {
            const ADDRESS = 0x27;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            torque: uint = 0..8,
        },

        /// Torque switch
        /// - 0: Disable (damping state)
        /// - 1: Enable
        /// - 2: Free state
        /// - 128: Calibration mode (SMS_STS)
        /// - Storage: SRAM
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

        /// Acceleration parameter
        /// - SMS_STS specific
        /// - Storage: SRAM
        register ACCELERATION {
            const ADDRESS = 0x29;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            acceleration: uint = 0..8,
        },

        /// Target position (steps)
        /// - Min: -32767, Max: 32767 (signed with bit 15)
        /// - Storage: SRAM
        register TARGET_POSITION {
            const ADDRESS = 0x2A;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Goal time (ms)
        /// - Storage: SRAM
        ///
        /// **Position mode:** Movement duration in milliseconds (0-9999)
        /// - How long the servo should take to reach the target position
        ///
        /// **Wheel mode:** Not used (unlike SCSCL which repurposes this for PWM output)
        /// - In wheel mode, use GOAL_SPEED register instead
        ///
        /// Reference: SMS_STS.h defines this as SMS_STS_GOAL_TIME_L (address 44 = 0x2C)
        register GOAL_TIME {
            const ADDRESS = 0x2C;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            time: uint = 0..16,
        },

        /// Goal speed (steps/s)
        /// - Storage: SRAM
        ///
        /// **Position mode:** Maximum speed for position moves (0-1000)
        ///
        /// **Wheel mode:** Signed rotation speed (bit 15 = direction)
        /// - Positive = CW, Negative = CCW
        /// - Encoding: if negative, set bit 15 and use absolute value
        ///
        /// Unlike SCSCL (which uses GOAL_TIME for PWM output), SMS_STS has a dedicated
        /// speed register that serves both modes.
        ///
        /// Reference: SMS_STS.h defines this as SMS_STS_GOAL_SPEED_L (address 46 = 0x2E)
        register GOAL_SPEED {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// Runtime torque limit (0.1% units)
        /// - SMS_STS specific
        /// - Storage: SRAM
        register TORQUE_LIMIT {
            const ADDRESS = 0x30;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            torque: uint = 0..16,
        },

        /// Lock flag (EEPROM write protection)
        /// - 0: Unlocked (EPROM writes saved)
        /// - 1: Locked (EPROM writes not saved)
        /// - Storage: SRAM
        /// - Address: 0x37 for SMS_STS (different from SCSCL 0x30!)
        register LOCK_FLAG {
            const ADDRESS = 0x37;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            locked: bool = 0,
        },

        /// Current position (steps, signed with bit 15)
        /// - Storage: SRAM (read-only)
        register CURRENT_POSITION {
            const ADDRESS = 0x38;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            position: uint = 0..16,
        },

        /// Current speed (steps/s, signed with bit 15)
        /// - Storage: SRAM (read-only)
        register CURRENT_SPEED {
            const ADDRESS = 0x3A;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            speed: uint = 0..16,
        },

        /// Current load (0.1% units, signed with bit 10)
        /// - Storage: SRAM (read-only)
        register CURRENT_LOAD {
            const ADDRESS = 0x3C;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            load: uint = 0..16,
        },

        /// Current voltage (0.1V units)
        /// - Storage: SRAM (read-only)
        register CURRENT_VOLTAGE {
            const ADDRESS = 0x3E;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            voltage: uint = 0..8,
        },

        /// Current temperature (°C)
        /// - Storage: SRAM (read-only)
        register CURRENT_TEMPERATURE {
            const ADDRESS = 0x3F;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            temperature: uint = 0..8,
        },

        /// Asynchronous write flag
        /// - Storage: SRAM (read-only)
        register ASYNCHRONOUS_WRITE_FLAG {
            const ADDRESS = 0x40;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            flag: bool = 0,
        },

        /// Servo status (error flags)
        /// - Storage: SRAM (read-only)
        register SERVO_STATUS {
            const ADDRESS = 0x41;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            voltage: bool = 0,
            temperature: bool = 2,
            overload: bool = 5,
        },

        /// Move flag
        /// - 1: Moving, 0: Stopped
        /// - Storage: SRAM (read-only)
        register MOVE_FLAG {
            const ADDRESS = 0x42;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            flag: bool = 0,
        },

        /// Current draw (mA, signed with bit 15)
        /// - Storage: SRAM (read-only)
        /// - Address: 0x45-0x46
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

