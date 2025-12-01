device_driver::create_device!(
    device_name: SclInternal,
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
                //UpperDigit, DigitUpper, DigitLower,
                //LowerDigit, Acronym,
            ];
            type DefmtFeature = "defmt";
        }

        /// Firmware major version number
        /// - Min: 0, Max: 253
        /// - Initial: 0
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        register FW_MAJOR_VERSION {
            const ADDRESS = 0x00;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Firmware major version number
            /// - Min: 0, Max: 253
            version_number: uint = 0..8,
        },

        /// Firmware minor version number
        /// - Min: 0, Max: 253
        /// - Initial: 5
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        register FW_MINOR_VERSION {
            const ADDRESS = 0x01;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 5;

            /// Firmware minor version number
            /// - Min: 0, Max: 253
            version_number: uint = 0..8,
        },

        /// Servo major version number
        /// - Min: 0, Max: 253
        /// - Initial: 5
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        register SERVO_MAJOR_VERSION {
            const ADDRESS = 0x03;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 5;

            /// Servo major version number
            /// - Min: 0, Max: 253
            version_number: uint = 0..8,
        },

        /// Servo minor version number
        /// - Min: 0, Max: 253
        /// - Initial: 15
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        register SERVO_MINOR_VERSION {
            const ADDRESS = 0x04;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            /// Servo minor version number
            /// - Min: 0, Max: 253
            version_number: uint = 0..8,
        },

        /// ID
        /// - Min: 0, Max: 253
        /// - Initial: 1
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// A unique identification code on the bus, with no duplicate ID numbers allowed on the same bus. ID number 254 (0xFE) is the broadcast ID, and broadcasts do not return response packets.
        register ID {
            const ADDRESS = 0x05;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// ID
            /// - Min: 0, Max: 253
            id: uint = 0..8,
        },

        /// Baudrate
        /// - Min: 0, Max: 7
        /// - Initial: 0
        /// - Unit:
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// 0-7 respectively represent baud rates as follows: 1000000, 500000, 250000, 128000, 115200, 76800, 57600 and 38400.
        register BAUDRATE {
            const ADDRESS = 0x06;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Baudrate
            /// - Min: 0, Max: 7
            baudrate: uint as enum BaudRate {
                Baud1000000 = 0,
                Baud500000 = 1,
                Baud250000 = 2,
                Baud128000 = 3,
                Baud115200 = 4,
                Baud76800 = 5,
                Baud57600 = 6,
                Baud38400 = 7,
                Unknown = catch_all,
            } = 0..8,
        },

        /// Return delay
        /// - Min: 0, Max: 254
        /// - Initial: 0
        /// - Unit: 2us
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// The minimum unit is 2us, and settable maximum value for response delay is 254*2=508us
        register RETURN_DELAY {
            const ADDRESS = 0x07;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Return delay
            /// - Min: 0, Max: 254
            /// - Unit: 2us
            delay: uint = 0..8,
        },

        /// Response status level
        /// - Min: 0, Max: 1
        /// - Initial: 1
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// 0: Except for read and PING commands, other commands do not return response packets. 1: Return response packets for all commands
        register RESPONSE_STATUS_LEVEL {
            const ADDRESS = 0x08;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// Response status level
            /// - Min: 0, Max: 1
            level: bool = 0,
        },

        /// Minimum angle
        /// - Min: 0, Max: 1022
        /// - Initial: 20
        /// - Unit: step
        /// - Storage: EPROM
        /// - Bytes: 2
        ///
        /// Set the minimum value limit for the operation angle, which should be smaller than the maximum angle limit. This value is set to 0 in motor mode.
        register MINIMUM_ANGLE {
            const ADDRESS = 0x09;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 20;

            /// Minimum angle
            /// - Min: 0, Max: 1022
            /// - Unit: step
            angle: uint = 0..16,
        },

        /// Maximum angle
        /// - Min: 0, Max: 1023
        /// - Initial: 1003
        /// - Unit: step
        /// - Storage: EPROM
        /// - Bytes: 2
        ///
        /// Set the maximum value limit for the operation angle, which should be greater than the minimum angle limit. This value is set to 0 in motor mode.
        register MAXIMUM_ANGLE {
            const ADDRESS = 0x0B;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1003;

            /// Maximum angle
            /// - Min: 0, Max: 1023
            /// - Unit: step
            angle: uint = 0..16,
        },

        /// Maximum temperature
        /// - Min: 0, Max: 100
        /// - Initial: 80
        /// - Unit: °C
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// The maximum operating temperature limit, when set to 80, means the maximum temperature is 80 degrees Celsius, with a precision setting of 1 degree Celsius.
        register MAXIMUM_TEMPERATURE {
            const ADDRESS = 0x0D;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            /// Maximum temperature
            /// - Min: 0, Max: 100
            /// - Unit: °C
            temperature: uint = 0..8,
        },

        /// Maximum input voltage
        /// - Min: 0, Max: 254
        /// - Initial: 90
        /// - Unit: 0.1V
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// If the maximum input voltage is set to 90, then the maximum operating voltage limit is 9.0V, with a precision of 0.1V.
        register MAXIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0E;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 90;

            /// Maximum input voltage
            /// - Min: 0, Max: 254
            /// - Unit: 0.1V
            voltage: uint = 0..8,
        },

        /// Minimum input voltage
        /// - Min: 0, Max: 254
        /// - Initial: 45
        /// - Unit: 0.1V
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// If the minimum input voltage is set to 50, then the minimum operating voltage limit is 5.0V, with a precision of 0.1V.
        register MINIMUM_INPUT_VOLTAGE {
            const ADDRESS = 0x0F;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 45;

            /// Minimum input voltage
            /// - Min: 0, Max: 254
            /// - Unit: 0.1V
            voltage: uint = 0..8,
        },

        /// Maximum torque
        /// - Min: 0, Max: 1000
        /// - Initial: 1000
        /// - Unit: 0.1%
        /// - Storage: EPROM
        /// - Bytes: 2
        ///
        /// Set the maximum output torque limit for the servo motor, 1000 = 100% * stall torque
        register MAXIMUM_TORQUE {
            const ADDRESS = 0x10;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 1000;

            /// Maximum torque
            /// - Min: 0, Max: 1000
            /// - Unit: 0.1%
            torque: uint = 0..16,
        },

        /// Phase
        /// - Min: 0, Max: 254
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Special function byte, do not modify unless there are specific requirements. Please refer to the special byte bit analysis for further details.
        register PHASE {
            const ADDRESS = 0x12;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Phase
            /// - Min: 0, Max: 254
            phase: uint = 0..8,
        },

        /// Unloading conditions
        /// - Min: 0, Max: 254
        /// - Initial: 36
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 The corresponding bit sets 0 to disable the corresponding protection. Voltage None Temperature None None Overload The corresponding bit sets 1 to enable the corresponding protection.
        register UNLOADING_CONDITIONS {
            const ADDRESS = 0x13;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 36;

            /// Voltage protection
            voltage: bool = 0,
            /// Temperature protection
            temperature: bool = 2,
            /// Overload protection
            overload: bool = 5,
        },

        /// LED alarm condition
        /// - Min: 0, Max: 254
        /// - Initial: 37
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 The corresponding bit sets 1 to enable the flashing light alarm. Voltage None Temperature None None Overload The corresponding bit sets 0 to disable the flashing light alarm.
        register LED_ALARM_CONDITION {
            const ADDRESS = 0x14;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 37;

            /// Voltage alarm
            voltage: bool = 0,
            /// Temperature alarm
            temperature: bool = 2,
            /// Overload alarm
            overload: bool = 5,
        },

        /// P (proportional) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 15
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Proportional coefficient of the control motor
        register P_COEFFICIENT {
            const ADDRESS = 0x15;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            /// P (proportional) coefficient
            /// - Min: 0, Max: 254
            coefficient: uint = 0..8,
        },

        /// D (derivative) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 15
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Differential coefficient of the control motor
        register D_COEFFICIENT {
            const ADDRESS = 0x16;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 15;

            /// D (derivative) coefficient
            /// - Min: 0, Max: 254
            coefficient: uint = 0..8,
        },

        /// I (integral) coefficient
        /// - Min: 0, Max: 254
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Integral coefficient of the control motor
        register I_COEFFICIENT {
            const ADDRESS = 0x17;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            /// I (integral) coefficient
            /// - Min: 0, Max: 254
            coefficient: uint = 0..8,
        },

        /// Minimum starting force
        /// - Min: 0, Max: 1000
        /// - Initial: 30
        /// - Unit: 0.1%
        /// - Storage: EPROM
        /// - Bytes: 2
        ///
        /// Set the minimum output torque limit of the servo, set 1000 = 100% * locked-rotor torque
        register MINIMUM_STARTING_FORCE {
            const ADDRESS = 0x18;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 30;

            /// Minimum starting force
            /// - Min: 0, Max: 1000
            /// - Unit: 0.1%
            force: uint = 0..16,
        },

        /// Clockwise insensitive zone
        /// - Min: 0, Max: 32
        /// - Initial: 1
        /// - Unit: Step
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Minimum unit is a minimum resolution angle
        register CLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1A;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// Clockwise insensitive zone
            /// - Min: 0, Max: 32
            /// - Unit: Step
            zone: uint = 0..8,
        },

        /// Anticlockwise insensitive zone
        /// - Min: 0, Max: 32
        /// - Initial: 1
        /// - Unit: Step
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Minimum unit is a minimum resolution angle
        register ANTICLOCKWISE_INSENSITIVE_ZONE {
            const ADDRESS = 0x1B;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// Anticlockwise insensitive zone
            /// - Min: 0, Max: 32
            /// - Unit: Step
            zone: uint = 0..8,
        },

        /// Hysteresis loop
        /// - Min: 0, Max: 32
        /// - Initial: 1
        /// - Unit: Step
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Minimum unit is a minimum resolution angle
        register HYSTERESIS_LOOP {
            const ADDRESS = 0x1C;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// Hysteresis loop
            /// - Min: 0, Max: 32
            /// - Unit: Step
            hysteresis_loop: uint = 0..8,
        },

        /// Protection torque
        /// - Min: 0, Max: 100
        /// - Initial: 20
        /// - Unit: 1.0%
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// Output torque after entering overload protection, e.g. set 20 means 20% of the maximum torque.
        register PROTECTION_TORQUE {
            const ADDRESS = 0x25;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 20;

            /// Protection torque
            /// - Min: 0, Max: 100
            /// - Unit: 1.0%
            torque: uint = 0..8,
        },

        /// Protection time
        /// - Min: 0, Max: 254
        /// - Initial: 100
        /// - Unit: 40ms
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// The current load output exceeds the overload torque and maintain the timing length, such as set 100 means 4 seconds, the maximum can be set to 10 seconds.
        register PROTECTION_TIME {
            const ADDRESS = 0x26;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 100;

            /// Protection time
            /// - Min: 0, Max: 254
            /// - Unit: 40ms
            time: uint = 0..8,
        },

        /// Overload torque
        /// - Min: 0, Max: 100
        /// - Initial: 80
        /// - Unit: 1.0%
        /// - Storage: EPROM
        /// - Bytes: 1
        ///
        /// The maximum torque threshold for initiating the overload protection time countdown. For example, if set to 80, it represents 80% of the maximum torque.
        register OVERLOAD_TORQUE {
            const ADDRESS = 0x27;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 80;

            /// Overload torque
            /// - Min: 0, Max: 100
            /// - Unit: 1.0%
            torque: uint = 0..8,
        },

        /// Torque switch
        /// - Min: 0, Max: 3
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// 0: Disables torque output/damping state. 1: Enables torque output. 2: Writes the free state.
        register TORQUE_SWITCH {
            const ADDRESS = 0x28;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Torque switch
            /// - Min: 0, Max: 3
            mode: uint as enum TorqueMode {
                /// Disable torque output/damping state
                Disable = 0,
                /// Enable torque output
                Enable = 1,
                /// Write the free state
                Free = 2,
                /// Unexpected value. This is a bug!
                Unknown = catch_all,
            } = 0..8,
        },

        /// Target position
        /// - Min: 0, Max: 1023
        /// - Initial: 0
        /// - Unit: Step
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// Each step corresponds to the value of the minimum resolution angle, and it is used in absolute position control mode. The maximum value should be the maximum effective angle.
        register TARGET_POSITION {
            const ADDRESS = 0x2A;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Target position
            /// - Min: 0, Max: 1023
            /// - Unit: Step
            position: uint = 0..10,
        },

        /// Operation time
        /// - Min: 0, Max: 9999
        /// - Initial: 0
        /// - Unit: 1ms
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// The movement time from the current position to the target position, takes effect when the running speed is 0.
        register OPERATION_TIME {
            const ADDRESS = 0x2C;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Operation time
            /// - Min: 0, Max: 9999
            /// - Unit: 1ms
            time: uint = 0..16,
        },

        /// Operation speed
        /// - Min: 0, Max: 1000
        /// - Initial: 0
        /// - Unit: step/s
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// The number of steps moved per unit time (per second).
        register OPERATION_SPEED {
            const ADDRESS = 0x2E;
            const SIZE_BITS = 16;
            type Access = RW;
            const RESET_VALUE = 0;

            /// Operation speed
            /// - Min: 0, Max: 1000
            /// - Unit: step/s
            speed: uint = 0..16,
        },

        /// Lock flag
        /// - Min: 0, Max: 1
        /// - Initial: 1
        /// - Unit: None
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// Writing 0 disables the write lock, allowing the values written to EPROM addresses to be saved even when power is lost. Writing 1 enables the write lock, preventing the values written to EPROM addresses from being saved when power is lost.
        register LOCK_FLAG {
            const ADDRESS = 0x30;
            const SIZE_BITS = 8;
            type Access = RW;
            const RESET_VALUE = 1;

            /// Lock flag
            /// - Min: 0, Max: 1
            locked: bool = 0,
        },

        /// Current position
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: Step
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// Feedback the number of steps in the current position, each step is the value of the minimum resolution angle; Absolute position control mode, the maximum value should be the maximum effective angle.
        register CURRENT_POSITION {
            const ADDRESS = 0x38;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Current position
            /// - Unit: Step
            position: uint = 0..16,
        },

        /// Current speed
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: step/s
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// Feedback the current speed of motor rotation, the number of steps of movement per unit time (per second).
        register CURRENT_SPEED {
            const ADDRESS = 0x3A;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Current speed
            /// - Unit: step/s
            speed: uint = 0..16,
        },

        /// Current load
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: 0.1%
        /// - Storage: SRAM
        /// - Bytes: 2
        ///
        /// The voltage duty cycle of the current control output driving motor.
        register CURRENT_LOAD {
            const ADDRESS = 0x3C;
            const SIZE_BITS = 16;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Current load
            /// - Unit: 0.1%
            load: uint = 0..16,
        },

        /// Current voltage
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: 0.1V
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// Current servo operation voltage
        register CURRENT_VOLTAGE {
            const ADDRESS = 0x3E;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Current voltage
            /// - Unit: 0.1V
            voltage: uint = 0..8,
        },

        /// Current temperature
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: °C
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// Current internal operating circuit of the servo
        register CURRENT_TEMPERATURE {
            const ADDRESS = 0x3F;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Current temperature
            /// - Unit: °C
            temperature: uint = 0..8,
        },

        /// Asynchronous write flag
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// The flag bit for using asynchronous write instructions.
        register ASYNCHRONOUS_WRITE_FLAG {
            const ADDRESS = 0x40;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Asynchronous write flag
            flag: bool = 0,
        },

        /// Servo status
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 The corresponding bit is set to 1 to indicate the corresponding error. Voltage None Temperature None None Overload The corresponding bit 0 means no phase error.
        register SERVO_STATUS {
            const ADDRESS = 0x41;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Voltage error
            voltage: bool = 0,
            /// Temperature error
            temperature: bool = 2,
            /// Overload error
            overload: bool = 5,
        },

        /// Move flag
        /// - Min: -1, Max: -1
        /// - Initial: 0
        /// - Unit: None
        /// - Storage: SRAM
        /// - Bytes: 1
        ///
        /// The flag is 1 when the servo is in motion and 0 when the servo is stopped.
        register MOVE_FLAG {
            const ADDRESS = 0x42;
            const SIZE_BITS = 8;
            type Access = RO;
            const RESET_VALUE = 0;

            /// Move flag
            flag: bool = 0,
        },
    }
);
pub const TARGET_POSITION_ADDR: u8 = 0x2A;
pub const CURRENT_POSITION_ADDR: u8 = 0x38;
