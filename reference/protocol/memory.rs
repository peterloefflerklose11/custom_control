macro_rules! define_register_enum {
    ($enum_name:ident, $($variant:ident => ($addr:expr, $size:expr, $desc:expr)),* $(,)?) => {
        #[allow(dead_code)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum $enum_name {
            $($variant),*
        }

        #[allow(dead_code)]
        impl $enum_name {
            pub const fn address(&self) -> u8 {
                match *self {
                    $(Self::$variant => $addr),*
                }
            }

            pub const fn size(&self) -> u8 {
                match *self {
                    $(Self::$variant => $size),*
                }
            }

            pub const fn description(&self) -> &'static str {
                match *self {
                    $(Self::$variant => $desc),*
                }
            }

            pub const fn name(&self) -> &'static str {
                match *self {
                    $(Self::$variant => stringify!($variant)),*
                }
            }

            pub fn iter() -> impl Iterator<Item = Self> {
                [
                    $(Self::$variant),*
                ].iter().copied()
            }
        }
    };
}

define_register_enum!(
    EepromRegister,
    ModelNumber => (0x00, 2, "Model Number"),
    FirmwareVersion => (0x02, 1, "Firmware Version"),
    ID => (0x05, 1, "Servo ID"),
    BaudRate => (0x06, 1, "Baud Rate"),
    ReturnDelay => (0x07, 1, "Return Delay Time"),
    ResponseStatus => (0x08, 1, "Response Status Level"),
    MinAngleLimit => (0x09, 2, "Minimum Angle Limit"),
    MaxAngleLimit => (0x0B, 2, "Maximum Angle Limit"),
    MaxTemperature => (0x0D, 1, "Maximum Temperature Limit"),
    MaxVoltage => (0x0E, 1, "Maximum Voltage Limit"),
    MinVoltage => (0x0F, 1, "Minimum Voltage Limit"),
    MaxTorque => (0x10, 2, "Maximum Torque"),
    UnloadCondition => (0x12, 1, "Unload Condition"),
    LedAlarm => (0x13, 1, "LED Alarm Condition"),
    PCoef => (0x15, 1, "P Coefficient"),
    DCoef => (0x16, 1, "D Coefficient"),
    ICoef => (0x17, 1, "I Coefficient"),
    MinStartupForce => (0x18, 2, "Minimum Startup Force"),
    CwDeadZone => (0x1A, 1, "Clockwise Dead Zone"),
    CcwDeadZone => (0x1B, 1, "Counter-Clockwise Dead Zone"),
    ProtectionCurrent => (0x1C, 2, "Protection Current"),
    AngularResolution => (0x1E, 1, "Angular Resolution"),
    Offset => (0x1F, 2, "Position Offset"),
    Mode => (0x21, 1, "Servo Mode"),
    ProtectionTorque => (0x22, 1, "Protection Torque"),
    ProtectionTime => (0x23, 1, "Protection Time"),
    OverloadTorque => (0x24, 1, "Overload Torque"),
    SpeedClosedLoopP => (0x25, 1, "Speed Closed-Loop P Coefficient"),
    OverCurrentProtectionTime => (0x26, 1, "Over-Current Protection Time"),
    VelocityClosedLoopI => (0x27, 1, "Velocity Closed-Loop I Coefficient"),
);

define_register_enum!(
    RamRegister,
    TorqueEnable => (0x28, 1, "Torque Enable"),
    Acc => (0x29, 1, "Acceleration"),
    GoalPosition => (0x2A, 2, "Goal Position"),
    GoalTime => (0x2C, 2, "Goal Time"),
    GoalSpeed => (0x2E, 2, "Goal Speed"),
    TorqueLimit => (0x30, 2, "Torque Limit"),
    Lock => (0x37, 1, "Lock EEPROM"),
    PresentPosition => (0x38, 2, "Present Position"),
    PresentSpeed => (0x3A, 2, "Present Speed"),
    PresentLoad => (0x3C, 2, "Present Load"),
    PresentVoltage => (0x3E, 1, "Present Voltage"),
    PresentTemperature => (0x3F, 1, "Present Temperature"),
    Status => (0x41, 1, "Motor Status"),
    Moving => (0x42, 1, "Moving Status"),
    PresentCurrent => (0x45, 2, "Present Current"),
);
