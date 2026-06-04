use crate::error::{Error, Result};

pub mod command_id {
    pub const SERIAL_LOOP_BACK: u8 = 0;
    pub const SET_PIN_MODE: u8 = 1;
    pub const DIGITAL_WRITE: u8 = 2;
    pub const DIGITAL_READ: u8 = 3;
    pub const ANALOG_WRITE: u8 = 4;
    pub const ANALOG_READ: u8 = 5;
    pub const ARE_YOU_THERE: u8 = 6;
    pub const READ_ULTRASONIC: u8 = 7;

    pub const DC_WRITE: u8 = 101;
    pub const SERVO_WRITE: u8 = 102;
    pub const BUZZER_WRITE: u8 = 103;
    pub const LED_WRITE: u8 = 104;
}

pub mod report_id {
    use super::command_id;

    pub const DIGITAL_REPORT: u8 = command_id::DIGITAL_WRITE;
    pub const ANALOG_REPORT: u8 = command_id::ANALOG_WRITE;
    pub const I_AM_HERE: u8 = command_id::ARE_YOU_THERE;
    pub const ULTRASONIC_REPORT: u8 = command_id::READ_ULTRASONIC;
    pub const DHT_REPORT: u8 = 11;
    pub const THINGBOT_SW_REPORT: u8 = 102;
    pub const DEBUG_PRINT: u8 = 99;
}

pub mod pin_mode_id {
    pub const INPUT: u8 = 0x01;
    pub const OUTPUT: u8 = 0x03;
    pub const INPUT_PULLUP: u8 = 0x05;
    pub const ANALOG: u8 = 0x07;
    pub const DHT: u8 = 0x11;
    pub const ULTRASONIC: u8 = 0x12;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
    Input,
    Output,
    InputPullup,
    Analog,
    Dht,
    Ultrasonic,
}

impl PinMode {
    pub const fn id(self) -> u8 {
        match self {
            Self::Input => pin_mode_id::INPUT,
            Self::Output => pin_mode_id::OUTPUT,
            Self::InputPullup => pin_mode_id::INPUT_PULLUP,
            Self::Analog => pin_mode_id::ANALOG,
            Self::Dht => pin_mode_id::DHT,
            Self::Ultrasonic => pin_mode_id::ULTRASONIC,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DhtType {
    Dht11,
    Dht22,
}

impl DhtType {
    pub const fn id(self) -> u8 {
        match self {
            Self::Dht11 => 11,
            Self::Dht22 => 22,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Motor {
    One,
    Two,
    Three,
    Four,
}

impl Motor {
    pub const fn id(self) -> u8 {
        match self {
            Self::One => 1,
            Self::Two => 2,
            Self::Three => 3,
            Self::Four => 4,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Servo {
    One,
    Two,
    Three,
    Four,
    Five,
}

impl Servo {
    pub const fn id(self) -> u8 {
        match self {
            Self::One => 1,
            Self::Two => 2,
            Self::Three => 3,
            Self::Four => 4,
            Self::Five => 5,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Led {
    One,
    Two,
}

impl Led {
    pub const fn id(self) -> u8 {
        match self {
            Self::One => 1,
            Self::Two => 2,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    payload: Vec<u8>,
}

impl Packet {
    pub fn new(payload: Vec<u8>) -> Result<Self> {
        if payload.is_empty() {
            return Err(Error::EmptyPacket);
        }
        if payload.len() > u8::MAX as usize {
            return Err(Error::ValueOutOfRange {
                field: "packet payload length",
                min: 1,
                max: u8::MAX as i32,
                actual: payload.len() as i32,
            });
        }
        Ok(Self { payload })
    }

    pub fn from_wire(bytes: &[u8]) -> Result<Self> {
        let Some((&declared, payload)) = bytes.split_first() else {
            return Err(Error::EmptyPacket);
        };
        if declared as usize != payload.len() {
            return Err(Error::InvalidPacketLength {
                declared,
                actual: payload.len(),
            });
        }
        Self::new(payload.to_vec())
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    pub fn to_wire(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.payload.len() + 1);
        bytes.push(self.payload.len() as u8);
        bytes.extend_from_slice(&self.payload);
        bytes
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    SerialLoopBack(u8),
    SetPinMode { pin: u8, mode: PinMode, option: u8 },
    SetAnalogInput { pin: u8, differential: u16 },
    DigitalWrite { pin: u8, value: bool },
    DigitalRead { pin: u8 },
    AnalogWrite { pin: u8, value: u16 },
    AnalogRead { pin: u8 },
    AreYouThere,
    SetDht { pin: u8, dht_type: DhtType },
    SetUltrasonic { trigger_pin: u8, echo_pin: u8 },
    ReadUltrasonic,
    DcWrite { motor: Motor, speed: u8 },
    ServoWrite { servo: Servo, angle: u8 },
    BuzzerWrite { frequency: u8 },
    LedWrite { led: Led, state: u8 },
}

impl Command {
    pub fn encode(&self) -> Result<Packet> {
        let payload = match *self {
            Self::SerialLoopBack(value) => vec![command_id::SERIAL_LOOP_BACK, value],
            Self::SetPinMode { pin, mode, option } => {
                vec![command_id::SET_PIN_MODE, pin, mode.id(), option]
            }
            Self::SetAnalogInput { pin, differential } => vec![
                command_id::SET_PIN_MODE,
                pin,
                PinMode::Analog.id(),
                high_byte(differential),
                low_byte(differential),
            ],
            Self::DigitalWrite { pin, value } => {
                vec![command_id::DIGITAL_WRITE, pin, u8::from(value)]
            }
            Self::DigitalRead { pin } => vec![command_id::DIGITAL_READ, pin],
            Self::AnalogWrite { pin, value } => vec![
                command_id::ANALOG_WRITE,
                pin,
                high_byte(value),
                low_byte(value),
            ],
            Self::AnalogRead { pin } => vec![command_id::ANALOG_READ, pin],
            Self::AreYouThere => vec![command_id::ARE_YOU_THERE],
            Self::SetDht { pin, dht_type } => {
                vec![
                    command_id::SET_PIN_MODE,
                    pin,
                    PinMode::Dht.id(),
                    dht_type.id(),
                ]
            }
            Self::SetUltrasonic {
                trigger_pin,
                echo_pin,
            } => vec![
                command_id::SET_PIN_MODE,
                echo_pin,
                PinMode::Ultrasonic.id(),
                trigger_pin,
            ],
            Self::ReadUltrasonic => vec![command_id::READ_ULTRASONIC],
            Self::DcWrite { motor, speed } => {
                vec![
                    command_id::DC_WRITE,
                    motor.id(),
                    bounded_u8("speed", speed, 0, 100)?,
                ]
            }
            Self::ServoWrite { servo, angle } => vec![
                command_id::SERVO_WRITE,
                servo.id(),
                bounded_u8("angle", angle, 0, 180)?,
            ],
            Self::BuzzerWrite { frequency } => vec![command_id::BUZZER_WRITE, frequency],
            Self::LedWrite { led, state } => {
                vec![
                    command_id::LED_WRITE,
                    led.id(),
                    bounded_u8("state", state, 0, 100)?,
                ]
            }
        };
        Packet::new(payload)
    }
}

pub struct ThingBotProtocol;

impl ThingBotProtocol {
    pub fn encode(command: &Command) -> Result<Vec<u8>> {
        Ok(command.encode()?.to_wire())
    }
}

const fn high_byte(value: u16) -> u8 {
    (value >> 8) as u8
}

const fn low_byte(value: u16) -> u8 {
    (value & 0xff) as u8
}

fn bounded_u8(field: &'static str, value: u8, min: u8, max: u8) -> Result<u8> {
    if value < min || value > max {
        return Err(Error::ValueOutOfRange {
            field,
            min: min as i32,
            max: max as i32,
            actual: value as i32,
        });
    }
    Ok(value)
}
