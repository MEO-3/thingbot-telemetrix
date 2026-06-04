use crate::error::Result;
use crate::protocol::{Command, DhtType, Led, Motor, PinMode, Servo};
use crate::report::{Report, decode_report};
use crate::transport::{SerialTransport, Transport};
use std::time::Duration;

pub struct Telemetrix<T = SerialTransport> {
    transport: T,
}

impl Telemetrix<SerialTransport> {
    pub fn connect(port_name: impl AsRef<str>) -> Result<Self> {
        Ok(Self::new(SerialTransport::open(port_name)?))
    }

    pub fn connect_with_settings(
        port_name: impl AsRef<str>,
        baud_rate: u32,
        timeout: Duration,
    ) -> Result<Self> {
        Ok(Self::new(SerialTransport::open_with_settings(
            port_name, baud_rate, timeout,
        )?))
    }
}

impl<T: Transport> Telemetrix<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    pub fn gpio(&mut self) -> GpioClient<'_, T> {
        GpioClient { device: self }
    }

    pub fn thingbot(&mut self) -> ThingBotClient<'_, T> {
        ThingBotClient { device: self }
    }

    pub fn ultrasonic(&mut self) -> UltrasonicClient<'_, T> {
        UltrasonicClient { device: self }
    }

    pub fn send(&mut self, command: Command) -> Result<()> {
        let packet = command.encode()?;
        self.transport.write_packet(&packet)
    }

    pub fn poll_report(&mut self) -> Result<Option<Report>> {
        let Some(packet) = self.transport.read_packet()? else {
            return Ok(None);
        };
        Ok(Some(decode_report(&packet)?))
    }

    pub fn are_you_there(&mut self) -> Result<()> {
        self.send(Command::AreYouThere)
    }

    pub fn set_dht(&mut self, pin: u8, dht_type: DhtType) -> Result<()> {
        self.send(Command::SetDht { pin, dht_type })
    }
}

pub struct GpioClient<'a, T> {
    device: &'a mut Telemetrix<T>,
}

impl<T: Transport> GpioClient<'_, T> {
    pub fn set_output(&mut self, pin: u8) -> Result<()> {
        self.device.send(Command::SetPinMode {
            pin,
            mode: PinMode::Output,
            option: 0,
        })
    }

    pub fn set_input(&mut self, pin: u8, reporting_enabled: bool) -> Result<()> {
        self.device.send(Command::SetPinMode {
            pin,
            mode: PinMode::Input,
            option: u8::from(reporting_enabled),
        })
    }

    pub fn set_input_pullup(&mut self, pin: u8, reporting_enabled: bool) -> Result<()> {
        self.device.send(Command::SetPinMode {
            pin,
            mode: PinMode::InputPullup,
            option: u8::from(reporting_enabled),
        })
    }

    pub fn set_analog_input(&mut self, pin: u8, differential: u16) -> Result<()> {
        self.device
            .send(Command::SetAnalogInput { pin, differential })
    }

    pub fn digital_write(&mut self, pin: u8, value: bool) -> Result<()> {
        self.device.send(Command::DigitalWrite { pin, value })
    }

    pub fn digital_read(&mut self, pin: u8) -> Result<()> {
        self.device.send(Command::DigitalRead { pin })
    }

    pub fn analog_write(&mut self, pin: u8, value: u16) -> Result<()> {
        self.device.send(Command::AnalogWrite { pin, value })
    }

    pub fn analog_read(&mut self, pin: u8) -> Result<()> {
        self.device.send(Command::AnalogRead { pin })
    }
}

pub struct ThingBotClient<'a, T> {
    device: &'a mut Telemetrix<T>,
}

impl<T: Transport> ThingBotClient<'_, T> {
    pub fn dc(&mut self, motor: Motor, speed: u8) -> Result<()> {
        self.device.send(Command::DcWrite { motor, speed })
    }

    pub fn servo(&mut self, servo: Servo, angle: u8) -> Result<()> {
        self.device.send(Command::ServoWrite { servo, angle })
    }

    pub fn buzzer(&mut self, frequency: u8) -> Result<()> {
        self.device.send(Command::BuzzerWrite { frequency })
    }

    pub fn led(&mut self, led: Led, state: u8) -> Result<()> {
        self.device.send(Command::LedWrite { led, state })
    }
}

pub struct UltrasonicClient<'a, T> {
    device: &'a mut Telemetrix<T>,
}

impl<T: Transport> UltrasonicClient<'_, T> {
    pub fn set_pin_mode(&mut self, trigger_pin: u8, echo_pin: u8) -> Result<()> {
        self.device.send(Command::SetUltrasonic {
            trigger_pin,
            echo_pin,
        })
    }

    pub fn read(&mut self) -> Result<()> {
        self.device.send(Command::ReadUltrasonic)
    }
}
