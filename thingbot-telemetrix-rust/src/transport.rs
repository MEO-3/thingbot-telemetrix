use crate::error::Result;
use crate::protocol::Packet;
use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

pub trait Transport {
    fn write_packet(&mut self, packet: &Packet) -> Result<()>;
    fn read_packet(&mut self) -> Result<Option<Packet>>;
}

pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    pub const DEFAULT_BAUD_RATE: u32 = 115_200;
    pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

    pub fn open(port_name: impl AsRef<str>) -> Result<Self> {
        Self::open_with_settings(port_name, Self::DEFAULT_BAUD_RATE, Self::DEFAULT_TIMEOUT)
    }

    pub fn open_with_settings(
        port_name: impl AsRef<str>,
        baud_rate: u32,
        timeout: Duration,
    ) -> Result<Self> {
        let port = serialport::new(port_name.as_ref(), baud_rate)
            .timeout(timeout)
            .open()?;
        Ok(Self { port })
    }

    pub fn from_port(port: Box<dyn serialport::SerialPort>) -> Self {
        Self { port }
    }
}

impl Transport for SerialTransport {
    fn write_packet(&mut self, packet: &Packet) -> Result<()> {
        self.port.write_all(&packet.to_wire())?;
        self.port.flush()?;
        Ok(())
    }

    fn read_packet(&mut self) -> Result<Option<Packet>> {
        let mut length = [0_u8; 1];
        match self.port.read_exact(&mut length) {
            Ok(()) => {}
            Err(error)
                if matches!(
                    error.kind(),
                    ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::UnexpectedEof
                ) =>
            {
                return Ok(None);
            }
            Err(error) => return Err(error.into()),
        }

        let mut payload = vec![0_u8; length[0] as usize];
        self.port.read_exact(&mut payload)?;

        let mut wire = Vec::with_capacity(payload.len() + 1);
        wire.push(length[0]);
        wire.extend_from_slice(&payload);
        Ok(Some(Packet::from_wire(&wire)?))
    }
}
