use thingbot_telemetrix_rust::{Packet, Result, Transport};

#[derive(Debug, Default, Clone)]
pub struct TestTransport {
    written: Vec<Vec<u8>>,
    incoming: Vec<Vec<u8>>,
}

impl TestTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_incoming<I>(incoming: I) -> Self
    where
        I: IntoIterator<Item = Vec<u8>>,
    {
        Self {
            written: Vec::new(),
            incoming: incoming.into_iter().collect(),
        }
    }

    pub fn written(&self) -> &[Vec<u8>] {
        &self.written
    }
}

impl Transport for TestTransport {
    fn write_packet(&mut self, packet: &Packet) -> Result<()> {
        self.written.push(packet.to_wire());
        Ok(())
    }

    fn read_packet(&mut self) -> Result<Option<Packet>> {
        if self.incoming.is_empty() {
            return Ok(None);
        }
        let bytes = self.incoming.remove(0);
        Ok(Some(Packet::from_wire(&bytes)?))
    }
}
