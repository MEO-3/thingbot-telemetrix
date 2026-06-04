use crate::error::{Error, Result};
use crate::protocol::{Packet, report_id};

#[derive(Debug, Clone, PartialEq)]
pub enum Report {
    Digital {
        pin: u8,
        value: bool,
    },
    Analog {
        pin: u8,
        value: u16,
    },
    IAmHere {
        arduino_id: u8,
    },
    Dht {
        pin: u8,
        temperature_celsius: f32,
        humidity_percent: f32,
    },
    Ultrasonic {
        echo_pin: u8,
        trigger_pin: u8,
        distance_cm: u16,
    },
    ThingBotSwitch {
        pin: u8,
        pressed: bool,
    },
    DebugPrint {
        debug_id: u8,
        value: u16,
    },
}

pub fn decode_report(packet: &Packet) -> Result<Report> {
    let payload = packet.payload();
    let Some((&report_id, data)) = payload.split_first() else {
        return Err(Error::EmptyPacket);
    };

    match report_id {
        report_id::DIGITAL_REPORT => {
            expect_len(report_id, data, 2, "2")?;
            Ok(Report::Digital {
                pin: data[0],
                value: data[1] != 0,
            })
        }
        report_id::ANALOG_REPORT => {
            expect_len(report_id, data, 3, "3")?;
            Ok(Report::Analog {
                pin: data[0],
                value: u16_from_bytes(data[1], data[2]),
            })
        }
        report_id::I_AM_HERE => {
            expect_len(report_id, data, 1, "1")?;
            Ok(Report::IAmHere {
                arduino_id: data[0],
            })
        }
        report_id::DHT_REPORT => {
            expect_len(report_id, data, 5, "5")?;
            Ok(Report::Dht {
                pin: data[0],
                humidity_percent: u16_from_bytes(data[1], data[2]) as f32 / 100.0,
                temperature_celsius: u16_from_bytes(data[3], data[4]) as f32 / 100.0,
            })
        }
        report_id::ULTRASONIC_REPORT => {
            expect_len(report_id, data, 4, "4")?;
            Ok(Report::Ultrasonic {
                echo_pin: data[0],
                trigger_pin: data[1],
                distance_cm: u16_from_bytes(data[2], data[3]),
            })
        }
        report_id::THINGBOT_SW_REPORT => {
            expect_len(report_id, data, 2, "2")?;
            Ok(Report::ThingBotSwitch {
                pin: data[0],
                pressed: data[1] == 0,
            })
        }
        report_id::DEBUG_PRINT => {
            expect_len(report_id, data, 3, "3")?;
            Ok(Report::DebugPrint {
                debug_id: data[0],
                value: u16_from_bytes(data[1], data[2]),
            })
        }
        other => Err(Error::UnknownReport(other)),
    }
}

fn expect_len(
    report_id: u8,
    data: &[u8],
    expected: usize,
    expected_label: &'static str,
) -> Result<()> {
    if data.len() != expected {
        return Err(Error::InvalidReportLength {
            report_id,
            expected: expected_label,
            actual: data.len(),
        });
    }
    Ok(())
}

const fn u16_from_bytes(high: u8, low: u8) -> u16 {
    ((high as u16) << 8) | low as u16
}
