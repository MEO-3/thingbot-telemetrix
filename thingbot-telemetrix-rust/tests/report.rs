use thingbot_telemetrix_rust::{Packet, Report, decode_report, report_id};

#[test]
fn decodes_dht_report() {
    let packet = Packet::from_wire(&[6, report_id::DHT_REPORT, 7, 0x12, 0x34, 0x09, 0xc4]).unwrap();

    let report = decode_report(&packet).unwrap();

    assert_eq!(
        report,
        Report::Dht {
            pin: 7,
            humidity_percent: 46.6,
            temperature_celsius: 25.0,
        }
    );
}

#[test]
fn decodes_switch_pressed_from_pullup_low() {
    let packet = Packet::from_wire(&[3, report_id::THINGBOT_SW_REPORT, 3, 0]).unwrap();

    let report = decode_report(&packet).unwrap();

    assert_eq!(
        report,
        Report::ThingBotSwitch {
            pin: 3,
            pressed: true
        }
    );
}
