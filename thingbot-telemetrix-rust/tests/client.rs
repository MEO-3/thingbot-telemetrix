mod common;

use common::TestTransport;
use thingbot_telemetrix_rust::{Report, Telemetrix, command_id, report_id};

#[test]
fn gpio_client_writes_expected_packet() {
    let transport = TestTransport::new();
    let mut device = Telemetrix::new(transport);

    device.gpio().digital_write(7, true).unwrap();
    let transport = device.into_transport();

    assert_eq!(
        transport.written(),
        &[vec![3, command_id::DIGITAL_WRITE, 7, 1]]
    );
}

#[test]
fn polls_report_from_transport() {
    let transport = TestTransport::with_incoming([vec![2, report_id::I_AM_HERE, 1]]);
    let mut device = Telemetrix::new(transport);

    let report = device.poll_report().unwrap();

    assert_eq!(report, Some(Report::IAmHere { arduino_id: 1 }));
}
