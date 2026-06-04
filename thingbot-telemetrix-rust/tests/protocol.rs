use thingbot_telemetrix_rust::{Command, Error, Packet, ThingBotProtocol, command_id, pin_mode_id};

#[test]
fn encodes_digital_write_packet() {
    let bytes = ThingBotProtocol::encode(&Command::DigitalWrite {
        pin: 7,
        value: true,
    })
    .unwrap();

    assert_eq!(bytes, vec![3, command_id::DIGITAL_WRITE, 7, 1]);
}

#[test]
fn encodes_analog_write_as_two_byte_value() {
    let bytes = ThingBotProtocol::encode(&Command::AnalogWrite {
        pin: 4,
        value: 1023,
    })
    .unwrap();

    assert_eq!(bytes, vec![4, command_id::ANALOG_WRITE, 4, 3, 255]);
}

#[test]
fn encodes_ultrasonic_pin_mode_with_echo_pin_first() {
    let bytes = ThingBotProtocol::encode(&Command::SetUltrasonic {
        trigger_pin: 5,
        echo_pin: 6,
    })
    .unwrap();

    assert_eq!(
        bytes,
        vec![4, command_id::SET_PIN_MODE, 6, pin_mode_id::ULTRASONIC, 5]
    );
}

#[test]
fn rejects_bad_wire_length() {
    let error = Packet::from_wire(&[3, command_id::DIGITAL_WRITE, 7]).unwrap_err();

    assert_eq!(
        error,
        Error::InvalidPacketLength {
            declared: 3,
            actual: 2
        }
    );
}
