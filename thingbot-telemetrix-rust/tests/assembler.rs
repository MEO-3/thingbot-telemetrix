use thingbot_telemetrix_rust::{Error, PacketAssembler};

#[test]
fn assembles_packet_from_single_chunk() {
    let mut assembler = PacketAssembler::new();
    assembler.push(&[3, 102, 0, 1]);

    let packet = assembler.next_packet().unwrap().unwrap();
    assert_eq!(packet.payload(), &[102, 0, 1]);
    assert!(assembler.next_packet().unwrap().is_none());
}

#[test]
fn assembles_packet_split_across_chunks() {
    // a DHT report split the way 20-byte BLE notifications could split it
    let wire = [6, 11, 2, 0x11, 0xC6, 0x08, 0x4D];
    let mut assembler = PacketAssembler::new();

    let (first, second) = wire.split_at(3);
    assembler.push(first);
    assert!(assembler.next_packet().unwrap().is_none());

    assembler.push(second);
    let packet = assembler.next_packet().unwrap().unwrap();
    assert_eq!(packet.payload(), &wire[1..]);
}

#[test]
fn assembles_multiple_packets_from_one_chunk() {
    let mut assembler = PacketAssembler::new();
    assembler.push(&[2, 6, 1, 3, 2, 8, 1]);

    assert_eq!(assembler.next_packet().unwrap().unwrap().payload(), &[6, 1]);
    assert_eq!(
        assembler.next_packet().unwrap().unwrap().payload(),
        &[2, 8, 1]
    );
    assert!(assembler.next_packet().unwrap().is_none());
}

#[test]
fn byte_at_a_time_delivery() {
    let wire = [3, 102, 0, 0];
    let mut assembler = PacketAssembler::new();
    for &byte in &wire[..wire.len() - 1] {
        assembler.push(&[byte]);
        assert!(assembler.next_packet().unwrap().is_none());
    }
    assembler.push(&[wire[wire.len() - 1]]);
    assert_eq!(
        assembler.next_packet().unwrap().unwrap().payload(),
        &wire[1..]
    );
}

#[test]
fn zero_length_frame_is_consumed_and_reported() {
    let mut assembler = PacketAssembler::new();
    assembler.push(&[0, 3, 102, 0, 1]);

    assert_eq!(assembler.next_packet(), Err(Error::EmptyPacket));
    // the bad frame is gone; the following packet still parses
    assert_eq!(
        assembler.next_packet().unwrap().unwrap().payload(),
        &[102, 0, 1]
    );
}

#[test]
fn empty_assembler_yields_none() {
    let mut assembler = PacketAssembler::new();
    assert!(assembler.next_packet().unwrap().is_none());
}
