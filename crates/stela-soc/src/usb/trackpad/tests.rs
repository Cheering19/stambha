use super::*;
use alloc::string::ToString;

#[test]
fn the_serial_is_the_machine_s_and_no_ecid() {
    let a = serial(0x0011_2233_4455_6677);
    assert_eq!(a, serial(0x0011_2233_4455_6677), "stable");
    assert_ne!(a, serial(0x0011_2233_4455_6678));
    assert_eq!(a.len(), 12);
    assert!(a.starts_with("STELA"));
    assert!(a[5..].bytes().all(|b| b.is_ascii_hexdigit()));
    assert!(!a.contains("2233"), "no digits of the ECID");
}

#[test]
fn identities_are_apple_s_ids() {
    assert_eq!(VENDOR, 0x05ac);
    assert_eq!(Identity::MagicTrackpad2.product(), 0x0265);
    assert_eq!(Identity::Builtin.product(), 0x0281);
}

#[test]
fn events_read_as_log_lines() {
    for (event, line) in [
        (TrackpadEvent::Configured(1), "configured 1"),
        (TrackpadEvent::Mode(1), "mode 1"),
        (TrackpadEvent::Feature(0xd9), "feature 0xd9"),
        (
            TrackpadEvent::Query {
                id: 0xd3,
                answered: true,
            },
            "query 0xd3",
        ),
        (
            TrackpadEvent::Query {
                id: 0x42,
                answered: false,
            },
            "query 0x42 refused",
        ),
        (
            TrackpadEvent::Unknown {
                request_type: 0xa1,
                request: 0x01,
                value: 0x0342,
                index: 1,
            },
            "unknown request_type=0xa1 request=0x01 value=0x0342 index=1",
        ),
        (TrackpadEvent::Dropped(3), "dropped 3 frames"),
    ] {
        assert_eq!(event.to_string(), line);
    }
}
