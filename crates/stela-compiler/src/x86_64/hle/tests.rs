use super::*;

#[test]
fn names_round_trip() {
    for routine in Routine::PRODUCTION {
        assert_eq!(Routine::from_name(routine.name()), Some(routine));
    }
    assert_eq!(Routine::from_name("bail"), None);
    assert_eq!(Routine::from_name("touch"), None);
    assert_eq!(Routine::from_name("Memmove"), None);
    assert_eq!(Routine::Bail.name(), "bail");
    assert_eq!(Routine::Touch.name(), "touch");
    let bits = Routine::PRODUCTION
        .iter()
        .chain(&[Routine::Bail, Routine::Touch])
        .fold(0, |bits, routine| {
            assert_eq!(bits & routine.bit(), 0, "{routine:?}");
            bits | routine.bit()
        });
    assert_eq!(bits, 0x7f);
}
