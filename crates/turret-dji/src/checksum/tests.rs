use super::{frame, header};
use crate::capture::RC_N1C;

#[test]
fn independently_published_capture_matches_both_crcs() {
    assert_eq!(header(&RC_N1C[..3]), 0x33);
    assert_eq!(frame(&RC_N1C[..11]), 0x2474);
    assert_eq!(frame(&RC_N1C), 0);
}

#[test]
fn empty_checksums_preserve_protocol_seeds() {
    assert_eq!(header(&[]), 0x77);
    assert_eq!(frame(&[]), 0x3692);
}
