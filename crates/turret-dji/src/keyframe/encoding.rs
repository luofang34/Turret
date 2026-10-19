use crate::checksum;

pub(super) fn frames(first_sequence: u16) -> [[u8; 32]; 3] {
    [
        frame(first_sequence, 0x24),
        frame(first_sequence.wrapping_add(1), 0x24),
        frame(first_sequence.wrapping_add(2), 0x04),
    ]
}

fn frame(sequence: u16, request: u8) -> [u8; 32] {
    // Mux port 22345 carries the complete 24-byte DUML frame.
    let mut bytes = [0_u8; 32];
    bytes[..8].copy_from_slice(&[0x55, 0xcc, 0x49, 0x57, 24, 0, 0, 0]);
    let sequence = sequence.to_le_bytes();
    let duml = &mut bytes[8..];
    duml[..11].copy_from_slice(&[0x55, 24, 4, 0, 2, 0x0e, sequence[0], sequence[1], 0, 1, 1]);
    duml[3] = checksum::header(&duml[..3]);
    duml[16] = request;
    duml[20] = request;
    let crc = checksum::frame(&duml[..22]).to_le_bytes();
    duml[22..].copy_from_slice(&crc);
    bytes
}
