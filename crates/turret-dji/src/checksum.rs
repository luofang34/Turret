/// Compute the DUML header CRC-8.
pub fn header(bytes: &[u8]) -> u8 {
    let mut crc = 0x77_u8;
    for byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 == 1 { 0x8c } else { 0 };
        }
    }
    crc
}

/// Compute the DUML frame CRC-16.
pub fn frame(bytes: &[u8]) -> u16 {
    let mut crc = 0x3692_u16;
    for byte in bytes {
        crc ^= u16::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 == 1 { 0x8408 } else { 0 };
        }
    }
    crc
}

#[cfg(test)]
mod tests;
