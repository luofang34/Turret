use crate::{ParseStats, checksum, stream::StreamBuffer};

const MIN_FRAME: usize = 13;
const MAX_FRAME: usize = 1023;

/// Raw DUML routing byte, retaining unrecognized device types and indexes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RouteAddress(pub u8);

impl RouteAddress {
    /// Low five bits identifying the device type.
    pub fn device_type(self) -> u8 {
        self.0 & 0x1f
    }

    /// High three bits identifying the device instance.
    pub fn index(self) -> u8 {
        self.0 >> 5
    }
}

/// Complete frame with validated CRCs, retaining the exact wire bytes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DumlFrame {
    raw: [u8; MAX_FRAME],
    length: usize,
}

impl DumlFrame {
    /// Original bytes including both checksums.
    pub fn raw(&self) -> &[u8] {
        &self.raw[..self.length]
    }

    /// Six-bit wire version; unknown versions are retained without semantic decoding.
    pub fn version(&self) -> u8 {
        self.raw[2] >> 2
    }

    /// Encoded source address.
    pub fn sender(&self) -> RouteAddress {
        RouteAddress(self.raw[4])
    }

    /// Encoded destination address.
    pub fn receiver(&self) -> RouteAddress {
        RouteAddress(self.raw[5])
    }

    /// Little-endian sequence number, without assumptions about request matching.
    pub fn sequence(&self) -> u16 {
        u16::from_le_bytes([self.raw[6], self.raw[7]])
    }

    /// Raw command flags, including direction, acknowledgment and encryption bits.
    pub fn flags(&self) -> u8 {
        self.raw[8]
    }

    /// Command namespace, including unknown values.
    pub fn command_set(&self) -> u8 {
        self.raw[9]
    }

    /// Command identifier, including unknown values.
    pub fn command_id(&self) -> u8 {
        self.raw[10]
    }

    /// Undecoded payload, excluding header and CRC-16 trailer.
    pub fn payload(&self) -> &[u8] {
        &self.raw[11..self.length - 2]
    }
}

/// Incremental DUML scanner with at most 1023 buffered bytes per stream.
///
/// Supply each direction and mux port to a separate parser. CRCs validate framing,
/// not authenticity. A plausible incomplete header waits for more bytes until
/// [`Self::finish`] marks a stream boundary.
pub struct DumlParser {
    buffer: StreamBuffer,
}

impl Default for DumlParser {
    fn default() -> Self {
        Self::new()
    }
}

impl DumlParser {
    /// Create an empty parser.
    pub fn new() -> Self {
        Self {
            buffer: StreamBuffer::new(),
        }
    }

    /// Consume an arbitrary chunk and synchronously emit validated frames.
    ///
    /// The callback owns each frame; retaining emitted frames is the caller's policy.
    pub fn push(&mut self, bytes: &[u8], mut emit: impl FnMut(DumlFrame)) {
        for byte in bytes {
            self.buffer.append(*byte);
            self.parse(false, &mut emit);
        }
    }

    /// Recover complete frames behind truncated candidates, then clear all leftovers.
    ///
    /// Use this at EOF or a known discontinuity, never at a normal USB chunk boundary.
    pub fn finish(&mut self, mut emit: impl FnMut(DumlFrame)) {
        self.parse(true, &mut emit);
    }

    /// Current incomplete input size.
    pub fn buffered_bytes(&self) -> usize {
        self.buffer.bytes().len()
    }

    /// Cumulative validation and discard counters.
    pub fn stats(&self) -> ParseStats {
        self.buffer.stats
    }

    fn parse(&mut self, eof: bool, emit: &mut impl FnMut(DumlFrame)) {
        while !self.buffer.bytes().is_empty() {
            if self.buffer.bytes()[0] != 0x55 {
                self.buffer.discard_one();
                continue;
            }
            if self.buffer.bytes().len() < 4 {
                if self.buffer.incomplete(eof) {
                    continue;
                }
                break;
            }
            let bytes = self.buffer.bytes();
            let length = usize::from(u16::from_le_bytes([bytes[1], bytes[2]]) & 0x03ff);
            if length < MIN_FRAME {
                self.buffer.stats.invalid_lengths =
                    self.buffer.stats.invalid_lengths.wrapping_add(1);
                self.buffer.discard_one();
                continue;
            }
            if checksum::header(&bytes[..3]) != bytes[3] {
                self.buffer.stats.header_crc_errors =
                    self.buffer.stats.header_crc_errors.wrapping_add(1);
                self.buffer.discard_one();
                continue;
            }
            if bytes.len() < length {
                if self.buffer.incomplete(eof) {
                    continue;
                }
                break;
            }
            let expected = u16::from_le_bytes([bytes[length - 2], bytes[length - 1]]);
            if checksum::frame(&bytes[..length - 2]) != expected {
                self.buffer.stats.frame_crc_errors =
                    self.buffer.stats.frame_crc_errors.wrapping_add(1);
                self.buffer.discard_one();
                continue;
            }
            emit(DumlFrame {
                raw: self.buffer.emit(length),
                length,
            });
        }
    }
}

#[cfg(test)]
mod tests;
