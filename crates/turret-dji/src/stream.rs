/// Cumulative observations; all counters wrap on overflow.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ParseStats {
    /// Complete frames emitted to the caller.
    pub frames: u64,
    /// Bytes discarded while searching for a valid frame.
    pub discarded_bytes: u64,
    /// Candidate headers rejected for an invalid or excessive length.
    pub invalid_lengths: u64,
    /// DUML candidate headers rejected by CRC-8.
    pub header_crc_errors: u64,
    /// DUML candidate frames rejected by CRC-16.
    pub frame_crc_errors: u64,
    /// Incomplete candidates abandoned at an explicit stream boundary.
    pub truncated_candidates: u64,
    /// Largest number of bytes simultaneously buffered.
    pub peak_buffered_bytes: usize,
}

pub(crate) struct StreamBuffer {
    storage: [u8; 1023],
    end: usize,
    start: usize,
    pub(crate) stats: ParseStats,
}

impl StreamBuffer {
    pub(crate) fn new() -> Self {
        Self {
            storage: [0; 1023],
            end: 0,
            start: 0,
            stats: ParseStats::default(),
        }
    }

    pub(crate) fn append(&mut self, byte: u8) {
        if self.end == self.storage.len() && self.start > 0 {
            self.storage.copy_within(self.start..self.end, 0);
            self.end -= self.start;
            self.start = 0;
        }
        // The parser consumes or rejects every full ten-bit frame before append.
        self.storage[self.end] = byte;
        self.end += 1;
        self.stats.peak_buffered_bytes = self.stats.peak_buffered_bytes.max(self.bytes().len());
    }

    pub(crate) fn bytes(&self) -> &[u8] {
        &self.storage[self.start..self.end]
    }

    pub(crate) fn discard_one(&mut self) {
        // Advancing a cursor avoids moving a large corrupt payload for each bad byte.
        self.consume(1);
        self.stats.discarded_bytes = self.stats.discarded_bytes.wrapping_add(1);
    }

    pub(crate) fn emit(&mut self, length: usize) -> [u8; 1023] {
        self.stats.frames = self.stats.frames.wrapping_add(1);
        let mut frame = [0; 1023];
        frame[..length].copy_from_slice(&self.bytes()[..length]);
        self.consume(length);
        frame
    }

    fn consume(&mut self, length: usize) {
        self.start += length;
        if self.start == self.end {
            self.end = 0;
            self.start = 0;
        }
    }

    pub(crate) fn incomplete(&mut self, eof: bool) -> bool {
        if eof {
            self.stats.truncated_candidates = self.stats.truncated_candidates.wrapping_add(1);
            self.discard_one();
        }
        eof
    }
}
