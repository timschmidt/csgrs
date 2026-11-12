use crate::IntervalError;
use core::cmp::Ordering;

// Private IEEE encoding authority. No native floating-point comparison is
// needed for finite validation, zero canonicalization or endpoint order.
#[derive(Clone, Copy)]
pub(crate) struct Format {
    sign: u64,
    infinity: u64,
}

pub(crate) const BINARY32: Format = Format {
    sign: 1 << 31,
    infinity: 0x7f80_0000,
};
pub(crate) const BINARY64: Format = Format {
    sign: 1 << 63,
    infinity: 0x7ff0_0000_0000_0000,
};

impl Format {
    pub(crate) fn finite(self, bits: u64) -> Result<u64, IntervalError> {
        if bits & self.infinity == self.infinity {
            Err(IntervalError::NonFiniteEndpoint)
        } else if bits & (self.sign - 1) == 0 {
            Ok(0)
        } else {
            Ok(bits)
        }
    }

    fn key(self, bits: u64) -> u64 {
        if bits & self.sign != 0 {
            self.sign - (bits & (self.sign - 1))
        } else {
            self.sign + bits
        }
    }

    pub(crate) fn compare(self, a: u64, b: u64) -> Ordering {
        self.key(a).cmp(&self.key(b))
    }
}
