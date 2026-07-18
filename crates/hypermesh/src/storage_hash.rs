use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

pub(crate) type StorageHashMap<K, V> = HashMap<K, V, BuildHasherDefault<StorageIdentityHasher>>;

#[derive(Default)]
pub(crate) struct StorageIdentityHasher(u64);

impl StorageIdentityHasher {
    #[inline]
    fn mix(&mut self, value: u64) {
        let mut mixed = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^= mixed >> 31;
        self.0 = self
            .0
            .rotate_left(23)
            .wrapping_add(mixed)
            .wrapping_mul(0x9e37_79b9_7f4a_7c15);
    }
}

impl Hasher for StorageIdentityHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        let mut chunks = bytes.chunks_exact(8);
        for chunk in &mut chunks {
            self.mix(u64::from_ne_bytes(
                chunk.try_into().expect("chunk has eight bytes"),
            ));
        }
        let remainder = chunks.remainder();
        if !remainder.is_empty() {
            let mut tail = [0_u8; 8];
            tail[..remainder.len()].copy_from_slice(remainder);
            self.mix(u64::from_ne_bytes(tail));
        }
        self.mix(bytes.len() as u64);
    }

    #[inline]
    fn write_usize(&mut self, value: usize) {
        self.mix(value as u64);
    }

    #[inline]
    fn write_u64(&mut self, value: u64) {
        self.mix(value);
    }
}
