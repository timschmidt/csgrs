use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

pub(crate) type StorageHashMap<K, V> = HashMap<K, V, BuildHasherDefault<StorageIdentityHasher>>;
pub(crate) type StorageSequenceHashMap<K, V> =
    HashMap<K, V, BuildHasherDefault<StorageSequenceHasher>>;

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

/// Hashes short sequences of trusted storage identities with one final
/// avalanche instead of independently avalanching every pointer word.
#[derive(Default)]
pub(crate) struct StorageSequenceHasher {
    state: u64,
    words: u64,
}

impl StorageSequenceHasher {
    #[inline]
    fn push(&mut self, value: u64) {
        self.state = self.state.rotate_left(19).wrapping_add(value);
        self.words += 1;
    }
}

impl Hasher for StorageSequenceHasher {
    #[inline]
    fn finish(&self) -> u64 {
        let mut mixed = self
            .state
            .wrapping_add(self.words.wrapping_mul(0x517c_c1b7_2722_0a95));
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^ (mixed >> 31)
    }

    fn write(&mut self, bytes: &[u8]) {
        let mut chunks = bytes.chunks_exact(8);
        for chunk in &mut chunks {
            self.push(u64::from_ne_bytes(
                chunk.try_into().expect("chunk has eight bytes"),
            ));
        }
        let remainder = chunks.remainder();
        if !remainder.is_empty() {
            let mut tail = [0_u8; 8];
            tail[..remainder.len()].copy_from_slice(remainder);
            self.push(u64::from_ne_bytes(tail));
        }
        self.push(bytes.len() as u64);
    }

    #[inline]
    fn write_usize(&mut self, value: usize) {
        self.push(value as u64);
    }

    #[inline]
    fn write_u64(&mut self, value: u64) {
        self.push(value);
    }
}

#[cfg(test)]
mod tests {
    use std::hash::{Hash, Hasher};

    use super::{StorageSequenceHashMap, StorageSequenceHasher};

    fn hash_key(key: [usize; 4]) -> u64 {
        let mut hasher = StorageSequenceHasher::default();
        key.hash(&mut hasher);
        hasher.finish()
    }

    #[test]
    fn sequence_hash_preserves_word_order_and_position() {
        let key = [0x1000, 0x2000, 0x3000, 0x4000];
        assert_ne!(hash_key(key), hash_key([key[1], key[0], key[2], key[3]]));
        assert_ne!(hash_key(key), hash_key([key[0], key[1], key[3], key[2]]));
        assert_ne!(hash_key(key), hash_key([0, key[0], key[1], key[2]]));
    }

    #[test]
    fn sequence_hash_map_distinguishes_full_storage_keys() {
        let mut map = StorageSequenceHashMap::default();
        for index in 0..1_024_usize {
            let key = [
                index << 4,
                (index + 1) << 4,
                (index + 2) << 4,
                (index + 3) << 4,
            ];
            map.insert(key, index);
        }
        for index in 0..1_024_usize {
            let key = [
                index << 4,
                (index + 1) << 4,
                (index + 2) << 4,
                (index + 3) << 4,
            ];
            assert_eq!(map.get(&key), Some(&index));
        }
    }
}
