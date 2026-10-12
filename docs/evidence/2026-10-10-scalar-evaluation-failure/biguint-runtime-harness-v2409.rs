//! Differential metadata checks with an independent bit-vector oracle.
extern crate alloc;

#[path = "../vendor/num-bigint/src/biguint/metadata_u32.rs"]
mod metadata_u32;
#[path = "../vendor/num-bigint/src/biguint/metadata_u64.rs"]
mod metadata_u64;

use patched::BigUint as Patched;
use upstream::BigUint as Upstream;

#[derive(Default)]
struct Audit {
    cases: u64,
    pairs: u64,
    helper_width_checks: u64,
    checksum: u64,
}

// Encode each input bit directly, independently of native count intrinsics and
// the dependency's word scans. Leading zero words are deliberately permitted.
fn oracle(words: &[u32]) -> (u64, Option<u64>, u64) {
    let mut bits = Vec::with_capacity(words.len() * 32);
    for &word in words {
        for bit in 0..32 {
            bits.push((word >> bit) & 1);
        }
    }
    while bits.last() == Some(&0) {
        bits.pop();
    }
    if bits.is_empty() {
        (0, None, 0)
    } else {
        (
            bits.len() as u64,
            Some(bits.iter().take_while(|&&bit| bit == 0).count() as u64),
            bits.iter().take_while(|&&bit| bit == 1).count() as u64,
        )
    }
}

impl Audit {
    fn word(&mut self, value: u64) {
        // A deterministic diagnostic checksum; the recorded source/binary/log
        // SHA-256 hashes provide the cryptographic artifact identity.
        self.checksum = self.checksum.wrapping_mul(0x100000001b3) ^ value;
    }

    fn pair(&mut self, candidate: &Patched, original: &Upstream) {
        let expected_words = original.to_u32_digits();
        assert_eq!(candidate.to_u32_digits(), expected_words);
        let expected = oracle(&expected_words);
        let actual = (
            candidate.bits(),
            candidate.trailing_zeros(),
            candidate.trailing_ones(),
        );
        let original_metadata = (
            original.bits(),
            original.trailing_zeros(),
            original.trailing_ones(),
        );
        assert_eq!(actual, expected);
        assert_eq!(original_metadata, expected);
        self.pairs += 1;
        self.word(actual.0);
        self.word(actual.1.unwrap_or(u64::MAX));
        self.word(actual.2);
        for word in expected_words {
            self.word(u64::from(word));
        }
    }

    fn case(&mut self, words: Vec<u32>) {
        let expected = oracle(&words);
        let mut narrow = words.clone();
        let narrow_capacity = narrow.capacity();
        assert_eq!(metadata_u32::zero_count(&narrow), expected.1);
        assert_eq!(metadata_u32::one_count(&narrow), expected.2);
        metadata_u32::trim_zero_suffix(&mut narrow);
        assert_eq!(narrow.capacity(), narrow_capacity);
        assert_eq!(metadata_u32::bit_count(&narrow), expected.0);

        let mut wide: Vec<u64> = words
            .chunks(2)
            .map(|pair| u64::from(pair[0]) | (u64::from(*pair.get(1).unwrap_or(&0)) << 32))
            .collect();
        let wide_capacity = wide.capacity();
        assert_eq!(metadata_u64::zero_count(&wide), expected.1);
        assert_eq!(metadata_u64::one_count(&wide), expected.2);
        metadata_u64::trim_zero_suffix(&mut wide);
        assert_eq!(wide.capacity(), wide_capacity);
        assert_eq!(metadata_u64::bit_count(&wide), expected.0);
        self.helper_width_checks += 2;

        let candidate = Patched::new(words.clone());
        let original = Upstream::new(words.clone());
        assert_eq!(candidate.to_u32_digits(), narrow);
        assert_eq!(candidate.to_u64_digits(), wide);
        assert_eq!(candidate.bits(), expected.0);
        assert_eq!(candidate.trailing_zeros(), expected.1);
        assert_eq!(candidate.trailing_ones(), expected.2);
        self.pair(&candidate, &original);
        self.pair(&Patched::from_slice(&words), &Upstream::from_slice(&words));

        // Exercise normalization through assignment, arithmetic, shifts and
        // bit clearing on a deterministic subset, including large values.
        if self.cases % 257 == 0 || words.len() > 2 {
            let mut assigned = Patched::from(u128::MAX);
            let mut assigned_original = Upstream::from(u128::MAX);
            assigned.assign_from_slice(&words);
            assigned_original.assign_from_slice(&words);
            self.pair(&assigned, &assigned_original);
            self.pair(&(&candidate + 1u32), &(&original + 1u32));
            self.pair(&((&candidate + 1u32) - 1u32), &((&original + 1u32) - 1u32));
            self.pair(&(&candidate * 3u32), &(&original * 3u32));
            self.pair(&(&candidate / 3u32), &(&original / 3u32));
            for shift in [1usize, 31, 32, 33, 63, 64, 65, 127] {
                self.pair(&(&candidate << shift), &(&original << shift));
                self.pair(&(&candidate >> shift), &(&original >> shift));
            }
            let mut cleared = candidate.clone();
            let mut cleared_original = original.clone();
            let top = expected.0.saturating_sub(1);
            cleared.set_bit(top, false);
            cleared_original.set_bit(top, false);
            self.pair(&cleared, &cleared_original);
        }
        self.cases += 1;
    }
}

fn main() {
    let mut audit = Audit::default();
    audit.case(Vec::new());
    for value in 0..=u16::MAX {
        audit.case(vec![u32::from(value)]);
    }
    for bit in (0..=256).chain([511, 512, 513, 1023, 1024, 1025, 4095, 4096]) {
        let mut power = vec![0; bit / 32 + 1];
        power[bit / 32] = 1 << (bit % 32);
        audit.case(power.clone());
        power.extend([0, 0, 0]);
        audit.case(power);
        let mut lower = vec![u32::MAX; bit / 32];
        lower.push(if bit % 32 == 0 {
            0
        } else {
            (1u32 << (bit % 32)) - 1
        });
        audit.case(lower.clone());
        lower.extend([0, 0, 0]);
        audit.case(lower);
    }
    let mut state = 0xd87f42e956731ab9u64;
    for length in [0, 1, 2, 3, 4, 7, 8, 16, 31, 32, 63, 64, 127, 128, 257] {
        audit.case(vec![0; length]);
        audit.case(vec![u32::MAX; length]);
        audit.case(vec![0xaaaa_aaaa; length]);
        audit.case(vec![0x5555_5555; length]);
        for _ in 0..16 {
            let words: Vec<u32> = (0..length)
                .map(|_| {
                    state = state
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    (state >> 32) as u32
                })
                .collect();
            audit.case(words.clone());
            let mut padded = words;
            padded.extend([0, 0, 0]);
            audit.case(padded);
        }
    }
    println!(
        "cases={} paired_values={} helper_width_checks={} diagnostic_checksum={:016x}",
        audit.cases, audit.pairs, audit.helper_width_checks, audit.checksum
    );
}
