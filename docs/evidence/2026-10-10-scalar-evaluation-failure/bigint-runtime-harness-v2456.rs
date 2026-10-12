//! Check sign and magnitude against independently normalized signed word inputs.
use patched::bigint::{
    BigInt as Candidate, BigUint as CandidateMagnitude, Sign as CandidateSign,
};
use upstream::{BigInt as Original, BigUint as OriginalMagnitude, Sign as OriginalSign};

#[derive(Default)]
struct Audit {
    fixtures: u64,
    pairs: u64,
    checksum: u64,
}

fn candidate_sign(sign: i8) -> CandidateSign {
    match sign {
        -1 => CandidateSign::Minus,
        0 => CandidateSign::NoSign,
        1 => CandidateSign::Plus,
        _ => unreachable!(),
    }
}

fn original_sign(sign: i8) -> OriginalSign {
    match sign {
        -1 => OriginalSign::Minus,
        0 => OriginalSign::NoSign,
        1 => OriginalSign::Plus,
        _ => unreachable!(),
    }
}

impl Audit {
    fn word(&mut self, value: u64) {
        self.checksum = self.checksum.wrapping_mul(0x100000001b3) ^ value;
    }

    fn pair(&mut self, candidate: &Candidate, original: &Original, sign: i8, words: &[u32]) {
        let candidate_code = match candidate.sign() {
            CandidateSign::Minus => -1,
            CandidateSign::NoSign => 0,
            CandidateSign::Plus => 1,
        };
        let original_code = match original.sign() {
            OriginalSign::Minus => -1,
            OriginalSign::NoSign => 0,
            OriginalSign::Plus => 1,
        };
        assert_eq!(candidate_code, sign);
        assert_eq!(original_code, sign);
        assert_eq!(candidate.magnitude().to_u32_digits(), words);
        assert_eq!(original.magnitude().to_u32_digits(), words);
        self.pairs += 1;
        self.word((sign + 1) as u64);
        self.word(words.len() as u64);
        for &word in words {
            self.word(u64::from(word));
        }
    }

    fn case(&mut self, words: &[u32], sign: i8) {
        // Build the expected representation from the input alone. No BigInt or
        // BigUint operation participates in this normalization oracle.
        let mut expected = if sign == 0 {
            Vec::new()
        } else {
            words.to_vec()
        };
        while expected.last() == Some(&0) {
            expected.pop();
        }
        let expected_sign = if expected.is_empty() { 0 } else { sign };
        let candidate = Candidate::from_slice(candidate_sign(sign), words);
        let original = Original::from_slice(original_sign(sign), words);
        self.pair(&candidate, &original, expected_sign, &expected);
        let from_magnitude = Candidate::from_biguint(
            candidate_sign(sign),
            CandidateMagnitude::new(words.to_vec()),
        );
        let original_from_magnitude = Original::from_biguint(
            original_sign(sign),
            OriginalMagnitude::new(words.to_vec()),
        );
        self.pair(
            &from_magnitude,
            &original_from_magnitude,
            expected_sign,
            &expected,
        );
        self.pair(
            &-candidate.clone(),
            &-original.clone(),
            -expected_sign,
            &expected,
        );
        self.pair(
            &(candidate.clone() - candidate),
            &(original.clone() - original),
            0,
            &[],
        );
        self.fixtures += 1;
    }
}

fn main() {
    let mut audit = Audit::default();
    for value in 0..=u16::MAX {
        for sign in [-1, 0, 1] {
            audit.case(&[u32::from(value)], sign);
        }
    }
    let mut state = 0x8a9d_54c3_b217_e605u64;
    for length in 0..=129 {
        let mut words = Vec::with_capacity(length);
        for index in 0..length {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            words.push(if index % 5 == 0 { 0 } else { state as u32 });
        }
        let mut padded = words.clone();
        padded.extend([0, 0, 0]);
        for input in [&words, &padded] {
            for sign in [-1, 0, 1] {
                audit.case(input, sign);
            }
        }
    }
    assert_eq!(audit.fixtures, 197_388);
    assert_eq!(audit.pairs, 789_552);
    println!(
        "fixtures={} paired_values={} diagnostic_checksum={:016x}",
        audit.fixtures, audit.pairs, audit.checksum
    );
}
