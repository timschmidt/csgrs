//! Compare bound fields and metadata-call counts with an independent word oracle.
use patched::bigint::{BigInt as Candidate, Sign as CandidateSign};
use upstream::{BigInt as Original, Sign as OriginalSign};

include!("modules.rs");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Observation {
    Unknown,
    Zero,
    NonZero {
        sign: Option<i8>,
        msd: Option<i32>,
        exact: bool,
    },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Calls {
    magnitude: u8,
    bits: u8,
    ones: u8,
    zeros: u8,
    sign: u8,
}

// These operations work on the input words, never on either BigInt type or its
// metadata. Increment/decrement endpoints avoid using trailing-bit predicates.
fn normalize(mut words: Vec<u32>) -> Vec<u32> {
    while words.last() == Some(&0) {
        words.pop();
    }
    words
}

fn increment(words: &[u32]) -> Vec<u32> {
    let mut result = words.to_vec();
    for word in &mut result {
        if *word != u32::MAX {
            *word += 1;
            return result;
        }
        *word = 0;
    }
    result.push(1);
    result
}

fn decrement(words: &[u32]) -> Vec<u32> {
    assert!(!words.is_empty());
    let mut result = words.to_vec();
    for word in &mut result {
        if *word != 0 {
            *word -= 1;
            return normalize(result);
        }
        *word = u32::MAX;
    }
    unreachable!()
}

fn bit_length(words: &[u32]) -> i64 {
    let Some(&last) = words.last() else {
        return 0;
    };
    assert_ne!(last, 0);
    let mut top = last;
    let mut bits = 32 * (words.len() as i64 - 1);
    while top != 0 {
        bits += 1;
        top /= 2;
    }
    bits
}

struct EndpointOracle {
    sign: i8,
    low_bits: i64,
    high_bits: i64,
    unknown: bool,
}

impl EndpointOracle {
    fn new(words: &[u32], sign: i8) -> Self {
        let magnitude = normalize(if sign == 0 {
            Vec::new()
        } else {
            words.to_vec()
        });
        let unknown = magnitude.is_empty() || magnitude.as_slice() == [1];
        if unknown {
            Self {
                sign,
                low_bits: 0,
                high_bits: 0,
                unknown,
            }
        } else {
            Self {
                sign,
                low_bits: bit_length(&decrement(&magnitude)),
                high_bits: bit_length(&increment(&magnitude)),
                unknown,
            }
        }
    }

    fn bound(&self, precision: i32) -> Observation {
        if self.unknown {
            Observation::Unknown
        } else {
            let msd = i32::try_from(i64::from(precision) + self.high_bits - 1).ok();
            Observation::NonZero {
                sign: Some(self.sign),
                msd,
                exact: msd.is_some() && self.low_bits == self.high_bits,
            }
        }
    }

    fn precisions(&self) -> [i32; 14] {
        let boundary = i64::from(i32::MAX) - (self.high_bits - 1).max(0);
        let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
        [
            i32::MIN,
            i32::MIN + 1,
            -4096,
            -65,
            -1,
            0,
            1,
            64,
            4096,
            i32::MAX - 1,
            i32::MAX,
            clamp(boundary - 1),
            clamp(boundary),
            clamp(boundary + 1),
        ]
    }
}

#[derive(Default)]
struct Audit {
    fixtures: u64,
    queries: u64,
    unknown: u64,
    overflow: u64,
    exact: u64,
    inexact: u64,
    checksum: u64,
}

impl Audit {
    fn word(&mut self, value: u64) {
        self.checksum = self.checksum.wrapping_mul(0x100000001b3) ^ value;
    }

    fn case(&mut self, words: &[u32], sign: i8) {
        let candidate = Candidate::from_slice(
            match sign {
                -1 => CandidateSign::Minus,
                0 => CandidateSign::NoSign,
                1 => CandidateSign::Plus,
                _ => unreachable!(),
            },
            words,
        );
        let original = Original::from_slice(
            match sign {
                -1 => OriginalSign::Minus,
                0 => OriginalSign::NoSign,
                1 => OriginalSign::Plus,
                _ => unreachable!(),
            },
            words,
        );
        let oracle = EndpointOracle::new(words, sign);
        for precision in oracle.precisions() {
            let expected = oracle.bound(precision);
            assert_eq!(candidate_plain::run(precision, &candidate), expected);
            assert_eq!(original_plain::run(precision, &original), expected);
            let (new_result, new_calls) = candidate_counted::run(precision, &candidate);
            let (old_result, old_calls) = original_counted::run(precision, &original);
            assert_eq!(new_result, expected);
            assert_eq!(old_result, expected);
            let mut calls = Calls {
                magnitude: 1,
                bits: 1,
                ..Calls::default()
            };
            match expected {
                Observation::Unknown => {
                    self.unknown += 1;
                    self.word(0);
                },
                Observation::Zero => {
                    panic!("An uncertain approximation cannot prove exact zero")
                },
                Observation::NonZero { sign, msd, exact } => {
                    calls.ones = 1;
                    calls.sign = 1;
                    calls.zeros = u8::from(msd.is_some());
                    match msd {
                        None => self.overflow += 1,
                        Some(_) if exact => self.exact += 1,
                        Some(_) => self.inexact += 1,
                    }
                    self.word((sign.unwrap() + 2) as u64);
                    self.word(u64::from(msd.is_some()));
                    self.word(msd.unwrap_or_default() as u32 as u64);
                    self.word(u64::from(exact));
                },
            }
            assert_eq!(new_calls, calls);
            assert_eq!(old_calls, calls);
            self.word(precision as u32 as u64);
            self.queries += 1;
        }
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
    for shift in [
        2, 3, 7, 8, 15, 16, 31, 32, 33, 63, 64, 65, 127, 128, 129, 255, 256, 257, 511, 512,
        513, 1023, 1024, 1025, 2047, 2048, 2049, 4095, 4096,
    ] {
        let mut power = vec![0; shift / 32 + 1];
        power[shift / 32] = 1 << (shift % 32);
        for magnitude in [
            decrement(&decrement(&power)),
            decrement(&power),
            power.clone(),
            increment(&power),
            increment(&increment(&power)),
        ] {
            let mut padded = magnitude.clone();
            padded.extend([0, 0, 0]);
            for words in [&magnitude, &padded] {
                for sign in [-1, 0, 1] {
                    audit.case(words, sign);
                }
            }
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
    assert_eq!(audit.fixtures, 198_258);
    assert_eq!(audit.queries, 2_775_612);
    assert_eq!(
        audit.unknown + audit.overflow + audit.exact + audit.inexact,
        audit.queries
    );
    assert!(audit.unknown > 0 && audit.overflow > 0 && audit.exact > 0 && audit.inexact > 0);
    println!(
        "fixtures={} queries={} plain_values={} counted_values={} unknown={} overflow={} exact={} inexact={} diagnostic_checksum={:016x}",
        audit.fixtures,
        audit.queries,
        2 * audit.queries,
        2 * audit.queries,
        audit.unknown,
        audit.overflow,
        audit.exact,
        audit.inexact,
        audit.checksum
    );
}
