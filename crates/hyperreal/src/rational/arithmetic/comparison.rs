impl PartialEq for Rational {
    fn eq(&self, other: &Self) -> bool {
        if self.sign != other.sign {
            return false;
        }
        if self.denominator == other.denominator {
            self.numerator == other.numerator
        } else if let Some(ordering) = compare_dyadic_magnitudes(self, other) {
            ordering.is_eq()
        } else if let Some(ordering) = compare_word_magnitudes(self, other) {
            crate::trace_dispatch!("rational", "comparison", "word-sized");
            ordering.is_eq()
        } else if self.msd_exact() != other.msd_exact() {
            crate::trace_dispatch!("rational", "comparison", "magnitude-bits");
            false
        } else {
            crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
            &self.numerator * &other.denominator == &other.numerator * &self.denominator
        }
    }
}

impl Eq for Rational {}

impl std::hash::Hash for Rational {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        // Rational construction maintains a reduced canonical numerator and
        // denominator, so value equality implies identical stored fields.
        self.sign.hash(state);
        self.numerator.hash(state);
        self.denominator.hash(state);
    }
}

impl PartialOrd for Rational {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering::*;
        match self.sign.cmp(&other.sign) {
            Less => return Some(Less),
            Greater => return Some(Greater),
            Equal => {
                if self.sign == NoSign {
                    return Some(Equal);
                }
            }
        }
        if self.denominator == other.denominator {
            match self.sign {
                Plus => self.numerator.partial_cmp(&other.numerator),
                Minus => other.numerator.partial_cmp(&self.numerator),
                NoSign => unreachable!(),
            }
        } else if let Some(ordering) = compare_dyadic_magnitudes(self, other) {
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else if let Some(ordering) = compare_word_magnitudes(self, other) {
            crate::trace_dispatch!("rational", "comparison", "word-sized");
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else if let (Some(left_msd), Some(right_msd)) =
            (self.msd_exact(), other.msd_exact())
            && left_msd != right_msd
        {
            crate::trace_dispatch!("rational", "comparison", "magnitude-bits");
            let ordering = left_msd.cmp(&right_msd);
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else {
            crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
            let left = &self.numerator * &other.denominator;
            let right = &other.numerator * &self.denominator;
            match self.sign {
                Plus => left.partial_cmp(&right),
                Minus => right.partial_cmp(&left),
                NoSign => unreachable!(),
            }
        }
    }
}

fn compare_word_magnitudes(left: &Rational, right: &Rational) -> Option<std::cmp::Ordering> {
    let left_cross = left
        .numerator
        .to_u128()?
        .checked_mul(right.denominator.to_u128()?)?;
    let right_cross = right
        .numerator
        .to_u128()?
        .checked_mul(left.denominator.to_u128()?)?;
    Some(left_cross.cmp(&right_cross))
}

fn compare_dyadic_magnitudes(left: &Rational, right: &Rational) -> Option<std::cmp::Ordering> {
    let left_denominator_shift = power_of_two_shift(&left.denominator)?;
    let right_denominator_shift = power_of_two_shift(&right.denominator)?;
    crate::trace_dispatch!("rational", "comparison", "dyadic-borrowed-digits");
    Some(compare_shifted_biguints(
        &left.numerator,
        right_denominator_shift,
        &right.numerator,
        left_denominator_shift,
    ))
}

fn power_of_two_shift(value: &BigUint) -> Option<u64> {
    let shift = value.trailing_zeros()?;
    (shift == value.bits() - 1).then_some(shift)
}

fn compare_shifted_biguints(
    left: &BigUint,
    left_shift: u64,
    right: &BigUint,
    right_shift: u64,
) -> std::cmp::Ordering {
    let left_bits = left.bits() + left_shift;
    let right_bits = right.bits() + right_shift;
    match left_bits.cmp(&right_bits) {
        std::cmp::Ordering::Equal => {}
        ordering => return ordering,
    }

    let common_shift = left_shift.min(right_shift);
    let mut left_digits = ShiftedU64Digits::new(left, left_shift - common_shift);
    let mut right_digits = ShiftedU64Digits::new(right, right_shift - common_shift);
    loop {
        match (left_digits.next(), right_digits.next()) {
            (Some(left), Some(right)) => match left.cmp(&right) {
                std::cmp::Ordering::Equal => {},
                ordering => return ordering,
            },
            (None, None) => return std::cmp::Ordering::Equal,
            _ => unreachable!("equal-width shifted magnitudes have equal digit counts"),
        }
    }
}

/// Allocation-free, most-significant-first `u64` digits for `value << shift`.
///
/// `BigUint` exposes a borrowed double-ended digit iterator. Combining adjacent
/// source digits while walking backward avoids materializing the shifted value
/// for exact dyadic comparisons.
struct ShiftedU64Digits<'a> {
    digits: num::bigint::U64Digits<'a>,
    bit_shift: u32,
    upper: Option<u64>,
    high_carry: Option<u64>,
    low_zero_words: u64,
}

impl<'a> ShiftedU64Digits<'a> {
    fn new(value: &'a BigUint, shift: u64) -> Self {
        let bit_shift = (shift % 64) as u32;
        let mut digits = value.iter_u64_digits();
        let upper = (bit_shift != 0).then(|| digits.next_back()).flatten();
        let high_carry = upper
            .map(|digit| digit >> (64 - bit_shift))
            .filter(|&digit| digit != 0);
        Self {
            digits,
            bit_shift,
            upper,
            high_carry,
            low_zero_words: shift / 64,
        }
    }
}

impl Iterator for ShiftedU64Digits<'_> {
    type Item = u64;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bit_shift == 0 {
            return self.digits.next_back().or_else(|| {
                (self.low_zero_words != 0).then(|| {
                    self.low_zero_words -= 1;
                    0
                })
            });
        }
        if let Some(carry) = self.high_carry.take() {
            return Some(carry);
        }
        if let Some(upper) = self.upper {
            let lower = self.digits.next_back();
            self.upper = lower;
            return Some(
                (upper << self.bit_shift)
                    | lower.unwrap_or_default() >> (64 - self.bit_shift),
            );
        }
        (self.low_zero_words != 0).then(|| {
            self.low_zero_words -= 1;
            0
        })
    }
}
