impl PartialEq for Rational {
    fn eq(&self, other: &Self) -> bool {
        if std::sync::Arc::ptr_eq(&self.0, &other.0) {
            true
        } else if self.sign != other.sign {
            false
        } else if self.denominator == other.denominator {
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
        if std::sync::Arc::ptr_eq(&self.0, &other.0) {
            return Some(Equal);
        }
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
        {
            let ordering = if left_msd != right_msd {
                crate::trace_dispatch!("rational", "comparison", "magnitude-bits");
                left_msd.cmp(&right_msd)
            } else if let Some(ordering) =
                compare_normalized_magnitude_intervals(self, other, left_msd)
            {
                crate::trace_dispatch!("rational", "comparison", "leading-bits-interval");
                ordering
            } else {
                crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
                let left = &self.numerator * &other.denominator;
                let right = &other.numerator * &self.denominator;
                left.cmp(&right)
            };
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

fn compare_normalized_magnitude_intervals(
    left: &Rational,
    right: &Rational,
    common_msd: i32,
) -> Option<std::cmp::Ordering> {
    let (left_lower, left_upper) = normalized_rational_magnitude_interval(left, common_msd)?;
    let (right_lower, right_upper) = normalized_rational_magnitude_interval(right, common_msd)?;
    if left_upper < right_lower {
        Some(std::cmp::Ordering::Less)
    } else if right_upper < left_lower {
        Some(std::cmp::Ordering::Greater)
    } else {
        None
    }
}

fn normalized_rational_magnitude_interval(
    value: &Rational,
    msd: i32,
) -> Option<(f64, f64)> {
    let (numerator_lower, numerator_upper) = normalized_biguint_interval(&value.numerator)?;
    let (denominator_lower, denominator_upper) = normalized_biguint_interval(&value.denominator)?;
    let raw_exponent =
        i128::from(value.numerator.bits()) - i128::from(value.denominator.bits());
    let scale_shift = raw_exponent - i128::from(msd);
    if !(0..=1).contains(&scale_shift) {
        return None;
    }
    let scale = if scale_shift == 0 { 1.0 } else { 2.0 };
    let lower = ((numerator_lower / denominator_upper) * scale).next_down();
    let upper = ((numerator_upper / denominator_lower) * scale).next_up();
    (lower.is_finite() && upper.is_finite()).then_some((lower, upper))
}

fn normalized_biguint_interval(value: &BigUint) -> Option<(f64, f64)> {
    const SIGNIFICAND_BITS: u64 = 53;

    let bits = value.bits();
    if bits == 0 {
        return None;
    }
    if bits <= SIGNIFICAND_BITS {
        let normalized =
            value.to_u64()? as f64 / (1_u64 << (bits.saturating_sub(1) as u32)) as f64;
        return Some((normalized, normalized));
    }

    let mut digits = value.iter_u64_digits();
    let high = digits.next_back()?;
    let high_bits = 64 - u64::from(high.leading_zeros());
    let leading = if high_bits >= SIGNIFICAND_BITS {
        high >> (high_bits - SIGNIFICAND_BITS)
    } else {
        let remaining = SIGNIFICAND_BITS - high_bits;
        (high << remaining) | (digits.next_back().unwrap_or_default() >> (64 - remaining))
    };
    let lower = leading as f64 / (1_u64 << (SIGNIFICAND_BITS - 1)) as f64;
    let upper = (leading + 1) as f64 / (1_u64 << (SIGNIFICAND_BITS - 1)) as f64;
    Some((lower, upper))
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
