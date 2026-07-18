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

    match left_shift.cmp(&right_shift) {
        std::cmp::Ordering::Greater => {
            let shift = usize::try_from(left_shift - right_shift)
                .expect("dyadic comparison shift fits usize");
            (left << shift).cmp(right)
        },
        std::cmp::Ordering::Less => {
            let shift = usize::try_from(right_shift - left_shift)
                .expect("dyadic comparison shift fits usize");
            left.cmp(&(right << shift))
        },
        std::cmp::Ordering::Equal => left.cmp(right),
    }
}
