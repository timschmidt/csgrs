impl Computable {
    /// Recover a positive exact expression for exp(self) from selected native logarithm recipes.
    /// Jacobi relation consumers use this to recover the complement from its logarithm
    /// without expanding it into a new numerical exponential. A log tag alone does
    /// not prove its domain: admit ln(1+r) only with a positive 1+r certificate, or
    /// for a computable residual, an existing approximation proving its lower bound
    /// strictly greater than -1. That cached route requires `|precision| <= 4096`
    /// and requests no new digits. A successful result receives a retained positive sign.
    ///
    /// Recognize an integer-zero leaf, six shared integer logarithms, rational/computable ln(1+r),
    /// binary-scaled rational logs, negation, sums, binary offsets of absolute size
    /// at most four, and multiplication by an exact integer of absolute value <=16.
    /// Negation recovers an inverse, sums a product, and scaling powers/positive roots.
    /// Even a zero integer power first requires successful recovery of its child.
    ///
    /// Consume one shared u8 visit charge on entry, then poll this wrapper's signal;
    /// poll again before publishing the result. Failure consumes prior visits and
    /// can leave strengthened facts from subqueries. None combines unrecognized syntax,
    /// invalid/unproved domains, guard failures and cancellation, without a typed reason.
    /// The visit count excludes constructors, exact-sign proofs and rational work.
    /// The rational-size check is partly post-construction; unrestricted shifts on
    /// binary-scaled log nodes can allocate before that check. These guards and polls
    /// are not total-work, retained-byte or cancellation-latency bounds.
    fn retained_log_argument(&self, remaining: &mut u8) -> Option<Self> {
        *remaining = remaining.checked_sub(1)?;
        if should_stop(&self.signal) {
            return None;
        }
        let positive = |value: Self| (value.exact_sign() == Some(Sign::Plus)).then_some(value);
        /// Reject the VeryLarge storage bucket before reading canonical coefficients, then require their combined bit count <=4096.
        /// This is a retention/recognition guard, not a bound on earlier construction or
        /// canonicalization work. It counts numerator plus denominator bits, including zero
        /// and unit conventions supplied by Rational.
        fn fits(value: &Rational) -> bool {
            !matches!(
                value.storage_class(),
                crate::RationalStorageClass::VeryLarge
            ) && value.numerator().bits() + value.denominator().bits() <= 4096
        }
        let result = match &self.internal.approximation {
            Approximation::Int(n) if n.is_zero() => Some(Self::one()),
            Approximation::PrescaledLnRational(residual) if fits(residual) => {
                positive(Self::rational(residual.add_one()))
            }
            Approximation::PrescaledLn(residual) => {
                let argument = residual.clone().add(Self::one());
                if argument.exact_sign() == Some(Sign::Plus) {
                    Some(argument)
                } else {
                    // ln construction/evaluation may already have certified the
                    // residual. Read that enclosure without requesting digits.
                    let (precision, approximation) = residual.cached()?;
                    if precision.unsigned_abs() > 4096 {
                        return None;
                    }
                    let lower = Rational::from_bigint(approximation - 1)
                        * Self::power_of_two_rational(precision);
                    (lower > Rational::from(-1)).then_some(argument)
                }
            }
            Approximation::BinaryScaledLnRational { residual, shift } if fits(residual) => {
                Some(positive(Self::rational(residual.add_one()))?.shift_left(*shift))
            }
            Approximation::Constant(kind) => {
                let value = match kind {
                    SharedConstant::Ln2 => 2,
                    SharedConstant::Ln3 => 3,
                    SharedConstant::Ln5 => 5,
                    SharedConstant::Ln6 => 6,
                    SharedConstant::Ln7 => 7,
                    SharedConstant::Ln10 => 10,
                    _ => return None,
                };
                Some(Self::rational(Rational::from(value)))
            }
            Approximation::Negate(child) => Some(child.retained_log_argument(remaining)?.inverse()),
            Approximation::Add(left, right) => Some(
                left.retained_log_argument(remaining)?
                    .multiply(right.retained_log_argument(remaining)?),
            ),
            Approximation::Offset(child, shift) if shift.unsigned_abs() <= 4 => {
                let mut value = child.retained_log_argument(remaining)?;
                for _ in 0..shift.unsigned_abs() {
                    value = if *shift < 0 {
                        value.sqrt()
                    } else {
                        value.square()
                    };
                }
                Some(value)
            }
            Approximation::Multiply(left, right) => {
                let (power, child) = if let Some(power) = left.exact_rational() {
                    (power, right)
                } else {
                    (right.exact_rational()?, left)
                };
                let power = power.to_integer_i64().filter(|p| p.unsigned_abs() <= 16)?;
                let argument = child.retained_log_argument(remaining)?;
                let mut value = Self::one();
                for _ in 0..power.unsigned_abs() {
                    value = value.multiply(argument.clone());
                }
                Some(if power < 0 { value.inverse() } else { value })
            }
            _ => None,
        }?;
        if result.exact_rational().is_some_and(|r| !fits(&r)) {
            return None;
        }
        if should_stop(&self.signal) {
            return None;
        }
        // Every admitted branch proves exp(self)>0, including the cached
        // residual enclosure above. Retain that proof on the reconstructed
        // argument so its inverse can consume it without fresh refinement.
        result
            .internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(Sign::Plus));
        Some(result)
    }
}
