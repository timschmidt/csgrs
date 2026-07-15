impl Computable {
    fn integer_ratio_nearest(&self, divisor: Computable) -> BigInt {
        // Low-precision nearest-integer quotient used only for range reduction.
        // Use fixed-precision values directly and explicit remainder correction
        // to avoid creating an extra inverse path before the caller's own
        // correction loop.
        // This keeps reduction for very large arguments on the cheap path while
        // preserving Payne-Hanek-style behavior.
        let precision: Precision = -4;
        let numerator = self.approx(precision);
        let denominator = divisor.approx(precision);
        if denominator.is_zero() {
            return BigInt::zero();
        }

        let same_sign = numerator.sign() == denominator.sign();
        let abs_numerator = numerator.magnitude().clone();
        let abs_denominator = denominator.magnitude().clone();
        let mut quotient = abs_numerator.clone() / abs_denominator.clone();
        let remainder = abs_numerator % abs_denominator.clone();
        if remainder >= (abs_denominator >> 1) {
            quotient += 1_u32;
        }

        if same_sign {
            BigInt::from_biguint(Sign::Plus, quotient)
        } else {
            BigInt::from_biguint(Sign::Minus, quotient)
        }
    }

    fn reduce_by_divisor(
        &self,
        divisor: &Self,
        low_prec: Precision,
        max_attempts: u32,
    ) -> Option<(Self, BigInt)> {
        let mut multiple = self.integer_ratio_nearest(divisor.clone());

        for _ in 0..max_attempts {
            let adjustment = divisor
                .clone()
                .multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
            let reduced = self.clone().add(adjustment);
            let reduced_appr = reduced.approx(low_prec);

            if reduced_appr > *signed::EIGHT {
                multiple += 1;
                continue;
            }
            if reduced_appr < -signed::EIGHT.clone() {
                multiple -= 1;
                continue;
            }

            return Some((reduced, multiple));
        }

        None
    }

    fn prescaled_exp(self) -> Self {
        // Preserve structural form while deferring the expensive approximation of
        // exp to the requested precision; this avoids recursive constructor
        // expansion when explicit reduction has already normalized the input.
        // exp(x) stays strictly positive across all domain values, so cache
        // that fact directly for fast sign/zero checks.
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledExp(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Natural Exponential function, raise Euler's Number to this number.
    pub fn exp(self) -> Computable {
        if self.exact_rational().as_ref().is_some_and(Rational::is_one) {
            // e^1 is the shared cached constant, not a fresh PrescaledExp node.
            crate::trace_dispatch!("computable", "exp", "exact-one-shared-e");
            return Self::e_constant();
        }
        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            // e^0 is exact and must stay outside the approximation pipeline.
            crate::trace_dispatch!("computable", "exp", "exact-zero-one");
            return Self::one();
        }
        if let Some(msd) = self.planning_sign_and_msd().1.flatten() {
            if msd <= 2 {
                crate::trace_dispatch!("computable", "exp", "structural-small-prescaled");
                return self.prescaled_exp();
            }
            if msd >= 4 {
                crate::trace_dispatch!("computable", "exp", "structural-large-range-reduction");
                let ln2 = Self::ln2();
                let low_prec: Precision = -4;
                const REDUCTION_MAX_ATTEMPTS: u32 = 64;

                if let Some((reduced, multiple)) =
                    self.reduce_by_divisor(&ln2, low_prec, REDUCTION_MAX_ATTEMPTS)
                {
                    crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                    return reduced.prescaled_exp().shift_left(
                        multiple
                            .try_into()
                            .expect("binary exponent should fit in i32"),
                    );
                }

                // If the cheap correction loop cannot converge at this scale,
                // prefer preserving a deferred-expensive symbolic form over
                // recursing deeper and risking stack blowup.
                crate::trace_dispatch!("computable", "exp", "ln2-range-reduction-fallback");
                return self.prescaled_exp();
            }
        }
        let low_prec: Precision = -4;
        let rough_appr: BigInt = self.approx(low_prec);
        // At precision -4, an approximation outside +/-8 implies |x| > 0.5.
        if rough_appr > *signed::EIGHT || rough_appr < -signed::EIGHT.clone() {
            // Keep the Taylor kernel near zero by subtracting k*ln(2), then reapply
            // the scale as a binary shift. This avoids slow huge-argument series work.
            let ln2 = Self::ln2();
            const REDUCTION_MAX_ATTEMPTS: u32 = 64;

            if let Some((reduced, multiple)) =
                self.reduce_by_divisor(&ln2, low_prec, REDUCTION_MAX_ATTEMPTS)
            {
                crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                return reduced.prescaled_exp().shift_left(
                    multiple
                        .try_into()
                        .expect("binary exponent should fit in i32"),
                );
            }

            // Fallback keeps large, symbolic arguments on the cold path and
            // avoids recursive expansion when the correction loop cannot
            // stabilize quickly.
            crate::trace_dispatch!("computable", "exp", "ln2-range-reduction-fallback");
            return self.prescaled_exp();
        }

        crate::trace_dispatch!("computable", "exp", "prescaled-kernel");
        self.prescaled_exp()
    }

    /// `exp(x) - 1`, retaining the small-argument intent until approximation.
    pub fn expm1(self) -> Computable {
        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            return Self::zero();
        }
        let exact_sign = match self.exact_sign() {
            Some(sign) => ExactSignCache::Valid(sign),
            None => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Expm1(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Calculate nearby multiple of pi.
    fn pi_multiple(&self) -> BigInt {
        // Use one low-precision quotient and a cheap correction instead of a full
        // high-precision division. Trig reduction calls this on hot paths; the
        // quotient/residual structure is the same problem addressed by
        // Payne-Hanek range reduction.
        let mut multiple = self.integer_ratio_nearest(Self::pi());
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::SIX {
            multiple += 1;
        } else if rough_appr <= -signed::SIX.clone() {
            multiple -= 1;
        }

        multiple
    }

    /// Calculate nearby multiple of pi/2.
    pub(super) fn half_pi_multiple(&self) -> BigInt {
        // Same nearest-multiple trick as `pi_multiple`, specialized for the quadrant
        // reductions used by sin/cos. Exact-rational inputs first try an integer
        // quotient against cached pi so huge arguments avoid constructing
        // x*(pi/2)^-1, then the residual correction validates the quadrant.
        let half_pi = Self::pi().shift_right(1);
        let mut multiple = self
            .exact_rational()
            .and_then(|rational| Self::half_pi_multiple_exact_rational(&rational))
            .unwrap_or_else(|| self.integer_ratio_nearest(half_pi.clone()));
        let adjustment =
            half_pi.multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::TWO {
            multiple += 1;
        } else if rough_appr <= -signed::TWO.clone() {
            multiple -= 1;
        }

        multiple
    }

    pub(super) fn half_pi_multiple_exact_rational(rational: &Rational) -> Option<BigInt> {
        // Large exact rationals are the hot scalar sin/cos construction path.
        // Estimate round(2*x/pi) with one cached pi approximation and integer
        // arithmetic, then let half_pi_multiple's residual correction validate
        // the result. This avoids building and approximating x * (pi/2)^-1.
        // It is a lightweight exact-rational variant of Payne-Hanek radian reduction.
        let msd = rational.msd_exact()?;
        if msd < 3 {
            return None;
        }

        let precision_bits = msd.checked_add(16)?.max(16);
        let shift = usize::try_from(precision_bits).ok()?;
        let pi_scaled = Self::pi().approx(-precision_bits).to_biguint()?;
        let numerator = rational.numerator() << (shift + 1);
        let denominator = rational.denominator() * &pi_scaled;
        if denominator.is_zero() {
            return None;
        }

        let rounded = (&numerator + (&denominator >> 1_usize)) / &denominator;
        Some(BigInt::from_biguint(rational.sign(), rounded))
    }

    fn medium_half_pi_multiple(rough_appr: &BigInt) -> BigInt {
        // For medium arguments the rough approximation already distinguishes the only
        // useful half-pi multiples, avoiding a second approximation of x/(pi/2).
        let positive = rough_appr.sign() != Sign::Minus;
        let magnitude = rough_appr.magnitude();
        let multiple = if magnitude < unsigned::FIVE.deref() {
            signed::ONE.clone()
        } else {
            signed::TWO.clone()
        };

        if positive { multiple } else { -multiple }
    }

    fn known_msd_for_trig_reduction(&self) -> Option<Option<Precision>> {
        // Trig construction used to call `approx(-1)` before deciding whether
        // an argument was already small or definitely huge. Exact rationals and
        // shared constants already carry enough structural MSD information, so
        // using it here avoids an extra approximation pass for generic scalar
        // sin/cos rows such as 1e6 and 1e30.
        match &self.internal.approximation {
            Approximation::One => Some(Some(0)),
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                None
            } else {
                Some(n.magnitude().bits() as Precision - 1)
            }),
            Approximation::Ratio(r) => Some(r.msd_exact()),
            Approximation::Constant(constant) => constant.bound_info().known_msd(),
            Approximation::Negate(child) => child.cheap_bound().known_msd(),
            Approximation::Offset(child, n) => child
                .cheap_bound()
                .known_msd()
                .map(|msd| msd.map(|value| value + *n)),
            _ => self.cheap_bound().known_msd(),
        }
    }

    fn trig_reduction_msd(&self) -> Option<Precision> {
        self.known_msd_for_trig_reduction().flatten()
    }

    fn exact_rational_half_pi_shortcut_magnitude(rational: &Rational) -> Option<Rational> {
        // Exact rationals with 1 <= |x| < 3/2 are the awkward medium trig rows:
        // not small enough for direct sin/cos, but close enough to pi/2 that a
        // dedicated residual beats full half-pi reduction and generic Add setup.
        if rational.msd_exact() != Some(0) {
            return None;
        }

        let magnitude = if rational.sign() == Sign::Minus {
            -rational.clone()
        } else {
            rational.clone()
        };
        if magnitude < *HALF_PI_SHORTCUT_RATIONAL_LIMIT {
            Some(magnitude)
        } else {
            None
        }
    }

    fn cos_reduced_by_half_pi(self, multiplier: BigInt) -> Computable {
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        // Reduce the nearest half-pi multiple modulo four and dispatch by the
        // exact sin/cos symmetry. This keeps the residual kernel below one radian.
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if quadrant.is_zero() {
            reduced.cos()
        } else if quadrant == *signed::ONE {
            reduced.sin().negate()
        } else if quadrant == *signed::TWO {
            reduced.cos().negate()
        } else {
            reduced.sin()
        }
    }

    /// Cosine of this number.
    pub fn cos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "cos", "exact-zero-one");
                return Self::one();
            }
            if rational.magnitude_at_least_power_of_two(2) {
                crate::trace_dispatch!("computable", "cos", "large-rational-deferred");
                return Self::cos_large_rational_deferred(rational);
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // cos(r) = sin(pi/2 - |r|) for exact medium positive/negative
                // rationals, keeping the generic subtraction node out of the path.
                crate::trace_dispatch!("computable", "cos", "medium-rational-half-pi-rewrite");
                return Self::prescaled_sin_half_pi_minus_rational(magnitude);
            }
        }
        if let Some((multiple, residual)) = self.integer_pi_plus_rational() {
            crate::trace_dispatch!("computable", "cos", "integer-pi-plus-rational");
            let reduced = Self::rational(residual).cos();
            return if (&multiple % signed::TWO.deref()).is_zero() {
                reduced
            } else {
                reduced.negate()
            };
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd <= 0 {
                // Known |x| < 2: go directly to the prescaled Taylor kernel.
                // The fallback rough approximation stays in place for unknown
                // magnitudes where structural bounds are not trustworthy.
                crate::trace_dispatch!("computable", "cos", "structural-small-prescaled");
                return Self::prescaled_cos(self);
            }
            if msd >= 3 {
                // Known |x| >= 8: skip the preliminary `approx(-1)` and go
                // straight to half-pi reduction. This is the hot large-argument
                // path for generic sin/cos benchmarks.
                let multiplier = Self::half_pi_multiple(&self);
                crate::trace_dispatch!("computable", "cos", "structural-large-half-pi-reduction");
                return self.cos_reduced_by_half_pi(multiplier);
            }
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "cos", "rough-small-prescaled");
            return Self::prescaled_cos(self);
        }

        let multiplier = if abs_rough_appr < unsigned::SIX.deref() {
            // Medium arguments can reuse the rough quadrant table. Larger values need the
            // more expensive nearest-half-pi reduction to keep the residual small.
            crate::trace_dispatch!("computable", "cos", "rough-medium-half-pi-reduction");
            Self::medium_half_pi_multiple(&rough_appr)
        } else {
            crate::trace_dispatch!("computable", "cos", "generic-half-pi-reduction");
            Self::half_pi_multiple(&self)
        };
        self.cos_reduced_by_half_pi(multiplier)
    }

    /// Sine of this number.
    pub fn sin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "sin", "exact-zero");
                return Self::zero();
            }
            if rational.magnitude_at_least_power_of_two(2) {
                crate::trace_dispatch!("computable", "sin", "large-rational-deferred");
                return Self::sin_large_rational_deferred(rational);
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // sin(r) = +/-cos(pi/2 - |r|) in the same exact medium window
                // used by cosine, preserving odd symmetry outside the kernel.
                crate::trace_dispatch!("computable", "sin", "medium-rational-half-pi-rewrite");
                let result = Self::prescaled_cos_half_pi_minus_rational(magnitude);
                return if rational.sign() == Sign::Minus {
                    result.negate()
                } else {
                    result
                };
            }
        }
        if let Some((multiple, residual)) = self.integer_pi_plus_rational() {
            crate::trace_dispatch!("computable", "sin", "integer-pi-plus-rational");
            let reduced = Self::rational(residual).sin();
            return if (&multiple % signed::TWO.deref()).is_zero() {
                reduced
            } else {
                reduced.negate()
            };
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd <= 0 {
                // Known |x| < 2: direct prescaled sine avoids reduction setup.
                crate::trace_dispatch!("computable", "sin", "structural-small-prescaled");
                return Self::prescaled_sin(self);
            }
            if msd >= 3 {
                // Known large input: avoid the extra rough approximation and
                // reduce by half-pi immediately.
                let multiplier = Self::half_pi_multiple(&self);
                let adjustment = Self::pi()
                    .shift_right(1)
                    .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
                let reduced = self.add(adjustment);
                let quadrant = ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref())
                    % signed::FOUR.deref();

                crate::trace_dispatch!("computable", "sin", "structural-large-half-pi-reduction");
                if quadrant.is_zero() {
                    return reduced.sin();
                } else if quadrant == *signed::ONE {
                    return reduced.cos();
                } else if quadrant == *signed::TWO {
                    return reduced.sin().negate();
                } else {
                    return reduced.cos().negate();
                }
            }
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "sin", "rough-small-prescaled");
            return Self::prescaled_sin(self);
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // Medium sine inputs are rewritten through exact symmetries instead of going
            // through the generic half-pi division path.
            let multiplier = Self::medium_half_pi_multiple(&rough_appr);
            crate::trace_dispatch!("computable", "sin", "rough-medium-special-rewrite");
            if multiplier == *signed::ONE {
                return Self::pi().shift_right(1).add(self.negate()).cos();
            } else if multiplier == *signed::MINUS_ONE {
                return Self::pi().shift_right(1).add(self).cos().negate();
            } else if multiplier == *signed::TWO {
                return Self::pi().add(self.negate()).sin();
            } else {
                return Self::pi().add(self).sin().negate();
            }
        }

        let multiplier = Self::half_pi_multiple(&self);
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        crate::trace_dispatch!("computable", "sin", "generic-half-pi-reduction");
        if quadrant.is_zero() {
            reduced.sin()
        } else if quadrant == *signed::ONE {
            reduced.cos()
        } else if quadrant == *signed::TWO {
            reduced.sin().negate()
        } else {
            reduced.cos().negate()
        }
    }

    /// Tangent of this number.
    pub fn tan(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "tan", "exact-zero");
                return Self::zero();
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                crate::trace_dispatch!("computable", "tan", "medium-rational-half-pi-cotangent");
                let result = Self::prescaled_cot_half_pi_minus_rational(magnitude);
                return if rational.sign() == Sign::Minus {
                    result.negate()
                } else {
                    result
                };
            }
            if rational.compare_magnitude(&NEAR_LARGE_RATIONAL_TRIG_THRESHOLD) != Ordering::Less {
                crate::trace_dispatch!("computable", "tan", "near-large-rational-deferred");
                return Self::tan_large_rational_deferred(rational);
            }
            if rational.magnitude_at_least_power_of_two(2) {
                crate::trace_dispatch!("computable", "tan", "large-rational-deferred");
                return Self::tan_large_rational_deferred(rational);
            }
        }
        if let Some((_multiple, residual)) = self.integer_pi_plus_rational() {
            // tan has period pi, so any exact integer pi multiple drops out.
            crate::trace_dispatch!("computable", "tan", "integer-pi-plus-rational");
            return Self::rational(residual).tan();
        }
        if self.planning_sign_and_msd().0 == Some(Sign::Minus) {
            // Odd symmetry lets known-negative values reuse the positive reducer
            // without paying a low-precision approximation just to discover sign.
            crate::trace_dispatch!("computable", "tan", "known-negative-symmetry");
            return self.negate().tan().negate();
        }
        if let Some(msd) = self.trig_reduction_msd()
            && msd <= 0
        {
            // Known |x| < 2: enter the tangent quotient kernel directly.
            crate::trace_dispatch!("computable", "tan", "structural-small-prescaled");
            return Self {
                internal: Arc::new(Node::new(Approximation::PrescaledTan(self), BoundCache::Invalid, ExactSignCache::Invalid)),
                signal: None,
            };
        }
        let rough_appr = self.approx(-1);
        if rough_appr.sign() == Sign::Minus {
            crate::trace_dispatch!("computable", "tan", "rough-negative-symmetry");
            return self.negate().tan().negate();
        }

        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "tan", "rough-small-prescaled");
            return Self {
                internal: Arc::new(Node::new(Approximation::PrescaledTan(self), BoundCache::Invalid, ExactSignCache::Invalid)),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::FIVE.deref() {
            // Near pi/2, cotangent of the complement converges faster and avoids the
            // unstable generic tan series at the pole.
            let complement = Self::pi().shift_right(1).add(self.negate());
            crate::trace_dispatch!("computable", "tan", "near-half-pi-cotangent-rewrite");
            return Self {
                internal: Arc::new(Node::new(Approximation::PrescaledCot(complement), BoundCache::Invalid, ExactSignCache::Invalid)),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // Near pi, reflect back to a small tangent argument.
            crate::trace_dispatch!("computable", "tan", "near-pi-reflection");
            return Self::pi().add(self.negate()).tan().negate();
        }

        let multiplier = Self::pi_multiple(&self);
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiplier)).negate());
        crate::trace_dispatch!("computable", "tan", "generic-pi-reduction");
        self.add(adjustment).tan()
    }

    pub(crate) fn sin_rational(rational: Rational) -> Computable {
        // Real-level rational trig already owns the Rational. Classify it here
        // so hot scalar constructors skip Ratio allocation followed by the same
        // exact-rational rediscovery inside Computable::sin.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "sin", "exact-zero");
            return Self::zero();
        }
        if rational.magnitude_at_least_power_of_two(3) {
            crate::trace_dispatch!("computable", "sin", "large-rational-deferred");
            return Self::sin_large_rational_deferred(rational);
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "sin", "medium-rational-half-pi-rewrite");
            let result = Self::prescaled_cos_half_pi_minus_rational(magnitude);
            return if rational.sign() == Sign::Minus {
                result.negate()
            } else {
                result
            };
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "sin", "structural-small-prescaled");
            return Self::prescaled_sin_rational(rational);
        }
        crate::trace_dispatch!("computable", "sin", "owned-rational-generic");
        Self::rational(rational).sin()
    }

    pub(crate) fn cos_rational(rational: Rational) -> Computable {
        // Owned rational cosine mirrors sin_rational. Keeping the branch table
        // shared at this level removes a constructor-only Ratio node from every
        // plain Real::cos(rational) call without changing approximation kernels.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "cos", "exact-zero-one");
            return Self::one();
        }
        if rational.magnitude_at_least_power_of_two(3) {
            crate::trace_dispatch!("computable", "cos", "large-rational-deferred");
            return Self::cos_large_rational_deferred(rational);
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "cos", "medium-rational-half-pi-rewrite");
            return Self::prescaled_sin_half_pi_minus_rational(magnitude);
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "cos", "structural-small-prescaled");
            return Self::prescaled_cos_rational(rational);
        }
        crate::trace_dispatch!("computable", "cos", "owned-rational-generic");
        Self::rational(rational).cos()
    }

    pub(crate) fn tan_rational(rational: Rational) -> Computable {
        // Tangent benefits most from classifying before Ratio construction: the
        // generic path probes sign/MSD and may build symmetry wrappers before it
        // reaches the small or near-pole kernel.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "tan", "exact-zero");
            return Self::zero();
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "tan", "medium-rational-half-pi-cotangent");
            let result = Self::prescaled_cot_half_pi_minus_rational(magnitude);
            return if rational.sign() == Sign::Minus {
                result.negate()
            } else {
                result
            };
        }
        if rational.magnitude_at_least_power_of_two(2) {
            crate::trace_dispatch!("computable", "tan", "large-rational-deferred");
            return Self::tan_large_rational_deferred(rational);
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "tan", "structural-small-prescaled");
            return Self::prescaled_tan_rational(rational);
        }
        crate::trace_dispatch!("computable", "tan", "owned-rational-generic");
        Self::rational(rational).tan()
    }
}
