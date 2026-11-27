impl Computable {
    /// Build e raised to a nonnegative integer by binary exponentiation of the shared e constant.
    /// The exp constructor uses this only for exponents 2 through 256 to retain a
    /// compact multiplication/square graph. Zero would return one. The number of
    /// loop steps is bounded by the u32 width, not the cost of later evaluation.
    fn exp_bounded_positive_integer_power(mut remaining: u32) -> Self {
        let mut base = Self::e_constant();
        let mut result = Self::one();

        while remaining != 0 {
            if remaining & 1 == 1 {
                result = result.multiply(base.clone());
            }
            remaining >>= 1;
            if remaining != 0 {
                base = base.square();
            }
        }

        result
    }

    /// Choose an integer n with |self/divisor - n| < 1 on completed, uninterrupted evaluation.
    /// Callers supply the positive shared divisor pi, pi/2 or ln(2). A guarded
    /// quotient of separate precision -4 approximations preserves the established
    /// selection; otherwise approximate the combined quotient at precision zero.
    /// Despite the name, this does not promise nearest-integer rounding or a unique
    /// answer. It may refine arbitrarily far and returns no cancellation receipt;
    /// an interrupted approximation cannot establish the stated error bound.
    fn integer_ratio_nearest(&self, divisor: Computable) -> BigInt {
        // A strict-error near-integer quotient: |self/divisor - n| < 1.
        // All callers use a positive shared divisor (pi, pi/2 or ln(2)).
        // The cheap path rounds separate 1/16 approximations N and D. Each is
        // off by less than one unit, so its rounded quotient is within 1 of
        // self/divisor when 2 * (|N| + |D|) < (|D| - 1)^2. Keep that
        // established selection there, so reduced representations and their
        // serialized forms are unchanged.
        let precision: Precision = -4;
        let numerator = self.approx(precision);
        let denominator = divisor.approx(precision);
        let abs_numerator = numerator.magnitude();
        let abs_denominator = denominator.magnitude();
        if *abs_denominator > BigUint::one()
            && (abs_numerator + abs_denominator) * 2_u32 < (abs_denominator - 1_u32).pow(2)
        {
            let mut quotient = abs_numerator / abs_denominator;
            if abs_numerator % abs_denominator >= (abs_denominator >> 1) {
                quotient += 1_u32;
            }
            let sign = if numerator.sign() == denominator.sign() {
                Sign::Plus
            } else {
                Sign::Minus
            };
            return BigInt::from_biguint(sign, quotient);
        }
        // Otherwise refine the quotient as one expression. That accounts for
        // divisor error at the numerator's magnitude, including huge opaque
        // amplitudes.
        self.clone()
            .multiply(divisor.inverse())
            .approx_signal(&self.signal, 0)
    }

    /// Try to express self as reduced + multiple*divisor with a small certified residual.
    /// For the positive divisors used by callers, adjust an initial near-integer
    /// quotient by one per attempt until the precision-low_prec residual integer
    /// lies in [-8,8]. Completed evaluation then gives |reduced| < 9*2^low_prec.
    /// Return None if no attempt accepts, including max_attempts=0; even that case
    /// computes the initial quotient first. The attempt count does not bound nested
    /// refinement, coefficient size or allocation, and is not a cancellation check.
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

    /// Test whether a cached exact nonzero binade proves |self| < 1/2 without evaluation.
    /// Accept an exact most-significant-bit exponent at most -2. Missing or inexact
    /// bounds, and even a cached exact zero, return false and leave the numeric
    /// kernel guard responsible for the range check. False is not a proof of largeness.
    pub(crate) fn exp_argument_is_known_small(&self) -> bool {
        self.cached_bound()
            .and_then(|bound| bound.known_msd())
            .flatten()
            .is_some_and(|msd| msd <= -2)
    }

    /// Try exp(self) = exp(reduced)*2^k using subtraction of k*ln(2).
    /// Allow 64 residual-correction attempts at precision -4, giving
    /// |reduced| < 9/16 after completed evaluation. Return None if correction fails
    /// or k does not fit the i32 binary-shift representation; neither means the
    /// mathematical exponential is undefined. Construction and the deferred kernel
    /// share this optional path. The attempt limit is not a total-work bound.
    pub(crate) fn reduced_exp(&self) -> Option<Self> {
        let (reduced, multiple) = self.reduce_by_divisor(&Self::ln2(), -4, 64)?;
        // Reduction is optional: a finite exact exponent can require a
        // binary scale larger than the node's i32 representation permits.
        // Retain the lazy exponential instead of panicking at construction.
        Some(
            reduced
                .prescaled_exp()
                .shift_left(multiple.try_into().ok()?),
        )
    }

    /// Retain an exponential node with a strict positive sign for a defined real operand.
    /// The name does not require a reduced argument: evaluation rechecks the range
    /// before using the small Taylor kernel. Do not approximate here. The new node
    /// starts without a numeric/magnitude cache or outer signal; its child retains
    /// its own handle. This helper does not validate malformed operand graphs.
    fn prescaled_exp(self) -> Self {
        // Preserve structural form while deferring the expensive approximation of
        // exp to the requested precision. This node may also hold an unreduced
        // operand; its kernel verifies the range before using the Taylor series.
        // exp(x) stays strictly positive across all domain values, so cache
        // that fact directly for fast sign/zero checks.
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledExp(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Construct the natural exponential e^self, retaining exact source structure.
    /// Recognized zero and one reuse constants; positive integers 2 through 256 use
    /// a shared-e power graph. Other inputs use a deferred exponential or ln(2)
    /// reduction with an exact binary scale. Planning bounds choose construction
    /// paths only; the approximation kernel checks its own valid range.
    ///
    /// The result denotes a positive real for every defined real input. Construction
    /// may approximate the argument to select a reduction, and an unrepresentable
    /// binary scale leaves a deferred node. Neither lazy construction nor the finite
    /// correction loop guarantees bounded evaluation work or memory.
    pub fn exp(self) -> Computable {
        let exact_rational = self.exact_rational();
        if exact_rational.as_ref().is_some_and(Rational::is_one) {
            // e^1 is the shared cached constant, not a fresh PrescaledExp node.
            crate::trace_dispatch!("computable", "exp", "exact-one-shared-e");
            return Self::e_constant();
        }
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            // e^0 is exact and must stay outside the approximation pipeline.
            crate::trace_dispatch!("computable", "exp", "exact-zero-one");
            return Self::one();
        }
        if let Some(exponent) = exact_rational
            .as_ref()
            .and_then(Rational::to_big_integer)
            .and_then(|integer| u32::try_from(integer).ok())
            .filter(|exponent| (2..=256).contains(exponent))
        {
            crate::trace_dispatch!("computable", "exp", "bounded-integer-e-power");
            return Self::exp_bounded_positive_integer_power(exponent);
        }
        if let Some(msd) = self.planning_sign_and_msd().1.flatten() {
            if msd <= 2 {
                // This is only a construction-cost shortcut, not a proof that
                // the argument is inside the small Taylor kernel's domain.
                crate::trace_dispatch!("computable", "exp", "structural-small-prescaled");
                return self.prescaled_exp();
            }
            if msd >= 4 {
                crate::trace_dispatch!("computable", "exp", "structural-large-range-reduction");
                if let Some(reduced) = self.reduced_exp() {
                    crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                    return reduced;
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
            if let Some(reduced) = self.reduced_exp() {
                crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                return reduced;
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

    /// Construct exp(self) - 1 with a dedicated small-argument representation.
    /// Recognized exact zero returns zero. Otherwise retain the operand and reuse
    /// its exact sign when available, since exp(x)-1 has the sign of real x.
    /// The kernel can evaluate near zero without subtracting rounded near-one
    /// values, and checks the range before choosing its series or general fallback.
    /// The new wrapper has no outer signal and starts without a magnitude cache;
    /// this is not a total-work or cancellation-outcome API.
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
            internal: Arc::new(Node::new(
                Approximation::Expm1(self),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Select an integer pi multiple for tangent reduction, with one residual correction.
    /// Start with integer_ratio_nearest(pi), approximate the residual at precision
    /// -1 and move one period when that integer reaches +6 or -6. This preserves
    /// the established reduction choice; it does not promise the uniquely nearest
    /// multiple, and its nested approximations have no total-work limit.
    fn pi_multiple(&self) -> BigInt {
        // Try a low-precision quotient before combined quotient refinement, then
        // apply a residual correction. Trig reduction calls this on hot paths; the
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

    /// Select a half-pi multiple for sine/cosine reduction with one residual correction.
    /// Try the exact-rational quotient estimate first; otherwise use the combined
    /// near-integer quotient. At precision -1, residual integers at least 2 or at
    /// most -2 move the multiple by one. The result selects a small residual for
    /// the kernels, not a certified nearest-integer rounding of 2*self/pi.
    /// The mathematical reduction assumes completed, uninterrupted approximations.
    pub(super) fn half_pi_multiple(&self) -> BigInt {
        // Same near-integer reduction as `pi_multiple`, specialized for the quadrant
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

    /// Estimate the half-pi quotient of an exact rational using sixteen guard bits.
    /// Delegate to the guarded estimator; callers must still apply their residual
    /// correction. None requests the generic path rather than asserting a small
    /// value or a failed mathematical reduction.
    pub(super) fn half_pi_multiple_exact_rational(rational: &Rational) -> Option<BigInt> {
        Self::half_pi_multiple_exact_rational_with_guard(rational, 16)
    }

    /// Estimate round(2*rational/pi) using cached pi and integer arithmetic.
    /// Require a nonzero rational with exact binade exponent at least 3. Approximate
    /// pi with max(16, quotient_guard_bits) extra bits beyond that exponent, round
    /// the nonnegative quotient by adding half the integer denominator, then restore
    /// the rational's sign. This is an estimate, not a proof of correct rounding;
    /// callers correct the residual before selecting a kernel quadrant.
    ///
    /// Return None for an inapplicable magnitude, precision overflow/conversion
    /// failure, nonpositive pi approximation or zero denominator. The guard argument
    /// is a precision choice, not a total-work or memory budget. The helper has no
    /// explicit signal polling and can request a large pi approximation/allocation.
    pub(super) fn half_pi_multiple_exact_rational_with_guard(
        rational: &Rational,
        quotient_guard_bits: Precision,
    ) -> Option<BigInt> {
        // Large exact rationals are the hot scalar sin/cos construction path.
        // Estimate round(2*x/pi) with one cached pi approximation and integer
        // arithmetic, then let half_pi_multiple's residual correction validate
        // the result. This avoids building and approximating x * (pi/2)^-1.
        // It is a lightweight exact-rational variant of Payne-Hanek radian reduction.
        let msd = rational.msd_exact()?;
        if msd < 3 {
            return None;
        }

        let precision_bits = msd.checked_add(quotient_guard_bits.max(16))?.max(16);
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

    /// Choose a signed half-pi multiple from an already available precision -1 approximation.
    /// Callers restrict its magnitude to 2 through 5: magnitudes below 5 select one
    /// half-pi, and 5 selects two, with the rough sign restored. This table avoids
    /// another quotient approximation; it is not a general range reducer and does
    /// not validate that caller range.
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

    /// Read an exact binade for trig dispatch without numerically approximating self.
    /// Return Some(Some(e)) for a known nonzero magnitude, Some(None) for exact zero,
    /// and None when this path lacks an exact magnitude. Inspect native integers,
    /// rationals and constants directly; use cheap child bounds for negation/shift
    /// and cheap bounds for other nodes. A shift translates the exponent.
    /// The integer bit-count cast and shifted-exponent addition are unchecked;
    /// this helper does not establish safety for every extreme representation.
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

    /// Extract a known nonzero binade for the small/large trig dispatch tests.
    /// Flatten both an unknown magnitude and exact zero to None; zero handling
    /// belongs to the constructors' earlier identity branches.
    fn trig_reduction_msd(&self) -> Option<Precision> {
        self.known_msd_for_trig_reduction().flatten()
    }

    /// Return the exact nonnegative magnitude of an owned rational, negating only negative values.
    fn rational_magnitude(value: Rational) -> Rational {
        if value.sign() == Sign::Minus {
            value.neg()
        } else {
            value
        }
    }

    /// Return a rational upper bound on |self|/pi for recognized, admitted real graphs.
    /// Use principal inverse-trig ranges, rational magnitudes, pi/tau/inverse-pi,
    /// negation, binary shifts in [-128,128], sums and multiplication by an exact
    /// rational. The triangle inequality may make the bound loose. Inverse-trig
    /// tags rely on their admission preconditions; this is not a domain validator
    /// for arbitrary raw syntax. Unknown forms or exhausted depth return None.
    ///
    /// budget limits recursive depth, not total visits: each branch receives the
    /// remaining depth independently. Rational arithmetic and exact-rational
    /// recognition have no coefficient-size or total-work bound here. No numerical
    /// approximation of self is needed to establish a successful bound.
    fn trig_pi_magnitude_upper_bound(&self, budget: usize) -> Option<Rational> {
        if budget == 0 {
            return None;
        }
        let next_budget = budget - 1;

        match &self.internal.approximation {
            // pi > 1, so every exact rational r satisfies |r| <= |r| * pi.
            Approximation::One => Some(Rational::one()),
            Approximation::Int(value) => Some(Self::rational_magnitude(Rational::from_bigint(
                value.clone(),
            ))),
            Approximation::Ratio(value) => Some(Self::rational_magnitude(value.clone())),
            Approximation::Constant(SharedConstant::Pi) => Some(Rational::one()),
            Approximation::Constant(SharedConstant::Tau) => Some(Rational::new(2)),
            Approximation::Constant(SharedConstant::InvPi) => Some(Rational::one()),
            // These constructors are admitted only for inputs in [0, 1], so
            // their results lie in [0, pi/2].
            Approximation::AcosPositive(_) | Approximation::AcosPositiveRational(_) => {
                Some(HALF_RATIONAL.clone())
            }
            // acos(-x), x in (0, 1], lies in [pi/2, pi].
            Approximation::AcosNegativeRational(_) => Some(Rational::one()),
            // Every admitted asin representation lies in [-pi/2, pi/2]. This
            // also covers acos of tiny rationals, which is retained internally
            // as pi/2 - asin(x).
            Approximation::AsinRational(_)
            | Approximation::PrescaledAsin(_)
            | Approximation::AsinDeferred(_) => Some(HALF_RATIONAL.clone()),
            Approximation::Negate(child) => child.trig_pi_magnitude_upper_bound(next_budget),
            Approximation::Offset(child, shift) => {
                // Keep this proof helper cheap even for deliberately enormous
                // symbolic shifts; the ordinary magnitude/reduction path can
                // handle those without materializing a giant Rational bound.
                if !(-128..=128).contains(shift) {
                    return None;
                }
                let bound = child.trig_pi_magnitude_upper_bound(next_budget)?;
                Some(bound * Self::power_of_two_rational(*shift))
            }
            Approximation::Add(left, right) => {
                let left = left.trig_pi_magnitude_upper_bound(next_budget)?;
                let right = right.trig_pi_magnitude_upper_bound(next_budget)?;
                Some(left + right)
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    let bound = right.trig_pi_magnitude_upper_bound(next_budget)?;
                    Some(bound * Self::rational_magnitude(scale))
                } else if let Some(scale) = right.exact_rational() {
                    let bound = left.trig_pi_magnitude_upper_bound(next_budget)?;
                    Some(bound * Self::rational_magnitude(scale))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Return |rational| exactly when 1 <= |rational| < 3/2; otherwise decline.
    /// This medium window permits a dedicated pi/2-|rational| residual for sine,
    /// cosine or cotangent, avoiding a generic subtraction/reduction graph. The
    /// lower boundary is included, and the upper boundary is excluded.
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

    /// Reconstruct cosine from residual self - multiplier*pi/2 and exact quadrant symmetry.
    /// For multiplier modulo four, select cos, -sin, -cos or sin of the residual.
    /// Try a retained pi/arctangent identity before allocating a prescaled kernel.
    /// The caller selects a multiplier giving a valid small kernel argument; this
    /// helper neither proves that range nor checks a supplied multiplier.
    fn cos_reduced_by_half_pi(self, multiplier: BigInt) -> Computable {
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        // Reduce the selected half-pi multiple modulo four and dispatch by the
        // exact sin/cos symmetry. The caller owns the small-residual precondition.
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if let Some((sine, cosine)) = reduced.pi_atan_sin_cos() {
            crate::trace_dispatch!("computable", "cos", "reduced-pi-atan-rewrite");
            return if quadrant.is_zero() {
                cosine
            } else if quadrant == *signed::ONE {
                sine.negate()
            } else if quadrant == *signed::TWO {
                cosine.negate()
            } else {
                sine
            };
        }
        if quadrant.is_zero() {
            Self::prescaled_cos(reduced)
        } else if quadrant == *signed::ONE {
            Self::prescaled_sin(reduced).negate()
        } else if quadrant == *signed::TWO {
            Self::prescaled_cos(reduced).negate()
        } else {
            Self::prescaled_sin(reduced)
        }
    }

    /// Reconstruct sine from residual self - multiplier*pi/2 and exact quadrant symmetry.
    /// For multiplier modulo four, select sin, cos, -sin or -cos of the residual.
    /// Prefer a retained pi/arctangent identity, then a prescaled kernel. As for
    /// cos_reduced_by_half_pi, the caller owns the residual-range precondition;
    /// negative multipliers use the same nonnegative modulo-four quadrant.
    fn sin_reduced_by_half_pi(self, multiplier: BigInt) -> Computable {
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if let Some((sine, cosine)) = reduced.pi_atan_sin_cos() {
            crate::trace_dispatch!("computable", "sin", "reduced-pi-atan-rewrite");
            return if quadrant.is_zero() {
                sine
            } else if quadrant == *signed::ONE {
                cosine
            } else if quadrant == *signed::TWO {
                sine.negate()
            } else {
                cosine.negate()
            };
        }
        if quadrant.is_zero() {
            Self::prescaled_sin(reduced)
        } else if quadrant == *signed::ONE {
            Self::prescaled_cos(reduced)
        } else if quadrant == *signed::TWO {
            Self::prescaled_sin(reduced).negate()
        } else {
            Self::prescaled_cos(reduced).negate()
        }
    }

    /// Recover the retained principal arctangent operand from native rational or generic atan tags.
    /// IntegralAtan stores atan(1/n), so invert its nonzero integer parameter.
    /// Prescaled/deferred tags return the shared child. Decline other shapes,
    /// including an outer negation or an angle already normalized to a pi multiple;
    /// None does not disprove that the value is an arctangent.
    fn atan_argument(&self) -> Option<Computable> {
        match &self.internal.approximation {
            Approximation::AtanRational(argument) => Some(Self::rational(argument.clone())),
            Approximation::IntegralAtan(_) => self.atan_rational_argument().map(Self::rational),
            Approximation::PrescaledAtan(argument) | Approximation::AtanDeferred(argument) => {
                Some(argument.clone())
            }
            _ => None,
        }
    }

    /// Recover a rational x from retained atan(x), including recursively negated forms.
    /// Read AtanRational directly and IntegralAtan(n) as 1/n, preserving its sign;
    /// a zero n declines through checked rational inversion. Negation uses atan's
    /// odd symmetry. Other representations decline. This recursive reader has no
    /// explicit depth or coefficient-work limit.
    pub(crate) fn atan_rational_argument(&self) -> Option<Rational> {
        match &self.internal.approximation {
            Approximation::AtanRational(argument) => Some(argument.clone()),
            // IntegralAtan retains atan(1/n), including the sign of n.
            // Share this exact inversion with native and Real consumers.
            Approximation::IntegralAtan(denominator) => {
                Rational::from_bigint(denominator.clone()).inverse().ok()
            }
            Approximation::Negate(argument) => argument.atan_rational_argument().map(Rational::neg),
            _ => None,
        }
    }

    /// Recover (scale, radicand) when a native atan operand is recognized as scale*sqrt(radicand).
    /// Use atan_argument followed by the shared exact pure-surd projection, which
    /// requires no additive rational part. This does not recover every equivalent
    /// angle, prove irrationality, or impose a canonical squarefree radicand.
    pub(crate) fn atan_pure_quadratic_surd_argument(&self) -> Option<(Rational, Rational)> {
        self.atan_argument()?.exact_pure_quadratic_surd()
    }

    /// Recover the retained principal arcsine operand from rational, prescaled or deferred asin tags.
    /// Return a rational node or clone the existing child. Other shapes decline.
    /// The inverse identity assumes an admitted argument in [-1,1]; this reader
    /// does not revalidate the domain or normalize arbitrary equivalent angles.
    pub(crate) fn asin_argument(&self) -> Option<Computable> {
        match &self.internal.approximation {
            Approximation::AsinRational(argument) => Some(Self::rational(argument.clone())),
            Approximation::PrescaledAsin(argument) | Approximation::AsinDeferred(argument) => {
                Some(argument.clone())
            }
            _ => None,
        }
    }

    /// Recover the retained principal arccosine operand from its three native acos tags.
    /// The negative-rational tag stores a positive magnitude, so negate it to
    /// recover the authored argument; positive tags preserve their operand.
    /// Domain admission is a caller invariant, not checked here. Other forms,
    /// including a pi/2-minus-asin encoding, decline.
    pub(crate) fn acos_argument(&self) -> Option<Computable> {
        match &self.internal.approximation {
            Approximation::AcosPositive(argument) => Some(argument.clone()),
            Approximation::AcosPositiveRational(argument) => Some(Self::rational(argument.clone())),
            Approximation::AcosNegativeRational(argument) => {
                Some(Self::rational(argument.clone().neg()))
            }
            _ => None,
        }
    }

    /// Construct the positive sqrt(1+argument²) used by atan sine/cosine identities.
    /// For a defined real argument this is at least one, so its inverse is defined.
    /// The exact square graph retains the operand relation for later proofs.
    fn unit_hypotenuse(argument: &Computable) -> Computable {
        Self::one().add(argument.clone().square()).sqrt()
    }

    /// Construct the nonnegative sqrt(1-argument²) for principal asin/acos complements.
    /// Callers must establish |argument| <= 1. This helper performs no typed domain
    /// check; selecting the nonnegative root is essential to the principal-angle
    /// identity, including zero at either endpoint.
    fn unit_complement(argument: &Computable) -> Computable {
        Self::one().add(argument.clone().square().negate()).sqrt()
    }

    /// Return exact sine/cosine expressions for recognized half-pi rotations of a principal arctangent.
    /// Enable recovery of the three normalized rational-pi anchors as well. Use
    /// x/sqrt(1+x²) and 1/sqrt(1+x²), exchanging them for odd half-pi rotations and
    /// applying the recognizer's signs. The inverse hypotenuse is shared between
    /// outputs. None is a structural-recognition decline, not a numerical result.
    fn pi_atan_sin_cos(&self) -> Option<(Computable, Computable)> {
        let (swap, sine_sign, cosine_sign, argument) = self.pi_atan_trig_argument(true)?;
        let inverse_hypotenuse = Self::unit_hypotenuse(&argument).inverse();
        let sine = argument.multiply(inverse_hypotenuse.clone());
        // For an odd multiple k of pi/2, sin(k*pi/2+/-atan(a)) and
        // cos(k*pi/2+/-atan(a)) exchange the factors of atan(a); the signs
        // come from the exact rotation.
        let (sine, cosine) = if swap {
            (inverse_hypotenuse, sine)
        } else {
            (sine, inverse_hypotenuse)
        };
        Some((
            if sine_sign == Sign::Minus {
                sine.negate()
            } else {
                sine
            },
            if cosine_sign == Sign::Minus {
                cosine.negate()
            } else {
                cosine
            },
        ))
    }

    /// Apply angle addition for a recognized odd k*pi/4 plus or minus atan(x).
    /// The recognizer supplies the diagonal sine/cosine signs and the atan sign.
    /// Build both numerators over the shared positive sqrt(2*(1+x²)); this retains
    /// exact dependence on x instead of numerically evaluating the angle. Decline
    /// when the structural recognizer cannot establish that form.
    fn diagonal_turn_atan_sin_cos(&self) -> Option<(Computable, Computable)> {
        let (sine_sign, cosine_sign, atan_sign, argument) =
            self.diagonal_turn_plus_or_minus_atan_argument()?;
        let signed = |value: Computable, sign| {
            if sign == Sign::Minus {
                value.negate()
            } else {
                value
            }
        };
        // For theta=+/-atan(x), cos(theta)=1/h and sin(theta)=+/-x/h,
        // h=sqrt(1+x*x)>0. A diagonal rotation has components +/-1/sqrt(2).
        // The addition identities therefore give both complete exact values.
        let inverse_denominator = Self::one()
            .add(argument.clone().square())
            .shift_left(1)
            .sqrt()
            .inverse();
        let argument = signed(argument, atan_sign);
        let sine = signed(Self::one(), sine_sign)
            .add(signed(argument.clone(), cosine_sign))
            .multiply(inverse_denominator.clone());
        let cosine = signed(Self::one(), cosine_sign)
            .add(signed(argument, sine_sign).negate())
            .multiply(inverse_denominator);
        Some((sine, cosine))
    }

    /// Construct cosine of a defined real angle measured in radians.
    /// Reuse retained zero, even symmetry and recognized principal inverse-trig
    /// relations before choosing rational residuals or half-pi range reduction.
    /// Preserve the original argument in deferred kernels when a shortcut does not
    /// apply; numerical approximation remains a separate request.
    ///
    /// Construction itself may approximate the argument to choose a quadrant.
    /// Structural recognition is intentionally partial, and neither a small range
    /// proof nor a deferred node supplies a total-work or memory bound.
    pub fn cos(self) -> Computable {
        if self.immediate_sign() == Some(RealSign::Zero) {
            crate::trace_dispatch!("computable", "cos", "retained-zero-one");
            let mut result = Self::one();
            result.signal = self.signal;
            return result;
        }
        if let Approximation::Negate(argument) = &self.internal.approximation {
            crate::trace_dispatch!("computable", "cos", "negative-even-rewrite");
            return argument.clone().cos();
        }
        if let Some((_, argument)) = self.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!("computable", "cos", "acos-minus-half-pi-rewrite");
            return Self::unit_complement(&argument);
        }
        if let Some(argument) = self.atan_argument() {
            crate::trace_dispatch!("computable", "cos", "atan-inverse-rewrite");
            return Self::unit_hypotenuse(&argument).inverse();
        }
        if let Some((_, cosine)) = self.pi_atan_sin_cos() {
            crate::trace_dispatch!("computable", "cos", "pi-atan-rewrite");
            return cosine;
        }
        if let Some((_, cosine)) = self.diagonal_turn_atan_sin_cos() {
            crate::trace_dispatch!("computable", "cos", "diagonal-turn-atan-rewrite");
            return cosine;
        }
        if let Some(argument) = self.asin_argument() {
            crate::trace_dispatch!("computable", "cos", "asin-complement-rewrite");
            return Self::unit_complement(&argument);
        }
        if let Some(argument) = self.acos_argument() {
            crate::trace_dispatch!("computable", "cos", "acos-inverse-rewrite");
            return argument;
        }
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
        if self
            .trig_pi_magnitude_upper_bound(32)
            .is_some_and(|bound| bound <= *HALF_RATIONAL)
        {
            crate::trace_dispatch!("computable", "cos", "structural-pi-bounded-prescaled");
            return Self::prescaled_cos(self);
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

    /// Construct sine of a defined real angle measured in radians.
    /// Reuse retained zero, odd symmetry and recognized principal inverse-trig
    /// relations before rational residuals or half-pi reduction. Exact quadrant
    /// symmetries select sine/cosine kernels while retaining source relations.
    /// Large rational arguments can defer reduction until evaluation.
    ///
    /// Construction may approximate other arguments to choose a reduction. A
    /// successful construction does not imply bounded evaluation work, and a
    /// failed structural match does not mean the corresponding identity is false.
    pub fn sin(self) -> Computable {
        if self.immediate_sign() == Some(RealSign::Zero) {
            crate::trace_dispatch!("computable", "sin", "retained-zero");
            let mut result = Self::zero();
            result.signal = self.signal;
            return result;
        }
        if let Approximation::Negate(argument) = &self.internal.approximation {
            crate::trace_dispatch!("computable", "sin", "negative-odd-rewrite");
            return argument.clone().sin().negate();
        }
        if let Some((orientation, argument)) = self.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!("computable", "sin", "acos-minus-half-pi-rewrite");
            return if orientation == Sign::Plus {
                argument.negate()
            } else {
                argument
            };
        }
        if let Some(argument) = self.atan_argument() {
            crate::trace_dispatch!("computable", "sin", "atan-inverse-rewrite");
            return argument
                .clone()
                .multiply(Self::unit_hypotenuse(&argument).inverse());
        }
        if let Some((sine, _)) = self.pi_atan_sin_cos() {
            crate::trace_dispatch!("computable", "sin", "pi-atan-rewrite");
            return sine;
        }
        if let Some((sine, _)) = self.diagonal_turn_atan_sin_cos() {
            crate::trace_dispatch!("computable", "sin", "diagonal-turn-atan-rewrite");
            return sine;
        }
        if let Some(argument) = self.asin_argument() {
            crate::trace_dispatch!("computable", "sin", "asin-inverse-rewrite");
            return argument;
        }
        if let Some(argument) = self.acos_argument() {
            crate::trace_dispatch!("computable", "sin", "acos-complement-rewrite");
            return Self::unit_complement(&argument);
        }
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
                crate::trace_dispatch!("computable", "sin", "structural-large-half-pi-reduction");
                return self.sin_reduced_by_half_pi(multiplier);
            }
        }
        if self
            .trig_pi_magnitude_upper_bound(32)
            .is_some_and(|bound| bound <= *HALF_RATIONAL)
        {
            crate::trace_dispatch!("computable", "sin", "structural-pi-bounded-prescaled");
            return Self::prescaled_sin(self);
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "sin", "rough-small-prescaled");
            return Self::prescaled_sin(self);
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // The rough quadrant table identifies the exact half-pi symmetry.
            // Enter the already-reduced kernel directly; recursively invoking
            // public sin/cos here can bounce at a low-precision rounding
            // boundary for cancellation-heavy symbolic expressions.
            let multiplier = Self::medium_half_pi_multiple(&rough_appr);
            crate::trace_dispatch!("computable", "sin", "rough-medium-special-rewrite");
            return self.sin_reduced_by_half_pi(multiplier);
        }

        let multiplier = Self::half_pi_multiple(&self);
        crate::trace_dispatch!("computable", "sin", "generic-half-pi-reduction");
        self.sin_reduced_by_half_pi(multiplier)
    }

    /// Construct tangent of a real radian angle for which cosine is nonzero.
    /// Use rational residuals, exact integer-pi periodicity and odd symmetry before
    /// small tangent, near-half-pi cotangent, or general pi-reduction paths.
    /// Construction can evaluate the argument and recursively retry a reduced form.
    ///
    /// This low-level constructor returns no typed pole error and does not prove
    /// its non-pole precondition. Use [crate::Real::tan] for checked domain handling.
    /// In particular, a small magnitude alone does not exclude a half-pi pole,
    /// and lazy construction is not a promise of completed evaluation.
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
                internal: Arc::new(Node::new(
                    Approximation::PrescaledTan(self),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
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
                internal: Arc::new(Node::new(
                    Approximation::PrescaledTan(self),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::FIVE.deref() {
            // Near pi/2, cotangent of the complement converges faster and avoids the
            // unstable generic tan series at the pole.
            let complement = Self::pi().shift_right(1).add(self.negate());
            crate::trace_dispatch!("computable", "tan", "near-half-pi-cotangent-rewrite");
            return Self {
                internal: Arc::new(Node::new(
                    Approximation::PrescaledCot(complement),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
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

    /// Construct sine directly from an owned rational for checked-Real consumers.
    /// Avoid allocating a Ratio leaf merely to rediscover its exact rational.
    /// Handle zero, |x| >= 8, the [1,3/2) magnitude window and nonzero |x| < 1
    /// before the generic constructor. These thresholds preserve this entry
    /// point's established dispatch; they need not equal Computable::sin's table.
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

    /// Construct cosine directly from an owned rational for checked-Real consumers.
    /// Use one at zero, deferred large-rational handling for |x| >= 8, the medium
    /// half-pi complement and the nonzero |x| < 1 rational kernel before falling
    /// back to cos. Sharing kernels does not imply identical allocation or dispatch
    /// to constructing a generic rational node first.
    pub(crate) fn cos_rational(rational: Rational) -> Computable {
        // Owned rational cosine mirrors sin_rational's entry strategy. Its direct
        // branches avoid a constructor-only Ratio node, while the generic fallback
        // still constructs one. Both paths use the existing approximation kernels.
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

    /// Construct tangent directly from an owned rational, avoiding a redundant Ratio leaf.
    /// Try zero, the [1,3/2) magnitude complement, |x| >= 4 deferral and nonzero
    /// |x| < 1 before the generic path. Finite rational angles cannot be odd
    /// half-pi poles because pi is transcendental, but their evaluation costs are
    /// not bounded by these classification thresholds.
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
