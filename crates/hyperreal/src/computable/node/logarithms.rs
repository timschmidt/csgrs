impl Computable {
    /// Construct a wrapper for the shared natural-logarithm-of-two constant.
    /// The wrapper carries known positive facts; numerical refinement is reused
    /// through the constant cache shared across threads, not through wrapper identity.
    pub(crate) fn ln2() -> Self {
        Self::shared_constant(SharedConstant::Ln2)
    }

    /// Divide out all copies of the caller-supplied prime and return their exponent.
    /// Callers use 2, 3, 5 or 7. Mutate value to its remaining cofactor; zero is
    /// left unchanged with exponent zero. Require prime > 1 rather than validating
    /// it here. A factor count exceeding i32 panics after the corresponding division;
    /// there is no rollback, cancellation polling or total-work limit.
    fn factor_small_prime_power(value: &mut BigUint, prime: u32) -> i32 {
        let prime_big = BigUint::from(prime);
        let mut exponent = 0_i32;
        while !value.is_zero() && (&*value % &prime_big).is_zero() {
            *value /= &prime_big;
            exponent = exponent
                .checked_add(1)
                .expect("small-prime factor exponent should fit in i32");
        }
        exponent
    }

    /// Try to retain a positive 7-smooth rational's logarithm as a sum of shared prime logs.
    /// Factor canonical numerator and denominator over 2,3,5,7, then use their signed
    /// exponent differences in that order. Any remaining cofactor, nonpositive input,
    /// one, or a single prime to exponent +1 declines. The last exclusion prevents
    /// shared log constants from recursively requesting themselves during cache fill;
    /// None does not mean the logarithm is undefined.
    ///
    /// At most four terms are retained, but factoring arbitrary-size coefficients
    /// is not bounded by that count. Individual exponents must fit i32; the factor
    /// helper panics on overflow. Every emitted base has a shared ln constant, which
    /// justifies expect when assembling the sum.
    fn ln_smooth_rational(rational: &Rational) -> Option<Self> {
        if rational.sign() != Sign::Plus {
            return None;
        }

        let mut numerator = rational.numerator().clone();
        let mut denominator = rational.denominator().clone();
        let mut terms = Vec::with_capacity(4);
        for base in [2_u32, 3, 5, 7] {
            let exponent = Self::factor_small_prime_power(&mut numerator, base)
                - Self::factor_small_prime_power(&mut denominator, base);
            if exponent != 0 {
                terms.push((base, exponent));
            }
        }
        if numerator != BigUint::one() || denominator != BigUint::one() || terms.is_empty() {
            return None;
        }
        if terms.len() == 1 && terms[0].1 == 1 {
            // Shared log constants approximate themselves by evaluating the
            // corresponding exact rational log. Do not rewrite ln(3) to the
            // shared ln3 node here or that internal cache fill would recurse.
            // Composite smooth values such as 9, 6, 45/14 still reduce below.
            return None;
        }

        // ln(prod p_i^e_i) = sum e_i ln(p_i). Retaining this symbolic sum
        // lets smooth exact rationals reuse shared log caches and delays all
        // series evaluation until the final requested precision. This is the
        // same argument-reduction principle used by the elementary kernels
        // below, applied at construction time for common scalar/matrix constants.
        let mut result = Self::zero();
        for (base, exponent) in terms {
            let magnitude = BigInt::from(exponent.abs());
            let mut term = Self::ln_constant(base)
                .expect("smooth-log bases are all shared")
                .multiply(Self::integer(magnitude));
            if exponent < 0 {
                term = term.negate();
            }
            result = result.add(term);
        }
        Some(result)
    }

    /// Prefer shared ln constants for exact integers 2,3,5,6,7,10, then try the prime-log sum.
    /// Extract the small integer once and keep the established probe order. This
    /// helper returns None on other forms, including nonpositive inputs, without
    /// claiming a domain failure. Availability of the six constants justifies unwrap;
    /// smooth factoring retains its coefficient-work and exponent limits.
    fn ln_shared_or_smooth_rational(rational: &Rational) -> Option<Self> {
        // Shared logs for small smooth bases reuse one approximation cache
        // across all expressions. Extracting the integer once avoids
        // constructing several candidate rationals just to reject them.
        // Smooth-factor decomposition below remains exact.
        if let Some(integer) = rational.to_integer_i64() {
            match integer {
                2 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln2");
                    return Some(Self::ln_constant(2).unwrap());
                }
                3 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln3");
                    return Some(Self::ln_constant(3).unwrap());
                }
                5 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln5");
                    return Some(Self::ln_constant(5).unwrap());
                }
                6 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln6");
                    return Some(Self::ln_constant(6).unwrap());
                }
                7 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln7");
                    return Some(Self::ln_constant(7).unwrap());
                }
                10 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln10");
                    return Some(Self::ln_constant(10).unwrap());
                }
                _ => {}
            }
        }
        if let Some(reduced) = Self::ln_smooth_rational(rational) {
            crate::trace_dispatch!("computable", "ln", "smooth-rational-shared-log-sum");
            return Some(reduced);
        }
        None
    }

    /// Try exact binary reduction of ln(rational), assuming the caller supplied rational >= 1.
    /// Below 3/2 retain rational-1 directly. Otherwise choose an exact binade scale,
    /// increase it once if needed, and verify the scaled argument lies strictly
    /// between 1/2 and 3/2 before retaining residual=scaled-1 and the binary shift.
    /// The represented value is ln(1+residual)+shift*ln(2).
    ///
    /// None means a magnitude/shift operation or range check declined, not a proved
    /// invalid logarithm. The early branch does not check positivity, so this is
    /// not an input validator. No numerical evaluation is needed, but exact rational
    /// scaling/normalization is not covered by a total-work or allocation bound.
    fn ln_binary_scaled_exact_rational(rational: &Rational) -> Option<Self> {
        // If exact binary scaling already puts a nonsmooth rational in the
        // ln1p kernel's convergence window, skip the generic Ratio -> ln()
        // recursion. That avoids re-discovering the same dyadic scale and
        // constructing a short-lived Offset tree for adversarial ln(1+x^2)
        // cases while preserving the existing smooth-log fast paths above.
        let three_halves = THREE_HALVES_RATIONAL.deref();
        if rational < three_halves {
            let fraction = rational.subtract_one();
            return Some(Self::prescaled_ln_rational(fraction));
        }

        let mut shift = rational.msd_exact()?;
        if shift < 0 {
            return None;
        }

        let mut scaled = rational.divide_by_power_of_two(shift)?;

        if &scaled >= three_halves {
            shift = shift.checked_add(1)?;
            scaled = rational.divide_by_power_of_two(shift)?;
        }

        if &scaled <= HALF_RATIONAL.deref() || &scaled >= three_halves {
            return None;
        }

        let fraction = scaled.subtract_one();
        Some(Self::binary_scaled_ln_rational(fraction, shift))
    }

    /// Construct a logarithm from an owned positive rational after an internal reduction.
    /// Return zero at one, normalize values below one by -ln(1/r), then prefer shared
    /// or smooth logs and binary-scaled residuals before the generic constructor.
    /// Nonpositive input panics with ArithmeticException; this internal helper does
    /// not return Problem. The positive-domain check makes rational inversion safe,
    /// while optional representation declines preserve the fallback.
    fn ln_exact_rational(rational: Rational) -> Self {
        // Internal exact-rational log constructor for reductions that already
        // have a positive rational argument. It reuses the shared small-log
        // constants instead of building fresh generic PrescaledLn trees.
        if rational.is_one() {
            crate::trace_dispatch!("computable", "ln", "exact-rational-one");
            return Self::zero();
        }
        if rational.sign() == Sign::Minus || rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "ln", "exact-rational-domain-error");
            panic!("ArithmeticException");
        }
        if rational < Rational::one() {
            crate::trace_dispatch!("computable", "ln", "exact-rational-inverse-rewrite");
            return Self::ln_exact_rational(rational.inverse().unwrap()).negate();
        }
        if let Some(reduced) = Self::ln_shared_or_smooth_rational(&rational) {
            return reduced;
        }
        if let Some(reduced) = Self::ln_binary_scaled_exact_rational(&rational) {
            crate::trace_dispatch!("computable", "ln", "exact-rational-binary-scaled-ln1p");
            return reduced;
        }
        crate::trace_dispatch!("computable", "ln", "exact-rational-generic");
        Self::rational(rational).ln()
    }

    /// Construct the natural logarithm of a strictly positive, defined real value.
    /// Preserve exact one, rational residuals, smooth prime-log sums and dyadic scale
    /// where recognized. Other inputs use certified magnitude facts and numerical
    /// probes to choose inversion, repeated square roots or binary scaling before
    /// a retained ln(1+x) kernel. Planning estimates alone do not authorize a rewrite.
    ///
    /// This low-level constructor may evaluate its operand, recurse or panic; known
    /// negative inputs panic with ArithmeticException. It does not provide typed
    /// domain checking for nonpositive or undefined graphs. Use [crate::Real::ln]
    /// for checked domain handling. Shift arithmetic has explicit invariant expects,
    /// and there is no total-work/memory bound or cancellation outcome for construction.
    pub fn ln(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.is_one()) {
            crate::trace_dispatch!("computable", "ln", "exact-one-zero");
            return Self::zero();
        }
        if let Approximation::Ratio(r) = &self.internal.approximation
            && r.sign() == Sign::Plus
        {
            if r.numerator() >= r.denominator() && r < THREE_HALVES_RATIONAL.deref() {
                let fraction = r.subtract_one();
                crate::trace_dispatch!("computable", "ln", "exact-rational-direct-ln1p");
                return Self::prescaled_ln_rational(fraction);
            }
            if r.numerator() >= r.denominator() {
                if let Some(reduced) = Self::ln_smooth_rational(r) {
                    crate::trace_dispatch!("computable", "ln", "smooth-rational-shared-log-sum");
                    return reduced;
                }
                if let Some(reduced) = Self::ln_binary_scaled_exact_rational(r) {
                    crate::trace_dispatch!("computable", "ln", "exact-rational-binary-scaled-ln1p");
                    return reduced;
                }
            }
            let (shift, reduced) = r.factor_two_powers();
            if shift != 0 {
                // ln(r * 2^k) = ln(r) + k ln(2). Pulling dyadic scale out keeps
                // binary64-derived dyadics on a cheap symbolic/log path. The
                // reduced factor is routed through exact-rational log reduction
                // so smooth values like 45/14 become cached prime-log sums
                // instead of low-precision probing plus a fresh ln1p tree.
                let reduced_ln = if reduced.is_one() {
                    Self::integer(BigInt::zero())
                } else {
                    Self::ln_exact_rational(reduced)
                };
                let shift: BigInt = shift.into();
                crate::trace_dispatch!("computable", "ln", "dyadic-scale-rewrite");
                return reduced_ln.add(Self::integer(shift).multiply(Self::ln2()));
            } else if let Some(reduced) = Self::ln_smooth_rational(r) {
                crate::trace_dispatch!("computable", "ln", "smooth-rational-shared-log-sum");
                return reduced;
            }
        }

        // Sixteenths, ie 8 == 0.5, 24 == 1.5
        let low_ln_limit = signed::EIGHT.deref();
        let high_ln_limit = signed::TWENTY_FOUR.deref();

        let low_prec = -4;
        // A planning hint can drift through sums and reciprocals. Using one
        // as a range certificate can recurse on an unchanged zero-shift
        // operand; only certified magnitudes justify these rewrites.
        let (known_sign, certified_msd) = self.certified_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "ln", "domain-negative-structural");
            panic!("ArithmeticException");
        }
        if let Some(msd) = certified_msd.flatten() {
            if known_sign == Some(Sign::Plus) && msd <= -2 {
                // Rewriting ln(x) -> -ln(1/x) is safe below 1/2;
                // a certified msd <= -2 guarantees that without probing.
                crate::trace_dispatch!("computable", "ln", "small-inverse-rewrite-structural");
                return self.inverse().ln().negate();
            }
            if known_sign == Some(Sign::Plus) && msd >= 2 {
                // Structurally large exact values can be reduced by powers of
                // two before probing. This avoids building sqrt/sqrt/log graphs
                // for moderate exact-rational logs such as ln(1+x^2).
                let mut extra_bits: i32 = msd;

                let mut scaled = self.clone().shift_right(extra_bits);
                let mut scaled_rough = scaled.approx(low_prec);
                while scaled_rough <= *low_ln_limit {
                    extra_bits = extra_bits.checked_sub(1).expect(
                        "Approximation should have few enough bits to fit in a 32-bit signed integer",
                    );
                    scaled = self.clone().shift_right(extra_bits);
                    scaled_rough = scaled.approx(low_prec);
                }
                while scaled_rough >= *high_ln_limit {
                    extra_bits = extra_bits.checked_add(1).expect(
                        "Approximation should have few enough bits to fit in a 32-bit signed integer",
                    );
                    scaled = self.clone().shift_right(extra_bits);
                    scaled_rough = scaled.approx(low_prec);
                }

                let scaled_result = scaled.ln();
                let extra: BigInt = extra_bits.into();
                crate::trace_dispatch!("computable", "ln", "binary-scale-reduction");
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }
        let rough_appr = self.approx(low_prec);
        if rough_appr < BigInt::zero() {
            crate::trace_dispatch!("computable", "ln", "domain-negative");
            panic!("ArithmeticException");
        }
        if rough_appr <= *low_ln_limit {
            // A completed precision -4 sample at most 8 puts a positive input
            // below 9/16. Invert and negate to move it away from zero.
            crate::trace_dispatch!("computable", "ln", "small-inverse-rewrite");
            return self.inverse().ln().negate();
        }
        if rough_appr >= *high_ln_limit {
            // Sixteenths, ie 64 == 4.0
            let sixty_four = signed::SIXTY_FOUR.deref();

            if rough_appr <= *sixty_four {
                // Moderate large values use repeated sqrt: ln(x) = 4 ln(sqrt(sqrt(x))).
                // That is cheaper than running ln1p far from one. This is a
                // local low-overhead form of logarithm argument reduction.
                let quarter = self.sqrt().sqrt().ln();
                crate::trace_dispatch!("computable", "ln", "sqrt-range-reduction");
                return quarter.shift_left(2);
            } else {
                // Very large values are scaled by powers of two before ln1p, then the
                // binary exponent is added back as k ln(2). This keeps the
                // final ln1p kernel in its documented convergence interval.
                let mut extra_bits: i32 = (rough_appr.bits() - 5).try_into().expect(
                    "Approximation should have few enough bits to fit in a 32-bit signed integer",
                );

                let mut scaled = self.clone().shift_right(extra_bits);
                let mut scaled_rough = scaled.approx(low_prec);
                // The final branch below computes ln(1+x), and requires |x| < 1/2.
                // A bit-length estimate can leave scaled values in [1.5, 2), so
                // verify the low-precision scaled value before recursing.
                while scaled_rough >= *high_ln_limit {
                    extra_bits = extra_bits.checked_add(1).expect(
                        "Approximation should have few enough bits to fit in a 32-bit signed integer",
                    );
                    scaled = self.clone().shift_right(extra_bits);
                    scaled_rough = scaled.approx(low_prec);
                }

                let scaled_result = scaled.ln();
                let extra: BigInt = extra_bits.into();
                crate::trace_dispatch!("computable", "ln", "binary-scale-reduction");
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }

        let minus_one = Self::integer(signed::MINUS_ONE.clone());
        let fraction = Self::add(self, minus_one);
        // Final path is ln(1+x), where the prior reductions keep |x| small enough for the
        // prescaled series.
        crate::trace_dispatch!("computable", "ln", "prescaled-ln1p-kernel");
        Self::prescaled_ln(fraction)
    }

    /// Retain ln(1+self) lazily, assuming the residual is a defined real greater than -1.
    /// The name does not require |self| < 1/2: the demand-time kernel certifies its
    /// series range and reduces larger valid residuals. Do not approximate or validate
    /// the domain here. The wrapper starts with invalid magnitude/sign facts and no
    /// outer signal; the retained child keeps its own handle.
    fn prescaled_ln(self) -> Self {
        // Lazy ln(1+x) node. Normal ln construction reduces x first, but ln_1p
        // may pass any in-domain residual; the kernel checks its own range.
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledLn(self),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain ln(1+rational) without allocating a separate rational child node.
    /// Require rational > -1, which makes the result's sign equal to the residual's
    /// sign even outside the small-series range. Store that exact sign but no magnitude
    /// fact or outer signal. The approximation kernel checks its own convergence
    /// range; this constructor does not validate the logarithm's domain.
    fn prescaled_ln_rational(rational: Rational) -> Self {
        // Exact-rational ln1p reductions can skip a Ratio child and feed the
        // residual directly to the approximation kernel. For the admitted
        // domain x > -1, ln(1+x) has the sign of x, even outside the series range.
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledLnRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Retain ln(1+residual)+shift*ln(2) in one exact rational/shift node.
    /// Require residual > -1. The mathematical shift is a coefficient of ln(2), so
    /// representing it does not require materializing the integer 2^shift. Native
    /// reduction supplies a small residual, while evaluation can also reduce larger
    /// valid imported residuals. Start without sign/magnitude facts or an outer
    /// signal; neither the domain nor numerical precision is validated here.
    fn binary_scaled_ln_rational(residual: Rational, shift: i32) -> Self {
        // Private constructor for ln(2^shift * (1+residual)) after exact
        // rational range reduction. This keeps the exact shift/residual facts
        // together for hot ln(1+x^2) offenders and lets the approximation
        // kernel combine the two terms without allocating a short-lived graph.
        Self {
            internal: Arc::new(Node::new(
                Approximation::BinaryScaledLnRational { residual, shift },
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain ln(1+self) without eagerly adding one or evaluating the residual.
    /// Callers must establish self > -1; [crate::Real::ln_1p] owns public domain
    /// checking. Inverse-hyperbolic constructions and the checked API may supply
    /// large valid residuals, so the kernel chooses reduction only when digits are
    /// requested. This wrapper adds no sign certificate or outer abort signal.
    pub(crate) fn ln_1p(self) -> Self {
        // Preserve a small residual without eagerly forming 1+x. Real's public
        // ln_1p and inverse-hyperbolic identities also pass larger residuals,
        // which the approximation kernel reduces only when digits are needed.
        self.prescaled_ln()
    }
}
