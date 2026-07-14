impl Computable {
    pub(crate) fn sqrt_rational(r: Rational) -> Self {
        // Preserve the rational leaf so sqrt can still collapse perfect
        // rational squares before allocating a generic Sqrt node.
        let rational = Self::rational(r);
        Self::sqrt(rational)
    }

    pub(crate) fn sqrt_squarefree_rational(radicand: Rational) -> Self {
        debug_assert!(radicand.sign() != Sign::Minus);
        let child = Self::rational(radicand);
        let exact_sign = match child.internal.facts.exact_sign() {
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            _ => ExactSignCache::Valid(Sign::Plus),
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Sqrt(child), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Square root of this number.
    pub fn sqrt(self) -> Computable {
        if let Approximation::Square(child) = &self.internal.approximation {
            // sqrt(x^2) can collapse to abs(x) when the sign is structurally known.
            match child.exact_sign() {
                Some(Sign::Plus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-positive-collapse");
                    return child.clone();
                }
                Some(Sign::Minus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-negative-abs-collapse");
                    return child.clone().negate();
                }
                Some(Sign::NoSign) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-zero-collapse");
                    return Self::zero();
                }
                None => {}
            }
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            let reduced = |scale: Rational, square_side: &Computable| {
                // Recognize c*x^2 where c is an exact square, preserving the symbolic x
                // instead of introducing a generic sqrt node.
                let (root, rest) = scale.extract_square_reduced();
                if !rest.is_one() {
                    return None;
                }
                let Approximation::Square(child) = &square_side.internal.approximation else {
                    return None;
                };
                match child.exact_sign() {
                    Some(Sign::Plus) => Some(child.clone().multiply(Self::rational(root))),
                    Some(Sign::Minus) => {
                        Some(child.clone().negate().multiply(Self::rational(root)))
                    }
                    Some(Sign::NoSign) => Some(Self::zero()),
                    None => None,
                }
            };

            if let Some(scale) = left.exact_rational()
                && let Some(value) = reduced(scale, right)
            {
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
            if let Some(scale) = right.exact_rational()
                && let Some(value) = reduced(scale, left)
            {
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
        }
        if let Some(rational) = self.exact_rational()
            && rational.sign() != Sign::Minus
            && rational.extract_square_will_succeed()
        {
            // Perfect rational squares stay exact. For scaled sqrt(2)/sqrt(3)
            // residuals, keep the irrational part shared and the exact scale
            // symbolic. Plain sqrt(2) and sqrt(3) deliberately stay on the
            // generic node because repeated cached approximation of a single
            // node is faster than a thread-local shared-cache lookup.
            let (root, rest) = rational.extract_square_reduced();
            if rest.is_one() {
                crate::trace_dispatch!("computable", "sqrt", "exact-rational-square");
                return Self::rational(root);
            }
            if !root.is_one()
                && let Some(shared_radicand @ (2 | 3)) = rest.to_integer_i64()
            {
                // For scaled sqrt(2)/sqrt(3), reuse the shared constant cache
                // for the irrational factor and keep the exact rational scale
                // separate. This is a measured construction-time win for scaled
                // square-free inputs without changing the single-node path for
                // plain sqrt(2)/sqrt(3).
                crate::trace_dispatch!("computable", "sqrt", "shared-squarefree-rational");
                let constant = Self::sqrt_constant(shared_radicand)
                    .expect("sqrt(2) and sqrt(3) are shared constants");
                return constant.multiply(Self::rational(root));
            }
        }
        crate::trace_dispatch!("computable", "sqrt", "generic-sqrt-node");
        let exact_sign = match self.internal.facts.exact_sign() {
            // Square roots are nonnegative where defined and preserve structural zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Sqrt(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    pub(crate) fn prescaled_atan(n: BigInt) -> Self {
        // atan(1/n) kernel used by pi and atan reduction constants. Passing the
        // denominator as an integer keeps the series loop division-only.
        Self {
            internal: Arc::new(Node::new(Approximation::IntegralAtan(n), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn atan_rational_deferred(rational: Rational) -> Self {
        // Exact rational atan reductions used to allocate intermediate
        // add/multiply/inverse nodes before reaching the small atan series. This
        // deferred node keeps the public constructor compact and performs the
        // same range reductions directly in the approximation kernel.
        crate::trace_dispatch!("computable", "constructor", "atan-rational-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::AtanRational(rational), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn asin_rational_deferred(rational: Rational) -> Self {
        // Exact rational asin uses a direct series for tiny/moderate inputs.
        // Storing the Rational in the approximation node keeps the hot path out
        // of the generic sqrt/atan transform and avoids a child approx lookup.
        crate::trace_dispatch!("computable", "constructor", "asin-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AsinRational(rational), BoundCache::Invalid, ExactSignCache::Valid(sign))),
            signal: None,
        }
    }

    /// Arctangent of this number.
    pub fn atan(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "atan", "exact-zero");
                return Self::zero();
            }
            if rational.sign() == Sign::Plus {
                crate::trace_dispatch!("computable", "atan", "exact-rational-deferred");
                return Self::atan_rational_deferred(rational);
            }
        }
        let (known_sign, planning_msd) = self.planning_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry");
            return self.negate().atan().negate();
        }
        if known_sign.is_none() && self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry-fallback");
            return self.negate().atan().negate();
        }
        if let Some(msd) = planning_msd.flatten() {
            if msd < -1 {
                crate::trace_dispatch!("computable", "atan", "structural-small-prescaled");
                return Self {
                    internal: Arc::new(Node::new(Approximation::PrescaledAtan(self), BoundCache::Invalid, ExactSignCache::Invalid)),
                    signal: None,
                };
            }
            if msd >= 5 {
                crate::trace_dispatch!("computable", "atan", "large-reciprocal-structural");
                return Self::pi()
                    .shift_right(1)
                    .add(self.inverse().atan().negate());
            }
        }

        let rough_appr = self.approx(-4);
        if rough_appr <= *signed::EIGHT {
            // Small atan arguments use the prescaled series directly.
            crate::trace_dispatch!("computable", "atan", "rough-small-prescaled");
            return Self {
                internal: Arc::new(Node::new(Approximation::PrescaledAtan(self), BoundCache::Invalid, ExactSignCache::Invalid)),
                signal: None,
            };
        }

        let one = Self::one();
        let half = one.clone().shift_right(1);
        if rough_appr <= *signed::SIXTEEN {
            // For middle-sized arguments, subtract atan(1/2) before recursing. This keeps
            // the residual small without jumping all the way to the reciprocal identity.
            // This follows the range-reduce-before-series pattern in Brent,
            // https://doi.org/10.1145/321941.321944.
            let numerator = self.clone().add(half.clone().negate());
            let denominator = one.add(self.multiply(half));
            crate::trace_dispatch!("computable", "atan", "medium-atan-half-reduction");
            return Self::prescaled_atan(BigInt::from(2_u8))
                .add(numerator.multiply(denominator.inverse()).atan());
        }

        // Large positive atan uses pi/2 - atan(1/x), which converges faster.
        crate::trace_dispatch!("computable", "atan", "large-reciprocal");
        Self::pi()
            .shift_right(1)
            .add(self.inverse().atan().negate())
    }

    /// Two-argument arctangent of `(self, x)`, returning the angle of the
    /// point `(x, self)` measured counterclockwise from the positive `x`
    /// axis in the principal range `(-pi, pi]`.
    ///
    /// `self` is the `y` coordinate and `x` is the `x` coordinate, matching
    /// the IEEE 754 `atan2(y, x)` convention. The implementation reduces to
    /// the single-argument [`Computable::atan`] kernel after a quadrant
    /// correction:
    /// - `x > 0`: returns `atan(self / x)`.
    /// - `x < 0` and `self >= 0`: returns `atan(self / x) + pi`.
    /// - `x < 0` and `self < 0`: returns `atan(self / x) - pi`.
    /// - axes return exact constants: `pi/2`, `-pi/2`, `pi`, or zero.
    /// - the origin `(0, 0)` returns zero, matching `f64::atan2`.
    pub fn atan2(self, x: Computable) -> Computable {
        let y_sign = self.structural_facts().sign.map(private_sign);
        let x_sign = x.structural_facts().sign.map(private_sign);
        match (y_sign, x_sign) {
            (Some(Sign::NoSign), Some(Sign::NoSign)) | (Some(Sign::NoSign), Some(Sign::Plus)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-zero-y");
                return Self::zero();
            }
            (Some(Sign::NoSign), Some(Sign::Minus)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-negative-x");
                return Self::pi();
            }
            (Some(Sign::Plus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-positive-y");
                return Self::pi().shift_right(1);
            }
            (Some(Sign::Minus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-negative-y");
                return Self::pi().shift_right(1).negate();
            }
            _ => {}
        }
        match (
            y_sign.unwrap_or_else(|| self.sign()),
            x_sign.unwrap_or_else(|| x.sign()),
        ) {
            (Sign::NoSign, Sign::Plus) => {
                crate::trace_dispatch!("computable", "atan2", "quadrant-right");
                return self.multiply(x.inverse()).atan();
            }
            (Sign::NoSign, Sign::NoSign) => {
                crate::trace_dispatch!("computable", "atan2", "unresolved-origin");
                return Self::zero();
            }
            (Sign::NoSign, Sign::Minus) => {
                let y_sign = self
                    .sign_until(ATAN2_SIGN_REFINEMENT_FLOOR)
                    .map(private_sign);
                return match y_sign {
                    Some(Sign::Minus) => {
                        crate::trace_dispatch!("computable", "atan2", "quadrant-lower-left");
                        self.multiply(x.inverse()).atan().add(Self::pi().negate())
                    }
                    Some(Sign::Plus) => {
                        crate::trace_dispatch!("computable", "atan2", "quadrant-upper-left");
                        self.multiply(x.inverse()).atan().add(Self::pi())
                    }
                    _ => {
                        crate::trace_dispatch!("computable", "atan2", "axis-negative-x");
                        Self::pi()
                    }
                };
            }
            (Sign::Plus, Sign::NoSign) => {
                crate::trace_dispatch!("computable", "atan2", "half-angle-positive-y");
                return Self::atan2_half_angle(self, x);
            }
            (Sign::Minus, Sign::NoSign) => {
                crate::trace_dispatch!("computable", "atan2", "half-angle-negative-y");
                return Self::atan2_half_angle(self, x);
            }
            _ => {}
        }
        let x_sign = x_sign.unwrap_or_else(|| x.sign());
        let y_sign = y_sign.unwrap_or_else(|| self.sign());
        let base = self.multiply(x.inverse()).atan();
        if x_sign == Sign::Plus {
            crate::trace_dispatch!("computable", "atan2", "quadrant-right");
            base
        } else if y_sign == Sign::Plus {
            crate::trace_dispatch!("computable", "atan2", "quadrant-upper-left");
            base.add(Self::pi())
        } else {
            crate::trace_dispatch!("computable", "atan2", "quadrant-lower-left");
            base.add(Self::pi().negate())
        }
    }

    fn atan2_half_angle(y: Computable, x: Computable) -> Computable {
        let radius = x.clone().square().add(y.clone().square()).sqrt();
        y.multiply(radius.add(x).inverse()).atan().shift_left(1)
    }

    /// Inverse sine of this number.
    pub fn asin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "asin", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "asin", "exact-negative-symmetry");
                    return self.negate().asin().negate();
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny asin(x) is handled by its dedicated series; the generic
                        // atan transform builds extra sqrt/division nodes.
                        crate::trace_dispatch!("computable", "asin", "exact-tiny-rational-series");
                        return Self::asin_rational_deferred(rational);
                    }
                    if rational >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                        // Near 1, use pi/2 - acos(x); acos has the endpoint transform.
                        crate::trace_dispatch!("computable", "asin", "endpoint-via-acos");
                        return Self::pi().shift_right(1).add(self.acos().negate());
                    }
                    crate::trace_dispatch!("computable", "asin", "positive-rational-via-acos");
                    return Self::pi().shift_right(1).add(self.acos().negate());
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "asin", "known-negative-symmetry");
            return self.negate().asin().negate();
        }
        let (_, planned_msd) = self.planning_sign_and_msd();
        if planned_msd.flatten().is_some_and(|msd| msd <= -4) {
            crate::trace_dispatch!("computable", "asin", "structural-tiny-prescaled");
            return Self::prescaled_asin(self);
        }

        crate::trace_dispatch!("computable", "asin", "generic-atan-sqrt-transform");
        Self::asin_deferred(self)
    }

    /// Inverse cosine of this number.
    pub fn acos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.is_one() {
                crate::trace_dispatch!("computable", "acos", "exact-one-zero");
                return Self::zero();
            }
            if rational.is_minus_one() {
                crate::trace_dispatch!("computable", "acos", "exact-minus-one-pi");
                return Self::pi();
            }
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "acos", "exact-zero-half-pi");
                return Self::pi().shift_right(1);
            }
            let rational_sign = rational.sign();
            let magnitude = if rational_sign == Sign::Minus {
                rational.neg()
            } else {
                rational
            };
            if magnitude.msd_exact().is_some_and(|msd| msd <= -4) {
                crate::trace_dispatch!("computable", "acos", "tiny-via-asin");
                return Self::pi().shift_right(1).add(self.asin().negate());
            }
            if rational_sign == Sign::Minus && magnitude >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                // Negative endpoint values mirror the positive endpoint transform.
                // Store the magnitude directly so construction stays as a single
                // deferred fact instead of rebuilding pi - acos(|x|).
                crate::trace_dispatch!("computable", "acos", "negative-rational-deferred");
                return Self::acos_negative_rational_deferred(magnitude);
            }
            if rational_sign == Sign::Plus {
                crate::trace_dispatch!("computable", "acos", "positive-rational-deferred");
                return Self::acos_positive_rational_deferred(magnitude);
            }
        }

        if self.exact_sign() == Some(Sign::Plus) {
            // For positive values, acos(x) = 2 atan(sqrt((1-x)/(1+x))). This is the
            // endpoint-friendly path for values near 1.
            crate::trace_dispatch!("computable", "acos", "positive-endpoint-deferred");
            return Self::acos_positive(self);
        }

        crate::trace_dispatch!("computable", "acos", "generic-half-pi-minus-asin");
        Self::pi().shift_right(1).add(self.asin().negate())
    }

    /// Inverse hyperbolic sine of this number.
    pub fn asinh(self) -> Computable {
        let exact_rational = self.exact_rational();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "asinh", "exact-zero");
            return Self::zero();
        }
        let (known_sign, planned_msd) = self.planning_sign_and_msd();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::Minus)
            || known_sign == Some(Sign::Minus)
        {
            crate::trace_dispatch!("computable", "asinh", "known-negative-symmetry");
            return self.negate().asinh().negate();
        }
        let exact_small = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd <= -1);
        let exact_large = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd >= 3);
        if exact_small {
            crate::trace_dispatch!("computable", "asinh", "exact-small-rational-series");
            if let Some(rational) = exact_rational {
                return Self::asinh_rational_deferred(rational);
            }
            return Self::prescaled_asinh(self);
        }
        if exact_large {
            let radicand = self.clone().square().add(Self::one());
            crate::trace_dispatch!("computable", "asinh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = planned_msd.flatten();
        let is_near_zero = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_zero {
            // Direct Computable approximation benches include construction in
            // the measured work, and the eager graph caches its children better
            // than a deferred Real-only wrapper.
            let square = self.clone().square();
            let one = Self::one();
            let denominator = square.clone().add(one.clone()).sqrt().add(one);
            crate::trace_dispatch!("computable", "asinh", "near-zero-ln1p-transform");
            return self.add(square.multiply(denominator.inverse())).ln_1p();
        }

        let radicand = self.clone().square().add(Self::one());
        crate::trace_dispatch!("computable", "asinh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic cosine of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn acosh(self) -> Computable {
        let exact_rational_msd = match &self.internal.approximation {
            Approximation::One => {
                crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                return Self::zero();
            }
            Approximation::Ratio(r) => {
                if r.is_one() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if r == &Rational::new(2) {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if r >= &Rational::new(2) {
                    let radicand = r.clone() * r.clone() - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-rational-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                r.msd_exact()
            }
            Approximation::Int(n) => {
                if n == signed::ONE.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if n == signed::TWO.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if n >= signed::TWO.deref() {
                    let r = Rational::from_bigint(n.clone());
                    let radicand = r.clone() * r - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-integer-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                if n.sign() == Sign::NoSign {
                    None
                } else {
                    Some(n.magnitude().bits() as Precision - 1)
                }
            }
            _ => None,
        };
        if let Approximation::Sqrt(child) = &self.internal.approximation
            && child
                .exact_rational()
                .is_some_and(|r| r == Rational::new(2))
        {
            crate::trace_dispatch!("computable", "acosh", "sqrt-two-asinh-one");
            return Self::asinh1_constant();
        }
        if exact_rational_msd.is_some_and(|msd| msd >= 3) {
            // Large exact rationals skip the low-precision near-one probe and
            // use the direct acosh identity.
            let one = Self::one();
            let radicand = self.clone().square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = self.planning_sign_and_msd().1.flatten();
        let is_near_one = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_one {
            // Keep the public Computable kernel eager for approximation-heavy
            // benches; Real uses a deferred wrapper when construction alone is
            // the hot path.
            let one = Self::one();
            let shifted = self.clone().add(one.clone().negate());
            let radicand = self.square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "near-one-ln1p-transform");
            return shifted.add(radicand.sqrt()).ln_1p();
        }

        // Generic identity for already validated large inputs.
        let one = Self::one();
        let radicand = self.clone().square().add(one.negate());
        crate::trace_dispatch!("computable", "acosh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic tangent of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn atanh(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "atanh", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "atanh", "exact-negative-symmetry");
                    return self.negate().atanh().negate();
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny atanh(x) is best served by the direct odd series.
                        crate::trace_dispatch!("computable", "atanh", "exact-tiny-prescaled");
                        return Self::atanh_rational_deferred(rational);
                    }
                    if rational.is_one_half() {
                        crate::trace_dispatch!("computable", "atanh", "exact-half-ln3");
                        return Self::ln_constant(3)
                            .expect("ln3 is a shared log constant")
                            .multiply(Self::half());
                    }
                    if !rational.is_one() {
                        // For exact rationals, atanh(x) is one exact ln ratio.
                        // That keeps common factors in the logarithm constructor
                        // instead of building a generic quotient Computable first.
                        // The final multiply uses the cached exact 1/2 leaf so
                        // construction does not allocate a new rational after
                        // the symbolic reduction has already succeeded.
                        let one = Rational::one();
                        let ratio = (one.clone() + rational.clone()) / (one - rational);
                        crate::trace_dispatch!("computable", "atanh", "exact-log-ratio");
                        return Self::ln_exact_rational(ratio).multiply(Self::half());
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atanh", "known-negative-symmetry");
            return self.negate().atanh().negate();
        }
        let (_, planned_msd) = self.planning_sign_and_msd();
        if planned_msd.flatten().is_some_and(|msd| msd <= -4) {
            crate::trace_dispatch!("computable", "atanh", "structural-tiny-prescaled");
            return Self::prescaled_atanh(self);
        }

        // General formula 1/2 * ln((1+x)/(1-x)). Tiny exact rationals avoid
        // this path because the odd atanh series has much less setup.
        crate::trace_dispatch!("computable", "atanh", "generic-log-ratio");
        let one = Self::one();
        let numerator = one.clone().add(self.clone());
        let denominator = one.add(self.negate());
        numerator
            .multiply(denominator.inverse())
            .ln()
            .multiply(Self::half())
    }

}
