impl Computable {
    /// Negate this number.
    pub fn negate(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            // Keep exact leaves exact; a Negate node would hide cheap rational
            // sign/MSD facts.
            return Self::rational(rational.neg());
        }
        if let Approximation::Negate(child) = &self.internal.approximation {
            // Double negation cancels at construction time so exact sign walks
            // and approximation stacks stay shallow.
            return child.clone();
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return right.clone().multiply_rational(scale.neg());
            }
            if let Some(scale) = right.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return left.clone().multiply_rational(scale.neg());
            }
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Preserve known exact signs for the cheap sign-first path in
            // predicates; this avoids a recursive sign walk on first query.
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(negate_sign(sign)),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Negate(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Multiplicative inverse of this number.
    pub fn inverse(self) -> Computable {
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            crate::trace_dispatch!("computable", "inverse", "shared-pi");
            return Self::pi_inverse_constant();
        }
        if self.shared_constant_kind() == Some(SharedConstant::InvPi) {
            crate::trace_dispatch!("computable", "inverse", "shared-inv-pi");
            return Self::pi();
        }
        if let Some(rational) = self.exact_rational()
            && let Ok(inverse) = rational.inverse()
        {
            // Exact rational reciprocals stay exact; this is common in BLAS
            // division by scalar constants.
            return Self::rational(inverse);
        }
        if let Approximation::Negate(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(-x) = -(1/x). The nonzero sign guard avoids manufacturing a
            // reciprocal of a value that may be zero.
            return child.clone().inverse().negate();
        }
        if let Approximation::Offset(child, n) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(x*2^n) = (1/x)*2^-n, preserving the cheap binary scale.
            return child.clone().inverse().shift_left(-n);
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && right.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                // 1/(q*x) = (1/q)/x. Peeling the exact scale lets chains like
                // negate(inverse(x * -7/8)) collapse every other step instead
                // of building deep multiply/inverse/negate stacks.
                return right.clone().inverse().multiply_rational(inverse_scale);
            }
            if let Some(scale) = right.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && left.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                return left.clone().inverse().multiply_rational(inverse_scale);
            }
        }
        if let Approximation::Inverse(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // Inverse of inverse collapses only when the inner value is
            // structurally nonzero.
            return child.clone();
        }
        if let Approximation::Square(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "inverse", "square-of-inverse");
            return child.clone().inverse().square();
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Reciprocal preserves sign for structurally nonzero values and lets
            // sign queries remain structural through inverse chains.
            ExactSignCache::Valid(sign) if sign != Sign::NoSign => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Inverse(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    pub(crate) fn shift_left(self, n: i32) -> Self {
        if n == 0 {
            return self;
        }
        if let Approximation::Offset(child, inner) = &self.internal.approximation {
            // Combine nested binary offsets rather than growing a chain of
            // no-op-ish wrappers.
            return child.clone().shift_left(inner + n);
        }
        // Exact sign is unchanged by binary scaling when the inner sign is
        // already proven; this makes compare/sign predicates avoid descending
        // into a one-step structural walk on hot paths.
        let exact_sign = match self.internal.facts.exact_sign() {
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Offset(self, n), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    fn shift_right(self, n: i32) -> Self {
        self.shift_left(-n)
    }

    /// Square of this number.
    pub fn square(self) -> Self {
        if let Some(rational) = self.exact_rational() {
            // Exact rationals can square without approximation or expression growth.
            return Self::rational(rational.clone() * rational);
        }
        if let Approximation::Negate(child) = &self.internal.approximation {
            // (-x)^2 is x^2; dropping the negate avoids an extra node in repeated products.
            return child.clone().square();
        }
        if let Approximation::Sqrt(child) = &self.internal.approximation {
            match child.exact_sign() {
                // sqrt(x)^2 can collapse only when x is structurally known nonnegative.
                Some(Sign::Plus) | Some(Sign::NoSign) => return child.clone(),
                _ => {}
            }
        }
        if let Approximation::Offset(child, n) = &self.internal.approximation {
            // (x * 2^n)^2 is x^2 * 2^(2n); keeping powers of two as offsets is much
            // cheaper than multiplying by an exact rational scale.
            return child.clone().square().shift_left(n * 2);
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational() {
                // Peel exact scales out of products before squaring symbolic factors.
                return right
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
            if let Some(scale) = right.exact_rational() {
                return left
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Squared values are nonnegative when defined; structural zero is
            // preserved as exact zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Square(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Multiply this number by some other number.
    pub fn multiply(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
            || matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
        {
            // Zero annihilates without preserving the other expression tree.
            return Self::zero();
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_one()) {
            // Multiplication by +/-1 stays as identity/negate so downstream exact-sign
            // queries still see the original structure.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_one()) {
            return self;
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return other.negate();
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return self.negate();
        }
        let exact_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match self.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match other.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(_)) | (Some(_), Some(Sign::NoSign)) => {
                    ExactSignCache::Valid(Sign::NoSign)
                }
                (Some(left), Some(right)) => ExactSignCache::Valid(if left == right {
                    Sign::Plus
                } else {
                    Sign::Minus
                }),
                _ => ExactSignCache::Invalid,
            }
        };
        if let Some((shift, sign)) = left_exact.as_ref().and_then(Rational::power_of_two_shift) {
            // Dyadic scales are represented as binary offsets, avoiding generic multiply
            // evaluation during approximation.
            let shifted = other.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Some((shift, sign)) = right_exact.as_ref().and_then(Rational::power_of_two_shift) {
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Collapse purely exact products immediately.
            return Self::rational(left.clone() * right.clone());
        }
        if let Some(scale) = left_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &other.internal.approximation
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                // Combine adjacent exact scales so factored symbolic products stay shallow.
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        if let Some(scale) = right_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &self.internal.approximation
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Multiply(self, other), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    pub(crate) fn multiply_rational(self, scale: Rational) -> Computable {
        if scale.sign() == Sign::NoSign {
            // Multiplying by zero drops the expression tree, including any
            // pending expensive approximation work.
            return Self::zero();
        }
        if let Some(value) = self.exact_rational() {
            // Exact symbolic leaves and exact-rational factors collapse directly.
            // This preserves cheap structural facts for chained scaling in
            // `fold_ref` and avoids building a new Multiply node.
            return Self::rational(value * scale);
        }
        if scale.is_one() {
            return self;
        }
        if scale.is_minus_one() {
            return self.negate();
        }
        if let Some((shift, sign)) = scale.power_of_two_shift() {
            // The borrowed Real fold path calls this often; recognize dyadic
            // scales before building a generic Multiply node.
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            // Peel and combine exact rational factors from existing multiply
            // nodes so repeated scalar rebalances stay shallow.
            if let Some(inner_scale) = left.exact_rational() {
                return right.clone().multiply_rational(inner_scale.clone() * scale);
            }
            if let Some(inner_scale) = right.exact_rational() {
                return left.clone().multiply_rational(inner_scale.clone() * scale);
            }
        }
        let scale_sign = scale.sign();
        let exact_sign = match (self.internal.facts.exact_sign(), scale_sign) {
            (ExactSignCache::Valid(Sign::NoSign), _) => ExactSignCache::Valid(Sign::NoSign),
            (ExactSignCache::Valid(Sign::Plus), Sign::Plus) => ExactSignCache::Valid(Sign::Plus),
            (ExactSignCache::Valid(Sign::Plus), Sign::Minus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Plus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Minus) => ExactSignCache::Valid(Sign::Plus),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Multiply(Self::rational(scale), self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Add some other number to this number.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Exact zero leaves are common after symbolic cancellation; avoid
            // wrapping the surviving operand in an Add node.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Symmetric exact-zero fast path for borrowed and owned additions.
            return self;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_one())
            && let Some(left) = left_exact.as_ref()
        {
            return Self::rational(left.add_one());
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_one())
            && let Some(right) = right_exact.as_ref()
        {
            return Self::rational(right.add_one());
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Fold exact leaf sums immediately so rational imports and parsed
            // dyadics stay outside the approximation graph.
            return Self::rational(left.clone() + right.clone());
        }
        let certified_bound = if let Some(rational) = right_exact.as_ref()
            && let Some(term) = self.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else if let Some(rational) = left_exact.as_ref()
            && let Some(term) = other.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else {
            BoundInfo::Unknown
        };
        // Store any c*K+q certificate directly on the Add node. The arithmetic
        // still falls back to a generic sum, but structural sign/fact queries
        // can answer from the certificate.
        let child_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match self.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match other.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(sign)) | (Some(sign), Some(Sign::NoSign)) => Some(sign),
                (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                _ => None,
            }
        };
        let certified_sign = certified_bound.known_sign().or(child_sign);
        Self {
            internal: Arc::new(Node::new(Approximation::Add(self, other), if certified_bound == BoundInfo::Unknown {
                    BoundCache::Invalid
                } else {
                    BoundCache::Valid(certified_bound)
                },
                match certified_sign {
                    Some(sign) => ExactSignCache::Valid(sign),
                    None => ExactSignCache::Invalid,
                },
            )),
            signal: None,
        }
    }

    pub(crate) fn integer(n: BigInt) -> Self {
        if n == *signed::ONE {
            return Self::one();
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Int(n), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    /// Error function, erf(x).
    pub fn erf(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::zero();
        }
        let series = Self {
            internal: Arc::new(Node::new(Approximation::ErfSeries(self.clone()), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        };
        let gaussian = self.square().negate().exp();
        let two_over_sqrt_pi = Self::pi().sqrt().inverse().shift_left(1);
        two_over_sqrt_pi.multiply(gaussian).multiply(series)
    }

    /// Complementary error function, erfc(x) = 1 - erf(x).
    pub fn erfc(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::one();
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Erfc(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Scaled complementary error function, erfcx(x) = exp(x^2) * erfc(x).
    pub fn erfcx(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::one();
        }
        self.clone().square().exp().multiply(self.erfc())
    }

    /// Standard normal CDF.
    pub fn pnorm(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::rational(HALF_RATIONAL.clone());
        }
        let sqrt2 = Self::sqrt_constant(2).unwrap_or_else(|| Self::integer(BigInt::from(2)).sqrt());
        let z = self.multiply(sqrt2.inverse());
        Self::one().add(z.erf()).shift_right(1)
    }

    /// Standard normal density.
    pub fn dnorm(self) -> Computable {
        let neg_half_x_sq = self.square().shift_right(1).negate();
        let sqrt_2pi = Self::pi().shift_left(1).sqrt();
        neg_half_x_sq.exp().multiply(sqrt_2pi.inverse())
    }

    /// Standard normal upper-tail probability, 1 - pnorm(x).
    pub fn normal_sf(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::rational(HALF_RATIONAL.clone());
        }
        Self {
            internal: Arc::new(Node::new(Approximation::NormalSf(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Standard normal probability mass over [lo, hi].
    pub fn normal_interval(lo: Computable, hi: Computable) -> Computable {
        if Computable::internal_structural_eq(&lo, &hi) {
            return Self::zero();
        }
        Self {
            internal: Arc::new(Node::new(Approximation::NormalInterval { lo, hi }, BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal CDF.
    pub fn log_pnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogPnorm(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal upper-tail probability.
    pub fn log_normal_sf(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogNormalSf(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal density.
    pub fn log_dnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogDnorm(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Standard normal quantile by Newton iteration with the analytic density.
    pub fn normal_quantile(p: Computable, seed: BigInt, seed_prec: Precision) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::NormalQuantile(Box::new(NormalQuantileData {
                    p,
                    seed,
                    seed_prec,
                })), BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Attach an abort signal checked by long-running approximation routines.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s);
    }

}
