struct InlineStack<T, const N: usize> {
    inline: [Option<T>; N],
    inline_len: usize,
    overflow: Vec<T>,
}

impl<T, const N: usize> InlineStack<T, N> {
    fn new() -> Self {
        Self {
            inline: std::array::from_fn(|_| None),
            inline_len: 0,
            overflow: Vec::new(),
        }
    }

    fn push(&mut self, value: T) {
        if self.overflow.is_empty() && self.inline_len < N {
            self.inline[self.inline_len] = Some(value);
            self.inline_len += 1;
        } else {
            self.overflow.push(value);
        }
    }

    fn pop(&mut self) -> Option<T> {
        if let Some(value) = self.overflow.pop() {
            return Some(value);
        }
        self.inline_len = self.inline_len.checked_sub(1)?;
        self.inline[self.inline_len].take()
    }
}

impl Computable {
    /// Maximum fractional bits admitted by approximation, including child
    /// requests and kernel guard bits. Larger demands return `Problem::Exhausted`.
    /// This precision resource limit does not bound input coefficient widths,
    /// graph traversal, total allocation, or elapsed time.
    pub const MAX_APPROXIMATION_BITS: Precision = 1_048_576;

    /// Certify |x| <= 1 (closed) or |x| < 1 (open) before a domain-dependent
    /// shortcut. An exact rational witness also resolves boundary expressions
    /// which numerical refinement alone cannot separate from +/-1.
    pub(crate) fn certify_unit_interval(
        &self,
        signal: &Option<Signal>,
        closed: bool,
    ) -> Result<Option<Rational>, Problem> {
        if evaluation_should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        if let Some(value) = self.bounded_rational_value() {
            return if value.numerator() < value.denominator()
                || (closed && value.numerator() == value.denominator())
            {
                Ok(Some(value))
            } else {
                Err(Problem::NotANumber)
            };
        }
        let mut precision: Precision = -8;
        loop {
            let sample = self.approx_signal(signal, precision)?;
            let limit = BigUint::one() << precision.unsigned_abs() as usize;
            let upper = sample.magnitude() + 1_u8;
            if upper < limit || (closed && upper == limit) {
                return Ok(None);
            }
            let outside = &limit + 1_u8;
            if sample.magnitude() > &outside || (!closed && sample.magnitude() == &outside) {
                return Err(Problem::NotANumber);
            }
            if precision <= -(Self::MAX_APPROXIMATION_BITS / 2) {
                return Err(Problem::Exhausted);
            }
            precision *= 2;
        }
    }

    /// Return an exact sign already retained at this expression's root.
    ///
    /// Unlike [`Self::structural_facts`], this query never walks the expression
    /// graph and never evaluates an approximation. It is intended for bounded
    /// accelerators whose inconclusive result has an authoritative fallback.
    pub(crate) fn immediate_sign(&self) -> Option<RealSign> {
        let (bound, sign) = self.internal.facts.snapshot();
        match sign {
            ExactSignCache::Valid(sign) => Some(public_sign(sign)),
            ExactSignCache::Invalid | ExactSignCache::Unknown => match bound {
                BoundCache::Valid(bound) => bound.known_sign().map(public_sign),
                BoundCache::Invalid => None,
            },
        }
    }

    /// An approximation of this Computable scaled to a specific precision.
    ///
    /// Since the value is scaled, the approximation is roughly `value * 2^(-p)`.
    /// Negative values of `p` request more precision.
    ///
    /// The approximation is scaled (thus, a larger value for more negative p)
    /// and should be accurate to within +/- 1 at the scale provided.
    ///
    /// Returns `Problem::Exhausted` if interrupted or if requested/intermediate
    /// fractional precision exceeds [`Self::MAX_APPROXIMATION_BITS`]. Unrepresentable
    /// precision arithmetic returns `Problem::OutOfRange`. Failed evaluations
    /// never publish a numeric approximation in this node's cache.
    ///
    /// Example: 0.875 is between 0 and 1 with zero bits of extra precision
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let n = Rational::fraction(7, 8).unwrap();
    /// let comp = Computable::rational(n);
    /// assert!((BigInt::zero() ..= BigInt::one()).contains(&comp.approx(0).unwrap()));
    /// ```
    ///
    /// Example: π * 2³ is a bit more than 25 but less than 26
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let pi = Computable::pi();
    /// let between_25_26 = (ToBigInt::to_bigint(&25).unwrap() ..= ToBigInt::to_bigint(&26).unwrap());
    /// assert!(between_25_26.contains(&pi.approx(-3).unwrap()));
    /// ```
    pub fn approx(&self, p: Precision) -> Result<BigInt, Problem> {
        self.approx_signal(&self.signal, p)
    }

    /// Return either the floor or the ceiling of this value.
    ///
    /// Unlike choosing one particular direction, this multivalued rounding
    /// operation does not need to decide whether the value is an integer. It
    /// makes one approximation request and propagates any evaluation failure.
    /// The particular adjacent integer is deliberately
    /// unspecified; use a certified directional rounding operation when that
    /// distinction matters.
    pub fn near_integer(&self) -> Result<BigInt, Problem> {
        // `approx(-2)` differs from 4*x by at most one. Rounding it after a
        // division by four adds at most another half, so the returned integer
        // differs from x by at most 3/4 and must be floor(x) or ceil(x).
        Ok(scale(self.approx(-2)?, -2))
    }

    /// Like [`Self::approx`], using the specified atomic stop signal.
    ///
    /// Once evaluation observes cancellation, it returns [`Problem::Exhausted`]
    /// even if another thread clears the flag. A failed request never publishes
    /// its partial result in the approximation cache. Completed child results
    /// may remain cached and can be reused by a later request.
    pub fn approx_signal(&self, signal: &Option<Signal>, p: Precision) -> Result<BigInt, Problem> {
        enum Frame<'a> {
            Eval(&'a Computable, Precision),
            FinishNegate(&'a Computable, Precision),
            FinishAdd(&'a Computable, Precision),
            FinishOffset(&'a Computable, Precision),
        }

        if should_stop(signal) || evaluation_exhausted() {
            return Err(Problem::Exhausted);
        }
        // Exact zero requires no precision-dependent allocation.
        if matches!(&self.internal.approximation, Approximation::Int(n) if n.is_zero()) {
            return Ok(BigInt::zero());
        }
        let p = checked_approximation_precision(p)?;

        // Exact integer leaves are cheaper to shift directly than to traverse
        // any synchronized cache, and the direct result uses no more limbs
        // than the requested precision.
        let uses_node_cache = match &self.internal.approximation {
            Approximation::Int(value) => {
                if !charge_evaluation_work() {
                    return Err(Problem::Exhausted);
                }
                // A gap beyond the integer's bit length rounds to zero for
                // either sign. Resolve the scaling helper's same shortcut
                // here, after the precision checks and work charge.
                if p > 0 && u64::from(p as u32) > value.bits() {
                    return Ok(BigInt::zero());
                }
                return Ok(scale_at_precision_ref(value, p));
            }
            Approximation::One => {
                if !charge_evaluation_work() {
                    return Err(Problem::Exhausted);
                }
                return Ok(scale_at_precision_ref(signed::ONE.deref(), p));
            }
            Approximation::Constant(constant) => {
                if let Some(cached) = Self::cached_constant_at_precision(*constant, p) {
                    return Ok(cached);
                }
                false
            }
            _ => true,
        };

        if uses_node_cache && !self.internal.facts.finite_real() {
            // A coarse numeric enclosure or an algebraic cancellation cannot
            // prove that discarded descendants exist. Validate the immutable
            // graph before any cache or kernel can publish a numeric value.
            self.ensure_finite_real(signal)?;
        }

        if uses_node_cache && let Some(cached) = self.internal.cached_at_precision(p) {
            return Ok(cached);
        }
        if let Some(integer) = self.exact_trig_integer() {
            let integer = BigInt::from(integer?);
            self.internal
                .facts
                .publish_exact_sign(ExactSignCache::Valid(integer.sign()));
            let result = scale_at_precision(integer, p);
            self.store_cache_value(signal, p, result.clone())?;
            return Ok(result);
        }
        if uses_node_cache
            && let Some(result) = self
                .cached_quadratic_tower_approximation(signal, p)
                .transpose()?
        {
            self.store_cache_value(signal, p, result.clone())?;
            return Ok(result);
        }

        if !matches!(
            &self.internal.approximation,
            Approximation::Negate(_) | Approximation::Add(_, _) | Approximation::Offset(_, _)
        ) {
            // Most node kinds evaluate as one kernel call. Only Negate/Add/Offset
            // are flattened below because they form the long chains seen in
            // parser, matrix, and structural-reduction workloads.
            let result = self.internal.approximate(signal, p)?;
            self.store_cache_value(signal, p, result.clone())?;
            return Ok(result);
        }

        // Common expression chains fit inline; unusually deep trees spill to
        // heap storage without changing the iterative traversal.
        let mut frames = InlineStack::<Frame<'_>, 16>::new();
        let mut values = InlineStack::<BigInt, 8>::new();
        frames.push(Frame::Eval(self, p));

        while let Some(frame) = frames.pop() {
            if evaluation_should_stop(signal) {
                return Err(Problem::Exhausted);
            }
            match frame {
                Frame::Eval(node, prec) => {
                    if matches!(&node.internal.approximation, Approximation::Int(n) if n.is_zero())
                    {
                        values.push(BigInt::zero());
                        continue;
                    }
                    let prec = checked_approximation_precision(prec)?;
                    if let Some(cached) = node.cached_at_precision(prec) {
                        values.push(cached);
                        continue;
                    }

                    if let Some(result) = node
                        .cached_quadratic_tower_approximation(signal, prec)
                        .transpose()?
                    {
                        node.store_cache_value(signal, prec, result.clone())?;
                        values.push(result);
                        continue;
                    }

                    match &node.internal.approximation {
                        Approximation::Negate(child) => {
                            // Flatten sign wrappers so a deep chain of negated
                            // sums does not recurse through approx_signal.
                            frames.push(Frame::FinishNegate(node, prec));
                            frames.push(Frame::Eval(child, prec));
                        }
                        Approximation::Add(left, right) => {
                            // Evaluate add children at two guard bits, then
                            // round once. This mirrors the recursive add kernel
                            // but avoids stack growth for chained additions.
                            frames.push(Frame::FinishAdd(node, prec));
                            // Evaluate larger predicted precision demands
                            // first, allowing siblings to reuse finer caches.
                            // Binary scaling participates in the hint: depth
                            // alone reverses the right order for decaying terms.
                            // Both operands retain exactly the same precision;
                            // this hint cannot alter the enclosure contract.
                            let (first, second) = if right.internal.facts.linear_demand()
                                > left.internal.facts.linear_demand()
                            {
                                (right, left)
                            } else {
                                (left, right)
                            };
                            let child_precision = prec.checked_sub(2).ok_or(Problem::OutOfRange)?;
                            frames.push(Frame::Eval(second, child_precision));
                            frames.push(Frame::Eval(first, child_precision));
                        }
                        Approximation::Offset(child, n) => {
                            // Binary offsets translate the requested precision
                            // instead of doing any arithmetic at finish time.
                            frames.push(Frame::FinishOffset(node, prec));
                            frames.push(Frame::Eval(
                                child,
                                prec.checked_sub(*n).ok_or(Problem::OutOfRange)?,
                            ));
                        }
                        _ => {
                            let result = node.internal.approximate(signal, prec)?;
                            node.store_cache_value(signal, prec, result.clone())?;
                            values.push(result);
                        }
                    }
                }
                Frame::FinishNegate(node, prec) => {
                    let result = -values.pop().expect("negate child result should exist");
                    node.store_cache_value(signal, prec, result.clone())?;
                    values.push(result);
                }
                Frame::FinishAdd(node, prec) => {
                    let right = values.pop().expect("add rhs result should exist");
                    let left = values.pop().expect("add lhs result should exist");
                    let result = scale(left + right, -2);
                    node.store_cache_value(signal, prec, result.clone())?;
                    values.push(result);
                }
                Frame::FinishOffset(node, prec) => {
                    let result = values.pop().expect("offset child result should exist");
                    node.store_cache_value(signal, prec, result.clone())?;
                    values.push(result);
                }
            }
        }

        Ok(values.pop().expect("evaluation should produce a result"))
    }

    /// Conservatively inspect cached and structural numeric facts.
    pub fn structural_facts(&self) -> RealStructuralFacts {
        let exact = self.exact_rational();
        let (cached_bound, cached_sign) = self.internal.facts.snapshot();

        let mut sign = match cached_sign {
            ExactSignCache::Valid(sign) => Some(public_sign(sign)),
            ExactSignCache::Invalid | ExactSignCache::Unknown => self.exact_sign().map(public_sign),
        };
        #[cfg(feature = "dispatch-trace")]
        if sign.is_some() {
            crate::trace_dispatch!("computable", "structural_facts", "exact-sign-cache");
        }
        if sign.is_none()
            && let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            crate::trace_dispatch!("computable", "structural_facts", "approximation-cache-sign");
            sign = Some(public_sign(appr.sign()));
        }

        let bound = match cached_bound {
            BoundCache::Valid(bound) => bound,
            BoundCache::Invalid => self.cheap_bound(),
        };
        if sign.is_none() {
            let bound_sign = bound.known_sign();
            #[cfg(feature = "dispatch-trace")]
            if bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "cheap-bound-sign");
            }
            sign = bound_sign.map(public_sign);
        }
        if sign.is_none() {
            let exact_bound_sign = exact
                .as_ref()
                .map(BoundInfo::from_rational)
                .as_ref()
                .and_then(BoundInfo::known_sign);
            #[cfg(feature = "dispatch-trace")]
            if exact_bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "exact-rational-bound");
            }
            sign = exact_bound_sign.map(public_sign);
        }
        let exact_bound = if sign.is_none() {
            // Keep exact-rational bounds deferred until sign could not be proven by
            // cheaper structural facts. This avoids unnecessary conversion work.
            exact.as_ref().map(BoundInfo::from_rational)
        } else {
            None
        };

        let zero = match sign {
            Some(RealSign::Zero) => ZeroKnowledge::Zero,
            Some(RealSign::Negative | RealSign::Positive) => ZeroKnowledge::NonZero,
            None => {
                if matches!(&bound, BoundInfo::Zero)
                    || matches!(&exact_bound, Some(BoundInfo::Zero))
                {
                    ZeroKnowledge::Zero
                } else if matches!(&bound, BoundInfo::NonZero { .. })
                    || matches!(&exact_bound, Some(BoundInfo::NonZero { .. }))
                {
                    ZeroKnowledge::NonZero
                } else {
                    ZeroKnowledge::Unknown
                }
            }
        };

        let magnitude = bound
            .magnitude_bits()
            .or_else(|| exact_bound.as_ref().and_then(BoundInfo::magnitude_bits));

        RealStructuralFacts {
            sign,
            zero,
            exact_rational: exact.is_some(),
            magnitude,
        }
    }

    /// Conservatively report whether structural inspection proves this value is zero.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "zero_status", "exact-sign-cache");
            return if sign == Sign::NoSign {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            };
        }

        match self.cheap_bound() {
            BoundInfo::Zero => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-zero");
                ZeroKnowledge::Zero
            }
            BoundInfo::NonZero { .. } => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-nonzero");
                ZeroKnowledge::NonZero
            }
            BoundInfo::Unknown => {
                crate::trace_dispatch!("computable", "zero_status", "unknown");
                ZeroKnowledge::Unknown
            }
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    pub fn sign_until(&self, min_precision: Precision) -> Option<RealSign> {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "sign_until", "exact-sign-cache");
            return Some(public_sign(sign));
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.internal
                .facts
                .publish_exact_sign(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "approximation-cache-sign");
            return Some(public_sign(sign));
        }

        // Prefer structural facts before touching extra approximation work.
        // This keeps sign queries cheap when bounds are already strong enough.
        if let Some(sign) = self.cheap_bound().known_sign() {
            crate::trace_dispatch!("computable", "sign_until", "cheap-bound-sign");
            return Some(public_sign(sign));
        }

        if let Some(sign) = self.binary64_filter_sign_until(min_precision) {
            self.internal
                .facts
                .publish_exact_sign(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "binary64-filter-sign");
            return Some(public_sign(sign));
        }

        crate::trace_dispatch!("computable", "sign_until", "precision-refinement");
        let start = if min_precision > 0 { min_precision } else { 0 };
        let mut p = start;
        let mut checked_algebraic_bound = false;
        let mut algebraic_zero_precision = None;
        loop {
            let appr = self.approx(p).ok()?;
            #[cfg(feature = "dispatch-trace")]
            crate::dispatch_trace::record_sign_refinement_precision(
                "sign_until_attempt_precision",
                p,
            );
            if appr.abs() > BigInt::one() {
                let sign = appr.sign();
                self.internal
                    .facts
                    .publish_exact_sign(ExactSignCache::Valid(sign));
                #[cfg(feature = "dispatch-trace")]
                crate::dispatch_trace::record_sign_refinement_precision(
                    "sign_until_decision_precision",
                    p,
                );
                return Some(public_sign(sign));
            }

            // Ordinary nonzero inputs usually separate before 64 bits and pay
            // no algebraic-metadata cost. At a shallower negative caller floor,
            // inspect metadata only after ordinary refinement is exhausted;
            // this still makes every certificate that fits that floor usable.
            // A proved target then stops refinement without crossing the floor.
            if p <= -64 || (p <= min_precision && min_precision <= -2) {
                if !checked_algebraic_bound {
                    checked_algebraic_bound = true;
                    algebraic_zero_precision = self.algebraic_zero_precision(min_precision);
                    if algebraic_zero_precision.is_some() {
                        crate::trace_dispatch!(
                            "computable",
                            "sign_until",
                            "algebraic-separation-target"
                        );
                    } else {
                        crate::trace_dispatch!(
                            "computable",
                            "sign_until",
                            "algebraic-separation-unavailable"
                        );
                    }
                }
                if algebraic_zero_precision.is_some_and(|target| p <= target) {
                    self.internal
                        .facts
                        .publish_exact_sign(ExactSignCache::Valid(Sign::NoSign));
                    crate::trace_dispatch!("computable", "sign_until", "algebraic-separation-zero");
                    return Some(RealSign::Zero);
                }
            }

            if p <= min_precision {
                break;
            }
            let mut next = (p * 3) / 2 - 16;
            if let Some(target) = algebraic_zero_precision
                && target < p
                && next < target
            {
                next = target;
            }
            p = if next < min_precision {
                min_precision
            } else {
                next
            };
            if evaluation_should_stop(&self.signal) {
                return None;
            }
        }

        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "sign_until", "exact-rational-zero");
            Some(RealSign::Zero)
        } else {
            #[cfg(feature = "dispatch-trace")]
            crate::dispatch_trace::record_sign_refinement_precision(
                "sign_until_unresolved_precision",
                p,
            );
            crate::trace_dispatch!("computable", "sign_until", "unknown");
            None
        }
    }

    fn cached(&self) -> Option<(Precision, BigInt)> {
        if let Some(constant) = self.shared_constant_kind() {
            SHARED_CONSTANT_CACHES[constant.cache_index()].get()
        } else {
            self.internal.cached_value()
        }
    }

    /// Try to compare two computable values exactly.
    ///
    /// Returns `None` when bounded refinement cannot prove an ordering. This is
    /// the public comparison API for callers that may be comparing equal or
    /// semantically equivalent values.
    pub fn try_compare_to(&self, other: &Self) -> Option<Ordering> {
        // Keep the default entry point's certified fast cascade in this
        // function. Routing these overwhelmingly common cases through the
        // caller-configurable API adds a measurable extra call boundary.
        if Self::internal_structural_eq(self, other) {
            return Some(Ordering::Equal);
        }
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(order);
        }
        let exact_signs = (self.exact_sign(), other.exact_sign());
        if let (Some(left), Some(right)) = exact_signs {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Greater);
                }
                _ => {}
            }
        }
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(
                left.partial_cmp(&right)
                    .expect("exact rationals should be comparable"),
            );
        }
        if let (Some(left), Some(right)) = exact_signs
            && matches!(left, Sign::Plus | Sign::Minus)
            && left == right
            && let (Some(Some(left_msd)), Some(Some(right_msd))) = (
                self.cheap_bound().known_msd(),
                other.cheap_bound().known_msd(),
            )
            && left_msd != right_msd
        {
            crate::trace_dispatch!("computable", "compare_to", "cheap-bound-msd-gap");
            return Some(match left {
                Sign::Plus => left_msd.cmp(&right_msd),
                Sign::Minus => right_msd.cmp(&left_msd),
                Sign::NoSign => Ordering::Equal,
            });
        }
        self.try_compare_to_until(other, DEFAULT_COMPARE_REFINEMENT_FLOOR)
    }

    /// Try to compare two computable values without refining past
    /// `min_precision`.
    pub fn try_compare_to_until(&self, other: &Self, min_precision: Precision) -> Option<Ordering> {
        if Self::internal_structural_eq(self, other) {
            return Some(Ordering::Equal);
        }

        // Keep exact leaf comparisons allocation-free for the hot path where both
        // operands are already exact. This avoids creating temporary rationals
        // on every comparator call.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(order);
        }

        if let Some(order) = self.exact_shared_perturbation_order(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-shared-perturbation");
            return Some(order);
        }

        let exact_signs = (self.exact_sign(), other.exact_sign());
        if let (Some(left), Some(right)) = exact_signs {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Greater);
                }
                _ => {}
            }
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Exact rationals compare directly; escalating to approximate comparison here is
            // both slower and can burn cache precision unnecessarily.
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(
                left.partial_cmp(&right)
                    .expect("exact rationals should be comparable"),
            );
        }

        if let (Some(left), Some(right)) = exact_signs
            && matches!(left, Sign::Plus | Sign::Minus)
            && left == right
            && let (Some(Some(left_msd)), Some(Some(right_msd))) = (
                self.cheap_bound().known_msd(),
                other.cheap_bound().known_msd(),
            )
            && left_msd != right_msd
        {
            // Same-sign values with different most-significant digits have a known
            // order without evaluating either value to a requested precision.
            crate::trace_dispatch!("computable", "compare_to", "exact-sign-msd-gap");
            return Some(match left {
                Sign::Plus => left_msd.cmp(&right_msd),
                Sign::Minus => right_msd.cmp(&left_msd),
                Sign::NoSign => unreachable!(),
            });
        }

        let self_bound = self.cheap_bound();
        let other_bound = other.cheap_bound();
        let self_bound_sign = self_bound.known_sign();
        let other_bound_sign = other_bound.known_sign();
        if let (Some(left), Some(right)) = (self_bound_sign, other_bound_sign) {
            match (left, right) {
                (Sign::Minus, Sign::NoSign | Sign::Plus) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Some(Ordering::Greater);
                }
                (Sign::NoSign, Sign::NoSign) => return Some(Ordering::Equal),
                _ => {}
            }
            if left == right
                && let (Some(Some(left_msd)), Some(Some(right_msd))) =
                    (self_bound.known_msd(), other_bound.known_msd())
                && left_msd != right_msd
            {
                // Same-sign structural bounds can decide exact ordering
                // before entering tolerance refinement.
                crate::trace_dispatch!("computable", "compare_to", "cheap-bound-msd-gap");
                return Some(match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => Ordering::Equal,
                });
            }
        }
        crate::trace_dispatch!("computable", "compare_to", "approx-refinement");
        let mut tolerance = (-20).max(min_precision);
        loop {
            let order = self.compare_absolute(other, tolerance).ok()?;
            if order != Ordering::Equal {
                return Some(order);
            }
            if tolerance <= min_precision {
                return None;
            }
            let next = if tolerance >= 0 {
                tolerance.saturating_sub(16)
            } else {
                tolerance.saturating_mul(2)
            };
            tolerance = next.max(min_precision);
        }
    }

    /// Compare two values to a specified absolute tolerance.
    ///
    /// More negative tolerances are more precise. `Equal` generally means that
    /// the requested tolerance did not separate the values; it is not an exact
    /// equality certificate. Predicate callers must use
    /// [`Computable::try_compare_to_until`] and preserve `None`.
    /// Numeric evaluation failures propagate as [`Problem`] instead of an
    /// ordering. Exact structural comparisons need no approximation request.
    pub fn compare_absolute(
        &self,
        other: &Self,
        tolerance: Precision,
    ) -> Result<Ordering, Problem> {
        Ok({
            // Fast-path exact leafs before structural perturbation checks.
            if let Some(order) = self.exact_rational_leaf_cmp(other) {
                crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
                return Ok(order);
            }

            if let Approximation::Add(left, right) = &self.internal.approximation
                && let Some(order) = if Self::internal_structural_eq(left, other) {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "dominant-perturbation-self"
                    );
                    Self::dominant_perturbation_order(left, right, other, Some(tolerance))
                } else if Self::internal_structural_eq(right, other) {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "dominant-perturbation-self-reversed"
                    );
                    Self::dominant_perturbation_order(right, left, other, Some(tolerance))
                } else {
                    None
                }
            {
                return Ok(order);
            }
            if let Approximation::Add(left, right) = &other.internal.approximation
                && let Some(order) = if Self::internal_structural_eq(left, self) {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "dominant-perturbation-other"
                    );
                    Self::dominant_perturbation_order(left, right, self, Some(tolerance))
                } else if Self::internal_structural_eq(right, self) {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "dominant-perturbation-other-reversed"
                    );
                    Self::dominant_perturbation_order(right, left, self, Some(tolerance))
                } else {
                    None
                }
            {
                return Ok(order.reverse());
            }

            if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
                // Compare exact rationals without normalizing both operands. The
                // method name refers to its absolute error tolerance, not to
                // absolute values: opposite signs and comparisons with zero must
                // therefore be decided by signed order before comparing magnitudes.
                crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
                return Ok(match (left.sign(), right.sign()) {
                    (Sign::Minus, Sign::Minus) => right.compare_magnitude(&left),
                    (Sign::Minus, Sign::NoSign | Sign::Plus) => Ordering::Less,
                    (Sign::NoSign, Sign::Minus) => Ordering::Greater,
                    (Sign::NoSign, Sign::NoSign) => Ordering::Equal,
                    (Sign::NoSign, Sign::Plus) => Ordering::Less,
                    (Sign::Plus, Sign::Minus | Sign::NoSign) => Ordering::Greater,
                    (Sign::Plus, Sign::Plus) => left.compare_magnitude(&right),
                });
            }

            let self_sign = self.exact_sign();
            let other_sign = other.exact_sign();
            match (self_sign, other_sign) {
                // Exact signs can order zero and opposite-signed values directly.
                (Some(Sign::Minus), Some(Sign::NoSign | Sign::Plus))
                | (Some(Sign::NoSign), Some(Sign::Plus)) => return Ok(Ordering::Less),
                (Some(Sign::Plus), Some(Sign::Minus | Sign::NoSign))
                | (Some(Sign::NoSign), Some(Sign::Minus)) => return Ok(Ordering::Greater),
                (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ok(Ordering::Equal),
                _ => {}
            }

            // Keep bound derivation lazy: only ask cheap_bound when exact sign facts
            // cannot already determine the ordering.
            if self_sign.is_none() || other_sign.is_none() {
                let self_bound = self.cheap_bound();
                let other_bound = other.cheap_bound();
                let self_structural_sign = self_sign.or(self_bound.known_sign());
                let other_structural_sign = other_sign.or(other_bound.known_sign());
                let self_msd = self_bound.known_msd();
                let other_msd = other_bound.known_msd();

                if let (BoundInfo::Zero, BoundInfo::Zero) = (&self_bound, &other_bound) {
                    return Ok(Ordering::Equal);
                }
                match (self_structural_sign, other_structural_sign) {
                    (Some(Sign::Minus), Some(Sign::NoSign | Sign::Plus))
                    | (Some(Sign::NoSign), Some(Sign::Plus)) => return Ok(Ordering::Less),
                    (Some(Sign::Plus), Some(Sign::Minus | Sign::NoSign))
                    | (Some(Sign::NoSign), Some(Sign::Minus)) => return Ok(Ordering::Greater),
                    (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ok(Ordering::Equal),
                    _ => {}
                }
                if let (Some(left_sign), Some(right_sign), Some(left_msd), Some(right_msd)) = (
                    self_structural_sign,
                    other_structural_sign,
                    self_msd,
                    other_msd,
                ) && left_msd != right_msd
                {
                    crate::trace_dispatch!("computable", "compare_absolute", "exact-sign-msd-gap");
                    match (left_sign, right_sign) {
                        (Sign::Plus, Sign::Plus) => return Ok(left_msd.cmp(&right_msd)),
                        (Sign::Minus, Sign::Minus) => return Ok(right_msd.cmp(&left_msd)),
                        _ => {}
                    }
                }
                // A known magnitude without a known sign cannot order values.
                // Same-sign values with exact, distinct MSDs already returned
                // above with the correct negative-value reversal.
            } else if self_sign == other_sign {
                let self_bound = self.cheap_bound();
                let other_bound = other.cheap_bound();
                if let (Some(Some(self_msd)), Some(Some(other_msd))) =
                    (self_bound.known_msd(), other_bound.known_msd())
                    && let (Some(left_sign), Some(right_sign)) = (self_sign, other_sign)
                    && left_sign == right_sign
                    && self_msd != other_msd
                {
                    crate::trace_dispatch!("computable", "compare_absolute", "exact-sign-msd-gap");
                    return Ok(match (left_sign, right_sign) {
                        (Sign::Plus, Sign::Plus) => self_msd.cmp(&other_msd),
                        (Sign::Minus, Sign::Minus) => other_msd.cmp(&self_msd),
                        _ => Ordering::Equal,
                    });
                }
            }
            crate::trace_dispatch!("computable", "compare_absolute", "approx-refinement");
            let needed = tolerance.checked_sub(1).ok_or(Problem::OutOfRange)?;
            let this = self.approx(needed)?;
            let alt = other.approx(needed)?;
            let max = alt.clone() + signed::ONE.deref();
            let min = alt.clone() - signed::ONE.deref();
            if this > max {
                Ordering::Greater
            } else if this < min {
                Ordering::Less
            } else {
                Ordering::Equal
            }
        })
    }

    #[inline]
    fn exact_rational_leaf_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.internal.approximation, &other.internal.approximation) {
            (Approximation::Ratio(left), Approximation::Ratio(right)) => left.partial_cmp(right),
            (Approximation::Int(left), Approximation::Int(right)) => Some(left.cmp(right)),
            (Approximation::One, Approximation::One) => Some(Ordering::Equal),
            (Approximation::One, Approximation::Int(right)) => Some(BigInt::one().cmp(right)),
            (Approximation::Int(left), Approximation::One) => Some(left.cmp(&BigInt::one())),
            _ => None,
        }
    }

    /// Most Significant Digit (Bit).
    /// Requires a cached approximation separated from zero and a representable exponent.
    fn known_msd(&self) -> Result<Precision, Problem> {
        Ok({
            if let Some((prec, appr)) = self.cached() {
                crate::verified::word::checked_bit_length_msd(
                    appr.magnitude().bits(),
                    1,
                    false,
                    prec,
                )
                .ok_or(Problem::OutOfRange)?
            } else {
                return Err(Problem::UnknownZero);
            }
        })
    }

    /// Most Significant Digit - or perhaps None if as yet undiscovered and less than p.
    #[cfg(test)]
    pub(crate) fn msd(&self, p: Precision) -> Result<Option<Precision>, Problem> {
        self.msd_signal(&self.signal, p)
    }

    pub(super) fn msd_signal(
        &self,
        signal: &Option<Signal>,
        p: Precision,
    ) -> Result<Option<Precision>, Problem> {
        Ok({
            if let Some(msd) = self.cheap_bound().known_msd() {
                return Ok(msd);
            }

            let cache = self.cached();
            let mut try_once = false;

            if cache.is_none() {
                try_once = true;
            } else if let Some((_prec, appr)) = cache {
                let one = signed::ONE.deref();
                let minus_one = signed::MINUS_ONE.deref();

                // Approximation results carry an absolute error of at most one at
                // their scale. Magnitude one therefore does not prove a leading
                // bit; refine until the approximation is separated from zero.
                if appr >= *minus_one && appr <= *one {
                    try_once = true;
                }
            }

            if try_once {
                let appr =
                    self.approx_signal(signal, p.checked_sub(1).ok_or(Problem::OutOfRange)?)?;
                if appr.magnitude() <= &BigUint::one() {
                    return Ok(None);
                }
            }

            Some(self.known_msd()?)
        })
    }

    const STOP_PRECISION: Precision = Precision::MIN / 3;

    /// MSD iteratively: 0, -16, -40, -76 etc. or p if that's lower.
    /// You can choose p to avoid unnecessary work.
    pub(crate) fn iter_msd_stop(&self, p: Precision) -> Result<Option<Precision>, Problem> {
        self.iter_msd_stop_signal(&self.signal, p)
    }

    pub(super) fn iter_msd_stop_signal(
        &self,
        signal: &Option<Signal>,
        p: Precision,
    ) -> Result<Option<Precision>, Problem> {
        Ok({
            let mut prec = 0;

            loop {
                let msd = self.msd_signal(signal, prec)?;
                if msd.is_some() {
                    return Ok(msd);
                }
                prec = prec
                    .checked_mul(3)
                    .and_then(|p| (p / 2).checked_sub(16))
                    .ok_or(Problem::OutOfRange)?;
                if prec <= p {
                    break;
                }
                if evaluation_should_stop(signal) {
                    return Err(Problem::Exhausted);
                }
            }
            self.msd_signal(signal, p)?
        })
    }

    /// MSD but iteratively without a guess as to precision.
    pub(super) fn iter_msd(&self) -> Result<Precision, Problem> {
        Ok({
            self.iter_msd_stop(Self::STOP_PRECISION)?
                .ok_or(Problem::UnknownZero)?
        })
    }
}
