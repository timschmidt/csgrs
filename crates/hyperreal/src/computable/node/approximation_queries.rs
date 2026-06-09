impl Computable {
    /// An approximation of this Computable scaled to a specific precision.
    ///
    /// Since the value is scaled, the approximation is roughly `value * 2^p`.
    /// Negative values of `p` request more precision.
    ///
    /// The approximation is scaled (thus, a larger value for more negative p)
    /// and should be accurate to within +/- 1 at the scale provided.
    ///
    /// Example: 0.875 is between 0 and 1 with zero bits of extra precision
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let n = Rational::fraction(7, 8).unwrap();
    /// let comp = Computable::rational(n);
    /// assert!((BigInt::zero() ..= BigInt::one()).contains(&comp.approx(0)));
    /// ```
    ///
    /// Example: π * 2³ is a bit more than 25 but less than 26
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let pi = Computable::pi();
    /// let between_25_26 = (ToBigInt::to_bigint(&25).unwrap() ..= ToBigInt::to_bigint(&26).unwrap());
    /// assert!(between_25_26.contains(&pi.approx(-3)));
    /// ```
    pub fn approx(&self, p: Precision) -> BigInt {
        self.approx_signal(&self.signal, p)
    }

    /// Like `approx` but specifying an atomic abort/ stop signal.
    pub fn approx_signal(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        enum Frame<'a> {
            Eval(&'a Computable, Precision),
            FinishNegate(&'a Computable, Precision),
            FinishAdd(&'a Computable, Precision),
            FinishOffset(&'a Computable, Precision),
        }

        if let Some(cached) = self.cached_at_precision(p) {
            return cached;
        }

        if !matches!(
            &*self.internal,
            Approximation::Negate(_) | Approximation::Add(_, _) | Approximation::Offset(_, _)
        ) {
            // Most node kinds evaluate as one kernel call. Only Negate/Add/Offset
            // are flattened below because they form the long chains seen in
            // parser, matrix, and structural-reduction workloads.
            let result = self.internal.approximate(signal, p);
            self.store_cache_value(p, result.clone());
            return result;
        }

        // Reserve a modest stack size for the flattened traversal path so long
        // chains of Negate/Add/Offset avoid repeated allocations.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<BigInt> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self, p));

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node, prec) => {
                    if let Some(cached) = node.cached_at_precision(prec) {
                        values.push(cached);
                        continue;
                    }

                    match &*node.internal {
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
                            frames.push(Frame::Eval(right, prec - 2));
                            frames.push(Frame::Eval(left, prec - 2));
                        }
                        Approximation::Offset(child, n) => {
                            // Binary offsets translate the requested precision
                            // instead of doing any arithmetic at finish time.
                            frames.push(Frame::FinishOffset(node, prec));
                            frames.push(Frame::Eval(child, prec - *n));
                        }
                        _ => {
                            let result = node.internal.approximate(signal, prec);
                            node.store_cache_value(prec, result.clone());
                            values.push(result);
                        }
                    }
                }
                Frame::FinishNegate(node, prec) => {
                    let result = -values.pop().expect("negate child result should exist");
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
                Frame::FinishAdd(node, prec) => {
                    let right = values.pop().expect("add rhs result should exist");
                    let left = values.pop().expect("add lhs result should exist");
                    let result = scale(left + right, -2);
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
                Frame::FinishOffset(node, prec) => {
                    let result = values.pop().expect("offset child result should exist");
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
            }
        }

        values.pop().expect("evaluation should produce a result")
    }

    /// Conservatively inspect cached and structural numeric facts.
    pub fn structural_facts(&self) -> RealStructuralFacts {
        let exact = self.exact_rational();

        let mut sign = self.exact_sign().map(public_sign);
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

        let bound = self.cheap_bound();
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
            self.exact_sign.replace(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "approximation-cache-sign");
            return Some(public_sign(sign));
        }

        // Prefer structural facts before touching extra approximation work.
        // This keeps sign queries cheap when bounds are already strong enough.
        if let Some(sign) = self.cheap_bound().known_sign() {
            crate::trace_dispatch!("computable", "sign_until", "cheap-bound-sign");
            return Some(public_sign(sign));
        }

        crate::trace_dispatch!("computable", "sign_until", "precision-refinement");
        let start = if min_precision > 0 { min_precision } else { 0 };
        let mut p = start;
        loop {
            let appr = self.approx(p);
            if appr.abs() > BigInt::one() {
                let sign = appr.sign();
                self.exact_sign.replace(ExactSignCache::Valid(sign));
                return Some(public_sign(sign));
            }

            if p <= min_precision {
                break;
            }
            let next = (p * 3) / 2 - 16;
            p = if next < min_precision {
                min_precision
            } else {
                next
            };
            if should_stop(&self.signal) {
                break;
            }
        }

        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "sign_until", "exact-rational-zero");
            Some(RealSign::Zero)
        } else {
            crate::trace_dispatch!("computable", "sign_until", "unknown");
            None
        }
    }

    /// Try to determine the exact sign, refining cached approximations as needed.
    pub fn sign(&self) -> Sign {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "sign", "exact-sign-cache");
            return sign;
        }
        {
            let cache = self.cache.borrow();
            if let Cache::Valid((_prec, cache_appr)) = &*cache {
                let sign = cache_appr.sign();
                if sign != Sign::NoSign {
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    crate::trace_dispatch!("computable", "sign", "approximation-cache-sign");
                    return sign;
                }
            }
        }
        // Delay approximation refinement until after structural information has
        // had a chance to prove the sign. This avoids precision work for
        // purely symbolic queries.
        if let Some(sign) = self.cheap_bound().known_sign() {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign", "cheap-bound-sign");
            return sign;
        }
        crate::trace_dispatch!("computable", "sign", "precision-refinement");
        let mut sign = Sign::NoSign;
        let mut p = 0;
        while p > -2000 && sign == Sign::NoSign {
            let appr = self.approx(p);
            p -= 10;
            sign = appr.sign();
        }
        if sign != Sign::NoSign {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
        }
        sign
    }

    fn cached(&self) -> Option<(Precision, BigInt)> {
        if let Some(constant) = self.shared_constant_kind() {
            SHARED_CONSTANT_CACHES.with(|caches| {
                let caches = caches.borrow();
                match &caches[constant.cache_index()] {
                    Cache::Valid((cache_prec, cache_appr)) => {
                        Some((*cache_prec, cache_appr.clone()))
                    }
                    Cache::Invalid => None,
                }
            })
        } else {
            let cache = self.cache.borrow();
            if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
                Some((*cache_prec, cache_appr.clone()))
            } else {
                None
            }
        }
    }

    /// Try to compare two computable values exactly.
    ///
    /// Returns `None` when bounded refinement cannot prove an ordering. This is
    /// the public comparison API for callers that may be comparing equal or
    /// semantically equivalent values.
    pub fn try_compare_to(&self, other: &Self) -> Option<Ordering> {
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

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Exact rationals compare directly; escalating to approximate comparison here is
            // both slower and can burn cache precision unnecessarily.
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(
                left.partial_cmp(&right)
                    .expect("exact rationals should be comparable"),
            );
        }

        if let (Some(left), Some(right)) = (self.exact_sign(), other.exact_sign()) {
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

            if matches!(left, Sign::Plus | Sign::Minus)
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
        }

        let self_bound = self.cheap_bound();
        let other_bound = other.cheap_bound();
        let self_bound_sign = self_bound.known_sign();
        let other_bound_sign = other_bound.known_sign();
        if let (Some(left), Some(right)) = (self_bound_sign, other_bound_sign) {
            match (left, right) {
                (Sign::Minus, Sign::Plus) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus) | (Sign::Plus, Sign::NoSign) => {
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
        let mut tolerance = -20;
        while tolerance > Precision::MIN {
            let order = self.compare_absolute(other, tolerance);
            if order != Ordering::Equal {
                return Some(order);
            }
            tolerance *= 2;
        }
        None
    }

    /// Compare two values to a specified tolerance (more negative numbers are more precise).
    pub fn compare_absolute(&self, other: &Self, tolerance: Precision) -> Ordering {
        // Fast-path exact leafs before structural perturbation checks.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return order;
        }

        if let Approximation::Add(left, right) = &*self.internal
            && let Some(order) = if Self::internal_structural_eq(left, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self"
                );
                Self::compare_absolute_dominant_perturbation(left, right, other, tolerance)
            } else if Self::internal_structural_eq(right, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self-reversed"
                );
                Self::compare_absolute_dominant_perturbation(right, left, other, tolerance)
            } else {
                None
            }
        {
            return order;
        }
        if let Approximation::Add(left, right) = &*other.internal
            && let Some(order) = if Self::internal_structural_eq(left, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other"
                );
                Self::compare_absolute_dominant_perturbation(left, right, self, tolerance)
            } else if Self::internal_structural_eq(right, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other-reversed"
                );
                Self::compare_absolute_dominant_perturbation(right, left, self, tolerance)
            } else {
                None
            }
        {
            return order.reverse();
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Compare exact-rational magnitudes without normalizing both operands.
            // This keeps the absolute-ordering branch allocation-light for symbolically
            // small values that are hit in compare-heavy workloads.
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return match (left.sign(), right.sign()) {
                (Sign::Minus, Sign::Minus) => right.compare_magnitude(&left),
                (Sign::Minus, Sign::Plus) => left.compare_magnitude(&right),
                (Sign::Plus, Sign::Minus) => left.compare_magnitude(&right),
                (Sign::Plus, Sign::Plus) => left.compare_magnitude(&right),
                (_, Sign::NoSign) => Ordering::Greater,
                (Sign::NoSign, _) => Ordering::Less,
            };
        }

        let self_sign = self.exact_sign();
        let other_sign = other.exact_sign();
        match (self_sign, other_sign) {
            // Exact signs can prove the nonzero ordering of absolute values.
            (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
            (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
            (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
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
                return Ordering::Equal;
            }
            match (self_structural_sign, other_structural_sign) {
                (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
                (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
                (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
                (Some(Sign::Minus), Some(Sign::Plus)) => return Ordering::Less,
                (Some(Sign::Plus), Some(Sign::Minus)) => return Ordering::Greater,
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
                    (Sign::Plus, Sign::Plus) => return left_msd.cmp(&right_msd),
                    (Sign::Minus, Sign::Minus) => return right_msd.cmp(&left_msd),
                    _ => {}
                }
            }
            if let (Some(Some(left_msd)), Some(Some(right_msd))) = (self_msd, other_msd) {
                if left_msd > tolerance && right_msd < tolerance {
                    // Cheap MSD bounds can prove a tolerance-separated absolute ordering
                    // before allocating fresh approximations.
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "exact-sign-tolerance-gap"
                    );
                    return Ordering::Greater;
                }
                if right_msd > tolerance && left_msd < tolerance {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "exact-sign-tolerance-gap"
                    );
                    return Ordering::Less;
                }
            }
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
                return match (left_sign, right_sign) {
                    (Sign::Plus, Sign::Plus) => self_msd.cmp(&other_msd),
                    (Sign::Minus, Sign::Minus) => other_msd.cmp(&self_msd),
                    _ => Ordering::Equal,
                };
            }
        }
        crate::trace_dispatch!("computable", "compare_absolute", "approx-refinement");
        let needed = tolerance - 1;
        let this = self.approx(needed);
        let alt = other.approx(needed);
        let max = alt.clone() + signed::ONE.deref();
        let min = alt.clone() - signed::ONE.deref();
        if this > max {
            Ordering::Greater
        } else if this < min {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }

    #[inline]
    fn exact_rational_leaf_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&*self.internal, &*other.internal) {
            (Approximation::Ratio(left), Approximation::Ratio(right)) => left.partial_cmp(right),
            (Approximation::Int(left), Approximation::Int(right)) => Some(left.cmp(right)),
            (Approximation::One, Approximation::One) => Some(Ordering::Equal),
            (Approximation::One, Approximation::Int(right)) => Some(BigInt::one().cmp(right)),
            (Approximation::Int(left), Approximation::One) => Some(left.cmp(&BigInt::one())),
            _ => None,
        }
    }

    /// Most Significant Digit (Bit).
    /// May panic or give incorrect answers if not yet discovered.
    fn known_msd(&self) -> Precision {
        if let Some((prec, appr)) = self.cached() {
            let length = appr.magnitude().bits() as Precision;
            prec + length - 1
        } else {
            panic!("Expected valid cache state for known MSD but it's invalid")
        }
    }

    /// Most Significant Digit - or perhaps None if as yet undiscovered and less than p.
    pub(crate) fn msd(&self, p: Precision) -> Option<Precision> {
        if let Some(msd) = self.cheap_bound().known_msd() {
            return msd;
        }

        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            let one = signed::ONE.deref();
            let minus_one = signed::MINUS_ONE.deref();

            if appr > *minus_one && appr < *one {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() < &BigUint::one() {
                return None;
            }
        }

        Some(self.known_msd())
    }

    const STOP_PRECISION: Precision = Precision::MIN / 3;

    /// MSD iteratively: 0, -16, -40, -76 etc. or p if that's lower.
    /// You can choose p to avoid unnecessary work.
    pub(crate) fn iter_msd_stop(&self, p: Precision) -> Option<Precision> {
        let mut prec = 0;

        loop {
            let msd = self.msd(prec);
            if msd.is_some() {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= p {
                break;
            }
            if should_stop(&self.signal) {
                break;
            }
        }
        self.msd(p)
    }

    /// MSD but iteratively without a guess as to precision.
    pub(super) fn iter_msd(&self) -> Precision {
        self.iter_msd_stop(Self::STOP_PRECISION)
            .unwrap_or(Self::STOP_PRECISION)
    }
}
