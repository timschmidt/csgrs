impl Computable {
    pub(crate) fn exp_rational(r: Rational) -> Self {
        if r.is_one() {
            // e^1 is hot enough to route to the shared e cache.
            Self::e_constant()
        } else {
            let rational = Self::rational(r);
            Self::exp(rational)
        }
    }

    fn shared_constant_kind(&self) -> Option<SharedConstant> {
        match &*self.internal {
            Approximation::Constant(constant) => Some(*constant),
            _ => None,
        }
    }

    fn power_of_two_rational(shift: Precision) -> Rational {
        if shift >= 0 {
            Rational::from_bigint(BigInt::one() << shift as usize)
        } else {
            Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << (-shift) as usize)
                .unwrap()
        }
    }

    fn shared_constant_term(&self) -> Option<(SharedConstant, Rational)> {
        // Recognize "exact rational scale times one shared constant" through
        // lightweight wrappers. This supports pi-3/e-2 style sign certificates
        // without needing a full symbolic Real class.
        match &*self.internal {
            Approximation::Constant(constant) => Some((*constant, Rational::one())),
            Approximation::Negate(child) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale.neg()))
            }
            Approximation::Offset(child, shift) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale * Self::power_of_two_rational(*shift)))
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    let (constant, inner_scale) = right.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                if let Some(scale) = right.exact_rational() {
                    let (constant, inner_scale) = left.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                None
            }
            _ => None,
        }
    }

    fn integer_pi_plus_rational(&self) -> Option<(BigInt, Rational)> {
        // Trig reducers often see values like k*pi + r after symbolic algebra.
        // If k is an exact integer, the period/parity can be handled without
        // estimating a quotient or building a cancellation-prone residual.
        fn extract(term: &Computable, offset: &Computable) -> Option<(BigInt, Rational)> {
            let rational = offset.exact_rational()?;
            let residual_is_kernel_sized = rational.sign() == Sign::NoSign
                || rational.msd_exact().is_some_and(|msd| msd < 0)
                || Computable::exact_rational_half_pi_shortcut_magnitude(&rational).is_some();
            if !residual_is_kernel_sized {
                return None;
            }
            let (constant, scale) = term.shared_constant_term()?;
            let pi_scale = match constant {
                SharedConstant::Pi => scale,
                SharedConstant::Tau => scale * Rational::new(2),
                _ => return None,
            };
            pi_scale
                .to_big_integer()
                .map(|multiple| (multiple, rational))
        }

        match &*self.internal {
            Approximation::Add(left, right) => {
                extract(left, right).or_else(|| extract(right, left))
            }
            _ => None,
        }
    }

    fn bound_from_strict_interval(lower: Rational, upper: Rational) -> BoundInfo {
        // Convert an interval that excludes zero into a reusable sign/MSD
        // certificate. If the interval crosses zero, preserve correctness by
        // returning Unknown.
        let zero = Rational::zero();
        let (sign, magnitude_lower, magnitude_upper) = if lower > zero {
            (Sign::Plus, lower, upper)
        } else if upper < zero {
            (Sign::Minus, upper.neg(), lower.neg())
        } else {
            return BoundInfo::Unknown;
        };

        let lower_msd = magnitude_lower.msd_exact();
        let upper_msd = magnitude_upper.msd_exact();
        let (msd, exact_msd) = match (lower_msd, upper_msd) {
            (Some(lower), Some(upper)) if lower == upper => (Some(lower), true),
            (Some(lower), Some(upper)) => (Some(lower.max(upper)), false),
            _ => (None, false),
        };

        BoundInfo::with_sign_msd(sign, msd, exact_msd)
    }

    fn constant_rational_sum_bound(
        term: &(SharedConstant, Rational),
        rational: &Rational,
    ) -> BoundInfo {
        // Specialized structural bound for c*K + q where K is a shared constant.
        // This is the computable-side companion to Real's ConstOffset class and
        // keeps generic Add nodes for pi-3 from needing approximation refinement.
        let (constant, scale) = term;
        let (lower, upper) = constant.interval();
        let scaled_lower = lower * scale;
        let scaled_upper = upper * scale;
        let (lower, upper) = if scaled_lower <= scaled_upper {
            (scaled_lower, scaled_upper)
        } else {
            (scaled_upper, scaled_lower)
        };

        Self::bound_from_strict_interval(lower + rational, upper + rational)
    }

    fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        // A cached value at precision q can answer any less precise request p
        // by shifting, but not a more precise one. Shared constants use the
        // thread-local cache; other nodes keep their cache beside the node.
        if let Some(constant) = self.shared_constant_kind() {
            if let Some(cached) = Self::cached_shared_constant_at_precision(constant, p) {
                return Some(cached);
            }
            if constant == SharedConstant::Tau
                && let Some(cached) =
                    Self::cached_shared_constant_at_precision(SharedConstant::Pi, p - 1)
            {
                // tau is exactly 2*pi, so a pi approximation at precision p-1
                // is already a tau approximation at precision p. Populate the
                // tau cache from pi instead of re-running the Machin pi kernel
                // when callers ask for tau after pi has been warmed.
                Self::store_shared_constant_cache_value(SharedConstant::Tau, p, cached.clone());
                return Some(cached);
            }
            if constant == SharedConstant::Pi
                && let Some(cached) =
                    Self::cached_shared_constant_at_precision(SharedConstant::Tau, p + 1)
            {
                // The same identity works in reverse: a tau approximation at
                // precision p+1 is already a pi approximation at precision p.
                // This matters for applications that use tau for trig
                // construction and later format pi; reuse the costly Machin
                // approximation instead of recomputing it under a different
                // shared-constant key.
                Self::store_shared_constant_cache_value(SharedConstant::Pi, p, cached.clone());
                return Some(cached);
            }
            return None;
        }

        let cache = self.cache.borrow();
        let cached = if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
            Some((*cache_prec, cache_appr.clone()))
        } else {
            None
        }?;

        if p >= cached.0 {
            if p == cached.0 {
                // Reusing the exact cached precision avoids a no-op BigInt shift.
                Some(cached.1)
            } else {
                Some(scale(cached.1, cached.0 - p))
            }
        } else {
            None
        }
    }

    fn cached_shared_constant_at_precision(
        constant: SharedConstant,
        p: Precision,
    ) -> Option<BigInt> {
        SHARED_CONSTANT_CACHES.with(|caches| {
            let caches = caches.borrow();
            let Cache::Valid((cache_prec, cache_appr)) = &caches[constant.cache_index()] else {
                return None;
            };
            if p >= *cache_prec {
                if p == *cache_prec {
                    // Reusing shared-constant precision avoids extra shift work.
                    Some(cache_appr.clone())
                } else {
                    Some(scale(cache_appr.clone(), *cache_prec - p))
                }
            } else {
                None
            }
        })
    }

    fn store_shared_constant_cache_value(constant: SharedConstant, p: Precision, value: BigInt) {
        SHARED_CONSTANT_CACHES.with(|caches| {
            caches.borrow_mut()[constant.cache_index()] = Cache::Valid((p, value));
        });
    }

    fn store_cache_value(&self, p: Precision, value: BigInt) {
        // Store only exact node approximation results, not temporary scaled
        // values. For shared constants this updates the global thread-local
        // cache so every cloned constant wrapper benefits.
        if let Some(constant) = self.shared_constant_kind() {
            Self::store_shared_constant_cache_value(constant, p, value);
        } else {
            self.cache.replace(Cache::Valid((p, value)));
        }
    }

    fn cached_bound(&self) -> Option<BoundInfo> {
        match self.bound.get() {
            BoundCache::Invalid => None,
            BoundCache::Valid(info) => Some(info),
        }
    }

    fn store_bound(&self, info: &BoundInfo) {
        // Unknown facts are intentionally not cached; a later approximation may
        // discover a real sign/MSD and should be allowed to populate the cache.
        if *info != BoundInfo::Unknown {
            self.bound.set(BoundCache::Valid(*info));
        }
    }

    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        if appr.abs() <= BigInt::one() {
            BoundInfo::Unknown
        } else {
            BoundInfo::with_sign_msd(
                appr.sign(),
                Some(prec + appr.magnitude().bits() as Precision - 1),
                false,
            )
        }
    }

    fn cheap_bound_shallow(&self, budget: usize) -> Option<BoundInfo> {
        // First try a shallow recursive walk. It is faster for common small
        // trees and avoids allocating the explicit stack used by deep chains.
        if let Some(info) = self.cached_bound() {
            return Some(info);
        }
        if budget == 0 {
            return None;
        }
        let info = match &*self.internal {
            Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                BoundInfo::Zero
            } else {
                BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
            }),
            Approximation::Constant(constant) => Some(constant.bound_info()),
            Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
            Approximation::AtanRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledCosRational(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
            Approximation::PrescaledCosHalfPiMinusRational(_)
            | Approximation::PrescaledSinHalfPiMinusRational(_)
            | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
            Approximation::Negate(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::negate)
            }
            Approximation::Offset(child, n) => child
                .cheap_bound_shallow(budget - 1)
                .map(|bound| bound.map_msd(|value| value + *n)),
            Approximation::Inverse(child) => child
                .cheap_bound_shallow(budget - 1)
                .map(BoundInfo::inverse),
            Approximation::Square(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::square)
            }
            Approximation::Sqrt(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::sqrt)
            }
            Approximation::Multiply(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.multiply(right))
            }
            Approximation::Add(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.add(right))
            }
            _ => Some(if let Some((prec, appr)) = self.cached() {
                Self::bound_from_approx(prec, &appr)
            } else {
                BoundInfo::Unknown
            }),
        };
        if let Some(ref value) = info {
            self.store_bound(value);
        }
        info
    }

    fn cheap_bound(&self) -> BoundInfo {
        const SHALLOW_BOUND_BUDGET: usize = 24;

        // The public structural API leans on this method heavily. It must stay
        // conservative: a false NonZero or sign certificate is a correctness
        // bug, while Unknown only costs later refinement.
        if let Some(info) = self.cached_bound() {
            return info;
        }

        if let Some(bound) = self.cheap_bound_shallow(SHALLOW_BOUND_BUDGET) {
            return bound;
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate,
            FinishOffset(i32),
            FinishInverse,
            FinishSquare,
            FinishSqrt,
            FinishAdd,
            FinishMultiply,
        }

        fn direct_bound(node: &Computable) -> Option<BoundInfo> {
            match &*node.internal {
                Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
                Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                    BoundInfo::Zero
                } else {
                    BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
                }),
                Approximation::Constant(constant) => Some(constant.bound_info()),
                Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
                Approximation::AtanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledCosRational(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(if let Some((prec, appr)) = node.cached() {
                    Computable::bound_from_approx(prec, &appr)
                } else {
                    BoundInfo::Unknown
                }),
            }
        }

        // Reserve small fixed-size stacks because bound queries are often called
        // on long symbolic chains and should not allocate repeatedly under
        // repeated structural fact traffic.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<BoundInfo> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

        // Deep addition/multiplication chains are common after algebra kernels.
        // Use an explicit stack so structural fact discovery cannot recurse
        // through thousands of nodes.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(bound) = direct_bound(node) {
                        values.push(bound);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, n) => {
                            frames.push(Frame::FinishOffset(*n));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("direct_bound should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate => {
                    let value = values.pop().expect("negate bound should exist");
                    values.push(value.negate());
                }
                Frame::FinishOffset(offset) => {
                    let value = values.pop().expect("offset bound should exist");
                    values.push(value.map_msd(|msd| msd + offset));
                }
                Frame::FinishInverse => {
                    let value = values.pop().expect("inverse bound should exist");
                    values.push(value.inverse());
                }
                Frame::FinishSquare => {
                    let value = values.pop().expect("square bound should exist");
                    values.push(value.square());
                }
                Frame::FinishSqrt => {
                    let value = values.pop().expect("sqrt bound should exist");
                    values.push(value.sqrt());
                }
                Frame::FinishAdd => {
                    let right = values.pop().expect("add rhs bound should exist");
                    let left = values.pop().expect("add lhs bound should exist");
                    values.push(left.add(right));
                }
                Frame::FinishMultiply => {
                    let right = values.pop().expect("multiply rhs bound should exist");
                    let left = values.pop().expect("multiply lhs bound should exist");
                    values.push(left.multiply(right));
                }
            }
        }

        let result = values
            .pop()
            .expect("bound evaluation should produce a result");
        self.store_bound(&result);
        result
    }

    fn exact_sign(&self) -> Option<Sign> {
        // `exact_sign` is stronger than "current approximation sign": it means
        // the expression shape or a separated cached approximation proves the
        // sign. Unknown is cached separately so impossible structural proofs do
        // not repeat on every predicate query.
        let cached_sign = self.exact_sign.get();
        match cached_sign {
            ExactSignCache::Valid(sign) => return Some(sign),
            ExactSignCache::Unknown => {
                if let Some((_, appr)) = self.cached()
                    && appr.abs() > BigInt::one()
                {
                    let sign = appr.sign();
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    return Some(sign);
                }
                return None;
            }
            ExactSignCache::Invalid => {}
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate(&'a Computable),
            FinishOffset(&'a Computable),
            FinishInverse(&'a Computable),
            FinishSquare(&'a Computable),
            FinishSqrt(&'a Computable),
            FinishAdd(&'a Computable),
            FinishMultiply(&'a Computable),
        }

        fn cached_exact_sign(node: &Computable) -> Option<Option<Sign>> {
            let cached_sign = node.exact_sign.get();
            match cached_sign {
                ExactSignCache::Invalid => None,
                ExactSignCache::Unknown => {
                    if let Some((_, appr)) = node.cached()
                        && appr.abs() > BigInt::one()
                    {
                        let sign = appr.sign();
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                        Some(Some(sign))
                    } else {
                        Some(None)
                    }
                }
                ExactSignCache::Valid(sign) => Some(Some(sign)),
            }
        }

        fn exact_sign_direct(node: &Computable) -> Option<Option<Sign>> {
            // Direct cases either know their sign structurally or are known not
            // to be structurally decidable without visiting children.
            if let Some(sign) = cached_exact_sign(node) {
                return Some(sign);
            }

            if let Some((_, appr)) = node.cached()
                && appr.abs() > BigInt::one()
            {
                return Some(Some(appr.sign()));
            }

            match &*node.internal {
                Approximation::One => Some(Some(Sign::Plus)),
                Approximation::Int(n) => Some(Some(n.sign())),
                Approximation::Constant(_) => Some(Some(Sign::Plus)),
                Approximation::Ratio(r) => Some(Some(r.sign())),
                Approximation::IntegralAtan(n) => Some(Some(n.sign())),
                Approximation::AtanRational(r) => Some(Some(r.sign())),
                Approximation::AsinRational(r) => Some(Some(r.sign())),
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(Some(r.sign()))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(Some(r.sign()))
                }
                Approximation::PrescaledCosRational(_) => Some(Some(Sign::Plus)),
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => Some(Some(Sign::Plus)),
                Approximation::AcosPositive(_)
                | Approximation::AcosPositiveRational(_)
                | Approximation::AcosNegativeRational(_)
                | Approximation::AcoshNearOne(_)
                | Approximation::AcoshDirect(_)
                | Approximation::Erfc(_)
                | Approximation::NormalSf(_)
                | Approximation::NormalInterval { .. } => Some(Some(Sign::Plus)),
                Approximation::LogPnorm(_)
                | Approximation::LogNormalSf(_)
                | Approximation::LogDnorm(_) => Some(Some(Sign::Minus)),
                Approximation::PrescaledAtan(child)
                | Approximation::PrescaledAsin(child)
                | Approximation::AsinDeferred(child)
                | Approximation::AsinhNearZero(child)
                | Approximation::AsinhDirect(child)
                | Approximation::PrescaledAsinh(child)
                | Approximation::AtanhDirect(child)
                | Approximation::PrescaledAtanh(child)
                | Approximation::Expm1(child) => Some(child.exact_sign()),
                Approximation::PrescaledExp(_) => Some(Some(Sign::Plus)),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(None),
            }
        }

        fn store_exact_sign(node: &Computable, sign: Option<Sign>) {
            node.exact_sign.replace(match sign {
                Some(sign) => ExactSignCache::Valid(sign),
                None => ExactSignCache::Unknown,
            });
        }

        // Structural sign on deep chains stays allocation-light so predicate-heavy
        // code does not needlessly allocate during exact-sign walks.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<Option<Sign>> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

        // Mirror cheap_bound's nonrecursive traversal for deep structural
        // expressions. This matters for predicate-heavy code that asks only for
        // sign and never needs numeric approximation.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(sign) = exact_sign_direct(node) {
                        store_exact_sign(node, sign);
                        values.push(sign);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, _) => {
                            frames.push(Frame::FinishOffset(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("exact_sign_direct should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate(node) => {
                    let value = values.pop().expect("negate sign should exist");
                    let result = value.map(negate_sign);
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishOffset(node) => {
                    let value = values.pop().expect("offset sign should exist");
                    store_exact_sign(node, value);
                    values.push(value);
                }
                Frame::FinishInverse(node) => {
                    let value = values.pop().expect("inverse sign should exist");
                    let result = match value {
                        Some(Sign::Plus) => Some(Sign::Plus),
                        Some(Sign::Minus) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSquare(node) => {
                    let value = values.pop().expect("square sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(_) => Some(Sign::Plus),
                        None => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSqrt(node) => {
                    let value = values.pop().expect("sqrt sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(Sign::Plus) => Some(Sign::Plus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishAdd(node) => {
                    let right = values.pop().expect("add rhs sign should exist");
                    let left = values.pop().expect("add lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), sign) | (sign, Some(Sign::NoSign)) => sign,
                        (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                        (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishMultiply(node) => {
                    let right = values.pop().expect("multiply rhs sign should exist");
                    let left = values.pop().expect("multiply lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), _) | (_, Some(Sign::NoSign)) => Some(Sign::NoSign),
                        (Some(Sign::Plus), Some(Sign::Plus))
                        | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                        (Some(Sign::Plus), Some(Sign::Minus))
                        | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
            }
        }

        let result = values
            .pop()
            .expect("exact sign evaluation should produce a result");
        store_exact_sign(self, result);
        result
    }

    #[cfg(test)]
    pub(super) fn planning_msd(&self) -> Option<Option<Precision>> {
        self.cheap_bound().planning_msd()
    }

    pub(crate) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        let bound = self.cheap_bound();
        (bound.known_sign(), bound.planning_msd())
    }

    fn exact_rational(&self) -> Option<Rational> {
        // Only exact leaf nodes are exposed here. Keeping this narrow prevents
        // constructor shortcuts from accidentally forcing approximation of a
        // composite just to discover that it is not rational.
        match &*self.internal {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(r) => Some(r.clone()),
            _ => None,
        }
    }

}
