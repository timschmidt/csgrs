#[derive(Clone)]
struct QuadraticSurd {
    rational: Rational,
    radical_scale: Rational,
    radicand: Option<Rational>,
}

impl QuadraticSurd {
    fn rational(value: Rational) -> Self {
        Self {
            rational: value,
            radical_scale: Rational::zero(),
            radicand: None,
        }
    }

    fn radical(scale: Rational, radicand: Rational) -> Self {
        if scale.sign() == Sign::NoSign {
            return Self::rational(Rational::zero());
        }
        Self {
            rational: Rational::zero(),
            radical_scale: scale,
            radicand: Some(radicand),
        }
    }

    fn normalize(mut self) -> Self {
        if self.radical_scale.sign() == Sign::NoSign {
            self.radicand = None;
        }
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let radicand = match (&self.radicand, &other.radicand) {
            (Some(left), Some(right)) if left != right => return None,
            (Some(left), _) => Some(left.clone()),
            (_, Some(right)) => Some(right.clone()),
            (None, None) => None,
        };
        Some(
            Self {
                rational: self.rational + other.rational,
                radical_scale: self.radical_scale + other.radical_scale,
                radicand,
            }
            .normalize(),
        )
    }

    fn negate(self) -> Self {
        Self {
            rational: self.rational.neg(),
            radical_scale: self.radical_scale.neg(),
            radicand: self.radicand,
        }
    }

    fn scale(self, scale: Rational) -> Self {
        Self {
            rational: self.rational * &scale,
            radical_scale: self.radical_scale * scale,
            radicand: self.radicand,
        }
        .normalize()
    }

    fn multiply(self, other: Self) -> Option<Self> {
        let radicand = match (&self.radicand, &other.radicand) {
            (Some(left), Some(right)) if left != right => return None,
            (Some(left), _) => Some(left.clone()),
            (_, Some(right)) => Some(right.clone()),
            (None, None) => None,
        };
        let radical_product = match &radicand {
            Some(radicand) => {
                &self.radical_scale * &other.radical_scale * radicand
            }
            None => Rational::zero(),
        };
        Some(
            Self {
                rational: &self.rational * &other.rational + radical_product,
                radical_scale: self.rational * other.radical_scale
                    + self.radical_scale * other.rational,
                radicand,
            }
            .normalize(),
        )
    }

    fn inverse(self) -> Option<Self> {
        let Some(radicand) = &self.radicand else {
            return self
                .rational
                .inverse()
                .ok()
                .map(Self::rational);
        };
        let denominator = &self.rational * &self.rational
            - &self.radical_scale * &self.radical_scale * radicand;
        let inverse_denominator = denominator.inverse().ok()?;
        Some(
            Self {
                rational: self.rational * &inverse_denominator,
                radical_scale: self.radical_scale.neg() * inverse_denominator,
                radicand: Some(radicand.clone()),
            }
            .normalize(),
        )
    }

    fn sign(&self) -> Sign {
        if self.radical_scale.sign() == Sign::NoSign {
            return self.rational.sign();
        }
        if self.rational.sign() == Sign::NoSign {
            return self.radical_scale.sign();
        }
        if self.rational.sign() == self.radical_scale.sign() {
            return self.rational.sign();
        }

        let radicand = self
            .radicand
            .as_ref()
            .expect("nonzero radical coefficient retains its radicand");
        let rational_square = &self.rational * &self.rational;
        let radical_square = &self.radical_scale * &self.radical_scale * radicand;
        match rational_square
            .partial_cmp(&radical_square)
            .expect("finite exact rationals are totally ordered")
        {
            Ordering::Less => self.radical_scale.sign(),
            Ordering::Equal => Sign::NoSign,
            Ordering::Greater => self.rational.sign(),
        }
    }
}

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
        match &self.internal.approximation {
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
        match &self.internal.approximation {
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

        match &self.internal.approximation {
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
        // process-wide cache; other nodes keep their cache beside the node.
        if let Approximation::Constant(constant) = &self.internal.approximation {
            return Self::cached_constant_at_precision(*constant, p);
        }

        self.internal.cached_at_precision(p)
    }

    fn cached_constant_at_precision(constant: SharedConstant, p: Precision) -> Option<BigInt> {
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
        None
    }

    fn cached_shared_constant_at_precision(
        constant: SharedConstant,
        p: Precision,
    ) -> Option<BigInt> {
        SHARED_CONSTANT_CACHES[constant.cache_index()].at_precision(p)
    }

    fn store_shared_constant_cache_value(constant: SharedConstant, p: Precision, value: BigInt) {
        SHARED_CONSTANT_CACHES[constant.cache_index()].store(p, value);
    }

    fn store_cache_value(&self, signal: &Option<Signal>, p: Precision, value: BigInt) {
        // Store only exact node approximation results, not temporary scaled
        // values. For shared constants this updates the process-wide cache so
        // every cloned constant wrapper and worker thread benefits.
        //
        // Abort-aware kernels may return an intentionally incomplete value.
        // Never publish one into shared state where an un-aborted clone could
        // later mistake it for a certified approximation.
        if should_stop(signal) {
            return;
        }
        let approximation_bound = Self::bound_from_approx(p, &value);
        if approximation_bound != BoundInfo::Unknown {
            // A separated approximation is a stronger certificate than an
            // absent or conservative Unknown result. Never replace an existing
            // structural certificate: it may carry an exact MSD that the
            // approximation error band cannot recover.
            self.internal
                .facts
                .set_bound_if_unresolved(BoundCache::Valid(approximation_bound));
        }
        if let Some(constant) = self.shared_constant_kind() {
            Self::store_shared_constant_cache_value(constant, p, value);
        } else {
            self.internal.store_cache_value(p, value);
        }
    }

    fn cached_bound(&self) -> Option<BoundInfo> {
        match self.internal.facts.bound() {
            BoundCache::Invalid => None,
            BoundCache::Valid(info) => Some(info),
        }
    }

    fn store_bound(&self, info: &BoundInfo) {
        if *info == BoundInfo::Unknown {
            // The expression is immutable, so repeating the same conservative
            // structural walk cannot improve this result. Cache Unknown only
            // while the slot is invalid; a concurrent or later separated
            // approximation atomically upgrades it through `set_bound`.
            self.internal
                .facts
                .set_bound_if_invalid(BoundCache::Valid(BoundInfo::Unknown));
        } else {
            self.internal.facts.set_bound(BoundCache::Valid(*info));
        }
    }

    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        let magnitude_bits = appr.magnitude().bits();
        if magnitude_bits <= 1 {
            BoundInfo::Unknown
        } else {
            BoundInfo::with_sign_msd(
                appr.sign(),
                Some(prec + magnitude_bits as Precision - 1),
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
        let info = match &self.internal.approximation {
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
            match &node.internal.approximation {
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

                    match &node.internal.approximation {
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
        let cached_sign = self.internal.facts.exact_sign();
        if let ExactSignCache::Valid(sign) = cached_sign {
            return Some(sign);
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            return Some(sign);
        }
        if matches!(self.internal.approximation, Approximation::Add(_, _))
            && let Some(sign) = self.inverse_trig_linear_sign()
        {
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            return Some(sign);
        }
        if cached_sign == ExactSignCache::Unknown {
            return None;
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
            let cached_sign = node.internal.facts.exact_sign();
            match cached_sign {
                ExactSignCache::Invalid => None,
                ExactSignCache::Unknown => {
                    if let Some((_, appr)) = node.cached()
                        && appr.abs() > BigInt::one()
                    {
                        let sign = appr.sign();
                        node.internal.facts.replace_exact_sign(ExactSignCache::Valid(sign));
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

            match &node.internal.approximation {
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
                Approximation::AcosPositiveRational(_)
                | Approximation::AcosNegativeRational(_)
                | Approximation::AcoshNearOne(_)
                | Approximation::AcoshDirect(_)
                | Approximation::Erfc(_)
                | Approximation::NormalSf(_) => Some(Some(Sign::Plus)),
                Approximation::AcosPositive(_) | Approximation::NormalInterval { .. } => Some(None),
                Approximation::LogPnorm(_)
                | Approximation::LogNormalSf(_)
                | Approximation::LogDnorm(_) => Some(Some(Sign::Minus)),
                Approximation::PrescaledAtan(child)
                | Approximation::AtanDeferred(child)
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
            node.internal.facts.replace_exact_sign(match sign {
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

                    match &node.internal.approximation {
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

        let mut result = values
            .pop()
            .expect("exact sign evaluation should produce a result");
        if result.is_none() {
            result = self.exact_quadratic_surd_sign();
        }
        store_exact_sign(self, result);
        result
    }

    fn exact_quadratic_surd_sign(&self) -> Option<Sign> {
        Some(self.exact_quadratic_surd()?.sign())
    }

    fn exact_quadratic_surd(&self) -> Option<QuadraticSurd> {
        const NODE_BUDGET: usize = 256;

        fn parse(
            node: &Computable,
            remaining: &mut usize,
            memo: &mut Option<Vec<(usize, QuadraticSurd)>>,
        ) -> Option<QuadraticSurd> {
            let key = Arc::as_ptr(&node.internal) as usize;
            let shared = Arc::strong_count(&node.internal) > 1;
            if shared
                && let Some((_, value)) = memo
                    .as_ref()
                    .and_then(|memo| memo.iter().find(|(candidate, _)| *candidate == key))
            {
                return Some(value.clone());
            }
            *remaining = remaining.checked_sub(1)?;
            let result = match &node.internal.approximation {
                Approximation::One => Some(QuadraticSurd::rational(Rational::one())),
                Approximation::Int(value) => {
                    Some(QuadraticSurd::rational(Rational::from_bigint(value.clone())))
                }
                Approximation::Ratio(value) => {
                    Some(QuadraticSurd::rational(value.clone()))
                }
                Approximation::Constant(SharedConstant::Sqrt2) => Some(
                    QuadraticSurd::radical(Rational::one(), Rational::new(2)),
                ),
                Approximation::Constant(SharedConstant::Sqrt3) => Some(
                    QuadraticSurd::radical(Rational::one(), Rational::new(3)),
                ),
                Approximation::Negate(child) => {
                    Some(parse(child, remaining, memo)?.negate())
                }
                Approximation::Offset(child, shift) => Some(
                    parse(child, remaining, memo)?
                        .scale(Computable::power_of_two_rational(*shift)),
                ),
                Approximation::Add(left, right) => {
                    parse(left, remaining, memo)?
                        .add(parse(right, remaining, memo)?)
                }
                Approximation::Multiply(left, right) => {
                    parse(left, remaining, memo)?
                        .multiply(parse(right, remaining, memo)?)
                }
                Approximation::Inverse(child) => {
                    parse(child, remaining, memo)?.inverse()
                }
                Approximation::Square(child) => {
                    let child = parse(child, remaining, memo)?;
                    child.clone().multiply(child)
                }
                Approximation::Sqrt(child) => {
                    let child = parse(child, remaining, memo)?;
                    if child.radical_scale.sign() != Sign::NoSign
                        || child.rational.sign() == Sign::Minus
                    {
                        return None;
                    }
                    if child.rational.sign() == Sign::NoSign {
                        return Some(QuadraticSurd::rational(Rational::zero()));
                    }
                    let (scale, radicand) = child.rational.extract_square_reduced();
                    if radicand.is_one() {
                        Some(QuadraticSurd::rational(scale))
                    } else {
                        Some(QuadraticSurd::radical(scale, radicand))
                    }
                }
                Approximation::LinearCombination3(combination) => {
                    let mut sum = QuadraticSurd::rational(Rational::zero());
                    for (coefficient, value) in combination
                        .coefficients
                        .iter()
                        .zip(combination.values.iter())
                    {
                        sum = sum.add(
                            parse(coefficient, remaining, memo)?.scale(value.clone()),
                        )?;
                    }
                    Some(sum)
                }
                _ => None,
            };
            if shared && let Some(value) = &result {
                memo.get_or_insert_with(|| Vec::with_capacity(8))
                    .push((key, value.clone()));
            }
            result
        }

        let mut remaining = NODE_BUDGET;
        // These DAGs are small and bounded, so a compact lazy vector avoids the
        // allocation and hashing overhead of a map while preserving shared nodes.
        let mut memo = None;
        let value = parse(self, &mut remaining, &mut memo)?;
        crate::trace_dispatch!("computable", "structural", "quadratic-surd");
        Some(value)
    }

    pub(crate) fn exact_pure_quadratic_surd(&self) -> Option<(Rational, Rational)> {
        let value = self.exact_quadratic_surd()?;
        if value.rational.sign() != Sign::NoSign {
            return None;
        }
        Some((value.radical_scale, value.radicand?))
    }

    pub(crate) fn exact_quadratic_surd_parts(
        &self,
    ) -> Option<(Rational, Rational, Option<Rational>)> {
        let value = self.exact_quadratic_surd()?;
        Some((value.rational, value.radical_scale, value.radicand))
    }

    #[cfg(test)]
    pub(super) fn planning_msd(&self) -> Option<Option<Precision>> {
        self.cheap_bound().planning_msd()
    }

    pub(crate) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        let bound = self.cheap_bound();
        (bound.known_sign(), bound.planning_msd())
    }

    pub(crate) fn exact_rational(&self) -> Option<Rational> {
        // Only exact leaf nodes are exposed here. Keeping this narrow prevents
        // constructor shortcuts from accidentally forcing approximation of a
        // composite just to discover that it is not rational.
        match &self.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(r) => Some(r.clone()),
            _ => None,
        }
    }

}
