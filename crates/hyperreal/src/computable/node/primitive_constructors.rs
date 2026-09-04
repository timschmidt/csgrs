impl Computable {
    #[inline]
    fn half() -> Self {
        // atanh/log-ratio reductions multiply by 1/2 after exact symbolic
        // simplification. Keeping the half rational cached avoids rebuilding a
        // tiny exact leaf on every construction, and still delays approximation
        // to the final Computable graph.
        Self::rational(HALF_RATIONAL.clone())
    }

    pub(crate) fn internal_structural_eq(left: &Self, right: &Self) -> bool {
        enum NodeComparison<'a> {
            Decided(bool),
            One((&'a Computable, &'a Computable)),
            Two(
                (&'a Computable, &'a Computable),
                (&'a Computable, &'a Computable),
            ),
        }

        #[inline(always)]
        fn compare_nodes<'a>(left: &'a Computable, right: &'a Computable) -> NodeComparison<'a> {
            match (&left.internal.approximation, &right.internal.approximation) {
                (Approximation::One, Approximation::One) => NodeComparison::Decided(true),
                (Approximation::Int(left), Approximation::Int(right))
                | (Approximation::IntegralAtan(left), Approximation::IntegralAtan(right)) => {
                    NodeComparison::Decided(left == right)
                }
                (Approximation::Constant(left), Approximation::Constant(right)) => {
                    NodeComparison::Decided(left == right)
                }
                (Approximation::Inverse(left), Approximation::Inverse(right))
                | (Approximation::Negate(left), Approximation::Negate(right))
                | (Approximation::Square(left), Approximation::Square(right))
                | (Approximation::PrescaledExp(left), Approximation::PrescaledExp(right))
                | (Approximation::Expm1(left), Approximation::Expm1(right))
                | (Approximation::Sqrt(left), Approximation::Sqrt(right))
                | (Approximation::PrescaledLn(left), Approximation::PrescaledLn(right))
                | (Approximation::PrescaledAtan(left), Approximation::PrescaledAtan(right))
                | (Approximation::AtanDeferred(left), Approximation::AtanDeferred(right))
                | (Approximation::PrescaledAsin(left), Approximation::PrescaledAsin(right))
                | (Approximation::AsinDeferred(left), Approximation::AsinDeferred(right))
                | (Approximation::AcosPositive(left), Approximation::AcosPositive(right))
                | (Approximation::AcoshNearOne(left), Approximation::AcoshNearOne(right))
                | (Approximation::AcoshDirect(left), Approximation::AcoshDirect(right))
                | (Approximation::AsinhNearZero(left), Approximation::AsinhNearZero(right))
                | (Approximation::AsinhDirect(left), Approximation::AsinhDirect(right))
                | (Approximation::PrescaledAsinh(left), Approximation::PrescaledAsinh(right))
                | (Approximation::AtanhDirect(left), Approximation::AtanhDirect(right))
                | (Approximation::PrescaledAtanh(left), Approximation::PrescaledAtanh(right))
                | (Approximation::PrescaledCos(left), Approximation::PrescaledCos(right))
                | (Approximation::PrescaledSin(left), Approximation::PrescaledSin(right))
                | (Approximation::PrescaledTan(left), Approximation::PrescaledTan(right))
                | (Approximation::PrescaledCot(left), Approximation::PrescaledCot(right))
                | (Approximation::ErfSeries(left), Approximation::ErfSeries(right))
                | (Approximation::Erfc(left), Approximation::Erfc(right))
                | (Approximation::NormalSf(left), Approximation::NormalSf(right))
                | (Approximation::LogPnorm(left), Approximation::LogPnorm(right))
                | (Approximation::LogNormalSf(left), Approximation::LogNormalSf(right))
                | (Approximation::LogDnorm(left), Approximation::LogDnorm(right))
                | (Approximation::SincSmall(left), Approximation::SincSmall(right))
                | (Approximation::CoscSmall(left), Approximation::CoscSmall(right)) => {
                    NodeComparison::One((left, right))
                }
                (Approximation::Add(left, right), Approximation::Add(left_rhs, right_rhs))
                | (
                    Approximation::Multiply(left, right),
                    Approximation::Multiply(left_rhs, right_rhs),
                ) => {
                    if Arc::ptr_eq(&left.internal, &right.internal)
                        && Arc::ptr_eq(&left_rhs.internal, &right_rhs.internal)
                    {
                        NodeComparison::One((left, left_rhs))
                    } else {
                        NodeComparison::Two((left, left_rhs), (right, right_rhs))
                    }
                }
                (Approximation::Ratio(left), Approximation::Ratio(right))
                | (
                    Approximation::PrescaledLnRational(left),
                    Approximation::PrescaledLnRational(right),
                )
                | (Approximation::AtanRational(left), Approximation::AtanRational(right))
                | (Approximation::AsinRational(left), Approximation::AsinRational(right))
                | (
                    Approximation::AcosPositiveRational(left),
                    Approximation::AcosPositiveRational(right),
                )
                | (
                    Approximation::AcosNegativeRational(left),
                    Approximation::AcosNegativeRational(right),
                )
                | (Approximation::AsinhRational(left), Approximation::AsinhRational(right))
                | (Approximation::AtanhRational(left), Approximation::AtanhRational(right))
                | (
                    Approximation::PrescaledCosRational(left),
                    Approximation::PrescaledCosRational(right),
                )
                | (Approximation::CosLargeRational(left), Approximation::CosLargeRational(right))
                | (
                    Approximation::PrescaledCosHalfPiMinusRational(left),
                    Approximation::PrescaledCosHalfPiMinusRational(right),
                )
                | (
                    Approximation::PrescaledSinRational(left),
                    Approximation::PrescaledSinRational(right),
                )
                | (Approximation::SinLargeRational(left), Approximation::SinLargeRational(right))
                | (
                    Approximation::PrescaledSinHalfPiMinusRational(left),
                    Approximation::PrescaledSinHalfPiMinusRational(right),
                )
                | (
                    Approximation::PrescaledCotHalfPiMinusRational(left),
                    Approximation::PrescaledCotHalfPiMinusRational(right),
                )
                | (Approximation::TanLargeRational(left), Approximation::TanLargeRational(right))
                | (
                    Approximation::PrescaledTanRational(left),
                    Approximation::PrescaledTanRational(right),
                ) => NodeComparison::Decided(left == right),
                (
                    Approximation::Offset(left, left_shift),
                    Approximation::Offset(right, right_shift),
                ) if left_shift == right_shift => NodeComparison::One((left, right)),
                (
                    Approximation::NthRoot(left, left_degree),
                    Approximation::NthRoot(right, right_degree),
                ) if left_degree == right_degree => NodeComparison::One((left, right)),
                (
                    Approximation::BinaryScaledLnRational {
                        residual: left_residual,
                        shift: left_shift,
                    },
                    Approximation::BinaryScaledLnRational {
                        residual: right_residual,
                        shift: right_shift,
                    },
                ) => NodeComparison::Decided(
                    left_residual == right_residual && left_shift == right_shift,
                ),
                (
                    Approximation::NormalInterval {
                        lo: left_lo,
                        hi: left_hi,
                    },
                    Approximation::NormalInterval {
                        lo: right_lo,
                        hi: right_hi,
                    },
                ) => NodeComparison::Two((left_lo, right_lo), (left_hi, right_hi)),
                (Approximation::NormalQuantile(left), Approximation::NormalQuantile(right))
                    if left.seed == right.seed && left.seed_prec == right.seed_prec =>
                {
                    NodeComparison::One((&left.p, &right.p))
                }
                _ => NodeComparison::Decided(false),
            }
        }

        fn compare_bounded(
            left: &Computable,
            right: &Computable,
            remaining: &mut usize,
        ) -> Option<bool> {
            if Arc::ptr_eq(&left.internal, &right.internal) {
                return Some(true);
            }
            if *remaining == 0 {
                return None;
            }
            *remaining -= 1;
            match compare_nodes(left, right) {
                NodeComparison::Decided(equal) => Some(equal),
                NodeComparison::One((left, right)) => compare_bounded(left, right, remaining),
                NodeComparison::Two((left, left_rhs), (right, right_rhs)) => {
                    match compare_bounded(left, left_rhs, remaining) {
                        Some(true) => compare_bounded(right, right_rhs, remaining),
                        Some(false) => Some(false),
                        None => None,
                    }
                }
            }
        }

        // Preserve the allocation-free path for ordinary shallow comparisons.
        // A computable is an immutable DAG, not necessarily a tree, so cap that
        // walk before shared binary edges can cause exponential revisitation.
        let mut remaining = 64;
        if let Some(equal) = compare_bounded(left, right, &mut remaining) {
            return equal;
        }

        // Retain one identity pair per already-compared node and finish the
        // comparison iteratively. This also makes unusually deep graphs safe.
        let mut pending = vec![(left, right)];
        let mut compared = HashSet::new();
        while let Some((left, right)) = pending.pop() {
            if Arc::ptr_eq(&left.internal, &right.internal) {
                continue;
            }
            match compare_nodes(left, right) {
                NodeComparison::Decided(true) => continue,
                NodeComparison::Decided(false) => return false,
                children => {
                    let identity = (
                        Arc::as_ptr(&left.internal),
                        Arc::as_ptr(&right.internal),
                    );
                    if !compared.insert(identity) {
                        continue;
                    }
                    match children {
                        NodeComparison::One(pair) => pending.push(pair),
                        NodeComparison::Two(first, second) => {
                            pending.push(second);
                            pending.push(first);
                        }
                        NodeComparison::Decided(_) => unreachable!("matched above"),
                    }
                }
            }
        }
        true
    }

    fn exact_shared_perturbation_order(&self, other: &Self) -> Option<Ordering> {
        if let Approximation::Add(left, right) = &self.internal.approximation {
            if Computable::internal_structural_eq(left, other) {
                return Self::dominant_perturbation_order(left, right, other, None);
            }
            if Computable::internal_structural_eq(right, other) {
                return Self::dominant_perturbation_order(right, left, other, None);
            }
        }
        if let Approximation::Add(left, right) = &other.internal.approximation {
            if Computable::internal_structural_eq(left, self) {
                return Self::dominant_perturbation_order(left, right, self, None)
                    .map(Ordering::reverse);
            }
            if Computable::internal_structural_eq(right, self) {
                return Self::dominant_perturbation_order(right, left, self, None)
                    .map(Ordering::reverse);
            }
        }
        None
    }

    fn dominant_perturbation_order(
        base: &Self,
        perturbation: &Self,
        comparable: &Self,
        tolerance: Option<Precision>,
    ) -> Option<Ordering> {
        if !Computable::internal_structural_eq(base, comparable) {
            return None;
        }

        let (perturb_sign, perturb_msd) = perturbation.planning_sign_and_msd();
        let perturb_sign = perturb_sign?;
        if tolerance.is_some_and(|tolerance| {
            perturb_msd
                .flatten()
                .is_some_and(|msd| msd < tolerance)
        }) {
            return Some(Ordering::Equal);
        }

        // `(base + perturbation) - base` is exactly the perturbation. The
        // ordering therefore depends only on its sign; the sign or magnitude
        // of a negative base must not reverse the comparison. Exact comparison
        // passes no tolerance and can retain that proof below its refinement
        // floor; absolute-tolerance comparison may deliberately collapse a
        // smaller perturbation to `Equal` above.
        match perturb_sign {
            Sign::Minus => Some(Ordering::Less),
            Sign::NoSign => Some(Ordering::Equal),
            Sign::Plus => Some(Ordering::Greater),
        }
    }

    /// Exactly zero.
    pub fn zero() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "zero");
        Self {
            internal: Arc::new(Node::new(Approximation::Int(BigInt::zero()), BoundCache::Valid(BoundInfo::Zero), ExactSignCache::Valid(Sign::NoSign))),
            signal: None,
        }
    }

    /// Exactly one.
    pub fn one() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "one");
        Self {
            internal: Arc::new(Node::new(Approximation::One, BoundCache::Valid(BoundInfo::with_sign(Sign::Plus, Some(0))), ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Approximate π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-pi");
        Self::shared_constant(SharedConstant::Pi)
    }

    pub(crate) fn pi_inverse_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-inv-pi");
        Self::shared_constant(SharedConstant::InvPi)
    }

    pub(crate) fn atan_inv5_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-inv5");
        Self::shared_constant(SharedConstant::AtanInv5)
    }

    pub(crate) fn atan_inv2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-inv2");
        Self::shared_constant(SharedConstant::AtanInv2)
    }

    pub(crate) fn atan2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan2");
        Self::shared_constant(SharedConstant::Atan2)
    }

    pub(crate) fn atan_three_halves_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-three-halves");
        Self::shared_constant(SharedConstant::AtanThreeHalves)
    }

    /// Approximate τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-tau");
        Self::shared_constant(SharedConstant::Tau)
    }

    /// Approximate e, Euler's number and the base of the natural logarithm.
    pub fn e() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e");
        Self::e_constant()
    }

    pub(crate) fn e_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e-internal");
        Self::shared_constant(SharedConstant::E)
    }

    pub(crate) fn ln_constant(base: u32) -> Option<Computable> {
        // Common logarithms are shared constants so repeated symbolic ln forms
        // reuse one approximation cache across cloned Real values.
        crate::trace_dispatch!("computable", "constructor", "shared-log-constant-probe");
        let constant = match base {
            2 => SharedConstant::Ln2,
            3 => SharedConstant::Ln3,
            5 => SharedConstant::Ln5,
            6 => SharedConstant::Ln6,
            7 => SharedConstant::Ln7,
            10 => SharedConstant::Ln10,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    pub(crate) fn sqrt_constant(n: i64) -> Option<Computable> {
        // sqrt(2) and sqrt(3) are exact trig outputs; caching them prevents
        // fresh sqrt kernels in every sin/cos special form.
        crate::trace_dispatch!("computable", "constructor", "shared-sqrt-constant-probe");
        let constant = match n {
            2 => SharedConstant::Sqrt2,
            3 => SharedConstant::Sqrt3,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    pub(crate) fn acosh2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-acosh2");
        Self::shared_constant(SharedConstant::Acosh2)
    }

    pub(crate) fn asinh1_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-asinh1");
        Self::shared_constant(SharedConstant::Asinh1)
    }

    pub(crate) fn prescaled_sin(value: Computable) -> Computable {
        // Caller promises argument reduction has already happened. Keeping this
        // constructor private prevents large arguments from entering the Taylor
        // kernel directly.
        if let Some((orientation, argument)) = value.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!(
                "computable",
                "constructor",
                "prescaled-sin-acos-minus-half-pi"
            );
            return if orientation == Sign::Plus {
                argument.negate()
            } else {
                argument
            };
        }
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin");
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledSin(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    pub(crate) fn prescaled_cos(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin. Cosine has exact
        // zero/one shortcuts in the public constructor, so this stays a raw
        // approximation node for already-small residuals.
        if let Some((_, argument)) = value.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!(
                "computable",
                "constructor",
                "prescaled-cos-acos-minus-half-pi"
            );
            return Self::one()
                .add(argument.square().negate())
                .sqrt();
        }
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos");
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledCos(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn has_certified_small_angle(&self) -> bool {
        // approx(-2) differs from 4*x by at most one. Therefore |a| <= 2
        // certifies |x| <= 3/4. Use an explicit un-aborted evaluation because
        // this result authorizes a kernel precondition rather than merely
        // choosing an optional optimization.
        self.approx_signal(&None, -2).magnitude() <= signed::TWO.magnitude()
    }

    pub(crate) fn sinc_small_if_certified(self) -> Option<Computable> {
        if !self.has_certified_small_angle() {
            return None;
        }
        let signal = self.signal.clone();
        crate::trace_dispatch!("computable", "constructor", "certified-small-sinc");
        Some(Self {
            internal: Arc::new(Node::new(
                Approximation::SincSmall(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal,
        })
    }

    pub(crate) fn cosc_small_if_certified(self) -> Option<Computable> {
        if !self.has_certified_small_angle() {
            return None;
        }
        let signal = self.signal.clone();
        crate::trace_dispatch!("computable", "constructor", "certified-small-cosc");
        Some(Self {
            internal: Arc::new(Node::new(
                Approximation::CoscSmall(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal,
        })
    }

    fn prescaled_cos_rational(rational: Rational) -> Computable {
        // Small exact-rational cosine construction is a scalar hot path. Store
        // the rational directly so construction avoids a child Ratio node; the
        // approximation dispatcher materializes the same kernel input later if
        // digits are requested.
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos-rational");
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledCosRational(rational), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    pub(crate) fn cos_large_rational_deferred(rational: Rational) -> Computable {
        // Real::cos for large plain rationals defers the expensive half-pi
        // reduction until digits are requested. This keeps construction and
        // structural queries cheap; the approximation node then performs direct
        // residual arithmetic without allocating the generic reducer graph.
        crate::trace_dispatch!("computable", "constructor", "cos-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::CosLargeRational(rational), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn prescaled_cos_half_pi_minus_rational(rational: Rational) -> Computable {
        // sin(x) for exact medium rational x is cos(pi/2 - x). Keeping the
        // residual as one node avoids the generic Add/Offset/Negate stack in
        // the cold scalar f64 and 7/5 benchmarks.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cos-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCosHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(internal, BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    fn prescaled_sin_half_pi_minus_rational(rational: Rational) -> Computable {
        // cos(x) for exact medium rational x is sin(pi/2 - x). This mirrors the
        // cosine shortcut above and keeps common dyadic imports off the generic
        // composite residual path.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-sin-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledSinHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(internal, BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    fn prescaled_cot_half_pi_minus_rational(rational: Rational) -> Computable {
        // tan(x) near pi/2 is cot(pi/2 - x). Keeping the residual exact avoids
        // the generic complement tree and lets the approximation layer evaluate
        // the local quotient directly.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cot-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCotHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(internal, BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    pub(crate) fn sin_large_rational_deferred(rational: Rational) -> Computable {
        // Same lazy-construction policy as cos_large_rational_deferred. The
        // approximation node evaluates the direct half-pi residual itself, so
        // exact 1e6/1e30 scalar rows avoid eager reducer graph construction.
        crate::trace_dispatch!("computable", "constructor", "sin-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::SinLargeRational(rational), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    pub(crate) fn tan_large_rational_deferred(rational: Rational) -> Computable {
        // Tangent used to run through generic pi reduction even for exact large
        // rationals. Deferring it into a dedicated approximation node lets the
        // hot 1e6/1e30 rows share the direct half-pi residual used by sin/cos.
        crate::trace_dispatch!("computable", "constructor", "tan-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::TanLargeRational(rational), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    pub(crate) fn prescaled_tan(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin; tangent additionally
        // relies on the public constructor to handle near-pole complements.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan");
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledTan(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn prescaled_sin_rational(rational: Rational) -> Computable {
        // Small exact-rational sine construction mirrors cosine and preserves
        // the exact sign without allocating a child Computable.
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin-rational");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledSinRational(rational), BoundCache::Invalid, ExactSignCache::Valid(sign))),
            signal: None,
        }
    }

    fn prescaled_tan_rational(rational: Rational) -> Computable {
        // Small exact-rational tangent uses the same construction shortcut as
        // sine; sign follows the rational argument on the reduced interval.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan-rational");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledTanRational(rational), BoundCache::Invalid, ExactSignCache::Valid(sign))),
            signal: None,
        }
    }

    fn prescaled_asinh(value: Computable) -> Computable {
        // Tiny exact-rational asinh inputs use a direct odd-power series. This
        // keeps public construction cheap for scalar endpoint benches and only
        // enters the kernel after |x| has been structurally certified tiny.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asinh");
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledAsinh(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn prescaled_asin(value: Computable) -> Computable {
        // Tiny non-rational asin inputs use the direct odd-power series. This
        // mirrors prescaled atan/asinh dispatch and avoids building the generic
        // atan/sqrt transform once the argument is structurally small.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asin");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledAsin(value), BoundCache::Invalid, sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid))),
            signal: None,
        }
    }

    fn asinh_rational_deferred(rational: Rational) -> Computable {
        // Same series as `prescaled_asinh`, but exact rationals can skip the
        // child Computable wrapper and feed the kernel directly.
        crate::trace_dispatch!("computable", "constructor", "asinh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AsinhRational(rational), BoundCache::Invalid, ExactSignCache::Valid(sign))),
            signal: None,
        }
    }

    fn prescaled_atanh(value: Computable) -> Computable {
        // Tiny non-rational atanh inputs use the direct odd-power series. This
        // keeps parity with exact-rational AtanhRational and avoids the heavier
        // log-ratio graph for already-small symbolic arguments.
        crate::trace_dispatch!("computable", "constructor", "prescaled-atanh");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(Approximation::PrescaledAtanh(value), BoundCache::Invalid, sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid))),
            signal: None,
        }
    }

    fn atanh_rational_deferred(rational: Rational) -> Computable {
        // Tiny exact-rational atanh uses the odd series directly. Keeping the
        // Rational payload avoids rebuilding a Ratio node in cold approximation
        // benches while preserving the symbolic value until the final request.
        crate::trace_dispatch!("computable", "constructor", "atanh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AtanhRational(rational), BoundCache::Invalid, ExactSignCache::Valid(sign))),
            signal: None,
        }
    }

    fn acos_positive(value: Computable) -> Computable {
        // For x >= 0, acos(x) is reduced with 2*atan(sqrt((1-x)/(1+x))).
        // A single deferred node avoids allocating that whole formula during
        // public construction of endpoint-heavy inverse trig expressions.
        crate::trace_dispatch!("computable", "constructor", "acos-positive-deferred");
        Self {
            // A positive argument may still be exactly one, where acos is
            // zero. The public constructor normalizes covered exact replay
            // forms first; unsupported forms must remain Unknown rather than
            // carrying a false strictly-positive certificate.
            internal: Arc::new(Node::new(Approximation::AcosPositive(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn acos_positive_rational_deferred(rational: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-positive-rational-deferred"
        );
        Self {
            internal: Arc::new(Node::new(Approximation::AcosPositiveRational(rational), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    fn acos_negative_rational_deferred(magnitude: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-negative-rational-deferred"
        );
        Self {
            internal: Arc::new(Node::new(Approximation::AcosNegativeRational(magnitude), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    fn asin_deferred(value: Computable) -> Computable {
        // Generic asin uses a stable atan/sqrt half-angle transform. Deferring
        // that formula keeps symbolic-radical construction lightweight and
        // leaves the exact input graph intact until approximation is requested.
        crate::trace_dispatch!("computable", "constructor", "asin-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AsinDeferred(value), BoundCache::Invalid, sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid))),
            signal: None,
        }
    }

    pub(crate) fn atanh_direct_deferred(value: Computable) -> Computable {
        // Endpoint atanh uses a deferred ln-ratio node. This keeps construction
        // cheap for predicate/scalar benches while preserving the same
        // approximation identity when a numeric value is requested.
        crate::trace_dispatch!("computable", "constructor", "atanh-direct-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::AtanhDirect(value), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    pub(crate) fn acosh_near_one_deferred(value: Computable) -> Computable {
        // Near-one acosh uses a deferred ln1p/sqrt reduction. That avoids
        // building the reduction graph during scalar construction while keeping
        // the cancellation-resistant approximation path.
        crate::trace_dispatch!("computable", "constructor", "acosh-near-one-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::AcoshNearOne(value), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    pub(crate) fn acosh_direct_deferred(value: Computable) -> Computable {
        // Large acosh uses a deferred direct ln/sqrt identity so construction
        // paths do not eagerly allocate the sqrt/log graph.
        crate::trace_dispatch!("computable", "constructor", "acosh-direct-deferred");
        Self {
            internal: Arc::new(Node::new(Approximation::AcoshDirect(value), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    pub(crate) fn asinh_near_zero_deferred(value: Computable) -> Computable {
        // Moderate/tiny asinh inputs use a deferred ln1p reduction so public
        // construction stays lightweight while approximation still avoids
        // cancellation near zero.
        crate::trace_dispatch!("computable", "constructor", "asinh-near-zero-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AsinhNearZero(value), BoundCache::Invalid, sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid))),
            signal: None,
        }
    }

    pub(crate) fn asinh_direct_deferred(value: Computable) -> Computable {
        // Large asinh inputs use a deferred direct ln/sqrt identity. The caller
        // chooses this only after sign and size reduction, so no extra probing
        // is needed during construction.
        crate::trace_dispatch!("computable", "constructor", "asinh-direct-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(Approximation::AsinhDirect(value), BoundCache::Invalid, sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid))),
            signal: None,
        }
    }

    fn shared_constant(constant: SharedConstant) -> Computable {
        // Shared constants start with valid structural facts. Approximation
        // values are cached globally per thread, but the bound/sign caches can
        // be initialized directly on each lightweight wrapper.
        crate::trace_dispatch!("computable", "constructor", "shared-constant-wrapper");
        Self {
            internal: Arc::new(Node::new(Approximation::Constant(constant), BoundCache::Valid(constant.bound_info()), ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Any Rational.
    pub fn rational(r: Rational) -> Computable {
        if r.sign() == Sign::NoSign {
            // Canonicalize rational zero at construction time. This exposes
            // exact sign/zero facts immediately and avoids a Ratio leaf in the
            // many higher-level code paths that still call `rational(0)`.
            crate::trace_dispatch!("computable", "constructor", "rational-zero-canonicalized");
            return Self::zero();
        }
        if r.is_one() {
            // Route rational one through the dedicated One node so callers that
            // import binary64-derived dyadic/integer identities get the same cheap constructor
            // and structural facts as `Computable::one()`.
            crate::trace_dispatch!("computable", "constructor", "rational-one-canonicalized");
            return Self::one();
        }
        if r.is_integer() {
            crate::trace_dispatch!("computable", "constructor", "rational-integer-canonicalized");
            return Self::integer(BigInt::from_biguint(r.sign(), r.numerator().clone()));
        }
        crate::trace_dispatch!("computable", "constructor", "rational-node");
        Self {
            internal: Arc::new(Node::new(Approximation::Ratio(r), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }
}
