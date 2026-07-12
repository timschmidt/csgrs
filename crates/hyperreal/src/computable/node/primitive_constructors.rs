impl Computable {
    #[inline]
    fn half() -> Self {
        // atanh/log-ratio reductions multiply by 1/2 after exact symbolic
        // simplification. Keeping the half rational cached avoids rebuilding a
        // tiny exact leaf on every construction, and still delays approximation
        // to the final Computable graph. This follows Boehm et al.'s exact-real
        // separation of symbolic construction from numerical refinement:
        // https://doi.org/10.1145/319838.319860.
        Self::rational(HALF_RATIONAL.clone())
    }

    pub(crate) fn internal_structural_eq(left: &Self, right: &Self) -> bool {
        fn compare_nodes(left: &Approximation, right: &Approximation) -> bool {
            match (left, right) {
                (Approximation::One, Approximation::One) => true,
                (Approximation::Int(left), Approximation::Int(right)) => left == right,
                (Approximation::Constant(left), Approximation::Constant(right)) => left == right,
                (Approximation::Inverse(left), Approximation::Inverse(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Negate(left), Approximation::Negate(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Add(left, right), Approximation::Add(left_rhs, right_rhs)) => {
                    Computable::internal_structural_eq(left, left_rhs)
                        && Computable::internal_structural_eq(right, right_rhs)
                }
                (
                    Approximation::Multiply(left, right),
                    Approximation::Multiply(left_rhs, right_rhs),
                ) => {
                    Computable::internal_structural_eq(left, left_rhs)
                        && Computable::internal_structural_eq(right, right_rhs)
                }
                (Approximation::Square(left), Approximation::Square(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Ratio(left), Approximation::Ratio(right)) => left == right,
                (
                    Approximation::Offset(left, left_shift),
                    Approximation::Offset(right, right_shift),
                ) => left_shift == right_shift && Computable::internal_structural_eq(left, right),
                (Approximation::PrescaledExp(left), Approximation::PrescaledExp(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Expm1(left), Approximation::Expm1(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Sqrt(left), Approximation::Sqrt(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledLn(left), Approximation::PrescaledLn(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledLnRational(left),
                    Approximation::PrescaledLnRational(right),
                ) => left == right,
                (
                    Approximation::BinaryScaledLnRational {
                        residual: left_residual,
                        shift: left_shift,
                    },
                    Approximation::BinaryScaledLnRational {
                        residual: right_residual,
                        shift: right_shift,
                    },
                ) => left_residual == right_residual && left_shift == right_shift,
                (Approximation::IntegralAtan(left), Approximation::IntegralAtan(right)) => {
                    left == right
                }
                (Approximation::PrescaledAtan(left), Approximation::PrescaledAtan(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AtanRational(left), Approximation::AtanRational(right)) => {
                    left == right
                }
                (Approximation::AsinRational(left), Approximation::AsinRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledAsin(left), Approximation::PrescaledAsin(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinDeferred(left), Approximation::AsinDeferred(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AcosPositive(left), Approximation::AcosPositive(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::AcosPositiveRational(left),
                    Approximation::AcosPositiveRational(right),
                )
                | (
                    Approximation::AcosNegativeRational(left),
                    Approximation::AcosNegativeRational(right),
                ) => left == right,
                (Approximation::AcoshNearOne(left), Approximation::AcoshNearOne(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AcoshDirect(left), Approximation::AcoshDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhNearZero(left), Approximation::AsinhNearZero(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhDirect(left), Approximation::AsinhDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledAsinh(left), Approximation::PrescaledAsinh(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhRational(left), Approximation::AsinhRational(right)) => {
                    left == right
                }
                (Approximation::AtanhDirect(left), Approximation::AtanhDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledAtanh(left), Approximation::PrescaledAtanh(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AtanhRational(left), Approximation::AtanhRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledCos(left), Approximation::PrescaledCos(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledCosRational(left),
                    Approximation::PrescaledCosRational(right),
                ) => left == right,
                (Approximation::CosLargeRational(left), Approximation::CosLargeRational(right)) => {
                    left == right
                }
                (
                    Approximation::PrescaledCosHalfPiMinusRational(left),
                    Approximation::PrescaledCosHalfPiMinusRational(right),
                ) => left == right,
                (Approximation::PrescaledSin(left), Approximation::PrescaledSin(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledSinRational(left),
                    Approximation::PrescaledSinRational(right),
                ) => left == right,
                (Approximation::SinLargeRational(left), Approximation::SinLargeRational(right)) => {
                    left == right
                }
                (
                    Approximation::PrescaledSinHalfPiMinusRational(left),
                    Approximation::PrescaledSinHalfPiMinusRational(right),
                ) => left == right,
                (
                    Approximation::PrescaledCotHalfPiMinusRational(left),
                    Approximation::PrescaledCotHalfPiMinusRational(right),
                ) => left == right,
                (Approximation::TanLargeRational(left), Approximation::TanLargeRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledTan(left), Approximation::PrescaledTan(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledTanRational(left),
                    Approximation::PrescaledTanRational(right),
                ) => left == right,
                (Approximation::PrescaledCot(left), Approximation::PrescaledCot(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::ErfSeries(left), Approximation::ErfSeries(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Erfc(left), Approximation::Erfc(right))
                | (Approximation::NormalSf(left), Approximation::NormalSf(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::NormalInterval {
                        lo: left_lo,
                        hi: left_hi,
                    },
                    Approximation::NormalInterval {
                        lo: right_lo,
                        hi: right_hi,
                    },
                ) => {
                    Computable::internal_structural_eq(left_lo, right_lo)
                        && Computable::internal_structural_eq(left_hi, right_hi)
                }
                (Approximation::LogPnorm(left), Approximation::LogPnorm(right))
                | (Approximation::LogNormalSf(left), Approximation::LogNormalSf(right))
                | (Approximation::LogDnorm(left), Approximation::LogDnorm(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::NormalQuantile {
                        p: left_p,
                        seed: left_seed,
                        seed_prec: left_seed_prec,
                    },
                    Approximation::NormalQuantile {
                        p: right_p,
                        seed: right_seed,
                        seed_prec: right_seed_prec,
                    },
                ) => {
                    left_seed == right_seed
                        && left_seed_prec == right_seed_prec
                        && Computable::internal_structural_eq(left_p, right_p)
                }
                _ => false,
            }
        }

        compare_nodes(&left.internal, &right.internal)
    }

    fn compare_absolute_dominant_perturbation(
        base: &Self,
        perturbation: &Self,
        comparable: &Self,
        tolerance: Precision,
    ) -> Option<Ordering> {
        if !Computable::internal_structural_eq(base, comparable) {
            return None;
        }

        let (base_sign, base_msd) = base.planning_sign_and_msd();
        let (perturb_sign, perturb_msd) = perturbation.planning_sign_and_msd();
        let base_sign = base_sign?;
        let perturb_sign = perturb_sign?;
        let base_msd = base_msd.flatten();
        let perturb_msd = perturb_msd.flatten();

        match (base_sign, perturb_sign) {
            (Sign::NoSign, Sign::NoSign) => Some(Ordering::Equal),
            (Sign::NoSign, _) => {
                if perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Some(Ordering::Equal)
                } else {
                    Some(Ordering::Greater)
                }
            }
            (base_sign, perturb_sign) if base_sign == perturb_sign => Some(
                if perturb_sign == Sign::NoSign || perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Ordering::Equal
                } else {
                    Ordering::Greater
                },
            ),
            (base_sign, perturb_sign) if base_sign != perturb_sign => {
                if perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Some(Ordering::Equal)
                } else if let (Some(base_msd), Some(perturb_msd)) = (base_msd, perturb_msd) {
                    if base_msd > perturb_msd {
                        Some(Ordering::Less)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Exactly zero.
    pub fn zero() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "zero");
        Self {
            internal: Box::new(Approximation::Int(BigInt::zero())),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Valid(BoundInfo::Zero)),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::NoSign)),
            signal: None,
        }
    }

    /// Exactly one.
    pub fn one() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "one");
        Self {
            internal: Box::new(Approximation::One),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Valid(BoundInfo::with_sign(Sign::Plus, Some(0)))),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin");
        Self {
            internal: Box::new(Approximation::PrescaledSin(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn prescaled_cos(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin. Cosine has exact
        // zero/one shortcuts in the public constructor, so this stays a raw
        // approximation node for already-small residuals.
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos");
        Self {
            internal: Box::new(Approximation::PrescaledCos(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_cos_rational(rational: Rational) -> Computable {
        // Small exact-rational cosine construction is a scalar hot path. Store
        // the rational directly so construction avoids a child Ratio node; the
        // approximation dispatcher materializes the same kernel input later if
        // digits are requested.
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos-rational");
        Self {
            internal: Box::new(Approximation::PrescaledCosRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(Approximation::CosLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
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
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn sin_large_rational_deferred(rational: Rational) -> Computable {
        // Same lazy-construction policy as cos_large_rational_deferred. The
        // approximation node evaluates the direct half-pi residual itself, so
        // exact 1e6/1e30 scalar rows avoid eager reducer graph construction.
        crate::trace_dispatch!("computable", "constructor", "sin-large-rational-deferred");
        Self {
            internal: Box::new(Approximation::SinLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn tan_large_rational_deferred(rational: Rational) -> Computable {
        // Tangent used to run through generic pi reduction even for exact large
        // rationals. Deferring it into a dedicated approximation node lets the
        // hot 1e6/1e30 rows share the direct half-pi residual used by sin/cos.
        crate::trace_dispatch!("computable", "constructor", "tan-large-rational-deferred");
        Self {
            internal: Box::new(Approximation::TanLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn prescaled_tan(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin; tangent additionally
        // relies on the public constructor to handle near-pole complements.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan");
        Self {
            internal: Box::new(Approximation::PrescaledTan(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_sin_rational(rational: Rational) -> Computable {
        // Small exact-rational sine construction mirrors cosine and preserves
        // the exact sign without allocating a child Computable.
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin-rational");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::PrescaledSinRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn prescaled_tan_rational(rational: Rational) -> Computable {
        // Small exact-rational tangent uses the same construction shortcut as
        // sine; sign follows the rational argument on the reduced interval.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan-rational");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::PrescaledTanRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn prescaled_asinh(value: Computable) -> Computable {
        // Tiny exact-rational asinh inputs use a direct odd-power series. This
        // keeps public construction cheap for scalar endpoint benches and only
        // enters the kernel after |x| has been structurally certified tiny.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asinh");
        Self {
            internal: Box::new(Approximation::PrescaledAsinh(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
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
            internal: Box::new(Approximation::PrescaledAsin(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    fn asinh_rational_deferred(rational: Rational) -> Computable {
        // Same series as `prescaled_asinh`, but exact rationals can skip the
        // child Computable wrapper and feed the kernel directly.
        crate::trace_dispatch!("computable", "constructor", "asinh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::AsinhRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(sign)),
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
            internal: Box::new(Approximation::PrescaledAtanh(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
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
            internal: Box::new(Approximation::AtanhRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn acos_positive(value: Computable) -> Computable {
        // For x >= 0, acos(x) is reduced with 2*atan(sqrt((1-x)/(1+x))).
        // A single deferred node avoids allocating that whole formula during
        // public construction of endpoint-heavy inverse trig expressions.
        crate::trace_dispatch!("computable", "constructor", "acos-positive-deferred");
        Self {
            internal: Box::new(Approximation::AcosPositive(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(Approximation::AcosPositiveRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(Approximation::AcosNegativeRational(magnitude)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(Approximation::AsinDeferred(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    pub(crate) fn atanh_direct_deferred(value: Computable) -> Computable {
        // Endpoint atanh uses a deferred ln-ratio node. This keeps construction
        // cheap for predicate/scalar benches while preserving the same
        // approximation identity when a numeric value is requested.
        crate::trace_dispatch!("computable", "constructor", "atanh-direct-deferred");
        Self {
            internal: Box::new(Approximation::AtanhDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn acosh_near_one_deferred(value: Computable) -> Computable {
        // Near-one acosh uses a deferred ln1p/sqrt reduction. That avoids
        // building the reduction graph during scalar construction while keeping
        // the cancellation-resistant approximation path.
        crate::trace_dispatch!("computable", "constructor", "acosh-near-one-deferred");
        Self {
            internal: Box::new(Approximation::AcoshNearOne(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn acosh_direct_deferred(value: Computable) -> Computable {
        // Large acosh uses a deferred direct ln/sqrt identity so construction
        // paths do not eagerly allocate the sqrt/log graph.
        crate::trace_dispatch!("computable", "constructor", "acosh-direct-deferred");
        Self {
            internal: Box::new(Approximation::AcoshDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            internal: Box::new(Approximation::AsinhNearZero(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
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
            internal: Box::new(Approximation::AsinhDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    fn shared_constant(constant: SharedConstant) -> Computable {
        // Shared constants start with valid structural facts. Approximation
        // values are cached globally per thread, but the bound/sign caches can
        // be initialized directly on each lightweight wrapper.
        crate::trace_dispatch!("computable", "constructor", "shared-constant-wrapper");
        Self {
            internal: Box::new(Approximation::Constant(constant)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Valid(constant.bound_info())),
            exact_sign: Cell::new(ExactSignCache::Valid(Sign::Plus)),
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
            // import exact f64/integer identities get the same cheap constructor
            // and structural facts as `Computable::one()`.
            crate::trace_dispatch!("computable", "constructor", "rational-one-canonicalized");
            return Self::one();
        }
        crate::trace_dispatch!("computable", "constructor", "rational-node");
        Self {
            internal: Box::new(Approximation::Ratio(r)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }
}
