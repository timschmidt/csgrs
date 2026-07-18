impl SharedConstant {
    pub(super) const COUNT: usize = 18;

    pub(super) fn cache_index(self) -> usize {
        match self {
            Self::E => 0,
            Self::Pi => 1,
            Self::InvPi => 2,
            Self::Tau => 3,
            Self::Ln2 => 4,
            Self::Ln3 => 5,
            Self::Ln5 => 6,
            Self::Ln6 => 7,
            Self::Ln7 => 8,
            Self::Ln10 => 9,
            Self::Sqrt2 => 10,
            Self::Sqrt3 => 11,
            Self::Acosh2 => 12,
            Self::Asinh1 => 13,
            Self::AtanInv2 => 14,
            Self::AtanInv5 => 15,
            Self::Atan2 => 16,
            Self::AtanThreeHalves => 17,
        }
    }
}

impl Approximation {
    pub fn approximate(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        use Approximation::*;

        // This is intentionally a thin dispatcher. Algebraic simplification and
        // cache selection live in `Computable` constructors so kernels can assume
        // their documented preconditions and avoid repeated shape checks.
        match self {
            Int(i) => scale(i.clone(), -p),
            One => scale(signed::ONE.deref().clone(), -p),
            Constant(c) => c.approximate(signal, p),
            Inverse(c) => inverse(signal, c, p),
            Negate(c) => -c.approx_signal(signal, p),
            Add(c1, c2) => add(signal, c1, c2, p),
            Multiply(c1, c2) => multiply(signal, c1, c2, p),
            LinearCombination3(form) => linear_combination3(signal, form, p),
            Square(c) => square(signal, c, p),
            Ratio(r) => ratio(r, p),
            Offset(c, n) => offset(signal, c, *n, p),
            PrescaledExp(c) => exp(signal, c, p),
            Expm1(c) => expm1(signal, c, p),
            Sqrt(c) => sqrt(signal, c, p),
            PrescaledLn(c) => ln(signal, c, p),
            PrescaledLnRational(r) => ln_rational(signal, r, p),
            BinaryScaledLnRational { residual, shift } => {
                binary_scaled_ln_rational(signal, residual, *shift, p)
            }
            IntegralAtan(i) => atan(signal, i, p),
            PrescaledAtan(c) => atan_computable(signal, c, p),
            AtanRational(r) => atan_rational(signal, r, p),
            AsinRational(r) => asin_rational(signal, r, p),
            PrescaledAsin(c) => asin_computable(signal, c, p),
            AsinDeferred(c) => asin_deferred(signal, c, p),
            AcosPositive(c) => acos_positive(signal, c, p),
            AcosPositiveRational(r) => acos_positive_rational(signal, r, p),
            AcosNegativeRational(r) => acos_negative_rational(signal, r, p),
            AcoshNearOne(c) => acosh_near_one(signal, c, p),
            AcoshDirect(c) => acosh_direct(signal, c, p),
            AsinhNearZero(c) => asinh_near_zero(signal, c, p),
            AsinhDirect(c) => asinh_direct(signal, c, p),
            PrescaledAsinh(c) => asinh_computable(signal, c, p),
            AsinhRational(r) => asinh_rational(signal, r, p),
            AtanhDirect(c) => atanh_direct(signal, c, p),
            PrescaledAtanh(c) => atanh_computable(signal, c, p),
            AtanhRational(r) => atanh_rational(signal, r, p),
            PrescaledCos(c) => cos(signal, c, p),
            PrescaledCosRational(r) => cos_rational(signal, r, p),
            CosLargeRational(r) => cos_large_rational(signal, r, p),
            PrescaledCosHalfPiMinusRational(r) => cos_half_pi_minus_rational(signal, r, p),
            PrescaledSin(c) => sin(signal, c, p),
            PrescaledSinRational(r) => sin_rational(signal, r, p),
            SinLargeRational(r) => sin_large_rational(signal, r, p),
            PrescaledSinHalfPiMinusRational(r) => sin_half_pi_minus_rational(signal, r, p),
            PrescaledCotHalfPiMinusRational(r) => cot_half_pi_minus_rational(signal, r, p),
            TanLargeRational(r) => tan_large_rational(signal, r, p),
            PrescaledTan(c) => tan(signal, c, p),
            PrescaledTanRational(r) => tan_rational(signal, r, p),
            PrescaledCot(c) => cot(signal, c, p),
            ErfSeries(c) => erf_series(signal, c, p),
            Erfc(c) => erfc(signal, c, p),
            NormalSf(c) => normal_sf(signal, c, p),
            NormalInterval { lo, hi } => normal_interval(signal, lo, hi, p),
            LogPnorm(c) => log_pnorm(signal, c, p),
            LogNormalSf(c) => log_normal_sf(signal, c, p),
            LogDnorm(c) => log_dnorm(signal, c, p),
            NormalQuantile(data) => normal_quantile(
                signal,
                &data.p,
                &data.seed,
                data.seed_prec,
                p,
            ),
        }
    }
}

impl SharedConstant {
    fn approximate(self, signal: &Option<Signal>, p: Precision) -> BigInt {
        // Every shared constant routes through the same enum so cloned public
        // constants share approximation caches. Some constants are still built
        // from series identities here, but the cache prevents redoing that work
        // for repeated scalar and matrix operations.
        match self {
            Self::E => e(p),
            Self::Pi => pi(signal, p),
            Self::InvPi => inverse(signal, &Computable::pi(), p),
            Self::Tau => pi(signal, p - 1),
            Self::Ln2 => ln2(signal, p),
            Self::Ln3 => ln_constant(signal, Rational::new(3), p),
            Self::Ln5 => ln_constant(signal, Rational::new(5), p),
            Self::Ln6 => ln_constant(signal, Rational::new(6), p),
            Self::Ln7 => ln_constant(signal, Rational::new(7), p),
            Self::Ln10 => ln_constant(signal, Rational::new(10), p),
            Self::Sqrt2 => sqrt_constant(signal, Rational::new(2), p),
            Self::Sqrt3 => sqrt_constant(signal, Rational::new(3), p),
            Self::Acosh2 => acosh2_constant(signal, p),
            Self::Asinh1 => asinh1_constant(signal, p),
            Self::AtanInv2 => atan(signal, &BigInt::from(2_u8), p),
            Self::AtanInv5 => atan(signal, &BigInt::from(5_u8), p),
            Self::Atan2 => atan2_constant(signal, p),
            Self::AtanThreeHalves => atan_three_halves_constant(signal, p),
        }
    }
}

fn raw(kind: Approximation) -> Computable {
    // Build a node with no constructor-level simplification. This is used only
    // for internal constant identities where adding public simplification would
    // either recurse back into the same constant or erase the intended kernel.
    Computable {
        internal: std::sync::Arc::new(crate::computable::node::Node::new(
            kind,
            Default::default(),
            Default::default(),
        )),
        signal: None,
    }
}
