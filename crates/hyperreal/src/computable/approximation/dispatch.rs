impl SharedConstant {
    /// Number of process-wide constant-approximation slots, matching the eighteen cache indices.
    /// This bounds the number of slots, not the bytes retained by their integers.
    pub(super) const COUNT: usize = 18;

    /// Map a shared constant to its distinct dense approximation-cache slot.
    /// Keep this exhaustive mapping synchronized with COUNT and the shared table.
    /// It is an internal table key; this method does not encode serialized enum tags.
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
    /// Dispatch one retained node kind to its integer approximation kernel at precision `p`.
    ///
    /// Return units of `2^p`, retaining authored payloads and kernel selectors.
    /// Constructors own simplification; Computable's evaluation entry point owns
    /// cache lookup/publication and flattens selected linear chains before reaching
    /// this dispatcher. This method itself does not consult or populate a cache.
    ///
    /// The graph must satisfy each operation's mathematical domain. Some kernels
    /// recheck local series ranges or use intrinsically admitted payloads; dispatch
    /// alone does not validate them. Forward `signal` where the kernel accepts it,
    /// but exact leaves and e have no local poll. Abort placeholders or partial
    /// sums carry no typed status in this return value. The caller must suppress
    /// interrupted publication; no uniform precision, work or allocation bound is
    /// introduced by this match.
    pub fn approximate(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        use Approximation::*;

        // Constructors simplify graphs; the Computable evaluation entry point
        // selects caches. Kernels retain their own documented domain obligations
        // and local range checks after this dispatch.
        match self {
            Int(i) => scale_at_precision(i.clone(), p),
            One => scale_at_precision(signed::ONE.deref().clone(), p),
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
            NthRoot(c, degree) => nth_root(signal, c, *degree, p),
            PrescaledLn(c) => ln(signal, c, p),
            PrescaledLnRational(r) => ln_rational(signal, r, p),
            BinaryScaledLnRational { residual, shift } => {
                binary_scaled_ln_rational(signal, residual, *shift, p)
            }
            IntegralAtan(i) => atan(signal, i, p),
            PrescaledAtan(c) => atan_computable(signal, c, p),
            AtanDeferred(c) => atan_deferred(signal, c, p),
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
            NormalQuantile(data) => normal_quantile(signal, &data.p, &data.seed, data.seed_prec, p),
            SincSmall(c) => sinc_small(signal, c, p),
            CoscSmall(c) => cosc_small(signal, c, p),
            InverseIsometricLatitude {
                psi,
                log_axis_ratio,
            } => inverse_isometric_latitude(signal, psi, log_axis_ratio, p),
            EllipticRf(arguments) => elliptic_carlson(signal, arguments, false, p),
            EllipticRd(arguments) => elliptic_carlson(signal, arguments, true, p),
            EllipticF(arguments) => elliptic_legendre(signal, arguments, false, p),
            EllipticE(arguments) => elliptic_legendre(signal, arguments, true, p),
            JacobiAmplitude(arguments) => inverse_legendre(signal, arguments, false, p),
            InverseEllipticE(arguments) => inverse_legendre(signal, arguments, true, p),
            JacobiSn(arguments) => jacobi(signal, arguments, 0, p),
            JacobiCn(arguments) => jacobi(signal, arguments, 1, p),
            JacobiDn(arguments) => jacobi(signal, arguments, 2, p),
        }
    }
}

impl SharedConstant {
    /// Evaluate a shared constant's recipe; its process-wide cache is managed by the outer evaluator.
    ///
    /// The enum selects exact series or identities. Tau requests pi at `p - 1`;
    /// that subtraction is unchecked. Calling the pi kernel here does not itself
    /// populate the separate pi slot. Cross-constant cache reuse happens before
    /// this routine in the outer cache lookup.
    ///
    /// Pass `signal` to kernels that accept it. The e kernel has no signal and may
    /// finish after a stop, so the caller remains responsible for publication.
    /// This routine provides neither a cache lock nor a total-work/resource budget.
    fn approximate(self, signal: &Option<Signal>, p: Precision) -> BigInt {
        // The outer evaluator keys shared caches by this enum. These recipes
        // compute on a cache miss; finer requests can still need new work.
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

/// Wrap an internal recipe in a fresh node with default facts and no attached signal.
///
/// Bypass constructor recognition when forming shared log/root identities, so
/// those constructors cannot simplify back to the constant currently being
/// filled. This helper does not validate the node's domain or supply a proof;
/// its callers must establish the recipe's invariants. Evaluation later receives
/// the explicit signal. The new Arc owns the node; there is no global raw-node cache.
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
