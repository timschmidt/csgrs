thread_local! {
    static PNORM_CAP_HI_CACHE: Computable = Computable::integer(BigInt::from(10)).pnorm();
    static PNORM_CAP_LO_CACHE: Computable =
        Computable::one().add(Computable::integer(BigInt::from(10)).pnorm().negate());
    static SQRT_PI_OVER_TWO_CACHE: Real = (Real::pi() / Real::from(2_i32)).unwrap().sqrt().unwrap();
    static SQRT_TWO_OVER_PI_CACHE: Real = (Real::from(2_i32) / Real::pi()).unwrap().sqrt().unwrap();
    static INV_SQRT_PI_CACHE: Real = Real::pi().sqrt().unwrap().inverse().unwrap();
}

static NORMAL_QUANTILE_SAFE_LO: std::sync::LazyLock<Rational> =
    std::sync::LazyLock::new(|| {
        Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(10_u8).pow(20)).unwrap()
    });
static NORMAL_QUANTILE_SAFE_HI: std::sync::LazyLock<Rational> =
    std::sync::LazyLock::new(|| Rational::one() - NORMAL_QUANTILE_SAFE_LO.clone());

struct NormalIntervalComponents {
    mass: Real,
    phi_lo: Real,
    phi_hi: Real,
}

impl Real {
    const NORMAL_MAX_ABS: f64 = 10.0;
    const NORMAL_COMPARE_TOLERANCE: Precision = -1000;
    const NORMAL_QUANTILE_SEED_PRECISION: Precision = -24;
    const STABLE_LOG_COMPARE_TOLERANCE: Precision = -1000;

    fn pnorm_cap_hi() -> Computable {
        PNORM_CAP_HI_CACHE.with(Clone::clone)
    }

    fn pnorm_cap_lo() -> Computable {
        PNORM_CAP_LO_CACHE.with(Clone::clone)
    }

    fn normal_quantile_inside_safe_subwindow(cdf: &Self) -> bool {
        cdf.exact_rational_ref().is_some_and(|p| {
            p >= &*NORMAL_QUANTILE_SAFE_LO && p <= &*NORMAL_QUANTILE_SAFE_HI
        })
    }

    fn normal_quantile_from_seeded_cdf(
        cdf: Self,
        seed: f64,
        trace_name: &'static str,
    ) -> Result<Self, Problem> {
        let _ = trace_name;
        let known_safe_subwindow = Self::normal_quantile_inside_safe_subwindow(&cdf);
        let folded = cdf.fold();
        if !known_safe_subwindow {
            if folded.compare_absolute(&Computable::one(), Self::NORMAL_COMPARE_TOLERANCE)
                != Ordering::Less
            {
                crate::trace_dispatch!("real", trace_name, "domain-computable-one-or-more");
                return Err(Problem::NotANumber);
            }
            if folded.compare_absolute(&Self::pnorm_cap_lo(), Self::NORMAL_COMPARE_TOLERANCE)
                != Ordering::Greater
            {
                crate::trace_dispatch!("real", trace_name, "domain-low-tail-exhausted");
                return Err(Problem::Exhausted);
            }
            if folded.compare_absolute(&Self::pnorm_cap_hi(), Self::NORMAL_COMPARE_TOLERANCE)
                != Ordering::Less
            {
                crate::trace_dispatch!("real", trace_name, "domain-high-tail-exhausted");
                return Err(Problem::Exhausted);
            }
        } else {
            crate::trace_dispatch!("real", trace_name, "exact-rational-safe-subwindow");
        }

        let seed = seed.clamp(-Self::NORMAL_MAX_ABS, Self::NORMAL_MAX_ABS);
        let seed_scale = 2_f64.powi(-Self::NORMAL_QUANTILE_SEED_PRECISION);
        let seed_int = BigInt::from((seed * seed_scale).round() as i64);

        crate::trace_dispatch!("real", trace_name, "normal-quantile-computable");
        Ok(Self::irrational_from_computable(Computable::normal_quantile(
            folded,
            seed_int,
            Self::NORMAL_QUANTILE_SEED_PRECISION,
        )))
    }

    #[allow(clippy::excessive_precision)]
    fn qnorm_seed_approx(p: f64) -> f64 {
        const A: [f64; 6] = [
            -3.969683028665376e+01,
            2.209460984245205e+02,
            -2.759285104469687e+02,
            1.383577518672690e+02,
            -3.066479806614716e+01,
            2.506628277459239e+00,
        ];
        const B: [f64; 5] = [
            -5.447609879822406e+01,
            1.615858368580409e+02,
            -1.556989798598866e+02,
            6.680131188771972e+01,
            -1.328068155288572e+01,
        ];
        const C: [f64; 6] = [
            -7.784894002430293e-03,
            -3.223964580411365e-01,
            -2.400758277161838e+00,
            -2.549732539343734e+00,
            4.374664141464968e+00,
            2.938163982698783e+00,
        ];
        const D: [f64; 4] = [
            7.784695709041462e-03,
            3.224671290700398e-01,
            2.445134137142996e+00,
            3.754408661907416e+00,
        ];
        const P_LOW: f64 = 0.02425;

        if p < P_LOW {
            let q = (-2.0 * p.ln()).sqrt();
            (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
                / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
        } else if p <= 1.0 - P_LOW {
            let q = p - 0.5;
            let r = q * q;
            (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5])
                * q
                / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r
                    + 1.0)
        } else {
            let q = (-2.0 * (1.0 - p).ln()).sqrt();
            -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q
                + C[5])
                / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
        }
    }

    /// The square root of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::SqrtNegative if this Real is negative.
    pub fn sqrt(self) -> Result<Real, Problem> {
        match self.best_sign() {
            Sign::Minus => {
                crate::trace_dispatch!("real", "sqrt", "domain-negative");
                return Err(Problem::SqrtNegative);
            }
            Sign::NoSign => {
                crate::trace_dispatch!("real", "sqrt", "exact-zero");
                return Ok(Self::zero());
            }
            Sign::Plus => {}
        }
        match &self.class {
            One if self.rational.extract_square_will_succeed() => {
                // Extract rational square factors before creating sqrt nodes.
                let (square, rest) = self.rational.extract_square_reduced();
                if rest.is_one() {
                    crate::trace_dispatch!("real", "sqrt", "rational-perfect-square");
                    return Ok(Self {
                        rational: square,
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                } else if !square.is_one()
                    && let Some(shared) = rest.to_integer_i64().and_then(constants::sqrt_constant)
                {
                    // sqrt(a^2 * r) = a*sqrt(r). For scaled sqrt(2)/sqrt(3),
                    // reuse the canonical shared computable so matrix/vector
                    // clones do not rebuild the same expensive approximation.
                    // Unscaled sqrt(2)/sqrt(3) keep the local node because
                    // repeated cached approximation of one node is faster.
                    crate::trace_dispatch!("real", "sqrt", "scaled-shared-sqrt-constant");
                    return Ok(Self {
                        rational: square,
                        class: shared.class,
                        computable: shared.computable,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                } else {
                    crate::trace_dispatch!("real", "sqrt", "rational-sqrt-special-form");
                    return Ok(Self {
                        rational: square,
                        class: Sqrt(rest.clone()),
                        computable: Some(Computable::sqrt_rational(rest)),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
            }
            Pi if self.rational.extract_square_will_succeed() => {
                // If only the rational scale is a square, keep sqrt(pi) as a
                // computable sqrt rather than inventing a symbolic sqrt-pi class
                // that has not shown benchmark wins.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest.is_one() {
                    crate::trace_dispatch!("real", "sqrt", "pi-scale-computable-sqrt");
                    return Ok(Self {
                        rational: square,
                        class: Irrational,
                        computable: Some(Computable::sqrt(self.into_computable())),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
            }
            Exp(exp) if self.rational.extract_square_will_succeed() => {
                // sqrt(e^x) = e^(x/2) when the rational scale is also a square.
                // Square-free residual scales fall through to the factored
                // const-product sqrt path below.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest.is_one() {
                    let exp = exp.clone() / Rational::new(2);
                    crate::trace_dispatch!("real", "sqrt", "exp-half-special-form");
                    return Ok(Self {
                        rational: square,
                        class: Exp(exp.clone()),
                        computable: Some(Computable::exp_rational(exp)),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
            }
            _ => (),
        }
        crate::trace_dispatch!("real", "sqrt", "generic-computable");
        Ok(self.make_computable(Computable::sqrt))
    }

    /// `sqrt(1 + x) - 1`, preserving the removable cancellation near zero.
    pub fn sqrt1pm1(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sqrt1pm1", "exact-zero");
            return Ok(Self::zero());
        }

        let root = (Self::one() + self.clone()).sqrt()?;
        crate::trace_dispatch!("real", "sqrt1pm1", "rationalized");
        self / (root + Self::one())
    }

    /// `sqrt(1 - x) - 1`, preserving the removable cancellation near zero.
    pub fn sqrt1m1(self) -> Result<Real, Problem> {
        (-self).sqrt1pm1()
    }

    /// Cube root of this Real.
    pub fn cbrt(self) -> Result<Real, Problem> {
        self.root_n(3)
    }

    /// nth root of this Real.
    ///
    /// Odd roots support negative inputs by symmetry. Even roots of negative
    /// values return [`Problem::SqrtNegative`].
    pub fn root_n(self, n: u32) -> Result<Real, Problem> {
        if n == 0 {
            crate::trace_dispatch!("real", "root_n", "zero-degree-domain-error");
            return Err(Problem::NotANumber);
        }
        if n == 1 {
            crate::trace_dispatch!("real", "root_n", "degree-one");
            return Ok(self);
        }
        if let One = &self.class
            && let Some(root) = self.rational.perfect_nth_root(n)
        {
            crate::trace_dispatch!("real", "root_n", "rational-perfect-root");
            return Ok(Self::new(root));
        }

        match self.best_sign() {
            Sign::NoSign => {
                crate::trace_dispatch!("real", "root_n", "exact-zero");
                Ok(Self::zero())
            }
            Sign::Minus if n.is_multiple_of(2) => {
                crate::trace_dispatch!("real", "root_n", "domain-negative-even-root");
                Err(Problem::SqrtNegative)
            }
            Sign::Minus => {
                crate::trace_dispatch!("real", "root_n", "negative-odd-root");
                Ok(-(-self).root_n(n)?)
            }
            Sign::Plus => {
                crate::trace_dispatch!("real", "root_n", "positive-rational-exponent");
                let exponent =
                    Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(n)).unwrap();
                self.pow_fraction(exponent)
            }
        }
    }

    /// Apply the exponential function to this Real parameter.
    pub fn exp(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "exp", "exact-zero-one");
            return Ok(Self::one());
        }
        match &self.class {
            One => {
                // exp(rational) is a first-class symbolic form used heavily by exact
                // constant products.
                crate::trace_dispatch!("real", "exp", "rational-exp-special-form");
                return Ok(Self {
                    rational: Rational::one(),
                    class: Exp(self.rational.clone()),
                    computable: Some(Computable::exp_rational(self.rational)),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                });
            }
            Ln(ln) => {
                if let Some(int) = self.rational.to_big_integer() {
                    // exp(k ln n) folds to n^k when k is integral.
                    crate::trace_dispatch!("real", "exp", "integer-log-collapse");
                    return Ok(Self {
                        rational: ln.clone().powi(int)?,
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
            }
            _ => (),
        }

        crate::trace_dispatch!("real", "exp", "generic-computable");
        Ok(self.make_computable(Computable::exp))
    }

    /// The base 10 logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn log10(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "log10", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        if let One = &self.class {
            // Scalar construction benches hit exact rationals here heavily.
            // Avoid building ln(x) and then simplifying ln(x)/ln(10).
            return Self::log10_rational(self.rational);
        }
        // Use the cached ln(10) symbolic constant. Division recognizes ln/ln10
        // and can return a lightweight Log10 class for exact log inputs.
        crate::trace_dispatch!("real", "log10", "ln-div-cached-ln10");
        self.ln()? / constants::scaled_ln(10, 1).unwrap()
    }

    fn log10_rational(r: Rational) -> Result<Real, Problem> {
        match r.cmp_one_structural() {
            std::cmp::Ordering::Less => {
                let inv = r.inverse()?;
                return Ok(-Self::log10_rational(inv)?);
            }
            std::cmp::Ordering::Equal => return Ok(Self::zero()),
            std::cmp::Ordering::Greater => {}
        }

        if let Some(n) = r.integer_magnitude()
            && let Some(log) = Self::integer_log(n, 10)
        {
            crate::trace_dispatch!("real", "log10", "rational-power-of-ten");
            return Ok(Self::new(Rational::new(log as i64)));
        }

        crate::trace_dispatch!("real", "log10", "rational-log10-special-form");
        let computable =
            Class::ln_computable(&r).multiply(Class::ln_computable(&rationals::TEN).inverse());
        Ok(Self {
            rational: Rational::one(),
            class: Log10(r),
            computable: Some(computable),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        })
    }

    /// The base 2 logarithm of this Real or Problem::NotANumber if this Real is not positive.
    pub fn log2(self) -> Result<Real, Problem> {
        // Domain check uses structural sign first. Refinement-forced sign
        // (`best_sign`) is reserved for the case where cheap inspection cannot
        // decide; rejecting structurally known nonpositive inputs avoids
        // ~2µs of computable work on the typical hot path.
        match self.structural_facts().sign {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                crate::trace_dispatch!("real", "log2", "domain-not-positive");
                return Err(Problem::NotANumber);
            }
            None => {
                if self.best_sign() != Sign::Plus {
                    crate::trace_dispatch!("real", "log2", "domain-not-positive");
                    return Err(Problem::NotANumber);
                }
            }
        }
        if let One = &self.class {
            return Self::log2_rational(self.rational);
        }
        crate::trace_dispatch!("real", "log2", "ln-div-cached-ln2");
        self.ln()? / constants::scaled_ln(2, 1).unwrap()
    }

    fn log2_rational(r: Rational) -> Result<Real, Problem> {
        match r.cmp_one_structural() {
            std::cmp::Ordering::Less => {
                let inv = r.inverse()?;
                return Ok(-Self::log2_rational(inv)?);
            }
            std::cmp::Ordering::Equal => return Ok(Self::zero()),
            std::cmp::Ordering::Greater => {}
        }

        if let Some(n) = r.integer_magnitude()
            && let Some(log) = Self::integer_log(n, 2)
        {
            crate::trace_dispatch!("real", "log2", "rational-power-of-two");
            return Ok(Self::new(Rational::new(log as i64)));
        }

        crate::trace_dispatch!("real", "log2", "rational-log2-special-form");
        let computable =
            Class::ln_computable(&r).multiply(Class::ln_computable(&rationals::TWO).inverse());
        Ok(Self {
            rational: Rational::one(),
            class: Log2(r),
            computable: Some(computable),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        })
    }

    // Find Some(m) integral log with respect to this base or else None
    // n should be positive (not zero) and base should be >= 2
    fn integer_log(n: &BigUint, base: u32) -> Option<u64> {
        use num::Integer;
        // TODO weed out some large failure cases early and return None

        if let Some(mut reduced) = n.to_u64() {
            // The scalar log benches mostly use decimal-sized inputs such as
            // 1e12. For values that fit in a machine word, repeated u64
            // division is much cheaper than allocating BigUint power ladders.
            if reduced <= 1 {
                return None;
            }
            let base = u64::from(base);
            let mut exponent = 0;
            while reduced % base == 0 {
                reduced /= base;
                exponent += 1;
            }
            return if reduced == 1 && exponent > 0 {
                Some(exponent)
            } else {
                None
            };
        }

        // Build powers by repeated squaring, divide by the largest usable power,
        // then walk back down. This recognizes n = base^k without trial-dividing
        // by base k times.
        // Calculate base^2 base^4 base^8 base^16 and so on until it is bigger than next
        let mut result: Option<u64> = None;
        let mut powers: Vec<BigUint> = Vec::new();
        let mut next = BigUint::from(base);
        powers.push(next.clone());

        let mut reduced = n.clone();
        let mut i = 1;
        loop {
            // TODO Looping, may need to handle cancellation
            next = next.pow(2);
            if next.bits() > reduced.bits() {
                break;
            }

            let (div, rem) = reduced.div_rem(&next);
            if rem != BigUint::ZERO {
                return None;
            }
            powers.push(next.clone());
            result = Some(result.unwrap_or(0) + (1 << i));
            reduced = div;
            i += 1;
        }

        while let Some(power) = powers.pop() {
            if reduced == *unsigned::ONE {
                break;
            }
            i -= 1;
            if power.bits() > reduced.bits() {
                continue;
            }
            let (div, rem) = reduced.div_rem(&power);
            if rem != BigUint::ZERO {
                return None;
            }
            result = Some(result.unwrap_or(0) + (1 << i));
            reduced = div;
        }

        if reduced == *unsigned::ONE {
            result
        } else {
            None
        }
    }

    // For input y = ln(r) with r positive gives
    // Some(k ln(s)) where there is a small integer m such that r = s^k.
    // or None
    fn ln_small(r: &Rational) -> Option<Real> {
        let n = r.integer_magnitude()?;

        // Recognize common integer powers so logs share cached scaled-ln constants
        // instead of creating many unrelated Ln nodes.
        // Check base 10 first because log10/ln scalar benches include 1e12 and
        // 1e-12; probing 2, 3, 5, 6, and 7 first made those cases regress.
        for base in [10, 2, 3, 5, 6, 7] {
            if let Some(n) = Self::integer_log(n, base) {
                return constants::scaled_ln(base, n as i64);
            }
        }

        None
    }

    // Ensure the resulting Real uses r > 1 for Ln(r)
    // this is convenient elsewhere and makes commonality more frequent
    // e.g. use Ln(2) rather than Ln(0.5)
    // Must be called with r > 0
    fn ln_rational(r: Rational) -> Result<Real, Problem> {
        match r.cmp_one_structural() {
            std::cmp::Ordering::Less => {
                let inv = r.inverse()?;
                if let Some(answer) = Self::ln_small(&inv) {
                    crate::trace_dispatch!("real", "ln", "rational-inverse-shared-log");
                    return Ok(-answer);
                }
                // Normalize ln(r<1) as -ln(1/r) to improve symbolic sharing.
                let new = Computable::rational(inv.clone());
                crate::trace_dispatch!("real", "ln", "rational-inverse-ln-special-form");
                Ok(Self {
                    rational: Rational::new(-1),
                    class: Ln(inv),
                    computable: Some(Computable::ln(new)),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            std::cmp::Ordering::Equal => {
                crate::trace_dispatch!("real", "ln", "rational-one-zero");
                Ok(Self::zero())
            }
            std::cmp::Ordering::Greater => {
                if let Some(answer) = Self::ln_small(&r) {
                    crate::trace_dispatch!("real", "ln", "rational-shared-log");
                    return Ok(answer);
                }
                // Positive rationals above one get a lightweight Ln certificate.
                let new = Computable::rational(r.clone());
                crate::trace_dispatch!("real", "ln", "rational-ln-special-form");
                Ok(Self {
                    rational: Rational::one(),
                    class: Ln(r),
                    computable: Some(Computable::ln(new)),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
        }
    }

    fn try_add_rational_to_ln_term(term: &Real, offset: Rational) -> Option<Real> {
        // Normalize q + a*ln(b) as a * (q/a + ln(b)) when the inner affine log
        // is positive. If the sign certificate is not cheap, return None and let
        // ordinary addition build a generic computable sum.
        if offset == *rationals::ZERO {
            return Some(term.clone());
        }
        if term.class == One {
            return Some(Real::new(&term.rational + offset));
        }
        let Ln(base) = &term.class else {
            return None;
        };
        if term.rational.sign() == Sign::NoSign {
            return Some(Real::new(offset));
        }
        let class_offset = offset / &term.rational;
        let (class, computable) = Class::make_ln_affine(class_offset, base.clone())?;
        Some(Real {
            rational: term.rational.clone(),
            class,
            computable: Some(computable),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        })
    }

    /// The natural logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn ln(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "ln", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        match &self.class {
            One => return Self::ln_rational(self.rational),
            Exp(exp) => {
                if self.rational.is_one() {
                    // ln(e^x) collapses exactly for the pure exponential class.
                    crate::trace_dispatch!("real", "ln", "pure-exp-collapse");
                    return Ok(Self {
                        rational: exp.clone(),
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                if exp == &*rationals::ONE && self.rational == *rationals::TWO {
                    crate::trace_dispatch!("real", "ln", "cached-one-plus-ln2");
                    return Ok(constants::one_plus_ln2());
                }
                // ln(a * e^x) = ln(a) + x for positive rational scale `a`.
                // The positive-offset case is stored as one factored `ln` class
                // so repeated predicates do not traverse a generic add graph.
                let log_scale = Self::ln_rational(self.rational)?;
                if let Some(answer) = Self::try_add_rational_to_ln_term(&log_scale, exp.clone()) {
                    crate::trace_dispatch!("real", "ln", "scaled-exp-affine-log-special-form");
                    return Ok(answer);
                }
                crate::trace_dispatch!("real", "ln", "scaled-exp-log-plus-exponent");
                return Ok(log_scale + Self::new(exp.clone()));
            }
            _ => (),
        }

        crate::trace_dispatch!("real", "ln", "generic-computable");
        Ok(self.make_computable(Computable::ln))
    }

    /// Natural logarithm of `1 + x`, preserving the small-residual shape.
    pub fn ln_1p(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "ln_1p", "exact-zero");
            return Ok(Self::zero());
        }
        if matches!(&self.class, One) {
            if self.rational.sign() == Sign::Minus {
                let one_plus = Rational::one() + self.rational.clone();
                if one_plus.sign() != Sign::Plus {
                    crate::trace_dispatch!("real", "ln_1p", "domain-one-plus-not-positive");
                    return Err(Problem::NotANumber);
                }
            }
            crate::trace_dispatch!("real", "ln_1p", "exact-rational-computable");
            return Ok(Self::irrational_from_computable(
                Computable::rational(self.rational).ln_1p(),
            ));
        }
        match self.best_sign() {
            Sign::Plus => {}
            Sign::Minus | Sign::NoSign => {
                let one_plus = Self::one() + self.clone();
                if one_plus.best_sign() != Sign::Plus {
                    crate::trace_dispatch!("real", "ln_1p", "domain-one-plus-not-positive");
                    return Err(Problem::NotANumber);
                }
            }
        }

        crate::trace_dispatch!("real", "ln_1p", "generic-computable");
        Ok(self.make_computable(Computable::ln_1p))
    }

    /// Alias for [`Real::ln_1p`].
    pub fn log1p(self) -> Result<Real, Problem> {
        self.ln_1p()
    }

    /// Natural logarithm of `1 - x`, preserving the small-residual shape.
    pub fn ln_1m(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "ln_1m", "exact-zero");
            return Ok(Self::zero());
        }
        if matches!(&self.class, One) {
            if self.rational.cmp_one_structural() != Ordering::Less {
                crate::trace_dispatch!("real", "ln_1m", "domain-one-minus-not-positive");
                return Err(Problem::NotANumber);
            }
            crate::trace_dispatch!("real", "ln_1m", "exact-rational-computable");
            return Ok(Self::irrational_from_computable(
                Computable::rational(-self.rational).ln_1p(),
            ));
        }
        (-self).ln_1p()
    }

    /// Alias for [`Real::ln_1m`].
    pub fn log1m(self) -> Result<Real, Problem> {
        self.ln_1m()
    }

    /// `exp(x) - 1`, preserving the small-argument shape.
    pub fn expm1(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "expm1", "exact-zero");
            return Self::zero();
        }
        if let Some(exp_value) = Self::exact_exp_of_ln_term(&self) {
            crate::trace_dispatch!("real", "expm1", "exact-ln-term");
            return exp_value - Self::one();
        }

        crate::trace_dispatch!("real", "expm1", "generic-computable");
        self.make_computable(Computable::expm1)
    }

    fn exact_exp_of_ln_term(value: &Self) -> Option<Self> {
        let Ln(base) = &value.class else {
            return None;
        };
        let exp = value.rational.to_big_integer()?;
        base.clone().powi(exp).ok().map(Self::new)
    }

    fn ln_1p_exp_tail(delta: Self) -> Result<Self, Problem> {
        if matches!(&delta.class, One) {
            return Self::ln_1p_exp_rational_tail(delta.rational);
        }

        if delta.definitely_zero() {
            return constants::scaled_ln(2, 1).ok_or(Problem::Exhausted);
        }

        Ok(Self::irrational_from_computable(delta.fold().exp().ln_1p()))
    }

    fn ln_1p_exp_rational_tail(delta: Rational) -> Result<Self, Problem> {
        if delta.is_zero() {
            constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)
        } else {
            Ok(Self::irrational_from_computable(
                Computable::exp_rational(delta).ln_1p(),
            ))
        }
    }

    fn softplus_rational(value: Rational) -> Result<Self, Problem> {
        match value.sign() {
            Sign::NoSign => constants::scaled_ln(2, 1).ok_or(Problem::Exhausted),
            Sign::Minus => Self::ln_1p_exp_rational_tail(value),
            Sign::Plus => {
                let correction = Computable::exp_rational(-value.clone()).ln_1p();
                Ok(Self::irrational_from_computable(
                    Computable::rational(value).add(correction),
                ))
            }
        }
    }

    fn exp_tail(delta: Self) -> Result<Self, Problem> {
        if matches!(&delta.class, One) {
            Ok(Self::exp_rational_tail(delta.rational))
        } else {
            delta.exp()
        }
    }

    fn exp_rational_tail(delta: Rational) -> Self {
        Self::irrational_from_computable(Computable::exp_rational(delta))
    }

    fn sigmoid_rational(value: Rational) -> Self {
        if value.sign() == Sign::Plus {
            let tail = Computable::exp_rational(-value);
            Self::irrational_from_computable(Computable::one().add(tail).inverse())
        } else {
            let tail = Computable::exp_rational(value);
            let denominator = Computable::one().add(tail.clone()).inverse();
            Self::irrational_from_computable(tail.multiply(denominator))
        }
    }

    fn rational_plus_ln_1p_exp_tail(value: Rational, delta: Rational) -> Result<Self, Problem> {
        if delta.is_zero() {
            Ok(Self::new(value) + constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?)
        } else {
            Ok(Self::irrational_from_computable(
                Computable::rational(value).add(Computable::exp_rational(delta).ln_1p()),
            ))
        }
    }

    fn rational_plus_ln_1m_exp_tail(value: Rational, delta: Rational) -> Self {
        Self::irrational_from_computable(
            Computable::rational(value)
                .add(Computable::exp_rational(delta).negate().ln_1p()),
        )
    }

    /// `ln(1 + exp(x))`, using the smaller exponential side when the sign is known.
    pub fn softplus(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "softplus", "exact-zero-ln2");
            return constants::scaled_ln(2, 1).ok_or(Problem::Exhausted);
        }
        if let Some(exp_value) = Self::exact_exp_of_ln_term(&self) {
            crate::trace_dispatch!("real", "softplus", "exact-ln-term");
            return (Self::one() + exp_value).ln();
        }
        if matches!(&self.class, One) {
            crate::trace_dispatch!("real", "softplus", "exact-rational-tail");
            return Self::softplus_rational(self.rational);
        }

        match self.best_sign() {
            Sign::NoSign => constants::scaled_ln(2, 1).ok_or(Problem::Exhausted),
            Sign::Minus => {
                crate::trace_dispatch!("real", "softplus", "negative-ln1p-exp");
                Self::ln_1p_exp_tail(self)
            }
            Sign::Plus => {
                crate::trace_dispatch!("real", "softplus", "positive-max-plus-ln1p-tail");
                let correction = Self::ln_1p_exp_tail(-self.clone())?;
                Ok(self + correction)
            }
        }
    }

    /// `ln(p / (1 - p))` for `0 < p < 1`.
    pub fn logit(self) -> Result<Real, Problem> {
        if self == constants::half() {
            crate::trace_dispatch!("real", "logit", "exact-half-zero");
            return Ok(Self::zero());
        }
        if matches!(&self.class, One) {
            if self.rational.sign() != Sign::Plus {
                crate::trace_dispatch!("real", "logit", "domain-not-positive");
                return Err(Problem::NotANumber);
            }
            if self.rational.cmp_one_structural() != Ordering::Less {
                crate::trace_dispatch!("real", "logit", "domain-not-below-one");
                return Err(Problem::NotANumber);
            }
            crate::trace_dispatch!("real", "logit", "exact-rational-stable-logs");
            let log_p = Self::ln_rational(self.rational.clone())?;
            let log_one_minus_p = Self::irrational_from_computable(
                Computable::rational(-self.rational).ln_1p(),
            );
            return Ok(log_p - log_one_minus_p);
        }
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "logit", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        if (Self::one() - self.clone()).best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "logit", "domain-not-below-one");
            return Err(Problem::NotANumber);
        }

        let log_p = self.clone().ln()?;
        let log_one_minus_p = self.ln_1m()?;
        Ok(log_p - log_one_minus_p)
    }

    /// Logistic sigmoid, `1 / (1 + exp(-x))`.
    pub fn sigmoid(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sigmoid", "exact-zero-half");
            return Ok(constants::half());
        }
        if let Some(exp_value) = Self::exact_exp_of_ln_term(&self) {
            crate::trace_dispatch!("real", "sigmoid", "exact-ln-term");
            return exp_value.clone() / (Self::one() + exp_value);
        }
        if matches!(&self.class, One) {
            crate::trace_dispatch!("real", "sigmoid", "exact-rational-tail");
            return Ok(Self::sigmoid_rational(self.rational));
        }

        match self.best_sign() {
            Sign::NoSign => Ok(constants::half()),
            Sign::Plus => {
                crate::trace_dispatch!("real", "sigmoid", "positive-tail");
                let tail = Self::exp_tail(-self)?;
                Self::one() / (Self::one() + tail)
            }
            Sign::Minus => {
                crate::trace_dispatch!("real", "sigmoid", "negative-tail");
                let tail = Self::exp_tail(self)?;
                tail.clone() / (Self::one() + tail)
            }
        }
    }

    /// `ln(exp(a) + exp(b))`, preserving the dominant log-space term when known.
    pub fn logaddexp(a: &Real, b: &Real) -> Result<Real, Problem> {
        if a == b {
            crate::trace_dispatch!("real", "logaddexp", "structural-equal-ln2");
            return Ok(a + constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?);
        }
        if let (Some(exp_a), Some(exp_b)) =
            (Self::exact_exp_of_ln_term(a), Self::exact_exp_of_ln_term(b))
        {
            crate::trace_dispatch!("real", "logaddexp", "exact-ln-terms");
            return (exp_a + exp_b).ln();
        }

        if matches!((&a.class, &b.class), (One, One)) {
            return match a.rational.partial_cmp(&b.rational) {
                Some(Ordering::Equal) => {
                    crate::trace_dispatch!("real", "logaddexp", "equal-ln2");
                    Ok(a + constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?)
                }
                Some(Ordering::Greater) => {
                    crate::trace_dispatch!("real", "logaddexp", "left-dominant");
                    Self::rational_plus_ln_1p_exp_tail(
                        a.rational.clone(),
                        &b.rational - &a.rational,
                    )
                }
                Some(Ordering::Less) => {
                    crate::trace_dispatch!("real", "logaddexp", "right-dominant");
                    Self::rational_plus_ln_1p_exp_tail(
                        b.rational.clone(),
                        &a.rational - &b.rational,
                    )
                }
                None => unreachable!("Rational values have total ordering"),
            };
        }

        let ordering = a.certified_cmp_until(b, Self::STABLE_LOG_COMPARE_TOLERANCE);

        match ordering {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Equal,
                ..
            } => {
                crate::trace_dispatch!("real", "logaddexp", "equal-ln2");
                Ok(a + constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?)
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                ..
            } => {
                crate::trace_dispatch!("real", "logaddexp", "left-dominant");
                let correction = Self::ln_1p_exp_tail(b - a)?;
                Ok(a + correction)
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                ..
            } => {
                crate::trace_dispatch!("real", "logaddexp", "right-dominant");
                let correction = Self::ln_1p_exp_tail(a - b)?;
                Ok(b + correction)
            }
            CertifiedRealOrdering::Unknown { .. } => {
                crate::trace_dispatch!("real", "logaddexp", "unknown-order-fallback");
                let left = a.clone().exp()?;
                let right = b.clone().exp()?;
                (left + right).ln()
            }
        }
    }

    /// `ln(exp(a) - exp(b))`, defined for `a > b`.
    pub fn logsubexp(a: &Real, b: &Real) -> Result<Real, Problem> {
        if let (Some(exp_a), Some(exp_b)) =
            (Self::exact_exp_of_ln_term(a), Self::exact_exp_of_ln_term(b))
        {
            crate::trace_dispatch!("real", "logsubexp", "exact-ln-terms");
            return (exp_a - exp_b).ln();
        }

        if matches!((&a.class, &b.class), (One, One)) {
            return match a.rational.partial_cmp(&b.rational) {
                Some(Ordering::Greater) => {
                    crate::trace_dispatch!("real", "logsubexp", "left-dominant");
                    Ok(Self::rational_plus_ln_1m_exp_tail(
                        a.rational.clone(),
                        &b.rational - &a.rational,
                    ))
                }
                Some(_) => {
                    crate::trace_dispatch!("real", "logsubexp", "domain-not-left-greater");
                    Err(Problem::NotANumber)
                }
                None => unreachable!("Rational values have total ordering"),
            };
        }

        let ordering = a.certified_cmp_until(b, Self::STABLE_LOG_COMPARE_TOLERANCE);

        match ordering {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                ..
            } => {
                crate::trace_dispatch!("real", "logsubexp", "left-dominant");
                let tail = Self::exp_tail(b - a)?;
                let correction = (-tail).make_computable(Computable::ln_1p);
                Ok(a + correction)
            }
            CertifiedRealOrdering::Known { .. } | CertifiedRealOrdering::Unknown { .. } => {
                crate::trace_dispatch!("real", "logsubexp", "domain-not-left-greater");
                Err(Problem::NotANumber)
            }
        }
    }

    /// The error function, erf(x).
    pub fn erf(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "erf", "exact-zero");
            return Self::zero();
        }

        crate::trace_dispatch!("real", "erf", "generic-computable");
        self.make_computable(Computable::erf)
    }

    /// Complementary error function, erfc(x) = 1 - erf(x).
    pub fn erfc(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "erfc", "exact-zero-one");
            return Self::one();
        }

        crate::trace_dispatch!("real", "erfc", "generic-computable");
        self.make_computable(Computable::erfc)
    }

    /// Scaled complementary error function, erfcx(x) = exp(x^2) * erfc(x).
    pub fn erfcx(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "erfcx", "exact-zero-one");
            return Ok(Self::one());
        }

        crate::trace_dispatch!("real", "erfcx", "generic-computable");
        Ok(self.make_computable(Computable::erfcx))
    }

    /// Standard normal density.
    pub fn dnorm(self) -> Result<Real, Problem> {
        let x: f64 = self.clone().into();
        if !x.is_finite() || x.abs() > Self::NORMAL_MAX_ABS {
            crate::trace_dispatch!("real", "dnorm", "domain-exhausted");
            return Err(Problem::Exhausted);
        }

        crate::trace_dispatch!("real", "dnorm", "generic-computable");
        Ok(self.make_computable(Computable::dnorm))
    }

    /// Standard normal cumulative distribution function.
    pub fn pnorm(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "pnorm", "exact-zero-half");
            return Ok(constants::half());
        }
        let x: f64 = self.clone().into();
        if !x.is_finite() || x.abs() > Self::NORMAL_MAX_ABS {
            crate::trace_dispatch!("real", "pnorm", "domain-exhausted");
            return Err(Problem::Exhausted);
        }

        crate::trace_dispatch!("real", "pnorm", "generic-computable");
        Ok(self.make_computable(Computable::pnorm))
    }

    /// Standard normal upper-tail probability, 1 - pnorm(x).
    pub fn normal_sf(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "normal_sf", "exact-zero-half");
            return Ok(constants::half());
        }
        let x: f64 = self.clone().into();
        if !x.is_finite() || x.abs() > Self::NORMAL_MAX_ABS {
            crate::trace_dispatch!("real", "normal_sf", "domain-exhausted");
            return Err(Problem::Exhausted);
        }

        crate::trace_dispatch!("real", "normal_sf", "generic-computable");
        Ok(self.make_computable(Computable::normal_sf))
    }

    /// Alias for [`Real::normal_sf`].
    pub fn pnorm_upper(self) -> Result<Real, Problem> {
        self.normal_sf()
    }

    /// Natural logarithm of the standard normal CDF.
    pub fn log_pnorm(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "log_pnorm", "exact-zero-minus-ln2");
            return Ok(-constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?);
        }
        Self::check_normal_window(&self, "log_pnorm")?;

        crate::trace_dispatch!("real", "log_pnorm", "generic-computable");
        Ok(self.make_computable(Computable::log_pnorm))
    }

    /// Natural logarithm of the standard normal upper-tail probability.
    pub fn log_normal_sf(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "log_normal_sf", "exact-zero-minus-ln2");
            return Ok(-constants::scaled_ln(2, 1).ok_or(Problem::Exhausted)?);
        }
        Self::check_normal_window(&self, "log_normal_sf")?;

        crate::trace_dispatch!("real", "log_normal_sf", "generic-computable");
        Ok(self.make_computable(Computable::log_normal_sf))
    }

    /// Natural logarithm of the standard normal density.
    pub fn log_dnorm(self) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "log_dnorm", "analytic-computable");
        Ok(self.make_computable(Computable::log_dnorm))
    }

    fn check_normal_window(value: &Self, _name: &'static str) -> Result<(), Problem> {
        let x: f64 = value.clone().into();
        if !x.is_finite() || x.abs() > Self::NORMAL_MAX_ABS {
            crate::trace_dispatch!("real", _name, "domain-exhausted");
            return Err(Problem::Exhausted);
        }
        Ok(())
    }

    /// Standard normal probability mass over [lo, hi].
    pub fn normal_interval(lo: &Self, hi: &Self) -> Result<Real, Problem> {
        if let Some(value) = Self::normal_interval_degenerate_or_invalid(lo, hi)? {
            return Ok(value);
        }

        Self::check_normal_window(lo, "normal_interval")?;
        Self::check_normal_window(hi, "normal_interval")?;

        crate::trace_dispatch!("real", "normal_interval", "generic-computable");
        Ok(Self::irrational_from_computable(Computable::normal_interval(
            lo.clone().fold(),
            hi.clone().fold(),
        )))
    }

    fn normal_interval_degenerate_or_invalid(
        lo: &Self,
        hi: &Self,
    ) -> Result<Option<Real>, Problem> {
        match lo.certified_cmp_until(hi, Self::NORMAL_COMPARE_TOLERANCE) {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Equal,
                ..
            } => {
                crate::trace_dispatch!("real", "normal_interval", "exact-equal-zero");
                Ok(Some(Self::zero()))
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                ..
            } => {
                crate::trace_dispatch!("real", "normal_interval", "domain-reversed-bounds");
                Err(Problem::NotANumber)
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                ..
            } => Ok(None),
            CertifiedRealOrdering::Unknown { .. } => {
                crate::trace_dispatch!("real", "normal_interval", "domain-unresolved-bounds");
                Err(Problem::NotANumber)
            }
        }
    }

    /// Alias for [`Real::normal_interval`].
    pub fn pnorm_diff(lo: &Self, hi: &Self) -> Result<Real, Problem> {
        Self::normal_interval(lo, hi)
    }

    fn sqrt_two() -> Real {
        constants::sqrt_constant(2).unwrap_or_else(|| {
            Real::new(rationals::TWO.clone())
                .sqrt()
                .expect("sqrt(2) should be defined")
        })
    }

    /// Inverse error function.
    pub fn erfinv(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "erfinv", "exact-zero");
            return Ok(Self::zero());
        }
        if self.best_sign() == Sign::Minus {
            crate::trace_dispatch!("real", "erfinv", "odd-symmetry");
            return Ok(-(-self).erfinv()?);
        }
        if self
            .clone()
            .fold()
            .compare_absolute(&Computable::one(), Self::NORMAL_COMPARE_TOLERANCE)
            != Ordering::Less
        {
            crate::trace_dispatch!("real", "erfinv", "domain-outside-open-unit");
            return Err(Problem::NotANumber);
        }

        if self.exact_rational().is_some() {
            crate::trace_dispatch!("real", "erfinv", "exact-rational-upper-tail-transform");
            let upper_tail = (Self::one() - self) * constants::half();
            return upper_tail.qnorm_upper()? / Self::sqrt_two();
        }

        crate::trace_dispatch!("real", "erfinv", "qnorm-transform");
        let p = (self + Self::one()) * constants::half();
        p.qnorm()? / Self::sqrt_two()
    }

    /// Inverse complementary error function.
    pub fn erfcinv(self) -> Result<Real, Problem> {
        if self.class == One && self.rational == *rationals::ONE {
            crate::trace_dispatch!("real", "erfcinv", "exact-one-zero");
            return Ok(Self::zero());
        }
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "erfcinv", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        match self.certified_cmp_until(&Self::from(2_i32), Self::NORMAL_COMPARE_TOLERANCE) {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                ..
            } => {}
            CertifiedRealOrdering::Known { .. } => {
                crate::trace_dispatch!("real", "erfcinv", "domain-two-or-more");
                return Err(Problem::NotANumber);
            }
            CertifiedRealOrdering::Unknown { .. } => {
                crate::trace_dispatch!("real", "erfcinv", "domain-unresolved-upper-bound");
                return Err(Problem::NotANumber);
            }
        }

        match self.certified_cmp_until(&Self::one(), Self::NORMAL_COMPARE_TOLERANCE) {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                ..
            } if self.exact_rational().is_some() => {
                crate::trace_dispatch!("real", "erfcinv", "complement-symmetry");
                return Ok(-(Self::from(2_i32) - self).erfcinv()?);
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                ..
            } if self.exact_rational().is_some() => {
                crate::trace_dispatch!("real", "erfcinv", "upper-tail-transform");
                let upper_tail = self * constants::half();
                return upper_tail.qnorm_upper()? / Self::sqrt_two();
            }
            CertifiedRealOrdering::Known {
                ordering: Ordering::Equal,
                ..
            } => {
                crate::trace_dispatch!("real", "erfcinv", "exact-one-zero");
                return Ok(Self::zero());
            }
            CertifiedRealOrdering::Known { .. } => {}
            CertifiedRealOrdering::Unknown { .. } => {}
        }

        crate::trace_dispatch!("real", "erfcinv", "fallback-qnorm-transform");
        let p = self * constants::half();
        (-p.qnorm()?) / Self::sqrt_two()
    }

    /// Inverse standard normal upper-tail probability.
    pub fn qnorm_upper(self) -> Result<Real, Problem> {
        if self.class == One && self.rational <= *rationals::ZERO {
            crate::trace_dispatch!("real", "qnorm_upper", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        if self.class == One && self.rational == *rationals::HALF {
            crate::trace_dispatch!("real", "qnorm_upper", "exact-half-zero");
            return Ok(Self::zero());
        }
        if self.class == One && self.rational >= *rationals::ONE {
            crate::trace_dispatch!("real", "qnorm_upper", "domain-one-or-more");
            return Err(Problem::NotANumber);
        }

        let cdf = Self::one() - self;
        cdf.qnorm()
    }

    fn check_normal_sigma(sigma: &Self, _name: &'static str) -> Result<(), Problem> {
        if sigma.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", _name, "domain-nonpositive-sigma");
            return Err(Problem::NotANumber);
        }
        Ok(())
    }

    fn standardize_normal_arg(
        self,
        mean: &Self,
        sigma: &Self,
        name: &'static str,
    ) -> Result<Real, Problem> {
        Self::check_normal_sigma(sigma, name)?;
        (self - mean.clone()) / sigma.clone()
    }

    /// Normal density with the given mean and positive standard deviation.
    pub fn normal_pdf(self, mean: &Self, sigma: &Self) -> Result<Real, Problem> {
        let z = self.standardize_normal_arg(mean, sigma, "normal_pdf")?;
        z.dnorm()? / sigma.clone()
    }

    /// Normal cumulative distribution with the given mean and positive standard deviation.
    pub fn normal_cdf(self, mean: &Self, sigma: &Self) -> Result<Real, Problem> {
        self.standardize_normal_arg(mean, sigma, "normal_cdf")?
            .pnorm()
    }

    /// Normal upper-tail probability with the given mean and positive standard deviation.
    pub fn normal_survival(self, mean: &Self, sigma: &Self) -> Result<Real, Problem> {
        self.standardize_normal_arg(mean, sigma, "normal_survival")?
            .normal_sf()
    }

    /// Normal quantile with the given mean and positive standard deviation.
    pub fn normal_quantile(self, mean: &Self, sigma: &Self) -> Result<Real, Problem> {
        Self::check_normal_sigma(sigma, "normal_quantile")?;
        Ok(mean.clone() + sigma.clone() * self.qnorm()?)
    }

    fn sqrt_pi_over_two() -> Result<Real, Problem> {
        Ok(SQRT_PI_OVER_TWO_CACHE.with(Clone::clone))
    }

    fn sqrt_two_over_pi() -> Result<Real, Problem> {
        Ok(SQRT_TWO_OVER_PI_CACHE.with(Clone::clone))
    }

    fn inv_sqrt_pi() -> Result<Real, Problem> {
        Ok(INV_SQRT_PI_CACHE.with(Clone::clone))
    }

    /// Upper-tail Mills ratio, normal_sf(x) / dnorm(x).
    pub fn normal_mills(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "normal_mills", "exact-zero-sqrt-pi-over-two");
            return Self::sqrt_pi_over_two();
        }
        let z = (self / Self::sqrt_two())?;
        Ok(Self::sqrt_pi_over_two()? * z.erfcx()?)
    }

    /// Standard normal hazard rate, dnorm(x) / normal_sf(x).
    pub fn normal_hazard(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "normal_hazard", "exact-zero-sqrt-two-over-pi");
            return Self::sqrt_two_over_pi();
        }
        let z = (self / Self::sqrt_two())?;
        Self::sqrt_two_over_pi()? / z.erfcx()?
    }

    /// Natural logarithm of the standard normal hazard rate.
    pub fn normal_log_hazard(self) -> Result<Real, Problem> {
        Ok(self.clone().log_dnorm()? - self.log_normal_sf()?)
    }

    /// Lower-tail inverse Mills ratio, dnorm(x) / pnorm(x).
    pub fn normal_inverse_mills(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "normal_inverse_mills",
                "exact-zero-sqrt-two-over-pi"
            );
            return Self::sqrt_two_over_pi();
        }
        self.clone().dnorm()? / self.pnorm()?
    }

    /// Probabilists' Hermite polynomial He_n(x).
    pub fn hermite_probabilists(n: usize, x: &Self) -> Real {
        if n == 0 {
            return Self::one();
        }
        if n == 1 {
            return x.clone();
        }

        let mut prev = Self::one();
        let mut current = x.clone();
        for k in 1..n {
            let coefficient = Self::new(Rational::from_bigint(BigInt::from(k)));
            let next = x.clone() * current.clone() - coefficient * prev;
            prev = current;
            current = next;
        }
        current
    }

    /// nth derivative of the standard normal density.
    pub fn dnorm_derivative(self, n: usize) -> Result<Real, Problem> {
        let polynomial = Self::hermite_probabilists(n, &self);
        let derivative = polynomial * self.dnorm()?;
        if n.is_multiple_of(2) {
            Ok(derivative)
        } else {
            Ok(-derivative)
        }
    }

    /// Alias for [`Real::dnorm_derivative`].
    pub fn gaussian_derivative(self, n: usize) -> Result<Real, Problem> {
        self.dnorm_derivative(n)
    }

    /// Raw moment of a standard normal random variable.
    pub fn standard_normal_moment(n: usize) -> Real {
        if n % 2 == 1 {
            return Self::zero();
        }

        let mut moment = BigInt::from(1_u8);
        for k in 1..=n / 2 {
            moment *= BigInt::from(2 * k - 1);
        }
        Self::new(Rational::from_bigint(moment))
    }

    /// Raw unnormalized moment over a standard normal interval.
    pub fn normal_interval_moment(lo: &Self, hi: &Self, n: usize) -> Result<Real, Problem> {
        if n == 0 {
            return Self::normal_interval(lo, hi);
        }
        if let Some(value) = Self::normal_interval_degenerate_or_invalid(lo, hi)? {
            return Ok(value);
        }
        if n == 1 {
            crate::trace_dispatch!("real", "normal_interval_moment", "closed-form-first");
            let phi_lo = lo.clone().dnorm()?;
            let phi_hi = hi.clone().dnorm()?;
            return Ok(phi_lo - phi_hi);
        }

        if n == 3 {
            crate::trace_dispatch!("real", "normal_interval_moment", "closed-form-third");
            let phi_lo = lo.clone().dnorm()?;
            let phi_hi = hi.clone().dnorm()?;
            let first = phi_lo.clone() - phi_hi.clone();
            let lo_boundary = Self::normal_boundary_power_density(lo, &phi_lo, 2)?;
            let hi_boundary = Self::normal_boundary_power_density(hi, &phi_hi, 2)?;
            return Ok(lo_boundary - hi_boundary + Self::from(2_i32) * first);
        }

        let i0 = Self::normal_interval(lo, hi)?;
        if i0.definitely_zero() {
            return Ok(Self::zero());
        }

        let components = Self::normal_interval_components_from_mass(lo, hi, i0)?;
        let phi_lo = components.phi_lo;
        let phi_hi = components.phi_hi;
        let first = phi_lo.clone() - phi_hi.clone();
        if n == 2 {
            crate::trace_dispatch!("real", "normal_interval_moment", "closed-form-second");
            let lo_boundary = Self::normal_boundary_power_density(lo, &phi_lo, 1)?;
            let hi_boundary = Self::normal_boundary_power_density(hi, &phi_hi, 1)?;
            return Ok(lo_boundary - hi_boundary + components.mass);
        }

        let mut moments = vec![components.mass, first];
        for k in 2..=n {
            let exp = BigInt::from(k - 1);
            let coefficient = Self::new(Rational::from_bigint(BigInt::from(k - 1)));
            let boundary =
                lo.clone().powi(exp.clone())? * phi_lo.clone() - hi.clone().powi(exp)? * phi_hi.clone();
            moments.push(boundary + coefficient * moments[k - 2].clone());
        }
        Ok(moments[n].clone())
    }

    fn normal_boundary_power_density(
        point: &Self,
        density: &Self,
        power: usize,
    ) -> Result<Self, Problem> {
        if power == 0 {
            return Ok(density.clone());
        }
        if matches!(point.class, One) {
            if point.rational.is_zero() {
                return Ok(Self::zero());
            }
            if point.rational.is_one() {
                return Ok(density.clone());
            }
        }
        if power == 1 {
            return Ok(point * density);
        }
        Ok(point.clone().powi(BigInt::from(power))? * density)
    }

    fn normal_interval_components_from_mass(
        lo: &Self,
        hi: &Self,
        mass: Self,
    ) -> Result<NormalIntervalComponents, Problem> {
        Ok(NormalIntervalComponents {
            mass,
            phi_lo: lo.clone().dnorm()?,
            phi_hi: hi.clone().dnorm()?,
        })
    }

    fn normal_interval_components(lo: &Self, hi: &Self) -> Result<NormalIntervalComponents, Problem> {
        let mass = Self::normal_interval(lo, hi)?;
        Self::normal_interval_components_from_mass(lo, hi, mass)
    }

    fn nondegenerate_normal_interval_components(
        lo: &Self,
        hi: &Self,
    ) -> Result<NormalIntervalComponents, Problem> {
        let components = Self::normal_interval_components(lo, hi)?;
        if components.mass.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        Ok(components)
    }

    /// Mean of a standard normal truncated to [lo, hi].
    pub fn truncated_normal_mean(lo: &Self, hi: &Self) -> Result<Real, Problem> {
        let components = Self::nondegenerate_normal_interval_components(lo, hi)?;
        let numerator = components.phi_lo - components.phi_hi;
        Ok(Self::irrational_from_computable(
            numerator.fold().multiply(components.mass.fold().inverse()),
        ))
    }

    /// Variance of a standard normal truncated to [lo, hi].
    pub fn truncated_normal_variance(lo: &Self, hi: &Self) -> Result<Real, Problem> {
        let components = Self::nondegenerate_normal_interval_components(lo, hi)?;
        let inv_mass = components.mass.clone().fold().inverse();
        let mean_numerator = components.phi_lo.clone() - components.phi_hi.clone();
        let second_numerator =
            lo.clone() * components.phi_lo - hi.clone() * components.phi_hi + components.mass;
        let mean = mean_numerator.fold().multiply(inv_mass.clone());
        let second = second_numerator.fold().multiply(inv_mass);
        Ok(Self::irrational_from_computable(
            second.add(mean.square().negate()),
        ))
    }

    fn exact_half_integer_twice(value: &Self) -> Result<i64, Problem> {
        let Some(rational) = value.exact_rational() else {
            return Err(Problem::NotANumber);
        };
        let twice = rational * Rational::new(2);
        let Some(integer) = twice.to_big_integer() else {
            return Err(Problem::NotANumber);
        };
        integer.to_i64().ok_or(Problem::NotANumber)
    }

    fn exact_positive_half_integer_twice(value: &Self) -> Result<u64, Problem> {
        let integer = Self::exact_half_integer_twice(value)?;
        if integer <= 0 {
            return Err(Problem::NotANumber);
        }
        integer.to_u64().ok_or(Problem::NotANumber)
    }

    fn exact_positive_integer(value: &Self) -> Result<u64, Problem> {
        let Some(rational) = value.exact_rational() else {
            return Err(Problem::NotANumber);
        };
        let Some(integer) = rational.to_big_integer() else {
            return Err(Problem::NotANumber);
        };
        if integer.sign() != Sign::Plus {
            return Err(Problem::NotANumber);
        }
        integer.to_u64().ok_or(Problem::NotANumber)
    }

    fn factorial_biguint(n: u64) -> BigUint {
        let mut result = BigUint::from(1_u8);
        for k in 2..=n {
            result *= BigUint::from(k);
        }
        result
    }

    fn binomial_biguint(n: u64, k: u64) -> BigUint {
        if k > n {
            return BigUint::from(0_u8);
        }
        let k = k.min(n - k);
        let mut result = BigUint::from(1_u8);
        for i in 1..=k {
            result *= BigUint::from(n - k + i);
            result /= BigUint::from(i);
        }
        result
    }

    fn rational_from_biguint(value: BigUint) -> Rational {
        Rational::from_bigint_fraction(BigInt::from_biguint(Sign::Plus, value), BigUint::from(1_u8))
            .unwrap()
    }

    fn gamma_half_integer(twice: i64) -> Result<Real, Problem> {
        if twice <= 0 && twice % 2 == 0 {
            return Err(Problem::NotANumber);
        }

        if twice > 0 && twice % 2 == 0 {
            let n = u64::try_from(twice / 2).map_err(|_| Problem::OutOfRange)?;
            return Ok(Self::new(Self::rational_from_biguint(
                Self::factorial_biguint(n - 1),
            )));
        }

        let sqrt_pi = Self::pi().sqrt()?;
        if twice > 0 {
            let k = u64::try_from((twice - 1) / 2).map_err(|_| Problem::OutOfRange)?;
            let numerator = Self::factorial_biguint(2 * k);
            let denominator = (BigUint::from(1_u8) << (2 * k)) * Self::factorial_biguint(k);
            let scale = Rational::from_bigint_fraction(
                BigInt::from_biguint(Sign::Plus, numerator),
                denominator,
            )
            .unwrap();
            return Ok(Self::new(scale) * sqrt_pi);
        }

        let m = u64::try_from((1 - twice) / 2).map_err(|_| Problem::OutOfRange)?;
        let numerator = (BigUint::from(1_u8) << (2 * m)) * Self::factorial_biguint(m);
        let denominator = Self::factorial_biguint(2 * m);
        let scale = Rational::from_bigint_fraction(
            BigInt::from_biguint(
                if m.is_multiple_of(2) {
                    Sign::Plus
                } else {
                    Sign::Minus
                },
                numerator,
            ),
            denominator,
        )
        .unwrap();
        Ok(Self::new(scale) * sqrt_pi)
    }

    fn gamma_half_integer_abs_scale(twice: i64) -> Result<(Rational, bool), Problem> {
        if twice <= 0 && twice % 2 == 0 {
            return Err(Problem::NotANumber);
        }

        if twice > 0 && twice % 2 == 0 {
            let n = u64::try_from(twice / 2).map_err(|_| Problem::OutOfRange)?;
            return Ok((
                Self::rational_from_biguint(Self::factorial_biguint(n - 1)),
                false,
            ));
        }

        if twice > 0 {
            let k = u64::try_from((twice - 1) / 2).map_err(|_| Problem::OutOfRange)?;
            let numerator = Self::factorial_biguint(2 * k);
            let denominator = (BigUint::from(1_u8) << (2 * k)) * Self::factorial_biguint(k);
            return Ok((
                Rational::from_bigint_fraction(
                    BigInt::from_biguint(Sign::Plus, numerator),
                    denominator,
                )
                .unwrap(),
                true,
            ));
        }

        let m = u64::try_from((1 - twice) / 2).map_err(|_| Problem::OutOfRange)?;
        let numerator = (BigUint::from(1_u8) << (2 * m)) * Self::factorial_biguint(m);
        let denominator = Self::factorial_biguint(2 * m);
        Ok((
            Rational::from_bigint_fraction(
                BigInt::from_biguint(Sign::Plus, numerator),
                denominator,
            )
            .unwrap(),
            true,
        ))
    }

    /// Gamma function for exact integer and half-integer real arguments.
    ///
    /// Poles at non-positive integers return [`Problem::NotANumber`]. Other
    /// shapes are left to a future approximation kernel.
    pub fn gamma(self) -> Result<Real, Problem> {
        let twice = Self::exact_half_integer_twice(&self)?;
        crate::trace_dispatch!("real", "gamma", "half-integer-closed-form");
        Self::gamma_half_integer(twice)
    }

    /// Natural logarithm of the absolute gamma function.
    ///
    /// This follows the common real `lgamma` convention: negative half-integer
    /// values are accepted and the sign of `gamma(x)` is discarded before `ln`.
    pub fn lgamma(self) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "lgamma", "log-abs-gamma");
        self.gamma()?.abs().ln()
    }

    /// Beta function `B(a, b) = gamma(a) * gamma(b) / gamma(a + b)`.
    pub fn beta(a: &Self, b: &Self) -> Result<Real, Problem> {
        if let (Ok(a_int), Ok(b_int)) = (Self::exact_positive_integer(a), Self::exact_positive_integer(b)) {
            let denominator_arg = a_int
                .checked_add(b_int)
                .and_then(|sum| sum.checked_sub(1))
                .ok_or(Problem::OutOfRange)?;
            crate::trace_dispatch!("real", "beta", "positive-integer-factorial-ratio");
            let numerator = Self::factorial_biguint(a_int - 1) * Self::factorial_biguint(b_int - 1);
            let denominator = Self::factorial_biguint(denominator_arg);
            return Ok(Self::new(
                Rational::from_bigint_fraction(
                    BigInt::from_biguint(Sign::Plus, numerator),
                    denominator,
                )
                .unwrap(),
            ));
        }

        crate::trace_dispatch!("real", "beta", "gamma-ratio");
        let numerator = a.clone().gamma()? * b.clone().gamma()?;
        numerator / (a + b).gamma()?
    }

    /// Natural logarithm of the absolute beta function.
    pub fn ln_beta(a: &Self, b: &Self) -> Result<Real, Problem> {
        let twice_a = Self::exact_half_integer_twice(a)?;
        let twice_b = Self::exact_half_integer_twice(b)?;
        let twice_sum = twice_a.checked_add(twice_b).ok_or(Problem::OutOfRange)?;
        let (scale_a, sqrt_a) = Self::gamma_half_integer_abs_scale(twice_a)?;
        let (scale_b, sqrt_b) = Self::gamma_half_integer_abs_scale(twice_b)?;
        let (scale_sum, sqrt_sum) = Self::gamma_half_integer_abs_scale(twice_sum)?;

        crate::trace_dispatch!("real", "ln_beta", "half-integer-scale-log");
        let scale = (&scale_a * &scale_b) / &scale_sum;
        let mut result = if scale.is_one() {
            Self::zero()
        } else {
            Self::new(scale).ln()?
        };

        let sqrt_pi_count = i32::from(sqrt_a) + i32::from(sqrt_b) - i32::from(sqrt_sum);
        if sqrt_pi_count != 0 {
            let scale = Rational::from_bigint_fraction(
                BigInt::from(sqrt_pi_count),
                BigUint::from(2_u8),
            )
            .unwrap();
            result += Self::new(scale) * Self::pi().ln()?;
        }
        Ok(result)
    }

    /// Alias for [`Real::ln_beta`].
    pub fn lbeta(a: &Self, b: &Self) -> Result<Real, Problem> {
        Self::ln_beta(a, b)
    }

    /// Regularized incomplete beta `I_x(a, b)`.
    ///
    /// Currently supports exact positive integer `a` and `b`, with `0 <= x <=
    /// 1`, through the finite binomial-tail identity.
    pub fn regularized_beta(a: &Self, b: &Self, x: &Self) -> Result<Real, Problem> {
        let a = Self::exact_positive_integer(a)?;
        let b = Self::exact_positive_integer(b)?;
        match x.best_sign() {
            Sign::Minus => return Err(Problem::NotANumber),
            Sign::NoSign => return Ok(Self::zero()),
            Sign::Plus => {}
        }
        let one_minus_x = Self::one() - x.clone();
        match one_minus_x.best_sign() {
            Sign::Minus => return Err(Problem::NotANumber),
            Sign::NoSign => return Ok(Self::one()),
            Sign::Plus => {}
        }
        if a == 1 && b == 1 {
            crate::trace_dispatch!("real", "regularized_beta", "uniform-identity");
            return Ok(x.clone());
        }
        if b == 1 {
            crate::trace_dispatch!("real", "regularized_beta", "right-unity-power");
            return x.clone().powi(BigInt::from(a));
        }
        if a == 1 {
            crate::trace_dispatch!("real", "regularized_beta", "left-unity-complement-power");
            return Ok(Self::one() - one_minus_x.powi(BigInt::from(b))?);
        }

        let n = a.checked_add(b).and_then(|sum| sum.checked_sub(1));
        let Some(n) = n else {
            return Err(Problem::OutOfRange);
        };

        crate::trace_dispatch!("real", "regularized_beta", "integer-binomial-tail");
        let mut total = Self::zero();
        for j in a..=n {
            let coeff = Self::new(Self::rational_from_biguint(Self::binomial_biguint(n, j)));
            let x_power = x.clone().powi(BigInt::from(j))?;
            let one_minus_power = one_minus_x.clone().powi(BigInt::from(n - j))?;
            total += coeff * x_power * one_minus_power;
        }
        Ok(total)
    }

    /// Complement of [`Real::regularized_beta`].
    pub fn regularized_beta_q(a: &Self, b: &Self, x: &Self) -> Result<Real, Problem> {
        let a = Self::exact_positive_integer(a)?;
        let b = Self::exact_positive_integer(b)?;
        match x.best_sign() {
            Sign::Minus => return Err(Problem::NotANumber),
            Sign::NoSign => return Ok(Self::one()),
            Sign::Plus => {}
        }
        let one_minus_x = Self::one() - x.clone();
        match one_minus_x.best_sign() {
            Sign::Minus => return Err(Problem::NotANumber),
            Sign::NoSign => return Ok(Self::zero()),
            Sign::Plus => {}
        }
        if a == 1 && b == 1 {
            crate::trace_dispatch!("real", "regularized_beta_q", "uniform-complement");
            return Ok(one_minus_x);
        }
        if b == 1 {
            crate::trace_dispatch!("real", "regularized_beta_q", "right-unity-complement-power");
            return Ok(Self::one() - x.clone().powi(BigInt::from(a))?);
        }
        if a == 1 {
            crate::trace_dispatch!("real", "regularized_beta_q", "left-unity-power");
            return one_minus_x.powi(BigInt::from(b));
        }

        let n = a.checked_add(b).and_then(|sum| sum.checked_sub(1));
        let Some(n) = n else {
            return Err(Problem::OutOfRange);
        };

        crate::trace_dispatch!("real", "regularized_beta_q", "integer-binomial-tail");
        let mut total = Self::zero();
        for j in 0..a {
            let coeff = Self::new(Self::rational_from_biguint(Self::binomial_biguint(n, j)));
            let x_power = x.clone().powi(BigInt::from(j))?;
            let one_minus_power = one_minus_x.clone().powi(BigInt::from(n - j))?;
            total += coeff * x_power * one_minus_power;
        }
        Ok(total)
    }

    fn nonnegative_half_power(x: &Self, twice_power: u64) -> Result<Real, Problem> {
        if twice_power == 0 {
            return Ok(Self::one());
        }

        let integer_power = twice_power / 2;
        let mut result = if integer_power == 0 {
            Self::one()
        } else {
            x.clone().powi(BigInt::from(integer_power))?
        };
        if twice_power % 2 == 1 {
            result *= x.clone().sqrt()?;
        }
        Ok(result)
    }

    fn gamma_recurrence_inverse(current_twice: u64) -> Result<Real, Problem> {
        if current_twice.is_multiple_of(2) {
            let denominator = Self::factorial_biguint(current_twice / 2);
            return Ok(Self::new(
                Rational::from_bigint_fraction(BigInt::from(1_u8), denominator).unwrap(),
            ));
        }

        let k = current_twice.div_ceil(2);
        let numerator = (BigUint::from(1_u8) << (2 * k)) * Self::factorial_biguint(k);
        let denominator = Self::factorial_biguint(2 * k);
        let rational =
            Rational::from_bigint_fraction(BigInt::from_biguint(Sign::Plus, numerator), denominator)
                .unwrap();
        Ok(Self::new(rational) * Self::inv_sqrt_pi()?)
    }

    fn regularized_gamma_recurrence_term(
        current_twice: u64,
        x: &Self,
        decay: &Self,
    ) -> Result<Real, Problem> {
        let power = Self::nonnegative_half_power(x, current_twice)?;
        Ok(power * decay * Self::gamma_recurrence_inverse(current_twice)?)
    }

    fn regularized_gamma_single_tail(
        a: &Self,
        x: &Self,
        upper: bool,
    ) -> Result<Real, Problem> {
        let target_twice = Self::exact_positive_half_integer_twice(a)?;
        match x.best_sign() {
            Sign::Minus => return Err(Problem::NotANumber),
            Sign::NoSign => {
                return Ok(if upper { Self::one() } else { Self::zero() });
            }
            Sign::Plus => {}
        }

        let decay = (-x.clone()).exp()?;
        let (mut value, mut current_twice) = if target_twice % 2 == 0 {
            let q = decay.clone();
            (if upper { q } else { Self::one() - q }, 2)
        } else {
            let sqrt_x = x.clone().sqrt()?;
            (if upper { sqrt_x.erfc() } else { sqrt_x.erf() }, 1)
        };

        while current_twice < target_twice {
            let term = Self::regularized_gamma_recurrence_term(current_twice, x, &decay)?;
            if upper {
                value += term;
            } else {
                value -= term;
            }
            current_twice += 2;
        }

        Ok(value)
    }

    /// Regularized lower incomplete gamma P(a, x).
    ///
    /// Currently supports exact positive integer and half-integer `a`, with
    /// `x >= 0`.
    pub fn regularized_gamma_p(a: &Self, x: &Self) -> Result<Real, Problem> {
        Self::regularized_gamma_single_tail(a, x, false)
    }

    /// Regularized upper incomplete gamma Q(a, x).
    ///
    /// Currently supports exact positive integer and half-integer `a`, with
    /// `x >= 0`.
    pub fn regularized_gamma_q(a: &Self, x: &Self) -> Result<Real, Problem> {
        Self::regularized_gamma_single_tail(a, x, true)
    }

    /// Chi-square CDF with `k` positive degrees of freedom.
    pub fn chi_square_cdf(x: &Self, k: u64) -> Result<Real, Problem> {
        if k == 0 {
            return Err(Problem::NotANumber);
        }
        let shape = Self::new(
            Rational::from_bigint_fraction(BigInt::from(k), BigUint::from(2_u8)).unwrap(),
        );
        let half_x = (x.clone() / Self::from(2_i32))?;
        Self::regularized_gamma_p(&shape, &half_x)
    }

    /// Chi-square upper-tail probability with `k` positive degrees of freedom.
    pub fn chi_square_sf(x: &Self, k: u64) -> Result<Real, Problem> {
        if k == 0 {
            return Err(Problem::NotANumber);
        }
        let shape = Self::new(
            Rational::from_bigint_fraction(BigInt::from(k), BigUint::from(2_u8)).unwrap(),
        );
        let half_x = (x.clone() / Self::from(2_i32))?;
        Self::regularized_gamma_q(&shape, &half_x)
    }

    /// Standard normal quantile, the inverse of [`Real::pnorm`].
    pub fn qnorm(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "qnorm", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        if self.class == One && self.rational == *rationals::HALF {
            crate::trace_dispatch!("real", "qnorm", "exact-half-zero");
            return Ok(Self::zero());
        }
        if self.class == One && self.rational >= *rationals::ONE {
            crate::trace_dispatch!("real", "qnorm", "domain-one-or-more");
            return Err(Problem::NotANumber);
        }

        let p_f64: f64 = self.clone().into();
        let seed = if p_f64 > 0.5 {
            let upper_tail = Self::one() - self.clone();
            let tail_f64: f64 = upper_tail.into();
            -Self::qnorm_seed_approx(tail_f64.max(1e-300))
        } else {
            Self::qnorm_seed_approx(p_f64.max(1e-300))
        };
        Self::normal_quantile_from_seeded_cdf(self, seed, "qnorm")
    }

    /// The sine of this Real.
    pub fn sin(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sin", "exact-zero");
            return Self::zero();
        }
        match &self.class {
            One => {
                // Plain rational trig still uses Computable, not SinPi/TanPi:
                // those exact certificates are reserved for rational multiples
                // of pi where algebra can later invert them. The owned helper
                // specializes before allocating a generic Ratio leaf.
                let computable = if self.rational.magnitude_at_least_power_of_two(3) {
                    // Keep the large-rational decision at the Real layer too:
                    // this path is already below 100 ns, so avoiding a second
                    // sign/MSD probe matters in Criterion.
                    crate::trace_dispatch!("real", "sin", "large-rational-deferred-node");
                    Computable::sin_large_rational_deferred(self.rational.clone())
                } else {
                    crate::trace_dispatch!("real", "sin", "rational-specialized-computable");
                    Computable::sin_rational(self.rational.clone())
                };
                return Self::irrational_from_computable(computable);
            }
            Pi => {
                // sin(q*pi) has exact small-denominator and reusable SinPi handling.
                crate::trace_dispatch!("real", "sin", "pi-rational-special-form");
                return Self::sin_pi_rational(self.rational);
            }
            _ => (),
        }
        if let Some((negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "sin", "integer-pi-offset-rewrite");
            let reduced = Self::irrational_from_computable(Computable::sin_rational(residual));
            return if negate { reduced.neg() } else { reduced };
        }

        crate::trace_dispatch!("real", "sin", "generic-computable");
        self.make_computable(Computable::sin)
    }

    /// Sine of `pi * x`, with exact rational-turn special cases.
    pub fn sin_pi(self) -> Real {
        if self.class == One {
            crate::trace_dispatch!("real", "sin_pi", "rational-special-form");
            return Self::sin_pi_rational(self.rational);
        }
        (self * Self::pi()).sin()
    }

    /// The cosine of this Real.
    pub fn cos(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "cos", "exact-zero-one");
            return Self::one();
        }
        match &self.class {
            One => {
                // Same policy as sine: exact pi multiples stay symbolic, while
                // plain rationals enter the specialized computable constructor.
                let computable = if self.rational.magnitude_at_least_power_of_two(3) {
                    crate::trace_dispatch!("real", "cos", "large-rational-deferred-node");
                    Computable::cos_large_rational_deferred(self.rational.clone())
                } else {
                    crate::trace_dispatch!("real", "cos", "rational-specialized-computable");
                    Computable::cos_rational(self.rational.clone())
                };
                return Self::irrational_from_computable(computable);
            }
            Pi => {
                if let Some(exact) = Self::cos_pi_rational(self.rational.clone()) {
                    crate::trace_dispatch!("real", "cos", "pi-rational-exact-table");
                    return exact;
                }
                // cos(q*pi) is represented through the same SinPi machinery with a
                // half-turn shift, keeping exact identities in one place.
                crate::trace_dispatch!("real", "cos", "pi-rational-sinpi-rewrite");
                return Self::sin_pi_rational(self.rational + rationals::HALF.clone());
            }
            _ => (),
        }
        if let Some((negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "cos", "integer-pi-offset-rewrite");
            let reduced = Self::irrational_from_computable(Computable::cos_rational(residual));
            return if negate { reduced.neg() } else { reduced };
        }

        crate::trace_dispatch!("real", "cos", "generic-computable");
        self.make_computable(Computable::cos)
    }

    /// Cosine of `pi * x`, with exact rational-turn special cases.
    pub fn cos_pi(self) -> Real {
        if self.class == One {
            if let Some(exact) = Self::cos_pi_rational(self.rational.clone()) {
                crate::trace_dispatch!("real", "cos_pi", "rational-exact-table");
                return exact;
            }
            crate::trace_dispatch!("real", "cos_pi", "rational-sinpi-rewrite");
            return Self::sin_pi_rational(self.rational + rationals::HALF.clone());
        }
        (self * Self::pi()).cos()
    }

    /// The tangent of this Real.
    pub fn tan(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "tan", "exact-zero");
            return Ok(Self::zero());
        }

        match &self.class {
            One => {
                // For non-pi rational arguments there are no exact tangent
                // certificates, but Computable::tan still applies small/medium
                // argument reductions without first allocating a Ratio leaf.
                crate::trace_dispatch!("real", "tan", "rational-specialized-computable");
                return Ok(Self::irrational_from_computable(Computable::tan_rational(
                    self.rational.clone(),
                )));
            }
            Pi => {
                return Self::tan_pi_rational(self.rational);
            }
            _ => (),
        }
        if let Some((_negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "tan", "integer-pi-offset-rewrite");
            return Ok(Self::irrational_from_computable(Computable::tan_rational(
                residual,
            )));
        }

        crate::trace_dispatch!("real", "tan", "generic-computable");
        Ok(self.make_computable(Computable::tan))
    }

    /// Tangent of `pi * x`, with exact rational-turn special cases.
    pub fn tan_pi(self) -> Result<Real, Problem> {
        if self.class == One {
            return Self::tan_pi_rational(self.rational);
        }
        (self * Self::pi()).tan()
    }

    fn tan_pi_rational(rational: Rational) -> Result<Real, Problem> {
        if rational.is_integer() {
            crate::trace_dispatch!("real", "tan", "pi-integer-zero");
            return Ok(Self::zero());
        }
        if rational.sign() == Sign::Plus && rational < *rationals::HALF {
            let denominator = rational.denominator();
            if denominator == unsigned::THREE.deref() {
                crate::trace_dispatch!("real", "tan", "pi-rational-exact-table");
                return Ok(constants::sqrt_three());
            }
            if denominator == unsigned::FOUR.deref() {
                crate::trace_dispatch!("real", "tan", "pi-rational-exact-table");
                return Ok(Self::one());
            }
            if denominator == unsigned::SIX.deref() {
                crate::trace_dispatch!("real", "tan", "pi-rational-exact-table");
                return Ok(constants::sqrt_three_over_three());
            }
        }
        // Rational multiples of pi get exact tangent values for the usual small
        // denominators, otherwise a compact TanPi certificate.
        let (neg, n) = tan_curve(rational);
        let mut r: Option<Real> = None;
        let d = n.denominator();
        if d == unsigned::TWO.deref() {
            crate::trace_dispatch!("real", "tan", "pi-half-pole");
            return Err(Problem::NotANumber);
        }
        if d == unsigned::THREE.deref() {
            r = Some(constants::sqrt_three());
        }
        if d == unsigned::FOUR.deref() {
            r = Some(Self::one());
        }
        if d == unsigned::SIX.deref() {
            r = Some(constants::sqrt_three_over_three());
        }
        if let Some(real) = r {
            crate::trace_dispatch!("real", "tan", "pi-rational-exact-table");
            return Ok(if neg { real.neg() } else { real });
        }

        let new = Computable::multiply(Computable::pi(), Computable::rational(n.clone()));
        let computable = Computable::prescaled_tan(new);
        crate::trace_dispatch!("real", "tan", "tanpi-special-form");
        Ok(if neg {
            Self {
                rational: Rational::new(-1),
                class: TanPi(n),
                computable: Some(computable),
                signal: None,
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
            }
        } else {
            Self {
                rational: Rational::one(),
                class: TanPi(n),
                computable: Some(computable),
                signal: None,
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
            }
        })
    }

    /// `sin(x) / x`, with the removable singularity `sinc(0) = 1`.
    pub fn sinc(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sinc", "exact-zero-one");
            return Ok(Self::one());
        }

        crate::trace_dispatch!("real", "sinc", "sin-over-x");
        self.clone().sin() / self
    }

    /// `sin(pi * x) / (pi * x)`, with the removable singularity `sinc_pi(0) = 1`.
    pub fn sinc_pi(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sinc_pi", "exact-zero-one");
            return Ok(Self::one());
        }

        crate::trace_dispatch!("real", "sinc_pi", "sinpi-over-pi-x");
        let denominator = self.clone() * Self::pi();
        self.sin_pi() / denominator
    }

    /// `(1 - cos(x)) / x^2`, with the removable singularity `cosc(0) = 1/2`.
    pub fn cosc(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "cosc", "exact-zero-half");
            return Ok(constants::half());
        }

        crate::trace_dispatch!("real", "cosc", "one-minus-cos-over-square");
        let numerator = Self::one() - self.clone().cos();
        numerator / (self.clone() * self)
    }

    fn pi_fraction(n: i64, d: u64) -> Real {
        if let Some(real) = constants::pi_fraction(n, d) {
            crate::trace_dispatch!("real", "pi_fraction", "cached-special-form");
            return real;
        }
        crate::trace_dispatch!("real", "pi_fraction", "constructed-generic");
        Self::new(Rational::fraction(n, d).unwrap()) * Self::pi()
    }

    fn asin_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::zero());
        }

        match &self.class {
            One => {
                // Exact inverse-trig table for rational endpoints and half-angle values.
                if self.rational.is_one() {
                    Some(Self::pi_fraction(1, 2))
                } else if self.rational.is_minus_one() {
                    Some(Self::pi_fraction(-1, 2))
                } else if self.rational == *rationals::HALF {
                    Some(Self::pi_fraction(1, 6))
                } else if self.rational.sign() == Sign::Minus
                    && self.rational.compare_magnitude(&rationals::HALF)
                        == std::cmp::Ordering::Equal
                {
                    Some(Self::pi_fraction(-1, 6))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                // Recognize sqrt(2)/2 and sqrt(3)/2 forms produced by exact trig.
                // This is a structural table lookup, not a numerical comparison:
                // the radicand must be an exact small integer and the rational
                // scale must have exact magnitude 1/2.
                let sign = self.rational.sign();
                let half_magnitude =
                    self.rational.compare_magnitude(&rationals::HALF) == std::cmp::Ordering::Equal;
                if !half_magnitude {
                    return None;
                }
                let angle = match r.to_integer_i64()? {
                    2 => rationals::QUARTER.clone(),
                    3 => rationals::THIRD.clone(),
                    _ => return None,
                };

                let angle = if sign == Sign::Minus {
                    angle.neg()
                } else {
                    angle
                };
                Some(Self::new(angle) * Self::pi())
            }
            SinPi(r) => {
                // asin(sin(q*pi)) can reuse the stored angle when it is already in the
                // principal branch represented by SinPi.
                if self.rational.is_one() {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational.is_minus_one() {
                    Some(Self::new(r.clone().neg()) * Self::pi())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn acos_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::pi_fraction(1, 2));
        }

        match &self.class {
            One => {
                if self.rational.is_one() {
                    Some(Self::zero())
                } else if self.rational.is_minus_one() {
                    Some(Self::pi())
                } else if self.rational == *rationals::HALF {
                    Some(Self::pi_fraction(1, 3))
                } else if self.rational.sign() == Sign::Minus
                    && self.rational.compare_magnitude(&rationals::HALF)
                        == std::cmp::Ordering::Equal
                {
                    Some(Self::pi_fraction(2, 3))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                let sign = self.rational.sign();
                let half_magnitude =
                    self.rational.compare_magnitude(&rationals::HALF) == std::cmp::Ordering::Equal;
                if !half_magnitude {
                    return None;
                }
                let angle = match (sign, r.to_integer_i64()?) {
                    (Sign::Minus, 2) => Rational::fraction(3, 4).unwrap(),
                    (_, 2) => rationals::QUARTER.clone(),
                    (Sign::Minus, 3) => Rational::fraction(5, 6).unwrap(),
                    (_, 3) => rationals::SIXTH.clone(),
                    _ => return None,
                };
                Some(Self::new(angle) * Self::pi())
            }
            SinPi(r) => {
                if self.rational.is_one() {
                    Some(Self::new(rationals::HALF.clone() - r.clone()) * Self::pi())
                } else if self.rational.is_minus_one() {
                    Some(Self::new(rationals::HALF.clone() + r.clone()) * Self::pi())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn atan_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::zero());
        }

        match &self.class {
            One => {
                // atan(+/-1) is one of the few rational inputs with an exact
                // pi-fraction result; catching it avoids constructing an atan node.
                if self.rational.is_one() {
                    Some(Self::pi_fraction(1, 4))
                } else if self.rational.is_minus_one() {
                    Some(Self::pi_fraction(-1, 4))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                if r.to_integer_i64() != Some(3) {
                    return None;
                }
                // atan(sqrt(3)) and atan(sqrt(3)/3) have exact pi-fraction answers.
                let sign = self.rational.sign();
                let angle = if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Equal {
                    Some(rationals::THIRD.clone())
                } else if self.rational.compare_magnitude(&rationals::THIRD)
                    == std::cmp::Ordering::Equal
                {
                    Some(rationals::SIXTH.clone())
                } else {
                    None
                }?;

                let angle = if sign == Sign::Minus {
                    angle.neg()
                } else {
                    angle
                };
                Some(Self::new(angle) * Self::pi())
            }
            TanPi(r) => {
                // Preserve exact inverse for TanPi certificates instead of going through
                // the generic atan kernel.
                if self.rational.is_one() {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational.is_minus_one() {
                    Some(Self::new(r.clone().neg()) * Self::pi())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// The inverse sine of this Real, or [`Problem::NotANumber`] outside [-1, 1].
    pub fn asin(self) -> Result<Real, Problem> {
        if let Some(exact) = self.asin_exact() {
            crate::trace_dispatch!("real", "asin", "exact-special-form");
            return Ok(exact);
        }
        if self.class == One {
            // Plain rationals use the computable asin kernel after cheap domain checks; it
            // has tiny/endpoint specializations that would be obscured by the atan formula.
            if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater {
                crate::trace_dispatch!("real", "asin", "rational-domain-error");
                return Err(Problem::NotANumber);
            }

            crate::trace_dispatch!("real", "asin", "rational-computable");
            return Ok(self.make_computable(|value| value.asin()));
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &rationals::ONE)
                == std::cmp::Ordering::Greater
        {
            crate::trace_dispatch!("real", "asin", "sqrt-domain-error");
            return Err(Problem::NotANumber);
        }
        if matches!(&self.class, Sqrt(_)) {
            // Sqrt inputs commonly arise from exact trig; keep them on the computable asin
            // path so recognizable forms survive longer.
            crate::trace_dispatch!("real", "asin", "sqrt-computable");
            return Ok(self.make_computable(|value| value.asin()));
        }

        // Generic identity asin(x) = atan(x / sqrt(1-x^2)).
        crate::trace_dispatch!("real", "asin", "generic-atan-sqrt-rewrite");
        let one = Self::one();
        let radicand = one.clone() - self.clone().powi(BigInt::from(2_u8))?;
        let denominator = radicand.sqrt().map_err(|problem| match problem {
            Problem::SqrtNegative => Problem::NotANumber,
            other => other,
        })?;
        (self / denominator)?.atan()
    }

    /// The inverse cosine of this Real, or [`Problem::NotANumber`] outside [-1, 1].
    pub fn acos(self) -> Result<Real, Problem> {
        if let Some(exact) = self.acos_exact() {
            crate::trace_dispatch!("real", "acos", "exact-special-form");
            return Ok(exact);
        }
        if self.class == One
            && self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater
        {
            // Exact rational domain failures are rejected before any
            // approximation machinery is constructed.
            crate::trace_dispatch!("real", "acos", "rational-domain-error");
            return Err(Problem::NotANumber);
        }
        if !matches!(&self.class, Sqrt(_))
            && let Some(asin) = self.asin_exact()
        {
            // acos(x) shares the exact asin table through pi/2 - asin(x).
            crate::trace_dispatch!("real", "acos", "asin-table-special-form");
            return Ok(Self::pi_fraction(1, 2) - asin);
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &rationals::ONE)
                == std::cmp::Ordering::Greater
        {
            crate::trace_dispatch!("real", "acos", "sqrt-domain-error");
            return Err(Problem::NotANumber);
        }

        crate::trace_dispatch!("real", "acos", "generic-computable");
        Ok(self.make_computable(|value| value.acos()))
    }

    /// The inverse tangent of this Real.
    pub fn atan(self) -> Result<Real, Problem> {
        if let Some(exact) = self.atan_exact() {
            crate::trace_dispatch!("real", "atan", "exact-special-form");
            return Ok(exact);
        }

        crate::trace_dispatch!("real", "atan", "generic-computable");
        Ok(self.make_computable(Computable::atan))
    }

    /// Two-argument arctangent of `(self, x)`, returning the angle of the
    /// point `(x, self)` measured counterclockwise from the positive `x`
    /// axis in the principal range `(-pi, pi]`.
    ///
    /// `self` is the `y` coordinate and `x` is the `x` coordinate, matching
    /// the IEEE 754 `atan2(y, x)` convention. The implementation reduces to
    /// the single-argument [`Real::atan`] kernel after a signed-pi quadrant
    /// correction, so existing `atan` exact special forms (such as
    /// `atan(1) = pi/4` or `atan(sqrt(3)) = pi/3`) flow through unchanged
    /// when the ratio `self / x` lands on one of them. Axes return exact
    /// pi multiples; the origin `(0, 0)` returns zero, matching the
    /// `f64::atan2` convention.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Real;
    /// // atan2(1, 1) == pi / 4
    /// assert_eq!(
    ///     Real::one().atan2(Real::one()),
    ///     (Real::pi() / Real::from(4_i32)).unwrap(),
    /// );
    /// // atan2(0, -1) == pi
    /// assert_eq!(Real::zero().atan2(-Real::one()), Real::pi());
    /// // atan2(1, 0) == pi / 2
    /// assert_eq!(
    ///     Real::one().atan2(Real::zero()),
    ///     (Real::pi() / Real::from(2_i32)).unwrap(),
    /// );
    /// ```
    pub fn atan2(self, x: Real) -> Real {
        // Structural sign first. `best_sign` for Irrational class refines the
        // computable graph until the sign is decided, which is dramatically
        // more expensive than reading already-derivable structural facts. Only
        // descend to the refinement path when structural inspection cannot
        // decide for one of the inputs.
        let y_sign = self.structural_facts().sign.map(num_sign_from_real);
        let x_sign = x.structural_facts().sign.map(num_sign_from_real);
        match (y_sign, x_sign) {
            (Some(Sign::NoSign), Some(Sign::NoSign)) | (Some(Sign::NoSign), Some(Sign::Plus)) => {
                crate::trace_dispatch!("real", "atan2", "axis-zero-y");
                return Self::zero();
            }
            (Some(Sign::NoSign), Some(Sign::Minus)) => {
                crate::trace_dispatch!("real", "atan2", "axis-negative-x");
                return Self::pi();
            }
            (Some(Sign::Plus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("real", "atan2", "axis-positive-y");
                return Self::pi_fraction(1, 2);
            }
            (Some(Sign::Minus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("real", "atan2", "axis-negative-y");
                return Self::pi_fraction(-1, 2);
            }
            _ => {}
        }
        let y_sign = y_sign.unwrap_or_else(|| self.best_sign());
        let x_sign = x_sign.unwrap_or_else(|| x.best_sign());
        if y_sign == Sign::NoSign && x_sign != Sign::Plus {
            crate::trace_dispatch!("real", "atan2", "generic-computable");
            return Self::irrational_from_computable(self.fold().atan2(x.fold()));
        }
        if x_sign == Sign::NoSign {
            crate::trace_dispatch!("real", "atan2", "generic-computable");
            return Self::irrational_from_computable(self.fold().atan2(x.fold()));
        }
        let ratio = (self / &x).expect("nonzero x rules out divide-by-zero");
        let base = ratio.atan().expect("Real::atan is total");
        if x_sign == Sign::Plus {
            crate::trace_dispatch!("real", "atan2", "quadrant-right");
            base
        } else if y_sign == Sign::Plus {
            crate::trace_dispatch!("real", "atan2", "quadrant-upper-left");
            base + Self::pi()
        } else {
            crate::trace_dispatch!("real", "atan2", "quadrant-lower-left");
            base - Self::pi()
        }
    }

    /// The hyperbolic sine of this Real.
    pub fn sinh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sinh", "exact-zero");
            return Ok(Self::zero());
        }
        if let Ln(base) = &self.class
            && let Some(int) = self.rational.to_big_integer()
        {
            // sinh(k*ln(n)) = (n^k - n^-k)/2 folds to an exact rational
            // whenever the symbolic ln scale is integral.
            let positive = base.clone().powi(int.clone())?;
            let negative = base.clone().powi(-int)?;
            crate::trace_dispatch!("real", "sinh", "integer-log-collapse");
            return Ok(Self::new((positive - negative) / Rational::new(2)));
        }
        crate::trace_dispatch!("real", "sinh", "generic-exp-identity");
        let positive = self.clone().exp()?;
        let negative = self.neg().exp()?;
        (positive - negative) / Self::new(Rational::new(2))
    }

    /// The hyperbolic cosine of this Real.
    pub fn cosh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "cosh", "exact-zero-one");
            return Ok(Self::one());
        }
        if let Ln(base) = &self.class
            && let Some(int) = self.rational.to_big_integer()
        {
            // cosh(k*ln(n)) = (n^k + n^-k)/2 folds to an exact rational
            // whenever the symbolic ln scale is integral.
            let positive = base.clone().powi(int.clone())?;
            let negative = base.clone().powi(-int)?;
            crate::trace_dispatch!("real", "cosh", "integer-log-collapse");
            return Ok(Self::new((positive + negative) / Rational::new(2)));
        }
        crate::trace_dispatch!("real", "cosh", "generic-exp-identity");
        let positive = self.clone().exp()?;
        let negative = self.neg().exp()?;
        (positive + negative) / Self::new(Rational::new(2))
    }

    /// The hyperbolic tangent of this Real.
    pub fn tanh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "tanh", "exact-zero");
            return Ok(Self::zero());
        }
        if let Ln(base) = &self.class
            && let Some(int) = self.rational.to_big_integer()
        {
            // tanh(k*ln(n)) = (n^2k - 1) / (n^2k + 1) folds to an exact
            // rational whenever the symbolic ln scale is integral.
            let squared = base.clone().powi(int * BigInt::from(2_u8))?;
            let one = Rational::one();
            crate::trace_dispatch!("real", "tanh", "integer-log-collapse");
            return Ok(Self::new((squared.clone() - one.clone()) / (squared + one)));
        }
        crate::trace_dispatch!("real", "tanh", "generic-exp-identity");
        let positive = self.clone().exp()?;
        let negative = self.neg().exp()?;
        (&positive - &negative) / (positive + negative)
    }

    /// The inverse hyperbolic sine of this Real.
    pub fn asinh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "asinh", "exact-zero");
            return Ok(Self::zero());
        }
        if self.class == One && self.rational.msd_exact().is_some_and(|msd| msd <= -4) {
            // Tiny exact rationals have a dedicated computable asinh series.
            // Enter it directly before Real-level odd symmetry expands the
            // expression into a larger ln1p graph.
            crate::trace_dispatch!("real", "asinh", "tiny-rational-computable");
            return Ok(self.make_computable(Computable::asinh));
        }
        if self.class == One {
            if self.rational.sign() == Sign::Minus {
                crate::trace_dispatch!("real", "asinh", "rational-negative-symmetry");
                return Ok(self.neg().asinh()?.neg());
            }
            if self.rational.msd_exact().is_some_and(|msd| msd < 3) {
                crate::trace_dispatch!("real", "asinh", "rational-near-zero-deferred-node");
                return Ok(self.make_computable(Computable::asinh_near_zero_deferred));
            }
            crate::trace_dispatch!("real", "asinh", "rational-direct-deferred-node");
            return Ok(self.make_computable(Computable::asinh_direct_deferred));
        }
        let folded = self.fold_ref();
        let (known_sign, planning_msd) = folded.planning_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("real", "asinh", "negative-symmetry");
            return Ok(self.neg().asinh()?.neg());
        } else if known_sign.is_none() && self.best_sign() == Sign::Minus {
            // Fall back to the slower exact sign check only when the planning
            // layer cannot determine sign from symbolic structure.
            crate::trace_dispatch!("real", "asinh", "negative-symmetry-fallback");
            return Ok(self.neg().asinh()?.neg());
        }
        let is_near_zero = match planning_msd.flatten() {
            Some(msd) => msd < 3,
            None => folded.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_zero {
            // Near zero, delegate to the deferred computable ln1p reduction so
            // public construction stays cheap without giving up the stable
            // approximation identity.
            crate::trace_dispatch!("real", "asinh", "near-zero-deferred-node");
            return Ok(self.make_computable(Computable::asinh_near_zero_deferred));
        }
        crate::trace_dispatch!("real", "asinh", "direct-deferred-node");
        Ok(self.make_computable(Computable::asinh_direct_deferred))
    }

    /// The inverse hyperbolic cosine of this Real, or [`Problem::NotANumber`] for values < 1.
    pub fn acosh(self) -> Result<Real, Problem> {
        if self.class == One {
            match self.rational.cmp_one_structural() {
                std::cmp::Ordering::Equal => {
                    crate::trace_dispatch!("real", "acosh", "exact-one-zero");
                    return Ok(Self::zero());
                }
                std::cmp::Ordering::Less => {
                    crate::trace_dispatch!("real", "acosh", "rational-domain-error");
                    return Err(Problem::NotANumber);
                }
                std::cmp::Ordering::Greater => {}
            }
            if self.rational.is_two() {
                crate::trace_dispatch!("real", "acosh", "exact-two-shared-constant");
                return Ok(Self::irrational_from_computable(
                    Computable::acosh2_constant(),
                ));
            }
            if self.rational >= *rationals::TWO {
                // Exact rationals at two or above are outside the
                // cancellation-prone acosh neighborhood. Use the direct
                // ln(x + sqrt(x^2 - 1)) node and reserve the ln1p transform
                // for values genuinely close to one.
                crate::trace_dispatch!(
                    "real",
                    "acosh",
                    "rational-at-least-two-direct-deferred-node"
                );
                return Ok(self.make_computable(Computable::acosh_direct_deferred));
            }
            if self.rational.msd_exact().is_some_and(|msd| msd >= 3) {
                // Large exact rationals cannot be in the cancellation-prone
                // neighborhood of one, so skip the low-precision proximity
                // probe and let the computable acosh kernel use its direct
                // large-input identity.
                crate::trace_dispatch!("real", "acosh", "large-rational-direct-deferred-node");
                return Ok(self.make_computable(Computable::acosh_direct_deferred));
            }
        } else if let Sqrt(r) = &self.class {
            // Domain-check factored sqrt values exactly: (a*sqrt(r))^2 = a^2*r.
            if self.rational.sign() == Sign::Minus
                || self
                    .rational
                    .compare_magnitude_squared_times(r, &rationals::ONE)
                    == std::cmp::Ordering::Less
            {
                crate::trace_dispatch!("real", "acosh", "sqrt-domain-error");
                return Err(Problem::NotANumber);
            }
            if self.rational.is_one() && r == &*rationals::TWO {
                crate::trace_dispatch!("real", "acosh", "sqrt-two-asinh-one");
                return Ok(Self::irrational_from_computable(
                    Computable::asinh1_constant(),
                ));
            }
            if self
                .rational
                .compare_magnitude_squared_times(r, &Rational::new(64))
                == std::cmp::Ordering::Less
            {
                crate::trace_dispatch!("real", "acosh", "sqrt-near-one-deferred-node");
                return Ok(self.make_computable(Computable::acosh_near_one_deferred));
            }
            crate::trace_dispatch!("real", "acosh", "sqrt-direct-deferred-node");
            return Ok(self.make_computable(Computable::acosh_direct_deferred));
        } else {
            let one = Self::one();
            if (self.clone() - one).best_sign() == Sign::Minus {
                crate::trace_dispatch!("real", "acosh", "generic-domain-error");
                return Err(Problem::NotANumber);
            }
        }
        let folded = self.fold_ref();
        let planned_acosh_msd = folded.planning_sign_and_msd().1;
        let is_near_one = match planned_acosh_msd.flatten() {
            Some(msd) => msd < 3,
            None => folded.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_one {
            // Near one, delegate to the deferred computable ln1p/sqrt
            // reduction so public construction does not allocate the full
            // approximation graph.
            crate::trace_dispatch!("real", "acosh", "near-one-deferred-node");
            return Ok(self.make_computable(Computable::acosh_near_one_deferred));
        }
        crate::trace_dispatch!("real", "acosh", "direct-deferred-node");
        Ok(self.make_computable(Computable::acosh_direct_deferred))
    }

    /// The inverse hyperbolic tangent of this Real.
    ///
    /// Returns [`Problem::Infinity`] at the endpoints `-1` and `1`, or
    /// [`Problem::NotANumber`] outside `(-1, 1)`.
    pub fn atanh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "atanh", "exact-zero");
            return Ok(Self::zero());
        }
        if self.class == One {
            if self.rational.is_one() || self.rational.is_minus_one() {
                crate::trace_dispatch!("real", "atanh", "endpoint-infinity");
                return Err(Problem::Infinity);
            }
            if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater {
                crate::trace_dispatch!("real", "atanh", "rational-domain-error");
                return Err(Problem::NotANumber);
            }
            if self.rational == *rationals::HALF {
                crate::trace_dispatch!("real", "atanh", "rational-half-ln3-special-form");
                return Ok(constants::half_ln3());
            }
            if self.rational.sign() == Sign::Minus
                && self.rational.compare_magnitude(&rationals::HALF) == std::cmp::Ordering::Equal
            {
                crate::trace_dispatch!("real", "atanh", "rational-minus-half-ln3-special-form");
                return Ok(-constants::half_ln3());
            }
            if self.rational.msd_exact().is_some_and(|msd| msd <= -4) {
                // Tiny rational atanh is faster in the dedicated computable kernel than
                // building ln((1+x)/(1-x))/2.
                crate::trace_dispatch!("real", "atanh", "tiny-rational-computable");
                return Ok(self.make_computable(Computable::atanh));
            }
            if self.rational.compare_magnitude(&rationals::SEVEN_EIGHTHS)
                != std::cmp::Ordering::Less
            {
                // Endpoint-adjacent rationals are hot in scalar predicates and
                // benchmarks; a deferred computable ln-ratio avoids eagerly
                // allocating the exact logarithm tree.
                crate::trace_dispatch!("real", "atanh", "endpoint-deferred-node");
                return Ok(self.make_computable(Computable::atanh_direct_deferred));
            }

            // This path deliberately keeps atanh(x) as the exact symbolic
            // `ln((1+x)/(1-x))/2` instead of approximating. Reuse the cached
            // unit rational so each construction only clones a tiny exact leaf
            // and does not rebuild/canonicalize it before the two rational
            // additions. This follows Boehm et al., "Exact Real Arithmetic: A
            // Case Study in Higher Order Programming" (1986), where symbolic
            // construction is kept separate from later numerical refinement.
            let one = rationals::ONE.clone();
            let ratio = (one.clone() + self.rational.clone()) / (one - self.rational);
            if ratio == *rationals::THREE {
                crate::trace_dispatch!("real", "atanh", "rational-half-ln3-special-form");
                return Ok(constants::half_ln3());
            }
            if ratio == *rationals::THIRD {
                crate::trace_dispatch!("real", "atanh", "rational-minus-half-ln3-special-form");
                return Ok(-constants::half_ln3());
            }
            // Non-tiny rationals can remain an exact logarithm ratio.
            crate::trace_dispatch!("real", "atanh", "rational-log-ratio-special-form");
            return Ok(Self::ln_rational(ratio)? * constants::half());
        }
        if let Sqrt(r) = &self.class {
            if r == &*rationals::TWO {
                if self.rational.compare_magnitude(&rationals::ONE) == std::cmp::Ordering::Equal {
                    crate::trace_dispatch!("real", "atanh", "sqrt-two-domain-error");
                    return Err(Problem::NotANumber);
                }
                if self.rational.compare_magnitude(&rationals::HALF) == std::cmp::Ordering::Equal {
                    // atanh(1/sqrt(2)) = ln(1 + sqrt(2)) = asinh(1). This
                    // exact structural identity is common enough to test
                    // before the generic squared-magnitude domain machinery.
                    crate::trace_dispatch!("real", "atanh", "sqrt-half-asinh-one");
                    let value = Self::irrational_from_computable(Computable::asinh1_constant());
                    return if self.rational.sign() == Sign::Minus {
                        Ok(-value)
                    } else {
                        Ok(value)
                    };
                }
            }
            match self
                .rational
                .compare_magnitude_squared_times(r, &rationals::ONE)
            {
                std::cmp::Ordering::Greater => {
                    // Exact sqrt domain failure avoids an approximation sign query.
                    crate::trace_dispatch!("real", "atanh", "sqrt-domain-error");
                    return Err(Problem::NotANumber);
                }
                std::cmp::Ordering::Equal => {
                    // Exact sqrt endpoint, e.g. sqrt(2)/2 scaled to magnitude one.
                    crate::trace_dispatch!("real", "atanh", "sqrt-endpoint-infinity");
                    return Err(Problem::Infinity);
                }
                std::cmp::Ordering::Less => {}
            }
            if self
                .rational
                .compare_magnitude_squared_times(r, &rationals::HALF)
                == std::cmp::Ordering::Equal
            {
                // atanh(1/sqrt(2)) = ln(1 + sqrt(2)) = asinh(1). This exact
                // structural identity is only possible after the broader
                // endpoint/domain check has proven the value is inside (-1, 1).
                crate::trace_dispatch!("real", "atanh", "sqrt-half-asinh-one");
                let value = Self::irrational_from_computable(Computable::asinh1_constant());
                return if self.rational.sign() == Sign::Minus {
                    Ok(-value)
                } else {
                    Ok(value)
                };
            }
        }
        if matches!(&self.class, Sqrt(_)) {
            // The exact sqrt-domain checks above already proved this input is
            // inside (-1, 1), so construction can stay deferred and materialize
            // the log-ratio graph only when digits are requested.
            crate::trace_dispatch!("real", "atanh", "sqrt-deferred-node");
            return Ok(self.make_computable(Computable::atanh_direct_deferred));
        }
        let one_real = Self::one();
        if self == one_real || self == -one_real.clone() {
            crate::trace_dispatch!("real", "atanh", "endpoint-infinity");
            return Err(Problem::Infinity);
        }
        crate::trace_dispatch!("real", "atanh", "generic-log-ratio-rewrite");
        let one = Self::one();
        let numerator = one.clone() + self.clone();
        let denominator = one - self;
        Ok((numerator / denominator)?.ln()? * constants::half())
    }

    fn recursive_powi(base: &Real, exp: &BigUint) -> Self {
        // Fallback for sign-unknown integer powers: repeated squaring is cheaper and more
        // exact than forcing ln/exp through a value whose sign cannot be certified.
        let mut result = Self::one();
        let mut factor = base.clone();
        let bits = exp.bits();
        for b in 0..bits {
            if exp.bit(b) {
                result *= factor.clone();
            }
            if b + 1 < bits {
                factor = factor.clone() * factor;
            }
        }
        result
    }

    fn compute_exp_ln_powi(value: Computable, exp: BigInt) -> Option<Computable> {
        match value.sign() {
            Sign::NoSign => None,
            Sign::Plus => Some(value.ln().multiply(Computable::integer(exp)).exp()),
            Sign::Minus => {
                // Take the power of the positive version and negate it afterwards.
                let value = value.negate();
                let odd = exp.bit(0);
                let exp = Computable::integer(exp);
                if odd {
                    Some(value.ln().multiply(exp).exp().negate())
                } else {
                    Some(value.ln().multiply(exp).exp())
                }
            }
        }
    }

    fn exp_ln_powi(self, exp: BigInt) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                // Unknown sign cannot safely use ln(base)*exp, so keep the exact
                // repeated-squaring fallback even though it may allocate more nodes.
                let power = Self::recursive_powi(&self, exp.magnitude());
                if exp.sign() == Sign::Minus {
                    power.inverse()
                } else {
                    Ok(power)
                }
            }
            Sign::Plus => {
                // Known-positive generic powers use exp(exp*ln(base)) to avoid a long
                // multiplication chain for large exponents.
                let value = self.fold();
                let exp = Computable::integer(exp);

                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Some(value.ln().multiply(exp).exp()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            Sign::Minus => {
                let odd = exp.bit(0);
                let value = self.fold();
                let exp = Computable::integer(exp);
                if odd {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: Some(value.ln().multiply(exp).exp().negate()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    })
                } else {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: Some(value.ln().multiply(exp).exp()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    })
                }
            }
        }
    }

    /// Raise this Real to some integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        if exp == *signed::ONE {
            crate::trace_dispatch!("real", "powi", "exponent-one");
            return Ok(self);
        }
        if exp.sign() == Sign::NoSign {
            if self.definitely_zero() {
                crate::trace_dispatch!("real", "powi", "zero-to-zero-domain-error");
                return Err(Problem::NotANumber);
            } else {
                crate::trace_dispatch!("real", "powi", "exponent-zero-one");
                return Ok(Self::one());
            }
        }
        if exp.sign() == Sign::Minus && self.definitely_zero() {
            crate::trace_dispatch!("real", "powi", "zero-negative-exponent-domain-error");
            return Err(Problem::NotANumber);
        }
        if exp == BigInt::from(-1_i8) {
            // The reciprocal path already knows about symbolic constant-product
            // classes (`Pi -> PiInv`, `e^x -> e^-x`, rationalized radicals).
            // Reuse those facts for x^-1 instead of constructing a generic
            // exp(ln(x) * -1) graph in the integer-power fallback.
            crate::trace_dispatch!("real", "powi", "negative-one-inverse");
            return self.inverse();
        }
        if let Ok(rational) = self.rational.clone().powi(exp.clone()) {
            match &self.class {
                One => {
                    // Pure rationals stay exact under integer powers.
                    crate::trace_dispatch!("real", "powi", "rational-exact");
                    return Ok(Self {
                        rational,
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                Sqrt(sqrt) => 'quick: {
                    // (a*sqrt(n))^k can peel off n^(k/2); this preserves exact sqrt
                    // structure for odd powers and collapses even powers to rationals.
                    let odd = exp.bit(0);
                    let Ok(rf2) = sqrt.clone().powi(exp.clone() >> 1) else {
                        break 'quick;
                    };
                    let product = rational * rf2;
                    if odd {
                        let n = Self {
                            rational: product,
                            class: Sqrt(sqrt.clone()),
                            computable: self.computable,
                            signal: None,
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        };
                        crate::trace_dispatch!("real", "powi", "sqrt-odd-special-form");
                        return Ok(n);
                    } else {
                        crate::trace_dispatch!("real", "powi", "sqrt-even-rational");
                        return Ok(Self::new(product));
                    }
                }
                _ => {
                    if let Some(computable) =
                        Self::compute_exp_ln_powi(self.computable_clone(), exp.clone())
                    {
                        // Reuse the exact rational scale while moving the irrational part
                        // to the cheaper exp(ln(x)*k) representation.
                        crate::trace_dispatch!("real", "powi", "irrational-exp-ln");
                        return Ok(Self {
                            rational,
                            class: Irrational,
                            computable: Some(computable),
                            signal: None,
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        });
                    }
                }
            }
        }
        crate::trace_dispatch!("real", "powi", "fallback-exp-ln-or-repeated-square");
        self.exp_ln_powi(exp)
    }

    /// Fractional (Non-integer) rational exponent.
    fn pow_fraction(self, exponent: Rational) -> Result<Self, Problem> {
        if exponent.denominator() == unsigned::TWO.deref() {
            // Half-integer powers are common enough to route through powi + sqrt, which
            // exposes exact-square simplifications.
            let n = exponent.shifted_big_integer(1);
            crate::trace_dispatch!("real", "pow", "half-integer-powi-sqrt");
            self.powi(n)?.sqrt()
        } else {
            crate::trace_dispatch!("real", "pow", "fractional-arbitrary");
            self.pow_arb(Real::new(exponent))
        }
    }

    /// Arbitrary, possibly irrational exponent.
    /// NB: Assumed not to be integer
    fn pow_arb(self, exponent: Self) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                if exponent.best_sign() == Sign::Plus {
                    crate::trace_dispatch!("real", "pow", "zero-positive-exponent");
                    Ok(Real::zero())
                } else {
                    crate::trace_dispatch!("real", "pow", "zero-nonpositive-domain-error");
                    Err(Problem::NotAnInteger)
                }
            }
            Sign::Minus => {
                crate::trace_dispatch!("real", "pow", "negative-arbitrary-domain-error");
                Err(Problem::NotAnInteger)
            }
            Sign::Plus => {
                let value = self.fold();
                let exp = exponent.fold();

                crate::trace_dispatch!("real", "pow", "positive-exp-ln");
                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Some(value.ln().multiply(exp).exp()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
        }
    }

    /// Raise this Real to an exact rational exponent.
    pub fn pow_rational(self, exponent: Rational) -> Result<Self, Problem> {
        if let Some(integer) = exponent.to_big_integer() {
            crate::trace_dispatch!("real", "pow_rational", "integer-exponent");
            return self.powi(integer);
        }

        if self.best_sign() == Sign::Minus && exponent.denominator().bit(0) {
            let Some(denominator) = exponent.denominator().to_u32() else {
                crate::trace_dispatch!("real", "pow_rational", "odd-denominator-exhausted");
                return Err(Problem::Exhausted);
            };
            let numerator = BigInt::from_biguint(exponent.sign(), exponent.numerator().clone());
            crate::trace_dispatch!("real", "pow_rational", "negative-odd-denominator-root");
            return self.root_n(denominator)?.powi(numerator);
        }

        crate::trace_dispatch!("real", "pow_rational", "generic-rational-exponent");
        self.pow(Real::new(exponent))
    }

    /// Raise this Real to some Real exponent.
    pub fn pow(self, exponent: Self) -> Result<Self, Problem> {
        if let Exp(ref n) = self.class
            && n == rationals::ONE.deref()
        {
            if self.rational.is_one() {
                // e^x with unit scale is just exp(x), preserving the symbolic exp path.
                crate::trace_dispatch!("real", "pow", "e-base-exp");
                return exponent.exp();
            } else {
                // (a*e)^x = a^x * e^x keeps the e^x part symbolic.
                let left = Real::new(self.rational).pow(exponent.clone())?;
                crate::trace_dispatch!("real", "pow", "scaled-e-base-split");
                return Ok(left * exponent.exp()?);
            }
        }
        /* could handle self == 10 =>  10 ^ log10(exponent) specially */
        if exponent.class == One {
            let r = exponent.rational;
            if r.is_integer() {
                if let Some(n) = r.to_integer_i64() {
                    // Small integer exponents are a structural fact, not an
                    // approximation. Dispatching them before materializing the
                    // full BigInt avoids cloning arbitrary-precision storage on
                    // the common pow(x, 2/3/17) path while preserving exact
                    // repeated-squaring semantics; see Boehm et al.,
                    // "Exact Real Arithmetic: A Case Study in Higher Order
                    // Programming" (1986) on keeping exact symbolic structure
                    // ahead of numeric refinement.
                    crate::trace_dispatch!("real", "pow", "small-integer-exponent");
                    return self.powi(BigInt::from(n));
                }
                if let Some(n) = r.to_big_integer() {
                    crate::trace_dispatch!("real", "pow", "integer-exponent");
                    return self.powi(n);
                }
            }
            crate::trace_dispatch!("real", "pow", "rational-exponent");
            return self.pow_fraction(r);
        }
        if exponent.definitely_zero() {
            crate::trace_dispatch!("real", "pow", "zero-exponent");
            return self.powi(BigInt::ZERO);
        }
        crate::trace_dispatch!("real", "pow", "arbitrary-exponent");
        self.pow_arb(exponent)
    }

    /// Is this Real an integer ?
    pub fn is_integer(&self) -> bool {
        self.class == One && self.rational.is_integer()
    }

    /// Is this Real known to be rational ?
    pub fn is_rational(&self) -> bool {
        self.class == One
    }

    /// Should we display this Real as a fraction ?
    pub fn prefer_fraction(&self) -> bool {
        self.class == One && self.rational.prefer_fraction()
    }
}
