impl Real {
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
                if self.rational.is_integer() {
                    crate::trace_dispatch!("real", "tan", "pi-integer-zero");
                    return Ok(Self::zero());
                }
                // Rational multiples of pi get exact tangent values for the usual small
                // denominators, otherwise a compact TanPi certificate.
                let (neg, n) = tan_curve(self.rational);
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
                    if neg {
                        return Ok(real.neg());
                    } else {
                        return Ok(real);
                    }
                } else {
                    let new =
                        Computable::multiply(Computable::pi(), Computable::rational(n.clone()));
                    let computable = Computable::prescaled_tan(new);
                    crate::trace_dispatch!("real", "tan", "tanpi-special-form");
                    if neg {
                        return Ok(Self {
                            rational: Rational::new(-1),
                            class: TanPi(n),
                            computable: Some(computable),
                            signal: None,
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        });
                    } else {
                        return Ok(Self {
                            rational: Rational::one(),
                            class: TanPi(n),
                            computable: Some(computable),
                            signal: None,
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        });
                    }
                }
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

