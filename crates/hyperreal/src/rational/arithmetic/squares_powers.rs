impl Rational {
    const EXTRACT_SQUARE_MAX_LEN: u64 = 5000;
    const SMALL_SQUARE_FACTORS: [(u64, u64); 7] = [
        (2, 4),
        (3, 9),
        (5, 25),
        (7, 49),
        (11, 121),
        (13, 169),
        (17, 289),
    ];
    const SMALL_SQUARE_PRODUCT: u64 = 260_620_460_100;

    #[inline]
    fn could_be_perfect_square(n: &BigUint) -> bool {
        // Every integer square is one of twelve residues modulo 64. This cheap
        // screen rejects 81.25% of arbitrary candidates before the
        // substantially more expensive arbitrary-precision square root.
        let residue_64 = (n % 64_u8)
            .to_u8()
            .expect("a residue modulo 64 fits in u8");
        if !matches!(
            residue_64,
            0 | 1 | 4 | 9 | 16 | 17 | 25 | 33 | 36 | 41 | 49 | 57
        ) {
            crate::trace_dispatch!("rational", "square_extraction", "mod64-reject");
            return false;
        }

        // Modulo 63 supplies an independent odd-factor screen. This remains a
        // proof-only rejection: candidates reaching `sqrt` may be nonsquares,
        // but no actual integer square can be discarded here.
        let residue_63 = (n % 63_u8)
            .to_u8()
            .expect("a residue modulo 63 fits in u8");
        let possible = matches!(
            residue_63,
            0 | 1 | 4 | 7 | 9 | 16 | 18 | 22 | 25 | 28 | 36 | 37 | 43 | 46 | 49 | 58
        );
        if !possible {
            crate::trace_dispatch!("rational", "square_extraction", "mod63-reject");
        }
        possible
    }

    // Some(root) squared is n, otherwise None
    fn try_perfect(n: &BigUint) -> Option<BigUint> {
        if !Self::could_be_perfect_square(n) {
            return None;
        }
        // BigUint::sqrt is cheap enough as a final check once small square
        // factors have been stripped.
        let root = n.sqrt();
        let square = &root * &root;
        if square == *n { Some(root) } else { None }
    }

    fn extract_square_u64(mut rest: u64) -> (u64, u64) {
        let mut root = 1_u64;
        for (prime, square) in [(2, 4), (3, 9), (5, 25), (7, 49), (11, 121), (13, 169), (17, 289)] {
            if rest == 1 {
                break;
            }
            while rest.is_multiple_of(square) {
                rest /= square;
                root *= prime;
            }
        }

        let divisors = if rest & 1 == 1 {
            [1_u64, 3, 5, 7, 11, 13, 15, 17, 19]
        } else {
            [1_u64, 2, 3, 5, 6, 7, 8, 10, 11]
        };
        for divisor in divisors {
            if rest == divisor {
                return (root, rest);
            }
            if rest.is_multiple_of(divisor) {
                let square = rest / divisor;
                let factor = square.isqrt();
                if factor * factor == square {
                    return (root * factor, divisor);
                }
            }
        }
        (root, rest)
    }

    // (root squared times rest) = n
    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        if let Some(small) = n.to_u64() {
            let (root, rest) = Self::extract_square_u64(small);
            if root == 1 && rest == small {
                return (BigUint::one(), n);
            }
            return (BigUint::from(root), BigUint::from(rest));
        }

        // Exact f64 imports and fixed-grid geometry produce large dyadic
        // denominators. A power of two needs no trial division: split its
        // exponent into a square root and at most one residual factor of two.
        let exponent = n.bits() - 1;
        if n.trailing_zeros() == Some(exponent) {
            crate::trace_dispatch!("rational", "square_extraction", "large-power-of-two");
            let root = BigUint::one()
                << usize::try_from(exponent / 2).expect("BigUint exponent fits usize");
            let rest = if exponent.is_multiple_of(2) {
                BigUint::one()
            } else {
                BigUint::from(2_u8)
            };
            return (root, rest);
        }

        // Partial square extraction is a performance shortcut, not full prime
        // factorization. It peels common small squares and then tests a few
        // residual divisors so sqrt simplification remains bounded.
        let one: BigUint = One::one();
        let mut root = one.clone();
        let mut rest = n;
        if rest.bits() > Self::EXTRACT_SQUARE_MAX_LEN {
            return (root, rest);
        }
        // All small prime squares are pairwise coprime, so one word-sized
        // remainder identifies every factor worth probing. Most arbitrary
        // numerators now avoid seven separate BigUint remainder operations.
        let small_square_residue = (&rest % Self::SMALL_SQUARE_PRODUCT)
            .to_u64()
            .expect("a residue modulo the small-square product fits in u64");
        crate::trace_dispatch!("rational", "square_extraction", "shared-small-factor-remainder");
        for (prime, square) in Self::SMALL_SQUARE_FACTORS {
            if rest == one {
                break;
            }
            if !small_square_residue.is_multiple_of(square) {
                continue;
            }
            let square = BigUint::from(square);
            while (&rest % &square).is_zero() {
                rest /= &square;
                root *= prime;
            }
        }

        let (divisors, divisor_lcm) = if rest.bit(0) {
            // Odd number so dividing by an even number won't get a whole result
            // `u8` covers this fixed probe table and converts straight to BigUint.
            ([1_u64, 3, 5, 7, 11, 13, 15, 17, 19], 4_849_845_u64)
        } else {
            ([1_u64, 2, 3, 5, 6, 7, 8, 10, 11], 9_240_u64)
        };
        // As above, one remainder answers all fixed small-divisor probes.
        let divisor_residue = (&rest % divisor_lcm)
            .to_u64()
            .expect("a residue modulo the divisor LCM fits in u64");
        crate::trace_dispatch!("rational", "square_extraction", "shared-divisor-remainder");

        for divisor_word in divisors {
            let divisor = BigUint::from(divisor_word);
            if rest == divisor {
                return (root, rest);
            }
            if divisor_residue.is_multiple_of(divisor_word) {
                let square = if divisor_word == 1 {
                    &rest
                } else {
                    // The owned quotient is needed only for the comparatively
                    // rare divisor that survives the shared remainder screen.
                    let quotient = &rest / &divisor;
                    if let Some(factor) = Self::try_perfect(&quotient) {
                        return (root * factor, divisor);
                    }
                    continue;
                };
                if let Some(factor) = Self::try_perfect(square) {
                    return (root * factor, divisor);
                }
            }
        }
        (root, rest)
    }

    /// For a value n, the result of this function is a pair (a, b)
    /// such that a * a * b = n.
    ///
    /// Where b is zero, a is the exact square root of n
    /// Otherwise, b is a residual for which no exact rational square root exists.
    pub fn extract_square_reduced(self) -> (Self, Self) {
        if self.sign == NoSign {
            return (Self::zero(), Self::zero());
        }
        let (sign, numerator, denominator) = self.into_parts();
        let (nroot, nrest) = Self::extract_square(numerator);
        let (droot, drest) = Self::extract_square(denominator);
        (
            Self::from_parts_raw(Plus, nroot, droot),
            Self::from_parts_raw(sign, nrest, drest),
        )
    }

    #[inline]
    fn retained_square_reduction(&self) -> Option<(Self, Self)> {
        let reduction = self
            .linear_cache
            .get()?
            .square_reduction
            .get()?;
        crate::trace_dispatch!("rational", "square_extraction", "retained-reduction");
        Some((reduction.square.clone(), reduction.rest.clone()))
    }

    #[cold]
    fn retain_square_reduction(&self, square: &Self, rest: &Self) {
        let reduction = CachedRationalSquareReduction {
            square: square.clone(),
            rest: rest.clone(),
        };
        if let Some(cached) = self.linear_cache.get() {
            let _ = cached.square_reduction.set(reduction);
            return;
        }

        let square_reduction = OnceLock::new();
        let _ = square_reduction.set(reduction);
        let _ = self
            .linear_cache
            .set(Box::new(CachedRationalArithmetic {
                primary: CachedRationalLinearEntry {
                    other: std::sync::Weak::new(),
                    kind: CachedRationalLinearKind::SquareReductionPlaceholder,
                    result: RATIONAL_ZERO.clone(),
                },
                secondary: OnceLock::new(),
                tertiary: OnceLock::new(),
                quaternary: OnceLock::new(),
                quinary: OnceLock::new(),
                square_reduction,
            }));
    }

    pub(crate) fn extract_square_reduced_retained(self) -> (Self, Self) {
        if let Some(reduction) = self.retained_square_reduction() {
            return reduction;
        }
        if !self
            .square_reuse_seen
            .load(std::sync::atomic::Ordering::Relaxed)
        {
            crate::trace_dispatch!("rational", "square_extraction", "reuse-observed");
            self.square_reuse_seen
                .store(true, std::sync::atomic::Ordering::Relaxed);
            return self.extract_square_reduced();
        }

        let reduction = self.clone().extract_square_reduced();
        self.retain_square_reduction(&reduction.0, &reduction.1);
        reduction
    }

    /// For very big rationals, the algorithm used for calculating a square
    /// root is not viable, in this case the predicate is false.
    pub fn extract_square_will_succeed(&self) -> bool {
        self.numerator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
            && self.denominator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
    }

    /// Return the exact nth root when both numerator and denominator are
    /// perfect nth powers. Negative values are supported for odd `n`.
    pub fn perfect_nth_root(&self, n: u32) -> Option<Self> {
        if n == 0 {
            return None;
        }
        if self.sign == NoSign {
            return Some(Self::zero());
        }
        if self.sign == Minus && n.is_multiple_of(2) {
            return None;
        }

        let numerator = self.numerator.nth_root(n);
        if numerator.pow(n) != self.numerator {
            return None;
        }

        let denominator = self.denominator.nth_root(n);
        if denominator.pow(n) != self.denominator {
            return None;
        }

        Some(Self::from_parts_raw(self.sign, numerator, denominator))
    }

    #[inline]
    fn pow_up_u64(&self, exp: u64) -> Self {
        if exp == 0 {
            return Self::one();
        }
        // A first call keeps the direct integer-power kernel below. If the same
        // immutable base is powered again, repeated squaring deliberately routes
        // through borrowed multiplication: each edge then reuses the bounded
        // product cache on subsequent calls without adding a power-specific
        // result cache or enlarging RationalData.
        if (2..=5).contains(&exp) {
            if self
                .power_reuse_seen
                .load(std::sync::atomic::Ordering::Relaxed)
            {
                crate::trace_dispatch!("rational", "powi", "retained-product-chain");
                return self.pow_up_by_retained_multiplication(exp);
            }
            // A racing first call may also use the direct path. Both results
            // are exact, and either call supplies reuse evidence for the next.
            self.power_reuse_seen
                .store(true, std::sync::atomic::Ordering::Relaxed);
        }
        if let Ok(exp) = u32::try_from(exp)
            && exp <= 64
        {
            if let (Some(numerator), Some(denominator)) =
                (self.numerator.to_u128(), self.denominator.to_u128())
                && let (Some(numerator), Some(denominator)) =
                    (numerator.checked_pow(exp), denominator.checked_pow(exp))
            {
                crate::trace_dispatch!("rational", "powi", "word-sized");
                return Self::from_reduced_word_parts(
                    if self.sign == Minus && exp % 2 == 1 {
                        Minus
                    } else {
                        Plus
                    },
                    numerator,
                    denominator,
                );
            }

            let denominator = self
                .dyadic_denominator_shift()
                .and_then(|shift| shift.checked_mul(u64::from(exp)))
                .and_then(|shift| usize::try_from(shift).ok())
                .map(|shift| {
                    crate::trace_dispatch!("rational", "powi", "dyadic-denominator-shift");
                    BigUint::one() << shift
                })
                .unwrap_or_else(|| self.denominator.pow(exp));
            return Self::from_parts_raw(
                if self.sign == Minus && exp % 2 == 1 {
                    Minus
                } else {
                    Plus
                },
                self.numerator.pow(exp),
                denominator,
            );
        }

        self.pow_up_by_multiplication(exp)
    }

    fn pow_up_by_retained_multiplication(&self, mut exp: u64) -> Self {
        match exp {
            2 => return self * self,
            3 => {
                let square = self * self;
                return &square * self;
            }
            4 => {
                let square = self * self;
                return &square * &square;
            }
            5 => {
                let square = self * self;
                let fourth = &square * &square;
                return &fourth * self;
            }
            _ => {}
        }
        let mut result = Self::one();
        let mut factor = self.clone();
        while exp > 0 {
            if exp & 1 == 1 {
                // The successively squared factor owns the useful retained
                // product edge. Keep it on the left so the common hit needs
                // one cache probe rather than a failed commutative fallback.
                result = &factor * &result;
            }
            exp >>= 1;
            if exp > 0 {
                factor = &factor * &factor;
            }
        }
        result
    }

    fn pow_up_by_multiplication(&self, mut exp: u64) -> Self {
        let mut result = Self::one();
        let mut factor = self.clone();
        while exp > 0 {
            if exp & 1 == 1 {
                result *= &factor;
            }
            exp >>= 1;
            if exp > 0 {
                factor = &factor * &factor;
            }
        }
        result
    }

    // This could grow unreasonably in terms of object size
    // so only call this for modest exp values.
    fn pow_up(&self, exp: &BigUint) -> Self {
        if let Some(exp) = exp.to_u64() {
            return self.pow_up_u64(exp);
        }

        let mut result = Self::one();
        let mut factor = self.clone();
        let bits = exp.bits();
        for b in 0..bits {
            if exp.bit(b) {
                result *= &factor;
            }
            if b + 1 < bits {
                factor = &factor * &factor;
            }
        }
        result
    }

    #[inline]
    pub(crate) fn powi_i64(self, exp: i64) -> Result<Self, Problem> {
        if exp == 0 {
            return Ok(Self::one());
        }
        if self.sign == NoSign {
            return if exp < 0 {
                Err(Problem::DivideByZero)
            } else {
                Ok(Self::zero())
            };
        }
        if self.is_integer() && self.numerator == *ONE.deref() {
            return if self.sign == Minus && exp & 1 != 0 {
                Ok(Self::new(-1))
            } else {
                Ok(Self::one())
            };
        }

        if exp < 0 {
            Ok(self.inverse()?.pow_up_u64(exp.unsigned_abs()))
        } else {
            Ok(self.pow_up_u64(exp as u64))
        }
    }

    /// Integer exponentiation. Raise this Rational to an integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        const TOO_MANY_BITS: u64 = 1000;
        if exp == BigInt::ZERO {
            return Ok(Self::one());
        }
        if self.sign == NoSign {
            return if exp.sign() == Minus {
                Err(Problem::DivideByZero)
            } else {
                Ok(Self::zero())
            };
        }
        // Plus or minus one exactly
        if self.is_integer() && self.numerator == *ONE.deref() {
            if self.sign == Minus && exp.bit(0) {
                return Ok(Self::new(-1));
            } else {
                return Ok(Self::one());
            }
        }
        if exp.bits() >= TOO_MANY_BITS {
            return Err(Problem::Exhausted);
        }
        match exp.sign() {
            Minus => Ok(self.inverse()?.pow_up(exp.magnitude())),
            Plus => Ok(self.pow_up(exp.magnitude())),
            NoSign => unreachable!(),
        }
    }
}
