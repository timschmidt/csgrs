/// Exact rational root recognition, partial square reduction and integer powers.
///
/// These kernels preserve rational values and supply symbolic `Real` shortcuts.
/// Partial extraction is not a squarefree normal-form oracle; eager-power size
/// admission is not a total resource contract. Cache reuse keeps constructed
/// factor/product relations on immutable owners with exact value semantics.
impl Rational {
    /// Bit-amplification threshold used only for integer exponents above 255.
    ///
    /// The guard compares `(numerator.bits() + denominator.bits()) * exponent`
    /// with 65,536, using division to avoid multiplication overflow. Smaller
    /// exponents bypass this threshold, so it is not an absolute output-size,
    /// allocation, work, or cancellation budget. `Real` can handle the resulting
    /// [`Problem::Exhausted`] by retaining an exact computable expression.
    const MAX_EAGER_POWER_OUTPUT_BITS: u64 = 1 << 16;

    /// Apply the eager-power size heuristic to an unsigned machine exponent.
    ///
    /// Every exponent through 255 passes, including zero without division.
    /// Larger exponents use the saturating sum of the stored magnitude widths;
    /// this helper neither reduces the operand nor accounts for intermediate
    /// allocations. Public power entry points handle zero and units first.
    fn eager_power_u64_fits_budget(&self, exp: u64) -> bool {
        exp <= u64::from(u8::MAX)
            || self
                .numerator
                .bits()
                .saturating_add(self.denominator.bits())
                <= Self::MAX_EAGER_POWER_OUTPUT_BITS / exp
    }

    /// Apply the same eager-power guard to an arbitrary-width exponent.
    ///
    /// An exponent of at most eight bits passes immediately. Wider values must
    /// fit `u64` and pass the machine-exponent guard; larger values return false.
    /// Zero and unit bases are handled by the caller before this test.
    fn eager_power_fits_budget(&self, exp: &BigUint) -> bool {
        // Modest exponents have bounded amplification relative to the input
        // rational and remain on the established fast path.
        if exp.bits() <= u64::from(u8::BITS) {
            return true;
        }
        exp.to_u64()
            .is_some_and(|exp| self.eager_power_u64_fits_budget(exp))
    }

    /// Maximum magnitude width admitted to general partial square extraction.
    ///
    /// The extraction kernel processes up to 5,000 bits inclusive after its
    /// word and power-of-two shortcuts. The public viability hint instead uses
    /// strictly less than this value for both stored rational components.
    const EXTRACT_SQUARE_MAX_LEN: u64 = 5000;
    /// Prime/square pairs removed repeatedly before residual root probes.
    ///
    /// The seven primes from 2 through 17 provide cheap common-factor cleanup,
    /// not full factorization. Pairwise coprime squares permit one shared
    /// remainder to identify every applicable entry.
    const SMALL_SQUARE_FACTORS: [(u64, u64); 7] = [
        (2, 4),
        (3, 9),
        (5, 25),
        (7, 49),
        (11, 121),
        (13, 169),
        (17, 289),
    ];
    /// Product of the seven prime squares used by the shared remainder screen.
    ///
    /// Each table square divides this positive `u64`, so the remainder modulo
    /// this product preserves divisibility by every table entry.
    const SMALL_SQUARE_PRODUCT: u64 = 260_620_460_100;
    /// Odd primes through 97 used to constrain possible power exponents.
    ///
    /// Powers of two are handled separately by trailing-zero counting. Missing
    /// larger factors do not certify a candidate exponent; exact root tests
    /// still decide the surviving candidates.
    const PERFECT_POWER_TRIAL_PRIMES: [u32; 24] = [
        3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79,
        83, 89, 97,
    ];

    /// Reject magnitudes whose residues cannot belong to an integer square.
    ///
    /// All nonnegative inputs, including zero, are admitted. The modulo-64
    /// screen runs before modulo 63; each failed screen emits its own trace.
    /// True means only that both necessary conditions passed, not that a root
    /// exists. Neither screen uses floating-point approximation.
    #[inline]
    fn could_be_perfect_square(n: &BigUint) -> bool {
        // Every integer square is one of twelve residues modulo 64. This cheap
        // screen rejects 81.25% of residue classes modulo 64 before the
        // arbitrary-precision square root; workload rejection rates vary.
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

    /// Return the nonnegative integer square root exactly when it squares to `n`.
    ///
    /// Residue screens reject impossible inputs first. Survivors use the
    /// backend integer square root and an exact multiplication check; zero
    /// returns zero. The `newton-square-root` trace labels this dispatch, not
    /// proof that the backend used Newton iteration for this particular input.
    /// There is no magnitude limit or caller-supplied work budget here.
    fn try_perfect(n: &BigUint) -> Option<BigUint> {
        if !Self::could_be_perfect_square(n) {
            return None;
        }
        // The backend integer square root supplies a candidate after the
        // necessary residue screens; multiplication below verifies it exactly.
        crate::trace_dispatch!("rational_algorithm", "root-extraction", "newton-square-root");
        let root = n.sqrt();
        let square = &root * &root;
        if square == *n { Some(root) } else { None }
    }

    /// Extract selected square factors from a strictly positive word magnitude.
    ///
    /// Returns positive `(root, rest)` with `root * root * rest` equal to the
    /// input. It strips prime squares through 17, then tests fixed odd/even
    /// residual divisors using exact integer roots. The residual need not be
    /// squarefree: `19² * 23` is returned unchanged. Intermediate products fit
    /// because the extracted square divides the positive original magnitude.
    ///
    /// Zero violates the private precondition: division never reduces it and
    /// the root multiplication overflows (or loops with overflow checks off).
    /// The public rational entry point handles zero before calling this helper.
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

    /// Partially factor a positive arbitrary-width magnitude as `root² * rest`.
    ///
    /// Word inputs use the word kernel. Larger powers of two split the exponent
    /// exactly, even above the general 5,000-bit cutoff; all other inputs above
    /// that cutoff return `(1, n)`. Remaining inputs strip selected small squares
    /// and test a fixed list of residual divisors with exact square checks.
    /// Shared remainders avoid repeated divisibility calculations.
    ///
    /// Both outputs are positive, but `rest` need not be squarefree or even a
    /// nonsquare. Zero is outside this helper's domain. The bit cutoff bounds
    /// only the general factoring path, not total allocation, cache memory or
    /// cancellation; converting a power-of-two shift to `usize` assumes a
    /// representable allocation.
    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        if let Some(small) = n.to_u64() {
            let (root, rest) = Self::extract_square_u64(small);
            if root == 1 && rest == small {
                return (BigUint::one(), n);
            }
            return (BigUint::from(root), BigUint::from(rest));
        }

        // Exact binary64-derived dyadics and fixed-grid geometry produce large dyadic
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
        // residual divisors to limit factoring work on this size-gated path.
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

    /// Extract rational square factors while preserving `self = a² * b`.
    ///
    /// Returns a nonnegative rational `a` and an integer `b` with the original
    /// sign. Zero returns `(0, 0)`; a fully extracted positive square has `b = 1`.
    /// The residual denominator is rationalized, so `1/2` becomes `(1/2, 2)`.
    /// Negative inputs are valid factorizations, not real square-root results.
    /// Internally unreduced values are canonicalized before extraction.
    ///
    /// Extraction is partial: it strips selected small factors and tests a
    /// fixed set of residual divisors. Non-power-of-two magnitudes above 5,000
    /// bits are left unchanged. Consequently `b` can retain square factors or
    /// be a perfect square, and equivalent surds need not receive a common
    /// normal form. There is no typed exhaustion or total work/memory budget.
    ///
    /// ```
    /// use hyperreal::Rational;
    /// assert_eq!(Rational::new(4).extract_square_reduced(),
    ///            (Rational::new(2), Rational::one()));
    /// assert_eq!(Rational::fraction(1, 2).unwrap().extract_square_reduced(),
    ///            (Rational::fraction(1, 2).unwrap(), Rational::new(2)));
    /// assert_eq!(Rational::zero().extract_square_reduced(),
    ///            (Rational::zero(), Rational::zero()));
    /// ```
    pub fn extract_square_reduced(self) -> (Self, Self) {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().clone().extract_square_reduced();
        }
        if self.sign == NoSign {
            return (Self::zero(), Self::zero());
        }
        let (sign, numerator, denominator) = self.into_parts();
        let (nroot, nrest) = Self::extract_square(numerator);
        let (droot, drest) = Self::extract_square(denominator);
        // Rationalize the residual denominator to align common quadratic surds
        // without claiming a complete squarefree normal form:
        //
        //   sqrt(nrest / drest) = sqrt(nrest * drest) / drest.
        //
        // Thus sqrt(1/2) becomes sqrt(2)/2 and can cancel structurally with
        // independently authored sqrt(2) terms.
        (
            Self::from_parts_raw(Plus, nroot, &droot * &drest),
            Self::from_parts_raw(sign, nrest * drest, BigUint::one()),
        )
    }

    /// Clone an already published square-factor pair from this immutable owner.
    ///
    /// A missing arithmetic cache or reduction slot is a miss. A hit emits the
    /// retained-reduction trace and trusts the installing caller's exact
    /// `self = square² * rest` relation; it performs no new factorization or
    /// validation. Cloned handles keep the retained magnitudes alive.
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

    /// Attempt to publish an exact square-factor pair for later reuse.
    ///
    /// The caller must supply this owner's reduction and preserve acyclic
    /// strong ownership; the relation is not checked here. The pair is strongly
    /// retained in the existing arithmetic cache, or in a new cache whose
    /// primary linear entry is an inert square-reduction placeholder.
    ///
    /// Occupied slots and lost installation races leave existing evidence
    /// untouched; failure to install the outer cache is not retried. Losing
    /// candidate handles are dropped. Fixed slot counts do not bound the byte
    /// size of retained factors, and this helper reports no allocation budget.
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

    /// Reuse partial square extraction after repeated requests on one owner.
    ///
    /// A published pair wins immediately. Otherwise the first observation sets
    /// the reuse bit and extracts without retaining a pair; later observations
    /// extract from a clone and attempt publication. Competing calls may repeat
    /// work or lose publication without changing the exact factorization.
    ///
    /// Canonicalization, partial-factor limits and zero/negative behavior are
    /// those of [`Self::extract_square_reduced`]. Clones share the observation
    /// and cache; equal independently allocated values need not share them.
    pub(crate) fn extract_square_reduced_retained(self) -> (Self, Self) {
        if let Some(reduction) = self.retained_square_reduction() {
            return reduction;
        }
        if !self.observe_retained_fact(RETAINED_SQUARE_REUSE_SEEN) {
            crate::trace_dispatch!("rational", "square_extraction", "reuse-observed");
            return self.extract_square_reduced();
        }

        let reduction = self.clone().extract_square_reduced();
        self.retain_square_reduction(&reduction.0, &reduction.1);
        reduction
    }

    /// Return a conservative size hint for attempting symbolic square extraction.
    ///
    /// True means both stored magnitude widths are strictly below 5,000 bits.
    /// This reads the original representation without canonicalizing it. It
    /// neither proves complete factorization nor certifies a squarefree residual.
    /// False does not imply extraction fails: the kernel handles exactly 5,000
    /// bits and has an unrestricted power-of-two shortcut. `Real` uses this
    /// hint to choose whether to attempt its symbolic square-root path.
    pub fn extract_square_will_succeed(&self) -> bool {
        self.numerator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
            && self.denominator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
    }

    /// Return an exact rational real `n`th root, if one exists and `n > 0`.
    ///
    /// Both reduced magnitudes must be perfect `n`th powers. Negative inputs
    /// admit only odd degrees; even roots use the nonnegative choice. Degree
    /// zero, a negative even root, and a nonperfect magnitude all return `None`.
    /// Positive-degree roots of zero are zero, and degree one returns the value.
    /// Internally unreduced inputs are canonicalized before these checks.
    ///
    /// Backend integer roots are verified by exact powering. One
    /// `newton-nth-root` dispatch trace precedes the numerator/denominator work;
    /// it does not count iterations or guarantee a backend algorithm. There is
    /// no input-size cutoff, cancellation hook or typed resource exhaustion.
    ///
    /// ```
    /// use hyperreal::Rational;
    /// assert_eq!(Rational::fraction(-8, 27).unwrap().perfect_nth_root(3),
    ///            Some(Rational::fraction(-2, 3).unwrap()));
    /// assert_eq!(Rational::new(2).perfect_nth_root(2), None);
    /// assert_eq!(Rational::new(-4).perfect_nth_root(2), None);
    /// assert_eq!(Rational::new(4).perfect_nth_root(0), None);
    /// ```
    pub fn perfect_nth_root(&self, n: u32) -> Option<Self> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().perfect_nth_root(n);
        }
        if n == 0 {
            return None;
        }
        if self.sign == NoSign {
            return Some(Self::zero());
        }
        if self.sign == Minus && n.is_multiple_of(2) {
            return None;
        }

        crate::trace_dispatch!("rational_algorithm", "root-extraction", "newton-nth-root");
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

    /// Intersect candidate-exponent constraints from selected prime valuations.
    ///
    /// For a positive magnitude, merge the multiplicities of two and odd primes
    /// through 97 into `exponent_gcd`. Zero in the accumulator means no factor
    /// has constrained it yet; one proves no exponent greater than one can
    /// work. The caller shares the accumulator across numerator and denominator.
    /// Unexamined factors can impose further constraints, so surviving values
    /// still require exact root checks. One contributes no constraint.
    ///
    /// This updates a local accumulator, not a retained object cache. It clones
    /// the magnitude and has no work budget. Zero magnitude violates the private
    /// precondition and can remain indefinitely in the divisibility loop.
    fn retain_perfect_power_factor_exponents(value: &BigUint, exponent_gcd: &mut u64) {
        if value.is_one() {
            return;
        }
        let mut rest = value.clone();
        let twos = rest.trailing_zeros().unwrap_or(0);
        if twos > 0 {
            *exponent_gcd = if *exponent_gcd == 0 {
                twos
            } else {
                Self::gcd_u64(*exponent_gcd, twos)
            };
            rest >>= usize::try_from(twos).expect("BigUint shift fits usize");
        }

        for prime in Self::PERFECT_POWER_TRIAL_PRIMES {
            if rest.is_one() || *exponent_gcd == 1 {
                break;
            }
            let prime = BigUint::from(prime);
            let mut multiplicity = 0_u64;
            while (&rest % &prime).is_zero() {
                rest /= &prime;
                multiplicity += 1;
            }
            if multiplicity > 0 {
                *exponent_gcd = if *exponent_gcd == 0 {
                    multiplicity
                } else {
                    Self::gcd_u64(*exponent_gcd, multiplicity)
                };
            }
        }
    }

    /// Decide primality for a `u32` candidate by exact trial division.
    ///
    /// Values below two fail; two passes, and other evens fail. Odd divisors are
    /// tested through the square root using `divisor <= candidate / divisor`
    /// to avoid multiplication overflow. No probabilistic test or explicit
    /// work/cancellation budget participates.
    fn is_prime_exponent(candidate: u32) -> bool {
        if candidate < 2 {
            return false;
        }
        if candidate.is_multiple_of(2) {
            return candidate == 2;
        }
        let mut divisor = 3_u32;
        while divisor <= candidate / divisor {
            if candidate.is_multiple_of(divisor) {
                return false;
            }
            divisor += 2;
        }
        true
    }

    /// Check whether a magnitude is an exact power of a nonnegative integer.
    ///
    /// The exponent must be positive; perfect-power callers supply primes at
    /// least two. One returns true immediately. Other values use an integer
    /// root followed by exact powering, with no resource guard. Degree zero
    /// violates the precondition: it panics in the backend except for the
    /// one shortcut, which returns true without consulting the degree.
    fn magnitude_is_exact_power(value: &BigUint, exponent: u32) -> bool {
        if value.is_one() {
            return true;
        }
        let root = value.nth_root(exponent);
        root.pow(exponent) == *value
    }

    /// Search for a rational base and integer exponent greater than one.
    ///
    /// For a reduced rational, numerator and denominator must share a valid
    /// exponent. Zero and both units return true by convention. Negative inputs
    /// use only odd exponents. Testing prime exponents suffices because every
    /// composite power is also a power at a prime divisor of its exponent.
    /// Internally unreduced inputs are canonicalized first.
    ///
    /// Small-prime multiplicities reject impossible cases or constrain the
    /// candidate exponents. Each survivor requires exact integer roots of both
    /// magnitudes. The search bound is the smaller bit length minus one among
    /// nonunit components, capped at `u32::MAX` for the backend root interface.
    ///
    /// That cap is an implementation limitation: this Boolean result cannot
    /// report an untested exponent beyond it, and a negative result is not a
    /// universal certificate for arbitrarily large inputs requiring such a
    /// prime exponent. There is no total-work, allocation or cancellation budget;
    /// the method does not report exhausted resources separately.
    pub fn is_perfect_power(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_perfect_power();
        }
        if self.sign == NoSign || (self.numerator.is_one() && self.denominator.is_one()) {
            crate::trace_dispatch!("rational_algorithm", "perfect-power", "unit");
            return true;
        }

        let mut exponent_gcd = 0_u64;
        Self::retain_perfect_power_factor_exponents(&self.numerator, &mut exponent_gcd);
        Self::retain_perfect_power_factor_exponents(&self.denominator, &mut exponent_gcd);
        if exponent_gcd == 1 {
            crate::trace_dispatch!(
                "rational_algorithm",
                "perfect-power",
                "factor-multiplicity-reject"
            );
            return false;
        }

        let max_exponent = [&self.numerator, &self.denominator]
            .into_iter()
            .filter(|value| !value.is_one())
            .map(|value| value.bits().saturating_sub(1))
            .min()
            .unwrap_or(2)
            .min(u64::from(u32::MAX)) as u32;

        for exponent in 2..=max_exponent {
            if !Self::is_prime_exponent(exponent)
                || (self.sign == Minus && exponent.is_multiple_of(2))
                || (exponent_gcd > 0 && !exponent_gcd.is_multiple_of(u64::from(exponent)))
            {
                continue;
            }
            crate::trace_dispatch!(
                "rational_algorithm",
                "perfect-power",
                "prime-root-candidate"
            );
            if Self::magnitude_is_exact_power(&self.numerator, exponent)
                && Self::magnitude_is_exact_power(&self.denominator, exponent)
            {
                crate::trace_dispatch!(
                    "rational_algorithm",
                    "perfect-power",
                    "prime-root-exact"
                );
                return true;
            }
        }
        crate::trace_dispatch!(
            "rational_algorithm",
            "perfect-power",
            "prime-root-reject"
        );
        false
    }

    /// Materialize a nonnegative machine power of a nonzero reduced rational.
    ///
    /// Callers handle zero/unit shortcuts and enforce the eager-power guard.
    /// Zero exponent returns one. Repeated requests for exponents two through
    /// five use retained borrowed-product chains; the first observation instead
    /// uses the direct kernel. Through exponent 64, checked `u128` powers win
    /// when both fit, otherwise backend powers run with a dyadic-denominator
    /// shift shortcut. Larger exponents use binary repeated squaring.
    ///
    /// Sign follows exponent parity. The helper neither enforces the outer
    /// size guard nor turns allocation failure into a typed outcome; retained
    /// products and intermediate magnitudes contribute additional memory.
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
        if (2..=5).contains(&exp)
            && self.observe_retained_fact(RETAINED_POWER_REUSE_SEEN)
        {
            crate::trace_dispatch!("rational", "powi", "retained-product-chain");
            crate::trace_dispatch!(
                "rational_algorithm",
                "powering",
                "retained-binary-product-chain"
            );
            return self.pow_up_by_retained_multiplication(exp);
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
                crate::trace_dispatch!("rational_algorithm", "powering", "word-checked-pow");
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
            crate::trace_dispatch!("rational_algorithm", "powering", "backend-binary-pow");
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

        crate::trace_dispatch!(
            "rational_algorithm",
            "powering",
            "binary-repeated-squaring"
        );
        self.pow_up_by_multiplication(exp)
    }

    /// Build a power through borrowed products that can retain reusable edges.
    ///
    /// Exponents two through five have short square/product chains. Other
    /// nonnegative exponents use binary repeated squaring; zero returns one.
    /// The squared factor stays on the left of accumulated products so that its
    /// cache is probed first. Operand order is part of the reuse behavior.
    ///
    /// The caller controls amplification. No separate power-result cache or
    /// work/memory budget is installed by this helper.
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

    /// Evaluate a nonnegative machine exponent by binary repeated squaring.
    ///
    /// An accumulator starts at one, multiplies selected powers from low bits
    /// to high bits, and avoids the last unused square. Zero returns one.
    /// Multiplication assignment and borrowed squaring retain their existing
    /// cache behavior; this helper supplies no independent resource guard.
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

    /// Materialize an unsigned arbitrary-width exponent after caller admission.
    ///
    /// Machine-width exponents delegate to the specialized kernel. Larger
    /// exponents use a bitwise repeated-squaring loop with no local size or
    /// work guard. The public power method's current admission guard rejects
    /// this wider case for nonzero nonunit bases before reaching this helper;
    /// calling it directly can cause uncontrolled intermediate growth.
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

    /// Raise a rational to a signed machine exponent with eager-size admission.
    ///
    /// This is the allocation-saving exponent interface used by `Real`.
    /// It has the same value/error rules as [`Self::powi`], including `0^0 = 1`,
    /// zero to a negative power returning [`Problem::DivideByZero`], unit
    /// shortcuts, and the amplification guard returning [`Problem::Exhausted`].
    /// Internally unreduced operands canonicalize before any shortcut.
    ///
    /// Unsigned absolute value handles `i64::MIN` without signed overflow.
    /// `Real` performs its own domain checks before entering this kernel and
    /// can retain a lazy exact expression when eager admission fails.
    #[inline]
    pub(crate) fn powi_i64(self, exp: i64) -> Result<Self, Problem> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().clone().powi_i64(exp);
        }
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

        if !self.eager_power_u64_fits_budget(exp.unsigned_abs()) {
            return Err(Problem::Exhausted);
        }

        if exp < 0 {
            Ok(self.inverse()?.pow_up_u64(exp.unsigned_abs()))
        } else {
            Ok(self.pow_up_u64(exp as u64))
        }
    }

    /// Raise this rational exactly to an arbitrary signed integer exponent.
    ///
    /// Zero exponent returns one, including `0^0`. Zero to a positive exponent
    /// returns zero; zero to a negative exponent returns
    /// [`Problem::DivideByZero`]. Both units handle arbitrarily wide exponents
    /// by parity. Other negative exponents invert the base before powering.
    /// Internally unreduced operands canonicalize before these shortcuts.
    ///
    /// For nonzero nonunit bases, exponent magnitudes through 255 always pass.
    /// Larger exponents must fit `u64` and satisfy the conservative 65,536-bit
    /// amplification estimate; otherwise this returns [`Problem::Exhausted`].
    /// That estimate does not bound total work, retained memory, cancellation,
    /// or outputs on the small-exponent path. Allocation failures are not
    /// converted into typed exhaustion. Success retains exact rational data.
    ///
    /// ```
    /// use hyperreal::{Problem, Rational};
    /// assert_eq!(Rational::fraction(-2, 3).unwrap().powi((-3).into()),
    ///            Rational::fraction(-27, 8));
    /// assert_eq!(Rational::zero().powi(0.into()), Ok(Rational::one()));
    /// assert_eq!(Rational::zero().powi((-1).into()), Err(Problem::DivideByZero));
    /// assert_eq!(Rational::new(10).powi(20_000.into()), Err(Problem::Exhausted));
    /// ```
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().clone().powi(exp);
        }
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
        if !self.eager_power_fits_budget(exp.magnitude()) {
            return Err(Problem::Exhausted);
        }
        match exp.sign() {
            Minus => Ok(self.inverse()?.pow_up(exp.magnitude())),
            Plus => Ok(self.pow_up(exp.magnitude())),
            NoSign => unreachable!(),
        }
    }
}
