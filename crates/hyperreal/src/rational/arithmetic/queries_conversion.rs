impl Rational {
    ///
    /// This is a stored-sign structural predicate. It deliberately avoids
    /// constructing a comparison value so downstream numeric kernels can ask
    /// identity questions without allocating or canonicalizing. This keeps
    /// algebraic simplification ahead of approximation, matching the exact-real
    /// arithmetic strategy described by Boehm, Cartwright, Riggle, and
    /// O'Donnell, "Exact Real Arithmetic: A Case Study in Higher Order
    /// Programming", LFP 1986, <https://doi.org/10.1145/319838.319860>.
    #[inline(always)]
    pub fn is_zero(&self) -> bool {
        self.sign == NoSign
    }

    /// Returns whether this rational is exactly one.
    ///
    /// Kept as a structural identity predicate so matrix/vector callers can
    /// specialize common homogeneous coordinates without constructing
    /// `Rational::one()` in hot paths.
    #[inline(always)]
    pub fn is_one(&self) -> bool {
        self.sign == Plus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
    }

    #[inline]
    pub(crate) fn is_two(&self) -> bool {
        self.sign == Plus
            && self.numerator.bits() == 2
            && self.numerator == *TWO.deref()
            && self.denominator == *ONE.deref()
    }

    #[inline]
    pub(crate) fn is_one_half(&self) -> bool {
        self.sign == Plus
            && self.numerator.bits() == 1
            && self.denominator.bits() == 2
            && self.numerator == *ONE.deref()
            && self.denominator == *TWO.deref()
    }

    #[inline]
    pub(crate) fn is_minus_one(&self) -> bool {
        self.sign == Minus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
    }

    #[inline]
    pub(crate) fn cmp_one_structural(&self) -> Ordering {
        match self.sign {
            Minus | NoSign => Ordering::Less,
            Plus => self.numerator.cmp(&self.denominator),
        }
    }

    #[inline]
    pub(crate) fn abs_cmp_one_structural(&self) -> Ordering {
        self.numerator.cmp(&self.denominator)
    }

    /// The integer part of this Rational.
    ///
    /// Non integer rationals will thus be truncated towards zero
    ///
    /// # Examples
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let approx_pi = Rational::fraction(22, 7).unwrap();
    /// let three = Rational::new(3);
    /// assert_eq!(approx_pi.trunc(), three);
    /// ```
    ///
    /// The integer result can be converted to a primitive integer type
    /// with suitable range
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let fraction = Rational::new(172) / Rational::new(9);
    /// let int: u8 = fraction.trunc().try_into().unwrap();
    /// assert_eq!(int, 19);
    /// ```
    pub fn trunc(&self) -> Self {
        if self.is_integer() {
            return self.clone();
        }
        let n = &self.numerator / &self.denominator;
        Self {
            sign: self.sign,
            numerator: n,
            denominator: ONE.deref().clone(),
        }
    }

    /// The fractional part of this Rational.
    ///
    /// If the rational was negative, this fraction will also be negative
    ///
    /// # Examples
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let approx_pi = Rational::fraction(22, 7).unwrap();
    /// let a_seventh = Rational::fraction(1, 7).unwrap();
    /// assert_eq!(approx_pi.fract(), a_seventh);
    /// ```
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let backward = Rational::fraction(-53, 9).unwrap();
    /// let fract = Rational::fraction(-8, 9).unwrap();
    /// assert_eq!(backward.fract(), fract);
    /// ```
    pub fn fract(&self) -> Self {
        if self.is_integer() {
            return Self::zero();
        }
        let n = &self.numerator % &self.denominator;
        Self {
            sign: self.sign,
            numerator: n,
            denominator: self.denominator.clone(),
        }
    }

    pub(crate) fn denominator(&self) -> &BigUint {
        &self.denominator
    }

    pub(crate) fn numerator(&self) -> &BigUint {
        &self.numerator
    }

    pub(crate) fn factor_two_powers(&self) -> (i32, Self) {
        // Split a rational into 2^shift * odd_part.  Computable multiplication consumes
        // the shift as an Offset node, which is cheaper than a generic exact scale.
        let numerator_shift = self.numerator.trailing_zeros().unwrap_or(0);
        let denominator_shift = self
            .denominator
            .trailing_zeros()
            .expect("Rational denominators are never zero");
        let shift = i32::try_from(numerator_shift).expect("shift should fit in i32")
            - i32::try_from(denominator_shift).expect("shift should fit in i32");
        let numerator =
            &self.numerator >> usize::try_from(numerator_shift).expect("shift should fit in usize");
        let denominator = &self.denominator
            >> usize::try_from(denominator_shift).expect("shift should fit in usize");

        (
            shift,
            Self {
                sign: self.sign,
                numerator,
                denominator,
            },
        )
    }

    pub(crate) fn divide_by_power_of_two(&self, shift: i32) -> Option<Self> {
        if shift < 0 {
            return None;
        }
        if self.sign == NoSign || self.numerator.is_zero() {
            return Some(Self::zero());
        }

        let shift = u64::try_from(shift).ok()?;
        let numerator_shift = self
            .numerator
            .trailing_zeros()
            .expect("non-zero numerator has trailing zeros")
            .min(shift);
        let denominator_shift = shift - numerator_shift;
        let numerator =
            &self.numerator >> usize::try_from(numerator_shift).expect("shift should fit");
        let denominator =
            &self.denominator << usize::try_from(denominator_shift).expect("shift should fit");
        Some(Self::from_fraction_parts(self.sign, numerator, denominator))
    }

    #[inline]
    pub(crate) fn power_of_two_shift(&self) -> Option<(i32, Sign)> {
        // Identify exact +/-2^k scales. Computable multiplication consumes these as
        // binary Offset nodes, which are cheaper than generic rational products.
        if self.sign == NoSign {
            return None;
        }

        let numerator_shift = self
            .numerator
            .trailing_zeros()
            .expect("Rational numerators are never zero for non-zero signs");
        if numerator_shift != self.numerator.bits() - 1 {
            return None;
        }

        let denominator_shift = self
            .denominator
            .trailing_zeros()
            .expect("Rational denominators are never zero");
        if denominator_shift != self.denominator.bits() - 1 {
            return None;
        }

        let numerator_shift = i32::try_from(numerator_shift).ok()?;
        let denominator_shift = i32::try_from(denominator_shift).ok()?;
        Some((numerator_shift - denominator_shift, self.sign))
    }

    pub(crate) fn magnitude_at_least_power_of_two(&self, exponent: u32) -> bool {
        // Hot constructors sometimes need only a threshold such as |x| >= 8.
        // Bit lengths prove almost every case without the exact shift/compare
        // used by msd_exact(); only boundary-sized rationals allocate a shifted
        // denominator for the final comparison.
        if self.sign == NoSign {
            return false;
        }

        let numerator_bits = self.numerator.bits();
        let target_bits = self.denominator.bits() + u64::from(exponent);
        if numerator_bits > target_bits {
            return true;
        }
        if numerator_bits < target_bits {
            return false;
        }

        self.numerator >= (&self.denominator << exponent as usize)
    }

    pub(crate) fn msd_exact(&self) -> Option<i32> {
        // Exact binary magnitude from bit lengths only. This is used by Real
        // and Computable structural queries to avoid an approximation just to
        // choose working precision.
        if self.sign == NoSign {
            return None;
        }

        let numerator_bits = self.numerator.bits() as i32;
        let denominator_bits = self.denominator.bits() as i32;
        let candidate = numerator_bits - denominator_bits;

        let below = if candidate >= 0 {
            self.numerator < (&self.denominator << candidate as usize)
        } else {
            (&self.numerator << (-candidate) as usize) < self.denominator
        };

        if below {
            Some(candidate - 1)
        } else {
            Some(candidate)
        }
    }

    pub(crate) fn to_f64_lossy(&self) -> Option<f64> {
        // Fast borrowed approximate conversion for modest rationals. If either
        // side cannot fit through num-traits' f64 conversion, callers fall back
        // to the general Computable approximation path.
        if self.sign == NoSign {
            return Some(0.0);
        }

        let msd = self.msd_exact()?;
        if msd > 1023 {
            return None;
        }
        if msd < -1075 {
            return Some(0.0);
        }

        let numerator = self.numerator.to_f64()?;
        let denominator = self.denominator.to_f64()?;
        let value = numerator / denominator;
        if !value.is_finite() {
            return None;
        }

        Some(match self.sign {
            Minus => -value,
            NoSign => 0.0,
            Plus => value,
        })
    }

    /// Is this Rational better understood as a fraction?
    ///
    /// If a decimal expansion of this fraction would never end this is true.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let third = Rational::fraction(1, 3).unwrap();
    /// assert!(third.prefer_fraction());
    /// ```
    pub fn prefer_fraction(&self) -> bool {
        let mut rem = self.denominator.clone();
        while (&rem % &*TEN).is_zero() {
            rem /= &*TEN;
        }
        while (&rem % &*FIVE).is_zero() {
            rem /= &*FIVE;
        }
        while (&rem % &*TWO).is_zero() {
            rem /= &*TWO;
        }
        rem != BigUint::one()
    }

    /// Left shift the value by any amount and return a [`BigInt`]
    /// of the truncated integer value.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Rational;
    /// use num::bigint::ToBigInt;
    /// let seven_fifths = Rational::fraction(7, 5).unwrap();
    /// let eleven = ToBigInt::to_bigint(&11).unwrap();
    /// assert_eq!(seven_fifths.shifted_big_integer(3), eleven);
    /// ```
    pub fn shifted_big_integer(&self, shift: i32) -> BigInt {
        let whole = (&self.numerator << shift) / &self.denominator;
        BigInt::from_biguint(self.sign, whole)
    }

    /// Compare the magnitudes of two rationals without flipping signs.
    ///
    /// This keeps exact-rational absolute comparisons in `Computable` on
    /// pre-normalized magnitude fields and avoids temporary allocations.
    #[inline]
    pub(crate) fn compare_magnitude(&self, other: &Self) -> Ordering {
        if self.denominator == other.denominator {
            return self.numerator.cmp(&other.numerator);
        }
        if self.numerator.bits() > 64 && self.numerator == other.numerator {
            // Equal numerators let absolute order be decided by the reciprocal
            // denominator order: |n/d1| > |n/d2| iff d1 < d2. Keep this guarded
            // to multi-limb numerators: benchmarks showed the extra equality
            // check regresses tiny exact-rational compares, while large
            // numerators avoid two BigUint products. This mirrors the adaptive
            // "cheap predicate first only when it is actually cheap" strategy
            // used in exact geometric computation; see Shewchuk, "Adaptive
            // Precision Floating-Point Arithmetic and Fast Robust Geometric
            // Predicates" (1997), and Yap, "Towards Exact Geometric
            // Computation" (1997).
            return other.denominator.cmp(&self.denominator);
        }
        (&self.numerator * &other.denominator).cmp(&(&other.numerator * &self.denominator))
    }

    /// Compare `|self|^2 * factor` with `other` without constructing an
    /// intermediate Rational.
    ///
    /// This is a structural predicate for sqrt-domain gates such as
    /// `|a*sqrt(r)| <= 1`. Avoiding canonicalization here keeps inverse
    /// trigonometric and hyperbolic dispatch in the exact-rational layer until
    /// a real approximation is actually required.
    #[inline]
    pub(crate) fn compare_magnitude_squared_times(&self, factor: &Self, other: &Self) -> Ordering {
        let left = &self.numerator * &self.numerator * &factor.numerator * &other.denominator;
        let right = &self.denominator * &self.denominator * &factor.denominator * &other.numerator;
        left.cmp(&right)
    }

    #[inline]
    pub(crate) fn detailed_rational_facts(&self) -> RationalFacts {
        let denominator_is_one = self.denominator == *ONE.deref();
        let numerator_bits = self.numerator.bits();
        let denominator_bits = self.denominator.bits();
        let denominator_is_power_of_two = Self::is_power_of_two(&self.denominator);
        let numerator_is_power_of_two =
            self.sign != NoSign && Self::is_power_of_two(&self.numerator);
        let storage = if self.sign == NoSign {
            RationalStorageClass::Zero
        } else if numerator_bits <= 64 && denominator_bits <= 64 {
            RationalStorageClass::WordSized
        } else if numerator_bits.saturating_add(denominator_bits) <= 4096 {
            RationalStorageClass::MultiLimb
        } else {
            RationalStorageClass::VeryLarge
        };

        RationalFacts {
            exact_integer: denominator_is_one,
            exact_small_integer_i64: denominator_is_one
                && (self.sign == NoSign || numerator_bits <= 63),
            exact_dyadic: denominator_is_power_of_two,
            power_of_two: numerator_is_power_of_two && denominator_is_power_of_two,
            storage,
        }
    }

    /// Either the corresponding [`BigInt`] or None if this value is not an integer.
    pub fn to_big_integer(&self) -> Option<BigInt> {
        self.integer_magnitude()
            .map(|magnitude| BigInt::from_biguint(self.sign, magnitude.clone()))
    }

    pub(crate) fn integer_magnitude(&self) -> Option<&BigUint> {
        // Integer-only callers usually need the non-negative magnitude. Return
        // the borrowed BigUint so they can avoid constructing a signed BigInt.
        (self.denominator == *ONE.deref()).then_some(&self.numerator)
    }

    pub(crate) fn to_integer_i64(&self) -> Option<i64> {
        let magnitude = self.integer_magnitude()?.to_i64()?;
        match self.sign {
            Plus | NoSign => Some(magnitude),
            Minus => magnitude.checked_neg(),
        }
    }

    /// The [`Sign`] of this value.
    #[inline]
    pub fn sign(&self) -> Sign {
        self.sign
    }

}
