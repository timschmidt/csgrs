impl Rational {
    #[inline]
    fn from_parts_raw(sign: Sign, numerator: BigUint, denominator: BigUint) -> Self {
        Self(Arc::new(RationalData {
            sign,
            numerator,
            denominator,
            product_cache: OnceLock::new(),
            linear_cache: OnceLock::new(),
            linear_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            power_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            square_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            exact_f64_view: std::sync::atomic::AtomicBool::new(false),
        }))
    }

    fn into_parts(self) -> (Sign, BigUint, BigUint) {
        let RationalData {
            sign,
            numerator,
            denominator,
            product_cache: _,
            linear_cache: _,
            linear_reuse_seen: _,
            power_reuse_seen: _,
            square_reuse_seen: _,
            exact_f64_view: _,
        } = Arc::try_unwrap(self.0).unwrap_or_else(|shared| RationalData {
            sign: shared.sign,
            numerator: shared.numerator.clone(),
            denominator: shared.denominator.clone(),
            product_cache: OnceLock::new(),
            linear_cache: OnceLock::new(),
            linear_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            power_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            square_reuse_seen: std::sync::atomic::AtomicBool::new(false),
            exact_f64_view: std::sync::atomic::AtomicBool::new(false),
        });
        (sign, numerator, denominator)
    }

    #[inline]
    fn retained_inverse(&self) -> Option<Self> {
        let cached = self.linear_cache.get()?;
        match cached.primary.kind {
            CachedRationalLinearKind::StrongInversePlaceholder => {
                return Some(cached.primary.result.clone());
            }
            CachedRationalLinearKind::WeakInversePlaceholder => {
                return cached.primary.other.upgrade().map(Self);
            }
            _ => {}
        }
        if let Some(inverse) = cached.tertiary.get() {
            match inverse.kind {
                CachedRationalLinearKind::StrongInversePlaceholder => {
                    return Some(inverse.result.clone());
                }
                CachedRationalLinearKind::WeakInversePlaceholder => {
                    return inverse.other.upgrade().map(Self);
                }
                _ => {}
            }
        }
        if let Some(inverse) = cached.quaternary.get() {
            match inverse.kind {
                CachedRationalLinearKind::StrongInversePlaceholder => {
                    return Some(inverse.result.clone());
                }
                CachedRationalLinearKind::WeakInversePlaceholder => {
                    return inverse.other.upgrade().map(Self);
                }
                _ => {}
            }
        }
        let inverse = cached.quinary.get()?;
        match inverse.kind {
            CachedRationalLinearKind::StrongInversePlaceholder => Some(inverse.result.clone()),
            CachedRationalLinearKind::WeakInversePlaceholder => inverse.other.upgrade().map(Self),
            _ => None,
        }
    }

    fn retain_inverse_entry(&self, inverse: CachedRationalUnary) -> bool {
        if let Some(cached) = self.linear_cache.get() {
            if cached.primary.kind.is_inverse_placeholder() {
                return false;
            }
            let entry = match inverse {
                CachedRationalUnary::Strong(inverse) => CachedRationalLinearEntry {
                    other: std::sync::Weak::new(),
                    kind: CachedRationalLinearKind::StrongInversePlaceholder,
                    result: inverse,
                },
                CachedRationalUnary::Weak(inverse) => CachedRationalLinearEntry {
                    other: inverse,
                    kind: CachedRationalLinearKind::WeakInversePlaceholder,
                    result: RATIONAL_ZERO.clone(),
                },
            };
            if cached.primary.kind.is_primary_placeholder() {
                return cached
                    .quaternary
                    .set(entry)
                    .or_else(|entry| cached.quinary.set(entry))
                    .is_ok();
            }
            return cached
                .tertiary
                .set(entry)
                .or_else(|entry| cached.quaternary.set(entry))
                .or_else(|entry| cached.quinary.set(entry))
                .is_ok();
        }

        // Preserve a direct primary linear-cache load on the established
        // add/sub hot path. When inverse retention initializes this lazy box
        // first, the inert primary slot marks secondary and tertiary as the
        // two available linear entries; quaternary and quinary remain available
        // for the other cycle-free unary pair and future placeholder layouts.
        let (kind, other, placeholder) = match inverse {
            CachedRationalUnary::Strong(inverse) => (
                CachedRationalLinearKind::StrongInversePlaceholder,
                std::sync::Weak::new(),
                inverse,
            ),
            CachedRationalUnary::Weak(inverse) => (
                CachedRationalLinearKind::WeakInversePlaceholder,
                inverse,
                RATIONAL_ZERO.clone(),
            ),
        };
        self.linear_cache
            .set(Box::new(CachedRationalArithmetic {
                primary: CachedRationalLinearEntry {
                    other,
                    kind,
                    result: placeholder,
                },
                secondary: OnceLock::new(),
                tertiary: OnceLock::new(),
                quaternary: OnceLock::new(),
                quinary: OnceLock::new(),
                square_reduction: OnceLock::new(),
            }))
            .is_ok()
    }

    #[cold]
    fn retain_inverse_pair(&self, inverse: &Self) {
        let _ = inverse.retain_inverse_entry(CachedRationalUnary::Weak(Arc::downgrade(&self.0)));
        let _ = self.retain_inverse_entry(CachedRationalUnary::Strong(inverse.clone()));
    }

    /// Zero, the additive identity.
    pub fn zero() -> Self {
        trace_rational_temporary!();
        RATIONAL_ZERO.clone()
    }

    /// One, the multiplicative identity.
    pub fn one() -> Self {
        trace_rational_temporary!();
        RATIONAL_ONE.clone()
    }

    fn minus_one() -> Self {
        trace_rational_temporary!();
        RATIONAL_MINUS_ONE.clone()
    }

    fn small_integer(sign: Sign, magnitude: u128) -> Option<Self> {
        let index = usize::try_from(magnitude.checked_sub(2)?).ok()?;
        let values = match sign {
            Plus => &SMALL_POSITIVE_RATIONALS,
            Minus => &SMALL_NEGATIVE_RATIONALS,
            NoSign => return Some(Self::zero()),
        };
        let value = values.get(index)?.get_or_init(|| {
            Self::from_parts_raw(
                sign,
                BigUint::from((index + 2) as u8),
                BigUint::one(),
            )
        });
        trace_rational_temporary!();
        Some(value.clone())
    }

    fn small_dyadic(sign: Sign, magnitude: u128, denominator: u128) -> Option<Self> {
        if magnitude == 0 || magnitude > 63 || magnitude & 1 == 0 || !denominator.is_power_of_two()
        {
            return None;
        }
        let shift = usize::try_from(denominator.trailing_zeros()).ok()?;
        if shift == 0 || shift > SMALL_DYADIC_MAX_SHIFT {
            return None;
        }
        let magnitude_index = usize::try_from(magnitude >> 1).ok()?;
        let index = (shift - 1) * SMALL_DYADIC_ODD_MAGNITUDES + magnitude_index;
        let values = match sign {
            Plus => &SMALL_POSITIVE_DYADICS,
            Minus => &SMALL_NEGATIVE_DYADICS,
            NoSign => return Some(Self::zero()),
        };
        let value = values.get(index)?.get_or_init(|| {
            Self::from_parts_raw(
                sign,
                BigUint::from(magnitude),
                BigUint::from(denominator),
            )
        });
        trace_rational_temporary!();
        Some(value.clone())
    }

    /// The non-negative Rational corresponding to the provided [`i64`].
    pub fn new(n: i64) -> Self {
        // Small scalar constructors are hot. Rational is stored as
        // Sign+BigUint, so going through BigInt first only adds allocation and
        // sign extraction work.
        let sign = if n < 0 { Minus } else { Plus };
        let magnitude = n.unsigned_abs();
        if magnitude == 0 {
            return Self::zero();
        }
        if magnitude == 1 {
            return if sign == Minus {
                Self::minus_one()
            } else {
                Self::one()
            };
        }
        if let Some(value) = Self::small_integer(sign, u128::from(magnitude)) {
            return value;
        }
        Self::from_integer_magnitude(
            sign,
            BigUint::from(magnitude),
        )
    }

    /// The Rational corresponding to the provided [`BigInt`].
    pub fn from_bigint(n: BigInt) -> Self {
        Self::from_bigint_fraction(n, BigUint::one()).unwrap()
    }

    /// The non-negative Rational corresponding to the provided [`i64`]
    /// numerator and [`u64`] denominator as a fraction.
    pub fn fraction(n: i64, d: u64) -> Result<Self, Problem> {
        if d == 0 {
            return Err(Problem::DivideByZero);
        }
        let sign = if n < 0 { Minus } else { Plus };
        // The storage type is already Sign+BigUint, so unsigned_abs gives the
        // exact magnitude type and avoids a temporary signed BigInt.
        let numerator = BigUint::from(n.unsigned_abs());
        let denominator = BigUint::from(d);
        Ok(Self::from_fraction_parts(sign, numerator, denominator).reduce())
    }

    /// The Rational corresponding to the provided [`BigInt`]
    /// numerator and [`BigUint`] denominator as a fraction.
    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Result<Self, Problem> {
        if denominator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        let (sign, numerator) = n.into_parts();
        let answer = Self::from_fraction_parts(sign, numerator, denominator);
        Ok(answer.reduce())
    }

    pub(crate) fn from_integer_magnitude(sign: Sign, numerator: BigUint) -> Self {
        if numerator.is_one() {
            return match sign {
                Minus => Self::minus_one(),
                Plus => Self::one(),
                NoSign => Self::zero(),
            };
        }
        Self::from_fraction_parts(sign, numerator, BigUint::one())
    }

    pub(crate) fn from_unsigned_integer(numerator: BigUint) -> Self {
        Self::from_integer_magnitude(Plus, numerator)
    }

    fn from_fraction_parts(sign: Sign, numerator: BigUint, denominator: BigUint) -> Self {
        if sign == NoSign || numerator.is_zero() {
            return Self::zero();
        }
        trace_rational_temporary!();
        Self::from_parts_raw(sign, numerator, denominator)
    }

    pub(crate) fn add_one(&self) -> Self {
        if self.sign == NoSign {
            return Self::one();
        }

        match self.sign {
            Plus => Self::from_fraction_parts(
                Plus,
                &self.numerator + &self.denominator,
                self.denominator.clone(),
            ),
            Minus => match self.numerator.cmp(&self.denominator) {
                Ordering::Greater => Self::from_fraction_parts(
                    Minus,
                    &self.numerator - &self.denominator,
                    self.denominator.clone(),
                ),
                Ordering::Equal => Self::zero(),
                Ordering::Less => Self::from_fraction_parts(
                    Plus,
                    &self.denominator - &self.numerator,
                    self.denominator.clone(),
                ),
            },
            NoSign => unreachable!(),
        }
    }

    pub(crate) fn subtract_one(&self) -> Self {
        if self.sign == NoSign {
            return Self::from_integer_magnitude(Minus, ONE.deref().clone());
        }

        match self.sign {
            Plus => match self.numerator.cmp(&self.denominator) {
                Ordering::Greater => Self::from_fraction_parts(
                    Plus,
                    &self.numerator - &self.denominator,
                    self.denominator.clone(),
                ),
                Ordering::Equal => Self::zero(),
                Ordering::Less => Self::from_fraction_parts(
                    Minus,
                    &self.denominator - &self.numerator,
                    self.denominator.clone(),
                ),
            },
            Minus => Self::from_fraction_parts(
                Minus,
                &self.numerator + &self.denominator,
                self.denominator.clone(),
            ),
            NoSign => unreachable!(),
        }
    }

    fn maybe_reduce(self) -> Self {
        if Self::is_power_of_two(&self.denominator) {
            let denominator = self.denominator.clone();
            trace_rational_reduction!(&self.numerator, &self.denominator);
            // Dyadic rationals dominate f64 imports and trig reduction scales.  When the
            // denominator is a power of two, remove common factors with shifts instead of a
            // full BigInt gcd.
            self.reduce_by_power_of_two_divisor(&denominator)
        } else {
            self.reduce()
        }
    }

    fn reduce_with_possible_divisor(self, possible_divisor: &BigUint) -> Self {
        if self.sign == NoSign || self.numerator.is_zero() {
            return Self::zero();
        }
        if self.denominator == *ONE.deref() || possible_divisor == &*ONE {
            return self;
        }

        trace_rational_reduction!(&self.numerator, &self.denominator);
        if Self::is_power_of_two(possible_divisor) {
            // Callers often already know a possible divisor from the operation they just
            // performed.  Preserve that hint for dyadic cases so reduction stays shift-only.
            return self.reduce_by_power_of_two_divisor(possible_divisor);
        }

        let divisor = num::Integer::gcd(&self.numerator, possible_divisor);
        trace_rational_gcd!(&self.numerator, possible_divisor, &divisor);
        if divisor == *ONE.deref() {
            self
        } else {
            trace_rational_temporary!();
            Self::from_parts_raw(
                self.sign,
                &self.numerator / &divisor,
                &self.denominator / divisor,
            )
        }
    }

    fn reduce(self) -> Self {
        if self.denominator == *ONE.deref() {
            return self;
        }

        trace_rational_reduction!(&self.numerator, &self.denominator);
        if Self::is_power_of_two(&self.denominator) {
            let denominator = self.denominator.clone();
            // Powers of two are common enough that avoiding gcd here shows up in scalar
            // import and matrix benchmarks.
            return self.reduce_by_power_of_two_divisor(&denominator);
        }

        let divisor = num::Integer::gcd(&self.numerator, &self.denominator);
        trace_rational_gcd!(&self.numerator, &self.denominator, &divisor);
        if divisor == *ONE.deref() {
            self
        } else {
            let numerator = &self.numerator / &divisor;
            let denominator = &self.denominator / &divisor;
            trace_rational_temporary!();
            Self::from_parts_raw(self.sign, numerator, denominator)
        }
    }

    fn biguint_power_of_two_shift(value: &BigUint) -> Option<u64> {
        if value.is_zero() {
            return None;
        }
        // BigUint has cheap trailing-zero and bit-length queries; together they identify a
        // dyadic denominator without allocating or dividing.
        let shift = value
            .trailing_zeros()
            .expect("non-zero BigUint has trailing zeros");
        (shift == value.bits() - 1).then_some(shift)
    }

    fn is_power_of_two(value: &BigUint) -> bool {
        Self::biguint_power_of_two_shift(value).is_some()
    }

    fn reduce_by_power_of_two_divisor(self, possible_divisor: &BigUint) -> Self {
        if self.sign == NoSign || self.numerator.is_zero() {
            return Self::zero();
        }
        let numerator_shift = self
            .numerator
            .trailing_zeros()
            .expect("non-zero numerator has trailing zeros");
        if numerator_shift == 0 {
            trace_rational_power_of_two_common_factor!(0);
            return self;
        }
        let divisor_shift = possible_divisor
            .trailing_zeros()
            .expect("power-of-two divisor has trailing zeros");
        let shift = numerator_shift.min(divisor_shift);
        if shift == 0 {
            trace_rational_power_of_two_common_factor!(0);
            return self;
        }
        let shift = usize::try_from(shift).expect("shift should fit in usize");
        trace_rational_power_of_two_common_factor!(shift as u64);
        // Shift out common powers of two directly.  This is the hot reduction path for
        // exactly representable binary fractions.
        trace_rational_temporary!();
        Self::from_parts_raw(
            self.sign,
            &self.numerator >> shift,
            &self.denominator >> shift,
        )
    }

    /// The inverse of this Rational.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let five = Rational::new(5);
    /// let a_fifth = Rational::fraction(1, 5).unwrap();
    /// assert_eq!(five.clone().inverse().unwrap(), a_fifth);
    /// assert_eq!(a_fifth.clone().inverse().unwrap(), five);
    /// ```
    pub fn inverse(self) -> Result<Self, Problem> {
        if let Some(inverse) = self.retained_inverse() {
            crate::trace_dispatch!("rational", "inverse", "retained");
            return Ok(inverse);
        }
        if self.numerator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        match Arc::try_unwrap(self.0) {
            Ok(data) => Ok(Self::from_parts_raw(
                data.sign,
                data.denominator,
                data.numerator,
            )),
            Err(shared) => {
                let owner = Self(shared);
                let inverse = Self::from_parts_raw(
                    owner.sign,
                    owner.denominator.clone(),
                    owner.numerator.clone(),
                );
                owner.retain_inverse_pair(&inverse);
                Ok(inverse)
            }
        }
    }

    /// Checks if the value is an integer.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Rational;
    /// assert!(Rational::new(5).is_integer());
    /// assert!(Rational::fraction(16, 4).unwrap().is_integer());
    /// assert!(!Rational::fraction(5, 4).unwrap().is_integer());
    /// ```
    pub fn is_integer(&self) -> bool {
        self.denominator == *ONE.deref()
    }

    /// Returns true when this rational has a power-of-two denominator.
    ///
    /// This is a cheap structural query used by higher-level exact arithmetic
    /// kernels to decide whether extra multiplication will stay on dyadic
    /// shift-only reductions or will likely trigger full BigInt gcd work.
    pub fn is_dyadic(&self) -> bool {
        Self::is_power_of_two(&self.denominator)
    }

    /// Return whether two rationals share the same reduced denominator.
    ///
    /// This is a structural query for higher-level exact kernels that carry
    /// common-scale facts opportunistically. It exposes only denominator
    /// equality, not the denominator itself, so geometry crates can select
    /// faster shared-scale schedules while `Rational` keeps ownership of its
    /// storage and reduction strategy, preserving object-level rational
    /// structure before scalar expansion.
    #[inline]
    pub fn same_denominator(&self, other: &Self) -> bool {
        self.denominator == other.denominator
    }

    pub(crate) fn dyadic_denominator_shift(&self) -> Option<u64> {
        Self::biguint_power_of_two_shift(&self.denominator)
    }

    fn from_signed_magnitude_difference(
        positive: BigUint,
        negative: BigUint,
        mut denominator: BigUint,
    ) -> Self {
        let (sign, mut numerator) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => return Self::zero(),
        };
        if let Some(denominator_shift) =
            Self::biguint_power_of_two_shift(&denominator)
        {
            let numerator_shift = numerator
                .trailing_zeros()
                .expect("non-zero numerator has trailing zeros");
            let common_shift = numerator_shift.min(denominator_shift);
            if common_shift != 0 {
                let shift = usize::try_from(common_shift)
                    .expect("dyadic reduction shift fits usize");
                numerator >>= shift;
                denominator >>= shift;
            }
            trace_rational_power_of_two_common_factor!(common_shift);
            trace_rational_temporary!();
            return Self::from_parts_raw(sign, numerator, denominator);
        }
        trace_rational_temporary!();
        Self::from_parts_raw(sign, numerator, denominator)
        .maybe_reduce()
    }

}
