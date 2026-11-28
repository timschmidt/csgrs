/// Exact value/size queries, symbolic binary scaling and primitive-float views.
///
/// Canonicalizing queries can allocate and retain reduced parts; raw views and
/// borrowed comparisons keep original storage available for size admission.
/// Rounded floats, exact representability facts and certified enclosures have
/// distinct contracts. This layer supplies no universal work/memory budget.
impl Rational {
    /// Borrow the process-lifetime shared rational one without cloning its handle.
    ///
    /// Borrowed exact kernels use this identity to avoid constructing a temporary
    /// rational. First access initializes the shared value if necessary.
    #[inline]
    pub fn one_ref() -> &'static Self {
        &RATIONAL_ONE
    }

    /// Return the address of this rational's immutable shared payload as a cache key.
    ///
    /// Clones have the same key; equal values with separate payloads need not.
    /// This is neither a numeric hash nor a stable serialization identifier.
    /// Callers retaining the key must also retain an owning clone so allocator
    /// address reuse cannot identify a different object. The query performs no
    /// canonicalization or numeric comparison.
    #[inline]
    pub fn storage_identity(&self) -> usize {
        Arc::as_ptr(&self.0) as usize
    }

    /// Classify stored magnitude widths for choosing exact arithmetic schedules.
    ///
    /// Stored zero returns `Zero`. Otherwise both widths at most 64 give
    /// `WordSized`; a combined width at most 4,096 gives `MultiLimb`; larger
    /// storage gives `VeryLarge`. Lazy fractions are inspected without reduction,
    /// so their class can differ from the canonical class in detailed facts.
    /// These buckets are not byte counts, work bounds or float-range certificates.
    #[inline]
    pub fn storage_class(&self) -> RationalStorageClass {
        if self.sign == NoSign {
            RationalStorageClass::Zero
        } else {
            let numerator_bits = self.numerator.bits();
            let denominator_bits = self.denominator.bits();
            if numerator_bits <= 64 && denominator_bits <= 64 {
                RationalStorageClass::WordSized
            } else if numerator_bits.saturating_add(denominator_bits) <= 4096 {
                RationalStorageClass::MultiLimb
            } else {
                RationalStorageClass::VeryLarge
            }
        }
    }

    /// Return whether the stored sign identifies exact zero.
    ///
    /// Valid rational construction keeps zero sign and magnitude consistent.
    /// This query does not allocate, compare magnitudes, reduce the fraction or
    /// populate a cache; it is not a validator for malformed private payloads.
    #[inline(always)]
    pub fn is_zero(&self) -> bool {
        self.sign == NoSign
    }

    /// Return whether this rational equals positive one exactly.
    ///
    /// Internally unreduced inputs first obtain their cached canonical value,
    /// which may allocate and reduce. Canonical inputs compare sign and both
    /// unit magnitudes without constructing a comparison rational.
    #[inline(always)]
    pub fn is_one(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_one();
        }
        self.sign == Plus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
    }

    /// Recognize positive two for exact scalar simplification.
    ///
    /// Canonicalize internally unreduced input, then test sign, numerator width
    /// and the exact numerator/denominator. False is an exact rejection on valid
    /// rational storage, not a failed numerical approximation.
    #[inline]
    pub(crate) fn is_two(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_two();
        }
        self.sign == Plus
            && self.numerator.bits() == 2
            && self.numerator == *TWO.deref()
            && self.denominator == *ONE.deref()
    }

    /// Recognize positive one half for exact scalar simplification.
    ///
    /// Internally unreduced inputs canonicalize first. Sign, small bit widths
    /// and exact unit/two magnitudes then decide the identity without refinement.
    #[inline]
    pub(crate) fn is_one_half(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_one_half();
        }
        self.sign == Plus
            && self.numerator.bits() == 1
            && self.denominator.bits() == 2
            && self.numerator == *ONE.deref()
            && self.denominator == *TWO.deref()
    }

    /// Recognize negative one exactly, canonicalizing lazy storage if necessary.
    ///
    /// Reduced inputs require negative sign and unit numerator and denominator;
    /// no temporary comparison rational or numerical approximation is needed.
    #[inline]
    pub(crate) fn is_minus_one(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_minus_one();
        }
        self.sign == Minus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
    }

    /// Compare this signed rational with one directly from stored parts.
    ///
    /// Negative values and zero are less; positive values compare numerator
    /// with denominator. The result is exact even for valid unreduced ratios,
    /// without normalization, multiplication, allocation or cache changes.
    #[inline]
    pub(crate) fn cmp_one_structural(&self) -> Ordering {
        match self.sign {
            Minus | NoSign => Ordering::Less,
            Plus => self.numerator.cmp(&self.denominator),
        }
    }

    /// Compare the absolute rational value with one using its stored magnitudes.
    ///
    /// Numerator/denominator order is exact for valid reduced or unreduced
    /// fractions. Sign is ignored and no normalization or allocation occurs.
    #[inline]
    pub(crate) fn abs_cmp_one_structural(&self) -> Ordering {
        self.numerator.cmp(&self.denominator)
    }

    /// Return the exact integer part, truncating toward zero.
    ///
    /// Internally unreduced storage canonicalizes first. Integers clone their
    /// canonical handle; other values divide the unsigned magnitudes and restore
    /// the sign, with a canonical zero for magnitudes below one. Division and
    /// canonicalization have no caller-supplied work or allocation budget.
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
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().trunc();
        }
        if self.is_integer() {
            return self.clone();
        }
        let n = &self.numerator / &self.denominator;
        if n.is_zero() {
            return Self::zero();
        }
        Self::from_parts_raw(self.sign, n, ONE.deref().clone())
    }

    /// Return `self - trunc(self)` exactly, with magnitude strictly below one.
    ///
    /// Nonzero fractional parts have the input sign; integers return zero.
    /// Lazy inputs canonicalize first. The unsigned remainder reuses the
    /// canonical denominator, since a reduced fraction's remainder is coprime
    /// to that denominator. Remainder dispatch is traced and has no local
    /// work/cancellation budget.
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
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().fract();
        }
        if self.is_integer() {
            return Self::zero();
        }
        let n = Self::remainder_magnitudes(
            "exact-fractional-remainder",
            &self.numerator,
            &self.denominator,
        );
        Self::from_parts_raw(self.sign, n, self.denominator.clone())
    }

    /// Borrow the canonical positive denominator, including one for zero.
    ///
    /// Together with [`Self::numerator`] and [`Self::sign`], this exposes the
    /// reduced exact ratio without cloning the returned magnitude. Obtaining
    /// that reference can reduce and cache internally unreduced input; it is
    /// not an allocation-free inspection of the original payload.
    pub fn denominator(&self) -> &BigUint {
        &self.canonicalized_ref().denominator
    }

    /// Borrow the canonical unsigned numerator magnitude, zero for zero.
    ///
    /// The sign is separate in [`Self::sign`]. Lazy ratios may allocate a cached
    /// canonical representation before returning this reference. Use
    /// [`Self::ratio_parts`] to inspect stored magnitudes without normalization.
    pub fn numerator(&self) -> &BigUint {
        &self.canonicalized_ref().numerator
    }

    /// Report the largest stored width needing possible canonicalization.
    ///
    /// Internally unreduced input returns the maximum numerator/denominator bit
    /// length even if its canonical cache is warm; reduced input returns zero.
    /// The query neither reduces nor allocates. Consumers can charge consistent
    /// size admission, but this width is not a count of arithmetic operations,
    /// retained bytes or actual remaining work.
    #[inline]
    pub fn reduction_input_bits(&self) -> u64 {
        if self.is_internally_unreduced() {
            self.numerator.bits().max(self.denominator.bits())
        } else {
            0
        }
    }

    /// Borrow the stored sign, numerator magnitude and positive denominator.
    ///
    /// Their ratio is exact, but the magnitudes need not be reduced. The view
    /// never canonicalizes, clones a magnitude or populates a cache. Budgeted
    /// consumers can inspect sizes before arithmetic. These raw parts are not
    /// a canonical serialization or a mathematical identity key.
    pub fn ratio_parts(&self) -> (Sign, &BigUint, &BigUint) {
        (self.sign, &self.numerator, &self.denominator)
    }

    /// Split the value exactly as `2^shift * odd_part` for symbolic log scaling.
    ///
    /// After canonicalizing lazy input, remove all factors of two from both
    /// magnitudes and preserve the sign. Nonzero residual magnitudes are odd;
    /// zero returns shift zero and zero. The extracted factor can become a
    /// binary scale instead of a generic rational expression.
    ///
    /// Each trailing-zero count must fit `i32` and each allocation shift must
    /// fit `usize`; failed conversions panic rather than report exhaustion.
    /// Magnitude copies/reduction are not covered by a total resource budget.
    pub(crate) fn factor_two_powers(&self) -> (i32, Self) {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().factor_two_powers();
        }
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
            Self::from_parts_raw(self.sign, numerator, denominator),
        )
    }

    /// Return the exact quotient by `2^shift` for a nonnegative machine shift.
    ///
    /// Negative shifts return `None` before canonicalization. Otherwise lazy
    /// input canonicalizes, zero returns zero, and numerator powers of two
    /// cancel before the remaining shift expands the denominator. The resulting
    /// fraction is reduced through the normal construction authority.
    ///
    /// None reports shift ineligibility, not resource exhaustion. Large accepted
    /// shifts can allocate large magnitudes; there is no work/memory budget or
    /// cancellation hook. Shift conversion assumes the supported address width.
    pub(crate) fn divide_by_power_of_two(&self, shift: i32) -> Option<Self> {
        if shift < 0 {
            return None;
        }
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().divide_by_power_of_two(shift);
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

    /// Recognize a nonzero signed power of two as `(exponent, sign)`.
    ///
    /// Canonicalize lazy input, then require both magnitudes to have one set bit.
    /// Zero and nonpowers return `None`. Each individual magnitude exponent must
    /// fit `i32`; an out-of-range exponent also returns `None`, without claiming
    /// that the rational is not a mathematical power of two. Symbolic consumers
    /// use successful recognition to construct an exact binary scale.
    #[inline]
    pub(crate) fn power_of_two_shift(&self) -> Option<(i32, Sign)> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().power_of_two_shift();
        }
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

    /// Decide `abs(self) >= 2^exponent` exactly for a nonnegative exponent.
    ///
    /// Zero fails. Stored bit lengths usually decide; equal-width cases compare
    /// borrowed shifted digits without constructing the shifted integer. Valid
    /// unreduced ratios are supported without canonicalization. The denominator
    /// width plus exponent must fit `u64`, as for the borrowed comparison helper;
    /// there is no total scanning-work or cancellation budget.
    pub(crate) fn magnitude_at_least_power_of_two(&self, exponent: u32) -> bool {
        // Hot constructors sometimes need only a threshold such as |x| >= 8.
        // Bit lengths prove almost every case. Boundary-sized rationals use the
        // same borrowed bit comparison as msd_exact(), avoiding a shifted
        // BigUint temporary for the final comparison.
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

        Self::compare_shifted_to(&self.denominator, u64::from(exponent), &self.numerator)
            != Ordering::Greater
    }

    /// Compare `left * 2^shift` with `right` using borrowed magnitude digits.
    ///
    /// Both magnitudes must be positive and the shifted width must fit `u64`.
    /// This adapter preserves the common shifted-integer comparison authority;
    /// it does not allocate a shifted `BigUint` or add validation for zero and
    /// overflow outside that helper's domain.
    #[inline]
    fn compare_shifted_to(left: &BigUint, shift: u64, right: &BigUint) -> Ordering {
        compare_shifted_biguints(left, shift, right, 0)
    }

    /// Return `floor(log2(abs(self)))` when nonzero and representable as `i32`.
    ///
    /// Zero or exponent-range failure returns `None`. Exact borrowed magnitude
    /// comparison works on unreduced ratios without canonicalization; no float
    /// logarithm or approximation participates.
    #[inline]
    pub(crate) fn msd_exact(&self) -> Option<i32> {
        self.msd_exact_with_offset(0).map(|(msd, _)| msd)
    }

    /// Return the exact floor-log2 magnitude plus offset, and a binary-scale flag.
    ///
    /// For a nonzero value the pair is `(floor(log2(abs(self))) + offset, binary)`,
    /// where `binary` proves that the original magnitude is an integer power of
    /// two. Zero or a final exponent outside `i32` returns `None`. Wide bit
    /// length arithmetic applies the offset before narrowing, so it can rescue
    /// an otherwise out-of-range exponent.
    ///
    /// The same borrowed shifted comparison establishes the binade and equality,
    /// including valid unreduced ratios. No canonicalization, shifted-magnitude
    /// allocation or floating-point logarithm is needed.
    #[inline]
    pub(crate) fn msd_exact_with_offset(&self, offset: i32) -> Option<(i32, bool)> {
        if self.sign == NoSign {
            return None;
        }
        let numerator_bits = self.numerator.bits();
        let denominator_bits = self.denominator.bits();
        let ordering = if numerator_bits >= denominator_bits {
            Self::compare_shifted_to(
                &self.denominator, numerator_bits - denominator_bits, &self.numerator,
            ).reverse()
        } else {
            Self::compare_shifted_to(
                &self.numerator, denominator_bits - numerator_bits, &self.denominator,
            )
        };
        let msd = crate::verified::word::checked_bit_length_msd(
            numerator_bits, denominator_bits, ordering == Ordering::Less, offset,
        )?;
        Some((msd, ordering == Ordering::Equal))
    }

    /// Round this exact rational to nearest binary64, with ties to even.
    ///
    /// Returns a finite float or `None` when its rounded magnitude overflows or
    /// the exact magnitude exponent is outside the internal `i32` query range.
    /// Zero becomes positive zero; tiny negative inputs can round to negative
    /// zero. Internally unreduced storage canonicalizes first.
    ///
    /// Dyadic paths combine scale and significand directly. Other inputs use
    /// exactly convertible small operands or exact quotient/remainder rounding
    /// to avoid double rounding. Conversion may learn dyadic facts, but success
    /// alone does not publish an exact-float or certified-filter fact. The exact
    /// rational remains authoritative. Wide quotient rounding can allocate and
    /// has no total-work or cancellation budget.
    #[inline]
    pub(crate) fn to_f64_lossy(&self) -> Option<f64> {
        // The unreduced bit is immutable and learned dyadic facts are
        // monotonic, so one relaxed snapshot covers both decisions.
        let retained = self
            .retained_facts
            .load(std::sync::atomic::Ordering::Relaxed);
        if retained & RETAINED_UNREDUCED_INTERNAL != 0 {
            return self.canonicalized_ref().to_f64_lossy();
        }
        // Direct rational rounding can use borrowed dyadic paths or exact
        // quotient arithmetic. None lets callers select their fallback route.
        if self.sign == NoSign {
            return Some(0.0);
        }

        // Exact binary inputs dominate geometric filters. Combine the
        // numerator and denominator exponents before scaling so normal results
        // avoid an exact shifted-magnitude comparison, `powi`, and division.
        if let Some(denominator_shift) = self.dyadic_denominator_shift_from_retained(retained)
            && let Some(value) = self
                .exact_word_dyadic_f64_magnitude(denominator_shift)
                .or_else(|| self.normal_dyadic_f64_magnitude(denominator_shift))
        {
            return Some(match self.sign {
                Minus => -value,
                NoSign => 0.0,
                Plus => value,
            });
        }

        let msd = self.msd_exact()?;
        if msd > 1023 {
            return None;
        }

        // Operands of at most 53 bits convert exactly, so one IEEE division
        // rounds the quotient correctly. Wider operands, including unreduced
        // ones such as long decimals over a power of ten, would round twice
        // that way; round their exact quotient instead, so the result depends
        // only on the value.
        let value = if msd < -1075 {
            0.0
        } else if self.numerator.bits() <= 53 && self.denominator.bits() <= 53 {
            self.numerator.to_f64()? / self.denominator.to_f64()?
        } else {
            let (significand, ulp) = self.rounded_binary_magnitude(53, -1074);
            // The significand is at most 2^53, so it is exact in binary64,
            // and scaling by a power of two is exact while the result is
            // finite.
            (significand as f64) * f64_power_of_two(ulp)?
        };
        if !value.is_finite() {
            return None;
        }

        Some(match self.sign {
            Minus => -value,
            NoSign => 0.0,
            Plus => value,
        })
    }

    /// Round the magnitude to a binary significand by exact quotient/remainder.
    ///
    /// Returns `(q, u)` representing `q * 2^u`, rounded to nearest with ties to
    /// even and `u >= minimum_ulp_exponent`. With `1 <= precision <= 63`, a
    /// positive valid ratio and representable shift arithmetic, `q <= 2^precision`
    /// including a carry into the next binade. Existing callers use `(53, -1074)`
    /// and `(24, -149)` for the target formats' subnormal floors. Sign is ignored
    /// and valid unreduced ratios need no normalization.
    ///
    /// This is not a checked arbitrary-format API: a wider quotient can fail
    /// the `u64` conversion, precision 64 can overflow on rounding carry, and
    /// extreme exponent arithmetic is not validated. Shifted integer temporaries
    /// and division have no caller-supplied allocation/work limit.
    fn rounded_binary_magnitude(&self, precision: u32, minimum_ulp_exponent: i64) -> (u64, i64) {
        use num::Integer as _;

        let numerator = &self.numerator;
        let denominator = &self.denominator;
        // 2^(shift - 1) < |self| < 2^(shift + 1); one comparison fixes the
        // binary exponent.
        let shift = i64::try_from(numerator.bits()).unwrap_or(i64::MAX)
            - i64::try_from(denominator.bits()).unwrap_or(i64::MAX);
        let at_least = if shift >= 0 {
            *numerator >= (denominator << shift.unsigned_abs())
        } else {
            (numerator << shift.unsigned_abs()) >= *denominator
        };
        let exponent = if at_least { shift } else { shift - 1 };
        let ulp = (exponent - (i64::from(precision) - 1)).max(minimum_ulp_exponent);
        let (dividend, divisor) = if ulp >= 0 {
            (numerator.clone(), denominator << ulp.unsigned_abs())
        } else {
            (numerator << ulp.unsigned_abs(), denominator.clone())
        };
        let (quotient, remainder) = dividend.div_rem(&divisor);
        let mut significand = quotient
            .to_u64()
            .expect("the quotient has at most `precision` bits");
        let twice_remainder = remainder << 1_u32;
        if twice_remainder > divisor || (twice_remainder == divisor && significand & 1 == 1) {
            significand += 1;
        }
        (significand, ulp)
    }

    /// Round this exact rational directly to binary32, nearest with ties to even.
    ///
    /// No intermediate binary64 rounding is used. Zero becomes positive zero;
    /// negative underflow can yield negative zero. A nonfinite rounded result
    /// or ineligible scale returns `None`. Stored unreduced ratios are consumed
    /// directly by exact quotient/remainder arithmetic without canonicalizing.
    /// The conversion preserves the rational source and installs no exact-view
    /// fact; shifted temporaries have no total allocation/work budget.
    pub(crate) fn to_f32_correctly_rounded(&self) -> Option<f32> {
        if self.sign == NoSign {
            return Some(0.0);
        }
        // |self| >= 2^(bits(n) - bits(d) - 1); from 2^128 on, binary32 overflows.
        if i128::from(self.numerator.bits()) - i128::from(self.denominator.bits()) > 128 {
            return None;
        }
        let (significand, ulp) = self.rounded_binary_magnitude(24, -149);
        // At most 2^24, so exact in binary32; the power-of-two scale is exact
        // while the result is finite.
        let value = (significand as f32) * f32_power_of_two(ulp)?;
        if !value.is_finite() {
            return None;
        }
        Some(if self.sign == Minus { -value } else { value })
    }

    /// Convert a small positive dyadic magnitude exactly by normal binary scaling.
    ///
    /// The caller must provide the actual power-of-two denominator exponent.
    /// A numerator of at most 53 bits and exponent at most 1,022 permit an exact
    /// integer conversion and normal scaling. Other shapes, including an empty
    /// zero numerator, return `None`. Sign is ignored; a false exponent can
    /// produce a wrong value because this helper does not inspect the denominator.
    #[inline]
    fn exact_word_dyadic_f64_magnitude(&self, denominator_shift: u64) -> Option<f64> {
        let numerator_bits = self.numerator.bits();
        if numerator_bits > 53 || denominator_shift > 1022 {
            return None;
        }
        // Both conversions and the normal power-of-two scaling are exact
        // under these bounds; the signed cast avoids the wider u64 lowering.
        let numerator = self.numerator.iter_u64_digits().next()? as i64;
        let scale = f64::from_bits((1023 - denominator_shift) << 52);
        Some((numerator as f64) * scale)
    }

    /// Round a positive dyadic magnitude whose exact binade is binary64-normal.
    ///
    /// The caller supplies the true denominator exponent for a nonzero numerator.
    /// Binades outside -1,022 through 1,023 return `None`. A normalized 64-bit
    /// head with an odd sticky bit preserves the final binary64 rounding choice;
    /// normal power-of-two scaling then restores the magnitude. Overflow returns
    /// `None`. Sign is ignored and no exact-representability fact is installed.
    /// Zero or inconsistent metadata is outside the private contract.
    #[inline]
    fn normal_dyadic_f64_magnitude(&self, denominator_shift: u64) -> Option<f64> {
        let numerator_bits = self.numerator.bits();
        let exponent = i128::from(numerator_bits - 1) - i128::from(denominator_shift);
        if !(-1022..=1023).contains(&exponent) {
            return None;
        }

        let high = Self::normalized_high_u64_round_to_odd(&self.numerator, numerator_bits)?;
        let significand = (high as f64) * f64::from_bits((1023_u64 - 63) << 52);
        let scale_exponent = u64::try_from(exponent + 1023).ok()?;
        let value = significand * f64::from_bits(scale_exponent << 52);
        value.is_finite().then_some(value)
    }

    /// Normalize a positive integer to a 64-bit head with discarded bits jammed odd.
    ///
    /// `bits` must equal the integer's actual bit length. Up to 64 bits, left
    /// alignment is exact. Wider inputs keep the highest 64 bits and set their
    /// low bit if any discarded bit is nonzero. This preserves sticky information
    /// for subsequent narrower nearest-even rounding; it is not that final
    /// rounding itself. Zero returns `None`. The scan borrows digits and may
    /// inspect the full tail without allocating a shifted magnitude.
    #[inline]
    fn normalized_high_u64_round_to_odd(value: &BigUint, bits: u64) -> Option<u64> {
        let mut digits = value.iter_u64_digits();
        let highest = digits.next_back()?;
        if bits <= 64 {
            return Some(highest << (64 - bits));
        }

        let highest_bits = u64::from(64 - highest.leading_zeros());
        let (mut high, discarded) = if highest_bits == 64 {
            (highest, digits.any(|digit| digit != 0))
        } else {
            let next = digits.next_back().unwrap_or_default();
            let high = (highest << (64 - highest_bits)) | (next >> highest_bits);
            let discarded_mask = (1_u64 << highest_bits) - 1;
            (
                high,
                next & discarded_mask != 0 || digits.any(|digit| digit != 0),
            )
        };
        high |= u64::from(discarded);
        Some(high)
    }

    /// Return a finite binary64 value only if it equals this rational exactly.
    ///
    /// Lazy input canonicalizes first. Zero succeeds as positive zero. Other
    /// inputs need a power-of-two denominator, at most 53 significant bits after
    /// removing numerator zeros, and set bits within binary64's finite exponent
    /// range. Non-dyadic, overly precise, overflowing and underflowing values
    /// return `None`; representable subnormals are supported.
    ///
    /// Success records the exact-view fact on the canonical owner. The query
    /// can learn dyadic facts or allocate during canonicalization/subnormal
    /// construction; it has no total resource budget. No rounding is adopted as
    /// a replacement for the exact rational.
    #[inline]
    pub fn dyadic_to_f64_exact(&self) -> Option<f64> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().dyadic_to_f64_exact();
        }
        if self.is_zero() {
            self.mark_exact_f64_view();
            return Some(0.0);
        }

        let denominator_shift = self.dyadic_denominator_shift()?;
        if !self.is_exact_f64_with_denominator_shift(denominator_shift) {
            return None;
        }
        let magnitude = self
            .normal_dyadic_f64_magnitude(denominator_shift)
            .or_else(|| self.exact_subnormal_dyadic_f64_magnitude(denominator_shift))?;
        let value = if self.sign == Minus {
            -magnitude
        } else {
            magnitude
        };
        self.mark_exact_f64_view();
        Some(value)
    }

    /// Prove finite binary64 representability of a positive dyadic magnitude.
    ///
    /// The numerator must be nonzero and `denominator_shift` must describe the
    /// actual denominator. A small normal significand succeeds immediately;
    /// otherwise trailing-zero counting verifies at most 53 significant bits,
    /// a highest set-bit exponent at most 1,023 and a lowest at least -1,074.
    /// Sign is immaterial. No float round trip, allocation or fact publication
    /// occurs; invalid zero/scale metadata is not validated here.
    #[inline]
    fn is_exact_f64_with_denominator_shift(&self, denominator_shift: u64) -> bool {
        let numerator_bits = self.numerator.bits();
        let greatest_exponent =
            i128::from(numerator_bits - 1) - i128::from(denominator_shift);
        // Every normal binary value with at most 53 numerator bits fits the
        // binary64 significand. Its least bit is no lower than 2^-1074, so no
        // trailing-zero scan is needed on this dominant geometry path.
        if numerator_bits <= 53 && (-1022..=1023).contains(&greatest_exponent) {
            return true;
        }

        let numerator_shift = self
            .numerator
            .trailing_zeros()
            .expect("non-zero numerator has a trailing-zero count");
        let significand_bits = numerator_bits - numerator_shift;
        let least_exponent = i128::from(numerator_shift) - i128::from(denominator_shift);

        // A binary64 value carries at most 53 significant bits, its greatest
        // set bit cannot exceed 2^1023, and its least set bit cannot fall below
        // the subnormal quantum 2^-1074. These structural checks prove exact
        // representability without allocating a round-trip Rational.
        significand_bits <= 53 && greatest_exponent <= 1023 && least_exponent >= -1074
    }

    /// Construct a previously proved exactly representable subnormal magnitude.
    ///
    /// The caller supplies the true denominator exponent and must already have
    /// proved exact binary64 representability. The exact magnitude exponent must
    /// lie from -1,074 through -1,023. Scaling to subnormal integer units builds
    /// the float bits; conversion/range failures return `None`.
    ///
    /// Right shifts are exact only under the caller's representability proof.
    /// For example, an unproved 3/2^1075 truncates to one subnormal unit here,
    /// although nearest-even conversion yields two. This helper is not a general
    /// subnormal-rounding routine and can allocate shifted temporaries.
    fn exact_subnormal_dyadic_f64_magnitude(&self, denominator_shift: u64) -> Option<f64> {
        if !(-1074..=-1023).contains(&self.msd_exact()?) {
            return None;
        }
        let unit_shift = 1074_i128 - i128::from(denominator_shift);
        let units = if unit_shift >= 0 {
            &self.numerator << usize::try_from(unit_shift).ok()?
        } else {
            &self.numerator >> usize::try_from(-unit_shift).ok()?
        };
        let units = units.to_u64()?;
        (units != 0 && units <= (1_u64 << 52)).then(|| f64::from_bits(units))
    }

    /// Return finite binary64 bounds containing this exact rational, when available.
    ///
    /// For `Some([lo, hi])`, `lo <= self <= hi`. Lazy inputs canonicalize first.
    /// Zero and exactly representable dyadics get singleton intervals. Other
    /// normal dyadics use adjacent outward floats, with a separate exact check
    /// near maximum finite magnitude. The general path encloses leading-digit
    /// ratios and scales outward when the raw bit-count difference lies in
    /// -1,022 through 1,023.
    ///
    /// `None` means this schedule could not produce finite bounds; it is not a
    /// proof that no finite enclosure exists. In particular, many non-dyadic
    /// subnormals and boundary cases decline. Bounds need not be the tightest
    /// adjacent pair. These are certified bounds on the retained exact source,
    /// not permission to replace it with rounded geometry. Canonicalization and
    /// subnormal construction have no total work/memory budget.
    #[inline]
    pub fn to_f64_enclosure(&self) -> Option<[f64; 2]> {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().to_f64_enclosure();
        }
        if self.sign == NoSign {
            return Some([0.0, 0.0]);
        }
        if let Some(denominator_shift) = self.dyadic_denominator_shift() {
            let exact = self.is_exact_f64_with_denominator_shift(denominator_shift);
            let magnitude = self
                .normal_dyadic_f64_magnitude(denominator_shift)
                .or_else(|| {
                    if exact {
                        self.exact_subnormal_dyadic_f64_magnitude(denominator_shift)
                    } else {
                        None
                    }
                });
            if let Some(magnitude) = magnitude {
                let value = if self.sign == Minus {
                    -magnitude
                } else {
                    magnitude
                };
                return Some(if exact {
                    [value, value]
                } else if magnitude == f64::MAX {
                    return self.f64_enclosure_at_rounded_max();
                } else {
                    [value.next_down(), value.next_up()]
                });
            }
        }

        // Leading numerator/denominator words already enclose the ratio.
        // Their bit-length difference supplies an exact power-of-two scale;
        // locating the ratio's exact binade would add a full bigint comparison
        // without strengthening these outward bounds.
        let scale_exponent =
            i128::from(self.numerator.bits()) - i128::from(self.denominator.bits());
        if !(-1022..=1023).contains(&scale_exponent) {
            return None;
        }
        let scale_exponent = i32::try_from(scale_exponent).ok()?;
        let (lower, upper) = normalized_rational_magnitude_interval(self, scale_exponent)?;
        let scale = f64::from_bits(u64::try_from(scale_exponent + 1023).ok()? << 52);
        let lower = (lower * scale).next_down();
        let upper = (upper * scale).next_up();
        if !lower.is_finite() || !upper.is_finite() {
            return None;
        }
        Some(match self.sign {
            Minus => [-upper, -lower],
            NoSign => [0.0, 0.0],
            Plus => [lower, upper],
        })
    }

    /// Enclose an inexact dyadic whose nearest magnitude rounded to binary64 MAX.
    ///
    /// The caller establishes exact binade 1,023 and the rounded-MAX condition.
    /// Comparing the normalized sticky head against MAX's exact head distinguishes
    /// values above MAX, which return `None`, from values within the finite
    /// range. The latter use MAX and its predecessor with sign-correct ordering.
    /// Arbitrary input is not validated by this specialized boundary helper.
    #[cold]
    #[inline(never)]
    fn f64_enclosure_at_rounded_max(&self) -> Option<[f64; 2]> {
        // The exact exponent is 1023 here. Rounding to MAX does not prove
        // the value is at most MAX: compare the normalized integer head.
        // MAX has 53 one bits; the odd sticky bit distinguishes a discarded
        // positive tail from equality without constructing a large rational.
        /// MAX's exact 53-one-bit significand aligned to the 64-bit sticky head.
        const MAX_HEAD: u64 = ((1_u64 << 53) - 1) << 11;
        let head = Self::normalized_high_u64_round_to_odd(
            &self.numerator,
            self.numerator.bits(),
        )?;
        if head > MAX_HEAD {
            return None;
        }
        Some(if self.sign == Minus {
            [-f64::MAX, -f64::MAX.next_down()]
        } else {
            [f64::MAX.next_down(), f64::MAX]
        })
    }

    /// Read the owner's retained proof that its value has an exact binary64 view.
    ///
    /// False means no retained proof, not nonrepresentability. This reads a fact
    /// bit without canonicalizing or producing a float. The setter's caller is
    /// responsible for establishing the proof for this same immutable value.
    #[inline]
    pub(crate) fn has_exact_f64_view(&self) -> bool {
        self.retained_fact(RETAINED_EXACT_F64_VIEW)
    }

    /// Read retained eligibility for the direct rational binary64 filter route.
    ///
    /// The flag concerns a previously established approximation/error contract,
    /// not exact float representability. False is missing evidence. The query
    /// neither canonicalizes nor reconstructs the approximation or its bound.
    #[inline]
    pub(crate) fn has_relative_f64_filter_view(&self) -> bool {
        self.retained_fact(RETAINED_RELATIVE_F64_FILTER_VIEW)
    }

    /// Record the caller's proof that direct rational conversion meets its filter bound.
    ///
    /// The Real filter consumer first establishes a finite approximation and its
    /// error contract, then marks this immutable owner. This setter only retains
    /// a bit: it validates neither the approximation nor its error and stores
    /// no float, error interval or independent replay certificate.
    #[inline]
    pub(crate) fn mark_relative_f64_filter_view(&self) {
        self.retain_fact(RETAINED_RELATIVE_F64_FILTER_VIEW);
    }

    /// Cheaply reject canonical denominators that cannot be powers of two.
    ///
    /// Lazy input canonicalizes. A retained dyadic decision is reused and traced;
    /// otherwise denominator one or an even denominator passes. True is only
    /// eligibility until a complete dyadic query establishes the fact: an even
    /// nonpower such as six passes cold but fails after that query. This helper
    /// does not itself establish an unknown dyadic decision.
    #[inline]
    pub(crate) fn denominator_could_be_dyadic(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().denominator_could_be_dyadic();
        }
        if self.retained_fact(RETAINED_DYADIC_KNOWN) {
            crate::trace_dispatch!("rational", "retained-facts", "dyadic-prefilter-hit");
            return self.retained_fact(RETAINED_DYADIC_VALUE);
        }
        self.denominator == *ONE || !self.denominator.bit(0)
    }

    /// Retain the caller's proof of exact finite binary64 representability.
    ///
    /// This sets a fact bit on the immutable owner without checking the rational
    /// or storing a float. Only callers with an exact import or conversion proof
    /// may use it; a fabricated flag is not independent validation evidence.
    #[inline]
    pub(crate) fn mark_exact_f64_view(&self) {
        self.retain_fact(RETAINED_EXACT_F64_VIEW);
    }

    /// Return whether the canonical rational has a nonterminating decimal expansion.
    ///
    /// After canonicalizing lazy input, remove denominator factors of ten, five
    /// and two. A remaining factor means nontermination; integers and zero return
    /// false. Sign has no effect. This is a presentation choice, not permission
    /// to round. Denominator cloning and repeated exact divisions have no local
    /// work/allocation budget.
    ///
    /// # Example
    ///
    /// ```
    /// use hyperreal::Rational;
    /// let third = Rational::fraction(1, 3).unwrap();
    /// assert!(third.prefer_fraction());
    /// ```
    pub fn prefer_fraction(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().prefer_fraction();
        }
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

    /// Return `trunc(self * 2^shift)` as a signed arbitrary-width integer.
    ///
    /// Positive shifts expand the numerator and negative shifts expand the
    /// denominator before unsigned division and sign restoration. Truncation is
    /// toward zero, including for negative values. Valid unreduced parts are
    /// consumed directly without canonicalization. Accepted `i32` shifts can
    /// allocate large integers; there is no typed exhaustion or cancellation.
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
        let whole = if shift >= 0 {
            (&self.numerator << shift as usize) / &self.denominator
        } else {
            &self.numerator / (&self.denominator << shift.unsigned_abs() as usize)
        };
        BigInt::from_biguint(self.sign, whole)
    }

    /// Return the tight exact enclosing pair on the grid of spacing `2^precision`.
    ///
    /// The endpoints are `floor(self/grid)*grid` and `ceil(self/grid)*grid`, with
    /// sign-correct ordering and a singleton for values already on the grid.
    /// Zero returns canonical zeros. Reduced dyadics on a nonpositive-precision
    /// grid can reuse their shared handles without division; other valid ratios,
    /// including unreduced ones, use exact quotient/remainder and reconstruction.
    ///
    /// This constructs rational endpoints, not approximate floats. Large shifts
    /// and result reductions have no total work/memory limit or cancellation;
    /// the internal expect relies on a power-of-two denominator being nonzero.
    pub(crate) fn enclosing_dyadic_interval(&self, precision: i32) -> [Self; 2] {
        if self.is_zero() {
            return [Self::zero(), Self::zero()];
        }
        if precision <= 0
            && self
                .dyadic_denominator_shift_if_reduced()
                .is_some_and(|shift| shift <= u64::from(precision.unsigned_abs()))
        {
            // Already on this grid: preserve the shared exact payload without
            // constructing shifted integers or performing a division.
            return [self.clone(), self.clone()];
        }
        let (floor, remainder) = if precision < 0 {
            num::Integer::div_rem(
                &(&self.numerator << precision.unsigned_abs() as usize),
                &self.denominator,
            )
        } else {
            num::Integer::div_rem(
                &self.numerator,
                &(&self.denominator << precision as usize),
            )
        };
        let ceil = if remainder.is_zero() {
            floor.clone()
        } else {
            &floor + BigUint::one()
        };
        let restore = |magnitude| {
            let numerator = BigInt::from_biguint(self.sign, magnitude);
            if precision < 0 {
                Self::from_bigint_fraction(
                    numerator,
                    BigUint::one() << precision.unsigned_abs() as usize,
                )
                .expect("a dyadic grid has a nonzero denominator")
            } else {
                Self::from_bigint(numerator << precision as usize)
            }
        };
        if self.sign == Minus {
            [restore(ceil), restore(floor)]
        } else {
            [restore(floor), restore(ceil)]
        }
    }

    /// Compare the absolute values exactly without constructing signed rationals.
    ///
    /// Equal stored denominators compare numerators directly. Equal numerators
    /// wider than 64 bits reverse denominator order. All other cases allocate
    /// cross-products. These comparisons support valid unreduced ratios without
    /// canonicalization; the helper is not universally allocation-free and has
    /// no explicit work or memory budget.
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
            // used in exact geometric computation.
            return other.denominator.cmp(&self.denominator);
        }
        (&self.numerator * &other.denominator).cmp(&(&other.numerator * &self.denominator))
    }

    /// Compare `abs(self)² * abs(factor)` with `abs(other)` exactly.
    ///
    /// All signs are ignored. Consumers intending the signed expression must
    /// establish that factor and other are nonnegative, as the sqrt-domain gates
    /// do. Stored cross-products support valid unreduced fractions without an
    /// intermediate Rational or canonicalization, but allocate arbitrary-width
    /// integers and have no total-work or memory bound.
    #[inline]
    pub(crate) fn compare_magnitude_squared_times(&self, factor: &Self, other: &Self) -> Ordering {
        let left = &self.numerator * &self.numerator * &factor.numerator * &other.denominator;
        let right = &self.denominator * &self.denominator * &factor.denominator * &other.numerator;
        left.cmp(&right)
    }

    /// Construct exact rational facts from canonical parts for downstream dispatch.
    ///
    /// Lazy input obtains its cached reduced value. The result identifies integer,
    /// dyadic and nonzero absolute power-of-two forms and the canonical size bucket.
    /// The small-integer flag conservatively requires at most 63 magnitude bits,
    /// so it excludes `i64::MIN` even though that value fits the signed type.
    /// Flags describe this canonical snapshot, not cache warmth or a serialized
    /// proof; obtaining it can allocate during normalization.
    #[inline]
    pub(crate) fn detailed_rational_facts(&self) -> RationalFacts {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().detailed_rational_facts();
        }
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

    /// Return the exact signed integer, or `None` when the value is fractional.
    ///
    /// Lazy input canonicalizes through the borrowed integer-magnitude query.
    /// Success clones that magnitude into a `BigInt`, without a primitive-width
    /// restriction; normalization and allocation have no local resource budget.
    pub fn to_big_integer(&self) -> Option<BigInt> {
        self.integer_magnitude()
            .map(|magnitude| BigInt::from_biguint(self.sign, magnitude.clone()))
    }

    /// Borrow the canonical unsigned magnitude only when the denominator is one.
    ///
    /// Lazy storage canonicalizes and may allocate a cached reduced payload.
    /// Zero returns its zero magnitude; nonintegers return `None`. The borrow
    /// avoids constructing a signed `BigInt` and remains tied to this owner.
    pub(crate) fn integer_magnitude(&self) -> Option<&BigUint> {
        // Integer-only callers usually need the non-negative magnitude. Return
        // the borrowed BigUint so they can avoid constructing a signed BigInt.
        let canonical = self.canonicalized_ref();
        (canonical.denominator == *ONE.deref()).then_some(&canonical.numerator)
    }

    /// Recognize an integer whose unsigned magnitude fits positive `i64` range.
    ///
    /// The integer query canonicalizes lazy input, converts the magnitude first,
    /// then restores sign. Fractional values and magnitudes above `i64::MAX`
    /// return `None`, including `i64::MIN`. Thus failure is not an exhaustive
    /// signed-range test; consumers use success as a small-integer shortcut.
    pub(crate) fn to_integer_i64(&self) -> Option<i64> {
        let magnitude = self.integer_magnitude()?.to_i64()?;
        match self.sign {
            Plus | NoSign => Some(magnitude),
            Minus => magnitude.checked_neg(),
        }
    }

    /// Return the exact stored sign, with `NoSign` denoting zero.
    ///
    /// Valid construction maintains this sign even for lazy fractions. No
    /// canonicalization, approximation, allocation or cache mutation is needed.
    #[inline]
    pub fn sign(&self) -> Sign {
        self.sign
    }

    /// Return whether this exact rational is strictly negative from its stored sign.
    ///
    /// Zero fails. The query does not allocate, reduce or consult numeric filters.
    #[inline]
    pub fn is_negative(&self) -> bool {
        self.sign == Minus
    }

    /// Return whether this exact rational is strictly positive from its stored sign.
    ///
    /// Zero fails. The query does not allocate, reduce or consult numeric filters.
    #[inline]
    pub fn is_positive(&self) -> bool {
        self.sign == Plus
    }

}

/// Construct exact binary64 `2^exponent`, including subnormal powers.
///
/// Exponents from -1,074 through 1,023 produce positive finite values by bit
/// construction. All others return `None`, including underflowing powers;
/// there is no rounding to zero or infinity and no allocation.
fn f64_power_of_two(exponent: i64) -> Option<f64> {
    match exponent {
        -1022..=1023 => Some(f64::from_bits(u64::try_from(exponent + 1023).ok()? << 52)),
        -1074..=-1023 => Some(f64::from_bits(1_u64 << (exponent + 1074))),
        _ => None,
    }
}

/// Construct exact binary32 `2^exponent`, including subnormal powers.
///
/// Exponents from -149 through 127 produce positive finite values by bit
/// construction. All others return `None`, including underflowing powers;
/// there is no rounding to zero or infinity and no allocation.
fn f32_power_of_two(exponent: i64) -> Option<f32> {
    match exponent {
        -126..=127 => Some(f32::from_bits(u32::try_from(exponent + 127).ok()? << 23)),
        -149..=-127 => Some(f32::from_bits(1_u32 << (exponent + 149))),
        _ => None,
    }
}
