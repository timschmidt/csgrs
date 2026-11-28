/// Construct exact signed fractions, retain reciprocal relations and recover canonical facts.
///
/// Public fraction constructors reject zero denominators. Private entry points
/// distinguish already reduced parts, marked lazy reduction and transient
/// arithmetic parts; their callers own any omitted validation/reduction proofs.
/// These routines preserve exact integers but have no general work, allocation
/// or cancellation budget. Static pools and object-local caches retain results
/// under the storage contracts described with their definitions.
impl Rational {
    /// Allocate a fresh payload from parts without validation, normalization or tracing.
    ///
    /// Installs empty caches and zero retained bits, including no unreduced marker.
    /// The caller must supply a positive denominator and consistent sign/zero state,
    /// and establish reduced parts before consumers that assume them. Transient
    /// unreduced arithmetic results must be reduced before such consumption; use
    /// the marked constructor when common factors intentionally remain observable.
    /// No pool lookup, GCD, typed error or resource check occurs here.
    #[inline]
    fn from_parts_raw(sign: Sign, numerator: BigUint, denominator: BigUint) -> Self {
        Self(Arc::new(RationalData {
            sign,
            numerator,
            denominator,
            product_cache: OnceLock::new(),
            linear_cache: CompactOnceBox::new(),
            retained_facts: std::sync::atomic::AtomicU32::new(0),
        }))
    }

    /// Allocate exact parts with the explicit internal-unreduced marker.
    ///
    /// The caller supplies a nonzero denominator and a sign consistent with the
    /// numerator; a zero numerator may retain a nonunit denominator until normalized.
    /// No reduction, validation, pool lookup or trace event occurs. Empty caches
    /// reserve the primary product slot for later canonicalization, allowing consumers
    /// to retain common factors without falsely treating raw parts as reduced.
    #[inline]
    pub(crate) fn from_parts_raw_unreduced(
        sign: Sign,
        numerator: BigUint,
        denominator: BigUint,
    ) -> Self {
        Self(Arc::new(RationalData {
            sign,
            numerator,
            denominator,
            product_cache: OnceLock::new(),
            linear_cache: CompactOnceBox::new(),
            retained_facts: std::sync::atomic::AtomicU32::new(RETAINED_UNREDUCED_INTERNAL),
        }))
    }

    /// Read the retained representation marker without checking for actual factors.
    ///
    /// True permits unreduced parts; it does not assert that a common factor exists.
    /// False trusts the construction path rather than recomputing a GCD.
    #[inline]
    pub(crate) fn is_internally_unreduced(&self) -> bool {
        self.retained_fact(RETAINED_UNREDUCED_INTERNAL)
    }

    /// Borrow self if unmarked, otherwise publish and borrow its canonical fraction.
    ///
    /// The reserved product-cache slot initializes once with reduced cloned parts,
    /// `other: None` and a lazy-canonicalization trace event. Concurrent callers share
    /// that initialized result. The original parts and marker remain unchanged.
    /// An unmarked value is not revalidated or reduced here; correctness relies on
    /// constructor invariants. The reserved-slot role is debug-asserted, not repaired.
    /// Reduction/cloning can allocate and has no local resource or cancellation bound.
    pub(crate) fn canonicalized_ref(&self) -> &Self {
        if !self.is_internally_unreduced() {
            return self;
        }
        let cached = self.product_cache.get_or_init(|| {
            crate::trace_dispatch!(
                "rational",
                "canonicalization",
                "lazy-internal-coordinate"
            );
            CachedRationalProduct {
                other: None,
                result: Self::from_fraction_parts_reduced(
                    self.sign,
                    self.numerator.clone(),
                    self.denominator.clone(),
                ),
            }
        });
        debug_assert!(cached.other.is_none());
        &cached.result
    }

    /// Consume a handle and return its stored sign, numerator and denominator.
    ///
    /// With one strong owner, move the limbs out even if weak references exist;
    /// otherwise clone the limbs and release this handle. Caches and retained bits
    /// are not returned. This does not canonicalize marked unreduced input, so
    /// callers must establish any reduction invariant needed by their next operation.
    fn into_parts(self) -> (Sign, BigUint, BigUint) {
        let RationalData {
            sign,
            numerator,
            denominator,
            product_cache: _,
            linear_cache: _,
            retained_facts: _,
        } = Arc::try_unwrap(self.0).unwrap_or_else(|shared| RationalData {
            sign: shared.sign,
            numerator: shared.numerator.clone(),
            denominator: shared.denominator.clone(),
            product_cache: OnceLock::new(),
            linear_cache: CompactOnceBox::new(),
            retained_facts: std::sync::atomic::AtomicU32::new(0),
        });
        (sign, numerator, denominator)
    }

    /// Look up a previously certified reciprocal through eligible cache slots.
    ///
    /// Checks primary, tertiary, quaternary then quinary; secondary is reserved for
    /// binary entries. Strong reciprocal tags clone their result, weak tags upgrade
    /// their target, and unrelated tags are skipped. An expired reciprocal encountered
    /// in an earlier slot returns None immediately rather than searching later slots.
    /// No arithmetic proof, zero check, publication or trace event occurs here.
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

    /// Try to retain a caller-proved reciprocal relation with strong or weak ownership.
    ///
    /// A primary reciprocal tag rejects another entry, including when its weak target
    /// has expired. Other primary placeholders send late entries to quaternary then
    /// quinary; a binary primary tries tertiary, quaternary then quinary. With no
    /// cache, publish a new box whose primary encodes the relation. Weak entries
    /// store an inert zero result; strong entries use an empty weak key.
    ///
    /// Returns true only when this attempt installs an entry. Full slots or a losing
    /// box-publication race return false, without replacement or a retry in the newly
    /// published box. Nonprimary duplicate tags are not deduplicated here. The helper
    /// does not validate reciprocity or ownership-cycle freedom. A strong target
    /// must not own a path back to this owner. Allocation and strong result
    /// retention are not governed by a byte budget.
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

    /// Best-effort installation of a proved reciprocal pair without a strong cycle.
    ///
    /// Try the inverse's weak backlink first, then this owner's strong forward link.
    /// Publication failures are ignored independently, so the pair need not become
    /// cached in both directions. The caller supplies a fresh reciprocal payload,
    /// distinct from this owner, so the forward link cannot create a strong cycle.
    /// Cache rejection changes reuse opportunities, not the arithmetic value.
    #[cold]
    fn retain_inverse_pair(&self, inverse: &Self) {
        let _ = inverse.retain_inverse_entry(CachedRationalUnary::Weak(Arc::downgrade(&self.0)));
        let _ = self.retain_inverse_entry(CachedRationalUnary::Strong(inverse.clone()));
    }

    /// Return shared canonical zero, the additive identity.
    ///
    /// Clones the process-lifetime zero handle and emits a rational-temporary event
    /// when tracing is enabled; that event is not evidence of a new allocation.
    pub fn zero() -> Self {
        trace_rational_temporary!();
        RATIONAL_ZERO.clone()
    }

    /// Return shared canonical positive one, the multiplicative identity.
    ///
    /// Clones the process-lifetime unit handle and emits the temporary event without
    /// copying integer limbs. First access may initialize the static value.
    pub fn one() -> Self {
        trace_rational_temporary!();
        RATIONAL_ONE.clone()
    }

    /// Return the shared negative unit and submit a rational-temporary trace event.
    /// No general fraction reduction or integer-limb clone is required.
    fn minus_one() -> Self {
        trace_rational_temporary!();
        RATIONAL_MINUS_ONE.clone()
    }

    /// Look up signed magnitudes 2 through 64 in lazy process-lifetime intern slots.
    ///
    /// Requires magnitude >= 2 (debug-asserted); larger than 64 returns None. Index
    /// is magnitude minus two. Eligible Plus/Minus values initialize or clone their
    /// separate slot and emit a temporary event; NoSign returns canonical zero.
    /// Out-of-domain small magnitudes can underflow index arithmetic when assertions
    /// are disabled. Slots and their retained arithmetic evidence have no eviction.
    fn small_integer(sign: Sign, magnitude: u128) -> Option<Self> {
        debug_assert!(magnitude >= 2);
        if magnitude > 64 {
            return None;
        }
        let index = magnitude as usize - 2;
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

    /// Intern eligible reduced fractions with odd numerator <=63 and denominator 2^s.
    ///
    /// Requires a nonzero numerator and a power-of-two denominator greater than one
    /// (debug assertions). Even or oversized numerators and s>63 return None. The
    /// index is (s-1)*32 + (numerator>>1); signs use separate pools, and eligible
    /// NoSign maps to zero. A hit clones the static handle and emits a temporary
    /// event. These unchecked representation assumptions are essential to cache-key
    /// correctness; arbitrary invalid denominators must not reach this helper.
    fn small_reduced_dyadic(sign: Sign, magnitude: u128, denominator: u128) -> Option<Self> {
        debug_assert_ne!(magnitude, 0);
        debug_assert!(denominator > 1 && denominator.is_power_of_two());
        if magnitude > 63 || magnitude & 1 == 0 {
            return None;
        }
        let shift = denominator.trailing_zeros() as usize;
        if shift > SMALL_DYADIC_MAX_SHIFT {
            return None;
        }
        let magnitude_index = (magnitude >> 1) as usize;
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

    /// Intern an already reduced, nonzero non-dyadic fraction within the 63 by 63 grid.
    ///
    /// Requires denominator>1, not a power of two, and gcd(magnitude,denominator)=1.
    /// Only the nonzero/denominator-shape conditions are debug-asserted; reduction
    /// is trusted. Either component exceeding 63 returns None. Index is
    /// (denominator-1)*63 + magnitude-1; signs select separate static pools, and
    /// eligible NoSign returns zero. Successful lookup emits a temporary event.
    /// There is no eviction, and invalid zero components can underflow indexing.
    fn small_reduced_general_fraction(
        sign: Sign,
        magnitude: u128,
        denominator: u128,
    ) -> Option<Self> {
        debug_assert_ne!(magnitude, 0);
        debug_assert!(denominator > 1 && !denominator.is_power_of_two());
        if magnitude > SMALL_GENERAL_MAX_MAGNITUDE as u128
            || denominator > SMALL_GENERAL_MAX_DENOMINATOR as u128
        {
            return None;
        }
        let magnitude_index = magnitude as usize - 1;
        let denominator_index = denominator as usize - 1;
        let index = denominator_index * SMALL_GENERAL_MAX_MAGNITUDE + magnitude_index;
        let values = match sign {
            Plus => &SMALL_POSITIVE_GENERAL_RATIONALS,
            Minus => &SMALL_NEGATIVE_GENERAL_RATIONALS,
            NoSign => return Some(Self::zero()),
        };
        let value = values[index].get_or_init(|| {
            Self::from_parts_raw(
                sign,
                BigUint::from(magnitude),
                BigUint::from(denominator),
            )
        });
        trace_rational_temporary!();
        Some(value.clone())
    }

    /// Construct the exact signed integer n, including the full i64 range.
    ///
    /// Uses unsigned_abs, so i64::MIN is preserved without signed-negation overflow.
    /// Zero, units and magnitudes through 64 use shared static values; larger values
    /// construct integer magnitudes directly without an intermediate BigInt.
    ///
    /// ```
    /// use hyperreal::Rational;
    /// assert_eq!(Rational::new(-5), Rational::fraction(-10, 2).unwrap());
    /// assert_eq!(Rational::new(i64::MIN).to_string(), "-9223372036854775808");
    /// ```
    pub fn new(n: i64) -> Self {
        // Small scalar constructors are hot. Rational is stored as
        // Sign+BigUint, so going through BigInt first only adds allocation and
        // sign extraction work.
        let sign = if n < 0 { Minus } else { Plus };
        Self::from_primitive_integer(sign, u128::from(n.unsigned_abs()))
    }

    /// Construct a signed integer from a bounded unsigned magnitude.
    ///
    /// Callers use Plus/Minus for nonzero magnitudes; zero ignores sign. Units and
    /// small integers reuse pools before the BigUint fallback. NoSign with nonzero
    /// magnitude is outside that contract: magnitude one takes the positive-unit
    /// branch while other magnitudes may normalize to zero. This routine does not
    /// validate that inconsistent sign/magnitude combination.
    pub(crate) fn from_primitive_integer(sign: Sign, magnitude: u128) -> Self {
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
        if let Some(value) = Self::small_integer(sign, magnitude) {
            return value;
        }
        Self::from_integer_magnitude(sign, BigUint::from(magnitude))
    }

    /// Consume a signed arbitrary-width integer and give it denominator one.
    ///
    /// The fraction constructor preserves sign and magnitude exactly. Its unwrap
    /// is justified by the supplied nonzero denominator; zero becomes canonical.
    /// This path does not generally intern small nonzero integers or impose a
    /// resource budget.
    pub fn from_bigint(n: BigInt) -> Self {
        Self::from_bigint_fraction(n, BigUint::one()).unwrap()
    }

    /// Construct the exact signed fraction n/d and remove common factors.
    ///
    /// Rejects d=0 as DivideByZero before zero-numerator normalization, including
    /// 0/0. The full i64 numerator range is accepted through unsigned_abs. Dyadic
    /// denominators use shift cancellation; other nonunit denominators use a GCD.
    /// Reduction is exact but has no local total-work, memory or cancellation limit.
    ///
    /// ```
    /// use hyperreal::{Problem, Rational};
    /// assert_eq!(Rational::fraction(-9, 18).unwrap(), "-0.5".parse::<Rational>().unwrap());
    /// assert_eq!(Rational::fraction(0, 0), Err(Problem::DivideByZero));
    /// ```
    pub fn fraction(n: i64, d: u64) -> Result<Self, Problem> {
        if d == 0 {
            return Err(Problem::DivideByZero);
        }
        let sign = if n < 0 { Minus } else { Plus };
        // The storage type is already Sign+BigUint, so unsigned_abs gives the
        // exact magnitude type and avoids a temporary signed BigInt.
        let numerator = BigUint::from(n.unsigned_abs());
        let denominator = BigUint::from(d);
        Ok(Self::from_fraction_parts_reduced(
            sign,
            numerator,
            denominator,
        ))
    }

    /// Consume a signed numerator and unsigned denominator to form a reduced fraction.
    ///
    /// Rejects denominator zero as DivideByZero even when the numerator is zero.
    /// Otherwise moves the BigInt's sign/magnitude into exact reduction; arbitrary
    /// finite limb widths are admitted subject to available resources. No total-work
    /// or allocation bound is enforced through the Result.
    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Result<Self, Problem> {
        if denominator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        let (sign, numerator) = n.into_parts();
        Ok(Self::from_fraction_parts_reduced(
            sign,
            numerator,
            denominator,
        ))
    }

    /// Construct an integer from a sign and owned unsigned magnitude.
    ///
    /// Magnitude one uses the shared signed unit or zero for NoSign. Other values
    /// use denominator one and the zero-normalizing parts constructor, which also
    /// turns NoSign into zero. Magnitudes 2..=64 are not interned by this path.
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

    /// Consume an unsigned magnitude as a nonnegative exact integer.
    /// Uses the integer-magnitude constructor's unit and zero normalization.
    pub(crate) fn from_unsigned_integer(numerator: BigUint) -> Self {
        Self::from_integer_magnitude(Plus, numerator)
    }

    /// Construct sign*numerator/2^denominator_shift from a proven reduced word.
    ///
    /// Requires Plus/Minus, nonzero numerator, and an odd numerator when the shift
    /// is nonzero; these are debug assertions, not typed input validation. IEEE
    /// float decoding supplies these proofs. Zero-shift small integers and eligible
    /// small dyadics reuse static pools; other cases allocate exact numerator and
    /// power-of-two denominator parts without another reduction.
    ///
    /// Retains known-dyadic bits and an exponent encoding when it fits the packed
    /// field. Larger shifts still construct exact denominators but omit the encoding.
    /// The u32 shift admits potentially large allocations with no local resource cap;
    /// the float callers normally supply much narrower exponents.
    pub(super) fn from_reduced_dyadic_word(
        sign: Sign,
        numerator: u64,
        denominator_shift: u32,
    ) -> Self {
        debug_assert_ne!(sign, NoSign);
        debug_assert_ne!(numerator, 0);
        debug_assert!(denominator_shift == 0 || numerator.trailing_zeros() == 0);
        let retained = RETAINED_DYADIC_KNOWN
            | RETAINED_DYADIC_VALUE
            | Self::encoded_dyadic_denominator_shift(u64::from(denominator_shift));
        // Primitive float decoding reaches this constructor with a reduced
        // word numerator. Reuse the existing bounded canonical values before
        // allocating a duplicate RationalData and two duplicate BigUints.
        let cached = if denominator_shift == 0 {
            if numerator == 1 {
                Some(if sign == Minus {
                    Self::minus_one()
                } else {
                    Self::one()
                })
            } else {
                Self::small_integer(sign, u128::from(numerator))
            }
        } else if numerator <= 63
            && denominator_shift
                <= u32::try_from(SMALL_DYADIC_MAX_SHIFT).expect("small dyadic shift fits u32")
        {
            Self::small_reduced_dyadic(
                sign,
                u128::from(numerator),
                1_u128 << denominator_shift,
            )
        } else {
            None
        };
        if let Some(value) = cached {
            value.retain_fact(retained);
            return value;
        }
        // IEEE-754 decoding has already stripped every common power of two.
        // Entering through the general fraction constructor would rebuild a
        // signed BigInt and re-check reduction facts that are known here.
        let value = Self::from_parts_raw(
            sign,
            BigUint::from(numerator),
            BigUint::one() << denominator_shift,
        );
        value.retain_fact(retained);
        trace_rational_temporary!();
        value
    }

    /// Normalize zero/sign and otherwise allocate unmarked parts without reducing.
    ///
    /// NoSign or zero numerator returns canonical zero before examining denominator.
    /// For nonzero values, emits a temporary event and delegates to raw allocation.
    /// The caller must validate a positive denominator and prove reduction or reduce
    /// the transient result before invariant-dependent consumers. It is not a
    /// fallible validation boundary; even a zero denominator can pass unchecked.
    fn from_fraction_parts(sign: Sign, numerator: BigUint, denominator: BigUint) -> Self {
        if sign == NoSign || numerator.is_zero() {
            return Self::zero();
        }
        trace_rational_temporary!();
        Self::from_parts_raw(sign, numerator, denominator)
    }

    /// Reduce validated positive-denominator parts and normalize zero.
    ///
    /// NoSign or zero numerator returns canonical zero immediately. A nonunit
    /// power-of-two denominator cancels the smaller binary valuation; other shapes
    /// use exact GCD and divide when needed. Emits reduction/factor/division and
    /// temporary trace events along the chosen path, then allocates unmarked parts.
    ///
    /// Denominator positivity is a caller precondition, not checked here: a zero
    /// denominator can become invalid stored state. Conversion of a binary shift to
    /// usize is asserted to fit the represented operands. Allocation, GCD and
    /// division have no typed work/memory exhaustion or cancellation contract.
    fn from_fraction_parts_reduced(
        sign: Sign,
        mut numerator: BigUint,
        mut denominator: BigUint,
    ) -> Self {
        if sign == NoSign || numerator.is_zero() {
            return Self::zero();
        }
        if denominator != *ONE.deref() {
            trace_rational_reduction!(&numerator, &denominator);
            if let Some(denominator_shift) = Self::biguint_power_of_two_shift(&denominator) {
                let common_shift = numerator
                    .trailing_zeros()
                    .expect("nonzero numerator has trailing zeros")
                    .min(denominator_shift);
                if common_shift != 0 {
                    let shift = usize::try_from(common_shift)
                        .expect("dyadic reduction shift fits usize");
                    numerator >>= shift;
                    denominator >>= shift;
                }
                trace_rational_power_of_two_common_factor!(common_shift);
            } else {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&numerator, &denominator);
                trace_rational_gcd!(&numerator, &denominator, &divisor);
                if divisor != *ONE.deref() {
                    trace_rational_division_algorithm!(
                        "reduction-numerator",
                        &numerator,
                        &divisor
                    );
                    trace_rational_division_algorithm!(
                        "reduction-denominator",
                        &denominator,
                        &divisor
                    );
                    numerator /= &divisor;
                    denominator /= divisor;
                }
            }
        }
        trace_rational_temporary!();
        Self::from_parts_raw(sign, numerator, denominator)
    }

    /// Add exact positive one while preserving a reduced denominator.
    ///
    /// Canonicalizes marked input first, handles zero and signed crossings, then
    /// uses gcd(n+d,d)=gcd(n,d) to avoid fresh reduction. Negative values compare
    /// numerator with denominator to choose the resulting sign or canonical zero.
    /// Validity/reduction is inherited from the source; no work budget is imposed.
    pub(crate) fn add_one(&self) -> Self {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().add_one();
        }
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

    /// Subtract exact positive one while preserving a reduced denominator.
    ///
    /// Canonicalizes marked input first. Zero becomes minus one; positive values
    /// compare numerator with denominator to decide their sign or zero, and negative
    /// values increase magnitude. The identity gcd(n-d,d)=gcd(n,d) justifies omitting
    /// a GCD for valid reduced input.
    pub(crate) fn subtract_one(&self) -> Self {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().subtract_one();
        }
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

    /// Fully reduce a valid transient fraction, preferring dyadic shift cancellation.
    ///
    /// A power-of-two denominator is cloned as the cancellation hint and traced
    /// before the shift path; other denominators call reduce. Despite the name,
    /// this is not an optional approximation. No-op paths can return the original
    /// handle and metadata rather than allocating a newly unmarked payload.
    fn maybe_reduce(self) -> Self {
        if Self::is_power_of_two(&self.denominator) {
            let denominator = self.denominator.clone();
            trace_rational_reduction!(&self.numerator, &self.denominator);
            // Binary64-derived dyadics and trig reduction scales dominate. When the
            // denominator is a power of two, remove common factors with shifts instead of a
            // full BigInt gcd.
            self.reduce_by_power_of_two_divisor(&denominator)
        } else {
            self.reduce()
        }
    }

    /// Reduce using a caller-proved bound on the possible common factor.
    ///
    /// The positive hint must divide the denominator and contain every factor that
    /// can cancel with the numerator; then gcd(numerator,hint) equals the needed
    /// full GCD. Reduced-input addition/subtraction use their denominator GCD, and
    /// the mean uses twice that GCD. These proofs are not checked here.
    ///
    /// Zero normalizes first; denominator one or hint one returns self. Power-of-two
    /// hints use shifts, otherwise only numerator versus hint is gcd-reduced. A
    /// valid but incomplete hint can leave unreduced parts, and a false divisor
    /// hint can truncate the denominator and change the value. This is not a
    /// public validation boundary. No-op paths preserve the original metadata;
    /// new reduced payloads start with empty caches and unmarked parts.
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

        let divisor =
            Self::gcd_magnitudes_with_mixed_width_fast_path(&self.numerator, possible_divisor);
        trace_rational_gcd!(&self.numerator, possible_divisor, &divisor);
        if divisor == *ONE.deref() {
            self
        } else {
            trace_rational_division_algorithm!("reduction-numerator", &self.numerator, &divisor);
            trace_rational_division_algorithm!(
                "reduction-denominator",
                &self.denominator,
                &divisor
            );
            trace_rational_temporary!();
            Self::from_parts_raw(
                self.sign,
                &self.numerator / &divisor,
                &self.denominator / divisor,
            )
        }
    }

    /// Remove the exact common factor from valid signed fraction parts.
    ///
    /// Denominator one returns self. Dyadic denominators use the binary-shift
    /// reducer; otherwise compute gcd(numerator,denominator) and divide both when
    /// it exceeds one. A GCD of one reuses self, so this routine does not always
    /// clear an internal-unreduced marker or discard metadata. It assumes a valid
    /// positive denominator and consistent sign; it is not arbitrary-state repair.
    /// GCD/division and temporary events preserve their existing execution order.
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

        let divisor =
            Self::gcd_magnitudes_with_mixed_width_fast_path(&self.numerator, &self.denominator);
        trace_rational_gcd!(&self.numerator, &self.denominator, &divisor);
        if divisor == *ONE.deref() {
            self
        } else {
            trace_rational_division_algorithm!("reduction-numerator", &self.numerator, &divisor);
            trace_rational_division_algorithm!(
                "reduction-denominator",
                &self.denominator,
                &divisor
            );
            let numerator = &self.numerator / &divisor;
            let denominator = &self.denominator / &divisor;
            trace_rational_temporary!();
            Self::from_parts_raw(self.sign, numerator, denominator)
        }
    }

    /// Return k exactly when the positive integer equals 2^k; zero returns None.
    ///
    /// One returns Some(0). The equality of trailing-zero count and bit length minus
    /// one proves there is only one set bit, using borrowed digits without division
    /// or a shifted allocation. Finding trailing zeros can scan stored limbs;
    /// the nonzero guard justifies the trailing_zeros expect.
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

    /// Test the exact single-set-bit certificate, excluding zero and including one.
    /// Delegates to the borrowed exponent recognizer without changing retained facts.
    fn is_power_of_two(value: &BigUint) -> bool {
        Self::biguint_power_of_two_shift(value).is_some()
    }

    /// Cancel binary factors using a caller-proved power-of-two divisor hint.
    ///
    /// The hint must be nonzero and a power of two dividing the denominator. The
    /// caller must also prove no omitted factor remains when a fully reduced result
    /// is required. This routine only inspects the hint's trailing-zero count.
    ///
    /// Zero normalizes first. An odd numerator returns self before inspecting the
    /// hint; otherwise shift both parts by min(v2(numerator),v2(hint)). A zero shift
    /// reuses self; a positive shift allocates fresh unmarked parts. A zero hint can
    /// panic, a false hint can change the value by truncating denominator bits, and
    /// a shift not fitting usize panics. These are unchecked-precondition failures,
    /// not typed public resource/domain outcomes.
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

    /// Return the exact reciprocal, rejecting zero with DivideByZero.
    ///
    /// Marked input canonicalizes first. A retained reciprocal is consulted before
    /// the numerator-zero test and trusted to satisfy the cache relation. Otherwise
    /// a sole strong owner moves its denominator/numerator into a fresh reciprocal
    /// payload, discarding old caches; shared ownership clones the limbs and tries
    /// a strong forward/weak reverse cache pair. Weak references alone do not force
    /// the shared path. Cache publication is best effort and does not affect value.
    ///
    /// No approximate arithmetic or predicate policy is used. Reduction, cloning and
    /// allocation have no total-work, retained-byte or cancellation budget here.
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
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().clone().inverse();
        }
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

    /// Test whether the canonical denominator is one, including canonical zero.
    ///
    /// Marked input may initialize a reduction cache; unmarked input is trusted to
    /// be reduced. This query is exact but is not always a constant-cost metadata test.
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
        self.canonicalized_ref().denominator == *ONE.deref()
    }

    /// Decide whether the canonical denominator is a power of two, including one.
    ///
    /// Marked input canonicalizes first. Existing known/value bits give a traced
    /// cache hit; otherwise scan the denominator and retain the exact boolean fact
    /// with a learned trace event. This query does not encode the exponent by itself.
    /// Integers and zero are dyadic. Unmarked parts must already be reduced; the
    /// query does not discover hidden common factors or guarantee fixed-cost work.
    pub fn is_dyadic(&self) -> bool {
        if self.is_internally_unreduced() {
            return self.canonicalized_ref().is_dyadic();
        }
        if self.retained_fact(RETAINED_DYADIC_KNOWN) {
            crate::trace_dispatch!("rational", "retained-facts", "dyadic-hit");
            return self.retained_fact(RETAINED_DYADIC_VALUE);
        }
        let is_dyadic = Self::is_power_of_two(&self.denominator);
        self.retain_fact(
            RETAINED_DYADIC_KNOWN
                | if is_dyadic {
                    RETAINED_DYADIC_VALUE
                } else {
                    0
                },
        );
        crate::trace_dispatch!("rational", "retained-facts", "dyadic-learned");
        is_dyadic
    }

    /// Compare canonical denominators for exact common-scale arithmetic.
    ///
    /// Canonicalizes self before other when their markers require it, potentially
    /// allocating/reducing and publishing caches. Equal denominators establish a
    /// shared scale, not equal values, signs, numerators or allocation identity.
    /// Unmarked stored parts are trusted to satisfy the reduction invariant.
    #[inline]
    pub fn same_denominator(&self, other: &Self) -> bool {
        self.canonicalized_ref().denominator == other.canonicalized_ref().denominator
    }

    /// Return k for a canonical denominator 2^k, or None for a non-dyadic value.
    ///
    /// Reads one relaxed fact snapshot. Marked input canonicalizes before querying;
    /// otherwise reuse an encoded exponent or known-negative fact, or learn from
    /// the stored denominator. Zero/integers use k=0. Exact recognition can scan
    /// limbs or perform lazy reduction; it has no local work/cancellation cap.
    #[inline]
    pub(crate) fn dyadic_denominator_shift(&self) -> Option<u64> {
        let retained = self
            .retained_facts
            .load(std::sync::atomic::Ordering::Relaxed);
        if retained & RETAINED_UNREDUCED_INTERNAL != 0 {
            return self.canonicalized_ref().dyadic_denominator_shift();
        }
        self.dyadic_denominator_shift_from_retained(retained)
    }

    /// Probe the dyadic exponent without promoting marked unreduced storage.
    ///
    /// A marked value returns None before canonicalization or recognition, even if
    /// its mathematical denominator is dyadic. For unmarked values, None instead
    /// means a non-dyadic denominator and the helper may learn facts. Callers must
    /// not conflate this representation-ineligible result with proved non-dyadicness.
    #[inline]
    pub(crate) fn dyadic_denominator_shift_if_reduced(&self) -> Option<u64> {
        let retained = self
            .retained_facts
            .load(std::sync::atomic::Ordering::Relaxed);
        if retained & RETAINED_UNREDUCED_INTERNAL != 0 {
            return None;
        }
        self.dyadic_denominator_shift_from_retained(retained)
    }

    /// Decode this owner's already checked retained-fact snapshot.
    ///
    /// Caller must exclude internal-unreduced storage and supply a snapshot belonging
    /// to these exact parts. An encoded exponent wins first without revalidation;
    /// known-nondyadic bits then return a traced None; otherwise scan and learn. A
    /// forged or unrelated encoded field can return a false exponent. This helper
    /// is proof reuse, not validation of caller-supplied cache data.
    #[inline]
    fn dyadic_denominator_shift_from_retained(&self, retained: u32) -> Option<u64> {
        let encoded_shift =
            (retained & RETAINED_DYADIC_SHIFT_MASK) >> RETAINED_DYADIC_SHIFT_OFFSET;
        if encoded_shift != 0 {
            return Some(u64::from(encoded_shift - 1));
        }
        if retained & RETAINED_DYADIC_KNOWN != 0 && retained & RETAINED_DYADIC_VALUE == 0 {
            crate::trace_dispatch!("rational", "retained-facts", "non-dyadic-hit");
            return None;
        }
        self.learn_dyadic_denominator_shift()
    }

    /// Recognize the stored denominator and OR its exact classification into facts.
    ///
    /// Caller must establish reduced/unmarked parts. Retains known/value bits and
    /// an encodable exponent; returns the exact exponent even if too large to pack.
    /// A large positive exponent may therefore be rediscovered on later calls.
    /// No independent dispatch event is emitted here; the immutable denominator
    /// justifies concurrent monotone publication of the same classification.
    #[cold]
    #[inline(never)]
    fn learn_dyadic_denominator_shift(&self) -> Option<u64> {
        let shift = Self::biguint_power_of_two_shift(&self.denominator);
        self.retain_fact(
            RETAINED_DYADIC_KNOWN
                | if shift.is_some() {
                    RETAINED_DYADIC_VALUE
                } else {
                    0
                }
                | shift
                    .map(Self::encoded_dyadic_denominator_shift)
                    .unwrap_or(0),
        );
        shift
    }

    /// Pack exponent plus one into the reserved field, or return zero if too large.
    ///
    /// Exponents through 2^23-2 fit; zero in the field means no stored exponent,
    /// not exponent zero or a non-dyadic certificate. This helper adds no known/value
    /// bits and does not check an operand. The bound proves its u32 conversion fits.
    #[inline]
    fn encoded_dyadic_denominator_shift(shift: u64) -> u32 {
        if shift <= RETAINED_DYADIC_SHIFT_MAX {
            (u32::try_from(shift).expect("bounded dyadic shift fits u32") + 1)
                << RETAINED_DYADIC_SHIFT_OFFSET
        } else {
            0
        }
    }

    /// Construct (positive-negative)/denominator and reduce the selected magnitude.
    ///
    /// Compares unsigned totals to select sign without underflow. Equal totals return
    /// canonical zero before consulting the denominator. The denominator must be
    /// positive by caller proof even in that equality case; this helper does not
    /// validate it. Nonzero results use exact parts reduction with its resource limits.
    fn from_signed_magnitude_difference(
        positive: BigUint,
        negative: BigUint,
        denominator: BigUint,
    ) -> Self {
        let (sign, numerator) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => return Self::zero(),
        };
        Self::from_fraction_parts_reduced(sign, numerator, denominator)
    }

}
