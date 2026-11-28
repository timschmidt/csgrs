/// Cheap exact equality evidence for rational consumers, including `Real`.
///
/// These shortcuts preserve stored fractions; they may learn denominator facts
/// but do not canonicalize an internally unreduced operand.
impl Rational {
    /// Decide equality when identity, stored parts or borrowed arithmetic suffice.
    ///
    /// Tries shared allocation, signs, equal stored denominators, reduced dyadic
    /// magnitudes, checked `u128` cross products, then exact binary magnitudes,
    /// in that order. Different magnitude results prove inequality; matching
    /// results (including two unavailable exponents) leave `None`. This is
    /// incomplete evidence, not a declaration that the values are unequal.
    ///
    /// The domain is valid rational storage, including correctly marked internal
    /// unreduced fractions. Dyadic recognition may retain denominator facts;
    /// neither fraction reduction nor arbitrary-width cross multiplication is
    /// performed. Magnitude scans can still visit all stored digits. There is
    /// no work budget or cancellation check, and the trace records selected
    /// arithmetic shortcuts rather than every successful identity/sign test.
    #[inline]
    pub(crate) fn immediate_equality(&self, other: &Self) -> Option<bool> {
        if std::sync::Arc::ptr_eq(&self.0, &other.0) {
            Some(true)
        } else if self.sign != other.sign {
            Some(false)
        } else if self.denominator == other.denominator {
            Some(self.numerator == other.numerator)
        } else if let Some(ordering) = compare_dyadic_magnitudes(self, other) {
            Some(ordering.is_eq())
        } else if let Some(ordering) = compare_word_magnitudes(self, other) {
            crate::trace_dispatch!("rational", "comparison", "word-sized");
            Some(ordering.is_eq())
        } else if self.msd_exact() != other.msd_exact() {
            crate::trace_dispatch!("rational", "comparison", "magnitude-bits");
            Some(false)
        } else {
            None
        }
    }
}

/// Exact equality of rational values, independent of allocation or reduction.
///
/// Stored signs must be consistent with zero and denominators must be nonzero.
/// Unresolved cheap evidence falls back to arbitrary-width cross products; it
/// does not return uncertainty or consult a predicate policy. The default `ne`
/// negates `eq` and uses the same proof path. Backend work/allocation is unbounded
/// by this interface, with no typed exhaustion or cancellation outcome.
impl PartialEq for Rational {
    /// Use immediate evidence, otherwise compare the two exact cross products.
    ///
    /// The fallback is lazy and traced once before multiplication. Earlier sign
    /// checks make unsigned magnitude equality sufficient. Stored unreduced parts
    /// need not be canonicalized to establish the answer.
    fn eq(&self, other: &Self) -> bool {
        self.immediate_equality(other).unwrap_or_else(|| {
            crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
            &self.numerator * &other.denominator == &other.numerator * &self.denominator
        })
    }
}

/// Declare the equivalence relation supplied by exact rational value equality.
///
/// This marker adds no methods; valid rationals have no NaN or unordered value.
impl Eq for Rational {}

/// Hash the canonical signed fraction so equal rational values hash equally.
///
/// Unlike comparison, hashing can reduce an internal unreduced fraction and
/// publish its canonical value in the shared product-cache slot reserved for
/// that purpose. Subsequent calls reuse it; the stored exact value is unchanged.
/// This is a Rust hash input contract, not a stable serialization/checksum format.
/// Reduction and hashing have no local resource budget or cancellation hook.
impl std::hash::Hash for Rational {
    /// Feed canonical sign, numerator and denominator to the caller's hasher.
    ///
    /// Canonicalization is borrowed or initialized on demand; the resulting hash
    /// is independent of the original common factors and allocation identity.
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let value = self.canonicalized_ref();
        value.sign.hash(state);
        value.numerator.hash(state);
        value.denominator.hash(state);
    }
}

/// Exact signed order for every valid rational pair, always returning `Some`.
///
/// Shared identity and signs precede equal-denominator, dyadic, checked-word,
/// binary-magnitude and leading-significand proofs. An unresolved filter falls
/// back to arbitrary-width cross products, never a rounded comparison. Negative
/// values reverse magnitude order. The default `lt`, `le`, `gt` and `ge` delegate
/// to this order; no `Ord` implementation is supplied here.
///
/// Recognition can retain denominator facts but does not reduce the operands.
/// There is no policy-dependent approximation, work limit, cancellation or typed
/// exhaustion at this layer; fallback products may allocate in proportion to
/// operand size.
impl PartialOrd for Rational {
    /// Resolve signed order through the ordered exact filters, then cross products.
    ///
    /// Equal-sign zeros return before magnitude work. Equal stored denominators
    /// compare numerators directly. When binary exponents fit `i32`, differing
    /// exponents decide order and matching exponents permit interval separation;
    /// an unavailable exponent instead reaches the general exact fallback.
    /// Unreachable zero-sign branches rely on the earlier zero return.
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering::*;
        if std::sync::Arc::ptr_eq(&self.0, &other.0) {
            return Some(Equal);
        }
        match self.sign.cmp(&other.sign) {
            Less => return Some(Less),
            Greater => return Some(Greater),
            Equal => {
                if self.sign == NoSign {
                    return Some(Equal);
                }
            }
        }
        if self.denominator == other.denominator {
            match self.sign {
                Plus => self.numerator.partial_cmp(&other.numerator),
                Minus => other.numerator.partial_cmp(&self.numerator),
                NoSign => unreachable!(),
            }
        } else if let Some(ordering) = compare_dyadic_magnitudes(self, other) {
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else if let Some(ordering) = compare_word_magnitudes(self, other) {
            crate::trace_dispatch!("rational", "comparison", "word-sized");
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else if let (Some(left_msd), Some(right_msd)) =
            (self.msd_exact(), other.msd_exact())
        {
            let ordering = if left_msd != right_msd {
                crate::trace_dispatch!("rational", "comparison", "magnitude-bits");
                left_msd.cmp(&right_msd)
            } else if let Some(ordering) =
                compare_normalized_magnitude_intervals(self, other, left_msd)
            {
                crate::trace_dispatch!("rational", "comparison", "leading-bits-interval");
                ordering
            } else {
                crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
                let left = &self.numerator * &other.denominator;
                let right = &other.numerator * &self.denominator;
                left.cmp(&right)
            };
            match self.sign {
                Plus => Some(ordering),
                Minus => Some(ordering.reverse()),
                NoSign => unreachable!(),
            }
        } else {
            crate::trace_dispatch!("rational", "comparison", "biguint-cross-product");
            let left = &self.numerator * &other.denominator;
            let right = &other.numerator * &self.denominator;
            match self.sign {
                Plus => left.partial_cmp(&right),
                Minus => right.partial_cmp(&left),
                NoSign => unreachable!(),
            }
        }
    }
}

/// Certify strict magnitude order from bounded leading-significand products.
///
/// For nonzero valid fractions, `common_msd` supplies the same power-of-two
/// normalization for both operands. Each numerator/denominator bit difference
/// minus that exponent must be zero or one; otherwise return `None`. The caller
/// uses their common exact floor-log2 magnitude, but equality to that floor is
/// not separately validated here. Signs are ignored.
///
/// Strict separation of outward bounds proves `Less` or `Greater`; overlap,
/// touching bounds and exact equality all return `None`. This is not an
/// arbitrary-width cross-product comparison: endpoints are at most `2^53`, so
/// the products and at most one left shift fit within `u128` (at most `2^107`).
/// Only leading borrowed digits are inspected, with no magnitude allocation.
fn compare_normalized_magnitude_intervals(
    left: &Rational,
    right: &Rational,
    common_msd: i32,
) -> Option<std::cmp::Ordering> {
    let normalized = |value: &Rational| {
        let numerator = normalized_biguint_significand_interval(&value.numerator)?;
        let denominator = normalized_biguint_significand_interval(&value.denominator)?;
        let raw_exponent = i128::from(numerator.2) - i128::from(denominator.2);
        let scale_shift = u32::try_from(raw_exponent - i128::from(common_msd)).ok()?;
        (scale_shift <= 1).then_some((numerator, denominator, scale_shift))
    };
    let (left_numerator, left_denominator, left_shift) = normalized(left)?;
    let (right_numerator, right_denominator, right_shift) = normalized(right)?;
    let product = |left: u64, right: u64, shift: u32| {
        (u128::from(left) * u128::from(right)) << shift
    };
    let left_upper = product(left_numerator.1, right_denominator.1, left_shift);
    let right_lower = product(right_numerator.0, left_denominator.0, right_shift);
    if left_upper < right_lower {
        Some(std::cmp::Ordering::Less)
    } else if product(right_numerator.1, left_denominator.1, right_shift)
        < product(left_numerator.0, right_denominator.0, left_shift)
    {
        Some(std::cmp::Ordering::Greater)
    } else {
        None
    }
}

/// Enclose `abs(value) / 2^msd` with outward binary64 bounds.
///
/// The fraction must have nonzero numerator and denominator. Their bit-count
/// difference minus `msd` must be zero or one; zero, a missing significand, an
/// ineligible scale or nonfinite bounds yields `None`. `msd` need not be the
/// exact floor-log2: the finite-float enclosure consumer uses the raw bit-count
/// difference and reapplies its power-of-two scale afterward.
///
/// Leading-significand endpoints convert exactly to binary64. Division rounds,
/// then multiplication by one or two is exact in this normalized range; one
/// adjacent float outward encloses that rounding. Bounds may be wider than the
/// exact value even for a representable fraction. The exact rational remains
/// authoritative. This helper ignores sign and allocates no magnitude storage.
fn normalized_rational_magnitude_interval(
    value: &Rational,
    msd: i32,
) -> Option<(f64, f64)> {
    let (numerator_lower, numerator_upper, numerator_bits) =
        normalized_biguint_significand_interval(&value.numerator)?;
    let (denominator_lower, denominator_upper, denominator_bits) =
        normalized_biguint_significand_interval(&value.denominator)?;
    let raw_exponent = i128::from(numerator_bits) - i128::from(denominator_bits);
    let scale_shift = raw_exponent - i128::from(msd);
    if !(0..=1).contains(&scale_shift) {
        return None;
    }
    /// Exact binary64 denominator for the 53-bit integer significand endpoints.
    const SIGNIFICAND_SCALE: f64 = (1_u64 << 52) as f64;
    let numerator_lower = numerator_lower as f64 / SIGNIFICAND_SCALE;
    let numerator_upper = numerator_upper as f64 / SIGNIFICAND_SCALE;
    let denominator_lower = denominator_lower as f64 / SIGNIFICAND_SCALE;
    let denominator_upper = denominator_upper as f64 / SIGNIFICAND_SCALE;
    let scale = if scale_shift == 0 { 1.0 } else { 2.0 };
    let lower = ((numerator_lower / denominator_upper) * scale).next_down();
    let upper = ((numerator_upper / denominator_lower) * scale).next_up();
    (lower.is_finite() && upper.is_finite()).then_some((lower, upper))
}

/// Bound a positive integer's normalized significand using at most two limbs.
///
/// Returns `(lower, upper, bits)` with `value / 2^(bits - 1)` between
/// `lower / 2^52` and `upper / 2^52`. Zero returns `None`. Up to 53 bits the
/// endpoints coincide exactly; wider values use the leading 53 bits and
/// their successor, even when every discarded bit is zero. The wide upper
/// bound is therefore strict. Endpoints lie in `[2^52, 2^53]`.
///
/// Borrowed most-significant `u64` digits avoid shifts or copies of the full
/// integer. Checked conversions also return `None` if their assumptions fail;
/// they succeed for a positive `BigUint` in the stated branches.
fn normalized_biguint_significand_interval(value: &BigUint) -> Option<(u64, u64, u64)> {
    /// Binary64 significand precision, including the leading implicit bit.
    const SIGNIFICAND_BITS: u64 = 53;

    let bits = value.bits();
    if bits == 0 {
        return None;
    }
    if bits <= SIGNIFICAND_BITS {
        let exact = value.to_u64()? << u32::try_from(SIGNIFICAND_BITS - bits).ok()?;
        return Some((exact, exact, bits));
    }

    let mut digits = value.iter_u64_digits();
    let high = digits.next_back()?;
    let high_bits = 64 - u64::from(high.leading_zeros());
    let leading = if high_bits >= SIGNIFICAND_BITS {
        high >> (high_bits - SIGNIFICAND_BITS)
    } else {
        let remaining = SIGNIFICAND_BITS - high_bits;
        (high << remaining) | (digits.next_back().unwrap_or_default() >> (64 - remaining))
    };
    Some((leading, leading + 1, bits))
}

/// Compare unsigned fraction magnitudes through checked `u128` products.
///
/// Returns `None` if any stored component exceeds `u128` or either cross product
/// overflows it. Otherwise the order is exact, including equality. Signs are
/// ignored and the caller supplies valid nonzero denominators; this helper
/// neither validates fractions nor reduces them or records a trace event.
fn compare_word_magnitudes(left: &Rational, right: &Rational) -> Option<std::cmp::Ordering> {
    crate::verified::word::compare_products(
        left.numerator.to_u128()?,
        left.denominator.to_u128()?,
        right.numerator.to_u128()?,
        right.denominator.to_u128()?,
    )
}

/// Compare magnitudes whose stored denominators are certified powers of two.
///
/// An operand marked internally unreduced is ineligible, even if its denominator
/// is dyadic. Recognition can learn and retain dyadic/non-dyadic facts, visiting
/// left before right; failure returns `None` before the comparison trace.
/// Success traces once and compares borrowed shifted numerators without forming
/// cross products. Signs are ignored. Callers exclude zero before unequal
/// denominator dyadic comparison; the shifted helper requires positive inputs.
/// No total-work or cancellation limit bounds the subsequent digit scan.
fn compare_dyadic_magnitudes(left: &Rational, right: &Rational) -> Option<std::cmp::Ordering> {
    let left_denominator_shift = left.dyadic_denominator_shift_if_reduced()?;
    let right_denominator_shift = right.dyadic_denominator_shift_if_reduced()?;
    crate::trace_dispatch!("rational", "comparison", "dyadic-borrowed-digits");
    Some(compare_shifted_biguints(
        &left.numerator,
        right_denominator_shift,
        &right.numerator,
        left_denominator_shift,
    ))
}

/// Compare two positive integer magnitudes after independent binary shifts.
///
/// Requires nonzero operands and `bits + shift` representable in `u64` on both
/// sides. These preconditions are not validated: a shifted zero can be ordered
/// incorrectly, and overflowing width addition panics with overflow checks or
/// wraps without them. Current rational magnitude callers establish positivity.
///
/// Unequal total bit counts decide immediately. Equal widths allow cancelling
/// the common shift, checking leading words and finally scanning borrowed words
/// most-significant first. Equal widths guarantee equal emitted word counts
/// for positive inputs, justifying the unreachable unequal-exhaustion branch.
/// No shifted `BigUint` is allocated. A full scan can include `shift / 64` low
/// zero words; this allocation saving is not a total-work or cancellation bound.
fn compare_shifted_biguints(
    left: &BigUint,
    left_shift: u64,
    right: &BigUint,
    right_shift: u64,
) -> std::cmp::Ordering {
    let left_bits = left.bits() + left_shift;
    let right_bits = right.bits() + right_shift;
    match left_bits.cmp(&right_bits) {
        std::cmp::Ordering::Equal => {}
        ordering => return ordering,
    }

    let common_shift = left_shift.min(right_shift);
    let left_shift = left_shift - common_shift;
    let right_shift = right_shift - common_shift;
    if left_shift == 0 && right_shift == 0 {
        return left.cmp(right);
    }
    // Equal total widths align the first emitted word. A mismatch decides the
    // exact order; an equal prefix still reaches the complete borrowed scan.
    let leading_digit = |value: &BigUint, shift: u64| {
        let bit_shift = (shift % 64) as u32;
        let mut digits = value.iter_u64_digits();
        let upper = digits.next_back().unwrap_or_default();
        if bit_shift == 0 {
            upper
        } else {
            let high_carry = upper >> (64 - bit_shift);
            if high_carry != 0 {
                high_carry
            } else {
                (upper << bit_shift)
                    | (digits.next_back().unwrap_or_default() >> (64 - bit_shift))
            }
        }
    };
    match leading_digit(left, left_shift).cmp(&leading_digit(right, right_shift)) {
        std::cmp::Ordering::Equal => {}
        ordering => return ordering,
    }

    let mut left_digits = ShiftedU64Digits::new(left, left_shift);
    let mut right_digits = ShiftedU64Digits::new(right, right_shift);
    loop {
        match (left_digits.next(), right_digits.next()) {
            (Some(left), Some(right)) => match left.cmp(&right) {
                std::cmp::Ordering::Equal => {},
                ordering => return ordering,
            },
            (None, None) => return std::cmp::Ordering::Equal,
            _ => unreachable!("equal-width shifted magnitudes have equal digit counts"),
        }
    }
}

/// Allocation-free, most-significant-first `u64` digits for `value << shift`.
///
/// `BigUint` exposes a borrowed double-ended digit iterator. Combining adjacent
/// source digits while walking backward avoids materializing the shifted value
/// for exact dyadic comparisons and binary-magnitude queries. Nonzero inputs
/// produce the usual word representation without a high zero word. Zero is
/// accepted mechanically but emits `shift / 64` zero words, so that stream is
/// not a canonical integer representation. Storage is constant and borrowed;
/// consuming a large shift can still require proportionally many iterations.
struct ShiftedU64Digits<'a> {
    /// Unconsumed source words; `next_back` supplies the next most-significant word.
    digits: num::bigint::U64Digits<'a>,
    /// Residual shift in `0..64`; zero selects the direct borrowed-word path.
    bit_shift: u32,
    /// Pending source word for the residual-shift window, absent at exhaustion
    /// or when no residual shift is needed.
    upper: Option<u64>,
    /// Nonzero high spill from the first source word, emitted once before
    /// the adjacent-word windows; absent when the spill is zero.
    high_carry: Option<u64>,
    /// Remaining full zero words appended for the `shift / 64` displacement.
    low_zero_words: u64,
}

/// Initialize a borrowed word stream without materializing the shifted integer.
impl<'a> ShiftedU64Digits<'a> {
    /// Split the shift into a residual bit window and trailing whole zero words.
    ///
    /// For a nonzero residual, consume the high source word and retain any
    /// nonzero carry. For zero residual, leave the source iterator untouched;
    /// the absent `upper` also prevents evaluating a shift by 64. Construction
    /// does not validate positivity or impose a limit on later iteration.
    fn new(value: &'a BigUint, shift: u64) -> Self {
        let bit_shift = (shift % 64) as u32;
        let mut digits = value.iter_u64_digits();
        let upper = (bit_shift != 0).then(|| digits.next_back()).flatten();
        let high_carry = upper
            .map(|digit| digit >> (64 - bit_shift))
            .filter(|&digit| digit != 0);
        Self {
            digits,
            bit_shift,
            upper,
            high_carry,
            low_zero_words: shift / 64,
        }
    }
}

/// Stream the shifted words from high to low, including positional zero words.
///
/// Each step uses constant storage and consumes at most one borrowed word.
/// After source words, it emits the remaining whole-shift zeros. Exhaustion
/// continues returning `None`; there is no explicit `FusedIterator`,
/// `ExactSizeIterator` or custom `size_hint` implementation. Default iterator
/// adaptors consume this stream without adding integer canonicalization.
impl Iterator for ShiftedU64Digits<'_> {
    /// One base-`2^64` digit, in descending positional order.
    type Item = u64;

    /// Emit the pending carry, adjacent-word combination or trailing zero.
    ///
    /// The zero-residual branch returns source words directly. In the residual
    /// branch, the last window uses a zero lower word. Only a nonzero initial
    /// carry is emitted, and every successful trailing-zero step decrements its
    /// remaining count. No full integer or collected digit vector is created.
    fn next(&mut self) -> Option<Self::Item> {
        if self.bit_shift == 0 {
            return self.digits.next_back().or_else(|| {
                (self.low_zero_words != 0).then(|| {
                    self.low_zero_words -= 1;
                    0
                })
            });
        }
        if let Some(carry) = self.high_carry.take() {
            return Some(carry);
        }
        if let Some(upper) = self.upper {
            let lower = self.digits.next_back();
            self.upper = lower;
            return Some(
                (upper << self.bit_shift)
                    | lower.unwrap_or_default() >> (64 - self.bit_shift),
            );
        }
        (self.low_zero_words != 0).then(|| {
            self.low_zero_words -= 1;
            0
        })
    }
}
