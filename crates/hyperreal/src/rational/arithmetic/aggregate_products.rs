/// Classify a positive word as an exact nonnegative power of a trace factor.
///
/// Requires `value > 0` and `factor >= 2`; repeated exact division leaves
/// one precisely for such powers, including exponent zero. This diagnostic
/// helper is compiled only with dispatch tracing and does not choose the
/// arithmetic result. Zero value with factor at least two, or factor one,
/// can loop forever. With factor zero, zero panics and other values simply
/// test equality to one. Callers use positive denominators and factor three or seven.
#[cfg(feature = "dispatch-trace")]
fn word_is_power_of(mut value: u128, factor: u128) -> bool {
    while value.is_multiple_of(factor) {
        value /= factor;
    }
    value == 1
}

/// Recognize positive words with no prime factors outside three, five and seven.
///
/// One succeeds as the empty product. This trace-only denominator classifier
/// strips those factors exactly; it does not change reduction dispatch.
/// Zero is outside the private domain and does not terminate in the loop.
#[cfg(feature = "dispatch-trace")]
fn word_is_smooth_357(mut value: u128) -> bool {
    for factor in [3_u128, 5, 7] {
        while value.is_multiple_of(factor) {
            value /= factor;
        }
    }
    value == 1
}

/// Nonnegative determinant-one reconstruction matrix for a half-GCD remainder pair.
///
/// For a valid reduction, original = U * reduced with columns acting on the
/// two reduced magnitudes. Column additions and right composition preserve
/// the unit determinant; private fields alone do not validate it. Derived
/// Clone copies coefficient magnitudes and Debug prints them, neither proving
/// the relation. Coefficient storage and multiplication have no byte/work cap.
#[derive(Clone, Debug)]
struct HalfGcdMatrix {
    /// Row-zero, column-zero coefficient of the first reduced magnitude.
    u00: BigUint,
    /// Row-zero, column-one coefficient of the second reduced magnitude.
    u01: BigUint,
    /// Row-one, column-zero coefficient of the first reduced magnitude.
    u10: BigUint,
    /// Row-one, column-one coefficient of the second reduced magnitude.
    u11: BigUint,
}

/// Exact matrix construction, composition and inverse-action helpers for half-GCD.
impl HalfGcdMatrix {
    /// Construct the identity reconstruction matrix for an unchanged magnitude pair.
    ///
    /// Its determinant is one and both off-diagonal entries are zero. Subsequent
    /// column additions and composition preserve the reconstruction relation.
    fn identity() -> Self {
        Self {
            u00: BigUint::one(),
            u01: BigUint::ZERO,
            u10: BigUint::ZERO,
            u11: BigUint::one(),
        }
    }

    /// Apply the determinant-one inverse to a candidate original magnitude pair.
    ///
    /// Computes `(u11*left - u01*right, u00*right - u10*left)` with unsigned
    /// products, returning `None` if either difference would be negative. The
    /// caller must establish determinant one; nonnegative output then supplies
    /// the reconstructed pair. This routine neither checks the determinant nor
    /// divides by it. An arbitrary
    /// matrix therefore computes its adjugate action, not its general inverse.
    /// Products allocate and are traced without a total work/memory budget.
    fn apply_inverse(&self, left: &BigUint, right: &BigUint) -> Option<(BigUint, BigUint)> {
        let first_positive =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u11, left);
        let first_negative =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u01, right);
        let second_positive =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u00, right);
        let second_negative =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u10, left);
        if first_positive < first_negative || second_positive < second_negative {
            return None;
        }
        Some((
            first_positive - first_negative,
            second_positive - second_negative,
        ))
    }

    /// Replace this matrix by its ordinary product with a right-hand matrix.
    ///
    /// Compute all four new entries before replacing the old ones. If each
    /// matrix reconstructs one successive reduction, their product reconstructs
    /// the composed reduction; determinant-one inputs retain that determinant.
    /// No invariant validation or coefficient-growth budget is performed.
    fn multiply_right(&mut self, right: &Self) {
        let u00 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u00, &right.u00)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u01,
                &right.u10,
            );
        let u01 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u00, &right.u01)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u01,
                &right.u11,
            );
        let u10 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u10, &right.u00)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u11,
                &right.u10,
            );
        let u11 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u10, &right.u01)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u11,
                &right.u11,
            );
        (self.u00, self.u01, self.u10, self.u11) = (u00, u01, u10, u11);
    }

    /// Add quotient times the right column to the left column.
    ///
    /// This composes the reconstruction matrix with the inverse of a step
    /// subtracting quotient times the left remainder from the right remainder.
    /// Nonnegative quotient zero is an identity update. The operation preserves
    /// determinant one when supplied a valid matrix, with unbounded coefficient
    /// allocation delegated to exact magnitude multiplication.
    fn update_left_column(&mut self, quotient: &BigUint) {
        self.u00 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u01,
            quotient,
        );
        self.u10 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u11,
            quotient,
        );
    }

    /// Add quotient times the left column to the right column.
    ///
    /// This transports reconstruction through subtraction of quotient times the
    /// right remainder from the left remainder. Nonnegative quotients preserve
    /// a valid unit determinant; this helper does not inspect the remainder pair
    /// or enforce a coefficient-growth budget.
    fn update_right_column(&mut self, quotient: &BigUint) {
        self.u01 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u00,
            quotient,
        );
        self.u11 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u10,
            quotient,
        );
    }
}

/// Selected reduced magnitudes together with their exact reconstruction relation.
///
/// The original pair is matrix times `(left, right)` and the determinant-one
/// matrix preserves their GCD. The half-GCD stop condition concerns their
/// absolute difference; these are not necessarily each half the original
/// width, nor necessarily sorted. This private carrier does not validate
/// caller-constructed fields or carry a resource/cancellation receipt.
struct HalfGcdReduction {
    /// First transformed nonnegative magnitude in reconstruction order.
    left: BigUint,
    /// Second transformed nonnegative magnitude in reconstruction order.
    right: BigUint,
    /// Unit-determinant nonnegative matrix mapping this pair back to the original.
    matrix: HalfGcdMatrix,
}

/// Alignment and width estimate for a fixed number of signed dyadic products.
///
/// The producer counts only live terms, sums their denominator exponents and
/// estimates aligned numerator growth. This plan selects an arithmetic schedule;
/// it neither stores the terms nor proves that a chosen word path will succeed.
struct DyadicProductSumPlan<const TERMS: usize> {
    /// Sum of factor denominator exponents per live term; inactive slots stay zero.
    denominator_shifts: [u64; TERMS],
    /// Maximum live denominator exponent, or zero when no term is live.
    max_shift: u64,
    /// Whether the conservative aligned numerator/carry estimate exceeds 128 bits.
    prefer_wide: bool,
}

/// Six u64 limbs for the checked 384-bit dyadic scratch accumulator.
///
/// This capacity controls fallback eligibility, not the complete scalar domain
/// or a total memory/work budget.
const DYADIC_STACK_LIMBS: usize = 6;

/// Unsigned little-endian 384-bit scratch magnitude for exact dyadic accumulation.
///
/// Default is zero; Copy/Clone duplicate the six limbs and Debug exposes them.
/// Checked addition can fail after mutating the limbs on a final carry, so callers
/// must discard failed scratch state before a general exact fallback. The fixed
/// buffer bounds this scratch storage, not the surrounding operation's resources.
#[derive(Clone, Copy, Debug, Default)]
struct DyadicStackAccumulator(
    /// Unsigned scratch digits, least significant limb first.
    [u64; DYADIC_STACK_LIMBS]
);

/// Signed dyadic result with a six-limb numerator and a power-of-two denominator.
///
/// Its value is sign * magnitude / 2^denominator_shift. Normal producers cancel
/// common powers of two and encode zero with NoSign, zero limbs and zero shift.
/// Private field construction does not check these invariants. Derived copies
/// and Debug retain or display the stored representation without rational reduction.
#[derive(Clone, Copy, Debug)]
struct DyadicStackSum {
    /// Sign of the numerator; NoSign denotes zero.
    sign: Sign,
    /// Unsigned numerator limbs after the producer's power-of-two cancellation.
    magnitude: DyadicStackAccumulator,
    /// Nonnegative exponent of the power-of-two denominator.
    denominator_shift: u64,
}

/// Native-word signed dyadic used before allocating a Rational.
///
/// The represented value is sign * magnitude / 2^denominator_shift. Canonical
/// producers use NoSign exactly for zero and an odd magnitude when the shift is
/// positive. Private construction and native adapters do not enforce normalization.
/// Copy/Clone and Debug operate on the stored fields, not on a reduced quotient.
#[derive(Clone, Copy, Debug)]
struct DyadicWord {
    /// Numerator sign; NoSign marks an inactive zero for the native kernel.
    sign: Sign,
    /// Unsigned numerator fitting u128; canonical zero stores zero here.
    magnitude: u128,
    /// Exponent of the exact power-of-two denominator, not a floating exponent.
    denominator_shift: u64,
}

/// Translate retained signs, magnitudes and exponents to and from the shared dyadic kernel.
///
/// These adapters copy producer invariants without normalization or validation;
/// they do not perform rational arithmetic or materialize heap-backed fractions.
impl DyadicWord {
    /// Translate the sign into native activity/negativity flags without normalization.
    ///
    /// NoSign becomes inactive; magnitude and denominator shift are copied verbatim.
    /// The caller retains responsibility for canonical zero and power-of-two reduction.
    fn into_native(self) -> crate::verified::dyadic::Word {
        crate::verified::dyadic::Word {
            active: self.sign != NoSign,
            negative: self.sign == Minus,
            magnitude: self.magnitude,
            shift: self.denominator_shift,
        }
    }

    /// Translate native activity and sign flags while retaining magnitude and shift.
    ///
    /// Inactive means NoSign regardless of the negative flag. This adapter trusts
    /// the producer: it neither clears inactive payloads nor normalizes active zero.
    fn from_native(word: crate::verified::dyadic::Word) -> Self {
        let sign = if !word.active { NoSign } else if word.negative { Minus } else { Plus };
        Self { sign, magnitude: word.magnitude, denominator_shift: word.shift }
    }
}

/// Determinant construction for two unbounded dyadic supporting lines.
///
/// For directions d1,d2 and start displacement v, denominator is cross(d1,d2),
/// and numerators are cross(v,d2), cross(v,d1). Their quotients give the two
/// parameters. Producers require a nonzero determinant and checked word bounds;
/// this carrier does not validate segment membership. Derived copies retain the
/// construction values without materializing parameter or coordinate rationals.
#[derive(Clone, Copy)]
struct DyadicLineIntersectionPlan {
    /// First line origin, in x/y coordinate order.
    first_start: [DyadicWord; 2],
    /// First line direction, end minus start, in x/y order.
    first_delta: [DyadicWord; 2],
    /// Nonzero signed determinant shared by both parameter quotients.
    denominator: DyadicWord,
    /// Signed determinants for the first and second line parameters, respectively.
    numerators: [DyadicWord; 2],
}
/// Retained exact dyadic origin and direction for repeated supporting-line queries.
///
/// Endpoints and their difference must fit native dyadic words. The source stores
/// start and end-minus-start; it does not retain endpoint handles, segment bounds
/// or a topology certificate. Coincident endpoints can be retained, but subsequent
/// intersection planning declines their zero determinant. Query failure can also
/// mean a representation bound, so None alone does not prove disjointness.
/// Copy/Clone duplicate compact fields and Debug displays their representation.
#[derive(Clone, Copy, Debug)]
pub struct ExactDyadicLine2 {
    /// Exact origin coordinates in x/y order.
    start: [DyadicWord; 2],
    /// Exact end-minus-start coordinates; a zero direction is representable.
    delta: [DyadicWord; 2],
}

/// Signed dyadic with a four-limb little-endian numerator.
///
/// The value is sign * magnitude / 2^denominator_shift. Producers normalize
/// powers of two and represent zero with NoSign. The fields do not validate this
/// invariant; Copy/Clone and Debug only copy or display them. This is a 256-bit
/// numerator carrier, not a bound on the denominator exponent or later allocation.
#[derive(Clone, Copy, Debug)]
struct DyadicWideWord {
    /// Numerator sign, with NoSign denoting zero.
    sign: Sign,
    /// Unsigned 256-bit numerator, least significant limb first.
    magnitude: [u64; 4],
    /// Exponent of the exact power-of-two denominator.
    denominator_shift: u64,
}

/// Line-intersection construction retaining four-limb determinant values.
///
/// It has the same determinant and parameter meaning as the compact plan, with
/// nonzero denominator and checked 256-bit numerators. Origins and directions
/// remain native words. Copy/Clone preserve the retained construction; no segment
/// clipping, point materialization or topology validation occurs here.
#[derive(Clone, Copy)]
struct DyadicWideLineIntersectionPlan {
    /// First supporting line origin in native x/y words.
    first_start: [DyadicWord; 2],
    /// First supporting line direction in native x/y words.
    first_delta: [DyadicWord; 2],
    /// Nonzero four-limb determinant shared by both parameters.
    denominator: DyadicWideWord,
    /// Four-limb determinants for the first and second line parameters.
    numerators: [DyadicWideWord; 2],
}
/// Two exact line parameters retained as signed dyadic determinant quotients.
///
/// Each parameter is `numerator[i] / denominator`, including their independent
/// powers of two. A nonzero denominator and consistent signs/magnitudes are
/// required. Retention avoids Rational allocation and GCD reduction; it carries
/// no line identity, segment-membership evidence or topology certificate.
/// Comparisons order the stored scalars even across unrelated source lines.
/// Clone/Debug copy or display fields without proving the defining relations.
/// Materialization has stricter shift/resource requirements than comparison.
#[derive(Clone, Debug)]
pub struct ExactDyadicLineParameters2 {
    /// Unsigned native numerator magnitudes for first and second parameters.
    numerator_magnitudes: [u128; 2],
    /// Nonzero unsigned native magnitude of the shared determinant.
    denominator_magnitude: u128,
    /// Power-of-two exponents for numerator zero, numerator one, then denominator.
    denominator_shifts: [u64; 3],
    /// Signs in the same two-numerators-then-denominator order; the last is nonzero.
    signs: [Sign; 3],
}
/// Deferred exact x/y coordinates over one native dyadic determinant.
///
/// Coordinate numerators retain origin*determinant + direction*parameter_numerator
/// in fixed-stack form. Dividing by the shared nonzero determinant yields the
/// point. Copies require no arbitrary-precision rational materialization; observing
/// coordinates can allocate and leaves nonzero quotients internally unreduced.
/// Copy/Clone and Debug preserve or display fields, not a validated incidence proof.
#[derive(Clone, Copy, Debug)]
pub struct ExactDyadicLinePoint2 {
    /// Affine coordinate numerators in x/y order, each a signed stack dyadic.
    coordinate_numerators: [DyadicStackSum; 2],
    /// Nonzero unsigned native determinant magnitude.
    denominator_magnitude: u128,
    /// Power-of-two denominator exponent of the determinant itself.
    denominator_shift: u64,
    /// Nonzero sign of the determinant; combined with each numerator sign.
    denominator_sign: Sign,
}
/// Two exact line parameters over a shared four-limb dyadic determinant.
///
/// Each signed numerator divided by the signed denominator defines one scalar.
/// The denominator must be nonzero and the stored signs must agree with zero
/// magnitudes. Comparison uses bounded integer products without reducing to
/// rationals; materialization can allocate according to exponent differences.
/// This carrier does not own source lines or establish segment incidence.
/// Clone/Debug copy or display the fields without validating their relations.
#[derive(Clone, Debug)]
pub struct ExactDyadicWideLineParameters2 {
    /// Two unsigned 256-bit numerators, each stored least significant limb first.
    numerator_magnitudes: [[u64; 4]; 2],
    /// Nonzero unsigned 256-bit determinant, least significant limb first.
    denominator_magnitude: [u64; 4],
    /// Dyadic denominator exponents for the two numerators and shared denominator.
    denominator_shifts: [u64; 3],
    /// Signs of the two numerators followed by the nonzero denominator sign.
    signs: [Sign; 3],
}
/// Deferred exact x/y coordinates over a four-limb dyadic determinant.
///
/// Each signed stack numerator is divided by the shared nonzero determinant.
/// Retention uses fixed fields; materialization creates potentially large,
/// internally unreduced rational quotients. Copy/Clone and Debug preserve or
/// display these fields without a new incidence proof or resource receipt.
#[derive(Clone, Copy, Debug)]
pub struct ExactDyadicWideLinePoint2 {
    /// Signed affine numerators in x/y order, each held in six limbs.
    coordinate_numerators: [DyadicStackSum; 2],
    /// Nonzero four-limb determinant magnitude in little-endian order.
    denominator_magnitude: [u64; 4],
    /// Power-of-two denominator exponent of the determinant.
    denominator_shift: u64,
    /// Nonzero determinant sign shared by both coordinate quotients.
    denominator_sign: Sign,
}

/// Materialize deferred affine coordinates over their shared native determinant.
///
/// Zero coordinates normalize immediately; nonzero quotients retain lazy exact
/// fraction parts. The carrier remains unchanged and later observation owns
/// canonical reduction and any additional allocation.
impl ExactDyadicLinePoint2 {
    /// Construct the two exact coordinate quotients without mutating this carrier.
    ///
    /// NoSign numerators become canonical zero. Nonzero coordinates use the
    /// unreduced quotient path, deferring general cancellation until observation.
    /// Requires valid numerator signs and a nonzero determinant; large shift
    /// differences can allocate heavily or panic if they do not fit usize. There
    /// is no total-work limit or typed exhaustion result.
    pub(crate) fn materialize_rationals(&self) -> [Rational; 2] {
        self.coordinate_numerators.map(|numerator| {
            if numerator.sign == NoSign {
                Rational::zero()
            } else {
                Rational::quotient_dyadic_stack_sum_unreduced(
                    numerator,
                    self.denominator_sign,
                    self.denominator_shift,
                    self.denominator_magnitude,
                )
            }
        })
    }
}

/// Recover the wide determinant and materialize its two deferred coordinate quotients.
///
/// The shared four-limb divisor retains its sign and binary exponent. Nonzero
/// coordinates become lazy exact fractions; these accessors trust construction
/// relations and do not revalidate source incidence or segment membership.
impl ExactDyadicWideLinePoint2 {
    /// Reassemble the retained wide determinant without validation or allocation.
    fn denominator(&self) -> DyadicWideWord {
        DyadicWideWord {
            sign: self.denominator_sign,
            magnitude: self.denominator_magnitude,
            denominator_shift: self.denominator_shift,
        }
    }

    /// Construct the two exact coordinate quotients without mutating this carrier.
    ///
    /// NoSign numerators become canonical zero. Nonzero coordinates use the
    /// unreduced quotient path, deferring general cancellation until observation.
    /// Requires valid numerator signs and a nonzero determinant; large shift
    /// differences can allocate heavily or panic if they do not fit usize. There
    /// is no total-work limit or typed exhaustion result.
    pub(crate) fn materialize_rationals(&self) -> [Rational; 2] {
        let denominator = self.denominator();
        self.coordinate_numerators.map(|numerator| {
            if numerator.sign == NoSign {
                Rational::zero()
            } else {
                Rational::quotient_dyadic_stack_sum_by_wide_unreduced(numerator, denominator)
            }
        })
    }
}

/// Compare retained determinant quotients and materialize them within the native tier.
///
/// Comparison aligns exact integer products without first reducing parameter
/// fractions. Normalized and general comparison schedules have distinct admission
/// contracts; materialization additionally requires reduced quotient parts to fit
/// u128 and can panic for an otherwise valid retained parameter.
impl ExactDyadicLineParameters2 {
    /// Pack two dyadic numerators and their shared denominator without reduction.
    ///
    /// Requires a nonzero denominator and consistent signs/magnitudes. No check
    /// or allocation is performed; arbitrary private inputs need not describe a
    /// valid quotient or any line intersection.
    fn from_words(numerators: [DyadicWord; 2], denominator: DyadicWord) -> Self {
        Self {
            numerator_magnitudes: [numerators[0].magnitude, numerators[1].magnitude],
            denominator_magnitude: denominator.magnitude,
            denominator_shifts: [
                numerators[0].denominator_shift,
                numerators[1].denominator_shift,
                denominator.denominator_shift,
            ],
            signs: [numerators[0].sign, numerators[1].sign, denominator.sign],
        }
    }

    /// Reassemble parameter numerator zero or one without validation or reduction.
    ///
    /// An index outside 0..2 panics on array access.
    fn numerator(&self, index: usize) -> DyadicWord {
        DyadicWord {
            sign: self.signs[index],
            magnitude: self.numerator_magnitudes[index],
            denominator_shift: self.denominator_shifts[index],
        }
    }

    /// Reassemble the shared signed dyadic denominator without validating nonzero.
    fn denominator(&self) -> DyadicWord {
        DyadicWord {
            sign: self.signs[2],
            magnitude: self.denominator_magnitude,
            denominator_shift: self.denominator_shifts[2],
        }
    }

    /// Multiply two u128 magnitudes exactly into four little-endian limbs.
    ///
    /// Delegates to the shared verified product kernel; zero is supported and no
    /// heap allocation or rational reduction is needed.
    #[inline]
    fn multiply_magnitudes(left: u128, right: u128) -> [u64; 4] {
        crate::verified::product::multiply_u128(left, right)
    }

    /// Return the exact bit length of a nonzero four-limb unsigned product.
    ///
    /// Zero panics at the nonzero-limb assertion. Valid output lies in 1..=256.
    #[inline]
    fn product_bit_length(product: &[u64; 4]) -> u128 {
        let index = product
            .iter()
            .rposition(|limb| *limb != 0)
            .expect("nonzero parameter product has a nonzero limb");
        u128::try_from(index * 64 + (64 - product[index].leading_zeros() as usize))
            .expect("fixed product bit length fits u128")
    }

    /// Compare nonzero products after shifting each highest bit to bit 255.
    ///
    /// Supplied lengths must be the actual lengths in 1..=256. Limbs are read
    /// virtually after normalization, without allocation. This orders normalized
    /// significands; the caller must already establish equal effective exponents
    /// for it to decide the full scaled-value order. Incorrect lengths are not
    /// validated and can panic or produce a wrong order.
    #[inline]
    fn compare_normalized_products(
        left: [u64; 4],
        left_bit_length: u128,
        right: [u64; 4],
        right_bit_length: u128,
    ) -> Ordering {
        let left_shift = u32::try_from(256 - left_bit_length)
            .expect("four-limb product normalization shift fits u32");
        let right_shift = u32::try_from(256 - right_bit_length)
            .expect("four-limb product normalization shift fits u32");
        let normalized_limb = |product: &[u64; 4], shift: u32, target: usize| {
            let word_shift = (shift / 64) as usize;
            if target < word_shift {
                return 0;
            }
            let source = target - word_shift;
            let bit_shift = shift % 64;
            let mut limb = product[source] << bit_shift;
            if bit_shift != 0 && source != 0 {
                limb |= product[source - 1] >> (64 - bit_shift);
            }
            limb
        };
        for target in (0..4).rev() {
            let ordering = normalized_limb(&left, left_shift, target)
                .cmp(&normalized_limb(&right, right_shift, target));
            if ordering != Ordering::Equal {
                return ordering;
            }
        }
        Ordering::Equal
    }

    /// Order parameter zero or one using fixed products and normalized significands.
    ///
    /// Resolve signs/zero first, then compare product bit lengths plus dyadic
    /// exponents in u128. Equal effective widths use virtual normalization to
    /// 256 bits. Valid carriers need no heap allocation, even for u64 exponents.
    /// An invalid index or a signed nonzero value with zero magnitude can panic;
    /// no source identity or defining equation is checked.
    #[cold]
    #[inline(never)]
    fn compare_parameter_normalized(&self, index: usize, other: &Self) -> Ordering {
        let left_numerator = self.numerator(index);
        let right_numerator = other.numerator(index);
        let left_denominator = self.denominator();
        let right_denominator = other.denominator();
        let left_sign = left_numerator.sign * left_denominator.sign;
        let right_sign = right_numerator.sign * right_denominator.sign;
        if left_sign != right_sign {
            return match (left_sign, right_sign) {
                (Minus, _) | (_, Plus) => Ordering::Less,
                (Plus, _) | (_, Minus) => Ordering::Greater,
                _ => Ordering::Equal,
            };
        }
        if left_sign == NoSign {
            return Ordering::Equal;
        }

        let left_shift = u128::from(left_denominator.denominator_shift)
            + u128::from(right_numerator.denominator_shift);
        let right_shift = u128::from(right_denominator.denominator_shift)
            + u128::from(left_numerator.denominator_shift);
        let left =
            Self::multiply_magnitudes(left_numerator.magnitude, right_denominator.magnitude);
        let right =
            Self::multiply_magnitudes(right_numerator.magnitude, left_denominator.magnitude);
        let left_bit_length = Self::product_bit_length(&left);
        let right_bit_length = Self::product_bit_length(&right);
        let ordering =
            match (left_bit_length + left_shift).cmp(&(right_bit_length + right_shift)) {
                Ordering::Equal => Self::compare_normalized_products(
                    left,
                    left_bit_length,
                    right,
                    right_bit_length,
                ),
                ordering => ordering,
            };
        if left_sign == Minus {
            ordering.reverse()
        } else {
            ordering
        }
    }

    /// Order parameter zero or one by exact signed, scaled cross-products.
    ///
    /// After signs/zero, an exponent separation of at least 256 decides order.
    /// Otherwise try six-limb aligned products and fall back to BigUint if they
    /// do not fit; that fallback can allocate. Negative quotients reverse magnitude
    /// order. Requires consistent magnitudes and nonzero denominators; an invalid
    /// index panics. No rational reduction or source-identity check occurs.
    fn compare_parameter(&self, index: usize, other: &Self) -> Ordering {
        let left_numerator = self.numerator(index);
        let right_numerator = other.numerator(index);
        let left_denominator = self.denominator();
        let right_denominator = other.denominator();
        let left_sign = left_numerator.sign * left_denominator.sign;
        let right_sign = right_numerator.sign * right_denominator.sign;
        if left_sign != right_sign {
            return match (left_sign, right_sign) {
                (Minus, _) | (_, Plus) => Ordering::Less,
                (Plus, _) | (_, Minus) => Ordering::Greater,
                _ => Ordering::Equal,
            };
        }
        if left_sign == NoSign {
            return Ordering::Equal;
        }

        let left_shift = u128::from(left_denominator.denominator_shift)
            + u128::from(right_numerator.denominator_shift);
        let right_shift = u128::from(right_denominator.denominator_shift)
            + u128::from(left_numerator.denominator_shift);
        let exponent_ordering = if left_shift >= right_shift + 256 {
            Some(Ordering::Greater)
        } else if right_shift >= left_shift + 256 {
            Some(Ordering::Less)
        } else {
            None
        };
        let common_shift = left_shift.min(right_shift);
        let mut left = DyadicStackAccumulator::default();
        let mut right = DyadicStackAccumulator::default();
        let ordering = exponent_ordering.unwrap_or_else(|| match (
            left.add_product(
                left_numerator.magnitude,
                right_denominator.magnitude,
                u64::try_from(left_shift - common_shift)
                    .expect("bounded dyadic comparison shift fits u64"),
            ),
            right.add_product(
                right_numerator.magnitude,
                left_denominator.magnitude,
                u64::try_from(right_shift - common_shift)
                    .expect("bounded dyadic comparison shift fits u64"),
            ),
        ) {
            (Some(()), Some(())) => left.0.iter().rev().cmp(right.0.iter().rev()),
            _ => {
                let mut left =
                    BigUint::from(left_numerator.magnitude) * right_denominator.magnitude;
                let mut right =
                    BigUint::from(right_numerator.magnitude) * left_denominator.magnitude;
                left <<= usize::try_from(left_shift - common_shift)
                    .expect("bounded dyadic comparison shift fits usize");
                right <<= usize::try_from(right_shift - common_shift)
                    .expect("bounded dyadic comparison shift fits usize");
                left.cmp(&right)
            }
        });
        if left_sign == Minus {
            ordering.reverse()
        } else {
            ordering
        }
    }
    /// Order the first retained scalar parameter against the other carrier.
    ///
    /// Uses exact signed cross-products without rational reduction. The carriers
    /// need valid nonzero denominators but need not refer to the same source line;
    /// this comparison does not establish common provenance or segment membership.
    pub fn compare_first_parameter(&self, other: &Self) -> Ordering {
        self.compare_parameter(0, other)
    }
    /// Order the second retained scalar parameter against the other carrier.
    ///
    /// Uses exact signed cross-products without rational reduction. The carriers
    /// need valid nonzero denominators but need not refer to the same source line;
    /// this comparison does not establish common provenance or segment membership.
    pub fn compare_second_parameter(&self, other: &Self) -> Ordering {
        self.compare_parameter(1, other)
    }
        /// Order the first parameter with the fixed-width normalized-product path.
        ///
        /// This exact comparison avoids the ordinary path's possible BigUint fallback.
        /// Valid carriers supply consistent signs and a nonzero denominator; comparing
        /// scalars does not prove that the carriers share a source line.
        pub fn compare_first_parameter_normalized(&self, other: &Self) -> Ordering {
        self.compare_parameter_normalized(0, other)
    }
        /// Order the second parameter with the fixed-width normalized-product path.
        ///
        /// This exact comparison avoids the ordinary path's possible BigUint fallback.
        /// Valid carriers supply consistent signs and a nonzero denominator; comparing
        /// scalars does not prove that the carriers share a source line.
        pub fn compare_second_parameter_normalized(&self, other: &Self) -> Ordering {
        self.compare_parameter_normalized(1, other)
    }

    /// Reduce parameter zero or one into an exact Rational.
    ///
    /// The native quotient path cancels the magnitude GCD and applies the exponent
    /// difference. That difference must fit i32 and the fully reduced numerator
    /// and denominator must each fit u128. None becomes an expect panic, even for
    /// a valid retained parameter outside that tier. An invalid index also panics.
    /// Comparison admits broader exponent metadata than this materializer, and
    /// allocation is not covered by a total-work or memory budget.
    pub(crate) fn materialize_parameter(&self, index: usize) -> Rational {
        Rational::quotient_dyadic_words(self.numerator(index), self.denominator())
            .expect("retained dyadic line parameter remains representable")
            .0
    }
}

/// Order retained four-limb determinant quotients and materialize exact rational parameters.
///
/// Comparison uses fixed integer products and certified exponent alignment.
/// Materialization instead builds and reduces potentially large rational parts;
/// fixed comparison scratch does not bound the cost of that conversion.
impl ExactDyadicWideLineParameters2 {
    /// Pack two dyadic numerators and their shared denominator without reduction.
    ///
    /// Requires a nonzero denominator and consistent signs/magnitudes. No check
    /// or allocation is performed; arbitrary private inputs need not describe a
    /// valid quotient or any line intersection.
    fn from_words(numerators: [DyadicWideWord; 2], denominator: DyadicWideWord) -> Self {
        Self {
            numerator_magnitudes: [numerators[0].magnitude, numerators[1].magnitude],
            denominator_magnitude: denominator.magnitude,
            denominator_shifts: [
                numerators[0].denominator_shift,
                numerators[1].denominator_shift,
                denominator.denominator_shift,
            ],
            signs: [numerators[0].sign, numerators[1].sign, denominator.sign],
        }
    }

    /// Reassemble parameter numerator zero or one without validation or reduction.
    ///
    /// An index outside 0..2 panics on array access.
    fn numerator(&self, index: usize) -> DyadicWideWord {
        DyadicWideWord {
            sign: self.signs[index],
            magnitude: self.numerator_magnitudes[index],
            denominator_shift: self.denominator_shifts[index],
        }
    }

    /// Reassemble the shared signed dyadic denominator without validating nonzero.
    fn denominator(&self) -> DyadicWideWord {
        DyadicWideWord {
            sign: self.signs[2],
            magnitude: self.denominator_magnitude,
            denominator_shift: self.denominator_shifts[2],
        }
    }

    /// Zero-extend a native dyadic numerator to four limbs without changing its value.
    ///
    /// Sign and denominator exponent are preserved without validation or reduction.
    fn wide_word(value: DyadicWord) -> DyadicWideWord {
        DyadicWideWord {
            sign: value.sign,
            magnitude: [
                value.magnitude as u64,
                (value.magnitude >> 64) as u64,
                0,
                0,
            ],
            denominator_shift: value.denominator_shift,
        }
    }

    /// Multiply two four-limb unsigned magnitudes exactly into eight little-endian limbs.
    ///
    /// Delegates to the shared verified kernel; zero is supported without allocation.
    fn multiply_magnitudes(left: [u64; 4], right: [u64; 4]) -> [u64; 8] {
        crate::verified::product::multiply(&left, &right)
    }

    /// Place an eight-limb product left-shifted into sixteen little-endian limbs.
    ///
    /// The comparison caller bounds shift by 511 after equal-width classification,
    /// so the result fits 1,024 bits. Zero source limbs are skipped. This helper
    /// does not check arbitrary shifts for capacity; out-of-range conversions or
    /// indices can panic. No heap allocation is performed.
    fn shifted_product(product: [u64; 8], shift: u128) -> [u64; 16] {
        let word_shift = usize::try_from(shift / 64)
            .expect("bounded wide comparison shift fits usize");
        let bit_shift = u32::try_from(shift % 64).expect("limb bit shift fits u32");
        let mut shifted = [0_u64; 16];
        for (index, limb) in product.into_iter().enumerate() {
            if limb == 0 {
                continue;
            }
            shifted[word_shift + index] |= limb << bit_shift;
            if bit_shift != 0 {
                shifted[word_shift + index + 1] |= limb >> (64 - bit_shift);
            }
        }
        shifted
    }

    /// Order two signed dyadic quotients using fixed-width exact products.
    ///
    /// Requires nonzero denominators and consistent signs/magnitudes. Signs and zero
    /// resolve first; nonzero cross-products occupy eight limbs. Compare bit lengths
    /// plus exponents in u128, then align equal-width cases in sixteen limbs. That
    /// equality bounds relative shift by 511. Negative order reverses. No rational
    /// reduction or heap allocation occurs; invalid zero products can panic.
    fn compare_parameter_words(
        left_numerator: DyadicWideWord,
        left_denominator: DyadicWideWord,
        right_numerator: DyadicWideWord,
        right_denominator: DyadicWideWord,
    ) -> Ordering {
        let left_sign = left_numerator.sign * left_denominator.sign;
        let right_sign = right_numerator.sign * right_denominator.sign;
        if left_sign != right_sign {
            return match (left_sign, right_sign) {
                (Minus, _) | (_, Plus) => Ordering::Less,
                (Plus, _) | (_, Minus) => Ordering::Greater,
                _ => Ordering::Equal,
            };
        }
        if left_sign == NoSign {
            return Ordering::Equal;
        }

        let left = Self::multiply_magnitudes(
            left_numerator.magnitude,
            right_denominator.magnitude,
        );
        let right = Self::multiply_magnitudes(
            right_numerator.magnitude,
            left_denominator.magnitude,
        );
        let bit_len = |value: &[u64; 8]| {
            let index = value
                .iter()
                .rposition(|limb| *limb != 0)
                .expect("nonzero parameter product has a nonzero limb");
            u128::try_from(index * 64 + (64 - value[index].leading_zeros() as usize))
                .expect("fixed product bit length fits u128")
        };
        let left_shift = u128::from(left_denominator.denominator_shift)
            + u128::from(right_numerator.denominator_shift);
        let right_shift = u128::from(right_denominator.denominator_shift)
            + u128::from(left_numerator.denominator_shift);
        let ordering = match (bit_len(&left) + left_shift).cmp(&(bit_len(&right) + right_shift)) {
            Ordering::Equal => {
                let common_shift = left_shift.min(right_shift);
                let left = Self::shifted_product(left, left_shift - common_shift);
                let right = Self::shifted_product(right, right_shift - common_shift);
                left.iter().rev().cmp(right.iter().rev())
            }
            ordering => ordering,
        };
        if left_sign == Minus {
            ordering.reverse()
        } else {
            ordering
        }
    }

    /// Order parameter zero or one by the fixed-width signed quotient comparator.
    ///
    /// An index outside 0..2 panics; valid carrier comparisons do not allocate or
    /// verify that source lines are shared.
    fn compare_parameter(&self, index: usize, other: &Self) -> Ordering {
        Self::compare_parameter_words(
            self.numerator(index),
            self.denominator(),
            other.numerator(index),
            other.denominator(),
        )
    }

    /// Order parameter zero or one against a native carrier after exact widening.
    ///
    /// Only the compact operands are zero-extended; no Rational is constructed.
    /// Both carriers must be valid, and an index outside 0..2 panics.
    fn compare_parameter_to_compact(
        &self,
        index: usize,
        other: &ExactDyadicLineParameters2,
    ) -> Ordering {
        Self::compare_parameter_words(
            self.numerator(index),
            self.denominator(),
            Self::wide_word(other.numerator(index)),
            Self::wide_word(other.denominator()),
        )
    }
    /// Order the first retained scalar parameter against the other carrier.
    ///
    /// Uses exact signed cross-products without rational reduction. The carriers
    /// need valid nonzero denominators but need not refer to the same source line;
    /// this comparison does not establish common provenance or segment membership.
    pub fn compare_first_parameter(&self, other: &Self) -> Ordering {
        self.compare_parameter(0, other)
    }
    /// Order the second retained scalar parameter against the other carrier.
    ///
    /// Uses exact signed cross-products without rational reduction. The carriers
    /// need valid nonzero denominators but need not refer to the same source line;
    /// this comparison does not establish common provenance or segment membership.
    pub fn compare_second_parameter(&self, other: &Self) -> Ordering {
        self.compare_parameter(1, other)
    }
    /// Order the first scalar parameter against a native-word carrier.
    ///
    /// Exact widening followed by fixed-width cross-products avoids Rational
    /// allocation. This does not require or prove a common supporting line.
    pub fn compare_first_parameter_to_compact(
        &self,
        other: &ExactDyadicLineParameters2,
    ) -> Ordering {
        self.compare_parameter_to_compact(0, other)
    }
    /// Order the second scalar parameter against a native-word carrier.
    ///
    /// Exact widening followed by fixed-width cross-products avoids Rational
    /// allocation. This does not require or prove a common supporting line.
    pub fn compare_second_parameter_to_compact(
        &self,
        other: &ExactDyadicLineParameters2,
    ) -> Ordering {
        self.compare_parameter_to_compact(1, other)
    }

    /// Construct and reduce the exact parameter zero or one from its wide quotient.
    ///
    /// Zero numerator sign returns zero; otherwise the nonzero denominator and
    /// sign/magnitude invariants are trusted. Exponent differences must fit usize
    /// and can require large allocations. Invalid index or invariant violations
    /// can panic; there is no typed exhaustion or cancellation contract.
    pub(crate) fn materialize_parameter(&self, index: usize) -> Rational {
        Rational::quotient_dyadic_wide_words(
            self.numerator(index),
            self.denominator(),
            false,
        )
    }
}

/// Accumulate aligned products in six limbs and convert their retained magnitudes.
///
/// Checked arithmetic distinguishes overwide products from subtotal overflow;
/// the latter can leave wrapped scratch that callers must discard. Narrowing is
/// checked, while BigUint conversion requires a nonzero magnitude and allocates.
impl DyadicStackAccumulator {
    /// Add left * right * 2^shift exactly when the result fits 384 bits.
    ///
    /// Zero products succeed without changing the buffer, even for large shifts.
    /// An overwide shifted product returns None before mutation; a final sum carry
    /// returns None after wrapping the buffer modulo 2^384. Discard failed scratch
    /// state before fallback. Delegates to the shared verified accumulator.
    fn add_product(&mut self, left: u128, right: u128, shift: u64) -> Option<()> {
        crate::verified::accumulate::add_product(&mut self.0, left, right, shift)
    }

    /// Add a four-limb magnitude times a u128 word, aligned by 2^shift.
    ///
    /// Uses the shared verified accumulator with the same success and failure
    /// mutation contract as add_product: an overwide product leaves the buffer
    /// unchanged, but sum overflow may wrap it before returning None. No allocation
    /// is needed; None is fixed-tier ineligibility, not a general arithmetic failure.
    fn add_wide_word_product(
        &mut self,
        left: [u64; 4],
        right: u128,
        shift: u64,
    ) -> Option<()> {
        crate::verified::accumulate::add_wide_product(&mut self.0, &left, right, shift)
    }

    /// Return the exact native magnitude if all limbs above the first two are zero.
    ///
    /// Zero succeeds. A wider magnitude returns None without mutation or allocation.
    fn to_u128(self) -> Option<u128> {
        crate::verified::limbs::to_u128(&self.0)
    }

    /// Materialize a nonzero accumulator as an unsigned arbitrary-precision magnitude.
    ///
    /// Split occupied u64 limbs into little-endian u32 digits and discard high zero
    /// digits. Zero panics at the nonzero-limb assertion; callers handle zero first.
    /// The temporary vector and BigUint can allocate despite the fixed input size.
    fn into_biguint(self) -> BigUint {
        let last = self
            .0
            .iter()
            .rposition(|value| *value != 0)
            .expect("nonzero accumulator has a nonzero limb");
        let mut digits = Vec::with_capacity((last + 1) * 2);
        for limb in &self.0[..=last] {
            digits.push(*limb as u32);
            digits.push((*limb >> 32) as u32);
        }
        while digits.last() == Some(&0) {
            digits.pop();
        }
        BigUint::new(digits)
    }
}

/// Exact magnitude, aggregate and dyadic construction helpers for rational arithmetic.
impl Rational {
    /// Decode the exact finite binary64 value into a normalized native dyadic word.
    ///
    /// Both signed zeros become canonical zero. Subnormals use exponent 1074;
    /// normal values include the implicit significand bit, then cancel powers of
    /// two. NaN, infinities and values requiring more than 128 numerator bits
    /// return None. No decimal rounding, float arithmetic or Rational allocation
    /// occurs; a finite value outside this tier is not mathematically invalid.
    fn exact_dyadic_f64_word(value: f64) -> Option<DyadicWord> {
        /// Binary64 sign bit, tested without changing the input bits.
        const SIGN_MASK: u64 = 1 << 63;
        /// Binary64 eleven-bit exponent field before its shift to the low bits.
        const EXPONENT_MASK: u64 = 0x7ff0_0000_0000_0000;
        /// Binary64 explicit 52-bit fraction field, excluding the hidden leading bit.
        const SIGNIFICAND_MASK: u64 = 0x000f_ffff_ffff_ffff;
        /// Leading significand bit restored for normal binary64 values.
        const IMPLICIT_BIT: u64 = SIGNIFICAND_MASK + 1;

        let bits = value.to_bits();
        let exponent = (bits & EXPONENT_MASK) >> 52;
        let significand = bits & SIGNIFICAND_MASK;
        if exponent == 0x7ff {
            return None;
        }
        if exponent == 0 && significand == 0 {
            return Some(DyadicWord {
                sign: NoSign,
                magnitude: 0,
                denominator_shift: 0,
            });
        }
        let sign = if bits & SIGN_MASK == 0 { Plus } else { Minus };
        let (mut magnitude, mut denominator_shift) = if exponent == 0 {
            (u128::from(significand), 1074)
        } else if exponent <= 1075 {
            (u128::from(IMPLICIT_BIT + significand), 1075 - exponent)
        } else {
            let shift = u32::try_from(exponent - 1075).ok()?;
            (
                Self::checked_word_shift_left(u128::from(IMPLICIT_BIT + significand), shift)?,
                0,
            )
        };
        let common_shift = u64::from(magnitude.trailing_zeros()).min(denominator_shift);
        magnitude >>= common_shift;
        denominator_shift -= common_shift;
        Some(DyadicWord {
            sign,
            magnitude,
            denominator_shift,
        })
    }

    /// Retain a dyadic line origin and exact end-minus-start direction.
    ///
    /// Canonicalize endpoint rationals as needed, then require power-of-two
    /// denominators and u128 numerators; checked differences must also fit. None
    /// is representation ineligibility, not proof that the supporting line is
    /// invalid. Coincident endpoints succeed with zero direction. Canonicalization
    /// can allocate even though the returned carrier itself is compact.
    pub(crate) fn exact_dyadic_line2_from_rationals(
        start: [&Self; 2],
        end: [&Self; 2],
    ) -> Option<ExactDyadicLine2> {
        let point_words = |point: [&Self; 2]| {
            Some([
                Self::known_dyadic_word(point[0])?,
                Self::known_dyadic_word(point[1])?,
            ])
        };
        let start = point_words(start)?;
        let end = point_words(end)?;
        Some(ExactDyadicLine2 {
            start,
            delta: [
                Self::difference_dyadic_words(end[0], start[0])?,
                Self::difference_dyadic_words(end[1], start[1])?,
            ],
        })
    }

    /// Retain a line directly from exact binary64 endpoint values.
    ///
    /// Decode each endpoint with exact_dyadic_f64_word, then compute checked native
    /// differences. Nonfinite input, endpoint width or difference width/alignment
    /// ineligibility returns None without distinguishing those causes. Signed zero
    /// is canonicalized and coincident endpoints are allowed; no segment or
    /// nondegeneracy predicate is applied.
    pub(crate) fn exact_dyadic_line2_from_f64(
        start: [f64; 2],
        end: [f64; 2],
    ) -> Option<ExactDyadicLine2> {
        let point_words = |point: [f64; 2]| {
            Some([
                Self::exact_dyadic_f64_word(point[0])?,
                Self::exact_dyadic_f64_word(point[1])?,
            ])
        };
        let start = point_words(start)?;
        let end = point_words(end)?;
        Some(ExactDyadicLine2 {
            start,
            delta: [
                Self::difference_dyadic_words(end[0], start[0])?,
                Self::difference_dyadic_words(end[1], start[1])?,
            ],
        })
    }

    /// Build a native-word determinant plan from four rational endpoints.
    ///
    /// Retain the first line, then delegate second-line conversion and determinant
    /// construction. Endpoint canonicalization can allocate. None conflates input
    /// or intermediate tier ineligibility with a zero determinant; no segment
    /// range test is performed.
    fn line_intersection2_plan_known_dyadic(
        first_start: [&Self; 2],
        first_end: [&Self; 2],
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<DyadicLineIntersectionPlan> {
        let first = Self::exact_dyadic_line2_from_rationals(first_start, first_end)?;
        Self::line_intersection2_plan_with_first_line(
            &first,
            second_start,
            second_end,
        )
    }

    /// Reuse a retained first line and convert the second line from rationals.
    ///
    /// Canonicalize second endpoints as needed, requiring native dyadic words,
    /// then delegate to the native-word planner. None is a fallback signal, not
    /// a proof of disjointness. The retained first-line invariants are trusted.
    fn line_intersection2_plan_with_first_line(
        first: &ExactDyadicLine2,
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<DyadicLineIntersectionPlan> {
        let point_words = |point: [&Self; 2]| {
            Some([
                Self::known_dyadic_word(point[0])?,
                Self::known_dyadic_word(point[1])?,
            ])
        };
        let second_start = point_words(second_start)?;
        let second_end = point_words(second_end)?;
        Self::line_intersection2_plan_with_first_line_words(first, second_start, second_end)
    }

    /// Reuse the first line and decode exact binary64 endpoints for the second.
    ///
    /// Nonfinite or overwide inputs return None; otherwise delegate to the
    /// native-word determinant planner. No rounded endpoint or segment clipping
    /// is introduced, and the retained first-line invariants are trusted.
    fn line_intersection2_plan_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<DyadicLineIntersectionPlan> {
        let point_words = |point: [f64; 2]| {
            Some([
                Self::exact_dyadic_f64_word(point[0])?,
                Self::exact_dyadic_f64_word(point[1])?,
            ])
        };
        Self::line_intersection2_plan_with_first_line_words(
            first,
            point_words(second_start)?,
            point_words(second_end)?,
        )
    }

    /// Construct exact native-word determinants for two supporting lines.
    ///
    /// Let d1,d2 be directions and v the second origin minus the first. Retain
    /// cross(d1,d2) as denominator and cross(v,d2), cross(v,d1) as numerators.
    /// Each cross-product tries native words then a checked stack sum narrowed
    /// to u128. Any failed difference, alignment, sum or narrowing returns None.
    /// A zero denominator also returns None, covering parallel, coincident and
    /// degenerate lines without distinguishing them. Nonzero denominator does not
    /// prove that either quotient lies in `[0,1]`; callers own segment selection.
    /// Inputs must satisfy dyadic word invariants. Planning uses fixed scratch
    /// storage and does not materialize parameter/coordinate rationals.
    fn line_intersection2_plan_with_first_line_words(
        first: &ExactDyadicLine2,
        second_start: [DyadicWord; 2],
        second_end: [DyadicWord; 2],
    ) -> Option<DyadicLineIntersectionPlan> {
        let difference = |left: [DyadicWord; 2], right: [DyadicWord; 2]| {
            Some([
                Self::difference_dyadic_words(left[0], right[0])?,
                Self::difference_dyadic_words(left[1], right[1])?,
            ])
        };
        let second_delta = difference(second_end, second_start)?;
        let start_delta = difference(second_start, first.start)?;
        let cross = |left, right| {
            Self::cross_dyadic_words_word(left, right).or_else(|| {
                Self::cross_dyadic_words(left, right).and_then(Self::dyadic_stack_sum_word)
            })
        };
        let denominator = cross(first.delta, second_delta)?;
        if denominator.sign == NoSign {
            return None;
        }
        let first_numerator = cross(start_delta, second_delta)?;
        let second_numerator = cross(start_delta, first.delta)?;
        Some(DyadicLineIntersectionPlan {
            first_start: first.start,
            first_delta: first.delta,
            denominator,
            numerators: [first_numerator, second_numerator],
        })
    }

    /// Build a four-limb determinant plan from four rational endpoints.
    ///
    /// Retain the first line, then delegate second-line conversion and determinant
    /// construction. Endpoint canonicalization can allocate. None conflates input
    /// or intermediate tier ineligibility with a zero determinant; no segment
    /// range test is performed.
    fn line_intersection2_wide_plan_known_dyadic(
        first_start: [&Self; 2],
        first_end: [&Self; 2],
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<DyadicWideLineIntersectionPlan> {
        let first = Self::exact_dyadic_line2_from_rationals(first_start, first_end)?;
        Self::line_intersection2_wide_plan_with_first_line(
            &first,
            second_start,
            second_end,
        )
    }

    /// Reuse a retained first line and convert the second line from rationals.
    ///
    /// Canonicalize second endpoints as needed, requiring native dyadic words,
    /// then delegate to the four-limb planner. None is a fallback signal, not
    /// a proof of disjointness. The retained first-line invariants are trusted.
    fn line_intersection2_wide_plan_with_first_line(
        first: &ExactDyadicLine2,
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<DyadicWideLineIntersectionPlan> {
        let point_words = |point: [&Self; 2]| {
            Some([
                Self::known_dyadic_word(point[0])?,
                Self::known_dyadic_word(point[1])?,
            ])
        };
        let second_start = point_words(second_start)?;
        let second_end = point_words(second_end)?;
        Self::line_intersection2_wide_plan_with_first_line_words(first, second_start, second_end)
    }

    /// Reuse the first line and decode exact binary64 endpoints for the second.
    ///
    /// Nonfinite or overwide inputs return None; otherwise delegate to the
    /// four-limb determinant planner. No rounded endpoint or segment clipping
    /// is introduced, and the retained first-line invariants are trusted.
    fn line_intersection2_wide_plan_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<DyadicWideLineIntersectionPlan> {
        let point_words = |point: [f64; 2]| {
            Some([
                Self::exact_dyadic_f64_word(point[0])?,
                Self::exact_dyadic_f64_word(point[1])?,
            ])
        };
        Self::line_intersection2_wide_plan_with_first_line_words(
            first,
            point_words(second_start)?,
            point_words(second_end)?,
        )
    }

    /// Construct exact four-limb determinants for two supporting lines.
    ///
    /// Let d1,d2 be directions and v the second origin minus the first. Retain
    /// cross(d1,d2) as denominator and cross(v,d2), cross(v,d1) as numerators.
    /// Cross-products use checked six-limb sums narrowed to four-limb magnitudes.
    /// Any failed difference, alignment, sum or narrowing returns None.
    /// A zero denominator also returns None, covering parallel, coincident and
    /// degenerate lines without distinguishing them. Nonzero denominator does not
    /// prove that either quotient lies in `[0,1]`; callers own segment selection.
    /// Inputs must satisfy dyadic word invariants. Planning uses fixed scratch
    /// storage and does not materialize parameter/coordinate rationals.
    fn line_intersection2_wide_plan_with_first_line_words(
        first: &ExactDyadicLine2,
        second_start: [DyadicWord; 2],
        second_end: [DyadicWord; 2],
    ) -> Option<DyadicWideLineIntersectionPlan> {
        let difference = |left: [DyadicWord; 2], right: [DyadicWord; 2]| {
            Some([
                Self::difference_dyadic_words(left[0], right[0])?,
                Self::difference_dyadic_words(left[1], right[1])?,
            ])
        };
        let second_delta = difference(second_end, second_start)?;
        let start_delta = difference(second_start, first.start)?;
        let denominator = Self::dyadic_stack_sum_wide_word(Self::cross_dyadic_words(
            first.delta,
            second_delta,
        )?)?;
        if denominator.sign == NoSign {
            return None;
        }
        let first_numerator = Self::dyadic_stack_sum_wide_word(Self::cross_dyadic_words(
            start_delta,
            second_delta,
        )?)?;
        let second_numerator = Self::dyadic_stack_sum_wide_word(Self::cross_dyadic_words(
            start_delta,
            first.delta,
        )?)?;
        Some(DyadicWideLineIntersectionPlan {
            first_start: first.start,
            first_delta: first.delta,
            denominator,
            numerators: [first_numerator, second_numerator],
        })
    }

    /// Compute the exact nonnegative GCD with shape-specialized rational dispatch.
    ///
    /// Zero is an identity, including gcd(0,0)=0. Word operands use the scalar
    /// binary kernel. A single wide operand admits zero/unit/power-of-two or
    /// one-remainder shortcuts. Two wide operands try equality, powers of two,
    /// 256-bit and 512-bit fixed buffers, then the wide Lehmer/remainder route
    /// with recursive half-GCD disabled. This ordering differs from the public
    /// general GCD entry point and is retained for arithmetic scheduling.
    ///
    /// Inputs are borrowed, but output and wide temporaries can allocate. Trace
    /// labels identify the selected route; there is no total-work, cancellation
    /// or typed resource-exhaustion contract.
    pub(crate) fn gcd_magnitudes_with_mixed_width_fast_path(
        left: &BigUint,
        right: &BigUint,
    ) -> BigUint {
        match (left.to_u128(), right.to_u128()) {
            (Some(left), Some(right)) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "binary-word");
                BigUint::from(Self::gcd_word(left, right))
            }
            (Some(0), None) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "identity-wide");
                right.clone()
            }
            (None, Some(0)) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "identity-wide");
                left.clone()
            }
            (Some(1), None) | (None, Some(1)) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "identity-wide");
                BigUint::one()
            }
            (Some(word), None) if word.is_power_of_two() => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "power-of-two-wide");
                let common_shift = u64::from(word.trailing_zeros()).min(
                    right
                        .trailing_zeros()
                        .expect("nonzero BigUint has trailing zeros"),
                );
                BigUint::one() << common_shift
            }
            (None, Some(word)) if word.is_power_of_two() => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "power-of-two-wide");
                let common_shift = u64::from(word.trailing_zeros()).min(
                    left.trailing_zeros()
                        .expect("nonzero BigUint has trailing zeros"),
                );
                BigUint::one() << common_shift
            }
            (Some(word), None) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "euclidean-wide-word");
                let remainder = (right % left)
                    .to_u128()
                    .expect("remainder is smaller than a u128 divisor");
                BigUint::from(Self::gcd_word(word, remainder))
            }
            (None, Some(word)) => {
                crate::trace_dispatch!("rational_algorithm", "gcd", "euclidean-wide-word");
                let remainder = (left % right)
                    .to_u128()
                    .expect("remainder is smaller than a u128 divisor");
                BigUint::from(Self::gcd_word(word, remainder))
            }
            (None, None) => {
                if left == right {
                    crate::trace_dispatch!("rational_algorithm", "gcd", "equal-wide");
                    return left.clone();
                }
                if Self::is_power_of_two(left) || Self::is_power_of_two(right) {
                    crate::trace_dispatch!("rational_algorithm", "gcd", "power-of-two-wide");
                    let common_shift = left
                        .trailing_zeros()
                        .expect("nonzero BigUint has trailing zeros")
                        .min(
                            right
                                .trailing_zeros()
                                .expect("nonzero BigUint has trailing zeros"),
                        );
                    return BigUint::one() << common_shift;
                }
                if let Some(divisor) = Self::gcd_fixed::<4>(left, right) {
                    crate::trace_dispatch!("rational_algorithm", "gcd", "binary-fixed-256");
                    return divisor;
                }
                if let Some(divisor) = Self::gcd_fixed::<8>(left, right) {
                    crate::trace_dispatch!("rational_algorithm", "gcd", "binary-fixed-512");
                    return divisor;
                }
                let (divisor, algorithm) = Self::gcd_wide_magnitudes(left, right, false);
                crate::trace_dispatch!("rational_algorithm", "gcd", algorithm);
                #[cfg(not(feature = "dispatch-trace"))]
                let _ = algorithm;
                divisor
            }
        }
    }

    /// Initial smaller-operand width selecting the wide Lehmer schedule.
    ///
    /// The 192-bit gate is reconsidered after an initial alignment remainder;
    /// an admitted batch loop can continue down to magnitudes above 128 bits.
    /// This crossover is an implementation setting, not a work bound or a new
    /// benchmark result.
    const LEHMER_GCD_THRESHOLD_BITS: u64 = 192;
    /// Inclusive input-width cutoff for the nonrecursive half-GCD reducer.
    ///
    /// At most 1,024 maximum operand bits uses adjusted subtraction directly.
    /// This controls recursion selection, not recursion cost or memory usage.
    const HALF_GCD_RECURSION_BASE_BITS: u64 = 1024;
    /// Minimum smaller width for balanced recursive half-GCD candidate attempts.
    ///
    /// At 16,384 bits or more, the wider engine also requires operand widths to
    /// differ by at most one and the candidate flag to be enabled. Ordinary GCD
    /// entry points leave that flag off; this is not a mathematical degree limit.
    const HALF_GCD_THRESHOLD_BITS: u64 = 16_384;

    /// Exact increasing table of 5^0 through 5^55, all powers fitting u128.
    ///
    /// Word fraction reduction binary-searches this table to recognize a
    /// power-of-five odd denominator and cancel that factor directly. The next
    /// power exceeds u128; the table is a shape recognizer, not a general
    /// factorization or rational-size limit.
    const POWERS_OF_FIVE: [u128; 56] = {
        let mut powers = [1_u128; 56];
        let mut index = 1;
        while index < powers.len() {
            powers[index] = powers[index - 1] * 5;
            index += 1;
        }
        powers
    };

    /// Delegate exact GCD of two unsigned words to the shared verified scalar kernel.
    ///
    /// Zero is an identity and two zeros return zero. This adapter adds no
    /// allocation, tracing or independent arithmetic implementation.
    #[inline]
    fn gcd_u64(left: u64, right: u64) -> u64 {
        crate::verified::gcd::gcd_u64(left, right)
    }

    /// Attempt exact binary GCD within a compile-time little-endian limb tier.
    ///
    /// `WORDS` must lie from two through floor(u32::MAX/64), enforced at compile
    /// time. Either magnitude wider than 64*WORDS bits returns `None` before
    /// conversion; zeros remain supported. Admitted parts fill fixed stack arrays
    /// and use the shared verified kernel, whose word-plus-shift or limb result
    /// is materialized as a BigUint.
    ///
    /// Ordinary callers select four or eight limbs. Array occupancy is bounded
    /// for each instantiation, but huge instantiations can exceed the stack and
    /// result materialization allocates. None is tier ineligibility, not resource
    /// exhaustion or an undecided GCD.
    fn gcd_fixed<const WORDS: usize>(left: &BigUint, right: &BigUint) -> Option<BigUint> {
        const { assert!(2 <= WORDS && WORDS <= (u32::MAX / 64) as usize) };
        let fixed = |value: &BigUint| {
            if value.bits() > u64::try_from(WORDS).expect("word count fits u64") * 64 {
                return None;
            }
            let mut words = [0_u64; WORDS];
            for (index, word) in value.iter_u64_digits().enumerate() {
                words[index] = word;
            }
            Some(words)
        };
        let left = fixed(left)?;
        let right = fixed(right)?;
        match crate::verified::fixed_gcd::gcd_fixed(left, right) {
            crate::verified::fixed_gcd::FixedGcd::Word { value, shift } => {
                Some(BigUint::from(value) << shift)
            }
            crate::verified::fixed_gcd::FixedGcd::Limbs(words) => {
                let digits = words
                    .into_iter()
                    .flat_map(|word| [word as u32, (word >> 32) as u32])
                    .collect();
                Some(BigUint::new(digits))
            }
        }
    }

    /// Delegate exact u128 GCD to the shared verified binary-word kernel.
    ///
    /// Zero is an identity and two zeros return zero. The result fits the word;
    /// this adapter does not allocate or install a rational cache entry.
    fn gcd_word(left: u128, right: u128) -> u128 {
        crate::verified::gcd::gcd_u128(left, right)
    }

    /// Select a signed Lehmer reduction matrix from a shared leading-bit window.
    ///
    /// Requires `larger >= smaller` (debug asserted). Shift both magnitudes by
    /// max(bits(larger)-62,0), then pass the resulting nonnegative signed words
    /// to the verified matrix selector. It returns a word-bounded unimodular
    /// matrix only after a sufficient batch; inability to convert or select a
    /// batch returns `None` for a Euclidean fallback.
    ///
    /// This builds a matrix, not the full GCD. Applying it to complete magnitudes
    /// still requires the caller's exact row/progress check. Head extraction can
    /// allocate shifted integers and has no work/cancellation budget.
    fn lehmer_gcd_matrix(larger: &BigUint, smaller: &BigUint) -> Option<[i128; 4]> {
        debug_assert!(larger >= smaller);
        let shift = larger.bits().saturating_sub(62);
        let high_larger = (larger >> usize::try_from(shift).ok()?).to_i128()?;
        let high_smaller = (smaller >> usize::try_from(shift).ok()?).to_i128()?;
        crate::verified::lehmer::matrix(high_larger, high_smaller)
    }

    /// Apply signed matrix rows as unsigned magnitudes and require strict progress.
    ///
    /// Each coefficient magnitude must fit u64. Rows use sums for equal signs
    /// and absolute differences otherwise, avoiding signed BigInt temporaries.
    /// Both result magnitudes must be strictly less than `larger`; coefficient
    /// ineligibility or failed progress returns `None`. No sorting is performed.
    ///
    /// GCD preservation requires a caller-proved unimodular matrix. This routine
    /// does not check that proof: even a zero matrix can pass its local progress
    /// test for a positive larger input. Wide products allocate without a budget.
    fn apply_lehmer_gcd_matrix(
        larger: &BigUint,
        smaller: &BigUint,
        [a, b, c, d]: [i128; 4],
    ) -> Option<(BigUint, BigUint)> {
        // Apply signed one-word coefficients without cloning wide values into
        // `BigInt`; a row magnitude is a sum or an absolute difference.
        let apply_row = |left: i128, right: i128| {
            let (left_coefficient, right_coefficient, add) =
                crate::verified::lehmer::row_coefficients(left, right)?;
            let mut left_product = larger * left_coefficient;
            let right_product = smaller * right_coefficient;
            if add {
                left_product += right_product;
                Some(left_product)
            } else if left_product >= right_product {
                Some(left_product - right_product)
            } else {
                Some(right_product - left_product)
            }
        };
        let first = apply_row(a, b)?;
        let second = apply_row(c, d)?;
        if &first >= larger || &second >= larger {
            return None;
        }
        Some((first, second))
    }

    /// Return the bit length of the exact absolute difference, zero for equality.
    ///
    /// This helper allocates the unsigned difference and ignores operand order.
    /// Half-GCD uses the result as a stop/progress measure, not as a bound on
    /// the individual operand widths or the cost of subsequent arithmetic.
    fn magnitude_difference_bits(left: &BigUint, right: &BigUint) -> u64 {
        if left >= right {
            (left - right).bits()
        } else {
            (right - left).bits()
        }
    }

    /// Attempt one adjusted Euclidean subtraction while transporting reconstruction.
    ///
    /// Both remainders must be positive and the matrix must reconstruct the
    /// original pair from them. Divide the larger by the smaller; when the
    /// ordinary remainder has at most stop_bits bits, reduce the quotient by
    /// one and add back the divisor. This avoids overshooting the stop scale
    /// when the maintained remainder invariant puts that divisor above it.
    ///
    /// Equal remainders, a zero adjusted quotient or absent strict decrease
    /// return `None` before mutation. Success changes one remainder and the
    /// opposite reconstruction column. A zero divisor is not validated and can
    /// panic. Products/matrix updates have no total work or allocation budget.
    fn half_gcd_sdiv_step(
        left: &mut BigUint,
        right: &mut BigUint,
        stop_bits: u64,
        matrix: &mut HalfGcdMatrix,
    ) -> Option<()> {
        if left == right {
            return None;
        }
        if left > right {
            let mut quotient = &*left / &*right;
            let mut remainder = &*left
                - Self::multiply_magnitudes("half-gcd-sdiv-product", &quotient, right);
            if remainder.bits() <= stop_bits {
                quotient -= 1_u8;
                remainder += &*right;
            }
            if quotient.is_zero() || remainder >= *left {
                return None;
            }
            *left = remainder;
            matrix.update_right_column(&quotient);
        } else {
            let mut quotient = &*right / &*left;
            let mut remainder = &*right
                - Self::multiply_magnitudes("half-gcd-sdiv-product", &quotient, left);
            if remainder.bits() <= stop_bits {
                quotient -= 1_u8;
                remainder += &*left;
            }
            if quotient.is_zero() || remainder >= *right {
                return None;
            }
            *right = remainder;
            matrix.update_left_column(&quotient);
        }
        Some(())
    }

    /// Build a reconstruction by adjusted subtraction until the difference is small.
    ///
    /// For positive inputs, set stop_bits=max(input widths)/2+1, start from the
    /// identity matrix, and repeat adjusted steps while the absolute difference
    /// exceeds that width. Return `None` if a step cannot progress. Success
    /// preserves the original pair through the returned determinant-one matrix.
    ///
    /// The stopping condition bounds the difference, not both remainder widths;
    /// close inputs can return unchanged. Zero inputs violate the private divisor
    /// precondition. Cloned remainders and coefficients have no resource budget.
    fn half_gcd_slow(left: &BigUint, right: &BigUint) -> Option<HalfGcdReduction> {
        let stop_bits = left.bits().max(right.bits()) / 2 + 1;
        let mut left = left.clone();
        let mut right = right.clone();
        let mut matrix = HalfGcdMatrix::identity();
        while Self::magnitude_difference_bits(&left, &right) > stop_bits {
            Self::half_gcd_sdiv_step(&mut left, &mut right, stop_bits, &mut matrix)?;
        }
        Some(HalfGcdReduction {
            left,
            right,
            matrix,
        })
    }

    /// Attempt a recursive half-GCD reconstruction for the wide candidate schedule.
    ///
    /// The wide caller supplies balanced positive inputs; recursive calls use
    /// selected high parts whose nonzero-divisor invariants must be maintained.
    /// At most 1,024 input bits uses the slow reducer. Larger inputs reduce high
    /// halves, apply their inverse matrix to full magnitudes, take adjusted steps,
    /// then optionally reduce a second high window and compose its matrix.
    /// A final step loop meets the same difference-width stop condition.
    ///
    /// Ineligible shifts, negative inverse results or lack of step progress can
    /// return `None`; selected second-window cases fall back to the slow reducer.
    /// This is not validation of arbitrary unbalanced or zero inputs: truncation
    /// can create a zero divisor and panic outside the calling invariant. A
    /// successful matrix preserves exact GCD, but neither recursion nor coefficient
    /// growth is governed by a total-work, memory or cancellation budget.
    fn half_gcd_reduce(left: &BigUint, right: &BigUint) -> Option<HalfGcdReduction> {
        let input_bits = left.bits().max(right.bits());
        if input_bits <= Self::HALF_GCD_RECURSION_BASE_BITS {
            return Self::half_gcd_slow(left, right);
        }

        let stop_bits = input_bits / 2 + 1;
        let three_quarter_bits = input_bits.saturating_mul(3) / 4;
        let first_shift = input_bits / 2;
        let first_shift_usize = usize::try_from(first_shift).ok()?;
        let high_left = left >> first_shift_usize;
        let high_right = right >> first_shift_usize;
        let first = Self::half_gcd_reduce(&high_left, &high_right)?;
        let mut matrix = first.matrix;
        let (mut reduced_left, mut reduced_right) = matrix.apply_inverse(left, right)?;

        while reduced_left.bits().max(reduced_right.bits()) > three_quarter_bits + 1
            && Self::magnitude_difference_bits(&reduced_left, &reduced_right) > stop_bits
        {
            Self::half_gcd_sdiv_step(
                &mut reduced_left,
                &mut reduced_right,
                stop_bits,
                &mut matrix,
            )?;
        }

        let reduced_bits = reduced_left.bits().max(reduced_right.bits());
        if reduced_bits > stop_bits + 2 {
            let second_shift = stop_bits
                .saturating_mul(2)
                .checked_sub(reduced_bits)?
                .checked_add(1)?;
            if second_shift == 0 {
                return Self::half_gcd_slow(left, right);
            }
            let second_shift_usize = usize::try_from(second_shift).ok()?;
            let high_left = &reduced_left >> second_shift_usize;
            let high_right = &reduced_right >> second_shift_usize;
            if high_left.bits().max(high_right.bits()) >= input_bits {
                return Self::half_gcd_slow(left, right);
            }
            let second = Self::half_gcd_reduce(&high_left, &high_right)?;
            let (next_left, next_right) = second
                .matrix
                .apply_inverse(&reduced_left, &reduced_right)?;
            reduced_left = next_left;
            reduced_right = next_right;
            matrix.multiply_right(&second.matrix);
        }

        while Self::magnitude_difference_bits(&reduced_left, &reduced_right) > stop_bits {
            Self::half_gcd_sdiv_step(
                &mut reduced_left,
                &mut reduced_right,
                stop_bits,
                &mut matrix,
            )?;
        }
        Some(HalfGcdReduction {
            left: reduced_left,
            right: reduced_right,
            matrix,
        })
    }

    /// Return the exact nonnegative GCD shared by rational and coefficient reduction.
    ///
    /// Zero is an identity and gcd(0,0)=0. After zero and u128 shortcuts, the wide
    /// engine uses Euclidean/Lehmer reduction with recursive half-GCD disabled.
    /// This public entry does not use the mixed-width helper's fixed-limb tiers.
    ///
    /// Dispatch tracing records the selected label and operand/result widths;
    /// the zero shortcut uses the binary-word label even for a wide other input.
    /// Borrowing inputs avoids owning them, not allocation of the output and wide
    /// temporaries. There is no total-work/memory budget or typed exhaustion.
    pub fn gcd_magnitudes(left: &BigUint, right: &BigUint) -> BigUint {
        let (divisor, algorithm) = if left.is_zero() {
            (right.clone(), "binary-word")
        } else if right.is_zero() {
            (left.clone(), "binary-word")
        } else if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            (
                BigUint::from(Self::gcd_word(left, right)),
                "binary-word",
            )
        } else {
            Self::gcd_wide_magnitudes(left, right, false)
        };

        #[cfg(feature = "dispatch-trace")]
        {
            crate::trace_dispatch!("rational_algorithm", "gcd", algorithm);
            crate::dispatch_trace::record_rational_gcd(left, right, &divisor);
        }
        #[cfg(not(feature = "dispatch-trace"))]
        let _ = algorithm;
        divisor
    }

    /// Compute the exact nonnegative GCD with the half-GCD candidate disabled.
    ///
    /// Zero and word shortcuts precede the same wide schedule used by the
    /// ordinary public GCD. This benchmark baseline does not publish that entry
    /// point's final GCD trace/operand record. Its name is not a promise that a
    /// Lehmer batch is actually selected, nor a measured performance claim.
    /// Wide temporaries are unbudgeted and zero/zero returns zero.
    pub fn gcd_magnitudes_lehmer_baseline(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() {
            return right.clone();
        }
        if right.is_zero() {
            return left.clone();
        }
        if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            return BigUint::from(Self::gcd_word(left, right));
        }
        Self::gcd_wide_magnitudes(left, right, false).0
    }

    /// Compute the exact GCD with the experimental recursive half-GCD route enabled.
    ///
    /// Zero and word shortcuts return directly. Wide pairs attempt recursion only
    /// at the configured smaller-width threshold and balanced-size condition;
    /// remaining work uses Lehmer/Euclidean reduction. Ineligible or nonprogressing
    /// candidate work falls back without changing the exact answer.
    ///
    /// Only the wide route emits this entry's final algorithm/operand trace;
    /// zero/word shortcuts do not. The candidate label does not promise faster
    /// completion, bounded resources or that every call uses recursion.
    pub fn gcd_magnitudes_half_gcd_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() {
            return right.clone();
        }
        if right.is_zero() {
            return left.clone();
        }
        if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            return BigUint::from(Self::gcd_word(left, right));
        }
        let (divisor, algorithm) = Self::gcd_wide_magnitudes(left, right, true);
        #[cfg(feature = "dispatch-trace")]
        {
            crate::trace_dispatch!("rational_algorithm", "gcd", algorithm);
            crate::dispatch_trace::record_rational_gcd(left, right, &divisor);
        }
        #[cfg(not(feature = "dispatch-trace"))]
        let _ = algorithm;
        divisor
    }

    /// Reduce two positive magnitudes and report the effective wide GCD route.
    ///
    /// Sort by magnitude. A word-sized smaller operand needs one remainder then
    /// word GCD. With the candidate enabled, balanced pairs with smaller width
    /// at least 16,384 attempt half-GCD, subtract the resulting pair once and
    /// require a decrease in maximum width before continuing recursion.
    ///
    /// Lehmer eligibility starts at 192 smaller bits, with one alignment remainder
    /// for initially unbalanced sizes. Once selected, batches can continue below
    /// that threshold while the smaller value exceeds 128 bits; failed matrix or
    /// progress checks use ordinary exact remainders. The label prefers successful
    /// half-GCD work, then used Lehmer batches, otherwise the remainder route.
    ///
    /// Zero inputs must be handled by callers: the initial word reduction divides
    /// by the smaller magnitude. Output/coefficients can grow or allocate without
    /// a total work/memory budget, cancellation hook or typed resource outcome.
    fn gcd_wide_magnitudes(
        left: &BigUint,
        right: &BigUint,
        half_gcd_enabled: bool,
    ) -> (BigUint, &'static str) {

        let (larger, smaller) = if left >= right {
            (left, right)
        } else {
            (right, left)
        };
        if let Some(smaller_word) = smaller.to_u128() {
            let remainder = (larger % smaller)
                .to_u128()
                .expect("remainder is smaller than a u128 divisor");
            return (
                BigUint::from(Self::gcd_word(smaller_word, remainder)),
                "euclidean-wide-word",
            );
        }

        let mut larger = larger.clone();
        let mut smaller = smaller.clone();
        let mut used_half_gcd = false;
        while half_gcd_enabled
            && smaller.bits() >= Self::HALF_GCD_THRESHOLD_BITS
            && larger.bits().abs_diff(smaller.bits()) <= 1
        {
            let before_bits = larger.bits().max(smaller.bits());
            let Some(reduction) = Self::half_gcd_reduce(&larger, &smaller) else {
                break;
            };
            let (mut first, mut second) = (reduction.left, reduction.right);
            if first == second {
                return (first, "recursive-half-gcd");
            }
            if first > second {
                first -= &second;
            } else {
                second -= &first;
            }
            (larger, smaller) = if first >= second {
                (first, second)
            } else {
                (second, first)
            };
            if larger.bits().max(smaller.bits()) >= before_bits {
                break;
            }
            used_half_gcd = true;
        }
        let mut lehmer_selected = smaller.bits() >= Self::LEHMER_GCD_THRESHOLD_BITS;
        if lehmer_selected && larger.bits().abs_diff(smaller.bits()) > 1 {
            let remainder = &larger % &smaller;
            larger = smaller;
            smaller = remainder;
            lehmer_selected = smaller.bits() >= Self::LEHMER_GCD_THRESHOLD_BITS
                && larger.bits().abs_diff(smaller.bits()) <= 1;
        }
        let mut used_lehmer = false;
        while !smaller.is_zero() {
            if lehmer_selected
                && smaller.bits() > 128
                && larger.bits().abs_diff(smaller.bits()) <= 1
                && let Some(matrix) = Self::lehmer_gcd_matrix(&larger, &smaller)
                && let Some((first, second)) =
                    Self::apply_lehmer_gcd_matrix(&larger, &smaller, matrix)
            {
                used_lehmer = true;
                (larger, smaller) = if first >= second {
                    (first, second)
                } else {
                    (second, first)
                };
                continue;
            }
            let remainder = &larger % &smaller;
            larger = smaller;
            smaller = remainder;
        }
        (
            larger,
            if used_half_gcd {
                "recursive-half-gcd"
            } else if used_lehmer {
                "lehmer-leading-limb"
            } else {
                "euclidean-wide-remainder"
            },
        )
    }

    /// Scale a rational slice by the positive LCM of its stored denominators.
    ///
    /// Every output is an exact integer and all inputs receive the same scale,
    /// preserving projective coordinates or polynomial zero sets. Internally
    /// unreduced inputs are supported without canonicalizing their caches; the
    /// stored LCM need not be minimal for the reduced values. Empty input returns
    /// an empty vector before denominator work or tracing. Nonempty inputs emit
    /// the clearing trace. Big integers and the result vector allocate without a
    /// total-work, memory or cancellation budget.
    pub fn clear_common_denominator_slice(values: &[&Self]) -> Vec<Self> {
        if values.is_empty() {
            return Vec::new();
        }
        let common_denominator = Self::common_denominator(values);
        crate::trace_dispatch!("rational", "common-scale", "clear-denominator-slice");
        values
            .iter()
            .map(|value| {
                if value.sign == NoSign {
                    return Self::zero();
                }
                let scale = &common_denominator / &value.denominator;
                Self::from_integer_magnitude(value.sign, &value.numerator * scale)
            })
            .collect()
    }

    /// Return the nonnegative integer GCD of the two reduced numerator magnitudes.
    ///
    /// Canonicalize each operand first, then ignore its sign and denominator.
    /// Zero is the identity and two zeros return zero, not a positive integer.
    /// Projective and affine consumers can reuse this integer content without
    /// owning a second GCD backend. Canonicalization can warm caches; reduction
    /// and result construction can allocate without a total resource budget.
    pub fn numerator_magnitude_gcd(&self, other: &Self) -> Self {
        let left = self.canonicalized_ref();
        let right = other.canonicalized_ref();
        Self::from_unsigned_integer(Self::gcd_magnitudes_with_mixed_width_fast_path(
            &left.numerator,
            &right.numerator,
        ))
    }

    /// Return the reduced unsigned numerator of a difference of exact dyadics.
    ///
    /// Canonicalize both operands, require power-of-two denominators, align signed
    /// magnitudes and cancel powers of two from their absolute difference. Equal
    /// values return integer zero. None means a nondyadic operand or a shift that
    /// does not fit usize, not exhausted resources or an undecided numerical value.
    /// This avoids installing retained linear-result relations, but can still warm
    /// canonical caches and allocate wide magnitudes without a work/memory budget.
    pub fn dyadic_difference_numerator_magnitude(&self, other: &Self) -> Option<Self> {
        let left = self.canonicalized_ref();
        let right = other.canonicalized_ref();
        let left_shift = Self::biguint_power_of_two_shift(&left.denominator)?;
        let right_shift = Self::biguint_power_of_two_shift(&right.denominator)?;
        let denominator_shift = left_shift.max(right_shift);
        let left_scale = usize::try_from(denominator_shift - left_shift).ok()?;
        let right_scale = usize::try_from(denominator_shift - right_shift).ok()?;
        let left_magnitude = &left.numerator << left_scale;
        let right_magnitude = &right.numerator << right_scale;
        let mut magnitude = match (left.sign, right.sign) {
            (NoSign, NoSign) => return Some(Self::zero()),
            (NoSign, _) => right_magnitude,
            (_, NoSign) => left_magnitude,
            (left_sign, right_sign) if left_sign != right_sign => {
                left_magnitude + right_magnitude
            }
            _ => match left_magnitude.cmp(&right_magnitude) {
                Ordering::Less => right_magnitude - left_magnitude,
                Ordering::Equal => return Some(Self::zero()),
                Ordering::Greater => left_magnitude - right_magnitude,
            },
        };
        let reduction_shift = magnitude
            .trailing_zeros()
            .expect("nonzero magnitude has trailing zeros")
            .min(denominator_shift);
        magnitude >>= usize::try_from(reduction_shift).ok()?;
        Some(Self::from_unsigned_integer(magnitude))
    }

    /// Scale a fixed rational tuple to signed primitive integer components.
    ///
    /// Use one positive scale: clear stored denominators, then divide every integer
    /// by the common magnitude GCD. This preserves orientation and projective
    /// meaning; it does not force the first nonzero component positive. Empty and
    /// all-zero arrays preserve shape. A lone nonzero component becomes its signed
    /// unit without constructing a common denominator. Valid internally unreduced
    /// inputs are supported without requiring canonicalization first.
    ///
    /// Multiple-component arithmetic and output handles can allocate without a
    /// resource budget. The primitive-integer-ratio trace is emitted only when the
    /// final content-division loop runs, not on every successful normalization.
    pub fn primitive_integer_ratio<const N: usize>(values: [&Self; N]) -> [Self; N] {
        let mut single = None;
        let mut multiple = false;
        for (index, value) in values.iter().enumerate() {
            if value.sign == NoSign {
                continue;
            }
            if single.is_some() {
                multiple = true;
                break;
            }
            single = Some((index, value.sign));
        }
        if !multiple {
            let Some((single_index, single_sign)) = single else {
                return values.map(|_| Self::zero());
            };
            return core::array::from_fn(|index| {
                if index == single_index {
                    Self::from_integer_magnitude(single_sign, BigUint::one())
                } else {
                    Self::zero()
                }
            });
        }
        let common_denominator = Self::common_denominator(&values);
        let mut integers = values.map(|value| {
            if value.sign == NoSign {
                return Self::zero();
            }
            let scale = &common_denominator / &value.denominator;
            Self::from_integer_magnitude(value.sign, &value.numerator * scale)
        });
        let mut content = BigUint::ZERO;
        for value in &integers {
            if value.sign == NoSign {
                continue;
            }
            content = if content.is_zero() {
                value.numerator.clone()
            } else {
                Self::gcd_magnitudes_with_mixed_width_fast_path(&content, &value.numerator)
            };
            if content.is_one() {
                return integers;
            }
        }
        if content.is_zero() || content.is_one() {
            return integers;
        }
        for value in &mut integers {
            if value.sign != NoSign {
                *value =
                    Self::from_integer_magnitude(value.sign, &value.numerator / &content);
            }
        }
        crate::trace_dispatch!("rational", "common-scale", "primitive-integer-ratio");
        integers
    }

    /// Normalize a rational slice directly to primitive signed BigInt components.
    ///
    /// The positive shared scaling has the same mathematical result as
    /// [`Self::primitive_integer_ratio`], preserving zero slots and sign orientation.
    /// Unlike that fixed-array routine, a lone nonzero entry still goes through
    /// stored-denominator clearing and content computation. Empty input returns
    /// before work or tracing; every nonempty input emits this routine's trace.
    ///
    /// Stored unreduced parts are supported without warming canonical caches.
    /// Temporary integers and the vector allocate without a total-work, memory or
    /// cancellation budget; avoiding Rational wrappers does not make this allocation-free.
    pub fn primitive_bigint_ratio(values: &[&Self]) -> Vec<BigInt> {
        if values.is_empty() {
            return Vec::new();
        }
        let common_denominator = Self::common_denominator(values);
        crate::trace_dispatch!("rational", "common-scale", "primitive-bigint-ratio");
        let mut integers = values
            .iter()
            .map(|value| {
                if value.sign == NoSign {
                    return BigInt::ZERO;
                }
                let scale = &common_denominator / &value.denominator;
                BigInt::from_biguint(value.sign, &value.numerator * scale)
            })
            .collect::<Vec<_>>();

        let mut content = BigUint::ZERO;
        for value in &integers {
            if value.is_zero() {
                continue;
            }
            content = if content.is_zero() {
                value.magnitude().clone()
            } else {
                Self::gcd_magnitudes_with_mixed_width_fast_path(&content, value.magnitude())
            };
            if content.is_one() {
                return integers;
            }
        }
        if content.is_zero() || content.is_one() {
            return integers;
        }
        let content = BigInt::from_biguint(Plus, content);
        for value in &mut integers {
            if !value.is_zero() {
                *value /= &content;
            }
        }
        integers
    }

    /// Scale a nonempty fixed tuple by the LCM of its stored denominators.
    ///
    /// All outputs are exact integers under one positive scale, which vector
    /// normalization can remove before self-dot and square-root work. No canonical
    /// cache observation is required, including for internally unreduced inputs.
    /// Requires N > 0: unlike the slice entry, an empty array reaches the helper's
    /// nonempty assertion/indexing and panics. This routine always emits its clearing
    /// trace after LCM construction; wide arithmetic has no resource budget.
    pub(crate) fn clear_common_denominator<const N: usize>(values: [&Self; N]) -> [Self; N] {
        let common_denominator = Self::common_denominator(&values);

        crate::trace_dispatch!("rational", "common-scale", "clear-denominator");
        core::array::from_fn(|index| {
            let value = values[index];
            if value.sign == NoSign {
                return Self::zero();
            }
            let scale = &common_denominator / &value.denominator;
            Self::from_integer_magnitude(value.sign, &value.numerator * scale)
        })
    }

    /// Evaluate a finite constant-first rational polynomial using integer Horner steps.
    ///
    /// Collect and canonicalize the entire coefficient iterator before selecting
    /// the last nonzero coefficient. Empty/all-zero polynomials yield zero; a
    /// constant or zero argument returns the canonical constant coefficient. The
    /// iterator is still consumed in those cases, so it must terminate and its
    /// later panics remain observable.
    ///
    /// For remaining inputs, clear coefficient denominators by D and maintain the
    /// suffix as numerator/(D*denominator_power) while multiplying by argument p/q.
    /// A zero suffix resets the denominator power before continuing. Only this
    /// Horner path emits the homogeneous-integer-horner trace. Nonzero noninteger
    /// output retains an exact internally unreduced ratio for later observation;
    /// integer and zero outputs use their direct constructors. Allocation and
    /// coefficient/power growth have no total-work or cancellation budget.
    pub(crate) fn eval_polynomial<'a>(
        coefficients: impl Iterator<Item = &'a Self>,
        argument: &Self,
    ) -> Self {
        let coefficients = coefficients
            .map(Self::canonicalized_ref)
            .collect::<Vec<_>>();
        let Some(degree) = coefficients.iter().rposition(|value| !value.is_zero()) else {
            return Self::zero();
        };
        if degree == 0 || argument.is_zero() {
            return coefficients[0].clone();
        }
        let coefficients = &coefficients[..=degree];
        let common_denominator = Self::common_denominator(coefficients);
        let integer_coefficient = |value: &Self| {
            if value.is_zero() {
                BigInt::ZERO
            } else if value.denominator == common_denominator {
                BigInt::from_biguint(value.sign, value.numerator.clone())
            } else {
                let scale = &common_denominator / &value.denominator;
                BigInt::from_biguint(value.sign, &value.numerator * scale)
            }
        };
        let argument = argument.canonicalized_ref();
        let p = BigInt::from_biguint(argument.sign, argument.numerator.clone());
        let q = BigInt::from_biguint(Plus, argument.denominator.clone());
        let mut numerator = integer_coefficient(coefficients[degree]);
        let mut denominator_power = BigInt::one();
        for coefficient in coefficients[..degree].iter().rev() {
            if numerator.is_zero() {
                // A vanishing suffix has no remaining denominator dependence.
                denominator_power = BigInt::one();
                numerator = integer_coefficient(coefficient);
                continue;
            }
            // The suffix value is numerator / (D * denominator_power).
            // Multiplication by p/q and addition of C/D preserve that invariant.
            numerator *= &p;
            denominator_power *= &q;
            if !coefficient.is_zero() {
                numerator += integer_coefficient(coefficient) * &denominator_power;
            }
        }
        crate::trace_dispatch!("rational", "polynomial", "homogeneous-integer-horner");
        let (sign, numerator) = numerator.into_parts();
        if sign == NoSign {
            return Self::zero();
        }
        let denominator = common_denominator * denominator_power.into_parts().1;
        if denominator.is_one() {
            return Self::from_integer_magnitude(sign, numerator);
        }
        // Sign and zero queries need no GCD. Numeric consumers canonicalize
        // this existing exact ratio representation once, on demand.
        Self::from_parts_raw_unreduced(sign, numerator, denominator)
    }

    /// Compute the positive LCM of stored denominators in a nonempty rational slice.
    ///
    /// Return the shared denominator if all agree; otherwise use the largest power
    /// of two when every denominator is dyadic, then an ordered GCD/LCM fold for
    /// the general case. This reads stored parts without canonicalization, so the
    /// result can exceed the minimal denominator of reduced values. Positive input
    /// denominators are a Rational invariant. Empty input panics: the debug guard
    /// is followed by paths that index the first element. Wide work is unbudgeted.
    fn common_denominator(values: &[&Self]) -> BigUint {
        debug_assert!(!values.is_empty());
        if values
            .iter()
            .all(|value| value.denominator == values[0].denominator)
        {
            values[0].denominator.clone()
        } else if values
            .iter()
            .all(|value| Self::is_power_of_two(&value.denominator))
        {
            values
                .iter()
                .max_by_key(|value| value.denominator.bits())
                .expect("fixed rational set is nonempty")
                .denominator
                .clone()
        } else {
            values.iter().skip(1).fold(
                values[0].denominator.clone(),
                |common, value| {
                    let divisor = Self::gcd_magnitudes(&common, &value.denominator);
                    (common / divisor) * &value.denominator
                },
            )
        }
    }

    /// Reduce (positive - negative)/denominator and select an exact Rational result.
    ///
    /// Requires a positive denominator. Equal totals return zero and trace it
    /// before checking that precondition; an invalid zero denominator can therefore
    /// be masked by cancellation, while other invalid calls can panic. Dyadic
    /// reduction cancels powers of two; a power-of-five odd part uses direct factor
    /// cancellation. Other denominators use the shared verified fraction reducer.
    /// Their three/seven/smooth/size classifications only label traces.
    ///
    /// The reduced nonzero result then uses the canonical word constructor and its
    /// cache schedule. Word arithmetic is bounded, but constructing a Rational can
    /// allocate. This private routine does not return typed validation failures.
    fn from_word_magnitude_difference(
        positive: u128,
        negative: u128,
        denominator: u128,
    ) -> Self {
        let (sign, magnitude) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => {
                crate::trace_dispatch!("rational", "word-result", "zero");
                return Self::zero();
            }
        };
        if denominator.is_power_of_two() {
            let common_shift = magnitude.trailing_zeros().min(denominator.trailing_zeros());
            return Self::from_reduced_word_parts(
                sign,
                magnitude >> common_shift,
                denominator >> common_shift,
            );
        }

        let common_shift = magnitude.trailing_zeros().min(denominator.trailing_zeros());
        let mut magnitude = magnitude >> common_shift;
        let mut denominator = denominator >> common_shift;
        let odd_denominator = denominator >> denominator.trailing_zeros();
        if Self::POWERS_OF_FIVE
            .binary_search(&odd_denominator)
            .is_ok()
        {
            crate::trace_dispatch!("rational", "word-reduction", "power-of-five-denominator");
            while denominator.is_multiple_of(5) && magnitude.is_multiple_of(5) {
                denominator /= 5;
                magnitude /= 5;
            }
        } else {
            #[cfg(feature = "dispatch-trace")]
            {
                let path = if word_is_power_of(odd_denominator, 3) {
                    "power-of-three-denominator"
                } else if word_is_power_of(odd_denominator, 7) {
                    "power-of-seven-denominator"
                } else if word_is_smooth_357(odd_denominator) {
                    "mixed-357-smooth-denominator"
                } else if odd_denominator <= u128::from(u16::MAX) {
                    "other-small-odd-denominator"
                } else if odd_denominator <= u128::from(u64::MAX) {
                    "other-word-odd-denominator"
                } else {
                    "other-wide-odd-denominator"
                };
                crate::trace_dispatch!("rational", "word-reduction", path);
            }
            (magnitude, denominator) = crate::verified::fraction::reduce(magnitude, denominator);
        }
        Self::from_reduced_word_parts(sign, magnitude, denominator)
    }

    /// Construct a nonzero Rational from caller-proved reduced native parts.
    ///
    /// Requires nonzero sign/magnitude, positive denominator and coprimality. Debug
    /// assertions check only the first three; no GCD proves the last. Units, small
    /// integers, small dyadics and small general fractions try shared cached values
    /// in that order before allocating raw BigUint parts. Traces report the chosen
    /// result class. Passing noncoprime unequal parts can publish storage marked
    /// reduced despite violating its invariant; this is not a normalizing constructor.
    fn from_reduced_word_parts(sign: Sign, magnitude: u128, denominator: u128) -> Self {
        debug_assert_ne!(sign, NoSign);
        debug_assert_ne!(magnitude, 0);
        debug_assert_ne!(denominator, 0);
        if magnitude == denominator {
            crate::trace_dispatch!("rational", "word-result", "unit");
            return if sign == Minus {
                Self::minus_one()
            } else {
                Self::one()
            };
        }
        if denominator == 1 {
            if let Some(value) = Self::small_integer(sign, magnitude) {
                crate::trace_dispatch!("rational", "word-result", "cached-small-integer");
                return value;
            }
            #[cfg(feature = "dispatch-trace")]
            {
                let path = match magnitude {
                    0..=127 => "uncached-integer-65-127",
                    128..=255 => "uncached-integer-128-255",
                    256..=1023 => "uncached-integer-256-1023",
                    1024..=4095 => "uncached-integer-1024-4095",
                    _ => "uncached-integer-wide",
                };
                crate::trace_dispatch!("rational", "word-result", path);
            }
        } else if denominator.is_power_of_two() {
            if let Some(value) = Self::small_reduced_dyadic(sign, magnitude, denominator) {
                crate::trace_dispatch!("rational", "word-result", "cached-small-dyadic");
                return value;
            }
            crate::trace_dispatch!("rational", "word-result", "dyadic-fraction");
        } else {
            if let Some(value) =
                Self::small_reduced_general_fraction(sign, magnitude, denominator)
            {
                crate::trace_dispatch!("rational", "word-result", "cached-small-general-fraction");
                return value;
            }
            if magnitude <= u128::from(u64::MAX) && denominator <= u128::from(u64::MAX) {
                crate::trace_dispatch!("rational", "word-result", "small-general-fraction");
            } else {
                crate::trace_dispatch!("rational", "word-result", "wide-general-fraction");
            }
        }
        Self::from_parts_raw(
            sign,
            Self::biguint_from_word(magnitude),
            Self::biguint_from_word(denominator),
        )
    }

    /// Convert an exact u128 magnitude to BigUint using a u64 value when it fits.
    ///
    /// Zero is supported. Both branches preserve the integer exactly; the narrower
    /// branch selects the corresponding From implementation, not a different
    /// arithmetic result or a guaranteed allocation-free path.
    #[inline]
    fn biguint_from_word(value: u128) -> BigUint {
        if let Ok(value) = u64::try_from(value) {
            BigUint::from(value)
        } else {
            BigUint::from(value)
        }
    }

    /// Attempt the unsigned magnitude/denominator product of stored rational factors.
    ///
    /// Signs are handled by the caller. Each stored part must fit u128; checked
    /// prefix products are tried first without cross-cancellation. A product overflow
    /// restarts the complete term through pairwise cancellation, but a part too wide
    /// to convert returns None directly. The successful direct pair need not be
    /// reduced. Empty factors give (1,1). A later zero does not bypass earlier
    /// conversion or prefix bounds; None means schedule ineligibility, not proof
    /// that the final exact product is too large.
    fn product_term_words<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut magnitude = 1_u128;
        let mut denominator = 1_u128;
        for factor in term {
            let numerator = factor.numerator.to_u128()?;
            let factor_denominator = factor.denominator.to_u128()?;
            let Some(next_magnitude) = magnitude.checked_mul(numerator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            let Some(next_denominator) = denominator.checked_mul(factor_denominator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            magnitude = next_magnitude;
            denominator = next_denominator;
        }
        Some((magnitude, denominator))
    }

    /// Attempt a native unsigned product after numerator-major cross-cancellation.
    ///
    /// First convert every stored numerator and positive denominator to u128, then
    /// delegate cancellation and checked prefix multiplication to the shared
    /// verified kernel. Signs do not participate. Empty input gives (1,1); a zero
    /// factor can still arrive after an overflowing multiplication prefix. None
    /// means conversion/prefix ineligibility, not resource exhaustion. Const-sized
    /// arrays use stack space proportional to FACTORS with no practical stack cap.
    fn product_term_words_cross_cancelled<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut numerators = [0_u128; FACTORS];
        let mut denominators = [1_u128; FACTORS];
        for i in 0..FACTORS {
            numerators[i] = term[i].numerator.to_u128()?;
            denominators[i] = term[i].denominator.to_u128()?;
        }

        crate::verified::fraction::cross_cancelled_product(numerators, denominators)
    }

    /// Attempt an exact rational sum from caller-supplied signed product terms.
    ///
    /// The signs must already include every factor sign and term polarity; NoSign
    /// terms are ignored. Checked totals share a native denominator, then their
    /// difference is reduced by the word-result constructor. None is a native
    /// schedule decline, including intermediate overflow despite a small final
    /// answer. Construction can allocate; no approximate result is substituted.
    fn signed_product_sum_words<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Self> {
        let (positive, negative, common_denominator) =
            Self::signed_product_sum_word_totals(terms, signs)?;
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            common_denominator,
        ))
    }

    /// Accumulate separate nonnegative word totals over one positive common denominator.
    ///
    /// The supplied signs define active terms and must include factor signs. Ignore
    /// NoSign terms without inspecting their factors, form unsigned product pairs,
    /// then build their denominator LCM and align each magnitude. Positive and
    /// negative totals are checked separately, so cancellation does not rescue an
    /// overflowing subtotal. Empty/all-inactive input returns (0,0,1).
    ///
    /// None reports conversion or intermediate native overflow, not a mathematical
    /// no-solution or total-resource outcome. Stored positive denominators are
    /// required; arrays use stack storage proportional to TERMS.
    fn signed_product_sum_word_totals<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<(u128, u128, u128)> {
        let mut magnitudes = [0_u128; TERMS];
        let mut denominators = [1_u128; TERMS];
        let mut common_denominator = None::<u128>;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let (magnitude, denominator) = Self::product_term_words(terms[i])?;
            magnitudes[i] = magnitude;
            denominators[i] = denominator;
            common_denominator = Some(match common_denominator {
                None => denominator,
                Some(common) if common == denominator => common,
                Some(common) => {
                    let divisor = Self::gcd_word(common, denominator);
                    common.checked_mul(denominator / divisor)?
                }
            });
        }

        let common_denominator = common_denominator.unwrap_or(1);

        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = magnitudes[i]
                .checked_mul(common_denominator / denominators[i])?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some((positive, negative, common_denominator))
    }

    /// Attempt the largest-magnitude axis and sign of (b-a) cross (c-a).
    ///
    /// Read stored rational parts into words, reduce coordinate differences, compute
    /// three signed determinant components and compare their reduced magnitudes.
    /// Exact ties retain the lowest axis index (x, then y, then z). A zero cross
    /// product returns None, as does any input/intermediate word ineligibility, so
    /// None does not itself prove collinearity. Success gives an exact component
    /// sign, not a unit normal or normalized vector.
    ///
    /// This path does not canonicalize caches or allocate Rational temporaries.
    /// It trusts positive stored denominators and consistent signs; differently
    /// stored equal rationals can have different native eligibility.
    pub(crate) fn dominant_affine_cross_axis_words(
        a: [&Self; 3],
        b: [&Self; 3],
        c: [&Self; 3],
    ) -> Option<(usize, Sign)> {
        /// Stored signed rational parts for the local checked determinant calculation.
        ///
        /// The tuple is (sign, unsigned magnitude, positive denominator). Raw input
        /// pairs need not be coprime; difference/reduction helpers establish reduced
        /// nonzero outputs, and canonical zero is (NoSign,0,1).
        type Word = (Sign, u128, u128);

        /// Convert three stored coordinate pairs to native parts without canonicalization.
        ///
        /// Preserve signs and denominators verbatim; any magnitude/denominator wider
        /// than u128 returns None. Rational invariants supply consistent signs and
        /// positive denominators.
        fn words(values: [&Rational; 3]) -> Option<[Word; 3]> {
            let word = |value: &Rational| {
                Some((
                    value.sign,
                    value.numerator.to_u128()?,
                    value.denominator.to_u128()?,
                ))
            };
            Some([word(values[0])?, word(values[1])?, word(values[2])?])
        }

        /// Compute a reduced signed difference of two native rational triples.
        ///
        /// Align by a checked denominator LCM, subtract through signed_word_sum and
        /// reduce the nonzero result with the verified fraction kernel. Exact zero
        /// uses (NoSign,0,1). Native alignment or signed-sum overflow returns None;
        /// positive denominators and consistent input signs are assumed.
        fn difference(left: Word, right: Word) -> Option<Word> {
            let (left_scale, right_scale, denominator) = if left.2 == right.2 {
                (1, 1, left.2)
            } else {
                let divisor = Rational::gcd_word(left.2, right.2);
                let left_scale = right.2 / divisor;
                (left_scale, left.2 / divisor, left.2.checked_mul(left_scale)?)
            };
            let left = (left.0, left.1.checked_mul(left_scale)?);
            let right = (-right.0, right.1.checked_mul(right_scale)?);
            let (sign, magnitude) = Rational::signed_word_sum(left, right)?;
            if sign == NoSign {
                return Some((NoSign, 0, 1));
            }
            let (magnitude, denominator) = crate::verified::fraction::reduce(magnitude, denominator);
            Some((sign, magnitude, denominator))
        }

        /// Form separate positive/negative totals for one cross-product component.
        ///
        /// Compute `ab[first]*ac[second] - ab[second]*ac[first]`, align the two unsigned
        /// product pairs and retain their common denominator without final reduction.
        /// Indices must be below three or array access panics. Product, alignment or
        /// subtotal overflow returns None even when the final difference is small.
        fn component(ab: [Word; 3], ac: [Word; 3], first: usize, second: usize) -> Option<(u128, u128, u128)> {
            let products = [
                Rational::word_product(ab[first], ac[second], true)?,
                Rational::word_product(ab[second], ac[first], false)?,
            ];
            let common_denominator = if products[0].2 == products[1].2 {
                products[0].2
            } else {
                let divisor = Rational::gcd_word(products[0].2, products[1].2);
                products[0]
                    .2
                    .checked_mul(products[1].2 / divisor)?
            };
            let mut positive = 0_u128;
            let mut negative = 0_u128;
            for (sign, magnitude, denominator) in products {
                let magnitude = magnitude.checked_mul(common_denominator / denominator)?;
                match sign {
                    Plus => positive = positive.checked_add(magnitude)?,
                    Minus => negative = negative.checked_add(magnitude)?,
                    NoSign => {},
                }
            }
            Some((positive, negative, common_denominator))
        }

        /// Reduce the absolute difference of two word totals and retain its sign.
        ///
        /// A positive denominator is required. Equal totals produce (0,1,NoSign);
        /// otherwise the verified fraction reducer returns coprime magnitude and
        /// denominator followed by the original difference sign.
        fn reduced_magnitude(parts: (u128, u128, u128)) -> (u128, u128, Sign) {
            let (positive, negative, denominator) = parts;
            let (magnitude, sign) = match positive.cmp(&negative) {
                Ordering::Greater => (positive - negative, Plus),
                Ordering::Less => (negative - positive, Minus),
                Ordering::Equal => return (0, 1, NoSign),
            };
            let (magnitude, denominator) = crate::verified::fraction::reduce(magnitude, denominator);
            (magnitude, denominator, sign)
        }

        /// Compare two nonnegative native rational magnitudes exactly when alignment fits.
        ///
        /// Equal denominators compare numerators directly. Otherwise divide denominator
        /// scales by their GCD before checked cross-products; overflow returns None,
        /// not an approximate order. Denominators must be positive. No reduction or
        /// heap allocation is performed.
        fn compare_magnitudes(left: (u128, u128), right: (u128, u128)) -> Option<Ordering> {
            if left.1 == right.1 {
                return Some(left.0.cmp(&right.0));
            }
            let divisor = Rational::gcd_word(left.1, right.1);
            Some(
                left.0
                    .checked_mul(right.1 / divisor)?
                    .cmp(&right.0.checked_mul(left.1 / divisor)?),
            )
        }

        let (a, b, c) = (words(a)?, words(b)?, words(c)?);
        let ab = [
            difference(b[0], a[0])?,
            difference(b[1], a[1])?,
            difference(b[2], a[2])?,
        ];
        let ac = [
            difference(c[0], a[0])?,
            difference(c[1], a[1])?,
            difference(c[2], a[2])?,
        ];
        let components = [
            reduced_magnitude(component(ab, ac, 1, 2)?),
            reduced_magnitude(component(ab, ac, 2, 0)?),
            reduced_magnitude(component(ab, ac, 0, 1)?),
        ];
        let mut axis = 0;
        for candidate in 1..3 {
            if compare_magnitudes(
                (components[candidate].0, components[candidate].1),
                (components[axis].0, components[axis].1),
            )? == Ordering::Greater
            {
                axis = candidate;
            }
        }
        (components[axis].2 != NoSign).then_some((axis, components[axis].2))
    }

    /// Attempt the sum of two signed rational product triples in native words.
    ///
    /// NoSign terms are skipped before denominator access; two inactive terms
    /// return zero. Active terms require positive denominators and consistent
    /// magnitudes. Build a checked LCM, align and separately add positive/negative
    /// totals, then reduce their difference. None means intermediate native
    /// ineligibility; an exact result may still fit after cancellation.
    fn two_product_word_sum(
        first: (Sign, u128, u128),
        second: (Sign, u128, u128),
    ) -> Option<Self> {
        let common_denominator = match (first.0, second.0) {
            (NoSign, NoSign) => return Some(Self::zero()),
            (NoSign, _) => second.2,
            (_, NoSign) => first.2,
            _ if first.2 == second.2 => first.2,
            _ => {
                let divisor = Self::gcd_word(first.2, second.2);
                first.2.checked_mul(second.2 / divisor)?
            }
        };
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for (sign, magnitude, denominator) in [first, second] {
            if sign == NoSign {
                continue;
            }
            let magnitude = magnitude.checked_mul(common_denominator / denominator)?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            common_denominator,
        ))
    }

    /// Multiply two unsigned rational word pairs and combine signs with a term polarity.
    ///
    /// A false polarity negates the product. NoSign short-circuits to (NoSign,0,1)
    /// before multiplication or denominator validation. Otherwise numerator and
    /// denominator products are checked independently, without cross-cancellation;
    /// overflow returns None. The result need not be reduced, and valid active
    /// inputs require positive denominators and consistent signs.
    fn word_product(
        left: (Sign, u128, u128),
        right: (Sign, u128, u128),
        positive: bool,
    ) -> Option<(Sign, u128, u128)> {
        let sign = (if positive { Plus } else { Minus }) * left.0 * right.0;
        if sign == NoSign {
            return Some((NoSign, 0, 1));
        }
        Some((
            sign,
            left.1.checked_mul(right.1)?,
            left.2.checked_mul(right.2)?,
        ))
    }

    /// Add two signed native magnitudes without a denominator or allocation.
    ///
    /// NoSign passes the other pair through verbatim, so callers must keep inactive
    /// payloads consistent. Active pairs use the verified signed-add kernel: zero
    /// is canonicalized to NoSign, and equal-sign overflow returns None. Opposite
    /// signs use an exact magnitude difference fitting the word.
    fn signed_word_sum(first: (Sign, u128), second: (Sign, u128)) -> Option<(Sign, u128)> {
        match (first.0, second.0) {
            (NoSign, _) => Some(second),
            (_, NoSign) => Some(first),
            _ => {
                let (negative, magnitude) = crate::verified::word::signed_add(
                    first.0 == Minus,
                    first.1,
                    second.0 == Minus,
                    second.1,
                )?;
                let sign = if magnitude == 0 {
                    NoSign
                } else if negative {
                    Minus
                } else {
                    Plus
                };
                Some((sign, magnitude))
            }
        }
    }

    /// Attempt exact multiplication by 2^shift through the verified word kernel.
    ///
    /// Return None for shift >= 128, including value zero, or for magnitude overflow.
    /// A smaller fitting shift succeeds without allocation. This is a native-width
    /// eligibility check, not a total work bound or a saturating operation.
    #[inline]
    fn checked_word_shift_left(value: u128, shift: u32) -> Option<u128> {
        crate::verified::word::checked_shift_left(value, shift)
    }

    /// Convert a dyadic word triple to an integer numerator at a common binary scale.
    ///
    /// Requires a positive power-of-two denominator and common_shift at least its
    /// exponent. Multiply the numerator by the remaining power of two and return
    /// (sign, aligned_magnitude, 1). The discarded common denominator belongs to
    /// the caller's shared scale; this triple alone is not the original fraction.
    ///
    /// A smaller common shift or checked-shift ineligibility returns None, even for
    /// zero at a shift of at least 128. The helper only counts denominator trailing
    /// zeros; a non-power-of-two denominator violates an unchecked precondition.
    fn scaled_dyadic_word_part(
        part: (Sign, u128, u128),
        common_shift: u32,
    ) -> Option<(Sign, u128, u128)> {
        let shift = common_shift.checked_sub(part.2.trailing_zeros())?;
        Some((
            part.0,
            Self::checked_word_shift_left(part.1, shift)?,
            1,
        ))
    }

    /// Attempt sign*magnitude/denominator times 2^scale_shift as a reduced word ratio.
    ///
    /// Requires a positive denominator. NoSign or zero magnitude returns zero before
    /// validating that precondition. Otherwise cancel the magnitude GCD and delegate
    /// power-of-two cancellation and checked scaling. None is native-result
    /// ineligibility, not mathematical failure or resource exhaustion. Successful
    /// Rational construction can allocate; invalid nonzero calls can panic.
    fn from_scaled_dyadic_quotient_component(
        sign: Sign,
        magnitude: u128,
        denominator: u128,
        scale_shift: i32,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }

        let divisor = Self::gcd_word(magnitude, denominator);
        Self::from_scaled_dyadic_quotient_component_with_divisor(
            sign,
            magnitude,
            denominator,
            scale_shift,
            divisor,
        )
    }

    /// Construct a scaled word quotient using a caller-supplied complete magnitude GCD.
    ///
    /// For nonzero output, denominator must be positive and divisor must equal
    /// GCD(magnitude,denominator). Debug assertions check nonzero divisor and exact
    /// divisibility, not that it is the greatest common divisor. Divide both parts,
    /// cancel the binary scale against the opposite part, and require the remaining
    /// shift/products to fit u128 before marking the result reduced. A smaller
    /// common divisor can violate that reduced-state invariant.
    ///
    /// NoSign or zero magnitude returns zero before these checks. None means native
    /// scale/width ineligibility; invalid input can panic rather than return a typed
    /// error. The i32 minimum exponent is handled through its unsigned magnitude.
    fn from_scaled_dyadic_quotient_component_with_divisor(
        sign: Sign,
        mut magnitude: u128,
        mut denominator: u128,
        scale_shift: i32,
        divisor: u128,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }
        debug_assert_ne!(divisor, 0);
        debug_assert_eq!(magnitude % divisor, 0);
        debug_assert_eq!(denominator % divisor, 0);
        magnitude /= divisor;
        denominator /= divisor;
        if scale_shift >= 0 {
            let shift = scale_shift as u32;
            let cancel = shift.min(denominator.trailing_zeros());
            denominator >>= cancel;
            magnitude = Self::checked_word_shift_left(magnitude, shift - cancel)?;
        } else {
            let shift = scale_shift.unsigned_abs();
            let cancel = shift.min(magnitude.trailing_zeros());
            magnitude >>= cancel;
            denominator = Self::checked_word_shift_left(denominator, shift - cancel)?;
        }
        Some(Self::from_reduced_word_parts(
            sign,
            magnitude,
            denominator,
        ))
    }

    /// Attempt a binary-scaled quotient while retaining remaining common factors.
    ///
    /// Requires a positive denominator. Zero sign or magnitude returns zero first;
    /// otherwise cancel only powers of two against the requested scale and require
    /// both stored parts to fit u128. Retain the exact result through the internally
    /// unreduced constructor, allowing later observation to perform general reduction.
    /// None can occur even when cancelling a common odd factor would make the final
    /// reduced result fit. Invalid input can panic; there is no typed resource budget.
    fn from_scaled_dyadic_quotient_component_unreduced(
        sign: Sign,
        mut magnitude: u128,
        mut denominator: u128,
        scale_shift: i32,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }
        if scale_shift >= 0 {
            let shift = scale_shift as u32;
            let cancel = shift.min(denominator.trailing_zeros());
            denominator >>= cancel;
            magnitude = Self::checked_word_shift_left(magnitude, shift - cancel)?;
        } else {
            let shift = scale_shift.unsigned_abs();
            let cancel = shift.min(magnitude.trailing_zeros());
            magnitude >>= cancel;
            denominator = Self::checked_word_shift_left(denominator, shift - cancel)?;
        }
        Some(Self::from_parts_raw_unreduced(
            sign,
            BigUint::from(magnitude),
            BigUint::from(denominator),
        ))
    }

    /// Attempt a native quotient of signed dyadics and return its magnitude GCD.
    ///
    /// Reject a zero denominator sign or magnitude with None. Compute the magnitude
    /// GCD, require denominator_shift minus numerator_shift to fit i32, then use the
    /// scaled reduced-word constructor. The returned divisor concerns the stored
    /// magnitudes, before exponent adjustment. Other input signs/magnitudes must be
    /// consistent; zero numerators still pass the exponent-conversion gate.
    ///
    /// None conflates zero divisor with shift/result ineligibility; callers decide
    /// whether to report an error or fall back. Successful calls record the native
    /// GCD when tracing is enabled, which can allocate temporary BigUint records in
    /// addition to result storage. There is no arbitrary-precision fallback here.
    fn quotient_dyadic_words(
        numerator: DyadicWord,
        denominator: DyadicWord,
    ) -> Option<(Self, u128)> {
        if denominator.sign == NoSign || denominator.magnitude == 0 {
            return None;
        }
        let divisor = Self::gcd_word(numerator.magnitude, denominator.magnitude);
        let scale_shift = i32::try_from(
            i128::from(denominator.denominator_shift)
                - i128::from(numerator.denominator_shift),
        )
        .ok()?;
        let result = Self::from_scaled_dyadic_quotient_component_with_divisor(
            numerator.sign * denominator.sign,
            numerator.magnitude,
            denominator.magnitude,
            scale_shift,
            divisor,
        )?;
        #[cfg(feature = "dispatch-trace")]
        let numerator_trace = BigUint::from(numerator.magnitude);
        #[cfg(feature = "dispatch-trace")]
        let denominator_trace = BigUint::from(denominator.magnitude);
        #[cfg(feature = "dispatch-trace")]
        let divisor_trace = BigUint::from(divisor);
        trace_rational_gcd!(&numerator_trace, &denominator_trace, &divisor_trace);
        Some((result, divisor))
    }

    /// Attempt exact (a+ib)/(c+id) from four dyadic rational word triples.
    ///
    /// Require power-of-two denominators for all four stored parts, including zeros.
    /// Align each complex pair to one binary scale, form ac+bd, bc-ad and c*c+d*d
    /// with checked native arithmetic, then restore the right/left scale ratio.
    /// The component constructors reduce exact quotients independently.
    ///
    /// None means a nondyadic denominator or intermediate/result tier ineligibility.
    /// Some(Err(DivideByZero)) means the completed native norm is zero; eligibility
    /// checks can decline before reaching that diagnosis. Some(Ok(...)) is exact
    /// success and emits the paired dyadic trace. Consistent signs/magnitudes are
    /// caller invariants; bounded word work does not eliminate result allocation.
    fn complex_dyadic_quotient_words(
        parts: [(Sign, u128, u128); 4],
    ) -> Option<Result<(Self, Self), crate::Problem>> {
        if !parts.iter().all(|part| part.2.is_power_of_two()) {
            return None;
        }
        let left_shift = parts[0]
            .2
            .trailing_zeros()
            .max(parts[1].2.trailing_zeros());
        let right_shift = parts[2]
            .2
            .trailing_zeros()
            .max(parts[3].2.trailing_zeros());
        let a = Self::scaled_dyadic_word_part(parts[0], left_shift)?;
        let b = Self::scaled_dyadic_word_part(parts[1], left_shift)?;
        let c = Self::scaled_dyadic_word_part(parts[2], right_shift)?;
        let d = Self::scaled_dyadic_word_part(parts[3], right_shift)?;

        let ac = Self::word_product(a, c, true)?;
        let bd = Self::word_product(b, d, true)?;
        let bc = Self::word_product(b, c, true)?;
        let ad = Self::word_product(a, d, false)?;
        let cc = Self::word_product(c, c, true)?;
        let dd = Self::word_product(d, d, true)?;
        let re = Self::signed_word_sum((ac.0, ac.1), (bd.0, bd.1))?;
        let im = Self::signed_word_sum((bc.0, bc.1), (ad.0, ad.1))?;
        let norm = Self::signed_word_sum((cc.0, cc.1), (dd.0, dd.1))?;
        if norm.0 == NoSign {
            return Some(Err(crate::Problem::DivideByZero));
        }
        let scale_shift = i32::try_from(right_shift).ok()? - i32::try_from(left_shift).ok()?;
        let re = Self::from_scaled_dyadic_quotient_component(
            re.0,
            re.1,
            norm.1,
            scale_shift,
        )?;
        let im = Self::from_scaled_dyadic_quotient_component(
            im.0,
            im.1,
            norm.1,
            scale_shift,
        )?;
        crate::trace_dispatch!("rational", "complex-quotient", "paired-dyadic-word-sized");
        Some(Ok((re, im)))
    }

    /// Attempt sign*magnitude/norm times scale_numerator/scale_denominator exactly.
    ///
    /// Nonzero output requires positive norm and positive scale parts. Reduce the
    /// component and scale ratios, cross-cancel their opposing factors, then require
    /// final numerator and denominator products to fit u128. The equal-scale case
    /// uses unit scale directly. NoSign or zero magnitude returns zero before
    /// checking any denominator or scale invariant.
    ///
    /// None means final native product overflow. Invalid private inputs can panic
    /// or pass a shortcut without validation; this is not a typed domain checker.
    /// Reduced result construction may allocate despite native cancellation.
    fn from_scaled_word_quotient_component(
        sign: Sign,
        mut magnitude: u128,
        mut norm: u128,
        mut scale_numerator: u128,
        mut scale_denominator: u128,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }

        (magnitude, norm) = crate::verified::fraction::reduce(magnitude, norm);
        if scale_numerator == scale_denominator {
            scale_numerator = 1;
            scale_denominator = 1;
        } else {
            (scale_numerator, scale_denominator) =
                crate::verified::fraction::reduce(scale_numerator, scale_denominator);
        }
        if scale_denominator != 1 {
            (magnitude, scale_denominator) = crate::verified::fraction::reduce(magnitude, scale_denominator);
        }
        if scale_numerator != 1 {
            (scale_numerator, norm) = crate::verified::fraction::reduce(scale_numerator, norm);
        }

        Some(Self::from_reduced_word_parts(
            sign,
            magnitude.checked_mul(scale_numerator)?,
            norm.checked_mul(scale_denominator)?,
        ))
    }

    /// Attempt exact complex division using a common native denominator for each pair.
    ///
    /// Parts are ordered a,b,c,d for (a+ib)/(c+id), with positive denominators and
    /// consistent signs. Build pairwise denominator LCMs, align numerators, form the
    /// conjugate-product components and squared norm, then restore the right/left
    /// LCM ratio through cross-cancellation. Intermediate LCMs, products and sums
    /// must fit u128 even when the eventual quotient is small.
    ///
    /// None requests a wider exact schedule; Some(Err(DivideByZero)) diagnoses a norm
    /// that reached the zero test. Only exact success emits the general-word trace.
    /// Unlike the dyadic recognizer, this helper trusts positive denominators and can
    /// panic on invalid triples. Result storage can allocate without a total budget.
    fn complex_word_quotient_words(
        parts: [(Sign, u128, u128); 4],
    ) -> Option<Result<(Self, Self), crate::Problem>> {
        let left_denominator = if parts[0].2 == parts[1].2 {
            parts[0].2
        } else {
            let divisor = Self::gcd_word(parts[0].2, parts[1].2);
            parts[0].2.checked_mul(parts[1].2 / divisor)?
        };
        let right_denominator = if parts[2].2 == parts[3].2 {
            parts[2].2
        } else {
            let divisor = Self::gcd_word(parts[2].2, parts[3].2);
            parts[2].2.checked_mul(parts[3].2 / divisor)?
        };
        let a = (
            parts[0].0,
            parts[0].1.checked_mul(left_denominator / parts[0].2)?,
            1,
        );
        let b = (
            parts[1].0,
            parts[1].1.checked_mul(left_denominator / parts[1].2)?,
            1,
        );
        let c = (
            parts[2].0,
            parts[2].1.checked_mul(right_denominator / parts[2].2)?,
            1,
        );
        let d = (
            parts[3].0,
            parts[3].1.checked_mul(right_denominator / parts[3].2)?,
            1,
        );

        let ac = Self::word_product(a, c, true)?;
        let bd = Self::word_product(b, d, true)?;
        let bc = Self::word_product(b, c, true)?;
        let ad = Self::word_product(a, d, false)?;
        let cc = Self::word_product(c, c, true)?;
        let dd = Self::word_product(d, d, true)?;
        let re = Self::signed_word_sum((ac.0, ac.1), (bd.0, bd.1))?;
        let im = Self::signed_word_sum((bc.0, bc.1), (ad.0, ad.1))?;
        let norm = Self::signed_word_sum((cc.0, cc.1), (dd.0, dd.1))?;
        if norm.0 == NoSign {
            return Some(Err(crate::Problem::DivideByZero));
        }
        let re = Self::from_scaled_word_quotient_component(
            re.0,
            re.1,
            norm.1,
            right_denominator,
            left_denominator,
        )?;
        let im = Self::from_scaled_word_quotient_component(
            im.0,
            im.1,
            norm.1,
            right_denominator,
            left_denominator,
        )?;
        crate::trace_dispatch!("rational", "complex-quotient", "paired-general-word-sized");
        Some(Ok((re, im)))
    }

    /// Multiply rational complex pairs, optionally conjugating the right operand.
    ///
    /// For left (a,b) and right (c,d), ordinary output is (ac-bd,ad+bc); conjugated
    /// output is (ac+bd,bc-ad). Scan stored word parts once, compute four checked
    /// products and the two sums, then fall back to exact signed-product reducers
    /// if either component is ineligible. A previously constructed native component
    /// can be discarded when its partner declines.
    ///
    /// Stored unreduced inputs retain their exact values; canonicalization follows
    /// the selected arithmetic path. Trace labels distinguish conjugation and native
    /// versus general scheduling. Wide arithmetic and result allocation are unbudgeted.
    fn complex_product_components_impl(
        left: [&Self; 2],
        right: [&Self; 2],
        conjugate_right: bool,
    ) -> (Self, Self) {
        let word_parts = [left[0], left[1], right[0], right[1]].map(|value| {
            Some((
                value.sign,
                value.numerator.to_u128()?,
                value.denominator.to_u128()?,
            ))
        });
        if let [Some(a), Some(b), Some(c), Some(d)] = word_parts
            && let (Some(ac), Some(bd), Some(ad), Some(bc)) = (
                Self::word_product(a, c, true),
                Self::word_product(b, d, conjugate_right),
                Self::word_product(a, d, !conjugate_right),
                Self::word_product(b, c, true),
            )
            && let (Some(re), Some(im)) = (
                Self::two_product_word_sum(ac, bd),
                Self::two_product_word_sum(ad, bc),
            )
        {
            crate::trace_dispatch!(
                "rational",
                "complex-product",
                if conjugate_right {
                    "paired-conjugate-word-sized"
                } else {
                    "paired-word-sized"
                }
            );
            return (re, im);
        }

        crate::trace_dispatch!(
            "rational",
            "complex-product",
            if conjugate_right {
                "paired-conjugate-general-fallback"
            } else {
                "paired-general-fallback"
            }
        );
        if conjugate_right {
            (
                Self::signed_product_sum2(
                    [true, true],
                    [[left[0], right[0]], [left[1], right[1]]],
                ),
                Self::signed_product_sum2(
                    [false, true],
                    [[left[0], right[1]], [left[1], right[0]]],
                ),
            )
        } else {
            (
                Self::signed_product_sum2(
                    [true, false],
                    [[left[0], right[0]], [left[1], right[1]]],
                ),
                Self::signed_product_sum2(
                    [true, true],
                    [[left[0], right[1]], [left[1], right[0]]],
                ),
            )
        }
    }

    /// Multiply exact rational complex pairs as (ac-bd,ad+bc).
    ///
    /// Inputs are left (a,b) and right (c,d), including valid internally unreduced
    /// rationals. The shared word scan tries a paired native schedule; any
    /// ineligibility falls back to exact general product sums without approximation.
    /// Outputs are exact rational components, not a complex wrapper. Intermediate
    /// work and result allocation have no total resource/cancellation contract.
    pub fn complex_product_components(left: [&Self; 2], right: [&Self; 2]) -> (Self, Self) {
        Self::complex_product_components_impl(left, right, false)
    }

    /// Divide exact rational complex pairs, rejecting a zero complex divisor.
    ///
    /// For (a+ib)/(c+id), return ((ac+bd)/(c*c+d*d),(bc-ad)/(c*c+d*d)). Try paired
    /// dyadic words, then general words, then the exact conjugate-product and shared
    /// norm fallback. Native None means continue; a native error is returned directly.
    /// A zero divisor returns Problem::DivideByZero even for a zero dividend, with
    /// no floating comparison or epsilon test.
    ///
    /// Inputs may have valid internally unreduced storage. The wide fallback checks
    /// the norm after constructing its numerators, so this is not an early validation
    /// or total-work bound. Canonicalization and temporary allocation follow the
    /// selected exact path; no precision policy or cancellation budget is accepted.
    pub fn complex_quotient_components(
        left: [&Self; 2],
        right: [&Self; 2],
    ) -> Result<(Self, Self), crate::Problem> {
        let word_parts = [left[0], left[1], right[0], right[1]].map(|value| {
            Some((
                value.sign,
                value.numerator.to_u128()?,
                value.denominator.to_u128()?,
            ))
        });
        if let [Some(a), Some(b), Some(c), Some(d)] = word_parts
        {
            if let Some(result) = Self::complex_dyadic_quotient_words([a, b, c, d]) {
                return result;
            }
            if let Some(result) = Self::complex_word_quotient_words([a, b, c, d]) {
                return result;
            }
        }

        let (re_numerator, im_numerator) =
            Self::complex_product_components_impl(left, right, true);
        let denominator =
            Self::signed_product_sum2([true, true], [[right[0], right[0]], [right[1], right[1]]]);
        if denominator.sign == NoSign {
            return Err(crate::Problem::DivideByZero);
        }
        crate::trace_dispatch!("rational", "complex-quotient", "paired-exact-rational");
        Ok((
            &re_numerator / &denominator,
            &im_numerator / &denominator,
        ))
    }

    /// Construct the signed homogeneous cofactor tuple from a caller-proved sparse row.
    ///
    /// Requires sparse_row < 3, sparse_column < 4, and that row's only nonzero entry
    /// at the selected column. For column j the result is (-1)^(j+1) times the 3x3
    /// minor obtained by deleting j. Expand through the sparse coefficient once,
    /// using the cross-product of the other two rows over the remaining columns.
    /// The selected output column is zero; no division by the fourth component occurs.
    ///
    /// Canonicalize the other rows and selected scale. Use the dyadic reducer only
    /// when all six relevant other-row coefficients are dyadic. A zero tuple or a
    /// point at infinity is returned as such, without classifying rank, uniqueness
    /// or affine validity. Invalid indices panic; incorrect sparsity can yield a
    /// tuple that does not annihilate the original matrix because omitted entries
    /// are never checked. Wide work/allocations have no resource budget.
    pub(crate) fn homogeneous_plane_intersection3_sparse(
        matrix: [[&Self; 4]; 3],
        sparse_row: usize,
        sparse_column: usize,
    ) -> [Self; 4] {
        let other_rows: [usize; 2] = match sparse_row {
            0 => [1, 2],
            1 => [0, 2],
            _ => [0, 1],
        };
        let mut columns = [0_usize; 3];
        let mut next = 0;
        for column in 0..4 {
            if column != sparse_column {
                columns[next] = column;
                next += 1;
            }
        }
        let [first, second, third] = columns;
        let left = matrix[other_rows[0]].map(Self::canonicalized_ref);
        let right = matrix[other_rows[1]].map(Self::canonicalized_ref);
        let scale = matrix[sparse_row][sparse_column].canonicalized_ref();
        let positive_terms = if (sparse_row + sparse_column) & 1 == 0 {
            [true, false]
        } else {
            [false, true]
        };
        let dyadic = columns.into_iter().all(|column| {
            left[column].dyadic_denominator_shift().is_some()
                && right[column].dyadic_denominator_shift().is_some()
        });
        let difference = |terms| {
            if dyadic {
                Self::signed_product_sum_known_dyadic(positive_terms, terms)
            } else {
                let positive = terms[0][0] * terms[0][1];
                let negative = terms[1][0] * terms[1][1];
                if positive_terms[0] {
                    &positive - &negative
                } else {
                    &negative - &positive
                }
            }
        };
        let cross = [
            difference([[left[second], right[third]], [left[third], right[second]]]),
            difference([[left[third], right[first]], [left[first], right[third]]]),
            difference([[left[first], right[second]], [left[second], right[first]]]),
        ];
        let mut result = core::array::from_fn(|_| Self::zero());
        for (column, value) in columns.into_iter().zip(cross) {
            result[column] = &value * scale;
        }
        crate::trace_dispatch!(
            "rational",
            "homogeneous-plane-intersection3",
            "sparse-exact-row-scaled-cofactors"
        );
        result
    }

    /// Invert a 3x3 rational matrix using an exact adjugate and shared reciprocal.
    ///
    /// The general aggregate handles valid reduced or internally unreduced entries.
    /// A zero determinant returns Problem::DivideByZero. Cofactor products remain
    /// exact rationals; no Real wrappers, approximate decisions or pivot tolerances
    /// are introduced. Fixed dimensions do not bound coefficient growth, allocation
    /// or total work; no cancellation input is accepted.
    pub(crate) fn matrix3_inverse_components(
        matrix: [[&Self; 3]; 3],
    ) -> Result<[[Self; 3]; 3], crate::Problem> {
        Self::matrix3_inverse_components_impl::<false>(matrix)
    }

    /// Invert a 3x3 matrix using caller-established dyadic product-sum schedules.
    ///
    /// Every entry must be an exact dyadic, including valid internally unreduced
    /// representations. The shared implementation preserves adjugate/determinant
    /// semantics and returns Problem::DivideByZero for singularity. Nondyadic factors
    /// reached by the unchecked dyadic reducer can panic; this is not a classifier.
    /// Intermediate exact arithmetic and output storage have no total resource bound.
    pub(crate) fn matrix3_inverse_components_known_dyadic(
        matrix: [[&Self; 3]; 3],
    ) -> Result<[[Self; 3]; 3], crate::Problem> {
        Self::matrix3_inverse_components_impl::<true>(matrix)
    }

    /// Evaluate the 3x3 adjugate, determinant and one exact reciprocal in fixed order.
    ///
    /// KNOWN_DYADIC selects the caller-proved dyadic product-sum schedule; otherwise
    /// use general signed products. Cofactors are already arranged in transposed
    /// adjugate order. Expand the determinant along the first matrix row, invert it
    /// once, then scale all nine entries. Singular inversion returns DivideByZero
    /// before the aggregate success trace. Dyadic preconditions are not validated
    /// up front; invalid classifications can panic. Work and memory are unbudgeted.
    fn matrix3_inverse_components_impl<const KNOWN_DYADIC: bool>(
        matrix: [[&Self; 3]; 3],
    ) -> Result<[[Self; 3]; 3], crate::Problem> {
        let m = matrix;
        let difference = |terms| {
            if KNOWN_DYADIC {
                Self::signed_product_sum_known_dyadic([true, false], terms)
            } else {
                Self::signed_product_sum2([true, false], terms)
            }
        };
        let c00 = difference([[m[1][1], m[2][2]], [m[1][2], m[2][1]]]);
        let c01 = difference([[m[0][2], m[2][1]], [m[0][1], m[2][2]]]);
        let c02 = difference([[m[0][1], m[1][2]], [m[0][2], m[1][1]]]);
        let c10 = difference([[m[1][2], m[2][0]], [m[1][0], m[2][2]]]);
        let c11 = difference([[m[0][0], m[2][2]], [m[0][2], m[2][0]]]);
        let c12 = difference([[m[0][2], m[1][0]], [m[0][0], m[1][2]]]);
        let c20 = difference([[m[1][0], m[2][1]], [m[1][1], m[2][0]]]);
        let c21 = difference([[m[0][1], m[2][0]], [m[0][0], m[2][1]]]);
        let c22 = difference([[m[0][0], m[1][1]], [m[0][1], m[1][0]]]);
        let determinant_terms = [
            [m[0][0], &c00],
            [m[0][1], &c10],
            [m[0][2], &c20],
        ];
        let determinant = if KNOWN_DYADIC {
            Self::signed_product_sum_known_dyadic([true, true, true], determinant_terms)
        } else {
            Self::signed_product_sum([true, true, true], determinant_terms)
        };
        let inverse_determinant = determinant.inverse()?;
        crate::trace_dispatch!("rational", "matrix3-inverse", "aggregate-cofactor");
        Ok([
            [
                c00 * &inverse_determinant,
                c01 * &inverse_determinant,
                c02 * &inverse_determinant,
            ],
            [
                c10 * &inverse_determinant,
                c11 * &inverse_determinant,
                c12 * &inverse_determinant,
            ],
            [
                c20 * &inverse_determinant,
                c21 * &inverse_determinant,
                c22 * &inverse_determinant,
            ],
        ])
    }

    /// Invert a 4x4 rational matrix through shared 2x2 minors and exact cofactors.
    ///
    /// The general rational schedule accepts valid stored unreduced entries and
    /// returns Problem::DivideByZero for a zero determinant. It reuses one determinant
    /// reciprocal across all sixteen adjugate entries without introducing Real wrappers
    /// or approximate predicates. Arithmetic, caches and output allocation have no
    /// total-work or cancellation budget.
    pub(crate) fn matrix4_inverse_components(
        matrix: [[&Self; 4]; 4],
    ) -> Result<[[Self; 4]; 4], crate::Problem> {
        Self::matrix4_inverse_components_impl(matrix)
    }

    /// Invert a dyadic 4x4 matrix by lifting each row to its smallest integer grid.
    ///
    /// Canonicalize every entry and require a power-of-two denominator. If D contains
    /// the four positive row scales and B=D*A, compute integer minors, determinant
    /// and adjugate of B, then use A^-1=B^-1*D: output column j receives the scale of
    /// input row j. The determinant sign is applied to each numerator while its
    /// positive magnitude is shared as denominator. Final quotients are reduced.
    ///
    /// Zero determinant returns Problem::DivideByZero before the success trace.
    /// A nondyadic entry or a shift not fitting usize panics at an invariant check;
    /// this entry trusts prior classification. BigInt lifting/minors and sixteen
    /// rational outputs can allocate without a work, memory or cancellation budget.
    pub(crate) fn matrix4_inverse_components_known_dyadic(
        matrix: [[&Self; 4]; 4],
    ) -> Result<[[Self; 4]; 4], crate::Problem> {
        // Lift each row onto its smallest integer grid. If B = D A for the
        // diagonal power-of-two scale D, then A^-1 = B^-1 D. This keeps every
        // minor fraction-free. After input canonicalization, intermediate cofactors
        // stay in BigInt until the sixteen final adjugate/determinant quotients.
        let row_shifts: [u64; 4] = matrix.map(|row| {
            row.into_iter()
                .map(|value| {
                    let value = value.canonicalized_ref();
                    value
                        .dyadic_denominator_shift()
                        .expect("known-dyadic matrix received a non-dyadic component")
                })
                .max()
                .expect("matrix rows are nonempty")
        });
        let m: [[BigInt; 4]; 4] = core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                let value = matrix[row][column].canonicalized_ref();
                let denominator_shift = value
                    .dyadic_denominator_shift()
                    .expect("known-dyadic matrix received a non-dyadic component");
                let lift = usize::try_from(row_shifts[row] - denominator_shift)
                    .expect("dyadic matrix row shift fits usize");
                BigInt::from_biguint(value.sign, value.numerator.clone()) << lift
            })
        });
        let difference = |a: &BigInt, b: &BigInt, c: &BigInt, d: &BigInt| a * b - c * d;
        let s = [
            difference(&m[0][0], &m[1][1], &m[1][0], &m[0][1]),
            difference(&m[0][0], &m[1][2], &m[1][0], &m[0][2]),
            difference(&m[0][0], &m[1][3], &m[1][0], &m[0][3]),
            difference(&m[0][1], &m[1][2], &m[1][1], &m[0][2]),
            difference(&m[0][1], &m[1][3], &m[1][1], &m[0][3]),
            difference(&m[0][2], &m[1][3], &m[1][2], &m[0][3]),
        ];
        let c = [
            difference(&m[2][0], &m[3][1], &m[3][0], &m[2][1]),
            difference(&m[2][0], &m[3][2], &m[3][0], &m[2][2]),
            difference(&m[2][0], &m[3][3], &m[3][0], &m[2][3]),
            difference(&m[2][1], &m[3][2], &m[3][1], &m[2][2]),
            difference(&m[2][1], &m[3][3], &m[3][1], &m[2][3]),
            difference(&m[2][2], &m[3][3], &m[3][2], &m[2][3]),
        ];
        let determinant = &s[0] * &c[5] - &s[1] * &c[4]
            + &s[2] * &c[3]
            + &s[3] * &c[2]
            - &s[4] * &c[1]
            + &s[5] * &c[0];
        let (determinant_sign, determinant) = determinant.into_parts();
        if determinant_sign == NoSign {
            return Err(crate::Problem::DivideByZero);
        }
        let cofactor = |positive_terms: [bool; 3], terms: [[&BigInt; 2]; 3], column: usize| {
            let mut value = BigInt::ZERO;
            for index in 0..3 {
                let term = terms[index][0] * terms[index][1];
                if positive_terms[index] {
                    value += term;
                } else {
                    value -= term;
                }
            }
            if determinant_sign == Minus {
                value = -value;
            }
            value <<= usize::try_from(row_shifts[column]).expect("dyadic row shift fits usize");
            Self::from_bigint_fraction(value, determinant.clone())
                .expect("matrix determinant is nonzero")
        };
        crate::trace_dispatch!("rational", "matrix4-inverse", "row-scaled-dyadic-integer");
        Ok([
            [
                cofactor([true, true, false], [[&m[1][1], &c[5]], [&m[1][3], &c[3]], [&m[1][2], &c[4]]], 0),
                cofactor([true, false, false], [[&m[0][2], &c[4]], [&m[0][1], &c[5]], [&m[0][3], &c[3]]], 1),
                cofactor([true, true, false], [[&m[3][1], &s[5]], [&m[3][3], &s[3]], [&m[3][2], &s[4]]], 2),
                cofactor([true, false, false], [[&m[2][2], &s[4]], [&m[2][1], &s[5]], [&m[2][3], &s[3]]], 3),
            ],
            [
                cofactor([true, false, false], [[&m[1][2], &c[2]], [&m[1][0], &c[5]], [&m[1][3], &c[1]]], 0),
                cofactor([true, true, false], [[&m[0][0], &c[5]], [&m[0][3], &c[1]], [&m[0][2], &c[2]]], 1),
                cofactor([true, false, false], [[&m[3][2], &s[2]], [&m[3][0], &s[5]], [&m[3][3], &s[1]]], 2),
                cofactor([true, true, false], [[&m[2][0], &s[5]], [&m[2][3], &s[1]], [&m[2][2], &s[2]]], 3),
            ],
            [
                cofactor([true, true, false], [[&m[1][0], &c[4]], [&m[1][3], &c[0]], [&m[1][1], &c[2]]], 0),
                cofactor([true, false, false], [[&m[0][1], &c[2]], [&m[0][0], &c[4]], [&m[0][3], &c[0]]], 1),
                cofactor([true, true, false], [[&m[3][0], &s[4]], [&m[3][3], &s[0]], [&m[3][1], &s[2]]], 2),
                cofactor([true, false, false], [[&m[2][1], &s[2]], [&m[2][0], &s[4]], [&m[2][3], &s[0]]], 3),
            ],
            [
                cofactor([true, false, false], [[&m[1][1], &c[1]], [&m[1][0], &c[3]], [&m[1][2], &c[0]]], 0),
                cofactor([true, true, false], [[&m[0][0], &c[3]], [&m[0][2], &c[0]], [&m[0][1], &c[1]]], 1),
                cofactor([true, false, false], [[&m[3][1], &s[1]], [&m[3][0], &s[3]], [&m[3][2], &s[0]]], 2),
                cofactor([true, true, false], [[&m[2][0], &s[3]], [&m[2][2], &s[0]], [&m[2][1], &s[1]]], 3),
            ],
        ])
    }

    /// Compute the general 4x4 inverse from two groups of six exact 2x2 minors.
    ///
    /// The groups use the first and last pair of rows. Their fixed six-product
    /// combination yields the determinant; an exact reciprocal scales each signed
    /// three-product cofactor into adjugate order. Singular inversion propagates
    /// DivideByZero before the aggregate success trace and output assembly.
    ///
    /// General signed-product reducers preserve valid unreduced inputs and can
    /// canonicalize or reuse arithmetic state as scheduled. Fixed minor counts do
    /// not constrain integer growth or complete-lifetime memory; no resource or
    /// cancellation receipt is attached.
    fn matrix4_inverse_components_impl(
        matrix: [[&Self; 4]; 4],
    ) -> Result<[[Self; 4]; 4], crate::Problem> {
        let m = matrix;
        let difference = |a: &Self, b: &Self, c: &Self, d: &Self| {
            Self::signed_product_sum2([true, false], [[a, b], [c, d]])
        };
        let s = [
            difference(m[0][0], m[1][1], m[1][0], m[0][1]),
            difference(m[0][0], m[1][2], m[1][0], m[0][2]),
            difference(m[0][0], m[1][3], m[1][0], m[0][3]),
            difference(m[0][1], m[1][2], m[1][1], m[0][2]),
            difference(m[0][1], m[1][3], m[1][1], m[0][3]),
            difference(m[0][2], m[1][3], m[1][2], m[0][3]),
        ];
        let c = [
            difference(m[2][0], m[3][1], m[3][0], m[2][1]),
            difference(m[2][0], m[3][2], m[3][0], m[2][2]),
            difference(m[2][0], m[3][3], m[3][0], m[2][3]),
            difference(m[2][1], m[3][2], m[3][1], m[2][2]),
            difference(m[2][1], m[3][3], m[3][1], m[2][3]),
            difference(m[2][2], m[3][3], m[3][2], m[2][3]),
        ];
        let determinant_terms = [
            [&s[0], &c[5]],
            [&s[1], &c[4]],
            [&s[2], &c[3]],
            [&s[3], &c[2]],
            [&s[4], &c[1]],
            [&s[5], &c[0]],
        ];
        let determinant_signs = [true, false, true, true, false, true];
        let determinant = Self::signed_product_sum(determinant_signs, determinant_terms);
        let scale = determinant.inverse()?;
        let cofactor = |positive_terms: [bool; 3], terms: [[&Self; 2]; 3]| {
            let value = Self::signed_product_sum(positive_terms, terms);
            value * &scale
        };
        crate::trace_dispatch!("rational", "matrix4-inverse", "aggregate-cofactor");
        Ok([
            [
                cofactor([true, true, false], [[m[1][1], &c[5]], [m[1][3], &c[3]], [m[1][2], &c[4]]]),
                cofactor([true, false, false], [[m[0][2], &c[4]], [m[0][1], &c[5]], [m[0][3], &c[3]]]),
                cofactor([true, true, false], [[m[3][1], &s[5]], [m[3][3], &s[3]], [m[3][2], &s[4]]]),
                cofactor([true, false, false], [[m[2][2], &s[4]], [m[2][1], &s[5]], [m[2][3], &s[3]]]),
            ],
            [
                cofactor([true, false, false], [[m[1][2], &c[2]], [m[1][0], &c[5]], [m[1][3], &c[1]]]),
                cofactor([true, true, false], [[m[0][0], &c[5]], [m[0][3], &c[1]], [m[0][2], &c[2]]]),
                cofactor([true, false, false], [[m[3][2], &s[2]], [m[3][0], &s[5]], [m[3][3], &s[1]]]),
                cofactor([true, true, false], [[m[2][0], &s[5]], [m[2][3], &s[1]], [m[2][2], &s[2]]]),
            ],
            [
                cofactor([true, true, false], [[m[1][0], &c[4]], [m[1][3], &c[0]], [m[1][1], &c[2]]]),
                cofactor([true, false, false], [[m[0][1], &c[2]], [m[0][0], &c[4]], [m[0][3], &c[0]]]),
                cofactor([true, true, false], [[m[3][0], &s[4]], [m[3][3], &s[0]], [m[3][1], &s[2]]]),
                cofactor([true, false, false], [[m[2][1], &s[2]], [m[2][0], &s[4]], [m[2][3], &s[0]]]),
            ],
            [
                cofactor([true, false, false], [[m[1][1], &c[1]], [m[1][0], &c[3]], [m[1][2], &c[0]]]),
                cofactor([true, true, false], [[m[0][0], &c[3]], [m[0][2], &c[0]], [m[0][1], &c[1]]]),
                cofactor([true, false, false], [[m[3][1], &s[1]], [m[3][0], &s[3]], [m[3][2], &s[0]]]),
                cofactor([true, true, false], [[m[2][0], &s[3]], [m[2][2], &s[0]], [m[2][1], &s[1]]]),
            ],
        ])
    }

    /// Canonicalize a rational and attempt its exact signed binary-fraction view.
    ///
    /// Accepts zero and reduced numerators fitting u128 with power-of-two
    /// denominators; returns None for a wider numerator or a non-dyadic value.
    /// Despite the name, it checks admission. Lazy reduction and fact learning may
    /// allocate or warm caches; this is not a bounded, read-only metadata cast.
    fn known_dyadic_word(value: &Self) -> Option<DyadicWord> {
        let value = value.canonicalized_ref();
        Some(DyadicWord {
            sign: value.sign,
            magnitude: value.numerator.to_u128()?,
            denominator_shift: value.dyadic_denominator_shift()?,
        })
    }

    /// Attempt exact oriented coefficients (nx, ny, nz, d) through three points.
    ///
    /// The normal is (p1-p0) cross (p2-p0), and d is minus its dot product with
    /// p0, so the equation is nx*x + ny*y + nz*z + d = 0. Collinear or duplicate
    /// points can successfully return four zeros; this does not certify a unique
    /// or nondegenerate plane. The normal is not normalized to unit length.
    ///
    /// Canonicalizes and checks all coordinates for dyadic denominators and u128
    /// numerators. Differences must fit checked native arithmetic, cross products
    /// use six-limb subtotals, and each reduced normal magnitude must narrow to
    /// four limbs before the wide-by-native offset sum. None means an admission,
    /// alignment or intermediate-width failure, even when final coefficients
    /// would fit; callers retain their general exact determinant fallback.
    ///
    /// Scratch arithmetic uses fixed buffers. Input canonicalization and final
    /// Rational materialization can allocate, especially for large denominators;
    /// no total-work, allocation or cancellation budget is enforced here. A
    /// completed compact construction emits the dyadic-stack-fused dispatch event.
    pub fn affine_plane3_coefficients_known_dyadic(
        points: [[&Self; 3]; 3],
    ) -> Option<[Self; 4]> {
        let point_words = |point: [&Self; 3]| {
            Some([
                Self::known_dyadic_word(point[0])?,
                Self::known_dyadic_word(point[1])?,
                Self::known_dyadic_word(point[2])?,
            ])
        };
        let first = point_words(points[0])?;
        let second = point_words(points[1])?;
        let third = point_words(points[2])?;
        let difference = |left: [DyadicWord; 3], right: [DyadicWord; 3]| {
            Some([
                Self::difference_dyadic_words(left[0], right[0])?,
                Self::difference_dyadic_words(left[1], right[1])?,
                Self::difference_dyadic_words(left[2], right[2])?,
            ])
        };
        let u = difference(second, first)?;
        let v = difference(third, first)?;
        let normal = [
            Self::cross_dyadic_words([u[1], u[2]], [v[1], v[2]])?,
            Self::cross_dyadic_words([u[2], u[0]], [v[2], v[0]])?,
            Self::cross_dyadic_words([u[0], u[1]], [v[0], v[1]])?,
        ];
        let normal_wide = [
            Self::dyadic_stack_sum_wide_word(normal[0])?,
            Self::dyadic_stack_sum_wide_word(normal[1])?,
            Self::dyadic_stack_sum_wide_word(normal[2])?,
        ];
        let offset =
            Self::product_sum_wide_narrow_words(normal_wide, first, [false; 3])?;
        crate::trace_dispatch!("rational", "affine-plane3", "dyadic-stack-fused");
        Some([
            Self::materialize_dyadic_stack_sum(normal[0]),
            Self::materialize_dyadic_stack_sum(normal[1]),
            Self::materialize_dyadic_stack_sum(normal[2]),
            Self::materialize_dyadic_stack_sum(offset),
        ])
    }

    /// Attempt left minus right using the shared checked native dyadic kernel.
    ///
    /// Both magnitudes align to the larger denominator exponent before inactive
    /// signs are applied. A shift of at least 128 therefore declines even for a
    /// stored zero. None also covers shifted-magnitude or signed-sum overflow;
    /// later cancellation does not rescue failed alignment. Success cancels
    /// common powers of two and canonicalizes zero without heap arithmetic.
    fn difference_dyadic_words(left: DyadicWord, right: DyadicWord) -> Option<DyadicWord> {
        crate::verified::dyadic::difference_word(left.into_native(), right.into_native())
            .map(DyadicWord::from_native)
    }

    /// Subtract six-limb unsigned subtotals over the common denominator 2^max_shift.
    ///
    /// Uses the shared dyadic difference/reduction kernel and cancels only powers
    /// of two present in both numerator and denominator. Equal subtotals become
    /// canonical NoSign, zero magnitude and exponent zero. Its internal None is
    /// exact equality, not exhaustion; no Rational or BigUint is materialized.
    fn finish_dyadic_stack_sum(
        positive: DyadicStackAccumulator,
        negative: DyadicStackAccumulator,
        max_shift: u64,
    ) -> DyadicStackSum {
        let Some((minus, magnitude, denominator_shift)) =
            crate::verified::dyadic::finish(positive.0, negative.0, max_shift)
        else {
            return DyadicStackSum {
                sign: NoSign,
                magnitude: DyadicStackAccumulator::default(),
                denominator_shift: 0,
            };
        };
        DyadicStackSum {
            sign: if minus { Minus } else { Plus },
            magnitude: DyadicStackAccumulator(magnitude),
            denominator_shift,
        }
    }

    /// Narrow a retained six-limb magnitude to u128 for another native operation.
    ///
    /// None means a nonzero high limb. Sign and denominator exponent are copied
    /// unchanged, including for zero; this adapter neither validates nor restores
    /// the producer's normalization and sign/magnitude invariants.
    fn dyadic_stack_sum_word(sum: DyadicStackSum) -> Option<DyadicWord> {
        Some(DyadicWord {
            sign: sum.sign,
            magnitude: sum.magnitude.to_u128()?,
            denominator_shift: sum.denominator_shift,
        })
    }

    /// Embed a native magnitude in the low two limbs of a six-limb retained sum.
    ///
    /// Zero-fills the remaining limbs and copies sign and denominator exponent.
    /// No arithmetic reduction, invariant validation or heap allocation occurs;
    /// the caller must retain the word producer's normalization proof.
    fn dyadic_word_stack_sum(word: DyadicWord) -> DyadicStackSum {
        let mut magnitude = DyadicStackAccumulator::default();
        magnitude.0[0] = word.magnitude as u64;
        magnitude.0[1] = (word.magnitude >> 64) as u64;
        DyadicStackSum {
            sign: word.sign,
            magnitude,
            denominator_shift: word.denominator_shift,
        }
    }

    /// Narrow a retained sum to the four-limb factor used by wide-by-native products.
    ///
    /// None means one of the two omitted high limbs is nonzero. Copies sign and
    /// denominator exponent without normalization or validation, preserving the
    /// existing producer proof rather than independently establishing one.
    fn dyadic_stack_sum_wide_word(sum: DyadicStackSum) -> Option<DyadicWideWord> {
        Some(DyadicWideWord {
            sign: sum.sign,
            magnitude: crate::verified::limbs::resize(&sum.magnitude.0)?,
            denominator_shift: sum.denominator_shift,
        })
    }

    /// Attempt a signed sum of N native dyadic products with six-limb subtotals.
    ///
    /// A true polarity adds the signed product; false subtracts it. Delegates
    /// descriptor construction and checked planning to the shared stack reducer,
    /// including inactive-term exponents. None is schedule ineligibility, not a
    /// zero result; an empty sum succeeds as canonical zero. No heap arithmetic
    /// is used, though the const-sized descriptor array grows with N.
    fn product_sum_dyadic_words<const N: usize>(
        left: [DyadicWord; N],
        right: [DyadicWord; N],
        positive_terms: [bool; N],
    ) -> Option<DyadicStackSum> {
        Self::product_sum_stack(Self::dyadic_word_products(left, right, positive_terms))
    }

    /// Describe signed dyadic products for the shared verified aggregate kernels.
    ///
    /// Multiplies polarity and both stored signs to set activity and negativity,
    /// while copying magnitudes and denominator exponents even for inactive terms.
    /// Performs no multiplication, normalization or exponent-overflow check here.
    /// Consumers plan all copied exponents before skipping inactive arithmetic.
    fn dyadic_word_products<const N: usize>(
        left: [DyadicWord; N],
        right: [DyadicWord; N],
        positive_terms: [bool; N],
    ) -> [crate::verified::aggregate::Product; N] {
        use crate::verified::aggregate::{Magnitude, Product};
        core::array::from_fn(|index| {
            let sign = (if positive_terms[index] { Plus } else { Minus })
                * left[index].sign
                * right[index].sign;
            Product {
                active: sign != NoSign,
                negative: sign == Minus,
                left: Magnitude::Word(left[index].magnitude),
                right: right[index].magnitude,
                left_shift: left[index].denominator_shift,
                right_shift: right[index].denominator_shift,
            }
        })
    }

    /// Attempt two signed dyadic products and their sum entirely in native words.
    ///
    /// Shared planning checks both exponent sums, including inactive descriptors.
    /// Each active product and alignment must fit u128 before signed addition;
    /// opposite signs can cancel at that final addition. None means a checked
    /// step declined. Success returns a normalized word, including canonical zero.
    #[inline]
    fn product_sum2_dyadic_words_word(
        left: [DyadicWord; 2],
        right: [DyadicWord; 2],
        positive_terms: [bool; 2],
    ) -> Option<DyadicWord> {
        crate::verified::aggregate::sum_products_word(&Self::dyadic_word_products(left, right, positive_terms))
            .map(DyadicWord::from_native)
    }

    /// Attempt the oriented 2D determinant `left[0]*right[1] - left[1]*right[0]`.
    ///
    /// Uses native factors with separate six-limb positive/negative subtotals and
    /// returns a normalized retained dyadic sum. Inherits checked exponent and
    /// subtotal limits from product_sum_dyadic_words; None does not prove a zero
    /// determinant. No rational coefficients are materialized here.
    fn cross_dyadic_words(left: [DyadicWord; 2], right: [DyadicWord; 2]) -> Option<DyadicStackSum> {
        Self::product_sum_dyadic_words(
            [left[0], left[1]],
            [right[1], right[0]],
            [true, false],
        )
    }

    /// Attempt the oriented 2D determinant with the two-product native schedule.
    ///
    /// Forms `left[0]*right[1] - left[1]*right[0]`, retaining its reduced dyadic word.
    /// Individual products and alignments must fit u128 before cancellation;
    /// None leaves callers free to try a wider exact determinant implementation.
    fn cross_dyadic_words_word(
        left: [DyadicWord; 2],
        right: [DyadicWord; 2],
    ) -> Option<DyadicWord> {
        Self::product_sum2_dyadic_words_word(
            [left[0], left[1]],
            [right[1], right[0]],
            [true, false],
        )
    }

    /// Attempt a signed sum of four-limb by u128 dyadic products in six-limb buffers.
    ///
    /// Combines stored signs with each add/subtract polarity and copies all
    /// denominator exponents into shared aggregate descriptors. Planning includes
    /// inactive exponents. None means exponent or unsigned-subtotal overflow;
    /// cancellation is performed only after both subtotals fit. Empty input
    /// succeeds as zero; fixed arithmetic scratch and N descriptors use no heap.
    fn product_sum_wide_narrow_words<const N: usize>(
        left: [DyadicWideWord; N],
        right: [DyadicWord; N],
        positive_terms: [bool; N],
    ) -> Option<DyadicStackSum> {
        use crate::verified::aggregate::{Magnitude, Product};
        let products = core::array::from_fn(|index| {
            let sign = (if positive_terms[index] { Plus } else { Minus })
                * left[index].sign
                * right[index].sign;
            Product {
                active: sign != NoSign,
                negative: sign == Minus,
                left: Magnitude::Wide(left[index].magnitude),
                right: right[index].magnitude,
                left_shift: left[index].denominator_shift,
                right_shift: right[index].denominator_shift,
            }
        });
        Self::product_sum_stack::<N>(products)
    }

    /// Adapt shared aggregate reduction to a normalized six-limb Rational carrier.
    ///
    /// Checks every descriptor's exponent sum, including inactive terms, then
    /// accumulates active products into separate unsigned 384-bit subtotals.
    /// Overflow of either subtotal declines before cancellation. Success retains
    /// the exact difference with common binary factors removed; empty or equal
    /// totals become canonical zero. No heap-backed denominator is constructed.
    fn product_sum_stack<const N: usize>(
        products: [crate::verified::aggregate::Product; N],
    ) -> Option<DyadicStackSum> {
        let (minus, magnitude, denominator_shift) =
            crate::verified::aggregate::sum_products::<N, DYADIC_STACK_LIMBS>(&products)?;
        let sign = if magnitude == [0; DYADIC_STACK_LIMBS] {
            NoSign
        } else if minus {
            Minus
        } else {
            Plus
        };
        Some(DyadicStackSum {
            sign,
            magnitude: DyadicStackAccumulator(magnitude),
            denominator_shift,
        })
    }

    /// Plan a common binary denominator for the active terms of a dot product.
    ///
    /// The caller supplies each full product sign; NoSign terms are skipped before
    /// inspecting operands. Live operands must have unmarked reduced dyadic
    /// storage: marked lazy fractions decline even when their values are dyadic.
    /// Fact queries may learn denominator metadata without canonicalizing inputs.
    ///
    /// Returns per-term exponent sums and their maximum, with zeros for inactive
    /// terms and for an empty plan. The pairwise exponent addition is ordinary
    /// u64 arithmetic and assumes its sum fits; it is not a checked exhaustion
    /// outcome. The general product-sum planner instead uses checked addition.
    fn dyadic_product_alignment<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<([u64; N], u64)> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; N];
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let shift = left[i].dyadic_denominator_shift_if_reduced()?
                + right[i].dyadic_denominator_shift_if_reduced()?;
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
        }
        Some((denominator_shifts, max_shift))
    }

    /// Evaluate a dot product through native, six-limb and BigUint dyadic schedules.
    ///
    /// Trusts caller-supplied product signs and first requires reduced dyadic
    /// admission for live operands. None means that planning declined; native or
    /// stack overflow instead continues to arbitrary-precision signed subtotals
    /// over one binary denominator. All inactive terms produce exact zero.
    ///
    /// Native and stack successes emit their accumulator dispatch events. The
    /// final path may allocate large shifted numerators and a denominator, assumes
    /// shifts fit usize, and has no local work/memory/cancellation budget. No input
    /// approximation or precision loss is used to recover from a compact decline.
    fn dot_products_dyadic<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let (denominator_shifts, max_shift) =
            Self::dyadic_product_alignment(left, right, signs)?;

        if let Some(result) =
            Self::dot_products_dyadic_words(left, right, signs, denominator_shifts, max_shift)
        {
            crate::trace_dispatch!("rational", "dot_product", "dyadic-word-accumulator");
            return Some(result);
        }

        if let Some(result) =
            Self::dot_products_dyadic_stack(left, right, signs, denominator_shifts, max_shift)
        {
            crate::trace_dispatch!("rational", "dot_product", "dyadic-stack-accumulator");
            return Some(result);
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic dot-product scale should fit in usize");
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Materialize a dot product only if its supplied plan fits the six-limb path.
    ///
    /// The caller must supply current product signs, correct dyadic exponent sums
    /// and a maximum covering every active term. Delegates checked accumulation
    /// to dot_products_dyadic_stack_sum, then materializes its normalized result.
    /// None leaves the general fallback to the caller; final denominator storage
    /// can allocate even though the accumulation itself uses fixed buffers.
    fn dot_products_dyadic_stack<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
        denominator_shifts: [u64; N],
        max_shift: u64,
    ) -> Option<Self> {
        Self::dot_products_dyadic_stack_sum(
            left,
            right,
            signs,
            denominator_shifts,
            max_shift,
        )
        .map(Self::materialize_dyadic_stack_sum)
    }

    /// Check dyadic admission and retain a dot product without its BigUint denominator.
    ///
    /// Uses caller-supplied product signs, plans reduced dyadic operands, then
    /// attempts native factors and six-limb signed subtotals. Returns None on
    /// planning or compact-width failure, with no arbitrary-precision retry here.
    /// Empty input succeeds as zero; marked unreduced live operands are ineligible.
    fn dot_products_known_dyadic_stack_sum<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<DyadicStackSum> {
        let (denominator_shifts, max_shift) =
            Self::dyadic_product_alignment(left, right, signs)?;
        Self::dot_products_dyadic_stack_sum(
            left,
            right,
            signs,
            denominator_shifts,
            max_shift,
        )
    }

    /// Accumulate a preplanned dot product into a normalized six-limb dyadic sum.
    ///
    /// Trusts the signs and exponent plan; it does not inspect denominators or
    /// canonicalize operands. Each live numerator must fit u128, and each shifted
    /// product and unsigned subtotal must fit 384 bits before cancellation. None
    /// discards local scratch after a conversion or accumulation decline.
    ///
    /// The caller must ensure max_shift covers every active denominator_shifts
    /// entry. Empty or equal subtotals return canonical zero. No heap arithmetic
    /// or output denominator materialization occurs in this helper.
    fn dot_products_dyadic_stack_sum<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
        denominator_shifts: [u64; N],
        max_shift: u64,
    ) -> Option<DyadicStackSum> {
        let mut positive = DyadicStackAccumulator::default();
        let mut negative = DyadicStackAccumulator::default();
        for i in 0..N {
            let accumulator = match signs[i] {
                Plus => &mut positive,
                Minus => &mut negative,
                NoSign => continue,
            };
            accumulator.add_product(
                left[i].numerator.to_u128()?,
                right[i].numerator.to_u128()?,
                max_shift - denominator_shifts[i],
            )?;
        }

        Some(Self::finish_dyadic_stack_sum(
            positive, negative, max_shift,
        ))
    }

    /// Build a Rational from a retained sum whose normalization is already proved.
    ///
    /// NoSign returns canonical zero immediately. Otherwise uses native reduced
    /// parts when the magnitude and binary denominator fit u128, or constructs a
    /// BigUint denominator and the stored limb magnitude. It supplies no general
    /// normalization: callers must establish nonzero magnitude and coprimality
    /// with 2^denominator_shift. A forged carrier can publish unreduced parts as
    /// reduced or violate constructor assertions.
    ///
    /// The wide path records a rational temporary and assumes the exponent fits
    /// usize. Six numerator limbs do not bound denominator allocation or total
    /// work; no resource budget or cancellation check is supplied here.
    fn materialize_dyadic_stack_sum(sum: DyadicStackSum) -> Self {
        if sum.sign == NoSign {
            return Self::zero();
        }
        if sum.denominator_shift < 128
            && let Some(magnitude) = sum.magnitude.to_u128()
        {
            return Self::from_reduced_word_parts(
                sum.sign,
                magnitude,
                1_u128 << sum.denominator_shift,
            );
        }

        let denominator = BigUint::one()
            << usize::try_from(sum.denominator_shift).expect("dyadic shift fits usize");
        trace_rational_temporary!();
        Self::from_parts_raw(
            sum.sign,
            sum.magnitude.into_biguint(),
            denominator,
        )
    }

    /// Attempt unsigned positive/negative u128 totals for a preplanned product sum.
    ///
    /// Trusts supplied term signs and dyadic exponents. Skips NoSign terms; each
    /// live stored numerator product, alignment and subtotal must fit checked
    /// native arithmetic. None occurs before subtraction, so cancellation cannot
    /// rescue overflowing subtotals. A shift of at least 128 also declines.
    ///
    /// The caller must supply a maximum covering every live exponent and current
    /// signs/plan for these exact operands; denominators are not revalidated.
    /// Empty factor products are one and an empty term list returns (0, 0).
    /// The common scale is retained by the caller, not returned with the totals.
    fn signed_product_sum_dyadic_word_totals<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Option<(u128, u128)> {
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let mut magnitude = 1_u128;
            for factor in terms[i] {
                magnitude = magnitude.checked_mul(factor.numerator.to_u128()?)?;
            }
            let scale_shift = u32::try_from(max_shift - denominator_shifts[i]).ok()?;
            let magnitude = Self::checked_word_shift_left(magnitude, scale_shift)?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some((positive, negative))
    }

    /// Attempt native signed-product subtotals while discovering their common scale.
    ///
    /// Computes full term signs and skips zeros, then checks each live factor's
    /// reduced dyadic admission, exponent addition and numerator multiplication.
    /// Raises both accumulated subtotals when the maximum exponent grows, or
    /// aligns a later term to that maximum. Conversion, shift, product or subtotal
    /// overflow returns None; no general fallback or canonicalization occurs here.
    ///
    /// Only the two unsigned totals are returned: their comparison decides the
    /// sum's sign because the omitted scale is positive. Materializing a value
    /// requires recovering that scale. Empty factor products are one; an empty
    /// sum returns (0, 0). Storage is native and has no heap arithmetic.
    fn signed_product_sum_dyadic_word_totals_unplanned<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<(u128, u128)> {
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        let mut max_shift = 0_u64;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                continue;
            }
            let mut magnitude = 1_u128;
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift =
                    shift.checked_add(factor.dyadic_denominator_shift_if_reduced()?)?;
                magnitude = magnitude.checked_mul(factor.numerator.to_u128()?)?;
            }
            // Both totals stay aligned to `max_shift`. Because each side is a
            // monotonic sum of magnitudes, raising that alignment has the same
            // checked-word success envelope as scaling every term afterward.
            if shift > max_shift {
                if positive != 0 || negative != 0 {
                    let scale_shift = u32::try_from(shift - max_shift).ok()?;
                    positive = Self::checked_word_shift_left(positive, scale_shift)?;
                    negative = Self::checked_word_shift_left(negative, scale_shift)?;
                }
                max_shift = shift;
            } else if shift < max_shift {
                let scale_shift = u32::try_from(max_shift - shift).ok()?;
                magnitude = Self::checked_word_shift_left(magnitude, scale_shift)?;
            }
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some((positive, negative))
    }

    /// Materialize a reduced rational from checked native dyadic product subtotals.
    ///
    /// Inherits the caller's signs and exponent-plan preconditions from
    /// signed_product_sum_dyadic_word_totals. None means native accumulation
    /// declined. Otherwise subtracts the totals and cancels common binary factors;
    /// equality returns canonical zero before denominator construction.
    ///
    /// Native accumulation does not require the final denominator to fit u128:
    /// exponents of at least 128 materialize a BigUint and record a temporary.
    /// That path assumes the exponent fits usize and has no allocation budget.
    fn signed_product_sum_dyadic_words_with_plan<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Option<Self> {
        let (positive, negative) = Self::signed_product_sum_dyadic_word_totals(
            terms,
            signs,
            denominator_shifts,
            max_shift,
        )?;
        let (sign, mut magnitude) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => return Some(Self::zero()),
        };
        let common_shift = u64::from(magnitude.trailing_zeros()).min(max_shift);
        magnitude >>= u32::try_from(common_shift).expect("u128 trailing-zero count fits u32");
        let denominator_shift = max_shift - common_shift;
        if denominator_shift < 128 {
            return Some(Self::from_reduced_word_parts(
                sign,
                magnitude,
                1_u128 << denominator_shift,
            ));
        }

        let denominator = BigUint::one()
            << usize::try_from(denominator_shift).expect("dyadic shift fits usize");
        trace_rational_temporary!();
        Some(Self::from_parts_raw(
            sign,
            BigUint::from(magnitude),
            denominator,
        ))
    }

    /// Adapt a preplanned dot product to the two-factor native product-sum reducer.
    ///
    /// Builds N borrowed factor pairs and preserves the caller's full product
    /// signs, per-term denominator exponents and common maximum without validation.
    /// Returns None on native-total failure; success may allocate a wide final
    /// denominator through signed_product_sum_dyadic_words_with_plan.
    fn dot_products_dyadic_words<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
        denominator_shifts: [u64; N],
        max_shift: u64,
    ) -> Option<Self> {
        let terms = std::array::from_fn(|i| [left[i], right[i]]);
        Self::signed_product_sum_dyadic_words_with_plan(
            terms,
            signs,
            denominator_shifts,
            max_shift,
        )
    }

    /// Sum dot-product terms whose stored product denominators are identical.
    ///
    /// Checks each active left-denominator times right-denominator, not equality
    /// of individual factor denominators. Caller-supplied NoSign terms are ignored;
    /// no active terms return canonical zero. None means a product-denominator
    /// mismatch, possibly after allocating earlier products.
    ///
    /// Matching terms use BigUint positive/negative numerator totals and one final
    /// exact reduction. Valid unreduced storage is accepted without canonicalizing
    /// inputs. Coefficient growth, allocations and cancellation are not budgeted.
    fn dot_products_equal_denominator<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let mut shared_denominator = None::<BigUint>;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            match &shared_denominator {
                None => shared_denominator = Some(denominator),
                Some(shared) if *shared == denominator => {}
                Some(_) => return None,
            }
        }

        let Some(denominator) = shared_denominator else {
            return Some(Self::zero());
        };

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = &left[i].numerator * &right[i].numerator;
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Multiply a term's stored denominator magnitudes for delayed exact reduction.
    ///
    /// Clones the first denominator and multiplies the remaining factors in order;
    /// an empty product is one. Does not canonicalize factors, cancel against
    /// numerators or skip work for zero-valued terms. Arbitrary-precision growth
    /// and allocation are left to the surrounding product-sum operation.
    fn product_term_denominator<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut factors = term.into_iter();
        let Some(first) = factors.next() else {
            return BigUint::one();
        };
        let mut denominator = first.denominator.clone();
        for factor in factors {
            denominator *= &factor.denominator;
        }
        denominator
    }

    /// Multiply a term's stored unsigned numerators without intermediate reduction.
    ///
    /// Clones the first magnitude and multiplies remaining factors in order,
    /// continuing the loop even after a zero factor. Empty products are one; signs
    /// and denominators are handled separately. BigUint work and allocation have
    /// no local resource limit, and valid lazy fraction storage is preserved.
    fn product_term_magnitude<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut factors = term.into_iter();
        let Some(first) = factors.next() else {
            return BigUint::one();
        };
        let mut magnitude = first.numerator.clone();
        for factor in factors {
            magnitude *= &factor.numerator;
        }
        magnitude
    }

    /// Combine an add/subtract polarity with every stored factor sign.
    ///
    /// True starts at Plus and false at Minus. Multiplies all signs even after
    /// encountering NoSign; zero factors make the final term inactive. An empty
    /// factor list retains the polarity, representing plus or minus one. No
    /// canonicalization, numeric predicate or heap arithmetic is needed.
    fn product_term_sign<const FACTORS: usize>(positive: bool, term: [&Self; FACTORS]) -> Sign {
        let mut sign = if positive { Plus } else { Minus };
        for factor in term {
            sign = sign * factor.sign;
        }
        sign
    }

    /// Recognize active reduced dyadic terms and estimate their native-sum width.
    ///
    /// Trusts caller-supplied complete term signs and skips NoSign terms. Checks
    /// each live factor's reduced dyadic admission and checked-adds its denominator
    /// exponent. None means a live factor declined or its exponent sum overflowed;
    /// marked lazy fractions are not promoted. Fact queries may warm metadata.
    ///
    /// Retains each exponent sum and their maximum. prefer_wide uses saturating
    /// numerator-bit sums, alignment growth and ceil(log2(live_terms)) to predict
    /// a width above 128; this conservative dispatch estimate is not a proof that
    /// native evaluation fails or a measured cost bound. Empty plans have zero
    /// exponents and do not prefer wide. Const-sized arrays scale with TERMS;
    /// this planner supplies no total resource or cancellation contract.
    fn product_sum_dyadic_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<DyadicProductSumPlan<TERMS>> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut numerator_bits = [0_u64; TERMS];
        let mut live_terms = 0_u64;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            live_terms += 1;
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift =
                    shift.checked_add(factor.dyadic_denominator_shift_if_reduced()?)?;
                numerator_bits[i] = numerator_bits[i].saturating_add(factor.numerator.bits());
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
        }
        let total_growth = live_terms.next_power_of_two().trailing_zeros();
        let prefer_wide = (0..TERMS).any(|i| {
            signs[i] != NoSign
                && numerator_bits[i]
                    .saturating_add(max_shift - denominator_shifts[i])
                    .saturating_add(u64::from(total_growth))
                    > u64::from(u128::BITS)
        });
        Some(DyadicProductSumPlan {
            denominator_shifts,
            max_shift,
            prefer_wide,
        })
    }

    /// Evaluate a preplanned dyadic product sum with arbitrary-precision subtotals.
    ///
    /// Trusts current term signs and exact per-term exponents with a common maximum
    /// covering them. Multiplies stored numerators, shifts each active term to that
    /// denominator, then reduces the difference of positive and negative totals.
    /// No denominator admission check or per-term rational construction is repeated.
    ///
    /// Shifts must fit usize. Even zero or empty totals reach construction of the
    /// common binary denominator before final reduction. Large integers and
    /// denominators are unbudgeted; this is an exact fallback, not typed exhaustion.
    fn signed_product_sum_dyadic_with_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Self {
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic product-sum scale should fit in usize");
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        )
    }

    /// Decide a preplanned dyadic sum's sign by comparing its BigUint subtotals.
    ///
    /// Trusts current complete term signs, exact exponents and a covering common
    /// maximum. Aligns active stored-numerator products and compares positive with
    /// negative, yielding the sum's ordering relative to zero; empty input is Equal.
    /// The omitted common denominator is positive, so no Rational or denominator
    /// needs construction. Shift conversion assumes usize fit; potentially large
    /// intermediate numerators still have no work, memory or cancellation budget.
    fn signed_product_sum_dyadic_ordering_with_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Ordering {
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic product-sum scale should fit in usize");
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        positive.cmp(&negative)
    }

    /// Attempt a two-factor sum's sign with four-limb by native products.
    ///
    /// Rejects FACTORS other than two, including an empty sum of that shape. For
    /// each active term, first tries a native right numerator and four-limb left;
    /// otherwise swaps their roles. Checks product alignment and separate unsigned
    /// six-limb subtotals before comparing high limbs first. None means a shape,
    /// width or subtotal limit, even when signed cancellation would fit.
    ///
    /// Trusts the caller's complete term signs and exact dyadic exponent plan with
    /// a covering maximum; denominators are not revalidated. For two-factor shape,
    /// empty input returns Equal. No denominator, Rational or BigUint arithmetic is
    /// materialized, and general fallback remains the caller's responsibility.
    fn signed_product_sum_dyadic_wide_narrow_ordering_with_plan<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Option<Ordering> {
        if FACTORS != 2 {
            return None;
        }
        let fixed_magnitude = |value: &BigUint| {
            if value.bits() > 256 {
                return None;
            }
            let mut words = [0_u64; 4];
            for (index, word) in value.iter_u64_digits().enumerate() {
                words[index] = word;
            }
            Some(words)
        };
        let mut positive = DyadicStackAccumulator::default();
        let mut negative = DyadicStackAccumulator::default();
        for i in 0..TERMS {
            let accumulator = match signs[i] {
                Plus => &mut positive,
                Minus => &mut negative,
                NoSign => continue,
            };
            let left = &terms[i][0].numerator;
            let right = &terms[i][1].numerator;
            let (wide, narrow) = if let Some(narrow) = right.to_u128() {
                (fixed_magnitude(left)?, narrow)
            } else {
                (fixed_magnitude(right)?, left.to_u128()?)
            };
            accumulator.add_wide_word_product(
                wide,
                narrow,
                max_shift - denominator_shifts[i],
            )?;
        }
        Some(positive.0.iter().rev().cmp(negative.0.iter().rev()))
    }

    /// Attempt an exact signed product sum with one stored denominator for all live factors.
    ///
    /// Each row is a product, added for true polarity and subtracted for false.
    /// Checks equality of every live factor's stored denominator, not merely the
    /// product denominators or canonical values. Zero products are skipped before
    /// that check. Valid unreduced storage is accepted without canonicalizing it;
    /// equal values in different representations can have different eligibility.
    ///
    /// None means a denominator mismatch, or a factor count outside u32 after the
    /// native path declines. Callers can use signed_product_sum for general exact
    /// evaluation. An empty sum or all-zero terms return Some(zero). Zero-factor
    /// products are one and delegate to the general reducer with their polarities.
    ///
    /// After validation, tries checked native reduction, then accumulates unsigned
    /// BigUint totals over the common denominator raised to FACTORS and reduces
    /// once. Intermediate integer growth, const-sized arrays and allocation have
    /// no local work/memory/cancellation limit. A compact decline is not exhaustion
    /// or evidence that the exact result does not exist.
    pub fn signed_product_sum_shared_denominator<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<Self> {
        if FACTORS == 0 {
            return Some(Self::signed_product_sum(positive_terms, terms));
        }
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        let mut shared_denominator = None::<&BigUint>;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                signs[i] = sign;
                continue;
            }
            nonzero_count += 1;
            signs[i] = sign;
            for factor in terms[i] {
                match shared_denominator {
                    None => shared_denominator = Some(&factor.denominator),
                    Some(shared) if shared == &factor.denominator => {}
                    Some(_) => return None,
                }
            }
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!(
                "rational",
                "product_sum",
                "shared-factor-denominator-all-zero"
            );
            return Some(Self::zero());
        }
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "word-sized-shared-scale");
            return Some(word);
        }
        let exponent = u32::try_from(FACTORS).ok()?;
        let denominator = shared_denominator
            .expect("nonzero product sum has a live factor denominator")
            .pow(exponent);
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = Self::product_term_magnitude(terms[i]);
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!("rational", "product_sum", "shared-factor-denominator");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Return the exact arithmetic mean of a nonempty slice of borrowed rationals.
    ///
    /// None means the slice is empty. The divisor is the complete slice length,
    /// including zero entries. Canonicalizes lazy operands before choosing a
    /// common dyadic, equal-denominator or LCM schedule; all-zero input succeeds
    /// as canonical zero. Accumulates signed integer magnitudes before final
    /// reduction instead of constructing every partial rational sum.
    ///
    /// Input canonicalization may itself reduce and warm caches, so this is not
    /// a promise of one GCD for the entire call. Large denominators and totals
    /// allocate without a total-work, memory or cancellation budget.
    pub fn mean_refs(values: &[&Self]) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
        let count = BigUint::from(values.len());
        Some(Self::sum_refs_with_denominator_factor(values, &count))
    }

    /// Return (a+b+c)/3 using a fixed three-value exact denominator schedule.
    ///
    /// Canonicalizes all operands, then returns zero for three zeros. If all live
    /// denominators agree, sums magnitudes over three times that denominator;
    /// otherwise multiplies all three denominators and scales each numerator by
    /// the other two. Zero entries still count in the divisor three.
    ///
    /// This centroid-oriented schedule avoids a preliminary LCM calculation but
    /// can build a larger product denominator. It does not choose the general
    /// mean's dyadic alignment path. Lazy input reduction and final reduction can
    /// allocate; no measured speed claim or local resource bound is implied.
    pub fn mean3_refs(values: [&Self; 3]) -> Self {
        let values = values.map(Self::canonicalized_ref);
        let mut live_count = 0_usize;
        let mut shared_denominator = None::<&BigUint>;
        let mut equal_denominator = true;
        for value in values {
            if value.sign == NoSign {
                continue;
            }
            live_count += 1;
            match shared_denominator {
                None => shared_denominator = Some(&value.denominator),
                Some(shared) if shared == &value.denominator => {}
                Some(_) => equal_denominator = false,
            }
        }
        if live_count == 0 {
            crate::trace_dispatch!("rational", "mean3", "all-zero");
            return Self::zero();
        }
        if equal_denominator {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for value in values {
                match value.sign {
                    Plus => positive += &value.numerator,
                    Minus => negative += &value.numerator,
                    NoSign => {}
                }
            }
            let denominator = shared_denominator
                .expect("nonzero three-value mean has a shared denominator")
                * BigUint::from(3_u8);
            crate::trace_dispatch!("rational", "mean3", "equal-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for index in 0..3 {
            let value = values[index];
            if value.sign == NoSign {
                continue;
            }
            let mut magnitude = value.numerator.clone();
            magnitude *= &values[(index + 1) % 3].denominator;
            magnitude *= &values[(index + 2) % 3].denominator;
            match value.sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        let denominator01 = &values[0].denominator * &values[1].denominator;
        let mut denominator = &denominator01 * &values[2].denominator;
        denominator *= BigUint::from(3_u8);
        crate::trace_dispatch!("rational", "mean3", "direct-product-denominator");
        Self::from_signed_magnitude_difference(positive, negative, denominator)
    }

    /// Evaluate the exact sum of borrowed rationals divided by a positive integer factor.
    ///
    /// The mean caller supplies its nonzero input count. Canonicalizes operands
    /// while recognizing live denominators, then tries dyadic alignment, equal
    /// denominators and an LCM in that order. Zero terms do not enlarge the common
    /// denominator. Accumulates positive/negative magnitudes and multiplies the
    /// final denominator by factor before reducing the result.
    ///
    /// The caller must establish factor > 0; it is not validated at entry. Empty,
    /// all-zero or cancelling sums can return zero even for an invalid zero factor,
    /// while a nonzero result can retain an invalid zero denominator. Repeated
    /// canonicalized views can reuse input caches. BigUint shifts, LCM work and
    /// retained/temporary allocations have no local budget or cancellation check.
    fn sum_refs_with_denominator_factor(values: &[&Self], factor: &BigUint) -> Self {
        let mut live_count = 0_usize;
        let mut common_dyadic_shift = Some(0_u64);
        let mut shared_denominator = None::<&BigUint>;
        let mut equal_denominator = true;
        for &value in values {
            let value = value.canonicalized_ref();
            if value.sign == NoSign {
                continue;
            }
            live_count += 1;
            common_dyadic_shift = match (common_dyadic_shift, value.dyadic_denominator_shift()) {
                (Some(current), Some(shift)) => Some(current.max(shift)),
                _ => None,
            };
            match shared_denominator {
                None => shared_denominator = Some(&value.denominator),
                Some(shared) if shared == &value.denominator => {}
                Some(_) => equal_denominator = false,
            }
        }
        if live_count == 0 {
            crate::trace_dispatch!("rational", "mean", "all-zero");
            return Self::zero();
        }

        if let Some(common_shift) = common_dyadic_shift {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for &value in values {
                let value = value.canonicalized_ref();
                if value.sign == NoSign {
                    continue;
                }
                let shift = value
                    .dyadic_denominator_shift()
                    .expect("all live mean inputs were certified dyadic");
                let magnitude = &value.numerator << (common_shift - shift);
                match value.sign {
                    Plus => positive += magnitude,
                    Minus => negative += magnitude,
                    NoSign => {}
                }
            }
            let mut denominator = BigUint::one() << common_shift;
            denominator *= factor;
            crate::trace_dispatch!("rational", "mean", "dyadic-shared-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        if equal_denominator {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for &value in values {
                let value = value.canonicalized_ref();
                match value.sign {
                    Plus => positive += &value.numerator,
                    Minus => negative += &value.numerator,
                    NoSign => {}
                }
            }
            let mut denominator = shared_denominator
                .expect("nonzero mean has a shared denominator")
                .clone();
            denominator *= factor;
            crate::trace_dispatch!("rational", "mean", "equal-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        let mut common_denominator = BigUint::one();
        for &value in values {
            let value = value.canonicalized_ref();
            if value.sign == NoSign {
                continue;
            }
            if value.denominator != *ONE.deref() {
                let divisor = Self::gcd_magnitudes_with_mixed_width_fast_path(
                    &common_denominator,
                    &value.denominator,
                );
                trace_rational_gcd!(&common_denominator, &value.denominator, &divisor);
                common_denominator *= &value.denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for &value in values {
            let value = value.canonicalized_ref();
            if value.sign == NoSign {
                continue;
            }
            let mut magnitude = value.numerator.clone();
            if value.denominator != common_denominator {
                magnitude *= &common_denominator / &value.denominator;
            }
            match value.sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        common_denominator *= factor;
        crate::trace_dispatch!("rational", "mean", "lcm-shared-denominator");
        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    /// Evaluate two signed products of two borrowed rational factors exactly.
    ///
    /// True polarity adds and false subtracts the corresponding product. Computes
    /// full structural signs, first attempts the shared checked native reducer,
    /// then delegates unchanged inputs to signed_product_sum. Native overflow or
    /// representation ineligibility therefore preserves general exact evaluation;
    /// zero terms and valid unreduced operands remain supported.
    ///
    /// Distinct dispatch events report native success or fallback. This entry
    /// point lets small determinant and complex-arithmetic consumers select their
    /// fixed shape without inspecting scalar storage. Arbitrary-precision fallback
    /// is unbudgeted and may allocate; it does not approximate the result.
    pub fn signed_product_sum2(
        positive_terms: [bool; 2],
        terms: [[&Self; 2]; 2],
    ) -> Self {
        let signs = [
            Self::product_term_sign(positive_terms[0], terms[0]),
            Self::product_term_sign(positive_terms[1], terms[1]),
        ];
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "fixed-two-by-two-word-sized");
            return word;
        }
        crate::trace_dispatch!("rational", "product_sum", "fixed-two-by-two-fallback");
        Self::signed_product_sum(positive_terms, terms)
    }

    /// Evaluate a signed sum of two-factor products under a dyadic-input proof.
    ///
    /// Canonicalizes every factor, computes full term signs and uses the dyadic
    /// dot-product schedules, including arbitrary-precision fallback when compact
    /// widths fail. True polarity adds and false subtracts. Empty or all-zero
    /// terms return exact zero; no denominator check is needed for inactive terms.
    ///
    /// The caller must establish dyadic factors. An active non-dyadic factor panics
    /// when admission declines, even if cancellation would make the final sum zero;
    /// this is not a checked general-rational entry point. Canonicalization and wide
    /// arithmetic can allocate and have no local work or cancellation budget.
    pub fn signed_product_sum_known_dyadic<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        let terms = terms.map(|term| term.map(Self::canonicalized_ref));
        let signs: [Sign; TERMS] = core::array::from_fn(|index| {
            Self::product_term_sign(positive_terms[index], terms[index])
        });
        crate::trace_dispatch!("rational", "product_sum", "known-dyadic-two-factor");
        Self::dot_products_dyadic(
            core::array::from_fn(|index| terms[index][0]),
            core::array::from_fn(|index| terms[index][1]),
            signs,
        )
        .expect("known-dyadic product sum received a non-dyadic factor")
    }

    /// Divide two exact rationals whose dyadic classification is already established.
    ///
    /// Canonicalizes both inputs, rejects a zero divisor with DivideByZero, then
    /// returns zero for a zero numerator. Only afterward queries dyadic exponents;
    /// an active non-dyadic operand violates the caller's proof and panics. Thus
    /// zero-over-non-dyadic can return zero without validating that classification.
    ///
    /// First tries native magnitude cancellation and a signed binary-scale
    /// difference fitting i32. On decline, cancels the arbitrary-precision
    /// magnitude GCD and shifts the appropriate quotient part by the exponent
    /// difference. Canonical dyadic inputs justify publishing reduced output.
    /// Distinct dispatch events identify native and wide cross-cancellation.
    ///
    /// Shift conversion assumes usize fit. Input canonicalization, magnitude GCD
    /// and shifted output allocation have no total-work, memory or cancellation
    /// budget; compact ineligibility does not become a typed resource error.
    pub(crate) fn quotient_known_dyadic(
        numerator: &Self,
        denominator: &Self,
    ) -> Result<Self, crate::Problem> {
        let numerator = numerator.canonicalized_ref();
        let denominator = denominator.canonicalized_ref();
        if denominator.sign == NoSign {
            return Err(crate::Problem::DivideByZero);
        }
        if numerator.sign == NoSign {
            return Ok(Self::zero());
        }
        let numerator_shift = numerator
            .dyadic_denominator_shift()
            .expect("known-dyadic numerator has a power-of-two denominator");
        let denominator_shift = denominator
            .dyadic_denominator_shift()
            .expect("known-dyadic denominator has a power-of-two denominator");
        if let (Some(numerator_word), Some(denominator_word), Ok(scale_shift)) = (
            numerator.numerator.to_u128(),
            denominator.numerator.to_u128(),
            i32::try_from(i128::from(denominator_shift) - i128::from(numerator_shift)),
        ) && let Some(result) = Self::from_scaled_dyadic_quotient_component(
            numerator.sign * denominator.sign,
            numerator_word,
            denominator_word,
            scale_shift,
        ) {
            crate::trace_dispatch!("rational", "div", "known-dyadic-word-cross-cancel");
            return Ok(result);
        }
        let divisor = Self::gcd_magnitudes_with_mixed_width_fast_path(
            &numerator.numerator,
            &denominator.numerator,
        );
        trace_rational_gcd!(&numerator.numerator, &denominator.numerator, &divisor);
        let (mut magnitude, mut scale) = if divisor.is_one() {
            (numerator.numerator.clone(), denominator.numerator.clone())
        } else {
            (
                &numerator.numerator / &divisor,
                &denominator.numerator / &divisor,
            )
        };
        if denominator_shift > numerator_shift {
            magnitude <<= usize::try_from(denominator_shift - numerator_shift)
                .expect("dyadic quotient shift fits usize");
        } else if numerator_shift > denominator_shift {
            scale <<= usize::try_from(numerator_shift - denominator_shift)
                .expect("dyadic quotient shift fits usize");
        }
        crate::trace_dispatch!("rational", "div", "known-dyadic-cross-cancel");
        trace_rational_temporary!();
        Ok(Self::from_parts_raw(
            numerator.sign * denominator.sign,
            magnitude,
            scale,
        ))
    }

    /// Materialize a dyadic quotient using the caller's already proved magnitude GCD.
    ///
    /// Inputs must be canonical dyadics with nonzero denominator. The supplied
    /// shifts must describe those exact operands, denominator_word must equal the
    /// denominator's numerator magnitude, and divisor must be their positive complete
    /// magnitude GCD. This helper neither rediscovers that proof nor validates it
    /// generally; debug checks on the native path are only partial safeguards.
    ///
    /// Attempts native cancellation/scaling first and returns true in the second
    /// tuple slot on success. Otherwise divides the stored numerator and native
    /// denominator magnitude by divisor, applies the exponent difference, and
    /// publishes reduced parts with false. A zero numerator can use the native
    /// zero shortcut. Invalid proof data can cause truncating division, invalid
    /// reduced storage or a panic, not a typed domain error.
    ///
    /// The wide branch records a temporary, assumes shifts fit usize and has no
    /// allocation budget. The caller owns the quotient dispatch event.
    fn quotient_known_dyadic_with_word_divisor(
        numerator: &Self,
        denominator: &Self,
        numerator_shift: u64,
        denominator_shift: u64,
        denominator_word: u128,
        divisor: u128,
    ) -> (Self, bool) {
        if let (Some(numerator_word), Ok(scale_shift)) = (
            numerator.numerator.to_u128(),
            i32::try_from(i128::from(denominator_shift) - i128::from(numerator_shift)),
        ) && let Some(result) = Self::from_scaled_dyadic_quotient_component_with_divisor(
            numerator.sign * denominator.sign,
            numerator_word,
            denominator_word,
            scale_shift,
            divisor,
        ) {
            return (result, true);
        }

        let mut magnitude = if divisor == 1 {
            numerator.numerator.clone()
        } else {
            &numerator.numerator / divisor
        };
        let mut scale = BigUint::from(denominator_word / divisor);
        if denominator_shift > numerator_shift {
            magnitude <<= usize::try_from(denominator_shift - numerator_shift)
                .expect("dyadic quotient shift fits usize");
        } else if numerator_shift > denominator_shift {
            scale <<= usize::try_from(numerator_shift - denominator_shift)
                .expect("dyadic quotient shift fits usize");
        }
        trace_rational_temporary!();
        (
            Self::from_parts_raw(numerator.sign * denominator.sign, magnitude, scale),
            false,
        )
    }

    /// Divide a nonzero normalized six-limb dyadic sum using a proved native GCD.
    ///
    /// The divisor's sign must be nonzero, its magnitude denominator_word positive,
    /// and denominator_shift must describe its dyadic value. divisor is the exact
    /// positive GCD of the numerator magnitude and denominator_word. These are
    /// caller proofs; debug assertions check only selected nonzero/divisibility
    /// conditions, not a complete admission contract.
    ///
    /// Tries native cancellation/scaling, otherwise materializes the numerator
    /// limbs, divides both magnitudes by divisor and applies the exponent difference
    /// before publishing reduced parts. No Rational numerator need be constructed
    /// first. Invalid proof data can panic or publish incorrect/unreduced storage.
    /// Wide shifts assume usize fit and can allocate without a resource budget.
    fn quotient_dyadic_stack_sum_with_word_divisor(
        numerator: DyadicStackSum,
        denominator_sign: Sign,
        denominator_shift: u64,
        denominator_word: u128,
        divisor: u128,
    ) -> Self {
        debug_assert_ne!(numerator.sign, NoSign);
        if let (Some(numerator_word), Ok(scale_shift)) = (
            numerator.magnitude.to_u128(),
            i32::try_from(
                i128::from(denominator_shift) - i128::from(numerator.denominator_shift),
            ),
        ) && let Some(result) = Self::from_scaled_dyadic_quotient_component_with_divisor(
            numerator.sign * denominator_sign,
            numerator_word,
            denominator_word,
            scale_shift,
            divisor,
        ) {
            return result;
        }

        let mut magnitude = numerator.magnitude.into_biguint();
        if divisor != 1 {
            debug_assert_eq!((&magnitude % divisor).to_u128(), Some(0));
            magnitude /= divisor;
        }
        let mut scale = BigUint::from(denominator_word / divisor);
        if denominator_shift > numerator.denominator_shift {
            magnitude <<= usize::try_from(denominator_shift - numerator.denominator_shift)
                .expect("dyadic quotient shift fits usize");
        } else if numerator.denominator_shift > denominator_shift {
            scale <<= usize::try_from(numerator.denominator_shift - denominator_shift)
                .expect("dyadic quotient shift fits usize");
        }
        trace_rational_temporary!();
        Self::from_parts_raw(numerator.sign * denominator_sign, magnitude, scale)
    }

    /// Materialize a retained nonzero dyadic sum divided by a native dyadic divisor.
    ///
    /// Requires consistent normalized numerator metadata, a nonzero denominator
    /// sign, positive denominator_word and its correct binary exponent. Tries the
    /// native unreduced quotient constructor, then forms the magnitude ratio with
    /// its exponent difference using BigUint parts. It does not compute a general
    /// magnitude GCD; common odd factors may remain until canonical observation.
    ///
    /// The result remains an exact Rational with unreduced provenance where needed.
    /// Zero numerators must be handled by the caller. Invalid denominator metadata
    /// is an invariant violation, not a typed DivideByZero result. Wide shifts
    /// assume usize fit and have no allocation or cancellation budget.
    fn quotient_dyadic_stack_sum_unreduced(
        numerator: DyadicStackSum,
        denominator_sign: Sign,
        denominator_shift: u64,
        denominator_word: u128,
    ) -> Self {
        debug_assert_ne!(numerator.sign, NoSign);
        if let (Some(numerator_word), Ok(scale_shift)) = (
            numerator.magnitude.to_u128(),
            i32::try_from(
                i128::from(denominator_shift) - i128::from(numerator.denominator_shift),
            ),
        ) && let Some(result) = Self::from_scaled_dyadic_quotient_component_unreduced(
            numerator.sign * denominator_sign,
            numerator_word,
            denominator_word,
            scale_shift,
        ) {
            return result;
        }

        let mut magnitude = numerator.magnitude.into_biguint();
        let mut scale = BigUint::from(denominator_word);
        if denominator_shift > numerator.denominator_shift {
            magnitude <<= usize::try_from(denominator_shift - numerator.denominator_shift)
                .expect("dyadic quotient shift fits usize");
        } else if numerator.denominator_shift > denominator_shift {
            scale <<= usize::try_from(numerator.denominator_shift - denominator_shift)
                .expect("dyadic quotient shift fits usize");
        }
        trace_rational_temporary!();
        Self::from_parts_raw_unreduced(
            numerator.sign * denominator_sign,
            magnitude,
            scale,
        )
    }

    /// Materialize a nonzero four-limb magnitude as a BigUint.
    ///
    /// Finds the highest nonzero u64 limb, writes low/high u32 digits in little-endian
    /// order and removes the final high zero digit if present. Sign and binary
    /// exponent are ignored; this is a magnitude conversion, not validation of a
    /// dyadic value. All-zero limbs panic. The digit vector holds at most eight
    /// entries, and the returned BigUint owns its digit storage.
    fn dyadic_wide_magnitude(value: DyadicWideWord) -> BigUint {
        let last = value
            .magnitude
            .iter()
            .rposition(|limb| *limb != 0)
            .expect("nonzero wide dyadic word has a nonzero limb");
        let mut digits = Vec::with_capacity((last + 1) * 2);
        for limb in &value.magnitude[..=last] {
            digits.push(*limb as u32);
            digits.push((*limb >> 32) as u32);
        }
        while digits.last() == Some(&0) {
            digits.pop();
        }
        BigUint::new(digits)
    }

    /// Materialize a quotient of retained four-limb dyadic values.
    ///
    /// The caller must establish consistent signs/magnitudes and a nonzero divisor.
    /// A zero numerator returns canonical zero after the denominator-sign debug
    /// assertion. Nonzero inputs become BigUint magnitudes, with the exponent
    /// difference applied to the numerator or denominator as appropriate.
    ///
    /// With unreduced=true, retains exact possibly noncoprime parts for lazy
    /// canonicalization and records a temporary. Otherwise the general fraction
    /// constructor fully reduces; its error is treated as a violated nonzero
    /// determinant invariant. This helper does not provide typed domain failure.
    /// Shift conversion assumes usize fit; four input limbs do not bound shifted
    /// output allocation, reduction work or cancellation latency.
    fn quotient_dyadic_wide_words(
        numerator: DyadicWideWord,
        denominator: DyadicWideWord,
        unreduced: bool,
    ) -> Self {
        debug_assert_ne!(denominator.sign, NoSign);
        if numerator.sign == NoSign {
            return Self::zero();
        }
        let mut magnitude = Self::dyadic_wide_magnitude(numerator);
        let mut scale = Self::dyadic_wide_magnitude(denominator);
        if denominator.denominator_shift > numerator.denominator_shift {
            magnitude <<= usize::try_from(
                denominator.denominator_shift - numerator.denominator_shift,
            )
            .expect("dyadic quotient shift fits usize");
        } else if numerator.denominator_shift > denominator.denominator_shift {
            scale <<= usize::try_from(
                numerator.denominator_shift - denominator.denominator_shift,
            )
            .expect("dyadic quotient shift fits usize");
        }
        let sign = numerator.sign * denominator.sign;
        if unreduced {
            trace_rational_temporary!();
            Self::from_parts_raw_unreduced(sign, magnitude, scale)
        } else {
            Self::from_bigint_fraction(BigInt::from_biguint(sign, magnitude), scale)
                .expect("retained nonzero determinant is a valid rational divisor")
        }
    }

    /// Form a lazy exact quotient from a six-limb numerator and four-limb divisor.
    ///
    /// Used to materialize an affine coordinate whose common determinant remains
    /// retained. Requires consistent nonzero numerator and denominator values;
    /// zero coordinates are handled before calling. Converts both magnitudes,
    /// applies their binary exponent difference and stores unreduced rational
    /// parts without a general GCD, retaining the need for later canonicalization.
    ///
    /// Invalid signs or zero magnitudes violate constructor invariants rather than
    /// returning a typed error. Shifts must fit usize. Compact inputs do not bound
    /// shifted output allocation or the eventual reduction, and no work/memory/
    /// cancellation budget is enforced here.
    fn quotient_dyadic_stack_sum_by_wide_unreduced(
        numerator: DyadicStackSum,
        denominator: DyadicWideWord,
    ) -> Self {
        debug_assert_ne!(numerator.sign, NoSign);
        let mut magnitude = numerator.magnitude.into_biguint();
        let mut scale = Self::dyadic_wide_magnitude(denominator);
        if denominator.denominator_shift > numerator.denominator_shift {
            magnitude <<= usize::try_from(
                denominator.denominator_shift - numerator.denominator_shift,
            )
            .expect("dyadic quotient shift fits usize");
        } else if numerator.denominator_shift > denominator.denominator_shift {
            scale <<= usize::try_from(
                numerator.denominator_shift - denominator.denominator_shift,
            )
            .expect("dyadic quotient shift fits usize");
        }
        trace_rational_temporary!();
        Self::from_parts_raw_unreduced(
            numerator.sign * denominator.sign,
            magnitude,
            scale,
        )
    }

    /// Attempt the exact intersection of two dyadic supporting lines with compact parameters.
    ///
    /// Canonicalizes rational endpoints as needed, requiring native dyadic origins,
    /// differences and determinant magnitudes. Forms each coordinate from the first
    /// origin, direction and retained determinant quotient. Returns retained first/
    /// second line parameters and x/y rationals; nonzero coordinates defer general
    /// fraction reduction until canonical observation.
    ///
    /// No segment-range or proper-crossing test is performed. None conflates
    /// non-dyadic or overwide inputs, checked intermediate failure and zero
    /// determinant, including parallel, coincident or degenerate lines. It is a
    /// fallback signal, not proof of disjointness or declared resource exhaustion.
    ///
    /// Retained parameters are not checked against their later materializer's
    /// stricter native quotient bounds: a valid point can be returned while a
    /// parameter accessor would panic. Endpoint canonicalization and coordinate
    /// materialization can allocate; fixed scratch buffers do not bound total work,
    /// output size or cancellation latency. Success records dyadic-stack-fused-point.
    pub(crate) fn line_intersection2_point_known_dyadic(
        first_start: [&Self; 2],
        first_end: [&Self; 2],
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<(ExactDyadicLineParameters2, [Self; 2])> {
        let plan = Self::line_intersection2_plan_known_dyadic(
            first_start,
            first_end,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_point_from_plan(plan)?;
        crate::trace_dispatch!("rational", "line-intersection2", "dyadic-stack-fused-point");
        Some((
            ExactDyadicLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Reuse a compact first supporting line and materialize its intersection coordinates.
    ///
    /// Trusts the retained first origin/direction, canonicalizes second rational
    /// endpoints as needed and uses native determinant magnitudes with checked
    /// coordinate accumulation. Returns compact parameters and exact x/y rationals,
    /// retaining lazy reduction for nonzero coordinates.
    ///
    /// No segment membership is checked. None combines input/width ineligibility
    /// and zero determinant. Returned parameters retain the stricter materialization
    /// limitation of line_intersection2_point_known_dyadic. Conversion and output
    /// allocation are unbudgeted; success emits the first-line fused-point event.
    pub(crate) fn line_intersection2_point_with_first_line(
        first: &ExactDyadicLine2,
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<(ExactDyadicLineParameters2, [Self; 2])> {
        let plan =
            Self::line_intersection2_plan_with_first_line(first, second_start, second_end)?;
        let point = Self::line_intersection2_point_from_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-fused-point-first-line"
        );
        Some((
            ExactDyadicLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Intersect a retained first line with exact binary64 second endpoints.
    ///
    /// Decodes each finite float's exact binary value, including canonical zero
    /// for either signed zero; no decimal rounding or approximate geometry enters
    /// the construction. Nonfinite, overwide or excessive-alignment input, zero
    /// determinant and checked coordinate failure all return None.
    ///
    /// Uses compact determinant parameters and materialized x/y rationals with
    /// lazy reduction for nonzero coordinates. Supporting-line parameters need not
    /// lie in the segments or fit their later native materializers. Trusts the
    /// retained first line. Output allocation is unbudgeted; success emits the
    /// first-line-f64 fused-point event.
    pub(crate) fn line_intersection2_point_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<(ExactDyadicLineParameters2, [Self; 2])> {
        let plan = Self::line_intersection2_plan_with_first_line_exact_dyadic_f64(
            first,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_point_from_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-fused-point-first-line-f64"
        );
        Some((
            ExactDyadicLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Retain an exact binary64-line intersection without materializing coordinates.
    ///
    /// Reuses the first line, decodes exact second endpoints and builds compact
    /// determinant parameters plus two six-limb affine coordinate numerators over
    /// their shared determinant. Neither coordinate division nor Rational output
    /// construction occurs here. Later observation can allocate and defer reduction.
    ///
    /// None combines nonfinite/input-width rejection, zero determinant and checked
    /// accumulation failure. No segment-range test or source-incidence certificate
    /// is added. Compact parameter accessors still have narrower materialization
    /// bounds than retention. This fixed-buffer path emits its retained-point event;
    /// it supplies no total resource guarantee for subsequent materialization.
    pub(crate) fn line_intersection2_retained_point_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<(ExactDyadicLineParameters2, ExactDyadicLinePoint2)> {
        let plan = Self::line_intersection2_plan_with_first_line_exact_dyadic_f64(
            first,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_retained_point_from_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-retained-point-first-line-f64"
        );
        Some((
            ExactDyadicLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Attempt a supporting-line intersection retaining four-limb determinant magnitudes.
    ///
    /// Endpoint coordinates and differences still require native dyadic words;
    /// the wider tier applies to determinants, not arbitrary-width source points.
    /// Canonicalizes rational endpoints as needed, then forms six-limb affine
    /// numerators and materializes exact x/y coordinates with lazy reduction.
    ///
    /// Returns wide parameters without clipping to either segment. None combines
    /// source/intermediate ineligibility with zero determinant, not proof of no
    /// intersection. Shifted output denominators can exceed the fixed scratch
    /// widths; allocation, reduction and cancellation are not budgeted here.
    /// Success emits the wide fused-point dispatch event.
    pub(crate) fn line_intersection2_point_known_dyadic_wide(
        first_start: [&Self; 2],
        first_end: [&Self; 2],
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<(ExactDyadicWideLineParameters2, [Self; 2])> {
        let plan = Self::line_intersection2_wide_plan_known_dyadic(
            first_start,
            first_end,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_point_from_wide_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-fused-point-wide"
        );
        Some((
            ExactDyadicWideLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Reuse the first line for four-limb determinant planning and exact coordinates.
    ///
    /// Second rational endpoints canonicalize as needed and must fit native dyadic
    /// words, as must their checked differences. The wider retained determinant
    /// quotients then produce lazy exact x/y rationals through six-limb affine sums.
    /// Trusts the retained first-line invariants and performs no segment clipping.
    ///
    /// None conflates source/width failure with parallel, coincident or degenerate
    /// supporting lines. Output allocation and later reduction have no local budget;
    /// success records the wide-first-line fused-point event.
    pub(crate) fn line_intersection2_point_wide_with_first_line(
        first: &ExactDyadicLine2,
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<(ExactDyadicWideLineParameters2, [Self; 2])> {
        let plan = Self::line_intersection2_wide_plan_with_first_line(
            first,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_point_from_wide_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-fused-point-wide-first-line"
        );
        Some((
            ExactDyadicWideLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Materialize a retained-line/binary64 intersection through four-limb determinants.
    ///
    /// Second endpoints denote their exact binary values. Rejects nonfinite values
    /// and native source/difference limits before wider determinant and six-limb
    /// coordinate arithmetic. None also covers zero determinant or checked
    /// intermediate failure; it is not a segment-miss classification.
    ///
    /// Returns wide parameters and exact x/y rationals with deferred nonzero
    /// fraction reduction. Trusts the first-line carrier. Materialization may
    /// allocate beyond fixed scratch widths without a resource budget, and success
    /// emits the wide-first-line-f64 fused-point event.
    pub(crate) fn line_intersection2_point_wide_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<(ExactDyadicWideLineParameters2, [Self; 2])> {
        let plan = Self::line_intersection2_wide_plan_with_first_line_exact_dyadic_f64(
            first,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_point_from_wide_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-fused-point-wide-first-line-f64"
        );
        Some((
            ExactDyadicWideLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Retain binary64-line intersection coordinates over a four-limb determinant.
    ///
    /// Reuses the first line and interprets second endpoints exactly as binary
    /// rationals. Source coordinates/differences remain native words; determinants
    /// use four limbs and each affine coordinate numerator uses six. Returns wide
    /// parameter quotients and a deferred point without dividing or allocating
    /// Rational coordinates. The fixed carrier can be materialized later.
    ///
    /// None covers nonfinite/input-width rejection, zero determinant and checked
    /// intermediate failure. No segment selection or new topology/incidence proof
    /// is supplied. Subsequent quotient allocation is not bounded by these fixed
    /// fields. Success emits the wide retained-point first-line-f64 event.
    pub(crate) fn line_intersection2_retained_point_wide_with_first_line_exact_dyadic_f64(
        first: &ExactDyadicLine2,
        second_start: [f64; 2],
        second_end: [f64; 2],
    ) -> Option<(ExactDyadicWideLineParameters2, ExactDyadicWideLinePoint2)> {
        let plan = Self::line_intersection2_wide_plan_with_first_line_exact_dyadic_f64(
            first,
            second_start,
            second_end,
        )?;
        let point = Self::line_intersection2_retained_point_from_wide_plan(plan)?;
        crate::trace_dispatch!(
            "rational",
            "line-intersection2",
            "dyadic-stack-retained-point-wide-first-line-f64"
        );
        Some((
            ExactDyadicWideLineParameters2::from_words(plan.numerators, plan.denominator),
            point,
        ))
    }

    /// Materialize x/y from a caller-proved nonzero wide supporting-line plan.
    ///
    /// Each coordinate is (origin*determinant + direction*first_numerator) divided
    /// by determinant. Four-limb by native products accumulate in six-limb signed
    /// sums. None means a checked affine accumulation declined; any already built
    /// first coordinate is discarded if the second declines.
    ///
    /// Zero numerators become canonical zero. Nonzero quotients retain unreduced
    /// exact parts, so cancellation is delayed until observation. Trusts all plan
    /// signs, scales and defining relations without revalidating source incidence
    /// or segment bounds. Quotient shifts/allocation have no local resource budget.
    fn line_intersection2_point_from_wide_plan(
        plan: DyadicWideLineIntersectionPlan,
    ) -> Option<[Self; 2]> {
        let coordinate = |index: usize| {
            let affine = Self::product_sum_wide_narrow_words(
                [plan.denominator, plan.numerators[0]],
                [plan.first_start[index], plan.first_delta[index]],
                [true, true],
            )?;
            if affine.sign == NoSign {
                return Some(Self::zero());
            }
            Some(Self::quotient_dyadic_stack_sum_by_wide_unreduced(
                affine,
                plan.denominator,
            ))
        };
        Some([coordinate(0)?, coordinate(1)?])
    }

    /// Retain both affine coordinate numerators from a valid wide line plan.
    ///
    /// Computes origin*determinant + direction*first_numerator for x and then y
    /// using checked four-limb by native products and six-limb subtotals. None
    /// discards local partial work on a width/alignment decline. Zero numerators
    /// remain successful retained coordinates over the shared nonzero determinant.
    ///
    /// Copies determinant sign, magnitude and exponent without division, Rational
    /// construction or heap arithmetic. The caller proves plan consistency and
    /// nonzero determinant; no incidence or segment membership is re-established.
    /// Fixed retention does not bound later coordinate materialization costs.
    fn line_intersection2_retained_point_from_wide_plan(
        plan: DyadicWideLineIntersectionPlan,
    ) -> Option<ExactDyadicWideLinePoint2> {
        let coordinate = |index: usize| {
            Self::product_sum_wide_narrow_words(
                [plan.denominator, plan.numerators[0]],
                [plan.first_start[index], plan.first_delta[index]],
                [true, true],
            )
        };
        Some(ExactDyadicWideLinePoint2 {
            coordinate_numerators: [coordinate(0)?, coordinate(1)?],
            denominator_magnitude: plan.denominator.magnitude,
            denominator_shift: plan.denominator.denominator_shift,
            denominator_sign: plan.denominator.sign,
        })
    }

    /// Attempt materialized parameters and coordinates for two dyadic supporting lines.
    ///
    /// Plans compact determinants, then materializes first and second parameters
    /// before constructing x/y. Each parameter must pass native quotient limits:
    /// its exponent difference fits i32 and its reduced parts fit u128. None can
    /// therefore occur even when the retained-parameter point path succeeds.
    ///
    /// Also declines non-dyadic/source-width input, zero determinant or checked
    /// coordinate accumulation. No proper-crossing or segment-range proof is made.
    /// Parameter rationals are reduced; nonzero coordinate rationals may retain
    /// lazy reduction. Conversion and output allocations are unbudgeted; completed
    /// construction emits dyadic-stack-fused. No partial tuple is published.
    pub(crate) fn line_intersection2_known_dyadic(
        first_start: [&Self; 2],
        first_end: [&Self; 2],
        second_start: [&Self; 2],
        second_end: [&Self; 2],
    ) -> Option<(Self, Self, [Self; 2])> {
        let plan = Self::line_intersection2_plan_known_dyadic(
            first_start,
            first_end,
            second_start,
            second_end,
        )?;
        let (first_parameter, _) =
            Self::quotient_dyadic_words(plan.numerators[0], plan.denominator)?;
        let (second_parameter, _) =
            Self::quotient_dyadic_words(plan.numerators[1], plan.denominator)?;
        let point = Self::line_intersection2_point_from_plan(plan)?;
        crate::trace_dispatch!("rational", "line-intersection2", "dyadic-stack-fused");
        Some((first_parameter, second_parameter, point))
    }

    /// Materialize x/y from a caller-proved compact nonzero determinant plan.
    ///
    /// Forms each origin*determinant + direction*first_numerator first with native
    /// signed product arithmetic, then retries with six-limb subtotals if needed.
    /// A failed checked retry returns None. Zero becomes canonical zero; otherwise
    /// divides by the shared determinant without a general magnitude GCD, retaining
    /// exact lazy fraction parts. Coordinates are attempted in x/y order.
    ///
    /// Trusts plan signs, scales, determinant nonzeroness and source relations. It
    /// neither materializes parameter rationals nor tests segment bounds. A later
    /// coordinate decline discards an earlier coordinate; quotient allocation can
    /// exceed scratch widths and has no local resource or cancellation budget.
    fn line_intersection2_point_from_plan(plan: DyadicLineIntersectionPlan) -> Option<[Self; 2]> {
        let coordinate = |index: usize| {
            let left = [plan.first_start[index], plan.numerators[0]];
            let right = [plan.denominator, plan.first_delta[index]];
            let affine = Self::product_sum2_dyadic_words_word(left, right, [true, true])
                .map(Self::dyadic_word_stack_sum)
                .or_else(|| Self::product_sum_dyadic_words(left, right, [true, true]))?;
            if affine.sign == NoSign {
                return Some(Self::zero());
            }
            Some(Self::quotient_dyadic_stack_sum_unreduced(
                affine,
                plan.denominator.sign,
                plan.denominator.denominator_shift,
                plan.denominator.magnitude,
            ))
        };
        Some([coordinate(0)?, coordinate(1)?])
    }

    /// Retain compact-plan affine coordinate numerators without quotient construction.
    ///
    /// For x then y, attempts native origin*determinant + direction*first_numerator,
    /// then checked six-limb accumulation. None discards the partial retained state
    /// when neither schedule fits. Exact zero remains a successful numerator.
    /// Copies the shared determinant metadata into a fixed deferred point.
    ///
    /// No Rational or heap-backed quotient is materialized. Caller-proved plan
    /// invariants, nonzero determinant and defining relations are trusted rather
    /// than checked here; no segment selection is implied. Subsequent observation
    /// can allocate and reduce beyond this fixed retention envelope.
    fn line_intersection2_retained_point_from_plan(
        plan: DyadicLineIntersectionPlan,
    ) -> Option<ExactDyadicLinePoint2> {
        let coordinate = |index: usize| {
            let left = [plan.first_start[index], plan.numerators[0]];
            let right = [plan.denominator, plan.first_delta[index]];
            Self::product_sum2_dyadic_words_word(left, right, [true, true])
                .map(Self::dyadic_word_stack_sum)
                .or_else(|| Self::product_sum_dyadic_words(left, right, [true, true]))
        };
        Some(ExactDyadicLinePoint2 {
            coordinate_numerators: [coordinate(0)?, coordinate(1)?],
            denominator_magnitude: plan.denominator.magnitude,
            denominator_shift: plan.denominator.denominator_shift,
            denominator_sign: plan.denominator.sign,
        })
    }

    /// Construct t=numerator/denominator and the exact point origin+t*delta.
    ///
    /// The caller establishes dyadic inputs; this operation does not restrict t
    /// to a segment or require a nonzero direction. Canonicalizes all inputs first,
    /// rejects zero denominator with DivideByZero, then requires dyadic exponents
    /// for numerator and denominator even when numerator is zero. Active non-dyadic
    /// coordinate factors violate the proof and can panic; inactive terms need
    /// not be classified. No partial parameter/point result is returned on failure.
    ///
    /// Native quotient magnitudes permit reuse of their GCD. For an odd denominator
    /// magnitude D>1 and native delta magnitude, the affine divisor is
    /// g1*gcd(delta_magnitude,D/g1), where g1=gcd(numerator_magnitude,D).
    /// The origin contribution vanishes modulo D and binary normalization removes
    /// only invertible powers of two, justifying this reused cancellation proof.
    /// It applies only when those admission conditions hold, not to every input.
    ///
    /// Coordinates first try six-limb dyadic accumulation, otherwise use the exact
    /// arbitrary-precision sum. The odd-divisor path can divide a retained numerator
    /// directly; other paths materialize it and use exact quotient reduction.
    /// Zero affine numerators become canonical zero. The parameter and successful
    /// coordinates are reduced rationals, unlike the lazy line-point constructors.
    ///
    /// Dispatch traces distinguish native/wide quotient construction, stack sums
    /// and reused affine cancellation. Canonicalization, fallback and shifted
    /// output allocation have no local work, memory or cancellation budget.
    pub(crate) fn parameterized_point2_known_dyadic(
        origin: [&Self; 2],
        delta: [&Self; 2],
        numerator: &Self,
        denominator: &Self,
    ) -> Result<(Self, [Self; 2]), crate::Problem> {
        let origin = origin.map(Self::canonicalized_ref);
        let delta = delta.map(Self::canonicalized_ref);
        let numerator = numerator.canonicalized_ref();
        let denominator = denominator.canonicalized_ref();
        if denominator.sign == NoSign {
            return Err(crate::Problem::DivideByZero);
        }
        let denominator_shift = denominator
            .dyadic_denominator_shift()
            .expect("known-dyadic denominator has a power-of-two denominator");
        let numerator_shift = numerator
            .dyadic_denominator_shift()
            .expect("known-dyadic numerator has a power-of-two denominator");
        let word_plan = numerator
            .numerator
            .to_u128()
            .zip(denominator.numerator.to_u128())
            .map(|(numerator_word, denominator_word)| {
                (
                    denominator_word,
                    Self::gcd_word(numerator_word, denominator_word),
                )
            });

        let parameter = if let Some((denominator_word, first_divisor)) = word_plan {
            #[cfg(feature = "dispatch-trace")]
            let divisor_trace = BigUint::from(first_divisor);
            trace_rational_gcd!(
                &numerator.numerator,
                &denominator.numerator,
                &divisor_trace
            );
            let (result, _word_result) = Self::quotient_known_dyadic_with_word_divisor(
                numerator,
                denominator,
                numerator_shift,
                denominator_shift,
                denominator_word,
                first_divisor,
            );
            crate::trace_dispatch!(
                "rational",
                "div",
                if _word_result {
                    "known-dyadic-word-cross-cancel"
                } else {
                    "known-dyadic-cross-cancel"
                }
            );
            result
        } else {
            Self::quotient_known_dyadic(numerator, denominator)?
        };

        let coordinates = std::array::from_fn(|index| {
            let terms = [[origin[index], denominator], [numerator, delta[index]]];
            let signs = [
                Self::product_term_sign(true, terms[0]),
                Self::product_term_sign(true, terms[1]),
            ];
            let stack_sum = Self::dot_products_known_dyadic_stack_sum(
                [terms[0][0], terms[1][0]],
                [terms[0][1], terms[1][1]],
                signs,
            );
            if stack_sum.is_some() {
                crate::trace_dispatch!("rational", "product_sum", "known-dyadic-two-factor");
                crate::trace_dispatch!("rational", "dot_product", "dyadic-stack-accumulator");
            }
            if let Some(stack_sum) = stack_sum
                && stack_sum.sign != NoSign
                && let Some((denominator_word, first_divisor)) = word_plan
                && denominator_word > 1
                && denominator_word % 2 == 1
                && let Some(delta_word) = delta[index].numerator.to_u128()
            {
                let remaining_denominator = denominator_word / first_divisor;
                let second_divisor = Self::gcd_word(delta_word, remaining_denominator);
                let divisor = first_divisor * second_divisor;
                #[cfg(feature = "dispatch-trace")]
                let numerator_trace = stack_sum.magnitude.into_biguint();
                #[cfg(feature = "dispatch-trace")]
                let divisor_trace = BigUint::from(divisor);
                trace_rational_gcd!(
                    &numerator_trace,
                    &denominator.numerator,
                    &divisor_trace
                );
                let result = Self::quotient_dyadic_stack_sum_with_word_divisor(
                    stack_sum,
                    denominator.sign,
                    denominator_shift,
                    denominator_word,
                    divisor,
                );
                crate::trace_dispatch!(
                    "rational",
                    "div",
                    "known-dyadic-affine-factor-cross-cancel"
                );
                return Ok(result);
            }

            let affine_numerator = stack_sum.map_or_else(
                || Self::signed_product_sum_known_dyadic([true, true], terms),
                Self::materialize_dyadic_stack_sum,
            );
            if affine_numerator.sign == NoSign {
                return Ok(Self::zero());
            }
            if let Some((denominator_word, first_divisor)) = word_plan
                && denominator_word > 1
                && denominator_word % 2 == 1
                && let Some(delta_word) = delta[index].numerator.to_u128()
            {
                let remaining_denominator = denominator_word / first_divisor;
                let second_divisor = Self::gcd_word(delta_word, remaining_denominator);
                let divisor = first_divisor * second_divisor;
                let affine_shift = affine_numerator
                    .dyadic_denominator_shift()
                    .expect("known-dyadic affine numerator has a power-of-two denominator");
                #[cfg(feature = "dispatch-trace")]
                let divisor_trace = BigUint::from(divisor);
                trace_rational_gcd!(
                    &affine_numerator.numerator,
                    &denominator.numerator,
                    &divisor_trace
                );
                let (result, _) = Self::quotient_known_dyadic_with_word_divisor(
                    &affine_numerator,
                    denominator,
                    affine_shift,
                    denominator_shift,
                    denominator_word,
                    divisor,
                );
                crate::trace_dispatch!(
                    "rational",
                    "div",
                    "known-dyadic-affine-factor-cross-cancel"
                );
                Ok(result)
            } else {
                Self::quotient_known_dyadic(&affine_numerator, denominator)
            }
        });
        let [x, y] = coordinates;

        Ok((parameter, [x?, y?]))
    }

    /// Return a reduced exact sum of fixed-shape signed rational products.
    ///
    /// Each row is multiplied, then added for true polarity or subtracted for false.
    /// Empty products are one; an empty sum or all-zero terms return canonical zero.
    /// Accepts valid unreduced storage without canonicalizing every input first.
    /// Structural signs skip zero terms before denominator planning.
    ///
    /// A dyadic width plan selects native accumulation when useful; without such a
    /// plan, the general native reducer is attempted. Then handles a single live
    /// product, arbitrary-precision dyadic alignment, equal stored product
    /// denominators or an LCM schedule in that order. Checked native decline
    /// continues exact evaluation, rather than approximating or dropping terms.
    ///
    /// Delayed reduction avoids a Rational for each product and partial sum;
    /// denominator planning may still compute GCDs. Geometry consumers retain their
    /// determinant/cofactor shape while scalar storage and reduction stay here.
    /// For exactly two products of two factors, signed_product_sum2 tries the native
    /// schedule directly. Const-sized arrays grow with TERMS, and arbitrary integer
    /// growth/allocation has no local total-work, memory or cancellation budget.
    pub fn signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Self {
        // Short determinant and cofactor polynomials are exact rational sums of
        // products. As with `dot_products`, build one denominator and reduce
        // only the final row. This targets the trace rows where fixed 3x3/4x4
        // inverse, division, and negative-powi kernels still paid repeated gcd
        // work after dot products had already been fused. The algebraic
        // strategy delays fractions and keeps the public fixed-size cofactor
        // formulas division-free.
        // Structural note: keep this hook scalar-local. Hyperlattice can use it
        // for exact cofactor and determinant kernels, while predicate and
        // triangulation crates should consume only the resulting exact signs or
        // values through their own abstraction boundaries.
        // An empty product is one. Handle that identity before schedules
        // which inspect a factor or assume a nonempty denominator plan.
        if FACTORS == 0 {
            let positive = positive_terms.iter().filter(|sign| **sign).count();
            return Self::from_bigint(BigInt::from(positive) - BigInt::from(TERMS - positive));
        }
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        for i in 0..TERMS {
            // Term signs are pure structural facts from stored rational signs.
            // Compute them once and reuse them across dyadic, equal-denominator,
            // and LCM reducers. This preserves Bareiss-style delayed reduction
            // while removing repeated sign walks from exact cofactor/determinant
            // product sums.
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "product_sum", "all-zero");
            return Self::zero();
        }
        let dyadic_plan = Self::product_sum_dyadic_plan(terms, signs);
        let prefer_wide_dyadic = dyadic_plan
            .as_ref()
            .is_some_and(|plan| plan.prefer_wide);
        if !prefer_wide_dyadic {
            if let Some(plan) = dyadic_plan.as_ref()
                && let Some(word) = Self::signed_product_sum_dyadic_words_with_plan(
                    terms,
                    signs,
                    plan.denominator_shifts,
                    plan.max_shift,
                )
            {
                crate::trace_dispatch!("rational", "product_sum", "dyadic-word-accumulator");
                return word;
            }
            if dyadic_plan.is_none()
                && let Some(word) = Self::signed_product_sum_words(terms, signs)
            {
                crate::trace_dispatch!("rational", "product_sum", "word-sized");
                return word;
            }
        }
        if nonzero_count == 1 {
            for i in 0..TERMS {
                match signs[i] {
                    Plus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            Self::product_term_magnitude(terms[i]),
                            BigUint::ZERO,
                            denominator,
                        );
                    }
                    Minus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            BigUint::ZERO,
                            Self::product_term_magnitude(terms[i]),
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
        }

        if let Some(plan) = dyadic_plan {
            // Structural-dispatch note: callers that know coordinates are
            // lifted from a common binary grid could pass that grid exponent
            // with the terms and jump directly to this reducer, avoiding the
            // exploratory denominator scans used by generic exact rationals.
            crate::trace_dispatch!("rational", "product_sum", "dyadic-shared-denominator");
            return Self::signed_product_sum_dyadic_with_plan(
                terms,
                signs,
                plan.denominator_shifts,
                plan.max_shift,
            );
        }

        let mut denominators: [BigUint; TERMS] = std::array::from_fn(|_| BigUint::ZERO);
        let mut shared_denominator = None::<BigUint>;
        let mut equal_denominator = true;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            match &shared_denominator {
                None => shared_denominator = Some(denominator.clone()),
                Some(shared) if *shared == denominator => {}
                Some(_) => equal_denominator = false,
            }
            denominators[i] = denominator;
        }

        if equal_denominator {
            let denominator = shared_denominator.expect("nonzero product sum has denominator");
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..TERMS {
                let sign = signs[i];
                if sign == NoSign {
                    continue;
                }
                let magnitude = Self::product_term_magnitude(terms[i]);
                match sign {
                    Plus => positive += magnitude,
                    Minus => negative += magnitude,
                    NoSign => {}
                }
            }

            crate::trace_dispatch!("rational", "product_sum", "equal-product-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        crate::trace_dispatch!("rational", "product_sum", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            if denominator != ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, denominator);
                trace_rational_gcd!(&common_denominator, denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            let denominator = &denominators[i];
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    // Keep complete planning and arbitrary-precision reduction out of the
    // common exact word-accumulator dispatchers. This helper is deliberately
    // shared only after every direct path declines.
    /// Decide a signed product sum's ordering against zero through complete scalar schedules.
    ///
    /// The caller supplies current complete product signs for these exact operands.
    /// Zero terms are skipped; this helper can also handle empty or single-live-term
    /// input, although public dispatchers usually resolve those cases earlier.
    /// A dyadic width estimate governs native probing; absent dyadic admission,
    /// tries general native totals. Four two-factor terms additionally try the
    /// wide-by-native six-limb ordering schedule.
    ///
    /// Remaining paths compare arbitrary-precision positive/negative totals using
    /// dyadic alignment, equal stored product denominators or an LCM. Every common
    /// denominator is positive, so no output Rational or final fraction reduction
    /// is needed. LCM planning still computes GCDs. Valid lazy fraction storage is
    /// supported; native ineligibility preserves exact fallback. Arrays, integer
    /// growth and allocation have no local work/memory/cancellation bound.
    #[inline(never)]
    fn signed_product_sum_ordering_fallback<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Ordering {
        let dyadic_plan = Self::product_sum_dyadic_plan(terms, signs);
        let prefer_wide_dyadic = dyadic_plan
            .as_ref()
            .is_some_and(|plan| plan.prefer_wide);
        if !prefer_wide_dyadic {
            if let Some(plan) = dyadic_plan.as_ref()
                && let Some((positive, negative)) =
                    Self::signed_product_sum_dyadic_word_totals(
                        terms,
                        signs,
                        plan.denominator_shifts,
                        plan.max_shift,
                    )
            {
                crate::trace_dispatch!(
                    "rational",
                    "product_sum_ordering",
                    "dyadic-word-accumulator"
                );
                return positive.cmp(&negative);
            }
            if dyadic_plan.is_none()
                && let Some((positive, negative, _)) =
                    Self::signed_product_sum_word_totals(terms, signs)
            {
                crate::trace_dispatch!("rational", "product_sum_ordering", "word-sized");
                return positive.cmp(&negative);
            }
        }
        if TERMS == 4
            && FACTORS == 2
            && let Some(plan) = dyadic_plan.as_ref()
            && let Some(ordering) =
                Self::signed_product_sum_dyadic_wide_narrow_ordering_with_plan(
                    terms,
                    signs,
                    plan.denominator_shifts,
                    plan.max_shift,
                )
        {
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "dyadic-stack-accumulator"
            );
            return ordering;
        }

        if let Some(plan) = dyadic_plan {
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "arbitrary-precision-dyadic"
            );
            return Self::signed_product_sum_dyadic_ordering_with_plan(
                terms,
                signs,
                plan.denominator_shifts,
                plan.max_shift,
            );
        }

        let mut denominators: [BigUint; TERMS] = std::array::from_fn(|_| BigUint::ZERO);
        let mut shared_denominator = None::<BigUint>;
        let mut equal_denominator = true;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            match &shared_denominator {
                None => shared_denominator = Some(denominator.clone()),
                Some(shared) if *shared == denominator => {}
                Some(_) => equal_denominator = false,
            }
            denominators[i] = denominator;
        }

        if equal_denominator {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..TERMS {
                match signs[i] {
                    Plus => positive += Self::product_term_magnitude(terms[i]),
                    Minus => negative += Self::product_term_magnitude(terms[i]),
                    NoSign => {}
                }
            }
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "arbitrary-precision-equal-denominator"
            );
            return positive.cmp(&negative);
        }

        let mut common_denominator = BigUint::one();
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            if denominator != ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, denominator);
                trace_rational_gcd!(&common_denominator, denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!(
            "rational",
            "product_sum_ordering",
            "arbitrary-precision-lcm"
        );
        positive.cmp(&negative)
    }

    // Keep sign reconstruction and every complete arbitrary-width route out of
    // the common four- and six-product word accumulator. A declined word probe
    // reaches this helper with the original inputs, so no fallback capability
    // depends on partial fast-path state.
    /// Reconstruct complete signs before ordering after a direct accumulator declines.
    ///
    /// Builds all product signs from the original factors and polarities, then
    /// counts live terms only until finding two. No live term returns Equal; one
    /// returns its sign without multiplying magnitudes. Otherwise delegates to the
    /// full ordering fallback. Does not consume or depend on partial fast-path
    /// scratch state, nor require a separately supplied failure proof.
    ///
    /// The four/six-term public dispatch uses this cold helper to keep full planning
    /// outside its common native path. Generic empty products still have signed-one
    /// meaning. Sign storage grows with TERMS; fallback arithmetic is unbudgeted.
    #[inline(never)]
    fn signed_product_sum_ordering_after_unplanned_declines<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Ordering {
        let signs = std::array::from_fn(|i| Self::product_term_sign(positive_terms[i], terms[i]));
        let mut nonzero_count = 0_usize;
        let mut nonzero_index = 0_usize;
        for (index, sign) in signs.iter().copied().enumerate() {
            if sign != NoSign {
                nonzero_count += 1;
                nonzero_index = index;
            }
            if nonzero_count == 2 {
                break;
            }
        }
        match nonzero_count {
            0 => {
                crate::trace_dispatch!("rational", "product_sum_ordering", "all-zero");
                Ordering::Equal
            }
            1 => {
                crate::trace_dispatch!(
                    "rational",
                    "product_sum_ordering",
                    "single-term-product"
                );
                match signs[nonzero_index] {
                    Minus => Ordering::Less,
                    Plus => Ordering::Greater,
                    NoSign => unreachable!("the retained product term is nonzero"),
                }
            }
            _ => Self::signed_product_sum_ordering_fallback(terms, signs),
        }
    }

    /// Compare an exact fixed-shape signed product sum with zero without constructing it.
    ///
    /// Each product is added for true polarity and subtracted for false. Empty
    /// products are one, so zero-factor shapes compare positive and negative term
    /// counts; empty sums return Equal. Valid unreduced factors are supported.
    ///
    /// Four/six two-factor terms first try unplanned native dyadic totals; the
    /// four-term case also requires the fourth term's first stored numerator to
    /// fit u128 before that probe, even if the term later proves inactive. On
    /// decline, reconstructs signs and uses the complete ordering fallback. Other
    /// shapes compute signs first, resolving zero/one live term structurally before
    /// native, fixed-stack or arbitrary-precision denominator schedules.
    ///
    /// All decisions use exact integer signs and positive common scales, not float
    /// approximations or tolerance. No result Rational is materialized, but LCM/GCD
    /// work, large integer temporaries and const-sized arrays remain unbudgeted.
    /// Dispatch traces describe the chosen proof path, not a numeric qualification.
    pub fn signed_product_sum_ordering<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Ordering {
        if FACTORS == 0 {
            let positive = positive_terms.iter().filter(|sign| **sign).count();
            return positive.cmp(&(TERMS - positive));
        }
        // Expanded affine and 2D orientation determinants have four or six
        // pair products and overwhelmingly stay in the word-sized dyadic
        // envelope. Use the fourth affine factor as a conservative admission
        // guard; a rejected probe simply continues to the complete bit-width
        // plan. Compute signs inside this common path so its successful case
        // does not first materialize and rescan a separate sign array.
        if FACTORS == 2 && (TERMS == 4 || TERMS == 6) {
            if (TERMS == 6 || terms[3][0].numerator.to_u128().is_some())
                && let Some((positive, negative)) =
                Self::signed_product_sum_dyadic_word_totals_unplanned(positive_terms, terms)
            {
                crate::trace_dispatch!(
                    "rational",
                    "product_sum_ordering",
                    "dyadic-word-accumulator"
                );
                return positive.cmp(&negative);
            }
            return Self::signed_product_sum_ordering_after_unplanned_declines(
                positive_terms,
                terms,
            );
        }
        let signs = std::array::from_fn(|i| Self::product_term_sign(positive_terms[i], terms[i]));
        let mut nonzero_count = 0_usize;
        let mut nonzero_index = 0_usize;
        for (index, sign) in signs.iter().copied().enumerate() {
            if sign != NoSign {
                nonzero_count += 1;
                nonzero_index = index;
            }
            if nonzero_count == 2 {
                break;
            }
        }
        match nonzero_count {
            0 => {
                crate::trace_dispatch!("rational", "product_sum_ordering", "all-zero");
                return Ordering::Equal;
            }
            1 => {
                crate::trace_dispatch!("rational", "product_sum_ordering", "single-term-product");
                return match signs[nonzero_index] {
                    Minus => Ordering::Less,
                    Plus => Ordering::Greater,
                    NoSign => unreachable!("the retained product term is nonzero"),
                };
            }
            _ => {}
        }
        Self::signed_product_sum_ordering_fallback(terms, signs)
    }

    /// Compare a dynamically sized signed sum of two-factor products with zero exactly.
    ///
    /// Requires one polarity per term and panics on a length mismatch before any
    /// empty/all-zero shortcut. True adds and false subtracts. Empty or all-zero
    /// input is Equal; one live product is decided from its structural sign.
    /// Valid unreduced storage is accepted without mandatory input reduction.
    ///
    /// Allocates the complete sign vector, then for multiple live terms attempts
    /// checked native products, denominator LCM, scaling and unsigned subtotals.
    /// Any decline restarts from the original terms with BigUint denominator and
    /// magnitude schedules. Compares positive/negative totals directly, avoiding an
    /// output Rational and final fraction reduction; LCM construction still uses
    /// GCDs. This path serves data-dependent sums such as polygon area.
    ///
    /// Even native success can allocate vectors proportional to term count.
    /// General coefficient growth, work, memory and cancellation have no local
    /// budget; compact overflow does not weaken the exact ordering result.
    pub fn signed_product_sum2_ordering_slice(
        positive_terms: &[bool],
        terms: &[[&Self; 2]],
    ) -> Ordering {
        assert_eq!(
            positive_terms.len(),
            terms.len(),
            "each rational product term must have a sign"
        );

        let signs: Vec<_> = positive_terms
            .iter()
            .zip(terms)
            .map(|(&positive, &term)| Self::product_term_sign(positive, term))
            .collect();
        let mut nonzero = signs
            .iter()
            .copied()
            .enumerate()
            .filter(|(_, sign)| *sign != NoSign);
        let Some((_first_index, first_sign)) = nonzero.next() else {
            crate::trace_dispatch!("rational", "product_sum_ordering_slice", "all-zero");
            return Ordering::Equal;
        };
        if nonzero.next().is_none() {
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering_slice",
                "single-term-product"
            );
            return match first_sign {
                Minus => Ordering::Less,
                Plus => Ordering::Greater,
                NoSign => unreachable!("the retained product term is nonzero"),
            };
        }

        let mut word_terms = Vec::with_capacity(terms.len());
        let mut word_common_denominator = 1_u128;
        let mut word_path = true;
        for (&sign, &term) in signs.iter().zip(terms) {
            if sign == NoSign {
                word_terms.push((0_u128, 1_u128));
                continue;
            }
            let Some((magnitude, denominator)) = Self::product_term_words(term) else {
                word_path = false;
                break;
            };
            let divisor = Self::gcd_word(word_common_denominator, denominator);
            let Some(common) =
                word_common_denominator.checked_mul(denominator / divisor)
            else {
                word_path = false;
                break;
            };
            word_common_denominator = common;
            word_terms.push((magnitude, denominator));
        }
        if word_path {
            let mut positive = 0_u128;
            let mut negative = 0_u128;
            for ((magnitude, denominator), sign) in
                word_terms.into_iter().zip(signs.iter().copied())
            {
                if sign == NoSign {
                    continue;
                }
                let Some(scaled) =
                    magnitude.checked_mul(word_common_denominator / denominator)
                else {
                    word_path = false;
                    break;
                };
                let total = match sign {
                    Plus => &mut positive,
                    Minus => &mut negative,
                    NoSign => unreachable!("zero terms were skipped"),
                };
                let Some(next) = total.checked_add(scaled) else {
                    word_path = false;
                    break;
                };
                *total = next;
            }
            if word_path {
                crate::trace_dispatch!(
                    "rational",
                    "product_sum_ordering_slice",
                    "word-sized"
                );
                return positive.cmp(&negative);
            }
        }

        crate::trace_dispatch!(
            "rational",
            "product_sum_ordering_slice",
            "arbitrary-precision"
        );
        let mut denominators = Vec::with_capacity(terms.len());
        let mut common_denominator = BigUint::one();
        for (&sign, &term) in signs.iter().zip(terms) {
            if sign == NoSign {
                denominators.push(BigUint::one());
                continue;
            }
            let denominator = Self::product_term_denominator(term);
            let divisor =
                Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, &denominator);
            common_denominator *= &denominator / &divisor;
            denominators.push(denominator);
        }
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for ((&sign, &term), denominator) in signs.iter().zip(terms).zip(&denominators) {
            if sign == NoSign {
                continue;
            }
            let mut magnitude = Self::product_term_magnitude(term);
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => unreachable!("zero terms were skipped"),
            }
        }
        positive.cmp(&negative)
    }

    /// Return the exact sum of N pairwise rational products with delayed reduction.
    ///
    /// Derives complete structural product signs and skips zeros; an empty or
    /// all-zero dot returns canonical zero. Tries the general native product-sum
    /// reducer before the single-live-product shortcut, then dyadic native/stack/
    /// BigUint schedules, equal stored product denominators and finally an LCM.
    /// Valid unreduced operands remain exact without an initial canonicalization
    /// pass. Native decline continues general arithmetic rather than losing terms.
    ///
    /// Dense vector/matrix consumers share this scalar denominator authority and
    /// avoid constructing every product and partial-sum Rational. Output is reduced,
    /// while planning itself can perform GCD work. Const arrays and BigUint products,
    /// denominators and totals have no local work/memory/cancellation budget.
    /// Dispatch events expose the selected schedule, not an approximation policy.
    pub(crate) fn dot_products<const N: usize>(left: [&Self; N], right: [&Self; N]) -> Self {
        // Dense vector and matrix dot products are exact rational linear
        // forms when all inputs are rational. Build one shared denominator and
        // canonicalize only the final sum instead of reducing every product
        // and partial sum, delaying fractions until the end.
        // Historical mat4 powi measurements of this aggregate schedule reported
        // the following from-f64 trace changes, not bounds for every route:
        // 161.75 to 32 reductions/call and 462.25 to 67.75
        // temporaries/call; keep future changes within noise of those counts.
        let mut signs = [NoSign; N];
        let mut nonzero_count = 0_usize;
        for i in 0..N {
            let sign = left[i].sign * right[i].sign;
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "dot_product", "all-zero");
            return Self::zero();
        }
        let terms = std::array::from_fn(|i| [left[i], right[i]]);
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "dot_product", "word-sized");
            return word;
        }
        if nonzero_count == 1 {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..N {
                match signs[i] {
                    Plus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        positive = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    Minus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        negative = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
            return Self::zero();
        }

        if let Some(dyadic) = Self::dot_products_dyadic(left, right, signs) {
            // Binary64-derived dyadics are the hottest exact-rational matrix path.
            // A common power-of-two denominator lets us scale numerators with
            // shifts and lets `maybe_reduce` avoid a BigInt gcd.
            // Structural-dispatch note: matrix/vector callers with retained
            // grid-scale metadata can route straight here and reserve the LCM
            // path for genuinely mixed rational inputs.
            crate::trace_dispatch!("rational", "dot_product", "dyadic-shared-denominator");
            return dyadic;
        }
        if let Some(equal_denominator) = Self::dot_products_equal_denominator(left, right, signs) {
            // Decimal rational fixtures often enter with identical product
            // denominators even after exact parsing. The LCM algorithm below is
            // still the right general fallback, but this structural fact means
            // there is no LCM to build and no per-term scale division. 2026-05
            // tracing target: lower non-dyadic rational dot-product gcd counts
            // without perturbing the dyadic fast path above. Targeted
            // Criterion, 200 samples/8s: hyperlattice hyperreal-rational
            // mat3 powi improved 2.83%, mat4 div_matrix improved 3.88%,
            // mat3 inverse_checked and mat4 powi stayed within noise.
            crate::trace_dispatch!("rational", "dot_product", "equal-product-denominator");
            return equal_denominator;
        }

        crate::trace_dispatch!("rational", "dot_product", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        let mut any_nonzero = false;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            if denominator != *ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, &denominator);
                trace_rational_gcd!(&common_denominator, &denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
            any_nonzero = true;
        }
        if !any_nonzero {
            return Self::zero();
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if denominator != common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    /// Attempt an exact sum of squares through existing borrowed-arithmetic caches.
    ///
    /// Empty input or any zero coordinate returns None, leaving the ordinary
    /// aggregate path to the caller. For a dense row, inspects every primary
    /// self-product state. Existing products admit the row if all are squares or
    /// conflict admission succeeds. With no primary products, requires the leading
    /// coordinate's arithmetic-history observation to admit reuse.
    ///
    /// These observations can canonicalize lazy units or set retention bits even
    /// when the attempt returns None; this is scheduling, not a read-only numeric
    /// predicate. None does not deny existence of the exact squared norm. Consumer
    /// dot3/dot4 paths separately check that both operands are the same sources.
    ///
    /// On admission, computes borrowed squares, observes each product for future
    /// linear reuse, then adds four lanes as two pairs followed by their sum.
    /// Other nonempty sizes use a left fold. Preserving that call order preserves
    /// cache opportunities. Success emits retained-self-dot; repeated results can
    /// share storage, but cache availability is not guaranteed by numeric equality.
    /// Bounded cache slots do not bound retained graph bytes, arithmetic growth or
    /// allocation, and no local resource/cancellation budget is supplied.
    #[inline(never)]
    pub(crate) fn self_dot_if_reused<const N: usize>(values: [&Self; N]) -> Option<Self> {
        if values.iter().any(|value| value.sign == NoSign) {
            return None;
        }
        let (any_product, all_self_products) = values.iter().fold(
            (false, true),
            |(any_product, all_self_products), value| {
                let state = value.retained_primary_self_product_state();
                (any_product || state.is_some(), all_self_products && state == Some(true))
            },
        );
        if any_product {
            if !all_self_products && !Self::conflicted_self_dot_is_reusable(values) {
                return None;
            }
        } else if !values.first()?.has_arithmetic_reuse_evidence() {
            return None;
        }

        let products = values.map(|value| value * value);
        for product in &products {
            let _ = product.has_arithmetic_reuse_evidence();
        }
        let sum = if N == 4 {
            let left = &products[0] + &products[1];
            let right = &products[2] + &products[3];
            &left + &right
        } else {
            let mut products = products.into_iter();
            let mut sum = products.next()?;
            for product in products {
                sum = &sum + &product;
            }
            sum
        };
        crate::trace_dispatch!("rational", "dot_product", "retained-self-dot");
        Some(sum)
    }

    /// Admit a conflicted dense self-dot from retained squares or one retry observation.
    ///
    /// First checks for a retained square on every owner, short-circuiting at the
    /// first miss. If that fails, the leading owner atomically claims its one-shot
    /// conflict-attempt bit. Repeated claims decline until an owning-state transition
    /// clears that bit; this does not prove that a square or sum is already cached.
    ///
    /// The all-squares test is vacuously true for an empty array, although the self-dot
    /// caller never invokes this branch for an empty self-dot. Unit recognition
    /// can canonicalize and the retry can mutate retention metadata. Input identity
    /// controls cache scheduling, not mathematical equality or the norm's existence.
    #[inline(never)]
    fn conflicted_self_dot_is_reusable<const N: usize>(values: [&Self; N]) -> bool {
        values
            .iter()
            .all(|value| value.has_retained_self_product())
            || values
                .first()
                .is_some_and(|value| value.admit_conflicted_self_dot_once())
    }

}
