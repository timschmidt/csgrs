use crate::Problem;
use num::bigint::Sign::{self, *};
use num::{BigInt, BigUint, ToPrimitive, bigint::ToBigInt, bigint::ToBigUint};
use num::{One, Zero};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::sync::LazyLock;

pub(crate) mod convert;

/// Ratio of two integers
///
/// This type is functionally a [`Sign`] with a ratio between two [`BigUint`]
/// (the numerator and denominator). The numerator and denominator are finite.
///
/// The "ordinary" floating point numbers are rationals, but when converted
/// the exact rational may not be what you intuitively expected. It's obvious
/// that one third isn't represented exactly as an f64, but not everybody
/// will realise that 0.3 isn't either.
///
/// # Examples
///
/// Parsing a rational from a simple fraction
/// ```
/// use hyperreal::Rational;
/// let half: Rational = "9/18".parse().unwrap();
/// ```
///
/// Parsing a decimal fraction
/// ```
/// use hyperreal::Rational;
/// let point_two_five: Rational = "0.25".parse().unwrap();
/// ```
///
/// Converting a 64-bit floating point number
/// ```
/// use hyperreal::Rational;
/// let r: Rational = 0.3_f64.try_into().unwrap();
/// assert!(r != Rational::fraction(3, 10).unwrap());
/// ```
///
/// Simple arithmetic
/// ```
/// use hyperreal::Rational;
/// let quarter = Rational::fraction(1, 4).unwrap();
/// let eighteen = Rational::new(18);
/// let two = Rational::one() + Rational::one();
/// let sixteen = eighteen - two;
/// let four = quarter * sixteen;
/// assert_eq!(four, Rational::new(4));
/// ```

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Rational {
    sign: Sign,
    numerator: BigUint,
    denominator: BigUint,
}

static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
static FIVE: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&5).unwrap());
static TEN: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&10).unwrap());

macro_rules! trace_rational_temporary {
    () => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_temporary();
    }};
}

macro_rules! trace_rational_reduction {
    ($numerator:expr, $denominator:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_reduction($numerator, $denominator);
    }};
}

macro_rules! trace_rational_gcd {
    ($left:expr, $right:expr, $divisor:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_gcd($left, $right, $divisor);
    }};
}

macro_rules! trace_rational_power_of_two_common_factor {
    ($shift:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_power_of_two_common_factor($shift);
    }};
}

impl Rational {
    /// Zero, the additive identity.
    pub fn zero() -> Self {
        trace_rational_temporary!();
        Self {
            sign: NoSign,
            numerator: BigUint::ZERO,
            denominator: BigUint::one(),
        }
    }

    /// One, the multiplicative identity.
    pub fn one() -> Self {
        trace_rational_temporary!();
        Self {
            sign: Plus,
            numerator: BigUint::one(),
            denominator: BigUint::one(),
        }
    }

    /// The non-negative Rational corresponding to the provided [`i64`].
    pub fn new(n: i64) -> Self {
        // Identity rationals are pervasive in symbolic normalization. Build
        // them directly instead of routing through BigInt conversion and
        // reduction; benchmarks cover this path because higher crates construct
        // matrix identities and scalar ones constantly.
        match n {
            0 => Self::zero(),
            1 => Self::one(),
            -1 => {
                trace_rational_temporary!();
                Self {
                    sign: Minus,
                    numerator: BigUint::one(),
                    denominator: BigUint::one(),
                }
            }
            _ => Self::from_bigint(ToBigInt::to_bigint(&n).unwrap()),
        }
    }

    /// The Rational corresponding to the provided [`BigInt`].
    pub fn from_bigint(n: BigInt) -> Self {
        Self::from_bigint_fraction(n, BigUint::one()).unwrap()
    }

    /// The non-negative Rational corresponding to the provided [`i64`]
    /// numerator and [`u64`] denominator as a fraction.
    pub fn fraction(n: i64, d: u64) -> Result<Self, Problem> {
        let numerator = ToBigInt::to_bigint(&n).unwrap();
        let denominator = ToBigUint::to_biguint(&d).unwrap();
        Self::from_bigint_fraction(numerator, denominator)
    }

    /// The Rational corresponding to the provided [`BigInt`]
    /// numerator and [`BigUint`] denominator as a fraction.
    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Result<Self, Problem> {
        if denominator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        trace_rational_temporary!();
        let answer = Self {
            sign,
            numerator,
            denominator,
        };
        Ok(answer.reduce())
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
            Self {
                sign: self.sign,
                numerator: self.numerator / &divisor,
                denominator: self.denominator / divisor,
            }
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
            let numerator = self.numerator / &divisor;
            let denominator = self.denominator / &divisor;
            trace_rational_temporary!();
            Self {
                sign: self.sign,
                numerator,
                denominator,
            }
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
        Self {
            sign: self.sign,
            numerator: self.numerator >> shift,
            denominator: self.denominator >> shift,
        }
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
        if self.numerator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        Ok(Self {
            sign: self.sign,
            numerator: self.denominator,
            denominator: self.numerator,
        })
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

    fn dyadic_denominator_shift(&self) -> Option<u64> {
        Self::biguint_power_of_two_shift(&self.denominator)
    }

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
        trace_rational_temporary!();
        Self {
            sign,
            numerator,
            denominator,
        }
        .maybe_reduce()
    }

    fn dot_products_dyadic<const N: usize>(left: [&Self; N], right: [&Self; N]) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; N];
        let mut any_nonzero = false;
        for i in 0..N {
            if left[i].sign * right[i].sign == NoSign {
                continue;
            }
            let shift =
                left[i].dyadic_denominator_shift()? + right[i].dyadic_denominator_shift()?;
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = left[i].sign * right[i].sign;
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

    fn dot_products_equal_denominator<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
    ) -> Option<Self> {
        let mut shared_denominator = None::<BigUint>;
        for i in 0..N {
            if left[i].sign * right[i].sign == NoSign {
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
            let sign = left[i].sign * right[i].sign;
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

    fn product_term_denominator<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut denominator = BigUint::one();
        for factor in term {
            denominator *= &factor.denominator;
        }
        denominator
    }

    fn product_term_magnitude<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut magnitude = BigUint::one();
        for factor in term {
            magnitude *= &factor.numerator;
        }
        magnitude
    }

    fn product_term_sign<const FACTORS: usize>(positive: bool, term: [&Self; FACTORS]) -> Sign {
        let mut sign = if positive { Plus } else { Minus };
        for factor in term {
            sign = sign * factor.sign;
        }
        sign
    }

    fn signed_product_sum_dyadic<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut any_nonzero = false;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                continue;
            }
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift += factor.dyadic_denominator_shift()?;
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
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
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn signed_product_sum_equal_denominator<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<Self> {
        let mut shared_denominator = None::<BigUint>;
        for i in 0..TERMS {
            if Self::product_term_sign(positive_terms[i], terms[i]) == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
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
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
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

        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    pub(crate) fn signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Self {
        // Short determinant and cofactor polynomials are exact rational sums of
        // products. As with `dot_products`, build one denominator and reduce
        // only the final row. This targets the trace rows where fixed 3x3/4x4
        // inverse, division, and negative-powi kernels still paid repeated gcd
        // work after dot products had already been fused. The algebraic
        // strategy follows the same fraction-delay idea as Bareiss fraction
        // free elimination (Bareiss, Math. Comp. 22(103), 1968,
        // https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/S0025-5718-1968-0226829-0.pdf)
        // and common-factor exact matrix work
        // (https://link.springer.com/article/10.1007/s11786-020-00495-9),
        // but keeps the public fixed-size cofactor formulas division-free.
        // 2026-05-09 targeted Criterion, 200 samples/8s: the realistic_blas
        // opt-in hook kept approximate matrix reciprocal/division rows at
        // 80-194 ns or better and kept borrowed approximate division unchanged,
        // while hyperreal-rational mat3/mat4 reciprocal held near 26.0/44.1 us
        // after the first fused run reported roughly 20-27% faster reciprocal
        // rows against the pre-hook baseline. Treat those as regression guards.
        debug_assert!(FACTORS > 0);
        if let Some(dyadic) = Self::signed_product_sum_dyadic(positive_terms, terms) {
            crate::trace_dispatch!("rational", "product_sum", "dyadic-shared-denominator");
            return dyadic;
        }
        if let Some(equal_denominator) =
            Self::signed_product_sum_equal_denominator(positive_terms, terms)
        {
            crate::trace_dispatch!("rational", "product_sum", "equal-product-denominator");
            return equal_denominator;
        }

        crate::trace_dispatch!("rational", "product_sum", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        let mut any_nonzero = false;
        for i in 0..TERMS {
            if Self::product_term_sign(positive_terms[i], terms[i]) == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            if denominator != *ONE.deref() {
                let divisor = num::Integer::gcd(&common_denominator, &denominator);
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
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            let mut magnitude = Self::product_term_magnitude(terms[i]);
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

    pub(crate) fn dot_products<const N: usize>(left: [&Self; N], right: [&Self; N]) -> Self {
        // Dense vector and matrix dot products are exact rational linear
        // forms when all inputs are rational. Build one shared denominator and
        // canonicalize only the final sum instead of reducing every product
        // and partial sum. This is the same "delay fractions until the end"
        // idea used by fraction-free exact linear algebra; see Bareiss-style
        // exact division/common-factor discussion in
        // https://link.springer.com/article/10.1007/s11786-020-00495-9 and
        // the pedagogical Gauss-Jordan fraction-delay variant at
        // https://openjournals.libs.uga.edu/tme/article/view/1957/1862.
        // Current 2026-05 trace goal: keep exact matrix rows at one rational
        // constructor per output cell. This dropped mat4 powi from-f64 trace
        // activity from 161.75 to 32 reductions/call and from 462.25 to 67.75
        // temporaries/call; keep future changes within noise of those counts.
        if let Some(dyadic) = Self::dot_products_dyadic(left, right) {
            // Dyadic f64 imports are the hottest exact-rational matrix path.
            // A common power-of-two denominator lets us scale numerators with
            // shifts and lets `maybe_reduce` avoid a BigInt gcd.
            crate::trace_dispatch!("rational", "dot_product", "dyadic-shared-denominator");
            return dyadic;
        }
        if let Some(equal_denominator) = Self::dot_products_equal_denominator(left, right) {
            // Decimal rational fixtures often enter with identical product
            // denominators even after exact parsing. The LCM algorithm below is
            // still the right general fallback, but this structural fact means
            // there is no LCM to build and no per-term scale division. 2026-05
            // tracing target: lower non-dyadic rational dot-product gcd counts
            // without perturbing the dyadic fast path above. Targeted
            // Criterion, 200 samples/8s: realistic_blas hyperreal-rational
            // mat3 powi improved 2.83%, mat4 div_matrix improved 3.88%,
            // mat3 inverse_checked and mat4 powi stayed within noise.
            crate::trace_dispatch!("rational", "dot_product", "equal-product-denominator");
            return equal_denominator;
        }

        crate::trace_dispatch!("rational", "dot_product", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        let mut any_nonzero = false;
        for i in 0..N {
            if left[i].sign * right[i].sign == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            if denominator != *ONE.deref() {
                let divisor = num::Integer::gcd(&common_denominator, &denominator);
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
            let sign = left[i].sign * right[i].sign;
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

    #[inline]
    pub(crate) fn is_one(&self) -> bool {
        self.sign == Plus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
    }

    #[inline]
    pub(crate) fn is_minus_one(&self) -> bool {
        self.sign == Minus && self.numerator == *ONE.deref() && self.denominator == *ONE.deref()
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

    pub(crate) fn to_f64_approx(&self) -> Option<f64> {
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

    /// Either the corresponding [`BigInt`] or None if this value is not an integer.
    pub fn to_big_integer(&self) -> Option<BigInt> {
        let whole = &self.numerator / &self.denominator;
        let round = &whole * &self.denominator;
        if self.numerator == round {
            debug_assert!(self.denominator == *ONE.deref());
            Some(BigInt::from_biguint(self.sign, whole))
        } else {
            debug_assert!(self.denominator != *ONE.deref());
            None
        }
    }

    /// The [`Sign`] of this value.
    #[inline]
    pub fn sign(&self) -> Sign {
        self.sign
    }

    const EXTRACT_SQUARE_MAX_LEN: u64 = 5000;

    fn make_squares() -> Vec<(BigUint, BigUint)> {
        // Tiny prime-square table covers the residuals that appear most often
        // in exact trig and matrix examples without running full factorization.
        vec![
            (
                ToBigUint::to_biguint(&2).unwrap(),
                ToBigUint::to_biguint(&4).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&3).unwrap(),
                ToBigUint::to_biguint(&9).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&5).unwrap(),
                ToBigUint::to_biguint(&25).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&7).unwrap(),
                ToBigUint::to_biguint(&49).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&11).unwrap(),
                ToBigUint::to_biguint(&121).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&13).unwrap(),
                ToBigUint::to_biguint(&169).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&17).unwrap(),
                ToBigUint::to_biguint(&289).unwrap(),
            ),
        ]
    }

    // Some(root) squared is n, otherwise None
    fn try_perfect(n: BigUint) -> Option<BigUint> {
        // BigUint::sqrt is cheap enough as a final check once small square
        // factors have been stripped.
        let root = n.sqrt();
        let square = &root * &root;
        if square == n { Some(root) } else { None }
    }

    // (root squared times rest) = n
    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        static SQUARES: LazyLock<Vec<(BigUint, BigUint)>> = LazyLock::new(Rational::make_squares);

        // Partial square extraction is a performance shortcut, not full prime
        // factorization. It peels common small squares and then tests a few
        // residual divisors so sqrt simplification remains bounded.
        let one: BigUint = One::one();
        let mut root = one.clone();
        let mut rest = n;
        if rest.bits() > Self::EXTRACT_SQUARE_MAX_LEN {
            return (root, rest);
        }
        for (p, s) in &*SQUARES {
            if rest == one {
                break;
            }
            while (&rest % s).is_zero() {
                rest /= s;
                root *= p;
            }
        }

        let divisors = if rest.bit(0) {
            // Odd number so dividing by an even number won't get a whole result
            [1, 3, 5, 7, 11, 13, 15, 17, 19]
        } else {
            [1, 2, 3, 5, 6, 7, 8, 10, 11]
        };

        for n in divisors {
            let divisor = ToBigUint::to_biguint(&n).unwrap();
            if rest == divisor {
                return (root, rest);
            }
            if (&rest % &divisor).is_zero() {
                let square = &rest / &divisor;
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
        let (nroot, nrest) = Self::extract_square(self.numerator);
        let (droot, drest) = Self::extract_square(self.denominator);
        (
            Self {
                sign: Plus,
                numerator: nroot,
                denominator: droot,
            },
            Self {
                sign: self.sign,
                numerator: nrest,
                denominator: drest,
            },
        )
    }

    /// For very big rationals, the algorithm used for calculating a square
    /// root is not viable, in this case the predicate is false.
    pub fn extract_square_will_succeed(&self) -> bool {
        self.numerator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
            && self.denominator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
    }

    // This could grow unreasonably in terms of object size
    // so only call this for modest exp values
    fn pow_up(&self, exp: &BigUint) -> Self {
        if exp == &BigUint::ZERO {
            return Self::one();
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

    /// Integer exponeniation. Raise this Rational to an integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        const TOO_MANY_BITS: u64 = 1000;
        // Arguably wrong if self is also zero
        if exp == BigInt::ZERO {
            return Ok(Self::one());
        }
        if self.sign == NoSign {
            return Ok(Self::zero());
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

impl AsRef<Rational> for Rational {
    fn as_ref(&self) -> &Rational {
        self
    }
}

use core::fmt;

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.denominator == *ONE.deref() {
            let int = self.numerator.to_string();
            return f.pad_integral(self.sign != Minus, "", &int);
        }

        if self.sign == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            f.write_str("+")?;
        }
        if f.alternate() {
            let whole = &self.numerator / &self.denominator;
            write!(f, "{whole}.")?;
            let round = &whole * &self.denominator;
            let mut left = &self.numerator - &round;
            let mut digits = f.precision().unwrap_or(1000);
            loop {
                left *= &*TEN;
                let digit = &left / &self.denominator;
                write!(f, "{digit}")?;
                left -= digit * &self.denominator;
                if left.is_zero() {
                    break;
                }
                digits -= 1;
                if digits == 0 {
                    break;
                }
            }
            Ok(())
        } else {
            let whole = &self.numerator / &self.denominator;
            let round = &whole * &self.denominator;
            let left = &self.numerator - &round;
            if whole.is_zero() {
                write!(f, "{left}/{}", self.denominator)
            } else {
                write!(f, "{whole} {left}/{}", self.denominator)
            }
        }
    }
}

impl std::str::FromStr for Rational {
    type Err = Problem;

    fn from_str(s: &str) -> Result<Self, Problem> {
        let mut sign: Sign = Plus;
        let s = match s.strip_prefix('-') {
            Some(s) => {
                sign = Minus;
                s
            }
            None => s,
        };
        if let Some((n, d)) = s.split_once('/') {
            let numerator = BigUint::parse_bytes(n.as_bytes(), 10).ok_or(Problem::BadFraction)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self {
                sign,
                numerator,
                denominator: BigUint::parse_bytes(d.as_bytes(), 10).ok_or(Problem::BadFraction)?,
            })
        } else if let Some((i, d)) = s.split_once('.') {
            let numerator = BigUint::parse_bytes(i.as_bytes(), 10).ok_or(Problem::BadDecimal)?;
            let whole = if numerator.is_zero() {
                Self {
                    sign: NoSign,
                    numerator,
                    denominator: One::one(),
                }
            } else {
                Self {
                    sign,
                    numerator,
                    denominator: One::one(),
                }
            };
            let numerator = BigUint::parse_bytes(d.as_bytes(), 10).ok_or(Problem::BadDecimal)?;
            if numerator.is_zero() {
                return Ok(whole);
            }
            let denominator = TEN.pow(d.len() as u32);
            let fraction = Self {
                sign,
                numerator,
                denominator,
            };
            Ok(whole + fraction)
        } else {
            let numerator = BigUint::parse_bytes(s.as_bytes(), 10).ok_or(Problem::BadInteger)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self {
                sign,
                numerator,
                denominator: One::one(),
            })
        }
    }
}

use core::ops::*;

impl<T: AsRef<Rational>> Add<T> for &Rational {
    type Output = Rational;

    fn add(self, other: T) -> Self::Output {
        use std::cmp::Ordering::*;

        let other = other.as_ref();
        if self.sign == NoSign {
            return other.clone();
        }
        if other.sign == NoSign {
            return self.clone();
        }

        let common_denominator = num::Integer::gcd(&self.denominator, &other.denominator);
        trace_rational_gcd!(&self.denominator, &other.denominator, &common_denominator);
        let left_scale = &other.denominator / &common_denominator;
        let right_scale = &self.denominator / &common_denominator;
        let denominator = &self.denominator * &left_scale;
        let a = &self.numerator * &left_scale;
        let b = &other.numerator * &right_scale;
        let (sign, numerator) = match (self.sign, other.sign) {
            (Plus, Plus) => (Plus, a + b),
            (Minus, Minus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Greater => (x, a - b),
                Equal => {
                    return Self::Output::zero();
                }
                Less => (y, b - a),
            },
        };
        trace_rational_temporary!();
        Self::Output {
            sign,
            numerator,
            denominator,
        }
        .reduce_with_possible_divisor(&common_denominator)
    }
}

impl<T: AsRef<Rational>> Add<T> for Rational {
    type Output = Self;

    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

impl Neg for &Rational {
    type Output = Rational;

    fn neg(self) -> Self::Output {
        trace_rational_temporary!();
        let mut ret = self.clone();
        ret.sign = -ret.sign;
        ret
    }
}

impl Neg for Rational {
    type Output = Self;

    fn neg(mut self) -> Self {
        self.sign = -self.sign;
        self
    }
}

impl<T: AsRef<Rational>> Sub<T> for &Rational {
    type Output = Rational;

    fn sub(self, other: T) -> Self::Output {
        use std::cmp::Ordering::*;

        let other = other.as_ref();
        if other.sign == NoSign {
            return self.clone();
        }
        if self.sign == NoSign {
            return -other;
        }

        let common_denominator = num::Integer::gcd(&self.denominator, &other.denominator);
        trace_rational_gcd!(&self.denominator, &other.denominator, &common_denominator);
        let left_scale = &other.denominator / &common_denominator;
        let right_scale = &self.denominator / &common_denominator;
        let denominator = &self.denominator * &left_scale;
        let a = &self.numerator * &left_scale;
        let b = &other.numerator * &right_scale;
        let (sign, numerator) = match (self.sign, other.sign) {
            (Plus, Minus) => (Plus, a + b),
            (Minus, Plus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Greater => (x, a - b),
                Equal => {
                    return Self::Output::zero();
                }
                Less => (-y, b - a),
            },
        };
        trace_rational_temporary!();
        Self::Output {
            sign,
            numerator,
            denominator,
        }
        .reduce_with_possible_divisor(&common_denominator)
    }
}

impl<T: AsRef<Rational>> Sub<T> for Rational {
    type Output = Self;

    fn sub(self, other: T) -> Self {
        &self - other.as_ref()
    }
}

impl<T: AsRef<Rational>> Mul<T> for &Rational {
    type Output = Rational;

    fn mul(self, other: T) -> Self::Output {
        let other = other.as_ref();
        let sign = self.sign * other.sign;
        let numerator = &self.numerator * &other.numerator;
        let denominator = &self.denominator * &other.denominator;
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output {
            sign,
            numerator,
            denominator,
        })
    }
}

impl<T: AsRef<Rational>> Mul<T> for Rational {
    type Output = Self;

    fn mul(self, other: T) -> Self {
        &self * other.as_ref()
    }
}

impl<T: AsRef<Rational>> MulAssign<T> for Rational {
    fn mul_assign(&mut self, other: T) {
        *self = &*self * other.as_ref();
    }
}

impl<T: AsRef<Rational>> Div<T> for &Rational {
    type Output = Rational;

    fn div(self, other: T) -> Self::Output {
        let other = other.as_ref();
        assert_ne!(other.numerator, BigUint::ZERO);
        let sign = self.sign * other.sign;
        let numerator = &self.numerator * &other.denominator;
        let denominator = &self.denominator * &other.numerator;
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output {
            sign,
            numerator,
            denominator,
        })
    }
}

impl<T: AsRef<Rational>> Div<T> for Rational {
    type Output = Self;

    fn div(self, other: T) -> Self {
        &self / other.as_ref()
    }
}

impl Rational {
    fn definitely_equal(&self, other: &Self) -> bool {
        if self.sign != other.sign {
            return false;
        }
        if self.denominator != other.denominator {
            return false;
        }
        self.numerator == other.numerator
    }
}

impl PartialEq for Rational {
    fn eq(&self, other: &Self) -> bool {
        if self.sign != other.sign {
            return false;
        }
        if self.denominator == other.denominator {
            self.numerator == other.numerator
        } else {
            Self::definitely_equal(&self.clone().reduce(), &other.clone().reduce())
        }
    }
}

impl PartialOrd for Rational {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering::*;
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
        } else {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display() {
        let many: Rational = "12345".parse().unwrap();
        let s = format!("{many}");
        assert_eq!(s, "12345");
        let five: Rational = "5".parse().unwrap();
        let third: Rational = "1/3".parse().unwrap();
        let s = format!("{}", five * third);
        assert_eq!(s, "1 2/3");
    }

    #[test]
    fn decimals() {
        let first: Rational = "0.0".parse().unwrap();
        assert_eq!(first, Rational::zero());
        let a: Rational = "0.4".parse().unwrap();
        let b: Rational = "2.5".parse().unwrap();
        let answer = a * b;
        assert_eq!(answer, Rational::one());
    }

    #[test]
    /// See e.g. https://discussions.apple.com/thread/252474975
    /// Apple calculator is not trustworthy if you are a programmer
    fn parse() {
        let big: Rational = "288230376151711743".parse().unwrap();
        let small: Rational = "45".parse().unwrap();
        let expected: Rational = "12970366926827028435".parse().unwrap();
        assert_eq!(big * small, expected);
    }

    #[test]
    fn parse_fractions() {
        let third: Rational = "1/3".parse().unwrap();
        let minus_four: Rational = "-4".parse().unwrap();
        let twelve: Rational = "12/20".parse().unwrap();
        let answer = third + minus_four * twelve;
        let expected: Rational = "-31/15".parse().unwrap();
        assert_eq!(answer, expected);
    }

    #[test]
    fn square_reduced() {
        let thirty_two = Rational::new(32);
        let (square, rest) = thirty_two.extract_square_reduced();
        let four = Rational::new(4);
        assert_eq!(square, four);
        let two = Rational::new(2);
        assert_eq!(rest, two);
        let minus_one = Rational::new(-1);
        let (square, rest) = minus_one.clone().extract_square_reduced();
        assert_eq!(square, Rational::one());
        assert_eq!(rest, minus_one);
    }

    #[test]
    fn signs() {
        let half: Rational = "4/8".parse().unwrap();
        let one = Rational::one();
        let minus_half = half - one;
        let two = Rational::new(2);
        let zero = Rational::zero();
        let minus_two = zero - two;
        let i2 = minus_two.inverse().unwrap();
        assert_eq!(i2, minus_half);
    }

    #[test]
    fn half_plus_one_times_two() {
        let two = Rational::new(2);
        let half = two.inverse().unwrap();
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        let sum = half + one;
        assert_eq!(sum * two, three);
    }

    #[test]
    fn three_divided_by_six() {
        let three = Rational::new(3);
        let six = Rational::new(6);
        let half: Rational = "1/2".parse().unwrap();
        assert_eq!(three / six, half);
    }

    #[test]
    fn one_plus_two() {
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(one + two, three);
    }

    #[test]
    fn two_minus_one() {
        let two = Rational::new(2);
        let one = Rational::one();
        assert_eq!(two - one, Rational::one());
    }

    #[test]
    fn two_times_three() {
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(two * three, Rational::new(6));
    }

    #[test]
    fn fract() {
        let seventy_ninths = Rational::fraction(70, 9).unwrap();
        assert_eq!(seventy_ninths.fract(), Rational::fraction(7, 9).unwrap());
        assert_eq!(
            seventy_ninths.neg().fract(),
            Rational::fraction(-7, 9).unwrap()
        );
        let six = Rational::new(6);
        assert_eq!(six.fract(), Rational::zero());
    }

    #[test]
    fn trunc() {
        let seventy_ninths = Rational::fraction(70, 9).unwrap();
        let whole = seventy_ninths.trunc();
        let frac = seventy_ninths.fract();
        assert_eq!(whole + frac, seventy_ninths);
        let shrink = Rational::fraction(-405, 11).unwrap();
        let whole = shrink.trunc();
        let frac = shrink.fract();
        assert_eq!(whole + frac, shrink);
        let zero = Rational::zero();
        let whole = zero.trunc();
        let frac = zero.fract();
        assert_eq!(whole, frac);
        assert_eq!(whole + frac, zero);
    }

    #[test]
    fn power() {
        let one_two_five = Rational::new(5).powi(ToBigInt::to_bigint(&-3).unwrap());
        assert_eq!(one_two_five, Rational::fraction(1, 125));
        let more = Rational::new(7).powi(11i32.into()).unwrap();
        assert_eq!(more, Rational::new(1_977_326_743));
    }

    #[test]
    fn sqrt_trouble() {
        for (n, root, rest) in [
            (1, 1, 1),
            (2, 1, 2),
            (3, 1, 3),
            (4, 2, 1),
            (16, 4, 1),
            (400, 20, 1),
            (1323, 21, 3),
            (4761, 69, 1),
            (123456, 8, 1929),
            (715716, 846, 1),
        ] {
            let n = Rational::new(n);
            let reduced = n.extract_square_reduced();
            assert_eq!(reduced, (Rational::new(root), Rational::new(rest)));
        }
    }

    #[test]
    fn decimal() {
        let decimal: Rational = "7.125".parse().unwrap();
        assert!(!decimal.prefer_fraction());
        let half: Rational = "4/8".parse().unwrap();
        assert!(!half.prefer_fraction());
        let third: Rational = "2/6".parse().unwrap();
        assert!(third.prefer_fraction());
    }

    #[test]
    fn power_of_two_shift_detects_only_power_of_two_ratios() {
        assert_eq!(
            Rational::fraction(8, 1).unwrap().power_of_two_shift(),
            Some((3, Plus))
        );
        assert_eq!(
            Rational::fraction(1, 8).unwrap().power_of_two_shift(),
            Some((-3, Plus))
        );
        assert_eq!(
            Rational::fraction(-4, 32).unwrap().power_of_two_shift(),
            Some((-3, Minus))
        );
        assert_eq!(Rational::fraction(7, 8).unwrap().power_of_two_shift(), None);
        assert_eq!(Rational::fraction(5, 6).unwrap().power_of_two_shift(), None);
        assert_eq!(Rational::zero().power_of_two_shift(), None);
    }

    #[test]
    fn magnitude_at_least_power_of_two_handles_threshold_boundaries() {
        assert!(
            !Rational::fraction(7, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(8, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(-9, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            !Rational::fraction(15, 2)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(16, 2)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(!Rational::zero().magnitude_at_least_power_of_two(3));
    }

    #[test]
    fn dyadic_add_sub_stay_reduced() {
        let three_eighths = Rational::fraction(3, 8).unwrap();
        let five_sixteenths = Rational::fraction(5, 16).unwrap();

        assert_eq!(
            &three_eighths + &five_sixteenths,
            Rational::fraction(11, 16).unwrap()
        );
        assert_eq!(
            &three_eighths - &five_sixteenths,
            Rational::fraction(1, 16).unwrap()
        );
        assert_eq!(
            &five_sixteenths - &three_eighths,
            Rational::fraction(-1, 16).unwrap()
        );
        assert_eq!(&three_eighths - &three_eighths, Rational::zero());
    }

    #[test]
    fn dot_products_match_pairwise_arithmetic() {
        let left = [
            Rational::fraction(3, 8).unwrap(),
            Rational::fraction(-5, 16).unwrap(),
            Rational::zero(),
            Rational::fraction(7, 10).unwrap(),
        ];
        let right = [
            Rational::fraction(11, 32).unwrap(),
            Rational::fraction(13, 64).unwrap(),
            Rational::fraction(17, 19).unwrap(),
            Rational::fraction(-23, 25).unwrap(),
        ];
        let expected = &(&left[0] * &right[0])
            + &(&left[1] * &right[1])
            + &(&left[2] * &right[2])
            + &(&left[3] * &right[3]);

        assert_eq!(
            Rational::dot_products(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ),
            expected
        );
    }

    #[test]
    fn dot_products_preserve_dyadic_exactness() {
        let left = [
            Rational::fraction(1, 8).unwrap(),
            Rational::fraction(3, 16).unwrap(),
            Rational::fraction(-5, 32).unwrap(),
        ];
        let right = [
            Rational::fraction(7, 4).unwrap(),
            Rational::fraction(-11, 8).unwrap(),
            Rational::fraction(13, 16).unwrap(),
        ];

        let dot = Rational::dot_products(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
        );
        assert!(dot.is_dyadic());
        assert_eq!(
            dot,
            &(&left[0] * &right[0]) + &(&left[1] * &right[1]) + &(&left[2] * &right[2])
        );
    }

    #[test]
    fn dot_products_handle_equal_non_dyadic_denominators() {
        let left = [
            Rational::fraction(7, 10).unwrap(),
            Rational::fraction(-9, 10).unwrap(),
            Rational::fraction(11, 10).unwrap(),
        ];
        let right = [
            Rational::fraction(13, 7).unwrap(),
            Rational::fraction(5, 7).unwrap(),
            Rational::fraction(-3, 7).unwrap(),
        ];

        assert_eq!(
            Rational::dot_products(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            ),
            &(&left[0] * &right[0]) + &(&left[1] * &right[1]) + &(&left[2] * &right[2])
        );
    }

    #[test]
    fn signed_product_sum_matches_pairwise_arithmetic() {
        let terms = [
            [
                Rational::fraction(3, 8).unwrap(),
                Rational::fraction(-5, 12).unwrap(),
                Rational::fraction(7, 11).unwrap(),
            ],
            [
                Rational::fraction(13, 9).unwrap(),
                Rational::fraction(17, 25).unwrap(),
                Rational::fraction(-19, 6).unwrap(),
            ],
            [
                Rational::fraction(-23, 10).unwrap(),
                Rational::fraction(29, 14).unwrap(),
                Rational::fraction(31, 15).unwrap(),
            ],
        ];
        let expected = &(&terms[0][0] * &terms[0][1] * &terms[0][2])
            - &(&terms[1][0] * &terms[1][1] * &terms[1][2])
            + &(&terms[2][0] * &terms[2][1] * &terms[2][2]);

        assert_eq!(
            Rational::signed_product_sum(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1], &terms[0][2]],
                    [&terms[1][0], &terms[1][1], &terms[1][2]],
                    [&terms[2][0], &terms[2][1], &terms[2][2]],
                ],
            ),
            expected
        );
    }

    #[test]
    fn signed_product_sum_preserves_dyadic_exactness() {
        let terms = [
            [
                Rational::fraction(1, 8).unwrap(),
                Rational::fraction(3, 16).unwrap(),
            ],
            [
                Rational::fraction(5, 32).unwrap(),
                Rational::fraction(7, 64).unwrap(),
            ],
            [
                Rational::fraction(-9, 4).unwrap(),
                Rational::fraction(11, 8).unwrap(),
            ],
        ];
        let sum = Rational::signed_product_sum(
            [true, false, true],
            [
                [&terms[0][0], &terms[0][1]],
                [&terms[1][0], &terms[1][1]],
                [&terms[2][0], &terms[2][1]],
            ],
        );

        assert!(sum.is_dyadic());
        assert_eq!(
            sum,
            &(&terms[0][0] * &terms[0][1]) - &(&terms[1][0] * &terms[1][1])
                + &(&terms[2][0] * &terms[2][1])
        );
    }

    #[test]
    fn signed_product_sum_handles_equal_non_dyadic_denominators() {
        let terms = [
            [
                Rational::fraction(7, 10).unwrap(),
                Rational::fraction(13, 7).unwrap(),
            ],
            [
                Rational::fraction(9, 10).unwrap(),
                Rational::fraction(5, 7).unwrap(),
            ],
            [
                Rational::fraction(11, 10).unwrap(),
                Rational::fraction(3, 7).unwrap(),
            ],
        ];

        assert_eq!(
            Rational::signed_product_sum(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1]],
                    [&terms[1][0], &terms[1][1]],
                    [&terms[2][0], &terms[2][1]],
                ],
            ),
            &(&terms[0][0] * &terms[0][1]) - &(&terms[1][0] * &terms[1][1])
                + &(&terms[2][0] * &terms[2][1])
        );
    }

    #[test]
    fn compare() {
        assert!(Rational::one() > Rational::zero());
        assert!(Rational::new(5) > Rational::new(4));
        assert!(Rational::new(-10) < Rational::new(5));
        assert!(Rational::fraction(1, 4).unwrap() < Rational::fraction(1, 3).unwrap());
    }

    #[test]
    fn same() {
        use std::cmp::Ordering;

        assert_eq!(
            Rational::zero().partial_cmp(&Rational::zero()),
            Some(Ordering::Equal)
        );
        assert_eq!(
            Rational::one().partial_cmp(&Rational::one()),
            Some(Ordering::Equal)
        );
        assert_eq!(
            Rational::new(-10).partial_cmp(&Rational::new(-10)),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn divide_by_zero() {
        let err = Rational::fraction(1, 0).unwrap_err();
        assert_eq!(err, Problem::DivideByZero);
        let zero = Rational::zero();
        let err = zero.inverse().unwrap_err();
        assert_eq!(err, Problem::DivideByZero);
    }

    #[test]
    fn operations_work_on_refs_on_rhs() {
        let a = Rational::new(2);
        let b = Rational::new(3);
        let c = Rational::new(6);
        assert_eq!(a.clone() * &b, c.clone());
        assert_eq!(c.clone() / &b, a.clone());
        assert_eq!(c.clone() - &a, Rational::new(4));
        assert_eq!(-&c, Rational::new(-6));
        assert_eq!(a.clone() + &b, Rational::new(5));
    }

    #[test]
    fn operations_work_on_refs() {
        let a = Rational::new(2);
        let b = Rational::new(3);
        let c = Rational::new(6);
        assert_eq!(&a * &b, c.clone());
        assert_eq!(&c / &b, a.clone());
        assert_eq!(&c - &a, Rational::new(4));
        assert_eq!(-&c, Rational::new(-6));
        assert_eq!(&a + &b, Rational::new(5));
    }
}
