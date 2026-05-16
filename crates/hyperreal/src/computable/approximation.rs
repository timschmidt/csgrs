use crate::Computable;
use crate::Rational;
use crate::computable::{Precision, Signal, scale, shift, should_stop, signed};
use num::bigint::Sign;
use num::{BigInt, BigUint, Signed, ToPrimitive};
use num::{One, Zero};
use serde::Deserialize;
use serde::Serialize;
use std::ops::Deref;
use std::sync::LazyLock;

// The elementary kernels in this file use the standard multiple-precision
// pattern of reducing the argument into a small interval, evaluating a guarded
// power series, then rounding once at the requested binary scale. The main
// references for those algorithm families are Brent, "Fast Multiple-Precision
// Evaluation of Elementary Functions", JACM 1976, https://doi.org/10.1145/321941.321944,
// and Brent/Zimmermann, "Modern Computer Arithmetic", Ch. 4,
// https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
// Comments at individual shortcuts call out hyperreal-specific representation
// choices added to avoid construction, allocation, or cache duplication costs.

static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
static FOUR_THIRDS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(4, 3).unwrap());
static SEVEN_FOURTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 4).unwrap());
static TWO_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(2));
static SEVENTY_NINE_TWENTIETHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(79, 20).unwrap());
static FOUR_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(4));
static TWENTY_SEVEN_FIFTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(27, 5).unwrap());
static ELEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(11, 2).unwrap());
static SEVEN_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(7));
static SEVENTEEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(17, 2).unwrap());
static QUARTER_PI_TAN_RESIDUAL_THRESHOLD: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(128));
static NEG_FOUR_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(-4));
static NEG_FOUR_BIGINT: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(-4));
static NEG_SEVENTY_NINE_TWENTIETHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-79, 20).unwrap());
static NEG_TWENTY_SEVEN_FIFTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-27, 5).unwrap());
static NEG_ELEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-11, 2).unwrap());
static NEG_SEVEN_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(-7));
static NEG_SEVENTEEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-17, 2).unwrap());

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) enum Approximation {
    // Exact integer leaf. This is the cheapest approximation source and also
    // exposes exact sign/MSD facts without any refinement.
    Int(BigInt),
    // Exact one is hot enough to avoid even the tiny BigInt payload carried by
    // Int(1). Real::one and integer identity conversion use this leaf.
    One,
    // Shared constants use a process-local approximation cache keyed by enum
    // discriminant; do not replace these with fresh expression trees.
    Constant(SharedConstant),
    // Generic reciprocal node. Constructors try to eliminate this for exact
    // rationals, double inverses, and signed binary offsets before it reaches
    // approximation.
    Inverse(Computable),
    // Sign wrapper kept separate so negate/negate and sign queries collapse
    // without touching child approximation caches.
    Negate(Computable),
    // Generic sum. The evaluator treats Add specially to avoid recursive stack
    // growth in deep expression chains.
    Add(Computable, Computable),
    // Generic product. Exact and dyadic scales are peeled off before this node
    // is created because multiplication dominates dense algebra kernels.
    Multiply(Computable, Computable),
    // Dedicated square node exposes sign/MSD facts and lets sqrt(square(x))
    // collapse structurally when x has a known sign.
    Square(Computable),
    // Exact rational leaf, used for imported floats and parser-folded exact
    // subexpressions.
    Ratio(Rational),
    // Binary scaling by 2^n. This is the preferred representation for dyadic
    // factors because approximation becomes a precision shift.
    Offset(Computable, i32),
    // The remaining Prescaled* variants are approximation kernels whose callers
    // have already reduced the argument into the range required by the series.
    PrescaledExp(Computable),
    Sqrt(Computable),
    PrescaledLn(Computable),
    PrescaledLnRational(Rational),
    BinaryScaledLnRational { residual: Rational, shift: i32 },
    // IntegralAtan stores atan(1/n), used by Machin-style pi and midpoint atan
    // reductions without constructing a rational reciprocal node.
    IntegralAtan(BigInt),
    PrescaledAtan(Computable),
    // Exact rational atan inputs are common in scalar benches. A single
    // deferred node performs the same small/medium/large reductions as
    // Computable::atan without allocating the intermediate add/divide graph.
    AtanRational(Rational),
    // Tiny exact rational asin inputs use the direct power series. Keeping the
    // rational in the node avoids a child Computable::approx call before
    // entering that series.
    AsinRational(Rational),
    PrescaledAsin(Computable),
    // Generic non-rational asin uses the stable half-angle atan transform. A
    // deferred node keeps construction thin for symbolic radicals and endpoint
    // inputs that may never be approximated.
    AsinDeferred(Computable),
    AcosPositive(Computable),
    // Exact-rational positive endpoint acos uses the same half-angle atan
    // transform, but computes the residual rational directly instead of
    // rebuilding a subtraction/division graph for every cold approximation.
    AcosPositiveRational(Rational),
    // Negative endpoint rational acos is pi - acos(|x|). Store |x| directly
    // so construction does not allocate a pi/subtraction graph.
    AcosNegativeRational(Rational),
    AcoshNearOne(Computable),
    AcoshDirect(Computable),
    AsinhNearZero(Computable),
    AsinhDirect(Computable),
    PrescaledAsinh(Computable),
    // Tiny exact-rational asinh/atanh inputs use odd-power series. Storing the
    // rational directly avoids rebuilding a Ratio child for every cold
    // approximation and keeps the exact value symbolic until the kernel rounds.
    AsinhRational(Rational),
    AtanhDirect(Computable),
    PrescaledAtanh(Computable),
    AtanhRational(Rational),
    PrescaledCos(Computable),
    // Small exact-rational Real::cos construction uses this leaf to avoid
    // allocating a Ratio child when the caller only builds or structurally
    // inspects the result. Approximation materializes the same rational series
    // input used by PrescaledCos.
    PrescaledCosRational(Rational),
    // Large exact-rational Real::cos construction is intentionally deferred:
    // range reduction needs cached pi plus BigInt quotient work, which is wasted
    // in scalar construction benchmarks and predicate-heavy code that never
    // asks for digits.
    CosLargeRational(Rational),
    // Exact medium rational trig inputs use dedicated pi/2 - r residual nodes.
    // This avoids rebuilding a generic Add(Offset(pi), -r) graph while keeping
    // approximation lazy until the caller asks for a precision.
    PrescaledCosHalfPiMinusRational(Rational),
    PrescaledSin(Computable),
    // Small exact-rational sine analogue of PrescaledCosRational.
    PrescaledSinRational(Rational),
    // Same lazy large-rational policy as cosine. Approximation uses direct
    // half-pi residual arithmetic so construction-included scalar benches do
    // not pay for an eager reduced expression tree.
    SinLargeRational(Rational),
    // Sine shares the same exact residual representation as cosine so the
    // endpoint identities stay cheap without a generic subtraction node.
    PrescaledSinHalfPiMinusRational(Rational),
    // Exact medium tangent inputs near pi/2 use cot(pi/2 - r). This direct
    // residual node avoids allocating the complement before entering the local
    // quotient kernel.
    PrescaledCotHalfPiMinusRational(Rational),
    // Tangent gets its own large-rational node because the generic path first
    // builds a pi-reduced residual and then a quotient tree. The direct kernel
    // below reuses the same half-pi residual as sin/cos and divides locally.
    TanLargeRational(Rational),
    PrescaledTan(Computable),
    // Small exact-rational tangent keeps construction lightweight and enters
    // the same local quotient kernel once digits are requested.
    PrescaledTanRational(Rational),
    PrescaledCot(Computable),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(super) enum SharedConstant {
    E,
    Pi,
    InvPi,
    Tau,
    Ln2,
    Ln3,
    Ln5,
    Ln6,
    Ln7,
    Ln10,
    Sqrt2,
    Sqrt3,
    Acosh2,
    Asinh1,
    AtanInv2,
    AtanInv5,
}

impl SharedConstant {
    pub(super) const COUNT: usize = 16;

    pub(super) fn cache_index(self) -> usize {
        match self {
            Self::E => 0,
            Self::Pi => 1,
            Self::InvPi => 2,
            Self::Tau => 3,
            Self::Ln2 => 4,
            Self::Ln3 => 5,
            Self::Ln5 => 6,
            Self::Ln6 => 7,
            Self::Ln7 => 8,
            Self::Ln10 => 9,
            Self::Sqrt2 => 10,
            Self::Sqrt3 => 11,
            Self::Acosh2 => 12,
            Self::Asinh1 => 13,
            Self::AtanInv2 => 14,
            Self::AtanInv5 => 15,
        }
    }
}

impl Approximation {
    pub fn approximate(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        use Approximation::*;

        // This is intentionally a thin dispatcher. Algebraic simplification and
        // cache selection live in `Computable` constructors so kernels can assume
        // their documented preconditions and avoid repeated shape checks.
        match self {
            Int(i) => scale(i.clone(), -p),
            One => scale(signed::ONE.deref().clone(), -p),
            Constant(c) => c.approximate(signal, p),
            Inverse(c) => inverse(signal, c, p),
            Negate(c) => -c.approx_signal(signal, p),
            Add(c1, c2) => add(signal, c1, c2, p),
            Multiply(c1, c2) => multiply(signal, c1, c2, p),
            Square(c) => square(signal, c, p),
            Ratio(r) => ratio(r, p),
            Offset(c, n) => offset(signal, c, *n, p),
            PrescaledExp(c) => exp(signal, c, p),
            Sqrt(c) => sqrt(signal, c, p),
            PrescaledLn(c) => ln(signal, c, p),
            PrescaledLnRational(r) => ln_rational(signal, r, p),
            BinaryScaledLnRational { residual, shift } => {
                binary_scaled_ln_rational(signal, residual, *shift, p)
            }
            IntegralAtan(i) => atan(signal, i, p),
            PrescaledAtan(c) => atan_computable(signal, c, p),
            AtanRational(r) => atan_rational(signal, r, p),
            AsinRational(r) => asin_rational(signal, r, p),
            PrescaledAsin(c) => asin_computable(signal, c, p),
            AsinDeferred(c) => asin_deferred(signal, c, p),
            AcosPositive(c) => acos_positive(signal, c, p),
            AcosPositiveRational(r) => acos_positive_rational(signal, r, p),
            AcosNegativeRational(r) => acos_negative_rational(signal, r, p),
            AcoshNearOne(c) => acosh_near_one(signal, c, p),
            AcoshDirect(c) => acosh_direct(signal, c, p),
            AsinhNearZero(c) => asinh_near_zero(signal, c, p),
            AsinhDirect(c) => asinh_direct(signal, c, p),
            PrescaledAsinh(c) => asinh_computable(signal, c, p),
            AsinhRational(r) => asinh_rational(signal, r, p),
            AtanhDirect(c) => atanh_direct(signal, c, p),
            PrescaledAtanh(c) => atanh_computable(signal, c, p),
            AtanhRational(r) => atanh_rational(signal, r, p),
            PrescaledCos(c) => cos(signal, c, p),
            PrescaledCosRational(r) => cos_rational(signal, r, p),
            CosLargeRational(r) => cos_large_rational(signal, r, p),
            PrescaledCosHalfPiMinusRational(r) => cos_half_pi_minus_rational(signal, r, p),
            PrescaledSin(c) => sin(signal, c, p),
            PrescaledSinRational(r) => sin_rational(signal, r, p),
            SinLargeRational(r) => sin_large_rational(signal, r, p),
            PrescaledSinHalfPiMinusRational(r) => sin_half_pi_minus_rational(signal, r, p),
            PrescaledCotHalfPiMinusRational(r) => cot_half_pi_minus_rational(signal, r, p),
            TanLargeRational(r) => tan_large_rational(signal, r, p),
            PrescaledTan(c) => tan(signal, c, p),
            PrescaledTanRational(r) => tan_rational(signal, r, p),
            PrescaledCot(c) => cot(signal, c, p),
        }
    }
}

impl SharedConstant {
    fn approximate(self, signal: &Option<Signal>, p: Precision) -> BigInt {
        // Every shared constant routes through the same enum so cloned public
        // constants share approximation caches. Some constants are still built
        // from series identities here, but the cache prevents redoing that work
        // for repeated scalar and matrix operations.
        match self {
            Self::E => e(p),
            Self::Pi => pi(signal, p),
            Self::InvPi => inverse(signal, &Computable::pi(), p),
            Self::Tau => pi(signal, p - 1),
            Self::Ln2 => ln2(signal, p),
            Self::Ln3 => ln_constant(signal, Rational::new(3), p),
            Self::Ln5 => ln_constant(signal, Rational::new(5), p),
            Self::Ln6 => ln_constant(signal, Rational::new(6), p),
            Self::Ln7 => ln_constant(signal, Rational::new(7), p),
            Self::Ln10 => ln_constant(signal, Rational::new(10), p),
            Self::Sqrt2 => sqrt_constant(signal, Rational::new(2), p),
            Self::Sqrt3 => sqrt_constant(signal, Rational::new(3), p),
            Self::Acosh2 => acosh2_constant(signal, p),
            Self::Asinh1 => asinh1_constant(signal, p),
            Self::AtanInv2 => atan(signal, &BigInt::from(2_u8), p),
            Self::AtanInv5 => atan(signal, &BigInt::from(5_u8), p),
        }
    }
}

fn raw(kind: Approximation) -> Computable {
    // Build a node with no constructor-level simplification. This is used only
    // for internal constant identities where adding public simplification would
    // either recurse back into the same constant or erase the intended kernel.
    Computable {
        internal: Box::new(kind),
        cache: std::cell::RefCell::new(crate::computable::Cache::Invalid),
        bound: std::cell::RefCell::new(crate::computable::BoundCache::Invalid),
        exact_sign: std::cell::RefCell::new(crate::computable::ExactSignCache::Invalid),
        signal: None,
    }
}

fn pi(signal: &Option<Signal>, p: Precision) -> BigInt {
    // Machin formula: pi = 4 * (4 atan(1/5) - atan(1/239)).
    // It converges much faster than a generic trig/log identity and is stable
    // enough to serve as the shared pi cache source. This is the same
    // arctangent/Machin-style family used in multiple-precision elementary
    // evaluation; see Brent, https://doi.org/10.1145/321941.321944.
    let atan5 = Computable::prescaled_atan(BigInt::from(5_u8));
    let atan_239 = Computable::prescaled_atan(BigInt::from(239_u16));
    let four = Computable::integer(BigInt::from(4_u8));
    let four_atan5 = four.clone().multiply(atan5);
    let sum = four_atan5.add(atan_239.negate());
    four.multiply(sum).approx_signal(signal, p)
}

fn ln2(signal: &Option<Signal>, p: Precision) -> BigInt {
    // A fixed atanh/log1p decomposition for ln(2). Keeping ln2 as its own
    // shared constant matters because exp/ln range reduction adds multiples of
    // ln2 frequently. The identity routes each piece through the reduced
    // ln(1+x) kernel, following the argument-reduction-plus-series approach in
    // Brent/Zimmermann, Ch. 4:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    let prescaled_9 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 9).unwrap(),
    )));
    let prescaled_24 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 24).unwrap(),
    )));
    let prescaled_80 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 80).unwrap(),
    )));

    let ln2_1 = Computable::integer(BigInt::from(7_u8)).multiply(prescaled_9);
    let ln2_2 = Computable::integer(BigInt::from(2_u8)).multiply(prescaled_24);
    let ln2_3 = Computable::integer(BigInt::from(3_u8)).multiply(prescaled_80);

    ln2_1
        .add(ln2_2.negate())
        .add(ln2_3)
        .approx_signal(signal, p)
}

fn ln_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // Non-ln2 logarithm constants reuse the normal logarithm reduction, then
    // benefit from the shared-constant cache on future calls.
    Computable::rational(n).ln().approx_signal(signal, p)
}

fn sqrt_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    // sqrt(2) and sqrt(3) are common exact trig results; they share caches even
    // though the approximation kernel is the generic sqrt.
    raw(Approximation::Sqrt(Computable::rational(n))).approx_signal(signal, p)
}

fn acosh2_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // acosh(2) = ln(2 + sqrt(3)). This exact value is common enough in
    // inverse-hyperbolic tests to benefit from the shared-constant cache.
    Computable::rational(Rational::new(2))
        .add(Computable::sqrt_constant(3).unwrap())
        .ln()
        .approx_signal(signal, p)
}

fn asinh1_constant(signal: &Option<Signal>, p: Precision) -> BigInt {
    // asinh(1) = ln(1 + sqrt(2)), also equal to atanh(sqrt(2)/2).
    Computable::one()
        .add(Computable::sqrt_constant(2).unwrap())
        .ln()
        .approx_signal(signal, p)
}

fn e_terms_for_precision(p: Precision) -> u32 {
    // Choose enough 1/k! terms so the binary-split tail is below the requested
    // bit precision. Positive precisions need only a tiny constant amount.
    let needed_bits = if p < 0 { (-p) as u64 + 4 } else { 4 };
    let mut factorial = BigUint::one();
    let mut n = 0_u32;
    loop {
        let next = &factorial * BigUint::from(n + 1);
        if next.bits() > needed_bits {
            return n;
        }
        factorial = next;
        n += 1;
    }
}

// Returns (P, Q) for sum_{k=a}^{b-1} 1 / prod_{j=a}^k j == P / Q
fn e_binary_split(a: u32, b: u32) -> (BigUint, BigUint) {
    // Binary splitting keeps numerator/denominator growth balanced. A linear
    // summation of rationals is noticeably more allocation-heavy for cold e.
    // This is the standard binary-splitting technique for series evaluation;
    // see Brent/Zimmermann, Sec. 4.9:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    if b - a == 1 {
        return (BigUint::one(), BigUint::from(a));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q) = e_binary_split(a, mid);
    let (right_p, right_q) = e_binary_split(mid, b);
    (left_p * &right_q + right_p, left_q * right_q)
}

fn rounded_ratio(numerator: BigUint, denominator: BigUint, p: Precision) -> BigInt {
    // All kernels return an integer scaled by 2^-p and accurate within one unit
    // at that scale. Centralizing rounding avoids subtly different half-up
    // behavior between constants.
    if p <= 0 {
        let shift = usize::try_from(-p).expect("precision shift should fit in usize");
        let dividend = numerator << shift;
        BigInt::from((dividend + (&denominator >> 1)) / denominator)
    } else {
        let shift = usize::try_from(p).expect("precision shift should fit in usize");
        let scaled_denominator = denominator << shift;
        BigInt::from((numerator + (&scaled_denominator >> 1)) / scaled_denominator)
    }
}

fn e(p: Precision) -> BigInt {
    // e = 1 + sum_{k>=1} 1/k!. The tail is evaluated as one rational by binary
    // splitting and rounded once at the target scale. Rounding only once keeps
    // the exact-real cache stable and avoids accumulating per-term rational
    // normalization costs.
    let terms = e_terms_for_precision(p);
    if terms == 0 {
        return rounded_ratio(BigUint::one(), BigUint::one(), p);
    }

    let (tail_p, tail_q) = e_binary_split(1, terms + 1);
    rounded_ratio(tail_q.clone() + tail_p, tail_q, p)
}

fn inverse(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Plan reciprocal precision from planning facts when available, otherwise fall
    // back to iterative probing. This keeps exact zero short-circuited and avoids
    // a full iterative MSD pass for structural operands that already expose a
    // useful magnitude envelope.
    let (sign, planned_msd) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = planned_msd.flatten().unwrap_or_else(|| c.iter_msd());
    let inv_msd = 1 - msd;
    let digits_needed = inv_msd - p + 3;
    let mut prec_needed = msd - digits_needed;
    let mut log_scale_factor = -p - prec_needed;

    let scaled_divisor = loop {
        if log_scale_factor < 0 {
            return Zero::zero();
        }

        let scaled = c.approx_signal(signal, prec_needed);
        if !scaled.is_zero() {
            break scaled;
        }

        if should_stop(signal) {
            return Zero::zero();
        }

        // `iter_msd` is deliberately cheap and may overestimate a value whose
        // leading bits come from cancellation, such as a nested sqrt/log
        // reduction. Refine instead of dividing by a rounded-zero denominator.
        prec_needed -= 8;
        log_scale_factor += 8;
        if log_scale_factor > 16_384 {
            panic!("ArithmeticException");
        }
    };

    let dividend = signed::ONE.deref() << log_scale_factor;
    let abs_scaled_divisor = scaled_divisor.abs();
    let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
    let result: BigInt = adj_dividend / abs_scaled_divisor;

    if scaled_divisor.sign() == Sign::Minus {
        -result
    } else {
        result
    }
}

fn add(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Addition first tries to prove one operand too small to affect the result
    // at precision p. That dominates deep structural sums and avoids touching
    // tiny terms when signs/MSDs are already known.
    let extra = 4;
    let cutoff = p - extra;
    let (sign1, planning_msd1) = c1.planning_sign_and_msd();
    let (sign2, planning_msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) {
        return c2.approx_signal(signal, p);
    }
    if sign2 == Some(Sign::NoSign) {
        return c1.approx_signal(signal, p);
    }
    let msd1 = planning_msd1.unwrap_or_else(|| c1.msd(cutoff));
    let msd2 = planning_msd2.unwrap_or_else(|| c2.msd(cutoff));

    match (msd1, msd2) {
        (None, None) => return Zero::zero(),
        (None, Some(_)) if sign2.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_), None) if sign1.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        (Some(left), Some(_right))
            if sign1 == sign2
                && sign2.is_some_and(|sign| sign != Sign::NoSign)
                && left < cutoff =>
        {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_left), Some(right))
            if sign1 == sign2
                && sign1.is_some_and(|sign| sign != Sign::NoSign)
                && right < cutoff =>
        {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        _ => (),
    }

    scale(
        c1.approx_signal(signal, p - 2) + c2.approx_signal(signal, p - 2),
        -2,
    )
}

fn msd_from_appr(prec: Precision, appr: &BigInt) -> Precision {
    prec + appr.magnitude().bits() as Precision - 1
}

fn multiply_with_known_msd(
    signal: &Option<Signal>,
    known: &Computable,
    known_msd: Precision,
    other: &Computable,
    p: Precision,
) -> BigInt {
    // Evaluate the unknown-size operand first, then request only the precision
    // actually needed from the known-size operand. This asymmetric planning is
    // cheaper for products of exact scales and expensive transcendental nodes.
    let prec_other = p - known_msd - 3;
    let appr_other = other.approx_signal(signal, prec_other);

    if appr_other.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let msd_other = msd_from_appr(prec_other, &appr_other);
    let prec_known = p - msd_other - 3;
    let appr_known = known.approx_signal(signal, prec_known);

    let scale_digits = prec_known + prec_other - p;
    scale(appr_known * appr_other, scale_digits)
}

fn multiply(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    // Prefer the operand with known larger magnitude as the precision anchor.
    // If one side is effectively zero at the planning cutoff, the product is
    // zero at the requested precision without evaluating both sides deeply.
    let half_prec = (p >> 1) - 1;
    let (sign1, msd1) = c1.planning_sign_and_msd();
    let (sign2, msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) || sign2 == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd1 = msd1.unwrap_or_else(|| c1.msd(half_prec));
    let msd2 = msd2.unwrap_or_else(|| c2.msd(half_prec));

    match (msd1, msd2) {
        (None, None) => Zero::zero(),
        (Some(msd_op1), None) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
        (None, Some(msd_op2)) => multiply_with_known_msd(signal, c2, msd_op2, c1, p),
        (Some(msd_op1), Some(msd_op2)) if msd_op2 > msd_op1 => {
            multiply_with_known_msd(signal, c2, msd_op2, c1, p)
        }
        (Some(msd_op1), Some(_msd_op2)) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
    }
}

fn square(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Square can reuse one approximation of the child. Constructors create this
    // node for repeated powers so multiplication does not duplicate child work.
    let half_prec = (p >> 1) - 1;
    let (sign, msd) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = match msd.unwrap_or_else(|| c.msd(half_prec)) {
        None => {
            return Zero::zero();
        }
        Some(msd) => msd,
    };
    let prec = p - msd - 3;

    let appr = c.approx_signal(signal, prec);

    if appr.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let scale_digits = prec + prec - p;
    scale(&appr * &appr, scale_digits)
}

fn ratio(r: &Rational, p: Precision) -> BigInt {
    // Exact rationals approximate by shifting the numerator/denominator ratio
    // directly; dyadic rationals make this path especially cheap.
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(-p)
    }
}

fn offset(signal: &Option<Signal>, c: &Computable, n: i32, p: Precision) -> BigInt {
    // x * 2^n at precision p is just x at precision p-n. This is why dyadic
    // scales are represented as Offset nodes instead of generic multiplication.
    c.approx_signal(signal, p - n)
}

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}

/* Only intended for Computable values < 0.5, others will be pre-scaled
 * in Computable::exp */
fn exp(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: caller has reduced |c| below roughly 1/2. The series
    // is intentionally simple here; range reduction belongs in `Computable::exp`.
    // That split mirrors standard multiple-precision exp algorithms: reduce
    // first, evaluate the Taylor series on the reduced input, and reconstruct.
    // See Brent, https://doi.org/10.1145/321941.321944.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 2;
    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 3;

    let op_appr = c.approx_signal(signal, op_prec);

    // Error in argument results in error of < 3/8 ulp.
    // Sum of term eval. rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.
    let scaled_1 = signed::ONE.deref() << -calc_precision;

    // The loop compares borrowed magnitudes. Calling `abs()` here allocates a
    // fresh BigInt every term and shows up in cold transcendental benches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scaled_1.clone();
    let mut sum = scaled_1;
    let mut n: i32 = 0;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_appr, op_prec) / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn sqrt(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Sqrt uses a fixed-size integer sqrt for moderate precision and recursive
    // Newton refinement for deeper requests. This avoids pulling in floating
    // approximations while keeping high-precision sqrt from scaling quadratically.
    // Newton sqrt/reciprocal-sqrt refinement is the standard arbitrary-precision
    // strategy described in Brent/Zimmermann, Secs. 1.5 and 4.2:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    let fp_prec: i32 = 140;
    let fp_op_prec: i32 = 150;

    let max_prec_needed = p.saturating_mul(2).saturating_sub(1);
    let (known_sign, planned_msd) = c.planning_sign_and_msd();
    if known_sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = match planned_msd {
        Some(Some(msd)) => msd,
        _ => match c.msd(max_prec_needed) {
            Some(msd) => msd,
            None => {
                let rough = c.approx_signal(signal, max_prec_needed);
                if rough.is_zero() {
                    return Zero::zero();
                }
                rough.magnitude().bits() as Precision - 1 + max_prec_needed
            }
        },
    };

    if msd <= max_prec_needed {
        return Zero::zero();
    }

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let result_msd = msd / 2;
    let result_digits = result_msd - p;

    if result_digits > fp_prec {
        // Compute less precise approximation and use a Newton iter.
        let appr_digits = result_digits / 2 + 6;
        // This should be conservative.  Is fewer enough?
        let appr_prec = result_msd - appr_digits;

        let last_appr = sqrt(signal, c, appr_prec);
        let prod_prec = 2 * appr_prec;

        let op_appr = c.approx_signal(signal, prod_prec);

        // Slightly fewer might be enough;
        // Compute (last_appr * last_appr + op_appr)/(last_appr/2)
        // while adjusting the scaling to make everything work

        let prod_prec_scaled_numerator = (&last_appr * &last_appr) + op_appr;
        let scaled_numerator = scale(prod_prec_scaled_numerator, appr_prec - p);

        let shifted_result = scaled_numerator / last_appr;

        (shifted_result + signed::ONE.deref()) / signed::TWO.deref()
    } else {
        // Use an approximation from the Num crate
        // Make sure all precisions are even
        let op_prec = (msd - fp_op_prec) & !1;
        let working_prec = op_prec - fp_op_prec;

        let scaled_bi_appr = c.approx_signal(signal, op_prec) << fp_op_prec;

        let scaled_sqrt = scaled_bi_appr.sqrt();

        let shift_count = working_prec / 2 - p;
        shift(scaled_sqrt, shift_count)
    }
}

// Compute cosine of |c| < 1
// uses a Taylor series expansion.
fn cos(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1. Argument reduction and exact pi-multiple
    // handling happen before this node is constructed. Keeping range reduction
    // outside the Taylor kernel is the same split used by multi-precision
    // sin/cos algorithms in Brent/Zimmermann, Ch. 4:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute cosine of an exact rational |r| < 1 without allocating a temporary
// Ratio node. This preserves the same Taylor algorithm as `cos` while keeping
// the stored rational symbolic until the final requested precision. 2026-05
// numerical_micro targeted runs showed the direct rational feed removes cold
// approximation setup from small exact-rational trig rows without changing the
// cached path.
fn cos_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn large_rational_half_pi_multiple(signal: &Option<Signal>, r: &Rational) -> BigInt {
    // Deferred large-rational trig needs the same nearest-half-pi quotient as
    // Computable::sin/cos, but rebuilding a Ratio node just to call the generic
    // reducer was the remaining hot path in exact 1e6/1e30 benchmarks. This is
    // the exact-rational Payne-Hanek quotient estimate from the constructor
    // layer, with the residual correction performed directly from cached pi.
    if let Some(multiple) = fixed_small_large_rational_half_pi_multiple(r) {
        return multiple;
    }

    let mut multiple = Computable::half_pi_multiple_exact_rational(r)
        .unwrap_or_else(|| Computable::rational(r.clone()).half_pi_multiple());
    let rough_appr = large_rational_half_pi_residual(signal, r, &multiple, -1);

    if rough_appr >= *crate::computable::signed::TWO {
        multiple += 1;
    } else if rough_appr <= -crate::computable::signed::TWO.deref().clone() {
        multiple -= 1;
    }

    multiple
}

fn fixed_small_large_rational_half_pi_multiple(r: &Rational) -> Option<BigInt> {
    // For |r| in [79/20, 4], the nearest half-pi multiple is exactly +/-3.
    // This catches tan rows just below the large-rational threshold while
    // staying above 5*pi/4.
    if r >= SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r <= FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r >= NEG_FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }

    // For |r| in [4, 27/5], the nearest half-pi multiple is exactly +/-3.
    // The upper bound is kept conservatively below 7*pi/4, so the shortcut
    // remains a cheap certificate rather than a fresh pi-dependent comparison.
    if r.msd_exact() != Some(2) {
        return None;
    }
    if r >= FOUR_RATIONAL.deref() && r <= TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_FOUR_RATIONAL.deref() && r >= NEG_TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }
    // For |r| in [11/2, 7], the nearest half-pi multiple is exactly +/-4.
    // The lower bound is above 7*pi/4 and the upper bound is below 9*pi/4,
    // covering the next promoted tan cluster without a pi-dependent probe.
    if r >= ELEVEN_HALVES_RATIONAL.deref() && r <= SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-4");
        return Some(BigInt::from(4));
    }
    if r <= NEG_ELEVEN_HALVES_RATIONAL.deref() && r >= NEG_SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg4");
        return Some(BigInt::from(-4));
    }
    // For |r| in [7, 17/2], the nearest half-pi multiple is exactly +/-5.
    // This sits between 9*pi/4 and 11*pi/4 and covers the next promoted tan
    // cluster without opening the full large-rational Payne-Hanek path.
    if r >= SEVEN_RATIONAL.deref() && r <= SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-5");
        return Some(BigInt::from(5));
    }
    if r <= NEG_SEVEN_RATIONAL.deref() && r >= NEG_SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg5");
        return Some(BigInt::from(-5));
    }
    None
}

fn large_rational_half_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Approximate r - multiple*pi/2 at precision p without allocating
    // Add(Multiply(Pi, k), Ratio(r)). This is the performance-critical part of
    // the direct large-rational kernels; it keeps the mathematical reduction
    // identical to the generic path while avoiding expression graph setup.
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    if multiple == signed::FOUR.deref() || multiple == NEG_FOUR_BIGINT.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-residual-two-pi");
        let pi_precision = work_precision - 6;
        let pi = Computable::pi().approx_signal(signal, pi_precision);
        let two_pi = scale(pi, pi_precision - work_precision + 1);
        let half_pi_multiple = if multiple.sign() == Sign::Minus {
            -two_pi
        } else {
            two_pi
        };
        return scale(rational - half_pi_multiple, -extra);
    }
    let multiple_msd = i32::try_from(multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 4;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let half_pi_multiple = scale(pi * multiple, pi_precision - work_precision - 1);
    scale(rational - half_pi_multiple, -extra)
}

fn large_rational_quadrant(multiple: &BigInt) -> BigInt {
    ((multiple % crate::computable::signed::FOUR.deref()) + crate::computable::signed::FOUR.deref())
        % crate::computable::signed::FOUR.deref()
}

fn large_rational_quarter_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    quarter_sign: i8,
    p: Precision,
) -> BigInt {
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    let quarter_multiple: BigInt = (multiple << 1) + BigInt::from(quarter_sign);
    let multiple_msd = i32::try_from(quarter_multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 5;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let quarter_pi_multiple = scale(pi * quarter_multiple, pi_precision - work_precision - 2);
    scale(rational - quarter_pi_multiple, -extra)
}

fn sin_cos_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn divide_scaled(numerator: BigInt, denominator: BigInt, p: Precision) -> BigInt {
    let abs_denominator = denominator.abs();
    if abs_denominator.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if denominator.sign() == Sign::Minus {
        -numerator << -p
    } else {
        numerator << -p
    };
    let adjustment = &abs_denominator >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_denominator;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_denominator
    }
}

fn tan_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> BigInt {
    let (sin_appr, cos_appr) = sin_cos_scaled_argument(signal, op_appr, op_prec, p);
    divide_scaled(sin_appr, cos_appr, p)
}

fn tan_large_rational_quarter_pi(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> Option<BigInt> {
    let quadrant = large_rational_quadrant(multiple).to_u8()?;

    let rough_residual = large_rational_half_pi_residual(signal, r, multiple, -8);
    let quarter_sign = if rough_residual >= *QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref() {
        1
    } else if rough_residual <= -QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref().clone() {
        -1
    } else {
        return None;
    };

    crate::trace_dispatch!("computable_approx", "tan", "quarter-pi-large-rational");
    let working_prec = p - 8;
    let op_prec = working_prec - 2;
    let delta = large_rational_quarter_pi_residual(signal, r, multiple, quarter_sign, op_prec);
    let tan_delta = tan_scaled_argument(signal, delta, op_prec, working_prec);
    let one = signed::ONE.deref() << -working_prec;
    let direct_tangent_form = (quadrant % 2 == 0) == (quarter_sign > 0);
    let (numerator, denominator) = if direct_tangent_form {
        (one.clone() + &tan_delta, one - tan_delta)
    } else {
        (tan_delta.clone() - &one, one + tan_delta)
    };
    Some(divide_scaled(numerator, denominator, p))
}

fn cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as cos(|x| < 1), but the argument approximation comes
    // from the direct residual above instead of a child Computable node.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as sin(|x| < 1), fed by the direct large-rational
    // residual to avoid constructing a generic reduced Computable expression.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn cos_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Construction-included benches pay heavily for eager half-pi reduction on
    // large exact rationals. Use the direct residual kernels here so the public
    // constructor can stay lazy without recursing back through Computable::cos.
    let multiple = large_rational_half_pi_multiple(signal, r);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(1) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -cos_large_rational_residual(signal, r, &multiple, p),
        Some(3) => sin_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

fn half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Specialized residual for exact medium trig inputs. It performs the same
    // guarded subtraction as the generic Add(Offset(pi), -r) path, but without
    // allocating or querying a composite expression on every cold approximation.
    let extra = 2;
    let work_precision = p - extra;
    let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
    let rational = ratio(r, work_precision);
    scale(half_pi - rational, -extra)
}

// Compute cosine of pi/2 - r for exact 1 <= r < 3/2.
fn cos_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Compute the exact rational residual directly from cached pi. The generic
    // equivalent would allocate a short Add tree before entering this same series.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute sine of |c| < 1
// uses a Taylor series expansion.
fn sin(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1. The caller keeps large arguments out of this
    // Taylor loop so huge sin/cos rows spend time in reduction, not series setup.
    // This follows the reduced-argument series scheme in Brent/Zimmermann, Ch. 4:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of `sin`. It avoids allocating a temporary
    // Computable leaf for PrescaledSinRational while still rounding the rational
    // argument exactly once at the requested guard precision.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same lazy public-construction policy as cosine. The direct residual
    // arithmetic avoids allocating the generic reduced expression tree while
    // preserving the standard quadrant identities.
    let multiple = large_rational_half_pi_multiple(signal, r);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => sin_large_rational_residual(signal, r, &multiple, p),
        Some(1) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(3) => -cos_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

// Compute sine of pi/2 - r for exact 1 <= r < 3/2.
fn sin_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Compute the exact rational residual directly from cached pi. The generic
    // equivalent would allocate a short Add tree before entering this same series.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_cos_half_pi_minus_rational(
    signal: &Option<Signal>,
    r: &Rational,
    p: Precision,
) -> (BigInt, BigInt) {
    // Shared complement kernel for cot(pi/2-r): compute the exact-rational
    // residual once, then feed both Taylor sums. This keeps the medium tangent
    // shortcut from paying two pi/rational subtractions per approximation.
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn cot_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // tan(r) near pi/2 is cot(pi/2-r). Reusing the direct exact-rational
    // residual for both numerator and denominator avoids the generic
    // PrescaledCot(Offset(pi/2, -r)) expression graph.
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let (sin_appr, cos_appr) = sin_cos_half_pi_minus_rational(signal, r, working_prec);
    let abs_sin = sin_appr.abs();

    if abs_sin.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = cos_appr << -p;
    let adjustment = &abs_sin >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_sin;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_sin
    }
}

// Compute tangent of |c| < 1.
// This uses the direct quotient tan(x) = sin(x) / cos(x),
// but computes both approximations locally to avoid building
// separate Computable trees for sin, cos, inverse, and multiply.
fn tan(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1 and not near a pole. The constructor rewrites
    // near-pi/2 inputs to cot(complement) before this quotient is used.
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let sin_appr = sin(signal, c, working_prec);
    let cos_appr = cos(signal, c, working_prec);
    let abs_cos = cos_appr.abs();

    if abs_cos.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if cos_appr.sign() == Sign::Minus {
        -sin_appr << -p
    } else {
        sin_appr << -p
    };
    let adjustment = &abs_cos >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_cos;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_cos
    }
}

fn tan_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same local quotient as `tan`, but both numerator and denominator consume
    // the stored exact rational directly. This keeps exact-rational tangent
    // approximation lazy without rebuilding child Ratio nodes.
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let sin_appr = sin_rational(signal, r, working_prec);
    let cos_appr = cos_rational(signal, r, working_prec);
    let abs_cos = cos_appr.abs();

    if abs_cos.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if cos_appr.sign() == Sign::Minus {
        -sin_appr << -p
    } else {
        sin_appr << -p
    };
    let adjustment = &abs_cos >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_cos;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_cos
    }
}

fn tan_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Large tangent is evaluated as a local sin/cos quotient after the same
    // direct half-pi reduction used by sin/cos. This keeps exact large rationals
    // off the generic pi-reduction path and avoids constructing inverse nodes.
    crate::trace_dispatch!("computable_approx", "tan", "large-rational-direct-quotient");
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let multiple = large_rational_half_pi_multiple(signal, r);
    if let Some(reduced) = tan_large_rational_quarter_pi(signal, r, &multiple, p) {
        return reduced;
    }
    let (sin_residual, cos_residual) =
        sin_cos_large_rational_residual(signal, r, &multiple, working_prec);
    let (sin_appr, cos_appr) = match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => (sin_residual, cos_residual),
        Some(1) => (cos_residual, -sin_residual),
        Some(2) => (-sin_residual, -cos_residual),
        Some(3) => (-cos_residual, sin_residual),
        _ => unreachable!("quadrant reduction is modulo four"),
    };
    let abs_cos = cos_appr.abs();

    if abs_cos.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if cos_appr.sign() == Sign::Minus {
        -sin_appr << -p
    } else {
        sin_appr << -p
    };
    let adjustment = &abs_cos >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_cos;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_cos
    }
}

// Compute cotangent of |c| < 1.
// This mirrors tan(x) = sin(x) / cos(x), but flips the quotient so
// tan(pi/2 - x) can avoid building an extra inverse Computable node.
fn cot(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Used only after a tangent complement reduction, where sin(c) should be
    // safely away from zero.
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let sin_appr = sin(signal, c, working_prec);
    let cos_appr = cos(signal, c, working_prec);
    let abs_sin = sin_appr.abs();

    if abs_sin.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = cos_appr << -p;
    let adjustment = &abs_sin >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_sin;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_sin
    }
}

// Compute an approximation of ln(1+x) to precision p.
// This assumes |x| < 1/2.
// It uses ln(1+x) = 2 * atanh(x / (2 + x)),
// whose odd-power series converges substantially faster
// than the direct Taylor series when x is near 1/2.
fn ln(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: this computes ln(1+x), not arbitrary ln(x). Public
    // construction keeps |x| < 1/2 by inversion, sqrt scaling, and powers of two.
    // The atanh transform is a standard log argument reduction for faster odd
    // power-series convergence; see Brent/Zimmermann, Ch. 4:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let scaled_x = scale(op_appr, op_prec - calc_precision);
    let scaled_one = signed::ONE.deref() << -calc_precision;
    let denominator = (&scaled_one << 1) + &scaled_x;

    let numerator = &scaled_x << -calc_precision;
    let y: BigInt = if numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-&numerator) + (&denominator >> 1)) / &denominator;
        -rounded
    } else {
        (&numerator + (&denominator >> 1)) / &denominator
    };

    let y_squared = scale(&y * &y, calc_precision);
    let mut current_power = y.clone();
    let mut current_term = y.clone();
    let mut sum = current_term.clone();
    let mut n = 1;

    // Keep the atanh-transformed ln series from allocating an absolute BigInt
    // on every odd term.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &y_squared, calc_precision);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum << 1, calc_precision - p)
}

fn ln_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact-rational ln1p paths already know the small residual. Feed it
    // directly into the same atanh-transformed series instead of wrapping it in
    // a temporary Ratio child and calling back through Computable::approx.
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let scaled_x = scale(op_appr, op_prec - calc_precision);
    let scaled_one = signed::ONE.deref() << -calc_precision;
    let denominator = (&scaled_one << 1) + &scaled_x;

    let numerator = &scaled_x << -calc_precision;
    let y: BigInt = if numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-&numerator) + (&denominator >> 1)) / &denominator;
        -rounded
    } else {
        (&numerator + (&denominator >> 1)) / &denominator
    };

    let y_squared = scale(&y * &y, calc_precision);
    let mut current_power = y.clone();
    let mut current_term = y.clone();
    let mut sum = current_term.clone();
    let mut n = 1;

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &y_squared, calc_precision);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum << 1, calc_precision - p)
}

fn binary_scaled_ln_rational(
    signal: &Option<Signal>,
    residual: &Rational,
    shift: i32,
    p: Precision,
) -> BigInt {
    // Exact-rational range reduction often produces ln(2^k * (1+x)) for small
    // rational x. Keep that known shape inside one approximation node instead
    // of constructing a transient add/multiply graph around the shared ln2
    // constant for every adversarial ln(1+x^2) row.
    crate::trace_dispatch!("computable_approx", "ln", "binary-scaled-rational");
    if shift == 0 {
        return ln_rational(signal, residual, p);
    }

    let sum_precision = p - 2;
    let residual_appr = ln_rational(signal, residual, sum_precision);
    let shift_abs = shift.unsigned_abs();
    let shift_msd = i32::try_from(u32::BITS - 1 - shift_abs.leading_zeros())
        .expect("shift magnitude bits fit in i32");
    let ln2_precision = sum_precision - shift_msd - 3;
    let ln2_appr = Computable::ln2().approx_signal(signal, ln2_precision);
    let ln2_term = scale(
        BigInt::from(shift) * ln2_appr,
        ln2_precision - sum_precision,
    );

    scale(residual_appr + ln2_term, -2)
}

// Approximate the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
fn atan(signal: &Option<Signal>, i: &BigInt, p: Precision) -> BigInt {
    // Integral atan is used for atan(1/n), where division by n^2 each iteration
    // is cheaper and more stable than approximating a rational Computable child.
    // This is the arctangent-series kernel used by the Machin pi computation;
    // see Brent, https://doi.org/10.1145/321941.321944.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 2; // conservative estimate > 0.
    // from Java implementation description:

    // Claim: each intermediate term is accurate
    // to 2*base^calc_precision.
    // Total rounding error in series computation is
    // 2*iterations_needed*base^calc_precision,
    // exclusive of error in op.

    let calc_precision = p - bound_log2(2 * iterations_needed) - 2;
    // Error in argument results in error of < 3/8 ulp.
    // Cumulative arithmetic rounding error is < 1/4 ulp.
    // Series truncation error < 1/4 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    let max_trunc_error: BigUint = BigUint::one() << (p - 2 - calc_precision);

    let scaled_1 = signed::ONE.deref() << (-calc_precision);
    let big_op_squared: BigInt = i * i;
    let inverse: BigInt = scaled_1 / i;

    let mut current_power = inverse.clone();
    let mut current_term = inverse.clone();
    let mut sum = inverse;

    let mut sign = 1;
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power /= &big_op_squared;
        sign = -sign;
        let signed_n: BigInt = (n * sign).into();
        current_term = &current_power / signed_n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate atan(c) for |c| < 1/2.
fn atan_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| is small. Larger atan inputs are reduced by
    // subtraction of atan(1/2) or the reciprocal identity before reaching here.
    // That reduction-before-series shape follows the elementary-function
    // approach in Brent, https://doi.org/10.1145/321941.321944.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-trig benches
    // run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_rational_small(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same Taylor kernel as `atan_computable`, but exact rational leaves can
    // provide the working approximation directly. This removes a Computable
    // child approximation call from tiny and residual atan reductions.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_anchor_residual(r: &Rational, anchor_numerator: u8, anchor_denominator: u8) -> Rational {
    // atan(r) - atan(a/b) = atan((r-a/b)/(1+r*a/b)). For exact positive
    // rationals, compute the residual from numerator/denominator parts in one
    // pass instead of constructing several temporary Rational values.
    let numerator_positive = r.numerator() * anchor_denominator;
    let numerator_negative = r.denominator() * anchor_numerator;
    let numerator = BigInt::from_biguint(Sign::Plus, numerator_positive)
        - BigInt::from_biguint(Sign::Plus, numerator_negative);
    let denominator = r.denominator() * anchor_denominator + r.numerator() * anchor_numerator;
    Rational::from_bigint_fraction(numerator, denominator)
        .expect("atan anchor residual denominator is positive")
}

fn atan_sqrt_rational_small(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same Taylor kernel as atan_rational_small for inputs known to satisfy
    // sqrt(r) <= 1/2. The sqrt is formed as an integer approximation directly
    // from the Rational, avoiding a transient Sqrt node followed by PrescaledAtan.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let sqrt_extra = 8;
    let op_appr = shift(ratio(r, 2 * op_prec - sqrt_extra).sqrt(), -(sqrt_extra / 2));
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atan_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact rational atan keeps the public constructor shallow and performs
    // range reduction directly here. The identities are the same as
    // Computable::atan: odd symmetry, atan(x)=pi/2-atan(1/x) for x>=2, and
    // atan(x)=atan(1/2)+atan((x-1/2)/(1+x/2)) in the middle interval.
    crate::trace_dispatch!("computable_approx", "atan", "exact-rational-reduction");
    match r.sign() {
        Sign::NoSign => return Zero::zero(),
        Sign::Minus => return -atan_rational(signal, &(-r.clone()), p),
        Sign::Plus => {}
    }

    if r.numerator() == &BigUint::one() {
        let denominator = BigInt::from_biguint(Sign::Plus, r.denominator().clone());
        if denominator > *signed::ONE.deref() {
            return atan(signal, &denominator, p);
        }
    }

    let half = HALF_RATIONAL.deref();
    if r <= &half {
        return atan_rational_small(signal, r, p);
    }

    if r.denominator() == &BigUint::one() && r.numerator() > &BigUint::one() {
        crate::trace_dispatch!("computable_approx", "atan", "large-integer-reciprocal");
        let extra = 3;
        let work_precision = p - extra;
        let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
        let denominator = BigInt::from_biguint(Sign::Plus, r.numerator().clone());
        let reduced = atan(signal, &denominator, work_precision);
        return scale(half_pi - reduced, -extra);
    }

    if r.msd_exact().is_some_and(|msd| msd >= 1) {
        let extra = 3;
        let work_precision = p - extra;
        let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
        let reciprocal = r.clone().inverse().expect("positive rational is nonzero");
        let reduced = atan_rational(signal, &reciprocal, work_precision);
        return scale(half_pi - reduced, -extra);
    }

    let extra = 3;
    let work_precision = p - extra;
    if r >= SEVEN_FOURTHS_RATIONAL.deref() && r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "two-anchor-half-pi");
        let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
        let anchor_tail =
            Computable::atan_inv2_constant().approx_signal(signal, work_precision + 1);
        let residual = atan_anchor_residual(r, 2, 1);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(half_pi - anchor_tail + reduced, -extra);
    }
    if r >= FOUR_THIRDS_RATIONAL.deref() && r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "three-halves-anchor");
        let quarter_pi = Computable::pi().approx_signal(signal, work_precision + 2);
        let anchor_tail =
            Computable::atan_inv5_constant().approx_signal(signal, work_precision + 2);
        let residual = atan_anchor_residual(r, 3, 2);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(quarter_pi + anchor_tail + scale(reduced, 2), -(extra + 2));
    }
    if r <= TWO_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "atan", "unit-anchor-pi-quarter");
        let quarter_pi = Computable::pi().approx_signal(signal, work_precision + 2);
        let residual = atan_anchor_residual(r, 1, 1);
        let reduced = atan_rational_small(signal, &residual, work_precision);
        return scale(quarter_pi + scale(reduced, 2), -(extra + 2));
    }

    let anchor = atan(signal, signed::TWO.deref(), work_precision);
    let residual = atan_anchor_residual(r, 1, 2);
    let reduced = atan_rational_small(signal, &residual, work_precision);
    scale(anchor + reduced, -extra)
}

fn asin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact rational asin uses the same direct series as `asin_computable`, but
    // bypasses the child Computable approximation. This is only selected after
    // construction certifies a tiny/moderate positive input, so the series
    // remains convergent enough to beat the generic sqrt/atan transform.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = (2 * n - 1) * (2 * n - 1);
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate asin(c) for small |c|.
fn asin_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asin series. It avoids the generic atan/sqrt
    // transform, which is overkill and slower when |x| is already very small.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-trig benches
    // run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = (2 * n - 1) * (2 * n - 1);
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn asin_deferred(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Generic asin uses asin(x) = 2*atan(x / (sqrt(1-x^2)+1)).
    // Keeping this as one approximation node mirrors the deferred acos and
    // inverse-hyperbolic reductions: construction preserves the compact input
    // graph, while approximation still enters the same stable transform.
    let one = Computable::one();
    let denominator = one.clone().add(c.clone().square().negate()).sqrt().add(one);
    c.clone()
        .multiply(denominator.inverse())
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_positive(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Positive-domain acos uses 2*atan(sqrt((1-x)/(1+x))). Keeping it as one
    // approximation node makes public construction cheap for endpoint-heavy
    // inverse trig rows while preserving the existing stable reduction.
    let one = Computable::one();
    let numerator = one.clone().add(c.clone().negate());
    let denominator = one.add(c.clone());
    numerator
        .multiply(denominator.inverse())
        .sqrt()
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_positive_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact-rational endpoint inputs benefit from forming the half-angle
    // residual directly. This follows the same cancellation-avoiding reduction
    // as `acos_positive` while avoiding a temporary Computable division tree.
    let residual = (Rational::one() - r.clone()) / (Rational::one() + r.clone());
    if residual <= Rational::fraction(1, 4).expect("nonzero denominator") {
        crate::trace_dispatch!("computable_approx", "acos", "small-rational-residual");
        return atan_sqrt_rational_small(signal, &residual, p - 1);
    }
    Computable::sqrt_rational(residual)
        .atan()
        .shift_left(1)
        .approx_signal(signal, p)
}

fn acos_negative_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    Computable::pi().approx_signal(signal, p) - acos_positive_rational(signal, r, p)
}

fn acosh_near_one(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near one, acosh(x) is ln1p((x - 1) + sqrt(x^2 - 1)). Deferring the graph
    // keeps construction cheap for endpoint-adjacent scalar rows without
    // changing the cancellation-avoiding approximation identity.
    let one = Computable::one();
    let shifted = c.clone().add(one.clone().negate());
    let radicand = c.clone().square().add(one.negate());
    shifted
        .add(radicand.sqrt())
        .ln_1p()
        .approx_signal(signal, p)
}

fn acosh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large acosh inputs use ln(x + sqrt(x^2 - 1)); this node is used by Real
    // construction paths where allocating that graph eagerly is the bottleneck.
    let one = Computable::one();
    let radicand = c.clone().square().add(one.negate());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

fn asinh_near_zero(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near zero, asinh(x) is evaluated through ln1p(x + x^2/(sqrt(1+x^2)+1)).
    // This deferred node removes construction overhead but preserves the
    // cancellation-resistant formula used by the public Real path.
    let square = c.clone().square();
    let one = Computable::one();
    let denominator = square.clone().add(one.clone()).sqrt().add(one);
    c.clone()
        .add(square.multiply(denominator.inverse()))
        .ln_1p()
        .approx_signal(signal, p)
}

fn asinh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large asinh inputs use ln(x + sqrt(1+x^2)); deferring the direct identity
    // keeps scalar construction from allocating the sqrt/log graph eagerly.
    let radicand = c.clone().square().add(Computable::one());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

// Approximate asinh(c) for small |c|.
fn asinh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asinh series. It avoids constructing the generic
    // ln(x + sqrt(1+x^2)) or ln1p expression for exact tiny rational inputs.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // This is the asin recurrence with alternating sign:
    // asinh(x) = x - x^3/6 + 3x^5/40 - ...
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = -((2 * n - 1) * (2 * n - 1));
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn asinh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny asinh series. It is the same
    // recurrence as `asinh_computable`, but it feeds the stored Rational
    // straight to the final precision request instead of allocating a temporary
    // Ratio node and child cache.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = -((2 * n - 1) * (2 * n - 1));
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atanh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Endpoint atanh construction should not eagerly allocate the full exact
    // log-ratio graph. Approximation still uses the stable identity
    // atanh(x) = 1/2 * ln((1+x)/(1-x)).
    let one = Computable::one();
    let numerator = one.clone().add(c.clone());
    let denominator = one.add(c.clone().negate());
    numerator
        .multiply(denominator.inverse())
        .ln()
        .multiply(Computable::rational(HALF_RATIONAL.clone()))
        .approx_signal(signal, p)
}

// Approximate atanh(c) for small |c|.
fn atanh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument atanh series, also reused by the ln1p kernel after
    // it transforms ln(1+x) into 2*atanh(x/(2+x)).
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-hyperbolic
    // benches run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atanh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny atanh series. This mirrors the
    // direct rational trig kernels: preserve the symbolic rational payload and
    // round only once at the requested working precision.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}
