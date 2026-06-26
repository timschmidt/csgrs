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

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    Expm1(Computable),
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
    ErfSeries(Computable),
    Erfc(Computable),
    NormalSf(Computable),
    NormalInterval { lo: Computable, hi: Computable },
    LogPnorm(Computable),
    LogNormalSf(Computable),
    LogDnorm(Computable),
    NormalQuantile {
        p: Computable,
        seed: BigInt,
        seed_prec: Precision,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    Atan2,
    AtanThreeHalves,
}
