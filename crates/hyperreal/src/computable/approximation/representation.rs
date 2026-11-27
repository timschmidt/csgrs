/// Shared exact 1/2 for averaging, half-angle reductions and rational interval midpoints.
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
/// Exact 4/5 upper threshold for the rational atan two-thirds anchor after earlier guards decline.
static FOUR_FIFTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(4, 5).unwrap());
/// Exact 4/3 lower threshold for the rational atan three-halves anchor, after the two anchor is tried.
static FOUR_THIRDS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(4, 3).unwrap());
/// Exact 7/4 lower threshold for choosing the rational atan anchor two.
static SEVEN_FOURTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 4).unwrap());
/// Shared exact two, including the upper comparison bound for rational atan anchor selection.
static TWO_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(2));
/// Exact 7/2 start of the positive fixed half-pi-multiple-two sector, subject to its magnitude guard.
static SEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 2).unwrap());
/// Exact 39/10 end of the positive fixed half-pi-multiple-two sector.
static THIRTY_NINE_TENTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(39, 10).unwrap());
/// Exact 79/20 start of the fixed half-pi-multiple-three sector below four.
static SEVENTY_NINE_TWENTIETHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(79, 20).unwrap());
/// Exact four separating the two adjacent positive fixed-multiple-three comparisons.
static FOUR_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(4));
/// Exact 27/5 end of the positive fixed half-pi-multiple-three sector.
static TWENTY_SEVEN_FIFTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(27, 5).unwrap());
/// Exact 11/2 start of the positive fixed half-pi-multiple-four sector.
static ELEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(11, 2).unwrap());
/// Exact seven shared by the positive four/five sector comparisons; the earlier four branch owns equality.
static SEVEN_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(7));
/// Written 17/2 upper comparison for the positive five sector; the prior magnitude guard actually requires input below eight.
static SEVENTEEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(17, 2).unwrap());
/// Integer 128 used against a precision -8 half-pi residual sample to choose a tangent quarter-pi anchor side; this is not an exact half-radian equality test.
static QUARTER_PI_TAN_RESIDUAL_THRESHOLD: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(128));
/// Exact minus four separating the two adjacent negative fixed-multiple-three comparisons.
static NEG_FOUR_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(-4));
/// Integer minus four identifying the dedicated negative two-pi residual path without constructing a rational.
static NEG_FOUR_BIGINT: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(-4));
/// Exact -7/2 bound for the reflected fixed half-pi-multiple-minus-two sector.
static NEG_SEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-7, 2).unwrap());
/// Exact -39/10 bound for the reflected fixed half-pi-multiple-minus-two sector.
static NEG_THIRTY_NINE_TENTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-39, 10).unwrap());
/// Exact -79/20 bound for the reflected fixed half-pi-multiple-minus-three sector near minus four.
static NEG_SEVENTY_NINE_TWENTIETHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-79, 20).unwrap());
/// Exact -27/5 end of the negative fixed half-pi-multiple-minus-three sector.
static NEG_TWENTY_SEVEN_FIFTHS_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-27, 5).unwrap());
/// Exact -11/2 start of the negative fixed half-pi-multiple-minus-four sector.
static NEG_ELEVEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-11, 2).unwrap());
/// Exact minus seven shared by reflected four/five sectors; the earlier minus-four branch owns equality.
static NEG_SEVEN_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::new(-7));
/// Written -17/2 comparison for the negative five sector; the prior magnitude guard actually requires input greater than minus eight.
static NEG_SEVENTEEN_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(-17, 2).unwrap());

/// Three-term exact sum of computable coefficients times rational weights.
/// Keep the fixed form compact and evaluate it without first constructing three
/// products and two sums. Children may share nodes/caches; cloning the form copies
/// the wrappers and rational weights. Zero weights can skip numeric child work,
/// so valid child domains must be established independently of that optimization.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(super) struct LinearCombination3 {
    /// Exact computable factors in evaluation order zero, one, two; shared child nodes are retained.
    pub(super) coefficients: [Computable; 3],
    /// Exact rational weights paired by index with coefficients; a zero weight permits skipping numerical child evaluation.
    pub(super) values: [Rational; 3],
}

/// Exact expression recipes consumed by approximation, structural analysis and native replay.
/// A variant denotes its mathematical expression, not a completed integer sample.
/// Domain and local-kernel preconditions below are obligations, not facts established
/// by selecting a tag. Several Prescaled variants recheck their local range and
/// reduce on demand; others rely on construction. General graph admission, exact
/// facts and approximation caches belong to the surrounding Node/Computable layers.
///
/// Derived Clone preserves shared Computable child allocations while copying owned
/// integer/rational payloads. Derived Debug exposes structure, not a numerical
/// projection. With serde, derived enum decoding and the nth-root degree hook are
/// followed by Node's bounded domain/range rejection checks; inconclusive checks do
/// not prove every arbitrary graph valid. Cache contents and cancellation handles
/// are not supplied by these payloads as mathematical evidence.
///
/// Keep variant order and payload grammar stable for native consumers. Serde derives
/// assign declaration-order variant indices, but this repository's JSON and Ciborium
/// CBOR encodings use variant names; indices are not a universal binary-wire layout.
/// Boxed uncommon payloads keep their data out of the enum's inline storage. No
/// uniform completion, work, memory or cancellation guarantee follows from a finite
/// recipe: kernels retain their documented precision and coarse-result limitations.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(super) enum Approximation {
    /// Exact signed integer leaf, also available to structural sign and magnitude recognition.
    Int(
        /// Authoritative integer value, with no binary exponent or approximation error.
        BigInt,
    ),
    /// Exact one without the owned BigInt payload of an Int(1) leaf.
    One,
    /// A named exact constant whose numerical requests can reuse the shared process-wide cache.
    Constant(
        /// Constant identity; its cache slot is selected by cache_index, not allocation identity.
        SharedConstant,
    ),
    /// Reciprocal 1/c for an admitted nonzero child; the kernel is not a typed domain validator.
    Inverse(
        /// Exact denominator whose nonzero domain must be justified by the caller.
        Computable,
    ),
    /// Exact sign reversal, retaining the child instead of changing its samples or caches.
    Negate(
        /// Defined real expression to negate.
        Computable,
    ),
    /// Exact sum; the outer evaluator can flatten supported addition chains to avoid recursive stack growth.
    Add(
        /// First addend, retaining its own node and wrapper controls.
        Computable,
        /// Second addend; child decision/evaluation order remains an implementation concern.
        Computable,
    ),
    /// Exact product after any constructor simplifications; no eager operand sampling is implied.
    Multiply(
        /// First factor of the retained product.
        Computable,
        /// Second factor; a zero product does not by itself validate an undefined child.
        Computable,
    ),
    /// Fixed three-term exact dot product without an expanded product/sum graph.
    LinearCombination3(
        /// Shared computable coefficients paired by index with exact rational weights.
        Box<LinearCombination3>,
    ),
    /// Exact c^2, exposing nonnegativity and potential root cancellation without immediate refinement.
    Square(
        /// Defined real source whose sign can matter when simplifying sqrt(c^2).
        Computable,
    ),
    /// Exact rational leaf, including exact binary values adopted from finite floating-point ingress.
    Ratio(
        /// Authoritative normalized rational value; this is not a rounded evaluation sample.
        Rational,
    ),
    /// Exact binary scaling c*2^n, generally evaluated through a shifted precision request.
    Offset(
        /// Source expression to scale.
        Computable,
        /// Signed binary exponent; representable storage does not ensure all evaluation precision arithmetic or allocations fit.
        i32,
    ),
    /// exp(c), with small-series range certification and fallback reduction performed on demand.
    PrescaledExp(
        /// Defined finite exponent; the tag alone does not assert a small magnitude.
        Computable,
    ),
    /// Exact exp(c)-1 with a cancellation-aware evaluator for small exponents.
    Expm1(
        /// Defined finite exponent retained before subtracting one.
        Computable,
    ),
    /// Nonnegative square root of an admitted nonnegative real child.
    Sqrt(
        /// Radicand; this variant does not define a complex root for negative values.
        Computable,
    ),
    /// ln(1+c), with local series range certification and fallback reduction at evaluation time.
    PrescaledLn(
        /// Additive residual c>-1, not the positive logarithm input 1+c.
        Computable,
    ),
    /// ln(1+r) from an exact rational residual, avoiding a temporary Ratio child during sampling.
    PrescaledLnRational(
        /// Exact residual r>-1; the rational evaluator handles its local range.
        Rational,
    ),
    /// ln(1+residual)+shift*ln(2), retaining exact logarithmic scaling without an extra sum graph.
    BinaryScaledLnRational {
        /// Exact additive log1p residual greater than minus one.
        residual: Rational,
        /// Signed coefficient of ln(2), not a precision request or allocated power-of-two integer.
        shift: i32,
    },
    /// Principal atan(1/n), using integer-denominator series and exact unit-denominator handling.
    IntegralAtan(
        /// Nonzero integer denominator; the geometric series needs abs(n)>=2 and unit magnitudes use an identity.
        BigInt,
    ),
    /// Principal atan(c), certifying the local half-unit series range or reducing further on demand.
    PrescaledAtan(
        /// Defined finite argument; an imported prescaled tag is not a range proof.
        Computable,
    ),
    /// Principal atan(c) with convergence reduction deferred until evaluation.
    AtanDeferred(
        /// Exact source retained so later inverse-trig relations can recover it.
        Computable,
    ),
    /// Principal atan(r), preserving an exact rational for native anchor/reciprocal reduction.
    AtanRational(
        /// Signed rational argument; extreme coefficient sizes retain the evaluator's documented range/resource limitations.
        Rational,
    ),
    /// Principal asin(r) on the closed real unit interval, using rational small-series or endpoint reduction.
    AsinRational(
        /// Signed exact argument in `[-1,1]`, including both endpoints.
        Rational,
    ),
    /// Principal asin(c) on `[-1,1]`, with a certified tiny-series range or a deferred transform.
    PrescaledAsin(
        /// Admitted real argument; the name does not certify the local series range.
        Computable,
    ),
    /// Principal asin(c) through a deferred half-angle atan/sqrt identity, including both real endpoints.
    AsinDeferred(
        /// Exact argument in `[-1,1]`.
        Computable,
    ),
    /// Principal acos(c) for 0<=c<=1 using the endpoint-stable half-angle identity.
    AcosPositive(
        /// Nonnegative argument, not a residual from one.
        Computable,
    ),
    /// Principal acos(r) for an exact argument in `[0,1]`, with direct rational half-angle residuals.
    AcosPositiveRational(
        /// Nonnegative rational argument at or below one.
        Rational,
    ),
    /// Principal acos(-r)=pi-acos(r), storing the magnitude rather than a negative input.
    AcosNegativeRational(
        /// Nonnegative magnitude r in `[0,1]`; the represented acos input is -r.
        Rational,
    ),
    /// Principal acosh(c) for c>=1 via a near-one log1p identity; no separate upper domain bound is implied.
    AcoshNearOne(
        /// Admitted argument at or above one.
        Computable,
    ),
    /// Principal acosh(c)=ln(c+sqrt(c^2-1)) with graph expansion deferred.
    AcoshDirect(
        /// Admitted argument c>=1; the identity also holds at one.
        Computable,
    ),
    /// asinh(c) via ln1p(c+c^2/(sqrt(1+c^2)+1)), preserving small differences.
    AsinhNearZero(
        /// Defined finite argument; near-zero describes scheduling, not a mathematical domain restriction.
        Computable,
    ),
    /// asinh(c)=ln(c+sqrt(1+c^2)) evaluated through a deferred exact graph.
    AsinhDirect(
        /// Defined finite argument; callers may apply odd symmetry to avoid large negative cancellation.
        Computable,
    ),
    /// asinh(c), certifying a local half-unit series range or reducing by sign.
    PrescaledAsinh(
        /// Defined finite argument, whose range is checked at evaluation rather than proved by the tag.
        Computable,
    ),
    /// asinh(r) with an exact rational small-series path and sign-normalized fallback.
    AsinhRational(
        /// Signed rational argument retained without a temporary Ratio node.
        Rational,
    ),
    /// Principal atanh(c)=ln((1+c)/(1-c))/2 on the open real unit interval.
    AtanhDirect(
        /// Admitted argument -1<c<1; endpoints are not finite values of this recipe.
        Computable,
    ),
    /// Principal atanh(c) with certified tiny-series scheduling or a direct logarithmic ratio.
    PrescaledAtanh(
        /// Admitted argument in (-1,1); local range proof is separate from this domain obligation.
        Computable,
    ),
    /// Principal atanh(r) retaining the rational for tiny-series or native log-ratio reduction.
    AtanhRational(
        /// Exact rational argument strictly between minus one and one.
        Rational,
    ),
    /// Reduced cosine with nominal abs(c)<1; the kernel does not validate that range.
    PrescaledCos(
        /// Exact reduced angle; internal tangent scheduling also reaches wider arguments, as documented by the kernel.
        Computable,
    ),
    /// Cosine of an admitted rational abs(r)<1 without a temporary sampled leaf.
    PrescaledCosRational(
        /// Signed rational angle in the local cosine range.
        Rational,
    ),
    /// Cosine of an exact rational, deferring half-pi reduction and quadrant selection until digits are requested.
    CosLargeRational(
        /// Authoritative rational angle; reduction does not replace it by a rounded residual.
        Rational,
    ),
    /// cos(pi/2-r) for the admitted rational interval 1<=r<3/2.
    PrescaledCosHalfPiMinusRational(
        /// Exact r, not the generally irrational reduced angle pi/2-r.
        Rational,
    ),
    /// Reduced sine with nominal abs(c)<1 and no local range-validation step.
    PrescaledSin(
        /// Exact reduced angle; the kernel separately records wider internal tangent scheduling.
        Computable,
    ),
    /// Sine of an admitted rational abs(r)<1 using direct rational sampling.
    PrescaledSinRational(
        /// Signed rational angle in the local sine range.
        Rational,
    ),
    /// Sine of an exact rational with lazy half-pi reduction and signed quadrant reconstruction.
    SinLargeRational(
        /// Original rational angle retained independently of temporary numerical residuals.
        Rational,
    ),
    /// sin(pi/2-r) for the admitted rational interval 1<=r<3/2.
    PrescaledSinHalfPiMinusRational(
        /// Exact r whose subtraction from pi/2 is sampled only during evaluation.
        Rational,
    ),
    /// cot(pi/2-r)=tan(r) for 1<=r<3/2; the current coarse-zero evaluator has a documented accuracy gap.
    PrescaledCotHalfPiMinusRational(
        /// Exact rational r; the payload is not a rational cotangent argument.
        Rational,
    ),
    /// Tangent of a retained rational using shared sine/cosine reduction; nonpoles and valid precision plans remain required.
    TanLargeRational(
        /// Exact angle; the current p>=1 zero path is not justified for every admitted angle.
        Rational,
    ),
    /// Reduced tangent, nominally abs(c)<1; wider native scheduling and its coarse-result gap are documented by the kernel.
    PrescaledTan(
        /// Exact nonpole angle; neither this tag nor a finite sample proves separation from a pole.
        Computable,
    ),
    /// Tangent of an admitted rational abs(r)<1 through direct component sampling and division.
    PrescaledTanRational(
        /// Signed rational angle in the nominal local tangent range.
        Rational,
    ),
    /// Cotangent of a nonzero reduced angle, nominally abs(c)<1; coarse zero is not a valid global bound.
    PrescaledCot(
        /// Exact angle away from the cotangent pole; small nonzero arguments can demand substantial refinement.
        Computable,
    ),
    /// Auxiliary odd series sqrt(pi)*exp(c^2)*erf(c)/2, not the normalized error function itself.
    ErfSeries(
        /// Defined finite series argument; the enclosing erf construction supplies Gaussian and normalization factors.
        Computable,
    ),
    /// Complementary error function 1-erf(c), retaining an exact complement before refinement.
    Erfc(
        /// Defined finite error-function argument.
        Computable,
    ),
    /// Standard normal survival probability Phi(-c), mathematically positive for finite c.
    NormalSf(
        /// Defined finite standardized argument, without the public Real adapter's practical-window check.
        Computable,
    ),
    /// Signed mass Phi(hi)-Phi(lo); native reversed endpoints represent a negative difference.
    NormalInterval {
        /// First oriented endpoint, whose CDF is subtracted.
        lo: Computable,
        /// Second oriented endpoint, whose CDF is added; ordering is not checked by this payload.
        hi: Computable,
    },
    /// Natural logarithm of the standard normal CDF, evaluated through a retained CDF/log composition.
    LogPnorm(
        /// Defined finite standardized argument; its CDF lies strictly in (0,1).
        Computable,
    ),
    /// Natural logarithm of the standard normal survival probability.
    LogNormalSf(
        /// Defined finite standardized argument; no rounded tail probability is authoritative.
        Computable,
    ),
    /// Analytic standard normal log density -c^2/2-ln(2*pi)/2.
    LogDnorm(
        /// Defined finite standardized argument, avoiding a tiny density intermediate.
        Computable,
    ),
    /// Unique standard normal inverse for a defined probability in (0,1), with a nonauthoritative seed.
    NormalQuantile(
        /// Probability definition and raw dyadic starting hint; exact relations or CDF bounds certify outputs.
        Box<NormalQuantileData>,
    ),
    /// Nonnegative algebraic nth root; native degree scheduling and serde admit degrees three through nine.
    NthRoot(
        /// Nonnegative real radicand even for odd degrees; signed odd roots are assembled outside this node.
        Computable,
        /// Degree constrained by deserialize_nth_root_degree on replay; the degree alone does not validate the radicand.
        #[cfg_attr(
            feature = "serde",
            serde(deserialize_with = "deserialize_nth_root_degree")
        )]
        u32,
    ),
    /// Analytic sin(c)/c on the constructor-certified range abs(c)<=3/4, with value one at zero.
    SincSmall(
        /// Exact small angle; evaluation avoids deciding whether this child is zero.
        Computable,
    ),
    /// Analytic (1-cos(c))/c^2 on abs(c)<=3/4, with value one half at zero.
    CoscSmall(
        /// Exact small angle whose range must be admitted independently of the tag.
        Computable,
    ),
    /// Unique real geodetic latitude in (-pi/2,pi/2) for finite isometric input and positive encoded axis ratio.
    InverseIsometricLatitude {
        /// Defined finite isometric latitude used as the exact inverse-equation target.
        psi: Computable,
        /// Defined finite ln(b/a), encoding positive polar/equatorial ratio exp(log_axis_ratio).
        log_axis_ratio: Computable,
    },
    /// Carlson R_F(x,y,z) with nonnegative x and positive y,z encoded through root/log children.
    EllipticRf(
        /// Ordered payload `[sqrt(x),ln(y),ln(z)]` after any constructor permutation; finite defined children are required.
        Box<[Computable; 3]>,
    ),
    /// Carlson R_D(x,y,z), retaining the distinguished positive z argument in its third payload position.
    EllipticRd(
        /// Payload `[sqrt(x),ln(y),ln(z)]` with nonnegative x and positive y,z; only the symmetric first pair may be permuted.
        Box<[Computable; 3]>,
    ),
    /// Continuous unwrapped incomplete Legendre F(phi|m), with m=1-exp(log_complement)<1.
    EllipticF(
        /// Payload `[phi,ln(1-m)]` with both children defined and finite; a serialized tag alone does not prove those premises.
        Box<[Computable; 2]>,
    ),
    /// Continuous unwrapped incomplete Legendre E(phi|m), including negative parameters through its log complement.
    EllipticE(
        /// Payload `[phi,ln(1-m)]` of defined finite children, not a wrapped amplitude or sampled integral.
        Box<[Computable; 2]>,
    ),
    /// Unique unwrapped inverse of F for m<1, retaining the input integral value rather than a finite root iterate.
    JacobiAmplitude(
        /// Payload `[u,ln(1-m)]` of defined finite children; the output is the amplitude phi.
        Box<[Computable; 2]>,
    ),
    /// Unique unwrapped real inverse of E for m<1.
    InverseEllipticE(
        /// Payload `[E_value,ln(1-m)]` of defined finite children; numerical brackets never replace this definition.
        Box<[Computable; 2]>,
    ),
    /// sin(phi) at a retained Jacobi amplitude, sharing its source and parameter with related components.
    JacobiSn(
        /// Payload `[phi,ln(1-m)]`; phi is already the amplitude, not the original Jacobi argument u.
        Box<[Computable; 2]>,
    ),
    /// cos(phi) at a retained Jacobi amplitude, preserving shared square-sum relations.
    JacobiCn(
        /// Payload `[phi,ln(1-m)]` with defined finite children and the same interpretation as JacobiSn.
        Box<[Computable; 2]>,
    ),
    /// Positive sqrt(cos(phi)^2+exp(log_complement)*sin(phi)^2) for defined finite payloads.
    JacobiDn(
        /// Payload `[phi,ln(1-m)]`; positivity depends on defined children, not merely possession of this tag.
        Box<[Computable; 2]>,
    ),
}

/// Decode a u32 degree and reject values outside the native direct-root range 3..=9.
/// Return the deserializer's typed error for malformed integers or unsupported
/// degrees. This field-level guard neither validates the radicand nor establishes
/// a general algebraic-degree completeness or total-resource bound.
#[cfg(feature = "serde")]
fn deserialize_nth_root_degree<'de, D>(deserializer: D) -> Result<u32, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let degree = u32::deserialize(deserializer)?;
    if (3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE).contains(&degree) {
        Ok(degree)
    } else {
        Err(serde::de::Error::custom(
            "direct nth-root degree is outside the supported range",
        ))
    }
}

/// Retained probability and dyadic starting hint for the unique standard-normal inverse.
/// The probability defines the result; the seed only schedules trial points. Exact
/// CDF reconstruction or numerical CDF enclosures supply evidence. Boxing this data
/// avoids reserving its owned integer hint inline in every expression variant.
/// Derived cloning/serialization retain the hint without certifying it or adding a
/// total-work bound; the constructor/evaluator assume a defined interior probability.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(super) struct NormalQuantileData {
    /// Exact probability definition, required to be finite and strictly between zero and one.
    pub(super) p: Computable,
    /// Untrusted signed integer numerator of a dyadic starting hint; it is never a result certificate.
    pub(super) seed: BigInt,
    /// Binary exponent of seed*2^seed_prec, distinct from requested output precision and any resource budget.
    pub(super) seed_prec: Precision,
}

/// Names for eighteen exact constants with shared process-wide approximation slots.
/// Copy/Clone and derived equality compare these keys; numerical evaluation uses
/// the separate exhaustive cache_index mapping and constant kernels. Rational
/// bounds and symbolic identities remain distinct from cached integer samples.
/// Serde names/variant indices and approximation-cache indices serve different
/// interfaces, even though their present orders agree. A slot count bounds neither
/// retained integer bytes nor the work required by a precision request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(super) enum SharedConstant {
    /// Euler's number exp(1), evaluated through the factorial-series constant kernel.
    E,
    /// The positive circle constant pi, using the shared pi approximation schedule.
    Pi,
    /// The exact reciprocal of pi, with its own shared approximation slot.
    InvPi,
    /// The exact full-turn angle 2*pi, distinct as a cache key from Pi.
    Tau,
    /// Natural logarithm of two, reused by binary range reduction.
    Ln2,
    /// Natural logarithm of three for shared prime-log reductions.
    Ln3,
    /// Natural logarithm of five for shared prime-log reductions.
    Ln5,
    /// Natural logarithm of six, retaining a dedicated named constant and cache slot.
    Ln6,
    /// Natural logarithm of seven for shared prime-log reductions.
    Ln7,
    /// Natural logarithm of ten for decimal-scale logarithmic reductions.
    Ln10,
    /// Positive square root of two, shared by half-angle and Gaussian constructions.
    Sqrt2,
    /// Positive square root of three as an exact named radical.
    Sqrt3,
    /// Principal acosh(2)=ln(2+sqrt(3)).
    Acosh2,
    /// asinh(1)=ln(1+sqrt(2)).
    Asinh1,
    /// Principal atan(1/2), used by retained rational-angle reductions.
    AtanInv2,
    /// Principal atan(1/5), also supplying complementary rational-angle anchors.
    AtanInv5,
    /// Principal atan(2), not the two-argument atan2 operation.
    Atan2,
    /// Principal atan(3/2), a shared middle-range rational-angle anchor.
    AtanThreeHalves,
}
