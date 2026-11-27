#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg(not(verus_keep_ghost))]
/// Retained exact-real expression with shared accelerator state and a per-wrapper stop signal.
/// Approximation is demand-driven for valid graphs and usable precision plans;
/// this type does not guarantee completion or validate every native construction.
/// Clone shares the node/caches and clones the signal handle; Debug is structural
/// and may expose diagnostic cache contents, not exact interchange.
///
/// With serde, recipes are serialized without signals or accelerator facts.
/// Arc contents are replayed as values, not shared allocation identities. Native
/// decoding rejects recognized invalid domains/ranges but may leave opaque ones
/// unproved. [`Real`](crate::Real) is the checked carrier whose import additionally
/// proves operand domains; see its serde-enabled from_json API. Successful native
/// decoding alone does not certify arbitrary expression validity.
pub struct Computable {
    /// Shared immutable recipe and its node-owned accelerators; cloning preserves this allocation.
    pub(super) internal: Arc<Node>,
    #[cfg_attr(feature = "serde", serde(skip))]
    /// Optional wrapper-specific stop handle; omitted on serialization and not part of scalar identity.
    pub(crate) signal: Option<Signal>,
}
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize), serde(transparent))]
#[cfg(not(verus_keep_ghost))]
/// Immutable exact recipe with shared atomic facts and a lazily allocated evaluation cache.
/// Constructors supply trusted facts; query/publication layers must justify them.
/// Serde writes only the transparent approximation payload, then reconstructs
/// admissible evidence on decoding rather than trusting cache bits.
pub(super) struct Node {
    #[cfg_attr(feature = "serde", serde(skip, default))]
    /// Packed caller-certified facts and nonmathematical scheduling hints, reset on replay.
    facts: AtomicFacts,
    /// Authoritative retained recipe whose operands and domains determine this node's meaning.
    approximation: Approximation,
    #[cfg_attr(feature = "serde", serde(skip, default))]
    /// Lazily allocated numeric/tower reuse state; not serialized proof.
    cache: EvaluationCache,
}

#[cfg(all(feature = "serde", not(verus_keep_ghost)))]
/// Decode native node payloads with rejection-only admission and fresh accelerator state.
impl<'de> Deserialize<'de> for Node {
    /// Decode the transparent recipe, reject recognized invalid domains/ranges, then reset accelerators.
    /// Children decode before this check. Only reduced-pi sine/tangent sign evidence
    /// is reconstructed here; other facts and the evaluation cache start at defaults.
    /// Serde errors describe refuted input, while successful decoding can still leave
    /// opaque domains unproved. No incoming cache, sign flag or shared Arc identity
    /// is accepted as proof, and this method imposes no whole-payload resource cap.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        // Preserve the transparent wire representation and default cache state.
        // Replay this bounded proof from the expression itself so a subsequent
        // bounded reducer can consume it without first warming a numeric cache.
        let approximation = Approximation::deserialize(deserializer)?;
        Self::check_serialized_rational_domain(&approximation).map_err(serde::de::Error::custom)?;
        Self::check_serialized_kernel_parameters(&approximation)
            .map_err(serde::de::Error::custom)?;
        let facts = AtomicFacts::default();
        if let Approximation::PrescaledSin(argument) | Approximation::PrescaledTan(argument) =
            &approximation
            && let Some(sign) = argument.reduced_pi_multiple_sign()
        {
            facts.replace_exact_sign(ExactSignCache::Valid(sign));
        }
        Ok(Self {
            facts,
            approximation,
            cache: EvaluationCache::default(),
        })
    }
}

#[cfg(all(feature = "serde", not(verus_keep_ghost)))]
/// Reject cheaply established invalid serialized domains and selected kernel parameters.
impl Node {
    /// Test a rejection predicate against an exact rational or a bounded domain contradiction.
    /// Try exact recognition, then the defined-rational evaluator, before the
    /// conditional symbolic contradiction route. A conditional identity may refute
    /// the requested domain but cannot certify a value or validate unknown children.
    /// False means no violation was established by these routes, not proven validity.
    fn serialized_argument_violates(
        value: &Computable,
        invalid: impl Fn(&Rational) -> bool,
    ) -> bool {
        // Preserve existing exact proofs, including large rational leaves.
        // The bounded fallback checks every operand and inverse denominator;
        // unlike general algebraic cancellation, it never erases an opaque
        // child's unresolved domain obligation, even under a zero factor.
        if let Some(rational) = value
            .exact_rational()
            .or_else(|| value.bounded_defined_rational())
        {
            return invalid(&rational);
        }
        // For rejection only, assume opaque dependencies denote reals and
        // ask whether the existing quotient proof contradicts the domain.
        // No conditional value/sign is published, including when it happens
        // to satisfy this domain: that does not prove its dependencies valid.
        value.bounded_domain_contradiction(invalid)
    }

    /// Reject recognized violations of selected stored kernel ranges without rewriting the graph.
    /// Check small rational trig magnitudes, the positive half-pi-complement window,
    /// and sinc/cosc's three-quarter range using rational contradictions or a named
    /// constant's certified binade. A missing magnitude or unrecognized expression
    /// can leave the range unproved. Ok is not general kernel-range admission; this
    /// step neither refines arbitrary children nor supplies a total resource bound.
    fn check_serialized_kernel_parameters(
        approximation: &Approximation,
    ) -> Result<(), &'static str> {
        // These variants store a reduced argument, not an arbitrary argument
        // to the corresponding total trigonometric function. Reuse the native
        // constructor's classification before accepting its range-dependent
        // sign facts and approximation bounds. Preserve the supplied graph.
        let invalid = match approximation {
            Approximation::PrescaledCosRational(r)
            | Approximation::PrescaledSinRational(r)
            | Approximation::PrescaledTanRational(r) => r.msd_exact().is_some_and(|msd| msd >= 0),
            Approximation::PrescaledCosHalfPiMinusRational(r)
            | Approximation::PrescaledSinHalfPiMinusRational(r)
            | Approximation::PrescaledCotHalfPiMinusRational(r) => {
                r.sign() != Sign::Plus
                    || Computable::exact_rational_half_pi_shortcut_magnitude(r).is_none()
            }
            Approximation::SincSmall(argument) | Approximation::CoscSmall(argument) => {
                if Self::serialized_argument_violates(argument, |rational| {
                    let limit = Rational::fraction(3, 4).expect("nonzero constant denominator");
                    rational.compare_magnitude(&limit) == Ordering::Greater
                }) {
                    true
                } else if let Approximation::Constant(constant) = &argument.internal.approximation {
                    // A certified MSD >= 0 proves |x| >= 1 > 3/4. Inspect only
                    // these trusted constants; do not refine an untrusted
                    // expression or mistake a planning estimate for a proof.
                    constant
                        .bound_info()
                        .known_msd()
                        .flatten()
                        .is_some_and(|msd| msd >= 0)
                } else {
                    // General expression domain/range admission remains open.
                    false
                }
            }
            _ => false,
        };
        if invalid {
            Err("Computable argument violates its stored kernel range")
        } else {
            Ok(())
        }
    }

    /// Reject recognized real-domain violations for the stored partial-function recipes.
    /// Interpret logarithm payloads as ln(1+x) residuals, negative-acos payloads as
    /// magnitudes and nth roots as nonnegative kernels. Closed/open interval endpoints
    /// follow the corresponding function; IntegralAtan requires a nonzero denominator.
    /// Children have already undergone decoding checks, so recognized invalid children
    /// cannot be erased by outer zero/cancellation. An inconclusive proof returns Ok,
    /// leaving complete checked admission to the Real import walk.
    fn check_serialized_rational_domain(approximation: &Approximation) -> Result<(), &'static str> {
        // Children have already passed the same check while decoding. Reject
        // an undefined child before a parent can hide it with multiplication
        // by zero, cancellation, or an outer Real scale. An inconclusive
        // bounded rational proof must not reject a valid graph.
        let outside_closed_unit = |r: &Rational| r < &-Rational::one() || r > &Rational::one();
        let outside_open_unit = |r: &Rational| r <= &-Rational::one() || r >= &Rational::one();
        let outside_positive_unit = |r: &Rational| r.sign() == Sign::Minus || r > &Rational::one();
        let invalid = match approximation {
            Approximation::Inverse(value) | Approximation::PrescaledCot(value) => {
                Self::serialized_argument_violates(value, |r| r.sign() == Sign::NoSign)
            }
            Approximation::Sqrt(value) | Approximation::NthRoot(value, _) => {
                Self::serialized_argument_violates(value, |r| r.sign() == Sign::Minus)
            }
            Approximation::PrescaledLn(value) => {
                Self::serialized_argument_violates(value, |r| r <= &-Rational::one())
            }
            // These variants store x for ln(1+x), not the logarithm's input.
            Approximation::PrescaledLnRational(r)
            | Approximation::BinaryScaledLnRational { residual: r, .. } => r <= &-Rational::one(),
            Approximation::IntegralAtan(denominator) => denominator.is_zero(),
            Approximation::AsinRational(r) => outside_closed_unit(r),
            Approximation::PrescaledAsin(value) | Approximation::AsinDeferred(value) => {
                Self::serialized_argument_violates(value, outside_closed_unit)
            }
            // Both rational endpoint variants store an absolute argument.
            Approximation::AcosPositiveRational(r) | Approximation::AcosNegativeRational(r) => {
                outside_positive_unit(r)
            }
            Approximation::AcosPositive(value) => {
                Self::serialized_argument_violates(value, outside_positive_unit)
            }
            Approximation::AtanhRational(r) => outside_open_unit(r),
            Approximation::PrescaledAtanh(value) | Approximation::AtanhDirect(value) => {
                Self::serialized_argument_violates(value, outside_open_unit)
            }
            Approximation::AcoshNearOne(value) | Approximation::AcoshDirect(value) => {
                Self::serialized_argument_violates(value, |r| r < &Rational::one())
            }
            Approximation::NormalQuantile(data) => {
                Self::serialized_argument_violates(&data.p, |r| {
                    r.sign() != Sign::Plus || r >= &Rational::one()
                })
            }
            _ => false,
        };
        if invalid {
            Err("Computable argument is outside its real-valued domain")
        } else {
            Ok(())
        }
    }
}

#[cfg(all(feature = "serde", not(verus_keep_ghost)))]
/// Apply conservative interval decisions to imported operand conditions.
impl Binary64Interval {
    /// Decide a domain condition from a valid outward interval, or leave it unresolved.
    /// Separate zero directly. Cosine's open (-3/2,3/2) and sine's open (0,3) or
    /// (-3,0) windows prove nonzero but do not refute other angles. For rational
    /// endpoints compare their outward enclosure, reversing bounds for Below and
    /// respecting inclusive flags. Some(false) is a refutation, None is inconclusive;
    /// this filter assumes a sound source enclosure and does not sample the graph.
    fn decide_imported_domain_condition(
        &self,
        condition: &ImportedDomainCondition,
    ) -> Option<bool> {
        let (lower, upper) = (self.lower, self.upper);
        let (endpoint, inclusive, above) = match condition {
            ImportedDomainCondition::NonZero => {
                return if lower > 0.0 || upper < 0.0 {
                    Some(true)
                } else if lower == 0.0 && upper == 0.0 {
                    Some(false)
                } else {
                    None
                };
            }
            // 3/2 < pi/2 and 3 < pi, so these windows contain no pole.
            ImportedDomainCondition::TrigNonZero(true) => {
                return (lower > -1.5 && upper < 1.5).then_some(true);
            }
            ImportedDomainCondition::TrigNonZero(false) => {
                return ((lower > 0.0 && upper < 3.0) || (lower > -3.0 && upper < 0.0))
                    .then_some(true);
            }
            ImportedDomainCondition::Above(endpoint, inclusive) => (endpoint, *inclusive, true),
            ImportedDomainCondition::Below(endpoint, inclusive) => (endpoint, *inclusive, false),
        };
        // Compare with an enclosure of the endpoint; unresolved overlap falls through.
        let endpoint = Self::from_rational(endpoint)?;
        let (near, far) = if above {
            (lower, upper)
        } else {
            (-upper, -lower)
        };
        let (endpoint_low, endpoint_high) = if above {
            (endpoint.lower, endpoint.upper)
        } else {
            (-endpoint.upper, -endpoint.lower)
        };
        if near > endpoint_high || (inclusive && near >= endpoint_high) {
            Some(true)
        } else if far < endpoint_low || (!inclusive && far <= endpoint_low) {
            Some(false)
        } else {
            None
        }
    }
}
#[cfg(all(feature = "serde", not(verus_keep_ghost)))]
/// One operand condition required before an imported partial operation may be consumed.
/// Inclusive endpoint tests, nonzeroness and trig-pole exclusion remain distinct;
/// false, unknown and successful proof must not be conflated.
enum ImportedDomainCondition {
    /// Require the operand itself to be nonzero.
    NonZero,
    /// Require the operand to lie above a rational endpoint.
    Above(
        /// Exact lower endpoint.
        Rational,
        /// True allows equality; false requires a strict inequality.
        bool,
    ),
    /// Require the operand to lie below a rational endpoint.
    Below(
        /// Exact upper endpoint.
        Rational,
        /// True allows equality; false requires a strict inequality.
        bool,
    ),
    /// Require a trigonometric denominator image to be nonzero.
    TrigNonZero(
        /// True selects cosine; false selects sine, preserving their different pole sets.
        bool,
    ),
}

#[cfg(all(feature = "serde", not(verus_keep_ghost)))]
/// Prove checked-import operand domains without letting parent identities hide invalid children.
impl Computable {
    /// Check imported partial-operation domains in operand-before-parent order.
    /// Native node decoding performs rejection-only checks. Real import invokes this
    /// walk before consuming the graph, including beneath zero outer scales. A
    /// refuted required condition returns ParseError; an unresolved one returns
    /// UnknownCertificate before its parent can refine the unproved operation.
    ///
    /// Use an inline-capacity-32 postorder stack which may grow on the heap. Serde
    /// expands Arc contents rather than preserving shared identities, so the imported
    /// tree is not deduplicated. A native DAG passed here may repeat shared work.
    /// Domain checks can refine already-admitted children; neither the stack capacity
    /// nor local precision/recognition limits bound total work, memory or cancellation.
    pub(crate) fn prove_imported_domains(&self) -> Result<(), crate::Problem> {
        // Decoding builds a tree: serde copies the value behind every shared
        // pointer, so the walk needs no visited set. Iterate in post-order so
        // deep payloads do not use the call stack.
        let mut stack = InlineStack::<(&Self, bool), 32>::new();
        stack.push((self, false));
        while let Some((value, expanded)) = stack.pop() {
            if expanded {
                value.prove_imported_operation_domain()?;
                continue;
            }
            stack.push((value, true));
            value
                .internal
                .approximation
                .for_each_operand(|operand| stack.push((operand, false)));
        }
        Ok(())
    }

    /// Prove this node's required conditions after its child domains have been established.
    /// Select zero/endpoint/trig-pole conditions for the handled partial recipes;
    /// a retained square suffices for a nonnegative radicand. Rational-only payload
    /// checks belong to decoding. An exact recognized value of this node can supply
    /// a defining-identity proof, such as quantile(Phi(r)), before numerical filters.
    ///
    /// Try the outward 32-node binary64 filter once, then exact/refinement routes in
    /// condition order. A refutation returns ParseError immediately; otherwise any
    /// unresolved condition yields UnknownCertificate after both conditions are tried.
    /// This is not a stand-alone validator of arbitrary native graphs or a total
    /// resource bound. The parent walk must supply operand-domain admission first.
    fn prove_imported_operation_domain(&self) -> Result<(), crate::Problem> {
        use ImportedDomainCondition::{Above, Below, NonZero, TrigNonZero};
        let one = Rational::one;
        let small = || Rational::fraction(3, 4).expect("nonzero constant denominator");
        let (operand, conditions) = match &self.internal.approximation {
            Approximation::Inverse(x) => (x, [Some(NonZero), None]),
            // The quotient kernels need a nonzero denominator; tan(x) and
            // cot(x) have poles where cos(x) or sin(x) vanishes.
            Approximation::PrescaledTan(x) => (x, [Some(TrigNonZero(true)), None]),
            Approximation::PrescaledCot(x) => (x, [Some(TrigNonZero(false)), None]),
            // x^2 >= 0 for every defined x. Real::abs builds sqrt(x^2) when
            // the sign of x is unresolved, possibly because x is zero.
            Approximation::Sqrt(x) | Approximation::NthRoot(x, _)
                if matches!(x.internal.approximation, Approximation::Square(_)) =>
            {
                return Ok(());
            }
            Approximation::Sqrt(x) | Approximation::NthRoot(x, _) => {
                (x, [Some(Above(Rational::zero(), true)), None])
            }
            // Stores x for ln(1+x).
            Approximation::PrescaledLn(x) => (x, [Some(Above(-one(), false)), None]),
            Approximation::PrescaledAsin(x) | Approximation::AsinDeferred(x) => {
                (x, [Some(Above(-one(), true)), Some(Below(one(), true))])
            }
            Approximation::AcosPositive(x) => (
                x,
                [
                    Some(Above(Rational::zero(), true)),
                    Some(Below(one(), true)),
                ],
            ),
            Approximation::AcoshNearOne(x) | Approximation::AcoshDirect(x) => {
                (x, [Some(Above(one(), true)), None])
            }
            Approximation::PrescaledAtanh(x) | Approximation::AtanhDirect(x) => {
                (x, [Some(Above(-one(), false)), Some(Below(one(), false))])
            }
            Approximation::NormalQuantile(data) => (
                &data.p,
                [
                    Some(Above(Rational::zero(), false)),
                    Some(Below(one(), false)),
                ],
            ),
            // The removable small-angle kernels evaluate their analytic
            // continuations only on the constructor-certified |x| <= 3/4.
            Approximation::SincSmall(x) | Approximation::CoscSmall(x) => {
                (x, [Some(Above(-small(), true)), Some(Below(small(), true))])
            }
            _ => return Ok(()),
        };
        // A defined exact identity of the node itself proves its domain without
        // refinement: quantile(Phi(r)) = r needs 0 < Phi(r) < 1 for rational r,
        // which refinement cannot separate from 1 when r is large.
        if self.exact_rational().is_some() {
            return Ok(());
        }
        // The existing outward-rounded 32-node filter decides most separated
        // operands without refinement or symbolic expansion.
        let interval = operand.binary64_filter_interval(&mut 32);
        let mut undecided = false;
        for condition in conditions.into_iter().flatten() {
            let decided = interval
                .as_ref()
                .and_then(|interval| interval.decide_imported_domain_condition(&condition));
            match decided.or_else(|| operand.imported_domain_condition(condition)) {
                Some(true) => {}
                Some(false) => return Err(crate::Problem::ParseError),
                None => undecided = true,
            }
        }
        if undecided {
            Err(crate::Problem::UnknownCertificate)
        } else {
            Ok(())
        }
    }

    // Native rational cotangents may retain sin(q + a*pi) as the
    // denominator after range reduction. For rational q != 0 and rational a,
    // neither sine nor cosine vanishes: otherwise q would be a nonzero
    // rational multiple of pi. Replay that proof from the graph, only after
    // ordinary nonzero admission declines. No sign or wire flag is trusted.
    // The post-order import walk has already proved every child domain.
    /// Recognize nonzero sine/cosine of a nonzero rational plus a rational pi multiple.
    /// Pi's transcendence excludes zeros for q+a*pi with rational q!=0 and a.
    /// Inspect only retained leaves, Add/Multiply and sign/binary-offset wrappers;
    /// do not expand potentially huge rational shifts. Rational coefficients of the
    /// pi term may be zero. Negation and finite binary scaling preserve nonzeroness.
    ///
    /// After at most 48 outer wrapper visits, use one shared 48-visit argument budget,
    /// including failed alternatives. Stop flags decline recognition. False means
    /// unsupported shape, depleted recognition allowance or cancellation, not a zero
    /// proof. Child domains must already be proved by the import walk.
    fn imported_affine_pi_trig_nonzero(&self) -> bool {
        // This proof needs only the stored rational/linear-pi shape. Do not
        // expand coefficients: even a single binary shift can denote a very
        // large rational. Sign and finite binary shifts preserve nonzeroness.
        /// Peel sign and binary-offset wrappers while charging each inspected node.
        /// Decrement the shared visit allowance before polling this wrapper's signal.
        /// None denotes depletion or a stop request; no coefficient expansion occurs.
        fn unwrapped<'a>(
            mut value: &'a Computable,
            remaining: &mut usize,
        ) -> Option<&'a Approximation> {
            loop {
                *remaining = remaining.checked_sub(1)?;
                if should_stop(&value.signal) {
                    return None;
                }
                match &value.internal.approximation {
                    Approximation::Negate(child) | Approximation::Offset(child, _) => {
                        value = child;
                    }
                    approximation => return Some(approximation),
                }
            }
        }

        /// Recognize a rational leaf beneath sign/offset wrappers and report its nonzeroness.
        /// Some(false) is a zero rational; None is an unrecognized or interrupted shape.
        /// Reuse the shared visit allowance and avoid materializing binary-scaled integers.
        fn rational_leaf(value: &Computable, remaining: &mut usize) -> Option<bool> {
            match unwrapped(value, remaining)? {
                Approximation::One => Some(true),
                Approximation::Int(value) => Some(!value.is_zero()),
                Approximation::Ratio(value) => Some(!value.is_zero()),
                _ => None,
            }
        }

        /// Recognize a rational multiple of pi or tau through wrapped products.
        /// Either multiplication order is tried using the same remaining allowance.
        /// A zero rational coefficient is permitted: only rational-multiple shape is
        /// needed here, not nonzeroness of this term. False is failed recognition.
        fn pi_multiple(value: &Computable, remaining: &mut usize) -> bool {
            match unwrapped(value, remaining) {
                Some(Approximation::Constant(SharedConstant::Pi | SharedConstant::Tau)) => true,
                Some(Approximation::Multiply(left, right)) => {
                    (rational_leaf(left, remaining).is_some() && pi_multiple(right, remaining))
                        || (rational_leaf(right, remaining).is_some()
                            && pi_multiple(left, remaining))
                }
                _ => false,
            }
        }

        let mut image = self;
        for _ in 0..48 {
            if should_stop(&image.signal) {
                return false;
            }
            match &image.internal.approximation {
                Approximation::Negate(child) | Approximation::Offset(child, _) => {
                    image = child;
                }
                Approximation::PrescaledSin(argument) | Approximation::PrescaledCos(argument) => {
                    let remaining = &mut 48;
                    return match unwrapped(argument, remaining) {
                        Some(Approximation::One) => true,
                        Some(Approximation::Int(value)) => !value.is_zero(),
                        Some(Approximation::Ratio(value)) => !value.is_zero(),
                        Some(Approximation::Add(left, right)) => {
                            (rational_leaf(left, remaining) == Some(true)
                                && pi_multiple(right, remaining))
                                || (rational_leaf(right, remaining) == Some(true)
                                    && pi_multiple(left, remaining))
                        }
                        _ => false,
                    };
                }
                _ => return false,
            }
        }
        false
    }
    /// Prove/refute a condition through exact structure or bounded-precision sign queries.
    /// Use Real's partial-comparison floor. NonZero tries ordinary sign first, then
    /// retained affine-pi trig nonzeroness. Trig conditions try the small-angle sign,
    /// pi-Laurent algebraic/rational-pi structure and finally the sine/cosine image.
    /// Endpoint conditions compare a recognized rational directly or query the signed
    /// difference. Some(false) refutes the condition; None is unproved, not invalid.
    ///
    /// All operand domains must already be admitted. Local node limits and the sign
    /// precision floor do not bound total work, and no separate cancellation receipt
    /// is returned by this Option interface.
    fn imported_domain_condition(&self, condition: ImportedDomainCondition) -> Option<bool> {
        // Native Real partial operations certify operand signs at this floor.
        let floor = crate::Real::PARTIAL_CMP_MIN_PRECISION;
        let (endpoint, inclusive, above) = match condition {
            ImportedDomainCondition::NonZero => {
                return self
                    .sign_until(floor)
                    .map(|sign| sign != RealSign::Zero)
                    .or_else(|| self.imported_affine_pi_trig_nonzero().then_some(true));
            }
            ImportedDomainCondition::TrigNonZero(cosine) => {
                // Keep tiny shifted arguments symbolic. On [-1,1], cosine
                // is positive and sine vanishes exactly at zero. This also
                // avoids expanding a potentially enormous rational scale in
                // the general Laurent proof below.
                if let Some(sign) = self.small_trig_argument_sign() {
                    return Some(cosine || sign != Sign::NoSign);
                }
                // Bounded refinement cannot prove a transcendental zero, so
                // read the exact pi-Laurent normal form first. Since pi is
                // transcendental, an algebraic x has cos(x) != 0 and has
                // sin(x) = 0 only at x = 0. For x = q*pi, sin(x) = 0 iff q is
                // an integer and cos(x) = 0 iff q + 1/2 is, as in Real::tan.
                if let Some(polynomial) = self.pi_laurent_polynomial(48) {
                    let terms = polynomial.terms.as_slice();
                    if terms
                        .iter()
                        .all(|(power, coefficient)| *power == 0 && coefficient.atoms.is_empty())
                    {
                        return if cosine {
                            Some(true)
                        } else {
                            self.sign_until(floor).map(|sign| sign != RealSign::Zero)
                        };
                    }
                    if let [(1, coefficient)] = terms
                        && coefficient.radicals.is_empty()
                        && coefficient.atoms.is_empty()
                    {
                        let half = Rational::fraction(1, 2).expect("nonzero constant denominator");
                        let shifted = if cosine {
                            coefficient.rational.clone() + half
                        } else {
                            coefficient.rational.clone()
                        };
                        return Some(!shifted.is_integer());
                    }
                }
                let image = if cosine {
                    self.clone().cos()
                } else {
                    self.clone().sin()
                };
                return image.sign_until(floor).map(|sign| sign != RealSign::Zero);
            }
            ImportedDomainCondition::Above(endpoint, inclusive) => (endpoint, inclusive, true),
            ImportedDomainCondition::Below(endpoint, inclusive) => (endpoint, inclusive, false),
        };
        let difference = match self.exact_rational() {
            Some(value) => match value.partial_cmp(&endpoint)? {
                Ordering::Less => RealSign::Negative,
                Ordering::Equal => RealSign::Zero,
                Ordering::Greater => RealSign::Positive,
            },
            None if endpoint.is_zero() => self.sign_until(floor)?,
            None => self
                .clone()
                .add(Self::rational(-endpoint))
                .sign_until(floor)?,
        };
        Some(match (difference, above) {
            (RealSign::Zero, _) => inclusive,
            (RealSign::Positive, true) | (RealSign::Negative, false) => true,
            (RealSign::Positive, false) | (RealSign::Negative, true) => false,
        })
    }
}

#[cfg(not(verus_keep_ghost))]
/// Construct immutable nodes and access their shared accelerators.
impl Node {
    /// Install an immutable recipe with caller-supplied bound/sign facts and empty evaluation cache.
    /// Derive only construction hints: Add takes the larger child demand plus two,
    /// Negate forwards it and Offset adds a clamped signed shift. Other recipes use
    /// zero demand and the selective inverse-angle/pi presence recognizer. Facts
    /// are trusted inputs, not validated proofs; construction is not domain admission.
    pub(crate) fn new(
        approximation: Approximation,
        bound: BoundCache,
        exact_sign: ExactSignCache,
    ) -> Self {
        let (contains_inverse_trig_or_pi, linear_demand) = match &approximation {
            Approximation::Add(left, right) => {
                let (left_inverse, left_demand) = left.internal.construction_hints();
                let (right_inverse, right_demand) = right.internal.construction_hints();
                (
                    left_inverse || right_inverse,
                    left_demand.max(right_demand).saturating_add(2),
                )
            }
            Approximation::Negate(child) => child.internal.construction_hints(),
            Approximation::Offset(child, shift) => {
                let (inverse, demand) = child.internal.construction_hints();
                let demand = i32::from(demand)
                    .saturating_add(*shift)
                    .clamp(i32::from(i16::MIN), i32::from(i16::MAX));
                (inverse, demand as i16)
            }
            _ => (
                Self::approximation_contains_inverse_trig_or_pi(&approximation),
                0,
            ),
        };
        Self {
            facts: AtomicFacts::new(
                bound,
                exact_sign,
                contains_inverse_trig_or_pi,
                linear_demand,
            ),
            approximation,
            cache: EvaluationCache::new(),
        }
    }

    /// Read presence and signed linear-demand hints from one relaxed atomic snapshot.
    /// If presence is unknown, compute/cache that hint separately; the retained
    /// demand still comes from the original snapshot. Neither value is a magnitude
    /// or domain certificate, and deserialized demand defaults to zero.
    fn construction_hints(&self) -> (bool, i16) {
        // Read both retained hints in the same atomic snapshot. The inverse
        // trig flag already needs this load during Add construction.
        let bits = self.facts.0.load(std::sync::atomic::Ordering::Relaxed);
        let contains_inverse_trig_or_pi = if bits & AtomicFacts::INVERSE_TRIG_OR_PI_KNOWN != 0 {
            bits & AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI != 0
        } else {
            self.contains_inverse_trig_or_pi()
        };
        let demand = ((bits & AtomicFacts::LINEAR_DEMAND_MASK) >> AtomicFacts::LINEAR_DEMAND_SHIFT)
            as u16 as i16;
        (contains_inverse_trig_or_pi, demand)
    }

    /// Recognize the selected inverse-angle/pi syntax used to schedule symbolic rewrites.
    /// Mark the listed anchors/inverse recipes and propagate through selected unary
    /// and binary wrappers. Unhandled tags, including Tau and some inverse-function
    /// forms, return false; this is not an exhaustive dependency or value test.
    /// Child queries may fill their own presence caches.
    fn approximation_contains_inverse_trig_or_pi(approximation: &Approximation) -> bool {
        match approximation {
            Approximation::Constant(SharedConstant::Pi)
            | Approximation::AtanRational(_)
            | Approximation::IntegralAtan(_)
            | Approximation::PrescaledAtan(_)
            | Approximation::AtanDeferred(_)
            | Approximation::InverseIsometricLatitude { .. }
            | Approximation::JacobiAmplitude(_)
            | Approximation::InverseEllipticE(_)
            | Approximation::AcosPositive(_)
            | Approximation::AcosPositiveRational(_)
            | Approximation::AcosNegativeRational(_) => true,
            Approximation::Negate(child)
            | Approximation::Offset(child, _)
            | Approximation::SincSmall(child)
            | Approximation::CoscSmall(child) => child.internal.contains_inverse_trig_or_pi(),
            Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
                left.internal.contains_inverse_trig_or_pi()
                    || right.internal.contains_inverse_trig_or_pi()
            }
            _ => false,
        }
    }

    /// Return or lazily compute the selective symbolic-rewrite presence hint.
    /// The immutable recipe determines the hint; unknown is distinct from cached
    /// false. This is scheduling metadata, not proof of numerical dependence on pi.
    pub(crate) fn contains_inverse_trig_or_pi(&self) -> bool {
        if let Some(value) = self.facts.contains_inverse_trig_or_pi() {
            return value;
        }
        let value = Self::approximation_contains_inverse_trig_or_pi(&self.approximation);
        self.facts.store_contains_inverse_trig_or_pi(value);
        value
    }

    /// Clone a retained sample at this or a coarser requested binary precision.
    /// Return None if no sample is present or finer work is needed. Delegate scaling
    /// to EvaluationCache without computing the expression or checking cancellation.
    pub(crate) fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        self.cache.at_precision(p)
    }

    /// Clone the finest retained sample and its binary precision, if present.
    /// This raw accessor reads only this node's cache, not a named constant's separate
    /// process-wide cache, and does not establish completion independently.
    pub(crate) fn cached_value(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }

    /// Offer a caller-certified completed sample to this node's monotone precision slot.
    /// The storage layer does not check its value, error bound or cancellation.
    /// Ordinary evaluation must use the outer signal-aware publication path before
    /// calling this helper; equal/coarser samples do not replace the retained one.
    pub(crate) fn store_cache_value(&self, p: Precision, value: BigInt) {
        self.cache.store(p, value);
    }

    #[cfg(test)]
    /// Expose a cloned node-local sample/precision pair to tests.
    /// Observe retained state without refinement or a shared-constant cache lookup.
    pub(crate) fn cache_snapshot(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }
}

#[cfg(not(verus_keep_ghost))]
/// A completed integer sample in binary precision units, retained for later coarser requests.
/// Storage trusts the publishing caller's error and cancellation checks.
struct CachedApproximation {
    /// Binary scale p of the retained integer approximation, with finer work at smaller p.
    precision: Precision,
    /// Owned sample integer representing value*2^(-precision) within its caller-certified error.
    value: BigInt,
}

#[derive(Default)]
#[cfg(not(verus_keep_ghost))]
/// Node-local optional approximation, exact tower and memoized radical-recognition decline.
/// Default leaves both owned values absent and the decline false. No policy or
/// caller-budget key is stored, and one entry does not bound retained bytes.
struct CachedEvaluation {
    /// Finest published numeric sample, absent until a completed result is offered.
    approximation: Option<CachedApproximation>,
    /// First retained exact quadratic normal form, cloned without keeping source graphs alive.
    quadratic_tower: Option<Box<Tower>>,
    /// Memoized failed radical sign recognition without a cause, budget key or retry override.
    radical_tower_declined: bool,
}
#[cfg(not(verus_keep_ghost))]
/// Lazily allocated synchronized storage tied to one immutable expression node.
/// A null atomic pointer avoids an unused lock allocation. The installed cell
/// lives until exclusive Drop; readers clone under the lock, and no guard escapes
/// into evaluation. Towers contain rational coefficients rather than expression
/// back-references. There is no eviction, byte cap or numerical validation here;
/// poisoned locks are recovered without re-establishing mathematical evidence.
struct EvaluationCache(
    /// Null or the uniquely installed owned lock; acquire/release publication protects initialization.
    std::sync::atomic::AtomicPtr<std::sync::RwLock<CachedEvaluation>>,
);

#[cfg(not(verus_keep_ghost))]
/// Own lazy cache publication, locked snapshots and first-success normal-form reuse.
impl EvaluationCache {
    /// Create a null cache pointer without allocating a lock or evaluation payload.
    fn new() -> Self {
        Self(std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()))
    }

    /// Acquire the installed lock pointer, if any, with a borrow tied to this cache.
    /// The pointer is installed once and never removed before exclusive Drop, so
    /// readers can safely borrow the cell without a reclamation protocol.
    fn cell(&self) -> Option<&std::sync::RwLock<CachedEvaluation>> {
        let pointer = self.0.load(std::sync::atomic::Ordering::Acquire);
        if pointer.is_null() {
            None
        } else {
            // SAFETY: the pointed-to cell is installed once and remains alive
            // until the enclosing Node is exclusively dropped.
            Some(unsafe { &*pointer })
        }
    }

    /// Return the cache lock, allocating and atomically publishing it on first use.
    /// A compare-exchange winner retains its allocation until Drop; a loser frees
    /// only its unpublished allocation and uses the winner. Acquire/release ordering
    /// publishes initialized storage. Allocation is infallible at the API boundary
    /// and has no explicit byte cap or cancellation check.
    fn cell_or_init(&self) -> &std::sync::RwLock<CachedEvaluation> {
        if let Some(cell) = self.cell() {
            return cell;
        }

        let allocated = Box::into_raw(Box::new(
            std::sync::RwLock::new(CachedEvaluation::default()),
        ));
        let pointer = match self.0.compare_exchange(
            std::ptr::null_mut(),
            allocated,
            std::sync::atomic::Ordering::AcqRel,
            std::sync::atomic::Ordering::Acquire,
        ) {
            Ok(_) => allocated,
            Err(installed) => {
                // SAFETY: this allocation lost the installation race and was
                // never published.
                unsafe { drop(Box::from_raw(allocated)) };
                installed
            }
        };
        // SAFETY: the winning pointer remains installed until Node::drop.
        unsafe { &*pointer }
    }

    /// Clone the retained sample under a read lock, or return None before initialization.
    /// Recover poisoned locks with their contained data; no mathematical revalidation
    /// occurs here. No lock guard escapes to expression evaluation.
    fn get(&self) -> Option<(Precision, BigInt)> {
        let guard = self
            .cell()?
            .read()
            .unwrap_or_else(|error| error.into_inner());
        let cached = guard.approximation.as_ref()?;
        Some((cached.precision, cached.value.clone()))
    }

    /// Read and rescale an available sample while holding the read lock.
    /// Finer-than-cached requests return None. Recover poisoned lock contents and
    /// clone the result; do not refine the source or poll cancellation.
    fn at_precision(&self, p: Precision) -> Option<BigInt> {
        let guard = self
            .cell()?
            .read()
            .unwrap_or_else(|error| error.into_inner());
        Self::value_at_precision(guard.approximation.as_ref()?, p)
    }

    #[inline(always)]
    /// Reuse a sample at equal precision or round it to a coarser binary grid.
    /// A finer request returns None. Use unsigned abs_diff so the precision gap can
    /// span the full i32 range. A gap above the integer's magnitude bits returns
    /// zero without a huge shift; otherwise round nearest with halves toward positive
    /// infinity. This rescales the cached approximation, not the exact represented
    /// real, and assumes the original sample's error contract was established.
    fn value_at_precision(cached: &CachedApproximation, p: Precision) -> Option<BigInt> {
        if p < cached.precision {
            None
        } else if p == cached.precision {
            Some(cached.value.clone())
        } else {
            // The positive gap can exceed i32::MAX even for valid precisions.
            let gap = p.abs_diff(cached.precision);
            let value = if gap == 1 {
                // Preserve scale's direct-clone path for a zero initial shift.
                cached.value.clone()
            } else if u64::from(gap) > cached.value.bits() {
                return Some(BigInt::zero());
            } else {
                &cached.value >> (gap - 1)
            };
            // Nearest integer, with halfway values rounded toward +infinity.
            Some((value + signed::ONE.deref()) >> 1)
        }
    }

    /// Retain the first sample or a strictly finer one under the write lock.
    /// Equal precision keeps the previously published integer, including when valid
    /// roundings differ. Coarser late-finishing workers cannot evict finer work.
    /// The caller certifies completion/error; this method does not validate samples,
    /// poll stops or cap integer storage, and it recovers poisoned lock contents.
    fn store(&self, p: Precision, value: BigInt) {
        let mut guard = self
            .cell_or_init()
            .write()
            .unwrap_or_else(|error| error.into_inner());
        // Concurrent evaluations may finish out of order. Never let a coarser
        // result evict a finer result already published.
        if guard
            .approximation
            .as_ref()
            .is_none_or(|cached| p < cached.precision)
        {
            guard.approximation = Some(CachedApproximation {
                precision: p,
                value,
            });
        }
    }

    /// Clone the retained exact quadratic tower under the read lock, if any.
    /// No graph is rebuilt; poisoned lock contents are used without revalidation.
    fn quadratic_tower(&self) -> Option<Tower> {
        let guard = self
            .cell()?
            .read()
            .unwrap_or_else(|error| error.into_inner());
        guard.quadratic_tower.as_deref().cloned()
    }

    /// Read whether the iterated radical sign recognizer previously declined this node.
    /// An unallocated cache means false. The flag records no cause, policy or caller
    /// budget and is not evidence of mathematical nonmembership or undecidability.
    fn radical_tower_declined(&self) -> bool {
        self.cell().is_some_and(|cell| {
            cell.read()
                .unwrap_or_else(|error| error.into_inner())
                .radical_tower_declined
        })
    }

    /// Permanently record a radical-recognition decline for this live node.
    /// Allocate the cache if needed and set the flag under its lock. No reason or
    /// retry key is retained, and this helper provides no invalidation/reset API.
    fn mark_radical_tower_declined(&self) {
        self.cell_or_init()
            .write()
            .unwrap_or_else(|error| error.into_inner())
            .radical_tower_declined = true;
    }

    /// Clone the first successful exact tower into this node's cache.
    /// Later stores keep the first tower without comparing equivalence. The caller
    /// must establish that it represents this immutable node; no coefficients or
    /// resource costs are bounded here. Poisoned lock contents remain usable.
    fn store_quadratic_tower(&self, tower: &Tower) {
        let mut guard = self
            .cell_or_init()
            .write()
            .unwrap_or_else(|error| error.into_inner());
        if guard.quadratic_tower.is_none() {
            guard.quadratic_tower = Some(Box::new(tower.clone()));
        }
    }
}

#[cfg(not(verus_keep_ghost))]
/// Construct an unallocated cache for ordinary defaults and replay.
impl Default for EvaluationCache {
    /// Create the same allocation-free empty cache as new, including on native replay.
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(not(verus_keep_ghost))]
/// Expose only a diagnostic sample snapshot through Debug.
impl std::fmt::Debug for EvaluationCache {
    /// Format only the optional cloned approximation snapshot for diagnostics.
    /// Tower/decline metadata is omitted. This can take the read lock and clone a
    /// big integer; it is not serialization or a complete cache-state report.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.get().fmt(formatter)
    }
}

#[cfg(not(verus_keep_ghost))]
/// Release the single installed cache allocation at the end of its owner's lifetime.
impl Drop for EvaluationCache {
    /// Free the installed lock and its owned samples/tower once this cache is exclusive.
    /// A null cache owns nothing. The owning node's lifetime and borrowed guards must
    /// prevent concurrent access during Drop; the pointer is never removed earlier.
    fn drop(&mut self) {
        let pointer = *self.0.get_mut();
        if !pointer.is_null() {
            // SAFETY: dropping the enclosing Node proves exclusive access and
            // no guard can still borrow this cell.
            unsafe { drop(Box::from_raw(pointer)) };
        }
    }
}

#[cfg(not(verus_keep_ghost))]
/// Expose the immutable approximation recipe through ordinary pattern matching.
impl Deref for Node {
    /// The immutable recipe exposed by Node dereferencing.
    type Target = Approximation;

    /// Borrow the immutable approximation recipe without exposing mutable cache state.
    fn deref(&self) -> &Self::Target {
        &self.approximation
    }
}

#[derive(Debug)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
/// One atomic word for bound, independent exact sign and symbolic scheduling hints.
/// Relaxed accesses update self-contained lanes; they do not publish external
/// proof objects. Callers supply valid facts and immutable recipe identity.
/// The word is omitted from native interchange. Debug exposes packed storage,
/// not a decoded mathematical certificate.
struct AtomicFacts(
    /// Packed bit lanes accessed atomically; reserved states are internal invariants, not wire flags.
    std::sync::atomic::AtomicU64,
);

#[cfg(not(verus_keep_ghost))]
/// Initialize empty facts for replay without deriving evidence from flags.
impl Default for AtomicFacts {
    /// Initialize all bits to zero: invalid bound/sign, unknown presence and zero demand.
    /// Replay uses this state instead of trusting serialized accelerator facts.
    fn default() -> Self {
        Self(std::sync::atomic::AtomicU64::new(0))
    }
}

/// Encode and atomically update self-contained fact/hint lanes.
impl AtomicFacts {
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bound tag for an uncomputed fact (low two bits zero).
    const TAG_INVALID: u64 = 0;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bound tag for a completed but inconclusive cheap query.
    const TAG_UNKNOWN: u64 = 1;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bound tag for a proved exact zero.
    const TAG_ZERO: u64 = 2;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bound tag carrying optional sign/binade metadata for a nonzero fact.
    const TAG_NONZERO: u64 = 3;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Start of the bound-local two-bit optional sign encoding.
    const SIGN_SHIFT: u32 = 2;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bit indicating the upper word contains a signed binade exponent.
    const MSD_PRESENT: u64 = 1 << 4;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bit distinguishing a certified binade from a planning estimate.
    const EXACT_MSD: u64 = 1 << 5;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Start of the independent three-bit exact-sign cache lane.
    const EXACT_SIGN_SHIFT: u32 = 6;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Mask covering the independent exact-sign lane only.
    const EXACT_SIGN_MASK: u64 = 0b111 << Self::EXACT_SIGN_SHIFT;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Bit distinguishing an uncomputed presence hint from either computed result.
    const INVERSE_TRIG_OR_PI_KNOWN: u64 = 1 << 9;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// True value of the selective inverse-angle/pi scheduling hint.
    const CONTAINS_INVERSE_TRIG_OR_PI: u64 = 1 << 10;
    // Signed saturated precision decrement along Add/Negate/Offset paths.
    // Used only for child ordering, never as a numerical bound. Deserialized
    // nodes default to zero and retain ordinary evaluation order.
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Start of the signed saturated i16 linear precision-demand hint; replay defaults to zero.
    const LINEAR_DEMAND_SHIFT: u32 = 16;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Mask selecting the linear child-ordering hint, not a numerical magnitude bound.
    const LINEAR_DEMAND_MASK: u64 = (u16::MAX as u64) << Self::LINEAR_DEMAND_SHIFT;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Independent sign/presence/demand lanes preserved by every bound replacement.
    const NON_BOUND_MASK: u64 = Self::EXACT_SIGN_MASK
        | Self::INVERSE_TRIG_OR_PI_KNOWN
        | Self::CONTAINS_INVERSE_TRIG_OR_PI
        | Self::LINEAR_DEMAND_MASK;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    /// Start of the upper 32-bit two's-complement binade payload.
    const MSD_SHIFT: u32 = 32;

    #[cfg(not(verus_keep_ghost))]
    /// Pack trusted bound/sign facts, a known presence hint and signed i16 demand.
    /// This is encoding, not proof validation; callers own consistency of all facts.
    /// The presence-known bit is set even for a false hint.
    fn new(
        bound: BoundCache,
        exact_sign: ExactSignCache,
        contains_inverse_trig_or_pi: bool,
        linear_demand: i16,
    ) -> Self {
        Self(std::sync::atomic::AtomicU64::new(
            Self::encode_bound(bound)
                | Self::encode_exact_sign(exact_sign)
                | ((linear_demand as u16 as u64) << Self::LINEAR_DEMAND_SHIFT)
                | Self::INVERSE_TRIG_OR_PI_KNOWN
                | if contains_inverse_trig_or_pi {
                    Self::CONTAINS_INVERSE_TRIG_OR_PI
                } else {
                    0
                },
        ))
    }

    #[cfg(not(verus_keep_ghost))]
    /// Read the two's-complement i16 scheduling demand from a relaxed atomic snapshot.
    /// The saturated hint orders linear child work; it is not a numerical bound.
    fn linear_demand(&self) -> i16 {
        ((self.0.load(std::sync::atomic::Ordering::Relaxed) & Self::LINEAR_DEMAND_MASK)
            >> Self::LINEAR_DEMAND_SHIFT) as u16 as i16
    }

    #[cfg(not(verus_keep_ghost))]
    /// Decode the optional presence hint, distinguishing uncomputed from false.
    /// Read with relaxed ordering because the word itself contains the entire fact.
    fn contains_inverse_trig_or_pi(&self) -> Option<bool> {
        let bits = self.0.load(std::sync::atomic::Ordering::Relaxed);
        (bits & Self::INVERSE_TRIG_OR_PI_KNOWN != 0)
            .then_some(bits & Self::CONTAINS_INVERSE_TRIG_OR_PI != 0)
    }

    #[cfg(not(verus_keep_ghost))]
    /// OR in a known presence result without disturbing other packed facts.
    /// False sets only knownness and cannot clear an already-true presence bit.
    /// The immutable graph requires independently computed results to agree.
    fn store_contains_inverse_trig_or_pi(&self, value: bool) {
        let bits = Self::INVERSE_TRIG_OR_PI_KNOWN
            | if value {
                Self::CONTAINS_INVERSE_TRIG_OR_PI
            } else {
                0
            };
        self.0.fetch_or(bits, std::sync::atomic::Ordering::Relaxed);
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == encoded_bound_bits(value),
    ))]
    /// Pack a bound tag, optional sign/binade and exact-binade flag into their lanes.
    /// Preserve signed i32 exponent bits through u32 conversion. Do not validate
    /// semantic combinations such as a NonZero bound carrying a zero sign; callers
    /// must supply consistent evidence. The ghost specification describes encoding.
    fn encode_bound(value: BoundCache) -> u64 {
        #[cfg(verus_keep_ghost)]
        proof! {
            assert(Self::MSD_PRESENT == 16u64) by (bit_vector);
            assert(Self::EXACT_MSD == 32u64) by (bit_vector);
        }
        match value {
            BoundCache::Invalid => Self::TAG_INVALID,
            BoundCache::Valid(BoundInfo::Unknown) => Self::TAG_UNKNOWN,
            BoundCache::Valid(BoundInfo::Zero) => Self::TAG_ZERO,
            BoundCache::Valid(BoundInfo::NonZero {
                sign,
                msd,
                exact_msd,
            }) => {
                let sign = match sign {
                    None => 0,
                    Some(Sign::Plus) => 1,
                    Some(Sign::Minus) => 2,
                    Some(Sign::NoSign) => 3,
                };
                let mut encoded = Self::TAG_NONZERO | ((sign as u64) << Self::SIGN_SHIFT);
                if let Some(msd) = msd {
                    encoded |= Self::MSD_PRESENT | ((msd as u32 as u64) << Self::MSD_SHIFT);
                }
                if exact_msd {
                    encoded |= Self::EXACT_MSD;
                }
                encoded
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    /// Decode only the bound lanes, ignoring independent sign and scheduling metadata.
    /// Every two-bit tag is handled, but decoded combinations are not independently
    /// validated as mathematical facts. Signed exponent bits are restored as i32.
    fn decode_bound(value: u64) -> BoundCache {
        match value & 0b11 {
            Self::TAG_INVALID => BoundCache::Invalid,
            Self::TAG_UNKNOWN => BoundCache::Valid(BoundInfo::Unknown),
            Self::TAG_ZERO => BoundCache::Valid(BoundInfo::Zero),
            Self::TAG_NONZERO => {
                let sign = match (value >> Self::SIGN_SHIFT) & 0b11 {
                    0 => None,
                    1 => Some(Sign::Plus),
                    2 => Some(Sign::Minus),
                    3 => Some(Sign::NoSign),
                    _ => unreachable!(),
                };
                let msd = (value & Self::MSD_PRESENT != 0)
                    .then_some(((value >> Self::MSD_SHIFT) as u32) as i32);
                BoundCache::Valid(BoundInfo::NonZero {
                    sign,
                    msd,
                    exact_msd: value & Self::EXACT_MSD != 0,
                })
            }
            _ => unreachable!(),
        }
    }

    #[cfg(not(verus_keep_ghost))]
    /// Read the bound lanes from one relaxed snapshot without deriving new evidence.
    fn bound(&self) -> BoundCache {
        Self::decode_bound(
            self.0.load(std::sync::atomic::Ordering::Relaxed) & !Self::EXACT_SIGN_MASK,
        )
    }

    #[cfg(not(verus_keep_ghost))]
    /// Decode bound and independent exact-sign state from the same relaxed atomic load.
    /// This gives a coherent word snapshot, not an additional consistency proof.
    fn snapshot(&self) -> (BoundCache, ExactSignCache) {
        let encoded = self.0.load(std::sync::atomic::Ordering::Relaxed);
        (
            Self::decode_bound(encoded & !Self::EXACT_SIGN_MASK),
            Self::decode_exact_sign(encoded),
        )
    }

    #[cfg(not(verus_keep_ghost))]
    /// Replace the bound lanes unconditionally while preserving non-bound metadata.
    /// Retry weak compare-exchange with relaxed ordering on races. This primitive
    /// can weaken a bound if misused; its caller owns proof consistency and monotonicity.
    fn set_bound(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    /// Install a bound only while the observed bound tag remains Invalid.
    /// Recheck after every compare-exchange failure and preserve sign/hint lanes.
    /// Any existing valid tag, including Unknown, prevents this write.
    fn set_bound_if_invalid(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            if current & 0b11 != Self::TAG_INVALID {
                return;
            }
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    /// Install a bound only while the observed tag is Invalid or valid Unknown.
    /// Keep existing zero/nonzero structural bounds, even when the offered sample
    /// would choose another exponent. Recheck races and preserve non-bound lanes;
    /// the offered bound itself still requires caller certification.
    fn set_bound_if_unresolved(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let tag = current & 0b11;
            if tag != Self::TAG_INVALID && tag != Self::TAG_UNKNOWN {
                return;
            }
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures exact_sign_denotation(result) == value,
            result & !0x1c0u64 == 0,
            ((result >> 6) & 7) <= 4,
    ))]
    /// Encode Invalid, Unknown and three definite signs in the independent three-bit lane.
    /// Codes zero through four occupy bits six through eight; no other bits are set.
    /// The ghost proof records this mapping, not the truth of a supplied sign.
    fn encode_exact_sign(value: ExactSignCache) -> u64 {
        let encoded = match value {
            ExactSignCache::Invalid => 0,
            ExactSignCache::Unknown => 1,
            ExactSignCache::Valid(Sign::Minus) => 2,
            ExactSignCache::Valid(Sign::NoSign) => 3,
            ExactSignCache::Valid(Sign::Plus) => 4,
        };
        #[cfg(verus_keep_ghost)]
        proof! {
            assert((((encoded as u64) << 6u32) >> 6u32) & 7u64 == encoded) by (bit_vector)
                requires encoded <= 4u64;
            assert(((encoded as u64) << 6u32) & !0x1c0u64 == 0u64) by (bit_vector)
                requires encoded <= 4u64;
        }
        encoded << Self::EXACT_SIGN_SHIFT
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires ((value >> 6) & 7) <= 4,
        ensures result == exact_sign_denotation(value),
    ))]
    /// Decode the independent sign lane, assuming its code is zero through four.
    /// Codes five through seven panic as invalid internal states. Other lanes do
    /// not affect the result; ghost preconditions explicitly restrict the code.
    fn decode_exact_sign(value: u64) -> ExactSignCache {
        #[cfg(verus_keep_ghost)]
        proof! {
            assert(Self::EXACT_SIGN_MASK == 0x1c0u64) by (bit_vector);
            assert((value & 0x1c0u64) >> 6u32 == (value >> 6u32) & 7u64) by (bit_vector);
        }
        match (value & Self::EXACT_SIGN_MASK) >> Self::EXACT_SIGN_SHIFT {
            0 => ExactSignCache::Invalid,
            1 => ExactSignCache::Unknown,
            2 => ExactSignCache::Valid(Sign::Minus),
            3 => ExactSignCache::Valid(Sign::NoSign),
            4 => ExactSignCache::Valid(Sign::Plus),
            _ => unreachable!("invalid atomic exact-sign cache state"),
        }
    }

    #[cfg(not(verus_keep_ghost))]
    /// Read the independent sign cache from a relaxed snapshot without refinement.
    fn exact_sign(&self) -> ExactSignCache {
        Self::decode_exact_sign(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }
    #[cfg(not(verus_keep_ghost))]
    /// Publish a sign state while retaining any already-certified sign and return the old state.
    /// Recheck after every failed compare-exchange. A contradictory definite sign
    /// triggers a debug assertion; release builds retain the existing certificate.
    /// Unknown/Invalid may replace each other, but neither erases a definite sign.
    /// Other lanes are preserved. Correct callers must prove signs agree for this
    /// immutable node; atomicity alone does not certify their mathematical truth.
    fn replace_exact_sign(&self, value: ExactSignCache) -> ExactSignCache {
        let encoded = Self::encode_exact_sign(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let previous = Self::decode_exact_sign(current);
            if let ExactSignCache::Valid(sign) = previous {
                debug_assert!(
                    !matches!(value, ExactSignCache::Valid(other) if other != sign),
                    "inconsistent exact signs for an immutable expression",
                );
                return previous;
            }
            let updated = (current & !Self::EXACT_SIGN_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return previous,
                Err(observed) => current = observed,
            }
        }
    }
}

#[cfg(not(verus_keep_ghost))]
/// Shared exact signed small integers used by approximation recurrences and binary scaling.
pub(crate) mod signed {
    use num::{BigInt, One};
    use std::sync::LazyLock;

    // Use the narrow primitive that holds each literal so `BigInt::from`
    // dispatches directly instead of routing through the `ToBigInt` helper.
    /// Shared exact integer minus one; initialized once for repeated kernel use.
    pub(crate) static MINUS_ONE: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(-1));
    /// Shared exact integer one; initialized once for repeated kernel use.
    pub(crate) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    /// Shared exact integer two; initialized once for repeated kernel use.
    pub(crate) static TWO: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(2_u8));
    /// Shared exact integer four; initialized once for repeated kernel use.
    pub(crate) static FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(4_u8));
    /// Shared exact integer six; initialized once for repeated kernel use.
    pub(crate) static SIX: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(6_u8));
    /// Shared exact integer eight; initialized once for repeated kernel use.
    pub(crate) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(8_u8));
    /// Shared exact integer sixteen; initialized once for repeated kernel use.
    pub(crate) static SIXTEEN: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(16_u8));
    /// Shared exact integer twenty-four; initialized once for repeated kernel use.
    pub(crate) static TWENTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(24_u8));
    /// Shared exact integer sixty-four; initialized once for repeated kernel use.
    pub(crate) static SIXTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(64_u8));
}

#[cfg(not(verus_keep_ghost))]
/// Shared exact nonnegative small integers for magnitude arithmetic and decimal extraction.
pub(crate) mod unsigned {
    use num::{BigUint, One};
    use std::sync::LazyLock;

    // These are small non-negative constants, so `u8` is the exact source type
    // and avoids the extra conversion trait path used before the bigint audit.
    /// Shared exact integer one; initialized once for repeated kernel use.
    pub(crate) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    /// Shared exact integer two; initialized once for repeated kernel use.
    pub(crate) static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
    /// Shared exact integer ten; initialized once for repeated kernel use.
    pub(crate) static TEN: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(10_u8));
    /// Shared exact integer five; initialized once for repeated kernel use.
    pub(crate) static FIVE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(5_u8));
    /// Shared exact integer six; initialized once for repeated kernel use.
    pub(crate) static SIX: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(6_u8));
}

#[cfg(not(verus_keep_ghost))]
/// Exact 3/2 exclusive upper bound for the rational half-pi complement shortcut.
static HALF_PI_SHORTCUT_RATIONAL_LIMIT: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
/// Exact 7/2 magnitude threshold for the near-large rational tangent schedule.
static NEAR_LARGE_RATIONAL_TRIG_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
/// Exact 7/8 threshold selecting the rational inverse-sine endpoint dispatch label.
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());
#[cfg(not(verus_keep_ghost))]
/// Exact 3/2 shared by logarithm residual-range construction.
static THREE_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
/// Exact 1/2 reused by inverse-function reductions; not a cached Computable node.
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
