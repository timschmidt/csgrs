//! Exact represented algebraic roots for solver residuals.
//!
//! This module is intentionally a representation layer, not a floating-point
//! approximation layer and not a complete algebraic-number field package. A
//! represented root carries the exact univariate polynomial row and the
//! certified isolating interval produced by [`crate::root_isolation`]. This is
//! a small exactness boundary: construction keeps exact object evidence, while
//! later predicates or candidate replay decide how that evidence may be
//! consumed.

use std::cmp::Ordering;
use std::collections::HashMap;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::{Rational, Real};

use crate::analysis::ProblemAnalysis;
use crate::interval::rational_interval_product;
use crate::model::{ConstraintKind, Problem};
use crate::root_isolation::{
    ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS, ALGEBRAIC_IMAGE_REFINEMENT_STEPS, IsolatedRootInterval,
    IsolatedRootRefinementReport, IsolatedRootRefinementStatus, RootIsolationConfig,
    RootIsolationStatus, UnivariateRootIsolationReport,
    isolate_univariate_polynomial_roots_with_config, polynomials_share_one_root_in_interval,
    refine_isolated_univariate_polynomial_interval,
};
use crate::symbolic::{Expr, SymbolId};
use crate::{
    AlgebraicRootBinaryTransformStatus, AlgebraicRootMobiusTransformStatus,
    AlgebraicRootPolynomialImageStatus, transform_algebraic_root_mobius,
    transform_algebraic_root_polynomial_image, transform_algebraic_roots_binary,
};

/// Validation status for represented algebraic-root evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootValidationStatus {
    /// The representation is structurally valid.
    Valid,
    /// The coefficient vector is empty, constant, or has no certifiably
    /// nonzero leading coefficient.
    InvalidPolynomial,
    /// The interval endpoints are ordered incorrectly.
    InvalidInterval,
    /// The interval does not claim exactly one distinct root.
    NonUnitIsolation,
    /// The exact point witness is outside the interval.
    WitnessOutsideInterval,
    /// The exact point witness does not satisfy the polynomial.
    WitnessDoesNotSatisfyPolynomial,
    /// Exact comparisons did not decide.
    Undecided,
}

/// Status for comparing two represented algebraic roots.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootComparisonStatus {
    /// Both inputs were valid and the ordering was certified.
    Compared,
    /// Both inputs name the same represented root evidence.
    SameRepresentation,
    /// One or both inputs failed structural validation.
    InvalidEvidence,
    /// The isolating intervals overlap, so this narrow comparison could not
    /// decide without further refinement or algebraic-number arithmetic.
    OverlappingIntervals,
    /// Exact endpoint comparison did not decide under the supplied policy.
    Undecided,
}

/// Certified or conservative comparison report for represented algebraic roots.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootComparisonReport {
    /// Final comparison status.
    pub status: AlgebraicRootComparisonStatus,
    /// Certified ordering when available. `None` means the comparison is
    /// intentionally unresolved, not guessed from approximations.
    pub ordering: Option<Ordering>,
    /// Compact diagnostic reason for invalid or unresolved comparisons.
    pub message: Option<String>,
}

/// Configuration for overlap-aware algebraic-root comparison.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRefinementComparisonConfig {
    /// Exact comparison/refinement policy used by `hyperlimit`.
    pub policy: PredicatePolicy,
    /// Maximum alternating refinement rounds when isolating intervals overlap.
    pub max_refinement_rounds: usize,
    /// Bisection steps attempted for each root in one refinement round.
    pub steps_per_round: usize,
}

impl Default for AlgebraicRootRefinementComparisonConfig {
    fn default() -> Self {
        Self {
            policy: PredicatePolicy::APPROXIMATE_512,
            max_refinement_rounds: 16,
            steps_per_round: 1,
        }
    }
}

/// Overlap-aware comparison report for represented algebraic roots.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRefinementComparisonReport {
    /// Final comparison after optional refinement.
    pub comparison: AlgebraicRootComparisonReport,
    /// Last left representation considered by the comparison.
    pub refined_left: AlgebraicRootRepresentation,
    /// Last right representation considered by the comparison.
    pub refined_right: AlgebraicRootRepresentation,
    /// Refinement reports applied to the left root.
    pub left_refinements: Vec<IsolatedRootRefinementReport>,
    /// Refinement reports applied to the right root.
    pub right_refinements: Vec<IsolatedRootRefinementReport>,
    /// Number of refinement rounds entered.
    pub refinement_rounds: usize,
}

/// Difference-backed comparison report for constructed algebraic values.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootDifferenceComparisonReport {
    /// First comparison pass, using direct interval order and configured
    /// source-root refinement.
    pub refinement: AlgebraicRootRefinementComparisonReport,
    /// Arithmetic evidence for `left - right` when the refinement pass still
    /// left overlapping intervals.
    pub difference: Option<AlgebraicRootArithmeticReport>,
    /// Final comparison after optional difference construction.
    pub comparison: AlgebraicRootComparisonReport,
}

/// Arithmetic operation over represented algebraic roots.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicRootArithmeticOp {
    /// Add two represented roots.
    Add,
    /// Subtract the right represented root from the left.
    Subtract,
    /// Multiply two represented roots.
    Multiply,
    /// Divide the left represented root by the right.
    Divide,
    /// Negate the left represented root.
    Negate,
}

/// Arithmetic status for represented algebraic roots.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootArithmeticStatus {
    /// The operation was computed exactly from rational witnesses.
    ComputedExactRationalWitness,
    /// The operation produced a general exact [`Real`] witness without a
    /// stored rational payload.
    ComputedExactRealWitness,
    /// The operation produced a new exact represented algebraic root.
    ComputedRepresentation,
    /// One or both inputs were invalid.
    InvalidEvidence,
    /// The operation needs non-rational algebraic-number arithmetic that this
    /// exact witness package intentionally does not claim.
    NonRationalInput,
    /// Exact arithmetic in the scalar package did not decide.
    Undecided,
}

/// Report for a narrow exact arithmetic operation on represented roots.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootArithmeticReport {
    /// Operation requested by the caller.
    pub operation: AlgebraicRootArithmeticOp,
    /// Arithmetic status.
    pub status: AlgebraicRootArithmeticStatus,
    /// Exact result when the operation is supported.
    pub exact_result: Option<Real>,
    /// Exact represented result when the operation preserves algebraic-root
    /// evidence but does not collapse to a rational witness.
    pub result_representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Status for constructing an affine image of a represented algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootAffineTransformStatus {
    /// The affine image was represented exactly.
    Transformed,
    /// The input representation failed structural validation.
    InvalidEvidence,
    /// The scale was exactly zero, so the image is not an invertible algebraic
    /// coordinate transform.
    ZeroScale,
    /// The transformed polynomial or interval could not be validated exactly.
    InvalidTransformedEvidence,
    /// Exact comparisons or coefficient arithmetic did not decide.
    Undecided,
}

/// Report for `beta = scale * alpha + offset`.
///
/// This is the first constructed represented-value operation for
/// non-rational algebraic roots. It is deliberately limited to invertible
/// affine transforms, where the defining polynomial can be changed exactly by
/// substitution and interval reflection without a full algebraic-number field.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootAffineTransformReport {
    /// Final transform status.
    pub status: AlgebraicRootAffineTransformStatus,
    /// Exact scale supplied by the caller.
    pub scale: Real,
    /// Exact offset supplied by the caller.
    pub offset: Real,
    /// Resulting represented root when construction succeeds.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Exact rational affine relation between two selected algebraic roots.
///
/// A returned relation certifies `right = scale * left + offset`.  The
/// coefficients are inferred from translation-invariant normalized
/// polynomial coefficients, but are accepted only after constructing the
/// affine image and replaying exact selected-root equality.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootAffineRelation {
    /// Nonzero exact-rational scale from the left root to the right root.
    pub scale: Real,
    /// Exact-rational offset from the scaled left root to the right root.
    pub offset: Real,
}

/// Status for evaluating a polynomial at a represented algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootPolynomialEvaluationStatus {
    /// Exact point evaluation produced a rational value.
    EvaluatedExactRationalWitness,
    /// Exact point evaluation produced a general exact [`Real`] value without
    /// a stored rational payload.
    EvaluatedExactRealWitness,
    /// Interval arithmetic proved the polynomial value is strictly positive
    /// throughout the isolating interval.
    IntervalCertifiedPositive,
    /// Interval arithmetic proved the polynomial value is strictly negative
    /// throughout the isolating interval.
    IntervalCertifiedNegative,
    /// The conservative interval enclosure contains zero.
    IntervalContainsZero,
    /// The represented root failed structural validation.
    InvalidEvidence,
    /// The evaluated polynomial is empty or has unsupported coefficients.
    InvalidPolynomial,
    /// Exact interval endpoint comparisons did not decide.
    Undecided,
}

/// Exact or conservative value report for `q(alpha)`.
///
/// The input polynomial is stored in ascending power order. For exact point
/// witnesses, `exact_value` is filled and compared exactly. For interval-only
/// represented roots, `interval_value` is a conservative exact interval
/// enclosure produced without primitive-float sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootPolynomialEvaluationReport {
    /// Final evaluation status.
    pub status: AlgebraicRootPolynomialEvaluationStatus,
    /// Exact value when the represented root has a point witness.
    pub exact_value: Option<Real>,
    /// Conservative interval value for interval-only roots.
    pub interval_value: Option<AlgebraicPolynomialValueInterval>,
    /// Certified sign when the report proves one.
    pub sign: Option<Ordering>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Conservative exact interval enclosure for a polynomial value.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicPolynomialValueInterval {
    /// Lower endpoint of the value enclosure.
    pub lower: Real,
    /// Upper endpoint of the value enclosure.
    pub upper: Real,
}

/// Status for evaluating a rational expression at a represented root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootRationalEvaluationStatus {
    /// Exact point evaluation produced a rational quotient.
    EvaluatedExactRationalWitness,
    /// Exact point evaluation produced a general exact [`Real`] quotient
    /// without a stored rational payload.
    EvaluatedExactRealWitness,
    /// Interval arithmetic certified a nonzero denominator and produced a
    /// rational value enclosure.
    IntervalEvaluated,
    /// The denominator is exactly zero at the selected algebraic root.
    CertifiedZeroDenominator,
    /// The denominator interval contains zero, so division is not certified.
    DenominatorMayContainZero,
    /// The represented root failed structural validation.
    InvalidEvidence,
    /// The numerator or denominator polynomial was empty or unsupported.
    InvalidPolynomial,
    /// Exact interval endpoint comparisons did not decide.
    Undecided,
}

/// Exact or conservative report for `p(alpha) / q(alpha)`.
///
/// This is intentionally a domain-checking operation, not algebraic field
/// construction. It reports the numerator and denominator evidence separately
/// so callers can distinguish an unknown denominator from an unknown quotient.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRationalEvaluationReport {
    /// Final rational evaluation status.
    pub status: AlgebraicRootRationalEvaluationStatus,
    /// Numerator evaluation evidence.
    pub numerator: AlgebraicRootPolynomialEvaluationReport,
    /// Denominator evaluation evidence.
    pub denominator: AlgebraicRootPolynomialEvaluationReport,
    /// Exact quotient when the root has a point witness and the denominator
    /// is certified nonzero.
    pub exact_value: Option<Real>,
    /// Conservative interval quotient for interval-only roots.
    pub interval_value: Option<AlgebraicPolynomialValueInterval>,
    /// Certified sign when the quotient sign is proved.
    pub sign: Option<Ordering>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Validation report for a represented algebraic root.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootValidationReport {
    /// Validation status.
    pub status: AlgebraicRootValidationStatus,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

impl AlgebraicRootValidationReport {
    fn valid() -> Self {
        Self {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        }
    }

    fn invalid(status: AlgebraicRootValidationStatus, message: impl Into<String>) -> Self {
        Self {
            status,
            message: Some(message.into()),
        }
    }
}

/// Exact representation of one real algebraic root.
///
/// The polynomial is stored in ascending power order. The isolating interval
/// is trusted only after validation confirms the local shape of the evidence;
/// uniqueness itself comes from the upstream Sturm/Collins-Loos isolation
/// report, not from primitive-float sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRepresentation {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol represented by the polynomial variable.
    pub symbol: SymbolId,
    /// Root interval ordinal within the source isolation report.
    pub interval_index: usize,
    /// Exact coefficients in ascending power order.
    pub polynomial_coefficients: Vec<Real>,
    /// Certified unit isolating interval or exact point interval.
    pub interval: IsolatedRootInterval,
    /// Validation evidence for the representation.
    pub validation: AlgebraicRootValidationReport,
}

impl AlgebraicRootRepresentation {
    /// Returns the stored exact point witness, when present.
    ///
    /// Point witnesses may be any exact `Real`, not only rationals.
    #[inline]
    pub fn exact_point_witness(&self) -> Option<&Real> {
        self.interval.exact_root.as_ref()
    }

    /// Returns whether this representation passed structural validation.
    pub fn is_valid(&self) -> bool {
        self.validation.status == AlgebraicRootValidationStatus::Valid
    }
}

/// Row-level status for algebraic-root representation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootRepresentationStatus {
    /// Every isolated interval in the row was converted to a valid
    /// representation.
    Represented,
    /// The row was supported and certified to have no real roots.
    NoRealRoots,
    /// The upstream isolation row was unsupported or undecided.
    UnsupportedIsolationStatus,
    /// The isolation report did not identify a solver symbol.
    MissingSymbol,
    /// The row could not be extracted as an exact-rational univariate
    /// polynomial.
    MissingPolynomial,
    /// At least one interval failed representation validation.
    InvalidEvidence,
}

/// Algebraic-root representation report for one active equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRepresentationReport {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol represented by the polynomial variable, when available.
    pub symbol: Option<SymbolId>,
    /// Row status.
    pub status: AlgebraicRootRepresentationStatus,
    /// Represented roots for supported rows.
    pub roots: Vec<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Build represented algebraic roots for active univariate equality rows.
///
/// This function runs the existing exact root isolator, then wraps each
/// unit-root interval with the exact polynomial coefficients that define the
/// root. The result is suitable for downstream APIs that need a persistent
/// algebraic object but are not yet implementing arithmetic on algebraic
/// numbers.
pub fn represent_univariate_algebraic_roots(
    analysis: &ProblemAnalysis<'_>,
    config: RootIsolationConfig,
) -> Vec<AlgebraicRootRepresentationReport> {
    let reports = isolate_univariate_polynomial_roots_with_config(analysis, config.clone());
    represent_univariate_algebraic_roots_from_reports(analysis, &reports, config.policy)
}

/// Build represented algebraic roots from existing isolation reports.
///
/// The caller supplies the isolation reports so path/curve code can preserve a
/// previous isolation pass and still obtain persistent algebraic-root objects.
/// Reports are matched by `constraint_index`; inactive and non-equality rows
/// are rejected by construction rather than guessed from residual text.
pub fn represent_univariate_algebraic_roots_from_reports(
    analysis: &ProblemAnalysis<'_>,
    reports: &[UnivariateRootIsolationReport],
    policy: PredicatePolicy,
) -> Vec<AlgebraicRootRepresentationReport> {
    reports
        .iter()
        .map(|report| represent_one_report(analysis.problem(), report, policy))
        .collect()
}

/// Validate one represented algebraic root.
///
/// This checks the representation payload itself: a nonconstant polynomial
/// over exact [`Real`] coefficients, an ordered interval, a unit distinct-root
/// claim, and, when a point witness is present, exact polynomial replay at
/// that witness. It does not re-run Sturm isolation; the upstream isolation
/// report remains the proof of uniqueness. Rational-only construction
/// packages may impose a narrower coefficient field at their own boundary.
pub fn validate_algebraic_root_representation(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootValidationReport {
    validate_root_payload(&root.polynomial_coefficients, &root.interval, policy)
}

/// Compare two represented algebraic roots without leaving the exact boundary.
///
/// This is deliberately narrower than a complete algebraic-number ordering
/// package. Both cached-valid payloads are replayed under [`PredicatePolicy::STRICT`]
/// before the requested policy is used for ordering. It certifies order when
/// exact point witnesses compare directly or when the effective isolating
/// intervals are disjoint; a point witness supersedes its stored outer bounds.
/// If intervals overlap, the report returns
/// [`AlgebraicRootComparisonStatus::OverlappingIntervals`] instead of sampling
/// a primitive approximation. This follows the exact construction/decision
/// separation and the Collins-Loos isolating-interval model cited in the module
/// docs.
pub fn compare_algebraic_root_representations(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootComparisonReport {
    if !algebraic_root_comparison_inputs_replay_strictly(left, right) {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::InvalidEvidence,
            None,
            Some(
                "both algebraic roots must replay valid local evidence before comparison"
                    .to_owned(),
            ),
        );
    }
    compare_admitted_algebraic_root_representations(left, right, policy)
}

fn compare_admitted_algebraic_root_representations(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootComparisonReport {
    if same_represented_root(left, right) {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::SameRepresentation,
            Some(Ordering::Equal),
            None,
        );
    }
    if let (Some(left), Some(right)) = (left.exact_point_witness(), right.exact_point_witness()) {
        let Some(ordering) = compare_algebraic_values(left, right, policy) else {
            return algebraic_comparison_report(
                AlgebraicRootComparisonStatus::Undecided,
                None,
                Some("could not compare exact point witnesses".to_owned()),
            );
        };
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(ordering),
            None,
        );
    }

    let left_lower = left.exact_point_witness().unwrap_or(&left.interval.lower);
    let left_upper = left.exact_point_witness().unwrap_or(&left.interval.upper);
    let right_lower = right.exact_point_witness().unwrap_or(&right.interval.lower);
    let right_upper = right.exact_point_witness().unwrap_or(&right.interval.upper);

    let Some(left_before_right) = compare_algebraic_values(left_upper, right_lower, policy) else {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Undecided,
            None,
            Some("could not compare left upper endpoint to right lower endpoint".to_owned()),
        );
    };
    if left_before_right == Ordering::Less {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(Ordering::Less),
            None,
        );
    }

    let Some(left_after_right) = compare_algebraic_values(left_lower, right_upper, policy) else {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Undecided,
            None,
            Some("could not compare left lower endpoint to right upper endpoint".to_owned()),
        );
    };
    if left_after_right == Ordering::Greater {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(Ordering::Greater),
            None,
        );
    }

    // A validated unit-root isolator whose exact endpoints coincide is
    // already an exact point even when the producer did not populate the
    // optional point cache. Recognize two such points here so every consumer
    // shares the same equality authority instead of reconstructing a
    // difference merely to rediscover coincident singleton intervals.
    if compare_algebraic_values(left_lower, left_upper, PredicatePolicy::STRICT)
        == Some(Ordering::Equal)
        && compare_algebraic_values(right_lower, right_upper, PredicatePolicy::STRICT)
            == Some(Ordering::Equal)
        && compare_algebraic_values(left_lower, right_lower, policy) == Some(Ordering::Equal)
    {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(Ordering::Equal),
            None,
        );
    }

    algebraic_comparison_report(
        AlgebraicRootComparisonStatus::OverlappingIntervals,
        None,
        Some("isolating intervals overlap; refine before ordering".to_owned()),
    )
}

/// Compare represented algebraic roots, refining overlapping intervals first.
///
/// This is still not a full algebraic-number field package: it performs only
/// the exact predicate work needed for ordering. When ordinary comparison
/// stops at overlapping isolating intervals, every non-point root is refined
/// with a Sturm sequence package in the standard real-root isolation style
/// until the intervals become disjoint, an exact rational witness appears, or
/// the configured work budget is exhausted. Exact point operands are already
/// maximally refined and therefore have no refinement report. Input admission
/// is replayed once before the loop. Refinement is a proof-producing operation,
/// and failure remains an explicit undecided report rather than a sampled
/// approximation.
pub fn compare_algebraic_root_representations_with_refinement(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    config: AlgebraicRootRefinementComparisonConfig,
) -> AlgebraicRootRefinementComparisonReport {
    if !algebraic_root_comparison_inputs_replay_strictly(left, right) {
        return algebraic_refinement_comparison_report(
            algebraic_comparison_report(
                AlgebraicRootComparisonStatus::InvalidEvidence,
                None,
                Some(
                    "both algebraic roots must replay valid local evidence before refinement"
                        .to_owned(),
                ),
            ),
            left.clone(),
            right.clone(),
            Vec::new(),
            Vec::new(),
            0,
        );
    }
    let comparison = compare_admitted_algebraic_root_representations(left, right, config.policy);
    compare_admitted_algebraic_root_representations_with_refinement(left, right, config, comparison)
}

fn compare_admitted_algebraic_root_representations_with_refinement(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    config: AlgebraicRootRefinementComparisonConfig,
    mut comparison: AlgebraicRootComparisonReport,
) -> AlgebraicRootRefinementComparisonReport {
    let mut refined_left = left.clone();
    let mut refined_right = right.clone();
    let mut left_refinements = Vec::new();
    let mut right_refinements = Vec::new();
    let mut refinement_rounds = 0;
    if comparison.status != AlgebraicRootComparisonStatus::OverlappingIntervals {
        return algebraic_refinement_comparison_report(
            comparison,
            refined_left,
            refined_right,
            left_refinements,
            right_refinements,
            refinement_rounds,
        );
    }

    for round in 0..config.max_refinement_rounds {
        refinement_rounds = round + 1;
        let root_config = RootIsolationConfig {
            policy: config.policy,
            max_interval_width: None,
            max_refinement_steps: config.steps_per_round,
        };
        let left_progress = if refined_left.exact_point_witness().is_some() {
            true
        } else {
            let left_refinement = refine_isolated_univariate_polynomial_interval(
                &refined_left.polynomial_coefficients,
                &refined_left.interval,
                root_config.clone(),
            );
            let progress =
                apply_refined_interval(&mut refined_left, &left_refinement, config.policy);
            left_refinements.push(left_refinement);
            progress
        };
        let right_progress = if refined_right.exact_point_witness().is_some() {
            true
        } else {
            let right_refinement = refine_isolated_univariate_polynomial_interval(
                &refined_right.polynomial_coefficients,
                &refined_right.interval,
                root_config,
            );
            let progress =
                apply_refined_interval(&mut refined_right, &right_refinement, config.policy);
            right_refinements.push(right_refinement);
            progress
        };

        if !left_progress || !right_progress {
            comparison = algebraic_comparison_report(
                AlgebraicRootComparisonStatus::Undecided,
                None,
                Some(
                    "algebraic root refinement did not produce valid interval evidence".to_owned(),
                ),
            );
            break;
        }
        comparison = compare_admitted_algebraic_root_representations(
            &refined_left,
            &refined_right,
            config.policy,
        );
        if comparison.status != AlgebraicRootComparisonStatus::OverlappingIntervals {
            break;
        }
    }

    algebraic_refinement_comparison_report(
        comparison,
        refined_left,
        refined_right,
        left_refinements,
        right_refinements,
        refinement_rounds,
    )
}

/// Compare represented roots by constructing and signing `left - right`.
///
/// This is the comparison counterpart to the bounded independent arithmetic
/// package. It first tries ordinary interval comparison plus exact Sturm
/// refinement. If source intervals still overlap, it constructs the algebraic
/// difference and decides the sign of that constructed value from its
/// certified isolating interval. Exact point/isolator equality is checked by
/// polynomial replay at the point; interval-only equality uses a polynomial
/// GCD and root count. If refinement narrows the operands without separating
/// them, those narrowed carriers feed difference construction. This is the
/// exactness-boundary construction/decision split: the value is built as exact
/// evidence, then the sign predicate reads only certified interval data. The
/// implementation follows Sturm's theorem and the standard real-root isolation
/// model used by the arithmetic and refinement modules.
pub fn compare_algebraic_root_representations_by_difference(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    config: AlgebraicRootRefinementComparisonConfig,
) -> AlgebraicRootDifferenceComparisonReport {
    if !algebraic_root_comparison_inputs_replay_strictly(left, right) {
        let comparison = algebraic_comparison_report(
            AlgebraicRootComparisonStatus::InvalidEvidence,
            None,
            Some(
                "both algebraic roots must replay valid local evidence before difference comparison"
                    .to_owned(),
            ),
        );
        let refinement = algebraic_refinement_comparison_report(
            comparison.clone(),
            left.clone(),
            right.clone(),
            Vec::new(),
            Vec::new(),
            0,
        );
        return algebraic_difference_comparison_report(comparison, refinement, None);
    }
    let direct = compare_admitted_algebraic_root_representations(left, right, config.policy);
    if matches!(
        direct.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    ) {
        let refinement = algebraic_refinement_comparison_report(
            direct.clone(),
            left.clone(),
            right.clone(),
            Vec::new(),
            Vec::new(),
            0,
        );
        return algebraic_difference_comparison_report(direct, refinement, None);
    }
    let common_root =
        admitted_represented_roots_share_isolated_common_root(left, right, config.policy);
    if common_root == Some(true) {
        let comparison = algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(Ordering::Equal),
            Some(
                "equality certified by a shared polynomial root in the overlapping isolating intervals"
                    .to_owned(),
            ),
        );
        let refinement = algebraic_refinement_comparison_report(
            comparison.clone(),
            left.clone(),
            right.clone(),
            Vec::new(),
            Vec::new(),
            0,
        );
        return algebraic_difference_comparison_report(comparison, refinement, None);
    }
    let refinement = compare_admitted_algebraic_root_representations_with_refinement(
        left,
        right,
        config.clone(),
        direct,
    );
    if matches!(
        refinement.comparison.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    ) {
        return algebraic_difference_comparison_report(
            refinement.comparison.clone(),
            refinement,
            None,
        );
    }
    if common_root.is_none()
        && admitted_represented_roots_share_isolated_common_root(
            &refinement.refined_left,
            &refinement.refined_right,
            config.policy,
        ) == Some(true)
    {
        return algebraic_difference_comparison_report(
            algebraic_comparison_report(
                AlgebraicRootComparisonStatus::Compared,
                Some(Ordering::Equal),
                Some("equality certified by a shared polynomial root in the overlapping isolating intervals".to_owned()),
            ),
            refinement,
            None,
        );
    }
    if refinement.comparison.status != AlgebraicRootComparisonStatus::OverlappingIntervals {
        return algebraic_difference_comparison_report(
            refinement.comparison.clone(),
            refinement,
            None,
        );
    }
    if let Some(difference_value) = translated_algebraic_root_difference(
        &refinement.refined_left,
        &refinement.refined_right,
        config.policy,
    ) && let Some(ordering) = algebraic_value_sign(&difference_value, config.policy)
    {
        let comparison = algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(ordering),
            Some("comparison decided by an exact translated-root difference".to_owned()),
        );
        let difference = algebraic_arithmetic_report(
            AlgebraicRootArithmeticOp::Subtract,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
            Some(difference_value),
            None,
            Some("selected roots are exact translates of one another".to_owned()),
        );
        return algebraic_difference_comparison_report(comparison, refinement, Some(difference));
    }

    let difference = arithmetic_algebraic_root_representations(
        &refinement.refined_left,
        Some(&refinement.refined_right),
        AlgebraicRootArithmeticOp::Subtract,
        config.policy,
    );
    let comparison = match difference.status {
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        | AlgebraicRootArithmeticStatus::ComputedExactRealWitness => {
            let Some(value) = difference.exact_result.as_ref() else {
                return algebraic_difference_comparison_report(
                    algebraic_comparison_report(
                        AlgebraicRootComparisonStatus::Undecided,
                        None,
                        Some("difference arithmetic omitted exact witness".to_owned()),
                    ),
                    refinement,
                    Some(difference),
                );
            };
            match algebraic_value_sign(value, config.policy) {
                Some(ordering) => algebraic_comparison_report(
                    AlgebraicRootComparisonStatus::Compared,
                    Some(ordering),
                    Some("comparison decided by exact difference witness".to_owned()),
                ),
                None => algebraic_comparison_report(
                    AlgebraicRootComparisonStatus::Undecided,
                    None,
                    Some("could not compare exact difference to zero".to_owned()),
                ),
            }
        }
        AlgebraicRootArithmeticStatus::ComputedRepresentation => {
            let Some(root) = difference.result_representation.as_ref() else {
                return algebraic_difference_comparison_report(
                    algebraic_comparison_report(
                        AlgebraicRootComparisonStatus::Undecided,
                        None,
                        Some("difference arithmetic omitted represented result".to_owned()),
                    ),
                    refinement,
                    Some(difference),
                );
            };
            match represented_root_sign(root, config.policy) {
                Some(ordering) => algebraic_comparison_report(
                    AlgebraicRootComparisonStatus::Compared,
                    Some(ordering),
                    Some("comparison decided by constructed algebraic difference".to_owned()),
                ),
                None => algebraic_comparison_report(
                    AlgebraicRootComparisonStatus::Undecided,
                    None,
                    Some("constructed difference interval did not separate from zero".to_owned()),
                ),
            }
        }
        AlgebraicRootArithmeticStatus::InvalidEvidence => algebraic_comparison_report(
            AlgebraicRootComparisonStatus::InvalidEvidence,
            None,
            difference.message.clone(),
        ),
        AlgebraicRootArithmeticStatus::NonRationalInput
        | AlgebraicRootArithmeticStatus::Undecided => algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Undecided,
            None,
            difference.message.clone(),
        ),
    };
    algebraic_difference_comparison_report(comparison, refinement, Some(difference))
}

/// Finds `left - right` when the selected roots are related by an exact
/// translation inferred from their defining polynomials.
///
/// For degree `n` polynomials, translating `P(x)` to `P(y - offset)` changes
/// the normalized next-to-leading coefficient by `-n * offset`. The resulting
/// candidate is accepted only after affine construction and an exact
/// common-root proof on the translated isolator, so unrelated polynomials
/// cannot turn the coefficient heuristic into topology evidence.
pub fn translated_algebraic_root_difference(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Real> {
    if !algebraic_root_comparison_inputs_replay_strictly(left, right) {
        return None;
    }
    let left_coefficients = &left.polynomial_coefficients;
    let right_coefficients = &right.polynomial_coefficients;
    if left_coefficients.len() != right_coefficients.len() || left_coefficients.len() < 2 {
        return None;
    }
    if left_coefficients
        .iter()
        .chain(right_coefficients)
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return None;
    }
    let degree = left_coefficients.len() - 1;
    let left_leading = left_coefficients.last()?;
    let right_leading = right_coefficients.last()?;
    let left_next = &left_coefficients[degree - 1];
    let right_next = &right_coefficients[degree - 1];
    let left_normalized = (left_next.clone() / left_leading.clone()).ok()?;
    let right_normalized = (right_next.clone() / right_leading.clone()).ok()?;
    let degree = u64::try_from(degree).ok()?;
    let offset = ((left_normalized - right_normalized) / Real::from(degree)).ok()?;
    let translated = transform_algebraic_root_affine(left, Real::one(), offset.clone(), policy);
    if translated.status != AlgebraicRootAffineTransformStatus::Transformed {
        return None;
    }
    let translated = translated.representation.as_ref()?;
    (admitted_represented_roots_share_isolated_common_root(translated, right, policy) == Some(true))
        .then(|| -offset)
}

/// Finds an exact rational affine relation between two selected roots.
///
/// The defining polynomials are first centered at the mean of all of their
/// complex roots.  If `right = scale * left + offset`, corresponding centered
/// monic coefficients of codimension `m` differ by `scale^m`.  The first
/// informative coefficient therefore supplies at most two rational scale
/// candidates.  A candidate is returned only after [`transform_algebraic_root_affine`]
/// and an exact common-root proof certify the selected isolating intervals;
/// coefficient inference alone is never accepted as evidence.
pub fn algebraic_root_affine_relation(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootAffineRelation> {
    let left_coefficients = &left.polynomial_coefficients;
    let right_coefficients = &right.polynomial_coefficients;
    if !algebraic_root_comparison_inputs_replay_strictly(left, right)
        || left_coefficients.len() != right_coefficients.len()
        || left_coefficients.len() < 2
        || left_coefficients
            .iter()
            .chain(right_coefficients)
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return None;
    }

    let degree = left_coefficients.len() - 1;
    let degree_real = Real::from(u64::try_from(degree).ok()?);
    let left_leading = left_coefficients.last()?;
    let right_leading = right_coefficients.last()?;
    let left_mean = -((left_coefficients[degree - 1].clone() / left_leading.clone()).ok()?
        / degree_real.clone())
    .ok()?;
    let right_mean = -((right_coefficients[degree - 1].clone() / right_leading.clone()).ok()?
        / degree_real)
        .ok()?;
    let left_centered = affine_transformed_polynomial(
        left_coefficients,
        &Real::one(),
        &(-left_mean.clone()),
        policy,
    )?;
    let right_centered = affine_transformed_polynomial(
        right_coefficients,
        &Real::one(),
        &(-right_mean.clone()),
        policy,
    )?;
    if left_centered.len() != degree + 1 || right_centered.len() != degree + 1 {
        return None;
    }

    let mut scales = Vec::with_capacity(2);
    for coefficient_index in (0..degree.saturating_sub(1)).rev() {
        let left = (left_centered[coefficient_index].clone() / left_leading.clone()).ok()?;
        let right = (right_centered[coefficient_index].clone() / right_leading.clone()).ok()?;
        let left_zero = left.exact_rational_ref()?.is_zero();
        let right_zero = right.exact_rational_ref()?.is_zero();
        if left_zero || right_zero {
            if left_zero != right_zero {
                return None;
            }
            continue;
        }
        let exponent = degree.checked_sub(coefficient_index)?;
        let exponent_u32 = u32::try_from(exponent).ok()?;
        let scale = (right / left).ok()?.root_n(exponent_u32).ok()?;
        scale.exact_rational_ref()?;
        scales.push(scale.clone());
        if exponent.is_multiple_of(2) {
            scales.push(-scale);
        }
        break;
    }
    if scales.is_empty() {
        scales.push(Real::one());
    }

    for scale in scales {
        let offset = &right_mean - &scale * &left_mean;
        let transformed =
            transform_algebraic_root_affine(left, scale.clone(), offset.clone(), policy);
        if transformed.status != AlgebraicRootAffineTransformStatus::Transformed {
            continue;
        }
        let transformed = transformed.representation.as_ref()?;
        if admitted_represented_roots_share_isolated_common_root(transformed, right, policy)
            == Some(true)
        {
            return Some(AlgebraicRootAffineRelation { scale, offset });
        }
    }
    None
}

fn admitted_represented_roots_share_isolated_common_root(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<bool> {
    match (left.exact_point_witness(), right.exact_point_witness()) {
        (Some(left), Some(right)) => {
            return Some(compare_algebraic_values(left, right, policy)? == Ordering::Equal);
        }
        (Some(point), None) => {
            return admitted_exact_point_is_represented_root(point, right, policy);
        }
        (None, Some(point)) => {
            return admitted_exact_point_is_represented_root(point, left, policy);
        }
        (None, None) => {}
    }
    let lower = match compare_algebraic_values(&left.interval.lower, &right.interval.lower, policy)?
    {
        Ordering::Less => &right.interval.lower,
        Ordering::Equal | Ordering::Greater => &left.interval.lower,
    };
    let upper = match compare_algebraic_values(&left.interval.upper, &right.interval.upper, policy)?
    {
        Ordering::Greater => &right.interval.upper,
        Ordering::Equal | Ordering::Less => &left.interval.upper,
    };
    polynomials_share_one_root_in_interval(
        &left.polynomial_coefficients,
        &right.polynomial_coefficients,
        lower,
        upper,
        policy,
    )
}

fn admitted_exact_point_is_represented_root(
    point: &Real,
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<bool> {
    if compare_algebraic_values(point, &root.interval.lower, policy)? == Ordering::Less
        || compare_algebraic_values(point, &root.interval.upper, policy)? == Ordering::Greater
    {
        return Some(false);
    }
    Some(
        algebraic_value_sign(
            &evaluate_polynomial(&root.polynomial_coefficients, point),
            policy,
        )? == Ordering::Equal,
    )
}

/// Compute exact arithmetic for represented roots under an explicit predicate
/// policy.
///
/// This is deliberately a witness arithmetic package, not a full algebraic
/// number field. When both required inputs carry exact point witnesses, the
/// result is computed exactly in [`Real`]: rational results use the dedicated
/// rational-witness status, while non-rational results use the exact-`Real`
/// witness status. When exactly one binary operand is an exact point,
/// add/subtract/multiply/divide by that scalar is lowered to exact affine or
/// linear-fractional construction; unary negation is the same structural
/// operation specialized to `scale = -1`.
/// Independent represented roots use the bounded resultant construction when
/// supported. These operations transform retained algebraic evidence rather
/// than sampling approximations. Every comparison, validation, and refinement
/// obeys `policy`; direct identity and point paths additionally replay their
/// stored source evidence under `STRICT`, and a STRICT caller never crosses an
/// implicit `APPROXIMATE_512` terminal. Unsupported algebraic arithmetic
/// remains explicit. This follows the exact-object rule from the
/// exact-geometric-computation model.
pub fn arithmetic_algebraic_root_representations(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    if !left.is_valid() || right.is_some_and(|root| !root.is_valid()) {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::InvalidEvidence,
            None,
            None,
            Some("algebraic root arithmetic requires valid represented inputs".to_owned()),
        );
    }
    if operation != AlgebraicRootArithmeticOp::Negate && right.is_none() {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::InvalidEvidence,
            None,
            None,
            Some("binary algebraic root arithmetic requires a right input".to_owned()),
        );
    }
    if let Some(report) = arithmetic_with_exact_point_operands(left, right, operation, policy) {
        return report;
    }
    if let Some(report) = arithmetic_with_one_exact_point_scalar(left, right, operation, policy) {
        return report;
    }
    if let Some(report) = arithmetic_with_same_representation(left, right, operation, policy) {
        return report;
    }
    if let Some(report) =
        arithmetic_with_independent_representations(left, right, operation, policy)
    {
        return report;
    }
    if operation == AlgebraicRootArithmeticOp::Negate {
        if !algebraic_root_payload_replays_strictly(left)
            || right.is_some_and(|root| !algebraic_root_payload_replays_strictly(root))
        {
            return stale_arithmetic_input_report(operation);
        }
        let representation = negate_algebraic_root_representation(left);
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedRepresentation,
            None,
            Some(representation),
            None,
        );
    }
    algebraic_arithmetic_report(
        operation,
        AlgebraicRootArithmeticStatus::NonRationalInput,
        None,
        None,
        Some("represented-root arithmetic exceeded the supported exact constructions".to_owned()),
    )
}

pub(super) fn algebraic_root_payload_replays_strictly(root: &AlgebraicRootRepresentation) -> bool {
    if let Some(valid) = replay_rational_arithmetic_input(root) {
        return valid;
    }
    if let Some(point) = root.interval.exact_root.as_ref()
        && root.interval.distinct_root_count == 1
        && root.interval.lower == *point
        && root.interval.upper == *point
        && root.polynomial_coefficients.len() == 2
        && root.polynomial_coefficients[1]
            .exact_rational_ref()
            .is_some_and(Rational::is_one)
        && exact_values_are_negations(&root.polynomial_coefficients[0], point)
    {
        return true;
    }
    validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
        == AlgebraicRootValidationStatus::Valid
}

#[inline]
fn algebraic_root_comparison_inputs_replay_strictly(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> bool {
    left.is_valid()
        && right.is_valid()
        && algebraic_root_payload_replays_strictly(left)
        && algebraic_root_payload_replays_strictly(right)
}

#[inline]
fn compare_algebraic_values(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        return left.partial_cmp(right);
    }
    compare_reals(left, right, policy).value()
}

#[inline]
fn algebraic_value_sign(value: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    if let Some(value) = value.exact_rational_ref() {
        return Some(if value.is_negative() {
            Ordering::Less
        } else if value.is_zero() {
            Ordering::Equal
        } else {
            Ordering::Greater
        });
    }
    compare_reals(value, &Real::zero(), policy).value()
}

fn exact_values_are_negations(left: &Real, right: &Real) -> bool {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        return rational_values_are_negations(left, right);
    }
    left.is_structural_negation_of(right)
}

#[inline]
fn rational_values_are_negations(left: &Rational, right: &Rational) -> bool {
    if left.is_zero() || right.is_zero() {
        left.is_zero() && right.is_zero()
    } else {
        left.is_negative() != right.is_negative()
            && left.numerator() == right.numerator()
            && left.denominator() == right.denominator()
    }
}

fn stale_arithmetic_input_report(
    operation: AlgebraicRootArithmeticOp,
) -> AlgebraicRootArithmeticReport {
    algebraic_arithmetic_report(
        operation,
        AlgebraicRootArithmeticStatus::InvalidEvidence,
        None,
        None,
        Some("algebraic root arithmetic rejected stale represented input evidence".to_owned()),
    )
}

fn replay_rational_arithmetic_input(root: &AlgebraicRootRepresentation) -> Option<bool> {
    if root.polynomial_coefficients.len() <= 1 {
        return Some(false);
    }
    let leading = root.polynomial_coefficients.last()?.exact_rational_ref()?;
    let lower = root.interval.lower.exact_rational_ref()?;
    let upper = root.interval.upper.exact_rational_ref()?;
    if let Some(point) = root
        .interval
        .exact_root
        .as_ref()
        .and_then(Real::exact_rational_ref)
        && root.interval.distinct_root_count == 1
        && root.polynomial_coefficients.len() == 2
        && leading.is_one()
        && lower == point
        && upper == point
        && root.polynomial_coefficients[0]
            .exact_rational_ref()
            .is_some_and(|constant| rational_values_are_negations(constant, point))
    {
        return Some(true);
    }
    if leading.is_zero() || lower > upper || root.interval.distinct_root_count != 1 {
        return Some(false);
    }
    let Some(point) = root.interval.exact_root.as_ref() else {
        return root
            .polynomial_coefficients
            .iter()
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
            .then_some(true);
    };
    let point = point.exact_rational_ref()?;
    if point < lower || point > upper {
        return Some(false);
    }
    if root.polynomial_coefficients.len() == 2
        && leading.is_one()
        && root.polynomial_coefficients[0]
            .exact_rational_ref()
            .is_some_and(|constant| rational_values_are_negations(constant, point))
    {
        return Some(true);
    }
    let mut value = Rational::zero();
    for coefficient in root.polynomial_coefficients.iter().rev() {
        value = &value * point + coefficient.exact_rational_ref()?;
    }
    Some(value.is_zero())
}

fn arithmetic_with_exact_point_operands(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    let left_value = left.exact_point_witness()?;
    let right_value = if operation == AlgebraicRootArithmeticOp::Negate {
        None
    } else {
        Some(right?.exact_point_witness()?)
    };
    if !algebraic_root_payload_replays_strictly(left)
        || right.is_some_and(|root| !algebraic_root_payload_replays_strictly(root))
    {
        return Some(stale_arithmetic_input_report(operation));
    }
    let result = match operation {
        AlgebraicRootArithmeticOp::Negate => -left_value.clone(),
        AlgebraicRootArithmeticOp::Add
        | AlgebraicRootArithmeticOp::Subtract
        | AlgebraicRootArithmeticOp::Multiply
        | AlgebraicRootArithmeticOp::Divide => {
            let right_value = right_value.expect("binary exact-point arithmetic has a right value");
            match operation {
                AlgebraicRootArithmeticOp::Add => left_value.clone() + right_value.clone(),
                AlgebraicRootArithmeticOp::Subtract => left_value.clone() - right_value.clone(),
                AlgebraicRootArithmeticOp::Multiply => left_value.clone() * right_value.clone(),
                AlgebraicRootArithmeticOp::Divide => {
                    match left_value.clone() / right_value.clone() {
                        Ok(quotient) => quotient,
                        Err(_) => {
                            let Some(reciprocal) =
                                crate::policy_division::reciprocal_with_policy(right_value, policy)
                            else {
                                return Some(algebraic_arithmetic_report(
                                    operation,
                                    AlgebraicRootArithmeticStatus::Undecided,
                                    None,
                                    None,
                                    Some("exact point division failed".to_owned()),
                                ));
                            };
                            left_value.clone() * reciprocal
                        }
                    }
                }
                AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
            }
        }
    };
    Some(exact_point_arithmetic_report(operation, result))
}

fn exact_point_arithmetic_report(
    operation: AlgebraicRootArithmeticOp,
    result: Real,
) -> AlgebraicRootArithmeticReport {
    if result.exact_rational_ref().is_some() {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
            Some(result),
            None,
            None,
        );
    }
    algebraic_arithmetic_report(
        operation,
        AlgebraicRootArithmeticStatus::ComputedExactRealWitness,
        Some(result),
        None,
        None,
    )
}

#[inline(always)]
pub(crate) fn canonical_linear_value_representation(
    source: &AlgebraicRootRepresentation,
    value: Real,
) -> AlgebraicRootRepresentation {
    let exact_root = Some(value.clone());
    let representation = AlgebraicRootRepresentation {
        constraint_index: source.constraint_index,
        symbol: source.symbol,
        interval_index: source.interval_index,
        polynomial_coefficients: vec![-value.clone(), Real::one()],
        interval: IsolatedRootInterval {
            lower: value.clone(),
            upper: value,
            exact_root,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport::valid(),
    };
    debug_assert_eq!(
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid,
        "canonical linear exact-value representation must validate",
    );
    representation
}

/// Lower a supported mixed exact-point/interval operation to an affine image.
///
/// the exact-geometric-computation model separates construction of exact
/// algebraic objects from later predicate decisions. A scalar affine image of one represented
/// root is a safe construction because `scale^n * P((y - offset) / scale)`
/// gives exact polynomial evidence for the image and preserves interval
/// evidence by exact endpoint transforms. An exact point divided by a
/// represented nonzero root is delegated to the linear-fractional construction
/// package. General products, sums, or quotients of two independent
/// non-rational algebraic roots need resultants or a full algebraic-number
/// package, so this helper refuses those cases.
fn arithmetic_with_one_exact_point_scalar(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    if operation == AlgebraicRootArithmeticOp::Negate {
        return None;
    }
    let right = right?;
    let left_value = left.exact_point_witness();
    let right_value = right.exact_point_witness();
    match (left_value, right_value) {
        (Some(_), Some(_)) | (None, None) => None,
        (Some(scalar), None) => {
            if !algebraic_root_payload_replays_strictly(left) {
                return Some(stale_arithmetic_input_report(operation));
            }
            match operation {
                AlgebraicRootArithmeticOp::Add => Some(affine_transform_arithmetic_report(
                    right,
                    Real::one(),
                    scalar.clone(),
                    operation,
                    policy,
                    "left exact point scalar added to represented right root",
                )),
                AlgebraicRootArithmeticOp::Subtract => Some(affine_transform_arithmetic_report(
                    right,
                    -Real::one(),
                    scalar.clone(),
                    operation,
                    policy,
                    "represented right root subtracted from left exact point scalar",
                )),
                AlgebraicRootArithmeticOp::Multiply => Some(multiply_by_exact_scalar_report(
                    right, scalar, operation, policy,
                )),
                AlgebraicRootArithmeticOp::Divide => Some(divide_exact_scalar_by_root_report(
                    right, scalar, operation, policy,
                )),
                AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
            }
        }
        (None, Some(scalar)) => {
            if !algebraic_root_payload_replays_strictly(right) {
                return Some(stale_arithmetic_input_report(operation));
            }
            match operation {
                AlgebraicRootArithmeticOp::Add => Some(affine_transform_arithmetic_report(
                    left,
                    Real::one(),
                    scalar.clone(),
                    operation,
                    policy,
                    "right exact point scalar added to represented left root",
                )),
                AlgebraicRootArithmeticOp::Subtract => Some(affine_transform_arithmetic_report(
                    left,
                    Real::one(),
                    -scalar.clone(),
                    operation,
                    policy,
                    "right exact point scalar subtracted from represented left root",
                )),
                AlgebraicRootArithmeticOp::Multiply => Some(multiply_by_exact_scalar_report(
                    left, scalar, operation, policy,
                )),
                AlgebraicRootArithmeticOp::Divide => Some(divide_by_exact_scalar_report(
                    left, scalar, operation, policy,
                )),
                AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
            }
        }
    }
}

fn multiply_by_exact_scalar_report(
    root: &AlgebraicRootRepresentation,
    scalar: &Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    let Some(ordering) = affine_scale_order(scalar, policy) else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not certify exact point scalar zero/nonzero for multiplication".to_owned()),
        );
    };
    if ordering == Ordering::Equal {
        if !algebraic_root_payload_replays_strictly(root) {
            return stale_arithmetic_input_report(operation);
        }
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
            Some(Real::zero()),
            None,
            None,
        );
    }
    affine_transform_arithmetic_report(
        root,
        scalar.clone(),
        Real::zero(),
        operation,
        policy,
        "represented root multiplied by nonzero exact point scalar",
    )
}

fn divide_by_exact_scalar_report(
    root: &AlgebraicRootRepresentation,
    scalar: &Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    let Some(ordering) = affine_scale_order(scalar, policy) else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not certify exact point divisor zero/nonzero".to_owned()),
        );
    };
    if ordering == Ordering::Equal {
        if !algebraic_root_payload_replays_strictly(root) {
            return stale_arithmetic_input_report(operation);
        }
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("division by a zero exact point is not a constructed value".to_owned()),
        );
    }
    let Ok(scale) = crate::policy_division::reciprocal_after_certified_nonzero(scalar) else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not invert exact point divisor exactly".to_owned()),
        );
    };
    affine_transform_arithmetic_report(
        root,
        scale,
        Real::zero(),
        operation,
        policy,
        "represented root divided by nonzero exact point scalar",
    )
}

fn divide_exact_scalar_by_root_report(
    root: &AlgebraicRootRepresentation,
    scalar: &Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    let Some(ordering) = affine_scale_order(scalar, policy) else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not certify exact point dividend zero/nonzero".to_owned()),
        );
    };
    if ordering == Ordering::Equal {
        if !algebraic_root_payload_replays_strictly(root) {
            return stale_arithmetic_input_report(operation);
        }
        return match represented_root_sign_admitted(root, policy) {
            Some(Ordering::Less | Ordering::Greater) => algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
                Some(Real::zero()),
                None,
                None,
            ),
            Some(Ordering::Equal) => algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::Undecided,
                None,
                None,
                Some("zero divided by a represented zero root is undefined".to_owned()),
            ),
            None => algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::Undecided,
                None,
                None,
                Some("could not certify the represented divisor away from zero".to_owned()),
            ),
        };
    }
    mobius_transform_arithmetic_report(
        root,
        Real::zero(),
        scalar.clone(),
        Real::one(),
        Real::zero(),
        operation,
        policy,
        "left exact point scalar divided by represented right root",
    )
}

/// Lowers operations involving the same represented root to polynomial images.
///
/// If both operands name the same algebraic object `alpha`, then `alpha +
/// alpha`, `alpha - alpha`, `alpha * alpha`, and `alpha / alpha` do not need a
/// general two-root algebraic-number field. They are respectively the exact
/// polynomial images `2*x`, `0`, `x^2`, and the constant `1` on the same
/// source evidence. This uses the resultant-backed image package and keeps the
/// the exact construction boundary explicit: independent non-rational operands still
/// return [`AlgebraicRootArithmeticStatus::NonRationalInput`].
fn arithmetic_with_same_representation(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    let right = right?;
    if !same_represented_root(left, right) {
        return None;
    }
    let image = match operation {
        AlgebraicRootArithmeticOp::Add => {
            return Some(affine_transform_arithmetic_report(
                left,
                Real::from(2),
                Real::zero(),
                operation,
                policy,
                "same represented root addition lowered to affine image",
            ));
        }
        AlgebraicRootArithmeticOp::Subtract => {
            if !algebraic_root_payload_replays_strictly(left) {
                return Some(stale_arithmetic_input_report(operation));
            }
            return Some(algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
                Some(Real::zero()),
                None,
                Some("same represented root subtracted from itself".to_owned()),
            ));
        }
        AlgebraicRootArithmeticOp::Multiply => {
            if let Some(report) = same_rational_quadratic_square_report(left, operation, policy) {
                return Some(report);
            }
            vec![Real::zero(), Real::zero(), Real::one()]
        }
        AlgebraicRootArithmeticOp::Divide => {
            if !algebraic_root_payload_replays_strictly(left) {
                return Some(stale_arithmetic_input_report(operation));
            }
            if !matches!(
                represented_root_sign_admitted(left, policy),
                Some(Ordering::Less | Ordering::Greater)
            ) {
                return Some(algebraic_arithmetic_report(
                    operation,
                    AlgebraicRootArithmeticStatus::Undecided,
                    None,
                    None,
                    Some(
                        "same represented root division requires nonzero root evidence".to_owned(),
                    ),
                ));
            }
            return Some(algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
                Some(Real::one()),
                None,
                Some("same nonzero represented root divided by itself".to_owned()),
            ));
        }
        AlgebraicRootArithmeticOp::Negate => return None,
    };
    let transform = transform_algebraic_root_polynomial_image(left, &image, policy);
    Some(match transform.status {
        AlgebraicRootPolynomialImageStatus::Transformed => {
            let exact_result = transform
                .representation
                .as_ref()
                .and_then(AlgebraicRootRepresentation::exact_point_witness)
                .filter(|value| value.exact_rational_ref().is_some())
                .cloned();
            if exact_result.is_some() {
                algebraic_arithmetic_report(
                    operation,
                    AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
                    exact_result,
                    None,
                    Some(
                        "same represented root arithmetic reduced to an exact rational witness"
                            .to_owned(),
                    ),
                )
            } else {
                algebraic_arithmetic_report(
                    operation,
                    AlgebraicRootArithmeticStatus::ComputedRepresentation,
                    None,
                    transform.representation,
                    Some("same represented root arithmetic lowered to polynomial image".to_owned()),
                )
            }
        }
        AlgebraicRootPolynomialImageStatus::InvalidEvidence
        | AlgebraicRootPolynomialImageStatus::InvalidTransformedEvidence => {
            algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::InvalidEvidence,
                None,
                transform.representation,
                transform.message,
            )
        }
        AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial
        | AlgebraicRootPolynomialImageStatus::ImageIsolationFailed
        | AlgebraicRootPolynomialImageStatus::UnsupportedDegree
        | AlgebraicRootPolynomialImageStatus::Undecided => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            transform.representation,
            transform.message,
        ),
    })
}

fn same_rational_quadratic_square_report(
    left: &AlgebraicRootRepresentation,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    let [constant, linear, quadratic] = left.polynomial_coefficients.as_slice() else {
        return None;
    };
    let constant = constant.exact_rational_ref()?;
    let linear = linear.exact_rational_ref()?;
    let quadratic = quadratic.exact_rational_ref()?;
    if quadratic.is_zero() {
        return Some(stale_arithmetic_input_report(operation));
    }
    let scale = -(linear / quadratic);
    let offset = -(constant / quadratic);
    if scale.is_zero() {
        if !algebraic_root_payload_replays_strictly(left) {
            return Some(stale_arithmetic_input_report(operation));
        }
        return Some(algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
            Some(Real::new(offset)),
            None,
            Some("same quadratic root square reduced to an exact rational witness".to_owned()),
        ));
    }
    Some(affine_transform_arithmetic_report(
        left,
        Real::new(scale),
        Real::new(offset),
        operation,
        policy,
        "same quadratic root square reduced through its defining relation",
    ))
}

/// Lowers independent add/subtract/multiply/divide to resultant-backed construction.
///
/// This is the first bounded algebraic-number arithmetic slice for two
/// non-rational operands.  The helper delegates to
/// [`crate::transform_algebraic_roots_binary`], which uses Sylvester
/// resultants, Sturm isolation, and standard real-root isolation.
/// Division is accepted only when the binary construction package certifies
/// the divisor interval away from zero without a primitive-float shortcut.
fn arithmetic_with_independent_representations(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    let right = right?;
    if left.exact_point_witness().is_some() || right.exact_point_witness().is_some() {
        return None;
    }
    if !matches!(
        operation,
        AlgebraicRootArithmeticOp::Add
            | AlgebraicRootArithmeticOp::Subtract
            | AlgebraicRootArithmeticOp::Multiply
            | AlgebraicRootArithmeticOp::Divide
    ) {
        return None;
    }
    let transform = transform_algebraic_roots_binary(left, right, operation, policy);
    Some(match transform.status {
        AlgebraicRootBinaryTransformStatus::Transformed => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedRepresentation,
            None,
            transform.representation,
            Some("independent represented-root arithmetic lowered to exact resultant".to_owned()),
        ),
        AlgebraicRootBinaryTransformStatus::InvalidEvidence
        | AlgebraicRootBinaryTransformStatus::InvalidTransformedEvidence => {
            algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::InvalidEvidence,
                None,
                transform.representation,
                transform.message,
            )
        }
        AlgebraicRootBinaryTransformStatus::UnsupportedCoefficient => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::NonRationalInput,
            None,
            transform.representation,
            transform.message,
        ),
        AlgebraicRootBinaryTransformStatus::UnsupportedOperation
        | AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
        | AlgebraicRootBinaryTransformStatus::UnsupportedDegree
        | AlgebraicRootBinaryTransformStatus::NonIsolatingImageInterval
        | AlgebraicRootBinaryTransformStatus::Undecided => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            transform.representation,
            transform.message,
        ),
    })
}

fn affine_transform_arithmetic_report(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
    success_message: &str,
) -> AlgebraicRootArithmeticReport {
    let transform = transform_algebraic_root_affine(root, scale, offset, policy);
    match transform.status {
        AlgebraicRootAffineTransformStatus::Transformed => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedRepresentation,
            None,
            transform.representation,
            Some(success_message.to_owned()),
        ),
        AlgebraicRootAffineTransformStatus::InvalidEvidence
        | AlgebraicRootAffineTransformStatus::InvalidTransformedEvidence => {
            algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::InvalidEvidence,
                None,
                transform.representation,
                transform.message,
            )
        }
        AlgebraicRootAffineTransformStatus::ZeroScale
        | AlgebraicRootAffineTransformStatus::Undecided => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            transform.representation,
            transform.message,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn mobius_transform_arithmetic_report(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
    success_message: &str,
) -> AlgebraicRootArithmeticReport {
    let transform = transform_algebraic_root_mobius(
        root,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    );
    match transform.status {
        AlgebraicRootMobiusTransformStatus::Transformed => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedRepresentation,
            None,
            transform.representation,
            Some(success_message.to_owned()),
        ),
        AlgebraicRootMobiusTransformStatus::InvalidEvidence
        | AlgebraicRootMobiusTransformStatus::InvalidTransformedEvidence => {
            algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::InvalidEvidence,
                None,
                transform.representation,
                transform.message,
            )
        }
        AlgebraicRootMobiusTransformStatus::NonInvertible
        | AlgebraicRootMobiusTransformStatus::DenominatorMayVanish
        | AlgebraicRootMobiusTransformStatus::Undecided => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            transform.representation,
            transform.message,
        ),
    }
}

/// Construct the exact affine image `beta = scale * alpha + offset`.
///
/// If `alpha` is represented by `P(x)` of degree `n` and `scale != 0`, then
/// `beta` is represented by `scale^n * P((y - offset) / scale)`. This keeps
/// all coefficients exact and preserves the isolating interval by transforming
/// endpoints, rather than sampling a midpoint. Positive-width isolators own
/// `(lower, upper]`; a decreasing transform therefore boundedly refines any
/// source endpoint roots before reversing the bounds. The operation follows
/// the exact-object boundary from the exact-geometric-computation model:
/// construction returns retained algebraic evidence, and later callers still
/// use comparison/evaluation reports for certified decisions.
pub fn transform_algebraic_root_affine(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootAffineTransformReport {
    if !root.is_valid()
        || validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::InvalidEvidence,
            scale,
            offset,
            None,
            Some("algebraic root representation must be valid before transformation".to_owned()),
        );
    }
    let Some(scale_sign) = affine_scale_order(&scale, policy) else {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::Undecided,
            scale,
            offset,
            None,
            Some("could not certify affine transform scale sign".to_owned()),
        );
    };
    if scale_sign == Ordering::Equal {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::ZeroScale,
            scale,
            offset,
            None,
            Some("affine algebraic-root construction requires nonzero scale".to_owned()),
        );
    }
    if root.exact_point_witness().is_none() && scale_sign == Ordering::Less {
        let endpoints_are_roots = if integer_polynomial_modularly_excludes_endpoint_roots(
            &root.polynomial_coefficients,
            [&root.interval.lower, &root.interval.upper],
        ) {
            Some(false)
        } else {
            algebraic_root_interval_endpoints_are_roots_slow(root, policy)
        };
        match endpoints_are_roots {
            Some(true) => {
                return transform_affine_after_ownership_refinement(root, scale, offset, policy);
            }
            Some(false) => {}
            None => {
                return algebraic_affine_transform_report(
                    AlgebraicRootAffineTransformStatus::Undecided,
                    scale,
                    offset,
                    None,
                    Some(
                        "could not decide source endpoint ownership for decreasing affine transform"
                            .to_owned(),
                    ),
                );
            }
        }
    }
    if let Some(source_value) = root.exact_point_witness() {
        if let (Some(source_value), Some(scale_value), Some(offset_value)) = (
            source_value.exact_rational_ref(),
            scale.exact_rational_ref(),
            offset.exact_rational_ref(),
        ) {
            let value = Real::from(scale_value * source_value + offset_value);
            return exact_rational_affine_image(root, scale, offset, value);
        }
        let value = eval_affine(source_value, &scale, &offset);
        return exact_affine_image(root, scale, offset, value);
    }
    let Some(polynomial_coefficients) =
        affine_transformed_polynomial(&root.polynomial_coefficients, &scale, &offset, policy)
    else {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::Undecided,
            scale,
            offset,
            None,
            Some("could not construct transformed polynomial exactly".to_owned()),
        );
    };
    let Some(interval) = affine_transformed_interval(&root.interval, &scale, &offset, scale_sign)
    else {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::Undecided,
            scale,
            offset,
            None,
            Some("could not construct transformed isolating interval exactly".to_owned()),
        );
    };
    finish_affine_transform(
        root,
        scale,
        offset,
        polynomial_coefficients,
        interval,
        policy,
    )
}

#[cold]
fn transform_affine_after_ownership_refinement(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootAffineTransformReport {
    let Some(refined) = refine_reversed_algebraic_root_ownership(root, policy) else {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::Undecided,
            scale,
            offset,
            None,
            Some(
                "could not refine source endpoint ownership for decreasing affine transform"
                    .to_owned(),
            ),
        );
    };
    transform_algebraic_root_affine(&refined, scale, offset, policy)
}

/// Decides whether either stored endpoint annihilates a represented root's
/// defining polynomial.
///
/// Positive-width isolators own `(lower, upper]`.  An orientation-reversing
/// map swaps the numeric endpoints but not that ownership convention, so any
/// defining-polynomial root at either endpoint must be removed or collapsed
/// before the image interval can be stored safely.
#[inline(never)]
pub(crate) fn algebraic_root_interval_endpoints_are_roots(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<bool> {
    if integer_polynomial_modularly_excludes_endpoint_roots(
        &root.polynomial_coefficients,
        [&root.interval.lower, &root.interval.upper],
    ) {
        return Some(false);
    }
    algebraic_root_interval_endpoints_are_roots_slow(root, policy)
}

#[cold]
fn algebraic_root_interval_endpoints_are_roots_slow(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<bool> {
    for endpoint in [&root.interval.lower, &root.interval.upper] {
        if compare_reals(
            &Real::eval_poly(&root.polynomial_coefficients, endpoint),
            &Real::zero(),
            policy,
        )
        .value()?
            == Ordering::Equal
        {
            return Some(true);
        }
    }
    Some(false)
}

const ENDPOINT_NONVANISHING_PRIME: u64 = 4_294_967_291;

#[inline(always)]
fn integer_polynomial_modularly_excludes_endpoint_roots(
    polynomial: &[Real],
    endpoints: [&Real; 2],
) -> bool {
    // One nonzero modular image is a complete certificate that an integer
    // polynomial does not vanish.  This keeps the overwhelmingly common small
    // integer endpoint check allocation-free; a zero residue is only
    // inconclusive and falls through to exact rational or Real evaluation.
    let Some(lower) = endpoints[0]
        .exact_rational_ref()
        .and_then(integer_mod_prime)
    else {
        return false;
    };
    let Some(upper) = endpoints[1]
        .exact_rational_ref()
        .and_then(integer_mod_prime)
    else {
        return false;
    };
    let mut values = [0_u64; 2];
    for coefficient in polynomial.iter().rev() {
        let Some(coefficient) = coefficient.exact_rational_ref().and_then(integer_mod_prime) else {
            return false;
        };
        values[0] = (values[0] * lower + coefficient) % ENDPOINT_NONVANISHING_PRIME;
        values[1] = (values[1] * upper + coefficient) % ENDPOINT_NONVANISHING_PRIME;
    }
    values.into_iter().all(|value| value != 0)
}

#[inline(always)]
fn integer_mod_prime(value: &Rational) -> Option<u64> {
    if !value.is_integer() {
        return None;
    }
    let magnitude = u64::try_from(value.numerator()).ok()? % ENDPOINT_NONVANISHING_PRIME;
    Some(if value.is_negative() && magnitude != 0 {
        ENDPOINT_NONVANISHING_PRIME - magnitude
    } else {
        magnitude
    })
}

/// Boundedly refines a source isolator until reversing it preserves `(l, u]`
/// root ownership, or until the selected endpoint root becomes an exact point.
pub(crate) fn refine_reversed_algebraic_root_ownership(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Box<AlgebraicRootRepresentation>> {
    let mut refined_root = Box::new(root.clone());
    for round in 0..=ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS {
        if refined_root.interval.exact_root.is_some()
            || !algebraic_root_interval_endpoints_are_roots(&refined_root, policy)?
        {
            return Some(refined_root);
        }
        if round == ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS {
            break;
        }
        let refinement = refine_isolated_univariate_polynomial_interval(
            &refined_root.polynomial_coefficients,
            &refined_root.interval,
            RootIsolationConfig {
                policy,
                max_interval_width: None,
                max_refinement_steps: ALGEBRAIC_IMAGE_REFINEMENT_STEPS,
            },
        );
        if !matches!(
            refinement.status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ) {
            return None;
        }
        let interval = refinement.refined_interval?;
        if interval == refined_root.interval {
            return None;
        }
        refined_root.interval = interval;
        refined_root.validation = validate_algebraic_root_representation(&refined_root, policy);
        if !refined_root.is_valid() {
            return None;
        }
    }
    None
}

fn exact_affine_image(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    value: Real,
) -> AlgebraicRootAffineTransformReport {
    let representation = canonical_linear_value_representation(root, value);
    algebraic_affine_transform_report(
        AlgebraicRootAffineTransformStatus::Transformed,
        scale,
        offset,
        Some(representation),
        None,
    )
}

#[inline(always)]
fn exact_rational_affine_image(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    value: Real,
) -> AlgebraicRootAffineTransformReport {
    let representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients: vec![-value.clone(), Real::one()],
        interval: IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value),
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport::valid(),
    };
    debug_assert_eq!(
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid,
        "canonical exact affine image must validate",
    );
    algebraic_affine_transform_report(
        AlgebraicRootAffineTransformStatus::Transformed,
        scale,
        offset,
        Some(representation),
        None,
    )
}

fn finish_affine_transform(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    polynomial_coefficients: Vec<Real>,
    interval: IsolatedRootInterval,
    policy: PredicatePolicy,
) -> AlgebraicRootAffineTransformReport {
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        validation: AlgebraicRootValidationReport::valid(),
    };
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    if !representation.is_valid() {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::InvalidTransformedEvidence,
            scale,
            offset,
            Some(representation),
            Some("transformed algebraic-root evidence did not validate".to_owned()),
        );
    }
    algebraic_affine_transform_report(
        AlgebraicRootAffineTransformStatus::Transformed,
        scale,
        offset,
        Some(representation),
        None,
    )
}

/// Evaluate an exact [`Real`]-coefficient polynomial at a represented
/// algebraic root.
///
/// This is the first consumer-facing scalar operation for
/// [`AlgebraicRootRepresentation`]. If the root carries an exact point
/// witness, the value is computed by exact Horner evaluation. Otherwise the
/// polynomial is evaluated over the isolating interval with conservative
/// interval arithmetic and the sign is certified only when the whole enclosure
/// lies on one side of zero. This follows the exact-object/certified-decision
/// split in the exact-geometric-computation model, and the isolating-interval model of the standard real-root isolation model.
pub fn evaluate_polynomial_at_algebraic_root(
    root: &AlgebraicRootRepresentation,
    polynomial_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootPolynomialEvaluationReport {
    if !root.is_valid() || !algebraic_root_payload_replays_strictly(root) {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence,
            None,
            None,
            None,
            Some(
                "algebraic root representation must be cached-valid and replay strictly before evaluation"
                    .to_owned(),
            ),
        );
    }
    evaluate_polynomial_at_admitted_algebraic_root(root, polynomial_coefficients, policy)
}

fn evaluate_polynomial_at_admitted_algebraic_root(
    root: &AlgebraicRootRepresentation,
    polynomial_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootPolynomialEvaluationReport {
    if polynomial_coefficients.is_empty() {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial,
            None,
            None,
            None,
            Some("polynomial evaluation requires at least one coefficient".to_owned()),
        );
    }
    let Some(polynomial) = trim_polynomial_for_evaluation(polynomial_coefficients, policy) else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            None,
            None,
            Some("could not trim evaluated polynomial coefficients exactly".to_owned()),
        );
    };
    if let Some(witness) = root.exact_point_witness() {
        let value = evaluate_polynomial(polynomial, witness);
        let is_rational = value.exact_rational_ref().is_some();
        let Some(sign) = exact_evaluation_value_sign(&value, policy) else {
            return algebraic_polynomial_evaluation_report(
                AlgebraicRootPolynomialEvaluationStatus::Undecided,
                Some(value),
                None,
                None,
                Some("could not certify exact point value sign".to_owned()),
            );
        };
        let status = if is_rational {
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
        } else {
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        };
        return algebraic_polynomial_evaluation_report(status, Some(value), None, Some(sign), None);
    }

    let Some(interval) = evaluate_polynomial_interval(
        polynomial,
        &AlgebraicPolynomialValueInterval {
            lower: root.interval.lower.clone(),
            upper: root.interval.upper.clone(),
        },
        policy,
    ) else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            None,
            None,
            Some("could not order interval polynomial endpoints exactly".to_owned()),
        );
    };
    let Some(lower_cmp) = compare_reals(&interval.lower, &Real::zero(), policy).value() else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            Some(interval),
            None,
            Some("could not compare interval lower endpoint with zero".to_owned()),
        );
    };
    let Some(upper_cmp) = compare_reals(&interval.upper, &Real::zero(), policy).value() else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            Some(interval),
            None,
            Some("could not compare interval upper endpoint with zero".to_owned()),
        );
    };
    if lower_cmp == Ordering::Greater {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive,
            None,
            Some(interval),
            Some(Ordering::Greater),
            None,
        );
    }
    if upper_cmp == Ordering::Less {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedNegative,
            None,
            Some(interval),
            Some(Ordering::Less),
            None,
        );
    }
    algebraic_polynomial_evaluation_report(
        AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero,
        None,
        Some(interval),
        None,
        Some("interval polynomial enclosure contains zero".to_owned()),
    )
}

/// Evaluate an exact [`Real`]-coefficient rational expression at a represented root.
///
/// Numerator and denominator are supplied in ascending power order. The
/// denominator must be certified nonzero before division is performed. For
/// point witnesses, this is exact scalar division in [`Real`]. For
/// interval-only roots, denominator intervals that contain zero remain explicit
/// domain uncertainty; certified positive or negative denominator intervals
/// are inverted conservatively and multiplied by the numerator interval. This
/// is the rational-expression companion to
/// [`evaluate_polynomial_at_algebraic_root`] and follows the exact
/// construction/certified-decision split the exact-geometric-computation model without approximating the represented root.
pub fn evaluate_rational_expression_at_algebraic_root(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootRationalEvaluationReport {
    if !root.is_valid() || !algebraic_root_payload_replays_strictly(root) {
        let invalid = algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence,
            None,
            None,
            None,
            Some(
                "algebraic root representation must be cached-valid and replay strictly before evaluation"
                    .to_owned(),
            ),
        );
        return rational_expression_evaluation_from_polynomial_reports(
            invalid.clone(),
            invalid,
            policy,
        );
    }
    let numerator =
        evaluate_polynomial_at_admitted_algebraic_root(root, numerator_coefficients, policy);
    let denominator =
        evaluate_polynomial_at_admitted_algebraic_root(root, denominator_coefficients, policy);
    rational_expression_evaluation_from_polynomial_reports(numerator, denominator, policy)
}

pub(crate) fn evaluate_rational_expression_with_denominator_evaluation(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator: AlgebraicRootPolynomialEvaluationReport,
    policy: PredicatePolicy,
) -> AlgebraicRootRationalEvaluationReport {
    // The denominator report came from the public evaluator for this same
    // immutable root. A non-invalid report therefore already proves strict
    // source admission for every numerator in the shared-denominator batch.
    // Preserve the public invalid-root report shape by cloning its identical
    // polynomial evidence instead of evaluating against rejected storage.
    let numerator = if matches!(
        denominator.status,
        AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
    ) {
        denominator.clone()
    } else {
        evaluate_polynomial_at_admitted_algebraic_root(root, numerator_coefficients, policy)
    };
    rational_expression_evaluation_from_polynomial_reports(numerator, denominator, policy)
}

fn rational_expression_evaluation_from_polynomial_reports(
    numerator: AlgebraicRootPolynomialEvaluationReport,
    denominator: AlgebraicRootPolynomialEvaluationReport,
    policy: PredicatePolicy,
) -> AlgebraicRootRationalEvaluationReport {
    if matches!(
        numerator.status,
        AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
    ) || matches!(
        denominator.status,
        AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
    ) {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::InvalidEvidence,
            numerator,
            denominator,
            None,
            None,
            None,
            Some(
                "algebraic root representation must be cached-valid and replay strictly before evaluation"
                    .to_owned(),
            ),
        );
    }
    if matches!(
        numerator.status,
        AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial
    ) || matches!(
        denominator.status,
        AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial
    ) {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::InvalidPolynomial,
            numerator,
            denominator,
            None,
            None,
            None,
            Some("rational expression evaluation requires supported numerator and denominator polynomials".to_owned()),
        );
    }

    if let (Some(numerator_value), Some(denominator_value)) = (
        numerator.exact_value.as_ref(),
        denominator.exact_value.as_ref(),
    ) {
        let Some(denominator_sign) = denominator.sign else {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::Undecided,
                numerator,
                denominator,
                None,
                None,
                None,
                Some("could not certify exact denominator sign".to_owned()),
            );
        };
        if denominator_sign == Ordering::Equal {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::CertifiedZeroDenominator,
                numerator,
                denominator,
                None,
                None,
                None,
                Some("denominator evaluates exactly to zero".to_owned()),
            );
        }
        if numerator.sign == Some(Ordering::Equal) {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness,
                numerator,
                denominator,
                Some(Real::zero()),
                None,
                Some(Ordering::Equal),
                None,
            );
        }
        let quotient_sign = numerator.sign.map(|numerator_sign| {
            if denominator_sign == Ordering::Less {
                numerator_sign.reverse()
            } else {
                numerator_sign
            }
        });
        if numerator_value == denominator_value {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness,
                numerator,
                denominator,
                Some(Real::one()),
                None,
                Some(Ordering::Greater),
                None,
            );
        }
        if denominator_value
            .exact_rational_ref()
            .is_some_and(Rational::is_one)
        {
            let quotient = numerator_value.clone();
            let status = if quotient.exact_rational_ref().is_some() {
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
            } else {
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness
            };
            return algebraic_rational_evaluation_report(
                status,
                numerator,
                denominator,
                Some(quotient),
                None,
                quotient_sign,
                None,
            );
        }
        if let (Some(numerator_value), Some(denominator_value)) = (
            numerator_value.exact_rational_ref(),
            denominator_value.exact_rational_ref(),
        ) {
            let quotient = Real::from(numerator_value / denominator_value);
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness,
                numerator,
                denominator,
                Some(quotient),
                None,
                quotient_sign,
                None,
            );
        }
        let Ok(reciprocal) =
            crate::policy_division::reciprocal_after_certified_nonzero(denominator_value)
        else {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::Undecided,
                numerator,
                denominator,
                None,
                None,
                None,
                Some("exact point division failed".to_owned()),
            );
        };
        let quotient = numerator_value.clone() * reciprocal;
        let sign = quotient_sign.or_else(|| exact_evaluation_value_sign(&quotient, policy));
        let status = if quotient.exact_rational_ref().is_some() {
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        } else {
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness
        };
        return algebraic_rational_evaluation_report(
            status,
            numerator,
            denominator,
            Some(quotient),
            None,
            sign,
            None,
        );
    }

    let Some(numerator_interval) = numerator.interval_value.as_ref() else {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::Undecided,
            numerator,
            denominator,
            None,
            None,
            None,
            Some("numerator interval was not available".to_owned()),
        );
    };
    let Some(denominator_interval) = denominator.interval_value.as_ref() else {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::Undecided,
            numerator,
            denominator,
            None,
            None,
            None,
            Some("denominator interval was not available".to_owned()),
        );
    };
    let Some(denominator_sign) = denominator.sign.filter(|sign| *sign != Ordering::Equal) else {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::DenominatorMayContainZero,
            numerator,
            denominator,
            None,
            None,
            None,
            Some(
                "denominator interval contains zero or could not be separated from zero".to_owned(),
            ),
        );
    };
    let Some(denominator_reciprocal) =
        interval_reciprocal_after_certified_nonzero(denominator_interval)
    else {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::Undecided,
            numerator,
            denominator,
            None,
            None,
            None,
            Some("could not invert denominator interval exactly".to_owned()),
        );
    };
    let Some(quotient) = interval_mul(numerator_interval, &denominator_reciprocal, policy) else {
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::Undecided,
            numerator,
            denominator,
            None,
            None,
            None,
            Some("could not multiply numerator and reciprocal intervals exactly".to_owned()),
        );
    };
    let sign = numerator
        .sign
        .map(|numerator_sign| {
            if denominator_sign == Ordering::Less {
                numerator_sign.reverse()
            } else {
                numerator_sign
            }
        })
        .or_else(|| interval_sign(&quotient, policy));
    algebraic_rational_evaluation_report(
        AlgebraicRootRationalEvaluationStatus::IntervalEvaluated,
        numerator,
        denominator,
        None,
        Some(quotient),
        sign,
        None,
    )
}

fn exact_evaluation_value_sign(value: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    if let Some(value) = value.exact_rational_ref() {
        return Some(if value.is_negative() {
            Ordering::Less
        } else if value.is_zero() {
            Ordering::Equal
        } else {
            Ordering::Greater
        });
    }
    compare_reals(value, &Real::zero(), policy).value()
}

fn negate_algebraic_root_representation(
    root: &AlgebraicRootRepresentation,
) -> AlgebraicRootRepresentation {
    // If p(r)=0, then q(x)=p(-x) has root -r. Reflecting the isolating
    // interval avoids a numeric midpoint estimate and preserves the exact
    // evidence object that the exact EGC model requires.
    let polynomial_coefficients = root
        .polynomial_coefficients
        .iter()
        .enumerate()
        .map(|(degree, coefficient)| {
            if degree % 2 == 0 {
                coefficient.clone()
            } else {
                -coefficient.clone()
            }
        })
        .collect::<Vec<_>>();
    let interval = IsolatedRootInterval {
        lower: -root.interval.upper.clone(),
        upper: -root.interval.lower.clone(),
        exact_root: root
            .interval
            .exact_root
            .as_ref()
            .map(|value| -value.clone()),
        distinct_root_count: root.interval.distinct_root_count,
    };
    let representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        validation: AlgebraicRootValidationReport::valid(),
    };
    debug_assert_eq!(
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid,
        "negation of strictly replayed algebraic-root evidence must validate",
    );
    representation
}

fn represent_one_report(
    problem: &Problem,
    report: &UnivariateRootIsolationReport,
    policy: PredicatePolicy,
) -> AlgebraicRootRepresentationReport {
    if report.status == RootIsolationStatus::NoRealRoots {
        return representation_report(
            report.constraint_index,
            report.symbol,
            AlgebraicRootRepresentationStatus::NoRealRoots,
            Vec::new(),
            None,
        );
    }
    if !matches!(
        report.status,
        RootIsolationStatus::Isolated | RootIsolationStatus::MultipleRoot
    ) {
        return representation_report(
            report.constraint_index,
            report.symbol,
            AlgebraicRootRepresentationStatus::UnsupportedIsolationStatus,
            Vec::new(),
            report
                .message
                .clone()
                .or_else(|| Some("isolation report did not contain represented roots".to_owned())),
        );
    }
    let Some(symbol) = report.symbol else {
        return representation_report(
            report.constraint_index,
            None,
            AlgebraicRootRepresentationStatus::MissingSymbol,
            Vec::new(),
            Some("isolation report did not carry a polynomial symbol".to_owned()),
        );
    };
    let Some(constraint) = problem.constraints.get(report.constraint_index) else {
        return representation_report(
            report.constraint_index,
            Some(symbol),
            AlgebraicRootRepresentationStatus::MissingPolynomial,
            Vec::new(),
            Some("constraint index is outside the analysis problem".to_owned()),
        );
    };
    if !constraint.active || constraint.kind != ConstraintKind::Equality {
        return representation_report(
            report.constraint_index,
            Some(symbol),
            AlgebraicRootRepresentationStatus::MissingPolynomial,
            Vec::new(),
            Some("algebraic root representation requires an active equality row".to_owned()),
        );
    }
    let Some(extracted) = collect_univariate_polynomial(&constraint.residual) else {
        return representation_report(
            report.constraint_index,
            Some(symbol),
            AlgebraicRootRepresentationStatus::MissingPolynomial,
            Vec::new(),
            Some("constraint residual is not a supported univariate polynomial".to_owned()),
        );
    };
    if extracted.symbol != Some(symbol) {
        return representation_report(
            report.constraint_index,
            Some(symbol),
            AlgebraicRootRepresentationStatus::MissingPolynomial,
            Vec::new(),
            Some("isolation symbol does not match extracted polynomial symbol".to_owned()),
        );
    }
    let Some(polynomial) = trim_polynomial(extracted.coefficients, policy) else {
        return representation_report(
            report.constraint_index,
            Some(symbol),
            AlgebraicRootRepresentationStatus::MissingPolynomial,
            Vec::new(),
            Some("could not trim polynomial coefficients exactly".to_owned()),
        );
    };

    let mut saw_invalid = false;
    let roots = report
        .intervals
        .iter()
        .enumerate()
        .map(|(interval_index, interval)| {
            let validation = validate_root_payload(&polynomial, interval, policy);
            if validation.status != AlgebraicRootValidationStatus::Valid {
                saw_invalid = true;
            }
            AlgebraicRootRepresentation {
                constraint_index: report.constraint_index,
                symbol,
                interval_index,
                polynomial_coefficients: polynomial.clone(),
                interval: interval.clone(),
                validation,
            }
        })
        .collect::<Vec<_>>();
    representation_report(
        report.constraint_index,
        Some(symbol),
        if saw_invalid {
            AlgebraicRootRepresentationStatus::InvalidEvidence
        } else {
            AlgebraicRootRepresentationStatus::Represented
        },
        roots,
        saw_invalid.then(|| "one or more isolated roots failed validation".to_owned()),
    )
}

fn validate_root_payload(
    polynomial: &[Real],
    interval: &IsolatedRootInterval,
    policy: PredicatePolicy,
) -> AlgebraicRootValidationReport {
    if polynomial.len() <= 1 {
        return AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::InvalidPolynomial,
            "represented algebraic roots require a nonconstant exact polynomial",
        );
    }
    match compare_reals(
        polynomial
            .last()
            .expect("a nonconstant polynomial has a leading coefficient"),
        &Real::zero(),
        PredicatePolicy::STRICT,
    )
    .value()
    {
        Some(Ordering::Less | Ordering::Greater) => {}
        Some(Ordering::Equal) => {
            return AlgebraicRootValidationReport::invalid(
                AlgebraicRootValidationStatus::InvalidPolynomial,
                "represented algebraic roots require a nonzero leading coefficient",
            );
        }
        None => {
            return AlgebraicRootValidationReport::invalid(
                AlgebraicRootValidationStatus::Undecided,
                "represented algebraic root leading coefficient was not certified under STRICT",
            );
        }
    }
    match compare_reals(&interval.lower, &interval.upper, policy).value() {
        Some(Ordering::Greater) => {
            return AlgebraicRootValidationReport::invalid(
                AlgebraicRootValidationStatus::InvalidInterval,
                "isolating interval lower endpoint is greater than upper endpoint",
            );
        }
        Some(Ordering::Less | Ordering::Equal) => {}
        None => {
            return AlgebraicRootValidationReport::invalid(
                AlgebraicRootValidationStatus::Undecided,
                "could not compare isolating interval endpoints",
            );
        }
    }
    if interval.distinct_root_count != 1 {
        return AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::NonUnitIsolation,
            "represented algebraic roots require exactly one distinct root",
        );
    }
    let Some(root) = &interval.exact_root else {
        return AlgebraicRootValidationReport::valid();
    };
    if !point_lies_in_interval(root, &interval.lower, &interval.upper, policy) {
        return AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::WitnessOutsideInterval,
            "exact point witness is outside its isolating interval",
        );
    }
    match compare_reals(
        &evaluate_polynomial(polynomial, root),
        &Real::zero(),
        policy,
    )
    .value()
    {
        Some(Ordering::Equal) => AlgebraicRootValidationReport::valid(),
        Some(Ordering::Less | Ordering::Greater) => AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::WitnessDoesNotSatisfyPolynomial,
            "exact point witness does not satisfy the represented polynomial",
        ),
        None => AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::Undecided,
            "could not replay exact point witness",
        ),
    }
}

fn point_lies_in_interval(
    point: &Real,
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> bool {
    let Some(lower_cmp) = compare_reals(point, lower, policy).value() else {
        return false;
    };
    let Some(upper_cmp) = compare_reals(point, upper, policy).value() else {
        return false;
    };
    lower_cmp != Ordering::Less && upper_cmp != Ordering::Greater
}

#[derive(Clone, Debug)]
struct ExtractedPolynomial {
    symbol: Option<SymbolId>,
    coefficients: Vec<Real>,
}

impl ExtractedPolynomial {
    fn constant(value: Real) -> Self {
        Self {
            symbol: None,
            coefficients: vec![value],
        }
    }

    fn symbol(symbol: SymbolId) -> Self {
        Self {
            symbol: Some(symbol),
            coefficients: vec![Real::zero(), Real::one()],
        }
    }

    fn scale(mut self, scale: Real) -> Self {
        for coefficient in &mut self.coefficients {
            *coefficient = coefficient.clone() * scale.clone();
        }
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let symbol = merge_symbol(self.symbol, other.symbol)?;
        let len = self.coefficients.len().max(other.coefficients.len());
        let mut coefficients = vec![Real::zero(); len];
        for (index, coefficient) in self.coefficients.into_iter().enumerate() {
            coefficients[index] = coefficients[index].clone() + coefficient;
        }
        for (index, coefficient) in other.coefficients.into_iter().enumerate() {
            coefficients[index] = coefficients[index].clone() + coefficient;
        }
        Some(Self {
            symbol,
            coefficients,
        })
    }

    fn multiply(self, other: Self) -> Option<Self> {
        let symbol = merge_symbol(self.symbol, other.symbol)?;
        let mut coefficients =
            vec![Real::zero(); self.coefficients.len() + other.coefficients.len() - 1];
        for (left_index, left) in self.coefficients.into_iter().enumerate() {
            for (right_index, right) in other.coefficients.iter().enumerate() {
                let index = left_index + right_index;
                coefficients[index] = coefficients[index].clone() + left.clone() * right.clone();
            }
        }
        Some(Self {
            symbol,
            coefficients,
        })
    }

    fn powi(self, exponent: i64) -> Option<Self> {
        if exponent < 0 {
            return None;
        }
        let mut result = Self::constant(Real::one());
        for _ in 0..exponent {
            result = result.multiply(self.clone())?;
        }
        Some(result)
    }
}

fn collect_univariate_polynomial(expression: &Expr) -> Option<ExtractedPolynomial> {
    match expression {
        Expr::Constant(value) => Some(ExtractedPolynomial::constant(value.clone())),
        Expr::Symbol(symbol) => Some(ExtractedPolynomial::symbol(symbol.id)),
        Expr::Add(left, right) => {
            collect_univariate_polynomial(left)?.add(collect_univariate_polynomial(right)?)
        }
        Expr::Sub(left, right) => collect_univariate_polynomial(left)?
            .add(collect_univariate_polynomial(right)?.scale(-Real::one())),
        Expr::Neg(value) => Some(collect_univariate_polynomial(value)?.scale(-Real::one())),
        Expr::Mul(left, right) => {
            collect_univariate_polynomial(left)?.multiply(collect_univariate_polynomial(right)?)
        }
        Expr::Div(left, right) => {
            let denominator = constant_value(right)?;
            let reciprocal = (Real::one() / denominator).ok()?;
            Some(collect_univariate_polynomial(left)?.scale(reciprocal))
        }
        Expr::PowI(value, exponent) => collect_univariate_polynomial(value)?.powi(*exponent),
        Expr::Sqrt(_)
        | Expr::Sin(_)
        | Expr::Cos(_)
        | Expr::Ln(_)
        | Expr::Log10(_)
        | Expr::Asin(_)
        | Expr::Acos(_)
        | Expr::Acosh(_)
        | Expr::Atanh(_) => None,
    }
}

fn merge_symbol(left: Option<SymbolId>, right: Option<SymbolId>) -> Option<Option<SymbolId>> {
    match (left, right) {
        (None, None) => Some(None),
        (Some(symbol), None) | (None, Some(symbol)) => Some(Some(symbol)),
        (Some(left), Some(right)) if left == right => Some(Some(left)),
        (Some(_), Some(_)) => None,
    }
}

fn constant_value(expression: &Expr) -> Option<Real> {
    let facts = expression.structural_facts();
    if !facts.dependencies.is_empty() {
        return None;
    }
    expression.eval_real(&HashMap::new()).ok()
}

fn trim_polynomial(mut polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    while polynomial.len() > 1 {
        let trailing = polynomial.last()?;
        match compare_reals(trailing, &Real::zero(), policy).value()? {
            Ordering::Equal => {
                polynomial.pop();
            }
            Ordering::Less | Ordering::Greater => break,
        }
    }
    if polynomial.is_empty() {
        polynomial.push(Real::zero());
    }
    Some(polynomial)
}

fn trim_polynomial_for_evaluation(polynomial: &[Real], policy: PredicatePolicy) -> Option<&[Real]> {
    let mut len = polynomial.len();
    while len > 1 {
        let trailing = &polynomial[len - 1];
        let is_zero = if let Some(trailing) = trailing.exact_rational_ref() {
            trailing.is_zero()
        } else {
            compare_reals(trailing, &Real::zero(), policy).value()? == Ordering::Equal
        };
        if !is_zero {
            break;
        }
        len -= 1;
    }
    (len > 0).then_some(&polynomial[..len])
}

fn evaluate_polynomial(polynomial: &[Real], point: &Real) -> Real {
    if let [constant] = polynomial {
        if constant.exact_rational_ref().is_some() {
            return constant.clone();
        }
        // Preserve the scalar reducer's canonicalization for symbolic
        // constants. Besides proving the `0*x + c` identity, this can expose
        // sign facts that a bare clone deliberately does not recompute.
        return Real::zero() * point.clone() + constant.clone();
    }
    Real::eval_poly(polynomial, point)
}

fn affine_transformed_polynomial(
    polynomial: &[Real],
    scale: &Real,
    offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    if polynomial
        .iter()
        .chain([scale, offset])
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
    {
        return affine_transformed_polynomial_rational_horner(polynomial, scale, offset);
    }
    affine_transformed_polynomial_real_horner(polynomial, scale, offset, policy)
}

fn affine_transformed_polynomial_rational_horner(
    polynomial: &[Real],
    scale: &Real,
    offset: &Real,
) -> Option<Vec<Real>> {
    let scale = scale.exact_rational_ref()?;
    let negative_offset = -offset.exact_rational_ref()?;
    let mut transformed = Vec::with_capacity(polynomial.len());
    transformed.push(polynomial.last()?.exact_rational_ref()?.clone());
    let mut scale_power = Rational::one();
    for coefficient in polynomial[..polynomial.len() - 1].iter().rev() {
        rational_polynomial_mul_monic_linear_in_place(&mut transformed, &negative_offset);
        scale_power = &scale_power * scale;
        let coefficient = coefficient.exact_rational_ref()?;
        transformed[0] = &transformed[0] + coefficient * &scale_power;
    }
    while transformed.len() > 1 && transformed.last().is_some_and(Rational::is_zero) {
        transformed.pop();
    }
    Some(transformed.into_iter().map(Real::from).collect())
}

fn affine_transformed_polynomial_real_horner(
    polynomial: &[Real],
    scale: &Real,
    offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let negative_offset = -offset.clone();
    let mut transformed = Vec::with_capacity(polynomial.len());
    transformed.push(polynomial.last()?.clone());
    let mut scale_power = Real::one();
    for coefficient in polynomial[..polynomial.len() - 1].iter().rev() {
        real_polynomial_mul_monic_linear_in_place(&mut transformed, &negative_offset);
        scale_power *= scale.clone();
        transformed[0] = transformed[0].clone() + coefficient.clone() * scale_power.clone();
    }
    trim_polynomial(transformed, policy)
}

fn rational_polynomial_mul_monic_linear_in_place(
    polynomial: &mut Vec<Rational>,
    constant: &Rational,
) {
    let old_len = polynomial.len();
    debug_assert!(old_len < polynomial.capacity());
    polynomial.push(polynomial[old_len - 1].clone());
    for index in (1..old_len).rev() {
        polynomial[index] = &polynomial[index] * constant + &polynomial[index - 1];
    }
    polynomial[0] = &polynomial[0] * constant;
}

fn real_polynomial_mul_monic_linear_in_place(polynomial: &mut Vec<Real>, constant: &Real) {
    let old_len = polynomial.len();
    debug_assert!(old_len < polynomial.capacity());
    polynomial.push(polynomial[old_len - 1].clone());
    for index in (1..old_len).rev() {
        polynomial[index] =
            polynomial[index].clone() * constant.clone() + polynomial[index - 1].clone();
    }
    polynomial[0] = polynomial[0].clone() * constant.clone();
}

#[cfg(test)]
fn affine_transformed_polynomial_power_sum(
    polynomial: &[Real],
    scale: &Real,
    offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let degree = polynomial.len().checked_sub(1)?;
    let mut transformed = vec![Real::zero(); degree + 1];
    for (power, coefficient) in polynomial.iter().enumerate() {
        let scale_factor = real_pow_nonnegative(scale, degree - power);
        let term_scale = coefficient.clone() * scale_factor;
        for binomial_power in 0..=power {
            let coefficient_index = binomial_power;
            let binomial = Real::from(binomial_coefficient(power, binomial_power) as i64);
            let offset_power = real_pow_nonnegative(&(-offset.clone()), power - binomial_power);
            transformed[coefficient_index] = transformed[coefficient_index].clone()
                + term_scale.clone() * binomial * offset_power;
        }
    }
    trim_polynomial(transformed, policy)
}

fn affine_transformed_interval(
    interval: &IsolatedRootInterval,
    scale: &Real,
    offset: &Real,
    scale_sign: Ordering,
) -> Option<IsolatedRootInterval> {
    let first = eval_affine(&interval.lower, scale, offset);
    let second = eval_affine(&interval.upper, scale, offset);
    let (lower, upper) = match scale_sign {
        Ordering::Less => (second, first),
        Ordering::Greater => (first, second),
        Ordering::Equal => return None,
    };
    Some(IsolatedRootInterval {
        lower,
        upper,
        exact_root: interval
            .exact_root
            .as_ref()
            .map(|root| eval_affine(root, scale, offset)),
        distinct_root_count: interval.distinct_root_count,
    })
}

fn affine_scale_order(scale: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    if let Some(scale) = scale.exact_rational_ref() {
        return Some(if scale.is_negative() {
            Ordering::Less
        } else if scale.is_zero() {
            Ordering::Equal
        } else {
            Ordering::Greater
        });
    }
    compare_reals(scale, &Real::zero(), policy).value()
}

fn eval_affine(value: &Real, scale: &Real, offset: &Real) -> Real {
    if let (Some(value), Some(scale), Some(offset)) = (
        value.exact_rational_ref(),
        scale.exact_rational_ref(),
        offset.exact_rational_ref(),
    ) {
        return Real::from(scale * value + offset);
    }
    scale.clone() * value.clone() + offset.clone()
}

#[cfg(test)]
fn real_pow_nonnegative(value: &Real, exponent: usize) -> Real {
    let mut result = Real::one();
    for _ in 0..exponent {
        result *= value.clone();
    }
    result
}

#[cfg(test)]
fn binomial_coefficient(n: usize, k: usize) -> u64 {
    let k = k.min(n - k);
    let mut result = 1_u64;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
}

fn evaluate_polynomial_interval(
    polynomial: &[Real],
    point: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<AlgebraicPolynomialValueInterval> {
    if let Some(value) = evaluate_rational_polynomial_interval(polynomial, point) {
        return Some(value);
    }
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?;
    let mut value = AlgebraicPolynomialValueInterval {
        lower: leading.clone(),
        upper: leading.clone(),
    };
    for coefficient in coefficients {
        value = interval_add(
            interval_mul(&value, point, policy)?,
            &AlgebraicPolynomialValueInterval {
                lower: coefficient.clone(),
                upper: coefficient.clone(),
            },
        );
    }
    Some(value)
}

fn evaluate_rational_polynomial_interval(
    polynomial: &[Real],
    point: &AlgebraicPolynomialValueInterval,
) -> Option<AlgebraicPolynomialValueInterval> {
    let point_lower = point.lower.exact_rational_ref()?;
    let point_upper = point.upper.exact_rational_ref()?;
    if point_lower > point_upper {
        return None;
    }
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut lower = leading.clone();
    let mut upper = leading.clone();
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        (lower, upper) = rational_interval_product(&lower, &upper, point_lower, point_upper);
        lower = lower + coefficient;
        upper = upper + coefficient;
    }
    Some(AlgebraicPolynomialValueInterval {
        lower: Real::from(lower),
        upper: Real::from(upper),
    })
}

fn interval_add(
    left: AlgebraicPolynomialValueInterval,
    right: &AlgebraicPolynomialValueInterval,
) -> AlgebraicPolynomialValueInterval {
    AlgebraicPolynomialValueInterval {
        lower: left.lower + right.lower.clone(),
        upper: left.upper + right.upper.clone(),
    }
}

fn interval_mul(
    left: &AlgebraicPolynomialValueInterval,
    right: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<AlgebraicPolynomialValueInterval> {
    if let (Some(left_lower), Some(left_upper), Some(right_lower), Some(right_upper)) = (
        left.lower.exact_rational_ref(),
        left.upper.exact_rational_ref(),
        right.lower.exact_rational_ref(),
        right.upper.exact_rational_ref(),
    ) {
        let (lower, upper) =
            rational_interval_product(left_lower, left_upper, right_lower, right_upper);
        return Some(AlgebraicPolynomialValueInterval {
            lower: Real::from(lower),
            upper: Real::from(upper),
        });
    }
    let mut products = [
        left.lower.clone() * right.lower.clone(),
        left.lower.clone() * right.upper.clone(),
        left.upper.clone() * right.lower.clone(),
        left.upper.clone() * right.upper.clone(),
    ];
    sort_reals_exact(&mut products, policy)?;
    Some(AlgebraicPolynomialValueInterval {
        lower: products[0].clone(),
        upper: products[3].clone(),
    })
}

fn interval_reciprocal_after_certified_nonzero(
    value: &AlgebraicPolynomialValueInterval,
) -> Option<AlgebraicPolynomialValueInterval> {
    // Reciprocal is strictly decreasing on each side of zero. The caller has
    // already certified that the whole ordered interval lies on one side, so
    // swapping the endpoint images is exact and needs no second predicate.
    let lower = crate::policy_division::reciprocal_after_certified_nonzero(&value.upper).ok()?;
    let upper = crate::policy_division::reciprocal_after_certified_nonzero(&value.lower).ok()?;
    Some(AlgebraicPolynomialValueInterval { lower, upper })
}

fn interval_sign(
    value: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let (Some(lower), Some(upper)) = (
        value.lower.exact_rational_ref(),
        value.upper.exact_rational_ref(),
    ) {
        return if lower.is_positive() {
            Some(Ordering::Greater)
        } else if upper.is_negative() {
            Some(Ordering::Less)
        } else if lower.is_zero() && upper.is_zero() {
            Some(Ordering::Equal)
        } else {
            None
        };
    }
    let lower = compare_reals(&value.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals(&value.upper, &Real::zero(), policy).value()?;
    if lower == Ordering::Greater {
        Some(Ordering::Greater)
    } else if upper == Ordering::Less {
        Some(Ordering::Less)
    } else if lower == Ordering::Equal && upper == Ordering::Equal {
        Some(Ordering::Equal)
    } else {
        None
    }
}

fn sort_reals_exact(values: &mut [Real], policy: PredicatePolicy) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering = compare_reals(&values[cursor], &values[cursor - 1], policy).value()?;
            if ordering != Ordering::Less {
                break;
            }
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

fn same_represented_root(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> bool {
    left.constraint_index == right.constraint_index
        && left.symbol == right.symbol
        && left.interval_index == right.interval_index
        && left.polynomial_coefficients == right.polynomial_coefficients
        && left.interval == right.interval
}

fn apply_refined_interval(
    root: &mut AlgebraicRootRepresentation,
    refinement: &IsolatedRootRefinementReport,
    policy: PredicatePolicy,
) -> bool {
    if !matches!(
        refinement.status,
        IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
    ) {
        return false;
    }
    let Some(interval) = &refinement.refined_interval else {
        return false;
    };
    root.interval = interval.clone();
    root.validation = validate_algebraic_root_representation(root, policy);
    root.is_valid()
}

/// Certifies the sign of one represented algebraic root from its exact
/// witness, isolating interval, and defining polynomial.
///
/// Cached-valid input is replayed under [`PredicatePolicy::STRICT`] before the
/// requested policy is used for the sign decision. Rational witnesses,
/// endpoints, and constant terms stay in their rational payloads.
///
/// A unit isolator touching zero is resolved from its `(lower, upper]`
/// ownership and constant coefficient, so callers do not need arbitrary
/// refinement merely to prove a strict sign or the selected zero root.
pub fn represented_root_sign(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if !root.is_valid() || !algebraic_root_payload_replays_strictly(root) {
        return None;
    }
    represented_root_sign_admitted(root, policy)
}

pub(crate) fn represented_root_sign_admitted(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let Some(value) = root.exact_point_witness() {
        return algebraic_value_sign(value, policy);
    }
    let upper = algebraic_value_sign(&root.interval.upper, policy)?;
    if upper == Ordering::Less {
        return Some(Ordering::Less);
    }
    let lower = algebraic_value_sign(&root.interval.lower, policy)?;
    if lower == Ordering::Greater {
        return Some(Ordering::Greater);
    }
    // Positive-width isolators exclude the lower endpoint. A polynomial root
    // at lower=0 therefore cannot be the selected root of (0, upper].
    if lower == Ordering::Equal && upper == Ordering::Greater {
        return Some(Ordering::Greater);
    }
    // Zero is now inside the owned interval, if present. The constant term
    // decides whether it is the unique selected root without refinement.
    let zero_is_root = algebraic_value_sign(root.polynomial_coefficients.first()?, policy)?;
    if zero_is_root == Ordering::Equal {
        return Some(Ordering::Equal);
    }
    if upper == Ordering::Equal && lower == Ordering::Less {
        return Some(Ordering::Less);
    }
    None
}

fn algebraic_comparison_report(
    status: AlgebraicRootComparisonStatus,
    ordering: Option<Ordering>,
    message: Option<String>,
) -> AlgebraicRootComparisonReport {
    AlgebraicRootComparisonReport {
        status,
        ordering,
        message,
    }
}

fn algebraic_difference_comparison_report(
    comparison: AlgebraicRootComparisonReport,
    refinement: AlgebraicRootRefinementComparisonReport,
    difference: Option<AlgebraicRootArithmeticReport>,
) -> AlgebraicRootDifferenceComparisonReport {
    AlgebraicRootDifferenceComparisonReport {
        refinement,
        difference,
        comparison,
    }
}

fn algebraic_refinement_comparison_report(
    comparison: AlgebraicRootComparisonReport,
    refined_left: AlgebraicRootRepresentation,
    refined_right: AlgebraicRootRepresentation,
    left_refinements: Vec<IsolatedRootRefinementReport>,
    right_refinements: Vec<IsolatedRootRefinementReport>,
    refinement_rounds: usize,
) -> AlgebraicRootRefinementComparisonReport {
    AlgebraicRootRefinementComparisonReport {
        comparison,
        refined_left,
        refined_right,
        left_refinements,
        right_refinements,
        refinement_rounds,
    }
}

fn algebraic_arithmetic_report(
    operation: AlgebraicRootArithmeticOp,
    status: AlgebraicRootArithmeticStatus,
    exact_result: Option<Real>,
    result_representation: Option<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootArithmeticReport {
    AlgebraicRootArithmeticReport {
        operation,
        status,
        exact_result,
        result_representation,
        message,
    }
}

fn algebraic_affine_transform_report(
    status: AlgebraicRootAffineTransformStatus,
    scale: Real,
    offset: Real,
    representation: Option<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootAffineTransformReport {
    AlgebraicRootAffineTransformReport {
        status,
        scale,
        offset,
        representation,
        message,
    }
}

fn algebraic_polynomial_evaluation_report(
    status: AlgebraicRootPolynomialEvaluationStatus,
    exact_value: Option<Real>,
    interval_value: Option<AlgebraicPolynomialValueInterval>,
    sign: Option<Ordering>,
    message: Option<String>,
) -> AlgebraicRootPolynomialEvaluationReport {
    AlgebraicRootPolynomialEvaluationReport {
        status,
        exact_value,
        interval_value,
        sign,
        message,
    }
}

fn algebraic_rational_evaluation_report(
    status: AlgebraicRootRationalEvaluationStatus,
    numerator: AlgebraicRootPolynomialEvaluationReport,
    denominator: AlgebraicRootPolynomialEvaluationReport,
    exact_value: Option<Real>,
    interval_value: Option<AlgebraicPolynomialValueInterval>,
    sign: Option<Ordering>,
    message: Option<String>,
) -> AlgebraicRootRationalEvaluationReport {
    AlgebraicRootRationalEvaluationReport {
        status,
        numerator,
        denominator,
        exact_value,
        interval_value,
        sign,
        message,
    }
}

fn representation_report(
    constraint_index: usize,
    symbol: Option<SymbolId>,
    status: AlgebraicRootRepresentationStatus,
    roots: Vec<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootRepresentationReport {
    AlgebraicRootRepresentationReport {
        constraint_index,
        symbol,
        status,
        roots,
        message,
    }
}

#[cfg(test)]
mod tests {
    use hyperreal::Rational;
    use num::bigint::{BigInt, BigUint};
    use proptest::prelude::*;

    use super::*;
    use crate::model::Constraint;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn ratio(numerator: i64, denominator: i64) -> Real {
        (real(numerator) / real(denominator)).unwrap()
    }

    fn exact_point_representation(
        constraint_index: usize,
        value: Real,
    ) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-value.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: value.clone(),
                upper: value.clone(),
                exact_root: Some(value),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        }
    }

    fn dyadic(exponent: usize) -> Real {
        Real::new(
            Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(1_u8) << exponent)
                .unwrap(),
        )
    }

    #[test]
    fn represented_roots_preserve_polynomial_and_interval_evidence() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "sqrt two roots",
            x.clone().powi(2) - Expr::int(2),
        ));
        let reports = represent_univariate_algebraic_roots(
            &problem.analyze(),
            RootIsolationConfig::default(),
        );

        assert_eq!(reports.len(), 1);
        assert_eq!(
            reports[0].status,
            AlgebraicRootRepresentationStatus::Represented
        );
        assert_eq!(reports[0].roots.len(), 2);
        assert!(
            reports[0]
                .roots
                .iter()
                .all(AlgebraicRootRepresentation::is_valid)
        );
        assert!(
            reports[0]
                .roots
                .iter()
                .all(|root| root.exact_point_witness().is_none())
        );
        assert_eq!(
            reports[0].roots[0].polynomial_coefficients,
            vec![real(-2), Real::zero(), Real::one()]
        );
    }

    #[test]
    fn exact_rational_witnesses_replay_against_polynomial() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "two rational roots",
            x.clone().powi(2) - Expr::int(1),
        ));
        let reports = represent_univariate_algebraic_roots(
            &problem.analyze(),
            RootIsolationConfig {
                max_interval_width: Some(Real::one()),
                max_refinement_steps: 8,
                ..RootIsolationConfig::default()
            },
        );

        assert_eq!(
            reports[0].status,
            AlgebraicRootRepresentationStatus::Represented
        );
        assert!(
            reports[0]
                .roots
                .iter()
                .any(|root| root.exact_point_witness() == Some(&real(1)))
        );
        assert!(reports[0].roots.iter().all(|root| {
            validate_algebraic_root_representation(root, PredicatePolicy::APPROXIMATE_512).status
                == AlgebraicRootValidationStatus::Valid
        }));
    }

    #[test]
    fn invalid_representations_are_rejected_antagonistically() {
        let invalid_count = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-1), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 2,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        assert_eq!(
            validate_algebraic_root_representation(
                &invalid_count,
                PredicatePolicy::APPROXIMATE_512
            )
            .status,
            AlgebraicRootValidationStatus::NonUnitIsolation
        );

        let bad_witness = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: real(0),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            ..invalid_count
        };
        assert_eq!(
            validate_algebraic_root_representation(&bad_witness, PredicatePolicy::APPROXIMATE_512)
                .status,
            AlgebraicRootValidationStatus::WitnessDoesNotSatisfyPolynomial
        );
    }

    #[test]
    fn represented_roots_accept_exact_real_coefficient_fields() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-sqrt_two, Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        assert_eq!(
            validate_algebraic_root_representation(&root, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid
        );
    }

    #[test]
    fn algebraic_root_comparison_orders_disjoint_intervals_and_witnesses() {
        let left = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: real(-1),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let right = AlgebraicRootRepresentation {
            constraint_index: 0,
            interval_index: 1,
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..left.clone()
        };

        let comparison =
            compare_algebraic_root_representations(&left, &right, PredicatePolicy::APPROXIMATE_512);
        assert_eq!(comparison.status, AlgebraicRootComparisonStatus::Compared);
        assert_eq!(comparison.ordering, Some(Ordering::Less));

        let same =
            compare_algebraic_root_representations(&left, &left, PredicatePolicy::APPROXIMATE_512);
        assert_eq!(
            same.status,
            AlgebraicRootComparisonStatus::SameRepresentation
        );
        assert_eq!(same.ordering, Some(Ordering::Equal));
    }

    #[test]
    fn algebraic_root_comparison_reports_overlap_and_invalid_evidence() {
        let valid = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(0),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let mut overlapping = valid.clone();
        overlapping.constraint_index = 1;
        overlapping.interval_index = 1;
        overlapping.interval = IsolatedRootInterval {
            lower: real(1),
            upper: real(3),
            exact_root: None,
            distinct_root_count: 1,
        };

        let comparison = compare_algebraic_root_representations(
            &valid,
            &overlapping,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            comparison.status,
            AlgebraicRootComparisonStatus::OverlappingIntervals
        );
        assert_eq!(comparison.ordering, None);

        let mut invalid = valid.clone();
        invalid.validation = AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::InvalidPolynomial,
            "test invalid",
        );
        assert_eq!(
            compare_algebraic_root_representations(
                &invalid,
                &valid,
                PredicatePolicy::APPROXIMATE_512
            )
            .status,
            AlgebraicRootComparisonStatus::InvalidEvidence
        );
    }

    #[test]
    fn algebraic_root_comparison_and_sign_replay_cached_valid_payloads() {
        let mut stale = exact_point_representation(0, real(2));
        stale.interval.lower = real(3);
        stale.interval.upper = real(3);
        stale.interval.exact_root = Some(real(3));
        let valid = exact_point_representation(1, real(4));

        assert!(stale.is_valid());
        assert_eq!(
            validate_algebraic_root_representation(&stale, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::WitnessDoesNotSatisfyPolynomial
        );
        assert_eq!(
            compare_algebraic_root_representations(&stale, &stale, PredicatePolicy::STRICT).status,
            AlgebraicRootComparisonStatus::InvalidEvidence
        );

        let refined = compare_algebraic_root_representations_with_refinement(
            &stale,
            &valid,
            AlgebraicRootRefinementComparisonConfig::default(),
        );
        assert_eq!(
            refined.comparison.status,
            AlgebraicRootComparisonStatus::InvalidEvidence
        );
        assert_eq!(refined.refinement_rounds, 0);
        assert!(refined.left_refinements.is_empty());
        assert!(refined.right_refinements.is_empty());

        let difference = compare_algebraic_root_representations_by_difference(
            &stale,
            &valid,
            AlgebraicRootRefinementComparisonConfig::default(),
        );
        assert_eq!(
            difference.comparison.status,
            AlgebraicRootComparisonStatus::InvalidEvidence
        );
        assert!(difference.difference.is_none());
        assert_eq!(represented_root_sign(&stale, PredicatePolicy::STRICT), None);
        assert_eq!(
            translated_algebraic_root_difference(&stale, &valid, PredicatePolicy::STRICT),
            None
        );
        assert_eq!(
            algebraic_root_affine_relation(&stale, &valid, PredicatePolicy::STRICT),
            None
        );
    }

    #[test]
    fn algebraic_root_comparison_uses_exact_witness_as_effective_interval() {
        let mut exact_three = exact_point_representation(0, real(3));
        exact_three.interval.lower = Real::zero();
        exact_three.interval.upper = real(4);
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let direct = compare_algebraic_root_representations(
            &exact_three,
            &sqrt_two,
            PredicatePolicy::STRICT,
        );
        assert_eq!(direct.status, AlgebraicRootComparisonStatus::Compared);
        assert_eq!(direct.ordering, Some(Ordering::Greater));

        let refined = compare_algebraic_root_representations_with_refinement(
            &exact_three,
            &sqrt_two,
            AlgebraicRootRefinementComparisonConfig::default(),
        );
        assert_eq!(refined.comparison.ordering, Some(Ordering::Greater));
        assert_eq!(refined.refinement_rounds, 0);
        assert!(refined.left_refinements.is_empty());
        assert!(refined.right_refinements.is_empty());
    }

    #[test]
    fn algebraic_root_refinement_skips_exact_point_operand() {
        let sqrt_two_value = real(2).sqrt().expect("positive exact square root");
        let sqrt_two = exact_point_representation(0, sqrt_two_value);
        let sqrt_three = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        assert_eq!(
            compare_algebraic_root_representations(
                &sqrt_two,
                &sqrt_three,
                PredicatePolicy::STRICT,
            )
            .status,
            AlgebraicRootComparisonStatus::OverlappingIntervals
        );

        let refined = compare_algebraic_root_representations_with_refinement(
            &sqrt_two,
            &sqrt_three,
            AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                max_refinement_rounds: 4,
                steps_per_round: 1,
            },
        );
        assert_eq!(
            refined.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(refined.comparison.ordering, Some(Ordering::Less));
        assert_eq!(refined.refinement_rounds, 1);
        assert!(refined.left_refinements.is_empty());
        assert_eq!(refined.right_refinements.len(), 1);
    }

    #[test]
    fn algebraic_root_refinement_supports_exact_real_coefficient_fields() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let exact_real_coefficients = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-sqrt_two, Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: ratio(3, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let rational_coefficients = AlgebraicRootRepresentation {
            constraint_index: 1,
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: ratio(7, 5),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..exact_real_coefficients.clone()
        };

        let report = compare_algebraic_root_representations_with_refinement(
            &exact_real_coefficients,
            &rational_coefficients,
            AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                max_refinement_rounds: 2,
                steps_per_round: 1,
            },
        );

        assert_eq!(
            report.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(report.comparison.ordering, Some(Ordering::Less));
        assert_eq!(report.refinement_rounds, 1);
        assert!(matches!(
            report.left_refinements[0].status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));
        assert!(matches!(
            report.right_refinements[0].status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));
        assert!(report.refined_left.interval.upper < report.refined_right.interval.lower);
    }

    #[test]
    fn algebraic_root_refinement_comparison_orders_overlapping_intervals() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: ratio(7, 5),
                upper: ratio(3, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let sqrt_three = AlgebraicRootRepresentation {
            constraint_index: 1,
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two.clone()
        };

        assert_eq!(
            compare_algebraic_root_representations(
                &sqrt_two,
                &sqrt_three,
                PredicatePolicy::APPROXIMATE_512
            )
            .status,
            AlgebraicRootComparisonStatus::OverlappingIntervals
        );

        let refined = compare_algebraic_root_representations_with_refinement(
            &sqrt_two,
            &sqrt_three,
            AlgebraicRootRefinementComparisonConfig {
                max_refinement_rounds: 8,
                steps_per_round: 1,
                ..AlgebraicRootRefinementComparisonConfig::default()
            },
        );

        assert_eq!(
            refined.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(refined.comparison.ordering, Some(Ordering::Less));
        assert!(!refined.left_refinements.is_empty());
        assert!(!refined.right_refinements.is_empty());
        assert!(refined.refined_left.interval.upper < refined.refined_right.interval.lower);
    }

    #[test]
    fn algebraic_root_difference_comparison_orders_constructed_overlap() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: ratio(7, 5),
                upper: ratio(3, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let sqrt_three = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(1),
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: ratio(29, 20),
                upper: ratio(9, 5),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two.clone()
        };

        let report = compare_algebraic_root_representations_by_difference(
            &sqrt_two,
            &sqrt_three,
            AlgebraicRootRefinementComparisonConfig {
                max_refinement_rounds: 0,
                ..AlgebraicRootRefinementComparisonConfig::default()
            },
        );

        assert_eq!(
            report.refinement.comparison.status,
            AlgebraicRootComparisonStatus::OverlappingIntervals
        );
        assert_eq!(
            report.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(report.comparison.ordering, Some(Ordering::Less));
        let difference = report.difference.as_ref().unwrap();
        assert_eq!(
            difference.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let difference_root = difference.result_representation.as_ref().unwrap();
        assert!(difference_root.interval.upper < Real::zero());
    }

    #[test]
    fn algebraic_root_difference_comparison_returns_direct_orders_without_construction() {
        let left = exact_point_representation(0, real(2));
        let right = exact_point_representation(1, real(3));

        let report = compare_algebraic_root_representations_by_difference(
            &left,
            &right,
            AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                max_refinement_rounds: 0,
                steps_per_round: 1,
            },
        );

        assert_eq!(
            report.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(report.comparison.ordering, Some(Ordering::Less));
        assert_eq!(report.refinement.refinement_rounds, 0);
        assert!(report.refinement.left_refinements.is_empty());
        assert!(report.refinement.right_refinements.is_empty());
        assert!(report.difference.is_none());

        let same = compare_algebraic_root_representations_by_difference(
            &left,
            &left,
            AlgebraicRootRefinementComparisonConfig::default(),
        );
        assert_eq!(
            same.comparison.status,
            AlgebraicRootComparisonStatus::SameRepresentation
        );
        assert_eq!(same.comparison.ordering, Some(Ordering::Equal));
        assert_eq!(same.refinement.refinement_rounds, 0);
        assert!(same.difference.is_none());
    }

    #[test]
    fn algebraic_root_comparison_recognizes_equal_irrational_singletons() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let cached = exact_point_representation(0, sqrt_two.clone());
        let uncached = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: sqrt_two.clone(),
                upper: sqrt_two,
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let comparison =
            compare_algebraic_root_representations(&uncached, &cached, PredicatePolicy::STRICT);
        assert_eq!(comparison.status, AlgebraicRootComparisonStatus::Compared);
        assert_eq!(comparison.ordering, Some(Ordering::Equal));
    }

    #[test]
    fn algebraic_root_difference_comparison_matches_exact_point_to_isolator() {
        let exact_two = exact_point_representation(0, real(2));
        let interval_two = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(6), real(-5), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: ratio(5, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let report = compare_algebraic_root_representations_by_difference(
            &exact_two,
            &interval_two,
            AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                max_refinement_rounds: 0,
                steps_per_round: 1,
            },
        );

        assert_eq!(
            report.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(report.comparison.ordering, Some(Ordering::Equal));
        assert!(
            report
                .comparison
                .message
                .as_deref()
                .is_some_and(|message| message.contains("shared polynomial root"))
        );
        assert_eq!(report.refinement.refinement_rounds, 0);
        assert!(report.difference.is_none());

        let exact_sqrt_two =
            exact_point_representation(2, real(2).sqrt().expect("positive exact square root"));
        let sqrt_two_interval = AlgebraicRootRepresentation {
            constraint_index: 3,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..interval_two
        };
        let exact_real_report = compare_algebraic_root_representations_by_difference(
            &exact_sqrt_two,
            &sqrt_two_interval,
            AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                max_refinement_rounds: 0,
                steps_per_round: 1,
            },
        );
        assert_eq!(exact_real_report.comparison.ordering, Some(Ordering::Equal));
        assert!(exact_real_report.difference.is_none());
    }

    #[test]
    fn algebraic_root_difference_comparison_certifies_shared_polynomial_root() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: ratio(7, 5),
                upper: ratio(3, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let same_root_with_extra_factor = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(1),
            polynomial_coefficients: vec![real(-6), real(-2), real(3), Real::one()],
            ..sqrt_two.clone()
        };

        let report = compare_algebraic_root_representations_by_difference(
            &sqrt_two,
            &same_root_with_extra_factor,
            AlgebraicRootRefinementComparisonConfig::default(),
        );

        assert_eq!(
            report.comparison.status,
            AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(report.comparison.ordering, Some(Ordering::Equal));
        assert_eq!(report.refinement.refinement_rounds, 0);
        assert!(report.difference.is_none());
    }

    #[test]
    fn algebraic_root_difference_certifies_exact_translation_beyond_512_bits() {
        let epsilon = dyadic(600);
        let half = ratio(1, 2);
        let left = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-half.clone(), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: half.clone(),
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let right = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(1),
            polynomial_coefficients: vec![
                &epsilon * &epsilon - half,
                -(&epsilon * Real::from(2_i8)),
                Real::one(),
            ],
            ..left.clone()
        };

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            assert_eq!(
                translated_algebraic_root_difference(&left, &right, policy),
                Some(-epsilon.clone())
            );
            assert_eq!(
                translated_algebraic_root_difference(&right, &left, policy),
                Some(epsilon.clone())
            );
            assert_eq!(
                algebraic_root_affine_relation(&left, &right, policy),
                Some(AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: epsilon.clone(),
                })
            );
            let report = compare_algebraic_root_representations_by_difference(
                &left,
                &right,
                AlgebraicRootRefinementComparisonConfig {
                    policy,
                    max_refinement_rounds: 0,
                    steps_per_round: 1,
                },
            );
            assert_eq!(
                report.comparison.status,
                AlgebraicRootComparisonStatus::Compared
            );
            assert_eq!(report.comparison.ordering, Some(Ordering::Less));
            let difference = report.difference.unwrap();
            assert_eq!(
                difference.status,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            );
            assert_eq!(difference.exact_result, Some(-epsilon.clone()));
        }
    }

    #[test]
    fn algebraic_root_affine_relation_certifies_scale_and_offset_beyond_512_bits() {
        let epsilon = dyadic(600);
        let half = ratio(1, 2);
        let scale = ratio(1, 2);
        let left = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-half.clone(), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: half,
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let right = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(1),
            interval_index: 0,
            polynomial_coefficients: vec![
                &epsilon * &epsilon - ratio(1, 8),
                -(&epsilon * Real::from(2_i8)),
                Real::one(),
            ],
            interval: IsolatedRootInterval {
                lower: ratio(1, 4) + epsilon.clone(),
                upper: ratio(1, 2) + epsilon.clone(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            assert_eq!(
                algebraic_root_affine_relation(&left, &right, policy),
                Some(AlgebraicRootAffineRelation {
                    scale: scale.clone(),
                    offset: epsilon.clone(),
                })
            );
            assert_eq!(
                algebraic_root_affine_relation(&right, &left, policy),
                Some(AlgebraicRootAffineRelation {
                    scale: Real::from(2_i8),
                    offset: -(Real::from(2_i8) * epsilon.clone()),
                })
            );
        }
    }

    #[test]
    fn represented_root_sign_uses_exact_zero_endpoint_evidence() {
        let negative_sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: Real::zero(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let positive_sqrt_two = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..negative_sqrt_two.clone()
        };
        let zero = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-1),
                upper: real(1),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..negative_sqrt_two.clone()
        };
        let crossing_nonzero = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-1), real(1), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-1),
                upper: real(1),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..negative_sqrt_two.clone()
        };

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            assert_eq!(
                represented_root_sign(&negative_sqrt_two, policy),
                Some(Ordering::Less)
            );
            assert_eq!(
                represented_root_sign(&positive_sqrt_two, policy),
                Some(Ordering::Greater)
            );
            assert_eq!(represented_root_sign(&zero, policy), Some(Ordering::Equal));
            assert_eq!(represented_root_sign(&crossing_nonzero, policy), None);
        }
    }

    #[test]
    fn represented_root_sign_respects_zero_endpoint_ownership() {
        // x(x-1): the source owns 1 in (0, 2], despite vanishing at 0.
        let positive = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![Real::zero(), real(-1), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        // x(x+1): the source owns 0 in (-1, 0], excluding -1.
        let zero = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![Real::zero(), Real::one(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(-1),
                upper: Real::zero(),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..positive.clone()
        };
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            assert_eq!(
                represented_root_sign(&positive, policy),
                Some(Ordering::Greater)
            );
            assert_eq!(represented_root_sign(&zero, policy), Some(Ordering::Equal));
        }
    }

    #[test]
    fn algebraic_root_arithmetic_uses_the_explicit_policy() {
        let left = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(2),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let right = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            ..left.clone()
        };

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = arithmetic_algebraic_root_representations(
                &left,
                Some(&right),
                AlgebraicRootArithmeticOp::Add,
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            );
            assert_eq!(report.exact_result, Some(real(5)));
        }
    }

    #[test]
    fn algebraic_root_arithmetic_uses_exact_rational_witnesses_only() {
        let left = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(2),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let right = AlgebraicRootRepresentation {
            constraint_index: 1,
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            ..left.clone()
        };

        let sum = arithmetic_algebraic_root_representations(
            &left,
            Some(&right),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            sum.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(sum.exact_result, Some(real(5)));

        let product = arithmetic_algebraic_root_representations(
            &left,
            Some(&right),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(product.exact_result, Some(real(6)));

        let quotient = arithmetic_algebraic_root_representations(
            &right,
            Some(&left),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(quotient.exact_result, Some((real(3) / real(2)).unwrap()));

        let negation = arithmetic_algebraic_root_representations(
            &left,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(negation.exact_result, Some(real(-2)));
    }

    #[test]
    fn algebraic_root_arithmetic_replays_cached_input_validation_strictly() {
        let mut stale = exact_point_representation(0, real(2));
        stale.polynomial_coefficients[0] = real(-3);
        assert!(stale.is_valid());

        let negation = arithmetic_algebraic_root_representations(
            &stale,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            negation.status,
            AlgebraicRootArithmeticStatus::InvalidEvidence
        );
        assert!(negation.exact_result.is_none());
        assert!(negation.result_representation.is_none());

        let valid = exact_point_representation(1, real(5));
        let sum = arithmetic_algebraic_root_representations(
            &valid,
            Some(&stale),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(sum.status, AlgebraicRootArithmeticStatus::InvalidEvidence);

        let mut stale_interval = AlgebraicRootRepresentation {
            constraint_index: 2,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        stale_interval.interval.lower = real(3);
        let zero = exact_point_representation(3, Real::zero());
        let zero_product = arithmetic_algebraic_root_representations(
            &stale_interval,
            Some(&zero),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            zero_product.status,
            AlgebraicRootArithmeticStatus::InvalidEvidence
        );
        let identity = arithmetic_algebraic_root_representations(
            &stale_interval,
            Some(&stale_interval),
            AlgebraicRootArithmeticOp::Subtract,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            identity.status,
            AlgebraicRootArithmeticStatus::InvalidEvidence
        );

        let mut noncanonical = exact_point_representation(4, real(2));
        noncanonical.polynomial_coefficients = vec![real(-4), real(2)];
        let accepted = arithmetic_algebraic_root_representations(
            &noncanonical,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            accepted.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(accepted.exact_result, Some(real(-2)));
    }

    #[test]
    fn algebraic_root_arithmetic_rejects_missing_binary_operand_before_routing() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let report = arithmetic_algebraic_root_representations(
            &sqrt_two,
            None,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicRootArithmeticStatus::InvalidEvidence
        );
        assert!(report.message.as_deref().unwrap().contains("right input"));
    }

    #[test]
    fn algebraic_root_arithmetic_returns_exact_nonrational_real_witnesses() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let pi_value = Real::pi();
        let sqrt_two = exact_point_representation(0, sqrt_two_value.clone());
        let pi = exact_point_representation(1, pi_value.clone());
        let expected = sqrt_two_value + pi_value;
        assert!(expected.exact_rational_ref().is_none());

        let sum = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&pi),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            sum.status,
            AlgebraicRootArithmeticStatus::ComputedExactRealWitness
        );
        assert_eq!(sum.exact_result, Some(expected));
        assert!(sum.result_representation.is_none());

        let negation = arithmetic_algebraic_root_representations(
            &sqrt_two,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            negation.status,
            AlgebraicRootArithmeticStatus::ComputedExactRealWitness
        );
        assert_eq!(negation.exact_result, Some(-real(2).sqrt().unwrap()));
        assert!(negation.result_representation.is_none());
    }

    #[test]
    fn algebraic_root_arithmetic_divides_by_policy_certified_exact_point() {
        let denominator_value = crate::test_support::exact_normal_positive();
        let numerator = exact_point_representation(0, Real::one());
        let denominator = exact_point_representation(1, denominator_value.clone());

        let quotient = arithmetic_algebraic_root_representations(
            &numerator,
            Some(&denominator),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            quotient.status,
            AlgebraicRootArithmeticStatus::ComputedExactRealWitness
        );
        assert!(quotient.result_representation.is_none());
        let value = quotient.exact_result.as_ref().expect("exact Real quotient");
        assert_eq!(
            compare_reals(
                &(value.clone() * denominator_value),
                &Real::one(),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn algebraic_root_arithmetic_certifies_zero_dividend_and_touching_nonzero_divisor() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let zero = exact_point_representation(1, Real::zero());
        let quotient = arithmetic_algebraic_root_representations(
            &zero,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            quotient.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(quotient.exact_result, Some(Real::zero()));

        let touching_zero_one = AlgebraicRootRepresentation {
            constraint_index: 2,
            polynomial_coefficients: vec![real(-1), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two
        };
        let identity = arithmetic_algebraic_root_representations(
            &touching_zero_one,
            Some(&touching_zero_one),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            identity.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(identity.exact_result, Some(Real::one()));
    }

    #[test]
    fn algebraic_root_arithmetic_reduces_same_quadratic_square_to_affine_relation() {
        let root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-1), Real::one(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let square = arithmetic_algebraic_root_representations(
            &root,
            Some(&root),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::STRICT,
        );
        let expected = transform_algebraic_root_affine(
            &root,
            -Real::one(),
            Real::one(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            square.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        assert_eq!(square.result_representation, expected.representation);
    }

    #[test]
    fn algebraic_root_arithmetic_supports_exact_real_scalars_and_reports_unsupported_fields() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let pi_value = Real::pi();
        let pi = exact_point_representation(1, pi_value.clone());
        let arithmetic = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&pi),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        let affine = transform_algebraic_root_affine(
            &sqrt_two,
            Real::one(),
            pi_value,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            arithmetic.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        assert_eq!(arithmetic.result_representation, affine.representation);

        let sqrt_two_value = real(2).sqrt().unwrap();
        let exact_real_left = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![-sqrt_two_value, Real::one()],
            ..sqrt_two.clone()
        };
        let exact_real_right = AlgebraicRootRepresentation {
            constraint_index: 2,
            polynomial_coefficients: vec![-Real::pi(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(4),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two
        };
        let unsupported = arithmetic_algebraic_root_representations(
            &exact_real_left,
            Some(&exact_real_right),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            unsupported.status,
            AlgebraicRootArithmeticStatus::NonRationalInput
        );
    }

    #[test]
    fn algebraic_root_arithmetic_rejects_interval_only_and_invalid_inputs() {
        let interval_only = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let report = arithmetic_algebraic_root_representations(
            &interval_only,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            report.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let negated = report.result_representation.as_ref().unwrap();
        assert_eq!(
            negated.polynomial_coefficients,
            vec![real(-2), Real::zero(), Real::one()]
        );
        assert_eq!(negated.interval.lower, real(-2));
        assert_eq!(negated.interval.upper, real(-1));
        assert!(negated.exact_point_witness().is_none());

        let mut invalid = interval_only;
        invalid.validation = AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::InvalidPolynomial,
            "test invalid",
        );
        let invalid_report = arithmetic_algebraic_root_representations(
            &invalid,
            None,
            AlgebraicRootArithmeticOp::Negate,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            invalid_report.status,
            AlgebraicRootArithmeticStatus::InvalidEvidence
        );
    }

    #[test]
    fn algebraic_root_arithmetic_transforms_interval_only_roots_by_rational_scalars() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let rational_three = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let rational_two = AlgebraicRootRepresentation {
            constraint_index: 2,
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(2),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            ..rational_three.clone()
        };
        let rational_zero = AlgebraicRootRepresentation {
            constraint_index: 3,
            polynomial_coefficients: vec![Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: Real::zero(),
                exact_root: Some(Real::zero()),
                distinct_root_count: 1,
            },
            ..rational_three.clone()
        };

        let sum = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&rational_three),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            sum.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let sum_root = sum.result_representation.as_ref().unwrap();
        assert_eq!(
            sum_root.polynomial_coefficients,
            vec![real(7), real(-6), Real::one()]
        );
        assert_eq!(sum_root.interval.lower, real(4));
        assert_eq!(sum_root.interval.upper, real(5));

        let difference = arithmetic_algebraic_root_representations(
            &rational_three,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Subtract,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            difference.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let difference_root = difference.result_representation.as_ref().unwrap();
        assert_eq!(
            difference_root.polynomial_coefficients,
            vec![real(7), real(-6), Real::one()]
        );
        assert_eq!(difference_root.interval.lower, real(1));
        assert_eq!(difference_root.interval.upper, real(2));

        let product = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&rational_two),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            product.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let product_root = product.result_representation.as_ref().unwrap();
        assert_eq!(
            product_root.polynomial_coefficients,
            vec![real(-8), Real::zero(), Real::one()]
        );
        assert_eq!(product_root.interval.lower, real(2));
        assert_eq!(product_root.interval.upper, real(4));

        let zero_product = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&rational_zero),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            zero_product.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(zero_product.exact_result, Some(Real::zero()));

        let divided_by_scalar = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&rational_two),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            divided_by_scalar.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let divided_root = divided_by_scalar.result_representation.as_ref().unwrap();
        assert_eq!(
            divided_root.polynomial_coefficients,
            vec![(real(-1) / real(2)).unwrap(), Real::zero(), Real::one()]
        );
        assert_eq!(divided_root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(divided_root.interval.upper, Real::one());

        let scalar_divided_by_root = arithmetic_algebraic_root_representations(
            &rational_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            scalar_divided_by_root.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let reciprocal_root = scalar_divided_by_root
            .result_representation
            .as_ref()
            .unwrap();
        assert_eq!(
            reciprocal_root.polynomial_coefficients,
            vec![real(4), Real::zero(), real(-2)]
        );
        assert_eq!(reciprocal_root.interval.lower, Real::one());
        assert_eq!(reciprocal_root.interval.upper, real(2));

        let same_sum = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            same_sum.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let same_sum_root = same_sum.result_representation.as_ref().unwrap();
        assert_eq!(
            same_sum_root.polynomial_coefficients,
            vec![real(-8), Real::zero(), Real::one()]
        );
        assert_eq!(same_sum_root.interval.lower, real(2));
        assert_eq!(same_sum_root.interval.upper, real(4));

        let same_product = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            same_product.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
        );
        assert_eq!(same_product.exact_result, Some(real(2)));
        assert!(same_product.result_representation.is_none());

        let same_difference = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Subtract,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(same_difference.exact_result, Some(Real::zero()));

        let same_quotient = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(same_quotient.exact_result, Some(Real::one()));

        let sqrt_three = AlgebraicRootRepresentation {
            constraint_index: 4,
            symbol: SymbolId(1),
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::one()],
            ..sqrt_two.clone()
        };
        let independent_sum = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_three),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            independent_sum.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        assert_eq!(
            independent_sum
                .result_representation
                .as_ref()
                .unwrap()
                .polynomial_coefficients,
            vec![
                Real::one(),
                Real::zero(),
                real(-10),
                Real::zero(),
                Real::one()
            ]
        );

        let independent_product = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_three),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            independent_product.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );

        let independent_quotient = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_three),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            independent_quotient.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
    }

    #[test]
    fn algebraic_root_polynomial_evaluation_certifies_witnesses_and_intervals() {
        let rational_root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let rational = evaluate_polynomial_at_algebraic_root(
            &rational_root,
            &[real(-9), Real::zero(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            rational.status,
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(rational.exact_value, Some(Real::zero()));
        assert_eq!(rational.sign, Some(Ordering::Equal));

        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let positive = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            positive.status,
            AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
        );
        assert_eq!(positive.sign, Some(Ordering::Greater));

        let crossing = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[real(-2), Real::zero(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            crossing.status,
            AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero
        );
        assert_eq!(crossing.sign, None);
    }

    #[test]
    fn algebraic_root_evaluation_distinguishes_exact_real_values_and_replays_input() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let sqrt_two = exact_point_representation(0, sqrt_two_value.clone());
        assert!(
            sqrt_two
                .exact_point_witness()
                .unwrap()
                .exact_rational_ref()
                .is_none()
        );

        let exact_real = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[Real::one(), Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            exact_real.status,
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        );
        assert_eq!(
            compare_reals(
                exact_real.exact_value.as_ref().unwrap(),
                &(Real::one() + &sqrt_two_value),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(exact_real.sign, Some(Ordering::Greater));

        let defining_zero = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[real(-2), Real::zero(), Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            defining_zero.status,
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(defining_zero.exact_value, Some(Real::zero()));

        let rational_point = exact_point_representation(1, real(3));
        let exact_constant = evaluate_polynomial_at_algebraic_root(
            &rational_point,
            &[Real::pi()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            exact_constant.status,
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        );

        let quotient = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one(), Real::one()],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            quotient.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness
        );
        assert_eq!(
            compare_reals(
                quotient.exact_value.as_ref().unwrap(),
                exact_real.exact_value.as_ref().unwrap(),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );

        let cancellation = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::zero(), Real::one()],
            &[Real::zero(), Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            cancellation.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(cancellation.exact_value, Some(Real::one()));

        let mut stale = sqrt_two;
        stale.polynomial_coefficients[0] = -Real::pi();
        assert!(stale.is_valid());
        let rejected = evaluate_polynomial_at_algebraic_root(
            &stale,
            &[Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            rejected.status,
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
        );
        assert!(rejected.exact_value.is_none());

        let rejected = evaluate_rational_expression_at_algebraic_root(
            &stale,
            &[Real::one()],
            &[Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            rejected.status,
            AlgebraicRootRationalEvaluationStatus::InvalidEvidence
        );
        assert_eq!(
            rejected.numerator.status,
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
        );
        assert_eq!(
            rejected.denominator.status,
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence
        );
    }

    #[test]
    fn algebraic_root_rational_evaluation_checks_denominator_domain() {
        let rational_root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let exact = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[real(1), Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            exact.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(exact.exact_value, Some(real(2)));
        assert_eq!(exact.sign, Some(Ordering::Greater));

        let denominator = crate::test_support::exact_normal_positive();
        let half = ratio(1, 2);
        let numerator = denominator.clone() * &half;
        let policy_exact = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            core::slice::from_ref(&numerator),
            core::slice::from_ref(&denominator),
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            policy_exact.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness
        );
        assert_eq!(
            policy_exact
                .exact_value
                .as_ref()
                .and_then(Real::exact_rational_normal_form),
            half.exact_rational()
        );

        let zero_over_policy_nonzero = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[Real::zero()],
            core::slice::from_ref(&denominator),
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            zero_over_policy_nonzero.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(zero_over_policy_nonzero.exact_value, Some(Real::zero()));
        assert_eq!(zero_over_policy_nonzero.sign, Some(Ordering::Equal));

        let unresolved = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[Real::one()],
            &[crate::test_support::terminal_zero()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            unresolved.status,
            AlgebraicRootRationalEvaluationStatus::Undecided
        );

        let zero_denominator = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[Real::one()],
            &[real(-3), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            zero_denominator.status,
            AlgebraicRootRationalEvaluationStatus::CertifiedZeroDenominator
        );

        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let policy_interval = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            core::slice::from_ref(&numerator),
            core::slice::from_ref(&denominator),
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            policy_interval.status,
            AlgebraicRootRationalEvaluationStatus::IntervalEvaluated
        );
        let policy_interval = policy_interval
            .interval_value
            .expect("constant quotient interval");
        assert_eq!(
            policy_interval.lower.exact_rational_normal_form(),
            half.exact_rational()
        );
        assert_eq!(
            policy_interval.upper.exact_rational_normal_form(),
            half.exact_rational()
        );

        let interval = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one()],
            &[Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            interval.status,
            AlgebraicRootRationalEvaluationStatus::IntervalEvaluated
        );
        assert_eq!(interval.sign, Some(Ordering::Greater));
        assert!(interval.interval_value.is_some());

        let negative_denominator = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one()],
            &[real(-4), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            negative_denominator.status,
            AlgebraicRootRationalEvaluationStatus::IntervalEvaluated
        );
        assert_eq!(negative_denominator.sign, Some(Ordering::Less));
        assert_eq!(
            negative_denominator.interval_value,
            Some(AlgebraicPolynomialValueInterval {
                lower: ratio(-1, 2),
                upper: ratio(-1, 3),
            })
        );

        let may_contain_zero = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            may_contain_zero.status,
            AlgebraicRootRationalEvaluationStatus::DenominatorMayContainZero
        );
    }

    #[test]
    fn algebraic_root_affine_transform_constructs_represented_values() {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let transformed = transform_algebraic_root_affine(
            &sqrt_two,
            real(2),
            real(3),
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            transformed.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let representation = transformed.representation.as_ref().unwrap();
        assert_eq!(
            representation.polynomial_coefficients,
            vec![Real::one(), real(-6), Real::one()]
        );
        assert_eq!(representation.interval.lower, real(5));
        assert_eq!(representation.interval.upper, real(7));
        assert!(representation.exact_point_witness().is_none());
        assert!(representation.is_valid());

        let reflected = transform_algebraic_root_affine(
            &sqrt_two,
            real(-1),
            Real::zero(),
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            reflected.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let reflected = reflected.representation.as_ref().unwrap();
        assert_eq!(reflected.interval.lower, real(-2));
        assert_eq!(reflected.interval.upper, real(-1));

        let zero_scale = transform_algebraic_root_affine(
            &sqrt_two,
            Real::zero(),
            real(4),
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            zero_scale.status,
            AlgebraicRootAffineTransformStatus::ZeroScale
        );
    }

    #[test]
    fn decreasing_affine_transform_excludes_a_foreign_source_endpoint_root() {
        // P(x) = (x - 1)(x^2 - 2).  The owned interval (1, 2] contains only
        // sqrt(2), while its excluded lower endpoint is another root.  Merely
        // swapping the endpoints after y = -x would incorrectly include -1.
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(2), real(-2), real(-1), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let report = transform_algebraic_root_affine(
            &source,
            real(-1),
            Real::zero(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = report.representation.expect("refined reflected root");
        assert!(transformed.interval.upper < real(-1));
        assert!(matches!(
            refine_isolated_univariate_polynomial_interval(
                &transformed.polynomial_coefficients,
                &transformed.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));
    }

    #[test]
    fn decreasing_affine_transform_retains_an_owned_upper_endpoint_root() {
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let report = transform_algebraic_root_affine(
            &source,
            real(-1),
            Real::zero(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = report.representation.expect("reflected exact root");
        assert_eq!(transformed.interval.lower, real(-2));
        assert_eq!(transformed.interval.upper, real(-2));
        assert_eq!(transformed.interval.exact_root, Some(real(-2)));
    }

    #[test]
    fn endpoint_root_detection_falls_back_after_inconclusive_modular_images() {
        let prime = i64::try_from(ENDPOINT_NONVANISHING_PRIME).unwrap();
        let modular_collision = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(prime), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            validation: AlgebraicRootValidationReport::valid(),
        };
        assert_eq!(
            algebraic_root_interval_endpoints_are_roots(
                &modular_collision,
                PredicatePolicy::STRICT,
            ),
            Some(false)
        );

        let sqrt_two = real(2).sqrt().unwrap();
        let exact_real_endpoint = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![-sqrt_two.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: sqrt_two,
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..modular_collision
        };
        assert_eq!(
            algebraic_root_interval_endpoints_are_roots(
                &exact_real_endpoint,
                PredicatePolicy::STRICT,
            ),
            Some(true)
        );
    }

    #[test]
    fn algebraic_root_affine_transform_preserves_exact_real_coefficients() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-sqrt_two.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        assert!(source.is_valid());
        assert!(
            source.polynomial_coefficients[0]
                .exact_rational_ref()
                .is_none()
        );

        let transformed =
            transform_algebraic_root_affine(&source, real(2), real(3), PredicatePolicy::STRICT);
        assert_eq!(
            transformed.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = transformed
            .representation
            .expect("the exact affine image must retain root evidence");
        assert_eq!(
            compare_reals(
                &transformed.polynomial_coefficients[0],
                &(-real(3) - real(2) * sqrt_two),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(transformed.polynomial_coefficients[1], Real::one());
        assert_eq!(transformed.interval.lower, real(5));
        assert_eq!(transformed.interval.upper, real(7));
        assert!(transformed.is_valid());
    }

    #[test]
    fn algebraic_root_affine_transform_preserves_exact_real_scale_and_offset() {
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        let scale = real(2).sqrt().expect("positive exact square root");
        let offset = Real::pi();
        let report = transform_algebraic_root_affine(
            &source,
            scale.clone(),
            offset.clone(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = report.representation.expect("exact-Real affine image");
        let expected = [
            offset.clone() * offset.clone() - real(4),
            real(-2) * offset.clone(),
            Real::one(),
        ];
        for (actual, expected) in transformed.polynomial_coefficients.iter().zip(expected) {
            assert_eq!(
                compare_reals(actual, &expected, PredicatePolicy::STRICT).value(),
                Some(Ordering::Equal)
            );
        }
        assert_eq!(
            compare_reals(
                &transformed.interval.lower,
                &(offset.clone() + scale.clone()),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &transformed.interval.upper,
                &(offset + real(2) * scale),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert!(transformed.is_valid());
    }

    #[test]
    fn algebraic_root_affine_transform_retains_an_arbitrary_exact_point_witness() {
        let source = exact_point_representation(0, real(2));
        let scale = real(2).sqrt().unwrap();
        let offset = Real::pi();
        let expected = real(2) * scale.clone() + offset.clone();
        let report =
            transform_algebraic_root_affine(&source, scale, offset, PredicatePolicy::STRICT);

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = report.representation.expect("exact-Real point image");
        assert_eq!(transformed.exact_point_witness(), Some(&expected));
        assert_eq!(transformed.interval.lower, expected);
        assert_eq!(
            validate_algebraic_root_representation(&transformed, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid
        );
    }

    #[test]
    fn algebraic_root_affine_transform_replays_cached_source_validation_strictly() {
        let mut stale = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };
        stale.interval.lower = real(3);
        assert!(stale.is_valid());

        let report = transform_algebraic_root_affine(
            &stale,
            Real::one(),
            Real::zero(),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::InvalidEvidence
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn algebraic_root_affine_transform_handles_degree_128_without_machine_binomials() {
        let mut polynomial = vec![Real::zero(); 129];
        polynomial[0] = real(-2);
        polynomial[128] = Real::one();
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: polynomial.clone(),
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport::valid(),
        };

        let report = transform_algebraic_root_affine(
            &source,
            Real::one(),
            Real::one(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let transformed = report.representation.expect("degree-128 affine image");
        let mut expected = Vec::with_capacity(129);
        let mut binomial = BigUint::from(1_u8);
        for power in 0..=128_usize {
            let mut coefficient = BigInt::from(binomial.clone());
            if (128 - power) % 2 == 1 {
                coefficient = -coefficient;
            }
            if power == 0 {
                coefficient -= 2;
            }
            expected.push(Real::from(Rational::from_bigint(coefficient)));
            if power < 128 {
                binomial = binomial * BigUint::from(128 - power) / BigUint::from(power + 1);
            }
        }
        assert_eq!(transformed.polynomial_coefficients, expected);
        assert_eq!(transformed.interval.lower, real(2));
        assert_eq!(transformed.interval.upper, real(3));
        assert!(transformed.is_valid());
    }

    #[test]
    fn affine_homogeneous_horner_matches_retained_power_sum() {
        let polynomial = [real(-7), real(3), Real::zero(), real(-2), real(5), real(1)];
        let scale = real(2);
        let offset = real(-1);
        let horner =
            affine_transformed_polynomial(&polynomial, &scale, &offset, PredicatePolicy::STRICT)
                .unwrap();
        let power_sum = affine_transformed_polynomial_power_sum(
            &polynomial,
            &scale,
            &offset,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(horner, power_sum);
    }

    #[test]
    fn affine_exact_real_horner_matches_retained_power_sum() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let polynomial = [
            -sqrt_two.clone(),
            Real::pi(),
            Real::zero(),
            sqrt_two,
            Real::one(),
        ];
        let scale = real(2);
        let offset = real(-1);
        let horner =
            affine_transformed_polynomial(&polynomial, &scale, &offset, PredicatePolicy::STRICT)
                .unwrap();
        let power_sum = affine_transformed_polynomial_power_sum(
            &polynomial,
            &scale,
            &offset,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(horner.len(), power_sum.len());
        for (horner, power_sum) in horner.iter().zip(&power_sum) {
            assert_eq!(
                compare_reals(horner, power_sum, PredicatePolicy::STRICT).value(),
                Some(Ordering::Equal)
            );
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn generated_rational_affine_horner_matches_retained_power_sum(
            coefficients in prop::collection::vec(-8_i16..=8, 1..7),
            scale in (-5_i16..=5).prop_filter("nonzero affine scale", |value| *value != 0),
            offset in -5_i16..=5,
        ) {
            prop_assume!(coefficients.last().is_some_and(|coefficient| *coefficient != 0));
            let polynomial = coefficients
                .into_iter()
                .map(|coefficient| real(i64::from(coefficient)))
                .collect::<Vec<_>>();
            let scale = real(i64::from(scale));
            let offset = real(i64::from(offset));
            let horner = affine_transformed_polynomial(
                &polynomial,
                &scale,
                &offset,
                PredicatePolicy::STRICT,
            );
            let power_sum = affine_transformed_polynomial_power_sum(
                &polynomial,
                &scale,
                &offset,
                PredicatePolicy::STRICT,
            );

            prop_assert_eq!(horner, power_sum);
        }

        #[test]
        fn generated_affine_exact_real_horner_matches_retained_power_sum(
            coefficient_pairs in prop::collection::vec((-3_i8..=3, -3_i8..=3), 1..6),
        ) {
            let sqrt_two = real(2).sqrt().expect("positive exact square root");
            let polynomial = coefficient_pairs
                .into_iter()
                .map(|(rational, radical)| {
                    real(i64::from(rational)) + real(i64::from(radical)) * sqrt_two.clone()
                })
                .collect::<Vec<_>>();
            let scale = real(2);
            let offset = real(-1);
            let horner = affine_transformed_polynomial(
                &polynomial,
                &scale,
                &offset,
                PredicatePolicy::STRICT,
            );
            let power_sum = affine_transformed_polynomial_power_sum(
                &polynomial,
                &scale,
                &offset,
                PredicatePolicy::STRICT,
            );

            prop_assert!(horner.is_some());
            prop_assert!(power_sum.is_some());
            let horner = horner.unwrap();
            let power_sum = power_sum.unwrap();
            prop_assert_eq!(horner.len(), power_sum.len());
            for (horner, power_sum) in horner.iter().zip(&power_sum) {
                prop_assert_eq!(
                    compare_reals(horner, power_sum, PredicatePolicy::STRICT).value(),
                    Some(Ordering::Equal)
                );
            }
        }

        #[test]
        fn generated_endpoint_root_detection_matches_integer_horner(
            coefficients in prop::collection::vec(-8_i8..=8, 2..7),
            lower in -8_i8..=8,
            upper in -8_i8..=8,
        ) {
            prop_assume!(coefficients.last().is_some_and(|coefficient| *coefficient != 0));
            let evaluate = |point: i8| {
                coefficients.iter().rev().fold(0_i128, |value, coefficient| {
                    value * i128::from(point) + i128::from(*coefficient)
                })
            };
            let expected = evaluate(lower) == 0 || evaluate(upper) == 0;
            let root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: coefficients
                    .iter()
                    .map(|coefficient| real(i64::from(*coefficient)))
                    .collect(),
                interval: IsolatedRootInterval {
                    lower: real(i64::from(lower)),
                    upper: real(i64::from(upper)),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            prop_assert_eq!(
                algebraic_root_interval_endpoints_are_roots(
                    &root,
                    PredicatePolicy::STRICT,
                ),
                Some(expected)
            );
        }

        #[test]
        fn generated_exact_real_point_arithmetic_never_claims_rational_witness(
            radicand in 1_u16..=64,
            integer_offset in -8_i16..=8,
        ) {
            let left_value = real(i64::from(radicand)).sqrt().unwrap();
            let right_value = Real::pi() + real(i64::from(integer_offset));
            let expected = left_value.clone() + right_value.clone();
            prop_assert!(expected.exact_rational_ref().is_none());
            let left = exact_point_representation(0, left_value);
            let right = exact_point_representation(1, right_value);

            let report = arithmetic_algebraic_root_representations(
                &left,
                Some(&right),
                AlgebraicRootArithmeticOp::Add,
                PredicatePolicy::STRICT,
            );

            prop_assert_eq!(
                report.status,
                AlgebraicRootArithmeticStatus::ComputedExactRealWitness
            );
            prop_assert_eq!(report.exact_result.as_ref(), Some(&expected));
            prop_assert!(report.result_representation.is_none());
        }
    }

    proptest! {
        #[test]
        fn generated_linear_roots_become_valid_represented_intervals(root in -64_i16..=64) {
            let root = i64::from(root);
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            problem.add_variable("x", real(0));
            problem.add_constraint(Constraint::equality(
                "generated linear root",
                x - Expr::int(root),
            ));

            let reports = represent_univariate_algebraic_roots(
                &problem.analyze(),
                RootIsolationConfig::default(),
            );

            prop_assert_eq!(reports.len(), 1);
            prop_assert_eq!(
                &reports[0].status,
                &AlgebraicRootRepresentationStatus::Represented
            );
            prop_assert_eq!(reports[0].roots.len(), 1);
            prop_assert!(reports[0].roots[0].is_valid());
            prop_assert_eq!(
                &reports[0].roots[0].polynomial_coefficients,
                &vec![real(-root), Real::one()]
            );
        }

        #[test]
        fn generated_rational_witness_comparisons_match_integer_order(
            left in -64_i16..=64,
            right in -64_i16..=64,
        ) {
            let left = i64::from(left);
            let right = i64::from(right);
            let left_root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-left), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(left),
                    upper: real(left),
                    exact_root: Some(real(left)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };
            let right_root = AlgebraicRootRepresentation {
                constraint_index: 1,
                interval_index: 0,
                polynomial_coefficients: vec![real(-right), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(right),
                    upper: real(right),
                    exact_root: Some(real(right)),
                    distinct_root_count: 1,
                },
                ..left_root.clone()
            };

            let report = compare_algebraic_root_representations(
                &left_root,
                &right_root,
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootComparisonStatus::Compared);
            prop_assert_eq!(report.ordering, Some(left.cmp(&right)));
        }

        #[test]
        fn generated_refinement_comparison_preserves_rational_witness_order(
            left in -16_i16..=16,
            right in -16_i16..=16,
        ) {
            let left = i64::from(left);
            let right = i64::from(right);
            let left_root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-left), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(left),
                    upper: real(left),
                    exact_root: Some(real(left)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };
            let right_root = AlgebraicRootRepresentation {
                constraint_index: 1,
                interval_index: 0,
                polynomial_coefficients: vec![real(-right), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(right),
                    upper: real(right),
                    exact_root: Some(real(right)),
                    distinct_root_count: 1,
                },
                ..left_root.clone()
            };

            let report = compare_algebraic_root_representations_with_refinement(
                &left_root,
                &right_root,
                AlgebraicRootRefinementComparisonConfig::default(),
            );

            prop_assert_eq!(report.comparison.status, AlgebraicRootComparisonStatus::Compared);
            prop_assert_eq!(report.comparison.ordering, Some(left.cmp(&right)));
            prop_assert_eq!(report.refinement_rounds, 0);
        }

        #[test]
        fn generated_rational_witness_arithmetic_matches_integer_arithmetic(
            left in -32_i16..=32,
            right in -32_i16..=32,
        ) {
            let left = i64::from(left);
            let right = i64::from(right);
            let left_root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-left), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(left),
                    upper: real(left),
                    exact_root: Some(real(left)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };
            let right_root = AlgebraicRootRepresentation {
                constraint_index: 1,
                interval_index: 0,
                polynomial_coefficients: vec![real(-right), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(right),
                    upper: real(right),
                    exact_root: Some(real(right)),
                    distinct_root_count: 1,
                },
                ..left_root.clone()
            };

            let sum = arithmetic_algebraic_root_representations(
                &left_root,
                Some(&right_root),
                AlgebraicRootArithmeticOp::Add,
                PredicatePolicy::APPROXIMATE_512,
            );
            let difference = arithmetic_algebraic_root_representations(
                &left_root,
                Some(&right_root),
                AlgebraicRootArithmeticOp::Subtract,
                PredicatePolicy::APPROXIMATE_512,
            );
            let quotient = if right != 0 {
                Some(arithmetic_algebraic_root_representations(
                    &left_root,
                    Some(&right_root),
                    AlgebraicRootArithmeticOp::Divide,
                    PredicatePolicy::APPROXIMATE_512,
                ))
            } else {
                None
            };

            prop_assert_eq!(sum.exact_result, Some(real(left + right)));
            prop_assert_eq!(difference.exact_result, Some(real(left - right)));
            if let Some(quotient) = quotient {
                prop_assert_eq!(
                    quotient.exact_result,
                    Some((real(left) / real(right)).unwrap())
                );
            }
        }

        #[test]
        fn generated_interval_only_negation_reflects_polynomial_and_interval(
            lower in -32_i16..=31,
            width in 1_i16..=32,
            constant in -16_i16..=16,
            linear in -16_i16..=16,
            quadratic in 1_i16..=16,
        ) {
            let lower = i64::from(lower);
            let upper = lower + i64::from(width);
            let constant = i64::from(constant);
            let linear = i64::from(linear);
            let quadratic = i64::from(quadratic);
            let root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(constant), real(linear), real(quadratic)],
                interval: IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = arithmetic_algebraic_root_representations(
                &root,
                None,
                AlgebraicRootArithmeticOp::Negate,
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(
                report.status,
                AlgebraicRootArithmeticStatus::ComputedRepresentation
            );
            let representation = report.result_representation.as_ref().unwrap();
            prop_assert_eq!(
                &representation.polynomial_coefficients,
                &vec![real(constant), real(-linear), real(quadratic)]
            );
            prop_assert_eq!(&representation.interval.lower, &real(-upper));
            prop_assert_eq!(&representation.interval.upper, &real(-lower));
            prop_assert!(representation.exact_point_witness().is_none());
        }

        #[test]
        fn generated_interval_only_addition_by_rational_scalar_matches_affine_transform(
            lower in -24_i16..=23,
            width in 1_i16..=24,
            constant in -12_i16..=12,
            linear in -12_i16..=12,
            quadratic in 1_i16..=12,
            offset in -12_i16..=12,
        ) {
            let lower = i64::from(lower);
            let upper = lower + i64::from(width);
            let offset = i64::from(offset);
            let root = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![
                    real(i64::from(constant)),
                    real(i64::from(linear)),
                    real(i64::from(quadratic)),
                ],
                interval: IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };
            let scalar = AlgebraicRootRepresentation {
                constraint_index: 1,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-offset), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(offset),
                    upper: real(offset),
                    exact_root: Some(real(offset)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            let arithmetic = arithmetic_algebraic_root_representations(
                &root,
                Some(&scalar),
                AlgebraicRootArithmeticOp::Add,
                PredicatePolicy::APPROXIMATE_512,
            );
            let affine = transform_algebraic_root_affine(
                &root,
                Real::one(),
                real(offset),
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(
                arithmetic.status,
                AlgebraicRootArithmeticStatus::ComputedRepresentation
            );
            prop_assert_eq!(affine.status, AlgebraicRootAffineTransformStatus::Transformed);
            prop_assert_eq!(arithmetic.result_representation, affine.representation);
        }

        #[test]
        fn generated_rational_interval_product_matches_all_endpoint_products(
            left_first in -32_i16..=32,
            left_second in -32_i16..=32,
            right_first in -32_i16..=32,
            right_second in -32_i16..=32,
        ) {
            let left_lower = Rational::new(i64::from(left_first.min(left_second)));
            let left_upper = Rational::new(i64::from(left_first.max(left_second)));
            let right_lower = Rational::new(i64::from(right_first.min(right_second)));
            let right_upper = Rational::new(i64::from(right_first.max(right_second)));
            let (lower, upper) = rational_interval_product(
                &left_lower,
                &left_upper,
                &right_lower,
                &right_upper,
            );
            let products = [
                &left_lower * &right_lower,
                &left_lower * &right_upper,
                &left_upper * &right_lower,
                &left_upper * &right_upper,
            ];
            let mut expected_lower = products[0].clone();
            let mut expected_upper = products[0].clone();
            for product in &products[1..] {
                if product < &expected_lower {
                    expected_lower = product.clone();
                }
                if product > &expected_upper {
                    expected_upper = product.clone();
                }
            }

            prop_assert_eq!(lower, expected_lower);
            prop_assert_eq!(upper, expected_upper);
        }

        #[test]
        fn generated_rational_witness_polynomial_evaluation_matches_integer_arithmetic(
            root in -32_i16..=32,
            constant in -32_i16..=32,
            linear in -16_i16..=16,
            quadratic in -8_i16..=8,
        ) {
            let root = i64::from(root);
            let constant = i64::from(constant);
            let linear = i64::from(linear);
            let quadratic = i64::from(quadratic);
            let represented = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-root), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(root),
                    upper: real(root),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = evaluate_polynomial_at_algebraic_root(
                &represented,
                &[real(constant), real(linear), real(quadratic)],
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(
                report.status,
                AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
            );
            prop_assert_eq!(
                report.exact_value,
                Some(real(constant + linear * root + quadratic * root * root))
            );
        }

        #[test]
        fn generated_rational_witness_rational_evaluation_matches_integer_arithmetic(
            root in -24_i16..=24,
            numerator_constant in -24_i16..=24,
            numerator_linear in -12_i16..=12,
            denominator_constant in -24_i16..=24,
            denominator_linear in -12_i16..=12,
        ) {
            let root = i64::from(root);
            let numerator_constant = i64::from(numerator_constant);
            let numerator_linear = i64::from(numerator_linear);
            let denominator_constant = i64::from(denominator_constant);
            let denominator_linear = i64::from(denominator_linear);
            let denominator = denominator_constant + denominator_linear * root;
            prop_assume!(denominator != 0);
            let represented = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-root), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(root),
                    upper: real(root),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = evaluate_rational_expression_at_algebraic_root(
                &represented,
                &[real(numerator_constant), real(numerator_linear)],
                &[real(denominator_constant), real(denominator_linear)],
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(
                report.status,
                AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
            );
            prop_assert_eq!(
                report.exact_value,
                Some((real(numerator_constant + numerator_linear * root) / real(denominator)).unwrap())
            );
        }

        #[test]
        fn generated_rational_witness_affine_transform_preserves_exact_root(
            root in -32_i16..=32,
            scale in (-12_i16..=12).prop_filter("nonzero affine scale", |value| *value != 0),
            offset in -32_i16..=32,
        ) {
            let root = i64::from(root);
            let scale = i64::from(scale);
            let offset = i64::from(offset);
            let represented = AlgebraicRootRepresentation {
                constraint_index: 0,
                symbol: SymbolId(0),
                interval_index: 0,
                polynomial_coefficients: vec![real(-root), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(root),
                    upper: real(root),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = transform_algebraic_root_affine(
                &represented,
                real(scale),
                real(offset),
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootAffineTransformStatus::Transformed);
            let transformed = report.representation.as_ref().unwrap();
            let expected = scale * root + offset;
            let expected = real(expected);
            prop_assert_eq!(transformed.exact_point_witness(), Some(&expected));
            prop_assert_eq!(
                &transformed.polynomial_coefficients,
                &vec![-expected.clone(), Real::one()]
            );
            prop_assert_eq!(&transformed.interval.lower, &expected);
            prop_assert_eq!(&transformed.interval.upper, &expected);
            prop_assert!(transformed.is_valid());
        }
    }
}
