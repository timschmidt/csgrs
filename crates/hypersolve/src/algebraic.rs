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

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::model::{ConstraintKind, Problem};
use crate::prepared::PreparedProblem;
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementReport, IsolatedRootRefinementStatus,
    RootIsolationConfig, RootIsolationStatus, UnivariateRootIsolationReport,
    isolate_univariate_polynomial_roots_with_config,
    refine_isolated_univariate_polynomial_interval,
};
use crate::symbolic::{Expr, SymbolId};
use crate::{
    AlgebraicRootBinaryTransformStatus, AlgebraicRootMobiusTransformStatus,
    AlgebraicRootPolynomialImageStatus, transform_algebraic_root_mobius,
    transform_algebraic_root_polynomial_image, transform_algebraic_roots_binary,
};

/// Representation kind for one isolated algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootKind {
    /// The isolator found an exact rational root value.
    ExactRationalWitness,
    /// The root is represented only by its exact polynomial and isolating
    /// interval.
    IsolatingInterval,
}

/// Validation status for represented algebraic-root evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootValidationStatus {
    /// The representation is structurally valid.
    Valid,
    /// The coefficient vector is empty, constant, or not exact-rational.
    InvalidPolynomial,
    /// The interval endpoints are ordered incorrectly.
    InvalidInterval,
    /// The interval does not claim exactly one distinct root.
    NonUnitIsolation,
    /// The exact rational witness is outside the interval.
    WitnessOutsideInterval,
    /// The exact rational witness does not satisfy the polynomial.
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
            policy: PredicatePolicy,
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
    /// Number of alternating refinement rounds used.
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

/// Status for evaluating a polynomial at a represented algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootPolynomialEvaluationStatus {
    /// The represented root had an exact rational witness, so the polynomial
    /// value was computed exactly.
    EvaluatedExactRationalWitness,
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
/// The input polynomial is stored in ascending power order. For rational
/// witnesses, `exact_value` is filled and compared exactly. For interval-only
/// represented roots, `interval_value` is a conservative exact interval
/// enclosure produced without primitive-float sampling.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootPolynomialEvaluationReport {
    /// Final evaluation status.
    pub status: AlgebraicRootPolynomialEvaluationStatus,
    /// Exact value when the represented root has a rational witness.
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
    /// Both numerator and denominator evaluated exactly at a rational witness.
    EvaluatedExactRationalWitness,
    /// Interval arithmetic certified a nonzero denominator and produced a
    /// rational value enclosure.
    IntervalEvaluated,
    /// The denominator is exactly zero at a rational witness.
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
    /// Exact quotient when the root has a rational witness and the denominator
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
    /// Whether this is an exact rational witness or a non-rational interval
    /// representation.
    pub kind: AlgebraicRootKind,
    /// Validation evidence for the representation.
    pub validation: AlgebraicRootValidationReport,
}

impl AlgebraicRootRepresentation {
    /// Returns the exact rational witness, when the root is represented by a
    /// point value.
    pub fn exact_rational_witness(&self) -> Option<&Real> {
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
    prepared: &PreparedProblem<'_>,
    config: RootIsolationConfig,
) -> Vec<AlgebraicRootRepresentationReport> {
    let reports = isolate_univariate_polynomial_roots_with_config(prepared, config.clone());
    represent_univariate_algebraic_roots_from_reports(prepared, &reports, config.policy)
}

/// Build represented algebraic roots from existing isolation reports.
///
/// The caller supplies the isolation reports so path/curve code can preserve a
/// previous isolation pass and still obtain persistent algebraic-root objects.
/// Reports are matched by `constraint_index`; inactive and non-equality rows
/// are rejected by construction rather than guessed from residual text.
pub fn represent_univariate_algebraic_roots_from_reports(
    prepared: &PreparedProblem<'_>,
    reports: &[UnivariateRootIsolationReport],
    policy: PredicatePolicy,
) -> Vec<AlgebraicRootRepresentationReport> {
    reports
        .iter()
        .map(|report| represent_one_report(prepared.problem(), report, policy))
        .collect()
}

/// Validate one represented algebraic root.
///
/// This checks the representation payload itself: nonconstant exact-rational
/// coefficients, an ordered interval, a unit distinct-root claim, and, when a
/// rational witness is present, exact polynomial replay at that witness. It
/// does not re-run Sturm isolation; the upstream isolation report remains the
/// proof of uniqueness.
pub fn validate_algebraic_root_representation(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootValidationReport {
    validate_root_payload(&root.polynomial_coefficients, &root.interval, policy)
}

/// Compare two represented algebraic roots without leaving the exact boundary.
///
/// This is deliberately narrower than a complete algebraic-number ordering
/// package. It certifies order when exact rational witnesses compare directly
/// or when isolating intervals are disjoint. If intervals overlap, the report
/// returns [`AlgebraicRootComparisonStatus::OverlappingIntervals`] instead of
/// sampling a primitive approximation. This follows the exact construction/
/// decision separation and the Collins-Loos isolating-interval model cited in
/// the module docs.
pub fn compare_algebraic_root_representations(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootComparisonReport {
    if !left.is_valid() || !right.is_valid() {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::InvalidEvidence,
            None,
            Some(
                "both algebraic roots must pass structural validation before comparison".to_owned(),
            ),
        );
    }
    if same_represented_root(left, right) {
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::SameRepresentation,
            Some(Ordering::Equal),
            None,
        );
    }
    if let (Some(left), Some(right)) = (
        left.exact_rational_witness(),
        right.exact_rational_witness(),
    ) {
        let Some(ordering) = compare_reals_with_policy(left, right, policy).value() else {
            return algebraic_comparison_report(
                AlgebraicRootComparisonStatus::Undecided,
                None,
                Some("could not compare exact rational witnesses".to_owned()),
            );
        };
        return algebraic_comparison_report(
            AlgebraicRootComparisonStatus::Compared,
            Some(ordering),
            None,
        );
    }

    let Some(left_before_right) =
        compare_reals_with_policy(&left.interval.upper, &right.interval.lower, policy).value()
    else {
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

    let Some(left_after_right) =
        compare_reals_with_policy(&left.interval.lower, &right.interval.upper, policy).value()
    else {
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
/// stops at overlapping isolating intervals, each root is refined with a
/// Sturm sequence package in the standard real-root isolation style until the
/// intervals become disjoint, an exact rational witness appears, or the
/// configured work budget is exhausted. Refinement is a proof-producing
/// operation, and failure remains an explicit
/// undecided report rather than a sampled approximation.
pub fn compare_algebraic_root_representations_with_refinement(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    config: AlgebraicRootRefinementComparisonConfig,
) -> AlgebraicRootRefinementComparisonReport {
    let mut refined_left = left.clone();
    let mut refined_right = right.clone();
    let mut comparison =
        compare_algebraic_root_representations(&refined_left, &refined_right, config.policy);
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
        let left_refinement = refine_isolated_univariate_polynomial_interval(
            &refined_left.polynomial_coefficients,
            &refined_left.interval,
            root_config.clone(),
        );
        let left_progress =
            apply_refined_interval(&mut refined_left, &left_refinement, config.policy);
        left_refinements.push(left_refinement);
        let right_refinement = refine_isolated_univariate_polynomial_interval(
            &refined_right.polynomial_coefficients,
            &refined_right.interval,
            root_config,
        );
        let right_progress =
            apply_refined_interval(&mut refined_right, &right_refinement, config.policy);
        right_refinements.push(right_refinement);

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
        comparison =
            compare_algebraic_root_representations(&refined_left, &refined_right, config.policy);
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
/// certified isolating interval. This is the exactness boundary construction/decision split:
/// the value is built as exact evidence, then the sign predicate reads only
/// certified interval data., Sturm's theorem, and the standard real-root isolation model, matching the arithmetic and
/// refinement modules cited at the point of use.
pub fn compare_algebraic_root_representations_by_difference(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    config: AlgebraicRootRefinementComparisonConfig,
) -> AlgebraicRootDifferenceComparisonReport {
    let refinement =
        compare_algebraic_root_representations_with_refinement(left, right, config.clone());
    if refinement.comparison.status != AlgebraicRootComparisonStatus::OverlappingIntervals {
        return algebraic_difference_comparison_report(
            refinement.comparison.clone(),
            refinement,
            None,
        );
    }

    let difference = arithmetic_algebraic_root_representations(
        left,
        Some(right),
        AlgebraicRootArithmeticOp::Subtract,
    );
    let comparison = match difference.status {
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness => {
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
            match compare_reals_with_policy(value, &Real::zero(), config.policy).value() {
                Some(ordering) => algebraic_comparison_report(
                    AlgebraicRootComparisonStatus::Compared,
                    Some(ordering),
                    Some("comparison decided by exact rational difference".to_owned()),
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

/// Compute exact arithmetic for represented roots with rational witnesses.
///
/// This is deliberately a witness arithmetic package, not a full algebraic
/// number field. When both required inputs carry exact rational witnesses, the
/// result is computed exactly in [`Real`]. When exactly one binary operand is
/// an exact rational witness, add/subtract/multiply/divide by that scalar is
/// lowered to exact affine or linear-fractional construction; unary negation
/// is the same structural operation specialized to `scale = -1`.
/// These operations transform retained algebraic evidence rather than sampling
/// approximations. Binary non-rational/non-rational arithmetic remains explicit
/// [`AlgebraicRootArithmeticStatus::NonRationalInput`]. This follows the exact
/// exact-object rule from the exact-geometric-computation model:
/// unsupported algebraic arithmetic remains explicit until a true
/// algebraic-number package exists.
pub fn arithmetic_algebraic_root_representations(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
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
    if let Some(report) =
        arithmetic_with_one_rational_scalar(left, right, operation, PredicatePolicy)
    {
        return report;
    }
    if let Some(report) =
        arithmetic_with_same_representation(left, right, operation, PredicatePolicy)
    {
        return report;
    }
    if let Some(report) =
        arithmetic_with_independent_representations(left, right, operation, PredicatePolicy)
    {
        return report;
    }
    let Some(left_value) = left.exact_rational_witness() else {
        if operation == AlgebraicRootArithmeticOp::Negate {
            let representation = negate_algebraic_root_representation(left);
            let status = if representation.is_valid() {
                AlgebraicRootArithmeticStatus::ComputedRepresentation
            } else {
                AlgebraicRootArithmeticStatus::Undecided
            };
            let message = (!representation.is_valid())
                .then(|| "negated algebraic root representation did not validate".to_owned());
            return algebraic_arithmetic_report(
                operation,
                status,
                None,
                Some(representation),
                message,
            );
        }
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::NonRationalInput,
            None,
            None,
            Some("left algebraic root has no exact rational witness".to_owned()),
        );
    };
    let result = match operation {
        AlgebraicRootArithmeticOp::Negate => -left_value.clone(),
        AlgebraicRootArithmeticOp::Add
        | AlgebraicRootArithmeticOp::Subtract
        | AlgebraicRootArithmeticOp::Multiply
        | AlgebraicRootArithmeticOp::Divide => {
            let Some(right) = right else {
                return algebraic_arithmetic_report(
                    operation,
                    AlgebraicRootArithmeticStatus::InvalidEvidence,
                    None,
                    None,
                    Some("binary algebraic root arithmetic requires a right input".to_owned()),
                );
            };
            let Some(right_value) = right.exact_rational_witness() else {
                return algebraic_arithmetic_report(
                    operation,
                    AlgebraicRootArithmeticStatus::NonRationalInput,
                    None,
                    None,
                    Some("right algebraic root has no exact rational witness".to_owned()),
                );
            };
            match operation {
                AlgebraicRootArithmeticOp::Add => left_value.clone() + right_value.clone(),
                AlgebraicRootArithmeticOp::Subtract => left_value.clone() - right_value.clone(),
                AlgebraicRootArithmeticOp::Multiply => left_value.clone() * right_value.clone(),
                AlgebraicRootArithmeticOp::Divide => {
                    let Ok(quotient) = left_value.clone() / right_value.clone() else {
                        return algebraic_arithmetic_report(
                            operation,
                            AlgebraicRootArithmeticStatus::Undecided,
                            None,
                            None,
                            Some("exact rational witness division failed".to_owned()),
                        );
                    };
                    quotient
                }
                AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
            }
        }
    };
    algebraic_arithmetic_report(
        operation,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
        Some(result),
        None,
        None,
    )
}

/// Lower a supported mixed rational/non-rational operation to an affine image.
///
/// the exact-geometric-computation model separates construction of exact
/// algebraic objects from later predicate decisions. A scalar affine image of one represented
/// root is a safe construction because `scale^n * P((y - offset) / scale)`
/// gives exact polynomial evidence for the image and preserves interval
/// evidence by exact endpoint transforms. A rational scalar divided by a
/// represented nonzero root is delegated to the linear-fractional construction
/// package. General products, sums, or quotients of two independent
/// non-rational algebraic roots need resultants or a full algebraic-number
/// package, so this helper refuses those cases.
fn arithmetic_with_one_rational_scalar(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootArithmeticReport> {
    if operation == AlgebraicRootArithmeticOp::Negate {
        return None;
    }
    let right = right?;
    let left_value = left.exact_rational_witness();
    let right_value = right.exact_rational_witness();
    match (left_value, right_value) {
        (Some(_), Some(_)) | (None, None) => None,
        (Some(scalar), None) => match operation {
            AlgebraicRootArithmeticOp::Add => Some(affine_transform_arithmetic_report(
                right,
                Real::one(),
                scalar.clone(),
                operation,
                policy,
                "left rational scalar added to represented right root",
            )),
            AlgebraicRootArithmeticOp::Subtract => Some(affine_transform_arithmetic_report(
                right,
                -Real::one(),
                scalar.clone(),
                operation,
                policy,
                "represented right root subtracted from left rational scalar",
            )),
            AlgebraicRootArithmeticOp::Multiply => Some(multiply_by_rational_scalar_report(
                right, scalar, operation, policy,
            )),
            AlgebraicRootArithmeticOp::Divide => Some(mobius_transform_arithmetic_report(
                right,
                Real::zero(),
                scalar.clone(),
                Real::one(),
                Real::zero(),
                operation,
                policy,
                "left rational scalar divided by represented right root",
            )),
            AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
        },
        (None, Some(scalar)) => match operation {
            AlgebraicRootArithmeticOp::Add => Some(affine_transform_arithmetic_report(
                left,
                Real::one(),
                scalar.clone(),
                operation,
                policy,
                "right rational scalar added to represented left root",
            )),
            AlgebraicRootArithmeticOp::Subtract => Some(affine_transform_arithmetic_report(
                left,
                Real::one(),
                -scalar.clone(),
                operation,
                policy,
                "right rational scalar subtracted from represented left root",
            )),
            AlgebraicRootArithmeticOp::Multiply => Some(multiply_by_rational_scalar_report(
                left, scalar, operation, policy,
            )),
            AlgebraicRootArithmeticOp::Divide => Some(divide_by_rational_scalar_report(
                left, scalar, operation, policy,
            )),
            AlgebraicRootArithmeticOp::Negate => unreachable!("handled above"),
        },
    }
}

fn multiply_by_rational_scalar_report(
    root: &AlgebraicRootRepresentation,
    scalar: &Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    let Some(ordering) = compare_reals_with_policy(scalar, &Real::zero(), policy).value() else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not certify rational scalar zero/nonzero for multiplication".to_owned()),
        );
    };
    if ordering == Ordering::Equal {
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
        "represented root multiplied by nonzero rational scalar",
    )
}

fn divide_by_rational_scalar_report(
    root: &AlgebraicRootRepresentation,
    scalar: &Real,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootArithmeticReport {
    let Some(ordering) = compare_reals_with_policy(scalar, &Real::zero(), policy).value() else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not certify rational divisor zero/nonzero".to_owned()),
        );
    };
    if ordering == Ordering::Equal {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("division by zero rational witness is not a constructed value".to_owned()),
        );
    }
    let Ok(scale) = Real::one() / scalar.clone() else {
        return algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::Undecided,
            None,
            None,
            Some("could not invert rational divisor exactly".to_owned()),
        );
    };
    affine_transform_arithmetic_report(
        root,
        scale,
        Real::zero(),
        operation,
        policy,
        "represented root divided by nonzero rational scalar",
    )
}

/// Lowers operations involving the same represented root to polynomial images.
///
/// If both operands name the same algebraic object `alpha`, then `alpha +
/// alpha`, `alpha - alpha`, `alpha * alpha`, and `alpha / alpha` do not need a
/// general two-root algebraic-number field. They are respectively the exact
/// polynomial images `2*x`, `0`, `x^2`, and the constant `1` on the same
/// source evidence. This uses the resultant-backed image package and keeps the
/// the exactness boundary construction boundary explicit: independent non-rational operands still
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
        AlgebraicRootArithmeticOp::Add => vec![Real::zero(), Real::from(2)],
        AlgebraicRootArithmeticOp::Subtract => {
            return Some(algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness,
                Some(Real::zero()),
                None,
                Some("same represented root subtracted from itself".to_owned()),
            ));
        }
        AlgebraicRootArithmeticOp::Multiply => vec![Real::zero(), Real::zero(), Real::one()],
        AlgebraicRootArithmeticOp::Divide => {
            if root_interval_contains_zero(&left.interval, policy).unwrap_or(true) {
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
        AlgebraicRootPolynomialImageStatus::Transformed => algebraic_arithmetic_report(
            operation,
            AlgebraicRootArithmeticStatus::ComputedRepresentation,
            None,
            transform.representation,
            Some("same represented root arithmetic lowered to polynomial image".to_owned()),
        ),
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
        | AlgebraicRootPolynomialImageStatus::NonMonotoneImage
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
    if left.exact_rational_witness().is_some() || right.exact_rational_witness().is_some() {
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
        | AlgebraicRootBinaryTransformStatus::UnsupportedCoefficient
        | AlgebraicRootBinaryTransformStatus::InvalidTransformedEvidence => {
            algebraic_arithmetic_report(
                operation,
                AlgebraicRootArithmeticStatus::InvalidEvidence,
                None,
                transform.representation,
                transform.message,
            )
        }
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

fn root_interval_contains_zero(
    interval: &IsolatedRootInterval,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = compare_reals_with_policy(&interval.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&interval.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
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
/// endpoints, rather than sampling a midpoint. The operation follows the exact
/// exact-object boundary from the exact-geometric-computation model:
/// construction returns retained algebraic evidence, and later callers still
/// use comparison/evaluation reports for certified decisions.
pub fn transform_algebraic_root_affine(
    root: &AlgebraicRootRepresentation,
    scale: Real,
    offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootAffineTransformReport {
    if !root.is_valid() {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::InvalidEvidence,
            scale,
            offset,
            None,
            Some("algebraic root representation must be valid before transformation".to_owned()),
        );
    }
    let Some(scale_sign) = compare_reals_with_policy(&scale, &Real::zero(), policy).value() else {
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
    if root.polynomial_coefficients.len() <= 1
        || root
            .polynomial_coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::InvalidEvidence,
            scale,
            offset,
            None,
            Some(
                "affine algebraic-root construction requires exact-rational polynomial evidence"
                    .to_owned(),
            ),
        );
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
    let Some(interval) = affine_transformed_interval(&root.interval, &scale, &offset, policy)
    else {
        return algebraic_affine_transform_report(
            AlgebraicRootAffineTransformStatus::Undecided,
            scale,
            offset,
            None,
            Some("could not construct transformed isolating interval exactly".to_owned()),
        );
    };
    let kind = if interval.exact_root.is_some() {
        AlgebraicRootKind::ExactRationalWitness
    } else {
        AlgebraicRootKind::IsolatingInterval
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        kind,
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

/// Evaluate an exact-rational polynomial at a represented algebraic root.
///
/// This is the first consumer-facing scalar operation for
/// [`AlgebraicRootRepresentation`]. If the root carries an exact rational
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
    if !root.is_valid() {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence,
            None,
            None,
            None,
            Some("algebraic root representation must be valid before evaluation".to_owned()),
        );
    }
    if polynomial_coefficients.is_empty() {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial,
            None,
            None,
            None,
            Some("polynomial evaluation requires at least one coefficient".to_owned()),
        );
    }
    let Some(polynomial) = trim_polynomial(polynomial_coefficients.to_vec(), policy) else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            None,
            None,
            Some("could not trim evaluated polynomial coefficients exactly".to_owned()),
        );
    };
    if polynomial
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial,
            None,
            None,
            None,
            Some("polynomial evaluation requires exact-rational coefficients".to_owned()),
        );
    }
    if let Some(witness) = root.exact_rational_witness() {
        let value = evaluate_polynomial(&polynomial, witness);
        let Some(sign) = compare_reals_with_policy(&value, &Real::zero(), policy).value() else {
            return algebraic_polynomial_evaluation_report(
                AlgebraicRootPolynomialEvaluationStatus::Undecided,
                Some(value),
                None,
                None,
                Some("could not certify exact rational witness value sign".to_owned()),
            );
        };
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness,
            Some(value),
            None,
            Some(sign),
            None,
        );
    }

    let Some(interval) = evaluate_polynomial_interval(
        &polynomial,
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
    let Some(lower_cmp) = compare_reals_with_policy(&interval.lower, &Real::zero(), policy).value()
    else {
        return algebraic_polynomial_evaluation_report(
            AlgebraicRootPolynomialEvaluationStatus::Undecided,
            None,
            Some(interval),
            None,
            Some("could not compare interval lower endpoint with zero".to_owned()),
        );
    };
    let Some(upper_cmp) = compare_reals_with_policy(&interval.upper, &Real::zero(), policy).value()
    else {
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

/// Evaluate an exact-rational rational expression at a represented root.
///
/// Numerator and denominator are supplied in ascending power order. The
/// denominator must be certified nonzero before division is performed. For
/// rational witnesses, this is exact scalar division in [`Real`]. For
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
    let numerator = evaluate_polynomial_at_algebraic_root(root, numerator_coefficients, policy);
    let denominator = evaluate_polynomial_at_algebraic_root(root, denominator_coefficients, policy);
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
            Some("algebraic root representation must be valid before evaluation".to_owned()),
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
        let Some(denominator_sign) =
            compare_reals_with_policy(denominator_value, &Real::zero(), policy).value()
        else {
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
        let Ok(quotient) = numerator_value.clone() / denominator_value.clone() else {
            return algebraic_rational_evaluation_report(
                AlgebraicRootRationalEvaluationStatus::Undecided,
                numerator,
                denominator,
                None,
                None,
                None,
                Some("exact rational division failed".to_owned()),
            );
        };
        let sign = compare_reals_with_policy(&quotient, &Real::zero(), policy).value();
        return algebraic_rational_evaluation_report(
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness,
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
    if interval_contains_zero(denominator_interval, policy).unwrap_or(true) {
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
    }
    let Some(denominator_reciprocal) = interval_reciprocal(denominator_interval, policy) else {
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
    let sign = interval_sign(&quotient, policy);
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
    let kind = if interval.exact_root.is_some() {
        AlgebraicRootKind::ExactRationalWitness
    } else {
        AlgebraicRootKind::IsolatingInterval
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        kind,
        validation: AlgebraicRootValidationReport::valid(),
    };
    representation.validation =
        validate_algebraic_root_representation(&representation, PredicatePolicy);
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
            Some("constraint index is outside the prepared problem".to_owned()),
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
                kind: if interval.exact_root.is_some() {
                    AlgebraicRootKind::ExactRationalWitness
                } else {
                    AlgebraicRootKind::IsolatingInterval
                },
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
    if polynomial.len() <= 1
        || polynomial
            .iter()
            .any(|value| value.exact_rational_ref().is_none())
    {
        return AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::InvalidPolynomial,
            "represented algebraic roots require nonconstant exact-rational polynomials",
        );
    }
    match compare_reals_with_policy(&interval.lower, &interval.upper, policy).value() {
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
            "exact rational witness is outside its isolating interval",
        );
    }
    match compare_reals_with_policy(
        &evaluate_polynomial(polynomial, root),
        &Real::zero(),
        policy,
    )
    .value()
    {
        Some(Ordering::Equal) => AlgebraicRootValidationReport::valid(),
        Some(Ordering::Less | Ordering::Greater) => AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::WitnessDoesNotSatisfyPolynomial,
            "exact rational witness does not satisfy the represented polynomial",
        ),
        None => AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::Undecided,
            "could not replay exact rational witness",
        ),
    }
}

fn point_lies_in_interval(
    point: &Real,
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> bool {
    let Some(lower_cmp) = compare_reals_with_policy(point, lower, policy).value() else {
        return false;
    };
    let Some(upper_cmp) = compare_reals_with_policy(point, upper, policy).value() else {
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
        match compare_reals_with_policy(trailing, &Real::zero(), policy).value()? {
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

fn evaluate_polynomial(polynomial: &[Real], point: &Real) -> Real {
    polynomial
        .iter()
        .rev()
        .cloned()
        .fold(Real::zero(), |value, coefficient| {
            value * point.clone() + coefficient
        })
}

fn affine_transformed_polynomial(
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
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let first = scale.clone() * interval.lower.clone() + offset.clone();
    let second = scale.clone() * interval.upper.clone() + offset.clone();
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    Some(IsolatedRootInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
        exact_root: interval
            .exact_root
            .as_ref()
            .map(|root| scale.clone() * root.clone() + offset.clone()),
        distinct_root_count: interval.distinct_root_count,
    })
}

fn real_pow_nonnegative(value: &Real, exponent: usize) -> Real {
    let mut result = Real::one();
    for _ in 0..exponent {
        result *= value.clone();
    }
    result
}

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
    let mut value = AlgebraicPolynomialValueInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for coefficient in polynomial.iter().rev() {
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

fn interval_reciprocal(
    value: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<AlgebraicPolynomialValueInterval> {
    if interval_contains_zero(value, policy)? {
        return None;
    }
    let lower_reciprocal = (Real::one() / value.lower.clone()).ok()?;
    let upper_reciprocal = (Real::one() / value.upper.clone()).ok()?;
    let mut endpoints = [lower_reciprocal, upper_reciprocal];
    sort_reals_exact(&mut endpoints, policy)?;
    Some(AlgebraicPolynomialValueInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
    })
}

fn interval_contains_zero(
    value: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = compare_reals_with_policy(&value.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&value.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn interval_sign(
    value: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    let lower = compare_reals_with_policy(&value.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&value.upper, &Real::zero(), policy).value()?;
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
            let ordering =
                compare_reals_with_policy(&values[cursor], &values[cursor - 1], policy).value()?;
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
    root.kind = if root.interval.exact_root.is_some() {
        AlgebraicRootKind::ExactRationalWitness
    } else {
        AlgebraicRootKind::IsolatingInterval
    };
    root.validation = validate_algebraic_root_representation(root, policy);
    root.is_valid()
}

fn represented_root_sign(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let Some(value) = root.exact_rational_witness() {
        return compare_reals_with_policy(value, &Real::zero(), policy).value();
    }
    let upper = compare_reals_with_policy(&root.interval.upper, &Real::zero(), policy).value()?;
    if upper == Ordering::Less {
        return Some(Ordering::Less);
    }
    let lower = compare_reals_with_policy(&root.interval.lower, &Real::zero(), policy).value()?;
    if lower == Ordering::Greater {
        return Some(Ordering::Greater);
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
    use proptest::prelude::*;

    use super::*;
    use crate::model::Constraint;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn ratio(numerator: i64, denominator: i64) -> Real {
        (real(numerator) / real(denominator)).unwrap()
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
            &PreparedProblem::new(&problem),
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
                .all(|root| root.kind == AlgebraicRootKind::IsolatingInterval)
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
            &PreparedProblem::new(&problem),
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
        assert!(reports[0].roots.iter().any(|root| {
            root.kind == AlgebraicRootKind::ExactRationalWitness
                && root.exact_rational_witness() == Some(&real(1))
        }));
        assert!(reports[0].roots.iter().all(|root| {
            validate_algebraic_root_representation(root, PredicatePolicy).status
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport::valid(),
        };
        assert_eq!(
            validate_algebraic_root_representation(&invalid_count, PredicatePolicy).status,
            AlgebraicRootValidationStatus::NonUnitIsolation
        );

        let bad_witness = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: real(0),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            ..invalid_count
        };
        assert_eq!(
            validate_algebraic_root_representation(&bad_witness, PredicatePolicy).status,
            AlgebraicRootValidationStatus::WitnessDoesNotSatisfyPolynomial
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
            kind: AlgebraicRootKind::IsolatingInterval,
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

        let comparison = compare_algebraic_root_representations(&left, &right, PredicatePolicy);
        assert_eq!(comparison.status, AlgebraicRootComparisonStatus::Compared);
        assert_eq!(comparison.ordering, Some(Ordering::Less));

        let same = compare_algebraic_root_representations(&left, &left, PredicatePolicy);
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
            kind: AlgebraicRootKind::IsolatingInterval,
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

        let comparison =
            compare_algebraic_root_representations(&valid, &overlapping, PredicatePolicy);
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
            compare_algebraic_root_representations(&invalid, &valid, PredicatePolicy).status,
            AlgebraicRootComparisonStatus::InvalidEvidence
        );
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
            kind: AlgebraicRootKind::IsolatingInterval,
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
            compare_algebraic_root_representations(&sqrt_two, &sqrt_three, PredicatePolicy).status,
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
            kind: AlgebraicRootKind::IsolatingInterval,
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
            kind: AlgebraicRootKind::ExactRationalWitness,
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
        );
        assert_eq!(product.exact_result, Some(real(6)));

        let quotient = arithmetic_algebraic_root_representations(
            &right,
            Some(&left),
            AlgebraicRootArithmeticOp::Divide,
        );
        assert_eq!(quotient.exact_result, Some((real(3) / real(2)).unwrap()));

        let negation = arithmetic_algebraic_root_representations(
            &left,
            None,
            AlgebraicRootArithmeticOp::Negate,
        );
        assert_eq!(negation.exact_result, Some(real(-2)));
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let report = arithmetic_algebraic_root_representations(
            &interval_only,
            None,
            AlgebraicRootArithmeticOp::Negate,
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
        assert_eq!(negated.kind, AlgebraicRootKind::IsolatingInterval);

        let mut invalid = interval_only;
        invalid.validation = AlgebraicRootValidationReport::invalid(
            AlgebraicRootValidationStatus::InvalidPolynomial,
            "test invalid",
        );
        let invalid_report = arithmetic_algebraic_root_representations(
            &invalid,
            None,
            AlgebraicRootArithmeticOp::Negate,
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
            kind: AlgebraicRootKind::IsolatingInterval,
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
            kind: AlgebraicRootKind::ExactRationalWitness,
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

        let same_product = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Multiply,
        );
        assert_eq!(
            same_product.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        let square_root = same_product.result_representation.as_ref().unwrap();
        assert_eq!(
            square_root.polynomial_coefficients,
            vec![real(4), real(-4), Real::one()]
        );

        let same_difference = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Subtract,
        );
        assert_eq!(same_difference.exact_result, Some(Real::zero()));

        let same_quotient = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_two),
            AlgebraicRootArithmeticOp::Divide,
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
        );
        assert_eq!(
            independent_product.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );

        let independent_quotient = arithmetic_algebraic_root_representations(
            &sqrt_two,
            Some(&sqrt_three),
            AlgebraicRootArithmeticOp::Divide,
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
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let rational = evaluate_polynomial_at_algebraic_root(
            &rational_root,
            &[real(-9), Real::zero(), Real::one()],
            PredicatePolicy,
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let positive = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[Real::one(), Real::one()],
            PredicatePolicy,
        );
        assert_eq!(
            positive.status,
            AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
        );
        assert_eq!(positive.sign, Some(Ordering::Greater));

        let crossing = evaluate_polynomial_at_algebraic_root(
            &sqrt_two,
            &[real(-2), Real::zero(), Real::one()],
            PredicatePolicy,
        );
        assert_eq!(
            crossing.status,
            AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero
        );
        assert_eq!(crossing.sign, None);
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
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let exact = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[real(1), Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy,
        );
        assert_eq!(
            exact.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        );
        assert_eq!(exact.exact_value, Some(real(2)));
        assert_eq!(exact.sign, Some(Ordering::Greater));

        let zero_denominator = evaluate_rational_expression_at_algebraic_root(
            &rational_root,
            &[Real::one()],
            &[real(-3), Real::one()],
            PredicatePolicy,
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let interval = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one()],
            &[Real::one(), Real::one()],
            PredicatePolicy,
        );
        assert_eq!(
            interval.status,
            AlgebraicRootRationalEvaluationStatus::IntervalEvaluated
        );
        assert_eq!(interval.sign, Some(Ordering::Greater));
        assert!(interval.interval_value.is_some());

        let may_contain_zero = evaluate_rational_expression_at_algebraic_root(
            &sqrt_two,
            &[Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy,
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport::valid(),
        };
        let transformed =
            transform_algebraic_root_affine(&sqrt_two, real(2), real(3), PredicatePolicy);
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
        assert_eq!(representation.kind, AlgebraicRootKind::IsolatingInterval);
        assert!(representation.is_valid());

        let reflected =
            transform_algebraic_root_affine(&sqrt_two, real(-1), Real::zero(), PredicatePolicy);
        assert_eq!(
            reflected.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let reflected = reflected.representation.as_ref().unwrap();
        assert_eq!(reflected.interval.lower, real(-2));
        assert_eq!(reflected.interval.upper, real(-1));

        let zero_scale =
            transform_algebraic_root_affine(&sqrt_two, Real::zero(), real(4), PredicatePolicy);
        assert_eq!(
            zero_scale.status,
            AlgebraicRootAffineTransformStatus::ZeroScale
        );
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
                &PreparedProblem::new(&problem),
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
                kind: AlgebraicRootKind::ExactRationalWitness,
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
                PredicatePolicy,
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
                kind: AlgebraicRootKind::ExactRationalWitness,
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
                kind: AlgebraicRootKind::ExactRationalWitness,
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
            );
            let difference = arithmetic_algebraic_root_representations(
                &left_root,
                Some(&right_root),
                AlgebraicRootArithmeticOp::Subtract,
            );
            let quotient = if right != 0 {
                Some(arithmetic_algebraic_root_representations(
                    &left_root,
                    Some(&right_root),
                    AlgebraicRootArithmeticOp::Divide,
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
                kind: AlgebraicRootKind::IsolatingInterval,
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = arithmetic_algebraic_root_representations(
                &root,
                None,
                AlgebraicRootArithmeticOp::Negate,
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
            prop_assert_eq!(&representation.kind, &AlgebraicRootKind::IsolatingInterval);
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
                kind: AlgebraicRootKind::IsolatingInterval,
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
                kind: AlgebraicRootKind::ExactRationalWitness,
                validation: AlgebraicRootValidationReport::valid(),
            };

            let arithmetic = arithmetic_algebraic_root_representations(
                &root,
                Some(&scalar),
                AlgebraicRootArithmeticOp::Add,
            );
            let affine = transform_algebraic_root_affine(
                &root,
                Real::one(),
                real(offset),
                PredicatePolicy,
            );

            prop_assert_eq!(
                arithmetic.status,
                AlgebraicRootArithmeticStatus::ComputedRepresentation
            );
            prop_assert_eq!(affine.status, AlgebraicRootAffineTransformStatus::Transformed);
            prop_assert_eq!(arithmetic.result_representation, affine.representation);
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
                kind: AlgebraicRootKind::ExactRationalWitness,
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = evaluate_polynomial_at_algebraic_root(
                &represented,
                &[real(constant), real(linear), real(quadratic)],
                PredicatePolicy,
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
                kind: AlgebraicRootKind::ExactRationalWitness,
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = evaluate_rational_expression_at_algebraic_root(
                &represented,
                &[real(numerator_constant), real(numerator_linear)],
                &[real(denominator_constant), real(denominator_linear)],
                PredicatePolicy,
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
                kind: AlgebraicRootKind::ExactRationalWitness,
                validation: AlgebraicRootValidationReport::valid(),
            };

            let report = transform_algebraic_root_affine(
                &represented,
                real(scale),
                real(offset),
                PredicatePolicy,
            );

            prop_assert_eq!(report.status, AlgebraicRootAffineTransformStatus::Transformed);
            let transformed = report.representation.as_ref().unwrap();
            let expected = scale * root + offset;
            prop_assert_eq!(transformed.exact_rational_witness(), Some(&real(expected)));
            prop_assert!(transformed.is_valid());
        }
    }
}
