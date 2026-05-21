//! Exact represented algebraic roots for solver residuals.
//!
//! This module is intentionally a representation layer, not a floating-point
//! approximation layer and not a complete algebraic-number field package. A
//! represented root carries the exact univariate polynomial row and the
//! certified isolating interval produced by [`crate::root_isolation`]. This is
//! the Yap boundary in a small form: construction keeps exact object evidence,
//! while later predicates or candidate replay decide how that evidence may be
//! consumed. See Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997), and Collins and Loos, "Real Zeros of Polynomials,"
//! in *Computer Algebra: Symbolic and Algebraic Computation* (1982).

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
            policy: PredicatePolicy::default(),
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
/// sampling a primitive approximation. This follows Yap's construction/
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
/// Sturm sequence package (C. Sturm, "Mémoire sur la résolution des équations
/// numériques", 1835) in the Collins-Loos real-root isolation style (G. E.
/// Collins and R. Loos, "Real Zeros of Polynomials", 1982) until the
/// intervals become disjoint, an exact rational witness appears, or the
/// configured work budget is exhausted. The function follows Yap's EGC
/// pattern (Chee K. Yap, "Towards Exact Geometric Computation", 1997):
/// refinement is a proof-producing operation, and failure remains an explicit
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
            validate_algebraic_root_representation(root, PredicatePolicy::default()).status
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
            validate_algebraic_root_representation(&invalid_count, PredicatePolicy::default())
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
            kind: AlgebraicRootKind::ExactRationalWitness,
            ..invalid_count
        };
        assert_eq!(
            validate_algebraic_root_representation(&bad_witness, PredicatePolicy::default()).status,
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

        let comparison =
            compare_algebraic_root_representations(&left, &right, PredicatePolicy::default());
        assert_eq!(comparison.status, AlgebraicRootComparisonStatus::Compared);
        assert_eq!(comparison.ordering, Some(Ordering::Less));

        let same = compare_algebraic_root_representations(&left, &left, PredicatePolicy::default());
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

        let comparison = compare_algebraic_root_representations(
            &valid,
            &overlapping,
            PredicatePolicy::default(),
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
            compare_algebraic_root_representations(&invalid, &valid, PredicatePolicy::default())
                .status,
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
                lower: real(1),
                upper: real(2),
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
            compare_algebraic_root_representations(
                &sqrt_two,
                &sqrt_three,
                PredicatePolicy::default()
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
                PredicatePolicy::default(),
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
    }
}
