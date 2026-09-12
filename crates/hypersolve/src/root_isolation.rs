//! Exact univariate algebraic root isolation.
//!
//! This module is a proof package, not a numerical proposal engine. It extracts
//! exact-rational univariate polynomial residuals, reduces repeated factors,
//! and isolates distinct real roots with Sturm sign-variation counts over
//! rational intervals. The boundary remains explicit: algebraic isolation
//! constructs certified exact intervals, while ordinary candidate replay
//! decides whether a solver assignment is acceptable.

use std::cmp::Ordering;
use std::collections::HashMap;

use hyperlimit::{PredicatePolicy, compare_reals, reciprocal_real};
use hyperreal::{Rational as HyperRational, Real};
use num::bigint::Sign;
use num::{BigInt, One, Zero};

use crate::analysis::ProblemAnalysis;
use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::integer_interpolation::{
    primitive_integer_polynomial, primitive_integer_polynomial_gcd,
    primitive_integer_sturm_sequence, rational_polynomial_exact_quotient,
};
use crate::interval::rational_interval_product;
use crate::model::{ConstraintKind, Problem};
use crate::symbolic::{Expr, SymbolId};

pub(crate) const ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS: usize = 8;
pub(crate) const ALGEBRAIC_IMAGE_REFINEMENT_STEPS: usize = 8;

/// Multiplicity evidence found before Sturm isolation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RootMultiplicityStatus {
    /// The input polynomial is square-free.
    SquareFree,
    /// A nonconstant gcd with the derivative was found exactly.
    RepeatedRootsDetected {
        /// Degree of `gcd(p, p')`.
        gcd_degree: usize,
    },
}

/// Status for one univariate root-isolation row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RootIsolationStatus {
    /// Every distinct real root was isolated.
    Isolated,
    /// Roots were isolated after removing repeated factors.
    MultipleRoot,
    /// The row is nonconstant and has no real roots.
    NoRealRoots,
    /// The row is outside the exact-rational univariate package.
    UnsupportedCoefficient,
    /// Exact comparisons or polynomial division did not decide.
    Undecided,
}

/// One exact isolating interval for a distinct real root.
#[derive(Clone, Debug, PartialEq)]
pub struct IsolatedRootInterval {
    /// Lower exact endpoint, excluded by positive-width partitioned proofs.
    pub lower: Real,
    /// Upper exact endpoint, included by positive-width partitioned proofs.
    pub upper: Real,
    /// Exact root value when subdivision or independent exact evidence found it.
    pub exact_root: Option<Real>,
    /// Number of distinct roots certified in the interval.
    pub distinct_root_count: usize,
}

/// One exact, sign-preserving Sturm chain for a univariate polynomial.
///
/// Rational inputs use the shared fraction-free primitive-integer kernel;
/// exact nonrational coefficients retain exact field division. The internal
/// polynomials stay opaque so callers share root-count evidence instead of
/// rebuilding or interpreting remainder infrastructure.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateSturmSequence {
    polynomials: Vec<Vec<Real>>,
}

/// Exact evidence at one point of a univariate Sturm chain.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UnivariateSturmPoint {
    /// The source polynomial vanishes at the point.
    Root,
    /// The source polynomial is nonzero and the chain has this many sign
    /// variations after zero-valued remainder members are omitted.
    NonRoot(usize),
}

impl UnivariateSturmSequence {
    /// Constructs the complete exact chain, or returns `None` when a required
    /// coefficient sign or division cannot be certified under `policy`.
    pub fn new(polynomial: &[Real], policy: PredicatePolicy) -> Option<Self> {
        if polynomial.is_empty() {
            return None;
        }
        let polynomials = primitive_integer_sturm_sequence(polynomial)
            .or_else(|| field_sturm_sequence(polynomial, policy))?;
        if polynomials.first()?.len() == 1
            && compare_reals(&polynomials[0][0], &Real::zero(), policy).value()? == Ordering::Equal
        {
            return None;
        }
        Some(Self { polynomials })
    }

    /// Returns the terminal nonzero remainder. For a chain of `p` and `p'`,
    /// this is their GCD up to a nonzero constant scale.
    pub fn terminal_polynomial(&self) -> &[Real] {
        self.polynomials
            .last()
            .expect("a Sturm chain retains its source polynomial")
    }

    /// Classifies one exact point and returns its sign-variation count when it
    /// is not a root of the source polynomial.
    pub fn classify_point(
        &self,
        point: &Real,
        policy: PredicatePolicy,
    ) -> Option<UnivariateSturmPoint> {
        let evaluation = evaluate_sturm_at(self, point, policy)?;
        Some(if evaluation.polynomial_sign == Ordering::Equal {
            UnivariateSturmPoint::Root
        } else {
            UnivariateSturmPoint::NonRoot(evaluation.variations)
        })
    }

    /// Counts distinct roots in the Sturm-owned half-open interval
    /// `(lower, upper]`.
    pub fn count_distinct_roots(
        &self,
        lower: &Real,
        upper: &Real,
        policy: PredicatePolicy,
    ) -> Option<usize> {
        sturm_count(self, lower, upper, policy)
    }
}

/// Report for isolating roots of one active univariate equality residual.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateRootIsolationReport {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol used by the univariate polynomial, if extraction reached
    /// symbol discovery.
    pub symbol: Option<SymbolId>,
    /// Degree of the extracted polynomial after trimming structural zeros.
    pub degree: Option<usize>,
    /// Final isolation status.
    pub status: RootIsolationStatus,
    /// Multiplicity evidence for supported nonconstant rows.
    pub multiplicity: Option<RootMultiplicityStatus>,
    /// Exact isolating intervals for distinct real roots.
    pub intervals: Vec<IsolatedRootInterval>,
    /// Compact unsupported/undecided reason for diagnostics.
    pub message: Option<String>,
}

/// Bounded refinement controls for univariate root isolation.
///
/// These controls do not introduce a tolerance acceptance rule. They only tell
/// the exact Sturm isolator how far to subdivide intervals that already have a
/// certified distinct-root count. Acceptance still belongs to exact candidate
/// replay or to a future algebraic-number package, preserving the exact
/// construction/proof boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct RootIsolationConfig {
    /// Exact comparison/refinement policy used by `hyperlimit`.
    pub policy: PredicatePolicy,
    /// Optional exact maximum width for non-point isolating intervals. A
    /// negative target is unreachable, so `max_refinement_steps` remains the
    /// hard termination bound.
    pub max_interval_width: Option<Real>,
    /// Maximum additional bisection steps once an interval has one certified
    /// root. This bounds work for clustered roots.
    pub max_refinement_steps: usize,
}

impl Default for RootIsolationConfig {
    fn default() -> Self {
        Self {
            policy: PredicatePolicy::APPROXIMATE_512,
            max_interval_width: None,
            max_refinement_steps: 0,
        }
    }
}

/// Status for refining one already-isolated algebraic root interval.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum IsolatedRootRefinementStatus {
    /// The interval was validated and optionally refined while preserving one root.
    Refined,
    /// An exact root witness was strictly replayed against the polynomial.
    ExactRoot,
    /// The coefficient vector is empty or constant.
    InvalidPolynomial,
    /// The interval endpoints are not ordered.
    InvalidInterval,
    /// The supplied interval did not certify exactly one distinct root.
    NonUnitIsolation,
    /// Exact comparisons, polynomial division, or Sturm counts did not decide.
    Undecided,
}

/// Refinement report for one isolated algebraic root interval.
///
/// The input interval is treated as exact evidence, not as a floating estimate.
/// A degree/sign certificate handles complete linear and quadratic cases;
/// otherwise the implementation recomputes the square-free Sturm count on the
/// partition-owned `(lower, upper]` interval. It then bisects only into the
/// subinterval that still contains the unique simple root. An explicit exact
/// witness owns its point and is checked against both stored bounds. This
/// follows Sturm's theorem, the Collins-Loos isolation model, and the
/// exact-object refinement boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct IsolatedRootRefinementReport {
    /// Final refinement status.
    pub status: IsolatedRootRefinementStatus,
    /// Original interval supplied by the caller.
    pub original_interval: IsolatedRootInterval,
    /// Refined interval when refinement or exact-root replay succeeded.
    pub refined_interval: Option<IsolatedRootInterval>,
    /// Number of bisection steps accepted.
    pub refinement_steps: usize,
    /// Compact diagnostic reason for invalid or undecided reports.
    pub message: Option<String>,
}

/// Replay status for an exact rational root witness found by isolation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootCandidateStatus {
    /// The isolating interval did not land on an exact rational root witness.
    NoExactRationalWitness,
    /// The exact rational witness was replayed and certified against all
    /// active residuals.
    ReplayCertified,
    /// The exact rational witness replayed but did not satisfy every active
    /// residual.
    ReplayRejected,
}

/// Candidate replay report for one isolated root interval.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootCandidateReport {
    /// Source constraint index from the isolation report.
    pub constraint_index: usize,
    /// Solver symbol bound for replay.
    pub symbol: Option<SymbolId>,
    /// Root interval ordinal within the isolation report.
    pub interval_index: usize,
    /// Exact rational root witness, when one exists.
    pub exact_root: Option<Real>,
    /// Full candidate certification report for exact rational witnesses.
    pub certification: Option<CandidateCertificationReport>,
    /// Replay status.
    pub status: AlgebraicRootCandidateStatus,
}

/// Status for Descartes-sign root-count bounds.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DescartesRootCountStatus {
    /// Descartes variation bounds were computed exactly.
    Counted,
    /// The expression is outside the exact-rational univariate package.
    UnsupportedCoefficient,
    /// Exact coefficient signs or degree trimming did not decide.
    Undecided,
}

/// Descartes sign-variation bounds for one univariate polynomial row.
///
/// Descartes' rule of signs gives an exact upper bound on positive real roots
/// and the parity of the gap to the true count. Applying the same rule to
/// `p(-x)` gives the negative-root bound. This is not full isolation, but it
/// is a cheap proof-producing algebraic filter that can reject impossible
/// root topologies before Sturm subdivision, using Descartes' rule of signs and
/// the standard real-root isolation model.
#[derive(Clone, Debug, PartialEq)]
pub struct DescartesRootCountReport {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol used by the univariate polynomial, when supported.
    pub symbol: Option<SymbolId>,
    /// Degree after exact trimming.
    pub degree: Option<usize>,
    /// Count status.
    pub status: DescartesRootCountStatus,
    /// Multiplicity of the root at zero, detected from leading zero
    /// coefficients. This is exact for supported rows.
    pub zero_root_multiplicity: Option<usize>,
    /// Descartes upper bound for positive roots.
    pub positive_variations: Option<usize>,
    /// Descartes upper bound for negative roots, computed from `p(-x)`.
    pub negative_variations: Option<usize>,
    /// True positive-root count has the same parity as this value.
    pub positive_root_count_parity: Option<usize>,
    /// True negative-root count has the same parity as this value.
    pub negative_root_count_parity: Option<usize>,
    /// Compact unsupported/undecided reason.
    pub message: Option<String>,
}

/// Status for Bernstein interval root-count bounds.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BernsteinRootCountStatus {
    /// Bernstein variation bounds were computed exactly.
    Counted,
    /// The interval is invalid, usually because `lower >= upper`.
    InvalidInterval,
    /// The expression is outside the exact-rational univariate package.
    UnsupportedCoefficient,
    /// Exact signs, endpoint comparisons, or coefficient conversion did not decide.
    Undecided,
}

/// Bernstein sign-variation bounds over one exact interval.
///
/// The polynomial is transformed from the power basis on `[lower, upper]` into
/// Bernstein form and Descartes sign variation is applied to the Bernstein
/// control coefficients. This gives a proof-producing bound for roots inside a
/// finite interval and is the standard subdivision-facing sibling of the
/// global Descartes count, using Bernstein-form root counting and the standard
/// real-root isolation model.
#[derive(Clone, Debug, PartialEq)]
pub struct BernsteinRootCountReport {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol used by the univariate polynomial, when supported.
    pub symbol: Option<SymbolId>,
    /// Degree after exact trimming.
    pub degree: Option<usize>,
    /// Interval lower endpoint.
    pub lower: Real,
    /// Interval upper endpoint.
    pub upper: Real,
    /// Count status.
    pub status: BernsteinRootCountStatus,
    /// Exact Bernstein coefficients over `[lower, upper]`, when supported.
    pub bernstein_coefficients: Vec<Real>,
    /// Bernstein sign-variation upper bound for roots in the interval.
    pub variation_bound: Option<usize>,
    /// True interval-root count has the same parity as this value.
    pub root_count_parity: Option<usize>,
    /// Whether `p(lower) == 0` was certified.
    pub root_at_lower: Option<bool>,
    /// Whether `p(upper) == 0` was certified.
    pub root_at_upper: Option<bool>,
    /// Compact unsupported/undecided reason.
    pub message: Option<String>,
}

/// Terminal status for one interval in recursive Bernstein subdivision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BernsteinSubdivisionIntervalStatus {
    /// The interval has no certified root evidence.
    Empty,
    /// A root was certified exactly at an interval endpoint.
    EndpointRoot,
    /// The interval has Bernstein variation one and therefore isolates at most
    /// one root. Endpoint roots are reported separately.
    Isolating,
    /// The interval still has multiple-root variation, or an interior root has
    /// not yet been separated from an endpoint root, at the configured depth.
    DepthLimit,
}

/// One terminal interval from recursive Bernstein subdivision.
#[derive(Clone, Debug, PartialEq)]
pub struct BernsteinSubdivisionInterval {
    /// Lower endpoint.
    pub lower: Real,
    /// Upper endpoint.
    pub upper: Real,
    /// Exact root witness for endpoint roots.
    pub exact_root: Option<Real>,
    /// Bernstein variation bound for this terminal interval.
    pub variation_bound: Option<usize>,
    /// Terminal interval status.
    pub status: BernsteinSubdivisionIntervalStatus,
}

/// Status for recursive Bernstein subdivision of one row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BernsteinSubdivisionStatus {
    /// Subdivision completed within the configured depth.
    Completed,
    /// At least one interval still had multiple possible roots, or had not yet
    /// separated interior root evidence from an endpoint root, at the depth
    /// limit.
    DepthLimit,
    /// The input interval was invalid.
    InvalidInterval,
    /// The expression is outside the exact-rational univariate package.
    UnsupportedCoefficient,
    /// Exact signs, endpoint comparisons, or coefficient conversion did not decide.
    Undecided,
}

/// Configuration for recursive Bernstein subdivision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BernsteinSubdivisionConfig {
    /// Exact comparison policy used by `hyperlimit`.
    pub policy: PredicatePolicy,
    /// Maximum recursive bisection depth.
    pub max_depth: usize,
}

impl Default for BernsteinSubdivisionConfig {
    fn default() -> Self {
        Self {
            policy: PredicatePolicy::APPROXIMATE_512,
            max_depth: 32,
        }
    }
}

/// Recursive Bernstein subdivision report for one univariate row.
///
/// This report is the subdivision-facing companion to
/// [`BernsteinRootCountReport`]. It repeatedly bisects intervals whose
/// Bernstein sign variation is greater than one and stops only when intervals
/// are empty, have an exact endpoint root, have variation one, or reach the
/// configured depth limit. It follows the Bernstein subdivision literature of
/// the Bernstein-form construction while preserving the exact proof boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct BernsteinSubdivisionReport {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Solver symbol used by the univariate polynomial, when supported.
    pub symbol: Option<SymbolId>,
    /// Degree after exact trimming.
    pub degree: Option<usize>,
    /// Initial lower endpoint.
    pub lower: Real,
    /// Initial upper endpoint.
    pub upper: Real,
    /// Final subdivision status.
    pub status: BernsteinSubdivisionStatus,
    /// Terminal intervals and exact endpoint roots.
    pub intervals: Vec<BernsteinSubdivisionInterval>,
    /// Compact unsupported/undecided reason.
    pub message: Option<String>,
}

/// Isolate distinct real roots for active univariate equality residuals.
///
/// The first implementation accepts exact-rational univariate polynomial rows
/// collected from the source expression tree. It deliberately rejects
/// non-equality, multivariate, transcendental, and non-exact-rational rows
/// instead of hiding them behind primitive floating-point estimates.
pub fn isolate_univariate_polynomial_roots(
    analysis: &ProblemAnalysis<'_>,
    policy: PredicatePolicy,
) -> Vec<UnivariateRootIsolationReport> {
    isolate_univariate_polynomial_roots_with_config(
        analysis,
        RootIsolationConfig {
            policy,
            ..RootIsolationConfig::default()
        },
    )
}

/// Isolate distinct real roots with explicit bounded-refinement controls.
pub fn isolate_univariate_polynomial_roots_with_config(
    analysis: &ProblemAnalysis<'_>,
    config: RootIsolationConfig,
) -> Vec<UnivariateRootIsolationReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in analysis.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(isolate_univariate_polynomial_expr_with_config(
            constraint_index,
            &constraint.residual,
            analysis.problem(),
            config.clone(),
        ));
    }
    reports
}

/// Isolate distinct real roots for one univariate polynomial expression.
///
/// This lower-level entry point is useful for tests and domain builders that
/// want the exact algebraic package without constructing a full solver pass.
pub fn isolate_univariate_polynomial_expr(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    policy: PredicatePolicy,
) -> UnivariateRootIsolationReport {
    isolate_univariate_polynomial_expr_with_config(
        constraint_index,
        expression,
        problem,
        RootIsolationConfig {
            policy,
            ..RootIsolationConfig::default()
        },
    )
}

/// Isolate distinct real roots for one expression with refinement controls.
pub fn isolate_univariate_polynomial_expr_with_config(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    config: RootIsolationConfig,
) -> UnivariateRootIsolationReport {
    let policy = config.policy;
    let extracted = match collect_univariate_polynomial(expression) {
        Some(extracted) => extracted,
        None => {
            return root_isolation_report(
                constraint_index,
                None,
                None,
                RootIsolationStatus::UnsupportedCoefficient,
                None,
                Vec::new(),
                Some("expression is not a supported univariate polynomial".to_owned()),
            );
        }
    };
    let Some(symbol) = extracted.symbol else {
        return root_isolation_report(
            constraint_index,
            None,
            Some(0),
            RootIsolationStatus::NoRealRoots,
            Some(RootMultiplicityStatus::SquareFree),
            Vec::new(),
            Some("constant polynomial row has no isolated variable roots".to_owned()),
        );
    };
    if !problem
        .variables
        .iter()
        .any(|variable| variable.symbol == symbol)
    {
        return root_isolation_report(
            constraint_index,
            Some(symbol),
            None,
            RootIsolationStatus::UnsupportedCoefficient,
            None,
            Vec::new(),
            Some("polynomial symbol is not present in the problem".to_owned()),
        );
    }
    let Some(poly) = trim_polynomial(extracted.coefficients, policy) else {
        return root_isolation_report(
            constraint_index,
            Some(symbol),
            None,
            RootIsolationStatus::Undecided,
            None,
            Vec::new(),
            Some("could not decide polynomial degree".to_owned()),
        );
    };
    let degree = poly.len().saturating_sub(1);
    if degree == 0 {
        return root_isolation_report(
            constraint_index,
            Some(symbol),
            Some(degree),
            RootIsolationStatus::NoRealRoots,
            Some(RootMultiplicityStatus::SquareFree),
            Vec::new(),
            Some("constant polynomial row has no isolated variable roots".to_owned()),
        );
    }
    if poly
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return root_isolation_report(
            constraint_index,
            Some(symbol),
            Some(degree),
            RootIsolationStatus::UnsupportedCoefficient,
            None,
            Vec::new(),
            Some("all root-isolation coefficients must be exact rationals".to_owned()),
        );
    }

    let derivative = derivative(&poly);
    let gcd = match polynomial_gcd(poly.clone(), derivative, policy) {
        Some(gcd) => gcd,
        None => {
            return root_isolation_report(
                constraint_index,
                Some(symbol),
                Some(degree),
                RootIsolationStatus::Undecided,
                None,
                Vec::new(),
                Some("polynomial gcd was undecided".to_owned()),
            );
        }
    };
    let gcd_degree = gcd.len().saturating_sub(1);
    let multiplicity = if gcd_degree > 0 {
        RootMultiplicityStatus::RepeatedRootsDetected { gcd_degree }
    } else {
        RootMultiplicityStatus::SquareFree
    };
    let square_free = if gcd_degree > 0 {
        match polynomial_div_rem_trimmed(poly, &gcd, policy).and_then(|(quotient, remainder)| {
            trimmed_polynomial_is_zero(&remainder, policy)?.then_some(quotient)
        }) {
            Some(square_free) => square_free,
            None => {
                return root_isolation_report(
                    constraint_index,
                    Some(symbol),
                    Some(degree),
                    RootIsolationStatus::Undecided,
                    Some(multiplicity),
                    Vec::new(),
                    Some("square-free quotient was undecided".to_owned()),
                );
            }
        }
    } else {
        // The original polynomial is already square-free.
        trim_polynomial(
            collect_univariate_polynomial(expression)
                .expect("already collected")
                .coefficients,
            policy,
        )
        .expect("already trimmed")
    };

    let intervals = match isolate_square_free_roots(&square_free, &config) {
        Some(intervals) => intervals,
        None => {
            return root_isolation_report(
                constraint_index,
                Some(symbol),
                Some(degree),
                RootIsolationStatus::Undecided,
                Some(multiplicity),
                Vec::new(),
                Some("Sturm isolation did not decide".to_owned()),
            );
        }
    };
    let status = if intervals.is_empty() {
        RootIsolationStatus::NoRealRoots
    } else if matches!(
        multiplicity,
        RootMultiplicityStatus::RepeatedRootsDetected { .. }
    ) {
        RootIsolationStatus::MultipleRoot
    } else {
        RootIsolationStatus::Isolated
    };
    root_isolation_report(
        constraint_index,
        Some(symbol),
        Some(degree),
        status,
        Some(multiplicity),
        intervals,
        None,
    )
}

/// Refine one exact isolating interval for a univariate polynomial.
///
/// This is the low-level refinement hook for represented algebraic numbers.
/// It does not approximate a root. It validates an exact coefficient-field
/// polynomial, uses degree and endpoint signs when those already prove a
/// unique root, and otherwise removes repeated factors with a polynomial gcd
/// and verifies the supplied partition-owned `(lower, upper]` interval with a
/// Sturm count. It then repeatedly bisects the proved one-root interval. Any
/// supplied exact witness is checked for closed containment, vanishing, and
/// interval uniqueness before it is accepted.
/// The algorithm uses Sturm's theorem and the standard real-root isolation
/// model; exact refinement remains the design rule used here.
pub fn refine_isolated_univariate_polynomial_interval(
    polynomial: &[Real],
    interval: &IsolatedRootInterval,
    config: RootIsolationConfig,
) -> IsolatedRootRefinementReport {
    let policy = config.policy;
    if interval.distinct_root_count != 1 {
        return root_refinement_report(
            IsolatedRootRefinementStatus::NonUnitIsolation,
            interval.clone(),
            None,
            0,
            Some("refinement requires an interval with exactly one distinct root".to_owned()),
        );
    }
    let endpoint_ordering = match compare_reals(&interval.lower, &interval.upper, policy).value() {
        Some(ordering @ (Ordering::Less | Ordering::Equal)) => ordering,
        Some(Ordering::Greater) => {
            return root_refinement_report(
                IsolatedRootRefinementStatus::InvalidInterval,
                interval.clone(),
                None,
                0,
                Some("refinement interval endpoints are reversed".to_owned()),
            );
        }
        None => {
            return root_refinement_report(
                IsolatedRootRefinementStatus::Undecided,
                interval.clone(),
                None,
                0,
                Some("could not compare refinement interval endpoints".to_owned()),
            );
        }
    };
    if endpoint_ordering == Ordering::Equal && interval.exact_root.is_none() {
        return root_refinement_report(
            IsolatedRootRefinementStatus::InvalidInterval,
            interval.clone(),
            None,
            0,
            Some("a collapsed refinement interval requires an exact witness".to_owned()),
        );
    }
    let Some(trimmed) = trim_polynomial_slice(polynomial, policy) else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not decide polynomial degree".to_owned()),
        );
    };
    if trimmed.len() <= 1 {
        return root_refinement_report(
            IsolatedRootRefinementStatus::InvalidPolynomial,
            interval.clone(),
            None,
            0,
            Some("refinement requires a nonconstant exact polynomial".to_owned()),
        );
    }
    if let Some(root) = &interval.exact_root {
        let lower_ordering = compare_reals(root, &interval.lower, policy).value();
        let upper_ordering = compare_reals(root, &interval.upper, policy).value();
        match (lower_ordering, upper_ordering) {
            (Some(Ordering::Less), _) | (_, Some(Ordering::Greater)) => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::InvalidInterval,
                    interval.clone(),
                    None,
                    0,
                    Some("exact root witness lies outside the refinement interval".to_owned()),
                );
            }
            (Some(Ordering::Equal | Ordering::Greater), Some(Ordering::Less | Ordering::Equal)) => {
            }
            _ => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::Undecided,
                    interval.clone(),
                    None,
                    0,
                    Some("could not prove exact root witness containment".to_owned()),
                );
            }
        }
        match sign_at(trimmed, root, policy) {
            Some(Ordering::Equal) => {}
            Some(Ordering::Less | Ordering::Greater) => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::NonUnitIsolation,
                    interval.clone(),
                    None,
                    0,
                    Some("exact root witness does not satisfy the polynomial".to_owned()),
                );
            }
            None => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::Undecided,
                    interval.clone(),
                    None,
                    0,
                    Some("could not replay the exact root witness".to_owned()),
                );
            }
        }
        // A vanishing singleton contains exactly one distinct root. A valid
        // linear polynomial has exactly one root globally, so containment is
        // already a complete uniqueness proof in that case.
        if endpoint_ordering == Ordering::Equal || trimmed.len() == 2 {
            return exact_root_refinement_report(interval, root.clone(), 0);
        }
    }
    if trimmed.len() == 2
        && let Ok(inverse) = crate::policy_division::reciprocal_after_certified_nonzero(&trimmed[1])
    {
        let root = -(&trimmed[0] * inverse);
        let lower_ordering = compare_reals(&root, &interval.lower, policy).value();
        let upper_ordering = compare_reals(&root, &interval.upper, policy).value();
        match (lower_ordering, upper_ordering) {
            (Some(Ordering::Less | Ordering::Equal), _) | (_, Some(Ordering::Greater)) => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::NonUnitIsolation,
                    interval.clone(),
                    None,
                    0,
                    Some(
                        "linear polynomial root lies outside the owned half-open interval"
                            .to_owned(),
                    ),
                );
            }
            (Some(Ordering::Greater), Some(Ordering::Less | Ordering::Equal)) => {
                return exact_root_refinement_report(interval, root, 0);
            }
            _ => {}
        }
    }
    if trimmed.len() <= 3 {
        let endpoint_signs = sign_at(trimmed, &interval.lower, policy).zip(sign_at(
            trimmed,
            &interval.upper,
            policy,
        ));
        if let Some((lower_sign, upper_sign)) = endpoint_signs {
            if lower_sign != Ordering::Equal
                && upper_sign != Ordering::Equal
                && lower_sign != upper_sign
            {
                // A sign-changing nonconstant polynomial of degree at most two
                // has exactly one root in the open interval. It cannot also
                // contain a double root or a second simple root within its
                // degree budget.
                if let Some(root) = &interval.exact_root {
                    return exact_root_refinement_report(interval, root.clone(), 0);
                }
                return refine_owned_one_root_interval(trimmed, interval, config, upper_sign);
            }
            if trimmed.len() == 2 {
                return match (lower_sign, upper_sign) {
                    (Ordering::Less | Ordering::Greater, Ordering::Equal) => {
                        exact_root_refinement_report(interval, interval.upper.clone(), 0)
                    }
                    _ => root_refinement_report(
                        IsolatedRootRefinementStatus::NonUnitIsolation,
                        interval.clone(),
                        None,
                        0,
                        Some("linear polynomial has no unique root in the interval".to_owned()),
                    ),
                };
            }
        }
    }
    let Some(square_free) = square_free_part(trimmed.to_vec(), policy) else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not compute square-free polynomial part".to_owned()),
        );
    };
    let Some(sturm) = UnivariateSturmSequence::new(&square_free, policy) else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not build Sturm sequence for refinement".to_owned()),
        );
    };
    let Some(lower_evaluation) = evaluate_sturm_at(&sturm, &interval.lower, policy) else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not evaluate the refinement interval's lower endpoint".to_owned()),
        );
    };
    let Some(upper_evaluation) = evaluate_sturm_at(&sturm, &interval.upper, policy) else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not evaluate the refinement interval's upper endpoint".to_owned()),
        );
    };
    let Some(mut root_count) = lower_evaluation
        .variations
        .checked_sub(upper_evaluation.variations)
    else {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("could not count roots in the half-open refinement interval".to_owned()),
        );
    };
    if interval.exact_root.is_some() && lower_evaluation.polynomial_sign == Ordering::Equal {
        let Some(closed_root_count) = root_count.checked_add(1) else {
            return root_refinement_report(
                IsolatedRootRefinementStatus::Undecided,
                interval.clone(),
                None,
                0,
                Some("could not include the witnessed lower endpoint in the root count".to_owned()),
            );
        };
        root_count = closed_root_count;
    }
    if root_count != 1 {
        return root_refinement_report(
            IsolatedRootRefinementStatus::NonUnitIsolation,
            interval.clone(),
            None,
            0,
            Some("Sturm count did not confirm exactly one owned root in interval".to_owned()),
        );
    }
    if let Some(root) = &interval.exact_root {
        return exact_root_refinement_report(interval, root.clone(), 0);
    }
    if upper_evaluation.polynomial_sign == Ordering::Equal {
        return exact_root_refinement_report(interval, interval.upper.clone(), 0);
    }
    if lower_evaluation.polynomial_sign != Ordering::Equal
        && lower_evaluation.polynomial_sign == upper_evaluation.polynomial_sign
    {
        return root_refinement_report(
            IsolatedRootRefinementStatus::Undecided,
            interval.clone(),
            None,
            0,
            Some("square-free endpoint signs did not bracket the unique root".to_owned()),
        );
    }

    refine_owned_one_root_interval(
        &sturm.polynomials[0],
        interval,
        config,
        upper_evaluation.polynomial_sign,
    )
}

// A square-free polynomial with exactly one root in `(lower, upper]` changes
// sign exactly once before its non-root upper endpoint. Comparing each
// midpoint only with that stable upper sign therefore selects the owned child
// even when the excluded lower endpoint is itself another root.
fn refine_owned_one_root_interval(
    polynomial: &[Real],
    interval: &IsolatedRootInterval,
    config: RootIsolationConfig,
    upper_sign: Ordering,
) -> IsolatedRootRefinementReport {
    let policy = config.policy;
    let mut lower = interval.lower.clone();
    let mut upper = interval.upper.clone();
    let mut steps = 0;
    for _ in 0..config.max_refinement_steps {
        if let Some(max_width) = &config.max_interval_width {
            let width = &upper - &lower;
            match compare_reals(&width, max_width, policy).value() {
                Some(Ordering::Less | Ordering::Equal) => break,
                Some(Ordering::Greater) => {}
                None => {
                    return root_refinement_report(
                        IsolatedRootRefinementStatus::Undecided,
                        interval.clone(),
                        None,
                        steps,
                        Some("could not compare refined interval width".to_owned()),
                    );
                }
            }
        }
        let midpoint = Real::average_pair(&lower, &upper);
        match sign_at(polynomial, &midpoint, policy) {
            Some(Ordering::Equal) => {
                return exact_root_refinement_report(interval, midpoint, steps + 1);
            }
            Some(sign) if sign == upper_sign => upper = midpoint,
            Some(Ordering::Less | Ordering::Greater) => lower = midpoint,
            None => {
                return root_refinement_report(
                    IsolatedRootRefinementStatus::Undecided,
                    interval.clone(),
                    None,
                    steps,
                    Some("could not evaluate polynomial at refinement midpoint".to_owned()),
                );
            }
        }
        steps += 1;
    }

    root_refinement_report(
        IsolatedRootRefinementStatus::Refined,
        interval.clone(),
        Some(IsolatedRootInterval {
            lower,
            upper,
            exact_root: None,
            distinct_root_count: 1,
        }),
        steps,
        None,
    )
}

/// Replay exact rational root witnesses produced by isolation.
///
/// Sturm isolation usually returns intervals, not concrete algebraic numbers.
/// When subdivision lands on a rational root exactly, this helper binds that
/// witness into a cloned candidate context and replays the full analysis
/// problem. Non-rational intervals are reported as explicit non-witnesses
/// rather than approximated. This follows the standard real-root isolation model and
/// the exact-decision rule that constructed algebraic evidence still needs exact replay
/// before becoming a solver decision.
pub fn certify_isolated_rational_root_witnesses(
    analysis: &ProblemAnalysis<'_>,
    base_context: &EvaluationContext,
    reports: &[UnivariateRootIsolationReport],
) -> Vec<AlgebraicRootCandidateReport> {
    certify_isolated_rational_root_witnesses_with_config(
        analysis,
        base_context,
        reports,
        CandidateCertificationConfig::default(),
    )
}

/// Replay exact rational root witnesses with a candidate-certification policy.
pub fn certify_isolated_rational_root_witnesses_with_config(
    analysis: &ProblemAnalysis<'_>,
    base_context: &EvaluationContext,
    reports: &[UnivariateRootIsolationReport],
    certification_config: CandidateCertificationConfig,
) -> Vec<AlgebraicRootCandidateReport> {
    let mut candidates = Vec::new();
    for report in reports {
        for (interval_index, interval) in report.intervals.iter().enumerate() {
            let Some(root) = interval.exact_root.clone() else {
                candidates.push(AlgebraicRootCandidateReport {
                    constraint_index: report.constraint_index,
                    symbol: report.symbol,
                    interval_index,
                    exact_root: None,
                    certification: None,
                    status: AlgebraicRootCandidateStatus::NoExactRationalWitness,
                });
                continue;
            };
            let Some(symbol) = report.symbol else {
                candidates.push(AlgebraicRootCandidateReport {
                    constraint_index: report.constraint_index,
                    symbol: None,
                    interval_index,
                    exact_root: Some(root),
                    certification: None,
                    status: AlgebraicRootCandidateStatus::NoExactRationalWitness,
                });
                continue;
            };
            let mut candidate = base_context.clone();
            candidate.bind(symbol, root.clone());
            let certification =
                certify_candidate_with_config(analysis, &candidate, certification_config);
            let status = if certification.all_satisfied() {
                AlgebraicRootCandidateStatus::ReplayCertified
            } else {
                AlgebraicRootCandidateStatus::ReplayRejected
            };
            candidates.push(AlgebraicRootCandidateReport {
                constraint_index: report.constraint_index,
                symbol: Some(symbol),
                interval_index,
                exact_root: Some(root),
                certification: Some(certification),
                status,
            });
        }
    }
    candidates
}

/// Compute Descartes sign-variation root-count bounds for active equality rows.
///
/// This is a bounded algebraic prefilter. It reports exact upper bounds and
/// parity constraints for positive and negative real roots of supported
/// exact-rational univariate polynomials. It deliberately does not estimate
/// roots numerically or certify a candidate solution.
pub fn count_descartes_univariate_polynomial_roots(
    analysis: &ProblemAnalysis<'_>,
    policy: PredicatePolicy,
) -> Vec<DescartesRootCountReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in analysis.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(count_descartes_univariate_polynomial_expr(
            constraint_index,
            &constraint.residual,
            analysis.problem(),
            policy,
        ));
    }
    reports
}

/// Compute Descartes sign-variation bounds for one expression.
pub fn count_descartes_univariate_polynomial_expr(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    policy: PredicatePolicy,
) -> DescartesRootCountReport {
    let extracted = match collect_univariate_polynomial(expression) {
        Some(extracted) => extracted,
        None => {
            return descartes_report(
                constraint_index,
                None,
                None,
                DescartesRootCountStatus::UnsupportedCoefficient,
                None,
                None,
                None,
                Some("expression is not a supported univariate polynomial".to_owned()),
            );
        }
    };
    let Some(symbol) = extracted.symbol else {
        return descartes_report(
            constraint_index,
            None,
            Some(0),
            DescartesRootCountStatus::Counted,
            Some(0),
            Some(0),
            Some(0),
            Some("constant polynomial row has no variable roots".to_owned()),
        );
    };
    if !problem
        .variables
        .iter()
        .any(|variable| variable.symbol == symbol)
    {
        return descartes_report(
            constraint_index,
            Some(symbol),
            None,
            DescartesRootCountStatus::UnsupportedCoefficient,
            None,
            None,
            None,
            Some("polynomial symbol is not present in the problem".to_owned()),
        );
    }

    let zero_root_multiplicity = match leading_zero_multiplicity(&extracted.coefficients, policy) {
        Some(multiplicity) => multiplicity,
        None => {
            return descartes_report(
                constraint_index,
                Some(symbol),
                None,
                DescartesRootCountStatus::Undecided,
                None,
                None,
                None,
                Some("could not decide zero-root multiplicity".to_owned()),
            );
        }
    };
    let Some(poly) = trim_polynomial(extracted.coefficients, policy) else {
        return descartes_report(
            constraint_index,
            Some(symbol),
            None,
            DescartesRootCountStatus::Undecided,
            Some(zero_root_multiplicity),
            None,
            None,
            Some("could not decide polynomial degree".to_owned()),
        );
    };
    if poly
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return descartes_report(
            constraint_index,
            Some(symbol),
            Some(poly.len().saturating_sub(1)),
            DescartesRootCountStatus::UnsupportedCoefficient,
            Some(zero_root_multiplicity),
            None,
            None,
            Some("all Descartes coefficients must be exact rationals".to_owned()),
        );
    }
    let positive = match sign_variations_for_coefficients(&poly, policy) {
        Some(variations) => variations,
        None => {
            return descartes_report(
                constraint_index,
                Some(symbol),
                Some(poly.len().saturating_sub(1)),
                DescartesRootCountStatus::Undecided,
                Some(zero_root_multiplicity),
                None,
                None,
                Some("could not decide positive coefficient signs".to_owned()),
            );
        }
    };
    let mut reflected = poly.clone();
    for (degree, coefficient) in reflected.iter_mut().enumerate() {
        if degree % 2 == 1 {
            *coefficient = -coefficient.clone();
        }
    }
    let negative = match sign_variations_for_coefficients(&reflected, policy) {
        Some(variations) => variations,
        None => {
            return descartes_report(
                constraint_index,
                Some(symbol),
                Some(poly.len().saturating_sub(1)),
                DescartesRootCountStatus::Undecided,
                Some(zero_root_multiplicity),
                Some(positive),
                None,
                Some("could not decide negative coefficient signs".to_owned()),
            );
        }
    };
    descartes_report(
        constraint_index,
        Some(symbol),
        Some(poly.len().saturating_sub(1)),
        DescartesRootCountStatus::Counted,
        Some(zero_root_multiplicity),
        Some(positive),
        Some(negative),
        None,
    )
}

/// Compute Bernstein interval root-count bounds for active equality rows.
///
/// Each supported row is converted exactly to Bernstein form over the supplied
/// interval. The sign variation of the Bernstein coefficients is an exact upper
/// bound on the number of roots in the interval, with the same parity as the
/// true count. Endpoint roots are reported separately so callers can keep open
/// and closed interval policy outside this algebraic filter.
pub fn count_bernstein_univariate_polynomial_interval_roots(
    analysis: &ProblemAnalysis<'_>,
    lower: Real,
    upper: Real,
    policy: PredicatePolicy,
) -> Vec<BernsteinRootCountReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in analysis.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(count_bernstein_univariate_polynomial_interval_expr(
            constraint_index,
            &constraint.residual,
            analysis.problem(),
            lower.clone(),
            upper.clone(),
            policy,
        ));
    }
    reports
}

/// Compute a Bernstein interval root-count bound for one expression.
pub fn count_bernstein_univariate_polynomial_interval_expr(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    lower: Real,
    upper: Real,
    policy: PredicatePolicy,
) -> BernsteinRootCountReport {
    match compare_reals(&lower, &upper, policy).value() {
        Some(Ordering::Less) => {}
        Some(Ordering::Equal | Ordering::Greater) => {
            return bernstein_report(
                constraint_index,
                None,
                None,
                lower,
                upper,
                BernsteinRootCountStatus::InvalidInterval,
                Vec::new(),
                None,
                None,
                None,
                Some("Bernstein interval requires lower < upper".to_owned()),
            );
        }
        None => {
            return bernstein_report(
                constraint_index,
                None,
                None,
                lower,
                upper,
                BernsteinRootCountStatus::Undecided,
                Vec::new(),
                None,
                None,
                None,
                Some("could not compare Bernstein interval endpoints".to_owned()),
            );
        }
    }

    let extracted = match collect_univariate_polynomial(expression) {
        Some(extracted) => extracted,
        None => {
            return bernstein_report(
                constraint_index,
                None,
                None,
                lower,
                upper,
                BernsteinRootCountStatus::UnsupportedCoefficient,
                Vec::new(),
                None,
                None,
                None,
                Some("expression is not a supported univariate polynomial".to_owned()),
            );
        }
    };
    let Some(symbol) = extracted.symbol else {
        return bernstein_report(
            constraint_index,
            None,
            Some(0),
            lower,
            upper,
            BernsteinRootCountStatus::Counted,
            vec![
                extracted
                    .coefficients
                    .first()
                    .cloned()
                    .unwrap_or_else(Real::zero),
            ],
            Some(false),
            Some(false),
            Some(0),
            Some("constant polynomial row has no variable roots".to_owned()),
        );
    };
    if !problem
        .variables
        .iter()
        .any(|variable| variable.symbol == symbol)
    {
        return bernstein_report(
            constraint_index,
            Some(symbol),
            None,
            lower,
            upper,
            BernsteinRootCountStatus::UnsupportedCoefficient,
            Vec::new(),
            None,
            None,
            None,
            Some("polynomial symbol is not present in the problem".to_owned()),
        );
    }
    let Some(poly) = trim_polynomial(extracted.coefficients, policy) else {
        return bernstein_report(
            constraint_index,
            Some(symbol),
            None,
            lower,
            upper,
            BernsteinRootCountStatus::Undecided,
            Vec::new(),
            None,
            None,
            None,
            Some("could not decide polynomial degree".to_owned()),
        );
    };
    if poly
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return bernstein_report(
            constraint_index,
            Some(symbol),
            Some(poly.len().saturating_sub(1)),
            lower,
            upper,
            BernsteinRootCountStatus::UnsupportedCoefficient,
            Vec::new(),
            None,
            None,
            None,
            Some("all Bernstein coefficients must start from exact rationals".to_owned()),
        );
    }
    let endpoint_lower =
        match compare_reals(&Real::eval_poly(&poly, &lower), &Real::zero(), policy).value() {
            Some(ordering) => ordering == Ordering::Equal,
            None => {
                return bernstein_report(
                    constraint_index,
                    Some(symbol),
                    Some(poly.len().saturating_sub(1)),
                    lower,
                    upper,
                    BernsteinRootCountStatus::Undecided,
                    Vec::new(),
                    None,
                    None,
                    None,
                    Some("could not decide lower endpoint sign".to_owned()),
                );
            }
        };
    let endpoint_upper =
        match compare_reals(&Real::eval_poly(&poly, &upper), &Real::zero(), policy).value() {
            Some(ordering) => ordering == Ordering::Equal,
            None => {
                return bernstein_report(
                    constraint_index,
                    Some(symbol),
                    Some(poly.len().saturating_sub(1)),
                    lower,
                    upper,
                    BernsteinRootCountStatus::Undecided,
                    Vec::new(),
                    Some(endpoint_lower),
                    None,
                    None,
                    Some("could not decide upper endpoint sign".to_owned()),
                );
            }
        };
    let bernstein = match power_to_bernstein_on_interval(&poly, &lower, &upper) {
        Some(coefficients) => coefficients,
        None => {
            return bernstein_report(
                constraint_index,
                Some(symbol),
                Some(poly.len().saturating_sub(1)),
                lower,
                upper,
                BernsteinRootCountStatus::Undecided,
                Vec::new(),
                Some(endpoint_lower),
                Some(endpoint_upper),
                None,
                Some("could not convert polynomial to Bernstein form".to_owned()),
            );
        }
    };
    let variations = match sign_variations_for_coefficients(&bernstein, policy) {
        Some(variations) => variations,
        None => {
            return bernstein_report(
                constraint_index,
                Some(symbol),
                Some(poly.len().saturating_sub(1)),
                lower,
                upper,
                BernsteinRootCountStatus::Undecided,
                bernstein,
                Some(endpoint_lower),
                Some(endpoint_upper),
                None,
                Some("could not decide Bernstein coefficient signs".to_owned()),
            );
        }
    };
    bernstein_report(
        constraint_index,
        Some(symbol),
        Some(poly.len().saturating_sub(1)),
        lower,
        upper,
        BernsteinRootCountStatus::Counted,
        bernstein,
        Some(endpoint_lower),
        Some(endpoint_upper),
        Some(variations),
        None,
    )
}

/// Recursively subdivide active equality rows with Bernstein root-count bounds.
///
/// This produces terminal interval evidence over a caller-supplied exact
/// interval. It is useful as a finite-interval algebraic filter before a
/// stronger Sturm proof or future algebraic-number construction.
pub fn subdivide_bernstein_univariate_polynomial_interval_roots(
    analysis: &ProblemAnalysis<'_>,
    lower: Real,
    upper: Real,
    config: BernsteinSubdivisionConfig,
) -> Vec<BernsteinSubdivisionReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in analysis.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(subdivide_bernstein_univariate_polynomial_interval_expr(
            constraint_index,
            &constraint.residual,
            analysis.problem(),
            lower.clone(),
            upper.clone(),
            config,
        ));
    }
    reports
}

/// Recursively subdivide one expression with Bernstein root-count bounds.
pub fn subdivide_bernstein_univariate_polynomial_interval_expr(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    lower: Real,
    upper: Real,
    config: BernsteinSubdivisionConfig,
) -> BernsteinSubdivisionReport {
    let first = count_bernstein_univariate_polynomial_interval_expr(
        constraint_index,
        expression,
        problem,
        lower.clone(),
        upper.clone(),
        config.policy,
    );
    if first.status != BernsteinRootCountStatus::Counted {
        return bernstein_subdivision_report(
            constraint_index,
            first.symbol,
            first.degree,
            lower,
            upper,
            match first.status {
                BernsteinRootCountStatus::InvalidInterval => {
                    BernsteinSubdivisionStatus::InvalidInterval
                }
                BernsteinRootCountStatus::UnsupportedCoefficient => {
                    BernsteinSubdivisionStatus::UnsupportedCoefficient
                }
                BernsteinRootCountStatus::Undecided => BernsteinSubdivisionStatus::Undecided,
                BernsteinRootCountStatus::Counted => unreachable!(),
            },
            Vec::new(),
            first.message,
        );
    }

    let symbol = first.symbol;
    let degree = first.degree;
    let (Some(root_at_lower), Some(root_at_upper), Some(variation_bound)) = (
        first.root_at_lower,
        first.root_at_upper,
        first.variation_bound,
    ) else {
        return bernstein_subdivision_report(
            constraint_index,
            symbol,
            degree,
            lower,
            upper,
            BernsteinSubdivisionStatus::Undecided,
            Vec::new(),
            Some("counted Bernstein interval omitted terminal evidence".to_owned()),
        );
    };

    let mut intervals = Vec::new();
    let mut hit_depth_limit = false;
    let mut undecided = None;
    subdivide_bernstein_interval(
        BernsteinSubdivisionNode {
            lower: lower.clone(),
            upper: upper.clone(),
            coefficients: BernsteinControls::Real(first.bernstein_coefficients),
            variation_bound,
            root_at_lower,
            root_at_upper,
        },
        config,
        0,
        &mut intervals,
        &mut hit_depth_limit,
        &mut undecided,
    );
    let status = if undecided.is_some() {
        BernsteinSubdivisionStatus::Undecided
    } else if hit_depth_limit {
        BernsteinSubdivisionStatus::DepthLimit
    } else {
        BernsteinSubdivisionStatus::Completed
    };
    bernstein_subdivision_report(
        constraint_index,
        symbol,
        degree,
        lower,
        upper,
        status,
        intervals,
        undecided,
    )
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

fn isolate_square_free_roots(
    polynomial: &[Real],
    config: &RootIsolationConfig,
) -> Option<Vec<IsolatedRootInterval>> {
    let policy = config.policy;
    let sturm = UnivariateSturmSequence::new(polynomial, policy)?;
    let bound = power_of_two_fujiwara_bound(polynomial)?;
    let lower = -bound.clone();
    let upper = bound;
    if sign_at(&sturm.polynomials[0], &lower, policy)? == Ordering::Equal
        || sign_at(&sturm.polynomials[0], &upper, policy)? == Ordering::Equal
    {
        // Fujiwara's inequality is strict at these endpoints.  Refuse to use
        // a broken bound rather than silently changing the open-interval
        // counting invariant below.
        return None;
    }
    let root_count = sturm_count(&sturm, &lower, &upper, policy)?;
    let mut intervals = Vec::new();
    isolate_interval(
        &sturm,
        SturmIsolationNode {
            lower,
            upper,
            lower_is_root: false,
            upper_is_root: false,
            root_count,
            refinement_step: 0,
        },
        config,
        &mut intervals,
    )?;
    Some(intervals)
}

struct SturmIsolationNode {
    lower: Real,
    upper: Real,
    lower_is_root: bool,
    upper_is_root: bool,
    root_count: usize,
    refinement_step: usize,
}

fn isolate_interval(
    sturm: &UnivariateSturmSequence,
    node: SturmIsolationNode,
    config: &RootIsolationConfig,
    intervals: &mut Vec<IsolatedRootInterval>,
) -> Option<()> {
    let SturmIsolationNode {
        lower,
        upper,
        lower_is_root,
        upper_is_root,
        root_count,
        refinement_step,
    } = node;
    let policy = config.policy;
    if root_count == 0 {
        return Some(());
    }
    if root_count == 1 {
        let refine = should_refine_one_root_interval(&lower, &upper, config, refinement_step)?;
        if !lower_is_root && !upper_is_root && !refine {
            intervals.push(IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            });
            return Some(());
        }
    }

    let midpoint = ((lower.clone() + upper.clone()) / Real::from(2)).ok()?;
    let first = &sturm.polynomials[0];
    let midpoint_is_root = sign_at(first, &midpoint, policy)? == Ordering::Equal;
    // For a square-free Sturm chain, V(a)-V(b) counts roots in (a,b].  Remove
    // a midpoint root from the raw left count; the parent's open-interval
    // count then determines the disjoint right count without a second walk of
    // the whole Sturm chain.  Known roots at either parent endpoint were
    // already excluded from that invariant.
    let midpoint_count = usize::from(midpoint_is_root);
    let left_count = sturm_count(sturm, &lower, &midpoint, policy)?.checked_sub(midpoint_count)?;
    let right_count = root_count
        .checked_sub(midpoint_count)?
        .checked_sub(left_count)?;
    let child_refinement_step = if root_count == 1 {
        refinement_step.checked_add(1)?
    } else {
        0
    };
    isolate_interval(
        sturm,
        SturmIsolationNode {
            lower,
            upper: midpoint.clone(),
            lower_is_root,
            upper_is_root: midpoint_is_root,
            root_count: left_count,
            refinement_step: child_refinement_step,
        },
        config,
        intervals,
    )?;
    if midpoint_is_root {
        intervals.push(IsolatedRootInterval {
            lower: midpoint.clone(),
            upper: midpoint.clone(),
            exact_root: Some(midpoint.clone()),
            distinct_root_count: 1,
        });
    }
    isolate_interval(
        sturm,
        SturmIsolationNode {
            lower: midpoint,
            upper,
            lower_is_root: midpoint_is_root,
            upper_is_root,
            root_count: right_count,
            refinement_step: child_refinement_step,
        },
        config,
        intervals,
    )
}

fn should_refine_one_root_interval(
    lower: &Real,
    upper: &Real,
    config: &RootIsolationConfig,
    refinement_step: usize,
) -> Option<bool> {
    if refinement_step >= config.max_refinement_steps {
        return Some(false);
    }
    let Some(max_width) = &config.max_interval_width else {
        return Some(false);
    };
    let width = upper.clone() - lower.clone();
    match compare_reals(&width, max_width, config.policy).value()? {
        Ordering::Greater => Some(true),
        Ordering::Equal | Ordering::Less => Some(false),
    }
}

fn field_sturm_sequence(polynomial: &[Real], policy: PredicatePolicy) -> Option<Vec<Vec<Real>>> {
    let p0 = sign_preserving_primitive_polynomial(polynomial.to_vec(), policy)?;
    if p0.len() == 1 {
        return Some(vec![p0]);
    }
    let p1 = sign_preserving_primitive_polynomial(derivative(&p0), policy)?;
    let mut sequence = vec![p0, p1];
    loop {
        let remainder = {
            let last = sequence.last()?;
            if last.len() == 1 {
                break;
            }
            let previous = sequence.get(sequence.len() - 2)?.clone();
            polynomial_div_rem_trimmed(previous, last, policy)?.1
        };
        if trimmed_polynomial_is_zero(&remainder, policy)? {
            break;
        }
        sequence.push(sign_preserving_primitive_polynomial(
            remainder.into_iter().map(|value| -value).collect(),
            policy,
        )?);
    }
    Some(sequence)
}

/// Clear rational denominators and coefficient content by a positive scale.
/// Multiplying an individual Sturm-chain member by a positive constant leaves
/// every sign variation unchanged, while keeping Euclidean remainders from
/// accumulating large rational numerators and denominators.
fn sign_preserving_primitive_polynomial(
    polynomial: Vec<Real>,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let polynomial = trim_polynomial(polynomial, policy)?;
    let leading_sign = compare_reals(polynomial.last()?, &Real::zero(), policy).value()?;
    if leading_sign == Ordering::Equal {
        return None;
    }
    let Some(primitive) = primitive_integer_polynomial(&polynomial) else {
        // Sturm isolation also supports exact-real coefficient fields. Their
        // scale cannot be cleared through the rational primitive kernel, so
        // preserve the original chain member and retain the complete path.
        return Some(polynomial);
    };
    let primitive_leading_sign = compare_reals(primitive.last()?, &Real::zero(), policy).value()?;
    // `primitive_integer_polynomial` uses a positive projective scale. Keep an
    // explicit check here because a sign flip would invalidate a Sturm chain.
    if primitive_leading_sign != leading_sign {
        return None;
    }
    Some(primitive)
}

fn sturm_count(
    sturm: &UnivariateSturmSequence,
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<usize> {
    let lower = evaluate_sturm_at(sturm, lower, policy)?;
    let upper = evaluate_sturm_at(sturm, upper, policy)?;
    lower.variations.checked_sub(upper.variations)
}

#[derive(Clone, Copy)]
struct SturmPointEvaluation {
    variations: usize,
    polynomial_sign: Ordering,
}

fn evaluate_sturm_at(
    sturm: &UnivariateSturmSequence,
    point: &Real,
    policy: PredicatePolicy,
) -> Option<SturmPointEvaluation> {
    let mut previous = None;
    let mut variations = 0;
    let mut polynomial_sign = None;
    for (index, polynomial) in sturm.polynomials.iter().enumerate() {
        let sign = sign_at(polynomial, point, policy)?;
        if index == 0 {
            polynomial_sign = Some(sign);
        }
        if sign == Ordering::Equal {
            continue;
        }
        if let Some(previous) = previous
            && previous != sign
        {
            variations += 1;
        }
        previous = Some(sign);
    }
    Some(SturmPointEvaluation {
        variations,
        polynomial_sign: polynomial_sign?,
    })
}

fn sign_at(polynomial: &[Real], point: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    let value = Real::eval_poly(polynomial, point);
    compare_reals(&value, &Real::zero(), policy).value()
}

/// Return a rational power-of-two instance of Fujiwara's real-root bound.
///
/// For `p(x) = a_n x^n + ... + a_0`, put
/// `M = max_i |a_i/a_n|^(1/(n-i))`.  If `|x| > 2M`, the lower terms divided
/// by the leading term are bounded by a strict geometric series with ratio
/// one half, so `p(x)` cannot be zero.  Each radical is rounded upward to a
/// power of two using exact numerator/denominator product bit lengths.  The
/// resulting endpoint is exact, cheap for the dyadic scalar kernels, and much
/// tighter than `1 + max_i |a_i/a_n|` on common high-degree polynomials.
fn power_of_two_fujiwara_bound(polynomial: &[Real]) -> Option<Real> {
    let degree = polynomial.len().checked_sub(1)?;
    let leading = polynomial.last()?.exact_rational_ref()?;
    if leading.is_zero() {
        return None;
    }

    let mut radical_exponent = 0_i128;
    for (index, coefficient) in polynomial[..degree].iter().enumerate() {
        let coefficient = coefficient.exact_rational_ref()?;
        if coefficient.is_zero() {
            continue;
        }
        let ratio_numerator = coefficient.numerator() * leading.denominator();
        let ratio_denominator = coefficient.denominator() * leading.numerator();
        let ratio_upper_exponent =
            i128::from(ratio_numerator.bits()) - i128::from(ratio_denominator.bits()) + 1;
        if ratio_upper_exponent <= 0 {
            continue;
        }
        let root_degree = i128::try_from(degree.checked_sub(index)?).ok()?;
        let candidate = (ratio_upper_exponent + root_degree - 1) / root_degree;
        radical_exponent = radical_exponent.max(candidate);
    }
    let bound_exponent = usize::try_from(radical_exponent.checked_add(1)?).ok()?;
    Some(Real::from(HyperRational::from_bigint(
        BigInt::one() << bound_exponent,
    )))
}

fn derivative(polynomial: &[Real]) -> Vec<Real> {
    if polynomial.len() <= 1 {
        return vec![Real::zero()];
    }
    polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient.clone() * Real::from(degree as i64))
        .collect()
}

pub(crate) fn polynomial_gcd(
    mut left: Vec<Real>,
    mut right: Vec<Real>,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    left = trim_polynomial(left, policy)?;
    right = trim_polynomial(right, policy)?;
    if let Some(gcd) = primitive_integer_polynomial_gcd(&left, &right) {
        return gcd_monic_normalize(gcd, policy);
    }
    loop {
        let right_is_zero = trimmed_polynomial_is_zero(&right, policy)?;
        if right_is_zero {
            break;
        }
        let (_, remainder) = polynomial_div_rem_trimmed(left, &right, policy)?;
        left = right;
        right = remainder;
    }
    gcd_monic_normalize(left, policy)
}

pub(crate) fn polynomials_share_one_root_in_interval(
    left: &[Real],
    right: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    if compare_reals(lower, upper, policy).value()? != Ordering::Less {
        return Some(false);
    }
    let gcd = if left == right {
        left.to_vec()
    } else {
        polynomial_gcd(left.to_vec(), right.to_vec(), policy)?
    };
    if gcd.len() <= 1 {
        return Some(false);
    }
    let square_free = square_free_part(gcd, policy)?;
    let sturm = UnivariateSturmSequence::new(&square_free, policy)?;
    match sturm_count(&sturm, lower, upper, policy)? {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    }
}

/// Decides whether `candidate` vanishes at the defining polynomial's one root
/// in the partition-owned `(lower, upper]` interval.
///
/// This is narrower than a generic common-root query: callers must supply an
/// interval whose `distinct_root_count` already certifies one defining root.
/// The GCD can therefore own either that same root or no root in the interval.
/// An exact point witness is replayed directly.
pub(crate) fn polynomial_vanishes_at_owned_root(
    defining_polynomial: &[Real],
    candidate: &[Real],
    interval: &IsolatedRootInterval,
    policy: PredicatePolicy,
) -> Option<bool> {
    if interval.distinct_root_count != 1 {
        return None;
    }
    if let Some(root) = interval.exact_root.as_ref() {
        return Some(
            compare_reals(&Real::eval_poly(candidate, root), &Real::zero(), policy).value()?
                == Ordering::Equal,
        );
    }
    if compare_reals(&interval.lower, &interval.upper, policy).value()? != Ordering::Less {
        return None;
    }
    let gcd = if defining_polynomial == candidate {
        defining_polynomial.to_vec()
    } else {
        polynomial_gcd(defining_polynomial.to_vec(), candidate.to_vec(), policy)?
    };
    if gcd.len() <= 1 {
        return Some(false);
    }
    polynomial_has_one_distinct_root_with_upper_ownership(
        &gcd,
        &interval.lower,
        &interval.upper,
        policy,
        UpperEndpointOwnership::Included,
    )
}

/// Proves whether `(lower, upper)` contains exactly one distinct root.
///
/// Endpoint roots are excluded. `Some(false)` is an exact proof that the open
/// interval does not contain exactly one distinct root (and also covers
/// non-increasing bounds); `None` means a required predicate or exact field
/// operation did not decide under `policy`. The proof schedule uses endpoint
/// signs, monotonicity, Bernstein variation, a complete repeated-quadratic
/// discriminant rule, square-free reduction, and finally a Sturm count.
pub fn polynomial_has_one_distinct_root_in_open_interval(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    polynomial_has_one_distinct_root_with_upper_ownership(
        polynomial,
        lower,
        upper,
        policy,
        UpperEndpointOwnership::Excluded,
    )
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum UpperEndpointOwnership {
    /// Count roots in `(lower, upper)`.
    Excluded,
    /// Count roots in `(lower, upper]`.
    Included,
}

fn polynomial_has_one_distinct_root_with_upper_ownership(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
    upper_ownership: UpperEndpointOwnership,
) -> Option<bool> {
    if compare_reals(lower, upper, policy).value()? != Ordering::Less {
        return Some(false);
    }
    let lower_value = Real::eval_poly(polynomial, lower);
    let upper_value = Real::eval_poly(polynomial, upper);
    let lower_sign = compare_reals(&lower_value, &Real::zero(), policy).value()?;
    let upper_sign = compare_reals(&upper_value, &Real::zero(), policy).value()?;
    let root_at_lower = lower_sign == Ordering::Equal;
    let root_at_upper = upper_sign == Ordering::Equal;
    if !root_at_lower && !root_at_upper && polynomial.len() <= 3 && lower_sign != upper_sign {
        return Some(true);
    }
    // A narrow candidate interval commonly arrives with an independent exact
    // existence proof (for example, selected algebraic-fiber isolation). An
    // exact Horner enclosure of the derivative certifies monotonicity in
    // linear work and avoids a high-degree Bernstein basis change whenever it
    // excludes zero. Endpoint signs then decide whether the interval owns one
    // root or none. Quadratics instead take their complete constant-size
    // Bernstein/discriminant path below.
    if polynomial.len() != 3
        && polynomial_derivative_interval_excludes_zero(polynomial, lower, upper, policy)
            == Some(true)
    {
        let open_root_count =
            usize::from(!root_at_lower && !root_at_upper && lower_sign != upper_sign);
        let root_count = open_root_count.checked_add(usize::from(
            upper_ownership == UpperEndpointOwnership::Included && root_at_upper,
        ))?;
        return Some(root_count == 1);
    }
    let variations = polynomial_interval_bernstein_variations(
        polynomial,
        lower,
        upper,
        lower_value,
        upper_value,
        Some((lower_sign, upper_sign)),
        policy,
    )?;
    if variations <= 1 {
        let root_count = variations.checked_add(usize::from(
            upper_ownership == UpperEndpointOwnership::Included && root_at_upper,
        ))?;
        return Some(root_count == 1);
    }
    if let [constant, linear, quadratic] = polynomial
        && let Some(has_one_root) = quadratic_multiple_variation_has_one_distinct_root(
            constant,
            linear,
            quadratic,
            root_at_lower,
            root_at_upper,
            policy,
        )
    {
        return Some(has_one_root);
    }
    let square_free = square_free_part(polynomial.to_vec(), policy)?;
    // Equal storage length proves that square-free reduction returned the
    // already-trimmed source unchanged: a nonconstant gcd would reduce the
    // quotient degree. Reuse the first Bernstein bound instead of repeating
    // the same quadratic basis conversion.
    let variations = if square_free.len() == polynomial.len() {
        variations
    } else {
        polynomial_interval_bernstein_variations(
            &square_free,
            lower,
            upper,
            Real::eval_poly(&square_free, lower),
            Real::eval_poly(&square_free, upper),
            None,
            policy,
        )?
    };
    if variations <= 1 {
        let root_count = variations.checked_add(usize::from(
            upper_ownership == UpperEndpointOwnership::Included && root_at_upper,
        ))?;
        return Some(root_count == 1);
    }
    let sturm = UnivariateSturmSequence::new(&square_free, policy)?;
    let half_open_root_count = sturm_count(&sturm, lower, upper, policy)?;
    let root_count = if upper_ownership == UpperEndpointOwnership::Included {
        half_open_root_count
    } else {
        half_open_root_count.checked_sub(usize::from(root_at_upper))?
    };
    Some(root_count == 1)
}

/// Resolve the only degree-two case that a multiple Bernstein variation can
/// still certify as one *distinct* root. For a zero-discriminant quadratic
/// `a(x-r)^2`, the interval Bernstein controls have signs proportional to
/// `(lower-r)^2`, `(lower-r)(upper-r)`, and `(upper-r)^2`; variation two is
/// therefore possible exactly when the repeated root is interior. A nonzero
/// discriminant leaves either zero or two open roots, never one distinct root.
fn quadratic_multiple_variation_has_one_distinct_root(
    constant: &Real,
    linear: &Real,
    quadratic: &Real,
    root_at_lower: bool,
    root_at_upper: bool,
    policy: PredicatePolicy,
) -> Option<bool> {
    let discriminant = linear * linear - Real::from(4_i8) * quadratic * constant;
    Some(
        compare_reals(&discriminant, &Real::zero(), policy).value()? == Ordering::Equal
            && !root_at_lower
            && !root_at_upper,
    )
}

/// Proves that a closed exact interval contains no distinct root.
///
/// A Horner interval exclusion handles the common narrow-interval case in
/// linear work. Endpoint replay and a square-free Sturm count remain the
/// complete exact fallback; an undecided coefficient predicate returns
/// `None` rather than treating absence of evidence as an empty interval.
pub(crate) fn polynomial_has_no_distinct_root_in_closed_interval(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    match compare_reals(lower, upper, policy).value()? {
        Ordering::Greater => return None,
        Ordering::Equal => {
            return Some(
                compare_reals(&Real::eval_poly(polynomial, lower), &Real::zero(), policy)
                    .value()?
                    != Ordering::Equal,
            );
        }
        Ordering::Less => {}
    }
    let lower_sign =
        compare_reals(&Real::eval_poly(polynomial, lower), &Real::zero(), policy).value()?;
    let upper_sign =
        compare_reals(&Real::eval_poly(polynomial, upper), &Real::zero(), policy).value()?;
    if lower_sign == Ordering::Equal || upper_sign == Ordering::Equal {
        return Some(false);
    }
    if polynomial_interval_excludes_zero(polynomial, lower, upper, policy) == Some(true) {
        return Some(true);
    }
    let square_free = square_free_part(polynomial.to_vec(), policy)?;
    let sturm = UnivariateSturmSequence::new(&square_free, policy)?;
    Some(sturm_count(&sturm, lower, upper, policy)? == 0)
}

fn polynomial_interval_enclosure(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<(Real, Real)> {
    let leading = polynomial.last()?.clone();
    let mut range_lower = leading.clone();
    let mut range_upper = leading;
    for coefficient in polynomial[..polynomial.len().saturating_sub(1)]
        .iter()
        .rev()
    {
        let products = [
            &range_lower * lower,
            &range_lower * upper,
            &range_upper * lower,
            &range_upper * upper,
        ];
        let mut product_lower = products[0].clone();
        let mut product_upper = products[0].clone();
        for product in &products[1..] {
            if compare_reals(product, &product_lower, policy).value()? == Ordering::Less {
                product_lower = product.clone();
            }
            if compare_reals(product, &product_upper, policy).value()? == Ordering::Greater {
                product_upper = product.clone();
            }
        }
        range_lower = product_lower + coefficient;
        range_upper = product_upper + coefficient;
    }
    Some((range_lower, range_upper))
}

fn polynomial_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    if let Some(excludes_zero) =
        exact_rational_polynomial_interval_excludes_zero(polynomial, lower, upper)
    {
        return Some(excludes_zero);
    }
    let (range_lower, range_upper) =
        polynomial_interval_enclosure(polynomial, lower, upper, policy)?;
    let range_lower_sign = compare_reals(&range_lower, &Real::zero(), policy).value()?;
    let range_upper_sign = compare_reals(&range_upper, &Real::zero(), policy).value()?;
    Some(range_lower_sign == Ordering::Greater || range_upper_sign == Ordering::Less)
}

fn polynomial_derivative_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    if polynomial.len() <= 1 {
        return Some(false);
    }
    if let Some(excludes_zero) =
        exact_rational_polynomial_derivative_interval_excludes_zero(polynomial, lower, upper)
    {
        return Some(excludes_zero);
    }
    polynomial_interval_excludes_zero(&derivative(polynomial), lower, upper, policy)
}

fn exact_rational_polynomial_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    if lower > upper {
        return None;
    }
    let mut coefficients = polynomial.iter().rev();
    let leading = coefficients.next()?.exact_rational_ref()?;
    let mut range_lower = leading.clone();
    let mut range_upper = leading.clone();
    for coefficient in coefficients {
        let coefficient = coefficient.exact_rational_ref()?;
        let (product_lower, product_upper) =
            rational_interval_product(&range_lower, &range_upper, lower, upper);
        range_lower = product_lower + coefficient;
        range_upper = product_upper + coefficient;
    }
    Some(range_lower.is_positive() || range_upper.is_negative())
}

fn exact_rational_polynomial_derivative_interval_excludes_zero(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<bool> {
    let lower = lower.exact_rational_ref()?;
    let upper = upper.exact_rational_ref()?;
    if lower > upper {
        return None;
    }
    let mut coefficients = polynomial.iter().enumerate().skip(1).rev();
    let (degree, leading) = coefficients.next()?;
    let degree = HyperRational::new(i64::try_from(degree).ok()?);
    let leading = leading.exact_rational_ref()? * &degree;
    let mut range_lower = leading.clone();
    let mut range_upper = leading;
    for (degree, coefficient) in coefficients {
        let (product_lower, product_upper) =
            rational_interval_product(&range_lower, &range_upper, lower, upper);
        let degree = HyperRational::new(i64::try_from(degree).ok()?);
        let coefficient = coefficient.exact_rational_ref()? * degree;
        range_lower = product_lower + &coefficient;
        range_upper = product_upper + coefficient;
    }
    Some(range_lower.is_positive() || range_upper.is_negative())
}

/// Refines one represented source root until its conservative image enclosure
/// contains one distinct root of the exact image polynomial.
///
/// Positive-width image intervals use the isolator's `(lower, upper]`
/// ownership convention. Exact image witnesses instead own their point: they
/// must lie within the supplied closed bounds and exactly annihilate the
/// nonconstant, policy-trimmed image polynomial, after which the result is
/// canonicalized to a point interval. Failed ordinary enclosures trigger at
/// most eight bounded source refinements; invalid exact evidence fails closed.
pub(crate) fn certify_algebraic_image_interval<F>(
    source_polynomial: &[Real],
    source_interval: &IsolatedRootInterval,
    image_polynomial: &[Real],
    policy: PredicatePolicy,
    mut enclosure: F,
) -> Option<IsolatedRootInterval>
where
    F: FnMut(&IsolatedRootInterval) -> Option<IsolatedRootInterval>,
{
    let image_polynomial = trim_polynomial_slice(image_polynomial, policy)?;
    if image_polynomial.len() <= 1 {
        return None;
    }
    let mut refined_source_interval = None;
    for round in 0..=ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS {
        let current_source_interval = refined_source_interval.as_ref().unwrap_or(source_interval);
        if let Some(mut image_interval) = enclosure(current_source_interval) {
            if let Some(root) = image_interval.exact_root.take() {
                let lower_ordering = compare_reals(&root, &image_interval.lower, policy).value()?;
                let upper_ordering = compare_reals(&root, &image_interval.upper, policy).value()?;
                if lower_ordering == Ordering::Less
                    || upper_ordering == Ordering::Greater
                    || sign_at(image_polynomial, &root, policy)? != Ordering::Equal
                {
                    return None;
                }
                image_interval.lower = root.clone();
                image_interval.upper = root.clone();
                image_interval.exact_root = Some(root);
                image_interval.distinct_root_count = 1;
                return Some(image_interval);
            }
            if polynomial_has_one_distinct_root_with_upper_ownership(
                image_polynomial,
                &image_interval.lower,
                &image_interval.upper,
                policy,
                UpperEndpointOwnership::Included,
            ) == Some(true)
            {
                image_interval.distinct_root_count = 1;
                return Some(image_interval);
            }
        }
        if round == ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS {
            break;
        }
        let refinement = refine_isolated_univariate_polynomial_interval(
            source_polynomial,
            current_source_interval,
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
        let refined = refinement.refined_interval?;
        if refined == *current_source_interval {
            return None;
        }
        refined_source_interval = Some(refined);
    }
    None
}

fn polynomial_interval_bernstein_variations(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
    lower_value: Real,
    upper_value: Real,
    endpoint_signs: Option<(Ordering, Ordering)>,
    policy: PredicatePolicy,
) -> Option<usize> {
    if let [_, _] = polynomial {
        let (lower_sign, upper_sign) = match endpoint_signs {
            Some(signs) => signs,
            None => (
                compare_reals(&lower_value, &Real::zero(), policy).value()?,
                compare_reals(&upper_value, &Real::zero(), policy).value()?,
            ),
        };
        return Some(sign_variations_for_orderings(&[lower_sign, upper_sign]));
    }
    if let [_, linear, quadratic] = polynomial {
        let derivative_at_lower = linear + &(quadratic * lower * Real::from(2_i8));
        let half_width_derivative =
            ((upper - lower) * derivative_at_lower / Real::from(2_i8)).ok()?;
        let middle = &lower_value + &half_width_derivative;
        let (lower_sign, upper_sign) = match endpoint_signs {
            Some(signs) => signs,
            None => (
                compare_reals(&lower_value, &Real::zero(), policy).value()?,
                compare_reals(&upper_value, &Real::zero(), policy).value()?,
            ),
        };
        let middle_sign = compare_reals(&middle, &Real::zero(), policy).value()?;
        return Some(sign_variations_for_orderings(&[
            lower_sign,
            middle_sign,
            upper_sign,
        ]));
    }
    if let Some(variations) =
        exact_rational_polynomial_interval_bernstein_variations(polynomial, lower, upper)
    {
        return Some(variations);
    }
    sign_variations_for_coefficients(
        &power_to_bernstein_on_interval(polynomial, lower, upper)?,
        policy,
    )
}

fn sign_variations_for_orderings(signs: &[Ordering]) -> usize {
    let mut previous = None;
    let mut variations = 0;
    for &sign in signs {
        if sign == Ordering::Equal {
            continue;
        }
        if previous.is_some_and(|previous| previous != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    variations
}

/// Computes the same affine power-to-Bernstein sign sequence as the generic
/// `Real` path after clearing every positive denominator once. Resultants and
/// isolator endpoints are exact rationals, so this fraction-free path avoids a
/// rational reduction after every intermediate multiply and add.
fn exact_rational_polynomial_interval_bernstein_variations(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<usize> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let coefficients = HyperRational::primitive_bigint_ratio(&rationals);
    let one = HyperRational::one();
    let endpoints = HyperRational::primitive_bigint_ratio(&[
        lower.exact_rational_ref()?,
        upper.exact_rational_ref()?,
        &one,
    ]);
    let [lower, upper, scale] = endpoints.as_slice() else {
        return None;
    };
    if scale.sign() != Sign::Plus {
        return None;
    }
    let width = upper - lower;
    let degree = coefficients.len().checked_sub(1)?;

    // S^n p((L + W x) / S), formed by affine Horner composition. The positive
    // factor S^n preserves every Bernstein coefficient sign.
    let mut shifted_power = vec![coefficients.last()?.clone()];
    let mut scale_power = BigInt::one();
    for coefficient in coefficients[..degree].iter().rev() {
        scale_power *= scale;
        let old_len = shifted_power.len();
        shifted_power.push(BigInt::zero());
        for power in (1..=old_len).rev() {
            shifted_power[power] =
                &shifted_power[power] * lower + &shifted_power[power - 1] * &width;
        }
        shifted_power[0] = &shifted_power[0] * lower + coefficient * &scale_power;
    }

    // Multiplying control i by the positive falling factorial (n)_i clears
    // every C(n,k) denominator. The weight recurrence remains integral:
    // w[k+1] = w[k] (i-k)/(n-k).
    let mut previous = None;
    let mut variations = 0_usize;
    let mut degree_falling = BigInt::one();
    for index in 0..=degree {
        if index != 0 {
            degree_falling *= BigInt::from(degree - index + 1);
        }
        let mut weight = degree_falling.clone();
        let mut value = BigInt::zero();
        for (power, coefficient) in shifted_power.iter().enumerate().take(index + 1) {
            value += coefficient * &weight;
            if power != index {
                weight *= BigInt::from(index - power);
                weight /= BigInt::from(degree - power);
            }
        }
        let sign = value.sign();
        if sign == Sign::NoSign {
            continue;
        }
        if previous.is_some_and(|previous| previous != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    Some(variations)
}

/// Returns the exact square-free part of a nonzero polynomial.
///
/// The result has the same distinct roots as `polynomial`. `None` means that
/// coefficient classification, GCD construction, or exact division could not
/// be certified under `policy`; it never returns an approximate polynomial.
pub fn square_free_part(polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    let polynomial = trim_polynomial(polynomial, policy)?;
    let derivative = derivative(&polynomial);
    let gcd = if polynomial
        .iter()
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
    {
        let derivative = trim_polynomial(derivative, policy)?;
        if let Some(gcd) = primitive_integer_polynomial_gcd(&polynomial, &derivative) {
            gcd_monic_normalize(gcd, policy)?
        } else {
            polynomial_gcd(polynomial.clone(), derivative, policy)?
        }
    } else {
        polynomial_gcd(polynomial.clone(), derivative, policy)?
    };
    if gcd.len() <= 1 {
        return Some(polynomial);
    }
    if let Some(quotient) = rational_polynomial_exact_quotient(&polynomial, &gcd) {
        return Some(quotient);
    }
    let (quotient, remainder) = polynomial_div_rem_trimmed(polynomial, &gcd, policy)?;
    trimmed_polynomial_is_zero(&remainder, policy)?.then_some(quotient)
}

pub(crate) fn polynomial_div_rem(
    dividend: Vec<Real>,
    divisor: &[Real],
    policy: PredicatePolicy,
) -> Option<(Vec<Real>, Vec<Real>)> {
    let divisor = trim_polynomial(divisor.to_vec(), policy)?;
    let dividend = trim_polynomial(dividend, policy)?;
    polynomial_div_rem_trimmed(dividend, &divisor, policy)
}

/// Divides while policy-trimming the divisor in borrowed storage. This is most
/// useful when nonrational coefficient handles are expensive to clone.
#[inline]
pub(crate) fn polynomial_div_rem_borrowed_divisor(
    dividend: Vec<Real>,
    divisor: &[Real],
    policy: PredicatePolicy,
) -> Option<(Vec<Real>, Vec<Real>)> {
    let divisor = trim_polynomial_slice(divisor, policy)?;
    let dividend = trim_polynomial(dividend, policy)?;
    polynomial_div_rem_trimmed(dividend, divisor, policy)
}

/// Divides two already-trimmed polynomials without cloning or recertifying the
/// divisor. The returned quotient and remainder are also trimmed.
fn polynomial_div_rem_trimmed(
    mut remainder: Vec<Real>,
    divisor: &[Real],
    policy: PredicatePolicy,
) -> Option<(Vec<Real>, Vec<Real>)> {
    if trimmed_polynomial_is_zero(divisor, policy)? {
        return None;
    }
    if remainder.len() < divisor.len() {
        return Some((vec![Real::zero()], remainder));
    }
    let mut quotient = vec![Real::zero(); remainder.len() - divisor.len() + 1];
    let divisor_degree = divisor.len() - 1;
    let divisor_leading_inverse = reciprocal_real(divisor.last()?, policy).ok()?.value()?;
    while remainder.len() >= divisor.len() && !trimmed_polynomial_is_zero(&remainder, policy)? {
        let degree_delta = remainder.len() - divisor.len();
        let scale = remainder.last()? * &divisor_leading_inverse;
        for (index, divisor_coefficient) in divisor.iter().enumerate().take(divisor_degree) {
            let target = degree_delta + index;
            remainder[target] -= &scale * divisor_coefficient;
        }
        // The selected quotient coefficient cancels the leading term by
        // construction. Remove that slot directly instead of materializing and
        // then certifying `(a / b) * b == a`. Each quotient degree is reached
        // only once, so its preallocated zero can likewise be replaced.
        remainder.pop();
        quotient[degree_delta] = scale;
        remainder = trim_polynomial(remainder, policy)?;
    }
    // The highest assigned quotient coefficient is a product of two values
    // already certified nonzero above, so the preallocated vector is canonical
    // without asking the scalar layer to prove that product again.
    Some((quotient, remainder))
}

/// One polynomial divisor whose leading degree and reciprocal have already
/// been certified for repeated quotient-ring reductions.
pub(crate) struct CertifiedPolynomialDivisor {
    coefficients: Vec<Real>,
    leading_inverse: Real,
}

impl CertifiedPolynomialDivisor {
    /// Trims and certifies one nonconstant divisor under `policy`.
    pub(crate) fn new(divisor: &[Real], policy: PredicatePolicy) -> Option<Self> {
        let coefficients = trim_polynomial(divisor.to_vec(), policy)?;
        if coefficients.len() <= 1 {
            return None;
        }
        let leading_inverse = reciprocal_real(coefficients.last()?, policy)
            .ok()?
            .value()?;
        Some(Self {
            coefficients,
            leading_inverse,
        })
    }

    /// Returns the certified divisor degree.
    pub(crate) fn degree(&self) -> usize {
        self.coefficients.len() - 1
    }

    /// Replaces `remainder` by its fixed-degree exact polynomial remainder.
    ///
    /// Quotient-ring tensor reduction does not need a trimmed quotient or
    /// remainder: every power at or above the divisor degree is eliminated by
    /// an exact field operation. Avoiding intermediate zero tests is essential
    /// when coefficients are correlated `Real` expressions whose cancellation
    /// follows from the very divisor relation being applied.
    pub(crate) fn remainder_in_place(&self, remainder: &mut Vec<Real>) {
        let divisor_degree = self.degree();
        if remainder.len() <= divisor_degree {
            return;
        }
        if let Some(mut reduced) = crate::integer_interpolation::rational_polynomial_remainder_modulo(
            remainder,
            &self.coefficients,
        ) {
            reduced.resize(divisor_degree, Real::zero());
            *remainder = reduced;
            return;
        }
        for power in (divisor_degree..remainder.len()).rev() {
            if remainder[power].definitely_zero() {
                continue;
            }
            let scale = &remainder[power] * &self.leading_inverse;
            let target_start = power - divisor_degree;
            for (index, divisor_coefficient) in
                self.coefficients.iter().take(divisor_degree).enumerate()
            {
                let target = target_start + index;
                remainder[target] = remainder[target].clone() - &scale * divisor_coefficient;
            }
        }
        remainder.truncate(divisor_degree);
    }
}

fn root_refinement_report(
    status: IsolatedRootRefinementStatus,
    original_interval: IsolatedRootInterval,
    refined_interval: Option<IsolatedRootInterval>,
    refinement_steps: usize,
    message: Option<String>,
) -> IsolatedRootRefinementReport {
    IsolatedRootRefinementReport {
        status,
        original_interval,
        refined_interval,
        refinement_steps,
        message,
    }
}

fn exact_root_refinement_report(
    original_interval: &IsolatedRootInterval,
    root: Real,
    refinement_steps: usize,
) -> IsolatedRootRefinementReport {
    root_refinement_report(
        IsolatedRootRefinementStatus::ExactRoot,
        original_interval.clone(),
        Some(IsolatedRootInterval {
            lower: root.clone(),
            upper: root.clone(),
            exact_root: Some(root),
            distinct_root_count: 1,
        }),
        refinement_steps,
        None,
    )
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

#[inline]
fn trim_polynomial_slice(mut polynomial: &[Real], policy: PredicatePolicy) -> Option<&[Real]> {
    while polynomial.len() > 1 {
        let trailing = polynomial.last()?;
        if let Some(trailing) = trailing.exact_rational_ref() {
            if trailing.is_zero() {
                polynomial = &polynomial[..polynomial.len() - 1];
                continue;
            }
            break;
        }
        match compare_reals(trailing, &Real::zero(), policy).value()? {
            Ordering::Equal => polynomial = &polynomial[..polynomial.len() - 1],
            Ordering::Less | Ordering::Greater => break,
        }
    }
    Some(polynomial)
}

/// Tests a polynomial after `trim_polynomial` has already certified its final
/// stored coefficient. A canonical vector with more than one coefficient is
/// therefore nonzero; only the scalar constant case still needs a predicate.
fn trimmed_polynomial_is_zero(polynomial: &[Real], policy: PredicatePolicy) -> Option<bool> {
    if polynomial.len() > 1 {
        return Some(false);
    }
    let coefficient = polynomial.first()?;
    Some(compare_reals(coefficient, &Real::zero(), policy).value()? == Ordering::Equal)
}

/// Monic-normalizes a GCD whose producing integer or Euclidean kernel has
/// already returned canonical coefficient storage.
fn gcd_monic_normalize(polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    if polynomial.len() == 1 {
        return Some(if trimmed_polynomial_is_zero(&polynomial, policy)? {
            vec![Real::zero()]
        } else {
            vec![Real::one()]
        });
    }
    let leading = polynomial.last()?.clone();
    let inverse = reciprocal_real(&leading, policy).ok()?.value()?;
    Some(
        polynomial
            .into_iter()
            .map(|coefficient| coefficient * &inverse)
            .collect(),
    )
}

fn leading_zero_multiplicity(polynomial: &[Real], policy: PredicatePolicy) -> Option<usize> {
    let mut multiplicity = 0;
    for coefficient in polynomial {
        match compare_reals(coefficient, &Real::zero(), policy).value()? {
            Ordering::Equal => multiplicity += 1,
            Ordering::Less | Ordering::Greater => return Some(multiplicity),
        }
    }
    Some(0)
}

fn sign_variations_for_coefficients(
    coefficients: &[Real],
    policy: PredicatePolicy,
) -> Option<usize> {
    let mut previous = None;
    let mut variations = 0;
    for coefficient in coefficients.iter().rev() {
        let sign = compare_reals(coefficient, &Real::zero(), policy).value()?;
        if sign == Ordering::Equal {
            continue;
        }
        if let Some(previous) = previous
            && previous != sign
        {
            variations += 1;
        }
        previous = Some(sign);
    }
    Some(variations)
}

fn power_to_bernstein_on_interval(
    polynomial: &[Real],
    lower: &Real,
    upper: &Real,
) -> Option<Vec<Real>> {
    let leading = polynomial.last()?.clone();
    let degree = polynomial.len().saturating_sub(1);
    let width = upper.clone() - lower.clone();
    // Horner composition by `lower + width*x` performs the affine power-basis
    // change in quadratic time without materializing large binomial integers.
    let mut shifted_power = vec![leading];
    for coefficient in polynomial[..degree].iter().rev() {
        let old_len = shifted_power.len();
        shifted_power.push(Real::zero());
        for power in (1..=old_len).rev() {
            shifted_power[power] =
                shifted_power[power].clone() * lower + shifted_power[power - 1].clone() * &width;
        }
        shifted_power[0] = shifted_power[0].clone() * lower + coefficient;
    }

    let mut bernstein = vec![Real::zero(); degree + 1];
    for (i, target) in bernstein.iter_mut().enumerate().take(degree + 1) {
        let mut value = Real::zero();
        let mut ratio = Real::one();
        for (j, coefficient) in shifted_power.iter().enumerate().take(i + 1) {
            value += coefficient * &ratio;
            if j != i {
                let numerator = Real::from(u64::try_from(i - j).ok()?);
                let denominator = Real::from(u64::try_from(degree - j).ok()?);
                ratio = ((ratio * numerator) / denominator).ok()?;
            }
        }
        *target = value;
    }
    Some(bernstein)
}

struct BernsteinSubdivisionNode {
    lower: Real,
    upper: Real,
    coefficients: BernsteinControls,
    variation_bound: usize,
    root_at_lower: bool,
    root_at_upper: bool,
}

// Public coefficient reports retain their exact magnitudes. Subdivision only
// needs signs and endpoint zeros, so a common positive scale may be omitted.
enum BernsteinControls {
    Real(Vec<Real>),
    Integer(Vec<BigInt>),
}

impl BernsteinControls {
    fn split_midpoint(self, depth: usize) -> Option<(Self, Self)> {
        match self {
            Self::Real(coefficients) => {
                // A quadratic often finishes after one cheap split. Convert
                // it only when another subdivision actually becomes necessary.
                let rationals = if depth != 0 || coefficients.len() > 3 {
                    coefficients
                        .iter()
                        .map(Real::exact_rational_ref)
                        .collect::<Option<Vec<_>>>()
                } else {
                    None
                };
                if let Some(rationals) = rationals {
                    let integers = HyperRational::primitive_bigint_ratio(&rationals);
                    let (left, right) = midpoint_subdivide_integer_bernstein(integers)?;
                    Some((Self::Integer(left), Self::Integer(right)))
                } else {
                    let (left, right) = midpoint_subdivide_bernstein(&coefficients)?;
                    Some((Self::Real(left), Self::Real(right)))
                }
            }
            Self::Integer(coefficients) => {
                let (left, right) = midpoint_subdivide_integer_bernstein(coefficients)?;
                Some((Self::Integer(left), Self::Integer(right)))
            }
        }
    }

    fn first_sign(&self, policy: PredicatePolicy) -> Option<Ordering> {
        match self {
            Self::Real(coefficients) => {
                compare_reals(coefficients.first()?, &Real::zero(), policy).value()
            }
            Self::Integer(coefficients) => Some(coefficients.first()?.cmp(&BigInt::zero())),
        }
    }

    fn variations(&self, policy: PredicatePolicy) -> Option<usize> {
        match self {
            Self::Real(coefficients) => sign_variations_for_coefficients(coefficients, policy),
            Self::Integer(coefficients) => {
                let mut previous = None;
                let mut variations = 0;
                for coefficient in coefficients {
                    let sign = coefficient.sign();
                    if sign == Sign::NoSign {
                        continue;
                    }
                    if previous.is_some_and(|previous| previous != sign) {
                        variations += 1;
                    }
                    previous = Some(sign);
                }
                Some(variations)
            }
        }
    }
}

fn midpoint_subdivide_integer_bernstein(
    mut work: Vec<BigInt>,
) -> Option<(Vec<BigInt>, Vec<BigInt>)> {
    let degree = work.len().checked_sub(1)?;
    let mut left = Vec::with_capacity(work.len());
    let mut right = vec![BigInt::zero(); work.len()];
    left.push(work.first()? << degree);
    right[degree] = work.get(degree)? << degree;

    // At level l, unhalved sums are 2^l times the usual tableau. Scale each
    // emitted boundary by 2^(n-l), giving every child control the SAME
    // positive factor 2^n. Keep one work row instead of the whole tableau.
    for level in 1..=degree {
        for index in 0..=degree - level {
            let (prefix, suffix) = work.split_at_mut(index + 1);
            prefix[index] += &suffix[0];
        }
        left.push(&work[0] << (degree - level));
        right[degree - level] = &work[degree - level] << (degree - level);
    }
    Some((left, right))
}

/// Split exact Bernstein coefficients at the midpoint with de Casteljau's
/// recurrence. The terminal value is shared by both children, so it is also
/// exact endpoint evidence for the subdivision point.
fn midpoint_subdivide_bernstein(coefficients: &[Real]) -> Option<(Vec<Real>, Vec<Real>)> {
    let degree = coefficients.len().checked_sub(1)?;
    let mut work = coefficients.to_vec();
    let mut left = Vec::with_capacity(coefficients.len());
    let mut right = vec![Real::zero(); coefficients.len()];
    left.push(work.first()?.clone());
    right[degree] = work.get(degree)?.clone();

    for level in 1..=degree {
        for index in 0..=degree - level {
            work[index] = ((work[index].clone() + work[index + 1].clone()) / Real::from(2)).ok()?;
        }
        left.push(work[0].clone());
        right[degree - level] = work[degree - level].clone();
    }
    Some((left, right))
}

fn subdivide_bernstein_interval(
    node: BernsteinSubdivisionNode,
    config: BernsteinSubdivisionConfig,
    depth: usize,
    intervals: &mut Vec<BernsteinSubdivisionInterval>,
    hit_depth_limit: &mut bool,
    undecided: &mut Option<String>,
) {
    if undecided.is_some() {
        return;
    }
    let BernsteinSubdivisionNode {
        lower,
        upper,
        coefficients,
        variation_bound,
        root_at_lower,
        root_at_upper,
    } = node;

    if root_at_lower {
        push_unique_bernstein_endpoint(intervals, lower.clone());
    }
    if root_at_upper {
        push_unique_bernstein_endpoint(intervals, upper.clone());
    }

    match variation_bound {
        0 => {
            intervals.push(BernsteinSubdivisionInterval {
                lower,
                upper,
                exact_root: None,
                variation_bound: Some(0),
                status: BernsteinSubdivisionIntervalStatus::Empty,
            });
        }
        1 if !root_at_lower && !root_at_upper => {
            intervals.push(BernsteinSubdivisionInterval {
                lower,
                upper,
                exact_root: None,
                variation_bound: Some(1),
                status: BernsteinSubdivisionIntervalStatus::Isolating,
            });
        }
        variation => {
            if depth >= config.max_depth {
                *hit_depth_limit = true;
                intervals.push(BernsteinSubdivisionInterval {
                    lower,
                    upper,
                    exact_root: None,
                    variation_bound: Some(variation),
                    status: BernsteinSubdivisionIntervalStatus::DepthLimit,
                });
                return;
            }
            let Some(midpoint) = ((lower.clone() + upper.clone()) / Real::from(2)).ok() else {
                *undecided = Some("could not bisect Bernstein interval".to_owned());
                return;
            };
            let Some((left_coefficients, right_coefficients)) = coefficients.split_midpoint(depth)
            else {
                *undecided = Some("could not subdivide exact Bernstein coefficients".to_owned());
                return;
            };
            let Some(midpoint_sign) = right_coefficients.first_sign(config.policy) else {
                *undecided = Some("could not decide Bernstein midpoint sign".to_owned());
                return;
            };
            let midpoint_is_root = midpoint_sign == Ordering::Equal;
            let Some(left_variation) = left_coefficients.variations(config.policy) else {
                *undecided = Some("could not decide left Bernstein coefficient signs".to_owned());
                return;
            };
            let Some(right_variation) = right_coefficients.variations(config.policy) else {
                *undecided = Some("could not decide right Bernstein coefficient signs".to_owned());
                return;
            };
            subdivide_bernstein_interval(
                BernsteinSubdivisionNode {
                    lower,
                    upper: midpoint.clone(),
                    coefficients: left_coefficients,
                    variation_bound: left_variation,
                    root_at_lower,
                    root_at_upper: midpoint_is_root,
                },
                config,
                depth + 1,
                intervals,
                hit_depth_limit,
                undecided,
            );
            subdivide_bernstein_interval(
                BernsteinSubdivisionNode {
                    lower: midpoint,
                    upper,
                    coefficients: right_coefficients,
                    variation_bound: right_variation,
                    root_at_lower: midpoint_is_root,
                    root_at_upper,
                },
                config,
                depth + 1,
                intervals,
                hit_depth_limit,
                undecided,
            );
        }
    }
}

fn push_unique_bernstein_endpoint(intervals: &mut Vec<BernsteinSubdivisionInterval>, root: Real) {
    if intervals
        .iter()
        .any(|interval| interval.exact_root.as_ref() == Some(&root))
    {
        return;
    }
    intervals.push(BernsteinSubdivisionInterval {
        lower: root.clone(),
        upper: root.clone(),
        exact_root: Some(root),
        variation_bound: Some(0),
        status: BernsteinSubdivisionIntervalStatus::EndpointRoot,
    });
}

fn root_isolation_report(
    constraint_index: usize,
    symbol: Option<SymbolId>,
    degree: Option<usize>,
    status: RootIsolationStatus,
    multiplicity: Option<RootMultiplicityStatus>,
    intervals: Vec<IsolatedRootInterval>,
    message: Option<String>,
) -> UnivariateRootIsolationReport {
    UnivariateRootIsolationReport {
        constraint_index,
        symbol,
        degree,
        status,
        multiplicity,
        intervals,
        message,
    }
}

#[allow(clippy::too_many_arguments)]
fn bernstein_report(
    constraint_index: usize,
    symbol: Option<SymbolId>,
    degree: Option<usize>,
    lower: Real,
    upper: Real,
    status: BernsteinRootCountStatus,
    bernstein_coefficients: Vec<Real>,
    root_at_lower: Option<bool>,
    root_at_upper: Option<bool>,
    variation_bound: Option<usize>,
    message: Option<String>,
) -> BernsteinRootCountReport {
    BernsteinRootCountReport {
        constraint_index,
        symbol,
        degree,
        lower,
        upper,
        status,
        bernstein_coefficients,
        root_count_parity: variation_bound.map(|count| count % 2),
        variation_bound,
        root_at_lower,
        root_at_upper,
        message,
    }
}

#[allow(clippy::too_many_arguments)]
fn bernstein_subdivision_report(
    constraint_index: usize,
    symbol: Option<SymbolId>,
    degree: Option<usize>,
    lower: Real,
    upper: Real,
    status: BernsteinSubdivisionStatus,
    intervals: Vec<BernsteinSubdivisionInterval>,
    message: Option<String>,
) -> BernsteinSubdivisionReport {
    BernsteinSubdivisionReport {
        constraint_index,
        symbol,
        degree,
        lower,
        upper,
        status,
        intervals,
        message,
    }
}

#[allow(clippy::too_many_arguments)]
fn descartes_report(
    constraint_index: usize,
    symbol: Option<SymbolId>,
    degree: Option<usize>,
    status: DescartesRootCountStatus,
    zero_root_multiplicity: Option<usize>,
    positive_variations: Option<usize>,
    negative_variations: Option<usize>,
    message: Option<String>,
) -> DescartesRootCountReport {
    DescartesRootCountReport {
        constraint_index,
        symbol,
        degree,
        status,
        zero_root_multiplicity,
        positive_root_count_parity: positive_variations.map(|count| count % 2),
        positive_variations,
        negative_root_count_parity: negative_variations.map(|count| count % 2),
        negative_variations,
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::context_from_problem;
    use crate::model::Constraint;
    use proptest::prelude::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn nonzero_constant_sturm_chains_need_no_derivative() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for constant in [
                Real::pi(),
                -Real::pi(),
                real(2).sqrt().unwrap(),
                -real(2).sqrt().unwrap(),
                real(5),
                real(-5),
            ] {
                for padding in 0..=2 {
                    let mut polynomial = vec![constant.clone()];
                    polynomial.resize(padding + 1, Real::zero());
                    let sequence = UnivariateSturmSequence::new(&polynomial, policy)
                        .expect("a certified nonzero constant owns an exact one-row Sturm chain");
                    assert_eq!(sequence.terminal_polynomial().len(), 1);
                    for point in [real(-1), Real::zero(), Real::pi()] {
                        assert_eq!(
                            sequence.classify_point(&point, policy),
                            Some(UnivariateSturmPoint::NonRoot(0))
                        );
                    }
                    assert_eq!(
                        sequence.count_distinct_roots(&real(-1), &real(1), policy),
                        Some(0)
                    );
                }
            }
            assert!(UnivariateSturmSequence::new(&[], policy).is_none());
            assert!(UnivariateSturmSequence::new(&[Real::zero(), Real::zero()], policy).is_none());
        }
    }

    #[test]
    fn dyadic_fujiwara_bound_is_exact_and_tightens_large_middle_coefficients() {
        let huge = BigInt::one() << 300_usize;
        let polynomial = vec![
            Real::one(),
            Real::from(HyperRational::from_bigint(-huge)),
            Real::zero(),
            Real::one(),
        ];
        assert_eq!(
            power_of_two_fujiwara_bound(&polynomial),
            Some(Real::from(HyperRational::from_bigint(
                BigInt::one() << 152_usize
            )))
        );

        let rational_polynomial = vec![
            Real::from(HyperRational::fraction(1, 3).expect("nonzero denominator")),
            Real::zero(),
            Real::from(HyperRational::fraction(1, 5).expect("nonzero denominator")),
        ];
        assert_eq!(
            power_of_two_fujiwara_bound(&rational_polynomial),
            Some(real(4))
        );
    }

    proptest! {
        #[test]
        fn dyadic_fujiwara_bound_contains_generated_rational_roots(
            roots in prop::collection::vec(-64_i16..=64, 1..=6),
            scale in 1_i16..=16,
        ) {
            let mut polynomial = vec![real(i64::from(scale))];
            for root in &roots {
                let root = real(i64::from(*root));
                let mut next = vec![Real::zero(); polynomial.len() + 1];
                for (index, coefficient) in polynomial.iter().enumerate() {
                    next[index] = next[index].clone() - coefficient.clone() * root.clone();
                    next[index + 1] = next[index + 1].clone() + coefficient.clone();
                }
                polynomial = next;
            }

            let bound = power_of_two_fujiwara_bound(&polynomial)
                .expect("generated integer polynomial has a bound");
            for root in roots {
                prop_assert!(real(i64::from(root)).abs() < bound);
            }
        }
    }

    #[test]
    fn fixed_degree_remainder_does_not_predicate_on_unresolved_correlated_coefficients() {
        let sqrt_two = real(2).sqrt().expect("positive square root");
        let sqrt_three = real(3).sqrt().expect("positive square root");
        let sum = &sqrt_two + &sqrt_three;
        let radical_zero =
            &sum * &sum - (real(5) + real(2) * real(6).sqrt().expect("positive square root"));
        assert!(!radical_zero.definitely_zero());

        let modulus = [real(-2), Real::zero(), Real::one()];
        let radical_dividend = vec![Real::zero(), Real::zero(), radical_zero, Real::one()];
        let (quotient, remainder) =
            polynomial_div_rem(radical_dividend, &modulus, PredicatePolicy::STRICT)
                .expect("bounded algebraic separation now certifies the radical identity");
        assert_eq!(quotient, vec![Real::zero(), Real::one()]);
        assert_eq!(remainder, vec![Real::zero(), real(2)]);

        // Keep the fixed-degree contract covered by an exact identity outside
        // the bounded algebraic certificate surface. Generic trimming cannot
        // classify this coefficient, while quotient-ring reduction must not
        // ask for its sign.
        let sine = real(1).sin();
        let cosine = real(1).cos();
        let unresolved_zero = &sine * &sine + &cosine * &cosine - real(1);
        assert!(!unresolved_zero.definitely_zero());
        let dividend = vec![Real::zero(), Real::zero(), unresolved_zero, Real::one()];
        assert!(
            polynomial_div_rem(dividend.clone(), &modulus, PredicatePolicy::STRICT).is_none(),
            "generic division should expose the unresolved leading cancellation"
        );
        let divisor = CertifiedPolynomialDivisor::new(&modulus, PredicatePolicy::STRICT)
            .expect("the quadratic modulus is certified once");
        let mut remainder = dividend;
        divisor.remainder_in_place(&mut remainder);
        assert_eq!(remainder.len(), 2);
        assert_eq!(remainder[1], real(2));
    }

    #[test]
    fn square_free_part_preserves_rational_and_exact_real_gcd_paths() {
        assert_eq!(
            square_free_part(
                vec![Real::one(), real(-2), Real::one()],
                PredicatePolicy::STRICT,
            )
            .unwrap(),
            vec![real(-1), Real::one()]
        );

        let sqrt_two = real(2).sqrt().expect("positive square root");
        let square_free = square_free_part(
            vec![real(2), real(-2) * &sqrt_two, Real::one()],
            PredicatePolicy::STRICT,
        )
        .expect("the exact-real repeated quadratic has a square-free part");
        assert_eq!(square_free.len(), 2);
        assert_eq!(
            compare_reals(&square_free[0], &(-sqrt_two), PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(square_free[1], Real::one());
    }

    proptest! {
        #[test]
        fn polynomial_division_recovers_generated_exact_products(
            divisor in prop::collection::vec(-5_i16..=5, 1..=8),
            quotient in prop::collection::vec(-5_i16..=5, 1..=8),
            divisor_padding in 0_usize..=3,
            dividend_padding in 0_usize..=3,
        ) {
            prop_assume!(divisor.last().is_some_and(|coefficient| *coefficient != 0));
            prop_assume!(quotient.last().is_some_and(|coefficient| *coefficient != 0));
            let mut divisor = divisor
                .into_iter()
                .map(|coefficient| real(i64::from(coefficient)))
                .collect::<Vec<_>>();
            let expected = quotient
                .into_iter()
                .map(|coefficient| real(i64::from(coefficient)))
                .collect::<Vec<_>>();
            let mut dividend = vec![Real::zero(); divisor.len() + expected.len() - 1];
            for (left_power, left) in divisor.iter().enumerate() {
                for (right_power, right) in expected.iter().enumerate() {
                    dividend[left_power + right_power] += left * right;
                }
            }
            dividend.extend((0..dividend_padding).map(|_| Real::zero()));
            divisor.extend((0..divisor_padding).map(|_| Real::zero()));

            let (borrowed, borrowed_remainder) = polynomial_div_rem_borrowed_divisor(
                dividend.clone(),
                &divisor,
                PredicatePolicy::STRICT,
            )
            .expect("borrowed generated exact product is divisible");
            prop_assert_eq!(borrowed, expected.clone());
            prop_assert_eq!(borrowed_remainder, vec![Real::zero()]);

            let (owned, remainder) = polynomial_div_rem(
                dividend,
                &divisor,
                PredicatePolicy::STRICT,
            )
            .expect("owned generated exact product is divisible");
            prop_assert_eq!(owned, expected);
            prop_assert_eq!(remainder, vec![Real::zero()]);
        }
    }

    #[test]
    fn fraction_free_bernstein_exceeds_machine_binomial_range_exactly() {
        // (x - 1/2)(x + 1)^79 has exactly one root in (2/5, 3/5). Degree 80
        // makes its central binomial coefficient larger than u64, exercising
        // the fraction-free falling-factorial basis conversion directly.
        let mut repeated_factor = vec![Real::one()];
        for _ in 0..79 {
            let mut next = vec![Real::zero(); repeated_factor.len() + 1];
            for (index, coefficient) in repeated_factor.iter().enumerate() {
                next[index] += coefficient.clone();
                next[index + 1] += coefficient.clone();
            }
            repeated_factor = next;
        }
        let half = (real(1) / real(2)).unwrap();
        let mut polynomial = vec![Real::zero(); repeated_factor.len() + 1];
        for (index, coefficient) in repeated_factor.iter().enumerate() {
            polynomial[index] -= half.clone() * coefficient.clone();
            polynomial[index + 1] += coefficient.clone();
        }

        assert_eq!(
            exact_rational_polynomial_interval_bernstein_variations(
                &polynomial,
                &(real(2) / real(5)).unwrap(),
                &(real(3) / real(5)).unwrap(),
            ),
            Some(1)
        );
    }

    #[test]
    fn open_interval_one_root_falls_back_after_inconclusive_bernstein_bound() {
        // In Bernstein form on [0, 1], this cubic has controls
        // [-2, 1, -1, 2], hence three sign variations. Its derivative
        // 30*x^2 - 30*x + 9 is strictly positive (negative discriminant), so
        // x = 1/2 is nevertheless its only real root. Bernstein variation
        // three is an upper bound of matching parity, not a proof that the
        // open interval contains something other than one distinct root.
        let polynomial = [real(-2), real(9), real(-15), real(10)];
        assert_eq!(
            exact_rational_polynomial_interval_bernstein_variations(
                &polynomial,
                &Real::zero(),
                &Real::one(),
            ),
            Some(3)
        );
        let square_free = square_free_part(polynomial.to_vec(), PredicatePolicy::STRICT)
            .expect("the cubic is square-free");
        let sturm = UnivariateSturmSequence::new(&square_free, PredicatePolicy::STRICT)
            .expect("the exact cubic has a Sturm sequence");
        assert_eq!(
            sturm.count_distinct_roots(&Real::zero(), &Real::one(), PredicatePolicy::STRICT,),
            Some(1)
        );
        assert_eq!(
            sturm.classify_point(&(real(1) / real(2)).unwrap(), PredicatePolicy::STRICT,),
            Some(UnivariateSturmPoint::Root)
        );
        assert_eq!(
            polynomial_has_one_distinct_root_in_open_interval(
                &polynomial,
                &Real::zero(),
                &Real::one(),
                PredicatePolicy::STRICT,
            ),
            Some(true)
        );
    }

    #[test]
    fn quadratic_multiple_bernstein_variation_distinguishes_repeated_root() {
        let cases = [
            // (2*x - 1)^2: one repeated interior root.
            ([real(1), real(-4), real(4)], true),
            // (4*x - 1)(4*x - 3): two distinct interior roots.
            ([real(3), real(-16), real(16)], false),
            // 20*x^2 - 20*x + 7: variation two but negative discriminant.
            ([real(7), real(-20), real(20)], false),
        ];
        for (polynomial, expected) in cases {
            assert_eq!(
                exact_rational_polynomial_interval_bernstein_variations(
                    &polynomial,
                    &Real::zero(),
                    &Real::one(),
                ),
                Some(2)
            );
            assert_eq!(
                polynomial_has_one_distinct_root_in_open_interval(
                    &polynomial,
                    &Real::zero(),
                    &Real::one(),
                    PredicatePolicy::STRICT,
                ),
                Some(expected)
            );
        }
    }

    #[test]
    fn open_interval_one_root_excludes_endpoint_roots_without_discarding_interior_roots() {
        let cases = [
            // x(2x - 1): lower endpoint plus one interior root.
            vec![Real::zero(), real(-1), real(2)],
            // (x - 1)(2x - 1): upper endpoint plus one interior root.
            vec![Real::one(), real(-3), real(2)],
            // x(x - 1)(2x - 1): both endpoints plus one interior root.
            vec![Real::zero(), Real::one(), real(-3), real(2)],
        ];
        for polynomial in cases {
            assert_eq!(
                polynomial_has_one_distinct_root_in_open_interval(
                    &polynomial,
                    &Real::zero(),
                    &Real::one(),
                    PredicatePolicy::STRICT,
                ),
                Some(true)
            );
        }

        assert_eq!(
            polynomial_has_one_distinct_root_in_open_interval(
                &[Real::zero(), Real::one()],
                &Real::zero(),
                &Real::one(),
                PredicatePolicy::STRICT,
            ),
            Some(false),
            "an endpoint root alone is not an open-interval root"
        );
    }

    #[test]
    fn algebraic_image_admission_uses_half_open_ownership_and_replays_exact_witnesses() {
        let source_polynomial = [real(-1), Real::one()];
        let source_interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: real(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        let candidate = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::one(),
            exact_root: None,
            distinct_root_count: 0,
        };

        let lower_endpoint_and_interior = [Real::zero(), real(-1), real(2)];
        let accepted = certify_algebraic_image_interval(
            &source_polynomial,
            &source_interval,
            &lower_endpoint_and_interior,
            PredicatePolicy::STRICT,
            |_| Some(candidate.clone()),
        )
        .expect("the excluded lower endpoint does not add an owned root");
        assert_eq!(accepted.distinct_root_count, 1);

        let upper_endpoint_and_interior = [Real::one(), real(-3), real(2)];
        assert!(
            certify_algebraic_image_interval(
                &source_polynomial,
                &source_interval,
                &upper_endpoint_and_interior,
                PredicatePolicy::STRICT,
                |_| Some(candidate.clone()),
            )
            .is_none(),
            "the included upper endpoint makes the owned interval nonunit"
        );

        let half = Real::average_pair(&Real::zero(), &Real::one());
        let valid_witness = IsolatedRootInterval {
            exact_root: Some(half.clone()),
            ..candidate.clone()
        };
        let exact = certify_algebraic_image_interval(
            &source_polynomial,
            &source_interval,
            &[real(-1), real(2)],
            PredicatePolicy::STRICT,
            |_| Some(valid_witness.clone()),
        )
        .expect("a valid exact image witness should be accepted");
        assert_eq!(exact.lower, half);
        assert_eq!(exact.lower, exact.upper);
        assert_eq!(exact.exact_root, Some(exact.lower.clone()));
        assert!(
            certify_algebraic_image_interval(
                &source_polynomial,
                &source_interval,
                &[Real::zero()],
                PredicatePolicy::STRICT,
                |_| Some(valid_witness.clone()),
            )
            .is_none(),
            "an exact witness cannot turn the zero polynomial into an isolated root"
        );

        for stale_root in [real(2), Real::average_pair(&Real::zero(), &half)] {
            let stale = IsolatedRootInterval {
                exact_root: Some(stale_root),
                ..candidate.clone()
            };
            assert!(
                certify_algebraic_image_interval(
                    &source_polynomial,
                    &source_interval,
                    &[real(-1), real(2)],
                    PredicatePolicy::STRICT,
                    |_| Some(stale.clone()),
                )
                .is_none(),
                "outside and nonvanishing exact witnesses must fail closed"
            );
        }
    }

    #[test]
    fn interval_presence_helpers_preserve_exact_real_and_undecided_boundaries() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let linear = vec![-sqrt_two.clone(), Real::one()];
        assert_eq!(
            polynomial_has_one_distinct_root_in_open_interval(
                &linear,
                &Real::one(),
                &real(2),
                PredicatePolicy::STRICT,
            ),
            Some(true)
        );
        assert_eq!(
            polynomial_has_no_distinct_root_in_closed_interval(
                &linear,
                &Real::zero(),
                &Real::one(),
                PredicatePolicy::STRICT,
            ),
            Some(true)
        );
        let repeated = vec![real(2), real(-2) * &sqrt_two, Real::one()];
        assert_eq!(
            polynomial_has_one_distinct_root_in_open_interval(
                &repeated,
                &Real::one(),
                &real(2),
                PredicatePolicy::STRICT,
            ),
            Some(true)
        );

        let sine = Real::one().sin();
        let cosine = Real::one().cos();
        let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
        assert_eq!(
            polynomial_has_one_distinct_root_in_open_interval(
                core::slice::from_ref(&unresolved_zero),
                &real(-1),
                &Real::one(),
                PredicatePolicy::STRICT,
            ),
            None
        );
        assert_eq!(
            polynomial_has_no_distinct_root_in_closed_interval(
                core::slice::from_ref(&unresolved_zero),
                &Real::zero(),
                &Real::zero(),
                PredicatePolicy::STRICT,
            ),
            None
        );
    }

    proptest! {
        #[test]
        fn generated_interval_presence_helpers_match_distinct_root_sets(
            roots in prop::collection::vec(-5_i8..=5, 1..=7),
            first_bound in -6_i8..=6,
            second_bound in -6_i8..=6,
            denominator in 1_u8..=5,
        ) {
            let mut polynomial = vec![Real::one()];
            for root in &roots {
                let root = Real::from(
                    HyperRational::fraction(i64::from(*root), u64::from(denominator))
                        .expect("generated denominator is positive"),
                );
                let mut next = vec![Real::zero(); polynomial.len() + 1];
                for (index, coefficient) in polynomial.iter().enumerate() {
                    next[index] -= coefficient * &root;
                    next[index + 1] += coefficient;
                }
                polynomial = next;
            }
            let mut distinct_roots = roots;
            distinct_roots.sort_unstable();
            distinct_roots.dedup();
            let (lower, upper) = if first_bound <= second_bound {
                (first_bound, second_bound)
            } else {
                (second_bound, first_bound)
            };
            let expected_open = lower < upper
                && distinct_roots
                    .iter()
                    .filter(|root| lower < **root && **root < upper)
                    .count()
                    == 1;
            let expected_half_open = lower < upper
                && distinct_roots
                    .iter()
                    .filter(|root| lower < **root && **root <= upper)
                    .count()
                    == 1;
            let expected_closed_empty = distinct_roots
                .iter()
                .all(|root| *root < lower || upper < *root);
            let lower = Real::from(
                HyperRational::fraction(i64::from(lower), u64::from(denominator))
                    .expect("generated denominator is positive"),
            );
            let upper = Real::from(
                HyperRational::fraction(i64::from(upper), u64::from(denominator))
                    .expect("generated denominator is positive"),
            );

            prop_assert_eq!(
                polynomial_has_one_distinct_root_in_open_interval(
                    &polynomial,
                    &lower,
                    &upper,
                    PredicatePolicy::STRICT,
                ),
                Some(expected_open)
            );
            prop_assert_eq!(
                polynomial_has_one_distinct_root_with_upper_ownership(
                    &polynomial,
                    &lower,
                    &upper,
                    PredicatePolicy::STRICT,
                    UpperEndpointOwnership::Included,
                ),
                Some(expected_half_open)
            );
            prop_assert_eq!(
                polynomial_has_no_distinct_root_in_closed_interval(
                    &polynomial,
                    &lower,
                    &upper,
                    PredicatePolicy::STRICT,
                ),
                Some(expected_closed_empty)
            );
        }
    }

    #[test]
    fn sturm_isolates_distinct_repeated_and_no_real_roots() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "three roots",
            x.clone().powi(3) - Expr::int(6) * x.clone().powi(2) + Expr::int(11) * x.clone()
                - Expr::int(6),
        ));
        problem.add_constraint(Constraint::equality(
            "repeated root",
            (x.clone() - Expr::int(2)).powi(2) * (x.clone() + Expr::int(1)),
        ));
        problem.add_constraint(Constraint::equality(
            "no real roots",
            x.powi(2) + Expr::int(1),
        ));

        let reports = isolate_univariate_polynomial_roots(
            &problem.analyze(),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].status, RootIsolationStatus::Isolated);
        assert_eq!(reports[0].intervals.len(), 3);
        assert_eq!(
            reports[0]
                .intervals
                .iter()
                .filter_map(|interval| interval.exact_root.clone())
                .collect::<Vec<_>>(),
            vec![real(1), real(2), real(3)]
        );
        assert!(
            reports[0]
                .intervals
                .windows(2)
                .all(|pair| pair[0].upper < pair[1].lower)
        );
        assert_eq!(
            reports[0].multiplicity,
            Some(RootMultiplicityStatus::SquareFree)
        );
        assert_eq!(reports[1].status, RootIsolationStatus::MultipleRoot);
        assert_eq!(
            reports[1].multiplicity,
            Some(RootMultiplicityStatus::RepeatedRootsDetected { gcd_degree: 1 })
        );
        assert_eq!(reports[1].intervals.len(), 2);
        assert_eq!(reports[2].status, RootIsolationStatus::NoRealRoots);
        assert!(reports[2].intervals.is_empty());
    }

    #[test]
    fn sturm_rejects_multivariate_rows_explicitly() {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_variable("y", real(0));
        problem.add_constraint(Constraint::equality("xy", x * y));

        let reports = isolate_univariate_polynomial_roots(
            &problem.analyze(),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(reports.len(), 1);
        assert_eq!(
            reports[0].status,
            RootIsolationStatus::UnsupportedCoefficient
        );
        assert!(reports[0].message.is_some());
    }

    #[test]
    fn bounded_refinement_and_rational_witness_replay_are_explicit() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "root minus one or one",
            x.clone().powi(2) - Expr::int(1),
        ));
        problem.add_constraint(Constraint::equality("select root one", x - Expr::int(1)));
        let analysis = problem.analyze();
        let reports = isolate_univariate_polynomial_roots_with_config(
            &analysis,
            RootIsolationConfig {
                policy: PredicatePolicy::APPROXIMATE_512,
                max_interval_width: Some(Real::one()),
                max_refinement_steps: 8,
            },
        );

        assert_eq!(reports[0].intervals.len(), 2);
        assert!(
            reports[0]
                .intervals
                .iter()
                .any(|interval| interval.exact_root == Some(real(1)))
        );
        let candidates = certify_isolated_rational_root_witnesses(
            &analysis,
            &context_from_problem(&problem),
            &reports,
        );

        assert!(candidates.iter().any(|candidate| {
            candidate.exact_root == Some(real(1))
                && candidate.status == AlgebraicRootCandidateStatus::ReplayCertified
        }));
        assert!(candidates.iter().any(|candidate| {
            candidate.exact_root == Some(real(-1))
                && candidate.status == AlgebraicRootCandidateStatus::ReplayRejected
        }));
    }

    #[test]
    fn isolated_interval_refinement_separates_nested_quadratic_roots() {
        let report = refine_isolated_univariate_polynomial_interval(
            &[real(-2), Real::zero(), Real::one()],
            &IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig {
                max_refinement_steps: 4,
                ..RootIsolationConfig::default()
            },
        );

        assert_eq!(report.status, IsolatedRootRefinementStatus::Refined);
        assert_eq!(report.refinement_steps, 4);
        let refined = report.refined_interval.expect("refined interval");
        assert_eq!(refined.distinct_root_count, 1);
        assert!(refined.lower >= real(1));
        assert!(refined.upper <= real(2));
    }

    #[test]
    fn isolated_interval_refinement_rejects_bad_evidence_antagonistically() {
        let bad_count = refine_isolated_univariate_polynomial_interval(
            &[real(-1), Real::zero(), Real::one()],
            &IsolatedRootInterval {
                lower: real(-2),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 2,
            },
            RootIsolationConfig::default(),
        );
        assert_eq!(
            bad_count.status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );

        let bad_polynomial = refine_isolated_univariate_polynomial_interval(
            &[real(0)],
            &IsolatedRootInterval {
                lower: real(0),
                upper: real(1),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig::default(),
        );
        assert_eq!(
            bad_polynomial.status,
            IsolatedRootRefinementStatus::InvalidPolynomial
        );
    }

    #[test]
    fn isolated_interval_refinement_strictly_replays_exact_witnesses() {
        let polynomial = [real(-2), Real::one()];
        let report = |lower: i64, upper: i64, root: i64| {
            refine_isolated_univariate_polynomial_interval(
                &polynomial,
                &IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                RootIsolationConfig::default(),
            )
        };

        assert_eq!(
            report(3, 1, 2).status,
            IsolatedRootRefinementStatus::InvalidInterval
        );
        assert_eq!(
            report(0, 1, 2).status,
            IsolatedRootRefinementStatus::InvalidInterval
        );
        assert_eq!(
            report(1, 4, 3).status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );
        assert_eq!(
            report(2, 3, 2).status,
            IsolatedRootRefinementStatus::ExactRoot
        );

        let constant = refine_isolated_univariate_polynomial_interval(
            &[Real::zero()],
            &IsolatedRootInterval {
                lower: Real::zero(),
                upper: Real::zero(),
                exact_root: Some(Real::zero()),
                distinct_root_count: 1,
            },
            RootIsolationConfig::default(),
        );
        assert_eq!(
            constant.status,
            IsolatedRootRefinementStatus::InvalidPolynomial
        );

        let nonunique = refine_isolated_univariate_polynomial_interval(
            &[real(6), real(-5), Real::one()],
            &IsolatedRootInterval {
                lower: Real::one(),
                upper: real(4),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            RootIsolationConfig::default(),
        );
        assert_eq!(
            nonunique.status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );

        let valid = refine_isolated_univariate_polynomial_interval(
            &[real(-4), Real::zero(), Real::one()],
            &IsolatedRootInterval {
                lower: Real::one(),
                upper: real(3),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            RootIsolationConfig::default(),
        );
        assert_eq!(valid.status, IsolatedRootRefinementStatus::ExactRoot);
        let exact = valid.refined_interval.expect("replayed exact witness");
        assert_eq!(exact.lower, real(2));
        assert_eq!(exact.upper, real(2));
    }

    #[test]
    fn isolated_interval_refinement_uses_half_open_endpoint_ownership() {
        let refine = |polynomial: &[Real]| {
            refine_isolated_univariate_polynomial_interval(
                polynomial,
                &IsolatedRootInterval {
                    lower: Real::one(),
                    upper: real(2),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                RootIsolationConfig::default(),
            )
        };

        let lower = refine(&[real(-1), Real::zero(), Real::one()]);
        assert_eq!(lower.status, IsolatedRootRefinementStatus::NonUnitIsolation);
        assert!(lower.refined_interval.is_none());

        let upper = refine(&[real(-4), Real::zero(), Real::one()]);
        assert_eq!(upper.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(
            upper
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some(real(2))
        );

        let both = refine(&[real(2), real(-3), Real::one()]);
        assert_eq!(both.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(
            both.refined_interval
                .and_then(|interval| interval.exact_root),
            Some(real(2))
        );

        let endpoint_and_interior = refine_isolated_univariate_polynomial_interval(
            &[real(3), real(-5), real(2)],
            &IsolatedRootInterval {
                lower: Real::one(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig {
                max_refinement_steps: 1,
                ..RootIsolationConfig::default()
            },
        );
        assert_eq!(
            endpoint_and_interior.status,
            IsolatedRootRefinementStatus::ExactRoot
        );
        assert_eq!(endpoint_and_interior.refinement_steps, 1);
        assert_eq!(
            endpoint_and_interior
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some((real(3) / real(2)).expect("nonzero divisor"))
        );

        let midpoint = refine_isolated_univariate_polynomial_interval(
            &[
                (real(-9) / real(4)).expect("nonzero divisor"),
                Real::zero(),
                Real::one(),
            ],
            &IsolatedRootInterval {
                lower: Real::one(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig {
                max_refinement_steps: 4,
                ..RootIsolationConfig::default()
            },
        );
        assert_eq!(midpoint.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(midpoint.refinement_steps, 1);
        assert_eq!(
            midpoint
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some((real(3) / real(2)).expect("nonzero divisor"))
        );
    }

    proptest! {
        #[test]
        fn isolated_interval_refinement_preserves_generated_repeated_root_carriers(
            numerator in -31_i16..=31,
            denominator in prop_oneof![Just(3_i16), Just(5_i16), Just(7_i16)],
            root_multiplicity in 1_usize..=3,
            outside_multiplicity in 1_usize..=2,
        ) {
            prop_assume!(numerator % denominator != 0);
            let root = Real::from(
                HyperRational::fraction(
                    i64::from(numerator),
                    u64::try_from(denominator).expect("generated positive denominator"),
                )
                    .expect("positive denominator"),
            );
            let outside = &root + real(4);
            let multiply = |left: &[Real], right: &[Real]| {
                let mut product = vec![Real::zero(); left.len() + right.len() - 1];
                for (left_power, left_coefficient) in left.iter().enumerate() {
                    for (right_power, right_coefficient) in right.iter().enumerate() {
                        product[left_power + right_power] +=
                            left_coefficient * right_coefficient;
                    }
                }
                product
            };
            let root_factor = [-root.clone(), Real::one()];
            let outside_factor = [-outside, Real::one()];
            let mut polynomial = vec![Real::one()];
            for _ in 0..root_multiplicity {
                polynomial = multiply(&polynomial, &root_factor);
            }
            for _ in 0..outside_multiplicity {
                polynomial = multiply(&polynomial, &outside_factor);
            }
            let lower = &root - real(1);
            let upper = &root + real(2);
            let report = refine_isolated_univariate_polynomial_interval(
                &polynomial,
                &IsolatedRootInterval {
                    lower,
                    upper,
                    exact_root: None,
                    distinct_root_count: 1,
                },
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 8,
                },
            );

            prop_assert_eq!(report.status, IsolatedRootRefinementStatus::Refined);
            prop_assert_eq!(report.refinement_steps, 8);
            let refined = report.refined_interval.expect("generated interval refines");
            prop_assert!(refined.lower < root);
            prop_assert!(root < refined.upper);
            prop_assert_eq!(
                &refined.upper - &refined.lower,
                Real::from(HyperRational::fraction(3, 256).expect("positive denominator")),
            );
        }

        #[test]
        fn isolated_interval_refinement_matches_generated_half_open_quadratic_counts(
            first_root in -8_i16..=8,
            second_root in -8_i16..=8,
            lower in -12_i16..=11,
            width in 1_i16..=8,
        ) {
            prop_assume!(first_root != second_root);
            let upper = lower + width;
            let first_root = i64::from(first_root);
            let second_root = i64::from(second_root);
            let lower = i64::from(lower);
            let upper = i64::from(upper);
            let polynomial = [
                real(first_root * second_root),
                real(-(first_root + second_root)),
                Real::one(),
            ];
            let roots = [first_root, second_root]
                .into_iter()
                .filter(|root| lower < *root && *root <= upper)
                .collect::<Vec<_>>();
            let report = refine_isolated_univariate_polynomial_interval(
                &polynomial,
                &IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 4,
                },
            );

            if let [root] = roots.as_slice() {
                prop_assert!(matches!(
                    report.status,
                    IsolatedRootRefinementStatus::Refined
                        | IsolatedRootRefinementStatus::ExactRoot
                ));
                let refined = report.refined_interval.expect("unit interval refines");
                if let Some(exact_root) = refined.exact_root {
                    prop_assert_eq!(exact_root, real(*root));
                } else {
                    prop_assert!(refined.lower < real(*root));
                    prop_assert!(real(*root) < refined.upper);
                }
            } else {
                prop_assert_eq!(
                    report.status,
                    IsolatedRootRefinementStatus::NonUnitIsolation,
                );
                prop_assert!(report.refined_interval.is_none());
            }
        }
    }

    #[test]
    fn isolated_interval_refinement_solves_trimmed_linear_carriers_exactly() {
        let refine = |lower, upper| {
            refine_isolated_univariate_polynomial_interval(
                &[real(-6), real(3), Real::zero(), Real::zero()],
                &IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                RootIsolationConfig::default(),
            )
        };

        let contained = refine(1, 3);
        assert_eq!(contained.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(contained.refinement_steps, 0);
        assert_eq!(
            contained
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some(real(2)),
        );

        let outside = refine(3, 4);
        assert_eq!(
            outside.status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );
        assert!(outside.refined_interval.is_none());

        let excluded_lower = refine(2, 3);
        assert_eq!(
            excluded_lower.status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );
        let included_upper = refine(1, 2);
        assert_eq!(
            included_upper
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some(real(2))
        );
    }

    #[test]
    fn isolated_interval_refinement_obeys_width_and_step_budgets_exactly() {
        let polynomial = [real(-2), Real::zero(), Real::one()];
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: real(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        let refine = |max_interval_width, max_refinement_steps| {
            refine_isolated_univariate_polynomial_interval(
                &polynomial,
                &interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width,
                    max_refinement_steps,
                },
            )
        };

        let validated = refine(None, 0);
        assert_eq!(validated.status, IsolatedRootRefinementStatus::Refined);
        assert_eq!(validated.refinement_steps, 0);
        assert_eq!(validated.refined_interval, Some(interval.clone()));

        let already_narrow = refine(Some(Real::one()), 8);
        assert_eq!(already_narrow.refinement_steps, 0);
        assert_eq!(already_narrow.refined_interval, Some(interval.clone()));

        let quarter = (Real::one() / real(4)).expect("nonzero divisor");
        let width_limited = refine(Some(quarter.clone()), 8);
        assert_eq!(width_limited.refinement_steps, 2);
        let width_limited = width_limited
            .refined_interval
            .expect("width-limited interval");
        assert_eq!(&width_limited.upper - &width_limited.lower, quarter);

        let step_limited = refine(Some((Real::one() / real(16)).expect("nonzero divisor")), 1);
        assert_eq!(step_limited.refinement_steps, 1);
        let step_limited = step_limited
            .refined_interval
            .expect("step-limited interval");
        assert_eq!(
            &step_limited.upper - &step_limited.lower,
            (Real::one() / real(2)).expect("nonzero divisor"),
        );

        let unreachable_width = refine(Some(real(-1)), 3);
        assert_eq!(unreachable_width.refinement_steps, 3);
        let unreachable_width = unreachable_width
            .refined_interval
            .expect("step cap still returns the valid interval");
        assert_eq!(
            &unreachable_width.upper - &unreachable_width.lower,
            (Real::one() / real(8)).expect("nonzero divisor"),
        );
    }

    #[test]
    fn isolated_interval_refinement_accepts_exact_real_coefficient_fields() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let polynomial = vec![-sqrt_two, Real::one()];
        let refinement = refine_isolated_univariate_polynomial_interval(
            &polynomial,
            &IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: Some((Real::one() / real(4)).expect("nonzero divisor")),
                max_refinement_steps: 4,
            },
        );
        assert_eq!(refinement.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(refinement.refinement_steps, 0);
        assert_eq!(
            refinement
                .refined_interval
                .and_then(|interval| interval.exact_root),
            Some(real(2).sqrt().expect("positive exact square root")),
        );
    }

    #[test]
    fn descartes_counts_positive_negative_and_zero_root_bounds() {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_variable("y", real(0));
        problem.add_constraint(Constraint::equality(
            "mixed signs",
            x.clone().powi(3) - Expr::int(6) * x.clone().powi(2) + Expr::int(11) * x.clone()
                - Expr::int(6),
        ));
        problem.add_constraint(Constraint::equality(
            "zero and negative roots",
            x.clone() * (x.clone() + Expr::int(2)),
        ));
        problem.add_constraint(Constraint::equality("multivariate unsupported", x * y));

        let reports = count_descartes_univariate_polynomial_roots(
            &problem.analyze(),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].status, DescartesRootCountStatus::Counted);
        assert_eq!(reports[0].positive_variations, Some(3));
        assert_eq!(reports[0].positive_root_count_parity, Some(1));
        assert_eq!(reports[0].negative_variations, Some(0));
        assert_eq!(reports[0].negative_root_count_parity, Some(0));
        assert_eq!(reports[0].zero_root_multiplicity, Some(0));

        assert_eq!(reports[1].status, DescartesRootCountStatus::Counted);
        assert_eq!(reports[1].zero_root_multiplicity, Some(1));
        assert_eq!(reports[1].positive_variations, Some(0));
        assert_eq!(reports[1].negative_variations, Some(1));

        assert_eq!(
            reports[2].status,
            DescartesRootCountStatus::UnsupportedCoefficient
        );
    }

    #[test]
    fn bernstein_counts_interval_roots_and_endpoint_witnesses() {
        let x = Expr::symbol(SymbolId(0), "x");
        let y = Expr::symbol(SymbolId(1), "y");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_variable("y", real(0));
        problem.add_constraint(Constraint::equality(
            "three roots",
            x.clone().powi(3) - Expr::int(6) * x.clone().powi(2) + Expr::int(11) * x.clone()
                - Expr::int(6),
        ));
        problem.add_constraint(Constraint::equality(
            "endpoint root",
            x.clone() - Expr::int(2),
        ));
        problem.add_constraint(Constraint::equality("multivariate unsupported", x * y));

        let reports = count_bernstein_univariate_polynomial_interval_roots(
            &problem.analyze(),
            real(0),
            real(2),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].status, BernsteinRootCountStatus::Counted);
        assert_eq!(reports[0].variation_bound, Some(1));
        assert_eq!(reports[0].root_count_parity, Some(1));
        assert_eq!(reports[0].root_at_lower, Some(false));
        assert_eq!(reports[0].root_at_upper, Some(true));
        assert_eq!(reports[0].bernstein_coefficients.len(), 4);

        assert_eq!(reports[1].status, BernsteinRootCountStatus::Counted);
        assert_eq!(reports[1].root_at_lower, Some(false));
        assert_eq!(reports[1].root_at_upper, Some(true));
        assert_eq!(reports[1].variation_bound, Some(0));

        assert_eq!(
            reports[2].status,
            BernsteinRootCountStatus::UnsupportedCoefficient
        );
    }

    #[test]
    fn bernstein_subdivision_partitions_empty_endpoint_isolating_and_depth_limit() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "three roots",
            x.clone().powi(3) - Expr::int(6) * x.clone().powi(2) + Expr::int(11) * x.clone()
                - Expr::int(6),
        ));
        problem.add_constraint(Constraint::equality(
            "empty interval",
            x.clone().powi(2) + Expr::int(1),
        ));
        let analysis = problem.analyze();
        let complete = subdivide_bernstein_univariate_polynomial_interval_roots(
            &analysis,
            real(0),
            real(4),
            BernsteinSubdivisionConfig {
                policy: PredicatePolicy::APPROXIMATE_512,
                max_depth: 8,
            },
        );

        assert_eq!(complete.len(), 2);
        assert_eq!(complete[0].status, BernsteinSubdivisionStatus::Completed);
        assert!(complete[0].intervals.iter().any(|interval| {
            interval.status == BernsteinSubdivisionIntervalStatus::EndpointRoot
                && interval.exact_root == Some(real(2))
        }));
        let endpoint_roots = complete[0]
            .intervals
            .iter()
            .filter(|interval| interval.status == BernsteinSubdivisionIntervalStatus::EndpointRoot)
            .count();
        assert_eq!(endpoint_roots, 3);
        for root in [real(1), real(2), real(3)] {
            assert!(
                complete[0]
                    .intervals
                    .iter()
                    .any(|interval| interval.exact_root.as_ref() == Some(&root))
            );
        }
        assert!(
            complete[1]
                .intervals
                .iter()
                .any(|interval| interval.status == BernsteinSubdivisionIntervalStatus::Empty)
        );

        let depth_limited = subdivide_bernstein_univariate_polynomial_interval_roots(
            &analysis,
            real(0),
            real(4),
            BernsteinSubdivisionConfig {
                policy: PredicatePolicy::APPROXIMATE_512,
                max_depth: 0,
            },
        );
        assert_eq!(
            depth_limited[0].status,
            BernsteinSubdivisionStatus::DepthLimit
        );
        assert!(depth_limited[0].intervals.iter().any(|interval| {
            interval.status == BernsteinSubdivisionIntervalStatus::DepthLimit
                && interval.variation_bound.unwrap_or(0) > 1
        }));
    }

    #[test]
    fn bernstein_subdivision_keeps_an_interior_root_beside_an_endpoint_root() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        problem.add_constraint(Constraint::equality(
            "endpoint and interior roots",
            x.clone() * (x - Expr::int(2)),
        ));

        let reports = subdivide_bernstein_univariate_polynomial_interval_roots(
            &problem.analyze(),
            real(0),
            real(4),
            BernsteinSubdivisionConfig {
                policy: PredicatePolicy::APPROXIMATE_512,
                max_depth: 4,
            },
        );

        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].status, BernsteinSubdivisionStatus::Completed);
        let exact_roots = reports[0]
            .intervals
            .iter()
            .filter_map(|interval| interval.exact_root.clone())
            .collect::<Vec<_>>();
        assert_eq!(exact_roots, vec![real(0), real(2)]);
    }

    // Closed-form boundary sums provide an oracle independent of the
    // adjacent-sum/de Casteljau implementation being tested.
    fn binomial_midpoint_controls(
        coefficients: &[num::BigRational],
    ) -> (Vec<num::BigRational>, Vec<num::BigRational>) {
        let binomial = |n: usize, k: usize| {
            let mut value = BigInt::one();
            for i in 0..k {
                value *= n - i;
                value /= i + 1;
            }
            value
        };
        let n = coefficients.len() - 1;
        let left = (0..=n)
            .map(|j| {
                (0..=j)
                    .map(|k| &coefficients[k] * binomial(j, k))
                    .sum::<num::BigRational>()
                    / (BigInt::one() << j)
            })
            .collect();
        let right = (0..=n)
            .map(|j| {
                (0..=n - j)
                    .map(|k| &coefficients[j + k] * binomial(n - j, k))
                    .sum::<num::BigRational>()
                    / (BigInt::one() << (n - j))
            })
            .collect();
        (left, right)
    }

    #[test]
    fn integer_bernstein_subdivision_preserves_one_positive_scale() {
        use num::{BigRational, Signed};

        assert!(midpoint_subdivide_integer_bernstein(Vec::new()).is_none());
        for degree in (0..=8).chain([16, 32, 64]) {
            for seed in 0..6 {
                let mut values: Vec<_> = (0..=degree)
                    .map(|i| {
                        let numerator = match seed {
                            0 => BigInt::zero(),
                            1 => BigInt::from(if i % 2 == 0 { 1 } else { -1 }),
                            2 if i % 3 != 0 => BigInt::zero(),
                            _ => {
                                BigInt::from(((i * 17 + seed * 13) % 31) as i64 - 15) << (seed * 30)
                            }
                        };
                        let denominator = if seed == 1 {
                            BigInt::one()
                        } else {
                            BigInt::from(1 + (i * 7 + seed) % 23)
                        };
                        BigRational::new(numerator, denominator)
                    })
                    .collect();
                let rationals: Vec<_> = values
                    .iter()
                    .map(|q| {
                        HyperRational::from_bigint_fraction(
                            q.numer().clone(),
                            q.denom().to_biguint().unwrap(),
                        )
                        .unwrap()
                    })
                    .collect();
                let mut controls =
                    HyperRational::primitive_bigint_ratio(&rationals.iter().collect::<Vec<_>>());
                let mut scale = values
                    .iter()
                    .zip(&controls)
                    .find(|(q, _)| !q.is_zero())
                    .map(|(q, c)| BigRational::from_integer(c.clone()) / q)
                    .unwrap_or_else(BigRational::one);
                assert!(scale.is_positive());
                for depth in 0..3 {
                    let (expected_left, expected_right) = binomial_midpoint_controls(&values);
                    let (left, right) = midpoint_subdivide_integer_bernstein(controls).unwrap();
                    scale *= BigInt::one() << degree;
                    for (expected, actual) in expected_left
                        .iter()
                        .chain(&expected_right)
                        .zip(left.iter().chain(&right))
                    {
                        assert_eq!(
                            expected * &scale,
                            BigRational::from_integer(actual.clone()),
                            "degree={degree}, seed={seed}, depth={depth}"
                        );
                    }
                    assert_eq!(left.last(), right.first());
                    if (seed + depth) % 2 == 0 {
                        values = expected_left;
                        controls = left;
                    } else {
                        values = expected_right;
                        controls = right;
                    }
                }
            }
        }
    }

    #[test]
    fn bernstein_subdivision_keeps_nonrational_and_undecided_controls() {
        let coefficients: Vec<_> = [2, -1, 0, 4, -3]
            .into_iter()
            .map(|value| real(value) * Real::pi())
            .collect();
        let expected = midpoint_subdivide_bernstein(&coefficients).unwrap();
        let (left, right) = BernsteinControls::Real(coefficients)
            .split_midpoint(0)
            .unwrap();
        for (actual, expected) in [(left, expected.0), (right, expected.1)] {
            let BernsteinControls::Real(actual) = actual else {
                panic!("nonrational controls must keep exact Real arithmetic");
            };
            assert_eq!(actual, expected);
        }
        let undecided =
            BernsteinControls::Real(vec![crate::test_support::terminal_zero(), real(1)]);
        assert_eq!(undecided.first_sign(PredicatePolicy::STRICT), None);
        assert_eq!(undecided.variations(PredicatePolicy::STRICT), None);
    }

    #[test]
    fn bernstein_subdivision_preserves_nonrational_interval_and_unknown_reports() {
        let x = Expr::symbol(SymbolId(0), "x");
        let expression = (x.clone() - Expr::int(1)) * (x - Expr::int(2));
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let config = BernsteinSubdivisionConfig {
            policy: PredicatePolicy::STRICT,
            max_depth: 4,
        };
        let report = subdivide_bernstein_univariate_polynomial_interval_expr(
            0,
            &expression,
            &problem,
            real(0),
            Real::pi(),
            config,
        );
        assert_eq!(report.status, BernsteinSubdivisionStatus::Completed);
        assert_eq!(report.intervals.len(), 2);
        let midpoint = (Real::pi() / real(2)).unwrap();
        for (interval, (lower, upper)) in report
            .intervals
            .iter()
            .zip([(real(0), midpoint.clone()), (midpoint, Real::pi())])
        {
            assert_eq!(
                interval.status,
                BernsteinSubdivisionIntervalStatus::Isolating
            );
            assert_eq!(interval.lower, lower);
            assert_eq!(interval.upper, upper);
            assert_eq!(interval.variation_bound, Some(1));
            assert!(interval.exact_root.is_none());
        }
        let limited = subdivide_bernstein_univariate_polynomial_interval_expr(
            0,
            &expression,
            &problem,
            real(0),
            Real::pi(),
            BernsteinSubdivisionConfig {
                max_depth: 0,
                ..config
            },
        );
        assert_eq!(limited.status, BernsteinSubdivisionStatus::DepthLimit);
        assert_eq!(limited.intervals.len(), 1);
        assert_eq!(limited.intervals[0].variation_bound, Some(2));
        let unknown = subdivide_bernstein_univariate_polynomial_interval_expr(
            0,
            &expression,
            &problem,
            real(0),
            crate::test_support::terminal_zero(),
            config,
        );
        assert_eq!(unknown.status, BernsteinSubdivisionStatus::Undecided);
        assert!(unknown.intervals.is_empty());
    }

    #[test]
    fn bernstein_public_coefficients_keep_actual_magnitudes() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let expression = (x.clone() - Expr::real((real(1) / real(3)).unwrap()))
            * (x - Expr::real((real(2) / real(3)).unwrap()));
        let report = count_bernstein_univariate_polynomial_interval_expr(
            0,
            &expression,
            &problem,
            real(0),
            real(1),
            PredicatePolicy::STRICT,
        );
        assert_eq!(report.status, BernsteinRootCountStatus::Counted);
        assert_eq!(
            report.bernstein_coefficients,
            vec![
                (real(2) / real(9)).unwrap(),
                (real(-5) / real(18)).unwrap(),
                (real(2) / real(9)).unwrap(),
            ]
        );
    }

    #[test]
    fn bernstein_subdivision_partitions_known_rational_root_multisets() {
        let compare = |a: &Real, b: &Real| {
            compare_reals(a, b, PredicatePolicy::STRICT)
                .value()
                .unwrap()
        };
        let ratio = |n, d| (real(n) / real(d)).unwrap();
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let config = BernsteinSubdivisionConfig {
            policy: PredicatePolicy::STRICT,
            max_depth: 24,
        };
        for degree in [2, 4, 8] {
            for seed in 0..3 {
                for kind in ["spread", "cluster", "repeated", "endpoint", "positive"] {
                    let mut roots: Vec<_> = (0..degree)
                        .map(|i| match kind {
                            "cluster" => ratio(1, 3) + ratio(i + 1 + seed, 1 << 12),
                            "repeated" => ratio(1, 3) + ratio(seed, 64),
                            "endpoint" if i == 0 => real(0),
                            "endpoint" if i == degree - 1 => real(1),
                            _ => ratio(8 * (i + 1) + seed, 8 * (degree + 1)),
                        })
                        .collect();
                    let mut expression =
                        Expr::int(if seed % 2 == 0 { seed + 1 } else { -seed - 1 });
                    if kind == "positive" {
                        let factor = (x.clone() - Expr::real(ratio(1, 2))).powi(2)
                            + Expr::real(ratio(1, 1 << 20));
                        for _ in 0..degree / 2 {
                            expression = expression * factor.clone();
                        }
                        roots.clear();
                    } else {
                        for root in &roots {
                            expression = expression * (x.clone() - Expr::real(root.clone()));
                        }
                    }
                    let report = subdivide_bernstein_univariate_polynomial_interval_expr(
                        7,
                        &expression,
                        &problem,
                        real(0),
                        real(1),
                        config,
                    );
                    assert!(matches!(
                        report.status,
                        BernsteinSubdivisionStatus::Completed
                            | BernsteinSubdivisionStatus::DepthLimit
                    ));
                    assert_eq!(report.constraint_index, 7);
                    assert_eq!(report.degree, Some(degree as usize));
                    assert_eq!(report.symbol, Some(SymbolId(0)));
                    let mut cursor = real(0);
                    let mut points = Vec::new();
                    let mut limited = false;
                    for interval in &report.intervals {
                        if interval.status == BernsteinSubdivisionIntervalStatus::EndpointRoot {
                            assert_eq!(interval.lower, interval.upper);
                            assert_eq!(interval.exact_root.as_ref(), Some(&interval.lower));
                            assert_eq!(interval.variation_bound, Some(0));
                            assert!(roots.contains(&interval.lower));
                            assert!(!points.contains(&interval.lower));
                            points.push(interval.lower.clone());
                            continue;
                        }
                        assert_eq!(compare(&interval.lower, &interval.upper), Ordering::Less);
                        assert_eq!(interval.lower, cursor);
                        cursor = interval.upper.clone();
                        assert!(interval.exact_root.is_none());
                        let inside = roots
                            .iter()
                            .filter(|root| {
                                compare(&interval.lower, root) == Ordering::Less
                                    && compare(root, &interval.upper) == Ordering::Less
                            })
                            .count();
                        let variation = interval.variation_bound.unwrap();
                        assert!(inside <= variation);
                        assert_eq!(inside % 2, variation % 2);
                        match interval.status {
                            BernsteinSubdivisionIntervalStatus::Empty => {
                                assert_eq!(inside, 0);
                                assert_eq!(variation, 0);
                            }
                            BernsteinSubdivisionIntervalStatus::Isolating => {
                                assert_eq!(inside, 1);
                                assert_eq!(variation, 1);
                                assert!(!roots.contains(&interval.lower));
                                assert!(!roots.contains(&interval.upper));
                            }
                            BernsteinSubdivisionIntervalStatus::DepthLimit => {
                                limited = true;
                                assert_eq!(
                                    interval.upper.clone() - &interval.lower,
                                    ratio(1, 1 << 24)
                                );
                            }
                            BernsteinSubdivisionIntervalStatus::EndpointRoot => unreachable!(),
                        }
                    }
                    assert_eq!(cursor, real(1));
                    assert_eq!(
                        report.status == BernsteinSubdivisionStatus::DepthLimit,
                        limited
                    );
                    for root in roots {
                        assert!(
                            points.contains(&root)
                                || report.intervals.iter().any(|interval| {
                                    interval.status != BernsteinSubdivisionIntervalStatus::Empty
                                        && compare(&interval.lower, &root) == Ordering::Less
                                        && compare(&root, &interval.upper) == Ordering::Less
                                })
                        );
                    }
                }
            }
        }
    }

    proptest! {
        #[test]
        fn midpoint_de_casteljau_matches_direct_exact_bernstein_conversion(
            a in -8_i16..=8,
            b in -8_i16..=8,
            c in -8_i16..=8,
            d in -8_i16..=8,
            lower in -8_i16..=8,
            width in 1_i16..=8,
        ) {
            let polynomial = [a, b, c, d]
                .into_iter()
                .map(|coefficient| real(i64::from(coefficient)))
                .collect::<Vec<_>>();
            let lower = real(i64::from(lower));
            let upper = lower.clone() + real(i64::from(width));
            let midpoint = ((lower.clone() + upper.clone()) / real(2))
                .expect("two is nonzero");
            let parent = power_to_bernstein_on_interval(&polynomial, &lower, &upper)
                .expect("fixed polynomial conversion");
            let (left, right) = midpoint_subdivide_bernstein(&parent)
                .expect("nonempty Bernstein coefficients");

            prop_assert_eq!(
                left,
                power_to_bernstein_on_interval(&polynomial, &lower, &midpoint)
                    .expect("left exact conversion")
            );
            prop_assert_eq!(
                right,
                power_to_bernstein_on_interval(&polynomial, &midpoint, &upper)
                    .expect("right exact conversion")
            );
        }

        #[test]
        fn bernstein_generated_quadratic_interval_counts_one_interior_root(
            root in -16_i16..=16,
            other in -32_i16..=32,
        ) {
            let root = i64::from(root);
            let other = i64::from(other);
            prop_assume!(other < root - 1 || other > root + 1);
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            problem.add_variable("x", real(0));
            problem.add_constraint(Constraint::equality(
                "generated one interval root",
                (x.clone() - Expr::int(root)) * (x - Expr::int(other)),
            ));

            let reports = count_bernstein_univariate_polynomial_interval_roots(
                &problem.analyze(),
                real(root - 1),
                real(root + 1),
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(reports.len(), 1);
            prop_assert_eq!(&reports[0].status, &BernsteinRootCountStatus::Counted);
            prop_assert_eq!(reports[0].root_at_lower, Some(false));
            prop_assert_eq!(reports[0].root_at_upper, Some(false));
            prop_assert_eq!(reports[0].variation_bound, Some(1));
            prop_assert_eq!(reports[0].root_count_parity, Some(1));
        }
    }
}
