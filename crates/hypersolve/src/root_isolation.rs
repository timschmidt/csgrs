//! Exact univariate algebraic root isolation.
//!
//! This module is a proof package, not a numerical proposal engine. It extracts
//! exact-rational univariate polynomial residuals, reduces repeated factors,
//! and isolates distinct real roots with Sturm sign-variation counts over
//! rational intervals. That keeps the Yap boundary explicit: algebraic
//! isolation constructs certified exact intervals, while ordinary candidate
//! replay still decides whether a solver assignment is acceptable. See Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997); Sturm, "Mémoire sur la résolution des équations numériques" (1835);
//! and Collins and Loos, "Real Zeros of Polynomials," in *Computer Algebra:
//! Symbolic and Algebraic Computation* (1982).

use std::cmp::Ordering;
use std::collections::HashMap;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::model::{ConstraintKind, Problem};
use crate::prepared::PreparedProblem;
use crate::symbolic::{Expr, SymbolId};

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
    /// Lower exact rational endpoint.
    pub lower: Real,
    /// Upper exact rational endpoint.
    pub upper: Real,
    /// Exact root value when subdivision landed on a rational root.
    pub exact_root: Option<Real>,
    /// Number of distinct roots certified in the interval.
    pub distinct_root_count: usize,
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
/// replay or to a future algebraic-number package, preserving Yap's
/// construction/proof boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct RootIsolationConfig {
    /// Exact comparison/refinement policy used by `hyperlimit`.
    pub policy: PredicatePolicy,
    /// Optional exact maximum width for non-rational isolating intervals.
    pub max_interval_width: Option<Real>,
    /// Maximum additional bisection steps once an interval has one certified
    /// root. This bounds work for clustered roots.
    pub max_refinement_steps: usize,
}

impl Default for RootIsolationConfig {
    fn default() -> Self {
        Self {
            policy: PredicatePolicy::default(),
            max_interval_width: None,
            max_refinement_steps: 0,
        }
    }
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
/// root topologies before Sturm subdivision. See Descartes, *La Géométrie*
/// (1637), Collins and Loos (1982), and Yap (1997).
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
/// global Descartes count. See Farouki and Rajan, "Algorithms for Polynomials
/// in Bernstein Form," *Computer Aided Geometric Design* 5.1 (1988), Collins
/// and Loos (1982), and Yap (1997).
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
    /// The interval still has variation greater than one at the configured
    /// subdivision depth.
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
    /// At least one interval still had multiple possible roots at the depth
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
            policy: PredicatePolicy::default(),
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
/// Farouki and Rajan (1988) while preserving Yap's exact proof boundary.
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
    prepared: &PreparedProblem<'_>,
    policy: PredicatePolicy,
) -> Vec<UnivariateRootIsolationReport> {
    isolate_univariate_polynomial_roots_with_config(
        prepared,
        RootIsolationConfig {
            policy,
            ..RootIsolationConfig::default()
        },
    )
}

/// Isolate distinct real roots with explicit bounded-refinement controls.
pub fn isolate_univariate_polynomial_roots_with_config(
    prepared: &PreparedProblem<'_>,
    config: RootIsolationConfig,
) -> Vec<UnivariateRootIsolationReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(isolate_univariate_polynomial_expr_with_config(
            constraint_index,
            &constraint.residual,
            prepared.problem(),
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
        match polynomial_div_rem(poly, &gcd, policy).and_then(|(quotient, remainder)| {
            is_zero_polynomial(&remainder, policy)?.then_some(quotient)
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

/// Replay exact rational root witnesses produced by isolation.
///
/// Sturm isolation usually returns intervals, not concrete algebraic numbers.
/// When subdivision lands on a rational root exactly, this helper binds that
/// witness into a cloned candidate context and replays the full prepared
/// problem. Non-rational intervals are reported as explicit non-witnesses
/// rather than approximated. This follows the Collins/Loos isolation model and
/// Yap's rule that constructed algebraic evidence still needs exact replay
/// before becoming a solver decision.
pub fn certify_isolated_rational_root_witnesses(
    prepared: &PreparedProblem<'_>,
    base_context: &EvaluationContext,
    reports: &[UnivariateRootIsolationReport],
) -> Vec<AlgebraicRootCandidateReport> {
    certify_isolated_rational_root_witnesses_with_config(
        prepared,
        base_context,
        reports,
        CandidateCertificationConfig::default(),
    )
}

/// Replay exact rational root witnesses with a candidate-certification policy.
pub fn certify_isolated_rational_root_witnesses_with_config(
    prepared: &PreparedProblem<'_>,
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
                certify_candidate_with_config(prepared, &candidate, certification_config);
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
    prepared: &PreparedProblem<'_>,
    policy: PredicatePolicy,
) -> Vec<DescartesRootCountReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(count_descartes_univariate_polynomial_expr(
            constraint_index,
            &constraint.residual,
            prepared.problem(),
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
    prepared: &PreparedProblem<'_>,
    lower: Real,
    upper: Real,
    policy: PredicatePolicy,
) -> Vec<BernsteinRootCountReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(count_bernstein_univariate_polynomial_interval_expr(
            constraint_index,
            &constraint.residual,
            prepared.problem(),
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
    match compare_reals_with_policy(&lower, &upper, policy).value() {
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
        match compare_reals_with_policy(&evaluate_polynomial(&poly, &lower), &Real::zero(), policy)
            .value()
        {
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
        match compare_reals_with_policy(&evaluate_polynomial(&poly, &upper), &Real::zero(), policy)
            .value()
        {
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
    prepared: &PreparedProblem<'_>,
    lower: Real,
    upper: Real,
    config: BernsteinSubdivisionConfig,
) -> Vec<BernsteinSubdivisionReport> {
    let mut reports = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        reports.push(subdivide_bernstein_univariate_polynomial_interval_expr(
            constraint_index,
            &constraint.residual,
            prepared.problem(),
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

    let mut intervals = Vec::new();
    let mut hit_depth_limit = false;
    let mut undecided = None;
    subdivide_bernstein_interval(
        constraint_index,
        expression,
        problem,
        lower.clone(),
        upper.clone(),
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
        first.symbol,
        first.degree,
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
    let sturm = sturm_sequence(polynomial, policy)?;
    let bound = cauchy_bound(polynomial, policy)?;
    let lower = -bound.clone();
    let upper = bound;
    let root_count = sturm_count(&sturm, &lower, &upper, policy)?;
    let mut intervals = Vec::new();
    isolate_interval(
        &sturm,
        &lower,
        &upper,
        root_count,
        config,
        0,
        &mut intervals,
    )?;
    Some(intervals)
}

fn isolate_interval(
    sturm: &[Vec<Real>],
    lower: &Real,
    upper: &Real,
    root_count: usize,
    config: &RootIsolationConfig,
    refinement_step: usize,
    intervals: &mut Vec<IsolatedRootInterval>,
) -> Option<()> {
    let policy = config.policy;
    if root_count == 0 {
        return Some(());
    }
    if root_count == 1 {
        if should_refine_one_root_interval(lower, upper, config, refinement_step)? {
            let midpoint = ((lower.clone() + upper.clone()) / Real::from(2)).ok()?;
            let first = &sturm[0];
            if sign_at(first, &midpoint, policy)? == Ordering::Equal {
                intervals.push(IsolatedRootInterval {
                    lower: midpoint.clone(),
                    upper: midpoint.clone(),
                    exact_root: Some(midpoint),
                    distinct_root_count: 1,
                });
                return Some(());
            }
            let left_count = sturm_count(sturm, lower, &midpoint, policy)?;
            let right_count = root_count.checked_sub(left_count)?;
            isolate_interval(
                sturm,
                lower,
                &midpoint,
                left_count,
                config,
                refinement_step + 1,
                intervals,
            )?;
            isolate_interval(
                sturm,
                &midpoint,
                upper,
                right_count,
                config,
                refinement_step + 1,
                intervals,
            )?;
            return Some(());
        }
        intervals.push(IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: None,
            distinct_root_count: 1,
        });
        return Some(());
    }

    let midpoint = ((lower.clone() + upper.clone()) / Real::from(2)).ok()?;
    let first = &sturm[0];
    if sign_at(first, &midpoint, policy)? == Ordering::Equal {
        intervals.push(IsolatedRootInterval {
            lower: midpoint.clone(),
            upper: midpoint.clone(),
            exact_root: Some(midpoint.clone()),
            distinct_root_count: 1,
        });
        let mut left_count = sturm_count(sturm, lower, &midpoint, policy)?;
        let mut right_count = sturm_count(sturm, &midpoint, upper, policy)?;
        // Sturm endpoint conventions can count a rational root that lies
        // exactly on the split point as part of one adjacent interval. The
        // point root has already been emitted above, so trim the adjacent
        // counts back to the remaining distinct roots before recursing.
        let remaining = root_count.checked_sub(1)?;
        while left_count + right_count > remaining {
            if left_count > 0 {
                left_count -= 1;
            } else if right_count > 0 {
                right_count -= 1;
            } else {
                return None;
            }
        }
        isolate_interval(sturm, lower, &midpoint, left_count, config, 0, intervals)?;
        isolate_interval(sturm, &midpoint, upper, right_count, config, 0, intervals)?;
        return Some(());
    }

    let left_count = sturm_count(sturm, lower, &midpoint, policy)?;
    let right_count = root_count.checked_sub(left_count)?;
    isolate_interval(sturm, lower, &midpoint, left_count, config, 0, intervals)?;
    isolate_interval(sturm, &midpoint, upper, right_count, config, 0, intervals)
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
    match compare_reals_with_policy(&width, max_width, config.policy).value()? {
        Ordering::Greater => Some(true),
        Ordering::Equal | Ordering::Less => Some(false),
    }
}

fn sturm_sequence(polynomial: &[Real], policy: PredicatePolicy) -> Option<Vec<Vec<Real>>> {
    let p0 = trim_polynomial(polynomial.to_vec(), policy)?;
    let p1 = trim_polynomial(derivative(&p0), policy)?;
    let mut sequence = vec![p0, p1];
    loop {
        let last = sequence.last()?.clone();
        if last.len() == 1 {
            break;
        }
        let previous = sequence.get(sequence.len() - 2)?.clone();
        let (_, remainder) = polynomial_div_rem(previous, &last, policy)?;
        if is_zero_polynomial(&remainder, policy)? {
            break;
        }
        sequence.push(remainder.into_iter().map(|value| -value).collect());
    }
    Some(sequence)
}

fn sturm_count(
    sturm: &[Vec<Real>],
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<usize> {
    let lower_variations = sign_variations(sturm, lower, policy)?;
    let upper_variations = sign_variations(sturm, upper, policy)?;
    lower_variations.checked_sub(upper_variations)
}

fn sign_variations(sturm: &[Vec<Real>], point: &Real, policy: PredicatePolicy) -> Option<usize> {
    let mut previous = None;
    let mut variations = 0;
    for polynomial in sturm {
        let sign = sign_at(polynomial, point, policy)?;
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

fn sign_at(polynomial: &[Real], point: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    let value = evaluate_polynomial(polynomial, point);
    compare_reals_with_policy(&value, &Real::zero(), policy).value()
}

fn cauchy_bound(polynomial: &[Real], policy: PredicatePolicy) -> Option<Real> {
    let leading = abs_real(polynomial.last()?, policy)?;
    let mut max_ratio = Real::zero();
    for coefficient in &polynomial[..polynomial.len() - 1] {
        let ratio = (abs_real(coefficient, policy)? / leading.clone()).ok()?;
        if compare_reals_with_policy(&ratio, &max_ratio, policy).value()? == Ordering::Greater {
            max_ratio = ratio;
        }
    }
    Some(max_ratio + Real::one())
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

fn polynomial_gcd(
    mut left: Vec<Real>,
    mut right: Vec<Real>,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    left = trim_polynomial(left, policy)?;
    right = trim_polynomial(right, policy)?;
    while !is_zero_polynomial(&right, policy)? {
        let (_, remainder) = polynomial_div_rem(left, &right, policy)?;
        left = right;
        right = trim_polynomial(remainder, policy)?;
    }
    Some(gcd_monic_normalize(left, policy)?)
}

fn polynomial_div_rem(
    dividend: Vec<Real>,
    divisor: &[Real],
    policy: PredicatePolicy,
) -> Option<(Vec<Real>, Vec<Real>)> {
    let divisor = trim_polynomial(divisor.to_vec(), policy)?;
    if is_zero_polynomial(&divisor, policy)? {
        return None;
    }
    let mut remainder = trim_polynomial(dividend, policy)?;
    if remainder.len() < divisor.len() {
        return Some((vec![Real::zero()], remainder));
    }
    let mut quotient = vec![Real::zero(); remainder.len() - divisor.len() + 1];
    let divisor_degree = divisor.len() - 1;
    let divisor_leading = divisor.last()?.clone();
    while remainder.len() >= divisor.len() && !is_zero_polynomial(&remainder, policy)? {
        let degree_delta = remainder.len() - divisor.len();
        let scale = (remainder.last()?.clone() / divisor_leading.clone()).ok()?;
        quotient[degree_delta] = quotient[degree_delta].clone() + scale.clone();
        for (index, divisor_coefficient) in divisor.iter().enumerate().take(divisor_degree + 1) {
            let target = degree_delta + index;
            remainder[target] =
                remainder[target].clone() - scale.clone() * divisor_coefficient.clone();
        }
        remainder = trim_polynomial(remainder, policy)?;
    }
    Some((trim_polynomial(quotient, policy)?, remainder))
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

fn is_zero_polynomial(polynomial: &[Real], policy: PredicatePolicy) -> Option<bool> {
    polynomial.iter().try_fold(true, |all_zero, coefficient| {
        let sign = compare_reals_with_policy(coefficient, &Real::zero(), policy).value()?;
        Some(all_zero && sign == Ordering::Equal)
    })
}

fn gcd_monic_normalize(mut polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    polynomial = trim_polynomial(polynomial, policy)?;
    if polynomial.len() == 1 {
        return Some(polynomial);
    }
    let leading = polynomial.last()?.clone();
    polynomial
        .into_iter()
        .map(|coefficient| (coefficient / leading.clone()).ok())
        .collect()
}

fn abs_real(value: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(value, &Real::zero(), policy).value()? {
        Ordering::Less => Some(-value.clone()),
        Ordering::Equal | Ordering::Greater => Some(value.clone()),
    }
}

fn leading_zero_multiplicity(polynomial: &[Real], policy: PredicatePolicy) -> Option<usize> {
    let mut multiplicity = 0;
    for coefficient in polynomial {
        match compare_reals_with_policy(coefficient, &Real::zero(), policy).value()? {
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
        let sign = compare_reals_with_policy(coefficient, &Real::zero(), policy).value()?;
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
    let degree = polynomial.len().saturating_sub(1);
    let width = upper.clone() - lower.clone();
    let mut shifted_power = vec![Real::zero(); degree + 1];
    for (power, coefficient) in polynomial.iter().enumerate() {
        for target_power in 0..=power {
            let binomial = Real::from(binomial(power, target_power)? as i64);
            let lower_power = pow_real_nonnegative(lower, power - target_power);
            let width_power = pow_real_nonnegative(&width, target_power);
            shifted_power[target_power] = shifted_power[target_power].clone()
                + coefficient.clone() * binomial * lower_power * width_power;
        }
    }

    let mut bernstein = vec![Real::zero(); degree + 1];
    for i in 0..=degree {
        let mut value = Real::zero();
        for (j, coefficient) in shifted_power.iter().enumerate().take(i + 1) {
            let numerator = Real::from(binomial(i, j)? as i64);
            let denominator = Real::from(binomial(degree, j)? as i64);
            value = value + ((coefficient.clone() * numerator) / denominator).ok()?;
        }
        bernstein[i] = value;
    }
    Some(bernstein)
}

#[allow(clippy::too_many_arguments)]
fn subdivide_bernstein_interval(
    constraint_index: usize,
    expression: &Expr,
    problem: &Problem,
    lower: Real,
    upper: Real,
    config: BernsteinSubdivisionConfig,
    depth: usize,
    intervals: &mut Vec<BernsteinSubdivisionInterval>,
    hit_depth_limit: &mut bool,
    undecided: &mut Option<String>,
) {
    if undecided.is_some() {
        return;
    }
    let report = count_bernstein_univariate_polynomial_interval_expr(
        constraint_index,
        expression,
        problem,
        lower.clone(),
        upper.clone(),
        config.policy,
    );
    if report.status != BernsteinRootCountStatus::Counted {
        *undecided = report
            .message
            .or_else(|| Some("Bernstein subdivision encountered an undecided interval".to_owned()));
        return;
    }

    if report.root_at_lower == Some(true) {
        push_unique_bernstein_endpoint(intervals, lower.clone());
    }
    if report.root_at_upper == Some(true) {
        push_unique_bernstein_endpoint(intervals, upper.clone());
    }

    match report.variation_bound {
        Some(0) => {
            intervals.push(BernsteinSubdivisionInterval {
                lower,
                upper,
                exact_root: None,
                variation_bound: Some(0),
                status: BernsteinSubdivisionIntervalStatus::Empty,
            });
        }
        Some(1) => {
            if report.root_at_lower == Some(true) || report.root_at_upper == Some(true) {
                return;
            }
            intervals.push(BernsteinSubdivisionInterval {
                lower,
                upper,
                exact_root: None,
                variation_bound: Some(1),
                status: BernsteinSubdivisionIntervalStatus::Isolating,
            });
        }
        Some(variation) => {
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
            subdivide_bernstein_interval(
                constraint_index,
                expression,
                problem,
                lower,
                midpoint.clone(),
                config,
                depth + 1,
                intervals,
                hit_depth_limit,
                undecided,
            );
            subdivide_bernstein_interval(
                constraint_index,
                expression,
                problem,
                midpoint,
                upper,
                config,
                depth + 1,
                intervals,
                hit_depth_limit,
                undecided,
            );
        }
        None => {
            *undecided = Some("Bernstein variation bound was unavailable".to_owned());
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

fn pow_real_nonnegative(value: &Real, exponent: usize) -> Real {
    (0..exponent).fold(Real::one(), |product, _| product * value.clone())
}

fn binomial(n: usize, k: usize) -> Option<u64> {
    if k > n {
        return Some(0);
    }
    let k = k.min(n - k);
    let mut result = 1_u128;
    for i in 1..=k {
        result = result.checked_mul((n - k + i) as u128)?;
        result /= i as u128;
    }
    u64::try_from(result).ok()
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
            &PreparedProblem::new(&problem),
            PredicatePolicy::default(),
        );

        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].status, RootIsolationStatus::Isolated);
        assert_eq!(reports[0].intervals.len(), 3);
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
            &PreparedProblem::new(&problem),
            PredicatePolicy::default(),
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
        let prepared = PreparedProblem::new(&problem);
        let reports = isolate_univariate_polynomial_roots_with_config(
            &prepared,
            RootIsolationConfig {
                policy: PredicatePolicy::default(),
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
            &prepared,
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
            &PreparedProblem::new(&problem),
            PredicatePolicy::default(),
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
            &PreparedProblem::new(&problem),
            real(0),
            real(2),
            PredicatePolicy::default(),
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
        let prepared = PreparedProblem::new(&problem);
        let complete = subdivide_bernstein_univariate_polynomial_interval_roots(
            &prepared,
            real(0),
            real(4),
            BernsteinSubdivisionConfig {
                policy: PredicatePolicy::default(),
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
        assert_eq!(endpoint_roots, 1);
        assert!(
            complete[1]
                .intervals
                .iter()
                .any(|interval| interval.status == BernsteinSubdivisionIntervalStatus::Empty)
        );

        let depth_limited = subdivide_bernstein_univariate_polynomial_interval_roots(
            &prepared,
            real(0),
            real(4),
            BernsteinSubdivisionConfig {
                policy: PredicatePolicy::default(),
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

    proptest! {
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
                &PreparedProblem::new(&problem),
                real(root - 1),
                real(root + 1),
                PredicatePolicy::default(),
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
