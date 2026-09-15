//! Exact real-root counts for polynomial fibers over one represented algebraic root.
//!
//! A bivariate subresultant becomes a univariate fiber polynomial after one
//! parameter is fixed to an algebraic root. Its coefficients then live in the
//! local field at that root, not in `Real`. This module represents those
//! coefficients as reduced rational functions of the retained parameter and
//! runs an ordinary Sturm sequence in the fiber parameter. The construction
//! counts distinct roots, including roots of even multiplicity, without
//! sampling either algebraic value.

use std::cmp::Ordering;

mod refinement;
pub use refinement::AlgebraicFiberRootRefiner;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::{Real, RealSign, ZeroKnowledge};

use crate::algebraic::{
    AlgebraicRootPolynomialEvaluationReport, AlgebraicRootPolynomialEvaluationStatus,
    AlgebraicRootRepresentation, algebraic_root_payload_replays_strictly,
    evaluate_polynomial_at_algebraic_root, validate_algebraic_root_representation,
};
use crate::curve_resultant::{BivariatePolynomial, CurveResultantParameter};
use crate::integer_interpolation::{
    rational_polynomial_inverse_modulo, rational_polynomial_product_modulo,
};
use crate::ordered_field_roots::{
    OrderedFieldPolynomialContext, OrderedFieldRootIsolationConfig,
    OrderedFieldRootIsolationStatus, isolate_ordered_field_polynomial_roots,
};
use crate::policy_division::strict_exact_zero_for_storage;
use crate::resultant::quotient_ring_fiber_resultant_polynomial;
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig, polynomial_div_rem,
    polynomial_gcd, polynomial_vanishes_at_owned_root,
    refine_isolated_univariate_polynomial_interval,
};

const LOCAL_FIELD_INTERVAL_SIGN_REFINEMENT_ROUNDS: usize = 8;

/// Final status for an algebraic-parameter fiber root count.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberRootCountStatus {
    /// The number of distinct roots in the open interval was certified.
    Counted,
    /// Every specialized fiber coefficient vanishes at the retained root.
    IdenticallyZeroFiber,
    /// A fiber root lies exactly on an interval endpoint, where this bounded
    /// open-interval contract intentionally does not choose ownership.
    EndpointRoot,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// The requested fiber interval is empty or reversed.
    InvalidInterval,
    /// A coefficient could not be represented by the exact `Real` local-field
    /// package.
    UnsupportedCoefficient,
    /// Exact arithmetic or a predicate did not complete.
    Undecided,
}

/// Exact report for counting roots of a bivariate polynomial fiber.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlgebraicFiberRootCountReport {
    /// Final root-count status.
    pub status: AlgebraicFiberRootCountStatus,
    /// Certified number of distinct roots when `status == Counted`.
    pub distinct_root_count: Option<usize>,
    /// Number of local-field Sturm polynomials built.
    pub sturm_sequence_length: usize,
    /// Exact bisection steps used to refine the retained root while signing
    /// local-field coefficients.
    pub retained_refinement_steps: usize,
    /// Weakest predicate certainty consumed by decisions visible to this
    /// package.
    pub certainty: Certainty,
    /// Compact diagnostic reason for a non-counted result.
    pub message: Option<&'static str>,
}

/// Final status for exact diagonal-root deflation in one algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberDiagonalDeflationStatus {
    /// One or more copies of `fiber_parameter - retained_parameter` were
    /// removed in the selected local field.
    Deflated,
    /// The selected diagonal value is not a root of the specialized fiber.
    NotARoot,
    /// Every specialized fiber coefficient vanishes at the retained root.
    IdenticallyZeroFiber,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// Exact-rational coefficient arithmetic is required by this package.
    UnsupportedCoefficient,
    /// Exact local-field arithmetic or a predicate did not complete.
    Undecided,
}

/// Exact residual after removing a known correlated diagonal fiber root.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicFiberDiagonalDeflationReport {
    /// Final construction status.
    pub status: AlgebraicFiberDiagonalDeflationStatus,
    /// Exact multiplicity removed from the selected fiber.
    pub multiplicity: usize,
    /// Fiber polynomial reduced modulo the retained root's defining
    /// polynomial after maximal diagonal deflation.
    pub reduced_polynomial: Option<BivariatePolynomial>,
    /// Weakest predicate certainty consumed by the local-field zero tests.
    pub certainty: Certainty,
}

/// Final status for reducing a rational function modulo one algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberRationalReductionStatus {
    /// The quotient is independent of the fiber parameter and was retained as
    /// one exact rational function of the algebraic base parameter.
    ReducedToRetainedField,
    /// The selected fiber parameter remains essential after exact reduction.
    FiberDependent,
    /// Every coefficient of the specialized fiber equation vanishes.
    IdenticallyZeroFiber,
    /// The specialized fiber equation is a nonzero constant and has no root.
    ConstantNonzeroFiber,
    /// The rational denominator vanishes throughout the fiber quotient ring.
    ZeroDenominator,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// A coefficient could not be represented by the exact local-field package.
    UnsupportedCoefficient,
    /// Exact local-field arithmetic or a predicate did not complete.
    Undecided,
}

/// Exact retained-field value of a rational function on one algebraic fiber.
///
/// When `status == ReducedToRetainedField`, the represented value is
/// `numerator_coefficients(alpha) / denominator_coefficients(alpha)`. Both
/// polynomials are reduced modulo the defining polynomial of `alpha`.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicFiberRationalReductionReport {
    /// Final reduction status.
    pub status: AlgebraicFiberRationalReductionStatus,
    /// Numerator in ascending powers of the retained algebraic parameter.
    pub numerator_coefficients: Vec<Real>,
    /// Nonzero denominator in ascending powers of the retained parameter.
    pub denominator_coefficients: Vec<Real>,
    /// Weakest predicate certainty consumed by local-field identities.
    pub certainty: Certainty,
}

/// Final status for projecting a polynomial image of one algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberPolynomialImageProjectionStatus {
    /// The exact rational-coefficient image eliminant was constructed.
    Constructed,
    /// Every coefficient of the specialized fiber equation vanishes.
    IdenticallyZeroFiber,
    /// The specialized fiber equation is a nonzero constant and has no root.
    ConstantNonzeroFiber,
    /// The image relation is identically zero.
    IdenticallyZeroImageRelation,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// A coefficient could not be represented by the exact local-field package.
    UnsupportedCoefficient,
    /// One explicitly bounded quotient-ring dimension was exceeded.
    DegreeLimitExceeded,
    /// Exact arithmetic or a predicate did not complete.
    Undecided,
}

/// Exact global eliminant for a polynomial image of one selected fiber.
///
/// If `status == Constructed`, `coefficients` is a polynomial in the image
/// variable whose real roots include every image of the authored selected
/// fiber. The caller must isolate and replay the intended local root and any
/// unsquared branch condition.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicFiberPolynomialImageProjectionReport {
    /// Final projection status.
    pub status: AlgebraicFiberPolynomialImageProjectionStatus,
    /// Global image eliminant in ascending powers of the image variable.
    pub coefficients: Vec<Real>,
    /// Exact image relation retained over the selected base parameter.
    ///
    /// `retained_relation.coefficients[first_power][second_power]` names the
    /// retained base parameter and image parameter respectively. This
    /// relation is the primitive local norm before the final norm through the
    /// retained parameter's defining polynomial. It can therefore be
    /// available even when construction of the larger global eliminant is
    /// declined.
    pub retained_relation: Option<BivariatePolynomial>,
    /// Source-fiber factor on which the complete image relation vanishes.
    ///
    /// The factor uses the same retained/fiber axis order as
    /// `fiber_equation`. It is removed before the residual image norm is
    /// constructed. A caller must test whether its specifically selected
    /// source root belongs to this factor before using the residual image;
    /// membership denotes a genuinely positive-dimensional image equation.
    pub identically_zero_fiber_factor: Option<BivariatePolynomial>,
    /// Degree of the specialized source fiber used by the local norm.
    pub fiber_degree: usize,
    /// Conservative image degree before exact trimming and square-free replay.
    pub image_degree_bound: usize,
    /// Weakest predicate certainty consumed by visible decisions.
    pub certainty: Certainty,
}

/// Bounded-memory dimensions for one direct algebraic-fiber image norm.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AlgebraicFiberPolynomialImageProjectionConfig {
    /// Maximum degree of the selected local fiber in its fiber parameter.
    pub max_fiber_degree: usize,
    /// Maximum degree of the retained base-root carrier.
    pub max_retained_degree: usize,
    /// Maximum source-fiber degree times image-relation degree.
    pub max_image_degree_bound: usize,
}

/// Final status for quotient-ring projection of one algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberProjectionStatus {
    /// The exact rational-coefficient projection polynomial was constructed.
    Constructed,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// Exact-rational coefficient arithmetic is required by this package.
    UnsupportedCoefficient,
    /// The bounded quotient-ring determinant construction did not complete.
    Undecided,
}

/// Exact rational projection polynomial for one algebraic fiber.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicFiberProjectionReport {
    /// Final construction status.
    pub status: AlgebraicFiberProjectionStatus,
    /// Resultant coefficients in ascending powers of the fiber parameter.
    pub coefficients: Vec<Real>,
}

/// Final status for exact isolation inside one selected algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberRootIsolationStatus {
    /// Every distinct fiber root in the requested interval was isolated.
    Isolated,
    /// The selected fiber has no root in the requested interval.
    NoRoots,
    /// Every specialized fiber coefficient vanishes at the selected root.
    IdenticallyZeroFiber,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// The requested fiber interval is empty or reversed.
    InvalidInterval,
    /// Exact `Real` coefficients or endpoints could not be consumed.
    UnsupportedCoefficient,
    /// Root separation exceeded the caller's subdivision budget.
    DepthLimit,
    /// Exact local-field arithmetic or a predicate did not complete.
    Undecided,
}

/// Bounded subdivision controls for selected-fiber root isolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AlgebraicFiberRootIsolationConfig {
    /// Maximum dyadic subdivision depth used to separate distinct roots.
    pub max_subdivision_depth: usize,
    /// Additional dyadic refinements applied after an interval owns one root.
    pub refinement_steps: usize,
}

impl Default for AlgebraicFiberRootIsolationConfig {
    fn default() -> Self {
        Self {
            max_subdivision_depth: 128,
            refinement_steps: 8,
        }
    }
}

/// Exact selected-fiber root isolators and construction evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicFiberRootIsolationReport {
    /// Final isolation status.
    pub status: AlgebraicFiberRootIsolationStatus,
    /// Ordered singleton intervals, including exact rational witnesses.
    pub intervals: Vec<IsolatedRootInterval>,
    /// Length of the last local-field Sturm sequence built.
    pub sturm_sequence_length: usize,
    /// Dyadic subdivisions performed across all restarts.
    pub subdivision_steps: usize,
    /// Exact bisections used while signing retained-field coefficients.
    pub retained_refinement_steps: usize,
    /// Weakest predicate certainty consumed by visible decisions.
    pub certainty: Certainty,
    /// Compact diagnostic reason for an incomplete report.
    pub message: Option<&'static str>,
}

/// Count distinct roots of one exact bivariate fiber over a represented root.
///
/// `retained_parameter` selects the parameter represented by `retained_root`;
/// the opposite parameter is counted in `(fiber_lower, fiber_upper)`. All
/// polynomial and endpoint values are exact [`Real`] scalars. Coefficients of the
/// specialized fiber are represented as fractions in the local field at the
/// selected root, reduced modulo its defining polynomial after every
/// operation. Sturm's theorem then counts distinct roots, so an isolated root
/// of even multiplicity is retained instead of being missed by an endpoint
/// sign-change test.
pub fn count_bivariate_fiber_roots_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootCountReport {
    count_bivariate_fiber_system_roots(
        &[polynomial],
        retained_parameter,
        retained_root,
        fiber_lower,
        fiber_upper,
        FiberIntervalEndpoints::RejectRoots,
        policy,
    )
}

/// Count distinct roots in several open intervals of one algebraic fiber.
///
/// This is the amortized counterpart to
/// [`count_bivariate_fiber_roots_at_algebraic_parameter`]. The bivariate
/// polynomial is specialized once, one local-field Sturm sequence is built,
/// and coefficient signs learned while evaluating one interval are reused by
/// the remaining intervals. Reports retain the input order and preserve the
/// ordinary per-interval endpoint and validation statuses.
///
/// The intervals need not be disjoint or sorted.
pub fn count_bivariate_fiber_roots_at_algebraic_parameter_intervals(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_intervals: &[(&Real, &Real)],
    policy: PredicatePolicy,
) -> Vec<AlgebraicFiberRootCountReport> {
    if fiber_intervals.is_empty() {
        return Vec::new();
    }
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => {
            return fiber_intervals
                .iter()
                .map(|_| fiber_root_count_error_report(error, 0, 0, Certainty::Exact))
                .collect();
        }
    };
    let first = match local_fiber_polynomial(polynomial, retained_parameter, &mut field) {
        Ok(first) => first,
        Err(error) => {
            return fiber_intervals
                .iter()
                .map(|_| {
                    fiber_root_count_error_report(error, 0, field.refinement_steps, field.certainty)
                })
                .collect();
        }
    };
    let kernel = match prepare_local_open_interval_root_count(first, &mut field) {
        Ok(kernel) => kernel,
        Err(error) => {
            return fiber_intervals
                .iter()
                .map(|_| {
                    fiber_root_count_error_report(error, 0, field.refinement_steps, field.certainty)
                })
                .collect();
        }
    };
    let mut sturm_boundary_cache = Vec::with_capacity(fiber_intervals.len() + 1);

    fiber_intervals
        .iter()
        .map(|(fiber_lower, fiber_upper)| {
            match field.compare(fiber_lower, fiber_upper) {
                Ok(Ordering::Less) => {}
                Ok(Ordering::Equal | Ordering::Greater) => {
                    return fiber_root_count_error_report(
                        LocalFieldError::InvalidInterval,
                        kernel.sequence_length(),
                        field.refinement_steps,
                        field.certainty,
                    );
                }
                Err(error) => {
                    return fiber_root_count_error_report(
                        error,
                        kernel.sequence_length(),
                        field.refinement_steps,
                        field.certainty,
                    );
                }
            }
            let outcome = match &kernel {
                LocalOpenIntervalRootCount::IdenticallyZeroFiber => {
                    Ok(LocalRootCountOutcome::IdenticallyZeroFiber)
                }
                LocalOpenIntervalRootCount::Constant => Ok(LocalRootCountOutcome::Counted {
                    count: 0,
                    sequence_length: 1,
                }),
                LocalOpenIntervalRootCount::Sturm(sequence) => {
                    // Match the single-interval endpoint/error order. Once the
                    // lower boundary decides the report, evaluating the upper
                    // boundary can only add irrelevant work and refinement.
                    let lower = match cached_local_sturm_boundary_variations(
                        &mut sturm_boundary_cache,
                        sequence,
                        fiber_lower,
                        &mut field,
                    ) {
                        Ok(Some(lower)) => lower,
                        Ok(None) => {
                            return fiber_root_count_outcome_report(
                                Ok(LocalRootCountOutcome::EndpointRoot {
                                    sequence_length: sequence.len(),
                                }),
                                &field,
                            );
                        }
                        Err(error) => {
                            return fiber_root_count_outcome_report(Err(error), &field);
                        }
                    };
                    match cached_local_sturm_boundary_variations(
                        &mut sturm_boundary_cache,
                        sequence,
                        fiber_upper,
                        &mut field,
                    ) {
                        Ok(Some(upper)) => lower
                            .checked_sub(upper)
                            .map(|count| LocalRootCountOutcome::Counted {
                                count,
                                sequence_length: sequence.len(),
                            })
                            .ok_or(LocalFieldError::Undecided),
                        Ok(None) => Ok(LocalRootCountOutcome::EndpointRoot {
                            sequence_length: sequence.len(),
                        }),
                        Err(error) => Err(error),
                    }
                }
            };
            fiber_root_count_outcome_report(outcome, &field)
        })
        .collect()
}

/// Count distinct roots in one closed exact bivariate fiber interval.
///
/// This is the endpoint-owning counterpart to
/// [`count_bivariate_fiber_roots_at_algebraic_parameter`]. Rational roots at
/// either bound are deflated exactly in the local algebraic field and counted
/// once regardless of multiplicity. It is intended for callers that already
/// own both interval endpoints and therefore do not need a half-open topology
/// convention.
pub fn count_bivariate_fiber_roots_at_algebraic_parameter_closed(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootCountReport {
    count_bivariate_fiber_system_roots(
        &[polynomial],
        retained_parameter,
        retained_root,
        fiber_lower,
        fiber_upper,
        FiberIntervalEndpoints::IncludeRoots,
        policy,
    )
}

/// Isolate every distinct root in one selected algebraic fiber interval.
///
/// The fiber is specialized once in `Q(alpha)`. Exact Bernstein subdivision
/// isolates simple roots without local-field division; rational roots
/// encountered at endpoints or subdivision points are deflated exactly and
/// retained as represented witnesses. A local-field Sturm sequence is reserved
/// for inseparable or repeated roots that do not reach a unit Bernstein
/// variation. No primitive element or floating approximation is constructed.
#[allow(clippy::too_many_arguments)]
pub fn isolate_bivariate_fiber_roots_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    config: AlgebraicFiberRootIsolationConfig,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootIsolationReport {
    let error_report = |status, certainty, message| AlgebraicFiberRootIsolationReport {
        status,
        intervals: Vec::new(),
        sturm_sequence_length: 0,
        subdivision_steps: 0,
        retained_refinement_steps: 0,
        certainty,
        message: Some(message),
    };
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => return fiber_root_isolation_error_report(error, Certainty::Exact),
    };
    match field.compare(fiber_lower, fiber_upper) {
        Ok(Ordering::Less) => {}
        Ok(Ordering::Equal | Ordering::Greater) => {
            return error_report(
                AlgebraicFiberRootIsolationStatus::InvalidInterval,
                field.certainty,
                "fiber root isolation requires lower < upper",
            );
        }
        Err(error) => {
            return fiber_root_isolation_error_report_with_progress(error, 0, 0, &field);
        }
    }
    let mut fiber = match local_fiber_polynomial(polynomial, retained_parameter, &mut field) {
        Ok(fiber) => fiber,
        Err(error) => {
            return fiber_root_isolation_error_report_with_progress(error, 0, 0, &field);
        }
    };
    if match local_polynomial_is_zero(&fiber, &mut field) {
        Ok(is_zero) => is_zero,
        Err(error) => {
            return fiber_root_isolation_error_report_with_progress(error, 0, 0, &field);
        }
    } {
        return AlgebraicFiberRootIsolationReport {
            status: AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber,
            intervals: Vec::new(),
            sturm_sequence_length: 0,
            subdivision_steps: 0,
            retained_refinement_steps: field.refinement_steps,
            certainty: field.certainty,
            message: Some("every specialized fiber coefficient vanishes at the retained root"),
        };
    }

    let bernstein_subdivision_steps = match isolate_local_polynomial_roots_bernstein(
        fiber.clone(),
        fiber_lower,
        fiber_upper,
        config,
        &mut field,
    ) {
        Ok((Some(intervals), subdivision_steps)) => {
            return AlgebraicFiberRootIsolationReport {
                status: if intervals.is_empty() {
                    AlgebraicFiberRootIsolationStatus::NoRoots
                } else {
                    AlgebraicFiberRootIsolationStatus::Isolated
                },
                intervals,
                sturm_sequence_length: 0,
                subdivision_steps,
                retained_refinement_steps: field.refinement_steps,
                certainty: field.certainty,
                message: None,
            };
        }
        Ok((None, subdivision_steps)) => subdivision_steps,
        Err(LocalFieldError::Undecided) => 0,
        Err(error) => {
            return fiber_root_isolation_error_report_with_progress(error, 0, 0, &field);
        }
    };

    let mut exact_roots = Vec::new();
    for endpoint in [fiber_lower, fiber_upper] {
        match deflate_local_polynomial_at_rational_root(fiber, endpoint, &mut field) {
            Ok((deflated, had_root)) => {
                fiber = deflated;
                if had_root {
                    exact_roots.push(endpoint.clone());
                }
            }
            Err(error) => {
                return fiber_root_isolation_error_report_with_progress(
                    error,
                    0,
                    bernstein_subdivision_steps,
                    &field,
                );
            }
        }
    }

    #[derive(Clone)]
    struct IsolationNode {
        lower: Real,
        upper: Real,
        lower_variations: usize,
        upper_variations: usize,
        depth: usize,
    }

    let mut subdivision_steps = bernstein_subdivision_steps;
    let mut sturm_sequence_length = 0_usize;
    let mut isolated = Vec::new();
    loop {
        if fiber.len() == 1 {
            isolated.clear();
            break;
        }
        // The Sturm sequence retains the unchanged source as its first row.
        // Move the carrier into that row; only a rational-root restart needs
        // to clone it back out for destructive synthetic deflation.
        let sequence = match local_sturm_sequence(std::mem::take(&mut fiber), &mut field) {
            Ok(sequence) => sequence,
            Err(error) => {
                return fiber_root_isolation_error_report_with_progress(
                    error,
                    sturm_sequence_length,
                    subdivision_steps,
                    &field,
                );
            }
        };
        sturm_sequence_length = sequence.len();
        let boundary_variations = |parameter: &Real, field: &mut LocalAlgebraicField| {
            match local_sturm_boundary_variations(&sequence, parameter, field) {
                Ok(Some(variations)) => Ok(variations),
                Ok(None) => Err(LocalFieldError::Undecided),
                Err(error) => Err(error),
            }
        };
        exact_roots.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
        exact_roots.dedup();
        let mut boundaries = Vec::with_capacity(exact_roots.len() + 2);
        boundaries.push(fiber_lower.clone());
        boundaries.extend(
            exact_roots
                .iter()
                .filter(|root| *root > fiber_lower && *root < fiber_upper)
                .cloned(),
        );
        boundaries.push(fiber_upper.clone());
        let boundary_variation_values = match boundaries
            .iter()
            .map(|boundary| boundary_variations(boundary, &mut field))
            .collect::<Result<Vec<_>, _>>()
        {
            Ok(variations) => variations,
            Err(error) => {
                return fiber_root_isolation_error_report_with_progress(
                    error,
                    sturm_sequence_length,
                    subdivision_steps,
                    &field,
                );
            }
        };
        let mut stack = boundaries
            .windows(2)
            .zip(boundary_variation_values.windows(2))
            .rev()
            .map(|(bounds, variations)| IsolationNode {
                lower: bounds[0].clone(),
                upper: bounds[1].clone(),
                lower_variations: variations[0],
                upper_variations: variations[1],
                depth: 0,
            })
            .collect::<Vec<_>>();
        isolated.clear();
        let mut rational_root = None;
        while let Some(mut node) = stack.pop() {
            let count = match node.lower_variations.checked_sub(node.upper_variations) {
                Some(count) => count,
                None => {
                    return fiber_root_isolation_error_report_with_progress(
                        LocalFieldError::Undecided,
                        sturm_sequence_length,
                        subdivision_steps,
                        &field,
                    );
                }
            };
            if count == 0 {
                continue;
            }
            let target_refinements = if count == 1 {
                config.refinement_steps
            } else {
                0
            };
            let mut retained_count = count;
            let mut refinement_steps = 0_usize;
            while count == 1
                && (refinement_steps < target_refinements
                    || exact_roots.contains(&node.lower)
                    || exact_roots.contains(&node.upper))
            {
                // Deflation removed represented roots from this Sturm
                // sequence, but the returned isolator belongs to the original
                // fiber. Its boundaries must exclude those removed roots too.
                if node.depth >= config.max_subdivision_depth {
                    return AlgebraicFiberRootIsolationReport {
                        status: AlgebraicFiberRootIsolationStatus::DepthLimit,
                        intervals: Vec::new(),
                        sturm_sequence_length,
                        subdivision_steps,
                        retained_refinement_steps: field.refinement_steps,
                        certainty: field.certainty,
                        message: Some("selected fiber root refinement exceeded its depth limit"),
                    };
                }
                let midpoint = match (&node.lower + &node.upper) / Real::from(2_u8) {
                    Ok(midpoint) => midpoint,
                    Err(_) => {
                        return fiber_root_isolation_error_report_with_progress(
                            LocalFieldError::Undecided,
                            sturm_sequence_length,
                            subdivision_steps,
                            &field,
                        );
                    }
                };
                subdivision_steps = subdivision_steps.saturating_add(1);
                let midpoint_sign =
                    match local_polynomial_sign_at(&sequence[0], &midpoint, &mut field) {
                        Ok(sign) => sign,
                        Err(error) => {
                            return fiber_root_isolation_error_report_with_progress(
                                error,
                                sturm_sequence_length,
                                subdivision_steps,
                                &field,
                            );
                        }
                    };
                if midpoint_sign == Ordering::Equal {
                    rational_root = Some(midpoint);
                    break;
                }
                let midpoint_variations = match boundary_variations(&midpoint, &mut field) {
                    Ok(variations) => variations,
                    Err(error) => {
                        return fiber_root_isolation_error_report_with_progress(
                            error,
                            sturm_sequence_length,
                            subdivision_steps,
                            &field,
                        );
                    }
                };
                let left_count = match node.lower_variations.checked_sub(midpoint_variations) {
                    Some(count) => count,
                    None => {
                        return fiber_root_isolation_error_report_with_progress(
                            LocalFieldError::Undecided,
                            sturm_sequence_length,
                            subdivision_steps,
                            &field,
                        );
                    }
                };
                let right_count = match midpoint_variations.checked_sub(node.upper_variations) {
                    Some(count) => count,
                    None => {
                        return fiber_root_isolation_error_report_with_progress(
                            LocalFieldError::Undecided,
                            sturm_sequence_length,
                            subdivision_steps,
                            &field,
                        );
                    }
                };
                if left_count == retained_count && right_count == 0 {
                    node.upper = midpoint;
                    node.upper_variations = midpoint_variations;
                } else if right_count == retained_count && left_count == 0 {
                    node.lower = midpoint;
                    node.lower_variations = midpoint_variations;
                } else {
                    return fiber_root_isolation_error_report_with_progress(
                        LocalFieldError::Undecided,
                        sturm_sequence_length,
                        subdivision_steps,
                        &field,
                    );
                }
                node.depth += 1;
                refinement_steps += 1;
                retained_count = left_count.max(right_count);
            }
            if rational_root.is_some() {
                break;
            }
            if retained_count == 1 {
                isolated.push(IsolatedRootInterval {
                    lower: node.lower,
                    upper: node.upper,
                    exact_root: None,
                    distinct_root_count: 1,
                });
                continue;
            }
            if node.depth >= config.max_subdivision_depth {
                return AlgebraicFiberRootIsolationReport {
                    status: AlgebraicFiberRootIsolationStatus::DepthLimit,
                    intervals: Vec::new(),
                    sturm_sequence_length,
                    subdivision_steps,
                    retained_refinement_steps: field.refinement_steps,
                    certainty: field.certainty,
                    message: Some("selected fiber root separation exceeded its depth limit"),
                };
            }
            let midpoint = match (&node.lower + &node.upper) / Real::from(2_u8) {
                Ok(midpoint) => midpoint,
                Err(_) => {
                    return fiber_root_isolation_error_report_with_progress(
                        LocalFieldError::Undecided,
                        sturm_sequence_length,
                        subdivision_steps,
                        &field,
                    );
                }
            };
            subdivision_steps = subdivision_steps.saturating_add(1);
            let midpoint_sign = match local_polynomial_sign_at(&sequence[0], &midpoint, &mut field)
            {
                Ok(sign) => sign,
                Err(error) => {
                    return fiber_root_isolation_error_report_with_progress(
                        error,
                        sturm_sequence_length,
                        subdivision_steps,
                        &field,
                    );
                }
            };
            if midpoint_sign == Ordering::Equal {
                rational_root = Some(midpoint);
                break;
            }
            let midpoint_variations = match boundary_variations(&midpoint, &mut field) {
                Ok(variations) => variations,
                Err(error) => {
                    return fiber_root_isolation_error_report_with_progress(
                        error,
                        sturm_sequence_length,
                        subdivision_steps,
                        &field,
                    );
                }
            };
            let next_depth = node.depth + 1;
            if midpoint_variations > node.upper_variations {
                stack.push(IsolationNode {
                    lower: midpoint.clone(),
                    upper: node.upper,
                    lower_variations: midpoint_variations,
                    upper_variations: node.upper_variations,
                    depth: next_depth,
                });
            }
            if node.lower_variations > midpoint_variations {
                stack.push(IsolationNode {
                    lower: node.lower,
                    upper: midpoint,
                    lower_variations: node.lower_variations,
                    upper_variations: midpoint_variations,
                    depth: next_depth,
                });
            }
        }

        let Some(root) = rational_root else {
            break;
        };
        // The sequence remains borrowed throughout subdivision, so defer this
        // source clone until the rare restart that actually consumes it.
        let source_fiber = sequence[0].clone();
        match deflate_local_polynomial_at_rational_root(source_fiber, &root, &mut field) {
            Ok((deflated, true)) => fiber = deflated,
            Ok((_, false)) => {
                return fiber_root_isolation_error_report_with_progress(
                    LocalFieldError::Undecided,
                    sturm_sequence_length,
                    subdivision_steps,
                    &field,
                );
            }
            Err(error) => {
                return fiber_root_isolation_error_report_with_progress(
                    error,
                    sturm_sequence_length,
                    subdivision_steps,
                    &field,
                );
            }
        }
        if !exact_roots.contains(&root) {
            exact_roots.push(root);
        }
    }

    let mut intervals = isolated;
    intervals.extend(exact_roots.into_iter().map(|root| IsolatedRootInterval {
        lower: root.clone(),
        upper: root.clone(),
        exact_root: Some(root),
        distinct_root_count: 1,
    }));
    intervals.sort_by(|first, second| {
        first
            .lower
            .partial_cmp(&second.lower)
            .unwrap_or(Ordering::Equal)
    });
    AlgebraicFiberRootIsolationReport {
        status: if intervals.is_empty() {
            AlgebraicFiberRootIsolationStatus::NoRoots
        } else {
            AlgebraicFiberRootIsolationStatus::Isolated
        },
        intervals,
        sturm_sequence_length,
        subdivision_steps,
        retained_refinement_steps: field.refinement_steps,
        certainty: field.certainty,
        message: None,
    }
}

/// Isolate a local-field polynomial by exact Bernstein subdivision.
///
/// `None` is a deliberate request to use the division-based Sturm fallback:
/// it occurs only when a basis-change index cannot be represented exactly,
/// subdivision exhausts its accelerator budget, or exact variation
/// bookkeeping does not identify a unique child. Repeated irrational roots
/// require Sturm counting: Bernstein subdivision cannot separate them.
fn isolate_local_polynomial_roots_bernstein(
    polynomial: Vec<LocalFieldElement>,
    fiber_lower: &Real,
    fiber_upper: &Real,
    config: AlgebraicFiberRootIsolationConfig,
    field: &mut LocalAlgebraicField,
) -> Result<(Option<Vec<IsolatedRootInterval>>, usize), LocalFieldError> {
    struct Context<'a> {
        field: &'a mut LocalAlgebraicField,
    }

    impl OrderedFieldPolynomialContext<LocalFieldElement> for Context<'_> {
        type Error = LocalFieldError;

        fn zero(&mut self) -> Result<LocalFieldElement, Self::Error> {
            Ok(LocalFieldElement::zero())
        }

        fn add(
            &mut self,
            left: &LocalFieldElement,
            right: &LocalFieldElement,
        ) -> Result<LocalFieldElement, Self::Error> {
            left.add(right, self.field)
        }

        fn multiply(
            &mut self,
            left: &LocalFieldElement,
            right: &LocalFieldElement,
        ) -> Result<LocalFieldElement, Self::Error> {
            left.multiply(right, self.field)
        }

        fn scale(
            &mut self,
            value: &LocalFieldElement,
            scale: &Real,
        ) -> Result<LocalFieldElement, Self::Error> {
            value.scale(scale, self.field)
        }

        fn sign(&mut self, value: &LocalFieldElement) -> Result<Ordering, Self::Error> {
            value.sign(self.field)
        }

        fn sign_if_separated(
            &mut self,
            value: &LocalFieldElement,
        ) -> Result<Option<Ordering>, Self::Error> {
            value.sign_if_separated(self.field)
        }
    }

    let mut context = Context { field };
    let report = isolate_ordered_field_polynomial_roots(
        polynomial,
        fiber_lower,
        fiber_upper,
        OrderedFieldRootIsolationConfig {
            // Deep subdivision cannot separate a repeated root. Keep this
            // accelerator bounded; the complete Sturm fallback retains the
            // caller's full isolation and refinement depth allowance.
            max_subdivision_depth: config.max_subdivision_depth.min(32),
            refinement_steps: config.refinement_steps,
        },
        &mut context,
    )?;
    match report.status {
        OrderedFieldRootIsolationStatus::Isolated => {
            Ok((Some(report.intervals), report.subdivision_steps))
        }
        OrderedFieldRootIsolationStatus::CompleteFallbackRequired => {
            Ok((None, report.subdivision_steps))
        }
        OrderedFieldRootIsolationStatus::IdenticallyZero
        | OrderedFieldRootIsolationStatus::InvalidInterval => Err(LocalFieldError::Undecided),
    }
}

/// Count distinct common roots of two exact bivariate fibers over a represented root.
///
/// Both original equations are specialized before their actual fiber degrees
/// are inspected. Their GCD is therefore computed in the selected local
/// algebraic field without assuming that either generic leading coefficient
/// survives specialization. This is the authoritative degree-drop-safe path
/// for an isolated candidate fiber.
pub fn count_bivariate_common_fiber_roots_at_algebraic_parameter(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootCountReport {
    count_bivariate_fiber_system_roots(
        &[first, second],
        retained_parameter,
        retained_root,
        fiber_lower,
        fiber_upper,
        FiberIntervalEndpoints::RejectRoots,
        policy,
    )
}

/// Maximally deflates the correlated root where both system parameters agree.
///
/// For a bivariate polynomial `F(s, t)` and a selected algebraic value
/// `s = alpha`, this divides the specialized fiber by `(t - alpha)` until the
/// remainder is nonzero in `Q(alpha)`. The quotient is exported again as a
/// bivariate polynomial reduced modulo alpha's defining polynomial, so a
/// caller can project only the residual contacts and retain the diagonal
/// contact separately. The symmetric `retained_parameter = Second` case
/// divides the first-parameter fiber by `(s - alpha)`.
///
/// Division is synthetic by a monic linear factor. It therefore introduces no
/// local-field denominator and does not construct a primitive-element tower.
pub fn deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicFiberDiagonalDeflationReport {
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => return diagonal_deflation_error_report(error, Certainty::Exact),
    };
    let mut fiber = match local_fiber_polynomial(polynomial, retained_parameter, &mut field) {
        Ok(fiber) => fiber,
        Err(error) => return diagonal_deflation_error_report(error, field.certainty),
    };
    match local_polynomial_is_zero(&fiber, &mut field) {
        Ok(true) => {
            return AlgebraicFiberDiagonalDeflationReport {
                status: AlgebraicFiberDiagonalDeflationStatus::IdenticallyZeroFiber,
                multiplicity: 0,
                reduced_polynomial: None,
                certainty: field.certainty,
            };
        }
        Ok(false) => {}
        Err(error) => return diagonal_deflation_error_report(error, field.certainty),
    }

    let diagonal_root =
        match LocalFieldElement::from_polynomial(vec![Real::zero(), Real::one()], &field) {
            Ok(root) => root,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        };
    let mut multiplicity = 0_usize;
    while fiber.len() > 1 {
        let degree = fiber.len() - 1;
        // Synthetic division produces every quotient coefficient in descending
        // order. Build that order directly so overwritten owning zeros never
        // allocate a numerator vector for each degree.
        let mut quotient = Vec::with_capacity(degree);
        quotient.push(fiber[degree].clone());
        for power in (1..degree).rev() {
            let product = match diagonal_root.multiply(&quotient[quotient.len() - 1], &field) {
                Ok(product) => product,
                Err(error) => return diagonal_deflation_error_report(error, field.certainty),
            };
            let coefficient = match fiber[power].add(&product, &field) {
                Ok(coefficient) => coefficient,
                Err(error) => return diagonal_deflation_error_report(error, field.certainty),
            };
            quotient.push(coefficient);
        }
        let product = match diagonal_root.multiply(&quotient[quotient.len() - 1], &field) {
            Ok(product) => product,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        };
        let remainder = match fiber[0].add(&product, &field) {
            Ok(remainder) => remainder,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        };
        match remainder.is_zero(&mut field) {
            Ok(true) => {
                quotient.reverse();
                fiber = quotient;
                multiplicity += 1;
            }
            Ok(false) => break,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        }
    }

    let reduced_polynomial = match local_fiber_to_bivariate(fiber, retained_parameter) {
        Some(polynomial) => polynomial,
        None => {
            return AlgebraicFiberDiagonalDeflationReport {
                status: AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient,
                multiplicity,
                reduced_polynomial: None,
                certainty: field.certainty,
            };
        }
    };
    AlgebraicFiberDiagonalDeflationReport {
        status: if multiplicity == 0 {
            AlgebraicFiberDiagonalDeflationStatus::NotARoot
        } else {
            AlgebraicFiberDiagonalDeflationStatus::Deflated
        },
        multiplicity,
        reduced_polynomial: Some(reduced_polynomial),
        certainty: field.certainty,
    }
}

/// Reduces `numerator / denominator` modulo one exact algebraic fiber.
///
/// The retained parameter is specialized to `alpha` in its local field and
/// both rational-function polynomials are reduced modulo `fiber_equation` in
/// the other parameter. If their residues differ by one scalar in `Q(alpha)`,
/// that scalar is exported without constructing the degree-multiplied global
/// projection of the selected fiber parameter.
///
/// `ReducedToRetainedField` proves an identity in the complete fiber quotient
/// ring. The caller must still know that the authored rational denominator is
/// nonzero at the selected fiber root; this routine reports
/// `ZeroDenominator` only when it vanishes throughout that quotient ring.
pub fn reduce_bivariate_rational_function_at_algebraic_parameter(
    fiber_equation: &BivariatePolynomial,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicFiberRationalReductionReport {
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, Certainty::Exact);
        }
    };
    let fiber = match local_fiber_polynomial(fiber_equation, retained_parameter, &mut field) {
        Ok(fiber) => fiber,
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
        }
    };
    match local_polynomial_is_zero(&fiber, &mut field) {
        Ok(true) => {
            return algebraic_fiber_rational_reduction_report(
                AlgebraicFiberRationalReductionStatus::IdenticallyZeroFiber,
                None,
                field.certainty,
            );
        }
        Ok(false) if fiber.len() == 1 => {
            return algebraic_fiber_rational_reduction_report(
                AlgebraicFiberRationalReductionStatus::ConstantNonzeroFiber,
                None,
                field.certainty,
            );
        }
        Ok(false) => {}
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
        }
    }

    let reduce = |polynomial: &BivariatePolynomial,
                  field: &mut LocalAlgebraicField|
     -> Result<Vec<LocalFieldElement>, LocalFieldError> {
        let polynomial = local_fiber_polynomial(polynomial, retained_parameter, field)?;
        local_polynomial_remainder(polynomial, &fiber, field)
    };
    let numerator = match reduce(numerator, &mut field) {
        Ok(numerator) => numerator,
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
        }
    };
    let denominator = match reduce(denominator, &mut field) {
        Ok(denominator) => denominator,
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
        }
    };

    let mut pivot = None;
    for (power, coefficient) in denominator.iter().enumerate() {
        match coefficient.is_zero(&mut field) {
            Ok(false) => {
                pivot = Some(power);
                break;
            }
            Ok(true) => {}
            Err(error) => {
                return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
            }
        }
    }
    let Some(pivot) = pivot else {
        return algebraic_fiber_rational_reduction_report(
            AlgebraicFiberRationalReductionStatus::ZeroDenominator,
            None,
            field.certainty,
        );
    };
    let zero = std::cell::OnceCell::new();
    let numerator_pivot = numerator
        .get(pivot)
        .unwrap_or_else(|| zero.get_or_init(LocalFieldElement::zero));
    let ratio = match numerator_pivot.divide_after_nonzero(&denominator[pivot], &mut field) {
        Ok(ratio) => ratio,
        Err(error) => {
            return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
        }
    };

    for power in 0..numerator.len().max(denominator.len()) {
        if power == pivot {
            // `ratio` was constructed from this exact coefficient pair, so
            // its cross-multiplied residual is identically zero.
            continue;
        }
        let numerator_coefficient = numerator
            .get(power)
            .unwrap_or_else(|| zero.get_or_init(LocalFieldElement::zero));
        let denominator_coefficient = denominator
            .get(power)
            .unwrap_or_else(|| zero.get_or_init(LocalFieldElement::zero));
        let residual = match ratio
            .multiply(denominator_coefficient, &field)
            .and_then(|product| numerator_coefficient.subtract(&product, &field))
        {
            Ok(residual) => residual,
            Err(error) => {
                return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
            }
        };
        match residual.is_zero(&mut field) {
            Ok(true) => {}
            Ok(false) => {
                return algebraic_fiber_rational_reduction_report(
                    AlgebraicFiberRationalReductionStatus::FiberDependent,
                    None,
                    field.certainty,
                );
            }
            Err(error) => {
                return algebraic_fiber_rational_reduction_error_report(error, field.certainty);
            }
        }
    }

    algebraic_fiber_rational_reduction_report(
        AlgebraicFiberRationalReductionStatus::ReducedToRetainedField,
        Some(normalized_local_field_rational_coefficients(ratio)),
        field.certainty,
    )
}

fn normalized_local_field_rational_coefficients(
    value: LocalFieldElement,
) -> (Vec<Real>, Vec<Real>) {
    let mut numerator = value.numerator;
    let mut denominator = value.denominator.unwrap_or_else(|| vec![Real::one()]);
    // A local-field quotient is projective in these two coefficient vectors.
    // Make the denominator monic when its exact leading scalar is invertible;
    // otherwise an irrelevant division scale would be raised through every
    // coefficient of a later algebraic image polynomial.
    if let Some(leading) = denominator.last()
        && let Ok(inverse) = Real::one() / leading
    {
        for coefficient in &mut numerator {
            *coefficient *= &inverse;
        }
        for coefficient in &mut denominator {
            *coefficient *= &inverse;
        }
    }
    (numerator, denominator)
}

fn algebraic_fiber_rational_reduction_report(
    status: AlgebraicFiberRationalReductionStatus,
    value: Option<(Vec<Real>, Vec<Real>)>,
    certainty: Certainty,
) -> AlgebraicFiberRationalReductionReport {
    let (numerator_coefficients, denominator_coefficients) = value.unwrap_or_default();
    AlgebraicFiberRationalReductionReport {
        status,
        numerator_coefficients,
        denominator_coefficients,
        certainty,
    }
}

fn algebraic_fiber_rational_reduction_error_report(
    error: LocalFieldError,
    certainty: Certainty,
) -> AlgebraicFiberRationalReductionReport {
    let status = match error {
        LocalFieldError::InvalidEvidence | LocalFieldError::InvalidInterval => {
            AlgebraicFiberRationalReductionStatus::InvalidEvidence
        }
        LocalFieldError::UnsupportedCoefficient => {
            AlgebraicFiberRationalReductionStatus::UnsupportedCoefficient
        }
        LocalFieldError::DivisionByZero | LocalFieldError::Undecided => {
            AlgebraicFiberRationalReductionStatus::Undecided
        }
    };
    algebraic_fiber_rational_reduction_report(status, None, certainty)
}

/// Projects `image_relation(u, z) = 0` through one selected algebraic fiber
/// `fiber_equation(alpha, u) = 0` without first constructing a global value
/// for `u`.
///
/// The first norm is evaluated in `Q(alpha)[u] / (fiber_equation)` by a
/// division-free multiplication-matrix determinant. Its coefficients are
/// made primitive in `alpha` before the second norm through the defining
/// polynomial of `alpha`; this removes pseudo-reduction scale that could
/// otherwise vanish on an unrelated factor of a reducible root carrier.
/// `image_parameter` identifies the `z` axis of `image_relation`; the other
/// axis is `u`.
///
/// The explicit degree limits protect the bounded-memory fast path. A caller
/// whose dimensions exceed them must retain or invoke a complete construction
/// path rather than interpreting `DegreeLimitExceeded` as geometric evidence.
pub fn project_algebraic_fiber_polynomial_image(
    fiber_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    image_relation: &BivariatePolynomial,
    image_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    config: AlgebraicFiberPolynomialImageProjectionConfig,
    policy: PredicatePolicy,
) -> AlgebraicFiberPolynomialImageProjectionReport {
    project_algebraic_fiber_polynomial_image_internal(
        fiber_equation,
        retained_parameter,
        image_relation,
        image_parameter,
        retained_root,
        config,
        policy,
        true,
    )
}

/// Retains the exact polynomial image relation over the selected base root.
///
/// This stops after the first norm in `Q(alpha)[u] / (fiber_equation)` and
/// avoids constructing the degree-multiplied global image polynomial.  A
/// caller can isolate the image directly as another fiber over `alpha`, then
/// replay the authored relation to reject roots contributed by other local
/// fiber conjugates.
pub fn project_algebraic_fiber_polynomial_image_relation(
    fiber_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    image_relation: &BivariatePolynomial,
    image_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    config: AlgebraicFiberPolynomialImageProjectionConfig,
    policy: PredicatePolicy,
) -> AlgebraicFiberPolynomialImageProjectionReport {
    project_algebraic_fiber_polynomial_image_internal(
        fiber_equation,
        retained_parameter,
        image_relation,
        image_parameter,
        retained_root,
        config,
        policy,
        false,
    )
}

#[allow(clippy::too_many_arguments)]
fn project_algebraic_fiber_polynomial_image_internal(
    fiber_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    image_relation: &BivariatePolynomial,
    image_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    config: AlgebraicFiberPolynomialImageProjectionConfig,
    policy: PredicatePolicy,
    construct_global_image: bool,
) -> AlgebraicFiberPolynomialImageProjectionReport {
    let report = |status, coefficients, fiber_degree, image_degree_bound, certainty| {
        AlgebraicFiberPolynomialImageProjectionReport {
            status,
            coefficients,
            retained_relation: None,
            identically_zero_fiber_factor: None,
            fiber_degree,
            image_degree_bound,
            certainty,
        }
    };
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => {
            return algebraic_fiber_polynomial_image_error_report(error, 0, 0, Certainty::Exact);
        }
    };
    let fiber = match local_fiber_polynomial(fiber_equation, retained_parameter, &mut field) {
        Ok(fiber) => fiber,
        Err(error) => {
            return algebraic_fiber_polynomial_image_error_report(error, 0, 0, field.certainty);
        }
    };
    match local_polynomial_is_zero(&fiber, &mut field) {
        Ok(true) => {
            return report(
                AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroFiber,
                Vec::new(),
                0,
                0,
                field.certainty,
            );
        }
        Ok(false) if fiber.len() == 1 => {
            return report(
                AlgebraicFiberPolynomialImageProjectionStatus::ConstantNonzeroFiber,
                Vec::new(),
                0,
                0,
                field.certainty,
            );
        }
        Ok(false) => {}
        Err(error) => {
            return algebraic_fiber_polynomial_image_error_report(error, 0, 0, field.certainty);
        }
    }
    let fiber_degree = fiber.len() - 1;
    if fiber_degree > config.max_fiber_degree
        || field.modulus().len() - 1 > config.max_retained_degree
    {
        return report(
            AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded,
            Vec::new(),
            fiber_degree,
            0,
            field.certainty,
        );
    }

    // Coefficients are indexed first by source-fiber power, then by image
    // power. Strictly proved zero storage does not contribute either degree;
    // an unsupported coefficient remains in the exact determinant.
    let mut image_coefficients = fiber_coefficient_polynomials(image_relation, image_parameter);
    for coefficient in &mut image_coefficients {
        trim_strict_exact_zero_storage(coefficient);
    }
    while image_coefficients.len() > 1
        && image_coefficients
            .last()
            .is_some_and(|coefficient| coefficient.iter().all(strict_exact_zero_for_storage))
    {
        image_coefficients.pop();
    }
    if image_coefficients
        .iter()
        .flatten()
        .all(strict_exact_zero_for_storage)
    {
        return report(
            AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation,
            Vec::new(),
            fiber_degree,
            0,
            field.certainty,
        );
    }
    let image_degree = image_coefficients
        .iter()
        .map(|coefficient| coefficient.len().saturating_sub(1))
        .max()
        .unwrap_or(0);
    let image_degree_bound = match fiber_degree.checked_mul(image_degree) {
        Some(bound) => bound,
        None => {
            return report(
                AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded,
                Vec::new(),
                fiber_degree,
                usize::MAX,
                field.certainty,
            );
        }
    };
    if image_degree_bound > config.max_image_degree_bound {
        return report(
            AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded,
            Vec::new(),
            fiber_degree,
            image_degree_bound,
            field.certainty,
        );
    }
    // A scalar polynomial is already below every validated retained modulus,
    // so no quotient reduction or fallible collection is needed here.
    let image_coefficients = image_coefficients
        .into_iter()
        .map(|coefficient| {
            coefficient
                .into_iter()
                .map(|coefficient| LocalFieldElement {
                    numerator: vec![coefficient],
                    denominator: None,
                })
                .collect()
        })
        .collect::<Vec<_>>();
    let mut norm_fiber = fiber;
    let mut identically_zero_fiber_factor = None;
    let primitive_image =
        match primitive_local_polynomial_image_norm(&norm_fiber, &image_coefficients, &mut field) {
            Ok(Some(image)) => image,
            Ok(None) => {
                let saturated = match saturate_identically_zero_image_fiber_factor(
                    &norm_fiber,
                    &image_coefficients,
                    retained_parameter,
                    &mut field,
                ) {
                    Ok(saturated) => saturated,
                    Err(error) => {
                        return algebraic_fiber_polynomial_image_error_report(
                            error,
                            fiber_degree,
                            image_degree_bound,
                            field.certainty,
                        );
                    }
                };
                let Some((residual_fiber, factor)) = saturated else {
                    return report(
                        AlgebraicFiberPolynomialImageProjectionStatus::Undecided,
                        Vec::new(),
                        fiber_degree,
                        image_degree_bound,
                        field.certainty,
                    );
                };
                identically_zero_fiber_factor = Some(factor);
                norm_fiber = residual_fiber;
                if norm_fiber.len() == 1 {
                    return AlgebraicFiberPolynomialImageProjectionReport {
                    status:
                        AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation,
                    coefficients: Vec::new(),
                    retained_relation: None,
                    identically_zero_fiber_factor,
                    fiber_degree,
                    image_degree_bound,
                    certainty: field.certainty,
                };
                }
                match primitive_local_polynomial_image_norm(
                    &norm_fiber,
                    &image_coefficients,
                    &mut field,
                ) {
                    Ok(Some(image)) => image,
                    Ok(None) => {
                        return AlgebraicFiberPolynomialImageProjectionReport {
                            status: AlgebraicFiberPolynomialImageProjectionStatus::Undecided,
                            coefficients: Vec::new(),
                            retained_relation: None,
                            identically_zero_fiber_factor,
                            fiber_degree,
                            image_degree_bound,
                            certainty: field.certainty,
                        };
                    }
                    Err(error) => {
                        return algebraic_fiber_polynomial_image_error_report(
                            error,
                            fiber_degree,
                            image_degree_bound,
                            field.certainty,
                        );
                    }
                }
            }
            Err(error) => {
                return algebraic_fiber_polynomial_image_error_report(
                    error,
                    fiber_degree,
                    image_degree_bound,
                    field.certainty,
                );
            }
        };
    if !construct_global_image {
        return AlgebraicFiberPolynomialImageProjectionReport {
            status: AlgebraicFiberPolynomialImageProjectionStatus::Constructed,
            coefficients: Vec::new(),
            retained_relation: Some(local_image_retained_relation(primitive_image)),
            identically_zero_fiber_factor,
            fiber_degree,
            image_degree_bound,
            certainty: field.certainty,
        };
    }
    if primitive_image
        .iter()
        .all(|coefficient| coefficient.len() <= 1)
    {
        let coefficients = normalize_projective_image_polynomial(
            primitive_image
                .iter()
                .map(|coefficient| coefficient.first().cloned().unwrap_or_else(Real::zero))
                .collect(),
        );
        return AlgebraicFiberPolynomialImageProjectionReport {
            status: AlgebraicFiberPolynomialImageProjectionStatus::Constructed,
            coefficients,
            retained_relation: Some(local_image_retained_relation(primitive_image)),
            identically_zero_fiber_factor,
            fiber_degree,
            image_degree_bound,
            certainty: field.certainty,
        };
    }
    let coefficients = quotient_ring_fiber_resultant_polynomial(
        field.modulus(),
        &primitive_image,
        config.max_retained_degree,
    );
    let retained_relation = local_image_retained_relation(primitive_image);
    match coefficients {
        Some(coefficients) => AlgebraicFiberPolynomialImageProjectionReport {
            status: AlgebraicFiberPolynomialImageProjectionStatus::Constructed,
            coefficients: normalize_projective_image_polynomial(coefficients),
            retained_relation: Some(retained_relation),
            identically_zero_fiber_factor,
            fiber_degree,
            image_degree_bound,
            certainty: field.certainty,
        },
        None => AlgebraicFiberPolynomialImageProjectionReport {
            status: AlgebraicFiberPolynomialImageProjectionStatus::Undecided,
            coefficients: Vec::new(),
            retained_relation: Some(retained_relation),
            identically_zero_fiber_factor,
            fiber_degree,
            image_degree_bound,
            certainty: field.certainty,
        },
    }
}

fn primitive_local_polynomial_image_norm(
    fiber: &[LocalFieldElement],
    image_coefficients: &[LocalImagePolynomial],
    field: &mut LocalAlgebraicField,
) -> Result<Option<Vec<Vec<Real>>>, LocalFieldError> {
    let fiber_degree = fiber
        .len()
        .checked_sub(1)
        .ok_or(LocalFieldError::Undecided)?;
    let matrix = local_polynomial_quotient_multiplication_matrix(fiber, image_coefficients, field)?;
    let local_image = local_polynomial_matrix_determinant(&matrix, fiber_degree, field)?;
    primitive_local_image_coefficients(local_image, field)
}

/// Removes source roots on which every image-parameter coefficient vanishes.
///
/// A polynomial `F(u)` independent of `z` divides `G(u,z)` over the selected
/// local field exactly when it divides every coefficient of `G` in `z`. Such
/// a factor makes the raw resultant identically zero but contributes no
/// isolated image roots. The returned factor remains explicit so the caller
/// can distinguish an unrelated conjugate component from its selected source
/// root lying on a genuinely positive-dimensional image.
fn saturate_identically_zero_image_fiber_factor(
    fiber: &[LocalFieldElement],
    image_coefficients: &[LocalImagePolynomial],
    retained_parameter: CurveResultantParameter,
    field: &mut LocalAlgebraicField,
) -> Result<Option<(Vec<LocalFieldElement>, BivariatePolynomial)>, LocalFieldError> {
    let image_coefficient_count = image_coefficients.iter().map(Vec::len).max().unwrap_or(1);
    let mut coefficients = Vec::with_capacity(image_coefficient_count);
    for image_power in 0..image_coefficient_count {
        let mut coefficient = image_coefficients
            .iter()
            .map(|source_coefficient| {
                source_coefficient
                    .get(image_power)
                    .cloned()
                    .unwrap_or_else(LocalFieldElement::zero)
            })
            .collect::<Vec<_>>();
        trim_local_polynomial(&mut coefficient, field)?;
        if !local_polynomial_is_zero(&coefficient, field)? {
            coefficients.push(coefficient);
        }
    }
    // The GCD of the image-parameter coefficients is independent of the
    // residual source fiber. Compute it once, then intersect each successive
    // residual with that same content so every primary multiplicity is
    // removed even when the image contains the factor only once.
    let mut common_image_factor = coefficients
        .first_mut()
        .map(std::mem::take)
        .unwrap_or_else(|| vec![LocalFieldElement::zero()]);
    for coefficient in coefficients.iter_mut().skip(1) {
        common_image_factor = local_polynomial_greatest_common_divisor(
            common_image_factor,
            std::mem::take(coefficient),
            field,
        )?;
        if common_image_factor.len() == 1 {
            break;
        }
    }
    let mut residual = fiber.to_vec();
    loop {
        let common_factor = local_polynomial_greatest_common_divisor(
            residual.clone(),
            common_image_factor.clone(),
            field,
        )?;
        if common_factor.len() == 1 {
            break;
        }
        residual = local_polynomial_divide_exact(residual, &common_factor, field)?;
        if residual.len() == 1 {
            break;
        }
    }
    if residual.len() == fiber.len() {
        return Ok(None);
    }
    let factor = local_polynomial_divide_exact(fiber.to_vec(), &residual, field)?;
    let residual = local_polynomial_clear_denominators(residual, field)?;
    let factor = local_polynomial_clear_denominators(factor, field)?;
    let factor =
        local_fiber_to_bivariate(factor, retained_parameter).ok_or(LocalFieldError::Undecided)?;
    Ok(Some((residual, factor)))
}

fn local_image_retained_relation(image: Vec<Vec<Real>>) -> BivariatePolynomial {
    let retained_count = image.iter().map(Vec::len).max().unwrap_or(1);
    let mut coefficients = vec![vec![Real::zero(); image.len()]; retained_count];
    for (image_power, coefficient) in image.into_iter().enumerate() {
        for (retained_power, value) in coefficient.into_iter().enumerate() {
            coefficients[retained_power][image_power] = value;
        }
    }
    for row in &mut coefficients {
        trim_strict_exact_zero_storage(row);
    }
    while coefficients.len() > 1
        && coefficients
            .last()
            .is_some_and(|row| row.iter().all(strict_exact_zero_for_storage))
    {
        coefficients.pop();
    }
    BivariatePolynomial::new(coefficients)
}

fn normalize_projective_image_polynomial(mut coefficients: Vec<Real>) -> Vec<Real> {
    trim_strict_exact_zero_storage(&mut coefficients);
    if let Some(leading) = coefficients.last().cloned()
        && let Ok(inverse) = Real::one() / leading
    {
        for coefficient in &mut coefficients {
            *coefficient *= &inverse;
        }
    }
    coefficients
}

fn trim_strict_exact_zero_storage(coefficients: &mut Vec<Real>) {
    while coefficients.len() > 1
        && coefficients
            .last()
            .is_some_and(strict_exact_zero_for_storage)
    {
        coefficients.pop();
    }
}

type LocalImagePolynomial = Vec<LocalFieldElement>;

fn local_field_element_is_structurally_zero(coefficient: &LocalFieldElement) -> bool {
    coefficient
        .numerator
        .iter()
        .all(|value| value.zero_status() == ZeroKnowledge::Zero)
}

fn local_image_polynomial_is_structurally_zero(polynomial: &[LocalFieldElement]) -> bool {
    polynomial
        .iter()
        .all(local_field_element_is_structurally_zero)
}

fn local_image_polynomial_zero() -> LocalImagePolynomial {
    vec![LocalFieldElement::zero()]
}

fn local_image_polynomial_one(
    field: &LocalAlgebraicField,
) -> Result<LocalImagePolynomial, LocalFieldError> {
    Ok(vec![LocalFieldElement::from_polynomial(
        vec![Real::one()],
        field,
    )?])
}

fn trim_local_image_polynomial(
    polynomial: &mut LocalImagePolynomial,
    field: &mut LocalAlgebraicField,
) -> Result<(), LocalFieldError> {
    while polynomial.len() > 1
        && polynomial
            .last()
            .ok_or(LocalFieldError::Undecided)?
            .is_zero(field)?
    {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(LocalFieldElement::zero());
    }
    Ok(())
}

fn local_image_polynomial_add(
    first: &[LocalFieldElement],
    second: &[LocalFieldElement],
    subtract: bool,
    field: &mut LocalAlgebraicField,
) -> Result<LocalImagePolynomial, LocalFieldError> {
    if local_image_polynomial_is_structurally_zero(second) {
        return Ok(first.to_vec());
    }
    if local_image_polynomial_is_structurally_zero(first) {
        return Ok(if subtract {
            local_image_polynomial_negated(second)
        } else {
            second.to_vec()
        });
    }
    let mut result = Vec::new();
    result
        .try_reserve_exact(first.len().max(second.len()))
        .map_err(|_| LocalFieldError::Undecided)?;
    for power in 0..first.len().max(second.len()) {
        let first = first
            .get(power)
            .cloned()
            .unwrap_or_else(LocalFieldElement::zero);
        let second = second
            .get(power)
            .cloned()
            .unwrap_or_else(LocalFieldElement::zero);
        result.push(if subtract {
            first.subtract(&second, field)?
        } else {
            first.add(&second, field)?
        });
    }
    trim_local_image_polynomial(&mut result, field)?;
    Ok(result)
}

fn local_image_polynomial_multiply(
    first: &[LocalFieldElement],
    second: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<LocalImagePolynomial, LocalFieldError> {
    if local_image_polynomial_is_structurally_zero(first)
        || local_image_polynomial_is_structurally_zero(second)
    {
        return Ok(local_image_polynomial_zero());
    }
    let length = first
        .len()
        .checked_add(second.len())
        .and_then(|length| length.checked_sub(1))
        .ok_or(LocalFieldError::Undecided)?;
    let mut result: Vec<Option<LocalFieldElement>> = vec![None; length];
    for (first_power, first_coefficient) in first.iter().enumerate() {
        if local_field_element_is_structurally_zero(first_coefficient) {
            continue;
        }
        for (second_power, second_coefficient) in second.iter().enumerate() {
            if local_field_element_is_structurally_zero(second_coefficient) {
                continue;
            }
            let product = first_coefficient.multiply(second_coefficient, field)?;
            let target = &mut result[first_power + second_power];
            match target {
                Some(retained) => *retained = retained.add(&product, field)?,
                None => *target = Some(product),
            }
        }
    }
    let mut result = result
        .into_iter()
        .map(|coefficient| coefficient.unwrap_or_else(LocalFieldElement::zero))
        .collect::<Vec<_>>();
    trim_local_image_polynomial(&mut result, field)?;
    Ok(result)
}

fn local_image_polynomial_scale(
    polynomial: &[LocalFieldElement],
    scale: &LocalFieldElement,
    field: &mut LocalAlgebraicField,
) -> Result<LocalImagePolynomial, LocalFieldError> {
    if local_image_polynomial_is_structurally_zero(polynomial)
        || local_field_element_is_structurally_zero(scale)
    {
        return Ok(local_image_polynomial_zero());
    }
    let mut result = polynomial
        .iter()
        .map(|coefficient| coefficient.multiply(scale, field))
        .collect::<Result<Vec<_>, _>>()?;
    trim_local_image_polynomial(&mut result, field)?;
    Ok(result)
}

fn local_image_polynomial_negated(polynomial: &[LocalFieldElement]) -> LocalImagePolynomial {
    polynomial
        .iter()
        .cloned()
        .map(|mut coefficient| {
            coefficient.negate();
            coefficient
        })
        .collect()
}

/// Builds multiplication by one image relation in the pseudo-quotient ring
/// of the selected fiber. Pseudo-reduction uses only ring operations and one
/// shared power of the source leading coefficient, so every matrix entry
/// remains denominator-free in `Q(alpha)`.
fn local_polynomial_quotient_multiplication_matrix(
    source: &[LocalFieldElement],
    relation: &[LocalImagePolynomial],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalImagePolynomial>, LocalFieldError> {
    let degree = source
        .len()
        .checked_sub(1)
        .ok_or(LocalFieldError::Undecided)?;
    let leading = source.last().ok_or(LocalFieldError::Undecided)?;
    if leading.is_zero(field)? {
        return Err(LocalFieldError::DivisionByZero);
    }
    let relation_degree = relation.len().saturating_sub(1);
    let matrix_len = degree
        .checked_mul(degree)
        .ok_or(LocalFieldError::Undecided)?;
    let mut matrix: Vec<Option<LocalImagePolynomial>> = Vec::new();
    matrix
        .try_reserve_exact(matrix_len)
        .map_err(|_| LocalFieldError::Undecided)?;
    matrix.resize_with(matrix_len, || None);
    let product_len = degree
        .checked_add(relation_degree)
        .ok_or(LocalFieldError::Undecided)?;
    for column in 0..degree {
        let mut product = Vec::new();
        product
            .try_reserve_exact(product_len)
            .map_err(|_| LocalFieldError::Undecided)?;
        product.resize_with(product_len, local_image_polynomial_zero);
        for (power, coefficient) in relation.iter().enumerate() {
            product[column + power].clone_from(coefficient);
        }
        for power in (degree..product_len).rev() {
            let eliminand = std::mem::take(&mut product[power]);
            let shift = power - degree;
            for coefficient in &mut product[..shift] {
                *coefficient = local_image_polynomial_scale(coefficient, leading, field)?;
            }
            for (source_power, source_coefficient) in source[..degree].iter().enumerate() {
                let index = shift + source_power;
                let retained = local_image_polynomial_scale(&product[index], leading, field)?;
                let removed = local_image_polynomial_scale(&eliminand, source_coefficient, field)?;
                product[index] = local_image_polynomial_add(&retained, &removed, true, field)?;
            }
        }
        for row in 0..degree {
            matrix[row * degree + column] = Some(std::mem::take(&mut product[row]));
        }
    }
    let mut completed = Vec::new();
    completed
        .try_reserve_exact(matrix_len)
        .map_err(|_| LocalFieldError::Undecided)?;
    for entry in matrix {
        completed.push(entry.ok_or(LocalFieldError::Undecided)?);
    }
    Ok(completed)
}

/// Division-free Berkowitz determinant over the commutative polynomial ring
/// `Q(alpha)[z]`. The returned characteristic-polynomial constant is adjusted
/// by `(-1)^n` to recover the determinant.
fn local_polynomial_matrix_determinant(
    entries: &[LocalImagePolynomial],
    dimension: usize,
    field: &mut LocalAlgebraicField,
) -> Result<LocalImagePolynomial, LocalFieldError> {
    if entries.len()
        != dimension
            .checked_mul(dimension)
            .ok_or(LocalFieldError::Undecided)?
    {
        return Err(LocalFieldError::Undecided);
    }
    let mut determinant = match dimension {
        0 => local_image_polynomial_one(field)?,
        1 => entries[0].clone(),
        2 => {
            let diagonal = local_image_polynomial_multiply(&entries[0], &entries[3], field)?;
            let off_diagonal = local_image_polynomial_multiply(&entries[1], &entries[2], field)?;
            local_image_polynomial_add(&diagonal, &off_diagonal, true, field)?
        }
        _ => {
            let characteristic = local_polynomial_matrix_characteristic(entries, dimension, field)?;
            let mut determinant = characteristic
                .last()
                .cloned()
                .ok_or(LocalFieldError::Undecided)?;
            if dimension % 2 == 1 {
                determinant = local_image_polynomial_negated(&determinant);
            }
            determinant
        }
    };
    trim_local_image_polynomial(&mut determinant, field)?;
    Ok(determinant)
}

fn local_polynomial_matrix_characteristic(
    entries: &[LocalImagePolynomial],
    dimension: usize,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalImagePolynomial>, LocalFieldError> {
    if entries.len()
        != dimension
            .checked_mul(dimension)
            .ok_or(LocalFieldError::Undecided)?
    {
        return Err(LocalFieldError::Undecided);
    }
    if dimension == 0 {
        return Ok(vec![local_image_polynomial_one(field)?]);
    }
    if dimension == 1 {
        return Ok(vec![
            local_image_polynomial_one(field)?,
            local_image_polynomial_negated(&entries[0]),
        ]);
    }

    let minor_dimension = dimension - 1;
    let mut minor = Vec::new();
    minor
        .try_reserve_exact(
            minor_dimension
                .checked_mul(minor_dimension)
                .ok_or(LocalFieldError::Undecided)?,
        )
        .map_err(|_| LocalFieldError::Undecided)?;
    for row in 1..dimension {
        for column in 1..dimension {
            minor.push(entries[row * dimension + column].clone());
        }
    }
    let minor_characteristic =
        local_polynomial_matrix_characteristic(&minor, minor_dimension, field)?;

    let mut first_column = Vec::new();
    first_column
        .try_reserve_exact(dimension + 1)
        .map_err(|_| LocalFieldError::Undecided)?;
    first_column.push(local_image_polynomial_one(field)?);
    first_column.push(local_image_polynomial_negated(&entries[0]));
    let row = &entries[1..dimension];
    let mut vector = (1..dimension)
        .map(|source_row| entries[source_row * dimension].clone())
        .collect::<Vec<_>>();
    for power in 0..minor_dimension {
        let mut product: Option<LocalImagePolynomial> = None;
        for (left, right) in row.iter().zip(&vector) {
            if local_image_polynomial_is_structurally_zero(left)
                || local_image_polynomial_is_structurally_zero(right)
            {
                continue;
            }
            let term = local_image_polynomial_multiply(left, right, field)?;
            product = Some(match product {
                Some(product) => local_image_polynomial_add(&product, &term, false, field)?,
                None => term,
            });
        }
        let product = product.unwrap_or_else(local_image_polynomial_zero);
        first_column.push(local_image_polynomial_negated(&product));
        if power + 1 == minor_dimension {
            break;
        }
        let mut next = Vec::new();
        next.try_reserve_exact(minor_dimension)
            .map_err(|_| LocalFieldError::Undecided)?;
        for matrix_row in 0..minor_dimension {
            let mut value: Option<LocalImagePolynomial> = None;
            for matrix_column in 0..minor_dimension {
                if local_image_polynomial_is_structurally_zero(
                    &minor[matrix_row * minor_dimension + matrix_column],
                ) || local_image_polynomial_is_structurally_zero(&vector[matrix_column])
                {
                    continue;
                }
                let term = local_image_polynomial_multiply(
                    &minor[matrix_row * minor_dimension + matrix_column],
                    &vector[matrix_column],
                    field,
                )?;
                value = Some(match value {
                    Some(value) => local_image_polynomial_add(&value, &term, false, field)?,
                    None => term,
                });
            }
            next.push(value.unwrap_or_else(local_image_polynomial_zero));
        }
        vector = next;
    }

    let mut characteristic = Vec::new();
    characteristic
        .try_reserve_exact(dimension + 1)
        .map_err(|_| LocalFieldError::Undecided)?;
    for output_power in 0..=dimension {
        let mut coefficient: Option<LocalImagePolynomial> = None;
        for source_power in 0..minor_characteristic.len().min(output_power + 1) {
            let factor = &first_column[output_power - source_power];
            if local_image_polynomial_is_structurally_zero(factor)
                || local_image_polynomial_is_structurally_zero(&minor_characteristic[source_power])
            {
                continue;
            }
            let term = local_image_polynomial_multiply(
                factor,
                &minor_characteristic[source_power],
                field,
            )?;
            coefficient = Some(match coefficient {
                Some(coefficient) => local_image_polynomial_add(&coefficient, &term, false, field)?,
                None => term,
            });
        }
        characteristic.push(coefficient.unwrap_or_else(local_image_polynomial_zero));
    }
    Ok(characteristic)
}

fn primitive_local_image_coefficients(
    mut image: LocalImagePolynomial,
    field: &mut LocalAlgebraicField,
) -> Result<Option<Vec<Vec<Real>>>, LocalFieldError> {
    trim_local_image_polynomial(&mut image, field)?;
    if image.len() == 1 && image[0].is_zero(field)? {
        return Ok(None);
    }
    if image
        .iter()
        .any(|coefficient| coefficient.denominator.is_some())
    {
        return Err(LocalFieldError::Undecided);
    }
    let mut content = None;
    for coefficient in &image {
        if coefficient.is_zero(field)? {
            continue;
        }
        content = Some(match content {
            Some(content) => polynomial_gcd(content, coefficient.numerator.clone(), field.policy)
                .ok_or(LocalFieldError::Undecided)?,
            None => coefficient.numerator.clone(),
        });
        if content.as_ref().is_some_and(|content| content.len() == 1) {
            break;
        }
    }
    let content = content.ok_or(LocalFieldError::Undecided)?;
    image
        .into_iter()
        .map(|coefficient| {
            if coefficient.is_zero(field)? {
                return Ok(vec![Real::zero()]);
            }
            if content.len() == 1 && content[0] == Real::one() {
                return Ok(coefficient.numerator);
            }
            let (quotient, remainder) =
                polynomial_div_rem(coefficient.numerator, &content, field.policy)
                    .ok_or(LocalFieldError::Undecided)?;
            for coefficient in &remainder {
                if coefficient.zero_status() != ZeroKnowledge::Zero
                    && field.compare(coefficient, &Real::zero())? != Ordering::Equal
                {
                    return Err(LocalFieldError::Undecided);
                }
            }
            Ok(quotient)
        })
        .collect::<Result<Vec<_>, _>>()
        .map(Some)
}

fn algebraic_fiber_polynomial_image_error_report(
    error: LocalFieldError,
    fiber_degree: usize,
    image_degree_bound: usize,
    certainty: Certainty,
) -> AlgebraicFiberPolynomialImageProjectionReport {
    let status = match error {
        LocalFieldError::InvalidEvidence | LocalFieldError::InvalidInterval => {
            AlgebraicFiberPolynomialImageProjectionStatus::InvalidEvidence
        }
        LocalFieldError::UnsupportedCoefficient => {
            AlgebraicFiberPolynomialImageProjectionStatus::UnsupportedCoefficient
        }
        LocalFieldError::DivisionByZero | LocalFieldError::Undecided => {
            AlgebraicFiberPolynomialImageProjectionStatus::Undecided
        }
    };
    AlgebraicFiberPolynomialImageProjectionReport {
        status,
        coefficients: Vec::new(),
        retained_relation: None,
        identically_zero_fiber_factor: None,
        fiber_degree,
        image_degree_bound,
        certainty,
    }
}

/// Projects one bivariate fiber through a degree-bounded algebraic quotient ring.
///
/// This is the bounded-memory fallback for systems whose generic bivariate
/// resultant has a large interpolation-degree bound. It computes the norm of
/// the specialized fiber directly; its determinant dimension is the retained
/// root's defining-polynomial degree. As with any resultant, roots contributed
/// by other defining-polynomial branches remain candidates that the caller
/// must replay against the selected root.
///
/// The determinant construction is exponential in the retained root's defining
/// degree, and `max_source_degree` explicitly bounds that matrix dimension.
/// Admission always replays the retained payload under the strict policy.
pub fn project_bivariate_fiber_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    max_source_degree: usize,
) -> AlgebraicFiberProjectionReport {
    // This quotient-ring norm needs only the immutable defining polynomial.
    // Share the exact LocalAlgebraicField admission boundary, then avoid
    // cloning the complete refinable root and constructing an unused cache.
    if !is_valid_local_algebraic_field_evidence(retained_root) {
        return AlgebraicFiberProjectionReport {
            status: AlgebraicFiberProjectionStatus::InvalidEvidence,
            coefficients: Vec::new(),
        };
    }
    let coefficients = match retained_parameter {
        CurveResultantParameter::First => {
            let fiber_coefficients = fiber_coefficient_polynomials(polynomial, retained_parameter);
            quotient_ring_fiber_resultant_polynomial(
                &retained_root.polynomial_coefficients,
                &fiber_coefficients,
                max_source_degree,
            )
        }
        CurveResultantParameter::Second => quotient_ring_fiber_resultant_polynomial(
            &retained_root.polynomial_coefficients,
            &polynomial.coefficients,
            max_source_degree,
        ),
    };
    match coefficients {
        Some(coefficients) => AlgebraicFiberProjectionReport {
            status: AlgebraicFiberProjectionStatus::Constructed,
            coefficients,
        },
        None => AlgebraicFiberProjectionReport {
            status: AlgebraicFiberProjectionStatus::Undecided,
            coefficients: Vec::new(),
        },
    }
}

fn local_fiber_to_bivariate(
    fiber: Vec<LocalFieldElement>,
    retained_parameter: CurveResultantParameter,
) -> Option<BivariatePolynomial> {
    if fiber
        .iter()
        .any(|coefficient| coefficient.denominator.is_some())
    {
        return None;
    }
    let coefficients = match retained_parameter {
        CurveResultantParameter::First => {
            let retained_count = fiber
                .iter()
                .map(|coefficient| coefficient.numerator.len())
                .max()
                .unwrap_or(0);
            let fiber_count = fiber.len();
            let mut coefficients = (0..retained_count)
                .map(|_| Vec::with_capacity(fiber_count))
                .collect::<Vec<_>>();
            for coefficient in fiber {
                let mut numerator = coefficient.numerator.into_iter();
                for retained_row in &mut coefficients {
                    retained_row.push(numerator.next().unwrap_or_else(Real::zero));
                }
                debug_assert!(numerator.next().is_none());
            }
            coefficients
        }
        CurveResultantParameter::Second => fiber
            .into_iter()
            .map(|coefficient| coefficient.numerator)
            .collect(),
    };
    Some(BivariatePolynomial::new(coefficients))
}

fn diagonal_deflation_error_report(
    error: LocalFieldError,
    certainty: Certainty,
) -> AlgebraicFiberDiagonalDeflationReport {
    let status = match error {
        LocalFieldError::InvalidEvidence | LocalFieldError::InvalidInterval => {
            AlgebraicFiberDiagonalDeflationStatus::InvalidEvidence
        }
        LocalFieldError::UnsupportedCoefficient => {
            AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient
        }
        LocalFieldError::DivisionByZero | LocalFieldError::Undecided => {
            AlgebraicFiberDiagonalDeflationStatus::Undecided
        }
    };
    AlgebraicFiberDiagonalDeflationReport {
        status,
        multiplicity: 0,
        reduced_polynomial: None,
        certainty,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FiberIntervalEndpoints {
    RejectRoots,
    IncludeRoots,
}

fn count_bivariate_fiber_system_roots(
    polynomials: &[&BivariatePolynomial],
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    endpoints: FiberIntervalEndpoints,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootCountReport {
    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => {
            return fiber_root_count_error_report(error, 0, 0, Certainty::Exact);
        }
    };
    match field.compare(fiber_lower, fiber_upper) {
        Ok(Ordering::Less) => {}
        Ok(Ordering::Equal | Ordering::Greater) => {
            return fiber_root_count_error_report(
                LocalFieldError::InvalidInterval,
                0,
                field.refinement_steps,
                field.certainty,
            );
        }
        Err(error) => {
            return fiber_root_count_error_report(
                error,
                0,
                field.refinement_steps,
                field.certainty,
            );
        }
    }
    let outcome = match polynomials {
        [polynomial] => count_fiber_roots(
            polynomial,
            retained_parameter,
            fiber_lower,
            fiber_upper,
            endpoints,
            &mut field,
        ),
        [first, second] => count_common_fiber_roots(
            first,
            second,
            retained_parameter,
            fiber_lower,
            fiber_upper,
            endpoints,
            &mut field,
        ),
        _ => Err(LocalFieldError::Undecided),
    };
    fiber_root_count_outcome_report(outcome, &field)
}

fn fiber_root_count_outcome_report(
    outcome: Result<LocalRootCountOutcome, LocalFieldError>,
    field: &LocalAlgebraicField,
) -> AlgebraicFiberRootCountReport {
    match outcome {
        Ok(LocalRootCountOutcome::Counted {
            count,
            sequence_length,
        }) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::Counted,
            Some(count),
            sequence_length,
            field.refinement_steps,
            field.certainty,
            None,
        ),
        Ok(LocalRootCountOutcome::IdenticallyZeroFiber) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber,
            None,
            0,
            field.refinement_steps,
            field.certainty,
            Some("every specialized fiber coefficient vanishes at the retained root"),
        ),
        Ok(LocalRootCountOutcome::EndpointRoot { sequence_length }) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::EndpointRoot,
            None,
            sequence_length,
            field.refinement_steps,
            field.certainty,
            Some("the specialized fiber has a root on an interval endpoint"),
        ),
        Err(error) => {
            fiber_root_count_error_report(error, 0, field.refinement_steps, field.certainty)
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LocalRootCountOutcome {
    Counted {
        count: usize,
        sequence_length: usize,
    },
    IdenticallyZeroFiber,
    EndpointRoot {
        sequence_length: usize,
    },
}

enum LocalOpenIntervalRootCount {
    IdenticallyZeroFiber,
    Constant,
    Sturm(Vec<Vec<LocalFieldElement>>),
}

impl LocalOpenIntervalRootCount {
    fn sequence_length(&self) -> usize {
        match self {
            Self::IdenticallyZeroFiber => 0,
            Self::Constant => 1,
            Self::Sturm(sequence) => sequence.len(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LocalFieldError {
    InvalidEvidence,
    InvalidInterval,
    UnsupportedCoefficient,
    DivisionByZero,
    Undecided,
}

fn local_evaluation_sign(
    evaluation: &AlgebraicRootPolynomialEvaluationReport,
) -> Result<Option<Ordering>, LocalFieldError> {
    if let Some(sign) = evaluation.sign {
        return Ok(Some(sign));
    }
    match evaluation.status {
        AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero => Ok(None),
        AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence => {
            Err(LocalFieldError::InvalidEvidence)
        }
        AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial => {
            Err(LocalFieldError::UnsupportedCoefficient)
        }
        AlgebraicRootPolynomialEvaluationStatus::Undecided
        | AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
        | AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedNegative => {
            Err(LocalFieldError::Undecided)
        }
    }
}

fn count_fiber_roots(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    fiber_lower: &Real,
    fiber_upper: &Real,
    endpoints: FiberIntervalEndpoints,
    field: &mut LocalAlgebraicField,
) -> Result<LocalRootCountOutcome, LocalFieldError> {
    let first = local_fiber_polynomial(polynomial, retained_parameter, field)?;
    count_local_polynomial_roots(first, fiber_lower, fiber_upper, endpoints, field)
}

fn count_common_fiber_roots(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    fiber_lower: &Real,
    fiber_upper: &Real,
    endpoints: FiberIntervalEndpoints,
    field: &mut LocalAlgebraicField,
) -> Result<LocalRootCountOutcome, LocalFieldError> {
    let first = local_fiber_polynomial(first, retained_parameter, field)?;
    let second = local_fiber_polynomial(second, retained_parameter, field)?;
    let first_is_zero = local_polynomial_is_zero(&first, field)?;
    let second_is_zero = local_polynomial_is_zero(&second, field)?;
    match (first_is_zero, second_is_zero) {
        (true, true) => return Ok(LocalRootCountOutcome::IdenticallyZeroFiber),
        (true, false) => {
            return count_local_polynomial_roots(
                second,
                fiber_lower,
                fiber_upper,
                endpoints,
                field,
            );
        }
        (false, true) => {
            return count_local_polynomial_roots(first, fiber_lower, fiber_upper, endpoints, field);
        }
        (false, false) => {}
    }
    if first.len() == 1 || second.len() == 1 {
        return Ok(LocalRootCountOutcome::Counted {
            count: 0,
            sequence_length: 1,
        });
    }
    let gcd = local_polynomial_greatest_common_divisor(first, second, field)?;
    count_local_polynomial_roots(gcd, fiber_lower, fiber_upper, endpoints, field)
}

fn local_fiber_polynomial(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    let mut specialized = fiber_coefficient_polynomials(polynomial, retained_parameter)
        .into_iter()
        .map(|coefficient| LocalFieldElement::from_polynomial(coefficient, field))
        .collect::<Result<Vec<_>, _>>()?;
    trim_local_polynomial(&mut specialized, field)?;
    Ok(specialized)
}

fn local_polynomial_is_zero(
    polynomial: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<bool, LocalFieldError> {
    Ok(polynomial.len() == 1 && polynomial[0].is_zero(field)?)
}

fn count_local_polynomial_roots(
    mut first: Vec<LocalFieldElement>,
    fiber_lower: &Real,
    fiber_upper: &Real,
    endpoints: FiberIntervalEndpoints,
    field: &mut LocalAlgebraicField,
) -> Result<LocalRootCountOutcome, LocalFieldError> {
    if local_polynomial_is_zero(&first, field)? {
        return Ok(LocalRootCountOutcome::IdenticallyZeroFiber);
    }
    if first.len() == 1 {
        return Ok(LocalRootCountOutcome::Counted {
            count: 0,
            sequence_length: 1,
        });
    }

    let mut endpoint_root_count = 0_usize;
    if endpoints == FiberIntervalEndpoints::IncludeRoots {
        let (deflated, had_root) =
            deflate_local_polynomial_at_rational_root(first, fiber_lower, field)?;
        first = deflated;
        endpoint_root_count += usize::from(had_root);
        let (deflated, had_root) =
            deflate_local_polynomial_at_rational_root(first, fiber_upper, field)?;
        first = deflated;
        endpoint_root_count += usize::from(had_root);
        if first.len() == 1 {
            return Ok(LocalRootCountOutcome::Counted {
                count: endpoint_root_count,
                sequence_length: 1,
            });
        }
    }

    let sequence = local_sturm_sequence(first, field)?;
    if endpoints == FiberIntervalEndpoints::RejectRoots {
        return count_local_sturm_sequence_roots(&sequence, fiber_lower, fiber_upper, field);
    }
    let sequence_length = sequence.len();
    let lower_variations = local_sign_variations(&sequence, fiber_lower, field)?;
    let upper_variations = local_sign_variations(&sequence, fiber_upper, field)?;
    let count = lower_variations
        .checked_sub(upper_variations)
        .and_then(|count| count.checked_add(endpoint_root_count))
        .ok_or(LocalFieldError::Undecided)?;
    Ok(LocalRootCountOutcome::Counted {
        count,
        sequence_length,
    })
}

fn prepare_local_open_interval_root_count(
    first: Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<LocalOpenIntervalRootCount, LocalFieldError> {
    if local_polynomial_is_zero(&first, field)? {
        return Ok(LocalOpenIntervalRootCount::IdenticallyZeroFiber);
    }
    if first.len() == 1 {
        return Ok(LocalOpenIntervalRootCount::Constant);
    }
    Ok(LocalOpenIntervalRootCount::Sturm(local_sturm_sequence(
        first, field,
    )?))
}

fn local_sturm_sequence(
    first: Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<Vec<LocalFieldElement>>, LocalFieldError> {
    let second = derivative_local_polynomial(&first, field)?;
    let mut sequence = vec![first, second];
    loop {
        let Some(last) = sequence.last() else {
            return Err(LocalFieldError::Undecided);
        };
        if last.len() == 1 {
            break;
        }
        let previous = sequence
            .get(sequence.len() - 2)
            .cloned()
            .ok_or(LocalFieldError::Undecided)?;
        let mut remainder = local_polynomial_remainder(previous, last, field)?;
        if remainder.len() == 1 && remainder[0].is_zero(field)? {
            break;
        }
        for coefficient in &mut remainder {
            coefficient.negate();
        }
        let sign = remainder
            .last()
            .ok_or(LocalFieldError::Undecided)?
            .sign(field)?;
        let leading = match sign {
            Ordering::Less => Real::from(-1_i8),
            Ordering::Greater => Real::one(),
            Ordering::Equal => return Err(LocalFieldError::DivisionByZero),
        };
        sequence.push(normalize_local_polynomial(remainder, leading, field)?);
    }
    Ok(sequence)
}

/// Fixes a nonzero polynomial's leading coefficient to the requested unit.
/// Sturm supplies the original sign so its scale is positive; a GCD needs
/// only monic normalization and introduces no additional sign decision.
fn normalize_local_polynomial(
    mut polynomial: Vec<LocalFieldElement>,
    unit: Real,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    let leading = polynomial.pop().ok_or(LocalFieldError::Undecided)?;
    let normalized = LocalFieldElement::from_polynomial(vec![unit], field)?;
    if !polynomial.is_empty() {
        let scale = normalized.divide_after_nonzero(&leading, field)?;
        for coefficient in &mut polynomial {
            *coefficient = coefficient.multiply(&scale, field)?;
        }
    }
    polynomial.push(normalized);
    Ok(polynomial)
}

fn count_local_sturm_sequence_roots(
    sequence: &[Vec<LocalFieldElement>],
    fiber_lower: &Real,
    fiber_upper: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<LocalRootCountOutcome, LocalFieldError> {
    let sequence_length = sequence.len();
    let Some(lower_variations) = local_sturm_boundary_variations(sequence, fiber_lower, field)?
    else {
        return Ok(LocalRootCountOutcome::EndpointRoot { sequence_length });
    };
    let Some(upper_variations) = local_sturm_boundary_variations(sequence, fiber_upper, field)?
    else {
        return Ok(LocalRootCountOutcome::EndpointRoot { sequence_length });
    };
    let count = lower_variations
        .checked_sub(upper_variations)
        .ok_or(LocalFieldError::Undecided)?;
    Ok(LocalRootCountOutcome::Counted {
        count,
        sequence_length,
    })
}

fn cached_local_sturm_boundary_variations<'a>(
    cache: &mut Vec<(&'a Real, Option<usize>)>,
    sequence: &[Vec<LocalFieldElement>],
    parameter: &'a Real,
    field: &mut LocalAlgebraicField,
) -> Result<Option<usize>, LocalFieldError> {
    if let Some((_, variations)) = cache
        .iter()
        .find(|(cached_parameter, _)| *cached_parameter == parameter)
    {
        return Ok(*variations);
    }
    let variations = local_sturm_boundary_variations(sequence, parameter, field)?;
    cache.push((parameter, variations));
    Ok(variations)
}

fn local_sturm_boundary_variations(
    sequence: &[Vec<LocalFieldElement>],
    parameter: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<Option<usize>, LocalFieldError> {
    let Some(first) = sequence.first() else {
        return Err(LocalFieldError::Undecided);
    };
    let first_sign = local_polynomial_sign_at(first, parameter, field)?;
    if first_sign == Ordering::Equal {
        return Ok(None);
    }
    let mut previous = first_sign;
    let mut variations = 0_usize;
    for polynomial in &sequence[1..] {
        let sign = local_polynomial_sign_at(polynomial, parameter, field)?;
        if sign == Ordering::Equal {
            continue;
        }
        if sign != previous {
            variations += 1;
        }
        previous = sign;
    }
    Ok(Some(variations))
}

fn deflate_local_polynomial_at_rational_root(
    mut polynomial: Vec<LocalFieldElement>,
    root: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<(Vec<LocalFieldElement>, bool), LocalFieldError> {
    let mut had_root = false;
    while polynomial.len() > 1
        && local_polynomial_sign_at(&polynomial, root, field)? == Ordering::Equal
    {
        let degree = polynomial.len() - 1;
        let mut quotient = Vec::with_capacity(degree);
        quotient.push(polynomial[degree].clone());
        for power in (1..degree).rev() {
            let coefficient = quotient
                .last()
                .ok_or(LocalFieldError::Undecided)?
                .scale(root, field)?
                .add(&polynomial[power], field)?;
            quotient.push(coefficient);
        }
        let remainder = quotient
            .last()
            .ok_or(LocalFieldError::Undecided)?
            .scale(root, field)?
            .add(&polynomial[0], field)?;
        if !remainder.is_zero(field)? {
            return Err(LocalFieldError::Undecided);
        }
        quotient.reverse();
        polynomial = quotient;
        had_root = true;
    }
    Ok((polynomial, had_root))
}

fn fiber_coefficient_polynomials(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
) -> Vec<Vec<Real>> {
    match retained_parameter {
        CurveResultantParameter::First => {
            let fiber_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..fiber_count)
                .map(|fiber_power| {
                    polynomial
                        .coefficients
                        .iter()
                        .map(|row| row.get(fiber_power).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect()
        }
        CurveResultantParameter::Second => polynomial.coefficients.clone(),
    }
}

fn derivative_local_polynomial(
    polynomial: &[LocalFieldElement],
    field: &LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    let mut derivative = polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, coefficient)| coefficient.scale(&Real::from(power as u64), field))
        .collect::<Result<Vec<_>, _>>()?;
    if derivative.is_empty() {
        derivative.push(LocalFieldElement::zero());
    }
    Ok(derivative)
}

fn local_polynomial_greatest_common_divisor(
    mut first: Vec<LocalFieldElement>,
    mut second: Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut first, field)?;
    trim_local_polynomial(&mut second, field)?;
    if local_polynomial_is_zero(&first, field)? {
        return Ok(second);
    }
    if local_polynomial_is_zero(&second, field)? {
        return Ok(first);
    }
    if first.len() < second.len() {
        std::mem::swap(&mut first, &mut second);
    }
    while !local_polynomial_is_zero(&second, field)? {
        let remainder = local_polynomial_remainder(first, &second, field)?;
        first = second;
        second = if local_polynomial_is_zero(&remainder, field)? {
            remainder
        } else {
            normalize_local_polynomial(remainder, Real::one(), field)?
        };
    }
    Ok(first)
}

fn local_polynomial_divide_exact(
    mut dividend: Vec<LocalFieldElement>,
    divisor: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut dividend, field)?;
    if divisor.is_empty() || (divisor.len() == 1 && divisor[0].is_zero(field)?) {
        return Err(LocalFieldError::DivisionByZero);
    }
    if dividend.len() < divisor.len() {
        return Err(LocalFieldError::Undecided);
    }
    let divisor_degree = divisor.len() - 1;
    let quotient_len = dividend.len() - divisor_degree;
    // Long division discovers nonzero quotient terms from highest degree to
    // lowest. Retain that order until the end, inserting owning zeros only for
    // degrees genuinely skipped when exact cancellation trims the dividend.
    let mut quotient = Vec::with_capacity(quotient_len);
    let mut next_quotient_degree = quotient_len;
    while dividend.len() >= divisor.len() && !(dividend.len() == 1 && dividend[0].is_zero(field)?) {
        let degree_delta = dividend.len() - divisor.len();
        while next_quotient_degree > degree_delta + 1 {
            quotient.push(LocalFieldElement::zero());
            next_quotient_degree -= 1;
        }
        let scale = dividend.last().ok_or(LocalFieldError::Undecided)?.divide(
            divisor.last().ok_or(LocalFieldError::DivisionByZero)?,
            field,
        )?;
        for (power, divisor_coefficient) in divisor.iter().enumerate().take(divisor_degree) {
            let product = scale.multiply(divisor_coefficient, field)?;
            dividend[degree_delta + power] =
                dividend[degree_delta + power].subtract(&product, field)?;
        }
        dividend.pop().ok_or(LocalFieldError::Undecided)?;
        quotient.push(scale);
        next_quotient_degree = degree_delta;
        trim_local_polynomial(&mut dividend, field)?;
    }
    if !local_polynomial_is_zero(&dividend, field)? {
        return Err(LocalFieldError::Undecided);
    }
    while next_quotient_degree > 0 {
        quotient.push(LocalFieldElement::zero());
        next_quotient_degree -= 1;
    }
    quotient.reverse();
    trim_local_polynomial(&mut quotient, field)?;
    Ok(quotient)
}

/// Multiplies one local polynomial by the product of all coefficient
/// denominators, preserving its roots while exporting denominator-free
/// bivariate evidence.
fn local_polynomial_clear_denominators(
    mut polynomial: Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut polynomial, field)?;
    if polynomial
        .iter()
        .all(|coefficient| coefficient.denominator.is_none())
    {
        return Ok(polynomial);
    }
    let mut prefixes = Vec::with_capacity(polynomial.len() + 1);
    prefixes.push(vec![Real::one()]);
    for coefficient in &polynomial {
        let previous = prefixes.last().ok_or(LocalFieldError::Undecided)?;
        let prefix = match &coefficient.denominator {
            Some(denominator) => field.multiply_polynomials(previous, denominator)?,
            None => previous.clone(),
        };
        prefixes.push(prefix);
    }
    let mut suffix = vec![Real::one()];
    for index in (0..polynomial.len()).rev() {
        let scale = field.multiply_polynomials(&prefixes[index], &suffix)?;
        let numerator = field.multiply_polynomials(&polynomial[index].numerator, &scale)?;
        let denominator = polynomial[index].denominator.take();
        polynomial[index].numerator = numerator;
        if let Some(denominator) = denominator {
            suffix = field.multiply_polynomials(&denominator, &suffix)?;
        }
    }
    trim_local_polynomial(&mut polynomial, field)?;
    Ok(polynomial)
}

fn local_polynomial_remainder(
    mut dividend: Vec<LocalFieldElement>,
    divisor: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut dividend, field)?;
    if divisor.is_empty() || (divisor.len() == 1 && divisor[0].is_zero(field)?) {
        return Err(LocalFieldError::DivisionByZero);
    }
    let divisor_degree = divisor.len() - 1;
    while dividend.len() >= divisor.len() && !(dividend.len() == 1 && dividend[0].is_zero(field)?) {
        let degree_delta = dividend.len() - divisor.len();
        let scale = dividend.last().ok_or(LocalFieldError::Undecided)?.divide(
            divisor.last().ok_or(LocalFieldError::DivisionByZero)?,
            field,
        )?;
        for (power, divisor_coefficient) in divisor.iter().enumerate().take(divisor_degree) {
            let product = scale.multiply(divisor_coefficient, field)?;
            dividend[degree_delta + power] =
                dividend[degree_delta + power].subtract(&product, field)?;
        }
        // Long division chose `scale` precisely to cancel this coefficient.
        // Preserve that field identity directly: expanded numerator and
        // denominator products need not have identical scalar-DAG ordering.
        dividend.pop().ok_or(LocalFieldError::Undecided)?;
        trim_local_polynomial(&mut dividend, field)?;
    }
    Ok(dividend)
}

fn trim_local_polynomial(
    polynomial: &mut Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<(), LocalFieldError> {
    while polynomial.len() > 1
        && polynomial
            .last()
            .ok_or(LocalFieldError::Undecided)?
            .is_zero(field)?
    {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(LocalFieldElement::zero());
    }
    Ok(())
}

fn local_polynomial_sign_at(
    polynomial: &[LocalFieldElement],
    parameter: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<Ordering, LocalFieldError> {
    let Some((leading, remaining)) = polynomial.split_last() else {
        return Ok(Ordering::Equal);
    };
    let mut value = leading.clone();
    for coefficient in remaining.iter().rev() {
        value = value.scale(parameter, field)?.add(coefficient, field)?;
    }
    value.sign(field)
}

fn local_sign_variations(
    sequence: &[Vec<LocalFieldElement>],
    parameter: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<usize, LocalFieldError> {
    let mut previous = None;
    let mut variations = 0_usize;
    for polynomial in sequence {
        let sign = local_polynomial_sign_at(polynomial, parameter, field)?;
        if sign == Ordering::Equal {
            continue;
        }
        if previous.is_some_and(|previous| previous != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    Ok(variations)
}

#[derive(Clone, Debug)]
struct LocalFieldElement {
    numerator: Vec<Real>,
    // `None` is the canonical, allocation-free representation of one.
    denominator: Option<Vec<Real>>,
}

impl LocalFieldElement {
    fn zero() -> Self {
        Self {
            numerator: vec![Real::zero()],
            denominator: None,
        }
    }

    fn from_polynomial(
        numerator: Vec<Real>,
        field: &LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        Ok(Self {
            numerator: field.reduce(numerator)?,
            denominator: None,
        })
    }

    fn add(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        self.combine(other, false, field)
    }

    fn subtract(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        self.combine(other, true, field)
    }

    fn combine(
        &self,
        other: &Self,
        subtract: bool,
        field: &LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        let (numerator, denominator) = match (&self.denominator, &other.denominator) {
            (None, None) => (
                field.add_polynomials(&self.numerator, &other.numerator, subtract)?,
                None,
            ),
            (Some(left_denominator), Some(right_denominator))
                if left_denominator == right_denominator =>
            {
                (
                    field.add_polynomials(&self.numerator, &other.numerator, subtract)?,
                    Some(left_denominator.clone()),
                )
            }
            (Some(left_denominator), None) => {
                let right = field.multiply_polynomials(&other.numerator, left_denominator)?;
                (
                    field.add_polynomials(&self.numerator, &right, subtract)?,
                    Some(left_denominator.clone()),
                )
            }
            (None, Some(right_denominator)) => {
                let left = field.multiply_polynomials(&self.numerator, right_denominator)?;
                (
                    field.add_polynomials(&left, &other.numerator, subtract)?,
                    Some(right_denominator.clone()),
                )
            }
            (Some(left_denominator), Some(right_denominator)) => {
                let left = field.multiply_polynomials(&self.numerator, right_denominator)?;
                let right = field.multiply_polynomials(&other.numerator, left_denominator)?;
                (
                    field.add_polynomials(&left, &right, subtract)?,
                    Some(field.multiply_polynomials(left_denominator, right_denominator)?),
                )
            }
        };
        Ok(Self {
            numerator,
            denominator,
        })
    }

    fn multiply(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        let denominator = match (&self.denominator, &other.denominator) {
            (None, None) => None,
            (Some(denominator), None) | (None, Some(denominator)) => Some(denominator.clone()),
            (Some(left), Some(right)) => Some(field.multiply_polynomials(left, right)?),
        };
        Ok(Self {
            numerator: field.multiply_polynomials(&self.numerator, &other.numerator)?,
            denominator,
        })
    }

    fn divide(
        &self,
        other: &Self,
        field: &mut LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        if other.is_zero(field)? {
            return Err(LocalFieldError::DivisionByZero);
        }
        self.divide_after_nonzero(other, field)
    }

    fn divide_after_nonzero(
        &self,
        other: &Self,
        field: &mut LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        let numerator = match &other.denominator {
            Some(denominator) => field.multiply_polynomials(&self.numerator, denominator)?,
            None => self.numerator.clone(),
        };
        let denominator = match &self.denominator {
            Some(denominator) => field.multiply_polynomials(denominator, &other.numerator)?,
            None => other.numerator.clone(),
        };
        if let Some(inverse) = field.inverse_polynomial(&denominator) {
            return Ok(Self {
                numerator: field.multiply_polynomials(&numerator, &inverse)?,
                denominator: None,
            });
        }
        Ok(Self {
            numerator,
            denominator: field.canonical_denominator(denominator),
        })
    }

    fn scale(&self, scale: &Real, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        Ok(Self {
            numerator: field.reduce(
                self.numerator
                    .iter()
                    .map(|coefficient| coefficient * scale)
                    .collect(),
            )?,
            denominator: self.denominator.clone(),
        })
    }

    fn negate(&mut self) {
        for coefficient in &mut self.numerator {
            *coefficient = -coefficient.clone();
        }
    }

    fn is_zero(&self, field: &mut LocalAlgebraicField) -> Result<bool, LocalFieldError> {
        field.is_zero_polynomial(&self.numerator)
    }

    fn sign(&self, field: &mut LocalAlgebraicField) -> Result<Ordering, LocalFieldError> {
        let numerator = field.sign_polynomial(&self.numerator)?;
        if numerator == Ordering::Equal {
            return Ok(Ordering::Equal);
        }
        let denominator = match &self.denominator {
            Some(denominator) => {
                let sign = field.sign_polynomial(denominator)?;
                if sign == Ordering::Equal {
                    return Err(LocalFieldError::DivisionByZero);
                }
                sign
            }
            None => Ordering::Greater,
        };
        Ok(if numerator == denominator {
            Ordering::Greater
        } else {
            Ordering::Less
        })
    }

    fn sign_if_separated(
        &self,
        field: &mut LocalAlgebraicField,
    ) -> Result<Option<Ordering>, LocalFieldError> {
        let Some(numerator) = field.sign_polynomial_if_separated(&self.numerator)? else {
            return Ok(None);
        };
        if numerator == Ordering::Equal {
            return Ok(Some(Ordering::Equal));
        }
        let denominator = match &self.denominator {
            Some(denominator) => match field.sign_polynomial_if_separated(denominator)? {
                Some(Ordering::Equal) => return Err(LocalFieldError::DivisionByZero),
                Some(sign) => sign,
                None => return Ok(None),
            },
            None => Ordering::Greater,
        };
        Ok(Some(if numerator == denominator {
            Ordering::Greater
        } else {
            Ordering::Less
        }))
    }
}

struct LocalAlgebraicField {
    root: AlgebraicRootRepresentation,
    signed_polynomials: Vec<(Vec<Real>, Ordering)>,
    inverse_polynomials: Vec<(Vec<Real>, Vec<Real>)>,
    policy: PredicatePolicy,
    certainty: Certainty,
    refinement_steps: usize,
}

fn is_valid_local_algebraic_field_evidence(root: &AlgebraicRootRepresentation) -> bool {
    root.is_valid()
        && root.interval.distinct_root_count == 1
        && root.polynomial_coefficients.len() > 1
        && algebraic_root_payload_replays_strictly(root)
}

impl LocalAlgebraicField {
    fn new(
        root: &AlgebraicRootRepresentation,
        policy: PredicatePolicy,
    ) -> Result<Self, LocalFieldError> {
        if !is_valid_local_algebraic_field_evidence(root) {
            return Err(LocalFieldError::InvalidEvidence);
        }
        Ok(Self {
            root: root.clone(),
            signed_polynomials: Vec::new(),
            inverse_polynomials: Vec::new(),
            policy,
            certainty: Certainty::Exact,
            refinement_steps: 0,
        })
    }

    fn modulus(&self) -> &[Real] {
        &self.root.polynomial_coefficients
    }

    fn observe_certainty(&mut self, certainty: Certainty) {
        self.certainty = match (self.certainty, certainty) {
            (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
            (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
            (Certainty::Exact, Certainty::Exact) => Certainty::Exact,
        };
    }

    fn consume<T>(&mut self, outcome: PredicateOutcome<T>) -> Result<T, LocalFieldError> {
        match outcome {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                self.observe_certainty(certainty);
                Ok(value)
            }
            PredicateOutcome::Unknown { .. } => Err(LocalFieldError::Undecided),
        }
    }

    fn compare(&mut self, left: &Real, right: &Real) -> Result<Ordering, LocalFieldError> {
        self.consume(compare_reals(left, right, self.policy))
    }

    fn reduce(&self, mut polynomial: Vec<Real>) -> Result<Vec<Real>, LocalFieldError> {
        while polynomial.len() > 1
            && polynomial
                .last()
                .is_some_and(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            polynomial.pop();
        }
        if polynomial.is_empty() {
            polynomial.push(Real::zero());
        }
        if polynomial.len() < self.modulus().len() {
            return Ok(polynomial);
        }
        polynomial_div_rem(polynomial, self.modulus(), self.policy)
            .map(|(_, remainder)| remainder)
            .ok_or(LocalFieldError::Undecided)
    }

    /// Inverts a nonzero coefficient on the retained real sheet. Exact
    /// Bezout division keeps subsequent arithmetic polynomial in the selected
    /// generator instead of repeatedly multiplying polynomial denominators.
    /// Foreign modulus factors may be removed only with a STRICT nonvanishing
    /// proof at that same selected root. Existing values and sign certificates
    /// remain valid because this changes the presentation, not the root.
    fn inverse_polynomial(&mut self, polynomial: &[Real]) -> Option<Vec<Real>> {
        if let [constant] = polynomial {
            return Some(vec![(Real::one() / constant).ok()?]);
        }
        if let Some((_, inverse)) = self
            .inverse_polynomials
            .iter()
            .find(|(known, _)| known == polynomial)
        {
            return self.reduce(inverse.clone()).ok();
        }
        let strict = PredicatePolicy::STRICT;
        let mut modulus = self.modulus().to_vec();
        loop {
            if let Some(inverse) = rational_polynomial_inverse_modulo(polynomial, &modulus) {
                self.root.polynomial_coefficients = modulus;
                self.inverse_polynomials
                    .push((polynomial.to_vec(), inverse.clone()));
                return Some(inverse);
            }
            // A modular inverse also declines at zero divisors of a reducible
            // presentation. Reuse the shared GCD before constructing Bezout
            // coefficients: those coefficients can be much larger than the
            // foreign factor that needs to be removed.
            let gcd = polynomial_gcd(modulus.clone(), polynomial.to_vec(), strict)?;
            if gcd.len() > 1 {
                if polynomial_vanishes_at_owned_root(&modulus, &gcd, &self.root.interval, strict)
                    != Some(false)
                {
                    return None;
                }
                let (quotient, remainder) = polynomial_div_rem(modulus.clone(), &gcd, strict)?;
                if quotient.len() <= 1
                    || quotient.len() >= modulus.len()
                    || !remainder
                        .iter()
                        .all(|value| strict_exact_zero_for_storage(value))
                {
                    return None;
                }
                modulus = quotient;
                continue;
            }
            let mut previous = modulus.clone();
            let mut current = polynomial_div_rem(polynomial.to_vec(), &modulus, strict)?.1;
            let mut previous_coefficient = vec![Real::zero()];
            let mut current_coefficient = vec![Real::one()];
            while !current
                .iter()
                .all(|value| strict_exact_zero_for_storage(value))
            {
                // Scale the Euclidean row and its Bezout coefficient
                // together. Retaining irrelevant rational contents here
                // causes exponential height growth even when the reduced
                // inverse itself has moderate coefficients.
                let leading_inverse = (Real::one() / current.last()?).ok()?;
                for coefficient in current.iter_mut().chain(&mut current_coefficient) {
                    *coefficient *= &leading_inverse;
                }
                let (quotient, remainder) = polynomial_div_rem(previous, &current, strict)?;
                let mut coefficient = vec![
                    Real::zero();
                    previous_coefficient
                        .len()
                        .max(quotient.len() + current_coefficient.len() - 1)
                ];
                for (power, value) in previous_coefficient.iter().enumerate() {
                    coefficient[power] += value;
                }
                for (first_power, first) in quotient.iter().enumerate() {
                    for (second_power, second) in current_coefficient.iter().enumerate() {
                        coefficient[first_power + second_power] -= first * second;
                    }
                }
                previous = current;
                current = remainder;
                previous_coefficient = current_coefficient;
                current_coefficient = polynomial_div_rem(coefficient, &modulus, strict)?.1;
            }
            if let [constant] = previous.as_slice() {
                let inverse = (Real::one() / constant).ok()?;
                let inverse = previous_coefficient
                    .into_iter()
                    .map(|value| value * &inverse)
                    .collect::<Vec<_>>();
                self.root.polynomial_coefficients = modulus;
                self.inverse_polynomials
                    .push((polynomial.to_vec(), inverse.clone()));
                return Some(inverse);
            }
            return None;
        }
    }

    fn canonical_denominator(&self, denominator: Vec<Real>) -> Option<Vec<Real>> {
        if denominator.len() == 1 && denominator[0] == Real::one() {
            None
        } else {
            Some(denominator)
        }
    }

    fn add_polynomials(
        &self,
        left: &[Real],
        right: &[Real],
        subtract: bool,
    ) -> Result<Vec<Real>, LocalFieldError> {
        let mut result = vec![Real::zero(); left.len().max(right.len())];
        for (power, coefficient) in left.iter().enumerate() {
            result[power] = result[power].clone() + coefficient;
        }
        for (power, coefficient) in right.iter().enumerate() {
            result[power] = if subtract {
                result[power].clone() - coefficient
            } else {
                result[power].clone() + coefficient
            };
        }
        self.reduce(result)
    }

    fn multiply_polynomials(
        &self,
        left: &[Real],
        right: &[Real],
    ) -> Result<Vec<Real>, LocalFieldError> {
        // Keep small native products cheap. Wider rational coefficient fields
        // share one integer denominator through multiplication and reduction.
        if left.len().saturating_mul(right.len()) >= 64
            && let Some(product) = rational_polynomial_product_modulo(left, right, self.modulus())
        {
            return Ok(product);
        }
        let mut result = vec![Real::zero(); left.len() + right.len() - 1];
        for (left_power, left_coefficient) in left.iter().enumerate() {
            for (right_power, right_coefficient) in right.iter().enumerate() {
                result[left_power + right_power] += left_coefficient * right_coefficient;
            }
        }
        self.reduce(result)
    }

    fn debug_assert_reduced_polynomial(&self, polynomial: &[Real]) {
        debug_assert!(!polynomial.is_empty());
        debug_assert!(polynomial.len() < self.modulus().len());
        debug_assert!(
            polynomial.len() == 1
                || polynomial
                    .last()
                    .is_some_and(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        );
    }

    fn known_polynomial_sign(&self, polynomial: &[Real]) -> Option<Ordering> {
        // Constants do not depend on the selected root. Reuse native exact
        // facts without searching or populating the local polynomial cache.
        if let [constant] = polynomial
            && let Some(sign) = constant.immediate_sign()
        {
            return Some(match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            });
        }
        if polynomial
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            return Some(Ordering::Equal);
        }
        self.signed_polynomials
            .iter()
            .find(|(signed, _)| signed == polynomial)
            .map(|(_, sign)| *sign)
    }

    fn sign_polynomial(&mut self, polynomial: &[Real]) -> Result<Ordering, LocalFieldError> {
        // A prior inverse may have removed a foreign modulus factor.
        if polynomial.len() >= self.modulus().len() {
            let reduced = self.reduce(polynomial.to_vec())?;
            return self.sign_polynomial(&reduced);
        }
        self.debug_assert_reduced_polynomial(polynomial);
        if let Some(sign) = self.known_polynomial_sign(polynomial) {
            return Ok(sign);
        }
        let sign = self.sign_reduced_polynomial(polynomial)?;
        self.signed_polynomials.push((polynomial.to_vec(), sign));
        Ok(sign)
    }

    fn sign_polynomial_if_separated(
        &mut self,
        polynomial: &[Real],
    ) -> Result<Option<Ordering>, LocalFieldError> {
        // A prior inverse may have removed a foreign modulus factor.
        if polynomial.len() >= self.modulus().len() {
            let reduced = self.reduce(polynomial.to_vec())?;
            return self.sign_polynomial_if_separated(&reduced);
        }
        self.debug_assert_reduced_polynomial(polynomial);
        if let Some(sign) = self.known_polynomial_sign(polynomial) {
            return Ok(Some(sign));
        }
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        let mut sign = local_evaluation_sign(&evaluation)?;
        // The primitive-rational local field has a fast modular GCD identity
        // path and should continue discovering exact dyadic subdivision roots.
        // General `Real` fields deliberately stay on interval evidence here.
        if sign.is_none()
            && self
                .modulus()
                .iter()
                .chain(polynomial)
                .all(|coefficient| coefficient.exact_rational_ref().is_some())
        {
            sign = Some(self.sign_reduced_polynomial(polynomial)?);
        }
        if let Some(sign) = sign {
            self.signed_polynomials.push((polynomial.to_vec(), sign));
        }
        Ok(sign)
    }

    fn is_zero_polynomial(&mut self, polynomial: &[Real]) -> Result<bool, LocalFieldError> {
        // A prior inverse may have removed a foreign modulus factor.
        if polynomial.len() >= self.modulus().len() {
            let reduced = self.reduce(polynomial.to_vec())?;
            return self.is_zero_polynomial(&reduced);
        }
        self.debug_assert_reduced_polynomial(polynomial);
        if let Some(sign) = self.known_polynomial_sign(polynomial) {
            return Ok(sign == Ordering::Equal);
        }
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            self.signed_polynomials.push((polynomial.to_vec(), sign));
            return Ok(sign == Ordering::Equal);
        }
        match polynomial_vanishes_at_owned_root(
            self.modulus(),
            polynomial,
            &self.root.interval,
            self.policy,
        ) {
            Some(true) => {
                self.signed_polynomials
                    .push((polynomial.to_vec(), Ordering::Equal));
                return Ok(true);
            }
            Some(false) => return Ok(false),
            None => {}
        }
        // A nonzero local coefficient can remain interval-ambiguous before
        // the selected root is narrow enough. Keep the exact identity query
        // authoritative, but allow ordinary separation to finish first when
        // the general-`Real` GCD is itself undecided.
        for _ in 0..LOCAL_FIELD_INTERVAL_SIGN_REFINEMENT_ROUNDS {
            self.refine_root()?;
            let evaluation =
                evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
            if let Some(sign) = local_evaluation_sign(&evaluation)? {
                self.signed_polynomials.push((polynomial.to_vec(), sign));
                return Ok(sign == Ordering::Equal);
            }
        }
        match polynomial_vanishes_at_owned_root(
            self.modulus(),
            polynomial,
            &self.root.interval,
            self.policy,
        ) {
            Some(true) => {
                self.signed_polynomials
                    .push((polynomial.to_vec(), Ordering::Equal));
                Ok(true)
            }
            Some(false) => Ok(false),
            None => Err(LocalFieldError::Undecided),
        }
    }

    fn sign_reduced_polynomial(
        &mut self,
        polynomial: &[Real],
    ) -> Result<Ordering, LocalFieldError> {
        if self.root.exact_point_witness().is_some() {
            return evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy)
                .sign
                .ok_or(LocalFieldError::Undecided);
        }

        // Most local-field coefficients separate from zero over the existing
        // isolating interval.  Prove that cheap case before constructing a
        // polynomial GCD; a zero coefficient necessarily leaves zero inside
        // the conservative interval enclosure and still reaches the exact
        // common-root test below.
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            return Ok(sign);
        }
        // Ordered sign queries usually need only a tighter isolating interval.
        // Refine once before paying for a common-root GCD.  Exact zero tests
        // use `is_zero_polynomial`, which takes the identity path directly.
        self.refine_root()?;
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            return Ok(sign);
        }

        let vanishes = polynomial_vanishes_at_owned_root(
            self.modulus(),
            polynomial,
            &self.root.interval,
            PredicatePolicy::STRICT,
        );
        if vanishes == Some(true) {
            return Ok(Ordering::Equal);
        }

        // A general `Real` GCD may itself be undecided even when the local
        // coefficient is nonzero. Continue narrowing the already-certified
        // singleton: every nonzero continuous image eventually separates
        // from zero. The bounded fallback preserves the API's non-hanging
        // contract for an unresolved exact identity.
        for _ in 0..LOCAL_FIELD_INTERVAL_SIGN_REFINEMENT_ROUNDS {
            let evaluation =
                evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
            if let Some(sign) = local_evaluation_sign(&evaluation)? {
                return Ok(sign);
            }
            self.refine_root()?;
        }
        // Sign determination by a Sturm-Tarski query avoids narrowing the
        // source root to the magnitude of an expanded field coefficient.
        if let Some(sign) =
            crate::root_sign::sign_at_selected_root(self.modulus(), polynomial, &self.root.interval)
        {
            return Ok(sign);
        }
        // A certified nonzero polynomial image of a selected algebraic root
        // eventually separates from zero. The hot refinement budget must not
        // reject that proof merely because field arithmetic produced a small
        // nonzero coefficient. An unresolved identity still returns Undecided.
        if vanishes == Some(false) {
            loop {
                if self.root.exact_point_witness().is_some() {
                    return Err(LocalFieldError::Undecided);
                }
                self.refine_root()?;
                let evaluation =
                    evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
                if let Some(sign) = local_evaluation_sign(&evaluation)? {
                    return Ok(sign);
                }
            }
        }
        Err(LocalFieldError::Undecided)
    }

    /// Refines the already-certified singleton interval by its retained sign
    /// change. This is the compact exact path for a simple root over general
    /// `Real` coefficients: the representation owns uniqueness, while
    /// opposite endpoint signs select the unique child without rebuilding a
    /// square-free factorization over the coefficient field.
    fn refine_root_by_sign_change(
        &mut self,
        max_refinement_steps: usize,
    ) -> Result<bool, LocalFieldError> {
        if self.root.exact_point_witness().is_some() {
            return Ok(true);
        }
        let mut lower = self.root.interval.lower.clone();
        let mut upper = self.root.interval.upper.clone();
        let mut lower_sign = match self.consume(compare_reals(
            &Real::eval_poly(self.modulus(), &lower),
            &Real::zero(),
            self.policy,
        )) {
            Ok(sign) => sign,
            Err(_) => return Ok(false),
        };
        let upper_sign = match self.consume(compare_reals(
            &Real::eval_poly(self.modulus(), &upper),
            &Real::zero(),
            self.policy,
        )) {
            Ok(sign) => sign,
            Err(_) => return Ok(false),
        };
        if upper_sign == Ordering::Equal {
            self.root.interval = IsolatedRootInterval {
                lower: upper.clone(),
                upper: upper.clone(),
                exact_root: Some(upper),
                distinct_root_count: 1,
            };
            return Ok(true);
        }
        // A positive-width isolator owns (lower, upper], not its lower
        // endpoint. Let the shared refiner recover the square-free sign
        // bracket when that excluded endpoint is another root.
        if lower_sign == Ordering::Equal || lower_sign == upper_sign {
            return Ok(false);
        }

        let mut steps = 0_usize;
        for _ in 0..max_refinement_steps {
            let midpoint = Real::average_pair(&lower, &upper);
            let midpoint_sign = self.consume(compare_reals(
                &Real::eval_poly(self.modulus(), &midpoint),
                &Real::zero(),
                self.policy,
            ))?;
            steps += 1;
            if midpoint_sign == Ordering::Equal {
                lower = midpoint.clone();
                upper = midpoint;
                break;
            }
            if midpoint_sign == lower_sign {
                lower = midpoint;
                lower_sign = midpoint_sign;
            } else if midpoint_sign == upper_sign {
                upper = midpoint;
            } else {
                return Err(LocalFieldError::Undecided);
            }
        }
        if steps == 0 {
            return Ok(false);
        }
        self.refinement_steps += steps;
        self.root.interval = IsolatedRootInterval {
            exact_root: (lower == upper).then(|| lower.clone()),
            lower,
            upper,
            distinct_root_count: 1,
        };
        Ok(true)
    }

    fn refine_root(&mut self) -> Result<(), LocalFieldError> {
        if !self.refine_root_by_sign_change(4)? {
            let refinement = refine_isolated_univariate_polynomial_interval(
                self.modulus(),
                &self.root.interval,
                RootIsolationConfig {
                    policy: self.policy,
                    max_interval_width: None,
                    max_refinement_steps: 4,
                },
            );
            let Some(refined_interval) = refinement.refined_interval else {
                return Err(match refinement.status {
                    IsolatedRootRefinementStatus::InvalidPolynomial
                    | IsolatedRootRefinementStatus::InvalidInterval
                    | IsolatedRootRefinementStatus::NonUnitIsolation => {
                        LocalFieldError::InvalidEvidence
                    }
                    IsolatedRootRefinementStatus::Refined
                    | IsolatedRootRefinementStatus::ExactRoot
                    | IsolatedRootRefinementStatus::Undecided => LocalFieldError::Undecided,
                });
            };
            if refined_interval == self.root.interval {
                return Err(LocalFieldError::Undecided);
            }
            self.refinement_steps += refinement.refinement_steps;
            self.root.interval = refined_interval;
        }
        self.root.validation = validate_algebraic_root_representation(&self.root, self.policy);
        if !self.root.is_valid() {
            return Err(LocalFieldError::InvalidEvidence);
        }
        Ok(())
    }
}

#[cold]
fn fiber_root_isolation_error_report(
    error: LocalFieldError,
    certainty: Certainty,
) -> AlgebraicFiberRootIsolationReport {
    let (status, message) = match error {
        LocalFieldError::InvalidEvidence => (
            AlgebraicFiberRootIsolationStatus::InvalidEvidence,
            "the retained algebraic-root evidence is invalid",
        ),
        LocalFieldError::InvalidInterval => (
            AlgebraicFiberRootIsolationStatus::InvalidInterval,
            "the fiber interval must be strictly ordered",
        ),
        LocalFieldError::UnsupportedCoefficient => (
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient,
            "selected-fiber isolation requires exact-rational coefficients",
        ),
        LocalFieldError::DivisionByZero => (
            AlgebraicFiberRootIsolationStatus::Undecided,
            "a local-field Sturm divisor vanished at the retained root",
        ),
        LocalFieldError::Undecided => (
            AlgebraicFiberRootIsolationStatus::Undecided,
            "exact local-field arithmetic or coefficient signing did not complete",
        ),
    };
    AlgebraicFiberRootIsolationReport {
        status,
        intervals: Vec::new(),
        sturm_sequence_length: 0,
        subdivision_steps: 0,
        retained_refinement_steps: 0,
        certainty,
        message: Some(message),
    }
}

#[cold]
fn fiber_root_isolation_error_report_with_progress(
    error: LocalFieldError,
    sturm_sequence_length: usize,
    subdivision_steps: usize,
    field: &LocalAlgebraicField,
) -> AlgebraicFiberRootIsolationReport {
    let mut report = fiber_root_isolation_error_report(error, field.certainty);
    report.sturm_sequence_length = sturm_sequence_length;
    report.subdivision_steps = subdivision_steps;
    report.retained_refinement_steps = field.refinement_steps;
    report
}

fn fiber_root_count_error_report(
    error: LocalFieldError,
    sturm_sequence_length: usize,
    retained_refinement_steps: usize,
    certainty: Certainty,
) -> AlgebraicFiberRootCountReport {
    let (status, message) = match error {
        LocalFieldError::InvalidEvidence => (
            AlgebraicFiberRootCountStatus::InvalidEvidence,
            "the retained algebraic-root evidence is invalid",
        ),
        LocalFieldError::InvalidInterval => (
            AlgebraicFiberRootCountStatus::InvalidInterval,
            "the fiber interval must be strictly ordered",
        ),
        LocalFieldError::UnsupportedCoefficient => (
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
            "the local-field Sturm package requires exact-rational coefficients",
        ),
        LocalFieldError::DivisionByZero => (
            AlgebraicFiberRootCountStatus::Undecided,
            "a local-field Sturm divisor vanished at the retained root",
        ),
        LocalFieldError::Undecided => (
            AlgebraicFiberRootCountStatus::Undecided,
            "exact local-field arithmetic or coefficient signing did not complete",
        ),
    };
    fiber_root_count_report(
        status,
        None,
        sturm_sequence_length,
        retained_refinement_steps,
        certainty,
        Some(message),
    )
}

fn fiber_root_count_report(
    status: AlgebraicFiberRootCountStatus,
    distinct_root_count: Option<usize>,
    sturm_sequence_length: usize,
    retained_refinement_steps: usize,
    certainty: Certainty,
    message: Option<&'static str>,
) -> AlgebraicFiberRootCountReport {
    AlgebraicFiberRootCountReport {
        status,
        distinct_root_count,
        sturm_sequence_length,
        retained_refinement_steps,
        certainty,
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebraic::{
        AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
        validate_algebraic_root_representation,
    };
    use crate::root_isolation::IsolatedRootInterval;
    use crate::symbolic::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn rational(numerator: i64, denominator: i64) -> Real {
        (real(numerator) / real(denominator)).unwrap()
    }

    pub(super) fn represented_root(
        polynomial_coefficients: Vec<Real>,
        lower: Real,
        upper: Real,
        policy: PredicatePolicy,
    ) -> AlgebraicRootRepresentation {
        let mut root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients,
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        root.validation = validate_algebraic_root_representation(&root, policy);
        assert!(root.is_valid());
        root
    }

    fn represented_exact_root(value: Real, policy: PredicatePolicy) -> AlgebraicRootRepresentation {
        let mut root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-value.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: value.clone(),
                upper: value.clone(),
                exact_root: Some(value),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        root.validation = validate_algebraic_root_representation(&root, policy);
        assert!(root.is_valid());
        root
    }

    #[test]
    fn diagonal_fiber_deflation_removes_the_exact_local_multiplicity() {
        // F(x, y) = (y - x)^3 (y + 1). At every selected x = alpha the
        // correlated root y = alpha has multiplicity three and leaves y + 1.
        let polynomial = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(0), real(1), real(1)],
            vec![real(0), real(0), real(-3), real(-3)],
            vec![real(0), real(3), real(3)],
            vec![real(-1), real(-1)],
        ]);
        let expected = BivariatePolynomial::new(vec![vec![real(1), real(1)]]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(2)],
                rational(2, 3),
                rational(3, 4),
                policy,
            );
            let report = deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberDiagonalDeflationStatus::Deflated
            );
            assert_eq!(report.multiplicity, 3);
            assert_eq!(report.reduced_polynomial, Some(expected.clone()));
            assert_eq!(report.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn local_fiber_export_moves_ragged_coefficients_in_both_orientations() {
        let fiber = vec![
            LocalFieldElement {
                numerator: vec![real(1), real(2)],
                denominator: None,
            },
            LocalFieldElement {
                numerator: vec![real(3)],
                denominator: None,
            },
        ];
        assert_eq!(
            local_fiber_to_bivariate(fiber.clone(), CurveResultantParameter::First),
            Some(BivariatePolynomial::new(vec![
                vec![real(1), real(3)],
                vec![real(2), Real::zero()],
            ]))
        );
        assert_eq!(
            local_fiber_to_bivariate(fiber, CurveResultantParameter::Second),
            Some(BivariatePolynomial::new(vec![
                vec![real(1), real(2)],
                vec![real(3)],
            ]))
        );

        assert_eq!(
            local_fiber_to_bivariate(
                vec![LocalFieldElement {
                    numerator: vec![real(1)],
                    denominator: Some(vec![Real::one()]),
                }],
                CurveResultantParameter::First,
            ),
            None
        );
    }

    #[test]
    fn local_constant_signs_reuse_native_proofs_without_field_cache_entries() {
        let tiny = real(2).powi_i64(-3000).unwrap();
        let atom = (real(2).sqrt().unwrap() + Real::one()).sin();
        let upper = &atom + real(2);
        let lower = &atom - Real::one();
        let retained =
            Real::diff_of_products(&Real::one(), &upper, &Real::one(), &lower) - real(3) + &tiny;
        assert!(retained.immediate_sign().is_none());
        assert_eq!(retained.exact_rational_normal_form(), tiny.exact_rational());
        assert!(retained.exact_rational_ref().is_none());
        assert_eq!(
            retained.immediate_sign(),
            Some(hyperreal::RealSign::Positive)
        );
        let constants = [
            (real(-2), Ordering::Less),
            (Real::zero(), Ordering::Equal),
            (real(2), Ordering::Greater),
            (-tiny.clone(), Ordering::Less),
            (tiny, Ordering::Greater),
            (-real(2).sqrt().unwrap(), Ordering::Less),
            (real(2).sqrt().unwrap(), Ordering::Greater),
            (-Real::pi(), Ordering::Less),
            (Real::pi(), Ordering::Greater),
            (-retained.clone(), Ordering::Less),
            (retained, Ordering::Greater),
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&alpha, policy).unwrap();
            let dependent = vec![Real::one(), Real::one()];
            assert_eq!(field.sign_polynomial(&dependent), Ok(Ordering::Greater));
            assert_eq!(field.signed_polynomials.len(), 1);
            for (constant, expected) in &constants {
                let polynomial = std::slice::from_ref(constant);
                assert_eq!(field.sign_polynomial(polynomial), Ok(*expected));
                assert_eq!(
                    field.sign_polynomial_if_separated(polynomial),
                    Ok(Some(*expected))
                );
                assert_eq!(
                    field.is_zero_polynomial(polynomial),
                    Ok(*expected == Ordering::Equal)
                );
                assert_eq!(field.signed_polynomials.len(), 1);
                assert_eq!(field.refinement_steps, 0);
                assert_eq!(field.certainty, Certainty::Exact);
            }
            assert_eq!(
                field.sign_polynomial_if_separated(&dependent),
                Ok(Some(Ordering::Greater))
            );
            assert_eq!(field.is_zero_polynomial(&dependent), Ok(false));
            assert_eq!(
                field.signed_polynomials,
                vec![(dependent, Ordering::Greater)]
            );
        }
    }

    #[test]
    fn local_constant_signs_keep_the_unresolved_native_fact_fallback() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let atom = (real(2).sqrt().unwrap() + Real::one()).sin();
            let upper = &atom + real(2);
            let lower = &atom - Real::one();
            let constant = Real::diff_of_products(&Real::one(), &upper, &Real::one(), &lower)
                - real(3)
                + real(2).powi_i64(-3000).unwrap();
            assert!(constant.immediate_sign().is_none());
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&alpha, policy).unwrap();
            let polynomial = vec![constant];
            assert_eq!(
                field.sign_polynomial_if_separated(&polynomial),
                Ok(Some(Ordering::Greater))
            );
            assert_eq!(
                field.signed_polynomials,
                vec![(polynomial.clone(), Ordering::Greater)]
            );
            assert_eq!(field.sign_polynomial(&polynomial), Ok(Ordering::Greater));
            assert_eq!(field.is_zero_polynomial(&polynomial), Ok(false));
            assert_eq!(field.refinement_steps, 0);
            assert_eq!(field.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn local_field_replays_stale_root_payloads_before_constant_fiber_shortcuts() {
        let valid = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            PredicatePolicy::STRICT,
        );
        let mut reversed = valid.clone();
        reversed.interval.lower = real(3);
        let mut zero_leading = valid;
        zero_leading.polynomial_coefficients[2] = Real::zero();
        let mut wrong_rational_witness = represented_exact_root(real(2), PredicatePolicy::STRICT);
        wrong_rational_witness.polynomial_coefficients[0] = real(-3);
        let mut outside_witness = represented_exact_root(real(2), PredicatePolicy::STRICT);
        outside_witness.interval.lower = real(3);
        outside_witness.interval.upper = real(4);
        let mut wrong_analytic_witness =
            represented_exact_root(Real::pi(), PredicatePolicy::STRICT);
        wrong_analytic_witness.polynomial_coefficients[0] = -Real::pi() - Real::one();
        for invalid in [
            reversed,
            zero_leading,
            wrong_rational_witness,
            outside_witness,
            wrong_analytic_witness,
        ] {
            assert!(
                invalid.is_valid(),
                "the stale cached flag is deliberately still valid"
            );
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                assert!(LocalAlgebraicField::new(&invalid, policy).is_err());
                for value in [-1, 0, 1] {
                    let polynomial = BivariatePolynomial::new(vec![vec![real(value)]]);
                    for retained_parameter in [
                        CurveResultantParameter::First,
                        CurveResultantParameter::Second,
                    ] {
                        let isolated = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                            &polynomial,
                            retained_parameter,
                            &invalid,
                            &Real::zero(),
                            &Real::one(),
                            AlgebraicFiberRootIsolationConfig::default(),
                            policy,
                        );
                        assert_eq!(
                            isolated.status,
                            AlgebraicFiberRootIsolationStatus::InvalidEvidence
                        );
                        assert!(isolated.intervals.is_empty());
                        let projected = project_bivariate_fiber_at_algebraic_parameter(
                            &polynomial,
                            retained_parameter,
                            &invalid,
                            8,
                        );
                        assert_eq!(
                            projected.status,
                            AlgebraicFiberProjectionStatus::InvalidEvidence
                        );
                        assert!(projected.coefficients.is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn local_field_zero_test_recognizes_an_uncached_singleton() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for point in [rational(1, 2), real(2).sqrt().unwrap(), Real::pi()] {
                let neighbor = &point + Real::one();
                let root = represented_root(
                    vec![&point * &neighbor, -(&point + neighbor), Real::one()],
                    point.clone(),
                    point.clone(),
                    policy,
                );
                assert!(root.exact_point_witness().is_none());
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                assert_eq!(field.is_zero_polynomial(&[-&point, Real::one()]), Ok(true));
                assert_eq!(field.root.exact_point_witness(), Some(&point));
                assert_eq!(
                    field.is_zero_polynomial(&[Real::one() - &point, Real::one()]),
                    Ok(false)
                );
                assert_eq!(field.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn local_fiber_count_preserves_uncached_singleton_identities() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for point in [rational(1, 2), real(2).sqrt().unwrap(), Real::pi()] {
                let neighbor = &point + Real::one();
                let root = represented_root(
                    vec![&point * &neighbor, -(&point + neighbor), Real::one()],
                    point.clone(),
                    point.clone(),
                    policy,
                );
                let mut witnessed = root.clone();
                witnessed.interval.exact_root = Some(point.clone());
                assert!(algebraic_root_payload_replays_strictly(&witnessed));
                for parameter in [
                    CurveResultantParameter::First,
                    CurveResultantParameter::Second,
                ] {
                    for shift in [Real::zero(), Real::one()] {
                        let constant = &shift - &point;
                        let polynomial = BivariatePolynomial::new(match parameter {
                            CurveResultantParameter::First => {
                                vec![vec![constant], vec![Real::one()]]
                            }
                            CurveResultantParameter::Second => vec![vec![constant, Real::one()]],
                        });
                        let count = |source| {
                            count_bivariate_fiber_roots_at_algebraic_parameter(
                                &polynomial,
                                parameter,
                                source,
                                &real(-1),
                                &Real::one(),
                                policy,
                            )
                        };
                        let expected = count(&witnessed);
                        assert_eq!(
                            expected.status,
                            if shift == Real::zero() {
                                AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
                            } else {
                                AlgebraicFiberRootCountStatus::Counted
                            }
                        );
                        assert_eq!(count(&root), expected);
                    }
                }
            }
        }
    }

    #[test]
    fn local_refinement_preserves_half_open_selected_root_ownership() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for selected in [Real::one(), rational(1, 2)] {
                // x(x - selected) has one owned root in (0, 1]. The zero at
                // the excluded lower endpoint must never replace it.
                let alpha = represented_root(
                    vec![Real::zero(), -&selected, Real::one()],
                    Real::zero(),
                    Real::one(),
                    policy,
                );
                let oracle = refine_isolated_univariate_polynomial_interval(
                    &alpha.polynomial_coefficients,
                    &alpha.interval,
                    RootIsolationConfig {
                        policy,
                        max_interval_width: None,
                        max_refinement_steps: 4,
                    },
                );
                assert_eq!(
                    oracle.refined_interval.unwrap().exact_root,
                    Some(selected.clone())
                );
                let mut field = LocalAlgebraicField::new(&alpha, policy).unwrap();
                assert_eq!(
                    field.sign_polynomial(&[-(&selected * rational(1, 2)), Real::one()]),
                    Ok(Ordering::Greater)
                );
                assert_eq!(field.root.exact_point_witness(), Some(&selected));
                assert!(algebraic_root_payload_replays_strictly(&field.root));
                assert_eq!(field.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn local_fiber_count_preserves_half_open_selected_root_ownership() {
        let diagonal =
            BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for selected in [Real::one(), rational(1, 2)] {
                let alpha = represented_root(
                    vec![Real::zero(), -&selected, Real::one()],
                    Real::zero(),
                    Real::one(),
                    policy,
                );
                for parameter in [
                    CurveResultantParameter::First,
                    CurveResultantParameter::Second,
                ] {
                    for (lower, upper, count) in [
                        (&selected * rational(3, 4), &selected * rational(5, 4), 1),
                        (-(&selected * rational(1, 4)), &selected * rational(1, 4), 0),
                    ] {
                        let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                            &diagonal, parameter, &alpha, &lower, &upper, policy,
                        );
                        assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                        assert_eq!(report.distinct_root_count, Some(count));
                        assert_eq!(report.certainty, Certainty::Exact);
                    }
                }
            }
        }
    }

    #[test]
    fn local_refinement_preserves_exact_point_payloads() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for selected in [rational(1, 2), real(2).sqrt().unwrap(), Real::pi()] {
                for (polynomial, upper) in [
                    (vec![-&selected, Real::one()], selected.clone()),
                    (vec![-&selected, Real::one()], &selected + Real::one()),
                    (
                        vec![&selected * &selected, -(&selected * real(2)), Real::one()],
                        &selected + Real::one(),
                    ),
                ] {
                    let alpha =
                        represented_root(polynomial, &selected - Real::one(), upper, policy);
                    let mut field = LocalAlgebraicField::new(&alpha, policy).unwrap();
                    field.refine_root().unwrap();
                    let witness = field.root.exact_point_witness().unwrap();
                    assert_eq!(
                        compare_reals(witness, &selected, PredicatePolicy::STRICT).value(),
                        Some(Ordering::Equal)
                    );
                    assert_eq!(witness.exact_rational_ref(), selected.exact_rational_ref());
                    assert!(algebraic_root_payload_replays_strictly(&field.root));
                    assert_eq!(field.certainty, Certainty::Exact);
                }
                let alpha = represented_exact_root(selected, policy);
                let mut field = LocalAlgebraicField::new(&alpha, policy).unwrap();
                field.refine_root().unwrap();
                assert_eq!(field.root, alpha);
                assert_eq!(field.refinement_steps, 0);
            }
        }
    }

    #[test]
    fn local_sign_refinement_retains_nested_quadratic_root_certificates() {
        let half = rational(1, 2);
        let alpha = half.clone().sqrt().unwrap();
        let selected = -half + (&alpha + rational(1, 4)).sqrt().unwrap();
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (scale, upper, steps) in [
                (Real::one(), &selected + rational(1, 8), 1),
                (Real::pi(), &selected + rational(1, 8), 1),
                (-Real::pi(), &selected + rational(1, 8), 1),
                (-Real::pi(), selected.clone(), 0),
            ] {
                let root = represented_root(
                    vec![-&alpha * &scale, scale.clone(), scale],
                    &selected - rational(1, 8),
                    upper,
                    policy,
                );
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                assert_eq!(field.refine_root_by_sign_change(1), Ok(true));
                let witness = field.root.exact_point_witness().unwrap();
                assert_eq!(
                    compare_reals(witness, &selected, PredicatePolicy::STRICT).value(),
                    Some(Ordering::Equal)
                );
                assert!(algebraic_root_payload_replays_strictly(&field.root));
                assert_eq!(field.refinement_steps, steps);
                assert_eq!(field.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn normalized_sturm_rows_preserve_variations_and_repeated_roots() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for scale_sign in [-1_i64, 1] {
                let root = represented_root(
                    vec![real(-2), Real::zero(), Real::one()],
                    Real::one(),
                    real(2),
                    policy,
                );
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                let element = |coefficients| {
                    LocalFieldElement::from_polynomial(coefficients, &field).unwrap()
                };
                let one = element(vec![Real::one()]);
                let selected_factor = vec![element(vec![Real::zero(), real(-1)]), one.clone()];
                let mut polynomial = vec![element(vec![real(-scale_sign), real(scale_sign)])];
                // Either sign of (alpha-1)*(u+1/2)*(u-alpha)^2*(u-3).
                // Positive row scaling must retain all three distinct roots.
                for factor in [
                    vec![element(vec![rational(1, 2)]), one.clone()],
                    selected_factor.clone(),
                    selected_factor,
                    vec![element(vec![real(-3)]), one],
                ] {
                    polynomial =
                        local_image_polynomial_multiply(&polynomial, &factor, &mut field).unwrap();
                }
                let sequence = local_sturm_sequence(polynomial, &mut field).unwrap();
                for row in sequence.iter().skip(2) {
                    let leading = row.last().unwrap();
                    assert!(leading.denominator.is_none());
                    assert!(
                        leading.numerator == vec![Real::one()]
                            || leading.numerator == vec![real(-1)]
                    );
                }
                for (lower, upper, expected) in [
                    (real(-1), real(4), 3),
                    (Real::zero(), real(2), 1),
                    (rational(3, 2), real(2), 0),
                ] {
                    assert!(
                        matches!(count_local_sturm_sequence_roots(&sequence, &lower, &upper, &mut field).unwrap(), LocalRootCountOutcome::Counted { count, .. } if count == expected)
                    );
                }
                assert_eq!(field.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn local_division_preserves_wide_exact_quotients_beyond_modular_reconstruction() {
        let wide = Real::from(hyperreal::Rational::from_bigint(
            num::BigInt::from(1) << 5000,
        ));
        let norm = &wide * &wide - real(2);
        let expected = vec![(&wide / &norm).unwrap(), (real(-1) / &norm).unwrap()];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let root = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
            let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field).unwrap();
            let divisor =
                LocalFieldElement::from_polynomial(vec![wide.clone(), Real::one()], &field)
                    .unwrap();
            let quotient = one.divide(&divisor, &mut field).unwrap();
            assert!(quotient.denominator.is_none());
            assert_eq!(quotient.numerator, expected);
            assert_eq!(field.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn local_division_preserves_arbitrary_exact_base_coefficients() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let radical = real(2).sqrt().unwrap();
            let root = represented_root(
                vec![-radical, Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
            let divisor =
                LocalFieldElement::from_polynomial(vec![Real::one(), Real::one()], &field).unwrap();
            let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field).unwrap();
            let inverse = one.divide(&divisor, &mut field).unwrap();
            assert!(inverse.denominator.is_none());
            assert!(
                divisor
                    .multiply(&inverse, &field)
                    .unwrap()
                    .subtract(&one, &field)
                    .unwrap()
                    .is_zero(&mut field)
                    .unwrap()
            );
            assert_eq!(field.certainty, Certainty::Exact);
            assert!(algebraic_root_payload_replays_strictly(&field.root));
        }
    }

    #[test]
    fn local_division_removes_only_foreign_factors_and_reuses_older_values() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for modulus in [
                vec![real(6), real(-2), real(-3), Real::one()],
                vec![
                    real(54),
                    real(-54),
                    real(-9),
                    real(25),
                    real(-9),
                    Real::one(),
                ],
            ] {
                // (alpha^2-2)*(alpha-3)^k at alpha=sqrt(2), k=1 or 3.
                let root = represented_root(modulus.clone(), Real::one(), real(2), policy);
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                let old_zero = LocalFieldElement::from_polynomial(
                    vec![real(-2), Real::zero(), Real::one()],
                    &field,
                )
                .unwrap();
                let denominator =
                    LocalFieldElement::from_polynomial(vec![real(-3), Real::one()], &field)
                        .unwrap();
                let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field).unwrap();
                let inverse = one.divide(&denominator, &mut field).unwrap();
                assert!(inverse.denominator.is_none());
                assert_eq!(field.modulus().len(), 3);
                assert!(algebraic_root_payload_replays_strictly(&field.root));
                assert!(old_zero.is_zero(&mut field).unwrap());
                assert_eq!(
                    denominator.multiply(&inverse, &field).unwrap().numerator,
                    vec![Real::one()]
                );
                let modulus = field.modulus().to_vec();
                for _ in 0..8 {
                    let inverse = one.divide(&denominator, &mut field).unwrap();
                    assert!(inverse.denominator.is_none());
                    assert_eq!(field.modulus(), &modulus);
                    assert!(inverse.numerator.len() < modulus.len());
                }
                assert_eq!(field.certainty, Certainty::Exact);
            }

            // At the other selected root, that same factor is zero and must
            // remain part of the field's meaning instead of being saturated.
            let root = represented_root(
                vec![real(6), real(-2), real(-3), Real::one()],
                rational(5, 2),
                rational(7, 2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
            let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field).unwrap();
            let zero =
                LocalFieldElement::from_polynomial(vec![real(-3), Real::one()], &field).unwrap();
            assert!(matches!(
                one.divide(&zero, &mut field),
                Err(LocalFieldError::DivisionByZero)
            ));
            assert_eq!(field.modulus().len(), 4);
        }
    }

    #[test]
    fn local_exact_division_preserves_skipped_degrees_and_reduced_coefficients() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            policy,
        );
        let mut field = LocalAlgebraicField::new(&alpha, policy).expect("valid local field");
        let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field)
            .expect("unit coefficient");
        let two =
            LocalFieldElement::from_polynomial(vec![real(2)], &field).expect("integer coefficient");
        let retained_root =
            LocalFieldElement::from_polynomial(vec![Real::zero(), Real::one()], &field)
                .expect("retained root coefficient");
        let half_root = retained_root
            .divide(&two, &mut field)
            .expect("nonzero rational denominator");
        assert!(half_root.denominator.is_none());

        let divisor = vec![one.clone(), one.clone()];
        let expected = vec![one, LocalFieldElement::zero(), half_root];
        let dividend = local_image_polynomial_multiply(&divisor, &expected, &mut field)
            .expect("exact product");
        let quotient = local_polynomial_divide_exact(dividend, &divisor, &mut field)
            .expect("the generated product divides exactly");

        assert_eq!(quotient.len(), expected.len());
        assert!(quotient[0].denominator.is_none());
        assert!(quotient[1].denominator.is_none());
        assert!(quotient[2].denominator.is_none());
        for (actual, expected) in quotient.iter().zip(&expected) {
            assert!(
                actual
                    .subtract(expected, &field)
                    .expect("local difference")
                    .is_zero(&mut field)
                    .expect("decidable local identity")
            );
        }
        assert_eq!(field.certainty, Certainty::Exact);
    }

    #[test]
    fn local_small_determinant_preserves_row_major_orientation() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            policy,
        );
        let mut field = LocalAlgebraicField::new(&alpha, policy).expect("valid local field");
        let constant = |value| {
            vec![
                LocalFieldElement::from_polynomial(vec![real(value)], &field)
                    .expect("constant local coefficient"),
            ]
        };
        let entries = vec![constant(1), constant(2), constant(3), constant(4)];
        let determinant = local_polynomial_matrix_determinant(&entries, 2, &mut field)
            .expect("valid two-by-two determinant");

        assert_eq!(determinant.len(), 1);
        assert_eq!(determinant[0].numerator, vec![real(-2)]);
        assert!(determinant[0].denominator.is_none());
        assert_eq!(field.certainty, Certainty::Exact);
        assert!(matches!(
            local_polynomial_matrix_determinant(&entries[..3], 2, &mut field),
            Err(LocalFieldError::Undecided)
        ));
    }

    #[test]
    fn local_image_convolution_preserves_sparse_zero_slots() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            policy,
        );
        let mut field = LocalAlgebraicField::new(&alpha, policy).expect("valid local field");
        let one = LocalFieldElement::from_polynomial(vec![Real::one()], &field)
            .expect("unit coefficient");
        let sparse = vec![one.clone(), LocalFieldElement::zero(), one];
        let product = local_image_polynomial_multiply(&sparse, &sparse, &mut field)
            .expect("sparse local convolution");

        assert_eq!(product.len(), 5);
        assert_eq!(product[0].numerator, vec![Real::one()]);
        assert!(local_field_element_is_structurally_zero(&product[1]));
        assert_eq!(product[2].numerator, vec![real(2)]);
        assert!(local_field_element_is_structurally_zero(&product[3]));
        assert_eq!(product[4].numerator, vec![Real::one()]);
        assert!(
            product
                .iter()
                .all(|coefficient| coefficient.denominator.is_none())
        );
        assert_eq!(field.certainty, Certainty::Exact);
    }

    #[test]
    fn rational_fiber_reduction_preserves_degree_fifteen_correlation() {
        // alpha^9 = 1/2 and 32768*u^15 = alpha make u a global degree-135
        // scalar. The correlated coordinate 3/5 + 32768*u^15 nevertheless
        // remains the degree-nine retained-field value 3/5 + alpha.
        let mut retained_polynomial = vec![Real::zero(); 10];
        retained_polynomial[0] = -rational(1, 2);
        retained_polynomial[9] = Real::one();
        let mut fiber_row = vec![Real::zero(); 16];
        fiber_row[15] = real(32_768);
        let fiber = BivariatePolynomial::new(vec![fiber_row.clone(), vec![real(-1)]]);
        let mut numerator_row = fiber_row;
        numerator_row[0] = rational(3, 5);
        let numerator = BivariatePolynomial::new(vec![numerator_row]);
        let denominator = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let dependent = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]);
        let shared_denominator = BivariatePolynomial::new(vec![vec![Real::one(), Real::one()]]);
        let shared_numerator =
            BivariatePolynomial::new(vec![vec![real(3), real(3)], vec![Real::one(), Real::one()]]);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                retained_polynomial.clone(),
                rational(9, 10),
                Real::one(),
                policy,
            );
            let report = reduce_bivariate_rational_function_at_algebraic_parameter(
                &fiber,
                &numerator,
                &denominator,
                CurveResultantParameter::First,
                &alpha,
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberRationalReductionStatus::ReducedToRetainedField
            );
            assert_eq!(
                report.numerator_coefficients,
                vec![rational(3, 5), Real::one()]
            );
            assert_eq!(report.denominator_coefficients, vec![Real::one()]);
            assert_eq!(report.certainty, Certainty::Exact);

            let dependent = reduce_bivariate_rational_function_at_algebraic_parameter(
                &fiber,
                &dependent,
                &denominator,
                CurveResultantParameter::First,
                &alpha,
                policy,
            );
            assert_eq!(
                dependent.status,
                AlgebraicFiberRationalReductionStatus::FiberDependent
            );
            assert_eq!(dependent.certainty, Certainty::Exact);

            let shared = reduce_bivariate_rational_function_at_algebraic_parameter(
                &fiber,
                &shared_numerator,
                &shared_denominator,
                CurveResultantParameter::First,
                &alpha,
                policy,
            );
            assert_eq!(
                shared.status,
                AlgebraicFiberRationalReductionStatus::ReducedToRetainedField
            );
            assert_eq!(shared.numerator_coefficients, vec![real(3), Real::one()]);
            assert_eq!(shared.denominator_coefficients, vec![Real::one()]);
            assert_eq!(shared.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn rational_fiber_reduction_preserves_ragged_zero_and_dependent_residuals() {
        let zero = BivariatePolynomial::new(vec![vec![Real::zero()]]);

        let first_fiber = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::zero(), Real::zero(), Real::one()],
            vec![real(-1)],
        ]);
        let first_denominator = BivariatePolynomial::new(vec![
            vec![real(4), real(4), real(4)],
            vec![real(2), real(2), real(2)],
        ]);
        let first_dependent = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]);

        let second_fiber = BivariatePolynomial::new(vec![
            vec![Real::zero(), real(-1)],
            vec![],
            vec![],
            vec![Real::one()],
        ]);
        let second_denominator = BivariatePolynomial::new(vec![
            vec![real(4), real(2)],
            vec![real(4), real(2)],
            vec![real(4), real(2)],
        ]);
        let second_dependent =
            BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let irreducible = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let reducible = represented_root(
                vec![real(6), real(-2), real(-3), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            for alpha in [&irreducible, &reducible] {
                for (fiber, denominator, dependent, retained_parameter) in [
                    (
                        &first_fiber,
                        &first_denominator,
                        &first_dependent,
                        CurveResultantParameter::First,
                    ),
                    (
                        &second_fiber,
                        &second_denominator,
                        &second_dependent,
                        CurveResultantParameter::Second,
                    ),
                ] {
                    let reduced_zero = reduce_bivariate_rational_function_at_algebraic_parameter(
                        fiber,
                        &zero,
                        denominator,
                        retained_parameter,
                        alpha,
                        policy,
                    );
                    assert_eq!(
                        reduced_zero.status,
                        AlgebraicFiberRationalReductionStatus::ReducedToRetainedField
                    );
                    assert_eq!(reduced_zero.numerator_coefficients, vec![Real::zero()]);
                    assert_eq!(reduced_zero.denominator_coefficients, vec![Real::one()]);
                    assert_eq!(reduced_zero.certainty, Certainty::Exact);

                    let fiber_dependent = reduce_bivariate_rational_function_at_algebraic_parameter(
                        fiber,
                        dependent,
                        &BivariatePolynomial::new(vec![vec![Real::one()]]),
                        retained_parameter,
                        alpha,
                        policy,
                    );
                    assert_eq!(
                        fiber_dependent.status,
                        AlgebraicFiberRationalReductionStatus::FiberDependent
                    );

                    let zero_denominator =
                        reduce_bivariate_rational_function_at_algebraic_parameter(
                            fiber,
                            dependent,
                            &zero,
                            retained_parameter,
                            alpha,
                            policy,
                        );
                    assert_eq!(
                        zero_denominator.status,
                        AlgebraicFiberRationalReductionStatus::ZeroDenominator
                    );
                }
            }
        }
    }

    #[test]
    fn primitive_local_image_content_accepts_policy_certified_zero_remainders() {
        let sqrt_two = real(2).sqrt().expect("positive square root");
        let sqrt_three = real(3).sqrt().expect("positive square root");
        let sum = &sqrt_two + &sqrt_three;
        let radical_zero =
            &sum * &sum - (real(5) + real(2) * real(6).sqrt().expect("positive square root"));

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let mut field = LocalAlgebraicField::new(&alpha, policy).expect("valid local field");
            let image = vec![
                LocalFieldElement::from_polynomial(vec![Real::one(), Real::one()], &field)
                    .expect("reduced first coefficient"),
                LocalFieldElement::from_polynomial(
                    vec![Real::one() + radical_zero.clone(), Real::one()],
                    &field,
                )
                .expect("reduced second coefficient"),
            ];

            let primitive = primitive_local_image_coefficients(image, &mut field)
                .expect("the exact common content should divide both coefficients")
                .expect("the image is not identically zero");
            assert_eq!(primitive, vec![vec![Real::one()], vec![Real::one()]]);
            assert_eq!(field.certainty, Certainty::Exact);

            // u = alpha and (1 + u)(1 + z) + radical_zero = 0 reduce to
            // the primitive image relation 1 + z = 0. Exercise the public
            // projection boundary, not only its local content helper.
            let fiber =
                BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
            let image = BivariatePolynomial::new(vec![
                vec![Real::one() + radical_zero.clone(), Real::one()],
                vec![Real::one(), Real::one()],
            ]);
            let report = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 1,
                    max_retained_degree: 2,
                    max_image_degree_bound: 1,
                },
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(report.coefficients, vec![Real::one(), Real::one()]);
            assert_eq!(report.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn primitive_local_image_content_rejects_unsupported_zero_remainders() {
        let sine = real(1).sin();
        let cosine = real(1).cos();
        let unsupported_zero = &sine * &sine + &cosine * &cosine - Real::one();
        assert_eq!(unsupported_zero.zero_status(), ZeroKnowledge::Unknown);

        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            policy,
        );
        let mut field = LocalAlgebraicField::new(&alpha, policy).expect("valid local field");
        let image = vec![
            LocalFieldElement::from_polynomial(vec![Real::one(), Real::one()], &field)
                .expect("reduced first coefficient"),
            LocalFieldElement::from_polynomial(
                vec![Real::one() + unsupported_zero, Real::one()],
                &field,
            )
            .expect("reduced second coefficient"),
        ];

        assert_eq!(
            primitive_local_image_coefficients(image, &mut field),
            Err(LocalFieldError::Undecided)
        );
    }

    #[test]
    fn polynomial_fiber_image_projects_a_nonsquare_radical_relation_directly() {
        // alpha^2 = 2, u^3 = alpha, and z^2 = 1 + u^2 imply
        // (z^2 - 1)^3 - 2 = 0. The construction must eliminate the complete
        // tower without first representing the degree-six scalar u.
        let fiber = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::zero(), Real::zero(), Real::one()],
            vec![real(-1)],
        ]);
        let image = BivariatePolynomial::new(vec![
            vec![real(-1), Real::zero(), Real::one()],
            vec![Real::zero()],
            vec![real(-1)],
        ]);
        let expected = vec![
            real(-3),
            Real::zero(),
            real(3),
            Real::zero(),
            real(-3),
            Real::zero(),
            Real::one(),
        ];
        // Local primitive normalization is projective; alpha^2 reduces to 2
        // in the retained field before this relation is exported.
        let expected_retained = BivariatePolynomial::new(vec![vec![
            Real::one(),
            Real::zero(),
            real(-1),
            Real::zero(),
            Real::one(),
            Real::zero(),
            rational(-1, 3),
        ]]);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let report = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 3,
                    max_retained_degree: 2,
                    max_image_degree_bound: 6,
                },
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(report.fiber_degree, 3);
            assert_eq!(report.image_degree_bound, 6);
            assert_eq!(report.coefficients, expected);
            assert_eq!(report.retained_relation, Some(expected_retained.clone()));
            assert_eq!(report.certainty, Certainty::Exact);

            let retained = project_algebraic_fiber_polynomial_image_relation(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 3,
                    max_retained_degree: 2,
                    max_image_degree_bound: 6,
                },
                policy,
            );
            assert_eq!(
                retained.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert!(retained.coefficients.is_empty());
            assert_eq!(retained.retained_relation, Some(expected_retained.clone()));
            assert_eq!(retained.certainty, Certainty::Exact);

            let bounded = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 3,
                    max_retained_degree: 2,
                    max_image_degree_bound: 5,
                },
                policy,
            );
            assert_eq!(
                bounded.status,
                AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded
            );
            assert_eq!(bounded.image_degree_bound, 6);
        }
    }

    #[test]
    fn polynomial_fiber_image_retains_base_parameter_axis() {
        // u=alpha and z=u must remain z-alpha over the selected base instead
        // of being flattened immediately to the global z^2-2 eliminant.
        let fiber = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        let image = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        let expected =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let report = project_algebraic_fiber_polynomial_image_relation(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 1,
                    max_retained_degree: 2,
                    max_image_degree_bound: 1,
                },
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(report.retained_relation, Some(expected.clone()));
            assert!(report.coefficients.is_empty());
            assert_eq!(report.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn polynomial_fiber_image_trims_only_strictly_exact_zero_degree() {
        let positive = crate::test_support::exact_normal_positive();
        let normalized_zero = real(2).powi_i64(-3000).unwrap() - positive.clone();
        let fiber = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        let relation = |trailing| {
            BivariatePolynomial::new(vec![
                vec![Real::zero(), Real::one(), trailing],
                vec![real(-1)],
            ])
        };
        let alpha = represented_root(
            vec![real(-2), Real::zero(), Real::one()],
            Real::one(),
            real(2),
            PredicatePolicy::STRICT,
        );
        let config = AlgebraicFiberPolynomialImageProjectionConfig {
            max_fiber_degree: 1,
            max_retained_degree: 2,
            max_image_degree_bound: 1,
        };

        assert_eq!(
            normalize_projective_image_polynomial(vec![Real::one(), normalized_zero.clone(),]),
            vec![Real::one()]
        );
        let retained = local_image_retained_relation(vec![
            vec![Real::one()],
            vec![Real::zero()],
            vec![normalized_zero.clone()],
        ]);
        assert_eq!(retained.coefficients, vec![vec![Real::one()]]);

        let report = project_algebraic_fiber_polynomial_image(
            &fiber,
            CurveResultantParameter::First,
            &relation(normalized_zero),
            CurveResultantParameter::Second,
            &alpha,
            config,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicFiberPolynomialImageProjectionStatus::Constructed
        );
        assert_eq!(report.image_degree_bound, 1);
        assert_eq!(
            report.coefficients,
            vec![real(-2), Real::zero(), Real::one()]
        );

        let terminal_zero = crate::test_support::terminal_zero();
        assert_eq!(
            normalize_projective_image_polynomial(vec![Real::one(), terminal_zero.clone()]).len(),
            2
        );
        assert_eq!(
            local_image_retained_relation(vec![
                vec![Real::one()],
                vec![Real::zero()],
                vec![terminal_zero.clone()],
            ])
            .coefficients[0]
                .len(),
            3
        );
        for trailing in [positive, terminal_zero] {
            let retained = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &relation(trailing),
                CurveResultantParameter::Second,
                &alpha,
                config,
                PredicatePolicy::STRICT,
            );
            assert_eq!(
                retained.status,
                AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded
            );
            assert_eq!(retained.image_degree_bound, 2);
        }
    }

    #[test]
    fn polynomial_fiber_image_saturates_an_identically_zero_source_conjugate() {
        // F=(u-alpha)(u-1)^2. The image G=(u-1)(z-u) vanishes for every z on
        // the unrelated repeated u=1 source component, so the raw resultant
        // is zero. Saturation must remove the whole primary component, retain
        // it explicitly, and project the residual selected image instead of
        // discarding every z root.
        let fiber = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one(), real(-2), Real::one()],
            vec![real(-1), real(2), real(-1)],
        ]);
        let image = BivariatePolynomial::new(vec![
            vec![Real::zero(), real(-1)],
            vec![Real::one(), Real::one()],
            vec![real(-1)],
        ]);
        let expected_factor =
            BivariatePolynomial::new(vec![vec![Real::one(), real(-2), Real::one()]]);
        let expected_retained = BivariatePolynomial::new(vec![
            vec![real(-2), real(-1)],
            vec![Real::one(), Real::one()],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-2), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let config = AlgebraicFiberPolynomialImageProjectionConfig {
                max_fiber_degree: 3,
                max_retained_degree: 2,
                max_image_degree_bound: 3,
            };
            let retained = project_algebraic_fiber_polynomial_image_relation(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                config,
                policy,
            );
            assert_eq!(
                retained.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(
                retained.identically_zero_fiber_factor,
                Some(expected_factor.clone())
            );
            assert_eq!(retained.retained_relation, Some(expected_retained.clone()));
            assert!(retained.coefficients.is_empty());
            assert_eq!(retained.certainty, Certainty::Exact);

            let global = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                config,
                policy,
            );
            assert_eq!(
                global.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(
                global.identically_zero_fiber_factor,
                Some(expected_factor.clone())
            );
            assert_eq!(global.retained_relation, Some(expected_retained.clone()));
            assert_eq!(
                global.coefficients,
                vec![real(-2), Real::zero(), Real::one()]
            );
            assert_eq!(global.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn polynomial_fiber_image_saturates_a_fourfold_multicoefficient_component() {
        // F=(u-alpha)(u-1)^4 and every z coefficient of G=(u-1)Q shares the
        // unrelated u=1 component. Saturation must remove all four source
        // multiplicities even though the image content contains it once.
        let fiber = BivariatePolynomial::new(vec![
            vec![real(0), real(1), real(-4), real(6), real(-4), real(1)],
            vec![real(-1), real(4), real(-6), real(4), real(-1)],
        ]);
        let image = BivariatePolynomial::new(vec![
            vec![real(-1), real(-1), real(0), real(-1)],
            vec![real(1), real(0), real(-1), real(1)],
            vec![real(-1), real(1), real(1)],
            vec![real(1)],
        ]);
        let expected_factor =
            BivariatePolynomial::new(vec![vec![real(1), real(-4), real(6), real(-4), real(1)]]);
        let irreducible_retained = BivariatePolynomial::new(vec![
            vec![real(-3), real(1), real(2), real(-1)],
            vec![real(3), real(0), real(-1), real(1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (defining, lower, upper, expected_retained) in [
                (
                    vec![real(-2), Real::zero(), Real::one()],
                    Real::one(),
                    real(2),
                    irreducible_retained.clone(),
                ),
                (
                    vec![real(6), real(-2), real(-3), real(1)],
                    rational(7, 5),
                    rational(3, 2),
                    BivariatePolynomial::new(vec![
                        vec![real(-7), real(-1), real(0), real(-1)],
                        vec![real(3), real(0), real(-1), real(1)],
                        vec![real(2), real(1), real(1)],
                    ]),
                ),
            ] {
                let alpha = represented_root(defining, lower, upper, policy);
                let report = project_algebraic_fiber_polynomial_image_relation(
                    &fiber,
                    CurveResultantParameter::First,
                    &image,
                    CurveResultantParameter::Second,
                    &alpha,
                    AlgebraicFiberPolynomialImageProjectionConfig {
                        max_fiber_degree: 5,
                        max_retained_degree: 3,
                        max_image_degree_bound: 15,
                    },
                    policy,
                );
                assert_eq!(
                    report.status,
                    AlgebraicFiberPolynomialImageProjectionStatus::Constructed
                );
                assert_eq!(
                    report.identically_zero_fiber_factor,
                    Some(expected_factor.clone())
                );
                assert_eq!(report.retained_relation, Some(expected_retained));
                assert!(report.coefficients.is_empty());
                assert_eq!(report.certainty, Certainty::Exact);
            }
        }
    }

    #[test]
    fn polynomial_fiber_image_preserves_a_degree_135_selected_tower() {
        let mut retained_polynomial = vec![Real::zero(); 10];
        retained_polynomial[0] = -rational(1, 2);
        retained_polynomial[9] = Real::one();
        let mut fiber_row = vec![Real::zero(); 16];
        fiber_row[15] = real(32_768);
        let fiber = BivariatePolynomial::new(vec![fiber_row, vec![real(-1)]]);
        let image = BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        let mut denominator = real(2);
        for _ in 0..9 {
            denominator *= real(32_768);
        }
        let expected_constant = -(Real::one() / denominator).unwrap();

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                retained_polynomial.clone(),
                rational(9, 10),
                Real::one(),
                policy,
            );
            let report = project_algebraic_fiber_polynomial_image(
                &fiber,
                CurveResultantParameter::First,
                &image,
                CurveResultantParameter::Second,
                &alpha,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: 15,
                    max_retained_degree: 9,
                    max_image_degree_bound: 15,
                },
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(report.fiber_degree, 15);
            assert_eq!(report.image_degree_bound, 15);
            assert_eq!(report.coefficients.len(), 136);
            assert_eq!(report.coefficients[0], expected_constant);
            assert!(
                report.coefficients[1..135]
                    .iter()
                    .all(|coefficient| coefficient == &Real::zero())
            );
            assert_eq!(report.coefficients[135], Real::one());
            assert_eq!(report.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn quotient_ring_fiber_projection_retains_the_complete_norm_roots() {
        // For 2 alpha^2 - 1 = 0 and F(alpha, y) = y - alpha, the norm is a
        // nonzero scalar multiple of 2 y^2 - 1.
        let polynomial = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(2)],
                rational(2, 3),
                rational(3, 4),
                policy,
            );
            let report = project_bivariate_fiber_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                8,
            );
            assert_eq!(report.status, AlgebraicFiberProjectionStatus::Constructed);
            assert_eq!(report.coefficients.len(), 3);
            assert_ne!(report.coefficients[0], Real::zero());
            assert_eq!(report.coefficients[1], Real::zero());
            assert_eq!(
                real(2) * report.coefficients[0].clone() + report.coefficients[2].clone(),
                Real::zero()
            );
        }
    }

    #[test]
    fn quotient_ring_fiber_projection_shares_local_field_admission_in_both_orientations() {
        let retained_first =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        let retained_second =
            BivariatePolynomial::new(vec![vec![Real::zero(), real(-1)], vec![Real::one()]]);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for defining_polynomial in [
                vec![real(-1), Real::zero(), real(2)],
                vec![real(2), real(-1), real(-4), real(2)],
            ] {
                let retained_root =
                    represented_root(defining_polynomial, rational(2, 3), rational(3, 4), policy);
                let first = project_bivariate_fiber_at_algebraic_parameter(
                    &retained_first,
                    CurveResultantParameter::First,
                    &retained_root,
                    8,
                );
                let second = project_bivariate_fiber_at_algebraic_parameter(
                    &retained_second,
                    CurveResultantParameter::Second,
                    &retained_root,
                    8,
                );
                assert_eq!(first.status, AlgebraicFiberProjectionStatus::Constructed);
                assert_eq!(second, first);
            }
        }

        let valid = represented_root(
            vec![real(-1), Real::zero(), real(2)],
            rational(2, 3),
            rational(3, 4),
            PredicatePolicy::STRICT,
        );
        let mut invalid_validation = valid.clone();
        invalid_validation.validation.status = AlgebraicRootValidationStatus::InvalidPolynomial;
        let mut nonunit = valid.clone();
        nonunit.interval.distinct_root_count = 2;
        let mut constant = valid;
        constant.polynomial_coefficients = vec![Real::one()];
        for invalid in [invalid_validation, nonunit, constant] {
            assert!(!is_valid_local_algebraic_field_evidence(&invalid));
            assert!(LocalAlgebraicField::new(&invalid, PredicatePolicy::STRICT).is_err());
            let report = project_bivariate_fiber_at_algebraic_parameter(
                &retained_first,
                CurveResultantParameter::First,
                &invalid,
                8,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberProjectionStatus::InvalidEvidence
            );
            assert!(report.coefficients.is_empty());
        }
    }

    #[test]
    fn quotient_ring_fiber_projection_obeys_an_explicit_degree_budget() {
        // For 2 alpha^10 - 1 = 0 and F(alpha, y) = y - alpha, the norm is a
        // nonzero scalar multiple of 2 y^10 - 1. Smaller explicit budgets must
        // reject the field before constructing the exact 2^10-state determinant.
        let polynomial = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let mut defining_polynomial = vec![Real::zero(); 11];
        defining_polynomial[0] = real(-1);
        defining_polynomial[10] = real(2);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                defining_polynomial.clone(),
                rational(9, 10),
                Real::one(),
                policy,
            );
            for max_degree in [0, 1, 8, 9] {
                let bounded = project_bivariate_fiber_at_algebraic_parameter(
                    &polynomial,
                    CurveResultantParameter::First,
                    &alpha,
                    max_degree,
                );
                assert_eq!(bounded.status, AlgebraicFiberProjectionStatus::Undecided);
                assert!(bounded.coefficients.is_empty());
            }

            let projected = project_bivariate_fiber_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                10,
            );
            assert_eq!(
                projected.status,
                AlgebraicFiberProjectionStatus::Constructed
            );
            assert_eq!(projected.coefficients.len(), 11);
            assert!(
                projected.coefficients[1..10]
                    .iter()
                    .all(|coefficient| coefficient == &Real::zero())
            );
            assert_eq!(
                real(2) * projected.coefficients[0].clone() + projected.coefficients[10].clone(),
                Real::zero()
            );
        }
    }

    #[test]
    fn selected_algebraic_fiber_isolation_uses_division_free_bernstein_path() {
        // For alpha=sqrt(1/2), isolate (y-alpha)(y-1/2)^2. The rational
        // midpoint is deflated exactly and the remaining selected-field root
        // stays algebraic without constructing a primitive element.
        let polynomial = BivariatePolynomial::new(vec![
            vec![Real::zero(), rational(1, 4), real(-1), Real::one()],
            vec![rational(-1, 4), Real::one(), real(-1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), Real::zero(), real(2)],
                rational(2, 3),
                rational(3, 4),
                policy,
            );
            let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                &Real::zero(),
                &Real::one(),
                AlgebraicFiberRootIsolationConfig {
                    max_subdivision_depth: 64,
                    refinement_steps: 4,
                },
                policy,
            );
            assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
            assert_eq!(report.intervals.len(), 2);
            assert_eq!(report.intervals[0].exact_root, Some(rational(1, 2)));
            assert!(report.intervals[1].exact_root.is_none());
            assert_eq!(report.intervals[1].distinct_root_count, 1);
            assert_eq!(report.sturm_sequence_length, 0);
            assert!(report.subdivision_steps > 0);
        }
    }

    #[test]
    fn selected_algebraic_fiber_isolation_falls_back_for_repeated_irrational_root() {
        // At alpha=sqrt(1/2), y^2-2 alpha y+1/2=(y-alpha)^2. Its
        // Bernstein variation remains two around the irrational double root,
        // so a bounded Bernstein attempt must retain the exact Sturm fallback.
        let polynomial = BivariatePolynomial::new(vec![
            vec![rational(1, 2), Real::zero(), Real::one()],
            vec![Real::zero(), real(-2)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), Real::zero(), real(2)],
                rational(2, 3),
                rational(3, 4),
                policy,
            );
            let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                &Real::zero(),
                &Real::one(),
                AlgebraicFiberRootIsolationConfig {
                    max_subdivision_depth: 4,
                    refinement_steps: 0,
                },
                policy,
            );
            assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
            assert_eq!(report.intervals.len(), 1);
            assert!(report.intervals[0].exact_root.is_none());
            assert_eq!(report.intervals[0].distinct_root_count, 1);
            assert!(report.sturm_sequence_length > 1);
        }
    }

    #[test]
    fn selected_fiber_fallback_partitions_at_a_deflated_rational_root() {
        // At alpha=sqrt(1/2), this is (y-1/2)(y-alpha)^2. Bernstein discovers
        // the rational root but cannot finish the irrational double root, so
        // the Sturm restart must use 1/2 as a boundary. No isolator for alpha
        // may straddle the other root after that root has been deflated.
        let polynomial = BivariatePolynomial::new(vec![
            vec![
                rational(-1, 4),
                rational(1, 2),
                rational(-1, 2),
                Real::one(),
            ],
            vec![Real::zero(), Real::one(), real(-2)],
        ]);
        let half = rational(1, 2);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), Real::zero(), real(2)],
                rational(2, 3),
                rational(3, 4),
                policy,
            );
            let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                &Real::zero(),
                &Real::one(),
                AlgebraicFiberRootIsolationConfig {
                    max_subdivision_depth: 4,
                    refinement_steps: 0,
                },
                policy,
            );
            assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
            assert_eq!(report.intervals.len(), 2);
            assert!(
                report
                    .intervals
                    .iter()
                    .any(|interval| interval.exact_root.as_ref() == Some(&half))
            );
            let algebraic = report
                .intervals
                .iter()
                .find(|interval| interval.exact_root.is_none())
                .expect("irrational double-root isolator");
            assert!(algebraic.lower >= half);
            assert!(report.sturm_sequence_length > 1);
            assert!(report.subdivision_steps > 1);
        }
    }

    #[test]
    fn selected_fiber_repeated_and_rational_roots_complete() {
        // At alpha=sqrt(1/2), combine its repeated root with rational roots
        // 1/4, 1/2, and 3/4. The Sturm coefficients include small nonzero
        // values which exceed the hot local-field signing schedule. Their
        // exact nonzero certificates must permit refinement to completion.
        let q = vec![
            rational(-3, 32),
            rational(11, 16),
            rational(-3, 2),
            Real::one(),
        ];
        let retained_first_coefficients = vec![
            {
                let mut row = vec![Real::zero(), Real::zero()];
                row.extend(q.iter().cloned());
                row
            },
            {
                let mut row = vec![Real::zero()];
                row.extend(q.iter().map(|coefficient| coefficient * real(-2)));
                row
            },
            q,
        ];
        let retained_second_coefficients = (0..retained_first_coefficients[0].len())
            .map(|power| {
                retained_first_coefficients
                    .iter()
                    .map(|row| row.get(power).cloned().unwrap_or_else(Real::zero))
                    .collect()
            })
            .collect();
        let retained_first = BivariatePolynomial::new(retained_first_coefficients);
        let retained_second = BivariatePolynomial::new(retained_second_coefficients);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for defining_polynomial in [
                vec![real(-1), Real::zero(), real(2)],
                vec![real(2), real(-1), real(-4), real(2)],
            ] {
                let alpha =
                    represented_root(defining_polynomial, rational(2, 3), rational(3, 4), policy);
                for (polynomial, retained_parameter) in [
                    (&retained_first, CurveResultantParameter::First),
                    (&retained_second, CurveResultantParameter::Second),
                ] {
                    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                        polynomial,
                        retained_parameter,
                        &alpha,
                        &Real::zero(),
                        &Real::one(),
                        AlgebraicFiberRootIsolationConfig {
                            max_subdivision_depth: 16,
                            refinement_steps: 0,
                        },
                        policy,
                    );
                    assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
                    assert_eq!(report.intervals.len(), 4);
                    for root in &report.intervals {
                        if let Some(value) = &root.exact_root {
                            assert_eq!(&root.lower, value);
                            assert_eq!(&root.upper, value);
                            assert!(
                                [rational(1, 4), rational(1, 2), rational(3, 4)].contains(value)
                            );
                            continue;
                        }
                        let replay = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
                            polynomial,
                            retained_parameter,
                            &alpha,
                            &root.lower,
                            &root.upper,
                            policy,
                        );
                        assert_eq!(replay.status, AlgebraicFiberRootCountStatus::Counted);
                        assert_eq!(replay.distinct_root_count, Some(1));
                    }
                    for value in [rational(1, 4), rational(1, 2), rational(3, 4)] {
                        assert!(report.intervals.iter().any(|root| {
                            root.exact_root.as_ref() == Some(&value)
                                || root.lower < value && value < root.upper
                        }));
                    }
                    let irrational = &report.intervals[2];
                    assert!(irrational.lower >= rational(1, 2));
                    assert!(irrational.upper <= rational(3, 4));
                    assert_eq!(irrational.distinct_root_count, 1);
                    assert!(report.sturm_sequence_length > 1);
                    assert!(report.subdivision_steps > 0);
                    assert!(report.retained_refinement_steps > 0);
                    assert_eq!(report.certainty, Certainty::Exact);
                }
            }
        }
    }

    #[test]
    fn local_field_sturm_counts_even_multiplicity_in_both_orientations() {
        // alpha = cbrt(1/2), beta = alpha^2 = cbrt(1/4).
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![],
            vec![real(0), real(-2)],
            vec![],
            vec![real(1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(0), real(2)],
                rational(3, 4),
                rational(4, 5),
                policy,
            );
            let beta = represented_root(
                vec![real(-1), real(0), real(0), real(4)],
                rational(3, 5),
                rational(2, 3),
                policy,
            );

            let retained_first = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &beta.interval.lower,
                &beta.interval.upper,
                policy,
            );
            assert_eq!(
                retained_first.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(retained_first.distinct_root_count, Some(1));
            assert_eq!(retained_first.certainty, Certainty::Exact);

            let retained_second = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::Second,
                &beta,
                &alpha.interval.lower,
                &alpha.interval.upper,
                policy,
            );
            assert_eq!(
                retained_second.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(retained_second.distinct_root_count, Some(1));
            assert_eq!(retained_second.certainty, Certainty::Exact);

            let empty_interval = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &real(0),
                &rational(1, 2),
                policy,
            );
            assert_eq!(
                empty_interval.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(empty_interval.distinct_root_count, Some(0));
        }
    }

    #[test]
    fn local_field_sturm_batches_candidate_intervals_without_changing_reports() {
        // At alpha = cbrt(1/2), this relation specializes to
        // (beta-alpha^2)^2. The sole distinct beta root is cbrt(1/4).
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![],
            vec![real(0), real(-2)],
            vec![],
            vec![real(1)],
        ]);
        let zero = real(0);
        let half = rational(1, 2);
        let three_fifths = rational(3, 5);
        let two_thirds = rational(2, 3);
        let one = real(1);
        let reversed_lower = rational(4, 5);
        let reversed_upper = rational(3, 4);
        let intervals = [
            (&zero, &half),
            (&three_fifths, &two_thirds),
            (&two_thirds, &one),
            (&reversed_lower, &reversed_upper),
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(0), real(2)],
                rational(3, 4),
                rational(4, 5),
                policy,
            );
            let batched = count_bivariate_fiber_roots_at_algebraic_parameter_intervals(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &intervals,
                policy,
            );
            assert_eq!(batched.len(), intervals.len());
            for ((lower, upper), batch_report) in intervals.iter().zip(&batched) {
                let independent = count_bivariate_fiber_roots_at_algebraic_parameter(
                    &relation,
                    CurveResultantParameter::First,
                    &alpha,
                    lower,
                    upper,
                    policy,
                );
                assert_eq!(batch_report.status, independent.status);
                assert_eq!(
                    batch_report.distinct_root_count,
                    independent.distinct_root_count
                );
                if batch_report.status != AlgebraicFiberRootCountStatus::InvalidInterval {
                    assert_eq!(
                        batch_report.sturm_sequence_length,
                        independent.sturm_sequence_length
                    );
                }
                assert_eq!(batch_report.certainty, independent.certainty);
            }
            assert_eq!(batched[0].distinct_root_count, Some(0));
            assert_eq!(batched[1].distinct_root_count, Some(1));
            assert_eq!(batched[2].distinct_root_count, Some(0));
            assert_eq!(
                batched[3].status,
                AlgebraicFiberRootCountStatus::InvalidInterval
            );
        }
    }

    #[test]
    fn local_field_sturm_batches_preserve_endpoint_reports_in_both_orientations() {
        let retained_first = BivariatePolynomial::new(vec![vec![real(1), real(-2), real(1)]]);
        let retained_second =
            BivariatePolynomial::new(vec![vec![real(1)], vec![real(-2)], vec![real(1)]]);
        let zero = real(0);
        let one = real(1);
        let two = real(2);
        let three = real(3);
        let intervals = [(&one, &two), (&one, &three), (&zero, &one)];

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for defining_polynomial in [
                vec![real(-2), real(0), real(1)],
                vec![real(6), real(-2), real(-3), real(1)],
            ] {
                let retained_root = represented_root(defining_polynomial, real(1), real(2), policy);
                for (relation, retained_parameter) in [
                    (&retained_first, CurveResultantParameter::First),
                    (&retained_second, CurveResultantParameter::Second),
                ] {
                    let batched = count_bivariate_fiber_roots_at_algebraic_parameter_intervals(
                        relation,
                        retained_parameter,
                        &retained_root,
                        &intervals,
                        policy,
                    );
                    for ((lower, upper), batch_report) in intervals.iter().zip(&batched) {
                        let independent = count_bivariate_fiber_roots_at_algebraic_parameter(
                            relation,
                            retained_parameter,
                            &retained_root,
                            lower,
                            upper,
                            policy,
                        );
                        assert_eq!(batch_report, &independent);
                        assert_eq!(batch_report.certainty, Certainty::Exact);
                    }
                    assert!(batched.iter().all(|report| {
                        report.status == AlgebraicFiberRootCountStatus::EndpointRoot
                            && report.distinct_root_count.is_none()
                    }));
                }
            }
        }
    }

    #[test]
    fn local_field_sturm_reports_an_identically_zero_specialization() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-1), real(0), real(2)],
            rational(2, 3),
            rational(3, 4),
            policy,
        );
        let defining = BivariatePolynomial::new(vec![vec![real(-1)], vec![real(0)], vec![real(2)]]);
        let report = count_bivariate_fiber_roots_at_algebraic_parameter(
            &defining,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            report.status,
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
        );
        assert_eq!(report.distinct_root_count, None);
    }

    #[test]
    fn local_field_sturm_handles_a_reducible_defining_polynomial() {
        // (a^2 - 2)(a - 3) defines sqrt(2) locally on (7/5, 3/2).
        // The quotient by the whole cubic is not a field, so this exercises
        // denominator signing and exact common-root zero detection at the
        // selected factor rather than assuming global irreducibility.
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![real(0), real(-2)],
            vec![real(1)],
        ]);
        let repeated_two_root_fiber = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(0), real(0), real(1)],
            vec![real(0), real(0), real(-2)],
            vec![real(1)],
        ]);
        let common_first =
            BivariatePolynomial::new(vec![vec![real(0), real(0), real(1)], vec![real(-1)]]);
        let common_second = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1), real(1)],
            vec![real(-1), real(-1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(6), real(-2), real(-3), real(1)],
                rational(7, 5),
                rational(3, 2),
                policy,
            );
            for retained_parameter in [
                CurveResultantParameter::First,
                CurveResultantParameter::Second,
            ] {
                let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                    &relation,
                    retained_parameter,
                    &alpha,
                    &real(1),
                    &real(2),
                    policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                assert_eq!(report.distinct_root_count, Some(1));
                assert_eq!(report.certainty, Certainty::Exact);
            }

            let two_roots = count_bivariate_fiber_roots_at_algebraic_parameter(
                &repeated_two_root_fiber,
                CurveResultantParameter::First,
                &alpha,
                &real(-2),
                &real(2),
                policy,
            );
            assert_eq!(two_roots.status, AlgebraicFiberRootCountStatus::Counted);
            assert_eq!(two_roots.distinct_root_count, Some(2));
            assert_eq!(two_roots.certainty, Certainty::Exact);

            let common = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &common_first,
                &common_second,
                CurveResultantParameter::First,
                &alpha,
                &real(1),
                &rational(5, 4),
                policy,
            );
            assert_eq!(common.status, AlgebraicFiberRootCountStatus::Counted);
            assert_eq!(common.distinct_root_count, Some(1));
            assert_eq!(common.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn local_field_gcd_handles_specialized_degree_drops_in_both_orientations() {
        // A=2*a^3-1 and B=4*b^3-1 vanish at alpha=cuberoot(1/2)
        // and beta=alpha^2=cuberoot(1/4). Both equations have generic degree
        // three in either fiber, but their A*B leading terms disappear at the
        // selected retained root. The specialized GCD is b-a^2 (or b-a^2 as
        // a polynomial in a), so generic-degree subresultants are unnecessary.
        let first = BivariatePolynomial::new(vec![
            vec![real(1), real(1), real(0), real(-4)],
            vec![],
            vec![real(-1)],
            vec![real(-2), real(0), real(0), real(8)],
        ]);
        let second = BivariatePolynomial::new(vec![
            vec![real(1), real(2), real(0), real(-4)],
            vec![],
            vec![real(-2)],
            vec![real(-2), real(0), real(0), real(8)],
        ]);
        let rootless_after_drop = BivariatePolynomial::new(vec![
            vec![real(1), real(0), real(-1)],
            vec![],
            vec![],
            vec![real(0), real(0), real(2)],
        ]);
        let beta_defining =
            BivariatePolynomial::new(vec![vec![real(-1), real(0), real(0), real(4)]]);
        let alpha_defining =
            BivariatePolynomial::new(vec![vec![real(-1)], vec![], vec![], vec![real(2)]]);
        let alpha_relation =
            BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![], vec![real(-1)]]);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(0), real(2)],
                rational(3, 4),
                rational(4, 5),
                policy,
            );
            let beta = represented_root(
                vec![real(-1), real(0), real(0), real(4)],
                rational(3, 5),
                rational(2, 3),
                policy,
            );
            for (retained_parameter, retained_root, lower, upper) in [
                (
                    CurveResultantParameter::First,
                    &alpha,
                    &beta.interval.lower,
                    &beta.interval.upper,
                ),
                (
                    CurveResultantParameter::Second,
                    &beta,
                    &alpha.interval.lower,
                    &alpha.interval.upper,
                ),
            ] {
                let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                    &first,
                    &second,
                    retained_parameter,
                    retained_root,
                    lower,
                    upper,
                    policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                assert_eq!(report.distinct_root_count, Some(1));
                assert_eq!(report.certainty, Certainty::Exact);
            }

            let rootless = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &rootless_after_drop,
                &beta_defining,
                CurveResultantParameter::First,
                &alpha,
                &beta.interval.lower,
                &beta.interval.upper,
                policy,
            );
            assert_eq!(rootless.status, AlgebraicFiberRootCountStatus::Counted);
            assert_eq!(rootless.distinct_root_count, Some(0));

            let one_zero_equation = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &alpha_defining,
                &alpha_relation,
                CurveResultantParameter::First,
                &alpha,
                &beta.interval.lower,
                &beta.interval.upper,
                policy,
            );
            assert_eq!(
                one_zero_equation.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(one_zero_equation.distinct_root_count, Some(1));

            let both_zero = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &alpha_defining,
                &alpha_defining,
                CurveResultantParameter::First,
                &alpha,
                &beta.interval.lower,
                &beta.interval.upper,
                policy,
            );
            assert_eq!(
                both_zero.status,
                AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
            );
        }
    }

    #[test]
    fn local_field_sturm_reports_rational_endpoint_roots_and_input_boundaries() {
        let policy = PredicatePolicy::STRICT;
        let half = rational(1, 2);
        let alpha = represented_exact_root(half.clone(), policy);
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![real(0), real(-2)],
            vec![real(1)],
        ]);

        let counted = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(counted.status, AlgebraicFiberRootCountStatus::Counted);
        assert_eq!(counted.distinct_root_count, Some(1));

        let endpoint = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &half,
            &real(1),
            policy,
        );
        assert_eq!(endpoint.status, AlgebraicFiberRootCountStatus::EndpointRoot);
        let repeated_endpoint_owned = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &half,
            &real(1),
            policy,
        );
        assert_eq!(
            repeated_endpoint_owned.status,
            AlgebraicFiberRootCountStatus::Counted
        );
        assert_eq!(repeated_endpoint_owned.distinct_root_count, Some(1));

        // b(b-a)(b-1) has three distinct roots at a=1/2. The closed
        // contract owns rational roots at either bound and counts each once.
        let endpoint_owned_relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(-1), real(1)],
            vec![real(0), real(1), real(-1)],
        ]);
        let lower_closed = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            &endpoint_owned_relation,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &rational(3, 4),
            policy,
        );
        assert_eq!(lower_closed.status, AlgebraicFiberRootCountStatus::Counted);
        assert_eq!(lower_closed.distinct_root_count, Some(2));
        let both_closed = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
            &endpoint_owned_relation,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(both_closed.status, AlgebraicFiberRootCountStatus::Counted);
        assert_eq!(both_closed.distinct_root_count, Some(3));

        let reversed = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &real(1),
            &real(0),
            policy,
        );
        assert_eq!(
            reversed.status,
            AlgebraicFiberRootCountStatus::InvalidInterval
        );

        let real_coefficient = count_bivariate_fiber_roots_at_algebraic_parameter(
            &BivariatePolynomial::new(vec![vec![Real::pi(), real(1)]]),
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            real_coefficient.status,
            AlgebraicFiberRootCountStatus::Counted
        );
        assert_eq!(real_coefficient.distinct_root_count, Some(0));

        let mut nonunit = alpha;
        nonunit.interval.distinct_root_count = 2;
        let invalid = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &nonunit,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            invalid.status,
            AlgebraicFiberRootCountStatus::InvalidEvidence
        );
    }

    #[test]
    fn selected_fiber_accepts_exact_real_base_coefficients() {
        // Retain alpha=sqrt(pi) without flattening it into a primitive
        // approximation. Both the defining modulus and an independent fiber
        // coefficient live over exact `Real`; y-alpha still has one selected
        // root in (1,2).
        let relation =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![real(-1)]]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![-Real::pi(), Real::zero(), Real::one()],
                Real::one(),
                real(2),
                policy,
            );
            let projection = project_bivariate_fiber_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                8,
            );
            assert_eq!(
                projection.status,
                AlgebraicFiberProjectionStatus::Constructed
            );
            assert_eq!(projection.coefficients.len(), 3);
            assert_eq!(projection.coefficients[1], Real::zero());
            assert_eq!(
                projection.coefficients[0].clone()
                    + Real::pi() * projection.coefficients[2].clone(),
                Real::zero()
            );

            let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &Real::one(),
                &real(2),
                AlgebraicFiberRootIsolationConfig::default(),
                policy,
            );
            assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
            assert_eq!(report.intervals.len(), 1);
        }
    }
}
