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

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::{Real, ZeroKnowledge};

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootPolynomialEvaluationReport,
    AlgebraicRootPolynomialEvaluationStatus, AlgebraicRootRepresentation,
    evaluate_polynomial_at_algebraic_root, validate_algebraic_root_representation,
};
use crate::curve_resultant::{BivariatePolynomial, CurveResultantParameter};
use crate::resultant::quotient_ring_fiber_resultant_polynomial;
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig, polynomial_div_rem,
    polynomials_share_one_root_in_interval, refine_isolated_univariate_polynomial_interval,
};

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
    /// Exact-rational coefficient arithmetic is required by this local-field
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
    /// Exact-rational coefficients and endpoints are required.
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
/// polynomial and endpoint values must be exact rationals. Coefficients of the
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
/// All polynomial coefficients and interval endpoints must be exact
/// rationals. The intervals need not be disjoint or sorted.
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
    if polynomial
        .coefficients
        .iter()
        .flatten()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return fiber_intervals
            .iter()
            .map(|_| {
                fiber_root_count_report(
                    AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
                    None,
                    0,
                    0,
                    Certainty::Exact,
                    Some("fiber root counting requires exact-rational coefficients and endpoints"),
                )
            })
            .collect();
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
            if fiber_lower.exact_rational_ref().is_none()
                || fiber_upper.exact_rational_ref().is_none()
            {
                return fiber_root_count_report(
                    AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
                    None,
                    kernel.sequence_length(),
                    field.refinement_steps,
                    field.certainty,
                    Some("fiber root counting requires exact-rational coefficients and endpoints"),
                );
            }
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
                    let lower = cached_local_sturm_boundary_variations(
                        &mut sturm_boundary_cache,
                        sequence,
                        fiber_lower,
                        &mut field,
                    );
                    let upper = cached_local_sturm_boundary_variations(
                        &mut sturm_boundary_cache,
                        sequence,
                        fiber_upper,
                        &mut field,
                    );
                    match (lower, upper) {
                        (Ok(Some(lower)), Ok(Some(upper))) => lower
                            .checked_sub(upper)
                            .map(|count| LocalRootCountOutcome::Counted {
                                count,
                                sequence_length: sequence.len(),
                            })
                            .ok_or(LocalFieldError::Undecided),
                        (Ok(None), _) | (_, Ok(None)) => Ok(LocalRootCountOutcome::EndpointRoot {
                            sequence_length: sequence.len(),
                        }),
                        (Err(error), _) | (_, Err(error)) => Err(error),
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
    if polynomial
        .coefficients
        .iter()
        .flatten()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
        || fiber_lower.exact_rational_ref().is_none()
        || fiber_upper.exact_rational_ref().is_none()
    {
        return error_report(
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient,
            Certainty::Exact,
            "fiber root isolation requires exact-rational coefficients and endpoints",
        );
    }
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
        Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
    }
    let mut fiber = match local_fiber_polynomial(polynomial, retained_parameter, &mut field) {
        Ok(fiber) => fiber,
        Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
    };
    if match local_polynomial_is_zero(&fiber, &mut field) {
        Ok(is_zero) => is_zero,
        Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
        Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
            Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
        let sequence = match local_sturm_sequence(fiber.clone(), &mut field) {
            Ok(sequence) => sequence,
            Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
            Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
                    return fiber_root_isolation_error_report(
                        LocalFieldError::Undecided,
                        field.certainty,
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
            for _ in 0..target_refinements {
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
                        return fiber_root_isolation_error_report(
                            LocalFieldError::Undecided,
                            field.certainty,
                        );
                    }
                };
                subdivision_steps = subdivision_steps.saturating_add(1);
                let midpoint_sign = match local_polynomial_sign_at(&fiber, &midpoint, &mut field) {
                    Ok(sign) => sign,
                    Err(error) => {
                        return fiber_root_isolation_error_report(error, field.certainty);
                    }
                };
                if midpoint_sign == Ordering::Equal {
                    rational_root = Some(midpoint);
                    break;
                }
                let midpoint_variations = match boundary_variations(&midpoint, &mut field) {
                    Ok(variations) => variations,
                    Err(error) => {
                        return fiber_root_isolation_error_report(error, field.certainty);
                    }
                };
                let left_count = match node.lower_variations.checked_sub(midpoint_variations) {
                    Some(count) => count,
                    None => {
                        return fiber_root_isolation_error_report(
                            LocalFieldError::Undecided,
                            field.certainty,
                        );
                    }
                };
                let right_count = match midpoint_variations.checked_sub(node.upper_variations) {
                    Some(count) => count,
                    None => {
                        return fiber_root_isolation_error_report(
                            LocalFieldError::Undecided,
                            field.certainty,
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
                    return fiber_root_isolation_error_report(
                        LocalFieldError::Undecided,
                        field.certainty,
                    );
                }
                node.depth += 1;
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
                    return fiber_root_isolation_error_report(
                        LocalFieldError::Undecided,
                        field.certainty,
                    );
                }
            };
            subdivision_steps = subdivision_steps.saturating_add(1);
            let midpoint_sign = match local_polynomial_sign_at(&fiber, &midpoint, &mut field) {
                Ok(sign) => sign,
                Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
            };
            if midpoint_sign == Ordering::Equal {
                rational_root = Some(midpoint);
                break;
            }
            let midpoint_variations = match boundary_variations(&midpoint, &mut field) {
                Ok(variations) => variations,
                Err(error) => return fiber_root_isolation_error_report(error, field.certainty),
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
        match deflate_local_polynomial_at_rational_root(fiber, &root, &mut field) {
            Ok((deflated, true)) => fiber = deflated,
            Ok((_, false)) | Err(_) => {
                return fiber_root_isolation_error_report(
                    LocalFieldError::Undecided,
                    field.certainty,
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
/// subdivision exhausts the caller's depth budget, or exact variation
/// bookkeeping does not identify a unique child. Repeated irrational roots
/// are the normal depth-limit case.
fn isolate_local_polynomial_roots_bernstein(
    mut polynomial: Vec<LocalFieldElement>,
    fiber_lower: &Real,
    fiber_upper: &Real,
    config: AlgebraicFiberRootIsolationConfig,
    field: &mut LocalAlgebraicField,
) -> Result<(Option<Vec<IsolatedRootInterval>>, usize), LocalFieldError> {
    #[derive(Clone)]
    struct Node {
        lower: Real,
        upper: Real,
        controls: Vec<LocalFieldElement>,
        depth: usize,
    }

    let mut exact_roots = Vec::new();
    for endpoint in [fiber_lower, fiber_upper] {
        let (deflated, had_root) =
            deflate_local_polynomial_at_rational_root(polynomial, endpoint, field)?;
        polynomial = deflated;
        if had_root {
            exact_roots.push(endpoint.clone());
        }
    }

    let mut subdivision_steps = 0_usize;
    loop {
        if polynomial.len() == 1 {
            exact_roots.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
            exact_roots.dedup();
            let intervals = exact_roots
                .into_iter()
                .map(|root| IsolatedRootInterval {
                    lower: root.clone(),
                    upper: root.clone(),
                    exact_root: Some(root),
                    distinct_root_count: 1,
                })
                .collect();
            return Ok((Some(intervals), subdivision_steps));
        }

        // A discovered rational root is a boundary for subsequent isolation.
        // Although it has been deflated from `polynomial`, allowing another
        // interval to straddle it would not isolate a root of the original
        // fiber.
        exact_roots.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
        exact_roots.dedup();
        let mut boundaries = Vec::with_capacity(exact_roots.len() + 2);
        boundaries.push(fiber_lower.clone());
        for root in &exact_roots {
            if field.compare(root, fiber_lower)? == Ordering::Greater
                && field.compare(root, fiber_upper)? == Ordering::Less
            {
                boundaries.push(root.clone());
            }
        }
        boundaries.push(fiber_upper.clone());

        let mut stack = Vec::with_capacity(boundaries.len());
        for segment in boundaries.windows(2).rev() {
            let Some(controls) =
                local_power_to_bernstein_on_interval(&polynomial, &segment[0], &segment[1], field)?
            else {
                return Ok((None, subdivision_steps));
            };
            stack.push(Node {
                lower: segment[0].clone(),
                upper: segment[1].clone(),
                controls,
                depth: 0,
            });
        }

        let mut isolated = Vec::new();
        let mut rational_root = None;
        while let Some(mut node) = stack.pop() {
            let variations = local_bernstein_sign_variations(&node.controls, field)?;
            if variations == 0 {
                continue;
            }
            if variations == 1 {
                let lower_sign = node
                    .controls
                    .first()
                    .ok_or(LocalFieldError::Undecided)?
                    .sign(field)?;
                for _ in 0..config.refinement_steps {
                    if node.depth >= config.max_subdivision_depth {
                        return Ok((None, subdivision_steps));
                    }
                    let midpoint = ((&node.lower + &node.upper) / Real::from(2_u8))
                        .map_err(|_| LocalFieldError::Undecided)?;
                    subdivision_steps = subdivision_steps.saturating_add(1);
                    let midpoint_sign = local_polynomial_sign_at(&polynomial, &midpoint, field)?;
                    if midpoint_sign == Ordering::Equal {
                        rational_root = Some(midpoint);
                        break;
                    }
                    if midpoint_sign == lower_sign {
                        node.lower = midpoint;
                    } else {
                        node.upper = midpoint;
                    }
                    node.depth += 1;
                }
                if rational_root.is_some() {
                    break;
                }
                isolated.push(IsolatedRootInterval {
                    lower: node.lower,
                    upper: node.upper,
                    exact_root: None,
                    distinct_root_count: 1,
                });
                continue;
            }
            if node.depth >= config.max_subdivision_depth {
                return Ok((None, subdivision_steps));
            }
            let midpoint = ((&node.lower + &node.upper) / Real::from(2_u8))
                .map_err(|_| LocalFieldError::Undecided)?;
            let (left, right) = subdivide_local_bernstein_half(&node.controls, field)?;
            subdivision_steps = subdivision_steps.saturating_add(1);
            if left
                .last()
                .ok_or(LocalFieldError::Undecided)?
                .is_zero(field)?
            {
                rational_root = Some(midpoint);
                break;
            }
            let left_variations = local_bernstein_sign_variations(&left, field)?;
            let right_variations = local_bernstein_sign_variations(&right, field)?;
            let next_depth = node.depth + 1;
            if right_variations != 0 {
                stack.push(Node {
                    lower: midpoint.clone(),
                    upper: node.upper,
                    controls: right,
                    depth: next_depth,
                });
            }
            if left_variations != 0 {
                stack.push(Node {
                    lower: node.lower,
                    upper: midpoint,
                    controls: left,
                    depth: next_depth,
                });
            }
        }

        if let Some(root) = rational_root {
            let (deflated, had_root) =
                deflate_local_polynomial_at_rational_root(polynomial, &root, field)?;
            if !had_root {
                return Err(LocalFieldError::Undecided);
            }
            polynomial = deflated;
            if !exact_roots.contains(&root) {
                exact_roots.push(root);
            }
            continue;
        }

        isolated.extend(exact_roots.into_iter().map(|root| IsolatedRootInterval {
            lower: root.clone(),
            upper: root.clone(),
            exact_root: Some(root),
            distinct_root_count: 1,
        }));
        isolated.sort_by(|first, second| {
            first
                .lower
                .partial_cmp(&second.lower)
                .unwrap_or(Ordering::Equal)
        });
        return Ok((Some(isolated), subdivision_steps));
    }
}

fn local_power_to_bernstein_on_interval(
    polynomial: &[LocalFieldElement],
    lower: &Real,
    upper: &Real,
    field: &LocalAlgebraicField,
) -> Result<Option<Vec<LocalFieldElement>>, LocalFieldError> {
    let Some(leading) = polynomial.last() else {
        return Err(LocalFieldError::Undecided);
    };
    let degree = polynomial.len().saturating_sub(1);
    let width = upper - lower;
    // Compose by `lower + width*x` using Horner form. This has no binomial
    // size ceiling and keeps the local-field path division-free.
    let mut shifted_power = vec![leading.clone()];
    for coefficient in polynomial[..degree].iter().rev() {
        let old_len = shifted_power.len();
        shifted_power.push(LocalFieldElement::zero());
        for power in (1..=old_len).rev() {
            let same_power = shifted_power[power].clone().scale(lower, field)?;
            let prior_power = shifted_power[power - 1].scale(&width, field)?;
            shifted_power[power] = same_power.add(&prior_power, field)?;
        }
        shifted_power[0] = shifted_power[0]
            .scale(lower, field)?
            .add(coefficient, field)?;
    }

    // x^j = sum_{i=j}^n C(i,j)/C(n,j) B_i^n(x).
    let mut controls = Vec::with_capacity(degree + 1);
    for index in 0..=degree {
        let mut control = LocalFieldElement::zero();
        let mut ratio = Real::one();
        for (power, coefficient) in shifted_power.iter().enumerate().take(index + 1) {
            control = control.add(&coefficient.scale(&ratio, field)?, field)?;
            if power != index {
                let Ok(numerator) = u64::try_from(index - power) else {
                    return Ok(None);
                };
                let Ok(denominator) = u64::try_from(degree - power) else {
                    return Ok(None);
                };
                ratio = ((ratio * Real::from(numerator)) / Real::from(denominator))
                    .map_err(|_| LocalFieldError::Undecided)?;
            }
        }
        controls.push(control);
    }
    Ok(Some(controls))
}

fn subdivide_local_bernstein_half(
    controls: &[LocalFieldElement],
    field: &LocalAlgebraicField,
) -> Result<(Vec<LocalFieldElement>, Vec<LocalFieldElement>), LocalFieldError> {
    let degree = controls
        .len()
        .checked_sub(1)
        .ok_or(LocalFieldError::Undecided)?;
    let half = Real::from(1_u8) / Real::from(2_u8);
    let half = half.map_err(|_| LocalFieldError::Undecided)?;
    let mut work = controls.to_vec();
    let mut left = Vec::with_capacity(controls.len());
    let mut right = Vec::with_capacity(controls.len());
    left.push(work[0].clone());
    right.push(work[degree].clone());
    for level in 1..=degree {
        for index in 0..=degree - level {
            work[index] = work[index]
                .add(&work[index + 1], field)?
                .scale(&half, field)?;
        }
        left.push(work[0].clone());
        right.push(work[degree - level].clone());
    }
    right.reverse();
    Ok((left, right))
}

fn local_bernstein_sign_variations(
    controls: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<usize, LocalFieldError> {
    let mut previous = None;
    let mut variations = 0_usize;
    for control in controls {
        let sign = control.sign(field)?;
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
        let mut quotient = vec![LocalFieldElement::zero(); degree];
        quotient[degree - 1] = fiber[degree].clone();
        for power in (1..degree).rev() {
            let product = match diagonal_root.multiply(&quotient[power], &field) {
                Ok(product) => product,
                Err(error) => return diagonal_deflation_error_report(error, field.certainty),
            };
            quotient[power - 1] = match fiber[power].add(&product, &field) {
                Ok(coefficient) => coefficient,
                Err(error) => return diagonal_deflation_error_report(error, field.certainty),
            };
        }
        let product = match diagonal_root.multiply(&quotient[0], &field) {
            Ok(product) => product,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        };
        let remainder = match fiber[0].add(&product, &field) {
            Ok(remainder) => remainder,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        };
        match remainder.is_zero(&mut field) {
            Ok(true) => {
                fiber = quotient;
                multiplicity += 1;
            }
            Ok(false) => break,
            Err(error) => return diagonal_deflation_error_report(error, field.certainty),
        }
    }

    let reduced_polynomial = match local_fiber_to_bivariate(&fiber, retained_parameter) {
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

/// Projects one bivariate fiber through a low-degree algebraic quotient ring.
///
/// This is the bounded-memory fallback for systems whose generic bivariate
/// resultant has a large interpolation-degree bound. It computes the norm of
/// the specialized fiber directly; its determinant dimension is the retained
/// root's defining-polynomial degree. As with any resultant, roots contributed
/// by other defining-polynomial branches remain candidates that the caller
/// must replay against the selected root.
pub fn project_bivariate_fiber_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicFiberProjectionReport {
    project_bivariate_fiber_at_algebraic_parameter_with_max_degree(
        polynomial,
        retained_parameter,
        retained_root,
        8,
        policy,
    )
}

/// Projects one bivariate fiber with an explicit quotient-ring degree budget.
///
/// The determinant construction is exponential in `max_source_degree`; callers
/// should opt above the default degree eight only for bounded operation-specific
/// fields whose generic resultant would be materially larger.
pub fn project_bivariate_fiber_at_algebraic_parameter_with_max_degree(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    max_source_degree: usize,
    policy: PredicatePolicy,
) -> AlgebraicFiberProjectionReport {
    let field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(LocalFieldError::InvalidEvidence | LocalFieldError::InvalidInterval) => {
            return AlgebraicFiberProjectionReport {
                status: AlgebraicFiberProjectionStatus::InvalidEvidence,
                coefficients: Vec::new(),
            };
        }
        Err(LocalFieldError::UnsupportedCoefficient) => {
            return AlgebraicFiberProjectionReport {
                status: AlgebraicFiberProjectionStatus::UnsupportedCoefficient,
                coefficients: Vec::new(),
            };
        }
        Err(LocalFieldError::DivisionByZero | LocalFieldError::Undecided) => {
            return AlgebraicFiberProjectionReport {
                status: AlgebraicFiberProjectionStatus::Undecided,
                coefficients: Vec::new(),
            };
        }
    };
    let fiber_coefficients = fiber_coefficient_polynomials(polynomial, retained_parameter);
    match quotient_ring_fiber_resultant_polynomial(
        &field.modulus,
        &fiber_coefficients,
        max_source_degree,
    ) {
        Some(coefficients) => AlgebraicFiberProjectionReport {
            status: AlgebraicFiberProjectionStatus::Constructed,
            coefficients,
        },
        None if polynomial
            .coefficients
            .iter()
            .flatten()
            .any(|coefficient| coefficient.exact_rational_ref().is_none()) =>
        {
            AlgebraicFiberProjectionReport {
                status: AlgebraicFiberProjectionStatus::UnsupportedCoefficient,
                coefficients: Vec::new(),
            }
        }
        None => AlgebraicFiberProjectionReport {
            status: AlgebraicFiberProjectionStatus::Undecided,
            coefficients: Vec::new(),
        },
    }
}

fn local_fiber_to_bivariate(
    fiber: &[LocalFieldElement],
    retained_parameter: CurveResultantParameter,
) -> Option<BivariatePolynomial> {
    let coefficients = fiber
        .iter()
        .map(|coefficient| {
            coefficient
                .denominator
                .is_none()
                .then(|| coefficient.numerator.clone())
        })
        .collect::<Option<Vec<_>>>()?;
    Some(BivariatePolynomial::new(match retained_parameter {
        CurveResultantParameter::First => {
            let retained_count = coefficients.iter().map(Vec::len).max().unwrap_or(0);
            (0..retained_count)
                .map(|retained_power| {
                    coefficients
                        .iter()
                        .map(|coefficient| {
                            coefficient
                                .get(retained_power)
                                .cloned()
                                .unwrap_or_else(Real::zero)
                        })
                        .collect()
                })
                .collect()
        }
        CurveResultantParameter::Second => coefficients,
    }))
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
    if fiber_lower.exact_rational_ref().is_none()
        || fiber_upper.exact_rational_ref().is_none()
        || polynomials.iter().any(|polynomial| {
            polynomial
                .coefficients
                .iter()
                .flatten()
                .any(|coefficient| coefficient.exact_rational_ref().is_none())
        })
    {
        return fiber_root_count_report(
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
            None,
            0,
            0,
            Certainty::Exact,
            Some("fiber root counting requires exact-rational coefficients and endpoints"),
        );
    }

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
    let mut first = local_fiber_polynomial(first, retained_parameter, field)?;
    let mut second = local_fiber_polynomial(second, retained_parameter, field)?;
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
    if first.len() < second.len() {
        std::mem::swap(&mut first, &mut second);
    }
    while !local_polynomial_is_zero(&second, field)? {
        let remainder = local_polynomial_remainder(first, &second, field)?;
        first = second;
        second = remainder;
    }
    count_local_polynomial_roots(first, fiber_lower, fiber_upper, endpoints, field)
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
        sequence.push(remainder);
    }
    Ok(sequence)
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
        let mut quotient = vec![LocalFieldElement::zero(); degree];
        quotient[degree - 1] = polynomial[degree].clone();
        for power in (1..degree).rev() {
            quotient[power - 1] = quotient[power]
                .scale(root, field)?
                .add(&polynomial[power], field)?;
        }
        let remainder = quotient[0].scale(root, field)?.add(&polynomial[0], field)?;
        if !remainder.is_zero(field)? {
            return Err(LocalFieldError::Undecided);
        }
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

fn local_polynomial_remainder(
    mut dividend: Vec<LocalFieldElement>,
    divisor: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut dividend, field)?;
    if divisor.is_empty() || (divisor.len() == 1 && divisor[0].is_zero(field)?) {
        return Err(LocalFieldError::DivisionByZero);
    }
    while dividend.len() >= divisor.len() && !(dividend.len() == 1 && dividend[0].is_zero(field)?) {
        let degree_delta = dividend.len() - divisor.len();
        let scale = dividend.last().ok_or(LocalFieldError::Undecided)?.divide(
            divisor.last().ok_or(LocalFieldError::DivisionByZero)?,
            field,
        )?;
        for (power, divisor_coefficient) in divisor.iter().enumerate() {
            let product = scale.multiply(divisor_coefficient, field)?;
            dividend[degree_delta + power] =
                dividend[degree_delta + power].subtract(&product, field)?;
        }
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
    let mut value = LocalFieldElement::zero();
    for coefficient in polynomial.iter().rev() {
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
        let numerator = match &other.denominator {
            Some(denominator) => field.multiply_polynomials(&self.numerator, denominator)?,
            None => self.numerator.clone(),
        };
        let denominator = match &self.denominator {
            Some(denominator) => field.multiply_polynomials(denominator, &other.numerator)?,
            None => other.numerator.clone(),
        };
        Ok(Self {
            numerator,
            denominator: field.canonical_denominator(denominator),
        })
    }

    fn scale(&self, scale: &Real, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        if scale.exact_rational_ref().is_none() {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
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
}

struct LocalAlgebraicField {
    root: AlgebraicRootRepresentation,
    modulus: Vec<Real>,
    signed_polynomials: Vec<(Vec<Real>, Ordering)>,
    policy: PredicatePolicy,
    certainty: Certainty,
    refinement_steps: usize,
}

impl LocalAlgebraicField {
    fn new(
        root: &AlgebraicRootRepresentation,
        policy: PredicatePolicy,
    ) -> Result<Self, LocalFieldError> {
        if !root.is_valid()
            || root.interval.distinct_root_count != 1
            || root.polynomial_coefficients.len() <= 1
        {
            return Err(LocalFieldError::InvalidEvidence);
        }
        if root
            .polynomial_coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
        {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
        Ok(Self {
            root: root.clone(),
            modulus: root.polynomial_coefficients.clone(),
            signed_polynomials: Vec::new(),
            policy,
            certainty: Certainty::Exact,
            refinement_steps: 0,
        })
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
        if polynomial
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
        {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
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
        if polynomial.len() < self.modulus.len() {
            return Ok(polynomial);
        }
        polynomial_div_rem(polynomial, &self.modulus, self.policy)
            .map(|(_, remainder)| remainder)
            .ok_or(LocalFieldError::Undecided)
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
        let mut result = vec![Real::zero(); left.len() + right.len() - 1];
        for (left_power, left_coefficient) in left.iter().enumerate() {
            for (right_power, right_coefficient) in right.iter().enumerate() {
                result[left_power + right_power] =
                    result[left_power + right_power].clone() + left_coefficient * right_coefficient;
            }
        }
        self.reduce(result)
    }

    fn sign_polynomial(&mut self, polynomial: &[Real]) -> Result<Ordering, LocalFieldError> {
        let polynomial = self.reduce(polynomial.to_vec())?;
        if polynomial
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            return Ok(Ordering::Equal);
        }
        if let Some((_, sign)) = self
            .signed_polynomials
            .iter()
            .find(|(signed, _)| signed == &polynomial)
        {
            return Ok(*sign);
        }
        let sign = self.sign_reduced_polynomial(&polynomial)?;
        self.signed_polynomials.push((polynomial, sign));
        Ok(sign)
    }

    fn is_zero_polynomial(&mut self, polynomial: &[Real]) -> Result<bool, LocalFieldError> {
        let polynomial = self.reduce(polynomial.to_vec())?;
        if polynomial
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            return Ok(true);
        }
        if let Some((_, sign)) = self
            .signed_polynomials
            .iter()
            .find(|(signed, _)| signed == &polynomial)
        {
            return Ok(*sign == Ordering::Equal);
        }
        let evaluation =
            evaluate_polynomial_at_algebraic_root(&self.root, &polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            self.signed_polynomials.push((polynomial, sign));
            return Ok(sign == Ordering::Equal);
        }
        let is_zero = polynomials_share_one_root_in_interval(
            &self.modulus,
            &polynomial,
            &self.root.interval.lower,
            &self.root.interval.upper,
            self.policy,
        )
        .ok_or(LocalFieldError::Undecided)?;
        if is_zero {
            self.signed_polynomials.push((polynomial, Ordering::Equal));
        }
        Ok(is_zero)
    }

    fn sign_reduced_polynomial(
        &mut self,
        polynomial: &[Real],
    ) -> Result<Ordering, LocalFieldError> {
        if self.root.exact_rational_witness().is_some() {
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

        match polynomials_share_one_root_in_interval(
            &self.modulus,
            polynomial,
            &self.root.interval.lower,
            &self.root.interval.upper,
            self.policy,
        ) {
            Some(true) => return Ok(Ordering::Equal),
            Some(false) => {}
            None => return Err(LocalFieldError::Undecided),
        }

        loop {
            let evaluation =
                evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
            if let Some(sign) = local_evaluation_sign(&evaluation)? {
                return Ok(sign);
            }
            self.refine_root()?;
        }
    }

    fn refine_root(&mut self) -> Result<(), LocalFieldError> {
        let refinement = refine_isolated_univariate_polynomial_interval(
            &self.modulus,
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
        self.root.kind = if self.root.interval.exact_root.is_some() {
            AlgebraicRootKind::ExactRationalWitness
        } else {
            AlgebraicRootKind::IsolatingInterval
        };
        self.root.validation = validate_algebraic_root_representation(&self.root, self.policy);
        if !self.root.is_valid() {
            return Err(LocalFieldError::InvalidEvidence);
        }
        Ok(())
    }
}

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

    fn represented_root(
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
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        root.validation = validate_algebraic_root_representation(&root, policy);
        assert!(root.is_valid());
        root
    }

    fn represented_rational_root(
        value: Real,
        policy: PredicatePolicy,
    ) -> AlgebraicRootRepresentation {
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
            kind: AlgebraicRootKind::ExactRationalWitness,
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
                policy,
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
    fn quotient_ring_fiber_projection_obeys_an_explicit_degree_budget() {
        // For 2 alpha^10 - 1 = 0 and F(alpha, y) = y - alpha, the norm is a
        // nonzero scalar multiple of 2 y^10 - 1. The general entry point keeps
        // its degree-eight allocation ceiling, while an operation that owns a
        // degree-ten field can opt into the exact 2^10-state determinant.
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
            let bounded = project_bivariate_fiber_at_algebraic_parameter(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                policy,
            );
            assert_eq!(bounded.status, AlgebraicFiberProjectionStatus::Undecided);
            assert!(bounded.coefficients.is_empty());

            let projected = project_bivariate_fiber_at_algebraic_parameter_with_max_degree(
                &polynomial,
                CurveResultantParameter::First,
                &alpha,
                10,
                policy,
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
        let alpha = represented_rational_root(half.clone(), policy);
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

        let unsupported = count_bivariate_fiber_roots_at_algebraic_parameter(
            &BivariatePolynomial::new(vec![vec![Real::pi(), real(1)]]),
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            unsupported.status,
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        );

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
}
