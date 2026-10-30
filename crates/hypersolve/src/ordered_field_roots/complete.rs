//! Distinct-root isolation using the shared ordered-field Sturm authority.

use super::*;
use crate::root_sign::{PreparedSturmChain, field_polynomial_sign};

struct Boundary {
    value: Real,
    exact_root: bool,
}

struct Node {
    lower: Real,
    upper: Real,
    count: usize,
    refinements: usize,
    depth: usize,
    lower_is_removed_root: bool,
    upper_is_removed_root: bool,
}

fn finish(
    status: OrderedFieldRootIsolationStatus,
    intervals: Vec<IsolatedRootInterval>,
    steps: usize,
) -> OrderedFieldRootIsolationReport {
    if crate::work_budget::exhaustion().is_some() {
        report(
            OrderedFieldRootIsolationStatus::BudgetExhausted,
            Vec::new(),
            steps,
        )
    } else {
        report(status, intervals, steps)
    }
}

fn exact_interval(root: &Real) -> IsolatedRootInterval {
    IsolatedRootInterval {
        lower: root.clone(),
        upper: root.clone(),
        exact_root: Some(root.clone()),
        distinct_root_count: 1,
    }
}

// The caller has already proved the first root occurrence. Remove all its
// multiplicities from the temporary counting polynomial, never from the
// original equation consumed by subsequent selected-field construction.
fn deflate<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    mut polynomial: Vec<C>,
    root: &Real,
    field: &mut F,
) -> Result<Vec<C>, F::Error> {
    let embedded = field.constant(root)?;
    loop {
        polynomial = ordered_field_polynomial_linear_quotient(&polynomial, &embedded, field)?;
        if polynomial.len() <= 1
            || field_polynomial_sign(&polynomial, root, field)? != Ordering::Equal
        {
            return Ok(polynomial);
        }
    }
}

/// Isolates every distinct root in a closed finite represented interval.
///
/// The lower bound must be strictly below the upper bound, as in the
/// Bernstein entry point. Reversed and zero-width ranges are invalid.
///
/// This complete route uses authoritative ordered-field signs and the shared
/// Sturm root count. Repeated, reducible and nonmonic equations are admitted;
/// no polynomial degree or dyadic subdivision-depth cap is imposed. Termination
/// assumes that the caller's exact field decisions complete. A configured
/// remainder-chain limit can still return `CompleteFallbackRequired`.
/// The existing Bernstein API remains the separate speculative fast path.
///
/// `refinement_steps` requests additional subdivisions after a singleton is
/// found. Every subdivision consumes one shared root-refinement unit. These
/// units do not bound coefficient growth, arithmetic, memory or elapsed time.
/// Refusal never publishes a partial list. Endpoint and discovered midpoint
/// roots are emitted once; all other intervals exclude them, including at
/// interval boundaries. Thus the caller can select each nonpoint interval
/// using its original equation, without discarding multiplicities or guards.
pub fn isolate_ordered_field_polynomial_roots_complete<C, F>(
    polynomial: Vec<C>,
    lower: &Real,
    upper: &Real,
    refinement_steps: usize,
    field: &mut F,
) -> Result<OrderedFieldRootIsolationReport, F::Error>
where
    C: Clone,
    F: OrderedFieldPolynomialContext<C>,
{
    isolate(polynomial, lower, upper, refinement_steps, None, field)
}

/// Runs the same complete Sturm algorithm with a local subdivision-depth limit.
///
/// The configured depth applies to separation and requested refinement in each
/// segment between discovered exact roots, as for native algebraic fibers.
/// Discovering an exact root creates new segments; this depth limit is not a
/// bound on total subdivisions or on arithmetic, coefficient size or memory.
/// Shared work accounting remains active across all segments and nested calls.
/// Reaching the depth limit returns `BudgetExhausted` without partial output;
/// a later larger-limit retry remains valid. The unrestricted entry point keeps
/// its existing behavior and shares this implementation.
pub fn isolate_ordered_field_polynomial_roots_complete_with_config<C, F>(
    polynomial: Vec<C>,
    lower: &Real,
    upper: &Real,
    config: OrderedFieldRootIsolationConfig,
    field: &mut F,
) -> Result<OrderedFieldRootIsolationReport, F::Error>
where
    C: Clone,
    F: OrderedFieldPolynomialContext<C>,
{
    isolate(
        polynomial,
        lower,
        upper,
        config.refinement_steps,
        Some(config.max_subdivision_depth),
        field,
    )
}

fn isolate<C, F>(
    mut polynomial: Vec<C>,
    lower: &Real,
    upper: &Real,
    refinement_steps: usize,
    depth_limit: Option<usize>,
    field: &mut F,
) -> Result<OrderedFieldRootIsolationReport, F::Error>
where
    C: Clone,
    F: OrderedFieldPolynomialContext<C>,
{
    use OrderedFieldRootIsolationStatus as Status;
    if crate::work_budget::exhaustion().is_some() {
        return Ok(finish(Status::BudgetExhausted, Vec::new(), 0));
    }
    let width = field.constant(&(upper - lower))?;
    if field.sign(&width)? != Ordering::Greater {
        return Ok(finish(Status::InvalidInterval, Vec::new(), 0));
    }
    if polynomial.is_empty() {
        polynomial.push(field.constant(&Real::zero())?);
    }
    trim_polynomial(&mut polynomial, field)?;
    if polynomial.len() == 1 {
        let status = if field.sign(&polynomial[0])? == Ordering::Equal {
            Status::IdenticallyZero
        } else {
            Status::Isolated
        };
        return Ok(finish(status, Vec::new(), 0));
    }
    let mut boundaries = Vec::with_capacity(2);
    for value in [lower, upper] {
        let exact_root = field_polynomial_sign(&polynomial, value, field)? == Ordering::Equal;
        if exact_root {
            polynomial = deflate(polynomial, value, field)?;
        }
        boundaries.push(Boundary {
            value: value.clone(),
            exact_root,
        });
    }
    let mut steps = 0_usize;
    'restart: loop {
        let Some(chain) = PreparedSturmChain::new(&polynomial, field)? else {
            return Ok(finish(Status::CompleteFallbackRequired, Vec::new(), steps));
        };
        // Discovering a represented root restarts with its exact quotient.
        // Boundaries remain ordered by certified dyadic subdivision, so no
        // approximate scalar comparison is needed to order the result.
        let mut intervals = Vec::new();
        for segment in 0..boundaries.len() - 1 {
            let lower = &boundaries[segment];
            let upper = &boundaries[segment + 1];
            if lower.exact_root {
                intervals.push(exact_interval(&lower.value));
            }
            let Some(count) = chain.count(&lower.value, &upper.value, field)? else {
                return Ok(finish(Status::CompleteFallbackRequired, Vec::new(), steps));
            };
            let mut stack = vec![Node {
                lower: lower.value.clone(),
                upper: upper.value.clone(),
                count,
                refinements: 0,
                depth: 0,
                lower_is_removed_root: lower.exact_root,
                upper_is_removed_root: upper.exact_root,
            }];
            while let Some(node) = stack.pop() {
                if node.count == 0 {
                    continue;
                }
                if node.count == 1
                    && node.refinements >= refinement_steps
                    && !node.lower_is_removed_root
                    && !node.upper_is_removed_root
                {
                    intervals.push(IsolatedRootInterval {
                        lower: node.lower,
                        upper: node.upper,
                        exact_root: None,
                        distinct_root_count: 1,
                    });
                    continue;
                }
                if depth_limit.is_some_and(|limit| node.depth >= limit) {
                    return Ok(finish(Status::BudgetExhausted, Vec::new(), steps));
                }
                if crate::work_budget::consume(crate::work_budget::WorkKind::RootRefinement, 1)
                    .is_err()
                {
                    return Ok(finish(Status::BudgetExhausted, Vec::new(), steps));
                }
                let Some(middle) = midpoint(&node.lower, &node.upper) else {
                    return Ok(finish(Status::CompleteFallbackRequired, Vec::new(), steps));
                };
                steps = steps.saturating_add(1);
                if field_polynomial_sign(&polynomial, &middle, field)? == Ordering::Equal {
                    polynomial = deflate(polynomial, &middle, field)?;
                    boundaries.insert(
                        segment + 1,
                        Boundary {
                            value: middle,
                            exact_root: true,
                        },
                    );
                    continue 'restart;
                }
                let Some(left_count) = chain.count(&node.lower, &middle, field)? else {
                    return Ok(finish(Status::CompleteFallbackRequired, Vec::new(), steps));
                };
                let Some(right_count) = node.count.checked_sub(left_count) else {
                    return Ok(finish(Status::CompleteFallbackRequired, Vec::new(), steps));
                };
                let refinements = if node.count == 1 {
                    node.refinements.saturating_add(1)
                } else {
                    0
                };
                stack.push(Node {
                    lower: middle.clone(),
                    upper: node.upper,
                    count: right_count,
                    refinements,
                    depth: node.depth.saturating_add(1),
                    lower_is_removed_root: false,
                    upper_is_removed_root: node.upper_is_removed_root,
                });
                stack.push(Node {
                    lower: node.lower,
                    upper: middle,
                    count: left_count,
                    refinements,
                    depth: node.depth.saturating_add(1),
                    lower_is_removed_root: node.lower_is_removed_root,
                    upper_is_removed_root: false,
                });
            }
        }
        let last = boundaries
            .last()
            .expect("both interval endpoints are retained");
        if last.exact_root {
            intervals.push(exact_interval(&last.value));
        }
        return Ok(finish(Status::Isolated, intervals, steps));
    }
}
