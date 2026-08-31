//! Division-free root isolation over an exact ordered coefficient field.
//!
//! The coefficient field remains owned by the caller.  This module only uses
//! addition, exact rational scaling, and sign predicates, so an already
//! selected algebraic or radical tower need not be collapsed into a primitive
//! element merely to isolate simple roots of a univariate polynomial.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::root_isolation::IsolatedRootInterval;

/// Arithmetic and sign authority for one exact ordered coefficient field.
///
/// `sign_if_separated` is a speculative fast predicate.  Returning `None`
/// never asserts equality; it asks the Bernstein isolator to subdivide or to
/// decline to the caller's complete fallback.
pub trait OrderedFieldPolynomialContext<C> {
    /// Caller-owned arithmetic or predicate error.
    type Error;

    /// Additive identity in this coefficient field.
    fn zero(&mut self) -> Result<C, Self::Error>;

    /// Exact field addition.
    fn add(&mut self, left: &C, right: &C) -> Result<C, Self::Error>;

    /// Exact multiplication by a represented real scalar.
    fn scale(&mut self, value: &C, scale: &Real) -> Result<C, Self::Error>;

    /// Authoritative exact sign.
    fn sign(&mut self, value: &C) -> Result<Ordering, Self::Error>;

    /// Cheap sign when exact interval separation is already available.
    fn sign_if_separated(&mut self, value: &C) -> Result<Option<Ordering>, Self::Error>;
}

/// Work limits for division-free ordered-field Bernstein isolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OrderedFieldRootIsolationConfig {
    /// Maximum dyadic subdivision depth before the complete caller fallback.
    pub max_subdivision_depth: usize,
    /// Additional dyadic refinements after a variation-one interval is found.
    pub refinement_steps: usize,
}

impl Default for OrderedFieldRootIsolationConfig {
    fn default() -> Self {
        Self {
            max_subdivision_depth: 128,
            refinement_steps: 8,
        }
    }
}

/// Terminal state of the division-free isolation attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OrderedFieldRootIsolationStatus {
    /// Every distinct root in the requested closed interval was isolated.
    Isolated,
    /// Every exact coefficient vanished.
    IdenticallyZero,
    /// A repeated/nonseparable root or speculative sign exceeded this path.
    CompleteFallbackRequired,
    /// The requested interval was empty or reversed.
    InvalidInterval,
}

/// Exact result of one division-free ordered-field isolation attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderedFieldRootIsolationReport {
    /// Terminal status.
    pub status: OrderedFieldRootIsolationStatus,
    /// Certified singleton intervals when `status == Isolated`.
    pub intervals: Vec<IsolatedRootInterval>,
    /// Number of dyadic subdivision steps performed.
    pub subdivision_steps: usize,
}

fn report(
    status: OrderedFieldRootIsolationStatus,
    intervals: Vec<IsolatedRootInterval>,
    subdivision_steps: usize,
) -> OrderedFieldRootIsolationReport {
    OrderedFieldRootIsolationReport {
        status,
        intervals,
        subdivision_steps,
    }
}

fn midpoint(lower: &Real, upper: &Real) -> Option<Real> {
    ((lower + upper) / Real::from(2_u8)).ok()
}

fn trim_polynomial<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &mut Vec<C>,
    field: &mut F,
) -> Result<(), F::Error> {
    while polynomial.len() > 1
        && field.sign(
            polynomial
                .last()
                .expect("a nonempty polynomial retains a leading coefficient"),
        )? == Ordering::Equal
    {
        polynomial.pop();
    }
    Ok(())
}

fn polynomial_sign_at_if_separated<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &[C],
    parameter: &Real,
    field: &mut F,
) -> Result<Option<Ordering>, F::Error> {
    let mut value = field.zero()?;
    for coefficient in polynomial.iter().rev() {
        let scaled = field.scale(&value, parameter)?;
        value = field.add(&scaled, coefficient)?;
    }
    field.sign_if_separated(&value)
}

fn deflate_at_represented_root<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    mut polynomial: Vec<C>,
    root: &Real,
    field: &mut F,
) -> Result<(Vec<C>, bool), F::Error> {
    let mut had_root = false;
    while polynomial.len() > 1
        && polynomial_sign_at_if_separated(&polynomial, root, field)? == Some(Ordering::Equal)
    {
        let degree = polynomial.len() - 1;
        let mut quotient = vec![field.zero()?; degree];
        quotient[degree - 1] = polynomial[degree].clone();
        for power in (1..degree).rev() {
            let product = field.scale(&quotient[power], root)?;
            quotient[power - 1] = field.add(&product, &polynomial[power])?;
        }
        let product = field.scale(&quotient[0], root)?;
        let remainder = field.add(&product, &polynomial[0])?;
        if field.sign(&remainder)? != Ordering::Equal {
            // The exact sign predicate selected this root, so a nonzero
            // synthetic remainder means the caller's field contract broke.
            return Ok((polynomial, had_root));
        }
        polynomial = quotient;
        trim_polynomial(&mut polynomial, field)?;
        had_root = true;
    }
    Ok((polynomial, had_root))
}

fn power_to_bernstein_on_interval<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &[C],
    lower: &Real,
    upper: &Real,
    field: &mut F,
) -> Result<Option<Vec<C>>, F::Error> {
    let Some(leading) = polynomial.last() else {
        return Ok(None);
    };
    let degree = polynomial.len().saturating_sub(1);
    let width = upper - lower;
    // Compose by `lower + width*x` using Horner form. Recomposition at each
    // child bounds expression depth by the authored polynomial degree.
    let mut shifted_power = vec![leading.clone()];
    for coefficient in polynomial[..degree].iter().rev() {
        let old_len = shifted_power.len();
        shifted_power.push(field.zero()?);
        for power in (1..=old_len).rev() {
            let same_power = field.scale(&shifted_power[power], lower)?;
            let prior_power = field.scale(&shifted_power[power - 1], &width)?;
            shifted_power[power] = field.add(&same_power, &prior_power)?;
        }
        let constant = field.scale(&shifted_power[0], lower)?;
        shifted_power[0] = field.add(&constant, coefficient)?;
    }

    // x^j = sum_{i=j}^n C(i,j)/C(n,j) B_i^n(x).
    let mut controls = Vec::with_capacity(degree + 1);
    for index in 0..=degree {
        let mut control = field.zero()?;
        let mut ratio = Real::one();
        for (power, coefficient) in shifted_power.iter().enumerate().take(index + 1) {
            let term = field.scale(coefficient, &ratio)?;
            control = field.add(&control, &term)?;
            if power != index {
                let (Ok(numerator), Ok(denominator)) =
                    (u64::try_from(index - power), u64::try_from(degree - power))
                else {
                    return Ok(None);
                };
                let Ok(next) = (ratio * Real::from(numerator)) / Real::from(denominator) else {
                    return Ok(None);
                };
                ratio = next;
            }
        }
        controls.push(control);
    }
    Ok(Some(controls))
}

fn bernstein_sign_variations<C, F: OrderedFieldPolynomialContext<C>>(
    controls: &[C],
    field: &mut F,
) -> Result<Option<usize>, F::Error> {
    // Preserve every exact assignment compatible with a nonseparated
    // coefficient. A common variation count remains a valid Descartes count;
    // disagreement asks the caller to subdivide.
    let mut states = vec![(None, 0_usize)];
    for control in controls {
        let sign = field.sign_if_separated(control)?;
        let options: &[Option<Ordering>] = match sign {
            Some(Ordering::Less) => &[Some(Ordering::Less)],
            Some(Ordering::Equal) => &[None],
            Some(Ordering::Greater) => &[Some(Ordering::Greater)],
            None => &[None, Some(Ordering::Less), Some(Ordering::Greater)],
        };
        let mut next = Vec::with_capacity(states.len().saturating_mul(options.len()));
        for &(previous, variations) in &states {
            for &option in options {
                let state = match option {
                    None => (previous, variations),
                    Some(sign) => (
                        Some(sign),
                        variations + usize::from(previous.is_some_and(|value| value != sign)),
                    ),
                };
                if !next.contains(&state) {
                    next.push(state);
                }
            }
        }
        states = next;
    }
    let first = states.first().map(|state| state.1).unwrap_or(0);
    Ok(states.iter().all(|state| state.1 == first).then_some(first))
}

/// Isolate all simple roots of `polynomial` in one closed represented range.
///
/// Variation-one intervals and rational roots are exact construction
/// evidence. Irrational repeated roots deliberately return
/// [`OrderedFieldRootIsolationStatus::CompleteFallbackRequired`], allowing a
/// caller with field division or a global projection to retain completeness.
pub fn isolate_ordered_field_polynomial_roots<C, F>(
    mut polynomial: Vec<C>,
    lower: &Real,
    upper: &Real,
    config: OrderedFieldRootIsolationConfig,
    field: &mut F,
) -> Result<OrderedFieldRootIsolationReport, F::Error>
where
    C: Clone,
    F: OrderedFieldPolynomialContext<C>,
{
    if lower.partial_cmp(upper) != Some(Ordering::Less) {
        return Ok(report(
            OrderedFieldRootIsolationStatus::InvalidInterval,
            Vec::new(),
            0,
        ));
    }
    if polynomial.is_empty() {
        polynomial.push(field.zero()?);
    }
    trim_polynomial(&mut polynomial, field)?;
    if polynomial.len() == 1 {
        let status = if field.sign(&polynomial[0])? == Ordering::Equal {
            OrderedFieldRootIsolationStatus::IdenticallyZero
        } else {
            OrderedFieldRootIsolationStatus::Isolated
        };
        return Ok(report(status, Vec::new(), 0));
    }

    let mut exact_roots = Vec::new();
    for endpoint in [lower, upper] {
        let (deflated, had_root) = deflate_at_represented_root(polynomial, endpoint, field)?;
        polynomial = deflated;
        if had_root {
            exact_roots.push(endpoint.clone());
        }
    }

    #[derive(Clone)]
    struct Node<C> {
        lower: Real,
        upper: Real,
        controls: Vec<C>,
        depth: usize,
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
            return Ok(report(
                OrderedFieldRootIsolationStatus::Isolated,
                intervals,
                subdivision_steps,
            ));
        }

        exact_roots.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
        exact_roots.dedup();
        let mut boundaries = Vec::with_capacity(exact_roots.len() + 2);
        boundaries.push(lower.clone());
        boundaries.extend(
            exact_roots
                .iter()
                .filter(|root| root > &lower && root < &upper)
                .cloned(),
        );
        boundaries.push(upper.clone());

        let mut stack = Vec::with_capacity(boundaries.len());
        for segment in boundaries.windows(2).rev() {
            let Some(controls) =
                power_to_bernstein_on_interval(&polynomial, &segment[0], &segment[1], field)?
            else {
                return Ok(report(
                    OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                    Vec::new(),
                    subdivision_steps,
                ));
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
            let variations = bernstein_sign_variations(&node.controls, field)?;
            if variations == Some(0) {
                continue;
            }
            if variations == Some(1) {
                let lower_sign = node
                    .controls
                    .first()
                    .map(|value| field.sign_if_separated(value))
                    .transpose()?
                    .flatten();
                for _ in 0..config.refinement_steps {
                    let Some(lower_sign) = lower_sign else {
                        break;
                    };
                    if lower_sign == Ordering::Equal {
                        break;
                    }
                    if node.depth >= config.max_subdivision_depth {
                        return Ok(report(
                            OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                            Vec::new(),
                            subdivision_steps,
                        ));
                    }
                    let Some(midpoint) = midpoint(&node.lower, &node.upper) else {
                        return Ok(report(
                            OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                            Vec::new(),
                            subdivision_steps,
                        ));
                    };
                    subdivision_steps = subdivision_steps.saturating_add(1);
                    let midpoint_sign =
                        polynomial_sign_at_if_separated(&polynomial, &midpoint, field)?;
                    let Some(midpoint_sign) = midpoint_sign else {
                        break;
                    };
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
                return Ok(report(
                    OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                    Vec::new(),
                    subdivision_steps,
                ));
            }
            let Some(midpoint) = midpoint(&node.lower, &node.upper) else {
                return Ok(report(
                    OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                    Vec::new(),
                    subdivision_steps,
                ));
            };
            let (Some(left), Some(right)) = (
                power_to_bernstein_on_interval(&polynomial, &node.lower, &midpoint, field)?,
                power_to_bernstein_on_interval(&polynomial, &midpoint, &node.upper, field)?,
            ) else {
                return Ok(report(
                    OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                    Vec::new(),
                    subdivision_steps,
                ));
            };
            subdivision_steps = subdivision_steps.saturating_add(1);
            if matches!(
                left.last()
                    .map(|value| field.sign_if_separated(value))
                    .transpose()?
                    .flatten(),
                Some(Ordering::Equal)
            ) {
                rational_root = Some(midpoint);
                break;
            }
            let left_variations = bernstein_sign_variations(&left, field)?;
            let right_variations = bernstein_sign_variations(&right, field)?;
            let next_depth = node.depth + 1;
            if right_variations != Some(0) {
                stack.push(Node {
                    lower: midpoint.clone(),
                    upper: node.upper,
                    controls: right,
                    depth: next_depth,
                });
            }
            if left_variations != Some(0) {
                stack.push(Node {
                    lower: node.lower,
                    upper: midpoint,
                    controls: left,
                    depth: next_depth,
                });
            }
        }

        if let Some(root) = rational_root {
            let (deflated, had_root) = deflate_at_represented_root(polynomial, &root, field)?;
            if !had_root {
                return Ok(report(
                    OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                    Vec::new(),
                    subdivision_steps,
                ));
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
        return Ok(report(
            OrderedFieldRootIsolationStatus::Isolated,
            isolated,
            subdivision_steps,
        ));
    }
}
