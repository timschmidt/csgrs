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
    let Some((leading, remaining)) = polynomial.split_last() else {
        let zero = field.zero()?;
        return field.sign_if_separated(&zero);
    };
    let mut value = leading.clone();
    for coefficient in remaining.iter().rev() {
        let scaled = field.scale(&value, parameter)?;
        value = field.add(&scaled, coefficient)?;
    }
    field.sign_if_separated(&value)
}

/// Returns the Euclidean quotient by `x - root` in the caller's exact field.
///
/// Coefficients are in ascending power order. The remainder is deliberately
/// not constructed or signed. A caller using the quotient as a deflation must
/// already own a root certificate, which may come from geometric incidence
/// rather than another evaluation of the polynomial. No division is required.
pub fn ordered_field_polynomial_linear_quotient<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &[C],
    root: &Real,
    field: &mut F,
) -> Result<Vec<C>, F::Error> {
    let degree = polynomial.len().saturating_sub(1);
    if degree == 0 {
        return Ok(Vec::new());
    }
    let mut quotient = Vec::with_capacity(degree);
    quotient.push(polynomial[degree].clone());
    for coefficient in polynomial[1..degree].iter().rev() {
        let product = field.scale(
            quotient.last().expect("the quotient has a leading term"),
            root,
        )?;
        quotient.push(field.add(&product, coefficient)?);
    }
    quotient.reverse();
    Ok(quotient)
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
        polynomial = ordered_field_polynomial_linear_quotient(&polynomial, root, field)?;
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
    // Compose by `lower + width*x` using Horner form. Periodic recomposition
    // bounds expression depth even when field operations retain lazy nodes.
    let mut shifted_power = vec![leading.clone()];
    for coefficient in polynomial[..degree].iter().rev() {
        let old_len = shifted_power.len();
        let highest_power = field.scale(&shifted_power[old_len - 1], &width)?;
        shifted_power.push(highest_power);
        for power in (1..old_len).rev() {
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
        let mut control = shifted_power[0].clone();
        let mut ratio = Real::one();
        if index != 0 {
            let (Ok(numerator), Ok(denominator)) = (u64::try_from(index), u64::try_from(degree))
            else {
                return Ok(None);
            };
            let Ok(next) = (ratio * Real::from(numerator)) / Real::from(denominator) else {
                return Ok(None);
            };
            ratio = next;
        }
        for (power, coefficient) in shifted_power.iter().enumerate().take(index + 1).skip(1) {
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

fn midpoint_subdivide<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    mut work: Vec<C>,
    field: &mut F,
) -> Result<(Vec<C>, Vec<C>), F::Error> {
    let degree = work.len() - 1;
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let mut left = Vec::with_capacity(work.len());
    let mut right = Vec::with_capacity(work.len());
    left.push(work[0].clone());
    right.push(work[degree].clone());
    for level in 1..=degree {
        for index in 0..=degree - level {
            let sum = field.add(&work[index], &work[index + 1])?;
            work[index] = field.scale(&sum, &half)?;
        }
        left.push(work[0].clone());
        right.push(work[degree - level].clone());
    }
    right.reverse();
    Ok((left, right))
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
            // Share the midpoint tableau between both children, but restart
            // from the authored polynomial every eight levels. Thus lazy
            // coefficient expressions cannot grow with subdivision depth.
            if node.depth != 0 && node.depth.is_multiple_of(8) {
                let Some(controls) =
                    power_to_bernstein_on_interval(&polynomial, &node.lower, &node.upper, field)?
                else {
                    return Ok(report(
                        OrderedFieldRootIsolationStatus::CompleteFallbackRequired,
                        Vec::new(),
                        subdivision_steps,
                    ));
                };
                node.controls = controls;
            }
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
            let (left, right) = midpoint_subdivide(node.controls, field)?;
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

#[cfg(test)]
mod tests {
    use super::*;

    struct RationalRealContext;

    impl OrderedFieldPolynomialContext<Real> for RationalRealContext {
        type Error = ();

        fn zero(&mut self) -> Result<Real, Self::Error> {
            Ok(Real::zero())
        }

        fn add(&mut self, left: &Real, right: &Real) -> Result<Real, Self::Error> {
            Ok(left + right)
        }

        fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, Self::Error> {
            Ok(value * scale)
        }

        fn sign(&mut self, value: &Real) -> Result<Ordering, Self::Error> {
            value.partial_cmp(&Real::zero()).ok_or(())
        }

        fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, Self::Error> {
            self.sign(value).map(Some)
        }
    }

    fn fraction(numerator: i64, denominator: i64) -> Real {
        (Real::from(numerator) / Real::from(denominator)).expect("nonzero integer denominator")
    }

    #[test]
    fn linear_quotient_reuses_certified_nonrational_roots_without_predicates() {
        struct ArithmeticOnly;
        impl OrderedFieldPolynomialContext<Real> for ArithmeticOnly {
            type Error = ();
            fn zero(&mut self) -> Result<Real, ()> {
                panic!("synthetic division does not reconstruct the zero remainder")
            }
            fn add(&mut self, left: &Real, right: &Real) -> Result<Real, ()> {
                Ok(left + right)
            }
            fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, ()> {
                Ok(value * scale)
            }
            fn sign(&mut self, _: &Real) -> Result<Ordering, ()> {
                panic!("the caller already owns the incidence proof")
            }
            fn sign_if_separated(&mut self, _: &Real) -> Result<Option<Ordering>, ()> {
                panic!("the quotient needs no speculative predicate")
            }
        }
        for root in [
            Real::zero(),
            Real::one(),
            Real::from(2).sqrt().unwrap(),
            Real::pi(),
        ] {
            // (x-root)(x^2+3x+7), with a freely chosen remainder. The
            // quotient remains the same even when root is not a zero.
            for remainder in [Real::zero(), Real::from(11)] {
                let polynomial = [
                    -Real::from(7) * &root + remainder,
                    Real::from(7) - Real::from(3) * &root,
                    Real::from(3) - &root,
                    Real::one(),
                ];
                let quotient = ordered_field_polynomial_linear_quotient(
                    &polynomial,
                    &root,
                    &mut ArithmeticOnly,
                )
                .unwrap();
                assert_eq!(
                    quotient
                        .iter()
                        .map(Real::exact_rational_normal_form)
                        .collect::<Vec<_>>(),
                    [7, 3, 1].map(|value| Real::from(value).exact_rational())
                );
            }
        }
        for polynomial in [vec![], vec![Real::from(7)]] {
            assert!(
                ordered_field_polynomial_linear_quotient(
                    &polynomial,
                    &Real::pi(),
                    &mut ArithmeticOnly,
                )
                .unwrap()
                .is_empty()
            );
        }
    }

    #[test]
    fn midpoint_tableau_matches_authored_polynomial_on_both_children() {
        for degree in 0..=9 {
            let polynomial: Vec<_> = (0..=degree)
                .map(|power| fraction((-1_i64).pow(power as u32) * (power + 2), power + 3))
                .collect();
            for (lower, upper) in [
                (fraction(-5, 3), fraction(-2, 3)),
                (Real::zero(), Real::one()),
                (fraction(7, 9), fraction(11, 6)),
            ] {
                let controls = power_to_bernstein_on_interval(
                    &polynomial,
                    &lower,
                    &upper,
                    &mut RationalRealContext,
                )
                .unwrap()
                .unwrap();
                let (left, right) = midpoint_subdivide(controls, &mut RationalRealContext).unwrap();
                let midpoint = midpoint(&lower, &upper).unwrap();
                for (actual, lower, upper) in
                    [(&left, &lower, &midpoint), (&right, &midpoint, &upper)]
                {
                    let expected = power_to_bernstein_on_interval(
                        &polynomial,
                        lower,
                        upper,
                        &mut RationalRealContext,
                    )
                    .unwrap()
                    .unwrap();
                    assert_eq!(actual, &expected, "degree {degree} on [{lower}, {upper}]");
                }
                assert_eq!(left.last(), right.first());
            }
        }
    }

    #[derive(Clone)]
    struct DepthTracked {
        value: Real,
        depth: usize,
    }

    #[derive(Default)]
    struct DepthTrackingContext {
        max_depth: usize,
    }

    impl OrderedFieldPolynomialContext<DepthTracked> for DepthTrackingContext {
        type Error = ();

        fn zero(&mut self) -> Result<DepthTracked, Self::Error> {
            Ok(DepthTracked {
                value: Real::zero(),
                depth: 0,
            })
        }

        fn add(
            &mut self,
            left: &DepthTracked,
            right: &DepthTracked,
        ) -> Result<DepthTracked, Self::Error> {
            let depth = left.depth.max(right.depth) + 1;
            self.max_depth = self.max_depth.max(depth);
            Ok(DepthTracked {
                value: &left.value + &right.value,
                depth,
            })
        }

        fn scale(
            &mut self,
            value: &DepthTracked,
            scale: &Real,
        ) -> Result<DepthTracked, Self::Error> {
            let depth = value.depth + 1;
            self.max_depth = self.max_depth.max(depth);
            Ok(DepthTracked {
                value: &value.value * scale,
                depth,
            })
        }

        fn sign(&mut self, value: &DepthTracked) -> Result<Ordering, Self::Error> {
            value.value.partial_cmp(&Real::zero()).ok_or(())
        }

        fn sign_if_separated(
            &mut self,
            value: &DepthTracked,
        ) -> Result<Option<Ordering>, Self::Error> {
            self.sign(value).map(Some)
        }
    }

    #[test]
    fn repeated_root_subdivision_keeps_coefficient_expression_depth_bounded() {
        let mut depths = Vec::new();
        for max_subdivision_depth in [16, 96] {
            let polynomial = [4, 0, -4, 0, 1].map(|value| DepthTracked {
                value: Real::from(value),
                depth: 0,
            });
            let mut field = DepthTrackingContext::default();
            let report = isolate_ordered_field_polynomial_roots(
                polynomial.to_vec(),
                &Real::one(),
                &Real::from(2),
                OrderedFieldRootIsolationConfig {
                    max_subdivision_depth,
                    refinement_steps: 0,
                },
                &mut field,
            )
            .unwrap();
            assert_eq!(
                report.status,
                OrderedFieldRootIsolationStatus::CompleteFallbackRequired
            );
            assert!(report.intervals.is_empty());
            assert!(report.subdivision_steps >= max_subdivision_depth);
            assert!(field.max_depth <= 81, "depth {}", field.max_depth);
            depths.push(field.max_depth);
        }
        assert_eq!(depths[0], depths[1]);
    }

    #[test]
    fn clustered_simple_roots_survive_periodic_coefficient_recomposition() {
        let center = fraction(1, 5);
        let radius = fraction(1, 1 << 20);
        let polynomial = vec![
            &center * &center - &radius * &radius,
            Real::from(-2) * &center,
            Real::one(),
        ];
        let report = isolate_ordered_field_polynomial_roots(
            polynomial,
            &Real::zero(),
            &Real::one(),
            OrderedFieldRootIsolationConfig {
                max_subdivision_depth: 64,
                refinement_steps: 8,
            },
            &mut RationalRealContext,
        )
        .unwrap();
        assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
        assert_eq!(report.intervals.len(), 2);
        assert!(report.subdivision_steps > 16);
        assert!(report.intervals[0].upper < report.intervals[1].lower);
        for (interval, root) in report
            .intervals
            .iter()
            .zip([&center - &radius, &center + &radius])
        {
            assert_eq!(interval.distinct_root_count, 1);
            assert!(interval.exact_root.is_none());
            assert!(interval.lower < root && root < interval.upper);
            assert!(&interval.upper - &interval.lower < radius);
        }
    }

    #[test]
    fn rational_endpoint_and_midpoint_roots_deflate_to_distinct_witnesses() {
        // x(x-1)(x-1/2)^2 has three distinct represented roots. This fixes
        // both endpoint ownership and descending synthetic-division storage.
        let polynomial = vec![
            Real::zero(),
            fraction(-1, 4),
            fraction(5, 4),
            Real::from(-2),
            Real::one(),
        ];
        let report = isolate_ordered_field_polynomial_roots(
            polynomial,
            &Real::zero(),
            &Real::one(),
            OrderedFieldRootIsolationConfig {
                max_subdivision_depth: 16,
                refinement_steps: 2,
            },
            &mut RationalRealContext,
        )
        .expect("rational field operations are total");

        assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
        assert_eq!(report.intervals.len(), 3);
        assert_eq!(report.intervals[0].exact_root, Some(Real::zero()));
        assert_eq!(report.intervals[1].exact_root, Some(fraction(1, 2)));
        assert_eq!(report.intervals[2].exact_root, Some(Real::one()));
        assert!(
            report
                .intervals
                .iter()
                .all(|interval| interval.distinct_root_count == 1)
        );
    }

    #[test]
    fn repeated_irrational_root_requests_the_complete_fallback() {
        // (x^2-2)^2 has one repeated irrational root in (1,2); bounded
        // division-free variation must not claim it is a simple isolator.
        let report = isolate_ordered_field_polynomial_roots(
            vec![
                Real::from(4),
                Real::zero(),
                Real::from(-4),
                Real::zero(),
                Real::one(),
            ],
            &Real::one(),
            &Real::from(2),
            OrderedFieldRootIsolationConfig {
                max_subdivision_depth: 4,
                refinement_steps: 0,
            },
            &mut RationalRealContext,
        )
        .expect("rational field operations are total");

        assert_eq!(
            report.status,
            OrderedFieldRootIsolationStatus::CompleteFallbackRequired
        );
        assert!(report.intervals.is_empty());
    }
}
