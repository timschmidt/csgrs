//! Enclosures of a connected selected graph, proved without display samples.

use super::*;
use hypersolve::RealInterval;

fn compare(a: &Real, b: &Real, policy: &CurveContext) -> ExactCurveResult<Ordering> {
    crate::classify::compare_reals(a, b, policy)
        .ok_or_else(|| blocked(UncertaintyReason::Predicate))
}

impl PolynomialCurveBranch2 {
    /// Bounds the connected source subarc inside the supplied abscissa interval.
    /// The anchor must lie strictly inside this branch. A missing box means
    /// subdivision is needed, not that the curve is absent or unbounded.
    pub(crate) fn projection_enclosure(
        &self,
        first: [&Real; 2],
        anchor: &Real,
        margin: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Option<Aabb2>, CurvePoint2)> {
        hypersolve::work_budget::consume(hypersolve::work_budget::WorkKind::RootRefinement, 1)
            .map_err(|_| blocked(UncertaintyReason::BudgetExhausted))?;
        let parameter: CurveParameter2 = anchor.clone().into();
        if !decided(
            parameter
                .cmp_by_refinement(self.range.start(), policy)
                .map_err(invalid)?,
        )?
        .is_gt()
            || !decided(
                parameter
                    .cmp_by_refinement(self.range.end(), policy)
                    .map_err(invalid)?,
            )?
            .is_lt()
            || !compare(first[0], anchor, policy)?.is_lt()
            || !compare(anchor, first[1], policy)?.is_lt()
            || !compare(margin, &Real::zero(), policy)?.is_gt()
        {
            return Err(invalid(CurveError::InvalidCurveParameter));
        }
        let source_point = self.interior_point(&parameter, policy)?;
        let image_point = self.map_point(source_point.clone(), policy)?;
        let mut point_bounds = None;
        for steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let bounds = match crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                &source_point,
                steps,
                policy,
            ) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                    return Err(blocked(UncertaintyReason::BudgetExhausted));
                }
                Classification::Uncertain(_) => continue,
            };
            if !compare(&(bounds.max().y() - bounds.min().y()), margin, policy)?.is_gt() {
                point_bounds = Some(bounds);
                break;
            }
        }
        let point = point_bounds.ok_or_else(|| blocked(UncertaintyReason::BudgetExhausted))?;
        let domain = self.topology().domain();
        let lower = if compare(first[0], &domain.first[0], policy)?.is_lt() {
            domain.first[0].clone()
        } else {
            first[0].clone()
        };
        let upper = if compare(first[1], &domain.first[1], policy)?.is_gt() {
            domain.first[1].clone()
        } else {
            first[1].clone()
        };
        let x = RealInterval { lower, upper };
        let anchor_x = RealInterval {
            lower: anchor.clone(),
            upper: anchor.clone(),
        };
        let equation = self.topology().equation();
        // At most twice the ordinate degree can make an edge pass through
        // another root of the anchor fiber. The admitted finite fiber is not
        // identically zero. Try distinct positive margins inside the requested
        // band; no nearby roots are merged or removed.
        let degree = equation
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        let choices = degree
            .checked_mul(2)
            .and_then(|n| n.checked_add(1))
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        for index in (1..=choices).rev() {
            let width = (margin * Real::from(index as u128)
                / Real::from(
                    choices
                        .checked_add(1)
                        .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?
                        as u128,
                ))
            .map_err(|error| invalid(error.into()))?;
            let mut bottom = point.min().y() - &width;
            let mut top = point.max().y() + &width;
            let lower_guard = compare(&bottom, &domain.second[0], policy)?.is_gt();
            let upper_guard = compare(&top, &domain.second[1], policy)?.is_lt();
            if !lower_guard {
                bottom = domain.second[0].clone();
            }
            if !upper_guard {
                top = domain.second[1].clone();
            }
            let edge_sign = |axis: &RealInterval, ordinate: &Real| {
                RealInterval::evaluate_bivariate_power_basis(
                    equation,
                    axis,
                    &RealInterval {
                        lower: ordinate.clone(),
                        upper: ordinate.clone(),
                    },
                )
                .and_then(|value| value.strict_nonzero_sign())
            };
            if (lower_guard && edge_sign(&anchor_x, &bottom).is_none())
                || (upper_guard && edge_sign(&anchor_x, &top).is_none())
            {
                continue;
            }
            if (lower_guard && edge_sign(&x, &bottom).is_none())
                || (upper_guard && edge_sign(&x, &top).is_none())
            {
                return Ok((None, image_point));
            }
            // The selected graph is continuous and contains the anchor. It
            // cannot leave this strip without meeting one of its horizontal
            // edges. Domain-clamped edges use the original topology bound.
            let bounds = Aabb2::new_unchecked(
                Point2::new(x.lower.clone(), bottom),
                Point2::new(x.upper.clone(), top),
            );
            return match self.image_box(bounds) {
                Ok(bounds) => Ok((Some(bounds), image_point)),
                Err(ExactCurveError::Blocked(error))
                    if error.reason() != UncertaintyReason::BudgetExhausted =>
                {
                    Ok((None, image_point))
                }
                Err(error) => Err(error),
            };
        }
        Ok((None, image_point))
    }
}
