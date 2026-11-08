//! Guarded constant-image loci in the original selected graph chart.
use super::selected_chord::form;
use super::*;
use crate::{CurveLocationComponent2, CurveParameterSet2};
use hypersolve::bivariate_topology::bivariate_polynomial_gcd;
use hypersolve::curve_resultant::{
    CurveIntersectionResultantConfig, CurveIntersectionResultantStatus,
    resultant_bivariate_polynomial_system_complete,
};

impl PolynomialCurveBranch2 {
    /// `None` proves that the extra guard vanishes on the whole selected arc.
    /// Otherwise every zero is checked on the original branch and active trim.
    fn map_denominator_zeros(
        &self,
        map: &crate::RationalMap2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Vec<CurveParameter2>>> {
        let (guard, weight) = self
            .pullback_form(
                map.denominator().clone(),
                hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
            )
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let equation = self.topology().geometric_equation();
        let common = bivariate_polynomial_gcd(equation, &guard, self.topology().config()).map_err(
            |error| {
                blocked(match error {
                    BivariateTopologyError::BudgetExhausted
                    | BivariateTopologyError::DegreeLimit
                    | BivariateTopologyError::RefinementLimit => UncertaintyReason::BudgetExhausted,
                    BivariateTopologyError::UnsupportedCoefficient => {
                        UncertaintyReason::Unsupported
                    }
                    _ => UncertaintyReason::Predicate,
                })
            },
        )?;
        // The existing checked simple-root slab prevents this graph from
        // changing between coprime factors. This is a factor-selection proof,
        // not a claim inferred from an arbitrary point sample.
        if self.selects_component(&common, policy)? {
            return Ok(None);
        }
        let residual = hypersolve::divide_bivariate_polynomial_exact(equation, &common)
            .ok_or_else(|| blocked(UncertaintyReason::Predicate))?;
        let report = resultant_bivariate_polynomial_system_complete(
            &residual,
            &guard,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                max_resultant_degree: self.topology().config().max_projection_degree,
                ..Default::default()
            },
        );
        match report.status {
            CurveIntersectionResultantStatus::Constructed => {}
            CurveIntersectionResultantStatus::BudgetExhausted
            | CurveIntersectionResultantStatus::DegreeBoundExceeded => {
                return Err(blocked(UncertaintyReason::BudgetExhausted));
            }
            CurveIntersectionResultantStatus::UndecidedCoefficient => {
                return Err(blocked(UncertaintyReason::Predicate));
            }
            _ => return Err(blocked(UncertaintyReason::Unsupported)),
        }
        let source = Curve2::from_polynomial_branch(self.source_branch(policy)?);
        let mut zeros = Vec::new();
        for mut parameter in self.section_abscissas(report.resultant_coefficients)? {
            let low = decided(
                parameter
                    .cmp_by_refinement(self.range.start(), policy)
                    .map_err(invalid)?,
            )?;
            let high = decided(
                parameter
                    .cmp_by_refinement(self.range.end(), policy)
                    .map_err(invalid)?,
            )?;
            if low.is_lt() || high.is_gt() {
                continue;
            }
            if low.is_eq() {
                parameter = self.range.start().clone();
            }
            if high.is_eq() {
                parameter = self.range.end().clone();
            }
            let point = source
                .point_at_with_policy(&parameter, policy)?
                .into_value();
            // Pullback clearing is not cancellation of an authored guard.
            if form(&point, weight.clone())?.compare_to_real(&Real::zero())? == Ordering::Equal {
                return Err(blocked(UncertaintyReason::Unsupported));
            }
            if form(&point, guard.clone())?.compare_to_real(&Real::zero())? == Ordering::Equal {
                zeros.push(parameter);
            }
        }
        Ok(Some(zeros))
    }

    /// Requires the caller's exact zero-tangent-derivative proof. A complete
    /// denominator census supplies the defined domain; one valid witness then
    /// identifies the constant, including independent selected query points.
    pub(super) fn constant_mapped_point_locations(
        &self,
        point: &CurvePoint2,
        map: &crate::RationalMap2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let Some(zeros) = self.map_denominator_zeros(map, policy)? else {
            return Ok(CurvePointLocations2::Locations(Vec::new()));
        };
        let chart = CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero());
        let mut components = Vec::new();
        let mut start = self.range.start().clone();
        let mut included = true;
        for (end, end_included) in zeros
            .iter()
            .map(|p| (p, false))
            .chain(std::iter::once((self.range.end(), true)))
        {
            if decided(start.cmp_by_refinement(end, policy).map_err(invalid)?)?.is_lt() {
                components.push(CurveLocationComponent2::new(
                    0,
                    chart.clone(),
                    CurveParameterSet2::Range(CurveParameterRange2::new_validated(
                        start.clone(),
                        end.clone(),
                    )),
                    [included, end_included],
                ));
            }
            start = end.clone();
            included = false;
        }
        let Some(first) = components.first() else {
            return Ok(CurvePointLocations2::Locations(Vec::new()));
        };
        let CurveParameterSet2::Range(range) = first.local_parameters() else {
            unreachable!("a nonempty open interval supplies the witness")
        };
        let witness = decided(range.strict_interior_scalar(policy).map_err(invalid)?)?;
        let image = self
            .point_at(&witness.into(), policy)?
            .rational_image_with_policy(map, policy)?;
        if !decided(image.same_point(point, policy))? {
            return Ok(CurvePointLocations2::Locations(Vec::new()));
        }
        Ok(if zeros.is_empty() {
            CurvePointLocations2::EntireCurve
        } else {
            CurvePointLocations2::Components(components)
        })
    }
}
