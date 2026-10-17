//! Exact incidence using an already retained circular support and chart.
use super::*;

impl RationalBezier2 {
    /// The caller has proved the requested range lies in the finite unit chart.
    /// A failed extra proof leaves the general incidence authority available.
    pub(super) fn retained_circle_incidence_on_unit_subrange(
        &self,
        point: &Point2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierPointIncidence2>> {
        let Some(circle) = self.retained_circular_conic() else {
            return Ok(None);
        };
        if self.degree() != 2
            && self
                .data
                .lineage
                .root
                .quadratic_conic_parameter_frame
                .get()
                .is_none()
        {
            return Ok(None);
        }
        let strict = policy.strict_counterpart();
        match is_zero(
            &(point.distance_squared(&circle.center) - &circle.radius_squared),
            &strict,
        ) {
            Some(false) => return Ok(Some(RationalBezierPointIncidence2::Parameters(Vec::new()))),
            Some(true) => {}
            None => return Ok(None),
        }
        // A nonconstant quadratic lying on a circle has a nondegenerate
        // projective frame: a degenerate frame lies on a line, whose circle
        // intersection is finite. Distinct endpoints prove nonconstancy and
        // also exclude a collapsed source range after subdivision/elevation.
        // A circle certificate alone does not prove a higher-degree chart is
        // injective; that case needs the retained quadratic parameter frame.
        if is_zero(&self.start().distance_squared(self.end()), &strict) != Some(false) {
            return Ok(None);
        }
        let Classification::Decided(parameters) =
            self.retained_circle_point_parameters(point, &strict)?
        else {
            return Ok(None);
        };
        let domain = CurveParameterDomain2::new(range, None);
        let mut retained = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            match domain.contains_finite_parameter(&parameter.clone().into(), &strict)? {
                Classification::Decided(true) => retained.push(parameter),
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => return Ok(None),
            }
        }
        Ok(Some(RationalBezierPointIncidence2::Parameters(retained)))
    }
}
