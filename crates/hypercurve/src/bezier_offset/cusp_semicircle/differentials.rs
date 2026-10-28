//! Differentials use the original circle chart and the selected point's field.

use super::*;
use crate::bezier_algebraic_image::map_rational_derivatives_from_power_basis;
use crate::{CurveFamily2, CurveOperation2, CurveVector2, ExactCurveError, ExactCurveResult};
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale, bivariate_subtract};

fn invalid(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Evaluation,
        CurveFamily2::RationalBezier,
        cause,
    )
}

fn decided<T>(value: Classification<T>) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

fn unsupported() -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Evaluation,
        CurveFamily2::RationalBezier,
        UncertaintyReason::Unsupported,
    )
}

fn lift(coefficients: Vec<Real>) -> BivariatePolynomial {
    BivariatePolynomial::new(coefficients.into_iter().map(|value| vec![value]).collect())
}

impl BezierAlgebraicCuspSemicircleFragment2 {
    pub(crate) fn derivatives_at_parameter(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveVector2>> {
        self.validate_policy(policy).map_err(invalid)?;
        let local = parameter
            .as_algebraic_cusp()
            .filter(|_| !parameter.is_algebraic_cusp_complement())
            .ok_or_else(|| invalid(CurveError::InvalidCurveParameter))?;
        local.validate_policy(policy).map_err(invalid)?;
        let circle = self.semicircle();
        let (nx, ny) = circle
            .represented_frame_unit_normal()
            .map_err(invalid)?
            .ok_or_else(unsupported)?;
        // The center is constant in this chart, even if its coordinates and
        // the evaluation parameter depend on the same selected field.
        let rx = circle.radial_distance() * &nx;
        let ry = circle.radial_distance() * &ny;
        let source = decided(
            RationalBezier2::from_homogeneous_controls_with_policy(
                vec![
                    crate::HomogeneousControl2::new(rx.clone(), ry.clone(), Real::one()),
                    crate::HomogeneousControl2::new(
                        -&ry * circle.turn_sign(),
                        &rx * circle.turn_sign(),
                        Real::zero(),
                    ),
                    crate::HomogeneousControl2::new(-rx, -ry, Real::one()),
                ],
                policy,
            )
            .map_err(invalid)?,
        )?;
        let angular = match local {
            BezierAlgebraicCuspSemicircleParameter2::Exact(value) => PointFormSource2::new(
                Point2::new(value.clone(), Real::zero()).into(),
                lift(vec![Real::zero(), Real::one()]),
                lift(vec![Real::one()]),
            ),
            _ => circle.angular_point_form(local, (&nx, &ny), policy)?,
        };
        // All derivative orders share this guarded value substitution. Keep
        // it as a point map instead of expanding its denominator into every
        // derivative polynomial; the original field and map guards survive
        // both interval evaluation and native replay.
        let angular_point = angular
            .source
            .rational_image(&crate::RationalMap2::new(
                [angular.numerator, BivariatePolynomial::new(Vec::new())],
                angular.denominator,
            ))
            .map_err(|error| error.with_operation(CurveOperation2::Evaluation))?;
        let retained_policy = policy.retained_object_policy_with_dependencies(
            [Some(self.data.policy), local.evidence_policy()]
                .into_iter()
                .flatten(),
        );
        let identity = Similarity2::try_from_real_affine(
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
        )
        .map_err(invalid)?;
        let chart = source.homogeneous_power_basis().map_err(invalid)?;
        decided(map_rational_derivatives_from_power_basis(
            chart.x_numerator.clone(),
            chart.y_numerator.clone(),
            chart.weight.clone(),
            max_order,
            Ok,
            |derivative| {
                let point = angular_point
                    .rational_image(&crate::RationalMap2::new(
                        [lift(derivative.dx_numerator), lift(derivative.dy_numerator)],
                        lift(derivative.denominator),
                    ))
                    .map_err(|error| error.with_operation(CurveOperation2::Evaluation))?;
                // A constant center can cancel from a derivative without
                // upgrading the certainty of the frame that selected it.
                Ok(Classification::Decided(CurveVector2::joint(
                    BezierSimilarityPoint2::new(point, identity.clone(), &retained_policy).into(),
                )))
            },
        )?)
    }
}

impl BezierAlgebraicCuspSemicircle2 {
    fn angular_point_form(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        (nx, ny): (&Real, &Real),
        policy: &CurveContext,
    ) -> ExactCurveResult<PointFormSource2> {
        let strict = policy.strict_counterpart();
        let computed;
        let radial = if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint {
                semicircle,
                radial,
                ..
            } = data.as_ref()
            && semicircle == self
        {
            // Incidence already retained this common field; do not repeat
            // coordinate alignment or rebuild independently selected roots.
            radial.as_ref()
        } else {
            let point = decided(
                parameter
                    .coincident_point_evidence(self, policy)
                    .map_err(invalid)?,
            )?
            .ok_or_else(unsupported)?;
            let center = decided(self.center_point_evidence(policy).map_err(invalid)?)?;
            computed =
                decided(common_point_displacement(&point, &center, &strict).map_err(invalid)?)?;
            &computed
        };
        for form in radial {
            decided(
                point_rational_denominator_sign(&form.source, &form.denominator, &strict)
                    .map_err(invalid)?,
            )?;
        }
        let normal = point_form_linear_combination(radial, nx, ny);
        let tangent = point_form_linear_combination(
            radial,
            &(-self.turn_sign() * ny),
            &(self.turn_sign() * nx),
        );
        // These linear projections share the same radial coordinate denominator.
        debug_assert_eq!(normal.denominator, tangent.denominator);
        let radius = bivariate_scale(normal.denominator.clone(), self.radial_distance());
        // u=b/(r+n+b). The second inverse covers the diameter endpoint u=1,
        // where the first has 0/0. Incidence and branch ownership were retained
        // by the parameter; every original radial guard is checked above.
        let mut numerator = tangent.numerator.clone();
        let mut denominator = bivariate_add(&bivariate_add(&radius, &normal.numerator), &numerator);
        if decided(
            point_rational_form_sign(&normal.source, &denominator, &normal.denominator, &strict)
                .map_err(invalid)?,
        )? == RealSign::Zero
        {
            numerator = bivariate_subtract(&radius, &normal.numerator);
            denominator = bivariate_add(&numerator, &tangent.numerator);
        }
        decided(
            point_rational_denominator_sign(&normal.source, &denominator, &strict)
                .map_err(invalid)?,
        )?;
        Ok(PointFormSource2::new(normal.source, numerator, denominator))
    }
}
