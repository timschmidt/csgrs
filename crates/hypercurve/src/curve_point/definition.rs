//! Replayable source definitions, without serialized certainty claims.

use super::*;
use crate::{
    BezierParallel2, BezierParameter2, CurveFamily2, CurveOperation2, CurveParameter2,
    ExactCurveError, ExactCurveResult, RationalBezier2, Similarity2,
};

/// Source definition of an exact point. Selected parameters retain their
/// original authority. Unsupported constructions have no such view yet.
#[derive(Clone, Debug)]
pub enum CurvePointDefinition2 {
    /// Authored exact coordinates.
    Coordinates(Point2),
    /// A rational curve evaluated at a retained parameter.
    Rational {
        source: RationalBezier2,
        parameter: CurveParameter2,
    },
    /// Correlated rational coordinates in one retained selected parameter.
    RationalExpression {
        /// Original selected parameter, shared by both coordinates.
        parameter: BezierParameter2,
        /// Ascending power coefficients of the x numerator.
        x_numerator: Vec<crate::Real>,
        /// Ascending power coefficients of the y numerator.
        y_numerator: Vec<crate::Real>,
        /// Shared ascending power denominator; replay proves it nonzero.
        denominator: Vec<crate::Real>,
    },
    /// A regular analytic parallel evaluated at a retained parameter.
    Parallel {
        source: BezierParallel2,
        parameter: CurveParameter2,
    },
    /// An exact similarity of a retained point.
    Similarity {
        source: CurvePoint2,
        transform: Similarity2,
    },
}

impl CurvePoint2 {
    /// Returns a checked source definition, without eliminating selected roots.
    /// Construction policies incompatible with strict replay remain blocked.
    pub fn definition(&self) -> ExactCurveResult<CurvePointDefinition2> {
        let blocked = || {
            ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        };
        match &self.0 {
            CurvePointData2::Exact(point) => Ok(CurvePointDefinition2::Coordinates(point.clone())),
            CurvePointData2::Algebraic(point) => {
                if let Some((source, parameter)) = point.native_curve_evaluation() {
                    return Ok(CurvePointDefinition2::Rational {
                        source: source.clone(),
                        parameter: BezierParameter2::Algebraic(parameter.clone()).into(),
                    });
                }
                let parameter = point.retained_parameter().ok_or_else(blocked)?;
                let (x, y, w) = point
                    .retained_coordinate_polynomials()
                    .ok_or_else(blocked)?;
                Ok(CurvePointDefinition2::RationalExpression {
                    parameter: BezierParameter2::Algebraic(parameter.clone()),
                    x_numerator: x.to_vec(),
                    y_numerator: y.to_vec(),
                    denominator: w.to_vec(),
                })
            }
            CurvePointData2::AnalyticParallel(point) => {
                let (source, parameter) = point.native_parallel_evaluation().ok_or_else(blocked)?;
                Ok(CurvePointDefinition2::Parallel {
                    source: source.clone(),
                    parameter,
                })
            }
            CurvePointData2::Similarity(point) => {
                let (source, transform) = point.strict_definition().ok_or_else(blocked)?;
                Ok(CurvePointDefinition2::Similarity {
                    source: source.clone(),
                    transform: transform.clone(),
                })
            }
            CurvePointData2::Endpoint(endpoint) => {
                let point = ExactCurveError::decided(
                    CurveOperation2::Evaluation,
                    endpoint.resolve(&CurveContext::STRICT).map_err(|cause| {
                        ExactCurveError::invalid_unattributed(CurveOperation2::Evaluation, cause)
                    })?,
                )?
                .ok_or_else(blocked)?;
                point.definition()
            }
            _ => Err(blocked()),
        }
    }
}

impl CurvePointDefinition2 {
    /// Replays the source with pole and regular-normal checks under strict
    /// predicates. The input definition carries no trusted validation flags.
    pub fn into_point(self) -> ExactCurveResult<CurvePoint2> {
        crate::policy::resolve_certified_operation(&CurveContext::STRICT, |policy| match self {
            Self::Coordinates(point) => Ok(point.into()),
            Self::Rational { source, parameter } => crate::curve::curve_evaluation::rational_point(
                &source,
                &parameter,
                CurveFamily2::RationalBezier,
                policy,
            ),
            Self::RationalExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            } => {
                let invalid = |cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::RationalBezier,
                        cause,
                    )
                };
                if x_numerator.is_empty() || y_numerator.is_empty() || denominator.is_empty() {
                    return Err(invalid(crate::CurveError::InvalidCurveParameter));
                }
                let sign = ExactCurveError::decided(
                    CurveOperation2::Evaluation,
                    crate::bezier_parameter::signed_coefficients_at_parameter(
                        &denominator,
                        &parameter,
                        policy,
                    )
                    .map_err(invalid)?,
                )?;
                if sign == hyperreal::RealSign::Zero {
                    return Err(invalid(crate::CurveError::InvalidCurveParameter));
                }
                match parameter {
                    BezierParameter2::Exact(t) => {
                        let evaluate = |coefficients: &[crate::Real]| {
                            coefficients
                                .iter()
                                .rev()
                                .fold(crate::Real::zero(), |sum, c| sum * &t + c)
                        };
                        let w = evaluate(&denominator);
                        Ok(Point2::new(
                            (evaluate(&x_numerator) / &w).map_err(|e| invalid(e.into()))?,
                            (evaluate(&y_numerator) / &w).map_err(|e| invalid(e.into()))?,
                        )
                        .into())
                    }
                    BezierParameter2::Algebraic(parameter) => Ok(
                        RationalBezierAlgebraicPointImage2::from_retained_expression(
                            parameter.clone(),
                            crate::bezier_algebraic_image::parameter_representation(
                                &parameter, policy,
                            ),
                            x_numerator,
                            y_numerator,
                            denominator,
                            "replayed a non-pole correlated rational point definition",
                        )
                        .into(),
                    ),
                }
            }
            Self::Parallel { source, parameter } => source.checked_native_point(&parameter, policy),
            Self::Similarity { source, transform } => {
                // Replaying the definition establishes its source under the
                // same strict policy; an arbitrary incompatible source cannot
                // acquire a stronger claim merely by being wrapped.
                source.definition()?.into_point()?;
                Ok(crate::BezierSimilarityPoint2::new(source, transform, policy).into())
            }
        })
        .map(crate::CurveOutcome::into_value)
    }
}
