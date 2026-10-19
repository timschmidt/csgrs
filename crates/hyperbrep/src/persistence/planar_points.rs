//! Checked planar source definitions embedded in the selected-parameter DAG.

use super::parallel::RawParallelSource;
use super::*;
use hypercurve::{CurvePointDefinition2, HomogeneousControl2, Similarity2};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawPlanarPoint {
    Coordinates([Real; 2]),
    Rational {
        homogeneous_controls: Vec<[Real; 3]>,
        parameter: RawParameter,
    },
    RationalExpression {
        parameter: RawParameter,
        x_numerator: Vec<Real>,
        y_numerator: Vec<Real>,
        denominator: Vec<Real>,
    },
    Parallel {
        source: RawParallelSource,
        distance: Real,
        parameter: RawParameter,
    },
    Similarity {
        source: Box<RawPlanarPoint>,
        coefficients: [Real; 6],
    },
    SupportingLineIntersection {
        first: Box<[RawPlanarPoint; 2]>,
        second: Box<[RawPlanarPoint; 2]>,
    },
}

impl GraphEncoder {
    pub(super) fn planar_point(
        &mut self,
        point: &hypercurve::CurvePoint2,
    ) -> Result<RawPlanarPoint, PersistenceError> {
        Ok(match point.definition().map_err(GeometryError::from)? {
            CurvePointDefinition2::Coordinates(point) => {
                RawPlanarPoint::Coordinates(curve_point_array(&point))
            }
            CurvePointDefinition2::Rational { source, parameter } => RawPlanarPoint::Rational {
                homogeneous_controls: source
                    .homogeneous_controls()
                    .iter()
                    .map(|c| [c.x().clone(), c.y().clone(), c.weight().clone()])
                    .collect(),
                parameter: self.curve_parameter(&parameter)?,
            },
            CurvePointDefinition2::RationalExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            } => RawPlanarPoint::RationalExpression {
                parameter: self.curve_parameter(&parameter.into())?,
                x_numerator,
                y_numerator,
                denominator,
            },
            CurvePointDefinition2::Parallel { source, parameter } => RawPlanarPoint::Parallel {
                source: raw_parallel_source(source.source()),
                distance: source.distance().clone(),
                parameter: self.curve_parameter(&parameter)?,
            },
            CurvePointDefinition2::Similarity { source, transform } => {
                let (a, b, c, d, x, y) = transform.affine_components();
                RawPlanarPoint::Similarity {
                    source: Box::new(self.planar_point(&source)?),
                    coefficients: [
                        a.clone(),
                        b.clone(),
                        c.clone(),
                        d.clone(),
                        x.clone(),
                        y.clone(),
                    ],
                }
            }
            CurvePointDefinition2::SupportingLineIntersection { first, second } => {
                RawPlanarPoint::SupportingLineIntersection {
                    first: Box::new([self.planar_point(&first[0])?, self.planar_point(&first[1])?]),
                    second: Box::new([
                        self.planar_point(&second[0])?,
                        self.planar_point(&second[1])?,
                    ]),
                }
            }
        })
    }
}

impl SelectionGraph {
    pub(super) fn planar_point(
        &self,
        raw: RawPlanarPoint,
    ) -> Result<hypercurve::CurvePoint2, PersistenceError> {
        let definition = match raw {
            RawPlanarPoint::Coordinates(point) => {
                CurvePointDefinition2::Coordinates(curve_point2(point))
            }
            RawPlanarPoint::Rational {
                homogeneous_controls,
                parameter,
            } => {
                let source = crate::geometry::resolve_planar_classification(
                    crate::error::classified(RationalBezier2::from_homogeneous_controls(
                        homogeneous_controls
                            .into_iter()
                            .map(|[x, y, w]| HomogeneousControl2::new(x, y, w))
                            .collect(),
                    ))
                    .map_err(GeometryError::from)?,
                )?;
                CurvePointDefinition2::Rational {
                    source,
                    parameter: self.curve_parameter(parameter)?,
                }
            }
            RawPlanarPoint::RationalExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            } => CurvePointDefinition2::RationalExpression {
                parameter: self.bezier_parameter(parameter)?,
                x_numerator,
                y_numerator,
                denominator,
            },
            RawPlanarPoint::Parallel {
                source,
                distance,
                parameter,
            } => CurvePointDefinition2::Parallel {
                source: hypercurve::BezierParallel2::from_source(source.into_source()?, distance),
                parameter: self.curve_parameter(parameter)?,
            },
            RawPlanarPoint::Similarity {
                source,
                coefficients: [a, b, c, d, x, y],
            } => CurvePointDefinition2::Similarity {
                source: self.planar_point(*source)?,
                transform: Similarity2::try_from_real_affine(a, b, c, d, x, y)
                    .map_err(GeometryError::from)?,
            },
            RawPlanarPoint::SupportingLineIntersection { first, second } => {
                let [a, b] = *first;
                let [c, d] = *second;
                CurvePointDefinition2::SupportingLineIntersection {
                    first: [self.planar_point(a)?, self.planar_point(b)?],
                    second: [self.planar_point(c)?, self.planar_point(d)?],
                }
            }
        };
        Ok(definition.into_point().map_err(GeometryError::from)?)
    }
}
