//! Lossless procedural source records; replay uses Hypercurve's strict admission.

use hypercurve::{
    BezierParallel2, BezierParallelSource2, BezierParameterRange2, CubicBezier2,
    HomogeneousControl2, QuadraticBezier2,
};

use super::*;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) enum RawParallelSource {
    Quadratic(Box<[[Real; 2]; 3]>),
    Cubic(Box<[[Real; 2]; 4]>),
    Rational {
        homogeneous_controls: Vec<[Real; 3]>,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) struct RawParallelPcurve {
    pub(super) source: RawParallelSource,
    pub(super) distance: Real,
    pub(super) range: [RawParameter; 2],
    pub(super) reversed: bool,
}

pub(super) fn raw_parallel_source(source: &BezierParallelSource2) -> RawParallelSource {
    match source {
        BezierParallelSource2::Quadratic(source) => {
            RawParallelSource::Quadratic(Box::new(source.control_points().map(curve_point_array)))
        }
        BezierParallelSource2::Cubic(source) => {
            RawParallelSource::Cubic(Box::new(source.control_points().map(curve_point_array)))
        }
        BezierParallelSource2::Rational(source) => RawParallelSource::Rational {
            homogeneous_controls: source
                .homogeneous_controls()
                .iter()
                .map(|control| {
                    [
                        control.x().clone(),
                        control.y().clone(),
                        control.weight().clone(),
                    ]
                })
                .collect(),
        },
    }
}

impl RawParallelPcurve {
    pub(super) fn into_curve(self, graph: &SelectionGraph) -> Result<Curve2, PersistenceError> {
        let source = match self.source {
            RawParallelSource::Quadratic(points) => {
                let [a, b, c] = points.map(curve_point2);
                BezierParallelSource2::Quadratic(QuadraticBezier2::new(a, b, c))
            }
            RawParallelSource::Cubic(points) => {
                let [a, b, c, d] = points.map(curve_point2);
                BezierParallelSource2::Cubic(CubicBezier2::new(a, b, c, d))
            }
            RawParallelSource::Rational {
                homogeneous_controls,
            } => {
                let source = crate::error::classified(RationalBezier2::from_homogeneous_controls(
                    homogeneous_controls
                        .into_iter()
                        .map(|[x, y, weight]| HomogeneousControl2::new(x, y, weight))
                        .collect(),
                ))
                .map_err(GeometryError::from)?;
                BezierParallelSource2::Rational(crate::geometry::resolve_planar_classification(
                    source,
                )?)
            }
        };
        let [start, end] = self.range;
        let start = graph.bezier_parameter(start)?;
        let end = graph.bezier_parameter(end)?;
        if start.cmp_by_refinement(&end).map_err(GeometryError::from)? != std::cmp::Ordering::Less {
            return Err(GeometryError::InvalidParameterDomain.into());
        }
        let range = BezierParameterRange2::try_new(start, end).map_err(GeometryError::from)?;
        // The archive stores a definition, never the original regularity proof.
        // This rechecks the complete interval, including selected endpoints.
        Ok(Curve2::try_analytic_parallel(
            BezierParallel2::from_source(source, self.distance),
            if self.reversed {
                range.reversed()
            } else {
                range
            },
        )
        .map_err(GeometryError::from)?)
    }
}
