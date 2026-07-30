//! Runtime `f64` triangulation entry points.

#[cfg(feature = "cdt")]
use crate::cdt::{ConstrainedDelaunayTriangulation, DelaunayTriangulation};
use crate::context::{TriangulationContext, TriangulationOutcome};
#[cfg(any(feature = "earcut", feature = "cdt"))]
use crate::error::{Error, Result};
#[cfg(feature = "cdt")]
use crate::types::Constraint;
#[cfg(feature = "earcut")]
use crate::types::TriangleIndices;
#[cfg(any(feature = "earcut", feature = "cdt"))]
use crate::types::{ExactPoint, Point2, Real};

/// Triangulate finite `f64` polygon input after exact dyadic lifting.
///
/// `hyperreal::Real::try_from(f64)` decodes IEEE-754 values as exact dyadic
/// rationals. This boundary therefore preserves the caller's finite binary
/// input exactly instead of making topology decisions in floating arithmetic.
#[cfg(feature = "earcut")]
pub fn earcut(
    context: &TriangulationContext,
    vertices: &[[f64; 2]],
    hole_indices: &[usize],
) -> Result<TriangulationOutcome<TriangleIndices>> {
    crate::earcut::triangulate(context, &lift_vertices(vertices)?, hole_indices)
}

/// Triangulate finite `f64` points after exact dyadic lifting.
#[cfg(feature = "cdt")]
pub fn delaunay(
    context: &TriangulationContext,
    points: &[[f64; 2]],
) -> Result<TriangulationOutcome<DelaunayTriangulation>> {
    validate_f64_vertices(points)?;
    let exact = lift_vertices(points)?;
    crate::cdt::delaunay(context, &exact)
}

/// Triangulate finite `f64` points with the BRIO-style batch schedule after
/// exact dyadic lifting. See [`crate::cdt::delaunay_spatial`] for tie behavior.
#[cfg(feature = "cdt")]
pub fn delaunay_spatial(
    context: &TriangulationContext,
    points: &[[f64; 2]],
) -> Result<TriangulationOutcome<DelaunayTriangulation>> {
    validate_f64_vertices(points)?;
    let exact = lift_vertices(points)?;
    crate::cdt::delaunay_spatial(context, &exact)
}

/// Triangulate finite `f64` points with constraints after exact dyadic lifting.
#[cfg(feature = "cdt")]
pub fn constrained_delaunay(
    context: &TriangulationContext,
    points: &[[f64; 2]],
    constraints: &[Constraint],
) -> Result<TriangulationOutcome<ConstrainedDelaunayTriangulation>> {
    validate_f64_vertices(points)?;
    validate_constraints(points.len(), constraints)?;
    let exact = lift_vertices(points)?;
    crate::cdt::constrained_delaunay(context, &exact, constraints)
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn validate_f64_vertices(vertices: &[[f64; 2]]) -> Result<()> {
    for point in vertices {
        if !point[0].is_finite() || !point[1].is_finite() {
            return Err(Error::InvalidInput {
                reason: "f64 coordinates must be finite",
            });
        }
    }

    Ok(())
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn lift_vertices(vertices: &[[f64; 2]]) -> Result<Vec<ExactPoint>> {
    validate_f64_vertices(vertices)?;
    vertices
        .iter()
        .map(|point| Ok(Point2::new(lift_real(point[0])?, lift_real(point[1])?)))
        .collect()
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn lift_real(value: f64) -> Result<Real> {
    Real::try_from(value).map_err(|_| Error::InvalidInput {
        reason: "f64 coordinates must be finite",
    })
}

#[cfg(feature = "cdt")]
fn validate_constraints(point_count: usize, constraints: &[Constraint]) -> Result<()> {
    for constraint in constraints {
        if constraint.from >= point_count || constraint.to >= point_count {
            return Err(Error::InvalidInput {
                reason: "constraint index out of bounds",
            });
        }
        if constraint.from == constraint.to {
            return Err(Error::InvalidInput {
                reason: "constraint endpoints must differ",
            });
        }
    }

    Ok(())
}

#[cfg(all(test, feature = "earcut"))]
mod tests {
    use super::*;

    const APPROX: TriangulationContext =
        TriangulationContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);

    #[cfg(feature = "earcut")]
    #[test]
    fn f64_earcut_accepts_plain_runtime_points() {
        let vertices = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];

        let triangles = earcut(&APPROX, &vertices, &[]).unwrap().value;

        assert_eq!(triangles.len(), 6);
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn f64_rejects_nan() {
        let vertices = [[0.0, 0.0], [f64::NAN, 0.0], [1.0, 1.0]];

        let error = earcut(&APPROX, &vertices, &[]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "f64 coordinates must be finite"
            }
        );
    }
}
