//! Exact linear and projected finite triangulation adapters for regions.
//!
//! [`triangulate_linear_contours`] retains native exact vertices. Finite entry
//! points consume explicitly projected vertices. In either case,
//! [`CurveRegion2`](crate::CurveRegion2) owns material/hole grouping and Hypertri
//! owns the ear-clipping predicates and triangulation certainty.

use crate::finite_projection::normalize_finite_ring_vertices;
use crate::{
    CurveCertainty, CurveContext, CurveError, CurveOutcome, CurveResult, FiniteRegionProfile2, Real,
};

/// A finite triangle emitted from a projected region profile.
///
/// The coordinates are projection-boundary `f64` values. Exact CAD topology
/// remains in [`crate::CurveRegion2`]; this type is intended for mesh generation,
/// rendering, and export layers.
pub type FiniteTriangle2 = [[f64; 2]; 3];

/// Triangulate native linear material/hole contours without quantizing vertices.
///
/// Hole ownership is supplied by the native profile. Curved segments are an
/// explicit unsupported representation. Hypertri owns the triangulation and
/// its predicate certainty; triangles retain the original exact Point2 values.
pub fn triangulate_linear_contours(
    context: &hypertri::TriangulationContext,
    material: &crate::Contour2,
    holes: &[crate::Contour2],
) -> hypertri::Result<hypertri::TriangulationOutcome<Vec<[crate::Point2; 3]>>> {
    let mut vertices = Vec::new();
    let mut hole_indices = Vec::new();
    for (i, ring) in std::iter::once(material).chain(holes).enumerate() {
        if i != 0 {
            hole_indices.push(vertices.len());
        }
        for segment in ring.segments() {
            let crate::Segment2::Line(line) = segment else {
                return Err(hypertri::Error::UnsupportedFeature {
                    feature: "nonlinear contour triangulation",
                });
            };
            vertices.push(line.start().clone());
        }
    }
    let points = vertices
        .iter()
        .map(|p| hypertri::Point2::new(p.x().clone(), p.y().clone()))
        .collect::<Vec<_>>();
    let outcome = hypertri::earcut(context, &points, &hole_indices)?;
    if !outcome.value.len().is_multiple_of(3) {
        return Err(hypertri::Error::InvalidInput {
            reason: "triangulation returned incomplete triangle indices",
        });
    }
    let triangles = outcome
        .value
        .as_chunks::<3>()
        .0
        .iter()
        .map(|indices| {
            let [a, b, c] = indices.map(|i| vertices.get(i).cloned());
            match (a, b, c) {
                (Some(a), Some(b), Some(c)) => Ok([a, b, c]),
                _ => Err(hypertri::Error::InvalidInput {
                    reason: "triangulation returned an out-of-range vertex index",
                }),
            }
        })
        .collect::<hypertri::Result<Vec<_>>>()?;
    Ok(hypertri::TriangulationOutcome {
        value: triangles,
        certainty: outcome.certainty,
    })
}

/// Triangulates a finite material ring with owned finite hole rings.
///
/// This function is the low-level adapter for consumers that already hold
/// projected profile rings. It normalizes repeated adjacent and closing
/// vertices, lifts finite coordinates into hyperreal-backed hypertri points,
/// and returns finite triangles by index into the normalized boundary vertices.
/// Exact predicate decisions happen in hypertri rather than in downstream
/// crates.
pub fn triangulate_finite_rings(
    material: &[[f64; 2]],
    holes: &[&[[f64; 2]]],
) -> crate::ExactCurveResult<Vec<FiniteTriangle2>> {
    triangulate_finite_rings_with_policy(material, holes, &crate::policy::principal_context())
        .map(CurveOutcome::into_value)
        .map_err(|cause| {
            crate::ExactCurveError::invalid_unattributed(crate::CurveOperation2::Subdivision, cause)
        })
}

/// [`triangulate_finite_rings`] under an explicit predicate policy.
pub(crate) fn triangulate_finite_rings_with_policy(
    material: &[[f64; 2]],
    holes: &[&[[f64; 2]]],
    policy: &CurveContext,
) -> CurveResult<CurveOutcome<Vec<FiniteTriangle2>>> {
    fn push_ring(
        ring: &[[f64; 2]],
        vertices: &mut Vec<[f64; 2]>,
        exact: &mut Vec<hypertri::Point2>,
    ) -> CurveResult<Option<usize>> {
        let normalized = normalize_finite_ring_vertices(ring)?;
        if normalized.len() < 3 {
            return Ok(None);
        }
        validate_no_repeated_ring_vertices(&normalized)?;

        let start = vertices.len();
        for [x, y] in normalized {
            vertices.push([x, y]);
            exact.push(hypertri::Point2::new(
                Real::try_from(x).map_err(|err| CurveError::Real(err.to_string()))?,
                Real::try_from(y).map_err(|err| CurveError::Real(err.to_string()))?,
            ));
        }
        Ok(Some(start))
    }

    let mut vertices = Vec::new();
    let mut exact = Vec::new();
    if push_ring(material, &mut vertices, &mut exact)?.is_none() {
        return Ok(CurveOutcome::new(Vec::new(), CurveCertainty::Certified));
    }

    let mut hole_indices = Vec::with_capacity(holes.len());
    for hole in holes {
        if let Some(start) = push_ring(hole, &mut vertices, &mut exact)? {
            hole_indices.push(start);
        }
    }

    let context = hypertri::TriangulationContext::new(policy.predicate_policy());
    let outcome = hypertri::earcut(&context, &exact, &hole_indices)
        .map_err(|err| CurveError::Topology(err.to_string()))?;
    let certainty = match outcome.certainty {
        hypertri::TriangulationCertainty::Certified => CurveCertainty::Certified,
        hypertri::TriangulationCertainty::Approximate512Consumed => {
            CurveCertainty::Approximate512Consumed
        }
    };
    triangles_from_indices(&vertices, &outcome.value)
        .map(|triangles| CurveOutcome::new(triangles, certainty))
}

fn triangles_from_indices(
    vertices: &[[f64; 2]],
    indices: &[usize],
) -> CurveResult<Vec<FiniteTriangle2>> {
    if !indices.len().is_multiple_of(3) {
        return Err(CurveError::Topology(
            "finite triangulation returned an incomplete triangle index group".into(),
        ));
    }
    indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|triangle| {
            Ok([
                *vertices.get(triangle[0]).ok_or_else(|| {
                    CurveError::Topology(
                        "finite triangulation returned a vertex index outside its profile".into(),
                    )
                })?,
                *vertices.get(triangle[1]).ok_or_else(|| {
                    CurveError::Topology(
                        "finite triangulation returned a vertex index outside its profile".into(),
                    )
                })?,
                *vertices.get(triangle[2]).ok_or_else(|| {
                    CurveError::Topology(
                        "finite triangulation returned a vertex index outside its profile".into(),
                    )
                })?,
            ])
        })
        .collect()
}

fn validate_no_repeated_ring_vertices(ring: &[[f64; 2]]) -> CurveResult<()> {
    for (index, point) in ring.iter().enumerate() {
        if ring[index + 1..].iter().any(|candidate| candidate == point) {
            return Err(CurveError::Topology(
                "finite triangulation ring must not contain repeated non-adjacent vertices".into(),
            ));
        }
    }
    Ok(())
}

impl FiniteRegionProfile2 {
    /// Triangulates this projected material-with-holes profile.
    ///
    /// Hole ownership was decided by hypercurve before this finite profile was
    /// built. The triangulation stage therefore receives a topology-preserving
    /// profile record rather than a bag of rings whose roles must be recovered
    /// from winding. Earcut-style triangulation is handled by hypertri using
    /// exact hyperreal predicates; see ear clipping and the exactness model, cited in
    /// the module documentation.
    pub fn triangulate(&self) -> crate::ExactCurveResult<Vec<FiniteTriangle2>> {
        self.triangulate_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
            .map_err(|cause| {
                crate::ExactCurveError::invalid_unattributed(
                    crate::CurveOperation2::Subdivision,
                    cause,
                )
            })
    }

    /// [`Self::triangulate`] under an explicit predicate policy.
    pub(crate) fn triangulate_with_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Vec<FiniteTriangle2>>> {
        let hole_refs = self
            .holes()
            .iter()
            .map(|hole| hole.points())
            .collect::<Vec<_>>();
        triangulate_finite_rings_with_policy(self.material().points(), &hole_refs, policy)
    }
}

#[cfg(test)]
mod tests {
    use super::triangles_from_indices;

    #[test]
    fn invalid_triangulation_indices_are_not_silently_dropped() {
        let vertices = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
        assert!(triangles_from_indices(&vertices, &[0, 1]).is_err());
        assert!(triangles_from_indices(&vertices, &[0, 1, 3]).is_err());
    }
}
