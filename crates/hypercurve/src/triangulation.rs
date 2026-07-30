//! Finite profile triangulation adapters for hypercurve regions.
//!
//! Triangulation consumes projected boundary vertices, but the ownership of
//! material and hole rings is decided before projection by [`LineArcRegion2`] and
//! [`RegionView2`](crate::RegionView2). Keeping the profile grouping in
//! hypercurve and delegating exact earcut predicates to hypertri follows exact-computation discipline. The ear-removal
//! basis is ear clipping.

use crate::finite_projection::normalize_finite_ring_vertices;
use crate::{CurveError, CurveResult, FiniteRegionProfile2, Real};

const TRIANGULATION_CONTEXT: hypertri::TriangulationContext =
    hypertri::TriangulationContext::new(hypertri::PredicatePolicy::STRICT);

/// A finite triangle emitted from a projected region profile.
///
/// The coordinates are projection-boundary `f64` values. Exact CAD topology
/// remains in [`crate::LineArcRegion2`]; this type is intended for mesh generation,
/// rendering, and export layers.
pub type FiniteTriangle2 = [[f64; 2]; 3];

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
) -> CurveResult<Vec<FiniteTriangle2>> {
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
        return Ok(Vec::new());
    }

    let mut hole_indices = Vec::with_capacity(holes.len());
    for hole in holes {
        if let Some(start) = push_ring(hole, &mut vertices, &mut exact)? {
            hole_indices.push(start);
        }
    }

    let indices = hypertri::earcut(&TRIANGULATION_CONTEXT, &exact, &hole_indices)
        .map_err(|err| CurveError::Topology(err.to_string()))?
        .into_value();
    triangles_from_indices(&vertices, &indices)
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
        .chunks_exact(3)
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
    pub fn triangulate(&self) -> CurveResult<Vec<FiniteTriangle2>> {
        let hole_refs = self
            .holes()
            .iter()
            .map(|hole| hole.points())
            .collect::<Vec<_>>();
        triangulate_finite_rings(self.material().points(), &hole_refs)
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
