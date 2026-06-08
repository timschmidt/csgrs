//! Finite profile triangulation adapters for hypercurve regions.
//!
//! Triangulation consumes projected boundary vertices, but the ownership of
//! material and hole rings is decided before projection by [`Region2`] and
//! [`RegionView2`](crate::RegionView2). Keeping the profile grouping in
//! hypercurve and delegating exact earcut predicates to hypertri follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>). The ear-removal
//! basis is Meisters, "Polygons Have Ears," *American Mathematical Monthly*
//! 82(6), 1975 (<https://doi.org/10.2307/2319703>).

use crate::finite_projection::normalize_finite_ring_vertices;
use crate::{CurveError, CurveResult, FiniteRegionProfile2, Real};

/// A finite triangle emitted from a projected region profile.
///
/// The coordinates are projection-boundary `f64` values. Exact CAD topology
/// remains in [`crate::Region2`]; this type is intended for mesh generation,
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

    let indices = hypertri::earcut(&exact, &hole_indices)
        .map_err(|err| CurveError::Topology(err.to_string()))?;
    Ok(indices
        .chunks_exact(3)
        .filter_map(|tri| {
            Some([
                *vertices.get(tri[0])?,
                *vertices.get(tri[1])?,
                *vertices.get(tri[2])?,
            ])
        })
        .collect())
}

impl FiniteRegionProfile2 {
    /// Triangulates this projected material-with-holes profile.
    ///
    /// Hole ownership was decided by hypercurve before this finite profile was
    /// built. The triangulation stage therefore receives a topology-preserving
    /// profile record rather than a bag of rings whose roles must be recovered
    /// from winding. Earcut-style triangulation is handled by hypertri using
    /// exact hyperreal predicates; see Meisters (1975) and Yap (1997), cited in
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
