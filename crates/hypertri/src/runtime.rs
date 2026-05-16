//! Runtime algorithm selection for compiled triangulation algorithms.

use crate::error::Result;
#[cfg(feature = "cdt")]
use crate::polygon::{open_ring_indices, rings_from_hole_indices};
use crate::types::{Point2, PolygonInput, TriangleIndices};

/// Polygon triangulation algorithm requested at runtime.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolygonTriangulationAlgorithm {
    /// Select an available algorithm from input shape and options.
    Auto,
    /// Use the earcut-style polygon algorithm.
    #[cfg(feature = "earcut")]
    Earcut,
    /// Use the constrained-Delaunay path.
    #[cfg(feature = "cdt")]
    ConstrainedDelaunay,
}

/// Triangle quality preference used by [`TriangulationOptions`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QualityPolicy {
    /// Prefer preserving the input polygon boundary with minimal extra work.
    PreserveBoundary,
    /// Prefer Delaunay-like triangle quality when that algorithm is compiled.
    PreferDelaunay,
}

/// Runtime options shared by polygon triangulation entry points.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TriangulationOptions {
    /// Algorithm requested by the caller.
    pub algorithm: PolygonTriangulationAlgorithm,
    /// Quality preference used by `Auto`.
    pub quality: QualityPolicy,
}

impl Default for TriangulationOptions {
    fn default() -> Self {
        Self {
            algorithm: PolygonTriangulationAlgorithm::Auto,
            quality: QualityPolicy::PreserveBoundary,
        }
    }
}

/// Triangulate a polygon using runtime algorithm selection.
pub fn triangulate_polygon(
    input: &PolygonInput,
    options: TriangulationOptions,
) -> Result<TriangleIndices> {
    triangulate_polygon_points(input.vertices(), input.hole_indices(), options)
}

/// Triangulate borrowed polygon buffers using runtime algorithm selection.
pub fn triangulate_polygon_points(
    vertices: &[Point2],
    hole_indices: &[usize],
    options: TriangulationOptions,
) -> Result<TriangleIndices> {
    #[cfg(not(any(feature = "earcut", feature = "cdt")))]
    let _ = (vertices, hole_indices);

    match resolve_algorithm(options) {
        #[cfg(feature = "earcut")]
        PolygonTriangulationAlgorithm::Earcut => crate::earcut::triangulate(vertices, hole_indices),
        #[cfg(feature = "cdt")]
        PolygonTriangulationAlgorithm::ConstrainedDelaunay => {
            triangulate_polygon_with_cdt(vertices, hole_indices)
        }
        PolygonTriangulationAlgorithm::Auto => Err(crate::Error::UnsupportedFeature {
            feature: "compiled polygon triangulation algorithm",
        }),
    }
}

#[cfg(feature = "cdt")]
fn triangulate_polygon_with_cdt(
    vertices: &[Point2],
    hole_indices: &[usize],
) -> Result<TriangleIndices> {
    if vertices.len() < 3 {
        return Ok(Vec::new());
    }

    let rings = rings_from_hole_indices(vertices, hole_indices)?;
    let mut constraints = Vec::new();
    append_ring_constraints(vertices, rings.exterior(), &mut constraints)?;
    for &hole in rings.holes() {
        append_ring_constraints(vertices, hole, &mut constraints)?;
    }
    let triangulation = crate::cdt::constrained_delaunay(vertices, &constraints)?;

    Ok(triangulation
        .triangles()
        .iter()
        .flat_map(|triangle| triangle.iter().copied())
        .collect())
}

#[cfg(feature = "cdt")]
fn append_ring_constraints(
    vertices: &[Point2],
    range: crate::polygon::RingRange,
    constraints: &mut Vec<crate::Constraint>,
) -> Result<()> {
    let ring = open_ring_indices(vertices, range);
    if ring.len() < 3 {
        return Err(crate::Error::InvalidInput {
            reason: "polygon ring is degenerate",
        });
    }

    constraints.extend(
        (0..ring.len())
            .map(|index| crate::Constraint::new(ring[index], ring[(index + 1) % ring.len()])),
    );
    Ok(())
}

fn resolve_algorithm(options: TriangulationOptions) -> PolygonTriangulationAlgorithm {
    match options.algorithm {
        PolygonTriangulationAlgorithm::Auto => auto_algorithm(options.quality),
        #[cfg(any(feature = "earcut", feature = "cdt"))]
        algorithm => algorithm,
    }
}

fn auto_algorithm(quality: QualityPolicy) -> PolygonTriangulationAlgorithm {
    // Structural-dispatch note: `Auto` currently respects only the caller's
    // quality preference and compile-time feature set. Once polygon
    // normalization carries cheap facts such as hole count, convexity,
    // duplicate/collinear removals, coordinate exact-rational kind, and
    // constraint density, this should select the lower-cost exact algorithm:
    // fan/earcut for convex or nearly convex simple rings, CDT for constraint
    // heavy inputs or when triangle quality is requested.
    match quality {
        #[cfg(feature = "cdt")]
        QualityPolicy::PreferDelaunay => PolygonTriangulationAlgorithm::ConstrainedDelaunay,
        #[cfg(feature = "earcut")]
        QualityPolicy::PreserveBoundary => PolygonTriangulationAlgorithm::Earcut,
        #[cfg(all(feature = "earcut", not(feature = "cdt")))]
        QualityPolicy::PreferDelaunay => PolygonTriangulationAlgorithm::Earcut,
        #[cfg(all(feature = "cdt", not(feature = "earcut")))]
        QualityPolicy::PreserveBoundary => PolygonTriangulationAlgorithm::ConstrainedDelaunay,
        #[allow(unreachable_patterns)]
        _ => PolygonTriangulationAlgorithm::Auto,
    }
}
