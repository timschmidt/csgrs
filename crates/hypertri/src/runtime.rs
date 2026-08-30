//! Runtime algorithm selection for compiled triangulation algorithms.

use crate::context::{TriangulationContext, TriangulationOutcome};
use crate::error::Result;
#[cfg(feature = "cdt")]
use crate::polygon::{open_ring_indices, rings_from_hole_indices};
use crate::predicate_evaluator::PredicateEvaluator;
use crate::types::{Point2, PolygonInput, PolygonInputFacts, TriangleIndices};

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

/// Runtime selection report for one completed polygon triangulation.
///
/// The report records the selected algorithm together with the structural
/// polygon facts that justified selection. These facts are advisory scheduling
/// metadata only; exact predicates inside the selected algorithm remain the
/// topology certificates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PolygonTriangulationReport {
    /// Algorithm selected for the completed triangulation.
    pub algorithm: PolygonTriangulationAlgorithm,
    /// Quality preference that participated in selection.
    pub quality: QualityPolicy,
    /// Structural facts used during runtime selection.
    pub facts: PolygonInputFacts,
}

/// Triangulate a polygon using runtime algorithm selection.
pub fn triangulate_polygon(
    context: &TriangulationContext,
    input: &PolygonInput,
    options: TriangulationOptions,
) -> Result<TriangulationOutcome<TriangleIndices>> {
    let evaluator = PredicateEvaluator::new(context);
    let (triangles, _) = triangulate_polygon_selected(&evaluator, input, options)?;
    Ok(evaluator.finish(triangles))
}

/// Triangulate a polygon and report the runtime selection facts.
pub fn triangulate_polygon_with_report(
    context: &TriangulationContext,
    input: &PolygonInput,
    options: TriangulationOptions,
) -> Result<TriangulationOutcome<(TriangleIndices, PolygonTriangulationReport)>> {
    let evaluator = PredicateEvaluator::new(context);
    let (triangles, algorithm) = triangulate_polygon_selected(&evaluator, input, options)?;
    Ok(evaluator.finish((
        triangles,
        PolygonTriangulationReport {
            algorithm,
            quality: options.quality,
            facts: input.facts().clone(),
        },
    )))
}

/// Triangulate borrowed polygon buffers using runtime algorithm selection.
pub fn triangulate_polygon_points(
    context: &TriangulationContext,
    vertices: &[Point2],
    hole_indices: &[usize],
    options: TriangulationOptions,
) -> Result<TriangulationOutcome<TriangleIndices>> {
    #[cfg(not(any(feature = "earcut", feature = "cdt")))]
    let _ = (vertices, hole_indices);

    let algorithm = resolve_algorithm(options)?;
    let evaluator = PredicateEvaluator::new(context);
    let triangles =
        triangulate_polygon_points_with_algorithm(&evaluator, vertices, hole_indices, algorithm)?;
    Ok(evaluator.finish(triangles))
}

fn triangulate_polygon_selected(
    evaluator: &PredicateEvaluator,
    input: &PolygonInput,
    options: TriangulationOptions,
) -> Result<(TriangleIndices, PolygonTriangulationAlgorithm)> {
    let algorithm = resolve_algorithm_for_facts(options, input.facts())?;
    let triangles = triangulate_polygon_points_with_algorithm(
        evaluator,
        input.vertices(),
        input.hole_indices(),
        algorithm,
    )?;
    Ok((triangles, algorithm))
}

fn triangulate_polygon_points_with_algorithm(
    evaluator: &PredicateEvaluator,
    vertices: &[Point2],
    hole_indices: &[usize],
    algorithm: PolygonTriangulationAlgorithm,
) -> Result<TriangleIndices> {
    #[cfg(not(any(feature = "earcut", feature = "cdt")))]
    let _ = (evaluator, vertices, hole_indices);

    match algorithm {
        #[cfg(feature = "earcut")]
        PolygonTriangulationAlgorithm::Earcut => {
            crate::earcut::triangulate_inner(evaluator, vertices, hole_indices)
        }
        #[cfg(feature = "cdt")]
        PolygonTriangulationAlgorithm::ConstrainedDelaunay => {
            triangulate_polygon_with_cdt(evaluator, vertices, hole_indices)
        }
        PolygonTriangulationAlgorithm::Auto => Err(crate::Error::UnsupportedFeature {
            feature: "compiled polygon triangulation algorithm",
        }),
    }
}

#[cfg(feature = "cdt")]
fn triangulate_polygon_with_cdt(
    evaluator: &PredicateEvaluator,
    vertices: &[Point2],
    hole_indices: &[usize],
) -> Result<TriangleIndices> {
    if vertices.len() < 3 {
        return Ok(Vec::new());
    }

    let rings = rings_from_hole_indices(vertices, hole_indices)?;
    let mut constraints = Vec::new();
    append_ring_constraints(evaluator, vertices, rings.exterior(), &mut constraints)?;
    for &hole in rings.holes() {
        append_ring_constraints(evaluator, vertices, hole, &mut constraints)?;
    }
    let triangulation = crate::cdt::constrained_delaunay_inner(evaluator, vertices, &constraints)?;

    Ok(triangulation
        .triangles()
        .iter()
        .flat_map(|triangle| triangle.iter().copied())
        .collect())
}

#[cfg(feature = "cdt")]
fn append_ring_constraints(
    evaluator: &PredicateEvaluator,
    vertices: &[Point2],
    range: crate::polygon::RingRange,
    constraints: &mut Vec<crate::Constraint>,
) -> Result<()> {
    let ring = open_ring_indices(evaluator, vertices, range)?;
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

fn resolve_algorithm(options: TriangulationOptions) -> Result<PolygonTriangulationAlgorithm> {
    match options.algorithm {
        PolygonTriangulationAlgorithm::Auto => resolve_auto_algorithm(options.quality, None),
        #[cfg(any(feature = "earcut", feature = "cdt"))]
        algorithm => Ok(algorithm),
    }
}

fn resolve_algorithm_for_facts(
    options: TriangulationOptions,
    facts: &PolygonInputFacts,
) -> Result<PolygonTriangulationAlgorithm> {
    match options.algorithm {
        PolygonTriangulationAlgorithm::Auto => resolve_auto_algorithm(options.quality, Some(facts)),
        #[cfg(any(feature = "earcut", feature = "cdt"))]
        algorithm => Ok(algorithm),
    }
}

fn resolve_auto_algorithm(
    quality: QualityPolicy,
    facts: Option<&PolygonInputFacts>,
) -> Result<PolygonTriangulationAlgorithm> {
    // Structural-dispatch note: `Auto` consumes only facts already retained on
    // `PolygonInput`; it does not probe primitive coordinates or run topology
    // predicates early. Degenerate/unknown-zero edges, uncertified ring
    // orientation, and unknown local turn consistency are conservative reasons
    // to keep the boundary-preserving earcut path when it is available, because
    // the CDT route has to materialize every ring edge as a constraint before
    // legalization. This is advisory scheduling, not a correctness
    // certificate; the selected algorithm still owns exact orientation and
    // containment predicates.
    let boundary_cleanup_preferred = facts.is_some_and(|facts| {
        facts.known_degenerate_edge_count() > 0 || facts.unknown_edge_zero_status_count() > 0
    });
    #[cfg(not(all(feature = "cdt", feature = "earcut")))]
    let _ = boundary_cleanup_preferred;
    let algorithm = match quality {
        #[cfg(all(feature = "cdt", feature = "earcut"))]
        QualityPolicy::PreferDelaunay if boundary_cleanup_preferred => {
            PolygonTriangulationAlgorithm::Earcut
        }
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
    };
    if algorithm == PolygonTriangulationAlgorithm::Auto {
        Err(crate::Error::UnsupportedFeature {
            feature: "compiled polygon triangulation algorithm",
        })
    } else {
        Ok(algorithm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "cdt")]
    use crate::types::Real;

    #[cfg(feature = "cdt")]
    fn point(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn unresolved_auto_cannot_reach_an_algorithm_executor() {
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let evaluator = PredicateEvaluator::new(&context);
        let error = triangulate_polygon_points_with_algorithm(
            &evaluator,
            &[],
            &[],
            PolygonTriangulationAlgorithm::Auto,
        )
        .unwrap_err();
        assert_eq!(
            error,
            crate::Error::UnsupportedFeature {
                feature: "compiled polygon triangulation algorithm"
            }
        );
    }

    #[cfg(feature = "cdt")]
    #[test]
    fn cdt_ring_constraint_builder_rejects_a_collapsed_closed_ring() {
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let evaluator = PredicateEvaluator::new(&context);
        let vertices = vec![point(0, 0), point(2, 0), point(0, 0)];
        let error = append_ring_constraints(
            &evaluator,
            &vertices,
            crate::polygon::RingRange { start: 0, end: 3 },
            &mut Vec::new(),
        )
        .unwrap_err();
        assert_eq!(
            error,
            crate::Error::InvalidInput {
                reason: "polygon ring is degenerate"
            }
        );
    }

    #[cfg(not(any(feature = "earcut", feature = "cdt")))]
    #[test]
    fn auto_reports_that_no_polygon_algorithm_was_compiled() {
        let error = resolve_algorithm(TriangulationOptions::default()).unwrap_err();
        assert_eq!(
            error,
            crate::Error::UnsupportedFeature {
                feature: "compiled polygon triangulation algorithm"
            }
        );
    }

    #[cfg(all(feature = "earcut", not(feature = "cdt")))]
    #[test]
    fn earcut_only_build_resolves_both_quality_policies() {
        assert_eq!(
            resolve_auto_algorithm(QualityPolicy::PreserveBoundary, None).unwrap(),
            PolygonTriangulationAlgorithm::Earcut
        );
        assert_eq!(
            resolve_auto_algorithm(QualityPolicy::PreferDelaunay, None).unwrap(),
            PolygonTriangulationAlgorithm::Earcut
        );
    }

    #[cfg(all(feature = "cdt", not(feature = "earcut")))]
    #[test]
    fn cdt_only_build_resolves_both_quality_policies() {
        assert_eq!(
            resolve_auto_algorithm(QualityPolicy::PreserveBoundary, None).unwrap(),
            PolygonTriangulationAlgorithm::ConstrainedDelaunay
        );
        assert_eq!(
            resolve_auto_algorithm(QualityPolicy::PreferDelaunay, None).unwrap(),
            PolygonTriangulationAlgorithm::ConstrainedDelaunay
        );
    }

    #[cfg(all(feature = "earcut", feature = "cdt"))]
    #[test]
    fn prefer_delaunay_keeps_boundary_cleanup_on_earcut() {
        let input = PolygonInput::new(
            vec![point(0, 0), point(2, 0), point(2, 0), point(0, 2)],
            vec![],
        );
        assert_eq!(
            resolve_algorithm_for_facts(
                TriangulationOptions {
                    algorithm: PolygonTriangulationAlgorithm::Auto,
                    quality: QualityPolicy::PreferDelaunay,
                },
                input.facts(),
            )
            .unwrap(),
            PolygonTriangulationAlgorithm::Earcut
        );
    }
}
