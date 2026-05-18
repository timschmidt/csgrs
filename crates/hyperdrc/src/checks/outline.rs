//! Shared board-outline geometry predicates for readiness checks.
//!
//! These helpers keep common rectangular-board fast paths in one place. They are
//! exact only for the narrow predicates they name, and callers still fall back
//! to CSG for non-rectangular outlines, cutouts, or boundary candidates.

use geo::BoundingRect;
use hyperlimit::{PredicatePolicy, compare_reals_with_policy};

use crate::PcbSketch;
use crate::geometry::{RuleGeometryProvenance, SourceGridFacts};
use crate::kicad::{CopperFeature, DrillFeature};

/// Return the board rectangle when the outline is one simple axis-aligned box.
pub(super) fn axis_aligned_outline_rect(outline: &PcbSketch) -> Option<geo::Rect<f64>> {
    axis_aligned_outline_rect_with_grid(outline, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

/// Return the board rectangle using retained source-grid facts for edge tests.
pub(super) fn axis_aligned_outline_rect_with_grid(
    outline: &PcbSketch,
    grid: SourceGridFacts,
) -> Option<geo::Rect<f64>> {
    let outline_geometry = outline.to_multipolygon();
    let [polygon] = outline_geometry.0.as_slice() else {
        return None;
    };
    if !polygon.interiors().is_empty() {
        return None;
    }

    let bounds = polygon.bounding_rect()?;
    let exterior = &polygon.exterior().0;
    if exterior.len() != 5 || exterior.first() != exterior.last() {
        return None;
    }

    let min = bounds.min();
    let max = bounds.max();
    let on_rect_edges = exterior.iter().take(exterior.len() - 1).all(|coord| {
        exact_eq_with_grid(coord.x, min.x, grid)
            || exact_eq_with_grid(coord.x, max.x, grid)
            || exact_eq_with_grid(coord.y, min.y, grid)
            || exact_eq_with_grid(coord.y, max.y, grid)
    });
    on_rect_edges.then_some(bounds)
}

/// Return whether a circular drill keepout is fully inside a rectangular board.
pub(super) fn drill_keepout_inside_rect(
    drill: &DrillFeature,
    rect: &geo::Rect<f64>,
    edge_clearance: f64,
) -> bool {
    drill_keepout_inside_rect_with_grid(
        drill,
        rect,
        edge_clearance,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Return whether a drill keepout is inside a rectangle using retained grid facts.
pub(super) fn drill_keepout_inside_rect_with_grid(
    drill: &DrillFeature,
    rect: &geo::Rect<f64>,
    edge_clearance: f64,
    grid: SourceGridFacts,
) -> bool {
    let radius = drill.diameter / 2.0 + edge_clearance;
    circle_inside_rect_with_grid(drill.location, radius, rect, grid)
}

/// Return whether feature bounds are fully inside the rectangular board.
pub(super) fn feature_bounds_inside_rect(feature: &CopperFeature, rect: &geo::Rect<f64>) -> bool {
    feature_bounds_inside_rect_with_grid(feature, rect, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

/// Return whether feature bounds are inside a rectangle using retained grid facts.
pub(super) fn feature_bounds_inside_rect_with_grid(
    feature: &CopperFeature,
    rect: &geo::Rect<f64>,
    grid: SourceGridFacts,
) -> bool {
    let Some(bounds) = feature.sketch.geometry().bounding_rect() else {
        return false;
    };
    let min = rect.min();
    let max = rect.max();
    let feature_min = bounds.min();
    let feature_max = bounds.max();

    // This rectangular containment gate can skip expensive CSG for clearly
    // interior copper. Keep the decision certified with source-grid facts as
    // Yap recommends in "Towards Exact Geometric Computation,"
    // Computational Geometry 7.1-2 (1997).
    exact_ge_with_grid(feature_min.x, min.x, grid)
        && exact_le_with_grid(feature_max.x, max.x, grid)
        && exact_ge_with_grid(feature_min.y, min.y, grid)
        && exact_le_with_grid(feature_max.y, max.y, grid)
}

/// Return whether feature bounds are strictly outside an edge-clearance band.
pub(super) fn feature_bounds_inside_rect_margin(
    feature: &CopperFeature,
    rect: &geo::Rect<f64>,
    margin: f64,
) -> bool {
    feature_bounds_inside_rect_margin_with_grid(
        feature,
        rect,
        margin,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Return whether feature bounds clear an edge band using retained grid facts.
pub(super) fn feature_bounds_inside_rect_margin_with_grid(
    feature: &CopperFeature,
    rect: &geo::Rect<f64>,
    margin: f64,
    grid: SourceGridFacts,
) -> bool {
    let Some(bounds) = feature.sketch.geometry().bounding_rect() else {
        return false;
    };
    let min = rect.min();
    let max = rect.max();
    let feature_min = bounds.min();
    let feature_max = bounds.max();

    // Strict comparisons preserve existing edge-band behavior at the exact
    // review threshold while skipping obvious interior features. This mirrors
    // the broad/narrow-phase structure in Ericson, *Real-Time Collision
    // Detection* (2005): rectangle predicates reject safe interior candidates,
    // and exact CSG/boundary-distance logic remains authoritative near edges.
    //
    // Retaining source-grid facts at this gate follows Yap's object-level EGC
    // boundary: do not discard exact input structure before a predicate chooses
    // its arithmetic. See Yap, "Towards Exact Geometric Computation,"
    // Computational Geometry 7.1-2 (1997).
    exact_gt_with_grid(feature_min.x, min.x + margin, grid)
        && exact_lt_with_grid(feature_max.x, max.x - margin, grid)
        && exact_gt_with_grid(feature_min.y, min.y + margin, grid)
        && exact_lt_with_grid(feature_max.y, max.y - margin, grid)
}

fn circle_inside_rect_with_grid(
    center: [f64; 2],
    radius: f64,
    rect: &geo::Rect<f64>,
    grid: SourceGridFacts,
) -> bool {
    let min = rect.min();
    let max = rect.max();
    // This predicate can skip later CSG work for clearly interior drill
    // keepouts, so its comparisons must remain certified. Carrying parser
    // source-grid facts to this boundary follows Yap's object-level exactness
    // discipline: keep representation structure with geometric objects until a
    // predicate deliberately selects arithmetic. See Yap, "Towards Exact
    // Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    exact_ge_with_grid(center[0] - radius, min.x, grid)
        && exact_le_with_grid(center[0] + radius, max.x, grid)
        && exact_ge_with_grid(center[1] - radius, min.y, grid)
        && exact_le_with_grid(center[1] + radius, max.y, grid)
}

fn exact_eq_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering == std::cmp::Ordering::Equal)
}

fn exact_ge_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering != std::cmp::Ordering::Less)
}

fn exact_gt_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering == std::cmp::Ordering::Greater)
}

fn exact_le_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering != std::cmp::Ordering::Greater)
}

fn exact_lt_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering == std::cmp::Ordering::Less)
}

fn exact_cmp_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> Option<std::cmp::Ordering> {
    // These outline helpers are broad/narrow phase gates: accepting a rectangle
    // or an interior feature may bypass a slower CSG check, so the comparison
    // itself must be a certified predicate. Finite parser coordinates are
    // lifted to exact dyadic `Real`s and ordered through `hyperlimit`, following
    // Yap's exact geometric computation boundary. See Yap, "Towards Exact
    // Geometric Computation," Computational Geometry 7.1-2 (1997).
    //
    let provenance = RuleGeometryProvenance::new("axis-aligned-outline-rect", grid);
    let left = provenance.lift_f64(left)?;
    let right = provenance.lift_f64(right)?;
    compare_reals_with_policy(&left, &right, PredicatePolicy::STRICT).value()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{SourceUnit, polygons_to_sketch, rect_polygon};
    use crate::kicad::CopperKind;

    use super::*;

    fn sketch_rect(center: [f64; 2], size: [f64; 2]) -> PcbSketch {
        polygons_to_sketch(
            vec![rect_polygon(center, size, 0.0)],
            Some(LayerMetadata {
                name: "outline helper test".to_string(),
            }),
        )
    }

    fn copper_rect(center: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some("HV_BUS".to_string()),
            kind: CopperKind::Segment,
            location: center,
            sketch: sketch_rect(center, size),
        }
    }

    #[test]
    fn axis_aligned_outline_rect_accepts_simple_box() {
        let outline = sketch_rect([50.0, 50.0], [100.0, 100.0]);

        let rect =
            axis_aligned_outline_rect(&outline).expect("simple rectangle should be detected");

        assert_eq!(rect.min().x, 0.0);
        assert_eq!(rect.min().y, 0.0);
        assert_eq!(rect.max().x, 100.0);
        assert_eq!(rect.max().y, 100.0);
    }

    #[test]
    fn axis_aligned_outline_rect_accepts_retained_gerber_grid() {
        let outline = sketch_rect([50.0, 50.0], [100.0, 100.0]);
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        let rect = axis_aligned_outline_rect_with_grid(&outline, grid)
            .expect("simple rectangle should be detected with retained Gerber grid facts");

        assert_eq!(rect.min().x, 0.0);
        assert!(
            exact_cmp_with_grid(0.5, rect.min().x, grid)
                .is_some_and(|ordering| ordering == std::cmp::Ordering::Greater)
        );
    }

    #[test]
    fn axis_aligned_outline_rect_rejects_near_edge_epsilon_drift() {
        let polygon = crate::geometry::polygon_from_points(vec![
            [0.0, 0.0],
            [100.0, 0.0],
            [100.0, 100.0],
            [5.0e-10, 50.0],
        ]);
        let outline = crate::geometry::polygon_to_sketch(
            polygon,
            Some(LayerMetadata {
                name: "near rectangle".to_string(),
            }),
        );

        assert!(
            axis_aligned_outline_rect(&outline).is_none(),
            "near-rectangular outlines must stay on the exact geometry path"
        );
    }

    #[test]
    fn feature_margin_predicate_is_strict_at_edge_band_boundary() {
        let outline = sketch_rect([50.0, 50.0], [100.0, 100.0]);
        let rect =
            axis_aligned_outline_rect(&outline).expect("simple rectangle should be detected");
        let clearly_inside = copper_rect([50.0, 50.0], [10.0, 10.0]);
        let touches_margin = copper_rect([2.0, 50.0], [2.0, 2.0]);

        assert!(feature_bounds_inside_rect_margin(
            &clearly_inside,
            &rect,
            1.0
        ));
        assert!(
            !feature_bounds_inside_rect_margin(&touches_margin, &rect, 1.0),
            "features touching the review band must stay on the exact CSG path"
        );
    }

    #[test]
    fn drill_keepout_inside_rect_accepts_retained_excellon_grid() {
        let outline = sketch_rect([50.0, 50.0], [100.0, 100.0]);
        let rect =
            axis_aligned_outline_rect(&outline).expect("simple rectangle should be detected");
        let drill = DrillFeature {
            location: [50.0, 50.0],
            diameter: 0.30,
            net: None,
            plated: false,
        };
        let grid = SourceGridFacts::source_grid(SourceUnit::Excellon, 1_000);

        assert!(drill_keepout_inside_rect_with_grid(
            &drill, &rect, 0.25, grid
        ));
    }
}
