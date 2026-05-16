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
        exact_eq(coord.x, min.x)
            || exact_eq(coord.x, max.x)
            || exact_eq(coord.y, min.y)
            || exact_eq(coord.y, max.y)
    });
    on_rect_edges.then_some(bounds)
}

/// Return whether a circular drill keepout is fully inside a rectangular board.
pub(super) fn drill_keepout_inside_rect(
    drill: &DrillFeature,
    rect: &geo::Rect<f64>,
    edge_clearance: f64,
) -> bool {
    let radius = drill.diameter / 2.0 + edge_clearance;
    circle_inside_rect(drill.location, radius, rect)
}

/// Return whether feature bounds are fully inside the rectangular board.
pub(super) fn feature_bounds_inside_rect(feature: &CopperFeature, rect: &geo::Rect<f64>) -> bool {
    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return false;
    };
    let min = rect.min();
    let max = rect.max();
    let feature_min = bounds.min();
    let feature_max = bounds.max();

    exact_ge(feature_min.x, min.x)
        && exact_le(feature_max.x, max.x)
        && exact_ge(feature_min.y, min.y)
        && exact_le(feature_max.y, max.y)
}

/// Return whether feature bounds are strictly outside an edge-clearance band.
pub(super) fn feature_bounds_inside_rect_margin(
    feature: &CopperFeature,
    rect: &geo::Rect<f64>,
    margin: f64,
) -> bool {
    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
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
    exact_gt(feature_min.x, min.x + margin)
        && exact_lt(feature_max.x, max.x - margin)
        && exact_gt(feature_min.y, min.y + margin)
        && exact_lt(feature_max.y, max.y - margin)
}

fn circle_inside_rect(center: [f64; 2], radius: f64, rect: &geo::Rect<f64>) -> bool {
    let min = rect.min();
    let max = rect.max();
    exact_ge(center[0] - radius, min.x)
        && exact_le(center[0] + radius, max.x)
        && exact_ge(center[1] - radius, min.y)
        && exact_le(center[1] + radius, max.y)
}

fn exact_eq(left: f64, right: f64) -> bool {
    exact_cmp(left, right).is_some_and(|ordering| ordering == std::cmp::Ordering::Equal)
}

fn exact_ge(left: f64, right: f64) -> bool {
    exact_cmp(left, right).is_some_and(|ordering| ordering != std::cmp::Ordering::Less)
}

fn exact_gt(left: f64, right: f64) -> bool {
    exact_cmp(left, right).is_some_and(|ordering| ordering == std::cmp::Ordering::Greater)
}

fn exact_le(left: f64, right: f64) -> bool {
    exact_cmp(left, right).is_some_and(|ordering| ordering != std::cmp::Ordering::Greater)
}

fn exact_lt(left: f64, right: f64) -> bool {
    exact_cmp(left, right).is_some_and(|ordering| ordering == std::cmp::Ordering::Less)
}

fn exact_cmp(left: f64, right: f64) -> Option<std::cmp::Ordering> {
    // These outline helpers are broad/narrow phase gates: accepting a rectangle
    // or an interior feature may bypass a slower CSG check, so the comparison
    // itself must be a certified predicate. Finite parser coordinates are
    // lifted to exact dyadic `Real`s and ordered through `hyperlimit`, following
    // Yap's exact geometric computation boundary. See Yap, "Towards Exact
    // Geometric Computation," Computational Geometry 7.1-2 (1997).
    //
    // Future KiCad/Gerber parser paths should replace this primitive-float
    // provenance with token-level [`SourceGridFacts`] so repeated rectangle
    // checks can choose a cheap shared-scale integer comparison without
    // re-lifting every f64 edge.
    let provenance = RuleGeometryProvenance::new(
        "axis-aligned-outline-rect",
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    );
    let left = provenance.lift_f64(left)?;
    let right = provenance.lift_f64(right)?;
    compare_reals_with_policy(&left, &right, PredicatePolicy::STRICT).value()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{polygons_to_sketch, rect_polygon};
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
}
