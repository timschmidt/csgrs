use super::spatial::LayerPolygonSpatialIndex;
use crate::geometry::{Coord, LineString, Polygon, rect_polygon};

fn coarse_unknown_bounds() -> Polygon<f64> {
    let native = crate::bounds_regressions::uncertain_bounds_polygon();
    // Midpoints on the three native edges are legitimate coarse report samples;
    // they do not enclose the full authoritative triangle.
    Polygon::from_exact_region(
        native.try_exact_region().unwrap().clone(),
        LineString(vec![
            Coord { x: 0.5, y: 0.0 },
            Coord { x: 0.5, y: 0.5 },
            Coord { x: 0.0, y: 0.5 },
            Coord { x: 0.5, y: 0.0 },
        ]),
        vec![],
    )
}
#[test]
fn layer_index_retains_stored_polygons_with_unknown_native_bounds() {
    let index = LayerPolygonSpatialIndex::new(&[coarse_unknown_bounds()], 0.0);
    let candidate = rect_polygon([0.94, 0.02], [0.005, 0.005], 0.0);
    assert_eq!(index.candidates_near_polygon(&candidate, 0.0), vec![0]);
}
#[test]
fn layer_index_retains_candidates_for_unknown_query_bounds() {
    let index =
        LayerPolygonSpatialIndex::new(&[rect_polygon([0.94, 0.02], [0.005, 0.005], 0.0)], 0.0);
    assert_eq!(
        index.candidates_near_polygon(&coarse_unknown_bounds(), 0.0),
        vec![0]
    );
}
#[test]
fn layer_index_retains_failed_sources_for_required_diagnostics() {
    let failed = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)
        .with_exact_construction_error("retained index failure");
    let index = LayerPolygonSpatialIndex::new(&[failed], 0.0);
    assert_eq!(
        index.candidates_near_polygon(&rect_polygon([100.0, 0.0], [1.0, 1.0], 0.0), 0.0),
        vec![0]
    );
}

#[test]
fn native_grid_preserves_closed_contacts_across_scales_and_cell_boundaries() {
    let polygons = vec![
        rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0),
        rect_polygon([1.0, 0.0], [1.0, 1.0], 0.0),
        rect_polygon([1.0_f64.next_up(), 0.0], [1.0, 1.0], 0.0),
        rect_polygon([1.0_f64.next_down(), 0.0], [1.0, 1.0], 0.0),
        rect_polygon([-1.0, 0.0], [1.0, 1.0], 0.0),
        rect_polygon([0.0, 0.0], [1e-300, 1e-300], 0.0),
        rect_polygon([1e100, 0.0], [1e90, 1.0], 0.0),
        rect_polygon([1e100, 0.0], [1e91, 1.0], 0.0),
        rect_polygon([1.3e308, 0.0], [4e307, 1e-310], 0.0),
        rect_polygon([1.3e308, 0.0], [2e307, 1e-310], 0.0),
    ];
    let index = LayerPolygonSpatialIndex::new(&polygons, 0.0);
    for radius in [0.0, 0.125, 1e300] {
        let clearance = crate::Scalar::try_from(radius).unwrap();
        for (left, polygon) in polygons.iter().enumerate() {
            let candidates = index.candidates_near_polygon(polygon, radius);
            let left_bounds = polygon.try_bounds().unwrap().unwrap();
            for (right, other) in polygons.iter().enumerate() {
                let right_bounds = other.try_bounds().unwrap().unwrap();
                let overlap = (0..2).all(|axis| {
                    crate::scalar::try_compare(
                        &(&left_bounds[axis + 2] + &clearance),
                        &right_bounds[axis],
                    )
                    .unwrap()
                    .is_ge()
                        && crate::scalar::try_compare(
                            &(&right_bounds[axis + 2] + &clearance),
                            &left_bounds[axis],
                        )
                        .unwrap()
                        .is_ge()
                });
                if overlap {
                    assert!(
                        candidates.contains(&right),
                        "left={left}, right={right}, radius={radius}"
                    );
                }
            }
        }
    }
}

#[test]
fn native_grid_retains_large_boxes_and_nonfinite_search_radii() {
    let polygons = vec![
        rect_polygon([0.0, 0.0], [1e6, 1e6], 0.0),
        rect_polygon([100.0, 0.0], [0.1, 0.1], 0.0),
        rect_polygon([1000.0, 0.0], [0.1, 0.1], 0.0),
    ];
    let index = LayerPolygonSpatialIndex::new(&polygons, 0.0);
    assert!(
        index
            .candidates_near_polygon(&polygons[1], 0.0)
            .contains(&0)
    );
    assert_eq!(
        index.candidates_near_polygon(&polygons[0], 0.0),
        vec![0, 1, 2]
    );
    assert_eq!(
        index.candidates_near_polygon(&polygons[1], f64::INFINITY),
        vec![0, 1, 2]
    );
    assert_eq!(
        index.candidates_near_polygon(&polygons[1], f64::NAN),
        vec![0, 1, 2]
    );
}
