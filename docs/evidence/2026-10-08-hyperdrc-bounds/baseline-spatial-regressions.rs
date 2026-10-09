use super::spatial::LayerPolygonSpatialIndex;
use crate::geometry::{Coord, LineString, Polygon, rect_polygon};

fn coarse_unknown_bounds() -> Polygon<f64> {
    let native=crate::bounds_regressions::uncertain_bounds_polygon();
    // Midpoints on the three native edges are legitimate coarse report samples;
    // they do not enclose the full authoritative triangle.
    Polygon::from_exact_region(native.try_exact_region().unwrap().clone(),LineString(vec![Coord{x:0.5,y:0.0},Coord{x:0.5,y:0.5},Coord{x:0.0,y:0.5},Coord{x:0.5,y:0.0}]),vec![])
}
#[test]
fn layer_index_retains_stored_polygons_with_unknown_native_bounds() {
    let index=LayerPolygonSpatialIndex::new(&[coarse_unknown_bounds()],0.0);
    let candidate=rect_polygon([0.94,0.02],[0.005,0.005],0.0);
    assert_eq!(index.candidates_near_polygon(&candidate,0.0),vec![0]);
}
#[test]
fn layer_index_retains_candidates_for_unknown_query_bounds() {
    let index=LayerPolygonSpatialIndex::new(&[rect_polygon([0.94,0.02],[0.005,0.005],0.0)],0.0);
    assert_eq!(index.candidates_near_polygon(&coarse_unknown_bounds(),0.0),vec![0]);
}
#[test]
fn layer_index_retains_failed_sources_for_required_diagnostics() {
    let failed=rect_polygon([0.0,0.0],[1.0,1.0],0.0).with_exact_construction_error("retained index failure");
    let index=LayerPolygonSpatialIndex::new(&[failed],0.0);
    assert_eq!(index.candidates_near_polygon(&rect_polygon([100.0,0.0],[1.0,1.0],0.0),0.0),vec![0]);
}
