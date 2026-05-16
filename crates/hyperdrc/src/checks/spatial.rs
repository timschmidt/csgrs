//! Small spatial indexes shared by readiness checks.
//!
//! These indexes are broad-phase filters only: callers must still run their
//! exact CSG or distance predicate on every returned candidate. The design is
//! intentionally simple and deterministic because readiness reports are easier
//! to review when candidate ordering is stable across platforms and runs.

use std::collections::BTreeMap;

use geo::{BoundingRect, Polygon};

use crate::kicad::{CopperFeature, DrillFeature};

const SPATIAL_GRID_EPSILON: f64 = 1.0e-9;

/// Layer-aware grid index over parsed KiCad copper features.
///
/// The index stores each feature by its parsed location and inflates queries by
/// both the queried feature span and the maximum indexed feature span. That is
/// a conservative broad-phase rule, not a proof of geometric contact. It follows
/// the standard two-stage collision-detection pattern described by Ericson,
/// *Real-Time Collision Detection* (2005): a cheap spatial partition proposes
/// candidate pairs, then exact geometry rejects false positives. HyperDRC uses
/// the same pattern for PCB readiness so large sparse boards do not require a
/// full all-pairs CSG pass. Buckets are grouped by layer so same-layer queries
/// borrow the layer key once instead of allocating one key per bucket probe.
pub(super) struct CopperSpatialIndex<'a> {
    features: &'a [&'a CopperFeature],
    buckets_by_layer: BTreeMap<String, BTreeMap<(i64, i64), Vec<usize>>>,
    all_layer_buckets: BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    maximum_span: f64,
}

impl<'a> CopperSpatialIndex<'a> {
    /// Build an index with a nominal query radius.
    pub(super) fn new(features: &'a [&'a CopperFeature], nominal_radius: f64) -> Self {
        let maximum_span = features
            .iter()
            .map(|feature| feature_span(feature))
            .fold(0.0_f64, f64::max);
        let cell_size = nominal_radius.max(maximum_span).max(SPATIAL_GRID_EPSILON);
        let mut buckets_by_layer: BTreeMap<String, BTreeMap<(i64, i64), Vec<usize>>> =
            BTreeMap::new();
        let mut all_layer_buckets: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();

        for (index, feature) in features.iter().enumerate() {
            let (bucket_x, bucket_y) = center_bucket_key(feature.location, cell_size);
            buckets_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .entry((bucket_x, bucket_y))
                .or_default()
                .push(index);
            all_layer_buckets
                .entry((bucket_x, bucket_y))
                .or_default()
                .push(index);
        }

        Self {
            features,
            buckets_by_layer,
            all_layer_buckets,
            cell_size,
            maximum_span,
        }
    }

    /// Number of populated grid buckets, useful for trace diagnostics.
    pub(super) fn bucket_count(&self) -> usize {
        self.buckets_by_layer.values().map(BTreeMap::len).sum()
    }

    /// Return same-layer candidate features near the queried feature.
    pub(super) fn same_layer_near_feature(
        &self,
        feature: &CopperFeature,
        radius: f64,
    ) -> Vec<usize> {
        let query_radius = radius + feature_span(feature) / 2.0 + self.maximum_span / 2.0;
        self.near_center_on_layer(feature.location, &feature.layer, query_radius)
    }

    /// Return candidate features on any layer near the queried feature.
    ///
    /// This supports checks such as reference-plane review, where signal copper
    /// on one layer is intentionally compared with ground copper on another
    /// layer. It is still only a conservative broad phase in the Ericson,
    /// *Real-Time Collision Detection* (2005) sense; callers must run exact CSG
    /// or distance predicates on returned candidates.
    pub(super) fn all_layers_near_feature(
        &self,
        feature: &CopperFeature,
        radius: f64,
    ) -> Vec<usize> {
        let query_radius = radius + feature_span(feature) / 2.0 + self.maximum_span / 2.0;
        self.near_center_all_layers(feature.location, query_radius)
    }

    /// Return same-layer candidate centers within a circle.
    ///
    /// This preserves checks that intentionally reason about parsed locations
    /// rather than copper outlines, such as looking for a ground-stitching via
    /// near an RF feed.
    pub(super) fn same_layer_centers_within(
        &self,
        center: [f64; 2],
        layer: &str,
        radius: f64,
    ) -> Vec<usize> {
        let radius_squared = radius * radius;
        self.near_center_on_layer(center, layer, radius)
            .into_iter()
            .filter(|&index| {
                let candidate = self.features[index];
                let dx = candidate.location[0] - center[0];
                let dy = candidate.location[1] - center[1];
                dx * dx + dy * dy <= radius_squared
            })
            .collect()
    }

    /// Return candidate features on any layer near a circular keepout.
    ///
    /// Use this for source geometry that has no layer of its own, such as
    /// mechanical drills. Callers still need their exact shape predicate.
    pub(super) fn all_layers_near_circle(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        let query_radius = radius + self.maximum_span / 2.0;
        self.near_center_all_layers(center, query_radius)
    }

    fn near_center_all_layers(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        let min_x = bucket_coordinate(center[0] - radius, self.cell_size);
        let max_x = bucket_coordinate(center[0] + radius, self.cell_size);
        let min_y = bucket_coordinate(center[1] - radius, self.cell_size);
        let max_y = bucket_coordinate(center[1] + radius, self.cell_size);
        let mut candidates = Vec::new();

        // Keep a second layerless bucket map for source geometry such as NPTH
        // drills that can interact with copper on any selected layer. This is
        // still Ericson's broad/narrow-phase grid from *Real-Time Collision
        // Detection* (2005), but avoids scanning every layer-qualified bucket
        // for every mechanical feature on sparse, multi-layer boards.
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if let Some(bucket) = self.all_layer_buckets.get(&(x, y)) {
                    candidates.extend(bucket.iter().copied());
                }
            }
        }

        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }

    fn near_center_on_layer(&self, center: [f64; 2], layer: &str, radius: f64) -> Vec<usize> {
        let Some(layer_buckets) = self.buckets_by_layer.get(layer) else {
            return Vec::new();
        };
        let min_x = bucket_coordinate(center[0] - radius, self.cell_size);
        let max_x = bucket_coordinate(center[0] + radius, self.cell_size);
        let min_y = bucket_coordinate(center[1] - radius, self.cell_size);
        let max_y = bucket_coordinate(center[1] + radius, self.cell_size);
        let mut candidates = Vec::new();

        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if let Some(bucket) = layer_buckets.get(&(x, y)) {
                    candidates.extend(bucket.iter().copied());
                }
            }
        }

        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }
}

/// Deterministic broad-phase index for flattened layer polygons.
///
/// The index stores polygon bounding-box centers and inflates each query by the
/// queried polygon span plus the largest indexed span. This is intentionally a
/// conservative broad phase in the style of Ericson, *Real-Time Collision
/// Detection* (2005): every returned candidate still goes through the exact
/// offset/intersection predicate in the caller. Keeping this helper
/// `pub(super)` avoids exposing bbox-center approximations as public geometry
/// semantics while letting layer checks share tested spatial infrastructure.
pub(super) struct LayerPolygonSpatialIndex {
    buckets: BTreeMap<(i64, i64), Vec<usize>>,
    bounds: Vec<Option<geo::Rect<f64>>>,
    cell_size: f64,
    maximum_span: f64,
}

impl LayerPolygonSpatialIndex {
    /// Build an index over polygon bounds with a nominal query radius.
    pub(super) fn new(polygons: &[Polygon<f64>], nominal_radius: f64) -> Self {
        let bounds = polygons
            .iter()
            .map(Polygon::bounding_rect)
            .collect::<Vec<_>>();
        let maximum_span = bounds
            .iter()
            .flatten()
            .map(rect_span)
            .fold(0.0_f64, f64::max);
        let cell_size = nominal_radius.max(maximum_span).max(SPATIAL_GRID_EPSILON);
        let mut buckets = BTreeMap::new();

        for (index, bounds) in bounds.iter().enumerate() {
            let Some(bounds) = bounds else {
                continue;
            };
            buckets
                .entry(center_bucket_key(rect_center(bounds), cell_size))
                .or_insert_with(Vec::new)
                .push(index);
        }

        Self {
            buckets,
            bounds,
            cell_size,
            maximum_span,
        }
    }

    /// Number of populated grid buckets, useful for trace diagnostics.
    pub(super) fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// Return candidate polygons near a stored polygon, excluding itself.
    pub(super) fn candidates_near(&self, polygon_index: usize, radius: f64) -> Vec<usize> {
        let Some(bounds) = self.bounds[polygon_index] else {
            return (0..self.bounds.len())
                .filter(|&index| index != polygon_index)
                .collect();
        };
        self.candidates_near_bounds(bounds, radius)
            .into_iter()
            .filter(|&index| index != polygon_index)
            .collect()
    }

    /// Return candidate indexed polygons near an external polygon.
    pub(super) fn candidates_near_polygon(
        &self,
        polygon: &Polygon<f64>,
        radius: f64,
    ) -> Vec<usize> {
        let Some(bounds) = polygon.bounding_rect() else {
            return (0..self.bounds.len()).collect();
        };
        self.candidates_near_bounds(bounds, radius)
    }

    /// Return later candidate indexes near a stored polygon.
    ///
    /// This supports same-layer unordered-pair checks without allocating a
    /// separate visited-pair set.
    pub(super) fn later_candidates_near(&self, polygon_index: usize, radius: f64) -> Vec<usize> {
        self.candidates_near(polygon_index, radius)
            .into_iter()
            .filter(|&index| index > polygon_index)
            .collect()
    }

    fn candidates_near_bounds(&self, bounds: geo::Rect<f64>, radius: f64) -> Vec<usize> {
        let query_radius = radius + rect_span(&bounds) / 2.0 + self.maximum_span / 2.0;
        let center = rect_center(&bounds);
        candidate_centers_within(&self.buckets, self.cell_size, center, query_radius)
    }
}

/// Grid index over drill and slot proxy centers.
///
/// Drill spacing is a center-distance predicate adjusted by both drill radii,
/// so a center-only broad phase is sufficient as long as each query is inflated
/// by the largest indexed radius plus the requested spacing. This follows the
/// same broad/narrow-phase structure described by Ericson, *Real-Time Collision
/// Detection* (2005): the index only proposes candidate pairs, while the caller
/// remains responsible for exact edge-gap comparison.
pub(super) struct DrillSpatialIndex<'a> {
    drills: &'a [DrillFeature],
    buckets: BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    maximum_radius: f64,
}

impl<'a> DrillSpatialIndex<'a> {
    /// Build an index for drill spacing queries.
    pub(super) fn new(drills: &'a [DrillFeature], nominal_spacing: f64) -> Self {
        let maximum_radius = drills
            .iter()
            .map(|drill| drill.diameter / 2.0)
            .fold(0.0_f64, f64::max);
        let cell_size = (maximum_radius * 2.0 + nominal_spacing).max(SPATIAL_GRID_EPSILON);
        let mut buckets: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();

        for (index, drill) in drills.iter().enumerate() {
            let key = center_bucket_key(drill.location, cell_size);
            buckets.entry(key).or_default().push(index);
        }

        Self {
            drills,
            buckets,
            cell_size,
            maximum_radius,
        }
    }

    /// Number of populated grid buckets, useful for trace diagnostics.
    pub(super) fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// Return later drill indexes that may violate spacing against `drill`.
    ///
    /// Returning only later indexes lets callers preserve one finding per
    /// unordered pair while avoiding a separate visited-pair set.
    pub(super) fn later_candidates_within_spacing(
        &self,
        drill_index: usize,
        spacing: f64,
    ) -> Vec<usize> {
        let drill = &self.drills[drill_index];
        let query_radius = drill.diameter / 2.0 + self.maximum_radius + spacing;
        let min_x = bucket_coordinate(drill.location[0] - query_radius, self.cell_size);
        let max_x = bucket_coordinate(drill.location[0] + query_radius, self.cell_size);
        let min_y = bucket_coordinate(drill.location[1] - query_radius, self.cell_size);
        let max_y = bucket_coordinate(drill.location[1] + query_radius, self.cell_size);
        let mut candidates = Vec::new();

        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if let Some(bucket) = self.buckets.get(&(x, y)) {
                    candidates.extend(bucket.iter().copied().filter(|&index| index > drill_index));
                }
            }
        }

        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }

    /// Return drill indexes whose centers are within `radius` of `center`.
    ///
    /// This supports cross-source drill-table matching where the exact
    /// predicate is center tolerance rather than edge spacing.
    pub(super) fn centers_within(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| {
                squared_distance(self.drills[index].location, center) <= radius * radius
            })
            .collect()
    }
}

/// Grid index over arbitrary point centers.
///
/// IPC-D-356 test records are points rather than board drill objects, but
/// drill-table consistency still matches them by center tolerance. This small
/// index keeps that cross-source lookup bounded while leaving diameter conflict
/// decisions to the exact caller-side predicate.
pub(super) struct PointSpatialIndex {
    points: Vec<[f64; 2]>,
    buckets: BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
}

impl PointSpatialIndex {
    /// Build an index over point centers using the expected query radius.
    pub(super) fn new(points: impl IntoIterator<Item = [f64; 2]>, nominal_radius: f64) -> Self {
        let cell_size = nominal_radius.max(SPATIAL_GRID_EPSILON);
        let points = points.into_iter().collect::<Vec<_>>();
        let mut buckets: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();

        for (index, point) in points.iter().enumerate() {
            buckets
                .entry(center_bucket_key(*point, cell_size))
                .or_default()
                .push(index);
        }

        Self {
            points,
            buckets,
            cell_size,
        }
    }

    /// Number of populated grid buckets, useful for trace diagnostics.
    pub(super) fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// Return point indexes whose centers are within `radius` of `center`.
    pub(super) fn centers_within(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| squared_distance(self.points[index], center) <= radius * radius)
            .collect()
    }
}

fn center_bucket_key(location: [f64; 2], cell_size: f64) -> (i64, i64) {
    (
        bucket_coordinate(location[0], cell_size),
        bucket_coordinate(location[1], cell_size),
    )
}

fn candidate_centers_within(
    buckets: &BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    center: [f64; 2],
    radius: f64,
) -> Vec<usize> {
    let min_x = bucket_coordinate(center[0] - radius, cell_size);
    let max_x = bucket_coordinate(center[0] + radius, cell_size);
    let min_y = bucket_coordinate(center[1] - radius, cell_size);
    let max_y = bucket_coordinate(center[1] + radius, cell_size);
    let mut candidates = Vec::new();

    for x in min_x..=max_x {
        for y in min_y..=max_y {
            if let Some(bucket) = buckets.get(&(x, y)) {
                candidates.extend(bucket.iter().copied());
            }
        }
    }

    candidates.sort_unstable();
    candidates.dedup();
    candidates
}

fn bucket_coordinate(value: f64, cell_size: f64) -> i64 {
    (value / cell_size).floor() as i64
}

fn squared_distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    dx * dx + dy * dy
}

fn rect_center(bounds: &geo::Rect<f64>) -> [f64; 2] {
    [
        (bounds.min().x + bounds.max().x) / 2.0,
        (bounds.min().y + bounds.max().y) / 2.0,
    ]
}

fn rect_span(bounds: &geo::Rect<f64>) -> f64 {
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    (width * width + height * height).sqrt()
}

fn feature_span(feature: &CopperFeature) -> f64 {
    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return 0.0;
    };
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    (width * width + height * height).sqrt()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{line_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{CopperFeature, CopperKind, DrillFeature};

    use super::{
        CopperSpatialIndex, DrillSpatialIndex, LayerPolygonSpatialIndex, PointSpatialIndex,
    };

    #[test]
    fn same_layer_near_feature_keeps_large_remote_centers_reachable() {
        let antenna = copper_line("ANT", "F.Cu", [-5.0, 0.0], [5.0, 0.0], 0.10);
        let nearby_large = copper_line("GPIO", "F.Cu", [5.45, 0.0], [15.45, 0.0], 0.10);
        let other_layer = copper_line("GPIO", "B.Cu", [5.45, 0.0], [15.45, 0.0], 0.10);
        let features = vec![&antenna, &nearby_large, &other_layer];
        let index = CopperSpatialIndex::new(&features, 0.25);

        let candidates = index.same_layer_near_feature(&antenna, 0.50);

        assert!(candidates.contains(&1));
        assert!(!candidates.contains(&2));
    }

    #[test]
    fn zero_radius_copper_index_uses_feature_span_sized_cells() {
        let via = copper_line("GND", "F.Cu", [0.0, 0.0], [0.1, 0.0], 0.10);
        let pad = copper_line("GND", "F.Cu", [0.0, 0.0], [0.6, 0.0], 0.60);
        let far = copper_line("GND", "F.Cu", [100.0, 0.0], [100.6, 0.0], 0.60);
        let features = vec![&pad, &far];
        let index = CopperSpatialIndex::new(&features, 0.0);

        let candidates = index.same_layer_near_feature(&via, 0.0);

        assert_eq!(candidates, vec![0]);
    }

    #[test]
    fn same_layer_centers_within_filters_by_center_distance() {
        let rf = copper_line("RF", "F.Cu", [0.0, 0.0], [1.0, 0.0], 0.10);
        let ground = copper_line("GND", "F.Cu", [0.3, 0.0], [0.4, 0.0], 0.10);
        let far_ground = copper_line("GND", "F.Cu", [2.0, 0.0], [2.1, 0.0], 0.10);
        let features = vec![&ground, &far_ground];
        let index = CopperSpatialIndex::new(&features, 0.50);

        let candidates = index.same_layer_centers_within(rf.location, &rf.layer, 0.50);

        assert_eq!(candidates, vec![0]);
    }

    #[test]
    fn all_layers_near_circle_includes_candidates_without_layer_filtering() {
        let top = copper_line("HOT", "F.Cu", [0.0, 0.0], [1.0, 0.0], 0.10);
        let bottom = copper_line("HOT", "B.Cu", [0.0, 0.2], [1.0, 0.2], 0.10);
        let far = copper_line("HOT", "B.Cu", [5.0, 0.0], [6.0, 0.0], 0.10);
        let features = vec![&top, &bottom, &far];
        let index = CopperSpatialIndex::new(&features, 0.50);

        let candidates = index.all_layers_near_circle([0.5, 0.0], 0.50);

        assert!(candidates.contains(&0));
        assert!(candidates.contains(&1));
        assert!(!candidates.contains(&2));
    }

    #[test]
    fn all_layers_near_feature_covers_cross_layer_reference_candidates() {
        let signal = copper_line("USB_D+", "F.Cu", [0.0, 0.0], [1.0, 0.0], 0.10);
        let reference = copper_line("GND", "In1.Cu", [0.0, 0.1], [1.0, 0.1], 0.10);
        let far_reference = copper_line("GND", "In1.Cu", [8.0, 0.0], [9.0, 0.0], 0.10);
        let features = vec![&reference, &far_reference];
        let index = CopperSpatialIndex::new(&features, 0.0);

        let candidates = index.all_layers_near_feature(&signal, 0.0);

        assert_eq!(candidates, vec![0]);
    }

    #[test]
    fn layer_polygon_index_culls_sparse_bounds_and_excludes_self() {
        let polygons = vec![
            rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0),
            rect_polygon([1.08, 0.0], [1.0, 1.0], 0.0),
            rect_polygon([100.0, 0.0], [1.0, 1.0], 0.0),
        ];
        let index = LayerPolygonSpatialIndex::new(&polygons, 0.10);

        assert_eq!(index.candidates_near(0, 0.10), vec![1]);
        assert_eq!(index.later_candidates_near(0, 0.10), vec![1]);
        assert!(index.candidates_near(2, 0.10).is_empty());
    }

    #[test]
    fn layer_polygon_index_accepts_external_query_polygons() {
        let polygons = vec![
            rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0),
            rect_polygon([50.0, 0.0], [1.0, 1.0], 0.0),
        ];
        let query = rect_polygon([0.8, 0.0], [0.2, 0.2], 0.0);
        let index = LayerPolygonSpatialIndex::new(&polygons, 0.0);

        assert_eq!(index.candidates_near_polygon(&query, 0.0), vec![0]);
    }

    #[test]
    fn drill_candidates_include_only_later_nearby_drills() {
        let drills = vec![
            drill([0.0, 0.0], 0.40),
            drill([0.55, 0.0], 0.40),
            drill([5.0, 0.0], 0.40),
        ];
        let index = DrillSpatialIndex::new(&drills, 0.20);

        let candidates = index.later_candidates_within_spacing(0, 0.20);

        assert_eq!(candidates, vec![1]);
        assert!(index.later_candidates_within_spacing(1, 0.20).is_empty());
    }

    #[test]
    fn drill_and_point_center_queries_filter_by_distance() {
        let drills = vec![
            drill([0.0, 0.0], 0.40),
            drill([0.05, 0.0], 0.40),
            drill([0.30, 0.0], 0.40),
        ];
        let drill_index = DrillSpatialIndex::new(&drills, 0.10);
        let point_index = PointSpatialIndex::new([[0.05, 0.0], [0.30, 0.0]], 0.10);

        assert_eq!(drill_index.centers_within([0.0, 0.0], 0.10), vec![0, 1]);
        assert_eq!(point_index.centers_within([0.0, 0.0], 0.10), vec![0]);
    }

    fn copper_line(
        net: &str,
        layer: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).expect("test line should be valid")],
                Some(LayerMetadata {
                    name: "test line".to_string(),
                }),
            ),
        }
    }

    fn drill(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: None,
            plated: true,
        }
    }
}
