//! Small spatial indexes shared by readiness checks.
//!
//! These indexes are broad-phase filters only: callers must still run their
//! exact CSG or distance predicate on every returned candidate. The design is
//! intentionally simple and deterministic because readiness reports are easier
//! to review when candidate ordering is stable across platforms and runs.

use std::collections::HashMap;

use crate::geometry::{Polygon, Rect};
use crate::kicad::{CopperFeature, DrillFeature};

const SPATIAL_GRID_EPSILON: f64 = 1.0e-9;

/// Layer-aware grid index over parsed KiCad copper features.
///
/// The index stores each feature by its parsed location and inflates queries by
/// both the queried feature span and the maximum indexed feature span. That is
/// a conservative broad-phase rule, not a proof of geometric contact. A cheap
/// spatial partition proposes candidate pairs, then exact geometry rejects
/// false positives. This keeps large sparse boards from requiring a
/// full all-pairs CSG pass. Buckets are grouped by layer so same-layer queries
/// borrow the layer key once instead of allocating one key per bucket probe.
pub(super) struct CopperSpatialIndex<'a> {
    features: &'a [&'a CopperFeature],
    buckets_by_layer: HashMap<String, HashMap<(i64, i64), Vec<usize>>>,
    all_layer_buckets: HashMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    maximum_span: Option<f64>,
}

impl<'a> CopperSpatialIndex<'a> {
    /// Build an index with a nominal query radius.
    pub(super) fn new(features: &'a [&'a CopperFeature], nominal_radius: f64) -> Self {
        let maximum_span = features
            .iter()
            .map(|feature| feature_span(feature))
            .try_fold(0.0_f64, |maximum, span| span.map(|span| maximum.max(span)));
        let cell_size = nominal_radius
            .max(maximum_span.unwrap_or(1.0))
            .max(SPATIAL_GRID_EPSILON);
        let mut buckets_by_layer: HashMap<String, HashMap<(i64, i64), Vec<usize>>> = HashMap::new();
        let mut all_layer_buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();

        for (index, feature) in features.iter().enumerate() {
            let Some(location) = feature.location_f64_compatibility() else {
                continue;
            };
            let (bucket_x, bucket_y) = center_bucket_key(location, cell_size);
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
        self.buckets_by_layer.values().map(HashMap::len).sum()
    }

    /// Return same-layer candidate features near the queried feature.
    pub(super) fn same_layer_near_feature(
        &self,
        feature: &CopperFeature,
        radius: f64,
    ) -> Vec<usize> {
        let Some((span, maximum)) = feature_span(feature).zip(self.maximum_span) else {
            return self
                .features
                .iter()
                .enumerate()
                .filter_map(|(index, other)| (other.layer == feature.layer).then_some(index))
                .collect();
        };
        let query_radius = (radius + span / 2.0 + maximum / 2.0).next_up();
        self.near_center_on_layer(
            feature.location_f64_compatibility_required(),
            &feature.layer,
            query_radius,
        )
    }

    /// Return candidate features on any layer near the queried feature.
    ///
    /// This supports checks such as reference-plane review, where signal copper
    /// on one layer is intentionally compared with ground copper on another
    /// layer. It is still only a conservative broad phase; callers must run
    /// exact CSG or distance predicates on returned candidates.
    pub(super) fn all_layers_near_feature(
        &self,
        feature: &CopperFeature,
        radius: f64,
    ) -> Vec<usize> {
        let Some((span, maximum)) = feature_span(feature).zip(self.maximum_span) else {
            return (0..self.features.len()).collect();
        };
        let query_radius = (radius + span / 2.0 + maximum / 2.0).next_up();
        self.near_center_all_layers(feature.location_f64_compatibility_required(), query_radius)
    }

    /// Return same-layer candidate centers within a circle.
    ///
    /// This preserves checks that intentionally reason about parsed locations
    /// rather than copper outlines, such as looking for a ground-stitching via
    /// near an RF feed.
    #[cfg(test)]
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
                let location = candidate.location_f64_compatibility_required();
                let dx = location[0] - center[0];
                let dy = location[1] - center[1];
                dx * dx + dy * dy <= radius_squared
            })
            .collect()
    }

    /// Return conservative same-layer grid candidates near a center.
    ///
    /// This performs no primitive circular-distance rejection. Exact callers
    /// use it so projection roundoff cannot discard a true threshold candidate
    /// before narrow-phase refinement.
    pub(super) fn same_layer_candidate_centers_near(
        &self,
        center: [f64; 2],
        layer: &str,
        radius: f64,
    ) -> Vec<usize> {
        self.near_center_on_layer(center, layer, radius)
            .into_iter()
            .filter(|&index| {
                conservative_projected_distance_may_be_within(
                    self.features[index].location_f64_compatibility_required(),
                    center,
                    radius,
                )
            })
            .collect()
    }

    /// Return candidate features on any layer near a circular keepout.
    ///
    /// Use this for source geometry that has no layer of its own, such as
    /// mechanical drills. Callers still need their exact shape predicate.
    pub(super) fn all_layers_near_circle(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        let Some(maximum) = self.maximum_span else {
            return (0..self.features.len()).collect();
        };
        let query_radius = (radius + maximum / 2.0).next_up();
        self.near_center_all_layers(center, query_radius)
    }

    fn near_center_all_layers(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        if self.maximum_span.is_none() {
            return (0..self.features.len()).collect();
        }
        candidate_centers_within(&self.all_layer_buckets, self.cell_size, center, radius)
    }

    fn near_center_on_layer(&self, center: [f64; 2], layer: &str, radius: f64) -> Vec<usize> {
        if self.maximum_span.is_none() {
            return self
                .features
                .iter()
                .enumerate()
                .filter_map(|(index, other)| (other.layer == layer).then_some(index))
                .collect();
        }
        let Some(buckets) = self.buckets_by_layer.get(layer) else {
            return Vec::new();
        };
        candidate_centers_within(buckets, self.cell_size, center, radius)
    }
}

/// Conservative grid over certified native polygon envelopes.
///
/// Each stored/query envelope covers every grid cell between its projected
/// endpoints. Bucket-coordinate conversion is monotone, so intersecting closed
/// enclosures share a bucket even when division rounds or integer keys saturate.
/// Unknown or large envelopes use an explicit fallback; report rings never cull.
pub(super) struct LayerPolygonSpatialIndex {
    buckets: HashMap<(i64, i64), Vec<usize>>,
    bounds: Vec<Option<Rect<f64>>>,
    unindexed: Vec<usize>,
    cell_size: f64,
}

impl LayerPolygonSpatialIndex {
    pub(super) fn new(polygons: &[Polygon<f64>], nominal_radius: f64) -> Self {
        let bounds = polygons
            .iter()
            .map(Polygon::conservative_bounds)
            .collect::<Vec<_>>();
        let mut spans = bounds
            .iter()
            .flatten()
            .map(|bounds| bounds.width().max(bounds.height()))
            .filter(|span| span.is_finite() && *span > 0.0)
            .collect::<Vec<_>>();
        spans.sort_unstable_by(f64::total_cmp);
        let median = spans.get(spans.len() / 2).copied().unwrap_or(1.0);
        let radius = if nominal_radius.is_finite() {
            nominal_radius.max(0.0)
        } else {
            0.0
        };
        let cell_size = median.max(radius).max(SPATIAL_GRID_EPSILON);
        let mut buckets = HashMap::<_, Vec<_>>::new();
        let mut unindexed = Vec::new();
        for (index, bound) in bounds.iter().enumerate() {
            let Some((min_x, max_x, min_y, max_y)) = bound
                .as_ref()
                .and_then(|bound| polygon_cell_range(bound, 0.0, cell_size))
            else {
                unindexed.push(index);
                continue;
            };
            for x in min_x..=max_x {
                for y in min_y..=max_y {
                    buckets.entry((x, y)).or_default().push(index);
                }
            }
        }
        Self {
            buckets,
            bounds,
            unindexed,
            cell_size,
        }
    }

    pub(super) fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    pub(super) fn candidates_near(&self, polygon_index: usize, radius: f64) -> Vec<usize> {
        let candidates = self.bounds[polygon_index].map_or_else(
            || (0..self.bounds.len()).collect(),
            |bounds| self.candidates_near_bounds(bounds, radius),
        );
        candidates
            .into_iter()
            .filter(|&index| index != polygon_index)
            .collect()
    }

    pub(super) fn candidates_near_polygon(
        &self,
        polygon: &Polygon<f64>,
        radius: f64,
    ) -> Vec<usize> {
        polygon.conservative_bounds().map_or_else(
            || (0..self.bounds.len()).collect(),
            |bounds| self.candidates_near_bounds(bounds, radius),
        )
    }

    pub(super) fn later_candidates_near(&self, polygon_index: usize, radius: f64) -> Vec<usize> {
        self.candidates_near(polygon_index, radius)
            .into_iter()
            .filter(|&index| index > polygon_index)
            .collect()
    }

    fn candidates_near_bounds(&self, bounds: Rect<f64>, radius: f64) -> Vec<usize> {
        let Some((min_x, max_x, min_y, max_y)) =
            polygon_cell_range(&bounds, radius, self.cell_size)
        else {
            return (0..self.bounds.len()).collect();
        };
        let mut candidates = self.unindexed.clone();
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if let Some(bucket) = self.buckets.get(&(x, y)) {
                    candidates.extend(bucket.iter().copied());
                }
            }
        }
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }
}

fn polygon_cell_range(
    bounds: &Rect<f64>,
    radius: f64,
    cell_size: f64,
) -> Option<(i64, i64, i64, i64)> {
    if !radius.is_finite() {
        return None;
    }
    let radius = radius.max(0.0);
    let lower = |value: f64| {
        if radius == 0.0 {
            value
        } else {
            (value - radius).next_down()
        }
    };
    let upper = |value: f64| {
        if radius == 0.0 {
            value
        } else {
            (value + radius).next_up()
        }
    };
    let coordinates = [
        lower(bounds.min().x),
        upper(bounds.max().x),
        lower(bounds.min().y),
        upper(bounds.max().y),
    ];
    if !coordinates.iter().all(|coordinate| coordinate.is_finite()) {
        return None;
    }
    let [min_x, max_x, min_y, max_y] = coordinates.map(|value| bucket_coordinate(value, cell_size));
    let width = i128::from(max_x) - i128::from(min_x) + 1;
    let height = i128::from(max_y) - i128::from(min_y) + 1;
    // Bound scheduling work without dropping large or unrepresentable boxes.
    (width > 0 && height > 0 && width.saturating_mul(height) <= 256)
        .then_some((min_x, max_x, min_y, max_y))
}

/// Grid index over drill and slot proxy centers.
///
/// Drill spacing is a center-distance predicate adjusted by both drill radii,
/// so a center-only broad phase is sufficient as long as each query is inflated
/// by the largest indexed radius plus the requested spacing. The index only
/// proposes candidate pairs, while the caller
/// remains responsible for exact edge-gap comparison.
pub(super) struct DrillSpatialIndex<'a> {
    drills: &'a [DrillFeature],
    buckets: HashMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    maximum_radius: f64,
}

impl<'a> DrillSpatialIndex<'a> {
    /// Build an index for drill spacing queries.
    pub(super) fn new(drills: &'a [DrillFeature], nominal_spacing: f64) -> Self {
        let maximum_radius = drills
            .iter()
            .filter_map(DrillFeature::diameter_f64_compatibility)
            .map(|diameter| diameter / 2.0)
            .fold(0.0_f64, f64::max);
        let cell_size = (maximum_radius * 2.0 + nominal_spacing).max(SPATIAL_GRID_EPSILON);
        let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();

        for (index, drill) in drills.iter().enumerate() {
            let key = center_bucket_key(drill.location_f64_compatibility_required(), cell_size);
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
        let query_radius =
            drill.diameter_f64_compatibility().unwrap_or(0.0) / 2.0 + self.maximum_radius + spacing;
        let min_x = bucket_coordinate(
            drill.location_f64_compatibility_required()[0] - query_radius,
            self.cell_size,
        );
        let max_x = bucket_coordinate(
            drill.location_f64_compatibility_required()[0] + query_radius,
            self.cell_size,
        );
        let min_y = bucket_coordinate(
            drill.location_f64_compatibility_required()[1] - query_radius,
            self.cell_size,
        );
        let max_y = bucket_coordinate(
            drill.location_f64_compatibility_required()[1] + query_radius,
            self.cell_size,
        );
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
    #[cfg(test)]
    pub(super) fn centers_within(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| {
                squared_distance(
                    self.drills[index].location_f64_compatibility_required(),
                    center,
                ) <= radius * radius
            })
            .collect()
    }

    /// Return conservative drill-center candidates without a primitive
    /// circular-distance rejection.
    pub(super) fn candidate_centers_near(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| {
                conservative_projected_distance_may_be_within(
                    self.drills[index].location_f64_compatibility_required(),
                    center,
                    radius,
                )
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
    buckets: HashMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
}

impl PointSpatialIndex {
    /// Build an index over point centers using the expected query radius.
    pub(super) fn new(points: impl IntoIterator<Item = [f64; 2]>, nominal_radius: f64) -> Self {
        let cell_size = nominal_radius.max(SPATIAL_GRID_EPSILON);
        let points = points.into_iter().collect::<Vec<_>>();
        let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();

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
    #[cfg(test)]
    pub(super) fn centers_within(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| squared_distance(self.points[index], center) <= radius * radius)
            .collect()
    }

    /// Return point indexes from buckets intersecting the query square.
    ///
    /// This broad phase deliberately does not decide the circular center-radius
    /// predicate. Callers that need certified topology or
    /// source-unit-aware distance comparisons must refine every returned
    /// candidate with their exact predicate.
    pub(super) fn candidate_centers_near(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        candidate_centers_within(&self.buckets, self.cell_size, center, radius)
            .into_iter()
            .filter(|&index| {
                conservative_projected_distance_may_be_within(self.points[index], center, radius)
            })
            .collect()
    }

    /// Return a stored point by index for narrow-phase refinement.
    pub(super) fn point(&self, index: usize) -> [f64; 2] {
        self.points[index]
    }
}

fn center_bucket_key(location: [f64; 2], cell_size: f64) -> (i64, i64) {
    (
        bucket_coordinate(location[0], cell_size),
        bucket_coordinate(location[1], cell_size),
    )
}

fn candidate_centers_within(
    buckets: &HashMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    center: [f64; 2],
    radius: f64,
) -> Vec<usize> {
    let min_x = bucket_coordinate(center[0] - radius, cell_size);
    let max_x = bucket_coordinate(center[0] + radius, cell_size);
    let min_y = bucket_coordinate(center[1] - radius, cell_size);
    let max_y = bucket_coordinate(center[1] + radius, cell_size);
    let x_count = i128::from(max_x) - i128::from(min_x) + 1;
    let y_count = i128::from(max_y) - i128::from(min_y) + 1;
    if !radius.is_finite()
        || !center.into_iter().all(f64::is_finite)
        || x_count.saturating_mul(y_count) > (buckets.len().max(1) as i128) * 16
    {
        let mut all = buckets.values().flatten().copied().collect::<Vec<_>>();
        all.sort_unstable();
        all.dedup();
        return all;
    }
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

/// Conservatively filter a projected center-distance candidate.
///
/// Each finite projection is treated as a one-ULP interval around the stored
/// primitive. The lower-bound axis separations can therefore reject only pairs
/// whose exact coordinates cannot be within the projected search radius. This
/// remains a non-certifying broad phase; callers perform the exact predicate.
fn conservative_projected_distance_may_be_within(
    left: [f64; 2],
    right: [f64; 2],
    radius: f64,
) -> bool {
    if !left.into_iter().chain(right).all(f64::is_finite) || !radius.is_finite() {
        return true;
    }
    let lower_dx =
        ((left[0] - right[0]).abs() - projection_ulp(left[0]) - projection_ulp(right[0])).max(0.0);
    let lower_dy =
        ((left[1] - right[1]).abs() - projection_ulp(left[1]) - projection_ulp(right[1])).max(0.0);
    let expanded_radius = radius.abs() + projection_ulp(radius);
    lower_dx * lower_dx + lower_dy * lower_dy <= expanded_radius * expanded_radius
}

fn projection_ulp(value: f64) -> f64 {
    (value.next_up() - value)
        .abs()
        .max((value - value.next_down()).abs())
}

#[cfg(test)]
fn squared_distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    dx * dx + dy * dy
}

pub(super) fn feature_span(feature: &CopperFeature) -> Option<f64> {
    let bounds = feature.region.conservative_bounds()?;
    let center = feature.location_f64_compatibility()?;
    // Cover the entire native envelope around the authored anchor, which need
    // not coincide with the geometric center (for example, custom artwork).
    let dx = (bounds.max().x - center[0])
        .abs()
        .max((bounds.min().x - center[0]).abs())
        .next_up();
    let dy = (bounds.max().y - center[1])
        .abs()
        .max((bounds.min().y - center[1]).abs())
        .next_up();
    let span = (2.0 * dx.hypot(dy).next_up()).next_up();
    span.is_finite().then_some(span)
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{line_polygon, polygons_to_profile, rect_polygon};
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

        let candidates = index.same_layer_centers_within(
            rf.location_f64_compatibility_required(),
            &rf.layer,
            0.50,
        );

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
    fn hash_bucket_queries_keep_sorted_deterministic_candidate_order() {
        let polygons = vec![
            rect_polygon([2.0, 0.0], [0.2, 0.2], 0.0),
            rect_polygon([0.0, 0.0], [0.2, 0.2], 0.0),
            rect_polygon([1.0, 0.0], [0.2, 0.2], 0.0),
        ];

        for _ in 0..32 {
            let index = LayerPolygonSpatialIndex::new(&polygons, 0.2);
            assert_eq!(
                index.candidates_near_polygon(&polygons[1], 2.0),
                vec![0, 1, 2]
            );
        }
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
            location: [
                crate::geometry::exact_real((start[0] + end[0]) / 2.0),
                crate::geometry::exact_real((start[1] + end[1]) / 2.0),
            ],
            region: polygons_to_profile(
                vec![line_polygon(start, end, width).expect("test line should be valid")],
                Some(LayerMetadata {
                    name: "test line".to_string(),
                }),
            ),
        }
    }

    fn drill(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: crate::geometry::exact_real(diameter),
            net: None,
            plated: true,
        }
    }
}

#[cfg(test)]
mod projection_index_regressions {
    use super::CopperSpatialIndex;
    use crate::geometry::{polygons_to_profile, rect_polygon};
    use crate::kicad::{CopperFeature, CopperKind};
    use crate::{PcbRegion, Scalar};

    fn feature(region: PcbRegion, layer: &str, anchor_x: i32) -> CopperFeature {
        CopperFeature {
            layer: layer.into(),
            net: Some("test".into()),
            kind: CopperKind::Artwork,
            region,
            location: [Scalar::from(anchor_x), Scalar::zero()],
        }
    }

    #[test]
    fn missing_native_bounds_keep_all_same_layer_candidates() {
        let make = || polygons_to_profile(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)], None);
        let known = feature(make(), "F.Cu", 0);
        let unknown = feature(
            make().with_exact_construction_error("retained native failure"),
            "F.Cu",
            1_000,
        );
        let other_layer = feature(make(), "B.Cu", 0);
        let features = [&unknown, &other_layer];
        let index = CopperSpatialIndex::new(&features, 0.0);
        assert_eq!(index.same_layer_near_feature(&known, 0.0), [0]);
        assert_eq!(index.all_layers_near_feature(&known, 0.0), [0, 1]);
        let known_features = [&known];
        let known_index = CopperSpatialIndex::new(&known_features, 0.0);
        assert_eq!(known_index.same_layer_near_feature(&unknown, 0.0), [0]);
    }

    #[test]
    fn native_extent_is_measured_about_the_authored_anchor() {
        let shape = || polygons_to_profile(vec![rect_polygon([100.0, 0.0], [1.0, 1.0], 0.0)], None);
        let query = feature(shape(), "F.Cu", 100);
        let displaced_anchor = feature(shape(), "F.Cu", 0);
        let features = [&displaced_anchor];
        let index = CopperSpatialIndex::new(&features, 0.0);
        assert_eq!(index.same_layer_near_feature(&query, 0.0), [0]);
    }
}
