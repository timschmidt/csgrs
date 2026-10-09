//! Conservative candidate scheduling shared by native polygons and points.

use std::collections::HashMap;

use crate::geometry::Rect;

use super::{SPATIAL_GRID_EPSILON, bucket_coordinate};

/// Every certified enclosure occupies all cells between its endpoint keys.
/// Monotone conversion preserves closed contacts despite rounding/saturation.
/// Unknown or oversized enclosures remain explicit fallback candidates.
pub(super) struct EnvelopeSpatialIndex {
    buckets: HashMap<(i64, i64), Vec<usize>>,
    bounds: Vec<Option<Rect<f64>>>,
    unindexed: Vec<usize>,
    cell_size: f64,
}

impl EnvelopeSpatialIndex {
    pub(super) fn new(bounds: Vec<Option<Rect<f64>>>, nominal_radius: f64) -> Self {
        let mut spans = bounds
            .iter()
            .flatten()
            .map(|bounds| bounds.width().max(bounds.height()))
            .filter(|span| span.is_finite() && *span > 0.0)
            .collect::<Vec<_>>();
        spans.sort_unstable_by(f64::total_cmp);
        let median = spans.get(spans.len() / 2).copied().unwrap_or(0.0);
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
                .and_then(|bound| cell_range(bound, 0.0, cell_size))
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

    pub(super) fn near_index(&self, index: usize, radius: f64) -> Vec<usize> {
        self.near_enclosure(self.bounds[index], radius)
    }

    pub(super) fn near_enclosure(&self, bounds: Option<Rect<f64>>, radius: f64) -> Vec<usize> {
        let Some((min_x, max_x, min_y, max_y)) =
            bounds.and_then(|bounds| cell_range(&bounds, radius, self.cell_size))
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

fn cell_range(bounds: &Rect<f64>, radius: f64, cell_size: f64) -> Option<(i64, i64, i64, i64)> {
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
