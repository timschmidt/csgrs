//! Exact broad-phase polygon bounds queries.
//!
//! This broad phase stores exact hyperreal AABBs. The current implementation
//! is linear; callers get the correct borrowed query surface now, and the
//! storage can be replaced by a tree without changing the API.

use crate::error::HypermeshResult;
use crate::geometry::{axis_ref, compare_real};
use crate::polygon::{ApproxBounds, ConvexPolygon};

/// Bounds for one polygon in a polygon set.
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonBounds {
    /// Source polygon index in the slice used to build the structure.
    pub polygon_index: usize,
    /// Exact bounds.
    pub bounds: ApproxBounds,
}

/// Exact broad-phase acceleration structure for polygon bounds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExactBvh {
    primitives: Vec<PolygonBounds>,
}

impl ExactBvh {
    /// Builds an exact broad-phase from borrowed polygons.
    pub fn build(polygons: &[ConvexPolygon]) -> HypermeshResult<Self> {
        let mut primitives = Vec::with_capacity(polygons.len());
        for (polygon_index, polygon) in polygons.iter().enumerate() {
            primitives.push(PolygonBounds {
                polygon_index,
                bounds: polygon_bounds(polygon)?,
            });
        }
        Ok(Self { primitives })
    }

    /// Returns the number of indexed primitives.
    pub fn len(&self) -> usize {
        self.primitives.len()
    }

    /// Returns whether no primitives are indexed.
    pub fn is_empty(&self) -> bool {
        self.primitives.is_empty()
    }

    /// Returns all primitive bounds.
    pub fn primitives(&self) -> &[PolygonBounds] {
        &self.primitives
    }

    /// Calls `callback` for every primitive whose bounds overlap `bounds`.
    pub fn query_bounds<F>(&self, bounds: &ApproxBounds, mut callback: F) -> HypermeshResult<()>
    where
        F: FnMut(usize),
    {
        for primitive in &self.primitives {
            if bounds_overlap(&primitive.bounds, bounds)? {
                callback(primitive.polygon_index);
            }
        }
        Ok(())
    }

    /// Calls `callback` for every overlapping primitive pair between two
    /// broad-phase structures.
    pub fn intersect_pairs<F>(&self, other: &Self, mut callback: F) -> HypermeshResult<()>
    where
        F: FnMut(usize, usize),
    {
        for left in &self.primitives {
            for right in &other.primitives {
                if bounds_overlap(&left.bounds, &right.bounds)? {
                    callback(left.polygon_index, right.polygon_index);
                }
            }
        }
        Ok(())
    }
}

/// Returns true when two exact AABBs overlap.
pub fn bounds_overlap(left: &ApproxBounds, right: &ApproxBounds) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(axis_ref(&left.max, axis), axis_ref(&right.min, axis))?.is_lt()
            || compare_real(axis_ref(&right.max, axis), axis_ref(&left.min, axis))?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn polygon_bounds(polygon: &ConvexPolygon) -> HypermeshResult<ApproxBounds> {
    if let Some(bounds) = &polygon.approx_bounds {
        return Ok(bounds.clone());
    }

    let vertices = polygon.vertices()?;
    let refs = vertices.iter().collect::<Vec<_>>();
    Ok(ApproxBounds::for_points(&refs))
}
