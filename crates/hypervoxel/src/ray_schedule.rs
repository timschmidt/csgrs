//! Exact ray/AABB scheduling predicates for prepared voxelization.
//!
//! This module is an acceleration layer only: it may reject triangle AABBs
//! that a parity ray cannot reach, but it never accepts an inside/outside
//! decision by itself. The interval overlap test is the exact-arithmetic
//! version of the slab method from Kay and Kajiya, "Ray Tracing Complex
//! Scenes," *SIGGRAPH Computer Graphics* 20(4), 1986. Following Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997, every
//! comparison is proof-producing; an undecided comparison is reported instead
//! of being repaired with a floating tolerance.

use core::cmp::Ordering;

use hyperreal::Real;

use crate::{ExactAabb3, HypervoxelError, HypervoxelResult};

/// Exact broad-phase relation between a ray and an axis-aligned box.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RayAabbIntersection {
    /// The ray is certified disjoint from the box.
    Disjoint,
    /// The ray may intersect or touch the box and must run the narrow phase.
    Intersects,
}

/// Classifies whether a ray can reach an exact AABB.
///
/// The returned relation is intentionally conservative: touching a slab,
/// starting inside a slab, or overlapping at a single exact parameter is
/// reported as [`RayAabbIntersection::Intersects`] so the downstream
/// ray/triangle predicate owns any boundary ambiguity. Only certified empty
/// parameter intervals become [`RayAabbIntersection::Disjoint`].
pub(crate) fn classify_ray_aabb_intersection(
    origin: &[Real; 3],
    direction: &[Real; 3],
    aabb: &ExactAabb3,
) -> HypervoxelResult<RayAabbIntersection> {
    let zero = Real::from(0);
    let mut lower = Some(zero.clone());
    let mut upper: Option<Real> = None;

    for axis in 0..3 {
        match compare(&direction[axis], &zero, axis)? {
            Ordering::Equal => {
                if compare(&origin[axis], &aabb.min[axis], axis)? == Ordering::Less
                    || compare(&origin[axis], &aabb.max[axis], axis)? == Ordering::Greater
                {
                    return Ok(RayAabbIntersection::Disjoint);
                }
            }
            Ordering::Greater | Ordering::Less => {
                let t0 = ((&aabb.min[axis] - &origin[axis]) / &direction[axis]).map_err(|_| {
                    HypervoxelError::UnknownScalarOrdering {
                        field: "ray-aabb-parameter",
                    }
                })?;
                let t1 = ((&aabb.max[axis] - &origin[axis]) / &direction[axis]).map_err(|_| {
                    HypervoxelError::UnknownScalarOrdering {
                        field: "ray-aabb-parameter",
                    }
                })?;
                let (entry, exit) = if compare(&direction[axis], &zero, axis)? == Ordering::Greater
                {
                    (t0, t1)
                } else {
                    (t1, t0)
                };

                if let Some(current) = &lower {
                    if compare(&entry, current, axis)? == Ordering::Greater {
                        lower = Some(entry);
                    }
                } else {
                    lower = Some(entry);
                }

                if let Some(current) = &upper {
                    if compare(&exit, current, axis)? == Ordering::Less {
                        upper = Some(exit);
                    }
                } else {
                    upper = Some(exit);
                }
            }
        }

        if let (Some(entry), Some(exit)) = (&lower, &upper) {
            if compare(exit, entry, axis)? == Ordering::Less {
                return Ok(RayAabbIntersection::Disjoint);
            }
        }
    }

    if let Some(exit) = &upper {
        if compare(exit, &zero, 0)? == Ordering::Less {
            return Ok(RayAabbIntersection::Disjoint);
        }
    }

    Ok(RayAabbIntersection::Intersects)
}

fn compare(left: &Real, right: &Real, axis: usize) -> HypervoxelResult<Ordering> {
    hyperlimit::compare_reals(left, right)
        .value()
        .ok_or(HypervoxelError::UnknownOrdering { axis })
}
