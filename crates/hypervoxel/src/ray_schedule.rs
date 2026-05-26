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

/// Exact broad-phase relation between a ray, an AABB, and a retained lower
/// parameter bound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RayAabbWindowIntersection {
    /// The ray is certified disjoint from the box.
    Disjoint,
    /// The ray may intersect the box, but only before the retained lower
    /// parameter bound.
    BeforeLower,
    /// The ray may intersect or touch the box at or after the lower bound and
    /// must run the narrow phase.
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
    let interval = classify_ray_aabb_parameter_interval(origin, direction, aabb)?;
    Ok(match interval {
        RayAabbParameterInterval::Disjoint => RayAabbIntersection::Disjoint,
        RayAabbParameterInterval::Intersects { .. } => RayAabbIntersection::Intersects,
    })
}

/// Classifies whether a ray can reach an exact AABB at or after a lower
/// parameter bound.
///
/// This is a component-local row scheduling filter. A triangle AABB whose
/// entire ray interval exits strictly before the first cell center on the row
/// cannot affect any parity decision for that row segment. Equality is kept in
/// [`RayAabbWindowIntersection::Intersects`] so exact ray/triangle replay, not
/// this broad phase, owns boundary-touch refusal. This is the slab schedule of
/// Kay and Kajiya, "Ray Tracing Complex Scenes," *SIGGRAPH Computer Graphics*
/// 20(4), 1986, used under Yap's EGC rule that acceleration may only reject
/// with certified exact comparisons.
pub(crate) fn classify_ray_aabb_intersection_from_lower(
    origin: &[Real; 3],
    direction: &[Real; 3],
    aabb: &ExactAabb3,
    lower_parameter: &Real,
) -> HypervoxelResult<RayAabbWindowIntersection> {
    match classify_ray_aabb_parameter_interval(origin, direction, aabb)? {
        RayAabbParameterInterval::Disjoint => Ok(RayAabbWindowIntersection::Disjoint),
        RayAabbParameterInterval::Intersects { upper } => {
            if let Some(exit) = upper {
                if compare(&exit, lower_parameter, 0)? == Ordering::Less {
                    return Ok(RayAabbWindowIntersection::BeforeLower);
                }
            }
            Ok(RayAabbWindowIntersection::Intersects)
        }
    }
}

enum RayAabbParameterInterval {
    Disjoint,
    Intersects { upper: Option<Real> },
}

fn classify_ray_aabb_parameter_interval(
    origin: &[Real; 3],
    direction: &[Real; 3],
    aabb: &ExactAabb3,
) -> HypervoxelResult<RayAabbParameterInterval> {
    let zero = Real::from(0);
    let mut lower = Some(zero.clone());
    let mut upper: Option<Real> = None;

    for axis in 0..3 {
        match compare(&direction[axis], &zero, axis)? {
            Ordering::Equal => {
                if compare(&origin[axis], &aabb.min[axis], axis)? == Ordering::Less
                    || compare(&origin[axis], &aabb.max[axis], axis)? == Ordering::Greater
                {
                    return Ok(RayAabbParameterInterval::Disjoint);
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
                return Ok(RayAabbParameterInterval::Disjoint);
            }
        }
    }

    if let Some(exit) = &upper {
        if compare(exit, &zero, 0)? == Ordering::Less {
            return Ok(RayAabbParameterInterval::Disjoint);
        }
    }

    Ok(RayAabbParameterInterval::Intersects { upper })
}

fn compare(left: &Real, right: &Real, axis: usize) -> HypervoxelResult<Ordering> {
    hyperlimit::compare_reals(left, right)
        .value()
        .ok_or(HypervoxelError::UnknownOrdering { axis })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i64) -> Real {
        value.into()
    }

    fn aabb(min: [i64; 3], max: [i64; 3]) -> ExactAabb3 {
        ExactAabb3 {
            min: [r(min[0]), r(min[1]), r(min[2])],
            max: [r(max[0]), r(max[1]), r(max[2])],
        }
    }

    #[test]
    fn lower_window_rejects_aabb_strictly_before_component_row_segment() {
        let origin = [r(0), r(0), r(0)];
        let direction = [r(1), r(0), r(0)];
        let relation = classify_ray_aabb_intersection_from_lower(
            &origin,
            &direction,
            &aabb([1, -1, -1], [2, 1, 1]),
            &r(3),
        )
        .unwrap();

        assert_eq!(relation, RayAabbWindowIntersection::BeforeLower);
    }

    #[test]
    fn lower_window_keeps_exact_exit_equality_in_narrow_phase() {
        let origin = [r(0), r(0), r(0)];
        let direction = [r(1), r(0), r(0)];
        let relation = classify_ray_aabb_intersection_from_lower(
            &origin,
            &direction,
            &aabb([1, -1, -1], [2, 1, 1]),
            &r(2),
        )
        .unwrap();

        assert_eq!(relation, RayAabbWindowIntersection::Intersects);
    }
}
