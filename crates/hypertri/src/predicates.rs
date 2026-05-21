//! Shared exact predicate helpers for triangulation topology.
//!
//! These wrappers keep algorithm modules from depending directly on predicate
//! provenance details. Exact code only consumes decided signs from the
//! crate-local kernel.

#![allow(dead_code)]

use std::cmp::Ordering;

use crate::error::Result;
use crate::kernel::Kernel;
use crate::types::{Point2, Real};
use crate::types::{Sign, TriangleLocation};

pub use hyperlimit::SegmentIntersection;

/// Decide the orientation of three points.
pub(crate) fn orient2d<K>(a: &Point2, b: &Point2, c: &Point2) -> Result<Sign>
where
    K: Kernel,
{
    K::orient2d(a, b, c)
}

/// Decide whether `point` lies inside or on the boundary of `abc`.
pub(crate) fn point_in_or_on_triangle<K>(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> Result<bool>
where
    K: Kernel,
{
    Ok(matches!(
        K::classify_point_triangle(a, b, c, point)?,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    ))
}

/// Decide whether a point lies on a closed segment.
pub(crate) fn point_on_segment<K>(a: &Point2, b: &Point2, point: &Point2) -> Result<bool>
where
    K: Kernel,
{
    if K::orient2d(a, b, point)? != Sign::Zero {
        return Ok(false);
    }

    Ok(in_closed_range::<K>(&point.x, &a.x, &b.x)? && in_closed_range::<K>(&point.y, &a.y, &b.y)?)
}

/// Decide whether a point is inside a closed ring by even-odd parity.
///
/// The ray-crossing comparison is written in orientation form so algorithms do
/// not construct inexact edge/ray intersection coordinates. This is the same
/// exactness discipline as the exact-geometric-computation model in Yap and
/// the predicate-centered approach in Shewchuk.
pub(crate) fn point_in_ring_even_odd<K>(
    vertices: &[Point2],
    ring: &[usize],
    point: &Point2,
) -> Result<bool>
where
    K: Kernel,
{
    if ring.len() < 3 {
        return Ok(false);
    }

    let mut inside = false;
    for i in 0..ring.len() {
        let a = &vertices[ring[i]];
        let b = &vertices[ring[(i + 1) % ring.len()]];

        if a == b {
            continue;
        }

        if point_on_segment::<K>(a, b, point)? {
            return Ok(true);
        }

        let a_above = K::cmp(&a.y, &point.y)? == Ordering::Greater;
        let b_above = K::cmp(&b.y, &point.y)? == Ordering::Greater;
        if a_above == b_above {
            continue;
        }

        let orientation = K::orient2d(a, b, point)?;
        let upward = K::cmp(&b.y, &a.y)? == Ordering::Greater;
        let crosses_right = matches!(
            (upward, orientation),
            (true, Sign::Positive) | (false, Sign::Negative)
        );
        if crosses_right {
            inside = !inside;
        }
    }

    Ok(inside)
}

/// Classify two closed line segments.
pub(crate) fn segment_intersection<K>(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Result<SegmentIntersection>
where
    K: Kernel,
{
    let ab_c = K::orient2d(a, b, c)?;
    let ab_d = K::orient2d(a, b, d)?;
    let cd_a = K::orient2d(c, d, a)?;
    let cd_b = K::orient2d(c, d, b)?;

    if ab_c == Sign::Zero && ab_d == Sign::Zero && cd_a == Sign::Zero && cd_b == Sign::Zero {
        return classify_collinear_segments::<K>(a, b, c, d);
    }

    if signs_strictly_differ(ab_c, ab_d) && signs_strictly_differ(cd_a, cd_b) {
        return Ok(SegmentIntersection::Proper);
    }

    if (ab_c == Sign::Zero && point_on_segment::<K>(a, b, c)?)
        || (ab_d == Sign::Zero && point_on_segment::<K>(a, b, d)?)
        || (cd_a == Sign::Zero && point_on_segment::<K>(c, d, a)?)
        || (cd_b == Sign::Zero && point_on_segment::<K>(c, d, b)?)
    {
        return Ok(SegmentIntersection::EndpointTouch);
    }

    Ok(SegmentIntersection::Disjoint)
}

fn classify_collinear_segments<K>(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Result<SegmentIntersection>
where
    K: Kernel,
{
    let use_x = K::cmp(&a.x, &b.x)? != Ordering::Equal || K::cmp(&c.x, &d.x)? != Ordering::Equal;

    let (a0, a1) = ordered_pair::<K>(
        if use_x { &a.x } else { &a.y },
        if use_x { &b.x } else { &b.y },
    )?;
    let (b0, b1) = ordered_pair::<K>(
        if use_x { &c.x } else { &c.y },
        if use_x { &d.x } else { &d.y },
    )?;

    let left = max_ref::<K>(a0, b0)?;
    let right = min_ref::<K>(a1, b1)?;

    match K::cmp(left, right)? {
        Ordering::Less => Ok(SegmentIntersection::CollinearOverlap),
        Ordering::Equal => Ok(SegmentIntersection::EndpointTouch),
        Ordering::Greater => Ok(SegmentIntersection::Disjoint),
    }
}

fn in_closed_range<K>(value: &Real, first: &Real, second: &Real) -> Result<bool>
where
    K: Kernel,
{
    let (min, max) = ordered_pair::<K>(first, second)?;
    Ok(K::cmp(value, min)? != Ordering::Less && K::cmp(value, max)? != Ordering::Greater)
}

fn ordered_pair<'a, K>(first: &'a Real, second: &'a Real) -> Result<(&'a Real, &'a Real)>
where
    K: Kernel,
{
    match K::cmp(first, second)? {
        Ordering::Greater => Ok((second, first)),
        Ordering::Less | Ordering::Equal => Ok((first, second)),
    }
}

fn max_ref<'a, K>(first: &'a Real, second: &'a Real) -> Result<&'a Real>
where
    K: Kernel,
{
    match K::cmp(first, second)? {
        Ordering::Less => Ok(second),
        Ordering::Equal | Ordering::Greater => Ok(first),
    }
}

fn min_ref<'a, K>(first: &'a Real, second: &'a Real) -> Result<&'a Real>
where
    K: Kernel,
{
    match K::cmp(first, second)? {
        Ordering::Greater => Ok(second),
        Ordering::Less | Ordering::Equal => Ok(first),
    }
}

fn signs_strictly_differ(first: Sign, second: Sign) -> bool {
    matches!(
        (first, second),
        (Sign::Negative, Sign::Positive) | (Sign::Positive, Sign::Negative)
    )
}
