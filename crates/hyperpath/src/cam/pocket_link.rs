//! Exact retained link graph for rectangular pocket schedules.
//!
//! This module stays deliberately on the path side of the Hyper split. It
//! turns a retained contour-parallel rectangular pocket schedule into exact
//! boundary segments plus exact connector candidates, but it does not perform
//! stock clipping, cutter engagement analysis, gouge detection, or mesh/solid
//! materialization. The distinction follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): construct exact
//! objects first, then expose the predicates needed to certify whether the
//! objects may become output. The contour/link separation is also consistent
//! with contour-parallel pocketing treatments such as Held, "On the
//! Computational Geometry of Pocket Machining" (1991), where offset contours
//! and linking moves are separate algorithmic objects.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::cam::{PocketOffsetRing, RectangularPocketPlan};
use crate::segment::LinePathSegment;

/// Axis-aligned side of a retained rectangular pocket ring.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PocketRingSide {
    /// Low-Y side from minimum X to maximum X.
    MinY,
    /// High-X side from minimum Y to maximum Y.
    MaxX,
    /// High-Y side from maximum X to minimum X.
    MaxY,
    /// Low-X side from maximum Y to minimum Y.
    MinX,
}

/// One exact boundary segment of a scheduled rectangular pocket ring.
///
/// The segment is a retained path-domain source record. It carries the ring
/// index and side so a downstream `hypermesh` intake can build or reject solid
/// topology while preserving source identity.
#[derive(Clone, Debug, PartialEq)]
pub struct PocketRingSegment {
    /// Source ring index from the pocket schedule.
    pub ring_index: usize,
    /// Rectangular side represented by `segment`.
    pub side: PocketRingSide,
    /// Exact side segment.
    pub segment: LinePathSegment,
}

/// One exact connector candidate between two adjacent pocket rings.
///
/// Connectors are emitted as axis-aligned dogleg legs between lower-left ring
/// corners. They are candidates only: feed, ramping, cutter radius, gouge, and
/// rest-material predicates still have to certify whether a CAM process may
/// use them.
#[derive(Clone, Debug, PartialEq)]
pub struct PocketLinkSegment {
    /// Outer/source ring index.
    pub from_ring: usize,
    /// Inner/target ring index.
    pub to_ring: usize,
    /// Zero-based leg index within the dogleg connector.
    pub leg_index: usize,
    /// Exact connector segment.
    pub segment: LinePathSegment,
}

/// Exact retained link graph over a rectangular pocket schedule.
///
/// `ring_segments` contains four oriented side segments for every positive-area
/// ring. `links` contains exact connector legs between adjacent rings. The
/// graph is intentionally a source graph rather than an accepted toolpath:
/// later exact arrangements, rest-material predicates, process constraints,
/// and mesh-domain intake decide whether and how these retained paths are used.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularPocketLinkGraph {
    /// Source pocket schedule.
    pub plan: RectangularPocketPlan,
    /// Exact boundary segments for every scheduled ring.
    pub ring_segments: Vec<PocketRingSegment>,
    /// Exact connector legs between adjacent rings.
    pub links: Vec<PocketLinkSegment>,
}

/// Errors while constructing retained rectangular pocket link graphs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PocketLinkGraphError {
    /// No ring was available to link.
    EmptyPlan,
    /// A scheduled ring index did not match its position in the plan.
    InvalidRingIndex,
    /// A scheduled ring did not have positive area.
    DegenerateRing,
    /// Adjacent rings were not exactly certified as nested.
    NonNestedRings,
    /// Exact comparison could not decide a required predicate.
    UnknownComparison,
    /// A generated connector endpoint failed exact equality validation.
    InvalidConnectorEndpoint,
}

/// Build an exact retained link graph from a rectangular pocket schedule.
///
/// The builder validates that every scheduled ring has positive extent and
/// that each adjacent pair is exactly nested. It then emits four exact
/// side-segments per ring plus a deterministic lower-left dogleg between
/// adjacent rings. The dogleg is represented as one or two axis-aligned
/// `LinePathSegment`s; zero-length legs are skipped after exact comparison.
///
/// This is not a pocketing executor. It is the path-domain graph carrier that a
/// later arrangement or hypermesh intake can certify, reject, or transform.
pub fn build_rectangular_pocket_link_graph(
    plan: RectangularPocketPlan,
    policy: PredicatePolicy,
) -> Result<RectangularPocketLinkGraph, PocketLinkGraphError> {
    if plan.rings.is_empty() {
        return Err(PocketLinkGraphError::EmptyPlan);
    }

    for (expected, ring) in plan.rings.iter().enumerate() {
        if ring.index != expected {
            return Err(PocketLinkGraphError::InvalidRingIndex);
        }
        validate_positive_ring(ring, policy)?;
    }
    for pair in plan.rings.windows(2) {
        validate_nested_rings(&pair[0], &pair[1], policy)?;
    }

    let mut ring_segments = Vec::with_capacity(plan.rings.len() * 4);
    for ring in &plan.rings {
        ring_segments.extend(ring_boundary_segments(ring, plan.pocket.provenance()));
    }

    let mut links = Vec::new();
    for pair in plan.rings.windows(2) {
        links.extend(lower_left_dogleg(
            &pair[0],
            &pair[1],
            plan.pocket.provenance(),
            policy,
        )?);
    }

    Ok(RectangularPocketLinkGraph {
        plan,
        ring_segments,
        links,
    })
}

fn ring_boundary_segments(
    ring: &PocketOffsetRing,
    provenance: crate::provenance::PathProvenance,
) -> [PocketRingSegment; 4] {
    let min_min = ring.min.clone();
    let max_min = Point2::new(ring.max.x.clone(), ring.min.y.clone());
    let max_max = ring.max.clone();
    let min_max = Point2::new(ring.min.x.clone(), ring.max.y.clone());
    [
        PocketRingSegment {
            ring_index: ring.index,
            side: PocketRingSide::MinY,
            segment: LinePathSegment::with_provenance(min_min.clone(), max_min.clone(), provenance),
        },
        PocketRingSegment {
            ring_index: ring.index,
            side: PocketRingSide::MaxX,
            segment: LinePathSegment::with_provenance(max_min, max_max.clone(), provenance),
        },
        PocketRingSegment {
            ring_index: ring.index,
            side: PocketRingSide::MaxY,
            segment: LinePathSegment::with_provenance(max_max, min_max.clone(), provenance),
        },
        PocketRingSegment {
            ring_index: ring.index,
            side: PocketRingSide::MinX,
            segment: LinePathSegment::with_provenance(min_max, min_min, provenance),
        },
    ]
}

fn lower_left_dogleg(
    outer: &PocketOffsetRing,
    inner: &PocketOffsetRing,
    provenance: crate::provenance::PathProvenance,
    policy: PredicatePolicy,
) -> Result<Vec<PocketLinkSegment>, PocketLinkGraphError> {
    let bend = Point2::new(inner.min.x.clone(), outer.min.y.clone());
    let mut links = Vec::with_capacity(2);
    push_link_leg(
        &mut links,
        outer.index,
        inner.index,
        0,
        outer.min.clone(),
        bend.clone(),
        provenance,
        policy,
    )?;
    push_link_leg(
        &mut links,
        outer.index,
        inner.index,
        1,
        bend,
        inner.min.clone(),
        provenance,
        policy,
    )?;
    if !links.is_empty()
        && (!points_equal(links.first().unwrap().segment.start(), &outer.min, policy)
            || !points_equal(links.last().unwrap().segment.end(), &inner.min, policy))
    {
        return Err(PocketLinkGraphError::InvalidConnectorEndpoint);
    }
    Ok(links)
}

fn push_link_leg(
    links: &mut Vec<PocketLinkSegment>,
    from_ring: usize,
    to_ring: usize,
    leg_index: usize,
    start: Point2,
    end: Point2,
    provenance: crate::provenance::PathProvenance,
    policy: PredicatePolicy,
) -> Result<(), PocketLinkGraphError> {
    if points_equal(&start, &end, policy) {
        return Ok(());
    }
    if !same_axis(&start, &end, policy)? {
        return Err(PocketLinkGraphError::InvalidConnectorEndpoint);
    }
    let segment = LinePathSegment::with_provenance(start.clone(), end.clone(), provenance);
    if !points_equal(segment.start(), &start, policy) || !points_equal(segment.end(), &end, policy)
    {
        return Err(PocketLinkGraphError::InvalidConnectorEndpoint);
    }
    links.push(PocketLinkSegment {
        from_ring,
        to_ring,
        leg_index,
        segment,
    });
    Ok(())
}

fn validate_positive_ring(
    ring: &PocketOffsetRing,
    policy: PredicatePolicy,
) -> Result<(), PocketLinkGraphError> {
    if compare(&ring.min.x, &ring.max.x, policy)? != Ordering::Less
        || compare(&ring.min.y, &ring.max.y, policy)? != Ordering::Less
    {
        return Err(PocketLinkGraphError::DegenerateRing);
    }
    Ok(())
}

fn validate_nested_rings(
    outer: &PocketOffsetRing,
    inner: &PocketOffsetRing,
    policy: PredicatePolicy,
) -> Result<(), PocketLinkGraphError> {
    let nested = [
        compare(&outer.min.x, &inner.min.x, policy)?,
        compare(&outer.min.y, &inner.min.y, policy)?,
        compare(&inner.max.x, &outer.max.x, policy)?,
        compare(&inner.max.y, &outer.max.y, policy)?,
    ];
    if nested
        .into_iter()
        .all(|ordering| matches!(ordering, Ordering::Less | Ordering::Equal))
    {
        Ok(())
    } else {
        Err(PocketLinkGraphError::NonNestedRings)
    }
}

fn same_axis(
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PocketLinkGraphError> {
    Ok(compare(&start.x, &end.x, policy)? == Ordering::Equal
        || compare(&start.y, &end.y, policy)? == Ordering::Equal)
}

fn points_equal(first: &Point2, second: &Point2, policy: PredicatePolicy) -> bool {
    compare_reals_with_policy(&first.x, &second.x, policy).value() == Some(Ordering::Equal)
        && compare_reals_with_policy(&first.y, &second.y, policy).value() == Some(Ordering::Equal)
}

fn compare(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Result<Ordering, PocketLinkGraphError> {
    compare_reals_with_policy(first, second, policy)
        .value()
        .ok_or(PocketLinkGraphError::UnknownComparison)
}
