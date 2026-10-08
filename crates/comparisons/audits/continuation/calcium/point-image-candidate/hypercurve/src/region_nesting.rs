//! Unordered native-edge input assembly for [`crate::CurveRegion2`].
//!
//! This module owns no region topology. It only proves that an unoriented
//! line/arc edge soup is a collection of endpoint-disjoint closed walks and
//! orders each walk. Interior intersections, overlaps, winding, face
//! selection, and material/hole roles are all left to the authoritative
//! `CurveRegion2` unary arrangement.

use crate::{Classification, CurveContext, Point2, Segment2, UncertaintyReason};

/// Orders an unordered native edge soup into closed walks.
///
/// Every endpoint must have exactly one other incident endpoint. This is the
/// only topology imposed by the input adapter: intersections away from source
/// endpoints deliberately remain in the curves for the general arrangement
/// kernel to split and regularize. A one-edge full circle is valid because its
/// start and end incidences match one another.
pub(crate) fn assemble_unordered_segment_rings(
    segments: &[Segment2],
    policy: &CurveContext,
) -> Result<Vec<Vec<Segment2>>, UncertaintyReason> {
    validate_endpoint_degree_two(segments, policy)?;

    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let mut current = segments[seed_index].clone();
        let ring_start = current.start().clone();
        let mut ring = vec![current.clone()];
        used[seed_index] = true;

        loop {
            match exact_points_match(current.end(), &ring_start, policy) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Err(reason),
            }

            let next = match unique_unused_incident_segment(current.end(), segments, &used, policy)
            {
                Classification::Decided(Some(next)) => next,
                Classification::Decided(None) => return Err(UncertaintyReason::Boundary),
                Classification::Uncertain(reason) => return Err(reason),
            };
            used[next.segment_index] = true;
            current = if next.reversed {
                segments[next.segment_index].reversed()
            } else {
                segments[next.segment_index].clone()
            };
            ring.push(current.clone());
        }

        canonicalize_ring_endpoints(&mut ring);
        rings.push(ring);
    }
    Ok(rings)
}

fn validate_endpoint_degree_two(
    segments: &[Segment2],
    policy: &CurveContext,
) -> Result<(), UncertaintyReason> {
    let endpoints = segments
        .iter()
        .flat_map(|segment| [segment.start(), segment.end()])
        .collect::<Vec<_>>();
    for (endpoint_index, endpoint) in endpoints.iter().enumerate() {
        let mut match_count = 0_usize;
        for (candidate_index, candidate) in endpoints.iter().enumerate() {
            if endpoint_index == candidate_index {
                continue;
            }
            match exact_points_match(endpoint, candidate, policy) {
                Classification::Decided(true) => match_count += 1,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Err(reason),
            }
        }
        if match_count != 1 {
            return Err(UncertaintyReason::Boundary);
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct NextSegment {
    segment_index: usize,
    reversed: bool,
}

fn unique_unused_incident_segment(
    target: &Point2,
    segments: &[Segment2],
    used: &[bool],
    policy: &CurveContext,
) -> Classification<Option<NextSegment>> {
    let mut selected = None;
    for (segment_index, segment) in segments.iter().enumerate() {
        if used[segment_index] {
            continue;
        }
        for (point, reversed) in [(segment.start(), false), (segment.end(), true)] {
            match exact_points_match(target, point, policy) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextSegment {
                        segment_index,
                        reversed,
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(selected)
}

/// Reuses one exact endpoint object at every certified join. This preserves
/// structural path connectivity after an `APPROXIMATE_512` terminal decides
/// equality without replacing either endpoint by a finite approximation.
fn canonicalize_ring_endpoints(ring: &mut [Segment2]) {
    for index in 1..ring.len() {
        ring[index] = segment_with_endpoints(
            &ring[index],
            ring[index - 1].end().clone(),
            ring[index].end().clone(),
        );
    }
    if let Some((first, rest)) = ring.split_first_mut() {
        let ring_start = first.start().clone();
        if let Some(last) = rest.last_mut() {
            *last = segment_with_endpoints(last, last.start().clone(), ring_start);
        } else if first.end() != &ring_start {
            *first = segment_with_endpoints(first, ring_start.clone(), ring_start);
        }
    }
}

fn segment_with_endpoints(segment: &Segment2, start: Point2, end: Point2) -> Segment2 {
    match segment {
        Segment2::Line(_) => Segment2::Line(crate::LineSeg2::new_unchecked(start, end)),
        Segment2::Arc(arc) => Segment2::Arc(crate::CircularArc2::new_unchecked_with_radius(
            start,
            end,
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )),
    }
}

fn exact_points_match(
    left: &Point2,
    right: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    match crate::classify::is_zero(&left.distance_squared(right), policy) {
        Some(value) => Classification::Decided(value),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}
