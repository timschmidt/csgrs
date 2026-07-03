//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, compare_real,
};
use crate::polygon::ConvexPolygon;
use crate::winding::{WindingNumberTransitionVector, WindingNumberVector};

/// Result of tracing one axis-aligned segment.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceAxisSegmentResult {
    /// Winding number after accepted crossings.
    pub winding: WindingNumberVector,
    /// Whether the path avoided exact edge hits.
    pub valid: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct CrossingEvent {
    point: Point3,
    support: Plane,
    normal_sign: i32,
    cross_sign: i32,
    delta_w: WindingNumberTransitionVector,
    on_edge: bool,
}

/// Traces an axis-aligned segment, accumulating polygon winding transitions.
pub fn trace_axis_segment(
    start: &Point3,
    end: &Point3,
    axis: usize,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let direction = compare_real(axis_ref(end, axis), axis_ref(start, axis))?;
    if direction.is_eq() {
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: true,
        });
    }

    let dir_sign = if direction.is_gt() { 1 } else { -1 };
    let mut events = Vec::new();
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let normal_axis = axis_ref(&polygon.support.normal, axis);
        if normal_axis.definitely_zero() {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        if !point_strictly_between_axis(&crossing, start, end, axis)? {
            continue;
        }

        let mut inside = true;
        let mut on_edge = false;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => on_edge = true,
                Classification::Negative => {}
            }
        }
        if !inside {
            continue;
        }

        let normal_sign = match crate::geometry::classify_real(normal_axis)? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        let cross_sign = normal_sign * -dir_sign;
        events.push(CrossingEvent {
            point: crossing,
            support: polygon.support.clone(),
            normal_sign,
            cross_sign,
            delta_w: polygon.delta_w.clone(),
            on_edge,
        });
    }

    let mut accepted: Vec<CrossingEvent> = Vec::new();
    for (index, event) in events.iter().enumerate() {
        if event.on_edge
            && !events.iter().enumerate().any(|(other_index, other)| {
                other_index != index
                    && other.point == event.point
                    && other.support == event.support
                    && other.normal_sign == event.normal_sign
                    && other.delta_w == event.delta_w
            })
        {
            return Ok(TraceAxisSegmentResult {
                winding,
                valid: false,
            });
        }

        if accepted.iter().any(|existing| {
            existing.point == event.point
                && existing.support == event.support
                && existing.normal_sign == event.normal_sign
                && existing.delta_w == event.delta_w
        }) {
            continue;
        }

        accepted.push(event.clone());
    }

    sort_crossing_events(&mut accepted, axis, dir_sign)?;

    for event in accepted {
        for (value, delta) in winding.iter_mut().zip(&event.delta_w) {
            *value += event.cross_sign * *delta;
        }
    }

    Ok(TraceAxisSegmentResult {
        winding,
        valid: true,
    })
}

/// Traces an L-shaped path using several axis orderings and returns the first
/// valid winding result.
pub fn trace_segment(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    const ORDERINGS: [[usize; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];

    for ordering in ORDERINGS {
        let mut current = start.clone();
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for axis in ordering {
            if compare_real(axis_ref(&current, axis), axis_ref(end, axis))?.is_ne() {
                let mut next = current.clone();
                *axis_mut(&mut next, axis) = axis_ref(end, axis).clone();
                if next != *end && point_lies_on_traced_surface(&next, polygons)? {
                    valid = false;
                    break;
                }
                let traced = trace_axis_segment(&current, &next, axis, &attempt, polygons)?;
                attempt = traced.winding;
                valid = traced.valid;
                current = next;
                if !valid {
                    break;
                }
            }
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn point_lies_on_traced_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        if classify_point(point, &polygon.support)? != Classification::On {
            continue;
        }

        let mut inside_polygon = true;
        for edge in &polygon.edges {
            if classify_point(point, edge)? == Classification::Positive {
                inside_polygon = false;
                break;
            }
        }
        if inside_polygon {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Finds a probe point off a polygon surface and reports its side.
pub fn find_probe_point(
    polygon: &ConvexPolygon,
) -> HypermeshResult<Option<(Point3, Classification)>> {
    find_probe_point_on_side(polygon, true)
}

fn find_probe_point_on_side(
    polygon: &ConvexPolygon,
    positive_side: bool,
) -> HypermeshResult<Option<(Point3, Classification)>> {
    if polygon.vertex_count() < 3 {
        return Ok(None);
    }

    let vertices = polygon.vertices()?;
    let center = centroid(&vertices).ok_or(HypermeshError::EmptyInput)?;
    let axis = dominant_normal_axis(&polygon.support)?;
    let normal_sign = crate::geometry::classify_real(axis_ref(&polygon.support.normal, axis))?;
    if normal_sign == Classification::On {
        return Ok(None);
    }

    let offset = probe_offset(&vertices, axis)?;
    let mut probe = center;
    let signed_offset = if (normal_sign == Classification::Positive) == positive_side {
        offset
    } else {
        -offset
    };
    *axis_mut(&mut probe, axis) = axis_ref(&probe, axis) + &signed_offset;

    let mut side = classify_point(&probe, &polygon.support)?;
    if side == Classification::On {
        *axis_mut(&mut probe, axis) = axis_ref(&probe, axis) + &signed_offset;
        side = classify_point(&probe, &polygon.support)?;
    }

    if side == Classification::On {
        Ok(None)
    } else {
        Ok(Some((probe, side)))
    }
}

/// Classifies a leaf polygon by tracing from a reference point to an off-face
/// probe and applying the host transition correction.
pub fn classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: leaf_edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        no_self_intersections: false,
        no_nested_components: false,
        approx_bounds: None,
    };

    let interior_points = interior_leaf_points(&leaf)?;
    for point in &interior_points {
        for positive_side in [true, false] {
            for (probe, probe_side) in
                bounded_probes_from_interior(point, support, bounds, positive_side)?
            {
                if point_lies_on_traced_surface(&probe, polygons)? {
                    continue;
                }
                let Ok(mut winding) = trace_segment(ref_point, &probe, ref_wnv, polygons) else {
                    continue;
                };
                if probe_side == Classification::Negative {
                    for (value, delta) in winding.iter_mut().zip(host_delta_w) {
                        *value -= *delta;
                    }
                }
                return Ok(winding);
            }
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn segment_plane_crossing(
    start: &Point3,
    end: &Point3,
    plane: &Plane,
) -> HypermeshResult<Option<Point3>> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let start_class = crate::geometry::classify_real(&start_value)?;
    let end_class = crate::geometry::classify_real(&end_value)?;

    if start_class == Classification::On || end_class == Classification::On {
        return Ok(None);
    }
    if start_class == end_class {
        return Ok(None);
    }

    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;
    Ok(Some(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    )))
}

fn point_strictly_between_axis(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    axis: usize,
) -> HypermeshResult<bool> {
    let start_to_point = compare_real(axis_ref(point, axis), axis_ref(start, axis))?;
    let point_to_end = compare_real(axis_ref(point, axis), axis_ref(end, axis))?;
    Ok((start_to_point.is_gt() && point_to_end.is_lt())
        || (start_to_point.is_lt() && point_to_end.is_gt()))
}

fn sort_crossing_events(
    events: &mut Vec<CrossingEvent>,
    axis: usize,
    dir_sign: i32,
) -> HypermeshResult<()> {
    let mut sorted: Vec<CrossingEvent> = Vec::with_capacity(events.len());
    for event in events.drain(..) {
        let mut insert_at = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                axis_ref(&event.point, axis),
                axis_ref(&existing.point, axis),
            )?;
            if (dir_sign > 0 && order.is_lt()) || (dir_sign < 0 && order.is_gt()) {
                insert_at = index;
                break;
            }
        }
        sorted.insert(insert_at, event);
    }
    *events = sorted;
    Ok(())
}

fn dominant_normal_axis(plane: &Plane) -> HypermeshResult<usize> {
    let abs = [
        plane.normal.x.clone().abs(),
        plane.normal.y.clone().abs(),
        plane.normal.z.clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn centroid(points: &[Point3]) -> Option<Point3> {
    if points.is_empty() {
        return None;
    }
    let mut sum = Point3::origin();
    for point in points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Some(Point3::new(
        (sum.x / denom.clone()).expect("point count is non-zero"),
        (sum.y / denom.clone()).expect("point count is non-zero"),
        (sum.z / denom).expect("point count is non-zero"),
    ))
}

fn interior_leaf_points(leaf: &ConvexPolygon) -> HypermeshResult<Vec<Point3>> {
    let vertices = leaf.vertices()?;
    let Some(center) = centroid(&vertices) else {
        return Ok(Vec::new());
    };

    let mut points = Vec::with_capacity(vertices.len() + 1);
    if point_strictly_inside_leaf(&center, leaf)? {
        points.push(center.clone());
    }

    let n = Real::from(vertices.len() as u64);
    let denom = &n + &Real::one();
    for vertex in vertices {
        let candidate = Point3::new(
            (((&center.x * &n) + vertex.x) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
            (((&center.y * &n) + vertex.y) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
            (((&center.z * &n) + vertex.z) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
        );
        if point_strictly_inside_leaf(&candidate, leaf)? {
            points.push(candidate);
        }
    }

    Ok(points)
}

fn point_strictly_inside_leaf(point: &Point3, leaf: &ConvexPolygon) -> HypermeshResult<bool> {
    let homogeneous = HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    leaf.contains_point_strictly(&homogeneous)
}

fn bounded_probes_from_interior(
    interior: &Point3,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
) -> HypermeshResult<Vec<(Point3, Classification)>> {
    let mut probes = Vec::new();
    let fractions = [(1u64, 2u64), (1, 3), (2, 3), (1, 4), (3, 4)];

    for axis in probe_axes(support)? {
        let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
        if normal_sign == Classification::On {
            continue;
        }

        let direction_positive = (normal_sign == Classification::Positive) == positive_side;
        let axis_value = axis_ref(interior, axis);
        let room = if direction_positive {
            axis_ref(&bounds.max, axis) - axis_value
        } else {
            axis_value - axis_ref(&bounds.min, axis)
        };
        if !compare_real(&room, &Real::zero())?.is_gt() {
            continue;
        }

        for (numerator, denominator) in fractions {
            let offset = ((room.clone() * Real::from(numerator)) / Real::from(denominator))
                .map_err(|_| HypermeshError::UnknownClassification)?;
            let mut probe = interior.clone();
            *axis_mut(&mut probe, axis) = if direction_positive {
                axis_value + &offset
            } else {
                axis_value - &offset
            };

            let side = classify_point(&probe, support)?;
            if side != Classification::On
                && !probes
                    .iter()
                    .any(|(existing, _): &(Point3, Classification)| existing == &probe)
            {
                probes.push((probe, side));
            }
        }
    }

    Ok(probes)
}

fn probe_axes(support: &Plane) -> HypermeshResult<Vec<usize>> {
    let dominant = dominant_normal_axis(support)?;
    let mut axes = vec![dominant];
    for axis in 0..3 {
        if axis != dominant && !axis_ref(&support.normal, axis).definitely_zero() {
            axes.push(axis);
        }
    }
    Ok(axes)
}

fn probe_offset(points: &[Point3], axis: usize) -> HypermeshResult<Real> {
    let mut min = axis_ref(&points[0], axis).clone();
    let mut max = min.clone();
    for point in &points[1..] {
        if compare_real(axis_ref(point, axis), &min)?.is_lt() {
            min = axis_ref(point, axis).clone();
        }
        if compare_real(axis_ref(point, axis), &max)?.is_gt() {
            max = axis_ref(point, axis).clone();
        }
    }
    let extent = max - min;
    if extent.definitely_zero() {
        Ok(Real::one())
    } else {
        Ok(
            (extent.abs() / Real::from(10)).expect("division by literal ten is valid")
                + Real::one(),
        )
    }
}
