//! Geometry distance helpers shared by board-level checks.
//!
//! `csgrs` handles boolean geometry. These helpers fill the gap for clearance
//! fallbacks where two shapes are close but do not intersect.

#[cfg(test)]
use geo::Polygon;
use geo::{Coord, LineString, MultiPolygon};
use hyperlimit::{CircleSegmentRelation, Point2, SegmentIntersection, classify_circle_segment2};

use crate::Scalar;
use crate::geometry::{RuleGeometryProvenance, SourceGridFacts};

/// Exact boundary distance over a finite polygon projection.
///
/// The `geo` polygons are a named compatibility input from current rendering
/// adapters. Every finite coordinate is lifted exactly as its IEEE-754 dyadic,
/// and all metric arithmetic after that boundary remains in [`Scalar`]. Empty
/// or non-finite input has no distance and returns `None` instead of smuggling
/// an infinity sentinel into the internal scalar domain.
pub(super) fn polygon_boundary_distance_scalar(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
) -> Option<Scalar> {
    polygon_boundary_distance_scalar_with_grid(left, right, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

/// Decide whether any polygon boundary pair is within an exact threshold.
///
/// Finite edge AABBs are used only to reject segment pairs whose axis gap is
/// conservatively greater than the outward-rounded threshold. Every surviving
/// pair is lifted and measured in [`Scalar`], and the walk exits on the first
/// exact threshold hit.
pub(super) fn polygon_boundaries_within_scalar(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
    threshold: &Scalar,
) -> bool {
    let Some(projected) = threshold.to_f64_lossy().filter(|value| value.is_finite()) else {
        return true;
    };
    let broad_threshold = if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    };
    left.0.iter().any(|left_polygon| {
        right.0.iter().any(|right_polygon| {
            let left_rings =
                std::iter::once(left_polygon.exterior()).chain(left_polygon.interiors().iter());
            let right_rings = std::iter::once(right_polygon.exterior())
                .chain(right_polygon.interiors().iter())
                .collect::<Vec<_>>();
            left_rings.into_iter().any(|left_ring| {
                right_rings.iter().any(|right_ring| {
                    ring_boundaries_within_scalar(left_ring, right_ring, threshold, broad_threshold)
                })
            })
        })
    })
}

pub(super) fn exact_point_polygon_boundary_within_scalar(
    point: &[Scalar; 2],
    projected_point: [f64; 2],
    polygons: &MultiPolygon<f64>,
    threshold: &Scalar,
) -> bool {
    let Some(projected) = threshold.to_f64_lossy().filter(|value| value.is_finite()) else {
        return true;
    };
    let broad_threshold = if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    };
    let provenance = RuleGeometryProvenance::new(
        "exact-point-polygon-clearance",
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    );
    polygons.0.iter().any(|polygon| {
        std::iter::once(polygon.exterior())
            .chain(polygon.interiors().iter())
            .any(|ring| {
                ring.0.windows(2).any(|segment| {
                    if segment_axis_gap_exceeds(
                        projected_point[0],
                        projected_point[0],
                        segment[0].x,
                        segment[1].x,
                        broad_threshold,
                    ) || segment_axis_gap_exceeds(
                        projected_point[1],
                        projected_point[1],
                        segment[0].y,
                        segment[1].y,
                        broad_threshold,
                    ) {
                        return false;
                    }
                    let Some(start) = lift_scalar_coord(segment[0], provenance) else {
                        return true;
                    };
                    let Some(end) = lift_scalar_coord(segment[1], provenance) else {
                        return true;
                    };
                    point_segment_within_threshold_from_scalars(point, &start, &end, threshold)
                        .unwrap_or(true)
                })
            })
    })
}

fn ring_boundaries_within_scalar(
    left: &LineString<f64>,
    right: &LineString<f64>,
    threshold: &Scalar,
    broad_threshold: f64,
) -> bool {
    left.0.windows(2).any(|left_segment| {
        right.0.windows(2).any(|right_segment| {
            if segment_axis_gap_exceeds(
                left_segment[0].x,
                left_segment[1].x,
                right_segment[0].x,
                right_segment[1].x,
                broad_threshold,
            ) || segment_axis_gap_exceeds(
                left_segment[0].y,
                left_segment[1].y,
                right_segment[0].y,
                right_segment[1].y,
                broad_threshold,
            ) {
                return false;
            }
            segment_segments_within_threshold_with_grid(
                left_segment[0],
                left_segment[1],
                right_segment[0],
                right_segment[1],
                threshold,
                SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
            )
            .unwrap_or(true)
        })
    })
}

fn segment_segments_within_threshold_with_grid(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    threshold: &Scalar,
    grid: SourceGridFacts,
) -> Option<bool> {
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return None;
    }
    if segments_intersect_with_grid(a_start, a_end, b_start, b_end, grid) {
        return Some(true);
    }

    let provenance = RuleGeometryProvenance::new("exact-clearance-threshold", grid);
    let a_start = lift_scalar_coord(a_start, provenance)?;
    let a_end = lift_scalar_coord(a_end, provenance)?;
    let b_start = lift_scalar_coord(b_start, provenance)?;
    let b_end = lift_scalar_coord(b_end, provenance)?;
    for (point, start, end) in [
        (&a_start, &b_start, &b_end),
        (&a_end, &b_start, &b_end),
        (&b_start, &a_start, &a_end),
        (&b_end, &a_start, &a_end),
    ] {
        if point_segment_within_threshold_from_scalars(point, start, end, threshold)? {
            return Some(true);
        }
    }
    Some(false)
}

fn point_segment_within_threshold_from_scalars(
    point: &[Scalar; 2],
    start: &[Scalar; 2],
    end: &[Scalar; 2],
    threshold: &Scalar,
) -> Option<bool> {
    if threshold < &Scalar::zero() {
        return Some(false);
    }
    let center = Point2::new(point[0].clone(), point[1].clone());
    let start = Point2::new(start[0].clone(), start[1].clone());
    let end = Point2::new(end[0].clone(), end[1].clone());
    let threshold_squared = threshold * threshold;
    classify_circle_segment2(&center, &threshold_squared, &start, &end)
        .value()
        .map(|relation| relation != CircleSegmentRelation::Disjoint)
}

fn segment_axis_gap_exceeds(
    left_start: f64,
    left_end: f64,
    right_start: f64,
    right_end: f64,
    broad_threshold: f64,
) -> bool {
    if !left_start.is_finite()
        || !left_end.is_finite()
        || !right_start.is_finite()
        || !right_end.is_finite()
    {
        return false;
    }
    let left_min = left_start.min(left_end);
    let left_max = left_start.max(left_end);
    let right_min = right_start.min(right_end);
    let right_max = right_start.max(right_end);
    let gap = if left_max < right_min {
        right_min - left_max
    } else if right_max < left_min {
        left_min - right_max
    } else {
        0.0
    };
    gap.next_down() > broad_threshold
}

pub(super) fn polygon_boundary_distance_scalar_with_grid(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
    grid: SourceGridFacts,
) -> Option<Scalar> {
    PreparedBoundaryDistance::new_with_grid(left, grid)?
        .distance_to(&PreparedBoundaryDistance::new_with_grid(right, grid)?)
}

/// Reusable exact boundary metric over one finite compatibility projection.
///
/// Coordinates are lifted into [`Scalar`] exactly once. Finite segment bounds
/// only schedule candidates; exact AABB lower bounds and HyperLimit segment
/// predicates decide every retained pair.
pub(super) struct PreparedBoundaryDistance {
    segments_by_min_x: Vec<LiftedSegment>,
}

impl PreparedBoundaryDistance {
    pub(super) fn new(polygons: &MultiPolygon<f64>) -> Option<Self> {
        Self::new_with_grid(polygons, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
    }

    fn new_with_grid(polygons: &MultiPolygon<f64>, grid: SourceGridFacts) -> Option<Self> {
        let provenance = RuleGeometryProvenance::new("exact-clearance-metric", grid);
        let mut segments_by_min_x = Vec::new();
        for polygon in &polygons.0 {
            for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors().iter()) {
                for segment in ring.0.windows(2) {
                    segments_by_min_x.push(LiftedSegment::new(segment[0], segment[1], provenance)?);
                }
            }
        }
        if segments_by_min_x.is_empty() {
            return None;
        }
        segments_by_min_x.sort_by(|left, right| left.finite_min_x.total_cmp(&right.finite_min_x));
        Some(Self { segments_by_min_x })
    }

    pub(super) fn distance_to(&self, other: &Self) -> Option<Scalar> {
        let (outer, indexed) = if self.segments_by_min_x.len() <= other.segments_by_min_x.len() {
            (&self.segments_by_min_x, &other.segments_by_min_x)
        } else {
            (&other.segments_by_min_x, &self.segments_by_min_x)
        };
        let mut minimum_squared =
            lifted_segment_distance_squared(outer.first()?, indexed.first()?)?;
        if minimum_squared == Scalar::zero() {
            return Some(Scalar::zero());
        }

        for left_segment in outer {
            let radius = conservative_sqrt_projection(&minimum_squared)?;
            let query_min_x = (left_segment.finite_min_x - radius).next_down();
            let query_max_x = (left_segment.finite_max_x + radius).next_up();
            let upper = indexed.partition_point(|segment| segment.finite_min_x <= query_max_x);
            for right_segment in &indexed[..upper] {
                if right_segment.finite_max_x < query_min_x
                    || segment_axis_gap_exceeds(
                        left_segment.finite_min_y,
                        left_segment.finite_max_y,
                        right_segment.finite_min_y,
                        right_segment.finite_max_y,
                        radius,
                    )
                    || lifted_segment_aabb_distance_squared(left_segment, right_segment)
                        .is_some_and(|lower_bound| lower_bound >= minimum_squared)
                {
                    continue;
                }
                if let Some(distance) = lifted_segment_distance_squared(left_segment, right_segment)
                    && distance < minimum_squared
                {
                    minimum_squared = distance;
                    if minimum_squared == Scalar::zero() {
                        return Some(Scalar::zero());
                    }
                }
            }
        }
        minimum_squared.sqrt().ok()
    }
}

fn conservative_sqrt_projection(squared: &Scalar) -> Option<f64> {
    let mut projected = squared.to_f64_lossy().filter(|value| value.is_finite())?;
    let provenance = RuleGeometryProvenance::new(
        "exact-clearance-index-radius",
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    );
    while provenance
        .lift_f64(projected)
        .is_some_and(|lifted| lifted < *squared)
    {
        projected = projected.next_up();
    }
    Some(projected.sqrt().next_up())
}

struct LiftedSegment {
    start: [Scalar; 2],
    end: [Scalar; 2],
    min_x: Scalar,
    min_y: Scalar,
    max_x: Scalar,
    max_y: Scalar,
    finite_min_x: f64,
    finite_min_y: f64,
    finite_max_x: f64,
    finite_max_y: f64,
}

impl LiftedSegment {
    fn new(start: Coord<f64>, end: Coord<f64>, provenance: RuleGeometryProvenance) -> Option<Self> {
        let finite_min_x = start.x.min(end.x);
        let finite_min_y = start.y.min(end.y);
        let finite_max_x = start.x.max(end.x);
        let finite_max_y = start.y.max(end.y);
        let start = lift_scalar_coord(start, provenance)?;
        let end = lift_scalar_coord(end, provenance)?;
        let (min_x, max_x) = ordered_pair(&start[0], &end[0])?;
        let (min_y, max_y) = ordered_pair(&start[1], &end[1])?;
        Some(Self {
            start,
            end,
            min_x,
            min_y,
            max_x,
            max_y,
            finite_min_x,
            finite_min_y,
            finite_max_x,
            finite_max_y,
        })
    }
}

fn ordered_pair(left: &Scalar, right: &Scalar) -> Option<(Scalar, Scalar)> {
    match left.partial_cmp(right)? {
        std::cmp::Ordering::Less | std::cmp::Ordering::Equal => Some((left.clone(), right.clone())),
        std::cmp::Ordering::Greater => Some((right.clone(), left.clone())),
    }
}

fn lifted_segment_aabb_distance_squared(
    left: &LiftedSegment,
    right: &LiftedSegment,
) -> Option<Scalar> {
    let dx = exact_interval_gap(&left.min_x, &left.max_x, &right.min_x, &right.max_x)?;
    let dy = exact_interval_gap(&left.min_y, &left.max_y, &right.min_y, &right.max_y)?;
    Some(&dx * &dx + &dy * &dy)
}

fn exact_interval_gap(
    left_min: &Scalar,
    left_max: &Scalar,
    right_min: &Scalar,
    right_max: &Scalar,
) -> Option<Scalar> {
    if left_max < right_min {
        Some(right_min - left_max)
    } else if right_max < left_min {
        Some(left_min - right_max)
    } else if left_min.partial_cmp(left_max).is_none() || right_min.partial_cmp(right_max).is_none()
    {
        None
    } else {
        Some(Scalar::zero())
    }
}

fn lifted_segment_distance_squared(left: &LiftedSegment, right: &LiftedSegment) -> Option<Scalar> {
    let a_start = Point2::new(left.start[0].clone(), left.start[1].clone());
    let a_end = Point2::new(left.end[0].clone(), left.end[1].clone());
    let b_start = Point2::new(right.start[0].clone(), right.start[1].clone());
    let b_end = Point2::new(right.end[0].clone(), right.end[1].clone());
    if !matches!(
        hyperlimit::classify_segment_intersection(&a_start, &a_end, &b_start, &b_end).value(),
        Some(SegmentIntersection::Disjoint)
    ) {
        return Some(Scalar::zero());
    }

    [
        point_segment_distance_squared_from_scalars(&left.start, &right.start, &right.end),
        point_segment_distance_squared_from_scalars(&left.end, &right.start, &right.end),
        point_segment_distance_squared_from_scalars(&right.start, &left.start, &left.end),
        point_segment_distance_squared_from_scalars(&right.end, &left.start, &left.end),
    ]
    .into_iter()
    .fold(None, minimum_scalar)
}

fn point_segment_distance_squared_from_scalars(
    point: &[Scalar; 2],
    start: &[Scalar; 2],
    end: &[Scalar; 2],
) -> Option<Scalar> {
    let dx = &end[0] - &start[0];
    let dy = &end[1] - &start[1];
    let length_squared = &dx * &dx + &dy * &dy;
    if length_squared == Scalar::zero() {
        return Some(scalar_point_distance_squared(point, start));
    }

    let point_dx = &point[0] - &start[0];
    let point_dy = &point[1] - &start[1];
    let numerator = point_dx * &dx + point_dy * &dy;
    let t = if numerator <= Scalar::zero() {
        Scalar::zero()
    } else if numerator >= length_squared {
        Scalar::one()
    } else {
        (numerator / &length_squared).ok()?
    };
    let projection = [&start[0] + &t * &dx, &start[1] + &t * &dy];
    Some(scalar_point_distance_squared(point, &projection))
}

fn lift_scalar_coord(coord: Coord<f64>, provenance: RuleGeometryProvenance) -> Option<[Scalar; 2]> {
    Some([provenance.lift_f64(coord.x)?, provenance.lift_f64(coord.y)?])
}

fn scalar_point_distance_squared(left: &[Scalar; 2], right: &[Scalar; 2]) -> Scalar {
    let dx = &left[0] - &right[0];
    let dy = &left[1] - &right[1];
    &dx * &dx + &dy * &dy
}

fn minimum_scalar(left: Option<Scalar>, right: Option<Scalar>) -> Option<Scalar> {
    match (left, right) {
        (Some(left), Some(right)) => Some(if left <= right { left } else { right }),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

#[cfg(test)]
pub(super) fn polygon_boundary_distance(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
) -> f64 {
    polygon_boundary_distance_with_grid(left, right, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

#[cfg(test)]
pub(super) fn polygon_boundary_distance_with_grid(
    left: &MultiPolygon<f64>,
    right: &MultiPolygon<f64>,
    grid: SourceGridFacts,
) -> f64 {
    // Boundary-distance fallbacks still return an approximate metric, but
    // topology gates inside the segment walk should see the parser's retained
    // source grid whenever the caller has one, preserving source structure at
    // the geometric boundary before expanding to scalar arithmetic.
    let mut minimum = f64::INFINITY;
    for left_polygon in &left.0 {
        for right_polygon in &right.0 {
            minimum = minimum.min(single_polygon_boundary_distance(
                left_polygon,
                right_polygon,
                grid,
            ));
        }
    }
    minimum
}

#[cfg(test)]
fn single_polygon_boundary_distance(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    grid: SourceGridFacts,
) -> f64 {
    let mut minimum = ring_boundary_distance(left.exterior(), right.exterior(), grid);

    for left_hole in left.interiors() {
        minimum = minimum.min(ring_boundary_distance(left_hole, right.exterior(), grid));
        for right_hole in right.interiors() {
            minimum = minimum.min(ring_boundary_distance(left_hole, right_hole, grid));
        }
    }

    for right_hole in right.interiors() {
        minimum = minimum.min(ring_boundary_distance(left.exterior(), right_hole, grid));
    }

    minimum
}

#[cfg(test)]
fn ring_boundary_distance(
    left: &LineString<f64>,
    right: &LineString<f64>,
    grid: SourceGridFacts,
) -> f64 {
    let mut minimum = f64::INFINITY;
    for left_segment in left.0.windows(2) {
        for right_segment in right.0.windows(2) {
            minimum = minimum.min(segment_distance_with_grid(
                left_segment[0],
                left_segment[1],
                right_segment[0],
                right_segment[1],
                grid,
            ));
        }
    }
    minimum
}

#[cfg(test)]
fn segment_distance(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> f64 {
    segment_distance_with_grid(
        a_start,
        a_end,
        b_start,
        b_end,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

#[cfg(test)]
fn segment_distance_with_grid(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> f64 {
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return f64::INFINITY;
    }

    if segments_intersect_with_grid(a_start, a_end, b_start, b_end, grid) {
        return 0.0;
    }

    point_segment_distance_with_grid(a_start, b_start, b_end, grid)
        .min(point_segment_distance_with_grid(
            a_end, b_start, b_end, grid,
        ))
        .min(point_segment_distance_with_grid(
            b_start, a_start, a_end, grid,
        ))
        .min(point_segment_distance_with_grid(
            b_end, a_start, a_end, grid,
        ))
}

#[cfg(test)]
fn point_segment_distance(point: Coord<f64>, start: Coord<f64>, end: Coord<f64>) -> f64 {
    point_segment_distance_with_grid(point, start, end, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

#[cfg(test)]
fn point_segment_distance_with_grid(
    point: Coord<f64>,
    start: Coord<f64>,
    end: Coord<f64>,
    grid: SourceGridFacts,
) -> f64 {
    if !coords_are_finite_3(point, start, end) {
        return f64::INFINITY;
    }

    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    if exact_coords_equal_with_grid(start, end, grid) {
        return distance([point.x, point.y], [start.x, start.y]);
    }
    if length_squared == 0.0 {
        // Metric-edge underflow: exact equality above proved this is not a
        // point segment, but f64 projection cannot represent the squared
        // length. Fall back to endpoint distance for a finite conservative
        // report magnitude; topology has already been handled by exact
        // predicates before this projection path.
        return distance([point.x, point.y], [start.x, start.y])
            .min(distance([point.x, point.y], [end.x, end.y]));
    }

    let t =
        (((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared).clamp(0.0, 1.0);
    distance([point.x, point.y], [start.x + t * dx, start.y + t * dy])
}

#[cfg(test)]
fn segments_intersect(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
) -> bool {
    segments_intersect_with_grid(
        a_start,
        a_end,
        b_start,
        b_end,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

fn segments_intersect_with_grid(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> bool {
    if !coords_are_finite_4(a_start, a_end, b_start, b_end) {
        return false;
    }

    let Some((a, b, c, d)) = lift_segment_points(a_start, a_end, b_start, b_end, grid) else {
        return false;
    };

    // Clearance geometry still arrives from `geo`/`csgrs` as finite f64 edge
    // coordinates, but f64 must remain an I/O compatibility boundary rather
    // than a topology kernel. IEEE-754 coordinates are lifted to exact dyadic
    // `Real`s, then the closed-segment classifier routes orientation and
    // interval tests through `hyperlimit`. Combinatorial decisions use exact
    // predicates; approximate coordinates only describe inputs or report
    // metric magnitudes.
    match hyperlimit::classify_segment_intersection(&a, &b, &c, &d).value() {
        Some(SegmentIntersection::Disjoint) => false,
        Some(_) => true,
        // A strict predicate over lifted finite dyadics should decide. If a
        // future symbolic source reaches this path undecided, report contact
        // conservatively so a clearance check does not silently miss a
        // violation.
        None => true,
    }
}

fn lift_segment_points(
    a_start: Coord<f64>,
    a_end: Coord<f64>,
    b_start: Coord<f64>,
    b_end: Coord<f64>,
    grid: SourceGridFacts,
) -> Option<(Point2, Point2, Point2, Point2)> {
    let provenance = RuleGeometryProvenance::new("clearance-segment-topology", grid);
    Some((
        lift_coord(a_start, provenance)?,
        lift_coord(a_end, provenance)?,
        lift_coord(b_start, provenance)?,
        lift_coord(b_end, provenance)?,
    ))
}

fn lift_coord(coord: Coord<f64>, provenance: RuleGeometryProvenance) -> Option<Point2> {
    Some(Point2::new(
        provenance.lift_f64(coord.x)?,
        provenance.lift_f64(coord.y)?,
    ))
}

#[cfg(test)]
fn exact_coords_equal_with_grid(
    left: Coord<f64>,
    right: Coord<f64>,
    grid: SourceGridFacts,
) -> bool {
    // Degenerate segment classification is a topology decision even when the
    // resulting distance magnitude is reported as f64. Lift finite coordinates
    // and ask `hyperlimit` for exact point equality instead of using
    // `length_squared == 0.0`, which can conflate a very small nonzero segment
    // with a point after primitive-float underflow. This keeps the clearance
    // fallback aligned with the exact-predicate boundary.
    //
    let provenance = RuleGeometryProvenance::new("clearance-degenerate-segment", grid);
    let Some(left) = lift_coord(left, provenance) else {
        return false;
    };
    let Some(right) = lift_coord(right, provenance) else {
        return false;
    };
    hyperlimit::point2_equal(&left, &right)
        .value()
        .unwrap_or(false)
}

#[cfg(test)]
fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    if !left[0].is_finite()
        || !left[1].is_finite()
        || !right[0].is_finite()
        || !right[1].is_finite()
    {
        return f64::INFINITY;
    }

    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn coords_are_finite_4(
    first: Coord<f64>,
    second: Coord<f64>,
    third: Coord<f64>,
    fourth: Coord<f64>,
) -> bool {
    first.x.is_finite()
        && first.y.is_finite()
        && second.x.is_finite()
        && second.y.is_finite()
        && third.x.is_finite()
        && third.y.is_finite()
        && fourth.x.is_finite()
        && fourth.y.is_finite()
}

#[cfg(test)]
fn coords_are_finite_3(first: Coord<f64>, second: Coord<f64>, third: Coord<f64>) -> bool {
    first.x.is_finite()
        && first.y.is_finite()
        && second.x.is_finite()
        && second.y.is_finite()
        && third.x.is_finite()
        && third.y.is_finite()
}

#[cfg(test)]
mod tests {
    use geo::{Coord, LineString, MultiPolygon, Polygon};

    use crate::geometry::{SourceGridFacts, SourceUnit};

    use super::{
        PreparedBoundaryDistance, point_segment_distance, polygon_boundary_distance,
        polygon_boundary_distance_scalar, polygon_boundary_distance_with_grid, segment_distance,
        segment_segments_within_threshold_with_grid, segments_intersect,
    };

    fn square(x: f64, y: f64, size: f64) -> Polygon<f64> {
        Polygon::new(
            LineString(vec![
                Coord { x, y },
                Coord { x: x + size, y },
                Coord {
                    x: x + size,
                    y: y + size,
                },
                Coord { x, y: y + size },
                Coord { x, y },
            ]),
            Vec::new(),
        )
    }

    #[test]
    fn segment_distance_reports_zero_for_endpoint_touch() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 0.0 }];
        let right = [Coord { x: 2.0, y: 0.0 }, Coord { x: 2.0, y: 2.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
    }

    #[test]
    fn segment_distance_is_expected_for_parallel_lines() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 0.0 }];
        let right = [Coord { x: 0.0, y: 1.0 }, Coord { x: 2.0, y: 1.0 }];

        assert!((segment_distance(left[0], left[1], right[0], right[1]) - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn point_segment_distance_uses_projection_for_internal_foot() {
        let point = Coord { x: 1.0, y: 1.0 };
        let start = Coord { x: 0.0, y: 0.0 };
        let end = Coord { x: 2.0, y: 0.0 };

        assert!((point_segment_distance(point, start, end) - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn point_segment_distance_falls_back_to_endpoint_for_degenerate_segment() {
        let point = Coord { x: 1.0, y: 2.0 };
        let endpoint = Coord { x: 3.0, y: 4.0 };

        assert_eq!(
            point_segment_distance(point, endpoint, endpoint),
            ((-2.0f64).powi(2) + (-2.0f64).powi(2)).sqrt()
        );
    }

    #[test]
    fn point_segment_distance_keeps_tiny_nonzero_segment_distinct_from_point() {
        let point = Coord {
            x: 1.0e-200,
            y: 0.0,
        };
        let start = Coord { x: 0.0, y: 0.0 };
        let end = Coord {
            x: 1.0e-200,
            y: 0.0,
        };

        assert_eq!(point_segment_distance(point, start, end), 0.0);
    }

    #[test]
    fn segments_intersect_uses_exact_closed_segment_topology() {
        assert!(segments_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 3.0, y: 0.0 }
        ));
        assert!(!segments_intersect(
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 3.0, y: 0.0 },
            Coord { x: 4.0, y: 0.0 }
        ));
    }

    #[test]
    fn polygon_boundary_distance_reports_touching_polygons_as_zero() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(1.0, 0.0, 1.0)]);

        assert_eq!(polygon_boundary_distance(&left, &right), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_accepts_retained_source_grid_provenance() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(1.0, 0.0, 1.0)]);
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        assert_eq!(
            polygon_boundary_distance_with_grid(&left, &right, grid),
            0.0
        );
    }

    #[test]
    fn polygon_boundary_distance_reports_separated_polygons_by_expected_gap() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(0.0, 3.0, 1.0)]);

        assert_eq!(polygon_boundary_distance(&left, &right), 2.0);
    }

    #[test]
    fn scalar_boundary_distance_retains_exact_projected_coordinates() {
        let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
        let right = MultiPolygon(vec![square(3.0, 0.0, 1.0)]);

        assert_eq!(
            polygon_boundary_distance_scalar(&left, &right),
            Some(crate::scalar::scalar("2"))
        );
    }

    #[test]
    fn prepared_boundary_reuses_large_exact_segment_field() {
        let field = MultiPolygon(
            (0..2_000)
                .map(|index| square(f64::from(index) * 4.0, 0.0, 1.0))
                .collect(),
        );
        let prepared_field =
            PreparedBoundaryDistance::new(&field).expect("finite field has exact boundaries");
        let started = std::time::Instant::now();

        for index in 0..64 {
            let probe = MultiPolygon(vec![square(f64::from(index) * 4.0, 2.0, 1.0)]);
            let prepared_probe =
                PreparedBoundaryDistance::new(&probe).expect("finite probe has exact boundaries");
            assert_eq!(
                prepared_field.distance_to(&prepared_probe),
                Some(crate::scalar::scalar("1"))
            );
        }

        assert!(
            started.elapsed().as_secs_f64() < 2.0,
            "prepared exact boundary field should be indexed and reused"
        );
    }

    #[test]
    fn polygon_boundary_distance_considers_hole_boundaries() {
        let outer = LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 4.0, y: 0.0 },
            Coord { x: 4.0, y: 4.0 },
            Coord { x: 0.0, y: 4.0 },
            Coord { x: 0.0, y: 0.0 },
        ]);
        let hole = LineString(vec![
            Coord { x: 1.5, y: 1.5 },
            Coord { x: 2.5, y: 1.5 },
            Coord { x: 2.5, y: 2.5 },
            Coord { x: 1.5, y: 2.5 },
            Coord { x: 1.5, y: 1.5 },
        ]);
        let with_hole = Polygon::new(outer, vec![hole]);
        let point_polygon = MultiPolygon(vec![with_hole]);

        let touch_hole = MultiPolygon(vec![Polygon::new(
            LineString(vec![
                Coord { x: 1.5, y: 1.5 },
                Coord { x: 2.5, y: 1.5 },
                Coord { x: 2.5, y: 2.5 },
                Coord { x: 1.5, y: 2.5 },
                Coord { x: 1.5, y: 1.5 },
            ]),
            Vec::new(),
        )]);

        assert_eq!(polygon_boundary_distance(&point_polygon, &touch_hole), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_of_empty_geometry_is_infinite() {
        let left = MultiPolygon(vec![]);
        let right = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);

        assert!(polygon_boundary_distance(&left, &right).is_infinite());
    }

    #[test]
    fn segment_distance_reports_zero_for_overlapping_collinear_segments() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 3.0, y: 0.0 }];
        let right = [Coord { x: 1.0, y: 0.0 }, Coord { x: 5.0, y: 0.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
    }

    #[test]
    fn segments_intersect_treats_touching_endpoints_as_intersection() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 2.0, y: 2.0 }];
        let right = [Coord { x: 2.0, y: 2.0 }, Coord { x: 4.0, y: 0.0 }];

        assert!(segments_intersect(left[0], left[1], right[0], right[1]));
    }

    #[test]
    fn polygon_boundary_distance_is_symmetric_for_hole_and_outer_inputs() {
        let outer = square(0.0, 0.0, 2.0);
        let inner = square(0.25, 0.25, 0.5);
        let with_hole = MultiPolygon(vec![Polygon::new(
            outer.exterior().clone(),
            vec![inner.exterior().clone()],
        )]);

        let outer_multi = MultiPolygon(vec![outer.clone()]);
        assert_eq!(
            polygon_boundary_distance(&outer_multi, &with_hole),
            polygon_boundary_distance(&with_hole, &MultiPolygon(vec![outer]))
        );
    }

    #[test]
    fn segment_distance_with_non_finite_endpoints_is_infinite() {
        let left = [
            Coord {
                x: f64::NAN,
                y: 0.0,
            },
            Coord { x: 1.0, y: 0.0 },
        ];
        let right = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 1.0 }];

        assert!(segment_distance(left[0], left[1], right[0], right[1]).is_infinite());
    }

    #[test]
    fn point_segment_distance_with_non_finite_endpoint_is_infinite() {
        assert!(
            point_segment_distance(
                Coord { x: 0.0, y: 0.0 },
                Coord {
                    x: f64::INFINITY,
                    y: 1.0
                },
                Coord { x: 1.0, y: 1.0 }
            )
            .is_infinite()
        );
    }

    #[test]
    fn segments_intersect_with_non_finite_inputs_returns_false() {
        assert!(!segments_intersect(
            Coord {
                x: f64::NAN,
                y: 0.0
            },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 1.0, y: 0.0 }
        ));
    }

    #[test]
    fn segment_distance_does_not_zero_parallel_traces_inside_old_epsilon() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 0.0 }];
        let right = [Coord { x: 0.0, y: 5.0e-13 }, Coord { x: 1.0, y: 5.0e-13 }];

        let measured = segment_distance(left[0], left[1], right[0], right[1]);
        assert!(measured > 0.0);
        assert!((measured - 5.0e-13).abs() <= f64::EPSILON);
    }

    #[test]
    fn segment_threshold_predicate_certifies_exact_dyadic_boundary() {
        let gap = 2.0_f64.powi(-40);
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 0.0 }];
        let right = [Coord { x: 0.0, y: gap }, Coord { x: 1.0, y: gap }];
        let threshold = crate::Scalar::try_from(gap).expect("finite dyadic threshold");
        let below = crate::Scalar::try_from(gap.next_down()).expect("finite dyadic threshold");

        assert_eq!(
            segment_segments_within_threshold_with_grid(
                left[0],
                left[1],
                right[0],
                right[1],
                &threshold,
                SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
            ),
            Some(true)
        );
        assert_eq!(
            segment_segments_within_threshold_with_grid(
                left[0],
                left[1],
                right[0],
                right[1],
                &below,
                SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
            ),
            Some(false)
        );
    }

    #[test]
    fn segment_distance_still_zeroes_tiny_exact_crossing() {
        let left = [Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 1.0e-13 }];
        let right = [Coord { x: 0.0, y: 1.0e-13 }, Coord { x: 1.0, y: 0.0 }];

        assert_eq!(segment_distance(left[0], left[1], right[0], right[1]), 0.0);
    }

    #[test]
    fn polygon_boundary_distance_skips_invalid_coordinates_as_no_geometry_overlap() {
        let invalid = Polygon::new(
            LineString(vec![
                Coord {
                    x: f64::NAN,
                    y: 0.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 1.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 1.0,
                },
                Coord {
                    x: f64::NAN,
                    y: 0.0,
                },
            ]),
            Vec::new(),
        );

        assert!(
            polygon_boundary_distance(
                &MultiPolygon(vec![invalid]),
                &MultiPolygon(vec![square(0.0, 0.0, 1.0)])
            )
            .is_infinite()
        );
    }
}
