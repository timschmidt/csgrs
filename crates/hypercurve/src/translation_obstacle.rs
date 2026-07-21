//! Exact translation obstacles (no-fit regions) for convex line contours.
//!
//! For fixed shape `A` and untranslated moving shape `B`, the forbidden set of
//! translations is the closed Minkowski sum `A + (-B)`. Supported convex
//! contours are normalized to counter-clockwise vertex order and merged by
//! exact edge angle in `O(n + m)` output work. Concavity remains an explicit
//! decomposition blocker rather than triggering sampling or a quadratic
//! pairwise-point hull.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, real_sign};
use crate::{
    Classification, Contour2, ContourPointLocation, CurvePolicy, CurveResult, LineSeg2, Point2,
    Segment2,
};

/// Operand named by a translation-obstacle blocker.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TranslationObstacleOperand2 {
    /// Stationary contour.
    Fixed,
    /// Contour translated during placement.
    Moving,
}

/// Explicit reason an exact translation obstacle was not materialized.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TranslationObstacleBlocker2 {
    /// The operand contains a non-line segment.
    UnsupportedSegment {
        operand: TranslationObstacleOperand2,
        segment_index: usize,
    },
    /// The operand has a self-contact or self-intersection.
    SelfContact {
        operand: TranslationObstacleOperand2,
    },
    /// Operand self-contact classification was indeterminate.
    UncertainSelfContact {
        operand: TranslationObstacleOperand2,
    },
    /// Exact signed area was unavailable, zero, or indeterminate.
    InvalidOrientation {
        operand: TranslationObstacleOperand2,
    },
    /// Convex edge merging needs a decomposition of this concave operand.
    ConvexDecompositionRequired {
        operand: TranslationObstacleOperand2,
        vertex_index: usize,
    },
    /// A local turn or edge-angle relation was indeterminate.
    UncertainOrdering,
    /// Collinear cleanup left fewer than three vertices.
    DegenerateContour {
        operand: TranslationObstacleOperand2,
    },
}

/// Exact closed set of moving-shape translations that touch or overlap a fixed shape.
#[derive(Clone, Debug, PartialEq)]
pub struct TranslationObstacle2 {
    boundary: Contour2,
    fixed_vertex_count: usize,
    moving_vertex_count: usize,
    merged_edge_count: usize,
}

impl TranslationObstacle2 {
    /// Return the exact boundary of the forbidden translation set.
    pub const fn boundary(&self) -> &Contour2 {
        &self.boundary
    }

    /// Return the normalized fixed-shape vertex count.
    pub const fn fixed_vertex_count(&self) -> usize {
        self.fixed_vertex_count
    }

    /// Return the normalized moving-shape vertex count.
    pub const fn moving_vertex_count(&self) -> usize {
        self.moving_vertex_count
    }

    /// Return the number of output edge directions after parallel-edge merging.
    pub const fn merged_edge_count(&self) -> usize {
        self.merged_edge_count
    }

    /// Classify a candidate moving-shape translation against the forbidden set.
    ///
    /// `Inside` means overlap, `Boundary` means contact, and `Outside` means the
    /// two closed contours are separated.
    pub fn classify_translation(
        &self,
        translation: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<ContourPointLocation>> {
        Ok(self.boundary.classify_point(translation, policy))
    }
}

/// Report for exact translation-obstacle construction.
#[derive(Clone, Debug, PartialEq)]
pub struct TranslationObstacleReport2 {
    source_fixed_segment_count: usize,
    source_moving_segment_count: usize,
    normalized_fixed_vertex_count: Option<usize>,
    normalized_moving_vertex_count: Option<usize>,
    obstacle: Option<TranslationObstacle2>,
    blocker: Option<TranslationObstacleBlocker2>,
}

impl TranslationObstacleReport2 {
    /// Return the source fixed-contour segment count.
    pub const fn source_fixed_segment_count(&self) -> usize {
        self.source_fixed_segment_count
    }

    /// Return the source moving-contour segment count.
    pub const fn source_moving_segment_count(&self) -> usize {
        self.source_moving_segment_count
    }

    /// Return the fixed vertex count after collinear normalization.
    pub const fn normalized_fixed_vertex_count(&self) -> Option<usize> {
        self.normalized_fixed_vertex_count
    }

    /// Return the moving vertex count after collinear normalization.
    pub const fn normalized_moving_vertex_count(&self) -> Option<usize> {
        self.normalized_moving_vertex_count
    }

    /// Return the completed forbidden translation region.
    pub const fn obstacle(&self) -> Option<&TranslationObstacle2> {
        self.obstacle.as_ref()
    }

    /// Return the explicit construction blocker.
    pub const fn blocker(&self) -> Option<&TranslationObstacleBlocker2> {
        self.blocker.as_ref()
    }

    /// Consume the report and return the completed obstacle.
    pub fn into_obstacle(self) -> Option<TranslationObstacle2> {
        self.obstacle
    }
}

/// Construct the closed forbidden translation set `fixed + (-moving)`.
pub fn translation_obstacle_convex(
    fixed: &Contour2,
    moving: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<TranslationObstacleReport2> {
    let source_fixed_segment_count = fixed.segments().len();
    let source_moving_segment_count = moving.segments().len();
    let blocked = |fixed_count, moving_count, blocker| TranslationObstacleReport2 {
        source_fixed_segment_count,
        source_moving_segment_count,
        normalized_fixed_vertex_count: fixed_count,
        normalized_moving_vertex_count: moving_count,
        obstacle: None,
        blocker: Some(blocker),
    };

    let fixed_vertices =
        match normalized_convex_vertices(fixed, TranslationObstacleOperand2::Fixed, policy)? {
            Ok(vertices) => vertices,
            Err(blocker) => return Ok(blocked(None, None, blocker)),
        };
    let moving_vertices =
        match normalized_convex_vertices(moving, TranslationObstacleOperand2::Moving, policy)? {
            Ok(vertices) => vertices,
            Err(blocker) => {
                return Ok(blocked(Some(fixed_vertices.len()), None, blocker));
            }
        };
    let fixed_count = fixed_vertices.len();
    let moving_count = moving_vertices.len();
    let reflected = moving_vertices
        .into_iter()
        .map(|point| Point2::new(-point.x().clone(), -point.y().clone()))
        .collect::<Vec<_>>();
    let obstacle_points = match merge_convex_minkowski(fixed_vertices, reflected, policy)? {
        Ok(points) => points,
        Err(blocker) => {
            return Ok(blocked(Some(fixed_count), Some(moving_count), blocker));
        }
    };
    let boundary = contour_from_points(&obstacle_points)?;
    Ok(TranslationObstacleReport2 {
        source_fixed_segment_count,
        source_moving_segment_count,
        normalized_fixed_vertex_count: Some(fixed_count),
        normalized_moving_vertex_count: Some(moving_count),
        obstacle: Some(TranslationObstacle2 {
            merged_edge_count: obstacle_points.len(),
            boundary,
            fixed_vertex_count: fixed_count,
            moving_vertex_count: moving_count,
        }),
        blocker: None,
    })
}

fn normalized_convex_vertices(
    contour: &Contour2,
    operand: TranslationObstacleOperand2,
    policy: &CurvePolicy,
) -> CurveResult<Result<Vec<Point2>, TranslationObstacleBlocker2>> {
    let mut vertices = Vec::with_capacity(contour.segments().len());
    for (segment_index, segment) in contour.segments().iter().enumerate() {
        match segment {
            Segment2::Line(line) => vertices.push(line.start().clone()),
            _ => {
                return Ok(Err(TranslationObstacleBlocker2::UnsupportedSegment {
                    operand,
                    segment_index,
                }));
            }
        }
    }
    match contour.has_self_contacts(policy)? {
        Classification::Decided(false) => {}
        Classification::Decided(true) => {
            return Ok(Err(TranslationObstacleBlocker2::SelfContact { operand }));
        }
        Classification::Uncertain(_) => {
            return Ok(Err(TranslationObstacleBlocker2::UncertainSelfContact {
                operand,
            }));
        }
    }
    let Some(area) = contour.signed_area()? else {
        return Ok(Err(TranslationObstacleBlocker2::InvalidOrientation {
            operand,
        }));
    };
    match real_sign(&area, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Negative) => vertices.reverse(),
        Some(RealSign::Zero) | None => {
            return Ok(Err(TranslationObstacleBlocker2::InvalidOrientation {
                operand,
            }));
        }
    }

    vertices = match remove_collinear_vertices(vertices, policy) {
        Ok(vertices) => vertices,
        Err(()) => return Ok(Err(TranslationObstacleBlocker2::UncertainOrdering)),
    };
    if vertices.len() < 3 {
        return Ok(Err(TranslationObstacleBlocker2::DegenerateContour {
            operand,
        }));
    }
    for vertex_index in 0..vertices.len() {
        let previous = &vertices[(vertex_index + vertices.len() - 1) % vertices.len()];
        let current = &vertices[vertex_index];
        let next = &vertices[(vertex_index + 1) % vertices.len()];
        let (incoming_x, incoming_y) = current.delta_from(previous);
        let (outgoing_x, outgoing_y) = next.delta_from(current);
        let turn = &incoming_x * &outgoing_y - &incoming_y * &outgoing_x;
        match real_sign(&turn, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Negative | RealSign::Zero) => {
                return Ok(Err(
                    TranslationObstacleBlocker2::ConvexDecompositionRequired {
                        operand,
                        vertex_index,
                    },
                ));
            }
            None => return Ok(Err(TranslationObstacleBlocker2::UncertainOrdering)),
        }
    }
    if rotate_to_lowest(&mut vertices, policy).is_err() {
        return Ok(Err(TranslationObstacleBlocker2::UncertainOrdering));
    }
    Ok(Ok(vertices))
}

fn remove_collinear_vertices(
    mut vertices: Vec<Point2>,
    policy: &CurvePolicy,
) -> Result<Vec<Point2>, ()> {
    loop {
        if vertices.len() <= 3 {
            return Ok(vertices);
        }
        let mut retained = Vec::with_capacity(vertices.len());
        for index in 0..vertices.len() {
            let previous = &vertices[(index + vertices.len() - 1) % vertices.len()];
            let current = &vertices[index];
            let next = &vertices[(index + 1) % vertices.len()];
            let (incoming_x, incoming_y) = current.delta_from(previous);
            let (outgoing_x, outgoing_y) = next.delta_from(current);
            let turn = &incoming_x * &outgoing_y - &incoming_y * &outgoing_x;
            match real_sign(&turn, policy) {
                Some(RealSign::Zero) => {}
                Some(_) => retained.push(current.clone()),
                None => return Err(()),
            }
        }
        if retained.len() == vertices.len() {
            return Ok(retained);
        }
        vertices = retained;
    }
}

fn rotate_to_lowest(vertices: &mut [Point2], policy: &CurvePolicy) -> Result<(), ()> {
    let mut lowest = 0usize;
    for index in 1..vertices.len() {
        let y_order = compare_reals(vertices[index].y(), vertices[lowest].y(), policy).ok_or(())?;
        if y_order == Ordering::Less
            || (y_order == Ordering::Equal
                && compare_reals(vertices[index].x(), vertices[lowest].x(), policy).ok_or(())?
                    == Ordering::Less)
        {
            lowest = index;
        }
    }
    vertices.rotate_left(lowest);
    Ok(())
}

fn merge_convex_minkowski(
    mut first: Vec<Point2>,
    mut second: Vec<Point2>,
    policy: &CurvePolicy,
) -> CurveResult<Result<Vec<Point2>, TranslationObstacleBlocker2>> {
    if rotate_to_lowest(&mut first, policy).is_err()
        || rotate_to_lowest(&mut second, policy).is_err()
    {
        return Ok(Err(TranslationObstacleBlocker2::UncertainOrdering));
    }
    let first_edges = polygon_edges(&first);
    let second_edges = polygon_edges(&second);
    let mut first_index = 0usize;
    let mut second_index = 0usize;
    let mut current = Point2::new(first[0].x() + second[0].x(), first[0].y() + second[0].y());
    let mut output = vec![current.clone()];

    while first_index < first_edges.len() || second_index < second_edges.len() {
        let (step_x, step_y) = if first_index == first_edges.len() {
            let edge = second_edges[second_index].clone();
            second_index += 1;
            edge
        } else if second_index == second_edges.len() {
            let edge = first_edges[first_index].clone();
            first_index += 1;
            edge
        } else {
            let first_edge = &first_edges[first_index];
            let second_edge = &second_edges[second_index];
            let cross = &first_edge.0 * &second_edge.1 - &first_edge.1 * &second_edge.0;
            match real_sign(&cross, policy) {
                Some(RealSign::Positive) => {
                    first_index += 1;
                    first_edge.clone()
                }
                Some(RealSign::Negative) => {
                    second_index += 1;
                    second_edge.clone()
                }
                Some(RealSign::Zero) => {
                    first_index += 1;
                    second_index += 1;
                    (
                        &first_edge.0 + &second_edge.0,
                        &first_edge.1 + &second_edge.1,
                    )
                }
                None => return Ok(Err(TranslationObstacleBlocker2::UncertainOrdering)),
            }
        };
        current = current.translated(step_x, step_y);
        output.push(current.clone());
    }
    if output.last() == output.first() {
        output.pop();
    }
    Ok(Ok(output))
}

fn polygon_edges(vertices: &[Point2]) -> Vec<(Real, Real)> {
    (0..vertices.len())
        .map(|index| vertices[(index + 1) % vertices.len()].delta_from(&vertices[index]))
        .collect()
}

fn contour_from_points(points: &[Point2]) -> CurveResult<Contour2> {
    let mut segments = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        segments.push(Segment2::Line(LineSeg2::try_new(
            points[index].clone(),
            points[(index + 1) % points.len()].clone(),
        )?));
    }
    Contour2::try_new(segments)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn contour(points: &[(i32, i32)]) -> Contour2 {
        let points = points
            .iter()
            .map(|(x, y)| Point2::new(r(*x), r(*y)))
            .collect::<Vec<_>>();
        contour_from_points(&points).unwrap()
    }

    fn decided<T>(classification: Classification<T>) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
        }
    }

    #[test]
    fn rectangle_translation_obstacle_is_exact_expanded_rectangle() {
        let fixed = contour(&[(0, 0), (2, 0), (2, 2), (0, 2)]);
        let moving = contour(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
        let report =
            translation_obstacle_convex(&fixed, &moving, &CurvePolicy::certified()).unwrap();
        let obstacle = report.obstacle().unwrap();
        assert_eq!(obstacle.merged_edge_count(), 4);
        assert_eq!(
            decided(
                obstacle
                    .classify_translation(&Point2::new(r(0), r(0)), &CurvePolicy::certified())
                    .unwrap()
            ),
            ContourPointLocation::Inside
        );
        assert_eq!(
            decided(
                obstacle
                    .classify_translation(&Point2::new(r(2), r(0)), &CurvePolicy::certified())
                    .unwrap()
            ),
            ContourPointLocation::Boundary
        );
        assert_eq!(
            decided(
                obstacle
                    .classify_translation(&Point2::new(r(3), r(0)), &CurvePolicy::certified())
                    .unwrap()
            ),
            ContourPointLocation::Outside
        );
        assert_eq!(obstacle.boundary().signed_area().unwrap(), Some(r(9)));
    }

    #[test]
    fn clockwise_inputs_normalize_without_changing_forbidden_set() {
        let fixed = contour(&[(0, 0), (0, 2), (2, 2), (2, 0)]);
        let moving = contour(&[(0, 0), (0, 1), (1, 1), (1, 0)]);
        let report =
            translation_obstacle_convex(&fixed, &moving, &CurvePolicy::certified()).unwrap();
        assert_eq!(
            report.obstacle().unwrap().boundary().signed_area().unwrap(),
            Some(r(9))
        );
    }

    #[test]
    fn concave_operand_requires_exact_convex_decomposition() {
        let fixed = contour(&[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]);
        let moving = contour(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
        let report =
            translation_obstacle_convex(&fixed, &moving, &CurvePolicy::certified()).unwrap();
        assert!(report.obstacle().is_none());
        assert!(matches!(
            report.blocker(),
            Some(TranslationObstacleBlocker2::ConvexDecompositionRequired {
                operand: TranslationObstacleOperand2::Fixed,
                ..
            })
        ));
    }
}
