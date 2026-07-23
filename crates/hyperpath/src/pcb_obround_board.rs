//! Exact obround PCB board predicates.
//!
//! Obround and capsule-shaped boards occur in sensors, USB dongles, tags, and
//! mechanical keep-in regions. This module retains that curved non-circular
//! board geometry as a spine segment plus diameter. It deliberately does not
//! clip copper, union pads, or construct solids; `hypermesh` owns those later
//! materialization steps. The predicates below only decide whether retained
//! path-domain copper objects fit inside the retained board with exact
//! clearance evidence.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::pcb::{
    ClearanceStatus, PadBoardClearanceReport, PcbCircularPad, PcbTrace, TraceClearanceReport,
    TraceWidthClass,
};
use crate::segment::LinePathSegment;

/// Cached facts for an exact obround PCB board outline.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbObroundBoardOutlineFacts {
    /// Exact-set facts across spine endpoints and board diameter.
    pub exact: RealExactSetFacts,
    /// Diameter sign class.
    pub diameter_class: TraceWidthClass,
    /// Whether the retained spine endpoints are structurally identical.
    pub degenerate_spine: Option<bool>,
}

/// Exact obround/capsule PCB board outline.
///
/// The board is the Minkowski sum of a retained spine segment and a disk of
/// `diameter / 2`. A trace or circular pad is certified inside the board by
/// eroding the board diameter by the copper diameter plus required clearance,
/// then checking exact point-to-spine squared distances for the retained
/// centerline endpoints or pad center.
///
/// Lossy polygonal approximations and primitive-float tolerances are not
/// topology evidence. The endpoint-only trace containment
/// check is valid because capsules are convex; see the standard convex-set
/// containment property used in Preparata and Shamos, *Computational Geometry:
/// An Introduction* (1985). Thus a straight centerline segment whose endpoints
/// lie in the eroded capsule lies wholly in that eroded capsule.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbObroundBoardOutline {
    spine: LinePathSegment,
    diameter: Real,
    facts: PcbObroundBoardOutlineFacts,
}

impl PcbObroundBoardOutline {
    /// Construct an obround board outline.
    pub fn new(spine: LinePathSegment, diameter: Real) -> Result<Self, &'static str> {
        let diameter_class = match diameter.structural_facts().sign {
            Some(RealSign::Negative) => return Err("obround board diameter must be nonnegative"),
            Some(RealSign::Zero) => TraceWidthClass::Zero,
            Some(RealSign::Positive) => TraceWidthClass::Positive,
            None => TraceWidthClass::Unknown,
        };
        let facts = PcbObroundBoardOutlineFacts {
            exact: Real::exact_set_facts([
                &spine.start().x,
                &spine.start().y,
                &spine.end().x,
                &spine.end().y,
                &diameter,
            ]),
            diameter_class,
            degenerate_spine: spine.facts().known_degenerate,
        };
        Ok(Self {
            spine,
            diameter,
            facts,
        })
    }

    /// Return exact retained board spine.
    pub const fn spine(&self) -> &LinePathSegment {
        &self.spine
    }

    /// Return exact board diameter.
    pub const fn diameter(&self) -> &Real {
        &self.diameter
    }

    /// Return cached exact facts.
    pub const fn facts(&self) -> &PcbObroundBoardOutlineFacts {
        &self.facts
    }
}

/// Check exact clearance from a trace to an obround board outline.
///
/// The board is first eroded by `trace_width / 2 + required_clearance`. The
/// trace centerline is certified inside that eroded board by checking both
/// exact endpoints against the retained spine. The squared-distance replay is
/// `4*d^2 <= (board_diameter - trace_width - 2*clearance)^2`, with the
/// allowable doubled radius sign checked before squaring so impossible
/// clearances cannot pass through a positive square.
pub fn check_trace_obround_board_clearance(
    trace: &PcbTrace,
    board: &PcbObroundBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let required_doubled =
        trace.swept().width().clone() + required_clearance.clone() * Real::from(2);
    let status = classify_points_inside_eroded_obround_board(
        [
            trace.swept().centerline().start(),
            trace.swept().centerline().end(),
        ],
        board,
        &required_doubled,
        policy,
    );
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check exact clearance from a circular pad to an obround board outline.
///
/// This is the non-circular curved-board counterpart of circular-board pad
/// clearance. The retained circular pad contributes its exact diameter to the
/// board erosion budget, and the pad center is checked against the eroded
/// capsule without triangulating the board arcs.
pub fn check_circular_pad_obround_board_clearance(
    pad: &PcbCircularPad,
    board: &PcbObroundBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let required_doubled = pad.diameter().clone() + required_clearance.clone() * Real::from(2);
    let status = classify_points_inside_eroded_obround_board(
        [pad.center(), pad.center()],
        board,
        &required_doubled,
        policy,
    );
    PadBoardClearanceReport {
        status,
        copper_gap: None,
    }
}

fn classify_points_inside_eroded_obround_board<'a>(
    points: impl IntoIterator<Item = &'a Point2>,
    board: &PcbObroundBoardOutline,
    required_doubled: &Real,
    policy: PredicatePolicy,
) -> ClearanceStatus {
    let allowable_doubled = board.diameter().clone() - required_doubled.clone();
    match compare_reals_with_policy(&allowable_doubled, &Real::zero(), policy).value() {
        Some(Ordering::Less) => return ClearanceStatus::ClearanceViolation,
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => return ClearanceStatus::Unknown,
    }
    let allowable_squared = allowable_doubled.clone() * allowable_doubled;
    for point in points {
        let Some(distance_squared) = point_segment_distance_squared(point, board.spine(), policy)
        else {
            return ClearanceStatus::Unknown;
        };
        let lhs = distance_squared * Real::from(4);
        match compare_reals_with_policy(&lhs, &allowable_squared, policy).value() {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return ClearanceStatus::ClearanceViolation,
            None => return ClearanceStatus::Unknown,
        }
    }
    ClearanceStatus::CertifiedClear
}

fn point_segment_distance_squared(
    point: &Point2,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let ab = Point2::new(
        segment.end().x.clone() - segment.start().x.clone(),
        segment.end().y.clone() - segment.start().y.clone(),
    );
    let ap = Point2::new(
        point.x.clone() - segment.start().x.clone(),
        point.y.clone() - segment.start().y.clone(),
    );
    let bp = Point2::new(
        point.x.clone() - segment.end().x.clone(),
        point.y.clone() - segment.end().y.clone(),
    );
    let length_squared = squared_norm(&ab);
    match compare_reals_with_policy(&length_squared, &Real::zero(), policy).value()? {
        Ordering::Equal => return Some(squared_norm(&ap)),
        Ordering::Less => return None,
        Ordering::Greater => {}
    }
    let projection = dot(&ap, &ab);
    if !matches!(
        compare_reals_with_policy(&projection, &Real::zero(), policy).value()?,
        Ordering::Greater
    ) {
        return Some(squared_norm(&ap));
    }
    if !matches!(
        compare_reals_with_policy(&projection, &length_squared, policy).value()?,
        Ordering::Less
    ) {
        return Some(squared_norm(&bp));
    }
    let cross_value = cross(&ap, &ab);
    let cross_squared = cross_value.clone() * cross_value;
    (cross_squared / length_squared).ok()
}

fn squared_norm(vector: &Point2) -> Real {
    Real::signed_product_sum(
        [true, true],
        [[&vector.x, &vector.x], [&vector.y, &vector.y]],
    )
}

fn dot(first: &Point2, second: &Point2) -> Real {
    Real::signed_product_sum([true, true], [[&first.x, &second.x], [&first.y, &second.y]])
}

fn cross(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.y.clone() - first.y.clone() * second.x.clone()
}
