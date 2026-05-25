//! Exact circular PCB board predicates.
//!
//! Circular and disk-like board outlines show up in wearables, sensors, and
//! panelized subassemblies. This module keeps that curved board geometry as a
//! retained hyperpath record with exact clearance predicates. It does not clip
//! copper, union shapes, or build meshes; those acceptance/materialization
//! stages remain owned by `hypermesh`.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::pcb::{
    ClearanceStatus, PadBoardClearanceReport, PcbCircularPad, PcbTrace, TraceClearanceReport,
    TraceWidthClass,
};
use crate::provenance::PathProvenance;

/// Cached facts for an exact circular PCB board outline.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCircularBoardOutlineFacts {
    /// Exact-set facts across center coordinates and radius.
    pub exact: RealExactSetFacts,
    /// Radius sign class.
    pub radius_class: TraceWidthClass,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact circular PCB board outline.
///
/// The board is retained as center plus radius, not as a polygonal
/// approximation. Clearance predicates compare exact squared distances against
/// exact squared allowable radii, avoiding square roots and primitive
/// tolerances. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): path search may propose candidates,
/// but exact object records and exact predicates decide acceptance. For a
/// straight trace segment inside a disk, endpoint checks are sufficient because
/// squared distance to the disk center is convex along the segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCircularBoardOutline {
    center: Point2,
    radius: Real,
    facts: PcbCircularBoardOutlineFacts,
}

impl PcbCircularBoardOutline {
    /// Construct a circular board outline with native provenance.
    pub fn new(center: Point2, radius: Real) -> Result<Self, &'static str> {
        Self::with_provenance(center, radius, PathProvenance::native())
    }

    /// Construct a circular board outline with source provenance.
    ///
    /// Negative radii are rejected instead of coerced. A zero-radius outline is
    /// retained because it is a valid exact degenerate object for antagonistic
    /// import tests and must simply fail positive clearance predicates.
    pub fn with_provenance(
        center: Point2,
        radius: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        let radius_class = match radius.structural_facts().sign {
            Some(RealSign::Negative) => return Err("circular board radius must be nonnegative"),
            Some(RealSign::Zero) => TraceWidthClass::Zero,
            Some(RealSign::Positive) => TraceWidthClass::Positive,
            None => TraceWidthClass::Unknown,
        };
        let facts = PcbCircularBoardOutlineFacts {
            exact: Real::exact_set_facts([&center.x, &center.y, &radius]),
            radius_class,
            provenance,
        };
        Ok(Self {
            center,
            radius,
            facts,
        })
    }

    /// Return exact board center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact board radius.
    pub const fn radius(&self) -> &Real {
        &self.radius
    }

    /// Return cached exact facts.
    pub const fn facts(&self) -> &PcbCircularBoardOutlineFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Check exact clearance from a trace to a circular board outline.
///
/// The predicate certifies that the entire swept straight trace stays inside
/// the disk by at least `required_clearance`. It compares the larger endpoint
/// center distance against `radius - trace_width/2 - clearance`, represented as
/// `4*d^2 <= (2*radius - trace_width - 2*clearance)^2`. The sign of the
/// allowable doubled radius is checked before squaring so an impossible
/// clearance budget cannot be accepted by a positive square.
pub fn check_trace_circular_board_clearance(
    trace: &PcbTrace,
    board: &PcbCircularBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let Some(max_distance_squared) = max_trace_endpoint_distance_squared(trace, board, policy)
    else {
        return unknown_trace_report();
    };
    let required_doubled =
        trace.swept().width().clone() + required_clearance.clone() * Real::from(2);
    let status = classify_inside_circular_board(
        max_distance_squared,
        board.radius(),
        &required_doubled,
        policy,
    );
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check exact clearance from a circular pad to a circular board outline.
///
/// This is the curved-board counterpart of rectangular pad board clearance:
/// the retained pad and retained board are both circles, and the decision is
/// `center_distance + pad_diameter/2 + clearance <= board_radius`. The
/// implementation compares squared exact distances after validating the
/// retained allowable doubled radius sign.
pub fn check_circular_pad_circular_board_clearance(
    pad: &PcbCircularPad,
    board: &PcbCircularBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let center_distance_squared = squared_distance(pad.center(), board.center());
    let required_doubled = pad.diameter().clone() + required_clearance.clone() * Real::from(2);
    let status = classify_inside_circular_board(
        center_distance_squared,
        board.radius(),
        &required_doubled,
        policy,
    );
    PadBoardClearanceReport {
        status,
        copper_gap: None,
    }
}

fn max_trace_endpoint_distance_squared(
    trace: &PcbTrace,
    board: &PcbCircularBoardOutline,
    policy: PredicatePolicy,
) -> Option<Real> {
    let start_distance = squared_distance(trace.swept().centerline().start(), board.center());
    let end_distance = squared_distance(trace.swept().centerline().end(), board.center());
    match compare_reals_with_policy(&start_distance, &end_distance, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(end_distance),
        Ordering::Greater => Some(start_distance),
    }
}

fn classify_inside_circular_board(
    distance_squared: Real,
    board_radius: &Real,
    required_doubled: &Real,
    policy: PredicatePolicy,
) -> ClearanceStatus {
    let allowable_doubled = board_radius.clone() * Real::from(2) - required_doubled.clone();
    match compare_reals_with_policy(&allowable_doubled, &Real::zero(), policy).value() {
        Some(Ordering::Less) => return ClearanceStatus::ClearanceViolation,
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => return ClearanceStatus::Unknown,
    }
    let lhs = distance_squared * Real::from(4);
    let rhs = allowable_doubled.clone() * allowable_doubled;
    match compare_reals_with_policy(&lhs, &rhs, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::CertifiedClear,
        Some(Ordering::Greater) => ClearanceStatus::ClearanceViolation,
        None => ClearanceStatus::Unknown,
    }
}

fn unknown_trace_report() -> TraceClearanceReport {
    TraceClearanceReport {
        status: ClearanceStatus::Unknown,
        centerline_intersection: None,
        axis_gap: None,
    }
}

fn squared_distance(first: &Point2, second: &Point2) -> Real {
    let dx = first.x.clone() - second.x.clone();
    let dy = first.y.clone() - second.y.clone();
    Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]])
}
