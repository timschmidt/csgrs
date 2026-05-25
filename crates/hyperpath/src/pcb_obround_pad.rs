//! Exact obround PCB pad predicates.
//!
//! Obround pads are common for slotted or leaded footprints. In `hyperpath`
//! they are retained as a spine segment plus diameter and certified with exact
//! path predicates. Copper unions, clipping, and solid materialization remain
//! `hypermesh` responsibilities.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals_with_policy,
};
use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::pcb::{
    ClearanceStatus, PadBoardClearanceReport, PcbBoardOutline, PcbTrace, TraceClearanceReport,
    TraceWidthClass,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Cached facts for an exact obround PCB pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbObroundPadFacts {
    /// Exact-set facts across spine endpoints and diameter.
    pub exact: RealExactSetFacts,
    /// Diameter sign class.
    pub diameter_class: TraceWidthClass,
    /// Whether the retained spine endpoints are structurally identical.
    pub degenerate_spine: Option<bool>,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact obround/capsule PCB pad.
///
/// The pad is the Minkowski sum of a retained spine segment and a disk of
/// `diameter / 2`. Clearance to a swept trace is therefore decided by the
/// exact squared distance between the two retained spine/centerline segments
/// compared against exact squared swept diameters. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// candidate routing may be heuristic, but object construction and clearance
/// acceptance are exact and report-bearing. Degenerate spines are allowed and
/// represent round pads without losing their source family.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbObroundPad {
    net: crate::pcb::NetId,
    layer: crate::pcb::TraceLayer,
    spine: LinePathSegment,
    diameter: Real,
    facts: PcbObroundPadFacts,
}

impl PcbObroundPad {
    /// Construct an obround pad with native provenance.
    pub fn new(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        spine: LinePathSegment,
        diameter: Real,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(net, layer, spine, diameter, PathProvenance::native())
    }

    /// Construct an obround pad with source provenance.
    ///
    /// The spine is retained exactly. No direction normalization is performed;
    /// the point/segment distance predicate below handles degenerate and
    /// non-axis-aligned spines without introducing square roots.
    pub fn with_provenance(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        spine: LinePathSegment,
        diameter: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        let diameter_class = match diameter.structural_facts().sign {
            Some(RealSign::Negative) => return Err("obround pad diameter must be nonnegative"),
            Some(RealSign::Zero) => TraceWidthClass::Zero,
            Some(RealSign::Positive) => TraceWidthClass::Positive,
            None => TraceWidthClass::Unknown,
        };
        let facts = PcbObroundPadFacts {
            exact: Real::exact_set_facts([
                &spine.start().x,
                &spine.start().y,
                &spine.end().x,
                &spine.end().y,
                &diameter,
            ]),
            diameter_class,
            degenerate_spine: spine.facts().known_degenerate,
            provenance,
        };
        Ok(Self {
            net,
            layer,
            spine,
            diameter,
            facts,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> crate::pcb::NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> crate::pcb::TraceLayer {
        self.layer
    }

    /// Return exact retained spine.
    pub const fn spine(&self) -> &LinePathSegment {
        &self.spine
    }

    /// Return exact pad diameter.
    pub const fn diameter(&self) -> &Real {
        &self.diameter
    }

    /// Return cached exact facts.
    pub const fn facts(&self) -> &PcbObroundPadFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Check same-layer different-net clearance between a trace and an obround pad.
///
/// The predicate uses exact segment intersection to certify copper contact and
/// otherwise computes the minimum endpoint-to-opposite-segment squared
/// distance. Interior foot distances use `cross^2 / |segment|^2`; endpoint
/// branches are chosen with exact dot-product comparisons. No primitive float
/// tolerance or polygonal arc approximation participates in the decision.
pub fn check_trace_obround_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbObroundPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer() != pad.layer() || trace.net() == pad.net() {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        segment_segment_distance_squared(trace.swept().centerline(), pad.spine(), policy)
    else {
        return unknown_trace_report();
    };
    classify_swept_segment_distance(
        distance_squared,
        trace.swept().width(),
        pad.diameter(),
        required_clearance,
        policy,
    )
}

/// Check exact clearance from an obround pad to an axis-aligned rectangular board.
///
/// This is a rectangular-envelope manufacturing predicate, not board clipping.
/// The retained obround's bounding box is the exact spine coordinate extrema
/// expanded by `diameter / 2`, and each margin is compared directly with the
/// requested clearance.
pub fn check_obround_pad_board_clearance(
    pad: &PcbObroundPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let Some((min_x, max_x, min_y, max_y)) = obround_outer_bounds(pad, policy) else {
        return unknown_pad_board_report();
    };
    let margins = [
        min_x - board.min().x.clone(),
        board.max().x.clone() - max_x,
        min_y - board.min().y.clone(),
        board.max().y.clone() - max_y,
    ];
    match classify_margins(&margins, required_clearance, policy) {
        Some((status, copper_gap)) => PadBoardClearanceReport {
            status,
            copper_gap: Some(copper_gap),
        },
        None => unknown_pad_board_report(),
    }
}

fn classify_swept_segment_distance(
    distance_squared: Real,
    trace_width: &Real,
    pad_diameter: &Real,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit = trace_width.clone() + pad_diameter.clone();
    let overlap_limit_squared = overlap_limit.clone() * overlap_limit;
    let clearance_limit =
        trace_width.clone() + pad_diameter.clone() + required_clearance.clone() * Real::from(2);
    let clearance_limit_squared = clearance_limit.clone() * clearance_limit;
    let status =
        match compare_reals_with_policy(&four_distance_squared, &overlap_limit_squared, policy)
            .value()
        {
            Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::NoShortViolation,
            Some(Ordering::Greater) => {
                match compare_reals_with_policy(
                    &four_distance_squared,
                    &clearance_limit_squared,
                    policy,
                )
                .value()
                {
                    Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                    Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                    None => ClearanceStatus::Unknown,
                }
            }
            None => ClearanceStatus::Unknown,
        };
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

fn segment_segment_distance_squared(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let intersection = classify_segment_intersection_with_facts(
        first.start(),
        first.end(),
        second.start(),
        second.end(),
        first.facts().segment,
        second.facts().segment,
    )
    .value()?;
    if !matches!(intersection, SegmentIntersection::Disjoint) {
        return Some(Real::zero());
    }
    let mut minimum = None;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.start(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.end(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.start(), first, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.end(), first, policy)?,
        policy,
    )?;
    minimum
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

fn obround_outer_bounds(
    pad: &PcbObroundPad,
    policy: PredicatePolicy,
) -> Option<(Real, Real, Real, Real)> {
    let radius = (pad.diameter().clone() / Real::from(2)).ok()?;
    let min_x =
        real_min(&pad.spine().start().x, &pad.spine().end().x, policy)?.clone() - radius.clone();
    let max_x =
        real_max(&pad.spine().start().x, &pad.spine().end().x, policy)?.clone() + radius.clone();
    let min_y =
        real_min(&pad.spine().start().y, &pad.spine().end().y, policy)?.clone() - radius.clone();
    let max_y = real_max(&pad.spine().start().y, &pad.spine().end().y, policy)?.clone() + radius;
    Some((min_x, max_x, min_y, max_y))
}

fn classify_margins(
    margins: &[Real; 4],
    required: &Real,
    policy: PredicatePolicy,
) -> Option<(ClearanceStatus, Real)> {
    let mut minimum_margin = margins[0].clone();
    for margin in margins {
        if matches!(
            compare_reals_with_policy(margin, &minimum_margin, policy).value(),
            Some(Ordering::Less)
        ) {
            minimum_margin = margin.clone();
        }
        match compare_reals_with_policy(margin, required, policy).value()? {
            Ordering::Less => return Some((ClearanceStatus::ClearanceViolation, margin.clone())),
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    Some((ClearanceStatus::CertifiedClear, minimum_margin))
}

fn update_minimum_distance(
    minimum: &mut Option<Real>,
    candidate: Real,
    policy: PredicatePolicy,
) -> Option<()> {
    let replace = match minimum.as_ref() {
        Some(current) => {
            compare_reals_with_policy(&candidate, current, policy).value()? == Ordering::Less
        }
        None => true,
    };
    if replace {
        *minimum = Some(candidate);
    }
    Some(())
}

fn unknown_trace_report() -> TraceClearanceReport {
    TraceClearanceReport {
        status: ClearanceStatus::Unknown,
        centerline_intersection: None,
        axis_gap: None,
    }
}

fn unknown_pad_board_report() -> PadBoardClearanceReport {
    PadBoardClearanceReport {
        status: ClearanceStatus::Unknown,
        copper_gap: None,
    }
}

fn real_min<'a>(left: &'a Real, right: &'a Real, policy: PredicatePolicy) -> Option<&'a Real> {
    match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(left),
        Ordering::Greater => Some(right),
    }
}

fn real_max<'a>(left: &'a Real, right: &'a Real, policy: PredicatePolicy) -> Option<&'a Real> {
    match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(right),
        Ordering::Greater => Some(left),
    }
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
