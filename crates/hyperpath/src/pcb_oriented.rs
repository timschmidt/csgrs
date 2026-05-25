//! Exact oriented PCB pad predicates.
//!
//! This module covers the first arbitrary-angle rectangular PCB pad carrier in
//! `hyperpath`. It deliberately remains a retained path/CAM/PCB record plus
//! exact clearance predicates: copper union, board clipping, and solid
//! materialization stay in `hypermesh`.

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

/// Cached facts for an exact oriented rectangular pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbOrientedRectPadFacts {
    /// Exact-set facts across center, extents, and local X-axis coordinates.
    pub exact: RealExactSetFacts,
    /// Width sign class.
    pub width_class: TraceWidthClass,
    /// Height sign class.
    pub height_class: TraceWidthClass,
    /// Exact unit-length certificate value for the retained local X axis.
    pub local_x_length_squared: Real,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact arbitrary-angle rectangular PCB pad with a retained unit local axis.
///
/// The orientation is stored as an exact unit vector `local_x`; `local_y` is
/// derived by the exact perpendicular `(-y, x)`. Construction rejects
/// non-unit vectors instead of normalizing them through a square root. The
/// clearance predicate then checks the retained centerline against the four
/// exact pad edges and compares exact squared distances. This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): heuristic route generation may propose a segment, but exact object
/// facts and predicates decide whether the candidate is accepted. It also
/// preserves the Lee/Hightower routing split between candidate paths and final
/// geometric certification.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbOrientedRectPad {
    net: crate::pcb::NetId,
    layer: crate::pcb::TraceLayer,
    center: Point2,
    width: Real,
    height: Real,
    local_x: Point2,
    facts: PcbOrientedRectPadFacts,
}

impl PcbOrientedRectPad {
    /// Construct an oriented rectangular pad with native provenance.
    pub fn new(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        local_x: Point2,
        policy: PredicatePolicy,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(
            net,
            layer,
            center,
            width,
            height,
            local_x,
            policy,
            PathProvenance::native(),
        )
    }

    /// Construct an oriented rectangular pad with source provenance.
    ///
    /// `local_x` must already be an exact unit vector. This is intentionally a
    /// construction precondition rather than an automatic normalization step:
    /// square-root normalization would manufacture a new object whose exactness
    /// and branch provenance are not present in typical footprint input.
    pub fn with_provenance(
        net: crate::pcb::NetId,
        layer: crate::pcb::TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        local_x: Point2,
        policy: PredicatePolicy,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        let width_class = classify_nonnegative_extent(&width, "oriented rect pad width")?;
        let height_class = classify_nonnegative_extent(&height, "oriented rect pad height")?;
        let local_x_length_squared = squared_norm(&local_x);
        if !matches!(
            compare_reals_with_policy(&local_x_length_squared, &Real::from(1), policy).value(),
            Some(Ordering::Equal)
        ) {
            return Err("oriented rect pad local x axis must be exact unit length");
        }
        let facts = PcbOrientedRectPadFacts {
            exact: Real::exact_set_facts([
                &center.x, &center.y, &width, &height, &local_x.x, &local_x.y,
            ]),
            width_class,
            height_class,
            local_x_length_squared,
            provenance,
        };
        Ok(Self {
            net,
            layer,
            center,
            width,
            height,
            local_x,
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

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact full local-X width.
    pub const fn width(&self) -> &Real {
        &self.width
    }

    /// Return exact full local-Y height.
    pub const fn height(&self) -> &Real {
        &self.height
    }

    /// Return retained exact local X unit vector.
    pub const fn local_x(&self) -> &Point2 {
        &self.local_x
    }

    /// Return derived exact local Y unit vector.
    pub fn local_y(&self) -> Point2 {
        Point2::new(-self.local_x.y.clone(), self.local_x.x.clone())
    }

    /// Return cached exact facts.
    pub const fn facts(&self) -> &PcbOrientedRectPadFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Check a trace against an exact oriented rectangular pad.
///
/// The test first replays exact segment/edge intersection against the retained
/// pad corners. If there is no contact, it computes the minimum of all exact
/// endpoint-to-opposite-segment squared distances. Interior foot distances use
/// the standard denominator-cleared projection formula `cross^2 / |edge|^2`;
/// endpoint branches use exact dot-product comparisons, avoiding square roots
/// and primitive tolerances.
pub fn check_trace_oriented_rect_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbOrientedRectPad,
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
        segment_oriented_rect_distance_squared(trace.swept().centerline(), pad, policy)
    else {
        return TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        };
    };
    classify_swept_distance_squared(
        distance_squared,
        trace.swept().width(),
        required_clearance,
        policy,
    )
}

/// Check exact clearance from an oriented rectangular pad to a rectangular board.
///
/// For an axis-aligned board envelope, pad-to-edge clearance is decided by the
/// exact extrema of the four retained oriented pad corners. This is not board
/// clipping or copper boolean execution; it is a path-domain manufacturing
/// predicate that reports whether the retained source pad fits the retained
/// rectangular board by the requested gap.
pub fn check_oriented_rect_pad_board_clearance(
    pad: &PcbOrientedRectPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let Some(corners) = oriented_rect_corners(pad) else {
        return unknown_pad_board_report();
    };
    let Some((min_x, max_x, min_y, max_y)) = corner_extrema(&corners, policy) else {
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

fn classify_nonnegative_extent(
    value: &Real,
    label: &'static str,
) -> Result<TraceWidthClass, &'static str> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) if label == "oriented rect pad width" => {
            Err("oriented rect pad width must be nonnegative")
        }
        Some(RealSign::Negative) => Err("oriented rect pad height must be nonnegative"),
        Some(RealSign::Zero) => Ok(TraceWidthClass::Zero),
        Some(RealSign::Positive) => Ok(TraceWidthClass::Positive),
        None => Ok(TraceWidthClass::Unknown),
    }
}

fn classify_swept_distance_squared(
    distance_squared: Real,
    trace_width: &Real,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit_squared = trace_width.clone() * trace_width.clone();
    let clearance_limit = trace_width.clone() + required_clearance.clone() * Real::from(2);
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

fn segment_oriented_rect_distance_squared(
    segment: &LinePathSegment,
    pad: &PcbOrientedRectPad,
    policy: PredicatePolicy,
) -> Option<Real> {
    let corners = oriented_rect_corners(pad)?;
    if point_inside_oriented_rect(segment.start(), pad, policy)?
        || point_inside_oriented_rect(segment.end(), pad, policy)?
    {
        return Some(Real::zero());
    }
    let edges = oriented_rect_edges(&corners);
    for edge in &edges {
        let intersection = classify_segment_intersection_with_facts(
            segment.start(),
            segment.end(),
            edge.start(),
            edge.end(),
            segment.facts().segment,
            edge.facts().segment,
        )
        .value()?;
        if !matches!(intersection, SegmentIntersection::Disjoint) {
            return Some(Real::zero());
        }
    }
    let mut minimum = None;
    for edge in &edges {
        update_minimum_distance(
            &mut minimum,
            point_segment_distance_squared(segment.start(), edge, policy)?,
            policy,
        )?;
        update_minimum_distance(
            &mut minimum,
            point_segment_distance_squared(segment.end(), edge, policy)?,
            policy,
        )?;
        update_minimum_distance(
            &mut minimum,
            point_segment_distance_squared(edge.start(), segment, policy)?,
            policy,
        )?;
        update_minimum_distance(
            &mut minimum,
            point_segment_distance_squared(edge.end(), segment, policy)?,
            policy,
        )?;
    }
    minimum
}

fn point_inside_oriented_rect(
    point: &Point2,
    pad: &PcbOrientedRectPad,
    policy: PredicatePolicy,
) -> Option<bool> {
    let half_width = (pad.width().clone() / Real::from(2)).ok()?;
    let half_height = (pad.height().clone() / Real::from(2)).ok()?;
    let delta = Point2::new(
        point.x.clone() - pad.center().x.clone(),
        point.y.clone() - pad.center().y.clone(),
    );
    let local_x = dot(&delta, pad.local_x());
    let local_y = dot(&delta, &pad.local_y());
    Some(
        !matches!(
            compare_reals_with_policy(&abs_real(&local_x, policy)?, &half_width, policy).value()?,
            Ordering::Greater
        ) && !matches!(
            compare_reals_with_policy(&abs_real(&local_y, policy)?, &half_height, policy)
                .value()?,
            Ordering::Greater
        ),
    )
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

fn oriented_rect_corners(pad: &PcbOrientedRectPad) -> Option<[Point2; 4]> {
    let half_width = (pad.width().clone() / Real::from(2)).ok()?;
    let half_height = (pad.height().clone() / Real::from(2)).ok()?;
    let local_y = pad.local_y();
    let ux = pad.local_x().x.clone() * half_width.clone();
    let uy = pad.local_x().y.clone() * half_width;
    let vx = local_y.x * half_height.clone();
    let vy = local_y.y * half_height;
    Some([
        Point2::new(
            pad.center().x.clone() - ux.clone() - vx.clone(),
            pad.center().y.clone() - uy.clone() - vy.clone(),
        ),
        Point2::new(
            pad.center().x.clone() + ux.clone() - vx.clone(),
            pad.center().y.clone() + uy.clone() - vy.clone(),
        ),
        Point2::new(
            pad.center().x.clone() + ux.clone() + vx.clone(),
            pad.center().y.clone() + uy.clone() + vy.clone(),
        ),
        Point2::new(
            pad.center().x.clone() - ux + vx,
            pad.center().y.clone() - uy + vy,
        ),
    ])
}

fn oriented_rect_edges(corners: &[Point2; 4]) -> [LinePathSegment; 4] {
    [
        LinePathSegment::new(corners[0].clone(), corners[1].clone()),
        LinePathSegment::new(corners[1].clone(), corners[2].clone()),
        LinePathSegment::new(corners[2].clone(), corners[3].clone()),
        LinePathSegment::new(corners[3].clone(), corners[0].clone()),
    ]
}

fn corner_extrema(
    corners: &[Point2; 4],
    policy: PredicatePolicy,
) -> Option<(Real, Real, Real, Real)> {
    let mut min_x = corners[0].x.clone();
    let mut max_x = corners[0].x.clone();
    let mut min_y = corners[0].y.clone();
    let mut max_y = corners[0].y.clone();
    for corner in corners.iter().skip(1) {
        if compare_reals_with_policy(&corner.x, &min_x, policy).value()? == Ordering::Less {
            min_x = corner.x.clone();
        }
        if compare_reals_with_policy(&corner.x, &max_x, policy).value()? == Ordering::Greater {
            max_x = corner.x.clone();
        }
        if compare_reals_with_policy(&corner.y, &min_y, policy).value()? == Ordering::Less {
            min_y = corner.y.clone();
        }
        if compare_reals_with_policy(&corner.y, &max_y, policy).value()? == Ordering::Greater {
            max_y = corner.y.clone();
        }
    }
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

fn unknown_pad_board_report() -> PadBoardClearanceReport {
    PadBoardClearanceReport {
        status: ClearanceStatus::Unknown,
        copper_gap: None,
    }
}

fn abs_real(value: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(value, &Real::zero(), policy).value()? {
        Ordering::Less => Some(-value.clone()),
        Ordering::Equal | Ordering::Greater => Some(value.clone()),
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
