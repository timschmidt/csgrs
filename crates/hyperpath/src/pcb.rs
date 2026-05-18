//! PCB trace path predicates.
//!
//! Classical PCB autorouters such as Lee's maze router and Hightower's line
//! router are excellent candidate generators, but their candidates must be
//! checked against exact swept geometry before they become trusted topology.
//! This module starts that boundary for straight trace segments. See Lee,
//! "An Algorithm for Path Connections and Its Applications," *IRE
//! Transactions on Electronic Computers* 1961, and Hightower, "A solution to
//! line-routing problems on the continuous plane," DAC 1969. The exact
//! predicate discipline follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals_with_policy,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment, real_sign};
use crate::swept::SweptLineSegment;

/// Stable PCB net identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NetId(pub u32);

/// PCB copper layer identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct TraceLayer(pub u16);

/// Coarse trace width class for routing schedule decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceWidthClass {
    /// Width is exactly zero.
    Zero,
    /// Width is structurally positive.
    Positive,
    /// Width sign is not structurally known.
    Unknown,
}

/// Cached facts for one trace segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbTraceFacts {
    /// Exact-set facts across geometry and width.
    pub exact: RealExactSetFacts,
    /// Trace width class.
    pub width_class: TraceWidthClass,
    /// Whether this trace is axis-aligned.
    pub axis_aligned: Option<Axis>,
    /// Source provenance inherited from the swept geometry.
    pub provenance: PathProvenance,
}

/// Straight swept PCB trace segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbTrace {
    net: NetId,
    layer: TraceLayer,
    swept: SweptLineSegment,
    facts: PcbTraceFacts,
}

/// Cached facts for a circular pad or via land.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbPadFacts {
    /// Exact-set facts across center coordinates and diameter.
    pub exact: RealExactSetFacts,
    /// Diameter class.
    pub diameter_class: TraceWidthClass,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact circular pad/via approximation for first routing predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCircularPad {
    net: NetId,
    layer: TraceLayer,
    center: Point2,
    diameter: Real,
    facts: PcbPadFacts,
}

/// Exact axis-aligned rectangular pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbRectPad {
    net: NetId,
    layer: TraceLayer,
    center: Point2,
    width: Real,
    height: Real,
    provenance: PathProvenance,
}

impl PcbRectPad {
    /// Construct an axis-aligned rectangular pad.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(net, layer, center, width, height, PathProvenance::native())
    }

    /// Construct an axis-aligned rectangular pad with source provenance.
    ///
    /// This is the first non-circular pad carrier. Rotated pads and rounded
    /// rectangles should be represented later as exact polygon/arc path
    /// geometry; this type keeps common SMD rectangular-pad clearance inside a
    /// small exact predicate surface.
    pub fn with_provenance(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        if real_sign(&width) == Some(hyperreal::RealSign::Negative) {
            return Err("rect pad width must be nonnegative");
        }
        if real_sign(&height) == Some(hyperreal::RealSign::Negative) {
            return Err("rect pad height must be nonnegative");
        }
        Ok(Self {
            net,
            layer,
            center,
            width,
            height,
            provenance,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact width.
    pub const fn width(&self) -> &Real {
        &self.width
    }

    /// Return exact height.
    pub const fn height(&self) -> &Real {
        &self.height
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }
}

/// Exact via stack spanning a contiguous layer range.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbViaStack {
    net: NetId,
    start_layer: TraceLayer,
    end_layer: TraceLayer,
    pad: PcbCircularPad,
    drill_diameter: Option<Real>,
}

impl PcbViaStack {
    /// Construct a via stack with the same circular land on every spanned layer.
    pub fn new(
        net: NetId,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
    ) -> Result<Self, &'static str> {
        if start_layer > end_layer {
            return Err("via start layer must not be above end layer");
        }
        let pad = PcbCircularPad::new(net, start_layer, center, land_diameter)?;
        Ok(Self {
            net,
            start_layer,
            end_layer,
            pad,
            drill_diameter: None,
        })
    }

    /// Construct a via stack with an exact drill diameter.
    pub fn with_drill(
        net: NetId,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
        drill_diameter: Real,
    ) -> Result<Self, &'static str> {
        let mut via = Self::new(net, start_layer, end_layer, center, land_diameter)?;
        if real_sign(&drill_diameter) == Some(hyperreal::RealSign::Negative) {
            return Err("via drill diameter must be nonnegative");
        }
        via.drill_diameter = Some(drill_diameter);
        Ok(via)
    }

    /// Return whether this via is present on `layer`.
    pub const fn spans_layer(&self, layer: TraceLayer) -> bool {
        self.start_layer.0 <= layer.0 && layer.0 <= self.end_layer.0
    }

    /// Return via net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return via center.
    pub const fn center(&self) -> &Point2 {
        self.pad.center()
    }

    /// Return via land diameter.
    pub const fn land_diameter(&self) -> &Real {
        self.pad.diameter()
    }

    /// Return optional exact drill diameter.
    pub const fn drill_diameter(&self) -> Option<&Real> {
        self.drill_diameter.as_ref()
    }

    /// Certify the annular ring against an exact minimum requirement.
    ///
    /// This checks `land >= drill + 2*minimum`. The comparison is exact and
    /// belongs at the fabrication-rule boundary; trace/via topology still uses
    /// clearance predicates. IPC-style annular-ring policy can layer richer
    /// manufacturing classes over this exact primitive later.
    pub fn certify_annular_ring(
        &self,
        minimum: &Real,
        policy: PredicatePolicy,
    ) -> ViaAnnularRingReport {
        let Some(drill) = self.drill_diameter.as_ref() else {
            return ViaAnnularRingReport::UnknownNoDrill;
        };
        if real_sign(minimum) == Some(hyperreal::RealSign::Negative) {
            return ViaAnnularRingReport::InvalidMinimum;
        }
        let required = drill.clone() + minimum.clone() * Real::from(2);
        match compare_reals_with_policy(self.land_diameter(), &required, policy).value() {
            Some(Ordering::Less) => ViaAnnularRingReport::Violation,
            Some(Ordering::Equal | Ordering::Greater) => ViaAnnularRingReport::Certified,
            None => ViaAnnularRingReport::Unknown,
        }
    }
}

/// Exact annular-ring certification result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaAnnularRingReport {
    /// The via land is certified large enough for drill plus minimum ring.
    Certified,
    /// The via land is certified too small.
    Violation,
    /// No drill diameter was available.
    UnknownNoDrill,
    /// The minimum annular ring was invalid.
    InvalidMinimum,
    /// Exact comparison could not decide.
    Unknown,
}

impl PcbCircularPad {
    /// Construct a circular pad or via land with native provenance.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        diameter: Real,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(net, layer, center, diameter, PathProvenance::native())
    }

    /// Construct a circular pad or via land with source provenance.
    ///
    /// This stores the exact center and diameter. More specific pad shapes
    /// should add exact polygon/arc carriers later; this circular carrier is
    /// enough for first via and round-pad clearance predicates.
    pub fn with_provenance(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        diameter: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        let diameter_class = match real_sign(&diameter) {
            Some(hyperreal::RealSign::Negative) => {
                return Err("pad diameter must be nonnegative");
            }
            Some(hyperreal::RealSign::Zero) => TraceWidthClass::Zero,
            Some(hyperreal::RealSign::Positive) => TraceWidthClass::Positive,
            None => TraceWidthClass::Unknown,
        };
        let facts = PcbPadFacts {
            exact: Real::exact_set_facts([&center.x, &center.y, &diameter]),
            diameter_class,
            provenance,
        };
        Ok(Self {
            net,
            layer,
            center,
            diameter,
            facts,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return exact center point.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact diameter.
    pub const fn diameter(&self) -> &Real {
        &self.diameter
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &PcbPadFacts {
        &self.facts
    }
}

impl PcbTrace {
    /// Construct a PCB trace from swept geometry and net metadata.
    pub fn new(net: NetId, layer: TraceLayer, swept: SweptLineSegment) -> Self {
        let width_class = match real_sign(swept.width()) {
            Some(hyperreal::RealSign::Zero) => TraceWidthClass::Zero,
            Some(hyperreal::RealSign::Positive) => TraceWidthClass::Positive,
            _ => TraceWidthClass::Unknown,
        };
        let facts = PcbTraceFacts {
            exact: swept.facts().exact,
            width_class,
            axis_aligned: swept.centerline().facts().axis_aligned,
            provenance: swept.provenance(),
        };
        Self {
            net,
            layer,
            swept,
            facts,
        }
    }

    /// Return net identifier.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return trace layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return swept trace geometry.
    pub const fn swept(&self) -> &SweptLineSegment {
        &self.swept
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &PcbTraceFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Exact clearance classification between two straight trace segments.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ClearanceStatus {
    /// Same-net or different-layer traces are not clearance violations here.
    NotApplicable,
    /// Centerlines intersect on the same layer and different nets.
    NoShortViolation,
    /// Axis-aligned swept geometry is certified separated.
    CertifiedClear,
    /// Axis-aligned swept geometry is certified too close.
    ClearanceViolation,
    /// Current exact specialized predicates could not decide this shape.
    Unknown,
}

/// Report for one trace-pair clearance check.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceClearanceReport {
    /// Classification status.
    pub status: ClearanceStatus,
    /// Centerline intersection, if it was decided.
    pub centerline_intersection: Option<SegmentIntersection>,
    /// Exact distance gap used by specialized axis-aligned checks.
    pub axis_gap: Option<Real>,
}

impl TraceClearanceReport {
    /// Returns whether this report proves the pair is route-safe.
    pub const fn is_certified_clear(&self) -> bool {
        matches!(
            self.status,
            ClearanceStatus::NotApplicable | ClearanceStatus::CertifiedClear
        )
    }
}

/// Check same-layer different-net trace clearance.
pub fn check_trace_clearance(
    first: &PcbTrace,
    second: &PcbTrace,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if first.layer != second.layer || first.net == second.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }

    let centerline = classify_segment_intersection_with_facts(
        first.swept.centerline().start(),
        first.swept.centerline().end(),
        second.swept.centerline().start(),
        second.swept.centerline().end(),
        first.swept.centerline().facts().segment,
        second.swept.centerline().facts().segment,
    )
    .value();

    if matches!(
        centerline,
        Some(
            SegmentIntersection::Proper
                | SegmentIntersection::EndpointTouch
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical
        )
    ) {
        return TraceClearanceReport {
            status: ClearanceStatus::NoShortViolation,
            centerline_intersection: centerline,
            axis_gap: Some(Real::zero()),
        };
    }

    match axis_aligned_gap(first.swept.centerline(), second.swept.centerline(), policy) {
        Some(gap) => {
            let doubled_gap = gap.clone() * Real::from(2);
            let required = first.swept.width().clone()
                + second.swept.width().clone()
                + required_clearance.clone() * Real::from(2);
            let status = match compare_reals_with_policy(&doubled_gap, &required, policy).value() {
                Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                None => ClearanceStatus::Unknown,
            };
            TraceClearanceReport {
                status,
                centerline_intersection: centerline,
                axis_gap: Some(gap),
            }
        }
        None => TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: centerline,
            axis_gap: None,
        },
    }
}

/// Check same-layer different-net clearance between a trace and a circular pad.
///
/// The predicate compares squared exact distances and squared required
/// separation, avoiding square roots and primitive tolerances. This is the
/// same exact-predicate strategy as the trace/trace checker: candidate routing
/// may be heuristic, but copper overlap and clearance acceptance are certified
/// by exact scalar comparisons.
pub fn check_trace_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbCircularPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer != pad.layer || trace.net == pad.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        axis_aligned_point_segment_distance_squared(trace.swept.centerline(), pad.center(), policy)
    else {
        return TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        };
    };
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit = trace.swept.width().clone() + pad.diameter().clone();
    let overlap_limit_squared = overlap_limit.clone() * overlap_limit;
    let clearance_limit = trace.swept.width().clone()
        + pad.diameter().clone()
        + required_clearance.clone() * Real::from(2);
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

/// Check a trace against a via stack on the trace's layer.
pub fn check_trace_via_clearance(
    trace: &PcbTrace,
    via: &PcbViaStack,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if !via.spans_layer(trace.layer()) {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let pad = PcbCircularPad::new(
        via.net(),
        trace.layer(),
        via.center().clone(),
        via.land_diameter().clone(),
    )
    .expect("validated via stack land diameter remains valid");
    check_trace_pad_clearance(trace, &pad, required_clearance, policy)
}

/// Check an axis-aligned trace against an axis-aligned rectangular pad.
///
/// The checker compares exact squared distances to squared copper-overlap and
/// clearance limits, avoiding square roots and primitive tolerances. It is a
/// specialized exact predicate for common rectangular SMD pads; general pad
/// outlines should later route through exact polygon/arc arrangements.
pub fn check_trace_rect_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbRectPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer != pad.layer || trace.net == pad.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        axis_aligned_segment_rect_distance_squared(trace.swept.centerline(), pad, policy)
    else {
        return TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        };
    };
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit = trace.swept.width().clone();
    let overlap_limit_squared = overlap_limit.clone() * overlap_limit;
    let clearance_limit = trace.swept.width().clone() + required_clearance.clone() * Real::from(2);
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

fn axis_aligned_gap(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    match (first.facts().axis_aligned, second.facts().axis_aligned) {
        (Some(Axis::X), Some(Axis::X))
            if intervals_overlap(
                &first.start().x,
                &first.end().x,
                &second.start().x,
                &second.end().x,
                policy,
            )? =>
        {
            coordinate_gap(&first.start().y, &second.start().y, policy)
        }
        (Some(Axis::Y), Some(Axis::Y))
            if intervals_overlap(
                &first.start().y,
                &first.end().y,
                &second.start().y,
                &second.end().y,
                policy,
            )? =>
        {
            coordinate_gap(&first.start().x, &second.start().x, policy)
        }
        (Some(Axis::X), Some(Axis::Y)) | (Some(Axis::Y), Some(Axis::X)) => {
            // If centerlines did not intersect, perpendicular axis-aligned
            // segments are separated in at least one axis. Use the larger
            // certified axis gap as a conservative lower bound only when one
            // axis gap is zero and the other is known.
            let x_gap = interval_gap(
                &first.start().x,
                &first.end().x,
                &second.start().x,
                &second.end().x,
                policy,
            )?;
            let y_gap = interval_gap(
                &first.start().y,
                &first.end().y,
                &second.start().y,
                &second.end().y,
                policy,
            )?;
            if real_sign(&x_gap) == Some(hyperreal::RealSign::Zero) {
                Some(y_gap)
            } else if real_sign(&y_gap) == Some(hyperreal::RealSign::Zero) {
                Some(x_gap)
            } else {
                None
            }
        }
        _ => None,
    }
}

fn intervals_overlap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = real_max(real_min(a0, a1, policy)?, real_min(b0, b1, policy)?, policy)?;
    let upper = real_min(real_max(a0, a1, policy)?, real_max(b0, b1, policy)?, policy)?;
    Some(!matches!(
        compare_reals_with_policy(lower, upper, policy).value()?,
        Ordering::Greater
    ))
}

fn interval_gap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    if intervals_overlap(a0, a1, b0, b1, policy)? {
        return Some(Real::zero());
    }
    let a_max = real_max(a0, a1, policy)?;
    let b_min = real_min(b0, b1, policy)?;
    if compare_reals_with_policy(a_max, b_min, policy).value()? == Ordering::Less {
        return Some(b_min.clone() - a_max.clone());
    }
    let b_max = real_max(b0, b1, policy)?;
    let a_min = real_min(a0, a1, policy)?;
    Some(a_min.clone() - b_max.clone())
}

fn coordinate_gap(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(first, second, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(second.clone() - first.clone()),
        Ordering::Greater => Some(first.clone() - second.clone()),
    }
}

fn axis_aligned_point_segment_distance_squared(
    segment: &LinePathSegment,
    point: &Point2,
    policy: PredicatePolicy,
) -> Option<Real> {
    let (axis_gap, cross_gap) = match segment.facts().axis_aligned {
        Some(Axis::X) => (
            interval_point_gap(&segment.start().x, &segment.end().x, &point.x, policy)?,
            coordinate_gap(&segment.start().y, &point.y, policy)?,
        ),
        Some(Axis::Y) => (
            interval_point_gap(&segment.start().y, &segment.end().y, &point.y, policy)?,
            coordinate_gap(&segment.start().x, &point.x, policy)?,
        ),
        None => return None,
    };
    Some(Real::signed_product_sum(
        [true, true],
        [[&axis_gap, &axis_gap], [&cross_gap, &cross_gap]],
    ))
}

fn interval_point_gap(a0: &Real, a1: &Real, point: &Real, policy: PredicatePolicy) -> Option<Real> {
    let min = real_min(a0, a1, policy)?;
    let max = real_max(a0, a1, policy)?;
    if compare_reals_with_policy(point, min, policy).value()? == Ordering::Less {
        return Some(min.clone() - point.clone());
    }
    if compare_reals_with_policy(point, max, policy).value()? == Ordering::Greater {
        return Some(point.clone() - max.clone());
    }
    Some(Real::zero())
}

fn axis_aligned_segment_rect_distance_squared(
    segment: &LinePathSegment,
    pad: &PcbRectPad,
    policy: PredicatePolicy,
) -> Option<Real> {
    let half_width = (pad.width().clone() / Real::from(2)).ok()?;
    let half_height = (pad.height().clone() / Real::from(2)).ok()?;
    let min_x = pad.center().x.clone() - half_width.clone();
    let max_x = pad.center().x.clone() + half_width;
    let min_y = pad.center().y.clone() - half_height.clone();
    let max_y = pad.center().y.clone() + half_height;

    let x_gap = interval_gap(&segment.start().x, &segment.end().x, &min_x, &max_x, policy)?;
    let y_gap = interval_gap(&segment.start().y, &segment.end().y, &min_y, &max_y, policy)?;
    Some(Real::signed_product_sum(
        [true, true],
        [[&x_gap, &x_gap], [&y_gap, &y_gap]],
    ))
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
