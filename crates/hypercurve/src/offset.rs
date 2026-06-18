//! Primitive parallel offsets for line and circular-arc segments.
//!
//! Offsetting is split into primitive parallel curves, joins/caps, and later
//! trimming/rebuild work. The staged construction follows Tiller and Hanson,
//! "Offsets of Two-Dimensional Profiles" (*IEEE Computer Graphics and
//! Applications* 4(9), 36-46, 1984). The reason checked offsets reject raw
//! self-intersections instead of accepting them is the offset-curve topology
//! described by Farouki and Neff, "Analytic Properties of Plane Offset Curves"
//! (*Computer Aided Geometric Design* 7(1-4), 83-99, 1990), where offsets may
//! form cusps and extraneous loops that require trimming.

use hyperreal::{Real, RealSign};

use crate::classify::{is_zero, real_sign};
use crate::contour::{Contour2, FillRule};
use crate::curve_string::CurveString2;
use crate::segment::{CircularArc2, LineSeg2, Segment2};
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, Point2, RetainedTopologyStatus,
    SegmentKindCounts, SelfContactReport2, UncertaintyReason,
};

/// Endpoint cap style for checked open curve-string outlines.
///
/// The cap is applied after the source curve string has been offset on both
/// sides. This enum describes only the endpoint construction; joins along the
/// left and right traces still use the primitive offset and line/round-join
/// machinery documented on [`CurveString2::offset_left_with_line_joins`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OffsetCap {
    /// Connect left and right traces with circular arcs centered on endpoints.
    Round,
    /// Connect left and right traces directly at each endpoint.
    Butt,
    /// Extend each trace by one half-width along endpoint tangents before
    /// adding straight endpoint connectors.
    Square,
}

/// Furthest exact stage reached by checked open curve-string offsetting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringOffsetStage2 {
    /// Primitive offset segments and joins were being materialized.
    OffsetConstruction,
    /// The raw joined offset was checked for self-contacting topology.
    SelfContactValidation,
}

/// Report for a checked open curve-string left offset.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOffsetReport2 {
    stage: CurveStringOffsetStage2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    raw_offset_segment_count: Option<usize>,
    raw_offset_segment_kind_counts: Option<SegmentKindCounts>,
    self_contact_report: Option<SelfContactReport2>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing checked open curve-string offsetting.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOffsetResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringOffsetReport2,
}

/// Furthest exact stage reached by checked closed-contour offsetting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourOffsetStage2 {
    /// Primitive offset segments and joins were being materialized.
    OffsetConstruction,
    /// The raw joined offset was checked for self-contacting topology.
    SelfContactValidation,
}

/// Report for a checked closed-contour left offset.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOffsetReport2 {
    stage: ContourOffsetStage2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    raw_offset_segment_count: Option<usize>,
    raw_offset_segment_kind_counts: Option<SegmentKindCounts>,
    self_contact_report: Option<SelfContactReport2>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing checked closed-contour offsetting.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOffsetResult2 {
    contour: Option<Contour2>,
    report: ContourOffsetReport2,
}

/// Furthest exact stage reached by checked open curve-string outline offsetting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringOutlineOffsetStage2 {
    /// The requested outline half-width was being classified.
    DistanceValidation,
    /// The source open curve string was checked for self-contacting topology.
    SourceSelfContactValidation,
    /// The left offset trace was being materialized.
    LeftOffsetConstruction,
    /// The right offset trace was being materialized.
    RightOffsetConstruction,
    /// Endpoint caps and cap-specific trace extensions were being materialized.
    CapConstruction,
    /// The closed outline contour was checked for closure and self-contacting topology.
    OutlineTopologyValidation,
}

/// Report for a checked closed outline around an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOutlineOffsetReport2 {
    stage: CurveStringOutlineOffsetStage2,
    cap: OffsetCap,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    left_offset_segment_count: Option<usize>,
    left_offset_segment_kind_counts: Option<SegmentKindCounts>,
    right_offset_segment_count: Option<usize>,
    right_offset_segment_kind_counts: Option<SegmentKindCounts>,
    outline_segment_count: Option<usize>,
    outline_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing checked open curve-string outline offsetting.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOutlineOffsetResult2 {
    outline: Option<Contour2>,
    report: CurveStringOutlineOffsetReport2,
}

impl LineSeg2 {
    /// Returns the constant-distance segment on this segment's left side.
    ///
    /// The offset direction is the normalized left normal `(-dy, dx) / length`.
    /// This is the primitive line-profile case used by profile offset
    /// algorithms; higher-level curve-string offsetting must still add joins,
    /// trim self-intersections, and rebuild topology. See Tiller and Hanson,
    /// "Offsets of Two-Dimensional Profiles" (1984), for the line/arc
    /// primitive plus trim-and-join framing used by many CAD offset pipelines.
    pub fn offset_left(&self, distance: Real) -> CurveResult<Self> {
        let length = self.length_squared().sqrt()?;
        let (dx, dy) = self.delta();
        let normal_x = ((-dy) / &length)?;
        let normal_y = (dx / &length)?;
        let offset_x = &normal_x * &distance;
        let offset_y = &normal_y * &distance;

        Self::try_new(
            self.start().translated(offset_x.clone(), offset_y.clone()),
            self.end().translated(offset_x, offset_y),
        )
    }
}

impl CircularArc2 {
    /// Returns the constant-distance arc on this arc's left side.
    ///
    /// Counter-clockwise arcs have their left normal on the circle interior, so
    /// a positive offset decreases radius. Clockwise arcs have their left normal
    /// on the exterior, so a positive offset increases radius. Radius collapse
    /// and radius sign reversal are returned as explicit uncertainty because
    /// the primitive arc no longer has a valid circular-arc image at that
    /// distance. Tiller and Hanson, "Offsets of Two-Dimensional Profiles"
    /// (1984), describe this concentric-arc primitive as one step in a complete
    /// profile offset pipeline.
    pub fn offset_left(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let radius = self.radius_squared().sqrt()?;
        let offset_radius = if self.is_clockwise() {
            &radius + &distance
        } else {
            &radius - &distance
        };

        match real_sign(&offset_radius, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        let radius_scale = (offset_radius / &radius)?;
        let offset = Self::try_from_center_with_bulge(
            scale_from_center(self.start(), self.center(), &radius_scale),
            scale_from_center(self.end(), self.center(), &radius_scale),
            self.center().clone(),
            self.is_clockwise(),
            self.bulge().cloned(),
        )?;
        Ok(Classification::Decided(offset))
    }
}

impl Segment2 {
    /// Returns this segment's left-side primitive offset.
    ///
    /// Lines always produce a translated line. Arcs produce a concentric arc
    /// when the requested distance leaves a positive radius; radius collapse or
    /// reversal is reported as uncertainty instead of fabricating degenerate
    /// topology.
    pub fn offset_left(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Line(line) => line
                .offset_left(distance)
                .map(Self::Line)
                .map(Classification::Decided),
            Self::Arc(arc) => arc
                .offset_left(distance, policy)
                .map(|arc| arc.map(Segment2::Arc)),
        }
    }
}

impl CurveString2 {
    /// Returns a left offset of this open curve string with straight-line joins.
    ///
    /// This is a raw offset-construction layer, not a full offset engine. Each
    /// source segment is first replaced by its primitive parallel offset. Adjacent
    /// offset lines are mitered by intersecting their supporting lines; joins
    /// that cannot be mitered are connected by a circular arc centered at the
    /// original shared vertex. A complete profile offset pipeline still has to
    /// classify join style, trim self-intersections, and cap open endpoints.
    /// Tiller and Hanson, "Offsets of Two-Dimensional Profiles" (1984),
    /// describe this staged primitive, join, and trim structure for
    /// two-dimensional profile offsets.
    pub fn offset_left_with_line_joins(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }

        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(self.segments(), &offsets, false, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(checked_joined_curve_string(joined))
    }

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections or cap open endpoints. It
    /// runs the joined open offset construction and then classifies the result
    /// with [`CurveString2::has_self_contacts`]. A detected self contact is
    /// reported as explicit uncertainty so callers can choose a future trimming
    /// path instead of consuming invalid raw linework. Farouki and Neff,
    /// "Analytic Properties of Plane Offset Curves" (1990), describe exactly
    /// these self-intersections and extraneous loops as offset topology that
    /// must be trimmed before the curve can represent the intended profile.
    pub fn offset_left_checked(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let result = self.offset_left_checked_with_report(distance, policy)?;
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(curve_string) = result.into_curve_string() {
            Ok(Classification::Decided(curve_string))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Returns a report-bearing raw joined left offset, rejecting self-contacting output.
    ///
    /// The report records source inventory, the raw joined offset segment count
    /// before self-contact validation, the final output count when accepted,
    /// and the exact blocker otherwise.
    pub fn offset_left_checked_with_report(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringOffsetResult2> {
        let source_segment_count = self.len();
        let source_segment_kind_counts = segment_kind_counts(self.segments());
        let offset = match self.offset_left_with_line_joins(distance, policy)? {
            Classification::Decided(offset) => offset,
            Classification::Uncertain(reason) => {
                return Ok(blocked_curve_string_offset_result(
                    CurveStringOffsetStage2::OffsetConstruction,
                    source_segment_count,
                    source_segment_kind_counts,
                    None,
                    None,
                    None,
                    retained_status_for_offset_blocker(reason),
                    reason,
                ));
            }
        };
        let raw_offset_segment_count = offset.len();
        let raw_offset_segment_kind_counts = segment_kind_counts(offset.segments());

        let self_contact = offset.has_self_contacts_with_report(policy)?;
        match self_contact.has_self_contacts() {
            Classification::Decided(false) => Ok(CurveStringOffsetResult2 {
                curve_string: Some(offset),
                report: CurveStringOffsetReport2 {
                    stage: CurveStringOffsetStage2::SelfContactValidation,
                    source_segment_count,
                    source_segment_kind_counts,
                    raw_offset_segment_count: Some(raw_offset_segment_count),
                    raw_offset_segment_kind_counts: Some(raw_offset_segment_kind_counts),
                    self_contact_report: Some(self_contact.report().clone()),
                    output_segment_count: Some(raw_offset_segment_count),
                    output_segment_kind_counts: Some(raw_offset_segment_kind_counts),
                    status: RetainedTopologyStatus::NativeExact,
                    blocker: None,
                },
            }),
            Classification::Decided(true) => Ok(blocked_curve_string_offset_result(
                CurveStringOffsetStage2::SelfContactValidation,
                source_segment_count,
                source_segment_kind_counts,
                Some(raw_offset_segment_count),
                Some(raw_offset_segment_kind_counts),
                Some(self_contact.report().clone()),
                RetainedTopologyStatus::Unsupported,
                UncertaintyReason::Unsupported,
            )),
            Classification::Uncertain(reason) => Ok(blocked_curve_string_offset_result(
                CurveStringOffsetStage2::SelfContactValidation,
                source_segment_count,
                source_segment_kind_counts,
                Some(raw_offset_segment_count),
                Some(raw_offset_segment_kind_counts),
                Some(self_contact.report().clone()),
                retained_status_for_offset_blocker(reason),
                reason,
            )),
        }
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// The outline follows the left joined offset, applies the selected
    /// [`OffsetCap`] at the end point, returns along the reversed right joined
    /// offset, and applies the matching cap at the start point. The `distance`
    /// is the half-width of the outline and must be strictly positive under
    /// the active policy. As with [`CurveString2::offset_left_checked`], this
    /// is still the raw offset-construction stage described by Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles" (1984): self-contacting
    /// input or output is rejected as explicit uncertainty until the
    /// trim/rebuild stage exists.
    pub fn offset_outline(
        &self,
        distance: Real,
        cap: OffsetCap,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline_with_report(distance, cap, policy)
            .map(Self::outline_classification_from_report)
    }

    /// Builds a report-bearing checked closed outline around this open curve string.
    ///
    /// The report records cap style, source and intermediate segment counts,
    /// the furthest exact stage reached, and the exact blocker for unresolved
    /// or unsupported outline topology.
    pub fn offset_outline_with_report(
        &self,
        distance: Real,
        cap: OffsetCap,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringOutlineOffsetResult2> {
        checked_outline_with_report(self, distance, cap, policy)
    }

    /// Builds a report-bearing checked closed outline with round endpoint caps.
    pub fn offset_outline_round_caps_with_report(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringOutlineOffsetResult2> {
        self.offset_outline_with_report(distance, OffsetCap::Round, policy)
    }

    /// Builds a report-bearing checked closed outline with butt endpoint caps.
    pub fn offset_outline_butt_caps_with_report(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringOutlineOffsetResult2> {
        self.offset_outline_with_report(distance, OffsetCap::Butt, policy)
    }

    /// Builds a report-bearing checked closed outline with square endpoint caps.
    pub fn offset_outline_square_caps_with_report(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringOutlineOffsetResult2> {
        self.offset_outline_with_report(distance, OffsetCap::Square, policy)
    }

    fn outline_classification_from_report(
        result: CurveStringOutlineOffsetResult2,
    ) -> Classification<Contour2> {
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match result.into_outline() {
            Some(outline) => Classification::Decided(outline),
            None => Classification::Uncertain(blocker),
        }
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// The outline follows the left joined offset, adds a round cap at the end
    /// point, returns along the reversed right joined offset, and adds a round
    /// cap at the start point. The `distance` is the half-width of the outline
    /// and must be strictly positive under the active policy. As with
    /// [`CurveString2::offset_left_checked`], this is still a raw offset
    /// construction: if the input or resulting closed outline self-contacts,
    /// the method returns explicit uncertainty instead of trimming. Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles" (1984), describe this
    /// primitive offset, cap, and trim decomposition for open profile offsets.
    pub fn offset_outline_round_caps(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline_round_caps_with_report(distance, policy)
            .map(Self::outline_classification_from_report)
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// This variant connects the left and right offset traces with straight
    /// endpoint caps. Those cap lines are the radial/perpendicular endpoint
    /// connectors in the same primitive-offset, cap, and trim decomposition
    /// used for open profiles by Tiller and Hanson, "Offsets of
    /// Two-Dimensional Profiles" (1984). The distance is the half-width and
    /// must be strictly positive. As with round caps, this constructor rejects
    /// self-contacting input or output instead of trimming the raw outline.
    pub fn offset_outline_butt_caps(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline_butt_caps_with_report(distance, policy)
            .map(Self::outline_classification_from_report)
    }

    /// Builds a checked closed outline with square endpoint caps.
    ///
    /// Square caps extend both offset traces by one half-width along the source
    /// endpoint tangent before connecting them with a straight cap line. For
    /// line endpoints this can be folded into the endpoint offset segment; for
    /// arc endpoints it becomes an explicit tangent extension line so the
    /// circular offset arc remains exact. This is still the primitive
    /// offset/cap construction stage described by Tiller and Hanson, "Offsets
    /// of Two-Dimensional Profiles" (1984): self-contacting input or output is
    /// rejected as uncertainty until the trim/rebuild stage exists.
    pub fn offset_outline_square_caps(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline_square_caps_with_report(distance, policy)
            .map(Self::outline_classification_from_report)
    }
}

impl Contour2 {
    /// Returns a left offset of this closed contour with straight-line joins.
    ///
    /// Line-line corners are mitered at the exact supporting-line intersection
    /// whenever that relation can be classified. Joins that cannot be mitered
    /// are connected by a circular arc centered at the original shared vertex.
    /// The returned contour is checked for closure, but this method deliberately
    /// does not trim self-intersections or resolve collapsed regions; those
    /// operations belong to the later full offset pipeline described by Tiller
    /// and Hanson, "Offsets of Two-Dimensional Profiles" (1984).
    pub fn offset_left_with_line_joins(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }

        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(self.segments(), &offsets, true, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(checked_joined_contour(joined, self.fill_rule()))
    }

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections. It runs the joined offset
    /// construction and then classifies the result with
    /// [`Contour2::has_self_contacts`]. A detected self contact is reported as
    /// explicit uncertainty so callers do not mistake an untrimmed raw offset
    /// for a regularized contour. This matches Farouki and Neff's offset-curve
    /// treatment of self-intersections and extraneous loops as a separate
    /// trimming stage, not a property of the primitive parallel curve itself.
    pub fn offset_left_checked(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let result = self.offset_left_checked_with_report(distance, policy)?;
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(contour) = result.into_contour() {
            Ok(Classification::Decided(contour))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Returns a report-bearing raw joined left offset, rejecting self-contacting output.
    ///
    /// The report records source inventory, the raw joined offset segment count
    /// before self-contact validation, the final output count when accepted,
    /// the retained fill rule, and the exact blocker otherwise.
    pub fn offset_left_checked_with_report(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourOffsetResult2> {
        let source_segment_count = self.len();
        let source_segment_kind_counts = segment_kind_counts(self.segments());
        let fill_rule = self.fill_rule();
        let offset = match self.offset_left_with_line_joins(distance, policy)? {
            Classification::Decided(offset) => offset,
            Classification::Uncertain(reason) => {
                return Ok(blocked_contour_offset_result(
                    ContourOffsetStage2::OffsetConstruction,
                    source_segment_count,
                    source_segment_kind_counts,
                    None,
                    None,
                    None,
                    fill_rule,
                    retained_status_for_offset_blocker(reason),
                    reason,
                ));
            }
        };
        let raw_offset_segment_count = offset.len();
        let raw_offset_segment_kind_counts = segment_kind_counts(offset.segments());

        let self_contact = offset.has_self_contacts_with_report(policy)?;
        match self_contact.has_self_contacts() {
            Classification::Decided(false) => Ok(ContourOffsetResult2 {
                contour: Some(offset),
                report: ContourOffsetReport2 {
                    stage: ContourOffsetStage2::SelfContactValidation,
                    source_segment_count,
                    source_segment_kind_counts,
                    raw_offset_segment_count: Some(raw_offset_segment_count),
                    raw_offset_segment_kind_counts: Some(raw_offset_segment_kind_counts),
                    self_contact_report: Some(self_contact.report().clone()),
                    output_segment_count: Some(raw_offset_segment_count),
                    output_segment_kind_counts: Some(raw_offset_segment_kind_counts),
                    fill_rule,
                    status: RetainedTopologyStatus::NativeExact,
                    blocker: None,
                },
            }),
            Classification::Decided(true) => Ok(blocked_contour_offset_result(
                ContourOffsetStage2::SelfContactValidation,
                source_segment_count,
                source_segment_kind_counts,
                Some(raw_offset_segment_count),
                Some(raw_offset_segment_kind_counts),
                Some(self_contact.report().clone()),
                fill_rule,
                RetainedTopologyStatus::Unsupported,
                UncertaintyReason::Unsupported,
            )),
            Classification::Uncertain(reason) => Ok(blocked_contour_offset_result(
                ContourOffsetStage2::SelfContactValidation,
                source_segment_count,
                source_segment_kind_counts,
                Some(raw_offset_segment_count),
                Some(raw_offset_segment_kind_counts),
                Some(self_contact.report().clone()),
                fill_rule,
                retained_status_for_offset_blocker(reason),
                reason,
            )),
        }
    }
}

impl ContourOffsetReport2 {
    /// Returns the furthest exact checked-offset stage reached.
    pub const fn stage(&self) -> ContourOffsetStage2 {
        self.stage
    }

    /// Returns the source contour segment count.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source contour.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the raw joined offset segment count before self-contact rejection.
    pub const fn raw_offset_segment_count(&self) -> Option<usize> {
        self.raw_offset_segment_count
    }

    /// Returns primitive-family counts for the raw joined contour offset.
    pub const fn raw_offset_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.raw_offset_segment_kind_counts
    }

    /// Returns self-contact scan evidence when validation was reached.
    pub const fn self_contact_report(&self) -> Option<&SelfContactReport2> {
        self.self_contact_report.as_ref()
    }

    /// Returns output segment count when the checked offset materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the checked materialized contour offset.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns the fill rule retained from the source contour.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns checked offset topology status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized checked offsets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourOffsetResult2 {
    /// Returns the materialized checked offset, if accepted.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized checked offset.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Returns retained checked-offset evidence.
    pub const fn report(&self) -> &ContourOffsetReport2 {
        &self.report
    }
}

impl CurveStringOutlineOffsetReport2 {
    /// Returns the furthest exact outline-offset stage reached.
    pub const fn stage(&self) -> CurveStringOutlineOffsetStage2 {
        self.stage
    }

    /// Returns the endpoint cap style used by this outline construction.
    pub const fn cap(&self) -> OffsetCap {
        self.cap
    }

    /// Returns the source open curve-string segment count.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source open curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the left offset trace segment count after raw joining.
    pub const fn left_offset_segment_count(&self) -> Option<usize> {
        self.left_offset_segment_count
    }

    /// Returns primitive-family counts for the left offset trace.
    pub const fn left_offset_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.left_offset_segment_kind_counts
    }

    /// Returns the right offset trace segment count after raw joining.
    pub const fn right_offset_segment_count(&self) -> Option<usize> {
        self.right_offset_segment_count
    }

    /// Returns primitive-family counts for the right offset trace.
    pub const fn right_offset_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.right_offset_segment_kind_counts
    }

    /// Returns the closed outline segment count before final topology rejection.
    pub const fn outline_segment_count(&self) -> Option<usize> {
        self.outline_segment_count
    }

    /// Returns primitive-family counts for the closed outline before final topology rejection.
    pub const fn outline_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.outline_segment_kind_counts
    }

    /// Returns checked outline topology status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized outline offsets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringOutlineOffsetResult2 {
    /// Returns the materialized checked outline, if accepted.
    pub const fn outline(&self) -> Option<&Contour2> {
        self.outline.as_ref()
    }

    /// Consumes this result and returns the materialized checked outline.
    pub fn into_outline(self) -> Option<Contour2> {
        self.outline
    }

    /// Returns retained checked-outline evidence.
    pub const fn report(&self) -> &CurveStringOutlineOffsetReport2 {
        &self.report
    }
}

impl CurveStringOffsetReport2 {
    /// Returns the furthest exact checked-offset stage reached.
    pub const fn stage(&self) -> CurveStringOffsetStage2 {
        self.stage
    }

    /// Returns the source curve-string segment count.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the raw joined offset segment count before self-contact rejection.
    pub const fn raw_offset_segment_count(&self) -> Option<usize> {
        self.raw_offset_segment_count
    }

    /// Returns primitive-family counts for the raw joined offset.
    pub const fn raw_offset_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.raw_offset_segment_kind_counts
    }

    /// Returns self-contact scan evidence when validation was reached.
    pub const fn self_contact_report(&self) -> Option<&SelfContactReport2> {
        self.self_contact_report.as_ref()
    }

    /// Returns output segment count when the checked offset materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the checked materialized offset.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns checked offset topology status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized checked offsets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringOffsetResult2 {
    /// Returns the materialized checked offset, if accepted.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized checked offset.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns retained checked-offset evidence.
    pub const fn report(&self) -> &CurveStringOffsetReport2 {
        &self.report
    }
}

fn segment_kind_counts(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        match segment {
            Segment2::Line(_) => counts.lines += 1,
            Segment2::Arc(_) => counts.arcs += 1,
        }
    }
    counts
}

fn blocked_curve_string_offset_result(
    stage: CurveStringOffsetStage2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    raw_offset_segment_count: Option<usize>,
    raw_offset_segment_kind_counts: Option<SegmentKindCounts>,
    self_contact_report: Option<SelfContactReport2>,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> CurveStringOffsetResult2 {
    CurveStringOffsetResult2 {
        curve_string: None,
        report: CurveStringOffsetReport2 {
            stage,
            source_segment_count,
            source_segment_kind_counts,
            raw_offset_segment_count,
            raw_offset_segment_kind_counts,
            self_contact_report,
            output_segment_count: None,
            output_segment_kind_counts: None,
            status,
            blocker: Some(blocker),
        },
    }
}

fn blocked_contour_offset_result(
    stage: ContourOffsetStage2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    raw_offset_segment_count: Option<usize>,
    raw_offset_segment_kind_counts: Option<SegmentKindCounts>,
    self_contact_report: Option<SelfContactReport2>,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> ContourOffsetResult2 {
    ContourOffsetResult2 {
        contour: None,
        report: ContourOffsetReport2 {
            stage,
            source_segment_count,
            source_segment_kind_counts,
            raw_offset_segment_count,
            raw_offset_segment_kind_counts,
            self_contact_report,
            output_segment_count: None,
            output_segment_kind_counts: None,
            fill_rule,
            status,
            blocker: Some(blocker),
        },
    }
}

fn blocked_curve_string_outline_offset_result(
    stage: CurveStringOutlineOffsetStage2,
    cap: OffsetCap,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    left_offset_segment_count: Option<usize>,
    left_offset_segment_kind_counts: Option<SegmentKindCounts>,
    right_offset_segment_count: Option<usize>,
    right_offset_segment_kind_counts: Option<SegmentKindCounts>,
    outline_segment_count: Option<usize>,
    outline_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> CurveStringOutlineOffsetResult2 {
    CurveStringOutlineOffsetResult2 {
        outline: None,
        report: CurveStringOutlineOffsetReport2 {
            stage,
            cap,
            source_segment_count,
            source_segment_kind_counts,
            left_offset_segment_count,
            left_offset_segment_kind_counts,
            right_offset_segment_count,
            right_offset_segment_kind_counts,
            outline_segment_count,
            outline_segment_kind_counts,
            status,
            blocker: Some(blocker),
        },
    }
}

const fn retained_status_for_offset_blocker(reason: UncertaintyReason) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Unsupported | UncertaintyReason::Boundary => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn checked_outline_with_report(
    source: &CurveString2,
    distance: Real,
    cap: OffsetCap,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringOutlineOffsetResult2> {
    let source_segment_count = source.len();
    let source_segment_kind_counts = segment_kind_counts(source.segments());
    match real_sign(&distance, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::DistanceValidation,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                None,
                None,
                None,
                None,
                None,
                None,
                RetainedTopologyStatus::Unsupported,
                UncertaintyReason::Unsupported,
            ));
        }
        None => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::DistanceValidation,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                None,
                None,
                None,
                None,
                None,
                None,
                RetainedTopologyStatus::Unresolved,
                UncertaintyReason::RealSign,
            ));
        }
    }

    match source.has_self_contacts(policy)? {
        Classification::Decided(false) => {}
        Classification::Decided(true) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::SourceSelfContactValidation,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                None,
                None,
                None,
                None,
                None,
                None,
                RetainedTopologyStatus::Unsupported,
                UncertaintyReason::Unsupported,
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::SourceSelfContactValidation,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                None,
                None,
                None,
                None,
                None,
                None,
                retained_status_for_offset_blocker(reason),
                reason,
            ));
        }
    }

    let left = match source.offset_left_with_line_joins(distance.clone(), policy)? {
        Classification::Decided(left) => left,
        Classification::Uncertain(reason) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::LeftOffsetConstruction,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                None,
                None,
                None,
                None,
                None,
                None,
                retained_status_for_offset_blocker(reason),
                reason,
            ));
        }
    };
    let left_offset_segment_count = left.len();
    let left_offset_segment_kind_counts = segment_kind_counts(left.segments());

    let right = match source.offset_left_with_line_joins(-distance.clone(), policy)? {
        Classification::Decided(right) => right,
        Classification::Uncertain(reason) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::RightOffsetConstruction,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                Some(left_offset_segment_count),
                Some(left_offset_segment_kind_counts),
                None,
                None,
                None,
                None,
                retained_status_for_offset_blocker(reason),
                reason,
            ));
        }
    };
    let right_offset_segment_count = right.len();
    let right_offset_segment_kind_counts = segment_kind_counts(right.segments());

    let offsets = OutlineOffsets {
        start_center: source.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        end_center: source.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_start: left.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_end: left.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_start: right.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_end: right.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left,
        right,
    };

    let segments = match outline_segments_for_cap(source, offsets, distance, cap, policy)? {
        Classification::Decided(segments) => segments,
        Classification::Uncertain(reason) => {
            return Ok(blocked_curve_string_outline_offset_result(
                CurveStringOutlineOffsetStage2::CapConstruction,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                Some(left_offset_segment_count),
                Some(left_offset_segment_kind_counts),
                Some(right_offset_segment_count),
                Some(right_offset_segment_kind_counts),
                None,
                None,
                retained_status_for_offset_blocker(reason),
                reason,
            ));
        }
    };
    let outline_segment_count = segments.len();
    let outline_segment_kind_counts = segment_kind_counts(&segments);

    match checked_outline_contour(segments, policy)? {
        Classification::Decided(outline) => Ok(CurveStringOutlineOffsetResult2 {
            outline: Some(outline),
            report: CurveStringOutlineOffsetReport2 {
                stage: CurveStringOutlineOffsetStage2::OutlineTopologyValidation,
                cap,
                source_segment_count,
                source_segment_kind_counts,
                left_offset_segment_count: Some(left_offset_segment_count),
                left_offset_segment_kind_counts: Some(left_offset_segment_kind_counts),
                right_offset_segment_count: Some(right_offset_segment_count),
                right_offset_segment_kind_counts: Some(right_offset_segment_kind_counts),
                outline_segment_count: Some(outline_segment_count),
                outline_segment_kind_counts: Some(outline_segment_kind_counts),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        }),
        Classification::Uncertain(reason) => Ok(blocked_curve_string_outline_offset_result(
            CurveStringOutlineOffsetStage2::OutlineTopologyValidation,
            cap,
            source_segment_count,
            source_segment_kind_counts,
            Some(left_offset_segment_count),
            Some(left_offset_segment_kind_counts),
            Some(right_offset_segment_count),
            Some(right_offset_segment_kind_counts),
            Some(outline_segment_count),
            Some(outline_segment_kind_counts),
            retained_status_for_offset_blocker(reason),
            reason,
        )),
    }
}

fn outline_segments_for_cap(
    source: &CurveString2,
    offsets: OutlineOffsets,
    distance: Real,
    cap: OffsetCap,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2>>> {
    match cap {
        OffsetCap::Round => outline_segments_with_round_caps(offsets, policy),
        OffsetCap::Butt => outline_segments_with_butt_caps(offsets),
        OffsetCap::Square => outline_segments_with_square_caps(source, offsets, distance),
    }
}

fn outline_segments_with_round_caps(
    offsets: OutlineOffsets,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2>>> {
    let OutlineOffsets {
        left,
        right,
        start_center,
        end_center,
        left_start,
        left_end,
        right_start,
        right_end,
    } = offsets;

    let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
    segments.extend(left.into_segments());
    match round_cap_arc(&left_end, &right_end, &end_center, policy)? {
        Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    segments.extend(reversed_segments(right.into_segments()));
    match round_cap_arc(&right_start, &left_start, &start_center, policy)? {
        Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(segments))
}

fn outline_segments_with_butt_caps(
    offsets: OutlineOffsets,
) -> CurveResult<Classification<Vec<Segment2>>> {
    let OutlineOffsets {
        left,
        right,
        left_start,
        left_end,
        right_start,
        right_end,
        ..
    } = offsets;

    let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
    segments.extend(left.into_segments());
    match cap_line(&left_end, &right_end)? {
        Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    segments.extend(reversed_segments(right.into_segments()));
    match cap_line(&right_start, &left_start)? {
        Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(segments))
}

fn outline_segments_with_square_caps(
    source: &CurveString2,
    offsets: OutlineOffsets,
    distance: Real,
) -> CurveResult<Classification<Vec<Segment2>>> {
    let OutlineOffsets {
        left,
        right,
        left_start,
        left_end,
        right_start,
        right_end,
        ..
    } = offsets;

    let start_tangent = unit_tangent_at_segment_start(
        source
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?,
    )?;
    let end_tangent = unit_tangent_at_segment_end(
        source
            .segments()
            .last()
            .ok_or(CurveError::EmptyCurveString)?,
    )?;
    let start_dx = &start_tangent.0 * &distance;
    let start_dy = &start_tangent.1 * &distance;
    let end_dx = &end_tangent.0 * &distance;
    let end_dy = &end_tangent.1 * &distance;

    let left_start_square = left_start.translated(-start_dx.clone(), -start_dy.clone());
    let right_start_square = right_start.translated(-start_dx, -start_dy);
    let left_end_square = left_end.translated(end_dx.clone(), end_dy.clone());
    let right_end_square = right_end.translated(end_dx, end_dy);

    let left = match extend_square_cap_trace(
        left.into_segments(),
        left_start_square.clone(),
        left_end_square.clone(),
    )? {
        Classification::Decided(left) => left,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let right = match extend_square_cap_trace(
        right.into_segments(),
        right_start_square.clone(),
        right_end_square.clone(),
    )? {
        Classification::Decided(right) => right,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
    segments.extend(left);
    match cap_line(&left_end_square, &right_end_square)? {
        Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    segments.extend(reversed_segments(right));
    match cap_line(&right_start_square, &left_start_square)? {
        Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(segments))
}

fn scale_from_center(point: &Point2, center: &Point2, scale: &Real) -> Point2 {
    let radius = point.delta_from(center);
    Point2::new(
        center.x() + (&radius.0 * scale),
        center.y() + (&radius.1 * scale),
    )
}

struct OutlineOffsets {
    left: CurveString2,
    right: CurveString2,
    start_center: Point2,
    end_center: Point2,
    left_start: Point2,
    left_end: Point2,
    right_start: Point2,
    right_end: Point2,
}

// Contour closure and self-contact checks are deliberately centralized so every
// open-outline cap style preserves the same "raw construction, no trimming"
// contract described by Tiller and Hanson (1984).
fn checked_outline_contour(
    segments: Vec<Segment2>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Contour2>> {
    let outline = match Contour2::try_new(segments) {
        Ok(outline) => outline,
        Err(CurveError::DisconnectedCurveString) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Err(CurveError::AmbiguousCurveStringConnection) => {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        Err(error) => return Err(error),
    };
    match outline.has_self_contacts(policy)? {
        Classification::Decided(false) => Ok(Classification::Decided(outline)),
        Classification::Decided(true) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn checked_joined_curve_string(segments: Vec<Segment2>) -> Classification<CurveString2> {
    CurveString2::try_new(segments)
        .map(Classification::Decided)
        .unwrap_or_else(classify_joined_topology_error)
}

fn checked_joined_contour(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
) -> Classification<Contour2> {
    Contour2::try_new_with_fill_rule(segments, fill_rule)
        .map(Classification::Decided)
        .unwrap_or_else(classify_joined_topology_error)
}

fn classify_joined_topology_error<T>(error: CurveError) -> Classification<T> {
    match error {
        CurveError::DisconnectedCurveString => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        CurveError::AmbiguousCurveStringConnection => {
            Classification::Uncertain(UncertaintyReason::RealSign)
        }
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn extend_square_cap_trace(
    mut segments: Vec<Segment2>,
    extended_start: Point2,
    extended_end: Point2,
) -> CurveResult<Classification<Vec<Segment2>>> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let original_start = segments[0].start().clone();
    match &segments[0] {
        Segment2::Line(line) => {
            segments[0] = match cap_line(&extended_start, line.end())? {
                Classification::Decided(line) => Segment2::Line(line),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }
        Segment2::Arc(_) => match cap_line(&extended_start, &original_start)? {
            Classification::Decided(line) => segments.insert(0, Segment2::Line(line)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
    }

    let last_index = segments.len() - 1;
    let original_end = segments[last_index].end().clone();
    match &segments[last_index] {
        Segment2::Line(line) => {
            segments[last_index] = match cap_line(line.start(), &extended_end)? {
                Classification::Decided(line) => Segment2::Line(line),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }
        Segment2::Arc(_) => match cap_line(&original_end, &extended_end)? {
            Classification::Decided(line) => segments.push(Segment2::Line(line)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
    }

    Ok(Classification::Decided(segments))
}

fn unit_tangent_at_segment_start(segment: &Segment2) -> CurveResult<(Real, Real)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.start()),
    }
}

fn unit_tangent_at_segment_end(segment: &Segment2) -> CurveResult<(Real, Real)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.end()),
    }
}

fn unit_tangent_for_line(line: &LineSeg2) -> CurveResult<(Real, Real)> {
    let length = line.length_squared().sqrt()?;
    let (dx, dy) = line.delta();
    Ok(((dx / &length)?, (dy / &length)?))
}

fn unit_tangent_for_arc_at_point(arc: &CircularArc2, point: &Point2) -> CurveResult<(Real, Real)> {
    let radius = arc.radius_squared().sqrt()?;
    let (rx, ry) = point.delta_from(arc.center());
    if arc.is_clockwise() {
        Ok(((ry / &radius)?, ((-rx) / &radius)?))
    } else {
        Ok(((-ry / &radius)?, (rx / &radius)?))
    }
}

fn offset_segments_left(
    segments: &[Segment2],
    distance: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2>>> {
    let mut offsets = Vec::with_capacity(segments.len());
    for segment in segments {
        match segment.offset_left(distance.clone(), policy)? {
            Classification::Decided(offset) => offsets.push(offset),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(offsets))
}

#[derive(Clone, Debug, PartialEq)]
enum OffsetJoin {
    Miter(Point2),
    Round { center: Point2 },
}

fn joined_offset_segments(
    source: &[Segment2],
    offsets: &[Segment2],
    closed: bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2>>> {
    if offsets.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }
    if source.len() != offsets.len() {
        return Err(CurveError::Topology(
            "source and offset segment counts differ".into(),
        ));
    }

    let join_count = if closed {
        offsets.len()
    } else {
        offsets.len().saturating_sub(1)
    };
    let mut joins = Vec::with_capacity(join_count);
    for index in 0..join_count {
        let next_index = (index + 1) % offsets.len();
        match classify_offset_join(
            &source[index],
            &source[next_index],
            &offsets[index],
            &offsets[next_index],
            policy,
        )? {
            Classification::Decided(join) => joins.push(join),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let mut joined = Vec::with_capacity(offsets.len() + join_count);
    for index in 0..offsets.len() {
        let start_override = start_miter_for_segment(index, offsets.len(), closed, &joins);
        let end_override = end_miter_for_segment(index, &joins);
        let adjusted = match adjust_offset_segment(
            &offsets[index],
            start_override.as_ref(),
            end_override.as_ref(),
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let adjusted_end = adjusted.end().clone();
        joined.push(adjusted);

        if let Some(OffsetJoin::Round { center }) = joins.get(index) {
            let to = offsets[(index + 1) % offsets.len()].start().clone();
            match append_round_join_if_needed(&mut joined, &adjusted_end, &to, center, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(joined))
}

fn classify_offset_join(
    source_previous: &Segment2,
    source_next: &Segment2,
    offset_previous: &Segment2,
    offset_next: &Segment2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<OffsetJoin>> {
    match (offset_previous, offset_next) {
        (Segment2::Line(previous), Segment2::Line(next)) => {
            match line_support_intersection(previous, next, policy)? {
                Classification::Decided(Some(point)) => {
                    Ok(Classification::Decided(OffsetJoin::Miter(point)))
                }
                Classification::Decided(None) => Ok(Classification::Decided(round_join(
                    source_previous,
                    source_next,
                ))),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        _ => Ok(Classification::Decided(round_join(
            source_previous,
            source_next,
        ))),
    }
}

fn round_join(previous: &Segment2, next: &Segment2) -> OffsetJoin {
    let _ = next;
    OffsetJoin::Round {
        center: previous.end().clone(),
    }
}

fn line_support_intersection(
    previous: &LineSeg2,
    next: &LineSeg2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Point2>>> {
    let (rx, ry) = previous.delta();
    let (sx, sy) = next.delta();
    let denominator = cross(&rx, &ry, &sx, &sy);

    match real_sign(&denominator, policy) {
        Some(RealSign::Zero) => Ok(Classification::Decided(None)),
        Some(RealSign::Positive | RealSign::Negative) => {
            let qmp = next.start().delta_from(previous.start());
            let numerator = cross(&qmp.0, &qmp.1, &sx, &sy);
            let t = (numerator / &denominator)?;
            Ok(Classification::Decided(Some(previous.point_at(t))))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn start_miter_for_segment(
    index: usize,
    segment_count: usize,
    closed: bool,
    joins: &[OffsetJoin],
) -> Option<Point2> {
    if !closed && index == 0 {
        return None;
    }
    let join_index = if index == 0 {
        segment_count - 1
    } else {
        index - 1
    };
    match joins.get(join_index) {
        Some(OffsetJoin::Miter(point)) => Some(point.clone()),
        _ => None,
    }
}

fn end_miter_for_segment(index: usize, joins: &[OffsetJoin]) -> Option<Point2> {
    match joins.get(index) {
        Some(OffsetJoin::Miter(point)) => Some(point.clone()),
        _ => None,
    }
}

fn adjust_offset_segment(
    segment: &Segment2,
    start_override: Option<&Point2>,
    end_override: Option<&Point2>,
) -> CurveResult<Classification<Segment2>> {
    match segment {
        Segment2::Line(line) => {
            let start = start_override
                .cloned()
                .unwrap_or_else(|| line.start().clone());
            let end = end_override.cloned().unwrap_or_else(|| line.end().clone());
            match LineSeg2::try_new(start, end) {
                Ok(line) => Ok(Classification::Decided(Segment2::Line(line))),
                Err(CurveError::ZeroLengthLine) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Err(error) => Err(error),
            }
        }
        Segment2::Arc(_) if start_override.is_some() || end_override.is_some() => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Segment2::Arc(arc) => Ok(Classification::Decided(Segment2::Arc(arc.clone()))),
    }
}

fn append_round_join_if_needed(
    joined: &mut Vec<Segment2>,
    from: &Point2,
    to: &Point2,
    center: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    let distance = from.distance_squared(to);
    match is_zero(&distance, policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match round_join_arc(from, to, center, clockwise) {
                Classification::Decided(arc) => {
                    joined.push(Segment2::Arc(arc));
                    Ok(Classification::Decided(()))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn round_cap_arc(
    from: &Point2,
    to: &Point2,
    center: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<CircularArc2>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            Ok(round_join_arc(from, to, center, clockwise))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn round_join_arc(
    from: &Point2,
    to: &Point2,
    center: &Point2,
    clockwise: bool,
) -> Classification<CircularArc2> {
    match CircularArc2::try_from_center(from.clone(), to.clone(), center.clone(), clockwise) {
        Ok(arc) => Classification::Decided(arc),
        // A round join is only valid when both offset endpoints are certified
        // to lie on the circle around the source vertex. If exact radii differ,
        // the primitive join stage has reached the unsupported trim/rebuild
        // boundary described by Tiller-Hanson and Farouki-Neff above.
        Err(CurveError::ZeroRadiusArc | CurveError::RadiusMismatch) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn cap_line(from: &Point2, to: &Point2) -> CurveResult<Classification<LineSeg2>> {
    match LineSeg2::try_new(from.clone(), to.clone()) {
        Ok(line) => Ok(Classification::Decided(line)),
        Err(CurveError::ZeroLengthLine) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Err(error) => Err(error),
    }
}

fn reversed_segments(segments: Vec<Segment2>) -> impl Iterator<Item = Segment2> {
    segments.into_iter().rev().map(|segment| segment.reversed())
}

fn round_join_clockwise(
    center: &Point2,
    from: &Point2,
    to: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let from_radius = from.delta_from(center);
    let to_radius = to.delta_from(center);
    let turn = cross(&from_radius.0, &from_radius.1, &to_radius.0, &to_radius.1);

    match real_sign(&turn, policy) {
        Some(RealSign::Positive) => Classification::Decided(false),
        Some(RealSign::Negative) => Classification::Decided(true),
        Some(RealSign::Zero) => Classification::Decided(true),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn cross(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * by) - (ay * bx)
}
