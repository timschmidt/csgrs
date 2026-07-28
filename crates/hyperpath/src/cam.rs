//! Exact CAM path and geometry reports.
//!
//! Subtractive CAM commonly generates contour-parallel pocket passes by
//! repeatedly offsetting a source boundary and cleaning the resulting
//! arrangements. This module returns exact rectangular rings and beads
//! immediately, and constructs link graphs directly from source geometry and
//! process parameters. It follows Yap, "Towards Exact Geometric Computation,"
//! by retaining exact candidate objects without implying they are accepted
//! output before arrangement, gouge, and process predicates certify them.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts, RealSign};

mod pocket_link;
mod rest;

pub use pocket_link::{
    PocketLinkGraphError, PocketLinkSegment, PocketRingSegment, PocketRingSide,
    RectangularPocketLinkGraph, rectangular_pocket_link_graph,
};
pub use rest::{
    RectangularRestCutRecord, RectangularRestMaterialError, RectangularRestMaterialGraph,
    RectangularRestMaterialStage, rectangular_rest_material_graph,
};

/// Exact axis-aligned rectangular pocket boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularPocket {
    min: Point2,
    max: Point2,
    exact: RealExactSetFacts,
}

/// One scheduled contour-parallel rectangular pocket ring.
///
/// The ring is an exact planning record, not a machined output contour. A later
/// arrangement stage still has to certify loop validity, linking, gouge
/// absence, and rest-material interaction.
#[derive(Clone, Debug, PartialEq)]
pub struct PocketOffsetRing {
    /// Zero-based ring index.
    pub index: usize,
    /// Exact inset from the source pocket boundary.
    pub inset: Real,
    /// Exact minimum corner of the inset rectangle.
    pub min: Point2,
    /// Exact maximum corner of the inset rectangle.
    pub max: Point2,
}

/// Reason rectangular ring or bead generation stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RectangularScheduleStop {
    /// The requested item limit was reached before geometric exhaustion.
    LimitReached,
    /// The next ring or bead would exceed the rectangular region.
    GeometryExhausted,
    /// Exact comparison could not certify whether the next item is valid.
    Unknown,
}

/// Immediate contour-parallel pocket-ring result.
#[derive(Clone, Debug, PartialEq)]
pub struct PocketRingReport {
    /// Rings in construction order.
    pub rings: Vec<PocketOffsetRing>,
    /// Why ring generation stopped.
    pub stop: RectangularScheduleStop,
}

/// Errors while constructing exact rectangular pockets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RectangularPocketError {
    /// Pocket bounds were not exactly ordered.
    UnorderedBounds,
}

/// Errors while generating exact rectangular pocket rings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PocketRingError {
    /// Tool radius was structurally negative.
    NegativeToolRadius,
    /// Stepover was not certified strictly positive.
    NonPositiveStepover,
    /// No rings were requested.
    ZeroMaxRings,
}

/// Fill direction for exact rectangular additive bead schedules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BeadFillAxis {
    /// Beads run horizontally; pitch advances in Y.
    Horizontal,
    /// Beads run vertically; pitch advances in X.
    Vertical,
}

/// One exact additive bead centerline inside a rectangular region.
///
/// This is a deposition schedule primitive, not a complete additive process
/// plan. Later region set algebra, bead overlap policy, starts/stops,
/// supports, and thermal/process constraints still need exact predicates
/// before output.
#[derive(Clone, Debug, PartialEq)]
pub struct AdditiveBeadLine {
    /// Zero-based bead index.
    pub index: usize,
    /// Exact centerline segment.
    pub segment: crate::segment::LinePathSegment,
    /// Exact bead center coordinate on the pitch axis.
    pub pitch_position: Real,
}

/// Immediate rectangular additive-bead result.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularBeadReport {
    /// Bead centerlines in construction order.
    pub beads: Vec<AdditiveBeadLine>,
    /// Why bead generation stopped.
    pub stop: RectangularScheduleStop,
}

/// Errors while generating exact additive beads.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RectangularBeadError {
    /// Bead width was not certified strictly positive.
    NonPositiveBeadWidth,
    /// Spacing was not certified strictly positive.
    NonPositiveSpacing,
    /// No beads were requested.
    ZeroMaxBeads,
}

/// One exact connector between adjacent additive bead centerlines.
///
/// The connector is a graph edge, not automatically an accepted extrusion or
/// travel move. The split follows Yap's exact-computation boundary and the
/// continuous-additive-toolpath graph literature, e.g. Zhao et al.,
/// "Continuous toolpath planning in a graphical framework for sparse infill
/// additive manufacturing": a path graph can be generated first, but exact
/// geometry and process predicates still decide whether each edge is usable.
#[derive(Clone, Debug, PartialEq)]
pub struct AdditiveInfillLink {
    /// Index of the bead whose traversal ends at the connector start.
    pub from_bead: usize,
    /// Index of the bead whose traversal starts at the connector end.
    pub to_bead: usize,
    /// Exact connector segment between the two bead traversal endpoints.
    pub connector: crate::segment::LinePathSegment,
}

/// Exact serpentine graph over a rectangular additive bead schedule.
///
/// `deposition_segments` are the bead centerlines in traversal order. Odd
/// beads are reversed so every connector joins the previous deposition end to
/// the next deposition start exactly. This is intentionally a topology carrier:
/// starts/stops, pressure advance, wipe/coast moves, support interaction, and
/// thermal constraints are later certifications.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularInfillGraph {
    /// Source rectangular region.
    pub region: RectangularPocket,
    /// Fill direction.
    pub axis: BeadFillAxis,
    /// Exact bead width.
    pub bead_width: Real,
    /// Exact centerline pitch.
    pub spacing: Real,
    /// Generated bead centerlines.
    pub beads: Vec<AdditiveBeadLine>,
    /// Why bead generation stopped.
    pub stop: RectangularScheduleStop,
    /// Bead centerlines oriented in serpentine traversal order.
    pub deposition_segments: Vec<crate::segment::LinePathSegment>,
    /// Exact connector edges between adjacent deposition segments.
    pub links: Vec<AdditiveInfillLink>,
}

/// Errors while constructing exact additive infill graphs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InfillGraphError {
    /// Bead generation failed.
    Beads(RectangularBeadError),
    /// No bead centerlines were available to graph.
    EmptyBeads,
    /// A generated connector endpoint failed exact equality validation.
    InvalidConnectorEndpoint,
}

/// Exact support-footprint containment status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportFootprintStatus {
    /// The expanded support footprint is exactly contained in the base region.
    ContainedInBase,
    /// At least one support-footprint side lies outside the base region.
    OutsideBase,
    /// Exact comparison could not certify containment under the policy.
    Unknown,
}

/// Exact rectangular additive support-footprint report.
///
/// Support generation is usually implemented as an image/slice heuristic. This
/// carrier keeps the construction in Yap's exact object layer: derive a
/// footprint, retain the source overhang/base rectangles, and expose a
/// predicate result before a downstream process planner accepts support moves.
/// The staged treatment mirrors additive slicing/support surveys such as
/// Kulkarni, Marsan, and Dutta, "A review of process planning techniques in
/// layered manufacturing", while avoiding tolerance-only geometry decisions.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularSupportReport {
    /// Overhang region that requested support.
    pub overhang: RectangularPocket,
    /// Base/build envelope used to validate the support footprint.
    pub base: RectangularPocket,
    /// Exact XY expansion margin around the overhang.
    pub xy_margin: Real,
    /// Expanded support footprint.
    pub footprint: RectangularPocket,
    /// Exact containment classification against `base`.
    pub status: SupportFootprintStatus,
}

/// Errors while constructing exact rectangular support footprints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportFootprintError {
    /// XY margin was structurally negative.
    NegativeMargin,
    /// Expanded support footprint bounds were not exactly ordered.
    InvalidFootprint,
}

/// Exact relation between two rectangular regions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RectangularRegionRelation {
    /// The rectangles have no common point.
    Disjoint,
    /// The rectangles meet at an edge or point, but have no shared area.
    Touching,
    /// The rectangles overlap with positive area.
    AreaOverlap,
}

/// Exact closed intersection of two rectangular regions.
///
/// This is a retained rectangular set-algebra carrier for additive clipping
/// and support/infill planning. It does not materialize mesh topology or run a
/// solid boolean. Exact rectangle/rectangle operations are a useful Yap-style
/// primitive: construct the candidate region, classify it by exact
/// comparisons, and keep the predicate result visible to downstream callers.
/// This mirrors CGAL arrangement practice where topology decisions are
/// explicit predicates rather than tolerance side effects.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRegionIntersection {
    /// First input region.
    pub first: RectangularPocket,
    /// Second input region.
    pub second: RectangularPocket,
    /// Closed intersection rectangle when the inputs touch or overlap.
    pub intersection: Option<RectangularPocket>,
    /// Certified relation between the inputs.
    pub relation: RectangularRegionRelation,
}

/// Exact rectangular subtraction planning record.
///
/// `remainder` contains positive-area rectangles covering `subject - cutter`
/// for axis-aligned rectangular inputs. The pieces are intentionally emitted as
/// a planning carrier rather than simplified into an arbitrary polygon; later
/// arrangement/linking stages can consume the pieces with their exact
/// relation status intact.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRegionDifference {
    /// Region being cut.
    pub subject: RectangularPocket,
    /// Region removed from `subject`.
    pub cutter: RectangularPocket,
    /// Exact intersection used for the subtraction, if any.
    pub intersection: Option<RectangularPocket>,
    /// Positive-area remainder rectangles.
    pub remainder: Vec<RectangularPocket>,
    /// Certified relation between subject and cutter.
    pub relation: RectangularRegionRelation,
}

/// Errors while constructing exact rectangular region set-algebra reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBooleanError {
    /// Exact comparison could not decide a required ordering.
    UnknownComparison,
    /// A generated rectangle failed bound validation.
    InvalidRegion,
}

impl RectangularPocket {
    /// Construct an exact rectangular pocket.
    pub fn new(min: Point2, max: Point2) -> Result<Self, RectangularPocketError> {
        if !ordered_closed(&min.x, &max.x) || !ordered_closed(&min.y, &max.y) {
            return Err(RectangularPocketError::UnorderedBounds);
        }
        let exact = Real::exact_set_facts([&min.x, &min.y, &max.x, &max.y]);
        Ok(Self { min, max, exact })
    }

    /// Return exact minimum corner.
    pub const fn min(&self) -> &Point2 {
        &self.min
    }

    /// Return exact maximum corner.
    pub const fn max(&self) -> &Point2 {
        &self.max
    }

    /// Return exact-set facts for pocket coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Return exact pocket width.
    pub fn width(&self) -> Real {
        self.max.x.clone() - self.min.x.clone()
    }

    /// Return exact pocket height.
    pub fn height(&self) -> Real {
        self.max.y.clone() - self.min.y.clone()
    }
}

/// Returns exact contour-parallel rings for a rectangular pocket.
///
/// The first ring is inset by `tool_radius`; each next ring is inset by one
/// additional `stepover`. The function stops before emitting a ring whose
/// bounds cannot be certified as ordered. This is only the pocket/rest graph
/// skeleton: no path linking, cutter engagement, corner cleanup, or rest
/// material decision is accepted here.
pub fn rectangular_pocket_rings(
    pocket: &RectangularPocket,
    tool_radius: Real,
    stepover: Real,
    max_rings: usize,
    policy: PredicatePolicy,
) -> Result<PocketRingReport, PocketRingError> {
    if max_rings == 0 {
        return Err(PocketRingError::ZeroMaxRings);
    }
    if tool_radius.structural_facts().sign == Some(RealSign::Negative) {
        return Err(PocketRingError::NegativeToolRadius);
    }
    if compare_reals_with_policy(&stepover, &Real::zero(), policy).value()
        != Some(Ordering::Greater)
    {
        return Err(PocketRingError::NonPositiveStepover);
    }

    let mut rings = Vec::new();
    let mut inset = tool_radius.clone();
    let stop_reason = loop {
        if rings.len() == max_rings {
            break RectangularScheduleStop::LimitReached;
        }
        let Some((min, max)) = inset_rect(pocket, &inset, policy) else {
            break RectangularScheduleStop::Unknown;
        };
        if min.is_none() {
            break RectangularScheduleStop::GeometryExhausted;
        }
        rings.push(PocketOffsetRing {
            index: rings.len(),
            inset: inset.clone(),
            min: min.unwrap(),
            max: max.unwrap(),
        });
        inset += stepover.clone();
    };

    Ok(PocketRingReport {
        rings,
        stop: stop_reason,
    })
}

/// Returns exact additive bead centerlines for a rectangular region.
///
/// The first bead centerline is inset by `bead_width / 2` from the low side of
/// the pitch axis, and later beads advance by `spacing`. This is the additive
/// analogue of the pocket-ring scheduler: it creates exact candidate
/// centerlines for infill/skin planning while leaving region set algebra,
/// supports, corner starts/stops, and process validation to downstream exact
/// predicates.
pub fn rectangular_beads(
    region: &RectangularPocket,
    axis: BeadFillAxis,
    bead_width: Real,
    spacing: Real,
    max_beads: usize,
    policy: PredicatePolicy,
) -> Result<RectangularBeadReport, RectangularBeadError> {
    if max_beads == 0 {
        return Err(RectangularBeadError::ZeroMaxBeads);
    }
    if compare_reals_with_policy(&bead_width, &Real::zero(), policy).value()
        != Some(Ordering::Greater)
    {
        return Err(RectangularBeadError::NonPositiveBeadWidth);
    }
    if compare_reals_with_policy(&spacing, &Real::zero(), policy).value() != Some(Ordering::Greater)
    {
        return Err(RectangularBeadError::NonPositiveSpacing);
    }
    let half_width = (bead_width.clone() / Real::from(2))
        .map_err(|_| RectangularBeadError::NonPositiveBeadWidth)?;
    let mut beads = Vec::new();
    let mut pitch_position = match axis {
        BeadFillAxis::Horizontal => region.min.y.clone() + half_width.clone(),
        BeadFillAxis::Vertical => region.min.x.clone() + half_width.clone(),
    };
    let stop_reason = loop {
        if beads.len() == max_beads {
            break RectangularScheduleStop::LimitReached;
        }
        let limit = match axis {
            BeadFillAxis::Horizontal => region.max.y.clone() - half_width.clone(),
            BeadFillAxis::Vertical => region.max.x.clone() - half_width.clone(),
        };
        let Some(ordering) = compare_reals_with_policy(&pitch_position, &limit, policy).value()
        else {
            break RectangularScheduleStop::Unknown;
        };
        if ordering == Ordering::Greater {
            break RectangularScheduleStop::GeometryExhausted;
        }

        let segment = match axis {
            BeadFillAxis::Horizontal => crate::segment::LinePathSegment::new(
                Point2::new(region.min.x.clone(), pitch_position.clone()),
                Point2::new(region.max.x.clone(), pitch_position.clone()),
            ),
            BeadFillAxis::Vertical => crate::segment::LinePathSegment::new(
                Point2::new(pitch_position.clone(), region.min.y.clone()),
                Point2::new(pitch_position.clone(), region.max.y.clone()),
            ),
        };
        beads.push(AdditiveBeadLine {
            index: beads.len(),
            segment,
            pitch_position: pitch_position.clone(),
        });
        pitch_position += spacing.clone();
    };

    Ok(RectangularBeadReport {
        beads,
        stop: stop_reason,
    })
}

/// Returns an exact serpentine infill graph for a rectangular region.
///
/// The graph alternates bead direction, then inserts exact straight connectors
/// between consecutive oriented bead centerlines. It validates every generated
/// connector endpoint with `hyperlimit` equality rather than relying on object
/// identity, preserving the Yap-style separation between construction and
/// predicate certification.
pub fn rectangular_serpentine_infill_graph(
    region: RectangularPocket,
    axis: BeadFillAxis,
    bead_width: Real,
    spacing: Real,
    max_beads: usize,
    policy: PredicatePolicy,
) -> Result<RectangularInfillGraph, InfillGraphError> {
    let report = rectangular_beads(
        &region,
        axis,
        bead_width.clone(),
        spacing.clone(),
        max_beads,
        policy,
    )
    .map_err(InfillGraphError::Beads)?;
    if report.beads.is_empty() {
        return Err(InfillGraphError::EmptyBeads);
    }

    let deposition_segments: Vec<_> = report
        .beads
        .iter()
        .enumerate()
        .map(|(index, bead)| {
            if index % 2 == 0 {
                bead.segment.clone()
            } else {
                crate::segment::LinePathSegment::new(
                    bead.segment.end().clone(),
                    bead.segment.start().clone(),
                )
            }
        })
        .collect();

    let mut links = Vec::new();
    for (index, pair) in deposition_segments.windows(2).enumerate() {
        let current = &pair[0];
        let next = &pair[1];
        let connector =
            crate::segment::LinePathSegment::new(current.end().clone(), next.start().clone());
        if !points_equal(current.end(), connector.start(), policy)
            || !points_equal(next.start(), connector.end(), policy)
        {
            return Err(InfillGraphError::InvalidConnectorEndpoint);
        }
        links.push(AdditiveInfillLink {
            from_bead: index,
            to_bead: index + 1,
            connector,
        });
    }

    Ok(RectangularInfillGraph {
        region,
        axis,
        bead_width,
        spacing,
        beads: report.beads,
        stop: report.stop,
        deposition_segments,
        links,
    })
}

/// Create and classify an exact rectangular support footprint.
///
/// The support footprint is the overhang rectangle expanded by `xy_margin` in
/// X and Y. The function does not clip to the base: clipping is an arrangement
/// or mesh-domain operation and should be represented explicitly later.
/// Instead, this returns the exact expanded footprint plus a containment
/// status.
pub fn rectangular_support_footprint(
    overhang: RectangularPocket,
    base: RectangularPocket,
    xy_margin: Real,
    policy: PredicatePolicy,
) -> Result<RectangularSupportReport, SupportFootprintError> {
    if compare_reals_with_policy(&xy_margin, &Real::zero(), policy).value() == Some(Ordering::Less)
    {
        return Err(SupportFootprintError::NegativeMargin);
    }

    let footprint_min = Point2::new(
        overhang.min.x.clone() - xy_margin.clone(),
        overhang.min.y.clone() - xy_margin.clone(),
    );
    let footprint_max = Point2::new(
        overhang.max.x.clone() + xy_margin.clone(),
        overhang.max.y.clone() + xy_margin.clone(),
    );
    let footprint = RectangularPocket::new(footprint_min, footprint_max)
        .map_err(|_| SupportFootprintError::InvalidFootprint)?;
    let status = classify_rect_containment(&footprint, &base, policy);

    Ok(RectangularSupportReport {
        overhang,
        base,
        xy_margin,
        footprint,
        status,
    })
}

/// Compute the exact closed intersection of two rectangular regions.
pub fn intersect_rectangular_regions(
    first: RectangularPocket,
    second: RectangularPocket,
    policy: PredicatePolicy,
) -> Result<RectangularRegionIntersection, RegionBooleanError> {
    let min = Point2::new(
        max_real(&first.min.x, &second.min.x, policy)?,
        max_real(&first.min.y, &second.min.y, policy)?,
    );
    let max = Point2::new(
        min_real(&first.max.x, &second.max.x, policy)?,
        min_real(&first.max.y, &second.max.y, policy)?,
    );
    let x_order = compare_reals_with_policy(&min.x, &max.x, policy)
        .value()
        .ok_or(RegionBooleanError::UnknownComparison)?;
    let y_order = compare_reals_with_policy(&min.y, &max.y, policy)
        .value()
        .ok_or(RegionBooleanError::UnknownComparison)?;
    let (intersection, relation) = match (x_order, y_order) {
        (Ordering::Greater, _) | (_, Ordering::Greater) => {
            (None, RectangularRegionRelation::Disjoint)
        }
        (Ordering::Equal, _) | (_, Ordering::Equal) => {
            let intersection =
                RectangularPocket::new(min, max).map_err(|_| RegionBooleanError::InvalidRegion)?;
            (Some(intersection), RectangularRegionRelation::Touching)
        }
        (Ordering::Less, Ordering::Less) => {
            let intersection =
                RectangularPocket::new(min, max).map_err(|_| RegionBooleanError::InvalidRegion)?;
            (Some(intersection), RectangularRegionRelation::AreaOverlap)
        }
    };

    Ok(RectangularRegionIntersection {
        first,
        second,
        intersection,
        relation,
    })
}

/// Subtract one exact rectangular region from another.
///
/// The positive-area remainder is split into at most four rectangles around the
/// intersection: left, right, bottom, and top strips. Edge-only contact does
/// not remove area, so the original subject is retained as the sole remainder.
pub fn subtract_rectangular_region(
    subject: RectangularPocket,
    cutter: RectangularPocket,
    policy: PredicatePolicy,
) -> Result<RectangularRegionDifference, RegionBooleanError> {
    let intersection_report =
        intersect_rectangular_regions(subject.clone(), cutter.clone(), policy)?;
    if intersection_report.relation != RectangularRegionRelation::AreaOverlap {
        return Ok(RectangularRegionDifference {
            subject: intersection_report.first,
            cutter: intersection_report.second,
            intersection: intersection_report.intersection,
            remainder: vec![subject],
            relation: intersection_report.relation,
        });
    }

    let intersection = intersection_report
        .intersection
        .clone()
        .ok_or(RegionBooleanError::InvalidRegion)?;
    let mut remainder = Vec::new();
    push_positive_rect(
        &mut remainder,
        Point2::new(subject.min.x.clone(), subject.min.y.clone()),
        Point2::new(intersection.min.x.clone(), subject.max.y.clone()),
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.max.x.clone(), subject.min.y.clone()),
        Point2::new(subject.max.x.clone(), subject.max.y.clone()),
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.min.x.clone(), subject.min.y.clone()),
        Point2::new(intersection.max.x.clone(), intersection.min.y.clone()),
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.min.x.clone(), intersection.max.y.clone()),
        Point2::new(intersection.max.x.clone(), subject.max.y.clone()),
        policy,
    )?;

    Ok(RectangularRegionDifference {
        subject: intersection_report.first,
        cutter: intersection_report.second,
        intersection: Some(intersection),
        remainder,
        relation: intersection_report.relation,
    })
}

fn inset_rect(
    pocket: &RectangularPocket,
    inset: &Real,
    policy: PredicatePolicy,
) -> Option<(Option<Point2>, Option<Point2>)> {
    let min = Point2::new(
        pocket.min.x.clone() + inset.clone(),
        pocket.min.y.clone() + inset.clone(),
    );
    let max = Point2::new(
        pocket.max.x.clone() - inset.clone(),
        pocket.max.y.clone() - inset.clone(),
    );
    let x_order = compare_reals_with_policy(&min.x, &max.x, policy).value()?;
    let y_order = compare_reals_with_policy(&min.y, &max.y, policy).value()?;
    if matches!(x_order, Ordering::Less | Ordering::Equal)
        && matches!(y_order, Ordering::Less | Ordering::Equal)
    {
        Some((Some(min), Some(max)))
    } else {
        Some((None, None))
    }
}

fn classify_rect_containment(
    inner: &RectangularPocket,
    outer: &RectangularPocket,
    policy: PredicatePolicy,
) -> SupportFootprintStatus {
    let comparisons = [
        compare_reals_with_policy(&outer.min.x, &inner.min.x, policy).value(),
        compare_reals_with_policy(&outer.min.y, &inner.min.y, policy).value(),
        compare_reals_with_policy(&inner.max.x, &outer.max.x, policy).value(),
        compare_reals_with_policy(&inner.max.y, &outer.max.y, policy).value(),
    ];
    if comparisons.iter().any(Option::is_none) {
        return SupportFootprintStatus::Unknown;
    }
    if comparisons
        .into_iter()
        .flatten()
        .all(|ordering| matches!(ordering, Ordering::Less | Ordering::Equal))
    {
        SupportFootprintStatus::ContainedInBase
    } else {
        SupportFootprintStatus::OutsideBase
    }
}

fn push_positive_rect(
    output: &mut Vec<RectangularPocket>,
    min: Point2,
    max: Point2,
    policy: PredicatePolicy,
) -> Result<(), RegionBooleanError> {
    if positive_extent(&min.x, &max.x, policy)? && positive_extent(&min.y, &max.y, policy)? {
        output
            .push(RectangularPocket::new(min, max).map_err(|_| RegionBooleanError::InvalidRegion)?);
    }
    Ok(())
}

fn positive_extent(
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> Result<bool, RegionBooleanError> {
    Ok(compare_reals_with_policy(min, max, policy)
        .value()
        .ok_or(RegionBooleanError::UnknownComparison)?
        == Ordering::Less)
}

fn max_real(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Result<Real, RegionBooleanError> {
    match compare_reals_with_policy(first, second, policy)
        .value()
        .ok_or(RegionBooleanError::UnknownComparison)?
    {
        Ordering::Less => Ok(second.clone()),
        Ordering::Equal | Ordering::Greater => Ok(first.clone()),
    }
}

fn min_real(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Result<Real, RegionBooleanError> {
    match compare_reals_with_policy(first, second, policy)
        .value()
        .ok_or(RegionBooleanError::UnknownComparison)?
    {
        Ordering::Greater => Ok(second.clone()),
        Ordering::Equal | Ordering::Less => Ok(first.clone()),
    }
}

fn points_equal(first: &Point2, second: &Point2, policy: PredicatePolicy) -> bool {
    compare_reals_with_policy(&first.x, &second.x, policy).value() == Some(Ordering::Equal)
        && compare_reals_with_policy(&first.y, &second.y, policy).value() == Some(Ordering::Equal)
}

fn ordered_closed(min: &Real, max: &Real) -> bool {
    matches!(
        compare_reals_with_policy(min, max, PredicatePolicy).value(),
        Some(Ordering::Less | Ordering::Equal)
    )
}
