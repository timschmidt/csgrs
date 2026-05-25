//! Exact CAM planning scaffolds.
//!
//! Subtractive CAM planners commonly generate contour-parallel pocket passes
//! by repeatedly offsetting a source boundary and then cleaning the resulting
//! arrangements. This module starts only the exact scheduling layer for
//! axis-aligned rectangular pockets. It follows Yap, "Towards Exact Geometric
//! Computation," by retaining exact objects and refusing to imply that a
//! scheduled ring is valid output until later arrangement, gouge, and linking
//! predicates certify it. The staged split mirrors CGAL-style offset pipelines
//! and the pair-wise offset literature used by contour-parallel machining.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts, RealSign};

use crate::provenance::PathProvenance;

/// Exact axis-aligned rectangular pocket boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularPocket {
    min: Point2,
    max: Point2,
    provenance: PathProvenance,
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

/// Reason contour-parallel ring scheduling stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PocketPlanStopReason {
    /// The requested maximum ring count was reached before geometric exhaustion.
    MaxRingsReached,
    /// The next inset would collapse at least one rectangle axis.
    GeometryExhausted,
    /// Exact comparison could not certify whether the next ring is valid.
    Unknown,
}

/// Exact contour-parallel pocket schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularPocketPlan {
    /// Source pocket boundary.
    pub pocket: RectangularPocket,
    /// Exact tool radius used for the first inset.
    pub tool_radius: Real,
    /// Exact stepover added between successive rings.
    pub stepover: Real,
    /// Scheduled rings in construction order.
    pub rings: Vec<PocketOffsetRing>,
    /// Why scheduling stopped.
    pub stop_reason: PocketPlanStopReason,
}

/// Errors while constructing exact rectangular pocket schedules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PocketPlanError {
    /// Pocket bounds were not exactly ordered.
    UnorderedBounds,
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

/// Exact rectangular additive bead schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularBeadPlan {
    /// Source rectangular region.
    pub region: RectangularPocket,
    /// Fill direction.
    pub axis: BeadFillAxis,
    /// Exact bead width.
    pub bead_width: Real,
    /// Exact centerline pitch.
    pub spacing: Real,
    /// Scheduled bead centerlines.
    pub beads: Vec<AdditiveBeadLine>,
    /// Why scheduling stopped.
    pub stop_reason: PocketPlanStopReason,
}

/// Errors while constructing exact additive bead schedules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BeadPlanError {
    /// Region bounds were not exactly ordered.
    UnorderedBounds,
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
    /// Source bead schedule.
    pub plan: RectangularBeadPlan,
    /// Bead centerlines oriented in serpentine traversal order.
    pub deposition_segments: Vec<crate::segment::LinePathSegment>,
    /// Exact connector edges between adjacent deposition segments.
    pub links: Vec<AdditiveInfillLink>,
}

/// Errors while constructing exact additive infill graphs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InfillGraphError {
    /// No bead centerlines were available to graph.
    EmptyBeadPlan,
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

/// Exact rectangular additive support footprint.
///
/// Support generation is usually implemented as an image/slice heuristic. This
/// carrier keeps the construction in Yap's exact object layer: derive a
/// footprint, retain the source overhang/base rectangles, and expose a
/// predicate result before a downstream process planner accepts support moves.
/// The staged treatment mirrors additive slicing/support surveys such as
/// Kulkarni, Marsan, and Dutta, "A review of process planning techniques in
/// layered manufacturing", while avoiding tolerance-only geometry decisions.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularSupportPlan {
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
pub enum SupportPlanError {
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
/// provenance and relation status intact.
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
    /// Construct an exact rectangular pocket with native provenance.
    pub fn new(min: Point2, max: Point2) -> Result<Self, PocketPlanError> {
        Self::with_provenance(min, max, PathProvenance::native())
    }

    /// Construct an exact rectangular pocket with source provenance.
    pub fn with_provenance(
        min: Point2,
        max: Point2,
        provenance: PathProvenance,
    ) -> Result<Self, PocketPlanError> {
        if !ordered_closed(&min.x, &max.x) || !ordered_closed(&min.y, &max.y) {
            return Err(PocketPlanError::UnorderedBounds);
        }
        let exact = Real::exact_set_facts([&min.x, &min.y, &max.x, &max.y]);
        Ok(Self {
            min,
            max,
            provenance,
            exact,
        })
    }

    /// Return exact minimum corner.
    pub const fn min(&self) -> &Point2 {
        &self.min
    }

    /// Return exact maximum corner.
    pub const fn max(&self) -> &Point2 {
        &self.max
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
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

/// Build an exact rectangular contour-parallel pocket schedule.
///
/// The first ring is inset by `tool_radius`; each next ring is inset by one
/// additional `stepover`. The function stops before emitting a ring whose
/// bounds cannot be certified as ordered. This is only the pocket/rest graph
/// skeleton: no path linking, cutter engagement, corner cleanup, or rest
/// material decision is accepted here.
pub fn build_rectangular_pocket_plan(
    pocket: RectangularPocket,
    tool_radius: Real,
    stepover: Real,
    max_rings: usize,
    policy: PredicatePolicy,
) -> Result<RectangularPocketPlan, PocketPlanError> {
    if max_rings == 0 {
        return Err(PocketPlanError::ZeroMaxRings);
    }
    if tool_radius.structural_facts().sign == Some(RealSign::Negative) {
        return Err(PocketPlanError::NegativeToolRadius);
    }
    if compare_reals_with_policy(&stepover, &Real::zero(), policy).value()
        != Some(Ordering::Greater)
    {
        return Err(PocketPlanError::NonPositiveStepover);
    }

    let mut rings = Vec::new();
    let mut inset = tool_radius.clone();
    let stop_reason = loop {
        if rings.len() == max_rings {
            break PocketPlanStopReason::MaxRingsReached;
        }
        let Some((min, max)) = inset_rect(&pocket, &inset, policy) else {
            break PocketPlanStopReason::Unknown;
        };
        if min.is_none() {
            break PocketPlanStopReason::GeometryExhausted;
        }
        rings.push(PocketOffsetRing {
            index: rings.len(),
            inset: inset.clone(),
            min: min.unwrap(),
            max: max.unwrap(),
        });
        inset = inset + stepover.clone();
    };

    Ok(RectangularPocketPlan {
        pocket,
        tool_radius,
        stepover,
        rings,
        stop_reason,
    })
}

/// Build an exact rectangular additive bead schedule.
///
/// The first bead centerline is inset by `bead_width / 2` from the low side of
/// the pitch axis, and later beads advance by `spacing`. This is the additive
/// analogue of the pocket-ring scheduler: it creates exact candidate
/// centerlines for infill/skin planning while leaving region set algebra,
/// supports, corner starts/stops, and process validation to downstream exact
/// predicates.
pub fn build_rectangular_bead_plan(
    region: RectangularPocket,
    axis: BeadFillAxis,
    bead_width: Real,
    spacing: Real,
    max_beads: usize,
    policy: PredicatePolicy,
) -> Result<RectangularBeadPlan, BeadPlanError> {
    if max_beads == 0 {
        return Err(BeadPlanError::ZeroMaxBeads);
    }
    if compare_reals_with_policy(&bead_width, &Real::zero(), policy).value()
        != Some(Ordering::Greater)
    {
        return Err(BeadPlanError::NonPositiveBeadWidth);
    }
    if compare_reals_with_policy(&spacing, &Real::zero(), policy).value() != Some(Ordering::Greater)
    {
        return Err(BeadPlanError::NonPositiveSpacing);
    }
    if !ordered_closed(&region.min.x, &region.max.x)
        || !ordered_closed(&region.min.y, &region.max.y)
    {
        return Err(BeadPlanError::UnorderedBounds);
    }

    let half_width =
        (bead_width.clone() / Real::from(2)).map_err(|_| BeadPlanError::NonPositiveBeadWidth)?;
    let mut beads = Vec::new();
    let mut pitch_position = match axis {
        BeadFillAxis::Horizontal => region.min.y.clone() + half_width.clone(),
        BeadFillAxis::Vertical => region.min.x.clone() + half_width.clone(),
    };
    let stop_reason = loop {
        if beads.len() == max_beads {
            break PocketPlanStopReason::MaxRingsReached;
        }
        let limit = match axis {
            BeadFillAxis::Horizontal => region.max.y.clone() - half_width.clone(),
            BeadFillAxis::Vertical => region.max.x.clone() - half_width.clone(),
        };
        let Some(ordering) = compare_reals_with_policy(&pitch_position, &limit, policy).value()
        else {
            break PocketPlanStopReason::Unknown;
        };
        if ordering == Ordering::Greater {
            break PocketPlanStopReason::GeometryExhausted;
        }

        let segment = match axis {
            BeadFillAxis::Horizontal => crate::segment::LinePathSegment::with_provenance(
                Point2::new(region.min.x.clone(), pitch_position.clone()),
                Point2::new(region.max.x.clone(), pitch_position.clone()),
                region.provenance,
            ),
            BeadFillAxis::Vertical => crate::segment::LinePathSegment::with_provenance(
                Point2::new(pitch_position.clone(), region.min.y.clone()),
                Point2::new(pitch_position.clone(), region.max.y.clone()),
                region.provenance,
            ),
        };
        beads.push(AdditiveBeadLine {
            index: beads.len(),
            segment,
            pitch_position: pitch_position.clone(),
        });
        pitch_position = pitch_position + spacing.clone();
    };

    Ok(RectangularBeadPlan {
        region,
        axis,
        bead_width,
        spacing,
        beads,
        stop_reason,
    })
}

/// Build an exact serpentine infill graph from a rectangular bead plan.
///
/// The graph alternates bead direction, then inserts exact straight connectors
/// between consecutive oriented bead centerlines. It validates every generated
/// connector endpoint with `hyperlimit` equality rather than relying on object
/// identity, preserving the Yap-style separation between construction and
/// predicate certification.
pub fn build_rectangular_serpentine_infill_graph(
    plan: RectangularBeadPlan,
    policy: PredicatePolicy,
) -> Result<RectangularInfillGraph, InfillGraphError> {
    if plan.beads.is_empty() {
        return Err(InfillGraphError::EmptyBeadPlan);
    }

    let deposition_segments: Vec<_> = plan
        .beads
        .iter()
        .enumerate()
        .map(|(index, bead)| {
            if index % 2 == 0 {
                bead.segment.clone()
            } else {
                crate::segment::LinePathSegment::with_provenance(
                    bead.segment.end().clone(),
                    bead.segment.start().clone(),
                    bead.segment.provenance(),
                )
            }
        })
        .collect();

    let mut links = Vec::new();
    for (index, pair) in deposition_segments.windows(2).enumerate() {
        let current = &pair[0];
        let next = &pair[1];
        let connector = crate::segment::LinePathSegment::with_provenance(
            current.end().clone(),
            next.start().clone(),
            plan.region.provenance,
        );
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
        plan,
        deposition_segments,
        links,
    })
}

/// Build and classify an exact rectangular support footprint.
///
/// The support footprint is the overhang rectangle expanded by `xy_margin` in
/// X and Y. The function does not clip to the base: clipping is an arrangement
/// or mesh-domain operation and should be represented explicitly later.
/// Instead, this returns the exact expanded footprint plus a containment
/// status.
pub fn build_rectangular_support_plan(
    overhang: RectangularPocket,
    base: RectangularPocket,
    xy_margin: Real,
    policy: PredicatePolicy,
) -> Result<RectangularSupportPlan, SupportPlanError> {
    if compare_reals_with_policy(&xy_margin, &Real::zero(), policy).value() == Some(Ordering::Less)
    {
        return Err(SupportPlanError::NegativeMargin);
    }

    let footprint_min = Point2::new(
        overhang.min.x.clone() - xy_margin.clone(),
        overhang.min.y.clone() - xy_margin.clone(),
    );
    let footprint_max = Point2::new(
        overhang.max.x.clone() + xy_margin.clone(),
        overhang.max.y.clone() + xy_margin.clone(),
    );
    let footprint =
        RectangularPocket::with_provenance(footprint_min, footprint_max, overhang.provenance)
            .map_err(|_| SupportPlanError::InvalidFootprint)?;
    let status = classify_rect_containment(&footprint, &base, policy);

    Ok(RectangularSupportPlan {
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
            let intersection = RectangularPocket::with_provenance(min, max, first.provenance)
                .map_err(|_| RegionBooleanError::InvalidRegion)?;
            (Some(intersection), RectangularRegionRelation::Touching)
        }
        (Ordering::Less, Ordering::Less) => {
            let intersection = RectangularPocket::with_provenance(min, max, first.provenance)
                .map_err(|_| RegionBooleanError::InvalidRegion)?;
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
        subject.provenance,
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.max.x.clone(), subject.min.y.clone()),
        Point2::new(subject.max.x.clone(), subject.max.y.clone()),
        subject.provenance,
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.min.x.clone(), subject.min.y.clone()),
        Point2::new(intersection.max.x.clone(), intersection.min.y.clone()),
        subject.provenance,
        policy,
    )?;
    push_positive_rect(
        &mut remainder,
        Point2::new(intersection.min.x.clone(), intersection.max.y.clone()),
        Point2::new(intersection.max.x.clone(), subject.max.y.clone()),
        subject.provenance,
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
    provenance: PathProvenance,
    policy: PredicatePolicy,
) -> Result<(), RegionBooleanError> {
    if positive_extent(&min.x, &max.x, policy)? && positive_extent(&min.y, &max.y, policy)? {
        output.push(
            RectangularPocket::with_provenance(min, max, provenance)
                .map_err(|_| RegionBooleanError::InvalidRegion)?,
        );
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
        compare_reals_with_policy(min, max, PredicatePolicy::default()).value(),
        Some(Ordering::Less | Ordering::Equal)
    )
}
