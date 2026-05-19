//! Boolean-topology handoff reports for Bezier curve relations.
//!
//! Bezier curve/curve predicates are only useful to path booleans after their
//! outputs are normalized into split events or explicit blockers. This module
//! provides that normalization layer without pretending that unresolved
//! algebraic regions are topology. The separation follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): exact
//! predicates either provide certified combinatorial data or retain an
//! auditable uncertainty object. The split/arrangement view follows the
//! intersection-insertion stage used by Greiner and Hormann, "Efficient
//! clipping of arbitrary polygons," *ACM Transactions on Graphics* 17(2),
//! 71-83 (1998), and Martinez, Rueda, and Feito, "A new algorithm for
//! computing Boolean operations on polygons," *Computers & Geosciences* 35(6),
//! 1177-1185 (2009).

use crate::{
    BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierIntersectionRegionIsolationCertificate, BezierIntersectionRegionShape,
    BezierIntersectionRegionSummary, Classification, IntersectionKind, LineLineIntersection,
    ParamRange, Point2, UncertaintyReason,
};
use hyperreal::Real;

/// Boolean-readiness state of a Bezier curve/curve relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanHandoffStatus {
    /// The relation certifies that no split events are required.
    NoEvents,
    /// Every retained event has exact parameters and can feed split insertion.
    SplitEventsReady,
    /// Point geometry is certified, but the curve parameters still need recovery.
    NeedsParameterRecovery,
    /// Same-image or finite-overlap geometry needs an overlap-aware resolver.
    NeedsOverlapResolver,
    /// Retained parameter regions still need algebraic isolation/refinement.
    NeedsRegionIsolation,
    /// The relation is not resolved enough for boolean topology.
    Unresolved,
    /// A lower-level primitive reported explicit predicate uncertainty.
    Uncertain,
}

/// Parameterized point event ready for future Bezier split insertion.
///
/// These events carry exact parameters on both source curves. A future Bezier
/// contour segment can evaluate the point from either curve at split time; the
/// optional point is retained when the source predicate already produced one.
/// Keeping point geometry separate from split parameters follows Yap's
/// predicate/construction boundary (1997) and avoids promoting isolated
/// regions that are not yet represented roots.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanPointEvent2 {
    /// Exact parameter on the first curve.
    pub first_param: Real,
    /// Exact parameter on the second curve.
    pub second_param: Real,
    /// Optional certified point supplied by the source predicate.
    pub point: Option<Point2>,
    /// Local contact kind when known.
    pub kind: Option<IntersectionKind>,
}

/// Parameterized overlap event that still needs boolean overlap policy.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOverlapEvent2 {
    /// Parameter range on the first curve.
    pub first_range: ParamRange,
    /// Parameter range on the second curve.
    pub second_range: ParamRange,
}

/// Machine-readable handoff from Bezier intersection predicates to booleans.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanHandoffReport2 {
    /// Coarse readiness state.
    pub status: BezierBooleanHandoffStatus,
    /// Exact split-ready point events.
    pub point_events: Vec<BezierBooleanPointEvent2>,
    /// Exact overlap ranges that require an overlap resolver.
    pub overlap_events: Vec<BezierBooleanOverlapEvent2>,
    /// Retained regions summarized for algebraic isolation.
    pub region_summary: Option<BezierIntersectionRegionSummary>,
    /// Optional retained isolation certificate used to classify a region frontier.
    pub isolation_certificate: Option<BezierIntersectionRegionIsolationCertificate>,
    /// Certified point witnesses that lack exact curve parameters.
    pub point_witnesses_needing_parameters: usize,
    /// Count of overlapping/same-image relation cases.
    pub overlap_relations_needing_resolution: usize,
    /// Count of unresolved predicate branches.
    pub unresolved_relations: usize,
    /// Count of lower-level uncertain primitive branches.
    pub uncertain_relations: usize,
    /// Explicit primitive uncertainty reason, when one was retained.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

impl BezierBooleanHandoffReport2 {
    /// Builds a boolean handoff report directly from a Bezier relation.
    ///
    /// This report does not run new geometry. It converts an already-certified
    /// relation into the data shape required by split-and-traverse booleans:
    /// point split events, overlap obligations, retained region obligations, or
    /// explicit blockers. Sederberg and Nishita's Bezier clipping cells
    /// ("Curve intersection using Bezier clipping," 1990) remain region
    /// obligations until a later algebraic isolator certifies represented
    /// roots.
    pub fn from_relation(relation: &BezierCurveRelation) -> Self {
        match relation {
            BezierCurveRelation::BoundingBoxesDisjoint | BezierCurveRelation::NoIntersection => {
                Self::no_events()
            }
            BezierCurveRelation::SameControlPolygon | BezierCurveRelation::SameCurveImage => {
                Self::overlap_relation()
            }
            BezierCurveRelation::SharedEndpoint => Self::parameter_recovery(1),
            BezierCurveRelation::EndpointIntersections { points }
            | BezierCurveRelation::IntersectionPoints { points } => {
                Self::parameter_recovery(points.len())
            }
            BezierCurveRelation::LineSegmentIntersection { intersection } => {
                Self::from_line_segment_intersection(intersection)
            }
            BezierCurveRelation::IntersectionRegions { regions } => Self::from_regions(regions),
            BezierCurveRelation::Unresolved => Self::unresolved(),
        }
    }

    /// Builds a boolean handoff from a classified Bezier relation.
    ///
    /// This is the convenience entry point for predicate APIs that return
    /// [`Classification`]. A classified uncertainty is retained as a boolean
    /// blocker instead of being collapsed into [`BezierCurveRelation::Unresolved`],
    /// preserving the difference between "the predicate could not decide" and
    /// "the relation was decided to need more algebra."
    pub fn from_classified_relation(relation: &Classification<BezierCurveRelation>) -> Self {
        match relation {
            Classification::Decided(relation) => Self::from_relation(relation),
            Classification::Uncertain(reason) => Self {
                status: BezierBooleanHandoffStatus::Uncertain,
                point_events: Vec::new(),
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 1,
                uncertainty_reason: Some(*reason),
            },
        }
    }

    /// Builds a report from a retained-region isolation certificate.
    ///
    /// A certificate is split-ready only when every terminal cell is an exact
    /// point cell. Target-width satisfaction alone is not enough for boolean
    /// topology: Yap's model requires a certified combinatorial object, not a
    /// small numeric box.
    pub fn from_isolation_certificate(
        certificate: &BezierIntersectionRegionIsolationCertificate,
    ) -> Self {
        let exact_cells = certificate.terminal_summary.exact_point_cells;
        let terminal_count = certificate.terminal_region_count;
        let split_ready = terminal_count > 0
            && exact_cells == terminal_count
            && certificate.terminal_summary.invalid_spans == 0
            && certificate.terminal_summary.unknown_regions == 0;

        let status = if terminal_count == 0 {
            BezierBooleanHandoffStatus::NoEvents
        } else if split_ready {
            BezierBooleanHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanHandoffStatus::NeedsRegionIsolation
        };

        Self {
            status,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: Some(certificate.terminal_summary.clone()),
            isolation_certificate: Some(certificate.clone()),
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    /// Returns true when the report can feed a split-event insertion stage.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierBooleanHandoffStatus::SplitEventsReady
    }

    /// Returns true when a later exact stage must run before boolean topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanHandoffStatus::NeedsParameterRecovery
                | BezierBooleanHandoffStatus::NeedsOverlapResolver
                | BezierBooleanHandoffStatus::NeedsRegionIsolation
                | BezierBooleanHandoffStatus::Unresolved
                | BezierBooleanHandoffStatus::Uncertain
        )
    }

    fn no_events() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::NoEvents,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn parameter_recovery(count: usize) -> Self {
        Self {
            status: if count == 0 {
                BezierBooleanHandoffStatus::NoEvents
            } else {
                BezierBooleanHandoffStatus::NeedsParameterRecovery
            },
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: count,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn overlap_relation() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::NeedsOverlapResolver,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 1,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn unresolved() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::Unresolved,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 1,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn from_line_segment_intersection(intersection: &LineLineIntersection) -> Self {
        match intersection {
            LineLineIntersection::None => Self::no_events(),
            LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            } => Self {
                status: BezierBooleanHandoffStatus::SplitEventsReady,
                point_events: vec![BezierBooleanPointEvent2 {
                    first_param: a_param.clone(),
                    second_param: b_param.clone(),
                    point: Some(point.clone()),
                    kind: Some(*kind),
                }],
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 0,
                uncertainty_reason: None,
            },
            LineLineIntersection::Overlap {
                a_range, b_range, ..
            } => Self {
                status: BezierBooleanHandoffStatus::NeedsOverlapResolver,
                point_events: Vec::new(),
                overlap_events: vec![BezierBooleanOverlapEvent2 {
                    first_range: a_range.clone(),
                    second_range: b_range.clone(),
                }],
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 1,
                unresolved_relations: 0,
                uncertain_relations: 0,
                uncertainty_reason: None,
            },
            LineLineIntersection::Uncertain { reason } => Self {
                status: BezierBooleanHandoffStatus::Uncertain,
                point_events: Vec::new(),
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 1,
                uncertainty_reason: Some(*reason),
            },
        }
    }

    fn from_regions(regions: &[BezierCurveIntersectionRegion]) -> Self {
        let summary = crate::summarize_bezier_intersection_regions(regions);
        let split_ready = !regions.is_empty()
            && summary.exact_point_cells == regions.len()
            && summary.invalid_spans == 0
            && summary.unknown_regions == 0;
        let status = if regions.is_empty() {
            BezierBooleanHandoffStatus::NoEvents
        } else if split_ready {
            BezierBooleanHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanHandoffStatus::NeedsRegionIsolation
        };
        let point_events = if split_ready {
            regions
                .iter()
                .filter_map(exact_point_event_from_region)
                .collect()
        } else {
            Vec::new()
        };

        Self {
            status,
            point_events,
            overlap_events: Vec::new(),
            region_summary: Some(summary),
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }
}

fn exact_point_event_from_region(
    region: &BezierCurveIntersectionRegion,
) -> Option<BezierBooleanPointEvent2> {
    let facts = crate::bezier_intersection_region_facts(region);
    if facts.shape != BezierIntersectionRegionShape::ExactPointCell {
        return None;
    }
    Some(BezierBooleanPointEvent2 {
        first_param: region.first().start().clone(),
        second_param: region.second().start().clone(),
        point: None,
        kind: None,
    })
}
