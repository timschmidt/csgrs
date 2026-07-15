//! Exact support k-DOP carriers and slab-projection classifiers.
//!
//! A k-DOP is represented here as a finite set of directional slabs
//! `min <= axis . point <= max`. The module deliberately stores the retained
//! support witnesses that produced each bound, preserving object-level structure
//! instead of expanding every decision into unrelated scalar tests. The
//! interval-overlap classifier is the support-function view used by k-DOP
//! collision systems.

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use hyperreal::Real;

use crate::classify::{
    ConvexPointLocation, HalfspaceFeasibility, SupportDopPlaneRelation, SupportDopRelation,
};
use crate::geometry::{Plane3, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome};
use crate::predicates::halfspace::{
    HalfspaceFeasibilityReport, classify_halfspace_feasibility3_with_policy,
};
use crate::predicates::order::compare_reals_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};

/// Exact integer direction used by a witnessed support slab.
///
/// The vector is intentionally not normalized. k-DOP comparisons only require
/// a consistent linear functional, and keeping integer directions preserves a
/// cheap shared-scale support representation for callers that build broad
/// phase objects from indexed point sets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopAxis3 {
    /// Integer coefficients of the support direction.
    pub direction: [i64; 3],
}

impl SupportDopAxis3 {
    /// Construct a support axis from exact integer coefficients.
    pub const fn new(direction: [i64; 3]) -> Self {
        Self { direction }
    }

    /// Return whether this axis has at least one nonzero coefficient.
    pub const fn is_nonzero(self) -> bool {
        self.direction[0] != 0 || self.direction[1] != 0 || self.direction[2] != 0
    }

    /// Validate that the direction can define a support functional.
    pub fn validate(self) -> Result<(), SupportDopValidationError> {
        if self.is_nonzero() {
            Ok(())
        } else {
            Err(SupportDopValidationError::ZeroAxis)
        }
    }

    /// Convert this integer axis to the point-vector carrier used by
    /// [`SupportDop3`].
    pub fn to_point3(self) -> Point3 {
        Point3::new(
            Real::from(self.direction[0]),
            Real::from(self.direction[1]),
            Real::from(self.direction[2]),
        )
    }

    /// Three axes that produce the same six planes as an exact AABB.
    pub const fn orthogonal_axes() -> [Self; 3] {
        [
            Self::new([1, 0, 0]),
            Self::new([0, 1, 0]),
            Self::new([0, 0, 1]),
        ]
    }

    /// Thirteen axes for a 26-DOP over axis, face-diagonal, and body-diagonal
    /// support directions.
    pub const fn kdop26_axes() -> [Self; 13] {
        [
            Self::new([1, 0, 0]),
            Self::new([0, 1, 0]),
            Self::new([0, 0, 1]),
            Self::new([1, 1, 0]),
            Self::new([1, -1, 0]),
            Self::new([1, 0, 1]),
            Self::new([1, 0, -1]),
            Self::new([0, 1, 1]),
            Self::new([0, 1, -1]),
            Self::new([1, 1, 1]),
            Self::new([1, 1, -1]),
            Self::new([1, -1, 1]),
            Self::new([-1, 1, 1]),
        ]
    }
}

/// Why a witnessed support-DOP consumer must conservatively expand exact slabs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopExpansionKind {
    /// No expansion is needed; every slab is an exact summary of exact source
    /// coordinates.
    None,
    /// The source entered through a primitive-float or external adapter edge.
    LossyAdapter,
    /// Coordinates were rounded to an integer grid before support extraction.
    IntegerGridRounding,
}

/// Conservative-expansion metadata attached to witnessed support-DOP bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopExpansionReport {
    /// Expansion source category.
    pub kind: SupportDopExpansionKind,
    /// Number of axes covered by this report.
    pub axis_count: usize,
    /// Number of slabs that must be treated as conservatively expanded.
    pub expanded_slabs: usize,
    /// Nonnegative distance expansion applied on both sides of each expanded
    /// slab.
    pub expansion: Real,
}

impl SupportDopExpansionReport {
    /// Build an exact no-expansion report for `axis_count` slabs.
    pub fn exact(axis_count: usize) -> Self {
        Self {
            kind: SupportDopExpansionKind::None,
            axis_count,
            expanded_slabs: 0,
            expansion: Real::from(0),
        }
    }

    /// Build an integer-grid rounding expansion report.
    pub fn integer_grid_rounding(axis_count: usize, expansion: Real) -> Self {
        Self {
            kind: SupportDopExpansionKind::IntegerGridRounding,
            axis_count,
            expanded_slabs: axis_count,
            expansion,
        }
    }

    /// Build a lossy-adapter expansion report for `axis_count` slabs.
    pub fn lossy_adapter(axis_count: usize) -> Self {
        Self {
            kind: SupportDopExpansionKind::LossyAdapter,
            axis_count,
            expanded_slabs: axis_count,
            expansion: Real::from(0),
        }
    }

    /// Validate internal report consistency.
    pub fn validate(&self) -> Result<(), SupportDopValidationError> {
        if !matches!(
            compare_real_default(&self.expansion, &Real::from(0)),
            Some(Ordering::Greater | Ordering::Equal)
        ) {
            return Err(SupportDopValidationError::NegativeExpansion);
        }
        match self.kind {
            SupportDopExpansionKind::None => {
                if self.expanded_slabs != 0
                    || !matches!(
                        compare_real_default(&self.expansion, &Real::from(0)),
                        Some(Ordering::Equal)
                    )
                {
                    return Err(SupportDopValidationError::ExpansionKindMismatch);
                }
            }
            SupportDopExpansionKind::LossyAdapter
            | SupportDopExpansionKind::IntegerGridRounding => {
                if self.expanded_slabs != self.axis_count {
                    return Err(SupportDopValidationError::ExpansionKindMismatch);
                }
            }
        }
        Ok(())
    }
}

/// Exact support witness for one side of one witnessed k-DOP slab.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportWitness3 {
    /// Source point index that witnesses the retained support distance.
    pub vertex: usize,
    /// Exact point copied from the source point set at construction time.
    pub point: Point3,
    /// Exact unnormalized support distance along the slab axis.
    pub distance: Real,
}

/// One min/max support slab with source-point witnesses.
#[derive(Clone, Debug, PartialEq)]
pub struct WitnessedSupportSlab3 {
    /// Integer direction used by this support functional.
    pub axis: SupportDopAxis3,
    /// Minimum support witness.
    pub min: SupportWitness3,
    /// Maximum support witness.
    pub max: SupportWitness3,
}

impl WitnessedSupportSlab3 {
    /// Return the conservative minimum distance after applying the containing
    /// expansion report.
    pub fn conservative_min_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.min.distance.clone() - expansion.expansion.clone()
    }

    /// Return the conservative maximum distance after applying the containing
    /// expansion report.
    pub fn conservative_max_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.max.distance.clone() + expansion.expansion.clone()
    }

    /// Convert this witnessed slab to the classifier slab used by
    /// [`SupportDop3`].
    pub fn to_support_slab3(&self) -> SupportSlab3 {
        SupportSlab3 {
            axis: self.axis.to_point3(),
            min: self.min.distance.clone(),
            max: self.max.distance.clone(),
            min_witness: Some(self.min.vertex),
            max_witness: Some(self.max.vertex),
        }
    }
}

/// Exact k-DOP bounds with support-vertex witnesses.
#[derive(Clone, Debug, PartialEq)]
pub struct WitnessedSupportDop3 {
    /// Number of points summarized by this object.
    pub vertex_count: usize,
    /// One min/max support slab per retained axis.
    pub slabs: Vec<WitnessedSupportSlab3>,
    /// Conservative adapter/rounding expansion metadata.
    pub expansion: SupportDopExpansionReport,
}

impl WitnessedSupportDop3 {
    /// Build a support-DOP from exact points and an explicit expansion report.
    pub fn from_points_with_expansion(
        points: &[Point3],
        axes: &[SupportDopAxis3],
        expansion: SupportDopExpansionReport,
    ) -> Result<Self, SupportDopValidationError> {
        if points.is_empty() {
            return Err(SupportDopValidationError::EmptyPointSet);
        }
        if axes.is_empty() {
            return Err(SupportDopValidationError::EmptyAxisSet);
        }
        if expansion.axis_count != axes.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        expansion.validate()?;
        let slabs = axes
            .iter()
            .map(|&axis| compute_witnessed_slab(points, axis))
            .collect::<Result<Vec<_>, _>>()?;
        let support = Self {
            vertex_count: points.len(),
            slabs,
            expansion,
        };
        support.validate_against_points(points)?;
        Ok(support)
    }

    /// Build an exact no-expansion support-DOP from points.
    pub fn from_points(
        points: &[Point3],
        axes: &[SupportDopAxis3],
    ) -> Result<Self, SupportDopValidationError> {
        Self::from_points_with_expansion(points, axes, SupportDopExpansionReport::exact(axes.len()))
    }

    /// Return this witnessed carrier as the predicate-level support-DOP
    /// classifier.
    pub fn to_support_dop3(&self) -> SupportDop3 {
        SupportDop3::from_slabs(
            self.slabs
                .iter()
                .map(WitnessedSupportSlab3::to_support_slab3)
                .collect(),
        )
    }

    /// Validate this k-DOP against exact source points.
    pub fn validate_against_points(
        &self,
        points: &[Point3],
    ) -> Result<(), SupportDopValidationError> {
        if points.is_empty() {
            return Err(SupportDopValidationError::EmptyPointSet);
        }
        if self.slabs.is_empty() {
            return Err(SupportDopValidationError::EmptyAxisSet);
        }
        if self.vertex_count != points.len() {
            return Err(SupportDopValidationError::VertexCountMismatch);
        }
        if self.expansion.axis_count != self.slabs.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        self.expansion.validate()?;
        for slab in &self.slabs {
            validate_witnessed_slab(slab, points)?;
        }
        Ok(())
    }

    /// Refresh slabs after a bounded set of point updates.
    pub fn refresh_for_changed_vertices(
        &mut self,
        points: &[Point3],
        changed_vertices: &[usize],
    ) -> Result<SupportDopRefreshReport, SupportDopValidationError> {
        if self.vertex_count != points.len() {
            return Err(SupportDopValidationError::VertexCountMismatch);
        }
        if self.expansion.axis_count != self.slabs.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        for &vertex in changed_vertices {
            if vertex >= points.len() {
                return Err(SupportDopValidationError::ChangedVertexOutOfRange);
            }
        }

        let mut report = SupportDopRefreshReport {
            changed_vertices: changed_vertices.len(),
            axis_count: self.slabs.len(),
            axes_rebuilt: 0,
            axes_extended: 0,
            axes_unchanged: 0,
            invalidated_witness_axes: 0,
        };

        for slab in &mut self.slabs {
            let witness_invalidated = changed_vertices
                .iter()
                .any(|&vertex| vertex == slab.min.vertex || vertex == slab.max.vertex);
            if witness_invalidated {
                *slab = compute_witnessed_slab(points, slab.axis)?;
                report.axes_rebuilt += 1;
                report.invalidated_witness_axes += 1;
                continue;
            }

            let mut extended = false;
            for &vertex in changed_vertices {
                let distance = support_distance_on_integer_axis(&points[vertex], slab.axis);
                if matches!(
                    compare_real_default(&distance, &slab.min.distance),
                    Some(Ordering::Less)
                ) {
                    slab.min = support_witness(vertex, &points[vertex], distance.clone());
                    extended = true;
                }
                if matches!(
                    compare_real_default(&distance, &slab.max.distance),
                    Some(Ordering::Greater)
                ) {
                    slab.max = support_witness(vertex, &points[vertex], distance);
                    extended = true;
                }
            }

            if extended {
                report.axes_extended += 1;
            } else {
                report.axes_unchanged += 1;
            }
        }

        self.validate_against_points(points)?;
        Ok(report)
    }
}

/// Refresh summary returned by
/// [`WitnessedSupportDop3::refresh_for_changed_vertices`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopRefreshReport {
    /// Number of changed point indices supplied by the caller.
    pub changed_vertices: usize,
    /// Number of support axes retained by the k-DOP.
    pub axis_count: usize,
    /// Axes fully rebuilt because a min or max witness changed.
    pub axes_rebuilt: usize,
    /// Axes updated in place because a changed non-witness became more
    /// extreme.
    pub axes_extended: usize,
    /// Axes untouched after evaluating changed non-witness points.
    pub axes_unchanged: usize,
    /// Axes whose retained support witness was explicitly invalidated.
    pub invalidated_witness_axes: usize,
}

/// Error returned by witnessed support-DOP construction or replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopValidationError {
    /// No source points were supplied.
    EmptyPointSet,
    /// No support axes were supplied.
    EmptyAxisSet,
    /// An axis direction was zero.
    ZeroAxis,
    /// Retained point count disagrees with the source point count.
    VertexCountMismatch,
    /// Expansion report axis count disagrees with the slab count.
    ExpansionAxisCountMismatch,
    /// A support witness points outside the source point slice.
    WitnessOutOfRange,
    /// A retained witness point no longer matches the source point.
    WitnessPointMismatch,
    /// A retained witness distance no longer equals the axis dot product.
    WitnessDistanceMismatch,
    /// A min witness is not minimal for its axis.
    WitnessNotMinimal,
    /// A max witness is not maximal for its axis.
    WitnessNotMaximal,
    /// A min/max ordering could not be certified for a slab.
    UnknownSlabOrder,
    /// Expansion distance is negative or not certifiably nonnegative.
    NegativeExpansion,
    /// Expansion kind, slab count, or zero-expansion state is contradictory.
    ExpansionKindMismatch,
    /// A changed point index is outside the source point slice.
    ChangedVertexOutOfRange,
}

/// One retained support slab of a 3D k-DOP.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportSlab3 {
    /// Projection direction used by this slab.
    pub axis: Point3,
    /// Minimum exact projection value.
    pub min: Real,
    /// Maximum exact projection value.
    pub max: Real,
    /// Index of the source point that attained [`Self::min`], when the slab
    /// was built from a point set.
    pub min_witness: Option<usize>,
    /// Index of the source point that attained [`Self::max`], when the slab
    /// was built from a point set.
    pub max_witness: Option<usize>,
}

impl SupportSlab3 {
    /// Construct a slab from explicit bounds.
    ///
    /// The bounds are accepted as data and validated by classification calls.
    /// Use [`SupportDop3::from_points`] when support witnesses should be
    /// derived from exact source points.
    pub fn new(axis: Point3, min: Real, max: Real) -> Self {
        Self {
            axis,
            min,
            max,
            min_witness: None,
            max_witness: None,
        }
    }

    /// Return the exact projection `axis . point`.
    pub fn project_point(&self, point: &Point3) -> Real {
        project_point_on_axis(&self.axis, point)
    }
}

/// Structural inconsistency in a retained support-DOP/AABB report.
///
/// The report validates the exact support-interval broad-phase reduction
/// instead of treating it as a lossy bounding-volume hint. Object evidence
/// remains available for replay across the geometric system.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopAabb3ValidationError {
    /// An empty retained DOP did not report the structural degenerate relation.
    EmptyDopRelationMismatch,
    /// A report item is not the next prefix slab tested by the classifier.
    SlabIndexMismatch,
    /// A retained slab has `min > max` but is not reported as degenerate.
    SlabBoundsInvalid,
    /// A valid retained slab is missing its exact query projection interval.
    MissingQueryInterval,
    /// A degenerate retained slab unexpectedly carries a query projection.
    DegenerateSlabHasQueryInterval,
    /// A retained query interval has `query_min > query_max`.
    QueryIntervalInvalid,
    /// A retained per-slab relation does not replay from retained intervals.
    SlabRelationMismatch,
    /// The first separating or degenerate slab does not match the terminal id.
    TerminalSlabMismatch,
    /// A non-terminal report does not cover every retained slab.
    MissingSlabEvidence,
    /// The retained slab relations derive a different coarse relation.
    RelationMismatch,
    /// Recomputing from source geometry did not reproduce this report.
    SourceReplayMismatch,
}

/// Per-slab exact interval evidence for [`SupportDopAabb3Report`].
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopAabb3SlabReport {
    /// Index of the retained slab tested by this report item.
    pub slab_index: usize,
    /// Exact retained slab copied into the report for standalone replay.
    pub slab: SupportSlab3,
    /// Minimum exact AABB projection on [`Self::slab`]'s axis.
    ///
    /// Degenerate slabs omit the query interval because invalid retained
    /// bounds decide the structural result before the query is needed.
    pub query_min: Option<Real>,
    /// Maximum exact AABB projection on [`Self::slab`]'s axis.
    pub query_max: Option<Real>,
    /// Relation derived from this slab alone.
    pub relation: SupportDopRelation,
}

/// Report-bearing exact support-DOP/AABB classification.
///
/// The coarse [`SupportDopRelation`] is retained for existing callers. This
/// report stores the visited slab prefix, each exact AABB support interval,
/// and the first terminal slab when a separating axis or invalid retained slab
/// is found. Non-separated reports contain every retained slab so downstream
/// broad-phase, voxel, and packing code can replay the conservative
/// support-interval predicate without reintroducing primitive-float tolerance.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopAabb3Report {
    /// Coarse relation derived from the retained slab evidence.
    pub relation: SupportDopRelation,
    /// Number of slabs retained by the source DOP.
    pub slab_count: usize,
    /// First slab that decided [`SupportDopRelation::Separated`] or
    /// [`SupportDopRelation::Degenerate`], when classification stopped early.
    pub terminal_slab: Option<usize>,
    /// Prefix of visited slab reports.
    pub slab_reports: Vec<SupportDopAabb3SlabReport>,
}

impl SupportDopAabb3Report {
    /// Validate retained slab evidence and the derived coarse relation.
    pub fn validate(&self) -> Result<(), SupportDopAabb3ValidationError> {
        if self.slab_count == 0 {
            return if self.relation == SupportDopRelation::Degenerate
                && self.terminal_slab.is_none()
                && self.slab_reports.is_empty()
            {
                Ok(())
            } else {
                Err(SupportDopAabb3ValidationError::EmptyDopRelationMismatch)
            };
        }

        let mut boundary = false;
        let mut derived_relation = None;
        for (position, slab_report) in self.slab_reports.iter().enumerate() {
            if slab_report.slab_index != position || position >= self.slab_count {
                return Err(SupportDopAabb3ValidationError::SlabIndexMismatch);
            }

            let slab_bounds_valid =
                decided_bool(validate_slab_bounds(&slab_report.slab, PredicatePolicy))
                    .ok_or(SupportDopAabb3ValidationError::SlabBoundsInvalid)?;

            if !slab_bounds_valid {
                if slab_report.relation != SupportDopRelation::Degenerate {
                    return Err(SupportDopAabb3ValidationError::SlabBoundsInvalid);
                }
                if slab_report.query_min.is_some() || slab_report.query_max.is_some() {
                    return Err(SupportDopAabb3ValidationError::DegenerateSlabHasQueryInterval);
                }
                if self.terminal_slab != Some(position) || position + 1 != self.slab_reports.len() {
                    return Err(SupportDopAabb3ValidationError::TerminalSlabMismatch);
                }
                derived_relation = Some(SupportDopRelation::Degenerate);
                break;
            }

            let query_min = slab_report
                .query_min
                .as_ref()
                .ok_or(SupportDopAabb3ValidationError::MissingQueryInterval)?;
            let query_max = slab_report
                .query_max
                .as_ref()
                .ok_or(SupportDopAabb3ValidationError::MissingQueryInterval)?;
            let query_interval_valid =
                match compare_reals_with_policy(query_min, query_max, PredicatePolicy) {
                    PredicateOutcome::Decided { value, .. } => value != Ordering::Greater,
                    PredicateOutcome::Unknown { .. } => false,
                };
            if !query_interval_valid {
                return Err(SupportDopAabb3ValidationError::QueryIntervalInvalid);
            }

            let replayed = match classify_interval_overlap(
                query_min,
                query_max,
                &slab_report.slab.min,
                &slab_report.slab.max,
                PredicatePolicy,
            ) {
                PredicateOutcome::Decided { value, .. } => value,
                PredicateOutcome::Unknown { .. } => {
                    return Err(SupportDopAabb3ValidationError::SlabRelationMismatch);
                }
            };
            if replayed != slab_report.relation {
                return Err(SupportDopAabb3ValidationError::SlabRelationMismatch);
            }

            match slab_report.relation {
                SupportDopRelation::Separated | SupportDopRelation::Degenerate => {
                    if self.terminal_slab != Some(position)
                        || position + 1 != self.slab_reports.len()
                    {
                        return Err(SupportDopAabb3ValidationError::TerminalSlabMismatch);
                    }
                    derived_relation = Some(slab_report.relation);
                    break;
                }
                SupportDopRelation::BoundaryTouch => boundary = true,
                SupportDopRelation::ConservativeOverlap => {}
            }
        }

        let derived_relation = match derived_relation {
            Some(relation) => relation,
            None => {
                if self.terminal_slab.is_some() {
                    return Err(SupportDopAabb3ValidationError::TerminalSlabMismatch);
                }
                if self.slab_reports.len() != self.slab_count {
                    return Err(SupportDopAabb3ValidationError::MissingSlabEvidence);
                }
                if boundary {
                    SupportDopRelation::BoundaryTouch
                } else {
                    SupportDopRelation::ConservativeOverlap
                }
            }
        };

        if derived_relation == self.relation {
            Ok(())
        } else {
            Err(SupportDopAabb3ValidationError::RelationMismatch)
        }
    }

    /// Replay this report against a source DOP and AABB bounds.
    pub fn validate_against_sources(
        &self,
        dop: &SupportDop3,
        min: &Point3,
        max: &Point3,
    ) -> Result<(), SupportDopAabb3ValidationError> {
        self.validate()?;
        match dop.classify_aabb3_report(min, max) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(SupportDopAabb3ValidationError::SourceReplayMismatch),
        }
    }
}

/// Structural inconsistency in a retained support-DOP/plane report.
///
/// The report validates a k-DOP/plane decision as two exact halfspace
/// feasibility queries: retained-DOP points satisfying the query plane's
/// `<= 0` side, and retained-DOP points satisfying the opposite closed side.
/// This is the fixed-dimensional LP view retained at the object-evidence
/// boundary. Individual slab carriers follow the k-DOP support-slab model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopPlane3ValidationError {
    /// An empty retained DOP did not report the structural degenerate relation.
    EmptyDopRelationMismatch,
    /// The retained slab halfspaces are not exactly two per source slab.
    SlabHalfspaceCountMismatch,
    /// The retained carrier feasibility report did not replay.
    CarrierFeasibilityMismatch,
    /// A side feasibility report was missing for a feasible retained DOP.
    MissingSideFeasibility,
    /// A side feasibility report was present for an infeasible retained DOP.
    UnexpectedSideFeasibility,
    /// The retained below-side feasibility report did not replay.
    BelowFeasibilityMismatch,
    /// The retained above-side feasibility report did not replay.
    AboveFeasibilityMismatch,
    /// The retained feasibility statuses derive a different coarse relation.
    RelationMismatch,
    /// Recomputing from source geometry did not reproduce this report.
    SourceReplayMismatch,
}

/// Report-bearing exact support-DOP/oriented-plane classification.
///
/// Slabs are lowered to halfspaces with the convention `normal . point + offset
/// <= 0`: each retained support interval `min <= axis . point <= max` becomes
/// `axis . point - max <= 0` and `-axis . point + min <= 0`. The report stores
/// those exact halfspaces plus feasibility reports for the carrier and both
/// closed query-plane sides. A relation is therefore a replayable consequence
/// of exact witnesses or exact infeasibility certificates, not a float
/// broad-phase guess.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopPlane3Report {
    /// Coarse relation between the retained DOP and the oriented plane.
    pub relation: SupportDopPlaneRelation,
    /// Number of slabs retained by the source DOP.
    pub slab_count: usize,
    /// Query plane used to form the closed side feasibility systems.
    pub plane: Plane3,
    /// Exact halfspaces produced from the retained support slabs.
    pub slab_halfspaces: Vec<Plane3>,
    /// Feasibility of the retained DOP carrier itself.
    pub carrier_feasibility: Option<HalfspaceFeasibilityReport>,
    /// Feasibility of `DOP ∩ {plane.normal . point + plane.offset <= 0}`.
    pub below_feasibility: Option<HalfspaceFeasibilityReport>,
    /// Feasibility of `DOP ∩ {plane.normal . point + plane.offset >= 0}`.
    pub above_feasibility: Option<HalfspaceFeasibilityReport>,
}

impl SupportDopPlane3Report {
    /// Validate retained halfspace evidence and the derived coarse relation.
    pub fn validate(&self) -> Result<(), SupportDopPlane3ValidationError> {
        if self.slab_count == 0 {
            return if self.relation == SupportDopPlaneRelation::Degenerate
                && self.slab_halfspaces.is_empty()
                && self.carrier_feasibility.is_none()
                && self.below_feasibility.is_none()
                && self.above_feasibility.is_none()
            {
                Ok(())
            } else {
                Err(SupportDopPlane3ValidationError::EmptyDopRelationMismatch)
            };
        }

        if self.slab_halfspaces.is_empty() {
            return if self.relation == SupportDopPlaneRelation::Degenerate
                && self.carrier_feasibility.is_none()
                && self.below_feasibility.is_none()
                && self.above_feasibility.is_none()
            {
                Ok(())
            } else {
                Err(SupportDopPlane3ValidationError::SlabHalfspaceCountMismatch)
            };
        }

        if self.slab_halfspaces.len() != self.slab_count * 2 {
            return Err(SupportDopPlane3ValidationError::SlabHalfspaceCountMismatch);
        }

        let carrier = self
            .carrier_feasibility
            .as_ref()
            .ok_or(SupportDopPlane3ValidationError::CarrierFeasibilityMismatch)?;
        if !decided_bool(carrier.validate_against_planes(&self.slab_halfspaces)).unwrap_or(false) {
            return Err(SupportDopPlane3ValidationError::CarrierFeasibilityMismatch);
        }

        if !carrier.is_feasible() {
            if self.below_feasibility.is_some() || self.above_feasibility.is_some() {
                return Err(SupportDopPlane3ValidationError::UnexpectedSideFeasibility);
            }
            return if self.relation == SupportDopPlaneRelation::Degenerate {
                Ok(())
            } else {
                Err(SupportDopPlane3ValidationError::RelationMismatch)
            };
        }

        let below = self
            .below_feasibility
            .as_ref()
            .ok_or(SupportDopPlane3ValidationError::MissingSideFeasibility)?;
        let above = self
            .above_feasibility
            .as_ref()
            .ok_or(SupportDopPlane3ValidationError::MissingSideFeasibility)?;

        let below_planes =
            side_halfspaces(&self.slab_halfspaces, &self.plane, PlaneQuerySide::Below);
        if !decided_bool(below.validate_against_planes(&below_planes)).unwrap_or(false) {
            return Err(SupportDopPlane3ValidationError::BelowFeasibilityMismatch);
        }

        let above_planes =
            side_halfspaces(&self.slab_halfspaces, &self.plane, PlaneQuerySide::Above);
        if !decided_bool(above.validate_against_planes(&above_planes)).unwrap_or(false) {
            return Err(SupportDopPlane3ValidationError::AboveFeasibilityMismatch);
        }

        let derived_relation = support_dop_plane_relation_from_side_feasibility(
            below.is_feasible(),
            above.is_feasible(),
        );
        if derived_relation == self.relation {
            Ok(())
        } else {
            Err(SupportDopPlane3ValidationError::RelationMismatch)
        }
    }

    /// Replay this report against a source DOP and query plane.
    pub fn validate_against_sources(
        &self,
        dop: &SupportDop3,
        plane: &Plane3,
    ) -> Result<(), SupportDopPlane3ValidationError> {
        self.validate()?;
        match dop.classify_plane3_report(plane) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(SupportDopPlane3ValidationError::SourceReplayMismatch),
        }
    }
}

/// Retained exact support k-DOP in 3D.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDop3 {
    slabs: Vec<SupportSlab3>,
    source_point_count: usize,
}

impl SupportDop3 {
    /// Build exact support slabs from axes and source points.
    ///
    /// Every bound is selected by exact Real ordering and records a source
    /// witness index. Empty axis or point lists produce a structurally
    /// degenerate empty carrier instead of guessing a topology result.
    pub fn from_points(axes: &[Point3], points: &[Point3]) -> PredicateOutcome<Self> {
        Self::from_points_with_policy(axes, points, PredicatePolicy)
    }

    /// Build exact support slabs from axes and source points with an explicit
    /// predicate policy.
    pub(crate) fn from_points_with_policy(
        axes: &[Point3],
        points: &[Point3],
        policy: PredicatePolicy,
    ) -> PredicateOutcome<Self> {
        crate::trace_dispatch!("hyperlimit", "support_dop3", "build-from-points");
        if axes.is_empty() || points.is_empty() {
            return PredicateOutcome::decided(
                Self {
                    slabs: Vec::new(),
                    source_point_count: points.len(),
                },
                Certainty::Exact,
                Escalation::Structural,
            );
        }

        let mut slabs = Vec::with_capacity(axes.len());
        let mut certainty = Certainty::Exact;
        let mut stage = Escalation::Structural;
        for axis in axes {
            match build_slab(axis, points, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: slab_certainty,
                    stage: slab_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, slab_certainty, slab_stage);
                    slabs.push(value);
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            }
        }

        PredicateOutcome::decided(
            Self {
                slabs,
                source_point_count: points.len(),
            },
            certainty,
            stage,
        )
    }

    /// Construct a k-DOP from already retained slabs.
    pub fn from_slabs(slabs: Vec<SupportSlab3>) -> Self {
        Self {
            slabs,
            source_point_count: 0,
        }
    }

    /// Return the retained slabs.
    pub fn slabs(&self) -> &[SupportSlab3] {
        &self.slabs
    }

    /// Return the number of source points used by [`Self::from_points`].
    pub const fn source_point_count(&self) -> usize {
        self.source_point_count
    }

    /// Classify a point against every retained slab.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<ConvexPointLocation> {
        self.classify_point_with_policy(point, PredicatePolicy)
    }

    /// Classify a point against every retained slab with an explicit policy.
    ///
    /// The inside convention is inclusive: a point is inside when every exact
    /// projection lies between the retained slab bounds. Boundary status is
    /// reported separately so downstream topology can distinguish strict
    /// interior from support contact.
    pub(crate) fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<ConvexPointLocation> {
        if self.slabs.is_empty() {
            return PredicateOutcome::decided(
                ConvexPointLocation::Degenerate,
                Certainty::Exact,
                Escalation::Structural,
            );
        }

        let mut certainty = Certainty::Exact;
        let mut stage = Escalation::Structural;
        let mut boundary = false;
        for slab in &self.slabs {
            match validate_slab_bounds(slab, policy) {
                PredicateOutcome::Decided {
                    value: true,
                    certainty: value_certainty,
                    stage: value_stage,
                } => absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage),
                PredicateOutcome::Decided {
                    value: false,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    return PredicateOutcome::decided(
                        ConvexPointLocation::Degenerate,
                        certainty,
                        stage,
                    );
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            }
            let value = slab.project_point(point);
            match classify_projection_interval(&value, &slab.min, &slab.max, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    match value {
                        ProjectionIntervalLocation::Below | ProjectionIntervalLocation::Above => {
                            return PredicateOutcome::decided(
                                ConvexPointLocation::Outside,
                                certainty,
                                stage,
                            );
                        }
                        ProjectionIntervalLocation::OnMin | ProjectionIntervalLocation::OnMax => {
                            boundary = true
                        }
                        ProjectionIntervalLocation::Inside => {}
                    }
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            }
        }

        PredicateOutcome::decided(
            if boundary {
                ConvexPointLocation::Boundary
            } else {
                ConvexPointLocation::Inside
            },
            certainty,
            stage,
        )
    }

    /// Classify an AABB by exact projection intervals against each slab.
    pub fn classify_aabb3(
        &self,
        min: &Point3,
        max: &Point3,
    ) -> PredicateOutcome<SupportDopRelation> {
        self.classify_aabb3_with_policy(min, max, PredicatePolicy)
    }

    /// Classify an AABB by exact projection intervals with an explicit policy.
    ///
    /// This is a conservative support-slab relation, not a full constructive
    /// intersection witness. A separated result is exact because one slab axis
    /// proves disjointness. A non-separated result certifies only that all
    /// retained support intervals overlap, which is the reusable bounding-volume
    /// predicate that downstream mesh, voxel, and packing crates can replay.
    pub(crate) fn classify_aabb3_with_policy(
        &self,
        min: &Point3,
        max: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopRelation> {
        match self.classify_aabb3_report_with_policy(min, max, policy) {
            PredicateOutcome::Decided {
                value,
                certainty,
                stage,
            } => PredicateOutcome::decided(value.relation, certainty, stage),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        }
    }

    /// Classify an AABB and retain exact per-slab projection evidence.
    pub fn classify_aabb3_report(
        &self,
        min: &Point3,
        max: &Point3,
    ) -> PredicateOutcome<SupportDopAabb3Report> {
        self.classify_aabb3_report_with_policy(min, max, PredicatePolicy)
    }

    /// Classify an AABB with an explicit policy and retain replayable evidence.
    ///
    /// This is the report-bearing form of [`Self::classify_aabb3_with_policy`].
    /// It records the exact support interval of the query box on each visited
    /// k-DOP axis and stops at the first terminal slab, matching the coarse
    /// classifier's scheduling while preserving object-level evidence.
    pub(crate) fn classify_aabb3_report_with_policy(
        &self,
        min: &Point3,
        max: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopAabb3Report> {
        if self.slabs.is_empty() {
            return PredicateOutcome::decided(
                SupportDopAabb3Report {
                    relation: SupportDopRelation::Degenerate,
                    slab_count: 0,
                    terminal_slab: None,
                    slab_reports: Vec::new(),
                },
                Certainty::Exact,
                Escalation::Structural,
            );
        }

        let mut certainty = Certainty::Exact;
        let mut stage = Escalation::Structural;
        let mut boundary = false;
        let mut slab_reports = Vec::with_capacity(self.slabs.len());
        for (slab_index, slab) in self.slabs.iter().enumerate() {
            match validate_slab_bounds(slab, policy) {
                PredicateOutcome::Decided {
                    value: true,
                    certainty: value_certainty,
                    stage: value_stage,
                } => absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage),
                PredicateOutcome::Decided {
                    value: false,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    slab_reports.push(SupportDopAabb3SlabReport {
                        slab_index,
                        slab: slab.clone(),
                        query_min: None,
                        query_max: None,
                        relation: SupportDopRelation::Degenerate,
                    });
                    return PredicateOutcome::decided(
                        SupportDopAabb3Report {
                            relation: SupportDopRelation::Degenerate,
                            slab_count: self.slabs.len(),
                            terminal_slab: Some(slab_index),
                            slab_reports,
                        },
                        certainty,
                        stage,
                    );
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            }
            let (query_min, query_max, interval_certainty, interval_stage) =
                match project_aabb3_on_axis(&slab.axis, min, max, policy) {
                    PredicateOutcome::Decided {
                        value,
                        certainty: value_certainty,
                        stage: value_stage,
                    } => (value.0, value.1, value_certainty, value_stage),
                    PredicateOutcome::Unknown { needed, stage } => {
                        return PredicateOutcome::unknown(needed, stage);
                    }
                };
            absorb_trace(
                &mut certainty,
                &mut stage,
                interval_certainty,
                interval_stage,
            );

            match classify_interval_overlap(&query_min, &query_max, &slab.min, &slab.max, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    slab_reports.push(SupportDopAabb3SlabReport {
                        slab_index,
                        slab: slab.clone(),
                        query_min: Some(query_min),
                        query_max: Some(query_max),
                        relation: value,
                    });
                    match value {
                        SupportDopRelation::Separated => {
                            return PredicateOutcome::decided(
                                SupportDopAabb3Report {
                                    relation: value,
                                    slab_count: self.slabs.len(),
                                    terminal_slab: Some(slab_index),
                                    slab_reports,
                                },
                                certainty,
                                stage,
                            );
                        }
                        SupportDopRelation::Degenerate => {
                            return PredicateOutcome::decided(
                                SupportDopAabb3Report {
                                    relation: value,
                                    slab_count: self.slabs.len(),
                                    terminal_slab: Some(slab_index),
                                    slab_reports,
                                },
                                certainty,
                                stage,
                            );
                        }
                        SupportDopRelation::BoundaryTouch => boundary = true,
                        SupportDopRelation::ConservativeOverlap => {}
                    }
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            }
        }

        PredicateOutcome::decided(
            SupportDopAabb3Report {
                relation: if boundary {
                    SupportDopRelation::BoundaryTouch
                } else {
                    SupportDopRelation::ConservativeOverlap
                },
                slab_count: self.slabs.len(),
                terminal_slab: None,
                slab_reports,
            },
            certainty,
            stage,
        )
    }

    /// Classify this retained DOP relative to an oriented plane.
    pub fn classify_plane3(&self, plane: &Plane3) -> PredicateOutcome<SupportDopPlaneRelation> {
        self.classify_plane3_with_policy(plane, PredicatePolicy)
    }

    /// Classify this retained DOP relative to an oriented plane with a policy.
    pub(crate) fn classify_plane3_with_policy(
        &self,
        plane: &Plane3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopPlaneRelation> {
        match self.classify_plane3_report_with_policy(plane, policy) {
            PredicateOutcome::Decided {
                value,
                certainty,
                stage,
            } => PredicateOutcome::decided(value.relation, certainty, stage),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        }
    }

    /// Classify this retained DOP relative to an oriented plane and retain
    /// exact halfspace feasibility evidence.
    pub fn classify_plane3_report(
        &self,
        plane: &Plane3,
    ) -> PredicateOutcome<SupportDopPlane3Report> {
        self.classify_plane3_report_with_policy(plane, PredicatePolicy)
    }

    /// Classify this retained DOP relative to an oriented plane with an
    /// explicit policy and retain exact feasibility evidence.
    ///
    /// This is a report-bearing k-DOP/plane broad-phase predicate. The retained
    /// support slabs are first replayed as exact halfspaces, then the carrier
    /// and both closed sides of the query plane are certified by the
    /// halfspace-feasibility predicate. The result follows the same
    /// report-first discipline as [`Self::classify_aabb3_report_with_policy`].
    pub(crate) fn classify_plane3_report_with_policy(
        &self,
        plane: &Plane3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopPlane3Report> {
        if self.slabs.is_empty() {
            return PredicateOutcome::decided(
                SupportDopPlane3Report {
                    relation: SupportDopPlaneRelation::Degenerate,
                    slab_count: 0,
                    plane: plane.clone(),
                    slab_halfspaces: Vec::new(),
                    carrier_feasibility: None,
                    below_feasibility: None,
                    above_feasibility: None,
                },
                Certainty::Exact,
                Escalation::Structural,
            );
        }

        let (slab_halfspaces, mut certainty, mut stage) =
            match support_dop_slab_halfspaces(&self.slabs, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty,
                    stage,
                } => (value, certainty, stage),
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };

        if slab_halfspaces.is_empty() {
            return PredicateOutcome::decided(
                SupportDopPlane3Report {
                    relation: SupportDopPlaneRelation::Degenerate,
                    slab_count: self.slabs.len(),
                    plane: plane.clone(),
                    slab_halfspaces,
                    carrier_feasibility: None,
                    below_feasibility: None,
                    above_feasibility: None,
                },
                certainty,
                stage,
            );
        }

        let carrier_feasibility =
            match classify_halfspace_feasibility3_with_policy(&slab_halfspaces, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    value
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };

        if carrier_feasibility.status == HalfspaceFeasibility::Infeasible {
            return PredicateOutcome::decided(
                SupportDopPlane3Report {
                    relation: SupportDopPlaneRelation::Degenerate,
                    slab_count: self.slabs.len(),
                    plane: plane.clone(),
                    slab_halfspaces,
                    carrier_feasibility: Some(carrier_feasibility),
                    below_feasibility: None,
                    above_feasibility: None,
                },
                certainty,
                stage,
            );
        }

        let below_planes = side_halfspaces(&slab_halfspaces, plane, PlaneQuerySide::Below);
        let below_feasibility =
            match classify_halfspace_feasibility3_with_policy(&below_planes, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    value
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };

        let above_planes = side_halfspaces(&slab_halfspaces, plane, PlaneQuerySide::Above);
        let above_feasibility =
            match classify_halfspace_feasibility3_with_policy(&above_planes, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: value_certainty,
                    stage: value_stage,
                } => {
                    absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                    value
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return PredicateOutcome::unknown(needed, stage);
                }
            };

        let relation = support_dop_plane_relation_from_side_feasibility(
            below_feasibility.is_feasible(),
            above_feasibility.is_feasible(),
        );
        PredicateOutcome::decided(
            SupportDopPlane3Report {
                relation,
                slab_count: self.slabs.len(),
                plane: plane.clone(),
                slab_halfspaces,
                carrier_feasibility: Some(carrier_feasibility),
                below_feasibility: Some(below_feasibility),
                above_feasibility: Some(above_feasibility),
            },
            certainty,
            stage,
        )
    }
}

/// Build an exact support k-DOP from axis directions and source points.
pub fn support_dop3_from_points(
    axes: &[Point3],
    points: &[Point3],
) -> PredicateOutcome<SupportDop3> {
    SupportDop3::from_points(axes, points)
}

/// Build a witnessed exact support k-DOP from integer axis directions and
/// source points.
pub fn witnessed_support_dop3_from_points(
    points: &[Point3],
    axes: &[SupportDopAxis3],
) -> Result<WitnessedSupportDop3, SupportDopValidationError> {
    WitnessedSupportDop3::from_points(points, axes)
}

fn compute_witnessed_slab(
    points: &[Point3],
    axis: SupportDopAxis3,
) -> Result<WitnessedSupportSlab3, SupportDopValidationError> {
    axis.validate()?;
    let first = points
        .first()
        .ok_or(SupportDopValidationError::EmptyPointSet)?;
    let first_distance = support_distance_on_integer_axis(first, axis);
    let mut min = support_witness(0, first, first_distance.clone());
    let mut max = support_witness(0, first, first_distance);
    for (vertex, point) in points.iter().enumerate().skip(1) {
        let distance = support_distance_on_integer_axis(point, axis);
        if matches!(
            compare_real_default(&distance, &min.distance),
            Some(Ordering::Less)
        ) {
            min = support_witness(vertex, point, distance.clone());
        }
        if matches!(
            compare_real_default(&distance, &max.distance),
            Some(Ordering::Greater)
        ) {
            max = support_witness(vertex, point, distance);
        }
    }
    Ok(WitnessedSupportSlab3 { axis, min, max })
}

fn validate_witnessed_slab(
    slab: &WitnessedSupportSlab3,
    points: &[Point3],
) -> Result<(), SupportDopValidationError> {
    slab.axis.validate()?;
    validate_support_witness(&slab.min, slab.axis, points)?;
    validate_support_witness(&slab.max, slab.axis, points)?;
    match compare_real_default(&slab.min.distance, &slab.max.distance) {
        Some(Ordering::Less | Ordering::Equal) => {}
        Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMinimal),
        None => return Err(SupportDopValidationError::UnknownSlabOrder),
    }
    for point in points {
        let distance = support_distance_on_integer_axis(point, slab.axis);
        match compare_real_default(&distance, &slab.min.distance) {
            Some(Ordering::Less) => return Err(SupportDopValidationError::WitnessNotMinimal),
            Some(Ordering::Equal | Ordering::Greater) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
        match compare_real_default(&distance, &slab.max.distance) {
            Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMaximal),
            Some(Ordering::Less | Ordering::Equal) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
    }
    Ok(())
}

fn validate_support_witness(
    retained: &SupportWitness3,
    axis: SupportDopAxis3,
    points: &[Point3],
) -> Result<(), SupportDopValidationError> {
    let point = points
        .get(retained.vertex)
        .ok_or(SupportDopValidationError::WitnessOutOfRange)?;
    if point != &retained.point {
        return Err(SupportDopValidationError::WitnessPointMismatch);
    }
    let replay_distance = support_distance_on_integer_axis(point, axis);
    if !matches!(
        compare_real_default(&replay_distance, &retained.distance),
        Some(Ordering::Equal)
    ) {
        return Err(SupportDopValidationError::WitnessDistanceMismatch);
    }
    Ok(())
}

fn support_witness(vertex: usize, point: &Point3, distance: Real) -> SupportWitness3 {
    SupportWitness3 {
        vertex,
        point: point.clone(),
        distance,
    }
}

fn support_distance_on_integer_axis(point: &Point3, axis: SupportDopAxis3) -> Real {
    point.x.clone() * Real::from(axis.direction[0])
        + point.y.clone() * Real::from(axis.direction[1])
        + point.z.clone() * Real::from(axis.direction[2])
}

fn compare_real_default(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals_with_policy(left, right, PredicatePolicy).value()
}

fn validate_slab_bounds(slab: &SupportSlab3, policy: PredicatePolicy) -> PredicateOutcome<bool> {
    match compare_reals_with_policy(&slab.min, &slab.max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value != Ordering::Greater, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn decided_bool(outcome: PredicateOutcome<bool>) -> Option<bool> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Some(value),
        PredicateOutcome::Unknown { .. } => None,
    }
}

fn support_dop_slab_halfspaces(
    slabs: &[SupportSlab3],
    policy: PredicatePolicy,
) -> PredicateOutcome<Vec<Plane3>> {
    let mut halfspaces = Vec::with_capacity(slabs.len() * 2);
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    for slab in slabs {
        match validate_slab_bounds(slab, policy) {
            PredicateOutcome::Decided {
                value: true,
                certainty: value_certainty,
                stage: value_stage,
            } => absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage),
            PredicateOutcome::Decided {
                value: false,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                return PredicateOutcome::decided(Vec::new(), certainty, stage);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }

        halfspaces.push(Plane3::new(slab.axis.clone(), negate(&slab.max)));
        halfspaces.push(Plane3::new(negate_point(&slab.axis), slab.min.clone()));
    }

    PredicateOutcome::decided(halfspaces, certainty, stage)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PlaneQuerySide {
    Below,
    Above,
}

fn side_halfspaces(
    slab_halfspaces: &[Plane3],
    plane: &Plane3,
    side: PlaneQuerySide,
) -> Vec<Plane3> {
    let mut halfspaces = Vec::with_capacity(slab_halfspaces.len() + 1);
    halfspaces.extend_from_slice(slab_halfspaces);
    match side {
        PlaneQuerySide::Below => halfspaces.push(plane.clone()),
        PlaneQuerySide::Above => halfspaces.push(negate_plane(plane)),
    }
    halfspaces
}

fn support_dop_plane_relation_from_side_feasibility(
    below_feasible: bool,
    above_feasible: bool,
) -> SupportDopPlaneRelation {
    match (below_feasible, above_feasible) {
        (true, true) => SupportDopPlaneRelation::Intersecting,
        (true, false) => SupportDopPlaneRelation::Below,
        (false, true) => SupportDopPlaneRelation::Above,
        (false, false) => SupportDopPlaneRelation::Degenerate,
    }
}

fn negate_plane(plane: &Plane3) -> Plane3 {
    Plane3::new(negate_point(&plane.normal), negate(&plane.offset))
}

fn negate_point(point: &Point3) -> Point3 {
    Point3::new(negate(&point.x), negate(&point.y), negate(&point.z))
}

fn negate(value: &Real) -> Real {
    sub_ref(&Real::from(0), value)
}

fn build_slab(
    axis: &Point3,
    points: &[Point3],
    policy: PredicatePolicy,
) -> PredicateOutcome<SupportSlab3> {
    let mut min = project_point_on_axis(axis, &points[0]);
    let mut max = min.clone();
    let mut min_witness = 0_usize;
    let mut max_witness = 0_usize;
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    for (index, point) in points.iter().enumerate().skip(1) {
        let projection = project_point_on_axis(axis, point);
        match compare_reals_with_policy(&projection, &min, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                if value == Ordering::Less {
                    min = projection.clone();
                    min_witness = index;
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
        match compare_reals_with_policy(&projection, &max, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                if value == Ordering::Greater {
                    max = projection;
                    max_witness = index;
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }

    PredicateOutcome::decided(
        SupportSlab3 {
            axis: axis.clone(),
            min,
            max,
            min_witness: Some(min_witness),
            max_witness: Some(max_witness),
        },
        certainty,
        stage,
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ProjectionIntervalLocation {
    Below,
    OnMin,
    Inside,
    OnMax,
    Above,
}

fn classify_projection_interval(
    value: &Real,
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<ProjectionIntervalLocation> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let min_cmp = match compare_reals_with_policy(value, min, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if min_cmp == Ordering::Less {
        return PredicateOutcome::decided(ProjectionIntervalLocation::Below, certainty, stage);
    }

    let max_cmp = match compare_reals_with_policy(value, max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let location = match (min_cmp, max_cmp) {
        (Ordering::Equal, _) => ProjectionIntervalLocation::OnMin,
        (_, Ordering::Equal) => ProjectionIntervalLocation::OnMax,
        (_, Ordering::Greater) => ProjectionIntervalLocation::Above,
        _ => ProjectionIntervalLocation::Inside,
    };
    PredicateOutcome::decided(location, certainty, stage)
}

fn classify_interval_overlap(
    query_min: &Real,
    query_max: &Real,
    slab_min: &Real,
    slab_max: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<SupportDopRelation> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    let query_before = match compare_reals_with_policy(query_max, slab_min, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if query_before == Ordering::Less {
        return PredicateOutcome::decided(SupportDopRelation::Separated, certainty, stage);
    }

    let query_after = match compare_reals_with_policy(query_min, slab_max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if query_after == Ordering::Greater {
        return PredicateOutcome::decided(SupportDopRelation::Separated, certainty, stage);
    }

    let relation = if query_before == Ordering::Equal || query_after == Ordering::Equal {
        SupportDopRelation::BoundaryTouch
    } else {
        SupportDopRelation::ConservativeOverlap
    };
    PredicateOutcome::decided(relation, certainty, stage)
}

fn project_aabb3_on_axis(
    axis: &Point3,
    min: &Point3,
    max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<(Real, Real)> {
    crate::trace_dispatch!("hyperlimit", "support_dop3", "project-aabb3");
    let axis_coords = [&axis.x, &axis.y, &axis.z];
    let box_min = [&min.x, &min.y, &min.z];
    let box_max = [&max.x, &max.y, &max.z];
    let mut lower_coords = box_min;
    let mut upper_coords = box_max;
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    for index in 0..3 {
        let min_term = mul(axis_coords[index], box_min[index]);
        let max_term = mul(axis_coords[index], box_max[index]);
        match compare_reals_with_policy(&min_term, &max_term, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                absorb_trace(&mut certainty, &mut stage, value_certainty, value_stage);
                if value == Ordering::Greater {
                    lower_coords[index] = box_max[index];
                    upper_coords[index] = box_min[index];
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }

    let lower = project_coords_on_axis(axis, lower_coords);
    let upper = project_coords_on_axis(axis, upper_coords);
    PredicateOutcome::decided((lower, upper), certainty, stage)
}

fn project_point_on_axis(axis: &Point3, point: &Point3) -> Real {
    project_coords_on_axis(axis, [&point.x, &point.y, &point.z])
}

fn project_coords_on_axis(axis: &Point3, coords: [&Real; 3]) -> Real {
    let x = mul(&axis.x, coords[0]);
    let y = mul(&axis.y, coords[1]);
    let z = mul(&axis.z, coords[2]);
    add(&add(&x, &y), &z)
}

fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

fn absorb_trace(
    certainty: &mut Certainty,
    stage: &mut Escalation,
    value_certainty: Certainty,
    value_stage: Escalation,
) {
    *certainty = max_certainty(*certainty, value_certainty);
    *stage = max_stage(*stage, value_stage);
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}
