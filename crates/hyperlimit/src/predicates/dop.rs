//! Directional support slabs, indexed witnesses, and conservative projection classifiers.
//!
//! Slabs encode `min <= axis . point <= max` using unnormalized functionals.
//! General carriers retain bounds and optional indices; witnessed carriers also
//! retain point copies and caller-declared projection margins. Bounds are exact
//! scalar expressions, while selection/classification certainty depends on policy.
//! General constructor/query outcomes carry receipts, but carriers and report
//! records do not retain them. Witnessed Result APIs and validators discard
//! consumed receipts and map some unknown comparisons to ordinary errors.
//!
//! Box queries test per-axis interval overlap; nonseparation need not imply a
//! common spatial point. Plane queries use carrier and closed-side halfspace
//! feasibility, inheriting its certificate limitations. Empty general carriers
//! classify as Degenerate. No total-work budget or typed allocation-exhaustion
//! contract is imposed here; refresh performs a full validation after updates.

use crate::predicate::PredicatePolicy;
use crate::predicates::decision::max_stage;
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

/// Integer coefficients of an unnormalized support functional `axis . point`.
///
/// Scaling an axis scales its projection bounds and margins; these are not
/// Euclidean distances unless the axis is unit length. Construction stores any
/// coefficients, including zero; [`Self::validate`] checks nonzeroness separately.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopAxis3 {
    /// Integer coefficients of the support direction.
    pub direction: [i64; 3],
}

impl SupportDopAxis3 {
    /// Store integer coefficients without checking nonzeroness or normalizing the direction.
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

    /// Convert coefficients exactly to the general point-vector carrier, without validation.
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

/// Caller-declared origin of a support projection margin.
///
/// These tags record a claimed loss model; they do not measure adapter error or
/// prove that a supplied margin bounds it. Exact finite-float import alone does
/// not require a lossy tag.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopExpansionKind {
    /// Declares no source-loss expansion; the tag itself does not prove fidelity.
    None,
    /// Declares loss through an adapter; no numeric error is inferred from the tag.
    LossyAdapter,
    /// Declares integer-grid rounding; callers supply a margin in projection units.
    IntegerGridRounding,
}

/// Caller-supplied expansion metadata in unnormalized projection units.
///
/// All slabs use the same scalar margin. Validation checks sign/count consistency,
/// not whether the margin encloses source loss. Fields are mutable and carry no
/// certainty receipt; revalidate after changes.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopExpansionReport {
    /// Expansion source category.
    pub kind: SupportDopExpansionKind,
    /// Number of axes covered by this report.
    pub axis_count: usize,
    /// Declared expanded count: zero for None-kind and axis_count for other kinds.
    pub expanded_slabs: usize,
    /// Scalar projection margin subtracted/added on each side; sign and adequacy are unchecked on storage.
    pub expansion: Real,
}

impl SupportDopExpansionReport {
    /// Build a zero-margin report with no expanded slabs; this records a claim, not a proof of source fidelity.
    pub fn exact(axis_count: usize) -> Self {
        Self {
            kind: SupportDopExpansionKind::None,
            axis_count,
            expanded_slabs: 0,
            expansion: Real::from(0),
        }
    }

    /// Store a grid-rounding margin for every axis without validating its sign or adequacy.
    pub fn integer_grid_rounding(axis_count: usize, expansion: Real) -> Self {
        Self {
            kind: SupportDopExpansionKind::IntegerGridRounding,
            axis_count,
            expanded_slabs: axis_count,
            expansion,
        }
    }

    /// Mark every axis as lossy with a zero numeric margin; no error bound is inferred.
    pub fn lossy_adapter(axis_count: usize) -> Self {
        Self {
            kind: SupportDopExpansionKind::LossyAdapter,
            axis_count,
            expanded_slabs: axis_count,
            expansion: Real::from(0),
        }
    }

    /// Check margin sign and kind/count consistency using the supplied policy.
    ///
    /// An unresolved nonnegativity comparison returns NegativeExpansion. None-kind
    /// requires zero margin and zero expanded count; other kinds require all axes
    /// counted as expanded. Decided approximate comparisons are accepted and their
    /// receipts discarded. Success does not validate an external loss bound.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), SupportDopValidationError> {
        if !matches!(
            compare_real_with_policy(&self.expansion, &Real::from(0), policy),
            Some(Ordering::Greater | Ordering::Equal)
        ) {
            return Err(SupportDopValidationError::NegativeExpansion);
        }
        match self.kind {
            SupportDopExpansionKind::None => {
                if self.expanded_slabs != 0
                    || !matches!(
                        compare_real_with_policy(&self.expansion, &Real::from(0), policy),
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

/// Retained source index, point copy, and unnormalized projection expression.
///
/// Public fields can be inconsistent. This record alone proves neither source
/// identity nor extremality; witnessed-DOP validation checks those relationships
/// under its supplied policy.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportWitness3 {
    /// Source point index that witnesses the retained support distance.
    pub vertex: usize,
    /// Exact point copied from the source point set at construction time.
    pub point: Point3,
    /// Retained axis dot-product expression, not a normalized Euclidean distance.
    pub distance: Real,
}

/// Integer-axis min/max records with source point copies.
///
/// No constructor invariant is enforced by this public record. The records carry
/// no selection receipts; use the containing DOP's validation against source points.
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
    /// Subtract the supplied scalar margin from the retained minimum projection.
    ///
    /// Does not validate the witness, report kind/counts, margin sign, or loss bound;
    /// uses the margin even when the report declares no expanded slabs.
    pub fn conservative_min_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.min.distance.clone() - expansion.expansion.clone()
    }

    /// Add the supplied scalar margin to the retained maximum projection.
    ///
    /// Does not validate the witness, report kind/counts, margin sign, or loss bound;
    /// uses the margin even when the report declares no expanded slabs.
    pub fn conservative_max_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.max.distance.clone() + expansion.expansion.clone()
    }

    /// Copy the axis, raw bounds, and witness indices to a classifier slab.
    ///
    /// Point copies and expansion metadata are omitted, and no margin is applied.
    /// The conversion performs no validation and does not establish source fidelity.
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

/// Support bounds with indexed source-point copies and declared expansion metadata.
///
/// Construction and validation consume policy decisions but return ordinary
/// Results without certainty receipts. Exact scalar storage alone does not prove
/// exact extremum selection under an approximate policy. Public mutation can
/// invalidate witness relationships; validation requires the source points.
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
    /// Build raw witnessed slabs, then validate them against all supplied source points.
    ///
    /// Requires nonempty points/axes, nonzero integer directions, matching expansion
    /// axis count, and consistent expansion metadata. Slabs are scanned and then
    /// rechecked for extremality. Unresolved comparisons become validation errors;
    /// decided comparisons discard certainty. The report's margin is stored, not
    /// applied to raw bounds, and its adequacy for any adapter loss is not proved.
    pub fn from_points_with_expansion(
        points: &[Point3],
        axes: &[SupportDopAxis3],
        expansion: SupportDopExpansionReport,
        policy: PredicatePolicy,
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
        expansion.validate(policy)?;
        let slabs = axes
            .iter()
            .map(|&axis| compute_witnessed_slab(points, axis, policy))
            .collect::<Result<Vec<_>, _>>()?;
        let support = Self {
            vertex_count: points.len(),
            slabs,
            expansion,
        };
        support.validate_against_points(points, policy)?;
        Ok(support)
    }

    /// Build witnessed slabs with zero-margin metadata via [`Self::from_points_with_expansion`].
    ///
    /// Inherits its validation and receipt limitations; the no-expansion tag is a
    /// caller-relative source claim, not an independent fidelity certificate.
    pub fn from_points(
        points: &[Point3],
        axes: &[SupportDopAxis3],
        policy: PredicatePolicy,
    ) -> Result<Self, SupportDopValidationError> {
        Self::from_points_with_expansion(
            points,
            axes,
            SupportDopExpansionReport::exact(axes.len()),
            policy,
        )
    }

    /// Convert raw slabs to a predicate carrier without applying expansion margins.
    ///
    /// Drops witness point copies and expansion metadata. The resulting carrier's
    /// source point count is zero, as for any [`SupportDop3::from_slabs`] construction.
    /// No validation or certainty transport is performed.
    pub fn to_support_dop3(&self) -> SupportDop3 {
        SupportDop3::from_slabs(
            self.slabs
                .iter()
                .map(WitnessedSupportSlab3::to_support_slab3)
                .collect(),
        )
    }

    /// Recheck counts, expansion consistency, witness identity/projections, and all extrema.
    ///
    /// Uses point PartialEq for retained copies and policy comparisons for projection
    /// relations. Unknown comparisons map to specific validation errors; approximate
    /// decisions can pass without a returned receipt. Every point is examined for
    /// each slab. Expansion adequacy relative to external source loss is not checked.
    pub fn validate_against_points(
        &self,
        points: &[Point3],
        policy: PredicatePolicy,
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
        self.expansion.validate(policy)?;
        for slab in &self.slabs {
            validate_witnessed_slab(slab, points, policy)?;
        }
        Ok(())
    }

    /// Update slabs for supplied changed indices, then validate against the full point set.
    ///
    /// A listed extremum witness triggers a full axis rebuild; other listed points
    /// can extend bounds. Duplicate indices are counted and revisited. Unresolved
    /// extension comparisons do not update an extremum, but the final full validation
    /// can reject unresolved or missed bounds. Final validation still scans every
    /// point for every slab, so work is not bounded only by the changed subset.
    ///
    /// Mutations are not transactional: an error can leave earlier slab updates in
    /// place. Counts/indices are checked first; expansion consistency and retained
    /// witness validity are rechecked after updates. Success returns counters without
    /// certainty receipts or a proof that an expansion margin covers external loss.
    pub fn refresh_for_changed_vertices(
        &mut self,
        points: &[Point3],
        changed_vertices: &[usize],
        policy: PredicatePolicy,
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
                *slab = compute_witnessed_slab(points, slab.axis, policy)?;
                report.axes_rebuilt += 1;
                report.invalidated_witness_axes += 1;
                continue;
            }

            let mut extended = false;
            for &vertex in changed_vertices {
                let distance = support_distance_on_integer_axis(&points[vertex], slab.axis);
                if matches!(
                    compare_real_with_policy(&distance, &slab.min.distance, policy),
                    Some(Ordering::Less)
                ) {
                    slab.min = support_witness(vertex, &points[vertex], distance.clone());
                    extended = true;
                }
                if matches!(
                    compare_real_with_policy(&distance, &slab.max.distance, policy),
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

        self.validate_against_points(points, policy)?;
        Ok(report)
    }
}

/// Counts from a completed witnessed-DOP refresh, not a work or timing measurement.
///
/// Rebuilt, extended, and unchanged axes partition the visited slabs on success.
/// Final full-source validation is additional work and is not counted here.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopRefreshReport {
    /// Length of the supplied index list, including duplicates.
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

/// Witnessed-DOP construction, refresh, or validation failure.
///
/// Some variants also represent undecided comparisons, as documented below. This
/// Result interface does not return the underlying predicate reason, stage, or
/// certainty, and successful approximate decisions can lose their qualification.
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
    /// The retained point copy differs under Point3 PartialEq from its indexed source.
    WitnessPointMismatch,
    /// Recomputed projection is unequal or its equality is undecided under the policy.
    WitnessDistanceMismatch,
    /// A min witness is not minimal for its axis.
    WitnessNotMinimal,
    /// A max witness is not maximal for its axis.
    WitnessNotMaximal,
    /// A bound or point-versus-extremum comparison did not decide.
    UnknownSlabOrder,
    /// Margin comparison is negative or unresolved under the supplied policy.
    NegativeExpansion,
    /// Expansion kind, slab count, or zero-expansion state is contradictory.
    ExpansionKindMismatch,
    /// A changed point index is outside the source point slice.
    ChangedVertexOutOfRange,
}

/// One unnormalized directional interval `min <= axis . point <= max`.
///
/// Public bounds and optional source indices carry no source points or certainty
/// receipts. General slab classification checks bound ordering, not axis
/// nonzeroness, witness validity, or joint feasibility of all slabs.
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
    /// Store an axis and explicit bounds with no witness indices or validation.
    ///
    /// Classification checks bound ordering as visited, but does not reject a zero
    /// axis. Use [`SupportDop3::from_points`] to select bounds and retain source indices;
    /// keep that constructor's outcome receipt separately.
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

/// Retained AABB-report inconsistency or failed/undecided replay check.
///
/// Local validation checks interval relations, not that the intervals were produced
/// by a particular source box. Source replay recomputes those relations. Both
/// interfaces discard predicate receipts and can accept approximate decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopAabb3ValidationError {
    /// An empty retained DOP did not report the structural degenerate relation.
    EmptyDopRelationMismatch,
    /// A report item is not the next prefix slab tested by the classifier.
    SlabIndexMismatch,
    /// Bound validity is undecided, or invalid bounds lack a Degenerate relation.
    SlabBoundsInvalid,
    /// A valid retained slab is missing its exact query projection interval.
    MissingQueryInterval,
    /// A degenerate retained slab unexpectedly carries a query projection.
    DegenerateSlabHasQueryInterval,
    /// Retained query order is inverted or undecided.
    QueryIntervalInvalid,
    /// Interval replay is undecided or yields a different per-slab relation.
    SlabRelationMismatch,
    /// The first separating or degenerate slab does not match the terminal id.
    TerminalSlabMismatch,
    /// A non-terminal report does not cover every retained slab.
    MissingSlabEvidence,
    /// The retained slab relations derive a different coarse relation.
    RelationMismatch,
    /// Source recomputation is undecided or its report value differs.
    SourceReplayMismatch,
}

/// Copied slab and selected query interval for one visited projection axis.
///
/// Fields contain no local certainty or source-box identity. They are mutable
/// claims until checked; local validation cannot reconstruct a source box from
/// this record.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopAabb3SlabReport {
    /// Index of the retained slab tested by this report item.
    pub slab_index: usize,
    /// Copied retained slab used by local replay; its source relationship is not stored.
    pub slab: SupportSlab3,
    /// Selected minimum AABB projection; absent when invalid bounds terminate before projection.
    pub query_min: Option<Real>,
    /// Selected maximum AABB projection; absent for an invalid terminal slab.
    pub query_max: Option<Real>,
    /// Relation derived from this slab alone.
    pub relation: SupportDopRelation,
}

/// Visited slab prefix and derived conservative AABB relation.
///
/// A separating or invalid slab terminates the prefix; otherwise every slab is
/// included. The enclosing PredicateOutcome carries operation certainty, which
/// this record does not store. Nonseparation and BoundaryTouch describe interval
/// relations and need not establish a common spatial intersection point.
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
    /// Check retained prefix structure, interval order, and the derived coarse relation.
    ///
    /// No source box is available here: internally consistent but invented projection
    /// intervals can pass. The slab axis and witness indices are not validated.
    /// Unknown comparisons map to validation errors; decided receipts are discarded,
    /// so success under an approximate policy is not an exactness certificate.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), SupportDopAabb3ValidationError> {
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

            let slab_bounds_valid = decided_bool(validate_slab_bounds(&slab_report.slab, policy))
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
            let query_interval_valid = match compare_reals_with_policy(query_min, query_max, policy)
            {
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
                policy,
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

    /// Validate locally, recompute from the supplied DOP/box, and compare report values.
    ///
    /// A failed or unknown recomputation becomes SourceReplayMismatch. The fresh
    /// outcome's certainty/stage are discarded, and report equality does not retain
    /// construction provenance or bind an earlier source allocation.
    pub fn validate_against_sources(
        &self,
        dop: &SupportDop3,
        min: &Point3,
        max: &Point3,
        policy: PredicatePolicy,
    ) -> Result<(), SupportDopAabb3ValidationError> {
        self.validate(policy)?;
        match dop.classify_aabb3_report(min, max, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(SupportDopAabb3ValidationError::SourceReplayMismatch),
        }
    }
}

/// Retained plane-report inconsistency or failed/undecided feasibility validation.
///
/// These errors describe structural and replay checks. They do not guarantee that
/// every infeasible status contains an independently checkable certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopPlane3ValidationError {
    /// An empty retained DOP did not report the structural degenerate relation.
    EmptyDopRelationMismatch,
    /// Halfspace count is not twice the slab count, or the empty sentinel form is inconsistent.
    SlabHalfspaceCountMismatch,
    /// Carrier evidence is absent, false on validation, or undecided.
    CarrierFeasibilityMismatch,
    /// A side feasibility report was missing for a feasible retained DOP.
    MissingSideFeasibility,
    /// A side feasibility report was present for an infeasible retained DOP.
    UnexpectedSideFeasibility,
    /// Below-side evidence validation is false or undecided.
    BelowFeasibilityMismatch,
    /// Above-side evidence validation is false or undecided.
    AboveFeasibilityMismatch,
    /// The retained feasibility statuses derive a different coarse relation.
    RelationMismatch,
    /// Source recomputation is undecided or its report value differs.
    SourceReplayMismatch,
}

/// Halfspace systems and feasibility reports used for a DOP/plane relation.
///
/// Each slab yields `axis . point - max <= 0` and `-axis . point + min <= 0`.
/// Carrier and both closed query sides are tested, so tangency counts as
/// Intersecting. Empty/invalid/infeasible carriers use Degenerate.
///
/// The enclosing outcome carries certainty; this mutable record does not. A
/// halfspace infeasibility report may have no certificate, so these fields alone
/// are not a universal standalone proof of the relation. Local validation inherits
/// that report validator's limitations and discards consumed receipts.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopPlane3Report {
    /// Coarse relation between the retained DOP and the oriented plane.
    pub relation: SupportDopPlaneRelation,
    /// Number of slabs retained by the source DOP.
    pub slab_count: usize,
    /// Query plane used to form the closed side feasibility systems.
    pub plane: Plane3,
    /// Stored halfspaces; local validation checks count, not derivation from a source DOP.
    pub slab_halfspaces: Vec<Plane3>,
    /// Carrier feasibility claim; absent for empty or invalid-bound sentinels.
    pub carrier_feasibility: Option<HalfspaceFeasibilityReport>,
    /// Feasibility of `DOP ∩ {plane.normal . point + plane.offset <= 0}`.
    pub below_feasibility: Option<HalfspaceFeasibilityReport>,
    /// Feasibility of `DOP ∩ {plane.normal . point + plane.offset >= 0}`.
    pub above_feasibility: Option<HalfspaceFeasibilityReport>,
}

impl SupportDopPlane3Report {
    /// Check report shape, delegated feasibility records, and the side-derived relation.
    ///
    /// Accepts the designated empty-halfspace Degenerate form without proving an
    /// invalid source slab. Nonempty systems require two halfspaces per recorded slab,
    /// then delegate carrier/side validation. Infeasible reports without certificates
    /// can pass that delegate. No source DOP is checked; unknown/false validations map
    /// to mismatch errors and decided receipts are discarded.
    ///
    /// The count check computes `slab_count * 2` with ordinary usize arithmetic;
    /// a forged overflowing count can panic in overflow-checking builds.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), SupportDopPlane3ValidationError> {
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
        if !decided_bool(carrier.validate_against_planes(&self.slab_halfspaces, policy))
            .unwrap_or(false)
        {
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
        if !decided_bool(below.validate_against_planes(&below_planes, policy)).unwrap_or(false) {
            return Err(SupportDopPlane3ValidationError::BelowFeasibilityMismatch);
        }

        let above_planes =
            side_halfspaces(&self.slab_halfspaces, &self.plane, PlaneQuerySide::Above);
        if !decided_bool(above.validate_against_planes(&above_planes, policy)).unwrap_or(false) {
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

    /// Validate locally, recompute from the supplied DOP/plane, and compare report values.
    ///
    /// Unknown or unequal recomputation becomes SourceReplayMismatch. The fresh
    /// certainty/stage are discarded; success does not restore the original query's
    /// receipt or independently repair missing feasibility certificates.
    pub fn validate_against_sources(
        &self,
        dop: &SupportDop3,
        plane: &Plane3,
        policy: PredicatePolicy,
    ) -> Result<(), SupportDopPlane3ValidationError> {
        self.validate(policy)?;
        match dop.classify_plane3_report(plane, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(SupportDopPlane3ValidationError::SourceReplayMismatch),
        }
    }
}

/// Ordered support slabs with a source-count annotation.
///
/// Represents their closed intersection, which may be unbounded or infeasible.
/// The empty list is handled as Degenerate by classifiers, not as a universal-set
/// identity. Axes need not be normalized or nonzero. Bounds store exact scalar
/// expressions, but no constructor certainty or source points are retained;
/// classification receipts describe the current query, not source fidelity.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDop3 {
    /// Ordered directional constraints, with no retained constructor receipt.
    slabs: Vec<SupportSlab3>,
    /// Input count from from_points; zero for from_slabs regardless of witness indices.
    source_point_count: usize,
}

impl SupportDop3 {
    /// Select extrema along each supplied axis, retaining first-attaining source indices.
    ///
    /// Empty axes or points produce an empty carrier with Exact/Structural certainty
    /// and the original point count. Nonempty scans aggregate comparison receipts and
    /// propagate the first Unknown. Approximate ordering can select qualified bounds;
    /// retain the returned receipt, since the carrier does not store it. Zero axes
    /// are accepted and joint feasibility is not separately checked.
    pub fn from_points(
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

    /// Store supplied slabs unchecked and set the source-point count to zero.
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

    /// Classify projections against slabs in order, retaining consumed comparison receipts.
    ///
    /// A visited inverted slab gives Degenerate; the first outside projection gives
    /// Outside. Later slabs are not checked after either terminal result. Otherwise
    /// an equality to any bound gives Boundary and all strict inequalities give
    /// Inside. Empty carriers give Exact/Structural Degenerate; an encountered Unknown
    /// propagates without completing the scan.
    ///
    /// Only bound order is validated. With zero/redundant axes, Boundary is a constraint
    /// contact label and need not be the topological boundary of the represented set.
    /// The receipt does not recover qualification lost during carrier construction.
    pub fn classify_point(
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

    /// Return the report classifier's coarse AABB relation with its receipt unchanged.
    ///
    /// A certified separating interval proves disjointness. Nonseparation, including
    /// BoundaryTouch, is only an axis-projection test, not a common intersection
    /// witness. Approximate results retain their qualification. See
    /// [`Self::classify_aabb3_report`] for traversal and input conventions.
    pub fn classify_aabb3(
        &self,
        min: &Point3,
        max: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopRelation> {
        match self.classify_aabb3_report(min, max, policy) {
            PredicateOutcome::Decided {
                value,
                certainty,
                stage,
            } => PredicateOutcome::decided(value.relation, certainty, stage),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        }
    }

    /// Project a coordinatewise box onto visited axes and retain the slab-prefix report.
    ///
    /// The two corner coordinates may be supplied in either order on each axis:
    /// projection selects each coordinate product's minimum and maximum. Slab bounds
    /// are checked before projection. The first inverted or separating slab returns
    /// Degenerate or Separated; later slabs remain unchecked. Empty carriers return
    /// Exact/Structural Degenerate without examining the box.
    ///
    /// Otherwise all slabs are visited: endpoint contact on any interval gives
    /// BoundaryTouch, and remaining overlap gives ConservativeOverlap. Both are
    /// conservative spatial relations. The outcome aggregates consumed receipts and
    /// propagates Unknown; the report itself carries no certainty or source fidelity.
    pub fn classify_aabb3_report(
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
                        SupportDopRelation::Separated | SupportDopRelation::Degenerate => {
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

    /// Return the plane report's coarse relation while preserving its outcome receipt.
    pub fn classify_plane3(
        &self,
        plane: &Plane3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SupportDopPlaneRelation> {
        match self.classify_plane3_report(plane, policy) {
            PredicateOutcome::Decided {
                value,
                certainty,
                stage,
            } => PredicateOutcome::decided(value.relation, certainty, stage),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        }
    }

    /// Lower slabs to closed halfspaces, then classify carrier and both plane-side systems.
    ///
    /// Order is carrier, below side, above side. Both closed sides feasible gives
    /// Intersecting, including tangency; only below/above feasible gives that strict
    /// side. Empty carriers, inverted slab bounds, or infeasible carriers give
    /// Degenerate, with no side queries for those early cases. The query plane and
    /// slab axes are not separately required to have nonzero normals.
    ///
    /// Each consumed decided receipt is accumulated and Unknown propagates. The
    /// returned report omits those receipts and inherits the halfspace solver's
    /// certificate limitations; retain the enclosing outcome for operation certainty.
    pub fn classify_plane3_report(
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

/// Forward to [`SupportDop3::from_points`], preserving its policy outcome and construction limits.
pub fn support_dop3_from_points_with_policy(
    axes: &[Point3],
    points: &[Point3],
    policy: PredicatePolicy,
) -> PredicateOutcome<SupportDop3> {
    SupportDop3::from_points(axes, points, policy)
}

/// Forward to [`WitnessedSupportDop3::from_points`] with integer axes and zero-margin metadata.
///
/// The Result reports validation errors but carries no predicate certainty receipt.
pub fn witnessed_support_dop3_from_points_with_policy(
    points: &[Point3],
    axes: &[SupportDopAxis3],
    policy: PredicatePolicy,
) -> Result<WitnessedSupportDop3, SupportDopValidationError> {
    WitnessedSupportDop3::from_points(points, axes, policy)
}

/// Scan nonempty points for candidate extrema on a nonzero integer axis.
///
/// Only decided strict improvements replace witnesses; equal or unknown comparisons
/// leave the current candidate. Ties therefore retain the first point. This helper
/// must be followed by full witnessed-slab validation before publishing success;
/// it discards receipts and does not itself reject unresolved ordering.
fn compute_witnessed_slab(
    points: &[Point3],
    axis: SupportDopAxis3,
    policy: PredicatePolicy,
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
            compare_real_with_policy(&distance, &min.distance, policy),
            Some(Ordering::Less)
        ) {
            min = support_witness(vertex, point, distance.clone());
        }
        if matches!(
            compare_real_with_policy(&distance, &max.distance, policy),
            Some(Ordering::Greater)
        ) {
            max = support_witness(vertex, point, distance);
        }
    }
    Ok(WitnessedSupportSlab3 { axis, min, max })
}

/// Validate the axis, both source witnesses, bound order, and every point's projection.
///
/// Unknown order becomes UnknownSlabOrder; inverted bounds or an outlying point
/// produce the corresponding extremum error. All comparison receipts are dropped.
fn validate_witnessed_slab(
    slab: &WitnessedSupportSlab3,
    points: &[Point3],
    policy: PredicatePolicy,
) -> Result<(), SupportDopValidationError> {
    slab.axis.validate()?;
    validate_support_witness(&slab.min, slab.axis, points, policy)?;
    validate_support_witness(&slab.max, slab.axis, points, policy)?;
    match compare_real_with_policy(&slab.min.distance, &slab.max.distance, policy) {
        Some(Ordering::Less | Ordering::Equal) => {}
        Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMinimal),
        None => return Err(SupportDopValidationError::UnknownSlabOrder),
    }
    for point in points {
        let distance = support_distance_on_integer_axis(point, slab.axis);
        match compare_real_with_policy(&distance, &slab.min.distance, policy) {
            Some(Ordering::Less) => return Err(SupportDopValidationError::WitnessNotMinimal),
            Some(Ordering::Equal | Ordering::Greater) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
        match compare_real_with_policy(&distance, &slab.max.distance, policy) {
            Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMaximal),
            Some(Ordering::Less | Ordering::Equal) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
    }
    Ok(())
}

/// Check an indexed point copy with PartialEq, then compare its recomputed projection.
///
/// Unknown projection equality is WitnessDistanceMismatch just like decided
/// inequality. No extremum check is made here and comparison receipts are dropped.
fn validate_support_witness(
    retained: &SupportWitness3,
    axis: SupportDopAxis3,
    points: &[Point3],
    policy: PredicatePolicy,
) -> Result<(), SupportDopValidationError> {
    let point = points
        .get(retained.vertex)
        .ok_or(SupportDopValidationError::WitnessOutOfRange)?;
    if point != &retained.point {
        return Err(SupportDopValidationError::WitnessPointMismatch);
    }
    let replay_distance = support_distance_on_integer_axis(point, axis);
    if !matches!(
        compare_real_with_policy(&replay_distance, &retained.distance, policy),
        Some(Ordering::Equal)
    ) {
        return Err(SupportDopValidationError::WitnessDistanceMismatch);
    }
    Ok(())
}

/// Package an index, cloned source point, and supplied projection without checking their relation.
fn support_witness(vertex: usize, point: &Point3, distance: Real) -> SupportWitness3 {
    SupportWitness3 {
        vertex,
        point: point.clone(),
        distance,
    }
}

/// Form the exact dot-product expression with integer coefficients, without axis validation.
fn support_distance_on_integer_axis(point: &Point3, axis: SupportDopAxis3) -> Real {
    point.x.clone() * Real::from(axis.direction[0])
        + point.y.clone() * Real::from(axis.direction[1])
        + point.z.clone() * Real::from(axis.direction[2])
}

/// Extract an optional policy ordering, discarding certainty, stage, and unresolved diagnostics.
fn compare_real_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    compare_reals_with_policy(left, right, policy).value()
}

/// Check only `slab.min <= slab.max`, preserving the comparison outcome; axes and witnesses are unchecked.
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

/// Extract an optional Boolean, discarding certainty, stage, and unresolved diagnostics.
fn decided_bool(outcome: PredicateOutcome<bool>) -> Option<bool> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Some(value),
        PredicateOutcome::Unknown { .. } => None,
    }
}

/// Validate slab orders and emit upper/lower closed halfspaces in source order.
///
/// An inverted slab returns an empty vector sentinel with consumed receipts;
/// Unknown propagates. Empty input also gives an empty vector. Axes and witness
/// indices are unchecked. Accepted slabs produce two planes each.
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

/// Which closed query-plane side to append to the retained slab system.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PlaneQuerySide {
    /// Closed side `normal . point + offset <= 0`.
    Below,
    /// Opposite closed side `normal . point + offset >= 0`.
    Above,
}

/// Clone the slab system and append the plane for Below or its negation for Above.
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

/// Map feasibility of both closed sides to Intersecting, strict Below/Above, or Degenerate.
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

/// Negate normal and offset together, reversing a closed halfspace inequality.
fn negate_plane(plane: &Plane3) -> Plane3 {
    Plane3::new(negate_point(&plane.normal), negate(&plane.offset))
}

/// Negate each point-vector coordinate without interpreting it as a normalized direction.
fn negate_point(point: &Point3) -> Point3 {
    Point3::new(negate(&point.x), negate(&point.y), negate(&point.z))
}

/// Form exact scalar negation as zero minus the supplied value.
fn negate(value: &Real) -> Real {
    sub_ref(&Real::from(0), value)
}

/// Scan a nonempty point slice for first-attaining min/max projection indices.
///
/// Compare minimum before maximum at each point and accumulate consumed receipts;
/// Unknown stops the scan. Panics for empty points, which callers exclude. The
/// axis is accepted without a nonzero check.
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

/// Location of a scalar relative to an assumed ordered closed interval.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ProjectionIntervalLocation {
    /// Strictly less than the lower bound.
    Below,
    /// Equal to the lower bound, including a collapsed interval.
    OnMin,
    /// Strictly between the ordered bounds.
    Inside,
    /// Equal to the upper bound after excluding lower-bound equality.
    OnMax,
    /// Strictly greater than the upper bound.
    Above,
}

/// Compare against the lower bound first, returning Below early, then the upper bound.
///
/// Assumes ordered bounds; equality at both endpoints prefers OnMin. Preserves
/// consumed receipts and propagates Unknown without inferring a relation.
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

/// Test ordered closed intervals for separation, endpoint touch, or overlap.
///
/// Checks query-max versus slab-min before query-min versus slab-max, stopping
/// at separation. Does not validate interval order. Consumed receipts accumulate;
/// Unknown propagates and this helper never returns Degenerate.
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

/// Select coordinatewise product extrema and form an axis-projection interval.
///
/// Corner coordinates may be unordered. Each axis coefficient times each corner
/// coordinate is compared before the lower/upper dot products are formed.
/// Consumes three comparisons, accumulates their receipts, and stops on Unknown.
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

/// Project a point using the shared coordinate-dot-product expression builder.
fn project_point_on_axis(axis: &Point3, point: &Point3) -> Real {
    project_coords_on_axis(axis, [&point.x, &point.y, &point.z])
}

/// Form the exact expression `(axis.x*x + axis.y*y) + axis.z*z` without deciding its sign.
fn project_coords_on_axis(axis: &Point3, coords: [&Real; 3]) -> Real {
    let x = mul(&axis.x, coords[0]);
    let y = mul(&axis.y, coords[1]);
    let z = mul(&axis.z, coords[2]);
    add(&add(&x, &y), &z)
}

/// Delegate scalar multiplication to the owning Real arithmetic helper.
fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

/// Delegate scalar addition to the owning Real arithmetic helper.
fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

/// Accumulate the weakest consumed certainty and latest escalation stage in place.
fn absorb_trace(
    certainty: &mut Certainty,
    stage: &mut Escalation,
    value_certainty: Certainty,
    value_stage: Escalation,
) {
    *certainty = max_certainty(*certainty, value_certainty);
    *stage = max_stage(*stage, value_stage);
}

/// Choose Approximate over Filtered over Exact, retaining the weaker of two receipts.
fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}

/// Finite controls for projection relations, uncertainty propagation, and retained validation errors.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicates::decision::stage_rank;

    /// Approximate-terminal policy for decided integer controls; strict uncertainty cases select STRICT explicitly.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Build a point from three exactly converted integer coordinates.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(x.into(), y.into(), z.into())
    }

    /// Require normal-form proof of the unit-circle identity at e, then return a separate unresolved zero.
    ///
    /// The shared angle-addition fixture supplies terminal-policy coverage without
    /// losing the original now-certified identity assertion.
    fn terminal_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    /// Build an unchecked x-axis slab from the supplied scalar bounds.
    fn x_slab(min: Real, max: Real) -> SupportSlab3 {
        SupportSlab3::new(p3(1, 0, 0), min, max)
    }

    /// Check all five scalar locations, separation on both sides, and selected strict unknowns.
    ///
    /// Integer relation assertions extract values; uncertainty controls assert Unknown
    /// without constraining its reason or stage.
    #[test]
    fn projection_and_overlap_helpers_cover_every_location_and_unknown_boundary() {
        for (value, expected) in [
            (-1, ProjectionIntervalLocation::Below),
            (0, ProjectionIntervalLocation::OnMin),
            (1, ProjectionIntervalLocation::Inside),
            (2, ProjectionIntervalLocation::OnMax),
            (3, ProjectionIntervalLocation::Above),
        ] {
            assert_eq!(
                classify_projection_interval(
                    &Real::from(value),
                    &Real::from(0),
                    &Real::from(2),
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }
        assert!(matches!(
            classify_projection_interval(
                &terminal_zero(),
                &Real::from(0),
                &Real::from(2),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_projection_interval(
                &(&Real::from(2) + &terminal_zero()),
                &Real::from(0),
                &Real::from(2),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(
            classify_interval_overlap(
                &Real::from(-3),
                &Real::from(-1),
                &Real::from(0),
                &Real::from(2),
                APPROX,
            )
            .value(),
            Some(SupportDopRelation::Separated)
        );
        assert_eq!(
            classify_interval_overlap(
                &Real::from(3),
                &Real::from(4),
                &Real::from(0),
                &Real::from(2),
                APPROX,
            )
            .value(),
            Some(SupportDopRelation::Separated)
        );
        assert!(matches!(
            classify_interval_overlap(
                &Real::from(-1),
                &terminal_zero(),
                &Real::from(0),
                &Real::from(2),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_interval_overlap(
                &(&Real::from(2) + &terminal_zero()),
                &Real::from(3),
                &Real::from(0),
                &Real::from(2),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Require Unknown through point/box/plane wrappers for selected unresolved bounds and projections.
    ///
    /// Also checks unresolved extremum construction. These fixtures do not assert a
    /// particular uncertainty reason or dispatch stage.
    #[test]
    fn strict_support_dop_classifiers_propagate_slab_and_projection_uncertainty() {
        let uncertain_bounds = SupportDop3::from_slabs(vec![x_slab(terminal_zero(), 0.into())]);
        assert!(matches!(
            uncertain_bounds.classify_point(&p3(0, 0, 0), PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            uncertain_bounds.classify_aabb3_report(
                &p3(0, 0, 0),
                &p3(1, 1, 1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            uncertain_bounds.classify_aabb3(&p3(0, 0, 0), &p3(1, 1, 1), PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));
        let plane = Plane3::new(p3(1, 0, 0), 0.into());
        assert!(matches!(
            uncertain_bounds.classify_plane3_report(&plane, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            uncertain_bounds.classify_plane3(&plane, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        let dop = SupportDop3::from_slabs(vec![x_slab(0.into(), 2.into())]);
        assert!(matches!(
            dop.classify_point(
                &Point3::new(terminal_zero(), 0.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            dop.classify_point(
                &Point3::new(&Real::from(2) + &terminal_zero(), 0.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            dop.classify_aabb3_report(
                &Point3::new(terminal_zero(), 0.into(), 0.into()),
                &p3(0, 1, 1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let opaque_zero = terminal_zero();
        assert!(matches!(
            dop.classify_aabb3_report(
                &Point3::new(&opaque_zero - &Real::from(1), 0.into(), 0.into()),
                &Point3::new(opaque_zero, 1.into(), 1.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            SupportDop3::from_points(
                &[p3(1, 0, 0)],
                &[
                    p3(0, 0, 0),
                    Point3::new(terminal_zero(), 0.into(), 0.into()),
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Require Unknown for an unresolved carrier axis and two unresolved query-plane controls.
    #[test]
    fn plane_report_propagates_carrier_and_each_query_side_uncertainty() {
        let uncertain_axis = SupportDop3::from_slabs(vec![SupportSlab3::new(
            Point3::new(1.into(), terminal_zero(), 0.into()),
            1.into(),
            2.into(),
        )]);
        assert!(matches!(
            uncertain_axis.classify_plane3_report(
                &Plane3::new(p3(1, 0, 0), 0.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let exact_slab = SupportDop3::from_slabs(vec![x_slab(0.into(), 2.into())]);
        let uncertain_normal = || Point3::new(0.into(), terminal_zero(), 0.into());
        assert!(matches!(
            exact_slab.classify_plane3_report(
                &Plane3::new(uncertain_normal(), 1.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            exact_slab.classify_plane3_report(
                &Plane3::new(uncertain_normal(), (-1).into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Require Unknown at selected min/max scan and box-projection comparisons.
    ///
    /// A negative-axis integer control must project the box to `[-2, 0]`; its receipt
    /// is not independently asserted.
    #[test]
    fn slab_building_distinguishes_unknown_minimum_and_later_unknown_maximum() {
        assert!(matches!(
            build_slab(
                &p3(1, 0, 0),
                &[
                    p3(0, 0, 0),
                    Point3::new(terminal_zero(), 0.into(), 0.into())
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            build_slab(
                &p3(1, 0, 0),
                &[
                    p3(0, 0, 0),
                    p3(1, 0, 0),
                    Point3::new(&Real::from(1) + &terminal_zero(), 0.into(), 0.into()),
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let projected = project_aabb3_on_axis(&p3(-1, 0, 0), &p3(0, 0, 0), &p3(2, 1, 1), APPROX)
            .value()
            .expect("integer AABB projection should decide");
        assert_eq!(projected, (Real::from(-2), Real::from(0)));
        assert!(matches!(
            project_aabb3_on_axis(
                &p3(1, 0, 0),
                &Point3::new(terminal_zero(), 0.into(), 0.into()),
                &p3(0, 1, 1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check validation errors for unresolved interval order/relation and a missing terminal index.
    #[test]
    fn aabb_report_validation_rejects_unknown_intervals_and_wrong_terminals() {
        let slab = x_slab(0.into(), 0.into());
        let report = SupportDopAabb3Report {
            relation: SupportDopRelation::ConservativeOverlap,
            slab_count: 1,
            terminal_slab: None,
            slab_reports: vec![SupportDopAabb3SlabReport {
                slab_index: 0,
                slab: slab.clone(),
                query_min: Some(0.into()),
                query_max: Some(terminal_zero()),
                relation: SupportDopRelation::ConservativeOverlap,
            }],
        };
        assert_eq!(
            report.validate(PredicatePolicy::STRICT),
            Err(SupportDopAabb3ValidationError::QueryIntervalInvalid)
        );

        let report = SupportDopAabb3Report {
            relation: SupportDopRelation::ConservativeOverlap,
            slab_count: 1,
            terminal_slab: None,
            slab_reports: vec![SupportDopAabb3SlabReport {
                slab_index: 0,
                slab: slab.clone(),
                query_min: Some(terminal_zero()),
                query_max: Some(2.into()),
                relation: SupportDopRelation::ConservativeOverlap,
            }],
        };
        assert_eq!(
            report.validate(PredicatePolicy::STRICT),
            Err(SupportDopAabb3ValidationError::SlabRelationMismatch)
        );

        let report = SupportDopAabb3Report {
            relation: SupportDopRelation::Separated,
            slab_count: 1,
            terminal_slab: None,
            slab_reports: vec![SupportDopAabb3SlabReport {
                slab_index: 0,
                slab,
                query_min: Some(1.into()),
                query_max: Some(2.into()),
                relation: SupportDopRelation::Separated,
            }],
        };
        assert_eq!(
            report.validate(APPROX),
            Err(SupportDopAabb3ValidationError::TerminalSlabMismatch)
        );
    }

    /// Check inverted witnessed bounds and unresolved bound/point comparisons map to their expected errors.
    #[test]
    fn witnessed_slab_validation_preserves_unknown_and_inverted_bounds() {
        let points = [p3(0, 0, 0), p3(1, 0, 0)];
        let axis = SupportDopAxis3::new([1, 0, 0]);
        let inverted = WitnessedSupportSlab3 {
            axis,
            min: support_witness(1, &points[1], 1.into()),
            max: support_witness(0, &points[0], 0.into()),
        };
        assert_eq!(
            validate_witnessed_slab(&inverted, &points, APPROX),
            Err(SupportDopValidationError::WitnessNotMinimal)
        );

        let uncertain_point = Point3::new(terminal_zero(), 0.into(), 0.into());
        let uncertain_points = [uncertain_point.clone(), p3(0, 0, 0)];
        let uncertain = WitnessedSupportSlab3 {
            axis,
            min: support_witness(0, &uncertain_point, terminal_zero()),
            max: support_witness(1, &uncertain_points[1], 0.into()),
        };
        assert_eq!(
            validate_witnessed_slab(&uncertain, &uncertain_points, PredicatePolicy::STRICT),
            Err(SupportDopValidationError::UnknownSlabOrder)
        );

        let opaque_point = Point3::new(terminal_zero(), 0.into(), 0.into());
        let points = [p3(0, 0, 0), p3(1, 0, 0), opaque_point];
        let uncertain_min_order = WitnessedSupportSlab3 {
            axis,
            min: support_witness(0, &points[0], 0.into()),
            max: support_witness(1, &points[1], 1.into()),
        };
        assert_eq!(
            validate_witnessed_slab(&uncertain_min_order, &points, PredicatePolicy::STRICT),
            Err(SupportDopValidationError::UnknownSlabOrder)
        );

        let opaque_point = Point3::new(terminal_zero(), 0.into(), 0.into());
        let points = [p3(-1, 0, 0), p3(0, 0, 0), opaque_point];
        let uncertain_max_order = WitnessedSupportSlab3 {
            axis,
            min: support_witness(0, &points[0], (-1).into()),
            max: support_witness(1, &points[1], 0.into()),
        };
        assert_eq!(
            validate_witnessed_slab(&uncertain_max_order, &points, PredicatePolicy::STRICT),
            Err(SupportDopValidationError::UnknownSlabOrder)
        );
    }

    /// Check selected unknown helpers, all four side-feasibility labels, and certainty/stage rankings.
    #[test]
    fn support_dop_helpers_cover_unknowns_side_relations_and_trace_ranks() {
        let slab = x_slab(terminal_zero(), 0.into());
        assert!(matches!(
            validate_slab_bounds(&slab, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            decided_bool(PredicateOutcome::unknown(
                crate::RefinementNeed::RealRefinement,
                Escalation::Undecided,
            )),
            None
        );
        assert!(matches!(
            support_dop_slab_halfspaces(&[slab], PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(
            support_dop_plane_relation_from_side_feasibility(true, true),
            SupportDopPlaneRelation::Intersecting
        );
        assert_eq!(
            support_dop_plane_relation_from_side_feasibility(true, false),
            SupportDopPlaneRelation::Below
        );
        assert_eq!(
            support_dop_plane_relation_from_side_feasibility(false, true),
            SupportDopPlaneRelation::Above
        );
        assert_eq!(
            support_dop_plane_relation_from_side_feasibility(false, false),
            SupportDopPlaneRelation::Degenerate
        );

        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(
            max_certainty(Certainty::Filtered, Certainty::Approximate),
            Certainty::Approximate
        );
        assert_eq!(
            max_stage(Escalation::Filter, Escalation::Exact),
            Escalation::Exact
        );
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);
    }
}
