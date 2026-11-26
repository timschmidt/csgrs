//! Plane classification helpers.
//!
//! Explicit planes use `normal . point + offset`; oriented point triples use
//! the sign convention of the 3D orientation determinant. Retained filters
//! accelerate those same expressions without changing their meaning. A chosen
//! policy governs unresolved signs, not plane validity or a total work budget.
//!
//! Keep operation outcomes with their local certainty. AABB reports rely on
//! that outer receipt, while triangle-plane reports currently discard child
//! certainty. Report validation checks specified consistency/replay conditions;
//! success does not independently establish geometry validity or source fidelity.

use crate::predicate::PredicatePolicy;
use crate::predicates::decision::{max_certainty, max_stage};
use core::cmp::Ordering;

use hyperreal::{
    AffineDet3ExactWordFilter, AffineDet3Filter, LinearForm3Filter, Rational,
    RationalLinearForm4Filter, RationalLinearForm4Query, Real, RealExactSetFacts, RealSign,
    ZeroKnowledge,
};

use crate::SymbolicDependencyMask;
use crate::classify::{PlaneAabbRelation, PlaneSegmentRelation, PlaneSide, PlaneTriangleRelation};
use crate::geometry::Point3;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::orient3d_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{
    map_outcome, resolve_real_sign, resolve_real_sign_direct, signed_term_filter,
};

/// Plane represented by `normal . point + offset = 0`.
///
/// Coefficients remain exact and need not form a unit normal. No constructor
/// check excludes a zero normal: then the expression is constant and does not
/// define an ordinary oriented plane. Classifiers evaluate that expression
/// without diagnosing the degenerate carrier. Public mutation invalidates any
/// previously derived facts or filters unless their transport is proved.
/// Derived equality compares stored scalar representations, not geometric
/// equivalence under a common nonzero coefficient scale.
#[derive(Clone, Debug, PartialEq)]
pub struct Plane3 {
    /// Plane normal vector.
    pub normal: Point3,
    /// Constant offset in `normal . point + offset = 0`.
    pub offset: Real,
}

/// Cheap structural facts for a [`Plane3`].
///
/// The facts are conservative scheduling metadata for repeated point-plane
/// classification. They record exact-set and sparse-support signals for the
/// coefficients of `normal . point + offset = 0`, but they do not decide which
/// side a query point lies on. Object structure remains at the geometry layer,
/// while certified predicates decide topology. Sparse coefficient support
/// carries the same structure into linear-form scheduling.
///
/// This Copy snapshot has public fields and no source identity or validation.
/// Query methods trust it; rebuild after changing coefficients. Masks produced
/// by [`Plane3::structural_facts`] use only their low four bits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Plane3Facts {
    /// Exact-rational representation facts for normal coordinates plus offset.
    pub coefficient_exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families for normal coordinates plus offset.
    ///
    /// Plane evidence can carry this scheduling fact next to exact-set
    /// and sparse coefficient facts without inspecting `Real` internals. It is
    /// not a side classification certificate; point-plane sidedness still comes
    /// from exact sign resolution.
    pub coefficient_symbolic_dependencies: SymbolicDependencyMask,
    /// Structural facts for the normal vector.
    pub normal: crate::geometry::Point3Facts,
    /// Bit mask of coefficients known to be exactly zero.
    ///
    /// Bits 0, 1, and 2 correspond to `normal.x`, `normal.y`, and `normal.z`;
    /// bit 3 corresponds to `offset`.
    pub coefficient_zero_mask: u8,
    /// Bit mask of coefficients known to be nonzero.
    pub coefficient_nonzero_mask: u8,
    /// Bit mask of coefficients whose zero status is unknown.
    pub coefficient_unknown_zero_mask: u8,
}

/// Failure to validate a retained plane/AABB report under the supplied policy.
///
/// The report validates the support-extrema reduction used to classify a box
/// against an oriented plane. These variants conflate some unresolved decisions
/// with detected mismatches; they do not retain the predicate's Unknown reason
/// or a validation certainty receipt. The report's validation methods describe
/// which fields and source relationships each check actually covers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneAabbReportValidationError {
    /// Lower exceeds upper, or the validator could not decide their ordering.
    ExtremumOrderMismatch,
    /// A sign differs from its recomputed value, or a sign could not be decided.
    ExtremumSignMismatch,
    /// The retained extrema signs derive a different coarse relation.
    RelationMismatch,
    /// Source replay is unresolved or fails stored-field equality with this report.
    SourceReplayMismatch,
}

/// Coarse vertex-side relation between a query triangle and an oriented plane.
///
/// This label carries no certainty or plane-nondegeneracy proof. The producing
/// triangle-report API may use approximate signs and does not retain that fact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrianglePlaneRelation {
    /// Every query vertex is strictly above the oriented plane.
    StrictlyAbove,
    /// Every query vertex is strictly below the oriented plane.
    StrictlyBelow,
    /// Every query vertex is on the plane.
    Coplanar,
    /// Query vertices occur on both sides, or on one side plus the plane.
    Straddling,
    /// At least one required orientation predicate was undecided.
    Unknown,
}

/// Structural inconsistency in a retained triangle/plane report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrianglePlaneReportValidationError {
    /// The retained vertex sides do not derive the retained relation.
    RelationMismatch,
    /// Recomputed relation/side fields differ, or supplied source indices are out of range.
    SourceReplayMismatch,
}

/// Report-bearing triangle/plane classification with retained side facts.
///
/// Retains only side values or missing decisions: per-vertex certainty,
/// escalation and Unknown reasons are discarded by the producer. Public fields
/// may also be authored directly. A self-consistent Unknown report can validate
/// successfully; this record alone does not certify triangle/plane topology.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TrianglePlaneReport {
    /// Coarse relation.
    pub relation: TrianglePlaneRelation,
    /// Per-query-vertex side, or `None` when the predicate was undecided.
    pub vertex_sides: [Option<PlaneSide>; 3],
}

impl TrianglePlaneReport {
    /// Validate that retained vertex side facts imply the reported relation.
    ///
    /// Checks the relation truth table only, without source geometry or predicate
    /// replay. None sides are consistent with Unknown and can produce Ok.
    pub fn validate(&self) -> Result<(), TrianglePlaneReportValidationError> {
        if triangle_plane_relation_from_sides(self.vertex_sides) == self.relation {
            Ok(())
        } else {
            Err(TrianglePlaneReportValidationError::RelationMismatch)
        }
    }

    /// Validate this report by recomputing it from source triangles.
    ///
    /// After structural validation, compares the complete relation/side arrays
    /// from the same classifier under `policy`. Matching unresolved reports are
    /// accepted. No child certainty is available to validate, and neither source
    /// plane nondegeneracy nor original source identity is established.
    pub fn validate_against_triangles(
        &self,
        plane: [&Point3; 3],
        query: [&Point3; 3],
        policy: PredicatePolicy,
    ) -> Result<(), TrianglePlaneReportValidationError> {
        self.validate()?;
        let replay = classify_triangle_against_oriented_plane_with_policy(plane, query, policy);
        if self == &replay {
            Ok(())
        } else {
            Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate this report against indexed source triangles.
    ///
    /// Checks structural consistency first, then bounds of all six indices,
    /// then relation/side replay equality. Out-of-range indices map to
    /// SourceReplayMismatch. Repeated indices and degenerate faces are not
    /// separately rejected; matching Unknown reports can pass.
    pub fn validate_against_sources(
        &self,
        points: &[Point3],
        face: [usize; 3],
        query: [usize; 3],
        policy: PredicatePolicy,
    ) -> Result<(), TrianglePlaneReportValidationError> {
        self.validate()?;
        if !triangle_indices_in_range(points, face) || !triangle_indices_in_range(points, query) {
            return Err(TrianglePlaneReportValidationError::SourceReplayMismatch);
        }
        let replay = classify_triangle_against_oriented_plane_with_policy(
            [&points[face[0]], &points[face[1]], &points[face[2]]],
            [&points[query[0]], &points[query[1]], &points[query[2]]],
            policy,
        );
        if self == &replay {
            Ok(())
        } else {
            Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
        }
    }
}

/// Report-bearing closed AABB/oriented-plane classification.
///
/// The coarse [`PlaneAabbRelation`] is retained for cheap callers. This report
/// keeps selected support points, exact expressions at those points, their sign
/// classifications and the per-axis term orderings used for selection. Under
/// certified decisions these points are the extrema. An approximate policy may
/// qualify selection or signs, so retain the enclosing [`PredicateOutcome`]'s
/// certainty; this record contains no independent certainty field.
/// Public fields and derived structural equality do not authenticate the source.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneAabbReport {
    /// Coarse relation between the closed box and plane.
    pub relation: PlaneAabbRelation,
    /// Selected lower support corner; minimizing when the selection decisions are certified.
    pub lower_point: Point3,
    /// Selected upper support corner; maximizing when the selection decisions are certified.
    pub upper_point: Point3,
    /// Exact expression value at [`Self::lower_point`].
    pub lower_value: Real,
    /// Exact expression value at [`Self::upper_point`].
    pub upper_value: Real,
    /// Sign classification of [`Self::lower_value`], qualified by the outer outcome.
    pub lower_sign: Sign,
    /// Sign classification of [`Self::upper_value`], qualified by the outer outcome.
    pub upper_sign: Sign,
    /// Per-axis ordering of `normal_i * min_i` versus `normal_i * max_i`.
    pub axis_term_orderings: [Ordering; 3],
}

impl PlaneAabbReport {
    /// Validate retained extrema ordering, signs, and derived relation.
    ///
    /// Checks lower <= upper, each stored sign, and the relation truth table in
    /// that order. It does not check support points, axis orderings or that the
    /// values were evaluated from any plane/box. Unresolved comparisons become
    /// mismatch errors; permitted approximate decisions can produce Ok without
    /// a returned certainty receipt. Use source replay to compare all fields.
    pub fn validate(&self, policy: PredicatePolicy) -> Result<(), PlaneAabbReportValidationError> {
        match compare_reals_with_policy(&self.lower_value, &self.upper_value, policy) {
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => return Err(PlaneAabbReportValidationError::ExtremumOrderMismatch),
            PredicateOutcome::Decided { .. } => {}
            PredicateOutcome::Unknown { .. } => {
                return Err(PlaneAabbReportValidationError::ExtremumOrderMismatch);
            }
        }

        if sign_real_with_policy(&self.lower_value, policy) != Some(self.lower_sign)
            || sign_real_with_policy(&self.upper_value, policy) != Some(self.upper_sign)
        {
            return Err(PlaneAabbReportValidationError::ExtremumSignMismatch);
        }

        if plane_aabb_relation_from_extrema(self.lower_sign, self.upper_sign) == self.relation {
            Ok(())
        } else {
            Err(PlaneAabbReportValidationError::RelationMismatch)
        }
    }

    /// Replay this report against source plane and box bounds.
    ///
    /// Runs [`Self::validate`] first, then requires a decided replay whose whole
    /// record equals this one by stored-field equality. An equivalent box or
    /// plane can choose different representations/tie corners and fail equality.
    /// Replay certainty is discarded, and unresolved replay is SourceReplayMismatch.
    pub fn validate_against_sources(
        &self,
        plane: &Plane3,
        min: &Point3,
        max: &Point3,
        policy: PredicatePolicy,
    ) -> Result<(), PlaneAabbReportValidationError> {
        self.validate(policy)?;
        match classify_plane_aabb3_report_with_policy(plane, min, max, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(PlaneAabbReportValidationError::SourceReplayMismatch),
        }
    }
}

impl Plane3Facts {
    /// Counts coefficients known to be exactly zero.
    pub fn coefficient_zero_count(self) -> u32 {
        self.coefficient_zero_mask.count_ones()
    }

    /// Counts coefficients known to be nonzero.
    pub fn coefficient_nonzero_count(self) -> u32 {
        self.coefficient_nonzero_mask.count_ones()
    }

    /// Counts coefficients with unknown zero status.
    pub fn coefficient_unknown_zero_count(self) -> u32 {
        self.coefficient_unknown_zero_mask.count_ones()
    }

    /// Returns whether the plane normal is structurally known to be zero.
    ///
    /// A zero normal is usually invalid domain geometry for oriented topology,
    /// but this fact remains advisory: callers must decide domain policy above
    /// the predicate layer.
    /// False is absence of this proof, not proof that the normal is nonzero.
    pub fn normal_known_zero(self) -> bool {
        self.normal.known_zero
    }

    /// Returns whether the normal has known zero or one-hot support.
    /// Zero normals qualify as sparse; this is not an oriented-plane validity test.
    pub fn normal_has_sparse_support(self) -> bool {
        self.normal.has_sparse_support()
    }

    /// Returns whether all coefficients share one exact denominator.
    pub fn has_shared_denominator_schedule(self) -> bool {
        self.coefficient_exact.has_shared_denominator_schedule()
    }

    /// Returns whether all coefficients are exact dyadics.
    pub fn has_dyadic_schedule(self) -> bool {
        self.coefficient_exact.has_dyadic_schedule()
    }
}

impl Plane3 {
    /// Construct a plane from a normal vector and offset.
    /// Stores coefficients unchanged, without normalizing or validating the normal.
    pub const fn new(normal: Point3, offset: Real) -> Self {
        Self { normal, offset }
    }

    /// Return structural facts for this plane's coefficients.
    /// The snapshot does not retain coefficient identity and is not updated by later mutation.
    pub fn structural_facts(&self) -> Plane3Facts {
        plane3_facts(self)
    }
}

/// Reusable exact-predicate evidence for a fixed explicit plane.
///
/// This value owns coefficient facts plus certified dyadic and rational
/// linear-form filters, but not the plane. Retain it alongside the source
/// plane when classifying many points against the same plane.
/// The Copy record does not bind itself to a source identity or detect changes:
/// callers must rebuild it after coefficient mutation or transformation. A
/// missing filter means that specialization declined, not that the plane is invalid.
#[derive(Clone, Copy, Debug)]
pub struct Plane3Evidence {
    /// Structural coefficient snapshot used to select exact expression schedules.
    facts: Plane3Facts,
    /// Optional certified floating linear-form filter for eligible exact-dyadic coefficients.
    filter: Option<LinearForm3Filter>,
    /// Optional certified floating filter with rational conversion-error bounds.
    rational_filter: Option<RationalLinearForm4Filter>,
}

impl Plane3Evidence {
    /// Return structural facts for the source plane.
    pub const fn facts(&self) -> Plane3Facts {
        self.facts
    }
}

/// Derive reusable exact-predicate evidence for an explicit plane.
/// Prepares dyadic and rational filters before collecting structural facts;
/// neither filter availability nor the facts validate a nonzero normal.
pub fn plane3_evidence(plane: &Plane3) -> Plane3Evidence {
    crate::trace_dispatch!("hyperlimit", "plane3_evidence", "derive");
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    let filter = LinearForm3Filter::from_reals(coefficients);
    let rational_filter = RationalLinearForm4Filter::from_reals(coefficients);
    Plane3Evidence {
        facts: plane.structural_facts(),
        filter,
        rational_filter,
    }
}

/// Classify a point using retained explicit-plane evidence and a policy.
///
/// `evidence` must have been derived from `plane` with [`plane3_evidence`].
/// Its coefficients must still match: no runtime source check protects against
/// stale or mismatched filters. Tries the retained dyadic filter, then the exact
/// rational path with its optional retained filter, then the generic Real path
/// with trusted facts. Successful filters remain certified under either policy;
/// only subsequent unresolved sign work may consume an approximate terminal.
#[inline]
pub fn classify_point_plane_with_evidence_and_policy(
    point: &Point3,
    plane: &Plane3,
    evidence: &Plane3Evidence,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    if let Some(sign) = evidence
        .filter
        .and_then(|filter| filter.sign([&point.x, &point.y, &point.z]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "classify_point_plane",
            "evidence-certified-linear-form3-filter"
        );
        return PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if let Some(outcome) =
        classify_exact_rational_point_plane(point, plane, evidence.rational_filter)
    {
        return outcome;
    }
    classify_point_plane_with_facts(point, plane, evidence.facts, policy)
}

/// Reusable exact-predicate evidence for a fixed oriented triangle plane.
///
/// This reduces the oriented plane through `a`, `b`, and `c` once into an exact
/// explicit plane. Repeated point queries can then share the same retained
/// point-plane evidence instead of rebuilding the `orient3d` determinant.
/// Unlike [`Plane3Evidence`], it owns the derived plane and offers only shared
/// access to it. It does not retain the original points or verify their affine
/// independence. The plane normal is `-(b-a) x (c-a)`, matching `orient3d` here.
#[derive(Clone, Debug)]
pub struct OrientedPlane3Evidence {
    /// Certified floating determinant filter for eligible exact-dyadic source triples.
    filter: Option<AffineDet3Filter>,
    /// Checked i128 homogeneous-plane fallback, prepared only when the dyadic filter is unavailable.
    exact_word_filter: Option<Box<AffineDet3ExactWordFilter>>,
    /// Exact explicit coefficients with the orientation determinant's sign convention.
    plane: Plane3,
    /// Snapshot derived from the owned plane, reused for expression scheduling.
    facts: Plane3Facts,
}

impl OrientedPlane3Evidence {
    /// Return the explicit plane built from the oriented point triple.
    /// This immutable view has the negative cross-product normal and is not normalized.
    pub fn plane(&self) -> &Plane3 {
        &self.plane
    }

    /// Return cached structural facts for the explicit plane coefficients.
    pub fn facts(&self) -> Plane3Facts {
        self.facts
    }
}

/// Derive reusable evidence for the oriented plane through `a`, `b`, and `c`.
///
/// Let `cross = (b-a) x (c-a)`. The owned plane is `(-cross, cross . a)`, so
/// `(a,b,c) = ((0,0,0),(1,0,0),(0,1,0))` classifies positive z as Below.
/// Collinear input is not rejected and may yield an all-zero expression.
/// Construction retains exact coefficients without a policy or normalizing division.
pub fn oriented_plane3_evidence(a: &Point3, b: &Point3, c: &Point3) -> OrientedPlane3Evidence {
    crate::trace_dispatch!("hyperlimit", "oriented_plane3_evidence", "derive");
    let abx = sub(&b.x, &a.x);
    let aby = sub(&b.y, &a.y);
    let abz = sub(&b.z, &a.z);
    let acx = sub(&c.x, &a.x);
    let acy = sub(&c.y, &a.y);
    let acz = sub(&c.z, &a.z);

    let cross_x = sub(&mul(&aby, &acz), &mul(&abz, &acy));
    let cross_y = sub(&mul(&abz, &acx), &mul(&abx, &acz));
    let cross_z = sub(&mul(&abx, &acy), &mul(&aby, &acx));

    let nx_ax = mul(&cross_x, &a.x);
    let ny_ay = mul(&cross_y, &a.y);
    let nz_az = mul(&cross_z, &a.z);
    let nxy_a = add(&nx_ax, &ny_ay);
    let dot_a = add(&nxy_a, &nz_az);
    let zero = sub(&a.x, &a.x);
    let nx = sub(&zero, &cross_x);
    let ny = sub(&zero, &cross_y);
    let nz = sub(&zero, &cross_z);

    let plane = Plane3::new(Point3::new(nx, ny, nz), dot_a);
    let facts = plane.structural_facts();
    let filter =
        AffineDet3Filter::from_reals([&a.x, &a.y, &a.z], [&b.x, &b.y, &b.z], [&c.x, &c.y, &c.z]);
    let exact_word_filter = if filter.is_none() {
        AffineDet3ExactWordFilter::from_reals(
            [&a.x, &a.y, &a.z],
            [&b.x, &b.y, &b.z],
            [&c.x, &c.y, &c.z],
        )
        .map(Box::new)
    } else {
        None
    };
    OrientedPlane3Evidence {
        filter,
        exact_word_filter,
        plane,
        facts,
    }
}

/// Classify a point using retained oriented-plane evidence and a policy.
/// Tries its dyadic determinant filter, then the optional checked-word filter,
/// falling back to its owned explicit plane and facts. Each declined filter
/// leaves the exact fallback available; no source triple is reconstructed.
#[inline]
pub fn classify_point_oriented_plane_with_evidence_and_policy(
    point: &Point3,
    evidence: &OrientedPlane3Evidence,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    if let Some(sign) = evidence
        .filter
        .and_then(|filter| filter.sign([&point.x, &point.y, &point.z]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "oriented_plane3_evidence",
            "certified-real-det3-filter"
        );
        return PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if evidence.exact_word_filter.is_some() {
        return classify_point_oriented_plane_exact_word(point, evidence, policy);
    }
    classify_point_plane_with_facts(point, &evidence.plane, evidence.facts, policy)
}

/// Try the retained checked-word determinant, then classify with the owned explicit plane.
/// Kept as a separate cold route; calling it without a word filter also reaches the fallback.
#[cold]
#[inline(never)]
fn classify_point_oriented_plane_exact_word(
    point: &Point3,
    evidence: &OrientedPlane3Evidence,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    if let Some(sign) = evidence
        .exact_word_filter
        .as_deref()
        .and_then(|filter| filter.sign([&point.x, &point.y, &point.z]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "oriented_plane3_evidence",
            "exact-word-homogeneous-det3"
        );
        return PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    classify_point_plane_with_facts(point, &evidence.plane, evidence.facts, policy)
}

/// Enter generic point-plane classification with a trusted coefficient snapshot.
/// Facts must still match the plane, especially before the known-exact scalar path.
#[inline(always)]
fn classify_point_plane_with_facts(
    point: &Point3,
    plane: &Plane3,
    facts: Plane3Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_real(point, plane, Some(facts), policy)
}

/// Classify a point relative to a plane with an explicit escalation policy.
///
/// Signs of `normal . point + offset` map to Below/On/Above. Tries a certified
/// dyadic filter, then a direct rational product sum, then the shared Real
/// pipeline. No normal-length or plane-validity check is performed. Approximate
/// On is qualified by the outcome and need not prove exact incidence.
pub fn classify_point_plane_with_policy(
    point: &Point3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    if let Some(sign) = Real::certified_linear_form3_sign(
        [
            &plane.normal.x,
            &plane.normal.y,
            &plane.normal.z,
            &plane.offset,
        ],
        [&point.x, &point.y, &point.z],
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "classify_point_plane",
            "certified-linear-form3-filter"
        );
        return PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if let Some(outcome) = classify_exact_rational_point_plane(point, plane, None) {
        return outcome;
    }
    classify_point_plane_real(point, plane, None, policy)
}

/// Decide an exposed-rational plane/query pair, using a retained filter before exact summation.
/// Requires all seven input scalars to expose rational references; otherwise
/// declines. A supplied filter must match the plane. Filter hits are Exact at
/// Filter stage, while complete rational product sums are Exact at Exact stage.
#[inline]
fn classify_exact_rational_point_plane(
    point: &Point3,
    plane: &Plane3,
    filter: Option<RationalLinearForm4Filter>,
) -> Option<PredicateOutcome<PlaneSide>> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::exact_rational_ref) else {
        return None;
    };
    let [Some(x), Some(y), Some(z)] = [&point.x, &point.y, &point.z].map(Real::exact_rational_ref)
    else {
        return None;
    };

    if let Some(sign) = filter.and_then(|filter| {
        RationalLinearForm4Query::from_affine_point3([x, y, z])
            .and_then(|query| filter.sign(&query))
    }) {
        crate::trace_dispatch!(
            "hyperlimit",
            "classify_point_plane",
            "evidence-rational-linear-form-filter"
        );
        return Some(PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Filter,
        ));
    }

    let one = Rational::one();
    let sign =
        match Rational::signed_product_sum_ordering([true; 4], [[a, x], [b, y], [c, z], [d, &one]])
        {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        };
    crate::trace_dispatch!(
        "hyperlimit",
        "classify_point_plane",
        "exact-rational-product-sum-sign"
    );
    Some(PredicateOutcome::decided(
        PlaneSide::from(crate::real::map_real_sign(sign)),
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Classify a point/plane pair directly through the generic Real expression pipeline.
///
/// Use this internal route when the caller already knows the pair is ineligible
/// or when constructing a filter for a single comparison would cost more than
/// it saves. This skips both initial dyadic filtering and the separate direct
/// rational point-plane route of the public classifier. The generic sign
/// resolver still has its structural, exact-recognition and refinement stages.
#[inline(always)]
pub(crate) fn classify_point_plane_without_filter_with_policy(
    point: &Point3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_real(point, plane, None, policy)
}

/// Try a locally prepared dyadic plane filter before the generic Real expression route.
/// The filter must match the supplied plane; no retained rational filter is tried here.
#[inline(always)]
fn classify_point_plane_with_filter(
    filter: Option<LinearForm3Filter>,
    point: &Point3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    if let Some(sign) = filter.and_then(|filter| filter.sign([&point.x, &point.y, &point.z])) {
        crate::trace_dispatch!(
            "hyperlimit",
            "classify_point_plane",
            "local-certified-linear-form3-filter"
        );
        return PredicateOutcome::decided(
            PlaneSide::from(crate::real::map_real_sign(sign)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    classify_point_plane_without_filter_with_policy(point, plane, policy)
}

/// Classify a closed segment relative to a plane with an explicit escalation
/// policy.
///
/// Queries start and then end through the generic point-plane route; an unknown
/// start prevents querying end. A decided relation joins both local certainty
/// and stage labels. Endpoint equality and plane nondegeneracy are not separately
/// validated, and no intersection coordinate is constructed.
pub fn classify_plane_segment_with_policy(
    plane: &Plane3,
    start: &Point3,
    end: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSegmentRelation> {
    let start_outcome = classify_point_plane_without_filter_with_policy(start, plane, policy);
    let (start_side, start_certainty, start_stage) = match start_outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => (value, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    let end_outcome = classify_point_plane_without_filter_with_policy(end, plane, policy);
    let (end_side, end_certainty, end_stage) = match end_outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => (value, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    let relation = match (start_side, end_side) {
        (PlaneSide::Below, PlaneSide::Below) => PlaneSegmentRelation::Below,
        (PlaneSide::Above, PlaneSide::Above) => PlaneSegmentRelation::Above,
        (PlaneSide::On, PlaneSide::On) => PlaneSegmentRelation::Coplanar,
        (PlaneSide::On, _) | (_, PlaneSide::On) => PlaneSegmentRelation::EndpointTouch,
        (PlaneSide::Below, PlaneSide::Above) | (PlaneSide::Above, PlaneSide::Below) => {
            PlaneSegmentRelation::Crossing
        }
    };
    PredicateOutcome::decided(
        relation,
        max_certainty(start_certainty, end_certainty),
        max_stage(start_stage, end_stage),
    )
}

/// Classify a triangle relative to a plane with an explicit escalation policy.
///
/// Prepares one dyadic filter, then eagerly classifies all three vertices in
/// a/b/c order before examining their outcomes. The first Unknown in that array
/// is returned; a decided result joins all three certainty/stage labels. This
/// classifies vertex sides without proving nonzero triangle area or plane validity.
pub fn classify_plane_triangle_with_policy(
    plane: &Plane3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneTriangleRelation> {
    let filter = LinearForm3Filter::from_reals([
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]);
    let outcomes = [
        classify_point_plane_with_filter(filter, a, plane, policy),
        classify_point_plane_with_filter(filter, b, plane, policy),
        classify_point_plane_with_filter(filter, c, plane, policy),
    ];
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let mut below = 0_u8;
    let mut above = 0_u8;
    let mut on = 0_u8;

    for outcome in outcomes {
        match outcome {
            PredicateOutcome::Decided {
                value,
                certainty: value_certainty,
                stage: value_stage,
            } => {
                certainty = max_certainty(certainty, value_certainty);
                stage = max_stage(stage, value_stage);
                match value {
                    PlaneSide::Below => below += 1,
                    PlaneSide::Above => above += 1,
                    PlaneSide::On => on += 1,
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }

    let relation = if below == 3 {
        PlaneTriangleRelation::Below
    } else if above == 3 {
        PlaneTriangleRelation::Above
    } else if on == 3 {
        PlaneTriangleRelation::Coplanar
    } else if below > 0 && above > 0 {
        PlaneTriangleRelation::Split
    } else {
        PlaneTriangleRelation::BoundaryTouch
    };
    PredicateOutcome::decided(relation, certainty, stage)
}

/// Classify a query triangle against an oriented plane triangle with an
/// explicit predicate policy and retained per-vertex side facts.
///
/// Eagerly evaluates all three orientation predicates, then extracts only their
/// side values. Certainty, stage and Unknown reasons are lost. The bare report
/// can therefore contain approximate sides without recording their qualification;
/// it must not alone be treated as certified evidence after an approximate call.
/// The defining point triple is not checked for noncollinearity.
pub fn classify_triangle_against_oriented_plane_with_policy(
    plane: [&Point3; 3],
    query: [&Point3; 3],
    policy: PredicatePolicy,
) -> TrianglePlaneReport {
    let outcomes = [
        orient3d_with_policy(plane[0], plane[1], plane[2], query[0], policy),
        orient3d_with_policy(plane[0], plane[1], plane[2], query[1], policy),
        orient3d_with_policy(plane[0], plane[1], plane[2], query[2], policy),
    ];

    let mut sides = [None, None, None];
    for (index, outcome) in outcomes.into_iter().enumerate() {
        sides[index] = outcome.value().map(PlaneSide::from);
    }

    TrianglePlaneReport {
        relation: triangle_plane_relation_from_sides(sides),
        vertex_sides: sides,
    }
}

/// Derive a retained triangle/plane relation from per-vertex side facts.
/// Any missing side yields Unknown, even if present sides already differ.
/// Three equal sides yield the corresponding strict-side or coplanar label;
/// every other complete combination is Straddling. No geometry is inspected.
pub fn triangle_plane_relation_from_sides(sides: [Option<PlaneSide>; 3]) -> TrianglePlaneRelation {
    let Some([a, b, c]) = transpose_plane_sides(sides) else {
        return TrianglePlaneRelation::Unknown;
    };
    let above = [a, b, c]
        .iter()
        .filter(|&&side| side == PlaneSide::Above)
        .count();
    let below = [a, b, c]
        .iter()
        .filter(|&&side| side == PlaneSide::Below)
        .count();
    let on = [a, b, c]
        .iter()
        .filter(|&&side| side == PlaneSide::On)
        .count();

    match (above, below, on) {
        (3, 0, 0) => TrianglePlaneRelation::StrictlyAbove,
        (0, 3, 0) => TrianglePlaneRelation::StrictlyBelow,
        (0, 0, 3) => TrianglePlaneRelation::Coplanar,
        _ => TrianglePlaneRelation::Straddling,
    }
}

/// Extract three side labels in order, declining on the first missing value.
fn transpose_plane_sides(sides: [Option<PlaneSide>; 3]) -> Option<[PlaneSide; 3]> {
    Some([sides[0]?, sides[1]?, sides[2]?])
}

/// Check only index bounds; repeated indices and degenerate geometry remain allowed.
fn triangle_indices_in_range(points: &[Point3], indices: [usize; 3]) -> bool {
    indices.iter().all(|&index| index < points.len())
}

/// Classify a closed 3D AABB relative to a plane with an explicit escalation
/// policy.
///
/// This is the Hyper-native counterpart to the mesh-kernel halfspace/bounds
/// test: it finds the minimum and maximum plane expression over the box by
/// selecting the lower/upper contribution for each source axis, then certifies
/// only those two extrema. It preserves exact `Real` arithmetic while avoiding
/// eight point-plane classifications.
/// It builds the full report internally and drops its fields while preserving
/// the outer certainty/stage. Bounds may be supplied in either order per axis;
/// degenerate boxes are accepted. No validity check rejects a zero plane normal.
pub fn classify_plane_aabb3_with_policy(
    plane: &Plane3,
    min: &Point3,
    max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneAabbRelation> {
    match classify_plane_aabb3_report_with_policy(plane, min, max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.relation, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Policy-controlled report-bearing AABB/plane classifier.
///
/// This evaluates the plane expression only at the selected minimum and maximum
/// support corners. The retained `axis_term_orderings` document which bound was
/// chosen on each axis, so callers can audit the interval reduction
/// without enumerating all eight corners.
///
/// Each coordinate pair defines a closed interval regardless of argument order.
/// Products are compared in x/y/z order; ties keep the first bound for the lower
/// corner and the second for the upper corner. The upper expression is resolved
/// before the lower, and both are required for a completed report. Every consumed
/// ordering/sign contributes to the outer certainty; report fields do not retain
/// their separate receipts. Under approximation, selected corners/signs need not
/// constitute exact support certificates. Unknown aborts without a partial report.
pub fn classify_plane_aabb3_report_with_policy(
    plane: &Plane3,
    min: &Point3,
    max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneAabbReport> {
    crate::trace_dispatch!("hyperlimit", "classify_plane_aabb3", "term-interval");
    let axis_normals = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    let box_min = [&min.x, &min.y, &min.z];
    let box_max = [&max.x, &max.y, &max.z];
    let mut lower_coords = box_min;
    let mut upper_coords = box_max;
    let mut axis_term_orderings = [Ordering::Equal; 3];
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    for axis in 0..3 {
        let min_term = mul(axis_normals[axis], box_min[axis]);
        let max_term = mul(axis_normals[axis], box_max[axis]);
        let ordering = match compare_reals_with_policy(&min_term, &max_term, policy) {
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
        axis_term_orderings[axis] = ordering;
        if ordering == Ordering::Greater {
            lower_coords[axis] = box_max[axis];
            upper_coords[axis] = box_min[axis];
        }
    }

    let facts = plane.structural_facts();
    let upper_value = point_plane_expression_from_coords(
        upper_coords,
        plane,
        Some(facts),
        "classify_plane_aabb3",
    );

    let upper_sign =
        match resolve_real_sign_direct(&upper_value, policy, RefinementNeed::RealRefinement) {
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
    let lower_value = point_plane_expression_from_coords(
        lower_coords,
        plane,
        Some(facts),
        "classify_plane_aabb3",
    );
    let lower_sign =
        match resolve_real_sign_direct(&lower_value, policy, RefinementNeed::RealRefinement) {
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
    let relation = plane_aabb_relation_from_extrema(lower_sign, upper_sign);
    PredicateOutcome::decided(
        PlaneAabbReport {
            relation,
            lower_point: point3_from_coords(lower_coords),
            upper_point: point3_from_coords(upper_coords),
            lower_value,
            upper_value,
            lower_sign,
            upper_sign,
            axis_term_orderings,
        },
        certainty,
        stage,
    )
}

/// Build the exact plane expression and resolve its sign with a lazy same-sign term filter.
/// Optional facts must match the coefficients; the filter constructs x/y/z
/// products only if structural expression facts did not already decide the sign.
fn classify_point_plane_real(
    point: &Point3,
    plane: &Plane3,
    plane_facts: Option<Plane3Facts>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    crate::trace_dispatch!("hyperlimit", "classify_point_plane", "real-dot");
    let value = point_plane_expression(point, plane, plane_facts);

    map_outcome(
        resolve_real_sign(
            &value,
            policy,
            || {
                let x_term = mul(&plane.normal.x, &point.x);
                let y_term = mul(&plane.normal.y, &point.y);
                let z_term = mul(&plane.normal.z, &point.z);
                signed_term_filter(&[
                    (&x_term, Sign::Positive),
                    (&y_term, Sign::Positive),
                    (&z_term, Sign::Positive),
                    (&plane.offset, Sign::Positive),
                ])
            },
            || None,
            RefinementNeed::RealRefinement,
        ),
        PlaneSide::from,
    )
}

/// Map extrema signs to a box relation, testing strictly negative upper before positive lower.
/// Assumes consistent lower/upper selection; this truth table does not check ordering.
fn plane_aabb_relation_from_extrema(lower_sign: Sign, upper_sign: Sign) -> PlaneAabbRelation {
    if upper_sign == Sign::Negative {
        PlaneAabbRelation::Below
    } else if lower_sign == Sign::Positive {
        PlaneAabbRelation::Above
    } else {
        PlaneAabbRelation::Intersecting
    }
}

/// Resolve a sign for report validation, dropping certainty, stage and Unknown diagnostics.
fn sign_real_with_policy(value: &Real, policy: PredicatePolicy) -> Option<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement).value()
}

/// Clone three exact scalar coordinates into an owned support-point record.
fn point3_from_coords(coordinates: [&Real; 3]) -> Point3 {
    Point3::new(
        coordinates[0].clone(),
        coordinates[1].clone(),
        coordinates[2].clone(),
    )
}

/// Build `normal . point + offset` as one fixed product-sum when the object
/// facts make that route valid.
///
/// Plane evidence carries coefficient exactness and sparse-support facts beside
/// the plane rather than forcing every query to rediscover them. This helper
/// consumes those facts at the predicate-object boundary and passes the whole
/// point-plane polynomial to `hyperreal` before scalar expansion. The
/// exact-rational path can then delay normalization across the complete form.
fn point_plane_expression(
    point: &Point3,
    plane: &Plane3,
    plane_facts: Option<Plane3Facts>,
) -> Real {
    point_plane_expression_from_coords(
        [&point.x, &point.y, &point.z],
        plane,
        plane_facts,
        "classify_point_plane",
    )
}

/// Form the four-term plane polynomial, trusting retained coefficient exactness when supplied.
/// Query exactness is inspected afresh. Only coefficient exact-rational facts
/// select the known-exact aggregate path here; sparse masks do not independently
/// select it. `trace_operation` labels this construction when tracing is enabled.
fn point_plane_expression_from_coords(
    coordinates: [&Real; 3],
    plane: &Plane3,
    plane_facts: Option<Plane3Facts>,
    trace_operation: &'static str,
) -> Real {
    let _ = trace_operation;
    let one = Real::one();
    let terms = [
        [&plane.normal.x, coordinates[0]],
        [&plane.normal.y, coordinates[1]],
        [&plane.normal.z, coordinates[2]],
        [&plane.offset, &one],
    ];

    if let Some(plane_facts) = plane_facts {
        let coordinate_exact = Real::exact_set_facts(coordinates);
        if plane_facts.coefficient_exact.all_exact_rational && coordinate_exact.all_exact_rational {
            crate::trace_dispatch!("hyperlimit", trace_operation, "evidence-exact-product-sum");
            return Real::exact_rational_signed_product_sum_known_exact([true; 4], terms);
        }
    }

    crate::trace_dispatch!("hyperlimit", trace_operation, "fixed-real-product-sum");
    Real::signed_product_sum([true; 4], terms)
}

/// Snapshot coefficient exactness, zero masks, symbolic dependencies and normal support.
/// Preserves the inspection order; neither source identity nor normal validity is retained.
fn plane3_facts(plane: &Plane3) -> Plane3Facts {
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    let coefficient_exact = Real::exact_set_facts(coefficients);
    let (coefficient_zero_mask, coefficient_nonzero_mask, coefficient_unknown_zero_mask) =
        plane_coefficient_zero_masks(coefficients);

    Plane3Facts {
        coefficient_exact,
        coefficient_symbolic_dependencies: plane_coefficient_symbolic_dependencies(coefficients),
        normal: plane.normal.structural_facts(),
        coefficient_zero_mask,
        coefficient_nonzero_mask,
        coefficient_unknown_zero_mask,
    }
}

/// Union the four coefficients' detailed symbolic dependency families in coordinate order.
fn plane_coefficient_symbolic_dependencies(coordinates: [&Real; 4]) -> SymbolicDependencyMask {
    coordinates
        .into_iter()
        .fold(SymbolicDependencyMask::NONE, |mask, coordinate| {
            mask.union(coordinate.detailed_facts().symbolic.dependencies)
        })
}

/// Partition coefficient indices into known-zero, known-nonzero and unknown masks.
/// Bits 0 through 3 follow the supplied normal-x/y/z/offset order.
fn plane_coefficient_zero_masks(coordinates: [&Real; 4]) -> (u8, u8, u8) {
    let mut known_zero_mask = 0_u8;
    let mut known_nonzero_mask = 0_u8;
    let mut unknown_zero_mask = 0_u8;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        let bit = 1_u8 << index;
        match coordinate.structural_facts().zero {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

/// Join a consumed decision's certainty and ranked stage into an aggregate receipt.
/// Stage ranking records summary metadata rather than chronological evaluation order.
fn absorb_trace(
    certainty: &mut Certainty,
    stage: &mut Escalation,
    value_certainty: Certainty,
    value_stage: Escalation,
) {
    *certainty = max_certainty(*certainty, value_certainty);
    *stage = max_stage(*stage, value_stage);
}

/// Classify a point relative to the oriented plane through `a`, `b`, and `c`
/// with an explicit escalation policy.
///
/// Maps the orientation determinant sign directly to Below/On/Above, preserving
/// its outcome metadata. This corresponds to the normal `-(b-a) x (c-a)`:
/// the positive z side of the standard counterclockwise xy triple is Below.
/// A degenerate defining triple is not rejected as invalid plane input.
pub fn classify_point_oriented_plane_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    crate::trace_dispatch!("hyperlimit", "classify_point_oriented_plane", "orient3d");
    map_outcome(
        orient3d_with_policy(a, b, c, point, policy),
        PlaneSide::from,
    )
}

/// Forward borrowed exact addition to the shared traced arithmetic helper.
fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

/// Forward borrowed exact multiplication to the shared traced arithmetic helper.
fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

/// Forward borrowed exact subtraction to the shared traced arithmetic helper.
fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

#[cfg(test)]
/// Plane relation, retained-evidence, policy/replay and dispatch-count regression witnesses.
/// Value-only comparisons are distinguished from complete receipt and trace assertions.
mod tests {
    use super::*;
    use crate::predicates::decision::{certainty_rank, stage_rank};
    use hyperreal::Rational;

    /// Permit approximate terminals in general fixtures; specific strict checks select their own policy.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    #[cfg(feature = "dispatch-trace")]
    /// Return this module's lazily initialized lock for trace reset/record/take sequences.
    fn dispatch_trace_test_lock() -> &'static std::sync::Mutex<()> {
        /// Process-lifetime mutex shared by this module's dispatch instrumentation tests.
        static LOCK: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
        LOCK.get_or_init(|| std::sync::Mutex::new(()))
    }

    /// Import a finite fixture number as its exact binary64 value.
    fn real(value: f64) -> Real {
        Real::try_from(value).expect("finite test Real")
    }

    /// Construct a point or coefficient vector from three exact binary64 fixture values.
    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    /// Construct an exact ratio; authored fixture denominators must be nonzero.
    fn rational(numerator: i64, denominator: u64) -> Real {
        Real::new(Rational::fraction(numerator, denominator).expect("valid test rational"))
    }

    /// Retain the now-recognized Pythagorean zero before returning an angle-addition terminal control.
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

    #[test]
    /// Check independent above/below expectations for the plane z = 2.
    fn classifies_point_plane() {
        let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0));

        assert_eq!(
            crate::classify_point_plane(&p3(0.0, 0.0, 3.0), &plane, APPROX).value(),
            Some(PlaneSide::Above)
        );
        assert_eq!(
            crate::classify_point_plane(&p3(0.0, 0.0, 1.0), &plane, APPROX).value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    /// Exercise all five endpoint-side relations against z = 2 with integer fixtures.
    fn classifies_plane_segment_relation() {
        let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0));

        assert_eq!(
            crate::classify_plane_segment(&plane, &p3(0.0, 0.0, 0.0), &p3(1.0, 0.0, 1.0), APPROX)
                .value(),
            Some(PlaneSegmentRelation::Below)
        );
        assert_eq!(
            crate::classify_plane_segment(&plane, &p3(0.0, 0.0, 3.0), &p3(1.0, 0.0, 4.0), APPROX)
                .value(),
            Some(PlaneSegmentRelation::Above)
        );
        assert_eq!(
            crate::classify_plane_segment(&plane, &p3(0.0, 0.0, 2.0), &p3(1.0, 0.0, 2.0), APPROX)
                .value(),
            Some(PlaneSegmentRelation::Coplanar)
        );
        assert_eq!(
            crate::classify_plane_segment(&plane, &p3(0.0, 0.0, 1.0), &p3(1.0, 0.0, 3.0), APPROX)
                .value(),
            Some(PlaneSegmentRelation::Crossing)
        );
        assert_eq!(
            crate::classify_plane_segment(&plane, &p3(0.0, 0.0, 2.0), &p3(1.0, 0.0, 3.0), APPROX)
                .value(),
            Some(PlaneSegmentRelation::EndpointTouch)
        );
    }

    #[test]
    /// Exercise all five triangle-side relations against z = 2 without inferring area validity.
    fn classifies_plane_triangle_relation() {
        let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0));

        assert_eq!(
            crate::classify_plane_triangle(
                &plane,
                &p3(0.0, 0.0, 0.0),
                &p3(1.0, 0.0, 1.0),
                &p3(0.0, 1.0, 1.0),
                APPROX
            )
            .value(),
            Some(PlaneTriangleRelation::Below)
        );
        assert_eq!(
            crate::classify_plane_triangle(
                &plane,
                &p3(0.0, 0.0, 3.0),
                &p3(1.0, 0.0, 4.0),
                &p3(0.0, 1.0, 3.0),
                APPROX
            )
            .value(),
            Some(PlaneTriangleRelation::Above)
        );
        assert_eq!(
            crate::classify_plane_triangle(
                &plane,
                &p3(0.0, 0.0, 2.0),
                &p3(1.0, 0.0, 2.0),
                &p3(0.0, 1.0, 2.0),
                APPROX
            )
            .value(),
            Some(PlaneTriangleRelation::Coplanar)
        );
        assert_eq!(
            crate::classify_plane_triangle(
                &plane,
                &p3(0.0, 0.0, 1.0),
                &p3(1.0, 0.0, 3.0),
                &p3(0.0, 1.0, 1.0),
                APPROX
            )
            .value(),
            Some(PlaneTriangleRelation::Split)
        );
        assert_eq!(
            crate::classify_plane_triangle(
                &plane,
                &p3(0.0, 0.0, 2.0),
                &p3(1.0, 0.0, 3.0),
                &p3(0.0, 1.0, 3.0),
                APPROX
            )
            .value(),
            Some(PlaneTriangleRelation::BoundaryTouch)
        );
    }

    #[test]
    /// Check below/above/intersecting boxes and reversed bounds against x - 2*y + z - 1.
    fn classifies_plane_aabb3_relation_from_extreme_terms() {
        let plane = Plane3::new(p3(1.0, -2.0, 1.0), real(-1.0));

        assert_eq!(
            crate::classify_plane_aabb3(&plane, &p3(0.0, 2.0, 0.0), &p3(1.0, 3.0, 1.0), APPROX)
                .value(),
            Some(PlaneAabbRelation::Below)
        );
        assert_eq!(
            crate::classify_plane_aabb3(&plane, &p3(3.0, 0.0, 1.0), &p3(4.0, 0.0, 2.0), APPROX)
                .value(),
            Some(PlaneAabbRelation::Above)
        );
        assert_eq!(
            crate::classify_plane_aabb3(&plane, &p3(0.0, 0.0, 0.0), &p3(2.0, 2.0, 2.0), APPROX)
                .value(),
            Some(PlaneAabbRelation::Intersecting)
        );
        assert_eq!(
            crate::classify_plane_aabb3(&plane, &p3(1.0, 3.0, 1.0), &p3(0.0, 2.0, 0.0), APPROX)
                .value(),
            Some(PlaneAabbRelation::Below)
        );
    }

    #[test]
    /// Check independent support corners, signs and axis ordering, then forged values/signs and changed-source rejection.
    fn plane_aabb3_report_retains_support_extrema() {
        let plane = Plane3::new(p3(1.0, -2.0, 1.0), real(-1.0));
        let min = p3(0.0, 0.0, 0.0);
        let max = p3(2.0, 2.0, 2.0);
        let report = crate::classify_plane_aabb3_report(&plane, &min, &max, APPROX)
            .value()
            .expect("exact rational plane/AABB report should decide");

        assert_eq!(report.relation, PlaneAabbRelation::Intersecting);
        assert_eq!(report.lower_point, p3(0.0, 2.0, 0.0));
        assert_eq!(report.upper_point, p3(2.0, 0.0, 2.0));
        assert_eq!(report.lower_sign, Sign::Negative);
        assert_eq!(report.upper_sign, Sign::Positive);
        assert_eq!(
            report.axis_term_orderings,
            [Ordering::Less, Ordering::Greater, Ordering::Less]
        );
        assert_eq!(report.validate(APPROX), Ok(()));
        assert_eq!(
            report.validate_against_sources(&plane, &min, &max, APPROX,),
            Ok(())
        );

        let mut forged = report.clone();
        forged.lower_value = forged.upper_value.clone() + Real::from(1);
        assert_eq!(
            forged.validate(APPROX),
            Err(PlaneAabbReportValidationError::ExtremumOrderMismatch)
        );

        let mut forged = report.clone();
        forged.lower_sign = Sign::Positive;
        assert_eq!(
            forged.validate(APPROX),
            Err(PlaneAabbReportValidationError::ExtremumSignMismatch)
        );

        assert_eq!(
            report.validate_against_sources(&plane, &min, &p3(3.0, 2.0, 2.0), APPROX),
            Err(PlaneAabbReportValidationError::SourceReplayMismatch)
        );
    }

    #[test]
    /// Check strict-side reports, reversed endpoint order and rejection of a forged coarse relation.
    fn plane_aabb3_report_distinguishes_strict_sides_and_reversed_bounds() {
        let plane = Plane3::new(p3(1.0, -2.0, 1.0), real(-1.0));
        let below = crate::classify_plane_aabb3_report(
            &plane,
            &p3(0.0, 2.0, 0.0),
            &p3(1.0, 3.0, 1.0),
            APPROX,
        )
        .value()
        .expect("below box should decide");
        let above = crate::classify_plane_aabb3_report(
            &plane,
            &p3(3.0, 0.0, 1.0),
            &p3(4.0, 0.0, 2.0),
            APPROX,
        )
        .value()
        .expect("above box should decide");
        let reversed = crate::classify_plane_aabb3_report(
            &plane,
            &p3(1.0, 3.0, 1.0),
            &p3(0.0, 2.0, 0.0),
            APPROX,
        )
        .value()
        .expect("reversed caller bounds should preserve endpoint set");

        assert_eq!(below.relation, PlaneAabbRelation::Below);
        assert_eq!(below.upper_sign, Sign::Negative);
        assert_eq!(below.validate(APPROX), Ok(()));
        assert_eq!(above.relation, PlaneAabbRelation::Above);
        assert_eq!(above.lower_sign, Sign::Positive);
        assert_eq!(above.validate(APPROX), Ok(()));
        assert_eq!(reversed.relation, PlaneAabbRelation::Below);
        assert_eq!(reversed.validate(APPROX), Ok(()));

        let mut forged = above.clone();
        forged.relation = PlaneAabbRelation::Intersecting;
        assert_eq!(
            forged.validate(APPROX),
            Err(PlaneAabbReportValidationError::RelationMismatch)
        );
    }

    #[test]
    /// Pin the orientation convention: the counterclockwise xy triple labels positive z Below.
    fn classifies_point_oriented_plane_from_points() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);

        assert_eq!(
            crate::classify_point_oriented_plane(&a, &b, &c, &p3(0.0, 0.0, 1.0), APPROX).value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    /// Check exact coefficient masks/schedules and retained-versus-immediate side-value agreement.
    fn plane_evidence_preserves_coefficient_structure() {
        let plane = Plane3::new(
            Point3::new(Real::from(0), Real::from(3), Real::from(0)),
            Real::from(-6),
        );
        let facts = plane.structural_facts();

        assert_eq!(facts.coefficient_zero_mask, 0b0101);
        assert_eq!(facts.coefficient_nonzero_mask, 0b1010);
        assert_eq!(facts.coefficient_unknown_zero_mask, 0);
        assert_eq!(facts.coefficient_zero_count(), 2);
        assert_eq!(facts.coefficient_nonzero_count(), 2);
        assert_eq!(facts.coefficient_unknown_zero_count(), 0);
        assert!(facts.normal_has_sparse_support());
        assert!(!facts.normal_known_zero());
        assert!(facts.has_dyadic_schedule());
        assert!(facts.has_shared_denominator_schedule());
        assert!(facts.coefficient_symbolic_dependencies.is_empty());

        let evidence = plane3_evidence(&plane);
        assert_eq!(evidence.facts(), facts);
        let point = Point3::new(0.into(), 3.into(), 0.into());
        assert_eq!(
            crate::classify_point_plane_with_evidence(&point, &plane, &evidence, APPROX).value(),
            crate::classify_point_plane(&point, &plane, APPROX).value()
        );
    }

    #[test]
    /// Require exact On/Exact-stage receipts for 3/3 + 5/5 + 7/7 - 3 in both plane paths.
    fn exact_rational_plane_cascade_preserves_cancellation_and_evidence() {
        let plane = Plane3::new(
            Point3::new(rational(1, 3), rational(1, 5), rational(1, 7)),
            Real::from(-3),
        );
        let point = Point3::new(Real::from(3), Real::from(5), Real::from(7));
        let evidence = plane3_evidence(&plane);
        let expected =
            PredicateOutcome::decided(PlaneSide::On, Certainty::Exact, Escalation::Exact);

        assert_eq!(
            crate::classify_point_plane(&point, &plane, APPROX),
            expected
        );
        assert_eq!(
            crate::classify_point_plane_with_evidence(&point, &plane, &evidence, APPROX),
            expected
        );
    }

    #[test]
    /// Compare 25 rational query side values with a materialized-expression sign from the same scalar kernel.
    /// This checks path agreement, not an independent sign oracle or complete receipt equality.
    fn exact_rational_plane_cascade_matches_materialized_expression_signs() {
        let plane = Plane3::new(
            Point3::new(rational(17, 19), rational(-23, 29), rational(31, 37)),
            rational(-41, 43),
        );
        let evidence = plane3_evidence(&plane);

        for index in -12_i64..=12 {
            let point = Point3::new(
                rational(index, 47),
                rational(2 * index - 1, 53),
                rational(3 * index + 2, 59),
            );
            let expression = point_plane_expression(&point, &plane, None);
            let expected = map_outcome(
                crate::resolve::resolve_real_sign_direct(
                    &expression,
                    PredicatePolicy::STRICT,
                    RefinementNeed::RealRefinement,
                ),
                PlaneSide::from,
            );

            assert_eq!(
                crate::classify_point_plane(&point, &plane, APPROX).value(),
                expected.value()
            );
            assert_eq!(
                crate::classify_point_plane_with_evidence(&point, &plane, &evidence, APPROX)
                    .value(),
                expected.value()
            );
        }
    }

    #[test]
    /// Compare three binary64 query side values from retained and direct orientation routes.
    /// Calling the word helper here does not itself prove a word-filter hit; it can fall back.
    fn oriented_plane_evidence_matches_orient3d_side() {
        let a = p3(-0.85, -0.7, -0.25);
        let b = p3(0.9, -0.35, 0.35);
        let c = p3(-0.35, 0.85, 0.05);
        let evidence = oriented_plane3_evidence(&a, &b, &c);
        assert_eq!(evidence.facts(), evidence.plane().structural_facts());

        for point in [
            p3(0.2, -0.1, 0.8),
            p3(-0.4, 0.3, -0.2),
            p3(0.1, 0.2, 0.38 * 0.1 + 0.24 * 0.2),
        ] {
            assert_eq!(
                classify_point_oriented_plane_exact_word(&point, &evidence, APPROX).value(),
                crate::classify_point_oriented_plane(&a, &b, &c, &point, APPROX).value()
            );
            assert_eq!(
                crate::classify_point_oriented_plane_with_evidence(&point, &evidence, APPROX)
                    .value(),
                crate::classify_point_oriented_plane(&a, &b, &c, &point, APPROX).value()
            );
        }
    }

    #[test]
    /// Require rational source word evidence and full retained/direct outcome equality for a pi query.
    fn oriented_plane_exact_word_evidence_falls_back_for_symbolic_queries() {
        let a = Point3::new(rational(1, 3), Real::from(0), Real::from(0));
        let b = Point3::new(Real::from(0), rational(1, 5), Real::from(0));
        let c = Point3::new(Real::from(0), Real::from(0), rational(1, 7));
        let point = Point3::new(Real::pi(), Real::from(0), Real::from(0));
        let evidence = oriented_plane3_evidence(&a, &b, &c);

        assert!(evidence.filter.is_none());
        assert!(evidence.exact_word_filter.is_some());
        assert_eq!(
            crate::classify_point_oriented_plane_with_evidence(&point, &evidence, APPROX),
            crate::classify_point_oriented_plane(&a, &b, &c, &point, APPROX)
        );
    }

    #[test]
    /// Require the exact structural Below receipt for pi - 4.
    fn classifies_point_plane_with_hyperreal_structural_facts() {
        use crate::predicate::{Certainty, Escalation};

        let plane = Plane3::new(Point3::new(Real::from(1), 0.into(), 0.into()), (-4).into());
        let point = Point3::new(Real::pi(), 0.into(), 0.into());

        assert_eq!(
            crate::classify_point_plane(&point, &plane, APPROX),
            PredicateOutcome::decided(PlaneSide::Below, Certainty::Exact, Escalation::Structural)
        );
    }

    #[test]
    /// Check PI/TRIG/EXP dependency-family propagation through plane facts and retained evidence.
    fn plane_evidence_summarizes_symbolic_dependencies() {
        let trig = (Real::from(hyperreal::Rational::fraction(1, 5).unwrap()) * Real::pi()).sin();
        let plane = Plane3::new(Point3::new(Real::pi(), trig, 0.into()), Real::e());
        let facts = plane.structural_facts();

        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(SymbolicDependencyMask::PI)
        );
        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(SymbolicDependencyMask::TRIG)
        );
        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(SymbolicDependencyMask::EXP)
        );

        let evidence = plane3_evidence(&plane);
        assert_eq!(
            evidence.facts().coefficient_symbolic_dependencies,
            facts.coefficient_symbolic_dependencies
        );
    }

    #[test]
    /// Exercise strict unknowns at segment endpoints, triangle vertices, box terms and extrema, plus receipt rankings.
    fn plane_unknown_paths_and_helper_rankings_are_explicit() {
        let uncertain = terminal_zero();
        let uncertain_plane = Plane3::new(
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            uncertain.clone(),
        );

        assert!(matches!(
            classify_plane_segment_with_policy(
                &uncertain_plane,
                &p3(0.0, 0.0, 0.0),
                &p3(0.0, 0.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_plane_segment_with_policy(
                &uncertain_plane,
                &p3(0.0, 0.0, -1.0),
                &p3(0.0, 0.0, 0.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_plane_triangle_with_policy(
                &uncertain_plane,
                &p3(0.0, 0.0, -1.0),
                &p3(1.0, 0.0, 0.0),
                &p3(0.0, 1.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let uncertain_axis_plane = Plane3::new(
            Point3::new(uncertain.clone(), Real::zero(), Real::zero()),
            Real::zero(),
        );
        assert!(matches!(
            classify_plane_aabb3_report_with_policy(
                &uncertain_axis_plane,
                &p3(0.0, 0.0, 0.0),
                &p3(1.0, 1.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_plane_aabb3_with_policy(
                &uncertain_axis_plane,
                &p3(0.0, 0.0, 0.0),
                &p3(1.0, 1.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_plane_aabb3_report_with_policy(
                &uncertain_plane,
                &p3(0.0, 0.0, 0.0),
                &p3(0.0, 0.0, 0.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_plane_aabb3_report_with_policy(
                &uncertain_plane,
                &p3(0.0, 0.0, 0.0),
                &p3(0.0, 0.0, 1.0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let facts = Plane3::new(
            Point3::new(uncertain.clone(), Real::zero(), Real::one()),
            Real::zero(),
        )
        .structural_facts();
        assert_eq!(facts.coefficient_unknown_zero_mask, 0b0001);

        assert_eq!(
            max_certainty(Certainty::Filtered, Certainty::Exact),
            Certainty::Filtered
        );
        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Approximate),
            Certainty::Approximate
        );
        assert_eq!(certainty_rank(Certainty::Exact), 0);
        assert_eq!(certainty_rank(Certainty::Filtered), 1);
        assert_eq!(certainty_rank(Certainty::Approximate), 2);
        assert_eq!(
            max_stage(Escalation::Exact, Escalation::Filter),
            Escalation::Exact
        );
        assert_eq!(
            max_stage(Escalation::Structural, Escalation::Refined),
            Escalation::Refined
        );
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);
    }

    #[test]
    /// Exercise symbolic filter declines, exact-word completion, changed-triangle replay and unresolved-order validation.
    /// The unresolved comparison intentionally maps to the existing extremum-order mismatch variant.
    fn plane_evidence_and_report_replay_cover_fallback_and_mismatch_paths() {
        let plane = Plane3::new(
            Point3::new(Real::pi(), Real::e(), Real::from(2).sqrt().unwrap()),
            Real::one().sin(),
        );
        let point = Point3::new(Real::from(1), Real::from(2), Real::from(3));
        let evidence = plane3_evidence(&plane);
        assert!(evidence.filter.is_none());
        assert!(evidence.rational_filter.is_none());
        assert_eq!(
            classify_point_plane_with_evidence_and_policy(&point, &plane, &evidence, APPROX),
            classify_point_plane_with_policy(&point, &plane, APPROX)
        );

        let a = Point3::new(rational(1, 3), Real::zero(), Real::zero());
        let b = Point3::new(Real::zero(), rational(1, 5), Real::zero());
        let c = Point3::new(Real::zero(), Real::zero(), rational(1, 7));
        let exact_word = oriented_plane3_evidence(&a, &b, &c);
        assert!(exact_word.filter.is_none());
        assert!(exact_word.exact_word_filter.is_some());
        let exact_query = p3(0.0, 0.0, 0.0);
        assert!(matches!(
            classify_point_oriented_plane_exact_word(&exact_query, &exact_word, APPROX),
            PredicateOutcome::Decided {
                stage: Escalation::Exact,
                ..
            }
        ));

        let face = [p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)];
        let above = [p3(0.0, 0.0, 1.0), p3(1.0, 0.0, 1.0), p3(0.0, 1.0, 1.0)];
        let below = [p3(0.0, 0.0, -1.0), p3(1.0, 0.0, -1.0), p3(0.0, 1.0, -1.0)];
        let report = classify_triangle_against_oriented_plane_with_policy(
            [&face[0], &face[1], &face[2]],
            [&above[0], &above[1], &above[2]],
            APPROX,
        );
        assert_eq!(
            report.validate_against_triangles(
                [&face[0], &face[1], &face[2]],
                [&below[0], &below[1], &below[2]],
                APPROX,
            ),
            Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
        );
        let points = [
            face[0].clone(),
            face[1].clone(),
            face[2].clone(),
            below[0].clone(),
            below[1].clone(),
            below[2].clone(),
        ];
        assert_eq!(
            report.validate_against_sources(&points, [0, 1, 2], [3, 4, 5], APPROX),
            Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
        );

        let base = classify_plane_aabb3_report_with_policy(
            &Plane3::new(p3(0.0, 0.0, 1.0), Real::zero()),
            &p3(0.0, 0.0, 0.0),
            &p3(1.0, 1.0, 1.0),
            APPROX,
        )
        .value()
        .unwrap();
        let mut unknown_order = base;
        unknown_order.lower_value = terminal_zero();
        unknown_order.upper_value = Real::zero();
        assert_eq!(
            unknown_order.validate(PredicatePolicy::STRICT),
            Err(PlaneAabbReportValidationError::ExtremumOrderMismatch)
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Require one rational-filter trace hit and zero generic/known-shared-denominator product-sum hits.
    /// The fifths fixture has an independent negative plane-expression expectation under STRICT.
    fn point_plane_evidence_reuses_rational_linear_form_filter() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let fifth = |value| Real::from(Rational::fraction(value, 5).unwrap());
        let plane = Plane3::new(Point3::new(fifth(2), fifth(-3), fifth(4)), fifth(-6));
        let point = Point3::new(fifth(5), fifth(5), fifth(5));
        let evidence = plane3_evidence(&plane);

        hyperreal::dispatch_trace::reset();
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            classify_point_plane_with_evidence_and_policy(
                &point,
                &plane,
                &evidence,
                PredicatePolicy::STRICT,
            )
        });

        assert_eq!(outcome.value(), Some(PlaneSide::Below));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "classify_point_plane",
                "evidence-rational-linear-form-filter"
            ),
            1
        );
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "classify_point_plane",
                "fixed-real-product-sum"
            ),
            0
        );
        assert_eq!(
            trace.path_count("real", "product_sum", "exact-rational-known-shared-denom"),
            0
        );
    }
}
