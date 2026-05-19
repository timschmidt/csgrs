//! Report-bearing region boolean entry points.
//!
//! The ordinary boolean APIs return geometry directly.  This module keeps that
//! surface intact and adds an auditable wrapper that records the operation and
//! replays exact boundary validation on the returned region.  This follows Yap's
//! system-level exactness requirement: a geometric construction is not
//! "exact" merely because its scalar coordinates are exact; the constructed
//! combinatorial object must also expose the predicates that justify its use
//! (C. K. Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997, <https://doi.org/10.1016/0925-7721(95)00040-2>).

use crate::{
    ArcArcIntersection, BooleanBoundaryLoopSet, BooleanOp, BoundaryContourNestingAuditReport2,
    Classification, Contour2, CurvePolicy, CurveResult, FillRule, LineArcIntersection,
    LineLineIntersection, Region2, RegionView2, SegmentIntersection, prepared::PreparedRegionView2,
};

/// Exact boundary-audit status for boolean boundary output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryAuditStatus {
    /// The operation produced no boundary contours.
    Empty,
    /// Every output contour is free of self contacts and no two output contours
    /// have a boundary contact under the active exact policy.
    Valid,
    /// At least one output contour has a non-adjacent self contact.
    SelfContact,
    /// Two distinct output contours touch, cross, or overlap.
    InterContourContact,
}

impl BooleanBoundaryAuditStatus {
    /// Returns true when the status certifies a usable regularized boundary.
    pub const fn is_valid(self) -> bool {
        matches!(self, Self::Empty | Self::Valid)
    }
}

/// Exact boundary-audit status for a boolean region result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanRegionAuditStatus {
    /// The operation produced the empty regularized set.
    Empty,
    /// Every result contour is free of self contacts and no two result contours
    /// have a boundary contact under the active exact policy.
    Valid,
    /// At least one result contour has a non-adjacent self contact.
    SelfContact,
    /// Two distinct result contours touch, cross, or overlap.
    InterContourContact,
}

impl From<BooleanBoundaryAuditStatus> for BooleanRegionAuditStatus {
    fn from(status: BooleanBoundaryAuditStatus) -> Self {
        match status {
            BooleanBoundaryAuditStatus::Empty => Self::Empty,
            BooleanBoundaryAuditStatus::Valid => Self::Valid,
            BooleanBoundaryAuditStatus::SelfContact => Self::SelfContact,
            BooleanBoundaryAuditStatus::InterContourContact => Self::InterContourContact,
        }
    }
}

/// Exact boundary-audit report for boundary-contour boolean output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BooleanBoundaryContourAuditReport2 {
    /// Final audit status.
    pub status: BooleanBoundaryAuditStatus,
    /// Number of output contours checked.
    pub contour_count: usize,
    /// Number of unordered contour pairs checked for boundary contact.
    pub checked_contour_pair_count: usize,
}

impl BooleanBoundaryContourAuditReport2 {
    /// Returns true when the audit certifies a usable regularized boundary.
    pub const fn is_valid(&self) -> bool {
        self.status.is_valid()
    }
}

/// Exact boundary-audit report for raw boolean boundary loops.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BooleanBoundaryLoopAuditReport2 {
    /// Final audit status.
    pub status: BooleanBoundaryAuditStatus,
    /// Number of output loops converted through the checked contour boundary.
    pub loop_count: usize,
    /// Number of unordered loop pairs checked for boundary contact.
    pub checked_loop_pair_count: usize,
}

impl BooleanBoundaryLoopAuditReport2 {
    /// Returns true when the audit certifies a usable regularized boundary.
    pub const fn is_valid(&self) -> bool {
        self.status.is_valid()
    }
}

/// Exact boundary-audit report for a boolean region result.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BooleanRegionAuditReport2 {
    /// Final audit status.
    pub status: BooleanRegionAuditStatus,
    /// Number of material contours in the result.
    pub material_contour_count: usize,
    /// Number of hole contours in the result.
    pub hole_contour_count: usize,
    /// Number of individual contours checked.
    pub checked_contour_count: usize,
    /// Number of unordered contour pairs checked for boundary contact.
    pub checked_contour_pair_count: usize,
}

impl BooleanRegionAuditReport2 {
    /// Returns true when the audit certifies a usable regularized boundary.
    pub const fn is_valid(&self) -> bool {
        matches!(
            self.status,
            BooleanRegionAuditStatus::Empty | BooleanRegionAuditStatus::Valid
        )
    }
}

/// Report-bearing result of a region boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanRegionReport2 {
    /// Requested operation.
    pub operation: BooleanOp,
    /// Role-assigned boolean result.
    pub result: Region2,
    /// Exact audit over the produced boundary contours.
    pub audit: BooleanRegionAuditReport2,
}

/// Report-bearing result of the complete contour-to-region boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanRegionPipelineReport2 {
    /// Requested operation.
    pub operation: BooleanOp,
    /// Boundary contours produced by boolean traversal before role assignment.
    pub boundary_contours: Vec<Contour2>,
    /// Exact audit over the boundary-contour product.
    pub boundary_audit: BooleanBoundaryContourAuditReport2,
    /// Exact audit over material/hole role assignment.
    pub nesting_audit: BoundaryContourNestingAuditReport2,
    /// Role-assigned boolean result reconstructed from `boundary_contours`.
    pub result: Region2,
    /// Exact audit over the reconstructed region boundary.
    pub region_audit: BooleanRegionAuditReport2,
}

/// Report-bearing result of a boundary-contour boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryContourReport2 {
    /// Requested operation.
    pub operation: BooleanOp,
    /// Checked boundary contours before material/hole role assignment.
    pub contours: Vec<Contour2>,
    /// Exact audit over the returned boundary contours.
    pub audit: BooleanBoundaryContourAuditReport2,
}

/// Report-bearing result of a raw boundary-loop boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoopReport2 {
    /// Requested operation.
    pub operation: BooleanOp,
    /// Checked closed boundary loops before material/hole role assignment.
    pub loops: BooleanBoundaryLoopSet,
    /// Exact audit over the loops after checked contour conversion.
    pub audit: BooleanBoundaryLoopAuditReport2,
}

impl Region2 {
    /// Computes a boolean region and returns a boundary-audited report.
    ///
    /// This is a certificate-bearing wrapper around [`Region2::boolean_region`].
    /// The boolean construction still uses the existing split/classify/traverse
    /// pipeline; after construction, the result boundary is replayed through
    /// exact self-contact and contour-pair predicates.  Greiner and Hormann's
    /// traversal model assumes that classified chains form valid result
    /// contours after intersection insertion (G. Greiner and K. Hormann,
    /// "Efficient clipping of arbitrary polygons," *ACM Transactions on
    /// Graphics* 17(2), 1998).  The audit makes that postcondition explicit for
    /// callers instead of leaving it implicit in the returned geometry.
    pub fn boolean_region_report(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionReport2>> {
        self.as_view()
            .boolean_region_report(&other.as_view(), op, fill_rule, policy)
    }

    /// Computes a boolean region and reports every exact acceptance stage.
    ///
    /// This full-pipeline report first exposes the boundary-contour product,
    /// then the exact nesting replay that assigns material/hole roles, and
    /// finally the reconstructed region-boundary audit. Vatti's clipping model
    /// treats boolean output as boundary transitions (B. R. Vatti, "A generic
    /// solution to polygon clipping," *Communications of the ACM* 35(7), 1992);
    /// Yap's exact-geometric-computation model requires the constructed object
    /// facts to remain certified at the API boundary. This report keeps both
    /// stages visible.
    pub fn boolean_region_pipeline_report(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
        self.as_view()
            .boolean_region_pipeline_report(&other.as_view(), op, fill_rule, policy)
    }

    /// Computes checked boolean boundary contours and returns an audit report.
    ///
    /// This is the contour-level counterpart to [`Region2::boolean_region_report`].
    /// It validates the returned contours without assigning material/hole roles,
    /// which is useful for callers that consume raw boundaries.  Vatti's
    /// scanline formulation builds boolean results as fill-state boundary
    /// transitions (B. R. Vatti, "A generic solution to polygon clipping,"
    /// *Communications of the ACM* 35(7), 1992); this report keeps that
    /// boundary product explicit and then applies Yap's rule that object-level
    /// topology must be certified before downstream use.
    pub fn boolean_boundary_contour_report(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryContourReport2>> {
        self.as_view()
            .boolean_boundary_contour_report(&other.as_view(), op, fill_rule, policy)
    }

    /// Computes boolean boundary loops and returns an audit report.
    ///
    /// This wraps [`Region2::boolean_boundary_loops`] without changing its
    /// uncertainty model.  Decided loops are converted through
    /// [`BooleanBoundaryLoopSet::to_contours`] using `fill_rule`, and those
    /// checked contours are audited for self contacts and loop-pair contacts.
    /// The conversion step follows Foster, Hormann, and Popa's warning that
    /// clipping with degenerate intersections must validate boundary
    /// coincidences explicitly (2019), while the report surface follows Yap's
    /// object-level certification discipline (1997).
    pub fn boolean_boundary_loop_report(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
        self.as_view()
            .boolean_boundary_loop_report(&other.as_view(), op, fill_rule, policy)
    }
}

impl RegionView2<'_> {
    /// Computes a boolean region and returns a boundary-audited report.
    ///
    /// The audit checks the returned contours with the same certified predicate
    /// boundary used by boolean construction.  Any uncertainty is propagated as
    /// [`Classification::Uncertain`] rather than converted to a tolerance-based
    /// success.  This is the exact-computation discipline advocated by Yap
    /// (1997): approximate or unresolved facts must remain explicit at the
    /// geometric-object layer.
    pub fn boolean_region_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionReport2>> {
        let result = match self.boolean_region(other, op, fill_rule, policy)? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let audit = match audit_boolean_region_result(&result, policy)? {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        Ok(Classification::Decided(BooleanRegionReport2 {
            operation: op,
            result,
            audit,
        }))
    }

    /// Computes checked boolean boundary contours and returns an audit report.
    ///
    /// The construction path is [`RegionView2::boolean_boundary_contours`].
    /// The audit then replays exact self-contact and contour-pair predicates on
    /// the returned contours.  Foster, Hormann, and Popa show that degenerate
    /// clipping needs explicit treatment of shared boundaries ("Clipping simple
    /// polygons with degenerate intersections," *Computers & Graphics: X* 2,
    /// 2019); this method makes the post-regularization contour validity
    /// visible instead of relying on implicit caller trust.
    pub fn boolean_boundary_contour_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryContourReport2>> {
        let contours = match self.boolean_boundary_contours(other, op, fill_rule, policy)? {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let audit = match audit_boolean_boundary_contours(&contours, policy)? {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        Ok(Classification::Decided(BooleanBoundaryContourReport2 {
            operation: op,
            contours,
            audit,
        }))
    }

    /// Computes a boolean region and reports boundary, nesting, and region audits.
    ///
    /// The construction path is [`RegionView2::boolean_boundary_contours`]
    /// followed by [`Region2::from_boundary_contours_report`]. Boundary hits in
    /// the nesting replay remain explicit uncertainty, following Hormann and
    /// Agathos' treatment of point-in-polygon boundary degeneracies
    /// (K. Hormann and A. Agathos, "The point in polygon problem for arbitrary
    /// polygons," *Computational Geometry* 20(3), 2001).
    pub fn boolean_region_pipeline_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
        let contours = match self.boolean_boundary_contours(other, op, fill_rule, policy)? {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        boolean_region_pipeline_report_from_contours(op, contours, policy)
    }

    /// Computes boolean boundary loops and returns an audit report.
    ///
    /// The construction path is [`RegionView2::boolean_boundary_loops`].  That
    /// API intentionally reports shared-boundary cases as
    /// [`Classification::Uncertain`]; this wrapper preserves that result.  For
    /// decided loops, `fill_rule` is used only to build checked contours for
    /// validation, not to alter loop traversal.
    pub fn boolean_boundary_loop_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
        let loops = match self.boolean_boundary_loops(other, op, policy)? {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        boolean_boundary_loop_report_from_loops(op, loops, fill_rule, policy)
    }

    /// Computes a boolean region against a prepared right operand and audits it.
    ///
    /// This preserves operation order as `self op other`, prepares only the
    /// left operand transiently, and then uses the prepared boolean pipeline.
    /// Prepared caches are object-level reuse data, not topology evidence by
    /// themselves; the returned audit still replays exact self-contact and
    /// contour-pair predicates on the constructed result, matching Yap's
    /// separation between approximate/filter layers and certified geometric
    /// object facts.
    pub fn boolean_region_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionReport2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_region_report(other, op, fill_rule, policy)
    }

    /// Computes a full boolean pipeline report against a prepared right operand.
    pub fn boolean_region_pipeline_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_region_pipeline_report(other, op, fill_rule, policy)
    }

    /// Computes checked boundary contours against a prepared right operand and
    /// returns an audit report.
    ///
    /// This preserves operation order as `self op other`; only the left operand
    /// is prepared transiently.  Prepared caches can prune decided misses, but
    /// the report's acceptance audit is still the exact contour audit used by
    /// the non-prepared path.
    pub fn boolean_boundary_contour_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryContourReport2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_contour_report(other, op, fill_rule, policy)
    }

    /// Computes audited boolean boundary loops against a prepared right operand.
    ///
    /// This preserves operation order as `self op other`; only the left operand
    /// is prepared transiently.  The audit still converts the returned loops
    /// through checked contours so prepared caches are not treated as topology
    /// proof.
    pub fn boolean_boundary_loop_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_loop_report(other, op, fill_rule, policy)
    }
}

impl PreparedRegionView2<'_> {
    /// Computes a boolean region against another prepared region and audits it.
    ///
    /// The construction stage is the same prepared split/classify/traverse path
    /// as [`PreparedRegionView2::boolean_region`]. The audit then validates the
    /// returned boundary without using prepared caches as proof: every contour
    /// self-contact and every contour-pair contact is replayed through the
    /// exact native segment predicates. This keeps Greiner-Hormann style
    /// boolean traversal as a construction step and Yap-style certified object
    /// facts as the acceptance step.
    pub fn boolean_region_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionReport2>> {
        let result = match self.boolean_region(other, op, fill_rule, policy)? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let audit = match audit_boolean_region_result(&result, policy)? {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        Ok(Classification::Decided(BooleanRegionReport2 {
            operation: op,
            result,
            audit,
        }))
    }

    /// Computes checked boolean boundary contours against another prepared
    /// region and returns an audit report.
    ///
    /// The prepared construction reuses cached broad-phase data; the audit does
    /// not. It replays exact native contour predicates so the certificate
    /// remains an object-layer fact rather than a cache-derived assumption, in
    /// the sense of Yap's exact geometric computation stack.
    pub fn boolean_boundary_contour_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryContourReport2>> {
        let contours = match self.boolean_boundary_contours(other, op, fill_rule, policy)? {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let audit = match audit_boolean_boundary_contours(&contours, policy)? {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        Ok(Classification::Decided(BooleanBoundaryContourReport2 {
            operation: op,
            contours,
            audit,
        }))
    }

    /// Computes a prepared boolean region and reports every acceptance stage.
    ///
    /// Prepared traversal supplies the boundary-contour construction. The
    /// certificate remains a fresh replay over contours, nesting, and final
    /// region boundary, so cached broad-phase data is never treated as proof.
    pub fn boolean_region_pipeline_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
        let contours = match self.boolean_boundary_contours(other, op, fill_rule, policy)? {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        boolean_region_pipeline_report_from_contours(op, contours, policy)
    }

    /// Computes boolean boundary loops against another prepared region and
    /// returns an audit report.
    ///
    /// Prepared traversal supplies the construction; the certificate is still a
    /// fresh exact replay over checked contour geometry.  This matches Yap's
    /// distinction between acceleration/filtering data and certified geometric
    /// object facts.
    pub fn boolean_boundary_loop_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
        let loops = match self.boolean_boundary_loops(other, op, policy)? {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        boolean_boundary_loop_report_from_loops(op, loops, fill_rule, policy)
    }

    /// Computes a boolean region against an ordinary region view and audits it.
    ///
    /// The right operand is prepared transiently for this call. The final audit
    /// is identical to [`PreparedRegionView2::boolean_region_report`], so
    /// mixed prepared/unprepared callers receive the same certificate surface
    /// as fully prepared callers.
    pub fn boolean_region_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionReport2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_region_report(&other, op, fill_rule, policy)
    }

    /// Computes a full boolean pipeline report against an ordinary region view.
    pub fn boolean_region_pipeline_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: crate::FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_region_pipeline_report(&other, op, fill_rule, policy)
    }

    /// Computes checked boundary contours against an ordinary region view and
    /// returns an audit report.
    ///
    /// The right operand is prepared transiently.  The returned report has the
    /// same audit semantics as [`PreparedRegionView2::boolean_boundary_contour_report`].
    pub fn boolean_boundary_contour_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryContourReport2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_contour_report(&other, op, fill_rule, policy)
    }

    /// Computes audited boolean boundary loops against an ordinary region view.
    ///
    /// The right operand is prepared transiently.  The returned report has the
    /// same audit semantics as [`PreparedRegionView2::boolean_boundary_loop_report`].
    pub fn boolean_boundary_loop_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_loop_report(&other, op, fill_rule, policy)
    }
}

fn boolean_region_pipeline_report_from_contours(
    op: BooleanOp,
    contours: Vec<Contour2>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanRegionPipelineReport2>> {
    let boundary_audit = match audit_boolean_boundary_contours(&contours, policy)? {
        Classification::Decided(audit) => audit,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let nesting = match Region2::from_boundary_contours_report(contours.clone(), policy)? {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let region_audit = match audit_boolean_region_result(&nesting.result, policy)? {
        Classification::Decided(audit) => audit,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(Classification::Decided(BooleanRegionPipelineReport2 {
        operation: op,
        boundary_contours: contours,
        boundary_audit,
        nesting_audit: nesting.audit,
        result: nesting.result,
        region_audit,
    }))
}

fn boolean_boundary_loop_report_from_loops(
    op: BooleanOp,
    loops: BooleanBoundaryLoopSet,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopReport2>> {
    let contours = loops.to_contours(fill_rule)?;
    let contour_audit = match audit_boolean_boundary_contours(&contours, policy)? {
        Classification::Decided(audit) => audit,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(Classification::Decided(BooleanBoundaryLoopReport2 {
        operation: op,
        loops,
        audit: BooleanBoundaryLoopAuditReport2 {
            status: contour_audit.status,
            loop_count: contour_audit.contour_count,
            checked_loop_pair_count: contour_audit.checked_contour_pair_count,
        },
    }))
}

fn audit_boolean_region_result(
    result: &Region2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanRegionAuditReport2>> {
    let contours = result
        .material_contours()
        .iter()
        .chain(result.hole_contours().iter())
        .collect::<Vec<_>>();
    let boundary_audit = match audit_boolean_contour_refs(&contours, policy)? {
        Classification::Decided(audit) => audit,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(Classification::Decided(BooleanRegionAuditReport2 {
        status: boundary_audit.status.into(),
        material_contour_count: result.material_contours().len(),
        hole_contour_count: result.hole_contours().len(),
        checked_contour_count: boundary_audit.contour_count,
        checked_contour_pair_count: boundary_audit.checked_contour_pair_count,
    }))
}

fn audit_boolean_boundary_contours(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryContourAuditReport2>> {
    let contour_refs = contours.iter().collect::<Vec<_>>();
    audit_boolean_contour_refs(&contour_refs, policy)
}

fn audit_boolean_contour_refs(
    contours: &[&Contour2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryContourAuditReport2>> {
    if contours.is_empty() {
        return Ok(Classification::Decided(
            BooleanBoundaryContourAuditReport2 {
                status: BooleanBoundaryAuditStatus::Empty,
                contour_count: 0,
                checked_contour_pair_count: 0,
            },
        ));
    }

    for contour in contours {
        match contour.has_self_contacts(policy)? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(Classification::Decided(
                    BooleanBoundaryContourAuditReport2 {
                        status: BooleanBoundaryAuditStatus::SelfContact,
                        contour_count: contours.len(),
                        checked_contour_pair_count: 0,
                    },
                ));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let mut checked_pairs = 0_usize;
    for first_index in 0..contours.len() {
        for second_index in (first_index + 1)..contours.len() {
            checked_pairs += 1;
            let events = contours[first_index]
                .curve_string()
                .intersect_curve_string(contours[second_index].curve_string(), policy)?;
            for event in events {
                match segment_intersection_has_contact(&event.relation) {
                    Classification::Decided(false) => {}
                    Classification::Decided(true) => {
                        return Ok(Classification::Decided(
                            BooleanBoundaryContourAuditReport2 {
                                status: BooleanBoundaryAuditStatus::InterContourContact,
                                contour_count: contours.len(),
                                checked_contour_pair_count: checked_pairs,
                            },
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }

    Ok(Classification::Decided(
        BooleanBoundaryContourAuditReport2 {
            status: BooleanBoundaryAuditStatus::Valid,
            contour_count: contours.len(),
            checked_contour_pair_count: checked_pairs,
        },
    ))
}

fn segment_intersection_has_contact(relation: &SegmentIntersection) -> Classification<bool> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None) => Classification::Decided(false),
        SegmentIntersection::LineLine(LineLineIntersection::Point { .. })
        | SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. }) => {
            Classification::Decided(true)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
            Classification::Uncertain(*reason)
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::None,
            ..
        } => Classification::Decided(false),
        SegmentIntersection::LineArc {
            result:
                LineArcIntersection::Point(_)
                | LineArcIntersection::TwoPoints {
                    first: _,
                    second: _,
                },
            ..
        } => Classification::Decided(true),
        SegmentIntersection::LineArc {
            result: LineArcIntersection::Uncertain { reason },
            ..
        } => Classification::Uncertain(*reason),
        SegmentIntersection::ArcArc(ArcArcIntersection::None) => Classification::Decided(false),
        SegmentIntersection::ArcArc(
            ArcArcIntersection::Point(_)
            | ArcArcIntersection::TwoPoints {
                first: _,
                second: _,
            }
            | ArcArcIntersection::Overlap { .. },
        ) => Classification::Decided(true),
        SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
            Classification::Uncertain(*reason)
        }
    }
}
