//! Prepared SDF classifiers.
//!
//! Prepared handles are the SDF-level analogue of `hyperlimit` prepared
//! predicates: they retain object shape and report exact evidence instead of
//! collapsing a query to primitive floats. The cell routines deliberately use
//! stronger primitive predicates when available, including the exact AABB/plane
//! and AABB/sphere tests inspired by classical box culling and by Yap's
//! exact-computation separation.

use hyperlimit::{
    Aabb3Intersection, AabbSphereIntersection, Certainty, Escalation, PlaneAabbRelation, Point3,
    PredicateOutcome, classify_aabb3_intersection, classify_aabb3_sphere_intersection,
    classify_plane_aabb3, classify_point_aabb3, classify_point_sphere3, compare_reals,
};
use std::cmp::Ordering;

use crate::batch::{
    SdfBatchDispatch, SdfCachePayoffReport, SdfCellBatchClassificationReport,
    SdfPointBatchClassificationReport,
};
use crate::dual_contour::{SdfDualContouringReport, dual_contouring_report_from_exact_sdf};
use crate::expr::SdfExpr;
use crate::facts::SdfFacts;
use crate::gradient::{
    SdfGradientReport, SdfNormalReport, gradient_expr_point, normal_from_gradient_report,
};
use crate::gradient_contour::{
    SdfGradientContourReport, gradient_contour_report_from_prepared_grid,
};
use crate::handoff::SdfVoxelHandoffReport;
use crate::interval::{SdfIntervalReport, interval_expr_cell};
use crate::lipschitz::{SdfLipschitzReport, lipschitz_expr_cell};
use crate::mesh::SdfMeshPreviewReport;
use crate::package::SdfHandoffPackage;
use crate::primitive::SdfPrimitive;
use crate::primitive::radius_squared_domain;
use crate::sampling::{
    SdfGridSamplingError, SdfGridSamplingReport, SdfPreviewGrid, SdfSamplingPrecision,
    SdfSamplingReport, sample_expr_grid_preview, sample_expr_points_preview, scalar_expr_point,
};
use crate::shader::{SdfShaderExportReport, export_expr_glsl_preview};
use crate::solver::{SdfProjectionProposal, SdfProjectionReplayReport};
use crate::status::{
    SdfCellClassificationReport, SdfCellLocation, SdfEvidenceStatus, SdfFreshness, SdfMetricStatus,
    SdfPointClassificationReport, SdfPointLocation,
};
use crate::voxel::{
    SdfHypervoxelHandoffReport, SdfVoxelCellGrid, SdfVoxelGridError, voxel_cell_bounds,
};

/// Prepared exact-aware SDF expression.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedSdf {
    expr: SdfExpr,
    facts: SdfFacts,
    prepared_source_version: Option<u64>,
    current_source_version: Option<u64>,
}

impl PreparedSdf {
    /// Prepare an expression for repeated classification.
    pub fn new(expr: SdfExpr) -> Self {
        let facts = SdfFacts::from_expr(&expr);
        Self {
            expr,
            facts,
            prepared_source_version: None,
            current_source_version: None,
        }
    }

    /// Prepare an expression that belongs to a caller-owned construction version.
    ///
    /// The version is metadata, not predicate evidence. It lets reports expose
    /// whether prepared facts are current relative to an external source object
    /// without changing the exact classification route; this mirrors Yap's
    /// separation between geometric-object packages and arithmetic decisions.
    pub fn new_versioned(expr: SdfExpr, source_version: u64) -> Self {
        let facts = SdfFacts::from_expr(&expr);
        Self {
            expr,
            facts,
            prepared_source_version: Some(source_version),
            current_source_version: Some(source_version),
        }
    }

    /// Return the retained expression.
    pub const fn expr(&self) -> &SdfExpr {
        &self.expr
    }

    /// Return cached structural facts for the retained expression.
    pub const fn facts(&self) -> &SdfFacts {
        &self.facts
    }

    /// Return the expression metric status.
    pub fn metric_status(&self) -> SdfMetricStatus {
        self.facts.metric_status
    }

    /// Return the source construction version used when this handle was prepared.
    pub const fn prepared_source_version(&self) -> Option<u64> {
        self.prepared_source_version
    }

    /// Return the latest source construction version known to this handle.
    pub const fn current_source_version(&self) -> Option<u64> {
        self.current_source_version
    }

    /// Return freshness relative to the known source construction version.
    pub const fn freshness(&self) -> SdfFreshness {
        match (self.prepared_source_version, self.current_source_version) {
            (None, _) => SdfFreshness::Unversioned,
            (Some(prepared), Some(current)) if prepared == current => SdfFreshness::Current,
            (Some(_), Some(_)) => SdfFreshness::Stale,
            (Some(_), None) => SdfFreshness::Current,
        }
    }

    /// Return a copy of this prepared handle with a refreshed source-version view.
    ///
    /// This is deliberately metadata-only: stale handles continue to answer
    /// exact predicates over the retained expression, but their reports carry
    /// `SdfFreshness::Stale` so downstream consumers can reject cached facts.
    pub fn with_current_source_version(mut self, current_source_version: u64) -> Self {
        self.current_source_version = Some(current_source_version);
        self
    }

    /// Classify a point and return a report rather than a bare boolean.
    pub fn classify_point(&self, point: &Point3) -> SdfPointClassificationReport {
        let outcome = classify_expr_point(&self.expr, point);
        let scalar_value = scalar_expr_point(&self.expr, point);
        SdfPointClassificationReport {
            point: point.clone(),
            location: outcome.value().unwrap_or(SdfPointLocation::Unknown),
            scalar_value,
            metric_status: self.metric_status(),
            evidence: SdfEvidenceStatus::from_outcome(&outcome),
            freshness: self.freshness(),
        }
    }

    /// Classify many points through one retained prepared handle.
    ///
    /// This is a scheduling surface, not alternate topology semantics: every
    /// item is classified by [`PreparedSdf::classify_point`], so scalar and
    /// batch results must match exactly. Later SIMD, trace, or parallel
    /// evaluators can specialize this method without changing the report
    /// contract.
    pub fn classify_points<'a, I>(&self, points: I) -> Vec<SdfPointClassificationReport>
    where
        I: IntoIterator<Item = &'a Point3>,
    {
        self.classify_points_report(points).reports
    }

    /// Classify many points and include dispatch/cache-payoff metadata.
    ///
    /// The current dispatch is scalar replay over one prepared expression. The
    /// metadata is deliberately non-certifying: per-point reports are the only
    /// topology evidence, while dispatch labels and payoff counters explain how
    /// work was scheduled and how much retained structure was reused.
    pub fn classify_points_report<'a, I>(&self, points: I) -> SdfPointBatchClassificationReport
    where
        I: IntoIterator<Item = &'a Point3>,
    {
        let reports = points
            .into_iter()
            .map(|point| self.classify_point(point))
            .collect::<Vec<_>>();
        SdfPointBatchClassificationReport {
            dispatch: SdfBatchDispatch::ScalarReplay,
            cache_payoff: self.cache_payoff(reports.len()),
            freshness: self.freshness(),
            reports,
        }
    }

    /// Return an exact symbolic gradient at a point when the active expression
    /// branch is certified.
    ///
    /// The result is solver/adapter evidence, not a topology decision. CSG ties,
    /// nonsmooth primitive branches, `sqrt` division domains, and affine
    /// covector transforms that are not yet represented return explicit
    /// unknown evidence instead of a tolerance-derived normal.
    pub fn gradient_point(&self, point: &Point3) -> SdfGradientReport {
        let outcome = gradient_expr_point(&self.expr, point);
        SdfGradientReport {
            point: point.clone(),
            gradient: outcome.clone().value(),
            gradient_status: self.facts.gradient_status,
            evidence: SdfEvidenceStatus::from_outcome(&outcome),
            freshness: self.freshness(),
        }
    }

    /// Return exact symbolic gradients for many points through one prepared handle.
    pub fn gradient_points<'a, I>(&self, points: I) -> Vec<SdfGradientReport>
    where
        I: IntoIterator<Item = &'a Point3>,
    {
        points
            .into_iter()
            .map(|point| self.gradient_point(point))
            .collect()
    }

    /// Return an exact unnormalized normal direction when a certified nonzero
    /// gradient is available at the query point.
    ///
    /// The report never normalizes through a square root or tolerance. A
    /// certified zero gradient, CSG tie, nonsmooth branch, or unsupported
    /// derivative route is reported explicitly.
    pub fn normal_point(&self, point: &Point3) -> SdfNormalReport {
        normal_from_gradient_report(self.gradient_point(point))
    }

    /// Return exact normal-direction reports for many points.
    pub fn normal_points<'a, I>(&self, points: I) -> Vec<SdfNormalReport>
    where
        I: IntoIterator<Item = &'a Point3>,
    {
        points
            .into_iter()
            .map(|point| self.normal_point(point))
            .collect()
    }

    /// Classify a closed AABB/cell conservatively.
    pub fn classify_cell(&self, min: &Point3, max: &Point3) -> SdfCellClassificationReport {
        let outcome = classify_expr_cell(&self.expr, min, max);
        SdfCellClassificationReport {
            min: min.clone(),
            max: max.clone(),
            location: outcome.value().unwrap_or(SdfCellLocation::Unknown),
            metric_status: self.metric_status(),
            evidence: SdfEvidenceStatus::from_outcome(&outcome),
            freshness: self.freshness(),
        }
    }

    /// Classify many closed AABB/cells through one retained prepared handle.
    ///
    /// Each pair is interpreted as `(min, max)` in the same inclusive sense as
    /// [`PreparedSdf::classify_cell`]. Hyperlimit interval predicates normalize
    /// endpoint order where the underlying primitive classifier supports it.
    pub fn classify_cells<'a, I>(&self, cells: I) -> Vec<SdfCellClassificationReport>
    where
        I: IntoIterator<Item = (&'a Point3, &'a Point3)>,
    {
        self.classify_cells_report(cells).reports
    }

    /// Classify many closed AABB/cells and include dispatch/cache-payoff metadata.
    pub fn classify_cells_report<'a, I>(&self, cells: I) -> SdfCellBatchClassificationReport
    where
        I: IntoIterator<Item = (&'a Point3, &'a Point3)>,
    {
        let reports = cells
            .into_iter()
            .map(|(min, max)| self.classify_cell(min, max))
            .collect::<Vec<_>>();
        SdfCellBatchClassificationReport {
            dispatch: SdfBatchDispatch::ScalarReplay,
            cache_payoff: self.cache_payoff(reports.len()),
            freshness: self.freshness(),
            reports,
        }
    }

    /// Compute a certified scalar interval over a closed AABB/cell when the
    /// retained expression has an exact interval route.
    pub fn interval_cell(&self, min: &Point3, max: &Point3) -> SdfIntervalReport {
        let outcome = interval_expr_cell(&self.expr, min, max);
        SdfIntervalReport {
            min: min.clone(),
            max: max.clone(),
            interval: outcome.clone().value(),
            evidence: SdfEvidenceStatus::from_outcome(&outcome),
            freshness: self.freshness(),
        }
    }

    /// Compute a conservative exact local Lipschitz bound over a closed AABB/cell.
    ///
    /// Missing bounds are reported as explicit unknown evidence. They are not
    /// replaced by sampled slopes or primitive-float finite differences.
    pub fn lipschitz_cell(&self, min: &Point3, max: &Point3) -> SdfLipschitzReport {
        let outcome = lipschitz_expr_cell(&self.expr, min, max);
        SdfLipschitzReport {
            min: min.clone(),
            max: max.clone(),
            bound: outcome.clone().value(),
            lipschitz_status: self.facts.lipschitz_status,
            evidence: SdfEvidenceStatus::from_outcome(&outcome),
            freshness: self.freshness(),
        }
    }

    /// Lower exact scalar views at points into primitive floats for previews.
    ///
    /// The returned report always marks samples as preview-only. Callers that
    /// need topology must replay point/cell predicates or consume a downstream
    /// exact replay report; lossy samples are never promoted by this API.
    pub fn sample_points_preview<'a, I>(
        &self,
        points: I,
        precision: SdfSamplingPrecision,
    ) -> SdfSamplingReport
    where
        I: IntoIterator<Item = &'a Point3>,
    {
        sample_expr_points_preview(
            &self.expr,
            points,
            precision,
            self.metric_status(),
            self.freshness(),
        )
    }

    /// Lower exact scalar views over a regular exact grid into primitive floats
    /// for previews.
    ///
    /// Grid points are constructed exactly from origin, per-axis step, and
    /// integer indices before scalar lowering. The returned samples remain
    /// preview-only and cannot certify topology.
    pub fn sample_grid_preview(
        &self,
        grid: SdfPreviewGrid,
        precision: SdfSamplingPrecision,
    ) -> Result<SdfGridSamplingReport, SdfGridSamplingError> {
        sample_expr_grid_preview(
            &self.expr,
            grid,
            precision,
            self.metric_status(),
            self.freshness(),
        )
    }

    /// Build a preview-only mesh report from a sampled exact grid.
    ///
    /// The report records Surface Nets style crossing diagnostics and may emit
    /// primitive-float proposal vertices/triangles. The output is still
    /// preview-only; topology consumers must replay exact predicates before
    /// accepting mesh structure.
    pub fn mesh_preview_from_grid(
        &self,
        grid: SdfPreviewGrid,
        precision: SdfSamplingPrecision,
    ) -> Result<SdfMeshPreviewReport, SdfGridSamplingError> {
        self.sample_grid_preview(grid, precision)
            .map(SdfMeshPreviewReport::surface_nets_diagnostic)
    }

    /// Build a dual-contouring proposal report from a regular exact grid.
    ///
    /// The report follows Ju, Losasso, Schaefer, and Warren's Dual Contouring
    /// Hermite/QEF shape, but keeps Yap's exact-computation boundary explicit:
    /// sampled scalar values are retained as adapter data, while endpoint
    /// signs, affine edge roots, and normals replay through the retained SDF
    /// where the current expression can certify them. The output is a mesh
    /// validation handoff candidate, not an accepted mesh.
    pub fn dual_contouring_report_from_grid(
        &self,
        grid: SdfPreviewGrid,
        precision: SdfSamplingPrecision,
    ) -> Result<SdfDualContouringReport, SdfGridSamplingError> {
        self.sample_grid_preview(grid, precision)
            .map(|samples| dual_contouring_report_from_exact_sdf(&self.expr, samples))
    }

    /// Build an approximated-gradient contouring proposal from a sampled grid.
    ///
    /// This route records finite-difference gradients, one-step projected
    /// surface candidates, primitive filters, and sampled face connectivity.
    /// The data is useful to downstream meshing algorithms, but it is not exact
    /// topology evidence. The report therefore keeps lossy-gradient and
    /// connectivity blockers visible, following Yap's EGC requirement that
    /// sampled views do not silently become accepted geometry.
    pub fn gradient_contouring_report_from_grid(
        &self,
        grid: SdfPreviewGrid,
        precision: SdfSamplingPrecision,
    ) -> Result<SdfGradientContourReport, SdfGridSamplingError> {
        self.sample_grid_preview(grid, precision)
            .map(gradient_contour_report_from_prepared_grid)
    }

    /// Export a preview-only GLSL scalar function.
    ///
    /// Constants are lowered through named lossy `Real` exports. The report
    /// records unsupported nodes, failed lowerings, and preview-only topology
    /// status instead of treating shader source as exact evidence.
    pub fn export_glsl_preview(
        &self,
        function_name: &str,
        precision: SdfSamplingPrecision,
    ) -> SdfShaderExportReport {
        export_expr_glsl_preview(
            &self.expr,
            function_name,
            precision,
            self.metric_status(),
            self.freshness(),
        )
    }

    /// Replay an external projection/intersection/fitting candidate through
    /// exact SDF classification.
    ///
    /// This method does not run an iterative solver. It is the acceptance
    /// boundary for candidates produced by `hypersolve` or another named
    /// proposal adapter.
    pub fn replay_projection_proposal(
        &self,
        proposal: SdfProjectionProposal,
    ) -> SdfProjectionReplayReport {
        let candidate_report = self.classify_point(&proposal.candidate);
        SdfProjectionReplayReport::from_candidate_report(
            proposal,
            candidate_report,
            self.metric_status(),
            self.freshness(),
        )
    }

    /// Package exact/certified cell classifications for a grid or voxel
    /// consumer.
    ///
    /// Unknown cells remain unknown in the returned report. This method does
    /// not consult preview samples and does not allocate `hypervoxel` storage;
    /// it is the continuous-field evidence envelope that a grid owner can
    /// consume or reject.
    pub fn classify_cells_for_handoff<'a, I>(&self, cells: I) -> SdfVoxelHandoffReport
    where
        I: IntoIterator<Item = (&'a Point3, &'a Point3)>,
    {
        SdfVoxelHandoffReport::from_cells(
            self.classify_cells(cells),
            self.metric_status(),
            self.freshness(),
        )
    }

    /// Classify an exact voxel-cell grid for a downstream `hypervoxel` owner.
    ///
    /// This method constructs exact cell AABBs from the requested frame and
    /// replays conservative SDF cell predicates for every cell. It does not
    /// allocate `hypervoxel` storage; it returns a report with frame readiness
    /// and occupancy labels that a grid owner can materialize or reject.
    pub fn classify_voxel_grid_for_handoff(
        &self,
        grid: SdfVoxelCellGrid,
    ) -> Result<SdfHypervoxelHandoffReport, SdfVoxelGridError> {
        grid.cell_count()?;
        grid.validate_positive_step()?;
        let classifications = voxel_cell_bounds(&grid)
            .iter()
            .map(|(min, max)| self.classify_cell(min, max))
            .collect();
        Ok(SdfHypervoxelHandoffReport::from_classifications(
            grid,
            classifications,
            self.metric_status(),
            self.freshness(),
        ))
    }

    /// Start a typed downstream handoff package from this prepared expression.
    ///
    /// The package carries retained continuous-field facts immediately and can
    /// be extended with optional adapter reports. Consumers must call
    /// `require_domain` for the domain they need; optional payload presence is
    /// never treated as topology evidence by itself.
    pub fn handoff_package(&self) -> SdfHandoffPackage {
        SdfHandoffPackage::new(self.facts.clone(), self.metric_status(), self.freshness())
    }

    fn cache_payoff(&self, query_count: usize) -> SdfCachePayoffReport {
        SdfCachePayoffReport::new(
            query_count,
            self.facts.node_count,
            self.facts.primitive_count,
            self.facts.transform_count,
        )
    }
}

fn classify_expr_point(expr: &SdfExpr, point: &Point3) -> PredicateOutcome<SdfPointLocation> {
    match expr {
        SdfExpr::Constant(_)
        | SdfExpr::Coordinate(_)
        | SdfExpr::Linear { .. }
        | SdfExpr::Add(_, _)
        | SdfExpr::Sub(_, _)
        | SdfExpr::Mul(_, _)
        | SdfExpr::Abs(_)
        | SdfExpr::Sqrt(_)
        | SdfExpr::Sin(_)
        | SdfExpr::Cos(_)
        | SdfExpr::Tan(_) => classify_scalar_expr_point(expr, point),
        SdfExpr::Primitive(primitive) => primitive.classify_point(point),
        SdfExpr::Union(left, right) => combine_point_union(
            classify_expr_point(left, point),
            classify_expr_point(right, point),
        ),
        SdfExpr::Intersection(left, right) => combine_point_intersection(
            classify_expr_point(left, point),
            classify_expr_point(right, point),
        ),
        SdfExpr::Complement(inner) => map_point_complement(classify_expr_point(inner, point)),
        SdfExpr::Offset { .. } => classify_scalar_expr_point(expr, point),
        SdfExpr::Transform { child, transform } => {
            classify_expr_point(child, &transform.inverse_point(point))
        }
    }
}

fn classify_expr_cell(
    expr: &SdfExpr,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfCellLocation> {
    match expr {
        SdfExpr::Constant(_)
        | SdfExpr::Coordinate(_)
        | SdfExpr::Linear { .. }
        | SdfExpr::Add(_, _)
        | SdfExpr::Sub(_, _)
        | SdfExpr::Mul(_, _)
        | SdfExpr::Abs(_)
        | SdfExpr::Sqrt(_)
        | SdfExpr::Sin(_)
        | SdfExpr::Cos(_)
        | SdfExpr::Tan(_) => classify_interval_cell(expr, min, max),
        SdfExpr::Primitive(primitive) => classify_primitive_cell(primitive, min, max),
        SdfExpr::Union(left, right) => combine_cell_union(
            classify_expr_cell(left, min, max),
            classify_expr_cell(right, min, max),
        ),
        SdfExpr::Intersection(left, right) => combine_cell_intersection(
            classify_expr_cell(left, min, max),
            classify_expr_cell(right, min, max),
        ),
        SdfExpr::Complement(inner) => map_cell_complement(classify_expr_cell(inner, min, max)),
        SdfExpr::Offset { .. } => classify_interval_cell(expr, min, max),
        SdfExpr::Transform { child, transform } => {
            let Some((child_min, child_max)) = transform.inverse_aabb(min, max) else {
                return crate::transform::unsupported_transform_outcome();
            };
            classify_expr_cell(child, &child_min, &child_max)
        }
    }
}

fn classify_scalar_expr_point(
    expr: &SdfExpr,
    point: &Point3,
) -> PredicateOutcome<SdfPointLocation> {
    let Some(value) = scalar_expr_point(expr, point) else {
        return PredicateOutcome::unknown(
            hyperlimit::RefinementNeed::Unsupported,
            Escalation::Undecided,
        );
    };
    match compare_reals(&value, &hyperreal::Real::zero()) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                Ordering::Less => SdfPointLocation::Inside,
                Ordering::Equal => SdfPointLocation::Boundary,
                Ordering::Greater => SdfPointLocation::Outside,
            };
            PredicateOutcome::decided(location, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn classify_interval_cell(
    expr: &SdfExpr,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfCellLocation> {
    match interval_expr_cell(expr, min, max) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => interval_to_cell(value.lower, value.upper, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn interval_to_cell(
    lower: hyperreal::Real,
    upper: hyperreal::Real,
    certainty: Certainty,
    stage: Escalation,
) -> PredicateOutcome<SdfCellLocation> {
    let zero = hyperreal::Real::zero();
    match compare_reals(&upper, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => {
            return PredicateOutcome::decided(
                SdfCellLocation::ConservativeInside,
                certainty,
                stage,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    match compare_reals(&lower, &zero) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => PredicateOutcome::decided(SdfCellLocation::ConservativeOutside, certainty, stage),
        PredicateOutcome::Decided { .. } => {
            PredicateOutcome::decided(SdfCellLocation::Boundary, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn classify_primitive_cell(
    primitive: &SdfPrimitive,
    min: &Point3,
    max: &Point3,
) -> PredicateOutcome<SdfCellLocation> {
    match primitive {
        SdfPrimitive::Plane { plane } => map_plane_aabb(classify_plane_aabb3(plane, min, max)),
        SdfPrimitive::Sphere {
            center,
            radius_squared,
        } => match radius_squared_domain(radius_squared) {
            PredicateOutcome::Decided { value: true, .. } => map_sphere_aabb(
                classify_aabb3_sphere_intersection(min, max, center, radius_squared),
                min,
                max,
                center,
                radius_squared,
            ),
            PredicateOutcome::Decided { value: false, .. } => PredicateOutcome::unknown(
                hyperlimit::RefinementNeed::Unsupported,
                Escalation::Undecided,
            ),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        SdfPrimitive::Aabb {
            min: shape_min,
            max: shape_max,
        } => map_aabb_aabb(
            classify_aabb3_intersection(shape_min, shape_max, min, max),
            shape_min,
            shape_max,
            min,
            max,
        ),
        SdfPrimitive::RoundedAabb { .. } => {
            classify_interval_cell(&SdfExpr::Primitive(primitive.clone()), min, max)
        }
        SdfPrimitive::Cylinder { .. } => {
            classify_interval_cell(&SdfExpr::Primitive(primitive.clone()), min, max)
        }
        SdfPrimitive::Capsule { .. } => {
            classify_interval_cell(&SdfExpr::Primitive(primitive.clone()), min, max)
        }
        SdfPrimitive::Torus { .. } => {
            classify_interval_cell(&SdfExpr::Primitive(primitive.clone()), min, max)
        }
        SdfPrimitive::Slab { .. } => {
            classify_interval_cell(&SdfExpr::Primitive(primitive.clone()), min, max)
        }
    }
}

fn map_plane_aabb(
    outcome: PredicateOutcome<PlaneAabbRelation>,
) -> PredicateOutcome<SdfCellLocation> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                PlaneAabbRelation::Below => SdfCellLocation::ConservativeInside,
                PlaneAabbRelation::Above => SdfCellLocation::ConservativeOutside,
                PlaneAabbRelation::Intersecting => SdfCellLocation::Boundary,
            };
            PredicateOutcome::decided(location, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_sphere_aabb(
    outcome: PredicateOutcome<AabbSphereIntersection>,
    min: &Point3,
    max: &Point3,
    center: &Point3,
    radius_squared: &hyperreal::Real,
) -> PredicateOutcome<SdfCellLocation> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => match value {
            AabbSphereIntersection::Disjoint => {
                PredicateOutcome::decided(SdfCellLocation::ConservativeOutside, certainty, stage)
            }
            AabbSphereIntersection::Touching => {
                PredicateOutcome::decided(SdfCellLocation::Boundary, certainty, stage)
            }
            AabbSphereIntersection::Overlapping => {
                let corners = corners(min, max);
                let mut boundary = false;
                for corner in &corners {
                    match classify_point_sphere3(center, radius_squared, corner) {
                        PredicateOutcome::Decided { value, .. } => match value {
                            hyperlimit::SpherePointLocation::Inside => {}
                            hyperlimit::SpherePointLocation::On => boundary = true,
                            hyperlimit::SpherePointLocation::Outside => {
                                return PredicateOutcome::decided(
                                    SdfCellLocation::Boundary,
                                    certainty,
                                    stage,
                                );
                            }
                        },
                        PredicateOutcome::Unknown { needed, stage } => {
                            return PredicateOutcome::unknown(needed, stage);
                        }
                    }
                }
                let location = if boundary {
                    SdfCellLocation::Boundary
                } else {
                    SdfCellLocation::ConservativeInside
                };
                PredicateOutcome::decided(location, certainty, stage)
            }
        },
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_aabb_aabb(
    outcome: PredicateOutcome<Aabb3Intersection>,
    shape_min: &Point3,
    shape_max: &Point3,
    cell_min: &Point3,
    cell_max: &Point3,
) -> PredicateOutcome<SdfCellLocation> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => match value {
            Aabb3Intersection::Disjoint => {
                PredicateOutcome::decided(SdfCellLocation::ConservativeOutside, certainty, stage)
            }
            Aabb3Intersection::Touching => {
                PredicateOutcome::decided(SdfCellLocation::Boundary, certainty, stage)
            }
            Aabb3Intersection::Overlapping => {
                let corners = corners(cell_min, cell_max);
                for corner in &corners {
                    match classify_point_aabb3(shape_min, shape_max, corner) {
                        PredicateOutcome::Decided { value, .. } => {
                            if value != hyperlimit::Aabb3PointLocation::Inside {
                                return PredicateOutcome::decided(
                                    SdfCellLocation::Boundary,
                                    certainty,
                                    stage,
                                );
                            }
                        }
                        PredicateOutcome::Unknown { needed, stage } => {
                            return PredicateOutcome::unknown(needed, stage);
                        }
                    }
                }
                PredicateOutcome::decided(SdfCellLocation::ConservativeInside, certainty, stage)
            }
        },
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn combine_point_union(
    left: PredicateOutcome<SdfPointLocation>,
    right: PredicateOutcome<SdfPointLocation>,
) -> PredicateOutcome<SdfPointLocation> {
    combine_point(left, right, |a, b| match (a, b) {
        (SdfPointLocation::Inside, _) | (_, SdfPointLocation::Inside) => SdfPointLocation::Inside,
        (SdfPointLocation::Boundary, _) | (_, SdfPointLocation::Boundary) => {
            SdfPointLocation::Boundary
        }
        (SdfPointLocation::Outside, SdfPointLocation::Outside) => SdfPointLocation::Outside,
        _ => SdfPointLocation::Unknown,
    })
}

fn combine_point_intersection(
    left: PredicateOutcome<SdfPointLocation>,
    right: PredicateOutcome<SdfPointLocation>,
) -> PredicateOutcome<SdfPointLocation> {
    combine_point(left, right, |a, b| match (a, b) {
        (SdfPointLocation::Outside, _) | (_, SdfPointLocation::Outside) => {
            SdfPointLocation::Outside
        }
        (SdfPointLocation::Boundary, _) | (_, SdfPointLocation::Boundary) => {
            SdfPointLocation::Boundary
        }
        (SdfPointLocation::Inside, SdfPointLocation::Inside) => SdfPointLocation::Inside,
        _ => SdfPointLocation::Unknown,
    })
}

fn combine_cell_union(
    left: PredicateOutcome<SdfCellLocation>,
    right: PredicateOutcome<SdfCellLocation>,
) -> PredicateOutcome<SdfCellLocation> {
    combine_cell(left, right, |a, b| match (a, b) {
        (SdfCellLocation::ConservativeInside, _) | (_, SdfCellLocation::ConservativeInside) => {
            SdfCellLocation::ConservativeInside
        }
        (SdfCellLocation::ConservativeOutside, SdfCellLocation::ConservativeOutside) => {
            SdfCellLocation::ConservativeOutside
        }
        (SdfCellLocation::Unknown, _) | (_, SdfCellLocation::Unknown) => SdfCellLocation::Unknown,
        _ => SdfCellLocation::Boundary,
    })
}

fn combine_cell_intersection(
    left: PredicateOutcome<SdfCellLocation>,
    right: PredicateOutcome<SdfCellLocation>,
) -> PredicateOutcome<SdfCellLocation> {
    combine_cell(left, right, |a, b| match (a, b) {
        (SdfCellLocation::ConservativeOutside, _) | (_, SdfCellLocation::ConservativeOutside) => {
            SdfCellLocation::ConservativeOutside
        }
        (SdfCellLocation::ConservativeInside, SdfCellLocation::ConservativeInside) => {
            SdfCellLocation::ConservativeInside
        }
        (SdfCellLocation::Unknown, _) | (_, SdfCellLocation::Unknown) => SdfCellLocation::Unknown,
        _ => SdfCellLocation::Boundary,
    })
}

fn combine_point<F>(
    left: PredicateOutcome<SdfPointLocation>,
    right: PredicateOutcome<SdfPointLocation>,
    combine: F,
) -> PredicateOutcome<SdfPointLocation>
where
    F: FnOnce(SdfPointLocation, SdfPointLocation) -> SdfPointLocation,
{
    match (left, right) {
        (
            PredicateOutcome::Decided {
                value: a,
                certainty,
                stage,
            },
            PredicateOutcome::Decided {
                value: b,
                certainty: right_certainty,
                stage: right_stage,
            },
        ) => PredicateOutcome::decided(
            combine(a, b),
            merge_certainty(certainty, right_certainty),
            merge_stage(stage, right_stage),
        ),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn combine_cell<F>(
    left: PredicateOutcome<SdfCellLocation>,
    right: PredicateOutcome<SdfCellLocation>,
    combine: F,
) -> PredicateOutcome<SdfCellLocation>
where
    F: FnOnce(SdfCellLocation, SdfCellLocation) -> SdfCellLocation,
{
    match (left, right) {
        (
            PredicateOutcome::Decided {
                value: a,
                certainty,
                stage,
            },
            PredicateOutcome::Decided {
                value: b,
                certainty: right_certainty,
                stage: right_stage,
            },
        ) => PredicateOutcome::decided(
            combine(a, b),
            merge_certainty(certainty, right_certainty),
            merge_stage(stage, right_stage),
        ),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

fn map_point_complement(
    outcome: PredicateOutcome<SdfPointLocation>,
) -> PredicateOutcome<SdfPointLocation> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.complemented(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn map_cell_complement(
    outcome: PredicateOutcome<SdfCellLocation>,
) -> PredicateOutcome<SdfCellLocation> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.complemented(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn corners(min: &Point3, max: &Point3) -> [Point3; 8] {
    [
        Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
        Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
        Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
        Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
    ]
}

fn merge_certainty(left: Certainty, right: Certainty) -> Certainty {
    if left == Certainty::Filtered || right == Certainty::Filtered {
        Certainty::Filtered
    } else {
        Certainty::Exact
    }
}

fn merge_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(right) > stage_rank(left) {
        right
    } else {
        left
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
