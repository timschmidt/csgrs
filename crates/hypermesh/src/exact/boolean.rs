//! Exact boolean operation entry points.
//!
//! The legacy boolmesh-derived public API mutates triangle topology through
//! primitive-float kernels. This module is the exact-stack replacement
//! boundary for the subset that is currently implemented: build certified
//! intersection events, form exact split-region loops, classify those regions,
//! triangulate them through feature-gated exact `hypertri`, assemble exact 3D
//! output triangles, and validate the resulting [`ExactMesh`].
//!
//! The operation policy is deliberately explicit. No-intersection named
//! booleans are handled by certified empty/disjoint/identity, convex,
//! coplanar, or exact ray-parity winding shortcuts; remaining split-region
//! cases require a selected-region policy or an explicit unsupported report
//! instead of a silently approximate union/intersection/difference decision.
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): topology decisions must be certified or represented
//! as policy choices/unknowns.

#[cfg(feature = "exact-triangulation")]
use super::adjacent::{has_full_face_adjacent_union, materialize_full_face_adjacent_union};
#[cfg(feature = "exact-triangulation")]
use super::affine_box::{
    AffineBoxOperation, has_affine_box_difference, has_affine_box_intersection,
    has_affine_box_union, materialize_affine_box_difference, materialize_affine_box_intersection,
    materialize_affine_box_union,
};
#[cfg(feature = "exact-triangulation")]
use super::affine_solid::{
    AffineOrthogonalSolidOperation, has_affine_orthogonal_solid_cells,
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_affine_orthogonal_solid_union,
};
#[cfg(feature = "exact-triangulation")]
use super::affine_surface::{
    arrange_coplanar_affine_surface_difference, arrange_coplanar_affine_surface_intersection,
    arrange_coplanar_affine_surface_union,
};
#[cfg(feature = "exact-triangulation")]
use super::bounds::AabbIntersectionKind;
#[cfg(feature = "exact-triangulation")]
use super::box_solid::{
    cell_difference_axis_aligned_boxes, cell_union_axis_aligned_boxes,
    difference_axis_aligned_boxes, empty_difference_axis_aligned_boxes,
    has_axis_aligned_box_cell_difference, has_axis_aligned_box_cell_union,
    has_axis_aligned_box_difference, has_axis_aligned_box_empty_difference,
    has_axis_aligned_box_intersection, has_axis_aligned_box_multi_difference,
    has_axis_aligned_box_nested_difference, has_axis_aligned_box_union,
    intersection_axis_aligned_boxes, multi_difference_axis_aligned_boxes,
    nested_difference_axis_aligned_boxes, union_axis_aligned_boxes,
};
#[cfg(feature = "exact-triangulation")]
use super::cells::triangulate_all_face_cells_with_cdt;
#[cfg(feature = "exact-triangulation")]
use super::construction::SegmentPlaneRelation;
#[cfg(feature = "exact-triangulation")]
use super::convex::{intersect_closed_convex_solids, subtract_closed_convex_solids_single_cap};
#[cfg(feature = "exact-triangulation")]
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
#[cfg(feature = "exact-triangulation")]
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
#[cfg(feature = "exact-triangulation")]
use super::intersection::MeshFacePairRelation;
#[cfg(feature = "exact-triangulation")]
use super::mesh::{ExactMesh, Triangle};
#[cfg(feature = "exact-triangulation")]
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, has_axis_aligned_orthogonal_solid_cells,
    materialize_axis_aligned_orthogonal_solid_cells,
};
#[cfg(feature = "exact-triangulation")]
use super::orthogonal_surface::{
    CoplanarOrthogonalSurfaceOperation, arrange_coplanar_orthogonal_surface_difference,
    arrange_coplanar_orthogonal_surface_intersection, arrange_coplanar_orthogonal_surface_union,
};
#[cfg(feature = "exact-triangulation")]
use super::provenance::PredicateUse;
#[cfg(feature = "exact-triangulation")]
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionRetention, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionTriangulation,
    checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut,
};
#[cfg(feature = "exact-triangulation")]
use super::reports::{
    ExactBooleanBlocker, ExactBooleanBlockerKind, ExactBooleanPreflight, ExactBooleanResult,
    ExactBooleanResultKind, ExactBooleanShortcutKind, ExactBooleanSupport,
    ExactBoundaryTouchingReport, ExactBoundaryTouchingStatus, ExactOpenSurfaceDisjointReport,
    ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactRefinementReport, ExactRefinementStatus, ExactSameSurfaceReport, ExactSameSurfaceStatus,
    ExactWindingReadinessReport, ExactWindingReadinessStatus,
};
#[cfg(feature = "exact-triangulation")]
use super::solid::{ConvexSolidMeshRelation, classify_mesh_vertices_against_convex_solid};
#[cfg(feature = "exact-triangulation")]
use super::surface::{
    CoplanarConvexSurfaceContainment, CoplanarSurfaceContainment,
    arrange_coplanar_convex_surface_component_holed_difference,
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference, arrange_coplanar_convex_surface_intersection,
    arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_convex_surface_multi_intersection,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_coplanar_surface_cutter_hole_contact_difference,
    arrange_coplanar_surface_multi_difference, arrange_single_triangle_coplanar_difference,
    arrange_single_triangle_coplanar_holed_difference, arrange_single_triangle_coplanar_union,
    certify_coplanar_convex_surface_containment, certify_coplanar_convex_surface_equivalence,
    certify_single_triangle_coplanar_containment, difference_single_triangle_coplanar_surfaces,
    intersect_single_triangle_coplanar_surfaces, union_single_triangle_coplanar_surfaces,
};
#[cfg(feature = "exact-triangulation")]
use super::validation::ValidationPolicy;
#[cfg(feature = "exact-triangulation")]
use super::volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
    classify_triangulated_regions_against_opposite_meshes,
};
#[cfg(feature = "exact-triangulation")]
use super::winding::{
    ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshReport, ClosedMeshWindingRelation,
    WindingReportError, classify_mesh_vertices_against_closed_mesh_winding_report,
};
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{SegmentIntersection, TriangleLocation, compare_reals, compare_reals_report};
#[cfg(feature = "exact-triangulation")]
use std::cmp::Ordering;

/// Exact selected-region boolean policy.
///
/// This policy is intentionally narrower than a named boolean operation. It
/// records the currently certified operation semantics: retain selected split
/// regions, optionally reject unresolved graph events, then validate the
/// materialized exact output mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBooleanPolicy {
    /// Which source-side regions should be retained in the output assembly.
    pub selection: ExactRegionSelection,
    /// Validation policy for the materialized output mesh.
    pub validation: ValidationPolicy,
    /// Reject the operation if graph extraction retained unknown events.
    pub reject_unknowns: bool,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanPolicy {
    /// Keep all selected-region output and allow boundary meshes.
    pub const KEEP_ALL_BOUNDARY: Self = Self {
        selection: ExactRegionSelection::KeepAll,
        validation: ValidationPolicy::ALLOW_BOUNDARY,
        reject_unknowns: true,
    };
}

/// Exact boolean operation request.
///
/// Named booleans are represented now, but they intentionally do not fall back
/// to legacy float winding. Certified shortcut cases execute directly, while
/// remaining named overlaps return [`DiagnosticKind::UnsupportedExactOperation`]
/// until split-region inside/outside classification is complete.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanOperation {
    /// Assemble explicitly selected source-side split regions.
    SelectedRegions(ExactRegionSelection),
    /// Exact union through certified shortcuts or future split-region winding.
    Union,
    /// Exact intersection through certified shortcuts or future split-region
    /// winding.
    Intersection,
    /// Exact difference through certified shortcuts or future split-region
    /// winding.
    Difference,
}

/// Boundary-only policy for named exact boolean operations.
///
/// Triangle meshes cannot represent lower-dimensional set intersections
/// without adding a separate curve/point output channel. Following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), this policy makes that application-layer projection explicit:
/// certified coplanar-touching graphs are either rejected, or projected into a
/// triangle-mesh-only result that preserves separate shells and discards
/// lower-dimensional intersection geometry.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoundaryBooleanPolicy {
    /// Reject boundary-only named booleans until a caller chooses a projection
    /// policy.
    Reject,
    /// Preserve separate shells for union, keep the left shell for difference,
    /// and return an empty triangle mesh for lower-dimensional intersections.
    PreserveSeparateShells,
}

/// Run the exact selected-region boolean pipeline.
///
/// The returned report keeps the audit artifacts needed to inspect why an
/// output mesh was produced. It does not use primitive-float representatives
/// for topology, and it does not hide unresolved exact predicates unless the
/// caller explicitly disables [`ExactBooleanPolicy::reject_unknowns`].
#[cfg(feature = "exact-triangulation")]
pub fn boolean_selected_regions(
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    if policy.reject_unknowns && graph_had_unknowns {
        return Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            "exact boolean graph contains unresolved predicate events",
        )));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        policy.selection,
        left,
        right,
    )
    .map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::IndexOutOfBounds,
            format!("exact boolean assembly failed: {error}"),
        ))
    })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, policy.validation)?;

    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::SelectedRegions {
            selection: policy.selection,
        },
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications: Vec::new(),
        mesh,
    };
    result
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact selected-region result/source replay failed: {error:?}"),
            ))
        })?;
    Ok(result)
}

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable selected-region pipeline. For
/// named booleans that are not covered by a certified shortcut, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] once all available
/// classifications are proof-producing. This keeps the missing operation
/// semantics visible at the API boundary instead of approximating them.
#[cfg(feature = "exact-triangulation")]
pub fn preflight_boolean_exact(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactBooleanPreflight, MeshError> {
    let support = match operation {
        ExactBooleanOperation::SelectedRegions(_) => ExactBooleanSupport::SelectedRegionPolicy,
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if left.triangles().is_empty() || right.triangles().is_empty() =>
        {
            ExactBooleanSupport::CertifiedEmptyOperand
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_bounds_disjoint(left, right) =>
        {
            ExactBooleanSupport::CertifiedBoundsDisjoint
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_identical(left, right) =>
        {
            ExactBooleanSupport::CertifiedIdentical
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_same_surface(left, right) =>
        {
            ExactBooleanSupport::CertifiedSameSurface
        }
        ExactBooleanOperation::Union if has_full_face_adjacent_union(left, right) => {
            ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if certify_coplanar_convex_surface_containment(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_convex_surface_intersection(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_convex_surface_multi_intersection(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_orthogonal_surface_intersection(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceIntersection
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_affine_surface_intersection(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarAffineSurfaceIntersection
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_component_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_multi_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_orthogonal_surface_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_affine_surface_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_multi_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_surface_multi_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_holed_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiHoledDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_component_holed_difference(left, right)
                .is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_orthogonal_surface_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_affine_surface_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
        }
        ExactBooleanOperation::Union if has_axis_aligned_box_union(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
        }
        ExactBooleanOperation::Intersection if has_axis_aligned_box_intersection(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_difference(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_multi_difference(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference
        }
        ExactBooleanOperation::Difference
            if has_axis_aligned_box_nested_difference(left, right) =>
        {
            ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_empty_difference(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
        }
        ExactBooleanOperation::Union if has_affine_box_union(left, right) => {
            ExactBooleanSupport::CertifiedAffineBoxUnion
        }
        ExactBooleanOperation::Intersection if has_affine_box_intersection(left, right) => {
            ExactBooleanSupport::CertifiedAffineBoxIntersection
        }
        ExactBooleanOperation::Difference if has_affine_box_difference(left, right) => {
            ExactBooleanSupport::CertifiedAffineBoxDifference
        }
        ExactBooleanOperation::Union
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ) =>
        {
            ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion
        }
        ExactBooleanOperation::Intersection
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ) =>
        {
            ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
        }
        ExactBooleanOperation::Difference
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
            ) =>
        {
            ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_coplanar_convex_surface_containment(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certified_coplanar_surface_boolean_support(left, right, operation).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
        }
        ExactBooleanOperation::Intersection
            if intersect_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
        }
        ExactBooleanOperation::Union
            if union_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
        }
        ExactBooleanOperation::Union
            if arrange_single_triangle_coplanar_union(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
        }
        ExactBooleanOperation::Difference
            if difference_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
        }
        ExactBooleanOperation::Difference
            if arrange_single_triangle_coplanar_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
        }
        ExactBooleanOperation::Difference
            if arrange_single_triangle_coplanar_holed_difference(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_open_surface_disjoint_report(left, right)?.is_certified() =>
        {
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => certified_convex_boolean_support(left, right)?
            .or_else(|| certified_convex_intersection_support(left, right, operation))
            .or_else(|| certified_convex_single_cap_difference_support(left, right, operation))
            .or(certified_winding_boolean_support(left, right)?)
            .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding),
    };

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiHoledDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
            | ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
            | ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
            | ExactBooleanSupport::CertifiedAffineBoxUnion
            | ExactBooleanSupport::CertifiedAffineBoxIntersection
            | ExactBooleanSupport::CertifiedAffineBoxDifference
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference
            | ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexSingleCapDifference
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated
            | ExactBooleanSupport::CertifiedWindingContainment
            | ExactBooleanSupport::CertifiedWindingSeparated
    ) {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns: false,
            retained_face_pairs: 0,
            retained_events: 0,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
        });
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let relation_counts = graph_relation_counts(&graph);
    if graph_had_unknowns {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::UnresolvedGraph,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
            arrangement_readiness: None,
        });
    }
    if relation_counts.construction_failed_events > 0 {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::UnresolvedGraph,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
            arrangement_readiness: None,
        });
    }
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let geometry = graph.face_split_geometry_plan(left, right)?;
        let region_plan = geometry.region_plan(left, right);
        let region_classifications =
            checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::SelectedRegionPolicy,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: region_plan.regions.len(),
            region_classifications,
            blocker: None,
            arrangement_readiness: None,
        });
    }
    let boundary_report = boundary_touching_report_from_graph(&graph, left, right)?;
    if boundary_report.is_certified() {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresBoundaryPolicy,
            graph_had_unknowns: boundary_report.graph_had_unknowns,
            retained_face_pairs: boundary_report.retained_face_pairs,
            retained_events: boundary_report.retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(boundary_report.blocker),
            arrangement_readiness: None,
        });
    }
    let planar_report = planar_arrangement_report_from_graph(&graph, left, right, operation)?;
    if planar_report.is_required() {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresPlanarArrangement,
            graph_had_unknowns: planar_report.graph_had_unknowns,
            retained_face_pairs: planar_report.retained_face_pairs,
            retained_events: planar_report.retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(planar_report.blocker),
            arrangement_readiness: planar_report.arrangement_readiness,
        });
    }
    let eager_axis_aligned_cell_support = match operation {
        ExactBooleanOperation::Union if has_axis_aligned_box_cell_union(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxCellUnion)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_cell_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxCellDifference)
        }
        _ => None,
    };
    if let Some(support) = eager_axis_aligned_cell_support {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
        });
    }
    if let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: axis_aligned_orthogonal_solid_support(solid_operation),
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
        });
    }
    if let Some((region_classifications, triangulations, _volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(&graph, left, right)?.filter(
            |(_, triangulations, volumetric_classifications)| {
                volumetric_classifications
                    .iter()
                    .all(|classification| classification.relation.is_materialization_decided())
                    && operation_retains_any_volumetric_region(
                        operation,
                        triangulations,
                        volumetric_classifications,
                    )
                    && volumetric_plan_materializes_operation(
                        operation,
                        triangulations,
                        volumetric_classifications,
                        left,
                        right,
                        ValidationPolicy::CLOSED,
                    )
            },
        )
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::CertifiedWindingMaterialized,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: triangulations.len(),
            region_classifications,
            blocker: None,
            arrangement_readiness: None,
        });
    }
    if graph_requires_coplanar_volumetric_cells(&relation_counts) {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresCoplanarVolumetricCells,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(
                relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells),
            ),
            arrangement_readiness: None,
        });
    }

    let winding_report = winding_readiness_report_from_graph(&graph, left, right, operation)?;

    Ok(ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns: winding_report.graph_had_unknowns,
        retained_face_pairs: winding_report.retained_face_pairs,
        retained_events: winding_report.retained_events,
        region_count: winding_report.region_count,
        region_classifications: winding_report.region_classifications,
        blocker: Some(winding_report.blocker),
        arrangement_readiness: None,
    })
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct GraphRelationCounts {
    candidate_pairs: usize,
    coplanar_overlapping_pairs: usize,
    coplanar_touching_pairs: usize,
    unknown_pairs: usize,
    construction_failed_events: usize,
}

#[cfg(feature = "exact-triangulation")]
impl GraphRelationCounts {
    const fn into_blocker(self, kind: ExactBooleanBlockerKind) -> ExactBooleanBlocker {
        ExactBooleanBlocker {
            kind,
            candidate_pairs: self.candidate_pairs,
            coplanar_overlapping_pairs: self.coplanar_overlapping_pairs,
            coplanar_touching_pairs: self.coplanar_touching_pairs,
            unknown_pairs: self.unknown_pairs,
            construction_failed_events: self.construction_failed_events,
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn graph_relation_counts(graph: &super::graph::ExactIntersectionGraph) -> GraphRelationCounts {
    let mut counts = GraphRelationCounts::default();
    for pair in &graph.face_pairs {
        let pair_has_unknown_event = pair
            .events
            .iter()
            .any(|event| matches!(event, super::graph::IntersectionEvent::Unknown));
        match pair.relation {
            MeshFacePairRelation::Candidate => counts.candidate_pairs += 1,
            MeshFacePairRelation::CoplanarOverlapping => counts.coplanar_overlapping_pairs += 1,
            MeshFacePairRelation::CoplanarTouching => counts.coplanar_touching_pairs += 1,
            MeshFacePairRelation::Unknown => counts.unknown_pairs += 1,
            MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
        }
        if pair.relation != MeshFacePairRelation::Unknown && pair_has_unknown_event {
            counts.unknown_pairs += 1;
        }
        counts.construction_failed_events += pair
            .events
            .iter()
            .filter(|event| {
                matches!(
                    event,
                    super::graph::IntersectionEvent::SegmentPlane {
                        relation: super::construction::SegmentPlaneRelation::ConstructionFailed,
                        ..
                    }
                )
            })
            .count();
    }
    counts
}

#[cfg(feature = "exact-triangulation")]
fn graph_requires_boundary_policy(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if graph_has_only_coplanar_touching_pairs(graph) {
        return Ok(true);
    }
    if !graph_has_only_boundary_contact_pairs(graph) {
        return Ok(false);
    }
    certified_closed_boundary_contact(left, right)
}

#[cfg(feature = "exact-triangulation")]
fn graph_has_only_coplanar_touching_pairs(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| pair.relation == MeshFacePairRelation::CoplanarTouching)
}

#[cfg(feature = "exact-triangulation")]
fn graph_has_only_coplanar_contact_pairs(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph.face_pairs.iter().all(|pair| {
            matches!(
                pair.relation,
                MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
            )
        })
        && graph
            .face_pairs
            .iter()
            .any(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping)
}

#[cfg(feature = "exact-triangulation")]
fn graph_requires_planar_arrangement(graph: &super::graph::ExactIntersectionGraph) -> bool {
    graph_has_only_coplanar_contact_pairs(graph)
}

#[cfg(feature = "exact-triangulation")]
fn graph_requires_coplanar_volumetric_cells(counts: &GraphRelationCounts) -> bool {
    // Coplanar source-face cells inside a closed volumetric overlap are not a
    // planar-surface output problem and not ordinary non-coplanar winding
    // cells. Following Yap, "Towards Exact Geometric Computation," Comput.
    // Geom. 7.1-2 (1997), keep that missing topology stage as a named exact
    // state instead of approximating the cells or relabeling them as generic
    // winding readiness.
    counts.coplanar_overlapping_pairs + counts.coplanar_touching_pairs > 0
}

#[cfg(feature = "exact-triangulation")]
fn graph_has_only_boundary_contact_pairs(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty() && graph.face_pairs.iter().all(boundary_contact_pair_shape)
}

#[cfg(feature = "exact-triangulation")]
fn boundary_contact_pair_shape(pair: &FacePairEvents) -> bool {
    match pair.relation {
        MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping => true,
        MeshFacePairRelation::Candidate => pair.events.iter().all(boundary_contact_candidate_event),
        MeshFacePairRelation::BoundsDisjoint
        | MeshFacePairRelation::PlaneSeparated
        | MeshFacePairRelation::Unknown => false,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boundary_contact_candidate_event(event: &IntersectionEvent) -> bool {
    // Positive-area coplanar contact between closed solids also retains
    // adjacent non-coplanar face pairs where an endpoint or coplanar source
    // edge lies on the opposite plane. Those are still boundary facts, not
    // volumetric crossings. Yap, "Towards Exact Geometric Computation,"
    // Comput. Geom. 7.1-2 (1997), requires us to preserve that event
    // distinction instead of collapsing every retained candidate into the
    // same unsupported topology bucket.
    match event {
        IntersectionEvent::SegmentPlane { relation, .. } => matches!(
            relation,
            SegmentPlaneRelation::Disjoint
                | SegmentPlaneRelation::Coplanar
                | SegmentPlaneRelation::EndpointOnPlane
        ),
        IntersectionEvent::CoplanarEdge { relation, .. } => {
            *relation != SegmentIntersection::Disjoint
        }
        IntersectionEvent::CoplanarVertex { location, .. } => matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ),
        IntersectionEvent::Unknown => false,
    }
}

#[cfg(feature = "exact-triangulation")]
fn certified_closed_boundary_contact(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(winding_error)?;

    Ok(mesh_vertices_are_boundary_or_outside(&left_in_right)
        && mesh_vertices_are_boundary_or_outside(&right_in_left)
        && (mesh_vertices_touch_boundary(&left_in_right)
            || mesh_vertices_touch_boundary(&right_in_left)))
}

#[cfg(feature = "exact-triangulation")]
fn mesh_vertices_are_boundary_or_outside(report: &ClosedMeshWindingMeshReport) -> bool {
    report.target_closed
        && report.vertices.iter().all(|vertex| {
            matches!(
                vertex.relation,
                ClosedMeshWindingRelation::Outside | ClosedMeshWindingRelation::Boundary
            )
        })
}

#[cfg(feature = "exact-triangulation")]
fn mesh_vertices_touch_boundary(report: &ClosedMeshWindingMeshReport) -> bool {
    report
        .vertices
        .iter()
        .any(|vertex| vertex.relation == ClosedMeshWindingRelation::Boundary)
}

/// Run an exact boolean operation request.
///
/// This entry point makes unsupported named booleans explicit rather than
/// silently dispatching to legacy tolerance code. That is a deliberate
/// exact-computation boundary: unsupported topology semantics are diagnostics,
/// not approximate decisions.
#[cfg(feature = "exact-triangulation")]
pub fn boolean_exact(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_exact_with_boundary_policy(
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    )
}

/// Run an exact boolean operation request with an explicit boundary policy.
///
/// This entry point is still strict about general winding. The additional
/// policy only applies when the exact event graph contains certified
/// boundary-only contact: coplanar touching, closed-solid coplanar boundary
/// overlap, or closed-solid edge/vertex contact whose retained candidate
/// events have no proper crossings, construction failures, or unknowns. In
/// that narrow case, [`ExactBoundaryBooleanPolicy::PreserveSeparateShells`]
/// projects lower-dimensional contact into triangle-mesh output instead of
/// silently invoking the legacy tolerance path.
#[cfg(feature = "exact-triangulation")]
pub fn boolean_exact_with_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    match operation {
        ExactBooleanOperation::SelectedRegions(selection) => boolean_selected_regions(
            left,
            right,
            ExactBooleanPolicy {
                selection,
                validation,
                reject_unknowns: true,
            },
        ),
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if left.triangles().is_empty() || right.triangles().is_empty() =>
        {
            boolean_empty_operand(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_bounds_disjoint(left, right) =>
        {
            boolean_disjoint_meshes(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_identical(left, right) =>
        {
            boolean_identical_meshes(left, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_same_surface(left, right) =>
        {
            boolean_same_surface_meshes(left, operation, validation)
        }
        ExactBooleanOperation::Union if has_full_face_adjacent_union(left, right) => {
            boolean_full_face_adjacent_union(left, right, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() =>
        {
            boolean_coplanar_convex_equivalent_surfaces(left, operation, validation)
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if certify_coplanar_convex_surface_containment(left, right).is_some() =>
        {
            boolean_coplanar_convex_containment_surfaces(left, right, operation, validation)
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_convex_surface_intersection(left, right).is_some() =>
        {
            boolean_coplanar_convex_arrangement_intersection(left, right, validation)
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_convex_surface_multi_intersection(left, right).is_some() =>
        {
            boolean_coplanar_convex_multi_intersection(left, right, validation)
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_orthogonal_surface_intersection(left, right).is_some() =>
        {
            boolean_coplanar_orthogonal_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Intersection
            if arrange_coplanar_affine_surface_intersection(left, right).is_some() =>
        {
            boolean_coplanar_affine_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_union(left, right).is_some() =>
        {
            boolean_coplanar_convex_arrangement_union(left, right, validation)
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_component_union(left, right).is_some() =>
        {
            boolean_coplanar_convex_arrangement_union(left, right, validation)
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_convex_surface_multi_union(left, right).is_some() =>
        {
            boolean_coplanar_convex_multi_union(left, right, validation)
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_orthogonal_surface_union(left, right).is_some() =>
        {
            boolean_coplanar_orthogonal_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
            if arrange_coplanar_affine_surface_union(left, right).is_some() =>
        {
            boolean_coplanar_affine_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_difference(left, right).is_some() =>
        {
            boolean_coplanar_convex_arrangement_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_multi_difference(left, right).is_some() =>
        {
            boolean_coplanar_convex_multi_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_surface_multi_difference(left, right).is_some() =>
        {
            boolean_coplanar_multi_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some() =>
        {
            boolean_coplanar_cutter_hole_contact_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_holed_difference(left, right).is_some() =>
        {
            boolean_coplanar_convex_containment_surfaces(left, right, operation, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some() =>
        {
            boolean_coplanar_convex_multi_holed_difference(left, right, operation, validation).map(
                |result| result.expect("caller checked convex coplanar multi-holed difference"),
            )
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_convex_surface_component_holed_difference(left, right)
                .is_some() =>
        {
            boolean_coplanar_convex_component_holed_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_orthogonal_surface_difference(left, right).is_some() =>
        {
            boolean_coplanar_orthogonal_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Difference
            if arrange_coplanar_affine_surface_difference(left, right).is_some() =>
        {
            boolean_coplanar_affine_surface(left, right, operation, validation)
        }
        ExactBooleanOperation::Union if has_axis_aligned_box_union(left, right) => {
            boolean_axis_aligned_box_union(left, right, validation)
        }
        ExactBooleanOperation::Intersection if has_axis_aligned_box_intersection(left, right) => {
            boolean_axis_aligned_box_intersection(left, right, validation)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_difference(left, right) => {
            boolean_axis_aligned_box_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_multi_difference(left, right) => {
            boolean_axis_aligned_box_multi_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference
            if has_axis_aligned_box_nested_difference(left, right) =>
        {
            boolean_axis_aligned_box_nested_difference(left, right, validation)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_empty_difference(left, right) => {
            boolean_axis_aligned_box_empty_difference(left, right, validation)
        }
        ExactBooleanOperation::Union if has_affine_box_union(left, right) => {
            boolean_affine_box(left, right, AffineBoxOperation::Union, validation)
        }
        ExactBooleanOperation::Intersection if has_affine_box_intersection(left, right) => {
            boolean_affine_box(left, right, AffineBoxOperation::Intersection, validation)
        }
        ExactBooleanOperation::Difference if has_affine_box_difference(left, right) => {
            boolean_affine_box(left, right, AffineBoxOperation::Difference, validation)
        }
        ExactBooleanOperation::Union
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ) =>
        {
            boolean_affine_orthogonal_solid(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
                validation,
            )
        }
        ExactBooleanOperation::Intersection
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ) =>
        {
            boolean_affine_orthogonal_solid(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
                validation,
            )
        }
        ExactBooleanOperation::Difference
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
            ) =>
        {
            boolean_affine_orthogonal_solid(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
                validation,
            )
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_coplanar_convex_surface_containment(left, right).is_some() =>
        {
            boolean_coplanar_convex_containment_surfaces(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            if let Some(result) =
                boolean_coplanar_surface_containment(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_intersection(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_convex_union(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_arrangement_union(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_corner_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_arrangement_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_holed_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_coplanar_cutter_hole_contact_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_convex_multi_holed_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_coplanar_convex_component_holed_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_affine_surface_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_open_surface_disjoint_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_containment_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_intersection_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_single_cap_difference_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if matches!(
                operation,
                ExactBooleanOperation::Union | ExactBooleanOperation::Difference
            ) && let Some(result) =
                boolean_axis_aligned_box_cell_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_axis_aligned_orthogonal_solid_cell_meshes(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_volumetric_winding_regions(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_winding_containment_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_boundary_touching_meshes(
                left,
                right,
                operation,
                validation,
                boundary_policy,
            )? {
                return Ok(result);
            }
            Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "named exact booleans require certified winding/inside-outside classification",
            )))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_arrangement_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let union = arrange_coplanar_convex_surface_union(left, right)
        .or_else(|| arrange_coplanar_convex_surface_component_union(left, right))
        .expect("caller checked convex coplanar arrangement union");
    let mesh = copy_mesh(
        &union.mesh,
        "exact coplanar convex arrangement union",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_multi_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let union = arrange_coplanar_convex_surface_multi_union(left, right)
        .expect("caller checked convex coplanar multi-component union");
    let mesh = copy_mesh(
        &union.mesh,
        "exact coplanar convex multi-component union",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiUnion,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_convex_intersection_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Intersection {
        return Ok(None);
    }
    let Some(intersection) = intersect_closed_convex_solids(left, right) else {
        return Ok(None);
    };
    intersection.validate_against_sources(left, right)?;
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact closed-convex solid intersection",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ConvexIntersection,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_arrangement_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let intersection = arrange_coplanar_convex_surface_intersection(left, right)
        .expect("caller checked convex coplanar arrangement intersection");
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact coplanar convex arrangement intersection",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_multi_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let intersection = arrange_coplanar_convex_surface_multi_intersection(left, right)
        .expect("caller checked convex coplanar multi-component intersection");
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact coplanar convex multi-component intersection",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_arrangement_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let difference = arrange_coplanar_convex_surface_difference(left, right)
        .expect("caller checked convex coplanar arrangement difference");
    let mesh = copy_mesh(
        &difference.mesh,
        "exact coplanar convex arrangement difference",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let difference = arrange_coplanar_convex_surface_multi_difference(left, right)
        .expect("caller checked convex coplanar multi-component difference");
    let mesh = copy_mesh(
        &difference.mesh,
        "exact coplanar convex multi-component difference",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let difference = arrange_coplanar_surface_multi_difference(left, right)
        .expect("caller checked coplanar nonconvex multi-component difference");
    difference.validate_difference_against_sources(left, right)?;
    let mesh = copy_mesh(
        &difference.mesh,
        "exact coplanar nonconvex multi-component difference",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceMultiDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_cutter_hole_contact_difference_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    arrange_coplanar_surface_cutter_hole_contact_difference(left, right)
        .map(|difference| {
            difference.validate_cutter_hole_contact_difference_against_sources(left, right)?;
            let mesh = copy_mesh(
                &difference.mesh,
                "exact coplanar cutter-hole contact difference",
                validation,
            )?;
            Ok(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceCutterHoleContactDifference,
            ))
        })
        .transpose()
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_cutter_hole_contact_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_coplanar_cutter_hole_contact_difference_optional(
        left,
        right,
        ExactBooleanOperation::Difference,
        validation,
    )
    .map(|result| result.expect("caller checked coplanar cutter-hole contact difference"))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Intersection {
        return Ok(None);
    }
    let Some(intersection) = intersect_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact coplanar surface partial-overlap intersection",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_convex_union(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Union {
        return Ok(None);
    }
    let Some(union) = union_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &union.mesh,
        "exact convex coplanar surface partial-overlap union",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceConvexUnion,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_arrangement_union(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Union {
        return Ok(None);
    }
    let Some(union) = arrange_single_triangle_coplanar_union(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &union.mesh,
        "exact planar-arrangement coplanar surface union",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_corner_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = difference_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact one-corner coplanar surface difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceCornerDifference,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_arrangement_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_single_triangle_coplanar_difference(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact planar-arrangement coplanar surface difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceArrangementDifference,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_single_triangle_coplanar_holed_difference(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact holed coplanar surface difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_multi_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_coplanar_convex_surface_multi_holed_difference(left, right)
    else {
        return Ok(None);
    };
    difference.validate_against_sources(left, right)?;
    let mesh = copy_mesh(
        &difference.mesh,
        "exact coplanar convex multi-holed difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiHoledDifference,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_component_holed_difference_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    arrange_coplanar_convex_surface_component_holed_difference(left, right)
        .map(|difference| {
            difference.validate_against_sources(left, right)?;
            let mesh = copy_mesh(
                &difference.mesh,
                "exact coplanar convex component-holed difference",
                validation,
            )?;
            Ok(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarConvexSurfaceComponentHoledDifference,
            ))
        })
        .transpose()
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_component_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_coplanar_convex_component_holed_difference_optional(
        left,
        right,
        ExactBooleanOperation::Difference,
        validation,
    )
    .map(|result| result.expect("caller checked convex coplanar component-holed difference"))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_orthogonal_surface_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let arrangement = match operation {
        ExactBooleanOperation::Union => arrange_coplanar_orthogonal_surface_union(left, right),
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_orthogonal_surface_intersection(left, right)
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_orthogonal_surface_difference(left, right)
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    };
    let Some(arrangement) = arrangement else {
        return Ok(None);
    };
    arrangement.validate_against_sources(left, right)?;
    let (label, shortcut) = match arrangement.operation {
        CoplanarOrthogonalSurfaceOperation::Union => (
            "exact coplanar orthogonal surface union",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceUnion,
        ),
        CoplanarOrthogonalSurfaceOperation::Intersection => (
            "exact coplanar orthogonal surface intersection",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceIntersection,
        ),
        CoplanarOrthogonalSurfaceOperation::Difference => (
            "exact coplanar orthogonal surface difference",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceDifference,
        ),
    };
    let mesh = copy_mesh(&arrangement.mesh, label, validation)?;
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_orthogonal_surface(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)
        .map(|result| result.expect("caller checked coplanar orthogonal surface arrangement"))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_affine_surface_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let arrangement = match operation {
        ExactBooleanOperation::Union => arrange_coplanar_affine_surface_union(left, right),
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_affine_surface_intersection(left, right)
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_affine_surface_difference(left, right)
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    };
    let Some(arrangement) = arrangement else {
        return Ok(None);
    };
    arrangement.validate_against_sources(left, right)?;
    let (label, shortcut) = match arrangement.operation {
        CoplanarOrthogonalSurfaceOperation::Union => (
            "exact coplanar affine surface union",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceUnion,
        ),
        CoplanarOrthogonalSurfaceOperation::Intersection => (
            "exact coplanar affine surface intersection",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceIntersection,
        ),
        CoplanarOrthogonalSurfaceOperation::Difference => (
            "exact coplanar affine surface difference",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceDifference,
        ),
    };
    let mesh = copy_mesh(&arrangement.mesh, label, validation)?;
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_affine_surface(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_coplanar_affine_surface_optional(left, right, operation, validation)
        .map(|result| result.expect("caller checked coplanar affine surface arrangement"))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = union_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box union");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxUnion,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = intersection_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box intersection");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxIntersection,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = difference_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box difference");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = multi_difference_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box split difference");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxMultiDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_nested_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = nested_difference_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box nested difference");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxNestedDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_empty_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = empty_difference_axis_aligned_boxes(left, right, validation)?
        .expect("caller checked axis-aligned box empty difference");
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::AxisAlignedBoxEmptyDifference,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_box_cell_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let (mesh, shortcut) = match operation {
        ExactBooleanOperation::Union => {
            let Some(mesh) = cell_union_axis_aligned_boxes(left, right, validation)? else {
                return Ok(None);
            };
            (mesh, ExactBooleanShortcutKind::AxisAlignedBoxCellUnion)
        }
        ExactBooleanOperation::Difference => {
            let Some(mesh) = cell_difference_axis_aligned_boxes(left, right, validation)? else {
                return Ok(None);
            };
            (mesh, ExactBooleanShortcutKind::AxisAlignedBoxCellDifference)
        }
        ExactBooleanOperation::Intersection | ExactBooleanOperation::SelectedRegions(_) => {
            return Ok(None);
        }
    };
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_axis_aligned_orthogonal_solid_cell_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
        return Ok(None);
    };
    let label = match solid_operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            "exact axis-aligned orthogonal solid cell union"
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            "exact axis-aligned orthogonal solid cell intersection"
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            "exact axis-aligned orthogonal solid cell difference"
        }
    };
    let Some(mesh) = materialize_axis_aligned_orthogonal_solid_cells(
        left,
        right,
        solid_operation,
        label,
        validation,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        axis_aligned_orthogonal_solid_shortcut(solid_operation),
    )))
}

#[cfg(feature = "exact-triangulation")]
const fn axis_aligned_orthogonal_solid_operation(
    operation: ExactBooleanOperation,
) -> Option<AxisAlignedOrthogonalSolidOperation> {
    match operation {
        ExactBooleanOperation::Union => Some(AxisAlignedOrthogonalSolidOperation::Union),
        ExactBooleanOperation::Intersection => {
            Some(AxisAlignedOrthogonalSolidOperation::Intersection)
        }
        ExactBooleanOperation::Difference => Some(AxisAlignedOrthogonalSolidOperation::Difference),
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

#[cfg(feature = "exact-triangulation")]
const fn axis_aligned_orthogonal_solid_support(
    operation: AxisAlignedOrthogonalSolidOperation,
) -> ExactBooleanSupport {
    match operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference
        }
    }
}

#[cfg(feature = "exact-triangulation")]
const fn axis_aligned_orthogonal_solid_shortcut(
    operation: AxisAlignedOrthogonalSolidOperation,
) -> ExactBooleanShortcutKind {
    match operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellUnion
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellIntersection
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellDifference
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_affine_box(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let arrangement = match operation {
        AffineBoxOperation::Union => materialize_affine_box_union(left, right, validation)?,
        AffineBoxOperation::Intersection => {
            materialize_affine_box_intersection(left, right, validation)?
        }
        AffineBoxOperation::Difference => {
            materialize_affine_box_difference(left, right, validation)?
        }
    }
    .expect("caller checked affine box materialization");
    arrangement.validate_against_sources(left, right)?;
    let shortcut = match operation {
        AffineBoxOperation::Union => ExactBooleanShortcutKind::AffineBoxUnion,
        AffineBoxOperation::Intersection => ExactBooleanShortcutKind::AffineBoxIntersection,
        AffineBoxOperation::Difference => ExactBooleanShortcutKind::AffineBoxDifference,
    };
    Ok(certified_shortcut_result(arrangement.mesh, shortcut))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_affine_orthogonal_solid(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let arrangement = match operation {
        AffineOrthogonalSolidOperation::Union => {
            materialize_affine_orthogonal_solid_union(left, right, validation)?
        }
        AffineOrthogonalSolidOperation::Intersection => {
            materialize_affine_orthogonal_solid_intersection(left, right, validation)?
        }
        AffineOrthogonalSolidOperation::Difference => {
            materialize_affine_orthogonal_solid_difference(left, right, validation)?
        }
    }
    .expect("caller checked affine orthogonal solid materialization");
    arrangement.validate_against_sources(left, right)?;
    let shortcut = match operation {
        AffineOrthogonalSolidOperation::Union => {
            ExactBooleanShortcutKind::AffineOrthogonalSolidCellUnion
        }
        AffineOrthogonalSolidOperation::Intersection => {
            ExactBooleanShortcutKind::AffineOrthogonalSolidCellIntersection
        }
        AffineOrthogonalSolidOperation::Difference => {
            ExactBooleanShortcutKind::AffineOrthogonalSolidCellDifference
        }
    };
    Ok(certified_shortcut_result(arrangement.mesh, shortcut))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_open_surface_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !certify_open_surface_disjoint_report(left, right)?.is_certified() {
        return Ok(None);
    }

    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact open-surface disjoint union",
            validation,
        )?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact open-surface disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact open-surface disjoint difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::OpenSurfaceDisjoint,
    )))
}

/// Certify whether two open surface meshes are disjoint by exact graph facts.
///
/// This is the report form of the open-surface named-boolean shortcut. It
/// validates the open-surface precondition from exact mesh facts, then records
/// the retained graph relation counts that prove no face pair survived exact
/// scheduling. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the absence of intersection topology is a certified
/// graph fact, not a tolerance side effect.
#[cfg(feature = "exact-triangulation")]
pub fn certify_open_surface_disjoint_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactOpenSurfaceDisjointReport, MeshError> {
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    if !left_open_surface || !right_open_surface {
        return Ok(open_surface_disjoint_report(
            ExactOpenSurfaceDisjointStatus::NotOpenSurface,
            left_open_surface,
            right_open_surface,
            false,
            0,
            0,
            GraphRelationCounts::default(),
        ));
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(&graph);
    let status = if graph_had_unknowns {
        ExactOpenSurfaceDisjointStatus::GraphUnknowns
    } else if graph.face_pairs.is_empty() {
        ExactOpenSurfaceDisjointStatus::Certified
    } else {
        ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
    };
    Ok(open_surface_disjoint_report(
        status,
        left_open_surface,
        right_open_surface,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn open_surface_disjoint_report(
    status: ExactOpenSurfaceDisjointStatus,
    left_open_surface: bool,
    right_open_surface: bool,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: GraphRelationCounts,
) -> ExactOpenSurfaceDisjointReport {
    let blocker_kind = if matches!(status, ExactOpenSurfaceDisjointStatus::GraphUnknowns) {
        ExactBooleanBlockerKind::NeedsRefinement
    } else {
        ExactBooleanBlockerKind::NeedsWinding
    };
    ExactOpenSurfaceDisjointReport {
        status,
        left_open_surface,
        right_open_surface,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        blocker: counts.into_blocker(blocker_kind),
    }
}

#[cfg(feature = "exact-triangulation")]
fn mesh_is_open_surface(mesh: &ExactMesh) -> bool {
    !mesh.triangles().is_empty()
        && !mesh.facts().mesh.closed_manifold
        && mesh.facts().mesh.boundary_edges > 0
        && mesh.facts().mesh.non_manifold_edges == 0
        && mesh.facts().mesh.non_manifold_vertices == 0
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_containment(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(containment) = certified_coplanar_surface_boolean_support(left, right, operation)
    else {
        return Ok(None);
    };

    let mesh = match (containment, operation) {
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Union) => copy_mesh(
            right,
            "exact coplanar surface containment union keeps outer right",
            validation,
        )?,
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact coplanar surface containment intersection keeps inner left",
                validation,
            )?
        }
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Difference) => {
            empty_mesh(
                "empty exact coplanar surface containment difference",
                validation,
            )?
        }
        (CoplanarSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Union) => copy_mesh(
            left,
            "exact coplanar surface containment union keeps outer left",
            validation,
        )?,
        (CoplanarSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact coplanar surface containment intersection keeps inner right",
                validation,
            )?
        }
        (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_),
        )
        | (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::SelectedRegions(_),
        ) => unreachable!("unsupported or selected operation filtered by caller"),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceContainment,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn certified_coplanar_surface_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<CoplanarSurfaceContainment> {
    let containment = certify_single_triangle_coplanar_containment(left, right)?;
    match (containment, operation) {
        (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::Union
            | ExactBooleanOperation::Intersection
            | ExactBooleanOperation::Difference,
        )
        | (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection,
        ) => Some(containment),
        (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_),
        )
        | (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::SelectedRegions(_),
        ) => None,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_same_surface_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            copy_mesh(mesh, "exact same-surface boolean result", validation)?
        }
        ExactBooleanOperation::Difference => {
            empty_mesh("empty exact same-surface difference", validation)?
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::SameSurface,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_full_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let union = materialize_full_face_adjacent_union(left, right, validation).ok_or_else(|| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            "exact full-face adjacent union certificate did not replay",
        ))
    })?;
    union
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact full-face adjacent union/source replay failed: {error:?}"),
            ))
        })?;
    Ok(certified_shortcut_result(
        union.mesh,
        ExactBooleanShortcutKind::FullFaceAdjacentUnion,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_equivalent_surfaces(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => copy_mesh(
            mesh,
            "exact coplanar convex equivalent surface result",
            validation,
        )?,
        ExactBooleanOperation::Difference => empty_mesh(
            "empty exact coplanar convex equivalent surface difference",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceEquivalence,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_convex_containment_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let containment = certify_coplanar_convex_surface_containment(left, right)
        .expect("caller checked convex coplanar containment");
    let mesh = match (containment.relation, operation) {
        (CoplanarConvexSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Union) => {
            copy_mesh(
                right,
                "exact coplanar convex containment union keeps outer right",
                validation,
            )?
        }
        (
            CoplanarConvexSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::Intersection,
        ) => copy_mesh(
            left,
            "exact coplanar convex containment intersection keeps inner left",
            validation,
        )?,
        (CoplanarConvexSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Difference) => {
            empty_mesh(
                "empty exact coplanar convex containment difference",
                validation,
            )?
        }
        (CoplanarConvexSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Union) => {
            copy_mesh(
                left,
                "exact coplanar convex containment union keeps outer left",
                validation,
            )?
        }
        (
            CoplanarConvexSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Intersection,
        ) => copy_mesh(
            right,
            "exact coplanar convex containment intersection keeps inner right",
            validation,
        )?,
        (CoplanarConvexSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Difference) => {
            let difference = arrange_coplanar_convex_surface_holed_difference(left, right)
                .expect("right-inside-left containment should materialize one holed sheet");
            copy_mesh(
                &difference.mesh,
                "exact coplanar convex containment holed difference",
                validation,
            )?
        }
        (_, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
    };

    let shortcut = if containment.relation == CoplanarConvexSurfaceContainment::RightInsideLeft
        && operation == ExactBooleanOperation::Difference
    {
        ExactBooleanShortcutKind::CoplanarConvexSurfaceHoledDifference
    } else {
        ExactBooleanShortcutKind::CoplanarConvexSurfaceContainment
    };

    Ok(certified_shortcut_result(mesh, shortcut))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_boundary_touching_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject {
        return Ok(None);
    }
    if !certify_boundary_touching_report(left, right)?.is_certified() {
        return Ok(None);
    }

    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact boundary-touch union preserving separate shells",
            validation,
        )?,
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact boundary-touch lower-dimensional intersection",
            validation,
        )?,
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact boundary-touch difference preserving left shell",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(Some(boundary_policy_shortcut_result(mesh, operation)))
}

/// Certify whether retained graph pairs are exclusively boundary-only contacts.
///
/// The report keeps the exact graph relation counts used by boundary-policy
/// preflight and by [`boolean_exact_with_boundary_policy`]. Boundary-only
/// topology is intentionally not silently materialized by the default named
/// boolean API; Yap's exact computation model requires this projection into a
/// triangle-mesh-only result to be an explicit caller policy.
#[cfg(feature = "exact-triangulation")]
pub fn certify_boundary_touching_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    boundary_touching_report_from_graph(&graph, left, right)
}

/// Certify whether a named operation needs planar arrangement output.
///
/// The report is intentionally narrower than full winding preflight. It only
/// answers the coplanar positive-area case where exact graph facts prove that
/// intersection, union, or difference output is a planar arrangement problem. Existing
/// single-triangle and convex multi-face coplanar shortcuts are reported as
/// already materialized so callers can distinguish a missing output model from
/// a handled certified fragment.
#[cfg(feature = "exact-triangulation")]
pub fn certify_planar_arrangement_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(planar_arrangement_report(
            operation,
            ExactPlanarArrangementStatus::NotNamedOperation,
            false,
            0,
            0,
            GraphRelationCounts::default(),
            None,
        ));
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    planar_arrangement_report_from_graph(&graph, left, right, operation)
}

/// Certify whether exact graph construction needs refinement.
///
/// This is the standalone report form of the `UnresolvedGraph` preflight
/// branch. It separates unknown predicate outcomes and failed exact
/// constructions from later boundary, planar-arrangement, or winding policy,
/// following Yap's requirement that unresolved combinatorics stay explicit
/// rather than being folded into a generic unsupported boolean.
#[cfg(feature = "exact-triangulation")]
pub fn certify_refinement_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactRefinementReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(refinement_report_from_graph(&graph, operation))
}

/// Prepare and report the exact facts needed by a future winding policy.
///
/// This function stops at the same boundary as unsupported nontrivial named
/// booleans: it extracts the certified graph, rejects unresolved/boundary/
/// planar-arrangement cases into explicit statuses, then validates split
/// regions and records opposite-plane classifications. It is an auditable
/// replacement for "try winding with floats" in Yap's exact-computation model.
#[cfg(feature = "exact-triangulation")]
pub fn certify_winding_readiness_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    winding_readiness_report_from_graph(&graph, left, right, operation)
}

#[cfg(feature = "exact-triangulation")]
/// Validate the retained graph/source-handle handoff for public reports.
///
/// Boolean preflight and report constructors are public exact-computation
/// boundaries. They must reject a retained graph whose face, edge, vertex, or
/// plane handles no longer replay against the source meshes before policy
/// reports can consume those events. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): exact state
/// includes the combinatorial object handles attached to predicate evidence,
/// not just the numeric predicate result.
fn validate_graph_source_handoff(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), MeshError> {
    graph
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("retained exact intersection graph failed source replay: {error:?}"),
            ))
        })
}

#[cfg(feature = "exact-triangulation")]
fn boundary_touching_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    let status = if graph_had_unknowns {
        ExactBoundaryTouchingStatus::GraphUnknowns
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactBoundaryTouchingStatus::Certified
    } else {
        ExactBoundaryTouchingStatus::NotBoundaryOnly
    };
    Ok(ExactBoundaryTouchingReport {
        status,
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: counts.into_blocker(if graph_had_unknowns {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }),
    })
}

#[cfg(feature = "exact-triangulation")]
fn refinement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
) -> ExactRefinementReport {
    let counts = graph_relation_counts(graph);
    let graph_had_unknowns = graph.has_unknowns();
    let needs_refinement = graph_had_unknowns || counts.construction_failed_events > 0;
    ExactRefinementReport {
        operation,
        status: if needs_refinement {
            ExactRefinementStatus::Required
        } else {
            ExactRefinementStatus::NotRequired
        },
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: needs_refinement
            .then(|| counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
    }
}

#[cfg(feature = "exact-triangulation")]
fn planar_arrangement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    let arrangement_readiness = if graph_had_unknowns {
        None
    } else {
        Some(graph.coplanar_arrangement_readiness_report(left, right)?)
    };
    let status = if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        ExactPlanarArrangementStatus::NotNamedOperation
    } else if graph_had_unknowns {
        ExactPlanarArrangementStatus::GraphUnknowns
    } else if coplanar_surface_output_already_materialized(left, right, operation) {
        ExactPlanarArrangementStatus::AlreadyMaterialized
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactPlanarArrangementStatus::BoundaryPolicyRequired
    } else if graph_requires_planar_arrangement(graph) {
        ExactPlanarArrangementStatus::Required
    } else {
        ExactPlanarArrangementStatus::NoPositiveOverlap
    };
    Ok(planar_arrangement_report(
        operation,
        status,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts,
        arrangement_readiness,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn planar_arrangement_report(
    operation: ExactBooleanOperation,
    status: ExactPlanarArrangementStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: GraphRelationCounts,
    arrangement_readiness: Option<super::graph::CoplanarArrangementReadinessReport>,
) -> ExactPlanarArrangementReport {
    let blocker_kind = match status {
        ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::NeedsRefinement,
        ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }
        _ => ExactBooleanBlockerKind::NeedsPlanarArrangement,
    };
    ExactPlanarArrangementReport {
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        blocker: counts.into_blocker(blocker_kind),
        arrangement_readiness,
    }
}

#[cfg(feature = "exact-triangulation")]
fn coplanar_surface_output_already_materialized(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return true;
    }
    match operation {
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_convex_surface_intersection(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_intersection(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_intersection(left, right).is_some()
                || arrange_coplanar_affine_surface_intersection(left, right).is_some()
                || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
        }
        ExactBooleanOperation::Union => {
            arrange_coplanar_convex_surface_union(left, right).is_some()
                || arrange_coplanar_convex_surface_component_union(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_union(left, right).is_some()
                || arrange_coplanar_affine_surface_union(left, right).is_some()
                || union_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_union(left, right).is_some()
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_convex_surface_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_holed_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_difference(left, right).is_some()
                || arrange_coplanar_affine_surface_difference(left, right).is_some()
                || difference_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_difference(left, right).is_some()
                || arrange_single_triangle_coplanar_holed_difference(left, right).is_some()
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

#[cfg(feature = "exact-triangulation")]
fn winding_readiness_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::NotNamedOperation,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
        ));
    }
    if graph_had_unknowns {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::GraphUnknowns,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement),
            None,
        ));
    }
    if graph_requires_boundary_policy(graph, left, right)? {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::BoundaryPolicyRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            None,
        ));
    }
    let planar_report = planar_arrangement_report_from_graph(graph, left, right, operation)?;
    if planar_report.is_required() {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::PlanarArrangementRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsPlanarArrangement),
            planar_report.arrangement_readiness,
        ));
    }
    if planar_report.status == ExactPlanarArrangementStatus::AlreadyMaterialized {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsPlanarArrangement),
            planar_report.arrangement_readiness,
        ));
    }
    if let Some((region_classifications, triangulations, _volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(graph, left, right)?.filter(
            |(_, triangulations, volumetric_classifications)| {
                volumetric_classifications
                    .iter()
                    .all(|classification| classification.relation.is_materialization_decided())
                    && operation_retains_any_volumetric_region(
                        operation,
                        triangulations,
                        volumetric_classifications,
                    )
                    && volumetric_plan_materializes_operation(
                        operation,
                        triangulations,
                        volumetric_classifications,
                        left,
                        right,
                        ValidationPolicy::CLOSED,
                    )
            },
        )
    {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::Ready,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            triangulations.len(),
            region_classifications,
            if graph_requires_coplanar_volumetric_cells(&counts) {
                counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells)
            } else {
                counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding)
            },
            None,
        ));
    }
    if graph_requires_coplanar_volumetric_cells(&counts) {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells),
            None,
        ));
    }
    if graph.face_pairs.is_empty() {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::NoNontrivialOverlap,
            graph_had_unknowns,
            0,
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
        ));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    Ok(winding_readiness_report(
        operation,
        ExactWindingReadinessStatus::Ready,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        region_plan.regions.len(),
        region_classifications,
        counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
        None,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn winding_readiness_report(
    operation: ExactBooleanOperation,
    status: ExactWindingReadinessStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    region_count: usize,
    region_classifications: Vec<FaceRegionPlaneClassification>,
    blocker: ExactBooleanBlocker,
    arrangement_readiness: Option<super::graph::CoplanarArrangementReadinessReport>,
) -> ExactWindingReadinessReport {
    ExactWindingReadinessReport {
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        region_count,
        region_classifications,
        blocker,
        arrangement_readiness,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_convex_containment_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(support) = certified_convex_boolean_support(left, right)? else {
        return Ok(None);
    };

    let left_in_right = classify_mesh_vertices_against_convex_solid(left, right);
    let right_in_left = classify_mesh_vertices_against_convex_solid(right, left);
    let mesh = match (left_in_right, right_in_left, operation) {
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Union) => copy_mesh(
            right,
            "exact convex containment union keeps outer right",
            validation,
        )?,
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact convex containment intersection keeps inner left",
                validation,
            )?
        }
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Difference) => {
            empty_mesh("empty exact convex containment difference", validation)?
        }
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Union) => copy_mesh(
            left,
            "exact convex containment union keeps outer left",
            validation,
        )?,
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact convex containment intersection keeps inner right",
                validation,
            )?
        }
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Difference) => {
            concatenate_meshes_with_options(
                left,
                right,
                true,
                "exact convex containment difference with inner reversed shell",
                validation,
            )?
        }
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Union,
        ) => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact convex separated union",
            validation,
        )?,
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Intersection,
        ) => empty_mesh("empty exact convex separated intersection", validation)?,
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Difference,
        ) => copy_mesh(
            left,
            "exact convex separated difference keeps left",
            validation,
        )?,
        (_, _, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
        _ => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        match support {
            ExactBooleanSupport::CertifiedConvexContainment => {
                ExactBooleanShortcutKind::ConvexContainment
            }
            ExactBooleanSupport::CertifiedConvexSeparated => {
                ExactBooleanShortcutKind::ConvexSeparated
            }
            _ => unreachable!("convex support helper returns only certified convex shortcuts"),
        },
    )))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_winding_containment_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(support) = certified_winding_boolean_support(left, right)? else {
        return Ok(None);
    };

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(winding_error)?;

    let mesh = match (left_in_right.relation, right_in_left.relation, operation) {
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Union) => {
            copy_mesh(
                right,
                "exact winding containment union keeps outer right",
                validation,
            )?
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact winding containment intersection keeps inner left",
                validation,
            )?
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Difference) => {
            empty_mesh("empty exact winding containment difference", validation)?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Union) => {
            copy_mesh(
                left,
                "exact winding containment union keeps outer left",
                validation,
            )?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact winding containment intersection keeps inner right",
                validation,
            )?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Difference) => {
            concatenate_meshes_with_options(
                left,
                right,
                true,
                "exact winding containment difference with inner reversed shell",
                validation,
            )?
        }
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Union,
        ) => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact winding separated union",
            validation,
        )?,
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Intersection,
        ) => empty_mesh("empty exact winding separated intersection", validation)?,
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Difference,
        ) => copy_mesh(
            left,
            "exact winding separated difference keeps left",
            validation,
        )?,
        (_, _, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
        _ => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        match support {
            ExactBooleanSupport::CertifiedWindingContainment => {
                ExactBooleanShortcutKind::WindingContainment
            }
            ExactBooleanSupport::CertifiedWindingSeparated => {
                ExactBooleanShortcutKind::WindingSeparated
            }
            _ => unreachable!("winding support helper returns only winding shortcuts"),
        },
    )))
}

#[cfg(feature = "exact-triangulation")]
type VolumetricWindingRegionPlan = (
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
    Vec<ExactVolumetricRegionClassification>,
);

#[cfg(feature = "exact-triangulation")]
fn boolean_volumetric_winding_regions(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some((region_classifications, triangulations, volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(&graph, left, right)?
    else {
        return Ok(None);
    };
    if !volumetric_classifications
        .iter()
        .all(|classification| classification.relation.is_materialization_decided())
    {
        return Ok(None);
    }
    if !operation_retains_any_volumetric_region(
        operation,
        &triangulations,
        &volumetric_classifications,
    ) {
        return Ok(None);
    }

    let assembly =
        ExactBooleanAssemblyPlan::from_region_triangulations_with_triangle_retention_and_sources(
            &triangulations,
            left,
            right,
            |triangulation, triangle| {
                volumetric_retention_for_operation(
                    operation,
                    triangulation,
                    triangle,
                    &volumetric_classifications,
                )
            },
        )
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                format!("exact winding region assembly failed: {error}"),
            ))
        })?;
    let mesh = match assembly.checked_to_exact_mesh_with_sources(left, right, validation) {
        Ok(mesh) => mesh,
        Err(_error) if graph_requires_coplanar_volumetric_cells(&graph_relation_counts(&graph)) => {
            return Ok(None);
        }
        Err(error) => return Err(error),
    };
    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::WindingMaterialized { operation },
        graph_had_unknowns: false,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications,
        mesh,
    };
    result
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact winding-materialized result/source replay failed: {error:?}"),
            ))
        })?;
    Ok(Some(result))
}

#[cfg(feature = "exact-triangulation")]
fn volumetric_winding_region_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<VolumetricWindingRegionPlan>, MeshError> {
    let counts = graph_relation_counts(graph);
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs != 0
        || counts.construction_failed_events != 0
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }
    if graph_requires_boundary_policy(graph, left, right)? {
        return Ok(None);
    }

    let cell_plan = match triangulate_all_face_cells_with_cdt(graph, left, right) {
        Ok(plan) => plan,
        Err(_error) if graph_requires_coplanar_volumetric_cells(&counts) => {
            // Coplanar source-face overlaps can expose constraint-normalization
            // cases that are not part of the current bounded volumetric cell
            // materializer. Keep Yap's exact boundary explicit: the caller
            // receives `RequiresCoplanarVolumetricCells` instead of a generic
            // triangulation failure or a tolerance fallback.
            return Ok(None);
        }
        Err(error) => {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("exact winding face-cell triangulation failed: {error}"),
            )));
        }
    };
    let Some((region_plan, triangulations)) = cell_plan else {
        return Ok(None);
    };
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let volumetric_classifications =
        classify_triangulated_regions_against_opposite_meshes(&triangulations, left, right)
            .map_err(volumetric_error)?;
    Ok(Some((
        region_classifications,
        triangulations,
        volumetric_classifications,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn operation_retains_any_volumetric_region(
    operation: ExactBooleanOperation,
    triangulations: &[FaceRegionTriangulation],
    classifications: &[ExactVolumetricRegionClassification],
) -> bool {
    triangulations.iter().any(|triangulation| {
        triangulation.triangles.chunks_exact(3).any(|triangle| {
            volumetric_retention_for_operation(
                operation,
                triangulation,
                [triangle[0], triangle[1], triangle[2]],
                classifications,
            ) != ExactRegionRetention::Drop
        })
    })
}

#[cfg(feature = "exact-triangulation")]
fn volumetric_plan_materializes_operation(
    operation: ExactBooleanOperation,
    triangulations: &[FaceRegionTriangulation],
    classifications: &[ExactVolumetricRegionClassification],
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> bool {
    let Ok(assembly) =
        ExactBooleanAssemblyPlan::from_region_triangulations_with_triangle_retention_and_sources(
            triangulations,
            left,
            right,
            |triangulation, triangle| {
                volumetric_retention_for_operation(
                    operation,
                    triangulation,
                    triangle,
                    classifications,
                )
            },
        )
    else {
        return false;
    };
    assembly
        .checked_to_exact_mesh_with_sources(left, right, validation)
        .is_ok()
}

#[cfg(feature = "exact-triangulation")]
fn volumetric_retention_for_operation(
    operation: ExactBooleanOperation,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    classifications: &[ExactVolumetricRegionClassification],
) -> ExactRegionRetention {
    let Some(classification) = classifications.iter().find(|classification| {
        classification.region_side == triangulation.side
            && classification.region_face == triangulation.face
            && classification.triangle == triangle
    }) else {
        return ExactRegionRetention::Drop;
    };
    // Boundary cells arise when every exact representative for a source-face
    // cell lies on the opposite closed mesh boundary. In mixed coplanar
    // volumetric overlaps that means the same geometric patch is normally
    // emitted by both operands. Yap, "Towards Exact Geometric Computation,"
    // Comput. Geom. 7.1-2 (1997), requires that this non-strict state remain
    // explicit, so we consume it through a deterministic owner policy instead
    // of pretending it is inside or outside: union/intersection keep the left
    // copy and drop the coincident right copy; difference drops coincident
    // boundary cells because the overlapped volume is removed from the left
    // operand and right boundary faces are only used as reversed interior caps.
    match (operation, triangulation.side, classification.relation) {
        (ExactBooleanOperation::Union, _, ExactVolumetricRegionRelation::Outside)
        | (ExactBooleanOperation::Union, MeshSide::Left, ExactVolumetricRegionRelation::Boundary)
        | (ExactBooleanOperation::Intersection, _, ExactVolumetricRegionRelation::Inside)
        | (
            ExactBooleanOperation::Intersection,
            MeshSide::Left,
            ExactVolumetricRegionRelation::Boundary,
        )
        | (
            ExactBooleanOperation::Difference,
            MeshSide::Left,
            ExactVolumetricRegionRelation::Outside,
        ) => ExactRegionRetention::Keep,
        (
            ExactBooleanOperation::Difference,
            MeshSide::Right,
            ExactVolumetricRegionRelation::Inside,
        ) => ExactRegionRetention::KeepReversed,
        _ => ExactRegionRetention::Drop,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_convex_single_cap_difference_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = subtract_closed_convex_solids_single_cap(left, right) else {
        return Ok(None);
    };
    difference.validate_against_sources(left, right)?;
    let mesh = copy_mesh(
        &difference.mesh,
        "exact closed-convex single-cap difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ConvexSingleCapDifference,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn certified_convex_intersection_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Intersection
            if intersect_closed_convex_solids(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedConvexIntersection)
        }
        _ => None,
    }
}

#[cfg(feature = "exact-triangulation")]
fn certified_convex_single_cap_difference_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Difference
            if subtract_closed_convex_solids_single_cap(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedConvexSingleCapDifference)
        }
        _ => None,
    }
}

#[cfg(feature = "exact-triangulation")]
fn certified_convex_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    if graph.has_unknowns() || !graph.face_pairs.is_empty() {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid(left, right);
    let right_in_left = classify_mesh_vertices_against_convex_solid(right, left);
    Ok(match (left_in_right, right_in_left) {
        (ConvexSolidMeshRelation::StrictlyInside, _)
        | (_, ConvexSolidMeshRelation::StrictlyInside) => {
            Some(ExactBooleanSupport::CertifiedConvexContainment)
        }
        (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
            Some(ExactBooleanSupport::CertifiedConvexSeparated)
        }
        _ => None,
    })
}

#[cfg(feature = "exact-triangulation")]
fn certified_winding_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if graph.has_unknowns()
        || !graph.face_pairs.is_empty()
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(winding_error)?;
    Ok(match (left_in_right.relation, right_in_left.relation) {
        (ClosedMeshWindingMeshRelation::StrictlyInside, _)
        | (_, ClosedMeshWindingMeshRelation::StrictlyInside) => {
            Some(ExactBooleanSupport::CertifiedWindingContainment)
        }
        (ClosedMeshWindingMeshRelation::Outside, ClosedMeshWindingMeshRelation::Outside) => {
            Some(ExactBooleanSupport::CertifiedWindingSeparated)
        }
        _ => None,
    })
}

#[cfg(feature = "exact-triangulation")]
fn winding_error(error: WindingReportError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact winding report/source replay failed: {error:?}"),
    ))
}

#[cfg(feature = "exact-triangulation")]
fn volumetric_error(error: ExactVolumetricRegionError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact volumetric winding region report/source replay failed: {error:?}"),
    ))
}

#[cfg(feature = "exact-triangulation")]
fn copy_mesh(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        mesh.triangles().to_vec(),
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn concatenate_meshes_with_options(
    left: &ExactMesh,
    right: &ExactMesh,
    reverse_right: bool,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = left.vertices().to_vec();
    let right_offset = vertices.len();
    vertices.extend_from_slice(right.vertices());
    let mut triangles = left.triangles().to_vec();
    triangles.extend(right.triangles().iter().map(|triangle| {
        let [a, b, c] = triangle.0;
        if reverse_right {
            Triangle([a + right_offset, c + right_offset, b + right_offset])
        } else {
            Triangle([a + right_offset, b + right_offset, c + right_offset])
        }
    }));
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_bounds_disjoint(left: &ExactMesh, right: &ExactMesh) -> bool {
    let (Some(left_bounds), Some(right_bounds)) = (&left.bounds().mesh, &right.bounds().mesh)
    else {
        return left.triangles().is_empty() || right.triangles().is_empty();
    };
    left_bounds.classify_intersection(right_bounds).value() == Some(AabbIntersectionKind::Disjoint)
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_identical(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.triangles() == right.triangles()
        && left.vertices().len() == right.vertices().len()
        && vertices_are_certified_equal(left, right)
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_same_surface(left: &ExactMesh, right: &ExactMesh) -> bool {
    certify_same_surface_report(left, right).is_certified()
}

#[cfg(feature = "exact-triangulation")]
/// Certify whether two meshes represent the same triangle surface.
///
/// The report preserves the exact coordinate-equality predicate certificates
/// used to find a vertex bijection and the sorted triangle sets compared after
/// remapping. This is the auditable form of the same-surface shortcut used by
/// named exact booleans, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): exact topology decisions should
/// expose the predicate facts that justify them.
pub fn certify_same_surface_report(left: &ExactMesh, right: &ExactMesh) -> ExactSameSurfaceReport {
    if left.vertices().len() != right.vertices().len() {
        return same_surface_report(
            ExactSameSurfaceStatus::VertexCountMismatch,
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
    }
    if left.triangles().len() != right.triangles().len() {
        return same_surface_report(
            ExactSameSurfaceStatus::TriangleCountMismatch,
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
    }

    let (left_to_right, predicates, status) = certified_vertex_permutation_report(left, right);
    if status != ExactSameSurfaceStatus::Certified {
        return same_surface_report(status, left_to_right, Vec::new(), predicates);
    }
    let right_to_left = invert_permutation(&left_to_right);

    let mut left_triangles = sorted_triangle_sets(left, None);
    let mut right_triangles = sorted_triangle_sets(right, Some(&right_to_left));
    left_triangles.sort_unstable();
    right_triangles.sort_unstable();
    let status = if left_triangles == right_triangles {
        ExactSameSurfaceStatus::Certified
    } else {
        ExactSameSurfaceStatus::TriangleSetMismatch
    };

    ExactSameSurfaceReport {
        status,
        left_to_right,
        right_to_left,
        left_triangles,
        right_triangles,
        predicates,
    }
}

#[cfg(feature = "exact-triangulation")]
fn same_surface_report(
    status: ExactSameSurfaceStatus,
    left_to_right: Vec<usize>,
    right_to_left: Vec<usize>,
    predicates: Vec<PredicateUse>,
) -> ExactSameSurfaceReport {
    ExactSameSurfaceReport {
        status,
        left_to_right,
        right_to_left,
        left_triangles: Vec::new(),
        right_triangles: Vec::new(),
        predicates,
    }
}

#[cfg(feature = "exact-triangulation")]
fn vertices_are_certified_equal(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices()
        .iter()
        .zip(right.vertices())
        .all(|(left, right)| {
            let left = left.to_hyperlimit_point();
            let right = right.to_hyperlimit_point();
            compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
                && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
                && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
        })
}

#[cfg(feature = "exact-triangulation")]
fn certified_vertex_permutation_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> (Vec<usize>, Vec<PredicateUse>, ExactSameSurfaceStatus) {
    let mut left_to_right = Vec::with_capacity(left.vertices().len());
    let mut used_right = vec![false; right.vertices().len()];
    let mut predicates = Vec::new();

    for left_vertex in left.vertices() {
        let left_point = left_vertex.to_hyperlimit_point();
        let mut match_index = None;
        let mut saw_undecided = false;
        for (right_index, right_vertex) in right.vertices().iter().enumerate() {
            if used_right[right_index] {
                continue;
            }
            let right_point = right_vertex.to_hyperlimit_point();
            let x = compare_reals_report(&left_point.x, &right_point.x);
            let y = compare_reals_report(&left_point.y, &right_point.y);
            let z = compare_reals_report(&left_point.z, &right_point.z);
            predicates.push(PredicateUse::from_certificate(x.certificate));
            predicates.push(PredicateUse::from_certificate(y.certificate));
            predicates.push(PredicateUse::from_certificate(z.certificate));
            let Some(x_value) = x.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let Some(y_value) = y.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let Some(z_value) = z.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let equal = x_value == Ordering::Equal
                && y_value == Ordering::Equal
                && z_value == Ordering::Equal;
            if equal {
                match_index = Some(right_index);
                break;
            }
        }
        let Some(match_index) = match_index else {
            let status = if saw_undecided {
                ExactSameSurfaceStatus::VertexMatchingUndecided
            } else {
                ExactSameSurfaceStatus::VertexCoordinateMismatch
            };
            return (left_to_right, predicates, status);
        };
        used_right[match_index] = true;
        left_to_right.push(match_index);
    }

    (left_to_right, predicates, ExactSameSurfaceStatus::Certified)
}

#[cfg(feature = "exact-triangulation")]
fn invert_permutation(permutation: &[usize]) -> Vec<usize> {
    let mut inverse = vec![0; permutation.len()];
    for (left_index, &right_index) in permutation.iter().enumerate() {
        inverse[right_index] = left_index;
    }
    inverse
}

#[cfg(feature = "exact-triangulation")]
fn sorted_triangle_sets(mesh: &ExactMesh, right_to_left: Option<&[usize]>) -> Vec<[usize; 3]> {
    mesh.triangles()
        .iter()
        .map(|triangle| {
            let mut vertices = triangle.0.map(|vertex| match right_to_left {
                Some(mapping) => mapping[vertex],
                None => vertex,
            });
            vertices.sort_unstable();
            vertices
        })
        .collect()
}

#[cfg(feature = "exact-triangulation")]
fn boolean_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            super::provenance::SourceProvenance::exact("exact disjoint left difference"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::BoundsDisjoint,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_empty_operand(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact intersection with empty operand", validation)?
        }
        ExactBooleanOperation::Difference if left.triangles().is_empty() => {
            empty_mesh("empty exact difference from empty left operand", validation)?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            super::provenance::SourceProvenance::exact("exact difference with empty right operand"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::EmptyOperand,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_identical_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            ExactMesh::new_with_policy(
                mesh.vertices().to_vec(),
                mesh.triangles().to_vec(),
                super::provenance::SourceProvenance::exact("exact identical boolean result"),
                validation,
            )?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            Vec::new(),
            Vec::new(),
            super::provenance::SourceProvenance::exact("empty exact identical difference"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::Identical,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn empty_mesh(label: &'static str, validation: ValidationPolicy) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn certified_shortcut_result(
    mesh: ExactMesh,
    shortcut: ExactBooleanShortcutKind,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::CertifiedShortcut { shortcut },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        mesh,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boundary_policy_shortcut_result(
    mesh: ExactMesh,
    operation: ExactBooleanOperation,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::BoundaryPolicyShortcut { operation },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        mesh,
    }
}

#[cfg(feature = "exact-triangulation")]
fn concatenate_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = left.vertices().to_vec();
    let right_offset = vertices.len();
    vertices.extend_from_slice(right.vertices());
    let mut triangles = left.triangles().to_vec();
    triangles.extend(right.triangles().iter().map(|triangle| {
        let [a, b, c] = triangle.0;
        Triangle([a + right_offset, b + right_offset, c + right_offset])
    }));
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        super::provenance::SourceProvenance::exact("exact disjoint union"),
        validation,
    )
}
