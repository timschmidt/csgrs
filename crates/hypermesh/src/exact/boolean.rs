//! Exact boolean operation entry points.
//!
//! The legacy boolmesh-derived public API mutates triangle topology through
//! primitive-float kernels. This module is the exact-stack replacement
//! boundary for the subset that is currently implemented: build certified
//! intersection events, form exact split-region loops, classify those regions,
//! triangulate them through feature-gated exact `hypertri`, assemble exact 3D
//! output triangles, and validate the resulting [`ExactMesh`].
//!
//! The operation policy is deliberately explicit. Until full winding and
//! inside/outside classification are ported, callers select which split-region
//! sides are retained rather than receiving a silently approximate
//! union/intersection/difference decision. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology
//! decisions must be certified or represented as policy choices/unknowns.

#[cfg(feature = "exact-triangulation")]
use super::bounds::AabbIntersectionKind;
#[cfg(feature = "exact-triangulation")]
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
#[cfg(feature = "exact-triangulation")]
use super::graph::build_intersection_graph;
#[cfg(feature = "exact-triangulation")]
use super::mesh::{ExactMesh, Triangle};
#[cfg(feature = "exact-triangulation")]
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionSelection, FaceRegionPlaneClassification,
    FaceRegionTriangulation, classify_face_regions_against_opposite_planes,
    triangulate_face_regions_with_earcut,
};
#[cfg(feature = "exact-triangulation")]
use super::validation::ValidationPolicy;
#[cfg(feature = "exact-triangulation")]
use hyperlimit::compare_reals;
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
/// to legacy float winding. Until exact inside/outside classification is
/// complete, only [`Self::SelectedRegions`] is executable; named operations
/// return [`DiagnosticKind::UnsupportedExactOperation`].
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanOperation {
    /// Assemble explicitly selected source-side split regions.
    SelectedRegions(ExactRegionSelection),
    /// Exact union once certified winding semantics are available.
    Union,
    /// Exact intersection once certified winding semantics are available.
    Intersection,
    /// Exact difference once certified winding semantics are available.
    Difference,
}

/// Auditable result of an exact selected-region boolean pipeline.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanResult {
    /// Whether graph extraction contained unknown events before policy checks.
    pub graph_had_unknowns: bool,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Exact projected triangulations used for assembly.
    pub triangulations: Vec<FaceRegionTriangulation>,
    /// Non-mutating exact output assembly.
    pub assembly: ExactBooleanAssemblyPlan,
    /// Materialized exact output mesh validated under the requested policy.
    pub mesh: ExactMesh,
}

/// Certified support level for a requested exact boolean operation.
///
/// This is the named-boolean staging boundary. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), frames exact geometric
/// computing as an application-level contract: unresolved combinatorics must be
/// represented explicitly instead of being decided by approximate arithmetic.
/// These variants therefore distinguish executable certified shortcuts from
/// cases whose split regions are available but still need exact winding policy.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanSupport {
    /// The request is an explicit selected-region assembly policy.
    SelectedRegionPolicy,
    /// A named operation was answered by exact empty-operand semantics.
    CertifiedEmptyOperand,
    /// A named operation was answered by certified disjoint AABBs.
    CertifiedBoundsDisjoint,
    /// A named operation was answered by exact coordinate and topology identity.
    CertifiedIdentical,
    /// Split-region facts were produced, but named winding semantics are not
    /// yet certified for this nontrivial overlap.
    RequiresCertifiedWinding,
    /// Graph extraction retained unresolved predicate events; callers must
    /// refine, reject, or use a policy that explicitly accepts uncertainty.
    UnresolvedGraph,
}

/// Preflight report for an exact boolean operation request.
///
/// The report gives callers a stable way to audit the current implementation
/// boundary. Shortcut variants are executable by [`boolean_exact`]. For
/// nontrivial named booleans, the report exposes the certified split-region
/// plane classifications that a future exact winding/inside-outside rule must
/// consume, without dispatching to the legacy tolerance kernel.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanPreflight {
    /// Requested operation.
    pub operation: ExactBooleanOperation,
    /// Certified support level for the request.
    pub support: ExactBooleanSupport,
    /// Whether retained graph events contain explicit unknowns.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact broad/narrow scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records across all retained face pairs.
    pub retained_events: usize,
    /// Number of split-region boundaries produced for classification.
    pub region_count: usize,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
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
    let graph_had_unknowns = graph.has_unknowns();
    if policy.reject_unknowns && graph_had_unknowns {
        return Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::DegenerateTriangle,
            "exact boolean graph contains unresolved predicate events",
        )));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        classify_face_regions_against_opposite_planes(&region_plan, left, right);
    let triangulations =
        triangulate_face_regions_with_earcut(&region_plan, left, right).map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let assembly =
        ExactBooleanAssemblyPlan::from_region_triangulations(&triangulations, policy.selection)
            .map_err(|error| {
                MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::IndexOutOfBounds,
                    format!("exact boolean assembly failed: {error}"),
                ))
            })?;
    let mesh = assembly.to_exact_mesh(policy.validation)?;

    Ok(ExactBooleanResult {
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        mesh,
    })
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
        | ExactBooleanOperation::Difference => ExactBooleanSupport::RequiresCertifiedWinding,
    };

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
    ) {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns: false,
            retained_face_pairs: 0,
            retained_events: 0,
            region_count: 0,
            region_classifications: Vec::new(),
        });
    }

    let graph = build_intersection_graph(left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        classify_face_regions_against_opposite_planes(&region_plan, left, right);
    let support = if graph_had_unknowns {
        ExactBooleanSupport::UnresolvedGraph
    } else {
        support
    };

    Ok(ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        region_count: region_plan.regions.len(),
        region_classifications,
    })
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
        | ExactBooleanOperation::Difference => Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            "named exact booleans require certified winding/inside-outside classification",
        ))),
    }
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
        && left
            .vertices()
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

    Ok(shortcut_result(mesh))
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

    Ok(shortcut_result(mesh))
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

    Ok(shortcut_result(mesh))
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
fn shortcut_result(mesh: ExactMesh) -> ExactBooleanResult {
    ExactBooleanResult {
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
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
