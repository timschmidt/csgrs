//! Auditable exact boolean reports.
//!
//! These types are the public evidence objects produced by the exact boolean
//! staging layer. They intentionally carry graph counts, predicate
//! certificates, and checked handoff artifacts instead of collapsing exact
//! topology decisions to `bool`. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): a geometric program
//! should expose whether a combinatorial decision was certified, unsupported,
//! or blocked on an application-level policy.

#[cfg(feature = "exact-triangulation")]
use hyperlimit::{Point3, compare_reals};
#[cfg(feature = "exact-triangulation")]
use std::cmp::Ordering;

#[cfg(feature = "exact-triangulation")]
use super::ExactMesh;
#[cfg(feature = "exact-triangulation")]
use super::MeshSide;
#[cfg(feature = "exact-triangulation")]
use super::boolean::ExactBooleanOperation;
#[cfg(feature = "exact-triangulation")]
use super::graph::{CoplanarArrangementReadinessReport, CoplanarArrangementReadinessStatus};
#[cfg(feature = "exact-triangulation")]
use super::provenance::PredicateUse;
#[cfg(feature = "exact-triangulation")]
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionSelection, FaceRegionPlaneClassification,
    FaceRegionPlaneRelation, FaceRegionPlaneValidationError, FaceRegionTriangulation,
};

/// Validation failure for an exact report object.
///
/// Report validation checks the evidence object itself, not the original
/// geometry. It lets tests, fuzzing, and downstream policy code assert that
/// status, blocker kind, graph counts, and retained artifacts agree before
/// later topology stages consume them. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), by making
/// metadata consistency part of the certified boundary.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactReportValidationError {
    /// A certified shortcut report unexpectedly carried a blocker.
    CertifiedReportHasBlocker,
    /// A blocked or unresolved report did not carry the required blocker.
    MissingBlocker,
    /// The blocker kind does not match the report support/status.
    WrongBlockerKind,
    /// The blocker kind and retained relation counts contradict each other.
    InvalidBlockerCounts,
    /// A report that should not materialize split-region facts retained them.
    UnexpectedRegionFacts,
    /// A winding-ready report did not retain checked region facts.
    MissingRegionFacts,
    /// A selected-region triangulation has no matching retained region/plane
    /// classification for its source region.
    UnclassifiedRegionTriangulation,
    /// A retained region/plane classification has no matching triangulated
    /// source region.
    OrphanedRegionClassification,
    /// An assembled selected-region output triangle has no matching retained
    /// source-region triangulation.
    UntriangulatedAssemblyRegion,
    /// An assembled selected-region output triangle uses a vertex source that
    /// is absent from the retained triangulation for its source region.
    AssemblyVertexOutsideTriangulation,
    /// A selected-region assembly retained an output vertex that no output
    /// triangle references.
    UnreferencedAssemblyVertex,
    /// A retained region/plane classification failed its own side-fact audit.
    InvalidRegionClassification(FaceRegionPlaneValidationError),
    /// A winding-ready report retained a region/plane classification that still
    /// depends on undecided or non-proof-producing predicate evidence.
    RegionClassificationNotProofProducing,
    /// A retained split-region triangulation failed its own audit.
    InvalidTriangulation,
    /// A retained output assembly plan failed its own audit.
    InvalidAssembly,
    /// The materialized output mesh failed retained-state validation.
    InvalidOutputMesh,
    /// A selected-region result's assembly and materialized mesh disagree.
    OutputMeshAssemblyMismatch,
    /// A shortcut result retained selected-region classification,
    /// triangulation, or assembly artifacts.
    ShortcutResultHasAssemblyArtifacts,
    /// A certified shortcut or boundary-policy result claimed unresolved graph
    /// events after materializing output topology.
    ShortcutResultHasUnknownGraph,
    /// A selected-region result claimed unresolved graph events after
    /// materializing output topology.
    SelectedRegionResultHasUnknownGraph,
    /// A selected-region result retained output triangles from a source side
    /// excluded by its declared selection policy.
    SelectedRegionAssemblyViolatesSelection,
    /// A certified graph shortcut retained graph events that contradict the
    /// shortcut status.
    UnexpectedGraphEvents,
    /// A required boundary or planar-arrangement report has no matching
    /// retained relation count.
    MissingRelationCount,
    /// A planar-arrangement report did not retain the checked coplanar graph
    /// readiness summary required for its status.
    MissingArrangementReadiness,
    /// A planar-arrangement report retained a readiness summary where none is
    /// coherent for its status.
    UnexpectedArrangementReadiness,
    /// A retained planar-arrangement readiness summary failed its own count
    /// audit.
    InvalidArrangementReadiness,
    /// The report's unknown-graph flag contradicts its status.
    GraphUnknownStatusMismatch,
    /// The report status contradicts retained preconditions, relation counts,
    /// operation class, or graph evidence.
    StatusEvidenceMismatch,
    /// Planar-arrangement blocker counts and retained readiness counts
    /// disagree.
    ArrangementReadinessMismatch,
    /// A same-surface report retained a non-bijective vertex permutation.
    InvalidPermutation,
    /// A certified same-surface report retained unequal remapped triangle sets.
    MismatchedTriangleSets,
}

#[cfg(feature = "exact-triangulation")]
fn blocker_kind(
    blocker: Option<&ExactBooleanBlocker>,
    expected: ExactBooleanBlockerKind,
) -> Result<(), ExactReportValidationError> {
    match blocker {
        Some(blocker) if blocker.kind == expected => Ok(()),
        Some(_) => Err(ExactReportValidationError::WrongBlockerKind),
        None => Err(ExactReportValidationError::MissingBlocker),
    }
}

#[cfg(feature = "exact-triangulation")]
fn no_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactReportValidationError> {
    if region_count == 0 && classifications.is_empty() {
        Ok(())
    } else {
        Err(ExactReportValidationError::UnexpectedRegionFacts)
    }
}

#[cfg(feature = "exact-triangulation")]
fn blocker_pair_count(blocker: &ExactBooleanBlocker) -> usize {
    blocker.candidate_pairs
        + blocker.coplanar_overlapping_pairs
        + blocker.coplanar_touching_pairs
        + blocker.unknown_pairs
}

#[cfg(feature = "exact-triangulation")]
fn validate_blocker_count_bounds(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    let blocker_pairs = blocker_pair_count(blocker);
    if (retained_face_pairs == 0 && retained_events != 0)
        || (retained_face_pairs != 0 && retained_events == 0)
        || (retained_face_pairs != 0 && blocker_pairs == 0)
        || blocker_pairs > retained_face_pairs
        || blocker.construction_failed_events > retained_events
    {
        Err(ExactReportValidationError::InvalidBlockerCounts)
    } else {
        Ok(())
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_arrangement_readiness_matches_blocker(
    readiness: &CoplanarArrangementReadinessReport,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactReportValidationError> {
    // The compact readiness report and the blocker are two public views of the
    // same retained coplanar graph state. Yap, "Towards Exact Geometric
    // Computation," Comput. Geom. 7.1-2 (1997), treats retained numerical
    // structure as part of the exact state; a later planar-cell or winding
    // policy must not be able to consume a summary with relabeled graph counts.
    if readiness.overlapping_graphs != blocker.coplanar_overlapping_pairs
        || readiness.touching_graphs != blocker.coplanar_touching_pairs
        || readiness.graph_count
            != blocker.coplanar_overlapping_pairs + blocker.coplanar_touching_pairs
    {
        Err(ExactReportValidationError::ArrangementReadinessMismatch)
    } else {
        Ok(())
    }
}

#[cfg(feature = "exact-triangulation")]
fn blocker_has_any_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker_pair_count(blocker) != 0 || blocker.construction_failed_events != 0
}

#[cfg(feature = "exact-triangulation")]
fn blocker_has_refinement_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0
}

#[cfg(feature = "exact-triangulation")]
fn operation_is_selected_region(operation: ExactBooleanOperation) -> bool {
    matches!(operation, ExactBooleanOperation::SelectedRegions(_))
}

#[cfg(feature = "exact-triangulation")]
fn checked_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactReportValidationError> {
    if region_count == 0 || classifications.is_empty() {
        return Err(ExactReportValidationError::MissingRegionFacts);
    }
    for classification in classifications {
        classification
            .validate()
            .map_err(ExactReportValidationError::InvalidRegionClassification)?;
        // A winding-ready handoff is stronger than a stored classification
        // artifact: future inside/outside policy must be able to consume
        // decided side facts, not an "unknown" region/plane relation. This is
        // Yap's exact-computation boundary applied to report state: undecided
        // predicates remain explicit blockers instead of being mislabeled as a
        // ready topological decision. See Yap, "Towards Exact Geometric
        // Computation," Computational Geometry 7.1-2 (1997).
        if !classification.all_proof_producing()
            || matches!(classification.relation, FaceRegionPlaneRelation::Unknown)
        {
            return Err(ExactReportValidationError::RegionClassificationNotProofProducing);
        }
    }
    Ok(())
}

/// Auditable result of an exact selected-region boolean pipeline.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanResult {
    /// Declared production path for this result.
    pub kind: ExactBooleanResultKind,
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

/// Declared production path for an exact boolean result.
///
/// Result kind is explicit so validation does not infer semantic intent from
/// empty vectors. That distinction matters for exact computing: selected-region
/// assembly, certified shortcuts, and boundary-policy projections are different
/// application contracts even when they all produce an empty mesh. The design
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), by retaining the policy boundary that produced the
/// topology.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanResultKind {
    /// The result came from split-region classification and selected assembly.
    SelectedRegions {
        /// Requested split-region retention rule.
        selection: ExactRegionSelection,
    },
    /// The result came from a certified named-boolean shortcut.
    CertifiedShortcut {
        /// Specific shortcut proof boundary that produced the result.
        shortcut: ExactBooleanShortcutKind,
    },
    /// The result came from an explicit lower-dimensional boundary projection
    /// policy.
    BoundaryPolicyShortcut {
        /// Named operation whose lower-dimensional contacts were projected into
        /// a triangle-mesh output.
        operation: ExactBooleanOperation,
    },
}

/// Executable certified shortcut used to produce a named boolean result.
///
/// This enum is intentionally narrower than [`ExactBooleanSupport`]: it names
/// only cases that have already materialized output topology. Retaining the
/// exact shortcut reason on [`ExactBooleanResultKind`] gives downstream audit
/// code the same explicit proof boundary advocated by Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), instead of
/// reducing all shortcut outputs to an undifferentiated mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanShortcutKind {
    /// Exact empty-operand semantics.
    EmptyOperand,
    /// Certified disjoint mesh AABBs.
    BoundsDisjoint,
    /// Exact coordinate and topology identity.
    Identical,
    /// Exact coordinate equality and matching triangle sets, modulo indexing
    /// and face orientation.
    SameSurface,
    /// Exact coplanar convex surface coverage, modulo triangulation.
    CoplanarConvexSurfaceEquivalence,
    /// Certified positive-area intersection of convex coplanar surface meshes.
    CoplanarConvexSurfaceIntersection,
    /// Certified simple-loop union of convex coplanar surface meshes.
    CoplanarConvexSurfaceArrangementUnion,
    /// Certified simple-loop difference of convex coplanar surface meshes.
    CoplanarConvexSurfaceArrangementDifference,
    /// Certified multi-component difference of convex coplanar surface meshes.
    CoplanarConvexSurfaceMultiDifference,
    /// Exact coplanar convex surface containment, modulo triangulation.
    CoplanarConvexSurfaceContainment,
    /// Certified one-hole coplanar convex surface difference.
    CoplanarConvexSurfaceHoledDifference,
    /// Certified graph absence for open surfaces.
    OpenSurfaceDisjoint,
    /// Certified closed-convex containment.
    ConvexContainment,
    /// Certified closed-convex separation.
    ConvexSeparated,
    /// Certified single-triangle coplanar surface containment.
    CoplanarSurfaceContainment,
    /// Certified coplanar single-triangle intersection output.
    CoplanarSurfaceIntersection,
    /// Certified convex coplanar single-triangle union output.
    CoplanarSurfaceConvexUnion,
    /// Certified simple planar-arrangement coplanar single-triangle union.
    CoplanarSurfaceArrangementUnion,
    /// Certified one-corner coplanar single-triangle difference output.
    CoplanarSurfaceCornerDifference,
    /// Certified simple planar-arrangement coplanar single-triangle difference.
    CoplanarSurfaceArrangementDifference,
    /// Certified one-hole coplanar single-triangle difference.
    CoplanarSurfaceHoledDifference,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanResult {
    /// Validate the retained artifacts in this selected-region or shortcut
    /// boolean result.
    ///
    /// Shortcut booleans can return a certified mesh only when no split-region
    /// artifacts are retained. Selected-region results audit every
    /// region/plane classification,
    /// triangulation, assembly invariant, and the materialized output mesh,
    /// then checks that assembly vertices and triangles still match the mesh.
    /// A selected-region result must retain nonempty region classifications
    /// and triangulations because those are the checked handoff facts that
    /// justify the assembly; otherwise a caller could relabel an empty
    /// shortcut-like object as a selected-region boolean.
    /// Every retained triangulation must also have at least one matching
    /// retained region/plane classification for its source side and face, so
    /// the mesh handoff cannot contain triangulated topology disconnected from
    /// the exact side facts prepared for winding policy. Conversely, every
    /// retained region/plane classification must belong to a triangulated
    /// source region so stale or relabeled side facts cannot be interpreted as
    /// part of the output proof. Selected-region reports also require those
    /// side facts to be proof-producing and decided, rather than carrying an
    /// unknown relation beside a materialized output. Output assembly triangles
    /// must likewise point back to retained triangulated source regions,
    /// preventing post-hoc provenance relabeling after materialization, and
    /// their vertex sources must be members of the retained triangulation
    /// boundary for that source region. The retained assembly must also avoid
    /// dead vertices so the topology handoff is the exact set consumed by mesh
    /// materialization. That keeps the final boolean handoff aligned with Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997): downstream topology receives a coherent chain of exact evidence
    /// rather than an opaque output mesh.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if !matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && (!self.region_classifications.is_empty()
                || !self.triangulations.is_empty()
                || !self.assembly.vertices.is_empty()
                || !self.assembly.triangles.is_empty())
        {
            return Err(ExactReportValidationError::ShortcutResultHasAssemblyArtifacts);
        }
        if !matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::ShortcutResultHasUnknownGraph);
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::SelectedRegionResultHasUnknownGraph);
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && (self.region_classifications.is_empty() || self.triangulations.is_empty())
        {
            return Err(ExactReportValidationError::MissingRegionFacts);
        }

        for classification in &self.region_classifications {
            classification
                .validate()
                .map_err(ExactReportValidationError::InvalidRegionClassification)?;
            if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
                && (!classification.all_proof_producing()
                    || matches!(classification.relation, FaceRegionPlaneRelation::Unknown))
            {
                return Err(ExactReportValidationError::RegionClassificationNotProofProducing);
            }
        }
        for triangulation in &self.triangulations {
            triangulation
                .validate()
                .map_err(|_| ExactReportValidationError::InvalidTriangulation)?;
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && self.triangulations.iter().any(|triangulation| {
                !self.region_classifications.iter().any(|classification| {
                    classification.region_side == triangulation.side
                        && classification.region_face == triangulation.face
                })
            })
        {
            return Err(ExactReportValidationError::UnclassifiedRegionTriangulation);
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && self.region_classifications.iter().any(|classification| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                })
            })
        {
            return Err(ExactReportValidationError::OrphanedRegionClassification);
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. })
            && self.assembly.triangles.iter().any(|triangle| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                })
            })
        {
            return Err(ExactReportValidationError::UntriangulatedAssemblyRegion);
        }
        if matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. }) {
            for triangle in &self.assembly.triangles {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                }) else {
                    return Err(ExactReportValidationError::UntriangulatedAssemblyRegion);
                };
                for &vertex in &triangle.vertices {
                    let Some(assembly_vertex) = self.assembly.vertices.get(vertex) else {
                        return Err(ExactReportValidationError::InvalidAssembly);
                    };
                    if !triangulation
                        .boundary
                        .iter()
                        .any(|source| source == &assembly_vertex.source)
                    {
                        return Err(ExactReportValidationError::AssemblyVertexOutsideTriangulation);
                    }
                }
            }
            if self
                .assembly
                .vertices
                .iter()
                .enumerate()
                .any(|(vertex, _)| {
                    !self
                        .assembly
                        .triangles
                        .iter()
                        .any(|triangle| triangle.vertices.contains(&vertex))
                })
            {
                return Err(ExactReportValidationError::UnreferencedAssemblyVertex);
            }
        }
        self.assembly
            .validate()
            .map_err(|_| ExactReportValidationError::InvalidAssembly)?;
        self.mesh
            .validate_retained_state()
            .map_err(|_| ExactReportValidationError::InvalidOutputMesh)?;

        let ExactBooleanResultKind::SelectedRegions { selection } = self.kind else {
            return Ok(());
        };

        if self
            .assembly
            .triangles
            .iter()
            .any(|triangle| !selection_keeps(selection, triangle.source_side))
        {
            return Err(ExactReportValidationError::SelectedRegionAssemblyViolatesSelection);
        }

        if self.assembly.vertices.len() != self.mesh.vertices().len()
            || self.assembly.triangles.len() != self.mesh.triangles().len()
        {
            return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
        }
        for (assembly_vertex, mesh_vertex) in
            self.assembly.vertices.iter().zip(self.mesh.vertices())
        {
            let mesh_point = mesh_vertex.to_hyperlimit_point();
            if !points_equal(&assembly_vertex.point, &mesh_point) {
                return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
            }
        }
        for (assembly_triangle, mesh_triangle) in
            self.assembly.triangles.iter().zip(self.mesh.triangles())
        {
            if assembly_triangle.vertices != mesh_triangle.0 {
                return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
            }
        }
        Ok(())
    }
}

#[cfg(feature = "exact-triangulation")]
fn points_equal(left: &Point3, right: &Point3) -> bool {
    matches!(
        compare_reals(&left.x, &right.x).value(),
        Some(Ordering::Equal)
    ) && matches!(
        compare_reals(&left.y, &right.y).value(),
        Some(Ordering::Equal)
    ) && matches!(
        compare_reals(&left.z, &right.z).value(),
        Some(Ordering::Equal)
    )
}

#[cfg(feature = "exact-triangulation")]
fn selection_keeps(selection: ExactRegionSelection, side: MeshSide) -> bool {
    matches!(
        (selection, side),
        (ExactRegionSelection::KeepAll, _)
            | (ExactRegionSelection::KeepLeft, MeshSide::Left)
            | (ExactRegionSelection::KeepRight, MeshSide::Right)
    )
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
    /// A named operation was answered by exact coordinate equality and matching
    /// triangle vertex sets, ignoring per-face orientation.
    CertifiedSameSurface,
    /// A named operation was answered by exact coplanar convex surface
    /// equivalence, ignoring triangulation.
    CertifiedCoplanarConvexSurfaceEquivalence,
    /// Intersection was materialized between convex coplanar surface meshes.
    CertifiedCoplanarConvexSurfaceIntersection,
    /// Union was materialized by a simple-loop arrangement between convex
    /// coplanar surface meshes.
    CertifiedCoplanarConvexSurfaceArrangementUnion,
    /// Difference was materialized by a simple-loop arrangement between
    /// convex coplanar surface meshes.
    CertifiedCoplanarConvexSurfaceArrangementDifference,
    /// Difference was materialized as multiple exact simple-loop components
    /// between convex coplanar surface meshes.
    CertifiedCoplanarConvexSurfaceMultiDifference,
    /// A named operation was answered by exact coplanar convex surface
    /// containment, ignoring triangulation.
    CertifiedCoplanarConvexSurfaceContainment,
    /// Difference was materialized as a one-hole arrangement between
    /// contained convex coplanar surface meshes.
    CertifiedCoplanarConvexSurfaceHoledDifference,
    /// A named operation was answered by exact no-intersection facts for open
    /// surface meshes.
    CertifiedOpenSurfaceDisjoint,
    /// A named operation was answered by certified closed-convex containment.
    CertifiedConvexContainment,
    /// A named operation was answered by certified single-triangle coplanar
    /// surface containment.
    CertifiedCoplanarSurfaceContainment,
    /// Intersection was materialized by exact clipping of two coplanar
    /// single-triangle surfaces.
    CertifiedCoplanarSurfaceIntersection,
    /// Union was materialized as a certified convex polygon for two coplanar
    /// single-triangle surfaces.
    CertifiedCoplanarSurfaceConvexUnion,
    /// Union was materialized by the simple single-loop planar arrangement for
    /// two coplanar single-triangle surfaces.
    CertifiedCoplanarSurfaceArrangementUnion,
    /// Difference was materialized as a certified one-corner cut from a
    /// coplanar single-triangle surface.
    CertifiedCoplanarSurfaceCornerDifference,
    /// Difference was materialized by the simple single-loop planar arrangement
    /// for two coplanar single-triangle surfaces.
    CertifiedCoplanarSurfaceArrangementDifference,
    /// Difference was materialized as a one-hole planar arrangement for
    /// contained coplanar single-triangle surfaces.
    CertifiedCoplanarSurfaceHoledDifference,
    /// A named operation was answered by a certified no-intersection convex
    /// separated relation that was not caught by mesh-level AABBs.
    CertifiedConvexSeparated,
    /// The retained graph contains only certified coplanar touching events.
    /// A caller must choose a boundary/shared-feature policy before this can
    /// become named boolean output.
    RequiresBoundaryPolicy,
    /// Coplanar positive-area overlap is certified, but the requested named
    /// output needs planar arrangement materialization.
    RequiresPlanarArrangement,
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
/// boundary. Shortcut variants are executable by `boolean_exact`. For
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
    /// Structured explanation for named operations that are certified enough
    /// to inspect but not yet executable by the selected policy.
    pub blocker: Option<ExactBooleanBlocker>,
    /// Checked coplanar-overlap readiness retained when preflight stops at a
    /// planar arrangement boundary.
    ///
    /// This deliberately keeps the exact graph handoff visible at the public
    /// preflight boundary. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), treats unresolved topology as
    /// structured program state; the positive-area coplanar graph evidence
    /// must not be flattened into a generic "unsupported" boolean.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanPreflight {
    /// Validate support, blocker, and retained artifact consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        // Preflight is the public contract between exact graph construction and
        // later boolean policy. Yap, "Towards Exact Geometric Computation,"
        // Computational Geometry 7.1-2 (1997), requires this boundary to
        // expose exact state rather than hide contradictions behind a boolean
        // success/failure bit.
        if (self.retained_face_pairs == 0 && self.retained_events != 0)
            || (self.retained_face_pairs != 0 && self.retained_events == 0)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        match self.support {
            ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedConvexSeparated => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::CertifiedReportHasBlocker);
                }
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresBoundaryPolicy => {
                if operation_is_selected_region(self.operation) || self.graph_had_unknowns {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresPlanarArrangement => {
                if operation_is_selected_region(self.operation)
                    || matches!(self.operation, ExactBooleanOperation::Intersection)
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(
                    readiness,
                    self.blocker.as_ref().unwrap(),
                )?;
                if !readiness.needs_planar_cells()
                    || self.blocker.as_ref().unwrap().coplanar_touching_pairs != 0
                {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCertifiedWinding => {
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(self.blocker.as_ref(), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::UnresolvedGraph => {
                if !self.graph_had_unknowns
                    && !self
                        .blocker
                        .as_ref()
                        .is_some_and(blocker_has_refinement_evidence)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::SelectedRegionPolicy => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if !operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.retained_face_pairs == 0 {
                    no_region_facts(self.region_count, &self.region_classifications)
                } else {
                    checked_region_facts(self.region_count, &self.region_classifications)
                }
            }
        }
    }
}

/// Missing exact policy or refinement that blocks named boolean output.
///
/// This is a report object, not an error. Yap's exact-computation model treats
/// unresolved application-layer topology as first-class state: a caller should
/// be able to distinguish "needs exact winding" from "needs a boundary output
/// policy" or "needs predicate refinement" without interpreting prose
/// diagnostics.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBooleanBlocker {
    /// Missing policy or refinement class.
    pub kind: ExactBooleanBlockerKind,
    /// Number of retained non-coplanar candidate face pairs.
    pub candidate_pairs: usize,
    /// Number of retained coplanar positive-overlap face pairs.
    pub coplanar_overlapping_pairs: usize,
    /// Number of retained coplanar touching face pairs.
    pub coplanar_touching_pairs: usize,
    /// Number of retained unknown face pairs.
    pub unknown_pairs: usize,
    /// Number of retained segment/plane events whose endpoint predicates
    /// certified a crossing but whose exact construction failed.
    pub construction_failed_events: usize,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanBlocker {
    /// Validate that this blocker kind is justified by retained graph relation
    /// counts.
    ///
    /// The counts are exact graph evidence, not decoration. A blocker that
    /// says "needs planar arrangement" while retaining unknown or non-coplanar
    /// candidate pairs would collapse distinct exact-computation states into
    /// one policy bucket. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), requires those unresolved
    /// states to stay explicit.
    pub fn validate_for_kind(
        &self,
        expected: ExactBooleanBlockerKind,
    ) -> Result<(), ExactReportValidationError> {
        if self.kind != expected {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        let valid = match expected {
            ExactBooleanBlockerKind::NeedsRefinement => {
                self.unknown_pairs > 0 || self.construction_failed_events > 0
            }
            ExactBooleanBlockerKind::NeedsBoundaryPolicy => {
                self.coplanar_touching_pairs > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.candidate_pairs == 0
                    && self.coplanar_overlapping_pairs == 0
            }
            ExactBooleanBlockerKind::NeedsPlanarArrangement => {
                self.coplanar_overlapping_pairs > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.candidate_pairs == 0
                    && self.coplanar_touching_pairs == 0
            }
            ExactBooleanBlockerKind::NeedsWinding => {
                self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.coplanar_overlapping_pairs == 0
                    && self.coplanar_touching_pairs == 0
            }
        };
        if valid {
            Ok(())
        } else {
            Err(ExactReportValidationError::InvalidBlockerCounts)
        }
    }
}

/// Exact boolean preflight blocker kind.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanBlockerKind {
    /// Predicate or equality refinement is required before policy can run.
    NeedsRefinement,
    /// A lower-dimensional shared-boundary output policy is required.
    NeedsBoundaryPolicy,
    /// A planar arrangement output model is required for coplanar surfaces.
    NeedsPlanarArrangement,
    /// Full winding/inside-outside classification is required.
    NeedsWinding,
}

/// Certification status for exact refinement preflight.
///
/// Refinement is the stage before application-level topology policy: exact
/// graph extraction retained an unknown predicate outcome or a construction
/// whose endpoint predicates certified an event but whose exact point/parameter
/// could not be built. Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), treats this as a different state
/// from winding or planar-arrangement policy, so it has a separate report.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactRefinementStatus {
    /// The graph contains no retained unknowns or construction failures.
    NotRequired,
    /// The graph contains retained evidence that must be refined before policy.
    Required,
}

/// Auditable report for unresolved exact graph refinement.
///
/// This report is intentionally narrower than boolean preflight. It answers
/// only whether exact graph construction is blocked by unknown predicates or
/// failed exact constructions, retaining the graph counts that justify the
/// answer. Later boundary, planar-arrangement, or winding reports should only
/// run as policy once this report is not required.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactRefinementReport {
    /// Named operation whose graph was inspected.
    pub operation: ExactBooleanOperation,
    /// Coarse refinement status.
    pub status: ExactRefinementStatus,
    /// Whether graph extraction retained unknown predicate outcomes.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Refinement blocker counts, present only when refinement is required.
    pub blocker: Option<ExactBooleanBlocker>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactRefinementReport {
    /// Return whether exact predicate/construction refinement is required.
    pub const fn is_required(&self) -> bool {
        matches!(self.status, ExactRefinementStatus::Required)
    }

    /// Validate status, retained counts, and refinement blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if (self.retained_face_pairs == 0 && self.retained_events != 0)
            || (self.retained_face_pairs != 0 && self.retained_events == 0)
        {
            return Err(ExactReportValidationError::InvalidBlockerCounts);
        }
        match self.status {
            ExactRefinementStatus::Required => {
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                let blocker = self.blocker.as_ref().unwrap();
                blocker.validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
                validate_blocker_count_bounds(
                    blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.graph_had_unknowns != (blocker.unknown_pairs > 0) {
                    return Err(ExactReportValidationError::InvalidBlockerCounts);
                }
            }
            ExactRefinementStatus::NotRequired => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::UnexpectedGraphEvents);
                }
                if self.graph_had_unknowns {
                    return Err(ExactReportValidationError::InvalidBlockerCounts);
                }
            }
        }
        Ok(())
    }
}

/// Certification status for same-surface named boolean shortcuts.
///
/// Same-surface detection is stronger than identical storage equality: it
/// allows vertex reindexing and face orientation changes when exact coordinate
/// equality proves a bijection and the remapped triangle vertex sets match.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactSameSurfaceStatus {
    /// The meshes have different vertex counts.
    VertexCountMismatch,
    /// The meshes have different triangle counts.
    TriangleCountMismatch,
    /// At least one required coordinate equality predicate was undecided.
    VertexMatchingUndecided,
    /// No exact vertex bijection exists.
    VertexCoordinateMismatch,
    /// A vertex bijection exists, but remapped triangle sets differ.
    TriangleSetMismatch,
    /// Exact vertex bijection and remapped triangle-set equality were certified.
    Certified,
}

/// Auditable same-surface certification report.
///
/// This is the report form of the same-surface boolean shortcut. It retains
/// the exact vertex permutation, remapped triangle sets, and scalar equality
/// predicate certificates used to prove coordinate equality. The design
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): shortcut topology decisions expose their certified
/// predicate trail rather than collapsing directly to `bool`.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactSameSurfaceReport {
    /// Coarse same-surface certification status.
    pub status: ExactSameSurfaceStatus,
    /// Mapping from each left vertex index to the matched right vertex index.
    pub left_to_right: Vec<usize>,
    /// Mapping from each right vertex index to the matched left vertex index.
    pub right_to_left: Vec<usize>,
    /// Sorted left triangle vertex sets.
    pub left_triangles: Vec<[usize; 3]>,
    /// Sorted right triangle vertex sets remapped into left vertex indices.
    pub right_triangles: Vec<[usize; 3]>,
    /// Predicate certificates used by exact coordinate equality checks.
    pub predicates: Vec<PredicateUse>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactSameSurfaceReport {
    /// Return whether same-surface equivalence was certified.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactSameSurfaceStatus::Certified)
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate same-surface report invariants.
    ///
    /// Rejection statuses are still evidence states: count mismatches must not
    /// retain coordinate predicates, vertex-matching failures may keep only the
    /// partial left-to-right matches and predicate trail, and triangle-set
    /// mismatches must retain a valid full vertex permutation. This keeps a
    /// failed shortcut auditable under Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), instead of
    /// allowing callers to attach arbitrary topology artifacts to a rejection.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        match self.status {
            ExactSameSurfaceStatus::VertexCountMismatch
            | ExactSameSurfaceStatus::TriangleCountMismatch => {
                if !self.left_to_right.is_empty()
                    || !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || !self.predicates.is_empty()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactSameSurfaceStatus::VertexMatchingUndecided
            | ExactSameSurfaceStatus::VertexCoordinateMismatch => {
                if !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || self.predicates.is_empty()
                    || !is_partial_injective_mapping(&self.left_to_right)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if matches!(
                    self.status,
                    ExactSameSurfaceStatus::VertexCoordinateMismatch
                ) && !self.all_proof_producing()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactSameSurfaceStatus::TriangleSetMismatch => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles.is_empty()
                    || self.right_triangles.is_empty()
                    || self.left_triangles == self.right_triangles
                {
                    return Err(ExactReportValidationError::MismatchedTriangleSets);
                }
            }
            ExactSameSurfaceStatus::Certified => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles != self.right_triangles {
                    return Err(ExactReportValidationError::MismatchedTriangleSets);
                }
                if !self.all_proof_producing() {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_full_permutation(
    left_to_right: &[usize],
    right_to_left: &[usize],
) -> Result<(), ExactReportValidationError> {
    if left_to_right.len() != right_to_left.len() {
        return Err(ExactReportValidationError::InvalidPermutation);
    }
    for (left, &right) in left_to_right.iter().enumerate() {
        if right >= right_to_left.len() || right_to_left[right] != left {
            return Err(ExactReportValidationError::InvalidPermutation);
        }
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn is_partial_injective_mapping(mapping: &[usize]) -> bool {
    let mut seen = Vec::with_capacity(mapping.len());
    for &right in mapping {
        if seen.contains(&right) {
            return false;
        }
        seen.push(right);
    }
    true
}

/// Certification status for an open-surface disjoint shortcut.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactOpenSurfaceDisjointStatus {
    /// At least one input is not an open surface mesh under exact validation facts.
    NotOpenSurface,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Exact graph extraction retained intersections or contacts.
    GraphHasFacePairs,
    /// Both inputs are open surfaces and the exact graph has no retained pairs.
    Certified,
}

/// Auditable report for certified open-surface disjointness.
///
/// This report retains the mesh-shape precondition and exact graph relation
/// counts used by the open-surface named-boolean shortcut. Following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), a no-intersection shortcut is exposed as certified graph state, not
/// as a hidden primitive-float or AABB-only decision.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactOpenSurfaceDisjointReport {
    /// Coarse certification status.
    pub status: ExactOpenSurfaceDisjointStatus,
    /// Whether the left mesh satisfies the exact open-surface precondition.
    pub left_open_surface: bool,
    /// Whether the right mesh satisfies the exact open-surface precondition.
    pub right_open_surface: bool,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
}

#[cfg(feature = "exact-triangulation")]
impl ExactOpenSurfaceDisjointReport {
    /// Return whether open-surface disjointness was certified.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactOpenSurfaceDisjointStatus::Certified)
    }

    /// Validate status, graph counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // Graph unknowns are refinement state, not open-surface topology
        // policy. Keeping this partition explicit follows Yap, "Towards Exact
        // Geometric Computation," Computational Geometry 7.1-2 (1997): a
        // later policy stage must not consume an unresolved predicate as if it
        // were certified no-intersection or winding evidence.
        let expected_kind = if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
        {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsWinding
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Status is certified combinatorial state, not a label layered over
        // arbitrary counts. Yap, "Towards Exact Geometric Computation,"
        // Computational Geometry 7.1-2 (1997), keeps these states explicit so
        // refinement, topology policy, and certified shortcuts are not
        // accidentally conflated.
        match self.status {
            ExactOpenSurfaceDisjointStatus::NotOpenSurface => {
                if (self.left_open_surface && self.right_open_surface)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                    || blocker_has_any_evidence(&self.blocker)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::GraphUnknowns => {
                if !self.left_open_surface
                    || !self.right_open_surface
                    || !blocker_has_refinement_evidence(&self.blocker)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::GraphHasFacePairs => {
                if !self.left_open_surface || !self.right_open_surface {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::Certified => {
                if !self.left_open_surface || !self.right_open_surface {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        if self.is_certified() && (self.retained_face_pairs != 0 || self.retained_events != 0) {
            return Err(ExactReportValidationError::UnexpectedGraphEvents);
        }
        if self.status == ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
            && self.retained_face_pairs == 0
        {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        Ok(())
    }
}

/// Certification status for boundary-only graph shortcuts.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactBoundaryTouchingStatus {
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Retained graph pairs were not exclusively coplanar touching pairs.
    NotBoundaryOnly,
    /// The graph contains only certified coplanar touching pairs.
    Certified,
}

/// Auditable report for certified boundary-only contacts.
///
/// Boundary-only contacts require a caller-selected output policy because a
/// triangle mesh cannot encode the lower-dimensional intersection itself.
/// This report retains the exact graph counts that justify that policy gap,
/// keeping the application boundary explicit in Yap's exact-geometric-
/// computation sense.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBoundaryTouchingReport {
    /// Coarse boundary-touching certification status.
    pub status: ExactBoundaryTouchingStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBoundaryTouchingReport {
    /// Return whether the graph is certified boundary-only contact.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactBoundaryTouchingStatus::Certified)
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // A boundary-only policy is meaningful only after the graph is
        // resolved. Unknown graph events remain refinement blockers, preserving
        // Yap's exact-state separation between undecided predicates and
        // application-level topology policy.
        let expected_kind = if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns) {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Boundary-only contact is an application policy boundary. Keep its
        // evidence separated from graph refinement and non-boundary winding
        // cases as required by Yap, "Towards Exact Geometric Computation,"
        // Computational Geometry 7.1-2 (1997).
        match self.status {
            ExactBoundaryTouchingStatus::GraphUnknowns => {
                if !blocker_has_refinement_evidence(&self.blocker) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactBoundaryTouchingStatus::NotBoundaryOnly => {
                if self.retained_face_pairs != 0
                    && self
                        .blocker
                        .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)
                        .is_ok()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactBoundaryTouchingStatus::Certified => {}
        }
        if self.is_certified() && self.blocker.coplanar_touching_pairs == 0 {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        if self.is_certified() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
        }
        Ok(())
    }
}

/// Certification status for planar-arrangement blockers.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactPlanarArrangementStatus {
    /// Selected-region assembly already carries its own explicit region policy.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// The requested named operation is already handled by a narrower certified
    /// coplanar surface output path.
    AlreadyMaterialized,
    /// The exact graph does not consist solely of positive-area coplanar
    /// overlaps requiring planar arrangement output.
    NoPositiveOverlap,
    /// Certified positive-area coplanar overlap requires a planar arrangement
    /// output model before this named operation can be materialized.
    Required,
}

/// Auditable report for planar-arrangement work left at the exact boundary.
///
/// Coplanar positive-area overlaps are real topology, not numerical noise.
/// This report records when the exact graph proves that a named union or
/// difference needs planar arrangement materialization instead of a volumetric
/// winding rule. Narrow single-triangle outputs are reported separately as
/// already materialized. This follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): missing application
/// topology is explicit certified state rather than an approximate fallback.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactPlanarArrangementReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Coarse planar-arrangement certification status.
    pub status: ExactPlanarArrangementStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap readiness summary retained from the graph
    /// layer.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactPlanarArrangementReport {
    /// Return whether this operation is blocked on planar arrangement output.
    pub const fn is_required(&self) -> bool {
        matches!(self.status, ExactPlanarArrangementStatus::Required)
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // A graph-unknown arrangement report has not reached planar-cell
        // policy. It is still blocked on predicate/construction refinement, a
        // distinct exact-computation state in Yap's sense.
        let expected_kind = if matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns) {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsPlanarArrangement
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Planar-cell extraction is a distinct topological obligation. These
        // checks preserve the exact-state partition advocated by Yap, "Towards
        // Exact Geometric Computation," Computational Geometry 7.1-2 (1997):
        // selected-region calls, unresolved graphs, already materialized
        // shortcuts, and missing planar arrangements must not masquerade as
        // one another.
        match self.status {
            ExactPlanarArrangementStatus::NotNamedOperation => {
                if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                    || blocker_has_any_evidence(&self.blocker)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::GraphUnknowns => {
                if !blocker_has_refinement_evidence(&self.blocker) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::Required => {
                if matches!(
                    self.operation,
                    ExactBooleanOperation::SelectedRegions(_) | ExactBooleanOperation::Intersection
                ) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        if self.is_required() && self.blocker.coplanar_overlapping_pairs == 0 {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        match self.status {
            ExactPlanarArrangementStatus::Required => {
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                if !readiness.needs_planar_cells()
                    || self.blocker.coplanar_touching_pairs != 0
                    || readiness.graph_count
                        != self.blocker.coplanar_overlapping_pairs
                            + self.blocker.coplanar_touching_pairs
                {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
            }
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap => {
                if let Some(readiness) = &self.arrangement_readiness {
                    readiness
                        .validate()
                        .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                    validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                    if readiness.status == CoplanarArrangementReadinessStatus::NoCoplanarOverlap
                        && self.blocker.coplanar_overlapping_pairs
                            + self.blocker.coplanar_touching_pairs
                            != 0
                    {
                        return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                    }
                }
            }
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::GraphUnknowns => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
            }
        }
        if self.is_required() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
        }
        Ok(())
    }
}

/// Certification status for the remaining exact winding handoff.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactWindingReadinessStatus {
    /// Selected-region assembly already carries its own explicit region policy.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Retained graph pairs are boundary-only contacts and need boundary
    /// output policy rather than winding.
    BoundaryPolicyRequired,
    /// Retained graph pairs are positive-area coplanar overlaps and need a
    /// planar arrangement output model rather than volumetric winding.
    PlanarArrangementRequired,
    /// The positive-area coplanar overlap was already handled by a certified
    /// planar-arrangement shortcut, so no volumetric winding handoff is needed.
    PlanarArrangementAlreadyMaterialized,
    /// The graph contains no retained face pairs requiring winding.
    NoNontrivialOverlap,
    /// Split regions and opposite-plane classifications were checked and are
    /// ready for the future exact winding/inside-outside policy.
    Ready,
}

/// Auditable report for the nontrivial overlap winding handoff.
///
/// This report is the certified boundary immediately before full named
/// union/intersection/difference winding semantics. It retains exact graph
/// counts and checked split-region plane classifications, but deliberately
/// does not choose inside/outside output. Following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), the missing
/// topological policy remains explicit state instead of a hidden tolerance
/// decision.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactWindingReadinessReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Coarse readiness status.
    pub status: ExactWindingReadinessStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Number of checked split regions prepared for winding.
    pub region_count: usize,
    /// Certified region-vs-opposite-plane classifications.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Relation counts for the blocker represented by this report.
    pub blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap readiness retained when winding is blocked by
    /// planar-cell extraction rather than by volumetric inside/outside policy.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactWindingReadinessReport {
    /// Return whether the report reached the winding-ready handoff.
    pub const fn is_ready(&self) -> bool {
        matches!(self.status, ExactWindingReadinessStatus::Ready)
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.region_classifications
            .iter()
            .all(FaceRegionPlaneClassification::all_proof_producing)
    }

    /// Validate status, blocker, and checked-region artifact consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactWindingReadinessStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        match self.status {
            ExactWindingReadinessStatus::GraphUnknowns => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if !blocker_has_refinement_evidence(&self.blocker) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::BoundaryPolicyRequired => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::PlanarArrangementRequired => {
                if matches!(
                    self.operation,
                    ExactBooleanOperation::SelectedRegions(_) | ExactBooleanOperation::Intersection
                ) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                if !readiness.needs_planar_cells() || self.blocker.coplanar_touching_pairs != 0 {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if let Some(readiness) = &self.arrangement_readiness {
                    readiness
                        .validate()
                        .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                    validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::Ready => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::NotNamedOperation
            | ExactWindingReadinessStatus::NoNontrivialOverlap => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                match self.status {
                    ExactWindingReadinessStatus::NotNamedOperation
                        if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    ExactWindingReadinessStatus::NoNontrivialOverlap
                        if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                            || self.retained_face_pairs != 0 =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    _ => {}
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
        }
    }
}
