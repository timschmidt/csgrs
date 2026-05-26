//! Exact boolmesh-kernel port scaffolding.
//!
//! This module is the landing zone for the direct port of the legacy
//! `boolean03`/`boolean45` kernels.  The intent is deliberately conservative:
//! keep the boolmesh dataflow recognizable, but replace primitive-float
//! decisions with exact objects from `hyperreal`, `hyperlattice`, `hyperlimit`,
//! and later `hypertri`.
//!
//! The staged split follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): exact objects, predicate decisions,
//! and topology mutations are separate artifacts that must replay together.
//! The halfedge construction shape follows the legacy boolmesh kernels already
//! in this crate (`boolean03` discovery/classification, then `boolean45`
//! halfedge assembly).  The retained-fragment view is also compatible with the
//! polygonal boundary model of Weiler and Atherton, "Hidden Surface Removal
//! Using Polygon Area Sorting," *SIGGRAPH* (1977): intersections produce
//! ordered boundary fragments before faces are emitted.

#[cfg(feature = "exact-triangulation")]
use super::boolean::ExactBooleanOperation;
#[cfg(feature = "exact-triangulation")]
use super::mesh::{ExactMesh, Triangle};
#[cfg(feature = "exact-triangulation")]
use super::provenance::SourceProvenance;
#[cfg(feature = "exact-triangulation")]
use super::reports::ExactBooleanShortcutKind;
#[cfg(feature = "exact-triangulation")]
use super::scalar::ExactReal;
#[cfg(feature = "exact-triangulation")]
use super::validation::ValidationPolicy;
#[cfg(feature = "exact-triangulation")]
use super::{AabbIntersectionKind, MeshError};
#[cfg(feature = "exact-triangulation")]
use hyperlimit::PredicateOutcome;

/// Legacy boolmesh kernel stage represented by the exact port.
///
/// These names intentionally mirror the old modules instead of inventing a new
/// boolean vocabulary.  A blocker can therefore say exactly which part of the
/// boolmesh/paper pipeline has not yet been ported to exact predicates.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoolMeshKernelStage {
    /// Edge/triangle intersection discovery, legacy `boolean03::kernel12`.
    Kernel12,
    /// Winding/classification, legacy `boolean03::kernel03`.
    Kernel03,
    /// Bidirectional discovery/classification package, legacy `Boolean03`.
    Boolean03,
    /// Output sizing and face map construction, legacy `boolean45::size_output`.
    SizeOutput,
    /// Ordered edge-event tail/head pairing, legacy `boolean45::pair_up`.
    PairUp,
    /// Source-edge fragment emission, legacy partial/whole edge stages.
    SourceEdgeEmission,
    /// New face-pair fragment emission, legacy `append_new_edges`.
    FacePairEdgeEmission,
    /// Output face loop assembly, legacy `boolean45` face staging.
    FaceAssembly,
    /// Exact triangulation of assembled faces through `hypertri`.
    Triangulation,
    /// Exact cleanup/simplification after triangulation.
    Cleanup,
}

/// Structured blocker for the exact boolmesh-kernel port.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshPortBlocker {
    /// First unported or unresolved boolmesh stage.
    pub stage: ExactBoolMeshKernelStage,
    /// Retained broad-phase face-pair candidates that require the stage.
    pub candidate_face_pairs: usize,
    /// Whether the whole-mesh AABB relation itself was undecidable.
    pub mesh_bounds_unknown: bool,
}

/// Exact face-pair key matching the boolmesh `p1q2`/`p2q1` ownership shape.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshFacePair {
    /// Source face from the left operand.
    pub left_face: usize,
    /// Source face from the right operand.
    pub right_face: usize,
}

/// Operand side used by exact boolmesh event provenance.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoolMeshSide {
    /// Left boolean operand.
    Left,
    /// Right boolean operand.
    Right,
}

/// Exact source-owned vertex handle.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshSourceVertex {
    /// Operand side that owns the vertex.
    pub side: ExactBoolMeshSide,
    /// Vertex index in that operand.
    pub vertex: usize,
}

/// Exact point construction used by future `kernel12` events.
///
/// The legacy boolmesh kernels store intersection coordinates in `v12`/`v21`
/// as primitive `Vec3` values.  The exact port stores the reason a point exists
/// and lets predicates replay it.  Edge parameters are rational `Real` values
/// so event ordering can use symbolic comparison instead of `dot(edge, point)`
/// on rounded coordinates.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub enum ExactBoolMeshPointConstruction {
    /// Existing source vertex reused without construction.
    SourceVertex(ExactBoolMeshSourceVertex),
    /// Point on a source edge at an exact parameter in `[0, 1]`.
    EdgeParameter {
        /// Operand side that owns the edge.
        side: ExactBoolMeshSide,
        /// Source edge tail vertex.
        tail: usize,
        /// Source edge head vertex.
        head: usize,
        /// Exact edge parameter measured from `tail` toward `head`.
        parameter: ExactReal,
    },
    /// Placeholder for a segment/plane construction owned by exact `kernel12`.
    SegmentPlane {
        /// Edge operand side.
        edge_side: ExactBoolMeshSide,
        /// Source edge tail vertex.
        tail: usize,
        /// Source edge head vertex.
        head: usize,
        /// Opposite source face.
        face: usize,
        /// Exact edge parameter measured from `tail` toward `head`.
        parameter: ExactReal,
    },
}

/// Ordered event on one source edge.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshEdgeEvent {
    /// Source edge owner.
    pub side: ExactBoolMeshSide,
    /// Source edge tail vertex.
    pub tail: usize,
    /// Source edge head vertex.
    pub head: usize,
    /// Exact parameter used by the `boolean45::pair_up` port.
    pub parameter: ExactReal,
    /// Collision/event id, preserving the boolmesh tie-break role of `cid`.
    pub collision: usize,
    /// Whether this event contributes a tail halfedge endpoint.
    pub is_tail: bool,
    /// Exact point construction retained for source replay.
    pub point: ExactBoolMeshPointConstruction,
}

/// Exact `Boolean03`-shaped package.
///
/// This mirrors the legacy `Boolean03` fields so the port can move one stage at
/// a time.  Empty vectors are meaningful for certified disjoint operands: they
/// prove the direct boolmesh workspace crossed discovery without invoking the
/// primitive-float adapter.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshBoolean03 {
    /// Left-edge/right-face ownership pairs, legacy `p1q2`.
    pub p1q2: Vec<ExactBoolMeshFacePair>,
    /// Right-edge/left-face ownership pairs, legacy `p2q1`.
    pub p2q1: Vec<ExactBoolMeshFacePair>,
    /// Signed event multiplicity along left edges, legacy `x12`.
    pub x12: Vec<i32>,
    /// Signed event multiplicity along right edges, legacy `x21`.
    pub x21: Vec<i32>,
    /// Left vertex winding/classification counters, legacy `w03`.
    pub w03: Vec<i32>,
    /// Right vertex winding/classification counters, legacy `w30`.
    pub w30: Vec<i32>,
}

/// Exact `boolean45`-shaped output staging metadata.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshBoolean45Stage {
    /// Per-output-face starting halfedge offsets, legacy `ih_per_f`.
    pub face_halfedge_offsets: Vec<usize>,
    /// Source-face to output-face map, legacy `face_pq2r`.
    pub source_face_to_output_face: Vec<Option<usize>>,
    /// Number of vertices copied from the left operand.
    pub vertices_from_left: usize,
    /// Number of vertices copied from the right operand.
    pub vertices_from_right: usize,
    /// Number of exact intersection vertices inserted by the port.
    pub inserted_intersection_vertices: usize,
}

/// Exact boolmesh workspace for one pair of operands.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshWorkspace {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Number of left source vertices.
    pub left_vertices: usize,
    /// Number of left source faces.
    pub left_faces: usize,
    /// Number of right source vertices.
    pub right_vertices: usize,
    /// Number of right source faces.
    pub right_faces: usize,
    /// Whole-mesh exact bounds relation, when both meshes are nonempty.
    pub mesh_bounds_relation: Option<PredicateOutcome<AabbIntersectionKind>>,
    /// Broad-phase face pairs that must continue to exact `kernel12`.
    pub candidate_face_pairs: Vec<ExactBoolMeshFacePair>,
    /// Current exact `Boolean03` package.
    pub boolean03: ExactBoolMeshBoolean03,
    /// Current exact `boolean45` staging package, if output assembly has run.
    pub boolean45: Option<ExactBoolMeshBoolean45Stage>,
    /// First missing boolmesh stage for this workspace.
    pub blocker: Option<ExactBoolMeshPortBlocker>,
}

impl ExactBoolMeshWorkspace {
    /// Build the first exact boolmesh workspace from source meshes.
    ///
    /// This is the exact counterpart to entering legacy `boolean03`: it records
    /// source sizes and exact broad-phase scheduling before any topology is
    /// emitted.  Certified disjoint mesh bounds produce an empty `Boolean03`
    /// package.  Any retained candidate face pair stops at `kernel12`, because
    /// the exact edge/triangle discovery loop is the next port chunk.
    pub fn from_sources(
        left: &ExactMesh,
        right: &ExactMesh,
        operation: ExactBooleanOperation,
    ) -> Self {
        let mesh_bounds_relation = match (&left.bounds().mesh, &right.bounds().mesh) {
            (Some(left_bounds), Some(right_bounds)) => {
                Some(left_bounds.classify_intersection(right_bounds))
            }
            _ => None,
        };
        let candidate_face_pairs = left
            .bounds()
            .candidate_face_pairs(right.bounds())
            .into_iter()
            .map(|[left_face, right_face]| ExactBoolMeshFacePair {
                left_face,
                right_face,
            })
            .collect::<Vec<_>>();
        let mesh_bounds_unknown =
            matches!(mesh_bounds_relation, Some(PredicateOutcome::Unknown { .. }));
        let blocker = if candidate_face_pairs.is_empty() && !mesh_bounds_unknown {
            None
        } else {
            Some(ExactBoolMeshPortBlocker {
                stage: ExactBoolMeshKernelStage::Kernel12,
                candidate_face_pairs: candidate_face_pairs.len(),
                mesh_bounds_unknown,
            })
        };
        Self {
            operation,
            left_vertices: left.vertices().len(),
            left_faces: left.triangles().len(),
            right_vertices: right.vertices().len(),
            right_faces: right.triangles().len(),
            mesh_bounds_relation,
            candidate_face_pairs,
            boolean03: ExactBoolMeshBoolean03 {
                p1q2: Vec::new(),
                p2q1: Vec::new(),
                x12: Vec::new(),
                x21: Vec::new(),
                w03: vec![0; left.vertices().len()],
                w30: vec![0; right.vertices().len()],
            },
            boolean45: None,
            blocker,
        }
    }

    /// Return whether this workspace crossed discovery as certified disjoint.
    pub fn is_certified_bounds_disjoint(&self) -> bool {
        self.blocker.is_none()
            && self.candidate_face_pairs.is_empty()
            && self.boolean03.p1q2.is_empty()
            && self.boolean03.p2q1.is_empty()
            && matches!(
                self.mesh_bounds_relation,
                Some(PredicateOutcome::Decided {
                    value: AabbIntersectionKind::Disjoint,
                    ..
                })
            )
    }

    /// Validate the workspace locally.
    pub fn validate(&self) -> Result<(), ExactBoolMeshValidationError> {
        if self.boolean03.w03.len() != self.left_vertices {
            return Err(ExactBoolMeshValidationError::LeftWindingCountMismatch);
        }
        if self.boolean03.w30.len() != self.right_vertices {
            return Err(ExactBoolMeshValidationError::RightWindingCountMismatch);
        }
        for pair in &self.candidate_face_pairs {
            if pair.left_face >= self.left_faces || pair.right_face >= self.right_faces {
                return Err(ExactBoolMeshValidationError::FacePairOutOfBounds);
            }
        }
        if let Some(blocker) = &self.blocker {
            if blocker.candidate_face_pairs != self.candidate_face_pairs.len() {
                return Err(ExactBoolMeshValidationError::BlockerCountMismatch);
            }
        }
        if self.is_certified_bounds_disjoint() {
            Ok(())
        } else if self.blocker.is_some() {
            Ok(())
        } else {
            Err(ExactBoolMeshValidationError::MissingBlocker)
        }
    }

    /// Replay this workspace from the supplied source meshes.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactBoolMeshValidationError> {
        self.validate()?;
        let replay = Self::from_sources(left, right, self.operation);
        if self == &replay {
            Ok(())
        } else {
            Err(ExactBoolMeshValidationError::SourceReplayMismatch)
        }
    }
}

/// Result of the currently executable exact boolmesh port slice.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshExecution {
    /// Workspace consumed by the execution.
    pub workspace: ExactBoolMeshWorkspace,
    /// Shortcut semantics produced by this boolmesh-shaped port slice.
    pub shortcut: ExactBooleanShortcutKind,
    /// Materialized exact output mesh.
    pub mesh: ExactMesh,
}

impl ExactBoolMeshExecution {
    /// Validate the execution and replay its workspace from the sources.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactBoolMeshValidationError> {
        self.workspace.validate_against_sources(left, right)?;
        self.mesh
            .validate_retained_state()
            .map_err(|_| ExactBoolMeshValidationError::InvalidOutputMesh)?;
        if self.workspace.is_certified_bounds_disjoint()
            && self.shortcut == ExactBooleanShortcutKind::BoundsDisjoint
        {
            Ok(())
        } else {
            Err(ExactBoolMeshValidationError::ShortcutMismatch)
        }
    }
}

/// Validation failure for exact boolmesh-port artifacts.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoolMeshValidationError {
    /// Left winding/classification vector does not match source vertices.
    LeftWindingCountMismatch,
    /// Right winding/classification vector does not match source vertices.
    RightWindingCountMismatch,
    /// A retained face-pair candidate names a missing source face.
    FacePairOutOfBounds,
    /// Blocker candidate counts do not match retained candidates.
    BlockerCountMismatch,
    /// A non-disjoint workspace had no named boolmesh-stage blocker.
    MissingBlocker,
    /// Replaying from source meshes did not reproduce the workspace.
    SourceReplayMismatch,
    /// The materialized output mesh failed retained-state validation.
    InvalidOutputMesh,
    /// Execution shortcut does not match the workspace state.
    ShortcutMismatch,
    /// The executable disjoint slice was requested for a non-disjoint pair.
    RequiresKernel12,
}

/// Build the exact boolmesh workspace for one operation.
#[cfg(feature = "exact-triangulation")]
pub fn exact_boolmesh_workspace(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> ExactBoolMeshWorkspace {
    ExactBoolMeshWorkspace::from_sources(left, right, operation)
}

/// Execute the currently ported exact boolmesh bounds-disjoint slice.
///
/// This is intentionally small but not a report-only layer: it materializes the
/// same no-contact outputs that the legacy boolmesh pipeline reaches with
/// empty `p1q2`/`p2q1` discovery.  Non-disjoint operands return
/// [`ExactBoolMeshValidationError::RequiresKernel12`], naming the next direct
/// port stage instead of routing through bounded planar certificates.
#[cfg(feature = "exact-triangulation")]
pub fn execute_exact_boolmesh_bounds_disjoint(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBoolMeshExecution, ExactBoolMeshValidationError> {
    let workspace = ExactBoolMeshWorkspace::from_sources(left, right, operation);
    workspace.validate()?;
    if !workspace.is_certified_bounds_disjoint() {
        return Err(ExactBoolMeshValidationError::RequiresKernel12);
    }
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)
            .map_err(|_| ExactBoolMeshValidationError::InvalidOutputMesh)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("exact boolmesh empty disjoint intersection", validation)
                .map_err(|_| ExactBoolMeshValidationError::InvalidOutputMesh)?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            SourceProvenance::exact("exact boolmesh disjoint left difference"),
            validation,
        )
        .map_err(|_| ExactBoolMeshValidationError::InvalidOutputMesh)?,
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactBoolMeshValidationError::RequiresKernel12);
        }
    };
    let execution = ExactBoolMeshExecution {
        workspace,
        shortcut: ExactBooleanShortcutKind::BoundsDisjoint,
        mesh,
    };
    execution.validate_against_sources(left, right)?;
    Ok(execution)
}

#[cfg(feature = "exact-triangulation")]
fn empty_mesh(label: &'static str, validation: ValidationPolicy) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact(label),
        validation,
    )
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
        Triangle([
            triangle.0[0] + right_offset,
            triangle.0[1] + right_offset,
            triangle.0[2] + right_offset,
        ])
    }));
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact boolmesh disjoint union"),
        validation,
    )
}
