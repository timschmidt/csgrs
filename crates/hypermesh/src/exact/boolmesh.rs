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
mod boolean45;
#[cfg(feature = "exact-triangulation")]
mod kernel12;

#[cfg(feature = "exact-triangulation")]
use super::boolean::ExactBooleanOperation;
#[cfg(feature = "exact-triangulation")]
use super::construction::{
    SegmentPlaneConstructionFailure, SegmentPlaneParameterRatio, SegmentPlaneRelation,
};
#[cfg(feature = "exact-triangulation")]
use super::graph::{IntersectionEvent, MeshSide, build_intersection_graph};
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
use boolean45::pair_source_edge_events;
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{PlaneSide, Point3, PredicateOutcome};
#[cfg(feature = "exact-triangulation")]
use kernel12::lower_kernel12_events;

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

/// Exact edge/face key used by the boolmesh `kernel12` port.
///
/// Legacy boolmesh names these tables `p1q2` and `p2q1`: a directed source
/// edge from one operand is paired with a source face from the other operand.
/// This exact representation keeps that ownership explicit instead of
/// collapsing the event to a face-pair id, because the later `boolean45`
/// pairing stage must sort and split along the source edge that produced the
/// construction.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBoolMeshEdgeFacePair {
    /// Source face pair that retained this edge/face contact.
    pub face_pair: ExactBoolMeshFacePair,
    /// Operand side owning the source edge.
    pub edge_side: ExactBoolMeshSide,
    /// Directed source edge endpoints in `edge_side` vertex index space.
    pub edge: [usize; 2],
    /// Operand side owning the opposite source face.
    pub face_side: ExactBoolMeshSide,
    /// Source face index in `face_side` face index space.
    pub face: usize,
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

/// Paired source-edge fragment produced by exact `boolean45::pair_up`.
///
/// The legacy boolmesh stage sorts [`EdgePt`] values along a source edge,
/// partitions tail/head events, then creates partial halfedges by zipping the
/// sorted halves.  This exact record keeps the same pairing decision but
/// stores source event provenance instead of output vertex ids, because final
/// vertex allocation is still owned by the later exact `boolean45` slices.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshPairedEdgeFragment {
    /// Source edge owner.
    pub side: ExactBoolMeshSide,
    /// Source edge tail vertex.
    pub tail: usize,
    /// Source edge head vertex.
    pub head: usize,
    /// Event that contributes the fragment tail endpoint.
    pub tail_event: ExactBoolMeshEdgeEvent,
    /// Event that contributes the fragment head endpoint.
    pub head_event: ExactBoolMeshEdgeEvent,
}

/// Ordered exact event run on one source edge.
///
/// Runs are the exact equivalent of the `pt_old` buckets consumed by
/// `boolean45::append_partial_edges`.  Events are sorted by exact edge
/// parameter, with collision id as the boolmesh tie-break.  Unpaired events are
/// retained explicitly because endpoint retention from `kernel03` has not yet
/// been ported into the exact path.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshSourceEdgeRun {
    /// Source edge owner.
    pub side: ExactBoolMeshSide,
    /// Source edge tail vertex.
    pub tail: usize,
    /// Source edge head vertex.
    pub head: usize,
    /// Ordered events on this directed source edge.
    pub events: Vec<ExactBoolMeshEdgeEvent>,
    /// Zipped tail/head fragments when both sides are present.
    pub fragments: Vec<ExactBoolMeshPairedEdgeFragment>,
    /// Number of ordered events that could not yet be paired.
    pub unpaired_events: usize,
}

/// Exact `boolean45::pair_up` staging over lowered source-edge events.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExactBoolMeshPairUpStage {
    /// Ordered source-edge runs.
    pub source_edge_runs: Vec<ExactBoolMeshSourceEdgeRun>,
    /// Exact parameter comparisons that were not decidable.
    pub unknown_orderings: usize,
    /// Runs that still have unpaired events before endpoint retention is
    /// available.
    pub unpaired_event_runs: usize,
}

/// Retained exact event from the direct boolmesh `kernel12` port.
///
/// The legacy `boolean03::kernel12` implementation combines shadow tests and
/// `f64` interpolation to discover edge/triangle contacts.  The exact port
/// keeps the same edge/face ownership, but consumes the determinant-ratio
/// segment/plane construction used by the exact narrow phase.  This is the
/// Yap boundary in code: predicate side facts, exact parameter, constructed
/// point, and source handles replay together before any topology mutation.
///
/// The segment/plane event substrate follows the orientation-predicate
/// decomposition used by Moller, "A Fast Triangle-Triangle Intersection
/// Test," *Journal of Graphics Tools* 2.2 (1997), and Guigue and Devillers,
/// "Fast and Robust Triangle-Triangle Overlap Test Using Orientation
/// Predicates," *Journal of Graphics Tools* 8.1 (2003), with construction
/// retained exactly as required by Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshKernel12Event {
    /// Edge/face ownership key for this event.
    pub edge_face: ExactBoolMeshEdgeFacePair,
    /// Coarse exact relation between the closed edge segment and face plane.
    pub relation: SegmentPlaneRelation,
    /// Exact intersection point for endpoint and proper-crossing events.
    pub point: Option<Point3>,
    /// Exact segment parameter measured from `edge[0]` toward `edge[1]`.
    pub parameter: Option<ExactReal>,
    /// Determinant numerator/denominator that produced [`Self::parameter`].
    pub parameter_ratio: Option<SegmentPlaneParameterRatio>,
    /// Structured construction failure when side predicates certified a
    /// crossing but exact point construction failed.
    pub construction_failure: Option<SegmentPlaneConstructionFailure>,
    /// Certified side of each edge endpoint against the opposite face plane.
    pub endpoint_sides: [Option<PlaneSide>; 2],
}

/// Exact `Boolean03`-shaped package.
///
/// This mirrors the legacy `Boolean03` fields so the port can move one stage at
/// a time.  Empty vectors are meaningful for certified disjoint operands: they
/// prove the direct boolmesh workspace crossed discovery without invoking the
/// primitive-float adapter.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBoolMeshBoolean03 {
    /// Left-edge/right-face ownership pairs, legacy `p1q2`.
    pub p1q2: Vec<ExactBoolMeshEdgeFacePair>,
    /// Right-edge/left-face ownership pairs, legacy `p2q1`.
    pub p2q1: Vec<ExactBoolMeshEdgeFacePair>,
    /// Signed event multiplicity along left edges, legacy `x12`.
    pub x12: Vec<i32>,
    /// Signed event multiplicity along right edges, legacy `x21`.
    pub x21: Vec<i32>,
    /// Exact left-edge/right-face intersection points, legacy `v12`.
    pub v12: Vec<Point3>,
    /// Exact right-edge/left-face intersection points, legacy `v21`.
    pub v21: Vec<Point3>,
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
    /// Raw exact edge/face events discovered by the ported `kernel12` stage.
    pub kernel12_events: Vec<ExactBoolMeshKernel12Event>,
    /// Retained `kernel12` event records whose relation was undecidable.
    pub kernel12_unknown_events: usize,
    /// Certified crossings whose exact point construction failed.
    pub kernel12_construction_failures: usize,
    /// Coplanar graph events retained by discovery but not yet lowered into
    /// boolmesh edge/face split records.
    pub kernel12_coplanar_events: usize,
    /// Current exact `Boolean03` package.
    pub boolean03: ExactBoolMeshBoolean03,
    /// Exact `boolean45::pair_up` staging over source-edge events.
    pub pair_up: ExactBoolMeshPairUpStage,
    /// Current exact `boolean45` staging package, if output assembly has run.
    pub boolean45: Option<ExactBoolMeshBoolean45Stage>,
    /// First missing boolmesh stage for this workspace.
    pub blocker: Option<ExactBoolMeshPortBlocker>,
}

impl ExactBoolMeshWorkspace {
    /// Build the first exact boolmesh workspace from source meshes.
    ///
    /// This is the exact counterpart to entering legacy `boolean03`: it records
    /// source sizes, exact broad-phase scheduling, and the retained
    /// `kernel12` edge/face discovery records before any topology is emitted.
    /// Certified disjoint mesh bounds produce an empty `Boolean03` package.
    /// Non-coplanar segment/plane contacts advance the blocker to `kernel03`;
    /// unresolved or coplanar discovery still names `kernel12`, because those
    /// boolmesh branches are the next direct-port slices.
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
        let kernel12 = discover_kernel12_events(left, right);
        let kernel12_lowering = lower_kernel12_events(&kernel12.events);
        let pair_up = pair_source_edge_events(kernel12_lowering.source_edge_events.clone());
        let blocker = if candidate_face_pairs.is_empty() && !mesh_bounds_unknown {
            None
        } else if mesh_bounds_unknown
            || kernel12.graph_failed
            || kernel12.unknown_events > 0
            || kernel12.construction_failures > 0
            || kernel12.coplanar_events > 0
        {
            Some(ExactBoolMeshPortBlocker {
                stage: ExactBoolMeshKernelStage::Kernel12,
                candidate_face_pairs: candidate_face_pairs.len(),
                mesh_bounds_unknown,
            })
        } else {
            Some(ExactBoolMeshPortBlocker {
                stage: ExactBoolMeshKernelStage::Kernel03,
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
            kernel12_events: kernel12.events,
            kernel12_unknown_events: kernel12.unknown_events,
            kernel12_construction_failures: kernel12.construction_failures,
            kernel12_coplanar_events: kernel12.coplanar_events,
            boolean03: ExactBoolMeshBoolean03 {
                p1q2: kernel12_lowering.p1q2,
                p2q1: kernel12_lowering.p2q1,
                x12: kernel12_lowering.x12,
                x21: kernel12_lowering.x21,
                v12: kernel12_lowering.v12,
                v21: kernel12_lowering.v21,
                w03: vec![0; left.vertices().len()],
                w30: vec![0; right.vertices().len()],
            },
            pair_up,
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
            && self.boolean03.x12.is_empty()
            && self.boolean03.x21.is_empty()
            && self.boolean03.v12.is_empty()
            && self.boolean03.v21.is_empty()
            && self.pair_up.source_edge_runs.is_empty()
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
        if self.boolean03.p1q2.len() != self.boolean03.x12.len()
            || self.boolean03.p1q2.len() != self.boolean03.v12.len()
        {
            return Err(ExactBoolMeshValidationError::Kernel12TableLengthMismatch);
        }
        if self.boolean03.p2q1.len() != self.boolean03.x21.len()
            || self.boolean03.p2q1.len() != self.boolean03.v21.len()
        {
            return Err(ExactBoolMeshValidationError::Kernel12TableLengthMismatch);
        }
        let lowered_event_count = self.boolean03.p1q2.len() + self.boolean03.p2q1.len();
        if pair_up_event_count(&self.pair_up) != lowered_event_count {
            return Err(ExactBoolMeshValidationError::PairUpEventCountMismatch);
        }
        validate_pair_up_stage(&self.pair_up, self.left_vertices, self.right_vertices)?;
        for pair in &self.candidate_face_pairs {
            if pair.left_face >= self.left_faces || pair.right_face >= self.right_faces {
                return Err(ExactBoolMeshValidationError::FacePairOutOfBounds);
            }
        }
        for event in &self.kernel12_events {
            validate_edge_face_pair(
                event.edge_face,
                self.left_vertices,
                self.left_faces,
                self.right_vertices,
                self.right_faces,
            )?;
            validate_kernel12_event_shape(event)?;
        }
        for pair in &self.boolean03.p1q2 {
            validate_edge_face_pair(
                *pair,
                self.left_vertices,
                self.left_faces,
                self.right_vertices,
                self.right_faces,
            )?;
            if pair.edge_side != ExactBoolMeshSide::Left
                || pair.face_side != ExactBoolMeshSide::Right
            {
                return Err(ExactBoolMeshValidationError::Boolean03OwnershipMismatch);
            }
        }
        for pair in &self.boolean03.p2q1 {
            validate_edge_face_pair(
                *pair,
                self.left_vertices,
                self.left_faces,
                self.right_vertices,
                self.right_faces,
            )?;
            if pair.edge_side != ExactBoolMeshSide::Right
                || pair.face_side != ExactBoolMeshSide::Left
            {
                return Err(ExactBoolMeshValidationError::Boolean03OwnershipMismatch);
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
    /// A retained edge/face event names a missing source edge or face.
    EdgeFacePairOutOfBounds,
    /// A retained edge/face event uses the same side for both operands.
    EdgeFacePairSideMismatch,
    /// A `Boolean03` ownership table contains the opposite directed side.
    Boolean03OwnershipMismatch,
    /// A `kernel12` event relation and exact construction payload disagree.
    Kernel12EventShapeMismatch,
    /// A lowered `kernel12` ownership table does not align with signed events
    /// or exact vertices.
    Kernel12TableLengthMismatch,
    /// Exact pair-up event runs do not match lowered `kernel12` tables.
    PairUpEventCountMismatch,
    /// A paired edge run contains an event from a different source edge.
    PairUpRunEventMismatch,
    /// A paired edge run has stale pairing counts.
    PairUpRunCountMismatch,
    /// A paired edge run names a missing source edge endpoint.
    PairUpEdgeOutOfBounds,
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
    /// The executable slice was requested for a workspace blocked at a later
    /// or unresolved boolmesh stage.
    PortBlocked(ExactBoolMeshKernelStage),
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
/// [`ExactBoolMeshValidationError::PortBlocked`], naming the next direct port
/// stage instead of routing through bounded planar certificates.
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
        return Err(ExactBoolMeshValidationError::PortBlocked(
            workspace
                .blocker
                .as_ref()
                .map(|blocker| blocker.stage)
                .unwrap_or(ExactBoolMeshKernelStage::Boolean03),
        ));
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
            return Err(ExactBoolMeshValidationError::PortBlocked(
                ExactBoolMeshKernelStage::Boolean03,
            ));
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
#[derive(Clone, Debug, Default, PartialEq)]
struct Kernel12Discovery {
    events: Vec<ExactBoolMeshKernel12Event>,
    unknown_events: usize,
    construction_failures: usize,
    coplanar_events: usize,
    graph_failed: bool,
}

#[cfg(feature = "exact-triangulation")]
fn discover_kernel12_events(left: &ExactMesh, right: &ExactMesh) -> Kernel12Discovery {
    let graph = match build_intersection_graph(left, right) {
        Ok(graph) => graph,
        Err(_) => {
            return Kernel12Discovery {
                graph_failed: true,
                ..Kernel12Discovery::default()
            };
        }
    };
    let mut discovery = Kernel12Discovery::default();
    for pair in &graph.face_pairs {
        let face_pair = ExactBoolMeshFacePair {
            left_face: pair.left_face,
            right_face: pair.right_face,
        };
        for event in &pair.events {
            match event {
                IntersectionEvent::SegmentPlane {
                    segment_side,
                    edge,
                    plane_side,
                    plane_face,
                    relation,
                    point,
                    parameter,
                    parameter_ratio,
                    construction_failure,
                    endpoint_sides,
                } => {
                    if *relation == SegmentPlaneRelation::Unknown {
                        discovery.unknown_events += 1;
                    }
                    if *relation == SegmentPlaneRelation::ConstructionFailed {
                        discovery.construction_failures += 1;
                    }
                    if *relation == SegmentPlaneRelation::Coplanar {
                        discovery.coplanar_events += 1;
                    }
                    discovery.events.push(ExactBoolMeshKernel12Event {
                        edge_face: ExactBoolMeshEdgeFacePair {
                            face_pair,
                            edge_side: boolmesh_side(*segment_side),
                            edge: *edge,
                            face_side: boolmesh_side(*plane_side),
                            face: *plane_face,
                        },
                        relation: *relation,
                        point: point.clone(),
                        parameter: parameter.clone(),
                        parameter_ratio: parameter_ratio.clone(),
                        construction_failure: *construction_failure,
                        endpoint_sides: *endpoint_sides,
                    });
                }
                IntersectionEvent::CoplanarEdge { .. }
                | IntersectionEvent::CoplanarVertex { .. } => {
                    discovery.coplanar_events += 1;
                }
                IntersectionEvent::Unknown => {
                    discovery.unknown_events += 1;
                }
            }
        }
    }
    discovery
}

#[cfg(feature = "exact-triangulation")]
fn boolmesh_side(side: MeshSide) -> ExactBoolMeshSide {
    match side {
        MeshSide::Left => ExactBoolMeshSide::Left,
        MeshSide::Right => ExactBoolMeshSide::Right,
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_edge_face_pair(
    pair: ExactBoolMeshEdgeFacePair,
    left_vertices: usize,
    left_faces: usize,
    right_vertices: usize,
    right_faces: usize,
) -> Result<(), ExactBoolMeshValidationError> {
    if pair.edge_side == pair.face_side {
        return Err(ExactBoolMeshValidationError::EdgeFacePairSideMismatch);
    }
    if pair.face_pair.left_face >= left_faces || pair.face_pair.right_face >= right_faces {
        return Err(ExactBoolMeshValidationError::FacePairOutOfBounds);
    }
    let (edge_vertices, face_count) = match pair.edge_side {
        ExactBoolMeshSide::Left => (left_vertices, right_faces),
        ExactBoolMeshSide::Right => (right_vertices, left_faces),
    };
    if pair.edge[0] >= edge_vertices || pair.edge[1] >= edge_vertices {
        return Err(ExactBoolMeshValidationError::EdgeFacePairOutOfBounds);
    }
    if pair.face >= face_count {
        return Err(ExactBoolMeshValidationError::EdgeFacePairOutOfBounds);
    }
    let expected_face = match pair.face_side {
        ExactBoolMeshSide::Left => pair.face_pair.left_face,
        ExactBoolMeshSide::Right => pair.face_pair.right_face,
    };
    if pair.face != expected_face {
        return Err(ExactBoolMeshValidationError::EdgeFacePairOutOfBounds);
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn validate_kernel12_event_shape(
    event: &ExactBoolMeshKernel12Event,
) -> Result<(), ExactBoolMeshValidationError> {
    let construction_is_empty = event.point.is_none()
        && event.parameter.is_none()
        && event.parameter_ratio.is_none()
        && event.construction_failure.is_none();
    match event.relation {
        SegmentPlaneRelation::Disjoint | SegmentPlaneRelation::Coplanar => {
            if construction_is_empty {
                Ok(())
            } else {
                Err(ExactBoolMeshValidationError::Kernel12EventShapeMismatch)
            }
        }
        SegmentPlaneRelation::EndpointOnPlane => {
            if event.point.is_some()
                && event.parameter.is_some()
                && event.parameter_ratio.is_none()
                && event.construction_failure.is_none()
            {
                Ok(())
            } else {
                Err(ExactBoolMeshValidationError::Kernel12EventShapeMismatch)
            }
        }
        SegmentPlaneRelation::ProperCrossing => {
            if event.point.is_some()
                && event.parameter.is_some()
                && event.parameter_ratio.is_some()
                && event.construction_failure.is_none()
            {
                Ok(())
            } else {
                Err(ExactBoolMeshValidationError::Kernel12EventShapeMismatch)
            }
        }
        SegmentPlaneRelation::Unknown => {
            if event.point.is_none()
                && event.parameter.is_none()
                && event.parameter_ratio.is_none()
                && event.construction_failure.is_none()
            {
                Ok(())
            } else {
                Err(ExactBoolMeshValidationError::Kernel12EventShapeMismatch)
            }
        }
        SegmentPlaneRelation::ConstructionFailed => {
            if event.point.is_none()
                && event.parameter.is_none()
                && event.parameter_ratio.is_none()
                && event.construction_failure.is_some()
            {
                Ok(())
            } else {
                Err(ExactBoolMeshValidationError::Kernel12EventShapeMismatch)
            }
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn pair_up_event_count(stage: &ExactBoolMeshPairUpStage) -> usize {
    stage
        .source_edge_runs
        .iter()
        .map(|run| run.events.len())
        .sum()
}

#[cfg(feature = "exact-triangulation")]
fn validate_pair_up_stage(
    stage: &ExactBoolMeshPairUpStage,
    left_vertices: usize,
    right_vertices: usize,
) -> Result<(), ExactBoolMeshValidationError> {
    let mut unpaired_event_runs = 0;
    for run in &stage.source_edge_runs {
        let vertex_count = match run.side {
            ExactBoolMeshSide::Left => left_vertices,
            ExactBoolMeshSide::Right => right_vertices,
        };
        if run.tail >= vertex_count || run.head >= vertex_count {
            return Err(ExactBoolMeshValidationError::PairUpEdgeOutOfBounds);
        }
        let tail_count = run.events.iter().filter(|event| event.is_tail).count();
        let head_count = run.events.len() - tail_count;
        let unpaired_events = tail_count.abs_diff(head_count);
        if unpaired_events > 0 {
            unpaired_event_runs += 1;
        }
        if run.unpaired_events != unpaired_events
            || run.fragments.len() != tail_count.min(head_count)
        {
            return Err(ExactBoolMeshValidationError::PairUpRunCountMismatch);
        }
        for event in &run.events {
            validate_pair_up_event(event, run, vertex_count)?;
        }
        for fragment in &run.fragments {
            if fragment.side != run.side || fragment.tail != run.tail || fragment.head != run.head {
                return Err(ExactBoolMeshValidationError::PairUpRunEventMismatch);
            }
            validate_pair_up_event(&fragment.tail_event, run, vertex_count)?;
            validate_pair_up_event(&fragment.head_event, run, vertex_count)?;
            if !fragment.tail_event.is_tail || fragment.head_event.is_tail {
                return Err(ExactBoolMeshValidationError::PairUpRunEventMismatch);
            }
        }
    }
    if stage.unpaired_event_runs != unpaired_event_runs {
        return Err(ExactBoolMeshValidationError::PairUpRunCountMismatch);
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn validate_pair_up_event(
    event: &ExactBoolMeshEdgeEvent,
    run: &ExactBoolMeshSourceEdgeRun,
    vertex_count: usize,
) -> Result<(), ExactBoolMeshValidationError> {
    if event.side != run.side
        || event.tail != run.tail
        || event.head != run.head
        || event.tail >= vertex_count
        || event.head >= vertex_count
    {
        return Err(ExactBoolMeshValidationError::PairUpRunEventMismatch);
    }
    Ok(())
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
