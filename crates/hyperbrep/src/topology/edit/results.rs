//! Retained topology changes from splitting and partitioning.

use crate::topology::*;

/// Stable identifiers produced by one exact edge split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaEdgeSplit {
    /// Newly inserted vertex at the split point.
    pub vertex: VertexId,
    /// First canonical edge half; this retains the source edge ID.
    pub first: EdgeId,
    /// Newly appended second canonical edge half.
    pub second: EdgeId,
    /// Existing edge-use IDs and their newly appended continuation uses.
    pub edge_uses: Vec<(EdgeUseId, EdgeUseId)>,
}

/// Stable identifiers produced by one exact planar face split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaFaceSplit {
    /// Newly appended chord edge shared by the two resulting faces.
    pub edge: EdgeId,
    /// Oppositely directed chord uses, in first-face then second-face order.
    pub edge_uses: [EdgeUseId; 2],
    /// First boundary wire; this retains the source face's outer-wire ID.
    pub first_wire: WireId,
    /// Newly appended second boundary wire.
    pub second_wire: WireId,
    /// First face; this retains the source face ID.
    pub first_face: FaceId,
    /// Newly appended second face.
    pub second_face: FaceId,
}

/// Stable identifiers produced by one exact curve-driven planar face split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaCurveFaceSplit {
    /// Optional canonical edge split that attached the curve start.
    pub start_edge: Option<ArenaEdgeSplit>,
    /// Optional canonical edge split that attached the curve end.
    pub end_edge: Option<ArenaEdgeSplit>,
    /// The resulting identity-stitched face split.
    pub face: ArenaFaceSplit,
}

/// Identifiers produced by an exact boundary-component bridge cycle.
///
/// Two bridges connect one inner wire to the outer wire. More generally, one
/// support threading several holes produces one bridge per retained material
/// fragment. The crossed boundary wires form a cycle and are compacted into
/// the two descendant outer wires.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaBoundaryBridgeFaceSplit {
    /// Newly appended bridge edges in canonical trace order.
    pub edges: Vec<EdgeId>,
    /// Oppositely directed uses of each bridge, in first-face then second-face
    /// order.
    pub edge_uses: Vec<[EdgeUseId; 2]>,
    /// First descendant boundary wire after compaction.
    pub first_wire: WireId,
    /// Second descendant boundary wire after compaction.
    pub second_wire: WireId,
    /// Complete pre-edit wire-ID map after compaction.
    ///
    /// Each index is an old [`WireId`]. Surviving records name their new ID;
    /// crossed components retired into the descendant loops map to `None`.
    pub wire_remap: Vec<Option<WireId>>,
    /// First descendant retaining the source face ID.
    pub first_face: FaceId,
    /// Newly appended second descendant.
    pub second_face: FaceId,
}

/// Per-trace evidence for one member of a boundary-component bridge cycle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaBoundaryBridgeCurveSplit {
    /// Optional canonical edge split that attached the curve start.
    pub start_edge: Option<ArenaEdgeSplit>,
    /// Optional canonical edge split that attached the curve end.
    pub end_edge: Option<ArenaEdgeSplit>,
    /// Index of this trace's bridge in [`ArenaBoundaryBridgeFaceSplit::edges`].
    pub bridge_index: usize,
    /// Shared bridge-cycle topology result.
    pub face: ArenaBoundaryBridgeFaceSplit,
}

/// Stable result of one exact retained surface-curve split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ArenaSurfaceCurveFaceSplit {
    /// An open trace attached to the source face's outer boundary.
    Open(ArenaCurveFaceSplit),
    /// One member of a bridge cycle joining crossed boundary components.
    Bridge(ArenaBoundaryBridgeCurveSplit),
    /// A closed trace wholly inside the source face's material region.
    Closed(ArenaClosedSurfaceCurveFaceSplit),
}

impl ArenaSurfaceCurveFaceSplit {
    /// Returns the outer-boundary chord result when this trace is open.
    pub const fn open(&self) -> Option<&ArenaCurveFaceSplit> {
        match self {
            Self::Open(split) => Some(split),
            Self::Bridge(_) | Self::Closed(_) => None,
        }
    }

    /// Returns boundary-component bridge evidence for this trace.
    pub const fn bridge(&self) -> Option<&ArenaBoundaryBridgeCurveSplit> {
        match self {
            Self::Bridge(split) => Some(split),
            Self::Open(_) | Self::Closed(_) => None,
        }
    }

    /// Returns the outer/inner-wire result when this trace is closed.
    pub const fn closed(&self) -> Option<&ArenaClosedSurfaceCurveFaceSplit> {
        match self {
            Self::Open(_) | Self::Bridge(_) => None,
            Self::Closed(split) => Some(split),
        }
    }

    /// Returns the source face retained by the first descendant.
    pub const fn first_face(&self) -> FaceId {
        match self {
            Self::Open(split) => split.face.first_face,
            Self::Bridge(split) => split.face.first_face,
            Self::Closed(split) => split.first_face,
        }
    }

    /// Returns the newly appended enclosed or second descendant.
    pub const fn second_face(&self) -> FaceId {
        match self {
            Self::Open(split) => split.face.second_face,
            Self::Bridge(split) => split.face.second_face,
            Self::Closed(split) => split.second_face,
        }
    }
}

/// Stable identifiers produced by one closed surface curve.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArenaClosedSurfaceCurveFaceSplit {
    /// Newly inserted vertex at the retained curve's public-domain seam.
    pub seam_vertex: VertexId,
    /// Newly inserted vertex at the exact midpoint used to avoid a loop edge.
    pub midpoint_vertex: VertexId,
    /// Two canonical curve halves shared by both descendants.
    pub edges: [EdgeId; 2],
    /// Uses on the retained first descendant's new boundary wire.
    pub first_edge_uses: [EdgeUseId; 2],
    /// Uses on the appended second descendant's new boundary wire.
    pub second_edge_uses: [EdgeUseId; 2],
    /// New boundary wire on the retained first descendant.
    pub first_wire: WireId,
    /// New boundary wire on the appended second descendant.
    pub second_wire: WireId,
    /// First descendant retaining the source face ID.
    pub first_face: FaceId,
    /// Newly appended second descendant.
    pub second_face: FaceId,
}

/// Stable result of deterministic exact multi-trace face partitioning.
#[derive(Clone, Debug)]
pub struct ArenaFacePartition {
    /// Original face ID; this remains the first descendant.
    pub source_face: FaceId,
    /// Final descendant faces in deterministic split order.
    pub faces: Vec<FaceId>,
    /// Canonical interior edit records followed by boundary records in input order.
    pub traces: Vec<ArenaFaceTracePartition>,
}

/// Exact arrangement segments and topology edits for one source trace.
#[derive(Clone, Debug)]
pub struct ArenaFaceTracePartition {
    /// Index in the caller's trace slice.
    pub source_index: usize,
    /// Arranged spatial segments, or the original unsplit boundary trace.
    pub segments: Vec<Curve3>,
    /// Identity-stitched splits for interior segments; empty for boundary traces.
    pub splits: Vec<ArenaSurfaceCurveFaceSplit>,
    /// Complete overlap proof when the trace already belongs to the source boundary.
    pub boundary: Option<ArenaBoundarySurfaceTrace>,
    /// Input and output revision for each retained split step.
    pub(crate) snapshots: Vec<(ArenaModel, ArenaModel)>,
}

/// An unchanged source boundary covering one complete retained surface trace.
#[derive(Clone, Debug)]
pub struct ArenaBoundarySurfaceTrace {
    pub source: ArenaModel,
    pub intersection: SurfaceIntersectionCurve,
    pub operand: SurfaceIntersectionOperand,
    pub overlaps: Vec<ArenaBoundaryTraceOverlap>,
}

/// Native boundary overlap in the two original public pcurve charts.
#[derive(Clone, Debug)]
pub struct ArenaBoundaryTraceOverlap {
    pub edge_use: EdgeUseId,
    pub trace_range: hypercurve::ParamRange,
    pub boundary_range: hypercurve::ParamRange,
    /// Complete native span witness for a circular overlap. Line overlaps
    /// retain their affine paired ranges directly.
    pub native_overlap: Option<hypercurve::CurveIntersectionOverlap2>,
}
