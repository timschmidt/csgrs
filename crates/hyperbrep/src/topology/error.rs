//! Typed construction, query and edit failures.

use super::*;

/// Kind of model record referenced by an invalid ID.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum EntityKind {
    /// Vertex record.
    Vertex,
    /// Spatial curve record.
    Curve3,
    /// Parameter-space curve record.
    Pcurve,
    /// Surface record.
    Surface,
    /// Edge record.
    Edge,
    /// Edge-use record.
    EdgeUse,
    /// Wire record.
    Wire,
    /// Face record.
    Face,
    /// Shell record.
    Shell,
    /// Solid record.
    Solid,
}

/// Failure while querying a validated model with a model-local ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QueryError {
    /// A supplied typed ID does not resolve in this model.
    InvalidReference {
        /// Referenced record family.
        kind: EntityKind,
        /// Invalid model-local index.
        index: usize,
    },
    /// Exact geometry evaluation failed.
    Geometry(GeometryError),
}

impl fmt::Display for QueryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidReference { kind, index } => {
                write!(formatter, "invalid {kind:?} reference at index {index}")
            }
            Self::Geometry(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for QueryError {}

impl From<GeometryError> for QueryError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

/// Failure while staging a transactional model edit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EditError {
    /// Exact geometry transformation failed.
    Geometry(GeometryError),
    /// A replacement targeted an ID outside the staged model.
    InvalidReference {
        /// Referenced record family.
        kind: EntityKind,
        /// Invalid model-local index.
        index: usize,
    },
    /// The staged snapshot did not pass canonical model validation.
    Validation(ValidationReport),
}

impl fmt::Display for EditError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Geometry(error) => error.fmt(formatter),
            Self::InvalidReference { kind, index } => {
                write!(formatter, "invalid {kind:?} edit target at index {index}")
            }
            Self::Validation(report) => report.fmt(formatter),
        }
    }
}

impl std::error::Error for EditError {}

impl From<GeometryError> for EditError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

/// Failure while applying a topology-changing exact edit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TopologyEditError {
    /// The target record does not exist in this model.
    InvalidReference {
        /// Referenced record family.
        kind: EntityKind,
        /// Invalid model-local index.
        index: usize,
    },
    /// A face split was requested for a complete closed-surface face.
    WholeSurfaceFace(FaceId),
    /// The vertex-only chord API supports only parameterized planes; authored
    /// surface curves use [`Model::split_face_by_surface_curve`].
    UnsupportedFaceSplitSurface(SurfaceKind),
    /// The requested curve family or parameter correspondence has no complete
    /// exact face-split certificate.
    UnsupportedFaceSplitCurve(Curve3Kind),
    /// One curve endpoint does not lie on the face's outer boundary.
    FaceSplitEndpointNotOnOuterBoundary {
        /// Face being split.
        face: FaceId,
        /// Curve endpoint that could not be attached to the boundary.
        endpoint: Endpoint,
    },
    /// One curve endpoint has more than one strict outer-edge location.
    FaceSplitEndpointAmbiguous {
        /// Face being split.
        face: FaceId,
        /// Curve endpoint with multiple exact boundary locations.
        endpoint: Endpoint,
    },
    /// A closed trace is not wholly inside the face's material region.
    ClosedFaceSplitNotInMaterial {
        /// Face whose material region does not wholly contain the trace.
        face: FaceId,
    },
    /// Two input traces have the same direction-independent exact carrier.
    DuplicateFaceSplitTrace {
        /// First duplicate index in the caller's trace slice.
        first: usize,
        /// Second duplicate index in the caller's trace slice.
        second: usize,
    },
    /// Two input traces have a positive-length collinear overlap.
    OverlappingFaceSplitTraces {
        /// First overlapping index in the caller's trace slice.
        first: usize,
        /// Second overlapping index in the caller's trace slice.
        second: usize,
    },
    /// One arranged trace segment does not belong to exactly one current
    /// descendant face.
    FaceSplitTraceNotInSingleRegion {
        /// Original face being partitioned.
        face: FaceId,
        /// Index in the caller's trace slice.
        trace: usize,
        /// Arranged segment index along the canonical trace direction.
        segment: usize,
    },
    /// One arranged trace segment belongs to more than one current descendant
    /// face.
    FaceSplitTraceAmbiguous {
        /// Original face being partitioned.
        face: FaceId,
        /// Index in the caller's trace slice.
        trace: usize,
        /// Arranged segment index along the canonical trace direction.
        segment: usize,
    },
    /// A requested split vertex is not a unique vertex of the outer boundary.
    VertexNotOnOuterBoundary {
        /// Face being split.
        face: FaceId,
        /// Vertex absent from, or repeated by, the outer boundary.
        vertex: VertexId,
    },
    /// The two split vertices are equal or adjacent on the outer boundary.
    DegenerateFaceSplit,
    /// The requested exact geometry operation failed.
    Geometry(GeometryError),
    /// Rebuilding an edited local record failed.
    Build(BuildError),
    /// The complete edited snapshot did not pass canonical validation.
    Validation(ValidationReport),
}

impl fmt::Display for TopologyEditError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidReference { kind, index } => {
                write!(
                    formatter,
                    "invalid {kind:?} topology-edit target at index {index}"
                )
            }
            Self::WholeSurfaceFace(face) => {
                write!(formatter, "face {face:?} has no trimmable boundary")
            }
            Self::UnsupportedFaceSplitSurface(surface) => {
                write!(
                    formatter,
                    "exact face splitting is unsupported on {surface:?}"
                )
            }
            Self::UnsupportedFaceSplitCurve(curve) => {
                write!(
                    formatter,
                    "exact curve-driven face splitting is unsupported for {curve:?}"
                )
            }
            Self::FaceSplitEndpointNotOnOuterBoundary { face, endpoint } => {
                write!(
                    formatter,
                    "{endpoint:?} endpoint is not on face {face:?}'s outer boundary"
                )
            }
            Self::FaceSplitEndpointAmbiguous { face, endpoint } => {
                write!(
                    formatter,
                    "{endpoint:?} endpoint is ambiguous on face {face:?}'s outer boundary"
                )
            }
            Self::ClosedFaceSplitNotInMaterial { face } => {
                write!(
                    formatter,
                    "closed face-split trace is not wholly inside face {face:?}'s material region"
                )
            }
            Self::DuplicateFaceSplitTrace { first, second } => {
                write!(
                    formatter,
                    "face-split traces {first} and {second} have the same direction-independent exact carrier"
                )
            }
            Self::OverlappingFaceSplitTraces { first, second } => {
                write!(
                    formatter,
                    "face-split traces {first} and {second} overlap over positive exact length"
                )
            }
            Self::FaceSplitTraceNotInSingleRegion {
                face,
                trace,
                segment,
            } => {
                write!(
                    formatter,
                    "face-split trace {trace} segment {segment} does not belong to one descendant of face {face:?}"
                )
            }
            Self::FaceSplitTraceAmbiguous {
                face,
                trace,
                segment,
            } => {
                write!(
                    formatter,
                    "face-split trace {trace} segment {segment} is ambiguous across descendants of face {face:?}"
                )
            }
            Self::VertexNotOnOuterBoundary { face, vertex } => {
                write!(
                    formatter,
                    "vertex {vertex:?} is not unique on face {face:?}'s outer boundary"
                )
            }
            Self::DegenerateFaceSplit => {
                formatter.write_str("face split vertices must be distinct and nonadjacent")
            }
            Self::Geometry(error) => error.fmt(formatter),
            Self::Build(error) => error.fmt(formatter),
            Self::Validation(report) => report.fmt(formatter),
        }
    }
}

impl std::error::Error for TopologyEditError {}

impl From<GeometryError> for TopologyEditError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

impl From<BuildError> for TopologyEditError {
    fn from(value: BuildError) -> Self {
        Self::Build(value)
    }
}

/// Endpoint involved in an edge or pcurve agreement failure.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Endpoint {
    /// Start of the directed interval.
    Start,
    /// End of the directed interval.
    End,
}

/// Failure while staging or validating a canonical BREP model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BuildError {
    /// An arena cannot assign another compact identifier.
    CapacityExceeded(EntityKind),
    /// A supplied typed ID does not resolve in this builder.
    InvalidReference {
        /// Referenced record family.
        kind: EntityKind,
        /// Invalid model-local index.
        index: usize,
    },
    /// Exact geometry construction or evaluation failed.
    Geometry(GeometryError),
    /// An edge references the same topological vertex twice.
    DegenerateEdge,
    /// A curve/domain endpoint does not agree with the edge's vertex point.
    EdgeEndpointMismatch {
        /// Mismatched endpoint.
        endpoint: Endpoint,
    },
    /// An affine pcurve-to-edge correspondence has zero scale.
    DegenerateParameterCorrespondence,
    /// A wire has no edge uses.
    EmptyWire,
    /// One edge use appeared more than once in a wire.
    DuplicateEdgeUse(EdgeUseId),
    /// An edge use already belongs to another wire.
    EdgeUseAlreadyOwned(EdgeUseId),
    /// Consecutive oriented edge uses do not share a topological vertex.
    DisconnectedWire {
        /// Index of the first edge use in the broken pair.
        at: usize,
    },
    /// A wire is not closed by topological vertex identity.
    OpenWire,
    /// A face repeats one of its wires.
    DuplicateWire(WireId),
    /// A wire already belongs to another face.
    WireAlreadyOwned(WireId),
    /// A boundaryless whole face was requested for a non-closed carrier.
    UnsupportedWholeSurface(SurfaceKind),
    /// A complete closed-surface face cannot also carry trimming wires.
    WholeSurfaceHasInnerBoundaries,
    /// A planar wire encloses no certified nonzero parameter-space area.
    DegenerateWireArea(WireId),
    /// A planar outer wire's winding disagrees with its face orientation.
    InconsistentWireOrientation(WireId),
    /// A periodic spherical trim has inner wires or a non-latitude boundary.
    UnsupportedSphericalTrim(WireId),
    /// A spherical latitude wire does not close by exactly one periodic turn.
    InvalidSphericalTrim(WireId),
    /// A face wire crosses or overlaps itself.
    SelfIntersectingWire(WireId),
    /// A face boundary wire crosses or overlaps another boundary wire.
    IntersectingFaceWires {
        /// First conflicting wire.
        first: WireId,
        /// Second conflicting wire.
        second: WireId,
    },
    /// An inner wire is not strictly contained by the outer wire.
    InnerWireOutside(WireId),
    /// Two inner wires are nested rather than defining disjoint holes.
    NestedInnerWires {
        /// First nested wire.
        first: WireId,
        /// Second nested wire.
        second: WireId,
    },
    /// The current geometry-family combination has no image-agreement proof.
    UnsupportedEdgeUseAgreement {
        /// Spatial curve family.
        curve: Curve3Kind,
        /// Pcurve family.
        pcurve: CurveFamily2,
        /// Surface family.
        surface: SurfaceKind,
    },
    /// The correspondence does not map pcurve endpoints to the directed edge domain.
    ParameterCorrespondenceMismatch {
        /// Mismatched pcurve endpoint.
        endpoint: Endpoint,
    },
    /// Surface-evaluated pcurve and spatial edge disagree.
    EdgeUseImageMismatch {
        /// Mismatched pcurve endpoint.
        endpoint: Endpoint,
    },
    /// The complete analytic support frame of a pcurve image disagrees with
    /// its spatial edge.
    EdgeUseSupportMismatch,
    /// The directed analytic sweep of a pcurve image disagrees with its edge
    /// interval.
    EdgeUseSweepMismatch,
    /// A shell has no faces.
    EmptyShell,
    /// A shell repeats one face.
    DuplicateFace(FaceId),
    /// A face already belongs to another shell.
    FaceAlreadyOwned(FaceId),
    /// Shell faces do not form one edge-connected component.
    DisconnectedShell,
    /// A solid repeats one shell.
    DuplicateShell(ShellId),
    /// A shell already belongs to another solid.
    ShellAlreadyOwned(ShellId),
    /// A shell edge does not have exactly two uses.
    NonManifoldSolidEdge {
        /// Edge with invalid use count.
        edge: EdgeId,
        /// Number of uses in the shell.
        uses: usize,
    },
    /// A closed-shell edge pair traverses in the same direction.
    InconsistentSolidEdgeOrientation(EdgeId),
    /// A closed planar shell encloses no certified nonzero signed volume.
    DegenerateShellVolume(ShellId),
    /// A solid's outer shell is not certified outward-oriented.
    InwardSolidShell(ShellId),
    /// A void shell is not certified inward-oriented.
    OutwardVoidShell(ShellId),
    /// The shell is not recognized by the active non-self-intersection proof.
    UnsupportedSolidShell(ShellId),
    /// Planar face sheets contact away from one shared topological edge or vertex.
    SelfIntersectingSolidShell(ShellId),
    /// A void shell is not strictly contained by the outer shell.
    VoidShellOutside(ShellId),
    /// Two void shells touch, overlap, or contain one another.
    IntersectingVoidShells {
        /// First conflicting void shell.
        first: ShellId,
        /// Second conflicting void shell.
        second: ShellId,
    },
    /// A staged edge use was never assigned to a wire.
    OrphanEdgeUse(EdgeUseId),
    /// A staged wire was never assigned to a face.
    OrphanWire(WireId),
    /// A staged face was never assigned to a shell.
    OrphanFace(FaceId),
}

impl fmt::Display for BuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for BuildError {}

impl From<GeometryError> for BuildError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

/// Complete set of blockers found while committing a staged model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidationReport {
    pub(super) errors: Vec<BuildError>,
}

impl ValidationReport {
    /// Returns all known blockers in deterministic arena order.
    pub fn errors(&self) -> &[BuildError] {
        &self.errors
    }

    /// Consumes the report into its blockers.
    pub fn into_errors(self) -> Vec<BuildError> {
        self.errors
    }
}

impl fmt::Display for ValidationReport {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{} model validation blocker(s)",
            self.errors.len()
        )
    }
}

impl std::error::Error for ValidationReport {}
