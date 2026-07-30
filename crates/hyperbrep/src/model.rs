//! Immutable typed-arena BREP model and validated construction.

use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt;
use std::sync::{Arc, OnceLock};

use hypercurve::{
    BooleanOp, CircularArc2, Classification, Contour2, ContourPointLocation, CubicBezier2, Curve2,
    CurveFamily2, CurveGeometry2, CurvePath2, CurvePolicy, FillRule, LineArcRegion2,
    LineLineIntersection, LineSeg2, Point2 as CurvePoint2, QuadraticBezier2, RationalBezier2,
    RationalQuadraticBezier2, RegionPointLocation, Segment2,
};
use hyperlattice::{Aabb, Matrix4, Point2, Point3, Real, Vector3};
use hyperlimit::{PredicateOutcome, compare_point3_lexicographic, compare_reals, point3_equal};

use crate::error::GeometryError;
use crate::geometry::{
    Curve3, Curve3ExactData, Curve3Kind, CurveParameterLocation, ParameterDomain, Pcurve, Surface,
    SurfaceExactData, SurfaceIntersectionCurve, SurfaceIntersectionOperand,
    SurfaceIntersectionPcurve, SurfaceIsoAxis, SurfaceKind, SurfaceParameterDomain,
    SurfacePcurveCorrespondence, affine_transform_orientation, materialize_nurbs_parameter_graph,
};

macro_rules! model_id {
    ($name:ident, $description:literal) => {
        #[doc = $description]
        #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(u32);

        impl $name {
            pub(crate) fn from_index(index: usize) -> Option<Self> {
                u32::try_from(index).ok().map(Self)
            }

            /// Returns the deterministic arena index for this model-local ID.
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

model_id!(VertexId, "Model-local vertex identifier.");
model_id!(Curve3Id, "Model-local spatial-curve identifier.");
model_id!(PcurveId, "Model-local surface-parameter curve identifier.");
model_id!(SurfaceId, "Model-local surface identifier.");
model_id!(EdgeId, "Model-local edge identifier.");
model_id!(EdgeUseId, "Model-local oriented edge-use identifier.");
model_id!(WireId, "Model-local wire identifier.");
model_id!(FaceId, "Model-local face identifier.");
model_id!(ShellId, "Model-local shell identifier.");
model_id!(SolidId, "Model-local solid identifier.");

/// Direction of an edge use relative to its edge's canonical parameterization.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Direction {
    /// Traverse from the edge's start vertex to its end vertex.
    Forward,
    /// Traverse from the edge's end vertex to its start vertex.
    Reversed,
}

impl Direction {
    /// Returns the opposite traversal direction.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
}

/// Orientation of a face or shell relative to its authored geometry.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Orientation {
    /// Use the authored orientation.
    Forward,
    /// Reverse the authored orientation.
    Reversed,
}

impl Orientation {
    /// Returns the opposite surface orientation.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
}

/// Exact relation from a face-local pcurve parameterization to an edge curve.
///
/// A BREP edge has one canonical 3D parameterization while each incident face
/// owns a pcurve parameterization. Those parameterizations need not be
/// affinely related: a native rational circular pcurve on a planar cap and an
/// angle-parameterized spatial circle are the canonical example.
#[derive(Clone, Debug)]
pub enum ParameterCorrespondence {
    /// `edge_parameter = scale * pcurve_parameter + offset`.
    Affine {
        /// Nonzero exact scale.
        scale: Real,
        /// Exact offset.
        offset: Real,
    },
    /// The pcurve's directed angular sweep fraction spans the directed edge
    /// domain.
    ///
    /// This relation is valid only for a native circular-arc pcurve and a
    /// spatial circular arc. Model validation certifies the complete image,
    /// support frame, orientation, and sweep before publication.
    AngularSweep,
}

impl ParameterCorrespondence {
    /// Constructs `edge_parameter = scale * pcurve_parameter + offset`.
    pub fn affine(scale: Real, offset: Real) -> Result<Self, BuildError> {
        match compare_reals(&scale, &Real::zero()) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            } => Err(BuildError::DegenerateParameterCorrespondence),
            PredicateOutcome::Decided { .. } => Ok(Self::Affine { scale, offset }),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                }))
            }
        }
    }

    /// Returns the identity parameter correspondence.
    pub fn identity() -> Self {
        Self::Affine {
            scale: Real::one(),
            offset: Real::zero(),
        }
    }

    /// Relates a native circular pcurve's directed sweep to the edge domain.
    pub const fn angular_sweep() -> Self {
        Self::AngularSweep
    }

    /// Returns affine coefficients, or `None` for a non-affine relation.
    pub const fn affine_coefficients(&self) -> Option<(&Real, &Real)> {
        match self {
            Self::Affine { scale, offset } => Some((scale, offset)),
            Self::AngularSweep => None,
        }
    }

    fn edge_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterDomain,
        direction: Direction,
        pcurve_parameter: &Real,
    ) -> Result<Real, GeometryError> {
        match self {
            Self::Affine { scale, offset } => Ok(scale * pcurve_parameter + offset),
            Self::AngularSweep => {
                let arc = pcurve
                    .circular_arc()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?;
                let point = pcurve.point_at(pcurve_parameter)?;
                let point = CurvePoint2::new(point.x, point.y);
                let fraction = match arc.sweep_fraction(&point, &CurvePolicy::certified())? {
                    Classification::Decided(fraction) => fraction,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason));
                    }
                };
                let (start, end) = match direction {
                    Direction::Forward => (edge_domain.start(), edge_domain.end()),
                    Direction::Reversed => (edge_domain.end(), edge_domain.start()),
                };
                Ok(start + &((end - start) * fraction))
            }
        }
    }

    fn pcurve_parameter(
        &self,
        pcurve: &Pcurve,
        edge_domain: &ParameterDomain,
        direction: Direction,
        edge_parameter: &Real,
    ) -> Result<Real, GeometryError> {
        match self {
            Self::Affine { scale, offset } => {
                ((edge_parameter - offset) / scale).map_err(|_| GeometryError::ProjectiveDivision)
            }
            Self::AngularSweep => {
                let arc = pcurve
                    .circular_arc()
                    .ok_or(GeometryError::UnsupportedPcurveContour)?;
                let (start, end) = match direction {
                    Direction::Forward => (edge_domain.start(), edge_domain.end()),
                    Direction::Reversed => (edge_domain.end(), edge_domain.start()),
                };
                let fraction = ((edge_parameter - start) / (end - start))
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                match arc.parameter_at_sweep_fraction(&fraction, &CurvePolicy::certified())? {
                    Classification::Decided(parameter) => Ok(parameter),
                    Classification::Uncertain(reason) => {
                        Err(GeometryError::PlanarClassificationUnresolved(reason))
                    }
                }
            }
        }
    }

    pub(crate) fn reversed_pcurve(&self, pcurve: &Pcurve) -> Self {
        match self {
            Self::Affine { scale, offset } => Self::Affine {
                scale: -scale.clone(),
                offset: scale * (pcurve.domain_start() + pcurve.domain_end()) + offset,
            },
            Self::AngularSweep => Self::AngularSweep,
        }
    }

    pub(crate) fn remapped_edge(
        &self,
        source: &ParameterDomain,
        target: &ParameterDomain,
        reversed: bool,
    ) -> Result<Self, GeometryError> {
        match self {
            Self::Affine { scale, offset } => {
                let factor = ((target.end() - target.start()) / (source.end() - source.start()))
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                Ok(if reversed {
                    Self::Affine {
                        scale: -(&factor * scale),
                        offset: target.end() - &factor * (offset - source.start()),
                    }
                } else {
                    Self::Affine {
                        scale: &factor * scale,
                        offset: target.start() + factor * (offset - source.start()),
                    }
                })
            }
            Self::AngularSweep => Ok(Self::AngularSweep),
        }
    }
}

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

/// Exact location of a point relative to a solid.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SolidPointLocation {
    /// The point belongs to the solid interior.
    Inside,
    /// The point does not belong to the solid.
    Outside,
    /// The point lies on a face, edge, or vertex.
    Boundary,
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

/// Stable identifiers produced by one exact edge split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdgeSplit {
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
pub struct FaceSplit {
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
pub struct CurveFaceSplit {
    /// Optional canonical edge split that attached the curve start.
    pub start_edge: Option<EdgeSplit>,
    /// Optional canonical edge split that attached the curve end.
    pub end_edge: Option<EdgeSplit>,
    /// The resulting identity-stitched face split.
    pub face: FaceSplit,
}

/// Stable result of one exact retained surface-curve split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SurfaceCurveFaceSplit {
    /// An open trace attached to the source face's outer boundary.
    Open(CurveFaceSplit),
    /// A closed trace wholly inside the source face's material region.
    Closed(ClosedSurfaceCurveFaceSplit),
}

impl SurfaceCurveFaceSplit {
    /// Returns the outer-boundary chord result when this trace is open.
    pub const fn open(&self) -> Option<&CurveFaceSplit> {
        match self {
            Self::Open(split) => Some(split),
            Self::Closed(_) => None,
        }
    }

    /// Returns the outer/inner-wire result when this trace is closed.
    pub const fn closed(&self) -> Option<&ClosedSurfaceCurveFaceSplit> {
        match self {
            Self::Open(_) => None,
            Self::Closed(split) => Some(split),
        }
    }

    /// Returns the source face retained by the first descendant.
    pub const fn first_face(&self) -> FaceId {
        match self {
            Self::Open(split) => split.face.first_face,
            Self::Closed(split) => split.first_face,
        }
    }

    /// Returns the newly appended enclosed or second descendant.
    pub const fn second_face(&self) -> FaceId {
        match self {
            Self::Open(split) => split.face.second_face,
            Self::Closed(split) => split.second_face,
        }
    }
}

/// Stable identifiers produced by one closed surface curve.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClosedSurfaceCurveFaceSplit {
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
pub struct FacePartition {
    /// Original face ID; this remains the first descendant.
    pub source_face: FaceId,
    /// Final descendant faces in deterministic split order.
    pub faces: Vec<FaceId>,
    /// Per-trace arrangement and edit records in canonical processing order.
    pub traces: Vec<FaceTracePartition>,
}

/// Exact arrangement segments and topology edits for one source trace.
#[derive(Clone, Debug)]
pub struct FaceTracePartition {
    /// Index in the caller's trace slice.
    pub source_index: usize,
    /// Canonically directed trace segments after exact crossing subdivision.
    pub segments: Vec<Curve3>,
    /// Identity-stitched face split corresponding to every segment.
    pub splits: Vec<SurfaceCurveFaceSplit>,
}

enum FaceSplitEndpointLocation {
    Vertex(VertexId),
    Edge { edge: EdgeId, parameter: Real },
}

struct OrderedFaceSplitTrace {
    source_index: usize,
    curve: Curve3,
    lower: Point3,
    upper: Point3,
}

struct OrderedSurfaceCurveTrace {
    source_index: usize,
    intersection: SurfaceIntersectionCurve,
    lower: Point3,
    upper: Point3,
    exact_key: Curve3,
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
    errors: Vec<BuildError>,
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

/// Canonical topological vertex.
#[derive(Clone, Debug)]
pub struct Vertex {
    point: Point3,
}

impl Vertex {
    /// Returns the exact model-space point.
    pub const fn point(&self) -> &Point3 {
        &self.point
    }
}

/// Canonical topological edge.
#[derive(Clone, Debug)]
pub struct Edge {
    start: VertexId,
    end: VertexId,
    curve: Curve3Id,
    domain: ParameterDomain,
}

impl Edge {
    /// Returns the canonical start vertex.
    pub const fn start(&self) -> VertexId {
        self.start
    }

    /// Returns the canonical end vertex.
    pub const fn end(&self) -> VertexId {
        self.end
    }

    /// Returns the spatial curve.
    pub const fn curve(&self) -> Curve3Id {
        self.curve
    }

    /// Returns the exact edge interval on its spatial curve.
    pub const fn domain(&self) -> &ParameterDomain {
        &self.domain
    }
}

/// One oriented use of an edge by a wire.
#[derive(Clone, Debug)]
pub struct EdgeUse {
    edge: EdgeId,
    direction: Direction,
    pcurve: PcurveId,
    parameter_correspondence: ParameterCorrespondence,
}

impl EdgeUse {
    /// Returns the referenced edge.
    pub const fn edge(&self) -> EdgeId {
        self.edge
    }

    /// Returns the traversal direction.
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// Returns the face-local pcurve.
    pub const fn pcurve(&self) -> PcurveId {
        self.pcurve
    }

    /// Returns the pcurve-to-edge parameter correspondence.
    pub const fn parameter_correspondence(&self) -> &ParameterCorrespondence {
        &self.parameter_correspondence
    }
}

/// Ordered closed boundary.
#[derive(Clone, Debug)]
pub struct Wire {
    edge_uses: Vec<EdgeUseId>,
}

impl Wire {
    /// Returns ordered edge uses.
    pub fn edge_uses(&self) -> &[EdgeUseId] {
        &self.edge_uses
    }
}

/// Boundary representation of one oriented surface region.
#[derive(Clone, Debug)]
enum FaceBoundary {
    /// The complete closed support surface, with no artificial seam topology.
    WholeSurface,
    /// One outer wire and zero or more inner wires.
    Trimmed { outer: WireId, inner: Vec<WireId> },
}

/// Oriented surface region.
#[derive(Clone, Debug)]
pub struct Face {
    surface: SurfaceId,
    orientation: Orientation,
    boundary: FaceBoundary,
}

impl Face {
    /// Returns the support surface.
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    /// Returns face orientation relative to its support surface.
    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// Returns the outer boundary, or `None` for a complete closed surface.
    pub const fn outer(&self) -> Option<WireId> {
        match &self.boundary {
            FaceBoundary::WholeSurface => None,
            FaceBoundary::Trimmed { outer, .. } => Some(*outer),
        }
    }

    /// Returns inner boundaries. Complete closed surfaces have none.
    pub fn inner(&self) -> &[WireId] {
        match &self.boundary {
            FaceBoundary::WholeSurface => &[],
            FaceBoundary::Trimmed { inner, .. } => inner,
        }
    }

    /// Returns whether this face is the complete closed support surface.
    pub const fn is_whole_surface(&self) -> bool {
        matches!(self.boundary, FaceBoundary::WholeSurface)
    }

    fn boundary_wires(&self) -> impl Iterator<Item = &WireId> {
        let outer = match &self.boundary {
            FaceBoundary::WholeSurface => None,
            FaceBoundary::Trimmed { outer, .. } => Some(outer),
        };
        outer.into_iter().chain(self.inner())
    }
}

/// Edge-connected collection of oriented faces.
#[derive(Clone, Debug)]
pub struct Shell {
    faces: Vec<FaceId>,
}

impl Shell {
    /// Returns shell faces.
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }
}

/// Volumetric region bounded by one outer shell.
#[derive(Clone, Debug)]
pub struct Solid {
    outer: ShellId,
    voids: Vec<ShellId>,
}

impl Solid {
    /// Returns the outer shell.
    pub const fn outer(&self) -> ShellId {
        self.outer
    }

    /// Returns void shells.
    pub fn voids(&self) -> &[ShellId] {
        &self.voids
    }
}

/// Counts of canonical model records.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ModelCounts {
    /// Number of vertices.
    pub vertices: usize,
    /// Number of spatial curves.
    pub curves: usize,
    /// Number of pcurves.
    pub pcurves: usize,
    /// Number of surfaces.
    pub surfaces: usize,
    /// Number of edges.
    pub edges: usize,
    /// Number of edge uses.
    pub edge_uses: usize,
    /// Number of wires.
    pub wires: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of shells.
    pub shells: usize,
    /// Number of solids.
    pub solids: usize,
}

/// Immutable validated BREP arena.
#[derive(Clone, Debug)]
pub struct Model {
    data: Arc<ModelData>,
}

/// Transactional copy-on-write model edit.
///
/// Replacement operations may temporarily violate model invariants so related
/// records can be changed together. [`Edit::commit`] replays the entire staged
/// snapshot through [`ModelBuilder`]; failure leaves the source model intact.
#[derive(Clone, Debug)]
pub struct Edit {
    staged: Model,
}

#[derive(Clone, Debug)]
struct ModelData {
    vertices: Vec<Vertex>,
    curves: Vec<Curve3>,
    pcurves: Vec<Pcurve>,
    surfaces: Vec<Surface>,
    edges: Vec<Edge>,
    edge_uses: Vec<EdgeUse>,
    wires: Vec<Wire>,
    faces: Vec<Face>,
    shells: Vec<Shell>,
    solids: Vec<Solid>,
    vertex_edges: Vec<Vec<EdgeId>>,
    edge_uses_by_edge: Vec<Vec<EdgeUseId>>,
    edge_use_wire: Vec<WireId>,
    wire_face: Vec<FaceId>,
    face_shell: Vec<ShellId>,
    shell_solid: Vec<Option<SolidId>>,
    certified_cylinders: Vec<Option<CertifiedCylinderShell>>,
    certified_spheres: Vec<Option<CertifiedSphereShell>>,
    certified_sphere_pairs: Vec<Option<CertifiedSpherePairShell>>,
    certified_cone_frustums: Vec<Option<CertifiedConeFrustumShell>>,
    certified_tori: Vec<Option<CertifiedTorusShell>>,
    certified_revolutions: Vec<Option<CertifiedRevolutionShell>>,
    certified_lofts: Vec<Option<CertifiedLoftShell>>,
    certified_curve_sweeps: Vec<Option<CertifiedCurveSweepShell>>,
    certified_prisms: Vec<Option<CertifiedPrismShell>>,
    bounds: OnceLock<Result<Option<Aabb>, GeometryError>>,
    face_contours: Vec<OnceLock<Result<Vec<Contour2>, GeometryError>>>,
}

#[derive(Clone, Debug)]
struct CertifiedZPrismShell {
    contour: Contour2,
    z_min: Real,
    z_max: Real,
}

#[derive(Clone, Debug)]
struct CertifiedPrismShell {
    outer: Contour2,
    holes: Vec<Contour2>,
    origin: Point3,
    u: Vector3,
    v: Vector3,
    extrusion: Vector3,
    parameter_min: Real,
    parameter_max: Real,
}

#[derive(Clone, Debug)]
struct CertifiedCylinderShell {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
    sphere_subtraction: Option<CertifiedCylinderSphereSubtraction>,
}

#[derive(Clone, Debug)]
enum CertifiedCylinderSphereSubtraction {
    Void {
        center: Point3,
        radius: Real,
    },
    Component {
        center: Point3,
        radius: Real,
        side: CertifiedCylinderSphereComponentSide,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedCylinderSphereComponentSide {
    Lower,
    Upper,
}

#[derive(Clone, Debug)]
struct CertifiedSphereShell {
    center: Point3,
    radius: Real,
    voids: Vec<CertifiedSphereVoid>,
    region: CertifiedSphereRegion,
}

#[derive(Clone, Debug)]
enum CertifiedSphereVoid {
    Sphere { center: Point3, radius: Real },
    Cylinder(Box<CertifiedSphereCylinderVoid>),
}

#[derive(Clone, Debug)]
struct CertifiedSphereCylinderVoid {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
}

#[derive(Clone, Debug)]
enum CertifiedSphereRegion {
    Whole,
    Axial(CertifiedSphereAxialClip),
    Radial(CertifiedSphereRadialClip),
    FiniteCylinder(CertifiedSphereFiniteCylinderRegion),
}

#[derive(Clone, Debug)]
struct CertifiedSphereAxialClip {
    axis: Vector3,
    min: Real,
    max: Real,
}

#[derive(Clone, Debug)]
struct CertifiedSphereRadialClip {
    axis: Vector3,
    radius: Real,
    side: CertifiedSphereRadialSide,
}

#[derive(Clone, Debug)]
struct CertifiedSphereFiniteCylinderRegion {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
    operation: CertifiedSphereFiniteCylinderOperation,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedSphereFiniteCylinderOperation {
    Union,
    Intersection,
    Difference,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedSphereRadialSide {
    Inside,
    Outside,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CertifiedSpherePairKind {
    Union,
    Intersection,
    Difference,
}

#[derive(Clone, Debug)]
struct CertifiedSpherePairShell {
    first_center: Point3,
    first_radius: Real,
    second_center: Point3,
    second_radius: Real,
    kind: CertifiedSpherePairKind,
}

#[derive(Clone, Debug)]
struct CertifiedSphericalCapFace {
    center: Point3,
    axis: Vector3,
    radius: Real,
    latitude: Real,
    upper: bool,
    orientation: Orientation,
}

#[derive(Clone, Debug)]
struct CertifiedConeFrustumShell {
    apex: Point3,
    axis: Vector3,
    semi_angle: Real,
    v_min: Real,
    v_max: Real,
    region: CertifiedConeFrustumRegion,
}

#[derive(Clone, Debug)]
enum CertifiedConeFrustumRegion {
    Whole,
    LongitudinalHalf { interior_normal: Vector3 },
}

#[derive(Clone, Debug)]
struct CertifiedTorusShell {
    center: Point3,
    axis: Vector3,
    major_radius: Real,
    minor_radius: Real,
    region: CertifiedTorusRegion,
}

#[derive(Clone, Debug)]
enum CertifiedTorusRegion {
    Whole,
    Axial { min: Real, max: Real },
    LongitudinalHalf { interior_normal: Vector3 },
}

#[derive(Clone, Debug)]
struct CertifiedRevolutionShell {
    axis_origin: Point3,
    axis: Vector3,
    profile: CertifiedRevolutionBoundary,
    voids: Vec<CertifiedRevolutionBoundary>,
}

#[derive(Clone, Debug)]
enum CertifiedRevolutionBoundary {
    Native(Contour2),
    Curved(CurvePath2),
}

impl CertifiedRevolutionBoundary {
    fn signed_x_first_moment(&self) -> Result<Option<Real>, GeometryError> {
        match self {
            Self::Native(contour) => contour.signed_x_first_moment().map_err(GeometryError::from),
            Self::Curved(path) => path
                .bezier_boundary_loop()
                .map_err(GeometryError::from)?
                .boundary_loop()
                .area_moments()
                .map(|moments| moments.map(|moments| moments.x_moment().clone()))
                .map_err(GeometryError::from),
        }
    }

    fn classify_point(
        &self,
        point: &CurvePoint2,
        policy: &CurvePolicy,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        match self {
            Self::Native(contour) => Ok(contour.classify_point(point, policy)),
            Self::Curved(path) => path
                .classify_point(point, policy)
                .map_err(GeometryError::from),
        }
    }

    fn start(&self) -> &CurvePoint2 {
        match self {
            Self::Native(contour) => contour.segments()[0].start(),
            Self::Curved(path) => path.start(),
        }
    }

    fn as_curve_path(&self) -> Result<CurvePath2, GeometryError> {
        match self {
            Self::Native(contour) => CurvePath2::try_new(
                contour
                    .segments()
                    .iter()
                    .cloned()
                    .map(curve2_from_segment)
                    .collect(),
            )
            .map_err(GeometryError::from),
            Self::Curved(path) => Ok(path.clone()),
        }
    }

    fn intersects(&self, other: &Self, policy: &CurvePolicy) -> Result<bool, GeometryError> {
        if let (Self::Native(first), Self::Native(second)) = (self, other) {
            return first
                .intersect_contour(second, policy)
                .map(|intersections| !intersections.is_empty())
                .map_err(GeometryError::from);
        }
        let result = self
            .as_curve_path()?
            .intersect_path(&other.as_curve_path()?, policy)
            .map_err(GeometryError::from)?;
        if !result.blockers().is_empty() {
            return Err(GeometryError::UnsupportedIntersection);
        }
        Ok(!result.contacts().is_empty() || !result.overlaps().is_empty())
    }

    const fn native_contour(&self) -> Option<&Contour2> {
        match self {
            Self::Native(contour) => Some(contour),
            Self::Curved(_) => None,
        }
    }
}

#[derive(Clone, Debug)]
struct CertifiedLoftShell {
    origin: Point3,
    u: Vector3,
    v: Vector3,
    height_axis: Vector3,
    spans: Vec<CertifiedLoftSpan>,
    parameter_volume: Real,
}

#[derive(Clone, Debug)]
struct CertifiedLoftSpan {
    start: Real,
    end: Real,
    interpolation: CertifiedLoftInterpolation,
}

#[derive(Clone, Debug)]
enum CertifiedLoftInterpolation {
    Homothetic {
        profile: Contour2,
        scale: Real,
        translation: CurvePoint2,
    },
    ConvexCorresponding {
        lower: Vec<CurvePoint2>,
        upper: Vec<CurvePoint2>,
    },
}

#[derive(Clone, Debug)]
struct CertifiedCurveSweepShell {
    profile: Contour2,
    holes: Vec<Contour2>,
    path: Curve3,
    u_path: Curve3,
    v_path: Curve3,
    area_scale_integral: Real,
}

struct TensorPathChain {
    lower: VertexId,
    upper: VertexId,
    curve: Curve3,
    parameter_start: Real,
    parameter_end: Real,
}

pub(crate) struct CertifiedZPrismProfile {
    pub(crate) outer: Contour2,
    pub(crate) holes: Vec<Contour2>,
    pub(crate) z_min: Real,
    pub(crate) z_max: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedSphereProfile {
    pub(crate) center: Point3,
    pub(crate) radius: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedCylinderProfile {
    pub(crate) origin: Point3,
    pub(crate) axis: Vector3,
    pub(crate) radius: Real,
    pub(crate) v_min: Real,
    pub(crate) v_max: Real,
}

impl CertifiedSphereProfile {
    pub(crate) fn strictly_contains_cylinder(
        &self,
        cylinder: &CertifiedCylinderProfile,
    ) -> Result<bool, GeometryError> {
        let offset = &self.center - &cylinder.origin;
        let center_parameter = offset.dot(&cylinder.axis);
        let radial = offset - cylinder.axis.clone() * &center_parameter;
        let radial_distance = radial
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let maximum_radial = radial_distance + &cylinder.radius;
        let lower_height = (&cylinder.v_min - &center_parameter).abs();
        let upper_height = (&cylinder.v_max - center_parameter).abs();
        let maximum_height = if decided_model_order(compare_reals(&lower_height, &upper_height))?
            == std::cmp::Ordering::Greater
        {
            lower_height
        } else {
            upper_height
        };
        Ok(decided_model_order(compare_reals(
            &(&maximum_radial * &maximum_radial + &maximum_height * &maximum_height),
            &(&self.radius * &self.radius),
        ))? == std::cmp::Ordering::Less)
    }
}

impl CertifiedCylinderProfile {
    pub(crate) fn strictly_contains_sphere(
        &self,
        sphere: &CertifiedSphereProfile,
    ) -> Result<bool, GeometryError> {
        let offset = &sphere.center - &self.origin;
        let center_parameter = offset.dot(&self.axis);
        let radial = offset - self.axis.clone() * &center_parameter;
        let radial_distance = radial
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        Ok(decided_model_order(compare_reals(
            &(&radial_distance + &sphere.radius),
            &self.radius,
        ))? == std::cmp::Ordering::Less
            && decided_model_order(compare_reals(
                &self.v_min,
                &(&center_parameter - &sphere.radius),
            ))? == std::cmp::Ordering::Less
            && decided_model_order(compare_reals(
                &(&center_parameter + &sphere.radius),
                &self.v_max,
            ))? == std::cmp::Ordering::Less)
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedConeFrustumProfile {
    pub(crate) apex: Point3,
    pub(crate) axis: Vector3,
    pub(crate) semi_angle: Real,
    pub(crate) v_min: Real,
    pub(crate) v_max: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedRevolutionProfile {
    pub(crate) axis_origin: Point3,
    pub(crate) axis: Vector3,
    pub(crate) profile: Contour2,
    pub(crate) holes: Vec<Contour2>,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedTorusProfile {
    pub(crate) center: Point3,
    pub(crate) axis: Vector3,
    pub(crate) major_radius: Real,
    pub(crate) minor_radius: Real,
}

impl Model {
    /// Returns record counts without scanning the model.
    pub fn counts(&self) -> ModelCounts {
        ModelCounts {
            vertices: self.data.vertices.len(),
            curves: self.data.curves.len(),
            pcurves: self.data.pcurves.len(),
            surfaces: self.data.surfaces.len(),
            edges: self.data.edges.len(),
            edge_uses: self.data.edge_uses.len(),
            wires: self.data.wires.len(),
            faces: self.data.faces.len(),
            shells: self.data.shells.len(),
            solids: self.data.solids.len(),
        }
    }

    /// Starts a transaction over an immutable shared snapshot.
    pub fn edit(&self) -> Edit {
        Edit {
            staged: self.clone(),
        }
    }

    /// Splits one canonical edge and every incident edge use at an exact
    /// interior edge parameter.
    ///
    /// The source edge ID is retained for its first canonical half. A vertex,
    /// second edge, split pcurves, and continuation edge uses are appended.
    /// Every owning wire is updated in traversal order, then the entire
    /// snapshot is replayed through [`ModelBuilder`] before publication.
    pub fn split_edge(
        &self,
        edge_id: EdgeId,
        parameter: Real,
    ) -> Result<(Self, EdgeSplit), TopologyEditError> {
        let edge = self
            .edge(edge_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Edge,
                index: edge_id.index(),
            })?
            .clone();
        if decided_model_order(compare_reals(&parameter, edge.domain.start()))?
            != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&parameter, edge.domain.end()))?
                != std::cmp::Ordering::Less
        {
            return Err(GeometryError::InvalidParameterDomain.into());
        }
        let split_point = self
            .curve(edge.curve)
            .expect("validated edge curve ID")
            .point_at(&parameter)?;
        let vertex = VertexId::from_index(self.data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        let second = EdgeId::from_index(self.data.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        let first_domain = ParameterDomain::new(edge.domain.start().clone(), parameter.clone())?;
        let second_domain = ParameterDomain::new(parameter.clone(), edge.domain.end().clone())?;
        let uses = self
            .uses_of_edge(edge_id)
            .expect("validated edge adjacency")
            .to_vec();

        struct UseSplit {
            use_id: EdgeUseId,
            original_pcurve: PcurveId,
            first_pcurve: Pcurve,
            second_pcurve: Pcurve,
            first_edge: EdgeId,
            second_edge: EdgeId,
            direction: Direction,
            first_correspondence: ParameterCorrespondence,
            second_correspondence: ParameterCorrespondence,
        }

        let mut splits = Vec::with_capacity(uses.len());
        for use_id in &uses {
            let edge_use = self
                .edge_use(*use_id)
                .expect("validated edge-use adjacency");
            let pcurve = self
                .pcurve(edge_use.pcurve)
                .expect("validated edge-use pcurve ID");
            let (first_pcurve, second_pcurve) = match &edge_use.parameter_correspondence {
                ParameterCorrespondence::AngularSweep => {
                    let arc = pcurve
                        .circular_arc()
                        .expect("validated angular-sweep correspondence uses a circular pcurve");
                    let (start, end) = match edge_use.direction {
                        Direction::Forward => (edge.domain.start(), edge.domain.end()),
                        Direction::Reversed => (edge.domain.end(), edge.domain.start()),
                    };
                    let fraction = ((&parameter - start) / (end - start))
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let (first, second) = match arc
                        .split_at_sweep_fraction(&fraction, &CurvePolicy::certified())
                        .map_err(GeometryError::from)?
                    {
                        Classification::Decided(fragments) => fragments,
                        Classification::Uncertain(reason) => {
                            return Err(
                                GeometryError::PlanarClassificationUnresolved(reason).into()
                            );
                        }
                    };
                    (
                        Pcurve::new(Curve2::from(first)),
                        Pcurve::new(Curve2::from(second)),
                    )
                }
                ParameterCorrespondence::Affine { .. } => {
                    let pcurve_parameter = edge_use.parameter_correspondence.pcurve_parameter(
                        pcurve,
                        &edge.domain,
                        edge_use.direction,
                        &parameter,
                    )?;
                    pcurve.split_at(&pcurve_parameter)?
                }
            };
            let (first_edge, first_half_domain, second_edge, second_half_domain) =
                match edge_use.direction {
                    Direction::Forward => (edge_id, &first_domain, second, &second_domain),
                    Direction::Reversed => (second, &second_domain, edge_id, &first_domain),
                };
            let first_correspondence = split_parameter_correspondence(
                &edge_use.parameter_correspondence,
                &first_pcurve,
                first_half_domain,
                edge_use.direction,
            )?;
            let second_correspondence = split_parameter_correspondence(
                &edge_use.parameter_correspondence,
                &second_pcurve,
                second_half_domain,
                edge_use.direction,
            )?;
            splits.push(UseSplit {
                use_id: *use_id,
                original_pcurve: edge_use.pcurve,
                first_pcurve,
                second_pcurve,
                first_edge,
                second_edge,
                direction: edge_use.direction,
                first_correspondence,
                second_correspondence,
            });
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        data.vertices.push(Vertex { point: split_point });
        data.edges[edge_id.index()] = Edge {
            start: edge.start,
            end: vertex,
            curve: edge.curve,
            domain: first_domain,
        };
        data.edges.push(Edge {
            start: vertex,
            end: edge.end,
            curve: edge.curve,
            domain: second_domain,
        });

        let mut pcurve_use_counts = vec![0_usize; data.pcurves.len()];
        for edge_use in &data.edge_uses {
            pcurve_use_counts[edge_use.pcurve.index()] += 1;
        }
        let mut split_uses = Vec::with_capacity(splits.len());
        for split in splits {
            let first_pcurve_id = if pcurve_use_counts[split.original_pcurve.index()] == 1 {
                data.pcurves[split.original_pcurve.index()] = split.first_pcurve;
                split.original_pcurve
            } else {
                let id = PcurveId::from_index(data.pcurves.len())
                    .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
                data.pcurves.push(split.first_pcurve);
                id
            };
            let second_pcurve_id = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(split.second_pcurve);
            data.edge_uses[split.use_id.index()] = EdgeUse {
                edge: split.first_edge,
                direction: split.direction,
                pcurve: first_pcurve_id,
                parameter_correspondence: split.first_correspondence,
            };
            let continuation = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge: split.second_edge,
                direction: split.direction,
                pcurve: second_pcurve_id,
                parameter_correspondence: split.second_correspondence,
            });
            split_uses.push((split.use_id, continuation));
        }
        for wire in &mut data.wires {
            let mut expanded = Vec::with_capacity(wire.edge_uses.len() + split_uses.len());
            for edge_use in &wire.edge_uses {
                expanded.push(*edge_use);
                if let Some((_, continuation)) =
                    split_uses.iter().find(|(source, _)| source == edge_use)
                {
                    expanded.push(*continuation);
                }
            }
            wire.edge_uses = expanded;
        }
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            EdgeSplit {
                vertex,
                first: edge_id,
                second,
                edge_uses: split_uses,
            },
        ))
    }

    /// Splits a trimmed planar face along one exact straight curve fragment.
    ///
    /// Each fragment endpoint is attached by mathematical identity: an
    /// existing outer-boundary vertex is reused, otherwise the unique
    /// containing canonical edge is split at its exact represented parameter.
    /// The resulting boundary vertices are then passed through
    /// [`Model::split_face`], so the chord is shared by identity and the
    /// complete model is revalidated before publication.
    pub fn split_face_by_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
    ) -> Result<(Self, CurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let Some(_) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let surface = self
            .surface(face.surface)
            .expect("validated face surface ID");
        if surface.kind() != SurfaceKind::Plane {
            return Err(TopologyEditError::UnsupportedFaceSplitSurface(
                surface.kind(),
            ));
        }
        if curve.kind() != Curve3Kind::Line {
            return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
        }

        let start = curve.start()?;
        let end = curve.end()?;
        let (staged, start_vertex, start_edge) =
            self.attach_face_split_endpoint(face_id, Endpoint::Start, &start)?;
        let (staged, end_vertex, end_edge) =
            staged.attach_face_split_endpoint(face_id, Endpoint::End, &end)?;
        let (staged, face) = staged.split_face(face_id, start_vertex, end_vertex)?;
        Ok((
            staged,
            CurveFaceSplit {
                start_edge,
                end_edge,
                face,
            },
        ))
    }

    /// Splits a face along one retained exact surface curve.
    ///
    /// The supplied pcurve is materialized without inverse fitting and retains
    /// its exact affine correspondence to the spatial curve parameter. Curve
    /// endpoints on a trimmed face attach to existing outer-boundary vertices
    /// or split the unique containing canonical edges. A closed latitude on a
    /// boundaryless whole sphere authors two complementary periodic caps. The
    /// new canonical curve is shared by two opposite edge uses, and the
    /// complete model is revalidated before publication.
    pub fn split_face_by_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
    ) -> Result<(Self, SurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let start = curve.start()?;
        let end = curve.end()?;
        if exact_point_order(&start, &end)? == std::cmp::Ordering::Equal {
            let (canonical_curve, canonical_pcurve);
            let reversed = curve.reversed()?;
            let (curve, pcurve) =
                if compare_curve3_exact_data(&curve.exact_data(), &reversed.exact_data())?
                    == std::cmp::Ordering::Greater
                {
                    canonical_curve = curve.reversed()?;
                    canonical_pcurve = pcurve.reversed()?;
                    (&canonical_curve, &canonical_pcurve)
                } else {
                    (curve, pcurve)
                };
            let (model, split) = if face.outer().is_none() {
                self.split_whole_sphere_by_surface_curve(face_id, curve, pcurve)?
            } else if self
                .surface(face.surface)
                .expect("validated face surface")
                .kind()
                == SurfaceKind::Sphere
                && (face.inner().is_empty()
                    && self.is_spherical_latitude_wire(
                        face.outer().expect("trimmed face carries outer wire"),
                    )?
                    || !face.inner().is_empty())
            {
                self.split_spherical_cap_by_surface_curve(face_id, curve, pcurve)?
            } else {
                self.split_face_by_closed_surface_curve(face_id, curve, pcurve)?
            };
            return Ok((model, SurfaceCurveFaceSplit::Closed(split)));
        }
        let Some(_) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let materialized = pcurve.materialize()?;
        let correspondence = match materialized.correspondence() {
            SurfacePcurveCorrespondence::Affine { scale, offset } => {
                ParameterCorrespondence::affine(scale.clone(), offset.clone())?
            }
            SurfacePcurveCorrespondence::AngularSweep { .. } => {
                ParameterCorrespondence::angular_sweep()
            }
        };
        let (staged, start_vertex, start_edge) =
            self.attach_face_split_endpoint(face_id, Endpoint::Start, &start)?;
        let (staged, end_vertex, end_edge) =
            staged.attach_face_split_endpoint(face_id, Endpoint::End, &end)?;
        let (staged, face) = staged.split_face_with_geometry(
            face_id,
            start_vertex,
            end_vertex,
            Some((
                curve.clone(),
                Pcurve::new(materialized.curve().clone()),
                correspondence,
            )),
        )?;
        Ok((
            staged,
            SurfaceCurveFaceSplit::Open(CurveFaceSplit {
                start_edge,
                end_edge,
                face,
            }),
        ))
    }

    fn split_whole_sphere_by_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        if !face.is_whole_surface()
            || self
                .surface(face.surface)
                .expect("validated whole-face surface")
                .kind()
                != SurfaceKind::Sphere
        {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        }
        let start = curve.start()?;
        let end = curve.end()?;
        if exact_point_order(&start, &end)? != std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        if exact_point_order(&start, &midpoint_point)? == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }

        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start() + forward_pcurve.domain_end())
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });

        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
            });
            edges.push(edge);

            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
            });
            forward_uses.push(forward_use);

            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let first_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: forward_uses.clone(),
        });
        let second_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: reverse_uses.clone(),
        });
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer: first_wire,
            inner: Vec::new(),
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: second_wire,
                inner: Vec::new(),
            },
        });
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [forward_uses[0], forward_uses[1]],
                second_edge_uses: [reverse_uses[0], reverse_uses[1]],
                first_wire,
                second_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    fn is_spherical_latitude_wire(&self, wire: WireId) -> Result<bool, TopologyEditError> {
        let wire = self.wire(wire).expect("validated spherical wire");
        let mut latitude = None;
        for edge_use_id in &wire.edge_uses {
            let edge_use = self
                .edge_use(*edge_use_id)
                .expect("validated spherical edge use");
            let Some(line) = self
                .pcurve(edge_use.pcurve)
                .expect("validated spherical pcurve")
                .line_segment()
            else {
                return Ok(false);
            };
            if !real_values_equal(line.start().y(), line.end().y())? {
                return Ok(false);
            }
            if let Some(expected) = &latitude {
                if !real_values_equal(line.start().y(), expected)? {
                    return Ok(false);
                }
            } else {
                latitude = Some(line.start().y().clone());
            }
        }
        Ok(latitude.is_some())
    }

    fn split_spherical_cap_by_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        if !inner.is_empty() {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }
        let old_wire = self.wire(*outer).expect("validated spherical cap wire");
        let old_line = self
            .pcurve(
                self.edge_use(old_wire.edge_uses[0])
                    .expect("validated spherical use")
                    .pcurve,
            )
            .expect("validated spherical pcurve")
            .line_segment()
            .expect("validated spherical latitude");
        let old_latitude = old_line.start().y().clone();
        let increasing =
            decided_model_order(compare_reals(old_line.end().x(), old_line.start().x()))?
                == std::cmp::Ordering::Greater;
        let upper = match face.orientation {
            Orientation::Forward => increasing,
            Orientation::Reversed => !increasing,
        };
        let materialized = pcurve.materialize()?;
        if materialized.curve().family() != CurveFamily2::Line {
            return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
        }
        let new_start = materialized.curve().start();
        let new_end = materialized.curve().end();
        if !real_values_equal(new_start.y(), new_end.y())? {
            return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
        }
        let new_latitude = new_start.y().clone();
        let relation = decided_model_order(compare_reals(&new_latitude, &old_latitude))?;
        if (upper && relation != std::cmp::Ordering::Greater)
            || (!upper && relation != std::cmp::Ordering::Less)
        {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }

        let start = curve.start()?;
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start() + forward_pcurve.domain_end())
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });
        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
            });
            edges.push(edge);
            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
            });
            forward_uses.push(forward_use);
            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let forward_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: forward_uses.clone(),
        });
        let reverse_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: reverse_uses.clone(),
        });

        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        if upper {
            data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
                outer: *outer,
                inner: vec![reverse_wire],
            };
            data.faces.push(Face {
                surface: face.surface,
                orientation: face.orientation,
                boundary: FaceBoundary::Trimmed {
                    outer: forward_wire,
                    inner: Vec::new(),
                },
            });
        } else {
            data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
                outer: reverse_wire,
                inner: Vec::new(),
            };
            data.faces.push(Face {
                surface: face.surface,
                orientation: face.orientation,
                boundary: FaceBoundary::Trimmed {
                    outer: forward_wire,
                    inner: vec![*outer],
                },
            });
        }
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [reverse_uses[0], reverse_uses[1]],
                second_edge_uses: [forward_uses[0], forward_uses[1]],
                first_wire: reverse_wire,
                second_wire: forward_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    fn split_face_by_closed_surface_curve(
        &self,
        face_id: FaceId,
        curve: &Curve3,
        pcurve: &SurfaceIntersectionPcurve,
    ) -> Result<(Self, ClosedSurfaceCurveFaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let start = curve.start()?;
        let end = curve.end()?;
        if exact_point_order(&start, &end)? != std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let midpoint = ((curve.domain().start() + curve.domain().end()) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let midpoint_point = curve.point_at(&midpoint)?;
        if exact_point_order(&start, &midpoint_point)? == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }

        let materialized_loop = pcurve.materialize()?;
        let loop_path = CurvePath2::try_new(vec![materialized_loop.curve().clone()])
            .map_err(GeometryError::from)?;
        let loop_area = loop_path
            .bezier_boundary_loop()
            .map_err(GeometryError::from)?
            .boundary_loop()
            .signed_area()
            .map_err(GeometryError::from)?;
        let loop_area = loop_area.ok_or(GeometryError::UnsupportedPcurveContour)?;
        let area_order = decided_model_order(compare_reals(&loop_area, &Real::zero()))?;
        if area_order == std::cmp::Ordering::Equal {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let expected_outer_order = match face.orientation {
            Orientation::Forward => std::cmp::Ordering::Greater,
            Orientation::Reversed => std::cmp::Ordering::Less,
        };
        let interior_forward = area_order == expected_outer_order;

        let policy = CurvePolicy::certified();
        let loop_start = materialized_loop.curve().start();
        let classify = |path: &CurvePath2,
                        point: &CurvePoint2|
         -> Result<ContourPointLocation, TopologyEditError> {
            match path
                .classify_point(point, &policy)
                .map_err(GeometryError::from)?
            {
                Classification::Decided(location) => Ok(location),
                Classification::Uncertain(reason) => {
                    Err(GeometryError::PlanarClassificationUnresolved(reason).into())
                }
            }
        };
        let outer_path = self.build_model_wire_curve_path(*outer)?;
        if classify(&outer_path, loop_start)? != ContourPointLocation::Inside {
            return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
        }
        for hole in inner {
            let hole_path = self.build_model_wire_curve_path(*hole)?;
            if classify(&hole_path, loop_start)? != ContourPointLocation::Outside {
                return Err(TopologyEditError::ClosedFaceSplitNotInMaterial { face: face_id });
            }
        }

        let ranges = [
            (curve.domain().start(), &midpoint),
            (&midpoint, curve.domain().end()),
        ];
        let mut halves = Vec::with_capacity(2);
        for (range_start, range_end) in ranges {
            let spatial = curve.subcurve(range_start, range_end)?;
            let retained = pcurve.subcurve(range_start, range_end)?;
            let materialized = retained.materialize()?;
            let forward_pcurve = Pcurve::new(materialized.curve().clone());
            let reverse_pcurve = forward_pcurve.reversed()?;
            let (forward_correspondence, reverse_correspondence) = match materialized
                .correspondence()
            {
                SurfacePcurveCorrespondence::Affine { scale, offset } => {
                    let source_span = range_end - range_start;
                    let spatial_span = spatial.domain().end() - spatial.domain().start();
                    let domain_scale = (spatial_span / source_span)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let edge_scale = &domain_scale * scale;
                    let edge_offset =
                        spatial.domain().start() + &domain_scale * (offset - range_start);
                    (
                        ParameterCorrespondence::affine(edge_scale.clone(), edge_offset.clone())?,
                        ParameterCorrespondence::affine(
                            -edge_scale.clone(),
                            edge_scale
                                * (forward_pcurve.domain_start() + forward_pcurve.domain_end())
                                + edge_offset,
                        )?,
                    )
                }
                SurfacePcurveCorrespondence::AngularSweep { .. } => (
                    ParameterCorrespondence::angular_sweep(),
                    ParameterCorrespondence::angular_sweep(),
                ),
            };
            halves.push((
                spatial,
                forward_pcurve,
                forward_correspondence,
                reverse_pcurve,
                reverse_correspondence,
            ));
        }

        let mut enclosed_inner = Vec::new();
        let mut exterior_inner = Vec::new();
        for hole in inner {
            let hole_path = self.build_model_wire_curve_path(*hole)?;
            let representative = hole_path.start();
            match classify(&loop_path, representative)? {
                ContourPointLocation::Inside => enclosed_inner.push(*hole),
                ContourPointLocation::Outside => exterior_inner.push(*hole),
                ContourPointLocation::Boundary => {
                    return Err(BuildError::IntersectingFaceWires {
                        first: *outer,
                        second: *hole,
                    }
                    .into());
                }
            }
        }

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let seam_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: start.clone(),
        });
        let midpoint_vertex = VertexId::from_index(data.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        data.vertices.push(Vertex {
            point: midpoint_point,
        });

        let mut edges = Vec::with_capacity(2);
        let mut forward_uses = Vec::with_capacity(2);
        let mut reverse_uses = Vec::with_capacity(2);
        for (index, (spatial, forward, forward_map, reverse, reverse_map)) in
            halves.into_iter().enumerate()
        {
            let curve_id = Curve3Id::from_index(data.curves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
            let domain = spatial.domain().clone();
            data.curves.push(spatial);
            let edge = EdgeId::from_index(data.edges.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
            data.edges.push(Edge {
                start: if index == 0 {
                    seam_vertex
                } else {
                    midpoint_vertex
                },
                end: if index == 0 {
                    midpoint_vertex
                } else {
                    seam_vertex
                },
                curve: curve_id,
                domain,
            });
            edges.push(edge);

            let forward_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(forward);
            let forward_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Forward,
                pcurve: forward_pcurve,
                parameter_correspondence: forward_map,
            });
            forward_uses.push(forward_use);

            let reverse_pcurve = PcurveId::from_index(data.pcurves.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
            data.pcurves.push(reverse);
            let reverse_use = EdgeUseId::from_index(data.edge_uses.len())
                .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
            data.edge_uses.push(EdgeUse {
                edge,
                direction: Direction::Reversed,
                pcurve: reverse_pcurve,
                parameter_correspondence: reverse_map,
            });
            reverse_uses.push(reverse_use);
        }
        reverse_uses.reverse();
        let (interior_uses, exterior_uses) = if interior_forward {
            (forward_uses, reverse_uses)
        } else {
            (reverse_uses, forward_uses)
        };

        let exterior_inner_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: exterior_uses.clone(),
        });
        let interior_outer_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: interior_uses.clone(),
        });
        exterior_inner.push(exterior_inner_wire);
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer: *outer,
            inner: exterior_inner,
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: interior_outer_wire,
                inner: enclosed_inner,
            },
        });
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            ClosedSurfaceCurveFaceSplit {
                seam_vertex,
                midpoint_vertex,
                edges: [edges[0], edges[1]],
                first_edge_uses: [exterior_uses[0], exterior_uses[1]],
                second_edge_uses: [interior_uses[0], interior_uses[1]],
                first_wire: exterior_inner_wire,
                second_wire: interior_outer_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    /// Deterministically partitions one face by retained exact
    /// surface-intersection curves.
    ///
    /// Curves are ordered by their unordered spatial endpoint pairs, so caller
    /// order does not affect the published topology. After each split, both
    /// endpoints of every remaining curve must belong to exactly one current
    /// descendant. The selected operand pcurve is materialized without inverse
    /// fitting, and every new edge remains identity-shared by its descendants.
    pub fn split_face_by_surface_curves(
        &self,
        face_id: FaceId,
        curves: &[SurfaceIntersectionCurve],
        operand: SurfaceIntersectionOperand,
    ) -> Result<(Self, FacePartition), TopologyEditError> {
        self.face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let mut ordered = Vec::with_capacity(curves.len());
        for (source_index, intersection) in curves.iter().enumerate() {
            let reversed = intersection.reversed()?;
            let intersection = if compare_curve3_exact_data(
                &intersection.curve().exact_data(),
                &reversed.curve().exact_data(),
            )? == std::cmp::Ordering::Greater
            {
                reversed
            } else {
                intersection.clone()
            };
            let start = intersection.curve().start()?;
            let end = intersection.curve().end()?;
            let (lower, upper) = if exact_point_order(&start, &end)? == std::cmp::Ordering::Greater
            {
                (end, start)
            } else {
                (start, end)
            };
            let trace = OrderedSurfaceCurveTrace {
                source_index,
                exact_key: intersection.curve().clone(),
                intersection,
                lower,
                upper,
            };
            let mut insertion = ordered.len();
            while insertion > 0
                && compare_ordered_surface_curve_traces(&trace, &ordered[insertion - 1])?
                    == std::cmp::Ordering::Less
            {
                insertion -= 1;
            }
            if insertion > 0
                && compare_ordered_surface_curve_traces(&trace, &ordered[insertion - 1])?
                    == std::cmp::Ordering::Equal
            {
                return Err(TopologyEditError::DuplicateFaceSplitTrace {
                    first: ordered[insertion - 1].source_index,
                    second: source_index,
                });
            }
            if insertion < ordered.len()
                && compare_ordered_surface_curve_traces(&trace, &ordered[insertion])?
                    == std::cmp::Ordering::Equal
            {
                return Err(TopologyEditError::DuplicateFaceSplitTrace {
                    first: ordered[insertion].source_index,
                    second: source_index,
                });
            }
            ordered.insert(insertion, trace);
        }

        let mut split_parameters = ordered
            .iter()
            .map(|trace| {
                vec![
                    trace.intersection.curve().domain().start().clone(),
                    trace.intersection.curve().domain().end().clone(),
                ]
            })
            .collect::<Vec<_>>();
        let materialized = ordered
            .iter()
            .map(|trace| trace.intersection.pcurve(operand).materialize())
            .collect::<Result<Vec<_>, _>>()?;
        for first_index in 0..ordered.len() {
            for second_index in (first_index + 1)..ordered.len() {
                let relation = materialized[first_index]
                    .curve()
                    .intersect_curve(
                        materialized[second_index].curve(),
                        &CurvePolicy::certified(),
                    )
                    .map_err(GeometryError::from)?;
                if !relation.is_complete() {
                    return Err(GeometryError::UnsupportedIntersection.into());
                }
                if !relation.overlaps().is_empty() {
                    return Err(TopologyEditError::OverlappingFaceSplitTraces {
                        first: ordered[first_index].source_index,
                        second: ordered[second_index].source_index,
                    });
                }
                for contact in relation.contacts() {
                    let Some(first_parameter) = contact.first().exact_curve_parameter() else {
                        return Err(GeometryError::UnsupportedIntersection.into());
                    };
                    let Some(second_parameter) = contact.second().exact_curve_parameter() else {
                        return Err(GeometryError::UnsupportedIntersection.into());
                    };
                    let first_spatial =
                        materialized[first_index].spatial_parameter_at(&first_parameter)?;
                    if !ordered[first_index]
                        .intersection
                        .curve()
                        .domain()
                        .contains(&first_spatial)?
                    {
                        return Err(GeometryError::UnsupportedIntersection.into());
                    }
                    insert_exact_split_parameter(
                        &mut split_parameters[second_index],
                        materialized[second_index].spatial_parameter_at(&second_parameter)?,
                    )?;
                }
            }
        }

        let mut staged = self.clone();
        let mut faces = vec![face_id];
        let mut traces = Vec::with_capacity(ordered.len());
        for (trace, parameters) in ordered.into_iter().zip(split_parameters) {
            let segments = parameters
                .windows(2)
                .map(|range| trace.intersection.subcurve(&range[0], &range[1]))
                .collect::<Result<Vec<_>, _>>()?;
            let mut splits = Vec::with_capacity(segments.len());
            let whole_closed_trace = segments.len() == 1
                && exact_point_order(&segments[0].curve().start()?, &segments[0].curve().end()?)?
                    == std::cmp::Ordering::Equal;
            if whole_closed_trace {
                let segment = &segments[0];
                let mut candidates = Vec::new();
                for (index, candidate) in faces.iter().copied().enumerate() {
                    match staged.split_face_by_surface_curve(
                        candidate,
                        segment.curve(),
                        segment.pcurve(operand),
                    ) {
                        Ok(result) => candidates.push((index, result)),
                        Err(TopologyEditError::ClosedFaceSplitNotInMaterial { .. }) => {}
                        Err(error) => return Err(error),
                    }
                }
                let (face_index, (next, split)) = match candidates.as_slice() {
                    [(index, result)] => (*index, result.clone()),
                    [] => {
                        return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                            face: face_id,
                            trace: trace.source_index,
                            segment: 0,
                        });
                    }
                    _ => {
                        return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                            face: face_id,
                            trace: trace.source_index,
                            segment: 0,
                        });
                    }
                };
                staged = next;
                faces[face_index] = split.first_face();
                faces.insert(face_index + 1, split.second_face());
                splits.push(split);
            }
            for (segment_index, segment) in segments.iter().enumerate() {
                if whole_closed_trace {
                    break;
                }
                let curve = segment.curve();
                let start = curve.start()?;
                let end = curve.end()?;
                let mut candidates = Vec::new();
                for (index, candidate) in faces.iter().copied().enumerate() {
                    let start_location =
                        staged.locate_face_split_endpoint(candidate, Endpoint::Start, &start);
                    let end_location =
                        staged.locate_face_split_endpoint(candidate, Endpoint::End, &end);
                    match (start_location, end_location) {
                        (Ok(_), Ok(_)) => candidates.push((index, candidate)),
                        (Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. }), _)
                        | (_, Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. })) =>
                            {}
                        (Err(error), _) | (_, Err(error)) => return Err(error),
                    }
                }
                if candidates.len() > 1 {
                    let midpoint = ((curve.domain().start() + curve.domain().end())
                        / Real::from(2))
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let parameter = segment.pcurve(operand).point_at(&midpoint)?;
                    let witness = CurvePoint2::new(parameter.x, parameter.y);
                    let mut interior_candidates = Vec::with_capacity(candidates.len());
                    for (index, candidate) in candidates {
                        match staged.classify_surface_parameter_on_face(candidate, &witness)? {
                            Classification::Decided(ContourPointLocation::Inside) => {
                                interior_candidates.push((index, candidate));
                            }
                            Classification::Decided(ContourPointLocation::Outside) => {}
                            Classification::Decided(ContourPointLocation::Boundary) => {
                                return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                                    face: face_id,
                                    trace: trace.source_index,
                                    segment: segment_index,
                                });
                            }
                            Classification::Uncertain(reason) => {
                                return Err(
                                    GeometryError::PlanarClassificationUnresolved(reason).into()
                                );
                            }
                        }
                    }
                    candidates = interior_candidates;
                }
                let (face_index, candidate) = match candidates.as_slice() {
                    [(index, candidate)] => (*index, *candidate),
                    [] => {
                        return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                            face: face_id,
                            trace: trace.source_index,
                            segment: segment_index,
                        });
                    }
                    _ => {
                        return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                            face: face_id,
                            trace: trace.source_index,
                            segment: segment_index,
                        });
                    }
                };
                let (next, split) = staged.split_face_by_surface_curve(
                    candidate,
                    curve,
                    segment.pcurve(operand),
                )?;
                staged = next;
                faces[face_index] = split.first_face();
                faces.insert(face_index + 1, split.second_face());
                splits.push(split);
            }
            traces.push(FaceTracePartition {
                source_index: trace.source_index,
                segments: segments
                    .into_iter()
                    .map(|segment| segment.curve().clone())
                    .collect(),
                splits,
            });
        }
        Ok((
            staged,
            FacePartition {
                source_face: face_id,
                faces,
                traces,
            },
        ))
    }

    /// Deterministically partitions one planar face by exact line fragments.
    ///
    /// Trace direction and caller order do not affect the resulting topology:
    /// unordered endpoint pairs are sorted by certified lexicographic point
    /// order. Every later trace is subdivided at exact crossings with earlier
    /// traces, then every arranged segment must attach to exactly one current
    /// descendant face. Duplicates, positive-length overlaps, and ambiguous
    /// descendant attachment are typed errors.
    pub fn split_face_by_curves(
        &self,
        face_id: FaceId,
        curves: &[Curve3],
    ) -> Result<(Self, FacePartition), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?;
        let Some(_) = face.outer() else {
            return Err(TopologyEditError::WholeSurfaceFace(face_id));
        };
        let surface = self
            .surface(face.surface)
            .expect("validated face surface ID");
        if surface.kind() != SurfaceKind::Plane {
            return Err(TopologyEditError::UnsupportedFaceSplitSurface(
                surface.kind(),
            ));
        }

        let mut traces = Vec::with_capacity(curves.len());
        for (source_index, curve) in curves.iter().enumerate() {
            if curve.kind() != Curve3Kind::Line {
                return Err(TopologyEditError::UnsupportedFaceSplitCurve(curve.kind()));
            }
            let start = curve.start()?;
            let end = curve.end()?;
            let (lower, upper) = if exact_point_order(&start, &end)? == std::cmp::Ordering::Greater
            {
                (end, start)
            } else {
                (start, end)
            };
            let trace = OrderedFaceSplitTrace {
                source_index,
                curve: Curve3::line(lower.clone(), upper.clone())?,
                lower,
                upper,
            };
            let mut insertion = traces.len();
            while insertion > 0
                && compare_ordered_face_split_traces(&trace, &traces[insertion - 1])?
                    == std::cmp::Ordering::Less
            {
                insertion -= 1;
            }
            if insertion > 0
                && compare_ordered_face_split_traces(&trace, &traces[insertion - 1])?
                    == std::cmp::Ordering::Equal
            {
                return Err(TopologyEditError::DuplicateFaceSplitTrace {
                    first: traces[insertion - 1].source_index,
                    second: source_index,
                });
            }
            if insertion < traces.len()
                && compare_ordered_face_split_traces(&trace, &traces[insertion])?
                    == std::cmp::Ordering::Equal
            {
                return Err(TopologyEditError::DuplicateFaceSplitTrace {
                    first: traces[insertion].source_index,
                    second: source_index,
                });
            }
            traces.insert(insertion, trace);
        }

        let mut staged = self.clone();
        let mut faces = vec![face_id];
        let mut prior_lines = Vec::with_capacity(traces.len());
        let mut partitioned_traces = Vec::with_capacity(traces.len());
        for trace in traces {
            let planar = projected_face_split_line(surface, &trace)?;
            let segments = arranged_face_split_segments(&trace, &planar, &prior_lines)?;
            let mut trace_splits = Vec::with_capacity(segments.len());
            for (segment_index, segment) in segments.iter().enumerate() {
                let start = segment.start()?;
                let end = segment.end()?;
                let mut candidates = Vec::new();
                for (index, candidate) in faces.iter().copied().enumerate() {
                    let start_location =
                        staged.locate_face_split_endpoint(candidate, Endpoint::Start, &start);
                    let end_location =
                        staged.locate_face_split_endpoint(candidate, Endpoint::End, &end);
                    match (start_location, end_location) {
                        (Ok(_), Ok(_)) => candidates.push((index, candidate)),
                        (Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. }), _)
                        | (_, Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary { .. })) =>
                            {}
                        (Err(error), _) | (_, Err(error)) => return Err(error),
                    }
                }
                let (face_index, candidate) = match candidates.as_slice() {
                    [(index, candidate)] => (*index, *candidate),
                    [] => {
                        return Err(TopologyEditError::FaceSplitTraceNotInSingleRegion {
                            face: face_id,
                            trace: trace.source_index,
                            segment: segment_index,
                        });
                    }
                    _ => {
                        return Err(TopologyEditError::FaceSplitTraceAmbiguous {
                            face: face_id,
                            trace: trace.source_index,
                            segment: segment_index,
                        });
                    }
                };
                let (next, split) = staged.split_face_by_curve(candidate, segment)?;
                staged = next;
                faces[face_index] = split.face.first_face;
                faces.insert(face_index + 1, split.face.second_face);
                trace_splits.push(SurfaceCurveFaceSplit::Open(split));
            }
            prior_lines.push((trace.source_index, planar));
            partitioned_traces.push(FaceTracePartition {
                source_index: trace.source_index,
                segments,
                splits: trace_splits,
            });
        }
        Ok((
            staged,
            FacePartition {
                source_face: face_id,
                faces,
                traces: partitioned_traces,
            },
        ))
    }

    fn attach_face_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<(Self, VertexId, Option<EdgeSplit>), TopologyEditError> {
        match self.locate_face_split_endpoint(face_id, endpoint, point)? {
            FaceSplitEndpointLocation::Vertex(vertex) => Ok((self.clone(), vertex, None)),
            FaceSplitEndpointLocation::Edge { edge, parameter } => {
                let (staged, split) = self.split_edge(edge, parameter)?;
                Ok((staged, split.vertex, Some(split)))
            }
        }
    }

    fn locate_face_split_endpoint(
        &self,
        face_id: FaceId,
        endpoint: Endpoint,
        point: &Point3,
    ) -> Result<FaceSplitEndpointLocation, TopologyEditError> {
        let face = self
            .face(face_id)
            .expect("curve-driven split prevalidates the face");
        let outer = face
            .outer()
            .expect("curve-driven split prevalidates a trimmed face");
        let wire = self.wire(outer).expect("validated face outer wire");
        let mut boundary_vertices = Vec::new();
        let mut boundary_edges = Vec::new();
        for use_id in wire.edge_uses() {
            let edge_use = self.edge_use(*use_id).expect("validated edge use");
            let edge = self.edge(edge_use.edge).expect("validated canonical edge");
            for vertex in [edge.start, edge.end] {
                if !boundary_vertices.contains(&vertex) {
                    boundary_vertices.push(vertex);
                }
            }
            if !boundary_edges.contains(&edge_use.edge) {
                boundary_edges.push(edge_use.edge);
            }
        }
        let mut matching_vertices = Vec::new();
        for vertex in boundary_vertices {
            if points_equal(
                &self
                    .vertex(vertex)
                    .expect("validated boundary vertex")
                    .point,
                point,
            )
            .map_err(TopologyEditError::Build)?
            {
                matching_vertices.push(vertex);
            }
        }
        match matching_vertices.as_slice() {
            [vertex] => return Ok(FaceSplitEndpointLocation::Vertex(*vertex)),
            [] => {}
            _ => {
                return Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                    face: face_id,
                    endpoint,
                });
            }
        }

        let mut locations = Vec::new();
        for edge_id in boundary_edges {
            let edge = self.edge(edge_id).expect("validated canonical edge");
            let curve = self.curve(edge.curve).expect("validated edge curve");
            let CurveParameterLocation::Parameters(parameters) = curve.parameters_of(point)? else {
                continue;
            };
            for parameter in parameters {
                if edge.domain.contains(&parameter)?
                    && decided_model_order(compare_reals(&parameter, edge.domain.start()))?
                        == std::cmp::Ordering::Greater
                    && decided_model_order(compare_reals(&parameter, edge.domain.end()))?
                        == std::cmp::Ordering::Less
                {
                    locations.push((edge_id, parameter));
                }
            }
        }
        match locations.len() {
            0 => Err(TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                face: face_id,
                endpoint,
            }),
            1 => {
                let (edge, parameter) = locations.pop().expect("one exact edge location");
                Ok(FaceSplitEndpointLocation::Edge { edge, parameter })
            }
            _ => Err(TopologyEditError::FaceSplitEndpointAmbiguous {
                face: face_id,
                endpoint,
            }),
        }
    }

    /// Splits one trimmed planar face between two nonadjacent outer-boundary
    /// vertices with an exact straight chord.
    ///
    /// The source face and outer-wire IDs are retained for the first result.
    /// The chord is one canonical edge with two opposite face-local uses.
    /// Existing holes are assigned to exactly one result by certified contour
    /// classification. The owning shell is updated, then the complete snapshot
    /// is replayed through [`ModelBuilder`] before publication.
    pub fn split_face(
        &self,
        face_id: FaceId,
        start: VertexId,
        end: VertexId,
    ) -> Result<(Self, FaceSplit), TopologyEditError> {
        self.split_face_with_geometry(face_id, start, end, None)
    }

    fn split_face_with_geometry(
        &self,
        face_id: FaceId,
        start: VertexId,
        end: VertexId,
        authored: Option<(Curve3, Pcurve, ParameterCorrespondence)>,
    ) -> Result<(Self, FaceSplit), TopologyEditError> {
        let face = self
            .face(face_id)
            .ok_or(TopologyEditError::InvalidReference {
                kind: EntityKind::Face,
                index: face_id.index(),
            })?
            .clone();
        for vertex in [start, end] {
            if self.vertex(vertex).is_none() {
                return Err(TopologyEditError::InvalidReference {
                    kind: EntityKind::Vertex,
                    index: vertex.index(),
                });
            }
        }
        let (outer, inner) = match face.boundary {
            FaceBoundary::WholeSurface => {
                return Err(TopologyEditError::WholeSurfaceFace(face_id));
            }
            FaceBoundary::Trimmed { outer, inner } => (outer, inner),
        };
        let surface = self
            .surface(face.surface)
            .expect("validated face surface ID");
        let authored_curve = authored.is_some();
        if authored.is_none() && surface.kind() != SurfaceKind::Plane {
            return Err(TopologyEditError::UnsupportedFaceSplitSurface(
                surface.kind(),
            ));
        }
        let outer_uses = self
            .wire(outer)
            .expect("validated face outer-wire ID")
            .edge_uses
            .clone();
        let directed_start = |edge_use_id: EdgeUseId| {
            let edge_use = self
                .edge_use(edge_use_id)
                .expect("validated wire edge-use ID");
            let edge = self
                .edge(edge_use.edge)
                .expect("validated edge-use edge ID");
            match edge_use.direction {
                Direction::Forward => edge.start,
                Direction::Reversed => edge.end,
            }
        };
        let unique_boundary_index = |vertex: VertexId| -> Result<usize, TopologyEditError> {
            let matches = outer_uses
                .iter()
                .enumerate()
                .filter_map(|(index, edge_use)| {
                    (directed_start(*edge_use) == vertex).then_some(index)
                })
                .collect::<Vec<_>>();
            if matches.len() == 1 {
                Ok(matches[0])
            } else {
                Err(TopologyEditError::VertexNotOnOuterBoundary {
                    face: face_id,
                    vertex,
                })
            }
        };
        let start_index = unique_boundary_index(start)?;
        let end_index = unique_boundary_index(end)?;
        let use_count = outer_uses.len();
        let forward_distance = (end_index + use_count - start_index) % use_count;
        if start == end
            || (!authored_curve && (forward_distance <= 1 || forward_distance + 1 >= use_count))
            || (authored_curve && forward_distance == 0)
        {
            return Err(TopologyEditError::DegenerateFaceSplit);
        }
        let cyclic_path = |from: usize, to: usize| {
            let mut path = Vec::new();
            let mut index = from;
            while index != to {
                path.push(outer_uses[index]);
                index = (index + 1) % use_count;
            }
            path
        };
        let mut first_path = cyclic_path(start_index, end_index);
        let mut second_path = cyclic_path(end_index, start_index);

        let (curve_geometry, forward_pcurve, forward_correspondence) = match authored {
            Some(authored) => authored,
            None => {
                let pcurve_start = |edge_use_id: EdgeUseId| -> Result<CurvePoint2, GeometryError> {
                    let edge_use = self
                        .edge_use(edge_use_id)
                        .expect("validated wire edge-use ID");
                    let pcurve = self
                        .pcurve(edge_use.pcurve)
                        .expect("validated edge-use pcurve ID");
                    let point = pcurve.point_at(pcurve.domain_start())?;
                    Ok(CurvePoint2::new(point.x, point.y))
                };
                let start_uv = pcurve_start(outer_uses[start_index])?;
                let end_uv = pcurve_start(outer_uses[end_index])?;
                let start_point = self
                    .vertex(start)
                    .expect("outer-boundary vertex resolves")
                    .point
                    .clone();
                let end_point = self
                    .vertex(end)
                    .expect("outer-boundary vertex resolves")
                    .point
                    .clone();
                (
                    Curve3::line(start_point, end_point)?,
                    Pcurve::new(Curve2::from(
                        LineSeg2::try_new(start_uv, end_uv).map_err(GeometryError::from)?,
                    )),
                    ParameterCorrespondence::identity(),
                )
            }
        };
        let edge_domain = curve_geometry.domain().clone();
        let reverse_pcurve = forward_pcurve.reversed()?;
        let (scale, offset) = forward_correspondence.affine_coefficients().ok_or(
            TopologyEditError::UnsupportedFaceSplitCurve(curve_geometry.kind()),
        )?;
        let reverse_correspondence = ParameterCorrespondence::affine(
            -scale.clone(),
            scale * (forward_pcurve.domain_start() + forward_pcurve.domain_end()) + offset,
        )?;

        let mut staged = self.clone();
        let data = Arc::make_mut(&mut staged.data);
        let curve = Curve3Id::from_index(data.curves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
        data.curves.push(curve_geometry);
        let edge = EdgeId::from_index(data.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        data.edges.push(Edge {
            start,
            end,
            curve,
            domain: edge_domain,
        });

        let first_pcurve = PcurveId::from_index(data.pcurves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
        data.pcurves.push(reverse_pcurve);
        let second_pcurve = PcurveId::from_index(data.pcurves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
        data.pcurves.push(forward_pcurve);

        let first_use = EdgeUseId::from_index(data.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        data.edge_uses.push(EdgeUse {
            edge,
            direction: Direction::Reversed,
            pcurve: first_pcurve,
            parameter_correspondence: reverse_correspondence,
        });
        let second_use = EdgeUseId::from_index(data.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        data.edge_uses.push(EdgeUse {
            edge,
            direction: Direction::Forward,
            pcurve: second_pcurve,
            parameter_correspondence: forward_correspondence,
        });
        first_path.push(first_use);
        second_path.push(second_use);
        data.wires[outer.index()].edge_uses = first_path;
        let second_wire = WireId::from_index(data.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        data.wires.push(Wire {
            edge_uses: second_path,
        });

        data.faces[face_id.index()] = Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer,
                inner: Vec::new(),
            },
        };
        let second_face = FaceId::from_index(data.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        data.faces.push(Face {
            surface: face.surface,
            orientation: face.orientation,
            boundary: FaceBoundary::Trimmed {
                outer: second_wire,
                inner: Vec::new(),
            },
        });

        reset_model_caches(data);
        let policy = CurvePolicy::certified();
        let mut first_inner = Vec::new();
        let mut second_inner = Vec::new();
        if !inner.is_empty() {
            let first_path = staged.build_model_wire_curve_path(outer)?;
            let second_path = staged.build_model_wire_curve_path(second_wire)?;
            for wire in inner {
                let contour = staged.build_model_wire_contour(wire)?;
                let representative = contour.segments()[0].start();
                let classify = |path: &CurvePath2| -> Result<bool, TopologyEditError> {
                    match path
                        .classify_point(representative, &policy)
                        .map_err(GeometryError::from)?
                    {
                        Classification::Decided(ContourPointLocation::Inside) => Ok(true),
                        Classification::Decided(ContourPointLocation::Outside) => Ok(false),
                        Classification::Decided(ContourPointLocation::Boundary) => {
                            Err(BuildError::IntersectingFaceWires {
                                first: outer,
                                second: wire,
                            }
                            .into())
                        }
                        Classification::Uncertain(reason) => {
                            Err(GeometryError::PlanarClassificationUnresolved(reason).into())
                        }
                    }
                };
                let in_first = classify(&first_path)?;
                let in_second = classify(&second_path)?;
                match (in_first, in_second) {
                    (true, false) => first_inner.push(wire),
                    (false, true) => second_inner.push(wire),
                    _ => {
                        return Err(BuildError::IntersectingFaceWires {
                            first: outer,
                            second: wire,
                        }
                        .into());
                    }
                }
            }
        }
        let data = Arc::make_mut(&mut staged.data);
        data.faces[face_id.index()].boundary = FaceBoundary::Trimmed {
            outer,
            inner: first_inner,
        };
        data.faces[second_face.index()].boundary = FaceBoundary::Trimmed {
            outer: second_wire,
            inner: second_inner,
        };
        let shell = self.data.face_shell[face_id.index()];
        let source_position = data.shells[shell.index()]
            .faces
            .iter()
            .position(|candidate| *candidate == face_id)
            .expect("validated face-shell adjacency");
        data.shells[shell.index()]
            .faces
            .insert(source_position + 1, second_face);
        reset_model_caches(data);
        let validated = staged
            .revalidated()
            .map_err(TopologyEditError::Validation)?;
        Ok((
            validated,
            FaceSplit {
                edge,
                edge_uses: [first_use, second_use],
                first_wire: outer,
                second_wire,
                first_face: face_id,
                second_face,
            },
        ))
    }

    fn revalidated(&self) -> Result<Self, ValidationReport> {
        let mut builder = ModelBuilder::new();
        self.append_to_builder(&mut builder)
            .map_err(single_validation_error)?;
        builder.finish()
    }

    pub(crate) fn append_to_builder(
        &self,
        builder: &mut ModelBuilder,
    ) -> Result<Vec<SolidId>, BuildError> {
        let vertices = self
            .data
            .vertices
            .iter()
            .map(|vertex| builder.vertex(vertex.point.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let curves = self
            .data
            .curves
            .iter()
            .map(|curve| builder.curve(curve.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let pcurves = self
            .data
            .pcurves
            .iter()
            .map(|pcurve| builder.pcurve(pcurve.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let surfaces = self
            .data
            .surfaces
            .iter()
            .map(|surface| builder.surface(surface.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let edges = self
            .data
            .edges
            .iter()
            .map(|edge| {
                builder.edge(
                    vertices[edge.start.index()],
                    vertices[edge.end.index()],
                    curves[edge.curve.index()],
                    edge.domain.clone(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let edge_uses = self
            .data
            .edge_uses
            .iter()
            .map(|edge_use| {
                builder.edge_use(
                    edges[edge_use.edge.index()],
                    edge_use.direction,
                    pcurves[edge_use.pcurve.index()],
                    edge_use.parameter_correspondence.clone(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let wires = self
            .data
            .wires
            .iter()
            .map(|wire| {
                builder.wire(
                    wire.edge_uses
                        .iter()
                        .map(|edge_use| edge_uses[edge_use.index()])
                        .collect(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let faces = self
            .data
            .faces
            .iter()
            .map(|face| match &face.boundary {
                FaceBoundary::WholeSurface => {
                    builder.whole_face(surfaces[face.surface.index()], face.orientation)
                }
                FaceBoundary::Trimmed { outer, inner } => builder.face(
                    surfaces[face.surface.index()],
                    face.orientation,
                    wires[outer.index()],
                    inner.iter().map(|wire| wires[wire.index()]).collect(),
                ),
            })
            .collect::<Result<Vec<_>, _>>()?;
        let shells = self
            .data
            .shells
            .iter()
            .map(|shell| {
                builder.shell(shell.faces.iter().map(|face| faces[face.index()]).collect())
            })
            .collect::<Result<Vec<_>, _>>()?;
        self.data
            .solids
            .iter()
            .map(|solid| {
                builder.solid(
                    shells[solid.outer.index()],
                    solid
                        .voids
                        .iter()
                        .map(|shell| shells[shell.index()])
                        .collect(),
                )
            })
            .collect()
    }

    /// Iterates over vertices in deterministic arena order.
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = (VertexId, &Vertex)> {
        self.data
            .vertices
            .iter()
            .enumerate()
            .map(|(index, record)| (VertexId(index as u32), record))
    }

    /// Iterates over spatial curves in deterministic arena order.
    pub fn curves(&self) -> impl ExactSizeIterator<Item = (Curve3Id, &Curve3)> {
        self.data
            .curves
            .iter()
            .enumerate()
            .map(|(index, record)| (Curve3Id(index as u32), record))
    }

    /// Iterates over pcurves in deterministic arena order.
    pub fn pcurves(&self) -> impl ExactSizeIterator<Item = (PcurveId, &Pcurve)> {
        self.data
            .pcurves
            .iter()
            .enumerate()
            .map(|(index, record)| (PcurveId(index as u32), record))
    }

    /// Iterates over surfaces in deterministic arena order.
    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = (SurfaceId, &Surface)> {
        self.data
            .surfaces
            .iter()
            .enumerate()
            .map(|(index, record)| (SurfaceId(index as u32), record))
    }

    /// Iterates over edges in deterministic arena order.
    pub fn edges(&self) -> impl ExactSizeIterator<Item = (EdgeId, &Edge)> {
        self.data
            .edges
            .iter()
            .enumerate()
            .map(|(index, record)| (EdgeId(index as u32), record))
    }

    /// Iterates over edge uses in deterministic arena order.
    pub fn edge_uses(&self) -> impl ExactSizeIterator<Item = (EdgeUseId, &EdgeUse)> {
        self.data
            .edge_uses
            .iter()
            .enumerate()
            .map(|(index, record)| (EdgeUseId(index as u32), record))
    }

    /// Iterates over wires in deterministic arena order.
    pub fn wires(&self) -> impl ExactSizeIterator<Item = (WireId, &Wire)> {
        self.data
            .wires
            .iter()
            .enumerate()
            .map(|(index, record)| (WireId(index as u32), record))
    }

    /// Iterates over faces in deterministic arena order.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = (FaceId, &Face)> {
        self.data
            .faces
            .iter()
            .enumerate()
            .map(|(index, record)| (FaceId(index as u32), record))
    }

    /// Iterates over shells in deterministic arena order.
    pub fn shells(&self) -> impl ExactSizeIterator<Item = (ShellId, &Shell)> {
        self.data
            .shells
            .iter()
            .enumerate()
            .map(|(index, record)| (ShellId(index as u32), record))
    }

    /// Iterates over solids in deterministic arena order.
    pub fn solids(&self) -> impl ExactSizeIterator<Item = (SolidId, &Solid)> {
        self.data
            .solids
            .iter()
            .enumerate()
            .map(|(index, record)| (SolidId(index as u32), record))
    }

    /// Returns the exact model-space bounds of all canonical vertices.
    ///
    /// The empty model has no bounds. The result is retained after the first
    /// certified coordinate-ordering pass.
    pub fn bounds(&self) -> Result<Option<Aabb>, GeometryError> {
        self.data
            .bounds
            .get_or_init(|| {
                compute_bounds(
                    &self.data.vertices,
                    &self.data.curves,
                    &self.data.certified_spheres,
                    &self.data.certified_sphere_pairs,
                )
            })
            .clone()
    }

    /// Applies one certified invertible affine transform to the entire model.
    ///
    /// Topology and typed IDs are retained. Exact curve and surface carriers
    /// are rebuilt in arena order, while pcurves and parameter maps remain in
    /// their authored surface coordinates.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        let reverses_orientation =
            affine_transform_orientation(transform)? == std::cmp::Ordering::Less;
        let vertices = self
            .data
            .vertices
            .iter()
            .map(|vertex| {
                transform
                    .transform_point3(&vertex.point)
                    .map(|point| Vertex { point })
                    .map_err(|_| GeometryError::TransformFailure)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let curves = self
            .data
            .curves
            .iter()
            .map(|curve| curve.transformed(transform))
            .collect::<Result<Vec<_>, _>>()?;
        let surfaces = self
            .data
            .surfaces
            .iter()
            .map(|surface| surface.transformed(transform, reverses_orientation))
            .collect::<Result<Vec<_>, _>>()?;
        let pcurves = if reverses_orientation {
            let mut reflection_sums = vec![None::<Real>; self.data.pcurves.len()];
            for (edge_use_index, edge_use) in self.data.edge_uses.iter().enumerate() {
                let wire = self.data.edge_use_wire[edge_use_index];
                let face = self.data.wire_face[wire.index()];
                let surface = &self.data.surfaces[self.data.faces[face.index()].surface.index()];
                let reflection_sum = match surface.exact_data() {
                    SurfaceExactData::RationalBezier { .. } => Real::one(),
                    SurfaceExactData::Nurbs { u_knots, .. } => {
                        &u_knots[0] + &u_knots[u_knots.len() - 1]
                    }
                    _ => Real::zero(),
                };
                let slot = &mut reflection_sums[edge_use.pcurve.index()];
                if let Some(existing) = slot {
                    match compare_reals(existing, &reflection_sum) {
                        PredicateOutcome::Decided {
                            value: std::cmp::Ordering::Equal,
                            ..
                        } => {}
                        PredicateOutcome::Decided { .. } => {
                            return Err(GeometryError::UnsupportedTransform);
                        }
                        PredicateOutcome::Unknown { needed, stage } => {
                            return Err(GeometryError::PredicateUnresolved { needed, stage });
                        }
                    }
                } else {
                    *slot = Some(reflection_sum);
                }
            }
            self.data
                .pcurves
                .iter()
                .zip(reflection_sums)
                .map(|(pcurve, reflection_sum)| {
                    pcurve.reflected_and_reversed_x(reflection_sum.unwrap_or_else(Real::zero))
                })
                .collect::<Result<Vec<_>, _>>()?
        } else {
            self.data.pcurves.clone()
        };
        let edge_uses = if reverses_orientation {
            self.data
                .edge_uses
                .iter()
                .map(|edge_use| {
                    let pcurve = &self.data.pcurves[edge_use.pcurve.index()];
                    EdgeUse {
                        edge: edge_use.edge,
                        direction: edge_use.direction.reversed(),
                        pcurve: edge_use.pcurve,
                        parameter_correspondence: edge_use
                            .parameter_correspondence
                            .reversed_pcurve(pcurve),
                    }
                })
                .collect()
        } else {
            self.data.edge_uses.clone()
        };
        let certified_cylinders = self
            .data
            .certified_cylinders
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedCylinderShell {
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            radius: certificate.radius.clone(),
                            v_min: certificate.v_min.clone(),
                            v_max: certificate.v_max.clone(),
                            sphere_subtraction: certificate
                                .sphere_subtraction
                                .as_ref()
                                .map(|subtraction| {
                                    Ok::<_, GeometryError>(match subtraction {
                                        CertifiedCylinderSphereSubtraction::Void {
                                            center,
                                            radius,
                                        } => CertifiedCylinderSphereSubtraction::Void {
                                            center: transform
                                                .transform_point3(center)
                                                .map_err(|_| GeometryError::TransformFailure)?,
                                            radius: radius.clone(),
                                        },
                                        CertifiedCylinderSphereSubtraction::Component {
                                            center,
                                            radius,
                                            side,
                                        } => CertifiedCylinderSphereSubtraction::Component {
                                            center: transform
                                                .transform_point3(center)
                                                .map_err(|_| GeometryError::TransformFailure)?,
                                            radius: radius.clone(),
                                            side: *side,
                                        },
                                    })
                                })
                                .transpose()?,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_spheres = self
            .data
            .certified_spheres
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedSphereShell {
                            center: transform
                                .transform_point3(&certificate.center)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            radius: certificate.radius.clone(),
                            region: match &certificate.region {
                                CertifiedSphereRegion::Whole => CertifiedSphereRegion::Whole,
                                CertifiedSphereRegion::Axial(clip) => {
                                    CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                                        axis: transform.transform_direction3(&clip.axis),
                                        min: clip.min.clone(),
                                        max: clip.max.clone(),
                                    })
                                }
                                CertifiedSphereRegion::Radial(clip) => {
                                    CertifiedSphereRegion::Radial(CertifiedSphereRadialClip {
                                        axis: transform.transform_direction3(&clip.axis),
                                        radius: clip.radius.clone(),
                                        side: clip.side,
                                    })
                                }
                                CertifiedSphereRegion::FiniteCylinder(region) => {
                                    CertifiedSphereRegion::FiniteCylinder(
                                        CertifiedSphereFiniteCylinderRegion {
                                            origin: transform
                                                .transform_point3(&region.origin)
                                                .map_err(|_| GeometryError::TransformFailure)?,
                                            axis: transform.transform_direction3(&region.axis),
                                            radius: region.radius.clone(),
                                            v_min: region.v_min.clone(),
                                            v_max: region.v_max.clone(),
                                            operation: region.operation,
                                        },
                                    )
                                }
                            },
                            voids: certificate
                                .voids
                                .iter()
                                .map(|void| {
                                    Ok(match void {
                                        CertifiedSphereVoid::Sphere { center, radius } => {
                                            CertifiedSphereVoid::Sphere {
                                                center: transform
                                                    .transform_point3(center)
                                                    .map_err(|_| GeometryError::TransformFailure)?,
                                                radius: radius.clone(),
                                            }
                                        }
                                        CertifiedSphereVoid::Cylinder(cylinder) => {
                                            CertifiedSphereVoid::Cylinder(Box::new(
                                                CertifiedSphereCylinderVoid {
                                                    origin: transform
                                                        .transform_point3(&cylinder.origin)
                                                        .map_err(|_| {
                                                            GeometryError::TransformFailure
                                                        })?,
                                                    axis: transform
                                                        .transform_direction3(&cylinder.axis),
                                                    radius: cylinder.radius.clone(),
                                                    v_min: cylinder.v_min.clone(),
                                                    v_max: cylinder.v_max.clone(),
                                                },
                                            ))
                                        }
                                    })
                                })
                                .collect::<Result<Vec<_>, GeometryError>>()?,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_sphere_pairs = self
            .data
            .certified_sphere_pairs
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedSpherePairShell {
                            first_center: transform
                                .transform_point3(&certificate.first_center)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            first_radius: certificate.first_radius.clone(),
                            second_center: transform
                                .transform_point3(&certificate.second_center)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            second_radius: certificate.second_radius.clone(),
                            kind: certificate.kind,
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_tori = self
            .data
            .certified_tori
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedTorusShell {
                            center: transform
                                .transform_point3(&certificate.center)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            major_radius: certificate.major_radius.clone(),
                            minor_radius: certificate.minor_radius.clone(),
                            region: match &certificate.region {
                                CertifiedTorusRegion::Whole => CertifiedTorusRegion::Whole,
                                CertifiedTorusRegion::Axial { min, max } => {
                                    CertifiedTorusRegion::Axial {
                                        min: min.clone(),
                                        max: max.clone(),
                                    }
                                }
                                CertifiedTorusRegion::LongitudinalHalf { interior_normal } => {
                                    CertifiedTorusRegion::LongitudinalHalf {
                                        interior_normal: transform
                                            .transform_direction3(interior_normal),
                                    }
                                }
                            },
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_revolutions = self
            .data
            .certified_revolutions
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedRevolutionShell {
                            axis_origin: transform
                                .transform_point3(&certificate.axis_origin)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            profile: certificate.profile.clone(),
                            voids: certificate.voids.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_lofts = self
            .data
            .certified_lofts
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedLoftShell {
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            u: transform.transform_direction3(&certificate.u),
                            v: transform.transform_direction3(&certificate.v),
                            height_axis: transform.transform_direction3(&certificate.height_axis),
                            spans: certificate.spans.clone(),
                            parameter_volume: certificate.parameter_volume.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_curve_sweeps = self
            .data
            .certified_curve_sweeps
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedCurveSweepShell {
                            profile: certificate.profile.clone(),
                            holes: certificate.holes.clone(),
                            path: certificate.path.transformed(transform)?,
                            u_path: transform_vector_curve(&certificate.u_path, transform)?,
                            v_path: transform_vector_curve(&certificate.v_path, transform)?,
                            area_scale_integral: certificate.area_scale_integral.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_cone_frustums = self
            .data
            .certified_cone_frustums
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedConeFrustumShell {
                            apex: transform
                                .transform_point3(&certificate.apex)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            axis: transform.transform_direction3(&certificate.axis),
                            semi_angle: certificate.semi_angle.clone(),
                            v_min: certificate.v_min.clone(),
                            v_max: certificate.v_max.clone(),
                            region: match &certificate.region {
                                CertifiedConeFrustumRegion::Whole => {
                                    CertifiedConeFrustumRegion::Whole
                                }
                                CertifiedConeFrustumRegion::LongitudinalHalf {
                                    interior_normal,
                                } => CertifiedConeFrustumRegion::LongitudinalHalf {
                                    interior_normal: transform
                                        .transform_direction3(interior_normal),
                                },
                            },
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let certified_prisms = self
            .data
            .certified_prisms
            .iter()
            .map(|certificate| {
                certificate
                    .as_ref()
                    .map(|certificate| {
                        Ok(CertifiedPrismShell {
                            outer: certificate.outer.clone(),
                            holes: certificate.holes.clone(),
                            origin: transform
                                .transform_point3(&certificate.origin)
                                .map_err(|_| GeometryError::TransformFailure)?,
                            u: transform.transform_direction3(&certificate.u),
                            v: transform.transform_direction3(&certificate.v),
                            extrusion: transform.transform_direction3(&certificate.extrusion),
                            parameter_min: certificate.parameter_min.clone(),
                            parameter_max: certificate.parameter_max.clone(),
                        })
                    })
                    .transpose()
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let wires = if reverses_orientation {
            self.data
                .wires
                .iter()
                .map(|wire| {
                    let mut edge_uses = wire.edge_uses.clone();
                    edge_uses.reverse();
                    Wire { edge_uses }
                })
                .collect()
        } else {
            self.data.wires.clone()
        };
        Ok(Self {
            data: Arc::new(ModelData {
                vertices,
                curves,
                pcurves,
                surfaces,
                edges: self.data.edges.clone(),
                edge_uses,
                wires,
                faces: self.data.faces.clone(),
                shells: self.data.shells.clone(),
                solids: self.data.solids.clone(),
                vertex_edges: self.data.vertex_edges.clone(),
                edge_uses_by_edge: self.data.edge_uses_by_edge.clone(),
                edge_use_wire: self.data.edge_use_wire.clone(),
                wire_face: self.data.wire_face.clone(),
                face_shell: self.data.face_shell.clone(),
                shell_solid: self.data.shell_solid.clone(),
                certified_cylinders,
                certified_spheres,
                certified_sphere_pairs,
                certified_cone_frustums,
                certified_tori,
                certified_revolutions,
                certified_lofts,
                certified_curve_sweeps,
                certified_prisms,
                bounds: OnceLock::new(),
                face_contours: (0..self.data.faces.len())
                    .map(|_| OnceLock::new())
                    .collect(),
            }),
        })
    }

    /// Returns the exact area of a supported validated face.
    ///
    /// Whole spheres and periodic latitude caps use analytic spherical area.
    /// Other active carriers integrate their exact parameter-space boundary
    /// and Jacobian without tessellation.
    pub fn face_area(&self, id: FaceId) -> Result<Real, QueryError> {
        let face = self.face(id).ok_or(QueryError::InvalidReference {
            kind: EntityKind::Face,
            index: id.index(),
        })?;
        let surface = self
            .surface(face.surface)
            .expect("validated face surface ID");
        if let SurfaceExactData::Sphere { radius, .. } = surface.exact_data() {
            if face.is_whole_surface() {
                return Ok(Real::from(4) * Real::pi() * &radius * radius);
            }
            let outer = face.outer().ok_or(GeometryError::UnsupportedMeasurement)?;
            if let [upper_wire] = face.inner() {
                let lower = self
                    .pcurve(
                        self.edge_use(self.wire(outer).expect("validated wire").edge_uses[0])
                            .expect("validated use")
                            .pcurve,
                    )
                    .expect("validated pcurve")
                    .line_segment()
                    .expect("validated spherical latitude")
                    .start()
                    .y()
                    .clone();
                let upper = self
                    .pcurve(
                        self.edge_use(self.wire(*upper_wire).expect("validated wire").edge_uses[0])
                            .expect("validated use")
                            .pcurve,
                    )
                    .expect("validated pcurve")
                    .line_segment()
                    .expect("validated spherical latitude")
                    .start()
                    .y()
                    .clone();
                return Ok(Real::tau() * &radius * radius * (upper.sin() - lower.sin()));
            }
            let wire = self.wire(outer).expect("validated spherical face wire");
            let mut latitude_cap = true;
            for edge_use in &wire.edge_uses {
                let edge_use = self.edge_use(*edge_use).expect("validated spherical use");
                let line = self
                    .pcurve(edge_use.pcurve)
                    .expect("validated spherical pcurve")
                    .line_segment()
                    .expect("validated spherical line pcurve");
                if !real_values_equal(line.start().y(), line.end().y())
                    .map_err(build_error_geometry)?
                {
                    latitude_cap = false;
                    break;
                }
            }
            if !latitude_cap {
                return Ok(self.signed_sphere_wire_area(outer, &radius)?.abs());
            }
            let first_use = self
                .edge_use(wire.edge_uses[0])
                .expect("validated spherical edge use");
            let pcurve = self
                .pcurve(first_use.pcurve)
                .expect("validated spherical pcurve");
            let line = pcurve
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            let increasing = decided_model_order(compare_reals(line.end().x(), line.start().x()))?
                == std::cmp::Ordering::Greater;
            let upper = match face.orientation {
                Orientation::Forward => increasing,
                Orientation::Reversed => !increasing,
            };
            let sine = line.start().y().clone().sin();
            let height_factor = if upper {
                Real::one() - sine
            } else {
                Real::one() + sine
            };
            return Ok(Real::from(2) * Real::pi() * &radius * radius * height_factor);
        }
        let outer = face.outer().ok_or(GeometryError::UnsupportedMeasurement)?;
        if let SurfaceExactData::Torus {
            major_radius,
            minor_radius,
            ..
        } = surface.exact_data()
        {
            let mut area = self
                .signed_torus_wire_area(outer, &major_radius, &minor_radius)?
                .abs();
            for inner in face.inner() {
                area -= self
                    .signed_torus_wire_area(*inner, &major_radius, &minor_radius)?
                    .abs();
            }
            return Ok(area);
        }
        if let SurfaceExactData::Cone { semi_angle, .. } = surface.exact_data() {
            let mut area = self.signed_cone_wire_area(outer, &semi_angle)?.abs();
            for inner in face.inner() {
                area -= self.signed_cone_wire_area(*inner, &semi_angle)?.abs();
            }
            return Ok(area);
        }
        if let SurfaceExactData::Revolution {
            profile,
            axis_origin,
            axis,
        } = surface.exact_data()
        {
            if !face.inner().is_empty() {
                return Err(GeometryError::UnsupportedMeasurement.into());
            }
            let profile = Curve3::from_exact_data(*profile)?;
            let wire = self.wire(outer).expect("validated revolution face wire");
            if wire.edge_uses.len() != 4 {
                return Err(GeometryError::UnsupportedMeasurement.into());
            }
            let mut u_values = Vec::with_capacity(2);
            let mut v_values = Vec::with_capacity(2);
            for use_id in &wire.edge_uses {
                let pcurve = self
                    .pcurve(
                        self.edge_use(*use_id)
                            .expect("validated revolution edge use")
                            .pcurve,
                    )
                    .expect("validated revolution pcurve");
                let segment = pcurve
                    .line_segment()
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
                if !real_values_equal(segment.start().x(), segment.end().x())
                    .map_err(build_error_geometry)?
                    && !real_values_equal(segment.start().y(), segment.end().y())
                        .map_err(build_error_geometry)?
                {
                    return Err(GeometryError::UnsupportedMeasurement.into());
                }
                insert_sorted_real(&mut u_values, segment.start().x())
                    .map_err(build_error_geometry)?;
                insert_sorted_real(&mut u_values, segment.end().x())
                    .map_err(build_error_geometry)?;
                insert_sorted_real(&mut v_values, segment.start().y())
                    .map_err(build_error_geometry)?;
                insert_sorted_real(&mut v_values, segment.end().y())
                    .map_err(build_error_geometry)?;
            }
            if u_values.len() != 2 || v_values.len() != 2 {
                return Err(GeometryError::UnsupportedMeasurement.into());
            }
            let profile = profile.subcurve(&v_values[0], &v_values[1])?;
            let radial_at = |point: &Point3| {
                let relative = point - &axis_origin;
                let axial = axis.dot(&relative);
                relative - axis.clone() * axial
            };
            let angular_span = &u_values[1] - &u_values[0];
            let line_image_area = || -> Result<Real, QueryError> {
                let start = profile.point_at(profile.domain().start())?;
                let end = profile.point_at(profile.domain().end())?;
                let start_radial = radial_at(&start);
                let end_radial = radial_at(&end);
                if decided_model_order(compare_reals(
                    &start_radial.cross(&end_radial).norm_squared(),
                    &Real::zero(),
                ))? != std::cmp::Ordering::Equal
                    || decided_model_order(compare_reals(
                        &start_radial.dot(&end_radial),
                        &Real::zero(),
                    ))? != std::cmp::Ordering::Greater
                {
                    return Err(GeometryError::UnsupportedMeasurement.into());
                }
                let start_radius = start_radial
                    .norm_squared()
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                let end_radius = end_radial
                    .norm_squared()
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                let profile_length = (&end - &start)
                    .norm_squared()
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                (angular_span.clone() * profile_length * (start_radius + end_radius)
                    / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)
                .map_err(QueryError::from)
            };
            return match profile.exact_data() {
                Curve3ExactData::Line(_) => line_image_area(),
                Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. }
                    if certified_monotone_line_curve_image(&profile)
                        .map_err(build_error_geometry)? =>
                {
                    line_image_area()
                }
                Curve3ExactData::EllipseArc(data) if data.circle => {
                    let start_point = profile.point_at(profile.domain().start())?;
                    let start_radial = radial_at(&start_point);
                    let start_radius = start_radial
                        .norm_squared()
                        .sqrt()
                        .map_err(|_| GeometryError::ElementaryFunction)?;
                    let radial_unit = (start_radial / &start_radius)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let center_radial = radial_unit.dot(&radial_at(&data.center));
                    let x_radial = radial_unit.dot(&data.x);
                    let y_radial = radial_unit.dot(&data.y);
                    let start_angle = &data.angle_at_start
                        + Real::from(data.direction)
                            * (profile.domain().start() - &data.domain_start);
                    let end_angle = &data.angle_at_start
                        + Real::from(data.direction)
                            * (profile.domain().end() - &data.domain_start);
                    let direction = Real::from(data.direction);
                    let integral_cos =
                        &direction * (end_angle.clone().sin() - start_angle.clone().sin());
                    let integral_sin =
                        direction * (start_angle.clone().cos() - end_angle.clone().cos());
                    let parameter_span = profile.domain().end() - profile.domain().start();
                    let radial_integral = center_radial * parameter_span
                        + &data.x_radius * (x_radial * integral_cos + y_radial * integral_sin);
                    Ok(angular_span * data.x_radius * radial_integral)
                }
                _ => Err(GeometryError::UnsupportedMeasurement.into()),
            };
        }
        let mut parameter_area = self.signed_model_wire_double_area(outer)?.abs();
        for inner in face.inner() {
            parameter_area -= self.signed_model_wire_double_area(*inner)?.abs();
        }
        let surface_scale = match surface.exact_data() {
            SurfaceExactData::Plane { u, v, .. } => u
                .cross(&v)
                .norm_squared()
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?,
            SurfaceExactData::Cylinder { radius, .. } => radius,
            SurfaceExactData::Extrusion { profile, direction } => {
                let profile = Curve3::from_exact_data(*profile)?;
                return self.extrusion_face_area(
                    face,
                    outer,
                    &profile,
                    &direction,
                    &parameter_area,
                );
            }
            data @ (SurfaceExactData::RationalBezier { .. } | SurfaceExactData::Nurbs { .. }) => {
                return affine_tensor_face_area(&data, &parameter_area)
                    .map_err(build_error_geometry)?
                    .ok_or(GeometryError::UnsupportedMeasurement)
                    .map_err(QueryError::from);
            }
            _ => return Err(GeometryError::UnsupportedMeasurement.into()),
        };
        let double_area = parameter_area * surface_scale;
        (double_area / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)
            .map_err(QueryError::from)
    }

    fn extrusion_face_area(
        &self,
        face: &Face,
        outer: WireId,
        profile: &Curve3,
        direction: &Vector3,
        parameter_double_area: &Real,
    ) -> Result<Real, QueryError> {
        if let Some(scale) =
            extrusion_constant_area_scale(profile, direction).map_err(build_error_geometry)?
        {
            return (parameter_double_area * scale / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)
                .map_err(QueryError::from);
        }
        if !face.inner().is_empty()
            || !certified_monotone_line_curve_image(profile).map_err(build_error_geometry)?
        {
            return Err(GeometryError::UnsupportedMeasurement.into());
        }
        let Some((u_min, u_max, v_min, v_max)) = self.axis_aligned_parameter_rectangle(outer)?
        else {
            return Err(GeometryError::UnsupportedMeasurement.into());
        };
        let rectangle_double_area = Real::from(2) * (&u_max - &u_min) * (&v_max - &v_min);
        if !real_values_equal(parameter_double_area, &rectangle_double_area)
            .map_err(build_error_geometry)?
            || !profile.domain().contains(&u_min)?
            || !profile.domain().contains(&u_max)?
        {
            return Err(GeometryError::UnsupportedMeasurement.into());
        }
        let restricted = profile.subcurve(&u_min, &u_max)?;
        let start = restricted.point_at(restricted.domain().start())?;
        let end = restricted.point_at(restricted.domain().end())?;
        let swept_area = (&end - &start)
            .cross(direction)
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?
            * (v_max - v_min);
        Ok(swept_area)
    }

    fn axis_aligned_parameter_rectangle(
        &self,
        wire: WireId,
    ) -> Result<Option<(Real, Real, Real, Real)>, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        if wire.edge_uses.len() != 4 {
            return Ok(None);
        }
        let mut u_values = Vec::with_capacity(2);
        let mut v_values = Vec::with_capacity(2);
        for edge_use in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let Some(line) = pcurve.line_segment() else {
                return Ok(None);
            };
            let constant_u = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let constant_v = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if constant_u == constant_v {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, line.start().x()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut u_values, line.end().x()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut v_values, line.start().y()).map_err(build_error_geometry)?;
            insert_sorted_real(&mut v_values, line.end().y()).map_err(build_error_geometry)?;
        }
        if u_values.len() != 2 || v_values.len() != 2 {
            return Ok(None);
        }
        Ok(Some((
            u_values.remove(0),
            u_values.remove(0),
            v_values.remove(0),
            v_values.remove(0),
        )))
    }

    fn signed_model_wire_double_area(&self, wire: WireId) -> Result<Real, GeometryError> {
        match self.signed_wire_double_area(wire) {
            Ok(area) => Ok(area),
            Err(GeometryError::UnsupportedPcurveContour) => {
                let area = self
                    .build_model_wire_curve_path(wire)?
                    .bezier_boundary_loop()
                    .map_err(GeometryError::from)?
                    .boundary_loop()
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
                Ok(Real::from(2) * area)
            }
            Err(error) => Err(error),
        }
    }

    /// Returns the exact volume of a validated solid.
    pub fn solid_volume(&self, id: SolidId) -> Result<Real, QueryError> {
        let solid = self.solid(id).ok_or(QueryError::InvalidReference {
            kind: EntityKind::Solid,
            index: id.index(),
        })?;
        if let Some(cylinder) = &self.data.certified_cylinders[id.index()] {
            let volume = Real::pi()
                * &cylinder.radius
                * &cylinder.radius
                * (&cylinder.v_max - &cylinder.v_min);
            let Some(subtraction) = &cylinder.sphere_subtraction else {
                return Ok(volume);
            };
            let excluded = match subtraction {
                CertifiedCylinderSphereSubtraction::Void { radius, .. } => sphere_volume(radius)?,
                CertifiedCylinderSphereSubtraction::Component {
                    center,
                    radius,
                    side,
                } => {
                    let center_parameter = (center - &cylinder.origin).dot(&cylinder.axis);
                    let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                        let cubic = (height * height * height / Real::from(3))
                            .map_err(|_| GeometryError::ProjectiveDivision)?;
                        Ok(Real::pi() * (radius * radius * height - cubic))
                    };
                    match side {
                        CertifiedCylinderSphereComponentSide::Lower => {
                            let upper = &cylinder.v_max - &center_parameter;
                            antiderivative(&upper)? - antiderivative(&-radius.clone())?
                        }
                        CertifiedCylinderSphereComponentSide::Upper => {
                            let lower = &cylinder.v_min - &center_parameter;
                            antiderivative(radius)? - antiderivative(&lower)?
                        }
                    }
                }
            };
            return Ok(volume - excluded);
        }
        if let Some(sphere) = &self.data.certified_spheres[id.index()] {
            if let CertifiedSphereRegion::Axial(clip) = &sphere.region {
                let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                    let cubic = (height * height * height / Real::from(3))
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    Ok(Real::pi() * (&sphere.radius * &sphere.radius * height - cubic))
                };
                return Ok(antiderivative(&clip.max)? - antiderivative(&clip.min)?);
            }
            if let CertifiedSphereRegion::Radial(clip) = &sphere.region {
                let axial_half_height = (&sphere.radius * &sphere.radius
                    - &clip.radius * &clip.radius)
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                let axial_cubic = &axial_half_height * &axial_half_height * axial_half_height;
                let retained_cubic = match clip.side {
                    CertifiedSphereRadialSide::Inside => {
                        &sphere.radius * &sphere.radius * &sphere.radius - axial_cubic
                    }
                    CertifiedSphereRadialSide::Outside => axial_cubic,
                };
                return (Real::from(4) * Real::pi() * retained_cubic / Real::from(3))
                    .map_err(|_| GeometryError::ProjectiveDivision)
                    .map_err(QueryError::from);
            }
            if let CertifiedSphereRegion::FiniteCylinder(region) = &sphere.region {
                let cylinder_volume =
                    Real::pi() * &region.radius * &region.radius * (&region.v_max - &region.v_min);
                let sphere_volume = sphere_volume(&sphere.radius)?;
                let overlap = sphere_finite_cylinder_overlap_volume(sphere, region)?;
                return Ok(match region.operation {
                    CertifiedSphereFiniteCylinderOperation::Union => {
                        sphere_volume + cylinder_volume - overlap
                    }
                    CertifiedSphereFiniteCylinderOperation::Intersection => overlap,
                    CertifiedSphereFiniteCylinderOperation::Difference => sphere_volume - overlap,
                });
            }
            let mut volume = sphere_volume(&sphere.radius)?;
            for void in &sphere.voids {
                volume -= match void {
                    CertifiedSphereVoid::Sphere { radius, .. } => sphere_volume(radius)?,
                    CertifiedSphereVoid::Cylinder(cylinder) => {
                        Real::pi()
                            * &cylinder.radius
                            * &cylinder.radius
                            * (&cylinder.v_max - &cylinder.v_min)
                    }
                };
            }
            return Ok(volume);
        }
        if let Some(pair) = &self.data.certified_sphere_pairs[id.index()] {
            let first_volume = sphere_volume(&pair.first_radius)?;
            let second_volume = sphere_volume(&pair.second_radius)?;
            let overlap = sphere_pair_intersection_volume(pair)?;
            return Ok(match pair.kind {
                CertifiedSpherePairKind::Union => first_volume + second_volume - overlap,
                CertifiedSpherePairKind::Intersection => overlap,
                CertifiedSpherePairKind::Difference => first_volume - overlap,
            });
        }
        if let Some(torus) = &self.data.certified_tori[id.index()] {
            return match &torus.region {
                CertifiedTorusRegion::Whole => Ok(Real::from(2)
                    * Real::pi()
                    * Real::pi()
                    * &torus.major_radius
                    * &torus.minor_radius
                    * &torus.minor_radius),
                CertifiedTorusRegion::Axial { min, max } => {
                    let antiderivative = |height: &Real| -> Result<Real, GeometryError> {
                        let radial = (&torus.minor_radius * &torus.minor_radius - height * height)
                            .sqrt()
                            .map_err(|_| GeometryError::ElementaryFunction)?;
                        let angle = (height / &torus.minor_radius)
                            .map_err(|_| GeometryError::ProjectiveDivision)?
                            .asin()
                            .map_err(|_| GeometryError::ElementaryFunction)?;
                        Ok(Real::from(2)
                            * Real::pi()
                            * &torus.major_radius
                            * (height * radial + &torus.minor_radius * &torus.minor_radius * angle))
                    };
                    Ok(antiderivative(max)? - antiderivative(min)?)
                }
                CertifiedTorusRegion::LongitudinalHalf { .. } => Ok(Real::pi()
                    * Real::pi()
                    * &torus.major_radius
                    * &torus.minor_radius
                    * &torus.minor_radius),
            };
        }
        if let Some(frustum) = &self.data.certified_cone_frustums[id.index()] {
            let sine = frustum.semi_angle.clone().sin();
            let cosine = frustum.semi_angle.clone().cos();
            let min_radius = &frustum.v_min * &sine;
            let max_radius = &frustum.v_max * sine;
            let height = (&frustum.v_max - &frustum.v_min) * cosine;
            let volume = (Real::pi()
                * height
                * (&min_radius * &min_radius
                    + &min_radius * &max_radius
                    + &max_radius * &max_radius)
                / Real::from(3))
            .map_err(|_| GeometryError::ProjectiveDivision)
            .map_err(QueryError::from)?;
            return match &frustum.region {
                CertifiedConeFrustumRegion::Whole => Ok(volume),
                CertifiedConeFrustumRegion::LongitudinalHalf { .. } => (volume / Real::from(2))
                    .map_err(|_| GeometryError::ProjectiveDivision)
                    .map_err(QueryError::from),
            };
        }
        if let Some(revolution) = &self.data.certified_revolutions[id.index()] {
            let mut first_moment = revolution
                .profile
                .signed_x_first_moment()?
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            for void in &revolution.voids {
                first_moment -= void
                    .signed_x_first_moment()?
                    .ok_or(GeometryError::UnsupportedMeasurement)?;
            }
            return Ok(Real::from(2) * Real::pi() * first_moment);
        }
        if let Some(loft) = &self.data.certified_lofts[id.index()] {
            let jacobian = loft.u.dot(&loft.v.cross(&loft.height_axis)).abs();
            return Ok(jacobian * &loft.parameter_volume);
        }
        if let Some(sweep) = &self.data.certified_curve_sweeps[id.index()] {
            let mut area = sweep
                .profile
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(GeometryError::UnsupportedMeasurement)?
                .abs();
            for hole in &sweep.holes {
                area -= hole
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(GeometryError::UnsupportedMeasurement)?
                    .abs();
            }
            let path_start = sweep.path.point_at(sweep.path.domain().start())?;
            let path_end = sweep.path.point_at(sweep.path.domain().end())?;
            let u = Vector3::from(sweep.u_path.point_at(sweep.u_path.domain().start())?);
            let v = Vector3::from(sweep.v_path.point_at(sweep.v_path.domain().start())?);
            let progress = u.cross(&v).dot(&(path_end - path_start)).abs();
            return Ok(area * progress * &sweep.area_scale_integral);
        }
        if let Some(prism) = &self.data.certified_prisms[id.index()] {
            let mut area = prism
                .outer
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(GeometryError::UnsupportedMeasurement)?
                .abs();
            for hole in &prism.holes {
                area -= hole
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(GeometryError::UnsupportedMeasurement)?
                    .abs();
            }
            let jacobian = prism.u.dot(&prism.v.cross(&prism.extrusion)).abs();
            return Ok(area * jacobian * (&prism.parameter_max - &prism.parameter_min));
        }
        let signed_six_volume = std::iter::once(&solid.outer)
            .chain(solid.voids.iter())
            .map(|shell| self.signed_shell_six_volume(*shell))
            .fold(Real::zero(), |sum, volume| sum + volume);
        (signed_six_volume / Real::from(6))
            .map_err(|_| GeometryError::ProjectiveDivision)
            .map_err(QueryError::from)
    }

    /// Classifies a point against a validated planar line-bounded solid.
    ///
    /// Face contours are materialized once from retained pcurves. Ray
    /// directions that hit an edge or vertex are discarded and retried; if
    /// every deterministic direction is degenerate, the result stays an
    /// explicit unresolved error.
    pub fn classify_point(
        &self,
        solid: SolidId,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let solid_id = solid;
        let solid = self.solid(solid_id).ok_or(QueryError::InvalidReference {
            kind: EntityKind::Solid,
            index: solid_id.index(),
        })?;
        if let Some(cylinder) = &self.data.certified_cylinders[solid_id.index()] {
            return self.classify_point_against_cylinder(cylinder, point);
        }
        if let Some(sphere) = &self.data.certified_spheres[solid_id.index()] {
            return self.classify_point_against_sphere(sphere, point);
        }
        if let Some(pair) = &self.data.certified_sphere_pairs[solid_id.index()] {
            return self.classify_point_against_sphere_pair(pair, point);
        }
        if let Some(torus) = &self.data.certified_tori[solid_id.index()] {
            return self.classify_point_against_torus(torus, point);
        }
        if let Some(frustum) = &self.data.certified_cone_frustums[solid_id.index()] {
            return self.classify_point_against_cone_frustum(frustum, point);
        }
        if let Some(revolution) = &self.data.certified_revolutions[solid_id.index()] {
            return self.classify_point_against_revolution(revolution, point);
        }
        if let Some(loft) = &self.data.certified_lofts[solid_id.index()] {
            return self.classify_point_against_loft(loft, point);
        }
        if let Some(sweep) = &self.data.certified_curve_sweeps[solid_id.index()] {
            return self.classify_point_against_curve_sweep(sweep, point);
        }
        if let Some(prism) = &self.data.certified_prisms[solid_id.index()] {
            return self.classify_point_against_prism(prism, point);
        }
        for shell in std::iter::once(&solid.outer).chain(solid.voids.iter()) {
            for face in &self.shell(*shell).expect("validated shell ID").faces {
                let signed = self.face_plane_value(*face, point)?;
                if decided_model_order(compare_reals(&signed, &Real::zero()))?
                    == std::cmp::Ordering::Equal
                {
                    match self.classify_point_on_face(*face, point)? {
                        Classification::Decided(ContourPointLocation::Inside)
                        | Classification::Decided(ContourPointLocation::Boundary) => {
                            return Ok(SolidPointLocation::Boundary);
                        }
                        Classification::Decided(ContourPointLocation::Outside) => {}
                        Classification::Uncertain(reason) => {
                            return Err(
                                GeometryError::PlanarClassificationUnresolved(reason).into()
                            );
                        }
                    }
                }
            }
        }

        if self.classify_point_against_shell(solid.outer, point)? == SolidPointLocation::Outside {
            return Ok(SolidPointLocation::Outside);
        }
        for void_shell in &solid.voids {
            if self.classify_point_against_shell(*void_shell, point)? == SolidPointLocation::Inside
            {
                return Ok(SolidPointLocation::Outside);
            }
        }
        Ok(SolidPointLocation::Inside)
    }

    fn classify_point_against_cylinder(
        &self,
        cylinder: &CertifiedCylinderShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &cylinder.origin;
        let axial = offset.dot(&cylinder.axis);
        let below = decided_model_order(compare_reals(&axial, &cylinder.v_min))?
            == std::cmp::Ordering::Less;
        let above = decided_model_order(compare_reals(&axial, &cylinder.v_max))?
            == std::cmp::Ordering::Greater;
        if below || above {
            return Ok(SolidPointLocation::Outside);
        }
        let radial = offset - cylinder.axis.clone() * &axial;
        let radial_squared = radial.norm_squared();
        let radius_squared = &cylinder.radius * &cylinder.radius;
        let radial_order = decided_model_order(compare_reals(&radial_squared, &radius_squared))?;
        if radial_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        if let Some(subtraction) = &cylinder.sphere_subtraction {
            let (center, radius) = match subtraction {
                CertifiedCylinderSphereSubtraction::Void { center, radius }
                | CertifiedCylinderSphereSubtraction::Component { center, radius, .. } => {
                    (center, radius)
                }
            };
            match decided_model_order(compare_reals(
                &(point - center).norm_squared(),
                &(radius * radius),
            ))? {
                std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                std::cmp::Ordering::Equal => return Ok(SolidPointLocation::Boundary),
                std::cmp::Ordering::Greater => {}
            }
        }
        let on_cap = decided_model_order(compare_reals(&axial, &cylinder.v_min))?
            == std::cmp::Ordering::Equal
            || decided_model_order(compare_reals(&axial, &cylinder.v_max))?
                == std::cmp::Ordering::Equal;
        if on_cap || radial_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    fn classify_point_against_revolution(
        &self,
        revolution: &CertifiedRevolutionShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &revolution.axis_origin;
        let axial = offset.dot(&revolution.axis);
        let radial = offset - revolution.axis.clone() * &axial;
        let radius = radial
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let profile_point = CurvePoint2::new(radius, axial);
        let location = match revolution
            .profile
            .classify_point(&profile_point, &CurvePolicy::certified())?
        {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        };
        match location {
            ContourPointLocation::Outside => return Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside => {}
        }
        for void in &revolution.voids {
            match void.classify_point(&profile_point, &CurvePolicy::certified())? {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(SolidPointLocation::Outside);
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(SolidPointLocation::Boundary);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(SolidPointLocation::Inside)
    }

    fn classify_point_against_loft(
        &self,
        loft: &CertifiedLoftShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let displacement = point - &loft.origin;
        let determinant = loft.u.dot(&loft.v.cross(&loft.height_axis));
        let parameter_u = (displacement.dot(&loft.v.cross(&loft.height_axis)) / &determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parameter_v = (loft.u.dot(&displacement.cross(&loft.height_axis)) / &determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parameter_t = (loft.u.dot(&loft.v.cross(&displacement)) / determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let t_order_min = decided_model_order(compare_reals(&parameter_t, &Real::zero()))?;
        let t_order_max = decided_model_order(compare_reals(&parameter_t, &Real::one()))?;
        if t_order_min == std::cmp::Ordering::Less || t_order_max == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let mut selected_span = None;
        for span in &loft.spans {
            let at_or_above = decided_model_order(compare_reals(&parameter_t, &span.start))?
                != std::cmp::Ordering::Less;
            let at_or_below = decided_model_order(compare_reals(&parameter_t, &span.end))?
                != std::cmp::Ordering::Greater;
            if at_or_above && at_or_below {
                selected_span = Some(span);
                break;
            }
        }
        let span = selected_span.ok_or(GeometryError::InvalidParameterDomain)?;
        let span_width = &span.end - &span.start;
        let local_parameter = ((&parameter_t - &span.start) / span_width)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let profile_point = CurvePoint2::new(parameter_u, parameter_v);
        let profile_location = match &span.interpolation {
            CertifiedLoftInterpolation::Homothetic {
                profile,
                scale,
                translation,
            } => {
                let factor = Real::one() + &local_parameter * (scale - Real::one());
                let normalized = CurvePoint2::new(
                    ((profile_point.x() - &local_parameter * translation.x()) / &factor)
                        .map_err(|_| GeometryError::ProjectiveDivision)?,
                    ((profile_point.y() - &local_parameter * translation.y()) / factor)
                        .map_err(|_| GeometryError::ProjectiveDivision)?,
                );
                match profile.classify_point(&normalized, &CurvePolicy::certified()) {
                    Classification::Decided(location) => location,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
            CertifiedLoftInterpolation::ConvexCorresponding { lower, upper } => {
                classify_convex_loft_section(lower, upper, &local_parameter, &profile_point)?
            }
        };
        match profile_location {
            ContourPointLocation::Outside => Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside
                if t_order_min == std::cmp::Ordering::Equal
                    || t_order_max == std::cmp::Ordering::Equal =>
            {
                Ok(SolidPointLocation::Boundary)
            }
            ContourPointLocation::Inside => Ok(SolidPointLocation::Inside),
        }
    }

    fn classify_point_against_curve_sweep(
        &self,
        sweep: &CertifiedCurveSweepShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let start = sweep.path.point_at(sweep.path.domain().start())?;
        let end = sweep.path.point_at(sweep.path.domain().end())?;
        let start_u = Vector3::from(sweep.u_path.point_at(sweep.u_path.domain().start())?);
        let start_v = Vector3::from(sweep.v_path.point_at(sweep.v_path.domain().start())?);
        let normal = start_u.cross(&start_v);
        let progress = normal.dot(&(end - &start));
        let parameter = (normal.dot(&(point - &start)) / &progress)
            .map_err(|_| QueryError::from(GeometryError::ProjectiveDivision))?;
        let lower_order = decided_model_order(compare_reals(&parameter, &Real::zero()))?;
        let upper_order = decided_model_order(compare_reals(&parameter, &Real::one()))?;
        if lower_order == std::cmp::Ordering::Less || upper_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let path_point = sweep.path.point_at(&parameter)?;
        let u = Vector3::from(sweep.u_path.point_at(&parameter)?);
        let v = Vector3::from(sweep.v_path.point_at(&parameter)?);
        let profile_point = project_point_to_plane_frame(point, &path_point, &u, &v)
            .map_err(build_error_geometry)?;
        let location = match sweep
            .profile
            .classify_point(&profile_point, &CurvePolicy::certified())
        {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        };
        match location {
            ContourPointLocation::Outside => return Ok(SolidPointLocation::Outside),
            ContourPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
            ContourPointLocation::Inside => {}
        }
        for hole in &sweep.holes {
            match hole.classify_point(&profile_point, &CurvePolicy::certified()) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(SolidPointLocation::Outside);
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(SolidPointLocation::Boundary);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        if lower_order == std::cmp::Ordering::Equal || upper_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    fn classify_point_against_sphere(
        &self,
        sphere: &CertifiedSphereShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        if let CertifiedSphereRegion::FiniteCylinder(region) = &sphere.region {
            return self.classify_point_against_sphere_finite_cylinder(sphere, region, point);
        }
        let radial_boundary = if let CertifiedSphereRegion::Radial(clip) = &sphere.region {
            let offset = point - &sphere.center;
            let axial = offset.dot(&clip.axis);
            let radial = offset - clip.axis.clone() * axial;
            let radial_order = decided_model_order(compare_reals(
                &radial.norm_squared(),
                &(&clip.radius * &clip.radius),
            ))?;
            if matches!(
                (clip.side, radial_order),
                (
                    CertifiedSphereRadialSide::Inside,
                    std::cmp::Ordering::Greater
                ) | (CertifiedSphereRadialSide::Outside, std::cmp::Ordering::Less)
            ) {
                return Ok(SolidPointLocation::Outside);
            }
            match radial_order {
                std::cmp::Ordering::Equal => true,
                std::cmp::Ordering::Less | std::cmp::Ordering::Greater => false,
            }
        } else {
            false
        };
        let clip_orders = if let CertifiedSphereRegion::Axial(clip) = &sphere.region {
            let height = (point - &sphere.center).dot(&clip.axis);
            let min = decided_model_order(compare_reals(&height, &clip.min))?;
            let max = decided_model_order(compare_reals(&height, &clip.max))?;
            if min == std::cmp::Ordering::Less || max == std::cmp::Ordering::Greater {
                return Ok(SolidPointLocation::Outside);
            }
            Some((min, max))
        } else {
            None
        };
        let distance_squared = (point - &sphere.center).norm_squared();
        let radius_squared = &sphere.radius * &sphere.radius;
        match decided_model_order(compare_reals(&distance_squared, &radius_squared))? {
            std::cmp::Ordering::Greater => return Ok(SolidPointLocation::Outside),
            std::cmp::Ordering::Equal => return Ok(SolidPointLocation::Boundary),
            std::cmp::Ordering::Less => {}
        }
        if radial_boundary
            || clip_orders.is_some_and(|(min, max)| {
                min == std::cmp::Ordering::Equal || max == std::cmp::Ordering::Equal
            })
        {
            return Ok(SolidPointLocation::Boundary);
        }
        for void in &sphere.voids {
            let location = match void {
                CertifiedSphereVoid::Sphere { center, radius } => {
                    match decided_model_order(compare_reals(
                        &(point - center).norm_squared(),
                        &(radius * radius),
                    ))? {
                        std::cmp::Ordering::Less => SolidPointLocation::Inside,
                        std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                        std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                    }
                }
                CertifiedSphereVoid::Cylinder(cylinder) => {
                    let offset = point - &cylinder.origin;
                    let axial = offset.dot(&cylinder.axis);
                    let min = decided_model_order(compare_reals(&axial, &cylinder.v_min))?;
                    let max = decided_model_order(compare_reals(&axial, &cylinder.v_max))?;
                    if min == std::cmp::Ordering::Less || max == std::cmp::Ordering::Greater {
                        SolidPointLocation::Outside
                    } else {
                        let radial = offset - cylinder.axis.clone() * &axial;
                        match decided_model_order(compare_reals(
                            &radial.norm_squared(),
                            &(&cylinder.radius * &cylinder.radius),
                        ))? {
                            std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                            std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                            std::cmp::Ordering::Less
                                if min == std::cmp::Ordering::Equal
                                    || max == std::cmp::Ordering::Equal =>
                            {
                                SolidPointLocation::Boundary
                            }
                            std::cmp::Ordering::Less => SolidPointLocation::Inside,
                        }
                    }
                }
            };
            match location {
                SolidPointLocation::Inside => return Ok(SolidPointLocation::Outside),
                SolidPointLocation::Boundary => return Ok(SolidPointLocation::Boundary),
                SolidPointLocation::Outside => {}
            }
        }
        Ok(SolidPointLocation::Inside)
    }

    fn classify_point_against_sphere_finite_cylinder(
        &self,
        sphere: &CertifiedSphereShell,
        cylinder: &CertifiedSphereFiniteCylinderRegion,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let sphere_order = decided_model_order(compare_reals(
            &(point - &sphere.center).norm_squared(),
            &(&sphere.radius * &sphere.radius),
        ))?;
        let offset = point - &cylinder.origin;
        let axial = offset.dot(&cylinder.axis);
        let axial_min = decided_model_order(compare_reals(&axial, &cylinder.v_min))?;
        let axial_max = decided_model_order(compare_reals(&axial, &cylinder.v_max))?;
        let cylinder_location =
            if axial_min == std::cmp::Ordering::Less || axial_max == std::cmp::Ordering::Greater {
                SolidPointLocation::Outside
            } else {
                let radial = offset - cylinder.axis.clone() * &axial;
                match decided_model_order(compare_reals(
                    &radial.norm_squared(),
                    &(&cylinder.radius * &cylinder.radius),
                ))? {
                    std::cmp::Ordering::Greater => SolidPointLocation::Outside,
                    std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
                    std::cmp::Ordering::Less
                        if axial_min == std::cmp::Ordering::Equal
                            || axial_max == std::cmp::Ordering::Equal =>
                    {
                        SolidPointLocation::Boundary
                    }
                    std::cmp::Ordering::Less => SolidPointLocation::Inside,
                }
            };
        let sphere_location = match sphere_order {
            std::cmp::Ordering::Less => SolidPointLocation::Inside,
            std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
            std::cmp::Ordering::Greater => SolidPointLocation::Outside,
        };
        Ok(match cylinder.operation {
            CertifiedSphereFiniteCylinderOperation::Union => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Inside, _) | (_, SolidPointLocation::Inside) => {
                        SolidPointLocation::Inside
                    }
                    (SolidPointLocation::Boundary, _) | (_, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Outside, SolidPointLocation::Outside) => {
                        SolidPointLocation::Outside
                    }
                }
            }
            CertifiedSphereFiniteCylinderOperation::Intersection => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Outside, _) | (_, SolidPointLocation::Outside) => {
                        SolidPointLocation::Outside
                    }
                    (SolidPointLocation::Boundary, _) | (_, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Inside, SolidPointLocation::Inside) => {
                        SolidPointLocation::Inside
                    }
                }
            }
            CertifiedSphereFiniteCylinderOperation::Difference => {
                match (sphere_location, cylinder_location) {
                    (SolidPointLocation::Outside, _) | (_, SolidPointLocation::Inside) => {
                        SolidPointLocation::Outside
                    }
                    (SolidPointLocation::Boundary, _)
                    | (SolidPointLocation::Inside, SolidPointLocation::Boundary) => {
                        SolidPointLocation::Boundary
                    }
                    (SolidPointLocation::Inside, SolidPointLocation::Outside) => {
                        SolidPointLocation::Inside
                    }
                }
            }
        })
    }

    fn classify_point_against_sphere_pair(
        &self,
        pair: &CertifiedSpherePairShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let first = decided_model_order(compare_reals(
            &(point - &pair.first_center).norm_squared(),
            &(&pair.first_radius * &pair.first_radius),
        ))?;
        let second = decided_model_order(compare_reals(
            &(point - &pair.second_center).norm_squared(),
            &(&pair.second_radius * &pair.second_radius),
        ))?;
        Ok(match pair.kind {
            CertifiedSpherePairKind::Union => {
                if first == std::cmp::Ordering::Less || second == std::cmp::Ordering::Less {
                    SolidPointLocation::Inside
                } else if first == std::cmp::Ordering::Equal || second == std::cmp::Ordering::Equal
                {
                    SolidPointLocation::Boundary
                } else {
                    SolidPointLocation::Outside
                }
            }
            CertifiedSpherePairKind::Intersection => {
                if first == std::cmp::Ordering::Greater || second == std::cmp::Ordering::Greater {
                    SolidPointLocation::Outside
                } else if first == std::cmp::Ordering::Equal || second == std::cmp::Ordering::Equal
                {
                    SolidPointLocation::Boundary
                } else {
                    SolidPointLocation::Inside
                }
            }
            CertifiedSpherePairKind::Difference => match (first, second) {
                (std::cmp::Ordering::Greater, _) => SolidPointLocation::Outside,
                (std::cmp::Ordering::Equal, std::cmp::Ordering::Less) => {
                    SolidPointLocation::Outside
                }
                (std::cmp::Ordering::Equal, _) => SolidPointLocation::Boundary,
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less) => SolidPointLocation::Outside,
                (std::cmp::Ordering::Less, std::cmp::Ordering::Equal) => {
                    SolidPointLocation::Boundary
                }
                (std::cmp::Ordering::Less, std::cmp::Ordering::Greater) => {
                    SolidPointLocation::Inside
                }
            },
        })
    }

    fn classify_point_against_torus(
        &self,
        torus: &CertifiedTorusShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &torus.center;
        let axial = offset.dot(&torus.axis);
        let region_boundary = match &torus.region {
            CertifiedTorusRegion::Whole => false,
            CertifiedTorusRegion::Axial { min, max } => {
                let min_order = decided_model_order(compare_reals(&axial, min))?;
                let max_order = decided_model_order(compare_reals(&axial, max))?;
                if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater
                {
                    return Ok(SolidPointLocation::Outside);
                }
                min_order == std::cmp::Ordering::Equal || max_order == std::cmp::Ordering::Equal
            }
            CertifiedTorusRegion::LongitudinalHalf { interior_normal } => {
                match decided_model_order(compare_reals(
                    &offset.dot(interior_normal),
                    &Real::zero(),
                ))? {
                    std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                    std::cmp::Ordering::Equal => true,
                    std::cmp::Ordering::Greater => false,
                }
            }
        };
        let radial = offset - torus.axis.clone() * &axial;
        let radial_squared = radial.norm_squared();
        let major_squared = &torus.major_radius * &torus.major_radius;
        let minor_squared = &torus.minor_radius * &torus.minor_radius;
        let implicit_base = &radial_squared + &axial * &axial + &major_squared - &minor_squared;
        let left = &implicit_base * &implicit_base;
        let right = Real::from(4) * major_squared * radial_squared;
        Ok(match decided_model_order(compare_reals(&left, &right))? {
            std::cmp::Ordering::Less if region_boundary => SolidPointLocation::Boundary,
            std::cmp::Ordering::Less => SolidPointLocation::Inside,
            std::cmp::Ordering::Equal => SolidPointLocation::Boundary,
            std::cmp::Ordering::Greater => SolidPointLocation::Outside,
        })
    }

    fn classify_point_against_prism(
        &self,
        prism: &CertifiedPrismShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let displacement = point - &prism.origin;
        let determinant = prism.u.dot(&prism.v.cross(&prism.extrusion));
        let planar_u = (displacement.dot(&prism.v.cross(&prism.extrusion)) / &determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let planar_v = (prism.u.dot(&displacement.cross(&prism.extrusion)) / &determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let extrusion_parameter = (prism.u.dot(&prism.v.cross(&displacement)) / determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let min_order =
            decided_model_order(compare_reals(&extrusion_parameter, &prism.parameter_min))?;
        let max_order =
            decided_model_order(compare_reals(&extrusion_parameter, &prism.parameter_max))?;
        if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let planar = CurvePoint2::new(planar_u, planar_v);
        let policy = CurvePolicy::certified();
        match prism.outer.classify_point(&planar, &policy) {
            Classification::Decided(ContourPointLocation::Outside) => {
                return Ok(SolidPointLocation::Outside);
            }
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(SolidPointLocation::Boundary);
            }
            Classification::Decided(ContourPointLocation::Inside) => {}
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        }
        for hole in &prism.holes {
            match hole.classify_point(&planar, &policy) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(SolidPointLocation::Outside);
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(SolidPointLocation::Boundary);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        if min_order == std::cmp::Ordering::Equal || max_order == std::cmp::Ordering::Equal {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    /// Maps one face-local pcurve parameter to the shared edge's canonical
    /// curve parameter.
    ///
    /// The result is exact for both affine relations and native circular
    /// angular-sweep relations. No inverse floating-point projection or
    /// tolerance search is performed.
    pub fn edge_parameter_at(
        &self,
        edge_use: EdgeUseId,
        pcurve_parameter: &Real,
    ) -> Result<Real, QueryError> {
        let edge_use_record = self
            .edge_use(edge_use)
            .ok_or(QueryError::InvalidReference {
                kind: EntityKind::EdgeUse,
                index: edge_use.index(),
            })?;
        let edge = self
            .edge(edge_use_record.edge)
            .expect("validated edge-use edge ID");
        let pcurve = self
            .pcurve(edge_use_record.pcurve)
            .expect("validated edge-use pcurve ID");
        edge_use_record
            .parameter_correspondence
            .edge_parameter(
                pcurve,
                &edge.domain,
                edge_use_record.direction,
                pcurve_parameter,
            )
            .map_err(QueryError::from)
    }

    /// Maps one shared edge parameter back to a face-local pcurve parameter.
    ///
    /// Native circular pcurves replay directed angular fraction through
    /// Hypercurve's exact rational-conic inverse; no sampled projection is
    /// used.
    pub fn pcurve_parameter_at(
        &self,
        edge_use: EdgeUseId,
        edge_parameter: &Real,
    ) -> Result<Real, QueryError> {
        let edge_use_record = self
            .edge_use(edge_use)
            .ok_or(QueryError::InvalidReference {
                kind: EntityKind::EdgeUse,
                index: edge_use.index(),
            })?;
        let edge = self
            .edge(edge_use_record.edge)
            .expect("validated edge-use edge ID");
        let pcurve = self
            .pcurve(edge_use_record.pcurve)
            .expect("validated edge-use pcurve ID");
        edge_use_record
            .parameter_correspondence
            .pcurve_parameter(
                pcurve,
                &edge.domain,
                edge_use_record.direction,
                edge_parameter,
            )
            .map_err(QueryError::from)
    }

    /// Returns a vertex by typed ID.
    pub fn vertex(&self, id: VertexId) -> Option<&Vertex> {
        self.data.vertices.get(id.index())
    }

    /// Returns a spatial curve by typed ID.
    pub fn curve(&self, id: Curve3Id) -> Option<&Curve3> {
        self.data.curves.get(id.index())
    }

    /// Returns a pcurve by typed ID.
    pub fn pcurve(&self, id: PcurveId) -> Option<&Pcurve> {
        self.data.pcurves.get(id.index())
    }

    /// Returns a surface by typed ID.
    pub fn surface(&self, id: SurfaceId) -> Option<&Surface> {
        self.data.surfaces.get(id.index())
    }

    /// Returns an edge by typed ID.
    pub fn edge(&self, id: EdgeId) -> Option<&Edge> {
        self.data.edges.get(id.index())
    }

    /// Returns an edge use by typed ID.
    pub fn edge_use(&self, id: EdgeUseId) -> Option<&EdgeUse> {
        self.data.edge_uses.get(id.index())
    }

    /// Returns a wire by typed ID.
    pub fn wire(&self, id: WireId) -> Option<&Wire> {
        self.data.wires.get(id.index())
    }

    /// Returns a face by typed ID.
    pub fn face(&self, id: FaceId) -> Option<&Face> {
        self.data.faces.get(id.index())
    }

    /// Returns a shell by typed ID.
    pub fn shell(&self, id: ShellId) -> Option<&Shell> {
        self.data.shells.get(id.index())
    }

    /// Returns a solid by typed ID.
    pub fn solid(&self, id: SolidId) -> Option<&Solid> {
        self.data.solids.get(id.index())
    }

    /// Returns edges incident to a vertex in deterministic insertion order.
    pub fn edges_at_vertex(&self, id: VertexId) -> Option<&[EdgeId]> {
        self.data.vertex_edges.get(id.index()).map(Vec::as_slice)
    }

    /// Returns all uses of an edge in deterministic insertion order.
    pub fn uses_of_edge(&self, id: EdgeId) -> Option<&[EdgeUseId]> {
        self.data
            .edge_uses_by_edge
            .get(id.index())
            .map(Vec::as_slice)
    }

    /// Returns the owning wire of an edge use.
    pub fn wire_of_edge_use(&self, id: EdgeUseId) -> Option<WireId> {
        self.data.edge_use_wire.get(id.index()).copied()
    }

    /// Returns the owning face of a wire.
    pub fn face_of_wire(&self, id: WireId) -> Option<FaceId> {
        self.data.wire_face.get(id.index()).copied()
    }

    /// Returns the owning shell of a face.
    pub fn shell_of_face(&self, id: FaceId) -> Option<ShellId> {
        self.data.face_shell.get(id.index()).copied()
    }

    /// Returns the owning solid of a shell, when it bounds a solid.
    pub fn solid_of_shell(&self, id: ShellId) -> Option<SolidId> {
        self.data.shell_solid.get(id.index()).copied().flatten()
    }

    fn signed_wire_double_area(&self, wire: WireId) -> Result<Real, GeometryError> {
        self.build_model_wire_contour(wire)?
            .signed_area()
            .map_err(GeometryError::from)?
            .map(|area| Real::from(2) * area)
            .ok_or(GeometryError::UnsupportedMeasurement)
    }

    fn signed_torus_wire_area(
        &self,
        wire: WireId,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let line = pcurve
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            let u_constant = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let v_constant = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if u_constant == v_constant {
                return Err(GeometryError::UnsupportedMeasurement);
            }
            if u_constant {
                let delta_v = line.end().y() - line.start().y();
                let delta_sin = line.end().y().clone().sin() - line.start().y().clone().sin();
                area += line.start().x().clone()
                    * minor_radius
                    * (major_radius * delta_v + minor_radius * delta_sin);
            }
        }
        Ok(area)
    }

    fn signed_sphere_wire_area(&self, wire: WireId, radius: &Real) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let line = self
                .pcurve(edge_use.pcurve)
                .expect("validated pcurve ID")
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            if real_values_equal(line.start().y(), line.end().y()).map_err(build_error_geometry)? {
                area += radius
                    * radius
                    * line.start().y().clone().sin()
                    * (line.end().x() - line.start().x());
            } else if !real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?
            {
                return Err(GeometryError::UnsupportedMeasurement);
            }
        }
        Ok(area)
    }

    fn signed_cone_wire_area(
        &self,
        wire: WireId,
        semi_angle: &Real,
    ) -> Result<Real, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut doubled_area = Real::zero();
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use_id).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            let line = pcurve
                .line_segment()
                .ok_or(GeometryError::UnsupportedMeasurement)?;
            let u_constant = real_values_equal(line.start().x(), line.end().x())
                .map_err(build_error_geometry)?;
            let v_constant = real_values_equal(line.start().y(), line.end().y())
                .map_err(build_error_geometry)?;
            if u_constant == v_constant {
                return Err(GeometryError::UnsupportedMeasurement);
            }
            if u_constant {
                doubled_area += line.start().x().clone()
                    * semi_angle.clone().sin()
                    * (line.end().y() * line.end().y() - line.start().y() * line.start().y());
            }
        }
        (doubled_area / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)
    }

    fn signed_shell_six_volume(&self, shell: ShellId) -> Real {
        let mut sum = Real::zero();
        for face_id in &self.shell(shell).expect("validated shell ID").faces {
            let face = self.face(*face_id).expect("validated face ID");
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated wire ID");
                let first_use = wire.edge_uses[0];
                let anchor_id = self.directed_vertices(first_use).0;
                let anchor = self.vertex(anchor_id).expect("validated vertex ID").point();
                for pair in wire.edge_uses[1..].windows(2) {
                    let first_id = self.directed_vertices(pair[0]).0;
                    let second_id = self.directed_vertices(pair[1]).0;
                    let first = self.vertex(first_id).expect("validated vertex ID").point();
                    let second = self.vertex(second_id).expect("validated vertex ID").point();
                    sum += Vector3::from(anchor.clone())
                        .dot(&Vector3::from(first.clone()).cross(&Vector3::from(second.clone())));
                }
            }
        }
        sum
    }

    fn classify_point_with_ray(
        &self,
        shell: ShellId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<Option<SolidPointLocation>, QueryError> {
        let mut crossings = 0_usize;
        for face_id in &self.shell(shell).expect("validated shell ID").faces {
            let face = self.face(*face_id).expect("validated face ID");
            let surface = self.surface(face.surface).expect("validated surface ID");
            let (u, v) = surface
                .plane_directions()
                .expect("the current validated face matrix contains only planes");
            let denominator = u.cross(v).dot(direction);
            if decided_model_order(compare_reals(&denominator, &Real::zero()))?
                == std::cmp::Ordering::Equal
            {
                continue;
            }
            let plane_value = self.face_plane_value(*face_id, point)?;
            let parameter =
                ((-plane_value) / denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
            match decided_model_order(compare_reals(&parameter, &Real::zero()))? {
                std::cmp::Ordering::Less | std::cmp::Ordering::Equal => continue,
                std::cmp::Ordering::Greater => {}
            }
            let intersection = point.clone() + direction.clone() * parameter;
            match self.classify_point_on_face(*face_id, &intersection)? {
                Classification::Decided(ContourPointLocation::Inside) => crossings += 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary)
                | Classification::Uncertain(hypercurve::UncertaintyReason::Boundary) => {
                    return Ok(None);
                }
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(Some(if crossings.is_multiple_of(2) {
            SolidPointLocation::Outside
        } else {
            SolidPointLocation::Inside
        }))
    }

    fn classify_point_against_shell(
        &self,
        shell: ShellId,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let directions = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::from_xyz(Real::one(), Real::from(2), Real::from(4)),
        ];
        for direction in directions {
            if let Some(location) = self.classify_point_with_ray(shell, point, &direction)? {
                return Ok(location);
            }
        }
        Err(
            GeometryError::PlanarClassificationUnresolved(hypercurve::UncertaintyReason::Boundary)
                .into(),
        )
    }

    fn classify_point_against_cone_frustum(
        &self,
        frustum: &CertifiedConeFrustumShell,
        point: &Point3,
    ) -> Result<SolidPointLocation, QueryError> {
        let offset = point - &frustum.apex;
        let axial = offset.dot(&frustum.axis);
        let cosine = frustum.semi_angle.clone().cos();
        let v = (&axial / cosine).map_err(|_| GeometryError::ProjectiveDivision)?;
        let min_order = decided_model_order(compare_reals(&v, &frustum.v_min))?;
        let max_order = decided_model_order(compare_reals(&v, &frustum.v_max))?;
        if min_order == std::cmp::Ordering::Less || max_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let radial = offset.clone() - frustum.axis.clone() * axial;
        let radius = v * frustum.semi_angle.clone().sin();
        let radial_order =
            decided_model_order(compare_reals(&radial.norm_squared(), &(&radius * &radius)))?;
        if radial_order == std::cmp::Ordering::Greater {
            return Ok(SolidPointLocation::Outside);
        }
        let region_boundary = match &frustum.region {
            CertifiedConeFrustumRegion::Whole => false,
            CertifiedConeFrustumRegion::LongitudinalHalf { interior_normal } => {
                match decided_model_order(compare_reals(
                    &offset.dot(interior_normal),
                    &Real::zero(),
                ))? {
                    std::cmp::Ordering::Less => return Ok(SolidPointLocation::Outside),
                    std::cmp::Ordering::Equal => true,
                    std::cmp::Ordering::Greater => false,
                }
            }
        };
        if radial_order == std::cmp::Ordering::Equal
            || min_order == std::cmp::Ordering::Equal
            || max_order == std::cmp::Ordering::Equal
            || region_boundary
        {
            Ok(SolidPointLocation::Boundary)
        } else {
            Ok(SolidPointLocation::Inside)
        }
    }

    fn face_plane_value(&self, face: FaceId, point: &Point3) -> Result<Real, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        let surface = self.surface(face.surface).expect("validated surface ID");
        let origin = surface
            .plane_origin()
            .expect("the current validated face matrix contains only planes");
        let (u, v) = surface
            .plane_directions()
            .expect("the current validated face matrix contains only planes");
        Ok(u.cross(v).dot(&(point - origin)))
    }

    fn classify_point_on_face(
        &self,
        face: FaceId,
        point: &Point3,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        let face_record = self.face(face).expect("validated face ID");
        let surface = self
            .surface(face_record.surface)
            .expect("validated surface ID");
        let origin = surface
            .plane_origin()
            .expect("the current validated face matrix contains only planes");
        let (u, v) = surface
            .plane_directions()
            .expect("the current validated face matrix contains only planes");
        let displacement = point - origin;
        let uu = u.dot(u);
        let uv = u.dot(v);
        let vv = v.dot(v);
        let du = displacement.dot(u);
        let dv = displacement.dot(v);
        let determinant = &uu * &vv - &uv * &uv;
        let parameter_u = ((&du * &vv - &dv * &uv) / &determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parameter_v = ((&dv * &uu - &du * &uv) / determinant)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let point = CurvePoint2::new(parameter_u, parameter_v);
        self.classify_surface_parameter_on_face(face, &point)
    }

    pub(crate) fn face_contours(&self, face: FaceId) -> Result<&[Contour2], GeometryError> {
        match self.data.face_contours[face.index()].get_or_init(|| self.build_face_contours(face)) {
            Ok(contours) => Ok(contours),
            Err(error) => Err(error.clone()),
        }
    }

    fn build_face_contours(&self, face: FaceId) -> Result<Vec<Contour2>, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        face.boundary_wires()
            .map(|wire| self.build_model_wire_contour(*wire))
            .collect()
    }

    fn build_model_wire_contour(&self, wire: WireId) -> Result<Contour2, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        let mut segments = Vec::with_capacity(wire.edge_uses.len());
        for edge_use in &wire.edge_uses {
            let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
            let pcurve = self.pcurve(edge_use.pcurve).expect("validated pcurve ID");
            segments.push(pcurve.segment()?);
        }
        Contour2::try_new(segments).map_err(GeometryError::from)
    }

    fn build_model_wire_curve_path(&self, wire: WireId) -> Result<CurvePath2, GeometryError> {
        let wire = self.wire(wire).expect("validated wire ID");
        CurvePath2::try_new(
            wire.edge_uses
                .iter()
                .map(|edge_use| {
                    let edge_use = self.edge_use(*edge_use).expect("validated edge-use ID");
                    self.pcurve(edge_use.pcurve)
                        .expect("validated pcurve ID")
                        .curve()
                        .clone()
                })
                .collect(),
        )
        .map_err(GeometryError::from)
    }

    pub(crate) fn classify_surface_parameter_on_face(
        &self,
        face: FaceId,
        point: &CurvePoint2,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        let face = self.face(face).expect("validated face ID");
        let FaceBoundary::Trimmed { outer, inner } = &face.boundary else {
            return Ok(Classification::Decided(ContourPointLocation::Inside));
        };
        let policy = CurvePolicy::certified();
        match self
            .build_model_wire_curve_path(*outer)?
            .classify_point(point, &policy)
            .map_err(GeometryError::from)?
        {
            Classification::Decided(ContourPointLocation::Outside) => {
                return Ok(Classification::Decided(ContourPointLocation::Outside));
            }
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
            Classification::Decided(ContourPointLocation::Inside) => {}
        }
        for wire in inner {
            match self
                .build_model_wire_curve_path(*wire)?
                .classify_point(point, &policy)
                .map_err(GeometryError::from)?
            {
                Classification::Decided(ContourPointLocation::Inside) => {
                    return Ok(Classification::Decided(ContourPointLocation::Outside));
                }
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(ContourPointLocation::Inside))
    }

    fn directed_vertices(&self, edge_use: EdgeUseId) -> (VertexId, VertexId) {
        let edge_use = self.edge_use(edge_use).expect("validated edge-use ID");
        let edge = self.edge(edge_use.edge).expect("validated edge ID");
        match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        }
    }

    pub(crate) fn certified_z_prism_profile(
        &self,
        solid_id: SolidId,
    ) -> Result<Option<CertifiedZPrismProfile>, GeometryError> {
        let Some(solid) = self.solid(solid_id) else {
            return Ok(None);
        };
        if !solid.voids.is_empty() {
            return Ok(None);
        }
        let shell = self
            .shell(solid.outer)
            .expect("validated solid shell reference");
        let certified_translation_family = self
            .data
            .certified_prisms
            .get(solid_id.index())
            .is_some_and(Option::is_some)
            || self
                .data
                .certified_cylinders
                .get(solid_id.index())
                .and_then(Option::as_ref)
                .is_some_and(|cylinder| cylinder.sphere_subtraction.is_none())
            || shell.faces.iter().all(|face| {
                matches!(
                    self.surface(self.face(*face).expect("validated shell face").surface())
                        .expect("validated face surface")
                        .kind(),
                    SurfaceKind::Plane | SurfaceKind::Extrusion
                )
            });
        if !certified_translation_family {
            return Ok(None);
        }
        let mut vertex_ids = HashSet::new();
        for face_id in &shell.faces {
            let face = self.face(*face_id).expect("validated shell face");
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated face wire");
                for edge_use_id in &wire.edge_uses {
                    let edge_use = self
                        .edge_use(*edge_use_id)
                        .expect("validated wire edge use");
                    let edge = self.edge(edge_use.edge).expect("validated edge reference");
                    vertex_ids.insert(edge.start);
                    vertex_ids.insert(edge.end);
                }
            }
        }
        let Some(first_id) = vertex_ids.iter().next() else {
            return Ok(None);
        };
        let first = self.vertex(*first_id).expect("validated vertex").point();
        let mut z_min = first.z.clone();
        let mut z_max = first.z.clone();
        for vertex_id in &vertex_ids {
            let z = &self.vertex(*vertex_id).expect("validated vertex").point().z;
            if decided_model_order(compare_reals(z, &z_min))? == std::cmp::Ordering::Less {
                z_min = z.clone();
            }
            if decided_model_order(compare_reals(z, &z_max))? == std::cmp::Ordering::Greater {
                z_max = z.clone();
            }
        }
        if decided_model_order(compare_reals(&z_min, &z_max))? != std::cmp::Ordering::Less {
            return Ok(None);
        }
        for vertex_id in &vertex_ids {
            let z = &self.vertex(*vertex_id).expect("validated vertex").point().z;
            let at_min =
                decided_model_order(compare_reals(z, &z_min))? == std::cmp::Ordering::Equal;
            let at_max =
                decided_model_order(compare_reals(z, &z_max))? == std::cmp::Ordering::Equal;
            if !at_min && !at_max {
                return Ok(None);
            }
        }

        for face_id in &shell.faces {
            let face = self.face(*face_id).expect("validated shell face");
            let mut profiles = Vec::with_capacity(face.inner().len() + 1);
            let mut is_top = true;
            for wire_id in face.boundary_wires() {
                let wire = self.wire(*wire_id).expect("validated cap wire");
                let mut segments = Vec::with_capacity(wire.edge_uses.len());
                for edge_use_id in &wire.edge_uses {
                    let (start, end) = self.directed_vertices(*edge_use_id);
                    let point = self.vertex(start).expect("validated vertex").point();
                    if decided_model_order(compare_reals(&point.z, &z_max))?
                        != std::cmp::Ordering::Equal
                    {
                        is_top = false;
                        break;
                    }
                    segments.push(self.projected_model_cap_segment(
                        *edge_use_id,
                        self.vertex(start).expect("validated vertex").point(),
                        self.vertex(end).expect("validated vertex").point(),
                    )?);
                }
                if !is_top {
                    break;
                }
                profiles.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
            }
            if is_top && !profiles.is_empty() {
                let outer = profiles.remove(0);
                return Ok(Some(CertifiedZPrismProfile {
                    outer,
                    holes: profiles,
                    z_min,
                    z_max,
                }));
            }
        }
        Ok(None)
    }

    pub(crate) fn certified_sphere_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedSphereProfile> {
        self.data
            .certified_spheres
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|sphere| {
                sphere.voids.is_empty() && matches!(sphere.region, CertifiedSphereRegion::Whole)
            })
            .map(|sphere| CertifiedSphereProfile {
                center: sphere.center.clone(),
                radius: sphere.radius.clone(),
            })
    }

    pub(crate) fn certified_cylinder_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedCylinderProfile> {
        self.data
            .certified_cylinders
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|cylinder| cylinder.sphere_subtraction.is_none())
            .map(|cylinder| CertifiedCylinderProfile {
                origin: cylinder.origin.clone(),
                axis: cylinder.axis.clone(),
                radius: cylinder.radius.clone(),
                v_min: cylinder.v_min.clone(),
                v_max: cylinder.v_max.clone(),
            })
    }

    pub(crate) fn certified_cone_frustum_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedConeFrustumProfile> {
        self.data
            .certified_cone_frustums
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|frustum| matches!(frustum.region, CertifiedConeFrustumRegion::Whole))
            .map(|frustum| CertifiedConeFrustumProfile {
                apex: frustum.apex.clone(),
                axis: frustum.axis.clone(),
                semi_angle: frustum.semi_angle.clone(),
                v_min: frustum.v_min.clone(),
                v_max: frustum.v_max.clone(),
            })
    }

    pub(crate) fn certified_torus_profile(&self, solid: SolidId) -> Option<CertifiedTorusProfile> {
        self.data
            .certified_tori
            .get(solid.index())
            .and_then(Option::as_ref)
            .filter(|torus| matches!(torus.region, CertifiedTorusRegion::Whole))
            .map(|torus| CertifiedTorusProfile {
                center: torus.center.clone(),
                axis: torus.axis.clone(),
                major_radius: torus.major_radius.clone(),
                minor_radius: torus.minor_radius.clone(),
            })
    }

    pub(crate) fn certified_revolution_profile(
        &self,
        solid: SolidId,
    ) -> Option<CertifiedRevolutionProfile> {
        let revolution = self
            .data
            .certified_revolutions
            .get(solid.index())
            .and_then(Option::as_ref)?;
        let profile = revolution.profile.native_contour()?.clone();
        let holes = revolution
            .voids
            .iter()
            .map(|void| void.native_contour().cloned())
            .collect::<Option<Vec<_>>>()?;
        Some(CertifiedRevolutionProfile {
            axis_origin: revolution.axis_origin.clone(),
            axis: revolution.axis.clone(),
            profile,
            holes,
        })
    }

    fn projected_model_cap_segment(
        &self,
        edge_use_id: EdgeUseId,
        start: &Point3,
        end: &Point3,
    ) -> Result<Segment2, GeometryError> {
        let edge_use = self
            .edge_use(edge_use_id)
            .expect("validated cap edge-use reference");
        let edge = self.edge(edge_use.edge).expect("validated cap edge");
        let curve = self.curve(edge.curve).expect("validated cap curve");
        match curve.kind() {
            Curve3Kind::Line => Ok(Segment2::Line(LineSeg2::try_new(
                CurvePoint2::new(start.x.clone(), start.y.clone()),
                CurvePoint2::new(end.x.clone(), end.y.clone()),
            )?)),
            Curve3Kind::CircleArc => {
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    unreachable!("circle kind carries ellipse-arc exact data");
                };
                let parameter = match edge_use.direction {
                    Direction::Forward => edge.domain.start(),
                    Direction::Reversed => edge.domain.end(),
                };
                let mut tangent = curve.derivative_at(parameter, 1)?.vector().clone();
                if edge_use.direction == Direction::Reversed {
                    tangent = -tangent;
                }
                let radial_x = &start.x - &data.center.x;
                let radial_y = &start.y - &data.center.y;
                let cross = radial_x * &tangent.0[1] - radial_y * &tangent.0[0];
                let clockwise = match decided_model_order(compare_reals(&cross, &Real::zero()))? {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Greater => false,
                    std::cmp::Ordering::Equal => return Err(GeometryError::InvalidArcSweep),
                };
                Ok(Segment2::Arc(CircularArc2::try_from_center(
                    CurvePoint2::new(start.x.clone(), start.y.clone()),
                    CurvePoint2::new(end.x.clone(), end.y.clone()),
                    CurvePoint2::new(data.center.x, data.center.y),
                    clockwise,
                )?))
            }
            _ => Err(GeometryError::UnsupportedPcurveContour),
        }
    }
}

fn single_validation_error(error: BuildError) -> ValidationReport {
    ValidationReport {
        errors: vec![error],
    }
}

impl Edit {
    /// Applies an orientation-preserving invertible affine transform in place.
    ///
    /// A failed transform leaves the previously staged snapshot untouched.
    pub fn transform(&mut self, transform: &Matrix4) -> Result<&mut Self, EditError> {
        let transformed = self.staged.transformed(transform)?;
        self.staged = transformed;
        Ok(self)
    }

    /// Replaces one exact vertex point while retaining its stable ID.
    pub fn replace_vertex(&mut self, id: VertexId, point: Point3) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(vertex) = data.vertices.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Vertex,
                index: id.index(),
            });
        };
        vertex.point = point;
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one spatial curve while retaining its stable ID.
    pub fn replace_curve(&mut self, id: Curve3Id, curve: Curve3) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.curves.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Curve3,
                index: id.index(),
            });
        };
        *slot = curve;
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one face-local pcurve while retaining its stable ID.
    pub fn replace_pcurve(&mut self, id: PcurveId, pcurve: Pcurve) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.pcurves.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Pcurve,
                index: id.index(),
            });
        };
        *slot = pcurve;
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one support surface while retaining its stable ID.
    pub fn replace_surface(
        &mut self,
        id: SurfaceId,
        surface: Surface,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.surfaces.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Surface,
                index: id.index(),
            });
        };
        *slot = surface;
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one edge's topology, carrier, and retained domain.
    pub fn replace_edge(
        &mut self,
        id: EdgeId,
        start: VertexId,
        end: VertexId,
        curve: Curve3Id,
        domain: ParameterDomain,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.edges.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Edge,
                index: id.index(),
            });
        };
        *slot = Edge {
            start,
            end,
            curve,
            domain,
        };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one oriented edge use and its exact parameter correspondence.
    pub fn replace_edge_use(
        &mut self,
        id: EdgeUseId,
        edge: EdgeId,
        direction: Direction,
        pcurve: PcurveId,
        parameter_correspondence: ParameterCorrespondence,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.edge_uses.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::EdgeUse,
                index: id.index(),
            });
        };
        *slot = EdgeUse {
            edge,
            direction,
            pcurve,
            parameter_correspondence,
        };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one wire's ordered edge-use sequence.
    pub fn replace_wire(
        &mut self,
        id: WireId,
        edge_uses: Vec<EdgeUseId>,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.wires.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Wire,
                index: id.index(),
            });
        };
        *slot = Wire { edge_uses };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one face's surface, orientation, and boundary wires.
    pub fn replace_face(
        &mut self,
        id: FaceId,
        surface: SurfaceId,
        orientation: Orientation,
        outer: WireId,
        inner: Vec<WireId>,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.faces.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Face,
                index: id.index(),
            });
        };
        *slot = Face {
            surface,
            orientation,
            boundary: FaceBoundary::Trimmed { outer, inner },
        };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one face with the complete closed support surface.
    pub fn replace_whole_face(
        &mut self,
        id: FaceId,
        surface: SurfaceId,
        orientation: Orientation,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.faces.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Face,
                index: id.index(),
            });
        };
        *slot = Face {
            surface,
            orientation,
            boundary: FaceBoundary::WholeSurface,
        };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one shell's ordered face set.
    pub fn replace_shell(
        &mut self,
        id: ShellId,
        faces: Vec<FaceId>,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.shells.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Shell,
                index: id.index(),
            });
        };
        *slot = Shell { faces };
        reset_model_caches(data);
        Ok(self)
    }

    /// Replaces one solid's outer and void shells.
    pub fn replace_solid(
        &mut self,
        id: SolidId,
        outer: ShellId,
        voids: Vec<ShellId>,
    ) -> Result<&mut Self, EditError> {
        let data = Arc::make_mut(&mut self.staged.data);
        let Some(slot) = data.solids.get_mut(id.index()) else {
            return Err(EditError::InvalidReference {
                kind: EntityKind::Solid,
                index: id.index(),
            });
        };
        *slot = Solid { outer, voids };
        reset_model_caches(data);
        Ok(self)
    }

    /// Commits this transaction as a new immutable model.
    pub fn commit(self) -> Result<Model, EditError> {
        self.staged.revalidated().map_err(EditError::Validation)
    }
}

fn reset_model_caches(data: &mut ModelData) {
    data.bounds = OnceLock::new();
    data.face_contours = (0..data.faces.len()).map(|_| OnceLock::new()).collect();
}

struct FaceLineMaterial {
    intervals: Vec<(Real, Real)>,
    contacts: Vec<Real>,
}

/// Incremental constructor for a canonical validated model.
#[derive(Debug, Default)]
pub struct ModelBuilder {
    vertices: Vec<Vertex>,
    curves: Vec<Curve3>,
    pcurves: Vec<Pcurve>,
    surfaces: Vec<Surface>,
    edges: Vec<Edge>,
    edge_uses: Vec<EdgeUse>,
    wires: Vec<Wire>,
    faces: Vec<Face>,
    shells: Vec<Shell>,
    solids: Vec<Solid>,
    vertex_edges: Vec<Vec<EdgeId>>,
    edge_uses_by_edge: Vec<Vec<EdgeUseId>>,
    edge_use_wire: Vec<Option<WireId>>,
    wire_face: Vec<Option<FaceId>>,
    face_shell: Vec<Option<ShellId>>,
    shell_solid: Vec<Option<SolidId>>,
}

impl ModelBuilder {
    /// Constructs an empty model builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an exact model-space vertex.
    pub fn vertex(&mut self, point: Point3) -> Result<VertexId, BuildError> {
        let id = VertexId::from_index(self.vertices.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Vertex))?;
        self.vertices.push(Vertex { point });
        self.vertex_edges.push(Vec::new());
        Ok(id)
    }

    /// Adds an immutable spatial curve.
    pub fn curve(&mut self, curve: Curve3) -> Result<Curve3Id, BuildError> {
        let id = Curve3Id::from_index(self.curves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Curve3))?;
        self.curves.push(curve);
        Ok(id)
    }

    /// Adds an immutable face-local pcurve.
    pub fn pcurve(&mut self, pcurve: Pcurve) -> Result<PcurveId, BuildError> {
        let id = PcurveId::from_index(self.pcurves.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Pcurve))?;
        self.pcurves.push(pcurve);
        Ok(id)
    }

    /// Adds an immutable parameterized surface.
    pub fn surface(&mut self, surface: Surface) -> Result<SurfaceId, BuildError> {
        let id = SurfaceId::from_index(self.surfaces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Surface))?;
        self.surfaces.push(surface);
        Ok(id)
    }

    /// Adds a topological edge with exact geometry and a retained subdomain.
    pub fn edge(
        &mut self,
        start: VertexId,
        end: VertexId,
        curve: Curve3Id,
        domain: ParameterDomain,
    ) -> Result<EdgeId, BuildError> {
        if start == end {
            return Err(BuildError::DegenerateEdge);
        }
        let start_point = self.vertex_ref(start)?.point();
        let end_point = self.vertex_ref(end)?.point();
        let curve_ref = self.curve_ref(curve)?;
        let curve_start = curve_ref.point_at(domain.start())?;
        let curve_end = curve_ref.point_at(domain.end())?;
        require_point_equal(
            start_point,
            &curve_start,
            BuildError::EdgeEndpointMismatch {
                endpoint: Endpoint::Start,
            },
        )?;
        require_point_equal(
            end_point,
            &curve_end,
            BuildError::EdgeEndpointMismatch {
                endpoint: Endpoint::End,
            },
        )?;

        let id = EdgeId::from_index(self.edges.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Edge))?;
        self.edges.push(Edge {
            start,
            end,
            curve,
            domain,
        });
        self.vertex_edges[start.index()].push(id);
        self.vertex_edges[end.index()].push(id);
        self.edge_uses_by_edge.push(Vec::new());
        Ok(id)
    }

    /// Adds an oriented edge use with face-local pcurve geometry.
    pub fn edge_use(
        &mut self,
        edge: EdgeId,
        direction: Direction,
        pcurve: PcurveId,
        parameter_correspondence: ParameterCorrespondence,
    ) -> Result<EdgeUseId, BuildError> {
        self.edge_ref(edge)?;
        self.pcurve_ref(pcurve)?;
        let id = EdgeUseId::from_index(self.edge_uses.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::EdgeUse))?;
        self.edge_uses.push(EdgeUse {
            edge,
            direction,
            pcurve,
            parameter_correspondence,
        });
        self.edge_uses_by_edge[edge.index()].push(id);
        self.edge_use_wire.push(None);
        Ok(id)
    }

    /// Adds an ordered, connected, closed wire.
    pub fn wire(&mut self, edge_uses: Vec<EdgeUseId>) -> Result<WireId, BuildError> {
        if edge_uses.is_empty() {
            return Err(BuildError::EmptyWire);
        }
        let mut unique = HashSet::with_capacity(edge_uses.len());
        for edge_use in &edge_uses {
            self.edge_use_ref(*edge_use)?;
            if !unique.insert(*edge_use) {
                return Err(BuildError::DuplicateEdgeUse(*edge_use));
            }
            if self.edge_use_wire[edge_use.index()].is_some() {
                return Err(BuildError::EdgeUseAlreadyOwned(*edge_use));
            }
        }
        for (index, pair) in edge_uses.windows(2).enumerate() {
            let (_, first_end) = self.directed_vertices(pair[0])?;
            let (second_start, _) = self.directed_vertices(pair[1])?;
            if first_end != second_start {
                return Err(BuildError::DisconnectedWire { at: index });
            }
        }
        let (first_start, _) = self.directed_vertices(edge_uses[0])?;
        let (_, last_end) = self.directed_vertices(*edge_uses.last().expect("nonempty wire"))?;
        if first_start != last_end {
            return Err(BuildError::OpenWire);
        }

        let id = WireId::from_index(self.wires.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        self.wires.push(Wire {
            edge_uses: edge_uses.clone(),
        });
        self.wire_face.push(None);
        for edge_use in edge_uses {
            self.edge_use_wire[edge_use.index()] = Some(id);
        }
        Ok(id)
    }

    /// Adds a face and certifies edge/pcurve/surface agreement.
    pub fn face(
        &mut self,
        surface: SurfaceId,
        orientation: Orientation,
        outer: WireId,
        inner: Vec<WireId>,
    ) -> Result<FaceId, BuildError> {
        let spherical = self.surface_ref(surface)?.kind() == SurfaceKind::Sphere;
        self.wire_ref(outer)?;
        let mut wires = Vec::with_capacity(inner.len() + 1);
        wires.push(outer);
        wires.extend(inner.iter().copied());
        let mut unique = HashSet::with_capacity(wires.len());
        for wire in &wires {
            self.wire_ref(*wire)?;
            if !unique.insert(*wire) {
                return Err(BuildError::DuplicateWire(*wire));
            }
            if self.wire_face[wire.index()].is_some() {
                return Err(BuildError::WireAlreadyOwned(*wire));
            }
            self.validate_wire_image(*wire, surface)?;
        }
        if spherical {
            if inner.len() > 1 {
                return Err(BuildError::UnsupportedSphericalTrim(outer));
            }
            if let Some(upper_wire) = inner.first() {
                self.validate_spherical_trim(outer, orientation)?;
                self.validate_spherical_trim(*upper_wire, orientation)?;
                let (lower_latitude, lower_direction) = self.spherical_trim_coordinates(outer)?;
                let (upper_latitude, upper_direction) =
                    self.spherical_trim_coordinates(*upper_wire)?;
                let expected_lower = match orientation {
                    Orientation::Forward => std::cmp::Ordering::Greater,
                    Orientation::Reversed => std::cmp::Ordering::Less,
                };
                if decided_model_order(compare_reals(&lower_latitude, &upper_latitude))?
                    != std::cmp::Ordering::Less
                    || lower_direction != expected_lower
                    || upper_direction == expected_lower
                {
                    return Err(BuildError::InvalidSphericalTrim(outer));
                }
            } else {
                match self.validate_spherical_trim(outer, orientation) {
                    Ok(()) => {}
                    Err(BuildError::UnsupportedSphericalTrim(_)) => {
                        self.validate_wire_simplicity(outer)?;
                        self.validate_wire_orientation(outer, orientation, true, surface)?;
                    }
                    Err(error) => return Err(error),
                }
            }
        } else {
            for wire in &wires {
                self.validate_wire_simplicity(*wire)?;
            }
            self.validate_wire_orientation(outer, orientation, true, surface)?;
            for wire in &inner {
                self.validate_wire_orientation(*wire, orientation, false, surface)?;
            }
            self.validate_wire_nesting(outer, &inner)?;
        }

        let id = FaceId::from_index(self.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        self.faces.push(Face {
            surface,
            orientation,
            boundary: FaceBoundary::Trimmed { outer, inner },
        });
        self.face_shell.push(None);
        for wire in wires {
            self.wire_face[wire.index()] = Some(id);
        }
        Ok(id)
    }

    /// Adds the complete closed support surface as one boundaryless face.
    ///
    /// This is currently defined for spheres. It represents the closed
    /// geometric carrier directly instead of inventing seam edges or
    /// zero-length pole topology.
    pub fn whole_face(
        &mut self,
        surface: SurfaceId,
        orientation: Orientation,
    ) -> Result<FaceId, BuildError> {
        let support = self.surface_ref(surface)?;
        if support.kind() != SurfaceKind::Sphere {
            return Err(BuildError::UnsupportedWholeSurface(support.kind()));
        }
        let id = FaceId::from_index(self.faces.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Face))?;
        self.faces.push(Face {
            surface,
            orientation,
            boundary: FaceBoundary::WholeSurface,
        });
        self.face_shell.push(None);
        Ok(id)
    }

    /// Adds one edge-connected shell.
    pub fn shell(&mut self, faces: Vec<FaceId>) -> Result<ShellId, BuildError> {
        if faces.is_empty() {
            return Err(BuildError::EmptyShell);
        }
        let mut unique = HashSet::with_capacity(faces.len());
        for face in &faces {
            self.face_ref(*face)?;
            if !unique.insert(*face) {
                return Err(BuildError::DuplicateFace(*face));
            }
            if self.face_shell[face.index()].is_some() {
                return Err(BuildError::FaceAlreadyOwned(*face));
            }
        }
        if !self.faces_connected(&faces)? {
            return Err(BuildError::DisconnectedShell);
        }

        let id = ShellId::from_index(self.shells.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Shell))?;
        self.shells.push(Shell {
            faces: faces.clone(),
        });
        self.shell_solid.push(None);
        for face in faces {
            self.face_shell[face.index()] = Some(id);
        }
        Ok(id)
    }

    /// Adds a regular solid bounded by one certified closed shell.
    pub fn solid(&mut self, outer: ShellId, voids: Vec<ShellId>) -> Result<SolidId, BuildError> {
        self.shell_ref(outer)?;
        let mut shells = Vec::with_capacity(voids.len() + 1);
        shells.push(outer);
        shells.extend(voids.iter().copied());
        let mut unique = HashSet::with_capacity(shells.len());
        for shell in &shells {
            self.shell_ref(*shell)?;
            if !unique.insert(*shell) {
                return Err(BuildError::DuplicateShell(*shell));
            }
            if self.shell_solid[shell.index()].is_some() {
                return Err(BuildError::ShellAlreadyOwned(*shell));
            }
            self.validate_closed_shell(*shell)?;
        }
        let candidate = Solid {
            outer,
            voids: voids.clone(),
        };
        let sphere = self.certified_sphere_solid(&candidate)?;
        let sphere_pair = self.certified_sphere_pair_shell(outer)?;
        let revolution = self.certified_revolution_shell(outer)?;
        let cylinder = self.certified_cylinder_solid(&candidate)?;
        let analytic_closed =
            sphere.is_some() || sphere_pair.is_some() || revolution.is_some() || cylinder.is_some();
        let simple_prism = if analytic_closed {
            false
        } else {
            self.certify_simple_prism_shell(outer)?
                || self.certify_internally_partitioned_prism_shell(outer)?
        };
        let line_arc_prism = if analytic_closed || simple_prism {
            false
        } else {
            self.certify_line_arc_prism_shell(outer)?
        };
        let curve_sweep = if analytic_closed || simple_prism || line_arc_prism {
            None
        } else {
            self.certified_curve_sweep_shell(outer)?
        };
        let loft = if analytic_closed || simple_prism || line_arc_prism || curve_sweep.is_some() {
            None
        } else {
            self.certified_loft_shell(outer)?
        };
        let convex_planar = if analytic_closed
            || simple_prism
            || line_arc_prism
            || curve_sweep.is_some()
            || loft.is_some()
        {
            false
        } else {
            self.certify_convex_planar_shell(outer)?
        };
        let general_planar = if analytic_closed
            || simple_prism
            || line_arc_prism
            || curve_sweep.is_some()
            || loft.is_some()
            || convex_planar
        {
            false
        } else {
            self.certify_planar_shell_non_self_intersection(outer)?
        };
        if !analytic_closed {
            self.validate_outer_shell_orientation(outer)?;
        }
        if !analytic_closed
            && !simple_prism
            && !line_arc_prism
            && self.certified_cone_frustum_shell(outer)?.is_none()
            && self.certified_torus_shell(outer)?.is_none()
            && curve_sweep.is_none()
            && loft.is_none()
            && !convex_planar
            && !general_planar
        {
            return Err(BuildError::UnsupportedSolidShell(outer));
        }
        for void_shell in &voids {
            let sphere_void = self
                .certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
                .is_some();
            let revolution_void = self
                .certified_oriented_revolution_shell(*void_shell, Orientation::Reversed)?
                .is_some();
            let cylinder_void = self
                .certified_oriented_cylinder_shell(*void_shell, Orientation::Reversed)?
                .is_some();
            if !sphere_void && !revolution_void && !cylinder_void {
                self.validate_void_shell_orientation(*void_shell)?;
            }
            let simple_prism_void = if sphere_void || revolution_void || cylinder_void {
                false
            } else {
                self.certify_simple_prism_shell(*void_shell)?
            };
            let general_planar_void =
                if sphere_void || revolution_void || cylinder_void || simple_prism_void {
                    false
                } else {
                    self.certify_planar_shell_non_self_intersection(*void_shell)?
                };
            if !sphere_void
                && !revolution_void
                && !cylinder_void
                && !simple_prism_void
                && !general_planar_void
            {
                return Err(BuildError::UnsupportedSolidShell(*void_shell));
            }
        }
        self.validate_void_shell_nesting(outer, &voids)?;

        let id = SolidId::from_index(self.solids.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Solid))?;
        self.solids.push(Solid {
            outer,
            voids: voids.clone(),
        });
        for shell in shells {
            self.shell_solid[shell.index()] = Some(id);
        }
        Ok(id)
    }

    /// Validates global ownership and commits an immutable model.
    pub fn finish(self) -> Result<Model, ValidationReport> {
        let mut errors = Vec::new();
        for (index, owner) in self.edge_use_wire.iter().enumerate() {
            if owner.is_none() {
                errors.push(BuildError::OrphanEdgeUse(EdgeUseId(index as u32)));
            }
        }
        for (index, owner) in self.wire_face.iter().enumerate() {
            if owner.is_none() {
                errors.push(BuildError::OrphanWire(WireId(index as u32)));
            }
        }
        for (index, owner) in self.face_shell.iter().enumerate() {
            if owner.is_none() {
                errors.push(BuildError::OrphanFace(FaceId(index as u32)));
            }
        }
        if !errors.is_empty() {
            return Err(ValidationReport { errors });
        }
        let face_count = self.faces.len();
        let certified_prisms = self
            .solids
            .iter()
            .map(|solid| {
                if solid.voids.is_empty() {
                    self.certified_prism_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_cylinders = self
            .solids
            .iter()
            .map(|solid| self.certified_cylinder_solid(solid))
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_lofts = self
            .solids
            .iter()
            .enumerate()
            .map(|(index, solid)| {
                if solid.voids.is_empty() && certified_prisms[index].is_none() {
                    self.certified_loft_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_curve_sweeps = self
            .solids
            .iter()
            .map(|solid| {
                if solid.voids.is_empty() {
                    self.certified_curve_sweep_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_spheres = self
            .solids
            .iter()
            .map(|solid| self.certified_sphere_solid(solid))
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_sphere_pairs = self
            .solids
            .iter()
            .map(|solid| {
                if solid.voids.is_empty() {
                    self.certified_sphere_pair_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_cone_frustums = self
            .solids
            .iter()
            .map(|solid| {
                if solid.voids.is_empty() {
                    self.certified_cone_frustum_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_tori = self
            .solids
            .iter()
            .map(|solid| {
                if solid.voids.is_empty() {
                    self.certified_torus_shell(solid.outer)
                } else {
                    Ok(None)
                }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        let certified_revolutions = self
            .solids
            .iter()
            .map(|solid| self.certified_revolution_solid(solid))
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| ValidationReport {
                errors: vec![error],
            })?;
        Ok(Model {
            data: Arc::new(ModelData {
                vertices: self.vertices,
                curves: self.curves,
                pcurves: self.pcurves,
                surfaces: self.surfaces,
                edges: self.edges,
                edge_uses: self.edge_uses,
                wires: self.wires,
                faces: self.faces,
                shells: self.shells,
                solids: self.solids,
                vertex_edges: self.vertex_edges,
                edge_uses_by_edge: self.edge_uses_by_edge,
                edge_use_wire: self.edge_use_wire.into_iter().flatten().collect(),
                wire_face: self.wire_face.into_iter().flatten().collect(),
                face_shell: self.face_shell.into_iter().flatten().collect(),
                shell_solid: self.shell_solid,
                certified_cylinders,
                certified_spheres,
                certified_sphere_pairs,
                certified_cone_frustums,
                certified_tori,
                certified_revolutions,
                certified_lofts,
                certified_curve_sweeps,
                certified_prisms,
                bounds: OnceLock::new(),
                face_contours: (0..face_count).map(|_| OnceLock::new()).collect(),
            }),
        })
    }

    fn vertex_ref(&self, id: VertexId) -> Result<&Vertex, BuildError> {
        self.vertices
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Vertex,
                index: id.index(),
            })
    }

    fn curve_ref(&self, id: Curve3Id) -> Result<&Curve3, BuildError> {
        self.curves
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Curve3,
                index: id.index(),
            })
    }

    fn pcurve_ref(&self, id: PcurveId) -> Result<&Pcurve, BuildError> {
        self.pcurves
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Pcurve,
                index: id.index(),
            })
    }

    fn surface_ref(&self, id: SurfaceId) -> Result<&Surface, BuildError> {
        self.surfaces
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Surface,
                index: id.index(),
            })
    }

    fn edge_ref(&self, id: EdgeId) -> Result<&Edge, BuildError> {
        self.edges
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Edge,
                index: id.index(),
            })
    }

    fn edge_use_ref(&self, id: EdgeUseId) -> Result<&EdgeUse, BuildError> {
        self.edge_uses
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::EdgeUse,
                index: id.index(),
            })
    }

    fn wire_ref(&self, id: WireId) -> Result<&Wire, BuildError> {
        self.wires
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Wire,
                index: id.index(),
            })
    }

    fn face_ref(&self, id: FaceId) -> Result<&Face, BuildError> {
        self.faces
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Face,
                index: id.index(),
            })
    }

    fn shell_ref(&self, id: ShellId) -> Result<&Shell, BuildError> {
        self.shells
            .get(id.index())
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Shell,
                index: id.index(),
            })
    }

    fn directed_vertices(&self, edge_use: EdgeUseId) -> Result<(VertexId, VertexId), BuildError> {
        let edge_use = self.edge_use_ref(edge_use)?;
        let edge = self.edge_ref(edge_use.edge)?;
        Ok(match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        })
    }

    fn validate_wire_image(&self, wire: WireId, surface: SurfaceId) -> Result<(), BuildError> {
        let surface = self.surface_ref(surface)?;
        let wire = self.wire_ref(wire)?;
        for edge_use_id in &wire.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use_id)?;
            let edge = self.edge_ref(edge_use.edge)?;
            let curve = self.curve_ref(edge.curve)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;

            let pcurve_parameters = [pcurve.domain_start(), pcurve.domain_end()];
            let expected_edge_parameters = match edge_use.direction {
                Direction::Forward => [edge.domain.start(), edge.domain.end()],
                Direction::Reversed => [edge.domain.end(), edge.domain.start()],
            };
            for (index, (pcurve_parameter, expected_edge_parameter)) in pcurve_parameters
                .into_iter()
                .zip(expected_edge_parameters)
                .enumerate()
            {
                let endpoint = if index == 0 {
                    Endpoint::Start
                } else {
                    Endpoint::End
                };
                let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                    pcurve,
                    &edge.domain,
                    edge_use.direction,
                    pcurve_parameter,
                )?;
                require_real_equal(
                    &edge_parameter,
                    expected_edge_parameter,
                    BuildError::ParameterCorrespondenceMismatch { endpoint },
                )?;
                let edge_point = curve.point_at(&edge_parameter)?;
                let surface_parameter = pcurve.point_at(pcurve_parameter)?;
                let surface_point = surface.point_at(&surface_parameter)?;
                require_point_equal(
                    &edge_point,
                    &surface_point,
                    BuildError::EdgeUseImageMismatch { endpoint },
                )?;
            }

            match (
                curve.kind(),
                pcurve.kind(),
                surface.kind(),
                &edge_use.parameter_correspondence,
            ) {
                (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Plane,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    // Both composed images are affine. Exact endpoint equality
                    // therefore certifies the complete interval.
                }
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::CircularArc,
                    SurfaceKind::Plane,
                    ParameterCorrespondence::AngularSweep,
                ) => {
                    self.validate_planar_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::RationalBezier,
                    CurveFamily2::RationalBezier,
                    SurfaceKind::Plane,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Nurbs,
                    CurveFamily2::Nurbs,
                    SurfaceKind::Plane,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_planar_spline_image(curve, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Cylinder,
                    ParameterCorrespondence::Affine { .. },
                ) => self.validate_cylinder_axial_line_image(pcurve)?,
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Cylinder,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_cylinder_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Sphere,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_sphere_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Torus,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_torus_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Cone,
                    ParameterCorrespondence::Affine { .. },
                ) => self.validate_cone_generator_image(pcurve)?,
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Cone,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_cone_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Extrusion,
                    ParameterCorrespondence::Affine { .. },
                ) => self.validate_extrusion_line_image(pcurve, surface)?,
                (
                    Curve3Kind::RationalBezier | Curve3Kind::Nurbs,
                    CurveFamily2::Line,
                    SurfaceKind::Extrusion,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_extrusion_profile_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Extrusion,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_extrusion_circle_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Revolution,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::RationalBezier,
                    CurveFamily2::Line,
                    SurfaceKind::Revolution,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Nurbs,
                    CurveFamily2::Line,
                    SurfaceKind::Revolution,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_revolution_meridian_image(curve, edge, edge_use, pcurve, surface)?
                }
                (
                    Curve3Kind::CircleArc,
                    CurveFamily2::Line,
                    SurfaceKind::Revolution,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    let line = pcurve
                        .line_segment()
                        .expect("line pcurve kind carries line geometry");
                    if real_values_equal(line.start().x(), line.end().x())? {
                        self.validate_revolution_meridian_image(
                            curve, edge, edge_use, pcurve, surface,
                        )?;
                    } else {
                        self.validate_revolution_circle_image(
                            curve, edge, edge_use, pcurve, surface,
                        )?;
                    }
                }
                (
                    Curve3Kind::RationalBezier,
                    CurveFamily2::Line,
                    SurfaceKind::RationalBezier,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::RationalBezier,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Nurbs,
                    CurveFamily2::Line,
                    SurfaceKind::Nurbs,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Line,
                    CurveFamily2::Line,
                    SurfaceKind::Nurbs,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_tensor_surface_iso_image(curve, edge, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::RationalBezier,
                    CurveFamily2::RationalBezier,
                    SurfaceKind::RationalBezier,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_rational_tensor_graph_image(curve, edge_use, pcurve, surface)?;
                }
                (
                    Curve3Kind::Nurbs,
                    CurveFamily2::RationalBezier,
                    SurfaceKind::Nurbs,
                    ParameterCorrespondence::Affine { .. },
                )
                | (
                    Curve3Kind::Nurbs,
                    CurveFamily2::Nurbs,
                    SurfaceKind::Nurbs,
                    ParameterCorrespondence::Affine { .. },
                ) => {
                    self.validate_nurbs_tensor_graph_image(curve, edge_use, pcurve, surface)?;
                }
                _ => {
                    return Err(BuildError::UnsupportedEdgeUseAgreement {
                        curve: curve.kind(),
                        pcurve: pcurve.kind(),
                        surface: surface.kind(),
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_planar_spline_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let lift = |point: &CurvePoint2| {
            surface.point_at(&Point2::new(point.x().clone(), point.y().clone()))
        };
        let expected = match pcurve.curve().geometry() {
            CurveGeometry2::RationalBezier(planar) => Curve3::rational_bezier(
                planar
                    .control_points()
                    .iter()
                    .map(lift)
                    .collect::<Result<Vec<_>, _>>()?,
                planar.weights().to_vec(),
            )?,
            CurveGeometry2::Nurbs(planar) => Curve3::nurbs(
                planar.degree(),
                planar
                    .control_points()
                    .iter()
                    .map(lift)
                    .collect::<Result<Vec<_>, _>>()?,
                planar.weights().to_vec(),
                planar.knots().to_vec(),
            )?,
            _ => return Err(BuildError::EdgeUseSupportMismatch),
        };
        let expected = match edge_use.direction {
            Direction::Forward => expected,
            Direction::Reversed => expected.reversed()?,
        };
        if curve_parameterizations_equal(curve, &expected)? {
            Ok(())
        } else {
            Err(BuildError::EdgeUseSupportMismatch)
        }
    }

    fn validate_rational_tensor_graph_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let SurfaceExactData::RationalBezier {
            control_points: surface_points,
            weights: surface_weights,
        } = surface.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let Curve3ExactData::RationalBezier {
            control_points: curve_points,
            weights: curve_weights,
        } = curve.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        if surface_points
            .iter()
            .zip(&surface_weights)
            .all(|(points, weights)| points.len() == 2 && weights.len() == 2)
        {
            let trimmed = self.trim_rational_tensor_profile(
                &surface_points,
                &surface_weights,
                edge_use,
                pcurve,
                SurfaceIsoAxis::V,
            )?;
            match self.validate_tensor_graph_controls(
                &curve_points,
                &curve_weights,
                &trimmed.points,
                &trimmed.weights,
                edge_use,
                pcurve,
                &trimmed.start,
                &(&trimmed.end - &trimmed.start),
                &Real::one(),
                &Real::zero(),
                SurfaceIsoAxis::V,
            ) {
                Ok(()) => return Ok(()),
                Err(BuildError::EdgeUseSupportMismatch) => {}
                Err(error) => return Err(error),
            }
        }
        let Some((transposed_points, transposed_weights)) =
            transpose_two_row_tensor(&surface_points, &surface_weights)
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let trimmed = self.trim_rational_tensor_profile(
            &transposed_points,
            &transposed_weights,
            edge_use,
            pcurve,
            SurfaceIsoAxis::U,
        )?;
        self.validate_tensor_graph_controls(
            &curve_points,
            &curve_weights,
            &trimmed.points,
            &trimmed.weights,
            edge_use,
            pcurve,
            &trimmed.start,
            &(&trimmed.end - &trimmed.start),
            &Real::one(),
            &Real::zero(),
            SurfaceIsoAxis::U,
        )
    }

    fn validate_nurbs_tensor_graph_image(
        &self,
        curve: &Curve3,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points: surface_points,
            weights: surface_weights,
            u_knots,
            v_knots,
            ..
        } = surface.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let Curve3ExactData::Nurbs {
            degree,
            control_points: curve_points,
            weights: curve_weights,
            knots: curve_knots,
        } = curve.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        if u_degree == 1
            && degree == v_degree
            && !surface_points.is_empty()
            && surface_points
                .iter()
                .zip(&surface_weights)
                .all(|(points, weights)| points.len() == 2 && weights.len() == 2)
        {
            let trimmed = self.trim_nurbs_tensor_profile(
                degree,
                &v_knots,
                &surface_points,
                &surface_weights,
                edge_use,
                pcurve,
                SurfaceIsoAxis::V,
            )?;
            let coefficient_offset = u_knots[u_degree].clone();
            let coefficient_scale = &u_knots[surface_points[0].len()] - &coefficient_offset;
            if real_slices_equal(&curve_knots, &trimmed.knots)? {
                match self.validate_nurbs_tensor_graph_controls(
                    degree,
                    &trimmed.knots,
                    &curve_points,
                    &curve_weights,
                    &trimmed.points,
                    &trimmed.weights,
                    edge_use,
                    pcurve,
                    &coefficient_scale,
                    &coefficient_offset,
                    SurfaceIsoAxis::V,
                ) {
                    Ok(()) => return Ok(()),
                    Err(BuildError::EdgeUseSupportMismatch) => {}
                    Err(error) => return Err(error),
                }
            }
        }
        if v_degree != 1
            || degree != u_degree
            || surface_points.len() != 2
            || surface_points
                .iter()
                .zip(&surface_weights)
                .any(|(points, weights)| points.is_empty() || points.len() != weights.len())
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let Some((transposed_points, transposed_weights)) =
            transpose_two_row_tensor(&surface_points, &surface_weights)
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let trimmed = self.trim_nurbs_tensor_profile(
            degree,
            &u_knots,
            &transposed_points,
            &transposed_weights,
            edge_use,
            pcurve,
            SurfaceIsoAxis::U,
        )?;
        if !real_slices_equal(&curve_knots, &trimmed.knots)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let coefficient_offset = v_knots[v_degree].clone();
        let coefficient_scale = &v_knots[surface_points.len()] - &coefficient_offset;
        self.validate_nurbs_tensor_graph_controls(
            degree,
            &trimmed.knots,
            &curve_points,
            &curve_weights,
            &trimmed.points,
            &trimmed.weights,
            edge_use,
            pcurve,
            &coefficient_scale,
            &coefficient_offset,
            SurfaceIsoAxis::U,
        )
    }

    fn tensor_graph_profile_interval(
        &self,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(Real, Real), BuildError> {
        let oriented = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        let start = oriented.curve().start();
        let end = oriented.curve().end();
        let (start, end) = match profile_axis {
            SurfaceIsoAxis::U => (start.x(), end.x()),
            SurfaceIsoAxis::V => (start.y(), end.y()),
        };
        if decided_model_order(compare_reals(start, end))? != std::cmp::Ordering::Less {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        Ok((start.clone(), end.clone()))
    }

    fn trim_rational_tensor_profile(
        &self,
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<TrimmedRationalTensorProfile, BuildError> {
        let (start, end) = self.tensor_graph_profile_interval(edge_use, pcurve, profile_axis)?;
        let profile = Curve3::rational_bezier(
            surface_points.iter().map(|row| row[0].clone()).collect(),
            surface_weights.iter().map(|row| row[0].clone()).collect(),
        )?
        .subcurve(&start, &end)?;
        let Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } = profile.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let direction = &surface_points[0][1] - &surface_points[0][0];
        Ok(TrimmedRationalTensorProfile {
            points: translation_tensor_rows(&control_points, &direction),
            weights: duplicated_tensor_weights(&weights),
            start,
            end,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn trim_nurbs_tensor_profile(
        &self,
        degree: usize,
        knots: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<TrimmedNurbsTensorProfile, BuildError> {
        let (start, end) = self.tensor_graph_profile_interval(edge_use, pcurve, profile_axis)?;
        let profile = Curve3::nurbs(
            degree,
            surface_points.iter().map(|row| row[0].clone()).collect(),
            surface_weights.iter().map(|row| row[0].clone()).collect(),
            knots.to_vec(),
        )?
        .subcurve(&start, &end)?;
        let Curve3ExactData::Nurbs {
            control_points,
            weights,
            knots,
            ..
        } = profile.exact_data()
        else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let direction = &surface_points[0][1] - &surface_points[0][0];
        Ok(TrimmedNurbsTensorProfile {
            points: translation_tensor_rows(&control_points, &direction),
            weights: duplicated_tensor_weights(&weights),
            knots,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn validate_nurbs_tensor_graph_controls(
        &self,
        degree: usize,
        knots: &[Real],
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        coefficient_scale: &Real,
        coefficient_offset: &Real,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(), BuildError> {
        let coefficients = self.validate_tensor_graph_spatial_controls(
            curve_points,
            curve_weights,
            surface_points,
            surface_weights,
        )?;
        let graph_coefficients = coefficients
            .iter()
            .map(|coefficient| coefficient_offset + coefficient_scale * coefficient)
            .collect::<Vec<_>>();
        let expected = materialize_nurbs_parameter_graph(
            degree,
            &graph_coefficients,
            &surface_weights
                .iter()
                .map(|weights| weights[0].clone())
                .collect::<Vec<_>>(),
            knots,
            profile_axis,
        )?;
        let oriented_pcurve = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        validate_projective_pcurve_equal(oriented_pcurve.curve(), &expected)
    }

    #[allow(clippy::too_many_arguments)]
    fn validate_tensor_graph_controls(
        &self,
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        parameter_start: &Real,
        parameter_span: &Real,
        coefficient_scale: &Real,
        coefficient_offset: &Real,
        profile_axis: SurfaceIsoAxis,
    ) -> Result<(), BuildError> {
        let coefficients = self.validate_tensor_graph_spatial_controls(
            curve_points,
            curve_weights,
            surface_points,
            surface_weights,
        )?;
        let oriented_pcurve = match edge_use.direction {
            Direction::Forward => pcurve.clone(),
            Direction::Reversed => pcurve.reversed()?,
        };
        let CurveGeometry2::RationalBezier(graph) = oriented_pcurve.curve().geometry() else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        let graph_coefficients = coefficients
            .iter()
            .map(|coefficient| coefficient_offset + coefficient_scale * coefficient)
            .collect::<Vec<_>>();
        let (expected_points, expected_weights) = rational_tensor_graph_controls(
            &surface_weights
                .iter()
                .map(|weights| weights[0].clone())
                .collect::<Vec<_>>(),
            &graph_coefficients,
            parameter_start,
            parameter_span,
            profile_axis,
        )?;
        validate_rational_pcurve_controls(graph, &expected_points, &expected_weights)
    }

    fn validate_tensor_graph_spatial_controls(
        &self,
        curve_points: &[Point3],
        curve_weights: &[Real],
        surface_points: &[Vec<Point3>],
        surface_weights: &[Vec<Real>],
    ) -> Result<Vec<Real>, BuildError> {
        if surface_points.is_empty()
            || surface_points.len() != surface_weights.len()
            || surface_points
                .iter()
                .zip(surface_weights)
                .any(|(points, weights)| points.len() != 2 || weights.len() != 2)
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        for weights in surface_weights {
            if !real_values_equal(&weights[0], &weights[1])? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
        }
        let direction = &surface_points[0][1] - &surface_points[0][0];
        let direction_axis = (0..3)
            .find_map(
                |axis| match compare_reals(&direction.0[axis], &Real::zero()) {
                    PredicateOutcome::Decided {
                        value: std::cmp::Ordering::Equal,
                        ..
                    } => None,
                    PredicateOutcome::Decided { .. } => Some(Ok(axis)),
                    PredicateOutcome::Unknown { needed, stage } => Some(Err(BuildError::Geometry(
                        GeometryError::PredicateUnresolved { needed, stage },
                    ))),
                },
            )
            .transpose()?
            .ok_or(BuildError::EdgeUseSupportMismatch)?;
        for row in surface_points {
            if !points_equal(&(row[0].clone() + direction.clone()), &row[1])? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
        }

        if curve_points.len() != surface_points.len()
            || curve_weights.len() != surface_weights.len()
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let weight_scale = (&curve_weights[0] / &surface_weights[0][0])
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let mut coefficients = Vec::with_capacity(curve_points.len());
        for ((curve_point, curve_weight), (surface_row, surface_row_weights)) in curve_points
            .iter()
            .zip(curve_weights)
            .zip(surface_points.iter().zip(surface_weights))
        {
            if !real_values_equal(curve_weight, &(&surface_row_weights[0] * &weight_scale))? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            let coefficient = ((point3_component(curve_point, direction_axis)
                - point3_component(&surface_row[0], direction_axis))
                / &direction.0[direction_axis])
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            if !points_equal(
                &(surface_row[0].clone() + direction.clone() * &coefficient),
                curve_point,
            )? {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            coefficients.push(coefficient);
        }
        Ok(coefficients)
    }

    fn validate_tensor_surface_iso_image(
        &self,
        curve: &Curve3,
        _edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("matched line pcurve carries line geometry");
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        if u_constant == v_constant {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (axis, constant, lower, upper) = if v_constant {
            (
                SurfaceIsoAxis::U,
                line.start().y(),
                line.start().x(),
                line.end().x(),
            )
        } else {
            (
                SurfaceIsoAxis::V,
                line.start().x(),
                line.start().y(),
                line.end().y(),
            )
        };
        let complete = surface.iso_curve(axis, constant)?;
        if !complete.domain().contains(lower)? || !complete.domain().contains(upper)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (source_start, source_end) = match edge_use.direction {
            Direction::Forward => (lower, upper),
            Direction::Reversed => (upper, lower),
        };
        let segment = match decided_model_order(compare_reals(source_start, source_end))? {
            std::cmp::Ordering::Less => complete.subcurve(source_start, source_end)?,
            std::cmp::Ordering::Greater => {
                complete.subcurve(source_end, source_start)?.reversed()?
            }
            std::cmp::Ordering::Equal => return Err(BuildError::EdgeUseSupportMismatch),
        };
        if !tensor_curve_images_equal(curve, &complete)?
            && !tensor_curve_images_equal(curve, &segment)?
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        Ok(())
    }

    fn validate_planar_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        debug_assert!(data.circle);
        let arc = pcurve
            .circular_arc()
            .expect("circular pcurve kind carries circular geometry");
        let sweep = match arc.directed_sweep_angle().map_err(GeometryError::from)? {
            Classification::Decided(sweep) => sweep,
            Classification::Uncertain(reason) => {
                return Err(BuildError::Geometry(
                    GeometryError::PlanarClassificationUnresolved(reason),
                ));
            }
        };
        require_real_equal(
            &sweep,
            &(edge.domain.end() - edge.domain.start()),
            BuildError::EdgeUseSweepMismatch,
        )?;

        let center_parameter = Point2::new(arc.center().x().clone(), arc.center().y().clone());
        let mapped_center = surface.point_at(&center_parameter)?;
        require_point_equal(
            &mapped_center,
            &data.center,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let radial_x = arc.start().x() - arc.center().x();
        let radial_y = arc.start().y() - arc.center().y();
        let (tangent_u, tangent_v) = if arc.is_clockwise() {
            (radial_y.clone(), -radial_x.clone())
        } else {
            (-radial_y.clone(), radial_x.clone())
        };
        let (plane_u, plane_v) = surface
            .plane_directions()
            .expect("plane kind carries plane directions");
        let mapped_tangent = plane_u.clone() * tangent_u + plane_v.clone() * tangent_v;
        let directed_start = match edge_use.direction {
            Direction::Forward => edge.domain.start(),
            Direction::Reversed => edge.domain.end(),
        };
        let mut edge_tangent = curve.derivative_at(directed_start, 1)?.vector().clone();
        if edge_use.direction == Direction::Reversed {
            edge_tangent = -edge_tangent;
        }
        require_vector_equal(
            &mapped_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_cylinder_axial_line_image(&self, pcurve: &Pcurve) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_cylinder_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        let SurfaceExactData::Cylinder { origin, axis, .. } = surface.exact_data() else {
            unreachable!("cylinder kind carries cylinder exact data");
        };
        let expected_center = origin + axis * line.start().y();
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let du = line.end().x() - line.start().x();
        let du_dt = (du / pcurve_span).map_err(|_| GeometryError::ProjectiveDivision)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&surface_parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_sphere_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        if u_constant == v_constant {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = surface.exact_data()
        else {
            unreachable!("sphere kind carries sphere exact data");
        };
        let (expected_center, expected_radius, surface_parameter, varying_u) = if v_constant {
            let latitude = line.start().y().clone();
            (
                center + axis.clone() * (&radius * latitude.clone().sin()),
                &radius * latitude.clone().cos(),
                Point2::new(line.start().x().clone(), latitude),
                true,
            )
        } else {
            (
                center,
                radius,
                Point2::new(line.start().x().clone(), line.start().y().clone()),
                false,
            )
        };
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let parameter_delta = if varying_u {
            line.end().x() - line.start().x()
        } else {
            line.end().y() - line.start().y()
        };
        let parameter_rate =
            (parameter_delta / pcurve_span).map_err(|_| GeometryError::ProjectiveDivision)?;
        let partials = surface.partials_at(&surface_parameter)?;
        let surface_tangent = if varying_u {
            partials.u().clone()
        } else {
            partials.v().clone()
        } * parameter_rate;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_torus_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
        } = surface.exact_data()
        else {
            unreachable!("torus kind carries torus exact data");
        };
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        if u_constant == v_constant {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (expected_center, expected_radius) = if v_constant {
            let v = line.start().y().clone();
            (
                center + axis * (&minor_radius * v.clone().sin()),
                major_radius + &minor_radius * v.cos(),
            )
        } else {
            let u = line.start().x().clone();
            let radial = x * u.clone().cos() + y * u.sin();
            (center + radial * major_radius, minor_radius)
        };
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let du_dt = ((line.end().x() - line.start().x()) / &pcurve_span)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let dv_dt = ((line.end().y() - line.start().y()) / pcurve_span)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let partials = surface.partials_at(&surface_parameter)?;
        let surface_tangent = partials.u().clone() * du_dt + partials.v().clone() * dv_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_cone_generator_image(&self, pcurve: &Pcurve) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_cone_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Cone {
            apex,
            axis,
            semi_angle,
            ..
        } = surface.exact_data()
        else {
            unreachable!("cone kind carries cone exact data");
        };
        let v = line.start().y().clone();
        let expected_center = apex + axis * (&v * semi_angle.clone().cos());
        let expected_radius = v * semi_angle.sin();
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let du_dt = ((line.end().x() - line.start().x()) / &pcurve_span)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&surface_parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_extrusion_line_image(
        &self,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        let u_constant = real_values_equal(line.start().x(), line.end().x())?;
        if u_constant {
            return Ok(());
        }
        let v_constant = real_values_equal(line.start().y(), line.end().y())?;
        let SurfaceExactData::Extrusion { profile, .. } = surface.exact_data() else {
            unreachable!("extrusion kind carries extrusion exact data");
        };
        if v_constant && matches!(*profile, Curve3ExactData::Line(_)) {
            Ok(())
        } else {
            Err(BuildError::EdgeUseSupportMismatch)
        }
    }

    fn validate_extrusion_profile_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().x(), line.end().x())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (profile_start, profile_end) =
            if decided_model_order(compare_reals(line.start().x(), line.end().x()))?
                == std::cmp::Ordering::Less
            {
                (line.start().x(), line.end().x())
            } else {
                (line.end().x(), line.start().x())
            };
        let (profile, direction) = surface
            .extrusion_profile_and_direction()
            .expect("extrusion kind carries extrusion geometry");
        let offset = direction.clone() * line.start().y();
        let expected = translated_curve(&profile.subcurve(profile_start, profile_end)?, &offset)?;
        let actual = curve.subcurve(edge.domain.start(), edge.domain.end())?;
        if !curve_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }

        let edge_domain_span = edge.domain.end() - edge.domain.start();
        let actual_span = actual.domain().end() - actual.domain().start();
        let profile_span = profile_end - profile_start;
        let expected_span = expected.domain().end() - expected.domain().start();
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.domain_start(), line.start().x()),
            (pcurve.domain_end(), line.end().x()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                pcurve_parameter,
            )?;
            let actual_parameter = actual.domain().start()
                + ((&edge_parameter - edge.domain.start()) * &actual_span / &edge_domain_span)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
            let expected_parameter = expected.domain().start()
                + ((surface_parameter - profile_start) * &expected_span / &profile_span)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
            require_real_equal(
                &actual_parameter,
                &expected_parameter,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        Ok(())
    }

    fn validate_extrusion_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        let SurfaceExactData::Extrusion { profile, direction } = surface.exact_data() else {
            unreachable!("extrusion kind carries extrusion exact data");
        };
        let Curve3ExactData::EllipseArc(profile_data) = *profile else {
            return Err(BuildError::EdgeUseSupportMismatch);
        };
        if !profile_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let expected_center = profile_data.center + direction * line.start().y();
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &profile_data.x_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let du = line.end().x() - line.start().x();
        let du_dt = (du / pcurve_span).map_err(|_| GeometryError::ProjectiveDivision)?;
        let surface_parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&surface_parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_revolution_meridian_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().x(),
            line.end().x(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        if real_values_equal(line.start().y(), line.end().y())? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let (profile_start, profile_end) =
            if decided_model_order(compare_reals(line.start().y(), line.end().y()))?
                == std::cmp::Ordering::Less
            {
                (line.start().y(), line.end().y())
            } else {
                (line.end().y(), line.start().y())
            };
        let expected = surface
            .revolution_meridian_curve(line.start().x())?
            .subcurve(profile_start, profile_end)?;
        let actual = curve.subcurve(edge.domain.start(), edge.domain.end())?;
        if !curve_parameterizations_equal(&actual, &expected)? {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let edge_domain_span = edge.domain.end() - edge.domain.start();
        let actual_span = actual.domain().end() - actual.domain().start();
        let profile_span = profile_end - profile_start;
        let expected_span = expected.domain().end() - expected.domain().start();
        for (pcurve_parameter, surface_parameter) in [
            (pcurve.domain_start(), line.start().y()),
            (pcurve.domain_end(), line.end().y()),
        ] {
            let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
                pcurve,
                &edge.domain,
                edge_use.direction,
                pcurve_parameter,
            )?;
            let actual_parameter = actual.domain().start()
                + ((&edge_parameter - edge.domain.start()) * &actual_span / &edge_domain_span)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
            let expected_parameter = expected.domain().start()
                + ((surface_parameter - profile_start) * &expected_span / &profile_span)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
            require_real_equal(
                &actual_parameter,
                &expected_parameter,
                BuildError::EdgeUseSupportMismatch,
            )?;
        }
        Ok(())
    }

    fn validate_revolution_circle_image(
        &self,
        curve: &Curve3,
        edge: &Edge,
        edge_use: &EdgeUse,
        pcurve: &Pcurve,
        surface: &Surface,
    ) -> Result<(), BuildError> {
        let line = pcurve
            .line_segment()
            .expect("line pcurve kind carries line geometry");
        require_real_equal(
            line.start().y(),
            line.end().y(),
            BuildError::EdgeUseSupportMismatch,
        )?;
        let Curve3ExactData::EllipseArc(curve_data) = curve.exact_data() else {
            unreachable!("circle kind carries ellipse-arc exact data");
        };
        if !curve_data.circle {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
        let SurfaceExactData::Revolution {
            profile,
            axis_origin,
            axis,
        } = surface.exact_data()
        else {
            unreachable!("revolution kind carries revolution exact data");
        };
        let profile = Curve3::from_exact_data(*profile)?;
        let profile_point = profile.point_at(line.start().y())?;
        let relative = &profile_point - &axis_origin;
        let axial = axis.clone() * axis.dot(&relative);
        let expected_center = axis_origin + axial;
        let expected_radius = (profile_point - &expected_center)
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        require_point_equal(
            &curve_data.center,
            &expected_center,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.x_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;
        require_real_equal(
            &curve_data.y_radius,
            &expected_radius,
            BuildError::EdgeUseSupportMismatch,
        )?;

        let pcurve_span = pcurve.domain_end() - pcurve.domain_start();
        let du_dt = ((line.end().x() - line.start().x()) / pcurve_span)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parameter = Point2::new(line.start().x().clone(), line.start().y().clone());
        let surface_tangent = surface.partials_at(&parameter)?.u().clone() * du_dt;
        let edge_parameter = edge_use.parameter_correspondence.edge_parameter(
            pcurve,
            &edge.domain,
            edge_use.direction,
            pcurve.domain_start(),
        )?;
        let edge_rate = match &edge_use.parameter_correspondence {
            ParameterCorrespondence::Affine { scale, .. } => scale,
            ParameterCorrespondence::AngularSweep => unreachable!("matched affine relation"),
        };
        let edge_tangent = curve.derivative_at(&edge_parameter, 1)?.vector().clone() * edge_rate;
        require_vector_equal(
            &surface_tangent,
            &edge_tangent,
            BuildError::EdgeUseSupportMismatch,
        )
    }

    fn validate_wire_orientation(
        &self,
        wire: WireId,
        orientation: Orientation,
        outer: bool,
        surface: SurfaceId,
    ) -> Result<(), BuildError> {
        let signed_area_order = match self.signed_wire_double_area(wire) {
            Ok(signed_double_area) => {
                decided_model_order(compare_reals(&signed_double_area, &Real::zero()))?
            }
            Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => {
                let path = self.build_wire_curve_path(wire)?;
                let area = path
                    .bezier_boundary_loop()
                    .map_err(GeometryError::from)?
                    .boundary_loop()
                    .signed_area()
                    .map_err(GeometryError::from)?;
                match area {
                    Some(area) => decided_model_order(compare_reals(&area, &Real::zero()))?,
                    None => self.tensor_graph_wire_orientation(wire, surface)?,
                }
            }
            Err(error) => return Err(error),
        };
        match signed_area_order {
            std::cmp::Ordering::Equal => Err(BuildError::DegenerateWireArea(wire)),
            value => {
                let expected = match (orientation, outer) {
                    (Orientation::Forward, true) | (Orientation::Reversed, false) => {
                        std::cmp::Ordering::Greater
                    }
                    (Orientation::Reversed, true) | (Orientation::Forward, false) => {
                        std::cmp::Ordering::Less
                    }
                };
                if value == expected {
                    Ok(())
                } else {
                    Err(BuildError::InconsistentWireOrientation(wire))
                }
            }
        }
    }

    fn tensor_graph_wire_orientation(
        &self,
        wire: WireId,
        surface: SurfaceId,
    ) -> Result<std::cmp::Ordering, BuildError> {
        let surface = self.surface_ref(surface)?;
        if !matches!(
            surface.kind(),
            SurfaceKind::RationalBezier | SurfaceKind::Nurbs
        ) {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        }
        let SurfaceParameterDomain::Closed(u_domain) = surface.domain().u() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        let SurfaceParameterDomain::Closed(v_domain) = surface.domain().v() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        let wire = self.wire_ref(wire)?;
        let mut graph_endpoints = None;
        let mut has_left_boundary = false;
        let mut has_right_boundary = false;
        let mut has_bottom_boundary = false;
        let mut has_top_boundary = false;
        for edge_use in &wire.edge_uses {
            let pcurve = self.pcurve_ref(self.edge_use_ref(*edge_use)?.pcurve)?;
            match pcurve.curve().geometry() {
                CurveGeometry2::RationalBezier(_) | CurveGeometry2::Nurbs(_) => {
                    if graph_endpoints.is_some() {
                        return Err(GeometryError::UnsupportedPcurveContour.into());
                    }
                    let start = pcurve.curve().start();
                    let end = pcurve.curve().end();
                    graph_endpoints = Some((
                        start.clone(),
                        end.clone(),
                        decided_model_order(compare_reals(end.x(), start.x()))?,
                        decided_model_order(compare_reals(end.y(), start.y()))?,
                    ));
                }
                CurveGeometry2::Line(line)
                    if real_values_equal(line.start().x(), line.end().x())? =>
                {
                    has_left_boundary |= real_values_equal(line.start().x(), u_domain.start())?;
                    has_right_boundary |= real_values_equal(line.start().x(), u_domain.end())?;
                }
                CurveGeometry2::Line(line)
                    if real_values_equal(line.start().y(), line.end().y())? =>
                {
                    has_bottom_boundary |= real_values_equal(line.start().y(), v_domain.start())?;
                    has_top_boundary |= real_values_equal(line.start().y(), v_domain.end())?;
                }
                CurveGeometry2::Line(_) => {
                    return Err(GeometryError::UnsupportedPcurveContour.into());
                }
                _ => return Err(GeometryError::UnsupportedPcurveContour.into()),
            }
        }
        let (graph_start, graph_end, graph_u_direction, graph_v_direction) =
            graph_endpoints.ok_or(GeometryError::UnsupportedPcurveContour)?;
        let mut candidates = Vec::with_capacity(2);
        let graph_on_left = real_values_equal(graph_start.x(), u_domain.start())?
            && real_values_equal(graph_end.x(), u_domain.start())?;
        let graph_on_right = real_values_equal(graph_start.x(), u_domain.end())?
            && real_values_equal(graph_end.x(), u_domain.end())?;
        let effective_left = if graph_on_left != graph_on_right {
            Some(if graph_on_left {
                !has_right_boundary
            } else {
                has_left_boundary
            })
        } else if has_left_boundary != has_right_boundary {
            Some(has_left_boundary)
        } else {
            None
        };
        if graph_v_direction != std::cmp::Ordering::Equal
            && let Some(effective_left) = effective_left
        {
            candidates.push(match (graph_v_direction, effective_left) {
                (std::cmp::Ordering::Greater, true) | (std::cmp::Ordering::Less, false) => {
                    std::cmp::Ordering::Greater
                }
                (std::cmp::Ordering::Less, true) | (std::cmp::Ordering::Greater, false) => {
                    std::cmp::Ordering::Less
                }
                (std::cmp::Ordering::Equal, _) => unreachable!("equal graph span rejected"),
            });
        }
        let graph_on_bottom = real_values_equal(graph_start.y(), v_domain.start())?
            && real_values_equal(graph_end.y(), v_domain.start())?;
        let graph_on_top = real_values_equal(graph_start.y(), v_domain.end())?
            && real_values_equal(graph_end.y(), v_domain.end())?;
        let effective_bottom = if graph_on_bottom != graph_on_top {
            Some(if graph_on_bottom {
                !has_top_boundary
            } else {
                has_bottom_boundary
            })
        } else if has_bottom_boundary != has_top_boundary {
            Some(has_bottom_boundary)
        } else {
            None
        };
        if graph_u_direction != std::cmp::Ordering::Equal
            && let Some(effective_bottom) = effective_bottom
        {
            candidates.push(match (graph_u_direction, effective_bottom) {
                (std::cmp::Ordering::Less, true) | (std::cmp::Ordering::Greater, false) => {
                    std::cmp::Ordering::Greater
                }
                (std::cmp::Ordering::Greater, true) | (std::cmp::Ordering::Less, false) => {
                    std::cmp::Ordering::Less
                }
                (std::cmp::Ordering::Equal, _) => unreachable!("equal graph span rejected"),
            });
        }
        let Some(first) = candidates.first().copied() else {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        };
        if candidates.iter().any(|candidate| *candidate != first) {
            return Err(GeometryError::UnsupportedPcurveContour.into());
        }
        Ok(first)
    }

    fn validate_spherical_trim(
        &self,
        wire: WireId,
        _orientation: Orientation,
    ) -> Result<(), BuildError> {
        let wire_record = self.wire_ref(wire)?;
        let mut first_u = None;
        let mut previous_u = None;
        let mut latitude = None;
        let mut direction = None;
        for edge_use_id in &wire_record.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use_id)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            let Some(line) = pcurve.line_segment() else {
                return Err(BuildError::UnsupportedSphericalTrim(wire));
            };
            if !real_values_equal(line.start().y(), line.end().y())? {
                return Err(BuildError::UnsupportedSphericalTrim(wire));
            }
            if let Some(expected) = &latitude {
                if !real_values_equal(line.start().y(), expected)? {
                    return Err(BuildError::UnsupportedSphericalTrim(wire));
                }
            } else {
                latitude = Some(line.start().y().clone());
            }
            if let Some(previous) = &previous_u {
                if !real_values_equal(line.start().x(), previous)? {
                    return Err(BuildError::InvalidSphericalTrim(wire));
                }
            } else {
                first_u = Some(line.start().x().clone());
            }
            let segment_direction =
                decided_model_order(compare_reals(line.end().x(), line.start().x()))?;
            if !matches!(
                segment_direction,
                std::cmp::Ordering::Less | std::cmp::Ordering::Greater
            ) || direction.is_some_and(|expected| expected != segment_direction)
            {
                return Err(BuildError::InvalidSphericalTrim(wire));
            }
            direction = Some(segment_direction);
            previous_u = Some(line.end().x().clone());
        }
        let latitude = latitude.ok_or(BuildError::UnsupportedSphericalTrim(wire))?;
        let half_pi =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_model_order(compare_reals(&latitude, &-half_pi.clone()))?
            != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&latitude, &half_pi))? != std::cmp::Ordering::Less
        {
            return Err(BuildError::InvalidSphericalTrim(wire));
        }
        let total = previous_u.expect("nonempty wire") - first_u.expect("nonempty wire");
        let expected = match direction.expect("nonempty wire") {
            std::cmp::Ordering::Greater => Real::tau(),
            std::cmp::Ordering::Less => -Real::tau(),
            std::cmp::Ordering::Equal => unreachable!("zero spans rejected"),
        };
        if !real_values_equal(&total, &expected)? {
            return Err(BuildError::InvalidSphericalTrim(wire));
        }
        Ok(())
    }

    fn spherical_trim_coordinates(
        &self,
        wire_id: WireId,
    ) -> Result<(Real, std::cmp::Ordering), BuildError> {
        let wire = self.wire_ref(wire_id)?;
        let first = self.pcurve_ref(self.edge_use_ref(wire.edge_uses[0])?.pcurve)?;
        let last = self.pcurve_ref(
            self.edge_use_ref(*wire.edge_uses.last().expect("validated nonempty wire"))?
                .pcurve,
        )?;
        let first = first
            .line_segment()
            .ok_or(BuildError::UnsupportedSphericalTrim(wire_id))?;
        let last = last
            .line_segment()
            .ok_or(BuildError::UnsupportedSphericalTrim(wire_id))?;
        Ok((
            first.start().y().clone(),
            decided_model_order(compare_reals(last.end().x(), first.start().x()))?,
        ))
    }

    fn validate_wire_nesting(&self, outer: WireId, inner: &[WireId]) -> Result<(), BuildError> {
        if inner.is_empty() {
            return Ok(());
        }
        let policy = CurvePolicy::certified();
        let outer_path = self.build_wire_curve_path(outer)?;
        for wire in inner {
            let path = self.build_wire_curve_path(*wire)?;
            let intersection = outer_path
                .intersect_path(&path, &policy)
                .map_err(GeometryError::from)?;
            if !intersection.is_disjoint() {
                return Err(BuildError::IntersectingFaceWires {
                    first: outer,
                    second: *wire,
                });
            }
            match outer_path
                .classify_point(path.start(), &policy)
                .map_err(GeometryError::from)?
            {
                Classification::Decided(ContourPointLocation::Inside) => {}
                Classification::Decided(_) => return Err(BuildError::InnerWireOutside(*wire)),
                Classification::Uncertain(reason) => {
                    return Err(BuildError::Geometry(
                        GeometryError::PlanarClassificationUnresolved(reason),
                    ));
                }
            }
        }
        for (index, first) in inner.iter().enumerate() {
            let first_path = self.build_wire_curve_path(*first)?;
            for second in &inner[(index + 1)..] {
                let second_path = self.build_wire_curve_path(*second)?;
                let intersection = first_path
                    .intersect_path(&second_path, &policy)
                    .map_err(GeometryError::from)?;
                if !intersection.is_disjoint() {
                    return Err(BuildError::IntersectingFaceWires {
                        first: *first,
                        second: *second,
                    });
                }
                let nested = classification_is_inside(
                    first_path
                        .classify_point(second_path.start(), &policy)
                        .map_err(GeometryError::from)?,
                )? || classification_is_inside(
                    second_path
                        .classify_point(first_path.start(), &policy)
                        .map_err(GeometryError::from)?,
                )?;
                if nested {
                    return Err(BuildError::NestedInnerWires {
                        first: *first,
                        second: *second,
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_wire_simplicity(&self, wire: WireId) -> Result<(), BuildError> {
        match self.build_wire_contour(wire) {
            Ok(contour) => {
                if contour
                    .intersect_self(&CurvePolicy::certified())
                    .map_err(GeometryError::from)?
                    .is_empty()
                {
                    Ok(())
                } else {
                    Err(BuildError::SelfIntersectingWire(wire))
                }
            }
            Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => {
                self.validate_curve_path_simplicity(wire)
            }
            Err(error) => Err(error),
        }
    }

    fn validate_curve_path_simplicity(&self, wire: WireId) -> Result<(), BuildError> {
        let path = self.build_wire_curve_path(wire)?;
        let curves = path.curves();
        let policy = CurvePolicy::certified();
        if curves.len() == 2 {
            let relation = curves[0]
                .intersect_curve(&curves[1], &policy)
                .map_err(GeometryError::from)?;
            if !relation.is_complete()
                || !relation.overlaps().is_empty()
                || relation.contacts().len() != 2
            {
                return Err(BuildError::SelfIntersectingWire(wire));
            }
            let expected = [
                (
                    curves[0].parameter_domain().start(),
                    curves[1].parameter_domain().end(),
                ),
                (
                    curves[0].parameter_domain().end(),
                    curves[1].parameter_domain().start(),
                ),
            ];
            let mut matched = [false; 2];
            for contact in relation.contacts() {
                let Some(first_parameter) = contact.first().exact_curve_parameter() else {
                    return Err(GeometryError::UnsupportedIntersection.into());
                };
                let Some(second_parameter) = contact.second().exact_curve_parameter() else {
                    return Err(GeometryError::UnsupportedIntersection.into());
                };
                let mut found = None;
                for (index, (expected_first, expected_second)) in expected.iter().enumerate() {
                    if !matched[index]
                        && real_values_equal(&first_parameter, expected_first)?
                        && real_values_equal(&second_parameter, expected_second)?
                    {
                        found = Some(index);
                        break;
                    }
                }
                let Some(index) = found else {
                    return Err(BuildError::SelfIntersectingWire(wire));
                };
                matched[index] = true;
            }
            return if matched.into_iter().all(|value| value) {
                Ok(())
            } else {
                Err(BuildError::SelfIntersectingWire(wire))
            };
        }
        for first_index in 0..curves.len() {
            for second_index in (first_index + 1)..curves.len() {
                let relation = curves[first_index]
                    .intersect_curve(&curves[second_index], &policy)
                    .map_err(GeometryError::from)?;
                if !relation.is_complete() {
                    return Err(GeometryError::UnsupportedIntersection.into());
                }
                let adjacent = second_index == first_index + 1
                    || (first_index == 0 && second_index + 1 == curves.len());
                if !relation.overlaps().is_empty() {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
                if !adjacent {
                    if !relation.contacts().is_empty() {
                        return Err(BuildError::SelfIntersectingWire(wire));
                    }
                    continue;
                }
                let (expected_first, expected_second) = if second_index == first_index + 1 {
                    (
                        curves[first_index].parameter_domain().end(),
                        curves[second_index].parameter_domain().start(),
                    )
                } else {
                    (
                        curves[first_index].parameter_domain().start(),
                        curves[second_index].parameter_domain().end(),
                    )
                };
                if relation.contacts().len() != 1 {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
                let contact = &relation.contacts()[0];
                let Some(first_parameter) = contact.first().exact_curve_parameter() else {
                    return Err(GeometryError::UnsupportedIntersection.into());
                };
                let Some(second_parameter) = contact.second().exact_curve_parameter() else {
                    return Err(GeometryError::UnsupportedIntersection.into());
                };
                if !real_values_equal(&first_parameter, expected_first)?
                    || !real_values_equal(&second_parameter, expected_second)?
                {
                    return Err(BuildError::SelfIntersectingWire(wire));
                }
            }
        }
        Ok(())
    }

    fn build_wire_curve_path(&self, wire: WireId) -> Result<CurvePath2, BuildError> {
        let wire = self.wire_ref(wire)?;
        Ok(CurvePath2::try_new(
            wire.edge_uses
                .iter()
                .map(|edge_use| {
                    let edge_use = self.edge_use_ref(*edge_use)?;
                    Ok(self.pcurve_ref(edge_use.pcurve)?.curve().clone())
                })
                .collect::<Result<Vec<_>, BuildError>>()?,
        )
        .map_err(GeometryError::from)?)
    }

    fn build_wire_contour(&self, wire: WireId) -> Result<Contour2, BuildError> {
        let wire = self.wire_ref(wire)?;
        let mut segments = Vec::with_capacity(wire.edge_uses.len());
        for edge_use in &wire.edge_uses {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let pcurve = self.pcurve_ref(edge_use.pcurve)?;
            segments.push(pcurve.segment()?);
        }
        Ok(Contour2::try_new(segments).map_err(GeometryError::from)?)
    }

    fn signed_wire_double_area(&self, wire: WireId) -> Result<Real, BuildError> {
        self.build_wire_contour(wire)?
            .signed_area()
            .map_err(GeometryError::from)?
            .map(|area| Real::from(2) * area)
            .ok_or(BuildError::DegenerateWireArea(wire))
    }

    fn faces_connected(&self, faces: &[FaceId]) -> Result<bool, BuildError> {
        if faces.len() == 1 {
            return Ok(true);
        }
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        let mut edge_faces: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use_id)?.edge;
                    edge_faces.entry(edge).or_default().push(*face_id);
                }
            }
        }
        let mut visited = HashSet::new();
        let mut queue = VecDeque::from([faces[0]]);
        while let Some(face) = queue.pop_front() {
            if !visited.insert(face) {
                continue;
            }
            let record = self.face_ref(face)?;
            for wire_id in record.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use_id)?.edge;
                    for neighbor in edge_faces.get(&edge).into_iter().flatten() {
                        if face_set.contains(neighbor) && !visited.contains(neighbor) {
                            queue.push_back(*neighbor);
                        }
                    }
                }
            }
        }
        Ok(visited.len() == faces.len())
    }

    fn validate_closed_shell(&self, shell: ShellId) -> Result<(), BuildError> {
        let mut uses: HashMap<EdgeId, Vec<Direction>> = HashMap::new();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge_use = self.edge_use_ref(*edge_use_id)?;
                    uses.entry(edge_use.edge)
                        .or_default()
                        .push(edge_use.direction);
                }
            }
        }
        for (edge, directions) in uses {
            if directions.len() != 2 {
                return Err(BuildError::NonManifoldSolidEdge {
                    edge,
                    uses: directions.len(),
                });
            }
            if directions[0] == directions[1] {
                return Err(BuildError::InconsistentSolidEdgeOrientation(edge));
            }
        }
        Ok(())
    }

    fn validate_outer_shell_orientation(&self, shell: ShellId) -> Result<(), BuildError> {
        let signed_six_volume = self.signed_shell_six_volume(shell)?;
        match compare_reals(&signed_six_volume, &Real::zero()) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Greater,
                ..
            } => Ok(()),
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            } => Err(BuildError::DegenerateShellVolume(shell)),
            PredicateOutcome::Decided { .. } => Err(BuildError::InwardSolidShell(shell)),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                }))
            }
        }
    }

    fn validate_void_shell_orientation(&self, shell: ShellId) -> Result<(), BuildError> {
        let signed_six_volume = self.signed_shell_six_volume(shell)?;
        match compare_reals(&signed_six_volume, &Real::zero()) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Less,
                ..
            } => Ok(()),
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            } => Err(BuildError::DegenerateShellVolume(shell)),
            PredicateOutcome::Decided { .. } => Err(BuildError::OutwardVoidShell(shell)),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                }))
            }
        }
    }

    fn validate_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        if voids.is_empty() {
            return Ok(());
        }
        if let Some((outer_center, outer_radius)) =
            self.certified_oriented_sphere_shell(outer, Orientation::Forward)?
        {
            let mut sphere_voids = Vec::with_capacity(voids.len());
            for void_shell in voids {
                let Some((center, radius)) =
                    self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
                else {
                    sphere_voids.clear();
                    break;
                };
                let clearance = &outer_radius - &radius;
                if decided_model_order(compare_reals(&clearance, &Real::zero()))?
                    != std::cmp::Ordering::Greater
                    || decided_model_order(compare_reals(
                        &(&center - &outer_center).norm_squared(),
                        &(&clearance * &clearance),
                    ))? != std::cmp::Ordering::Less
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
                sphere_voids.push((*void_shell, center, radius));
            }
            if sphere_voids.len() == voids.len() {
                for first_index in 0..sphere_voids.len() {
                    for second_index in (first_index + 1)..sphere_voids.len() {
                        let (first_shell, first_center, first_radius) = &sphere_voids[first_index];
                        let (second_shell, second_center, second_radius) =
                            &sphere_voids[second_index];
                        let radius_sum = first_radius + second_radius;
                        if decided_model_order(compare_reals(
                            &(first_center - second_center).norm_squared(),
                            &(&radius_sum * &radius_sum),
                        ))? != std::cmp::Ordering::Greater
                        {
                            return Err(BuildError::IntersectingVoidShells {
                                first: *first_shell,
                                second: *second_shell,
                            });
                        }
                    }
                }
                return Ok(());
            }
            let [void_shell] = voids else {
                return Err(BuildError::UnsupportedSolidShell(voids[0]));
            };
            let Some(cylinder) =
                self.certified_oriented_cylinder_shell(*void_shell, Orientation::Reversed)?
            else {
                return Err(BuildError::UnsupportedSolidShell(*void_shell));
            };
            if !self.sphere_strictly_contains_cylinder(&outer_center, &outer_radius, &cylinder)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            return Ok(());
        }
        if let Some(cylinder) = self.certified_cylinder_shell(outer)? {
            let [void_shell] = voids else {
                return Err(BuildError::UnsupportedSolidShell(voids[0]));
            };
            let Some((center, radius)) =
                self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
            else {
                return Err(BuildError::UnsupportedSolidShell(*void_shell));
            };
            if !self.cylinder_strictly_contains_sphere(&cylinder, &center, &radius)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            return Ok(());
        }
        if let Some(outer_revolution) =
            self.certified_oriented_revolution_shell(outer, Orientation::Forward)?
        {
            let policy = CurvePolicy::certified();
            let mut revolution_voids = Vec::with_capacity(voids.len());
            for void_shell in voids {
                let Some(void) =
                    self.certified_oriented_revolution_shell(*void_shell, Orientation::Reversed)?
                else {
                    return Err(BuildError::UnsupportedSolidShell(*void_shell));
                };
                if !points_equal(&outer_revolution.axis_origin, &void.axis_origin)?
                    || !vectors_equal(&outer_revolution.axis, &void.axis)?
                    || outer_revolution
                        .profile
                        .intersects(&void.profile, &policy)?
                    || !classification_is_inside(
                        outer_revolution
                            .profile
                            .classify_point(void.profile.start(), &policy)?,
                    )?
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
                revolution_voids.push((*void_shell, void.profile));
            }
            for first_index in 0..revolution_voids.len() {
                for second_index in (first_index + 1)..revolution_voids.len() {
                    let (first_shell, first) = &revolution_voids[first_index];
                    let (second_shell, second) = &revolution_voids[second_index];
                    let boundaries_intersect = first.intersects(second, &policy)?;
                    let nested =
                        classification_is_inside(first.classify_point(second.start(), &policy)?)?
                            || classification_is_inside(
                                second.classify_point(first.start(), &policy)?,
                            )?;
                    if boundaries_intersect || nested {
                        return Err(BuildError::IntersectingVoidShells {
                            first: *first_shell,
                            second: *second_shell,
                        });
                    }
                }
            }
            return Ok(());
        }
        let all_straight_planar = self.is_straight_planar_shell(outer)?
            && voids
                .iter()
                .map(|shell| self.is_straight_planar_shell(*shell))
                .collect::<Result<Vec<_>, _>>()?
                .into_iter()
                .all(|planar| planar);
        if all_straight_planar {
            return self.validate_planar_void_shell_nesting(outer, voids);
        }
        let outer_prism = self
            .certified_z_prism_shell(outer)?
            .ok_or(BuildError::UnsupportedSolidShell(outer))?;
        let mut void_prisms = Vec::with_capacity(voids.len());
        for void_shell in voids {
            let prism = self
                .certified_z_prism_shell(*void_shell)?
                .ok_or(BuildError::UnsupportedSolidShell(*void_shell))?;
            if decided_model_order(compare_reals(&outer_prism.z_min, &prism.z_min))?
                != std::cmp::Ordering::Less
                || decided_model_order(compare_reals(&prism.z_max, &outer_prism.z_max))?
                    != std::cmp::Ordering::Less
                || !outer_prism
                    .contour
                    .intersect_contour(&prism.contour, &CurvePolicy::certified())
                    .map_err(GeometryError::from)?
                    .is_empty()
                || !classification_is_inside(outer_prism.contour.classify_point(
                    prism.contour.segments()[0].start(),
                    &CurvePolicy::certified(),
                ))?
            {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            void_prisms.push((*void_shell, prism));
        }

        for first_index in 0..void_prisms.len() {
            for second_index in (first_index + 1)..void_prisms.len() {
                let (first_shell, first) = &void_prisms[first_index];
                let (second_shell, second) = &void_prisms[second_index];
                let separated_in_z =
                    decided_model_order(compare_reals(&first.z_max, &second.z_min))?
                        == std::cmp::Ordering::Less
                        || decided_model_order(compare_reals(&second.z_max, &first.z_min))?
                            == std::cmp::Ordering::Less;
                if separated_in_z {
                    continue;
                }
                let policy = CurvePolicy::certified();
                let boundaries_intersect = !first
                    .contour
                    .intersect_contour(&second.contour, &policy)
                    .map_err(GeometryError::from)?
                    .is_empty();
                let nested = classification_is_inside(
                    first
                        .contour
                        .classify_point(second.contour.segments()[0].start(), &policy),
                )? || classification_is_inside(
                    second
                        .contour
                        .classify_point(first.contour.segments()[0].start(), &policy),
                )?;
                if boundaries_intersect || nested {
                    return Err(BuildError::IntersectingVoidShells {
                        first: *first_shell,
                        second: *second_shell,
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_planar_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        for void_shell in voids {
            if self.planar_shells_contact(outer, *void_shell)? {
                return Err(BuildError::VoidShellOutside(*void_shell));
            }
            let vertices = self.shell_vertex_set(*void_shell)?;
            for vertex in vertices {
                if self
                    .classify_point_against_planar_shell(outer, self.vertex_ref(vertex)?.point())?
                    != SolidPointLocation::Inside
                {
                    return Err(BuildError::VoidShellOutside(*void_shell));
                }
            }
        }
        for first_index in 0..voids.len() {
            for second_index in (first_index + 1)..voids.len() {
                let first = voids[first_index];
                let second = voids[second_index];
                if self.planar_shells_contact(first, second)?
                    || self.planar_shell_contains_vertex(first, second)?
                    || self.planar_shell_contains_vertex(second, first)?
                {
                    return Err(BuildError::IntersectingVoidShells { first, second });
                }
            }
        }
        Ok(())
    }

    fn planar_shell_contains_vertex(
        &self,
        container: ShellId,
        candidate: ShellId,
    ) -> Result<bool, BuildError> {
        let vertex = self
            .shell_vertex_set(candidate)?
            .into_iter()
            .min()
            .ok_or(BuildError::EmptyShell)?;
        Ok(
            self.classify_point_against_planar_shell(container, self.vertex_ref(vertex)?.point())?
                != SolidPointLocation::Outside,
        )
    }

    fn certify_simple_prism_shell(&self, shell: ShellId) -> Result<bool, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        return Ok(false);
                    }
                }
            }
        }
        self.certify_translation_shell_topology(shell)
    }

    fn certify_internally_partitioned_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !matches!(
                self.surface_ref(face.surface)?.kind(),
                SurfaceKind::Plane | SurfaceKind::Extrusion
            ) {
                return Ok(false);
            }
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    let edge = self.edge_use_ref(*edge_use)?.edge;
                    if matches!(
                        self.curve_ref(self.edge_ref(edge)?.curve)?.kind(),
                        Curve3Kind::Line | Curve3Kind::CircleArc
                    ) {
                        continue;
                    }
                    let incident = &self.edge_uses_by_edge[edge.index()];
                    if incident.len() != 2 {
                        return Ok(false);
                    }
                    let mut support = None;
                    for incident_use in incident {
                        let Some(wire) = self.edge_use_wire[incident_use.index()] else {
                            return Ok(false);
                        };
                        let Some(incident_face) = self.wire_face[wire.index()] else {
                            return Ok(false);
                        };
                        if !face_set.contains(&incident_face) {
                            return Ok(false);
                        }
                        let incident_face = self.face_ref(incident_face)?;
                        if self.surface_ref(incident_face.surface)?.kind() != SurfaceKind::Plane {
                            return Ok(false);
                        }
                        if let Some(expected) = support {
                            if !matches!(
                                self.surface_ref(expected)?
                                    .intersect_surface(self.surface_ref(incident_face.surface)?)?,
                                crate::SurfaceSurfaceIntersection::Coincident
                            ) {
                                return Ok(false);
                            }
                        } else {
                            support = Some(incident_face.surface);
                        }
                    }
                }
            }
        }
        self.certify_translation_shell_topology(shell)
    }

    fn certify_convex_planar_shell(&self, shell: ShellId) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut vertices = HashSet::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(false);
            }
            let surface = self.surface_ref(face.surface)?;
            if surface.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
            let Some(outer) = face.outer() else {
                return Ok(false);
            };
            let contour = match self.build_wire_contour(outer) {
                Ok(contour) => contour,
                Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => {
                    return Ok(false);
                }
                Err(error) => return Err(error),
            };
            let mut turn = None;
            for index in 0..contour.segments().len() {
                let first = contour.segments()[index].start();
                let second = contour.segments()[(index + 1) % contour.segments().len()].start();
                let third = contour.segments()[(index + 2) % contour.segments().len()].start();
                if !matches!(contour.segments()[index], Segment2::Line(_)) {
                    return Ok(false);
                }
                let cross = (second.x() - first.x()) * (third.y() - second.y())
                    - (second.y() - first.y()) * (third.x() - second.x());
                let order = decided_model_order(compare_reals(&cross, &Real::zero()))?;
                if order == std::cmp::Ordering::Equal {
                    continue;
                }
                if turn.is_some_and(|expected| expected != order) {
                    return Ok(false);
                }
                turn = Some(order);
            }
            if turn.is_none() {
                return Ok(false);
            }
            vertices.extend(self.face_boundary_vertex_set(face)?);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            let origin = surface
                .plane_origin()
                .expect("convex planar certificate prevalidates planes");
            let (u, v) = surface
                .plane_directions()
                .expect("convex planar certificate prevalidates planes");
            let mut normal = u.cross(v);
            if face.orientation == Orientation::Reversed {
                normal = -normal;
            }
            for vertex in &vertices {
                let value = normal.dot(&(self.vertex_ref(*vertex)?.point() - origin));
                if decided_model_order(compare_reals(&value, &Real::zero()))?
                    == std::cmp::Ordering::Greater
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    fn certify_planar_shell_non_self_intersection(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        if !self.is_straight_planar_shell(shell)? {
            return Ok(false);
        }
        let faces = &self.shell_ref(shell)?.faces;

        for first_index in 0..faces.len() {
            for second_index in (first_index + 1)..faces.len() {
                let first_id = faces[first_index];
                let second_id = faces[second_index];
                let first = self.face_ref(first_id)?;
                let second = self.face_ref(second_id)?;
                let first_surface = self.surface_ref(first.surface)?;
                let second_surface = self.surface_ref(second.surface)?;
                match first_surface.intersect_surface(second_surface)? {
                    crate::SurfaceSurfaceIntersection::None => {}
                    crate::SurfaceSurfaceIntersection::Coincident => {
                        let overlaps = self.coincident_planar_faces_conflict(
                            first_id,
                            second_id,
                            first_surface,
                            true,
                        )?;
                        if overlaps {
                            return Err(BuildError::SelfIntersectingSolidShell(shell));
                        }
                    }
                    crate::SurfaceSurfaceIntersection::Line(line) => {
                        let overlaps = self.transverse_planar_face_interiors_overlap(
                            first_id,
                            second_id,
                            &line.point,
                            &line.direction,
                            true,
                        )?;
                        if overlaps {
                            return Err(BuildError::SelfIntersectingSolidShell(shell));
                        }
                    }
                    _ => return Ok(false),
                }
            }
        }
        Ok(true)
    }

    fn is_straight_planar_shell(&self, shell: ShellId) -> Result<bool, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    }

    fn planar_shells_contact(
        &self,
        first_shell: ShellId,
        second_shell: ShellId,
    ) -> Result<bool, BuildError> {
        for first_id in &self.shell_ref(first_shell)?.faces {
            for second_id in &self.shell_ref(second_shell)?.faces {
                let first = self.face_ref(*first_id)?;
                let second = self.face_ref(*second_id)?;
                let first_surface = self.surface_ref(first.surface)?;
                let second_surface = self.surface_ref(second.surface)?;
                match first_surface.intersect_surface(second_surface)? {
                    crate::SurfaceSurfaceIntersection::None => {}
                    crate::SurfaceSurfaceIntersection::Coincident => {
                        if self.coincident_planar_faces_conflict(
                            *first_id,
                            *second_id,
                            first_surface,
                            false,
                        )? {
                            return Ok(true);
                        }
                    }
                    crate::SurfaceSurfaceIntersection::Line(line) => {
                        if self.transverse_planar_face_interiors_overlap(
                            *first_id,
                            *second_id,
                            &line.point,
                            &line.direction,
                            false,
                        )? {
                            return Ok(true);
                        }
                    }
                    _ => return Ok(true),
                }
            }
        }
        Ok(false)
    }

    fn coincident_planar_faces_conflict(
        &self,
        first: FaceId,
        second: FaceId,
        common_surface: &Surface,
        allow_shared_topology: bool,
    ) -> Result<bool, BuildError> {
        let first_region = self.face_region_in_plane_frame(first, common_surface)?;
        let second_region = self.face_region_in_plane_frame(second, common_surface)?;
        match first_region
            .boolean_region(
                &second_region,
                BooleanOp::Intersection,
                FillRule::NonZero,
                &CurvePolicy::certified(),
            )
            .map_err(GeometryError::from)?
        {
            Classification::Decided(region) if !region.is_empty() => return Ok(true),
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        }

        let first_edges = self.face_edge_set(first)?;
        let second_edges = self.face_edge_set(second)?;
        for first_edge_id in &first_edges {
            let first_edge = self.edge_ref(*first_edge_id)?;
            let first_start = self.vertex_ref(first_edge.start)?.point();
            let first_end = self.vertex_ref(first_edge.end)?.point();
            let first_segment = LineSeg2::try_new(
                project_point_to_surface_plane(first_start, common_surface)?,
                project_point_to_surface_plane(first_end, common_surface)?,
            )
            .map_err(GeometryError::from)?;
            for second_edge_id in &second_edges {
                let second_edge = self.edge_ref(*second_edge_id)?;
                let second_segment = LineSeg2::try_new(
                    project_point_to_surface_plane(
                        self.vertex_ref(second_edge.start)?.point(),
                        common_surface,
                    )?,
                    project_point_to_surface_plane(
                        self.vertex_ref(second_edge.end)?.point(),
                        common_surface,
                    )?,
                )
                .map_err(GeometryError::from)?;
                match first_segment
                    .intersect_line(&second_segment, &CurvePolicy::certified())
                    .map_err(GeometryError::from)?
                {
                    LineLineIntersection::None => {}
                    LineLineIntersection::Point { a_param, .. } => {
                        let contact = first_start.clone() + (first_end - first_start) * a_param;
                        if !allow_shared_topology
                            || !self.faces_share_vertex_at(first, second, &contact)?
                        {
                            return Ok(true);
                        }
                    }
                    LineLineIntersection::Overlap { .. } => {
                        if !allow_shared_topology || first_edge_id != second_edge_id {
                            return Ok(true);
                        }
                    }
                    LineLineIntersection::Uncertain { reason } => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }
        Ok(false)
    }

    fn transverse_planar_face_interiors_overlap(
        &self,
        first: FaceId,
        second: FaceId,
        point: &Point3,
        direction: &Vector3,
        allow_shared_topology: bool,
    ) -> Result<bool, BuildError> {
        let first_material = self.face_line_material(first, point, direction)?;
        let second_material = self.face_line_material(second, point, direction)?;
        let first_edges = self.face_edge_set(first)?;
        let second_edges = self.face_edge_set(second)?;
        let shared = first_edges
            .intersection(&second_edges)
            .copied()
            .collect::<Vec<_>>();
        for first in &first_material.intervals {
            for second in &second_material.intervals {
                let lower = maximum_real(&first.0, &second.0)?;
                let upper = minimum_real(&first.1, &second.1)?;
                if decided_model_order(compare_reals(&lower, &upper))? != std::cmp::Ordering::Less {
                    continue;
                }
                if !allow_shared_topology {
                    return Ok(true);
                }
                let mut covered = false;
                for edge_id in &shared {
                    let edge = self.edge_ref(*edge_id)?;
                    let start =
                        line_parameter(self.vertex_ref(edge.start)?.point(), point, direction)?;
                    let end = line_parameter(self.vertex_ref(edge.end)?.point(), point, direction)?;
                    let edge_lower = minimum_real(&start, &end)?;
                    let edge_upper = maximum_real(&start, &end)?;
                    if decided_model_order(compare_reals(&edge_lower, &lower))?
                        != std::cmp::Ordering::Greater
                        && decided_model_order(compare_reals(&edge_upper, &upper))?
                            != std::cmp::Ordering::Less
                    {
                        covered = true;
                        break;
                    }
                }
                if !covered {
                    return Ok(true);
                }
            }
        }
        for contact in &first_material.contacts {
            if parameter_in_line_material(
                contact,
                &second_material.intervals,
                &second_material.contacts,
            )? && (!allow_shared_topology
                || !self.faces_share_vertex_at(
                    first,
                    second,
                    &(point.clone() + direction.clone() * contact),
                )?)
            {
                return Ok(true);
            }
        }
        for contact in &second_material.contacts {
            if parameter_in_line_material(
                contact,
                &first_material.intervals,
                &first_material.contacts,
            )? && (!allow_shared_topology
                || !self.faces_share_vertex_at(
                    first,
                    second,
                    &(point.clone() + direction.clone() * contact),
                )?)
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn face_line_material(
        &self,
        face_id: FaceId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<FaceLineMaterial, BuildError> {
        let face = self.face_ref(face_id)?;
        let vertices = self.face_boundary_vertex_set(face)?;
        let parameters = vertices
            .iter()
            .map(|vertex| line_parameter(self.vertex_ref(*vertex)?.point(), point, direction))
            .collect::<Result<Vec<_>, _>>()?;
        if parameters.is_empty() {
            return Ok(FaceLineMaterial {
                intervals: Vec::new(),
                contacts: Vec::new(),
            });
        }
        let (lower, upper) = exact_real_min_max(&parameters)?;
        if decided_model_order(compare_reals(&lower, &upper))? != std::cmp::Ordering::Less {
            return Ok(FaceLineMaterial {
                intervals: Vec::new(),
                contacts: Vec::new(),
            });
        }
        let surface = self.surface_ref(face.surface)?;
        let line_start = point.clone() + direction.clone() * &lower;
        let line_end = point.clone() + direction.clone() * &upper;
        let source = LineSeg2::try_new(
            project_point_to_surface_plane(&line_start, surface)?,
            project_point_to_surface_plane(&line_end, surface)?,
        )
        .map_err(GeometryError::from)?;
        let region = self.face_region_in_plane_frame(face_id, surface)?;
        let mut cuts = vec![lower.clone(), upper.clone()];
        for wire in face.boundary_wires() {
            for segment in self.build_wire_contour(*wire)?.segments() {
                let Segment2::Line(boundary) = segment else {
                    return Ok(FaceLineMaterial {
                        intervals: Vec::new(),
                        contacts: Vec::new(),
                    });
                };
                match source
                    .intersect_line(boundary, &CurvePolicy::certified())
                    .map_err(GeometryError::from)?
                {
                    LineLineIntersection::None => {}
                    LineLineIntersection::Point { a_param, .. } => {
                        insert_sorted_real(&mut cuts, &(&lower + (&upper - &lower) * a_param))?;
                    }
                    LineLineIntersection::Overlap { a_range, .. } => {
                        for parameter in [a_range.start(), a_range.end()] {
                            insert_sorted_real(
                                &mut cuts,
                                &(&lower + (&upper - &lower) * parameter),
                            )?;
                        }
                    }
                    LineLineIntersection::Uncertain { reason } => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }
        let mut contacts = Vec::new();
        for cut in &cuts {
            let contact = point.clone() + direction.clone() * cut;
            let contact = project_point_to_surface_plane(&contact, surface)?;
            match region.classify_point(&contact, &CurvePolicy::certified()) {
                Classification::Decided(RegionPointLocation::Boundary) => {
                    insert_sorted_real(&mut contacts, cut)?;
                }
                Classification::Decided(
                    RegionPointLocation::Inside | RegionPointLocation::Outside,
                ) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        let mut intervals = Vec::new();
        for interval in cuts.windows(2) {
            if decided_model_order(compare_reals(&interval[0], &interval[1]))?
                != std::cmp::Ordering::Less
            {
                continue;
            }
            let midpoint = ((&interval[0] + &interval[1]) / Real::from(2))
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let midpoint = point.clone() + direction.clone() * midpoint;
            let midpoint = project_point_to_surface_plane(&midpoint, surface)?;
            match region.classify_point(&midpoint, &CurvePolicy::certified()) {
                Classification::Decided(
                    RegionPointLocation::Inside | RegionPointLocation::Boundary,
                ) => intervals.push((interval[0].clone(), interval[1].clone())),
                Classification::Decided(RegionPointLocation::Outside) => {}
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(FaceLineMaterial {
            intervals,
            contacts,
        })
    }

    fn faces_share_vertex_at(
        &self,
        first: FaceId,
        second: FaceId,
        point: &Point3,
    ) -> Result<bool, BuildError> {
        let first = self.face_boundary_vertex_set(self.face_ref(first)?)?;
        let second = self.face_boundary_vertex_set(self.face_ref(second)?)?;
        for vertex in first.intersection(&second) {
            if points_equal(self.vertex_ref(*vertex)?.point(), point)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub(crate) fn classify_point_against_planar_shell(
        &self,
        shell: ShellId,
        point: &Point3,
    ) -> Result<SolidPointLocation, BuildError> {
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            if decided_model_order(compare_reals(
                &planar_surface_value(surface, point),
                &Real::zero(),
            ))? == std::cmp::Ordering::Equal
            {
                match self
                    .face_region_in_plane_frame(*face_id, surface)?
                    .classify_point(
                        &project_point_to_surface_plane(point, surface)?,
                        &CurvePolicy::certified(),
                    ) {
                    Classification::Decided(
                        RegionPointLocation::Inside | RegionPointLocation::Boundary,
                    ) => return Ok(SolidPointLocation::Boundary),
                    Classification::Decided(RegionPointLocation::Outside) => {}
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                    }
                }
            }
        }

        for direction in [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::from_xyz(Real::one(), Real::from(2), Real::from(4)),
        ] {
            if let Some(location) =
                self.classify_point_against_planar_shell_with_ray(shell, point, &direction)?
            {
                return Ok(location);
            }
        }
        Err(
            GeometryError::PlanarClassificationUnresolved(hypercurve::UncertaintyReason::Boundary)
                .into(),
        )
    }

    fn classify_point_against_planar_shell_with_ray(
        &self,
        shell: ShellId,
        point: &Point3,
        direction: &Vector3,
    ) -> Result<Option<SolidPointLocation>, BuildError> {
        let mut crossings = 0_usize;
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            let surface = self.surface_ref(face.surface)?;
            let (u, v) = surface
                .plane_directions()
                .expect("planar shell classifier prevalidates planes");
            let denominator = u.cross(v).dot(direction);
            if decided_model_order(compare_reals(&denominator, &Real::zero()))?
                == std::cmp::Ordering::Equal
            {
                continue;
            }
            let parameter = ((-planar_surface_value(surface, point)) / denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            if decided_model_order(compare_reals(&parameter, &Real::zero()))?
                != std::cmp::Ordering::Greater
            {
                continue;
            }
            let intersection = point.clone() + direction.clone() * parameter;
            match self
                .face_region_in_plane_frame(*face_id, surface)?
                .classify_point(
                    &project_point_to_surface_plane(&intersection, surface)?,
                    &CurvePolicy::certified(),
                ) {
                Classification::Decided(RegionPointLocation::Inside) => crossings += 1,
                Classification::Decided(RegionPointLocation::Outside) => {}
                Classification::Decided(RegionPointLocation::Boundary)
                | Classification::Uncertain(hypercurve::UncertaintyReason::Boundary) => {
                    return Ok(None);
                }
                Classification::Uncertain(reason) => {
                    return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
                }
            }
        }
        Ok(Some(if crossings.is_multiple_of(2) {
            SolidPointLocation::Outside
        } else {
            SolidPointLocation::Inside
        }))
    }

    fn face_region_in_plane_frame(
        &self,
        face_id: FaceId,
        frame: &Surface,
    ) -> Result<LineArcRegion2, BuildError> {
        let face = self.face_ref(face_id)?;
        let mut contours = Vec::with_capacity(face.inner().len() + 1);
        for wire_id in face.boundary_wires() {
            let wire = self.wire_ref(*wire_id)?;
            let segments = wire
                .edge_uses
                .iter()
                .map(|edge_use| {
                    let (start, end) = self.directed_vertices(*edge_use)?;
                    LineSeg2::try_new(
                        project_point_to_surface_plane(self.vertex_ref(start)?.point(), frame)?,
                        project_point_to_surface_plane(self.vertex_ref(end)?.point(), frame)?,
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                    .map_err(BuildError::from)
                })
                .collect::<Result<Vec<_>, _>>()?;
            contours.push(Contour2::try_new(segments).map_err(GeometryError::from)?);
        }
        let outer = contours.remove(0);
        Ok(LineArcRegion2::new(vec![outer], contours))
    }

    fn face_edge_set(&self, face: FaceId) -> Result<HashSet<EdgeId>, BuildError> {
        let face = self.face_ref(face)?;
        let mut edges = HashSet::new();
        for wire in face.boundary_wires() {
            edges.extend(self.wire_edge_set(*wire)?);
        }
        Ok(edges)
    }

    fn certify_line_arc_prism_shell(&self, shell: ShellId) -> Result<bool, BuildError> {
        let mut cap_surfaces = HashSet::new();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {
                    cap_surfaces.insert(face.surface);
                }
                SurfaceKind::Extrusion => {}
                _ => return Ok(false),
            }
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                    if !matches!(
                        self.curve_ref(edge.curve)?.kind(),
                        Curve3Kind::Line | Curve3Kind::CircleArc
                    ) {
                        return Ok(false);
                    }
                }
            }
        }
        if cap_surfaces.len() != 2 {
            return Ok(false);
        }
        self.certify_translation_shell_topology(shell)
    }

    fn certify_translation_shell_topology(&self, shell: ShellId) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 4 {
            return Ok(false);
        }
        let groups = self.planar_face_groups(faces)?;
        for first_index in 0..groups.len() {
            for second_index in (first_index + 1)..groups.len() {
                if self.certify_prism_cap_pair(
                    faces,
                    &groups[first_index],
                    &groups[second_index],
                )? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    fn certified_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedPrismShell>, BuildError> {
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_line_arc_prism_shell(shell)?
        {
            return Ok(None);
        }
        let faces = &self.shell_ref(shell)?.faces;
        let groups = self.planar_face_groups(faces)?;
        for first_index in 0..groups.len() {
            for second_index in (first_index + 1)..groups.len() {
                let first_group = &groups[first_index];
                let second_group = &groups[second_index];
                if !self.certify_prism_cap_pair(faces, first_group, second_group)? {
                    continue;
                }
                let first_face = self.face_ref(faces[first_group[0]])?;
                let SurfaceExactData::Plane { origin, u, v } =
                    self.surface_ref(first_face.surface)?.exact_data()
                else {
                    continue;
                };
                let first_boundary = self
                    .cap_boundary_use_loops(faces, first_group)?
                    .expect("certified cap group has a manifold boundary");
                let second_boundary = self
                    .cap_boundary_use_loops(faces, second_group)?
                    .expect("certified cap group has a manifold boundary");
                let first_vertices = self.boundary_loop_vertex_set(&first_boundary)?;
                let second_vertices = self.boundary_loop_vertex_set(&second_boundary)?;
                let cross_edge = self
                    .shell_edge_set(faces)?
                    .into_iter()
                    .find(|edge_id| {
                        let edge = &self.edges[edge_id.index()];
                        (first_vertices.contains(&edge.start)
                            && second_vertices.contains(&edge.end))
                            || (first_vertices.contains(&edge.end)
                                && second_vertices.contains(&edge.start))
                    })
                    .expect("certified cap pair has cross edges");
                let edge = self.edge_ref(cross_edge)?;
                let (first_vertex, second_vertex) = if first_vertices.contains(&edge.start) {
                    (edge.start, edge.end)
                } else {
                    (edge.end, edge.start)
                };
                let extrusion = self.vertex_ref(second_vertex)?.point()
                    - self.vertex_ref(first_vertex)?.point();
                let determinant = u.dot(&v.cross(&extrusion));
                if decided_model_order(compare_reals(&determinant, &Real::zero()))?
                    == std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let mut outer = None;
                let mut holes = Vec::new();
                for uses in first_boundary {
                    let segments = uses
                        .iter()
                        .map(|edge_use_id| {
                            let edge_use = self.edge_use_ref(*edge_use_id)?;
                            let edge = self.edge_ref(edge_use.edge)?;
                            if self.curve_ref(edge.curve)?.kind() == Curve3Kind::Line {
                                let (start, end) = self.directed_vertices(*edge_use_id)?;
                                Ok(Segment2::Line(
                                    LineSeg2::try_new(
                                        project_point_to_plane_frame(
                                            self.vertex_ref(start)?.point(),
                                            &origin,
                                            &u,
                                            &v,
                                        )?,
                                        project_point_to_plane_frame(
                                            self.vertex_ref(end)?.point(),
                                            &origin,
                                            &u,
                                            &v,
                                        )?,
                                    )
                                    .map_err(GeometryError::from)?,
                                ))
                            } else {
                                self.pcurve_ref(edge_use.pcurve)?
                                    .segment()
                                    .map_err(BuildError::from)
                            }
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    let contour = Contour2::try_new(segments).map_err(GeometryError::from)?;
                    let area = contour.signed_area().map_err(GeometryError::from)?.ok_or(
                        BuildError::DegenerateWireArea(
                            first_face.outer().expect("trimmed cap face"),
                        ),
                    )?;
                    let positive = decided_model_order(compare_reals(&area, &Real::zero()))?
                        == std::cmp::Ordering::Greater;
                    let is_outer = positive == (first_face.orientation == Orientation::Forward);
                    if is_outer {
                        if outer.replace(contour).is_some() {
                            return Ok(None);
                        }
                    } else {
                        holes.push(contour);
                    }
                }
                let Some(outer) = outer else {
                    return Ok(None);
                };
                return Ok(Some(CertifiedPrismShell {
                    outer,
                    holes,
                    origin,
                    u,
                    v,
                    extrusion,
                    parameter_min: Real::zero(),
                    parameter_max: Real::one(),
                }));
            }
        }
        Ok(None)
    }

    fn certified_spherical_cap_face(
        &self,
        face_id: FaceId,
    ) -> Result<Option<CertifiedSphericalCapFace>, BuildError> {
        let face = self.face_ref(face_id)?;
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        if !face.inner().is_empty() {
            return Ok(None);
        }
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let wire = self.wire_ref(outer)?;
        let Some(first_use) = wire.edge_uses.first() else {
            return Ok(None);
        };
        let pcurve = self.pcurve_ref(self.edge_use_ref(*first_use)?.pcurve)?;
        let Some(line) = pcurve.line_segment() else {
            return Ok(None);
        };
        let increasing = decided_model_order(compare_reals(line.end().x(), line.start().x()))?
            == std::cmp::Ordering::Greater;
        let upper = match face.orientation {
            Orientation::Forward => increasing,
            Orientation::Reversed => !increasing,
        };
        Ok(Some(CertifiedSphericalCapFace {
            center,
            axis,
            radius,
            latitude: line.start().y().clone(),
            upper,
            orientation: face.orientation,
        }))
    }

    fn certified_sphere_pair_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedSpherePairShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() != 2 {
            return Ok(None);
        }
        let Some(first) = self.certified_spherical_cap_face(faces[0])? else {
            return Ok(None);
        };
        let Some(second) = self.certified_spherical_cap_face(faces[1])? else {
            return Ok(None);
        };
        let displacement = &second.center - &first.center;
        let distance_squared = displacement.norm_squared();
        if decided_model_order(compare_reals(&distance_squared, &Real::zero()))?
            != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let radius_sum = &first.radius + &second.radius;
        let radius_difference = (&first.radius - &second.radius).abs();
        if decided_model_order(compare_reals(
            &distance_squared,
            &(&radius_sum * &radius_sum),
        ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &distance_squared,
                &(&radius_difference * &radius_difference),
            ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let first_pole = if first.upper {
            first.center.clone() + first.axis.clone() * &first.radius
        } else {
            first.center.clone() - first.axis.clone() * &first.radius
        };
        let second_pole = if second.upper {
            second.center.clone() + second.axis.clone() * &second.radius
        } else {
            second.center.clone() - second.axis.clone() * &second.radius
        };
        let first_inside_second = decided_model_order(compare_reals(
            &(&first_pole - &second.center).norm_squared(),
            &(&second.radius * &second.radius),
        ))? == std::cmp::Ordering::Less;
        let second_inside_first = decided_model_order(compare_reals(
            &(&second_pole - &first.center).norm_squared(),
            &(&first.radius * &first.radius),
        ))? == std::cmp::Ordering::Less;

        match (first.orientation, second.orientation) {
            (Orientation::Forward, Orientation::Forward)
                if first_inside_second == second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: first.center,
                    first_radius: first.radius,
                    second_center: second.center,
                    second_radius: second.radius,
                    kind: if first_inside_second {
                        CertifiedSpherePairKind::Intersection
                    } else {
                        CertifiedSpherePairKind::Union
                    },
                }))
            }
            (Orientation::Forward, Orientation::Reversed)
                if !first_inside_second && second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: first.center,
                    first_radius: first.radius,
                    second_center: second.center,
                    second_radius: second.radius,
                    kind: CertifiedSpherePairKind::Difference,
                }))
            }
            (Orientation::Reversed, Orientation::Forward)
                if first_inside_second && !second_inside_first =>
            {
                Ok(Some(CertifiedSpherePairShell {
                    first_center: second.center,
                    first_radius: second.radius,
                    second_center: first.center,
                    second_radius: first.radius,
                    kind: CertifiedSpherePairKind::Difference,
                }))
            }
            _ => Ok(None),
        }
    }

    fn certified_sphere_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        if let Some((center, radius)) =
            self.certified_oriented_sphere_shell(solid.outer, Orientation::Forward)?
        {
            let mut voids = Vec::with_capacity(solid.voids.len());
            for shell in &solid.voids {
                if let Some((void_center, void_radius)) =
                    self.certified_oriented_sphere_shell(*shell, Orientation::Reversed)?
                {
                    voids.push(CertifiedSphereVoid::Sphere {
                        center: void_center,
                        radius: void_radius,
                    });
                    continue;
                }
                if solid.voids.len() != 1 {
                    return Ok(None);
                }
                let Some(cylinder) =
                    self.certified_oriented_cylinder_shell(*shell, Orientation::Reversed)?
                else {
                    return Ok(None);
                };
                if !self.sphere_strictly_contains_cylinder(&center, &radius, &cylinder)? {
                    return Ok(None);
                }
                voids.push(CertifiedSphereVoid::Cylinder(Box::new(
                    CertifiedSphereCylinderVoid {
                        origin: cylinder.origin,
                        axis: cylinder.axis,
                        radius: cylinder.radius,
                        v_min: cylinder.v_min,
                        v_max: cylinder.v_max,
                    },
                )));
            }
            return Ok(Some(CertifiedSphereShell {
                center,
                radius,
                voids,
                region: CertifiedSphereRegion::Whole,
            }));
        }
        if !solid.voids.is_empty() {
            return Ok(None);
        }
        self.certified_sphere_segment_shell(solid.outer)
    }

    fn sphere_strictly_contains_cylinder(
        &self,
        sphere_center: &Point3,
        sphere_radius: &Real,
        cylinder: &CertifiedCylinderShell,
    ) -> Result<bool, BuildError> {
        if cylinder.sphere_subtraction.is_some() {
            return Ok(false);
        }
        CertifiedSphereProfile {
            center: sphere_center.clone(),
            radius: sphere_radius.clone(),
        }
        .strictly_contains_cylinder(&CertifiedCylinderProfile {
            origin: cylinder.origin.clone(),
            axis: cylinder.axis.clone(),
            radius: cylinder.radius.clone(),
            v_min: cylinder.v_min.clone(),
            v_max: cylinder.v_max.clone(),
        })
        .map_err(BuildError::from)
    }

    fn cylinder_strictly_contains_sphere(
        &self,
        cylinder: &CertifiedCylinderShell,
        sphere_center: &Point3,
        sphere_radius: &Real,
    ) -> Result<bool, BuildError> {
        if cylinder.sphere_subtraction.is_some() {
            return Ok(false);
        }
        CertifiedCylinderProfile {
            origin: cylinder.origin.clone(),
            axis: cylinder.axis.clone(),
            radius: cylinder.radius.clone(),
            v_min: cylinder.v_min.clone(),
            v_max: cylinder.v_max.clone(),
        }
        .strictly_contains_sphere(&CertifiedSphereProfile {
            center: sphere_center.clone(),
            radius: sphere_radius.clone(),
        })
        .map_err(BuildError::from)
    }

    fn certified_sphere_segment_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut sphere_faces = Vec::new();
        let mut cylinder_faces = Vec::new();
        let mut planar_faces = 0_usize;
        for face_id in faces {
            match self.surface_ref(self.face_ref(*face_id)?.surface)?.kind() {
                SurfaceKind::Plane => planar_faces += 1,
                SurfaceKind::Sphere => sphere_faces.push(*face_id),
                SurfaceKind::Cylinder => cylinder_faces.push(*face_id),
                _ => return Ok(None),
            }
        }
        if !cylinder_faces.is_empty() {
            if planar_faces == 0 {
                return self.certified_sphere_cylinder_shell(shell, &sphere_faces, &cylinder_faces);
            }
            if let Some(certificate) = self.certified_sphere_cylinder_union_shell(
                shell,
                faces,
                &sphere_faces,
                &cylinder_faces,
            )? {
                return Ok(Some(certificate));
            }
            return self.certified_sphere_cylinder_capped_shell(
                shell,
                faces,
                &sphere_faces,
                &cylinder_faces,
            );
        }
        if sphere_faces.len() != 1 {
            return Ok(None);
        }
        if !self.face_ref(sphere_faces[0])?.inner().is_empty() {
            return self.certified_sphere_band_shell(shell, sphere_faces[0]);
        }
        let Some(cap) = self.certified_spherical_cap_face(sphere_faces[0])? else {
            return Ok(None);
        };
        if cap.orientation != Orientation::Forward {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 1 {
            return Ok(None);
        }
        let height = &cap.radius * cap.latitude.clone().sin();
        let expected_center = cap.center.clone() + cap.axis.clone() * &height;
        let expected_radius = &cap.radius * cap.latitude.clone().cos();
        let Some(boundaries) = self.cap_boundary_use_loops(faces, &cap_groups[0])? else {
            return Ok(None);
        };
        if boundaries.len() != 1 {
            return Ok(None);
        }
        let mut sweep = Real::zero();
        for edge_use_id in &boundaries[0] {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(None);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &expected_radius)?
                || !real_values_equal(&circle.y_radius, &expected_radius)?
            {
                return Ok(None);
            }
            sweep += edge.domain.end() - edge.domain.start();
        }
        if !real_values_equal(&sweep, &Real::tau())? {
            return Ok(None);
        }

        let expected_outward = if cap.upper {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        for index in &cap_groups[0] {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            if decided_model_order(compare_reals(
                &normal.cross(&cap.axis).norm_squared(),
                &Real::zero(),
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - &cap.center).dot(&cap.axis), &height)?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(&cap.axis),
                Orientation::Reversed => -normal.dot(&cap.axis),
            };
            if decided_model_order(compare_reals(&oriented, &Real::zero()))? != expected_outward {
                return Ok(None);
            }
        }
        Ok(Some(CertifiedSphereShell {
            center: cap.center,
            radius: cap.radius.clone(),
            voids: Vec::new(),
            region: CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                axis: cap.axis,
                min: if cap.upper {
                    height.clone()
                } else {
                    -cap.radius.clone()
                },
                max: if cap.upper { cap.radius } else { height },
            }),
        }))
    }

    fn certified_sphere_band_shell(
        &self,
        shell: ShellId,
        sphere_face_id: FaceId,
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let sphere_face = self.face_ref(sphere_face_id)?;
        if sphere_face.orientation != Orientation::Forward {
            return Ok(None);
        }
        let [upper_wire] = sphere_face.inner() else {
            return Ok(None);
        };
        let Some(lower_wire) = sphere_face.outer() else {
            return Ok(None);
        };
        let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
        let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(sphere_face.surface)?.exact_data()
        else {
            unreachable!("sphere face carries sphere exact data");
        };
        let expected_min = &radius * lower_latitude.sin();
        let expected_max = &radius * upper_latitude.sin();
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 {
            return Ok(None);
        }
        let mut caps = Vec::with_capacity(2);
        for group in &cap_groups {
            let Some(cap) =
                self.certified_sphere_planar_cap_group(faces, group, &center, &axis, &radius)?
            else {
                return Ok(None);
            };
            caps.push(cap);
        }
        if decided_model_order(compare_reals(&caps[0].0, &caps[1].0))?
            == std::cmp::Ordering::Greater
        {
            caps.swap(0, 1);
        }
        if !real_values_equal(&caps[0].0, &expected_min)?
            || !real_values_equal(&caps[1].0, &expected_max)?
            || caps[0].1 != std::cmp::Ordering::Less
            || caps[1].1 != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::Axial(CertifiedSphereAxialClip {
                axis,
                min: expected_min,
                max: expected_max,
            }),
        }))
    }

    fn certified_sphere_planar_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        radius: &Real,
    ) -> Result<Option<(Real, std::cmp::Ordering)>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 1 {
            return Ok(None);
        }
        let first_use = *boundaries[0].first().ok_or(BuildError::EmptyWire)?;
        let first_edge = self.edge_ref(self.edge_use_ref(first_use)?.edge)?;
        let Curve3ExactData::EllipseArc(first_circle) =
            self.curve_ref(first_edge.curve)?.exact_data()
        else {
            return Ok(None);
        };
        if !first_circle.circle {
            return Ok(None);
        }
        let height = (&first_circle.center - center).dot(axis);
        let expected_center = center.clone() + axis.clone() * &height;
        let expected_radius = (radius * radius - &height * &height)
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        if !points_equal(&first_circle.center, &expected_center)? {
            return Ok(None);
        }
        let mut sweep = Real::zero();
        for edge_use_id in &boundaries[0] {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(None);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &expected_radius)?
                || !real_values_equal(&circle.y_radius, &expected_radius)?
            {
                return Ok(None);
            }
            sweep += edge.domain.end() - edge.domain.start();
        }
        if !real_values_equal(&sweep, &Real::tau())? {
            return Ok(None);
        }
        let mut outward = None;
        for index in group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            if decided_model_order(compare_reals(
                &normal.cross(axis).norm_squared(),
                &Real::zero(),
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - center).dot(axis), &height)?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(axis),
                Orientation::Reversed => -normal.dot(axis),
            };
            let direction = decided_model_order(compare_reals(&oriented, &Real::zero()))?;
            if direction == std::cmp::Ordering::Equal
                || outward.is_some_and(|expected| expected != direction)
            {
                return Ok(None);
            }
            outward = Some(direction);
        }
        Ok(outward.map(|outward| (height, outward)))
    }

    fn certified_sphere_cylinder_shell(
        &self,
        shell: ShellId,
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let (
            center,
            axis,
            radius,
            lower_latitude,
            upper_latitude,
            radial_side,
            cylinder_orientation,
        ) = match sphere_faces {
            [first_id, second_id] => {
                let Some(first) = self.certified_spherical_cap_face(*first_id)? else {
                    return Ok(None);
                };
                let Some(second) = self.certified_spherical_cap_face(*second_id)? else {
                    return Ok(None);
                };
                if first.orientation != Orientation::Forward
                    || second.orientation != Orientation::Forward
                    || first.upper == second.upper
                    || !points_equal(&first.center, &second.center)?
                    || !vectors_equal(&first.axis, &second.axis)?
                    || !real_values_equal(&first.radius, &second.radius)?
                {
                    return Ok(None);
                }
                let (lower, upper) = if first.upper {
                    (second, first)
                } else {
                    (first, second)
                };
                (
                    lower.center,
                    lower.axis,
                    lower.radius,
                    lower.latitude,
                    upper.latitude,
                    CertifiedSphereRadialSide::Inside,
                    Orientation::Forward,
                )
            }
            [band_id] => {
                let face = self.face_ref(*band_id)?;
                if face.orientation != Orientation::Forward {
                    return Ok(None);
                }
                let Some(lower_wire) = face.outer() else {
                    return Ok(None);
                };
                let [upper_wire] = face.inner() else {
                    return Ok(None);
                };
                let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
                let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
                let SurfaceExactData::Sphere {
                    center,
                    axis,
                    radius,
                    ..
                } = self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                (
                    center,
                    axis,
                    radius,
                    lower_latitude,
                    upper_latitude,
                    CertifiedSphereRadialSide::Outside,
                    Orientation::Reversed,
                )
            }
            _ => return Ok(None),
        };
        let Some(cylinder) =
            self.certified_cylinder_side_faces(shell, cylinder_faces, cylinder_orientation)?
        else {
            return Ok(None);
        };
        if !vectors_equal(&axis, &cylinder.axis)?
            || decided_model_order(compare_reals(&cylinder.radius, &radius))?
                != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &center - &cylinder.origin;
        let center_parameter = center_offset.dot(&cylinder.axis);
        let radial_offset = center_offset - cylinder.axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
            != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let lower_height = &radius * lower_latitude.clone().sin();
        let upper_height = &radius * upper_latitude.clone().sin();
        if decided_model_order(compare_reals(&lower_height, &Real::zero()))?
            != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(&upper_height, &Real::zero()))?
                != std::cmp::Ordering::Greater
            || !real_values_equal(&lower_height, &-upper_height.clone())?
            || !real_values_equal(&(&radius * lower_latitude.clone().cos()), &cylinder.radius)?
            || !real_values_equal(&(&radius * upper_latitude.clone().cos()), &cylinder.radius)?
            || !real_values_equal(&cylinder.v_min, &(&center_parameter + &lower_height))?
            || !real_values_equal(&cylinder.v_max, &(&center_parameter + &upper_height))?
        {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::Radial(CertifiedSphereRadialClip {
                axis: cylinder.axis,
                radius: cylinder.radius,
                side: radial_side,
            }),
        }))
    }

    fn certified_sphere_cylinder_union_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let [sphere_face_id] = sphere_faces else {
            return Ok(None);
        };
        let sphere_face = self.face_ref(*sphere_face_id)?;
        if sphere_face.orientation != Orientation::Forward {
            return Ok(None);
        }
        let Some(lower_wire) = sphere_face.outer() else {
            return Ok(None);
        };
        let [upper_wire] = sphere_face.inner() else {
            return Ok(None);
        };
        let (lower_latitude, _) = self.spherical_trim_coordinates(lower_wire)?;
        let (upper_latitude, _) = self.spherical_trim_coordinates(*upper_wire)?;
        let SurfaceExactData::Sphere {
            center,
            axis,
            radius,
            ..
        } = self.surface_ref(sphere_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let Some(first_cylinder_face) = cylinder_faces.first() else {
            return Ok(None);
        };
        let cylinder_surface_id = self.face_ref(*first_cylinder_face)?.surface;
        if cylinder_faces
            .iter()
            .any(|face| self.face_ref(*face).map(|face| face.surface) != Ok(cylinder_surface_id))
        {
            return Ok(None);
        }
        let SurfaceExactData::Cylinder {
            origin,
            axis: cylinder_axis,
            radius: cylinder_radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };
        if !vectors_equal(&axis, &cylinder_axis)?
            || decided_model_order(compare_reals(&cylinder_radius, &radius))?
                != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &center - &origin;
        let center_parameter = center_offset.dot(&cylinder_axis);
        let radial_offset = center_offset - cylinder_axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
            != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let lower_height = &radius * lower_latitude.clone().sin();
        let upper_height = &radius * upper_latitude.clone().sin();
        if decided_model_order(compare_reals(&lower_height, &Real::zero()))?
            != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(&upper_height, &Real::zero()))?
                != std::cmp::Ordering::Greater
            || !real_values_equal(&lower_height, &-upper_height.clone())?
            || !real_values_equal(&(&radius * lower_latitude.clone().cos()), &cylinder_radius)?
            || !real_values_equal(&(&radius * upper_latitude.clone().cos()), &cylinder_radius)?
        {
            return Ok(None);
        }
        let lower_intersection = &center_parameter + &lower_height;
        let upper_intersection = &center_parameter + &upper_height;
        let mut lower_faces = Vec::new();
        let mut upper_faces = Vec::new();
        for face in cylinder_faces {
            let Some((face_min, face_max)) = self.cylinder_face_v_bounds(*face)? else {
                return Ok(None);
            };
            if decided_model_order(compare_reals(&face_max, &lower_intersection))?
                != std::cmp::Ordering::Greater
            {
                lower_faces.push(*face);
            } else if decided_model_order(compare_reals(&face_min, &upper_intersection))?
                != std::cmp::Ordering::Less
            {
                upper_faces.push(*face);
            } else {
                return Ok(None);
            }
        }
        let Some(lower) =
            self.certified_cylinder_side_faces(shell, &lower_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        let Some(upper) =
            self.certified_cylinder_side_faces(shell, &upper_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        if !points_equal(&lower.origin, &upper.origin)?
            || !vectors_equal(&lower.axis, &upper.axis)?
            || !real_values_equal(&lower.radius, &upper.radius)?
            || !real_values_equal(&lower.v_max, &lower_intersection)?
            || !real_values_equal(&upper.v_min, &upper_intersection)?
            || decided_model_order(compare_reals(
                &(&lower.v_min - &center_parameter),
                &-radius.clone(),
            ))? != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(&(&upper.v_max - &center_parameter), &radius))?
                != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 {
            return Ok(None);
        }
        let mut matched = [false; 2];
        for group in &cap_groups {
            let lower_match = self.certified_cylinder_cap_group(
                faces,
                group,
                &lower,
                &lower.v_min,
                std::cmp::Ordering::Less,
            )?;
            let upper_match = self.certified_cylinder_cap_group(
                faces,
                group,
                &upper,
                &upper.v_max,
                std::cmp::Ordering::Greater,
            )?;
            match (lower_match, upper_match) {
                (true, false) if !matched[0] => matched[0] = true,
                (false, true) if !matched[1] => matched[1] = true,
                _ => return Ok(None),
            }
        }
        if matched != [true, true] {
            return Ok(None);
        }
        Ok(Some(CertifiedSphereShell {
            center,
            radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::FiniteCylinder(CertifiedSphereFiniteCylinderRegion {
                origin: lower.origin,
                axis: lower.axis,
                radius: lower.radius,
                v_min: lower.v_min,
                v_max: upper.v_max,
                operation: CertifiedSphereFiniteCylinderOperation::Union,
            }),
        }))
    }

    fn certified_oriented_sphere_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<(Point3, Real)>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        match faces.as_slice() {
            [face_id] => {
                let face = self.face_ref(*face_id)?;
                if !face.is_whole_surface() || face.orientation != orientation {
                    return Ok(None);
                }
                let SurfaceExactData::Sphere { center, radius, .. } =
                    self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                Ok(Some((center, radius)))
            }
            [first_id, second_id] => {
                let first_face = self.face_ref(*first_id)?;
                let second_face = self.face_ref(*second_id)?;
                if first_face.orientation != orientation
                    || second_face.orientation != orientation
                    || first_face.surface != second_face.surface
                {
                    return Ok(None);
                }
                let Some(first) = self.certified_spherical_cap_face(*first_id)? else {
                    return Ok(None);
                };
                let Some(second) = self.certified_spherical_cap_face(*second_id)? else {
                    return Ok(None);
                };
                if first.upper == second.upper
                    || !points_equal(&first.center, &second.center)?
                    || !vectors_equal(&first.axis, &second.axis)?
                    || !real_values_equal(&first.radius, &second.radius)?
                {
                    return Ok(None);
                }
                Ok(Some((first.center, first.radius)))
            }
            _ => self.certified_subdivided_sphere_shell(shell, orientation),
        }
    }

    fn certified_subdivided_sphere_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<(Point3, Real)>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut surface_id = None;
        let mut lower_cap = None;
        let mut upper_cap = None;
        let mut bands: Vec<(Real, Real)> = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if face.orientation != orientation
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::Sphere
            {
                return Ok(None);
            }
            match surface_id {
                Some(expected) if expected != face.surface => return Ok(None),
                Some(_) => {}
                None => surface_id = Some(face.surface),
            }
            if face.inner().is_empty() {
                let Some(cap) = self.certified_spherical_cap_face(*face_id)? else {
                    return Ok(None);
                };
                let slot = if cap.upper {
                    &mut upper_cap
                } else {
                    &mut lower_cap
                };
                if slot.replace(cap).is_some() {
                    return Ok(None);
                }
            } else if let [upper_wire] = face.inner() {
                let Some(lower_wire) = face.outer() else {
                    return Ok(None);
                };
                let (lower, _) = self.spherical_trim_coordinates(lower_wire)?;
                let (upper, _) = self.spherical_trim_coordinates(*upper_wire)?;
                if decided_model_order(compare_reals(&lower, &upper))? != std::cmp::Ordering::Less {
                    return Ok(None);
                }
                let mut insertion = bands.len();
                while insertion > 0
                    && decided_model_order(compare_reals(&lower, &bands[insertion - 1].0))?
                        == std::cmp::Ordering::Less
                {
                    insertion -= 1;
                }
                bands.insert(insertion, (lower, upper));
            } else {
                return Ok(None);
            }
        }
        let (Some(lower_cap), Some(upper_cap)) = (lower_cap, upper_cap) else {
            return Ok(None);
        };
        if bands.is_empty()
            || !points_equal(&lower_cap.center, &upper_cap.center)?
            || !vectors_equal(&lower_cap.axis, &upper_cap.axis)?
            || !real_values_equal(&lower_cap.radius, &upper_cap.radius)?
            || !real_values_equal(&bands[0].0, &lower_cap.latitude)?
            || !real_values_equal(
                &bands.last().expect("nonempty sphere bands").1,
                &upper_cap.latitude,
            )?
        {
            return Ok(None);
        }
        for pair in bands.windows(2) {
            if !real_values_equal(&pair[0].1, &pair[1].0)? {
                return Ok(None);
            }
        }
        Ok(Some((lower_cap.center, lower_cap.radius)))
    }

    fn certified_cone_frustum_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedConeFrustumShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut cone_surface = None;
        let mut u_values = Vec::new();
        let mut v_values = Vec::new();
        let mut face_coordinates = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(None);
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => continue,
                SurfaceKind::Cone if face.orientation == Orientation::Forward => {}
                _ => return Ok(None),
            }
            match cone_surface {
                Some(expected) if expected != face.surface => return Ok(None),
                Some(_) => {}
                None => cone_surface = Some(face.surface),
            }
            let Some(outer) = face.outer() else {
                return Ok(None);
            };
            let wire = self.wire_ref(outer)?;
            if wire.edge_uses.len() < 4 {
                return Ok(None);
            }
            let mut face_u = Vec::with_capacity(8);
            let mut face_v = Vec::with_capacity(8);
            let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
            for edge_use_id in &wire.edge_uses {
                let edge_use = self.edge_use_ref(*edge_use_id)?;
                let pcurve = self.pcurve_ref(edge_use.pcurve)?;
                let Some(line) = pcurve.line_segment() else {
                    return Ok(None);
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                if u_constant == v_constant {
                    return Ok(None);
                }
                face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                parameter_segments.push(Segment2::Line(line.clone()));
            }
            let (u_min, u_max) = exact_real_min_max(&face_u)?;
            let (v_min, v_max) = exact_real_min_max(&face_v)?;
            let contour = Contour2::try_new(parameter_segments).map_err(GeometryError::from)?;
            let represented_area = contour
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?
                .abs();
            let rectangle_area = (&u_max - &u_min) * (&v_max - &v_min);
            if !real_values_equal(&represented_area, &rectangle_area)? {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, &u_min)?;
            insert_sorted_real(&mut u_values, &u_max)?;
            insert_sorted_real(&mut v_values, &v_min)?;
            insert_sorted_real(&mut v_values, &v_max)?;
            face_coordinates.push((u_min, u_max, v_min, v_max));
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if !matches!(cap_groups.len(), 2 | 3)
            || face_coordinates.len() < 2
            || u_values.len() < 2
            || v_values.len() < 2
        {
            return Ok(None);
        }
        let v_min = v_values[0].clone();
        let v_max = v_values.last().expect("frustum has axial values").clone();
        if decided_model_order(compare_reals(&v_min, &Real::zero()))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let SurfaceExactData::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } = self
            .surface_ref(cone_surface.expect("frustum has cone faces"))?
            .exact_data()
        else {
            unreachable!("cone kind carries cone exact data");
        };
        let mut cells = HashSet::new();
        for (u_min, u_max, face_v_min, face_v_max) in &face_coordinates {
            let u_start = exact_real_index(&u_values, u_min)?;
            let u_end = exact_real_index(&u_values, u_max)?;
            let v_start = exact_real_index(&v_values, face_v_min)?;
            let v_end = exact_real_index(&v_values, face_v_max)?;
            if u_start >= u_end || v_start >= v_end {
                return Ok(None);
            }
            for u_cell in u_start..u_end {
                for v_cell in v_start..v_end {
                    if !cells.insert((u_cell, v_cell)) {
                        return Ok(None);
                    }
                }
            }
        }

        let region = if cap_groups.len() == 3 {
            let Some(interior_normal) = self.certified_cone_longitudinal_cap_groups(
                faces,
                &cap_groups,
                &apex,
                &axis,
                &semi_angle,
                &v_min,
                &v_max,
            )?
            else {
                return Ok(None);
            };
            if !certified_periodic_longitudinal_half_coverage(
                &u_values,
                &v_values,
                &cells,
                &x,
                &y,
                &interior_normal,
            )? {
                return Ok(None);
            }
            CertifiedConeFrustumRegion::LongitudinalHalf { interior_normal }
        } else {
            if face_coordinates.len() < 4
                || u_values.len() < 5
                || !real_values_equal(
                    &(u_values.last().expect("frustum u grid") - &u_values[0]),
                    &Real::tau(),
                )?
                || cells.len() != (u_values.len() - 1) * (v_values.len() - 1)
            {
                return Ok(None);
            }
            CertifiedConeFrustumRegion::Whole
        };
        Ok(Some(CertifiedConeFrustumShell {
            apex,
            axis,
            semi_angle,
            v_min,
            v_max,
            region,
        }))
    }

    #[allow(clippy::too_many_arguments)]
    fn certified_cone_longitudinal_cap_groups(
        &self,
        faces: &[FaceId],
        groups: &[Vec<usize>],
        apex: &Point3,
        axis: &Vector3,
        semi_angle: &Real,
        v_min: &Real,
        v_max: &Real,
    ) -> Result<Option<Vector3>, BuildError> {
        let cosine = semi_angle.clone().cos();
        let mut lower = false;
        let mut upper = false;
        let mut interior_normal = None;
        for group in groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
                return Ok(None);
            };
            if boundaries.len() != 1 {
                return Ok(None);
            }
            let mut group_origin = None;
            let mut group_normal = None::<Vector3>;
            for index in group {
                let face = self.face_ref(faces[*index])?;
                let SurfaceExactData::Plane { origin, u, v } =
                    self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                let normal = u.cross(&v);
                let oriented = match face.orientation {
                    Orientation::Forward => normal,
                    Orientation::Reversed => -normal,
                }
                .normalize()
                .map_err(|_| GeometryError::ElementaryFunction)?;
                if let Some(expected) = &group_normal {
                    if !vectors_equal(expected, &oriented)? {
                        return Ok(None);
                    }
                } else {
                    group_origin = Some(origin);
                    group_normal = Some(oriented);
                }
            }
            let Some(origin) = group_origin else {
                return Ok(None);
            };
            let outward = group_normal.expect("nonempty planar group");
            if real_values_equal(&outward.cross(axis).norm_squared(), &Real::zero())? {
                let parameter = ((&origin - apex).dot(axis) / &cosine)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                let direction =
                    decided_model_order(compare_reals(&outward.dot(axis), &Real::zero()))?;
                if real_values_equal(&parameter, v_min)?
                    && direction == std::cmp::Ordering::Less
                    && !lower
                {
                    lower = true;
                } else if real_values_equal(&parameter, v_max)?
                    && direction == std::cmp::Ordering::Greater
                    && !upper
                {
                    upper = true;
                } else {
                    return Ok(None);
                }
            } else if real_values_equal(&outward.dot(axis), &Real::zero())?
                && real_values_equal(&outward.dot(&(apex - &origin)), &Real::zero())?
                && interior_normal.is_none()
            {
                interior_normal = Some(-outward);
            } else {
                return Ok(None);
            }
        }
        if lower && upper {
            Ok(interior_normal)
        } else {
            Ok(None)
        }
    }

    fn certified_sphere_cylinder_capped_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        sphere_faces: &[FaceId],
        cylinder_faces: &[FaceId],
    ) -> Result<Option<CertifiedSphereShell>, BuildError> {
        let [sphere_face] = sphere_faces else {
            return Ok(None);
        };
        let Some(sphere_cap) = self.certified_spherical_cap_face(*sphere_face)? else {
            return Ok(None);
        };
        if sphere_cap.orientation != Orientation::Forward {
            return Ok(None);
        }
        let Some(first_cylinder_face) = cylinder_faces.first() else {
            return Ok(None);
        };
        let cylinder_surface_id = self.face_ref(*first_cylinder_face)?.surface;
        if cylinder_faces
            .iter()
            .any(|face| self.face_ref(*face).map(|face| face.surface) != Ok(cylinder_surface_id))
        {
            return Ok(None);
        }
        let SurfaceExactData::Cylinder {
            origin,
            axis,
            radius: cylinder_radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };
        if !vectors_equal(&sphere_cap.axis, &axis)?
            || decided_model_order(compare_reals(&cylinder_radius, &sphere_cap.radius))?
                != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &sphere_cap.center - &origin;
        let center_parameter = center_offset.dot(&axis);
        let radial_offset = center_offset - axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
            != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let intersection_height = &sphere_cap.radius * sphere_cap.latitude.clone().sin();
        let height_order = decided_model_order(compare_reals(&intersection_height, &Real::zero()))?;
        if height_order == std::cmp::Ordering::Equal
            || !real_values_equal(
                &(&sphere_cap.radius * sphere_cap.latitude.clone().cos()),
                &cylinder_radius,
            )?
        {
            return Ok(None);
        }
        let intersection_parameter = &center_parameter + &intersection_height;
        let (orientation, cylinder) = match (
            self.certified_cylinder_side_faces(shell, cylinder_faces, Orientation::Forward)?,
            self.certified_cylinder_side_faces(shell, cylinder_faces, Orientation::Reversed)?,
        ) {
            (Some(cylinder), None) => (Orientation::Forward, cylinder),
            (None, Some(cylinder)) => (Orientation::Reversed, cylinder),
            _ => return Ok(None),
        };
        let intersection_at_min = real_values_equal(&cylinder.v_min, &intersection_parameter)?;
        let intersection_at_max = real_values_equal(&cylinder.v_max, &intersection_parameter)?;
        if intersection_at_min == intersection_at_max {
            return Ok(None);
        }
        let extends_positive = intersection_at_min;
        let extends_toward_center = match height_order {
            std::cmp::Ordering::Less => extends_positive,
            std::cmp::Ordering::Greater => !extends_positive,
            std::cmp::Ordering::Equal => unreachable!("zero height was rejected"),
        };
        let operation = match (orientation, extends_toward_center) {
            (Orientation::Forward, true) => CertifiedSphereFiniteCylinderOperation::Intersection,
            (Orientation::Forward, false) => CertifiedSphereFiniteCylinderOperation::Union,
            (Orientation::Reversed, true) => CertifiedSphereFiniteCylinderOperation::Difference,
            (Orientation::Reversed, false) => return Ok(None),
        };
        let sphere_retains_intersection_side =
            sphere_cap.upper == (height_order == std::cmp::Ordering::Greater);
        if sphere_retains_intersection_side
            != (operation == CertifiedSphereFiniteCylinderOperation::Intersection)
        {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        let [cap_group] = cap_groups.as_slice() else {
            return Ok(None);
        };
        let cap_parameter = if intersection_at_min {
            cylinder.v_max.clone()
        } else {
            cylinder.v_min.clone()
        };
        let ordinary_cap_direction = if intersection_at_min {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Less
        };
        let expected_cap_direction = if orientation == Orientation::Forward {
            ordinary_cap_direction
        } else {
            ordinary_cap_direction.reverse()
        };
        if !self.certified_cylinder_cap_group(
            faces,
            cap_group,
            &cylinder,
            &cap_parameter,
            expected_cap_direction,
        )? {
            return Ok(None);
        }
        let relative_cap = &cap_parameter - &center_parameter;
        match operation {
            CertifiedSphereFiniteCylinderOperation::Union => {
                let outside_pole = if extends_positive {
                    decided_model_order(compare_reals(&relative_cap, &sphere_cap.radius))?
                        == std::cmp::Ordering::Greater
                } else {
                    decided_model_order(compare_reals(&relative_cap, &-sphere_cap.radius.clone()))?
                        == std::cmp::Ordering::Less
                };
                if !outside_pole {
                    return Ok(None);
                }
            }
            CertifiedSphereFiniteCylinderOperation::Intersection
            | CertifiedSphereFiniteCylinderOperation::Difference => {
                let half_height = (&sphere_cap.radius * &sphere_cap.radius
                    - &cylinder_radius * &cylinder_radius)
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                if decided_model_order(compare_reals(&relative_cap, &-half_height.clone()))?
                    != std::cmp::Ordering::Greater
                    || decided_model_order(compare_reals(&relative_cap, &half_height))?
                        != std::cmp::Ordering::Less
                {
                    return Ok(None);
                }
            }
        }
        let (region_min, region_max) = match operation {
            CertifiedSphereFiniteCylinderOperation::Union => (cylinder.v_min, cylinder.v_max),
            CertifiedSphereFiniteCylinderOperation::Intersection
            | CertifiedSphereFiniteCylinderOperation::Difference => {
                if height_order == std::cmp::Ordering::Less {
                    (
                        &center_parameter - &sphere_cap.radius - &cylinder.radius,
                        cap_parameter.clone(),
                    )
                } else {
                    (
                        cap_parameter.clone(),
                        &center_parameter + &sphere_cap.radius + &cylinder.radius,
                    )
                }
            }
        };
        Ok(Some(CertifiedSphereShell {
            center: sphere_cap.center,
            radius: sphere_cap.radius,
            voids: Vec::new(),
            region: CertifiedSphereRegion::FiniteCylinder(CertifiedSphereFiniteCylinderRegion {
                origin: cylinder.origin,
                axis: cylinder.axis,
                radius: cylinder.radius,
                v_min: region_min,
                v_max: region_max,
                operation,
            }),
        }))
    }

    fn certified_torus_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedTorusShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut side_faces = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::Torus
                    if face.orientation == Orientation::Forward && face.inner().is_empty() =>
                {
                    side_faces.push(*face_id);
                }
                _ => return Ok(None),
            }
        }
        if side_faces.is_empty() {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() > 2 {
            return Ok(None);
        }

        let mut surface_id = None;
        let mut u_values = Vec::new();
        let quarter =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        let mut v_values = (0..=4)
            .map(|index| &quarter * Real::from(index))
            .collect::<Vec<_>>();
        let mut face_coordinates = Vec::with_capacity(side_faces.len());
        for face_id in &side_faces {
            let face = self.face_ref(*face_id)?;
            match surface_id {
                Some(expected) if expected != face.surface => return Ok(None),
                Some(_) => {}
                None => surface_id = Some(face.surface),
            }
            let Some(outer) = face.outer() else {
                return Ok(None);
            };
            let wire = self.wire_ref(outer)?;
            if wire.edge_uses.len() < 4 {
                return Ok(None);
            }
            let mut face_u = Vec::with_capacity(8);
            let mut face_v = Vec::with_capacity(8);
            let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
            for edge_use_id in &wire.edge_uses {
                let edge_use = self.edge_use_ref(*edge_use_id)?;
                if self.curve_ref(self.edge_ref(edge_use.edge)?.curve)?.kind()
                    != Curve3Kind::CircleArc
                {
                    return Ok(None);
                }
                let pcurve = self.pcurve_ref(edge_use.pcurve)?;
                let Some(line) = pcurve.line_segment() else {
                    return Ok(None);
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                if u_constant == v_constant {
                    return Ok(None);
                }
                face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                parameter_segments.push(Segment2::Line(line.clone()));
            }
            let (u_min, u_max) = exact_real_min_max(&face_u)?;
            let (v_min, v_max) = exact_real_min_max(&face_v)?;
            let contour = Contour2::try_new(parameter_segments).map_err(GeometryError::from)?;
            let represented_area = contour
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?
                .abs();
            if !real_values_equal(&represented_area, &((&u_max - &u_min) * (&v_max - &v_min)))? {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, &u_min)?;
            insert_sorted_real(&mut u_values, &u_max)?;
            insert_sorted_real(&mut v_values, &v_min)?;
            insert_sorted_real(&mut v_values, &v_max)?;
            face_coordinates.push((u_min, u_max, v_min, v_max));
        }
        let zero = Real::zero();
        let tau = Real::tau();
        for (_, _, v_min, v_max) in &face_coordinates {
            if decided_model_order(compare_reals(v_min, &zero))? == std::cmp::Ordering::Less
                || decided_model_order(compare_reals(v_max, &tau))? == std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
        }

        let SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
            ..
        } = self
            .surface_ref(surface_id.expect("torus shell has faces"))?
            .exact_data()
        else {
            unreachable!("torus kind carries torus exact data");
        };
        let mut cells = HashSet::new();
        for (u_min, u_max, v_min, v_max) in &face_coordinates {
            let u_start = exact_real_index(&u_values, u_min)?;
            let u_end = exact_real_index(&u_values, u_max)?;
            let v_start = exact_real_index(&v_values, v_min)?;
            let v_end = exact_real_index(&v_values, v_max)?;
            if u_start >= u_end || v_start >= v_end {
                return Ok(None);
            }
            for u_cell in u_start..u_end {
                for v_cell in v_start..v_end {
                    if !cells.insert((u_cell, v_cell)) {
                        return Ok(None);
                    }
                }
            }
        }

        if let [group] = cap_groups.as_slice()
            && let Some(interior_normal) = self.certified_torus_longitudinal_cap_group(
                faces,
                group,
                &center,
                &axis,
                &major_radius,
                &minor_radius,
            )?
        {
            if !real_values_equal(
                &(v_values.last().expect("torus v grid") - &v_values[0]),
                &Real::tau(),
            )? || !certified_periodic_longitudinal_half_coverage(
                &u_values,
                &v_values,
                &cells,
                &x,
                &y,
                &interior_normal,
            )? {
                return Ok(None);
            }
            return Ok(Some(CertifiedTorusShell {
                center,
                axis,
                major_radius,
                minor_radius,
                region: CertifiedTorusRegion::LongitudinalHalf { interior_normal },
            }));
        }

        if u_values.len() < 5
            || !real_values_equal(
                &(u_values.last().expect("torus u grid") - &u_values[0]),
                &Real::tau(),
            )?
        {
            return Ok(None);
        }
        let mut caps = Vec::with_capacity(cap_groups.len());
        for group in &cap_groups {
            let Some(cap) = self.certified_torus_cap_group(
                faces,
                group,
                &center,
                &axis,
                &major_radius,
                &minor_radius,
            )?
            else {
                return Ok(None);
            };
            caps.push(cap);
        }
        let (axial_min, axial_max) = match caps.len() {
            0 => (-minor_radius.clone(), minor_radius.clone()),
            1 if caps[0].1 == std::cmp::Ordering::Less => (caps[0].0.clone(), minor_radius.clone()),
            1 if caps[0].1 == std::cmp::Ordering::Greater => {
                (-minor_radius.clone(), caps[0].0.clone())
            }
            1 => return Ok(None),
            2 => {
                if decided_model_order(compare_reals(&caps[0].0, &caps[1].0))?
                    == std::cmp::Ordering::Greater
                {
                    caps.swap(0, 1);
                }
                if decided_model_order(compare_reals(&caps[0].0, &caps[1].0))?
                    != std::cmp::Ordering::Less
                    || caps[0].1 != std::cmp::Ordering::Less
                    || caps[1].1 != std::cmp::Ordering::Greater
                {
                    return Ok(None);
                }
                (caps[0].0.clone(), caps[1].0.clone())
            }
            _ => unreachable!("at most two torus cap groups"),
        };

        for v_cell in 0..v_values.len() - 1 {
            let first_height = &minor_radius * v_values[v_cell].clone().sin();
            let second_height = &minor_radius * v_values[v_cell + 1].clone().sin();
            let (cell_min, cell_max) = exact_real_min_max(&[first_height, second_height])?;
            let expected = decided_model_order(compare_reals(&cell_min, &axial_min))?
                != std::cmp::Ordering::Less
                && decided_model_order(compare_reals(&cell_max, &axial_max))?
                    != std::cmp::Ordering::Greater;
            let actual = (0..u_values.len() - 1).all(|u_cell| cells.contains(&(u_cell, v_cell)));
            let partial = (0..u_values.len() - 1).any(|u_cell| cells.contains(&(u_cell, v_cell)));
            if actual != expected || (partial && !actual) {
                return Ok(None);
            }
        }
        Ok(Some(CertifiedTorusShell {
            center,
            axis,
            major_radius,
            minor_radius,
            region: if cap_groups.is_empty() {
                CertifiedTorusRegion::Whole
            } else {
                CertifiedTorusRegion::Axial {
                    min: axial_min,
                    max: axial_max,
                }
            },
        }))
    }

    fn certified_torus_longitudinal_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Option<Vector3>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 2 {
            return Ok(None);
        }

        let mut circle_centers = Vec::with_capacity(2);
        for boundary in &boundaries {
            let mut boundary_center = None;
            let mut sweep = Real::zero();
            for edge_use_id in boundary {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
                else {
                    return Ok(None);
                };
                if !circle.circle
                    || !real_values_equal(&circle.x_radius, minor_radius)?
                    || !real_values_equal(&circle.y_radius, minor_radius)?
                {
                    return Ok(None);
                }
                if let Some(expected) = &boundary_center {
                    if !points_equal(expected, &circle.center)? {
                        return Ok(None);
                    }
                } else {
                    boundary_center = Some(circle.center.clone());
                }
                sweep += edge.domain.end() - edge.domain.start();
            }
            if !real_values_equal(&sweep, &Real::tau())? {
                return Ok(None);
            }
            let Some(boundary_center) = boundary_center else {
                return Ok(None);
            };
            let offset = &boundary_center - center;
            if !real_values_equal(&offset.dot(axis), &Real::zero())?
                || !real_values_equal(&offset.norm_squared(), &(major_radius * major_radius))?
            {
                return Ok(None);
            }
            circle_centers.push(boundary_center);
        }
        let first_offset = &circle_centers[0] - center;
        let second_offset = &circle_centers[1] - center;
        if !real_values_equal(
            &(first_offset + second_offset).norm_squared(),
            &Real::zero(),
        )? {
            return Ok(None);
        }

        let mut outward = None::<Vector3>;
        for index in group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            if !real_values_equal(&normal.dot(axis), &Real::zero())?
                || !real_values_equal(&normal.dot(&(center - &origin)), &Real::zero())?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal,
                Orientation::Reversed => -normal,
            }
            .normalize()
            .map_err(|_| GeometryError::ElementaryFunction)?;
            if let Some(expected) = &outward {
                if !vectors_equal(expected, &oriented)? {
                    return Ok(None);
                }
            } else {
                outward = Some(oriented);
            }
        }
        Ok(outward.map(|outward| -outward))
    }

    fn certified_torus_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Option<(Real, std::cmp::Ordering)>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 2 {
            return Ok(None);
        }
        let first_use = *boundaries[0].first().ok_or(BuildError::EmptyWire)?;
        let first_edge = self.edge_ref(self.edge_use_ref(first_use)?.edge)?;
        let Curve3ExactData::EllipseArc(first_circle) =
            self.curve_ref(first_edge.curve)?.exact_data()
        else {
            return Ok(None);
        };
        if !first_circle.circle {
            return Ok(None);
        }
        let axial = (&first_circle.center - center).dot(axis);
        let expected_center = center.clone() + axis.clone() * &axial;
        if !points_equal(&first_circle.center, &expected_center)?
            || decided_model_order(compare_reals(&axial, &(-minor_radius.clone())))?
                != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&axial, minor_radius))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let radial_offset = (minor_radius * minor_radius - &axial * &axial)
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let expected_radii = [major_radius - &radial_offset, major_radius + &radial_offset];
        let mut matched_radii = [false; 2];
        for boundary in &boundaries {
            let mut sweep = Real::zero();
            let mut radius_index = None;
            for edge_use_id in boundary {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
                else {
                    return Ok(None);
                };
                if !circle.circle || !points_equal(&circle.center, &expected_center)? {
                    return Ok(None);
                }
                let mut this_radius = None;
                for (index, expected) in expected_radii.iter().enumerate() {
                    if real_values_equal(&circle.x_radius, expected)?
                        && real_values_equal(&circle.y_radius, expected)?
                    {
                        this_radius = Some(index);
                        break;
                    }
                }
                let Some(this_radius) = this_radius else {
                    return Ok(None);
                };
                if radius_index.is_some_and(|index| index != this_radius) {
                    return Ok(None);
                }
                radius_index = Some(this_radius);
                sweep += edge.domain.end() - edge.domain.start();
            }
            if !real_values_equal(&sweep, &Real::tau())? {
                return Ok(None);
            }
            let Some(radius_index) = radius_index else {
                return Ok(None);
            };
            if matched_radii[radius_index] {
                return Ok(None);
            }
            matched_radii[radius_index] = true;
        }
        if matched_radii != [true, true] {
            return Ok(None);
        }

        let mut outward = None;
        for index in group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            let axial_normal = normal.dot(axis);
            if decided_model_order(compare_reals(
                &normal.cross(axis).norm_squared(),
                &Real::zero(),
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - center).dot(axis), &axial)?
            {
                return Ok(None);
            }
            let oriented_axial = match face.orientation {
                Orientation::Forward => axial_normal,
                Orientation::Reversed => -axial_normal,
            };
            let this_outward = decided_model_order(compare_reals(&oriented_axial, &Real::zero()))?;
            if this_outward == std::cmp::Ordering::Equal
                || outward.is_some_and(|expected| expected != this_outward)
            {
                return Ok(None);
            }
            outward = Some(this_outward);
        }
        Ok(outward.map(|outward| (axial, outward)))
    }

    fn certified_curve_sweep_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 5 {
            return Ok(None);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            let kind = self.surface_ref(face.surface)?.kind();
            if !matches!(kind, SurfaceKind::Plane | SurfaceKind::RationalBezier)
                || (kind == SurfaceKind::RationalBezier && !face.inner().is_empty())
            {
                return Ok(None);
            }
        }
        if let Some(certificate) = self.certified_grouped_curve_sweep_shell(shell)? {
            return Ok(Some(certificate));
        }
        for lower in faces {
            let lower_face = self.face_ref(*lower)?;
            if lower_face.orientation != Orientation::Reversed
                || self.surface_ref(lower_face.surface)?.kind() != SurfaceKind::Plane
            {
                continue;
            }
            for upper in faces {
                let upper_face = self.face_ref(*upper)?;
                if upper_face.orientation != Orientation::Forward
                    || self.surface_ref(upper_face.surface)?.kind() != SurfaceKind::Plane
                {
                    continue;
                }
                if let Some(certificate) =
                    self.certified_curve_sweep_cap_pair(shell, *lower, *upper)?
                {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    fn certified_grouped_curve_sweep_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let planar_groups = self.planar_face_groups(faces)?;
        for lower_group in &planar_groups {
            if lower_group
                .iter()
                .any(|index| self.faces[faces[*index].index()].orientation != Orientation::Reversed)
            {
                continue;
            }
            for upper_group in &planar_groups {
                if lower_group == upper_group
                    || upper_group.iter().any(|index| {
                        self.faces[faces[*index].index()].orientation != Orientation::Forward
                    })
                {
                    continue;
                }
                if let Some(certificate) =
                    self.certified_grouped_curve_sweep_cap_pair(faces, lower_group, upper_group)?
                {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    fn certified_grouped_curve_sweep_cap_pair(
        &self,
        faces: &[FaceId],
        lower_group: &[usize],
        upper_group: &[usize],
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let Some(lower_boundaries) = self.cap_boundary_use_loops(faces, lower_group)? else {
            return Ok(None);
        };
        let Some(upper_boundaries) = self.cap_boundary_use_loops(faces, upper_group)? else {
            return Ok(None);
        };
        if lower_boundaries.is_empty()
            || lower_boundaries.len() != upper_boundaries.len()
            || lower_boundaries.iter().any(|boundary| boundary.len() < 3)
            || upper_boundaries.iter().any(|boundary| boundary.len() < 3)
        {
            return Ok(None);
        }

        let lower_face = self.face_ref(faces[lower_group[0]])?;
        let upper_face = self.face_ref(faces[upper_group[0]])?;
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.boundary_loop_vertex_set(&lower_boundaries)?;
        let upper_vertices = self.boundary_loop_vertex_set(&upper_boundaries)?;
        if !lower_vertices.is_disjoint(&upper_vertices) {
            return Ok(None);
        }
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        let mut correspondence = HashMap::new();
        let mut connector_by_lower = HashMap::new();
        let mut path_parameter_bounds = None;
        for chain in
            self.complete_tensor_path_chains(faces, &face_set, &lower_vertices, &upper_vertices)?
        {
            if let Some((start, end)) = &path_parameter_bounds {
                if !real_values_equal(start, &chain.parameter_start)?
                    || !real_values_equal(end, &chain.parameter_end)?
                {
                    return Ok(None);
                }
            } else {
                path_parameter_bounds =
                    Some((chain.parameter_start.clone(), chain.parameter_end.clone()));
            }
            if correspondence.insert(chain.lower, chain.upper).is_some()
                || connector_by_lower
                    .insert(chain.lower, chain.curve)
                    .is_some()
            {
                return Ok(None);
            }
        }
        let Some((path_parameter_start, path_parameter_end)) = path_parameter_bounds else {
            return Ok(None);
        };
        let count = correspondence.len();
        if count < 3
            || connector_by_lower.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }

        let reference = *correspondence
            .keys()
            .min()
            .expect("grouped curve sweep has connectors");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let path = connector_by_lower[&reference].clone();
        if !real_values_equal(path.domain().start(), &Real::zero())?
            || !real_values_equal(path.domain().end(), &Real::one())?
            || !certified_affine_sweep_progress(&path, &lower_u.cross(&lower_v))?
        {
            return Ok(None);
        }
        let Curve3ExactData::RationalBezier {
            control_points: path_controls,
            weights: path_weights,
        } = path.exact_data()
        else {
            unreachable!("connector kind was checked");
        };
        let mut lower_coordinates = HashMap::with_capacity(count);
        let mut connector_controls = HashMap::with_capacity(count);
        let mut sorted_lower = correspondence.keys().copied().collect::<Vec<_>>();
        sorted_lower.sort();
        for lower in &sorted_lower {
            lower_coordinates.insert(
                *lower,
                project_point_to_plane_frame(
                    self.vertex_ref(*lower)?.point(),
                    &reference_point,
                    &lower_u,
                    &lower_v,
                )?,
            );
            let Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } = connector_by_lower[lower].exact_data()
            else {
                unreachable!("connector kind was checked");
            };
            if control_points.len() != path_controls.len()
                || !rational_bezier_weights_proportional(&weights, &path_weights)?
            {
                return Ok(None);
            }
            connector_controls.insert(*lower, control_points);
        }

        let mut basis = None;
        for first in &sorted_lower {
            let first_coordinate = &lower_coordinates[first];
            for second in &sorted_lower {
                let second_coordinate = &lower_coordinates[second];
                let determinant = first_coordinate.x() * second_coordinate.y()
                    - first_coordinate.y() * second_coordinate.x();
                if decided_model_order(compare_reals(&determinant, &Real::zero()))?
                    != std::cmp::Ordering::Equal
                {
                    basis = Some((*first, *second, determinant));
                    break;
                }
            }
            if basis.is_some() {
                break;
            }
        }
        let Some((first_basis, second_basis, basis_determinant)) = basis else {
            return Ok(None);
        };
        let first_coordinate = &lower_coordinates[&first_basis];
        let second_coordinate = &lower_coordinates[&second_basis];
        let first_controls = &connector_controls[&first_basis];
        let second_controls = &connector_controls[&second_basis];
        let mut u_controls = Vec::with_capacity(path_controls.len());
        let mut v_controls = Vec::with_capacity(path_controls.len());
        for index in 0..path_controls.len() {
            let first_difference = &first_controls[index] - &path_controls[index];
            let second_difference = &second_controls[index] - &path_controls[index];
            u_controls.push(divide_vector_by_real(
                first_difference.clone() * second_coordinate.y()
                    - second_difference.clone() * first_coordinate.y(),
                &basis_determinant,
            )?);
            v_controls.push(divide_vector_by_real(
                second_difference * first_coordinate.x() - first_difference * second_coordinate.x(),
                &basis_determinant,
            )?);
        }
        let Some(area_scale_integral) = certified_sweep_frame_area_integral(
            &u_controls,
            &v_controls,
            &path_weights,
            &lower_u.cross(&lower_v),
        )?
        else {
            return Ok(None);
        };
        for lower in &sorted_lower {
            let coordinate = &lower_coordinates[lower];
            for (index, actual) in connector_controls[lower].iter().enumerate() {
                let expected = path_controls[index].clone()
                    + u_controls[index].clone() * coordinate.x()
                    + v_controls[index].clone() * coordinate.y();
                if !points_equal(actual, &expected)? {
                    return Ok(None);
                }
            }
        }

        let ordered_boundaries = lower_boundaries
            .iter()
            .map(|boundary| {
                let mut ordered = boundary
                    .iter()
                    .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
                    .collect::<Result<Vec<_>, _>>()?;
                ordered.reverse();
                Ok(ordered)
            })
            .collect::<Result<Vec<_>, BuildError>>()?;
        let ordered_corner_boundaries = ordered_boundaries
            .iter()
            .map(|boundary| {
                boundary
                    .iter()
                    .copied()
                    .filter(|vertex| correspondence.contains_key(vertex))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let ordered_corners = ordered_corner_boundaries
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        if ordered_corner_boundaries
            .iter()
            .any(|boundary| boundary.len() < 3)
            || ordered_corners.len() != count
            || ordered_corners
                .iter()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }

        let cap_indices = lower_group
            .iter()
            .chain(upper_group)
            .copied()
            .collect::<HashSet<_>>();
        let mut side_groups = HashMap::<SurfaceId, Vec<usize>>::new();
        for (index, face_id) in faces.iter().enumerate() {
            if cap_indices.contains(&index) {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            if face.orientation != Orientation::Forward
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::RationalBezier
            {
                return Ok(None);
            }
            side_groups.entry(face.surface).or_default().push(index);
        }
        if side_groups.len() != count {
            return Ok(None);
        }
        let mut represented_surfaces = HashSet::with_capacity(count);
        for ordered_corners in &ordered_corner_boundaries {
            for index in 0..ordered_corners.len() {
                let start = ordered_corners[index];
                let end = ordered_corners[(index + 1) % ordered_corners.len()];
                let start_coordinate = &lower_coordinates[&start];
                let end_coordinate = &lower_coordinates[&end];
                let mut matching = None;
                for (surface_id, group) in &side_groups {
                    let restricted_surface = self.restricted_curve_sweep_side_surface(
                        *surface_id,
                        &path_parameter_start,
                        &path_parameter_end,
                    )?;
                    let SurfaceExactData::RationalBezier {
                        control_points,
                        weights,
                    } = restricted_surface.exact_data()
                    else {
                        unreachable!("side surface kind was checked");
                    };
                    if framed_rational_bezier_surface_equal(
                        &control_points,
                        &weights,
                        RationalBezierSweepControlView {
                            path_points: &path_controls,
                            path_weights: &path_weights,
                            u_controls: &u_controls,
                            v_controls: &v_controls,
                        },
                        start_coordinate,
                        end_coordinate,
                    )? && matching.replace((*surface_id, group)).is_some()
                    {
                        return Ok(None);
                    }
                }
                let Some((surface_id, group)) = matching else {
                    return Ok(None);
                };
                if !represented_surfaces.insert(surface_id)
                    || !self.certifies_tensor_face_group(
                        faces,
                        group,
                        &Real::zero(),
                        &Real::one(),
                        &path_parameter_start,
                        &path_parameter_end,
                    )?
                {
                    return Ok(None);
                }
            }
        }
        if represented_surfaces.len() != side_groups.len() {
            return Ok(None);
        }

        let contours = ordered_boundaries
            .iter()
            .map(|boundary| {
                let points = boundary
                    .iter()
                    .map(|vertex| {
                        project_point_to_plane_frame(
                            self.vertex_ref(*vertex)?.point(),
                            &reference_point,
                            &lower_u,
                            &lower_v,
                        )
                    })
                    .collect::<Result<Vec<_>, BuildError>>()?;
                Ok(Contour2::try_new(
                    (0..points.len())
                        .map(|index| {
                            LineSeg2::try_new(
                                points[index].clone(),
                                points[(index + 1) % points.len()].clone(),
                            )
                            .map(Segment2::Line)
                            .map_err(GeometryError::from)
                        })
                        .collect::<Result<Vec<_>, _>>()?,
                )
                .map_err(GeometryError::from)?)
            })
            .collect::<Result<Vec<_>, BuildError>>()?;
        let mut profile = None;
        let mut holes = Vec::new();
        for contour in contours {
            let area = contour.signed_area().map_err(GeometryError::from)?.ok_or(
                BuildError::DegenerateShellVolume(
                    self.face_shell[faces[0].index()].expect("shell faces are assigned"),
                ),
            )?;
            match decided_model_order(compare_reals(&area, &Real::zero()))? {
                std::cmp::Ordering::Greater => {
                    if profile.replace(contour).is_some() {
                        return Ok(None);
                    }
                }
                std::cmp::Ordering::Less => holes.push(contour),
                std::cmp::Ordering::Equal => return Ok(None),
            }
        }
        let Some(profile) = profile else {
            return Ok(None);
        };
        Ok(Some(CertifiedCurveSweepShell {
            profile,
            holes,
            path,
            u_path: vector_control_curve(u_controls, &path_weights)?,
            v_path: vector_control_curve(v_controls, &path_weights)?,
            area_scale_integral,
        }))
    }

    fn complete_tensor_path_chains(
        &self,
        faces: &[FaceId],
        shell_faces: &HashSet<FaceId>,
        lower_vertices: &HashSet<VertexId>,
        upper_vertices: &HashSet<VertexId>,
    ) -> Result<Vec<TensorPathChain>, BuildError> {
        let mut groups = HashMap::<Curve3Id, Vec<(EdgeId, Real, Real)>>::new();
        for edge_id in self.shell_edge_set(faces)? {
            let edge = self.edge_ref(edge_id)?;
            if self.curve_ref(edge.curve)?.kind() != Curve3Kind::RationalBezier {
                continue;
            }
            let Some((start, end)) = self.tensor_path_boundary_interval(edge_id, shell_faces)?
            else {
                continue;
            };
            groups
                .entry(edge.curve)
                .or_default()
                .push((edge_id, start, end));
        }

        let mut chains = Vec::new();
        for (curve_id, segments) in groups {
            let curve = self.curve_ref(curve_id)?;
            let edge_intervals = segments
                .iter()
                .map(|(edge, _, _)| {
                    let domain = self.edge_ref(*edge)?.domain.clone();
                    Ok((domain.start().clone(), domain.end().clone()))
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            if !self.exact_intervals_tile(
                &edge_intervals,
                curve.domain().start(),
                curve.domain().end(),
            )? {
                continue;
            }
            let parameter_start = segments
                .iter()
                .map(|(_, start, _)| start.clone())
                .try_fold(None, |minimum, candidate| {
                    Ok::<_, BuildError>(Some(match minimum {
                        Some(minimum) => minimum_real(&minimum, &candidate)?,
                        None => candidate,
                    }))
                })?
                .expect("one or more path segments");
            let parameter_end = segments
                .iter()
                .map(|(_, _, end)| end.clone())
                .try_fold(None, |maximum, candidate| {
                    Ok::<_, BuildError>(Some(match maximum {
                        Some(maximum) => maximum_real(&maximum, &candidate)?,
                        None => candidate,
                    }))
                })?
                .expect("one or more path segments");
            let parameter_intervals = segments
                .iter()
                .map(|(_, start, end)| (start.clone(), end.clone()))
                .collect::<Vec<_>>();
            if !self.exact_intervals_tile(&parameter_intervals, &parameter_start, &parameter_end)? {
                continue;
            }

            let mut adjacency = HashMap::<VertexId, Vec<VertexId>>::new();
            for (edge_id, _, _) in &segments {
                let edge = self.edge_ref(*edge_id)?;
                adjacency.entry(edge.start).or_default().push(edge.end);
                adjacency.entry(edge.end).or_default().push(edge.start);
            }
            let lowers = adjacency
                .keys()
                .copied()
                .filter(|vertex| lower_vertices.contains(vertex))
                .collect::<Vec<_>>();
            let uppers = adjacency
                .keys()
                .copied()
                .filter(|vertex| upper_vertices.contains(vertex))
                .collect::<Vec<_>>();
            if lowers.len() != 1
                || uppers.len() != 1
                || adjacency.iter().any(|(vertex, adjacent)| {
                    let expected = if *vertex == lowers[0] || *vertex == uppers[0] {
                        1
                    } else {
                        2
                    };
                    adjacent.len() != expected
                })
            {
                continue;
            }
            let mut visited = HashSet::new();
            let mut pending = vec![lowers[0]];
            while let Some(vertex) = pending.pop() {
                if visited.insert(vertex) {
                    pending.extend(adjacency[&vertex].iter().copied());
                }
            }
            if visited.len() != adjacency.len() {
                continue;
            }
            let lower_point = self.vertex_ref(lowers[0])?.point();
            let upper_point = self.vertex_ref(uppers[0])?.point();
            let curve_start = curve.point_at(curve.domain().start())?;
            let curve_end = curve.point_at(curve.domain().end())?;
            let curve = if points_equal(&curve_start, lower_point)?
                && points_equal(&curve_end, upper_point)?
            {
                curve.clone()
            } else if points_equal(&curve_start, upper_point)?
                && points_equal(&curve_end, lower_point)?
            {
                curve.reversed()?
            } else {
                continue;
            };
            chains.push(TensorPathChain {
                lower: lowers[0],
                upper: uppers[0],
                curve,
                parameter_start,
                parameter_end,
            });
        }
        Ok(chains)
    }

    fn tensor_path_boundary_interval(
        &self,
        edge: EdgeId,
        shell_faces: &HashSet<FaceId>,
    ) -> Result<Option<(Real, Real)>, BuildError> {
        let mut interval = None;
        for edge_use in &self.edge_uses_by_edge[edge.index()] {
            let Some(wire) = self.edge_use_wire[edge_use.index()] else {
                continue;
            };
            let Some(face_id) = self.wire_face[wire.index()] else {
                continue;
            };
            if !shell_faces.contains(&face_id)
                || self.surface_ref(self.face_ref(face_id)?.surface)?.kind()
                    != SurfaceKind::RationalBezier
            {
                continue;
            }
            let edge_use = self.edge_use_ref(*edge_use)?;
            let Some(line) = self.pcurve_ref(edge_use.pcurve)?.line_segment() else {
                continue;
            };
            let on_path_boundary = real_values_equal(line.start().x(), line.end().x())?
                && (real_values_equal(line.start().x(), &Real::zero())?
                    || real_values_equal(line.start().x(), &Real::one())?);
            if !on_path_boundary {
                continue;
            }
            let order = decided_model_order(compare_reals(line.start().y(), line.end().y()))?;
            if order == std::cmp::Ordering::Equal {
                continue;
            }
            let candidate = if order == std::cmp::Ordering::Less {
                (line.start().y().clone(), line.end().y().clone())
            } else {
                (line.end().y().clone(), line.start().y().clone())
            };
            if let Some((start, end)) = &interval {
                if !real_values_equal(start, &candidate.0)?
                    || !real_values_equal(end, &candidate.1)?
                {
                    return Ok(None);
                }
            } else {
                interval = Some(candidate);
            }
        }
        Ok(interval)
    }

    fn restricted_curve_sweep_side_surface(
        &self,
        surface: SurfaceId,
        start: &Real,
        end: &Real,
    ) -> Result<Surface, BuildError> {
        let source = self.surface_ref(surface)?.clone();
        if real_values_equal(start, &Real::zero())? && real_values_equal(end, &Real::one())? {
            return Ok(source);
        }
        if decided_model_order(compare_reals(start, &Real::zero()))? == std::cmp::Ordering::Less
            || decided_model_order(compare_reals(end, &Real::one()))? == std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(start, end))? != std::cmp::Ordering::Less
        {
            return Err(GeometryError::InvalidParameterDomain.into());
        }
        let selected =
            if decided_model_order(compare_reals(end, &Real::one()))? == std::cmp::Ordering::Less {
                source.split_v_at(end)?.0
            } else {
                source
            };
        if decided_model_order(compare_reals(start, &Real::zero()))? == std::cmp::Ordering::Equal {
            return Ok(selected);
        }
        let relative = (start / end).map_err(|_| GeometryError::ProjectiveDivision)?;
        Ok(selected.split_v_at(&relative)?.1)
    }

    fn certifies_tensor_face_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        u_start: &Real,
        u_end: &Real,
        v_start: &Real,
        v_end: &Real,
    ) -> Result<bool, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(false);
        };
        if boundaries.len() != 1 {
            return Ok(false);
        }
        let mut sides = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
        for edge_use in &boundaries[0] {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let Some(line) = self.pcurve_ref(edge_use.pcurve)?.line_segment() else {
                return Ok(false);
            };
            let x_constant = real_values_equal(line.start().x(), line.end().x())?;
            let y_constant = real_values_equal(line.start().y(), line.end().y())?;
            let (side, first, second) =
                if x_constant && real_values_equal(line.start().x(), u_start)? {
                    (0, line.start().y(), line.end().y())
                } else if x_constant && real_values_equal(line.start().x(), u_end)? {
                    (1, line.start().y(), line.end().y())
                } else if y_constant && real_values_equal(line.start().y(), v_start)? {
                    (2, line.start().x(), line.end().x())
                } else if y_constant && real_values_equal(line.start().y(), v_end)? {
                    (3, line.start().x(), line.end().x())
                } else {
                    return Ok(false);
                };
            let order = decided_model_order(compare_reals(first, second))?;
            if order == std::cmp::Ordering::Equal {
                return Ok(false);
            }
            sides[side].push(if order == std::cmp::Ordering::Less {
                (first.clone(), second.clone())
            } else {
                (second.clone(), first.clone())
            });
        }
        for (intervals, start, end) in [
            (&sides[0], v_start, v_end),
            (&sides[1], v_start, v_end),
            (&sides[2], u_start, u_end),
            (&sides[3], u_start, u_end),
        ] {
            if !self.exact_intervals_tile(intervals, start, end)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn exact_intervals_tile(
        &self,
        intervals: &[(Real, Real)],
        start: &Real,
        end: &Real,
    ) -> Result<bool, BuildError> {
        if intervals.is_empty() {
            return Ok(false);
        }
        let mut used = vec![false; intervals.len()];
        let mut current = start.clone();
        for _ in 0..intervals.len() {
            let mut next = None;
            for (index, (start, end)) in intervals.iter().enumerate() {
                if used[index] || !real_values_equal(start, &current)? {
                    continue;
                }
                if next.replace((index, end)).is_some() {
                    return Ok(false);
                }
            }
            let Some((index, end)) = next else {
                return Ok(false);
            };
            used[index] = true;
            current = end.clone();
        }
        real_values_equal(&current, end)
    }

    fn certified_curve_sweep_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        if lower_edges.iter().any(|edge_id| {
            self.curves[self.edges[edge_id.index()].curve.index()].kind() != Curve3Kind::Line
        }) || upper_edges.iter().any(|edge_id| {
            self.curves[self.edges[edge_id.index()].curve.index()].kind() != Curve3Kind::Line
        }) {
            return Ok(None);
        }
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                ((lower_vertices.contains(&edge.start) && upper_vertices.contains(&edge.end))
                    || (lower_vertices.contains(&edge.end) && upper_vertices.contains(&edge.start)))
                .then_some(*edge_id)
            })
            .collect::<HashSet<_>>();
        if cross_edges.len() != count
            || shell_edges.len() != count * 3
            || self.shell_ref(shell)?.faces.len() != count + 2
        {
            return Ok(None);
        }

        let mut correspondence = HashMap::with_capacity(count);
        let mut connector_by_lower = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = self.edge_ref(*edge_id)?;
            let (lower, upper, curve) = if lower_vertices.contains(&edge.start) {
                (edge.start, edge.end, self.curve_ref(edge.curve)?.clone())
            } else {
                (
                    edge.end,
                    edge.start,
                    self.curve_ref(edge.curve)?.reversed()?,
                )
            };
            if curve.kind() != Curve3Kind::RationalBezier
                || correspondence.insert(lower, upper).is_some()
                || connector_by_lower.insert(lower, curve).is_some()
            {
                return Ok(None);
            }
        }
        if correspondence.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }
        for lower_edge_id in &lower_edges {
            let lower_edge = self.edge_ref(*lower_edge_id)?;
            let mapped_start = correspondence[&lower_edge.start];
            let mapped_end = correspondence[&lower_edge.end];
            if !upper_edges.iter().any(|upper_edge_id| {
                let upper_edge = &self.edges[upper_edge_id.index()];
                (upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                    || (upper_edge.start == mapped_end && upper_edge.end == mapped_start)
            }) {
                return Ok(None);
            }
        }

        let reference = *lower_vertices
            .iter()
            .min()
            .expect("curve sweep cap has vertices");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let path = connector_by_lower[&reference].clone();
        if !real_values_equal(path.domain().start(), &Real::zero())?
            || !real_values_equal(path.domain().end(), &Real::one())?
            || !certified_affine_sweep_progress(&path, &lower_u.cross(&lower_v))?
        {
            return Ok(None);
        }
        let Curve3ExactData::RationalBezier {
            control_points: path_controls,
            weights: path_weights,
        } = path.exact_data()
        else {
            unreachable!("connector kind was checked");
        };
        for lower in &lower_vertices {
            let offset = self.vertex_ref(*lower)?.point() - &reference_point;
            let Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } = connector_by_lower[lower].exact_data()
            else {
                unreachable!("connector kind was checked");
            };
            if !translated_rational_bezier_data_equal(
                &control_points,
                &weights,
                &path_controls,
                &path_weights,
                &offset,
            )? {
                return Ok(None);
            }
        }

        let mut represented_lower_edges = HashSet::with_capacity(count);
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            if face.orientation != Orientation::Forward
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::RationalBezier
            {
                return Ok(None);
            }
            let Some(wire_id) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire_id)?;
            let lower_hits = edges
                .intersection(&lower_edges)
                .copied()
                .collect::<Vec<_>>();
            let upper_hits = edges
                .intersection(&upper_edges)
                .copied()
                .collect::<Vec<_>>();
            let cross_hits = edges
                .intersection(&cross_edges)
                .copied()
                .collect::<Vec<_>>();
            if edges.len() != 4
                || lower_hits.len() != 1
                || upper_hits.len() != 1
                || cross_hits.len() != 2
                || !represented_lower_edges.insert(lower_hits[0])
            {
                return Ok(None);
            }
            let lower_edge = self.edge_ref(lower_hits[0])?;
            let mapped_start = correspondence[&lower_edge.start];
            let mapped_end = correspondence[&lower_edge.end];
            let upper_edge = self.edge_ref(upper_hits[0])?;
            if !((upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                || (upper_edge.start == mapped_end && upper_edge.end == mapped_start))
            {
                return Ok(None);
            }
            for (lower, upper) in [
                (lower_edge.start, mapped_start),
                (lower_edge.end, mapped_end),
            ] {
                if !cross_hits.iter().any(|edge_id| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == lower && edge.end == upper)
                        || (edge.start == upper && edge.end == lower)
                }) {
                    return Ok(None);
                }
            }
            let start_offset = self.vertex_ref(lower_edge.start)?.point() - &reference_point;
            let end_offset = self.vertex_ref(lower_edge.end)?.point() - &reference_point;
            let SurfaceExactData::RationalBezier {
                control_points,
                weights,
            } = self.surface_ref(face.surface)?.exact_data()
            else {
                unreachable!("side surface kind was checked");
            };
            if !translated_rational_bezier_surface_equal(
                &control_points,
                &weights,
                &path_controls,
                &path_weights,
                &start_offset,
                &end_offset,
            )? {
                return Ok(None);
            }
        }
        if represented_lower_edges != lower_edges {
            return Ok(None);
        }

        let mut ordered = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        ordered.reverse();
        let reference_position = ordered
            .iter()
            .position(|vertex| *vertex == reference)
            .expect("lower cap order contains every vertex");
        ordered.rotate_left(reference_position);
        let profile_points = ordered
            .iter()
            .map(|vertex| {
                project_point_to_plane_frame(
                    self.vertex_ref(*vertex)?.point(),
                    &reference_point,
                    &lower_u,
                    &lower_v,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let profile = Contour2::try_new(
            (0..profile_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        profile_points[index].clone(),
                        profile_points[(index + 1) % profile_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                })
                .collect::<Result<Vec<_>, _>>()?,
        )
        .map_err(GeometryError::from)?;
        let area = profile
            .signed_area()
            .map_err(GeometryError::from)?
            .ok_or(BuildError::DegenerateShellVolume(shell))?;
        if decided_model_order(compare_reals(&area, &Real::zero()))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedCurveSweepShell {
            profile,
            holes: Vec::new(),
            path,
            u_path: constant_vector_curve(&lower_u, &path_weights)?,
            v_path: constant_vector_curve(&lower_v, &path_weights)?,
            area_scale_integral: Real::one(),
        }))
    }

    fn certified_loft_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 5 {
            return Ok(None);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !matches!(
                self.surface_ref(face.surface)?.kind(),
                SurfaceKind::Plane | SurfaceKind::RationalBezier
            ) || !face.inner().is_empty()
            {
                return Ok(None);
            }
            for wire_id in face.boundary_wires() {
                for use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*use_id)?.edge)?;
                    if self.curve_ref(edge.curve)?.kind() != Curve3Kind::Line {
                        return Ok(None);
                    }
                }
            }
        }
        for lower in faces {
            if self.face_ref(*lower)?.orientation != Orientation::Reversed {
                continue;
            }
            for upper in faces {
                if self.face_ref(*upper)?.orientation != Orientation::Forward {
                    continue;
                }
                if let Some(certificate) = self.certified_loft_cap_pair(shell, *lower, *upper)? {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    fn certified_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if let Some(certificate) =
            self.certified_two_section_loft_cap_pair(shell, lower_face_id, upper_face_id)?
        {
            return Ok(Some(certificate));
        }
        self.certified_multi_section_loft_cap_pair(shell, lower_face_id, upper_face_id)
    }

    fn certified_multi_section_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let shell_vertices = self.shell_vertex_set(shell)?;
        let reference = *lower_vertices.iter().min().expect("loft cap has vertices");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let upper_height_reference =
            self.vertex_ref(*upper_vertices.iter().min().expect("loft cap has vertices"))?;
        let normal = lower_u.cross(&lower_v);
        let height_denominator = normal.dot(&(upper_height_reference.point() - &reference_point));
        if decided_model_order(compare_reals(&height_denominator, &Real::zero()))?
            == std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let mut sorted_vertices = shell_vertices.iter().copied().collect::<Vec<_>>();
        sorted_vertices.sort_by_key(|vertex| vertex.index());
        let mut layers = Vec::<(Real, Vec<VertexId>)>::new();
        for vertex in sorted_vertices {
            let height = (normal.dot(&(self.vertex_ref(vertex)?.point() - &reference_point))
                / &height_denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            if decided_model_order(compare_reals(&height, &Real::zero()))?
                == std::cmp::Ordering::Less
                || decided_model_order(compare_reals(&height, &Real::one()))?
                    == std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            let mut inserted = false;
            for index in 0..layers.len() {
                match decided_model_order(compare_reals(&height, &layers[index].0))? {
                    std::cmp::Ordering::Equal => {
                        layers[index].1.push(vertex);
                        inserted = true;
                        break;
                    }
                    std::cmp::Ordering::Less => {
                        layers.insert(index, (height.clone(), vec![vertex]));
                        inserted = true;
                        break;
                    }
                    std::cmp::Ordering::Greater => {}
                }
            }
            if !inserted {
                layers.push((height, vec![vertex]));
            }
        }
        if layers.len() < 3
            || layers.iter().any(|(_, vertices)| vertices.len() != count)
            || decided_model_order(compare_reals(&layers[0].0, &Real::zero()))?
                != std::cmp::Ordering::Equal
            || decided_model_order(compare_reals(
                &layers.last().expect("loft has layers").0,
                &Real::one(),
            ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let mut vertex_layer = HashMap::with_capacity(shell_vertices.len());
        for (layer, (_, vertices)) in layers.iter().enumerate() {
            for vertex in vertices {
                vertex_layer.insert(*vertex, layer);
            }
        }
        let mut ring_edges = vec![HashSet::new(); layers.len()];
        let mut connector_edges = vec![HashSet::new(); layers.len() - 1];
        for edge_id in &shell_edges {
            let edge = self.edge_ref(*edge_id)?;
            let first = vertex_layer[&edge.start];
            let second = vertex_layer[&edge.end];
            if first == second {
                ring_edges[first].insert(*edge_id);
            } else if first.abs_diff(second) == 1 {
                connector_edges[first.min(second)].insert(*edge_id);
            } else {
                return Ok(None);
            }
        }
        if ring_edges.iter().any(|edges| edges.len() != count)
            || connector_edges.iter().any(|edges| edges.len() != count)
            || ring_edges[0] != lower_edges
            || ring_edges[ring_edges.len() - 1] != upper_edges
            || self.shell_ref(shell)?.faces.len() != 2 + count * (layers.len() - 1)
        {
            return Ok(None);
        }

        let mut first_order = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        first_order.reverse();
        let reference_position = first_order
            .iter()
            .position(|vertex| *vertex == reference)
            .expect("lower cap order contains every lower vertex");
        first_order.rotate_left(reference_position);
        let mut ordered_layers = vec![first_order];
        let mut correspondences = Vec::with_capacity(layers.len() - 1);
        for span in 0..layers.len() - 1 {
            let mut correspondence = HashMap::with_capacity(count);
            for edge_id in &connector_edges[span] {
                let edge = self.edge_ref(*edge_id)?;
                let (lower, upper) = if vertex_layer[&edge.start] == span {
                    (edge.start, edge.end)
                } else {
                    (edge.end, edge.start)
                };
                if vertex_layer[&upper] != span + 1 || correspondence.insert(lower, upper).is_some()
                {
                    return Ok(None);
                }
            }
            if correspondence.len() != count
                || correspondence
                    .values()
                    .copied()
                    .collect::<HashSet<_>>()
                    .len()
                    != count
            {
                return Ok(None);
            }
            let Some(next_order) = ordered_layers[span]
                .iter()
                .map(|vertex| correspondence.get(vertex).copied())
                .collect::<Option<Vec<_>>>()
            else {
                return Ok(None);
            };
            if next_order.iter().copied().collect::<HashSet<_>>()
                != layers[span + 1].1.iter().copied().collect::<HashSet<_>>()
            {
                return Ok(None);
            }
            for index in 0..count {
                let first = next_order[index];
                let second = next_order[(index + 1) % count];
                if !ring_edges[span + 1].iter().any(|edge_id| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == first && edge.end == second)
                        || (edge.start == second && edge.end == first)
                }) {
                    return Ok(None);
                }
            }
            ordered_layers.push(next_order);
            correspondences.push(correspondence);
        }
        if ordered_layers
            .last()
            .expect("loft has an upper layer")
            .iter()
            .copied()
            .collect::<HashSet<_>>()
            != upper_vertices
        {
            return Ok(None);
        }

        let mut represented_lower_edges = vec![HashSet::new(); layers.len() - 1];
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            let Some(wire) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire)?;
            if edges.len() != 4 {
                return Ok(None);
            }
            let ring_hits = ring_edges
                .iter()
                .enumerate()
                .flat_map(|(layer, ring)| {
                    edges
                        .intersection(ring)
                        .copied()
                        .map(move |edge| (layer, edge))
                })
                .collect::<Vec<_>>();
            let connector_hits = connector_edges
                .iter()
                .enumerate()
                .flat_map(|(span, connectors)| {
                    edges
                        .intersection(connectors)
                        .copied()
                        .map(move |edge| (span, edge))
                })
                .collect::<Vec<_>>();
            if ring_hits.len() != 2
                || connector_hits.len() != 2
                || ring_hits[1].0 != ring_hits[0].0 + 1
                || connector_hits
                    .iter()
                    .any(|(span, _)| *span != ring_hits[0].0)
            {
                return Ok(None);
            }
            let span = ring_hits[0].0;
            let lower_edge_id = ring_hits[0].1;
            let upper_edge_id = ring_hits[1].1;
            if !represented_lower_edges[span].insert(lower_edge_id) {
                return Ok(None);
            }
            let lower_edge = self.edge_ref(lower_edge_id)?;
            let mapped_start = correspondences[span][&lower_edge.start];
            let mapped_end = correspondences[span][&lower_edge.end];
            let upper_edge = self.edge_ref(upper_edge_id)?;
            if !((upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                || (upper_edge.start == mapped_end && upper_edge.end == mapped_start))
            {
                return Ok(None);
            }
            for (lower, upper) in [
                (lower_edge.start, mapped_start),
                (lower_edge.end, mapped_end),
            ] {
                if !connector_hits.iter().any(|(_, edge_id)| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == lower && edge.end == upper)
                        || (edge.start == upper && edge.end == lower)
                }) {
                    return Ok(None);
                }
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::RationalBezier => {
                    let expected = [
                        self.vertex_ref(lower_edge.start)?.point(),
                        self.vertex_ref(lower_edge.end)?.point(),
                        self.vertex_ref(mapped_start)?.point(),
                        self.vertex_ref(mapped_end)?.point(),
                    ];
                    if !self.certified_bilinear_loft_side(face.surface, expected)? {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        if represented_lower_edges
            .iter()
            .enumerate()
            .any(|(span, represented)| *represented != ring_edges[span])
        {
            return Ok(None);
        }

        let mapped_reference = ordered_layers.last().expect("loft has an upper layer")[0];
        let height_axis = self.vertex_ref(mapped_reference)?.point() - &reference_point;
        let uu = lower_u.dot(&lower_u);
        let uv = lower_u.dot(&lower_v);
        let vv = lower_v.dot(&lower_v);
        let determinant = &uu * &vv - &uv * &uv;
        let project = |point: &Point3, origin: &Point3| -> Result<CurvePoint2, BuildError> {
            let displacement = point - origin;
            let du = displacement.dot(&lower_u);
            let dv = displacement.dot(&lower_v);
            Ok(CurvePoint2::new(
                ((&du * &vv - &dv * &uv) / &determinant)
                    .map_err(|_| GeometryError::ProjectiveDivision)?,
                ((&dv * &uu - &du * &uv) / &determinant)
                    .map_err(|_| GeometryError::ProjectiveDivision)?,
            ))
        };
        let mut profiles = Vec::with_capacity(layers.len());
        let mut contours = Vec::with_capacity(layers.len());
        for (layer, ordered) in ordered_layers.iter().enumerate() {
            let baseline = reference_point.clone() + height_axis.clone() * &layers[layer].0;
            let points = ordered
                .iter()
                .map(|vertex| project(self.vertex_ref(*vertex)?.point(), &baseline))
                .collect::<Result<Vec<_>, _>>()?;
            let contour = Contour2::try_new(
                (0..points.len())
                    .map(|index| {
                        LineSeg2::try_new(
                            points[index].clone(),
                            points[(index + 1) % points.len()].clone(),
                        )
                        .map(Segment2::Line)
                        .map_err(GeometryError::from)
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            )
            .map_err(GeometryError::from)?;
            let area = contour
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?;
            if decided_model_order(compare_reals(&area, &Real::zero()))?
                != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            profiles.push(points);
            contours.push(contour);
        }
        let mut spans = Vec::with_capacity(layers.len() - 1);
        let mut parameter_volume = Real::zero();
        for span in 0..layers.len() - 1 {
            let lower_points = &profiles[span];
            let upper_points = &profiles[span + 1];
            let interpolation =
                if let Some(scale) = certified_homothetic_loft_scale(lower_points, upper_points)? {
                    let translation = CurvePoint2::new(
                        upper_points[0].x() - &scale * lower_points[0].x(),
                        upper_points[0].y() - &scale * lower_points[0].y(),
                    );
                    CertifiedLoftInterpolation::Homothetic {
                        profile: contours[span].clone(),
                        scale,
                        translation,
                    }
                } else {
                    if !certified_convex_loft_interpolation(lower_points, upper_points)? {
                        return Ok(None);
                    }
                    CertifiedLoftInterpolation::ConvexCorresponding {
                        lower: lower_points.clone(),
                        upper: upper_points.clone(),
                    }
                };
            let area_integral = loft_parameter_area_integral(lower_points, upper_points)?;
            let width = &layers[span + 1].0 - &layers[span].0;
            parameter_volume += &width * area_integral;
            spans.push(CertifiedLoftSpan {
                start: layers[span].0.clone(),
                end: layers[span + 1].0.clone(),
                interpolation,
            });
        }
        if decided_model_order(compare_reals(&parameter_volume, &Real::zero()))?
            != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedLoftShell {
            origin: reference_point,
            u: lower_u,
            v: lower_v,
            height_axis,
            spans,
            parameter_volume,
        }))
    }

    fn certified_two_section_loft_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedLoftShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                ((lower_vertices.contains(&edge.start) && upper_vertices.contains(&edge.end))
                    || (lower_vertices.contains(&edge.end) && upper_vertices.contains(&edge.start)))
                .then_some(*edge_id)
            })
            .collect::<HashSet<_>>();
        if cross_edges.len() != count
            || shell_edges.len() != lower_edges.len() + upper_edges.len() + cross_edges.len()
            || self.shell_ref(shell)?.faces.len() != count + 2
        {
            return Ok(None);
        }
        let mut correspondence = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = self.edge_ref(*edge_id)?;
            let (lower, upper) = if lower_vertices.contains(&edge.start) {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if correspondence.insert(lower, upper).is_some() {
                return Ok(None);
            }
        }
        if correspondence.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            let Some(wire) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire)?;
            if edges.len() != 4
                || edges.intersection(&lower_edges).count() != 1
                || edges.intersection(&upper_edges).count() != 1
                || edges.intersection(&cross_edges).count() != 2
            {
                return Ok(None);
            }
        }

        let reference = *lower_vertices.iter().min().expect("loft cap has vertices");
        let mapped_reference = correspondence[&reference];
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let mapped_reference_point = self.vertex_ref(mapped_reference)?.point().clone();
        let extrusion = &mapped_reference_point - &reference_point;
        for edge_id in &lower_edges {
            let edge = self.edge_ref(*edge_id)?;
            let mapped_start = correspondence[&edge.start];
            let mapped_end = correspondence[&edge.end];
            if !upper_edges.iter().any(|candidate| {
                let candidate = &self.edges[candidate.index()];
                (candidate.start == mapped_start && candidate.end == mapped_end)
                    || (candidate.start == mapped_end && candidate.end == mapped_start)
            }) {
                return Ok(None);
            }
        }

        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::RationalBezier => {
                    let wire = self.wire_ref(face.outer().expect("loft side has an outer wire"))?;
                    let lower_edge_id = wire
                        .edge_uses
                        .iter()
                        .map(|use_id| self.edge_use_ref(*use_id).map(|edge_use| edge_use.edge))
                        .collect::<Result<Vec<_>, _>>()?
                        .into_iter()
                        .find(|edge| lower_edges.contains(edge))
                        .expect("certified loft side has one lower edge");
                    let lower_edge = self.edge_ref(lower_edge_id)?;
                    let expected = [
                        self.vertex_ref(lower_edge.start)?.point(),
                        self.vertex_ref(lower_edge.end)?.point(),
                        self.vertex_ref(correspondence[&lower_edge.start])?.point(),
                        self.vertex_ref(correspondence[&lower_edge.end])?.point(),
                    ];
                    if !self.certified_bilinear_loft_side(face.surface, expected)? {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }

        let uu = lower_u.dot(&lower_u);
        let uv = lower_u.dot(&lower_v);
        let vv = lower_v.dot(&lower_v);
        let determinant = &uu * &vv - &uv * &uv;
        let project = |point: &Point3, origin: &Point3| -> Result<CurvePoint2, BuildError> {
            let displacement = point - origin;
            let du = displacement.dot(&lower_u);
            let dv = displacement.dot(&lower_v);
            Ok(CurvePoint2::new(
                ((&du * &vv - &dv * &uv) / &determinant)
                    .map_err(|_| GeometryError::ProjectiveDivision)?,
                ((&dv * &uu - &du * &uv) / &determinant)
                    .map_err(|_| GeometryError::ProjectiveDivision)?,
            ))
        };
        let mut ordered_vertices = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        ordered_vertices.reverse();
        let lower_points = ordered_vertices
            .iter()
            .map(|vertex| project(self.vertex_ref(*vertex)?.point(), &reference_point))
            .collect::<Result<Vec<_>, _>>()?;
        let upper_points = ordered_vertices
            .iter()
            .map(|vertex| {
                project(
                    self.vertex_ref(correspondence[vertex])?.point(),
                    &mapped_reference_point,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let profile = Contour2::try_new(
            (0..lower_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        lower_points[index].clone(),
                        lower_points[(index + 1) % lower_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                })
                .collect::<Result<Vec<_>, _>>()?,
        )
        .map_err(GeometryError::from)?;
        let area = profile
            .signed_area()
            .map_err(GeometryError::from)?
            .ok_or(BuildError::DegenerateShellVolume(shell))?;
        if decided_model_order(compare_reals(&area, &Real::zero()))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let interpolation =
            if let Some(scale) = certified_homothetic_loft_scale(&lower_points, &upper_points)? {
                CertifiedLoftInterpolation::Homothetic {
                    profile,
                    scale,
                    translation: CurvePoint2::new(Real::zero(), Real::zero()),
                }
            } else {
                if !certified_convex_loft_interpolation(&lower_points, &upper_points)? {
                    return Ok(None);
                }
                CertifiedLoftInterpolation::ConvexCorresponding {
                    lower: lower_points.clone(),
                    upper: upper_points.clone(),
                }
            };
        let parameter_area_integral = loft_parameter_area_integral(&lower_points, &upper_points)?;
        if decided_model_order(compare_reals(&parameter_area_integral, &Real::zero()))?
            != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedLoftShell {
            origin: reference_point,
            u: lower_u,
            v: lower_v,
            height_axis: extrusion,
            spans: vec![CertifiedLoftSpan {
                start: Real::zero(),
                end: Real::one(),
                interpolation,
            }],
            parameter_volume: parameter_area_integral,
        }))
    }

    fn certified_bilinear_loft_side(
        &self,
        surface_id: SurfaceId,
        expected: [&Point3; 4],
    ) -> Result<bool, BuildError> {
        let SurfaceExactData::RationalBezier {
            control_points,
            weights,
        } = self.surface_ref(surface_id)?.exact_data()
        else {
            return Ok(false);
        };
        if control_points.len() != 2
            || control_points.iter().any(|row| row.len() != 2)
            || weights.len() != 2
            || weights.iter().any(|row| row.len() != 2)
        {
            return Ok(false);
        }
        let reference_weight = &weights[0][0];
        for weight in weights.iter().flatten().skip(1) {
            if decided_model_order(compare_reals(weight, reference_weight))?
                != std::cmp::Ordering::Equal
            {
                return Ok(false);
            }
        }
        let controls = [
            &control_points[0][0],
            &control_points[0][1],
            &control_points[1][0],
            &control_points[1][1],
        ];
        const SYMMETRIES: [[usize; 4]; 8] = [
            [0, 1, 2, 3],
            [1, 0, 3, 2],
            [2, 3, 0, 1],
            [3, 2, 1, 0],
            [0, 2, 1, 3],
            [2, 0, 3, 1],
            [1, 3, 0, 2],
            [3, 1, 2, 0],
        ];
        for symmetry in SYMMETRIES {
            let mut matches = true;
            for index in 0..4 {
                if !points_equal(controls[index], expected[symmetry[index]])? {
                    matches = false;
                    break;
                }
            }
            if matches {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn certified_revolution_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        self.certified_oriented_revolution_shell(shell, Orientation::Forward)
    }

    fn certified_revolution_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        let Some(mut certificate) =
            self.certified_oriented_revolution_shell(solid.outer, Orientation::Forward)?
        else {
            return Ok(None);
        };
        for shell in &solid.voids {
            let Some(void) =
                self.certified_oriented_revolution_shell(*shell, Orientation::Reversed)?
            else {
                return Ok(None);
            };
            if !points_equal(&certificate.axis_origin, &void.axis_origin)?
                || !vectors_equal(&certificate.axis, &void.axis)?
            {
                return Ok(None);
            }
            certificate.voids.push(void.profile);
        }
        Ok(Some(certificate))
    }

    fn certified_oriented_revolution_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<CertifiedRevolutionShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 8 {
            return Ok(None);
        }
        let mut groups: HashMap<SurfaceId, Vec<FaceId>> = HashMap::new();
        let mut planar_faces = 0;
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Revolution
                    if face.orientation == orientation && face.inner().is_empty() =>
                {
                    groups.entry(face.surface).or_default().push(*face_id);
                }
                SurfaceKind::Plane => planar_faces += 1,
                _ => return Ok(None),
            }
        }
        if groups.is_empty() || groups.values().any(|group| group.len() < 4) {
            return Ok(None);
        }

        let quarter =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        let mut axis_origin = None;
        let mut axis = None;
        let mut meridian_ray: Option<Vector3> = None;
        let mut angular_origin: Option<Real> = None;
        let mut profile_curves = Vec::with_capacity(groups.len() + planar_faces);
        for (surface_id, group) in groups {
            let SurfaceExactData::Revolution {
                profile,
                axis_origin: this_origin,
                axis: this_axis,
            } = self.surface_ref(surface_id)?.exact_data()
            else {
                unreachable!("revolution kind carries revolution data");
            };
            if let Some(expected) = &axis_origin
                && !points_equal(expected, &this_origin)?
            {
                return Ok(None);
            }
            if let Some(expected) = &axis
                && !vectors_equal(expected, &this_axis)?
            {
                return Ok(None);
            }
            axis_origin.get_or_insert_with(|| this_origin.clone());
            axis.get_or_insert_with(|| this_axis.clone());
            let profile_curve = Curve3::from_exact_data(*profile)?;
            let mut to_profile_point = |point: &Point3,
                                        require_positive: bool|
             -> Result<Option<CurvePoint2>, BuildError> {
                let relative = point - &this_origin;
                let axial = this_axis.dot(&relative);
                let radial = relative - this_axis.clone() * &axial;
                let radial_squared = radial.norm_squared();
                let radial_order =
                    decided_model_order(compare_reals(&radial_squared, &Real::zero()))?;
                if let Some(expected) = &meridian_ray {
                    if decided_model_order(compare_reals(
                        &expected.cross(&radial).norm_squared(),
                        &Real::zero(),
                    ))? != std::cmp::Ordering::Equal
                    {
                        return Ok(None);
                    }
                    let radius = expected.dot(&radial);
                    if require_positive
                        && decided_model_order(compare_reals(&radius, &Real::zero()))?
                            != std::cmp::Ordering::Greater
                    {
                        return Ok(None);
                    }
                    return Ok(Some(CurvePoint2::new(radius, axial)));
                }
                if radial_order != std::cmp::Ordering::Greater {
                    return if require_positive {
                        Ok(None)
                    } else {
                        Ok(Some(CurvePoint2::new(Real::zero(), axial)))
                    };
                }
                let radius = radial_squared
                    .sqrt()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                if require_positive {
                    meridian_ray =
                        Some((radial / &radius).map_err(|_| GeometryError::ProjectiveDivision)?);
                } else {
                    return Ok(None);
                }
                Ok(Some(CurvePoint2::new(radius, axial)))
            };
            let mut u_values = Vec::new();
            let mut v_values = Vec::new();
            let mut face_coordinates = Vec::with_capacity(group.len());
            for face_id in group {
                let outer = self
                    .face_ref(face_id)?
                    .outer()
                    .expect("trimmed revolution face");
                let wire = self.wire_ref(outer)?;
                if wire.edge_uses.len() < 4 {
                    return Ok(None);
                }
                let mut face_u = Vec::with_capacity(wire.edge_uses.len() * 2);
                let mut face_v = Vec::with_capacity(wire.edge_uses.len() * 2);
                let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
                for use_id in &self.wire_ref(outer)?.edge_uses {
                    let pcurve = self.pcurve_ref(self.edge_use_ref(*use_id)?.pcurve)?;
                    let Some(line) = pcurve.line_segment() else {
                        return Ok(None);
                    };
                    let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                    let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                    if u_constant == v_constant {
                        return Ok(None);
                    }
                    face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                    face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                    parameter_segments.push(Segment2::Line(line.clone()));
                }
                let (u_min, u_max) = exact_real_min_max(&face_u)?;
                let (v_min, v_max) = exact_real_min_max(&face_v)?;
                let contour = Contour2::try_new(parameter_segments).map_err(GeometryError::from)?;
                let represented_area = contour
                    .signed_area()
                    .map_err(GeometryError::from)?
                    .ok_or(BuildError::DegenerateShellVolume(shell))?
                    .abs();
                let rectangle_area = (&u_max - &u_min) * (&v_max - &v_min);
                if !real_values_equal(&represented_area, &rectangle_area)? {
                    return Ok(None);
                }
                insert_sorted_real(&mut u_values, &u_min)?;
                insert_sorted_real(&mut u_values, &u_max)?;
                insert_sorted_real(&mut v_values, &v_min)?;
                insert_sorted_real(&mut v_values, &v_max)?;
                face_coordinates.push((u_min, u_max, v_min, v_max));
            }
            if u_values.len() != 5 || v_values.len() < 2 {
                return Ok(None);
            }
            for pair in u_values.windows(2) {
                if !real_values_equal(&(&pair[1] - &pair[0]), &quarter)? {
                    return Ok(None);
                }
            }
            let mut covered_cells = HashSet::new();
            for (u_min, u_max, v_min, v_max) in face_coordinates {
                let u_start = exact_real_index(&u_values, &u_min)?;
                let u_end = exact_real_index(&u_values, &u_max)?;
                let v_start = exact_real_index(&v_values, &v_min)?;
                let v_end = exact_real_index(&v_values, &v_max)?;
                if u_end != u_start + 1 || v_start >= v_end {
                    return Ok(None);
                }
                for v_cell in v_start..v_end {
                    if !covered_cells.insert((u_start, v_cell)) {
                        return Ok(None);
                    }
                }
            }
            if covered_cells.len() != (u_values.len() - 1) * (v_values.len() - 1) {
                return Ok(None);
            }
            if !real_values_equal(
                &(u_values[3].clone() + &quarter - &u_values[0]),
                &(Real::from(2) * Real::pi()),
            )? {
                return Ok(None);
            }
            if let Some(expected) = &angular_origin {
                if !real_values_equal(expected, &u_values[0])? {
                    return Ok(None);
                }
            } else {
                angular_origin = Some(u_values[0].clone());
            }

            let represented_profile = profile_curve.subcurve(
                &v_values[0],
                v_values.last().expect("revolution group has v values"),
            )?;
            let start_point = represented_profile.point_at(represented_profile.domain().start())?;
            let end_point = represented_profile.point_at(represented_profile.domain().end())?;
            let Some(start) = to_profile_point(&start_point, true)? else {
                return Ok(None);
            };
            let Some(end) = to_profile_point(&end_point, true)? else {
                return Ok(None);
            };
            let profile_curve = match represented_profile.exact_data() {
                Curve3ExactData::Line(_) => {
                    Curve2::from(LineSeg2::try_new(start, end).map_err(GeometryError::from)?)
                }
                Curve3ExactData::RationalBezier {
                    control_points,
                    weights,
                } => {
                    let controls = control_points
                        .iter()
                        .map(|point| to_profile_point(point, false))
                        .collect::<Result<Option<Vec<_>>, _>>()?;
                    let Some(controls) = controls else {
                        return Ok(None);
                    };
                    planar_rational_bezier_curve(controls, weights.clone())?
                }
                Curve3ExactData::Nurbs {
                    degree,
                    control_points,
                    weights,
                    knots,
                } => {
                    let controls = control_points
                        .iter()
                        .map(|point| to_profile_point(point, false))
                        .collect::<Result<Option<Vec<_>>, _>>()?;
                    let Some(controls) = controls else {
                        return Ok(None);
                    };
                    if weights.iter().all(|weight| weight == &weights[0]) {
                        Curve2::try_polynomial_bspline(degree, controls, knots.clone())
                            .map_err(GeometryError::from)?
                    } else {
                        Curve2::try_nurbs(degree, controls, weights.clone(), knots.clone())
                            .map_err(GeometryError::from)?
                    }
                }
                Curve3ExactData::EllipseArc(data) if data.circle => {
                    let Some(center) = to_profile_point(&data.center, false)? else {
                        return Ok(None);
                    };
                    let ray = meridian_ray
                        .as_ref()
                        .expect("positive profile endpoint establishes a meridian ray");
                    let meridian_normal = this_axis.cross(ray);
                    if decided_model_order(compare_reals(
                        &data.x.dot(&meridian_normal),
                        &Real::zero(),
                    ))? != std::cmp::Ordering::Equal
                        || decided_model_order(compare_reals(
                            &data.y.dot(&meridian_normal),
                            &Real::zero(),
                        ))? != std::cmp::Ordering::Equal
                    {
                        return Ok(None);
                    }
                    let tangent = represented_profile
                        .derivative_at(represented_profile.domain().start(), 1)?
                        .vector()
                        .clone();
                    let tangent_radial = ray.dot(&tangent);
                    let tangent_axial = this_axis.dot(&tangent);
                    let turn = (start.x() - center.x()) * &tangent_axial
                        - (start.y() - center.y()) * &tangent_radial;
                    let clockwise = match decided_model_order(compare_reals(&turn, &Real::zero()))?
                    {
                        std::cmp::Ordering::Less => true,
                        std::cmp::Ordering::Greater => false,
                        std::cmp::Ordering::Equal => return Ok(None),
                    };
                    let arc = CircularArc2::try_from_center(start, end, center, clockwise)
                        .map_err(GeometryError::from)?;
                    let sweep = match arc.directed_sweep_angle().map_err(GeometryError::from)? {
                        Classification::Decided(sweep) => sweep,
                        Classification::Uncertain(reason) => {
                            return Err(BuildError::Geometry(
                                GeometryError::PlanarClassificationUnresolved(reason),
                            ));
                        }
                    };
                    if !real_values_equal(
                        &sweep,
                        &(represented_profile.domain().end()
                            - represented_profile.domain().start()),
                    )? {
                        return Ok(None);
                    }
                    Curve2::from(arc)
                }
                _ => return Ok(None),
            };
            profile_curves.push(profile_curve);
        }
        if planar_faces > 0 {
            if orientation != Orientation::Forward {
                return Ok(None);
            }
            let Some(planar_segments) = self.certified_revolution_planar_profile_segments(
                faces,
                axis_origin
                    .as_ref()
                    .expect("revolution side faces establish an axis origin"),
                axis.as_ref()
                    .expect("revolution side faces establish an axis"),
            )?
            else {
                return Ok(None);
            };
            profile_curves.extend(planar_segments.into_iter().map(curve2_from_segment));
        }

        let mut ordered = Vec::with_capacity(profile_curves.len());
        ordered.push(profile_curves.remove(0));
        while !profile_curves.is_empty() {
            let end = ordered.last().expect("seeded profile").end();
            let mut matching = None;
            for (index, curve) in profile_curves.iter().enumerate() {
                if curve_points_equal(curve.start(), end)? && matching.replace(index).is_some() {
                    return Ok(None);
                }
            }
            let Some(index) = matching else {
                return Ok(None);
            };
            ordered.push(profile_curves.remove(index));
        }
        if !curve_points_equal(
            ordered.last().expect("nonempty profile").end(),
            ordered[0].start(),
        )? {
            return Ok(None);
        }
        let profile = if ordered.iter().all(|curve| {
            matches!(
                curve.geometry(),
                CurveGeometry2::Line(_) | CurveGeometry2::CircularArc(_)
            )
        }) {
            let segments = ordered
                .into_iter()
                .map(|curve| match curve.geometry() {
                    CurveGeometry2::Line(line) => Segment2::Line(line.clone()),
                    CurveGeometry2::CircularArc(arc) => Segment2::Arc(arc.clone()),
                    _ => unreachable!("native revolution profile was prefiltered"),
                })
                .collect();
            let contour = Contour2::try_new(segments).map_err(GeometryError::from)?;
            let revolution_profile_area = contour.signed_area().map_err(GeometryError::from)?;
            if !contour
                .intersect_self(&CurvePolicy::certified())
                .map_err(GeometryError::from)?
                .is_empty()
                || decided_model_order(compare_reals(
                    &revolution_profile_area.ok_or(BuildError::DegenerateShellVolume(shell))?,
                    &Real::zero(),
                ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            CertifiedRevolutionBoundary::Native(contour)
        } else {
            let path = CurvePath2::try_new(ordered).map_err(GeometryError::from)?;
            let area = path
                .bezier_boundary_loop()
                .map_err(GeometryError::from)?
                .boundary_loop()
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?;
            if decided_model_order(compare_reals(&area, &Real::zero()))?
                != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            CertifiedRevolutionBoundary::Curved(path)
        };
        Ok(Some(CertifiedRevolutionShell {
            axis_origin: axis_origin.expect("revolution has faces"),
            axis: axis.expect("revolution has faces"),
            profile,
            voids: Vec::new(),
        }))
    }

    fn certified_revolution_planar_profile_segments(
        &self,
        faces: &[FaceId],
        axis_origin: &Point3,
        axis: &Vector3,
    ) -> Result<Option<Vec<Segment2>>, BuildError> {
        let groups = self.planar_face_groups(faces)?;
        if groups.is_empty() {
            return Ok(Some(Vec::new()));
        }
        let mut segments = Vec::with_capacity(groups.len());
        for group in groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                return Ok(None);
            };
            if boundaries.len() != 2 {
                return Ok(None);
            }
            let mut center = None;
            let mut normal_direction = None;
            for index in &group {
                let face = self.face_ref(faces[*index])?;
                let surface = self.surface_ref(face.surface)?;
                let Some(origin) = surface.plane_origin() else {
                    return Ok(None);
                };
                let Some((u, v)) = surface.plane_directions() else {
                    return Ok(None);
                };
                let normal = match face.orientation {
                    Orientation::Forward => u.cross(v),
                    Orientation::Reversed => -u.cross(v),
                };
                if decided_model_order(compare_reals(
                    &normal.cross(axis).norm_squared(),
                    &Real::zero(),
                ))? != std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let direction =
                    decided_model_order(compare_reals(&normal.dot(axis), &Real::zero()))?;
                if !matches!(
                    direction,
                    std::cmp::Ordering::Less | std::cmp::Ordering::Greater
                ) || normal_direction.is_some_and(|expected| expected != direction)
                {
                    return Ok(None);
                }
                normal_direction = Some(direction);
                let axial = axis.dot(&(origin - axis_origin));
                let candidate = axis_origin.clone() + axis.clone() * axial;
                if let Some(expected) = &center {
                    if !points_equal(expected, &candidate)? {
                        return Ok(None);
                    }
                } else {
                    center = Some(candidate);
                }
            }
            let center = center.expect("nonempty planar face group");
            let mut radii = Vec::with_capacity(2);
            for boundary in boundaries {
                let mut radius = None;
                let mut sweep = Real::zero();
                for edge_use_id in boundary {
                    let edge_use = self.edge_use_ref(edge_use_id)?;
                    let edge = self.edge_ref(edge_use.edge)?;
                    let curve = self.curve_ref(edge.curve)?;
                    let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                        return Ok(None);
                    };
                    if !data.circle || !points_equal(&data.center, &center)? {
                        return Ok(None);
                    }
                    if let Some(expected) = &radius {
                        if !real_values_equal(expected, &data.x_radius)?
                            || !real_values_equal(expected, &data.y_radius)?
                        {
                            return Ok(None);
                        }
                    } else if !real_values_equal(&data.x_radius, &data.y_radius)? {
                        return Ok(None);
                    } else {
                        radius = Some(data.x_radius);
                    }
                    sweep += edge.domain.end() - edge.domain.start();
                }
                if !real_values_equal(&sweep, &Real::tau())? {
                    return Ok(None);
                }
                radii.push(radius.expect("nonempty cap boundary"));
            }
            if decided_model_order(compare_reals(&radii[0], &radii[1]))?
                == std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
            let (inner, outer) = if decided_model_order(compare_reals(&radii[0], &radii[1]))?
                == std::cmp::Ordering::Less
            {
                (radii.remove(0), radii.remove(0))
            } else {
                (radii.remove(1), radii.remove(0))
            };
            let axial = axis.dot(&(&center - axis_origin));
            let (start, end) = if normal_direction.expect("cap group has an effective normal")
                == std::cmp::Ordering::Greater
            {
                (
                    CurvePoint2::new(outer, axial.clone()),
                    CurvePoint2::new(inner, axial),
                )
            } else {
                (
                    CurvePoint2::new(inner, axial.clone()),
                    CurvePoint2::new(outer, axial),
                )
            };
            segments.push(Segment2::Line(
                LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
            ));
        }
        Ok(Some(segments))
    }

    fn certified_cylinder_side_faces(
        &self,
        shell: ShellId,
        side_faces: &[FaceId],
        orientation: Orientation,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        if side_faces.len() < 3 {
            return Ok(None);
        }
        let cylinder_surface_id = self.face_ref(side_faces[0])?.surface;
        if side_faces
            .iter()
            .any(|face| self.face_ref(*face).map(|face| face.surface) != Ok(cylinder_surface_id))
        {
            return Ok(None);
        }
        let SurfaceExactData::Cylinder {
            origin,
            axis,
            radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };

        let mut u_values = Vec::new();
        let mut v_values = Vec::new();
        let mut face_coordinates = Vec::with_capacity(side_faces.len());
        for face_id in side_faces {
            let face = self.face_ref(*face_id)?;
            if face.orientation != orientation || !face.inner().is_empty() {
                return Ok(None);
            }
            let Some(outer) = face.outer() else {
                return Ok(None);
            };
            let wire = self.wire_ref(outer)?;
            if wire.edge_uses.len() < 4 {
                return Ok(None);
            }
            let mut face_u = Vec::with_capacity(8);
            let mut face_v = Vec::with_capacity(8);
            let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
            for edge_use_id in &wire.edge_uses {
                let pcurve = self.pcurve_ref(self.edge_use_ref(*edge_use_id)?.pcurve)?;
                let Some(line) = pcurve.line_segment() else {
                    return Ok(None);
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                if u_constant == v_constant {
                    return Ok(None);
                }
                face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                parameter_segments.push(Segment2::Line(line.clone()));
            }
            let (u_min, u_max) = exact_real_min_max(&face_u)?;
            let (v_min, v_max) = exact_real_min_max(&face_v)?;
            let contour = Contour2::try_new(parameter_segments).map_err(GeometryError::from)?;
            let represented_area = contour
                .signed_area()
                .map_err(GeometryError::from)?
                .ok_or(BuildError::DegenerateShellVolume(shell))?
                .abs();
            let rectangle_area = (&u_max - &u_min) * (&v_max - &v_min);
            if !real_values_equal(&represented_area, &rectangle_area)? {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, &u_min)?;
            insert_sorted_real(&mut u_values, &u_max)?;
            insert_sorted_real(&mut v_values, &v_min)?;
            insert_sorted_real(&mut v_values, &v_max)?;
            face_coordinates.push((u_min, u_max, v_min, v_max));
        }
        if u_values.len() != 5 || v_values.len() < 2 {
            return Ok(None);
        }
        let quarter =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        for pair in u_values.windows(2) {
            if !real_values_equal(&(&pair[1] - &pair[0]), &quarter)? {
                return Ok(None);
            }
        }
        let v_min = v_values[0].clone();
        let v_max = v_values.last().expect("cylinder has axial values").clone();
        if decided_model_order(compare_reals(&v_min, &v_max))? != std::cmp::Ordering::Less {
            return Ok(None);
        }

        let mut covered_cells = HashSet::new();
        for (u_min, u_max, face_v_min, face_v_max) in face_coordinates {
            let u_start = exact_real_index(&u_values, &u_min)?;
            let u_end = exact_real_index(&u_values, &u_max)?;
            let v_start = exact_real_index(&v_values, &face_v_min)?;
            let v_end = exact_real_index(&v_values, &face_v_max)?;
            if u_end != u_start + 1 || v_start >= v_end {
                return Ok(None);
            }
            for v_cell in v_start..v_end {
                if !covered_cells.insert((u_start, v_cell)) {
                    return Ok(None);
                }
            }
        }
        if covered_cells.len() != (u_values.len() - 1) * (v_values.len() - 1) {
            return Ok(None);
        }
        Ok(Some(CertifiedCylinderShell {
            origin,
            axis,
            radius,
            v_min,
            v_max,
            sphere_subtraction: None,
        }))
    }

    fn cylinder_face_v_bounds(&self, face_id: FaceId) -> Result<Option<(Real, Real)>, BuildError> {
        let face = self.face_ref(face_id)?;
        if self.surface_ref(face.surface)?.kind() != SurfaceKind::Cylinder {
            return Ok(None);
        }
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        let mut values = Vec::new();
        for edge_use_id in &self.wire_ref(outer)?.edge_uses {
            let Some(line) = self
                .pcurve_ref(self.edge_use_ref(*edge_use_id)?.pcurve)?
                .line_segment()
            else {
                return Ok(None);
            };
            values.extend([line.start().y().clone(), line.end().y().clone()]);
        }
        exact_real_min_max(&values).map(Some)
    }

    fn certified_cylinder_cap_group(
        &self,
        faces: &[FaceId],
        cap_group: &[usize],
        cylinder: &CertifiedCylinderShell,
        cap_parameter: &Real,
        expected_direction: std::cmp::Ordering,
    ) -> Result<bool, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, cap_group)? else {
            return Ok(false);
        };
        let [boundary] = boundaries.as_slice() else {
            return Ok(false);
        };
        let expected_center = cylinder.origin.clone() + cylinder.axis.clone() * cap_parameter;
        let mut sweep = Real::zero();
        for edge_use_id in boundary {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(false);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &cylinder.radius)?
                || !real_values_equal(&circle.y_radius, &cylinder.radius)?
            {
                return Ok(false);
            }
            sweep += edge.domain.end() - edge.domain.start();
        }
        if !real_values_equal(&sweep, &Real::tau())? {
            return Ok(false);
        }
        for index in cap_group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(false);
            };
            let normal = u.cross(&v);
            if decided_model_order(compare_reals(
                &normal.cross(&cylinder.axis).norm_squared(),
                &Real::zero(),
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(
                    &(origin - &cylinder.origin).dot(&cylinder.axis),
                    cap_parameter,
                )?
            {
                return Ok(false);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(&cylinder.axis),
                Orientation::Reversed => -normal.dot(&cylinder.axis),
            };
            if decided_model_order(compare_reals(&oriented, &Real::zero()))? != expected_direction {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn certified_cylinder_sphere_component_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        side_faces: &[FaceId],
        sphere_faces: &[FaceId],
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let [sphere_face] = sphere_faces else {
            return Ok(None);
        };
        let Some(sphere_cap) = self.certified_spherical_cap_face(*sphere_face)? else {
            return Ok(None);
        };
        if sphere_cap.orientation != Orientation::Reversed {
            return Ok(None);
        }
        let Some(mut cylinder) =
            self.certified_cylinder_side_faces(shell, side_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        if !vectors_equal(&sphere_cap.axis, &cylinder.axis)?
            || decided_model_order(compare_reals(&cylinder.radius, &sphere_cap.radius))?
                != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &sphere_cap.center - &cylinder.origin;
        let center_parameter = center_offset.dot(&cylinder.axis);
        let radial_offset = center_offset - cylinder.axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
            != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let intersection_height = &sphere_cap.radius * sphere_cap.latitude.clone().sin();
        if !real_values_equal(
            &(&sphere_cap.radius * sphere_cap.latitude.clone().cos()),
            &cylinder.radius,
        )? {
            return Ok(None);
        }
        let (side, cap_parameter, expected_cap_direction) = if sphere_cap.upper {
            if decided_model_order(compare_reals(&intersection_height, &Real::zero()))?
                != std::cmp::Ordering::Greater
                || !real_values_equal(&cylinder.v_min, &(&center_parameter + &intersection_height))?
                || decided_model_order(compare_reals(
                    &(&cylinder.v_max - &center_parameter),
                    &sphere_cap.radius,
                ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            (
                CertifiedCylinderSphereComponentSide::Upper,
                cylinder.v_max.clone(),
                std::cmp::Ordering::Greater,
            )
        } else {
            if decided_model_order(compare_reals(&intersection_height, &Real::zero()))?
                != std::cmp::Ordering::Less
                || !real_values_equal(&cylinder.v_max, &(&center_parameter + &intersection_height))?
                || decided_model_order(compare_reals(
                    &(&cylinder.v_min - &center_parameter),
                    &-sphere_cap.radius.clone(),
                ))? != std::cmp::Ordering::Less
            {
                return Ok(None);
            }
            (
                CertifiedCylinderSphereComponentSide::Lower,
                cylinder.v_min.clone(),
                std::cmp::Ordering::Less,
            )
        };

        let cap_groups = self.planar_face_groups(faces)?;
        let [cap_group] = cap_groups.as_slice() else {
            return Ok(None);
        };
        if !self.certified_cylinder_cap_group(
            faces,
            cap_group,
            &cylinder,
            &cap_parameter,
            expected_cap_direction,
        )? {
            return Ok(None);
        }
        cylinder.sphere_subtraction = Some(CertifiedCylinderSphereSubtraction::Component {
            center: sphere_cap.center,
            radius: sphere_cap.radius,
            side,
        });
        Ok(Some(cylinder))
    }

    fn certified_oriented_cylinder_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut side_faces = Vec::new();
        let mut sphere_faces = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(None);
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::Cylinder => side_faces.push(*face_id),
                SurfaceKind::Sphere => sphere_faces.push(*face_id),
                _ => return Ok(None),
            }
        }
        if !sphere_faces.is_empty() {
            if orientation != Orientation::Forward {
                return Ok(None);
            }
            return self.certified_cylinder_sphere_component_shell(
                shell,
                faces,
                &side_faces,
                &sphere_faces,
            );
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 || side_faces.len() < 3 {
            return Ok(None);
        }
        let Some(cylinder) = self.certified_cylinder_side_faces(shell, &side_faces, orientation)?
        else {
            return Ok(None);
        };

        let expected_centers = [
            cylinder.origin.clone() + cylinder.axis.clone() * &cylinder.v_min,
            cylinder.origin.clone() + cylinder.axis.clone() * &cylinder.v_max,
        ];
        let mut matched_centers = [false; 2];
        for cap_group in cap_groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &cap_group)? else {
                return Ok(None);
            };
            if boundaries.len() != 1 {
                return Ok(None);
            }
            let mut sweep = Real::zero();
            let mut center_index = None;
            for edge_use_id in &boundaries[0] {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let curve = self.curve_ref(edge.curve)?;
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    return Ok(None);
                };
                if !data.circle
                    || decided_model_order(compare_reals(&data.x_radius, &cylinder.radius))?
                        != std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let mut this_center = None;
                for (index, expected) in expected_centers.iter().enumerate() {
                    if points_equal(&data.center, expected)? {
                        this_center = Some(index);
                        break;
                    }
                }
                let Some(this_center) = this_center else {
                    return Ok(None);
                };
                if center_index.is_some_and(|index| index != this_center) {
                    return Ok(None);
                }
                center_index = Some(this_center);
                sweep += edge.domain.end() - edge.domain.start();
            }
            if decided_model_order(compare_reals(&sweep, &Real::tau()))?
                != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
            let Some(center_index) = center_index else {
                return Ok(None);
            };
            if matched_centers[center_index] {
                return Ok(None);
            }
            matched_centers[center_index] = true;
            let outward = if center_index == 0 {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
            let expected_direction = if orientation == Orientation::Forward {
                outward
            } else {
                outward.reverse()
            };
            let cap_parameter = if center_index == 0 {
                &cylinder.v_min
            } else {
                &cylinder.v_max
            };
            if !self.certified_cylinder_cap_group(
                faces,
                &cap_group,
                &cylinder,
                cap_parameter,
                expected_direction,
            )? {
                return Ok(None);
            }
        }
        if matched_centers != [true, true] {
            return Ok(None);
        }
        Ok(Some(cylinder))
    }

    fn certified_cylinder_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        self.certified_oriented_cylinder_shell(shell, Orientation::Forward)
    }

    fn certified_cylinder_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let Some(mut cylinder) = self.certified_cylinder_shell(solid.outer)? else {
            return Ok(None);
        };
        if solid.voids.is_empty() {
            return Ok(Some(cylinder));
        }
        let [void_shell] = solid.voids.as_slice() else {
            return Ok(None);
        };
        let Some((center, radius)) =
            self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
        else {
            return Ok(None);
        };
        if !self.cylinder_strictly_contains_sphere(&cylinder, &center, &radius)? {
            return Ok(None);
        }
        cylinder.sphere_subtraction =
            Some(CertifiedCylinderSphereSubtraction::Void { center, radius });
        Ok(Some(cylinder))
    }

    fn certified_z_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedZPrismShell>, BuildError> {
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_line_arc_prism_shell(shell)?
        {
            return Ok(None);
        }
        let faces = &self.shell_ref(shell)?.faces;
        let mut vertices = HashSet::new();
        for face_id in faces {
            vertices.extend(self.face_boundary_vertex_set(self.face_ref(*face_id)?)?);
        }
        let Some(first_vertex) = vertices.iter().next() else {
            return Ok(None);
        };
        let first_z = self.vertex_ref(*first_vertex)?.point().z.clone();
        let mut z_min = first_z.clone();
        let mut z_max = first_z;
        for vertex in &vertices {
            let z = &self.vertex_ref(*vertex)?.point().z;
            if decided_model_order(compare_reals(z, &z_min))? == std::cmp::Ordering::Less {
                z_min = z.clone();
            }
            if decided_model_order(compare_reals(z, &z_max))? == std::cmp::Ordering::Greater {
                z_max = z.clone();
            }
        }
        if decided_model_order(compare_reals(&z_min, &z_max))? != std::cmp::Ordering::Less {
            return Ok(None);
        }
        for vertex in &vertices {
            let z = &self.vertex_ref(*vertex)?.point().z;
            if decided_model_order(compare_reals(z, &z_min))? != std::cmp::Ordering::Equal
                && decided_model_order(compare_reals(z, &z_max))? != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
        }

        for group in self.planar_face_groups(faces)? {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                continue;
            };
            if boundaries.len() != 1 {
                continue;
            }
            let mut segments = Vec::with_capacity(boundaries[0].len());
            let mut horizontal_cap = true;
            for edge_use in &boundaries[0] {
                let (start, end) = self.directed_vertices(*edge_use)?;
                let start = self.vertex_ref(start)?.point();
                let end = self.vertex_ref(end)?.point();
                if decided_model_order(compare_reals(&start.z, &z_max))?
                    != std::cmp::Ordering::Equal
                    || decided_model_order(compare_reals(&end.z, &z_max))?
                        != std::cmp::Ordering::Equal
                {
                    horizontal_cap = false;
                    break;
                }
                segments.push(self.projected_cap_segment(*edge_use, start, end)?);
            }
            if horizontal_cap {
                return Ok(Some(CertifiedZPrismShell {
                    contour: Contour2::try_new(segments).map_err(GeometryError::from)?,
                    z_min,
                    z_max,
                }));
            }
        }
        Ok(None)
    }

    fn projected_cap_segment(
        &self,
        edge_use_id: EdgeUseId,
        start: &Point3,
        end: &Point3,
    ) -> Result<Segment2, BuildError> {
        let edge_use = self.edge_use_ref(edge_use_id)?;
        let edge = self.edge_ref(edge_use.edge)?;
        let curve = self.curve_ref(edge.curve)?;
        match curve.kind() {
            Curve3Kind::Line => Ok(Segment2::Line(
                LineSeg2::try_new(
                    CurvePoint2::new(start.x.clone(), start.y.clone()),
                    CurvePoint2::new(end.x.clone(), end.y.clone()),
                )
                .map_err(GeometryError::from)?,
            )),
            Curve3Kind::CircleArc => {
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    unreachable!("circle kind carries ellipse-arc exact data");
                };
                require_real_equal(&data.center.z, &start.z, BuildError::EdgeUseSupportMismatch)?;
                let parameter = match edge_use.direction {
                    Direction::Forward => edge.domain.start(),
                    Direction::Reversed => edge.domain.end(),
                };
                let mut tangent = curve.derivative_at(parameter, 1)?.vector().clone();
                if edge_use.direction == Direction::Reversed {
                    tangent = -tangent;
                }
                let radial_x = &start.x - &data.center.x;
                let radial_y = &start.y - &data.center.y;
                let cross = radial_x * &tangent.0[1] - radial_y * &tangent.0[0];
                let clockwise = match decided_model_order(compare_reals(&cross, &Real::zero()))? {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Greater => false,
                    std::cmp::Ordering::Equal => {
                        return Err(BuildError::EdgeUseSupportMismatch);
                    }
                };
                Ok(Segment2::Arc(
                    CircularArc2::try_from_center(
                        CurvePoint2::new(start.x.clone(), start.y.clone()),
                        CurvePoint2::new(end.x.clone(), end.y.clone()),
                        CurvePoint2::new(data.center.x, data.center.y),
                        clockwise,
                    )
                    .map_err(GeometryError::from)?,
                ))
            }
            _ => Err(BuildError::UnsupportedEdgeUseAgreement {
                curve: curve.kind(),
                pcurve: self.pcurve_ref(edge_use.pcurve)?.kind(),
                surface: SurfaceKind::Plane,
            }),
        }
    }

    fn planar_face_groups(&self, faces: &[FaceId]) -> Result<Vec<Vec<usize>>, BuildError> {
        let mut groups = Vec::<Vec<usize>>::new();
        let mut group_surfaces = Vec::<SurfaceId>::new();
        for (index, face_id) in faces.iter().enumerate() {
            let face = self.face_ref(*face_id)?;
            if self.surface_ref(face.surface)?.kind() != SurfaceKind::Plane {
                continue;
            }
            let mut position = None;
            for (index, surface) in group_surfaces.iter().enumerate() {
                if *surface == face.surface
                    || matches!(
                        self.surface_ref(*surface)?
                            .intersect_surface(self.surface_ref(face.surface)?)?,
                        crate::SurfaceSurfaceIntersection::Coincident
                    )
                {
                    position = Some(index);
                    break;
                }
            }
            if let Some(position) = position {
                groups[position].push(index);
            } else {
                group_surfaces.push(face.surface);
                groups.push(vec![index]);
            }
        }
        Ok(groups)
    }

    fn cap_boundary_use_loops(
        &self,
        faces: &[FaceId],
        group: &[usize],
    ) -> Result<Option<Vec<Vec<EdgeUseId>>>, BuildError> {
        let mut uses_by_edge = HashMap::<EdgeId, Vec<EdgeUseId>>::new();
        for index in group {
            let face = self.face_ref(faces[*index])?;
            for wire in face.boundary_wires() {
                for edge_use in &self.wire_ref(*wire)?.edge_uses {
                    uses_by_edge
                        .entry(self.edge_use_ref(*edge_use)?.edge)
                        .or_default()
                        .push(*edge_use);
                }
            }
        }
        if uses_by_edge
            .values()
            .any(|uses| uses.is_empty() || uses.len() > 2)
        {
            return Ok(None);
        }
        let boundary_uses = uses_by_edge
            .values()
            .filter(|uses| uses.len() == 1)
            .map(|uses| uses[0])
            .collect::<Vec<_>>();
        if boundary_uses.is_empty() {
            return Ok(None);
        }
        let mut outgoing = HashMap::<VertexId, EdgeUseId>::new();
        for edge_use in &boundary_uses {
            let (start, _) = self.directed_vertices(*edge_use)?;
            if outgoing.insert(start, *edge_use).is_some() {
                return Ok(None);
            }
        }
        let mut remaining = boundary_uses.into_iter().collect::<HashSet<_>>();
        let mut loops = Vec::new();
        while !remaining.is_empty() {
            let first = *remaining.iter().min().expect("nonempty boundary-use set");
            let (loop_start, _) = self.directed_vertices(first)?;
            let mut current = first;
            let mut uses = Vec::new();
            loop {
                if !remaining.remove(&current) {
                    return Ok(None);
                }
                uses.push(current);
                let (_, end) = self.directed_vertices(current)?;
                if end == loop_start {
                    break;
                }
                let Some(next) = outgoing.get(&end) else {
                    return Ok(None);
                };
                current = *next;
            }
            loops.push(uses);
        }
        Ok(Some(loops))
    }

    fn boundary_loop_vertex_set(
        &self,
        loops: &[Vec<EdgeUseId>],
    ) -> Result<HashSet<VertexId>, BuildError> {
        loops
            .iter()
            .flatten()
            .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
            .collect()
    }

    fn boundary_loop_edge_set(
        &self,
        loops: &[Vec<EdgeUseId>],
    ) -> Result<HashSet<EdgeId>, BuildError> {
        loops
            .iter()
            .flatten()
            .map(|edge_use| self.edge_use_ref(*edge_use).map(|record| record.edge))
            .collect()
    }

    fn certify_prism_cap_pair(
        &self,
        faces: &[FaceId],
        first_group: &[usize],
        second_group: &[usize],
    ) -> Result<bool, BuildError> {
        let Some(first_boundary) = self.cap_boundary_use_loops(faces, first_group)? else {
            return Ok(false);
        };
        let Some(second_boundary) = self.cap_boundary_use_loops(faces, second_group)? else {
            return Ok(false);
        };
        let first_count = first_boundary.iter().map(Vec::len).sum::<usize>();
        let second_count = second_boundary.iter().map(Vec::len).sum::<usize>();
        if first_count < 2 || second_count < 2 {
            return Ok(false);
        }
        let first_face = self.face_ref(faces[first_group[0]])?;
        let second_face = self.face_ref(faces[second_group[0]])?;
        let first_surface = self.surface_ref(first_face.surface)?;
        let second_surface = self.surface_ref(second_face.surface)?;
        let Some((first_u, first_v)) = first_surface.plane_directions() else {
            return Ok(false);
        };
        let Some((second_u, second_v)) = second_surface.plane_directions() else {
            return Ok(false);
        };
        if decided_model_order(compare_reals(
            &first_u
                .cross(first_v)
                .cross(&second_u.cross(second_v))
                .norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(false);
        }

        let first_vertices = self.boundary_loop_vertex_set(&first_boundary)?;
        let second_vertices = self.boundary_loop_vertex_set(&second_boundary)?;
        if !first_vertices.is_disjoint(&second_vertices) {
            return Ok(false);
        }
        let shell_edges = self.shell_edge_set(faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                let first_to_second =
                    first_vertices.contains(&edge.start) && second_vertices.contains(&edge.end);
                let second_to_first =
                    second_vertices.contains(&edge.start) && first_vertices.contains(&edge.end);
                (first_to_second || second_to_first).then_some(*edge_id)
            })
            .collect::<Vec<_>>();
        let count = cross_edges.len();
        if count < 2 {
            return Ok(false);
        }

        let mut first_to_second = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = &self.edges[edge_id.index()];
            let (first, second) = if first_vertices.contains(&edge.start) {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if first_to_second.insert(first, second).is_some() {
                return Ok(false);
            }
        }
        if first_to_second.len() != count
            || first_to_second
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(false);
        }
        let first_vertex = *first_to_second
            .keys()
            .next()
            .expect("certified cap pair has cross edges");
        let translation = self.vertex_ref(first_to_second[&first_vertex])?.point()
            - self.vertex_ref(first_vertex)?.point();
        for (first, second) in &first_to_second {
            let candidate = self.vertex_ref(*second)?.point() - self.vertex_ref(*first)?.point();
            if !vectors_equal(&translation, &candidate)? {
                return Ok(false);
            }
        }

        let first_edges = self.boundary_loop_edge_set(&first_boundary)?;
        let second_edges = self.boundary_loop_edge_set(&second_boundary)?;
        let cross_edges = cross_edges.into_iter().collect::<HashSet<_>>();
        let lateral_indices = faces
            .iter()
            .enumerate()
            .filter_map(|(index, _)| {
                (!first_group.contains(&index) && !second_group.contains(&index)).then_some(index)
            })
            .collect::<Vec<_>>();
        let lateral_edges = lateral_indices
            .iter()
            .map(|index| self.face_edge_set(faces[*index]))
            .collect::<Result<Vec<_>, _>>()?;
        let mut remaining = (0..lateral_indices.len()).collect::<HashSet<_>>();
        let mut lateral_groups = Vec::<Vec<usize>>::new();
        while let Some(seed) = remaining.iter().min().copied() {
            remaining.remove(&seed);
            let mut members = vec![seed];
            loop {
                let mut attached = None;
                for candidate in remaining.iter().copied() {
                    let candidate_face = self.face_ref(faces[lateral_indices[candidate]])?;
                    let candidate_surface = candidate_face.surface;
                    let mut joins = false;
                    for member in &members {
                        let member_face = self.face_ref(faces[lateral_indices[*member]])?;
                        let supports_equal = candidate_surface == member_face.surface
                            || (self.surface_ref(candidate_surface)?.kind() == SurfaceKind::Plane
                                && self.surface_ref(member_face.surface)?.kind()
                                    == SurfaceKind::Plane
                                && matches!(
                                    self.surface_ref(candidate_surface)?.intersect_surface(
                                        self.surface_ref(member_face.surface)?
                                    )?,
                                    crate::SurfaceSurfaceIntersection::Coincident
                                ));
                        if supports_equal
                            && lateral_edges[candidate]
                                .intersection(&lateral_edges[*member])
                                .any(|edge| {
                                    !matches!(
                                        self.curve_ref(
                                            self.edge_ref(*edge)
                                                .expect("validated face edge")
                                                .curve
                                        )
                                        .expect("validated edge curve")
                                        .kind(),
                                        Curve3Kind::Line | Curve3Kind::CircleArc
                                    )
                                })
                        {
                            joins = true;
                            break;
                        }
                    }
                    if joins {
                        attached = Some(candidate);
                        break;
                    }
                }
                let Some(attached) = attached else {
                    break;
                };
                remaining.remove(&attached);
                members.push(attached);
            }
            lateral_groups.push(
                members
                    .into_iter()
                    .map(|member| lateral_indices[member])
                    .collect(),
            );
        }
        for group in lateral_groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &group)? else {
                return Ok(false);
            };
            if boundaries.len() != 1 {
                return Ok(false);
            }
            let edges = self.boundary_loop_edge_set(&boundaries)?;
            let first_count = edges.intersection(&first_edges).count();
            let second_count = edges.intersection(&second_edges).count();
            let cross_count = edges.intersection(&cross_edges).count();
            if first_count == 0
                || second_count == 0
                || cross_count != 2
                || edges.len() != first_count + second_count + cross_count
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn wire_vertex_set(&self, wire: WireId) -> Result<HashSet<VertexId>, BuildError> {
        self.wire_ref(wire)?
            .edge_uses
            .iter()
            .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
            .collect()
    }

    fn face_boundary_vertex_set(&self, face: &Face) -> Result<HashSet<VertexId>, BuildError> {
        let mut vertices = HashSet::new();
        for wire in face.boundary_wires() {
            vertices.extend(self.wire_vertex_set(*wire)?);
        }
        Ok(vertices)
    }

    fn wire_edge_set(&self, wire: WireId) -> Result<HashSet<EdgeId>, BuildError> {
        self.wire_ref(wire)?
            .edge_uses
            .iter()
            .map(|edge_use| self.edge_use_ref(*edge_use).map(|edge_use| edge_use.edge))
            .collect()
    }

    fn shell_edge_set(&self, faces: &[FaceId]) -> Result<HashSet<EdgeId>, BuildError> {
        let mut edges = HashSet::new();
        for face in faces {
            let face = self.face_ref(*face)?;
            for wire in face.boundary_wires() {
                edges.extend(self.wire_edge_set(*wire)?);
            }
        }
        Ok(edges)
    }

    fn shell_vertex_set(&self, shell: ShellId) -> Result<HashSet<VertexId>, BuildError> {
        let mut vertices = HashSet::new();
        for face_id in &self.shell_ref(shell)?.faces {
            vertices.extend(self.face_boundary_vertex_set(self.face_ref(*face_id)?)?);
        }
        Ok(vertices)
    }

    pub(crate) fn shell_representative_point(&self, shell: ShellId) -> Result<Point3, BuildError> {
        let vertex = self
            .shell_vertex_set(shell)?
            .into_iter()
            .min()
            .ok_or(BuildError::EmptyShell)?;
        Ok(self.vertex_ref(vertex)?.point().clone())
    }

    pub(crate) fn signed_shell_six_volume(&self, shell: ShellId) -> Result<Real, BuildError> {
        let mut sum = Real::zero();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            for wire_id in face.boundary_wires() {
                let wire = self.wire_ref(*wire_id)?;
                let first_use = *wire.edge_uses.first().ok_or(BuildError::EmptyWire)?;
                let (anchor_id, _) = self.directed_vertices(first_use)?;
                let anchor = self.vertex_ref(anchor_id)?.point();
                for pair in wire.edge_uses[1..].windows(2) {
                    let (first_id, _) = self.directed_vertices(pair[0])?;
                    let (second_id, _) = self.directed_vertices(pair[1])?;
                    let first = self.vertex_ref(first_id)?.point();
                    let second = self.vertex_ref(second_id)?.point();
                    sum += Vector3::from(anchor.clone())
                        .dot(&Vector3::from(first.clone()).cross(&Vector3::from(second.clone())));
                }
            }
        }
        Ok(sum)
    }

    pub(crate) fn certify_void_shell_nesting(
        &self,
        outer: ShellId,
        voids: &[ShellId],
    ) -> Result<(), BuildError> {
        self.validate_void_shell_nesting(outer, voids)
    }

    pub(crate) fn shell_is_planar(&self, shell: ShellId) -> Result<bool, BuildError> {
        for face in &self.shell_ref(shell)?.faces {
            if self.surface_ref(self.face_ref(*face)?.surface)?.kind() != SurfaceKind::Plane {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

fn certified_homothetic_loft_scale(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<Option<Real>, GeometryError> {
    if lower.len() != upper.len() || lower.len() < 3 {
        return Ok(None);
    }
    let lower_dx = lower[1].x() - lower[0].x();
    let lower_dy = lower[1].y() - lower[0].y();
    let upper_dx = upper[1].x() - upper[0].x();
    let upper_dy = upper[1].y() - upper[0].y();
    let scale = if decided_model_order(compare_reals(&lower_dx, &Real::zero()))?
        != std::cmp::Ordering::Equal
    {
        (upper_dx / &lower_dx).map_err(|_| GeometryError::ProjectiveDivision)?
    } else {
        (upper_dy / &lower_dy).map_err(|_| GeometryError::ProjectiveDivision)?
    };
    if decided_model_order(compare_reals(&scale, &Real::zero()))? != std::cmp::Ordering::Greater {
        return Ok(None);
    }
    for index in 0..lower.len() {
        for (actual, expected) in [
            (
                upper[index].x() - upper[0].x(),
                &scale * (lower[index].x() - lower[0].x()),
            ),
            (
                upper[index].y() - upper[0].y(),
                &scale * (lower[index].y() - lower[0].y()),
            ),
        ] {
            if decided_model_order(compare_reals(&actual, &expected))? != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
        }
    }
    Ok(Some(scale))
}

fn certified_convex_loft_interpolation(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<bool, GeometryError> {
    if lower.len() != upper.len() || lower.len() < 3 {
        return Ok(false);
    }
    let edge = |points: &[CurvePoint2], index: usize| {
        let next = (index + 1) % points.len();
        (
            points[next].x() - points[index].x(),
            points[next].y() - points[index].y(),
        )
    };
    let cross =
        |first: &(Real, Real), second: &(Real, Real)| &first.0 * &second.1 - &first.1 * &second.0;
    for index in 0..lower.len() {
        let next = (index + 1) % lower.len();
        let lower_edge = edge(lower, index);
        let lower_next = edge(lower, next);
        let upper_edge = edge(upper, index);
        let upper_next = edge(upper, next);
        let lower_turn = cross(&lower_edge, &lower_next);
        let upper_turn = cross(&upper_edge, &upper_next);
        let mixed_turn = cross(&lower_edge, &upper_next) + cross(&upper_edge, &lower_next);
        if decided_model_order(compare_reals(&lower_turn, &Real::zero()))?
            != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&upper_turn, &Real::zero()))?
                != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&mixed_turn, &Real::zero()))?
                == std::cmp::Ordering::Less
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn loft_parameter_area_integral(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
) -> Result<Real, GeometryError> {
    let cross =
        |first: &CurvePoint2, second: &CurvePoint2| first.x() * second.y() - first.y() * second.x();
    let mut lower_twice_area = Real::zero();
    let mut upper_twice_area = Real::zero();
    let mut mixed = Real::zero();
    for index in 0..lower.len() {
        let next = (index + 1) % lower.len();
        lower_twice_area += cross(&lower[index], &lower[next]);
        upper_twice_area += cross(&upper[index], &upper[next]);
        mixed += cross(&lower[index], &upper[next]) + cross(&upper[index], &lower[next]);
    }
    ((Real::from(2) * lower_twice_area + mixed + Real::from(2) * upper_twice_area) / Real::from(12))
        .map_err(|_| GeometryError::ProjectiveDivision)
}

fn classify_convex_loft_section(
    lower: &[CurvePoint2],
    upper: &[CurvePoint2],
    parameter: &Real,
    point: &CurvePoint2,
) -> Result<ContourPointLocation, GeometryError> {
    let one_minus = Real::one() - parameter;
    let interpolate = |index: usize| {
        CurvePoint2::new(
            &one_minus * lower[index].x() + parameter * upper[index].x(),
            &one_minus * lower[index].y() + parameter * upper[index].y(),
        )
    };
    let mut boundary = false;
    for index in 0..lower.len() {
        let start = interpolate(index);
        let end = interpolate((index + 1) % lower.len());
        let side = (end.x() - start.x()) * (point.y() - start.y())
            - (end.y() - start.y()) * (point.x() - start.x());
        match decided_model_order(compare_reals(&side, &Real::zero()))? {
            std::cmp::Ordering::Less => return Ok(ContourPointLocation::Outside),
            std::cmp::Ordering::Equal => boundary = true,
            std::cmp::Ordering::Greater => {}
        }
    }
    Ok(if boundary {
        ContourPointLocation::Boundary
    } else {
        ContourPointLocation::Inside
    })
}

fn classification_is_inside(
    classification: Classification<ContourPointLocation>,
) -> Result<bool, BuildError> {
    match classification {
        Classification::Decided(ContourPointLocation::Inside) => Ok(true),
        Classification::Decided(_) => Ok(false),
        Classification::Uncertain(reason) => Err(BuildError::Geometry(
            GeometryError::PlanarClassificationUnresolved(reason),
        )),
    }
}

fn project_point_to_plane_frame(
    point: &Point3,
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
) -> Result<CurvePoint2, BuildError> {
    let displacement = point - origin;
    let uu = u.dot(u);
    let uv = u.dot(v);
    let vv = v.dot(v);
    let du = displacement.dot(u);
    let dv = displacement.dot(v);
    let determinant = &uu * &vv - &uv * &uv;
    Ok(CurvePoint2::new(
        ((&du * &vv - &dv * &uv) / &determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&dv * &uu - &du * &uv) / determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn certified_affine_sweep_progress(path: &Curve3, normal: &Vector3) -> Result<bool, BuildError> {
    let Curve3ExactData::RationalBezier {
        control_points,
        weights,
    } = path.exact_data()
    else {
        return Ok(false);
    };
    let Some(degree) = control_points.len().checked_sub(1) else {
        return Ok(false);
    };
    let Some(elevated_degree) = degree.checked_add(1) else {
        return Ok(false);
    };
    let scalar = |point: &Point3| normal.dot(&Vector3::from(point.clone()));
    let start = scalar(&control_points[0]);
    let end = scalar(&control_points[degree]);
    if decided_model_order(compare_reals(&(&end - &start), &Real::zero()))?
        != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let numerators = control_points
        .iter()
        .zip(&weights)
        .map(|(control, weight)| weight * scalar(control))
        .collect::<Vec<_>>();
    for index in 0..=elevated_degree {
        let lower_count = Real::from(
            u128::try_from(elevated_degree - index).expect("usize is representable as u128"),
        );
        let upper_count =
            Real::from(u128::try_from(index).expect("usize is representable as u128"));
        let elevated_numerator = (if index <= degree {
            &lower_count * &numerators[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &numerators[index - 1]
        } else {
            Real::zero()
        };
        let affine_times_weight = (if index <= degree {
            &lower_count * &start * &weights[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &end * &weights[index - 1]
        } else {
            Real::zero()
        };
        if !real_values_equal(&elevated_numerator, &affine_times_weight)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn translated_rational_bezier_data_equal(
    actual_points: &[Point3],
    actual_weights: &[Real],
    expected_points: &[Point3],
    expected_weights: &[Real],
    offset: &Vector3,
) -> Result<bool, BuildError> {
    if actual_points.len() != expected_points.len()
        || actual_weights.len() != expected_weights.len()
        || actual_points.is_empty()
    {
        return Ok(false);
    }
    let weight_scale = (&actual_weights[0] / &expected_weights[0])
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    for ((actual_point, actual_weight), (expected_point, expected_weight)) in actual_points
        .iter()
        .zip(actual_weights)
        .zip(expected_points.iter().zip(expected_weights))
    {
        if !points_equal(actual_point, &(expected_point.clone() + offset))?
            || !real_values_equal(actual_weight, &(expected_weight * &weight_scale))?
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn rational_bezier_weights_proportional(
    actual: &[Real],
    expected: &[Real],
) -> Result<bool, BuildError> {
    if actual.len() != expected.len() || actual.is_empty() {
        return Ok(false);
    }
    let scale = (&actual[0] / &expected[0]).map_err(|_| GeometryError::ProjectiveDivision)?;
    for (actual, expected) in actual.iter().zip(expected) {
        if !real_values_equal(actual, &(expected * &scale))? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn translated_curve(curve: &Curve3, offset: &Vector3) -> Result<Curve3, BuildError> {
    match curve.exact_data() {
        Curve3ExactData::Line(data) => Ok(Curve3::line(data.start + offset, data.end + offset)?),
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Ok(Curve3::rational_bezier(
            control_points
                .into_iter()
                .map(|point| point + offset)
                .collect(),
            weights,
        )?),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Ok(Curve3::nurbs(
            degree,
            control_points
                .into_iter()
                .map(|point| point + offset)
                .collect(),
            weights,
            knots,
        )?),
        Curve3ExactData::EllipseArc(mut data) => {
            data.center = data.center + offset;
            Ok(Curve3::from_exact_data(Curve3ExactData::EllipseArc(data))?)
        }
    }
}

fn extrusion_constant_area_scale(
    profile: &Curve3,
    direction: &Vector3,
) -> Result<Option<Real>, BuildError> {
    let constant = match profile.exact_data() {
        Curve3ExactData::Line(_) => true,
        Curve3ExactData::EllipseArc(data) if data.circle => {
            let normal = data.x.cross(&data.y);
            real_values_equal(&direction.cross(&normal).norm_squared(), &Real::zero())?
        }
        _ => false,
    };
    if !constant {
        return Ok(None);
    }
    Ok(Some(
        profile
            .derivative_at(profile.domain().start(), 1)?
            .vector()
            .cross(direction)
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?,
    ))
}

fn certified_monotone_line_curve_image(curve: &Curve3) -> Result<bool, BuildError> {
    let control_points = match curve.exact_data() {
        Curve3ExactData::RationalBezier { control_points, .. }
        | Curve3ExactData::Nurbs { control_points, .. } => control_points,
        _ => return Ok(false),
    };
    let start = curve.point_at(curve.domain().start())?;
    let end = curve.point_at(curve.domain().end())?;
    if !points_equal(
        control_points
            .first()
            .expect("validated spline has control points"),
        &start,
    )? || !points_equal(
        control_points
            .last()
            .expect("validated spline has control points"),
        &end,
    )? {
        return Ok(false);
    }
    let direction = &end - &start;
    let length_squared = direction.norm_squared();
    if decided_model_order(compare_reals(&length_squared, &Real::zero()))?
        != std::cmp::Ordering::Greater
    {
        return Ok(false);
    }
    let mut previous_projection = Real::zero();
    for control in control_points {
        let relative = control - &start;
        if decided_model_order(compare_reals(
            &relative.cross(&direction).norm_squared(),
            &Real::zero(),
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(false);
        }
        let projection = relative.dot(&direction);
        if decided_model_order(compare_reals(&projection, &previous_projection))?
            == std::cmp::Ordering::Less
            || decided_model_order(compare_reals(&projection, &length_squared))?
                == std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
        previous_projection = projection;
    }
    Ok(true)
}

struct AffineTensorImage {
    spatial_area: Real,
    parameter_area: Real,
    constant_weights: bool,
    separable_weights: bool,
}

fn affine_tensor_face_area(
    data: &SurfaceExactData,
    parameter_double_area: &Real,
) -> Result<Option<Real>, BuildError> {
    let Some(image) = affine_tensor_image(data)? else {
        return Ok(None);
    };
    if image.constant_weights {
        let double_area = (parameter_double_area * image.spatial_area / image.parameter_area)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        return Ok(Some(
            (double_area / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?,
        ));
    }
    if !image.separable_weights
        || !real_values_equal(
            parameter_double_area,
            &(Real::from(2) * &image.parameter_area),
        )?
    {
        return Ok(None);
    }
    Ok(Some(image.spatial_area))
}

fn affine_tensor_image(data: &SurfaceExactData) -> Result<Option<AffineTensorImage>, BuildError> {
    match data {
        SurfaceExactData::RationalBezier {
            control_points,
            weights,
        } => {
            let u_count = control_points[0].len();
            let v_count = control_points.len();
            let u_denominator =
                Real::from(u128::try_from(u_count - 1).expect("usize is representable as u128"));
            let v_denominator =
                Real::from(u128::try_from(v_count - 1).expect("usize is representable as u128"));
            let u_coordinates = (0..u_count)
                .map(|index| {
                    (Real::from(u128::try_from(index).expect("usize is representable as u128"))
                        / &u_denominator)
                        .map_err(|_| GeometryError::ProjectiveDivision.into())
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            let v_coordinates = (0..v_count)
                .map(|index| {
                    (Real::from(u128::try_from(index).expect("usize is representable as u128"))
                        / &v_denominator)
                        .map_err(|_| GeometryError::ProjectiveDivision.into())
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            affine_control_net_image(
                control_points,
                &u_coordinates,
                &v_coordinates,
                &Real::one(),
                &Real::one(),
                tensor_weights_are_constant(weights)?,
                tensor_weights_are_separable(weights)?,
            )
        }
        SurfaceExactData::Nurbs {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        } => {
            let u_count = control_points[0].len();
            let v_count = control_points.len();
            let u_start = &u_knots[*u_degree];
            let u_end = &u_knots[u_count];
            let v_start = &v_knots[*v_degree];
            let v_end = &v_knots[v_count];
            let u_span = u_end - u_start;
            let v_span = v_end - v_start;
            let u_coordinates =
                normalized_greville_coordinates(u_count, *u_degree, u_knots, u_start, &u_span)?;
            let v_coordinates =
                normalized_greville_coordinates(v_count, *v_degree, v_knots, v_start, &v_span)?;
            affine_control_net_image(
                control_points,
                &u_coordinates,
                &v_coordinates,
                &u_span,
                &v_span,
                tensor_weights_are_constant(weights)?,
                tensor_weights_are_separable(weights)?,
            )
        }
        _ => Ok(None),
    }
}

fn tensor_weights_are_constant(weights: &[Vec<Real>]) -> Result<bool, BuildError> {
    let Some(first) = weights.first().and_then(|row| row.first()) else {
        return Ok(false);
    };
    for weight in weights.iter().flatten().skip(1) {
        if !real_values_equal(weight, first)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn tensor_weights_are_separable(weights: &[Vec<Real>]) -> Result<bool, BuildError> {
    let Some(first_row) = weights.first() else {
        return Ok(false);
    };
    let Some(first) = first_row.first() else {
        return Ok(false);
    };
    if weights.iter().any(|row| row.len() != first_row.len()) {
        return Ok(false);
    }
    // These cross-products prove that the positive weight matrix has rank one
    // without choosing a potentially expression-expanding quotient.
    for row in weights.iter().skip(1) {
        for (weight, first_row_weight) in row.iter().zip(first_row) {
            if !real_values_equal(&(weight * first), &(&row[0] * first_row_weight))? {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn normalized_greville_coordinates(
    count: usize,
    degree: usize,
    knots: &[Real],
    domain_start: &Real,
    domain_span: &Real,
) -> Result<Vec<Real>, BuildError> {
    let degree_scalar = Real::from(u128::try_from(degree).expect("usize is representable as u128"));
    (0..count)
        .map(|index| {
            let sum = knots[index + 1..=index + degree]
                .iter()
                .fold(Real::zero(), |sum, knot| sum + knot);
            let greville = (sum / &degree_scalar).map_err(|_| GeometryError::ProjectiveDivision)?;
            ((greville - domain_start) / domain_span)
                .map_err(|_| GeometryError::ProjectiveDivision.into())
        })
        .collect()
}

fn affine_control_net_image(
    control_points: &[Vec<Point3>],
    u_coordinates: &[Real],
    v_coordinates: &[Real],
    u_parameter_span: &Real,
    v_parameter_span: &Real,
    constant_weights: bool,
    separable_weights: bool,
) -> Result<Option<AffineTensorImage>, BuildError> {
    let origin = &control_points[0][0];
    let u = &control_points[0][control_points[0].len() - 1] - origin;
    let v = &control_points[control_points.len() - 1][0] - origin;
    let cross_squared = u.cross(&v).norm_squared();
    if decided_model_order(compare_reals(&cross_squared, &Real::zero()))?
        != std::cmp::Ordering::Greater
    {
        return Ok(None);
    }
    for (row, v_coordinate) in control_points.iter().zip(v_coordinates) {
        for (point, u_coordinate) in row.iter().zip(u_coordinates) {
            let expected = origin.clone() + u.clone() * u_coordinate + v.clone() * v_coordinate;
            if !points_equal(point, &expected)? {
                return Ok(None);
            }
        }
    }
    let spatial_area = cross_squared
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    Ok(Some(AffineTensorImage {
        spatial_area,
        parameter_area: u_parameter_span * v_parameter_span,
        constant_weights,
        separable_weights,
    }))
}

fn divide_vector_by_real(vector: Vector3, denominator: &Real) -> Result<Vector3, BuildError> {
    Ok(Vector3::from_xyz(
        (vector.0[0].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
        (vector.0[1].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
        (vector.0[2].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn certified_sweep_frame_area_integral(
    u_controls: &[Vector3],
    v_controls: &[Vector3],
    weights: &[Real],
    normal: &Vector3,
) -> Result<Option<Real>, BuildError> {
    if u_controls.len() != v_controls.len()
        || u_controls.len() != weights.len()
        || u_controls.is_empty()
    {
        return Ok(None);
    }
    for axis in u_controls.iter().chain(v_controls) {
        if !real_values_equal(&axis.dot(normal), &Real::zero())? {
            return Ok(None);
        }
    }
    let degree = u_controls.len() - 1;
    let Some(product_degree) = degree.checked_mul(2) else {
        return Ok(None);
    };
    let normal_squared = normal.norm_squared();
    let mut constant_area = true;
    let mut determinant_numerators = Vec::with_capacity(product_degree + 1);
    for product_index in 0..=product_degree {
        let first_min = product_index.saturating_sub(degree);
        let first_max = degree.min(product_index);
        let mut cross_coefficient = Vector3::zero();
        let mut weight_coefficient = Real::zero();
        for first in first_min..=first_max {
            let second = product_index - first;
            let coefficient = model_bernstein_product_coefficient(degree, first, second)?;
            let weighted_u = u_controls[first].clone() * &weights[first];
            let weighted_v = v_controls[second].clone() * &weights[second];
            cross_coefficient = cross_coefficient + weighted_u.cross(&weighted_v) * &coefficient;
            weight_coefficient += &coefficient * &weights[first] * &weights[second];
        }
        let expected = normal.clone() * weight_coefficient;
        constant_area &= vectors_equal(&cross_coefficient, &expected)?;
        let determinant_numerator = (cross_coefficient.dot(normal) / &normal_squared)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        if !vectors_equal(
            &cross_coefficient,
            &(normal.clone() * &determinant_numerator),
        )? {
            return Ok(None);
        }
        determinant_numerators.push(determinant_numerator);
    }
    if constant_area {
        return Ok(Some(Real::one()));
    }
    for weight in &weights[1..] {
        if !real_values_equal(weight, &weights[0])? {
            return Ok(None);
        }
    }
    let weight_squared = &weights[0] * &weights[0];
    let mut integral = Real::zero();
    for numerator in determinant_numerators {
        let coefficient =
            (numerator / &weight_squared).map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_model_order(compare_reals(&coefficient, &Real::zero()))?
            != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        integral += coefficient;
    }
    Ok(Some(
        (integral
            / Real::from(
                u128::try_from(product_degree + 1).expect("usize is representable as u128"),
            ))
        .map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn model_bernstein_product_coefficient(
    degree: usize,
    first: usize,
    second: usize,
) -> Result<Real, BuildError> {
    let numerator = model_binomial_real(degree, first)? * model_binomial_real(degree, second)?;
    (numerator
        / model_binomial_real(
            degree
                .checked_mul(2)
                .ok_or(GeometryError::ProjectiveDivision)?,
            first + second,
        )?)
    .map_err(|_| GeometryError::ProjectiveDivision.into())
}

fn model_binomial_real(n: usize, k: usize) -> Result<Real, BuildError> {
    let k = k.min(n - k);
    let mut result = Real::one();
    for index in 1..=k {
        result = (result
            * Real::from(u128::try_from(n + 1 - index).expect("usize is representable as u128"))
            / Real::from(u128::try_from(index).expect("usize is representable as u128")))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    }
    Ok(result)
}

fn vector_control_curve(controls: Vec<Vector3>, weights: &[Real]) -> Result<Curve3, GeometryError> {
    Curve3::rational_bezier(
        controls.into_iter().map(Point3::from).collect(),
        weights.to_vec(),
    )
}

fn constant_vector_curve(vector: &Vector3, weights: &[Real]) -> Result<Curve3, GeometryError> {
    vector_control_curve(vec![vector.clone(); weights.len()], weights)
}

fn transform_vector_curve(curve: &Curve3, transform: &Matrix4) -> Result<Curve3, GeometryError> {
    let Curve3ExactData::RationalBezier {
        control_points,
        weights,
    } = curve.exact_data()
    else {
        return Err(GeometryError::UnsupportedTransform);
    };
    Curve3::rational_bezier(
        control_points
            .into_iter()
            .map(|point| Point3::from(transform.transform_direction3(&Vector3::from(point))))
            .collect(),
        weights,
    )
}

fn translated_rational_bezier_surface_equal(
    actual_points: &[Vec<Point3>],
    actual_weights: &[Vec<Real>],
    path_points: &[Point3],
    path_weights: &[Real],
    start_offset: &Vector3,
    end_offset: &Vector3,
) -> Result<bool, BuildError> {
    if actual_points.len() != path_points.len()
        || actual_weights.len() != path_weights.len()
        || path_points.is_empty()
        || actual_points.iter().any(|row| row.len() != 2)
        || actual_weights.iter().any(|row| row.len() != 2)
    {
        return Ok(false);
    }
    for reverse_path in [false, true] {
        for swap_profile in [false, true] {
            let path_index = |row: usize| {
                if reverse_path {
                    path_points.len() - 1 - row
                } else {
                    row
                }
            };
            let first_path_index = path_index(0);
            let weight_scale = (&actual_weights[0][0] / &path_weights[first_path_index])
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let mut equal = true;
            for row in 0..path_points.len() {
                let expected_row = path_index(row);
                for column in 0..2 {
                    let expected_column = if swap_profile { 1 - column } else { column };
                    let offset = if expected_column == 0 {
                        start_offset
                    } else {
                        end_offset
                    };
                    if !points_equal(
                        &actual_points[row][column],
                        &(path_points[expected_row].clone() + offset),
                    )? || !real_values_equal(
                        &actual_weights[row][column],
                        &(&path_weights[expected_row] * &weight_scale),
                    )? {
                        equal = false;
                        break;
                    }
                }
                if !equal {
                    break;
                }
            }
            if equal {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

#[derive(Clone, Copy)]
struct RationalBezierSweepControlView<'a> {
    path_points: &'a [Point3],
    path_weights: &'a [Real],
    u_controls: &'a [Vector3],
    v_controls: &'a [Vector3],
}

fn framed_rational_bezier_surface_equal(
    actual_points: &[Vec<Point3>],
    actual_weights: &[Vec<Real>],
    frame: RationalBezierSweepControlView<'_>,
    start: &CurvePoint2,
    end: &CurvePoint2,
) -> Result<bool, BuildError> {
    let RationalBezierSweepControlView {
        path_points,
        path_weights,
        u_controls,
        v_controls,
    } = frame;
    if actual_points.len() != path_points.len()
        || actual_weights.len() != path_weights.len()
        || u_controls.len() != path_points.len()
        || v_controls.len() != path_points.len()
        || path_points.is_empty()
        || actual_points.iter().any(|row| row.len() != 2)
        || actual_weights.iter().any(|row| row.len() != 2)
    {
        return Ok(false);
    }
    for reverse_path in [false, true] {
        for swap_profile in [false, true] {
            let path_index = |row: usize| {
                if reverse_path {
                    path_points.len() - 1 - row
                } else {
                    row
                }
            };
            let first_path_index = path_index(0);
            let weight_scale = (&actual_weights[0][0] / &path_weights[first_path_index])
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let mut equal = true;
            for row in 0..path_points.len() {
                let expected_row = path_index(row);
                for column in 0..2 {
                    let coordinate = if (column == 0) ^ swap_profile {
                        start
                    } else {
                        end
                    };
                    let expected = path_points[expected_row].clone()
                        + u_controls[expected_row].clone() * coordinate.x()
                        + v_controls[expected_row].clone() * coordinate.y();
                    if !points_equal(&actual_points[row][column], &expected)?
                        || !real_values_equal(
                            &actual_weights[row][column],
                            &(&path_weights[expected_row] * &weight_scale),
                        )?
                    {
                        equal = false;
                        break;
                    }
                }
                if !equal {
                    break;
                }
            }
            if equal {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn project_point_to_surface_plane(
    point: &Point3,
    surface: &Surface,
) -> Result<CurvePoint2, BuildError> {
    let origin = surface
        .plane_origin()
        .expect("planar certificate prevalidates plane surfaces");
    let (u, v) = surface
        .plane_directions()
        .expect("planar certificate prevalidates plane surfaces");
    project_point_to_plane_frame(point, origin, u, v)
}

fn planar_surface_value(surface: &Surface, point: &Point3) -> Real {
    let origin = surface
        .plane_origin()
        .expect("planar certificate prevalidates plane surfaces");
    let (u, v) = surface
        .plane_directions()
        .expect("planar certificate prevalidates plane surfaces");
    u.cross(v).dot(&(point - origin))
}

fn line_parameter(
    candidate: &Point3,
    point: &Point3,
    direction: &Vector3,
) -> Result<Real, BuildError> {
    ((candidate - point).dot(direction) / direction.norm_squared())
        .map_err(|_| GeometryError::ProjectiveDivision)
        .map_err(BuildError::from)
}

fn minimum_real(first: &Real, second: &Real) -> Result<Real, BuildError> {
    Ok(
        if decided_model_order(compare_reals(first, second))? == std::cmp::Ordering::Greater {
            second.clone()
        } else {
            first.clone()
        },
    )
}

fn maximum_real(first: &Real, second: &Real) -> Result<Real, BuildError> {
    Ok(
        if decided_model_order(compare_reals(first, second))? == std::cmp::Ordering::Less {
            second.clone()
        } else {
            first.clone()
        },
    )
}

fn parameter_in_line_material(
    parameter: &Real,
    intervals: &[(Real, Real)],
    contacts: &[Real],
) -> Result<bool, BuildError> {
    for interval in intervals {
        if decided_model_order(compare_reals(parameter, &interval.0))? != std::cmp::Ordering::Less
            && decided_model_order(compare_reals(parameter, &interval.1))?
                != std::cmp::Ordering::Greater
        {
            return Ok(true);
        }
    }
    for contact in contacts {
        if decided_model_order(compare_reals(parameter, contact))? == std::cmp::Ordering::Equal {
            return Ok(true);
        }
    }
    Ok(false)
}

fn vectors_equal(left: &Vector3, right: &Vector3) -> Result<bool, BuildError> {
    for axis in 0..3 {
        match compare_reals(&left.0[axis], &right.0[axis]) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            } => {}
            PredicateOutcome::Decided { .. } => return Ok(false),
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                }));
            }
        }
    }
    Ok(true)
}

fn point3_component(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => unreachable!("three-dimensional component index"),
    }
}

fn points_equal(left: &Point3, right: &Point3) -> Result<bool, BuildError> {
    match point3_equal(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

fn curve_points_equal(left: &CurvePoint2, right: &CurvePoint2) -> Result<bool, BuildError> {
    Ok(real_values_equal(left.x(), right.x())? && real_values_equal(left.y(), right.y())?)
}

fn validate_rational_pcurve_controls(
    actual: &RationalBezier2,
    expected_points: &[CurvePoint2],
    expected_weights: &[Real],
) -> Result<(), BuildError> {
    validate_weighted_pcurve_controls(
        actual.control_points(),
        actual.weights(),
        expected_points,
        expected_weights,
    )
}

fn validate_weighted_pcurve_controls(
    actual_points: &[CurvePoint2],
    actual_weights: &[Real],
    expected_points: &[CurvePoint2],
    expected_weights: &[Real],
) -> Result<(), BuildError> {
    if actual_points.len() != expected_points.len()
        || actual_weights.len() != expected_weights.len()
        || actual_weights.is_empty()
    {
        return Err(BuildError::EdgeUseSupportMismatch);
    }
    let weight_scale = (&actual_weights[0] / &expected_weights[0])
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    for ((actual_point, actual_weight), (expected_point, expected_weight)) in actual_points
        .iter()
        .zip(actual_weights)
        .zip(expected_points.iter().zip(expected_weights))
    {
        if !curve_points_equal(actual_point, expected_point)?
            || !real_values_equal(actual_weight, &(expected_weight * &weight_scale))?
        {
            return Err(BuildError::EdgeUseSupportMismatch);
        }
    }
    Ok(())
}

fn curve2_from_segment(segment: Segment2) -> Curve2 {
    match segment {
        Segment2::Line(line) => Curve2::from(line),
        Segment2::Arc(arc) => Curve2::from(arc),
    }
}

fn planar_rational_bezier_curve(
    control_points: Vec<CurvePoint2>,
    weights: Vec<Real>,
) -> Result<Curve2, BuildError> {
    if control_points.len() != weights.len() || control_points.len() < 2 {
        return Err(GeometryError::PlanarCurveConstruction(
            hypercurve::CurveError::InvalidRationalBezier,
        )
        .into());
    }
    let uniform = weights.iter().all(|weight| weight == &weights[0]);
    match (uniform, control_points.as_slice(), weights.as_slice()) {
        (true, [start, control, end], _) => Ok(Curve2::from(QuadraticBezier2::new(
            start.clone(),
            control.clone(),
            end.clone(),
        ))),
        (true, [start, first, second, end], _) => Ok(Curve2::from(CubicBezier2::new(
            start.clone(),
            first.clone(),
            second.clone(),
            end.clone(),
        ))),
        (false, [start, control, end], [start_weight, control_weight, end_weight]) => {
            Ok(Curve2::from(
                RationalQuadraticBezier2::try_new(
                    start.clone(),
                    control.clone(),
                    end.clone(),
                    start_weight.clone(),
                    control_weight.clone(),
                    end_weight.clone(),
                )
                .map_err(GeometryError::from)?,
            ))
        }
        _ => Ok(Curve2::from(
            RationalBezier2::try_new(control_points, weights).map_err(GeometryError::from)?,
        )),
    }
}

fn validate_projective_pcurve_equal(actual: &Curve2, expected: &Curve2) -> Result<(), BuildError> {
    match (actual.geometry(), expected.geometry()) {
        (CurveGeometry2::RationalBezier(actual), CurveGeometry2::RationalBezier(expected)) => {
            validate_weighted_pcurve_controls(
                actual.control_points(),
                actual.weights(),
                expected.control_points(),
                expected.weights(),
            )
        }
        (CurveGeometry2::Nurbs(actual), CurveGeometry2::Nurbs(expected)) => {
            if actual.degree() != expected.degree()
                || !real_slices_equal(actual.knots(), expected.knots())?
            {
                return Err(BuildError::EdgeUseSupportMismatch);
            }
            validate_weighted_pcurve_controls(
                actual.control_points(),
                actual.weights(),
                expected.control_points(),
                expected.weights(),
            )
        }
        _ => Err(BuildError::EdgeUseSupportMismatch),
    }
}

fn real_slices_equal(left: &[Real], right: &[Real]) -> Result<bool, BuildError> {
    if left.len() != right.len() {
        return Ok(false);
    }
    for (left, right) in left.iter().zip(right) {
        if !real_values_equal(left, right)? {
            return Ok(false);
        }
    }
    Ok(true)
}

type TensorControlNet = (Vec<Vec<Point3>>, Vec<Vec<Real>>);

struct TrimmedRationalTensorProfile {
    points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    start: Real,
    end: Real,
}

struct TrimmedNurbsTensorProfile {
    points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    knots: Vec<Real>,
}

fn translation_tensor_rows(control_points: &[Point3], direction: &Vector3) -> Vec<Vec<Point3>> {
    control_points
        .iter()
        .map(|point| vec![point.clone(), point.clone() + direction])
        .collect()
}

fn duplicated_tensor_weights(weights: &[Real]) -> Vec<Vec<Real>> {
    weights
        .iter()
        .map(|weight| vec![weight.clone(), weight.clone()])
        .collect()
}

fn transpose_two_row_tensor(
    points: &[Vec<Point3>],
    weights: &[Vec<Real>],
) -> Option<TensorControlNet> {
    if points.len() != 2
        || weights.len() != 2
        || points[0].is_empty()
        || points[0].len() != points[1].len()
        || weights[0].len() != points[0].len()
        || weights[1].len() != points[0].len()
    {
        return None;
    }
    let mut transposed_points = Vec::with_capacity(points[0].len());
    let mut transposed_weights = Vec::with_capacity(points[0].len());
    for index in 0..points[0].len() {
        transposed_points.push(vec![points[0][index].clone(), points[1][index].clone()]);
        transposed_weights.push(vec![weights[0][index].clone(), weights[1][index].clone()]);
    }
    Some((transposed_points, transposed_weights))
}

fn rational_tensor_graph_controls(
    weights: &[Real],
    coefficients: &[Real],
    parameter_start: &Real,
    parameter_span: &Real,
    profile_axis: SurfaceIsoAxis,
) -> Result<(Vec<CurvePoint2>, Vec<Real>), BuildError> {
    if weights.len() < 2 || weights.len() != coefficients.len() {
        return Err(BuildError::EdgeUseSupportMismatch);
    }
    let control_count = weights.len();
    let denominator =
        Real::from(u128::try_from(control_count).map_err(|_| GeometryError::InvalidDegree)?);
    let coefficient_homogeneous = weights
        .iter()
        .zip(coefficients)
        .map(|(weight, coefficient)| weight * coefficient)
        .collect::<Vec<_>>();
    let mut elevated_points = Vec::with_capacity(control_count + 1);
    let mut elevated_weights = Vec::with_capacity(control_count + 1);
    for index in 0..=control_count {
        let alpha = (Real::from(u128::try_from(index).map_err(|_| GeometryError::InvalidDegree)?)
            / &denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let one_minus_alpha = Real::one() - &alpha;
        let previous_weight = (index > 0).then(|| weights[index - 1].clone());
        let next_weight = (index < control_count).then(|| weights[index].clone());
        let elevated_weight = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight)
            + next_weight
                .as_ref()
                .map_or_else(Real::zero, |weight| &one_minus_alpha * weight);
        let local_parameter_homogeneous = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight);
        let parameter_homogeneous =
            parameter_start * &elevated_weight + parameter_span * local_parameter_homogeneous;
        let previous_coefficient = if index > 0 {
            &alpha * &coefficient_homogeneous[index - 1]
        } else {
            Real::zero()
        };
        let next_coefficient = if index < control_count {
            &one_minus_alpha * &coefficient_homogeneous[index]
        } else {
            Real::zero()
        };
        let coefficient = ((previous_coefficient + next_coefficient) / &elevated_weight)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parameter = (parameter_homogeneous / &elevated_weight)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        elevated_points.push(match profile_axis {
            SurfaceIsoAxis::U => CurvePoint2::new(parameter, coefficient),
            SurfaceIsoAxis::V => CurvePoint2::new(coefficient, parameter),
        });
        elevated_weights.push(elevated_weight);
    }
    Ok((elevated_points, elevated_weights))
}

fn build_error_geometry(error: BuildError) -> GeometryError {
    match error {
        BuildError::Geometry(error) => error,
        _ => unreachable!("exact scalar comparison only returns geometry errors"),
    }
}

fn tensor_curve_images_equal(actual: &Curve3, expected: &Curve3) -> Result<bool, BuildError> {
    if let Curve3ExactData::Line(line) = actual.exact_data() {
        let (points, weights, degree) = match expected.exact_data() {
            Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } => (control_points, weights, 1),
            Curve3ExactData::Nurbs {
                degree,
                control_points,
                weights,
                ..
            } => (control_points, weights, degree),
            _ => return Ok(false),
        };
        if degree != 1 || points.len() != 2 || weights.len() != 2 {
            return Ok(false);
        }
        return Ok(real_values_equal(&weights[0], &weights[1])?
            && points_equal(&line.start, &points[0])?
            && points_equal(&line.end, &points[1])?);
    }
    let unpack = |curve: &Curve3| match curve.exact_data() {
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Some((control_points, weights, None, None)),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Some((control_points, weights, Some(degree), Some(knots))),
        _ => None,
    };
    let Some((actual_points, actual_weights, actual_degree, actual_knots)) = unpack(actual) else {
        return Ok(false);
    };
    let Some((expected_points, expected_weights, expected_degree, expected_knots)) =
        unpack(expected)
    else {
        return Ok(false);
    };
    if actual_degree != expected_degree
        || actual_points.len() != expected_points.len()
        || actual_weights.len() != expected_weights.len()
    {
        return Ok(false);
    }
    match (actual_knots.as_ref(), expected_knots.as_ref()) {
        (Some(actual), Some(expected)) if actual.len() == expected.len() => {
            for (actual, expected) in actual.iter().zip(expected) {
                if !real_values_equal(actual, expected)? {
                    return Ok(false);
                }
            }
        }
        (None, None) => {}
        _ => return Ok(false),
    }
    for (actual, expected) in actual_points.iter().zip(&expected_points) {
        if !points_equal(actual, expected)? {
            return Ok(false);
        }
    }
    let weight_scale = (&actual_weights[0] / &expected_weights[0])
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    for (actual, expected) in actual_weights.iter().zip(&expected_weights) {
        if !real_values_equal(actual, &(expected * &weight_scale))? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn curve_parameterizations_equal(actual: &Curve3, expected: &Curve3) -> Result<bool, BuildError> {
    if actual.kind() != expected.kind()
        || !real_values_equal(actual.domain().start(), expected.domain().start())?
        || !real_values_equal(actual.domain().end(), expected.domain().end())?
    {
        return Ok(false);
    }
    match (actual.exact_data(), expected.exact_data()) {
        (Curve3ExactData::Line(actual), Curve3ExactData::Line(expected)) => {
            Ok(points_equal(&actual.start, &expected.start)?
                && points_equal(&actual.end, &expected.end)?)
        }
        (
            Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. },
            Curve3ExactData::RationalBezier { .. } | Curve3ExactData::Nurbs { .. },
        ) => tensor_curve_images_equal(actual, expected),
        (Curve3ExactData::EllipseArc(actual), Curve3ExactData::EllipseArc(expected)) => {
            Ok(actual.circle == expected.circle
                && actual.direction == expected.direction
                && points_equal(&actual.center, &expected.center)?
                && vectors_equal(&actual.x, &expected.x)?
                && vectors_equal(&actual.y, &expected.y)?
                && real_values_equal(&actual.x_radius, &expected.x_radius)?
                && real_values_equal(&actual.y_radius, &expected.y_radius)?
                && real_values_equal(&actual.angle_at_start, &expected.angle_at_start)?)
        }
        _ => Ok(false),
    }
}

fn real_values_equal(left: &Real, right: &Real) -> Result<bool, BuildError> {
    match compare_reals(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value == std::cmp::Ordering::Equal),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

fn insert_exact_split_parameter(
    parameters: &mut Vec<Real>,
    candidate: Real,
) -> Result<(), GeometryError> {
    let mut insertion = parameters.len();
    for (index, parameter) in parameters.iter().enumerate() {
        match decided_model_order(compare_reals(&candidate, parameter))? {
            std::cmp::Ordering::Less => {
                insertion = index;
                break;
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    parameters.insert(insertion, candidate);
    Ok(())
}

fn split_parameter_correspondence(
    source: &ParameterCorrespondence,
    pcurve: &Pcurve,
    edge_domain: &ParameterDomain,
    direction: Direction,
) -> Result<ParameterCorrespondence, BuildError> {
    match source {
        ParameterCorrespondence::AngularSweep => Ok(ParameterCorrespondence::AngularSweep),
        ParameterCorrespondence::Affine { .. } => {
            let (edge_start, edge_end) = match direction {
                Direction::Forward => (edge_domain.start(), edge_domain.end()),
                Direction::Reversed => (edge_domain.end(), edge_domain.start()),
            };
            let pcurve_width = pcurve.domain_end() - pcurve.domain_start();
            let scale = ((edge_end - edge_start) / pcurve_width)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let offset = edge_start - &scale * pcurve.domain_start();
            ParameterCorrespondence::affine(scale, offset)
        }
    }
}

fn exact_real_min_max(values: &[Real]) -> Result<(Real, Real), BuildError> {
    let Some(first) = values.first() else {
        return Err(BuildError::Geometry(GeometryError::InvalidParameterDomain));
    };
    let mut min = first.clone();
    let mut max = first.clone();
    for value in values.iter().skip(1) {
        if decided_model_order(compare_reals(value, &min))? == std::cmp::Ordering::Less {
            min = value.clone();
        }
        if decided_model_order(compare_reals(value, &max))? == std::cmp::Ordering::Greater {
            max = value.clone();
        }
    }
    Ok((min, max))
}

fn insert_sorted_real(values: &mut Vec<Real>, value: &Real) -> Result<(), BuildError> {
    for index in 0..values.len() {
        match decided_model_order(compare_reals(value, &values[index]))? {
            std::cmp::Ordering::Less => {
                values.insert(index, value.clone());
                return Ok(());
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    values.push(value.clone());
    Ok(())
}

fn exact_real_index(values: &[Real], value: &Real) -> Result<usize, BuildError> {
    for (index, candidate) in values.iter().enumerate() {
        if real_values_equal(candidate, value)? {
            return Ok(index);
        }
    }
    unreachable!("value was inserted into the exact grid")
}

fn certified_periodic_longitudinal_half_coverage(
    u_values: &[Real],
    v_values: &[Real],
    cells: &HashSet<(usize, usize)>,
    x: &Vector3,
    y: &Vector3,
    interior_normal: &Vector3,
) -> Result<bool, BuildError> {
    if u_values.len() < 2 || v_values.len() < 2 {
        return Ok(false);
    }
    let angular_span = u_values.last().expect("nonempty torus u grid") - &u_values[0];
    if decided_model_order(compare_reals(&angular_span, &Real::tau()))?
        == std::cmp::Ordering::Greater
    {
        return Ok(false);
    }

    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let signed_radial = |angle: &Real| {
        let radial = x.clone() * angle.clone().cos() + y.clone() * angle.clone().sin();
        radial.dot(interior_normal)
    };
    let mut selected_span = Real::zero();
    for u_cell in 0..u_values.len() - 1 {
        let width = &u_values[u_cell + 1] - &u_values[u_cell];
        if decided_model_order(compare_reals(&width, &Real::zero()))? != std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
        let complete = (0..v_values.len() - 1).all(|v_cell| cells.contains(&(u_cell, v_cell)));
        let partial = (0..v_values.len() - 1).any(|v_cell| cells.contains(&(u_cell, v_cell)));
        if partial && !complete {
            return Ok(false);
        }
        let midpoint = ((&u_values[u_cell] + &u_values[u_cell + 1]) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let start_order = decided_model_order(compare_reals(
            &signed_radial(&u_values[u_cell]),
            &Real::zero(),
        ))?;
        let midpoint_order =
            decided_model_order(compare_reals(&signed_radial(&midpoint), &Real::zero()))?;
        let end_order = decided_model_order(compare_reals(
            &signed_radial(&u_values[u_cell + 1]),
            &Real::zero(),
        ))?;
        if complete {
            if decided_model_order(compare_reals(&width, &quarter))? == std::cmp::Ordering::Greater
                || start_order == std::cmp::Ordering::Less
                || midpoint_order != std::cmp::Ordering::Greater
                || end_order == std::cmp::Ordering::Less
            {
                return Ok(false);
            }
            selected_span += width;
        } else {
            if decided_model_order(compare_reals(&width, &Real::pi()))?
                == std::cmp::Ordering::Greater
                || start_order == std::cmp::Ordering::Greater
                || midpoint_order != std::cmp::Ordering::Less
                || end_order == std::cmp::Ordering::Greater
            {
                return Ok(false);
            }
        }
    }
    if !real_values_equal(&selected_span, &Real::pi())? {
        return Ok(false);
    }

    if decided_model_order(compare_reals(&angular_span, &Real::tau()))? == std::cmp::Ordering::Less
    {
        let complement_start = u_values.last().expect("nonempty torus u grid");
        let complement_end = &u_values[0] + Real::tau();
        let midpoint = ((complement_start + &complement_end) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_model_order(compare_reals(
            &signed_radial(complement_start),
            &Real::zero(),
        ))? == std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(&signed_radial(&midpoint), &Real::zero()))?
                != std::cmp::Ordering::Less
            || decided_model_order(compare_reals(
                &signed_radial(&complement_end),
                &Real::zero(),
            ))? == std::cmp::Ordering::Greater
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn require_real_equal(left: &Real, right: &Real, mismatch: BuildError) -> Result<(), BuildError> {
    match compare_reals(left, right) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Equal,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(mismatch),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

fn require_vector_equal(
    left: &Vector3,
    right: &Vector3,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    if vectors_equal(left, right)? {
        Ok(())
    } else {
        Err(mismatch)
    }
}

fn require_point_equal(
    left: &Point3,
    right: &Point3,
    mismatch: BuildError,
) -> Result<(), BuildError> {
    match point3_equal(left, right) {
        PredicateOutcome::Decided { value: true, .. } => Ok(()),
        PredicateOutcome::Decided { value: false, .. } => Err(mismatch),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                needed,
                stage,
            }))
        }
    }
}

fn sphere_volume(radius: &Real) -> Result<Real, GeometryError> {
    (Real::from(4) * Real::pi() * radius * radius * radius / Real::from(3))
        .map_err(|_| GeometryError::ProjectiveDivision)
}

fn sphere_finite_cylinder_overlap_volume(
    sphere: &CertifiedSphereShell,
    cylinder: &CertifiedSphereFiniteCylinderRegion,
) -> Result<Real, GeometryError> {
    let center_parameter = (&sphere.center - &cylinder.origin).dot(&cylinder.axis);
    let relative_min = &cylinder.v_min - &center_parameter;
    let relative_max = &cylinder.v_max - center_parameter;
    let maximum = |first: &Real, second: &Real| -> Result<Real, GeometryError> {
        Ok(
            if decided_model_order(compare_reals(first, second))? == std::cmp::Ordering::Less {
                second.clone()
            } else {
                first.clone()
            },
        )
    };
    let minimum = |first: &Real, second: &Real| -> Result<Real, GeometryError> {
        Ok(
            if decided_model_order(compare_reals(first, second))? == std::cmp::Ordering::Greater {
                second.clone()
            } else {
                first.clone()
            },
        )
    };
    let lower = maximum(&relative_min, &-sphere.radius.clone())?;
    let upper = minimum(&relative_max, &sphere.radius)?;
    if decided_model_order(compare_reals(&lower, &upper))? != std::cmp::Ordering::Less {
        return Ok(Real::zero());
    }
    let half_height = (&sphere.radius * &sphere.radius - &cylinder.radius * &cylinder.radius)
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let sphere_primitive = |height: &Real| -> Result<Real, GeometryError> {
        let cubic = (height * height * height / Real::from(3))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        Ok(Real::pi() * (&sphere.radius * &sphere.radius * height - cubic))
    };
    let cylinder_primitive =
        |height: &Real| Real::pi() * &cylinder.radius * &cylinder.radius * height;
    let mut volume = Real::zero();
    let lower_cap_end = minimum(&upper, &-half_height.clone())?;
    if decided_model_order(compare_reals(&lower, &lower_cap_end))? == std::cmp::Ordering::Less {
        volume += sphere_primitive(&lower_cap_end)? - sphere_primitive(&lower)?;
    }
    let core_start = maximum(&lower, &-half_height.clone())?;
    let core_end = minimum(&upper, &half_height)?;
    if decided_model_order(compare_reals(&core_start, &core_end))? == std::cmp::Ordering::Less {
        volume += cylinder_primitive(&core_end) - cylinder_primitive(&core_start);
    }
    let upper_cap_start = maximum(&lower, &half_height)?;
    if decided_model_order(compare_reals(&upper_cap_start, &upper))? == std::cmp::Ordering::Less {
        volume += sphere_primitive(&upper)? - sphere_primitive(&upper_cap_start)?;
    }
    Ok(volume)
}

fn sphere_pair_intersection_volume(pair: &CertifiedSpherePairShell) -> Result<Real, GeometryError> {
    let distance = (&pair.second_center - &pair.first_center)
        .norm_squared()
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let radius_sum = &pair.first_radius + &pair.second_radius;
    let radius_difference = &pair.first_radius - &pair.second_radius;
    let overlap_depth = &radius_sum - &distance;
    let bracket = &distance * &distance + Real::from(2) * &distance * &radius_sum
        - Real::from(3) * &radius_difference * &radius_difference;
    (Real::pi() * &overlap_depth * overlap_depth * bracket / (Real::from(12) * distance))
        .map_err(|_| GeometryError::ProjectiveDivision)
}

fn compute_bounds(
    vertices: &[Vertex],
    curves: &[Curve3],
    spheres: &[Option<CertifiedSphereShell>],
    sphere_pairs: &[Option<CertifiedSpherePairShell>],
) -> Result<Option<Aabb>, GeometryError> {
    let mut bounds = vertices
        .first()
        .map(|first| (first.point.clone(), first.point.clone()));
    for vertex in vertices.iter().skip(1) {
        let (mins, maxs) = bounds.as_mut().expect("first vertex initialized bounds");
        update_min(&mut mins.x, &vertex.point.x)?;
        update_min(&mut mins.y, &vertex.point.y)?;
        update_min(&mut mins.z, &vertex.point.z)?;
        update_max(&mut maxs.x, &vertex.point.x)?;
        update_max(&mut maxs.y, &vertex.point.y)?;
        update_max(&mut maxs.z, &vertex.point.z)?;
    }
    for curve in curves {
        let curve_bounds = curve.bounds()?;
        let (mins, maxs) =
            bounds.get_or_insert_with(|| (curve_bounds.mins.clone(), curve_bounds.maxs.clone()));
        update_min(&mut mins.x, &curve_bounds.mins.x)?;
        update_min(&mut mins.y, &curve_bounds.mins.y)?;
        update_min(&mut mins.z, &curve_bounds.mins.z)?;
        update_max(&mut maxs.x, &curve_bounds.maxs.x)?;
        update_max(&mut maxs.y, &curve_bounds.maxs.y)?;
        update_max(&mut maxs.z, &curve_bounds.maxs.z)?;
    }
    for sphere in spheres.iter().flatten() {
        union_sphere_bounds(&mut bounds, &sphere.center, &sphere.radius)?;
    }
    for pair in sphere_pairs.iter().flatten() {
        union_sphere_bounds(&mut bounds, &pair.first_center, &pair.first_radius)?;
        union_sphere_bounds(&mut bounds, &pair.second_center, &pair.second_radius)?;
    }
    Ok(bounds.map(|(mins, maxs)| Aabb::new(mins, maxs)))
}

fn union_sphere_bounds(
    bounds: &mut Option<(Point3, Point3)>,
    center: &Point3,
    radius: &Real,
) -> Result<(), GeometryError> {
    let sphere_min = Point3::new(&center.x - radius, &center.y - radius, &center.z - radius);
    let sphere_max = Point3::new(&center.x + radius, &center.y + radius, &center.z + radius);
    let (mins, maxs) = bounds.get_or_insert_with(|| (sphere_min.clone(), sphere_max.clone()));
    update_min(&mut mins.x, &sphere_min.x)?;
    update_min(&mut mins.y, &sphere_min.y)?;
    update_min(&mut mins.z, &sphere_min.z)?;
    update_max(&mut maxs.x, &sphere_max.x)?;
    update_max(&mut maxs.y, &sphere_max.y)?;
    update_max(&mut maxs.z, &sphere_max.z)?;
    Ok(())
}

fn update_min(current: &mut Real, candidate: &Real) -> Result<(), GeometryError> {
    if decided_model_order(compare_reals(candidate, current))? == std::cmp::Ordering::Less {
        *current = candidate.clone();
    }
    Ok(())
}

fn update_max(current: &mut Real, candidate: &Real) -> Result<(), GeometryError> {
    if decided_model_order(compare_reals(candidate, current))? == std::cmp::Ordering::Greater {
        *current = candidate.clone();
    }
    Ok(())
}

fn decided_model_order(
    outcome: PredicateOutcome<std::cmp::Ordering>,
) -> Result<std::cmp::Ordering, GeometryError> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

fn exact_point_order(left: &Point3, right: &Point3) -> Result<std::cmp::Ordering, GeometryError> {
    match compare_point3_lexicographic(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

pub(crate) fn compare_curve3_exact_data(
    left: &Curve3ExactData,
    right: &Curve3ExactData,
) -> Result<std::cmp::Ordering, GeometryError> {
    let rank = |data: &Curve3ExactData| match data {
        Curve3ExactData::Line(_) => 0_u8,
        Curve3ExactData::RationalBezier { .. } => 1,
        Curve3ExactData::Nurbs { .. } => 2,
        Curve3ExactData::EllipseArc(data) if data.circle => 3,
        Curve3ExactData::EllipseArc(_) => 4,
    };
    let family = rank(left).cmp(&rank(right));
    if family != std::cmp::Ordering::Equal {
        return Ok(family);
    }
    match (left, right) {
        (Curve3ExactData::Line(left), Curve3ExactData::Line(right)) => {
            compare_point_pairs(&left.start, &left.end, &right.start, &right.end)
        }
        (
            Curve3ExactData::RationalBezier {
                control_points: left_points,
                weights: left_weights,
            },
            Curve3ExactData::RationalBezier {
                control_points: right_points,
                weights: right_weights,
            },
        ) => {
            let controls = compare_point3_slices(left_points, right_points)?;
            if controls != std::cmp::Ordering::Equal {
                return Ok(controls);
            }
            compare_real_slices(left_weights, right_weights)
        }
        (
            Curve3ExactData::Nurbs {
                degree: left_degree,
                control_points: left_points,
                weights: left_weights,
                knots: left_knots,
            },
            Curve3ExactData::Nurbs {
                degree: right_degree,
                control_points: right_points,
                weights: right_weights,
                knots: right_knots,
            },
        ) => {
            let degree = left_degree.cmp(right_degree);
            if degree != std::cmp::Ordering::Equal {
                return Ok(degree);
            }
            let controls = compare_point3_slices(left_points, right_points)?;
            if controls != std::cmp::Ordering::Equal {
                return Ok(controls);
            }
            let weights = compare_real_slices(left_weights, right_weights)?;
            if weights != std::cmp::Ordering::Equal {
                return Ok(weights);
            }
            compare_real_slices(left_knots, right_knots)
        }
        (Curve3ExactData::EllipseArc(left), Curve3ExactData::EllipseArc(right)) => {
            let center = exact_point_order(&left.center, &right.center)?;
            if center != std::cmp::Ordering::Equal {
                return Ok(center);
            }
            let x = compare_vector3(&left.x, &right.x)?;
            if x != std::cmp::Ordering::Equal {
                return Ok(x);
            }
            let y = compare_vector3(&left.y, &right.y)?;
            if y != std::cmp::Ordering::Equal {
                return Ok(y);
            }
            for (left_value, right_value) in [
                (&left.x_radius, &right.x_radius),
                (&left.y_radius, &right.y_radius),
                (&left.domain_start, &right.domain_start),
                (&left.domain_end, &right.domain_end),
                (&left.angle_at_start, &right.angle_at_start),
            ] {
                let order = decided_model_order(compare_reals(left_value, right_value))?;
                if order != std::cmp::Ordering::Equal {
                    return Ok(order);
                }
            }
            Ok(left.direction.cmp(&right.direction))
        }
        _ => unreachable!("equal exact-curve family ranks have matching variants"),
    }
}

fn compare_point_pairs(
    left_first: &Point3,
    left_second: &Point3,
    right_first: &Point3,
    right_second: &Point3,
) -> Result<std::cmp::Ordering, GeometryError> {
    let first = exact_point_order(left_first, right_first)?;
    if first != std::cmp::Ordering::Equal {
        return Ok(first);
    }
    exact_point_order(left_second, right_second)
}

fn compare_point3_slices(
    left: &[Point3],
    right: &[Point3],
) -> Result<std::cmp::Ordering, GeometryError> {
    for (left_point, right_point) in left.iter().zip(right) {
        let order = exact_point_order(left_point, right_point)?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(left.len().cmp(&right.len()))
}

fn compare_real_slices(left: &[Real], right: &[Real]) -> Result<std::cmp::Ordering, GeometryError> {
    for (left_value, right_value) in left.iter().zip(right) {
        let order = decided_model_order(compare_reals(left_value, right_value))?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(left.len().cmp(&right.len()))
}

fn compare_vector3(left: &Vector3, right: &Vector3) -> Result<std::cmp::Ordering, GeometryError> {
    for index in 0..3 {
        let order = decided_model_order(compare_reals(&left[index], &right[index]))?;
        if order != std::cmp::Ordering::Equal {
            return Ok(order);
        }
    }
    Ok(std::cmp::Ordering::Equal)
}

fn compare_ordered_face_split_traces(
    left: &OrderedFaceSplitTrace,
    right: &OrderedFaceSplitTrace,
) -> Result<std::cmp::Ordering, GeometryError> {
    let lower = exact_point_order(&left.lower, &right.lower)?;
    if lower != std::cmp::Ordering::Equal {
        return Ok(lower);
    }
    exact_point_order(&left.upper, &right.upper)
}

fn compare_ordered_surface_curve_traces(
    left: &OrderedSurfaceCurveTrace,
    right: &OrderedSurfaceCurveTrace,
) -> Result<std::cmp::Ordering, GeometryError> {
    let lower = exact_point_order(&left.lower, &right.lower)?;
    if lower != std::cmp::Ordering::Equal {
        return Ok(lower);
    }
    let upper = exact_point_order(&left.upper, &right.upper)?;
    if upper != std::cmp::Ordering::Equal {
        return Ok(upper);
    }
    compare_curve3_exact_data(&left.exact_key.exact_data(), &right.exact_key.exact_data())
}

fn projected_face_split_line(
    surface: &Surface,
    trace: &OrderedFaceSplitTrace,
) -> Result<LineSeg2, TopologyEditError> {
    let project = |point: &Point3| {
        let origin = surface
            .plane_origin()
            .expect("face-split projection prevalidates a plane");
        let (u, v) = surface
            .plane_directions()
            .expect("face-split projection prevalidates a plane");
        let displacement = point - origin;
        let uu = u.dot(u);
        let uv = u.dot(v);
        let vv = v.dot(v);
        let du = displacement.dot(u);
        let dv = displacement.dot(v);
        let determinant = &uu * &vv - &uv * &uv;
        Ok::<_, GeometryError>(CurvePoint2::new(
            ((&du * &vv - &dv * &uv) / &determinant)
                .map_err(|_| GeometryError::ProjectiveDivision)?,
            ((&dv * &uu - &du * &uv) / determinant)
                .map_err(|_| GeometryError::ProjectiveDivision)?,
        ))
    };
    LineSeg2::try_new(project(&trace.lower)?, project(&trace.upper)?)
        .map_err(GeometryError::from)
        .map_err(TopologyEditError::from)
}

fn arranged_face_split_segments(
    trace: &OrderedFaceSplitTrace,
    planar: &LineSeg2,
    prior_lines: &[(usize, LineSeg2)],
) -> Result<Vec<Curve3>, TopologyEditError> {
    let policy = CurvePolicy::certified();
    let mut cuts = Vec::new();
    for (prior_source, prior) in prior_lines {
        match planar
            .intersect_line(prior, &policy)
            .map_err(GeometryError::from)?
        {
            LineLineIntersection::None => {}
            LineLineIntersection::Point { a_param, .. } => {
                insert_face_split_cut(&mut cuts, a_param)?;
            }
            LineLineIntersection::Overlap { .. } => {
                return Err(TopologyEditError::OverlappingFaceSplitTraces {
                    first: *prior_source,
                    second: trace.source_index,
                });
            }
            LineLineIntersection::Uncertain { reason } => {
                return Err(GeometryError::PlanarClassificationUnresolved(reason).into());
            }
        }
    }

    let mut boundaries = Vec::with_capacity(cuts.len() + 2);
    boundaries.push(Real::zero());
    boundaries.extend(cuts);
    boundaries.push(Real::one());
    boundaries
        .windows(2)
        .map(|range| {
            Curve3::line(
                trace.curve.point_at(&range[0])?,
                trace.curve.point_at(&range[1])?,
            )
            .map_err(TopologyEditError::from)
        })
        .collect()
}

fn insert_face_split_cut(cuts: &mut Vec<Real>, parameter: Real) -> Result<(), TopologyEditError> {
    if decided_model_order(compare_reals(&parameter, &Real::zero()))? != std::cmp::Ordering::Greater
        || decided_model_order(compare_reals(&parameter, &Real::one()))? != std::cmp::Ordering::Less
    {
        return Ok(());
    }
    for index in 0..cuts.len() {
        match decided_model_order(compare_reals(&parameter, &cuts[index]))? {
            std::cmp::Ordering::Less => {
                cuts.insert(index, parameter);
                return Ok(());
            }
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Greater => {}
        }
    }
    cuts.push(parameter);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hypercurve::{CircularArc2, Curve2, LineSeg2, Point2 as CurvePoint2};
    use hyperlattice::Vector3;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn uv(x: i32, y: i32) -> CurvePoint2 {
        CurvePoint2::new(r(x), r(y))
    }

    fn add_planar_triangle(builder: &mut ModelBuilder) -> (FaceId, [EdgeId; 3]) {
        let vertices = [
            builder.vertex(p(0, 0, 0)).unwrap(),
            builder.vertex(p(2, 0, 0)).unwrap(),
            builder.vertex(p(0, 3, 0)).unwrap(),
        ];
        let model_points = [p(0, 0, 0), p(2, 0, 0), p(0, 3, 0)];
        let parameter_points = [uv(0, 0), uv(2, 0), uv(0, 3)];
        let mut edges = Vec::new();
        let mut edge_uses = Vec::new();
        for index in 0..3 {
            let next = (index + 1) % 3;
            let curve = builder
                .curve(
                    Curve3::line(model_points[index].clone(), model_points[next].clone()).unwrap(),
                )
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::unit(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        parameter_points[index].clone(),
                        parameter_points[next].clone(),
                    )
                    .unwrap(),
                )))
                .unwrap();
            let edge_use = builder
                .edge_use(
                    edge,
                    Direction::Forward,
                    pcurve,
                    ParameterCorrespondence::identity(),
                )
                .unwrap();
            edges.push(edge);
            edge_uses.push(edge_use);
        }
        let wire = builder.wire(edge_uses).unwrap();
        let surface = builder
            .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        (face, edges.try_into().unwrap())
    }

    fn planar_circle_model() -> (Model, [EdgeUseId; 4]) {
        let mut builder = ModelBuilder::new();
        let points = [p(2, 0, 0), p(0, 2, 0), p(-2, 0, 0), p(0, -2, 0)];
        let parameters = [uv(2, 0), uv(0, 2), uv(-2, 0), uv(0, -2)];
        let vertices = points
            .iter()
            .cloned()
            .map(|point| builder.vertex(point).unwrap())
            .collect::<Vec<_>>();
        let half_pi = (Real::pi() / r(2)).unwrap();
        let mut uses = Vec::new();
        for index in 0..4 {
            let next = (index + 1) % 4;
            let start_angle = &half_pi * r(index as i32);
            let end_angle = &half_pi * r(index as i32 + 1);
            let curve = builder
                .curve(
                    Curve3::circle_arc(
                        p(0, 0, 0),
                        Vector3::x(),
                        Vector3::y(),
                        r(2),
                        start_angle.clone(),
                        end_angle.clone(),
                    )
                    .unwrap(),
                )
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::new(start_angle, end_angle).unwrap(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    CircularArc2::try_from_center(
                        parameters[index].clone(),
                        parameters[next].clone(),
                        uv(0, 0),
                        false,
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::angular_sweep(),
                    )
                    .unwrap(),
            );
        }
        let use_ids: [EdgeUseId; 4] = uses.clone().try_into().unwrap();
        let wire = builder.wire(uses).unwrap();
        let surface = builder
            .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        builder.shell(vec![face]).unwrap();
        (builder.finish().unwrap(), use_ids)
    }

    #[test]
    fn angular_sweep_correspondence_certifies_a_complete_planar_circle() {
        let (model, uses) = planar_circle_model();
        let half = (Real::one() / r(2)).unwrap();
        let first_parameter = model.edge_parameter_at(uses[0], &half).unwrap();
        let first_use = model.edge_use(uses[0]).unwrap();
        let edge = model.edge(first_use.edge()).unwrap();
        let spatial = model
            .curve(edge.curve())
            .unwrap()
            .point_at(&first_parameter)
            .unwrap();
        let face = model.faces().next().unwrap().1;
        let surface_parameter = model
            .pcurve(first_use.pcurve())
            .unwrap()
            .point_at(&half)
            .unwrap();
        let from_face = model
            .surface(face.surface())
            .unwrap()
            .point_at(&surface_parameter)
            .unwrap();
        assert_eq!(point3_equal(&spatial, &from_face).value(), Some(true));
        assert!(matches!(
            model.edge_use(uses[0]).unwrap().parameter_correspondence(),
            ParameterCorrespondence::AngularSweep
        ));

        let json = model.to_json().unwrap();
        assert!(json.contains("\"AngularSweep\""));
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.edge_parameter_at(uses[0], &half).unwrap(),
                &first_parameter,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_patch_certifies_complete_meridian_and_latitude_images() {
        let profile = Curve3::line(p(2, 0, 0), p(3, 0, 1)).unwrap();
        let surface = Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap();
        let quarter = (Real::pi() / Real::from(2)).unwrap();
        let parameters = [
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(quarter.clone(), Real::zero()),
            Point2::new(quarter.clone(), Real::one()),
            Point2::new(Real::zero(), Real::one()),
        ];
        let points = parameters
            .iter()
            .map(|parameter| surface.point_at(parameter).unwrap())
            .collect::<Vec<_>>();
        let mut builder = ModelBuilder::new();
        let vertices = points
            .iter()
            .cloned()
            .map(|point| builder.vertex(point).unwrap())
            .collect::<Vec<_>>();
        let curves = [
            Curve3::circle_arc(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::y(),
                r(2),
                Real::zero(),
                quarter.clone(),
            )
            .unwrap(),
            Curve3::line(points[1].clone(), points[2].clone()).unwrap(),
            Curve3::circle_arc(
                p(0, 0, 1),
                Vector3::x(),
                Vector3::y(),
                r(3),
                Real::zero(),
                quarter.clone(),
            )
            .unwrap(),
            Curve3::line(points[0].clone(), points[3].clone()).unwrap(),
        ];
        let domains = [
            ParameterDomain::new(Real::zero(), quarter.clone()).unwrap(),
            ParameterDomain::unit(),
            ParameterDomain::new(Real::zero(), quarter.clone()).unwrap(),
            ParameterDomain::unit(),
        ];
        let endpoints = [(0, 1), (1, 2), (3, 2), (0, 3)];
        let edges = curves
            .into_iter()
            .zip(domains)
            .zip(endpoints)
            .map(|((curve, domain), (start, end))| {
                let curve = builder.curve(curve).unwrap();
                builder
                    .edge(vertices[start], vertices[end], curve, domain)
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let pcurve_points = [
            (
                CurvePoint2::new(Real::zero(), Real::zero()),
                CurvePoint2::new(quarter.clone(), Real::zero()),
            ),
            (
                CurvePoint2::new(quarter.clone(), Real::zero()),
                CurvePoint2::new(quarter.clone(), Real::one()),
            ),
            (
                CurvePoint2::new(quarter.clone(), Real::one()),
                CurvePoint2::new(Real::zero(), Real::one()),
            ),
            (
                CurvePoint2::new(Real::zero(), Real::one()),
                CurvePoint2::new(Real::zero(), Real::zero()),
            ),
        ];
        let directions = [
            Direction::Forward,
            Direction::Forward,
            Direction::Reversed,
            Direction::Reversed,
        ];
        let correspondences = [
            ParameterCorrespondence::affine(quarter.clone(), Real::zero()).unwrap(),
            ParameterCorrespondence::identity(),
            ParameterCorrespondence::affine(-quarter.clone(), quarter).unwrap(),
            ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap(),
        ];
        let uses = edges
            .into_iter()
            .zip(pcurve_points)
            .zip(directions)
            .zip(correspondences)
            .map(|(((edge, (start, end)), direction), correspondence)| {
                let pcurve = builder
                    .pcurve(Pcurve::new(Curve2::from(
                        LineSeg2::try_new(start, end).unwrap(),
                    )))
                    .unwrap();
                builder
                    .edge_use(edge, direction, pcurve, correspondence)
                    .unwrap()
            })
            .collect();
        let wire = builder.wire(uses).unwrap();
        let surface = builder.surface(surface).unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        builder.shell(vec![face]).unwrap();
        builder.finish().unwrap();
    }

    #[test]
    fn analytic_image_certificate_rejects_a_sheared_arc_between_matching_endpoints() {
        let mut builder = ModelBuilder::new();
        let vertices = [
            builder.vertex(p(1, 0, 0)).unwrap(),
            builder.vertex(p(-1, 0, 0)).unwrap(),
        ];
        let angle_domains = [(Real::zero(), Real::pi()), (Real::pi(), Real::tau())];
        let parameter_points = [(uv(1, 0), uv(-1, 0)), (uv(-1, 0), uv(1, 0))];
        let mut uses = Vec::new();
        for index in 0..2 {
            let (start_angle, end_angle) = &angle_domains[index];
            let curve = builder
                .curve(
                    Curve3::circle_arc(
                        p(0, 0, 0),
                        Vector3::x(),
                        Vector3::y(),
                        Real::one(),
                        start_angle.clone(),
                        end_angle.clone(),
                    )
                    .unwrap(),
                )
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[(index + 1) % 2],
                    curve,
                    ParameterDomain::new(start_angle.clone(), end_angle.clone()).unwrap(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    CircularArc2::try_from_center(
                        parameter_points[index].0.clone(),
                        parameter_points[index].1.clone(),
                        uv(0, 0),
                        false,
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::angular_sweep(),
                    )
                    .unwrap(),
            );
        }
        let wire = builder.wire(uses).unwrap();
        let sheared_plane = builder
            .surface(
                Surface::plane(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::from_xyz(Real::one(), Real::one(), Real::zero()),
                )
                .unwrap(),
            )
            .unwrap();
        assert_eq!(
            builder.face(sheared_plane, Orientation::Forward, wire, Vec::new()),
            Err(BuildError::EdgeUseSupportMismatch)
        );
    }

    fn add_planar_wire(builder: &mut ModelBuilder, coordinates: &[(i32, i32)]) -> WireId {
        let vertices = coordinates
            .iter()
            .map(|(x, y)| builder.vertex(p(*x, *y, 0)).unwrap())
            .collect::<Vec<_>>();
        let mut uses = Vec::with_capacity(coordinates.len());
        for index in 0..coordinates.len() {
            let next = (index + 1) % coordinates.len();
            let (x0, y0) = coordinates[index];
            let (x1, y1) = coordinates[next];
            let curve = builder
                .curve(Curve3::line(p(x0, y0, 0), p(x1, y1, 0)).unwrap())
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::unit(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(uv(x0, y0), uv(x1, y1)).unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::identity(),
                    )
                    .unwrap(),
            );
        }
        builder.wire(uses).unwrap()
    }

    #[test]
    fn builder_commits_private_typed_arenas_with_retained_adjacency() {
        let mut builder = ModelBuilder::new();
        let (face, edges) = add_planar_triangle(&mut builder);
        let shell = builder.shell(vec![face]).unwrap();
        let model = builder.finish().unwrap();

        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 3,
                curves: 3,
                pcurves: 3,
                surfaces: 1,
                edges: 3,
                edge_uses: 3,
                wires: 1,
                faces: 1,
                shells: 1,
                solids: 0,
            }
        );
        assert_eq!(model.shell(shell).unwrap().faces(), &[face]);
        assert_eq!(model.uses_of_edge(edges[0]).unwrap().len(), 1);
        assert_eq!(
            model
                .edges_at_vertex(model.edge(edges[0]).unwrap().start())
                .unwrap()
                .len(),
            2
        );
    }

    #[test]
    fn builder_rejects_topologically_disconnected_wires() {
        let mut builder = ModelBuilder::new();
        let vertices = [
            builder.vertex(p(0, 0, 0)).unwrap(),
            builder.vertex(p(1, 0, 0)).unwrap(),
            builder.vertex(p(2, 0, 0)).unwrap(),
        ];
        let mut uses = Vec::new();
        for (start, end) in [(0, 1), (2, 1)] {
            let curve = builder
                .curve(Curve3::line(p(start, 0, 0), p(end, 0, 0)).unwrap())
                .unwrap();
            let edge = builder
                .edge(
                    vertices[start as usize],
                    vertices[end as usize],
                    curve,
                    ParameterDomain::unit(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(uv(start, 0), uv(end, 0)).unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::identity(),
                    )
                    .unwrap(),
            );
        }
        assert_eq!(
            builder.wire(uses),
            Err(BuildError::DisconnectedWire { at: 0 })
        );
    }

    #[test]
    fn finish_reports_orphan_topology_instead_of_publishing_invalid_model() {
        let mut builder = ModelBuilder::new();
        let start = builder.vertex(p(0, 0, 0)).unwrap();
        let end = builder.vertex(p(1, 0, 0)).unwrap();
        let curve = builder
            .curve(Curve3::line(p(0, 0, 0), p(1, 0, 0)).unwrap())
            .unwrap();
        let edge = builder
            .edge(start, end, curve, ParameterDomain::unit())
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(uv(0, 0), uv(1, 0)).unwrap(),
            )))
            .unwrap();
        let edge_use = builder
            .edge_use(
                edge,
                Direction::Forward,
                pcurve,
                ParameterCorrespondence::identity(),
            )
            .unwrap();

        let report = builder.finish().unwrap_err();
        assert_eq!(report.errors(), &[BuildError::OrphanEdgeUse(edge_use)]);
    }

    #[test]
    fn rational_line_image_certificate_rejects_collinear_backtracking() {
        let monotone = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 0), p(3, 0, 0)],
            vec![Real::one(), r(2), r(5)],
        )
        .unwrap();
        assert!(certified_monotone_line_curve_image(&monotone).unwrap());

        let backtracking = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(3, 0, 0), p(2, 0, 0)],
            vec![Real::one(), r(2), r(5)],
        )
        .unwrap();
        assert!(!certified_monotone_line_curve_image(&backtracking).unwrap());
    }

    fn extrusion_rectangle(profile: Curve3, direction: Vector3, height: Real) -> (Model, FaceId) {
        let mut builder = ModelBuilder::new();
        let offset = direction.clone() * &height;
        let start = profile.start().unwrap();
        let end = profile.end().unwrap();
        let top_start = start.clone() + &offset;
        let top_end = end.clone() + &offset;
        let vertices = [
            builder.vertex(start.clone()).unwrap(),
            builder.vertex(end.clone()).unwrap(),
            builder.vertex(top_end.clone()).unwrap(),
            builder.vertex(top_start.clone()).unwrap(),
        ];
        let profile_domain = profile.domain().clone();
        let top_profile = translated_curve(&profile, &offset).unwrap();
        let curves = [
            builder.curve(profile.clone()).unwrap(),
            builder.curve(Curve3::line(end, top_end).unwrap()).unwrap(),
            builder.curve(top_profile).unwrap(),
            builder
                .curve(Curve3::line(start, top_start).unwrap())
                .unwrap(),
        ];
        let edges = [
            builder
                .edge(vertices[0], vertices[1], curves[0], profile_domain.clone())
                .unwrap(),
            builder
                .edge(vertices[1], vertices[2], curves[1], ParameterDomain::unit())
                .unwrap(),
            builder
                .edge(vertices[3], vertices[2], curves[2], profile_domain.clone())
                .unwrap(),
            builder
                .edge(vertices[0], vertices[3], curves[3], ParameterDomain::unit())
                .unwrap(),
        ];
        let u_start = profile_domain.start().clone();
        let u_end = profile_domain.end().clone();
        let pcurve_points = [
            (
                CurvePoint2::new(u_start.clone(), Real::zero()),
                CurvePoint2::new(u_end.clone(), Real::zero()),
            ),
            (
                CurvePoint2::new(u_end.clone(), Real::zero()),
                CurvePoint2::new(u_end.clone(), height.clone()),
            ),
            (
                CurvePoint2::new(u_end.clone(), height.clone()),
                CurvePoint2::new(u_start.clone(), height.clone()),
            ),
            (
                CurvePoint2::new(u_start.clone(), height),
                CurvePoint2::new(u_start.clone(), Real::zero()),
            ),
        ];
        let profile_span = &u_end - &u_start;
        let correspondences = [
            ParameterCorrespondence::affine(profile_span.clone(), u_start.clone()).unwrap(),
            ParameterCorrespondence::identity(),
            ParameterCorrespondence::affine(-profile_span, u_end).unwrap(),
            ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap(),
        ];
        let directions = [
            Direction::Forward,
            Direction::Forward,
            Direction::Reversed,
            Direction::Reversed,
        ];
        let mut uses = Vec::with_capacity(4);
        for index in 0..4 {
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        pcurve_points[index].0.clone(),
                        pcurve_points[index].1.clone(),
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edges[index],
                        directions[index],
                        pcurve,
                        correspondences[index].clone(),
                    )
                    .unwrap(),
            );
        }
        let wire = builder.wire(uses).unwrap();
        let surface = builder
            .surface(Surface::extrusion(profile, direction).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        builder.shell(vec![face]).unwrap();
        (builder.finish().unwrap(), face)
    }

    #[test]
    fn extrusion_area_certifies_rational_line_images_and_rejects_variable_speed_profiles() {
        let profiles = [
            Curve3::rational_bezier(
                vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
                vec![Real::one(), r(2), r(3)],
            )
            .unwrap(),
            Curve3::nurbs(
                2,
                vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
                vec![Real::one(), r(2), r(3)],
                vec![r(2), r(2), r(2), r(5), r(5), r(5)],
            )
            .unwrap(),
        ];
        for profile in profiles {
            let (model, face) = extrusion_rectangle(profile, Vector3::z(), r(3));
            let area = model.face_area(face).unwrap();
            assert_eq!(
                compare_reals(&area, &r(6)).value(),
                Some(std::cmp::Ordering::Equal)
            );
            let replayed = crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(
                compare_reals(&replayed.face_area(face).unwrap(), &r(6)).value(),
                Some(std::cmp::Ordering::Equal)
            );
        }

        let curved = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 1, 0), p(2, 0, 0)],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap();
        let (model, face) = extrusion_rectangle(curved, Vector3::z(), r(3));
        assert_eq!(
            model.face_area(face),
            Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
        );

        let circle = Curve3::circle_arc(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            (Real::pi() / r(2)).unwrap(),
        )
        .unwrap();
        let (normal_model, normal_face) = extrusion_rectangle(circle.clone(), Vector3::z(), r(3));
        assert_eq!(
            compare_reals(
                &normal_model.face_area(normal_face).unwrap(),
                &(r(3) * Real::pi()),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let (oblique_model, oblique_face) = extrusion_rectangle(
            circle,
            Vector3::from_xyz(Real::one(), Real::zero(), Real::one()),
            r(3),
        );
        assert_eq!(
            oblique_model.face_area(oblique_face),
            Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
        );
    }

    #[test]
    fn canonical_model_and_geometry_are_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}

        assert_send_sync::<Curve3>();
        assert_send_sync::<Pcurve>();
        assert_send_sync::<Surface>();
        assert_send_sync::<Model>();
    }

    #[test]
    fn planar_face_certifies_inner_wire_orientation_nesting_and_area() {
        let mut builder = ModelBuilder::new();
        let outer = add_planar_wire(&mut builder, &[(0, 0), (10, 0), (10, 10), (0, 10)]);
        let inner = add_planar_wire(&mut builder, &[(3, 3), (3, 7), (7, 7), (7, 3)]);
        let surface = builder
            .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, outer, vec![inner])
            .unwrap();
        builder.shell(vec![face]).unwrap();
        let model = builder.finish().unwrap();
        assert_eq!(
            compare_reals(&model.face_area(face).unwrap(), &Real::from(84)).value(),
            Some(std::cmp::Ordering::Equal)
        );

        let mut wrong_orientation = ModelBuilder::new();
        let outer = add_planar_wire(
            &mut wrong_orientation,
            &[(0, 0), (10, 0), (10, 10), (0, 10)],
        );
        let inner = add_planar_wire(&mut wrong_orientation, &[(3, 3), (7, 3), (7, 7), (3, 7)]);
        let surface = wrong_orientation
            .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        assert_eq!(
            wrong_orientation.face(surface, Orientation::Forward, outer, vec![inner]),
            Err(BuildError::InconsistentWireOrientation(inner))
        );

        let mut outside = ModelBuilder::new();
        let outer = add_planar_wire(&mut outside, &[(0, 0), (10, 0), (10, 10), (0, 10)]);
        let inner = add_planar_wire(&mut outside, &[(12, 2), (12, 4), (14, 4), (14, 2)]);
        let surface = outside
            .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        assert_eq!(
            outside.face(surface, Orientation::Forward, outer, vec![inner]),
            Err(BuildError::InnerWireOutside(inner))
        );
    }
}
