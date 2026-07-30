//! Hyper-native exact boundary representation.
//!
//! HyperBREP owns immutable retained 3D geometry and typed-arena topology.
//! [`hyperreal::Real`] is the only authoritative scalar. Mathematical
//! decisions are delegated to certified Hyperlimit predicates; an unresolved
//! predicate is an error, never an implicit tolerance decision.
//!
//! Construction is a validation boundary: [`ModelBuilder`] rejects invalid
//! local relationships as they are staged, and [`ModelBuilder::finish`]
//! publishes an immutable [`Model`] only after global ownership checks pass.

#![warn(missing_docs)]

mod error;
mod geometry;
mod model;
mod persistence;

pub mod boolean;
pub mod builder;
#[cfg(feature = "tessellation")]
pub mod tessellation;

pub use boolean::{
    BooleanError, BooleanOperation, BooleanResult, ClassifiedFace, FacePairIntersection,
    FacePairRelation, FacePairTrim, FaceSelection, FaceSelectionAction, SolidIntersectionGraph,
    partition_contained_face_by_plane_region,
};
pub use builder::{Axis, ConstructionError, LoftSection, RationalBezierSweepFrame, TensorPatch};
pub use error::{GeometryError, GeometryResult};
pub use geometry::{
    Curve3, Curve3Kind, CurveDerivative3, CurveParameterLocation, CurveSurfaceIntersection,
    CurveSurfacePoint, IntersectionMultiplicity, MaterializedSurfacePcurve, ParameterDomain,
    Pcurve, Surface, SurfaceBounds, SurfaceDomain, SurfaceIntersectionComponents,
    SurfaceIntersectionCurve, SurfaceIntersectionLine, SurfaceIntersectionOperand,
    SurfaceIntersectionParameterRay, SurfaceIntersectionPcurve, SurfaceIntersectionRay,
    SurfaceIsoAxis, SurfaceKind, SurfaceParameterDomain, SurfacePartials,
    SurfacePcurveCorrespondence, SurfaceSurfaceIntersection,
};
pub use hyperlattice::{Aabb, Matrix4, Point2, Point3, Real, Vector2, Vector3};
pub use model::{
    BridgeCurveFaceSplit, BridgePairFaceSplit, BuildError, ClosedSurfaceCurveFaceSplit, Curve3Id,
    CurveFaceSplit, Direction, Edge, EdgeId, EdgeSplit, EdgeUse, EdgeUseId, Edit, EditError,
    Endpoint, EntityKind, Face, FaceId, FacePartition, FaceSplit, FaceTracePartition, Model,
    ModelBuilder, ModelCounts, Orientation, ParameterCorrespondence, PcurveId, QueryError, Shell,
    ShellId, Solid, SolidId, SolidPointLocation, SurfaceCurveFaceSplit, SurfaceId,
    TopologyEditError, ValidationReport, Vertex, VertexId, Wire, WireId,
};
pub use persistence::{PersistenceError, RawModel};
