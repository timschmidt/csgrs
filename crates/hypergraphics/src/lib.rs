//! Exact 3D scene geometry with an explicit primitive-float rendering boundary.
//!
//! Scene geometry stays in `Real` and `Point3` form until an explicit render
//! export projects it to finite `f64` values. Certified Hypercurve subdivision
//! retains source-chord error reports alongside exact curve, path, and
//! role-classified region line meshes. The crate also provides colored meshes,
//! an orbit camera, grid and axis helpers, and an unlit OpenGL/WebGL backend.
//!
//! The default `exact` feature enables Hyperreal-owned geometry, predicates,
//! triangulation, and camera helpers. Renderer-only consumers can disable
//! default features and retain the primitive-float projection and backend APIs.
//!
//! `hyperlimit` supplies robust geometric predicates and `hypertri` supplies
//! triangulation. Rendering APIs expose lossy `f64`/`f32` views and must not be
//! used as topology certificates.

#![warn(missing_docs)]

pub mod backend;
pub mod camera;
pub mod error;
#[cfg(feature = "exact")]
pub mod geometry;
pub mod render;
#[cfg(feature = "exact")]
pub mod scene;

#[cfg(feature = "exact")]
pub use camera::ExactCamera;
pub use camera::{ApproxPoint3, Projection64, ScreenPoint, Viewport};
pub use error::{Error, Result};
#[cfg(feature = "exact")]
pub use geometry::{ExactMesh, ExactVertex, TriangleOrientation};
#[cfg(feature = "exact")]
pub use hyperlattice::{Point3, Vector2, Vector3};
#[cfg(feature = "exact")]
pub use hyperlimit::{Certainty, Escalation, PredicatePolicy, RefinementNeed};
#[cfg(feature = "exact")]
pub use hyperreal::{Rational, Real};
#[cfg(feature = "exact")]
pub use hypertri::{Point2, TriangulationContext, TriangulationOutcome};
pub use render::{Color3, Primitive, RenderVertex64};
#[cfg(feature = "exact")]
pub use scene::{
    CertifiedCurveLineMesh, CertifiedCurveRegionLineMesh, CertifiedCurveRegionLoopLineEvidence,
    axes_mesh, curve_line_mesh, curve_path_line_mesh, curve_region_line_mesh, grid_mesh,
    polygon_surface_mesh, triangle_mesh,
};
