//! Exact 3D scene geometry with an explicit primitive-float rendering boundary.
//!
//! Scene geometry stays in [`Real`] and [`Point3`] form until an explicit render
//! export projects it to finite `f64` values. The crate also provides colored
//! meshes, an orbit camera, grid and axis helpers, and an unlit OpenGL/WebGL
//! backend.
//!
//! `hyperlimit` supplies robust geometric predicates and `hypertri` supplies
//! triangulation. Rendering APIs expose lossy `f64`/`f32` views and must not be
//! used as topology certificates.

#![warn(missing_docs)]

pub mod backend;
pub mod camera;
pub mod error;
pub mod geometry;
pub mod scene;

pub use camera::{ExactCamera, Projection64, ScreenPoint, Viewport};
pub use error::{Error, Result};
pub use geometry::{
    Color3, ExactMesh, ExactVertex, Primitive, RenderVertex64, TriangleOrientation,
};
pub use hyperlattice::{Point2, Point3, Vector2, Vector3};
pub use hyperreal::{Rational, Real};
pub use scene::{axes_mesh, grid_mesh, polygon_surface_mesh};
