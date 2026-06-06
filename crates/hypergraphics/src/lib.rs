//! Hyperreal-owned 3D graphics geometry and a small glow renderer.
//!
//! This crate extracts the low-level desktop 3D renderer shape from
//! CopperForge's `render3d` module: colored meshes, an orbit camera, grid and
//! axis helpers, and an unlit OpenGL shader. The adaptation point is ownership:
//! scene geometry stays in [`Real`] / [`Point3`] form until an explicit render
//! export projects it to finite primitive values.
//!
//! `hyperlimit` remains responsible for exact geometric predicates and
//! `hypertri` remains responsible for triangulation. Rendering APIs expose only
//! lossy f64/f32 views and must not be used as topology certificates.

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
