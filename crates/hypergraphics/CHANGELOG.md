# Changelog

## [0.1.0] - Unreleased

Initial release.

Hypergraphics connects exact Hyper geometry to a small unlit OpenGL and WebGL
renderer. Scene positions remain `hyperreal::Real` values while geometry is
constructed, triangulated, and queried; conversion to `f64` and `f32` happens
only through named camera, export, and GPU-upload APIs. The crate does not
create windows or graphics contexts, and provides no materials or lighting.

- `ExactMesh` stores colored line or triangle vertices with exact positions.
  Scene helpers include `axes_mesh`, `grid_mesh`, `triangle_mesh`, and
  `polygon_surface_mesh`, which triangulates an exact polygon with holes
  through Hypertri under an explicit predicate policy.
- `curve_line_mesh`, `curve_path_line_mesh`, and `curve_region_line_mesh`
  convert Hypercurve curves, paths, and region boundaries into line meshes
  through certified subdivision, retaining the exact maximum chord error.
  Exhausted or undecidable subdivision is an error rather than a fixed-count
  fallback.
- `ExactMesh::triangle_orientation_against` classifies a point against a mesh
  triangle as negative, coplanar, positive, or explicitly unknown.
- `ExactCamera` keeps orbit-camera state exact until `projection64` builds an
  `f64` `Projection64`. Projection and the approximate `unproject_to_z0`
  are explicitly lossy, and unprojection returns `ApproxPoint3`.
- Exports and constructors reject `NaN`, infinity, and values that overflow
  `f32`; GPU uploads reject partial vertices and draw counts beyond OpenGL's
  range.
- The `backend` module provides `GpuColoredMesh` and `UnlitProgram` for
  desktop GLSL 3.30, GLSL ES 3.00 (WebGL 2), and GLSL ES 1.00 (WebGL 1) with
  `OES_vertex_array_object`. Allocation, upload, draw, and release are
  `unsafe` because the caller must guarantee the current `glow` context.
- The default `exact` feature enables the exact geometry layer. With default
  features disabled, the crate is a renderer-only backend that accepts finite
  interleaved vertices and projection matrices.
- Requires Rust 1.95.0 or later.
