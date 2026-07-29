//! Constructive-solid-geometry grammar over native Hyper geometry.
//!
//! [`solid`] constructs and operates directly on [`TriangleMesh`], while
//! [`curve`] constructs native `hypercurve::CurveRegion2`,
//! `hypercurve::CurveString2`, and `hypercurve::CurvePath2` values and converts
//! filled regions into solids through extrusion, revolution, and sweep.
//! Hypermesh owns solid topology and Booleans; Hypercurve owns planar topology
//! and Booleans. Primitive floats remain at explicit adapter and I/O boundaries.
//!
//! The optional `AttributedMesh` attaches aligned face metadata without
//! becoming another modeling carrier.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//! # Features
//! #### Default
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **obj-io**: Wavefront `.obj` mesh import/export
//! - **gltf-io**: self-contained glTF/GLB triangle-scene import and glTF export
//! - **vrml-io**: VRML 2.0 `.wrl` indexed-face scene import
//! - **gerber-io**: Gerber/RS-274X import/export for native filled and open curves
//! - Exact convex hull and Minkowski sum are always available through `hypermesh`.
//! - **metaballs**: enables an implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **hypertri**: hyperreal-backed polygon and curve triangulation
//!
//! #### Optional
//! - **offset**: regularized planar offsets returned as native filled regions
//! - **svg-io**: native curve import/export for SVG
//! - **gerber-io**: native curve import/export for Gerber
//! - **truetype-text**: filled regions from TrueType fonts
//! - **hershey-text**: open strings from Hershey fonts
//! - **image-io**: filled regions from raster contours
//! - **bevymesh**: conversion from native triangle geometry to Bevy

#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]
// JavaScript bindings intentionally mirror flat host APIs, and public geometry
// errors retain their structured diagnostic payloads.
#![allow(
    clippy::items_after_test_module,
    clippy::new_without_default,
    clippy::result_large_err,
    clippy::should_implement_trait,
    clippy::too_many_arguments
)]

pub mod adapter;
#[cfg(feature = "attributed")]
pub mod attributed;
#[cfg(feature = "attributed")]
pub use attributed::{AttributeAlignmentError, AttributedMesh};
#[cfg(feature = "curve")]
pub mod curve;
pub mod errors;
// These small scalar-boundary helpers are shared by independently optional
// import, wasm, sampled-field, text, and curve features. Each feature subset
// intentionally leaves a different subset unused.
#[allow(dead_code)]
pub(crate) mod hyper_math;
pub(crate) mod implicit;
pub mod io;
pub mod parts;
pub mod solid;

pub mod voxels;

pub use hypermesh::TriangleMesh;
pub use hyperreal::Real;

#[cfg(feature = "wasm")]
pub mod wasm;
