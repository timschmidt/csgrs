//! A fast **Constructive Solid Geometry (CSG)** library built around Boolean
//! operations (*union*, *difference*, *intersection*, *xor*) composed from the
//! Hyper geometry crates. `Mesh` routes topology and booleans through
//! `hypermesh` and `Profile` is backed by `hypercurve`, keeping primitive
//! floats at audited API and IO boundaries.
//! The optional [`PolygonMesh`] backend retains planar polygon faces and
//! converts them explicitly to triangle-only [`mesh::Mesh`] geometry.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//! # Features
//! #### Default
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **gerber-io**: Gerber/RS-274X import/export for 2D `Profile` geometry
//! - Exact convex hull and Minkowski sum are always available through `hypermesh`.
//! - **metaballs**: enables an implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **hypertri**: hyperreal-backed polygon and sketch triangulation
//!
//! #### Optional
//! - **parallel**: use rayon for multithreading
//! - **offset**: transitional finite offset/skeleton bridge; certified simple
//!   sharp offsets are handled by hypercurve and remaining regularized buffers
//!   are recomposed into native `Profile` topology
//! - **svg-io**: create `Profile`s from and convert `Profile`s to SVG's
//! - **gerber-io**: create `Profile`s from and convert `Profile`s to Gerber files
//! - **truetype-text**: create `Profile`s using TrueType fonts `.ttf`
//! - **hershey-text**: create `Profile`s using Hershey fonts (`.jhf`)
//! - **image-io**: make `Profile`s from images
//! - **bevymesh**: for conversion to a bevy `Mesh`

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

pub mod errors;
#[allow(dead_code, unused_imports)]
pub(crate) mod hyper_math;
pub mod io;
pub mod mesh;
pub mod parts;
#[cfg(feature = "polygon-mesh")]
pub mod polygon_mesh;
#[cfg(feature = "polygon-mesh")]
pub use polygon_mesh::PolygonMesh;
#[cfg(feature = "sketch")]
pub mod sketch;
#[cfg(feature = "sketch")]
/// Preferred module path for the hypercurve-backed 2D profile API.
pub mod profile {
    pub use crate::sketch::*;
}
#[cfg(feature = "sketch")]
pub use sketch::Profile;
pub mod vertex;

pub mod csg;
pub mod triangulated;
pub mod voxels;

pub use hyperreal::Real;

#[cfg(feature = "wasm")]
pub mod wasm;

#[cfg(test)]
mod tests;
