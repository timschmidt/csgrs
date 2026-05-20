//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) trees.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//! # Features
//! #### Default
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **gerber-io**: Gerber/RS-274X import/export for 2D `Profile` geometry
//! - **chull-io**: convex hull and minkowski sum
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
#![cfg_attr(test, allow(deprecated))]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

pub mod errors;
pub mod float_types;
pub mod io;
pub mod mesh;
pub mod polygon;
#[cfg(feature = "sketch")]
pub mod sketch;
#[cfg(feature = "sketch")]
pub mod profile {
    pub use crate::sketch::*;
}
#[cfg(feature = "sketch")]
pub use sketch::Profile;
pub mod vertex;

#[cfg(feature = "offset")]
pub mod toolpath;

pub mod csg;
pub mod triangulated;
pub mod voxels;

/// Compatibility re-exports for trait imports used in examples.
pub mod traits {
    pub use crate::csg::CSG;
}

#[cfg(feature = "wasm")]
pub mod wasm;

#[cfg(test)]
mod tests;
