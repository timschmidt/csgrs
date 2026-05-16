//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) trees.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//! # Features
//! #### Default
//! - **f64**: use f64 as Real
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **gerber-io**: Gerber/RS-274X import/export for 2D `Sketch` geometry
//! - **chull-io**: convex hull and minkowski sum
//! - **metaballs**: enables an implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **offset**: use `geo` buffer operations
//! - **cavalier**: use `cavalier_contours` for arc-preserving 2D booleans and offsets
//! - **delaunay**: use `geo`s `spade` feature for triangulation
//!
//! #### Optional
//! - **f32**: use f32 as Real, this conflicts with f64
//! - **parallel**: use rayon for multithreading
//! - **svg-io**: create `Sketch`s from and convert `Sketch`s to SVG's
//! - **gerber-io**: create `Sketch`s from and convert `Sketch`s to Gerber files
//! - **truetype-text**: create `Sketch`s using TrueType fonts `.ttf`
//! - **hershey-text**: create `Sketch`s using Hershey fonts (`.jhf`)
//! - **image-io**: make `Sketch`s from images
//! - **earcut**: use `geo`s `earcutr` feature for triangulation
//! - **delaunay-rs**: use the `delaunay` crate for point-set triangulation
//! - **bevymesh**: for conversion to a bevy `Mesh`

#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

#[cfg(feature = "cavalier")]
pub mod cavalier;
pub mod errors;
pub mod float_types;
pub mod io;
pub mod mesh;
#[cfg(feature = "nurbs")]
pub mod nurbs;
pub mod polygon;
pub mod sketch;
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

#[cfg(feature = "bmesh")]
pub mod bmesh;

#[cfg(any(
    all(feature = "delaunay", feature = "earcut"),
    all(feature = "delaunay", feature = "delaunay-rs"),
    all(feature = "earcut", feature = "delaunay-rs"),
    not(any(feature = "delaunay", feature = "earcut", feature = "delaunay-rs"))
))]
compile_error!(
    "Exactly one triangulation feature must be specified: 'delaunay', 'earcut', or 'delaunay-rs'"
);

#[cfg(any(
    all(feature = "f64", feature = "f32"),
    not(any(feature = "f64", feature = "f32"))
))]
compile_error!("Either 'f64' or 'f32' feature must be specified, but not both");

#[cfg(test)]
mod tests;
