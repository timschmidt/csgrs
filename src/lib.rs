//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) trees.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]
//!
//! # Features
//! #### Default
//! - **f64**: use f64 as Real
//! - [**stl-io**](https://en.wikipedia.org/wiki/STL_(file_format)): `.stl` import/export
//! - [**dxf-io**](https://en.wikipedia.org/wiki/AutoCAD_DXF): `.dxf` import/export
//! - **chull-io**: convex hull and minkowski sum
//! - **metaballs**: enables an implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **offset**: use `geo-buf` for offset operations
//! - **delaunay**: use `geo`s `spade` feature for triangulation
//!
//! #### Optional
//! - **f32**: use f32 as Real, this conflicts with f64
//! - **parallel**: use rayon for multithreading
//! - **svg-io**: create `Sketch`s from and convert `Sketch`s to SVG's
//! - **truetype-text**: create `Sketch`s using TrueType fonts `.ttf`
//! - **hershey-text**: create `Sketch`s using Hershey fonts (`.jhf`)
//! - **image-io**: make `Sketch`s from images
//! - **earcut**: use `geo`s `earcutr` feature for triangulation
//! - **bevymesh**: for conversion to a bevy `Mesh`

#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

pub mod errors;
pub mod float_types;
pub mod io;
pub mod mesh;
pub mod nurbs;
pub mod sketch;
pub mod traits;
pub mod voxels;
pub mod toolpath;

#[cfg(any(
    all(feature = "delaunay", feature = "earcut"),
    not(any(feature = "delaunay", feature = "earcut"))
))]
compile_error!("Either 'delaunay' or 'earcut' feature must be specified, but not both");

#[cfg(any(
    all(feature = "f64", feature = "f32"),
    not(any(feature = "f64", feature = "f32"))
))]
compile_error!("Either 'f64' or 'f32' feature must be specified, but not both");

#[cfg(test)]
mod tests;
