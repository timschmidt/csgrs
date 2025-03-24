//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](bsp) trees.
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
//! - **metaballs**: enables a `CSG` implementation of [metaballs](https://en.wikipedia.org/wiki/Metaballs)
//! - **hashmap**: enables use of hashbrown for slice, related helper functions, and `is_manifold`
//! - **sdf**: signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) using [fast-surface-nets](https://crates.io/crates/fast-surface-nets)
//! - **offset**: use `geo-buf` for offset operations
//! - **delaunay**: use `geo`s `spade` feature for triangulation
//!
//! #### Optional
//! - **f32**: use f32 as Real, this conflicts with f64
//! - **parallel**: use rayon for multithreading
//! - **svg-io**: create `CSG`s from and convert `CSG`s to SVG's
//! - **truetype-text**: create `CSG`s using TrueType fonts `.ttf`
//! - **hershey-text**: create `CSG`s using Hershey fonts (`.jhf`)
//! - **image-io**: make 2d `CSG`s from images
//! - **earcut**: use `geo`s `earcutr` feature for triangulation

#![forbid(unsafe_code)]
#![deny(unused)]
#![warn(clippy::missing_const_for_fn, clippy::approx_constant, clippy::all)]

pub mod errors;
pub mod float_types;
pub mod vertex;
pub mod plane;
pub mod polygon;
pub mod bsp;
pub mod csg;
pub mod shapes2d;
pub mod shapes3d;
pub mod extrudes;
pub mod io;

#[cfg(any(all(feature = "delaunay", feature = "earcut"), not(any(feature = "delaunay", feature = "earcut"))))]
compile_error!("Either 'delaunay' or 'earcut' feature must be specified, but not both");

#[cfg(any(all(feature = "f64", feature = "f32"), not(any(feature = "f64", feature = "f32"))))]
compile_error!("Either 'f64' or 'f32' feature must be specified, but not both");

pub use csg::CSG;
pub use vertex::Vertex;

#[cfg(feature = "hashmap")]
pub mod flatten_slice;

#[cfg(feature = "truetype-text")]
pub mod truetype;

#[cfg(feature = "image-io")]
pub mod image;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "chull-io")]
pub mod convex_hull;

#[cfg(feature = "hershey-text")]
pub mod hershey;

#[cfg(feature = "sdf")]
pub mod sdf;

#[cfg(feature = "sdf")]
pub mod tpms;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(test)]
mod tests;
