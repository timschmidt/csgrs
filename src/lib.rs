//#![allow(dead_code)]
#![forbid(unsafe_code)]

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

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(test)]
mod tests;
