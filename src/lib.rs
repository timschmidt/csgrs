//! A fast, optionally multithreaded **Constructive Solid Geometry (CSG)** library,
//! built around Boolean operations (*union*, *difference*, *intersection*, *xor*) on sets of polygons stored in [BSP](bsp) trees.
//!
//! ![Example CSG output][Example CSG output]
#![cfg_attr(doc, doc = doc_image_embed::embed_image!("Example CSG output", "docs/csg.png"))]

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

/// Prelude containing the common types defined by csgrs
pub mod prelude {
    pub use crate::csg::CSG;
    pub use crate::polygon::Polygon;
    pub use crate::plane::Plane;
    pub use crate::vertex::Vertex;
    #[cfg(feature = "metaballs")]
    pub use crate::metaballs::MetaBall;
}

#[cfg(test)]
mod tests;
