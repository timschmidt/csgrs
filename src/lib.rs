//#![allow(dead_code)]
#![forbid(unsafe_code)]

pub mod errors;
pub mod float_types;
pub mod io;
pub mod mesh;
pub mod nurbs;
pub mod sketch;
pub mod traits;
pub mod voxels;

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
