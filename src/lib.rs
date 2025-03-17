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

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(test)]
mod tests;
