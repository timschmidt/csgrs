#![allow(dead_code)]
#![forbid(unsafe_code)]

pub mod errors;
pub mod float_types;
pub mod vertex;
pub mod plane;
pub mod polygon;
pub mod bsp;
pub mod csg;

#[cfg(test)]
mod tests;
