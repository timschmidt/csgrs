#![allow(dead_code)]
#![forbid(unsafe_code)]

const EPSILON: f64 = 1e-5;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

pub mod enums;
pub mod vertex;
pub mod plane;
pub mod node;
pub mod polygon;
pub mod csg;

#[cfg(test)]
mod tests;
