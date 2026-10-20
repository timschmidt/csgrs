//! Exact CAD interchange, separate from the authoritative BREP kernel.
//!
//! The first layer is bounded [`part21`] syntax. Parsing a STEP exchange
//! structure does not validate an EXPRESS schema or create trusted geometry.
//! Unknown entity records remain addressable by their original identifiers.

#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
// Rustdoc's auto-trait traversal includes the native model's retained carriers.
#![recursion_limit = "256"]

pub mod part21;
pub mod step;
