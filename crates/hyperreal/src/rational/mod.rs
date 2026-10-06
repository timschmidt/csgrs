//! Exact rational arithmetic and conversion support.
//!
//! `arithmetic` owns the representation and hot arithmetic kernels. `convert`
//! and `parse` are kept separate because they are colder entry points with
//! different readability pressure.

mod arithmetic;
mod convert;
mod parse;

pub use arithmetic::*;
