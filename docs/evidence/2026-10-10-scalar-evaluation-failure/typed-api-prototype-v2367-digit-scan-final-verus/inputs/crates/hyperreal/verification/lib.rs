//! Verus entry point: imports the production kernels without copying bodies.
#![feature(proc_macro_hygiene)]

#[path = "../src/verified/mod.rs"]
mod verified;

mod bit_length_model;
mod computable_bounds;
mod computable_cache;
mod digit_scan_model;
mod integer_approximation_model;
mod magnitude_model;
mod rational_model;
mod real_approximation_model;

#[path = "../src/structural.rs"]
mod structural;
