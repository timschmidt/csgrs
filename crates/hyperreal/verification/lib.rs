//! Verus entry point: imports the production kernels without copying bodies.
#![feature(proc_macro_hygiene)]

#[path = "../src/verified/mod.rs"]
mod verified;

mod rational_model;
mod magnitude_model;
mod bit_length_model;
