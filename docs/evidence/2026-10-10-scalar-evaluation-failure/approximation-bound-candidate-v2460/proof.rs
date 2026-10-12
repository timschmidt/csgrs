//! Verus entry point: imports the production kernels without copying bodies.
#![feature(proc_macro_hygiene)]

#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/verified/mod.rs"]
mod verified;

#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bit_length_model.rs"]
mod bit_length_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/approximation-bound-candidate-v2460/computable_bounds.rs"]
mod computable_bounds;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/computable_cache.rs"]
mod computable_cache;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/digit_scan_model.rs"]
mod digit_scan_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/integer_approximation_model.rs"]
mod integer_approximation_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/magnitude_model.rs"]
mod magnitude_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/rational_model.rs"]
mod rational_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/real_approximation_model.rs"]
mod real_approximation_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/word_scan_model.rs"]
mod word_scan_model;

#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/structural.rs"]
mod structural;
