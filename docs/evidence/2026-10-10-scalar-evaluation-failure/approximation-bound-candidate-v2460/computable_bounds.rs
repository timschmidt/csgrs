//! Verify the included production bound operations against their typed contracts.
//! Denotation lemmas refine these contracts conditional on valid input bounds.
//! Production callers and the remaining bound operations remain open.
use num::bigint::{BigInt, Sign};
use crate::structural::{MagnitudeBits, RealSign};


include!("/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/computable/node/bounds.rs");

include!("/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bound_denotation.rs");

// Only the static producer method is under test; no graph representation is imported.
#[verus_verify]
struct Computable;
include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/approximation-bound-candidate-v2460/candidate/crates/hyperreal/src/computable/node/approximation_bound.rs");
