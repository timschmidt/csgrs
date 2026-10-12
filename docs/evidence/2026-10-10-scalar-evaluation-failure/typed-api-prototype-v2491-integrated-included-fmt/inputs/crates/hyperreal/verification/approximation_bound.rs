// Verify the shared static producer; expression graph state remains outside this proof.
use super::*;
use num::bigint::BigInt;

#[verus_verify]
struct Computable;
include!("../src/computable/node/approximation_bound.rs");
