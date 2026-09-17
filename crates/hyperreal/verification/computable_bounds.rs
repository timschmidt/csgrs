//! Verify the included production bound operations against their typed contracts.
//! Denotation lemmas refine these contracts conditional on valid input bounds.
//! Production callers and the remaining bound operations remain open.
use num::bigint::Sign;
use crate::structural::{MagnitudeBits, RealSign};

verus! {
// Import the actual enum variants from the pinned dependency. This declaration
// supplies no contracts for its arithmetic, equality or conversion operations.
#[verifier::external_type_specification]
pub struct ImportedSign(Sign);
}

include!("../src/computable/node/bounds.rs");

include!("bound_denotation.rs");
