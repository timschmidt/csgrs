//! Verify the included production bound operations against their typed contracts.
//! Real denotations, callers and the remaining bound operations remain open.
use num::bigint::Sign;

verus! {
// Import the actual enum variants from the pinned dependency. This declaration
// supplies no contracts for its arithmetic, equality or conversion operations.
#[verifier::external_type_specification]
pub struct ImportedSign(Sign);
}

include!("../src/computable/node/bounds.rs");
