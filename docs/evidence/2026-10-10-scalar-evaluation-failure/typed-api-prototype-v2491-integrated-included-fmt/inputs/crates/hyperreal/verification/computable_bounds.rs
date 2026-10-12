// Verify the included production bound operations against their typed contracts.
// Denotation lemmas refine these contracts conditional on valid input bounds.
// Production callers and the remaining bound operations remain open.
use crate::structural::{MagnitudeBits, RealSign};
use num::bigint::Sign;

include!("../src/computable/node/bounds.rs");

include!("bound_denotation.rs");
