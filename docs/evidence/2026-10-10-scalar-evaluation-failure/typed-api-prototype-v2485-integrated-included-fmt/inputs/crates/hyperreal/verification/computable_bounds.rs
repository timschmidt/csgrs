// Verify the included production bound operations against their typed contracts.
// Denotation lemmas refine these contracts conditional on valid input bounds.
// Production callers and the remaining bound operations remain open.
use num::bigint::Sign;
use crate::structural::{MagnitudeBits, RealSign};

include!("../src/computable/node/bounds.rs");

include!("bound_denotation.rs");
