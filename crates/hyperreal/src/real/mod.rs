//! Public symbolic real type and its semantic operation groups.
//!
//! Most implementation still lives in `arithmetic` because the representation
//! invariants, constructor simplifications, structural facts, and borrowed
//! arithmetic fast paths share the same private fields. The sibling modules
//! name the semantic areas so readers can discover the intended split without
//! paying performance risk from moving hot code prematurely.

mod approximation;
mod arithmetic;
mod constructors;
mod convert;
mod exact_set;
mod facts;
mod linear_combination;
mod tests;

pub use arithmetic::*;
pub use exact_set::{
    RealExactSetDenominatorKind, RealExactSetDyadicExponentClass, RealExactSetFacts,
    RealExactSetSignPattern,
};
