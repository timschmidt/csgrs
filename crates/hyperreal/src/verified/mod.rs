//! Shared executable arithmetic kernels and their optional Verus proof annotations.
//!
//! The ordinary Hyperreal library and verification/lib.rs import this same module
//! tree. The child files contain the production bodies, attached specifications
//! and ghost lemmas; the verification entry point does not substitute copied
//! runtime algorithms. Normal builds omit ghost declarations and erase proof
//! blocks through the two local macros below. Erasure establishes no proof of
//! the remaining executable behavior by itself.
//!
//! The children own native word/float arithmetic, products and accumulation,
//! limb operations, GCD selection/reduction, fractions and dyadic aggregates.
//! Their own module/item docs specify admitted inputs and failures. The shared
//! location does not make every input valid or supply a universal resource cap,
//! and proof annotations alone are not a current validation or completeness claim.
//!
//! Both erasure macros are lexical helpers for this module and its descendants,
//! not exported public APIs. Under verus_keep_ghost these local definitions are
//! absent and the verification environment supplies proof syntax. Private Rustdoc
//! can render the normal helpers and executable items; ordinary Cargo docs do not
//! include declarations gated on the ghost configuration.

/// Erase a proof block in ordinary builds without expanding its contents.
///
/// Matches any balanced sequence of token trees, including empty input, and emits
/// no tokens. Child kernels use it in statement/item contexts for proof assertions,
/// lemma calls and specification-only syntax. Names and operations inside are not
/// resolved or executed by ordinary Rust: no assertion, validation or side effect
/// survives this shim. It does not produce a unit-valued expression.
///
/// Defined only when verus_keep_ghost is absent. The ghost configuration uses the
/// verification environment's proof macro instead; this shim does not validate
/// those bodies or suppress obligations in a Verus run.
#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

/// Erase a ghost declaration in ordinary builds without introducing bindings.
///
/// Accepts arbitrary balanced token trees, including empty input, and emits
/// nothing. Kernels use it for ghost snapshots and specification-only bindings
/// needed by later proof blocks. Such names remain unavailable to ordinary runtime
/// code; the initializer is neither resolved nor evaluated. Like proof, this is
/// an erasure shim for declaration/statement contexts, not a unit expression.
///
/// Defined only outside verus_keep_ghost and kept lexically visible to child
/// modules. The verification configuration supplies its own ghost-declaration
/// syntax; normal compilation of an erased declaration proves nothing about it.
#[cfg(not(verus_keep_ghost))]
macro_rules! proof_decl {
    ($($tokens:tt)*) => {};
}

pub(crate) mod accumulate;
pub(crate) mod aggregate;
pub(crate) mod division;
pub(crate) mod dyadic;
pub(crate) mod fixed_gcd;
pub(crate) mod float;
pub(crate) mod fraction;
pub(crate) mod gcd;
pub(crate) mod lehmer;
pub(crate) mod limbs;
pub(crate) mod product;
pub(crate) mod word;
