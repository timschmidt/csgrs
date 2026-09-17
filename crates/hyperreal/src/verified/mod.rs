//! Executable kernels shared by Cargo and the Verus verification entry point.
//!
//! Only proof annotations disappear in a normal build. There is no second
//! implementation of these functions in the verification harness.

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(not(verus_keep_ghost))]
macro_rules! proof_decl {
    ($($tokens:tt)*) => {};
}

pub(crate) mod division;
pub(crate) mod fixed_gcd;
pub(crate) mod float;
pub(crate) mod gcd;
pub(crate) mod lehmer;
pub(crate) mod limbs;
pub(crate) mod word;
