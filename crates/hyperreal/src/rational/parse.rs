//! Parse and display helpers for [`Rational`].
//!
//! Parsing behavior remains in `arithmetic.rs`; this file exists to match the
//! planned split layout.
//! Rational parsing implementations live in [`super::arithmetic`].
//!
//! Parsing stays close to the representation constructors so decimal and
//! fractional input can reuse canonicalization and small-constant fast paths.
