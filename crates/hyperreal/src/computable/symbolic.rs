//! Symbolic constructor and simplification helpers for [`Computable`].
//!
//! This module is currently a compatibility shim while code is moved into the
//! planned layout.
//! Symbolic computable simplifications live in [`super::node`].
//!
//! The simplifiers are kept beside node construction and cache maintenance so
//! they can preserve exact structural facts before approximation. This file is
//! a semantic waypoint rather than a re-export layer.
