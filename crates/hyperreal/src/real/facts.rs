//! Documentation anchor for structural fact and zero/sign queries on [`crate::Real`].
//!
//! This module defines no fact queries. Individual scalar queries live in
//! [`super::arithmetic`], including its `arithmetic/facts.rs` implementation file;
//! [`super::exact_set`] owns aggregate summaries of borrowed scalar sequences.
//! Those implementations determine which facts are structural, cached or require
//! explicit refinement. This anchor supplies navigation, not another fact cache.
