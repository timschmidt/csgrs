//! Documentation anchor for approximation and refinement of [`crate::Real`].
//!
//! This module defines no evaluators. Primitive floating-point egress lives in
//! [`super::convert`]; certified interval and sign/refinement queries live in
//! [`super::arithmetic`]. Those APIs distinguish explicit lossy projection from
//! certified bounds and unresolved decisions. The underlying computable graph
//! and its approximation caches remain owned by the computable layer.
