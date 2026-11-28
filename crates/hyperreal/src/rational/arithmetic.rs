//! Exact rational storage, arithmetic, conversion queries and retained evidence.
//!
//! Included fragments share this module's private representation and helper
//! authority; their file boundaries do not define separate numerical kernels.
//! Exact operations may retain shared results or canonicalize marked unreduced
//! parts. Caches and borrowed filters reduce some work, but this layer has no
//! universal total-work, retained-byte or cancellation contract. Test fragments
//! exercise values, reuse and trace behavior without defining the public API.

use crate::Problem;
use crate::structural::{RationalFacts, RationalStorageClass};
use num::bigint::Sign::{self, *};
use num::{BigInt, BigUint, ToPrimitive};
use num::{One, Zero};
#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize};
use std::cmp::Ordering;
use std::ops::Deref;
use std::sync::{Arc, LazyLock, OnceLock};

include!("arithmetic/representation.rs");
include!("arithmetic/algorithm_dispatch.rs");
include!("arithmetic/barrett_division.rs");
include!("arithmetic/toom4_multiplication.rs");
include!("arithmetic/toom6_multiplication.rs");
include!("arithmetic/toom8_multiplication.rs");
include!("arithmetic/ntt_multiplication.rs");
include!("arithmetic/construction.rs");
include!("arithmetic/aggregate_products.rs");
include!("arithmetic/queries_conversion.rs");
include!("arithmetic/squares_powers.rs");
include!("arithmetic/as_ref.rs");
include!("arithmetic/format_parse.rs");
include!("arithmetic/ops.rs");
include!("arithmetic/comparison.rs");
include!("arithmetic/tests.rs");

/// Integration witnesses for retained arithmetic layouts and source lifetimes.
#[cfg(test)]
mod retention_tests;
