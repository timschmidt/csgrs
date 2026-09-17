//! Core computable expression graph.
//!
//! This facade keeps the shared imports and includes semantic implementation
//! slices. The included files remain in one module so private graph/cache
//! invariants and hot constructor rewrites keep their original visibility.

use crate::computable::approximation::{
    Approximation, LinearCombination3, NormalQuantileData, SharedConstant,
};
use crate::{MagnitudeBits, Rational, RealSign, RealStructuralFacts, ZeroKnowledge};
use core::cmp::Ordering;
use num::Signed;
use num::{BigInt, BigUint, bigint::Sign};
use num::{One, Zero};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::{
    collections::HashSet,
    ops::{Deref, Neg},
    sync::{Arc, LazyLock},
};

include!("node/bounds.rs");
include!("node/representation.rs");
include!("node/primitive_constructors.rs");
include!("node/structural_analysis.rs");
include!("node/exp_relation.rs");
include!("node/exp_relation_reuse.rs");
include!("node/exp_trig.rs");
include!("node/logarithms.rs");
include!("node/roots_inverse_hyperbolic.rs");
include!("node/algebra.rs");
include!("node/approximation_queries.rs");
include!("node/scale.rs");
include!("node/tests.rs");
include!("node/exp_relation_tests.rs");
include!("node/exp_relation_reuse_tests.rs");
include!("node/linear_demand_tests.rs");
include!("node/fractional_separation_tests.rs");
include!("node/cache_rescale_tests.rs");
include!("node/precision_boundary_tests.rs");
