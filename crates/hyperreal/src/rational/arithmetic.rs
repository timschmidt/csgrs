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
include!("arithmetic/construction.rs");
include!("arithmetic/aggregate_products.rs");
include!("arithmetic/queries_conversion.rs");
include!("arithmetic/squares_powers.rs");
include!("arithmetic/as_ref.rs");
include!("arithmetic/format_parse.rs");
include!("arithmetic/ops.rs");
include!("arithmetic/comparison.rs");
include!("arithmetic/tests.rs");
