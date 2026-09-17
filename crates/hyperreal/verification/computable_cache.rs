//! Pure cache codecs from the actual production representation file.
//! Atomic operations, allocation, synchronization and callers remain open.
use vstd::prelude::*;
use num::bigint::Sign;
use crate::computable_bounds::{BoundCache, BoundInfo, ExactSignCache};

include!("../src/computable/node/representation.rs");
include!("cache_encoding_model.rs");
