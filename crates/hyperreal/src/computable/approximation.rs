use crate::Computable;
use crate::Rational;
use crate::computable::{Precision, Signal, scale, shift, should_stop, signed};
use num::bigint::Sign;
use num::{BigInt, BigUint, Signed, ToPrimitive};
use num::{One, Zero};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::ops::Deref;
use std::sync::LazyLock;

include!("approximation/representation.rs");
include!("approximation/dispatch.rs");
include!("approximation/constants.rs");
include!("approximation/arithmetic_kernels.rs");
include!("approximation/exp_sqrt.rs");
include!("approximation/trig.rs");
include!("approximation/logarithms.rs");
include!("approximation/inverse_trig.rs");
include!("approximation/inverse_hyperbolic.rs");
include!("approximation/statistics.rs");
