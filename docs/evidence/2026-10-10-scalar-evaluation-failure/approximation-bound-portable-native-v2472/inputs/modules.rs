mod verified { pub(crate) mod word { include!("word.rs"); } }

mod candidate_plain {
use patched::bigint::{BigInt, Sign};
include!("bound.rs");
struct Computable;
include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/approximation-bound-candidate-v2460/candidate/crates/hyperreal/src/computable/node/approximation_bound.rs");
include!("convert.rs");
pub(crate) fn run(precision: i32, value: &BigInt) -> crate::Observation {
    observation(Computable::bound_from_approx(precision, value))
}
}

mod original_plain {
use upstream::{BigInt, Sign};
include!("bound.rs");
struct Computable;
include!("original.rs");
include!("convert.rs");
pub(crate) fn run(precision: i32, value: &BigInt) -> crate::Observation {
    observation(Computable::bound_from_approx(precision, value))
}
}

mod candidate_counted {
use patched::bigint::{BigInt as ActualBigInt, BigUint as ActualMagnitude, Sign};
include!("bound.rs");
struct Computable;
include!("instrumentation.rs");
include!("/home/tim/Documents/GitHub/workspace/csgrs/docs/evidence/2026-10-10-scalar-evaluation-failure/approximation-bound-candidate-v2460/candidate/crates/hyperreal/src/computable/node/approximation_bound.rs");
include!("convert.rs");
pub(crate) fn run(precision: i32, value: &ActualBigInt) -> (crate::Observation, crate::Calls) {
    let calls = Counters::default();
    let value = BigInt { value, calls: &calls };
    let result = observation(Computable::bound_from_approx(precision, &value));
    (result, calls.0.get())
}
}

mod original_counted {
use upstream::{BigInt as ActualBigInt, BigUint as ActualMagnitude, Sign};
include!("bound.rs");
struct Computable;
include!("instrumentation.rs");
include!("original.rs");
include!("convert.rs");
pub(crate) fn run(precision: i32, value: &ActualBigInt) -> (crate::Observation, crate::Calls) {
    let calls = Counters::default();
    let value = BigInt { value, calls: &calls };
    let result = observation(Computable::bound_from_approx(precision, &value));
    (result, calls.0.get())
}
}
