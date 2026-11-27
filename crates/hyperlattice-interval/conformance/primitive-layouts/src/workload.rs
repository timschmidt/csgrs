//! Common, bounded fixtures for native and Wasm probes.
//!
//! Both layouts run the same sequence of scalar kernels. The chains retain each
//! provider's storage between operations, not the original operands' algebraic
//! correlations. These small fixtures support comparison, not universal bounds
//! on scalar completion, stack use or elapsed time.
use crate::{Primitive, Soa};
use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use hyperlattice_interval::{IntervalLatticeError, IntervalVector};

/// Construct the experiment's sixteenth-grid inputs for four concrete scalar types.
///
/// The implementations are assertion-based fixture builders, not general fallible
/// ingress. [`inputs`] supplies small numerators and nonnegative widths; arbitrary
/// direct calls can overflow integer intermediates, round f32 numerators, or panic
/// on scalar constructor failure, including unqualified floating environments.
pub trait Fixture: Primitive {
    /// Build [numerator/16, (numerator+width)/16] on the fixture's bounded domain.
    /// Direct callers must satisfy the concrete implementation's arithmetic and ingress limits.
    fn lane(numerator: i32, width: i32) -> Self;
}

/// Represent sixteenths exactly as multiples of sixteen on the i32 Q8 grid.
impl Fixture for FixedInterval<i32, 8> {
    /// Scale both numerators by sixteen and unwrap checked ingress.
    /// The i32 sum/products must fit; reversed endpoints panic at construction.
    fn lane(n: i32, w: i32) -> Self {
        Self::new(n * 16, (n + w) * 16).unwrap()
    }
}
/// Represent sixteenths exactly as multiples of 2^28 on the i64 Q32 grid.
impl Fixture for FixedInterval<i64, 32> {
    /// Widen before shifting, but compute n+w in i32 first; that sum must fit.
    /// The widened shifts fit for every i32 numerator; reversed bounds panic.
    fn lane(n: i32, w: i32) -> Self {
        Self::new(i64::from(n) << 28, i64::from(n + w) << 28).unwrap()
    }
}
/// Construct exact small binary32 fixture endpoints on qualified targets.
impl Fixture for F32Enclosure {
    /// Cast each i32 numerator to f32, divide by sixteen, then unwrap ingress.
    /// Large direct inputs can round during the cast; n+w is still an i32 sum.
    fn lane(n: i32, w: i32) -> Self {
        Self::new(n as f32 / 16.0, (n + w) as f32 / 16.0).unwrap()
    }
}
/// Construct binary64 sixteenths exactly when the i32 numerator sum fits.
impl Fixture for F64Enclosure {
    /// Convert the two i32 numerators exactly, divide by sixteen, and unwrap ingress.
    /// Reversed bounds or unqualified targets panic; n+w must fit before conversion.
    fn lane(n: i32, w: i32) -> Self {
        Self::new(f64::from(n) / 16.0, f64::from(n + w) / 16.0).unwrap()
    }
}

/// Seeds are reduced to a bounded domain before any arithmetic. Native float
/// arithmetic here only constructs exact binary fixture inputs, never results.
///
/// Reduce seed modulo 17. Lane i uses numerator (i+seed)%17-8 and width i%3
/// for a, and (i+2*seed)%13-6 and width i%2 for b, all divided by sixteen.
/// N=0 constructs two unit products without calling lane. Constructors are
/// unwrapped; the supplied family/target must admit these fixtures. Inline arrays
/// scale with N; this is no stack or total-work budget for arbitrary dimensions.
pub fn inputs<I: Fixture, const N: usize>(
    seed: u32,
) -> (IntervalVector<I, N>, IntervalVector<I, N>) {
    let seed = (seed % 17) as usize;
    let a = core::array::from_fn(|i| I::lane(((i + seed) % 17) as i32 - 8, (i % 3) as i32));
    let b = core::array::from_fn(|i| I::lane(((i + 2 * seed) % 13) as i32 - 6, (i % 2) as i32));
    (
        IntervalVector::new(a).unwrap(),
        IntervalVector::new(b).unwrap(),
    )
}

/// Run four add/subtract pairs, intersect with a, hull with b, then dot with a.
///
/// Each step consumes the preceding Cartesian enclosure; subtracting b does not
/// undo addition of a nonsingleton b. Intersection/hull are regular interval-set
/// operations, and the first scalar failure propagates. Inputs remain borrowed
/// and reusable. The unit context adds no resource accounting.
pub fn native_chain<I: Primitive, const N: usize>(
    a: &IntervalVector<I, N>,
    b: &IntervalVector<I, N>,
) -> Result<I, IntervalLatticeError> {
    let mut value = *a;
    for _ in 0..4 {
        value = value.add(b, &mut ())?.sub(b, &mut ())?;
    }
    value = value.intersection(a, &mut ())?.hull(b, &mut ())?;
    value.dot(a, &mut ())
}

/// Run the native_chain sequence while retaining separate arrays between steps.
///
/// Ingress/materialization are outside this function; hull's empty-operand path
/// may still reconstruct native vectors internally. The final result is a scalar
/// enclosure, preserving operation order, empty/unit semantics and typed failures.
pub fn soa_chain<I: Primitive, const N: usize>(
    a: &Soa<I, N>,
    b: &Soa<I, N>,
) -> Result<I, IntervalLatticeError> {
    let mut value = *a;
    for _ in 0..4 {
        value = value.add(b)?.sub(b)?;
    }
    value = value.intersection(a)?.hull(b)?;
    value.dot(a)
}
