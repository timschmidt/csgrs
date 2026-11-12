//! Common, bounded fixtures for native and Wasm probes.
use crate::{Primitive, Soa};
use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use hyperlattice_interval::{IntervalLatticeError, IntervalVector};

pub trait Fixture: Primitive {
    fn lane(numerator: i32, width: i32) -> Self;
}

impl Fixture for FixedInterval<i32, 8> {
    fn lane(n: i32, w: i32) -> Self {
        Self::new(n * 16, (n + w) * 16).unwrap()
    }
}
impl Fixture for FixedInterval<i64, 32> {
    fn lane(n: i32, w: i32) -> Self {
        Self::new(i64::from(n) << 28, i64::from(n + w) << 28).unwrap()
    }
}
impl Fixture for F32Enclosure {
    fn lane(n: i32, w: i32) -> Self {
        Self::new(n as f32 / 16.0, (n + w) as f32 / 16.0).unwrap()
    }
}
impl Fixture for F64Enclosure {
    fn lane(n: i32, w: i32) -> Self {
        Self::new(f64::from(n) / 16.0, f64::from(n + w) / 16.0).unwrap()
    }
}

/// Seeds are reduced to a bounded domain before any arithmetic. Native float
/// arithmetic here only constructs exact binary fixture inputs, never results.
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
