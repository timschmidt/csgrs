//! Layout conformance through native comparison, independent fixed arithmetic and replay.
//!
//! Comparisons include full values/errors and subsequent consumption. Only the
//! bounded fixed oracle derives its arithmetic independently; native agreement
//! elsewhere shares scalar kernels and cannot alone prove those kernels correct.
use hyperlattice_interval::hyperinterval::{
    F32Enclosure, F64Enclosure, FixedInterval, IntervalError,
};
use hyperlattice_interval::{IntervalLatticeError, IntervalVector};
use interval_layout_conformance::{Primitive, Soa};
use std::fmt::Debug;

/// Compare round trips and five operations, then consume intersection/hull in a dot.
/// Arithmetic Result equality includes scalar errors; composition's set operations
/// are expected to succeed on the validated fixtures and are unwrapped separately.
fn compare<I: Primitive + Eq + Debug, const N: usize>(
    a: IntervalVector<I, N>,
    b: IntervalVector<I, N>,
) {
    let x = Soa::from_vector(&a);
    let y = Soa::from_vector(&b);
    assert_eq!(x.to_vector(), Ok(a));
    assert_eq!(y.to_vector(), Ok(b));
    assert_eq!(x.add(&y).and_then(|v| v.to_vector()), a.add(&b, &mut ()));
    assert_eq!(x.sub(&y).and_then(|v| v.to_vector()), a.sub(&b, &mut ()));
    assert_eq!(x.hull(&y).and_then(|v| v.to_vector()), a.hull(&b, &mut ()));
    assert_eq!(
        x.intersection(&y).and_then(|v| v.to_vector()),
        a.intersection(&b, &mut ())
    );
    assert_eq!(x.dot(&y), a.dot(&b, &mut ()));
    // Feed generated results back in, including canonical empty intersections.
    let native = a
        .intersection(&b, &mut ())
        .unwrap()
        .hull(&a, &mut ())
        .unwrap();
    let soa = x.intersection(&y).unwrap().hull(&x).unwrap();
    assert_eq!(soa.to_vector(), Ok(native));
    assert_eq!(soa.dot(&y), native.dot(&b, &mut ()));
}

#[test]
/// Exhaust all 23,409 ordered interval pairs with raw endpoints in -8..=8.
/// Independent integer corner extrema and floor/ceiling division certify Q3 dot
/// bounds; separate endpoint formulas check sums, differences, intersection and
/// hull. Two lanes include a reflected interval, then native comparison exercises
/// subsequent consumption without replacing the independent expectations.
fn independent_fixed_oracle_and_composition() {
    /// Three fractional bits make raw product bounds require outward division by eight.
    type I = FixedInterval<i32, 3>;
    for al in -8..=8 {
        for au in al..=8 {
            for bl in -8..=8 {
                for bu in bl..=8 {
                    let a =
                        IntervalVector::new([I::new(al, au).unwrap(), I::new(-au, -al).unwrap()])
                            .unwrap();
                    let b = IntervalVector::new([I::new(bl, bu).unwrap(), I::new(bl, bu).unwrap()])
                        .unwrap();
                    let x = Soa::from_vector(&a);
                    let y = Soa::from_vector(&b);
                    let bounds = |v: Soa<I, 2>| {
                        v.to_vector()
                            .unwrap()
                            .lanes()
                            .map(|l| l.map(|i| i.bounds().unwrap()))
                    };
                    assert_eq!(
                        bounds(x.add(&y).unwrap()),
                        Some([(al + bl, au + bu), (-au + bl, -al + bu)])
                    );
                    assert_eq!(
                        bounds(x.sub(&y).unwrap()),
                        Some([(al - bu, au - bl), (-au - bu, -al - bl)])
                    );
                    let products = |l: i32, u: i32| {
                        let corners = [l * bl, l * bu, u * bl, u * bu];
                        let low = *corners.iter().min().unwrap();
                        let high = *corners.iter().max().unwrap();
                        (low.div_euclid(8), -(-high).div_euclid(8))
                    };
                    let p = products(al, au);
                    let q = products(-au, -al);
                    assert_eq!(x.dot(&y).unwrap().bounds(), Some((p.0 + q.0, p.1 + q.1)));
                    let retained = [(al.max(bl), au.min(bu)), ((-au).max(bl), (-al).min(bu))];
                    assert_eq!(
                        bounds(x.intersection(&y).unwrap()),
                        retained.iter().all(|(l, u)| l <= u).then_some(retained)
                    );
                    assert_eq!(
                        bounds(x.hull(&y).unwrap()),
                        Some([(al.min(bl), au.max(bu)), ((-au).min(bl), (-al).max(bu))])
                    );
                    compare(a, b);
                }
            }
        }
    }
}

/// Exercise two nonempty products, empty/nonempty and empty/empty at nine dimensions.
/// Explicitly distinguish the zero-dimensional unit product and its zero dot image.
fn dimensions<I: Primitive + Eq + Debug>(a: I, b: I) {
    /// Instantiate the same full-value/composition comparisons at each literal dimension.
    macro_rules! check {
        ($($n:literal),*) => { $(
            compare(IntervalVector::<_, $n>::new([a; $n]).unwrap(), IntervalVector::new([b; $n]).unwrap());
            compare(IntervalVector::<I, $n>::empty().unwrap(), IntervalVector::new([a; $n]).unwrap());
            compare(IntervalVector::<I, $n>::empty().unwrap(), IntervalVector::empty().unwrap());
        )* };
    }
    check!(0, 1, 2, 3, 4, 16, 64, 256, 1024);
    let unit = IntervalVector::<I, 0>::new([]).unwrap();
    assert!(unit.is_singleton());
    assert!(!Soa::from_vector(&unit).to_vector().unwrap().is_empty());
    assert_eq!(
        Soa::from_vector(&unit).dot(&Soa::from_vector(&unit)),
        Ok(I::zero().unwrap())
    );
}

#[test]
/// Apply the dimension matrix to Q8, Q32, binary32 and binary64 sixteenth-grid inputs.
fn all_families_and_dimensions() {
    dimensions(
        FixedInterval::<i32, 8>::new(-16, 32).unwrap(),
        FixedInterval::new(0, 16).unwrap(),
    );
    dimensions(
        FixedInterval::<i64, 32>::new(-1 << 28, 2 << 28).unwrap(),
        FixedInterval::new(0, 1 << 28).unwrap(),
    );
    dimensions(
        F32Enclosure::new(-0.0625, 0.125).unwrap(),
        F32Enclosure::new(0.0, 0.0625).unwrap(),
    );
    dimensions(
        F64Enclosure::new(-0.0625, 0.125).unwrap(),
        F64Enclosure::new(0.0, 0.0625).unwrap(),
    );
}

#[test]
/// Compare selected signed extremes at coarsest/finest valid scales, then reject bad scales.
/// This is a boundary corpus, not enumeration of every endpoint in the full domain.
/// A two-term dot separately checks that later cancellation cannot hide first-product overflow.
fn full_domain_extremes_and_failure_order() {
    /// Generate all ordered pairs from seven extreme numerators for one width/scale.
    macro_rules! fixed {
        ($int:ty, $frac:literal) => {{
            /// Selected fixed family; boundary numerators keep their exact scale.
            type I = FixedInterval<$int, $frac>;
            let extremes = [
                <$int>::MIN,
                <$int>::MIN + 1,
                -1,
                0,
                1,
                <$int>::MAX - 1,
                <$int>::MAX,
            ];
            for &al in &extremes {
                for &au in &extremes {
                    if al > au {
                        continue;
                    }
                    for &bl in &extremes {
                        for &bu in &extremes {
                            if bl > bu {
                                continue;
                            }
                            compare(
                                IntervalVector::new([I::new(al, au).unwrap()]).unwrap(),
                                IntervalVector::new([I::new(bl, bu).unwrap()]).unwrap(),
                            );
                        }
                    }
                }
            }
        }};
    }
    fixed!(i32, 0);
    fixed!(i32, 31);
    fixed!(i64, 0);
    fixed!(i64, 63);
    assert_eq!(
        IntervalVector::<FixedInterval<i32, 32>, 0>::new([]),
        Err(IntervalLatticeError::Scalar(IntervalError::InvalidScale))
    );
    assert_eq!(
        IntervalVector::<FixedInterval<i64, 64>, 0>::empty(),
        Err(IntervalLatticeError::Scalar(IntervalError::InvalidScale))
    );
    // An overflowing first product cannot be hidden by a later cancellation.
    /// Whole-unit grid on which MAX times two fails before the negative term is visited.
    type I = FixedInterval<i32, 0>;
    let a =
        IntervalVector::new([I::point(i32::MAX).unwrap(), I::point(-i32::MAX).unwrap()]).unwrap();
    let b = IntervalVector::new([I::point(2).unwrap(); 2]).unwrap();
    assert_eq!(
        Soa::from_vector(&a).dot(&Soa::from_vector(&b)),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
}

#[test]
/// Check subnormal outward rounding, overflow, signed-zero normalization and invalid ingress.
/// Each IEEE family also gets 2,000 deterministic finite-bit fixtures, full native
/// comparisons and canonical JSON replay. Random filtering has no internal retry
/// budget; these fixed-seed runs are finite evidence, not exhaustive IEEE coverage.
fn ieee_bits_subnormals_rounding_and_overflow() {
    /// Instantiate the same edge cases and wrapping-LCG finite-bit stream for one IEEE width.
    macro_rules! floats {
        ($interval:ty, $float:ty, $bits:ty) => {{
            let point = |v| IntervalVector::new([<$interval>::point(v).unwrap()]).unwrap();
            let tiny = <$float>::from_bits(1);
            let a = Soa::from_vector(&point(tiny));
            assert_eq!(
                a.add(&a).unwrap().to_vector().unwrap().lanes().unwrap()[0].bounds(),
                Some((<$float>::from_bits(2), <$float>::from_bits(2)))
            );
            assert_eq!(
                a.dot(&Soa::from_vector(&point(0.5))).unwrap().bounds(),
                Some((0.0, tiny))
            );
            assert_eq!(
                Soa::from_vector(&point(<$float>::MAX)).add(&a).err(),
                Some(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
            let zero = Soa::from_vector(&point(-0.0)).to_vector().unwrap();
            assert_eq!(zero.lanes().unwrap()[0].bounds().unwrap().0.to_bits(), 0);
            assert!(<$interval>::new(<$float>::NAN, 0.0).is_err());
            assert!(<$interval>::new(0.0, <$float>::INFINITY).is_err());
            assert_eq!(
                <$interval>::new(1.0, -1.0),
                Err(IntervalError::ReversedEndpoints)
            );
            let mut state = 20261112_u64;
            for _ in 0..2000 {
                let mut next = || {
                    loop {
                        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                        let value = <$float>::from_bits(state as $bits);
                        if value.is_finite() {
                            break value;
                        }
                    }
                };
                let a = next();
                let b = next();
                let vector =
                    IntervalVector::new([<$interval>::new(a.min(b), a.max(b)).unwrap()]).unwrap();
                compare(vector, point(next()));
                // Existing canonical wire validation remains the replay authority.
                let text = serde_json::to_string(&vector).unwrap();
                let restored: IntervalVector<$interval, 1> = serde_json::from_str(&text).unwrap();
                assert_eq!(Soa::from_vector(&restored).to_vector(), Ok(vector));
                assert_eq!(
                    serde_json::to_string(&Soa::from_vector(&restored).to_vector().unwrap())
                        .unwrap(),
                    text
                );
            }
        }};
    }
    floats!(F32Enclosure, f32, u32);
    floats!(F64Enclosure, f64, u64);
}

#[test]
/// Replay native JSON through SoA, consume it, and reject invalid order/dimensions.
/// An empty earlier lane must not conceal a later reversed scalar. Canonical output
/// is validated again; accepted noncanonical input text need not be reproduced verbatim.
fn canonical_replay_revalidates_and_consumes() {
    /// Two-lane Q32 carrier whose wire endpoints are signed i64 numerators.
    type V = IntervalVector<FixedInterval<i64, 32>, 2>;
    for wire in [
        "null",
        "[[0,1],[-9223372036854775808,9223372036854775807]]",
        "[null,[0,1]]",
    ] {
        let native: V = serde_json::from_str(wire).unwrap();
        let restored = Soa::from_vector(&native).to_vector().unwrap();
        assert_eq!(restored, native);
        let encoded = serde_json::to_string(&restored).unwrap();
        assert_eq!(serde_json::from_str::<V>(&encoded).unwrap(), native);
        compare(native, restored);
    }
    for bad in ["[[2,1],[0,1]]", "[null,[2,1]]", "[]", "[[0,1]]"] {
        assert!(serde_json::from_str::<V>(bad).is_err(), "{bad}");
    }
}
