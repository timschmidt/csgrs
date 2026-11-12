#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{IntervalArithmetic, IntervalMatrix, IntervalVector};

// Independent endpoint oracle: enumerate every corner of the two independent
// input boxes using exact i64 arithmetic, without calling any interval kernel.
fn dot_bounds(a: &[(i32, i32)], b: &[(i32, i32)]) -> (i32, i32) {
    let mut result = (i64::MAX, i64::MIN);
    for mask in 0..(1_usize << (2 * a.len())) {
        let mut sum = 0_i64;
        for i in 0..a.len() {
            let x = if mask & (1 << i) == 0 { a[i].0 } else { a[i].1 };
            let y = if mask & (1 << (a.len() + i)) == 0 {
                b[i].0
            } else {
                b[i].1
            };
            sum += i64::from(x) * i64::from(y);
        }
        result = (result.0.min(sum), result.1.max(sum));
    }
    (result.0.try_into().unwrap(), result.1.try_into().unwrap())
}

fn exact_products<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    let inner = IntervalVector::new([scalar(-1, 1), scalar(2, 3)]).unwrap();
    let outer = IntervalVector::new([scalar(-2, 2), scalar(1, 4)]).unwrap();
    assert!(inner.interior_subset_of(&outer, &mut context()).unwrap());
    assert!(!outer.subset_of(&inner, &mut context()).unwrap());
    assert_eq!(inner.intersection(&outer, &mut context()).unwrap(), inner);
    assert_eq!(inner.hull(&outer, &mut context()).unwrap(), outer);
    let difference = outer.sub(&inner, &mut context()).unwrap();
    assert_eq!(difference.lanes(), Some(&[scalar(-3, 3), scalar(-2, 2)]));
    let m = IntervalMatrix::new([inner.into_lanes().unwrap()]).unwrap();
    let n = IntervalMatrix::new([outer.into_lanes().unwrap()]).unwrap();
    assert!(m.subset_of(&n, &mut context()).unwrap());
    assert!(m.interior_subset_of(&n, &mut context()).unwrap());
    assert!(!n.subset_of(&m, &mut context()).unwrap());
    assert_eq!(m.hull(&n, &mut context()).unwrap(), n);
    assert_eq!(m.intersection(&n, &mut context()).unwrap(), m);
    assert_eq!(
        n.sub(&m, &mut context()).unwrap().into_rows(),
        Some([difference.into_lanes().unwrap()])
    );
    let unit = IntervalMatrix::<I, 0, 2>::new([]).unwrap();
    assert!(unit.interior_subset_of(&unit, &mut context()).unwrap());
    assert_eq!(unit.into_rows(), Some([]));
    let ranges = [(-1, -1), (-1, 0), (-1, 1), (0, 0), (0, 1), (1, 1)];
    // Exhaust every two-lane dot product over this finite endpoint domain.
    for a0 in ranges {
        for a1 in ranges {
            for b0 in ranges {
                for b1 in ranges {
                    let a = IntervalVector::new([scalar(a0.0, a0.1), scalar(a1.0, a1.1)]).unwrap();
                    let b = IntervalVector::new([scalar(b0.0, b0.1), scalar(b1.0, b1.1)]).unwrap();
                    let expected = dot_bounds(&[a0, a1], &[b0, b1]);
                    assert_eq!(
                        a.dot(&b, &mut context()).unwrap(),
                        scalar(expected.0, expected.1)
                    );
                    let left = IntervalMatrix::new([
                        a.clone().into_lanes().unwrap(),
                        b.clone().into_lanes().unwrap(),
                    ])
                    .unwrap();
                    let right = IntervalMatrix::new([
                        [scalar(b0.0, b0.1), scalar(a0.0, a0.1)],
                        [scalar(b1.0, b1.1), scalar(a1.0, a1.1)],
                    ])
                    .unwrap();
                    let left_bounds = [[a0, a1], [b0, b1]];
                    let columns = [[b0, b1], [a0, a1]];
                    let expected_matrix = core::array::from_fn(|r| {
                        core::array::from_fn(|c| {
                            let bounds = dot_bounds(&left_bounds[r], &columns[c]);
                            scalar(bounds.0, bounds.1)
                        })
                    });
                    assert_eq!(
                        left.mul_matrix(&right, &mut context()).unwrap().rows(),
                        Some(&expected_matrix)
                    );
                    let row = IntervalMatrix::new([a.into_lanes().unwrap()]).unwrap();
                    let column = IntervalMatrix::new(b.into_lanes().unwrap().map(|x| [x])).unwrap();
                    let product = row.mul_matrix(&column, &mut context()).unwrap();
                    assert_eq!(
                        product.rows().unwrap()[0][0],
                        scalar(expected.0, expected.1)
                    );
                }
            }
        }
    }
    // Every 2x2 point matrix and every 2D input box on the same endpoint grid.
    for a in -1..=1 {
        for b in -1..=1 {
            for c in -1..=1 {
                for d in -1..=1 {
                    let matrix = IntervalMatrix::new([
                        [scalar(a, a), scalar(b, b)],
                        [scalar(c, c), scalar(d, d)],
                    ])
                    .unwrap();
                    for x in ranges {
                        for y in ranges {
                            let vector =
                                IntervalVector::new([scalar(x.0, x.1), scalar(y.0, y.1)]).unwrap();
                            let shift =
                                IntervalVector::new([scalar(2, 2), scalar(-3, -3)]).unwrap();
                            let output = matrix
                                .affine_enclosure(&vector, &shift, &mut context())
                                .unwrap();
                            let e0 = dot_bounds(&[(a, a), (b, b)], &[x, y]);
                            let e1 = dot_bounds(&[(c, c), (d, d)], &[x, y]);
                            assert_eq!(
                                output.lanes().unwrap(),
                                &[scalar(e0.0 + 2, e0.1 + 2), scalar(e1.0 - 3, e1.1 - 3)]
                            );
                            let restored = matrix
                                .transpose(&mut context())
                                .unwrap()
                                .transpose(&mut context())
                                .unwrap();
                            assert_eq!(restored, matrix);
                            // Consume generated output in another product, with independently
                            // computed expectations for its declared Cartesian enclosure.
                            let next = matrix.mul_vector(&output, &mut context()).unwrap();
                            let input = [(e0.0 + 2, e0.1 + 2), (e1.0 - 3, e1.1 - 3)];
                            let n0 = dot_bounds(&[(a, a), (b, b)], &input);
                            let n1 = dot_bounds(&[(c, c), (d, d)], &input);
                            assert_eq!(
                                next.lanes().unwrap(),
                                &[scalar(n0.0, n0.1), scalar(n1.0, n1.1)]
                            );
                        }
                    }
                }
            }
        }
    }
}

#[cfg(feature = "fixed")]
#[test]
fn exhaustive_fixed_exact_oracles() {
    exact_products(
        |a, b| hyperlattice_interval::hyperinterval::FixedInterval::<i32, 0>::new(a, b).unwrap(),
        || (),
    );
    exact_products(
        |a, b| {
            hyperlattice_interval::hyperinterval::FixedInterval::<i64, 0>::new(a.into(), b.into())
                .unwrap()
        },
        || (),
    );
}

#[cfg(feature = "rational")]
#[test]
fn exhaustive_rational_exact_oracles() {
    exact_products(
        |a, b| {
            hyperlattice_interval::hyperinterval::RationalInterval::new(a.into(), b.into()).unwrap()
        },
        || (),
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn exhaustive_dyadic_exact_oracles() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval};
    exact_products(
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
fn exhaustive_strict_float_exact_oracles() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    exact_products(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    exact_products(|a, b| F64Enclosure::new(a.into(), b.into()).unwrap(), || ());
}

#[cfg(feature = "fixed")]
mod fixed {
    use super::*;
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    use hyperlattice_interval::{BoxN, IntervalLatticeError};
    type I = FixedInterval<i32, 2>;
    fn i(a: i32, b: i32) -> I {
        I::new(a, b).unwrap()
    }

    #[test]
    fn fractional_products_are_outward_and_overflow_is_typed() {
        let ranges: Vec<_> = (-2..=2)
            .flat_map(|a| (a..=2).map(move |b| (a, b)))
            .collect();
        for a in &ranges {
            for b in &ranges {
                for c in &ranges {
                    for d in &ranges {
                        let x = IntervalVector::new([i(a.0, a.1), i(b.0, b.1)]).unwrap();
                        let y = IntervalVector::new([i(c.0, c.1), i(d.0, d.1)]).unwrap();
                        let (lo, hi) = x.dot(&y, &mut ()).unwrap().bounds().unwrap();
                        let oracle = dot_bounds(&[*a, *b], &[*c, *d]);
                        assert!(lo * 4 <= oracle.0 && hi * 4 >= oracle.1);
                        // Two separately rounded products: at most two grid units of slack.
                        assert!(oracle.0 - lo * 4 < 8 && hi * 4 - oracle.1 < 8);
                    }
                }
            }
        }
        let huge = IntervalVector::new([i(i32::MAX, i32::MAX)]).unwrap();
        let two = IntervalVector::new([i(8, 8)]).unwrap();
        assert_eq!(
            huge.dot(&two, &mut ()),
            Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
        let identity = IntervalMatrix::new([[i(4, 4)]]).unwrap();
        assert_eq!(identity.mul_vector(&huge, &mut ()).unwrap(), huge);
        // Addition overflow after a representable product must also propagate.
        let huge_two = IntervalVector::new([i(i32::MAX, i32::MAX); 2]).unwrap();
        let ones = IntervalVector::new([i(4, 4); 2]).unwrap();
        let error = huge_two.dot(&ones, &mut ()).unwrap_err();
        assert_eq!(error, IntervalLatticeError::Scalar(IntervalError::Overflow));
        use core::error::Error;
        assert_eq!(
            error.source().unwrap().to_string(),
            "interval endpoint overflow"
        );
        assert_eq!(
            error.to_string(),
            "interval arithmetic failed: interval endpoint overflow"
        );
        let bad_index = identity.row(1, &mut ()).unwrap_err();
        assert!(bad_index.source().is_none());
        assert_eq!(bad_index.to_string(), "index 1 is outside dimension 1");
    }

    #[test]
    fn canonical_empty_unit_and_invalid_families() {
        let unit = IntervalVector::<I, 0>::new([]).unwrap();
        let empty = IntervalVector::<I, 0>::empty().unwrap();
        assert!(unit.is_singleton() && !unit.is_empty());
        assert!(empty.is_empty() && !empty.is_singleton());
        assert!(unit.interior_subset_of(&unit, &mut ()).unwrap());
        assert_eq!(unit.dot(&unit, &mut ()).unwrap(), i(0, 0));
        assert!(empty.dot(&unit, &mut ()).unwrap().is_empty());
        assert!(empty.subset_of(&unit, &mut ()).unwrap());
        assert!(!unit.subset_of(&empty, &mut ()).unwrap());
        let mixed = IntervalVector::new([I::empty().unwrap(), i(1, 3)]).unwrap();
        assert_eq!(mixed, BoxN::<I, 2>::empty().unwrap());
        let a = IntervalMatrix::<I, 2, 0>::new([[], []]).unwrap();
        let b = IntervalMatrix::<I, 0, 3>::new([]).unwrap();
        let product = a.mul_matrix(&b, &mut ()).unwrap();
        assert_eq!(product.rows(), Some(&[[i(0, 0); 3]; 2]));
        assert!(
            a.mul_matrix::<3>(&IntervalMatrix::empty().unwrap(), &mut ())
                .unwrap()
                .is_empty()
        );
        assert!(IntervalVector::<FixedInterval<i32, 32>, 0>::new([]).is_err());
        assert!(IntervalVector::<FixedInterval<i64, 64>, 0>::empty().is_err());
        assert!(IntervalMatrix::<FixedInterval<i32, 32>, 0, 4>::new([]).is_err());
        assert!(IntervalMatrix::<FixedInterval<i32, 32>, 3, 0>::empty().is_err());
    }

    #[test]
    fn relations_projections_and_empty_shortcuts() {
        let a = IntervalVector::new([i(-4, 4), i(0, 8)]).unwrap();
        let b = IntervalVector::new([i(-2, 2), i(2, 6)]).unwrap();
        assert!(b.interior_subset_of(&a, &mut ()).unwrap());
        assert!(!a.interior_subset_of(&a, &mut ()).unwrap());
        assert_eq!(a.hull(&b, &mut ()).unwrap(), a);
        assert_eq!(a.intersection(&b, &mut ()).unwrap(), b);
        let disjoint = IntervalVector::new([i(5, 6), i(2, 3)]).unwrap();
        let empty = a.intersection(&disjoint, &mut ()).unwrap();
        assert!(empty.is_empty());
        assert_eq!(empty.hull(&a, &mut ()).unwrap(), a);
        assert_eq!(a.hull(&empty, &mut ()).unwrap(), a);
        assert!(empty.hull(&empty, &mut ()).unwrap().is_empty());
        assert!(a.add(&empty, &mut ()).unwrap().is_empty());
        assert!(empty.sub(&a, &mut ()).unwrap().is_empty());
        assert_eq!(
            a.sub(&b, &mut ()).unwrap().lanes(),
            Some(&[i(-6, 6), i(-6, 6)])
        );
        let m = IntervalMatrix::new([a.into_lanes().unwrap(), b.into_lanes().unwrap()]).unwrap();
        assert_eq!(m.row(1, &mut ()).unwrap(), b);
        assert_eq!(
            m.column(0, &mut ()).unwrap().lanes(),
            Some(&[i(-4, 4), i(-2, 2)])
        );
        let e = IntervalMatrix::<I, 2, 2>::empty().unwrap();
        assert!(e.row(0, &mut ()).unwrap().is_empty());
        assert!(e.column(0, &mut ()).unwrap().is_empty());
        assert_eq!(
            e.row(2, &mut ()),
            Err(IntervalLatticeError::IndexOutOfBounds {
                index: 2,
                dimension: 2
            })
        );
        assert_eq!(
            m.column(2, &mut ()),
            Err(IntervalLatticeError::IndexOutOfBounds {
                index: 2,
                dimension: 2
            })
        );
        assert!(e.transpose(&mut ()).unwrap().is_empty());
        assert_eq!(m.hull(&e, &mut ()).unwrap(), m);
        assert_eq!(e.hull(&m, &mut ()).unwrap(), m);
        assert!(e.hull(&e, &mut ()).unwrap().is_empty());
        assert!(e.subset_of(&m, &mut ()).unwrap());
        assert!(e.interior_subset_of(&m, &mut ()).unwrap());
        assert!(!m.subset_of(&e, &mut ()).unwrap());
        assert!(!m.interior_subset_of(&m, &mut ()).unwrap());
        assert_eq!(m.intersection(&m, &mut ()).unwrap(), m);
        assert!(m.intersection(&e, &mut ()).unwrap().is_empty());
        assert!(m.add(&e, &mut ()).unwrap().is_empty());
        let zero = IntervalMatrix::new([[i(0, 0); 2]; 2]).unwrap();
        assert_eq!(m.sub(&zero, &mut ()).unwrap(), m);
        assert_eq!(m.add(&zero, &mut ()).unwrap(), m);
        assert!(m.mul_vector(&empty, &mut ()).unwrap().is_empty());
        let huge = IntervalMatrix::new([[i(i32::MAX, i32::MAX)]]).unwrap();
        let two = IntervalVector::new([i(8, 8)]).unwrap();
        assert!(
            huge.affine_enclosure(&two, &IntervalVector::empty().unwrap(), &mut ())
                .unwrap()
                .is_empty()
        );
    }
}

#[cfg(feature = "dyadic")]
#[test]
fn shared_budget_copies_exhaustion_and_retry() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let i = |n: i32| I::point(n.into(), 0).unwrap();
    let a = IntervalMatrix::new([[i(2), i(-3)], [i(5), i(7)]]).unwrap();
    let x = IntervalVector::new([i(11), i(13)]).unwrap();
    let mut one_row_budget = DyadicBudget::new(256, 1_000_000);
    let row = IntervalVector::new([i(2), i(-3)]).unwrap();
    row.dot(&x, &mut one_row_budget).unwrap();
    let spent = 1_000_000 - one_row_budget.remaining_work();
    let mut insufficient = DyadicBudget::new(256, spent);
    assert_eq!(
        a.mul_vector(&x, &mut insufficient),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    let result = a
        .mul_vector(&x, &mut DyadicBudget::new(256, 1_000_000))
        .unwrap();
    assert_eq!(result.lanes(), Some(&[i(-17), i(146)]));
    assert_eq!(
        a.transpose(&mut DyadicBudget::new(256, 0)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert!(a.row(0, &mut DyadicBudget::new(256, 0)).is_err());
    assert!(a.column(0, &mut DyadicBudget::new(256, 0)).is_err());
    assert!(
        a.hull(
            &IntervalMatrix::empty().unwrap(),
            &mut DyadicBudget::new(256, 0)
        )
        .is_err()
    );
    assert!(
        x.hull(
            &IntervalVector::empty().unwrap(),
            &mut DyadicBudget::new(256, 0)
        )
        .is_err()
    );
    for error in [
        a.mul_matrix(&a, &mut DyadicBudget::new(256, 0))
            .unwrap_err(),
        a.hull(&a, &mut DyadicBudget::new(256, 0)).unwrap_err(),
        a.intersection(&a, &mut DyadicBudget::new(256, 0))
            .unwrap_err(),
        a.subset_of(&a, &mut DyadicBudget::new(256, 0)).unwrap_err(),
        x.hull(&x, &mut DyadicBudget::new(256, 0)).unwrap_err(),
        x.intersection(&x, &mut DyadicBudget::new(256, 0))
            .unwrap_err(),
        x.subset_of(&x, &mut DyadicBudget::new(256, 0)).unwrap_err(),
        a.affine_enclosure(&x, &x, &mut DyadicBudget::new(256, 0))
            .unwrap_err(),
    ] {
        assert_eq!(
            error,
            IntervalLatticeError::Scalar(IntervalError::WorkExhausted)
        );
    }
    assert!(
        a.mul_vector(&x, &mut DyadicBudget::new(1, 1_000_000))
            .is_err()
    );
    assert_eq!(
        a.mul_vector(&x, &mut DyadicBudget::new(256, 1_000_000))
            .unwrap(),
        result
    );
}
