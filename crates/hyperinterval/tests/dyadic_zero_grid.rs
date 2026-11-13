#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval as D, IntervalError};

fn d(a: i32, b: i32, e: i32) -> D {
    D::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> DyadicBudget {
    DyadicBudget::new(64, 1000)
}

#[test]
fn zero_does_not_force_an_irrelevant_grid() {
    let zero = d(0, 0, 0);
    for exponent in [
        i32::MIN,
        i32::MIN + 1,
        -1000,
        -17,
        -1,
        0,
        1,
        17,
        1000,
        i32::MAX - 1,
        i32::MAX,
    ] {
        for a in -7..=7 {
            for b in a..=7 {
                let x = d(a, b, exponent);
                assert_eq!(x.add(&zero, &mut budget()).unwrap(), x);
                assert_eq!(zero.add(&x, &mut budget()).unwrap(), x);
                assert_eq!(x.sub(&zero, &mut budget()).unwrap(), x);
                assert_eq!(zero.sub(&x, &mut budget()).unwrap(), d(-b, -a, exponent));
                let hull = d(a.min(0), b.max(0), exponent);
                assert_eq!(x.hull(&zero, &mut budget()).unwrap(), hull);
                assert_eq!(zero.hull(&x, &mut budget()).unwrap(), hull);
                let intersection = if a <= 0 && b >= 0 {
                    zero.clone()
                } else {
                    D::empty()
                };
                assert_eq!(x.intersection(&zero, &mut budget()).unwrap(), intersection);
                assert_eq!(zero.intersection(&x, &mut budget()).unwrap(), intersection);
                assert_eq!(
                    x.min_image(&zero, &mut budget()).unwrap(),
                    d(a.min(0), b.min(0), exponent)
                );
                assert_eq!(
                    zero.min_image(&x, &mut budget()).unwrap(),
                    d(a.min(0), b.min(0), exponent)
                );
                assert_eq!(
                    x.max_image(&zero, &mut budget()).unwrap(),
                    d(a.max(0), b.max(0), exponent)
                );
                assert_eq!(
                    zero.max_image(&x, &mut budget()).unwrap(),
                    d(a.max(0), b.max(0), exponent)
                );
                assert_eq!(zero.subset_of(&x, &mut budget()).unwrap(), a <= 0 && b >= 0);
                assert_eq!(x.subset_of(&zero, &mut budget()).unwrap(), a == 0 && b == 0);
                assert_eq!(
                    zero.interior_subset_of(&x, &mut budget()).unwrap(),
                    a < 0 && b > 0
                );
                assert!(!x.interior_subset_of(&zero, &mut budget()).unwrap());
                assert_eq!(zero.strictly_left_of(&x, &mut budget()).unwrap(), a > 0);
                assert_eq!(x.strictly_left_of(&zero, &mut budget()).unwrap(), b < 0);
            }
        }
        assert_eq!(zero.regrid(exponent, &mut budget()).unwrap(), zero);
    }
}

#[test]
fn genuinely_distinct_nonzero_scales_still_exhaust() {
    let tiny = d(1, 1, i32::MIN);
    let huge = d(1, 1, i32::MAX);
    for result in [
        huge.add(&tiny, &mut budget()),
        huge.sub(&tiny, &mut budget()),
        huge.hull(&tiny, &mut budget()),
    ] {
        assert_eq!(result, Err(IntervalError::SizeExhausted));
    }
    assert_eq!(
        d(-1, 1, 0).add(&tiny, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(0, 1, 0).sub(&tiny, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let empty = D::empty();
    assert_eq!(
        empty.add(&huge, &mut DyadicBudget::new(0, 0)).unwrap(),
        empty
    );
    assert_eq!(
        empty
            .intersection(&tiny, &mut DyadicBudget::new(0, 0))
            .unwrap(),
        empty
    );
}

// Every operation retains its own result type; comparisons are mapped to a
// point only in this test dispatcher so the resource sweep can share a loop.
fn operation(which: usize, a: &D, b: &D, budget: &mut DyadicBudget) -> Result<D, IntervalError> {
    match which {
        0 => a.add(b, budget),
        1 => a.sub(b, budget),
        2 => a.hull(b, budget),
        3 => a.intersection(b, budget),
        4 => a.min_image(b, budget),
        5 => a.max_image(b, budget),
        6 => a
            .subset_of(b, budget)
            .map(|v| d(i32::from(v), i32::from(v), 0)),
        7 => a
            .interior_subset_of(b, budget)
            .map(|v| d(i32::from(v), i32::from(v), 0)),
        8 => a
            .strictly_left_of(b, budget)
            .map(|v| d(i32::from(v), i32::from(v), 0)),
        _ => unreachable!(),
    }
}

#[test]
fn work_thresholds_large_mantissas_and_retry() {
    use num_bigint::BigInt;
    let zero = d(0, 0, 0);
    for bits in [1, 7, 64, 257] {
        let magnitude: BigInt = (BigInt::from(1) << bits) - 1;
        for exponent in [i32::MIN, -17, 0, 17, i32::MAX] {
            for (lo, hi) in [
                (-&magnitude, magnitude.clone()),
                (magnitude.clone(), magnitude.clone()),
            ] {
                let x = D::new(lo, hi, exponent).unwrap();
                let before = x.clone();
                for (a, b) in [(&x, &zero), (&zero, &x)] {
                    for which in 0..9 {
                        let mut ample = DyadicBudget::new(258, 10000);
                        let expected = operation(which, a, b, &mut ample).unwrap();
                        let cost = 10000 - ample.remaining_work();
                        assert!(cost > 0);
                        for work in 0..cost {
                            assert_eq!(
                                operation(which, a, b, &mut DyadicBudget::new(258, work)),
                                Err(IntervalError::WorkExhausted)
                            );
                        }
                        assert_eq!(
                            operation(which, a, b, &mut DyadicBudget::new(258, cost)).unwrap(),
                            expected
                        );
                        // The mantissa still has to fit; zero does not bypass
                        // the size contract for the nonzero operand.
                        assert_eq!(
                            operation(which, a, b, &mut DyadicBudget::new(0, 10000)),
                            Err(IntervalError::SizeExhausted)
                        );
                    }
                }
                assert_eq!(x, before);
                assert_eq!(
                    zero.add(&x, &mut DyadicBudget::new(bits + 1, 10000))
                        .unwrap(),
                    x
                );
            }
        }
    }
    for exponent in [i32::MIN, -1, 0, 1, i32::MAX] {
        let cost = if exponent <= 0 { 2 } else { 4 };
        for work in 0..cost {
            assert_eq!(
                zero.regrid(exponent, &mut DyadicBudget::new(0, work)),
                Err(IntervalError::WorkExhausted)
            );
        }
        assert_eq!(
            zero.regrid(exponent, &mut DyadicBudget::new(0, cost))
                .unwrap(),
            zero
        );
    }
}

#[test]
fn nonzero_addition_preserves_exact_output_and_accounting() {
    // Independently align small signed integers on an i64 grid. No scalar
    // arithmetic or interval wrapper supplies the expectation.
    for e in -7..=7 {
        for f in -7..=7 {
            for (a, b) in [(-7, -1), (-3, 5), (1, 7)] {
                for (c, z) in [(-5, -3), (-1, 3), (3, 7)] {
                    let x = d(a, b, e);
                    let y = d(c, z, f);
                    let grid = e.min(f);
                    let values = [
                        i64::from(a) << (e - grid),
                        i64::from(b) << (e - grid),
                        i64::from(c) << (f - grid),
                        i64::from(z) << (f - grid),
                    ];
                    let [aa, bb, cc, dd] = values;
                    let bits = |v: i32| u64::from(32 - v.unsigned_abs().leading_zeros());
                    let left = bits(a).max(bits(b)) + (e - grid) as u64;
                    let right = bits(c).max(bits(z)) + (f - grid) as u64;
                    let cost = 2 * (left + right + left.max(right) + 1);
                    let mut budget = DyadicBudget::new(64, cost);
                    assert_eq!(
                        x.add(&y, &mut budget).unwrap(),
                        D::new((aa + cc).into(), (bb + dd).into(), grid).unwrap()
                    );
                    assert_eq!(budget.remaining_work(), 0);
                    let mut budget = DyadicBudget::new(64, cost);
                    assert_eq!(
                        x.sub(&y, &mut budget).unwrap(),
                        D::new((aa - dd).into(), (bb - cc).into(), grid).unwrap()
                    );
                    assert_eq!(budget.remaining_work(), 0);
                }
            }
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_then_operation() {
    const CHILD: &str = "HYPERINTERVAL_ZERO_GRID_REPLAY";
    if let Ok(encoded) = std::env::var(CHILD) {
        let values: Vec<D> = serde_json::from_str(&encoded).unwrap();
        assert_eq!(serde_json::to_string(&values).unwrap(), encoded);
        for (index, exponent) in [i32::MIN, -17, 0, 17, i32::MAX].into_iter().enumerate() {
            let x = &values[index];
            assert_eq!(*x, d(-3, 7, exponent));
            assert_eq!(x.add(&d(0, 0, 0), &mut budget()).unwrap(), *x);
            assert_eq!(
                x.intersection(&d(0, 0, 0), &mut budget()).unwrap(),
                d(0, 0, 0)
            );
        }
        return;
    }
    let values: Vec<_> = [i32::MIN, -17, 0, 17, i32::MAX]
        .into_iter()
        .map(|e| d(-3, 7, e))
        .collect();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_then_operation",
            "--nocapture",
        ])
        .env(CHILD, serde_json::to_string(&values).unwrap())
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
