#![cfg(feature = "dyadic")]

use hyperinterval::{
    DyadicBudget, DyadicInterval as D, ExtendedInterval, IntervalError, IntervalSet2,
};

type E = ExtendedInterval<D>;
type S = IntervalSet2<D>;

fn d(lo: i64, hi: i64, exponent: i32) -> D {
    D::new(lo.into(), hi.into(), exponent).unwrap()
}
fn budget() -> DyadicBudget {
    DyadicBudget::new(128, 4096)
}

#[test]
fn finite_overlap_matches_exact_common_grid() {
    for e in -2_i32..=2 {
        for f in -2_i32..=2 {
            for a in -3..=3 {
                for b in a..=3 {
                    let x = d(a, b, e);
                    for c in -3..=3 {
                        for h in c..=3 {
                            let y = d(c, h, f);
                            let g = e.min(f);
                            let separated =
                                (b << (e - g)) < (c << (f - g)) || (h << (f - g)) < (a << (e - g));
                            assert_eq!(x.overlaps(&y, &mut budget()), Ok(!separated));
                            assert_eq!(x.disjoint(&y, &mut budget()), Ok(separated));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn exponent_extremes_touching_empty_and_exhausted_retry() {
    let tiny = d(1, 1, i32::MIN);
    let huge = d(1, 1, i32::MAX);
    assert_eq!(
        tiny.intersection(&huge, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let mut one = DyadicBudget::new(1, 1);
    assert_eq!(tiny.disjoint(&huge, &mut one), Ok(true));
    assert_eq!(one.remaining_work(), 0);
    assert_eq!(
        huge.overlaps(&tiny, &mut DyadicBudget::new(1, 1)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        huge.overlaps(&tiny, &mut DyadicBudget::new(1, 2)),
        Ok(false)
    );
    let coarse = d(1, 3, 1);
    let touching = d(6, 7, 0);
    assert_eq!(
        coarse.overlaps(&touching, &mut DyadicBudget::new(1, 100)),
        Err(IntervalError::SizeExhausted)
    );
    assert!(coarse.overlaps(&touching, &mut budget()).unwrap());
    for x in [D::empty(), huge] {
        let mut empty_budget = DyadicBudget::new(0, 0);
        assert!(D::empty().disjoint(&x, &mut empty_budget).unwrap());
        assert!(!x.overlaps(&D::empty(), &mut empty_budget).unwrap());
    }
}

#[test]
fn split_division_preserves_the_gap_and_closed_ray_endpoints() {
    let one = E::from_finite(d(1, 1, 0));
    let denominator = E::from_finite(d(-1, 1, 0));
    let quotient = one.div(&denominator, -4, &mut budget()).unwrap();
    assert_eq!(quotient.domain, hyperinterval::DomainCoverage::Partial);
    let rays = quotient.range;
    assert_eq!(rays.components().len(), 2);
    for (lo, hi, e, expected) in [
        (-1, 1, -1, false),
        (-2, -1, 0, true),
        (1, 1, 0, true),
        (0, 1, 0, true),
    ] {
        let connected = S::from_interval(E::from_finite(d(lo, hi, e)));
        assert_eq!(rays.overlaps(&connected, &mut budget()), Ok(expected));
        assert_eq!(connected.disjoint(&rays, &mut budget()), Ok(!expected));
    }
    let left = E::below(d(-1, -1, 0)).unwrap();
    let right = E::above(d(1, 1, 0)).unwrap();
    assert!(left.disjoint(&right, &mut budget()).unwrap());
    assert!(right.overlaps(&one, &mut budget()).unwrap());
    assert!(
        E::entire()
            .overlaps(&right, &mut DyadicBudget::new(0, 0))
            .unwrap()
    );
    assert!(
        S::empty()
            .disjoint(&rays, &mut DyadicBudget::new(0, 0))
            .unwrap()
    );
}

fn connected_inputs() -> Vec<(E, Option<(i64, i64)>)> {
    // The two outer ranks denote bound directions. Intersection of two closed
    // components depends only on endpoint order and includes equality.
    let mut components = vec![(E::empty(), None)];
    for lo in -3..=2 {
        for hi in lo.max(-2)..=3 {
            let component = match (lo, hi) {
                (-3, 3) => E::entire(),
                (-3, b) => E::below(d(b, b, 0)).unwrap(),
                (a, 3) => E::above(d(a, a, 0)).unwrap(),
                (a, b) => E::from_finite(d(a, b, 0)),
            };
            components.push((component, Some((lo, hi))));
        }
    }
    components
}

#[test]
fn all_one_and_two_component_unions_on_a_small_arrangement() {
    let components = connected_inputs();
    let intersects = |a: (i64, i64), b: (i64, i64)| a.0 <= b.1 && b.0 <= a.1;
    let mut unions = vec![(S::empty(), Vec::new())];
    for (a, ranks_a) in &components {
        if let Some(x) = ranks_a {
            unions.push((S::from_interval(a.clone()), vec![*x]));
        }
        for (b, ranks_b) in &components {
            let common = ranks_a.zip(*ranks_b).is_some_and(|(x, y)| intersects(x, y));
            assert_eq!(a.overlaps(b, &mut budget()), Ok(common));
            assert_eq!(a.disjoint(b, &mut budget()), Ok(!common));
            if let (Some(x), Some(y)) = (ranks_a, ranks_b)
                && x.1 < y.0
            {
                unions.push((
                    S::new(a.clone(), b.clone(), &mut budget()).unwrap(),
                    vec![*x, *y],
                ));
            }
        }
    }
    assert_eq!(unions.len(), 112);
    for (a, left) in &unions {
        for (b, right) in &unions {
            let common = left
                .iter()
                .any(|&x| right.iter().any(|&y| intersects(x, y)));
            assert_eq!(a.overlaps(b, &mut budget()), Ok(common));
            assert_eq!(a.disjoint(b, &mut budget()), Ok(!common));
        }
    }
}

#[test]
fn later_component_comparisons_share_work_and_retry_cleanly() {
    let rays = S::new(
        E::below(d(-1, -1, 0)).unwrap(),
        E::above(d(1, 1, 0)).unwrap(),
        &mut budget(),
    )
    .unwrap();
    let right = S::from_interval(E::above(d(2, 2, 0)).unwrap());
    let mut measured = budget();
    assert!(rays.overlaps(&right, &mut measured).unwrap());
    let used = budget().remaining_work() - measured.remaining_work();
    assert!(used > 0);
    assert_eq!(
        rays.disjoint(&right, &mut DyadicBudget::new(128, used - 1)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        rays.disjoint(&right, &mut DyadicBudget::new(128, used)),
        Ok(false)
    );
}

#[test]
fn sorted_disjoint_components_skip_pairs_already_proved_separate() {
    let points = |a, b| {
        S::new(
            E::from_finite(d(a, a, 0)),
            E::from_finite(d(b, b, 0)),
            &mut budget(),
        )
        .unwrap()
    };
    let left = points(-5, -3);
    let right = points(1, 3);
    // Each left component precedes the first right component. Their three-
    // and two-bit comparisons suffice for the entire pair of unions.
    let mut exact = DyadicBudget::new(3, 5);
    assert_eq!(left.disjoint(&right, &mut exact), Ok(true));
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(
        left.overlaps(&right, &mut DyadicBudget::new(3, 4)),
        Err(IntervalError::WorkExhausted)
    );
}

#[test]
fn unbounded_directions_and_adjacent_wide_endpoints() {
    let one = d(1, 1, 0);
    let high = (one.lower().unwrap().0 << 2048_usize) + 1_u8;
    let edge = &high << 1_usize;
    let below = E::below(D::point(high.clone(), 1).unwrap()).unwrap();
    let above = E::above(D::point(&edge + 1_u8, 0).unwrap()).unwrap();
    let far_below = E::below(d(1, 1, i32::MIN)).unwrap();
    let far_above = E::above(d(1, 1, i32::MAX)).unwrap();
    let mut none = DyadicBudget::new(0, 0);
    assert_eq!(below.overlaps(&far_below, &mut none), Ok(true));
    assert_eq!(above.disjoint(&far_above, &mut none), Ok(false));
    let touch = E::above(D::point(high, 1).unwrap()).unwrap();
    for (right, expected) in [(above, false), (touch, true)] {
        for (a, b) in [(&below, &right), (&right, &below)] {
            assert_eq!(
                a.overlaps(b, &mut DyadicBudget::new(64, 8192)),
                Err(IntervalError::SizeExhausted)
            );
            assert_eq!(
                a.overlaps(b, &mut DyadicBudget::new(2050, 8192)),
                Ok(expected)
            );
            assert_eq!(
                a.disjoint(b, &mut DyadicBudget::new(2050, 8192)),
                Ok(!expected)
            );
        }
    }
}

#[test]
fn normalization_preserves_budget_receipts_and_failure_precedence() {
    fn ordered_parts(a: &E, b: &E, work: &mut DyadicBudget) -> Result<Vec<E>, IntervalError> {
        if a.is_empty() {
            return Ok(if b.is_empty() {
                vec![]
            } else {
                vec![b.clone()]
            });
        }
        if b.is_empty() {
            return Ok(vec![a.clone()]);
        }
        if a.strictly_left_of(b, work)? {
            Ok(vec![a.clone(), b.clone()])
        } else if b.strictly_left_of(a, work)? {
            Ok(vec![b.clone(), a.clone()])
        } else {
            Ok(vec![a.hull(b, work)?])
        }
    }

    let inputs = connected_inputs();
    for (a, _) in &inputs {
        for (b, _) in &inputs {
            for bits in [0, 1, 128] {
                for units in [0, 1, 2, 3, 4, 8, 16, 64, 4096] {
                    let mut reference = DyadicBudget::new(bits, units);
                    let mut measured = reference.clone();
                    let expected = ordered_parts(a, b, &mut reference);
                    let actual = S::new(a.clone(), b.clone(), &mut measured)
                        .map(|set| set.components().to_vec());
                    assert_eq!(actual, expected);
                    assert_eq!(measured.remaining_work(), reference.remaining_work());
                }
            }
        }
    }
}
