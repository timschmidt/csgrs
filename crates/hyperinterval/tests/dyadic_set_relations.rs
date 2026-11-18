#![cfg(feature = "dyadic")]

use hyperinterval::{
    DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalError, IntervalSet2,
};

fn budget() -> DyadicBudget {
    DyadicBudget::new(128, 4096)
}

fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}

#[test]
fn normalized_unions_match_every_cell_and_maximal_component() {
    // These eleven cells alternate endpoints and open intervals. The first
    // and last represent the unbounded exterior cells.
    let mask = |lo: i32, hi: i32| -> u16 {
        (-5..=5).fold(0, |bits, k| (bits << 1) | u16::from(lo <= k && k <= hi))
    };
    let mut palette = vec![(E::empty(), 0), (E::entire(), mask(-5, 5))];
    for a in -2..=2 {
        let point = || d(a.into(), a.into());
        palette.push((E::above(point()).unwrap(), mask(2 * a, 5)));
        palette.push((E::below(point()).unwrap(), mask(-5, 2 * a)));
        for b in a..=2 {
            palette.push((E::from_finite(d(a.into(), b.into())), mask(2 * a, 2 * b)));
        }
    }
    let mut unions = std::collections::BTreeMap::new();
    for (a, left) in &palette {
        for (b, right) in &palette {
            let image = IntervalSet2::new(a.clone(), b.clone(), &mut budget()).unwrap();
            let bits = left | right;
            let mut runs = Vec::new();
            let mut run = 0;
            for i in (0..11).rev() {
                if bits & (1 << i) != 0 {
                    run |= 1 << i;
                } else if run != 0 {
                    runs.push(run);
                    run = 0;
                }
            }
            if run != 0 {
                runs.push(run);
            }
            assert_eq!(image.components().len(), runs.len());
            for (component, run) in image.components().iter().zip(runs) {
                let (expected, _) = palette.iter().find(|(_, cells)| *cells == run).unwrap();
                assert_eq!(component, expected);
            }
            unions.entry(bits).or_insert(image);
        }
    }
    for (left, a) in &unions {
        for (right, b) in &unions {
            assert_eq!(a.subset_of(b, &mut budget()), Ok(left & !right == 0));
        }
    }
}

#[test]
fn extreme_singletons_remain_separable_through_extended_bounds() {
    let point = |exponent| DyadicInterval::point(1.into(), exponent).unwrap();
    let left = E::from_finite(point(i32::MIN));
    let right = E::from_finite(point(i32::MAX));
    let mut one = DyadicBudget::new(1, 1);
    assert_eq!(left.strictly_left_of(&right, &mut one), Ok(true));
    assert_eq!(one.remaining_work(), 0);
    assert_eq!(
        right.strictly_left_of(&left, &mut DyadicBudget::new(1, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        right.strictly_left_of(&left, &mut DyadicBudget::new(1, 1)),
        Ok(false)
    );
}
