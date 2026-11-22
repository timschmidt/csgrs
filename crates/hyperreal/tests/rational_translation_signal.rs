use hyperreal::{Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn check_root_bounds(value: &Real, subtracted: &Rational, negative: bool) {
    let [lo, hi] = value.certified_rational_interval(-96).unwrap();
    let (lo, hi) = if negative {
        (subtracted - hi, subtracted - lo)
    } else {
        (lo - subtracted, hi - subtracted)
    };
    assert!(lo > Rational::one());
    assert!(&lo * &lo < Rational::from(2));
    assert!(&hi * &hi > Rational::from(2));
    assert!(hi < Rational::from(2));
}

#[test]
fn all_rational_translations_keep_the_symbolic_operands_signal() {
    let q = Rational::fraction(7, 5).unwrap();
    let shift = Real::from(q.clone());
    let mut root = Real::from(2).sqrt().unwrap();
    let independent = root.clone();
    let stop = Arc::new(AtomicBool::new(false));
    root.abort(stop.clone());
    for warm in [false, true] {
        if warm {
            check_root_bounds(&root, &Rational::zero(), false);
        }
        // Attach before construction and toggle afterwards. No snapshot of a flag suffices.
        let values = [
            (&root + &shift, q.clone(), false),
            (&shift + &root, q.clone(), false),
            (&root - &shift, -&q, false),
            (&shift - &root, q.clone(), true),
            (&shift - (&root + &shift), Rational::zero(), true),
        ];
        stop.store(true, Ordering::Relaxed);
        assert!(root.certified_rational_interval(-96).is_none());
        for (value, _, _) in &values {
            assert!(value.certified_rational_interval(-96).is_none());
            assert!(value.cached_rational_interval(-96).is_none());
        }
        check_root_bounds(&independent, &Rational::zero(), false);
        stop.store(false, Ordering::Relaxed);
        for (value, offset, negative) in values {
            check_root_bounds(&value, &offset, negative);
        }
    }
}

#[test]
fn cached_constants_and_opaque_offsets_use_the_unique_refining_operand() {
    for (constant, lower, upper) in [(Real::pi(), 3, 4), (Real::e(), 2, 3)] {
        let independent = constant.clone();
        let mut value = constant;
        let stop = Arc::new(AtomicBool::new(false));
        value.abort(stop.clone());
        let other_stop = Arc::new(AtomicBool::new(true));
        for shift in [Rational::from(3), Rational::fraction(7, 5).unwrap()] {
            let mut q = Real::from(shift.clone());
            q.abort(other_stop.clone());
            let translated = [
                (&value + &q, shift.clone(), false),
                (&q + &value, shift.clone(), false),
                (&value - &q, -&shift, false),
                (&q - &value, shift, true),
            ];
            stop.store(true, Ordering::Relaxed);
            assert!(independent.certified_rational_interval(-32).is_some());
            for (result, _, _) in &translated {
                assert!(result.certified_dyadic_interval(-64).is_none());
            }
            stop.store(false, Ordering::Relaxed);
            for (result, offset, negated) in translated {
                let [lo, hi] = result.certified_rational_interval(-64).unwrap();
                let (lo, hi) = if negated {
                    (&offset - hi, &offset - lo)
                } else {
                    (lo - &offset, hi - &offset)
                };
                assert!(lo > Rational::from(lower) && hi < Rational::from(upper));
            }
        }
    }
}
