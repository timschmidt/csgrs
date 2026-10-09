use super::Point2;
use hyperreal::{Real, ZeroKnowledge};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn origin(kind: usize) -> [Real; 2] {
    match kind {
        0 => [Real::zero(), Real::zero()],
        1 => [Real::pi(), Real::e()],
        2 => [
            Real::from(7).sin() + Real::from(53).sqrt().unwrap(),
            Real::from(3).sqrt().unwrap() + Real::from(5).cos(),
        ],
        _ => unreachable!(),
    }
}

fn translated(origin: &[Real; 2], scale: &Real, x: i32, y: i32) -> Point2 {
    Point2::new(
        (&origin[0] + Real::from(x)) * scale,
        (&origin[1] + Real::from(y)) * scale,
    )
}

fn prepare(point: &Point2, precision: i32) {
    for coordinate in [point.x(), point.y()] {
        assert!(coordinate.certified_rational_interval(precision).is_some());
    }
}

#[test]
fn point_equality_matches_integer_oracle_before_and_after_preparation() {
    for kind in 0..3 {
        let origin = origin(kind);
        for scale in ["1", "-3", "1e400", "1e-400"] {
            let scale: Real = scale.parse().unwrap();
            for prepared in [false, true] {
                for ax in -2..=2 {
                    for ay in -2..=2 {
                        let left = translated(&origin, &scale, ax, ay);
                        for bx in -2..=2 {
                            for by in -2..=2 {
                                let right = translated(&origin, &scale, bx, by);
                                if prepared {
                                    prepare(&left, -128);
                                    prepare(&right, -128);
                                }
                                // Translation by any common real and scaling
                                // by a nonzero rational are injective. The
                                // expected equality uses only input integers.
                                let expected = ax == bx && ay == by;
                                assert_eq!(left == right, expected);
                                assert_eq!(right == left, expected);
                                assert_eq!(left.clone() == right.clone(), expected);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn overlapping_cached_enclosures_do_not_merge_tiny_separations() {
    let epsilon: Real = "1e-40".parse().unwrap();
    for scale in ["1", "-3", "1e400", "1e-400"] {
        let scale: Real = scale.parse().unwrap();
        for axis in 0..2 {
            let left = translated(&origin(1), &scale, 0, 0);
            let mut shifted = origin(1);
            shifted[axis] = &shifted[axis] + &epsilon;
            let right = translated(&shifted, &scale, 0, 0);
            for precision in [-8, -128, -16, -96] {
                prepare(&left, precision);
                prepare(&right, precision);
                if precision <= -32 {
                    let a = [left.x(), left.y()][axis]
                        .certified_rational_interval(-32)
                        .unwrap();
                    let b = [right.x(), right.y()][axis]
                        .certified_rational_interval(-32)
                        .unwrap();
                    assert!(a[0] <= b[1] && b[0] <= a[1]);
                }
                assert_ne!(left, right);
                assert_eq!(left, Point2::new(left.x().clone(), left.y().clone()));
            }
        }
    }
}

#[test]
fn unresolved_equality_remains_inconclusive_with_warm_enclosures() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
    assert_eq!(unresolved_zero.zero_status(), ZeroKnowledge::Unknown);
    let left = Point2::from_values(0, 0);
    let right = Point2::new(unresolved_zero, Real::zero());
    for precision in [-8, -128, -16, -96] {
        assert_ne!(left, right);
        prepare(&right, precision);
        assert_ne!(left, right);
    }
}

#[test]
fn aborted_cached_operands_keep_exact_equality_behavior() {
    let mut coordinate = origin(2)[0].clone();
    coordinate.certified_rational_interval(-128).unwrap();
    let other = &coordinate + Real::from(3);
    other.certified_rational_interval(-128).unwrap();
    let signal = Arc::new(AtomicBool::new(true));
    coordinate.abort(signal.clone());
    assert!(coordinate.certified_rational_interval(-32).is_none());
    let left = Point2::new(coordinate, Real::zero());
    let right = Point2::new(other, Real::zero());
    assert_eq!(left, left.clone());
    assert_ne!(left, right);
    signal.store(false, Ordering::Relaxed);
    assert_ne!(left, right);
}

#[test]
fn shared_point_comparisons_are_stable_across_threads() {
    let origin = origin(2);
    let scale: Real = "-3".parse().unwrap();
    let left = translated(&origin, &scale, 7, -4);
    let equal = translated(&origin, &scale, 7, -4);
    let distinct = translated(&origin, &scale, 7, -3);
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                for precision in [-8, -128, -16, -96] {
                    prepare(&left, precision);
                    assert_eq!(left, equal);
                    assert_ne!(left, distinct);
                }
            });
        }
    });
}
