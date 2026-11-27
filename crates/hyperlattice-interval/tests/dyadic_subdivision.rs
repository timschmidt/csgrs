//! Exact dyadic partition and explicit outward-regridding regression witnesses.
//!
//! Literal endpoint expectations distinguish bisection from rounded enclosure transport.
//! Finite cases cover closed overlap, extreme scales, empty/unit sets, typed failures
//! and unchanged-source retry; they do not establish general solver termination.

#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalLatticeError};
/// Construct a fixture interval with mantissa endpoints a,b and binary exponent e.
/// Native normalization may change the stored exponent while preserving the set;
/// all callers supply ordered endpoints, and construction failure panics via unwrap.
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
/// Fresh deterministic allowance: 16,384 magnitude bits and 100,000,000 work units.
/// These caps admit ordinary fixtures and are neither elapsed-time nor heap-byte limits.
fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 100000000)
}
#[test]
/// Split the first coordinate of a two-dimensional box at the exact midpoint 1/2.
/// Literal children preserve the second interval; their hull recovers the source and
/// their intersection is exactly the shared midpoint slice. Both are source subsets.
fn exact_closed_partition_and_first_axis_geometry() {
    let source = BoxN::new([d(-3, 4, 0), d(9, 11, 0)]).unwrap();
    let [left, right] = source.bisect_axis(0, &mut budget()).unwrap().unwrap();
    assert_eq!(left, BoxN::new([d(-6, 1, -1), d(9, 11, 0)]).unwrap());
    assert_eq!(right, BoxN::new([d(1, 8, -1), d(9, 11, 0)]).unwrap());
    assert_eq!(left.hull(&right, &mut budget()).unwrap(), source);
    assert_eq!(
        left.intersection(&right, &mut budget()).unwrap(),
        BoxN::new([d(1, 1, -1), d(9, 11, 0)]).unwrap()
    );
    assert!(left.subset_of(&source, &mut budget()).unwrap());
    assert!(right.subset_of(&source, &mut budget()).unwrap());
}
#[test]
/// Distinguish no split, invalid axis and an unrepresentable exact half.
/// Empty and singleton coordinates return None; an invalid axis rejects even on empty
/// or unit products. Sufficient-budget bisection at the finest odd grid returns Overflow.
fn empty_point_invalid_axis_and_exact_half_failure() {
    let empty = BoxN::<DyadicInterval, 1>::empty().unwrap();
    assert!(empty.bisect_axis(0, &mut budget()).unwrap().is_none());
    assert!(matches!(
        empty.bisect_axis(1, &mut budget()),
        Err(IntervalLatticeError::IndexOutOfBounds {
            index: 1,
            dimension: 1
        })
    ));
    let point = BoxN::new([d(3, 3, 0)]).unwrap();
    assert!(point.bisect_axis(0, &mut budget()).unwrap().is_none());
    let unit = BoxN::<DyadicInterval, 0>::new([]).unwrap();
    assert!(matches!(
        unit.bisect_axis(0, &mut budget()),
        Err(IntervalLatticeError::IndexOutOfBounds { .. })
    ));
    let tiny = BoxN::new([d(0, 1, i32::MIN)]).unwrap();
    assert_eq!(
        tiny.bisect_axis(0, &mut budget()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
}
#[test]
/// Require a constrained bisection to fail without changing its source, then retry.
/// The small budget restricts both size and work; this test accepts either error and
/// does not establish which limit failed or a precise exhaustion threshold.
fn exhaustion_does_not_mutate_or_poison_retry() {
    let source = BoxN::new([d(-7, 9, 0), d(3, 8, -3)]).unwrap();
    let copy = source.clone();
    assert!(source.bisect_axis(1, &mut DyadicBudget::new(1, 0)).is_err());
    assert_eq!(source, copy);
    assert!(source.bisect_axis(1, &mut budget()).unwrap().is_some());
}

#[test]
/// Check literal outward neighbors on a coarser grid and exact finer-grid transport.
/// Source inclusion, empty/unit identities and retry after a constrained failure are
/// asserted separately. Coarser regridding is an enclosure, not an exact partition.
fn outward_grid_transport_preserves_empty_unit_and_exact_containment() {
    let source = BoxN::new([d(-3, 7, -3), d(1, 3, -4)]).unwrap();
    let result = source.regrid(-2, &mut budget()).unwrap();
    assert_eq!(result, BoxN::new([d(-2, 4, -2), d(0, 1, -2)]).unwrap());
    assert!(source.subset_of(&result, &mut budget()).unwrap());
    assert_eq!(source.regrid(-16, &mut budget()).unwrap(), source);
    let empty = BoxN::<DyadicInterval, 0>::empty().unwrap();
    let unit = BoxN::<DyadicInterval, 0>::new([]).unwrap();
    assert_eq!(empty.regrid(-64, &mut budget()).unwrap(), empty);
    assert_eq!(unit.regrid(-64, &mut budget()).unwrap(), unit);
    assert!(source.regrid(-100, &mut DyadicBudget::new(1, 0)).is_err());
    assert!(source.regrid(-2, &mut budget()).is_ok());
}

#[test]
/// Split across zero at the minimum exponent and compare children to a much larger box.
/// Literal child intervals retain the minimum grid. Six subsequent closed/interior
/// inclusion checks share one two-bit / 64-work-unit budget despite the full exponent
/// gap; enclosing-to-child inclusion is false. No common giant grid is materialized.
fn partition_children_fit_a_cartesian_interior_across_extreme_scales() {
    let source = BoxN::new([d(-1, 1, i32::MIN), d(-1, 1, 0)]).unwrap();
    let enclosing = BoxN::new([d(-1, 1, i32::MAX), d(-2, 2, 0)]).unwrap();
    let [left, right] = source.bisect_axis(0, &mut budget()).unwrap().unwrap();
    assert_eq!(left, BoxN::new([d(-1, 0, i32::MIN), d(-1, 1, 0)]).unwrap());
    assert_eq!(right, BoxN::new([d(0, 1, i32::MIN), d(-1, 1, 0)]).unwrap());
    let mut query = DyadicBudget::new(2, 64);
    for half in [left, right] {
        assert_eq!(half.subset_of(&enclosing, &mut query), Ok(true));
        assert_eq!(half.interior_subset_of(&enclosing, &mut query), Ok(true));
        assert_eq!(enclosing.subset_of(&half, &mut query), Ok(false));
    }
}
