#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalLatticeError};
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 100000000)
}
#[test]
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
fn exhaustion_does_not_mutate_or_poison_retry() {
    let source = BoxN::new([d(-7, 9, 0), d(3, 8, -3)]).unwrap();
    let copy = source.clone();
    assert!(source.bisect_axis(1, &mut DyadicBudget::new(1, 0)).is_err());
    assert_eq!(source, copy);
    assert!(source.bisect_axis(1, &mut budget()).unwrap().is_some());
}

#[test]
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
