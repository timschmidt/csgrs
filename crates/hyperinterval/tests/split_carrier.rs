#[cfg(feature = "fixed")]
#[test]
fn primitive_carrier_is_copy_without_union_allocation() {
    use hyperinterval::{ExtendedInterval, FixedInterval, IntervalSet2, SignSet};
    type Set = IntervalSet2<FixedInterval<i32, 30>>;
    fn is_copy<T: Copy>() {}
    is_copy::<Set>();
    let empty = Set::empty();
    assert!(empty.is_empty());
    assert!(!empty.is_singleton());
    assert_eq!(empty.signs(), SignSet::Empty);
    assert_eq!(empty.components().len(), 0);
    assert_eq!(Set::from_interval(ExtendedInterval::empty()), empty);
    let point = Set::from_interval(ExtendedInterval::from_finite(
        FixedInterval::point(0).unwrap(),
    ));
    assert!(point.is_singleton());
    assert_eq!(point.signs(), SignSet::Zero);
    let copied = point;
    assert_eq!(copied, point);
}
