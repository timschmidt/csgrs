#![no_main]

use hyperinterval::{FixedInterval, IntegerRange, IntervalError};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    type Fixed = FixedInterval<i32, 30>;
    if let Ok(value) = serde_json::from_slice::<Fixed>(data) {
        if let Some((lower, upper)) = value.bounds() {
            assert!(lower <= upper);
        }
        let encoded = serde_json::to_vec(&value).unwrap();
        assert_eq!(serde_json::from_slice::<Fixed>(&encoded).unwrap(), value);
    }
    assert!(serde_json::from_slice::<FixedInterval<i32, 32>>(data).is_err());
    if data.len() < 16 {
        return;
    }
    let values: [i32; 4] =
        core::array::from_fn(|i| i32::from_le_bytes(data[4 * i..4 * i + 4].try_into().unwrap()));
    let [a, b, c, d] = values;
    let proposed = Fixed::new(a, b);
    assert_eq!(proposed.is_ok(), a <= b);
    if a > b {
        assert_eq!(proposed, Err(IntervalError::ReversedEndpoints));
    }
    let first = Fixed::new(a.min(b), a.max(b)).unwrap();
    let second = Fixed::new(c.min(d), c.max(d)).unwrap();
    let common = first.intersection(&second);
    let hull = first.hull(&second);
    for point in [a, b, c, d, 0] {
        assert_eq!(
            common.contains_point(point),
            first.contains_point(point) && second.contains_point(point)
        );
        if first.contains_point(point) || second.contains_point(point) {
            assert!(hull.contains_point(point));
        }
    }
    assert!(first.subset_of(&hull));
    assert!(common.subset_of(&first));
    let integers = IntegerRange::new(a.min(b), a.max(b)).unwrap();
    assert_eq!(integers.signs(), first.signs());
    assert_eq!(integers.bounds(), first.bounds());
});
