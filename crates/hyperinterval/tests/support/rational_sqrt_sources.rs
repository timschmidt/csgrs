use hyperinterval::RationalInterval;
use hyperreal::Rational;

use crate::coordinate;

// Construct one source at a time. Retrying an earlier case must not warm this one.
pub fn source(case: u8, warm: bool) -> (RationalInterval, i64) {
    let right = coordinate(281_474_976_710_662., 3.);
    assert_eq!(right.reduction_input_bits(), 98);
    if warm {
        assert_eq!(right.numerator().bits(), 2);
    }
    let (left, lower_root) = match case {
        0 => return (RationalInterval::point(right), 27),
        1 => (Rational::from(3), 27),
        2 => (coordinate(6., 2.), 22),
        _ => unreachable!(),
    };
    (RationalInterval::new(left, right).unwrap(), lower_root)
}
