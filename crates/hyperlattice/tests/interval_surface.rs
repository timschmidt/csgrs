#![cfg(feature = "interval-rational")]
use hyperlattice::interval::hyperinterval::RationalInterval;
use hyperlattice::{BoxN, IntervalMatrix, IntervalVector};

#[test]
fn shared_interval_types_are_consumable_through_hyperlattice() {
    let interval = |a: i32, b: i32| RationalInterval::new(a.into(), b.into()).unwrap();
    let matrix = IntervalMatrix::new([[interval(2, 2), interval(-1, -1)]]).unwrap();
    let vector: BoxN<_, 2> = IntervalVector::new([interval(1, 3), interval(4, 5)]).unwrap();
    let output = matrix.mul_vector(&vector, &mut ()).unwrap();
    assert_eq!(output.lanes(), Some(&[interval(-3, 2)]));
    // This is the companion's actual type, not a wrapper or duplicate kernel.
    let _: hyperlattice::interval::IntervalVector<_, 1> = output;
}
