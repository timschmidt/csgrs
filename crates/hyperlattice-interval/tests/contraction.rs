#![cfg(any(feature = "fixed", feature = "float-enclosure"))]
#[cfg(feature = "fixed")]
use hyperlattice_interval::IntervalArithmetic;
use hyperlattice_interval::hyperinterval::IntervalError;
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix};

#[cfg(feature = "fixed")]
#[test]
fn fixed_constants_extremes_and_unit_products() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    macro_rules! highest_scale {
        ($i:ty) => {{
            assert_eq!(<$i>::one(), Err(IntervalError::Overflow));
            let m = IntervalMatrix::new([[<$i>::point(1).unwrap()]]).unwrap();
            assert_eq!(
                m.identity_minus(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
            for result in [
                m.contraction_margin_one(&mut ()),
                m.contraction_margin_infinity(&mut ()),
            ] {
                assert_eq!(
                    result,
                    Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
                );
            }
            let empty = IntervalMatrix::<$i, 1, 1>::empty().unwrap();
            assert!(empty.identity_minus(&mut ()).unwrap().is_empty());
            assert!(empty.contraction_margin_one(&mut ()).unwrap().is_empty());
            assert!(
                empty
                    .contraction_margin_infinity(&mut ())
                    .unwrap()
                    .is_empty()
            );
            let unit = IntervalMatrix::<$i, 0, 0>::new([]).unwrap();
            assert_eq!(unit.identity_minus(&mut ()).unwrap(), unit);
            assert_eq!(
                unit.contraction_margin_one(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
        }};
    }
    highest_scale!(FixedInterval<i32, 31>);
    highest_scale!(FixedInterval<i64, 63>);
    assert_eq!(
        FixedInterval::<i32, 32>::one(),
        Err(IntervalError::InvalidScale)
    );
    assert_eq!(
        FixedInterval::<i64, 64>::one(),
        Err(IntervalError::InvalidScale)
    );
    type I = FixedInterval<i32, 0>;
    let m = IntervalMatrix::new([[I::point(i32::MIN).unwrap()]]).unwrap();
    assert_eq!(
        m.identity_minus(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    assert_eq!(
        m.contraction_margin_infinity(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    // A later off-diagonal subtraction fails after successful earlier entries.
    let m = IntervalMatrix::new([
        [I::point(1).unwrap(), I::point(i32::MIN).unwrap()],
        [I::point(0).unwrap(); 2],
    ])
    .unwrap();
    assert_eq!(
        m.identity_minus(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    assert_eq!(m.rows().unwrap()[0][0], I::point(1).unwrap());
}

#[cfg(feature = "float-enclosure")]
#[test]
fn subnormal_tails_and_boundary_margins_are_not_rounded_to_proofs() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    macro_rules! case {
        ($i:ty, $f:ty) => {{
            let p = |x| <$i>::point(x).unwrap();
            let tiny = <$f>::from_bits(1);
            let below_one = <$f>::from_bits((1.0 as $f).to_bits() - 1);
            let above_one = <$f>::from_bits((1.0 as $f).to_bits() + 1);
            let m = IntervalMatrix::new([[p(tiny)]]).unwrap();
            let expected = <$i>::new(below_one, 1.).unwrap();
            assert_eq!(
                m.identity_minus(&mut ()).unwrap().rows().unwrap()[0][0],
                expected
            );
            assert_eq!(m.contraction_margin_one(&mut ()).unwrap(), expected);
            assert_eq!(m.contraction_margin_infinity(&mut ()).unwrap(), expected);
            let m = IntervalMatrix::new([[p(-tiny)]]).unwrap();
            assert_eq!(
                m.identity_minus(&mut ()).unwrap().rows().unwrap()[0][0],
                <$i>::new(1., above_one).unwrap()
            );
            let m = IntervalMatrix::new([[p(1.)]]).unwrap();
            assert_eq!(m.contraction_margin_one(&mut ()).unwrap(), p(0.));
            let m = IntervalMatrix::new([[p(1.), p(tiny)], [p(0.), p(0.)]]).unwrap();
            // The exact margin is -tiny. Sequential outward norm/subtraction
            // includes zero and must never turn it into strict contraction.
            let result = m.contraction_margin_infinity(&mut ()).unwrap();
            assert!(p(-tiny).subset_of(&result));
            assert!(p(0.).subset_of(&result));
            let m = IntervalMatrix::new([[p(<$f>::MAX); 2]; 2]).unwrap();
            assert_eq!(
                m.contraction_margin_one(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
        }};
    }
    case!(F32Enclosure, f32);
    case!(F64Enclosure, f64);
}
