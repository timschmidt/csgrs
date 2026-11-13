//! Headless boundary consumer: 1 = success, -1 = invalid radius, -2 = overflow.
//! Selectors 1/2 return full endpoints only after status selector 0 succeeds.
use hyperlattice_interval::{
    AffineMap, CenteredMatrix, CenteredVector, IntervalLatticeError, IntervalMatrix,
    IntervalVector,
    hyperinterval::{F64Enclosure, FixedInterval, IntervalError},
};

#[unsafe(no_mangle)]
pub extern "C" fn translated_half(c: i32, r: i32, selector: u32) -> i64 {
    assert!(selector <= 2);
    type I = FixedInterval<i32, 31>;
    let p = |x| I::point(x).unwrap();
    let result = CenteredVector::new(
        IntervalVector::new([p(c)]).unwrap(),
        IntervalVector::new([p(r)]).unwrap(),
    )
    .and_then(|form| form.affine_source(&mut ()))
    .and_then(|source| {
        AffineMap::new(
            IntervalMatrix::new([[p(1 << 30)]]).unwrap(),
            IntervalVector::new([p(0)]).unwrap(),
        )
        .unwrap()
        .image(source)
        .enclosure(&mut ())
    });
    match result {
        Err(error) => failure(error),
        Ok(bounds) => {
            let (lower, upper) = bounds.lanes().unwrap()[0].bounds().unwrap();
            match selector {
                0 => 1,
                1 => i64::from(lower),
                _ => i64::from(upper),
            }
        }
    }
}

fn failure(error: IntervalLatticeError) -> i64 {
    match error {
        IntervalLatticeError::NegativeRadius => -1,
        IntervalLatticeError::Scalar(IntervalError::Overflow) => -2,
        _ => -99, // The independent consumer explicitly rejects this status.
    }
}

macro_rules! vector {
    ($name:ident, $endpoint:ty) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $name(c: $endpoint, r: $endpoint, selector: u32) -> i64 {
            assert!(selector <= 2);
            type I = FixedInterval<$endpoint, 0>;
            let center = IntervalVector::new([I::point(c).unwrap()]).unwrap();
            let radius = IntervalVector::new([I::point(r).unwrap()]).unwrap();
            let form = match CenteredVector::new(center, radius) {
                Ok(form) => form,
                Err(error) => return failure(error),
            };
            let result = form.enclosure(&mut ());
            assert_eq!(form.into_parts(), Some((center, radius)));
            match result {
                Err(error) => failure(error),
                Ok(bounds) => {
                    let (lower, upper) = bounds.lanes().unwrap()[0].bounds().unwrap();
                    match selector {
                        0 => 1,
                        1 => i64::from(lower),
                        _ => i64::from(upper),
                    }
                }
            }
        }
    };
}
vector!(vector_i32, i32);
vector!(vector_i64, i64);

#[unsafe(no_mangle)]
pub extern "C" fn matrix(c: i32, r: i32, row: u32, selector: u32) -> i64 {
    assert!(row <= 1 && selector <= 2);
    type I = FixedInterval<i32, 0>;
    let p = |x| I::point(x).unwrap();
    let form = match CenteredMatrix::new(
        IntervalMatrix::new([[p(c), p(0)], [p(0), p(c)]]).unwrap(),
        IntervalMatrix::new([[p(r), p(0)], [p(0), p(r)]]).unwrap(),
    ) {
        Ok(form) => form,
        Err(error) => return failure(error),
    };
    let result = form
        .enclosure(&mut ())
        .and_then(|m| m.mul_vector(&IntervalVector::new([p(1), p(-1)]).unwrap(), &mut ()));
    match result {
        Err(error) => failure(error),
        Ok(result) => {
            let (lower, upper) = result.lanes().unwrap()[row as usize].bounds().unwrap();
            match selector {
                0 => 1,
                1 => i64::from(lower),
                _ => i64::from(upper),
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn empty_unit_and_qualification() -> u32 {
    type I = FixedInterval<i32, 31>;
    let empty = CenteredVector::<I, 0>::empty().unwrap();
    assert!(empty.enclosure(&mut ()).unwrap().is_empty());
    let unit = CenteredVector::new(
        IntervalVector::<I, 0>::new([]).unwrap(),
        IntervalVector::new([]).unwrap(),
    )
    .unwrap();
    assert!(unit.enclosure(&mut ()).unwrap().is_singleton());
    u32::from(F64Enclosure::point(0.0) == Err(IntervalError::UnqualifiedEnvironment))
}
