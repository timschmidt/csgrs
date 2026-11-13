//! Retained authored definitions and complete endpoint projections.
use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F32Enclosure, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{
    CenteredMatrix, CenteredVector, IntervalArithmetic, IntervalMatrix, IntervalVector,
};
use std::{hint::black_box, time::Instant};
mod allocations;

fn family<I: IntervalArithmetic + PartialEq + core::fmt::Debug, const N: usize>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let center =
        IntervalVector::new(core::array::from_fn(|i| scalar(i as i32 - 3, i as i32 - 3))).unwrap();
    let radius = IntervalVector::new(core::array::from_fn(|i| {
        scalar((i % 3) as i32, (i % 3) as i32)
    }))
    .unwrap();
    let form = CenteredVector::<I, N>::new(center.clone(), radius.clone()).unwrap();
    let expected = IntervalVector::new(core::array::from_fn(|i| {
        scalar(i as i32 - 3 - (i % 3) as i32, i as i32 - 3 + (i % 3) as i32)
    }))
    .unwrap();
    assert_eq!(form.enclosure(&mut context()).unwrap(), expected);
    let mut reference_context = context();
    let lower = center.sub(&radius, &mut reference_context).unwrap();
    let upper = center.add(&radius, &mut reference_context).unwrap();
    assert_eq!(
        lower.hull(&upper, &mut reference_context).unwrap(),
        expected
    );
    let matrix = CenteredMatrix::new(
        IntervalMatrix::new(core::array::from_fn(|_| {
            core::array::from_fn(|_| scalar(3, 3))
        }))
        .unwrap(),
        IntervalMatrix::new(core::array::from_fn(|_| {
            core::array::from_fn(|_| scalar(1, 1))
        }))
        .unwrap(),
    )
    .unwrap();
    let expected_matrix = IntervalMatrix::<I, 2, 3>::new(core::array::from_fn(|_| {
        core::array::from_fn(|_| scalar(2, 4))
    }))
    .unwrap();
    assert_eq!(matrix.enclosure(&mut context()).unwrap(), expected_matrix);
    eprintln!(
        "{name},{N},vector_bytes={},definition_bytes={},pair_bytes={},matrix_definition_bytes={}",
        core::mem::size_of_val(&center),
        core::mem::size_of_val(&form),
        core::mem::size_of::<(IntervalVector<I, N>, IntervalVector<I, N>)>(),
        core::mem::size_of_val(&matrix)
    );
    let mut owned = Some((center.clone(), radius.clone()));
    let translated = form.affine_source(&mut context()).unwrap();
    assert_eq!(translated.enclosure(&mut context()).unwrap(), expected);
    eprintln!(
        "{name},{N},translated_bytes={}",
        core::mem::size_of_val(&translated)
    );
    for action in 0..6 {
        let mut execute = || match action {
            0 => {
                let (c, r) = black_box(owned.take().unwrap());
                let form = CenteredVector::new(c, r).unwrap();
                owned = Some(black_box(form.into_parts().unwrap()));
            }
            1 => {
                black_box(black_box(&form).enclosure(&mut context()).unwrap());
            }
            2 => {
                let (c, r) = (black_box(&center), black_box(&radius));
                let mut context = context();
                let low = c.sub(r, &mut context).unwrap();
                let high = c.add(r, &mut context).unwrap();
                black_box(low.hull(&high, &mut context).unwrap());
            }
            4 => {
                black_box(black_box(&form).affine_source(&mut context()).unwrap());
            }
            5 => {
                black_box(black_box(&translated).enclosure(&mut context()).unwrap());
            }
            _ => {
                black_box(black_box(&matrix).enclosure(&mut context()).unwrap());
            }
        };
        for _ in 0..32 {
            execute();
        }
        for sample in 0..20 {
            let (elapsed, calls, bytes) = allocations::measure(|| {
                let start = Instant::now();
                for _ in 0..128 {
                    execute();
                }
                start.elapsed().as_nanos()
            });
            if primitive {
                assert_eq!((calls, bytes), (0, 0));
            }
            println!("{name},{N},{action},{sample},128,{elapsed},{calls},{bytes}");
        }
    }
    assert_eq!(owned.unwrap(), form.into_parts().unwrap());
}

fn main() {
    println!("family,dimension,action,sample,repetitions,nanoseconds,allocations,bytes");
    family::<_, 4>(
        "i32q16",
        |a, b| FixedInterval::<i32, 16>::new(a << 16, b << 16).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "i32q16",
        |a, b| FixedInterval::<i32, 16>::new(a << 16, b << 16).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "i64q32",
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "i64q32",
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "f32",
        |a, b| F32Enclosure::new(a as f32, b as f32).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "f32",
        |a, b| F32Enclosure::new(a as f32, b as f32).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "f64",
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "f64",
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "rational",
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
        false,
    );
    family::<_, 64>(
        "rational",
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
        false,
    );
    family::<_, 4>(
        "dyadic",
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
    family::<_, 64>(
        "dyadic",
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
}
