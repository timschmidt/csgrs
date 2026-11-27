//! Allocation-request sentinel for primitive authored forms and translated-source conversion.
//!
//! The counter is process-global and requires an isolated, nonnested measured scope.
//! This finite zero-request assertion does not measure stack use or general heap bounds.

#![cfg(feature = "fixed")]
#[path = "../conformance/allocations.rs"]
/// Shared System allocator wrapper counting requests during the measured closure.
mod allocations;
use hyperlattice_interval::{
    CenteredMatrix, CenteredVector, IntervalArithmetic, IntervalMatrix, IntervalVector,
};

/// Exercise a 64-lane form, a 2-by-3 form, invalid radii and an empty projection.
/// The supplied copyable primitive point is nonnegative and its doubled value fits;
/// the second point is negative. Projection, translated conversion and ownership
/// extraction execute inside the caller's measurement; black_box consumes results.
fn check<I: IntervalArithmetic<Context = ()> + Copy>(point: I, negative: I) {
    let center = IntervalVector::new([point; 64]).unwrap();
    let form = CenteredVector::new(center, center).unwrap();
    std::hint::black_box(form.enclosure(&mut ()).unwrap());
    let source = form.affine_source(&mut ()).unwrap();
    std::hint::black_box(source.enclosure(&mut ()).unwrap());
    std::hint::black_box(form.into_parts());
    let m = IntervalMatrix::new([[point; 3]; 2]).unwrap();
    let form = CenteredMatrix::new(m, m).unwrap();
    std::hint::black_box(form.enclosure(&mut ()).unwrap());
    std::hint::black_box(form.into_parts());
    assert!(CenteredVector::new(center, IntervalVector::new([negative; 64]).unwrap()).is_err());
    assert!(CenteredMatrix::new(m, IntervalMatrix::new([[negative; 3]; 2]).unwrap()).is_err());
    std::hint::black_box(
        CenteredVector::<I, 0>::empty()
            .unwrap()
            .enclosure(&mut ())
            .unwrap(),
    );
}

#[test]
/// Require zero allocation requests/bytes for both fixed families and optional IEEE families.
/// The entire sequence runs within one counter scope; integer fixed inputs are raw
/// units while IEEE inputs are halves. This sentinel does not establish allocation
/// behavior for rational/dyadic coefficients or concurrent calls.
fn primitive_authored_forms_allocate_nothing() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    let (_, count, bytes) = allocations::measure(|| {
        check(
            FixedInterval::<i32, 8>::point(1).unwrap(),
            FixedInterval::point(-1).unwrap(),
        );
        check(
            FixedInterval::<i64, 32>::point(1).unwrap(),
            FixedInterval::point(-1).unwrap(),
        );
        #[cfg(feature = "float-enclosure")]
        {
            use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
            check(
                F32Enclosure::point(0.5).unwrap(),
                F32Enclosure::point(-0.5).unwrap(),
            );
            check(
                F64Enclosure::point(0.5).unwrap(),
                F64Enclosure::point(-0.5).unwrap(),
            );
        }
    });
    assert_eq!((count, bytes), (0, 0));
}
