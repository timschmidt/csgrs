//! This integration test is a separate consumer crate with Rust's default
//! recursion limit. A larger limit in the library must not hide a failure to
//! use its native carriers with standard thread and unwind bounds.
use hypercurve::{
    BezierParallelVerificationOptions, Curve2, CurveParallelFidelityMetric2, CurvePath2,
    CurveRegion2, LineSeg2, Point2, Real,
};

fn standard_traits<T: Send + Sync + std::panic::UnwindSafe + std::panic::RefUnwindSafe>() {}

#[test]
fn native_curve_carriers_keep_standard_consumer_traits() {
    standard_traits::<Curve2>();
    standard_traits::<CurvePath2>();
    standard_traits::<CurveRegion2>();
    standard_traits::<std::sync::Arc<std::sync::Mutex<Vec<CurveRegion2>>>>();
}

#[test]
fn generated_curve_keeps_source_after_thread_transfer_and_unwind() {
    let source = CurvePath2::try_new(vec![
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(2, 0))
            .unwrap()
            .into(),
    ])
    .unwrap();
    let options =
        BezierParallelVerificationOptions::try_new((Real::one() / Real::from(20)).unwrap(), 16)
            .unwrap();
    let output = source
        .approximate_parallel_blend2d_certified(Real::one(), &options)
        .unwrap();
    let curve = output.path().curves()[0].clone();
    let curve = std::thread::spawn(move || curve.reversed().unwrap())
        .join()
        .unwrap();
    assert_eq!(
        curve.start().coordinates(),
        Some(&Point2::from_values(2, 1))
    );
    assert_eq!(curve.end().coordinates(), Some(&Point2::from_values(0, 1)));
    let original_bounds = curve.bounds().unwrap();
    assert!(
        std::panic::catch_unwind(|| {
            assert_eq!(curve.bounds().unwrap(), original_bounds);
            panic!("discard this consumer operation");
        })
        .is_err()
    );
    assert_eq!(curve.bounds().unwrap(), original_bounds);
    let receipt = curve.parallel_fidelity().unwrap();
    assert_eq!(receipt.source_path(), &source);
    assert_eq!(receipt.source_curve_index(), 0);
    assert_eq!(receipt.source_distance(), &Real::one());
    assert_eq!(receipt.metric(), CurveParallelFidelityMetric2::ExactImage);
    assert_eq!(receipt.error_bound(), Real::zero());
}
