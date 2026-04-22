#[cfg(all(feature = "realistic-backend", feature = "approx-backend"))]
#[test]
fn explicit_backend_types_can_coexist() {
    use realistic_blas::{ApproxBackend, RealisticBackend, Scalar, Vector3, ZeroStatus};

    let realistic: Scalar<RealisticBackend> = Scalar::try_from(3.0).unwrap();
    let approx: Scalar<ApproxBackend> = Scalar::<ApproxBackend>::approx(4.0, 0.01).unwrap();

    let realistic_vector =
        Vector3::<RealisticBackend>::new([realistic.clone(), realistic.clone(), realistic]);
    let approx_vector = Vector3::<ApproxBackend>::new([approx.clone(), approx.clone(), approx]);

    assert_eq!(realistic_vector.dot(&realistic_vector), Scalar::from(27));
    assert_eq!(
        approx_vector.dot(&approx_vector).zero_status(),
        ZeroStatus::NonZero
    );
}
