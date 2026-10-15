#![cfg(feature = "curve")]
//! Exact line profiles keep their meaning across triangulation and extrusion.
use csgrs::{GeometryCertainty, GeometryContext, Real, curve};
use hypercurve::{Contour2, CurveRegion2};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn profile(shift: &Real) -> CurveRegion2 {
    let rectangle = |x0: Real, y0: Real, x1: Real, y1: Real| {
        Contour2::from_real_ring(&[
            [&x0 + shift, y0.clone()],
            [&x1 + shift, y0.clone()],
            [&x1 + shift, y1.clone()],
            [&x0 + shift, y1],
            [&x0 + shift, y0],
        ])
        .unwrap()
    };
    CurveRegion2::try_from_native_contours(
        vec![rectangle(0.into(), 0.into(), q(10, 3), q(7, 3))],
        vec![rectangle(q(1, 3), q(1, 3), q(2, 3), q(2, 3))],
    )
    .unwrap()
}
fn validate(shift: Real) {
    let region = profile(&shift);
    assert_eq!(region.filled_area().unwrap(), Some(q(23, 3)));
    for context in [&GeometryContext::STRICT, &GeometryContext::APPROXIMATE_512] {
        let flat = curve::try_triangulate(&region, context).unwrap();
        assert_eq!(flat.certainty, GeometryCertainty::Certified);
        let twice_area = flat.value.triangles.iter().fold(Real::zero(), |sum, t| {
            let [a, b, c] = t.indices().map(|i| &flat.value.positions[i]);
            sum + (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x)
        });
        assert_eq!(twice_area, q(46, 3));
        let solid = curve::try_extrude(&region, 2.into(), context).unwrap();
        assert_eq!(solid.certainty, GeometryCertainty::Certified);
        assert!(
            solid
                .value
                .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let six_volume = solid.value.triangles.iter().fold(Real::zero(), |sum, t| {
            let [a, b, c] = t.indices().map(|i| solid.value.positions[i].to_vector());
            sum + a.dot(&b.cross(&c))
        });
        assert_eq!(six_volume, Real::from(92));
        let allowed_x = [Real::zero(), q(1, 3), q(2, 3), q(10, 3)].map(|x| x + &shift);
        assert!(solid.value.positions.iter().all(|p| allowed_x.contains(&p.x)));
    }
}
#[test]
fn rational_material_and_hole_keep_exact_area_and_volume() {
    validate(Real::zero());
}
#[test]
fn exact_profiles_beyond_float_range_are_still_modeling_inputs() {
    validate(Real::from(10).powi(400.into()).unwrap());
}

#[test]
fn extrusion_adapters_retain_native_failure_categories_and_sources() {
    use csgrs::adapter::{AdapterError, RawCurveRegion};
    use csgrs::errors::ValidationErrorKind;
    use std::error::Error;

    let region = RawCurveRegion::from_native(profile(&Real::zero()));
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unknown_zero = &sine * &sine + &cosine * &cosine - Real::one();
    for error in [
        region.extrude(unknown_zero.clone(), &GeometryContext::STRICT),
        region.extrude_vector(
            [Real::zero(), Real::zero(), unknown_zero],
            &GeometryContext::STRICT,
        ),
    ] {
        let error = error.unwrap_err();
        let AdapterError::Construction(native) = &error else {
            panic!("lost native extrusion diagnostic: {error:?}");
        };
        assert_eq!(native.kind(), ValidationErrorKind::Undecided);
        assert!(error.source().is_some());
    }
    let zero = region
        .extrude(Real::zero(), &GeometryContext::STRICT)
        .unwrap_err();
    let AdapterError::Construction(native) = zero else {
        panic!("lost invalid-input diagnostic: {zero:?}");
    };
    assert_eq!(native.kind(), ValidationErrorKind::InvalidInput);
    assert!(region.extrude(Real::one(), &GeometryContext::STRICT).is_ok());
}
