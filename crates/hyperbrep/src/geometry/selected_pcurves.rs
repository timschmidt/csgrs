//! Selected chart regression cases; the affine equations are independent oracles.

use super::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurveParameter2, CurveParameterRange2, QuadraticBezier2,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected() -> CurveParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
    .into()
}

fn retained() -> SurfaceIntersectionPcurve {
    let curve = Curve2::from(QuadraticBezier2::new(
        CurvePoint2::from_values(0, 0),
        CurvePoint2::new(q(1, 2), 0.into()),
        CurvePoint2::from_values(1, 1),
    ))
    .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), selected()).unwrap())
    .unwrap();
    SurfaceIntersectionPcurve::retained_curve(curve).unwrap()
}

#[test]
fn selected_retained_pcurve_materializes() {
    assert!(retained().materialize().is_ok());
}

#[test]
fn selected_iso_pcurve_materializes() {
    let domain = retained().domain;
    for axis in [TensorAxis::U, TensorAxis::V] {
        assert!(
            SurfaceIntersectionPcurve::tensor_iso(
                domain.clone(),
                3.into(),
                axis,
                (-2).into(),
                5.into(),
            )
            .materialize()
            .is_ok()
        );
    }
}

fn verify_selected_mapping(mapping: &SurfaceIntersectionPcurve) {
    let materialized = mapping.materialize().unwrap();
    if matches!(
        mapping.mapping,
        SurfaceIntersectionPcurveMapping::RetainedCurve { .. }
    ) {
        // The authored source is (t,t²); this is a separately derived point.
        let local = q(1, 2).into();
        assert!(
            materialized
                .curve()
                .point_at(&local)
                .unwrap()
                .coincides_with(&CurvePoint2::new(q(1, 2), q(1, 4)).into())
                .unwrap()
        );
        let expected = ((q(1, 2) - &mapping.source_offset) / &mapping.source_scale).unwrap();
        assert_eq!(
            materialized
                .spatial_parameter_at(&local)
                .unwrap()
                .compare(&expected.into())
                .unwrap(),
            Ordering::Equal
        );
    }
    let (first, last) = materialized.curve().parameter_endpoints();
    for (local, spatial) in [
        (first, mapping.domain.start()),
        (last, mapping.domain.end()),
    ] {
        assert_eq!(
            materialized
                .spatial_parameter_at(local)
                .unwrap()
                .compare(spatial)
                .unwrap(),
            Ordering::Equal
        );
        let source = spatial
            .curve_parameter()
            .affine_image(&mapping.source_scale, &mapping.source_offset)
            .unwrap();
        let expected = match &mapping.mapping {
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                curve.point_at(&source).unwrap()
            }
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => source
                .affine_point(0.into(), constant.clone(), 1.into(), 0.into())
                .unwrap(),
            SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => source
                .affine_point(1.into(), 0.into(), 0.into(), constant.clone())
                .unwrap(),
            _ => unreachable!(),
        };
        assert!(
            expected
                .coincides_with(&materialized.curve().point_at(local).unwrap())
                .unwrap()
        );
    }
    let mut builder = crate::ModelBuilder::new();
    builder
        .pcurve(crate::Pcurve::new(materialized.curve().clone()))
        .unwrap();
    let json = builder.finish().unwrap().to_json().unwrap();
    let restored = crate::RawModel::from_json(&json, &crate::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    let restored = restored.pcurves().next().unwrap();
    assert!(
        restored
            .curve()
            .start()
            .coincides_with(&materialized.curve().start())
            .unwrap()
    );
    assert!(
        restored
            .curve()
            .end()
            .coincides_with(&materialized.curve().end())
            .unwrap()
    );
}

#[test]
fn selected_charts_keep_affine_maps_reversal_restriction_and_archives() {
    let original = retained();
    let mut mappings = vec![original.clone()];
    let mut reflected = original.clone();
    reflected.source_scale = (-2).into();
    reflected.source_offset = 5.into();
    reflected.domain = original.domain.affine_image(&-q(1, 2), &q(5, 2)).unwrap();
    mappings.push(reflected);
    for axis in [TensorAxis::U, TensorAxis::V] {
        for scale in [-2, 3] {
            for offset in [-5, 5] {
                mappings.push(SurfaceIntersectionPcurve::tensor_iso(
                    original.domain.clone(),
                    3.into(),
                    axis,
                    scale.into(),
                    offset.into(),
                ));
            }
        }
    }
    for mapping in mappings {
        for mapping in [
            mapping.clone(),
            mapping.reversed().unwrap(),
            mapping.reversed().unwrap().reversed().unwrap(),
        ] {
            verify_selected_mapping(&mapping);
            // Retain the selected end while restricting the represented end.
            let (start, end) = (mapping.domain.start().clone(), mapping.domain.end().clone());
            assert!(matches!(
                mapping.subcurve(&end, &start),
                Err(GeometryError::InvalidParameterDomain)
            ));
            assert!(matches!(
                mapping.subcurve(&start, &start),
                Err(GeometryError::InvalidParameterDomain)
            ));
            assert!(matches!(
                mapping.subcurve(&Real::from(-100).into(), &end),
                Err(GeometryError::ParameterOutsideDomain)
            ));
            verify_selected_mapping(&mapping.subcurve(&start, &end).unwrap());
        }
    }
    let middle = original
        .subcurve(&q(1, 2).into(), original.domain.end())
        .unwrap();
    verify_selected_mapping(&middle);
    verify_selected_mapping(&middle.reversed().unwrap());
    // Both cut parameters may be represented even when the source endpoint is selected.
    let inner = original.subcurve(&q(1, 3).into(), &q(2, 3).into()).unwrap();
    verify_selected_mapping(&inner);
}

#[test]
fn angular_maps_keep_seams_and_reject_exterior_selected_parameters() {
    let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let circle = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        2.into(),
        0.into(),
        Real::tau(),
    )
    .unwrap();
    for curve in [circle.clone(), circle.reversed().unwrap()] {
        let map = plane.planar_pcurve(&curve).unwrap().unwrap();
        for (local, expected) in [(Real::zero(), Real::zero()), (Real::one(), Real::tau())] {
            assert_eq!(
                map.spatial_parameter_at(&local.into())
                    .unwrap()
                    .compare(&expected.into())
                    .unwrap(),
                Ordering::Equal
            );
        }
        let exterior = selected().affine_image(&Real::one(), &Real::one()).unwrap();
        assert!(matches!(
            map.spatial_parameter_at(&exterior),
            Err(GeometryError::ParameterOutsideDomain)
        ));
        // Selected interior angular coordinates have no represented view.
        assert!(matches!(
            map.spatial_parameter_at(&selected()),
            Err(GeometryError::UnsupportedPcurveContour)
        ));
        assert!(map.spatial_parameter_at(&q(1, 2).into()).is_ok());
    }
}

#[test]
fn materialized_parameter_map_checks_domain() {
    let mapping = SurfaceIntersectionPcurve::tensor_iso_v(ParameterRange::unit(), 3.into())
        .materialize()
        .unwrap();
    for outside in [-1, 2] {
        assert!(matches!(
            mapping.spatial_parameter_at(&Real::from(outside).into()),
            Err(GeometryError::ParameterOutsideDomain)
        ));
    }
}
