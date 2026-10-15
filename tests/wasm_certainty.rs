#![cfg(feature = "wasm")]

use csgrs::wasm::{
    context_js::{GeometryBoolResultJs, GeometryCertaintyJs, GeometryQueryStatusJs},
    curve_js::{CurveRegionJs, GeometryMeshResultJs},
    mesh_js::MeshJs,
    point_js::Point3Js,
};
use csgrs::{GeometryCertainty, GeometryOutcome, Real, curve, solid};
use hyperlattice::Point3;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn named_receipts_preserve_both_certainty_values() {
    for certainty in [
        GeometryCertainty::Certified,
        GeometryCertainty::Approximate512Consumed,
    ] {
        let expected = GeometryCertaintyJs::from(certainty);
        let query = GeometryBoolResultJs::from(GeometryOutcome {
            value: Some(false),
            certainty,
        });
        assert_eq!(query.status(), GeometryQueryStatusJs::Decided);
        assert_eq!(query.certainty(), Some(expected));
        assert_eq!(query.value(), Some(false));
        assert_eq!(query.reason(), None);
        let result = GeometryMeshResultJs::from(GeometryOutcome {
            value: solid::cube(Real::one()).expect("valid primitive fixture"),
            certainty,
        });
        assert_eq!(result.certainty(), expected);
        assert_eq!(
            result.approximate_512_consumed(),
            certainty == GeometryCertainty::Approximate512Consumed
        );
        assert_eq!(result.into_mesh().triangle_count(), 12);
    }
}

#[test]
fn curve_boundary_is_distinct_from_empty_and_outside() {
    let square = CurveRegionJs::from(curve::square(Real::one()));
    let empty = CurveRegionJs::new();
    for approximate in [false, true] {
        let boundary = square
            .contains_xy_with_context(0.0, 0.5, approximate)
            .unwrap();
        assert_eq!(boundary.status(), GeometryQueryStatusJs::Boundary);
        assert_eq!(boundary.value(), None);
        assert_eq!(boundary.certainty(), Some(GeometryCertaintyJs::Certified));
        for outside in [
            square
                .contains_xy_with_context(-1.0, 0.5, approximate)
                .unwrap(),
            empty.contains_xy_with_context(0.0, 0.0, approximate).unwrap(),
        ] {
            assert_eq!(outside.status(), GeometryQueryStatusJs::Decided);
            assert_eq!(outside.value(), Some(false));
            assert_eq!(outside.certainty(), Some(GeometryCertaintyJs::Certified));
            assert_eq!(outside.reason(), None);
        }
    }
    assert_eq!(empty.contains_xy(0.0, 0.0).unwrap(), Some(false));
    assert_eq!(
        curve::contains_xy(&curve::empty(), Real::zero(), Real::zero()),
        Some(false)
    );
}

#[test]
fn curve_uncertainty_survives_a_warm_approximate_boundary_query() {
    let square =
        curve::try_translated(&curve::square(Real::one()), unresolved_zero(), Real::zero())
            .unwrap();
    let square = CurveRegionJs::from(square);
    for approximate in [false, true, false] {
        let result = square
            .contains_xy_with_context(0.0, 0.5, approximate)
            .unwrap();
        assert_eq!(result.value(), None);
        if approximate {
            assert_eq!(result.status(), GeometryQueryStatusJs::Boundary);
            assert_eq!(
                result.certainty(),
                Some(GeometryCertaintyJs::Approximate512Consumed)
            );
            assert!(result.approximate_512_consumed());
            assert_eq!(result.reason(), None);
        } else {
            assert_eq!(result.status(), GeometryQueryStatusJs::Uncertain);
            assert_eq!(result.certainty(), None);
            assert!(result.reason().is_some());
        }
    }
}

#[test]
fn unknown_mesh_query_does_not_claim_a_certified_value() {
    let cube = MeshJs::from(solid::cube(Real::one()).expect("valid primitive fixture"));
    let half = (Real::one() / Real::from(2)).unwrap();
    let point = Point3Js::from(Point3::new(unresolved_zero(), half.clone(), half));
    for approximate in [false, true, false] {
        let result = cube.contains_point_with_context(&point, approximate).unwrap();
        if approximate {
            assert_eq!(result.status(), GeometryQueryStatusJs::Decided);
            assert_eq!(result.value(), Some(false));
            assert_eq!(
                result.certainty(),
                Some(GeometryCertaintyJs::Approximate512Consumed)
            );
        } else {
            assert_eq!(result.status(), GeometryQueryStatusJs::Unknown);
            assert_eq!(result.value(), None);
            assert_eq!(result.certainty(), None);
            assert!(result.reason().is_some());
        }
    }
}
