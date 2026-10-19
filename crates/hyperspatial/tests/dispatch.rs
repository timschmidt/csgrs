mod common;
use common::*;
use hyperlattice::Point3;
use hyperspatial::*;
use std::sync::Arc;

// Reuse the scalar owner's finite inventory; this package is still unpublished.
#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

fn assert_tag(value: &Real, expected: &str) {
    let serialized: serde_json::Value = serde_json::from_str(&value.to_json()).unwrap();
    let actual = match &serialized["class"] {
        serde_json::Value::String(name) => name.as_str(),
        serde_json::Value::Object(fields) if fields.len() == 1 => fields.keys().next().unwrap(),
        other => panic!("unexpected certificate {other}"),
    };
    assert_eq!(actual, expected);
}

fn close(actual: &Real, expected: &Real) {
    let bound = r("0.000000000000000000000000000001");
    assert_eq!(
        hyperlimit::compare_reals(&(actual - expected).abs(), &bound, PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Less)
    );
}

#[test]
fn all_22_certificate_recipes_cross_fresh_and_warmed_height_workflows() {
    let cases = scalar_cases::optimized_certificate_representatives();
    assert_eq!(cases.len(), 22);
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let anchor = position(0.into(), 0.into(), 0.into(), ellipsoid.clone(), &mut ctx);
    let frame = LocalFrame::new(&anchor, LocalAxes::Ned, &mut ctx).unwrap();
    for case in cases {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        assert_tag(&case.value, case.certificate);
        let fresh = Real::from_json(&case.value.to_json()).unwrap();
        assert!(format!("{fresh:?}").contains("primitive_approx_cache: Empty"));
        for value in [case.value.clone(), fresh] {
            for warmed in [false, true] {
                if warmed {
                    let _ = value.to_f32_lossy();
                    match std::env::var("HYPERREAL_EXPECT_F32_CACHE").as_deref() {
                        Ok("present") => assert!(
                            format!("{value:?}").contains("primitive_approx_cache: F32(Some(")
                        ),
                        Ok("absent") => {
                            assert!(!format!("{value:?}").contains("primitive_approx_cache: F32("))
                        }
                        Ok(other) => panic!("unexpected cache expectation {other}"),
                        Err(_) => {}
                    }
                    let _ = value.to_f64_lossy();
                    assert!(format!("{value:?}").contains("primitive_approx_cache: F64(Some("));
                }
                assert_tag(&value, case.certificate);
                let source = position(
                    0.into(),
                    0.into(),
                    value.clone(),
                    ellipsoid.clone(),
                    &mut ctx,
                );
                let ecef = source.to_ecef(&mut ctx).unwrap();
                let inverse = ecef
                    .to_geodetic_closest(ellipsoid.clone(), PoleMeridian::Reject, &mut ctx)
                    .unwrap();
                equal(inverse.height().as_metres(), &value);
                let local = frame.to_local_position(&ecef, &mut ctx).unwrap();
                equal(&local.coordinates().z, &(-&value));
                assert_eq!(
                    frame
                        .to_ecef_position(&local, &mut ctx)
                        .unwrap()
                        .coordinates(),
                    ecef.coordinates()
                );
                assert_ne!(local.receipt().certainty(), Certainty::Approximate);
            }
        }
    }
    let exact = scalar_cases::fraction(1, 3);
    let _ = exact.to_f64_lossy();
    let clone = exact.clone();
    match std::env::var("HYPERREAL_EXPECT_EXACT_CLONE_CACHE").as_deref() {
        Ok("present") => {
            assert!(format!("{clone:?}").contains("primitive_approx_cache: F64(Some("));
        }
        Ok("absent") => {
            assert!(format!("{clone:?}").contains("primitive_approx_cache: Empty"));
        }
        Ok(other) => panic!("unexpected cache expectation {other}"),
        Err(_) => {}
    }
}

#[test]
fn all_484_ordered_certificate_pairs_define_native_normal_offset_inverses() {
    let cases = scalar_cases::optimized_certificate_representatives();
    let sqrt2 = Real::from(2).sqrt().unwrap();
    let latitude = Angle::degrees(45.into());
    for first in &cases {
        for second in &cases {
            let mut ctx = context();
            let a = Real::from(100) + &first.value;
            let b = Real::from(100) + &second.value;
            let h = &first.value + &second.value;
            let ellipsoid = Arc::new(
                Ellipsoid::from_axes(
                    Length::metres(a.clone()),
                    Length::metres(b.clone()),
                    &mut ctx,
                )
                .unwrap(),
            );
            // At normal latitude 45 degrees the surface foot is
            // (a²,b²)/hypot(a,b), and a positive normal offset adds h/sqrt(2).
            let d = Real::hypot2(&a, &b).unwrap();
            let delta = (&h / &sqrt2).unwrap();
            let point = EcefPosition::new(
                Point3::new(
                    ((&a * &a) / &d).unwrap() + &delta,
                    Real::zero(),
                    ((&b * &b) / d).unwrap() + delta,
                ),
                reference(),
                input(),
                &mut ctx,
            )
            .unwrap();
            let inverse = point
                .to_geodetic_closest(ellipsoid, PoleMeridian::Reject, &mut ctx)
                .unwrap_or_else(|error| {
                    panic!("{} / {}: {error}", first.certificate, second.certificate)
                });
            close(
                inverse.latitude().angle().as_radians(),
                latitude.as_radians(),
            );
            close(inverse.height().as_metres(), &h);
            let frame = LocalFrame::new(&inverse, LocalAxes::Enu, &mut ctx).unwrap();
            let local = frame.to_local_position(&point, &mut ctx).unwrap();
            equal(&local.coordinates().x, &Real::zero());
            equal(&local.coordinates().y, &Real::zero());
            equal(&local.coordinates().z, &Real::zero());
        }
    }
}
