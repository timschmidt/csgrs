mod common;
use common::*;
use hyperreal::StructuralKind;
use hyperspatial::*;
use std::sync::Arc;

#[test]
fn every_public_scalar_family_survives_the_local_height_workflow() {
    let pi = Real::pi();
    let values = [
        (StructuralKind::ExactRational, r("1.5")),
        (StructuralKind::PiLike, pi.clone()),
        (StructuralKind::ExpLike, Real::e()),
        (StructuralKind::SqrtLike, Real::from(2).sqrt().unwrap()),
        (StructuralKind::LogLike, Real::from(2).ln().unwrap()),
        (StructuralKind::TrigExact, r("0.2").sin_pi()),
        (StructuralKind::ProductConstant, &pi * &pi * Real::e()),
        (StructuralKind::ComputableOpaque, Real::one().sin()),
    ];
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let origin = position(0.into(), 0.into(), 0.into(), ellipsoid.clone(), &mut ctx);
    let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut ctx).unwrap();
    for (kind, value) in values {
        assert_eq!(value.detailed_facts().symbolic.kind, kind);
        let source = position(
            0.into(),
            0.into(),
            value.clone(),
            ellipsoid.clone(),
            &mut ctx,
        );
        let ecef = source.to_ecef(&mut ctx).unwrap();
        let local = frame.to_local_position(&ecef, &mut ctx).unwrap();
        equal(&local.coordinates().x, &Real::zero());
        equal(&local.coordinates().y, &Real::zero());
        equal(&local.coordinates().z, &value);
        let restored = frame.to_ecef_position(&local, &mut ctx).unwrap();
        equal(&restored.coordinates().x, &(r("6378137") + value));
    }
}

#[test]
fn shared_sources_remain_consistent_across_threads_and_cache_warming() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let source = Arc::new(position(
        0.into(),
        0.into(),
        Real::pi(),
        ellipsoid,
        &mut ctx,
    ));
    let handles: Vec<_> = (0..4)
        .map(|_| {
            let source = source.clone();
            std::thread::spawn(move || {
                let mut ctx = context();
                for _ in 0..8 {
                    let point = source.to_ecef(&mut ctx).unwrap();
                    let _ = point.coordinates().to_f64_array_lossy().unwrap();
                    equal(&point.coordinates().x, &(r("6378137") + Real::pi()));
                    assert_eq!(point.receipt().certainty(), Certainty::Exact);
                }
            })
        })
        .collect();
    for handle in handles {
        handle.join().unwrap();
    }
    equal(source.height().as_metres(), &Real::pi());
}

#[test]
fn explicit_input_qualification_is_not_promoted_by_exact_arithmetic() {
    let mut ctx = context();
    let ellipsoid = Arc::new(Ellipsoid::wgs84(&mut ctx).unwrap());
    let qualification = InputAccuracy::Qualified(Arc::from(
        "synthetic example with unspecified measurement covariance",
    ));
    let source = GeodeticPosition::new(
        Latitude::new(Angle::degrees(0.into()), &mut ctx).unwrap(),
        Longitude::unrolled(Angle::degrees(0.into())),
        Length::metres(0.into()),
        ellipsoid,
        reference(),
        InputProvenance::new("qualified input", qualification.clone()),
        &mut ctx,
    )
    .unwrap();
    let point = source.to_ecef(&mut ctx).unwrap();
    assert_eq!(point.receipt().certainty(), Certainty::Exact);
    let EcefSource::Geodetic(retained) = point.source() else {
        panic!("lost construction source");
    };
    assert_eq!(
        retained.receipt().input().unwrap().accuracy(),
        &qualification
    );
}
