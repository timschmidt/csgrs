#![no_main]

use std::{cmp::Ordering, sync::Arc};

use hyperlattice::Vector3;
use hyperlimit::{PredicateOutcome, compare_reals};
use hyperspatial::*;
use libfuzzer_sys::fuzz_target;

fn equal(left: &Real, right: &Real) {
    assert!(matches!(
        compare_reals(left, right, PredicatePolicy::STRICT),
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        }
    ));
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
    let a = Real::from(i32::from(data[0] % 64) + 1);
    let b = Real::from(i32::from(data[1] % 64) + 1);
    let ellipsoid = Arc::new(
        Ellipsoid::from_axes(
            Length::metres(a.clone()),
            Length::metres(b.clone()),
            &mut context,
        )
        .unwrap(),
    );
    let latitude = [-90, -60, -45, -30, 0, 30, 45, 60, 90][usize::from(data[2]) % 9];
    let longitude = (i32::from(data[3] % 17) - 8) * 45;
    let source = GeodeticPosition::new(
        Latitude::new(Angle::degrees(latitude.into()), &mut context).unwrap(),
        Longitude::unrolled(Angle::degrees(longitude.into())),
        Length::metres(Real::zero()),
        ellipsoid,
        ReferenceFrame::fixed("fuzz:static").unwrap(),
        InputProvenance::new("bounded fuzz input", InputAccuracy::ExactModel),
        &mut context,
    )
    .unwrap();
    let ecef = source.to_ecef(&mut context).unwrap();
    let p = ecef.coordinates();
    let surface =
        ((&p.x * &p.x + &p.y * &p.y) / (&a * &a)).unwrap() + ((&p.z * &p.z) / (&b * &b)).unwrap();
    equal(&surface, &Real::one());
    let axes = if data[4] & 1 == 0 {
        LocalAxes::Enu
    } else {
        LocalAxes::Ned
    };
    let frame = LocalFrame::new(&source, axes, &mut context).unwrap();
    for i in 0..3 {
        for j in 0..3 {
            let dot = Vector3::new(frame.rotation()[i].clone())
                .dot(&Vector3::new(frame.rotation()[j].clone()));
            equal(&dot, &Real::from(i32::from(i == j)));
        }
    }
    let height = Real::from(i32::from(data[4]) - 128);
    let elevated = GeodeticPosition::new(
        source.latitude().clone(),
        source.longitude().clone(),
        Length::metres(height.clone()),
        source.ellipsoid().clone(),
        source.reference().clone(),
        InputProvenance::new("normal displacement", InputAccuracy::ExactModel),
        &mut context,
    )
    .unwrap()
    .to_ecef(&mut context)
    .unwrap();
    let expected_vertical = if axes == LocalAxes::Enu {
        height
    } else {
        -height
    };
    let mut current = elevated.clone();
    for _ in 0..=data[5] % 4 {
        let local = frame.to_local_position(&current, &mut context).unwrap();
        equal(&local.coordinates().x, &Real::zero());
        equal(&local.coordinates().y, &Real::zero());
        equal(&local.coordinates().z, &expected_vertical);
        current = frame.to_ecef_position(&local, &mut context).unwrap();
        assert_ne!(current.receipt().certainty(), Certainty::Approximate);
    }
    equal(&current.coordinates().x, &elevated.coordinates().x);
    equal(&current.coordinates().y, &elevated.coordinates().y);
    equal(&current.coordinates().z, &elevated.coordinates().z);
});
