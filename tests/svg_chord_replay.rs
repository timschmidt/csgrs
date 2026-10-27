#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, CurvePath2, CurvePointLocations2,
    CurveRegion2, Real,
};

#[test]
fn svg_adapter_replays_a_selected_chord_before_further_split() {
    let mut coefficients = vec![Real::zero(); 8];
    coefficients[0] = (-1).into();
    coefficients[7] = 2.into();
    let beta: CurveParameter2 = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(
                (Real::from(3) / Real::from(4)).unwrap(),
                Real::one(),
            )
            .unwrap(),
        )
        .unwrap(),
    )
    .into();
    let at = |y: i32| {
        beta.affine_point(1.into(), 0.into(), 0.into(), y.into())
            .unwrap()
    };
    let path = CurvePath2::try_new(vec![Curve2::try_line(at(0), at(2)).unwrap()]).unwrap();
    let encoded = export_svg(&CurveRegion2::empty(), &[], &[path]).unwrap();
    let (region, wires, paths) = import_svg(&encoded).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    let curve = &paths[0].curves()[0];
    assert!(curve.geometry().is_none());
    assert!(curve.start().coordinates().is_none());
    assert!(curve.start().coincides_with(&at(0)).unwrap());
    assert!(curve.end().coincides_with(&at(2)).unwrap());
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&at(1)).unwrap()
    else {
        panic!("one exact selected midpoint")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = curve
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    assert!(first.end().coincides_with(&at(1)).unwrap());
    assert!(second.start().coincides_with(&first.end()).unwrap());
    let encoded_split = export_svg(
        &region,
        &wires,
        &[CurvePath2::try_new(vec![first, second]).unwrap()],
    )
    .unwrap();
    let (_, _, split) = import_svg(&encoded_split).unwrap();
    assert_eq!(split[0].curves().len(), 2);
    assert!(split[0].start().coincides_with(&at(0)).unwrap());
    assert!(split[0].end().coincides_with(&at(2)).unwrap());
    assert!(split[0].curves()[0].end().coincides_with(&at(1)).unwrap());
}
