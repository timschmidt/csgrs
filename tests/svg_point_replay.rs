#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCornerMode2, CurveFillet2, CurveParameter2,
    CurvePath2, CurvePointDefinition2, CurvePointLocations2, CurveRegion2, Real,
};

#[test]
fn svg_adapter_replays_a_fillet_cut_before_another_exact_split() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let mut coefficients = vec![Real::zero(); 8];
    coefficients[0] = (-1).into();
    coefficients[7] = 2.into();
    let beta: CurveParameter2 = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
        )
        .unwrap(),
    )
    .into();
    let at = |x, y| beta.affine_point(1.into(), x, 0.into(), y).unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::try_line(at(0.into(), 0.into()), at(1.into(), 0.into())).unwrap(),
        Curve2::try_line(at(1.into(), 0.into()), at(1.into(), 1.into())).unwrap(),
    ])
    .unwrap();
    let mut solutions = path
        .fillet_vertex(1, &CurveFillet2::new(q(1, 4)), CurveCornerMode2::TrimOnly)
        .unwrap()
        .into_solutions();
    assert_eq!(solutions.len(), 1);
    let filleted = solutions.pop().unwrap();
    let line = CurvePath2::try_new(vec![filleted.curves()[0].clone()]).unwrap();
    let text = export_svg(&CurveRegion2::empty(), &[], &[line]).unwrap();
    let (region, wires, paths) = import_svg(&text).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    let curve = &paths[0].curves()[0];
    assert!(matches!(
        curve.end().definition().unwrap(),
        CurvePointDefinition2::SupportingLineIntersection { .. }
    ));
    assert!(curve.end().coincides_with(&at(q(3, 4), 0.into())).unwrap());
    let midpoint = at(q(3, 8), 0.into());
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&midpoint).unwrap()
    else {
        panic!("one exact midpoint")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = curve
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    assert!(first.end().coincides_with(&midpoint).unwrap());
    assert!(second.start().coincides_with(&midpoint).unwrap());
    let again = export_svg(
        &region,
        &wires,
        &[CurvePath2::try_new(vec![first, second]).unwrap()],
    )
    .unwrap();
    let (_, _, paths) = import_svg(&again).unwrap();
    assert!(
        paths[0]
            .start()
            .coincides_with(&at(0.into(), 0.into()))
            .unwrap()
    );
    assert!(paths[0].end().coincides_with(&at(q(3, 4), 0.into())).unwrap());
}
