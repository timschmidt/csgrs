#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCornerMode2, CurveFamily2, CurveFillet2,
    CurveParameter2, CurvePath2, CurvePointLocations2, CurveRegion2, Real,
};

#[test]
fn svg_adapter_replays_a_selected_circle_before_split_reversal_and_another_archive() {
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
    let text = export_svg(&CurveRegion2::empty(), &[], &[filleted]).unwrap();
    let (region, wires, paths) = import_svg(&text).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 3);
    let curve = &paths[0].curves()[1];
    assert_eq!(curve.family(), CurveFamily2::CircularArc);
    assert!(curve.start().coincides_with(&at(q(3, 4), 0.into())).unwrap());
    assert!(curve.end().coincides_with(&at(1.into(), q(1, 4))).unwrap());
    let interior = at(q(9, 10), q(1, 20));
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&interior).unwrap()
    else {
        panic!("one exact circle-interior location")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = curve.split_at(locations[0].parameter().unwrap()).unwrap();
    assert!(first.end().coincides_with(&interior).unwrap());
    assert!(second.start().coincides_with(&interior).unwrap());
    let again = export_svg(
        &region,
        &wires,
        &[
            CurvePath2::try_new(vec![second.reversed().unwrap(), first.reversed().unwrap()])
                .unwrap(),
        ],
    )
    .unwrap();
    let (_, _, paths) = import_svg(&again).unwrap();
    assert!(
        paths[0]
            .start()
            .coincides_with(&at(1.into(), q(1, 4)))
            .unwrap()
    );
    assert!(paths[0].end().coincides_with(&at(q(3, 4), 0.into())).unwrap());
    assert_eq!(export_svg(&region, &wires, &paths).unwrap(), again);
}
