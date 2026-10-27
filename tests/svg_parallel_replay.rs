#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{
    BezierParameter2, BezierParameterRange2, Curve2, CurvePath2, CurvePointLocations2,
    CurveRegion2, Point2, QuadraticBezier2, Real,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn svg_adapter_replays_a_procedural_parallel_before_further_incidence() {
    let source = QuadraticBezier2::new(
        Point2::new(0.into(), 0.into()),
        Point2::new(q(1, 2), 0.into()),
        Point2::new(1.into(), 1.into()),
    );
    let curve = Curve2::try_analytic_parallel(
        source.parallel_left(q(-1, 4)).unwrap(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        )
        .unwrap(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![curve]).unwrap();
    let encoded = export_svg(&CurveRegion2::empty(), &[], &[path]).unwrap();
    let (region, wires, paths) = import_svg(&encoded).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    let curve = &paths[0].curves()[0];
    assert!(curve.geometry().is_none());
    assert_eq!(
        curve.analytic_parallel().unwrap().parallel().distance(),
        &q(-1, 4)
    );
    let shift = (q(1, 4) / Real::from(2).sqrt().unwrap()).unwrap();
    let expected = Point2::new(q(1, 2) + &shift, q(1, 4) - shift).into();
    assert!(
        curve
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&expected).unwrap()
    else {
        panic!("the regular parabola parallel has one matching parameter")
    };
    assert_eq!(locations.len(), 1);
    assert_eq!(
        locations[0]
            .parameter()
            .unwrap()
            .compare(&q(1, 2).into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
}
