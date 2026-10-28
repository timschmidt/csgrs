#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{
    BooleanOp, CircularArc2, CurvePath2, CurveRegion2, FillRule, OffsetCornerStyle2, Point2,
    Real, RegionPointLocation,
};

#[test]
fn svg_adapter_retains_exact_filled_annulus_before_further_offsets() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let x = Real::from(2).sqrt().unwrap();
    let center = Point2::new(x.clone(), Real::from(3));
    let circle = |radius, clockwise| {
        let p = Point2::new(&x + Real::from(radius), Real::from(3));
        CurvePath2::try_new(vec![
            CircularArc2::try_from_center(p.clone(), p, center.clone(), clockwise)
                .unwrap()
                .into(),
        ])
        .unwrap()
    };
    let source = CurveRegion2::try_from_boundary_paths(
        &[circle(2, false), circle(1, true)],
        FillRule::NonZero,
    )
    .unwrap();
    let text = export_svg(&source, &[], &[]).unwrap();
    assert!(text.contains("data-hypercurve-path=\"5:"));
    let (region, wires, paths) = import_svg(&text).unwrap();
    assert!(wires.is_empty() && paths.is_empty());
    assert_eq!(region.loop_role_counts().unwrap(), (1, 1));
    for (p, expected) in [
        (center, RegionPointLocation::Outside),
        (
            Point2::new(&x + q(3, 2), Real::from(3)),
            RegionPointLocation::Inside,
        ),
        (
            Point2::new(&x + q(3, 5), Real::from(3) + q(4, 5)),
            RegionPointLocation::Boundary,
        ),
    ] {
        assert_eq!(region.classify_point(&p.into()).unwrap(), expected);
    }
    assert!(
        region
            .boolean_region(&source, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert_eq!(export_svg(&region, &wires, &paths).unwrap(), text);
    let grown = region.offset(q(1, 4), &OffsetCornerStyle2::Round).unwrap();
    let restored = grown.offset(q(-1, 4), &OffsetCornerStyle2::Round).unwrap();
    assert!(
        restored
            .boolean_region(&source, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let invalid = "<svg xmlns=\"http://www.w3.org/2000/svg\"><path data-hypercurve-path=\"5:4843503500000001ff00000000\" d=\"M0 0 L1 0 L1 1 Z\"/></svg>";
    assert!(matches!(
        import_svg(invalid),
        Err(csgrs::io::IoError::Unsupported { format: "SVG", .. })
    ));
}
