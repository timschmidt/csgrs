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

#[test]
fn svg_adapter_consumes_generated_selected_circle_fillet_and_offsets() {
    use hypercurve::{
        BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
        BezierParameterPolynomial, Curve2, CurveCornerMode2, CurveFillet2,
        CurveFilletContact2, CurvePointDefinition2, RationalMap2,
    };
    use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
    let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
    let center = CurvePointDefinition2::PolynomialFiber {
        equation: BivariatePolynomial::new(vec![
            vec![
                Real::from(-2),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ],
            vec![Real::one()],
        ]),
        parameter: BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    Real::from(-1),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::from(2),
                ])
                .unwrap(),
                BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
            )
            .unwrap(),
        ),
        root: IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let at = |x: Real, y: Real| {
        center
            .rational_image(&RationalMap2::new(
                [
                    BivariatePolynomial::new(vec![vec![x], vec![Real::one()]]),
                    BivariatePolynomial::new(vec![vec![y, Real::one()]]),
                ],
                BivariatePolynomial::new(vec![vec![Real::one()]]),
            ))
            .unwrap()
    };
    // Original half disk: N=(-4/5,3/5), T=(3/5,4/5). Radius 1/4
    // fillet at C-N has M=C-sN+T/4, s=sqrt(1/2), and these contacts.
    let s = q(1, 2).sqrt().unwrap();
    let previous = at(&s * q(16, 15) + q(1, 5), &s * q(-4, 5) + q(4, 15));
    let next = at(&s * q(4, 5), &s * q(-3, 5));
    let fixture = include_str!("fixtures/selected_circle_source.svg");
    let (empty, wires, paths) = import_svg(fixture).unwrap();
    assert!(empty.is_empty() && wires.is_empty());
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 1);
    let arc = paths[0].curves()[0].clone();
    let diameter = Curve2::try_line(arc.end(), arc.start()).unwrap();
    let source = CurvePath2::try_new(vec![arc, diameter]).unwrap();
    let request = CurveFillet2 {
        radius: q(1, 4),
        center: Some(at(&s * q(4, 5) + q(3, 20), &s * q(-3, 5) + q(1, 5))),
        contacts: [
            Some(CurveFilletContact2::Point(previous.clone())),
            Some(CurveFilletContact2::Point(next.clone())),
        ],
    };
    let solutions = source
        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly)
        .unwrap();
    assert_eq!(solutions.candidate_count(), 1);
    let region =
        CurveRegion2::try_from_boundary_paths(&solutions.into_solutions(), FillRule::NonZero)
            .unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (1, 0));
    assert_eq!(region.boundary_paths().unwrap()[0].curves().len(), 3);
    for (point, expected) in [
        (previous, RegionPointLocation::Boundary),
        (next, RegionPointLocation::Boundary),
        (at(q(4, 5), q(-3, 5)), RegionPointLocation::Outside),
        (at(q(3, 20), q(1, 5)), RegionPointLocation::Inside),
    ] {
        assert_eq!(region.classify_point(&point).unwrap(), expected);
    }
    let text = export_svg(&region, &[], &[]).unwrap();
    let (replay, wires, paths) = import_svg(&text).unwrap();
    assert!(wires.is_empty() && paths.is_empty());
    assert_eq!(export_svg(&replay, &wires, &paths).unwrap(), text);
    assert!(
        replay
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let grown = replay.offset(q(1, 16), &OffsetCornerStyle2::Round).unwrap();
    let restored = grown.offset(q(-1, 16), &OffsetCornerStyle2::Round).unwrap();
    assert!(
        restored
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn svg_adapter_replays_empty_results_and_preserves_qualification() {
    let center = Point2::new(Real::zero(), Real::zero());
    let start = Point2::new(Real::one(), Real::zero());
    let boundary = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(start.clone(), start, center, false)
            .unwrap()
            .into(),
    ])
    .unwrap();
    let source =
        CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).unwrap();
    let erased = source.boolean_region(&source, BooleanOp::Difference).unwrap();
    for empty in [CurveRegion2::empty(), erased] {
        assert!(empty.is_empty());
        let text = export_svg(&empty, &[], &[]).unwrap();
        assert!(text.contains("data-hypercurve-path=\"5:484350350000000000000000\""));
        let (replay, wires, paths) = import_svg(&text).unwrap();
        assert!(replay.is_empty() && wires.is_empty() && paths.is_empty());
        assert_eq!(export_svg(&replay, &wires, &paths).unwrap(), text);
        let restored = replay.boolean_region(&source, BooleanOp::Union).unwrap();
        assert!(
            restored
                .boolean_region(&source, BooleanOp::Xor)
                .unwrap()
                .is_empty()
        );
        assert!(
            replay
                .offset(Real::one(), &OffsetCornerStyle2::Round)
                .unwrap()
                .is_empty()
        );
    }
    let qualified = hypercurve::provisional(|| {
        let zero = (Real::e() + Real::one()).sin()
            - Real::e().sin() * Real::one().cos()
            - Real::e().cos() * Real::one().sin();
        let start = Point2::new(Real::e().sin(), Real::zero());
        let end = Point2::new(Real::e().sin() + zero, Real::zero());
        let path = CurvePath2::try_new(vec![
            hypercurve::QuadraticBezier2::new(
                start,
                Point2::new(Real::zero(), Real::one()),
                end,
            )
            .into(),
        ])
        .unwrap();
        CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap()
    });
    assert_eq!(
        qualified.certainty(),
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    let qualified = qualified.into_unverified();
    assert!(qualified.is_empty());
    assert!(matches!(
        export_svg(&qualified, &[], &[]),
        Err(csgrs::io::IoError::Construction {
            format: "SVG",
            source: csgrs::errors::ValidationError::ExactCurve(hypercurve::ExactCurveError::Blocked(reason)),
        }) if reason.reason() == hypercurve::UncertaintyReason::Predicate
    ));
}
