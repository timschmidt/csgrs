#![cfg(feature = "svg")]

use hypercurve::{
    BezierParallel2, BezierParameter2, BezierParameterRange2, CubicBezier2, Curve2,
    CurveParameterRange2, CurvePath2, CurveRegion2, FillRule, LineSeg2, OffsetCornerStyle2, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, Real, RegionPointLocation, SvgGeometry2,
    export_svg_document, import_svg_document,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn exact(value: Real) -> BezierParameter2 {
    BezierParameter2::Exact(value)
}

fn range(start: BezierParameter2, end: BezierParameter2) -> BezierParameterRange2 {
    BezierParameterRange2::try_new(start, end).unwrap()
}

fn unit_curve(parallel: BezierParallel2) -> Curve2 {
    Curve2::try_analytic_parallel(parallel, range(exact(Real::zero()), exact(Real::one()))).unwrap()
}

fn document(path: CurvePath2) -> String {
    export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![path],
    ))
    .unwrap()
}

fn parabola() -> QuadraticBezier2 {
    QuadraticBezier2::new(point(0, 0), Point2::new(q(1, 2), 0.into()), point(1, 1))
}

#[test]
fn parallel_sources_replay_without_changing_their_family_or_normal_side() {
    let sqrt2 = Real::from(2).sqrt().unwrap();
    let sqrt5 = Real::from(5).sqrt().unwrap();
    let ellipse = RationalQuadraticBezier2::try_new(
        point(2, 0),
        point(2, 1),
        point(0, 1),
        Real::one(),
        (&sqrt2 / Real::from(2)).unwrap(),
        Real::one(),
    )
    .unwrap();
    // C(t)=(t-t^3/3,t^2), with exact speed 1+t^2.
    let ph = CubicBezier2::new(
        point(0, 0),
        Point2::new(q(1, 3), 0.into()),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), 1.into()),
    );
    let cases = [
        (
            parabola().parallel_left(q(-1, 4)).unwrap(),
            Point2::new(
                q(1, 2) + (q(1, 4) / &sqrt2).unwrap(),
                q(1, 4) - (q(1, 4) / &sqrt2).unwrap(),
            ),
        ),
        (
            ellipse.parallel_left(q(-1, 8)).unwrap(),
            Point2::new(
                &sqrt2 + (q(1, 8) / &sqrt5).unwrap(),
                (&sqrt2 / Real::from(2)).unwrap() + (q(1, 4) / &sqrt5).unwrap(),
            ),
        ),
        (
            ph.parallel_left(q(1, 4)).unwrap(),
            Point2::new(q(31, 120), q(2, 5)),
        ),
        (
            QuadraticBezier2::new(point(0, 0), point(0, 0), point(1, 1))
                .parallel_left(Real::zero())
                .unwrap(),
            Point2::new(q(1, 4), q(1, 4)),
        ),
    ];
    for (parallel, expected) in cases {
        let original = unit_curve(parallel.clone());
        let encoded = document(
            CurvePath2::try_new(vec![original.clone(), original.reversed().unwrap()]).unwrap(),
        );
        let replay = import_svg_document(&encoded).unwrap();
        let curves = replay.paths()[0].curves();
        let forward = curves[0].analytic_parallel().unwrap();
        let reverse = curves[1].analytic_parallel().unwrap();
        assert_eq!(forward.parallel().source(), parallel.source());
        assert_eq!(forward.parallel().distance(), parallel.distance());
        assert!(!forward.is_reversed() && reverse.is_reversed());
        assert!(std::ptr::eq(
            forward.parallel().distance(),
            reverse.parallel().distance()
        ));
        assert!(
            curves[0]
                .point_at(&q(1, 2).into())
                .unwrap()
                .coincides_with(&expected.into())
                .unwrap()
        );
        let contacts = curves[0].intersect_curve(&curves[1]).unwrap();
        assert!(contacts.is_complete());
        assert_eq!(contacts.overlaps().len(), 1);
        assert_eq!(document(replay.paths()[0].clone()), encoded);
    }
}

#[test]
fn cusp_split_replay_keeps_one_support_and_selected_boundary() {
    let parallel = parabola().parallel_left(Real::one()).unwrap();
    let analysis = parallel
        .singularity_analysis(&CurveParameterRange2::unit())
        .unwrap();
    let [cusp] = analysis.parallel_cusps() else {
        panic!("one selected cusp")
    };
    let first =
        Curve2::try_analytic_parallel(parallel.clone(), range(exact(Real::zero()), cusp.clone()))
            .unwrap();
    let second =
        Curve2::try_analytic_parallel(parallel.clone(), range(cusp.clone(), exact(Real::one())))
            .unwrap();
    let encoded = document(CurvePath2::try_new(vec![first, second]).unwrap());
    let replay = import_svg_document(&encoded).unwrap();
    let curves = replay.paths()[0].curves();
    let first = curves[0].analytic_parallel().unwrap();
    let second = curves[1].analytic_parallel().unwrap();
    assert!(std::ptr::eq(
        first.parallel().distance(),
        second.parallel().distance()
    ));
    assert_eq!(
        first.range().end().compare(&cusp.clone().into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        second.range().start().compare(first.range().end()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert!(curves[0].end().coincides_with(&curves[1].start()).unwrap());
    let first_parameter = first.range().end().as_bezier_parameter().unwrap();
    let second_parameter = second.range().start().as_bezier_parameter().unwrap();
    let (BezierParameter2::Algebraic(first_root), BezierParameter2::Algebraic(second_root)) =
        (first_parameter, second_parameter)
    else {
        panic!("selected algebraic cusp")
    };
    assert!(std::ptr::eq(
        first_root.polynomial(),
        second_root.polynomial()
    ));
    assert!(
        Curve2::try_analytic_parallel(parallel, range(exact(Real::zero()), exact(Real::one())))
            .is_err()
    );
}

fn cap_path() -> CurvePath2 {
    let parallel = QuadraticBezier2::new(point(0, 0), point(2, 2), point(4, 0))
        .parallel_left(Real::one())
        .unwrap();
    let left = parallel.point_at(&Real::zero()).unwrap();
    let right = parallel.point_at(&Real::one()).unwrap();
    let lower_left = Point2::new(left.x().clone(), (-2).into());
    let lower_right = Point2::new(right.x().clone(), (-2).into());
    CurvePath2::try_new(vec![
        unit_curve(parallel).reversed().unwrap(),
        LineSeg2::try_new(left, lower_left.clone()).unwrap().into(),
        LineSeg2::try_new(lower_left, lower_right.clone())
            .unwrap()
            .into(),
        LineSeg2::try_new(lower_right, right).unwrap().into(),
    ])
    .unwrap()
}

fn verify_cap(document: &str) {
    let replay = import_svg_document(document).unwrap();
    let path = &replay.paths()[0];
    let parallel = path.curves()[0].analytic_parallel().unwrap();
    assert!(parallel.is_reversed());
    assert_eq!(parallel.parallel().distance(), &Real::one());
    assert!(
        path.curves()[0]
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&point(2, 2).into())
            .unwrap()
    );
    let region = CurveRegion2::try_from_boundary_paths(replay.paths(), FillRule::NonZero).unwrap();
    assert_eq!(
        region.classify_point(&point(2, 0).into()).unwrap(),
        RegionPointLocation::Inside
    );
    assert_eq!(
        region.classify_point(&point(2, 3).into()).unwrap(),
        RegionPointLocation::Outside
    );
    assert!(
        region
            .boolean_regions(&region)
            .unwrap()
            .difference()
            .is_empty()
    );
    let expanded = region.offset(q(1, 16), &OffsetCornerStyle2::Round).unwrap();
    assert!(
        expanded
            .boolean_regions(&expanded)
            .unwrap()
            .difference()
            .is_empty()
    );
    assert_eq!(
        expanded.classify_point(&point(2, 0).into()).unwrap(),
        RegionPointLocation::Inside
    );
}

#[test]
fn fresh_process_replays_a_parallel_boundary_before_boolean_and_offset() {
    const INPUT: &str = "HYPERCURVE_PARALLEL_SVG_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        verify_cap(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let encoded = document(cap_path());
    let path = std::env::temp_dir().join(format!(
        "hypercurve-parallel-svg-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, encoded).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replays_a_parallel_boundary_before_boolean_and_offset",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

fn wire_real(bytes: &mut Vec<u8>, value: &Real) {
    let value = value.to_bytes();
    bytes.extend_from_slice(&u32::try_from(value.len()).unwrap().to_be_bytes());
    bytes.extend(value);
}

fn wire_quad(points: [Point2; 3]) -> Vec<u8> {
    let mut bytes = vec![0, 2];
    for point in points {
        wire_real(&mut bytes, point.x());
        wire_real(&mut bytes, point.y());
    }
    bytes
}

fn wire_scalar(value: Real) -> Vec<u8> {
    let mut bytes = vec![1];
    wire_real(&mut bytes, &value);
    bytes
}

fn wire_support(source: u32, distance: Real) -> Vec<u8> {
    let mut bytes = vec![5];
    bytes.extend_from_slice(&source.to_be_bytes());
    wire_real(&mut bytes, &distance);
    bytes
}

fn wire_parallel(support: u32, start: u32, end: u32, reversed: bool) -> Vec<u8> {
    let mut bytes = vec![6];
    for reference in [support, start, end] {
        bytes.extend_from_slice(&reference.to_be_bytes());
    }
    bytes.push(u8::from(reversed));
    bytes
}

fn wire_document(nodes: Vec<Vec<u8>>, paths: &[u32]) -> String {
    use std::fmt::Write;
    let mut bytes = b"HCP4".to_vec();
    bytes.extend_from_slice(&u32::try_from(nodes.len()).unwrap().to_be_bytes());
    bytes.extend(nodes.into_iter().flatten());
    bytes.extend_from_slice(&u32::try_from(paths.len()).unwrap().to_be_bytes());
    for reference in paths {
        bytes.extend_from_slice(&reference.to_be_bytes());
    }
    let mut extension = String::from("4:");
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{extension}" d="M0 0 L1 1"/></svg>"#
    )
}

#[test]
fn parallel_replay_rejects_invalid_references_and_changed_singular_ranges() {
    use hypercurve::{ExactCurveError, SvgError};
    let source = || wire_quad([point(0, 0), Point2::new(q(1, 2), 0.into()), point(1, 1)]);
    for support in [0, 2, 4, 99] {
        let encoded = wire_document(
            vec![
                source(),
                wire_support(0, q(-1, 4)),
                wire_scalar(Real::zero()),
                wire_scalar(Real::one()),
                wire_parallel(support, 2, 3, false),
            ],
            &[4],
        );
        assert!(matches!(
            import_svg_document(&encoded),
            Err(SvgError::MalformedInput(_))
        ));
    }
    let forward = wire_document(vec![source(), wire_support(2, Real::one())], &[]);
    assert!(matches!(
        import_svg_document(&forward),
        Err(SvgError::MalformedInput(_))
    ));
    let wrong_kind = wire_document(
        vec![wire_scalar(Real::one()), wire_support(0, Real::one())],
        &[],
    );
    assert!(matches!(
        import_svg_document(&wrong_kind),
        Err(SvgError::MalformedInput(_))
    ));
    for start_end in [(Real::one(), Real::zero()), (q(1, 2), q(1, 2))] {
        let encoded = wire_document(
            vec![
                source(),
                wire_support(0, q(-1, 4)),
                wire_scalar(start_end.0),
                wire_scalar(start_end.1),
                wire_parallel(1, 2, 3, false),
            ],
            &[4],
        );
        assert!(matches!(
            import_svg_document(&encoded),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
    for source in [source(), wire_quad([point(0, 0), point(0, 0), point(1, 1)])] {
        let encoded = wire_document(
            vec![
                source,
                wire_support(0, Real::one()),
                wire_scalar(Real::zero()),
                wire_scalar(Real::one()),
                wire_parallel(1, 2, 3, false),
            ],
            &[],
        );
        // The requested parallel range is invalid even when the final path is empty.
        assert!(matches!(
            import_svg_document(&encoded),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
    let regular_piece = wire_document(
        vec![
            source(),
            wire_support(0, Real::one()),
            wire_scalar(Real::zero()),
            wire_scalar(q(1, 4)),
            wire_parallel(1, 2, 3, true),
        ],
        &[4],
    );
    let decoded = import_svg_document(&regular_piece).unwrap();
    assert!(
        decoded.paths()[0].curves()[0]
            .analytic_parallel()
            .unwrap()
            .is_reversed()
    );
}

#[test]
fn rational_parallel_replay_checks_only_the_declared_range_for_poles() {
    use hypercurve::{ExactCurveError, SvgError, UncertaintyReason};
    // C(t)=(t/(1-2t),0). The authored unit chart has a pole, but [2,3] does not.
    let mut source = vec![0, 5];
    source.extend_from_slice(&2_u32.to_be_bytes());
    for value in [0, 0, 1, 1, 0, -1] {
        wire_real(&mut source, &Real::from(value));
    }
    for distance in [Real::zero(), q(1, 4)] {
        let encoded = wire_document(
            vec![
                source.clone(),
                wire_support(0, distance.clone()),
                wire_scalar(2.into()),
                wire_scalar(3.into()),
                wire_parallel(1, 2, 3, false),
            ],
            &[4],
        );
        let decoded = import_svg_document(&encoded).unwrap();
        let curve = &decoded.paths()[0].curves()[0];
        assert!(
            curve
                .start()
                .coincides_with(&Point2::new(q(-2, 3), distance.clone()).into())
                .unwrap()
        );
        assert!(
            curve
                .end()
                .coincides_with(&Point2::new(q(-3, 5), distance.clone()).into())
                .unwrap()
        );
        let exported = document(decoded.paths()[0].clone());
        let repeated = import_svg_document(&exported).unwrap();
        assert!(
            repeated.paths()[0].curves()[0]
                .start()
                .coincides_with(&curve.start())
                .unwrap()
        );
        let invalid = wire_document(
            vec![
                source.clone(),
                wire_support(0, distance),
                wire_scalar(q(1, 4)),
                wire_scalar(q(3, 4)),
                wire_parallel(1, 2, 3, false),
            ],
            &[4],
        );
        assert!(
            matches!(import_svg_document(&invalid), Err(SvgError::Exact(ExactCurveError::Blocked(ref blocker))) if blocker.reason() == UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn exterior_parallel_projection_keeps_the_original_parameter_chart() {
    let source = QuadraticBezier2::new(point(0, 0), Point2::new(q(1, 2), 0.into()), point(1, 0));
    let curve = Curve2::try_analytic_parallel(
        source.parallel_left(q(1, 4)).unwrap(),
        range(exact(2.into()), exact(3.into())),
    )
    .unwrap();
    let options = hypercurve::FiniteProjectionOptions::try_new(0.01).unwrap();
    for (curve, expected) in [
        (curve.clone(), [[2.0, 0.25], [3.0, 0.25]]),
        (curve.reversed().unwrap(), [[3.0, 0.25], [2.0, 0.25]]),
    ] {
        let path = CurvePath2::try_new(vec![curve]).unwrap();
        assert_eq!(
            path.project_to_finite_polyline(&options).unwrap().points(),
            &expected
        );
        let encoded = document(path);
        let replay = import_svg_document(&encoded).unwrap();
        let domain = replay.paths()[0].curves()[0].parameter_domain();
        assert_eq!(
            domain.start().compare(&Real::from(2).into()).unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            domain.end().compare(&Real::from(3).into()).unwrap(),
            std::cmp::Ordering::Equal
        );
    }
}

#[test]
fn parallel_byte_exhaustion_keeps_a_later_replay_available() {
    use hypercurve::{
        SvgError, SvgOptions, export_svg_document_with_options, import_svg_document_with_options,
    };
    let curve = unit_curve(parabola().parallel_left(q(-1, 4)).unwrap());
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![CurvePath2::try_new(vec![curve]).unwrap()],
    );
    let small = SvgOptions {
        max_extension_bytes: 64,
        ..SvgOptions::default()
    };
    assert!(matches!(
        export_svg_document_with_options(&geometry, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let document = export_svg_document(&geometry).unwrap();
    assert!(matches!(
        import_svg_document_with_options(&document, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let replay = import_svg_document(&document).unwrap();
    assert_eq!(
        replay.paths()[0].curves()[0]
            .analytic_parallel()
            .unwrap()
            .parallel()
            .distance(),
        &q(-1, 4)
    );
}

#[test]
fn selected_parallel_trim_replays_through_a_reflected_algebraic_similarity() {
    use hypercurve::{
        BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Similarity2,
    };
    let beta = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-1).into(), 0.into(), 2.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), q(3, 4)).unwrap(),
        )
        .unwrap(),
    );
    let curve = unit_curve(parabola().parallel_left(q(-1, 4)).unwrap())
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), beta.clone().into()).unwrap(),
        )
        .unwrap();
    let imported =
        import_svg_document(&document(CurvePath2::try_new(vec![curve]).unwrap())).unwrap();
    let scale = Real::from(2).sqrt().unwrap();
    let transform = Similarity2::try_from_real_affine(
        -&scale,
        Real::zero(),
        Real::zero(),
        scale.clone(),
        1.into(),
        (-1).into(),
    )
    .unwrap();
    let transformed = imported.paths()[0].curves()[0]
        .transform_similarity(&transform)
        .unwrap();
    let replay =
        import_svg_document(&document(CurvePath2::try_new(vec![transformed]).unwrap())).unwrap();
    let curve = &replay.paths()[0].curves()[0];
    let expected = Point2::new(
        q(3, 4) - (&scale / Real::from(2)).unwrap(),
        q(-5, 4) + (&scale / Real::from(4)).unwrap(),
    );
    assert!(
        curve
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&expected.into())
            .unwrap()
    );
    assert_eq!(
        curve
            .parameter_domain()
            .end()
            .compare(&beta.into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    let distance = curve.analytic_parallel().unwrap().parallel().distance();
    assert_eq!(
        distance.partial_cmp(&(scale / Real::from(4)).unwrap()),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
fn different_parallel_distances_replay_one_shared_source() {
    let line = |start: Point2, end: Point2| {
        let mut bytes = vec![0, 0];
        for point in [start, end] {
            wire_real(&mut bytes, point.x());
            wire_real(&mut bytes, point.y());
        }
        bytes
    };
    let encoded = wire_document(
        vec![
            wire_quad([point(0, 0), Point2::new(q(1, 2), 0.into()), point(1, 0)]),
            wire_support(0, q(1, 4)),
            wire_support(0, q(1, 2)),
            wire_scalar(Real::zero()),
            wire_scalar(Real::one()),
            wire_parallel(1, 3, 4, false),
            line(
                Point2::new(1.into(), q(1, 4)),
                Point2::new(1.into(), q(1, 2)),
            ),
            wire_parallel(2, 3, 4, true),
            line(
                Point2::new(0.into(), q(1, 2)),
                Point2::new(0.into(), q(1, 4)),
            ),
        ],
        &[5, 6, 7, 8],
    );
    let first = import_svg_document(&encoded).unwrap();
    let second = import_svg_document(&document(first.paths()[0].clone())).unwrap();
    for geometry in [first, second] {
        let curves = geometry.paths()[0].curves();
        let lower = curves[0].analytic_parallel().unwrap();
        let upper = curves[2].analytic_parallel().unwrap();
        assert!(std::ptr::eq(
            lower.parallel().source(),
            upper.parallel().source()
        ));
        assert_eq!(lower.parallel().distance(), &q(1, 4));
        assert_eq!(upper.parallel().distance(), &q(1, 2));
        assert!(
            curves[0]
                .start()
                .coincides_with(&Point2::new(0.into(), q(1, 4)).into())
                .unwrap()
        );
        assert!(
            curves[2]
                .start()
                .coincides_with(&Point2::new(1.into(), q(1, 2)).into())
                .unwrap()
        );
        let region =
            CurveRegion2::try_from_boundary_paths(geometry.paths(), FillRule::NonZero).unwrap();
        assert_eq!(
            region
                .classify_point(&Point2::new(q(1, 2), q(3, 8)).into())
                .unwrap(),
            RegionPointLocation::Inside
        );
    }
}
