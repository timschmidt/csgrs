#![cfg(feature = "svg")]

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, CurveParameterRange2, CurvePath2,
    CurvePointLocations2, CurveRegion2, ExactCurveError, Point2, QuadraticBezier2, Real,
    Similarity2, SvgError, SvgGeometry2, SvgOptions, export_svg_document,
    export_svg_document_with_options, import_svg_document, import_svg_document_with_options,
};
use std::cmp::Ordering;
use std::fmt::Write;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn source() -> Curve2 {
    QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0)).into()
}

fn root(coefficients: Vec<Real>, lower: Real, upper: Real) -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(lower, upper).unwrap(),
        )
        .unwrap(),
    )
}

fn beta() -> BezierParameter2 {
    root(vec![(-1).into(), 0.into(), 2.into()], q(1, 2), q(3, 4))
}

fn restrict(source: &Curve2, start: CurveParameter2, end: CurveParameter2) -> Curve2 {
    source
        .restrict_source(CurveParameterRange2::try_new(start, end).unwrap())
        .unwrap()
}

fn geometry(curves: Vec<Curve2>) -> SvgGeometry2 {
    SvgGeometry2::new(
        CurveRegion2::empty(),
        Vec::new(),
        vec![CurvePath2::try_new(curves).unwrap()],
    )
}

fn verify_trimmed_pair(document: &str) {
    let imported = import_svg_document(document).unwrap();
    assert!(imported.region().is_empty());
    assert!(imported.wires().is_empty());
    let [path] = imported.paths() else {
        panic!("one retained path")
    };
    let [forward, backward] = path.curves() else {
        panic!("two retained restrictions")
    };
    let (native, range, reversed) = forward.source_restriction().unwrap();
    assert!(!reversed);
    assert_eq!(native.geometry(), source().geometry());
    assert_eq!(
        range.start().compare(&Real::zero().into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&beta().into()).unwrap(),
        Ordering::Equal
    );
    let (other_native, other_range, reversed) = backward.source_restriction().unwrap();
    assert!(reversed);
    assert!(std::ptr::eq(
        native.geometry().unwrap(),
        other_native.geometry().unwrap()
    ));
    assert_eq!(
        range.end().compare(other_range.end()).unwrap(),
        Ordering::Equal
    );
    assert!(forward.end().coincides_with(&backward.start()).unwrap());
    assert!(forward.start().coincides_with(&backward.end()).unwrap());

    // This interior point follows directly from x=2t, y=4t(1-t).
    let expected = point(1, 1).into();
    assert!(
        forward
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
    let CurvePointLocations2::Locations(locations) = forward.point_locations(&expected).unwrap()
    else {
        panic!("a noncollapsed quadratic has one visit")
    };
    assert_eq!(locations.len(), 1);
    assert_eq!(
        locations[0]
            .parameter()
            .unwrap()
            .compare(&q(1, 2).into())
            .unwrap(),
        Ordering::Equal
    );
    let shortened = restrict(forward, q(1, 2).into(), beta().into());
    let replayed =
        import_svg_document(&export_svg_document(&geometry(vec![shortened.clone()])).unwrap())
            .unwrap();
    let next = &replayed.paths()[0].curves()[0];
    assert!(next.start().coincides_with(&expected).unwrap());
    assert!(next.end().coincides_with(&forward.end()).unwrap());
    let contacts = next
        .intersect_curve(&shortened.reversed().unwrap())
        .unwrap();
    assert!(contacts.is_complete());
    assert_eq!(contacts.overlaps().len(), 1);
}

#[test]
fn fresh_process_replays_selected_trim_before_further_operations() {
    const INPUT: &str = "HYPERCURVE_SELECTED_SVG_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        verify_trimmed_pair(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let trimmed = restrict(&source(), Real::zero().into(), beta().into());
    let document = export_svg_document(&geometry(vec![
        trimmed.clone(),
        trimmed.reversed().unwrap(),
    ]))
    .unwrap();
    assert!(document.contains("data-hypercurve-path=\"4:"));
    let path = std::env::temp_dir().join(format!(
        "hypercurve-selected-svg-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &document).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replays_selected_trim_before_further_operations",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(
        output.status.success(),
        "fresh process failed: {}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn projective_parameters_replay_one_shared_selected_root() {
    let selected = root(
        vec![
            (-1).into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            2.into(),
        ],
        q(1, 2),
        Real::one(),
    );
    let first = selected
        .projective_image(&[0.into(), 1.into()], &[1.into(), 1.into()])
        .unwrap();
    let second = selected
        .projective_image(&[0.into(), 2.into()], &[1.into(), 1.into()])
        .unwrap();
    let native = source();
    let a = restrict(&native, Real::zero().into(), first.clone().into());
    let b = restrict(&native, first.clone().into(), second.clone().into());
    let input = geometry(vec![
        a.clone(),
        b.clone(),
        b.reversed().unwrap(),
        a.reversed().unwrap(),
        native,
    ]);
    let document = export_svg_document(&input).unwrap();
    let imported = import_svg_document(&document).unwrap();
    let curves = imported.paths()[0].curves();
    assert_eq!(curves.len(), 5);
    let (_, first_range, _) = curves[0].source_restriction().unwrap();
    let (_, second_range, _) = curves[1].source_restriction().unwrap();
    let BezierParameter2::Algebraic(first_image) = first_range.end().as_bezier_parameter().unwrap()
    else {
        panic!("selected image")
    };
    let BezierParameter2::Algebraic(second_image) =
        second_range.end().as_bezier_parameter().unwrap()
    else {
        panic!("selected image")
    };
    let (base_a, numerator_a, denominator_a) = first_image.source_chart().unwrap();
    let (base_b, numerator_b, denominator_b) = second_image.source_chart().unwrap();
    assert!(std::ptr::eq(base_a.polynomial(), base_b.polynomial()));
    assert_eq!(base_a.polynomial().coefficients().len(), 8);
    assert_eq!(numerator_a, &[Real::zero(), Real::one()]);
    assert_eq!(numerator_b, &[Real::zero(), Real::from(2)]);
    assert_eq!(denominator_a, &[Real::one(), Real::one()]);
    assert_eq!(denominator_b, denominator_a);
    assert_eq!(
        first_range.end().compare(&first.into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        second_range.end().compare(&second.into()).unwrap(),
        Ordering::Equal
    );
    assert!(curves[0].end().coincides_with(&curves[1].start()).unwrap());
    // Determinism is checked separately from selected-value and incidence proofs.
    assert_eq!(export_svg_document(&input).unwrap(), document);
}

#[test]
fn algebraic_coefficients_and_authoritative_metadata_survive_replay() {
    let a = Real::from(2).sqrt().unwrap();
    let b = Real::from(3).sqrt().unwrap();
    let native: Curve2 = QuadraticBezier2::new(
        point(0, 0),
        Point2::new(a.clone(), b.clone()),
        Point2::new(Real::from(2) * &a, Real::zero()),
    )
    .into();
    let trimmed = restrict(&native, Real::zero().into(), beta().into());
    let document = export_svg_document(&geometry(vec![trimmed])).unwrap();
    let start = document.find(" d=\"").unwrap() + 4;
    let end = start + document[start..].find('"').unwrap();
    let mut changed_display = document.clone();
    changed_display.replace_range(start..end, "M900 900 L901 900");
    let imported = import_svg_document(&changed_display).unwrap();
    let curve = &imported.paths()[0].curves()[0];
    assert_eq!(
        curve.source_restriction().unwrap().0.geometry(),
        native.geometry()
    );
    let expected = Point2::new(a, (b / Real::from(2)).unwrap()).into();
    assert!(
        curve
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
}

#[test]
fn replayed_restriction_consumes_a_similarity_and_another_archive() {
    let trimmed = restrict(&source(), Real::zero().into(), beta().into());
    let imported =
        import_svg_document(&export_svg_document(&geometry(vec![trimmed])).unwrap()).unwrap();
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
    let next =
        import_svg_document(&export_svg_document(&geometry(vec![transformed])).unwrap()).unwrap();
    let expected = Point2::new(Real::one() - &scale, scale - Real::one()).into();
    let curve = &next.paths()[0].curves()[0];
    assert!(
        curve
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
    assert_eq!(
        curve
            .parameter_domain()
            .end()
            .compare(&beta().into())
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn source_restrictions_replay_each_authored_family_in_its_original_chart() {
    let controls = vec![point(0, 0), point(1, 2), point(3, 0)];
    let knots = vec![0, 0, 0, 1, 1, 1]
        .into_iter()
        .map(Real::from)
        .collect::<Vec<_>>();
    let sources = vec![
        Curve2::from(hypercurve::LineSeg2::try_new(point(0, 0), point(3, 0)).unwrap()),
        Curve2::from(
            hypercurve::CircularArc2::from_bulge(point(0, 0), point(2, 0), q(1, 2)).unwrap(),
        ),
        source(),
        Curve2::from(hypercurve::CubicBezier2::new(
            point(0, 0),
            point(0, 2),
            point(2, 1),
            point(3, 0),
        )),
        Curve2::from(
            hypercurve::RationalQuadraticBezier2::try_new(
                point(0, 0),
                point(1, 2),
                point(3, 0),
                Real::one(),
                q(1, 3),
                Real::one(),
            )
            .unwrap(),
        ),
        Curve2::from(
            hypercurve::RationalBezier2::try_new(
                vec![point(0, 0), point(1, 2), point(2, -1), point(3, 0)],
                vec![Real::one(), q(1, 3), 2.into(), Real::one()],
            )
            .unwrap(),
        ),
        Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone()).unwrap(),
        Curve2::try_nurbs(2, controls, vec![Real::one(), q(1, 3), Real::one()], knots).unwrap(),
    ];
    for source in sources {
        let trimmed = restrict(&source, q(1, 4).into(), q(3, 4).into());
        let document = export_svg_document(&geometry(vec![trimmed.clone()])).unwrap();
        let imported = import_svg_document(&document).unwrap();
        let curve = &imported.paths()[0].curves()[0];
        let (replayed_source, range, reversed) = curve.source_restriction().unwrap();
        assert!(!reversed);
        assert_eq!(replayed_source.geometry(), source.geometry());
        assert_eq!(
            range.start().compare(&q(1, 4).into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            range.end().compare(&q(3, 4).into()).unwrap(),
            Ordering::Equal
        );
        assert!(curve.start().coincides_with(&trimmed.start()).unwrap());
        assert!(curve.end().coincides_with(&trimmed.end()).unwrap());
        let shorter = restrict(curve, q(1, 2).into(), q(3, 4).into());
        assert!(
            shorter
                .start()
                .coincides_with(&source.point_at(&q(1, 2).into()).unwrap())
                .unwrap()
        );
    }
}

fn put_len(bytes: &mut Vec<u8>, count: usize) {
    bytes.extend_from_slice(&u32::try_from(count).unwrap().to_be_bytes());
}

fn put_real(bytes: &mut Vec<u8>, value: Real) {
    let encoded = value.to_bytes();
    put_len(bytes, encoded.len());
    bytes.extend(encoded);
}

fn root_record(coefficients: &[i32], lower: Real, upper: Real) -> Vec<u8> {
    let mut bytes = vec![2];
    put_len(&mut bytes, coefficients.len());
    for coefficient in coefficients {
        put_real(&mut bytes, (*coefficient).into());
    }
    put_real(&mut bytes, lower);
    put_real(&mut bytes, upper);
    bytes
}

fn scalar_record(value: Real) -> Vec<u8> {
    let mut bytes = vec![1];
    put_real(&mut bytes, value);
    bytes
}

fn native_record() -> Vec<u8> {
    let mut bytes = vec![0, 2];
    for coordinate in [0, 0, 1, 2, 2, 0] {
        put_real(&mut bytes, coordinate.into());
    }
    bytes
}

fn restriction_record(source: usize, start: usize, end: usize) -> Vec<u8> {
    let mut bytes = vec![4];
    for reference in [source, start, end] {
        put_len(&mut bytes, reference);
    }
    bytes.push(0);
    bytes
}

fn document_bytes(bytes: &[u8]) -> String {
    let mut encoded = String::from("4:");
    for byte in bytes {
        write!(&mut encoded, "{byte:02x}").unwrap();
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{encoded}" d="M0 0 L1 1"/></svg>"#
    )
}

fn graph_document(records: &[Vec<u8>], path: &[usize]) -> String {
    let mut bytes = b"HCP4".to_vec();
    put_len(&mut bytes, records.len());
    for record in records {
        bytes.extend(record);
    }
    put_len(&mut bytes, path.len());
    for reference in path {
        put_len(&mut bytes, *reference);
    }
    document_bytes(&bytes)
}

#[test]
fn invalid_root_selectors_and_domains_never_become_geometry() {
    for selected in [
        root_record(&[1, 0, 1], q(1, 2), q(3, 4)),
        root_record(&[-1, 0, 1], (-2).into(), 2.into()),
        root_record(&[-1, 0, 2], q(1, 10), q(1, 5)),
    ] {
        let document = graph_document(
            &[
                native_record(),
                scalar_record(0.into()),
                selected,
                restriction_record(0, 1, 2),
            ],
            &[3],
        );
        assert!(matches!(
            import_svg_document(&document),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
    for (start, end) in [
        (q(-1, 2), q(1, 2)),
        (q(1, 2), 2.into()),
        (q(3, 4), q(1, 4)),
        (q(1, 2), q(1, 2)),
    ] {
        let document = graph_document(
            &[
                native_record(),
                scalar_record(start),
                scalar_record(end),
                restriction_record(0, 1, 2),
            ],
            &[3],
        );
        assert!(matches!(
            import_svg_document(&document),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
}

#[test]
fn malformed_graph_references_and_chart_poles_are_explicit_failures() {
    for restriction in [
        restriction_record(1, 1, 2),
        restriction_record(3, 1, 2),
        restriction_record(0, 0, 2),
        restriction_record(0, 1, 99),
    ] {
        let document = graph_document(
            &[
                native_record(),
                scalar_record(0.into()),
                scalar_record(q(1, 2)),
                restriction,
            ],
            &[3],
        );
        assert!(matches!(
            import_svg_document(&document),
            Err(SvgError::MalformedInput(_))
        ));
    }
    let mut image = vec![3];
    put_len(&mut image, 0);
    for coefficient in [0, 1, 0, 0] {
        put_real(&mut image, coefficient.into());
    }
    let document = graph_document(&[root_record(&[-1, 0, 2], q(1, 2), q(3, 4)), image], &[]);
    assert!(matches!(
        import_svg_document(&document),
        Err(SvgError::Exact(ExactCurveError::Blocked(ref blocker)))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    let wrong_path = graph_document(&[scalar_record(0.into())], &[0]);
    assert!(matches!(
        import_svg_document(&wrong_path),
        Err(SvgError::MalformedInput(_))
    ));
    let huge = document_bytes(b"HCP4\xff\xff\xff\xff");
    assert!(matches!(
        import_svg_document(&huge),
        Err(SvgError::SizeOverflow { .. })
    ));
    let truncated = document_bytes(b"HCP4\x00\x00\x00\x01\x02");
    assert!(matches!(
        import_svg_document(&truncated),
        Err(SvgError::MalformedInput(_))
    ));
    assert!(matches!(
        import_svg_document(&graph_document(&[vec![255]], &[])),
        Err(SvgError::Unsupported(_))
    ));
}

#[test]
fn exact_extension_byte_exhaustion_does_not_poison_retry() {
    let trimmed = restrict(&source(), Real::zero().into(), beta().into());
    let input = geometry(vec![trimmed]);
    let few_segments = SvgOptions {
        curve_tolerance: 1.0e-6,
        max_curve_segments: 8,
        ..SvgOptions::default()
    };
    assert!(matches!(
        export_svg_document_with_options(&input, few_segments),
        Err(SvgError::SizeOverflow { .. })
    ));
    let small = SvgOptions {
        max_extension_bytes: 64,
        ..SvgOptions::default()
    };
    assert!(matches!(
        export_svg_document_with_options(&input, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let document = export_svg_document(&input).unwrap();
    assert!(matches!(
        import_svg_document_with_options(&document, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let imported = import_svg_document(&document).unwrap();
    assert_eq!(
        imported.paths()[0].curves()[0]
            .parameter_domain()
            .end()
            .compare(&beta().into())
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        export_svg_document(&geometry(vec![source()]))
            .unwrap()
            .contains("data-hypercurve-path=\"3:")
    );
}

#[test]
fn finite_projection_consumes_the_retained_domain_and_traversal() {
    let native: Curve2 = hypercurve::LineSeg2::try_new(point(0, 0), point(8, 0))
        .unwrap()
        .into();
    let trimmed = restrict(&native, q(1, 4).into(), q(3, 4).into());
    let options = hypercurve::FiniteProjectionOptions::try_new(0.01).unwrap();
    for (curve, expected) in [
        (trimmed.clone(), [[2.0, 0.0], [6.0, 0.0]]),
        (trimmed.reversed().unwrap(), [[6.0, 0.0], [2.0, 0.0]]),
    ] {
        let path = CurvePath2::try_new(vec![curve]).unwrap();
        let projection = path.project_to_finite_polyline(&options).unwrap();
        assert_eq!(projection.points(), &expected);
        assert!(!projection.is_closed());
    }
    assert_eq!(
        trimmed
            .parameter_domain()
            .start()
            .compare(&q(1, 4).into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        trimmed
            .parameter_domain()
            .end()
            .compare(&q(3, 4).into())
            .unwrap(),
        Ordering::Equal
    );
}
