#![cfg(feature = "svg")]

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, CurvePath2, CurvePoint2,
    CurvePointDefinition2, CurvePointLocations2, CurveRegion2, ExactCurveError, FillRule,
    OffsetCornerStyle2, Point2, Real, RegionPointLocation, SvgError, SvgGeometry2,
    export_svg_document, import_svg_document,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected() -> CurveParameter2 {
    let mut coefficients = vec![Real::zero(); 8];
    coefficients[0] = (-1).into();
    coefficients[7] = 2.into();
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
        )
        .unwrap(),
    )
    .into()
}

fn at(parameter: &CurveParameter2, y: i32) -> CurvePoint2 {
    parameter
        .affine_point(1.into(), 0.into(), 0.into(), y.into())
        .unwrap()
}

fn document(path: CurvePath2) -> String {
    export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![path],
    ))
    .unwrap()
}

fn verify_selected(document_text: &str) {
    let replay = import_svg_document(document_text).unwrap();
    let path = &replay.paths()[0];
    assert_eq!(path.curves().len(), 3);
    let beta = selected();
    for (curve, (start, end)) in path.curves().iter().zip([(0, 1), (1, 2), (2, 0)]) {
        assert!(curve.geometry().is_none());
        assert!(curve.start().coordinates().is_none());
        assert!(curve.start().coincides_with(&at(&beta, start)).unwrap());
        assert!(curve.end().coincides_with(&at(&beta, end)).unwrap());
        let CurvePointDefinition2::Rational { parameter, .. } = curve.start().definition().unwrap()
        else {
            panic!("selected native source point remains authoritative")
        };
        assert_eq!(parameter.compare(&beta).unwrap(), std::cmp::Ordering::Equal);
    }
    let contacts = path.curves()[0].intersect_curve(&path.curves()[2]).unwrap();
    assert!(contacts.is_complete());
    assert_eq!(contacts.overlaps().len(), 1);
    let CurvePointLocations2::Locations(locations) =
        path.curves()[2].point_locations(&at(&beta, 1)).unwrap()
    else {
        panic!("one selected midpoint")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = path.curves()[2]
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    assert!(first.end().coincides_with(&at(&beta, 1)).unwrap());
    assert!(second.start().coincides_with(&first.end()).unwrap());
    let second_replay =
        import_svg_document(&document(CurvePath2::try_new(vec![first, second]).unwrap())).unwrap();
    assert!(
        second_replay.paths()[0]
            .start()
            .coincides_with(&at(&beta, 2))
            .unwrap()
    );
    assert!(
        second_replay.paths()[0]
            .end()
            .coincides_with(&at(&beta, 0))
            .unwrap()
    );
    assert_eq!(document(path.clone()), document_text);
}

#[test]
fn selected_chord_split_replays_in_a_fresh_process_before_further_incidence() {
    const INPUT: &str = "HYPERCURVE_CHORD_SVG_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        verify_selected(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let beta = selected();
    let chord = Curve2::try_line(at(&beta, 0), at(&beta, 2)).unwrap();
    let CurvePointLocations2::Locations(locations) = chord.point_locations(&at(&beta, 1)).unwrap()
    else {
        panic!("one selected midpoint")
    };
    let (first, second) = chord
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    let encoded =
        document(CurvePath2::try_new(vec![first, second, chord.reversed().unwrap()]).unwrap());
    let path =
        std::env::temp_dir().join(format!("hypercurve-chord-svg-{}.svg", std::process::id()));
    std::fs::write(&path, encoded).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_chord_split_replays_in_a_fresh_process_before_further_incidence",
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

#[test]
fn algebraic_coordinate_chords_replay_before_region_boolean_and_offset() {
    let x = Real::from(2).sqrt().unwrap();
    let points: [CurvePoint2; 4] = [
        Point2::new(x.clone(), 0.into()).into(),
        Point2::new(&x + Real::one(), 0.into()).into(),
        Point2::new(&x + Real::one(), 2.into()).into(),
        Point2::new(x.clone(), 2.into()).into(),
    ];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| Curve2::try_line(points[i].clone(), points[(i + 1) % 4].clone()).unwrap())
            .collect(),
    )
    .unwrap();
    let replay = import_svg_document(&document(path)).unwrap();
    for (i, curve) in replay.paths()[0].curves().iter().enumerate() {
        assert!(curve.start().coincides_with(&points[i]).unwrap());
        assert!(curve.end().coincides_with(&points[(i + 1) % 4]).unwrap());
    }
    let region = CurveRegion2::try_from_boundary_paths(replay.paths(), FillRule::NonZero).unwrap();
    let inside = Point2::new(&x + q(1, 2), 1.into()).into();
    let outside = Point2::new(&x - q(1, 2), 1.into()).into();
    assert_eq!(
        region.classify_point(&inside).unwrap(),
        RegionPointLocation::Inside
    );
    assert_eq!(
        region.classify_point(&outside).unwrap(),
        RegionPointLocation::Outside
    );
    assert!(
        region
            .boolean_regions(&region)
            .unwrap()
            .difference()
            .is_empty()
    );
    let expanded = region.offset(q(1, 8), &OffsetCornerStyle2::Round).unwrap();
    assert_eq!(
        expanded.classify_point(&inside).unwrap(),
        RegionPointLocation::Inside
    );
    assert!(
        expanded
            .boolean_regions(&expanded)
            .unwrap()
            .difference()
            .is_empty()
    );
}

fn wire_real(bytes: &mut Vec<u8>, value: Real) {
    let value = value.to_bytes();
    bytes.extend_from_slice(&u32::try_from(value.len()).unwrap().to_be_bytes());
    bytes.extend(value);
}

fn wire_point(x: i32, y: i32) -> Vec<u8> {
    let mut bytes = vec![7];
    wire_real(&mut bytes, x.into());
    wire_real(&mut bytes, y.into());
    bytes
}

fn wire_support(start: u32, end: u32, axis: u8, increases: bool) -> Vec<u8> {
    let mut bytes = vec![11];
    bytes.extend_from_slice(&start.to_be_bytes());
    bytes.extend_from_slice(&end.to_be_bytes());
    bytes.extend([axis, u8::from(increases)]);
    bytes
}

fn wire_range(support: u32, start: u32, end: u32, axis: u8, increases: bool) -> Vec<u8> {
    let mut bytes = vec![12];
    for reference in [support, start, end] {
        bytes.extend_from_slice(&reference.to_be_bytes());
    }
    bytes.extend([axis, u8::from(increases)]);
    bytes
}

fn wire(nodes: Vec<Vec<u8>>, paths: &[u32]) -> (String, String) {
    use std::fmt::Write;
    let mut bytes = b"HCP4".to_vec();
    bytes.extend_from_slice(&u32::try_from(nodes.len()).unwrap().to_be_bytes());
    bytes.extend(nodes.into_iter().flatten());
    bytes.extend_from_slice(&u32::try_from(paths.len()).unwrap().to_be_bytes());
    for path in paths {
        bytes.extend_from_slice(&path.to_be_bytes());
    }
    let mut metadata = String::from("4:");
    for byte in bytes {
        write!(&mut metadata, "{byte:02x}").unwrap();
    }
    (
        format!(
            r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{metadata}" d="M0 0 L1 1"/></svg>"#
        ),
        metadata,
    )
}

#[test]
fn replay_retains_a_valid_alternative_chart_and_exterior_support_range() {
    let (input, metadata) = wire(
        vec![
            wire_point(0, 0),
            wire_point(2, 1),
            wire_support(0, 1, 1, true),
            wire_point(6, 3),
            wire_point(4, 2),
            wire_range(2, 3, 4, 1, false),
        ],
        &[5],
    );
    let decoded = import_svg_document(&input).unwrap();
    let curve = &decoded.paths()[0].curves()[0];
    assert!(
        curve
            .start()
            .coincides_with(&Point2::from_values(6, 3).into())
            .unwrap()
    );
    assert!(
        curve
            .end()
            .coincides_with(&Point2::from_values(4, 2).into())
            .unwrap()
    );
    let output = document(decoded.paths()[0].clone());
    assert!(
        output.contains(&format!(r#"data-hypercurve-path="{metadata}""#)),
        "the supplied y chart must survive even though the ordinary constructor schedules x first"
    );
}

#[test]
fn invalid_chord_definitions_are_checked_before_an_empty_final_path() {
    for (axis, increases) in [(1, true), (0, false)] {
        let (input, _) = wire(
            vec![
                wire_point(0, 0),
                wire_point(2, 0),
                wire_support(0, 1, axis, increases),
            ],
            &[],
        );
        assert!(matches!(
            import_svg_document(&input),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
    for (start, end) in [(0, 0), (1, 0), (3, 1)] {
        let (input, _) = wire(
            vec![
                wire_point(0, 0),
                wire_point(2, 0),
                wire_support(0, 1, 0, true),
                wire_point(1, 1),
                wire_range(2, start, end, 0, true),
            ],
            &[],
        );
        assert!(matches!(
            import_svg_document(&input),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ));
    }
    for support in [0, 3, 99] {
        let (input, _) = wire(
            vec![
                wire_point(0, 0),
                wire_point(2, 0),
                wire_support(0, 1, 0, true),
                wire_range(support, 0, 1, 0, true),
            ],
            &[],
        );
        assert!(matches!(
            import_svg_document(&input),
            Err(SvgError::MalformedInput(_))
        ));
    }
}

#[test]
fn correlated_rational_expression_endpoints_replay_their_shared_selected_root() {
    let beta = selected().as_bezier_parameter().unwrap().clone();
    let make = |offset: Real| {
        CurvePointDefinition2::RationalExpression {
            parameter: beta.clone(),
            x_numerator: vec![0.into(), 1.into()],
            y_numerator: vec![offset.clone(), offset, 1.into()],
            denominator: vec![1.into(), 1.into()],
        }
        .into_point()
        .unwrap()
    };
    // x=beta/(1+beta), y=beta^2/(1+beta)+offset.
    let start = make(Real::zero());
    let end = make(Real::one());
    assert!(matches!(
        start.definition().unwrap(),
        CurvePointDefinition2::RationalExpression { .. }
    ));
    let encoded = document(
        CurvePath2::try_new(vec![Curve2::try_line(start.clone(), end.clone()).unwrap()]).unwrap(),
    );
    let replay = import_svg_document(&encoded).unwrap();
    let curve = &replay.paths()[0].curves()[0];
    assert!(curve.start().coincides_with(&start).unwrap());
    assert!(curve.end().coincides_with(&end).unwrap());
    for (point, offset) in [(curve.start(), Real::zero()), (curve.end(), Real::one())] {
        let CurvePointDefinition2::RationalExpression {
            parameter,
            x_numerator,
            y_numerator,
            denominator,
        } = point.definition().unwrap()
        else {
            panic!("preserve one correlated rational definition")
        };
        assert_eq!(
            CurveParameter2::from(parameter)
                .compare(&beta.clone().into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(x_numerator, vec![Real::zero(), Real::one()]);
        assert_eq!(y_numerator, vec![offset.clone(), offset, Real::one()]);
        assert_eq!(denominator, vec![Real::one(), Real::one()]);
    }
    let midpoint = make(q(1, 2));
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&midpoint).unwrap()
    else {
        panic!("one rational-expression midpoint")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = curve
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    let further =
        import_svg_document(&document(CurvePath2::try_new(vec![first, second]).unwrap())).unwrap();
    assert!(
        further.paths()[0].curves()[0]
            .end()
            .coincides_with(&midpoint)
            .unwrap()
    );
    assert_eq!(document(replay.paths()[0].clone()), encoded);
}

#[test]
fn a_parallel_endpoint_and_its_incident_chord_replay_one_source_and_selector() {
    use hypercurve::{BezierParameterRange2, QuadraticBezier2};
    let beta = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-1).into(), 0.into(), 2.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), q(3, 4)).unwrap(),
        )
        .unwrap(),
    );
    let source = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(q(1, 2), 0.into()),
        Point2::from_values(1, 1),
    );
    let curve = Curve2::try_analytic_parallel(
        source.parallel_left(q(-1, 4)).unwrap(),
        BezierParameterRange2::try_new(BezierParameter2::Exact(Real::zero()), beta.clone())
            .unwrap(),
    )
    .unwrap();
    let endpoint = curve.end();
    assert!(matches!(
        endpoint.definition().unwrap(),
        CurvePointDefinition2::Parallel { .. }
    ));
    for endpoint in [
        endpoint.clone(),
        endpoint.definition().unwrap().into_point().unwrap(),
    ] {
        let chord = Curve2::try_line(endpoint, Point2::from_values(2, 0).into()).unwrap();
        let encoded = document(CurvePath2::try_new(vec![curve.clone(), chord]).unwrap());
        let replay = import_svg_document(&encoded).unwrap();
        let curves = replay.paths()[0].curves();
        let CurvePointDefinition2::Parallel { source, parameter } =
            curves[1].start().definition().unwrap()
        else {
            panic!("retain the selected parallel endpoint")
        };
        assert!(std::ptr::eq(
            source.distance(),
            curves[0].analytic_parallel().unwrap().parallel().distance()
        ));
        assert_eq!(
            parameter.compare(&beta.clone().into()).unwrap(),
            std::cmp::Ordering::Equal
        );
        let t = q(1, 2).sqrt().unwrap();
        let speed = Real::from(3).sqrt().unwrap();
        let expected = Point2::new(
            &t + (&t / (Real::from(2) * &speed)).unwrap(),
            q(1, 2) - (q(1, 4) / speed).unwrap(),
        )
        .into();
        assert!(curves[1].start().coincides_with(&expected).unwrap());
        assert!(curves[0].end().coincides_with(&curves[1].start()).unwrap());
        let CurvePointLocations2::Locations(locations) =
            curves[1].point_locations(&expected).unwrap()
        else {
            panic!("incident endpoint remains queryable")
        };
        assert_eq!(locations.len(), 1);
        assert_eq!(document(replay.paths()[0].clone()), encoded);
    }
}

#[test]
fn shared_chord_dependencies_stay_compact_and_byte_exhaustion_recovers() {
    use hypercurve::{
        SvgOptions, export_svg_document_with_options, import_svg_document_with_options,
    };
    let beta = selected();
    let chord = Curve2::try_line(at(&beta, 0), at(&beta, 2)).unwrap();
    let build = |count: usize| {
        CurvePath2::try_new(
            (0..count)
                .map(|i| {
                    if i.is_multiple_of(2) {
                        chord.clone()
                    } else {
                        chord.reversed().unwrap()
                    }
                })
                .collect(),
        )
        .unwrap()
    };
    let texts = [document(build(4)), document(build(8)), document(build(16))];
    let metadata_len = |text: &str| {
        text.split_once("data-hypercurve-path=\"")
            .unwrap()
            .1
            .split('"')
            .next()
            .unwrap()
            .len()
    };
    let sizes = texts.each_ref().map(|text| metadata_len(text));
    assert_eq!(sizes[2] - sizes[1], 2 * (sizes[1] - sizes[0]));
    assert!(
        sizes[1] - sizes[0] < 128 * 4,
        "additional pieces must reference shared definitions"
    );
    let replay = import_svg_document(&texts[2]).unwrap();
    let root = |point: CurvePoint2| {
        let CurvePointDefinition2::Rational { parameter, .. } = point.definition().unwrap() else {
            panic!("native rational selected endpoint")
        };
        let BezierParameter2::Algebraic(root) = parameter.as_bezier_parameter().unwrap() else {
            panic!("selected root")
        };
        root.clone()
    };
    let first = root(replay.paths()[0].curves()[0].start());
    let last = root(replay.paths()[0].curves()[15].end());
    assert!(std::ptr::eq(first.polynomial(), last.polynomial()));
    let small = SvgOptions {
        max_extension_bytes: 64,
        ..SvgOptions::default()
    };
    let geometry = SvgGeometry2::new(CurveRegion2::empty(), vec![], vec![build(4)]);
    assert!(matches!(
        export_svg_document_with_options(&geometry, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    assert!(matches!(
        import_svg_document_with_options(&texts[0], small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let repeated = import_svg_document(&export_svg_document(&geometry).unwrap()).unwrap();
    assert!(
        repeated.paths()[0]
            .start()
            .coincides_with(&at(&beta, 0))
            .unwrap()
    );
}

#[test]
fn selected_point_records_reject_wrong_references_and_original_denominator_poles() {
    let mut endpoint = vec![13];
    endpoint.extend_from_slice(&0_u32.to_be_bytes());
    endpoint.push(1);
    let (input, _) = wire(vec![wire_point(0, 0), endpoint], &[]);
    assert!(matches!(
        import_svg_document(&input),
        Err(SvgError::MalformedInput(_))
    ));
    let mut source = vec![0, 5];
    source.extend_from_slice(&2_u32.to_be_bytes());
    for value in [0, 0, 1, 1, 0, 1] {
        wire_real(&mut source, value.into());
    }
    let mut parameter = vec![1];
    wire_real(&mut parameter, q(1, 2));
    for (tag, source_reference, parameter_reference) in
        [(8, 1_u32, 1_u32), (8, 3, 1), (8, 0, 0), (10, 0, 1)]
    {
        let mut point = vec![tag];
        point.extend_from_slice(&source_reference.to_be_bytes());
        point.extend_from_slice(&parameter_reference.to_be_bytes());
        let (input, _) = wire(vec![source.clone(), parameter.clone(), point], &[]);
        assert!(matches!(
            import_svg_document(&input),
            Err(SvgError::MalformedInput(_))
        ));
    }
    let mut point = vec![9];
    point.extend_from_slice(&1_u32.to_be_bytes());
    for value in [1, 1, 0] {
        point.extend_from_slice(&1_u32.to_be_bytes());
        wire_real(&mut point, value.into());
    }
    let (input, _) = wire(vec![source, parameter, point], &[]);
    assert!(matches!(
        import_svg_document(&input),
        Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
    ));
}

#[test]
fn mixed_replay_preserves_a_hole_and_disconnected_material_through_operations() {
    use hypercurve::{CircularArc2, LineSeg2};
    let x = Real::from(2).sqrt().unwrap();
    let rectangle = |left: i32, right: i32, bottom: i32, top: i32, mixed: bool| {
        let points = [
            Point2::new(&x + Real::from(left), bottom.into()),
            Point2::new(&x + Real::from(right), bottom.into()),
            Point2::new(&x + Real::from(right), top.into()),
            Point2::new(&x + Real::from(left), top.into()),
        ];
        CurvePath2::try_new(
            (0..4)
                .map(|i| {
                    let start = points[i].clone();
                    let end = points[(i + 1) % 4].clone();
                    if mixed && i.is_multiple_of(2) {
                        Curve2::from(LineSeg2::try_new(start, end).unwrap())
                    } else {
                        Curve2::try_line(start.into(), end.into()).unwrap()
                    }
                })
                .collect(),
        )
        .unwrap()
    };
    let outer = rectangle(-3, 3, -3, 3, true);
    let circle_start = Point2::new(&x + Real::one(), 0.into());
    let hole = CurvePath2::try_new(vec![Curve2::from(
        CircularArc2::try_from_center(
            circle_start.clone(),
            circle_start,
            Point2::new(x.clone(), 0.into()),
            true,
        )
        .unwrap(),
    )])
    .unwrap();
    let separate = rectangle(6, 8, -1, 1, false);
    let original = vec![outer, hole, separate];
    let encoded = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        original.clone(),
    ))
    .unwrap();
    let replay = import_svg_document(&encoded).unwrap();
    // Native line/arc strokes are returned as wires by the established SVG
    // API. Reassemble all three exact boundaries without using display chords.
    assert_eq!(replay.paths().len(), 2);
    let [hole_wire] = replay.wires() else {
        panic!("one exact native circular wire")
    };
    let decoded_hole = CurvePath2::try_new(
        hole_wire
            .segments()
            .iter()
            .map(|segment| match segment {
                hypercurve::Segment2::Line(line) => Curve2::from(line.clone()),
                hypercurve::Segment2::Arc(arc) => Curve2::from(arc.clone()),
            })
            .collect(),
    )
    .unwrap();
    let decoded = vec![
        replay.paths()[0].clone(),
        decoded_hole,
        replay.paths()[1].clone(),
    ];
    assert_eq!(decoded.len(), 3);
    for (source, decoded) in original.iter().zip(&decoded) {
        assert_eq!(source.curves().len(), decoded.curves().len());
        for (source, decoded) in source.curves().iter().zip(decoded.curves()) {
            assert!(source.start().coincides_with(&decoded.start()).unwrap());
            assert!(source.end().coincides_with(&decoded.end()).unwrap());
            assert_eq!(source.geometry().is_some(), decoded.geometry().is_some());
        }
    }
    let region = CurveRegion2::try_from_boundary_paths(&decoded, FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (2, 1));
    let mut holes = region
        .boundary_profiles()
        .unwrap()
        .iter()
        .map(|profile| profile.holes().len())
        .collect::<Vec<_>>();
    holes.sort_unstable();
    assert_eq!(holes, [0, 1]);
    for (dx, expected) in [
        (0, RegionPointLocation::Outside),
        (1, RegionPointLocation::Boundary),
        (2, RegionPointLocation::Inside),
        (4, RegionPointLocation::Outside),
        (7, RegionPointLocation::Inside),
    ] {
        assert_eq!(
            region
                .classify_point(&Point2::new(&x + Real::from(dx), 0.into()).into())
                .unwrap(),
            expected
        );
    }
    let reconstructed =
        CurveRegion2::try_from_boundary_paths(&region.boundary_paths().unwrap(), FillRule::NonZero)
            .unwrap();
    let compared = region.boolean_regions(&reconstructed).unwrap();
    assert!(compared.difference().is_empty());
    assert!(compared.xor().is_empty());
    let empty = compared.xor();
    let recovered = empty.boolean_regions(&region).unwrap().union().clone();
    assert_eq!(recovered.loop_role_counts().unwrap(), (2, 1));
    let expanded = recovered
        .offset(q(1, 4), &OffsetCornerStyle2::Round)
        .unwrap();
    assert_eq!(expanded.loop_role_counts().unwrap(), (2, 1));
    for (dx, expected) in [
        (Real::zero(), RegionPointLocation::Outside),
        (q(3, 4), RegionPointLocation::Boundary),
        (q(7, 8), RegionPointLocation::Inside),
    ] {
        assert_eq!(
            expanded
                .classify_point(&Point2::new(&x + dx, 0.into()).into())
                .unwrap(),
            expected
        );
    }
    assert!(
        expanded
            .boolean_regions(&expanded)
            .unwrap()
            .difference()
            .is_empty()
    );
    assert_eq!(
        export_svg_document(&SvgGeometry2::new(CurveRegion2::empty(), vec![], decoded,)).unwrap(),
        encoded
    );
}
