#![cfg(feature = "svg")]

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BooleanOp, Curve2, CurveCornerMode2, CurveFamily2, CurveFillet2,
    CurveParameter2, CurvePath2, CurvePoint2, CurvePointDefinition2, CurvePointLocations2,
    CurveRegion2, ExactCurveError, FillRule, Point2, Real, RegionPointLocation, Similarity2,
    SvgError, SvgGeometry2, SvgOptions, export_svg_document, export_svg_document_with_options,
    import_svg_document, import_svg_document_with_options,
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
            BezierParameterInterval::try_new(q(3, 4), 1.into()).unwrap(),
        )
        .unwrap(),
    )
    .into()
}
fn at(beta: &CurveParameter2, x: Real, y: Real) -> CurvePoint2 {
    beta.affine_point(1.into(), x, 0.into(), y).unwrap()
}
fn encode(path: CurvePath2) -> String {
    export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![path],
    ))
    .unwrap()
}
fn fillet() -> CurvePath2 {
    let beta = selected();
    let path = CurvePath2::try_new(vec![
        Curve2::try_line(at(&beta, 0.into(), 0.into()), at(&beta, 1.into(), 0.into())).unwrap(),
        Curve2::try_line(at(&beta, 1.into(), 0.into()), at(&beta, 1.into(), 1.into())).unwrap(),
    ])
    .unwrap();
    let mut solutions = path
        .fillet_vertex(1, &CurveFillet2::new(q(1, 4)), CurveCornerMode2::TrimOnly)
        .unwrap()
        .into_solutions();
    assert_eq!(solutions.len(), 1);
    solutions.pop().unwrap()
}
fn closed_fillet() -> CurvePath2 {
    let beta = selected();
    let mut pieces = fillet().curves().to_vec();
    pieces.extend([
        Curve2::try_line(at(&beta, 1.into(), 1.into()), at(&beta, 0.into(), 1.into())).unwrap(),
        Curve2::try_line(at(&beta, 0.into(), 1.into()), at(&beta, 0.into(), 0.into())).unwrap(),
    ]);
    CurvePath2::try_new(pieces).unwrap()
}
fn verify(text: &str) {
    let replay = import_svg_document(text).unwrap();
    let path = &replay.paths()[0];
    assert_eq!(path.curves().len(), 3);
    let circle = &path.curves()[1];
    assert_eq!(circle.family(), CurveFamily2::CircularArc);
    assert!(circle.geometry().is_none());
    let beta = selected();
    assert!(
        circle
            .start()
            .coincides_with(&at(&beta, q(3, 4), 0.into()))
            .unwrap()
    );
    assert!(
        circle
            .end()
            .coincides_with(&at(&beta, 1.into(), q(1, 4)))
            .unwrap()
    );
    // C=(beta+3/4,1/4), r=1/4. The rational chart at u=1/4
    // has radial components (3/5,-4/5), so this point is exact.
    let interior = at(&beta, q(9, 10), q(1, 20));
    let CurvePointLocations2::Locations(locations) = circle.point_locations(&interior).unwrap()
    else {
        panic!("one selected point on the retained quarter circle")
    };
    assert_eq!(locations.len(), 1);
    let parameter = locations[0].parameter().unwrap().clone();
    assert!(
        circle
            .point_at(&parameter)
            .unwrap()
            .coincides_with(&interior)
            .unwrap()
    );
    let (first, second) = circle.split_at(parameter).unwrap();
    assert!(first.end().coincides_with(&interior).unwrap());
    assert!(second.start().coincides_with(&interior).unwrap());
    let reversed =
        CurvePath2::try_new(vec![second.reversed().unwrap(), first.reversed().unwrap()]).unwrap();
    let second_text = encode(reversed);
    let twice = import_svg_document(&second_text).unwrap();
    assert!(
        twice.paths()[0]
            .start()
            .coincides_with(&circle.end())
            .unwrap()
    );
    assert!(
        twice.paths()[0]
            .end()
            .coincides_with(&circle.start())
            .unwrap()
    );
    assert_eq!(encode(twice.paths()[0].clone()), second_text);
    assert_eq!(encode(path.clone()), text);
}

#[test]
fn selected_circular_fillet_replays_in_a_fresh_process_before_split_and_reversal() {
    const INPUT: &str = "HYPERCURVE_CIRCLE_SVG_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let path = fillet();
    let text = encode(path);
    verify(&text);
    let file =
        std::env::temp_dir().join(format!("hypercurve-circle-svg-{}.svg", std::process::id()));
    std::fs::write(&file, &text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_circular_fillet_replays_in_a_fresh_process_before_split_and_reversal",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output()
        .unwrap();
    std::fs::remove_file(file).unwrap();
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn replayed_fillet_reenters_region_boolean_and_boundary_reconstruction() {
    let beta = selected();
    let text = encode(closed_fillet());
    let replay = import_svg_document(&text).unwrap();
    let region = CurveRegion2::try_from_boundary_paths(replay.paths(), FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (1, 0));
    // Independent coordinates relative to the degree-seven corner. In the
    // rounded corner, (x-3/4)^2+(y-1/4)^2 compares exactly with 1/16.
    for (x, y, expected) in [
        (q(1, 2), q(1, 2), RegionPointLocation::Inside),
        (q(9, 10), q(1, 10), RegionPointLocation::Inside),
        (q(99, 100), q(1, 100), RegionPointLocation::Outside),
        (q(9, 10), q(1, 20), RegionPointLocation::Boundary),
    ] {
        assert_eq!(region.classify_point(&at(&beta, x, y)).unwrap(), expected);
    }
    let empty = CurveRegion2::empty();
    for operation in [BooleanOp::Union, BooleanOp::Difference, BooleanOp::Xor] {
        let result = region.boolean_region(&empty, operation).unwrap();
        assert_eq!(result.loop_role_counts().unwrap(), (1, 0));
        assert!(
            result
                .boolean_region(&region, BooleanOp::Difference)
                .unwrap()
                .is_empty()
        );
        let boundaries = result.boundary_paths().unwrap();
        assert_eq!(boundaries.len(), 1);
        let again = import_svg_document(&encode(boundaries[0].clone())).unwrap();
        let rebuilt =
            CurveRegion2::try_from_boundary_paths(again.paths(), FillRule::NonZero).unwrap();
        assert!(
            rebuilt
                .boolean_region(&region, BooleanOp::Difference)
                .unwrap()
                .is_empty()
        );
        assert!(
            region
                .boolean_region(&rebuilt, BooleanOp::Difference)
                .unwrap()
                .is_empty()
        );
    }
    for operation in [BooleanOp::Intersection, BooleanOp::Difference] {
        assert!(empty.boolean_region(&region, operation).unwrap().is_empty());
    }
    assert!(
        region
            .boolean_region(&empty, BooleanOp::Intersection)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn changed_neighbors_replay_both_crossings_without_inheriting_tangency() {
    let replay = import_svg_document(&encode(fillet())).unwrap();
    let circle = &replay.paths()[0].curves()[1];
    let beta = selected();
    let start = at(&beta, q(3, 4), 0.into());
    // The new line has slope 1/2. Substitution into the original circle
    // equation gives contacts at the original endpoint and (beta+19/20,1/10).
    let interior = at(&beta, q(19, 20), q(1, 10));
    let neighbor = Curve2::try_line(at(&beta, q(7, 4), q(1, 2)), start.clone()).unwrap();
    for (first, second) in [(circle, &neighbor), (&neighbor, circle)] {
        let result = first.intersect_curve(second).unwrap();
        assert!(result.is_complete());
        assert_eq!(result.contacts().len(), 2);
        assert!(result.overlaps().is_empty());
        for expected in [&start, &interior] {
            assert!(
                result
                    .contacts()
                    .iter()
                    .any(|contact| contact.point().coincides_with(expected).unwrap())
            );
        }
        assert!(
            result
                .contacts()
                .iter()
                .all(|contact| contact.is_certified_transverse())
        );
    }
}

#[test]
fn mixed_generated_circle_replay_preserves_holes_and_disconnected_material() {
    let beta = selected();
    let vertices = [
        at(&beta, q(1, 5), q(3, 10)),
        at(&beta, q(2, 5), q(3, 10)),
        at(&beta, q(2, 5), q(1, 2)),
        at(&beta, q(1, 5), q(1, 2)),
    ];
    let hole = CurvePath2::try_new(
        (0..4)
            .map(|i| Curve2::try_line(vertices[i].clone(), vertices[(i + 1) % 4].clone()).unwrap())
            .collect(),
    )
    .unwrap()
    .reversed()
    .unwrap();
    let start = Point2::new(q(9, 2), 0.into());
    let native = CurvePath2::try_new(vec![
        hypercurve::CircularArc2::try_from_center(
            start.clone(),
            start,
            Point2::new(4.into(), 0.into()),
            false,
        )
        .unwrap()
        .into(),
    ])
    .unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![closed_fillet(), hole, native],
    ))
    .unwrap();
    let replay = import_svg_document(&text).unwrap();
    assert_eq!(replay.paths().len(), 2);
    assert_eq!(replay.wires().len(), 1);
    let mut paths = replay.paths().to_vec();
    paths.push(
        CurvePath2::try_new(
            replay.wires()[0]
                .segments()
                .iter()
                .map(|segment| match segment {
                    hypercurve::Segment2::Line(line) => Curve2::from(line.clone()),
                    hypercurve::Segment2::Arc(arc) => Curve2::from(arc.clone()),
                })
                .collect(),
        )
        .unwrap(),
    );
    let region = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (2, 1));
    for (point, expected) in [
        (at(&beta, q(3, 10), q(2, 5)), RegionPointLocation::Outside),
        (at(&beta, q(3, 5), q(1, 2)), RegionPointLocation::Inside),
        (
            Point2::new(4.into(), 0.into()).into(),
            RegionPointLocation::Inside,
        ),
        (
            Point2::new(3.into(), 0.into()).into(),
            RegionPointLocation::Outside,
        ),
    ] {
        assert_eq!(region.classify_point(&point).unwrap(), expected);
    }
    let union = region
        .boolean_region(&CurveRegion2::empty(), BooleanOp::Union)
        .unwrap();
    assert_eq!(union.loop_role_counts().unwrap(), (2, 1));
    assert!(
        union
            .boolean_region(&region, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
    assert!(
        union
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let rebuilt =
        CurveRegion2::try_from_boundary_paths(&union.boundary_paths().unwrap(), FillRule::NonZero)
            .unwrap();
    assert_eq!(rebuilt.loop_role_counts().unwrap(), (2, 1));
}

#[test]
fn algebraic_similarity_and_reflection_keep_selected_circle_replay() {
    let source = fillet();
    let scale = Real::from(2).sqrt().unwrap();
    for transform in [
        Similarity2::try_from_real_affine(
            scale.clone(),
            -scale.clone(),
            scale.clone(),
            scale.clone(),
            3.into(),
            (-2).into(),
        )
        .unwrap(),
        Similarity2::try_from_real_affine(
            -scale.clone(),
            0.into(),
            0.into(),
            scale,
            1.into(),
            4.into(),
        )
        .unwrap(),
    ] {
        let transformed = source.transform_similarity(&transform).unwrap();
        let text = encode(transformed.clone());
        let replay = import_svg_document(&text).unwrap();
        for (before, after) in transformed.curves().iter().zip(replay.paths()[0].curves()) {
            assert_eq!(before.family(), after.family());
            assert!(before.start().coincides_with(&after.start()).unwrap());
            assert!(before.end().coincides_with(&after.end()).unwrap());
        }
        let interior = CurvePointDefinition2::Similarity {
            source: at(&selected(), q(9, 10), q(1, 20)),
            transform,
        }
        .into_point()
        .unwrap();
        let CurvePointLocations2::Locations(locations) = replay.paths()[0].curves()[1]
            .point_locations(&interior)
            .unwrap()
        else {
            panic!("the independently transformed point remains on the selected arc")
        };
        assert_eq!(locations.len(), 1);
        assert_eq!(encode(replay.paths()[0].clone()), text);
    }
}

fn wire_real(bytes: &mut Vec<u8>, value: Real) {
    let value = value.to_bytes();
    bytes.extend_from_slice(&u32::try_from(value.len()).unwrap().to_be_bytes());
    bytes.extend(value);
}
fn record(tag: u8, references: &[u32]) -> Vec<u8> {
    let mut bytes = vec![tag];
    for reference in references {
        bytes.extend_from_slice(&reference.to_be_bytes());
    }
    bytes
}
fn point(x: i32, y: i32) -> Vec<u8> {
    let mut bytes = vec![7];
    wire_real(&mut bytes, x.into());
    wire_real(&mut bytes, y.into());
    bytes
}
fn chord(tag: u8, references: &[u32]) -> Vec<u8> {
    let mut bytes = record(tag, references);
    bytes.extend([0, 1]);
    bytes
}
fn circle(center: u32, anchor: u32, radius: i32) -> Vec<u8> {
    let mut bytes = record(20, &[center, anchor]);
    wire_real(&mut bytes, radius.into());
    bytes.push(1); // clockwise from the left normal, hence the right half circle
    bytes
}
fn parameter(support: u32, value: Real) -> Vec<u8> {
    let mut bytes = record(21, &[support]);
    wire_real(&mut bytes, value);
    bytes
}
fn range(support: u32, start: u32, end: u32) -> Vec<u8> {
    let mut bytes = record(23, &[support, start, end]);
    bytes.push(0);
    bytes
}
fn unit_tangent(source: u32, x: Real, y: Real) -> Vec<u8> {
    let mut bytes = record(25, &[source]);
    wire_real(&mut bytes, x);
    wire_real(&mut bytes, y);
    bytes
}
fn base() -> Vec<Vec<u8>> {
    vec![
        point(0, 0),
        point(1, 0),
        chord(11, &[0, 1]),
        chord(12, &[2, 0, 1]),
        circle(0, 3, 1),
    ]
}
fn wire(nodes: Vec<Vec<u8>>, paths: &[u32]) -> String {
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
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{metadata}" d="M0 0 L1 1"/></svg>"#
    )
}
fn invalid(nodes: Vec<Vec<u8>>) {
    let error = import_svg_document(&wire(nodes, &[])).err();
    assert!(
        matches!(
            error,
            Some(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ),
        "expected a proved invalid construction, received {error:?}"
    );
}

#[test]
fn unused_circle_records_reject_wrong_typed_forward_and_cross_circle_references() {
    for extra in [
        circle(4, 3, 1),
        circle(0, 0, 1),
        circle(5, 3, 1),
        parameter(0, 0.into()),
        parameter(5, 0.into()),
        record(24, &[4, 4]),
        record(24, &[4, 5]),
        record(24, &[0, 1]),
        range(4, 0, 1),
        unit_tangent(0, 1.into(), 0.into()),
        unit_tangent(5, 1.into(), 0.into()),
    ] {
        let mut nodes = base();
        nodes.push(extra);
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    let mut nodes = base();
    nodes.extend([
        circle(0, 3, 1),
        parameter(4, 0.into()),
        parameter(5, 1.into()),
        range(4, 6, 7),
    ]);
    assert!(matches!(
        import_svg_document(&wire(nodes, &[])),
        Err(SvgError::MalformedInput(_))
    ));
}

#[test]
fn chord_unit_direction_replay_checks_length_parallelism_and_orientation() {
    for (x, y) in [(0, 0), (2, 0), (0, 1), (-1, 0)] {
        let mut nodes = base();
        nodes.push(unit_tangent(3, x.into(), y.into()));
        invalid(nodes);
    }
    let nodes = vec![
        point(0, 0),
        point(3, 4),
        chord(11, &[0, 1]),
        unit_tangent(2, q(3, 5), q(4, 5)),
        chord(12, &[3, 0, 1]),
        circle(0, 4, 1),
        parameter(5, 0.into()),
        parameter(5, 1.into()),
        range(5, 6, 7),
    ];
    let replay = import_svg_document(&wire(nodes, &[8])).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&Point2::new(q(-4, 5), q(3, 5)).into())
            .unwrap()
    );
    assert!(
        replay.paths()[0]
            .end()
            .coincides_with(&Point2::new(q(4, 5), q(-3, 5)).into())
            .unwrap()
    );
    let text = encode(replay.paths()[0].clone());
    assert_eq!(
        encode(import_svg_document(&text).unwrap().paths()[0].clone()),
        text
    );
}

#[test]
fn chord_normal_contact_records_reprove_the_displacement_relation() {
    let contact = |source, chord, point, distance| {
        let mut bytes = record(22, &[source, chord, point]);
        wire_real(&mut bytes, distance);
        bytes
    };
    // Unit circle in a horizontal chord frame; a vertical chord's negative
    // normal gives its interior rightmost point (1,0).
    let mut nodes = base();
    nodes.extend([point(0, 1), chord(11, &[0, 5]), chord(12, &[6, 0, 5])]);
    // The vertical support uses Y as its monotone coordinate.
    let length = nodes[6].len();
    nodes[6][length - 2] = 1;
    let length = nodes[7].len();
    nodes[7][length - 2] = 1;
    for (support, point, distance) in [(3, 1, -1), (7, 0, -1), (7, 1, 2), (7, 5, -1), (7, 1, 1)] {
        let mut invalid_nodes = nodes.clone();
        invalid_nodes.push(contact(4, support, point, distance.into()));
        invalid(invalid_nodes);
    }
    nodes.extend([
        contact(4, 7, 1, (-1).into()),
        parameter(4, 0.into()),
        range(4, 9, 8),
    ]);
    let replay = import_svg_document(&wire(nodes, &[10])).unwrap();
    assert!(
        replay.paths()[0]
            .end()
            .coincides_with(&Point2::new(1.into(), 0.into()).into())
            .unwrap()
    );
}

#[test]
fn retained_circle_point_queries_clip_the_arc_and_distinguish_nearby_points() {
    let text = encode(fillet());
    let replay = import_svg_document(&text).unwrap();
    let circle = &replay.paths()[0].curves()[1];
    let beta = selected();
    let epsilon = (0..80).fold(Real::one(), |value, _| (value / Real::from(2)).unwrap());
    for (x, y) in [
        (q(3, 4), q(1, 4)),             // center
        (q(3, 5), q(1, 20)),            // on the support beyond the retained quarter
        (q(3, 4), q(1, 2)),             // opposite side
        (q(9, 10) + epsilon, q(1, 20)), // distinct, despite display resolution
    ] {
        let CurvePointLocations2::Locations(locations) =
            circle.point_locations(&at(&beta, x, y)).unwrap()
        else {
            panic!("a circular arc is not a constant point")
        };
        assert!(locations.is_empty());
    }
    for expected in [at(&beta, q(3, 4), 0.into()), at(&beta, 1.into(), q(1, 4))] {
        let CurvePointLocations2::Locations(locations) = circle.point_locations(&expected).unwrap()
        else {
            panic!("one endpoint location")
        };
        assert_eq!(locations.len(), 1);
        assert!(
            circle
                .point_at(&locations[0].parameter().unwrap())
                .unwrap()
                .coincides_with(&expected)
                .unwrap()
        );
    }
}

#[test]
fn replay_reproves_radius_incidence_selected_half_and_ascending_domain() {
    let mut zero = base();
    zero[4] = circle(0, 3, 0);
    invalid(zero);
    for value in [Real::from(-1), Real::from(2)] {
        let mut nodes = base();
        nodes.push(parameter(4, value));
        invalid(nodes);
    }
    for (start, end) in [(1, 0), (0, 0)] {
        let mut nodes = base();
        nodes.extend([
            parameter(4, start.into()),
            parameter(4, end.into()),
            range(4, 5, 6),
        ]);
        invalid(nodes);
    }
    for (x, y) in [(0, 0), (1, 1), (-1, 0)] {
        let mut nodes = base();
        nodes.extend([point(x, y), record(24, &[4, 5])]);
        invalid(nodes);
    }
    let mut nodes = base();
    // Both valid scalar chart boundaries and an independently supplied radial
    // point must survive without trusting the SVG display polyline.
    nodes.extend([
        parameter(4, 0.into()),
        parameter(4, 1.into()),
        record(24, &[4, 1]),
        range(4, 5, 7),
    ]);
    let replay = import_svg_document(&wire(nodes, &[8])).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&Point2::new(0.into(), 1.into()).into())
            .unwrap()
    );
    assert!(
        replay.paths()[0]
            .end()
            .coincides_with(&Point2::new(1.into(), 0.into()).into())
            .unwrap()
    );
}

#[test]
fn circle_dependency_depth_is_reached_before_publication_and_retry_succeeds() {
    let mut nodes = base();
    nodes.extend([
        parameter(4, 0.into()),
        parameter(4, 1.into()),
        range(4, 5, 6),
    ]);
    let text = wire(nodes, &[7]);
    let small = SvgOptions {
        max_point_definition_depth: 0,
        ..SvgOptions::default()
    };
    let large = SvgOptions {
        max_point_definition_depth: 1,
        ..small
    };
    for _ in 0..2 {
        assert!(matches!(
            import_svg_document_with_options(&text, small),
            Err(SvgError::SizeOverflow {
                limit: "configured exact point-definition depth"
            })
        ));
        let replay = import_svg_document_with_options(&text, large).unwrap();
        assert!(matches!(
            export_svg_document_with_options(&replay, small),
            Err(SvgError::SizeOverflow {
                limit: "configured exact point-definition depth"
            })
        ));
        let exported = export_svg_document_with_options(&replay, large).unwrap();
        let byte_limited = SvgOptions {
            max_extension_bytes: 64,
            ..large
        };
        assert!(matches!(
            import_svg_document_with_options(&exported, byte_limited),
            Err(SvgError::SizeOverflow { .. })
        ));
        assert!(matches!(
            export_svg_document_with_options(&replay, byte_limited),
            Err(SvgError::SizeOverflow { .. })
        ));
        assert_eq!(
            export_svg_document_with_options(
                &import_svg_document_with_options(&exported, large).unwrap(),
                large
            )
            .unwrap(),
            exported
        );
    }
}
