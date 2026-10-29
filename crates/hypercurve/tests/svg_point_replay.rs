#![cfg(feature = "svg")]

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCertainty, CurveCornerMode2, CurveError, CurveFillet2,
    CurveParameter2, CurvePath2, CurvePoint2, CurvePointDefinition2, CurvePointLocations2,
    CurveRegion2, ExactCurveError, Point2, PredicatePolicy, RationalMap2, Real, Similarity2,
    SvgError, SvgGeometry2, SvgOptions, export_svg_document, export_svg_document_with_options,
    import_svg_document, import_svg_document_with_options,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn selected(degree: usize) -> BezierParameter2 {
    let mut c = vec![Real::zero(); degree + 1];
    c[0] = (-1).into();
    c[degree] = 2.into();
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(c).unwrap(),
            BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
        )
        .unwrap(),
    )
}
fn at(beta: &CurveParameter2, x: Real, y: Real) -> CurvePoint2 {
    beta.affine_point(1.into(), x, 0.into(), y).unwrap()
}
fn geometry(path: CurvePath2) -> SvgGeometry2 {
    SvgGeometry2::new(CurveRegion2::empty(), vec![], vec![path])
}
fn document(path: CurvePath2) -> String {
    export_svg_document(&geometry(path)).unwrap()
}
fn fiber() -> CurvePoint2 {
    // alpha^2=2, beta^3=alpha, with alpha>1 and beta>1.
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 0, 1], &[-1]]),
        parameter: BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    (-2).into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
            )
            .unwrap(),
        )
        .into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: q(3, 2),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn translation() -> RationalMap2 {
    RationalMap2::new(
        [polynomial(&[&[1], &[1]]), polynomial(&[&[0, 1]])],
        polynomial(&[&[1]]),
    )
}
fn verify_fillet_line(text: &str) {
    let replay = import_svg_document(text).unwrap();
    let curve = &replay.paths()[0].curves()[0];
    let beta: CurveParameter2 = selected(7).into();
    assert!(
        curve
            .start()
            .coincides_with(&at(&beta, 0.into(), 0.into()))
            .unwrap()
    );
    assert!(
        curve
            .end()
            .coincides_with(&at(&beta, q(3, 4), 0.into()))
            .unwrap()
    );
    assert!(matches!(
        curve.end().definition().unwrap(),
        CurvePointDefinition2::SupportingLineIntersection { .. }
    ));
    let midpoint = at(&beta, q(3, 8), 0.into());
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
    let next =
        import_svg_document(&document(CurvePath2::try_new(vec![first, second]).unwrap())).unwrap();
    assert!(
        next.paths()[0]
            .end()
            .coincides_with(&at(&beta, q(3, 4), 0.into()))
            .unwrap()
    );
    assert_eq!(document(replay.paths()[0].clone()), text);
}
#[test]
fn a_fillet_trimmed_line_replays_in_a_fresh_process_before_further_split() {
    const INPUT: &str = "HYPERCURVE_POINT_SVG_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        verify_fillet_line(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let beta: CurveParameter2 = selected(7).into();
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
    let filleted = solutions.pop().unwrap();
    assert_eq!(filleted.curves().len(), 3);
    assert!(
        filleted.curves()[1]
            .start()
            .coincides_with(&at(&beta, q(3, 4), 0.into()))
            .unwrap()
    );
    assert!(
        filleted.curves()[1]
            .end()
            .coincides_with(&at(&beta, 1.into(), q(1, 4)))
            .unwrap()
    );
    let text = document(CurvePath2::try_new(vec![filleted.curves()[0].clone()]).unwrap());
    let file =
        std::env::temp_dir().join(format!("hypercurve-point-svg-{}.svg", std::process::id()));
    std::fs::write(&file, text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "a_fillet_trimmed_line_replays_in_a_fresh_process_before_further_split",
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
fn polynomial_fibers_maps_and_algebraic_similarities_keep_their_defining_relations() {
    let source = fiber();
    let map = RationalMap2::new(
        [polynomial(&[&[], &[1]]), polynomial(&[&[0, 1]])],
        polynomial(&[&[1, 1]]),
    );
    let inverse = RationalMap2::new(
        [polynomial(&[&[], &[1]]), polynomial(&[&[0, 1]])],
        polynomial(&[&[1, -1]]),
    );
    let mapped = source.rational_image(&map).unwrap();
    let scale = Real::from(2).sqrt().unwrap();
    let transform = Similarity2::try_from_real_affine(
        -scale.clone(),
        0.into(),
        0.into(),
        scale,
        3.into(),
        2.into(),
    )
    .unwrap();
    let image = CurvePointDefinition2::Similarity {
        source: mapped.clone(),
        transform: transform.clone(),
    }
    .into_point()
    .unwrap();
    let path = CurvePath2::try_new(vec![Curve2::try_line(mapped, image).unwrap()]).unwrap();
    let text = document(path);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || import_svg_document(&text));
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
        let replay = outcome.into_unverified().unwrap();
        let curve = &replay.paths()[0].curves()[0];
        let CurvePointDefinition2::RationalImage {
            source: retained,
            map: retained_map,
        } = curve.start().definition().unwrap()
        else {
            panic!("retain rational image")
        };
        assert_eq!(retained_map, map);
        assert!(retained.coordinates().is_none());
        assert!(matches!(
            retained.definition().unwrap(),
            CurvePointDefinition2::PolynomialFiber { .. }
        ));
        for relation in [
            polynomial(&[&[0, 0, 0, 1], &[-1]]),
            polynomial(&[&[-2], &[], &[1]]),
        ] {
            assert_eq!(
                retained
                    .rational_form(relation, polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap(),
                Ordering::Equal
            );
        }
        assert!(
            curve
                .start()
                .rational_image(&inverse)
                .unwrap()
                .coincides_with(&source)
                .unwrap()
        );
        let CurvePointDefinition2::Similarity {
            source: shared,
            transform: replayed_transform,
        } = curve.end().definition().unwrap()
        else {
            panic!("retain similarity")
        };
        assert_eq!(replayed_transform, transform);
        assert_eq!(shared, curve.start());
        assert!(
            curve
                .end()
                .coincides_with(
                    &CurvePointDefinition2::Similarity {
                        source: curve.start(),
                        transform: transform.clone()
                    }
                    .into_point()
                    .unwrap()
                )
                .unwrap()
        );
        let reversed = curve.reversed().unwrap();
        assert!(reversed.end().coincides_with(&curve.start()).unwrap());
        assert_eq!(document(replay.paths()[0].clone()), text);
    }
}

#[test]
fn intersections_keep_independent_selected_supports_and_exterior_parameters() {
    let x: CurveParameter2 = selected(7).into();
    let y: CurveParameter2 = selected(5).into();
    let vertical = [at(&x, 0.into(), 2.into()), at(&x, 0.into(), 3.into())];
    let horizontal = [
        y.affine_point(0.into(), 2.into(), 1.into(), 0.into())
            .unwrap(),
        y.affine_point(0.into(), 3.into(), 1.into(), 0.into())
            .unwrap(),
    ];
    let point = CurvePointDefinition2::SupportingLineIntersection {
        first: vertical.clone(),
        second: horizontal.clone(),
    }
    .into_point()
    .unwrap();
    let path =
        CurvePath2::try_new(vec![Curve2::try_line(point, vertical[0].clone()).unwrap()]).unwrap();
    let text = document(path);
    let replay = import_svg_document(&text).unwrap();
    let actual = replay.paths()[0].start();
    assert!(actual.coordinates().is_none());
    for (a, b, expected) in [(1, 0, &x), (0, 1, &y)] {
        assert_eq!(
            actual
                .affine_form(a.into(), b.into(), 0.into())
                .parameter()
                .unwrap()
                .compare(expected)
                .unwrap(),
            Ordering::Equal
        );
    }
    let CurvePointDefinition2::SupportingLineIntersection { first, second } =
        actual.definition().unwrap()
    else {
        panic!("retain both supports")
    };
    for (a, b) in first
        .iter()
        .chain(&second)
        .zip(vertical.iter().chain(&horizontal))
    {
        assert!(a.coincides_with(b).unwrap());
    }
    assert_eq!(document(replay.paths()[0].clone()), text);
}

fn wire_real(bytes: &mut Vec<u8>, value: Real) {
    let value = value.to_bytes();
    bytes.extend_from_slice(&u32::try_from(value.len()).unwrap().to_be_bytes());
    bytes.extend(value);
}
fn wire_polynomial(bytes: &mut Vec<u8>, polynomial: &BivariatePolynomial) {
    bytes.extend_from_slice(
        &u32::try_from(polynomial.coefficients.len())
            .unwrap()
            .to_be_bytes(),
    );
    for row in &polynomial.coefficients {
        bytes.extend_from_slice(&u32::try_from(row.len()).unwrap().to_be_bytes());
        for value in row {
            wire_real(bytes, value.clone());
        }
    }
}
fn wire_point(x: i32, y: i32) -> Vec<u8> {
    let mut bytes = vec![7];
    wire_real(&mut bytes, x.into());
    wire_real(&mut bytes, y.into());
    bytes
}
fn wire_parameter(value: i32) -> Vec<u8> {
    let mut bytes = vec![1];
    wire_real(&mut bytes, value.into());
    bytes
}
fn wire_fiber(
    parameter: u32,
    equation: &BivariatePolynomial,
    lower: Real,
    upper: Real,
    exact: Option<Real>,
) -> Vec<u8> {
    let mut bytes = vec![14];
    bytes.extend_from_slice(&parameter.to_be_bytes());
    wire_polynomial(&mut bytes, equation);
    wire_real(&mut bytes, lower);
    wire_real(&mut bytes, upper);
    bytes.push(u8::from(exact.is_some()));
    if let Some(exact) = exact {
        wire_real(&mut bytes, exact);
    }
    bytes
}
fn wire_map(source: u32, map: &RationalMap2) -> Vec<u8> {
    let mut bytes = vec![15];
    bytes.extend_from_slice(&source.to_be_bytes());
    for p in map.numerators().iter().chain([map.denominator()]) {
        wire_polynomial(&mut bytes, p);
    }
    bytes
}
fn wire_similarity(source: u32, matrix: [i32; 6]) -> Vec<u8> {
    let mut bytes = vec![16];
    bytes.extend_from_slice(&source.to_be_bytes());
    for value in matrix {
        wire_real(&mut bytes, value.into());
    }
    bytes
}
fn wire_intersection(references: [u32; 4]) -> Vec<u8> {
    let mut bytes = vec![17];
    for r in references {
        bytes.extend_from_slice(&r.to_be_bytes());
    }
    bytes
}
fn wire_support(start: u32, end: u32) -> Vec<u8> {
    let mut bytes = vec![11];
    for r in [start, end] {
        bytes.extend_from_slice(&r.to_be_bytes());
    }
    bytes.extend([0, 1]);
    bytes
}
fn wire_range(support: u32, start: u32, end: u32) -> Vec<u8> {
    let mut bytes = vec![12];
    for r in [support, start, end] {
        bytes.extend_from_slice(&r.to_be_bytes());
    }
    bytes.extend([0, 1]);
    bytes
}
fn wire_endpoint(source: u32) -> Vec<u8> {
    let mut bytes = vec![13];
    bytes.extend_from_slice(&source.to_be_bytes());
    bytes.push(1);
    bytes
}
fn wire(nodes: Vec<Vec<u8>>, paths: &[u32]) -> String {
    use std::fmt::Write;
    let mut bytes = b"HCP4".to_vec();
    bytes.extend_from_slice(&u32::try_from(nodes.len()).unwrap().to_be_bytes());
    bytes.extend(nodes.into_iter().flatten());
    bytes.extend_from_slice(&u32::try_from(paths.len()).unwrap().to_be_bytes());
    for p in paths {
        bytes.extend_from_slice(&p.to_be_bytes());
    }
    let mut metadata = String::from("4:");
    for byte in bytes {
        write!(&mut metadata, "{byte:02x}").unwrap();
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{metadata}" d="M0 0 L1 1"/></svg>"#
    )
}
fn invalid(text: &str) {
    assert!(
        matches!(
            import_svg_document(text),
            Err(SvgError::Exact(ExactCurveError::Invalid { .. }))
        ),
        "a malformed construction must not become empty success"
    );
}
#[test]
fn new_point_records_reject_wrong_typed_forward_and_self_references() {
    let equation = polynomial(&[&[-2, 0, 1]]);
    let cases = vec![
        vec![
            wire_point(0, 0),
            wire_fiber(0, &equation, 1.into(), 2.into(), None),
        ],
        vec![wire_fiber(0, &equation, 1.into(), 2.into(), None)],
        vec![wire_parameter(0), wire_map(0, &translation())],
        vec![wire_map(1, &translation()), wire_point(0, 0)],
        vec![wire_parameter(0), wire_similarity(0, [1, 0, 0, 1, 0, 0])],
        vec![wire_similarity(0, [1, 0, 0, 1, 0, 0])],
        vec![
            wire_point(0, 0),
            wire_point(1, 0),
            wire_parameter(0),
            wire_intersection([0, 1, 0, 2]),
        ],
        vec![
            wire_point(0, 0),
            wire_point(1, 0),
            wire_intersection([0, 1, 0, 3]),
        ],
    ];
    for nodes in cases {
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    for matrix in [[2, 0, 0, 1, 0, 0], [1, 1, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0]] {
        invalid(&wire(
            vec![wire_point(0, 0), wire_similarity(0, matrix)],
            &[],
        ));
    }
    for references in [[0, 0, 2, 3], [0, 1, 0, 1], [0, 1, 2, 3]] {
        invalid(&wire(
            vec![
                wire_point(0, 0),
                wire_point(1, 0),
                wire_point(0, 1),
                wire_point(1, 1),
                wire_intersection(references),
            ],
            &[],
        ));
    }
}
#[test]
fn polynomial_fiber_replay_proves_the_selector_and_exact_root_witness() {
    let equation = polynomial(&[&[-2, 0, 1]]);
    for (lower, upper, exact) in [
        ((-2).into(), 2.into(), None),
        (2.into(), 3.into(), None),
        (1.into(), 2.into(), Some(1.into())),
        (2.into(), 1.into(), None),
        (0.into(), 1.into(), Some(1.into())),
    ] {
        invalid(&wire(
            vec![
                wire_parameter(0),
                wire_fiber(0, &equation, lower, upper, exact),
            ],
            &[],
        ));
    }
    let input = wire(
        vec![
            wire_parameter(0),
            wire_fiber(0, &equation, 1.into(), 2.into(), None),
            wire_point(1, 2),
            wire_support(1, 2),
            wire_range(3, 1, 2),
        ],
        &[4],
    );
    let replay = import_svg_document(&input).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&Point2::new(0.into(), Real::from(2).sqrt().unwrap()).into())
            .unwrap()
    );
    let exact = wire(
        vec![
            wire_parameter(0),
            wire_fiber(
                0,
                &polynomial(&[&[-1, 1]]),
                1.into(),
                1.into(),
                Some(1.into()),
            ),
            wire_point(1, 2),
            wire_support(1, 2),
            wire_range(3, 1, 2),
        ],
        &[4],
    );
    invalid(&wire(
        vec![
            wire_parameter(0),
            wire_fiber(
                0,
                &polynomial(&[&[-1, 1]]),
                0.into(),
                1.into(),
                Some(1.into()),
            ),
        ],
        &[],
    ));
    let replay = import_svg_document(&exact).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&Point2::from_values(0, 1).into())
            .unwrap()
    );
}
#[test]
fn original_rational_map_poles_are_checked_on_the_selected_fiber() {
    let equation = polynomial(&[&[-2, 0, 1]]);
    let pole = equation.clone();
    let removable = RationalMap2::new([pole.clone(), pole.clone()], pole);
    let zero = RationalMap2::new(
        [polynomial(&[&[1]]), polynomial(&[&[1]])],
        polynomial(&[&[0]]),
    );
    for map in [removable, zero] {
        let text = wire(
            vec![
                wire_parameter(0),
                wire_fiber(0, &equation, 1.into(), 2.into(), None),
                wire_map(1, &map),
            ],
            &[],
        );
        assert!(matches!(
            import_svg_document(&text),
            Err(SvgError::Exact(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            }))
        ));
    }
    let text = wire(
        vec![
            wire_parameter(0),
            wire_fiber(0, &equation, 1.into(), 2.into(), None),
            wire_map(1, &translation()),
            wire_point(2, 2),
            wire_support(2, 3),
            wire_range(4, 2, 3),
        ],
        &[5],
    );
    let replay = import_svg_document(&text).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&Point2::new(1.into(), Real::from(2).sqrt().unwrap()).into())
            .unwrap()
    );
}
#[test]
fn point_depth_limits_cover_endpoint_indirection_and_retry_without_cached_failure() {
    let input = wire(
        vec![
            wire_point(0, 0),
            wire_similarity(0, [1, 0, 0, 1, 1, 0]),
            wire_point(2, 0),
            wire_support(1, 2),
            wire_range(3, 1, 2),
            wire_endpoint(4),
            wire_similarity(5, [1, 0, 0, 1, 1, 0]),
        ],
        &[4],
    );
    let one = SvgOptions {
        max_point_definition_depth: 1,
        ..SvgOptions::default()
    };
    assert!(matches!(
        import_svg_document_with_options(&input, one),
        Err(SvgError::SizeOverflow {
            limit: "configured exact point-definition depth"
        })
    ));
    assert!(
        import_svg_document_with_options(
            &input,
            SvgOptions {
                max_point_definition_depth: 2,
                ..one
            }
        )
        .is_ok()
    );
    let mut nodes = vec![wire_point(0, 0)];
    for i in 0..300 {
        nodes.push(wire_similarity(i, [1, 0, 0, 1, 1, 0]));
    }
    nodes.extend([
        wire_point(301, 0),
        wire_support(300, 301),
        wire_range(302, 300, 301),
    ]);
    let deep = wire(nodes, &[303]);
    assert!(matches!(
        import_svg_document(&deep),
        Err(SvgError::SizeOverflow {
            limit: "configured exact point-definition depth"
        })
    ));
    assert!(
        import_svg_document_with_options(
            &deep,
            SvgOptions {
                max_point_definition_depth: 300,
                ..one
            }
        )
        .is_ok()
    );
    let mut point = fiber();
    for _ in 0..8 {
        point = point.rational_image(&translation()).unwrap();
    }
    let end = point.rational_image(&translation()).unwrap();
    let model = geometry(CurvePath2::try_new(vec![Curve2::try_line(point, end).unwrap()]).unwrap());
    assert!(matches!(
        export_svg_document_with_options(&model, one),
        Err(SvgError::SizeOverflow {
            limit: "configured exact point-definition depth"
        })
    ));
    let larger = SvgOptions {
        max_point_definition_depth: 9,
        ..one
    };
    let encoded = export_svg_document_with_options(&model, larger).unwrap();
    assert_eq!(
        export_svg_document_with_options(
            &import_svg_document_with_options(&encoded, larger).unwrap(),
            larger
        )
        .unwrap(),
        encoded
    );
    let leaf = wire(
        vec![
            wire_point(0, 0),
            wire_point(1, 0),
            wire_support(0, 1),
            wire_range(2, 0, 1),
        ],
        &[3],
    );
    assert!(
        import_svg_document_with_options(
            &leaf,
            SvgOptions {
                max_point_definition_depth: 0,
                ..one
            }
        )
        .is_ok()
    );
}
fn record_count(text: &str) -> usize {
    let metadata = text
        .split("data-hypercurve-path=\"")
        .nth(1)
        .unwrap()
        .split('"')
        .next()
        .unwrap();
    assert!(metadata.starts_with("4:"));
    usize::from_str_radix(&metadata[10..18], 16).unwrap()
}
#[test]
fn shared_rational_map_prefixes_stay_linear_and_byte_exhaustion_recovers() {
    let mut counts = Vec::new();
    for n in [4usize, 8, 16] {
        let mut point = fiber();
        let mut curves = Vec::new();
        for _ in 0..n {
            let next = point.rational_image(&translation()).unwrap();
            curves.push(Curve2::try_line(point, next.clone()).unwrap());
            point = next;
        }
        let model = geometry(CurvePath2::try_new(curves).unwrap());
        let text = export_svg_document(&model).unwrap();
        let count = record_count(&text);
        counts.push(count);
        println!(
            "{n} mapped spans: {count} definition records, {} bytes",
            text.len()
        );
        assert!(
            count <= 4 * n + 8,
            "shared prefixes must not expand into repeated definition trees"
        );
        assert_eq!(
            export_svg_document(&import_svg_document(&text).unwrap()).unwrap(),
            text
        );
        let small = SvgOptions {
            max_extension_bytes: 64,
            ..SvgOptions::default()
        };
        assert!(matches!(
            export_svg_document_with_options(&model, small),
            Err(SvgError::SizeOverflow { .. })
        ));
        assert!(matches!(
            import_svg_document_with_options(&text, small),
            Err(SvgError::SizeOverflow { .. })
        ));
        assert_eq!(export_svg_document(&model).unwrap(), text);
        assert!(import_svg_document(&text).is_ok());
    }
    assert!(counts[1] <= 2 * counts[0] + 8 && counts[2] <= 2 * counts[1] + 8);
}

#[test]
fn chord_displacements_preserve_direction_origin_translation_and_source() {
    let beta: CurveParameter2 = selected(7).into();
    let first = at(&beta, 0.into(), 0.into());
    let last = at(&beta, 3.into(), 4.into());
    let forward = Curve2::try_line(first, last).unwrap();
    for reversed in [false, true] {
        let source = if reversed {
            forward.reversed().unwrap()
        } else {
            forward.clone()
        };
        for left_normal in [false, true] {
            for distance in [q(5, 2), q(-5, 2), Real::zero()] {
                for explicit_origin in [false, true] {
                    let origin = explicit_origin.then(|| at(&beta, (-2).into(), 7.into()));
                    let mut points = Vec::new();
                    let mut expected = Vec::new();
                    for at_end in [false, true] {
                        let (ox, oy) = if explicit_origin {
                            (-2, 7)
                        } else if at_end != reversed {
                            (3, 4)
                        } else {
                            (0, 0)
                        };
                        let (dx, dy) = if left_normal { (-4, 3) } else { (3, 4) };
                        let sign = if reversed { -1 } else { 1 };
                        let x = Real::from(ox) + &distance * q(sign * dx, 5) + q(7, 3);
                        let y = Real::from(oy) + &distance * q(sign * dy, 5) - q(1, 2);
                        expected.push(at(&beta, x, y));
                        points.push(
                            CurvePointDefinition2::ChordDisplacement {
                                source: source.clone(),
                                origin: origin.clone(),
                                at_end,
                                distance: distance.clone(),
                                translation: [q(7, 3), q(-1, 2)],
                                left_normal,
                            }
                            .into_point()
                            .unwrap(),
                        );
                    }
                    // An arbitrary origin intentionally makes both endpoint
                    // selections coincide; use a distinct fixed endpoint.
                    if explicit_origin {
                        points[1] = at(&beta, 20.into(), 20.into());
                        expected[1] = points[1].clone();
                    }
                    let text = document(
                        CurvePath2::try_new(vec![
                            Curve2::try_line(points[0].clone(), points[1].clone()).unwrap(),
                        ])
                        .unwrap(),
                    );
                    let replay = import_svg_document(&text).unwrap();
                    let curve = &replay.paths()[0].curves()[0];
                    for (point, expected) in [curve.start(), curve.end()].iter().zip(&expected) {
                        assert!(point.coincides_with(expected).unwrap());
                    }
                    let CurvePointDefinition2::ChordDisplacement {
                        source: retained,
                        origin: retained_origin,
                        distance: retained_distance,
                        translation,
                        left_normal: retained_normal,
                        ..
                    } = curve.start().definition().unwrap()
                    else {
                        panic!("retain displacement definition")
                    };
                    assert!(retained.start().coincides_with(&source.start()).unwrap());
                    assert!(retained.end().coincides_with(&source.end()).unwrap());
                    assert_eq!(retained_distance, distance);
                    assert_eq!(translation, [q(7, 3), q(-1, 2)]);
                    assert_eq!(retained_normal, left_normal);
                    assert_eq!(retained_origin.is_some(), explicit_origin);
                    if let (Some(a), Some(b)) = (retained_origin, &origin) {
                        assert!(a.coincides_with(b).unwrap());
                    }
                    assert_eq!(document(replay.paths()[0].clone()), text);
                }
            }
        }
    }
}

fn wire_displacement(source: u32, origin: Option<u32>, left_normal: bool) -> Vec<u8> {
    let mut bytes = vec![18];
    bytes.extend_from_slice(&source.to_be_bytes());
    bytes.push(u8::from(origin.is_some()));
    if let Some(origin) = origin {
        bytes.extend_from_slice(&origin.to_be_bytes());
    }
    for value in [q(5, 2), q(7, 3), q(-1, 2)] {
        wire_real(&mut bytes, value);
    }
    bytes.push(u8::from(left_normal));
    bytes
}
fn wire_displaced_point(recipe: u32, at_end: bool) -> Vec<u8> {
    let mut bytes = vec![19];
    bytes.extend_from_slice(&recipe.to_be_bytes());
    bytes.push(u8::from(at_end));
    bytes
}

#[test]
fn displacement_recipes_validate_references_and_share_endpoint_selections() {
    let nodes = vec![
        wire_point(0, 0),
        wire_point(3, 4),
        wire_support(0, 1),
        wire_range(2, 0, 1),
    ];
    for invalid_node in [
        wire_displacement(0, None, true),
        wire_displacement(4, None, true),
        wire_displacement(3, Some(2), true),
        wire_displacement(3, Some(4), true),
        wire_displaced_point(3, false),
        wire_displaced_point(4, false),
    ] {
        let mut nodes = nodes.clone();
        nodes.push(invalid_node);
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    let mut nodes = nodes;
    nodes.extend([
        wire_displacement(3, None, true),
        wire_displaced_point(4, false),
        wire_displaced_point(4, true),
        wire_support(5, 6),
        wire_range(7, 5, 6),
    ]);
    let input = wire(nodes, &[8]);
    let zero = SvgOptions {
        max_point_definition_depth: 0,
        ..SvgOptions::default()
    };
    assert!(matches!(
        import_svg_document_with_options(&input, zero),
        Err(SvgError::SizeOverflow {
            limit: "configured exact point-definition depth"
        })
    ));
    let model = import_svg_document(&input).unwrap();
    let curve = &model.paths()[0].curves()[0];
    for (point, (x, y)) in [curve.start(), curve.end()]
        .iter()
        .zip([(q(1, 3), 1.into()), (q(10, 3), 5.into())])
    {
        assert!(point.coincides_with(&Point2::new(x, y).into()).unwrap());
    }
    let text = export_svg_document(&model).unwrap();
    assert_eq!(
        record_count(&text),
        9,
        "one source range and displacement recipe serve both endpoints"
    );
    assert_eq!(
        export_svg_document(&import_svg_document(&text).unwrap()).unwrap(),
        text
    );
}

#[test]
fn rational_query_pair_records_replay_both_sources_and_original_guards() {
    let pair = |references: [u32; 2], second_denominator: BivariatePolynomial| {
        let mut bytes = vec![26];
        for (reference, denominator) in references
            .into_iter()
            .zip([polynomial(&[&[1]]), second_denominator])
        {
            bytes.extend_from_slice(&reference.to_be_bytes());
            wire_polynomial(&mut bytes, &polynomial(&[&[0, 1]]));
            wire_polynomial(&mut bytes, &denominator);
        }
        bytes
    };
    for nodes in [
        vec![wire_parameter(0), pair([0, 0], polynomial(&[&[1]]))],
        vec![wire_point(0, 0), pair([0, 1], polynomial(&[&[1]]))],
        vec![
            wire_point(0, 0),
            pair([0, 2], polynomial(&[&[1]])),
            wire_point(1, 1),
        ],
    ] {
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    invalid(&wire(
        vec![wire_point(0, 2), pair([0, 0], polynomial(&[&[-2, 1]]))],
        &[],
    ));
    // A valid unused record is still admitted before the nonempty path.
    let text = wire(
        vec![
            wire_point(0, 2),
            pair([0, 0], polynomial(&[&[1]])),
            wire_point(3, 2),
            wire_support(0, 2),
            wire_range(3, 0, 2),
        ],
        &[4],
    );
    assert_eq!(import_svg_document(&text).unwrap().paths().len(), 1);
}

#[test]
fn query_parameter_records_validate_references_guards_and_depth_before_use() {
    let query = |reference: u32, denominator: BivariatePolynomial| {
        let mut bytes = vec![27];
        bytes.extend_from_slice(&reference.to_be_bytes());
        wire_polynomial(&mut bytes, &polynomial(&[&[0, 1]]));
        wire_polynomial(&mut bytes, &denominator);
        bytes
    };
    for nodes in [
        vec![wire_parameter(0), query(0, polynomial(&[&[1]]))],
        vec![query(0, polynomial(&[&[1]]))],
        vec![query(1, polynomial(&[&[1]])), wire_point(0, 2)],
    ] {
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    invalid(&wire(
        vec![wire_point(0, 2), query(0, polynomial(&[&[-2, 1]]))],
        &[],
    ));
    let text = wire(
        vec![
            wire_point(0, 2),
            query(0, polynomial(&[&[1]])),
            wire_point(3, 2),
            wire_support(0, 2),
            wire_range(3, 0, 2),
        ],
        &[4],
    );
    let small = SvgOptions {
        max_point_definition_depth: 0,
        ..SvgOptions::default()
    };
    assert!(matches!(
        import_svg_document_with_options(&text, small),
        Err(SvgError::SizeOverflow {
            limit: "configured exact point-definition depth"
        })
    ));
    assert_eq!(import_svg_document(&text).unwrap().paths().len(), 1);
}

#[test]
fn query_parameter_over_original_fiber_survives_reexport() {
    let mut query = vec![27];
    query.extend_from_slice(&1_u32.to_be_bytes());
    wire_polynomial(&mut query, &polynomial(&[&[0, 1]]));
    wire_polynomial(&mut query, &polynomial(&[&[2]]));
    let mut curve = vec![0, 2];
    for coordinate in [
        Real::zero(),
        Real::zero(),
        q(1, 2),
        Real::zero(),
        Real::one(),
        Real::one(),
    ] {
        wire_real(&mut curve, coordinate);
    }
    let mut restriction = vec![4];
    for reference in [3_u32, 4, 2] {
        restriction.extend_from_slice(&reference.to_be_bytes());
    }
    restriction.push(0);
    let text = wire(
        vec![
            wire_parameter(2),
            wire_fiber(
                0,
                &polynomial(&[&[0, 0, 0, 1], &[-1]]),
                1.into(),
                2.into(),
                None,
            ),
            query,
            curve,
            wire_parameter(0),
            restriction,
        ],
        &[5],
    );
    let imported = import_svg_document(&text).expect("valid source query record imports");
    let endpoint = imported.paths()[0].end();
    assert_eq!(
        endpoint
            .rational_form(polynomial(&[&[0, -1], &[], &[1]]), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        endpoint
            .rational_form(polynomial(&[&[0], &[], &[], &[4]]), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::one())
            .unwrap(),
        Ordering::Equal
    );
    let saved =
        export_svg_document(&imported).expect("an admitted query parameter must remain exportable");
    let replay = import_svg_document(&saved).unwrap();
    assert!(replay.paths()[0].end().coincides_with(&endpoint).unwrap());
    let curve = &replay.paths()[0].curves()[0];
    let middle = curve
        .parameter_domain()
        .end()
        .affine_image(&q(1, 2), &Real::zero())
        .unwrap();
    let (first, second) = curve.split_at(middle).unwrap();
    assert!(first.end().coincides_with(&second.start()).unwrap());
}

#[test]
fn selected_coefficient_root_records_validate_source_guards_and_selection() {
    let root = |reference: u32, denominator: BivariatePolynomial, lower: Real, upper: Real| {
        let mut bytes = vec![28];
        bytes.extend_from_slice(&3_u32.to_be_bytes());
        for numerator in [
            polynomial(&[&[-2]]),
            polynomial(&[&[0]]),
            polynomial(&[&[1]]),
        ] {
            bytes.extend_from_slice(&reference.to_be_bytes());
            wire_polynomial(&mut bytes, &numerator);
            wire_polynomial(&mut bytes, &denominator);
        }
        wire_real(&mut bytes, lower);
        wire_real(&mut bytes, upper);
        bytes.push(0);
        bytes
    };
    for nodes in [
        vec![root(0, polynomial(&[&[1]]), 1.into(), 2.into())],
        vec![
            wire_parameter(0),
            root(0, polynomial(&[&[1]]), 1.into(), 2.into()),
        ],
        vec![
            root(1, polynomial(&[&[1]]), 1.into(), 2.into()),
            wire_point(0, 0),
        ],
    ] {
        assert!(matches!(
            import_svg_document(&wire(nodes, &[])),
            Err(SvgError::MalformedInput(_))
        ));
    }
    for (denominator, lower, upper) in [
        (polynomial(&[&[0]]), Real::one(), Real::from(2)),
        (polynomial(&[&[1]]), Real::from(2), Real::from(3)),
        (polynomial(&[&[1]]), Real::from(-2), Real::from(2)),
    ] {
        // Even unused definitions must be checked; invalid roots are not empty geometry.
        invalid(&wire(
            vec![wire_point(0, 0), root(0, denominator, lower, upper)],
            &[],
        ));
    }
    let text = wire(
        vec![
            wire_point(0, 0),
            root(0, polynomial(&[&[1]]), 1.into(), 2.into()),
            wire_point(3, 0),
            wire_support(1, 2),
            wire_range(3, 1, 2),
        ],
        &[4],
    );
    let geometry = import_svg_document(&text).unwrap();
    let point = geometry.paths()[0].curves()[0].start();
    assert_eq!(
        point
            .rational_form(polynomial(&[&[-2], &[], &[1]]), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        point
            .affine_form(1.into(), 0.into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Greater
    );
}
