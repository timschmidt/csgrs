#![cfg(feature = "svg")]

use hypercurve::{
    BezierSplitFragment2, BezierSubcurve2, CircularArc2, Classification, CubicBezier2, Curve2,
    CurveContext, CurveFamily2, CurvePath2, LineSeg2, NurbsCurve2, Point2, PolynomialSplineCurve2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real, Segment2, Similarity2,
    SvgError, SvgGeometry2, SvgOptions, export_svg_document, import_svg_document,
    import_svg_document_with_options, parse_svg_path_data,
};

fn rational(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn single_curve_geometry(curve: Curve2) -> SvgGeometry2 {
    SvgGeometry2::new(
        hypercurve::CurveRegion2::empty(),
        Vec::new(),
        vec![CurvePath2::try_new(vec![curve]).unwrap()],
    )
}

fn single_imported_curve(geometry: &SvgGeometry2) -> Curve2 {
    match (geometry.wires(), geometry.paths()) {
        ([wire], []) => match wire.segments() {
            [Segment2::Line(line)] => Curve2::from(line.clone()),
            [Segment2::Arc(arc)] => Curve2::from(arc.clone()),
            segments => panic!("expected one native segment, got {}", segments.len()),
        },
        ([], [path]) => {
            assert_eq!(path.curves().len(), 1);
            path.curves()[0].clone()
        }
        (wires, paths) => panic!(
            "expected one imported curve, got {} wires and {} paths",
            wires.len(),
            paths.len()
        ),
    }
}

fn remove_exact_path_attribute(mut document: String) -> String {
    let marker = " data-hypercurve-path=\"";
    let start = document.find(marker).expect("exact path attribute");
    let value_start = start + marker.len();
    let value_end = value_start
        + document[value_start..]
            .find('"')
            .expect("exact path attribute terminator");
    document.replace_range(start..=value_end, "");
    document
}

#[test]
fn path_parser_covers_every_svg_path_command_family() {
    let paths = parse_svg_path_data(
        "M0 0 l2 0 v2 h-2 z \
         M3 0 C4 0 4 1 5 1 S6 2 7 1 Q8 0 9 1 T11 1",
    )
    .unwrap();

    assert_eq!(paths.len(), 2);
    assert!(paths[0].is_closed());
    assert!(!paths[1].is_closed());
    assert_eq!(paths[0].path().curves().len(), 4);
    assert_eq!(
        paths[1]
            .path()
            .curves()
            .iter()
            .map(|curve| curve.family())
            .collect::<Vec<_>>(),
        [
            CurveFamily2::CubicBezier,
            CurveFamily2::CubicBezier,
            CurveFamily2::QuadraticBezier,
            CurveFamily2::QuadraticBezier,
        ]
    );

    let arcs = parse_svg_path_data("M1 0 A1 1 45 0 1 -1 0 a1 1 0 0 1 2 0 z").unwrap();
    assert_eq!(arcs.len(), 1);
    assert!(arcs[0].is_closed());
    assert_eq!(
        arcs[0]
            .path()
            .curves()
            .iter()
            .map(|curve| curve.family())
            .collect::<Vec<_>>(),
        [CurveFamily2::CircularArc, CurveFamily2::CircularArc]
    );

    let corrected = parse_svg_path_data("M0 0 A1 1 0 0 1 4 0").unwrap();
    assert_eq!(
        corrected[0].path().curves()[0].family(),
        CurveFamily2::CircularArc
    );
    let zero_radius = parse_svg_path_data("M0 0 A0 1 0 0 1 4 0").unwrap();
    assert_eq!(
        zero_radius[0].path().curves()[0].family(),
        CurveFamily2::Line
    );
}

#[test]
fn document_import_preserves_cubic_fills_and_strokes() {
    let geometry = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg">
            <path d="M0 0 C0 2 2 2 2 0 Z" stroke="black"/>
        </svg>"#,
    )
    .unwrap();

    assert_eq!(
        geometry
            .region()
            .loop_role_counts(&CurveContext::STRICT)
            .unwrap(),
        Classification::Decided((1, 0))
    );
    assert!(
        geometry
            .region()
            .boundary_loops()
            .iter()
            .flat_map(|boundary| boundary.fragments())
            .any(|fragment| matches!(
                fragment,
                BezierSplitFragment2::Materialized {
                    curve: BezierSubcurve2::Cubic(_),
                    ..
                }
            ))
    );
    assert!(geometry.wires().is_empty());
    assert_eq!(geometry.paths().len(), 1);
    assert!(
        geometry.paths()[0]
            .curves()
            .iter()
            .any(|curve| curve.family() == CurveFamily2::CubicBezier)
    );
}

#[test]
fn document_import_supports_all_csgrs_shape_elements() {
    for document in [
        r#"<svg xmlns="http://www.w3.org/2000/svg"><circle cx="3" cy="4" r="2"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><ellipse cx="3" cy="4" rx="2" ry="1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><rect x="1" y="2" width="4" height="3" rx="1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 2,0 1,1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 2,0 1,1" stroke="black"/></svg>"#,
    ] {
        let geometry = import_svg_document(document).unwrap();
        assert!(!geometry.region().is_empty(), "{document}");
    }

    let line = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><line x1="0" y1="1" x2="2" y2="3" fill="none" stroke="black"/></svg>"#,
    )
    .unwrap();
    assert!(line.region().is_empty());
    assert_eq!(line.wires().len(), 1);
}

#[test]
fn document_import_applies_inherited_styles_and_all_affine_transform_forms() {
    let geometry = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg">
            <g fill="none" transform="translate(3-4) scale(2) rotate(0) skewX(0) skewY(0)">
                <polygon style="fill:black;fill-rule:evenodd" points="0-0 2-0 2-2 0-2"/>
                <line x2="2" y2="2" stroke="black" transform="matrix(1 0 0 1 1 1)"/>
                <circle r="10" opacity="0%"/>
            </g>
        </svg>"#,
    )
    .unwrap();

    assert!(!geometry.region().is_empty());
    assert_eq!(geometry.wires().len(), 1);
    assert_eq!(
        geometry
            .region()
            .loop_role_counts(&CurveContext::STRICT)
            .unwrap(),
        Classification::Decided((1, 0))
    );
}

#[test]
fn document_import_unions_filled_shapes_and_skips_transparent_geometry() {
    let geometry = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg">
            <rect width="1" height="1"/>
            <rect x="3" width="1" height="1"/>
            <circle r="100" opacity="0"/>
        </svg>"#,
    )
    .unwrap();

    assert_eq!(
        geometry
            .region()
            .loop_role_counts(&CurveContext::STRICT)
            .unwrap(),
        Classification::Decided((2, 0))
    );
}

#[test]
fn polyline_fill_closes_while_stroke_remains_open() {
    let geometry = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 2,0 1,1" stroke="black"/></svg>"#,
    )
    .unwrap();

    assert!(!geometry.region().is_empty());
    assert_eq!(geometry.wires().len(), 1);
    assert_ne!(geometry.wires()[0].start(), geometry.wires()[0].end());
}

#[test]
fn export_round_trips_filled_and_open_topology() {
    let source = import_svg_document(
        r#"<svg xmlns="http://www.w3.org/2000/svg">
            <rect width="3" height="2"/>
            <path d="M4 0 Q5 2 6 0" fill="none" stroke="black"/>
        </svg>"#,
    )
    .unwrap();
    let document = export_svg_document(&source).unwrap();
    let reparsed = import_svg_document(&document).unwrap();

    assert!(!reparsed.region().is_empty());
    assert_eq!(reparsed.wires().len() + reparsed.paths().len(), 1);
    assert!(document.contains("viewBox="));
    assert!(document.contains("fill-rule=\"evenodd\""));
}

#[test]
fn exact_extension_round_trips_every_curve_family() {
    let one_third = rational(1, 3);
    let square_root_two = Real::from(2).sqrt().unwrap();
    let curves = vec![
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(square_root_two.clone(), Real::zero()),
                Point2::new(square_root_two, Real::from(2)),
            )
            .unwrap(),
        ),
        Curve2::from(CircularArc2::from_bulge(point(0, 0), point(2, 0), rational(1, 2)).unwrap()),
        Curve2::from(QuadraticBezier2::new(
            point(0, 0),
            Point2::new(one_third.clone(), Real::from(2)),
            point(2, 0),
        )),
        Curve2::from(CubicBezier2::new(
            point(0, 0),
            point(0, 2),
            Point2::new(Real::from(2), one_third.clone()),
            point(3, 0),
        )),
        Curve2::from(
            RationalQuadraticBezier2::try_new(
                point(0, 0),
                point(1, 2),
                point(2, 0),
                Real::one(),
                one_third.clone(),
                Real::one(),
            )
            .unwrap(),
        ),
        Curve2::from(
            RationalBezier2::try_new(
                vec![point(0, 0), point(1, 3), point(2, -1), point(4, 0)],
                vec![Real::one(), one_third.clone(), Real::from(2), Real::one()],
            )
            .unwrap(),
        ),
        Curve2::from(
            PolynomialSplineCurve2::try_new(
                2,
                vec![point(0, 0), point(1, 2), point(3, 0)],
                vec![0, 0, 0, 1, 1, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap(),
        ),
        Curve2::from(
            NurbsCurve2::try_new(
                2,
                vec![point(0, 0), point(1, 2), point(3, 0)],
                vec![Real::one(), one_third, Real::one()],
                vec![0, 0, 0, 1, 1, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap(),
        ),
    ];

    for source in curves {
        let document = export_svg_document(&single_curve_geometry(source.clone())).unwrap();
        assert!(document.contains("data-hypercurve-path=\"1:"));
        let imported = import_svg_document(&document).unwrap();
        assert_eq!(
            single_imported_curve(&imported),
            source,
            "{:?}",
            source.family()
        );
    }
}

#[test]
fn exact_extension_retains_periodic_spline_semantics() {
    let control_points = vec![point(0, 0), point(2, 0), point(2, 2), point(0, 2)];
    let period_knots = vec![0, 1, 2, 3, 4]
        .into_iter()
        .map(Real::from)
        .collect::<Vec<_>>();
    let curves = [
        Curve2::try_periodic_polynomial_bspline(2, control_points.clone(), period_knots.clone())
            .unwrap(),
        Curve2::try_periodic_nurbs(
            2,
            control_points,
            vec![Real::one(), rational(2, 3), Real::from(2), Real::one()],
            period_knots,
        )
        .unwrap(),
    ];

    for source in curves {
        assert!(source.is_periodic());
        let document = export_svg_document(&single_curve_geometry(source.clone())).unwrap();
        let imported = single_imported_curve(&import_svg_document(&document).unwrap());
        assert_eq!(imported, source);
        assert!(imported.is_periodic());
        assert_eq!(imported.period(), source.period());
    }
}

#[test]
fn exact_extension_round_trips_a_connected_mixed_family_path() {
    let curves = vec![
        Curve2::from(LineSeg2::try_new(point(0, 0), point(1, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(point(1, 0), point(2, 2), point(3, 0))),
        Curve2::from(
            RationalQuadraticBezier2::try_new(
                point(3, 0),
                point(4, -2),
                point(5, 0),
                Real::one(),
                rational(2, 3),
                Real::one(),
            )
            .unwrap(),
        ),
        Curve2::from(
            RationalBezier2::try_new(
                vec![point(5, 0), point(6, 2), point(7, -2), point(8, 0)],
                vec![Real::one(), Real::from(2), rational(1, 2), Real::one()],
            )
            .unwrap(),
        ),
    ];
    let source = CurvePath2::try_new(curves).unwrap();
    let geometry = SvgGeometry2::new(
        hypercurve::CurveRegion2::empty(),
        Vec::new(),
        vec![source.clone()],
    );
    let imported = import_svg_document(&export_svg_document(&geometry).unwrap()).unwrap();
    assert_eq!(imported.paths(), &[source]);
}

#[test]
fn exact_extension_obeys_svg_transforms_without_family_demotion() {
    let source = Curve2::from(
        RationalBezier2::try_new(
            vec![point(0, 0), point(1, 2), point(2, -1), point(3, 0)],
            vec![Real::one(), rational(1, 3), Real::from(2), Real::one()],
        )
        .unwrap(),
    );
    let document = export_svg_document(&single_curve_geometry(source.clone()))
        .unwrap()
        .replacen("<path ", "<path transform=\"translate(3 4)\" ", 1);
    let transform = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::from(3),
        Real::from(4),
    )
    .unwrap();
    let expected = source.transform_similarity(&transform).unwrap();

    let imported = import_svg_document(&document).unwrap();
    let imported_curve = single_imported_curve(&imported);
    assert_eq!(imported_curve, expected);
    assert_eq!(imported_curve.family(), CurveFamily2::RationalBezier);
}

#[test]
fn native_svg_commands_round_trip_natively_representable_families() {
    let curves = [
        (
            Curve2::from(LineSeg2::try_new(point(0, 0), point(2, 1)).unwrap()),
            " L ",
        ),
        (
            Curve2::from(
                CircularArc2::try_from_center(point(1, 0), point(-1, 0), point(0, 0), true)
                    .unwrap(),
            ),
            " A ",
        ),
        (
            Curve2::from(QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0))),
            " Q ",
        ),
        (
            Curve2::from(CubicBezier2::new(
                point(0, 0),
                point(1, 2),
                point(2, 2),
                point(3, 0),
            )),
            " C ",
        ),
    ];

    for (source, command) in curves {
        let document = export_svg_document(&single_curve_geometry(source.clone())).unwrap();
        assert!(document.contains(command), "{:?}", source.family());
        let plain_svg = remove_exact_path_attribute(document);
        let imported = import_svg_document(&plain_svg).unwrap();
        assert_eq!(
            single_imported_curve(&imported),
            source,
            "{:?}",
            source.family()
        );
    }
}

#[test]
fn full_circle_uses_two_native_arcs_and_exactly_round_trips_as_one_curve() {
    let source = Curve2::from(
        CircularArc2::try_from_center(point(2, 0), point(2, 0), point(0, 0), false).unwrap(),
    );
    let document = export_svg_document(&single_curve_geometry(source.clone())).unwrap();
    assert_eq!(document.matches(" A ").count(), 2);
    assert_eq!(
        single_imported_curve(&import_svg_document(&document).unwrap()),
        source
    );
}

#[test]
fn exact_extension_is_bounded_and_strictly_validated() {
    let geometry = single_curve_geometry(Curve2::from(CubicBezier2::new(
        point(0, 0),
        point(1, 2),
        point(2, 2),
        point(3, 0),
    )));
    let small = SvgOptions {
        max_extension_bytes: 64,
        ..SvgOptions::default()
    };
    assert!(matches!(
        geometry.to_svg_with_options(small),
        Err(SvgError::SizeOverflow { .. })
    ));

    for extension in ["2:00", "1:0", "1:not-hexadecimal", "1:00000000"] {
        let document = format!(
            r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{extension}" d="M0 0 L1 1"/></svg>"#
        );
        assert!(import_svg_document(&document).is_err());
    }

    let oversized = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="1:{}" d="M0 0 L1 1"/></svg>"#,
        "00".repeat(65)
    );
    assert!(matches!(
        import_svg_document_with_options(&oversized, small),
        Err(SvgError::SizeOverflow { .. })
    ));
}

#[test]
fn strict_import_rejects_unrepresentable_features_and_invalid_dimensions() {
    for document in [
        r#"<svg xmlns="http://www.w3.org/2000/svg"><text>geometry</text></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><line x2="1" stroke="black" stroke-dasharray="1 1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><svg x="1"><rect width="1" height="1"/></svg></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><ellipse rx="2" ry="-1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><rect width="2" height="2" rx="-1"/></svg>"#,
        r#"<svg xmlns="http://www.w3.org/2000/svg"><polygon points="0,0 1,1"/></svg>"#,
    ] {
        assert!(import_svg_document(document).is_err(), "{document}");
    }

    assert!(matches!(
        parse_svg_path_data("M0 0 A2 1 0 0 1 2 0"),
        Err(SvgError::Unsupported(_))
    ));
}

#[test]
fn options_bound_import_sampling_and_export_projection() {
    let tight = SvgOptions {
        curve_tolerance: 0.000_001,
        max_curve_segments: 8,
        max_extension_bytes: 4096,
    };
    assert!(matches!(
        import_svg_document_with_options(
            r#"<svg xmlns="http://www.w3.org/2000/svg"><circle r="10"/></svg>"#,
            tight,
        ),
        Err(SvgError::SizeOverflow { .. })
    ));

    assert!(SvgGeometry2::empty().to_svg().is_err());
}
