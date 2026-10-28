#![cfg(feature = "svg")]

use hypercurve::{
    BooleanOp, CircularArc2, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule,
    OffsetCornerStyle2, Point2, PredicatePolicy, QuadraticBezier2, Real, RegionPointLocation,
    SvgError, SvgGeometry2, SvgOptions, export_svg_document, export_svg_document_with_options,
    import_svg_document, import_svg_document_with_options,
};
use std::fmt::Write;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}
fn rectangle(x: i32, y: i32, width: i32, height: i32) -> CurvePath2 {
    let p = [
        point(x, y),
        point(x + width, y),
        point(x + width, y + height),
        point(x, y + height),
    ];
    CurvePath2::try_new(
        (0..4)
            .map(|i| Curve2::try_line(p[i].clone().into(), p[(i + 1) % 4].clone().into()).unwrap())
            .collect(),
    )
    .unwrap()
}
fn geometry(paths: &[CurvePath2]) -> SvgGeometry2 {
    SvgGeometry2::new(
        CurveRegion2::try_from_boundary_paths(paths, FillRule::NonZero).unwrap(),
        vec![],
        vec![],
    )
}
fn extension(text: &str) -> &str {
    text.split("data-hypercurve-path=\"")
        .nth(1)
        .unwrap()
        .split('"')
        .next()
        .unwrap()
}
fn encoded(bytes: &[u8]) -> String {
    let mut result = String::from("5:");
    for byte in bytes {
        write!(&mut result, "{byte:02x}").unwrap();
    }
    result
}
fn decoded(text: &str) -> Vec<u8> {
    let hex = extension(text).strip_prefix("5:").unwrap();
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect()
}
fn document(bytes: &[u8]) -> String {
    format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\"><path fill=\"black\" stroke=\"none\" data-hypercurve-path=\"{}\" d=\"M0 0 L20 0 L20 20 Z\"/></svg>",
        encoded(bytes)
    )
}
fn assert_same_set(first: &CurveRegion2, second: &CurveRegion2) {
    assert!(
        first
            .boolean_region(second, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn compound_region_replay_retains_holes_islands_and_disconnected_material() {
    let source = geometry(&[
        rectangle(0, 0, 10, 10),
        rectangle(2, 2, 6, 6).reversed().unwrap(),
        rectangle(4, 4, 2, 2),
        rectangle(20, 0, 2, 2),
    ]);
    let text = export_svg_document(&source).unwrap();
    assert_eq!(text.matches("data-hypercurve-path=\"5:").count(), 1);
    let restored = import_svg_document(&text).unwrap();
    assert!(restored.wires().is_empty() && restored.paths().is_empty());
    assert_eq!(restored.region().loop_role_counts().unwrap(), (3, 1));
    for (p, expected) in [
        (point(1, 1), RegionPointLocation::Inside),
        (point(3, 3), RegionPointLocation::Outside),
        (point(5, 5), RegionPointLocation::Inside),
        (point(21, 1), RegionPointLocation::Inside),
        (point(15, 1), RegionPointLocation::Outside),
        (point(2, 3), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            restored.region().classify_point(&p.into()).unwrap(),
            expected
        );
    }
    assert_same_set(restored.region(), source.region());
    assert_eq!(export_svg_document(&restored).unwrap(), text);
    let grown = restored
        .region()
        .offset(
            q(1, 4),
            &OffsetCornerStyle2::Miter {
                limit: Real::from(2),
            },
        )
        .unwrap();
    let again = grown
        .offset(
            q(-1, 4),
            &OffsetCornerStyle2::Miter {
                limit: Real::from(2),
            },
        )
        .unwrap();
    assert_same_set(&again, source.region());
}

#[test]
fn filled_circle_metadata_preserves_exact_boundary_across_display_tolerances() {
    let x = Real::from(2).sqrt().unwrap();
    let center = Point2::new(x.clone(), Real::from(3));
    let endpoint = Point2::new(&x + Real::one(), Real::from(3));
    let circle = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(endpoint.clone(), endpoint, center.clone(), false)
            .unwrap()
            .into(),
    ])
    .unwrap();
    let source = geometry(&[circle]);
    let boundary = Point2::new(&x + q(3, 5), Real::from(3) + q(4, 5));
    let mut extensions = Vec::new();
    for tolerance in [0.8, 0.01, 0.8] {
        let text = export_svg_document_with_options(
            &source,
            SvgOptions {
                curve_tolerance: tolerance,
                ..SvgOptions::default()
            },
        )
        .unwrap();
        extensions.push(extension(&text).to_owned());
        let before = text.find(" d=\"").unwrap() + 5;
        let after = before + text[before..].find('"').unwrap();
        let altered = format!(
            "{}M100 100 L101 100 L101 101 Z{}",
            &text[..before],
            &text[after..]
        );
        let restored = import_svg_document(&altered).unwrap();
        assert_eq!(
            restored
                .region()
                .classify_point(&boundary.clone().into())
                .unwrap(),
            RegionPointLocation::Boundary
        );
        assert_eq!(
            restored
                .region()
                .classify_point(&center.clone().into())
                .unwrap(),
            RegionPointLocation::Inside
        );
        assert_eq!(
            restored
                .region()
                .classify_point(&point(100, 100).into())
                .unwrap(),
            RegionPointLocation::Outside
        );
        assert_same_set(restored.region(), source.region());
    }
    assert!(extensions.windows(2).all(|pair| pair[0] == pair[1]));
}

#[test]
fn compound_region_replay_rejects_invalid_roots_counts_headers_and_open_paths() {
    let text = export_svg_document(&geometry(&[rectangle(0, 0, 2, 2)])).unwrap();
    let bytes = decoded(&text);
    let mut invalid = Vec::new();
    let mut changed = bytes.clone();
    let n = changed.len();
    changed[n - 4..].copy_from_slice(&u32::MAX.to_be_bytes());
    invalid.push(changed);
    let mut changed = bytes.clone();
    let n = changed.len();
    changed[n - 24..n - 20].copy_from_slice(&u32::MAX.to_be_bytes());
    invalid.push(changed);
    let mut changed = bytes.clone();
    let n = changed.len();
    changed[n - 20..n - 16].copy_from_slice(&u32::MAX.to_be_bytes());
    invalid.push(changed);
    let mut changed = bytes.clone();
    changed.push(0);
    invalid.push(changed);
    let mut changed = bytes.clone();
    changed[3] = b'4';
    invalid.push(changed);
    let mut changed = bytes.clone();
    let n = changed.len();
    changed[n - 20..n - 16].copy_from_slice(&3_u32.to_be_bytes());
    changed.truncate(n - 4);
    invalid.push(changed);
    // An unreferenced invalid definition must fail even with no path roots.
    let mut changed = b"HCP5".to_vec();
    changed.extend_from_slice(&1_u32.to_be_bytes());
    changed.push(255);
    changed.extend_from_slice(&0_u32.to_be_bytes());
    invalid.push(changed);
    for (index, bytes) in invalid.into_iter().enumerate() {
        let text = document(&bytes);
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let result = hypercurve::evaluate_under(policy, || import_svg_document(&text));
            let error = result.as_unverified().as_ref().unwrap_err();
            assert!(
                match index {
                    0 | 3 | 4 => matches!(error, SvgError::MalformedInput(_)),
                    1 | 2 => matches!(error, SvgError::SizeOverflow { .. }),
                    5 => matches!(
                        error,
                        SvgError::Exact(hypercurve::ExactCurveError::Invalid { .. })
                    ),
                    6 => matches!(error, SvgError::Unsupported(_)),
                    _ => unreachable!(),
                },
                "wrong typed failure for case {index}: {error:?}"
            );
        }
    }
}

#[test]
fn empty_compound_metadata_is_a_valid_further_operand() {
    let mut bytes = b"HCP5".to_vec();
    bytes.extend_from_slice(&0_u32.to_be_bytes());
    bytes.extend_from_slice(&0_u32.to_be_bytes());
    let empty = import_svg_document(&document(&bytes)).unwrap();
    assert!(empty.region().is_empty() && empty.paths().is_empty() && empty.wires().is_empty());
    let source = geometry(&[rectangle(0, 0, 2, 2)]);
    let union = empty
        .region()
        .boolean_region(source.region(), BooleanOp::Union)
        .unwrap();
    assert_same_set(&union, source.region());
    assert!(
        empty
            .region()
            .offset(Real::one(), &OffsetCornerStyle2::Round)
            .unwrap()
            .is_empty()
    );
    // Existing finite-bounds export policy remains explicit for a whole empty document.
    assert!(export_svg_document(&empty).is_err());
}

#[test]
fn compound_extension_byte_limits_cover_all_paths_and_allow_retry() {
    let source = geometry(&[rectangle(0, 0, 2, 2), rectangle(4, 0, 2, 2)]);
    let text = export_svg_document(&source).unwrap();
    let length = decoded(&text).len();
    assert!(length > 64);
    let small = SvgOptions {
        max_extension_bytes: length - 1,
        ..SvgOptions::default()
    };
    assert!(matches!(
        export_svg_document_with_options(&source, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    assert!(matches!(
        import_svg_document_with_options(&text, small),
        Err(SvgError::SizeOverflow { .. })
    ));
    let exact = SvgOptions {
        max_extension_bytes: length,
        ..SvgOptions::default()
    };
    assert_eq!(
        export_svg_document_with_options(&source, exact).unwrap(),
        text
    );
    assert_same_set(
        import_svg_document_with_options(&text, exact)
            .unwrap()
            .region(),
        source.region(),
    );
}

#[test]
fn compound_fill_replay_applies_svg_transforms_to_exact_boundaries() {
    let source = geometry(&[
        rectangle(0, 0, 4, 4),
        rectangle(1, 1, 2, 2).reversed().unwrap(),
    ]);
    let text = export_svg_document(&source).unwrap().replacen(
        "<path ",
        "<path transform=\"matrix(-2 0 0 2 10 5)\" ",
        1,
    );
    let replay = import_svg_document(&text).unwrap();
    assert_eq!(replay.region().loop_role_counts().unwrap(), (1, 1));
    for (p, location) in [
        (point(9, 6), RegionPointLocation::Inside),
        (point(6, 9), RegionPointLocation::Outside),
        (point(8, 9), RegionPointLocation::Boundary),
        (point(11, 6), RegionPointLocation::Outside),
    ] {
        assert_eq!(replay.region().classify_point(&p.into()).unwrap(), location);
    }
    let expected = geometry(&[
        rectangle(2, 5, 8, 8),
        rectangle(4, 7, 4, 4).reversed().unwrap(),
    ]);
    assert_same_set(replay.region(), expected.region());
}

#[test]
fn compound_fill_and_legacy_strokes_replay_without_consuming_approximation() {
    let filled = geometry(&[rectangle(0, 0, 2, 2)]);
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(point(3, 0), point(4, 2), point(5, 0)).into(),
    ])
    .unwrap();
    let source = SvgGeometry2::new(filled.region().clone(), vec![], vec![path.clone()]);
    let text = export_svg_document(&source).unwrap();
    assert!(
        text.contains("data-hypercurve-path=\"3:") && text.contains("data-hypercurve-path=\"5:")
    );
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let replay = import_svg_document(&text).unwrap();
            assert_eq!(replay.paths(), std::slice::from_ref(&path));
            assert_same_set(replay.region(), source.region());
            assert_eq!(export_svg_document(&replay).unwrap(), text);
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
}

#[test]
fn filled_region_export_revalidates_qualified_connectivity() {
    // This pre-existing policy control is outside the algebraic envelope.
    // Its qualification must still survive any interchange boundary.
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    let source = hypercurve::provisional(|| {
        let p = [
            point(0, 0),
            point(4, 0),
            Point2::new(Real::from(4) + zero.clone(), Real::zero()),
            Point2::new(Real::from(4) + zero, Real::from(2)),
            point(0, 2),
        ];
        let path = CurvePath2::try_new(vec![
            Curve2::try_line(p[0].clone().into(), p[1].clone().into()).unwrap(),
            Curve2::try_line(p[2].clone().into(), p[3].clone().into()).unwrap(),
            Curve2::try_line(p[3].clone().into(), p[4].clone().into()).unwrap(),
            Curve2::try_line(p[4].clone().into(), p[0].clone().into()).unwrap(),
        ])
        .unwrap();
        geometry(&[path])
    });
    assert_eq!(source.certainty(), CurveCertainty::Approximate512Consumed);
    let source = source.into_unverified();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let normalized =
            hypercurve::evaluate_under(policy, || source.region().regularized_region());
        let paths = hypercurve::evaluate_under(policy, || source.region().boundary_paths());
        if policy == PredicatePolicy::STRICT {
            for blocked in [
                normalized.as_unverified().as_ref().unwrap_err(),
                paths.as_unverified().as_ref().unwrap_err(),
            ] {
                assert!(
                    matches!(blocked, hypercurve::ExactCurveError::Blocked(reason) if reason.reason() == hypercurve::UncertaintyReason::Predicate)
                );
            }
        } else {
            assert!(normalized.as_unverified().is_ok() && paths.as_unverified().is_ok());
            assert_eq!(
                normalized.certainty(),
                CurveCertainty::Approximate512Consumed
            );
            assert_eq!(paths.certainty(), CurveCertainty::Approximate512Consumed);
        }
    }

    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let exported = hypercurve::evaluate_under(policy, || export_svg_document(&source));
        assert!(
            exported.as_unverified().is_err(),
            "an unresolved retained join cannot become certified native region metadata"
        );
    }
}
