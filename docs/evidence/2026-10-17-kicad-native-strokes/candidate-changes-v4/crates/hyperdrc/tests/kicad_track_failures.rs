//! Malformed track source cannot be published as a successful board.
use hypercurve::RegionPointLocation;
use hyperdrc::Scalar;
use hyperdrc::kicad::{BoardModel, TrackLoadError, TrackLoadFailure, load_kicad_pcb};
use std::sync::atomic::{AtomicU64, Ordering};

fn board(fields: &str) -> anyhow::Result<BoardModel> {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "hyperdrc-track-failure-{}-{}.kicad_pcb",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(
        &path,
        format!("(kicad_pcb (net 1 signal) (segment (layer F.Cu) (net 1) {fields}))"),
    )
    .unwrap();
    let result = load_kicad_pcb(&path);
    std::fs::remove_file(path).unwrap();
    result
}

fn invalid_source(fields: &str, expected: TrackLoadFailure) {
    for policy in [
        hyperlimit::PredicatePolicy::STRICT,
        hyperlimit::PredicatePolicy::APPROXIMATE_512,
        hyperlimit::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || board(fields)).into_unverified();
        let error = result.expect_err("invalid source must not publish a board");
        let track = error
            .downcast_ref::<TrackLoadError>()
            .expect("typed track source error");
        assert_eq!(track.segment_index, 0);
        assert_eq!(track.failure, expected, "{fields}");
        assert!(format!("{error:#}").contains("failed to load tracks from"));
    }
}

macro_rules! malformed {
    ($name:ident, $fields:literal, $failure:expr) => {
        #[test]
        fn $name() {
            invalid_source($fields, $failure);
        }
    };
}
malformed!(
    start_missing,
    " (end 12 20) (width 2)",
    TrackLoadFailure::MissingField { field: "start" }
);
malformed!(
    start_empty,
    "(start) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "start",
        expected: 2,
        actual: 0
    }
);
malformed!(
    start_bad,
    "(start bad 20) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 0
    }
);
malformed!(
    start_extra,
    "(start 10 20 30) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "start",
        expected: 2,
        actual: 3
    }
);
malformed!(
    start_duplicate,
    "(start 10 20) (start 10 20) (end 12 20) (width 2)",
    TrackLoadFailure::DuplicateField { field: "start" }
);
malformed!(
    start_nested,
    "(start (value 10) 20) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 0
    }
);
malformed!(
    start_nan,
    "(start NaN 20) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 0
    }
);
malformed!(
    start_infinity,
    "(start inf 20) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 0
    }
);
malformed!(
    start_short,
    "(start 10) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "start",
        expected: 2,
        actual: 1
    }
);
malformed!(
    start_bad_y,
    "(start 10 bad) (end 12 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 1
    }
);
malformed!(
    end_missing,
    "(start 10 20)  (width 2)",
    TrackLoadFailure::MissingField { field: "end" }
);
malformed!(
    end_empty,
    "(start 10 20) (end) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "end",
        expected: 2,
        actual: 0
    }
);
malformed!(
    end_bad,
    "(start 10 20) (end bad 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "end",
        value_index: 0
    }
);
malformed!(
    end_extra,
    "(start 10 20) (end 10 20 30) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "end",
        expected: 2,
        actual: 3
    }
);
malformed!(
    end_duplicate,
    "(start 10 20) (end 12 20) (end 12 20) (width 2)",
    TrackLoadFailure::DuplicateField { field: "end" }
);
malformed!(
    end_nested,
    "(start 10 20) (end (value 10) 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "end",
        value_index: 0
    }
);
malformed!(
    end_nan,
    "(start 10 20) (end NaN 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "end",
        value_index: 0
    }
);
malformed!(
    end_infinity,
    "(start 10 20) (end inf 20) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "end",
        value_index: 0
    }
);
malformed!(
    end_short,
    "(start 10 20) (end 10) (width 2)",
    TrackLoadFailure::InvalidArity {
        field: "end",
        expected: 2,
        actual: 1
    }
);
malformed!(
    end_bad_y,
    "(start 10 20) (end 10 bad) (width 2)",
    TrackLoadFailure::InvalidNumber {
        field: "end",
        value_index: 1
    }
);
malformed!(
    width_missing,
    "(start 10 20) (end 12 20) ",
    TrackLoadFailure::MissingField { field: "width" }
);
malformed!(
    width_empty,
    "(start 10 20) (end 12 20) (width)",
    TrackLoadFailure::InvalidArity {
        field: "width",
        expected: 1,
        actual: 0
    }
);
malformed!(
    width_bad,
    "(start 10 20) (end 12 20) (width bad)",
    TrackLoadFailure::InvalidNumber {
        field: "width",
        value_index: 0
    }
);
malformed!(
    width_extra,
    "(start 10 20) (end 12 20) (width 2 3)",
    TrackLoadFailure::InvalidArity {
        field: "width",
        expected: 1,
        actual: 2
    }
);
malformed!(
    width_duplicate,
    "(start 10 20) (end 12 20) (width 2) (width 2)",
    TrackLoadFailure::DuplicateField { field: "width" }
);
malformed!(
    width_nested,
    "(start 10 20) (end 12 20) (width (value 10))",
    TrackLoadFailure::InvalidNumber {
        field: "width",
        value_index: 0
    }
);
malformed!(
    width_nan,
    "(start 10 20) (end 12 20) (width NaN)",
    TrackLoadFailure::InvalidNumber {
        field: "width",
        value_index: 0
    }
);
malformed!(
    width_infinity,
    "(start 10 20) (end 12 20) (width inf)",
    TrackLoadFailure::InvalidNumber {
        field: "width",
        value_index: 0
    }
);
malformed!(
    width_negative,
    "(start 10 20) (end 12 20) (width -1)",
    TrackLoadFailure::NegativeWidth
);
malformed!(
    width_negative_subrange,
    "(start 10 20) (end 12 20) (width -1e-400)",
    TrackLoadFailure::NegativeWidth
);
malformed!(
    zero_width_missing_end,
    "(start 10 20) (width 0)",
    TrackLoadFailure::MissingField { field: "end" }
);
malformed!(
    zero_width_invalid_start,
    "(start bad 20) (end 12 20) (width 0)",
    TrackLoadFailure::InvalidNumber {
        field: "start",
        value_index: 0
    }
);

#[test]
fn valid_source_controls_preserve_metadata_and_known_membership() {
    let model = board("(start 10 20) (end 12 20) (width 2)").unwrap();
    assert_eq!(model.copper.len(), 1);
    let feature = &model.copper[0];
    assert_eq!(feature.layer, "F.Cu");
    assert_eq!(feature.net.as_deref(), Some("signal"));
    assert_eq!(feature.location, [Scalar::from(11), Scalar::from(20)]);
    // Both the finite legacy body and the intended round stroke contain this
    // central point and exclude this distant point. This is not full fidelity.
    for region in [
        feature.region.clone(),
        model.all_copper().unwrap(),
        model.copper_layers(&[]).unwrap().pop().unwrap().1,
    ] {
        assert_eq!(
            region.classify_xy(11.into(), 20.into()),
            Ok(RegionPointLocation::Inside)
        );
        assert_eq!(
            region.classify_xy(1000.into(), 1000.into()),
            Ok(RegionPointLocation::Outside)
        );
        assert!(region.try_into_native().is_ok());
    }
}

#[test]
fn validated_zero_width_is_empty_even_when_coordinates_exceed_binary64() {
    for fields in [
        "(start 10 20) (end 12 20) (width 0)",
        "(start 1e400 0) (end 1e400 2) (width 0)",
    ] {
        let model = board(fields).unwrap();
        assert!(model.copper.is_empty());
        assert_eq!(model.all_copper().unwrap().try_is_empty(), Ok(true));
    }
}

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    assert!(matches!(polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
        if blocker.reason()==hypercurve::UncertaintyReason::BudgetExhausted));
}

#[test]
fn exhausted_source_load_cannot_escape_as_a_board_and_later_load_recovers() {
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(
        hyperlimit::PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            escaped = Some(board("(start 10 20) (end 12 20) (width 2)"));
        },
    );
    assert!(denied.result.is_err());
    let error = escaped
        .unwrap()
        .expect_err("exhausted load cannot escape as a board");
    let track = error
        .downcast_ref::<TrackLoadError>()
        .expect("typed budget failure survives scope exit");
    let TrackLoadFailure::BudgetExceeded(budget) = track.failure else {
        panic!("{track:?}")
    };
    assert_eq!(budget.limit, 0);
    assert_eq!(budget.used, 0);
    assert_eq!(budget.requested, 1);
    assert!(std::error::Error::source(track).is_some());
    let retried = board("(start 10 20) (end 12 20) (width 2)").unwrap();
    assert_eq!(
        retried.copper[0].region.classify_xy(11.into(), 20.into()),
        Ok(RegionPointLocation::Inside)
    );
}

#[test]
fn exact_source_ranges_remain_certified_after_native_handoff() {
    // These are the original finite-adapter failure fixtures, now admitted
    // exactly. Endpoint components are nondecreasing in every listed case.
    for (fields, start, end, length, width) in [
        (
            "(start 0 0) (end 2 0) (width 1e-400)",
            ["0", "0"],
            ["2", "0"],
            "2",
            "1e-400",
        ),
        (
            "(start 0 0) (end 1e-200 0) (width 2)",
            ["0", "0"],
            ["1e-200", "0"],
            "1e-200",
            "2",
        ),
        (
            "(start 1e400 0) (end 1e400 2) (width 2)",
            ["1e400", "0"],
            ["1e400", "2"],
            "2",
            "2",
        ),
        (
            "(start -1e308 0) (end 1e308 0) (width 2)",
            ["-1e308", "0"],
            ["1e308", "0"],
            "2e308",
            "2",
        ),
        (
            "(start 0 0) (end 2 0) (width 1e400)",
            ["0", "0"],
            ["2", "0"],
            "2",
            "1e400",
        ),
        (
            "(start 100000000000000000000 0) (end 100000000000000000001 0) (width 2)",
            ["100000000000000000000", "0"],
            ["100000000000000000001", "0"],
            "1",
            "2",
        ),
    ] {
        let start = start.map(|v| v.parse::<Scalar>().unwrap());
        let end = end.map(|v| v.parse::<Scalar>().unwrap());
        let width = width.parse::<Scalar>().unwrap();
        let radius = (&width / Scalar::from(2)).unwrap();
        let expected_area =
            length.parse::<Scalar>().unwrap() * &width + Scalar::pi() * &radius * &radius;
        let midpoint = [
            ((&start[0] + &end[0]) / Scalar::from(2)).unwrap(),
            ((&start[1] + &end[1]) / Scalar::from(2)).unwrap(),
        ];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board(fields).unwrap();
                assert_eq!(model.copper.len(), 1);
                assert_eq!(model.copper[0].location, midpoint);
                let profile = model.all_copper().unwrap();
                assert_eq!(
                    profile.classify_xy(midpoint[0].clone(), midpoint[1].clone()),
                    Ok(RegionPointLocation::Inside)
                );
                assert_eq!(
                    profile.classify_xy(&start[0] - &radius, start[1].clone()),
                    Ok(RegionPointLocation::Boundary)
                );
                let native = profile.try_into_native().unwrap();
                let rebuilt = hypercurve::CurveRegion2::try_from_boundary_paths(
                    &native.boundary_paths().unwrap(),
                    hypercurve::FillRule::NonZero,
                )
                .unwrap();
                for region in [&native, &rebuilt] {
                    assert_eq!(
                        hyperlimit::compare_reals(
                            &region.filled_area().unwrap().unwrap(),
                            &expected_area,
                            hyperlimit::PredicatePolicy::STRICT,
                        )
                        .value(),
                        Some(std::cmp::Ordering::Equal),
                        "{fields}"
                    );
                    let bounds = region.bounds().unwrap().unwrap();
                    for (actual, expected) in [
                        (bounds.min_x(), &start[0] - &radius),
                        (bounds.min_y(), &start[1] - &radius),
                        (bounds.max_x(), &end[0] + &radius),
                        (bounds.max_y(), &end[1] + &radius),
                    ] {
                        assert_eq!(
                            hyperlimit::compare_reals(
                                actual,
                                &expected,
                                hyperlimit::PredicatePolicy::STRICT,
                            )
                            .value(),
                            Some(std::cmp::Ordering::Equal),
                            "{fields}"
                        );
                    }
                }
            });
            assert!(outcome.certified().is_some());
        }
    }
}

#[test]
fn second_segment_failure_does_not_publish_the_valid_prefix() {
    let error = board("(start 10 20) (end 12 20) (width 2)) (segment (start 0 0) (end 2 0)")
        .expect_err("valid prefix must not escape as a complete board");
    let track = error.downcast_ref::<TrackLoadError>().unwrap();
    assert_eq!(track.segment_index, 1);
    assert_eq!(
        track.failure,
        TrackLoadFailure::MissingField { field: "width" }
    );
}

#[test]
fn cli_preserves_source_diagnostic_and_cannot_allow_a_failed_load() {
    let path = std::env::temp_dir().join(format!(
        "hyperdrc-track-cli-{}.kicad_pcb",
        std::process::id()
    ));
    std::fs::write(
        &path,
        "(kicad_pcb (segment (start 10 20) (end 12 20) (width 2)) (segment (start 0 0) (end 2 0)))",
    )
    .unwrap();
    let result = std::process::Command::new(env!("CARGO_BIN_EXE_hyperdrc"))
        .args(["--kicad-pcb"])
        .arg(&path)
        .args([
            "--check",
            "copper-net-intent",
            "--format",
            "json",
            "--allow-findings",
        ])
        .output()
        .unwrap();
    std::fs::remove_file(&path).unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(
        result.stdout.is_empty(),
        "no success report for a partial board"
    );
    let diagnostic = String::from_utf8(result.stderr).unwrap();
    assert!(diagnostic.contains(&path.display().to_string()));
    assert!(
        diagnostic.contains("KiCad segment 2: missing width"),
        "{diagnostic}"
    );
}

#[test]
fn coincident_tracks_are_exact_disks_including_decimal_source_precision() {
    for (fields, center, radius) in [
        ("(start 1 2) (end 1 2) (width 2)", ["1", "2"], "1"),
        ("(start 3.0000 4.00) (end 3 4) (width 2)", ["3", "4"], "1"),
        (
            "(start 9007199254740992.1 -0.00000000000000000000001) (end 9007199254740992.1 -0.00000000000000000000001) (width 0.20000000000000000000004)",
            ["9007199254740992.1", "-0.00000000000000000000001"],
            "0.10000000000000000000002",
        ),
    ] {
        let center = center.map(|value| value.parse::<Scalar>().unwrap());
        let radius = radius.parse::<Scalar>().unwrap();
        let expected_area = Scalar::pi() * &radius * &radius;
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board(fields).unwrap();
                assert_eq!(model.copper.len(), 1);
                let feature = &model.copper[0];
                assert_eq!(feature.location, center);
                assert_eq!(feature.layer, "F.Cu");
                assert_eq!(feature.net.as_deref(), Some("signal"));
                let region = &feature.region;
                assert_eq!(
                    hyperlimit::compare_reals(
                        &region.try_area().unwrap(),
                        &expected_area,
                        hyperlimit::PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(std::cmp::Ordering::Equal)
                );
                assert_eq!(
                    region.classify_xy(center[0].clone(), center[1].clone()),
                    Ok(RegionPointLocation::Inside)
                );
                assert_eq!(
                    region.classify_xy(&center[0] + &radius, center[1].clone()),
                    Ok(RegionPointLocation::Boundary)
                );
                assert_eq!(
                    region.classify_xy(&center[0] + &radius * Scalar::from(2), center[1].clone()),
                    Ok(RegionPointLocation::Outside)
                );
                let native = region.clone().try_into_native().unwrap();
                let bounds = native.bounds().unwrap().unwrap();
                assert_eq!(bounds.min_x(), &(&center[0] - &radius));
                assert_eq!(bounds.max_x(), &(&center[0] + &radius));
                assert_eq!(
                    hyperlimit::compare_reals(
                        &native.filled_area().unwrap().unwrap(),
                        &expected_area,
                        hyperlimit::PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(std::cmp::Ordering::Equal)
                );
            });
            assert!(outcome.certified().is_some());
        }
    }
}
