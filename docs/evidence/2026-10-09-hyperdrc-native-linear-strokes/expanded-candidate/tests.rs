use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
use hyperdrc::Scalar;
use hyperdrc::geometry::{line_polygon, polygons_to_profile};
use hyperdrc::kicad::{BoardModel, CopperKind, load_kicad_pcb};
use std::sync::atomic::{AtomicU64, Ordering};

fn scalar(token: &str) -> Scalar {
    token.parse().unwrap()
}

fn same(actual: &Scalar, expected: &Scalar) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal),
        "actual={actual:?}, expected={expected:?}"
    );
}

fn board(fields: &str) -> BoardModel {
    static SEQUENCE: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "hyperdrc-linear-source-{}-{}.kicad_pcb",
        std::process::id(),
        SEQUENCE.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(
        &path,
        format!("(kicad_pcb (net 1 signal) (segment (layer F.Cu) (net 1) {fields}))"),
    )
    .unwrap();
    let result = load_kicad_pcb(&path);
    std::fs::remove_file(path).unwrap();
    result.unwrap()
}

#[test]
fn large_finite_stroke_does_not_collapse_its_width() {
    let polygon = line_polygon([1e20, 0.0], [1e20, 16_384.0], 16_384.0).unwrap();
    same(&polygon.try_area().unwrap(), &Scalar::from(268_435_456));
    let region = polygons_to_profile(vec![polygon], None);
    let x = scalar("1e20");
    assert_eq!(
        region.classify_xy(&x + Scalar::from(8192), Scalar::one()),
        Ok(Boundary)
    );
    assert_eq!(
        region.classify_xy(&x + Scalar::from(8193), Scalar::one()),
        Ok(Outside)
    );
}

#[test]
fn finite_length_squaring_cannot_underflow_to_zero() {
    let length = 1e-200;
    let polygon = line_polygon([0.0, 0.0], [length, 0.0], length).unwrap();
    let value = Scalar::try_from(length).unwrap();
    same(&polygon.try_area().unwrap(), &(&value * &value));
}

#[test]
fn subnormal_width_is_halved_after_lifting() {
    let width = f64::from_bits(1);
    let polygon = line_polygon([0.0, 0.0], [2.0, 0.0], width).unwrap();
    same(
        &polygon.try_area().unwrap(),
        &(Scalar::from(2) * Scalar::try_from(width).unwrap()),
    );
}

#[test]
fn finite_endpoint_difference_can_exceed_float_range() {
    let polygon = line_polygon([-f64::MAX, 0.0], [f64::MAX, 0.0], 1.0).unwrap();
    same(
        &polygon.try_area().unwrap(),
        &(Scalar::from(2) * Scalar::try_from(f64::MAX).unwrap()),
    );
}

#[test]
fn finite_length_squaring_cannot_overflow() {
    let polygon = line_polygon([0.0, 0.0], [1e200, 0.0], 2.0).unwrap();
    same(
        &polygon.try_area().unwrap(),
        &(Scalar::from(2) * Scalar::try_from(1e200).unwrap()),
    );
}

#[test]
fn pythagorean_diagonal_has_exact_rational_area() {
    let polygon = line_polygon([-1.0, -1.0], [2.0, 3.0], 0.25).unwrap();
    same(&polygon.try_area().unwrap(), &scalar("1.25"));
}

#[test]
fn irrational_diagonal_retains_native_surd_width() {
    let polygon = line_polygon([0.0, 0.0], [1.0, 1.0], 2.0).unwrap();
    same(
        &polygon.try_area().unwrap(),
        &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap()),
    );
}

#[test]
fn ordinary_butt_stroke_keeps_its_contract() {
    let polygon = line_polygon([0.0, 0.0], [2.0, 0.0], 0.5).unwrap();
    same(&polygon.try_area().unwrap(), &Scalar::one());
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(
        region.classify_xy(scalar("-0.1"), Scalar::zero()),
        Ok(Outside)
    );
    assert_eq!(
        region.classify_xy(Scalar::zero(), Scalar::zero()),
        Ok(Boundary)
    );
}

#[test]
fn rejected_finite_adapter_inputs_remain_absent() {
    for (start, end, width) in [
        ([0.0, 0.0], [0.0, 0.0], 1.0),
        ([0.0, 0.0], [1.0, 0.0], 0.0),
        ([0.0, 0.0], [1.0, 0.0], -1.0),
        ([f64::NAN, 0.0], [1.0, 0.0], 1.0),
        ([0.0, 0.0], [1.0, 0.0], f64::INFINITY),
    ] {
        assert!(line_polygon(start, end, width).is_none());
    }
}

#[test]
fn kicad_linear_tracks_include_both_round_caps() {
    let board = board("(start 0 0) (end 2 0) (width 2)");
    assert_eq!(board.copper.len(), 1);
    let feature = &board.copper[0];
    assert_eq!(feature.kind, CopperKind::Segment);
    assert_eq!(feature.net.as_deref(), Some("signal"));
    same(
        &feature.region.try_area().unwrap(),
        &(Scalar::from(4) + Scalar::pi()),
    );
    for (x, expected) in [
        ("-1", Boundary),
        ("-0.5", Inside),
        ("3", Boundary),
        ("3.00001", Outside),
    ] {
        assert_eq!(
            feature.region.classify_xy(scalar(x), Scalar::zero()),
            Ok(expected)
        );
    }
}

#[test]
fn track_decimal_endpoints_and_width_are_never_rounded_first() {
    let board = board(
        "(start 9007199254740992.1 0) (end 9007199254740992.3 0) (width 0.20000000000000000000004)",
    );
    assert_eq!(board.copper.len(), 1);
    let width = scalar("0.20000000000000000000004");
    let radius = scalar("0.10000000000000000000002");
    same(&board.copper[0].location[0], &scalar("9007199254740992.2"));
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(scalar("0.2") * width + Scalar::pi() * &radius * radius),
    );
}

#[test]
fn track_subrange_width_and_length_remain_nonzero() {
    let board = board("(start 0 0) (end 1e-400 0) (width 2e-400)");
    assert_eq!(board.copper.len(), 1);
    let radius = scalar("1e-400");
    same(
        &board.copper[0].region.try_area().unwrap(),
        &((Scalar::from(2) + Scalar::pi()) * &radius * radius),
    );
}

#[test]
fn track_superrange_coordinates_stay_native() {
    let board = board("(start 1e400 0) (end 1e400 2) (width 2)");
    assert_eq!(board.copper.len(), 1);
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::from(4) + Scalar::pi()),
    );
    same(&board.copper[0].location[0], &scalar("1e400"));
}

#[test]
fn coincident_track_endpoints_keep_the_width_disk() {
    let board = board("(start 3 4) (end 3 4) (width 2)");
    assert_eq!(board.copper.len(), 1);
    same(&board.copper[0].region.try_area().unwrap(), &Scalar::pi());
    assert_eq!(
        board.copper[0]
            .region
            .classify_xy(Scalar::from(4), Scalar::from(4)),
        Ok(Boundary)
    );
}

#[test]
fn zero_track_width_is_certified_empty() {
    assert!(board("(start 0 0) (end 2 0) (width 0)").copper.is_empty());
}

#[test]
fn malformed_track_fields_retain_source_failure() {
    for fields in [
        "(start 0 0) (end 2 0)",
        "(start 0 0) (end 2 0) (width bad)",
        "(start 0 0) (end 2 0) (width -1)",
        "(start 0 0) (end 2 0) (width NaN)",
        "(start 0 0) (end 2 0) (width 1 2)",
        "(start bad 0) (end 2 0) (width 1)",
        "(start 0 0) (width 1)",
        "(start 0 0 4) (end 2 0) (width 1)",
    ] {
        let board = board(fields);
        assert_eq!(board.copper.len(), 1, "{fields}");
        assert!(board.copper[0].region.try_area().is_err(), "{fields}");
        assert!(
            board.copper[0]
                .region
                .classify_xy(Scalar::zero(), Scalar::zero())
                .is_err(),
            "{fields}"
        );
    }
}

#[test]
fn reversed_stroke_preserves_exact_bounds_and_area() {
    let forward = line_polygon([1e20, 0.0], [1e20 + 16_384.0, 16_384.0], 0.5).unwrap();
    let reverse = line_polygon([1e20 + 16_384.0, 16_384.0], [1e20, 0.0], 0.5).unwrap();
    same(&forward.try_area().unwrap(), &reverse.try_area().unwrap());
    for (a, b) in forward
        .try_bounds()
        .unwrap()
        .unwrap()
        .iter()
        .zip(reverse.try_bounds().unwrap().unwrap())
    {
        same(a, &b);
    }
}

#[test]
fn exact_diagonal_track_area_includes_caps_without_rounding_the_width() {
    let board = board("(start -1 -1) (end 2 3) (width 0.2)");
    assert_eq!(board.copper.len(), 1);
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::one() + scalar("0.01") * Scalar::pi()),
    );
}

#[test]
fn irrational_diagonal_track_keeps_line_and_arc_geometry() {
    let board = board("(start 0 0) (end 1 1) (width 2)");
    assert_eq!(board.copper.len(), 1);
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap() + Scalar::pi()),
    );
    assert_eq!(
        board.copper[0]
            .region
            .classify_xy(Scalar::from(-1), Scalar::zero()),
        Ok(Boundary)
    );
}

#[test]
fn track_caps_drive_actual_short_readiness() {
    let mut left = board("(start 0 0) (end 2 0) (width 2)");
    let mut right = board("(start 3 0) (end 5 0) (width 2)");
    right.copper[0].net = Some("other".to_owned());
    left.copper.extend(right.copper);
    let findings =
        hyperdrc::checks::continuity::different_net_short_readiness(&left, &[], &Scalar::zero());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "different-net-short-readiness");
}

#[test]
fn tangent_track_caps_do_not_create_filled_overlap() {
    let left = board("(start 0 0) (end 2 0) (width 2)");
    let right = board("(start 4 0) (end 6 0) (width 2)");
    let overlap = left.copper[0]
        .region
        .try_intersection(&right.copper[0].region)
        .unwrap();
    assert_eq!(overlap.try_is_empty(), Ok(true));
    assert_eq!(
        left.copper[0]
            .region
            .classify_xy(Scalar::from(3), Scalar::zero()),
        Ok(Boundary)
    );
    assert_eq!(
        right.copper[0]
            .region
            .classify_xy(Scalar::from(3), Scalar::zero()),
        Ok(Boundary)
    );
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
        if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted));
}

#[test]
fn denied_stroke_construction_retains_failure_after_scope_exit() {
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(
        hyperdrc::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            escaped = line_polygon([0.0, 0.0], [3.0, 4.0], 0.25);
        },
    );
    assert!(denied.result.is_err());
    let escaped = escaped.unwrap();
    assert!(escaped.try_area().is_err());
    assert!(
        polygons_to_profile(vec![escaped], None)
            .try_into_native()
            .is_err()
    );
    same(
        &line_polygon([0.0, 0.0], [3.0, 4.0], 0.25)
            .unwrap()
            .try_area()
            .unwrap(),
        &scalar("1.25"),
    );
}

#[test]
fn denied_track_import_cannot_disappear_or_become_a_valid_origin() {
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(
        hyperdrc::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            escaped = Some(board("(start 10 20) (end 12 20) (width 2)"));
        },
    );
    assert!(denied.result.is_err());
    let escaped = escaped.unwrap();
    assert_eq!(escaped.copper.len(), 1);
    assert!(escaped.copper[0].region.try_area().is_err());
    assert!(
        escaped.copper[0]
            .region
            .classify_xy(Scalar::zero(), Scalar::zero())
            .is_err()
    );
    let retried = board("(start 10 20) (end 12 20) (width 2)");
    same(
        &retried.copper[0].region.try_area().unwrap(),
        &(Scalar::from(4) + Scalar::pi()),
    );
}

#[test]
fn warmed_stroke_queries_recheck_exhausted_scope() {
    let polygon = line_polygon([0.0, 0.0], [3.0, 4.0], 0.25).unwrap();
    let region = polygons_to_profile(vec![polygon.clone()], None);
    polygon.try_area().unwrap();
    polygon.try_bounds().unwrap();
    region.try_area().unwrap();
    let denied = hypercurve::evaluate_with_budget(
        hyperdrc::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            assert!(polygon.try_area().is_err());
            assert!(polygon.try_bounds().is_err());
            assert!(region.try_area().is_err());
            assert!(region.clone().try_into_native().is_err());
        },
    );
    assert!(denied.result.is_err());
    same(&region.try_area().unwrap(), &scalar("1.25"));
}

#[test]
fn fresh_zero_budget_admits_rational_butt_strokes() {
    let evaluated = hypercurve::evaluate_with_budget(
        hyperdrc::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || line_polygon([1e20, 0.0], [1e20, 16_384.0], 16_384.0).unwrap(),
    );
    assert_eq!(evaluated.usage, hypercurve::WorkUsage::default());
    same(
        &evaluated
            .result
            .unwrap()
            .certified()
            .unwrap()
            .try_area()
            .unwrap(),
        &Scalar::from(268_435_456),
    );
}

#[test]
fn ambient_approximate_policy_does_not_change_stroke_geometry() {
    let result = hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
        line_polygon([0.0, 0.0], [1e-200, 0.0], 1e-200).unwrap()
    })
    .into_value();
    let length = Scalar::try_from(1e-200).unwrap();
    same(&result.try_area().unwrap(), &(&length * &length));
}
