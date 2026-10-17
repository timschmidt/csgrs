use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
use hyperdrc::Scalar;
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

#[test]
fn source_strokes_retain_strict_receipts_inside_approximate_and_preview_scopes() {
    let check = || {
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
            hyperlimit::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let model = board("(start 0 0) (end 1 1) (width 2)");
                let profile = model.all_copper().unwrap();
                same(
                    &profile.try_area().unwrap(),
                    &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap() + Scalar::pi()),
                );
                assert_eq!(
                    profile.classify_xy(scalar("-1"), Scalar::zero()),
                    Ok(Boundary)
                );
                assert_eq!(
                    profile.classify_xy(scalar("-1.000000000001"), Scalar::zero()),
                    Ok(Outside)
                );
                let native = profile.try_into_native().unwrap();
                let rebuilt = hypercurve::CurveRegion2::try_from_boundary_paths(
                    &native.boundary_paths().unwrap(),
                    hypercurve::FillRule::NonZero,
                )
                .unwrap();
                same(
                    &rebuilt.filled_area().unwrap().unwrap(),
                    &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap() + Scalar::pi()),
                );
            });
            assert!(outcome.certified().is_some());
        }
    };
    check();
    hypercurve::CurvePreviewOptions::try_approximate_512(1000.0, 1000.0)
        .unwrap()
        .evaluate(check);
}

#[test]
fn fresh_zero_budget_load_retains_native_rational_direction_certificates() {
    let outcome = hypercurve::evaluate_with_budget(
        hyperlimit::PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,
        || board("(start 0 0) (end 3 4) (width 2)"),
    );
    assert_eq!(outcome.usage, hypercurve::WorkUsage::default());
    let model = outcome.result.unwrap().certified().unwrap();
    same(
        &model.copper[0].region.try_area().unwrap(),
        &(Scalar::from(10) + Scalar::pi()),
    );
}
