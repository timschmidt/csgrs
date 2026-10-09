use hypercurve::RegionPointLocation;
use hyperdrc::Scalar;
use hyperdrc::geometry::{SourceScalar, SourceUnit};
use hyperdrc::kicad::{BoardModel, load_kicad_pcb};
use std::sync::atomic::{AtomicU64, Ordering};

fn scalar(token: &str) -> Scalar {
    SourceScalar::parse(SourceUnit::KiCadMillimeter, token)
        .unwrap()
        .value
}

fn same(actual: &Scalar, expected: &Scalar) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal),
        "actual={actual:?}, expected={expected:?}"
    );
}

fn bounds(region: &hyperdrc::PcbRegion) -> [Scalar; 4] {
    let native = region.clone().try_into_native().unwrap();
    let bounds = native.bounds().unwrap().unwrap();
    [
        bounds.min_x().clone(),
        bounds.min_y().clone(),
        bounds.max_x().clone(),
        bounds.max_y().clone(),
    ]
}

fn board(footprint_at: &str, shape: &str, details: &str) -> BoardModel {
    static SEQUENCE: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "hyperdrc-pad-source-{}-{}.kicad_pcb",
        std::process::id(),
        SEQUENCE.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(
        &path,
        format!("(kicad_pcb (footprint fixture {footprint_at} (pad 1 thru_hole {shape} (layers F.Cu) {details})))"),
    ).unwrap();
    let result = load_kicad_pcb(&path);
    std::fs::remove_file(path).unwrap();
    result.unwrap()
}

fn rectangle_case(shape: &str, extra: &str) {
    let board = board(
        "",
        shape,
        &format!(
            "(at 9007199254740992.1 -0.00000000000000000000001) (size 0.20000000000000000000004 0.4) {extra}"
        ),
    );
    assert_eq!(board.copper.len(), 1);
    let feature = &board.copper[0];
    let center = [scalar("9007199254740992.1"), scalar("-1e-23")];
    let radius = scalar("0.10000000000000000000002");
    assert_eq!(feature.location, center);
    let bounds = bounds(&feature.region);
    same(&bounds[0], &(&center[0] - &radius));
    same(&bounds[2], &(&center[0] + &radius));
    assert_eq!(
        feature
            .region
            .classify_xy(&center[0] + &radius, center[1].clone()),
        Ok(RegionPointLocation::Boundary)
    );
}

#[test]
fn decimal_rectangle_geometry_agrees_with_its_exact_marker() {
    rectangle_case("rect", "");
}

#[test]
fn decimal_roundrect_geometry_agrees_with_its_exact_marker() {
    rectangle_case("roundrect", "(roundrect_rratio 0.25)");
}

#[test]
fn decimal_trapezoid_geometry_agrees_with_its_exact_marker() {
    rectangle_case("trapezoid", "(rect_delta 0 0)");
}

#[test]
fn decimal_circle_diameter_is_halved_after_exact_parsing() {
    let board = board(
        "",
        "circle",
        "(at 9007199254740992.1 0) (size 0.20000000000000000000004 0.20000000000000000000004)",
    );
    assert_eq!(board.copper.len(), 1);
    let center = scalar("9007199254740992.1");
    let radius = scalar("0.10000000000000000000002");
    let bounds = bounds(&board.copper[0].region);
    same(&bounds[0], &(&center - &radius));
    same(&bounds[2], &(&center + &radius));
}

#[test]
fn decimal_oval_keeps_the_straight_body_and_both_cap_centers() {
    let board = board(
        "",
        "oval",
        "(at 9007199254740992.1 0) (size 0.60000000000000000000004 0.2)",
    );
    assert_eq!(board.copper.len(), 3);
    let center = scalar("9007199254740992.1");
    let half_length = scalar("0.30000000000000000000002");
    let mut union = board.copper[0].region.clone();
    for feature in &board.copper[1..] {
        union = union.try_union(&feature.region).unwrap();
    }
    let bounds = bounds(&union);
    same(&bounds[0], &(&center - &half_length));
    same(&bounds[2], &(&center + &half_length));
}

#[test]
fn roundrect_ratio_product_keeps_source_digits() {
    let ratio = scalar("0.10000000000000000000001");
    let radius = Scalar::from(2) * ratio;
    let board = board(
        "",
        "roundrect",
        "(size 2 4) (roundrect_rratio 0.10000000000000000000001)",
    );
    let expected = Scalar::from(8) - (Scalar::from(4) - Scalar::pi()) * &radius * radius;
    same(&board.copper[0].region.try_area().unwrap(), &expected);
}

#[test]
fn chamfer_ratio_product_keeps_source_digits() {
    let distance = scalar("0.20000000000000000000002");
    let board = board(
        "",
        "roundrect",
        "(size 2 4) (roundrect_rratio 0) (chamfer_ratio 0.10000000000000000000001) (chamfer top_left top_right bottom_right bottom_left)",
    );
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::from(8) - Scalar::from(2) * &distance * distance),
    );
}

#[test]
fn selected_chamfer_keeps_rounding_on_the_other_three_corners() {
    let board = board(
        "",
        "roundrect",
        "(size 2 4) (roundrect_rratio 0.25) (chamfer_ratio 0.125) (chamfer top_left)",
    );
    let quarter = scalar("0.25");
    let rounded_loss = Scalar::from(3) * (Scalar::one() - &quarter * Scalar::pi()) * &quarter;
    let chamfer_loss = scalar("0.03125");
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::from(8) - rounded_loss - chamfer_loss),
    );
}

#[test]
fn footprint_rotation_uses_kicad_coordinate_handedness() {
    let board = board("(at 10.1 20.2 90)", "rect", "(at 2.3 0 0) (size 2 4)");
    let feature = &board.copper[0];
    assert_eq!(feature.location, [scalar("10.1"), scalar("17.9")]);
    assert_eq!(
        feature.region.classify_xy(scalar("10.1"), scalar("17.9")),
        Ok(RegionPointLocation::Inside)
    );
}

#[test]
fn pad_orientation_is_absolute_in_the_board_frame() {
    let board = board("(at 0 0 90)", "rect", "(at 0 0 0) (size 2 4)");
    let bounds = bounds(&board.copper[0].region);
    assert_eq!(bounds, [-1, -2, 1, 2].map(Scalar::from));
}

#[test]
fn omitted_pad_orientation_is_axis_aligned_in_the_board_frame() {
    let board = board("(at 0 0 90)", "rect", "(at 0 0) (size 2 4)");
    let bounds = bounds(&board.copper[0].region);
    assert_eq!(bounds, [-1, -2, 1, 2].map(Scalar::from));
}

#[test]
fn pad_rotation_uses_kicad_handedness_without_adding_footprint_angle() {
    let board = board(
        "(at 0 0 90)",
        "trapezoid",
        "(at 0 0 90) (size 2 4) (rect_delta 0 1)",
    );
    assert_eq!(
        board.copper[0].region.classify_xy(2.into(), scalar("1.5")),
        Ok(RegionPointLocation::Boundary)
    );
}

#[test]
fn drill_offset_uses_absolute_pad_rotation() {
    let board = board(
        "(at 10 20 90)",
        "rect",
        "(at 2 0 90) (size 4 4) (drill 1 (offset 1 0))",
    );
    assert_eq!(board.drills.len(), 1);
    assert_eq!(board.drills[0].location, [10, 17].map(Scalar::from));
}

#[test]
fn nonzero_dimensions_below_binary64_range_remain_native() {
    for shape in ["rect", "circle", "oval", "trapezoid", "roundrect"] {
        let board = board("", shape, "(size 1e-400 1e-400)");
        assert_eq!(board.copper.len(), 1, "{shape}");
        assert_eq!(board.copper[0].region.try_is_empty(), Ok(false), "{shape}");
        assert_eq!(
            board.copper[0]
                .region
                .classify_xy(Scalar::zero(), Scalar::zero()),
            Ok(RegionPointLocation::Inside),
            "{shape}"
        );
    }
}

#[test]
fn overflowing_finite_view_does_not_erase_exact_source_geometry() {
    let board = board("(at 1e400 0)", "rect", "(size 2 4)");
    assert_eq!(board.copper[0].location, [scalar("1e400"), Scalar::zero()]);
    same(
        &board.copper[0].region.try_area().unwrap(),
        &Scalar::from(8),
    );
    assert_eq!(
        board.copper[0]
            .region
            .classify_xy(scalar("1e400"), Scalar::zero()),
        Ok(RegionPointLocation::Inside)
    );
}

fn fails(shape: &str, details: &str, expected_field: &str) {
    let board = board("", shape, details);
    assert_eq!(board.copper.len(), 1);
    let error = board.copper[0].region.try_area().unwrap_err();
    assert!(error.detail.contains(expected_field), "{error}");
    assert!(board.copper[0].region.try_is_empty().is_err());
}

#[test]
fn malformed_rounding_ratio_does_not_select_the_default() {
    for ratio in ["", "oops", "NaN", "inf", "(bad)"] {
        fails(
            "roundrect",
            &format!("(size 2 4) (roundrect_rratio {ratio})"),
            "roundrect_rratio",
        );
    }
}

#[test]
fn malformed_chamfer_ratio_does_not_select_a_rounded_rectangle() {
    for ratio in ["", "oops", "NaN", "inf", "(bad)"] {
        fails(
            "roundrect",
            &format!("(size 2 4) (chamfer_ratio {ratio}) (chamfer top_left)"),
            "chamfer_ratio",
        );
    }
}

#[test]
fn malformed_delta_does_not_select_an_ordinary_rectangle() {
    for delta in ["", "1", "oops 0", "NaN 0", "(bad) 0"] {
        fails(
            "trapezoid",
            &format!("(size 2 4) (rect_delta {delta})"),
            "rect_delta",
        );
    }
}

#[test]
fn malformed_pad_position_does_not_select_the_origin() {
    for position in ["", "1", "oops 0", "NaN 0", "(bad) 0"] {
        fails("rect", &format!("(size 2 4) (at {position})"), "pad at");
    }
}

#[test]
fn malformed_pad_angle_does_not_select_zero_rotation() {
    for angle in ["oops", "NaN", "inf", "(bad)"] {
        fails("rect", &format!("(size 2 4) (at 0 0 {angle})"), "pad at");
    }
}

#[test]
fn malformed_footprint_position_is_a_retained_source_failure() {
    for at in ["(at)", "(at 1)", "(at oops 0)", "(at 0 0 oops)"] {
        let board = board(at, "rect", "(size 2 4)");
        assert_eq!(board.copper.len(), 1);
        assert!(
            board.copper[0]
                .region
                .try_area()
                .unwrap_err()
                .detail
                .contains("footprint at")
        );
    }
}

#[test]
fn absent_optional_parameters_keep_the_documented_defaults() {
    let rect = board("", "rect", "(size 2 4)");
    same(&rect.copper[0].region.try_area().unwrap(), &8.into());
    let rounded = board("", "roundrect", "(size 2 4)");
    same(
        &rounded.copper[0].region.try_area().unwrap(),
        &(Scalar::from(7) + scalar("0.25") * Scalar::pi()),
    );
    let trapezoid = board("", "trapezoid", "(size 2 4)");
    same(&trapezoid.copper[0].region.try_area().unwrap(), &8.into());
}

#[test]
fn an_unrepresentable_required_width_marker_is_uncertain() {
    let board = board("(at 1e400 0)", "rect", "(size 2 4)");
    let findings = hyperdrc::checks::copper_width_readiness(&board, &[], &Scalar::from(3));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
}

#[test]
fn an_unrepresentable_required_net_marker_is_uncertain() {
    let board = board("(at 1e400 0)", "rect", "(size 2 4)");
    let findings = hyperdrc::checks::copper_net_intent(&board, &[]);
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
}

#[test]
fn a_failed_source_cannot_supply_a_fabricated_origin_marker() {
    let board = board("(at oops 0)", "rect", "(size 2 4)");
    let findings = hyperdrc::checks::copper_net_intent(&board, &[]);
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
}

#[test]
fn every_chamfer_selection_keeps_unselected_round_corners_after_rotation() {
    let corners = ["top_left", "top_right", "bottom_right", "bottom_left"];
    for mask in 0_u8..16 {
        let selected = corners
            .iter()
            .enumerate()
            .filter_map(|(i, name)| ((mask & (1 << i)) != 0).then_some(*name))
            .collect::<Vec<_>>()
            .join(" ");
        for angle in [0, 90] {
            let board = board(
                "",
                "roundrect",
                &format!(
                    "(at 0 0 {angle}) (size 2 4) (roundrect_rratio 0.25) (chamfer_ratio 0.125) (chamfer {selected})"
                ),
            );
            let count = Scalar::from(mask.count_ones());
            let expected = Scalar::from(8)
                - (Scalar::from(4) - &count)
                    * (Scalar::one() - scalar("0.25") * Scalar::pi())
                    * scalar("0.25")
                - count * scalar("0.03125");
            same(&board.copper[0].region.try_area().unwrap(), &expected);
            for (i, [x_sign, y_sign]) in
                [[-1, -1], [1, -1], [1, 1], [-1, 1]].into_iter().enumerate()
            {
                let x = scalar("0.875") * Scalar::from(x_sign);
                let y = scalar("1.875") * Scalar::from(y_sign);
                let [x, y] = if angle == 90 { [y, -x] } else { [x, y] };
                let expected = if mask & (1 << i) == 0 {
                    RegionPointLocation::Outside
                } else {
                    RegionPointLocation::Boundary
                };
                assert_eq!(
                    board.copper[0].region.classify_xy(x, y),
                    Ok(expected),
                    "mask={mask}, angle={angle}, corner={i}"
                );
            }
        }
    }
}

#[test]
fn maximum_rounding_and_chamfer_distances_keep_degenerate_straight_edges_valid() {
    for extra in ["", "(chamfer_ratio 0.5) (chamfer top_left bottom_right)"] {
        let board = board(
            "",
            "roundrect",
            &format!("(size 2 2) (roundrect_rratio 0.5) {extra}"),
        );
        let expected = if extra.is_empty() {
            Scalar::pi()
        } else {
            Scalar::one() + scalar("0.5") * Scalar::pi()
        };
        same(&board.copper[0].region.try_area().unwrap(), &expected);
        assert_eq!(
            board.copper[0].region.classify_xy(0.into(), 0.into()),
            Ok(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn zero_chamfer_ratio_preserves_rounding_even_with_selected_corners() {
    let board = board(
        "",
        "roundrect",
        "(size 2 4) (chamfer_ratio 0) (chamfer top_left bottom_right)",
    );
    same(
        &board.copper[0].region.try_area().unwrap(),
        &(Scalar::from(7) + scalar("0.25") * Scalar::pi()),
    );
}

#[test]
fn custom_pad_placement_keeps_an_unrepresentable_exact_center() {
    let board = board(
        "(at 1e400 0 90)",
        "custom",
        "(at 2 0 90) (primitives (gr_poly (pts (xy -1 -2) (xy 1 -2) (xy 1 2) (xy -1 2)) (width 0) (fill yes)))",
    );
    let region = &board.copper[0].region;
    same(&region.try_area().unwrap(), &Scalar::from(8));
    assert_eq!(
        board.copper[0].location,
        [scalar("1e400"), Scalar::from(-2)]
    );
    assert_eq!(
        region.classify_xy(scalar("1e400") + Scalar::from(2), Scalar::from(-2)),
        Ok(RegionPointLocation::Boundary)
    );
}

#[test]
fn an_unrepresentable_required_drill_marker_is_uncertain() {
    let mut board = board("(at 1e400 0)", "circle", "(size 2 2) (drill 1)");
    assert_eq!(board.drills.len(), 1);
    board.drills[0].plated = false;
    let findings = hyperdrc::checks::routed_slot_readiness(&board, &Scalar::from(2));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
}

#[test]
fn a_failed_shape_cannot_supply_an_ordinary_net_marker() {
    let board = board("", "roundrect", "(size 2 4) (roundrect_rratio oops)");
    let findings = hyperdrc::checks::copper_net_intent(&board, &[]);
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
}

#[test]
fn exhausted_marker_projection_retains_uncertainty_and_can_retry() {
    let board = board("", "rect", "(size 2 4)");
    let denied = hypercurve::evaluate_with_budget(
        hyperlimit::PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,
        || {
            let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                [-36, 1368, -11034, 31728, -38280, 16620]
                    .into_iter()
                    .map(Scalar::from)
                    .collect(),
            )
            .unwrap();
            assert!(polynomial.isolate_unit_interval_roots().is_err());
            let findings = hyperdrc::checks::copper_net_intent(&board, &[]);
            assert_eq!(findings.len(), 1);
            assert_eq!(findings[0].check, "geometry-uncertainty");
        },
    );
    assert!(denied.result.is_err());
    assert_eq!(
        hyperdrc::checks::copper_net_intent(&board, &[])[0].check,
        "copper-net-intent"
    );
}
