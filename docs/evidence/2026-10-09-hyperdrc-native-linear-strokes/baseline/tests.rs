use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
use hyperdrc::geometry::{line_polygon, polygons_to_profile};
use hyperdrc::kicad::{BoardModel, CopperKind, load_kicad_pcb};
use hyperdrc::Scalar;
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
    std::fs::write(&path, format!("(kicad_pcb (net 1 signal) (segment (layer F.Cu) (net 1) {fields}))")).unwrap();
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
    assert_eq!(region.classify_xy(&x + Scalar::from(8192), Scalar::one()), Ok(Boundary));
    assert_eq!(region.classify_xy(&x + Scalar::from(8193), Scalar::one()), Ok(Outside));
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
    same(&polygon.try_area().unwrap(), &(Scalar::from(2) * Scalar::try_from(width).unwrap()));
}

#[test]
fn finite_endpoint_difference_can_exceed_float_range() {
    let polygon = line_polygon([-f64::MAX, 0.0], [f64::MAX, 0.0], 1.0).unwrap();
    same(&polygon.try_area().unwrap(), &(Scalar::from(2) * Scalar::try_from(f64::MAX).unwrap()));
}

#[test]
fn finite_length_squaring_cannot_overflow() {
    let polygon = line_polygon([0.0, 0.0], [1e200, 0.0], 2.0).unwrap();
    same(&polygon.try_area().unwrap(), &(Scalar::from(2) * Scalar::try_from(1e200).unwrap()));
}

#[test]
fn pythagorean_diagonal_has_exact_rational_area() {
    let polygon = line_polygon([-1.0, -1.0], [2.0, 3.0], 0.25).unwrap();
    same(&polygon.try_area().unwrap(), &scalar("1.25"));
}

#[test]
fn irrational_diagonal_retains_native_surd_width() {
    let polygon = line_polygon([0.0, 0.0], [1.0, 1.0], 2.0).unwrap();
    same(&polygon.try_area().unwrap(), &(Scalar::from(2) * Scalar::from(2).sqrt().unwrap()));
}

#[test]
fn ordinary_butt_stroke_keeps_its_contract() {
    let polygon = line_polygon([0.0, 0.0], [2.0, 0.0], 0.5).unwrap();
    same(&polygon.try_area().unwrap(), &Scalar::one());
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(region.classify_xy(scalar("-0.1"), Scalar::zero()), Ok(Outside));
    assert_eq!(region.classify_xy(Scalar::zero(), Scalar::zero()), Ok(Boundary));
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
    same(&feature.region.try_area().unwrap(), &(Scalar::from(4) + Scalar::pi()));
    for (x, expected) in [("-1", Boundary), ("-0.5", Inside), ("3", Boundary), ("3.00001", Outside)] {
        assert_eq!(feature.region.classify_xy(scalar(x), Scalar::zero()), Ok(expected));
    }
}

#[test]
fn track_decimal_endpoints_and_width_are_never_rounded_first() {
    let board = board("(start 9007199254740992.1 0) (end 9007199254740992.3 0) (width 0.20000000000000000000004)");
    assert_eq!(board.copper.len(), 1);
    let width = scalar("0.20000000000000000000004");
    let radius = scalar("0.10000000000000000000002");
    same(&board.copper[0].location[0], &scalar("9007199254740992.2"));
    same(&board.copper[0].region.try_area().unwrap(), &(scalar("0.2") * width + Scalar::pi() * &radius * radius));
}

#[test]
fn track_subrange_width_and_length_remain_nonzero() {
    let board = board("(start 0 0) (end 1e-400 0) (width 2e-400)");
    assert_eq!(board.copper.len(), 1);
    let radius = scalar("1e-400");
    same(&board.copper[0].region.try_area().unwrap(), &((Scalar::from(2) + Scalar::pi()) * &radius * radius));
}

#[test]
fn track_superrange_coordinates_stay_native() {
    let board = board("(start 1e400 0) (end 1e400 2) (width 2)");
    assert_eq!(board.copper.len(), 1);
    same(&board.copper[0].region.try_area().unwrap(), &(Scalar::from(4) + Scalar::pi()));
    same(&board.copper[0].location[0], &scalar("1e400"));
}

#[test]
fn coincident_track_endpoints_keep_the_width_disk() {
    let board = board("(start 3 4) (end 3 4) (width 2)");
    assert_eq!(board.copper.len(), 1);
    same(&board.copper[0].region.try_area().unwrap(), &Scalar::pi());
    assert_eq!(board.copper[0].region.classify_xy(Scalar::from(4), Scalar::from(4)), Ok(Boundary));
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
        assert!(board.copper[0].region.classify_xy(Scalar::zero(), Scalar::zero()).is_err(), "{fields}");
    }
}
