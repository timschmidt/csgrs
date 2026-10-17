//! Malformed track source cannot be published as a successful board.
use hypercurve::RegionPointLocation;
use hyperdrc::Scalar;
use hyperdrc::kicad::{BoardModel, load_kicad_pcb};
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

fn invalid_source(fields: &str) {
    for policy in [
        hyperlimit::PredicatePolicy::STRICT,
        hyperlimit::PredicatePolicy::APPROXIMATE_512,
        hyperlimit::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || board(fields)).into_unverified();
        assert!(result.is_err(), "invalid source published a board: {fields}");
    }
}

macro_rules! malformed {
    ($name:ident,$fields:literal) => {
        #[test]
        fn $name() {
            invalid_source($fields);
        }
    };
}
malformed!(start_missing, " (end 12 20) (width 2)");
malformed!(start_empty, "(start) (end 12 20) (width 2)");
malformed!(start_bad, "(start bad 20) (end 12 20) (width 2)");
malformed!(start_extra, "(start 10 20 30) (end 12 20) (width 2)");
malformed!(
    start_duplicate,
    "(start 10 20) (start 10 20) (end 12 20) (width 2)"
);
malformed!(start_nested, "(start (value 10) 20) (end 12 20) (width 2)");
malformed!(start_nan, "(start NaN 20) (end 12 20) (width 2)");
malformed!(start_infinity, "(start inf 20) (end 12 20) (width 2)");
malformed!(start_short, "(start 10) (end 12 20) (width 2)");
malformed!(start_bad_y, "(start 10 bad) (end 12 20) (width 2)");
malformed!(end_missing, "(start 10 20)  (width 2)");
malformed!(end_empty, "(start 10 20) (end) (width 2)");
malformed!(end_bad, "(start 10 20) (end bad 20) (width 2)");
malformed!(end_extra, "(start 10 20) (end 10 20 30) (width 2)");
malformed!(
    end_duplicate,
    "(start 10 20) (end 12 20) (end 12 20) (width 2)"
);
malformed!(end_nested, "(start 10 20) (end (value 10) 20) (width 2)");
malformed!(end_nan, "(start 10 20) (end NaN 20) (width 2)");
malformed!(end_infinity, "(start 10 20) (end inf 20) (width 2)");
malformed!(end_short, "(start 10 20) (end 10) (width 2)");
malformed!(end_bad_y, "(start 10 20) (end 10 bad) (width 2)");
malformed!(width_missing, "(start 10 20) (end 12 20) ");
malformed!(width_empty, "(start 10 20) (end 12 20) (width)");
malformed!(width_bad, "(start 10 20) (end 12 20) (width bad)");
malformed!(width_extra, "(start 10 20) (end 12 20) (width 2 3)");
malformed!(
    width_duplicate,
    "(start 10 20) (end 12 20) (width 2) (width 2)"
);
malformed!(width_nested, "(start 10 20) (end 12 20) (width (value 10))");
malformed!(width_nan, "(start 10 20) (end 12 20) (width NaN)");
malformed!(width_infinity, "(start 10 20) (end 12 20) (width inf)");
malformed!(width_negative, "(start 10 20) (end 12 20) (width -1)");
malformed!(
    width_negative_subrange,
    "(start 10 20) (end 12 20) (width -1e-400)"
);
malformed!(zero_width_missing_end, "(start 10 20) (width 0)");
malformed!(
    zero_width_invalid_start,
    "(start bad 20) (end 12 20) (width 0)"
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
    assert!(escaped.unwrap().is_err());
    let retried = board("(start 10 20) (end 12 20) (width 2)").unwrap();
    assert_eq!(
        retried.copper[0].region.classify_xy(11.into(), 20.into()),
        Ok(RegionPointLocation::Inside)
    );
}
