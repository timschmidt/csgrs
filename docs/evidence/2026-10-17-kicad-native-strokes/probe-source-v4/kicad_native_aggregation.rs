use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
use hypercurve::{CurveRegion2, RegionPointLocation};
use hyperdrc::geometry::{
    LineString, Polygon, empty_profile, polygon_from_points, polygon_to_profile,
};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, load_kicad_pcb};
use hyperdrc::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar};
use hyperlimit::PredicatePolicy;
use std::sync::atomic::{AtomicU64, Ordering};

fn s(token: &str) -> Scalar {
    token.parse().unwrap()
}
fn same(actual: &Scalar, expected: &Scalar) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}
fn at(region: &PcbRegion, x: &str, y: &str, expected: RegionPointLocation) {
    assert_eq!(region.classify_xy(s(x), s(y)), Ok(expected));
}
fn square(x: f64, y: f64, width: f64) -> PcbRegion {
    polygon_to_profile(
        polygon_from_points(vec![
            [x, y],
            [x + width, y],
            [x + width, y + width],
            [x, y + width],
        ]),
        None,
    )
}
fn feature(layer: &str, region: PcbRegion) -> CopperFeature {
    CopperFeature {
        layer: layer.into(),
        net: Some("signal".into()),
        kind: CopperKind::Segment,
        region,
        location: [Scalar::zero(), Scalar::zero()],
    }
}
fn model(copper: Vec<CopperFeature>) -> BoardModel {
    BoardModel {
        source: "aggregation-contract".into(),
        copper,
        drills: vec![],
        board_outline: None,
        panel_features: None,
    }
}
fn profiles(board: &BoardModel) -> [PcbRegion; 2] {
    let mut layers = board.copper_layers(&[]).unwrap();
    assert_eq!(layers.len(), 1);
    assert_eq!(layers[0].0, "F.Cu");
    let layer = layers.pop().unwrap().1;
    assert_eq!(layer.metadata().as_ref().unwrap().name, "KiCad F.Cu");
    let all = board.all_copper().unwrap();
    assert_eq!(all.metadata().as_ref().unwrap().name, "KiCad copper");
    [all, layer]
}
fn source(fields: &str) -> BoardModel {
    static SEQUENCE: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "native-aggregation-{}-{}.kicad_pcb",
        std::process::id(),
        SEQUENCE.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(&path, format!("(kicad_pcb (segment (layer F.Cu) {fields}))")).unwrap();
    let loaded = load_kicad_pcb(&path);
    std::fs::remove_file(path).unwrap();
    loaded.unwrap()
}

#[test]
fn subrange_stroke_composes_without_a_finite_report_ring() {
    let board = source("(start 0 0) (end 2 0) (width 1e-400)");
    assert!(board.copper[0].region.try_to_multipolygon().is_err());
    let radius = s("5e-401");
    for region in profiles(&board) {
        same(
            &region.try_area().unwrap(),
            &(s("2e-400") + Scalar::pi() * &radius * &radius),
        );
        at(&region, "1", "0", Inside);
        at(&region, "0", "5e-401", Boundary);
        at(&region, "1", "1e-400", Outside);
        let empty = region.try_difference(&region).unwrap();
        assert_eq!(empty.try_is_empty(), Ok(true));
        same(
            &empty.try_union(&region).unwrap().try_area().unwrap(),
            &(s("2e-400") + Scalar::pi() * &radius * &radius),
        );
        assert!(region.try_to_multipolygon().is_err());
    }
}

#[test]
fn superrange_stroke_survives_empty_accumulation_and_native_handoff() {
    let mut board = source("(start 1e400 0) (end 1e400 2) (width 2)");
    assert!(board.copper[0].region.try_to_multipolygon().is_err());
    board.copper.insert(0, feature("F.Cu", empty_profile(None)));
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &(Scalar::from(4) + Scalar::pi()));
        at(&region, "1e400", "1", Inside);
        assert_eq!(
            region.classify_xy(s("1e400") - Scalar::one(), Scalar::one()),
            Ok(Boundary)
        );
        let native = region.try_into_native().unwrap();
        same(
            &native.filled_area().unwrap().unwrap(),
            &(Scalar::from(4) + Scalar::pi()),
        );
    }
}

#[test]
fn canonical_empty_and_complete_subtraction_remain_composable() {
    let mut board = model(vec![]);
    assert!(board.copper_layers(&[]).unwrap().is_empty());
    let empty = board.all_copper().unwrap();
    assert_eq!(empty.try_is_empty(), Ok(true));
    board.copper.push(feature("F.Cu", empty));
    board.copper.push(feature("F.Cu", square(0.0, 0.0, 2.0)));
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(4));
        let erased = region.try_difference(&square(0.0, 0.0, 2.0)).unwrap();
        let next = model(vec![feature("F.Cu", erased)]);
        for output in profiles(&next) {
            assert_eq!(output.try_is_empty(), Ok(true));
            assert_eq!(
                output.try_intersection(&region).unwrap().try_is_empty(),
                Ok(true)
            );
            same(
                &output.try_union(&region).unwrap().try_area().unwrap(),
                &Scalar::from(4),
            );
        }
    }
}

#[test]
fn holes_overlaps_and_disconnected_material_keep_set_semantics() {
    let frame = square(0.0, 0.0, 4.0)
        .try_difference(&square(1.0, 1.0, 2.0))
        .unwrap();
    let mut board = model(vec![
        feature("F.Cu", frame),
        feature("F.Cu", square(6.0, 0.0, 1.0)),
    ]);
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(13));
        at(&region, "2", "2", Outside);
        at(&region, "0.5", "0.5", Inside);
        at(&region, "6.5", "0.5", Inside);
    }
    board.copper.push(feature("F.Cu", square(1.0, 1.0, 2.0)));
    board.copper.push(feature("F.Cu", square(0.0, 0.0, 4.0)));
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(17));
        at(&region, "2", "2", Inside);
        at(&region, "1", "2", Inside);
        at(&region, "4", "2", Boundary);
        at(&region, "5", "2", Outside);
        assert_eq!(
            region.try_difference(&region).unwrap().try_is_empty(),
            Ok(true)
        );
    }
}

#[test]
fn malformed_input_cannot_hide_behind_empty_or_layer_aggregation() {
    for reversed in [false, true] {
        let invalid = polygon_to_profile(
            Polygon::new(LineString(vec![]), vec![]),
            Some(LayerMetadata {
                name: "invalid authored feature".into(),
            }),
        );
        let mut inputs = vec![
            feature("F.Cu", empty_profile(None)),
            feature("F.Cu", invalid),
        ];
        if reversed {
            inputs.reverse();
        }
        let board = model(inputs);
        for error in [
            board.all_copper().unwrap_err(),
            board.copper_layers(&[]).unwrap_err(),
        ] {
            assert_eq!(error.source.as_deref(), Some("invalid authored feature"));
            assert!(error.detail.contains("fewer than three"));
        }
        assert!(board.copper_layers(&["B.Cu".into()]).unwrap().is_empty());
    }
}

#[test]
fn layer_selection_metadata_and_mutation_do_not_reuse_stale_geometry() {
    let mut board = model(vec![
        feature("F.Cu", square(0.0, 0.0, 2.0)),
        feature("B.Cu", square(4.0, 0.0, 1.0)),
    ]);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        assert!(
            hypercurve::evaluate_under(policy, || {
                let layers = board.copper_layers(&["F.Cu".into(), "B.Cu".into()]).unwrap();
                assert_eq!(
                    layers
                        .iter()
                        .map(|(name, _)| name.as_str())
                        .collect::<Vec<_>>(),
                    ["B.Cu", "F.Cu"]
                );
                same(&layers[0].1.try_area().unwrap(), &Scalar::one());
                same(&layers[1].1.try_area().unwrap(), &Scalar::from(4));
                same(
                    &board.all_copper().unwrap().try_area().unwrap(),
                    &Scalar::from(5),
                );
                let selected = board.copper_layers(&["F.Cu".into()]).unwrap();
                same(&selected[0].1.try_area().unwrap(), &Scalar::from(4));
                selected[0].1.try_to_multipolygon().unwrap();
            })
            .certified()
            .is_some()
        );
    }
    let previous = board.all_copper().unwrap();
    previous.try_to_multipolygon().unwrap();
    board.copper[0].region.replace_native(CurveRegion2::empty());
    same(
        &board.all_copper().unwrap().try_area().unwrap(),
        &Scalar::one(),
    );
    same(&previous.try_area().unwrap(), &Scalar::from(5));
}

#[test]
fn native_quadratic_and_linear_components_share_aggregation() {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2,
    };
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 2),
            Point2::from_values(2, 0),
        )),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(0, 0)).unwrap(),
        ),
    ])
    .unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    assert!(native.native_contours_fast_path().unwrap().is_none());
    let board = model(vec![
        feature("F.Cu", PcbRegion::new(native, None)),
        feature("F.Cu", square(5.0, 0.0, 1.0)),
    ]);
    for region in profiles(&board) {
        // y = 2x - x² on [0, 2] has area 4/3; the disjoint square adds 1.
        same(
            &region.try_area().unwrap(),
            &(Scalar::from(7) / Scalar::from(3)).unwrap(),
        );
        at(&region, "1", "0.5", Inside);
        at(&region, "1", "1.5", Outside);
        at(&region, "5.5", "0.5", Inside);
        assert_eq!(
            region.try_difference(&region).unwrap().try_is_empty(),
            Ok(true)
        );
    }
}

#[test]
fn warmed_aggregation_cannot_escape_exhaustion_and_retry_recovers() {
    let board = model(vec![feature("F.Cu", square(0.0, 0.0, 2.0))]);
    profiles(&board).into_iter().for_each(|r| {
        r.try_to_multipolygon().unwrap();
    });
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(
        PredicatePolicy::STRICT,
        hypercurve::WorkBudget::ZERO,
        || {
            let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                [-36, 1368, -11034, 31728, -38280, 16620]
                    .into_iter()
                    .map(Scalar::from)
                    .collect(),
            )
            .unwrap();
            assert!(matches!(polynomial.isolate_unit_interval_roots(),
            Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted));
            escaped = Some((board.all_copper(), board.copper_layers(&[])));
        },
    );
    assert!(denied.result.is_err());
    let (all, layers) = escaped.unwrap();
    assert!(all.unwrap_err().detail.contains("budget"));
    assert!(layers.unwrap_err().detail.contains("budget"));
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(4));
    }
}

#[test]
fn aggregation_preserves_source_nonfinite_history_after_exact_intersection() {
    let circle = polygon_to_profile(
        hyperdrc::geometry::circle_polygon([1e308, 0.0], 1e308, 16), None);
    let clipped = circle.try_intersection(&square(1.0, 0.0, 1.0)).unwrap();
    same(&clipped.try_area().unwrap(), &Scalar::one());
    let diagnosed = |region: &PcbRegion| {
        hyperdrc::checks::layer::layer_sanity("F.Cu", region, None).iter().any(|v|
            v.message.as_deref().is_some_and(|s| s.contains("non-finite coordinates")))
    };
    assert!(diagnosed(&clipped), "the source-history diagnostic is a precondition");
    let board = model(vec![feature("F.Cu", clipped)]);
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::one());
        assert!(diagnosed(&region), "aggregation must retain source history");
    }
}

#[test]
fn authored_report_decomposition_is_separate_from_exact_union_topology() {
    let a = square(0.0, 0.0, 2.0);
    let b = square(1.0, 0.0, 2.0);
    let mut expected = a.try_to_multipolygon().unwrap().0;
    expected.extend(b.try_to_multipolygon().unwrap().0);
    let board = model(vec![feature("F.Cu", a), feature("F.Cu", b)]);
    for region in profiles(&board) {
        // Input rectangles overlap in a 1 by 2 strip: exact union area is 6.
        same(&region.try_area().unwrap(), &Scalar::from(6));
        at(&region, "1.5", "1", Inside);
        let report = region.try_to_multipolygon().unwrap();
        assert_eq!(report.0.len(), 2, "authored reports are not canonical components");
        assert_eq!(report.0, expected);
        let native = region.try_into_native().unwrap();
        assert_eq!(native.material_components().unwrap().len(), 1);
    }
}

#[test]
fn failed_report_prefix_is_not_cached_and_native_composition_recovers() {
    let mut board = source("(start 0 0) (end 2 0) (width 1e-400)");
    board.copper.insert(0, feature("F.Cu", square(4.0, 0.0, 1.0)));
    for region in profiles(&board) {
        let radius = s("5e-401");
        let area = Scalar::one() + s("2e-400") + Scalar::pi() * &radius * &radius;
        for _ in 0..2 {
            assert!(region.try_to_multipolygon().is_err(), "no successful finite prefix may escape");
            same(&region.try_area().unwrap(), &area);
            at(&region, "1", "0", Inside);
            at(&region, "4.5", "0.5", Inside);
        }
    }
    board.copper[1].region.replace_native(square(0.0, 0.0, 1.0).try_into_native().unwrap());
    for region in profiles(&board) {
        same(&region.try_area().unwrap(), &Scalar::from(2));
        assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
    }
}

#[test]
fn report_sources_are_owned_scope_checked_and_isolated_from_mutation() {
    let mut board = model(vec![feature("F.Cu", square(0.0, 0.0, 1.0)), feature("F.Cu", square(3.0, 0.0, 1.0))]);
    let mut region = board.all_copper().unwrap();
    board.copper.clear();
    let original = region.clone();
    let threads = (0..2).map(|_| {
        let owned = region.clone();
        std::thread::spawn(move || {
            let mut view = owned.try_to_multipolygon().unwrap();
            assert_eq!(view.0.len(), 2);
            view.0.clear();
            assert_eq!(owned.try_to_multipolygon().unwrap().0.len(), 2);
            same(&owned.try_area().unwrap(), &Scalar::from(2));
        })
    }).collect::<Vec<_>>();
    for thread in threads { thread.join().unwrap(); }
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512, PredicatePolicy::STRICT] {
        assert!(hypercurve::evaluate_under(policy, || {
            assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
            same(&region.try_area().unwrap(), &Scalar::from(2));
        }).certified().is_some());
    }
    let mut escaped = None;
    let denied = hypercurve::evaluate_with_budget(PredicatePolicy::STRICT, hypercurve::WorkBudget::ZERO, || {
        let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect()).unwrap();
        assert!(polynomial.isolate_unit_interval_roots().is_err());
        escaped = Some(region.try_to_multipolygon());
    });
    assert!(denied.result.is_err());
    assert!(escaped.unwrap().unwrap_err().detail.contains("budget"));
    assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 2);
    region.replace_native(CurveRegion2::empty());
    assert!(region.try_to_multipolygon().unwrap().0.is_empty());
    same(&region.try_area().unwrap(), &Scalar::zero());
    assert_eq!(original.try_to_multipolygon().unwrap().0.len(), 2);
    same(&original.try_area().unwrap(), &Scalar::from(2));
}
