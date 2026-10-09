mod affine_probe;
mod native_drill_probe;
mod ipc_probe;
mod layer_order_probe;
mod point_consumer_probe;
mod authored_center_probe;

use hyperdrc::geometry::{circle_polygon, polygons_to_profile, rect_polygon};
use hyperdrc::{PcbRegion, PcbRegionExt, Scalar};
use std::hint::black_box;
use std::time::Instant;

fn witness(region: &PcbRegion, expected_area: &Scalar) -> String {
    let area = hypercurve::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
        region.try_area().unwrap()
    })
    .certified()
    .unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&area, expected_area, hyperlimit::PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert!(region.try_bounding_box().unwrap().is_some());
    let view = region.try_to_multipolygon().unwrap();
    let mut points = 0;
    let mut hash = 0xcbf29ce484222325_u64;
    for polygon in &view.0 {
        for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
            for coordinate in &ring.0 {
                points += 1;
                for value in [coordinate.x, coordinate.y] {
                    for byte in value.to_bits().to_le_bytes() {
                        hash ^= u64::from(byte);
                        hash = hash.wrapping_mul(0x100000001b3);
                    }
                }
            }
        }
    }
    format!("{}:{points}:{hash}", view.0.len())
}

fn measure_region(label: &str, expected_area: Scalar, mut query: impl FnMut() -> PcbRegion) {
    let expected = witness(&query(), &expected_area);
    let iterations = 100;
    let started = Instant::now();
    for _ in 0..iterations {
        let region = query();
        black_box(region.try_bounding_box().unwrap());
        black_box(region);
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(witness(&query(), &expected_area), expected);
    println!("operation_{label}\t{iterations}\t{elapsed}\t{expected}");
}

fn quadratic() -> hypercurve::CurveRegion2 {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2,
        QuadraticBezier2,
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
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap()
}

fn main() {
    // Keep the original fourteen witnesses first for byte-for-byte replay.
    let left = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)], None);
    let right = polygons_to_profile(vec![rect_polygon([1.0, 0.0], [2.0, 2.0], 0.0)], None);
    measure_region("union_and_bounds", 6.into(), || {
        left.try_union(&right).unwrap()
    });
    measure_region("intersection_and_bounds", 2.into(), || {
        left.try_intersection(&right).unwrap()
    });
    measure_region("difference_and_bounds", 2.into(), || {
        left.try_difference(&right).unwrap()
    });
    measure_region("xor_and_bounds", 4.into(), || left.try_xor(&right).unwrap());
    let authored = polygons_to_profile(vec![circle_polygon([0.0, 0.0], 1.0, 17)], None);
    let circle = PcbRegion::new(authored.try_into_native().unwrap(), None);
    measure_region(
        "circle_offset_and_bounds",
        Scalar::from(4) * Scalar::pi(),
        || circle.offset(Scalar::one()).unwrap(),
    );
    let drill = hyperdrc::kicad::DrillFeature {
        location: [Scalar::zero(), Scalar::zero()],
        diameter: 2.into(),
        plated: false,
        net: None,
    };
    measure_region("drill_circle_and_bounds", Scalar::pi(), || {
        hyperdrc::checks::drill::drills_to_region(
            std::slice::from_ref(&drill),
            "probe",
            "operation-cost",
        )
        .unwrap()
    });
    let point = [Scalar::zero(), Scalar::zero()];
    let query = || {
        circle
            .classify_xy(point[0].clone(), point[1].clone())
            .unwrap()
    };
    assert_eq!(query(), hypercurve::RegionPointLocation::Inside);
    let iterations = 1_000;
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(query(), hypercurve::RegionPointLocation::Inside);
    println!("operation_classification\t{iterations}\t{elapsed}\tinside");
    let query = || circle.try_bounding_box().unwrap().unwrap();
    let expected = query();
    assert_eq!(expected.mins.x, Scalar::from(-1));
    assert_eq!(expected.mins.y, Scalar::from(-1));
    assert_eq!(expected.maxs.x, Scalar::one());
    assert_eq!(expected.maxs.y, Scalar::one());
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(query(), expected);
    println!("operation_bounds\t{iterations}\t{elapsed}\t-1,-1,1,1");

    let cap = quadratic();
    let cap_polygon = PcbRegion::new(cap.clone(), None)
        .try_to_multipolygon()
        .unwrap()
        .0
        .remove(0);
    measure_region(
        "single_quadratic_aggregation_and_bounds",
        (Scalar::from(4) / Scalar::from(3)).unwrap(),
        || polygons_to_profile(vec![cap_polygon.clone()], None),
    );
    let disjoint = rect_polygon([5.0, 0.0], [1.0, 1.0], 0.0);
    measure_region(
        "mixed_quadratic_aggregation_and_bounds",
        (Scalar::from(7) / Scalar::from(3)).unwrap(),
        || polygons_to_profile(vec![cap_polygon.clone(), disjoint.clone()], None),
    );
    measure_region(
        "replacement_and_bounds",
        (Scalar::from(4) / Scalar::from(3)).unwrap(),
        || {
            let mut replaced = left.clone();
            replaced.replace_native(cap.clone());
            replaced
        },
    );
    assert_eq!(left.try_area().unwrap(), Scalar::from(4));
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(left.try_area().unwrap());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(left.try_area().unwrap(), Scalar::from(4));
    println!("operation_area\t{iterations}\t{elapsed}\t4");
    assert!(!left.try_is_empty().unwrap());
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(left.try_is_empty().unwrap());
    }
    let elapsed = started.elapsed().as_nanos();
    assert!(!left.try_is_empty().unwrap());
    println!("operation_empty\t{iterations}\t{elapsed}\tfalse");
    let expected = left.clone().try_into_native().unwrap();
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(left.clone().try_into_native().unwrap());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(left.clone().try_into_native().unwrap(), expected);
    println!("operation_clone_and_native_handoff\t{iterations}\t{elapsed}\t4:-1,-1,1,1");
    area_costs();
    bounds_costs();
    stencil_costs();
    affine_probe::run();
    native_drill_probe::run();
    ipc_probe::run();
    layer_order_probe::run();
    point_consumer_probe::run();
    authored_center_probe::run();
}

fn measure_value<T>(
    label: &str,
    iterations: usize,
    mut query: impl FnMut() -> T,
    check: impl Fn(&T) -> String,
) {
    let expected = check(&query());
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(check(&query()), expected);
    println!("operation_{label}\t{iterations}\t{elapsed}\t{expected}");
}

fn area_is(area: &Scalar, expected: &Scalar, witness: &str) -> String {
    assert_eq!(
        hyperlimit::compare_reals(area, expected, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
    witness.into()
}

fn shapes_witness(shapes: &[hyperdrc::report::ViolationPolygon], samples: usize) -> String {
    assert_eq!(shapes.len(), 1);
    assert_eq!(shapes[0].exterior.len(), samples + 1);
    assert!(shapes[0].holes.is_empty());
    assert!(shapes[0].area.is_finite());
    let mut hash = 0xcbf29ce484222325_u64;
    for value in std::iter::once(&shapes[0].area).chain(shapes[0].exterior.iter().flatten()) {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    format!("1:{}:{hash}", samples + 1)
}

fn area_costs() {
    use hyperdrc::geometry::{MultiPolygon, multipolygon_to_shapes_scalar};
    let pi = Scalar::pi();
    for samples in [17, 1_000_000] {
        let polygon = circle_polygon([0.0, 0.0], 1.0, samples);
        let profile = polygons_to_profile(vec![polygon.clone()], None);
        let before = witness(&profile, &pi);
        measure_value(
            &format!("warm_polygon_area_{samples}"),
            10_000,
            || polygon.try_area().unwrap(),
            |area| area_is(area, &pi, &format!("pi:{before}")),
        );
        assert_eq!(witness(&profile, &pi), before);
    }
    for samples in [17, 128] {
        measure_value(
            &format!("construct_polygon_and_area_{samples}"),
            100,
            || {
                let polygon = circle_polygon([0.0, 0.0], 1.0, samples);
                let area = polygon.try_area().unwrap();
                (polygon, area)
            },
            |(polygon, area)| {
                let profile = polygons_to_profile(vec![polygon.clone()], None);
                area_is(area, &pi, &format!("pi:{}", witness(&profile, &pi)))
            },
        );
        let polygons = MultiPolygon(vec![circle_polygon([0.0, 0.0], 1.0, samples)]);
        let min_area: Scalar = "3.13".parse().unwrap();
        measure_value(
            &format!("report_area_filter_{samples}"),
            1_000,
            || multipolygon_to_shapes_scalar(&polygons, &min_area).unwrap(),
            |shapes| shapes_witness(shapes, samples),
        );
    }
    let square = rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0);
    let overlap = rect_polygon([1.0, 0.0], [2.0, 2.0], 0.0);
    let repeated = polygons_to_profile(vec![square.clone(), square, overlap], None);
    let before = witness(&repeated, &Scalar::from(6));
    measure_value(
        "overlapping_duplicate_layer_area",
        1_000,
        || repeated.try_area().unwrap(),
        |area| area_is(area, &Scalar::from(6), &format!("6:{before}")),
    );
    assert_eq!(witness(&repeated, &Scalar::from(6)), before);

    let pad = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let copper = polygons_to_profile(vec![pad.clone()], None);
    let paste = polygons_to_profile(vec![pad.clone(), pad], None);
    let min_ratio: Scalar = "0.9".parse().unwrap();
    let max_ratio: Scalar = "1.1".parse().unwrap();
    let zero = Scalar::zero();
    assert_eq!(paste.try_to_multipolygon().unwrap().0.len(), 2);
    let coverage_witness = witness(&paste, &Scalar::one());
    let empty = |findings: &Vec<hyperdrc::report::Violation>| {
        assert!(findings.is_empty(), "{findings:?}");
        format!("0:1:{coverage_witness}")
    };
    measure_value(
        "duplicate_paste_union_coverage",
        100,
        || {
            hyperdrc::checks::layer::paste_aperture_ratio(
                "F.Paste", &paste, "F.Cu", &copper, &min_ratio, &max_ratio, &zero,
            )
        },
        empty,
    );
    measure_value(
        "duplicate_mask_union_coverage",
        100,
        || {
            hyperdrc::checks::layer::solder_mask_opening_ratio_readiness(
                "F.Cu", &copper, "F.Mask", &paste, &min_ratio, &max_ratio, &zero,
            )
        },
        empty,
    );
    measure_value(
        "duplicate_aperture_windowpane",
        100,
        || {
            hyperdrc::checks::stencil::thermal_pad_paste_windowpane_readiness(
                "F.Paste", &paste, "F.Cu", &copper, &zero, &min_ratio, &zero,
            )
        },
        |findings| {
            assert_eq!(findings.len(), 1, "{findings:?}");
            assert_eq!(findings[0].check, "thermal-pad-paste-windowpane-readiness");
            format!("1:1:{coverage_witness}")
        },
    );
    assert_eq!(witness(&paste, &Scalar::one()), coverage_witness);
}

fn uncertain_bounds_polygon() -> hyperdrc::geometry::Polygon<f64> {
    use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Scalar::one();
    let points = [
        Point2::from_values(0, 0),
        Point2::from_values(1, 0),
        Point2::new(zero, 1.into()),
    ];
    let segments = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| Segment2::Line(LineSeg2::try_new(a, b).unwrap()))
        .collect();
    let native = CurveRegion2::try_from_native_contours(
        vec![Contour2::try_new(segments).unwrap()],
        vec![],
    )
    .unwrap();
    let mut polygons = PcbRegion::new(native, None).try_to_multipolygon().unwrap().0;
    assert_eq!(polygons.len(), 1);
    polygons.remove(0)
}

fn bounds_costs() {
    let expected: [Scalar; 4] = [(-1).into(), (-1).into(), 1.into(), 1.into()];
    for samples in [17, 1_000_000] {
        let polygon = circle_polygon([0.0, 0.0], 1.0, samples);
        let profile = polygons_to_profile(vec![polygon.clone()], None);
        let before = witness(&profile, &Scalar::pi());
        measure_value(
            &format!("warm_polygon_bounds_{samples}"),
            10_000,
            || polygon.try_bounds().unwrap().unwrap(),
            |bounds| {
                assert_eq!(bounds, &expected);
                format!("-1,-1,1,1:{before}")
            },
        );
        assert_eq!(witness(&profile, &Scalar::pi()), before);
    }
    for samples in [17, 128] {
        measure_value(
            &format!("construct_polygon_and_bounds_{samples}"),
            100,
            || {
                let polygon = circle_polygon([0.0, 0.0], 1.0, samples);
                let bounds = polygon.try_bounds().unwrap().unwrap();
                (polygon, bounds)
            },
            |(polygon, bounds)| {
                assert_eq!(bounds, &expected);
                format!(
                    "-1,-1,1,1:{}",
                    witness(
                        &polygons_to_profile(vec![polygon.clone()], None),
                        &Scalar::pi()
                    )
                )
            },
        );
    }
    let empty = circle_polygon([0.0, 0.0], 0.0, 17);
    measure_value(
        "warm_empty_polygon_bounds",
        10_000,
        || empty.try_bounds().unwrap(),
        |bounds| {
            assert!(bounds.is_none());
            "empty".into()
        },
    );
    let uncertain = uncertain_bounds_polygon();
    assert!(uncertain.try_area().is_ok());
    measure_value(
        "uncertain_polygon_bounds",
        100,
        || uncertain.try_bounds().unwrap_err(),
        |error| {
            assert_eq!(error.operation, "polygon-bounds");
            assert!(error.detail.contains("Ordering"));
            "typed-ordering-failure".into()
        },
    );
    let mixed = polygons_to_profile(
        vec![
            circle_polygon([0.0, 0.0], 1.0, 17),
            circle_polygon([0.0, 0.0], 1.0, 128),
        ],
        None,
    );
    let before = witness(&mixed, &Scalar::pi());
    measure_value(
        "duplicate_islands_mixed_samples",
        100,
        || {
            hyperdrc::checks::layer::duplicate_layer_island_readiness(
                "F.Cu",
                &mixed,
                &Scalar::zero(),
            )
        },
        |findings| {
            assert_eq!(findings.len(), 1);
            assert_eq!(findings[0].check, "layer-sanity");
            assert_eq!(findings[0].locations, vec![[0.0, 0.0]]);
            format!("1:0,0:{before}")
        },
    );
    assert_eq!(witness(&mixed, &Scalar::pi()), before);

    let mut sparse = (0..1000)
        .map(|i| rect_polygon([i as f64 * 10.0, 0.0], [1.0, 1.0], 0.0))
        .collect::<Vec<_>>();
    sparse.extend([
        rect_polygon([0.0, 100.0], [1.0, 1.0], 0.0),
        rect_polygon([0.0, 100.0], [1.0, 1.0], 0.0),
        rect_polygon([0.0, 100.0], [1.0, 1.0], 0.0),
    ]);
    let sparse = polygons_to_profile(sparse, None);
    let before = witness(&sparse, &Scalar::from(1001));
    measure_value(
        "sparse_duplicate_islands_1003",
        20,
        || {
            hyperdrc::checks::layer::duplicate_layer_island_readiness(
                "F.Cu",
                &sparse,
                &Scalar::zero(),
            )
        },
        |findings| {
            assert_eq!(findings.len(), 1);
            assert_eq!(findings[0].check, "layer-sanity");
            assert_eq!(findings[0].locations, vec![[0.0, 100.0]]);
            format!("1:0,100:{before}")
        },
    );
    assert_eq!(witness(&sparse, &Scalar::from(1001)), before);
}

fn findings_witness(findings: &[hyperdrc::Violation], count: usize, check: &str) -> String {
    assert_eq!(findings.len(), count, "{findings:?}");
    assert!(findings.iter().all(|finding| finding.check == check));
    let bytes = serde_json::to_vec(findings).unwrap();
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in &bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{count}:{}:{hash}", bytes.len())
}

fn stencil_costs() {
    let zero = Scalar::zero();
    let left = rect_polygon([0.0, 0.0], [0.5, 0.5], 0.0);
    let right = rect_polygon([2.0, 0.0], [0.5, 0.5], 0.0);
    let paste = polygons_to_profile(vec![left.clone()], None);
    let copper = polygons_to_profile(vec![left, right], None);
    let before = witness(&copper, &(Scalar::one() / Scalar::from(2)).unwrap());
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let unknown_zero = &sine * &sine + &cosine * &cosine - Scalar::one();
    let cases = [
        (
            "stencil_boundary_pair",
            Scalar::from(2),
            1,
            "tombstone-paste-imbalance-readiness",
        ),
        (
            "stencil_below_pair_gap",
            "1.999999999999999999999999".parse().unwrap(),
            0,
            "tombstone-paste-imbalance-readiness",
        ),
        (
            "stencil_unrepresentable_radius",
            "1e10000".parse().unwrap(),
            1,
            "tombstone-paste-imbalance-readiness",
        ),
        (
            "stencil_unresolved_radius",
            Scalar::from(2) + &unknown_zero,
            1,
            "geometry-uncertainty",
        ),
    ];
    let delta: Scalar = "0.25".parse().unwrap();
    for (label, radius, count, check) in cases {
        measure_value(
            label,
            100,
            || {
                hyperdrc::checks::tombstone_paste_imbalance_readiness(
                    "F.Paste", &paste, "F.Cu", &copper, &radius, &delta, &zero,
                )
            },
            |findings| findings_witness(findings, count, check),
        );
    }
    assert_eq!(
        witness(&copper, &(Scalar::one() / Scalar::from(2)).unwrap()),
        before
    );
    let mut sparse = (0..1000)
        .map(|i| rect_polygon([i as f64 * 10.0, 10.0], [0.5, 0.5], 0.0))
        .collect::<Vec<_>>();
    sparse.extend([
        rect_polygon([0.0, 0.0], [0.5, 0.5], 0.0),
        rect_polygon([2.0, 0.0], [0.5, 0.5], 0.0),
    ]);
    let sparse = polygons_to_profile(sparse, None);
    let area = (Scalar::from(501) / Scalar::from(2)).unwrap();
    let before = witness(&sparse, &area);
    measure_value(
        "stencil_sparse_1002_pads",
        20,
        || {
            hyperdrc::checks::tombstone_paste_imbalance_readiness(
                "F.Paste",
                &paste,
                "F.Cu",
                &sparse,
                &2.into(),
                &delta,
                &zero,
            )
        },
        |findings| findings_witness(findings, 1, "tombstone-paste-imbalance-readiness"),
    );
    assert_eq!(witness(&sparse, &area), before);

    use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
    let via = CopperFeature {
        layer: "F.Cu".into(),
        net: Some("GND".into()),
        kind: CopperKind::Via,
        region: polygons_to_profile(vec![circle_polygon([0.0, 0.0], 0.25, 17)], None),
        location: [zero.clone(), zero.clone()],
    };
    let drill = DrillFeature {
        location: [zero.clone(), zero.clone()],
        diameter: delta.clone(),
        plated: true,
        net: Some("GND".into()),
    };
    let known = BoardModel {
        source: "stencil-cost".into(),
        copper: vec![via.clone()],
        drills: vec![drill.clone()],
        board_outline: None,
        panel_features: None,
    };
    let mut uncertain = known.clone();
    uncertain.copper[0].location[0] = Scalar::one() + unknown_zero;
    uncertain.copper[0].region =
        polygons_to_profile(vec![circle_polygon([1.0, 0.0], 0.25, 17)], None);
    uncertain.drills[0].diameter = Scalar::one();
    let mut unrepresentable = known.clone();
    unrepresentable.drills.clear();
    unrepresentable.copper[0].location[0] = "1e10000".parse().unwrap();
    let paste = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [4.0, 4.0], 0.0)], None);
    for (label, board, check) in [
        ("stencil_matched_via", known, "paste-via-exposure-readiness"),
        ("stencil_unresolved_drill", uncertain, "geometry-uncertainty"),
        (
            "stencil_unrepresentable_marker",
            unrepresentable,
            "geometry-uncertainty",
        ),
    ] {
        let area = (Scalar::pi() / Scalar::from(16)).unwrap();
        let before = witness(&board.copper[0].region, &area);
        measure_value(
            label,
            100,
            || {
                hyperdrc::checks::paste_via_exposure_readiness(
                    "F.Paste",
                    &paste,
                    &board,
                    &[],
                    &zero,
                )
            },
            |findings| findings_witness(findings, 1, check),
        );
        assert_eq!(witness(&board.copper[0].region, &area), before);
    }
}
