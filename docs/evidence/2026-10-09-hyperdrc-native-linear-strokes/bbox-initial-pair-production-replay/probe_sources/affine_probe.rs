use super::*;
use hyperdrc::geometry::{LineString, Polygon, transform_polygon};

fn transformed_witness(polygon: &Polygon, area: &Scalar, bounds: [i32; 4]) -> String {
    for (bound, expected) in polygon.try_bounds().unwrap().unwrap().iter().zip(bounds) {
        assert_eq!(
            hyperlimit::compare_reals(bound, &expected.into(), hyperlimit::PredicatePolicy::STRICT)
                .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
    witness(&polygons_to_profile(vec![polygon.clone()], None), area)
}

pub(super) fn run() {
    let rectangle = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0);
    measure_value(
        "affine_rectangle_identity",
        1_000,
        || transform_polygon(&rectangle, [0.0, 0.0], 0.0),
        |polygon| transformed_witness(polygon, &8.into(), [-1, -2, 1, 2]),
    );
    measure_value(
        "affine_rectangle_quarterturn",
        1_000,
        || transform_polygon(&rectangle, [10.0, 20.0], 90.0),
        |polygon| transformed_witness(polygon, &8.into(), [8, 19, 12, 21]),
    );
    for samples in [17, 128] {
        let circle = circle_polygon([0.0, 0.0], 2.0, samples);
        let area = Scalar::from(4) * Scalar::pi();
        measure_value(
            &format!("affine_circle_{samples}"),
            100,
            || transform_polygon(&circle, [3.0, -4.0], 30.0),
            |polygon| {
                assert_eq!(polygon.exterior().0.len(), samples + 1);
                transformed_witness(polygon, &area, [1, -6, 5, -2])
            },
        );
    }
    let mut polygons = PcbRegion::new(quadratic(), None)
        .try_to_multipolygon()
        .unwrap()
        .0;
    assert_eq!(polygons.len(), 1);
    let cap = polygons.remove(0);
    let cap_area = (Scalar::from(4) / Scalar::from(3)).unwrap();
    measure_value(
        "affine_quadratic_cap",
        100,
        || transform_polygon(&cap, [10.0, 20.0], 90.0),
        |polygon| transformed_witness(polygon, &cap_area, [9, 20, 10, 22]),
    );
    let empty = circle_polygon([0.0, 0.0], 0.0, 17);
    measure_value(
        "affine_empty",
        1_000,
        || transform_polygon(&empty, [10.0, 20.0], 90.0),
        |polygon| {
            assert_eq!(polygon.try_area().unwrap(), Scalar::zero());
            assert_eq!(polygon.try_bounds().unwrap(), None);
            assert!(polygon.try_exact_region().unwrap().is_empty());
            "certified-empty".into()
        },
    );
    let failed = Polygon::new(LineString(Vec::new()), Vec::new());
    measure_value(
        "affine_source_failure",
        1_000,
        || transform_polygon(&failed, [10.0, 20.0], 90.0),
        |polygon| {
            let error = polygon.try_area().unwrap_err();
            assert!(error.detail.contains("polygon-affine-transform"));
            assert!(error.detail.contains("fewer than three"));
            assert!(polygon.try_exact_region().is_err());
            "retained-source-failure".into()
        },
    );
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    measure_value(
        "affine_exhausted_scope",
        1_000,
        || {
            let mut escaped = None;
            let denied = hypercurve::evaluate_with_budget(
                hyperlimit::PredicatePolicy::STRICT,
                hypercurve::WorkBudget::ZERO,
                || {
                    assert!(polynomial.isolate_unit_interval_roots().is_err());
                    escaped = Some(transform_polygon(&rectangle, [10.0, 20.0], 90.0));
                },
            );
            assert!(denied.result.is_err());
            escaped.unwrap()
        },
        |polygon| {
            let error = polygon.try_area().unwrap_err();
            assert!(error.detail.contains("work budget exhausted"));
            assert!(polygon.try_exact_region().is_err());
            "retained-budget-failure".into()
        },
    );
}
