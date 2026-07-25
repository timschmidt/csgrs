#[path = "../benches/common/pathological.rs"]
mod pathological_fixture;

use std::collections::HashSet;

use hypercurve::{BooleanOp, Classification, CurveFamily2, CurvePolicy, FillRule};
use pathological_fixture::build_native_cell;
#[cfg(feature = "predicates")]
use pathological_fixture::{MemoryTier, NativeDataset};

#[test]
fn pathological_cell_covers_every_curve_and_real_representation_family() {
    let cell = build_native_cell(0);
    let families = cell
        .source_path
        .curves()
        .iter()
        .map(|curve| curve.family())
        .collect::<HashSet<_>>();
    assert_eq!(
        families,
        HashSet::from([
            CurveFamily2::Line,
            CurveFamily2::CircularArc,
            CurveFamily2::QuadraticBezier,
            CurveFamily2::CubicBezier,
            CurveFamily2::RationalQuadraticBezier,
            CurveFamily2::RationalBezier,
            CurveFamily2::PolynomialBSpline,
            CurveFamily2::Nurbs,
        ])
    );

    let representation_names = cell
        .representations
        .iter()
        .map(|sample| sample.name)
        .collect::<HashSet<_>>();
    assert_eq!(representation_names.len(), cell.representations.len());
    for required in [
        "small_integer_rational",
        "multi_limb_fraction_rational",
        "f32_dyadic_rational",
        "f64_dyadic_rational",
        "pi",
        "pi_power",
        "pi_inverse",
        "exp_rational",
        "pi_exp",
        "pi_inverse_exp",
        "constant_product",
        "constant_offset",
        "square_root",
        "pi_square_root",
        "constant_product_square_root",
        "natural_logarithm",
        "logarithm_affine",
        "logarithm_product",
        "log10",
        "log2",
        "sin_pi_rational",
        "tan_pi_rational",
        "opaque_computable",
    ] {
        assert!(
            representation_names.contains(required),
            "missing {required}"
        );
    }

    assert_eq!(cell.source.len(), 1);
    assert_eq!(cell.rotated.len(), 1);
    assert_ne!(cell.source_path.start(), cell.rotated_path.start());
}

#[test]
fn pathological_cell_reaches_curved_intersections_and_decidable_polygon_booleans() {
    let cell = build_native_cell(0);
    let policy = CurvePolicy::certified();
    #[cfg(feature = "predicates")]
    {
        let evidence = cell
            .source
            .intersect_region(&cell.rotated, &policy)
            .expect("all-family intersections are evidenceable");
        assert!(evidence.blockers().is_empty());
        let results = cell
            .source
            .boolean_regions(&cell.rotated, &policy)
            .expect("all-family immediate curved Booleans are decided");
        assert!(results.authored_carrier_pair_count() > 0);
        assert!(results.candidate_carrier_pair_count() > 0);
        assert!(
            results.topology_point_classification_count() < results.topology_fragment_count(),
            "certified interior transversal contacts should reuse adjacent fragment classification"
        );
        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            assert!(!results.region(operation).is_empty());
        }
    }

    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        assert!(matches!(
            cell.source_projection.boolean_region(
                &cell.rotated_projection,
                operation,
                FillRule::EvenOdd,
                &policy,
            ),
            Ok(Classification::Decided(_))
        ));
    }
}

#[test]
#[cfg(feature = "predicates")]
fn pathological_pi_weight_conic_decides_native_booleans_without_projection() {
    // Cell two assigns the exact transcendental value pi to both authored
    // rational-quadratic weights. Its conic/cubic contacts formerly reached
    // root isolation but blocked while forcing the Real-coefficient roots
    // through the rational-coefficient algebraic-number image package.
    let cell = build_native_cell(2);
    let policy = CurvePolicy::certified();
    let evidence = cell
        .source
        .intersect_region(&cell.rotated, &policy)
        .expect("pi-weight conic/cubic intersections retain exact evidence");
    assert!(evidence.blockers().is_empty(), "{:#?}", evidence.blockers());

    let results = cell
        .source
        .boolean_regions(&cell.rotated, &policy)
        .expect("pi-weight all-family pair completes immediate exact Booleans");
    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let _exact_region = results.region(operation);
    }
}

#[test]
#[cfg(feature = "predicates")]
fn full_pathological_native_workload_decides_all_268_exact_booleans() {
    let dataset = NativeDataset::build(MemoryTier::Mib100);
    let policy = CurvePolicy::certified();
    let operations = [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ];
    let mut decided = 0_usize;

    for (cell_index, cell) in dataset.cells.iter().enumerate() {
        let evidence = cell
            .source
            .intersect_region(&cell.rotated, &policy)
            .unwrap_or_else(|error| {
                panic!("pathological cell {cell_index} failed exact intersections: {error}")
            });
        assert!(
            evidence.blockers().is_empty(),
            "pathological cell {cell_index} retained blockers: {:#?}",
            evidence.blockers()
        );

        let results = cell
            .source
            .boolean_regions(&cell.rotated, &policy)
            .unwrap_or_else(|error| {
                panic!("pathological cell {cell_index} immediate Booleans failed: {error}")
            });
        for operation in operations {
            let _exact_region = results.region(operation);
            decided += 1;
        }
    }

    assert_eq!(dataset.cells.len(), 67);
    assert_eq!(decided, 268);
}
