#[path = "../benches/common/pathological.rs"]
mod pathological_fixture;

use std::collections::HashSet;

use hypercurve::{BooleanOp, Classification, CurveFamily2, CurvePolicy, FillRule};
use pathological_fixture::build_native_cell;

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
    let prepared = cell
        .source
        .retain_boolean(&cell.rotated, &policy)
        .expect("all-family pair reaches curved Boolean preparation");
    assert!(prepared.authored_carrier_pair_count() > 0);
    assert!(prepared.carrier_pair_count() > 0);
    #[cfg(feature = "predicates")]
    {
        let evidence = prepared
            .intersection_result()
            .expect("all-family intersections are evidenceable");
        assert!(evidence.blockers().is_empty());
    }

    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        #[cfg(feature = "predicates")]
        prepared
            .boolean_region(operation)
            .expect("all-family curved Boolean is decided");
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
    #[cfg(feature = "predicates")]
    assert!(
        prepared
            .boolean_topology_point_classification_count()
            .unwrap()
            < prepared.boolean_topology_fragment_count().unwrap(),
        "certified interior transversal contacts should reuse adjacent fragment classification"
    );
}
