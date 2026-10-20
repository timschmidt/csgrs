#![cfg(feature = "curve")]

#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, curve,
    errors::{ValidationError, ValidationErrorKind},
    solid,
};
use hypercurve::{BooleanOp, CurveRegion2};

fn r(n: i32) -> Real {
    n.into()
}
fn f(n: f64) -> Real {
    Real::try_from(n).unwrap()
}
fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - r(1))
}
fn failure<T: std::fmt::Debug>(result: Result<T, ValidationError>, kind: ValidationErrorKind) {
    assert_eq!(
        result
            .expect_err("failed construction became geometry")
            .kind(),
        kind
    );
}
fn profile(outcome: GeometryOutcome<CurveRegion2>) -> CurveRegion2 {
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    assert!(!outcome.value.is_empty());
    let profiles = outcome.value.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
    assert!(!material.segments().is_empty());
    outcome.value
}

#[test]
fn invalid_inputs_and_derived_geometry_fail_through_every_primary_family() {
    use ValidationErrorKind::InvalidInput;
    for result in [
        curve::involute_gear(r(0), 16, r(20), r(0), r(0), 4),
        curve::involute_gear(r(1), 3, r(20), r(0), r(0), 4),
        curve::involute_gear(r(1), 16, r(90), r(0), r(0), 4),
        curve::involute_gear(r(1), 16, r(20), r(100), r(0), 4),
        curve::cycloidal_gear(r(1), 2, r(1), r(0), 4),
        curve::cycloidal_gear(r(1), 16, q(3, 4), r(0), 8),
        curve::involute_rack(r(1), 0, r(20), r(0), r(0)),
        curve::involute_rack(r(1), 4, r(20), r(0), r(4)),
        curve::cycloidal_rack(r(1), 4, r(0), 3),
        curve::cycloidal_rack(r(1), 4, r(100), 8),
        curve::airfoil_naca4(r(2), r(4), r(12), r(1), 9),
        curve::airfoil_naca4(r(2), r(0), r(12), r(1), 20),
        curve::airfoil_naca4(r(2), r(4), r(100), r(1), 20),
    ] {
        failure(result, InvalidInput);
    }
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for result in [
            curve::involute_gear_with_context(r(-1), 16, r(20), r(0), r(0), 4, &context),
            curve::cycloidal_gear_with_context(r(1), 16, r(-1), r(0), 4, &context),
            curve::involute_rack_with_context(r(1), 4, r(0), r(0), r(0), &context),
            curve::cycloidal_rack_with_context(r(1), 4, r(-1), 8, &context),
            curve::airfoil_naca4_with_context(r(-1), r(4), r(12), r(1), 20, &context),
        ] {
            failure(result, InvalidInput);
        }
        for height in [r(0), r(1)] {
            for result in [
                solid::spur_gear_involute(
                    r(0),
                    16,
                    r(20),
                    r(0),
                    r(0),
                    4,
                    height.clone(),
                    &context,
                ),
                solid::spur_gear_cycloid(r(0), 16, r(1), r(0), 4, height.clone(), &context),
                solid::helical_involute_gear(
                    r(0),
                    16,
                    r(20),
                    r(0),
                    r(0),
                    4,
                    height.clone(),
                    r(10),
                    3,
                    &context,
                ),
            ] {
                failure(result, InvalidInput);
            }
        }
    }
}

#[test]
fn every_sampling_family_rejects_capacity_overflow_before_allocation() {
    for count in [usize::MAX, usize::MAX / 2] {
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            for result in [
                curve::involute_gear_with_context(
                    r(1),
                    16,
                    r(20),
                    r(0),
                    r(0),
                    count,
                    &context,
                ),
                curve::involute_gear_with_context(r(1), count, r(20), r(0), r(0), 4, &context),
                curve::cycloidal_gear_with_context(r(1), 16, r(1), r(0), count, &context),
                curve::cycloidal_gear_with_context(r(1), count, r(1), r(0), 4, &context),
                curve::involute_rack_with_context(r(1), count, r(20), r(0), r(0), &context),
                curve::cycloidal_rack_with_context(r(1), 4, r(0), count, &context),
                curve::cycloidal_rack_with_context(r(1), count, r(0), 8, &context),
                curve::airfoil_naca4_with_context(r(0), r(0), r(12), r(1), count, &context),
            ] {
                assert!(matches!(result, Err(ValidationError::CapacityOverflow)));
            }
            for result in [
                solid::spur_gear_involute(r(1), 16, r(20), r(0), r(0), count, r(1), &context),
                solid::spur_gear_cycloid(r(1), 16, r(1), r(0), count, r(1), &context),
                solid::helical_involute_gear(
                    r(1),
                    16,
                    r(20),
                    r(0),
                    r(0),
                    count,
                    r(1),
                    r(10),
                    3,
                    &context,
                ),
            ] {
                failure(result, ValidationErrorKind::ResourceExhausted);
            }
        }
    }
    assert!(
        !curve::involute_rack(r(1), 2, r(30), r(0), r(0))
            .unwrap()
            .is_empty()
    );
}

#[test]
fn undecided_parameters_do_not_become_empty_profiles_or_solids() {
    for _ in 0..2 {
        for result in [
            curve::involute_gear(unresolved_zero(), 16, r(20), r(0), r(0), 4),
            curve::cycloidal_gear(unresolved_zero(), 16, r(1), r(0), 4),
            curve::involute_rack(unresolved_zero(), 4, r(20), r(0), r(0)),
            curve::cycloidal_rack(unresolved_zero(), 4, r(0), 8),
            curve::airfoil_naca4(r(2), r(4), r(12), unresolved_zero(), 20),
        ] {
            failure(result, ValidationErrorKind::Undecided);
        }
        for result in [
            solid::spur_gear_involute(
                unresolved_zero(),
                16,
                r(20),
                r(0),
                r(0),
                4,
                r(1),
                &GeometryContext::STRICT,
            ),
            solid::spur_gear_cycloid(
                unresolved_zero(),
                16,
                r(1),
                r(0),
                4,
                r(1),
                &GeometryContext::STRICT,
            ),
            solid::helical_involute_gear(
                unresolved_zero(),
                16,
                r(20),
                r(0),
                r(0),
                4,
                r(1),
                r(10),
                3,
                &GeometryContext::STRICT,
            ),
        ] {
            failure(result, ValidationErrorKind::Undecided);
        }
    }
}

fn exact(a: &Real, b: &Real) {
    assert_eq!(a.certified_eq_until(b, -1024).as_bool(), Some(true));
}
fn volume_and_topology(mesh: &TriangleMesh, expected_volume: &Real) {
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    let six_volume = mesh.triangles.iter().fold(r(0), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    exact(&six_volume, &(r(6) * expected_volume));
}

#[test]
fn independent_rack_area_membership_and_extrusion_compose() {
    // Two 30-degree teeth: backing width 3*pi/2+5/(2*sqrt(3));
    // two trapezoids of height 9/4 and average width pi/2+1/(4*sqrt(3)).
    let area = q(15, 4) * Real::pi() + q(29, 24) * r(3).sqrt().unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let rack = profile(
            curve::involute_rack_with_context(r(1), 2, r(30), r(0), r(0), &context).unwrap(),
        );
        exact(&rack.filled_area().unwrap().unwrap(), &area);
        for (x, y, expected) in [
            (r(0), r(0), Some(true)),
            (Real::pi(), r(0), Some(true)),
            ((Real::pi() / r(2)).unwrap(), r(0), Some(false)),
            ((Real::pi() / r(2)).unwrap(), r(-2), Some(true)),
            (r(0), r(1), None),
        ] {
            let location = curve::contains_xy_with_context(&rack, x, y, &context).unwrap();
            assert_eq!(location.certainty, GeometryCertainty::Certified);
            assert_eq!(location.value, expected);
        }
        let translated = curve::try_translated(&rack, r(5), r(-3)).unwrap();
        exact(&translated.filled_area().unwrap().unwrap(), &area);
        let union = translated
            .boolean_region(&curve::empty(), BooleanOp::Union)
            .unwrap();
        let extruded = curve::try_extrude(&union, r(2), &context).unwrap();
        assert_eq!(extruded.certainty, GeometryCertainty::Certified);
        volume_and_topology(&extruded.value, &(r(2) * &area));
        let empty = union
            .boolean_region(&translated, BooleanOp::Difference)
            .unwrap();
        assert!(empty.is_empty());
        assert!(
            curve::try_extrude(&empty, r(2), &context)
                .unwrap()
                .value
                .triangles
                .is_empty()
        );
    }
}

#[test]
fn independent_gear_and_airfoil_membership_matches_the_sampled_contract() {
    for region in [
        curve::involute_gear(r(1), 16, r(20), r(0), r(0), 4).unwrap(),
        curve::cycloidal_gear(r(1), 16, r(1), r(0), 4).unwrap(),
    ] {
        assert_eq!(curve::contains_xy(&region, r(0), r(0)).unwrap(), Some(true));
        // The addendum radius is 9; no sampled edge extends to radius 10.
        for (x, y) in [(r(10), r(0)), (r(-10), r(0)), (r(0), r(10)), (r(0), r(-10))] {
            assert_eq!(curve::contains_xy(&region, x, y).unwrap(), Some(false));
        }
    }
    let airfoil = curve::airfoil_naca4(r(0), r(0), r(12), r(1), 20).unwrap();
    // Exact evaluation at x=1/4 of the existing represented binary64 NACA
    // coefficients with t=3/25, independently reduced to one rational.
    let ordinate = (Real::from(547_982_870_563_693_971_u64)
        / Real::from(9_223_372_036_854_775_808_u64))
    .unwrap();
    for (x, y, expected) in [
        (r(0), r(0), None),
        (q(1, 4), ordinate.clone(), None),
        (q(1, 4), -ordinate, None),
        (q(1, 4), r(0), Some(true)),
        (q(1, 4), q(1, 5), Some(false)),
        (q(1, 4), q(-1, 5), Some(false)),
    ] {
        assert_eq!(curve::contains_xy(&airfoil, x, y).unwrap(), expected);
    }
}

#[test]
fn approximate_validation_receipts_survive_cache_reuse_and_solid_composition() {
    // Warm the exact symmetric ring before consuming approximate camber-zero evidence.
    curve::airfoil_naca4(r(0), r(0), r(12), r(1), 20).unwrap();
    for _ in 0..2 {
        failure(
            curve::airfoil_naca4(unresolved_zero(), r(0), r(12), r(1), 20),
            ValidationErrorKind::Undecided,
        );
        let outcome = curve::airfoil_naca4_with_context(
            unresolved_zero(),
            r(0),
            r(12),
            r(1),
            20,
            &GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
        assert!(!outcome.value.is_empty());
    }
    failure(
        curve::airfoil_naca4(unresolved_zero(), r(0), r(12), r(1), 20),
        ValidationErrorKind::Undecided,
    );
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        for result in [
            solid::spur_gear_involute(
                r(1),
                16,
                r(20),
                unresolved_zero(),
                r(0),
                3,
                r(1),
                &context,
            ),
            solid::spur_gear_cycloid(r(1), 16, r(1), unresolved_zero(), 3, r(1), &context),
            solid::helical_involute_gear(
                r(1),
                16,
                r(20),
                unresolved_zero(),
                r(0),
                3,
                r(2),
                r(15),
                3,
                &context,
            ),
        ] {
            if context == GeometryContext::STRICT {
                failure(result, ValidationErrorKind::Undecided);
            } else {
                let outcome = result.unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
                assert!(!outcome.value.triangles.is_empty());
            }
        }
    }
}

#[test]
fn original_full_resolution_readme_and_benchmark_profiles_remain_certified() {
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for outcome in [
            curve::involute_gear_with_context(
                f(0.22),
                14,
                f(20.0),
                f(0.02),
                f(0.01),
                5,
                &context,
            ),
            curve::cycloidal_gear_with_context(f(0.22), 14, f(0.275), f(0.02), 8, &context),
            curve::involute_rack_with_context(f(0.32), 7, f(20.0), f(0.02), f(0.01), &context),
            curve::cycloidal_rack_with_context(f(0.32), 7, f(0.02), 12, &context),
            curve::airfoil_naca4_with_context(f(2.0), f(4.0), f(12.0), f(3.0), 96, &context),
            curve::involute_gear_with_context(r(2), 20, r(20), r(0), r(0), 4, &context),
            curve::cycloidal_gear_with_context(r(2), 12, r(2), r(0), 4, &context),
            curve::involute_rack_with_context(r(2), 4, r(20), r(0), r(0), &context),
            curve::cycloidal_rack_with_context(r(2), 4, r(0), 8, &context),
            curve::airfoil_naca4_with_context(r(2), r(4), r(12), r(20), 80, &context),
        ] {
            profile(outcome.unwrap());
        }
    }
}
