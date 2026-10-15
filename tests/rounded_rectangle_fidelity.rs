#![cfg(feature = "curve")]
//! Sampling fidelity is separate from a certified topology decision.
use csgrs::{GeometryCertainty, GeometryContext, Real, curve};
use hypercurve::CurveRegion2;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn vertices(region: &CurveRegion2) -> Vec<[Real; 2]> {
    let profiles = region.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
    material
        .segments()
        .iter()
        .map(|edge| [edge.start().x().clone(), edge.start().y().clone()])
        .collect()
}

#[test]
fn one_chord_per_corner_retains_all_straight_sides_and_exact_area() {
    let region = curve::rounded_rectangle(4.into(), 2.into(), q(1, 2), 1).unwrap();
    // A 4x2 rectangle minus four right triangles with legs 1/2.
    assert_eq!(region.filled_area().unwrap(), Some(q(15, 2)));
    let expected = [
        [Real::zero(), q(1, 2)],
        [q(1, 2), Real::zero()],
        [q(7, 2), Real::zero()],
        [4.into(), q(1, 2)],
        [4.into(), q(3, 2)],
        [q(7, 2), 2.into()],
        [q(1, 2), 2.into()],
        [Real::zero(), q(3, 2)],
    ];
    let actual = vertices(&region);
    assert_eq!(actual.len(), expected.len());
    assert!(expected.iter().all(|point| actual.contains(point)));
}

#[test]
fn clamped_capsules_keep_nonzero_straight_sides() {
    for (width, height) in [(4, 2), (2, 4)] {
        for radius in [1, 100] {
            let region =
                curve::rounded_rectangle(width.into(), height.into(), radius.into(), 1)
                    .unwrap();
            // The corner radius clamps to one: 8 - 4*(1/2) = 6.
            assert_eq!(region.filled_area().unwrap(), Some(6.into()));
            assert_eq!(vertices(&region).len(), 6);
        }
    }
}

#[test]
fn original_published_sampling_resolutions_retain_tangent_vertices() {
    for (label, width, height, requested_radius, radius, samples) in [
        ("benchmark", 12.into(), 8.into(), 2.into(), 2.into(), 8),
        (
            "README rational dimensions",
            q(24, 10),
            q(15, 10),
            q(28, 100),
            q(28, 100),
            12,
        ),
        (
            "README literal ingress",
            Real::try_from(2.4).unwrap(),
            Real::try_from(1.5).unwrap(),
            Real::try_from(0.28).unwrap(),
            Real::try_from(0.28).unwrap(),
            12,
        ),
        ("strict catalog", 2.into(), 1.into(), q(3, 4), q(1, 2), 4),
    ] {
        let result = curve::rounded_rectangle_with_context(
            width.clone(),
            height.clone(),
            requested_radius,
            samples,
            &GeometryContext::STRICT,
        )
        .unwrap();
        assert_eq!(result.certainty, GeometryCertainty::Certified);
        let actual = vertices(&result.value);
        for expected in [
            [Real::zero(), radius.clone()],
            [radius.clone(), Real::zero()],
            [&width - &radius, Real::zero()],
            [width.clone(), radius.clone()],
            [width.clone(), &height - &radius],
            [&width - &radius, height.clone()],
            [radius.clone(), height.clone()],
            [Real::zero(), &height - &radius],
        ] {
            assert!(
                actual.contains(&expected),
                "{label}: omitted tangent {expected:?}"
            );
        }
    }
}

#[test]
fn full_radius_and_sharp_corner_controls_remain_defined() {
    let diamond = curve::rounded_rectangle(2.into(), 2.into(), 1.into(), 1).unwrap();
    assert_eq!(diamond.filled_area().unwrap(), Some(2.into()));
    assert_eq!(vertices(&diamond).len(), 4);
    for (radius, samples) in [(0, 3), (1, 0)] {
        let sharp =
            curve::rounded_rectangle(4.into(), 2.into(), radius.into(), samples).unwrap();
        assert_eq!(sharp.filled_area().unwrap(), Some(8.into()));
        assert_eq!(vertices(&sharp).len(), 4);
    }
}

#[test]
fn algebraic_corners_and_symbolic_dimensions_keep_independently_known_areas() {
    let octants = curve::rounded_rectangle(4.into(), 2.into(), q(1, 2), 2).unwrap();
    // Four quarter-circle sectors become eight triangles of central angle pi/4.
    let area = Real::from(7) + q(1, 2) * Real::from(2).sqrt().unwrap();
    // Real's structural equality does not identify differently ordered sums.
    // Require a strict proof of mathematical equality, never a float tolerance.
    assert_eq!(
        hyperlimit::compare_reals(
            &octants.filled_area().unwrap().unwrap(),
            &area,
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let symbolic = curve::rounded_rectangle(Real::pi(), 2.into(), 10.into(), 1).unwrap();
    assert_eq!(
        hyperlimit::compare_reals(
            &symbolic.filled_area().unwrap().unwrap(),
            &(Real::from(2) * Real::pi() - Real::from(2)),
            hyperlimit::PredicatePolicy::STRICT,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
}

fn pipeline() -> csgrs::TriangleMesh {
    use hypercurve::BooleanOp;
    let source = curve::rounded_rectangle_with_context(
        4.into(),
        2.into(),
        q(1, 2),
        1,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(source.certainty, GeometryCertainty::Certified);
    let translated = curve::try_translated_with_context(
        &source.value,
        10.into(),
        (-3).into(),
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(translated.certainty, GeometryCertainty::Certified);
    let cutter =
        curve::try_translated(&curve::square(1.into()).unwrap(), 11.into(), q(-5, 2)).unwrap();
    let cut = hypercurve::evaluate_under(hyperlimit::PredicatePolicy::STRICT, || {
        translated
            .value
            .boolean_region(&cutter, BooleanOp::Difference)
    });
    assert!(cut.is_certified());
    let cut = cut.into_unverified().unwrap();
    assert_eq!(cut.filled_area().unwrap(), Some(q(13, 2)));
    let restored = cut.boolean_region(&cutter, BooleanOp::Union).unwrap();
    assert_eq!(restored.filled_area().unwrap(), Some(q(15, 2)));
    assert!(
        restored
            .boolean_region(&translated.value, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
    let solid = curve::try_extrude(&cut, 3.into(), &GeometryContext::STRICT).unwrap();
    assert_eq!(solid.certainty, GeometryCertainty::Certified);
    assert_solid(&solid.value);
    solid.value
}

fn assert_solid(mesh: &csgrs::TriangleMesh) {
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(six_volume, Real::from(117));
}

#[test]
fn corrected_profiles_compose_with_transforms_holes_booleans_and_extrusion() {
    pipeline();
}

#[cfg(feature = "obj-io")]
#[test]
fn dyadic_profile_pipeline_survives_the_obj_boundary_without_value_loss() {
    let mesh = pipeline();
    let exported = csgrs::io::obj::to_obj(&mesh, "rounded-hole").unwrap();
    let restored =
        csgrs::io::obj::from_obj_with_context(exported.as_bytes(), &GeometryContext::STRICT)
            .unwrap();
    assert_eq!(restored.certainty, GeometryCertainty::Certified);
    // All coordinates in this fixture are dyadic and exactly representable at
    // the finite export boundary. This is not a losslessness claim for curves.
    assert_eq!(restored.value.positions, mesh.positions);
    assert_solid(&restored.value);
}

#[test]
fn approximate_clamp_facts_remain_qualified_and_do_not_warm_strict_success() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let radius = &sine * &sine + &cosine * &cosine;
    assert!(
        curve::rounded_rectangle_with_context(
            2.into(),
            4.into(),
            radius.clone(),
            1,
            &GeometryContext::STRICT
        )
        .is_err()
    );
    let result = curve::rounded_rectangle_with_context(
        2.into(),
        4.into(),
        radius.clone(),
        1,
        &GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!result.value.is_empty());
    assert!(
        curve::rounded_rectangle_with_context(
            2.into(),
            4.into(),
            radius,
            1,
            &GeometryContext::STRICT
        )
        .is_err()
    );
}

#[test]
fn capacity_covers_every_corner_endpoint_even_before_empty_shortcuts() {
    let count = (isize::MAX as usize) / (4 * std::mem::size_of::<[Real; 2]>());
    assert!(matches!(
        curve::rounded_rectangle(Real::zero(), Real::one(), Real::one(), count),
        Err(csgrs::errors::ValidationError::CapacityOverflow)
    ));
}

#[cfg(feature = "gerber-io")]
#[test]
fn public_gerber_obrounds_retain_straight_sides_through_format_roundtrip() {
    use csgrs::io::gerber;
    use hypercurve::{Point2, RegionPointLocation};
    let source = b"%FSLAX46Y46*%\n%MOMM*%\n%ADD10O,4.0X2.0*%\nD10*\nX0Y0D03*\nM02*\n";
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let imported = gerber::import_gerber_with_context(source, &context).unwrap();
        assert_eq!(imported.certainty, GeometryCertainty::Certified);
        let exported = gerber::export_gerber(&imported.value.0).unwrap();
        let restored = gerber::import_gerber_with_context(&exported, &context).unwrap();
        for region in [&imported.value.0, &restored.value.0] {
            for (x, y, expected) in [
                (-1, -1, RegionPointLocation::Boundary),
                (1, -1, RegionPointLocation::Boundary),
                (-1, 1, RegionPointLocation::Boundary),
                (1, 1, RegionPointLocation::Boundary),
                (0, 0, RegionPointLocation::Inside),
                (3, 0, RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    region
                        .classify_point(&Point2::from_values(x, y).into())
                        .unwrap(),
                    expected
                );
            }
        }
    }
}
