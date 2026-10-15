#![cfg(feature = "metaballs")]
//! Preserve the published rendering and benchmark workloads through exact extraction.
use csgrs::{
    GeometryCertainty, GeometryContext, Real,
    solid::{self, MetaBall},
};
use hyperlattice::Point3;

fn r(value: f64) -> Real {
    Real::try_from(value).unwrap()
}
fn point(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
fn validate(balls: &[MetaBall], resolution: usize, iso: Real, padding: Real) {
    let bounds = hypersdf::MetaballField::new(balls, iso.clone())
        .unwrap()
        .sampling_bounds(&padding)
        .unwrap()
        .unwrap();
    let outcome = solid::metaballs_with_context(
        balls,
        (resolution, resolution, resolution),
        iso,
        padding,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let mesh = outcome.value;
    assert!(!mesh.triangles.is_empty());
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, t| {
        let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(
        hyperlimit::classify_real_sign(&six_volume, hyperlimit::PredicatePolicy::STRICT)
            .value(),
        Some(hyperlimit::Sign::Positive)
    );
    let actual = solid::bounding_box(&mesh).unwrap().unwrap();
    for (value, lower, upper) in [
        (&actual.mins.x, &bounds.mins.x, &bounds.maxs.x),
        (&actual.mins.y, &bounds.mins.y, &bounds.maxs.y),
        (&actual.mins.z, &bounds.mins.z, &bounds.maxs.z),
        (&actual.maxs.x, &bounds.mins.x, &bounds.maxs.x),
        (&actual.maxs.y, &bounds.mins.y, &bounds.maxs.y),
        (&actual.maxs.z, &bounds.mins.z, &bounds.maxs.z),
    ] {
        assert_eq!(
            hyperlimit::compare_reals(value, lower, hyperlimit::PredicatePolicy::STRICT)
                .value(),
            Some(std::cmp::Ordering::Greater)
        );
        assert_eq!(
            hyperlimit::compare_reals(value, upper, hyperlimit::PredicatePolicy::STRICT)
                .value(),
            Some(std::cmp::Ordering::Less)
        );
    }
}

#[test]
fn feature_pipeline_metaballs_complete_at_original_resolution_twenty() {
    let balls = [
        MetaBall::new(Point3::origin(), 2.into()),
        MetaBall::new(Point3::new(2.into(), 0.into(), 0.into()), 2.into()),
    ];
    validate(&balls, 20, 1.into(), 1.into());
}

#[test]
fn readme_metaballs_complete_with_all_three_original_sources() {
    let balls = [
        MetaBall::new(point(-0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(point(0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(point(0.0, 0.55, 0.25), r(0.58)),
    ];
    validate(&balls, 8, r(0.7), r(0.25));
}
