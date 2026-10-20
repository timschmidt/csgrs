//! Failed construction must not enter a pipeline as successful empty geometry.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{Real, adapter::RawTriangleMesh};
use csgrs::{
    TriangleMesh,
    errors::ValidationError,
    solid::{self, SolidExt},
};
use hyperlattice::{Point3, Vector3};

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
fn negative_dimensions_are_errors_through_the_public_adapter() {
    assert!(RawTriangleMesh::cube(-Real::one()).is_err());
    assert!(RawTriangleMesh::cuboid(Real::one(), -Real::one(), Real::one()).is_err());
    assert!(RawTriangleMesh::sphere(-Real::one(), 8, 4).is_err());
    assert!(RawTriangleMesh::cylinder(-Real::one(), Real::one(), 8).is_err());
}

#[test]
fn undecided_dimensions_are_errors_through_the_public_adapter() {
    assert!(RawTriangleMesh::cube(unresolved_zero()).is_err());
}

#[test]
fn invalid_resolution_is_an_error_even_for_zero_radius() {
    for radius in [Real::one(), Real::zero()] {
        assert!(RawTriangleMesh::sphere(radius.clone(), 2, 4).is_err());
        assert!(RawTriangleMesh::sphere(radius.clone(), 8, 1).is_err());
        assert!(RawTriangleMesh::cylinder(radius, Real::one(), 2).is_err());
    }
}

#[test]
fn capacity_overflow_is_an_error_through_the_public_adapter() {
    assert!(RawTriangleMesh::sphere(Real::one(), usize::MAX, 2).is_err());
    assert!(RawTriangleMesh::cylinder(Real::one(), Real::one(), usize::MAX).is_err());
}

#[test]
fn native_errors_remain_structured_through_adapters() {
    use csgrs::adapter::AdapterError;
    assert!(matches!(
        solid::cube(-Real::one()),
        Err(ValidationError::NegativeParameter { name: "width" })
    ));
    assert!(matches!(
        RawTriangleMesh::cube(-Real::one()),
        Err(AdapterError::Construction(
            ValidationError::NegativeParameter { name: "width" }
        ))
    ));
    assert!(matches!(
        RawTriangleMesh::cube(unresolved_zero()),
        Err(AdapterError::Construction(
            ValidationError::PredicateUndecided { .. }
        ))
    ));
    assert!(matches!(
        RawTriangleMesh::sphere(Real::one(), 2, 4),
        Err(AdapterError::Construction(
            ValidationError::InvalidResolution {
                name: "segments",
                min: 3,
                actual: 2
            }
        ))
    ));
    assert!(matches!(
        RawTriangleMesh::sphere(Real::one(), usize::MAX, 2),
        Err(AdapterError::Construction(ValidationError::CapacityOverflow))
    ));
}

#[test]
fn all_native_primitive_families_expose_invalid_parameters() {
    let results = [
        solid::cube(-Real::one()),
        solid::cuboid(Real::one(), -Real::one(), Real::one()),
        solid::sphere(-Real::one(), 8, 4),
        solid::cylinder(-Real::one(), Real::one(), 8),
        solid::ellipsoid(Real::one(), Real::one(), -Real::one(), 8, 4),
        solid::frustum(Real::one(), -Real::one(), Real::one(), 8),
        solid::frustum_between(
            Point3::origin(),
            Point3::origin() + Vector3::z(),
            -Real::one(),
            Real::one(),
            8,
        ),
        solid::octahedron(-Real::one()),
        solid::icosahedron(-Real::one()),
    ];
    for result in results {
        assert!(matches!(
            result,
            Err(ValidationError::NegativeParameter { .. })
        ));
    }
    assert!(matches!(
        solid::arrow(Point3::origin(), Vector3::z(), 2, false),
        Err(ValidationError::InvalidResolution { .. })
    ));
}

fn assert_canonical_empty(mesh: TriangleMesh) {
    assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
    let cube = solid::cube(Real::one()).unwrap();
    let restored = mesh.try_union(&cube).unwrap();
    // Independent exact volume, not a vertex/triangle count or wrapper comparison.
    let six_volume = restored.triangles.iter().fold(Real::zero(), |volume, t| {
        let [a, b, c] = t.indices().map(|i| restored.positions[i].to_vector());
        volume + a.dot(&b.cross(&c))
    });
    assert_eq!(six_volume, Real::from(6));
    let removed = restored.try_difference(&cube).unwrap();
    assert!(removed.positions.is_empty() && removed.triangles.is_empty());
}

#[test]
fn proven_degeneracies_are_composable_empty_sets() {
    for result in [
        solid::cube(Real::zero()),
        solid::cuboid(Real::one(), Real::zero(), Real::one()),
        solid::sphere(Real::zero(), 8, 4),
        solid::cylinder(Real::zero(), Real::one(), 8),
        solid::cylinder(Real::one(), Real::zero(), 8),
        solid::ellipsoid(Real::one(), Real::one(), Real::zero(), 8, 4),
        solid::frustum(Real::zero(), Real::zero(), Real::one(), 8),
        solid::frustum_between(
            Point3::origin(),
            Point3::origin(),
            Real::one(),
            Real::one(),
            8,
        ),
        solid::octahedron(Real::zero()),
        solid::icosahedron(Real::zero()),
        solid::arrow(
            Point3::origin(),
            Vector3::from_xyz(Real::zero(), Real::zero(), Real::zero()),
            8,
            false,
        ),
    ] {
        assert_canonical_empty(result.unwrap());
    }
}

#[test]
fn zero_size_does_not_conceal_invalid_or_undecided_parameters() {
    for result in [
        solid::cuboid(Real::zero(), -Real::one(), Real::one()),
        solid::ellipsoid(Real::zero(), -Real::one(), Real::one(), 8, 4),
        solid::frustum(Real::zero(), -Real::one(), Real::zero(), 8),
    ] {
        assert!(matches!(
            result,
            Err(ValidationError::NegativeParameter { .. })
        ));
    }
    for result in [
        solid::cuboid(Real::zero(), unresolved_zero(), Real::one()),
        solid::ellipsoid(Real::zero(), unresolved_zero(), Real::one(), 8, 4),
        solid::frustum(Real::zero(), unresolved_zero(), Real::zero(), 8),
    ] {
        assert!(matches!(
            result,
            Err(ValidationError::PredicateUndecided { .. })
        ));
    }
}

#[test]
fn representable_counts_with_unrepresentable_byte_sizes_fail_before_allocation() {
    let count = isize::MAX as usize / std::mem::size_of::<Point3>() + 1;
    assert!(matches!(
        solid::sphere(Real::one(), count, 2),
        Err(ValidationError::CapacityOverflow)
    ));
    assert!(matches!(
        solid::cylinder(Real::one(), Real::one(), count),
        Err(ValidationError::CapacityOverflow)
    ));
}

#[test]
fn directed_negative_height_keeps_its_exact_solid() {
    for height in [-3, 3] {
        let mesh = solid::cylinder(Real::one(), Real::from(height), 4).unwrap();
        assert!(mesh.is_closed_manifold());
        let six_volume = mesh.triangles.iter().fold(Real::zero(), |volume, t| {
            let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
            volume + a.dot(&b.cross(&c))
        });
        assert_eq!(six_volume, Real::from(36)); // diamond area 2, height 3
        for p in mesh.positions.iter() {
            assert!(p.z == Real::zero() || p.z == Real::from(height));
        }
    }
}

#[test]
fn failed_construction_is_not_retained_and_does_not_poison_recovery() {
    use solid::cache::{self, CachePolicy};
    for policy in [CachePolicy::DEFAULT, CachePolicy::DISABLED] {
        cache::with_policy(policy, || {
            cache::clear();
            for _ in 0..2 {
                assert!(solid::cube(unresolved_zero()).is_err());
                assert!(solid::sphere(Real::one(), usize::MAX, 2).is_err());
                assert_eq!(cache::usage().primitives, 0);
            }
            let cube = solid::cube(Real::one()).unwrap();
            assert_eq!(
                cube.positions[6],
                Point3::new(Real::one(), Real::one(), Real::one())
            );
        });
    }
}

#[test]
fn stronger_scalar_proof_can_recover_the_same_constructor() {
    use solid::cache;
    cache::clear();
    let pi = Real::pi();
    let lower = pi.certified_rational_interval(-1024).unwrap()[0].clone();
    let width = &pi - Real::from(lower);
    // Source caches may already supply a stronger proof. Cold success is fine;
    // an undecided result must not be retained as a permanent failure.
    if let Err(error) = solid::cube(width.clone()) {
        assert!(matches!(error, ValidationError::PredicateUndecided { .. }));
        assert_eq!(cache::usage().primitives, 0);
    }
    assert_eq!(
        width.certified_sign_until(-1100).sign(),
        Some(hyperreal::RealSign::Positive)
    );
    let mesh = solid::cube(width.clone()).unwrap();
    assert_eq!(
        mesh.positions[6],
        Point3::new(width.clone(), width.clone(), width)
    );
    assert_eq!(cache::usage().primitives, 1);
}

#[cfg(feature = "curve")]
#[test]
fn torus_unsupported_undecided_invalid_and_empty_are_distinct() {
    for major in [Real::one(), Real::zero()] {
        assert!(matches!(
            solid::torus(major, Real::one(), 8, 4),
            Err(ValidationError::UnsupportedPrimitive { .. })
        ));
    }
    assert!(matches!(
        solid::torus(Real::one() + unresolved_zero(), Real::one(), 8, 4),
        Err(ValidationError::PredicateUndecided { .. })
    ));
    assert!(matches!(
        solid::torus(-Real::one(), Real::zero(), 8, 4),
        Err(ValidationError::NegativeParameter { .. })
    ));
    assert_canonical_empty(solid::torus(Real::one(), Real::zero(), 8, 4).unwrap());
}
