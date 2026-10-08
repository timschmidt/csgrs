//! Differential checks for grammar cache retention, identity, and certainty.

use std::sync::Arc;

use csgrs::{GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh, solid};
use hyperlattice::Vector3;
use solid::cache::{self, CachePolicy, CacheUsage};

const STRICT: GeometryContext = GeometryContext::STRICT;
const APPROXIMATE: GeometryContext = GeometryContext::APPROXIMATE_512;
const ONE_EACH: CachePolicy = CachePolicy {
    primitives: 1,
    distributions: 1,
    merges: 1,
    flattenings: 1,
    slices: 1,
};

fn assert_mesh_eq(left: &TriangleMesh, right: &TriangleMesh) {
    assert_eq!(left.positions, right.positions);
    assert_eq!(left.triangles, right.triangles);
}

fn assert_outcome_eq(
    left: &GeometryOutcome<TriangleMesh>,
    right: &GeometryOutcome<TriangleMesh>,
) {
    assert_mesh_eq(&left.value, &right.value);
    assert_eq!(left.certainty, right.certainty);
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn primitives_preserve_scalar_structure_with_and_without_retention() {
    cache::clear();
    for width in [
        Real::zero(),
        -Real::one(),
        Real::one(),
        (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap(),
        Real::from(2).sqrt().unwrap(),
        Real::pi(),
        Real::e(),
    ] {
        let cold = solid::cube(width.clone());
        let warm = solid::cube(width.clone());
        assert!(Arc::ptr_eq(&cold.positions, &warm.positions));
        let uncached = cache::with_policy(CachePolicy::DISABLED, || {
            let first = solid::cube(width.clone());
            let second = solid::cube(width.clone());
            assert_mesh_eq(&first, &second);
            if !first.positions.is_empty() {
                assert!(!Arc::ptr_eq(&first.positions, &second.positions));
            }
            assert_eq!(cache::usage(), CacheUsage::default());
            first
        });
        assert_mesh_eq(&cold, &warm);
        assert_mesh_eq(&cold, &uncached);
    }
}

#[test]
fn every_cached_primitive_family_replays_the_same_exact_rows() {
    use hyperlattice::Point3;

    let constructors: &[fn() -> TriangleMesh] = &[
        || solid::cuboid(Real::from(2), Real::from(3), Real::from(4)),
        || solid::sphere(Real::from(2), 8, 4),
        || solid::ellipsoid(Real::from(2), Real::from(3), Real::from(4), 8, 4),
        || {
            solid::frustum_between(
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(Real::zero(), Real::zero(), Real::from(3)),
                Real::from(2),
                Real::one(),
                8,
            )
        },
        || solid::octahedron(Real::from(2)),
        || solid::icosahedron(Real::from(2)),
        #[cfg(feature = "curve")]
        || solid::torus(Real::from(4), Real::one(), 8, 4),
    ];
    for build in constructors {
        cache::clear();
        let cold = build();
        let warm = build();
        assert!(!cold.triangles.is_empty());
        assert!(Arc::ptr_eq(&cold.positions, &warm.positions));
        let uncached = cache::with_policy(CachePolicy::DISABLED, build);
        assert_mesh_eq(&cold, &warm);
        assert_mesh_eq(&cold, &uncached);
    }
}

#[test]
fn fifo_hits_do_not_refresh_insertion_order() {
    cache::clear();
    cache::with_policy(
        CachePolicy {
            primitives: 2,
            ..CachePolicy::DISABLED
        },
        || {
            let first = solid::cube(Real::one());
            let second = solid::cube(Real::from(2));
            assert!(Arc::ptr_eq(
                &first.positions,
                &solid::cube(Real::one()).positions
            ));
            solid::cube(Real::from(3));
            assert_eq!(cache::usage().primitives, 2);
            assert!(Arc::ptr_eq(
                &second.positions,
                &solid::cube(Real::from(2)).positions
            ));
            let rebuilt = solid::cube(Real::one());
            assert!(!Arc::ptr_eq(&first.positions, &rebuilt.positions));
            assert_mesh_eq(&first, &rebuilt);
        },
    );
}

#[test]
fn nested_scopes_shrink_immediately_and_restore_after_unwinding() {
    cache::clear();
    let original = cache::policy();
    cache::with_policy(ONE_EACH, || {
        solid::cube(Real::one());
        let panic = std::panic::catch_unwind(|| {
            cache::with_policy(CachePolicy::DISABLED, || {
                assert_eq!(cache::usage(), CacheUsage::default());
                assert_eq!(cache::policy(), CachePolicy::DISABLED);
                panic!("exercise scoped policy restoration");
            });
        });
        assert!(panic.is_err());
        assert_eq!(cache::policy(), ONE_EACH);
        assert_eq!(cache::usage(), CacheUsage::default());
        cache::with_policy(CachePolicy::DEFAULT, || {
            for width in 1..=4 {
                solid::cube(Real::from(width));
            }
            assert_eq!(cache::usage().primitives, 4);
        });
        assert_eq!(cache::usage().primitives, 1);
        cache::clear();
        assert_eq!(cache::policy(), ONE_EACH);
    });
    assert_eq!(cache::policy(), original);
}

#[test]
fn workers_have_independent_policy_and_retained_storage() {
    cache::with_policy(CachePolicy::DISABLED, || {
        let worker = std::thread::spawn(|| {
            assert_eq!(cache::policy(), CachePolicy::DEFAULT);
            assert_eq!(cache::usage(), CacheUsage::default());
            solid::cube(Real::one());
            assert_eq!(cache::usage().primitives, 1);
        });
        worker.join().unwrap();
        assert_eq!(cache::policy(), CachePolicy::DISABLED);
        assert_eq!(cache::usage(), CacheUsage::default());
    });
}

#[test]
fn merge_keys_include_input_order_and_both_storage_identities() {
    cache::clear();
    let cube = solid::cube(Real::one());
    let large = solid::cube(Real::from(2));
    let inputs = [cube.clone(), large.clone()];
    let merged = solid::merge(&inputs);
    let mut warm = solid::merge(&inputs);
    assert!(Arc::ptr_eq(&merged.positions, &warm.positions));
    Arc::make_mut(&mut warm.positions)[0].x = Real::from(99);
    assert_mesh_eq(&merged, &solid::merge(&inputs));
    for changed in [
        [large, cube.clone()],
        [cube.clone(), warm],
        [cube.clone(), {
            let mut changed = cube;
            Arc::make_mut(&mut changed.triangles).reverse();
            changed
        }],
    ] {
        let cached = solid::merge(&changed);
        let uncached = cache::with_policy(CachePolicy::DISABLED, || solid::merge(&changed));
        assert_mesh_eq(&cached, &uncached);
        assert!(!Arc::ptr_eq(&merged.positions, &cached.positions));
    }
}

#[test]
fn each_distribution_family_preserves_geometry_and_certainty() {
    cache::clear();
    let cube = solid::cube(Real::one());
    for context in [STRICT, APPROXIMATE] {
        for family in 0..3 {
            let build = || {
                match family {
                    0 => solid::distribute_linear_with_context(
                        &cube,
                        2,
                        Vector3::x(),
                        Real::from(3),
                        &context,
                    ),
                    1 => solid::distribute_grid_with_context(
                        &cube,
                        2,
                        2,
                        Real::from(3),
                        Real::from(3),
                        &context,
                    ),
                    _ => solid::distribute_arc_with_context(
                        &cube,
                        2,
                        Real::from(3),
                        Real::zero(),
                        Real::from(90),
                        &context,
                    ),
                }
                .unwrap()
            };
            let cold = build();
            let warm = build();
            assert!(Arc::ptr_eq(&cold.value.positions, &warm.value.positions));
            let uncached = cache::with_policy(CachePolicy::DISABLED, build);
            assert_outcome_eq(&cold, &warm);
            assert_outcome_eq(&cold, &uncached);
            assert_eq!(cold.certainty, GeometryCertainty::Certified);
        }
    }
}

#[test]
fn approximate_distribution_cannot_satisfy_a_strict_request() {
    cache::clear();
    let cube = solid::cube(Real::one());
    let step = Real::one() + unresolved_zero();
    let build = |context| {
        solid::distribute_grid_with_context(&cube, 1, 2, step.clone(), Real::one(), context)
    };
    let cold = build(&APPROXIMATE).unwrap();
    let warm = build(&APPROXIMATE).unwrap();
    assert_eq!(cold.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(Arc::ptr_eq(&cold.value.positions, &warm.value.positions));
    assert!(build(&STRICT).is_err());
    assert_eq!(cache::usage().distributions, 1);
    let uncached = cache::with_policy(CachePolicy::DISABLED, || {
        assert!(build(&STRICT).is_err());
        build(&APPROXIMATE).unwrap()
    });
    assert_outcome_eq(&cold, &warm);
    assert_outcome_eq(&cold, &uncached);
}

#[test]
fn all_active_caches_obey_their_limits_and_release_entries() {
    cache::clear();
    cache::with_policy(ONE_EACH, || {
        for width in 1..=3 {
            let cube = solid::cube(Real::from(width));
            solid::merge(&[cube.clone(), cube.clone()]);
            solid::distribute_linear_with_context(
                &cube,
                2,
                Vector3::x(),
                Real::from(4),
                &STRICT,
            )
            .unwrap();
            #[cfg(feature = "curve")]
            {
                solid::flatten_with_context(&cube, &STRICT).unwrap();
                solid::slice_z_with_context(&cube, Real::zero(), &STRICT).unwrap();
            }
            assert_eq!(
                cache::usage(),
                CacheUsage {
                    primitives: 1,
                    distributions: 1,
                    merges: 1,
                    flattenings: usize::from(cfg!(feature = "curve")),
                    slices: usize::from(cfg!(feature = "curve")),
                }
            );
        }
        cache::with_policy(CachePolicy::DISABLED, || {
            assert_eq!(cache::usage(), CacheUsage::default());
        });
        assert_eq!(cache::usage(), CacheUsage::default());
    });
}

#[cfg(feature = "curve")]
#[test]
fn curve_results_preserve_boundaries_and_context_when_warm_or_disabled() {
    use hyperlattice::Point3;
    use hypermesh::Triangle;

    cache::clear();
    let cube = solid::cube(Real::one());
    let z = unresolved_zero();
    let sliced = solid::slice_z_with_context(&cube, z.clone(), &APPROXIMATE).unwrap();
    let warm = solid::slice_z_with_context(&cube, z.clone(), &APPROXIMATE).unwrap();
    assert_eq!(sliced.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(solid::slice_z_with_context(&cube, z.clone(), &STRICT).is_err());
    let uncached = cache::with_policy(CachePolicy::DISABLED, || {
        solid::slice_z_with_context(&cube, z.clone(), &APPROXIMATE).unwrap()
    });
    for result in [warm, uncached] {
        assert_eq!(sliced.certainty, result.certainty);
        assert_eq!(sliced.value.0, result.value.0);
        assert_eq!(sliced.value.1, result.value.1);
        assert!(sliced.value.2.is_empty() && result.value.2.is_empty());
    }
    let uncertain = TriangleMesh::new(
        vec![
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::one(), Real::zero()),
            Point3::new(Real::from(2), Real::from(2) + unresolved_zero(), Real::zero()),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    for mesh in [cube, uncertain] {
        let cold = solid::flatten_with_context(&mesh, &APPROXIMATE).unwrap();
        let warm = solid::flatten_with_context(&mesh, &APPROXIMATE).unwrap();
        if cold.certainty == GeometryCertainty::Approximate512Consumed {
            assert!(solid::flatten_with_context(&mesh, &STRICT).is_err());
        }
        let uncached = cache::with_policy(CachePolicy::DISABLED, || {
            solid::flatten_with_context(&mesh, &APPROXIMATE).unwrap()
        });
        assert_eq!(cold, warm);
        assert_eq!(cold, uncached);
    }
}
