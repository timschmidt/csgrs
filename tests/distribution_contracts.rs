use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    solid::{
        self, SolidExt,
        cache::{self, CachePolicy},
    },
};
use hypermesh::{BooleanOp, HypermeshError, Point3, Triangle, Vector3};
use std::{error::Error, sync::Arc};

fn r(n: i32) -> Real {
    n.into()
}
fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}
fn exact(a: &Real, b: &Real) {
    assert_eq!(a.certified_eq_until(b, -1024).as_bool(), Some(true));
}
fn canonical_empty(mesh: &TriangleMesh) -> bool {
    mesh.positions.is_empty() && mesh.triangles.is_empty()
}
fn families(
    mesh: &TriangleMesh,
    count: usize,
    context: &GeometryContext,
) -> [Result<GeometryOutcome<TriangleMesh>, ValidationError>; 3] {
    [
        solid::distribute_linear_with_context(mesh, count, Vector3::x(), r(3), context),
        solid::distribute_grid_with_context(mesh, 1, count, r(3), r(3), context),
        solid::distribute_arc_with_context(mesh, count, r(3), r(0), r(90), context),
    ]
}
fn volume(mesh: &TriangleMesh, expected: Real) {
    // Divergence theorem with F=(0,0,z). Each fixture's exact face flux is
    // rational. Prove every term before summing, avoiding a large unreduced
    // expression; a declined proof fails rather than accepting Unknown.
    let six = mesh.triangles.iter().fold(r(0), |sum, t| {
        let [a, b, c] = t.indices().map(|i| &mesh.positions[i]);
        let flux = (&a.z + &b.z + &c.z) * &(b - a).cross(&(c - a)).0[2];
        sum + Real::from(
            flux.exact_rational_normal_form()
                .expect("every fixture face must have a proved rational volume contribution"),
        )
    });
    exact(&six, &(r(6) * expected));
}
fn contains(mesh: &TriangleMesh, x: Real, y: Real, z: Real, expected: bool) {
    assert_eq!(
        solid::contains_point(mesh, &Point3::new(x, y, z)).unwrap(),
        expected
    );
}
fn topology(mesh: &TriangleMesh) {
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
}

#[test]
fn source_validation_precedes_zero_one_and_disjoint_copy_shortcuts() {
    let cube = solid::cube(r(1)).unwrap();
    let open = TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(r(1), r(0), r(0)),
            Point3::new(r(0), r(1), r(0)),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    let mut degenerate = cube.clone();
    Arc::make_mut(&mut degenerate.triangles)[0] = Triangle::new(0, 0, 0);
    let mut unbalanced = cube.clone();
    let triangle = &mut Arc::make_mut(&mut unbalanced.triangles)[0];
    std::mem::swap(&mut triangle.v0, &mut triangle.v2);
    let sources = [
        TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)]),
        TriangleMesh::new(vec![Point3::origin()], vec![]),
        TriangleMesh::new(vec![], vec![Triangle::new(0, 0, 0)]),
        open,
        degenerate,
        unbalanced,
    ];
    cache::clear();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for source in &sources {
            for count in [0, 1, 2] {
                for result in families(source, count, &context) {
                    let error = result.unwrap_err();
                    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
                    assert!(
                        error
                            .source()
                            .unwrap()
                            .downcast_ref::<HypermeshError>()
                            .is_some()
                    );
                }
            }
            assert!(
                solid::distribute_grid_with_context(source, 0, 2, r(3), r(3), &context)
                    .is_err()
            );
            for result in [
                solid::distribute_linear(source, 2, Vector3::x(), r(3)),
                solid::distribute_grid(source, 1, 2, r(3), r(3)),
                solid::distribute_arc(source, 2, r(3), r(0), r(90)),
            ] {
                assert!(matches!(result, Err(ValidationError::Mesh(_))));
            }
        }
    }
    assert_eq!(cache::usage().distributions, 0);
}

#[test]
fn canonical_empty_and_zero_copy_outputs_remain_composable() {
    let cube = solid::cube(r(1)).unwrap();
    let empty = solid::empty();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for source in [&cube, &empty] {
            for result in families(source, 0, &context) {
                let outcome = result.unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                assert!(canonical_empty(&outcome.value));
                let restored = outcome.value.try_union(&cube).unwrap();
                volume(&restored, r(1));
                let removed = restored.try_difference(&cube).unwrap();
                assert!(canonical_empty(&removed));
                for result in families(&removed, 2, &context) {
                    assert!(canonical_empty(&result.unwrap().value));
                }
            }
        }
        // Empty input requires no copy buffers even for a very large count.
        for result in families(&empty, usize::MAX, &context) {
            assert!(canonical_empty(&result.unwrap().value));
        }
    }
    for source in [&cube, &empty] {
        for count in [0, 1, 2] {
            assert!(matches!(
                solid::distribute_linear(source, count, Vector3::zero(), r(1)),
                Err(ValidationError::ZeroParameter {
                    name: "distribution direction"
                })
            ));
        }
    }
}

#[test]
fn capacity_exhaustion_is_typed_and_leaves_subsequent_work_available() {
    let cube = solid::cube(r(1)).unwrap();
    cache::clear();
    for result in families(&cube, usize::MAX, &GeometryContext::STRICT) {
        assert_eq!(
            result.unwrap_err().kind(),
            ValidationErrorKind::ResourceExhausted
        );
    }
    assert_eq!(
        solid::distribute_grid(&cube, usize::MAX, 2, r(2), r(2)).unwrap_err(),
        ValidationError::CapacityOverflow
    );
    assert_eq!(cache::usage().distributions, 0);
    let recovered = solid::distribute_linear(&cube, 2, Vector3::x(), r(2)).unwrap();
    volume(&recovered, r(2));
    topology(&recovered);
}

#[test]
fn native_causes_and_consumed_certainty_survive_strict_approximate_strict() {
    let cube = solid::cube(r(1)).unwrap();
    let s = Real::e().sin();
    let c = Real::e().cos();
    let step = r(1) + &s * &s + &c * &c - r(1);
    let build = |context| {
        solid::distribute_grid_with_context(&cube, 1, 2, step.clone(), r(1), context)
    };
    cache::clear();
    for policy in [CachePolicy::DEFAULT, CachePolicy::DISABLED] {
        cache::with_policy(policy, || {
            for _ in 0..2 {
                let error = build(&GeometryContext::STRICT).unwrap_err();
                assert_eq!(error.kind(), ValidationErrorKind::Undecided);
                assert!(
                    error
                        .source()
                        .unwrap()
                        .downcast_ref::<HypermeshError>()
                        .is_some()
                );
                let approximate = build(&GeometryContext::APPROXIMATE_512).unwrap();
                assert_eq!(
                    approximate.certainty,
                    GeometryCertainty::Approximate512Consumed
                );
            }
            assert_eq!(
                solid::distribute_grid(&cube, 1, 2, step.clone(), r(1))
                    .unwrap_err()
                    .kind(),
                ValidationErrorKind::Undecided
            );
        });
    }
}

#[test]
fn original_full_distributions_satisfy_independent_volume_membership_and_composition() {
    let cube = solid::cube(r(1)).unwrap();
    let linear = solid::distribute_linear(&cube, 8, Vector3::x(), r(2)).unwrap();
    let grid = solid::distribute_grid(&cube, 4, 4, r(2), r(2)).unwrap();
    let arc = solid::distribute_arc(&cube, 12, r(10), r(0), r(330)).unwrap();
    for i in 0..8 {
        contains(&linear, r(2 * i) + q(1, 2), q(1, 2), q(1, 2), true);
        contains(&linear, r(2 * i) + q(3, 2), q(1, 2), q(1, 2), false);
    }
    for row in 0..4 {
        for column in 0..4 {
            contains(
                &grid,
                r(2 * column) + q(1, 2),
                r(2 * row) + q(1, 2),
                q(1, 2),
                true,
            );
        }
    }
    contains(&grid, q(3, 2), q(1, 2), q(1, 2), false);
    for i in 0..12 {
        let theta = Real::pi() * q(i, 6);
        let cosine = theta.clone().cos();
        let sine = theta.sin();
        // The analytically rotated center of a translated unit cube.
        contains(
            &arc,
            q(21, 2) * &cosine - q(1, 2) * &sine,
            q(21, 2) * sine + q(1, 2) * cosine,
            q(1, 2),
            true,
        );
    }
    contains(&arc, r(0), r(0), q(1, 2), false);
    for (mesh, count) in [(linear, 8), (grid, 16), (arc, 12)] {
        volume(&mesh, r(count));
        topology(&mesh);
        let accumulated = solid::empty().try_union(&mesh).unwrap();
        volume(&accumulated, r(count));
        topology(&accumulated);
        let removed = accumulated.try_difference(&mesh).unwrap();
        assert!(canonical_empty(&removed));
    }
}

#[test]
fn touching_overlapping_signed_and_single_arc_layouts_have_the_expected_sets() {
    let cube = solid::cube(r(1)).unwrap();
    for (spacing, expected) in [(r(1), r(2)), (q(1, 2), q(3, 2)), (r(0), r(1))] {
        let result = solid::distribute_linear(&cube, 2, Vector3::x(), spacing).unwrap();
        volume(&result, expected);
        topology(&result);
        contains(&result, q(1, 4), q(1, 2), q(1, 2), true);
    }
    let directed =
        solid::distribute_linear(&cube, 2, Vector3::from_xyz(r(3), r(4), r(0)), r(-5))
            .unwrap();
    contains(&directed, q(-5, 2), q(-7, 2), q(1, 2), true);
    contains(&directed, q(1, 2), q(1, 2), q(1, 2), true);
    volume(&directed, r(2));
    let midpoint = solid::distribute_arc(&cube, 1, r(-3), r(0), r(180)).unwrap();
    let bounds = solid::try_bounding_box(&midpoint).unwrap().unwrap();
    assert_eq!(bounds.mins, Point3::new(r(-1), r(-3), r(0)));
    assert_eq!(bounds.maxs, Point3::new(r(0), r(-2), r(1)));
    volume(&midpoint, r(1));
    topology(&midpoint);
}

#[test]
fn concatenation_preserves_multiplicity_and_pwn_distributions_accept_it() {
    let cube = solid::cube(r(1)).unwrap();
    let duplicate = solid::merge(&[cube.clone(), cube.clone()]).unwrap();
    assert_eq!(duplicate.triangles.len(), 24);
    volume(&duplicate, r(2)); // Oriented multiplicity, not the regularized-set volume.
    let distributed = solid::distribute_linear(&duplicate, 2, Vector3::x(), r(3)).unwrap();
    let regularized = solid::boolean(&distributed, &solid::empty(), BooleanOp::Union).unwrap();
    volume(&regularized, r(2));
    topology(&regularized);
    let touching = cube.translated(r(1), r(1), r(1)).unwrap();
    let balanced = solid::merge(&[cube, touching]).unwrap();
    let outcome = solid::distribute_grid_with_context(
        &balanced,
        1,
        1,
        r(0),
        r(0),
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    volume(&outcome.value, r(2));
}

#[test]
fn cache_reuse_eviction_disable_and_mutation_preserve_validity() {
    let cube = solid::cube(r(1)).unwrap();
    cache::clear();
    cache::with_policy(
        CachePolicy {
            distributions: 1,
            merges: 1,
            ..CachePolicy::DEFAULT
        },
        || {
            let cold = solid::distribute_linear(&cube, 2, Vector3::x(), r(3)).unwrap();
            let mut warm = solid::distribute_linear(&cube, 2, Vector3::x(), r(3)).unwrap();
            assert!(Arc::ptr_eq(&cold.positions, &warm.positions));
            Arc::make_mut(&mut warm.positions)[0].x = r(99);
            assert_eq!(
                cold,
                solid::distribute_linear(&cube, 2, Vector3::x(), r(3)).unwrap()
            );
            solid::distribute_grid(&cube, 1, 2, r(3), r(3)).unwrap();
            let evicted = solid::distribute_linear(&cube, 2, Vector3::x(), r(3)).unwrap();
            assert_eq!(cold, evicted);
            let disabled = cache::with_policy(CachePolicy::DISABLED, || {
                solid::distribute_linear(&cube, 2, Vector3::x(), r(3))
            })
            .unwrap();
            assert_eq!(cold, disabled);
            let mut edited = cube.clone();
            Arc::make_mut(&mut edited.triangles)[0].v0 = usize::MAX;
            assert!(matches!(
                solid::distribute_linear(&edited, 2, Vector3::x(), r(3)),
                Err(ValidationError::Mesh(
                    HypermeshError::VertexIndexOutOfBounds { .. }
                ))
            ));
            assert!(matches!(
                solid::merge(&[cube.clone(), edited]),
                Err(ValidationError::Mesh(
                    HypermeshError::VertexIndexOutOfBounds { .. }
                ))
            ));
            assert_eq!(
                cold,
                solid::distribute_linear(&cube, 2, Vector3::x(), r(3)).unwrap()
            );
        },
    );
}
