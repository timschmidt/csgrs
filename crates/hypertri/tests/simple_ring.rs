#![cfg(feature = "earcut")]

use hypertri::{
    Error, Point2, PredicatePolicy, Real, Sign, TriangulationCertainty, TriangulationContext,
    triangulate_simple_ring,
};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);
const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn points(values: &[(i32, i32)]) -> Vec<Point2> {
    values
        .iter()
        .map(|&(x, y)| Point2::new(x.into(), y.into()))
        .collect()
}

fn twice_area(points: &[Point2], triangles: &[usize]) -> Real {
    triangles
        .as_chunks::<3>()
        .0
        .iter()
        .fold(Real::zero(), |sum, t| {
            let [a, b, c] = t.map(|i| &points[i]);
            sum + (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x)
        })
}

#[test]
fn concave_ring_retains_authored_orientation_in_every_cyclic_order() {
    let original = points(&[(3, 1), (1, 1), (1, 3), (0, 3), (0, 0), (3, 0)]);
    for context in [STRICT, APPROX] {
        for reverse in [false, true] {
            for shift in 0..original.len() {
                let mut ring = original.clone();
                if reverse {
                    ring.reverse();
                }
                ring.rotate_left(shift);
                let outcome = triangulate_simple_ring(&context, &ring).unwrap();
                assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
                assert_eq!(
                    outcome.value.winding,
                    if reverse {
                        Sign::Negative
                    } else {
                        Sign::Positive
                    }
                );
                let expected = Real::from(if reverse { -10 } else { 10 });
                assert_eq!(
                    twice_area(&ring, &outcome.value.triangles)
                        .certified_eq_until(&expected, -128)
                        .as_bool(),
                    Some(true)
                );
                // Every boundary edge occurs in its authored direction exactly
                // once. This catches an area-preserving orientation mismatch.
                for i in 0..ring.len() {
                    let edge = [i, (i + 1) % ring.len()];
                    let count = outcome
                        .value
                        .triangles
                        .as_chunks::<3>()
                        .0
                        .iter()
                        .flat_map(|t| [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]])
                        .filter(|e| *e == edge)
                        .count();
                    assert_eq!(count, 1);
                }
            }
        }
    }
}

#[test]
fn collinear_vertices_and_explicit_closure_keep_the_same_boundary() {
    for context in [STRICT, APPROX] {
        let mut ring = points(&[(0, 0), (1, 0), (2, 0), (2, 2), (0, 2)]);
        for closed in [false, true] {
            if closed {
                ring.push(ring[0].clone());
            }
            let outcome = triangulate_simple_ring(&context, &ring).unwrap();
            assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
            assert_eq!(outcome.value.triangles.len(), 9);
            for index in 0..5 {
                assert!(outcome.value.triangles.contains(&index));
            }
            assert_eq!(twice_area(&ring, &outcome.value.triangles), Real::from(8));
        }
    }
}

#[test]
fn invalid_boundaries_never_publish_a_partial_triangulation() {
    let fixtures: &[&[(i32, i32)]] = &[
        &[],
        &[(0, 0)],
        &[(0, 0), (1, 0)],
        &[(0, 0), (1, 0), (2, 0)],
        &[(0, 0), (2, 2), (0, 2), (2, 0)],
        &[(0, 0), (3, 3), (0, 2), (2, 0)],
        &[(0, 0), (2, 0), (2, 2), (0, 0), (0, 2)],
        &[(0, 0), (2, 0), (1, 0), (2, 2), (0, 2)],
        &[(0, 0), (2, 0), (2, 0), (2, 2), (0, 2)],
    ];
    for context in [STRICT, APPROX] {
        for fixture in fixtures {
            assert!(
                matches!(
                    triangulate_simple_ring(&context, &points(fixture)),
                    Err(Error::InvalidInput { .. })
                ),
                "{fixture:?}"
            );
        }
        let recovered =
            triangulate_simple_ring(&context, &points(&[(0, 0), (2, 0), (0, 2)])).unwrap();
        assert_eq!(recovered.certainty, TriangulationCertainty::Certified);
    }
}

#[test]
fn qualified_closure_cannot_be_reused_as_a_strict_proof() {
    let sin = Real::e().sin();
    let cos = Real::e().cos();
    let left = &sin * &sin + &cos * &cos + Real::from(2);
    let ring = vec![
        Point2::new(left.clone(), Real::zero()),
        Point2::new(&left + Real::one(), Real::zero()),
        Point2::new(&left + Real::one(), Real::one()),
        Point2::new(left, Real::one()),
        Point2::new(Real::from(3), Real::zero()),
    ];
    for context in [STRICT, APPROX, STRICT] {
        let result = triangulate_simple_ring(&context, &ring);
        if context == APPROX {
            let outcome = result.unwrap();
            assert_eq!(
                outcome.certainty,
                TriangulationCertainty::Approximate512Consumed
            );
            assert_eq!(outcome.value.triangles.len(), 6);
        } else {
            assert!(matches!(result, Err(Error::PredicateUndecided { .. })));
        }
    }
}

#[test]
fn exact_near_collinearity_is_not_tolerance_normalized() {
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let ring = vec![
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::one(), -&tiny),
        Point2::new(Real::from(2), Real::zero()),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::zero(), Real::from(2)),
    ];
    let outcome = triangulate_simple_ring(&STRICT, &ring).unwrap();
    assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
    assert_eq!(
        twice_area(&ring, &outcome.value.triangles),
        Real::from(8) + Real::from(2) * tiny
    );
}
