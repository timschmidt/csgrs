#![cfg(feature = "earcut")]

#[path = "../src/test_uncertainty.rs"]
mod test_uncertainty;

use hypertri::{
    Error, Point2, PredicatePolicy, Real, Sign, TriangulationCertainty, TriangulationContext,
    triangulate_simple_ring, triangulate_simple_ring_around,
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
    let left = Real::from(3)
        + test_uncertainty::terminal_zero(
            (&sin * &sin + &cos * &cos + Real::from(2)) - Real::from(3),
        );
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

/// The unordered vertex sets of a triangulation, in original ring indices.
fn triangle_sets(triangles: &[usize], original: impl Fn(usize) -> usize) -> Vec<[usize; 3]> {
    let mut sets = triangles
        .as_chunks::<3>()
        .0
        .iter()
        .map(|triangle| {
            let mut set = triangle.map(&original);
            set.sort_unstable();
            set
        })
        .collect::<Vec<_>>();
    sets.sort_unstable();
    sets
}

/// Every boundary edge occurs in its authored direction exactly once.
fn assert_authored_boundary(ring_len: usize, triangles: &[usize]) {
    for i in 0..ring_len {
        let edge = [i, (i + 1) % ring_len];
        let count = triangles
            .as_chunks::<3>()
            .0
            .iter()
            .flat_map(|t| [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]])
            .filter(|e| *e == edge)
            .count();
        assert_eq!(count, 1, "{edge:?}");
    }
}

#[test]
fn apex_ring_fans_from_the_apex_and_cuts_only_its_hidden_pockets() {
    // A strip whose bottom is one straight edge from L = 0 to R = 1 and whose
    // top zigzags through apexes 2, 4, 6 and valleys 3, 5. Every vertex L
    // sees is joined to it; the apexes it cannot see are cut off as ears.
    let strip = points(&[(0, 0), (10, 0), (9, 2), (7, 1), (5, 2), (3, 1), (1, 2)]);
    let expected = vec![[0, 1, 3], [0, 3, 5], [0, 5, 6], [1, 2, 3], [3, 4, 5]];
    for context in [STRICT, APPROX] {
        for shift in 0..strip.len() {
            let mut ring = strip.clone();
            ring.rotate_left(shift);
            let original = |index: usize| (index + shift) % strip.len();
            let apex = (strip.len() - shift) % strip.len();
            let outcome = triangulate_simple_ring_around(&context, &ring, apex).unwrap();
            assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
            assert_eq!(outcome.value.winding, Sign::Positive);
            assert_eq!(
                triangle_sets(&outcome.value.triangles, original),
                expected,
                "shift {shift}"
            );
            assert_authored_boundary(ring.len(), &outcome.value.triangles);
            assert_eq!(twice_area(&ring, &outcome.value.triangles), Real::from(28));
        }
        // The same ring traversed backwards keeps its own winding and fans
        // at L, now index 6.
        let mut reversed = strip.clone();
        reversed.reverse();
        let outcome = triangulate_simple_ring_around(&context, &reversed, 6).unwrap();
        assert_eq!(outcome.value.winding, Sign::Negative);
        assert_authored_boundary(reversed.len(), &outcome.value.triangles);
        let at_apex = outcome
            .value
            .triangles
            .as_chunks::<3>()
            .0
            .iter()
            .filter(|triangle| triangle.contains(&6))
            .count();
        assert_eq!(at_apex, 3);
        assert_eq!(
            twice_area(&reversed, &outcome.value.triangles),
            Real::from(-28)
        );
    }
}

#[test]
fn apex_ring_fans_a_convex_ring_entirely_and_keeps_collinear_vertices() {
    // A convex ring with one collinear vertex, index 1, on its bottom edge.
    let ring = points(&[(0, 0), (2, 0), (4, 0), (5, 2), (4, 4), (0, 4), (-1, 2)]);
    for context in [STRICT, APPROX] {
        for apex in 0..ring.len() {
            let outcome = triangulate_simple_ring_around(&context, &ring, apex).unwrap();
            assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
            let triangles = outcome.value.triangles.as_chunks::<3>().0;
            assert_eq!(triangles.len(), ring.len() - 2);
            // Every apex sees the whole convex ring and is in every
            // triangle, except the two ends of the collinear bottom edge:
            // the triangle on the far half of that edge cannot contain them.
            let at_apex = triangles.iter().filter(|t| t.contains(&apex)).count();
            let expected = if apex == 0 || apex == 2 {
                ring.len() - 3
            } else {
                ring.len() - 2
            };
            assert_eq!(at_apex, expected, "apex {apex}: {triangles:?}");
            assert_authored_boundary(ring.len(), &outcome.value.triangles);
            assert_eq!(
                twice_area(&ring, &outcome.value.triangles),
                Real::from(2 * 4 * 4 + 4 + 4)
            );
        }
    }
}

#[test]
fn apex_ring_rejects_an_absent_apex_and_invalid_boundaries() {
    let square = points(&[(0, 0), (2, 0), (2, 2), (0, 2)]);
    for context in [STRICT, APPROX] {
        assert!(matches!(
            triangulate_simple_ring_around(&context, &square, 4),
            Err(Error::InvalidInput { .. })
        ));
        // A closing repeat of the first point is not a separate apex.
        let mut closed = square.clone();
        closed.push(square[0].clone());
        assert!(matches!(
            triangulate_simple_ring_around(&context, &closed, 4),
            Err(Error::InvalidInput { .. })
        ));
        assert!(triangulate_simple_ring_around(&context, &closed, 0).is_ok());
        let crossing = points(&[(0, 0), (2, 2), (0, 2), (2, 0)]);
        assert!(matches!(
            triangulate_simple_ring_around(&context, &crossing, 0),
            Err(Error::InvalidInput { .. })
        ));
    }
}

#[test]
fn apex_ring_qualified_closure_is_not_a_strict_proof() {
    let sin = Real::e().sin();
    let cos = Real::e().cos();
    let left = Real::from(3)
        + test_uncertainty::terminal_zero(
            (&sin * &sin + &cos * &cos + Real::from(2)) - Real::from(3),
        );
    let ring = vec![
        Point2::new(left.clone(), Real::zero()),
        Point2::new(&left + Real::one(), Real::zero()),
        Point2::new(&left + Real::one(), Real::one()),
        Point2::new(left, Real::one()),
        Point2::new(Real::from(3), Real::zero()),
    ];
    for context in [STRICT, APPROX, STRICT] {
        let result = triangulate_simple_ring_around(&context, &ring, 0);
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

fn exact(value: Real) -> hyperreal::Rational {
    value
        .exact_rational()
        .expect("rational fixtures have rational areas")
}

/// Twice the signed area of one triangle.
fn twice_triangle_area(points: &[Point2], triangle: [usize; 3]) -> hyperreal::Rational {
    let [a, b, c] = triangle.map(|index| &points[index]);
    exact((&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x))
}

/// Whether `point` lies strictly between `start` and `end` on their segment.
fn strictly_inside_segment(point: &Point2, start: &Point2, end: &Point2) -> bool {
    let [dx, dy] = [&end.x - &start.x, &end.y - &start.y];
    let [px, py] = [&point.x - &start.x, &point.y - &start.y];
    let zero = hyperreal::Rational::zero();
    let along = exact(&px * &dx + &py * &dy);
    exact(&dx * &py - &dy * &px) == zero && along > zero && along < exact(&dx * &dx + &dy * &dy)
}

/// Checks a triangulation of a simple ring against the ring alone: `n - 2`
/// triangles, each with the ring's winding and nonzero area, summing to the
/// ring's area; every boundary edge once in its authored direction and every
/// other edge once in each direction; and no ring vertex inside an edge.
fn assert_tiles_ring(ring: &[Point2], triangles: &[usize], winding: Sign, label: &str) {
    let triangles = triangles.as_chunks::<3>().0;
    assert_eq!(triangles.len(), ring.len() - 2, "{label}");
    let zero = hyperreal::Rational::zero();
    let mut total = zero.clone();
    let mut edges = Vec::new();
    for &triangle in triangles {
        let area = twice_triangle_area(ring, triangle);
        match winding {
            Sign::Positive => assert!(area > zero, "{label}: {triangle:?}"),
            Sign::Negative => assert!(area < zero, "{label}: {triangle:?}"),
            Sign::Zero => panic!("{label}: a ring has nonzero winding"),
        }
        total = total + area;
        let [a, b, c] = triangle;
        edges.extend([[a, b], [b, c], [c, a]]);
    }
    let ring_area = (0..ring.len()).fold(zero.clone(), |sum, i| {
        let [a, b] = [&ring[i], &ring[(i + 1) % ring.len()]];
        sum + exact(&a.x * &b.y - &a.y * &b.x)
    });
    assert_eq!(total, ring_area, "{label}");
    let count = |edge: [usize; 2]| edges.iter().filter(|&&e| e == edge).count();
    for &[from, to] in &edges {
        assert_eq!(count([from, to]), 1, "{label}: edge {from} {to}");
        assert_ne!(
            from,
            (to + 1) % ring.len(),
            "{label}: edge {from} {to} runs against the ring"
        );
        if to != (from + 1) % ring.len() {
            assert_eq!(count([to, from]), 1, "{label}: interior edge {from} {to}");
        }
        for (index, point) in ring.iter().enumerate() {
            assert!(
                index == from
                    || index == to
                    || !strictly_inside_segment(point, &ring[from], &ring[to]),
                "{label}: edge {from} {to} passes over vertex {index}"
            );
        }
    }
}

#[test]
fn apex_ring_tiles_every_cyclic_order_orientation_and_apex() {
    let fixtures: &[(&str, &[(i32, i32)])] = &[
        ("concave", &[(3, 1), (1, 1), (1, 3), (0, 3), (0, 0), (3, 0)]),
        // A comb whose two narrow pockets reach almost to the far side.
        (
            "deep pockets",
            &[
                (0, 0),
                (9, 0),
                (9, 9),
                (7, 9),
                (7, 1),
                (6, 1),
                (6, 9),
                (3, 9),
                (3, 1),
                (2, 1),
                (2, 9),
                (0, 9),
            ],
        ),
        // A chevron and a star, whose reflex vertices are apexes in turn.
        ("chevron", &[(0, 0), (4, 2), (8, 0), (4, 8)]),
        (
            "star",
            &[
                (0, -6),
                (1, -1),
                (6, 0),
                (1, 1),
                (0, 6),
                (-1, 1),
                (-6, 0),
                (-1, -1),
            ],
        ),
        // Forward collinear vertices on convex and reflex sides, and a reflex
        // corner between two collinear runs.
        (
            "forward collinear",
            &[
                (0, 0),
                (2, 0),
                (4, 0),
                (4, 1),
                (4, 2),
                (3, 2),
                (2, 2),
                (2, 3),
                (2, 4),
                (1, 4),
                (0, 4),
                (0, 2),
            ],
        ),
        (
            "strip",
            &[(0, 0), (10, 0), (9, 2), (7, 1), (5, 2), (3, 1), (1, 2)],
        ),
    ];
    for &(name, fixture) in fixtures {
        let base = points(fixture);
        let n = base.len();
        for context in [STRICT, APPROX] {
            for reverse in [false, true] {
                let winding = if reverse {
                    Sign::Negative
                } else {
                    Sign::Positive
                };
                let mut order = (0..n).collect::<Vec<_>>();
                if reverse {
                    order.reverse();
                }
                // The triangle sets of each apex, in fixture indices, from the
                // first rotation; every other rotation must reproduce them.
                let mut expected = vec![None; n];
                for shift in 0..n {
                    let mut rotated = order.clone();
                    rotated.rotate_left(shift);
                    let ring = rotated.iter().map(|&i| base[i].clone()).collect::<Vec<_>>();
                    for apex in 0..n {
                        let label =
                            format!("{name}, reverse {reverse}, shift {shift}, apex {apex}");
                        let outcome = triangulate_simple_ring_around(&context, &ring, apex)
                            .unwrap_or_else(|error| panic!("{label}: {error:?}"));
                        assert_eq!(
                            outcome.certainty,
                            TriangulationCertainty::Certified,
                            "{label}"
                        );
                        assert_eq!(outcome.value.winding, winding, "{label}");
                        assert_tiles_ring(&ring, &outcome.value.triangles, winding, &label);
                        assert!(outcome.value.triangles.contains(&apex), "{label}");
                        let sets = triangle_sets(&outcome.value.triangles, |i| rotated[i]);
                        match &expected[rotated[apex]] {
                            None => expected[rotated[apex]] = Some(sets),
                            Some(first) => assert_eq!(first, &sets, "{label}"),
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn apex_ring_rejects_invalid_boundaries_at_every_apex() {
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
            for apex in 0..=fixture.len() {
                assert!(
                    matches!(
                        triangulate_simple_ring_around(&context, &points(fixture), apex),
                        Err(Error::InvalidInput { .. })
                    ),
                    "{fixture:?}, apex {apex}"
                );
            }
        }
    }
}

#[test]
fn apex_ring_keeps_exact_near_collinearity_at_every_apex() {
    // A vertex 10^-12 off the straight bottom edge is reflex or convex, never
    // collinear.
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    for offset in [tiny.clone(), -tiny] {
        let ring = vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), -&offset),
            Point2::new(Real::from(2), Real::zero()),
            Point2::new(Real::from(2), Real::from(2)),
            Point2::new(Real::zero(), Real::from(2)),
        ];
        for context in [STRICT, APPROX] {
            for apex in 0..ring.len() {
                let label = format!("offset {offset:?}, apex {apex}");
                let outcome = triangulate_simple_ring_around(&context, &ring, apex).unwrap();
                assert_eq!(
                    outcome.certainty,
                    TriangulationCertainty::Certified,
                    "{label}"
                );
                assert_tiles_ring(&ring, &outcome.value.triangles, Sign::Positive, &label);
                assert_eq!(
                    twice_area(&ring, &outcome.value.triangles),
                    Real::from(8) + Real::from(2) * &offset,
                    "{label}"
                );
            }
        }
    }
}
