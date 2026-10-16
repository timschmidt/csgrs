#![cfg(feature = "cdt")]

//! Common-cell regression extracted from the retained 26-box Boolean fixture.
//! Coordinates are exact integer multiples of its x=3 projection. Outside
//! constraints differ; the bounded five-vertex cell and its vertex order agree.

use hypertri::{
    Constraint, ExactPoint, PredicatePolicy, Real, TriangulationCertainty, TriangulationContext,
    cdt::constrained_triangulation_convex_hull,
};
use std::collections::BTreeSet;

const POINTS: &[(usize, [i64; 2])] = &[
    (9, [60, 0]),
    (15, [60, 60]),
    (16, [30, 0]),
    (17, [120, 0]),
    (23, [120, 60]),
    (24, [30, 30]),
    (25, [120, 30]),
    (31, [120, 120]),
    (32, [90, 30]),
    (33, [150, 30]),
    (39, [150, 120]),
    (40, [90, 0]),
    (46, [90, 60]),
    (49, [110, 60]),
    (54, [120, 40]),
    (64, [60, 20]),
    (65, [60, 30]),
    (72, [75, 30]),
    (73, [90, 40]),
    (74, [102, 48]),
    (81, [120, 75]),
    (83, [90, 90]),
];

struct Face {
    ids: &'static [usize],
    constraints: &'static [[usize; 2]],
    double_area: i128,
}

fn faces() -> [Face; 3] {
    [
        Face {
            ids: &[9, 16, 17, 23, 25, 32, 40, 54, 64, 72, 73, 74],
            constraints: &[
                [9, 16],
                [9, 40],
                [9, 64],
                [16, 64],
                [17, 25],
                [17, 40],
                [23, 54],
                [23, 74],
                [25, 32],
                [25, 54],
                [32, 72],
                [32, 73],
                [32, 74],
                [64, 72],
                [72, 73],
                [73, 74],
            ],
            double_area: 5400,
        },
        Face {
            ids: &[15, 23, 24, 25, 31, 32, 46, 49, 54, 65, 72, 73, 74, 81, 83],
            constraints: &[
                [15, 24],
                [15, 46],
                [15, 65],
                [15, 83],
                [23, 49],
                [23, 54],
                [23, 74],
                [23, 81],
                [24, 65],
                [25, 32],
                [25, 54],
                [31, 81],
                [31, 83],
                [32, 72],
                [32, 73],
                [32, 74],
                [46, 49],
                [46, 73],
                [46, 83],
                [49, 74],
                [49, 81],
                [65, 72],
                [72, 73],
                [73, 74],
            ],
            double_area: 8100,
        },
        Face {
            ids: &[23, 25, 32, 33, 39, 49, 54, 74, 81],
            constraints: &[
                [23, 49],
                [23, 54],
                [23, 74],
                [23, 81],
                [25, 32],
                [25, 33],
                [25, 54],
                [32, 74],
                [33, 39],
                [39, 81],
                [49, 74],
                [49, 81],
            ],
            double_area: 5400,
        },
    ]
}

fn coordinates(index: usize) -> [i64; 2] {
    POINTS.iter().find(|(id, _)| *id == index).unwrap().1
}

fn twice_area(triangle: [usize; 3]) -> i128 {
    let [a, b, c] = triangle.map(|index| coordinates(index).map(i128::from));
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

#[test]
fn common_cells_retain_one_triangulation_despite_surrounding_constraints() {
    let cell = [23, 25, 32, 54, 74];
    let expected = BTreeSet::from([[23, 32, 54], [23, 32, 74], [25, 32, 54]]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = TriangulationContext::new(policy);
        for face in faces() {
            let points = face
                .ids
                .iter()
                .map(|&id| {
                    let [x, y] = coordinates(id);
                    ExactPoint::new(Real::from(x), Real::from(y))
                })
                .collect::<Vec<_>>();
            let original = face
                .constraints
                .iter()
                .map(|edge| {
                    Constraint::new(
                        face.ids.binary_search(&edge[0]).unwrap(),
                        face.ids.binary_search(&edge[1]).unwrap(),
                    )
                })
                .collect::<Vec<_>>();
            for order in 0..3 {
                let mut constraints = original.clone();
                if order == 1 {
                    constraints.reverse();
                }
                if order == 2 {
                    constraints.rotate_left(3);
                }
                let outcome =
                    constrained_triangulation_convex_hull(&context, &points, &constraints)
                        .expect("certified common-cell triangulation");
                assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
                let triangles = outcome
                    .value
                    .iter()
                    .map(|triangle| triangle.map(|index| face.ids[index]))
                    .collect::<Vec<_>>();
                // Independent integer area oracle, separate from shared topology validation.
                assert!(triangles.iter().all(|&triangle| twice_area(triangle) > 0));
                assert_eq!(
                    triangles
                        .iter()
                        .map(|&triangle| twice_area(triangle))
                        .sum::<i128>(),
                    face.double_area
                );
                let common = triangles
                    .into_iter()
                    .filter(|triangle| triangle.iter().all(|id| cell.contains(id)))
                    .map(|mut triangle| {
                        triangle.sort_unstable();
                        triangle
                    })
                    .collect::<BTreeSet<_>>();
                assert_eq!(
                    common, expected,
                    "point IDs {:?}, constraint order {order}",
                    face.ids
                );
                assert_eq!(
                    common
                        .iter()
                        .map(|&triangle| twice_area(triangle).abs())
                        .sum::<i128>(),
                    1080
                );
            }
        }
    }
}

fn next_order(order: &mut [usize; 5]) -> bool {
    let Some(pivot) = (0..4).rev().find(|&index| order[index] < order[index + 1]) else {
        return false;
    };
    let successor = (pivot + 1..5)
        .rev()
        .find(|&index| order[pivot] < order[index])
        .unwrap();
    order.swap(pivot, successor);
    order[pivot + 1..].reverse();
    true
}

fn edges(triangles: &[[usize; 3]]) -> BTreeSet<[usize; 2]> {
    triangles
        .iter()
        .flat_map(|&[a, b, c]| [[a, b], [b, c], [c, a]])
        .map(|mut edge| {
            edge.sort_unstable();
            edge
        })
        .collect()
}

#[test]
fn greedy_cell_matches_the_exhaustive_oracle_under_vertex_orders_and_affine_maps() {
    let original = [[120, 60], [120, 30], [90, 30], [120, 40], [102, 48]];
    let boundary = [[0, 3], [0, 4], [1, 2], [1, 3], [2, 4]];
    // A pentagon has five combinatorial triangulations. Exactly two contain
    // the degenerate triangle P-R-T; these are the three remaining tilings.
    // Enumerating them is independent of incremental insertion and recovery.
    let tilings = [
        [[0, 4, 2], [0, 2, 3], [2, 1, 3]],
        [[2, 1, 4], [1, 3, 4], [0, 4, 3]],
        [[2, 1, 3], [2, 3, 4], [0, 4, 3]],
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = TriangulationContext::new(policy);
        for transform in 0..3 {
            let mut order = [0, 1, 2, 3, 4];
            loop {
                let mut inverse = [0; 5];
                for (new, &old) in order.iter().enumerate() {
                    inverse[old] = new;
                }
                let points = order.map(|old| {
                    let [x, y] = original[old];
                    let [x, y] = match transform {
                        0 => [x, y],
                        1 => [x + 2 * y + 7, y - 11],
                        2 => [-x, 3 * y],
                        _ => unreachable!(),
                    };
                    ExactPoint::new(Real::from(x), Real::from(y))
                });
                let constraints = boundary.map(|[a, b]| Constraint::new(inverse[a], inverse[b]));
                let candidates = tilings
                    .map(|triangles| triangles.map(|triangle| triangle.map(|old| inverse[old])));
                let expected = candidates
                    .iter()
                    .min_by_key(|triangles| edges(triangles.as_slice()))
                    .unwrap()
                    .iter()
                    .map(|&triangle| {
                        let mut triangle = triangle;
                        triangle.sort_unstable();
                        triangle
                    })
                    .collect::<BTreeSet<_>>();
                let outcome =
                    constrained_triangulation_convex_hull(&context, &points, &constraints).unwrap();
                assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
                let actual = outcome
                    .value
                    .into_iter()
                    .map(|mut triangle| {
                        triangle.sort_unstable();
                        triangle
                    })
                    .collect::<BTreeSet<_>>();
                assert_eq!(actual, expected, "order={order:?}, transform={transform}");
                if !next_order(&mut order) {
                    break;
                }
            }
        }
    }
}
