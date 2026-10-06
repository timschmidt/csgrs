use hypermesh::{
    ConvexPolygon, MeshCertainty, MeshContext, PairwiseIntersection, Point3, PredicatePolicy, Real,
};
use proptest::prelude::*;

#[derive(Clone, Copy)]
enum Expected {
    Disjoint,
    NonCoplanarPoint([i32; 3]),
    NonCoplanarSegment([[i32; 3]; 2]),
    CoplanarPoint([i32; 3]),
    CoplanarSegment([[i32; 3]; 2]),
    CoplanarOverlap,
}

struct Case {
    id: &'static str,
    left: &'static [[i32; 3]],
    right: &'static [[i32; 3]],
    expected: Expected,
}

const CASES: &[Case] = &[
    Case {
        id: "parallel_support_disjoint",
        left: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        right: &[[0, 0, 1], [2, 0, 1], [0, 2, 1]],
        expected: Expected::Disjoint,
    },
    Case {
        id: "noncoplanar_vertex_contact",
        left: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        right: &[[2, 0, 0], [3, 0, 1], [3, 0, -1]],
        expected: Expected::NonCoplanarPoint([2, 0, 0]),
    },
    Case {
        id: "noncoplanar_transverse_segment",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[1, -1, -1], [1, 3, -1], [1, 1, 1]],
        expected: Expected::NonCoplanarSegment([[1, 0, 0], [1, 2, 0]]),
    },
    Case {
        id: "noncoplanar_disjoint_support_line_intervals",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[1, 5, -1], [1, 9, -1], [1, 7, 1]],
        expected: Expected::Disjoint,
    },
    Case {
        id: "noncoplanar_crossing_crossing_point_contact",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[1, 2, -1], [1, 4, 1], [1, 6, -1]],
        expected: Expected::NonCoplanarPoint([1, 3, 0]),
    },
    Case {
        id: "noncoplanar_contained_support_line_interval",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[1, 0, -1], [1, 2, -1], [1, 2, 1]],
        expected: Expected::NonCoplanarSegment([[1, 1, 0], [1, 2, 0]]),
    },
    Case {
        id: "noncoplanar_z_axis_support_line",
        left: &[[0, -1, 0], [0, 1, 0], [0, 0, 4]],
        right: &[[-1, 0, 1], [1, 0, 1], [0, 0, 3]],
        expected: Expected::NonCoplanarSegment([[0, 0, 1], [0, 0, 3]]),
    },
    Case {
        id: "noncoplanar_convex_quad_containment",
        left: &[[0, 0, 0], [2, 0, 0], [2, 4, 0], [0, 4, 0]],
        right: &[[1, 1, -1], [1, 3, -1], [1, 3, 1], [1, 1, 1]],
        expected: Expected::NonCoplanarSegment([[1, 1, 0], [1, 3, 0]]),
    },
    Case {
        id: "noncoplanar_shared_edge",
        left: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        right: &[[2, 0, 0], [0, 0, 0], [1, 0, 2]],
        expected: Expected::NonCoplanarSegment([[0, 0, 0], [2, 0, 0]]),
    },
    Case {
        id: "coplanar_disjoint",
        left: &[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
        right: &[[3, 0, 0], [4, 0, 0], [3, 1, 0]],
        expected: Expected::Disjoint,
    },
    Case {
        id: "coplanar_vertex_vertex_contact",
        left: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        right: &[[2, 0, 0], [3, -1, 0], [3, 0, 0]],
        expected: Expected::CoplanarPoint([2, 0, 0]),
    },
    Case {
        id: "coplanar_vertex_edge_t_junction",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[2, 0, 0], [1, -1, 0], [3, -1, 0]],
        expected: Expected::CoplanarPoint([2, 0, 0]),
    },
    Case {
        id: "coplanar_full_edge_contact",
        left: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        right: &[[0, 0, 0], [0, -2, 0], [2, 0, 0]],
        expected: Expected::CoplanarSegment([[0, 0, 0], [2, 0, 0]]),
    },
    Case {
        id: "coplanar_partial_edge_contact",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[1, 0, 0], [1, -2, 0], [3, 0, 0]],
        expected: Expected::CoplanarSegment([[1, 0, 0], [3, 0, 0]]),
    },
    Case {
        id: "coplanar_contained_area",
        left: &[[0, 0, 0], [4, 0, 0], [0, 4, 0]],
        right: &[[0, 0, 0], [2, 0, 0], [0, 2, 0]],
        expected: Expected::CoplanarOverlap,
    },
    Case {
        id: "coplanar_identical_area",
        left: &[[-2, -1, 0], [2, -1, 0], [2, 1, 0], [-2, 1, 0]],
        right: &[[-2, -1, 0], [2, -1, 0], [2, 1, 0], [-2, 1, 0]],
        expected: Expected::CoplanarOverlap,
    },
    Case {
        id: "coplanar_crossing_area_without_contained_vertex",
        left: &[[-2, -1, 0], [2, -1, 0], [2, 1, 0], [-2, 1, 0]],
        right: &[[-1, -2, 0], [1, -2, 0], [1, 2, 0], [-1, 2, 0]],
        expected: Expected::CoplanarOverlap,
    },
];

fn point(coordinates: [i32; 3]) -> Point3 {
    let [x, y, z] = coordinates;
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn polygon(context: &MeshContext, vertices: &[[i32; 3]], reverse: bool) -> ConvexPolygon {
    let mut points = vertices.iter().copied().map(point).collect::<Vec<_>>();
    if reverse {
        points.reverse();
    }
    match points.as_slice() {
        [a, b, c] => hypermesh::convex_triangle(context, a, b, c, 0, 0)
            .unwrap()
            .into_value(),
        [a, b, c, d] => hypermesh::convex_quad(context, a, b, c, d, 0, 0)
            .unwrap()
            .into_value(),
        _ => panic!("intersection corpus polygons must be triangles or quads"),
    }
}

fn unordered_segment_matches(actual: [&Point3; 2], expected: [[i32; 3]; 2]) -> bool {
    let expected = expected.map(point);
    (actual[0] == &expected[0] && actual[1] == &expected[1])
        || (actual[0] == &expected[1] && actual[1] == &expected[0])
}

fn assert_expected(case: &Case, actual: &PairwiseIntersection) {
    let matches = match (case.expected, actual) {
        (Expected::Disjoint, PairwiseIntersection::Disjoint)
        | (Expected::CoplanarOverlap, PairwiseIntersection::CoplanarOverlap(_)) => true,
        (Expected::NonCoplanarPoint(expected), PairwiseIntersection::NonCoplanarPoint(actual))
        | (Expected::CoplanarPoint(expected), PairwiseIntersection::CoplanarPoint(actual)) => {
            actual.point == point(expected)
        }
        (
            Expected::NonCoplanarSegment(expected),
            PairwiseIntersection::NonCoplanarSegment(actual),
        )
        | (Expected::CoplanarSegment(expected), PairwiseIntersection::CoplanarSegment(actual)) => {
            unordered_segment_matches([&actual.v0, &actual.v1], expected)
        }
        _ => false,
    };
    assert!(matches, "intersection corpus case {}: {actual:?}", case.id);
}

#[test]
fn exact_pairwise_intersection_corpus_covers_policy_order_and_orientation() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for case in CASES {
            for reverse_left in [false, true] {
                for reverse_right in [false, true] {
                    let left = polygon(&context, case.left, reverse_left);
                    let right = polygon(&context, case.right, reverse_right);
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&right, &left)
                        } else {
                            (&left, &right)
                        };
                        let outcome = hypermesh::intersect_polygons(&context, first, second, 41)
                            .unwrap_or_else(|error| {
                                panic!(
                                    "intersection corpus case {} failed under {policy:?}: {error}",
                                    case.id
                                )
                            });
                        assert_eq!(outcome.certainty, MeshCertainty::Certified, "{}", case.id);
                        assert_expected(case, &outcome.value);
                    }
                }
            }
        }
    }
}

fn rectangle(context: &MeshContext, x0: i32, x1: i32, y0: i32, y1: i32) -> ConvexPolygon {
    let vertices = [[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]];
    polygon(context, &vertices, false)
}

fn vertical_rectangle(
    context: &MeshContext,
    x: i32,
    y0: i32,
    y1: i32,
    z0: i32,
    z1: i32,
) -> ConvexPolygon {
    let vertices = [[x, y0, z0], [x, y1, z0], [x, y1, z1], [x, y0, z1]];
    polygon(context, &vertices, false)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn coplanar_rectangle_classifier_matches_exact_open_interval_oracle(
        ax0 in -8i32..8,
        ax1 in -8i32..8,
        ay0 in -8i32..8,
        ay1 in -8i32..8,
        bx0 in -8i32..8,
        bx1 in -8i32..8,
        by0 in -8i32..8,
        by1 in -8i32..8,
    ) {
        prop_assume!(ax0 != ax1 && ay0 != ay1 && bx0 != bx1 && by0 != by1);
        let [ax0, ax1] = [ax0.min(ax1), ax0.max(ax1)];
        let [ay0, ay1] = [ay0.min(ay1), ay0.max(ay1)];
        let [bx0, bx1] = [bx0.min(bx1), bx0.max(bx1)];
        let [by0, by1] = [by0.min(by1), by0.max(by1)];
        let expected_area = ax0.max(bx0) < ax1.min(bx1) && ay0.max(by0) < ay1.min(by1);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let context = MeshContext::new(policy);
            let left = rectangle(&context, ax0, ax1, ay0, ay1);
            let right = rectangle(&context, bx0, bx1, by0, by1);
            for invert_left in [false, true] {
                for invert_right in [false, true] {
                    let left = if invert_left { left.inverted() } else { left.clone() };
                    let right = if invert_right { right.inverted() } else { right.clone() };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&right, &left)
                        } else {
                            (&left, &right)
                        };
                        let outcome =
                            hypermesh::intersect_polygons(&context, first, second, 1).unwrap();
                        prop_assert_eq!(outcome.certainty, MeshCertainty::Certified);
                        prop_assert_eq!(
                            matches!(outcome.value, PairwiseIntersection::CoplanarOverlap(_)),
                            expected_area,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn noncoplanar_rectangle_slices_match_exact_closed_interval_oracle(
        x in -8i32..8,
        left_x_margin in 1i32..8,
        right_x_margin in 1i32..8,
        ay0 in -8i32..8,
        ay1 in -8i32..8,
        by0 in -8i32..8,
        by1 in -8i32..8,
        negative_z in 1i32..8,
        positive_z in 1i32..8,
    ) {
        prop_assume!(ay0 != ay1 && by0 != by1);
        let [ay0, ay1] = [ay0.min(ay1), ay0.max(ay1)];
        let [by0, by1] = [by0.min(by1), by0.max(by1)];
        let overlap_minimum = ay0.max(by0);
        let overlap_maximum = ay1.min(by1);

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let context = MeshContext::new(policy);
            let left = rectangle(
                &context,
                x - left_x_margin,
                x + right_x_margin,
                ay0,
                ay1,
            );
            let right = vertical_rectangle(
                &context,
                x,
                by0,
                by1,
                -negative_z,
                positive_z,
            );
            for invert_left in [false, true] {
                for invert_right in [false, true] {
                    let left = if invert_left { left.inverted() } else { left.clone() };
                    let right = if invert_right { right.inverted() } else { right.clone() };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&right, &left)
                        } else {
                            (&left, &right)
                        };
                        let outcome =
                            hypermesh::intersect_polygons(&context, first, second, 1).unwrap();
                        prop_assert_eq!(outcome.certainty, MeshCertainty::Certified);
                        match overlap_minimum.cmp(&overlap_maximum) {
                            std::cmp::Ordering::Greater => {
                                prop_assert!(matches!(
                                    outcome.value,
                                    PairwiseIntersection::Disjoint
                                ));
                            }
                            std::cmp::Ordering::Equal => {
                                let PairwiseIntersection::NonCoplanarPoint(actual) = outcome.value
                                else {
                                    prop_assert!(false, "expected one closed-interval contact");
                                    unreachable!();
                                };
                                prop_assert_eq!(actual.point, point([x, overlap_minimum, 0]));
                            }
                            std::cmp::Ordering::Less => {
                                let PairwiseIntersection::NonCoplanarSegment(actual) = outcome.value
                                else {
                                    prop_assert!(false, "expected one closed-interval segment");
                                    unreachable!();
                                };
                                prop_assert!(unordered_segment_matches(
                                    [&actual.v0, &actual.v1],
                                    [[x, overlap_minimum, 0], [x, overlap_maximum, 0]],
                                ));
                            }
                        }
                    }
                }
            }
        }
    }
}
