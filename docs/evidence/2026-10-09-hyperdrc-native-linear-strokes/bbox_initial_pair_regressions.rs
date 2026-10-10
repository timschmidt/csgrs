use hypercurve::{Aabb2, Classification, Point2, PredicatePolicy, Real, evaluate_under};

fn equal(actual: &Real, expected: &Real) {
    assert_eq!((actual - expected).exact_rational_normal_form(), Real::zero().exact_rational());
}

#[test]
fn bbox_point_permutations_match_an_independent_integer_extrema_oracle() {
    let permutations = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for scale in ["1", "-3", "1e400", "1e-400"] {
            for warm in [false, true] {
                for order in permutations {
                    for x in [[-2, 0, 3], [3, 3, -2], [0, 0, 0]] {
                        for y in [[5, -7, 1], [5, 5, -7], [0, 0, 0]] {
                            let scale: Real = scale.parse().unwrap();
                            let positive = scale.exact_rational_ref().unwrap() > Real::zero().exact_rational_ref().unwrap();
                            let origin = Real::pi() + Real::from(53).sqrt().unwrap();
                            let map = |value: i32| (&origin + Real::from(value)) * &scale;
                            let points = order.map(|i| Point2::new(map(x[i]), map(y[i])));
                            if warm {
                                for point in &points {
                                    point.x().certified_rational_interval(-128).unwrap();
                                    point.y().certified_rational_interval(-128).unwrap();
                                }
                            }
                            let outcome = evaluate_under(policy, || Aabb2::from_points(&points)).certified().unwrap();
                            let Classification::Decided(bounds) = outcome else { panic!("integer-separated translated coordinates must decide") };
                            for (lower, upper, values) in [(bounds.min_x(), bounds.max_x(), x), (bounds.min_y(), bounds.max_y(), y)] {
                                let lo = *values.iter().min().unwrap();
                                let hi = *values.iter().max().unwrap();
                                equal(lower, &map(if positive { lo } else { hi }));
                                equal(upper, &map(if positive { hi } else { lo }));
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn bbox_empty_singleton_duplicate_and_pair_inputs_preserve_exact_extrema() {
    assert!(matches!(Aabb2::from_points(std::iter::empty()), Classification::Uncertain(_)));
    let huge: Real = "1e400".parse().unwrap();
    let tiny: Real = "1e-400".parse().unwrap();
    let first = Point2::new(huge.clone(), -huge.clone());
    let second = Point2::new(&huge + &tiny, -&huge - &tiny);
    for points in [vec![first.clone()], vec![first.clone(), first.clone()], vec![first.clone(), second.clone()], vec![second.clone(), first.clone()]] {
        let Classification::Decided(bounds) = Aabb2::from_points(&points) else { panic!("rational inputs must decide") };
        equal(bounds.min_x(), &huge);
        equal(bounds.max_y(), &(-&huge));
        let distinct = points.iter().any(|point| point.x() == second.x());
        equal(bounds.max_x(), if distinct { second.x() } else { first.x() });
        equal(bounds.min_y(), if distinct { second.y() } else { first.y() });
    }
}
