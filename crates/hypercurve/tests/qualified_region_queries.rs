use hypercurve::{
    Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2, PredicatePolicy,
    QuadraticBezier2, Real, RegionPointLocation,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn positive_height_qualified_empty_region() -> (Real, CurveRegion2) {
    let height = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        + Real::from(2).powi_i64(-1024).unwrap();
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(
            point(0, 0),
            Point2::new(Real::one(), height.clone()),
            point(2, 0),
        )
        .into(),
        Curve2::try_line(point(2, 0).into(), point(0, 0).into()).unwrap(),
    ])
    .unwrap();
    let constructed = hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[hypercurve::CurveRegionLoopRole::Material],
            &[FillRule::EvenOdd],
        )
        .unwrap()
    });
    assert_eq!(
        constructed.certainty(),
        CurveCertainty::Approximate512Consumed
    );
    let region = constructed.into_unverified();
    assert!(region.is_empty());
    // The source cap has positive area 2*height/3. Prove the independent
    // sign after construction so this check does not warm its decision path.
    assert_eq!(
        height.certified_sign_until(-2048).sign(),
        Some(hypercurve::RealSign::Positive)
    );
    (height, region)
}

fn check_qualified_region_query<T: std::fmt::Debug>(
    result: hypercurve::Provisional<hypercurve::ExactCurveResult<T>>,
    policy: PredicatePolicy,
) -> Option<T> {
    if policy == PredicatePolicy::STRICT {
        assert!(
            matches!(result.as_unverified(), Err(hypercurve::ExactCurveError::Blocked(reason)) if reason.reason() == hypercurve::UncertaintyReason::Predicate),
            "strict query lost inherited topology: {:?}",
            result.as_unverified()
        );
        None
    } else {
        assert_eq!(
            result.certainty(),
            CurveCertainty::Approximate512Consumed,
            "{:?}",
            result.as_unverified()
        );
        Some(result.into_unverified().unwrap())
    }
}

#[test]
fn qualified_empty_membership_queries_retain_topology_receipts() {
    let (height, region) = positive_height_qualified_empty_region();
    let points = [
        Point2::new(Real::one(), (height / Real::from(4)).unwrap()).into(),
        point(3, 1).into(),
    ];
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let one = hypercurve::evaluate_under(policy, || region.classify_point(&points[0]));
        if let Some(value) = check_qualified_region_query(one, policy) {
            assert_eq!(value, RegionPointLocation::Outside);
        }
        let batch = hypercurve::evaluate_under(policy, || region.classify_points(&points));
        if let Some(values) = check_qualified_region_query(batch, policy) {
            assert_eq!(values, vec![RegionPointLocation::Outside; 2]);
        }
    }
}

#[test]
fn qualified_empty_area_queries_retain_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        for result in [
            hypercurve::evaluate_under(policy, || region.signed_area()),
            hypercurve::evaluate_under(policy, || region.filled_area()),
        ] {
            if let Some(area) = check_qualified_region_query(result, policy) {
                assert_eq!(area, Some(Real::zero()));
            }
        }
    }
}

#[test]
fn qualified_empty_role_queries_retain_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        for result in [
            hypercurve::evaluate_under(policy, || region.loop_roles().map(|roles| roles.len())),
            hypercurve::evaluate_under(policy, || {
                region
                    .loop_role_counts()
                    .map(|(material, holes)| material + holes)
            }),
            hypercurve::evaluate_under(policy, || {
                region.boundary_profiles().map(|profiles| profiles.len())
            }),
        ] {
            if let Some(count) = check_qualified_region_query(result, policy) {
                assert_eq!(count, 0);
            }
        }
    }
}

#[test]
fn qualified_empty_native_view_retain_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            region
                .native_contours_fast_path()
                .map(|view| view.map(|view| view.is_empty()))
        });
        if let Some(view) = check_qualified_region_query(result, policy) {
            assert_eq!(view, Some(true));
        }
    }
}

#[test]
fn qualified_empty_material_components_retain_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || region.material_components());
        if let Some(components) = check_qualified_region_query(result, policy) {
            assert!(components.is_empty());
        }
    }
}

#[test]
fn qualified_empty_side_and_fact_queries_retain_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let side = hypercurve::evaluate_under(policy, || {
            region.filled_side_is_left().map(|sides| sides.len())
        });
        if let Some(count) = check_qualified_region_query(side, policy) {
            assert_eq!(count, 0);
        }
        let facts = hypercurve::evaluate_under(policy, || region.structural_facts());
        check_qualified_region_query(facts, policy);
    }
}

#[test]
fn qualified_empty_bounds_keep_topology_receipts() {
    let (_, region) = positive_height_qualified_empty_region();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || region.bounds());
        if let Some(bounds) = check_qualified_region_query(result, policy) {
            assert_eq!(bounds, None);
        }
    }
}

#[test]
fn qualified_empty_similarity_keeps_native_lowering_receipt() {
    let (_, region) = positive_height_qualified_empty_region();
    let transform = hypercurve::Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::from(3),
        Real::from(-2),
    )
    .unwrap();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || region.transform_similarity(&transform));
        if let Some(result) = check_qualified_region_query(result, policy) {
            assert!(result.is_empty());
            assert!(
                matches!(result.regularized_region(), Err(hypercurve::ExactCurveError::Blocked(reason)) if reason.reason() == hypercurve::UncertaintyReason::Predicate)
            );
        }
    }
}

fn exact_square_region() -> CurveRegion2 {
    let points = [point(4, 4), point(6, 4), point(6, 6), point(4, 6)];
    let boundary = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::try_line(points[i].clone().into(), points[(i + 1) % 4].clone().into())
                    .unwrap()
            })
            .collect(),
    )
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).unwrap()
}

#[test]
fn exact_region_queries_remain_certified_across_policy_changes() {
    for region in [CurveRegion2::default(), exact_square_region()] {
        let empty = region.is_empty();
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let result = hypercurve::evaluate_under(policy, || {
                let count = usize::from(!empty);
                let area = if empty { Real::zero() } else { Real::from(4) };
                let inside = if empty {
                    RegionPointLocation::Outside
                } else {
                    RegionPointLocation::Inside
                };
                let points = [point(5, 5).into(), point(9, 9).into()];
                assert_eq!(region.classify_point(&points[0]).unwrap(), inside);
                assert_eq!(
                    region.classify_points(&points).unwrap(),
                    vec![inside, RegionPointLocation::Outside]
                );
                assert_eq!(region.signed_area().unwrap(), Some(area.clone()));
                assert_eq!(region.filled_area().unwrap(), Some(area));
                assert_eq!(region.loop_roles().unwrap().len(), count);
                assert_eq!(region.loop_role_counts().unwrap(), (count, 0));
                assert_eq!(region.boundary_profiles().unwrap().len(), count);
                assert_eq!(region.filled_side_is_left().unwrap(), vec![true; count]);
                let facts = region.structural_facts().unwrap();
                assert_eq!(
                    (facts.material_contour_count, facts.hole_contour_count),
                    (count, 0)
                );
                assert_eq!(
                    region.native_contours_fast_path().unwrap().unwrap().len(),
                    count
                );
                assert_eq!(
                    region.bounds().unwrap(),
                    if empty {
                        None
                    } else {
                        Some(hypercurve::Aabb2::new_unchecked(point(4, 4), point(6, 6)))
                    }
                );
                let components = region.material_components().unwrap();
                assert_eq!(components.len(), count);
                for component in components {
                    assert_eq!(component.filled_area().unwrap(), Some(Real::from(4)));
                }
                let transform = hypercurve::Similarity2::try_from_real_affine(
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::from(3),
                    Real::from(-2),
                )
                .unwrap();
                let transformed = region.transform_similarity(&transform).unwrap();
                assert_eq!(transformed.is_empty(), empty);
                assert_eq!(
                    transformed.classify_point(&point(8, 3).into()).unwrap(),
                    inside
                );
                assert!(transformed.regularized_region().is_ok());
            });
            assert_eq!(result.certainty(), CurveCertainty::Certified);
        }
    }
}

#[test]
fn qualified_nonempty_queries_and_components_retain_topology_receipts() {
    let (height, collapsed) = positive_height_qualified_empty_region();
    let square = exact_square_region();
    let result = hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        collapsed
            .boolean_region(&square, hypercurve::BooleanOp::Union)
            .unwrap()
    });
    assert_eq!(result.certainty(), CurveCertainty::Approximate512Consumed);
    let region = result.into_unverified();
    assert!(!region.is_empty());
    // The intended union also contains the disjoint positive-height cap;
    // its source area is 4 + 2*height/3, strictly greater than the retained 4.
    let cap_point = Point2::new(Real::one(), (height / Real::from(4)).unwrap());
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        for (p, expected) in [
            (point(5, 5), RegionPointLocation::Inside),
            (cap_point.clone(), RegionPointLocation::Outside),
        ] {
            if let Some(location) = check_qualified_region_query(
                hypercurve::evaluate_under(policy, || region.classify_point(&p.into())),
                policy,
            ) {
                assert_eq!(location, expected);
            }
        }
        for result in [
            hypercurve::evaluate_under(policy, || region.signed_area()),
            hypercurve::evaluate_under(policy, || region.filled_area()),
        ] {
            if let Some(area) = check_qualified_region_query(result, policy) {
                assert_eq!(area, Some(Real::from(4)));
            }
        }
        for result in [
            hypercurve::evaluate_under(policy, || region.loop_roles().map(|r| r.len())),
            hypercurve::evaluate_under(policy, || region.boundary_profiles().map(|r| r.len())),
            hypercurve::evaluate_under(policy, || region.filled_side_is_left().map(|r| r.len())),
        ] {
            if let Some(count) = check_qualified_region_query(result, policy) {
                assert_eq!(count, 1);
            }
        }
        if let Some(facts) = check_qualified_region_query(
            hypercurve::evaluate_under(policy, || region.structural_facts()),
            policy,
        ) {
            assert_eq!(
                (facts.material_contour_count, facts.hole_contour_count),
                (1, 0)
            );
        }
        if let Some(count) = check_qualified_region_query(
            hypercurve::evaluate_under(policy, || {
                region.native_contours_fast_path().map(|r| r.unwrap().len())
            }),
            policy,
        ) {
            assert_eq!(count, 1);
        }
        if let Some(bounds) = check_qualified_region_query(
            hypercurve::evaluate_under(policy, || region.bounds()),
            policy,
        ) {
            assert_eq!(
                bounds,
                Some(hypercurve::Aabb2::new_unchecked(point(4, 4), point(6, 6)))
            );
        }
        if let Some(components) = check_qualified_region_query(
            hypercurve::evaluate_under(policy, || region.material_components()),
            policy,
        ) {
            assert_eq!(components.len(), 1);
            for component in components {
                assert!(
                    matches!(component.regularized_region(), Err(hypercurve::ExactCurveError::Blocked(reason)) if reason.reason() == hypercurve::UncertaintyReason::Predicate)
                );
                let area = hypercurve::evaluate_under(policy, || component.filled_area());
                assert_eq!(
                    check_qualified_region_query(area, policy),
                    Some(Some(Real::from(4)))
                );
            }
        }
        // Consuming the qualified union does not alter the exact operand.
        assert_eq!(square.filled_area().unwrap(), Some(Real::from(4)));
    }
}
