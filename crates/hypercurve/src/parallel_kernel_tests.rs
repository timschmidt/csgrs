//! Exact analytic-parallel incidence and intersection kernel regressions.

use crate::{
    BezierParameter2, Classification, CubicBezier2, CurveContext, CurveIntersectionCandidates2,
    CurveOverlapOrientation2, CurveParameterRange2, CurvePoint2, Point2, QuadraticBezier2,
    RationalBezier2, RationalBezierIntersectionOverlap2, RationalQuadraticBezier2, Real, RealSign,
};

use crate::bezier_offset::{
    BezierParallelIncidence2, BezierParallelIntersectionContact2, BezierParallelIntersectionSet2,
    BezierParallelPairIntersectionContact2, BezierParallelPairIntersectionSet2,
};

#[path = "../tests/support/scalars.rs"]
mod support;

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn decided_parallel_set(
    result: Classification<BezierParallelIntersectionSet2>,
) -> BezierParallelIntersectionSet2 {
    let Classification::Decided(result) = result else {
        panic!("parallel intersections remained uncertain");
    };
    result
}

fn decided_parallel_pair_set(
    result: Classification<BezierParallelPairIntersectionSet2>,
) -> BezierParallelPairIntersectionSet2 {
    match result {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => {
            panic!("parallel/parallel intersections remained uncertain: {reason:?}")
        }
    }
}

fn pair_has_exact_parameters(
    contacts: &[BezierParallelPairIntersectionContact2],
    first: Real,
    second: Real,
) -> bool {
    contacts.iter().any(|contact| {
        contact.first_parameter().scalar() == Some(&first)
            && contact.second_parameter().scalar() == Some(&second)
    })
}

fn only_parallel_contacts(
    intersections: &BezierParallelIntersectionSet2,
) -> &[BezierParallelIntersectionContact2] {
    assert!(intersections.is_complete());
    assert!(intersections.overlaps().is_empty());
    intersections.contacts()
}

fn only_parallel_overlap(
    intersections: &BezierParallelIntersectionSet2,
) -> &RationalBezierIntersectionOverlap2 {
    assert!(intersections.is_complete());
    assert!(intersections.contacts().is_empty());
    let [overlap] = intersections.overlaps() else {
        panic!(
            "expected one parallel overlap, found {}",
            intersections.overlaps().len()
        );
    };
    overlap
}

fn rootless_homogeneous_factor_parabola() -> RationalBezier2 {
    // Homogeneous power basis
    //   (X, Y, W) = (t(t + 2), t^2(t + 2), t + 2)
    // represents the regular non-PH parabola (t, t^2). The common factor has
    // its only root at t=-2, outside the authored parameter interval.
    RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(2, 7), r(0)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(2), q(7, 3), q(8, 3), r(3)],
    )
    .unwrap()
}

fn rootless_homogeneous_factor_vertical() -> RationalBezier2 {
    // (0, 2u(u + 2), u + 2) represents the same finite segment as (0, 2u).
    RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(r(0), q(4, 5)), p(0, 2)],
        vec![r(2), q(5, 2), r(3)],
    )
    .unwrap()
}

fn rootful_homogeneous_factor_vertical() -> RationalBezier2 {
    // (0, 2u(u - 1/3), u - 1/3) has a removable projective base point at
    // u=1/3. Hypercurve deliberately retains that authored domain boundary.
    RationalBezier2::try_new(
        vec![p(0, 0), p(0, -2), p(0, 2)],
        vec![q(-1, 3), q(1, 6), q(2, 3)],
    )
    .unwrap()
}

fn rationally_reparameterized_parabola_parallel() -> RationalBezier2 {
    // This degree-six rational curve is the exact unit left parallel of
    // P(t)=(3t/8, 9t^2/64). Its parameter u rationalizes
    // sqrt(16+9t^2), and the parameter correspondence is
    //
    //   t = (4u-u^2)/(6-3u).
    //
    // The source is not PH in its authored parameter, so this overlap cannot
    // use same-parameter rational materialization.
    RationalBezier2::try_new(
        vec![
            Point2::new(r(0), r(1)),
            Point2::new(q(-1, 18), r(1)),
            Point2::new(q(-15, 134), q(133, 134)),
            Point2::new(q(-43, 264), q(43, 44)),
            Point2::new(q(-117, 580), q(1111, 1160)),
            Point2::new(q(-25, 112), q(211, 224)),
            Point2::new(q(-9, 40), q(301, 320)),
        ],
        vec![
            r(1),
            q(3, 4),
            q(67, 120),
            q(33, 80),
            q(29, 96),
            q(7, 32),
            q(5, 32),
        ],
    )
    .unwrap()
}

fn nonlinearly_reparameterized_parabola() -> RationalBezier2 {
    // P(v)=(3v/8,9v^2/64) authored through v=(t^2+t)/2. The derivative of
    // the reparameterization is strictly positive on [0,1], while the unit
    // parallel remains non-PH. Against rationally_reparameterized_parabola_parallel
    // the common parameter component is
    //
    //   (6-3u)(t^2+t) - 2(4u-u^2) = 0,
    //
    // which is irreducible and nonlinear in both parameters.
    RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(3, 64), r(0)),
            Point2::new(q(1, 8), q(3, 512)),
            Point2::new(q(15, 64), q(9, 256)),
            Point2::new(q(3, 8), q(9, 64)),
        ],
        vec![r(1); 5],
    )
    .unwrap()
}

#[test]
fn exact_parallel_point_incidence_rejects_the_opposite_normal_branch() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let parallel = source.parallel_left(r(1)).unwrap();
    let right_parallel = source.parallel_left(r(-1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .point_incidence(&p(1, 1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            parallel
                .point_incidence(&p(1, -1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
        assert_eq!(
            parallel
                .contains_point(&p(1, 1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(true)
        );
        assert_eq!(
            parallel
                .contains_point(&p(1, -1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(false)
        );
        assert_eq!(
            right_parallel
                .point_incidence(&p(1, -1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            right_parallel
                .point_incidence(&p(1, 1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn parallel_point_incidence_uses_approximate_512_only_as_a_terminal_decision() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0));
    let parallel = source.parallel_left(undecidable_zero).unwrap();

    assert_eq!(
        parallel.point_incidence(
            &p(1, 0),
            &CurveParameterRange2::unit(),
            &CurveContext::STRICT
        ),
        Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign
        ))
    );
    assert_eq!(
        parallel.point_incidence(
            &p(1, 0),
            &CurveParameterRange2::unit(),
            &CurveContext::APPROXIMATE_512
        ),
        Ok(Classification::Decided(
            BezierParallelIncidence2::Parameters(vec![BezierParameter2::Exact(q(1, 2))])
        ))
    );
}

#[test]
fn finite_parallel_point_incidence_owns_poles_roots_and_normal_sheets() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![Real::one(), -Real::one()]).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let endpoints = if reversed { [3, 2] } else { [2, 3] };
            let Classification::Decided(range) = CurveParameterRange2::try_new(
                r(endpoints[0]).into(),
                r(endpoints[1]).into(),
                &policy,
            )
            .unwrap() else {
                panic!("exact finite range")
            };
            for distance in [Real::zero(), q(1, 10)] {
                let parallel = source.parallel_left(distance.clone()).unwrap();
                let shift = (distance.clone() / r(2).sqrt().unwrap()).unwrap();
                let point = Point2::new(q(2, 3) + &shift, q(2, 3) - &shift);
                let Classification::Decided(BezierParallelIncidence2::Parameters(parameters)) =
                    parallel.point_incidence(&point, &range, &policy).unwrap()
                else {
                    panic!("the exterior endpoint remains incident despite a remote pole")
                };
                assert_eq!(parameters.len(), 1);
                assert_eq!(
                    parameters[0]
                        .cmp_by_refinement(&BezierParameter2::Exact(r(2)), &policy)
                        .unwrap(),
                    Classification::Decided(std::cmp::Ordering::Equal)
                );
                if distance != Real::zero() {
                    let opposite = Point2::new(q(2, 3) - &shift, q(2, 3) + &shift);
                    assert_eq!(
                        parallel.contains_point(&opposite, &range, &policy).unwrap(),
                        Classification::Decided(false)
                    );
                }
            }
            let stationary = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
                .parallel_left(Real::one())
                .unwrap();
            assert_eq!(
                stationary
                    .contains_point(&Point2::new(q(25, 4), Real::one()), &range, &policy)
                    .unwrap(),
                Classification::Decided(true)
            );
            assert_eq!(
                source
                    .parallel_left(Real::zero())
                    .unwrap()
                    .contains_point(&p(0, 0), &range, &policy)
                    .unwrap(),
                Classification::Decided(false)
            );
        }
    }
}

#[test]
fn exact_parallel_point_incidence_retains_algebraic_parameters() {
    // `x(t)=t+t^2` reaches x=1 at the nonrepresented root
    // `(-1+sqrt(5))/2`; its tangent is regular over the complete domain.
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0));
    let parallel = source.parallel_left(r(1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(BezierParallelIncidence2::Parameters(parameters)) = parallel
            .point_incidence(&p(1, 1), &CurveParameterRange2::unit(), &policy)
            .unwrap()
        else {
            panic!("algebraic parallel incidence was not decided");
        };
        let [BezierParameter2::Algebraic(parameter)] = parameters.as_slice() else {
            panic!("parallel incidence did not retain its algebraic parameter");
        };
        assert_eq!(parameter.polynomial().degree(), 2);

        assert_eq!(
            parallel
                .point_incidence(&p(1, -1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn rational_parallel_point_incidence_preserves_projective_parameterization() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(2, 0)], vec![r(1), r(2), r(3)]).unwrap();
    let parallel = source.parallel_left(r(2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .point_incidence(
                    &Point2::new(q(5, 4), r(2)),
                    &CurveParameterRange2::unit(),
                    &policy
                )
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(q(1, 2))
            ]))
        );
        assert_eq!(
            parallel
                .point_incidence(
                    &Point2::new(q(5, 4), r(-2)),
                    &CurveParameterRange2::unit(),
                    &policy
                )
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn collapsed_circular_parallel_reports_entire_curve_point_incidence() {
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    let parallel = source.parallel_left(r(1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .point_incidence(&p(0, 0), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
        assert_eq!(
            parallel
                .point_incidence(&p(1, 0), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(Vec::new()))
        );
    }
}

#[test]
fn parallel_point_incidence_rejects_projective_poles_and_source_singularities() {
    let projective =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)])
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
    let singular = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(1))
        .unwrap();
    let zero_distance_singular = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let zero_distance_constant = QuadraticBezier2::new(p(3, 4), p(3, 4), p(3, 4))
        .parallel_left(r(0))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            projective
                .point_incidence(&p(0, 0), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
        assert_eq!(
            singular
                .point_incidence(&p(0, 1), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
        assert_eq!(
            zero_distance_singular
                .point_incidence(&p(0, 0), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::Parameters(vec![
                BezierParameter2::Exact(r(0))
            ]))
        );
        assert_eq!(
            zero_distance_constant
                .point_incidence(&p(3, 4), &CurveParameterRange2::unit(), &policy)
                .unwrap(),
            Classification::Decided(BezierParallelIncidence2::EntireCurve)
        );
    }
}

#[test]
fn parallel_pair_replays_a_general_non_ph_contact_under_both_policies() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1))
        .parallel_left(r(1))
        .unwrap();
    let second = QuadraticBezier2::new(p(1, 1), p(1, 2), p(2, 3))
        .parallel_left(r(1))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let candidates = first
            .parallel_intersection_candidates(&second, &policy)
            .unwrap();
        assert!(matches!(
            candidates,
            Classification::Decided(CurveIntersectionCandidates2::Candidates { .. })
        ));
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(pair_has_exact_parameters(
            intersections.contacts(),
            r(0),
            r(0)
        ));
        let contact = intersections
            .contacts()
            .iter()
            .find(|contact| {
                contact.first_parameter().scalar() == Some(&r(0))
                    && contact.second_parameter().scalar() == Some(&r(0))
            })
            .unwrap();
        assert!(contact.is_certified_transverse());
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Positive));
        assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Zero));
    }
}

#[test]
fn parallel_pair_rejects_the_opposite_normal_square_branch() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1))
        .parallel_left(r(1))
        .unwrap();
    // At (0,0), the opposite right-normal branch of `second` meets the
    // selected first parallel at (0,1); its selected left branch does not.
    let second = QuadraticBezier2::new(p(-1, 1), p(-1, 2), p(0, 3))
        .parallel_left(r(1))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(!pair_has_exact_parameters(
            intersections.contacts(),
            r(0),
            r(0)
        ));
    }
}

#[test]
fn parallel_pair_structural_overlap_preserves_relative_orientation() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1))
        .parallel_left(r(1))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let same = decided_parallel_pair_set(
            first
                .parallel_intersections(&first.clone(), &policy)
                .unwrap(),
        );
        let [same_overlap] = same.overlaps() else {
            panic!("identical carriers did not retain one overlap");
        };
        assert_eq!(same_overlap.orientation(), CurveOverlapOrientation2::Same);
        assert_eq!(
            same_overlap.second_range().scalar_endpoints(),
            Some((&r(0), &r(1)))
        );

        let reversed = decided_parallel_pair_set(
            first
                .parallel_intersections(&first.reversed(), &policy)
                .unwrap(),
        );
        let [reversed_overlap] = reversed.overlaps() else {
            panic!("reversed carrier did not retain one overlap");
        };
        assert_eq!(
            reversed_overlap.orientation(),
            CurveOverlapOrientation2::Reversed
        );
        assert_eq!(
            reversed_overlap.second_range().scalar_endpoints(),
            Some((&r(1), &r(0)))
        );
    }
}

#[test]
fn parallel_pair_certifies_partial_source_overlap_and_reparameterization() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1));
    let subcurve = source
        .subcurve_between_exact_with_policy(&q(1, 4), &q(3, 4), &CurveContext::STRICT)
        .unwrap();
    let first = source.parallel_left(r(1)).unwrap();
    let same = subcurve.parallel_left(r(1)).unwrap();
    let reversed_source = QuadraticBezier2::new(
        subcurve.end().clone(),
        subcurve.control().clone(),
        subcurve.start().clone(),
    );
    let reversed = reversed_source.parallel_left(r(-1)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (second, orientation, second_start, second_end) in [
            (&same, CurveOverlapOrientation2::Same, r(0), r(1)),
            (&reversed, CurveOverlapOrientation2::Reversed, r(1), r(0)),
        ] {
            assert_eq!(
                first
                    .parallel_intersection_candidates(second, &policy)
                    .unwrap(),
                Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
            );
            let intersections =
                decided_parallel_pair_set(first.parallel_intersections(second, &policy).unwrap());
            assert!(intersections.is_complete(), "{intersections:?}");
            assert!(intersections.contacts().is_empty());
            let [overlap] = intersections.overlaps() else {
                panic!("partial parallel overlap was not retained exactly");
            };
            assert_eq!(overlap.orientation(), orientation);
            assert_eq!(
                overlap.first_range().scalar_endpoints(),
                Some((&q(1, 4), &q(3, 4)))
            );
            assert_eq!(
                overlap.second_range().scalar_endpoints(),
                Some((&second_start, &second_end))
            );
            assert!(overlap.includes_start());
            assert!(overlap.includes_end());
        }
    }
}

#[test]
fn parallel_pair_overlap_retains_off_correspondence_contacts() {
    let source = CubicBezier2::new(p(0, 0), p(1, 4), p(3, -4), p(4, 0));
    let first = source.parallel_left(q(1, 2)).unwrap();
    let second = first.clone();
    assert!(matches!(
        first
            .exact_pythagorean_hodograph_offset(&CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(None)
    ));

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert_eq!(intersections.overlaps().len(), 1, "{intersections:?}");
        assert_eq!(intersections.contacts().len(), 2, "{intersections:?}");
        assert!(
            intersections
                .contacts()
                .iter()
                .all(BezierParallelPairIntersectionContact2::is_certified_transverse)
        );
    }
}

#[test]
fn parallel_pair_retains_an_isolated_boundary_of_a_source_component() {
    let first = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1))
        .parallel_left(q(1, 2))
        .unwrap();
    let second = QuadraticBezier2::new(p(1, 1), Point2::new(q(3, 2), r(2)), p(2, 4))
        .parallel_left(q(1, 2))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.overlaps().is_empty());
        assert!(pair_has_exact_parameters(
            intersections.contacts(),
            r(1),
            r(0),
        ));
    }
}

#[test]
fn parallel_pair_removes_a_false_same_source_component() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1));
    let first = source.parallel_left(q(1, 2)).unwrap();
    let unequal = source.parallel_left(r(1)).unwrap();
    let opposite_branch = source.parallel_left(q(-1, 2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for second in [&unequal, &opposite_branch] {
            assert_eq!(
                first
                    .parallel_intersection_candidates(second, &policy)
                    .unwrap(),
                Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
            );
            let intersections =
                decided_parallel_pair_set(first.parallel_intersections(second, &policy).unwrap());
            assert!(intersections.is_complete(), "{intersections:?}");
            assert!(intersections.is_empty(), "{intersections:?}");
        }
    }
}

#[test]
fn parallel_pair_component_saturation_retains_residual_isolated_contact() {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -2), p(3, 0));
    let first = source.parallel_left(r(1)).unwrap();
    let second = source.parallel_left(r(2)).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            first
                .parallel_intersection_candidates(&second, &policy)
                .unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
        );
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.overlaps().is_empty());
        assert_eq!(intersections.contacts().len(), 1, "{intersections:?}");
        assert!(
            intersections.contacts()[0]
                .first_parameter()
                .scalar()
                .is_none()
        );
        assert!(
            intersections.contacts()[0]
                .second_parameter()
                .scalar()
                .is_none()
        );
    }
}

#[test]
fn parallel_pair_replays_a_non_source_speed_component_residual() {
    // The first source derivative is
    // `(1 + 2t) * (1, t)`. Its root `t = -1/2` lies outside the authored
    // interval, so the source is regular and non-PH on `[0, 1]`, but both
    // squared parallel-pair equations still contain the unrelated factor
    // `(1 + 2t)^2`. Saturation must retain that factor's norm intersection
    // separately and replay the residual endpoint contact at `(0, 0)`.
    let first = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), r(0)),
        Point2::new(r(1), q(1, 6)),
        Point2::new(r(2), q(7, 6)),
    )
    .parallel_left(r(1))
    .unwrap();
    let second = QuadraticBezier2::new(
        p(0, 0),
        Point2::new(q(1, 2), r(0)),
        Point2::new(r(1), q(1, 2)),
    )
    .parallel_left(r(1))
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            first.exact_pythagorean_hodograph_offset(&policy).unwrap(),
            Classification::Decided(None)
        ));
        assert!(matches!(
            second.exact_pythagorean_hodograph_offset(&policy).unwrap(),
            Classification::Decided(None)
        ));
        let intersections =
            decided_parallel_pair_set(first.parallel_intersections(&second, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(pair_has_exact_parameters(
            intersections.contacts(),
            r(0),
            r(0),
        ));
    }
}

#[test]
fn parallel_pair_rational_delegate_preserves_operand_parameter_order() {
    let rational_first = QuadraticBezier2::new(p(0, 1), p(0, 2), p(1, 3))
        .parallel_left(r(0))
        .unwrap();
    let general_second = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1))
        .parallel_left(r(1))
        .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let forward = decided_parallel_pair_set(
            rational_first
                .parallel_intersections(&general_second, &policy)
                .unwrap(),
        );
        assert!(forward.is_complete(), "{forward:?}");
        assert!(pair_has_exact_parameters(forward.contacts(), r(0), r(0)));
        let forward_contact = forward
            .contacts()
            .iter()
            .find(|contact| {
                contact.first_parameter().scalar() == Some(&r(0))
                    && contact.second_parameter().scalar() == Some(&r(0))
            })
            .unwrap();
        assert_eq!(
            forward_contact.tangent_cross_sign(),
            Some(RealSign::Negative)
        );
        assert_eq!(forward_contact.tangent_dot_sign(), Some(RealSign::Zero));

        let reverse = decided_parallel_pair_set(
            general_second
                .parallel_intersections(&rational_first, &policy)
                .unwrap(),
        );
        assert!(reverse.is_complete(), "{reverse:?}");
        assert!(pair_has_exact_parameters(reverse.contacts(), r(0), r(0)));
        let reverse_contact = reverse
            .contacts()
            .iter()
            .find(|contact| {
                contact.first_parameter().scalar() == Some(&r(0))
                    && contact.second_parameter().scalar() == Some(&r(0))
            })
            .unwrap();
        assert_eq!(
            reverse_contact.tangent_cross_sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(reverse_contact.tangent_dot_sign(), Some(RealSign::Zero));
    }
}

#[test]
fn parallel_rational_intersection_candidates_retain_both_finite_parameters() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&vertical, &policy)
                .unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::Candidates {
                first_parameters: vec![BezierParameter2::Exact(q(1, 2))],
                second_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            })
        );
    }
}

#[test]
fn parallel_rational_intersection_candidates_retain_algebraic_projection() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(CurveIntersectionCandidates2::Candidates {
            first_parameters: parallel_parameters,
            second_parameters: other_parameters,
        }) = parallel
            .intersection_candidates(&vertical, &policy)
            .unwrap()
        else {
            panic!("parallel/rational algebraic projections were not decided");
        };
        let [BezierParameter2::Algebraic(parameter)] = parallel_parameters.as_slice() else {
            panic!("parallel projection did not retain its algebraic parameter");
        };
        assert!(parameter.polynomial().degree() >= 2);
        assert_eq!(other_parameters, vec![BezierParameter2::Exact(q(1, 2))]);
    }
}

#[test]
fn parallel_rational_intersection_candidates_report_disjoint_and_shared_components() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let disjoint = RationalBezier2::try_new(vec![p(10, 0), p(10, 2)], vec![r(1), r(1)]).unwrap();
    let coincident = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&disjoint, &policy)
                .unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::NoIntersection)
        );
        assert_eq!(
            parallel
                .intersection_candidates(&coincident, &policy)
                .unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
        );
    }
}

#[test]
fn parallel_rational_intersections_retain_a_boundary_parameter_fiber() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let constant = RationalBezier2::try_new(vec![p(0, 1); 5], vec![r(1); 5]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let candidates = parallel
            .intersection_candidates(&constant, &policy)
            .unwrap();
        assert!(
            matches!(
                candidates,
                Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
            ),
            "{candidates:?}"
        );
        let intersections =
            decided_parallel_set(parallel.intersections(&constant, &policy).unwrap());
        assert!(intersections.is_complete());
        assert!(intersections.contacts().is_empty());
        assert!(intersections.overlaps().is_empty());
        assert!(!intersections.is_empty());
        let [component] = intersections.parameter_components() else {
            panic!("the constant target must retain one complete parameter fiber");
        };
        assert_eq!(
            component.parallel_parameter(),
            Some(&BezierParameter2::Exact(r(0)))
        );
        assert_eq!(component.other_parameter(), None);
        assert_eq!(component.point(), &CurvePoint2::from(p(0, 1)));
        assert!(!component.is_entire_parameter_square());
    }
}

#[test]
fn collapsed_parallel_retain_a_fixed_other_parameter_fiber() {
    let source =
        RationalQuadraticBezier2::try_new(p(1, 0), p(1, 1), p(0, 1), r(1), r(1), r(2)).unwrap();
    let parallel = source.parallel_left(r(1)).unwrap();
    let crossing = RationalBezier2::try_new(vec![p(-1, 0), p(1, 0)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&crossing, &policy).unwrap());
        assert!(intersections.is_complete());
        assert!(intersections.contacts().is_empty());
        assert!(intersections.overlaps().is_empty());
        let [component] = intersections.parameter_components() else {
            panic!("the collapsed parallel must retain one complete parameter fiber");
        };
        assert_eq!(component.parallel_parameter(), None);
        assert_eq!(
            component.other_parameter(),
            Some(&BezierParameter2::Exact(q(1, 2)))
        );
        assert_eq!(component.point(), &CurvePoint2::from(p(0, 0)));
        assert!(!component.is_entire_parameter_square());
    }
}

#[test]
fn coincident_constant_curves_retain_the_entire_parameter_square() {
    let parallel = QuadraticBezier2::new(p(3, 4), p(3, 4), p(3, 4))
        .parallel_left(r(0))
        .unwrap();
    let constant = RationalBezier2::try_new(vec![p(3, 4); 3], vec![r(1); 3]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&constant, &policy).unwrap());
        assert!(intersections.is_complete());
        assert!(intersections.contacts().is_empty());
        assert!(intersections.overlaps().is_empty());
        let [component] = intersections.parameter_components() else {
            panic!("coincident constant curves must retain the full parameter square");
        };
        assert_eq!(component.parallel_parameter(), None);
        assert_eq!(component.other_parameter(), None);
        assert_eq!(component.point(), &CurvePoint2::from(p(3, 4)));
        assert!(component.is_entire_parameter_square());
    }
}

#[test]
fn parallel_intersection_parameter_components_reuse_the_supplement_pointer() {
    assert_eq!(
        std::mem::size_of::<BezierParallelIntersectionSet2>(),
        2 * std::mem::size_of::<std::sync::Arc<[u8]>>()
            + std::mem::size_of::<Option<std::sync::Arc<()>>>()
    );
}

#[test]
fn parallel_rational_intersections_saturate_rootless_homogeneous_axis_content() {
    let factored_parallel = rootless_homogeneous_factor_parabola()
        .parallel_left(r(1))
        .unwrap();
    let ordinary_parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1))
        .parallel_left(r(1))
        .unwrap();
    let ordinary_vertical =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 2)], vec![r(1), r(1)]).unwrap();
    let factored_vertical = rootless_homogeneous_factor_vertical();

    for (parallel, vertical) in [
        (&factored_parallel, &ordinary_vertical),
        (&ordinary_parallel, &factored_vertical),
    ] {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(CurveIntersectionCandidates2::Candidates {
                first_parameters: parallel_parameters,
                second_parameters: other_parameters,
            }) = parallel.intersection_candidates(vertical, &policy).unwrap()
            else {
                panic!("rootless homogeneous axis content was not saturated");
            };
            assert_eq!(parallel_parameters.len(), 2);
            assert_eq!(other_parameters.len(), 2);
            assert!(parallel_parameters.contains(&BezierParameter2::Exact(r(0))));
            assert!(other_parameters.contains(&BezierParameter2::Exact(q(1, 2))));
            assert!(other_parameters.contains(&BezierParameter2::Exact(q(5, 8))));

            let intersections =
                decided_parallel_set(parallel.intersections(vertical, &policy).unwrap());
            let contacts = only_parallel_contacts(&intersections);
            assert_eq!(contacts.len(), 2);
            assert!(
                contacts
                    .iter()
                    .any(|contact| { contact.point() == &CurvePoint2::from(p(0, 1)) })
            );
            assert!(contacts.iter().any(|contact| {
                contact.point() == &CurvePoint2::from(Point2::new(r(0), q(5, 4)))
            }));
        }
    }
}

#[test]
fn parallel_rational_axis_saturation_retains_in_domain_projective_base_points() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1))
        .parallel_left(r(1))
        .unwrap();
    let rootful = rootful_homogeneous_factor_vertical();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.intersection_candidates(&rootful, &policy).unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn parallel_rational_candidates_use_approximate_512_only_as_a_terminal_decision() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(undecidable_zero)
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    assert_eq!(
        parallel.intersection_candidates(&vertical, &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign
        ))
    );
    assert_eq!(
        parallel.intersection_candidates(&vertical, &CurveContext::APPROXIMATE_512),
        Ok(Classification::Decided(
            CurveIntersectionCandidates2::Candidates {
                first_parameters: vec![BezierParameter2::Exact(q(1, 2))],
                second_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            }
        ))
    );
}

#[test]
fn parallel_rational_candidates_reject_projective_poles_and_source_singularities() {
    let projective_source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)])
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
    let singular_source = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(1))
        .unwrap();
    let projective_other =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1), p(2, 0)], vec![r(1), r(-1), r(1)]).unwrap();
    let finite_other = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            projective_source
                .intersection_candidates(&finite_other, &policy)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
        assert_eq!(
            singular_source
                .intersection_candidates(&finite_other, &policy)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
        assert_eq!(
            QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
                .parallel_left(r(1))
                .unwrap()
                .intersection_candidates(&projective_other, &policy)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn zero_distance_parallel_candidates_keep_stationary_source_intersection() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(0, -1), p(0, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel
                .intersection_candidates(&vertical, &policy)
                .unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::Candidates {
                first_parameters: vec![BezierParameter2::Exact(r(0))],
                second_parameters: vec![BezierParameter2::Exact(q(1, 2))],
            })
        );
    }
}

#[test]
fn parallel_rational_contacts_replay_exact_pair_and_transversality() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&vertical, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, 1)));
        assert!(contacts[0].is_certified_transverse());
    }
}

#[test]
fn parallel_rational_contacts_reject_the_squared_opposite_branch() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&vertical, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(3, 4))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, 1)));
    }
}

#[test]
fn parallel_rational_contacts_preserve_negative_distance_and_weight_orientation() {
    let right_parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(-1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(-1), r(-1)]).unwrap();
    let rational_source = RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![r(-1), r(-1)])
        .unwrap()
        .parallel_left(r(1))
        .unwrap();
    let positive_vertical =
        RationalBezier2::try_new(vec![p(1, -2), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(right_parallel.intersections(&vertical, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 4))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, -1)));

        let intersections = decided_parallel_set(
            rational_source
                .intersections(&positive_vertical, &policy)
                .unwrap(),
        );
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(3, 4))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, 1)));
    }
}

#[test]
fn parallel_rational_contacts_replay_one_algebraic_parameter_exactly() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&vertical, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parallel_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, 1)));
    }
}

#[test]
fn parallel_rational_contacts_replay_identical_algebraic_parameters() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let target = RationalBezier2::try_new(
        vec![p(1, 0), Point2::new(r(1), q(1, 2)), p(1, 2)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            (
                contacts[0].parallel_parameter(),
                contacts[0].other_parameter()
            ),
            (
                BezierParameter2::Algebraic(_),
                BezierParameter2::Algebraic(_)
            )
        ));
        assert!((contacts[0].point()).coordinates().is_none());
    }
}

#[test]
fn parallel_rational_contacts_lift_coupled_distinct_algebraic_parameters() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(0)).unwrap();
    let target = RationalBezier2::try_new(vec![p(0, 1), p(2, 0)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersections(&target, &policy).unwrap();
        let intersections = decided_parallel_set(replay.clone());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parallel_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert!(matches!(
            contacts[0].other_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert_ne!(
            contacts[0].parallel_parameter(),
            contacts[0].other_parameter()
        );
        assert!((contacts[0].point()).coordinates().is_none());
    }
}

#[test]
fn parallel_rational_lift_pairs_multiple_algebraic_projections_without_cross_product() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(0)).unwrap();
    let target = RationalBezier2::try_new(
        vec![Point2::new(r(0), q(-1, 5)), Point2::new(r(2), q(9, 5))],
        vec![r(1), r(1)],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersections(&target, &policy).unwrap();
        let intersections = decided_parallel_set(replay.clone());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 2);
        assert!(contacts.iter().all(|contact| {
            matches!(contact.parallel_parameter(), BezierParameter2::Algebraic(_))
                && matches!(contact.other_parameter(), BezierParameter2::Algebraic(_))
        }));
        assert_ne!(contacts[0].point(), contacts[1].point());
    }
}

#[test]
fn parallel_rational_contacts_replay_selected_branch_at_a_coupled_algebraic_pair() {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let parallel = source.parallel_left(r(1)).unwrap();
    let target = RationalBezier2::try_new(vec![p(-1, 1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersections(&target, &policy).unwrap();
        let intersections = decided_parallel_set(replay.clone());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 2);
        assert!(contacts.iter().any(|contact| {
            contact.parallel_parameter() == &BezierParameter2::Exact(r(0))
                && contact.other_parameter() == &BezierParameter2::Exact(q(1, 2))
        }));
        assert!(contacts.iter().any(|contact| {
            matches!(contact.parallel_parameter(), BezierParameter2::Algebraic(_))
                && matches!(contact.other_parameter(), BezierParameter2::Algebraic(_))
        }));
    }
}

#[test]
fn parallel_rational_contacts_handle_higher_nullity_algebraic_fibers() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let target =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 0), p(1, 2)], vec![r(1), r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let replay = parallel.intersections(&target, &policy).unwrap();
        let intersections = decided_parallel_set(replay);
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parallel_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert!(matches!(
            contacts[0].other_parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert!((contacts[0].point()).coordinates().is_none());
    }
}

#[test]
fn zero_distance_parallel_contacts_keep_stationary_source_contact() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
        .parallel_left(r(0))
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(0, -1), p(0, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&vertical, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(r(0))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(0, 0)));
        assert!(!contacts[0].is_certified_transverse());
    }
}

#[test]
fn parallel_rational_contacts_resolve_selected_and_opposite_shared_components() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let disjoint = RationalBezier2::try_new(vec![p(10, 0), p(10, 2)], vec![r(1), r(1)]).unwrap();
    let coincident = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![r(1), r(1)]).unwrap();
    let opposite = RationalBezier2::try_new(vec![p(0, -1), p(2, -1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let disjoint_intersections =
            decided_parallel_set(parallel.intersections(&disjoint, &policy).unwrap());
        assert!(disjoint_intersections.is_empty());
        let intersections =
            decided_parallel_set(parallel.intersections(&coincident, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
        let opposite_intersections =
            decided_parallel_set(parallel.intersections(&opposite, &policy).unwrap());
        assert!(opposite_intersections.is_empty());
    }
}

#[test]
fn parallel_rational_contacts_retain_partial_and_reversed_overlap_ranges() {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(r(1))
        .unwrap();
    let partial = RationalBezier2::try_new(vec![p(1, 1), p(3, 1)], vec![r(1), r(1)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&partial, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&q(1, 2), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &q(1, 2)))
        );
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);

        let reversed_intersections = decided_parallel_set(
            parallel
                .intersections(&partial.reversed(), &policy)
                .unwrap(),
        );
        let reversed = only_parallel_overlap(&reversed_intersections);
        assert_eq!(
            reversed.first_range().scalar_endpoints(),
            Some((&q(1, 2), &Real::one()))
        );
        assert_eq!(reversed.orientation(), CurveOverlapOrientation2::Reversed);
    }
}

#[test]
fn parallel_rational_contacts_transport_a_nonlinear_rational_parameter_component() {
    let parallel = QuadraticBezier2::new(
        p(0, 0),
        Point2::new(q(3, 16), r(0)),
        Point2::new(q(3, 8), q(9, 64)),
    )
    .parallel_left(r(1))
    .unwrap();
    let target = rationally_reparameterized_parabola_parallel();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            parallel
                .exact_pythagorean_hodograph_offset(&policy)
                .unwrap(),
            Classification::Decided(None)
        ));
        assert_eq!(
            parallel.intersection_candidates(&target, &policy).unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
        );

        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);

        let Classification::Decided(partial) = target
            .subcurve_between_exact_with_policy(&q(1, 4), &q(3, 4), &policy)
            .unwrap()
        else {
            panic!("rationalized parallel subcurve was not decided");
        };
        let partial_intersections =
            decided_parallel_set(parallel.intersections(&partial, &policy).unwrap());
        let partial = only_parallel_overlap(&partial_intersections);
        assert_eq!(
            partial.first_range().scalar_endpoints(),
            Some((&q(5, 28), &q(13, 20)))
        );
        assert_eq!(
            partial.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(partial.orientation(), CurveOverlapOrientation2::Same);

        let reversed_intersections =
            decided_parallel_set(parallel.intersections(&target.reversed(), &policy).unwrap());
        let reversed = only_parallel_overlap(&reversed_intersections);
        assert_eq!(
            reversed.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            reversed.second_range().scalar_endpoints(),
            Some((&Real::one(), &Real::zero()))
        );
        assert_eq!(reversed.orientation(), CurveOverlapOrientation2::Reversed);
    }
}

#[test]
fn parallel_rational_contacts_transport_an_implicit_parameter_component() {
    let parallel = nonlinearly_reparameterized_parabola()
        .parallel_left(Real::one())
        .unwrap();
    let target = rationally_reparameterized_parabola_parallel();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            parallel
                .exact_pythagorean_hodograph_offset(&policy)
                .unwrap(),
            Classification::Decided(None)
        ));
        assert!(matches!(
            parallel.intersection_candidates(&target, &policy).unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
        ));
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
    }
}

#[test]
fn parallel_rational_contacts_partition_two_turning_implicit_graphs() {
    // P(v)=(v,v^2), source v=1/16+(t-1/2)^2/32, and target
    // v=u(1-u) produce H(t,u)=u^2-u+1/16+(t-1/2)^2/32. The source image is
    // covered by two target branches, and each branch turns at t=1/2.
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(9, 128), q(81, 16_384)),
            Point2::new(q(1, 16), q(63, 16_384)),
            Point2::new(q(23, 384), q(179, 49_152)),
            Point2::new(q(1, 16), q(63, 16_384)),
            Point2::new(q(9, 128), q(81, 16_384)),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let target = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 4), Real::zero()),
            Point2::new(q(1, 3), q(1, 6)),
            Point2::new(q(1, 4), Real::zero()),
            p(0, 0),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let half = q(1, 2);

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.contacts().is_empty());
        let [lower_left, lower_right, upper_left, upper_right] = intersections.overlaps() else {
            panic!("two turning parameter graphs must produce four overlap cells");
        };
        for (left, right) in [(lower_left, lower_right), (upper_left, upper_right)] {
            assert_eq!(
                left.first_range().scalar_endpoints(),
                Some((&Real::zero(), &half))
            );
            assert_eq!(
                right.first_range().scalar_endpoints(),
                Some((&half, &Real::one()))
            );
            assert_eq!(left.second_range().end(), right.second_range().start());
        }
        assert_eq!(
            [
                lower_left.orientation(),
                lower_right.orientation(),
                upper_left.orientation(),
                upper_right.orientation(),
            ],
            [
                CurveOverlapOrientation2::Reversed,
                CurveOverlapOrientation2::Same,
                CurveOverlapOrientation2::Same,
                CurveOverlapOrientation2::Reversed,
            ]
        );
    }
}

#[test]
fn parallel_rational_contacts_partition_a_closed_implicit_oval() {
    // P(v)=(v,v^2), source v=(t-1/2)^2, and target
    // v=1/16-(u-1/2)^2 produce the closed parameter correspondence
    // (t-1/2)^2+(u-1/2)^2=1/16. Neither parameter is a global graph
    // coordinate, so the authoritative component topology must traverse folds
    // in both projections.
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 4), q(1, 16)),
            Point2::new(Real::zero(), q(-1, 16)),
            Point2::new(q(-1, 12), q(1, 16)),
            Point2::new(Real::zero(), q(-1, 16)),
            Point2::new(q(1, 4), q(1, 16)),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let target = RationalBezier2::try_new(
        vec![
            Point2::new(q(-3, 16), q(9, 256)),
            Point2::new(q(1, 16), q(-15, 256)),
            Point2::new(q(7, 48), q(59, 768)),
            Point2::new(q(1, 16), q(-15, 256)),
            Point2::new(q(-3, 16), q(9, 256)),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let quarter = q(1, 4);
    let half = q(1, 2);
    let three_quarters = q(3, 4);

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.contacts().is_empty());
        let [lower_left, lower_right, upper_left, upper_right] = intersections.overlaps() else {
            panic!("the closed parameter oval must produce four overlap cells");
        };
        for overlap in [lower_left, upper_left] {
            assert_eq!(
                overlap.first_range().scalar_endpoints(),
                Some((&quarter, &half))
            );
        }
        for overlap in [lower_right, upper_right] {
            assert_eq!(
                overlap.first_range().scalar_endpoints(),
                Some((&half, &three_quarters))
            );
        }
        assert_eq!(
            [
                lower_left.orientation(),
                lower_right.orientation(),
                upper_left.orientation(),
                upper_right.orientation(),
            ],
            [
                CurveOverlapOrientation2::Reversed,
                CurveOverlapOrientation2::Same,
                CurveOverlapOrientation2::Same,
                CurveOverlapOrientation2::Reversed,
            ]
        );
    }
}

#[test]
fn parallel_rational_contacts_partition_an_implicit_cusp() {
    // P(v)=(v,v^2), source v=(t-1/2)^3, and target
    // v=(u-1/2)^2 produce the singular parameter correspondence
    // (u-1/2)^2=(t-1/2)^3. The two real branches meet at one cusp
    // parameter pair and must be published as independent exact cells.
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(-1, 8), q(1, 64)),
            Point2::new(Real::zero(), q(-1, 64)),
            Point2::new(q(1, 40), q(1, 64)),
            Point2::new(Real::zero(), q(-1, 64)),
            Point2::new(q(-1, 40), q(1, 64)),
            Point2::new(Real::zero(), q(-1, 64)),
            Point2::new(q(1, 8), q(1, 64)),
        ],
        vec![Real::one(); 7],
    )
    .unwrap();
    let target = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 4), q(1, 16)),
            Point2::new(Real::zero(), q(-1, 16)),
            Point2::new(q(-1, 12), q(1, 16)),
            Point2::new(Real::zero(), q(-1, 16)),
            Point2::new(q(1, 4), q(1, 16)),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let half = q(1, 2);

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.contacts().is_empty());
        assert_eq!(intersections.overlaps().len(), 2);
        for orientation in [
            CurveOverlapOrientation2::Reversed,
            CurveOverlapOrientation2::Same,
        ] {
            let overlap = intersections
                .overlaps()
                .iter()
                .find(|overlap| overlap.orientation() == orientation)
                .expect("the cusp must retain both oriented branches");
            assert_eq!(
                overlap.first_range().scalar_endpoints(),
                Some((&half, &Real::one()))
            );
            assert_eq!(overlap.second_range().start().scalar(), Some(&half));
            assert!(overlap.second_range().end().scalar().is_none());
        }
    }
}

#[test]
fn parallel_rational_contacts_partition_a_noninjective_parameter_component() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 0))
        .parallel_left(Real::one())
        .unwrap();
    // x(u)=4u(1-u) traverses the selected line parallel once in each
    // direction, meeting at the stationary parameter u=1/2.
    let target = RationalBezier2::try_new(
        vec![
            p(0, 1),
            p(1, 1),
            Point2::new(q(4, 3), Real::one()),
            p(1, 1),
            p(0, 1),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            parallel.intersection_candidates(&target, &policy).unwrap(),
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant)
        ));
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.contacts().is_empty());
        assert_eq!(intersections.overlaps().len(), 2);
        let same = intersections
            .overlaps()
            .iter()
            .find(|overlap| overlap.orientation() == CurveOverlapOrientation2::Same)
            .expect("forward noninjective branch was not retained");
        let reversed = intersections
            .overlaps()
            .iter()
            .find(|overlap| overlap.orientation() == CurveOverlapOrientation2::Reversed)
            .expect("reverse noninjective branch was not retained");
        assert_eq!(
            same.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            same.second_range().scalar_endpoints(),
            Some((&Real::zero(), &q(1, 2)))
        );
        assert_eq!(
            reversed.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            reversed.second_range().scalar_endpoints(),
            Some((&Real::one(), &q(1, 2)))
        );
    }
}

#[test]
fn parallel_rational_contacts_retain_an_isolated_component_domain_touch() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 0))
        .parallel_left(Real::one())
        .unwrap();
    // x(u)=-(2u-1)^2 lies outside the parallel's x-domain except for the
    // stationary touch x=0 at u=1/2. The algebraic equations share a full
    // parameter component, but its intersection with the closed authored
    // parameter square is one isolated contact rather than an overlap.
    let target = RationalBezier2::try_new(
        vec![
            p(-1, 1),
            p(0, 1),
            Point2::new(q(1, 3), Real::one()),
            p(0, 1),
            p(-1, 1),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        let contacts = only_parallel_contacts(&intersections);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parallel_parameter(),
            &BezierParameter2::Exact(Real::zero())
        );
        assert_eq!(
            contacts[0].other_parameter(),
            &BezierParameter2::Exact(q(1, 2))
        );
        assert_eq!(contacts[0].point(), &CurvePoint2::from(p(0, 1)));
        assert!(!contacts[0].is_certified_transverse());
    }
}

#[test]
fn parallel_rational_component_can_yield_overlaps_and_an_isolated_contact() {
    let parallel = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 0))
        .parallel_left(Real::one())
        .unwrap();
    // x(u)=u(1/2-u)(u-3/4)^2 is inside the parallel domain on
    // [0,1/2], traversing that small range in both directions, and touches
    // x=0 once more at the isolated double root u=3/4.
    let target = RationalBezier2::try_new(
        vec![
            p(0, 1),
            Point2::new(q(9, 128), Real::one()),
            Point2::new(q(-5, 64), Real::one()),
            Point2::new(q(7, 128), Real::one()),
            Point2::new(q(-1, 32), Real::one()),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        assert!(intersections.is_complete(), "{intersections:?}");
        assert_eq!(intersections.overlaps().len(), 2);
        let [contact] = intersections.contacts() else {
            panic!("mixed component did not retain exactly one isolated contact");
        };
        assert_eq!(
            contact.parallel_parameter(),
            &BezierParameter2::Exact(Real::zero())
        );
        assert_eq!(contact.other_parameter(), &BezierParameter2::Exact(q(3, 4)));
        assert_eq!(contact.point(), &CurvePoint2::from(p(0, 1)));
        assert!(
            intersections
                .overlaps()
                .iter()
                .any(|overlap| overlap.orientation() == CurveOverlapOrientation2::Same)
        );
        assert!(
            intersections
                .overlaps()
                .iter()
                .any(|overlap| { overlap.orientation() == CurveOverlapOrientation2::Reversed })
        );
    }
}

#[test]
fn parallel_rational_contacts_clip_a_component_at_both_curve_domains() {
    let source = QuadraticBezier2::new(
        p(0, 0),
        Point2::new(q(3, 16), r(0)),
        Point2::new(q(3, 8), q(9, 64)),
    );
    let target = rationally_reparameterized_parabola_parallel();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel = source
            .subcurve_between_exact_with_policy(&Real::zero(), &q(7, 10), &policy)
            .unwrap()
            .parallel_left(r(1))
            .unwrap();
        let Classification::Decided(target) = target
            .subcurve_between_exact_with_policy(&q(1, 10), &q(9, 10), &policy)
            .unwrap()
        else {
            panic!("rationalized target subcurve was not decided");
        };
        let intersections = decided_parallel_set(parallel.intersections(&target, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);

        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&q(13, 133), &Real::one()))
        );
        assert!(matches!(
            overlap.second_range().start(),
            BezierParameter2::Exact(value) if value == &Real::zero()
        ));
        assert!(matches!(
            overlap.second_range().end(),
            BezierParameter2::Algebraic(_)
        ));
        assert_eq!(
            overlap
                .second_range()
                .end()
                .cmp_by_refinement(&BezierParameter2::Exact(q(4, 5)), &policy)
                .unwrap(),
            Classification::Decided(std::cmp::Ordering::Greater)
        );
        assert_eq!(
            overlap
                .second_range()
                .end()
                .cmp_by_refinement(&BezierParameter2::Exact(q(9, 10)), &policy)
                .unwrap(),
            Classification::Decided(std::cmp::Ordering::Less)
        );
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
    }
}

#[test]
fn zero_distance_non_ph_parallel_reuses_the_exact_source_overlap() {
    let source = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let same_source = RationalBezier2::try_new(
        source.control_points().into_iter().cloned().collect(),
        vec![Real::one(); 3],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections =
            decided_parallel_set(parallel.intersections(&same_source, &policy).unwrap());
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
    }
}

#[test]
fn independently_constructed_ph_parallel_reuses_rational_overlap_authority() {
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), Real::zero()),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), Real::one()),
    );
    let parallel = source.parallel_left(Real::one()).unwrap();
    let Classification::Decided(Some(materialized)) = parallel
        .exact_pythagorean_hodograph_offset(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("canonical PH cubic did not materialize exactly");
    };
    let Classification::Decided(independently_constructed) =
        RationalBezier2::from_homogeneous_controls_with_policy(
            materialized.curve().homogeneous_controls().to_vec(),
            &CurveContext::STRICT,
        )
        .unwrap()
    else {
        panic!("the reconstructed PH endpoints must remain finite");
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let intersections = decided_parallel_set(
            parallel
                .intersections(&independently_constructed, &policy)
                .unwrap(),
        );
        let overlap = only_parallel_overlap(&intersections);
        assert_eq!(
            overlap.first_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
        assert_eq!(
            overlap.second_range().scalar_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );
    }
}

#[test]
fn parallel_rational_contacts_inherit_the_approximate_512_terminal() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
        .parallel_left(undecidable_zero)
        .unwrap();
    let vertical = RationalBezier2::try_new(vec![p(1, -1), p(1, 1)], vec![r(1), r(1)]).unwrap();

    assert_eq!(
        parallel.intersections(&vertical, &CurveContext::STRICT),
        Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign
        ))
    );
    let intersections = decided_parallel_set(
        parallel
            .intersections(&vertical, &CurveContext::APPROXIMATE_512)
            .unwrap(),
    );
    let contacts = only_parallel_contacts(&intersections);
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].point(), &CurvePoint2::from(p(1, 0)));
}

fn circle_point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn analytic_parallel_intersects_independently_parameterized_circles_exactly() {
    let center = circle_point(1, 2);
    let source = QuadraticBezier2::new(circle_point(0, 0), circle_point(1, 0), circle_point(1, 1));
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let mut contact_count = 0;
        for distance in [quarter.clone(), -quarter.clone()] {
            let parallel = source.parallel_left(distance.clone()).unwrap();
            let outward = distance == quarter;
            let radius_scale = Real::one() - distance;
            let scaled = |point: Point2| {
                let radial = point.delta_from(&center);
                center.translated(&radial.0 * &radius_scale, &radial.1 * &radius_scale)
            };
            for major in [false, true] {
                if major && !outward {
                    continue;
                }
                let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
                    scaled(circle_point(1, 1)),
                    scaled(circle_point(2, 1)),
                    scaled(circle_point(2, 2)),
                    if major {
                        -&half_sqrt_two
                    } else {
                        half_sqrt_two.clone()
                    },
                )
                .unwrap()
                .into();
                let intersections = match parallel.intersections(&circle, &policy).unwrap() {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        panic!("analytic/circle intersection remained uncertain: {reason:?}")
                    }
                };
                assert!(intersections.is_complete());
                // The positive parallel stays at y<=1 while the circle has
                // y>=5/4. Its certified empty incidence must close the query.
                assert_eq!(intersections.contacts().len(), usize::from(!outward));
                contact_count += intersections.contacts().len();
            }
        }
        assert_eq!(contact_count, 1);
    }
}

#[test]
fn analytic_parallel_circle_tangency_retains_zero_cross_evidence() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for swap_axes in [false, true] {
            let p = |x, y| {
                if swap_axes {
                    circle_point(y, x)
                } else {
                    circle_point(x, y)
                }
            };
            for reverse_source in [false, true] {
                let source = if reverse_source {
                    QuadraticBezier2::new(p(2, 0), p(0, 0), p(-2, 0))
                } else {
                    QuadraticBezier2::new(p(-2, 0), p(0, 0), p(2, 0))
                };
                // Reflection or source reversal changes the selected left
                // normal; negating the distance preserves the same offset set.
                let distance = if swap_axes != reverse_source {
                    -Real::one()
                } else {
                    Real::one()
                };
                let parallel = source.parallel_left(distance).unwrap();
                for reverse_circle in [false, true] {
                    let mut controls = [p(1, 0), p(1, 1), p(0, 1)];
                    if reverse_circle {
                        controls.reverse();
                    }
                    let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
                        controls[0].clone(),
                        controls[1].clone(),
                        controls[2].clone(),
                        half_sqrt_two.clone(),
                    )
                    .unwrap()
                    .into();
                    let intersections = match parallel.intersections(&circle, &policy).unwrap() {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            panic!("analytic/circle tangency remained uncertain: {reason:?}")
                        }
                    };
                    assert!(intersections.is_complete());
                    let [contact] = intersections.contacts() else {
                        panic!("analytic/circle tangency must retain exactly one contact")
                    };
                    assert_eq!(
                        contact
                            .point()
                            .coincides_with(&p(0, 1).into(), &policy)
                            .value,
                        Classification::Decided(true)
                    );
                    for (parameter, expected) in [
                        (contact.parallel_parameter(), half.clone()),
                        (
                            contact.other_parameter(),
                            if reverse_circle {
                                Real::zero()
                            } else {
                                Real::one()
                            },
                        ),
                    ] {
                        assert_eq!(
                            parameter
                                .cmp_by_refinement(&BezierParameter2::Exact(expected), &policy)
                                .unwrap(),
                            Classification::Decided(std::cmp::Ordering::Equal)
                        );
                    }
                    assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Zero));
                    assert_eq!(
                        contact.tangent_dot_sign(),
                        Some(if reverse_source != reverse_circle {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    assert!(!contact.is_certified_transverse());
                }
            }
        }
    }
}

#[test]
fn analytic_parallel_circle_fast_path_excludes_other_support_contacts() {
    let source = QuadraticBezier2::new(
        circle_point(-2, -1),
        circle_point(0, -1),
        circle_point(2, -1),
    );
    let half_sqrt_two = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
    let circle: RationalBezier2 = RationalQuadraticBezier2::try_unit_end_weights(
        circle_point(1, 0),
        circle_point(1, 1),
        circle_point(0, 1),
        half_sqrt_two,
    )
    .unwrap()
    .into();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel = source.parallel_left(Real::one()).unwrap();
        let intersections = match parallel.intersections(&circle, &policy).unwrap() {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                panic!("analytic/circle span filtering remained uncertain: {reason:?}")
            }
        };
        assert!(intersections.is_complete());
        let [contact] = intersections.contacts() else {
            panic!("one of two supporting-circle contacts lies on the retained quarter")
        };
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Positive));
        assert!(contact.is_certified_transverse());
    }
}
