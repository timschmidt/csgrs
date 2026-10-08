#[cfg(test)]
mod retained_structural_pair_domain_regression {
    use super::*;

    #[test]
    fn retained_structural_correspondence_preserves_off_diagonal_contact_domains() {
        let point = |x, y| Point2::from_values(x, y);
        let source = CubicBezier2::new(point(0, 0), point(1, 4), point(3, -4), point(4, 0));
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = source.parallel_left(half.clone()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(reference) =
                parallel.parallel_intersections(&parallel, &policy).unwrap()
            else {
                panic!("the complete native structural pair must be certified");
            };
            assert!(reference.is_complete());
            assert_eq!(reference.contacts().len(), 2);
            let contact = &reference.contacts()[0];
            let first = contact.first_parameter();
            let second = contact.second_parameter();
            let (lower, upper) = match first.cmp_by_refinement(second, &policy).unwrap() {
                Classification::Decided(Ordering::Less) => (first, second),
                Classification::Decided(Ordering::Greater) => (second, first),
                _ => panic!("the reference contact must be off the diagonal"),
            };
            let Classification::Decided(split) =
                lower.strict_scalar_between_ordered(upper, &policy).unwrap()
            else {
                panic!("distinct crossing parameters must admit a separating cut");
            };
            let ranges = [
                CurveParameterRange2::new_validated(Real::zero().into(), split.clone().into()),
                CurveParameterRange2::new_validated(split.into(), Real::one().into()),
            ];
            for swap in [false, true] {
                let requested = if swap {
                    [&ranges[1], &ranges[0]]
                } else {
                    [&ranges[0], &ranges[1]]
                };
                let domains = requested.map(|range| CurveParameterDomain2::new(range, None));
                let mut expected = Vec::new();
                for contact in reference.contacts() {
                    let parameters = [contact.first_parameter(), contact.second_parameter()];
                    if domains.iter().zip(parameters).all(|(domain, parameter)| {
                        domain
                            .contains_finite_parameter(parameter, &policy)
                            .unwrap()
                            == Classification::Decided(true)
                    }) {
                        expected.push(contact);
                    }
                }
                assert_eq!(
                    expected.len(),
                    1,
                    "the separated domains contain one off-diagonal crossing"
                );
                let Classification::Decided(actual) = parallel
                    .parallel_intersections_on_regular_ranges(
                        &parallel,
                        requested[0],
                        requested[1],
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("retained structural pair replay must classify");
                };
                assert!(actual.is_complete());
                assert_eq!(actual.contacts().len(), expected.len());
                for (got, wanted) in actual.contacts().iter().zip(expected) {
                    for (got, wanted) in [
                        (got.first_parameter(), wanted.first_parameter()),
                        (got.second_parameter(), wanted.second_parameter()),
                    ] {
                        assert_eq!(
                            got.same_value(wanted, &policy).unwrap(),
                            Classification::Decided(true)
                        );
                    }
                }
            }
        }
    }
}
