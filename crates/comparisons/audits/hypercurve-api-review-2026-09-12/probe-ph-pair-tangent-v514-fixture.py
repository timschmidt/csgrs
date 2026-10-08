def instrument(s):
    a=s.index('    fn regular_ph_branch_pairs_retain_exact_cusp_incidence()')
    b=s.index('    #[test]',a+10)
    part=s[a:b]
    needle='                        let contact = &result.contacts()[0];'
    assert part.count(needle)==1
    part=part.replace(needle,needle+"""
                        // Curvature is 2/[t^2(1+t^2)^2]. At t=1/2 the
                        // radius 25/128 center locus changes from negative
                        // to positive speed. Its owned one-sided tangents
                        // are opposite, although both point derivatives vanish.
                        for (parameter, range, expected) in [
                            (contact.first_parameter(), &left, RealSign::Negative),
                            (contact.second_parameter(), &right, RealSign::Positive),
                        ] {
                            let (_, dot) = match parallel
                                .vector_tangent_cross_and_dot_signs_on_regular_range(
                                    parameter, &q(3,5), &q(4,5),
                                    &CurveParameterRange2::from_bezier_range(range.clone()),
                                    &policy,
                                ).unwrap() {
                                Classification::Decided(signs) => signs,
                                Classification::Uncertain(reason) => panic!("independent PH tangent blocked: {reason:?}"),
                            };
                            assert_eq!(dot,expected);
                        }
                        eprintln!("PH pair tangent cross={:?} dot={:?}",contact.tangent_cross_sign(),contact.tangent_dot_sign());
                        assert_eq!(contact.tangent_cross_sign(),Some(RealSign::Zero));
                        assert_eq!(contact.tangent_dot_sign(),Some(RealSign::Negative));
""")
    return s[:a]+part+s[b:]
