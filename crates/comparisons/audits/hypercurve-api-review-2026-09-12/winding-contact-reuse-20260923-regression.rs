    #[test]
    fn selected_corner_candidates_reenter_normalization_with_retained_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let cases = [
                (nonrepresented_chord_rational_arc_corner_region(&policy, false, false, false), 2, q(1, 100)),
                (nonrepresented_chord_rational_arc_corner_region(&policy, true, false, false), 1, q(1, 100)),
                (independent_oblique_chord_pair_corner_region(&policy, false), 1, Real::one()),
            ];
            for (source, corner, radius) in cases {
                let outcome = source.fillet_loop_vertex_by_radius(
                    0, corner, radius, CurveCornerMode2::TrimOrExtend, &policy,
                ).unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(outcome.value.candidate_count() > 0);
                for_each_corner_region(&outcome.value, |candidate| {
                    let normalized = candidate.regularized_region_raw(&policy)
                        .expect("retained corner contacts must replay without a new coordinate field");
                    assert!(normalized.has_regularized_filled_left_topology(&policy));
                    assert!(!normalized.is_empty());
                    assert_eq!(normalized.regularized_region_raw(&policy).unwrap(), normalized);
                });
            }
        }
    }

