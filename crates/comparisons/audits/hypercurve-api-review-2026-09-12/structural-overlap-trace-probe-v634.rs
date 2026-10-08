
#[cfg(test)]
mod structural_overlap_trace_regression {
    use super::*;

    fn reparameterized_offset(policy: CurveContext, reversed: bool) {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 4),
            Point2::from_values(3, -4),
            Point2::from_values(4, 0),
        );
        let half = q(1, 2);
        let reference = source.parallel_left(half.clone()).unwrap();
        let Classification::Decided(reference_contacts) = reference
            .parallel_intersections(&reference, &policy).unwrap()
        else { panic!("the independently qualified S-cubic must classify") };
        assert!(reference_contacts.is_complete());
        assert_eq!(reference_contacts.contacts().len(), 2);

        // Exact Bernstein coefficients of C((t+t^2)/2), derived with rational
        // polynomial composition. The chart maps 0 -> 0 and 1 -> 1, and its
        // derivative (1+2t)/2 is strictly positive throughout the unit span.
        // It preserves the source trace, selected left normal, and all offset
        // crossings without introducing stationary source points or poles.
        let controls = vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 4), Real::one()),
            Point2::new(q(13, 20), q(9, 5)),
            Point2::new(q(101, 80), q(33, 20)),
            Point2::new(q(43, 20), q(-1, 5)),
            Point2::new(q(13, 4), Real::from(-3)),
            Point2::from_values(4, 0),
        ];
        let reparameterized = RationalBezier2::try_new(controls, vec![Real::one(); 7]).unwrap();
        let parallel = reparameterized.parallel_left(half.clone()).unwrap();
        let other = if reversed {
            reparameterized.reversed().parallel_left(-half).unwrap()
        } else {
            parallel.clone()
        };
        let actual = parallel.parallel_intersections(&other, &policy).unwrap();
        let Classification::Decided(actual) = actual else {
            panic!("the same finite offset trace must remain representable and decidable");
        };
        eprintln!("REPARAMETERIZED_OVERLAP reversed={reversed} complete={} contacts={} overlaps={}", actual.is_complete(), actual.contacts().len(), actual.overlaps().len());
        assert!(actual.is_complete(), "the structural correspondence needs complete residual evidence");
        assert_eq!(actual.contacts().len(), 2, "monotone reparameterization preserves both off-correspondence contacts");
    }

    #[test]
    fn monotone_reparameterization_preserves_structural_pair_crossings_strict() {
        reparameterized_offset(CurveContext::STRICT, false);
    }

    #[test]
    fn monotone_reparameterization_preserves_structural_pair_crossings_approximate() {
        reparameterized_offset(CurveContext::APPROXIMATE_512, false);
    }

    #[test]
    fn reversed_monotone_reparameterization_preserves_structural_pair_crossings_strict() {
        reparameterized_offset(CurveContext::STRICT, true);
    }

    #[test]
    fn reversed_monotone_reparameterization_preserves_structural_pair_crossings_approximate() {
        reparameterized_offset(CurveContext::APPROXIMATE_512, true);
    }
}
