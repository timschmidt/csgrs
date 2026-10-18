use hypercurve::Point2;
use hyperlimit::PredicatePolicy;
use hyperreal::{Real, ZeroKnowledge};

fn fraction(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn mixed_rational_and_symbolic_coordinates_keep_both_proof_paths() {
    let epsilon: Real = "1e-400".parse().unwrap();
    for rational_axis in 0..2 {
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            hypercurve::evaluate_under(policy, || {
                let make = |rational: Real, symbolic: Real| {
                    if rational_axis == 0 {
                        Point2::new(rational, symbolic)
                    } else {
                        Point2::new(symbolic, rational)
                    }
                };
                let left = make(fraction(1, 3), Real::pi() * Real::from(2));
                let equal = make(fraction(2, 6), Real::pi() + Real::pi());
                let different_rational =
                    make(fraction(1, 3) + &epsilon, Real::pi() * Real::from(2));
                let different_symbolic = make(fraction(1, 3), Real::pi() * Real::from(3));
                assert_eq!(left, equal);
                assert_eq!(equal, left);
                assert_ne!(left, different_rational);
                assert_ne!(left, different_symbolic);
                assert_eq!(left, left.clone());
            })
            .certified()
            .expect("all consumed facts are exact");
        }
    }
}

#[test]
fn exact_construction_coordinates_match_the_affine_line_oracle() {
    // At t=1/n on (0,0)+t(n,m), the exact point is (1,m/n).
    // All constructor inputs below are integers and therefore dyadic.
    for n in 2..18 {
        for m in -5..=5 {
            let zero = Real::zero();
            let one = Real::one();
            let dx = Real::from(n);
            let dy = Real::from(m);
            let (_, [x, y]) = Real::exact_rational_parameterized_point2_known_dyadic(
                [&zero, &zero],
                [&dx, &dy],
                &one,
                &dx,
            )
            .unwrap();
            assert!(x.exact_rational_ref().is_some() && y.exact_rational_ref().is_some());
            let actual = Point2::new(x, y);
            let expected = Point2::new(Real::one(), fraction(m, n));
            let distinct = Point2::new(Real::one(), fraction(m + 1, n));
            assert_eq!(actual, expected);
            assert_eq!(expected, actual);
            assert_ne!(actual, distinct);
        }
    }
}

#[test]
fn known_rational_coordinate_preserves_symbolic_coordinate_proofs() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let identity_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let epsilon: Real = "1e-400".parse().unwrap();
    // The trigonometric identity is zero; adding this exact positive value
    // gives a distinct point even while its structural proof is unresolved.
    let distinct_symbolic = &identity_zero + epsilon;
    for rational_axis in 0..2 {
        let make = |symbolic: Real| {
            if rational_axis == 0 {
                Point2::new(fraction(1, 3), symbolic)
            } else {
                Point2::new(symbolic, fraction(1, 3))
            }
        };
        let origin = make(Real::zero());
        let equal = make(identity_zero.clone());
        let distinct = make(distinct_symbolic.clone());
        for precision in [-8, -128, -16, -96] {
            for coordinate in [equal.x(), equal.y(), distinct.x(), distinct.y()] {
                coordinate.certified_rational_interval(precision).unwrap();
            }
            assert_ne!(identity_zero.zero_status(), ZeroKnowledge::NonZero);
            assert_ne!(distinct_symbolic.zero_status(), ZeroKnowledge::Zero);
            // A new exact-zero proof may strengthen an inconclusive comparison.
            // Do not require Unknown or a particular failed proof history.
            if identity_zero.zero_status() == ZeroKnowledge::Zero {
                assert_eq!(origin, equal);
                assert_eq!(equal, origin);
            }
            // Equality of the rational coordinate alone must not certify this
            // independently distinct symbolic coordinate, on either axis.
            assert_ne!(origin, distinct);
            assert_ne!(distinct, origin);
        }
    }
}
