// Unrun public contract regressions for the ordered-interval migration.
#[cfg(test)]
mod finite_parameter_interval_contract {
    use hypercurve::{
        BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierMonotoneSpan,
        BezierParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification,
        Curve2, CurveContext, CurveError, CurveParameter2, ExactCurveError, Point2,
        QuadraticBezier2, RationalBezier2, Real,
    };
    use std::cmp::Ordering;

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }

    fn square_root(sign: i32, policy: &CurveContext) -> BezierAlgebraicParameter2 {
        let (lower, upper) = if sign < 0 { (-2, -1) } else { (1, 2) };
        let span = BezierMonotoneSpan::new(Real::from(lower), Real::from(upper)).unwrap();
        let interval = decided(BezierParameterInterval::from_monotone_span(&span, policy).unwrap());
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap())
    }

    #[test]
    fn exterior_root_images_preserve_equations_and_native_curve_domains() {
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new((Real::one() / Real::from(2)).unwrap(), Real::zero()),
            Point2::from_values(1, 1),
        );
        let native = Curve2::from(source.clone());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for sign in [-1, 1] {
                let root = square_root(sign, &policy);
                let parameter = BezierParameter2::Algebraic(root.clone());
                let value = Real::from(sign) * Real::from(2).sqrt().unwrap();
                assert!(matches!(
                    parameter.cmp_by_refinement(&BezierParameter2::Exact(value.clone()), &policy),
                    Ok(Classification::Decided(Ordering::Equal))
                ));
                // The unrestricted polynomial image of P(t)=(t,t²) is
                // (±sqrt(2),2); the authored segment still owns [0,1].
                let image = source.point_at_algebraic_parameter(&root, &policy).unwrap();
                assert_eq!(image.status(), BezierAlgebraicImageStatus::Transformed);
                assert_eq!(
                    image.x().unwrap().compare_to_real(&value, &policy),
                    Classification::Decided(Ordering::Equal)
                );
                assert_eq!(
                    image.y().unwrap().compare_to_real(&Real::from(2), &policy),
                    Classification::Decided(Ordering::Equal)
                );
                assert!(matches!(
                    native.point_at(&CurveParameter2::from(parameter), &policy),
                    Err(ExactCurveError::Invalid {
                        cause: CurveError::InvalidCurveParameter,
                        ..
                    })
                ));
            }
        }
    }

    #[test]
    fn positive_unit_weights_do_not_admit_exterior_algebraic_poles() {
        // W(t)=2-t² has Bernstein weights (2,2,1), all positive on
        // the unit span. At either exterior root, the y numerator is 2,
        // so these are actual affine poles rather than removable factors.
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
            ],
            vec![Real::from(2), Real::from(2), Real::one()],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for sign in [-1, 1] {
                let root = square_root(sign, &policy);
                let image = source.point_at_algebraic_parameter(&root, &policy).unwrap();
                assert_eq!(image.status(), BezierAlgebraicImageStatus::XImageFailed);
                assert!(image.x().is_none() && image.y().is_none());
                assert!(image.retained_coordinate_polynomials().is_none());
            }
        }
    }
}
