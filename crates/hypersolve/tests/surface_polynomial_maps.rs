use hyperreal::Real;
use hypersolve::bivariate_arithmetic::{bivariate_compose, polynomial_add, polynomial_scale};
use hypersolve::curve_substitution::{
    bernstein_scalar_power_basis, bspline_span_basis_power_basis,
};
use hypersolve::{BivariatePolynomial, BsplineKnotSpanSubstitutionStatus};

fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

#[test]
fn bernstein_coefficients_preserve_exact_large_degree_identities() {
    let mut monomial = vec![Real::zero(); 81];
    monomial[80] = Real::one();
    assert_eq!(bernstein_scalar_power_basis(&monomial), monomial);
    let linear = (0..81).map(|i| q(i, 80)).collect::<Vec<_>>();
    let mut expected = vec![Real::zero(); 81];
    expected[1] = Real::one();
    assert_eq!(bernstein_scalar_power_basis(&linear), expected);
    let mut expected = vec![Real::zero(); 81];
    expected[0] = Real::pi();
    assert_eq!(
        bernstein_scalar_power_basis(&vec![Real::pi(); 81]),
        expected
    );
    assert!(bernstein_scalar_power_basis(&[]).is_empty());
}

#[test]
fn nonuniform_bspline_spans_reproduce_partition_linear_and_quadratic_fields() {
    let knots = vec![
        2.into(),
        2.into(),
        2.into(),
        3.into(),
        5.into(),
        5.into(),
        5.into(),
    ];
    let greville = [2.into(), q(5, 2), 4.into(), 5.into()];
    let squared = [4.into(), 6.into(), 15.into(), 25.into()];
    for span in [2, 3] {
        let basis = bspline_span_basis_power_basis(&knots, 2, span, -512).unwrap();
        let combine = |coefficients: &[Real; 4]| {
            basis.iter().enumerate().fold(Vec::new(), |sum, (i, p)| {
                polynomial_add(&sum, &polynomial_scale(p, &coefficients[span - 2 + i]))
            })
        };
        assert_eq!(
            combine(&std::array::from_fn(|_| Real::one())),
            vec![Real::one(), Real::zero(), Real::zero()]
        );
        assert_eq!(
            combine(&greville),
            vec![Real::zero(), Real::one(), Real::zero()]
        );
        assert_eq!(
            combine(&squared),
            vec![Real::zero(), Real::zero(), Real::one()]
        );
    }
    assert_eq!(
        bspline_span_basis_power_basis(&knots, 2, 0, -512),
        Err(BsplineKnotSpanSubstitutionStatus::SpanOutOfRange)
    );
    let mut invalid = knots.clone();
    invalid[3] = 1.into();
    assert_eq!(
        bspline_span_basis_power_basis(&invalid, 2, 2, -512),
        Err(BsplineKnotSpanSubstitutionStatus::DecreasingKnotVector)
    );
    let repeated = vec![
        0.into(),
        0.into(),
        0.into(),
        q(1, 2),
        q(1, 2),
        1.into(),
        1.into(),
        1.into(),
    ];
    assert_eq!(
        bspline_span_basis_power_basis(&repeated, 2, 3, -512),
        Err(BsplineKnotSpanSubstitutionStatus::ZeroLengthSpan)
    );
}

#[test]
fn mixed_polynomial_pullback_preserves_joint_relation() {
    // (3u-4v+2-2)^2 + (4u+3v-1+1)^2 = 25(u²+v²).
    let p = BivariatePolynomial::new(vec![
        vec![5.into(), 2.into(), 1.into()],
        vec![(-4).into()],
        vec![1.into()],
    ]);
    let x = BivariatePolynomial::new(vec![vec![2.into(), (-4).into()], vec![3.into()]]);
    let y = BivariatePolynomial::new(vec![vec![(-1).into(), 3.into()], vec![4.into()]]);
    let mapped = bivariate_compose(&p, &x, &y);
    for (i, row) in mapped.coefficients.iter().enumerate() {
        for (j, c) in row.iter().enumerate() {
            assert_eq!(
                c,
                &Real::from(if (i, j) == (2, 0) || (i, j) == (0, 2) {
                    25
                } else {
                    0
                })
            );
        }
    }
}

#[test]
fn homogeneous_field_evaluation_preserves_degree_and_shared_denominator() {
    use hypersolve::DenseTensorPolynomial;
    use hypersolve::recursive_quadratic_field::{
        RecursiveQuadraticField, evaluate_bivariate_homogeneous_polynomial,
    };
    let one = DenseTensorPolynomial::try_new(vec![], vec![Real::one()]).unwrap();
    let field = RecursiveQuadraticField::base(vec![], one.clone(), one).unwrap();
    let x = field.constant(2.into()).unwrap();
    let y = field.constant(3.into()).unwrap();
    let w = field.constant(5.into()).unwrap();
    let p = BivariatePolynomial::new(vec![
        vec![1.into(), 0.into(), 2.into()],
        vec![0.into(), 3.into()],
    ]);
    let value = evaluate_bivariate_homogeneous_polynomial(&p, &x, &y, &w, 2).unwrap();
    assert_eq!(
        value.exact_real_value_with_retained_witnesses(),
        Some(Real::from(61))
    );
    assert!(evaluate_bivariate_homogeneous_polynomial(&p, &x, &y, &w, 1).is_none());
}
