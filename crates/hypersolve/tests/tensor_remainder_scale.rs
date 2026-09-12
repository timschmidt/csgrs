use hyperreal::{Rational, Real};
use hypersolve::{DenseTensorPolynomial, PredicatePolicy};

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

#[test]
fn tensor_remainders_preserve_wide_scale_and_independent_fiber_values() {
    let first_root = fraction(-3, 7);
    let second_root = fraction(5, 11);
    let modulus = vec![
        &first_root * &second_root,
        -(&first_root + &second_root),
        Real::one(),
    ];
    let wide = Real::from(2).powi_i64(1024).unwrap() + Real::from(17);
    for scale in [wide.clone(), -wide.inverse_ref().unwrap()] {
        let coefficients = (0..30)
            .map(|index| &scale * fraction((index * index - 9) as i64, (index % 7 + 1) as u64))
            .collect::<Vec<_>>();
        let tensor = DenseTensorPolynomial::try_new(vec![3, 5, 2], coefficients.clone()).unwrap();
        for modulus_scale in [fraction(2, 3), fraction(-5, 7)] {
            let modulus = modulus
                .iter()
                .map(|value| value * &modulus_scale)
                .collect::<Vec<_>>();
            let reduced = tensor
                .reduce_axis_modulo(1, &modulus, PredicatePolicy::STRICT)
                .unwrap();
            assert_eq!(reduced.dimensions(), &[3, 2, 2]);
            for outer in 0..3 {
                for inner in 0..2 {
                    // At either independent rational root, the original
                    // polynomial and its remainder must have the same value.
                    // These two values determine the linear remainder without
                    // repeating polynomial long division in the oracle.
                    let evaluate = |root: &Real| {
                        (0..5).fold(Real::zero(), |sum, power| {
                            sum + &coefficients[outer * 10 + power * 2 + inner]
                                * root.clone().powi_i64(power as i64).unwrap()
                        })
                    };
                    let first_value = evaluate(&first_root);
                    let second_value = evaluate(&second_root);
                    let slope =
                        ((second_value - &first_value) / (&second_root - &first_root)).unwrap();
                    let constant = first_value - &slope * &first_root;
                    assert_eq!(reduced.coefficients()[outer * 4 + inner], constant);
                    assert_eq!(reduced.coefficients()[outer * 4 + 2 + inner], slope);
                }
            }
        }
        assert_eq!(tensor.coefficients(), coefficients);
    }
}
