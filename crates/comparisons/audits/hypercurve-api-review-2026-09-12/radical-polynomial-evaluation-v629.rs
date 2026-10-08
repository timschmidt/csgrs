
#[cfg(test)]
mod radical_polynomial_evaluation_probe {
    use super::*;

    fn cold_cusp_polynomial(scale: Rational, quadratic_formula: bool) {
        let target = Real::new(Rational::fraction(81, 25).unwrap());
        let cube_root = target.root_n(3).unwrap();
        let alpha = if quadratic_formula {
            let constant = Real::one() - cube_root;
            let quadratic = Real::from(4);
            let discriminant = -Real::from(4) * &quadratic * &constant;
            let denominator = Real::from(2) * quadratic;
            (discriminant / (&denominator * &denominator)).unwrap().sqrt().unwrap()
        } else {
            ((cube_root - Real::one()) / Real::from(4)).unwrap().sqrt().unwrap()
        };
        let coefficients = [196, 0, -1400, 0, -2325, 0, 1900, 0, 10000]
            .map(|value| Real::new(Rational::new(value) * &scale));
        // Sign immediately: constructing the equivalent factored expression
        // first could publish a reusable radical certificate.
        let expanded = Real::eval_poly(&coefficients, &alpha);
        let sign = expanded.certified_sign_until(-512).sign();
        eprintln!("COLD_SCALAR_IDENTITY quadratic_formula={quadratic_formula} sign={sign:?}");
        assert_eq!(sign, Some(RealSign::Zero));
    }

    #[test]
    fn cold_integral_cusp_polynomial() {
        cold_cusp_polynomial(Rational::one(), false);
    }

    #[test]
    fn cold_actual_resultant_cusp_polynomial() {
        cold_cusp_polynomial(Rational::fraction(16777216, 625).unwrap(), false);
    }

    #[test]
    fn cold_quadratic_formula_cusp_polynomial() {
        cold_cusp_polynomial(Rational::fraction(16777216, 625).unwrap(), true);
    }
}
