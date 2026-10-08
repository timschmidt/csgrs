
#[cfg(test)]
mod radical_polynomial_evaluation_probe {
    use super::*;

    #[test]
    fn cusp_boundary_polynomial_preserves_exact_radical_identity() {
        let target = Real::new(Rational::fraction(81, 25).unwrap());
        let cube_root = target.clone().root_n(3).unwrap();
        let alpha = ((&cube_root - Real::one()) / Real::from(4)).unwrap().sqrt().unwrap();
        let coefficients = [196, 0, -1400, 0, -2325, 0, 1900, 0, 10000].map(Real::from);
        let square = &alpha * &alpha;
        let expanded = Real::eval_poly(&coefficients, &alpha);
        let even = Real::eval_poly(&[196, -1400, -2325, 1900, 10000].map(Real::from), &square);
        let cusp = Real::eval_poly(&[-14, 0, 75, 0, 300, 0, 400].map(Real::from), &alpha);
        let cubic = Real::eval_poly(&[-14, 75, 300, 400].map(Real::from), &square);
        let speed = Real::one() + Real::from(4) * &square;
        let factored = speed.clone() * &speed * speed - target;
        for (label, value) in [
            ("expanded_degree8", &expanded),
            ("even_degree4", &even),
            ("cusp_degree6", &cusp),
            ("cubic_in_square", &cubic),
            ("factored_cusp", &factored),
        ] {
            eprintln!("SCALAR_IDENTITY {label} sign={:?}", value.certified_sign_until(-512).sign());
        }
        assert_eq!(expanded.certified_sign_until(-512).sign(), Some(RealSign::Zero));
    }
}
