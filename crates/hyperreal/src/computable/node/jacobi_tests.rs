#[cfg(test)]
mod jacobi_node_tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable { internal: Arc::new(Node::new(approximation, BoundCache::Invalid, ExactSignCache::Invalid)), signal: None }
    }

    #[test]
    fn logarithm_argument_recovery_is_bounded_and_domain_checked() {
        let q = |n| Computable::rational(Rational::from(n));
        let ln = raw(Approximation::PrescaledLnRational(Rational::fraction(3, 7).unwrap()));
        let cases = [
            (Computable::zero(), Rational::one()),
            (ln.clone(), Rational::fraction(10, 7).unwrap()),
            (ln.clone().negate(), Rational::fraction(7, 10).unwrap()),
            (ln.clone().add(ln.clone()), Rational::fraction(100, 49).unwrap()),
            (Computable::ln2().shift_left(2), Rational::from(16)),
            (Computable::ln_constant(3).unwrap().multiply(q(-3)), Rational::fraction(1, 27).unwrap()),
            (q(2).multiply(Computable::ln_constant(5).unwrap()), Rational::from(25)),
            (raw(Approximation::BinaryScaledLnRational { residual: Rational::fraction(1, 4).unwrap(), shift: 3 }), Rational::from(10)),
            (raw(Approximation::PrescaledLn(q(1))), Rational::from(2)),
        ];
        for (value, expected) in cases {
            let recovered = value.retained_log_argument(&mut 64).unwrap();
            assert_eq!(recovered.bounded_ring_rational().or_else(|| recovered.bounded_quotient_rational()), Some(expected));
            assert!(value.retained_log_argument(&mut 0).is_none());
        }
        for base in [2, 3, 5, 6, 7, 10] {
            assert_eq!(Computable::ln_constant(base).unwrap().retained_log_argument(&mut 64).unwrap().exact_rational(), Some(Rational::from(base)));
        }
        let root = Computable::ln2().shift_left(-1).retained_log_argument(&mut 64).unwrap();
        assert_eq!(root.square().exact_rational(), Some(Rational::from(2)));
        for value in [
            raw(Approximation::PrescaledLnRational(Rational::from(-2))),
            raw(Approximation::PrescaledLn(q(-2))),
            Computable::pi(), ln.clone().shift_left(5),
            ln.clone().multiply(q(17)), ln.clone().multiply(Computable::pi()),
            raw(Approximation::Multiply(Computable::pi(), q(2))),
        ] { assert!(value.retained_log_argument(&mut 64).is_none()); }
        let residual = Computable::rational(Rational::fraction(-1, 3).unwrap()).sin();
        let value = raw(Approximation::PrescaledLn(residual.clone()));
        assert!(value.retained_log_argument(&mut 64).is_none());
        residual.approx(-32);
        let cached = residual.cached();
        let recovered = value.retained_log_argument(&mut 64).unwrap();
        assert_eq!(recovered.immediate_sign(), Some(RealSign::Positive),
            "logarithm replay must carry its proven positive domain into an inverse");
        assert_eq!(cached, residual.cached(), "recognition never requests digits");
        let signal = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let mut aborted = ln;
        aborted.abort(signal);
        assert!(aborted.retained_log_argument(&mut 64).is_none());
    }

    #[test]
    fn jacobi_recognition_declines_unmatched_or_excessive_structure() {
        let angle = Computable::pi().inverse();
        let log = Computable::ln2();
        let sn = angle.clone().jacobi_component(log.clone(), 0);
        let cn = angle.clone().jacobi_component(log.clone(), 1);
        let dn = angle.jacobi_component(log, 2);
        assert!(sn.square().jacobi_square_term(0).is_none());
        assert!(cn.clone().square().jacobi_square_sum(&cn.clone().square()).is_none());
        let mut nested = cn.square();
        for _ in 0..9 { nested = raw(Approximation::Offset(nested, 1)); }
        assert!(nested.jacobi_square_term(8).is_none());
        assert!(Computable::pi().square().jacobi_square_sum(&dn.square()).is_none());
        let mut exploding = Computable::ln2();
        for _ in 0..8 { exploding = raw(Approximation::Offset(exploding, 4)); }
        assert!(exploding.retained_log_argument(&mut 64).is_none());
    }
}
