impl Computable {
    pub(crate) fn elliptic_legendre(self, log_complement: Self, second: bool) -> Self {
        let signal = self
            .signal
            .clone()
            .or_else(|| log_complement.signal.clone());
        let matched = match &self.internal.approximation {
            Approximation::JacobiAmplitude(arguments) if !second => Some(arguments),
            Approximation::InverseEllipticE(arguments) if second => Some(arguments),
            _ => None,
        };
        if let Some(arguments) = matched
            && Self::internal_structural_eq(&arguments[1], &log_complement)
        {
            let mut result = arguments[0].clone();
            result.signal = signal.or(result.signal);
            return result;
        }
        // The even, pi-periodic integrand contributes the same complete
        // integral over every signed quarter. Preserve this exact endpoint
        // relation for epsilon and later consumers, rather than requiring
        // numerical subtraction of equivalent complete/incomplete formulas.
        if let Some(multiple) = self
            .pi_rational_multiple()
            .map(|scale| scale * Rational::from(2))
            && multiple.is_integer()
        {
            let mut result = Self::elliptic_legendre_complete(log_complement, second)
                .multiply_rational(multiple);
            result.signal = signal.or(result.signal);
            return result;
        }
        let sign = self.exact_sign();
        let payload = Box::new([self, log_complement]);
        let approximation = if second {
            Approximation::EllipticE(payload)
        } else {
            Approximation::EllipticF(payload)
        };
        Self {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal,
        }
    }

    pub(crate) fn elliptic_legendre_complete(log_complement: Self, second: bool) -> Self {
        let first =
            Self::elliptic_carlson([Self::zero(), log_complement.clone(), Self::zero()], second);
        if !second {
            return first;
        }
        let other =
            Self::elliptic_carlson([Self::zero(), Self::zero(), log_complement.clone()], true);
        first
            .add(other)
            .multiply(log_complement.exp())
            .multiply(Self::rational(
                Rational::fraction(1, 3).expect("nonzero denominator"),
            ))
    }
}
