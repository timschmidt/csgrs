impl Computable {
    pub(crate) fn inverse_elliptic_legendre(self, log_complement: Self, second: bool) -> Self {
        let signal = self.signal.clone().or_else(|| log_complement.signal.clone());
        let matched = match &self.internal.approximation {
            Approximation::EllipticF(arguments) if !second => Some(arguments),
            Approximation::EllipticE(arguments) if second => Some(arguments),
            _ => None,
        };
        if let Some(arguments) = matched
            && Self::internal_structural_eq(&arguments[1], &log_complement)
        {
            let mut result = arguments[0].clone();
            result.signal = signal.or(result.signal);
            return result;
        }
        // Rational scaling may already have been folded into the Computable
        // DAG. Recognize only a bounded, structural multiple of this Q.
        fn scaled(value: &Computable, budget: u8) -> Option<(Rational, &Computable)> {
            if budget == 0 {
                return None;
            }
            match &value.internal.approximation {
                Approximation::Negate(child) => {
                    let (scale, base) = scaled(child, budget - 1)?;
                    Some((-scale, base))
                }
                Approximation::Offset(child, shift) => {
                    let (scale, base) = scaled(child, budget - 1)?;
                    Some((scale * Computable::power_of_two_rational(*shift), base))
                }
                Approximation::Multiply(left, right) => {
                    let (factor, child) = if let Some(factor) = left.exact_rational() {
                        (factor, right)
                    } else if let Some(factor) = right.exact_rational() {
                        (factor, left)
                    } else {
                        return Some((Rational::one(), value));
                    };
                    let (scale, base) = scaled(child, budget - 1)?;
                    Some((factor * scale, base))
                }
                _ => Some((Rational::one(), value)),
            }
        }
        let complete = Self::elliptic_legendre_complete(log_complement.clone(), second);
        if let Some((scale, base)) = scaled(&self, 32)
            && let Some((complete_scale, complete_base)) = scaled(&complete, 32)
            && Self::internal_structural_eq(base, complete_base)
        {
            // Q is positive by construction, so its rational scale is nonzero.
            let multiple = scale / complete_scale;
            if multiple.is_integer() {
                let mut result = Self::pi().multiply_rational(multiple).shift_left(-1);
                result.signal = signal;
                return result;
            }
        }
        let sign = self.exact_sign();
        let payload = Box::new([self, log_complement]);
        let approximation = if second {
            Approximation::InverseEllipticE(payload)
        } else {
            Approximation::JacobiAmplitude(payload)
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
}
