/// Retain unwrapped Legendre definitions and exact inverse/quarter-period relations.
impl Computable {
    /// Construct unwrapped F or E from this amplitude and a retained log(1-m).
    /// Finite valid children give a positive complement c=exp(log_complement).
    /// Select E when second is true. First cancel a matching inverse node when its
    /// log-complement child is structurally equal, then recognize integer multiples
    /// of pi/2 and retain the corresponding multiple of the complete integral.
    /// These are structural/construction witnesses, not arbitrary scalar equality
    /// or numerical snapping of nearby amplitudes.
    ///
    /// Otherwise box the original amplitude and log complement, carrying any known
    /// amplitude sign into the integral's sign fact and leaving its magnitude bound
    /// unknown. Prefer the amplitude signal to the complement signal; shortcut
    /// results retain that selected signal ahead of any result attachment. This
    /// builder does not validate arbitrary child graphs or impose a work budget.
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

    /// Construct complete K or E from log(c), where c=1-m must be positive through valid finite data.
    /// K is RF(0,c,1). Complete E is the positive combination
    /// `c*(RD(0,c,1)+RD(0,1,c))/3`, avoiding cancellation between first-kind terms.
    /// Pass Carlson's encoded root/log payloads, not raw integral arguments, and
    /// retain the log source for both permutations and c's exponential. Child signal
    /// propagation and numerical/resource limits belong to the shared constructors
    /// and evaluators; no independent complete-integral approximation is stored.
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
