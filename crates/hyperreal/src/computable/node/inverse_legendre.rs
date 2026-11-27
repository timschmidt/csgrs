/// Retain unwrapped inverse Legendre equations and recognized exact forward/quarter-period relations.
impl Computable {
    /// Construct the inverse of F, or E when second is true, from a value and log(1-m).
    /// Require valid finite children, so the complement is positive. First cancel a
    /// matching forward kind with a structurally equal parameter child. Otherwise
    /// construct the corresponding positive complete integral Q and peel limited
    /// rational scaling from both expressions. An integer scale ratio with a shared
    /// structural base returns that multiple of pi/2; no numerical closeness or
    /// general scalar-equality decision establishes this identity.
    ///
    /// A missed recognition retains the original pair in a boxed inverse node,
    /// carrying any known input sign and leaving its magnitude bound unknown.
    /// Prefer the input signal to the parameter signal; exact shortcut results
    /// retain that selection. The local depth limit is not a total-work or allocation
    /// budget, and this builder performs no arbitrary child-domain validation.
    pub(crate) fn inverse_elliptic_legendre(self, log_complement: Self, second: bool) -> Self {
        let signal = self
            .signal
            .clone()
            .or_else(|| log_complement.signal.clone());
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
        /// Peel rational scaling from one structural chain, returning an exact factor and borrowed residual base.
        /// Stop with None before inspecting a node when budget is zero. Descend through
        /// Negate, binary Offset and Multiply with a recognized rational factor; try the
        /// left factor before the right. A product with neither factor recognized, or
        /// any other node, is retained as the base with factor one. This is not recursive
        /// normalization of an arbitrary product tree.
        ///
        /// The caller starts with 32 visits for each chain. That count bounds traversal
        /// depth only: rational recognition, multiplication and power-of-two allocation
        /// have their own costs. The shift helper does not support negating i32::MIN.
        /// There is no cancellation poll or typed resource outcome; None declines the
        /// shortcut and leaves the inverse construction available.
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
