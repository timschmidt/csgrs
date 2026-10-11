impl Computable {
    /// Reduce a bounded linear pi form without approximating enormous binary
    /// scales. Successful extraction proves every discarded operand is real.
    /// Unsupported syntax or coefficient growth declines this optional rewrite.
    pub(super) fn try_symbolic_sin_cos(
        &self,
        signal: &Option<Signal>,
    ) -> Result<Option<(Self, Self)>, Problem> {
        // The existing rational reducers already handle these leaves directly.
        if matches!(
            self.internal.approximation,
            Approximation::One | Approximation::Int(_) | Approximation::Ratio(_)
        ) {
            return Ok(None);
        }
        if evaluation_should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        let Some((quadrant, coefficient, residual)) =
            trig_linear_form(self, 128, &mut 256).and_then(PiLinearForm::reduced_angle)
        else {
            return Ok(None);
        };

        let (sine, cosine) = if coefficient.is_zero() {
            (Self::zero(), Self::one())
        } else {
            // Use one canonical small argument shape. Recognizing this fixed
            // point prevents repeated rewriting inside the prescaled kernels,
            // even when the source contains cancelling offsets or a large r.
            let angle = Self {
                internal: Arc::new(Node::new(
                    Approximation::Multiply(Self::pi(), Self::rational(coefficient)),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
                signal: signal.clone(),
            };
            if quadrant == 0 && residual.is_zero() && Self::internal_structural_eq(self, &angle) {
                return Ok(None);
            }
            (
                Self::prescaled_sin(angle.clone()),
                Self::prescaled_cos(angle),
            )
        };

        let (sine, cosine) = if residual.is_zero() {
            (sine, cosine)
        } else {
            // Split the rational part through exact angle addition. Merely
            // reducing q modulo two in q*pi+r could undo a later numerical
            // reduction of a large r and cause the reducers to cycle.
            // Use dedicated rational nodes, including for medium residuals.
            // Their numeric kernels never reconstruct a generic pi+r trig
            // argument that could enter this rewrite again. For |r|<1 the
            // small constructors' retained sign certificates are valid.
            let (sin_r, cos_r) = if residual.magnitude_at_least_power_of_two(0) {
                (
                    Self::sin_large_rational_deferred(residual.clone()),
                    Self::cos_large_rational_deferred(residual),
                )
            } else {
                (
                    Self::prescaled_sin_rational(residual.clone()),
                    Self::prescaled_cos_rational(residual),
                )
            };
            (
                sine.clone()
                    .multiply(cos_r.clone())
                    .add(cosine.clone().multiply(sin_r.clone())),
                cosine.multiply(cos_r).add(sine.multiply(sin_r).negate()),
            )
        };
        let (mut sine, mut cosine) = match quadrant {
            0 => (sine, cosine),
            1 => (cosine, sine.negate()),
            2 => (sine.negate(), cosine.negate()),
            3 => (cosine.negate(), sine),
            _ => unreachable!("exact quadrant is reduced modulo four"),
        };
        if evaluation_should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        sine.signal = signal.clone();
        cosine.signal = signal.clone();
        Ok(Some((sine, cosine)))
    }
}
