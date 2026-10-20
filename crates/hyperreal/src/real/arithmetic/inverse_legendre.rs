impl Real {
    /// Jacobi's amplitude: the unique real `phi` for which `F(phi|m) = self`.
    ///
    /// This is the continuous, unwrapped inverse of [`Self::elliptic_f`].
    /// `parameter` is the squared modulus, with domain `m < 1`, including
    /// negative values. Invalid parameters return [`Problem::NotANumber`];
    /// an unresolved domain proof returns [`Problem::Exhausted`].
    /// The result retains its defining equation and can be refined without
    /// replacing it with a finite root estimate. Existing scalar precision
    /// and cancellation contracts apply; no work or heap-byte bound is added.
    pub fn jacobi_amplitude(self, parameter: &Self) -> Result<Self, Problem> {
        self.inverse_elliptic_legendre(parameter, false)
    }

    /// Unique real `phi` for which the unwrapped integral `E(phi|m) = self`.
    ///
    /// Inverts [`Self::elliptic_e`], with the parameter, failure, refinement
    /// and resource contracts of [`Self::jacobi_amplitude`].
    pub fn inverse_elliptic_e(self, parameter: &Self) -> Result<Self, Problem> {
        self.inverse_elliptic_legendre(parameter, true)
    }

    fn inverse_elliptic_legendre(self, parameter: &Self, second: bool) -> Result<Self, Problem> {
        let log_complement = (Self::one() - parameter).ln()?.fold();
        if self.definitely_zero() || parameter.definitely_zero() {
            return Ok(self);
        }
        Ok(Self::elliptic_result(
            self.fold().inverse_elliptic_legendre(log_complement, second),
        ))
    }

    // Recover exact public carriers after a defining forward/inverse relation
    // cancels. Other preimages retain their exact Computable expression.
    fn elliptic_result(value: Computable) -> Self {
        if let Some(scale) = value.pi_rational_multiple() {
            return Self::pi().scaled_by_rational(&scale);
        }
        Self::irrational_from_computable(value).make_computable(|value| value)
    }
}
