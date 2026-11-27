/// Admit real inverse-Legendre domains and recover recognized exact public result carriers.
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

    /// Share parameter admission and exact zero/circular shortcuts for both inverse kinds.
    /// Validate and fold log(1-m) before returning the input for a known zero value
    /// or m=0. Otherwise retain the folded value and parameter in the native inverse
    /// and pass it through elliptic_result. Domain errors stay typed; no wrapping,
    /// finite root estimate or extra work/precision budget is introduced.
    fn inverse_elliptic_legendre(self, parameter: &Self, second: bool) -> Result<Self, Problem> {
        let log_complement = (Self::one() - parameter).ln()?.fold();
        if self.definitely_zero() || parameter.definitely_zero() {
            return Ok(self);
        }
        Ok(Self::elliptic_result(
            self.fold()
                .inverse_elliptic_legendre(log_complement, second),
        ))
    }

    /// Recover recognized exact public carriers after a forward/inverse or endpoint relation.
    /// Prefer a recognized rational multiple of pi. Otherwise pass the same exact
    /// Computable through the shared identity conversion, which can recover rational
    /// or admitted pure-quadratic carriers and retains other expressions as opaque
    /// Computable-backed values. This is limited structural reconstruction, not a
    /// canonical-form or general scalar-equality decision; no rounding is introduced.
    /// The pi branch constructs a fresh public carrier and does not transport the
    /// supplied Computable's abort signal. Inverting a flagged complete integral
    /// to pi/2 exposes this existing attachment loss; exact value recovery alone
    /// does not establish cancellation propagation through the public adapter.
    fn elliptic_result(value: Computable) -> Self {
        if let Some(scale) = value.pi_rational_multiple() {
            return Self::pi().scaled_by_rational(&scale);
        }
        Self::irrational_from_computable(value).make_computable(|value| value)
    }
}
