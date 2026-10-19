impl Real {
    /// Jacobi's `(sn(u|m), cn(u|m), dn(u|m))`, sharing the exact amplitude.
    ///
    /// `self` is any real `u`; `parameter` is the squared modulus `m < 1`,
    /// including negative values. The amplitude is continuous and unwrapped;
    /// `dn` takes the positive branch. Matching outputs retain their unit
    /// and elliptic square-sum relations for subsequent scalar operations.
    /// Invalid parameters return [`Problem::NotANumber`], and an unresolved
    /// domain proof returns [`Problem::Exhausted`]. Existing scalar precision
    /// and cancellation contracts apply; no work or heap-byte bound is added.
    pub fn jacobi_sncndn(self, parameter: &Self) -> Result<(Self, Self, Self), Problem> {
        let complement = Self::one() - parameter;
        let log_complement = complement.clone().ln()?.fold();
        if self.definitely_zero() {
            return Ok((Self::zero(), Self::one(), Self::one()));
        }
        let amplitude = if parameter.definitely_zero() {
            self.fold()
        } else {
            self.fold().inverse_elliptic_legendre(log_complement.clone(), false)
        };
        if amplitude.pi_rational_multiple().is_some_and(|scale| (scale * Rational::from(2)).is_integer()) {
            let angle = Self::elliptic_result(amplitude);
            let sn = angle.clone().sin();
            let cn = angle.cos();
            let dn = if sn.definitely_zero() { Self::one() } else { complement.sqrt()? };
            return Ok((sn, cn, dn));
        }
        let [sn, cn, dn] = std::array::from_fn(|kind| {
            Self::elliptic_result(amplitude.clone().jacobi_component(log_complement.clone(), kind))
        });
        Ok((sn, cn, dn))
    }
}
