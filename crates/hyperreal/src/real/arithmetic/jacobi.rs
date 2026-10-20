impl Real {
    /// Jacobi's epsilon `epsilon(u|m) = E(am(u|m)|m)` for real `u` and `m < 1`.
    ///
    /// `self` is a Jacobi argument, not the amplitude accepted by
    /// [`Self::elliptic_e`]. The continuous unwrapped composition preserves
    /// real quasi-periods and its defining inverse relation. Negative
    /// parameters are supported, and `epsilon(u|0)=u`.
    ///
    /// Parameter validation, errors, refinement and cancellation follow
    /// [`Self::jacobi_amplitude`]. The exact native composition introduces
    /// no new serialized node, total-work limit or retained-heap bound.
    pub fn jacobi_epsilon(self, parameter: &Self) -> Result<Self, Problem> {
        self.jacobi_amplitude(parameter)?.elliptic_e(parameter)
    }

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
            self.fold()
                .inverse_elliptic_legendre(log_complement.clone(), false)
        };
        Self::jacobi_components(amplitude, complement, log_complement)
    }

    /// Jacobi's `(sn(u|m), cn(u|m), dn(u|m), epsilon(u|m))`, sharing
    /// the same continuous amplitude and parameter expressions.
    ///
    /// The input, failure, refinement and cancellation contracts are those
    /// of [`Self::jacobi_sncndn`] and [`Self::jacobi_epsilon`]. Each component
    /// has the same exact definition as a separate call. Sharing avoids
    /// constructing and refining a second inverse for epsilon; it does not
    /// add a total-work or retained-heap bound.
    pub fn jacobi_sncndn_epsilon(
        self,
        parameter: &Self,
    ) -> Result<(Self, Self, Self, Self), Problem> {
        let complement = Self::one() - parameter;
        let log_complement = complement.clone().ln()?.fold();
        if self.definitely_zero() {
            return Ok((Self::zero(), Self::one(), Self::one(), self));
        }
        let zero_parameter = parameter.definitely_zero();
        let amplitude = if zero_parameter {
            self.clone().fold()
        } else {
            self.clone().fold()
                .inverse_elliptic_legendre(log_complement.clone(), false)
        };
        let epsilon = if zero_parameter {
            self
        } else {
            Self::elliptic_result(
                amplitude
                    .clone()
                    .elliptic_legendre(log_complement.clone(), true),
            )
        };
        let (sn, cn, dn) = Self::jacobi_components(amplitude, complement, log_complement)?;
        Ok((sn, cn, dn, epsilon))
    }

    fn jacobi_components(
        amplitude: Computable,
        complement: Self,
        log_complement: Computable,
    ) -> Result<(Self, Self, Self), Problem> {
        if amplitude
            .pi_rational_multiple()
            .is_some_and(|scale| (scale * Rational::from(2)).is_integer())
        {
            let angle = Self::elliptic_result(amplitude);
            let sn = angle.clone().sin();
            let cn = angle.cos();
            let dn = if sn.definitely_zero() {
                Self::one()
            } else {
                complement.sqrt()?
            };
            return Ok((sn, cn, dn));
        }
        let [sn, cn, dn] = std::array::from_fn(|kind| {
            Self::elliptic_result(
                amplitude
                    .clone()
                    .jacobi_component(log_complement.clone(), kind),
            )
        });
        Ok((sn, cn, dn))
    }
}
