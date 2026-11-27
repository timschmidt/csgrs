/// Admit real Jacobi parameters and share exact amplitudes across components and epsilon.
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
    /// Exact carrier reconstruction can lose an attached abort signal. For an
    /// input constructed by [`Self::elliptic_k`] at m=3/4, amplitude recovery
    /// rebuilds pi/2 without that attachment. Equal exact outputs need not retain
    /// identical controls.
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
    /// Parameter validation precedes even a known-zero input shortcut. Zero and
    /// recognized quarter-period outputs use exact public carriers, which can be
    /// constructed without the input's abort attachment; general shared-node
    /// cancellation tests do not establish attachment preservation on those paths.
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
    /// The mathematical input and failure domains are those of
    /// [`Self::jacobi_sncndn`] and [`Self::jacobi_epsilon`]. Each component
    /// has the same exact definition as a separate call. Sharing avoids
    /// constructing and refining a second inverse for epsilon; it does not
    /// add a total-work or retained-heap bound. Parameter validation precedes
    /// zero shortcuts; epsilon keeps the input itself at u=0 or m=0. Scalar
    /// refinement/cancellation rules apply, but exact carrier recovery can drop
    /// component abort attachments. For a flagged [`Self::elliptic_k`] result at
    /// m=3/4, shared epsilon retains the input flag that separate epsilon
    /// evaluation loses during pi/2 recovery;
    /// equality of values is not equality of cancellation behavior.
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
            self.clone()
                .fold()
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

    /// Recover quarter-period carriers or construct three components sharing an admitted amplitude.
    /// Require complement=1-m>0 and log_complement=ln(complement) from the same
    /// validated parameter. A structurally recognized integer multiple of pi/2
    /// uses exact public sine/cosine; dn is one at even quarters and the positive
    /// square root of the complement at odd quarters. Square-root failures propagate.
    /// Otherwise construct sn/cn/dn tags in that order over shared child nodes and
    /// pass each through exact carrier recovery. No numerical angle fitting or
    /// new domain proof is performed. Endpoint reconstruction can drop abort
    /// attachments; sharing an exact value does not promise identical controls.
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
