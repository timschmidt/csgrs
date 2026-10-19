impl Real {
    /// Complete elliptic integral of the first kind, `K(m) = F(pi/2|m)`.
    ///
    /// The argument is the **parameter** (squared modulus), and must satisfy
    /// `m < 1`. Negative parameters are supported. Domain errors return
    /// [`Problem::NotANumber`]; an unresolved domain proof returns
    /// [`Problem::Exhausted`]. The result is an exact, refinable native value.
    pub fn elliptic_k(parameter: &Self) -> Result<Self, Problem> {
        Self::elliptic_complete(parameter, false)
    }

    /// Complete elliptic integral of the second kind, `E(pi/2|m)`.
    ///
    /// Uses the same parameter domain `m < 1` as [`Self::elliptic_k`]. The
    /// finite endpoint `m = 1` is outside this API's domain. A positive Carlson
    /// combination avoids cancellation near that endpoint.
    pub fn elliptic_e_complete(parameter: &Self) -> Result<Self, Problem> {
        Self::elliptic_complete(parameter, true)
    }

    /// Incomplete elliptic integral `F(self|m)`, with amplitude in radians.
    ///
    /// Represents the integral from zero to `self` of
    /// `1/sqrt(cos(t)^2 + (1-m)*sin(t)^2)`. Every real amplitude is admitted;
    /// it is **not wrapped**. The parameter domain and errors are those of
    /// [`Self::elliptic_k`]. Period reduction is certified during refinement,
    /// without a floor decision at seams or a floating-point intermediate.
    /// Existing scalar precision and cancellation contracts apply; this adds
    /// no total-work, heap-byte or wall-time bound.
    pub fn elliptic_f(self, parameter: &Self) -> Result<Self, Problem> {
        self.elliptic_incomplete(parameter, false)
    }

    /// Incomplete elliptic integral `E(self|m)`, with amplitude in radians.
    ///
    /// Represents the integral from zero to `self` of
    /// `sqrt(cos(t)^2 + (1-m)*sin(t)^2)`. The unwrapped amplitude, parameter
    /// domain, error and resource contracts of [`Self::elliptic_f`] apply.
    pub fn elliptic_e(self, parameter: &Self) -> Result<Self, Problem> {
        self.elliptic_incomplete(parameter, true)
    }

    fn elliptic_complete(parameter: &Self, second: bool) -> Result<Self, Problem> {
        let log_complement = (Self::one() - parameter).ln()?;
        if parameter.definitely_zero() {
            return Self::pi() / Self::from(2);
        }
        Ok(Self::irrational_from_computable(
            Computable::elliptic_legendre_complete(log_complement.fold(), second),
        ))
    }

    fn elliptic_incomplete(self, parameter: &Self, second: bool) -> Result<Self, Problem> {
        // Domain validation precedes both identity shortcuts. The logarithm
        // makes the serialized parameter intrinsically valid.
        let log_complement = (Self::one() - parameter).ln()?;
        if self.definitely_zero() || parameter.definitely_zero() {
            return Ok(self);
        }
        Ok(Self::irrational_from_computable(
            self.fold().elliptic_legendre(log_complement.fold(), second),
        ))
    }
}
