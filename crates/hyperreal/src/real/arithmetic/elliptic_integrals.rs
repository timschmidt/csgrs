impl Real {
    /// Carlson's symmetric elliptic integral of the first kind, `RF(x,y,z)`.
    ///
    /// Represents `integral(0..infinity, 1/sqrt((t+x)(t+y)(t+z))) / 2`.
    /// Arguments must be nonnegative, with at most one zero. Invalid or
    /// divergent inputs return [`Problem::NotANumber`]; an unresolved domain
    /// proof returns [`Problem::Exhausted`]. Evaluation retains the defining
    /// integral and refines certified enclosures, without a finite series model
    /// or floating-point intermediate. Existing scalar precision and abort
    /// contracts apply; this does not add a total-work or heap-byte budget.
    pub fn elliptic_rf(x: &Self, y: &Self, z: &Self) -> Result<Self, Problem> {
        Self::elliptic_carlson(x, y, z, false)
    }

    /// Carlson's elliptic integral `RD(x,y,z)`, symmetric in `x` and `y`.
    ///
    /// Represents `3/2 * integral(0..infinity,
    /// 1/((t+z)*sqrt((t+x)(t+y)(t+z))))`.
    /// Here `x,y >= 0`, at most one of `x,y` is zero, and `z > 0`.
    /// The exact-value, error and resource contracts of [`Self::elliptic_rf`]
    /// also apply. This function is not symmetric in its third argument.
    pub fn elliptic_rd(x: &Self, y: &Self, z: &Self) -> Result<Self, Problem> {
        Self::elliptic_carlson(x, y, z, true)
    }

    fn elliptic_carlson(x: &Self, y: &Self, z: &Self, second: bool) -> Result<Self, Problem> {
        let mut values = [x, y, z];
        let signs = [x.operation_sign()?, y.operation_sign()?, z.operation_sign()?];
        if signs.contains(&Sign::Minus)
            || signs.iter().filter(|&&s| s == Sign::NoSign).count() > 1
            || (second && signs[2] == Sign::NoSign)
        {
            return Err(Problem::NotANumber);
        }
        if x == y && y == z {
            crate::trace_dispatch!("real", "elliptic_carlson", "exact-diagonal");
            let inverse_root = x.clone().sqrt()?.inverse()?;
            return if second { inverse_root / x } else { Ok(inverse_root) };
        }
        if let Some(index) = signs.iter().position(|&s| s == Sign::NoSign) {
            values.swap(0, index);
        }
        // Encode the domain in the defining payload itself: square(root_x),
        // exp(log_y), exp(log_z). Every finite deserialized payload is valid.
        // The optional zero has been permuted to x; RD never swaps z.
        let arguments = [
            values[0].clone().sqrt()?.fold(),
            values[1].clone().ln()?.fold(),
            values[2].clone().ln()?.fold(),
        ];
        crate::trace_dispatch!("real", "elliptic_carlson", "certified-duplication");
        Ok(Self::irrational_from_computable(Computable::elliptic_carlson(arguments, second)))
    }
}
