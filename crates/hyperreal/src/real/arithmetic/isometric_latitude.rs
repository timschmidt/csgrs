impl Real {
    /// Recover geodetic latitude from ellipsoidal isometric latitude.
    ///
    /// `axis_ratio` is the positive polar/equatorial axis ratio `b/a`.
    /// Spherical, oblate and prolate ellipsoids are supported. The unique
    /// result lies strictly between `-pi/2` and `pi/2`, in radians. It retains
    /// its defining equation and refines with certified enclosures; no finite
    /// iteration result or floating-point seed becomes the represented value.
    ///
    /// For eccentricity `e = sqrt(1-(b/a)^2)` the defining equation is
    /// `self = asinh(tan(phi)) - e*atanh(e*sin(phi))`. The prolate continuation
    /// uses `+c*atan(c*sin(phi))`, `c = sqrt((b/a)^2-1)`.
    /// Nonpositive ratios and unresolved positivity return the same typed
    /// problems as [`Self::ln`]. Existing scalar precision, memory and abort
    /// limitations apply; this method adds no total-work or heap-byte budget.
    pub fn inverse_isometric_latitude(self, axis_ratio: &Self) -> Result<Self, Problem> {
        // Validate before the identity shortcut. Storing ln(b/a) also makes
        // positivity intrinsic to the serialized defining equation.
        let logarithm = axis_ratio.clone().ln()?;
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "inverse_isometric_latitude", "exact-equator");
            return Ok(Self::zero());
        }
        crate::trace_dispatch!(
            "real",
            "inverse_isometric_latitude",
            "certified-monotone-inverse"
        );
        Ok(Self::irrational_from_computable(
            self.fold().inverse_isometric_latitude(logarithm.fold()),
        ))
    }
}
