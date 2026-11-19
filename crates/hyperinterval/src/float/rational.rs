use super::{BINARY32, BINARY64, F32Enclosure, F64Enclosure, IntervalError, qualify};

macro_rules! projection {
    ($name:ident, $format:ident) => {
        impl $name {
            /// Tightest finite IEEE enclosure of a closed exact rational interval.
            ///
            /// Each endpoint rounds outward on its own binary grid. A fraction
            /// such as 1/10 lies between adjacent IEEE values. An endpoint closer
            /// to zero than the least subnormal rounds outward to zero or that subnormal.
            /// Returns [`IntervalError::Overflow`] if a finite endpoint cannot
            /// enclose the source. Empty succeeds without work on every target.
            ///
            /// Requires the `rational` and `dyadic` features. Nonempty inputs
            /// first require a qualified environment. Both endpoints share the
            /// caller's budget. Each spends one unit on magnitude inspection.
            /// Directed division and IEEE projection charge their normal stages
            /// when the stored bit lengths alone do not settle the bound.
            #[inline]
            pub fn from_rational(
                source: &crate::RationalInterval,
                budget: &mut crate::DyadicBudget,
            ) -> Result<Self, IntervalError> {
                let Some((lo, hi)) = source.bounds() else {
                    return Ok(Self::empty());
                };
                qualify()?;
                Self::from_bits(
                    $format.rational_bound(lo, false, budget)?,
                    $format.rational_bound(hi, true, budget)?,
                )
            }
        }
    };
}

projection!(F32Enclosure, BINARY32);
projection!(F64Enclosure, BINARY64);
