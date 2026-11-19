use crate::IntervalError;
use crate::bounds::Bounds;
use crate::ieee::{BINARY32, BINARY64};
use core::cmp::Ordering;

macro_rules! proposal {
    ($name:ident, $float:ty, $bits:ty, $format:ident) => {
        /// Finite primitive endpoint estimates for non-authoritative proposals.
        ///
        /// Arithmetic uses the native float environment and is not directed
        /// outward. These estimates cannot prove containment, exclusion, sign,
        /// or topology. Certify a proposed operation by replaying its exact
        /// source through an authoritative backend. No proposal-to-certificate
        /// conversion exists.
        ///
        /// ```compile_fail
        /// use hyperinterval::F32ProposalInterval;
        /// let p = F32ProposalInterval::point(1.0).unwrap();
        /// p.contains_point(1.0); // A proposal cannot certify membership.
        /// ```
        ///
        /// ```compile_fail
        /// use hyperinterval::F64ProposalInterval;
        /// let p = F64ProposalInterval::point(1.0).unwrap();
        /// p.signs(); // A proposal cannot supply authoritative sign sets.
        /// ```
        #[cfg_attr(
            feature = "rational",
            doc = "
```compile_fail
use hyperinterval::{F64ProposalInterval, RationalInterval};
let proposal = F64ProposalInterval::point(1.0).unwrap();
let certified: RationalInterval = proposal.into();
```
"
        )]
        #[derive(Clone, Copy, Debug)]
        pub struct $name {
            estimates: Option<Bounds<$float>>,
        }

        // Equality compares canonical stored bits, not native floating-point
        // predicates that may be affected by subnormal environment settings.
        impl PartialEq for $name {
            fn eq(&self, other: &Self) -> bool {
                let bits =
                    |value: &Self| value.estimates().map(|(a, b)| (a.to_bits(), b.to_bits()));
                bits(self) == bits(other)
            }
        }

        impl Eq for $name {}

        impl $name {
            /// Validates finite ordered estimates. Signed zeros become +0.
            #[inline]
            pub fn new(lower: $float, upper: $float) -> Result<Self, IntervalError> {
                Self::from_bits(lower.to_bits(), upper.to_bits())
            }

            // An integer call boundary keeps zero canonicalization independent
            // of DAZ while allowing the validator to specialize for this width.
            #[inline(never)]
            fn from_bits(lower: $bits, upper: $bits) -> Result<Self, IntervalError> {
                let (a, b) = $format.ordered_finite_pair(u64::from(lower), u64::from(upper))?;
                Ok(Self::from_ordered_bits(a as $bits, b as $bits))
            }

            fn from_ordered_bits(lower: $bits, upper: $bits) -> Self {
                Self {
                    estimates: Some(Bounds::ordered(
                        <$float>::from_bits(lower),
                        <$float>::from_bits(upper),
                    )),
                }
            }

            /// A single finite proposed value, without an exact-source claim.
            pub fn point(value: $float) -> Result<Self, IntervalError> {
                Self::new(value, value)
            }

            /// An explicit empty proposal, distinct from an invalid estimate.
            pub const fn empty() -> Self {
                Self { estimates: None }
            }

            /// Whether no estimate is present; this is not a proof of an empty source.
            pub fn is_empty(&self) -> bool {
                self.estimates.is_none()
            }

            /// Endpoint estimates, or None for empty. No enclosure guarantee.
            pub fn estimates(&self) -> Option<($float, $float)> {
                self.estimates.map(Bounds::endpoints)
            }

            /// Native midpoint estimate, or an absent value for empty input. Under rounding to nearest
            /// with gradual underflow, even a subnormal midpoint rounds once.
            pub fn midpoint_estimate(&self) -> Option<$float> {
                self.estimates().map(|(a, b)| a.midpoint(b))
            }

            // Validate every arithmetic result before selecting extrema. Bit
            // keys avoid treating subnormals as zero under a DAZ environment.
            fn collect(values: &[$float]) -> Result<Self, IntervalError> {
                let mut range: Option<(u64, u64)> = None;
                for value in values {
                    let bits = $format
                        .finite(u64::from(value.to_bits()))
                        .map_err(|_| IntervalError::Overflow)?;
                    range = Some(match range {
                        None => (bits, bits),
                        Some((a, b)) => (
                            if $format.compare(bits, a) == Ordering::Less {
                                bits
                            } else {
                                a
                            },
                            if $format.compare(bits, b) == Ordering::Greater {
                                bits
                            } else {
                                b
                            },
                        ),
                    });
                }
                match range {
                    None => Ok(Self::empty()),
                    // Every candidate passed finite validation before selection.
                    Some((a, b)) => Ok(Self::from_ordered_bits(a as $bits, b as $bits)),
                }
            }

            /// Native sum estimates, with no outward-rounding claim.
            pub fn add(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.estimates(), other.estimates()) {
                    (Some((a, b)), Some((c, d))) => Self::collect(&[a + c, b + d]),
                    _ => Ok(Self::empty()),
                }
            }

            /// Native difference estimates, with no outward-rounding claim.
            pub fn sub(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.estimates(), other.estimates()) {
                    (Some((a, b)), Some((c, d))) => Self::collect(&[a - d, b - c]),
                    _ => Ok(Self::empty()),
                }
            }

            /// Negated endpoint estimates, remaining non-authoritative.
            pub fn neg(&self) -> Self {
                match self.estimates() {
                    None => Self::empty(),
                    Some((a, b)) => Self::new(-b, -a).expect("finite ordered negation"),
                }
            }

            /// Native four-corner product estimates.
            pub fn mul(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.estimates(), other.estimates()) {
                    (Some((a, b)), Some((c, d))) => Self::collect(&[a * c, a * d, b * c, b * d]),
                    _ => Ok(Self::empty()),
                }
            }

            /// Native quotient estimates. Rejects a zero-spanning denominator
            /// even when the numerator is empty; this is not certified division.
            pub fn div(&self, other: &Self) -> Result<Self, IntervalError> {
                if let Some((a, b)) = other.estimates() {
                    if $format.compare(u64::from(a.to_bits()), 0) != Ordering::Greater
                        && $format.compare(u64::from(b.to_bits()), 0) != Ordering::Less
                    {
                        return Err(IntervalError::ContainsZero);
                    }
                }
                match (self.estimates(), other.estimates()) {
                    (Some((a, b)), Some((c, d))) => Self::collect(&[a / c, a / d, b / c, b / d]),
                    _ => Ok(Self::empty()),
                }
            }

            /// Native estimates of `scale * x + offset`. Scalar inputs are
            /// validated even when this proposal is empty.
            pub fn affine(&self, scale: $float, offset: $float) -> Result<Self, IntervalError> {
                let scale = Self::point(scale)?;
                let offset = Self::point(offset)?;
                self.mul(&scale)?.add(&offset)
            }
        }

        #[cfg(feature = "serde")]
        impl serde::Serialize for $name {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.estimates()
                    .map(|(a, b)| (a.to_bits(), b.to_bits()))
                    .serialize(serializer)
            }
        }

        #[cfg(feature = "serde")]
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                match Option::<($bits, $bits)>::deserialize(deserializer)? {
                    None => Ok(Self::empty()),
                    Some((a, b)) => Self::new(<$float>::from_bits(a), <$float>::from_bits(b)),
                }
                .map_err(serde::de::Error::custom)
            }
        }
    };
}

proposal!(F32ProposalInterval, f32, u32, BINARY32);
proposal!(F64ProposalInterval, f64, u64, BINARY64);

impl F32ProposalInterval {
    /// Native estimate of the half-width, or `None` for empty.
    /// Taking the midpoint of the upper and negated lower endpoints keeps the
    /// full finite range representable. With gradual underflow and rounding
    /// to nearest, a subnormal half-width rounds once, with ties to even.
    #[inline]
    pub fn radius_estimate(&self) -> Option<f32> {
        self.estimates().map(|(lo, hi)| (-lo).midpoint(hi))
    }
}
