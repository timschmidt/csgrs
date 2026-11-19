use crate::bounds::Bounds;
use crate::ieee::{BINARY32, BINARY64};
use crate::{DomainCoverage, IntervalError, RangeEnclosure, SignSet};
use core::cmp::Ordering;

// Cargo distinguishes the ordinary and threaded Preview 1 targets even on
// compilers whose stable target cfgs coincide for the two ABIs.
#[cfg(all(
    target_arch = "wasm32",
    target_os = "wasi",
    feature = "float-enclosure-wasip1"
))]
const BACKEND_SUPPORTED: bool = matches!(target_tuple::TARGET.as_bytes(), b"wasm32-wasip1");
#[cfg(not(all(
    target_arch = "wasm32",
    target_os = "wasi",
    feature = "float-enclosure-wasip1"
)))]
const BACKEND_SUPPORTED: bool = cfg!(any(
    target_arch = "x86_64",
    all(target_arch = "wasm32", target_os = "unknown")
));

fn qualify() -> Result<(), IntervalError> {
    if BACKEND_SUPPORTED {
        Ok(())
    } else {
        Err(IntervalError::UnqualifiedEnvironment)
    }
}

macro_rules! enclosure {
    ($name:ident, $endpoint:ident, $float:ty, $bits:ty, $format:ident, $width:ident, $limbs:literal) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        struct $endpoint($bits);

        impl PartialOrd for $endpoint {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }
        impl Ord for $endpoint {
            fn cmp(&self, other: &Self) -> Ordering {
                $format.compare(u64::from(self.0), u64::from(other.0))
            }
        }

        /// Closed real set with finite exact IEEE endpoints, or canonical empty.
        ///
        /// Basic arithmetic computes outward bounds with integer IEEE kernels,
        /// independent of ambient float rounding and FTZ/DAZ. No native float
        /// arithmetic or allocation occurs. Nonempty ingress is
        /// qualified on x86_64 and wasm32-unknown-unknown. The optional
        /// `float-enclosure-wasip1` feature also qualifies wasm32-wasip1. Other targets return
        /// `UnqualifiedEnvironment`. Empty is valid on every target.
        /// Overflow is an error, never saturation or an infinite endpoint.
        /// Both input zero signs represent the same canonical real zero.
        ///
        /// Enclosure signs describe members of a set, not an exact scalar's
        /// equality to zero. No proposal conversion or scalar ordering exists.
        #[cfg_attr(
            feature = "float-proposal",
            doc = "
```compile_fail
use hyperinterval::{F64Enclosure, F64ProposalInterval};
let proposal = F64ProposalInterval::point(1.0).unwrap();
let enclosure: F64Enclosure = proposal.into();
```
"
        )]
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        pub struct $name {
            bounds: Option<Bounds<$endpoint>>,
        }

        /// Exact nonnegative endpoint distance of a checked finite interval.
        ///
        /// Retains the defining endpoints, including widths above the largest
        /// finite float or between its grid points. `Ord`/`Eq` compare exact
        /// width values by bounded integer arithmetic, independent of ambient
        /// float state. Different definitions of equal widths can serialize
        /// differently; serialization preserves the definition, not a canonical
        /// mathematical-value encoding. No allocator or native float arithmetic.
        #[derive(Clone, Copy, Debug)]
        pub struct $width {
            bounds: Bounds<$endpoint>,
        }
        impl $width {
            /// Exact interval whose endpoint difference defines this width.
            pub fn defining_interval(&self) -> $name {
                $name {
                    bounds: Some(self.bounds),
                }
            }
            /// Explicit outward projection into the finite endpoint family.
            /// Overflow is typed; the retained exact width remains unchanged.
            pub fn enclosure(&self) -> Result<$name, IntervalError> {
                let (a, b) = self.bounds.endpoints();
                let (lo, hi) = $format.add(u64::from(b.0), $format.negate(u64::from(a.0)))?;
                $name::from_bits(lo, hi)
            }
        }
        impl Ord for $width {
            fn cmp(&self, other: &Self) -> Ordering {
                let (a, b) = self.bounds.endpoints();
                let (c, d) = other.bounds.endpoints();
                $format.compare_width::<$limbs>(
                    u64::from(a.0),
                    u64::from(b.0),
                    u64::from(c.0),
                    u64::from(d.0),
                )
            }
        }
        impl PartialOrd for $width {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }
        impl PartialEq for $width {
            fn eq(&self, other: &Self) -> bool {
                self.cmp(other) == Ordering::Equal
            }
        }
        impl Eq for $width {}
        #[cfg(feature = "serde")]
        impl serde::Serialize for $width {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.defining_interval().serialize(serializer)
            }
        }
        #[cfg(feature = "serde")]
        impl<'de> serde::Deserialize<'de> for $width {
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                $name::deserialize(deserializer)?
                    .width()
                    .ok_or_else(|| serde::de::Error::custom(IntervalError::EmptyDomain))
            }
        }

        impl $name {
            /// Whether the integer IEEE backend is qualified on this target.
            /// This does not query or change ambient rounding state.
            pub const fn backend_supported() -> bool {
                BACKEND_SUPPORTED
            }

            /// Exact finite IEEE ingress, validating both endpoints and order.
            /// `0.1` is its binary value, not the rational 1/10.
            pub fn new(lower: $float, upper: $float) -> Result<Self, IntervalError> {
                qualify()?;
                let a = $format.finite(u64::from(lower.to_bits()))?;
                let b = $format.finite(u64::from(upper.to_bits()))?;
                Ok(Self {
                    bounds: Some(Bounds::new($endpoint(a as $bits), $endpoint(b as $bits))?),
                })
            }

            /// Exact singleton from a finite IEEE value.
            pub fn point(value: $float) -> Result<Self, IntervalError> {
                qualify()?;
                let bits = $format.finite(u64::from(value.to_bits()))?;
                Ok(Self {
                    bounds: Some(Bounds::point($endpoint(bits as $bits))),
                })
            }

            /// Canonical empty set; no special endpoint encodes emptiness.
            pub const fn empty() -> Self {
                Self { bounds: None }
            }

            /// Whether this set is empty.
            pub fn is_empty(&self) -> bool {
                self.bounds.is_none()
            }

            /// Whether this set is a single exact real value.
            pub fn is_singleton(&self) -> bool {
                self.bounds.is_some_and(Bounds::is_singleton)
            }

            fn bits(&self) -> Option<(u64, u64)> {
                self.bounds.map(|b| {
                    let (a, b) = b.endpoints();
                    (u64::from(a.0), u64::from(b.0))
                })
            }

            fn from_bits(a: u64, b: u64) -> Result<Self, IntervalError> {
                Self::new(
                    <$float>::from_bits(a as $bits),
                    <$float>::from_bits(b as $bits),
                )
            }

            /// Exact endpoints, or None for empty. Bit transfer performs no rounding.
            pub fn bounds(&self) -> Option<($float, $float)> {
                self.bits().map(|(a, b)| {
                    (
                        <$float>::from_bits(a as $bits),
                        <$float>::from_bits(b as $bits),
                    )
                })
            }

            /// Exact endpoint distance, retained without subtraction or rounding.
            /// Empty has no width. Use the width's explicit `enclosure` method
            /// only when a finite floating-point interval projection is wanted.
            pub fn width(&self) -> Option<$width> {
                self.bounds.map(|bounds| $width { bounds })
            }

            /// Tight outward enclosure of the exact midpoint `(lower+upper)/2`.
            /// Halves are not rounded before addition; overflow of the unhalved
            /// sum is avoided. Empty maps to empty. No point is chosen implicitly.
            pub fn midpoint_enclosure(&self) -> Result<Self, IntervalError> {
                match self.bits() {
                    None => Ok(Self::empty()),
                    Some((a, b)) => {
                        let (lo, hi) = $format.half_sum(a, b)?;
                        Self::from_bits(lo, hi)
                    }
                }
            }

            /// Tight outward enclosure of the exact radius `(upper-lower)/2`.
            /// This can enclose half a subnormal without rounding it to zero.
            /// It measures the source radius, not a radius about a rounded center.
            pub fn radius_enclosure(&self) -> Result<Self, IntervalError> {
                match self.bits() {
                    None => Ok(Self::empty()),
                    Some((a, b)) => {
                        let (lo, hi) = $format.half_sum(b, $format.negate(a))?;
                        Self::from_bits(lo, hi)
                    }
                }
            }

            /// Exact lower endpoint, or None for empty.
            pub fn lower(&self) -> Option<$float> {
                self.bounds().map(|v| v.0)
            }

            /// Exact upper endpoint, or None for empty.
            pub fn upper(&self) -> Option<$float> {
                self.bounds().map(|v| v.1)
            }

            /// Membership of an exact finite IEEE point. Nonfinite input fails
            /// even when this set is empty.
            pub fn contains_point(&self, value: $float) -> Result<bool, IntervalError> {
                qualify()?;
                let bits = $format.finite(u64::from(value.to_bits()))?;
                Ok(self
                    .bounds
                    .is_some_and(|b| b.contains($endpoint(bits as $bits))))
            }

            /// Set inclusion; empty is included in every set.
            pub fn subset_of(&self, other: &Self) -> bool {
                match (self.bounds, other.bounds) {
                    (None, _) => true,
                    (Some(_), None) => false,
                    (Some(a), Some(b)) => a.subset(b),
                }
            }

            /// Inclusion in the real interior, with strict endpoint inequalities.
            pub fn interior_subset_of(&self, other: &Self) -> bool {
                match (self.bounds, other.bounds) {
                    (None, _) => true,
                    (Some(_), None) => false,
                    (Some(a), Some(b)) => a.interior_subset(b),
                }
            }

            /// Whether both sets are nonempty and this set lies strictly left.
            pub fn strictly_left_of(&self, other: &Self) -> bool {
                match (self.bounds, other.bounds) {
                    (Some(a), Some(b)) => a.left_of(b),
                    _ => false,
                }
            }

            /// Whether the intersection is empty.
            pub fn disjoint(&self, other: &Self) -> bool {
                self.is_empty()
                    || other.is_empty()
                    || self.strictly_left_of(other)
                    || other.strictly_left_of(self)
            }

            /// Whether the sets share at least one real value.
            pub fn overlaps(&self, other: &Self) -> bool {
                !self.disjoint(other)
            }

            /// Smallest closed interval containing both sets.
            pub fn hull(&self, other: &Self) -> Self {
                Self {
                    bounds: match (self.bounds, other.bounds) {
                        (Some(a), Some(b)) => Some(a.hull(b)),
                        (a, b) => a.or(b),
                    },
                }
            }

            /// Exact set intersection, or empty when disjoint.
            pub fn intersection(&self, other: &Self) -> Self {
                Self {
                    bounds: match (self.bounds, other.bounds) {
                        (Some(a), Some(b)) => a.intersection(b),
                        _ => None,
                    },
                }
            }

            /// Signs present in this set; not a decision about an enclosed point.
            pub fn signs(&self) -> SignSet {
                self.bounds
                    .map_or(SignSet::Empty, |b| b.signs($endpoint(0)))
            }

            /// Exact absolute-value image, retaining zero as the minimum across
            /// zero. Uses integer sign/order operations and preserves subnormals.
            pub fn abs(&self) -> Self {
                let Some((a, b)) = self.bits() else {
                    return Self::empty();
                };
                if $format.compare(a, 0) != Ordering::Less {
                    return *self;
                }
                if $format.compare(b, 0) == Ordering::Less {
                    return self.neg();
                }
                let reflected = $format.negate(a);
                let upper = if $format.compare(reflected, b) == Ordering::Greater {
                    reflected
                } else {
                    b
                };
                Self {
                    bounds: Some(Bounds::ordered($endpoint(0), $endpoint(upper as $bits))),
                }
            }

            /// Exact image of `min(x, y)` over independent members, using
            /// integer IEEE order. Either empty operand gives empty.
            pub fn min_image(&self, other: &Self) -> Self {
                Self {
                    bounds: match (self.bounds, other.bounds) {
                        (Some(a), Some(b)) => Some(a.min_image(b)),
                        _ => None,
                    },
                }
            }

            /// Exact image of `max(x, y)` over independent members.
            /// Either empty operand gives empty; this differs from hull.
            pub fn max_image(&self, other: &Self) -> Self {
                Self {
                    bounds: match (self.bounds, other.bounds) {
                        (Some(a), Some(b)) => Some(a.max_image(b)),
                        _ => None,
                    },
                }
            }

            /// Exact reflected set.
            pub fn neg(&self) -> Self {
                Self {
                    bounds: self.bits().map(|(a, b)| {
                        Bounds::ordered(
                            $endpoint($format.negate(b) as $bits),
                            $endpoint($format.negate(a) as $bits),
                        )
                    }),
                }
            }

            /// Tight finite outward sum. Required infinite endpoints fail.
            pub fn add(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bits(), other.bits()) {
                    (Some((a, b)), Some((c, d))) => {
                        Self::from_bits($format.add(a, c)?.0, $format.add(b, d)?.1)
                    }
                    _ => Ok(Self::empty()),
                }
            }

            /// Tight finite outward Minkowski difference.
            pub fn sub(&self, other: &Self) -> Result<Self, IntervalError> {
                self.add(&other.neg())
            }

            fn corner_image(&self, other: &Self, divide: bool) -> Result<Self, IntervalError> {
                let (Some((a, b)), Some((c, d))) = (self.bits(), other.bits()) else {
                    return Ok(Self::empty());
                };
                let mut result = Self::empty();
                for (x, y) in [(a, c), (a, d), (b, c), (b, d)] {
                    let (lo, hi) = if divide {
                        $format.div(x, y)?
                    } else {
                        $format.mul(x, y)?
                    };
                    result = result.hull(&Self::from_bits(lo, hi)?);
                }
                Ok(result)
            }

            /// Tight finite outward product hull, treating operands independently.
            pub fn mul(&self, other: &Self) -> Result<Self, IntervalError> {
                self.corner_image(other, false)
            }

            /// Tight finite outward quotient hull. A zero-containing denominator
            /// fails even when the numerator is empty.
            pub fn div(&self, other: &Self) -> Result<Self, IntervalError> {
                if other.signs().contains_zero() {
                    return Err(IntervalError::ContainsZero);
                }
                self.corner_image(other, true)
            }

            /// Tight finite reciprocal; a zero-containing input fails.
            pub fn reciprocal(&self) -> Result<Self, IntervalError> {
                if self.is_empty() {
                    return Ok(Self::empty());
                }
                Self::from_bits($format.one(), $format.one())?.div(self)
            }

            /// Tight finite outward principal square root, using integer
            /// arithmetic independent of native rounding and FTZ/DAZ modes.
            /// Negative source members are excluded with `Partial` coverage;
            /// strictly negative nonempty input returns `EmptyDomain`. Empty
            /// maps to empty with `Complete` coverage on every target.
            pub fn sqrt(&self) -> Result<RangeEnclosure<Self>, IntervalError> {
                let Some((a, b)) = self.bits() else {
                    return Ok(RangeEnclosure {
                        range: Self::empty(),
                        domain: DomainCoverage::Complete,
                    });
                };
                if $format.compare(b, 0).is_lt() {
                    return Err(IntervalError::EmptyDomain);
                }
                let partial = $format.compare(a, 0).is_lt();
                let (lower, lower_ceil) = $format.sqrt(if partial { 0 } else { a })?;
                let upper = if a == b {
                    lower_ceil
                } else {
                    $format.sqrt(b)?.1
                };
                Ok(RangeEnclosure {
                    range: Self::from_bits(lower, upper)?,
                    domain: if partial {
                        DomainCoverage::Partial
                    } else {
                        DomainCoverage::Complete
                    },
                })
            }

            /// Tight outward pointwise square, with zero minimum across zero.
            pub fn square(&self) -> Result<Self, IntervalError> {
                let Some((a, b)) = self.bits() else {
                    return Ok(Self::empty());
                };
                let aa = $format.mul(a, a)?;
                let bb = $format.mul(b, b)?;
                let mut result = Self::from_bits(aa.0, aa.1)?.hull(&Self::from_bits(bb.0, bb.1)?);
                if self.signs().contains_zero() {
                    result = result.hull(&Self::from_bits(0, 0)?);
                }
                Ok(result)
            }

            /// Outward nonnegative integer power. At most 32 squaring and 32
            /// product stages per source endpoint, each evaluating both directed
            /// bounds. Intermediate rounding may widen the image
            /// or require Overflow. Zero exponent maps nonempty input to one.
            pub fn checked_pow(&self, exponent: u32) -> Result<Self, IntervalError> {
                if exponent == 2 {
                    return self.square();
                }
                let Some((a, b)) = self.bits() else {
                    return Ok(Self::empty());
                };
                let aa = $format.power(a, exponent)?;
                let bb = $format.power(b, exponent)?;
                let mut result = Self::from_bits(aa.0, aa.1)?.hull(&Self::from_bits(bb.0, bb.1)?);
                if exponent != 0 && exponent.is_multiple_of(2) && self.signs().contains_zero() {
                    result = result.hull(&Self::from_bits(0, 0)?);
                }
                Ok(result)
            }

            /// Tight outward `scale * x + offset`, rounded only after exact
            /// multiplication and addition. Cancellation can recover a finite
            /// result from an out-of-range product. Scalar inputs must be finite
            /// even when this set is empty.
            pub fn affine(&self, scale: $float, offset: $float) -> Result<Self, IntervalError> {
                let scale = Self::point(scale)?;
                let offset = Self::point(offset)?;
                let Some((a, b)) = self.bits() else {
                    return Ok(Self::empty());
                };
                let scale = scale.bits().expect("scalar point").0;
                let offset = offset.bits().expect("scalar point").0;
                let aa = $format.affine(a, scale, offset)?;
                let bb = $format.affine(b, scale, offset)?;
                Ok(Self::from_bits(aa.0, aa.1)?.hull(&Self::from_bits(bb.0, bb.1)?))
            }

            /// Adjacent finite IEEE value below an exact finite input.
            pub fn next_down(value: $float) -> Result<$float, IntervalError> {
                qualify()?;
                let bits = $format.finite(u64::from(value.to_bits()))?;
                Ok(<$float>::from_bits($format.next_down(bits)? as $bits))
            }

            /// Adjacent finite IEEE value above an exact finite input.
            pub fn next_up(value: $float) -> Result<$float, IntervalError> {
                qualify()?;
                let bits = $format.finite(u64::from(value.to_bits()))?;
                Ok(<$float>::from_bits($format.next_up(bits)? as $bits))
            }

            /// Expands each endpoint by one representable value, failing if
            /// either required endpoint would be infinite. Empty stays empty.
            pub fn expanded(&self) -> Result<Self, IntervalError> {
                match self.bits() {
                    None => Ok(Self::empty()),
                    Some((a, b)) => Self::from_bits($format.next_down(a)?, $format.next_up(b)?),
                }
            }

            /// Exact conversion of the represented endpoints to rational form.
            /// This preserves the binary IEEE values, not a decimal rendering.
            #[cfg(feature = "rational")]
            pub fn to_rational(&self) -> crate::RationalInterval {
                match self.bounds() {
                    None => crate::RationalInterval::empty(),
                    Some((a, b)) => crate::RationalInterval::new(
                        hyperreal::Rational::try_from(a).expect("validated finite IEEE endpoint"),
                        hyperreal::Rational::try_from(b).expect("validated finite IEEE endpoint"),
                    )
                    .expect("exact conversion preserves endpoint order"),
                }
            }

            /// Tight finite IEEE enclosure of an exact dyadic source interval.
            ///
            /// Endpoints round outward, including to zero/minimum subnormal on
            /// underflow. Off-grid bounds widen; this is containment, not exact
            /// equality with the borrowed source. If no finite enclosure exists,
            /// returns Overflow instead of infinity or saturation. Empty maps to
            /// empty without work; nonempty input requires backend qualification.
            ///
            /// Both endpoint conversions share the caller's DyadicBudget. Each
            /// reserves three linear stages of max(input magnitude bits, IEEE
            /// significand bits plus one), before limb inspection/allocation.
            /// Extreme exponents do not cause exponent-sized shifts. Budget
            /// accounting excludes actual heap bytes and cancellation as usual.
            /// Failure publishes no partial interval and leaves the source intact.
            #[cfg(feature = "dyadic")]
            pub fn from_dyadic(
                source: &crate::DyadicInterval,
                budget: &mut crate::DyadicBudget,
            ) -> Result<Self, IntervalError> {
                let Some((a, b, exponent)) = source.bounds() else {
                    return Ok(Self::empty());
                };
                qualify()?;
                let lower = $format.enclose_dyadic(a, exponent, budget)?.0;
                let upper = $format.enclose_dyadic(b, exponent, budget)?.1;
                Self::from_bits(lower, upper)
            }

            /// Exact common-scale dyadic conversion. Mantissa construction and
            /// scale alignment share the caller's size/work budget.
            #[cfg(feature = "dyadic")]
            pub fn to_dyadic(
                &self,
                budget: &mut crate::DyadicBudget,
            ) -> Result<crate::DyadicInterval, IntervalError> {
                let Some((a, b)) = self.bits() else {
                    return Ok(crate::DyadicInterval::empty());
                };
                $format
                    .dyadic_point(a, budget)?
                    .hull(&$format.dyadic_point(b, budget)?, budget)
            }
        }

        #[cfg(feature = "serde")]
        impl serde::Serialize for $name {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.bits()
                    .map(|(a, b)| (a as $bits, b as $bits))
                    .serialize(serializer)
            }
        }
        #[cfg(feature = "serde")]
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                match Option::<($bits, $bits)>::deserialize(deserializer)? {
                    None => Ok(Self::empty()),
                    Some((a, b)) => Self::from_bits(u64::from(a), u64::from(b)),
                }
                .map_err(serde::de::Error::custom)
            }
        }
    };
}

enclosure!(F32Enclosure, Binary32, f32, u32, BINARY32, F32Width, 5);
enclosure!(F64Enclosure, Binary64, f64, u64, BINARY64, F64Width, 33);

#[cfg(all(feature = "rational", feature = "dyadic"))]
mod rational;

impl F32Enclosure {
    /// Converts both endpoints exactly to binary64. Empty stays empty.
    pub fn to_f64_enclosure(&self) -> Result<F64Enclosure, IntervalError> {
        let Some((lo, hi)) = self.bits() else {
            return Ok(F64Enclosure::empty());
        };
        F64Enclosure::from_bits(
            BINARY64.enclose_from(BINARY32, lo)?.0,
            BINARY64.enclose_from(BINARY32, hi)?.1,
        )
    }
}

impl F64Enclosure {
    /// Tightest binary32 enclosure. Rounds lower and upper endpoints outward,
    /// preserving empty. Returns [`IntervalError::Overflow`] when the source
    /// extends beyond the finite binary32 range.
    pub fn to_f32_enclosure(&self) -> Result<F32Enclosure, IntervalError> {
        let Some((lo, hi)) = self.bits() else {
            return Ok(F32Enclosure::empty());
        };
        F32Enclosure::from_bits(
            BINARY32.enclose_from(BINARY64, lo)?.0,
            BINARY32.enclose_from(BINARY64, hi)?.1,
        )
    }
}
