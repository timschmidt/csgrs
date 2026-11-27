//! Exact boxes authored as singleton centers and nonnegative singleton radii.
//!
//! These definitions retain their components even when absolute endpoints cannot
//! be represented by the chosen scalar family. Endpoint enclosure is an explicit
//! query, distinct from the source-bounds decomposition in `midpoint_radius`.
//! Construction and validation have no scalar context; numerical projections
//! delegate to the existing Cartesian carriers and share their scalar accounting.

use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector};

/// Owned center and radius products, in that order, shared by both shape variants.
type Components<P> = (P, P);

/// Reject the first radius whose exact native sign set contains a negative value.
///
/// Callers first establish singleton radius products; this helper does not check
/// singleton or empty status itself. It visits the supplied finite component
/// iterator in order, accepting zero and the empty iterator without arithmetic.
/// Validation is unbudgeted and introduces no rounded sign decision.
fn validate_radius<'a, I: IntervalArithmetic + 'a>(
    radii: impl Iterator<Item = &'a I>,
) -> Result<(), IntervalLatticeError> {
    for radius in radii {
        if radius.signs().contains_negative() {
            return Err(IntervalLatticeError::NegativeRadius);
        }
    }
    Ok(())
}

// Both shapes have identical construction/projection rules. Only iteration of
// the radius components differs: lane order versus flattened row-major order.
// Arithmetic is delegated to the existing Cartesian/scalar operations.
/// Generate matching authored-form storage, validation, projection and Serde APIs.
///
/// The product and dimension list choose vector or matrix shape; the supplied
/// radius iterator fixes lane or flattened row-major validation order. Numerical
/// operations remain in the existing product/scalar layers. The description
/// literal becomes the generated type's shape-specific opening documentation.
macro_rules! centered {
    ($name:ident, $product:ident, [$($dimension:ident),+], $description:literal,
     $radius:ident => $radii:expr) => {
        #[doc = $description]
        ///
        /// Each lane denotes `[center-radius, center+radius]`. Both components
        /// must be exact singletons in the same scalar backend, with nonnegative
        /// radii. Construction transfers ownership without computing endpoints.
        /// Thus the exact definition survives endpoint projection overflow.
        ///
        /// This differs from the existing midpoint/radius decomposition, which
        /// retains source bounds. Structural equality compares the authored
        /// components; serde validates them again, without trusting cached flags.
        /// Vector forms can convert to an exact translated affine source with
        /// [`CenteredVector::affine_source`]. An explicit enclosure can also
        /// enter box operations without replacing this definition.
        /// Clone and conditional Copy follow the component carriers and have no
        /// arithmetic context. Validation and serialization do not budget those
        /// ownership operations or provide a total memory bound.
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        pub struct $name<I: IntervalArithmetic, $(const $dimension: usize),+> {
            /// None is canonical empty; Some owns validated singleton center/radius
            /// products with nonnegative radii. Unit products remain Some.
            components: Option<Components<$product<I, $($dimension),+>>>,
        }

        /// Construction, immutable component access and explicit endpoint projection.
        impl<I: IntervalArithmetic, $(const $dimension: usize),+> $name<I, $($dimension),+> {
            /// Validates singleton centers, singleton radii, then nonnegative
            /// radius signs in component order. Empty components are invalid;
            /// use `empty()` for the canonical empty set. No scalar copies or
            /// arithmetic occur; signs are exact native facts.
            /// Inputs are consumed, including on validation failure. Valid
            /// zero-sized unit products require no radius-entry checks.
            pub fn new(
                center: $product<I, $($dimension),+>,
                radius: $product<I, $($dimension),+>,
            ) -> Result<Self, IntervalLatticeError> {
                if !center.is_singleton() {
                    return Err(IntervalLatticeError::NonPointCenter);
                }
                if !radius.is_singleton() {
                    return Err(IntervalLatticeError::NonPointRadius);
                }
                let $radius = &radius;
                validate_radius($radii)?;
                Ok(Self { components: Some((center, radius)) })
            }

            /// Canonical empty set, validating the scalar family even when a
            /// dimension is zero. This differs from a zero-dimensional unit.
            pub fn empty() -> Result<Self, IntervalLatticeError> {
                I::validate_family()?;
                Ok(Self { components: None })
            }

            /// Whether the exact represented set is empty.
            pub const fn is_empty(&self) -> bool {
                self.components.is_none()
            }

            /// Authored exact center, absent only for the canonical empty set.
            pub const fn center(&self) -> Option<&$product<I, $($dimension),+>> {
                match &self.components {
                    Some((center, _)) => Some(center),
                    None => None,
                }
            }

            /// Authored exact nonnegative radius; no midpoint was rounded.
            pub const fn radius(&self) -> Option<&$product<I, $($dimension),+>> {
                match &self.components {
                    Some((_, radius)) => Some(radius),
                    None => None,
                }
            }

            /// Transfers components without copying or projecting them.
            pub fn into_parts(self) -> Option<Components<$product<I, $($dimension),+>>> {
                self.components
            }

            /// Outward projection into the chosen scalar endpoint family.
            /// Computes `center-radius`, `center+radius`, then their hull,
            /// sharing the context across all operations in that order.
            /// Bounds may round outward or fail typed. No partial output
            /// escapes; the retained exact definition remains unchanged.
            /// The complete lower product is evaluated before the complete
            /// upper product, then the hull; each pass follows lane or row-major
            /// order. Empty/unit products invoke no scalar arithmetic. Previously
            /// charged work is not refunded if a later pass fails.
            pub fn enclosure(
                &self, context: &mut I::Context,
            ) -> Result<$product<I, $($dimension),+>, IntervalLatticeError> {
                let Some((center, radius)) = &self.components else {
                    return $product::empty();
                };
                let lower = center.sub(radius, context)?;
                let upper = center.add(radius, context)?;
                lower.hull(&upper, context)
            }
        }

        #[cfg(feature = "serde")]
        /// Encode authored components directly, without projecting endpoints.
        impl<I: IntervalArithmetic + serde::Serialize, $(const $dimension: usize),+>
            serde::Serialize for $name<I, $($dimension),+>
        {
            /// Serialize the optional center/radius pair with the native product codecs.
            /// No projected coordinates or cached validity flags are emitted.
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.components.serialize(serializer)
            }
        }

        #[cfg(feature = "serde")]
        /// Revalidate decoded products and authored-form invariants through constructors.
        impl<'de, I: IntervalArithmetic + serde::Deserialize<'de>, $(const $dimension: usize),+>
            serde::Deserialize<'de> for $name<I, $($dimension),+>
        {
            /// Decode the optional pair, then validate singleton components and signs.
            /// None still validates the scalar family through `empty`. Native product
            /// decoding checks dimensions and scalar bounds; constructor errors become
            /// Serde errors. No endpoint projection or arithmetic context is involved.
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                // Reuse existing arbitrary-dimension carrier validation.
                match Option::<Components<$product<I, $($dimension),+>>>::deserialize(deserializer)? {
                    Some((center, radius)) => Self::new(center, radius),
                    None => Self::empty(),
                }.map_err(serde::de::Error::custom)
            }
        }
    };
}

centered!(
    CenteredVector, IntervalVector, [N],
    "A Cartesian box defined by independently authored exact centers and radii.",
    radius => radius.lanes().expect("singleton radius").iter()
);
centered!(
    CenteredMatrix, IntervalMatrix, [R, C],
    "A Cartesian matrix set defined by independently authored exact centers and radii.",
    radius => radius.rows().expect("singleton radius").iter().flatten()
);
