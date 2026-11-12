use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector};

type Components<P> = (P, P);

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
        /// This form is not an affine source. An explicitly requested enclosure
        /// can enter existing box operations without replacing this definition.
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        pub struct $name<I: IntervalArithmetic, $(const $dimension: usize),+> {
            components: Option<Components<$product<I, $($dimension),+>>>,
        }

        impl<I: IntervalArithmetic, $(const $dimension: usize),+> $name<I, $($dimension),+> {
            /// Validates singleton centers, singleton radii, then nonnegative
            /// radius signs in component order. Empty components are invalid;
            /// use `empty()` for the canonical empty set. No scalar copies or
            /// arithmetic occur; signs are exact native facts.
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
        impl<I: IntervalArithmetic + serde::Serialize, $(const $dimension: usize),+>
            serde::Serialize for $name<I, $($dimension),+>
        {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                self.components.serialize(serializer)
            }
        }

        #[cfg(feature = "serde")]
        impl<'de, I: IntervalArithmetic + serde::Deserialize<'de>, $(const $dimension: usize),+>
            serde::Deserialize<'de> for $name<I, $($dimension),+>
        {
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
