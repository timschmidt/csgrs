//! Unpublished layout experiment; no production API or arithmetic authority.
//!
//! The scalar constructors validate reconstruction, including on platforms
//! where IEEE ingress is unqualified. Separate arrays cannot implement the
//! production vector's borrowed `lanes()` API without extra storage.
//!
//! This compares inline Cartesian storage, not correlated affine images. Scalar
//! rounding and errors remain Hyperinterval's; the unit context adds no work
//! budget. Arrays and temporary copies grow with N, with no allocation or SIMD
//! introduced by this adapter. Native probes and the Wasm client consume the same
//! workload definitions. Agreement with the native layout alone is not an
//! independent arithmetic oracle or a performance guarantee.
#![no_std]
#![forbid(unsafe_code)]

use hyperlattice_interval::hyperinterval::{
    F32Enclosure, F64Enclosure, FixedInterval, IntervalError,
};
use hyperlattice_interval::{IntervalArithmetic, IntervalLatticeError, IntervalVector};

pub mod workload;

/// Endpoint extraction and checked reconstruction for the four primitive families.
///
/// Implementations cover i32/i64 fixed intervals at any type-level scale and
/// qualified f32/f64 enclosures. Invalid fixed scales remain constructor errors.
/// Bounds are exact stored endpoints, with None reserved for empty; restore must
/// preserve their meaning through the existing scalar constructor. This adds no
/// arithmetic authority to the sealed, unit-context [`IntervalArithmetic`] trait.
pub trait Primitive: IntervalArithmetic<Context = ()> + Copy {
    /// Raw fixed numerator or finite IEEE endpoint; Default only initializes scratch arrays.
    type Endpoint: Copy + Default;
    /// Copy both endpoints without rounding, or return None for the empty scalar set.
    fn bounds(&self) -> Option<(Self::Endpoint, Self::Endpoint)>;
    /// Reconstruct nonempty ordered bounds, preserving scalar validation failures.
    fn restore(lower: Self::Endpoint, upper: Self::Endpoint) -> Result<Self, IntervalError>;
}

/// Internal callback-order regression that can exercise the private zip helper.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// An empty first lane cannot hide a later error; all-empty success visits all lanes.
    fn empty_lane_does_not_skip_later_failure() {
        /// Valid Q8 scalar family for observing callback order without arithmetic overflow.
        type I = FixedInterval<i32, 8>;
        let v = Soa::from_vector(&IntervalVector::new([I::point(0).unwrap(); 3]).unwrap());
        let mut calls = 0;
        let result = v.zip(&v, |_, _, _| {
            calls += 1;
            if calls == 2 {
                Err(IntervalError::Overflow)
            } else {
                I::empty()
            }
        });
        assert_eq!(calls, 2);
        assert_eq!(
            result.err(),
            Some(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
        let mut calls = 0;
        assert!(
            v.zip(&v, |_, _, _| {
                calls += 1;
                I::empty()
            })
            .unwrap()
            .to_vector()
            .unwrap()
            .is_empty()
        );
        assert_eq!(calls, 3);
    }
}

/// Generate endpoint adapters for one signed width, retaining every fractional-bit parameter.
macro_rules! fixed {
    ($endpoint:ty) => {
        /// Preserve the fixed grid; reconstruction validates scale before endpoint order.
        impl<const F: u32> Primitive for FixedInterval<$endpoint, F> {
            /// Signed numerator in units of 2^-F, not a rounded real coordinate.
            type Endpoint = $endpoint;
            /// Forward exact raw endpoints, including the empty discriminator.
            fn bounds(&self) -> Option<($endpoint, $endpoint)> {
                self.bounds()
            }
            /// Forward checked fixed ingress; invalid scale and reversed bounds stay typed.
            fn restore(lower: $endpoint, upper: $endpoint) -> Result<Self, IntervalError> {
                Self::new(lower, upper)
            }
        }
    };
}
fixed!(i32);
fixed!(i64);

/// Generate adapters that preserve finite IEEE values and delegate platform qualification.
macro_rules! float {
    ($interval:ty, $endpoint:ty) => {
        /// Extract stored IEEE values and reconstruct through qualified finite ingress.
        impl Primitive for $interval {
            /// Native IEEE endpoint; its binary value is authoritative.
            type Endpoint = $endpoint;
            /// Transfer endpoint bits without floating arithmetic; empty yields None.
            fn bounds(&self) -> Option<($endpoint, $endpoint)> {
                self.bounds()
            }
            /// Qualify ingress, reject nonfinite or reversed bounds, and canonicalize signed zero.
            fn restore(lower: $endpoint, upper: $endpoint) -> Result<Self, IntervalError> {
                Self::new(lower, upper)
            }
        }
    };
}
float!(F32Enclosure, f32);
float!(F64Enclosure, f64);

/// Separate endpoint arrays with one canonical Cartesian empty discriminator.
/// Fields are private; there is no unchecked ingress or separate wire format.
///
/// Some of two empty arrays is the zero-dimensional unit product, distinct from
/// None. Nonempty storage holds validated, ordered endpoints for every lane.
/// Clone and Copy duplicate the inline arrays; storage and temporary work scale
/// with N. Operations return native scalar failures without partial output and
/// leave the operands reusable. They do not retain cross-coordinate dependencies.
#[derive(Clone, Copy)]
pub struct Soa<I: Primitive, const N: usize> {
    /// Canonical empty tag or all lower endpoints followed by all upper endpoints.
    bounds: Option<EndpointArrays<I::Endpoint, N>>,
}

/// Parallel arrays for the same N lanes: tuple position zero is lower, one is upper.
type EndpointArrays<E, const N: usize> = ([E; N], [E; N]);

/// Conversion and ordered scalar delegation for the experimental layout.
impl<I: Primitive, const N: usize> Soa<I, N> {
    /// Copy an already-validated Cartesian vector into separate endpoint arrays.
    ///
    /// Preserves empty/unit products and endpoint values without arithmetic or
    /// fresh family validation. The expect relies on IntervalVector's invariant
    /// that a stored nonempty product contains no empty scalar lane.
    pub fn from_vector(vector: &IntervalVector<I, N>) -> Self {
        Self {
            bounds: vector.lanes().map(|lanes| {
                let pairs = lanes.map(|lane| lane.bounds().expect("nonempty Cartesian lane"));
                (pairs.map(|p| p.0), pairs.map(|p| p.1))
            }),
        }
    }

    /// Restore through the existing scalar and Cartesian constructors.
    ///
    /// Empty delegates to the native empty constructor. Nonempty storage first
    /// constructs an empty scalar scratch value even at N=0, restores lanes in
    /// index order, then validates/canonicalizes the Cartesian result. The first
    /// failure aborts without modifying self. IEEE reconstruction rechecks target
    /// qualification; an empty/unit result alone does not certify IEEE ingress.
    pub fn to_vector(&self) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match &self.bounds {
            None => IntervalVector::empty(),
            Some((lower, upper)) => {
                let mut lanes = [I::empty()?; N];
                for i in 0..N {
                    lanes[i] = I::restore(lower[i], upper[i])?;
                }
                IntervalVector::new(lanes)
            }
        }
    }

    /// Restore left then right lane and apply the callback in increasing index order.
    ///
    /// An already-empty operand only validates the family and bypasses the callback.
    /// For nonempty operands, remember an empty result but still visit later lanes;
    /// reconstruction/callback errors stop immediately and suppress partial output.
    /// After successful traversal, validate the family and canonicalize emptiness.
    /// Unit/unit calls no callback. Default endpoints only fill temporary slots;
    /// they are never substitutes for an error or a returned empty lane.
    fn zip(
        &self,
        other: &Self,
        mut operation: impl FnMut(&I, &I, &mut ()) -> Result<I, IntervalError>,
    ) -> Result<Self, IntervalLatticeError> {
        let (Some((al, au)), Some((bl, bu))) = (&self.bounds, &other.bounds) else {
            I::validate_family()?;
            return Ok(Self { bounds: None });
        };
        let mut lower = [I::Endpoint::default(); N];
        let mut upper = lower;
        let mut empty = false;
        for i in 0..N {
            let a = I::restore(al[i], au[i])?;
            let b = I::restore(bl[i], bu[i])?;
            match operation(&a, &b, &mut ())?.bounds() {
                Some((l, u)) => {
                    lower[i] = l;
                    upper[i] = u;
                }
                // Keep evaluating later lanes, just as IntervalVector::zip
                // does before canonicalization. No partial output is published.
                None => empty = true,
            }
        }
        I::validate_family()?;
        Ok(Self {
            bounds: if empty { None } else { Some((lower, upper)) },
        })
    }

    /// Enclose coordinate sums through scalar addition; an empty operand yields empty.
    /// Rounding and first-error behavior follow ordered lane reconstruction and zip.
    pub fn add(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::add)
    }

    /// Enclose coordinate differences, treating repeated operand references independently.
    /// A box minus itself need not be zero; empty and first-error behavior follow zip.
    pub fn sub(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::sub)
    }

    /// Exact Cartesian intersection, canonicalizing empty lanes after all successful work.
    /// An empty intermediate lane does not bypass later reconstruction or callback errors.
    pub fn intersection(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        self.zip(other, I::intersection)
    }

    /// Smallest Cartesian containing box, which can exceed the operands' set union.
    ///
    /// With an empty operand, reconstruct both native vectors left-to-right, call
    /// their hull (including native scalar copies), then import the result. With
    /// two nonempty operands, reconstruct and hull individual lanes through zip.
    pub fn hull(&self, other: &Self) -> Result<Self, IntervalLatticeError> {
        // Empty/nonempty hull copies through the native path, including its
        // family validation and scalar copy calls.
        if self.bounds.is_none() || other.bounds.is_none() {
            return Ok(Self::from_vector(
                &self.to_vector()?.hull(&other.to_vector()?, &mut ())?,
            ));
        }
        self.zip(other, I::hull)
    }

    /// Dot-product enclosure with a zero seed and left-to-right multiply/add steps.
    ///
    /// An empty operand delegates to scalar empty; unit/unit delegates to scalar
    /// zero. Otherwise each lane restores left then right before multiplication
    /// and accumulation. Intermediate overflow is not canceled by a later term.
    /// Repeated references do not imply dependency-aware squaring. Native rounding
    /// and errors are preserved without adding a resource counter or certainty receipt.
    pub fn dot(&self, other: &Self) -> Result<I, IntervalLatticeError> {
        let (Some((al, au)), Some((bl, bu))) = (&self.bounds, &other.bounds) else {
            return Ok(I::empty()?);
        };
        let mut sum = I::zero()?;
        for i in 0..N {
            let a = I::restore(al[i], au[i])?;
            let b = I::restore(bl[i], bu[i])?;
            let product = a.mul(&b, &mut ())?;
            sum = sum.add(&product, &mut ())?;
        }
        Ok(sum)
    }
}
