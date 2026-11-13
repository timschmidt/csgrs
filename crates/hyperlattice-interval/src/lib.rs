//! Fixed-size Cartesian interval algebra, re-exported by Hyperlattice.
//!
//! Vectors and matrices denote Cartesian products of scalar sets. Arithmetic returns
//! an enclosure; repeated variables and shared input coordinates can introduce
//! correlations that a Cartesian product does not retain. An affine enclosure
//! is not the exact correlated affine image. [`AffineImage`] retains the source
//! and its maps instead, including through further affine operations and replay.
//! No midpoint is selected implicitly.
//!
//! Scalar arithmetic and environment qualification belong to Hyperinterval.
//! The primitive build is allocation-free `no_std`, with dimension-bounded
//! loops and inline storage. Dyadic operations share one caller-owned budget
//! across all lanes and nested products. Rational arithmetic has no internal
//! work/heap bound. Explicit cloning and serialization are caller-owned work.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod affine;
mod backend;
mod centered;
mod contraction;
mod matrix;
mod midpoint_radius;
mod norms;
#[cfg(feature = "serde")]
mod serde;
mod vector;
mod widths;

pub use affine::{AffineBox, AffineChain, AffineImage, AffineMap, AffineSource, TranslatedBox};
pub use backend::IntervalArithmetic;
pub use centered::{CenteredMatrix, CenteredVector};
pub use hyperinterval;
pub use matrix::IntervalMatrix;
pub use midpoint_radius::{MidpointRadiusMatrix, MidpointRadiusVector};
pub use vector::IntervalVector;

/// An axis-aligned box, with the same Cartesian semantics as an interval vector.
pub type BoxN<I, const N: usize> = IntervalVector<I, N>;

/// Failure of an interval-linear operation. Empty sets are successful values.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum IntervalLatticeError {
    /// The scalar backend rejected an operation; its typed cause is retained.
    Scalar(hyperinterval::IntervalError),
    /// An affine map's matrix is empty or has a nonsingleton coefficient.
    NonPointAffineMatrix,
    /// An affine map's translation is empty or has a nonsingleton component.
    NonPointAffineTranslation,
    /// An authored center is empty or contains more than one point.
    NonPointCenter,
    /// An authored radius is empty or contains more than one point.
    NonPointRadius,
    /// An authored radius lane is negative.
    NegativeRadius,
    /// A bounded affine chain already contains its declared maximum maps.
    AffineCapacityExhausted {
        /// Maximum retained map count, fixed by the chain's type.
        capacity: usize,
    },
    /// A row or column index lies outside its declared dimension.
    IndexOutOfBounds {
        /// Requested zero-based index.
        index: usize,
        /// Number of rows or columns.
        dimension: usize,
    },
}

impl From<hyperinterval::IntervalError> for IntervalLatticeError {
    fn from(value: hyperinterval::IntervalError) -> Self {
        Self::Scalar(value)
    }
}

impl core::fmt::Display for IntervalLatticeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Scalar(error) => write!(f, "interval arithmetic failed: {error}"),
            Self::NonPointAffineMatrix => {
                f.write_str("affine matrix must contain exactly one matrix")
            }
            Self::NonPointAffineTranslation => {
                f.write_str("affine translation must contain exactly one vector")
            }
            Self::AffineCapacityExhausted { capacity } => {
                write!(f, "affine chain capacity {capacity} exhausted")
            }
            Self::NonPointCenter => f.write_str("center must contain exactly one point"),
            Self::NonPointRadius => f.write_str("radius must contain exactly one point"),
            Self::NegativeRadius => f.write_str("radius must be nonnegative"),
            Self::IndexOutOfBounds { index, dimension } => {
                write!(f, "index {index} is outside dimension {dimension}")
            }
        }
    }
}

impl core::error::Error for IntervalLatticeError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Scalar(error) => Some(error),
            Self::AffineCapacityExhausted { .. }
            | Self::IndexOutOfBounds { .. }
            | Self::NonPointAffineMatrix
            | Self::NonPointAffineTranslation
            | Self::NonPointCenter
            | Self::NonPointRadius
            | Self::NegativeRadius => None,
        }
    }
}

// Stable Rust 1.88 has no array::try_from_fn. Initialized elements drop on
// failure; only a fully constructed array is published. No unsafe or heap.
fn array_result<T, E, const N: usize>(
    mut f: impl FnMut(usize) -> Result<T, E>,
) -> Result<[T; N], E> {
    let mut elements = core::array::from_fn(|_| None);
    for (index, slot) in elements.iter_mut().enumerate() {
        *slot = Some(f(index)?);
    }
    Ok(elements.map(|slot| slot.expect("all slots initialized")))
}

fn dot<'a, I: IntervalArithmetic + 'a>(
    terms: impl Iterator<Item = (&'a I, &'a I)>,
    context: &mut I::Context,
) -> Result<I, hyperinterval::IntervalError> {
    let mut sum = I::zero()?;
    for (a, b) in terms {
        let product = a.mul(b, context)?;
        sum = sum.add(&product, context)?;
    }
    Ok(sum)
}

#[cfg(feature = "dyadic")]
mod subdivision;
