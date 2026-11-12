//! Fixed-size Cartesian interval algebra, re-exported by Hyperlattice.
//!
//! Each carrier denotes a Cartesian product of scalar sets. Arithmetic returns
//! an enclosure; repeated variables and shared input coordinates can introduce
//! correlations that a Cartesian product does not retain. An affine enclosure
//! is not the exact correlated affine image. No midpoint is selected implicitly.
//!
//! Scalar arithmetic and environment qualification belong to Hyperinterval.
//! The primitive build is allocation-free `no_std`, with dimension-bounded
//! loops and inline storage. Dyadic operations share one caller-owned budget
//! across all lanes and nested products. Rational arithmetic has no internal
//! work/heap bound. Explicit cloning and serialization are caller-owned work.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod backend;
mod matrix;
#[cfg(feature = "serde")]
mod serde;
mod vector;

pub use backend::IntervalArithmetic;
pub use hyperinterval;
pub use matrix::IntervalMatrix;
pub use vector::IntervalVector;

/// An axis-aligned box, with the same Cartesian semantics as an interval vector.
pub type BoxN<I, const N: usize> = IntervalVector<I, N>;

/// Failure of an interval-linear operation. Empty sets are successful values.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum IntervalLatticeError {
    /// The scalar backend rejected an operation; its typed cause is retained.
    Scalar(hyperinterval::IntervalError),
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
            Self::IndexOutOfBounds { .. } => None,
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
