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
//!
//! Empty Cartesian products are distinct from zero-dimensional unit products.
//! Constructors validate the scalar family even when there are no coordinates.
//! [`IntervalLatticeError`] retains scalar failures as sources; no partial carrier
//! is returned on failure, although a caller's arithmetic context may already have
//! consumed work. Set enclosures and structural equality do not supply a geometry
//! predicate policy, topology certificate, or proof that two retained definitions
//! denote the same set.
//!
//! # Features and private documentation
//!
//! No backends are enabled by default. `fixed` selects signed i32/i64 fixed-point
//! intervals; `float-enclosure` selects qualified primitive IEEE enclosures;
//! `dyadic` selects arbitrary-precision dyadics with shared work/size accounting;
//! `rational` selects exact rational intervals with Hyperreal/std dependencies.
//! `hyperreal` enables the dyadic and scalar Hyperreal bridge dependencies, not
//! an additional implementation of [`IntervalArithmetic`] for native Real.
//! The `float-enclosure-aarch64-linux-gnu` and `float-enclosure-wasip1` features
//! forward target-qualification selections to Hyperinterval. `serde` enables
//! native carrier serialization and validated deserialization.
//!
//! The crate itself remains `no_std`; optional dependencies have their own
//! allocation, standard-library and compiler requirements. Its declared Rust 1.88
//! minimum applies to the primitive configuration; Hyperreal requires Rust 1.95.
//! From the workspace, include private helpers with
//! `cargo doc -p hyperlattice-interval --all-features --no-deps --document-private-items`.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

/// Retained affine definitions, parameter domains, bounded chains and translated boxes.
mod affine;
/// Sealed forwarding of scalar interval arithmetic and its shared context.
mod backend;
/// Independently authored point centers and nonnegative point radii.
mod centered;
/// Identity remainders and norm-based contraction-margin enclosures.
mod contraction;
/// Row-major Cartesian matrix sets and ordered interval products.
mod matrix;
/// Retained midpoint/radius views of original Cartesian intervals.
mod midpoint_radius;
/// Vector/matrix norm, matrix-measure and dominance-margin enclosures.
mod norms;
/// Native serialization with constructor validation of decoded carrier definitions.
#[cfg(feature = "serde")]
mod serde;
/// Cartesian vector sets, canonical emptiness and coordinatewise arithmetic.
mod vector;
/// Exact coordinate-width queries and widest-axis selection.
mod widths;

pub use affine::{AffineBox, AffineChain, AffineImage, AffineMap, AffineSource, TranslatedBox};
pub use backend::IntervalArithmetic;
pub use centered::{CenteredMatrix, CenteredVector};
pub use hyperinterval;
pub use matrix::IntervalMatrix;
pub use midpoint_radius::{MidpointRadiusMatrix, MidpointRadiusVector};
pub use vector::IntervalVector;

/// An axis-aligned box, with the same Cartesian semantics as an interval vector.
///
/// This alias adds no storage or validation. Any empty coordinate makes the set
/// empty; with N=0, new([]) is a singleton unit product distinct from empty().
pub type BoxN<I, const N: usize> = IntervalVector<I, N>;

/// Failure of an interval-linear operation. Empty sets are successful values.
///
/// Scalar causes are wrapped without reclassification. Shape, radius, index and
/// capacity failures are local variants, with no nested source. This error carries
/// no partial output or context rollback; previously charged work can remain spent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum IntervalLatticeError {
    /// The scalar backend rejected an operation; its typed cause is retained.
    Scalar(
        /// Unchanged scalar diagnostic, also exposed through Error::source.
        hyperinterval::IntervalError,
    ),
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

/// Preserve the native scalar error as a typed cause for lattice consumers.
impl From<hyperinterval::IntervalError> for IntervalLatticeError {
    /// Wrap the error unchanged, including domain, environment and resource failures.
    fn from(value: hyperinterval::IntervalError) -> Self {
        Self::Scalar(value)
    }
}

/// Human-readable diagnostics, including nested scalar messages and index/capacity values.
impl core::fmt::Display for IntervalLatticeError {
    /// Write the selected diagnostic; formatter errors propagate and no replay format is implied.
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

/// Expose only scalar failures as nested causes without flattening their typed diagnostics.
impl core::error::Error for IntervalLatticeError {
    /// Borrow the wrapped IntervalError for Scalar, or return None for local failures.
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

/// Construct all N elements in ascending index order, returning the first callback error.
///
/// Uses an inline Option array to provide fallible array construction on the declared
/// compiler without unsafe code or helper-owned heap allocation. Calls f exactly once
/// for each attempted index; N=0 returns an empty array without invoking it. Successful
/// completion moves every initialized value into the result without cloning.
///
/// On an ordinary error, the initialized prefix is dropped in index order and no
/// partial array escapes. Nested calls drop the failing inner prefix before completed
/// outer elements. Callback effects, including budget consumption, are not rolled back.
/// Callback/drop panics propagate under ordinary Rust unwinding/abort rules. The helper
/// does not bound callback work or T's storage/allocations; inline storage scales with N.
fn array_result<T, E, const N: usize>(
    mut f: impl FnMut(usize) -> Result<T, E>,
) -> Result<[T; N], E> {
    let mut elements = core::array::from_fn(|_| None);
    for (index, slot) in elements.iter_mut().enumerate() {
        *slot = Some(f(index)?);
    }
    Ok(elements.map(|slot| slot.expect("all slots initialized")))
}

/// Enclose a sum of interval products in iterator order using one shared arithmetic context.
///
/// Constructs scalar zero first, then multiplies each pair and adds that product to
/// the running sum. The empty iterator therefore returns zero, not an empty set.
/// Scalar zero, multiplication or addition errors return immediately; consumed work
/// is not refunded. There is no fused product-sum, reassociation, budget reset, or
/// early exit merely because an intermediate enclosure is empty.
///
/// Callers supply finite row/column iterators and handle empty Cartesian operands
/// before entering this helper. Scalar endpoint rounding and dependency loss remain
/// backend/enclosure semantics; even identical references are treated as independent
/// operands. The iterator interface itself supplies no total-work or length bound.
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

/// Exact closed dyadic bisection and explicit outward regridding of Cartesian vectors.
#[cfg(feature = "dyadic")]
mod subdivision;

/// Ordered construction/failure cleanup witnesses for the private array builder.
#[cfg(test)]
mod tests;
