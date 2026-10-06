//! Error and result types for exact voxel-grid construction.

use std::fmt;

/// Result alias used by `hypervoxel`.
pub type HypervoxelResult<T> = Result<T, HypervoxelError>;

/// Validation failures surfaced before a grid can be treated as exact.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HypervoxelError {
    /// A grid depth exceeded the current bounded integer address model.
    DepthTooLarge { depth: u8, max_supported: u8 },
    /// A grid cell axis was structurally non-positive.
    NonPositiveCellAxis { axis: usize },
    /// A grid cell axis could not be proved positive from scalar facts.
    UnknownCellAxisSign { axis: usize },
    /// Integer address conversion would overflow the current exact bounds API.
    AddressOverflow,
    /// A child index was outside the octree child range `0..8`.
    InvalidChildIndex(u8),
    /// The caller requested an operation at a depth outside the grid frame.
    DepthOutsideFrame { depth: u8, frame_depth: u8 },
    /// Exact ordering was required but could not be certified.
    UnknownOrdering { axis: usize },
    /// A lossy adapter could not represent an exact scalar as a primitive float.
    LossyExportUnavailable { field: &'static str },
    /// A query requires all addresses to live at one grid depth.
    MismatchedAddressDepth { left: u8, right: u8 },
    /// A certified scalar ordering was required for aggregate bounds.
    UnknownScalarOrdering { field: &'static str },
    /// An axis transform does not form a valid signed permutation.
    InvalidAxisPermutation,
    /// A support-mask direction must use axis `0..3` and sign `-1` or `1`.
    InvalidSupportDirection,
    /// Explicit aggregate cells exceeded the finite frame they were summarized into.
    InvalidAggregateSummary {
        /// Total cells in the finite frame or region.
        total_cells: usize,
        /// Explicit cells supplied to the summary.
        explicit_cells: usize,
    },
    /// Source geometry failed a structural preflight check.
    InvalidSourceGeometry {
        /// Human-readable validation failure.
        reason: &'static str,
    },
    /// Continuous-field rows failed exact storage-admission checks.
    InvalidContinuousFieldMaterialization {
        /// Human-readable validation failure.
        reason: &'static str,
    },
}

impl fmt::Display for HypervoxelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DepthTooLarge {
                depth,
                max_supported,
            } => write!(
                f,
                "grid depth {depth} exceeds supported exact address depth {max_supported}"
            ),
            Self::NonPositiveCellAxis { axis } => {
                write!(f, "cell axis {axis} is structurally non-positive")
            }
            Self::UnknownCellAxisSign { axis } => {
                write!(f, "cell axis {axis} is not structurally known positive")
            }
            Self::AddressOverflow => write!(f, "voxel address arithmetic overflowed"),
            Self::InvalidChildIndex(index) => write!(f, "invalid octree child index {index}"),
            Self::DepthOutsideFrame { depth, frame_depth } => write!(
                f,
                "address depth {depth} is outside frame depth {frame_depth}"
            ),
            Self::UnknownOrdering { axis } => {
                write!(f, "axis {axis} ordering could not be certified")
            }
            Self::LossyExportUnavailable { field } => {
                write!(f, "lossy export could not represent exact field {field}")
            }
            Self::MismatchedAddressDepth { left, right } => {
                write!(f, "address depths differ: {left} vs {right}")
            }
            Self::UnknownScalarOrdering { field } => {
                write!(f, "scalar ordering for {field} could not be certified")
            }
            Self::InvalidAxisPermutation => {
                write!(f, "axis transform is not a valid signed permutation")
            }
            Self::InvalidSupportDirection => {
                write!(f, "support direction must use axis 0..3 and sign -1 or 1")
            }
            Self::InvalidAggregateSummary {
                total_cells,
                explicit_cells,
            } => write!(
                f,
                "aggregate summary has {explicit_cells} explicit cells for {total_cells} total cells"
            ),
            Self::InvalidSourceGeometry { reason } => {
                write!(f, "invalid exact source geometry: {reason}")
            }
            Self::InvalidContinuousFieldMaterialization { reason } => {
                write!(
                    f,
                    "invalid exact continuous-field materialization: {reason}"
                )
            }
        }
    }
}

impl std::error::Error for HypervoxelError {}
