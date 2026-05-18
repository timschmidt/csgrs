//! Exact-aware packing carriers and feasibility replay.
//!
//! `hyperpack` owns item/bin/sheet/container models, placements, free-space
//! reports, heuristic proposal metadata, exact lower-bound placeholders, and
//! feasibility replay. Heuristics such as shelf, skyline, MaxRects, guillotine,
//! extreme-point, DBLF, layer, and LAFF are proposal surfaces until their output
//! is replayed exactly.
//!
//! This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): combinatorial acceptance
//! is based on exact containment/no-overlap/support checks or explicit unknowns.

pub mod error;
pub mod model;
pub mod replay;

pub use error::{PackError, PackResult};
pub use hyperreal::Real;
pub use model::{
    AxisBox3, Bin3, ContainerFrame3, FreeSpaceReport3, HeuristicFamily, Item3, ItemId,
    LowerBoundReport, PackingReport3, Placement3,
};
pub use replay::{FeasibilityReplay3, FeasibilityStatus};
