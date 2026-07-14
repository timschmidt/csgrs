//! Packing models, placements, and proposal reports.

use hyperreal::{Real, RealSign};

use crate::{PackError, PackResult};

/// Stable item id.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ItemId(String);

/// Axis-aligned exact box dimensions.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisBox3 {
    /// Width along x.
    pub x: Real,
    /// Depth along y.
    pub y: Real,
    /// Height along z.
    pub z: Real,
}

/// Packable item.
#[derive(Clone, Debug, PartialEq)]
pub struct Item3 {
    /// Item id.
    pub id: ItemId,
    /// Exact axis-aligned dimensions.
    pub size: AxisBox3,
}

/// Exact bin/sheet/container volume.
#[derive(Clone, Debug, PartialEq)]
pub struct Bin3 {
    /// Bin size.
    pub size: AxisBox3,
}

/// Coordinate frame/provenance for a container.
#[derive(Clone, Debug, PartialEq)]
pub struct ContainerFrame3 {
    /// Frame name.
    pub name: String,
    /// Exact grid or unit statement.
    pub unit: String,
}

/// Item placement at the lower-left-bottom origin.
#[derive(Clone, Debug, PartialEq)]
pub struct Placement3 {
    /// Placed item id.
    pub item: ItemId,
    /// x origin.
    pub x: Real,
    /// y origin.
    pub y: Real,
    /// z origin.
    pub z: Real,
}

/// Heuristic family that proposed a packing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HeuristicFamily {
    /// 2D shelf proposal.
    Shelf2,
    /// 2D skyline proposal.
    Skyline2,
    /// 2D MaxRects proposal.
    MaxRects2,
    /// 2D guillotine proposal.
    Guillotine2,
    /// 3D extreme-point proposal.
    ExtremePoint3,
    /// 3D maximal-space proposal.
    MaximalSpace3,
    /// 3D deepest-bottom-left-fill proposal.
    Dblf3,
    /// 3D layer proposal.
    Layer3,
    /// 3D LAFF proposal.
    Laff3,
}

/// Retained free-space summary after proposal or replay.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeSpaceReport3 {
    /// Free boxes retained by a heuristic or exact replay.
    pub boxes: Vec<AxisBox3>,
    /// Whether free-space coverage is exact.
    pub exact: bool,
}

/// Lower-bound or optimality-gap evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct LowerBoundReport {
    /// Exact lower bound on used bins or objective value.
    pub lower_bound: Real,
    /// Optional exact incumbent objective.
    pub incumbent: Option<Real>,
    /// Human-readable method label.
    pub method: String,
}

/// Proposal and replay report for a packing.
#[derive(Clone, Debug, PartialEq)]
pub struct PackingReport3 {
    /// Proposal heuristic family.
    pub heuristic: HeuristicFamily,
    /// Deterministic multistart seed.
    pub seed: u64,
    /// Placements proposed by the heuristic.
    pub placements: Vec<Placement3>,
    /// Free-space evidence.
    pub free_space: FreeSpaceReport3,
    /// Lower-bound evidence.
    pub lower_bound: Option<LowerBoundReport>,
}

impl ItemId {
    /// Creates a non-empty item id.
    pub fn new(value: impl Into<String>) -> PackResult<Self> {
        let value = value.into();
        if value.is_empty() {
            return Err(PackError::EmptyIdentifier);
        }
        Ok(Self(value))
    }

    /// Returns the id text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl AxisBox3 {
    /// Creates positive exact dimensions.
    pub fn new(x: Real, y: Real, z: Real) -> PackResult<Self> {
        for value in [&x, &y, &z] {
            match value.refine_sign_until(-64) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Negative | RealSign::Zero) | None => {
                    return Err(PackError::NonPositiveDimension);
                }
            }
        }
        Ok(Self { x, y, z })
    }

    /// Exact volume.
    pub fn volume(&self) -> Real {
        &self.x * &self.y * &self.z
    }
}
