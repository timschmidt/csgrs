//! Exact 3D cuboid orientation policies.
//!
//! Axis-aligned cuboid rotations are represented as the six permutations of
//! `(x, y, z)`. This avoids approximate rotation matrices. Geometric state
//! changes are accepted only after exact predicates or explicit validation
//! reports.

use std::collections::BTreeMap;

use hyperreal::Real;

use crate::{
    AxisBox3, Bin3, FeasibilityStatus, Item3, ItemId, PackError, PackResult, PackingVerification3,
    Placement3, verify_packing_3d,
};

/// Six exact axis permutations for an axis-aligned cuboid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Orientation3 {
    /// `(x, y, z)`.
    Xyz,
    /// `(x, z, y)`.
    Xzy,
    /// `(y, x, z)`.
    Yxz,
    /// `(y, z, x)`.
    Yzx,
    /// `(z, x, y)`.
    Zxy,
    /// `(z, y, x)`.
    Zyx,
}

/// Exact 3D item with an explicit orientation policy.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedItem3 {
    /// Item id.
    pub id: ItemId,
    /// Source item size before orientation is applied.
    pub size: AxisBox3,
    /// Orientations this item may legally use.
    pub allowed_orientations: Vec<Orientation3>,
    /// Source unit/provenance label for validation reports.
    pub source_unit: String,
}

/// Placement of an oriented 3D item.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedPlacement3 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact x coordinate.
    pub x: Real,
    /// Exact y coordinate.
    pub y: Real,
    /// Exact z coordinate.
    pub z: Real,
    /// Orientation used by this placement.
    pub orientation: Orientation3,
}

/// Validation report for oriented 3D inputs and placements.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientationValidationReport3 {
    /// Number of oriented placements checked.
    pub checked_placements: usize,
    /// Number of oriented item policies checked.
    pub checked_items: usize,
    /// Item ids with empty orientation policies.
    pub empty_orientation_items: Vec<ItemId>,
    /// Placement item ids using an orientation not allowed by the item policy.
    pub illegal_orientation_items: Vec<ItemId>,
    /// Human-readable exact validation facts.
    pub facts: Vec<String>,
}

/// Full oriented one-bin 3D verification report.
#[derive(Clone, Debug, PartialEq)]
pub struct OrientedPackingVerification3 {
    /// Orientation-policy validation facts.
    pub orientation: OrientationValidationReport3,
    /// Exact 3D replay after legal axis permutation.
    pub packing: PackingVerification3,
}

impl OrientedItem3 {
    /// Creates an oriented cuboid item policy.
    pub fn new(
        id: ItemId,
        size: AxisBox3,
        allowed_orientations: Vec<Orientation3>,
        source_unit: impl Into<String>,
    ) -> Self {
        Self {
            id,
            size,
            allowed_orientations,
            source_unit: source_unit.into(),
        }
    }
}

impl Orientation3 {
    /// Applies this orientation exactly by permuting dimensions.
    pub fn apply(self, size: &AxisBox3) -> AxisBox3 {
        match self {
            Self::Xyz => size.clone(),
            Self::Xzy => AxisBox3 {
                x: size.x.clone(),
                y: size.z.clone(),
                z: size.y.clone(),
            },
            Self::Yxz => AxisBox3 {
                x: size.y.clone(),
                y: size.x.clone(),
                z: size.z.clone(),
            },
            Self::Yzx => AxisBox3 {
                x: size.y.clone(),
                y: size.z.clone(),
                z: size.x.clone(),
            },
            Self::Zxy => AxisBox3 {
                x: size.z.clone(),
                y: size.x.clone(),
                z: size.y.clone(),
            },
            Self::Zyx => AxisBox3 {
                x: size.z.clone(),
                y: size.y.clone(),
                z: size.x.clone(),
            },
        }
    }
}

/// Verifies a 3D packing with explicit axis-permutation orientation policies.
///
/// Illegal orientation use is reported as infeasible and is never corrected by
/// silently swapping dimensions. Legal orientations lower to ordinary
/// axis-aligned cuboids and then pass through exact containment/no-overlap
/// replay.
pub fn verify_oriented_packing_3d(
    bin: &Bin3,
    items: &[OrientedItem3],
    placements: &[OrientedPlacement3],
) -> PackResult<OrientedPackingVerification3> {
    let item_map = items
        .iter()
        .map(|item| (item.id.clone(), item))
        .collect::<BTreeMap<ItemId, &OrientedItem3>>();
    let mut orientation = OrientationValidationReport3 {
        checked_placements: 0,
        checked_items: items.len(),
        empty_orientation_items: Vec::new(),
        illegal_orientation_items: Vec::new(),
        facts: Vec::new(),
    };

    for item in items {
        if item.allowed_orientations.is_empty() {
            orientation.empty_orientation_items.push(item.id.clone());
            orientation
                .facts
                .push(format!("{} has no allowed orientations", item.id.as_str()));
        }
    }

    let mut replay_items = BTreeMap::<ItemId, Item3>::new();
    let mut replay_placements = Vec::with_capacity(placements.len());
    for placement in placements {
        orientation.checked_placements += 1;
        let item = item_map
            .get(&placement.item)
            .ok_or(PackError::MissingItem)?;
        if !item.allowed_orientations.contains(&placement.orientation) {
            orientation
                .illegal_orientation_items
                .push(placement.item.clone());
            orientation.facts.push(format!(
                "{} uses disallowed orientation {:?}",
                placement.item.as_str(),
                placement.orientation
            ));
        }
        replay_items
            .entry(item.id.clone())
            .or_insert_with(|| Item3 {
                id: item.id.clone(),
                size: placement.orientation.apply(&item.size),
            });
        replay_placements.push(Placement3 {
            item: placement.item.clone(),
            x: placement.x.clone(),
            y: placement.y.clone(),
            z: placement.z.clone(),
        });
    }
    for item in items {
        replay_items
            .entry(item.id.clone())
            .or_insert_with(|| Item3 {
                id: item.id.clone(),
                size: item.size.clone(),
            });
    }

    let mut packing = verify_packing_3d(
        bin,
        &replay_items.into_values().collect::<Vec<_>>(),
        &replay_placements,
    )?;
    if !orientation.empty_orientation_items.is_empty()
        || !orientation.illegal_orientation_items.is_empty()
    {
        packing.feasibility.status = FeasibilityStatus::Infeasible;
        packing
            .feasibility
            .facts
            .extend(orientation.facts.iter().cloned());
    }

    Ok(OrientedPackingVerification3 {
        orientation,
        packing,
    })
}
