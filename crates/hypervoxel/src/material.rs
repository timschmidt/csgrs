//! Material-region query reports.
//!
//! Material laws remain outside `hypervoxel`; cells carry compact
//! [`MaterialRegionId`] handles into side tables. These reports validate that a
//! grid's material references can be resolved before `hyperphysics`,
//! `hyperparts`, or fabrication code interprets the material. This is another
//! object-level fact boundary in the sense of Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997.

use std::collections::BTreeSet;

use std::collections::BTreeMap;

use crate::{
    LegacyAdapterKind, LegacyAdapterStatus, MaterialRegionId, SparseVoxelGrid, VoxelPayload,
    VoxelSideTables,
};

/// Material references observed in a sparse grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterialRegionQuery {
    /// Distinct material regions referenced by cells.
    pub referenced: BTreeSet<MaterialRegionId>,
    /// Referenced material regions missing from the side table.
    pub missing_records: BTreeSet<MaterialRegionId>,
}

impl MaterialRegionQuery {
    /// Returns whether every referenced material region has side-table metadata.
    pub fn is_fully_resolved(&self) -> bool {
        self.missing_records.is_empty()
    }
}

/// Queries material-region references over explicitly stored sparse cells.
pub fn query_material_regions(
    grid: &SparseVoxelGrid,
    side_tables: &VoxelSideTables,
) -> MaterialRegionQuery {
    let mut referenced = BTreeSet::new();
    let mut missing_records = BTreeSet::new();
    for (_, cell) in grid.iter() {
        if let VoxelPayload::MaterialRegion(region) = cell.payload {
            referenced.insert(region);
            if side_tables.material(region).is_none() {
                missing_records.insert(region);
            }
        }
    }
    MaterialRegionQuery {
        referenced,
        missing_records,
    }
}

/// Lossy display color for a material region.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MaterialDisplayColor {
    /// Red channel.
    pub r: u8,
    /// Green channel.
    pub g: u8,
    /// Blue channel.
    pub b: u8,
    /// Alpha channel.
    pub a: u8,
}

/// Material display palette for preview/export adapters.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MaterialDisplayPalette {
    colors: BTreeMap<MaterialRegionId, MaterialDisplayColor>,
}

impl MaterialDisplayPalette {
    /// Inserts a display color for a material region.
    pub fn insert(
        &mut self,
        id: MaterialRegionId,
        color: MaterialDisplayColor,
    ) -> Option<MaterialDisplayColor> {
        self.colors.insert(id, color)
    }

    /// Returns a display color for a material region.
    pub fn color(&self, id: MaterialRegionId) -> Option<MaterialDisplayColor> {
        self.colors.get(&id).copied()
    }
}

/// Report for lossy material color lookup.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterialColorLookupReport {
    /// Number of material regions referenced by the grid.
    pub referenced_regions: usize,
    /// Number of referenced material regions with display colors.
    pub resolved_colors: usize,
    /// Referenced material regions missing display colors.
    pub missing_colors: Vec<MaterialRegionId>,
    /// Explicit lossy adapter status.
    pub adapter: LegacyAdapterStatus,
}

/// Looks up preview colors for material regions without interpreting materials.
pub fn lookup_material_display_colors(
    query: &MaterialRegionQuery,
    palette: &MaterialDisplayPalette,
) -> MaterialColorLookupReport {
    let missing_colors = query
        .referenced
        .iter()
        .copied()
        .filter(|id| palette.color(*id).is_none())
        .collect::<Vec<_>>();
    MaterialColorLookupReport {
        referenced_regions: query.referenced.len(),
        resolved_colors: query.referenced.len() - missing_colors.len(),
        missing_colors,
        adapter: LegacyAdapterStatus::lossy(
            LegacyAdapterKind::GreedyMesh,
            "material display color lookup",
        ),
    }
}
