//! Material-region query reports.
//!
//! Material laws remain outside `hypervoxel`; cells carry compact
//! [`MaterialRegionId`] handles into side tables. These reports validate that a
//! grid's material references can be resolved before `hyperphysics`,
//! `hyperparts`, or fabrication code interprets the material.

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
    /// Returns whether any material region was referenced.
    pub fn has_references(&self) -> bool {
        !self.referenced.is_empty()
    }

    /// Returns whether every referenced material region has side-table metadata.
    pub fn is_fully_resolved(&self) -> bool {
        self.has_references() && self.missing_records.is_empty()
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
    /// Whether at least one material region was referenced.
    ///
    /// A preview palette with no referenced material regions is vacuous. It is
    /// not a complete material-display mapping for an artifact.
    pub has_material_regions: bool,
    /// Number of referenced material regions with display colors.
    pub resolved_colors: usize,
    /// Referenced material regions missing display colors.
    pub missing_colors: Vec<MaterialRegionId>,
    /// Whether every referenced material region has an explicit display color.
    ///
    /// Display colors are still preview adapter data, not physical material
    /// laws. This flag only certifies palette completeness for visualization
    /// lookup. Missing display data remains explicit rather than being filled
    /// by a default color that could hide material-region distinctions.
    pub complete_display_palette_ready: bool,
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
        has_material_regions: query.has_references(),
        resolved_colors: query.referenced.len() - missing_colors.len(),
        complete_display_palette_ready: query.has_references() && missing_colors.is_empty(),
        missing_colors,
        adapter: LegacyAdapterStatus::new(LegacyAdapterKind::GreedyMesh),
    }
}
