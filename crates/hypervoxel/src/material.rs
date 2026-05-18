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
    AggregateCertainty, LegacyAdapterKind, LegacyAdapterStatus, MaterialRegionId, SparseVoxelGrid,
    VoxelPayload, VoxelSideTables,
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

/// Metadata audit for material regions referenced by a grid.
///
/// Hypervoxel does not own material laws. This report is the voxel-side
/// contract that a downstream material/physics crate can inspect before it
/// interprets density, composition, elasticity, conductivity, optical constants,
/// or fabrication state. Following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997, pp. 3-23, the grid preserves
/// object-level evidence and explicit unknowns instead of guessing missing
/// material facts from payload IDs or display labels.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterialRegionMetadataReport {
    /// Number of distinct material regions referenced by cells.
    pub referenced_regions: usize,
    /// Whether at least one material region was referenced.
    ///
    /// Absence of material regions is useful diagnostic information, but it is
    /// not complete material evidence. Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997, requires exact
    /// object facts to remain tied to the object evidence that produced them.
    pub has_material_regions: bool,
    /// Number of referenced regions with side-table records.
    pub resolved_records: usize,
    /// Referenced material regions missing from the side table.
    pub missing_records: BTreeSet<MaterialRegionId>,
    /// Referenced material regions with exact density metadata.
    pub records_with_density: BTreeSet<MaterialRegionId>,
    /// Referenced material regions whose records omit density.
    pub records_missing_density: BTreeSet<MaterialRegionId>,
    /// Referenced material regions whose records have an empty label.
    pub empty_labels: BTreeSet<MaterialRegionId>,
    /// Referenced material regions whose records have empty provenance.
    pub empty_provenance: BTreeSet<MaterialRegionId>,
    /// Conservative certainty of the material metadata carried by hypervoxel.
    pub certainty: AggregateCertainty,
}

impl MaterialRegionMetadataReport {
    /// Returns whether every referenced material region has complete metadata
    /// for the fields that hypervoxel is allowed to audit.
    pub fn is_complete(&self) -> bool {
        self.has_material_regions
            && self.missing_records.is_empty()
            && self.records_missing_density.is_empty()
            && self.empty_labels.is_empty()
            && self.empty_provenance.is_empty()
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

/// Audits material side-table metadata for regions referenced by a grid.
pub fn report_material_region_metadata(
    query: &MaterialRegionQuery,
    side_tables: &VoxelSideTables,
) -> MaterialRegionMetadataReport {
    let mut records_with_density = BTreeSet::new();
    let mut records_missing_density = BTreeSet::new();
    let mut empty_labels = BTreeSet::new();
    let mut empty_provenance = BTreeSet::new();

    for id in query
        .referenced
        .iter()
        .filter(|id| !query.missing_records.contains(id))
    {
        let record = side_tables
            .material(*id)
            .expect("query marked this material as resolved");
        if record.density.is_some() {
            records_with_density.insert(*id);
        } else {
            records_missing_density.insert(*id);
        }
        if record.label.is_empty() {
            empty_labels.insert(*id);
        }
        if record.provenance.is_empty() {
            empty_provenance.insert(*id);
        }
    }

    let resolved_records = query.referenced.len() - query.missing_records.len();
    let has_material_regions = !query.referenced.is_empty();
    let certainty = if has_material_regions
        && query.missing_records.is_empty()
        && records_missing_density.is_empty()
        && empty_labels.is_empty()
        && empty_provenance.is_empty()
    {
        AggregateCertainty::Exact
    } else {
        AggregateCertainty::Unknown
    };

    MaterialRegionMetadataReport {
        referenced_regions: query.referenced.len(),
        has_material_regions,
        resolved_records,
        missing_records: query.missing_records.clone(),
        records_with_density,
        records_missing_density,
        empty_labels,
        empty_provenance,
        certainty,
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
    /// lookup. Following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, missing display data remains an
    /// explicit report fact instead of being filled by a default color that
    /// could hide material-region distinctions.
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
        adapter: LegacyAdapterStatus::lossy(
            LegacyAdapterKind::GreedyMesh,
            "material display color lookup",
        ),
    }
}
