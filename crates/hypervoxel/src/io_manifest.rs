//! Report-bearing voxel interchange manifests.
//!
//! Image stacks and voxel file formats are useful interoperability routes, but
//! their filenames, palettes, and byte layouts must not silently define metric
//! spacing, material meaning, or occupancy truth. These manifests make that
//! boundary explicit. Missing metadata remains unknown unless supplied by a
//! caller policy, following Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997.

use crate::{
    FreshnessStatus, GridSource, LegacyAdapterKind, LegacyAdapterStatus, LengthUnit,
    SideTableLinkStatus,
};

/// Image-stack container family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ImageStackContainer {
    /// ZIP archive of PNG slices.
    ZippedPng,
    /// Zstd-compressed QOI slice stack.
    ZstdQoi,
}

/// Common voxel interchange family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelInterchangeFormat {
    /// MagicaVoxel `.vox`.
    MagicaVoxel,
    /// BINVOX.
    Binvox,
    /// VDB/OpenVDB/NanoVDB family.
    Vdb,
    /// VTK/VTI image data.
    VtkImageData,
    /// NRRD.
    Nrrd,
    /// NIfTI.
    Nifti,
    /// DICOM-style slice set.
    DicomSlices,
    /// MHD/MHA.
    MetaImage,
    /// Raw volume plus sidecar metadata.
    RawWithSidecar,
    /// HDF5 volume.
    Hdf5,
    /// Zarr array.
    Zarr,
    /// N5 array.
    N5,
    /// VTM.
    Vtm,
}

/// Semantic channel mapping for image/volume adapters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelChannelMapping {
    /// Binary or alpha occupancy mask.
    OccupancyMask,
    /// Material-region label/palette channel.
    MaterialRegion,
    /// Scalar field-sample channel.
    FieldSample,
    /// Process-state label channel.
    ProcessState,
    /// Display-only color channel.
    DisplayOnly,
}

/// Slice naming policy for image-stack adapters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelSliceNaming {
    /// Slice filenames or keys are explicitly indexed.
    ExplicitIndex,
    /// Slice filenames are lexicographically ordered but not semantically indexed.
    Lexicographic,
    /// Slice naming is unknown or not applicable.
    Unknown,
}

/// Slice ordering policy for image-stack adapters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelSliceOrdering {
    /// Slices are ordered from low to high index along the slice axis.
    LowToHigh,
    /// Slices are ordered from high to low index along the slice axis.
    HighToLow,
    /// Slice ordering is unknown or not applicable.
    Unknown,
}

/// Voxel index convention for interchange adapters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelIndexConvention {
    /// Cell-centered voxel samples.
    CellCenter,
    /// Corner/lattice-node samples.
    NodeCorner,
    /// Index convention is unknown.
    Unknown,
}

/// Compression or archive status for an IO route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelIoCompression {
    /// No compression or archive layer.
    None,
    /// ZIP archive container.
    Zip,
    /// Zstd compression.
    Zstd,
    /// Format-native compression.
    Native,
    /// Compression is unknown.
    Unknown,
}

/// Metadata replay status for an interchange route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelIoMetadataStatus {
    /// Metric spacing, units, and payload mapping are explicit.
    ExactReplay,
    /// Metadata is partly missing and must remain unknown.
    Unknown,
}

/// Payload mapping status for an interchange route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelIoPayloadStatus {
    /// Payload values round-trip exactly.
    ExactReplay,
    /// Payload values are mapped by certified labels or scalar intervals.
    CertifiedMapping,
    /// Payload values lost precision or palette information.
    Lossy,
    /// Payload meaning is not fully known.
    Unknown,
}

/// Palette/label mapping status for display or material labels.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelIoPaletteStatus {
    /// Labels or palette entries are explicit.
    Explicit,
    /// Palette or label information was lost.
    Lost,
    /// Palette or label meaning is not known.
    Unknown,
}

/// Exact metadata known for an import/export route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelIoMetadata {
    /// Whether the voxel dimensions are explicit in index space.
    pub dimensions: Option<[u64; 3]>,
    /// Whether the file axis order is explicit as source-axis indices.
    pub axis_order: Option<[usize; 3]>,
    /// Whether the exact grid origin is supplied by metadata or caller policy.
    pub has_explicit_origin: bool,
    /// Whether metric spacing is present and explicit.
    pub has_explicit_spacing: bool,
    /// Whether units are present and explicit.
    pub units: Option<LengthUnit>,
    /// Whether payload/channel mapping is present and explicit.
    pub has_payload_mapping: bool,
    /// Whether palette/label mapping is present and explicit.
    pub has_label_mapping: bool,
    /// Whether missing slices or tiles have an explicit policy.
    pub has_missing_slice_policy: bool,
    /// Whether duplicate slices or tiles have an explicit policy.
    pub has_duplicate_slice_policy: bool,
    /// Slice naming policy, when the route is a slice stack.
    pub slice_naming: VoxelSliceNaming,
    /// Slice ordering policy, when the route is a slice stack.
    pub slice_ordering: VoxelSliceOrdering,
    /// Voxel index convention for interpreting samples.
    pub index_convention: VoxelIndexConvention,
    /// Compression/archive status for this route.
    pub compression: VoxelIoCompression,
}

impl VoxelIoMetadata {
    /// Returns whether this metadata is sufficient to replay exact grid meaning.
    pub fn is_exact_replay_metadata(&self) -> bool {
        self.dimensions.is_some()
            && self.axis_order_is_permutation()
            && self.has_explicit_origin
            && self.has_explicit_spacing
            && self.units.is_some()
            && self.has_payload_mapping
            && self.has_missing_slice_policy
            && self.has_duplicate_slice_policy
            && self.slice_naming != VoxelSliceNaming::Unknown
            && self.slice_ordering != VoxelSliceOrdering::Unknown
            && self.index_convention != VoxelIndexConvention::Unknown
            && self.compression != VoxelIoCompression::Unknown
    }

    /// Returns whether the declared axis order is a complete permutation.
    ///
    /// Axis order is a combinatorial grid fact, not a display hint. Following
    /// Yap's object-level exactness rule, an invalid or missing axis order must
    /// keep metadata unknown instead of being guessed from array layout.
    pub fn axis_order_is_permutation(&self) -> bool {
        matches!(self.axis_order, Some(order) if {
            let mut seen = [false; 3];
            order.iter().all(|axis| {
                if *axis >= 3 || seen[*axis] {
                    false
                } else {
                    seen[*axis] = true;
                    true
                }
            })
        })
    }
}

/// Manifest for a ZIP PNG or zstd QOI slice-stack route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImageStackManifest {
    /// Container family.
    pub container: ImageStackContainer,
    /// Number of slices declared.
    pub slices: usize,
    /// Number of channels declared.
    pub channels: usize,
    /// Bit depth per channel.
    pub bit_depth: u8,
    /// Semantic channel mappings.
    pub channel_mappings: Vec<VoxelChannelMapping>,
    /// Explicit metadata supplied by the route or caller policy.
    pub metadata: VoxelIoMetadata,
    /// Source artifact or imported volume version, when known.
    pub source: Option<GridSource>,
    /// Expected source version for exact replay, when known.
    pub expected_source: Option<GridSource>,
    /// Number of material/field/process side-table links required by payloads.
    pub required_side_table_links: usize,
    /// Number of required side-table links supplied with the import/export.
    pub supplied_side_table_links: usize,
}

/// Manifest for a common voxel interchange route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelInterchangeManifest {
    /// Format family.
    pub format: VoxelInterchangeFormat,
    /// Explicit metadata supplied by the route or caller policy.
    pub metadata: VoxelIoMetadata,
    /// Whether payload values round-trip exactly.
    pub payload_exact: bool,
    /// Whether payload mapping is certified by labels or intervals.
    pub certified_payload_mapping: bool,
    /// Whether payload precision or palette information is known to be lost.
    pub lost_payload_information: bool,
    /// Source artifact or imported volume version, when known.
    pub source: Option<GridSource>,
    /// Expected source version for exact replay, when known.
    pub expected_source: Option<GridSource>,
    /// Number of material/field/process side-table links required by payloads.
    pub required_side_table_links: usize,
    /// Number of required side-table links supplied with the import/export.
    pub supplied_side_table_links: usize,
}

/// Adapter report for image-stack or voxel-interchange routes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelIoReport {
    /// Freshness/exact replay status.
    pub freshness: FreshnessStatus,
    /// Number of unknown metadata fields.
    pub unknown_metadata_fields: usize,
    /// Number of channels without semantic mapping.
    pub unmapped_channels: usize,
    /// Explicit adapter status.
    pub adapter: LegacyAdapterStatus,
    /// Metadata replay status.
    pub metadata_status: VoxelIoMetadataStatus,
    /// Payload mapping status.
    pub payload_status: VoxelIoPayloadStatus,
    /// Palette/label status.
    pub palette_status: VoxelIoPaletteStatus,
    /// Freshness of the imported/exported source artifact.
    pub source_freshness: FreshnessStatus,
    /// Completeness of material/field/process side-table links.
    pub side_table_links: SideTableLinkStatus,
    /// Slice naming policy carried by metadata.
    pub slice_naming: VoxelSliceNaming,
    /// Slice ordering policy carried by metadata.
    pub slice_ordering: VoxelSliceOrdering,
    /// Voxel index convention carried by metadata.
    pub index_convention: VoxelIndexConvention,
    /// Compression/archive status carried by metadata.
    pub compression: VoxelIoCompression,
}

impl ImageStackManifest {
    /// Builds an adapter report without reading image bytes.
    pub fn report(&self) -> VoxelIoReport {
        let unknown_metadata_fields = unknown_metadata_fields(&self.metadata);
        let unmapped_channels = self.channels.saturating_sub(self.channel_mappings.len());
        let source_freshness =
            source_freshness(self.source.as_ref(), self.expected_source.as_ref());
        let side_table_links = side_table_links(
            self.required_side_table_links,
            self.supplied_side_table_links,
        );
        let exact = unknown_metadata_fields == 0
            && unmapped_channels == 0
            && source_freshness != FreshnessStatus::Stale
            && side_table_links != SideTableLinkStatus::Missing;
        let payload_status = if unmapped_channels > 0 || !self.metadata.has_payload_mapping {
            VoxelIoPayloadStatus::Unknown
        } else if self.bit_depth < 8 {
            VoxelIoPayloadStatus::Lossy
        } else {
            VoxelIoPayloadStatus::CertifiedMapping
        };
        VoxelIoReport {
            freshness: if exact {
                FreshnessStatus::Current
            } else {
                FreshnessStatus::Unknown
            },
            unknown_metadata_fields,
            unmapped_channels,
            adapter: adapter(exact, "image-stack manifest"),
            metadata_status: metadata_status(&self.metadata),
            payload_status,
            palette_status: palette_status(&self.metadata),
            source_freshness,
            side_table_links,
            slice_naming: self.metadata.slice_naming,
            slice_ordering: self.metadata.slice_ordering,
            index_convention: self.metadata.index_convention,
            compression: self.metadata.compression,
        }
    }
}

impl VoxelInterchangeManifest {
    /// Builds an adapter report without reading volume bytes.
    pub fn report(&self) -> VoxelIoReport {
        let unknown_metadata_fields = unknown_metadata_fields(&self.metadata);
        let source_freshness =
            source_freshness(self.source.as_ref(), self.expected_source.as_ref());
        let side_table_links = side_table_links(
            self.required_side_table_links,
            self.supplied_side_table_links,
        );
        let exact = unknown_metadata_fields == 0
            && self.payload_exact
            && source_freshness != FreshnessStatus::Stale
            && side_table_links != SideTableLinkStatus::Missing;
        let payload_status = if self.payload_exact {
            VoxelIoPayloadStatus::ExactReplay
        } else if self.lost_payload_information {
            VoxelIoPayloadStatus::Lossy
        } else if self.certified_payload_mapping {
            VoxelIoPayloadStatus::CertifiedMapping
        } else {
            VoxelIoPayloadStatus::Unknown
        };
        VoxelIoReport {
            freshness: if exact {
                FreshnessStatus::Current
            } else {
                FreshnessStatus::Unknown
            },
            unknown_metadata_fields,
            unmapped_channels: 0,
            adapter: adapter(exact, "voxel interchange manifest"),
            metadata_status: metadata_status(&self.metadata),
            payload_status,
            palette_status: palette_status(&self.metadata),
            source_freshness,
            side_table_links,
            slice_naming: self.metadata.slice_naming,
            slice_ordering: self.metadata.slice_ordering,
            index_convention: self.metadata.index_convention,
            compression: self.metadata.compression,
        }
    }
}

fn source_freshness(source: Option<&GridSource>, expected: Option<&GridSource>) -> FreshnessStatus {
    match (source, expected) {
        (Some(source), Some(expected)) if source == expected => FreshnessStatus::Current,
        (Some(_), Some(_)) => FreshnessStatus::Stale,
        _ => FreshnessStatus::Unknown,
    }
}

fn side_table_links(required: usize, supplied: usize) -> SideTableLinkStatus {
    if required == 0 {
        SideTableLinkStatus::NotRequired
    } else if supplied >= required {
        SideTableLinkStatus::Complete
    } else {
        SideTableLinkStatus::Missing
    }
}

fn unknown_metadata_fields(metadata: &VoxelIoMetadata) -> usize {
    usize::from(metadata.dimensions.is_none())
        + usize::from(!metadata.axis_order_is_permutation())
        + usize::from(!metadata.has_explicit_origin)
        + usize::from(!metadata.has_explicit_spacing)
        + usize::from(metadata.units.is_none())
        + usize::from(!metadata.has_payload_mapping)
        + usize::from(!metadata.has_missing_slice_policy)
        + usize::from(!metadata.has_duplicate_slice_policy)
        + usize::from(metadata.slice_naming == VoxelSliceNaming::Unknown)
        + usize::from(metadata.slice_ordering == VoxelSliceOrdering::Unknown)
        + usize::from(metadata.index_convention == VoxelIndexConvention::Unknown)
        + usize::from(metadata.compression == VoxelIoCompression::Unknown)
}

fn metadata_status(metadata: &VoxelIoMetadata) -> VoxelIoMetadataStatus {
    if metadata.is_exact_replay_metadata() {
        VoxelIoMetadataStatus::ExactReplay
    } else {
        VoxelIoMetadataStatus::Unknown
    }
}

fn palette_status(metadata: &VoxelIoMetadata) -> VoxelIoPaletteStatus {
    if metadata.has_label_mapping {
        VoxelIoPaletteStatus::Explicit
    } else if metadata.has_payload_mapping {
        VoxelIoPaletteStatus::Lost
    } else {
        VoxelIoPaletteStatus::Unknown
    }
}

fn adapter(exact: bool, policy: &'static str) -> LegacyAdapterStatus {
    if exact {
        LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, policy)
    } else {
        LegacyAdapterStatus::lossy(LegacyAdapterKind::ImportExport, policy)
    }
}
