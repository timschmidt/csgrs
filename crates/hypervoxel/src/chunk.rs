//! Chunk and page metadata for exact voxel grids.
//!
//! Chunking is a storage/layout concern, not a geometric predicate. The types
//! here keep that distinction explicit: chunk IDs are integer partitions of
//! exact voxel addresses, while metric cell bounds still come from
//! [`crate::GridFrame`]. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997, by preserving the
//! object-level grid structure instead of deriving paging decisions from
//! approximate world coordinates.

use crate::{HypervoxelError, HypervoxelResult, VoxelAddress};

/// Power-of-two chunk shape in finest cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChunkShape {
    /// Base-2 log of the number of cells along each chunk axis.
    pub log2_cells: u8,
}

impl ChunkShape {
    /// Creates a chunk shape after validating it can fit in the address model.
    pub fn new(log2_cells: u8) -> HypervoxelResult<Self> {
        if log2_cells > crate::frame::MAX_ADDRESS_DEPTH {
            return Err(HypervoxelError::DepthTooLarge {
                depth: log2_cells,
                max_supported: crate::frame::MAX_ADDRESS_DEPTH,
            });
        }
        Ok(Self { log2_cells })
    }

    /// Returns the number of finest cells along one chunk axis.
    pub fn cells_per_axis(self) -> u64 {
        1_u64 << self.log2_cells
    }
}

/// Integer chunk coordinate at a specific grid depth.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ChunkAddress {
    /// Depth of addresses partitioned by this chunk address.
    pub depth: u8,
    /// Integer chunk coordinates.
    pub xyz: [u64; 3],
}

/// Exact chunk/local decomposition of a voxel address.
///
/// The decomposition is entirely integer-grid based: it never consults metric
/// world coordinates or primitive floats. This is the chunk-level counterpart
/// to Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997: storage layout may be optimized, but exact object identity
/// remains a replayable structural fact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChunkLocalAddress {
    /// Shape used for the split.
    pub shape: ChunkShape,
    /// Chunk address containing the voxel.
    pub chunk: ChunkAddress,
    /// Local coordinates inside the chunk.
    pub local_xyz: [u64; 3],
    /// Number of finest cells along one chunk axis at this address depth.
    pub local_extent: u64,
    /// Whether local coordinates are inside the chunk extent.
    pub local_in_bounds: bool,
    /// Whether recombining chunk and local coordinates reproduces the address.
    pub exact_recompose_ready: bool,
}

impl ChunkAddress {
    /// Computes the chunk address containing a voxel address.
    pub fn containing(address: VoxelAddress, shape: ChunkShape) -> Self {
        let shift = shape.log2_cells.min(address.depth);
        Self {
            depth: address.depth,
            xyz: [
                address.xyz[0] >> shift,
                address.xyz[1] >> shift,
                address.xyz[2] >> shift,
            ],
        }
    }

    /// Splits an exact voxel address into chunk and local integer coordinates.
    pub fn split(address: VoxelAddress, shape: ChunkShape) -> ChunkLocalAddress {
        let shift = shape.log2_cells.min(address.depth);
        let local_extent = 1_u64 << shift;
        let mask = local_extent - 1;
        let chunk = Self::containing(address, shape);
        let local_xyz = [
            address.xyz[0] & mask,
            address.xyz[1] & mask,
            address.xyz[2] & mask,
        ];
        let recomposed = [
            (chunk.xyz[0] << shift) | local_xyz[0],
            (chunk.xyz[1] << shift) | local_xyz[1],
            (chunk.xyz[2] << shift) | local_xyz[2],
        ];
        let local_in_bounds = local_xyz.iter().all(|coord| *coord < local_extent);
        ChunkLocalAddress {
            shape,
            chunk,
            local_xyz,
            local_extent,
            local_in_bounds,
            exact_recompose_ready: local_in_bounds && recomposed == address.xyz,
        }
    }
}

/// Deterministic chunk/page summary for a sparse grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPageSummary {
    /// Chunk shape used for partitioning.
    pub shape: ChunkShape,
    /// Number of occupied pages.
    pub page_count: usize,
    /// Number of explicitly stored cells included in the summary.
    pub stored_cells: usize,
    /// Whether at least one explicit stored cell contributed to the page summary.
    ///
    /// An empty address stream can be summarized exactly as empty, but it does
    /// not prove that a paging adapter preserved any voxel object identity.
    /// Keeping this evidence bit explicit follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997: exact
    /// replay claims should be grounded in retained object facts, not vacuous
    /// layout inequalities.
    pub has_stored_cells: bool,
    /// Whether page addresses were derived purely from exact integer voxel addresses.
    ///
    /// Chunk paging is not a geometric predicate. This flag records that the
    /// page summary is an exact integer partition, following Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997:
    /// storage layout facts stay separate from floating-world coordinates.
    pub exact_integer_partition: bool,
    /// Maximum number of finest cells represented by the occupied pages.
    pub page_capacity_cells: usize,
    /// Whether at least one stored address exists and every stored address is covered by an occupied page.
    pub exact_page_cover_ready: bool,
    /// Chunk addresses in deterministic order.
    pub pages: Vec<ChunkAddress>,
}

impl ChunkPageSummary {
    /// Builds a summary from explicit sparse-grid addresses.
    pub fn from_addresses(
        shape: ChunkShape,
        addresses: impl IntoIterator<Item = VoxelAddress>,
    ) -> Self {
        let mut pages = std::collections::BTreeSet::new();
        let mut stored_cells = 0_usize;
        for address in addresses {
            stored_cells += 1;
            pages.insert(ChunkAddress::containing(address, shape));
        }
        let pages = pages.into_iter().collect::<Vec<_>>();
        let cells_per_chunk_axis = shape.cells_per_axis() as usize;
        let page_capacity_cells =
            pages.len() * cells_per_chunk_axis * cells_per_chunk_axis * cells_per_chunk_axis;
        Self {
            shape,
            page_count: pages.len(),
            stored_cells,
            has_stored_cells: stored_cells > 0,
            exact_integer_partition: true,
            page_capacity_cells,
            exact_page_cover_ready: stored_cells > 0 && stored_cells <= page_capacity_cells,
            pages,
        }
    }
}
