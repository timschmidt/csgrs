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
        Self {
            shape,
            page_count: pages.len(),
            stored_cells,
            pages,
        }
    }
}
