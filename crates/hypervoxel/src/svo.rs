//! Hyper-owned sparse voxel octree DAG storage.
//!
//! `voxelis` remains the performance seed for a production SVO-DAG backend.
//! This module establishes the exact semantic contract first: nodes are
//! interned by value, edits path-copy through integer addresses, and every
//! branch carries conservative aggregate facts. Geometric objects preserve
//! facts rather than reducing every decision to scalar approximation.

use rustc_hash::FxHashMap;

use crate::{
    AggregateCertainty, GridFrame, HypervoxelError, HypervoxelResult, MaterialRegionId,
    OccupancyState, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelOccupancyInterval, VoxelPayload,
};

/// Interned SVO node identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SvoNodeId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum SvoNodeKey {
    Leaf {
        cell: VoxelCell,
        remaining_depth: u8,
    },
    Branch([SvoNodeId; 8]),
}

const AGGREGATE_HAS_BOUNDARY: u8 = 1 << 0;
const AGGREGATE_HAS_MIXED: u8 = 1 << 1;
const AGGREGATE_HAS_UNKNOWN: u8 = 1 << 2;
const AGGREGATE_HAS_LOSSY: u8 = 1 << 3;

/// Sorted material-region summary optimized for the overwhelmingly common
/// zero- and one-material cases. A full set is allocated only when a subtree
/// actually crosses material regions.
#[derive(Clone, Debug, PartialEq, Eq)]
enum SvoMaterialRegions {
    None,
    One(MaterialRegionId),
    Many(Box<[MaterialRegionId]>),
}

impl SvoMaterialRegions {
    fn len(&self) -> usize {
        match self {
            Self::None => 0,
            Self::One(_) => 1,
            Self::Many(regions) => regions.len(),
        }
    }

    fn insert_into(&self, regions: &mut Vec<MaterialRegionId>) {
        let source: &[MaterialRegionId] = match self {
            Self::None => &[],
            Self::One(region) => std::slice::from_ref(region),
            Self::Many(regions) => regions,
        };
        for region in source {
            if let Err(index) = regions.binary_search(region) {
                regions.insert(index, *region);
            }
        }
    }

    fn from_sorted(regions: Vec<MaterialRegionId>) -> Self {
        match regions.as_slice() {
            [] => Self::None,
            [region] => Self::One(*region),
            _ => Self::Many(regions.into_boxed_slice()),
        }
    }

    fn to_set(&self) -> std::collections::BTreeSet<MaterialRegionId> {
        match self {
            Self::None => std::collections::BTreeSet::new(),
            Self::One(region) => std::iter::once(*region).collect(),
            Self::Many(regions) => regions.iter().copied().collect(),
        }
    }
}

/// Compact exact aggregate stored beside each physical DAG node.
///
/// Public aggregate packets contain two materialized [`hyperreal::Real`]
/// bounds and a `BTreeSet`. Those are appropriate at API boundaries, but
/// storing them on every node dominates the octree payload. The SVO keeps the
/// sufficient integer statistics and flags instead and materializes the rich
/// packet once for the current root.
#[derive(Clone, Debug, PartialEq, Eq)]
struct SvoAggregate {
    definite_filled_cells: usize,
    possible_occupied_cells: usize,
    material_regions: SvoMaterialRegions,
    flags: u8,
    remaining_depth: u8,
}

impl SvoAggregate {
    fn from_uniform_cell(remaining_depth: u8, cell: &VoxelCell) -> Self {
        let total_cells = logical_leaf_cells(remaining_depth);
        let mut flags = 0;
        flags |= u8::from(cell.occupancy == OccupancyState::Boundary) * AGGREGATE_HAS_BOUNDARY;
        flags |= u8::from(cell.occupancy == OccupancyState::Mixed) * AGGREGATE_HAS_MIXED;
        flags |= u8::from(cell.occupancy == OccupancyState::Unknown) * AGGREGATE_HAS_UNKNOWN;
        flags |= u8::from(
            cell.occupancy == OccupancyState::LossyAdapterValue
                || matches!(cell.payload, VoxelPayload::LossyAdapterValue(_)),
        ) * AGGREGATE_HAS_LOSSY;
        let material_regions = match cell.payload {
            VoxelPayload::MaterialRegion(region) => SvoMaterialRegions::One(region),
            _ => SvoMaterialRegions::None,
        };
        let definite_filled_cells =
            usize::from(cell.occupancy == OccupancyState::Filled) * total_cells;
        let possible_occupied_cells = usize::from(matches!(
            cell.occupancy,
            OccupancyState::Filled
                | OccupancyState::Boundary
                | OccupancyState::Mixed
                | OccupancyState::Unknown
                | OccupancyState::LossyAdapterValue
        )) * total_cells;
        Self {
            definite_filled_cells,
            possible_occupied_cells,
            material_regions,
            flags,
            remaining_depth,
        }
    }

    fn from_aggregates<'a>(facts: impl IntoIterator<Item = &'a Self>, remaining_depth: u8) -> Self {
        let mut definite_filled_cells = 0;
        let mut possible_occupied_cells = 0;
        let mut material_regions = Vec::new();
        let mut flags = 0;

        for fact in facts {
            debug_assert_eq!(fact.remaining_depth + 1, remaining_depth);
            definite_filled_cells += fact.definite_filled_cells;
            possible_occupied_cells += fact.possible_occupied_cells;
            fact.material_regions.insert_into(&mut material_regions);
            flags |= fact.flags;
            if fact.has_mixed() || !(fact.all_empty() || fact.all_filled()) {
                flags |= AGGREGATE_HAS_MIXED;
            }
        }

        Self {
            definite_filled_cells,
            possible_occupied_cells,
            material_regions: SvoMaterialRegions::from_sorted(material_regions),
            flags,
            remaining_depth,
        }
    }

    fn child_count(&self) -> usize {
        logical_leaf_cells(self.remaining_depth)
    }

    fn all_empty(&self) -> bool {
        self.possible_occupied_cells == 0
    }

    fn all_filled(&self) -> bool {
        self.definite_filled_cells == self.child_count()
    }

    fn has_mixed(&self) -> bool {
        self.flags & AGGREGATE_HAS_MIXED != 0
    }

    fn conservative_occupancy(&self) -> OccupancyState {
        if self.flags & AGGREGATE_HAS_LOSSY != 0 {
            OccupancyState::LossyAdapterValue
        } else if self.flags & AGGREGATE_HAS_UNKNOWN != 0 {
            OccupancyState::Unknown
        } else if self.all_empty() {
            OccupancyState::Empty
        } else if self.all_filled() && self.material_regions.len() <= 1 {
            OccupancyState::Filled
        } else if self.flags & AGGREGATE_HAS_BOUNDARY != 0 {
            OccupancyState::Boundary
        } else {
            OccupancyState::Mixed
        }
    }

    fn to_public(&self) -> VoxelAggregateFacts {
        let child_count = self.child_count();
        let certainty = aggregate_certainty(child_count, self.flags);
        VoxelAggregateFacts {
            child_count,
            all_empty: self.all_empty(),
            all_filled: self.all_filled(),
            has_boundary: self.flags & AGGREGATE_HAS_BOUNDARY != 0,
            has_mixed: self.has_mixed(),
            has_unknown: self.flags & AGGREGATE_HAS_UNKNOWN != 0,
            has_lossy: self.flags & AGGREGATE_HAS_LOSSY != 0,
            material_regions: self.material_regions.to_set(),
            occupancy_interval: VoxelOccupancyInterval::from_counts(
                child_count,
                self.definite_filled_cells,
                self.possible_occupied_cells,
                certainty,
            ),
            certainty,
        }
    }
}

fn aggregate_certainty(child_count: usize, flags: u8) -> AggregateCertainty {
    if child_count == 0 {
        AggregateCertainty::Unknown
    } else if flags & AGGREGATE_HAS_LOSSY != 0 {
        AggregateCertainty::Lossy
    } else if flags & AGGREGATE_HAS_UNKNOWN != 0 {
        AggregateCertainty::Unknown
    } else if flags & (AGGREGATE_HAS_BOUNDARY | AGGREGATE_HAS_MIXED) != 0 {
        AggregateCertainty::Certified
    } else {
        AggregateCertainty::Exact
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum SvoNode {
    Leaf {
        cell: VoxelCell,
        aggregate: SvoAggregate,
    },
    Branch {
        children: [SvoNodeId; 8],
        aggregate: SvoAggregate,
    },
}

impl SvoNode {
    fn aggregate(&self) -> &SvoAggregate {
        match self {
            Self::Leaf { aggregate, .. } | Self::Branch { aggregate, .. } => aggregate,
        }
    }
}

/// Storage statistics for the semantic SVO-DAG backend.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SvoDagStats {
    /// Number of unique interned nodes.
    pub nodes: usize,
    /// Number of unique leaves.
    pub leaves: usize,
    /// Number of unique branches.
    pub branches: usize,
}

/// Exact semantic sparse voxel octree with DAG interning.
#[derive(Clone, Debug, PartialEq)]
pub struct SvoVoxelGrid {
    frame: GridFrame,
    nodes: Vec<SvoNode>,
    interned: FxHashMap<SvoNodeKey, SvoNodeId>,
    root: SvoNodeId,
    root_aggregate: VoxelAggregateFacts,
}

impl SvoVoxelGrid {
    /// Creates an empty interned SVO grid.
    pub fn new(frame: GridFrame) -> Self {
        let frame_depth = frame.depth();
        let empty_cell = VoxelCell::empty();
        let mut grid = Self {
            frame,
            nodes: Vec::new(),
            interned: FxHashMap::default(),
            root: SvoNodeId(0),
            root_aggregate: VoxelAggregateFacts::from_uniform_cell(
                logical_leaf_cells(frame_depth),
                &empty_cell,
            ),
        };
        let root = grid.intern_leaf(empty_cell, frame_depth);
        grid.root = root;
        grid
    }

    /// Compacts sparse-grid storage into an interned SVO-DAG.
    pub fn from_sparse_grid(source: &SparseVoxelGrid) -> HypervoxelResult<Self> {
        let mut grid = Self::new(source.frame().clone());
        let mut non_finest_depth_cells = 0_usize;
        let mut entries = Vec::with_capacity(source.len());

        for (address, cell) in source.iter() {
            entries.push((*address, *cell));
            if address.depth != source.frame().depth() {
                non_finest_depth_cells += 1;
            }
        }

        if non_finest_depth_cells == 0 {
            entries.sort_unstable_by_key(|(address, _)| address.morton_code());
            grid.root = grid.build_finest_sparse_subtree(&entries, 0);
            grid.refresh_root_aggregate();
        } else {
            for (address, cell) in entries {
                grid.set(address, cell)?;
            }
        }
        Ok(grid)
    }

    /// Returns the grid frame.
    pub fn frame(&self) -> &GridFrame {
        &self.frame
    }

    /// Returns the root node id.
    pub fn root(&self) -> SvoNodeId {
        self.root
    }

    /// Returns root aggregate facts.
    pub fn aggregate(&self) -> &VoxelAggregateFacts {
        &self.root_aggregate
    }

    /// Bytes occupied by each node in the contiguous SVO node array, excluding
    /// the interning table and rare multi-material side allocations.
    pub const fn node_storage_stride_bytes() -> usize {
        std::mem::size_of::<SvoNode>()
    }

    /// Bytes occupied by the live contiguous SVO node array, excluding spare
    /// vector capacity, the interning table, and rare multi-material side
    /// allocations.
    pub fn node_storage_bytes(&self) -> usize {
        self.nodes.len() * Self::node_storage_stride_bytes()
    }

    /// Returns storage statistics.
    pub fn stats(&self) -> SvoDagStats {
        let mut stats = SvoDagStats {
            nodes: self.nodes.len(),
            ..SvoDagStats::default()
        };
        for node in &self.nodes {
            match node {
                SvoNode::Leaf { .. } => stats.leaves += 1,
                SvoNode::Branch { .. } => stats.branches += 1,
            }
        }
        stats
    }

    /// Expands this SVO-DAG into canonical sparse-grid storage.
    pub fn to_sparse_grid(&self) -> HypervoxelResult<SparseVoxelGrid> {
        let mut sparse = SparseVoxelGrid::new(self.frame.clone());
        self.replay_node_to_sparse(self.root, [0, 0, 0], self.frame.depth(), &mut sparse)?;
        Ok(sparse)
    }

    /// Reads a cell from the SVO, returning exact empty for collapsed empty
    /// subtrees.
    pub fn get(&self, address: VoxelAddress) -> HypervoxelResult<VoxelCell> {
        self.validate_address(address)?;
        let mut node_id = self.root;
        let mut current_depth = 0;
        loop {
            match self.node(node_id) {
                SvoNode::Leaf { cell, .. } => return Ok(*cell),
                SvoNode::Branch { children, .. } => {
                    if current_depth == address.depth {
                        let occupancy = self.node(node_id).aggregate().conservative_occupancy();
                        return Ok(VoxelCell {
                            occupancy,
                            payload: crate::VoxelPayload::Occupancy(occupancy),
                        });
                    }
                    let child = child_index(address, current_depth);
                    node_id = children[child as usize];
                    current_depth += 1;
                }
            }
        }
    }

    /// Sets a cell by path-copying and interning changed nodes.
    pub fn set(&mut self, address: VoxelAddress, cell: VoxelCell) -> HypervoxelResult<()> {
        self.validate_address(address)?;
        self.root = self.set_recursive(self.root, address, 0, cell)?;
        self.refresh_root_aggregate();
        Ok(())
    }

    fn validate_address(&self, address: VoxelAddress) -> HypervoxelResult<()> {
        if address.depth > self.frame.depth() {
            return Err(HypervoxelError::DepthOutsideFrame {
                depth: address.depth,
                frame_depth: self.frame.depth(),
            });
        }
        Ok(())
    }

    fn node(&self, id: SvoNodeId) -> &SvoNode {
        &self.nodes[id.0 as usize]
    }

    fn refresh_root_aggregate(&mut self) {
        self.root_aggregate = self.node(self.root).aggregate().to_public();
    }

    fn intern_leaf(&mut self, cell: VoxelCell, remaining_depth: u8) -> SvoNodeId {
        let key = SvoNodeKey::Leaf {
            cell,
            remaining_depth,
        };
        if let Some(id) = self.interned.get(&key) {
            return *id;
        }
        let id = SvoNodeId(self.nodes.len() as u32);
        let aggregate = SvoAggregate::from_uniform_cell(remaining_depth, &cell);
        self.nodes.push(SvoNode::Leaf { cell, aggregate });
        self.interned.insert(key, id);
        id
    }

    fn intern_branch(&mut self, children: [SvoNodeId; 8], remaining_depth: u8) -> SvoNodeId {
        if children.iter().all(|child| *child == children[0])
            && let SvoNode::Leaf { cell, .. } = self.node(children[0])
        {
            return self.intern_leaf(*cell, remaining_depth);
        }

        let key = SvoNodeKey::Branch(children);
        if let Some(id) = self.interned.get(&key) {
            return *id;
        }
        let id = SvoNodeId(self.nodes.len() as u32);
        let aggregate = SvoAggregate::from_aggregates(
            children.iter().map(|child| self.node(*child).aggregate()),
            remaining_depth,
        );
        self.nodes.push(SvoNode::Branch {
            children,
            aggregate,
        });
        self.interned.insert(key, id);
        id
    }

    fn build_finest_sparse_subtree(
        &mut self,
        entries: &[(VoxelAddress, VoxelCell)],
        current_depth: u8,
    ) -> SvoNodeId {
        let remaining_depth = self.frame.depth() - current_depth;
        if entries.is_empty() {
            return self.intern_leaf(VoxelCell::empty(), remaining_depth);
        }
        if remaining_depth == 0 {
            debug_assert_eq!(entries.len(), 1);
            return self.intern_leaf(entries[0].1, 0);
        }

        let mut boundaries = [0_usize; 9];
        let mut cursor = 0_usize;
        for child in 0..8_u8 {
            boundaries[usize::from(child)] = cursor;
            while cursor < entries.len() && child_index(entries[cursor].0, current_depth) == child {
                cursor += 1;
            }
        }
        boundaries[8] = cursor;
        debug_assert_eq!(cursor, entries.len());
        let mut children = [SvoNodeId(0); 8];
        for child in 0..8 {
            children[child] = self.build_finest_sparse_subtree(
                &entries[boundaries[child]..boundaries[child + 1]],
                current_depth + 1,
            );
        }
        self.intern_branch(children, remaining_depth)
    }

    fn set_recursive(
        &mut self,
        node_id: SvoNodeId,
        address: VoxelAddress,
        current_depth: u8,
        cell: VoxelCell,
    ) -> HypervoxelResult<SvoNodeId> {
        if current_depth == address.depth {
            return Ok(self.intern_leaf(cell, self.frame.depth() - current_depth));
        }

        let remaining_child_depth = self.frame.depth() - current_depth - 1;
        let mut children = match self.node(node_id) {
            SvoNode::Leaf { cell, .. } => {
                let cell = *cell;
                [self.intern_leaf(cell, remaining_child_depth); 8]
            }
            SvoNode::Branch { children, .. } => *children,
        };
        let child = child_index(address, current_depth);
        children[child as usize] =
            self.set_recursive(children[child as usize], address, current_depth + 1, cell)?;
        Ok(self.intern_branch(children, self.frame.depth() - current_depth))
    }

    fn replay_node_to_sparse(
        &self,
        node_id: SvoNodeId,
        origin: [u64; 3],
        remaining_depth: u8,
        sparse: &mut SparseVoxelGrid,
    ) -> HypervoxelResult<()> {
        match self.node(node_id) {
            SvoNode::Leaf { cell, .. } => {
                if cell.occupancy == OccupancyState::Empty {
                    return Ok(());
                }
                replay_leaf_block(sparse, self.frame.depth(), origin, remaining_depth, *cell)
            }
            SvoNode::Branch { children, .. } => {
                let child_remaining = remaining_depth - 1;
                let child_extent = 1_u64 << child_remaining;
                for child in 0..8_u8 {
                    let child_origin = [
                        origin[0] + child_extent * u64::from(child & 0b001),
                        origin[1] + child_extent * u64::from((child & 0b010) >> 1),
                        origin[2] + child_extent * u64::from((child & 0b100) >> 2),
                    ];
                    self.replay_node_to_sparse(
                        children[child as usize],
                        child_origin,
                        child_remaining,
                        sparse,
                    )?;
                }
                Ok(())
            }
        }
    }
}

fn replay_leaf_block(
    sparse: &mut SparseVoxelGrid,
    frame_depth: u8,
    origin: [u64; 3],
    remaining_depth: u8,
    cell: VoxelCell,
) -> HypervoxelResult<()> {
    let extent = 1_u64 << remaining_depth;
    for z in origin[2]..origin[2] + extent {
        for y in origin[1]..origin[1] + extent {
            for x in origin[0]..origin[0] + extent {
                sparse.set(VoxelAddress::new(frame_depth, [x, y, z])?, cell)?;
            }
        }
    }
    Ok(())
}

fn logical_leaf_cells(remaining_depth: u8) -> usize {
    1_usize << (3 * usize::from(remaining_depth))
}

fn child_index(address: VoxelAddress, current_depth: u8) -> u8 {
    let shift = address.depth - current_depth - 1;
    let x = ((address.xyz[0] >> shift) & 1) as u8;
    let y = ((address.xyz[1] >> shift) & 1) as u8;
    let z = ((address.xyz[2] >> shift) & 1) as u8;
    x | (y << 1) | (z << 2)
}
