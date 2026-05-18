//! Hyper-owned sparse voxel octree DAG storage.
//!
//! `voxelis` remains the performance seed for a production SVO-DAG backend.
//! This module establishes the exact semantic contract first: nodes are
//! interned by value, edits path-copy through integer addresses, and every
//! branch carries conservative aggregate facts. This follows Yap's guidance in
//! "Towards Exact Geometric Computation," *Computational Geometry*, 1997, that
//! geometric objects should preserve facts instead of reducing every decision
//! to scalar approximation.

use std::collections::BTreeMap;

use crate::{
    GridFrame, HypervoxelError, HypervoxelResult, VoxelAddress, VoxelAggregateFacts, VoxelCell,
};

/// Interned SVO node identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SvoNodeId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum SvoNodeKey {
    Leaf(VoxelCell),
    Branch([SvoNodeId; 8]),
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum SvoNode {
    Leaf {
        cell: VoxelCell,
        aggregate: VoxelAggregateFacts,
    },
    Branch {
        children: [SvoNodeId; 8],
        aggregate: VoxelAggregateFacts,
    },
}

impl SvoNode {
    fn aggregate(&self) -> &VoxelAggregateFacts {
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
    interned: BTreeMap<SvoNodeKey, SvoNodeId>,
    root: SvoNodeId,
}

impl SvoVoxelGrid {
    /// Creates an empty interned SVO grid.
    pub fn new(frame: GridFrame) -> Self {
        let mut grid = Self {
            frame,
            nodes: Vec::new(),
            interned: BTreeMap::new(),
            root: SvoNodeId(0),
        };
        let root = grid.intern_leaf(VoxelCell::empty());
        grid.root = root;
        grid
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
        self.node(self.root).aggregate()
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

    /// Reads a cell from the SVO, returning exact empty for collapsed empty
    /// subtrees.
    pub fn get(&self, address: VoxelAddress) -> HypervoxelResult<VoxelCell> {
        self.validate_address(address)?;
        self.get_recursive(self.root, address, 0)
    }

    /// Sets a cell by path-copying and interning changed nodes.
    pub fn set(&mut self, address: VoxelAddress, cell: VoxelCell) -> HypervoxelResult<()> {
        self.validate_address(address)?;
        self.root = self.set_recursive(self.root, address, 0, cell)?;
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

    fn intern_leaf(&mut self, cell: VoxelCell) -> SvoNodeId {
        let key = SvoNodeKey::Leaf(cell);
        if let Some(id) = self.interned.get(&key) {
            return *id;
        }
        let id = SvoNodeId(self.nodes.len() as u32);
        let aggregate = VoxelAggregateFacts::from_cells([&cell]);
        self.nodes.push(SvoNode::Leaf { cell, aggregate });
        self.interned.insert(key, id);
        id
    }

    fn intern_branch(&mut self, children: [SvoNodeId; 8]) -> SvoNodeId {
        if children.iter().all(|child| *child == children[0]) {
            return children[0];
        }

        let key = SvoNodeKey::Branch(children);
        if let Some(id) = self.interned.get(&key) {
            return *id;
        }
        let id = SvoNodeId(self.nodes.len() as u32);
        let child_facts = children
            .iter()
            .map(|child| self.node(*child).aggregate())
            .collect::<Vec<_>>();
        let aggregate = VoxelAggregateFacts::from_aggregates(child_facts);
        self.nodes.push(SvoNode::Branch {
            children,
            aggregate,
        });
        self.interned.insert(key, id);
        id
    }

    fn get_recursive(
        &self,
        node_id: SvoNodeId,
        address: VoxelAddress,
        current_depth: u8,
    ) -> HypervoxelResult<VoxelCell> {
        match self.node(node_id) {
            SvoNode::Leaf { cell, .. } => Ok(*cell),
            SvoNode::Branch { children, .. } => {
                if current_depth == address.depth {
                    return Ok(VoxelCell {
                        occupancy: self.node(node_id).aggregate().conservative_occupancy(),
                        payload: crate::VoxelPayload::Occupancy(
                            self.node(node_id).aggregate().conservative_occupancy(),
                        ),
                    });
                }
                let child = child_index(address, current_depth);
                self.get_recursive(children[child as usize], address, current_depth + 1)
            }
        }
    }

    fn set_recursive(
        &mut self,
        node_id: SvoNodeId,
        address: VoxelAddress,
        current_depth: u8,
        cell: VoxelCell,
    ) -> HypervoxelResult<SvoNodeId> {
        if current_depth == address.depth {
            return Ok(self.intern_leaf(cell));
        }

        let mut children = match self.node(node_id).clone() {
            SvoNode::Leaf { cell, .. } => [self.intern_leaf(cell); 8],
            SvoNode::Branch { children, .. } => children,
        };
        let child = child_index(address, current_depth);
        children[child as usize] =
            self.set_recursive(children[child as usize], address, current_depth + 1, cell)?;
        Ok(self.intern_branch(children))
    }
}

fn child_index(address: VoxelAddress, current_depth: u8) -> u8 {
    let shift = address.depth - current_depth - 1;
    let x = ((address.xyz[0] >> shift) & 1) as u8;
    let y = ((address.xyz[1] >> shift) & 1) as u8;
    let z = ((address.xyz[2] >> shift) & 1) as u8;
    x | (y << 1) | (z << 2)
}
