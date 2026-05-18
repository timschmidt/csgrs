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
    VoxelEditReport,
};

/// Interned SVO node identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SvoNodeId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum SvoNodeKey {
    Leaf {
        cell: VoxelCell,
        remaining_depth: u8,
    },
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

/// Report returned by an SVO-DAG edit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SvoEditReport {
    /// Sparse-grid-compatible edit facts.
    pub edit: VoxelEditReport,
    /// Root node before the path-copy edit.
    pub previous_root: SvoNodeId,
    /// Root node after the path-copy edit.
    pub current_root: SvoNodeId,
    /// Whether the canonical root changed.
    pub root_changed: bool,
    /// Number of interned nodes before the edit.
    pub previous_nodes: usize,
    /// Number of interned nodes after the edit.
    pub current_nodes: usize,
    /// Whether the path-copy edit can be replayed as exact storage evidence.
    ///
    /// SVO-DAG updates are exact only when the address was frame-validated and
    /// the new cell is exact-ready. This mirrors Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997: the compressed
    /// representation cannot promote an unknown or lossy cell to exact state.
    pub exact_path_replay_ready: bool,
}

/// Exact replay report for an interned SVO-DAG grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SvoStorageReport {
    /// Frame depth represented by the root.
    pub frame_depth: u8,
    /// Logical leaf cells represented by the root subtree.
    pub logical_leaf_cells: usize,
    /// Current root node.
    pub root: SvoNodeId,
    /// Current interning statistics.
    pub stats: SvoDagStats,
    /// Root aggregate facts over logical cells.
    pub root_aggregate: VoxelAggregateFacts,
    /// Whether the root aggregate spans the complete frame.
    pub root_aggregate_covers_frame: bool,
    /// Whether the root aggregate contains any non-empty semantic evidence.
    ///
    /// A collapsed empty root is a compact exact absence representation, but
    /// it is not positive replay evidence for a modeled voxel object. Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997, keeps exactness attached to represented objects and
    /// predicates, so SVO replay readiness records this evidence bit directly.
    pub has_materialized_evidence: bool,
    /// Whether compressed storage can be consumed as exact replay evidence.
    ///
    /// The report checks logical coverage in addition to interning statistics:
    /// a compact DAG is useful only if its root facts describe the whole grid
    /// frame with non-empty, non-lossy, non-unknown semantic evidence, not
    /// merely the number of physical nodes.
    pub exact_dag_replay_ready: bool,
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
        let frame_depth = frame.depth();
        let mut grid = Self {
            frame,
            nodes: Vec::new(),
            interned: BTreeMap::new(),
            root: SvoNodeId(0),
        };
        let root = grid.intern_leaf(VoxelCell::empty(), frame_depth);
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

    /// Returns a report describing exact replay readiness for compressed
    /// storage.
    pub fn report(&self) -> SvoStorageReport {
        let logical_leaf_cells = logical_leaf_cells(self.frame.depth());
        let root_aggregate = self.aggregate().clone();
        let root_aggregate_covers_frame = root_aggregate.child_count == logical_leaf_cells
            && root_aggregate.occupancy_interval.total_cells == logical_leaf_cells;
        let has_materialized_evidence = !root_aggregate.all_empty;
        let exact_dag_replay_ready = root_aggregate_covers_frame
            && has_materialized_evidence
            && !root_aggregate.has_unknown
            && !root_aggregate.has_lossy;
        SvoStorageReport {
            frame_depth: self.frame.depth(),
            logical_leaf_cells,
            root: self.root,
            stats: self.stats(),
            root_aggregate,
            root_aggregate_covers_frame,
            has_materialized_evidence,
            exact_dag_replay_ready,
        }
    }

    /// Reads a cell from the SVO, returning exact empty for collapsed empty
    /// subtrees.
    pub fn get(&self, address: VoxelAddress) -> HypervoxelResult<VoxelCell> {
        self.validate_address(address)?;
        self.get_recursive(self.root, address, 0)
    }

    /// Sets a cell by path-copying and interning changed nodes.
    pub fn set(&mut self, address: VoxelAddress, cell: VoxelCell) -> HypervoxelResult<()> {
        self.set_with_report(address, cell).map(|_| ())
    }

    /// Sets a cell and returns exact path-copy/interner replay evidence.
    pub fn set_with_report(
        &mut self,
        address: VoxelAddress,
        cell: VoxelCell,
    ) -> HypervoxelResult<SvoEditReport> {
        self.validate_address(address)?;
        let previous_root = self.root;
        let previous_nodes = self.nodes.len();
        let previous = self.get(address)?;
        self.root = self.set_recursive(self.root, address, 0, cell)?;
        let current_nodes = self.nodes.len();
        let edit = VoxelEditReport {
            address,
            previous: Some(previous),
            current: cell,
            frame_validated: true,
            stored_explicit_cell: cell.occupancy != crate::OccupancyState::Empty,
            removed_explicit_cell: cell.occupancy == crate::OccupancyState::Empty
                && previous.occupancy != crate::OccupancyState::Empty,
            semantic_noop: previous == cell,
            exact_edit_replay_ready: cell.report().exact_cell_evidence_ready,
        };
        let exact_path_replay_ready = edit.exact_edit_replay_ready;
        Ok(SvoEditReport {
            edit,
            previous_root,
            current_root: self.root,
            root_changed: previous_root != self.root,
            previous_nodes,
            current_nodes,
            exact_path_replay_ready,
        })
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

    fn intern_leaf(&mut self, cell: VoxelCell, remaining_depth: u8) -> SvoNodeId {
        let key = SvoNodeKey::Leaf {
            cell,
            remaining_depth,
        };
        if let Some(id) = self.interned.get(&key) {
            return *id;
        }
        let id = SvoNodeId(self.nodes.len() as u32);
        let aggregate =
            VoxelAggregateFacts::from_uniform_cell(logical_leaf_cells(remaining_depth), &cell);
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
            return Ok(self.intern_leaf(cell, self.frame.depth() - current_depth));
        }

        let remaining_child_depth = self.frame.depth() - current_depth - 1;
        let mut children = match self.node(node_id).clone() {
            SvoNode::Leaf { cell, .. } => [self.intern_leaf(cell, remaining_child_depth); 8],
            SvoNode::Branch { children, .. } => children,
        };
        let child = child_index(address, current_depth);
        children[child as usize] =
            self.set_recursive(children[child as usize], address, current_depth + 1, cell)?;
        Ok(self.intern_branch(children))
    }
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
