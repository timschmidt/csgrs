//! Hyper-owned sparse voxel octree DAG storage.
//!
//! `voxelis` remains the performance seed for a production SVO-DAG backend.
//! This module establishes the exact semantic contract first: nodes are
//! interned by value, edits path-copy through integer addresses, and every
//! branch carries conservative aggregate facts. Geometric objects preserve
//! facts rather than reducing every decision to scalar approximation.

use rustc_hash::FxHashMap;

use crate::{
    GridFrame, HypervoxelError, HypervoxelResult, OccupancyState, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell, VoxelEditReport,
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
    /// the new cell is exact-ready. Compression cannot promote an unknown or
    /// lossy cell to exact state.
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
    /// it is not positive replay evidence for a modeled voxel object. SVO
    /// replay readiness records this evidence bit directly.
    pub has_materialized_evidence: bool,
    /// Whether compressed storage can be consumed as exact replay evidence.
    ///
    /// The report checks logical coverage in addition to interning statistics:
    /// a compact DAG is useful only if its root facts describe the whole grid
    /// frame with non-empty, non-lossy, non-unknown semantic evidence, not
    /// merely the number of physical nodes.
    pub exact_dag_replay_ready: bool,
}

/// Exact sparse-grid replay report for an interned SVO-DAG.
///
/// The SVO-DAG is a compressed representation; this report proves how it
/// expands back to the canonical sparse-grid object facts. Empty collapsed
/// leaves remain implicit sparse absence, while non-empty collapsed leaves are
/// expanded to full-resolution frame leaves. Compression can accelerate and
/// share storage, but exact consumers need replayable object representation and
/// explicit blockers. The readiness bit is semantic, not a rendering claim.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SvoSparseReplayReport {
    /// Frame depth represented by the replay.
    pub frame_depth: u8,
    /// Logical full-resolution leaf cells represented by the frame.
    pub logical_leaf_cells: usize,
    /// Root SVO storage report used as source evidence.
    pub storage: SvoStorageReport,
    /// Number of SVO nodes visited by replay traversal.
    pub visited_nodes: usize,
    /// Number of branch nodes visited.
    pub visited_branches: usize,
    /// Number of leaf nodes visited.
    pub visited_leaves: usize,
    /// Full-resolution empty cells represented by skipped collapsed leaves.
    pub skipped_empty_leaf_cells: usize,
    /// Full-resolution non-empty cells represented by expanded leaves.
    pub expanded_non_empty_leaf_cells: usize,
    /// Explicit cells written to the sparse replay.
    pub materialized_sparse_cells: usize,
    /// Non-empty expanded cells whose payloads were exact-ready.
    pub exact_payload_cells: usize,
    /// Non-empty expanded cells carrying unknown evidence.
    pub unknown_leaf_cells: usize,
    /// Non-empty expanded cells carrying lossy adapter evidence.
    pub lossy_leaf_cells: usize,
    /// Largest remaining collapsed depth expanded by any non-empty leaf.
    pub max_expanded_remaining_depth: u8,
    /// Aggregate facts recomputed from the replayed sparse grid.
    pub replay_aggregate: VoxelAggregateFacts,
    /// Whether replayed sparse aggregate counts match the SVO root aggregate.
    ///
    /// Parent SVO aggregates can conservatively mark representation-level
    /// mixed subtrees even when the fully expanded sparse replay has exact
    /// filled/empty counts. This flag compares the semantic occupancy counts,
    /// material set, and unknown/lossy/boundary blockers that must survive
    /// replay, while both full aggregate packets remain available for audit.
    pub aggregate_replay_matches_root: bool,
    /// Whether the SVO-DAG can be consumed as exact sparse-grid replay
    /// evidence.
    pub exact_sparse_replay_ready: bool,
}

/// Exact sparse-to-SVO-DAG compaction report.
///
/// This report is the import-side counterpart to [`SvoSparseReplayReport`]:
/// it records how canonical sparse cells were admitted into compressed SVO-DAG
/// storage and then replayed back for semantic comparison. Compression is
/// exact only when it preserves replayable object facts and reports blockers
/// instead of repairing them. This report concerns voxel semantics rather than
/// rendering throughput.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SvoCompactionReport {
    /// Explicit non-empty sparse cells offered for compaction.
    pub source_cells: usize,
    /// Source cells already at the frame depth.
    pub finest_depth_cells: usize,
    /// Source cells coarser than the frame depth.
    ///
    /// Sparse-grid storage treats an explicit coarse address as a stored fact
    /// at that address, while SVO replay expands collapsed non-empty leaves to
    /// finest-depth descendants. Coarse source cells are therefore accepted
    /// into storage for audit, but they block exact sparse round-trip
    /// readiness.
    pub non_finest_depth_cells: usize,
    /// Source cells whose payloads are exact-ready.
    pub exact_payload_cells: usize,
    /// Source cells carrying unknown evidence.
    pub unknown_cells: usize,
    /// Source cells carrying lossy adapter evidence.
    pub lossy_cells: usize,
    /// Number of source-cell insertions applied to the SVO.
    ///
    /// Canonical finest-depth imports use one bottom-up reduction pass;
    /// noncanonical coarse imports retain sequential path-copy semantics for
    /// audit compatibility.
    pub applied_edits: usize,
    /// Edits that were semantic no-ops against the current SVO state.
    pub semantic_noops: usize,
    /// Edits that changed the canonical root node.
    pub root_changes: usize,
    /// Number of interned nodes after compaction.
    pub compacted_nodes: usize,
    /// Number of unique interned leaves after compaction.
    pub compacted_leaves: usize,
    /// Number of unique interned branches after compaction.
    pub compacted_branches: usize,
    /// Source explicit-cell count minus compacted node count when positive.
    pub node_savings_vs_sparse_cells: usize,
    /// SVO storage report after compaction.
    pub storage: SvoStorageReport,
    /// Replay report from the compacted SVO back to canonical sparse storage.
    pub sparse_replay: SvoSparseReplayReport,
    /// Whether replayed sparse storage is exactly the same as the source
    /// sparse storage.
    pub semantic_round_trip_matches_source: bool,
    /// Whether sparse-to-SVO compaction can be consumed as exact replay
    /// evidence.
    pub exact_svo_compaction_ready: bool,
}

/// Exact semantic sparse voxel octree with DAG interning.
#[derive(Clone, Debug, PartialEq)]
pub struct SvoVoxelGrid {
    frame: GridFrame,
    nodes: Vec<SvoNode>,
    interned: FxHashMap<SvoNodeKey, SvoNodeId>,
    root: SvoNodeId,
}

impl SvoVoxelGrid {
    /// Creates an empty interned SVO grid.
    pub fn new(frame: GridFrame) -> Self {
        let frame_depth = frame.depth();
        let mut grid = Self {
            frame,
            nodes: Vec::new(),
            interned: FxHashMap::default(),
            root: SvoNodeId(0),
        };
        let root = grid.intern_leaf(VoxelCell::empty(), frame_depth);
        grid.root = root;
        grid
    }

    /// Compacts canonical sparse-grid storage into an interned SVO-DAG.
    ///
    /// The compactor path-copies every explicit sparse cell into a fresh SVO,
    /// preserving the original [`VoxelCell`] evidence. It then replays the SVO
    /// back to sparse storage and compares that replay with the source grid.
    /// This round trip is the exactness gate: DAG interning and collapsed
    /// leaves are allowed to change representation size, but not the object
    /// facts accepted by sparse-grid consumers.
    pub fn from_sparse_grid_with_report(
        source: &SparseVoxelGrid,
    ) -> HypervoxelResult<(Self, SvoCompactionReport)> {
        let mut grid = Self::new(source.frame().clone());
        let mut source_cells = 0_usize;
        let mut finest_depth_cells = 0_usize;
        let mut non_finest_depth_cells = 0_usize;
        let mut exact_payload_cells = 0_usize;
        let mut unknown_cells = 0_usize;
        let mut lossy_cells = 0_usize;
        let mut entries = Vec::with_capacity(source.len());

        for (address, cell) in source.iter() {
            entries.push((*address, *cell));
            source_cells += 1;
            if address.depth == source.frame().depth() {
                finest_depth_cells += 1;
            } else {
                non_finest_depth_cells += 1;
            }
            let cell_report = cell.report();
            exact_payload_cells += usize::from(cell_report.exact_cell_evidence_ready);
            unknown_cells += usize::from(cell_report.has_unknown);
            lossy_cells += usize::from(cell_report.has_lossy);
        }

        let (applied_edits, semantic_noops, root_changes) = if non_finest_depth_cells == 0 {
            let previous_root = grid.root;
            entries.sort_unstable_by_key(|(address, _)| address.morton_code());
            grid.root = grid.build_finest_sparse_subtree(&entries, 0);
            (source_cells, 0, usize::from(grid.root != previous_root))
        } else {
            let mut applied_edits = 0_usize;
            let mut semantic_noops = 0_usize;
            let mut root_changes = 0_usize;
            for (address, cell) in entries {
                let edit = grid.set_with_report(address, cell)?;
                applied_edits += 1;
                semantic_noops += usize::from(edit.edit.semantic_noop);
                root_changes += usize::from(edit.root_changed);
            }
            (applied_edits, semantic_noops, root_changes)
        };

        let stats = grid.stats();
        let (replayed_sparse, sparse_replay) = grid.replay_sparse_grid_with_report()?;
        let storage = sparse_replay.storage.clone();
        let semantic_round_trip_matches_source = &replayed_sparse == source;
        let exact_svo_compaction_ready = source_cells > 0
            && non_finest_depth_cells == 0
            && unknown_cells == 0
            && lossy_cells == 0
            && exact_payload_cells == source_cells
            && sparse_replay.exact_sparse_replay_ready
            && semantic_round_trip_matches_source;
        let report = SvoCompactionReport {
            source_cells,
            finest_depth_cells,
            non_finest_depth_cells,
            exact_payload_cells,
            unknown_cells,
            lossy_cells,
            applied_edits,
            semantic_noops,
            root_changes,
            compacted_nodes: stats.nodes,
            compacted_leaves: stats.leaves,
            compacted_branches: stats.branches,
            node_savings_vs_sparse_cells: source_cells.saturating_sub(stats.nodes),
            storage,
            sparse_replay,
            semantic_round_trip_matches_source,
            exact_svo_compaction_ready,
        };
        Ok((grid, report))
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

    /// Replays this SVO-DAG into canonical sparse-grid storage with an exact
    /// expansion report.
    ///
    /// This is intentionally a semantic replay operation rather than a fast
    /// iterator over physical nodes. A collapsed non-empty leaf at a coarser
    /// address represents every descendant full-resolution cell with the same
    /// exact payload, so replay expands those descendants and then recomputes
    /// sparse-grid aggregate facts. Collapsed empty leaves are counted but not
    /// inserted, matching [`SparseVoxelGrid`]'s implicit-empty convention.
    pub fn replay_sparse_grid_with_report(
        &self,
    ) -> HypervoxelResult<(SparseVoxelGrid, SvoSparseReplayReport)> {
        let mut sparse = SparseVoxelGrid::new(self.frame.clone());
        let mut counters = SvoSparseReplayCounters::default();
        self.replay_node_to_sparse(
            self.root,
            [0, 0, 0],
            self.frame.depth(),
            &mut sparse,
            &mut counters,
        )?;

        let logical_leaf_cells = logical_leaf_cells(self.frame.depth());
        let replay_aggregate = VoxelAggregateFacts::from_explicit_cells_in_frame(
            logical_leaf_cells,
            sparse.iter().map(|(_, cell)| cell),
        )?;
        let storage = self.report();
        let aggregate_replay_matches_root =
            replay_aggregate_matches_root(&replay_aggregate, &storage.root_aggregate);
        let exact_sparse_replay_ready = storage.exact_dag_replay_ready
            && aggregate_replay_matches_root
            && counters.materialized_sparse_cells == counters.expanded_non_empty_leaf_cells
            && counters.unknown_leaf_cells == 0
            && counters.lossy_leaf_cells == 0
            && counters.exact_payload_cells == counters.expanded_non_empty_leaf_cells;

        let report = SvoSparseReplayReport {
            frame_depth: self.frame.depth(),
            logical_leaf_cells,
            storage,
            visited_nodes: counters.visited_nodes,
            visited_branches: counters.visited_branches,
            visited_leaves: counters.visited_leaves,
            skipped_empty_leaf_cells: counters.skipped_empty_leaf_cells,
            expanded_non_empty_leaf_cells: counters.expanded_non_empty_leaf_cells,
            materialized_sparse_cells: counters.materialized_sparse_cells,
            exact_payload_cells: counters.exact_payload_cells,
            unknown_leaf_cells: counters.unknown_leaf_cells,
            lossy_leaf_cells: counters.lossy_leaf_cells,
            max_expanded_remaining_depth: counters.max_expanded_remaining_depth,
            replay_aggregate,
            aggregate_replay_matches_root,
            exact_sparse_replay_ready,
        };
        Ok((sparse, report))
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
        let aggregate = VoxelAggregateFacts::from_aggregates(
            children.iter().map(|child| self.node(*child).aggregate()),
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
        counters: &mut SvoSparseReplayCounters,
    ) -> HypervoxelResult<()> {
        counters.visited_nodes += 1;
        match self.node(node_id) {
            SvoNode::Leaf { cell, .. } => {
                counters.visited_leaves += 1;
                let represented_cells = logical_leaf_cells(remaining_depth);
                if cell.occupancy == OccupancyState::Empty {
                    counters.skipped_empty_leaf_cells += represented_cells;
                    return Ok(());
                }
                counters.expanded_non_empty_leaf_cells += represented_cells;
                counters.max_expanded_remaining_depth =
                    counters.max_expanded_remaining_depth.max(remaining_depth);
                let cell_report = cell.report();
                counters.unknown_leaf_cells +=
                    usize::from(cell_report.has_unknown) * represented_cells;
                counters.lossy_leaf_cells += usize::from(cell_report.has_lossy) * represented_cells;
                counters.exact_payload_cells +=
                    usize::from(cell_report.exact_cell_evidence_ready) * represented_cells;
                replay_leaf_block(
                    sparse,
                    self.frame.depth(),
                    origin,
                    remaining_depth,
                    *cell,
                    counters,
                )
            }
            SvoNode::Branch { children, .. } => {
                counters.visited_branches += 1;
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
                        counters,
                    )?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Default)]
struct SvoSparseReplayCounters {
    visited_nodes: usize,
    visited_branches: usize,
    visited_leaves: usize,
    skipped_empty_leaf_cells: usize,
    expanded_non_empty_leaf_cells: usize,
    materialized_sparse_cells: usize,
    exact_payload_cells: usize,
    unknown_leaf_cells: usize,
    lossy_leaf_cells: usize,
    max_expanded_remaining_depth: u8,
}

fn replay_leaf_block(
    sparse: &mut SparseVoxelGrid,
    frame_depth: u8,
    origin: [u64; 3],
    remaining_depth: u8,
    cell: VoxelCell,
    counters: &mut SvoSparseReplayCounters,
) -> HypervoxelResult<()> {
    let extent = 1_u64 << remaining_depth;
    for z in origin[2]..origin[2] + extent {
        for y in origin[1]..origin[1] + extent {
            for x in origin[0]..origin[0] + extent {
                sparse.set(VoxelAddress::new(frame_depth, [x, y, z])?, cell)?;
                counters.materialized_sparse_cells += 1;
            }
        }
    }
    Ok(())
}

fn replay_aggregate_matches_root(replay: &VoxelAggregateFacts, root: &VoxelAggregateFacts) -> bool {
    replay.child_count == root.child_count
        && replay.all_empty == root.all_empty
        && replay.all_filled == root.all_filled
        && replay.has_boundary == root.has_boundary
        && replay.has_unknown == root.has_unknown
        && replay.has_lossy == root.has_lossy
        && replay.material_regions == root.material_regions
        && replay.occupancy_interval.total_cells == root.occupancy_interval.total_cells
        && replay.occupancy_interval.definite_filled_cells
            == root.occupancy_interval.definite_filled_cells
        && replay.occupancy_interval.possible_occupied_cells
            == root.occupancy_interval.possible_occupied_cells
        && replay.occupancy_interval.lower == root.occupancy_interval.lower
        && replay.occupancy_interval.upper == root.occupancy_interval.upper
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
