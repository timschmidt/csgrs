/// Snapshot of pool capacity and allocation counters.
#[derive(Debug, Default)]
pub struct AllocatorStats {
    /// Number of slots currently counted as allocated.
    pub allocated_blocks: usize,
    /// Number of recycled slots available for reuse.
    pub free_blocks: usize,
    /// Bytes reserved per slot.
    pub block_size: usize,
    /// Alignment of each slot.
    pub block_align: usize,
    /// Total bytes reserved by the pool.
    pub memory_budget: usize,
}
