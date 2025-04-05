#[derive(Debug, Default)]
pub struct AllocatorStats {
    pub allocated_blocks: usize,
    pub free_blocks: usize,
    pub block_size: usize,
    pub block_align: usize,
    pub memory_budget: usize,
}
