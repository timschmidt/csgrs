//! Fixed-capacity pool allocators used by the Voxelis node interner.
//!
//! [`PoolAllocator`] manages its own free list. [`PoolAllocatorLite`] leaves
//! free-index ownership to its caller for lower per-slot overhead.

#[cfg(feature = "memory_stats")]
mod allocator_stats;

mod pool_allocator;
mod pool_allocator_lite;

#[cfg(feature = "memory_stats")]
pub use allocator_stats::AllocatorStats;

pub use pool_allocator::PoolAllocator;
pub use pool_allocator_lite::PoolAllocatorLite;
