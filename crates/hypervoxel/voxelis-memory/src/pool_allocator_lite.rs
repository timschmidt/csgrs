use std::alloc::Layout;

#[cfg(feature = "memory_stats")]
use super::AllocatorStats;

/// Fixed-capacity pool whose caller owns the free-index data structure.
///
/// Passing `None` to [`Self::allocate`] consumes a fresh bump slot. Passing
/// `Some(index)` reuses that exact slot; the caller must previously have
/// deallocated it and must not offer the same index twice. Likewise, only live
/// initialized indices may be read, mutated, or deallocated.
///
/// Live values are not dropped when the pool itself is dropped. Deallocate
/// them explicitly when `T` owns resources. Zero-sized marker types are
/// supported; their indices are logical rather than distinct addresses.
pub struct PoolAllocatorLite<T> {
    memory: *mut T,
    layout: Layout,
    capacity: usize,
    next: usize,
    #[cfg(feature = "memory_stats")]
    stats: AllocatorStats,
}

// SAFETY: moving the pool transfers unique ownership of its allocation. No
// value is accessed without borrowing the pool, so this is sound when `T` can
// itself be transferred between threads.
unsafe impl<T: Send> Send for PoolAllocatorLite<T> {}

// SAFETY: shared access exposes only `&T`; mutation requires `&mut self`.
// Sharing is therefore sound when shared references to `T` are thread-safe.
unsafe impl<T: Sync> Sync for PoolAllocatorLite<T> {}

impl<T> PoolAllocatorLite<T> {
    /// Returns the bytes reserved per slot.
    #[inline(always)]
    pub const fn block_size() -> usize {
        std::mem::size_of::<T>()
    }

    /// Returns the alignment required by `T`.
    #[inline(always)]
    pub const fn align() -> usize {
        std::mem::align_of::<T>()
    }

    /// Reserves storage for exactly `capacity` values.
    ///
    /// # Panics
    ///
    /// Panics for zero capacity, a capacity that does not fit in `u32`, layout
    /// overflow, or allocation failure.
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "Capacity must be greater than 0");
        assert!(
            capacity < u32::MAX as usize,
            "Capacity must be less than u32::MAX"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::new");

        let block_size = Self::block_size();
        let block_align = Self::align();
        let actual_size = if block_size == 0 {
            // GlobalAlloc requires a nonzero layout. One alignment-sized
            // allocation provides a suitably aligned address for every ZST.
            block_align
        } else {
            block_size
                .checked_mul(capacity)
                .expect("Pool allocation size overflow")
        };

        let layout = Layout::from_size_align(actual_size, block_align).expect("Invalid layout");

        #[cfg(feature = "memory_stats")]
        let stats = AllocatorStats {
            block_size,
            block_align,
            memory_budget: actual_size,
            ..Default::default()
        };

        let memory = unsafe {
            // SAFETY: `layout` has nonzero size and valid alignment. The raw
            // allocation is not interpreted as `T` until a slot is initialized.
            let ptr = std::alloc::alloc_zeroed(layout) as *mut T;

            if ptr.is_null() {
                std::alloc::handle_alloc_error(layout);
            }

            ptr
        };

        debug_assert!(
            (memory as usize).is_multiple_of(block_align),
            "Memory not properly aligned"
        );

        debug_assert!(
            memory.align_offset(Self::align()) == 0,
            "Memory not properly aligned"
        );

        Self {
            memory,
            layout,
            capacity,
            next: 0,
            #[cfg(feature = "memory_stats")]
            stats,
        }
    }

    /// Returns the value in a live initialized slot.
    #[inline(always)]
    pub fn get(&self, index: u32) -> &T {
        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::get");

        // SAFETY: the bounds check keeps the pointer within the allocation;
        // the caller-managed index contract requires this slot to hold a `T`.
        unsafe { &*self.memory.add(index as usize) }
    }

    /// Returns the value in a live initialized slot mutably.
    #[inline(always)]
    pub fn get_mut(&mut self, index: u32) -> &mut T {
        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::get_mut");

        // SAFETY: the bounds check keeps the pointer within the allocation;
        // the unique pool borrow prevents aliases through this API, and the
        // caller-managed index contract requires a live `T`.
        unsafe { &mut *self.memory.add(index as usize) }
    }

    /// Stores `value` in a fresh slot or caller-supplied free slot.
    ///
    /// # Panics
    ///
    /// Panics when no fresh slot remains and `next_free` is `None`, or when a
    /// supplied index is out of range.
    pub fn allocate(&mut self, value: T, next_free: Option<u32>) -> u32 {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::allocate");

        let index = match next_free {
            Some(index) => {
                #[cfg(feature = "memory_stats")]
                {
                    self.stats.free_blocks -= 1;
                    self.stats.allocated_blocks += 1;
                }

                index
            }
            None => {
                #[cfg(feature = "memory_stats")]
                {
                    self.stats.allocated_blocks += 1;
                }

                if self.next < self.capacity {
                    let index = self.next;
                    self.next += 1;
                    index as u32
                } else {
                    panic!("Out of memory");
                }
            }
        };

        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );

        // SAFETY: the checked index is within this allocation.
        let ptr = unsafe { self.memory.add(index as usize) };
        // SAFETY: the caller contract requires a supplied index to be free; a
        // bump index has never been initialized. `write` initializes the slot.
        unsafe { std::ptr::write(ptr, value) };

        index
    }

    /// Drops a live value so its index can be returned to the caller's free list.
    ///
    /// The caller must record the index and pass it at most once to a future
    /// allocation.
    pub fn deallocate(&mut self, index: u32) {
        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::deallocate");

        // SAFETY: the checked index is within this allocation.
        let ptr = unsafe { self.memory.add(index as usize) };
        // SAFETY: the caller-managed index contract requires this slot to
        // contain one live value and forbids duplicate deallocation.
        unsafe { std::ptr::drop_in_place(ptr) };

        #[cfg(feature = "memory_stats")]
        {
            self.stats.free_blocks += 1;
            self.stats.allocated_blocks -= 1;
        }
    }
}

impl<T> Drop for PoolAllocatorLite<T> {
    fn drop(&mut self) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("PoolAllocatorLite::drop");

        unsafe {
            // SAFETY: `memory` was allocated with this exact layout and has
            // not been deallocated. Live values are intentionally not dropped.
            std::alloc::dealloc(self.memory as *mut u8, self.layout);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_allocator_basic() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(4);
        let mut free_list = Vec::new();

        let id1 = allocator.allocate(42, None);
        let id2 = allocator.allocate(24, None);

        assert_ne!(id1, id2);

        assert_eq!(*allocator.get(id1), 42);
        assert_eq!(*allocator.get(id2), 24);

        allocator.deallocate(id1);
        free_list.push(id1);
        let id3 = allocator.allocate(242, free_list.pop());
        assert_eq!(id1, id3);
    }

    #[test]
    #[should_panic(expected = "Out of memory")]
    fn test_pool_allocator_out_of_memory() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(2);

        let _id1 = allocator.allocate(42, None);
        let _id2 = allocator.allocate(24, None);
        let _id3 = allocator.allocate(22, None);
    }

    #[repr(align(16))]
    struct Aligned16;

    #[test]
    fn test_pool_allocator_alignment() {
        let _allocator: PoolAllocatorLite<Aligned16> = PoolAllocatorLite::new(4);
        assert_eq!(PoolAllocatorLite::<Aligned16>::align(), 16);
    }

    #[test]
    fn test_pool_allocator_reuse_order() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(4);
        let mut free_list = Vec::new();

        let id1 = allocator.allocate(1, None);
        let id2 = allocator.allocate(2, None);
        let _id3 = allocator.allocate(3, None);

        allocator.deallocate(id2);
        free_list.push(id2);
        allocator.deallocate(id1);
        free_list.push(id1);

        let new_id1 = allocator.allocate(4, free_list.pop());
        let new_id2 = allocator.allocate(5, free_list.pop());

        assert_eq!(new_id1, id1);
        assert_eq!(new_id2, id2);
    }

    #[test]
    fn test_pool_allocator_capacity_edge() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(1);
        let mut free_list = Vec::new();

        let id = allocator.allocate(42, None);
        assert_eq!(id, 0);

        allocator.deallocate(id);
        free_list.push(id);

        let new_id = allocator.allocate(24, free_list.pop());
        assert_eq!(new_id, 0);
    }
}
