use std::{alloc::Layout, ptr::NonNull};

#[cfg(feature = "memory_stats")]
use super::AllocatorStats;

/// Fixed-capacity pool with an internal LIFO free list.
///
/// Returned indices remain valid until passed to [`Self::deallocate`]. The
/// allocator does not track initialization separately, so only indices
/// returned by [`Self::allocate`] and not subsequently deallocated may be
/// passed to [`Self::get`], [`Self::get_mut`], or [`Self::deallocate`].
///
/// Live values are not dropped when the pool itself is dropped. Deallocate
/// them explicitly when `T` owns resources.
pub struct PoolAllocator<T> {
    memory: NonNull<T>,
    free_blocks: *mut T,
    next: usize,
    capacity: usize,
    layout: Layout,
    base_ptr: usize,
    block_size: usize,
    #[cfg(feature = "memory_stats")]
    stats: AllocatorStats,
}

impl<T> PoolAllocator<T> {
    /// Returns the stride reserved for one value and a free-list pointer.
    #[inline(always)]
    pub const fn block_size() -> usize {
        let size = std::mem::size_of::<T>();
        let min_size = std::mem::size_of::<*mut T>();

        if size < min_size { min_size } else { size }
    }

    /// Returns the alignment required by both values and free-list pointers.
    #[inline(always)]
    pub const fn align() -> usize {
        let type_align = std::mem::align_of::<T>();
        let ptr_align = std::mem::align_of::<*mut T>();

        if type_align < ptr_align {
            ptr_align
        } else {
            type_align
        }
    }

    /// Reserves storage for exactly `capacity` values.
    ///
    /// # Panics
    ///
    /// Panics for zero capacity, capacities that do not fit in `u32`, layout
    /// overflow, or allocation failure.
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "Capacity must be greater than 0");
        assert!(
            capacity < u32::MAX as usize,
            "Capacity must be less than u32::MAX"
        );

        let block_size = Self::block_size();
        let block_align = Self::align();
        let actual_size = block_size
            .checked_mul(capacity)
            .expect("Pool allocation size overflow");

        let layout = Layout::from_size_align(actual_size, block_align).expect("Invalid layout");

        #[cfg(feature = "memory_stats")]
        let stats = AllocatorStats {
            block_size,
            block_align,
            memory_budget: actual_size,
            ..Default::default()
        };

        let memory = unsafe {
            // SAFETY: `layout` has nonzero size and valid alignment. A null
            // result is converted to a panic before the pointer is retained.
            NonNull::new(std::alloc::alloc_zeroed(layout) as *mut T)
                .expect("Failed to allocate memory pool")
        };

        let base_ptr = memory.as_ptr() as usize;

        Self {
            memory,
            free_blocks: std::ptr::null_mut(),
            next: 0,
            capacity,
            layout,
            base_ptr,
            block_size,
            #[cfg(feature = "memory_stats")]
            stats,
        }
    }

    /// Returns the value in a currently allocated slot.
    ///
    /// The index must have been returned by [`Self::allocate`] and must not
    /// have been deallocated.
    pub fn get(&self, index: u32) -> &T {
        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );
        debug_assert!(
            (self.base_ptr as *mut T).align_offset(Self::align()) == 0,
            "Memory not properly aligned"
        );

        let ptr = self.index_to_ptr(index);

        // SAFETY: the bounds check places `ptr` within the allocation. The
        // allocator contract requires this slot to contain a live `T`.
        unsafe { &*ptr }
    }

    /// Returns the value in a currently allocated slot mutably.
    ///
    /// The index must have been returned by [`Self::allocate`] and must not
    /// have been deallocated.
    pub fn get_mut(&mut self, index: u32) -> &mut T {
        assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {index} capacity: {}",
            self.capacity
        );
        debug_assert!(
            (self.base_ptr as *mut T).align_offset(Self::align()) == 0,
            "Memory not properly aligned"
        );

        let ptr = self.index_to_ptr(index);

        // SAFETY: the bounds check places `ptr` within the allocation. The
        // unique allocator borrow prevents another reference through this API,
        // and the allocator contract requires a live `T` in the slot.
        unsafe { &mut *ptr }
    }

    /// Stores `value`, returning its stable slot index.
    ///
    /// Recycled slots are reused in LIFO order.
    ///
    /// # Panics
    ///
    /// Panics when every slot is live.
    pub fn allocate(&mut self, value: T) -> u32 {
        if !self.free_blocks.is_null() {
            #[cfg(feature = "memory_stats")]
            {
                self.stats.free_blocks -= 1;
                self.stats.allocated_blocks += 1;
            }

            let ptr = self.free_blocks;
            // SAFETY: every free-list entry contains the pointer written by
            // `deallocate`, and `ptr` is the current list head.
            let next_free = unsafe { *(ptr as *mut *mut T) };
            self.free_blocks = next_free;

            // SAFETY: `ptr` names a free, properly aligned slot. `write`
            // initializes it without trying to drop the stored list pointer.
            unsafe { std::ptr::write(ptr, value) };

            let index = self.ptr_to_index(ptr);

            debug_assert!(
                index < self.capacity as u32,
                "Block index out of bounds index: {index} capacity: {}",
                self.capacity
            );

            index
        } else if self.next < self.capacity {
            #[cfg(feature = "memory_stats")]
            {
                self.stats.allocated_blocks += 1;
            }

            let index = self.next as u32;
            self.next += 1;

            debug_assert!(
                index < self.capacity as u32,
                "Block index out of bounds index: {index} capacity: {}",
                self.capacity
            );

            let ptr = self.index_to_ptr(index);
            // SAFETY: `index < capacity`, the bump slot has not previously
            // been initialized, and `index_to_ptr` preserves slot alignment.
            unsafe { std::ptr::write(ptr, value) };

            index
        } else {
            panic!("Out of memory");
        }
    }

    /// Drops the value at `index` and adds the slot to the free list.
    ///
    /// # Panics
    ///
    /// Panics for an out-of-range index or a slot already on the free list.
    /// The index must otherwise identify a currently allocated value.
    pub fn deallocate(&mut self, index: u32) {
        assert!(index < self.capacity as u32, "Block index out of bounds");

        let ptr = self.index_to_ptr(index);

        let mut current = self.free_blocks;
        while !current.is_null() {
            if current == ptr {
                panic!("Double free detected");
            }
            // SAFETY: `current` is a free-list entry initialized with a next
            // pointer by an earlier successful deallocation.
            current = unsafe { *(current as *mut *mut T) };
        }

        // SAFETY: the caller contract and free-list scan establish that the
        // in-bounds slot contains one live `T`.
        unsafe { std::ptr::drop_in_place(ptr) };

        #[cfg(feature = "memory_stats")]
        {
            self.stats.free_blocks += 1;
            self.stats.allocated_blocks -= 1;
        }

        unsafe {
            // SAFETY: `T` has been dropped, and each slot is large and aligned
            // enough to store a free-list pointer by construction.
            *(ptr as *mut *mut T) = self.free_blocks;
            self.free_blocks = ptr;
        }
    }

    #[inline(always)]
    fn ptr_to_index(&self, ptr: *mut T) -> u32 {
        ((ptr as usize - self.base_ptr) / self.block_size) as u32
    }

    #[inline(always)]
    const fn index_to_ptr(&self, index: u32) -> *mut T {
        (self.base_ptr + (index as usize * self.block_size)) as *mut T
    }
}

impl<T> Drop for PoolAllocator<T> {
    fn drop(&mut self) {
        unsafe {
            // SAFETY: `memory` was allocated with this exact layout and has
            // not been deallocated. Live values are intentionally not dropped.
            std::alloc::dealloc(self.memory.as_ptr() as *mut u8, self.layout);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_allocator_basic() {
        let mut allocator: PoolAllocator<u32> = PoolAllocator::new(4);

        let id1 = allocator.allocate(42);
        let id2 = allocator.allocate(24);

        assert_ne!(id1, id2);

        assert_eq!(*allocator.get(id1), 42);
        assert_eq!(*allocator.get(id2), 24);

        allocator.deallocate(id1);
        let id3 = allocator.allocate(242);
        assert_eq!(id1, id3);
    }

    #[test]
    #[should_panic(expected = "Out of memory")]
    fn test_pool_allocator_out_of_memory() {
        let mut allocator: PoolAllocator<u32> = PoolAllocator::new(2);

        let _id1 = allocator.allocate(42);
        let _id2 = allocator.allocate(24);
        let _id3 = allocator.allocate(22); // Should panic
    }

    #[test]
    #[should_panic(expected = "Double free detected")]
    fn test_pool_allocator_double_free() {
        let mut allocator: PoolAllocator<u32> = PoolAllocator::new(2);

        let id = allocator.allocate(42);
        allocator.deallocate(id);
        allocator.deallocate(id); // Should panic
    }

    #[repr(align(16))]
    struct Aligned16;

    #[test]
    fn test_pool_allocator_alignment() {
        assert_eq!(PoolAllocator::<u8>::block_size() % 8, 0);
        assert_eq!(PoolAllocator::<Aligned16>::align(), 16);
    }

    #[test]
    fn test_pool_allocator_reuse_order() {
        let mut allocator: PoolAllocator<u32> = PoolAllocator::new(4);

        // Allocate all blocks
        let id1 = allocator.allocate(1);
        let id2 = allocator.allocate(2);
        let _ = allocator.allocate(3);

        // Free in specific order
        allocator.deallocate(id2); // Middle
        allocator.deallocate(id1); // First

        // Check LIFO order
        let new_id1 = allocator.allocate(4);
        let new_id2 = allocator.allocate(5);

        // We should get blocks in reverse deallocation order
        assert_eq!(new_id1, id1);
        assert_eq!(new_id2, id2);
    }

    #[test]
    fn test_pool_allocator_capacity_edge() {
        let mut allocator: PoolAllocator<u32> = PoolAllocator::new(1);

        // Aloocate single block
        let id = allocator.allocate(42);
        assert_eq!(id, 0);

        // Deallocate and allocate again - should reuse the same block
        allocator.deallocate(id);
        let new_id = allocator.allocate(24);
        assert_eq!(new_id, 0);
    }
}
