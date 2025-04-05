use std::alloc::Layout;

#[cfg(feature = "memory_stats")]
use super::AllocatorStats;

pub struct PoolAllocatorLite<T> {
    memory: *mut T,
    layout: Layout,
    capacity: usize,
    next: usize,
    #[cfg(feature = "memory_stats")]
    stats: AllocatorStats,
}

unsafe impl<T> Send for PoolAllocatorLite<T> {}
unsafe impl<T> Sync for PoolAllocatorLite<T> {}

impl<T> PoolAllocatorLite<T> {
    #[inline(always)]
    pub const fn block_size() -> usize {
        std::mem::size_of::<T>()
    }

    #[inline(always)]
    pub const fn align() -> usize {
        std::mem::align_of::<T>()
    }

    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "Capacity must be greater than 0");
        assert!(
            capacity < u32::MAX as usize,
            "Capacity must be less than u32::MAX"
        );

        let block_size = Self::block_size();
        let block_align = Self::align();
        let actual_size = block_size * capacity;

        let layout = Layout::from_size_align(actual_size, block_align).expect("Invalid layout");

        #[cfg(feature = "memory_stats")]
        let stats = AllocatorStats {
            block_size,
            block_align,
            memory_budget: actual_size,
            ..Default::default()
        };

        let memory = unsafe {
            let ptr = std::alloc::alloc_zeroed(layout) as *mut T;

            if ptr.is_null() {
                std::alloc::handle_alloc_error(layout);
            }

            ptr
        };

        debug_assert!(
            (memory as usize) % block_align == 0,
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

    #[inline(always)]
    pub fn get(&self, index: u32) -> &T {
        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );

        unsafe { &*self.memory.add(index as usize) }
    }

    #[inline(always)]
    pub fn get_mut(&mut self, index: u32) -> &mut T {
        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );

        unsafe { &mut *self.memory.add(index as usize) }
    }

    pub fn allocate(&mut self, value: T, next_free: Option<u32>) -> u32 {
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

        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );

        let ptr = unsafe { self.memory.add(index as usize) };
        unsafe { std::ptr::write(ptr, value) };

        index
    }

    pub fn deallocate(&mut self, index: u32) {
        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );

        let ptr = unsafe { self.memory.add(index as usize) };
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
        unsafe {
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
        let _id3 = allocator.allocate(22, None); // Should panic
    }

    #[test]
    fn test_pool_allocator_alignment() {
        #[repr(align(16))]
        struct Aligned16(u64);
        let _allocator: PoolAllocatorLite<Aligned16> = PoolAllocatorLite::new(4);
        assert_eq!(PoolAllocatorLite::<Aligned16>::align(), 16);
    }

    #[test]
    fn test_pool_allocator_reuse_order() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(4);
        let mut free_list = Vec::new();

        // Allocate all blocks
        let id1 = allocator.allocate(1, None);
        let id2 = allocator.allocate(2, None);
        let _id3 = allocator.allocate(3, None);

        // Free in specific order
        allocator.deallocate(id2); // Middle
        free_list.push(id2);
        allocator.deallocate(id1); // First
        free_list.push(id1);

        // Check LIFO order
        let new_id1 = allocator.allocate(4, free_list.pop());
        let new_id2 = allocator.allocate(5, free_list.pop());

        // We should get blocks in reverse deallocation order
        assert_eq!(new_id1, id1);
        assert_eq!(new_id2, id2);
    }

    #[test]
    fn test_pool_allocator_capacity_edge() {
        let mut allocator: PoolAllocatorLite<u32> = PoolAllocatorLite::new(1);
        let mut free_list = Vec::new();

        // Aloocate single block
        let id = allocator.allocate(42, None);
        assert_eq!(id, 0);

        // Deallocate and allocate again - should reuse the same block
        allocator.deallocate(id);
        free_list.push(id);

        let new_id = allocator.allocate(24, free_list.pop());
        assert_eq!(new_id, 0);
    }
}
