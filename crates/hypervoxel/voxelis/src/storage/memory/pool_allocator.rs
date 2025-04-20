use std::{alloc::Layout, ptr::NonNull};

#[cfg(feature = "memory_stats")]
use super::AllocatorStats;

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
    #[inline(always)]
    pub const fn block_size() -> usize {
        let size = std::mem::size_of::<T>();
        let min_size = std::mem::size_of::<*mut T>();

        if size < min_size { min_size } else { size }
    }

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

    pub fn get(&self, index: u32) -> &T {
        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );
        debug_assert!(
            (self.base_ptr as *mut T).align_offset(Self::align()) == 0,
            "Memory not properly aligned"
        );

        let ptr = self.index_to_ptr(index);

        unsafe { &*ptr }
    }

    pub fn get_mut(&mut self, index: u32) -> &mut T {
        debug_assert!(
            index < self.capacity as u32,
            "Block index out of bounds index: {} capacity: {}",
            index,
            self.capacity
        );
        debug_assert!(
            (self.base_ptr as *mut T).align_offset(Self::align()) == 0,
            "Memory not properly aligned"
        );

        let ptr = self.index_to_ptr(index);

        unsafe { &mut *ptr }
    }

    pub fn allocate(&mut self, value: T) -> u32 {
        if !self.free_blocks.is_null() {
            #[cfg(feature = "memory_stats")]
            {
                self.stats.free_blocks -= 1;
                self.stats.allocated_blocks += 1;
            }

            let ptr = self.free_blocks;
            let next_free = unsafe { *(ptr as *mut *mut T) };
            self.free_blocks = next_free;

            unsafe { std::ptr::write(ptr, value) };

            let index = self.ptr_to_index(ptr);

            debug_assert!(
                index < self.capacity as u32,
                "Block index out of bounds index: {} capacity: {}",
                index,
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
                "Block index out of bounds index: {} capacity: {}",
                index,
                self.capacity
            );

            let ptr = self.index_to_ptr(index);
            unsafe { std::ptr::write(ptr, value) };

            index
        } else {
            panic!("Out of memory");
        }
    }

    pub fn deallocate(&mut self, index: u32) {
        assert!(index < self.capacity as u32, "Block index out of bounds");

        let ptr = self.index_to_ptr(index);
        unsafe { std::ptr::drop_in_place(ptr) };

        let mut current = self.free_blocks;
        while !current.is_null() {
            if current == ptr {
                panic!("Double free detected");
            }
            current = unsafe { *(current as *mut *mut T) };
        }

        #[cfg(feature = "memory_stats")]
        {
            self.stats.free_blocks += 1;
            self.stats.allocated_blocks -= 1;
        }

        unsafe {
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
