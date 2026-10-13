use hyperreal::endpoint_cost_diagnostic as diagnostic;
use std::alloc::{GlobalAlloc, Layout, System};

pub struct Counted;

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            diagnostic::allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        diagnostic::deallocated(layout.size());
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let new_pointer = unsafe { System.realloc(pointer, layout, size) };
        if !new_pointer.is_null() {
            diagnostic::deallocated(layout.size());
            diagnostic::allocated(size);
        }
        new_pointer
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;
