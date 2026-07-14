# Voxelis Memory

`voxelis-memory` supplies fixed-capacity pool allocators used by the Voxelis
SVO-DAG interner.

## API

- `PoolAllocator<T>` owns its free list and reuses deallocated indices.
- `PoolAllocatorLite<T>` accepts a caller-managed free index, avoiding an
  internal free-list pointer.
- `AllocatorStats` is available with `memory_stats` and reports capacity,
  allocation, and free-block counters.

```rust
use voxelis_memory::PoolAllocator;

let mut pool = PoolAllocator::new(16);
let index = pool.allocate(42_u32);
assert_eq!(*pool.get(index), 42);
pool.deallocate(index);
```

Both allocators reserve their full capacity at construction and panic when the
pool is exhausted. Indices are valid only while allocated; callers must not
read an uninitialized or deallocated slot, deallocate a slot twice, or reuse a
`PoolAllocatorLite` index that is still live. Values must be deallocated before
the allocator is dropped when their destructors matter.

## Features

- `memory_stats` exposes `AllocatorStats`.
- `tracy` adds Tracy spans to the lightweight allocator.

## Development

```sh
cargo test -p voxelis-memory --all-features --locked
cargo clippy -p voxelis-memory --all-targets --all-features --locked -- -D warnings
```

## References

- [Rust `Layout` documentation](https://doc.rust-lang.org/std/alloc/struct.Layout.html) defines the size and alignment contract used for pool allocations.
- [The Rustonomicon: Allocating Memory](https://doc.rust-lang.org/nomicon/vec/vec-alloc.html) explains the raw-allocation invariants behind contiguous storage.

The allocator design supports [Voxelis](../voxelis/README.md). See
[HyperVoxel](../README.md) for the exact-aware voxel layer and wider Hyper
ecosystem.
