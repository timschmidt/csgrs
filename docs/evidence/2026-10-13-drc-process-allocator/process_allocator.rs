// The executable or standalone test binary owns its allocator choice.
// The reusable library never installs a process-global allocator.
#[global_allocator]
static ALLOCATOR: mimalloc::MiMalloc = mimalloc::MiMalloc;
