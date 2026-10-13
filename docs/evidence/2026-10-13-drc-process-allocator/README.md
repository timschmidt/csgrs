An isolated DRC process-allocator trial is being prepared against the unchanged
endpoint/P4 parent. The candidate uses pinned mimalloc for Rust allocations in
the CLI and standalone library-test process. It does not install a global
allocator for reusable-library consumers or enable C malloc override.

Two shared allocator contract tests accompany broad DRC qualification and both
original native gates. If candidate gates pass, the fixed application trial
will compare both complete boards, full report bytes, timing, peak RSS, startup
and executable size. No speedup or runtime admission is claimed.
