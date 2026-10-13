An isolated DRC process-allocator trial is frozen against the unchanged
endpoint/P4 parent. The candidate uses pinned mimalloc for Rust allocations in
the CLI and standalone library-test process. It leaves allocator choice to
reusable-library consumers and does not enable C malloc override.

[V2 preparation](preparation-v2/protocol.json) verifies the 10,132-file baseline,
10,133-file candidate, two identical allocator controls and 150 dependency
archive/source inputs. Formatting and locked metadata pass. [V1 preparation
failure](preparation-failure-v1.json) is preserved: the local Cargo cache lacks
per-file checksum manifests, so V2 verifies the locked archives and every
archived source file directly. V1 stopped before copying sources or compiling.

Broad DRC qualification and both original native gates remain to run. If the
candidate passes both gates, the fixed application trial will compare complete
boards, full report bytes, timing, peak RSS, startup and executable size. No
speedup or runtime admission is claimed.
