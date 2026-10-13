The isolated DRC allocator trial keeps mathematical sources unchanged and selects
pinned mimalloc only in the native CLI and library unit-test executable. Library
consumers retain allocator choice; the direct dependency adds the pinned native
allocator to the DRC build graph. C malloc override is disabled.

[V2 preparation](preparation-v2/protocol.json) verifies the 10,132-file baseline,
10,133-file candidate, identical controls and 150 dependency archive/source
inputs. [V1 preparation failure](preparation-failure-v1.json) is preserved; V2
uses locked archives because this Cargo cache has no per-file checksum manifest.

Both variants pass formatting, both strict Clippy modes, two allocator controls,
1,646 DRC tests and warning-denied docs. The baseline fails the board gate at
132.223696162 worker-CPU seconds / 120 and passes the 60-second copper-neck body
gate. Passing assertions do not print their internal duration.

[V2 reader failure](qualification-failure-v2.json) preserves all 27 completed
commands. The candidate allocator shim uses a GOT pointer; V3 verifies its ELF
relocation to `mi_malloc_aligned` and continues only unfinished inspection and
candidate gates. Sources, binaries and assertions remain unchanged.

The conditional application runner is still unexecuted. Its V2 only updates the
qualification reference and versioned output paths. The original fixed design
remains four report preflights, 20 board samples and 10 batches of 50 help
processes, with full bytes, timing, RSS and executable size retained. Candidate
gate success is required before it starts. No runtime admission is claimed.
