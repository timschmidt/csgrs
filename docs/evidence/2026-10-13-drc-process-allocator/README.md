The isolated DRC allocator candidate passes both original native gates with
mathematical sources unchanged. It selects pinned mimalloc in the native CLI and
library unit-test executable. Library consumers retain allocator choice; the
native allocator is a direct DRC build dependency. C malloc override is disabled.

[Qualification assessment](qualification-assessment-v3.json) verifies 34 distinct
commands: 27 preserved from V2 and seven new continuation commands. Both variants
pass two allocator controls, all 1,646 DRC tests, formatting, both strict Clippy
modes and warning-denied docs. Focused controls overlap the full suite;
independently linked integration/doctest drivers retain their own allocator.

The baseline board gate fails at 132.223696162 worker-CPU seconds / 120. The
candidate passes the unchanged 120-second gate, and both variants pass the
60-second copper-neck check-body gate. Passing assertions do not print their
internal durations. Both boards emit 12 and 197 findings; full report bytes still
require comparison. The candidate CLI grows by 158,256 bytes and its unit-test
executable by 21,608 bytes.

[V2 preparation](preparation-v2/protocol.json) verifies the 10,132-file baseline,
10,133-file candidate, identical controls and 150 dependency archive/source
inputs. [V1 preparation failure](preparation-failure-v1.json) records the absent
Cargo checksum manifest; V2 uses locked archives. [V2 linkage-reader failure](qualification-failure-v2.json)
is also preserved. V3 verifies each indirect GOT dispatch through its exact ELF
relocation to `mi_malloc_aligned`, without rebuilding or rerunning passed tests.

The committed application runner is ready for the unchanged fixed design: four
report preflights, 20 board samples and 10 batches of 50 help processes, recording
complete bytes, timing, maximum RSS and executable size. No samples have run yet.
No runtime admission or full-plan completion is claimed.
