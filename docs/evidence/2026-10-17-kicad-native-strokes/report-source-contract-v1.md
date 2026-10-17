# Preserve authored report sources without making them native prerequisites

Candidate v3 closes the new exact source/aggregation tests, but drops the
existing report decomposition described by `PcbRegionExt`. Its first full-board
run remains inside the first `layer-sanity` after the worker accumulated
150.11 CPU seconds. That candidate is rejected and cancelled as incomplete;
its original 120-second assertion did not execute and is not relabelled a
pass. The original workload and both remaining gates are retained for v4.

The owning layer is the PCB region's report-view state, not native topology.
Retain authored source owners so finite projection can occur when explicitly
requested. Preserve source order, sample counts, exact authorities and source
diagnostics. Never claim that authored report components are disjoint; exact
material decomposition continues to use the kernel independently.

Before implementing, reproduce with completed v3 libraries: overlapping
authored rectangles require two report profiles but one exact union of area
six. Preserve independent geometry oracles separately from report-array
equality. Also exercise a finite prefix followed by a failed projection,
retry/recovery, mutation, cloned return buffers, concurrent report requests,
warm denied scopes, and strict/approximate/strict behavior. Failed or partial
projections cannot populate a successful cache. Native replacement must drop
the old source list and its diagnostic history. All earlier tests remain.

Retained sources are owned values with shared native geometry. Their lifetime
follows the aggregate owner. This introduces retained source/report state;
measure its costs. No entry-count, heap-byte, total-work, recursion-depth or
cancellation bound is added. External limits remain explicit containment,
not an internal resource guarantee.
