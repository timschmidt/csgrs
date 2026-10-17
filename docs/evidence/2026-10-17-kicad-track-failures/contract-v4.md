# Preserve full-board completion with native coincident tracks

The first candidate fails the unchanged HVP109A board and copper-neck gates:
segment 480 has coincident endpoints and positive width. The previous loader
silently omitted it; a failure-only change makes an ordinary board unloadable.
Do not retain that candidate or weaken either full-scale gate.

For a circular-aperture track whose two exact source endpoints coincide, the
represented set is a disk centered at that point with radius width/2. Reuse the
existing STRICT `geometry::circle_region` constructor, as pads and vias already
do. SourceScalar parses rationals, so equality of its exact values certifies
source coincidence; equality after binary64 projection is insufficient.
Check the shared budget around native construction and preserve its typed
failure. No new stroke kernel, tolerance, or sampled authoritative circle is
introduced. Other positive tracks keep the finite adapter and typed failures.

Add independently derived disk area and point-membership assertions, exact
decimal centers/radii beyond binary64 precision, source numeric aliases,
STRICT→APPROXIMATE→STRICT calls, and native downstream consumption. Retain the
existing coincident-source fixture, now requiring the native disk. A distinct
endpoint pair that merely collapses in binary64 must still fail, not become
that disk. Retain exact-zero-width, invalid-source and budget controls.

This extends the declared migration: formerly omitted coincident tracks now
produce native disks, and valid board geometry/findings can intentionally
change. Document every added source track and every report difference. Require
all previously materialized feature metadata and report-coordinate bits to
remain unchanged. Compare matched costs only on unchanged output contracts;
measure affected boards separately without calling earlier omissions a faster
successful operation. Keep the original complete workloads and their limits.

The two legacy fixtures remain byte-for-byte intact. The degenerate fixture
must now contain the exact disk and no non-copper or panel fallback. The
incomplete fixture still requires the typed missing-endpoint load failure and
its separate pad/via/zone checks. All original 26 native-stroke regressions
remain unchanged. General round caps and noncoincident exact stroke fidelity
remain open; this is not restoration of the rejected 35-file integration.
