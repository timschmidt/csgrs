# Fixed text cost comparison

Before observing candidate costs, retain both original README inputs (Asar CSG
at 72 points; ROWMANS CSG at size 1) and both original benchmark inputs (Asar
csgrs benchmark at 24 points; FUTURAL CSGRS at size 2). Build standalone drivers
against pinned baseline and qualified candidate libraries with the same
toolchain and optimization level. Each command constructs and materializes the
complete output and rejects empty/error results for these nonempty fixtures.

Measure three paired repetitions for each fixture in fresh cold processes and
in fresh processes with one identical untimed warmup. Alternate baseline-first
and candidate-first ordering. Retain every row, including unfavorable costs.
Run allocation instrumentation separately from uninstrumented timing, once per
fixture, variant and temperature. Record requested allocation bytes, peak live
requested bytes and post-drop retained deltas; these are not allocator overhead
or total process memory. Preserve process resource records and library/driver
byte sizes. Include completion and policy/certainty scope with the measurements.

Serialization happens outside the timed construction interval. Compare every
Hershey edge endpoint byte for byte to the original reproduction, including the
custom positive/negative fixtures. Check candidate determinism separately.
TrueType has an explicit representation and unit migration, so record its
completion and costs without claiming the old sampled polygons and new native
outlines are the same output contract or treating their cost ratio as a
performance improvement. Independent semantic tests admit the new geometry.

No optimization claim is made. This is proportionate focused cost evidence,
not a cross-library campaign or a total-work/heap/cancellation guarantee.
