# Public format-boundary reproductions

Owner: CSGRS `io::obj` and `io::stl`, with topology validation owned by Hypermesh.
Relevant properties: P1, P6, P17, P18, P26, P27, P29, P35 and P37.
These tests reproduce hypotheses on unchanged current sources before any fix.

The finite domain uses rational cubes whose coordinates are representable in
binary32/binary64, and malformed triangle indices. Numerical decision certainty,
closed topology, exact box volume and format projection are checked separately.
A successful exporter must not silently write missing vertex references, erase
nonzero finite projected coordinates through fixed decimal formatting, or report
successful binary STL output after binary32 quantization collapses faces.

For unit and small binary32 cubes, export -> import -> native strict topology
validation -> union with canonical empty must complete. Box boundary planes and
six times the expected exact volume supply an independent semantic oracle.
The tiny textual-coordinate cases check binary64 round-trip values explicitly;
they do not claim that an arbitrary decimal approximation preserves the original
exact construction or its provenance. The translated unit cube at 2^24 is valid
before export but cannot retain its faces in binary32; typed rejection is the
required outcome. Invalid OBJ indices must fail before `write_obj` writes bytes.

Positive unit/empty controls remain alongside every failing hypothesis. No test
changes policy or reduces an existing fixture. These are format reproductions,
not an external published-package pipeline or a complete format support matrix.

The reproduction also builds an unchanged-source OBJ cost executable, without
running timed comparisons yet. It covers canonical empty geometry, a unit cube
and 256 disconnected unit cubes. A later paired comparison must use identical
probe sources and retain all samples and full output bytes. Input construction,
strict topology/volume validation, one export warmup and output verification are
outside timing; each timed export includes complete string materialization, and
all result strings remain alive until timing ends. Reported string capacities
cover retained payload buffers only, not total allocations, native memory or
process peak RSS. This focused warm-export probe cannot establish application
speed or cold-cache performance.
