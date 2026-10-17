# Existing test migration discovered by full validation

The first candidate reproduced two existing tests whose success assertions
encode the silent omission being removed by contract-v2. Preserve their
complete source fixtures without changing any numeric value or object.

- `degenerate_and_non_copper_geometry_is_ignored_without_fallback_shapes`
  includes a positive-width coincident track. Previously it expected an empty
  successful board. It will require the loader's typed unsupported-geometry
  error at segment index 1. Parse the same fixture's non-copper footprint and
  collinear panel graphic directly and retain their original empty assertions.
  The separate public zero-width tests continue to require valid empty loads.
- `incomplete_objects_retain_failed_pad_geometry_without_panicking` includes a
  track missing its endpoint. Previously that track disappeared before the
  test inspected the invalid pad. It will require typed MissingField(end) at
  segment index 0. Feed the same complete source fixture to the existing pad
  and zone parsers, and separately pass its unchanged via nodes to the via
  parser. Preserve the failed-pad, layer-sanity diagnostic, and empty-drill
  assertions. The full fixture is still rejected at the public boundary;
  these additional family checks cannot publish it as a BoardModel.

Rename those tests to state the new failure boundary. This is the explicit
source-failure migration declared before implementation, not a performance
claim or preservation of the old silent-skip behavior. Both original tests
and their failed run remain in the first snapshot. The independently archived
26-test native-stroke file remains completely unchanged and is replayed as-is.

An existing direct via-parser test also must handle the new Result with an
asserted success, rather than discard it. No runtime construction, policy,
fixture resolution, original workload gate or new acceptance expectation is
changed by the final test correction. Qualify the final source in a fresh
target with all affected checks again.
