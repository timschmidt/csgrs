# Reproduction on 00e612307

`reproduction-v1` is retained as a failed diagnostic build: its driver used a
nonexistent free `solid::translate` function. No behavior evidence is credited
to that build. The versioned v2 driver uses the public `SolidExt::translated`.

`reproduction-v2` compiles and its ordinary probe succeeds. It demonstrates:

- All three strict wrappers turn invalid-index errors into successful empty
  meshes. Zero direction and an undecided grid also become successful empties.
- Zero-copy layouts accept malformed indices and open sources. One-copy and
  disjoint two-copy layouts publish open meshes as locally certified success.
- Overlapping open input and an undecided Boolean return `Geometry(String)`;
  their native diagnostic source and meaningful failure category are lost.
- Grid count multiplication overflow is classified as invalid arguments, and
  a representable count whose vector byte capacity overflows panics.
- Concatenating a cube before a one-point mesh with index `usize::MAX` publishes
  index 7 after release-mode arithmetic wraps. The invalid source index is
  replaced by an apparently valid reference into another operand.
- Canonical empty distributions complete in all ordinary zero/one/two-copy
  cases; these successful results must remain supported.

The separate enormous-count linear and arc children both print allocation
failure diagnostics and terminate with exit 137 under the recorded external
512-MiB/10-second containment plus two-second kill grace. They do not return a
typed error. This intentionally makes the overall diagnostic phase fail; it
is not a successful workload or an internal resource guarantee.

The complete original workloads succeed and their exact coordinate/index JSON
is retained: linear 8 (64 positions/96 triangles), grid 4x4 (128/192), arc 12
(96/144), duplicate concatenation (16/24), and the feature benchmark's 22-copy
concatenation (176/264). Counts and byte comparisons are preservation evidence;
independent set, volume and topology checks remain required for acceptance.

Implementation will add native checked concatenation using Hypermesh's existing
index validator and preserve admissible retained closed-PWN facts. A native `validate_closed_pwn` entry point will share the Boolean soup
validator, including the combined certainty of face nondegeneracy and edge
balance. CSGRS will use that entry point before distribution shortcuts without
introducing unnecessary bounds queries or materializing polygons. This keeps
topology validation in its owning kernel and preserves native causes/receipts. Capacity checks do not
establish total-work, scalar-growth, secondary-allocation or cancellation bounds.
