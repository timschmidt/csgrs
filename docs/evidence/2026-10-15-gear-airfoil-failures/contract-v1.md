# Sampled gear, rack and airfoil failure contract

Target: `f1dab2248` plus the unchanged 35 pending native files. The preceding
turn made verified progress on Hilbert clipping; those native files remain
separately unadmitted.

Owning layer: CSGRS owns parameter validation, sampling, buffer sizing and
composition into the native Hypercurve ring constructor. Hyperreal owns scalar
domain errors, Hyperlimit owns policy decisions, Hypercurve owns topology, and
the existing extrusion functions own mesh conversion. Reuse those layers.
Properties: P1, P5, P6, P8, P9, P17–P20, P25, P27, P35, P37, P39.

Migrate `involute_gear`, `cycloidal_gear`, `involute_rack`, `cycloidal_rack` and
`airfoil_naca4` to typed results with context-bearing forms. Preserve their
existing admissible sampled sets and sampling resolutions. Strict is the
default; approximate policy does not imply approximate consumption. Preserve
native causes and aggregate receipts through spur and helical solid wrappers.
No failure may become successful empty geometry or a panic. Invalid counts,
invalid dimensions/derived flank geometry, undecided predicates, scalar errors,
capacity overflow and allocation failure remain distinguishable where applicable.

The existing domain is explicit: positive modules and nonnegative clearance
and backlash; involute pressure angles strictly between 0 and 90 degrees;
at least four involute gear teeth, three cycloidal gear teeth, one rack tooth;
at least two gear flank segments and four cycloidal rack flank segments.
Derived root radii, tooth widths, gap widths and first-lobe ordering must satisfy
their existing inequalities. Airfoil camber and position are in [0,10), thickness
in (0,100), chord positive and sample count at least ten. Positive camber requires
positive camber position. Zero dimensions outside this domain fail explicitly.

These outputs are finite sampled polygons, not certificates of fidelity to an
analytic involute/cycloid/NACA surface. Preserve the legacy NACA coefficient
values (represented binary64 constants) and its existing ring closure in this
failure-only migration; do not silently change exact output under that name.
Any analytic-fidelity/coordinate migration requires separate evidence.

Before code changes, reproduce invalid/undecided empty results, capacity panics
or hidden overflow and successful empty solid wrappers. Pin the current source
graph and native build. Retain original README and benchmark inputs at their
full resolution; compare exact serialized coordinate expressions before/after
as a preservation check, separately from independent mathematical evidence.

Acceptance requires an independent rack area and point-membership oracle,
native profile topology checks, empty-set composition, exact downstream
extrusion and topology/volume where supported; all five public families and
three solid wrappers must preserve typed failures and context receipts. Keep
strict→approximate→strict cases, original regressions and useful full workloads.
Do not weaken a fixture, policy or previously certified success.

Compute checked sizes before input-controlled sampling and use existing checked
reservations. Capacity checks are not full work, coefficient-growth, retained
heap-byte or cancellation budgets. Dangerous capacity tests use values that
must fail arithmetic/byte-size checks before allocation, with external
1800-second/8-GiB containment. Broader resource guarantees remain open.

Run affected focused tests, root curve/all-feature suites, Clippy, fmt, docs,
doctests, actual benchmark callers and example builds after the final edit.
No exposed C/WASM operation names these constructors, so do not claim a real
binding rerun. Preserve all measurements and failed trials. The release path,
pending native cost gates, historical artifact loss and full plan remain open.
