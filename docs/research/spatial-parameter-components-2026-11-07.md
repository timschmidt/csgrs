# Spatial parameter components — 7 November 2026

This capability extension starts from published main
`8931c4261e2006a36cf1e722aa511c26397284e4`. Hypercurve owns component membership,
exact affine parameter charts, clipping and source-germ evaluation. HyperBRep
owns spatial chart transport, complete world-coordinate incidence and consumers.
The affected contracts are P14–P20, P25, P35–P37 and P39. This is an extension of
formerly unsupported input, not a behavior-preserving simplification.

Before implementation, the original identity tensor/native degree-one spline
fixture and independently selected zero reproduced `UnsupportedParameterLocation`.
Its unit-chart abscissa is zero on `[0,1/3]`, `3t-1` on `[1/3,2/3]` and `3-3t`
on `[2/3,1]`; the other coordinates vanish. Its complete zero locus is therefore
`[0,1/3] union {1}`. The selected query has no stored scalar coordinates. This
full fixture remains beside the independent inclusion and exclusion controls.

`CurveParameterLocation::Components` now reuses Hypercurve's existing parameter
components. Exact affine transport and closed-domain intersection preserve
selected endpoints, original denominator exclusions and source-span identity.
Tensor inversion intersects projection loci in the unchanged original source
atlas; continuous output requires coordinate coverage of all three spatial
axes. The native finite path retains its original world-coordinate validation.
Only unsupported native incidence retries the existing complete mapped path;
invalid input and actual resource failures still propagate.

The new component membership and evaluation methods make output consumable.
`point_at_component` validates membership and the current source chart, then
evaluates that source germ through the existing source/surface evaluator.
It does not trust an old query-point relation. Distinct one-sided UV germs
remain distinct even when a tensor maps them to the same world point.
Ordinary parameter-only evaluation at an ambiguous knot still fails explicitly.
Finite-only contact and topology consumers reject continuous results with typed
errors; they cannot silently treat them as absent. Query metadata is recomputed
after native replay; archive version 17 is unchanged.

The source freeze contains 3,803 tracked inputs. Its complete patch against the
base has SHA-256
`c59c4e3546bc2d7b552249fd5c2026e85f10176d91dac453c3e8d37a141ec877`.
Twenty-two final local commands pass with unchanged input hashes and verified
raw-log hashes. Full release Hypercurve records 3,178 successful test/doctest
harness reports and 37 existing ignores; all-feature BRep records 862 and IO
127. MSRV 1.95.0 BRep/IO records 989. Counts include child-process reports and
are not counts of distinct tests. Stable/MSRV strict Clippy, affected docs,
WASM compilation, fuzz-target compilation/Clippy, benchmark builds, formatting,
repository budget and spelling checks pass. Failed attempts and explicitly
canceled pre-correction suites remain separate from final success. Existing
solver-budget exhaustion/retry tests pass; cancellation and a total-work or
heap bound remain open. These checks do not replace the full pre-push matrix.

A fixture-derived pure Rust probe executes the original assertions on WASM
with the default stack and no host imports. Two models retain the mixed locus
and discontinuous germs. They replay in independent WASM memory, in native
execution, and in a fresh native process without constructing the fixtures
first. Further restriction/query and reflected nonuniform model transformation
consume the replayed definitions. Independent native construction also replays
in WASM. Semantic assertions precede the separate deterministic-byte check.
The 6,668-byte continuous model has SHA-256
`339654ff4c0d4538e6ad0834167f076af4a9c4b128b3682b84d417dddb919fe8`;
the 7,751-byte germ model has SHA-256
`06ad492417dce646f7788d52276201ebbb050879fd2263dbe96d3d2786bd9ec6`.
The probe's private transport does not qualify a public ABI or every WASM case.

All 15 unchanged ordinary exact geometry/material/replay workloads complete
with matching receipts, three fresh alternating process samples per variant.
Descriptive median candidate/baseline time ratios range from 0.9835 to 1.0577.
Fourteen cases have equal median allocation counts and cumulative requested
bytes. Covering cylinders differs by six median calls and 648 requested bytes,
within the observed process variation. These are not retained/peak heap counts.
Concurrent host validation prevents a statistical speedup or nonregression
claim. The formerly unsupported mixed inverse is not compared to completed
baseline work. A copied recipe description incorrectly called nonlinear recut
new; the assessment corrects that sentence, preserving raw samples and recipes.

Raw qualification, failures, source patches, probe sources and cost samples are
retained in the local evidence record
`docs/evidence/2026-11-07-spatial-parameter-components` at
`c2698dec9aae12a133731a761bc6430d441b0bee`, branch
`research-spatial-components-20261107`. The record contains 295 files and
17,228,892 bytes, SHA-256
`afad0fd0fc91abf2f60fcd796f4fa5d01ae4d48ff5520ed1498abd3e89f9e341`.
Its local parent is the unchanged pending BRep evidence payload. The
[archive index](../evidence-archive.md) distinguishes published and local
availability. Compiled binaries and third-party package bytes are excluded.
The separate evidence repository still needs explicit publication authorization.

Direct native spatial NURBS mixed inverse queries, punctured selected implicit
branches, general continuous overlap assembly, arrangements and solid closure
remain incomplete. All BRep work packages remain open or in progress. The
[support matrix](../../crates/hyperbrep/docs/SUPPORT_MATRIX.md) and
[implementation ledger](../../crates/hyperbrep/docs/IMPLEMENTATION_STATUS.json)
retain the distinction between target coverage and demonstrated completion.
