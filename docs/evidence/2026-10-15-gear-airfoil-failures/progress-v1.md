# Sampled gear, rack and airfoil failure migration

On `f1dab2248` plus the unchanged 35 pending native files, the five sampled
constructors hid invalid, undecided and excessive-count inputs as 22 empty
profiles and five capacity panics. The three gear solid wrappers also published
six Certified empty meshes for invalid profiles. The target-revision
[reproduction](baseline-findings-v1.md) retains those failures before editing.

`involute_gear`, `cycloidal_gear`, `involute_rack`, `cycloidal_rack` and
`airfoil_naca4` now return `Result`. Their `_with_context` forms use shared
`GeometryDecisions` and native ring construction, returning operation-local
certainty. Scalar and native errors propagate; checked capacities/reservations
replace unchecked sampler buffers. Spur and helical gear wrappers consume the
profile and extrusion receipts together. Invalid profiles fail even when the
requested extrusion thickness is zero. README signatures, domains and callers
describe the migration.

This changes the failure/API contract while preserving successful finite
sampled sets. All ten original full-resolution README and profile-benchmark
fixtures have byte-identical serialized coordinate expressions. The NACA
coefficients remain their original represented binary64 values, with the
existing trailing-edge closure. No analytic-surface fidelity bound is claimed.

Acceptance follows [the declared contract](contract-v1.md), the repository's
`properties.md` and `docs/admission.md`. Independent rack area
`15*pi/4 + 29*sqrt(3)/24`, tooth/gap/backing membership, represented NACA sample
ordinates and gear radial witnesses establish semantics separately from the
serialization comparison. Translation, union with empty, exact extrusion,
closed-manifold topology and independently expected volume pass; self-subtraction
produces an empty profile that remains consumable by extrusion.

The focused suite exercises all five families and all three solid wrappers.
Invalid input, undecided validation and capacity exhaustion remain distinct.
The repeated strict/approximate/strict cases show that approximate facts are
not promoted by cached rings or solid composition; approximate policy on
certifiable original fixtures still returns Certified. These are local
dependency-chain receipts, not complete end-to-end construction provenance.

The completed phases are retained in full:

| Phase | Result | Scope |
| --- | --- | --- |
| `reproduction-v1` | 3/3 commands pass as diagnostics | Baseline defects, ten full fixtures and independent rack oracle |
| `candidate-v1` | 3/3 commands pass | Private build, workspace fmt, seven focused semantic tests |
| `qualification-v1` | 11/11 commands pass | 260 curve tests; 367 all-feature tests with one existing ignored test; warning-denied Clippy and docs; one doctest; both affected benchmark builds; README example build; public probe and ten exact fixture comparisons |
| `resources-v1` | 46/46 commands pass | Warning-denied allocation-driver builds, calibration, twenty allocation processes, twenty timing processes and two feature catalog processes |

The final source in the focused and broad qualification snapshots is identical.
The resource snapshot adds only the standalone allocator driver, whose separate
builds, calibration and executions validate that instrumentation. The driver is
never linked into the shipped library. The shared benchmark harness and embedded
font are pinned. E16's source guard replaces E15's standalone reproduction probe
with its own, while preserving every shared production input. Raw commands,
features, environment, compiler, source hashes, patches and dependency/executable
hashes remain in the phase records.

The fixed [resource contract](cost-contract-v1.md) uses five alternating pairs
per cold/warm condition on the original five profile benchmark cases. All 100
timing rows preserve work units, output size and checksums. The two affected
whole-catalog cases also complete on both binaries with matching output fields;
their four rows are completion checks. Their mixed-policy catalog is not a
whole-workflow certification or matched performance claim.

Candidate/baseline latency ratios below show the median of five paired ratios
and the full observed range. Ratios above one are adverse and remain recorded.

| Original constructor | Cold median [range] | Warm median [range] |
| --- | ---: | ---: |
| Involute gear | 1.020 [0.948, 1.034] | 0.876 [0.808, 1.042] |
| Cycloidal gear | 1.014 [0.980, 1.122] | 0.995 [0.946, 1.027] |
| Involute rack | 1.012 [0.915, 1.068] | 0.914 [0.782, 1.166] |
| Cycloidal rack | 1.065 [0.939, 1.088] | 0.965 [0.804, 1.536] |
| Airfoil | 1.020 [0.975, 1.035] | 1.003 [0.975, 1.012] |

This is cost disclosure for a correctness migration, with no optimization
claim. The cold cycloidal-rack median is 6.5% higher, and the single warm pair
at 1.536 remains included. No unfavorable sample was removed, and the matrix
was not extended after observing results. Cold uses a fresh process without
explicit warmup and one timed iteration. Warm uses eight warmup iterations
and eight timed iterations. Shared scalar and OS caches are not evicted
between cases. Timing includes parameter creation, construction, materialized
results, component counting and disposal, with all-feature dispatch tracing.

The standalone allocation probe measures each case/condition/variant in a
separate process, with one construction/drop warmup for warm allocation runs.
The following candidate-minus-baseline deltas are the same cold and warm,
except where separate peak values are shown. All twenty complete rows remain
in [the checked summary](qualification-summary-v1.json).

| Constructor | Allocation / reallocation count delta | Allocation / reallocation requested-byte delta | Peak additional requested-byte delta, cold / warm |
| --- | ---: | ---: | ---: |
| Involute gear | -4 / -2 | -448 / -1024 | +2208 / +1792 |
| Cycloidal gear | -4 / 0 | -224 / 0 | +5616 / +1920 |
| Involute rack | 0 / 0 | 0 / 0 | 0 / 0 |
| Cycloidal rack | 0 / 0 | 0 / 0 | 0 / 0 |
| Airfoil | 0 / -1 | +7584 / -15552 | -192 / +1296 |

Retained requested-byte deltas after dropping each region are unchanged in
all ten comparisons. Requested bytes exclude allocator metadata and allocations
outside Rust's global allocator. Peak requested bytes are measured above the
pre-construction live baseline; they are not process RSS or heap-byte limits.
Across the twenty timing processes, baseline RSS ranges from 24,680–25,184 KiB
and candidate RSS from 24,360–25,096 KiB. The allocation processes range from
12,936–17,676 KiB and 13,088–17,824 KiB respectively. RSS includes untimed setup.
No affinity, frequency lock or isolation from unrelated processes is claimed;
the [host snapshot](cost-host-v1.json) records the environment.

The all-feature `libcsgrs.rlib` grows from 9,863,140 to 9,910,970 bytes (+47,830).
The profile benchmark grows by 56 bytes, and the feature benchmark shrinks by
10,552 bytes. These are matched local artifacts, not a cross-platform package
or release-size claim. Their complete sizes and hashes are in the summary.

Final closure audits every inherited artifact expectation, both private E16
overlay targets and their uppers, all immutable E15 lower files, and standalone
probe/fixture outputs. The inherited 27,093 missing paths retain their expected
hashes; the audit requires no new loss or mismatch. Only `current_work` in
`docs/properties-status.json` may change; bytes outside that value and every
other decoded key must match `9025de62`. The obsolete scalar-integration
`checkpoint_v4.py` remains unexecuted and unstaged.

The 35 pending native files remain separately unadmitted. Their prior runtime
gates, combined library regressions and seven Hypercircuit fixtures are not
discharged by these root-suite passes. Legacy text failures, remaining adapters,
full resource bounds/cancellation, end-to-end provenance, external package and
release validation, formal/fuzz/coverage work and the complete properties plan
remain open. Checked sampler buffers do not bound all native work, expression
or coefficient growth, retained heap bytes or cancellation; dangerous tests
use external 1800-second/8-GiB containment. The render example was built, not
fully rendered, and no new C/WASM operation is exposed by this patch. No full-CI,
release, push or full-plan completion is claimed.
