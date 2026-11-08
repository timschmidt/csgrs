# Native NURBS parameter components — 7 November 2026

This capability extension starts at research commit
`827574f9aaaf538c9468ace5328d87b5334e9f39`. Hypercurve owns checked parameter
metadata; HyperBRep owns native spatial inversion, chart transport and source
point construction. The affected contracts are P14–P20, P25 and P35–P39.
The final 3,806-input source freeze has complete patch SHA-256
`830594e0d6b8d2263d0b8273a865816cbc1b4e01368ebe951c9ee9adc9c91de0`.
Later capability/admission edits change documentation only.

The original degree-one NURBS has x=0 on `[0,1]`, x=t-1 on `[1,2]`, x=3-t
on `[2,3]`, and zero other coordinates. Both represented and independent
selected zero queries reproduced `UnsupportedParameterLocation` after exact
piecewise evaluation. Its complete inverse is `[0,1] union {3}`. The smaller
three-control regression retains its original input and now demands exact
interval success. Native positive-weight and knot-multiplicity rules remain;
discontinuous native input still fails with `InvalidKnotMultiplicity`.

Native decomposition gives finitely many rational Bézier spans with positive
denominators. Within the existing complete finite span-inverse authority, each
span's point preimage is finite or the entire span: a nonzero univariate
polynomial cannot vanish on an interval. Exact affine charts transport these
sets. Native knot multiplicities establish positional continuity, so finite
knot visits can be retained on both incident charts. The result keeps proved
constant intervals, selected roots and source germs. Unsupported lower-level
incidence remains typed; no general algebraic inverse completeness is inferred.

`CurveLocationComponent2::from_span_parameters` checks a nonzero chart, a local
closure inside `[0,1]` and nonempty included singletons. It validates parameter
metadata, not geometric incidence. Finite-only consumers retain their explicit
blockers for unsupported continuous assembly.

An independent acceptance check found that the first prototype returned the
correct point value but retained a temporary Bézier source/local parameter.
That source identity failure is preserved separately from geometric success.
After chart validation, component evaluation now uses the original NURBS
evaluator. Selected points retain that source and the authored global parameter;
native positional continuity justifies their value at a shared knot. Scalar
parameters keep the existing coordinate view. The failed source, test-copy
compilation attempts, explicitly canceled prototype MSRV run and raw costs
remain distinct from final qualification. Cancellation is not exhaustion or
completed geometry.

Seven maintained native functions cover nonunit degree-one charts, positive
weighted quadratic spans, independent selected queries/interior parameters,
finite visits, negative membership, changed source/chart/index controls,
restriction/reversal/reflected affine continuation, cold replay and actual
solver-budget/retry behavior. For `q(s)=s(s^5+s-1)`, a degree-six span adds
the isolated visit `5+3alpha` next to `[2,5]`, where alpha is independently
selected and has no stored scalar view. Its finite Bézier source first
completes on both revisions; only the base native mixed query blocks.

Eighteen final commands pass with identical complete input manifests and
verified log hashes. Hypercurve records 3,180 successful test/doctest harness
reports and 37 existing ignores; all-feature BRep 869, IO 127, and MSRV 1.95.0
BRep/IO 996. Counts include replay subprocess reports. Stable/MSRV strict
owner/downstream Clippy, strict docs, WASM/fuzz checks and benchmark builds pass.
Final formatting, repository-budget and spelling checks pass separately.
This is scoped admission; the earlier spatial increment's 280-command run
qualifies a different revision. A new native pre-push qualification is required.

A fixture-derived pure Rust probe runs all seven functions on WASM with the
default stack and no host imports. Its native archive 17 model includes the
component-generated selected point itself. Fresh WASM memory and a fresh native
process validate its original NURBS/global parameter, exact membership, further
restriction/reversal/query and deterministic re-export. Independently native
construction replays in WASM; byte equality is checked after semantic assertions.
The 5,793-byte model has SHA-256
`203750bb026737770b45f0fc3ab6a043eaf22966b0b1cf0d06a9bfd92a4af68b`.
The private transport does not qualify a public ABI or every WASM workflow.

All 15 unchanged ordinary exact workloads complete with matching receipts in
three fresh alternating process samples per variant. Final descriptive median
time ratios range from 0.7718 to 1.1683; covering bores is the adverse 1.1683
row. Median cumulative allocations/requested bytes differ by +5/+472 for same
height, -5/-760 for covering cylinders and +3/+120 for covering bores; twelve
rows match. These are not retained/peak heap measurements. Four unchanged finite
inverse benchmark rows also complete with matching checksums. Concurrent host
validation prevents statistical speedup/nonregression claims; release performance
needs the unchanged complete cohort on a quieter host. Final prism/coplanar
executables grow 33,408 bytes and the finite-inverse executable 26,552 bytes.
This is a capability extension, without a source-reduction or size-optimization
claim. Earlier prototype measurements remain historical; differing final binaries
triggered a fresh full campaign. New exact mixed success is never compared with
the base's unsupported outcome as completed work.

Solver budgets count refinements/promotions, not total geometry work, coefficient
growth, heap or cancellation latency. Zero allowance does not imply exhaustion
when a cheap exact proof needs no counted solver work. No uniform resource bound
is claimed. The local record is
`docs/evidence/2026-11-07-native-nurbs-components` at
`70daed6650e35274bbdec2b86ae286fa2a22f343`, branch
`research-native-components-20261107`: 477 files, 24,644,143 bytes, SHA-256
`946785094fa4067d8d1f5ce139f795a158f3e176159926ba6465691b576ec3cf`.
All 438 local records and four historical excluded-package digest rows verify;
the pending integration payload is unchanged. The [archive index](../evidence-archive.md)
distinguishes local and published availability. Publication to the separate
evidence repository remains pending explicit authorization.

At this increment, the original punctured selected `y=sqrt(x)` image still
required `[0,1/2) union (1/2,1]` and returned unsupported. The
[8 November follow-up](implicit-guard-components-2026-11-08.md) records its later
constant-image completion. The original exact research probe
completes four denominator censuses, including y-dependent, endpoint and
independent algebraic roots. A fifth case, `F=(y^2-x)(y-2)` with `D=y-2` on
the positive sqrt branch, should have no denominator roots but remains unsupported
because an unselected component is shared. Resultant zero is not a complete
selected-branch census. General continuous assembly, operation closure, runtime
fuzz/coverage, resources and BRep delivery remain open. The existing
[support matrix](../../crates/hyperbrep/docs/SUPPORT_MATRIX.md) and
[implementation ledger](../../crates/hyperbrep/docs/IMPLEMENTATION_STATUS.json)
retain all historical increments and unchanged work-package statuses.
