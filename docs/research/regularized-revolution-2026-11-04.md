# R20: regularized segmented revolution, 4 November 2026

The retained mesh milestone adds `curve::revolve_solid` and the corresponding
scalar, C, C++, Python and WASM entry points. Legacy `revolve` and
`revolve_from` retain their partial swept-boundary meaning. Analytic/BRep
revolution, general exact-curve closure and internal resource bounds remain
open. The sanitizer campaign below did **not** complete its stress envelope.

## Contract and reproduced gap

The owning layer is CSGRS modeling composition, under P6, P17–P20, P25–P27
and P30. Hypercurve remains the region authority and Hypermesh the 3D union
authority. Baseline `9a8bd78184` reproduces both intended legacy behaviors:
opposite-side partial sweeps can have winding two, and an axis-crossing loop
is invalid. An aligned overlapping pair can also fail legacy mesh validation.
Two new regularized-material witnesses failed before the implementation; the
original 24 profile-sweep tests were retained unchanged.

The new operation uses the existing profile conversion and angular domain:
nonzero partial angles of magnitude below 360°, or proved integral full turns;
at least two segments for partial turns and three for full turns. Full turns
cover the sampled set once. Each signed radius keeps the placement
`(x cos(t), y, x sin(t))`. Exact line profiles retain native coordinates.
Nonlinear profiles retain the existing sampling and projection qualifications;
no analytic fidelity or topology guarantee is added for that sampling.

Material and holes are split by the existing exact axis-region operation.
Their one-sided polyhedra use the same authored angular samples and are
united by the native regularized Boolean. A planar fold is used only for a
full turn with an even lattice. For odd lattices or partial sweeps, folding
would change the negative side's phase and is not used.

The construction argument is local to this segmented model. Consecutive
angular samples differ by less than π under these segment minima. Away from
the axis, each signed radial band maps injectively into its angular sector;
the profile's material/hole ownership is retained. Bands on one side have
disjoint interiors, and the arrangement union of the signed sides selects
material once. Axis vertices are shared; lower-dimensional contacts follow
regularized closed PWN semantics and need not be manifold. This argument
does not establish an implementation proof for arbitrary symbolic profiles.

Canonical empty input gives canonical empty output after parameter validation.
Invalid angles, unsupported profile conversion and undecided predicates keep
their existing typed channels. A platform-addressability preflight prevents
an impossible angular allocation with `CapacityOverflow`; no internal total
work, retained-memory or cancellation bound is introduced.

The affected capability record is deliberately narrower than the
[exact-curve target](hypercurve_exact_curve_closure_plan.txt):

| Capability | Demonstrated milestone |
| --- | --- |
| Representation | Native segmented mesh; exact line-profile coordinates and signed phase retained |
| Evaluation/differentials | Existing profile conversion; no new curve evaluation or differential algorithm |
| Incidence/order | Existing exact axis split and 3D arrangement; independent winding checks |
| Region admission | Existing `CurveRegion2` input semantics; output is a closed PWN mesh |
| Booleans | Signed-piece union, subtraction, restoration and canonical empty continuation |
| Repeated offsets | No new qualification |
| Corner edits | No new qualification |
| Reconstruction/transforms | No new qualification; legacy explicit-start operation unchanged |
| Exact replay | No new native replay qualification; STL continuation is explicitly projected |

## Acceptance evidence

The final core passed 687 root tests/doctests with seven existing ignored
tests. This includes all 30 profile-sweep tests and all 576 matrix rows in
674.46 seconds. The matrix covers six source families, both policies,
±179/180/181/270/359/360° and segment counts 3/4/5/6: opposite annuli,
axis crossings, crossing holes, a hole ending at the axis, disconnected pieces
and axis contacts. Author orientation alternates. Each row checks at least
70 exact membership queries against independently derived sector trapezoids
and at least 50 independent oriented cap-ray winding counts. Certified
support-zero points are outside this interior oracle; unknown signs are not
silently skipped. The test runs every row by default.

Additional checks retain sub-binary64 features, exact volumes 27 and 31 for
the two missing witnesses, independently authored signed-piece union, and
successful output after its profile is dropped. Subtraction, restoration and
empty symmetric difference consume the result across strict/approximate/strict
policies. STL artifact replay retains projection provenance and is tested as
the parsed model, without claiming general preservation of native coordinates.
Focused checks after the full suite extend the segment boundary to partial
N=2 and invalid full N=2, and add the independently authored piece-union check.

All four scalar adapter families preserve success, certainty and original
failure causes. FFI's 13 unit tests, actual C/C++/Python consumers, and the
generated JavaScript/TypeScript suite pass. Stable all-target/all-feature
Clippy, FFI Clippy and MSRV 1.95 default/dispatch-trace Clippy pass with warnings
denied. Warning-denied all-feature rustdoc, the new fuzz target's Clippy and
SynapsCAD `1f1ec3e66d` all-target/dispatch-trace Clippy also pass against the
candidate. The downstream application source remains unchanged; its existing
whole-CI text timeout is not requalified by a Clippy pass. The standalone fuzz
crate's stale lockfile was reproduced on the
baseline and reconciled: 61 unused codec-related packages removed, none added
or upgraded. Locked checks of every fuzz binary pass.

The new fuzz target leaves the old sweep decoder and manifold assertions
unchanged. It checks six bounded integer source families, certainty, closed
PWN validity, retained native evidence and independent winding. The original
576 seed files remain available in the evidence record. Its instrumented
address-sanitizer run at a 15-second input timeout and 2,048 MiB RSS limit
**timed out** after ten executions, with peak process RSS 1,473 MiB. Seed
`03 04 02 01 01 01 01` describes the axis-touching hole at 359°, N=5 under
the approximate policy. The stack is in exact mesh-union triangulation's
lexicographic edge selection. No memory error was reported before termination;
this is not a passing sanitizer campaign or typed internal exhaustion.
The same geometric matrix row (325) completes independently in release in
6.61 seconds, with 108 triangles, 108 membership and 108 winding checks.
The unchanged stress seed completes in the ordinary release fuzz executable
in 2.18 seconds with 40,088 KiB maximum process RSS; this fixed-input replay
does not exercise sanitizer instrumentation. All 576 original seeds also
complete in the ordinary release executable in 221.61 seconds, with 44,560 KiB
maximum process RSS (577 calls including the empty initialization input).
That is fixed-corpus execution without coverage instrumentation or a mutation
search. Two explicit ASan smoke seeds
(overlapping annuli and an axis-crossing rectangle at 270°, N=3) complete with
116 MiB peak process RSS. They do not replace the full corpus campaign.
The 15-second stress failure and seed are retained, without raising its limit
or weakening the corpus. A first sandbox run instead failed because
LeakSanitizer could not inspect its own threads; that environmental attempt
is retained separately from the real host timeout.
The instrumented build emits nightly's `recursion_depth_exceeding_limit`
warning on Hypercurve's recursive region auto-trait evaluation. A fresh
nightly check reproduces it on the unchanged `9a8bd78184` source archive.
Stable/MSRV checks are warning-clean; the nightly compatibility issue remains
open, with no unsafe auto-trait implementation or lint suppression added.

## Cost and disposition

Three alternating baseline/candidate process pairs, 20 warm operations per
fixture, measure five successful legacy controls with the same allocator
instrumentation and output materialization. Every exact serialized coordinate,
index, receipt and evidence row is byte-identical; the shared row digest is
`b8c114723e05da52d184ba88b2828d5ba6e363934d3cfa08f5c22f4be84d94bc`.
Allocation counts, requested bytes, peak logical live bytes and retained bytes
are identical. Median timings include adverse small movements; see
[PERFORMANCE.md](../../PERFORMANCE.md#r20-regularized-segmented-revolution).
These shared-host samples do not establish a statistical latency guarantee.

The new opposite-side material workload completes in about 195 ms with
208,222,247 requested allocation bytes, 819,378 peak logical live bytes and
197,440 retained bytes. These differ from RSS and from a total resource bound.
Its legacy winding-two boundary is a different contract, not an equivalent
performance comparator. The crossing rectangle takes about 986 µs. All
measured results release their logical retained allocations on drop.

The additive mesh API is retained for its tested set semantics and consumer
closure. The sanitizer stress envelope, triangulation cost, arbitrary profile
coverage, analytic/BRep continuation and internal work/memory/cancellation
control remain open research. No push or publication is part of this checkpoint.
Raw failures, complete results, source attribution, probes, locks and the
stress seed are archived through the [evidence index](../evidence-archive.md).
