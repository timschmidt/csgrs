# Direct endpoint evidence for region connectivity

Parent: Hypercurve `7af48343b93c7f2110e60d26a94c316aad996795`; shared dependencies retain Hyperreal `28eaac35e8c24006c532b25a393b7d430d03823c` and Hypersolve `d7cd3d9ffe33e6ef26c0c409dda986a88f16f7ef`.

The pending candidate removes the private `RetainedEndpointEvidence` bundle and `RetainedEndpointEquality` enum. The bundle eagerly built six optional views, including copied coordinate-root payloads and multiple source/point representations, before comparing an endpoint. Region connectivity now checks retained source and selected-circle identities before obtaining the existing general `CurvePoint2` projection and invoking the shared point equality predicate. Corner promotion and ray winding also obtain their endpoint values directly. All private callers are migrated; there is no alias or replacement point representation.

Positive identity shortcuts are retained for equal Bezier sources and source parameters, equivalent analytic parameterized curves, selected-circle overlaps, and mapped circle/chord endpoint incidence. Identity cannot decide inequality. Unrelated points use `CurvePoint2::same_point`; uncertainties remain explicit. Its coordinate-root comparison now accepts a certified unequal coordinate even when the other coordinate is undecided, preserving the old connectivity predicate's disproof capability at the shared point layer.

A coordinate projection used only by existing exact-coordinate oracle tests is compiled only for tests. No public API changes. The two existing tests that called the removed bundle now request the general endpoint projection directly; their geometry, candidate counts and set assertions are unchanged.

## Candidate history

- v1: first all-target check completed with one dead-code warning for the now test-only circle coordinate projection. The runner rejected this as unclean and reaped the build; zero tests executed. Immutable inputs remain in `/tmp/hypercurve-direct-endpoints-v1-2026-09-24`.
- v2: makes that projection test-only. Both all-target feature configurations pass without warnings. Release build and semantic qualification are pending; no closure or performance result is yet claimed. Inputs are bound by `direct-endpoints-20260924-v2-sources.json` in `/tmp/hypercurve-direct-endpoints-v2-2026-09-24`.

The earlier corner payload/loop-layout assertions remain unchanged in this production candidate. Their migration still needs exact geometry or interval-coverage oracles. All fourteen previously recorded nonpasses remain open unless fresh execution demonstrates otherwise. The larger API and four-closure implementation goal remains active.

### Admission review before qualification

The v2 candidate is not qualified for commit. Code review found that `validate_retained_source_endpoint_image` does not establish definedness for scalar retained endpoints; the old bundle construction incidentally evaluated them before any identity shortcut. Moving identity first can therefore admit two reversed copies ending at the same rational pole. The finite-endpoint obligation belongs in provenance admission, before connectivity. A follow-up candidate must enforce it there and test a reversed rational/conic pair with `t=1/2` as a true pole versus finite `t=1/4`. Existing source identity remains a positive connectivity certificate only for admitted affine endpoints.

The v2 public integration run completed: 19 of 20 passed; `homogeneous_boundary_closes_through_boolean_corners_and_offset` exceeded 90 seconds. A direct parent build/replay is running, so this is not yet classified as a regression or an existing limit. The full v2 library sweep is still running. All source bytes remain frozen until both runs are terminal and reaped.

The exact parent Hypercurve/Hyperreal dependency snapshot also exceeds the same 90-second bound on the homogeneous-boundary composition. Parent executable and source bindings are in `direct-endpoints-20260924-parent-path-{selection,terminal}.json` and the parent sources manifest. This establishes that the bound failure predates this refactor; it remains an open public composition/performance issue, beyond the fourteen library nonpasses.

### v2 complete comparison and the admission counterexample

The full v2 library inventory finished: 1,206 passed, six ignored, and the same fourteen pre-existing nonpasses as the parent. No previously passing library case regressed. The point/path integrations retained nineteen passes, with the parent-confirmed homogeneous composition exceeding 90 seconds. All processes were reaped.

The independent public-constructor control then confirmed the review finding. It passes against the exact parent release library and fails against v2: the reversed rational-line pair with a shared t=1/2 pole is wrongly admitted by the intermediate candidate. The control includes the line W=1-2t and conic W=(1-2t)^2, finite neighboring intervals, and both policies. Source/library/executable hashes and return codes are bound in `direct-endpoints-20260924-admission-control-terminal.json`; the control source is retained alongside it. Only bounded policy/family/range metadata is printed.

v3 moves the existing scalar endpoint-definedness check into `validate_retained_source_endpoint_image`, before connectivity identity. It adds that regression to the library and preserves all four source/range positive and negative controls under both policies. The exact reference-cutter test now uses the existing optional exact projection rather than demanding stored coordinates from the general point value. All 2,044 physical source inputs are frozen in `/tmp/hypercurve-direct-endpoints-v3-2026-09-24`. The final pass inventory will rerun every previously passing/ignored library case and the new regression; v2's fourteen unchanged nonpasses remain explicitly recorded, not relabeled as passes. The same nineteen public integration successes are rerun; the parent-confirmed 90-second composition limit remains open.

## Qualified and committed

Hypercurve `68bc4f1a5abae015f1908e40d4ea28a226db1502` contains v3. Both all-target feature checks are warning-free. The new pole regression, all 1,206 previously passing library cases and nineteen public integration cases pass; six library cases remain ignored. The fourteen unchanged library nonpasses were rerun on v2 and matched the parent, then explicitly carried forward for v3. The parent-confirmed homogeneous composition still exceeds 90 seconds and remains open. All 2,044 input bytes match the immutable final snapshot, staged content and committed HEAD. Hypercurve is clean and all owned processes were reaped before commit. Qualification, staged and post-commit records carry the exact bindings. No public API or compatibility interface was introduced; the full implementation goal remains active.
