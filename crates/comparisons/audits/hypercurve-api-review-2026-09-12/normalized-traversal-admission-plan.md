# Normalize and consolidate public arrangement admission

Parent: Hypercurve `2d26b0ae4d6178700d5c26ccdab5f8ac25a1f787`.

The independent public probes bind to the archived parent library SHA-256 `e314d2523910509b23aae886ead2a98aef68181343ec712968035bba2cd47032`. The retained-loop probe records eight admission failures (both policies: coincident even-odd loops, material/hole cancellation, nested material seams, adjacent material seams). The traversal probe records four admission failures (both policies: overlapping walks with canceled boundaries, self-crossing walks). Explicit normalization passes the independent expected memberships. These are failures of publication invariants, not mathematical counterexamples to the regularizer.

This increment consolidates the three public traversal factories into `CurveRegion2::try_from_arrangement_traversal`, returning the same immediate contextual result/outcome as the path constructors. Linear and rational overlap clients pass their refined graph and traversal directly. No compatibility wrappers remain. The public factory validates source connectivity/provenance, then uses the existing unary arrangement authority to publish its regularized even-odd set. The internal certified face-walk builder keeps its direct certificate-preserving route, avoiding redundant normalization of Boolean results. Authored cusp endpoint tangent hints are still invalidated after graph reconnection.

Migrate integration tests, benchmarks, and the region fuzz target. Assert normalized filled-left area and ownership rather than authored orientation or redundant collinear fragment counts. Retain mathematical area, selected evidence, source provenance, and invalid-connectivity checks. Add explicit public regressions for crossings, overlaps, duplicate cancellation, membership, certainty, and reentry.

Use `/tmp/hypercurve-region-admission-qualification`, with the pinned Hyperreal snapshot. Exclude the unfinished working `src/bezier_offset.rs` candidate from all builds and commits. Do not edit production/test/probe sources while a qualification process is live. Archive source manifests, the patch, actual binaries, and results; reap all handles before follow-up edits.

Remaining public raw-loop constructors and operation producers remain separate follow-up work. This increment does not claim that every region producer is normalized or that the full implementation goal is complete.
