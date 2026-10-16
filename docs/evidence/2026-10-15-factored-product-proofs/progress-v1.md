# Bounded shared-factor proofs for complete drill composition

The scalar candidate completes all original strict 8/16/24/32-segment drilled
body workflows: empty accumulation, subtraction to canonical empty and reunion.
The original independent membership, polygonal-volume enclosure and manifold
checks also pass. `candidate-v2/result.json` records all 22 commands passing.
This is a completeness extension, not a performance-only change or a claim
that the complete properties plan is satisfied.

## Contract and implementation

The owning layer is Hyperreal's exact rational normal-form recognition, under
P2–P5/P7–P9 and the downstream composition requirements P17/P18/P20/P21/P24/P25.
The public documentation is `Real::exact_rational_normal_form`. The initial
contract and target-revision reproduction are in `baseline-v1.md`, the phase
protocols, and the preserved baseline failures.

The remaining 32-segment predicate had a degree-eight exact-zero expression
whose expanded form needs 145 terms, above the existing 64-term limit. Rather
than increase that limit, the candidate retains individually admitted polynomial
factors, matches factors common to every summand, and expands only the residual.
Only a proved-zero residual certifies the original sum. This uses the identity
`C * 0 = 0`; no nonzero assumption about C is required. Every operand, including
any inverse inside C, must still pass the original domain checks.

The same recognition attempt retains its remaining limits: 512 nodes, depth 64,
32 atoms, 64 terms, degree eight, 4096 coefficient bits and 8192 work charges.
Addition, negation, power-of-two scaling and literal rational scaling may be
distributed; general products of sums are not distributed. Matching preserves
factor multiplicity. There is no persistent negative cache or larger retry
budget, and authoritative expression construction/serialization is unchanged.
These local bounds do not establish an internal total-query work or heap-byte
bound. Recorded commands use external wall/address-space containment; Node/WASM
retains wall containment without an address-space cap because of its virtual
memory reservations.

Five added scalar tests exercise the large zero identity, outer signed
1/10^12 displacements, a nonzero residual with an independently derived sign,
scaled nested product sums, invalid/unproved inverse operands, degree-nine
decline, work exhaustion, cancellation and recovery. The independent rational
interval calculation in `residual_oracle_v1.py` establishes `6 < z < 7` and
`-1 < h < -1/2`; the perturbed result is `-epsilon*z*h^2`.

## Downstream meaning and certainty

The broader matrix exposed existing assertions that required permanent
uncertainty even after new exact proofs became available. Their failures are
retained in qualification-v1 through v4 and downstream-frontier-v1. Replays
against both the preserved 045621eba graph and E12 establish that several of
these stronger results predate E12. The graphs also include the explicitly
recorded pending native worktree; this is not a claim of a clean published
baseline or admission of those native changes.

The original e/pi two-row solver, all 22 representation cases, full four-tooth/
eight-sample rack, 16-segment crescent, 16-segment scalar/JavaScript through-drill
and rotated concave VRML polygon remain in the tests. Their acceptance now
requires stronger receipts and independent semantics where applicable:

- Exact row and aggregate solver residuals, with all 22 sparse and pattern
  preserving solver cases completing.
- Eight rack pitch crossings and exact boundary/inside/outside witnesses at
  y offsets of 1/10^12; redundant authored vertices may be compacted by the
  documented native constructor. Existing tip/root checks remain.
- Certified crescent construction and projection with independent membership.
- Certified scalar and JavaScript drill results under both policies, manifold
  geometry where exposed, independent material/hole/outside points and the
  original 144-triangle binding artifact check.
- Certified triangulation of the unchanged VRML polygon, with three triangles,
  no ignored degeneracies and a separately certified area enclosure within
  1/10^12 of its independently derived area 3. This is an enclosure, not exact
  area equality or closed-solid validation.

Additional unresolved `sin(e)^2 + cos(e)^2 - 1` fixtures preserve terminal-policy
coverage: strict Unknown/error, qualified approximate completion, then strict
Unknown/error after reuse. Selecting an approximate policy alone does not make
an exactly proved result approximate. Boundary contracts and standalone
replays are linked in the adjacent `*-contract-v1.md` files.

The real WASM consumer also exposed a stale expectation at the original
opposite-rotation curve boundary `(0.5,0)`. That point now requires Certified
boundary under both policies, with independent inside/outside checks. The
same square's boundary `(1,0.5)` remains Uncertain/qualified Boundary/Uncertain
and is retained as additional policy coverage. The original mesh uncertainty
case is unchanged. `wasm-rotation-v1` records the actual module's receipts and
`wasm-rotation-contract-v1.md` states the independent rotation identity and scope.

Documented compaction of redundant vertices can change newly constructed
artifacts across proof-capability revisions. No cross-revision byte-preserving
geometry optimization is claimed. Deterministic serialization and semantic
equality remain separate contracts.

## Cost evidence

`cost-contract-v1.json` freezes source copies, dependency hashes, toolchain,
hardware, flags, workload construction, warmup and five alternating sample
pairs. The baseline is 7d69ef51d (the last broadly qualified scalar before the
E7–E12 normal-form extensions); the candidate is the recorded E12 scalar. The
unchanged earlier control probe and the new normal-form probe are compiled with
identical flags and a counted System allocator. `cost-summary-v1.json` retains
all 380 raw rows across 38 workload/mode combinations, including unfavorable
rows. The shared host and short instrumented samples limit timing inference.

Every previously completed control stays certified. Allocation counts,
requested bytes, additional peak live requested bytes and retained byte deltas
are identical for all 24 control combinations. Median latency changes span
-1.81% to +2.73% with rebuilt inputs and -1.87% to +14.67% with retained inputs.
The unfavorable retained rational control rises from 180.21 ns to 206.65 ns;
it is retained, not excluded or presented as a speedup.

Each of the six newly supported normal forms certifies all 256 queries in every
candidate sample and both construction modes; the baseline returns Unknown
for all. Their additional work buys completion and is not comparable as a
speedup over the baseline's incomplete answer. Candidate median query costs
range from 16.41 to 474.66 microseconds. The largest measured additional live
requested allocation is 83,968 bytes; all measured retained deltas are zero.
These allocator-request quantities are not process peak memory or global heap
bounds. Per-command `time -v` reports are retained separately.

The unresolved trigonometric control remains Unknown on both revisions. Its
retained-input median rises from 0.79 to 2.40 microseconds, with allocations
24→75 and requested bytes 1312→3514. This unsuccessful-query overhead remains
an optimization opportunity, not successful-completion throughput.

ELF text grows by 4,184 bytes for the unchanged control executable and 56,993
bytes for the normal-form executable. The matched Hyperreal rlib grows from
35,635,970 to 37,063,280 bytes (including metadata/debug information). Exact
artifact hashes, section sizes and all raw allocation/timing data are retained.
No downstream or cross-library speedup, general leak-freedom or full runtime
admission is claimed.

## Final checks and remaining work

All 13 Rust commands in `qualification-v5` passed: formatting; 924 default and
1031 all-feature scalar tests; 1545 foundation tests; 240 native mesh tests
(seven ignored); 2493 native curve tests (33 ignored); 102 minimal, 237 curve
and 344 all-feature root tests (one ignored in the latter); selected all-target
Clippy with warnings denied; and 28 scalar doctests. These overlapping feature
matrix counts are not a count of unique tests.

The original phase remains failed because its last command rejected the global
wasm-bindgen CLI 0.2.122, while the unchanged Cargo.lock requires 0.2.126.
`wasm-tools-v1` installs the locked CLI into a separate target and retains its
source archive, lockfile and binary hashes. `wasm-bindings-v1` then builds the
real module and reproduces the stale rotation assertion above. After changing
only the CJS consumer, `wasm-bindings-v2` passes the actual generated Node/WASM
consumer and strict TypeScript compilation. All Rust and other compiled source
hashes equal the successful Rust snapshot, so those checks need no repeat.
`qualification-summary-v1.json` records this aggregate acceptance while both
failed binding runs remain unchanged. The checkpoint script verifies counts,
logs, source correspondence and preserved artifacts; this document alone
supplies no passing result.

The 35 existing native Hypercurve/HyperDRC files are included in test snapshots
and remain separately unadmitted. Earlier native runtime gates and Hypercircuit
downstream fixture failures are not resolved by this scalar corpus. Typed
Boolean adapter errors, broader useful-workflow completion, full-query resource
contracts, end-to-end provenance, formal/coverage/fuzz campaigns and the clean
external distribution/release path remain open. No push or full pre-push CI
claim is made.

Historical preservation remains incomplete after the user-confirmed external
`/tmp` cleanup: 26,957 historical paths and 136 pinned diagnostic inputs were
lost. Their expected hashes remain in the inherited audit. The final audit must
retain every missing expectation and verify current survivors; it must not
silently treat missing artifacts as recovered or drop them from the denominator.
Only `current_work` in `docs/properties-status.json` may change; raw bytes outside
that value and all other decoded keys remain those of 9025de62.
