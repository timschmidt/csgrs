# Exact radius composition for named machine budgets

**Not retained.** Phase 8 section 8.3 was exercised on CSGRS
`0a26f2206ef13211b928429785792cd3c37668f7`, Interface
`32a59de0d69436427e5b97297c4929fd908bf326`, and firmware
`2d68e82b7341db7b747b20e4ad1455a4c1113a00`. The candidate preserves measured
exact behavior and improves successful budget timing, but exceeds the declared
owner file allowance and increases immediate-rejection cost. No budget revision
or performance exception is adopted. The full plan remains active.

## Contract and implementation

Hyperinterval owns nonnegative exact radii; Hyperreal retains scalar arithmetic,
cache and square-root authority; Interface retains named physical sources,
units, configuration/capability identity and its final strict budget decision.
Properties P1/P4/P6/P8–P11/P19/P25–P27/P31–P33/P36–P39. Existing `RealBall`
accepts rational radii and cannot directly represent the machine budget's
irrational Euclidean norms. Replacing those norms by rational enclosures would
change the existing exact definitions and is not attempted.

The absent `RationalRadius`/`RealRadius` import is reproduced before editing
production source. The proposed host-only `real-radius` feature uses checked
nonnegative rational ingress, native exact Euclidean norms, a u32/nonzero-u64
time ratio, and owned sums/products. Private payloads retain construction
proofs; no arbitrary Real or serialized positivity flag is trusted. Native
sqrt/division failures remain typed. Native arithmetic/resource exclusions
remain explicit, without a new whole-work bound.

The consumer replaces exactly the original source/controller comparisons in
short-circuit order, after the unchanged requested-total check. It retains the
zero-seeded square/add-assign norm sequence, the positions of Real conversions,
left-associated sums, all named Real/Rational fields, and the final strict
comparison. No geometric approximation, new anonymous report total or second
scalar authority is introduced. This is a distinct experiment from the rejected
machine-profile endpoint-storage candidate.

## Exact behavior and qualification

All seven native budget transcripts, configuration/capability identities,
serialized exact components, allocations, peak/live memory and cleanup records
match baseline. The instrumented complete consumer transcript also matches,
including every scalar dispatch count. WASM replay agrees with native output
and invokes zero host calls. The corpus has four successful budgets, two
immediate invalid-input rejections and one fully computed exceeded-budget result.

The owner witness covers negative/zero/large rational ingress, irrational
norms, full-width unsigned ratios, subsequent sums/products/norms and exact
serialization. Complete trace comparisons pass for 24 matched cases in fresh
processes with cold or one-warmup histories. An initial shared-input comparison
fails even for legacy-versus-legacy: native retained work warms the second
run. Separately constructing small-integer operands in one process is also
insufficient because interning can share that work. The complete assertion is
moved to matched fresh processes; no counts or cases are dropped. Original
value/serialization assertions remain, and both failures plus the legacy-only
control are preserved.

Candidate Hyperinterval checks report 253 passing completions in 45 result
lines, including nested subprocess fixtures. The isolated feature passes its
three applicable tests. All 199 Interface core tests pass, including existing
schedule and canonical planner assertions. All-feature/all-target owner Clippy
with `-D warnings` and workspace formatting pass. These counts overlap and are
not summed as distinct tests. Broader qualification stops after the cost gates;
there is no full-stack, package, MCU, HIL or release completion claim.

## Costs and disposition

The configured final native budget probe changes 1,356,048 → 1,357,048 bytes
(+1,000), with complete `.text` +528 bytes. WASM changes 1,686,302 → 1,687,282
(+980), with code payload +977. Those probe metrics fit the declared local
caps; they do not establish a complete cumulative or deployment admission.

Of 41 existing owner executables, 25 are byte-identical. Nine exceed the
predeclared +4,096-byte incremental file allowance: `float_cost`, `host_cost`,
`atan_cost`, `proposal_cost` and `order_cost` add 4,152 bytes; `primitive_root_cost`
adds 4,144; `tangent_cost` adds 4,392; `division_cost` adds 4,424; and
`fractional_power_cost` adds 4,488. Their exception tables each grow 3,192 bytes.
This is measured section attribution, not a proven compiler-causality claim.
No strip/unwind change, cap increase or cross-target offset follows.

After initial warmup drift on very short rejection paths, a confirmation runs
12 explicit warmup processes and eight alternating ABBA/BAAB blocks. Every
measured process uses 31 samples of 64 complete calls per case, on CPU 0 with
ASLR disabled, through the same executable path and working directory, without
concurrent agent builds/tests. The confirmation records 444,416 timed calls;
253,952 succeed and 190,464 return the expected typed rejection. Warmup and
allocation-control calls are excluded from those totals.

| Existing budget case | Change range across eight blocks |
| --- | ---: |
| Ordinary allocations, success | −0.85% to −0.52% |
| Zero allocations, success | −0.55% to −0.01% |
| Invalid requested total, immediate rejection | +4.80% to +4.98% |
| Negative source allocation, immediate rejection | +3.84% to +3.86% |
| Insufficient total, computed rejection | −0.53% to −0.18% |
| Negative travel minimum, success | −1.04% to −0.69% |
| Zero configuration uncertainty, success | −1.78% to −1.38% |

Each entry compares the mean of two process medians per variant/block. The
immediate-path increases are only 4.75–4.91 ns and 4.64–4.66 ns per call;
they are reported at their actual scale, not as a general workload slowdown.
Successful-budget gains and those repeated adverse cases are both retained.
No universal significance or causal layout claim is made. The current allowance
and unchanged cost requirements do not admit this candidate. Full owner timing,
successive_operations, peak-stack and final deployment qualification are not
completed after that disposition.

The complete candidate is preserved outside active history under
`refs/interval-unaccepted/budget-radius-20261116`: CSGRS
`36316830cad23d4e56c52c9109fbb51a6950b755`, Interface
`2ce31a651e8ba18ff75302ed1714c9b51ba316e9`. All eight candidate paths, including
the new tests, feature declarations and documentation, are restored. The new
API is not retained without its intended consumer. All 45 cumulative accounts,
existing caps and fixed goal baseline remain unchanged.

After restoration, Hyperinterval reports 247 passing completions in 44 result
lines (including nested subprocess fixtures), and all 199 Interface core tests
pass. Strict owner/Interface Clippy and workspace formatting pass. All 41 owner
executables, both native/WASM budget artifacts and the instrumented trace
executable reproduce baseline SHA-256 hashes. Native transcripts, serialization,
allocation records and WASM replay match. All 86 prior frozen stages verify
unchanged. Named-budget shared arithmetic remains open; no inlining/layout
retry of this candidate is performed. No push or publication.

Raw evidence: `docs/evidence/2026-11-16-interval-budget-radius-composition` in the
evidence repository, including exact patches, commands, full dispatch records,
all timing samples and warmups, artifact hashes, failed witnesses and restoration
receipts.
