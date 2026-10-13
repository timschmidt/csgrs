This tests a radical-only inline slot with SmallVec's `union` feature after the
previous storage variants were rejected. The previous fixed sample set stays
closed. Three fresh sources share the same eight storage tests: Vec with the
current features, Vec with `union`, and one inline radical slot with `union`.
The feature-only control is necessary because dependency features also affect
other native SmallVec consumers.

Every source starts from the uninstrumented endpoint/P4 combination. Polynomial
terms, persistent closed certificates and opaque atom vectors remain Vec. The
radical candidate's algebra code matches the earlier radical-only prototype;
the separate feature change is the new hypothesis. No package version or
checksum changes. Exact operations, ordered insertion/removal, original retry
and memo release, proof retention, owner release and all recognition limits
remain unchanged.

Preparation froze three 10,132-file sources with four changed files each.
The two Vec controls differ only in the feature flag; the radical candidate
differs from the feature control only in algebra.rs. Package versions/checksums
are unchanged. Qualification completed in all three fresh targets. Each source recorded
its actual layouts and passed the eight shared storage controls,
six cache-frontier controls, eleven P4 regressions, all-feature/default scalar
libraries, doctests, formatting, strict Clippy and warning-denied docs. Actual
dependency feature sets are bound to the builds: baseline uses no SmallVec
features, and both union variants use exactly `union` for these scalar checks.
Each source passes 882 all-feature and 795 default scalar library tests, 28
doctests, the shared and inherited controls, formatting, strict Clippy and docs.
Configuration and focused counts overlap. See [correctness-v1.json](correctness-v1.json).

All seven recorded layouts match across the three sources: Rational 8 bytes,
coefficient 56, polynomial 24, persistent coefficient 32, persistent polynomial
24, Computable 16 and Node 56. The radical candidate avoids the previous extra
eight coefficient bytes. These equal type sizes establish neither equal total
memory nor faster execution.

The fixed comparison completed 18 preflights and 90 measurement processes
in five predeclared three-source blocks, with separate timing/allocation
executables and fresh targets. All builds finished before measurements. Every
mathematical witness and representation inventory matches. The three pairwise
contrasts reuse this one closed sample set; no earlier samples were extended.
See [cost-assessment-v1.json](cost-assessment-v1.json) for all samples, paired
ratios, ratios of medians, ranges, memory fields and probe sizes.

| Comparison | Phases slower in all five blocks | Phases slower in at least four | Phases faster in all five | Probe size increase |
| --- | ---: | ---: | ---: | ---: |
| Feature control / baseline | 2 / 68 | 10 / 68 | 6 / 68 | 872–992 bytes |
| Radical candidate / baseline | 13 / 68 | 30 / 68 | 7 / 68 | 9,424–10,568 bytes |
| Radical candidate / feature control | 9 / 68 | 26 / 68 | 5 / 68 | 8,552–9,584 bytes |

Against both controls, the candidate reduces allocation calls and allocated
bytes in 36 of 68 phases, and peak requested bytes in 33. No measured memory
field increases; retained requested-byte deltas match throughout. All memory
deltas repeat identically in the five blocks. Per 1,000 inputs, the largest
reductions are 8,142 calls, 309,280 allocated bytes and 144 peak bytes. The
feature-only control changes none of these memory counters.

Repeated scalar slowdowns remain. Warm conjugate products take median
688.549 / 692.653 / 777.518 ns per input for baseline / feature / radicals;
the candidate is slower in five and four paired blocks, respectively. Cloned
opaque linear atoms are slower in all five blocks against both controls.
The feature control's reciprocal timings vary substantially, so large ratios
of medians for those rows cannot be treated as robust candidate gains. Every
sample remains in the evidence. These descriptive results establish neither
statistical significance nor an application speedup.

[decision-v1.json](decision-v1.json) rejects the radical candidate: the
performance-before-memory ladder does not support retaining these slowdowns
for the heap savings. The feature-only change is also unadmitted. No native
qualification, native cost controls, full-report replay or production CLI
build was run for these variants. Those obligations remain open for any future
supported candidate. The copy-boundary review led to a narrower
[closed-certificate memo experiment](../2026-10-13-closed-certificate-memo/README.md).
Its fresh sources pass scalar qualification and have completed fixed costs.
It advances provisionally to native investigation with scalar regressions open;
it changes no radical storage or dependency features.

The native parent remains unadmitted: its original board assertion fails at
133.31248412 seconds of worker CPU against 120; its separate 60-second neck
check-body assertion passes. Those are parent results. All 35 pending native
entries, all 39 properties, 14 targets and crate-specific future work remain in
scope.
