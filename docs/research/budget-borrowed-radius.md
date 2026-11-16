# Borrowed radius proofs for named machine budgets

**Not retained.** Phase 8 section 8.3 was exercised on CSGRS
`1b454718b08b5562b11f72291c755b30b7b15d3f` and Interface
`32a59de0d69436427e5b97297c4929fd908bf326`. Borrowed validation preserves the
measured behavior but still exceeds owner size caps and slows immediate
negative-source rejection. No cap revision or performance exception is adopted.

Hyperinterval owns nonnegative radius invariants; Hyperreal retains arithmetic,
cache and decision primitives; Interface retains named physical sources, units,
input storage, identities and its final strict budget decision. Properties
P1/P4/P6/P8–P11/P19/P25–P27/P31–P33/P36–P39. The absent borrowed-radius API was
reproduced before production edits. Existing radius/interval helpers do not
supply this borrowed proof and exact irrational norm composition together.

Unlike the [owned-radius experiment](budget-radius-composition.md), this
candidate keeps rational allocations in the caller and transports a private,
lifetime-bound `RationalRadiusRef`. It validates the original ordered comparison
without moving the rational payload. Conversion to `RealRadius` clones once at
the original consumer conversion site. Generated exact norms, unsigned time
ratios and owned sums/products retain their original scalar operations and
order. No rational enclosure replaces an irrational radius, and no general
Real sign decision or new total-resource bound is introduced.

The new tests verify original-storage identity, pointer-sized witness storage,
negative/zero/large rational inputs, irrational composition, serialization and
complete dispatch under matched fresh-process histories. Compile-fail tests
check that a witness cannot outlive or permit mutation of its source. The
Hyperinterval suite reports 255 passing completions in 45 result lines,
including nested subprocess fixtures; all 199 Interface core tests pass.
Owner all-feature/all-target Clippy and workspace formatting pass.

All seven budget transcripts preserve exact components, configuration/capability
identities, serialization and allocation/live/peak records. Complete instrumented
consumer traces match. Native/WASM replay agrees with zero WASM host calls.
These are corpus results, not a universal proof or full release qualification.

The final native budget executable adds 904 file bytes and 448 `.text` bytes;
WASM adds 927 file bytes and 924 code-payload bytes. Twenty-five of 41 owner
executables remain byte-identical. Nine exceed the declared +4,096-byte file
cap, adding 4,128–4,488 bytes; their exception tables grow by 3,184 bytes.
This is section attribution, not proven compiler causality.

Twelve explicit warmup processes precede eight alternating ABBA/BAAB timing
blocks. Each block compares means of two process medians per variant, using the
same executable path, CPU 0, disabled ASLR and no concurrent agent builds/tests.
The 444,416 timed calls comprise 253,952 successes and 190,464 typed rejections;
warmup and allocation-control calls are excluded. Successful cases improve
0.38–1.57%, and computed budget-exceeded cases improve 1.11–1.58%. Invalid-total
rejection adds 0.13–0.34 ns (0.15–0.39%); negative-source rejection consistently
adds 2.87–2.91 ns (2.63–2.66%). All favorable and adverse samples are retained.
No universal significance or direct matched comparison against the preceding
candidate is claimed. The negative-source result independently blocks admission.

The candidate is recoverable at
`refs/interval-unaccepted/budget-borrowed-radius-20261116`: CSGRS
`73e6669222e2d176045fd3126bd60f46bb3dd9d6`, Interface
`40b731946a8f2743274dbe71c55e9a83e9659e9d`. All eight candidate paths are
restored. No API is retained without its intended named-budget consumer.
All 45 cumulative size accounts and the fixed program baseline remain unchanged.
Full owner timing, successive_operations, complete stack, firmware/HIL and
coordinated release checks were not executed after the rejection. The full
fifteen-phase goal and named-budget migration remain open. No push or publication.

Raw evidence: `docs/evidence/2026-11-16-interval-budget-borrowed-radius` in the
evidence repository, including patches, commands, full traces, raw timing
samples, artifact hashes and restoration receipts.

After restoration, Hyperinterval reports 247 passing completions in 44 result
lines (including nested subprocess fixtures), and Interface passes all 199
tests. Strict owner/Interface Clippy and workspace formatting pass. All 44
measured artifacts reproduce their baseline hashes; full transcripts and replay
match. All 87 earlier frozen evidence stages verify unchanged.


Review commit `2f463ef2830ab36a15585e66066105db6e759207`. Evidence archive commit
`c25f14ce87dfe51b40203f122e49e94e0770dbd8`: 269 files / 3,082,821 bytes,
SHA-256 `c26a51ee3419e79d1b7922a99b84c7de8c98e07b02d36e03c991f3cac7198a2d`.
