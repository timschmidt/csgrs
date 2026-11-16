# Matrix scaling reconsideration

The matrix-only candidate previously rejected for size was reproduced at
`c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb` under the owner-authorized bounded
size exception. It is not retained: a new completed-operation comparison found
an independent latency gap. The original production array builder and matrix
surface are restored. Two independent tests of callback order and partial-value
destruction remain, with a bounded account for their test executable.

## Contract and reproduction

Hyperlattice-interval owns the proposed shared-context, row-major matrix scalar
image. The result is a Cartesian enclosure, including explicit empty and
zero-dimensional unit products; it does not retain correlation from a shared
uncertain scalar. Scalar multiplication, errors and resource accounting remain
with Hyperinterval. The current missing-API witness fails with 19 method-not-found
errors before the source change. The experiment restores the original hard cases
and adds complete work-cut comparisons and a downstream zero-dispatch witness.

The candidate passes 2,022 native tests/doctests across the owner, Hyperlattice,
Hyperdynamics and Hypersolve (133 suites, no failures or ignored tests), strict
Clippy, documentation and formatting. Rust 1.88 owner tests/Clippy and Thumb/WASM
checks pass. The Rust 1.95 Thumb check initially fails because the target is not
installed; after installation, all four stable portable configurations pass.
The original failure log remains in the record.

A linked WASM client exercises the new method against an independent integer
floor/ceiling oracle: 243 cases per binary, 1,078 checked status/endpoint values,
including signed limits, overflow, empty operands and invalid input. Its API
version is 121 bytes smaller than the explicit-scalar version. The existing
44,640-byte centered WASM module is unchanged. Normal firmware and Interface
workspace graphs exclude this crate; this candidate changes no firmware image
and establishes no firmware flash-capacity claim.

## Cost changes the disposition

All 42 existing measured native text sections are identical for the initial
matrix candidate. The old reachability executable grows by eight file bytes;
the attributed successive-operations executable grows by 48. Those fit the
declared existing-artifact caps. Final binaries, code sections and named text
are measured separately; no stripped-size substitute is used.

The new five-backend client compares the same completed 2×2 scaling operation
against four explicit scalar multiplications, including output materialization
and destruction. Its file grows 15,504 bytes, text grows 7,232, and named text
grows 6,990. This exceeds its initial allowance. A scoped larger size allowance
was considered and recorded as a proposal, but was never adopted.

Four CPU-0 ABBA blocks with ASLR disabled cover seven existing owner probes and
the new operation: 128 runs, 72,640 samples and 10,731,520 completed operations.
All normalized output/allocation records agree. The new API's median costs are
8.45–29.26% above its completed scalar control across the five backends. The
existing very short probes also show timing variation despite identical code;
those observations are retained and are not advertised as speedups or regressions.

Whole-process Callgrind counts rise from 419,393,181 to 453,847,404 instructions
for the new client. These counts include warmup, printing and teardown. Symbol
attribution identifies fallible array construction, mapping and cleanup code;
inclusive counts are not additive or a measurement of one operation alone.

Two scoped experiments fail to resolve the cost:

- A single-pass construction algorithm preserves callback/drop-order tests and
  passes 99 owner tests/doctests, but remains 6.51–53.39% slower than the explicit
  control. It saves only 192 file bytes relative to the initial candidate.
- A targeted inline hint on the new public wrapper, with the original shared
  builder restored, remains 7.55–32.59% slower and saves only 560 file bytes.

Each focused campaign has 16 runs, 1,600 samples and 1,638,400 completed operations.
Both experiments are removed. No further attribute/layout trials or relaxed
fixtures follow. The size exception does not qualify the measured latency gap.
The full successive-operations timing campaign was not run after this rejection;
its final binaries were built and measured. No full CI, fuzz, coverage or release
qualification is claimed for the rejected API.

## Retained tests and cumulative account

The retained tests exercise the existing helper's first-error callback prefix,
successful value order, zero-length construction, exactly-once cleanup, and
destruction of a partial inner row before previously completed outer rows. They
ran against the original helper before either experiment. No production function,
public API, existing assertion or policy is changed by retaining them.

After restoration, 96 owner tests/doctests, the four existing dispatch-control
records, two Rust 1.88 no-feature unit tests, strict owner Clippy and workspace
formatting pass. All 41 other native artifacts retain identical text, read-only
data, writable data and unwind-section bytes. Their raw-file differences reach
at most +56 bytes. The centered WASM artifact is byte-identical to baseline.

The test harness contains two additional tests, so it is an added workload.
Its bounded allowance is recorded separately from the rejected API's proposal:

| Metric | Before | Retained | Growth | Byte cap and percentage cap |
| --- | ---: | ---: | ---: | --- |
| Test executable file | 1,051,288 | 1,076,136 | 24,848 | 32,768 and 4% |
| Test executable text | 599,684 | 611,892 | 12,208 | 16,384 and 4% |
| Unique named text | 591,406 | 603,321 | 11,915 | 16,384 and 4% |

Both caps hold for each metric. This pays for independent correctness witnesses,
assertions and diagnostics in the test harness; it grants no production arithmetic
regression. The [cumulative account](interval-size-accounts.json) fixes the
goal-wide baseline at the revision above, records each artifact independently,
and retains remaining allowances. Future exceptions must carry these costs
forward. They cannot reset the baseline or offset one target against another.

Raw commands, source patches, initial cap failures, rejected budget proposal,
profiles, outputs and restoration checks are in
`docs/evidence/2026-11-16-interval-matrix-scale-reconsideration`. The full interval
plan remains active. The next size-only refactor eligible for fresh review is
Taylor derivative child scheduling, whose prior witness removed an allocation.
