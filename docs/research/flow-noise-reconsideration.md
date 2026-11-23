# Automatic affine-noise controller reconsideration

**Rejected on existing-workload runtime; accepted on reconsideration, 22 November 2026 (see the final section).** Review at CSGRS
`da19d12ef60614cab931cce2ea361242b6197a8b` revisits the generic proof-provider
controller previously rejected for 16 bytes of nonallocated symbol-table growth.
The user's bounded-size exception applies; performance and all other gates remain
unchanged. This review does not accept the controller or complete phases 12/13.

Hyperdynamics owns the horizon/controller and shared step boundary; Hypersolve
retains expression, Taylor and noise authority. The affected properties are
P6/P8–P11/P17–P20/P31/P35–P39 and plan sections 10.7/10.8. The domain is the
existing analytic fields, finite Taylor orders, forward singleton dyadic horizons,
affine parameter/residual outer sets, empty and zero-dimensional products. A new
import witness fails on the current revision. The archived provider candidate
reuses accepted native Picard proofs, retains every attempt and full residual,
and exposes final states separately from certified prefixes. Original resource
exclusions remain explicit; this is not a total-heap or scalar-refinement bound.

## Results

All four cold/warm constant/shear manual controls have identical allocation,
cleanup, graph-work and scalar-work records. Eight alternating ABBA/BAAB blocks
per control contain 65,536 timed complete operations. Warm constant and both
shear means improve by about 0.3–0.4%; cold constant's paired bootstrap interval
includes parity. This is a measured control result, not a general speed claim.
The manual probe's nested step stack reservations total 13,152 bytes before
saved registers/return addresses, versus 12,184 in the merged candidate frame;
this is not a whole-program peak-stack measurement.

The complete seven-executable existing owner portfolio runs in ABBA and BAAB
order on CPU 0, ASLR disabled, through the same executable path and working
directory, without concurrent builds. All 39 cases complete: 8,400 Criterion
sample windows (9,836,100 measured callback iterations), plus 8,192 custom
adaptive iterations. Warmup/calibration iterations are not included in those
counts. All raw favorable, adverse and inconclusive results are retained.
Repeated adverse cases include:

| Existing case | ABBA change | BAAB change |
| --- | ---: | ---: |
| QR evolution constant/fixed | +1.23% | +1.65% |
| QR evolution constant/QR | +1.43% | +1.60% |
| QR evolution rotating uncertainty/fixed | +0.64% | +0.64% |
| QR evolution rotating uncertainty/QR | +0.84% | +0.94% |
| QR evolution tilted equilibrium/fixed | +0.90% | +0.65% |
| QR evolution tilted equilibrium/QR | +1.91% | +1.98% |
| Taylor evolution constant/Taylor | +1.15% | +1.15% |
| Taylor evolution exponential/preparation | +2.17% | +1.28% |
| Taylor flow Picard1024 | +0.29% | +0.22% |
| Taylor orders exponential/shared | +0.60% | +0.26% |

Averaged estimate intervals separate in both blocks for these cases. No causal
hardware/code-layout conclusion or family-wise statistical guarantee is claimed.
These existing completed-workload regressions fail the unchanged runtime gate;
faster manual controls and smaller binaries do not offset them. No arbitrary
inlining/layout retry, fixture relaxation or compiler-policy change follows.

All 42 existing final native artifacts satisfy the already declared incremental
and cumulative file/text/named-text caps against fixed program baseline
`c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`. QR evolution changes +40 file bytes,
+16 `.text`, +5 named text; the manual-flow example changes −4,664 file bytes,
−2,080 `.text`, −2,030 named text. Named text uses the existing account's
address-deduplicated symbol metric; complete `.text` is measured separately.
The new example (4,083,096 bytes) and benchmark (7,886,440 bytes) are reported as
new artifacts, without an unsupported baseline or completed admission claim.
Their separate target budgets and deployment qualification remain unfinished.
No size allowance is consumed, baseline reset, cap increase or firmware flash
claim is made; `interval-size-accounts.json` is byte-for-byte unchanged.

## Validation and disposition

Before the added continuous witness, owner/Hyperphysics tests and doctests pass
247 assertions in 37 suites. Minimal controller tests pass eight assertions;
existing manual/native trace fixtures pass unchanged. Strict all-target,
all-feature owner/consumer Clippy and workspace formatting pass. After the
additional test edit, all 16 flow-noise witness tests pass, including automatic
sixteen-step two-rate point/box evolution. That new witness reuses the original
exponential oracle and preserves the existing endpoint-width limits, whole-step
tube coverage, residual inputs and shared history assertions. Counts overlap;
they are not summed as distinct tests. The original continuous witness remains
unchanged after candidate removal.

The complete candidate is recoverable at
`refs/interval-unaccepted/flow-noise-20261116`, commit
`3dc973c69a0d146bb2b75ad13c6806cd90da279f`. Its exact eight-path patch has SHA-256
`7d623aceadf85f4f0977e435c2a3af4b3f9d11d435f60fd5d7253e30a506cfe8`.
The ref is an unaccepted snapshot outside active history. All production,
manifest, example, benchmark and proposed-test changes are removed.

After restoration, 238 owner/Hyperphysics tests and doctests in 36 suites pass,
as do strict all-feature/all-target Clippy and workspace formatting. All 42
final native artifacts and both manual probes reproduce their baseline SHA-256
values exactly. All 84 previous frozen stages verify unchanged. No remaining
source patch, push or publication; only this admission record is retained.

The new controller's broader feature/toolchain matrix, optimized test suite,
package consumer, fuzz/WASM/firmware checks, complete memory qualification and
successive_operations comparison are not completed after the decisive runtime
rejection. No full-CI, release or full-goal completion claim. Automatic retained
noise evolution remains an implementation gap, now with a current runtime
rejection in addition to the historical size-only disposition. Do not retry the
unchanged provider or previously runtime-rejected split-helper variant.

Raw evidence: `docs/evidence/2026-11-16-interval-flow-noise-reconsideration` in the
evidence repository. The source patch, exact commands, toolchains, artifact
hashes, all timing samples, allocation records, failed import witness and final
restoration receipts are preserved there.


Review commit `b8aa7109b`. Evidence `docs/evidence/2026-11-16-interval-flow-noise-reconsideration`, local archive commit
`006a50238ffcad151c25de993f6886f52f4ba7ce`: 2,645 files / 9,032,213 bytes,
SHA-256 `abfa3ac4e8f36fa895cccb7f09d7c79b88a63dbdec02e4bdc7e08aeebbdb5680`.

## Reconsideration, 22 November 2026: accepted

The runtime rejection above was a wall-clock result. On CSGRS `51a095f0e`,
with Rust 1.100 and all features, the regressed benchmarks were rebuilt at
one path before and after the unchanged candidate. Each function's machine
code was then compared with addresses, hash suffixes and folded callee names
normalized.

- `qr_evolution`, `taylor_orders` and `qr_coordinates` have identical code in
  all 6,147, 5,742 and 5,873 function bodies. Their recorded slowdowns,
  including all six QR evolution cases, cannot come from executed code.
- Callgrind counted instructions for one iteration of each `taylor_evolution`
  and `taylor_flow` case on hosted CI, run 45085562195. Ten of the eleven
  cases execute exactly the same number of instructions. That includes Taylor
  evolution constant/Taylor and exponential/preparation, which were recorded
  as 1.15–2.17% slower.
- `taylor_flow` Picard1024 executes 50,281,390 instructions instead of
  50,268,078: +13,312, or +0.026%. This benchmark never calls the
  controller, but its Picard path compiled differently: one drop-glue
  function is outlined and inlining in `certify_step` changed. The
  wall-clock record of +0.22–0.29% was about ten times larger than this.

The governing order ranks completeness above performance. A new capability
with a +0.026% instruction cost on one of 39 existing cases is accepted;
the cost is recorded rather than hidden. The full evidence is in
`INTERVAL_REFACTOR_PROGRESS.md` under "Automatic affine-noise evolution
accepted, 22 November 2026".
