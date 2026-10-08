# numbers follow-through: checked ln(1+x)

The logarithm range repair is RETAINED, uncommitted, on 2026-09-06. It follows
the retained atan and asin/atanh repairs; no donor numerical code was copied.
The broad reference audit is OPEN. The subsequent192,000bit asin/asinh
coefficient failures are confirmed and recorded in series-limit-README.md;
this bounded logarithm qualification is not a whole-library correctness claim.

## Cause and proof

The generic PrescaledLn kernel calculates ln(1+x), using the odd series for
`2*atanh(x/(2+x))`. Its error budget assumed a small residual, but Real::ln_1p
and inverse-hyperbolic identities could supply arbitrary in-domain residuals.
The unconditional `p >= 0 -> 0` shortcut was therefore incorrect. For example,
public ln1p(-255/256) returned zero at precision zero instead of a value within
one of ln(1/256). Earlier frozen asinh/acosh controls had eight such failures
per build. Fine-precision series convergence also needs the stated range.

The kernel now checks the operand sample it already needs. If `a` approximates
x at precision s with at most one integer unit of error, then
`bits(|a|) <= -s-1` proves `|a| <= 2^(-s-1)-1`, and hence `|x| <= 1/2` after
including the sample error. Only then is the existing local series used.
No additional approximation or allocation is required for ordinary fine
requests. Coarse requests use the same bounded check at the -1-bit working
schedule before returning zero: `|ln(1+x)| <= ln(2) < 1` on that interval.

Rejected ranges use `ln(1+x) = 2*ln(sqrt(1+x))`, with the existing sqrt and
normal logarithm reductions. The extra sqrt is important: simply retrying
ln(1+x) could reproduce the same ambiguous half-boundary residual. For a
normal ln residual near +/-1/2, sqrt(1+x)-1 is near -0.2929 or +0.2247,
strictly inside the certified range even after approximation error. Large
positive and near-minus-one arguments still receive normal logarithm scaling.
No wider series convergence radius or stronger equality decision is assumed.

Construction remains lazy. Abort is checked before work and before fallback;
aborted approximation results are not published to the shared cache. Existing
PrescaledLn serialization remains usable. Private rational residual kernels,
which already have exact constructor-side range reduction, are unchanged.
No new public API, node variant, object field or dependency.

## Qualification

- A new public Real::ln_1p regression fails on the immediately preceding
  worktree and passes after the repair. The original `log_boundary.rs` source
  remains unchanged, SHA256
  `64609c2ab917cc4fdabedf26544bbda6cbc59e7e386f5bf8c7d3d2d6436465c7`.
- All 77 original asinh/acosh cases pass in both debug and release (154 total),
  replacing eight failures/build. Sources, frozen binaries and complete process
  records are checked by analyze-log.mjs; no failure or timeout is discarded.
- The earlier80asin/atanh requests per build were also rerun against the final
  log build: all160pass. They are separate from the old frozen inverse-repair
  binaries, not inferred to pass because a historical validator stayed green.
- New native directed-MPFR checks per build: 48 public residual/history checks,
  624 valid working-sample boundary roundings/refinements, 224 extreme-scale
  and warm/cold controls, 3,200 public/opaque residual-grid controls, and 90
  signed estimated-sum/history controls: **4,186 total**. Exact dyadic inputs
  and independently directed radical enclosures are used. Tests cover proximity
  to -1 at 2^-1024, positive 2^1024, tiny signed inputs, cancellation-sensitive
  sums, and requested output precision through 512 bits.
- Additional tests cover no eager approximation, abort/cache recovery, and
  serde roundtrips. Full native gates: **741 debug passes**, **848 release /
  all-feature passes**, including 19/24 doctests.
- Final source downstream --lib passes: Hyperlattice19 + Hyperlimit242 +
  Hypertri3 + Hypersolve432 + Hypercurve897 = **1,593**, Hypercurve one ignored.
  Hypercurve final run finished in604.24s on eight test threads/CPU8-15.
  Its HEAD was5156f67a4afc651a6f655d4ac2ba8a2317a7c3ff; source hashes were
  checked at launch/end. A concurrent user PERFORMANCE.md change was preserved.
- The obsolete, single-CPU Hypercurve candidate run was explicitly terminated
  after checking its exact PID/parent/command. Its retained signal15 log is
  not credited as a passing or final-source run; the replacement is separate.
- Final debug Memcheck: six log-domain tests pass in38.35s, terminal exit0,
  zero errors. Leak checking disabled; this is not a peak-memory/leak-freedom
  result. Format and all-feature Clippy (library and all targets, -Dwarnings)
  pass; logs are retained separately.

## Paired measurements

`log_controls.rs`: 256 preconstructed inputs per process, alternating 32/128-bit
requests. Timed work includes clone, public Computable function construction,
and approximation. Each result is enclosed by independent 4096-bit directed
MPFR outside timing. All baseline timed outputs were numerically valid; coarse
wrong answers are not included in speed ratios.

198 frozen CPU6-pinned fresh-process observations, 11 families, nine alternating
before/after rounds. Round zero excluded: 176 observations, eight pairs/family.
analyze-log-bench.mjs verifies source/binary hashes, record counts, status, and
stable per-variant checksums; it reports20,000 paired bootstrap resamples in
log-bench-analysis.json. In this corpus all cross-variant checksums also match,
although the mathematical contract permits differing valid approximations.

Ordinary log, near-one, small/warm inverse-hyperbolic and large-input controls
are broadly near parity, with unchanged allocations. Two selected families
benefit from checked range reduction at fine precision:

| Family | Median before/after CPU ratio | Paired 95% interval | Allocation calls | Requested bytes |
| --- | ---: | --- | --- | --- |
| asinh-moderate | 1.38860 | 1.25412–1.48961 | 72,196 -> 58,820 | 2,516,896 -> 2,245,536 |
| acosh-moderate | 1.18307 | 1.04795–1.21573 | 54,152 -> 53,384 | 1,940,288 -> 1,930,048 |

These are corpus-specific improvements, not general library or cross-language
speed claims. Concurrent host work was not excluded. Allocator counters are
inside timings; byte counts are cumulative requests, not peak residency.
Other public ln1p-based functions were exercised by full tests, not individually
benchmarked here. A possible asymmetric positive series range is not selected
without separate proof/measurement; current code preserves the existing radius.

Harness size: file2,106,696 -> 2,107,584 bytes (+888); text+712, data+24,
bss-696, loaded total+40 bytes. No object-layout growth; no universal binary-size
claim. Incremental production diff +28/-11 lines and203 test lines over the
retained inverse-series checkpoint.

## Evidence and reproduction

Qualification manifest: numbers-qualification/Cargo.toml, offline Rust target
`.audit-targets/ireal-derivative-18555`. Frozen binaries/sources are under
`.audit-numbers.rjcbha/log-*`. Source/binary hashes are asserted by the analyzers.
`run-log-repair.mjs debug|release|paired|neighbor-debug|neighbor-release` records every invocation and status;
subprocess execution requires explicit sandbox approval on this host.
`analyze-log.mjs` validates the completed scalar checkpoint, including the
separate final Hypercurve run. Previous donor/atan/inverse validators preserve
their historical snapshots and must not be treated as current worktree hashes.
