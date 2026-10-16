# Hilbert clipping admission progress

The preceding goal turn made progress: commit `362e157c1` retained exact-path
construction failures. This change fixes the reproduced Hilbert composition
defect at the modeling layer by invoking the existing native trim kernel.

## Implemented contract

`hilbert_paths` / `hilbert_paths_with_context` construct the same finite
cell-center polyline and return native `CurvePathRegionTrim2` records. The
records retain complete positive-length clipping, connected runs, source
segment indices and parameters, selected endpoints and boundary contacts.
No new intersection or topology kernel is introduced in CSGRS.

`hilbert_strings` now returns `Result`. Its explicitly narrower geometry-only
view uses exact represented endpoints through native line/string construction.
Nonrepresented selected endpoints return Unsupported from this convenience
API and remain available through the authoritative path API. Invalid inputs,
undecided predicates and native errors do not become empty geometry. Padding
is a grid-bounds inset, not boundary clearance. Public docs and existing
benchmark/render callers describe and handle the migration.

## Executed evidence

- `reproduction-v1`: all 3 diagnostic commands pass, reproducing hole crossings,
  missing valid intervals, rejected boundary overlap and empty-on-error on the
  exact target source graph and pinned E14 library.
- `candidate-v1`: setup and fmt pass; 3 semantic tests pass and 5 fail. The
  rejected string conversion and incorrect structural equality assertions are
  diagnosed in `candidate-v1-findings.md`. Source, executable and logs remain.
- `candidate-v2`: all 10 commands pass on the final source. Focused semantics:
  8 tests; curve suite: 253; all-feature suite: 360 with 1 existing ignore;
  doctests: 1. Workspace fmt, root all-feature/all-target Clippy with warnings
  denied, warning-denied public documentation, the affected benchmark build
  and the README render example build all pass.
- `workloads-v1`: all 21 commands pass (directory preparation plus 20 original
  caller executions). The fixed five alternating baseline/candidate pairs in
  each cold/warm condition preserve work-unit/output-size/checksum fields.
  Independent directed interval and full original grid-coverage tests provide
  the semantic evidence; run counts alone do not establish correctness.

Exact tests cover the hole and disconnected-strip expectations, positive-length
overlap, isolated tangency, empty subtraction and recovery, invalid parameters,
strict→approximate→strict receipts, symbolic translation/nonuniform scaling,
source parameter fractions and boundary contact replay, algebraic endpoints
and downstream native path/trim operations. Original order-3 benchmark and
order-4 render inputs and resolutions are preserved. The example executable
was built; its original Hilbert geometry ran in tests. No full image-rendering
campaign or real binding execution is claimed for this change.

## Cost and limits

All cost observations, including regressions, remain in the raw logs and
`qualification-summary-v1.json`. On this host with both all-feature binaries
and dispatch tracing, median paired candidate/baseline latency is **2.872×
cold** and **5.364× warm**. Baseline per-operation ranges are 0.771–0.919 ms
cold and 0.262–0.336 ms warm; candidate ranges are 2.354–2.612 ms cold and
1.410–1.524 ms warm. Whole-process peak RSS ranges are 14,748–15,156 KiB for
baseline and 14,836–15,520 KiB for candidate. These are process measurements,
not kernel-only memory or allocation counts. See `cost-contract-v1.md` for
timing boundaries and cache conditions. No speedup is claimed: native evidence
costs more than the former incomplete midpoint filter.

The order cap limits source point count. Checked reservations do not bound all
native allocations, coefficient growth, total work or cancellation. Test
containment is external: 1800 seconds and 8 GiB address space per process.
Private `/dev/shm` overlay uppers retain every old lower artifact and failed
trial without adding another full build copy to the full `/tmp` quota.

The closure script checks every earlier expected artifact and rejects any new
loss or mismatch. The inherited 27,093 missing paths remain expected and are
not claimed recovered. Only `current_work` in `docs/properties-status.json` may
change; both raw bytes outside its value and decoded other keys are guarded
against `9025de62`.

All 35 pending native files remain byte-identical and separately unadmitted.
This is workspace qualification, not clean-package/release or full CI
admission. Legacy sampled shape/text failures, remaining adapters, complete
provenance/resource contracts, formal/fuzz/coverage obligations and the wider
properties plan remain open. The full goal is not complete.
