# Matched zero-factor WASM costs — checkpoint 73

Freeze before timing; reuse checkpoint-72 rational modules without rebuilding.
Use the same 114 native cost cases: both policies and retained/fresh source
lifecycles, yielding 456 groups. Measure separately with one persistent module
instance per worker process and with a fresh module instance per group. Fresh
instances are preconditioned; neither label means measured cold startup.

All workers are separate processes on CPU 2. Compile the selected module once
per worker before observations, using the exact Node/V8 flags from checkpoint
72. The orchestrator is on CPU 0. Record OS/CPU/frequency/load/process snapshots
before pilots, after pilots and after the campaign. No reserved core, fixed
frequency, continuously observed contention or universal independence is claimed.
Preserve every sample, including sub-millisecond batches and slow observations.

The frozen rational bridge allocates/parses JSON outside measurement, prepares
full input roots and expected output, then makes eight untimed query warmups.
The host clocks one zf_batch call, including JS-to-WASM boundary and every query's
full-report destruction. Finish performs another untimed query and checks full
report equality, checksum and source wires. Output decoding/serialization, full
native-report comparison, instance setup/GC and preparation are outside the clock.
This is warmed-query batch cost, not whole-expression or module-startup latency.
Source-fresh groups reconstruct roots from preparsed input inside each query.

Pilot batches 1 and 32 run for every group, both variants and instance modes;
retain all 3,648 pilot observations. For each group and instance mode, derive one
common iteration count from the slower 32-iteration pilot, targeting 3 ms, capped
to [1,20000]. Never retune counts after looking at measured results.

For each instance mode collect 24 paired blocks, with 12 AB and 12 BA orders.
Each pair uses the same shuffled group order and iteration counts. Alternate
which instance-mode pair runs first with a balanced randomized schedule. Freeze
all schedules with the corrected deterministic rejection-sampled generator.
This yields 43,776 measured rows over 912 group/mode combinations. Each worker
uses explicit GC after every eighth observation, outside the batch; persistent
WASM globals survive, fresh instances do not. All worker terminal captures and
the input/module/source/config/plan hashes must validate before inference.

Use paired candidate/baseline ns-per-query ratios, their median, the corrected
20,000-resample bootstrap and independent binomial order-statistic median interval.
Report all 24 raw paired ratios/ranges and both intervals per group. Intervals
are conditional on sampling assumptions and are not multiplicity-adjusted.
Separate unchanged-result groups from newly Transformed outputs: computing a
previously rejected answer is additional functionality, not equal-work slowdown.
Keep zero guards, nondivision/nonzero-constant bypasses, bounded deflation and
oversized deflation visible; no universal speedup is inferred from one category.

Record linear-memory capacity only as a diagnostic. This is not an instrumented
allocation campaign, nor a peak/live/RSS measurement. No new WASM allocator
counters or native timing results substitute for WASM evidence. Checkpoint 72's
nonrational/history qualification remains valid, but its differently scoped
last-report-retaining bridge is not included in these timing claims.

Retain no production change solely on this checkpoint. Representative consumer
regressions and stripped application-size gates still precede adoption. Full
donor/reference reading and inventory reconciliation remain separate open work.
