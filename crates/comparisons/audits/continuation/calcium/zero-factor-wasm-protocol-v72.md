# Zero-factor WASM correctness protocol — checkpoint 72

This is qualification, not a timing campaign or retention decision. Keep all
source trees unchanged. Build rational and history bridges for both the retained
baseline and isolated checkpoint-70 candidate, using shared scalar dependencies
and the existing build cache. Copy only the four WASM modules to an exclusive
/tmp directory. The rational importer must match the native checkpoint-71
importer exactly. Reuse the previously read history bridge/work unchanged.

Compile modules once with V8 flags --expose-gc --no-liftoff --no-wasm-tier-up
--no-wasm-lazy-compilation, record imports/exports and require no host imports.
Pin execution to CPU 2, but make no throughput claims from this qualification.
Record Node/V8, module/source hashes, all terminal captures and linear-memory
samples. Linear-memory size is neither per-query allocation nor peak/RSS.

Rational bridge: allocate an input buffer once (positive length, at most 128 MiB),
copy the hash-bound JSON from the host, then parse once outside the query batch.
Prepare creates input roots, verifies their complete wire identities, computes
the full expected report and executes eight untimed warmups. A batch consumes
and drops every full report through black_box, matching native lifecycle
semantics; its polynomial-length checksum is not a correctness oracle. Finish
repeats a full query outside the batch, checks the full report and source wires,
then serializes output. Work is released before the next prepare; module-global
state can persist. ABI misuse must trap in fresh disposable control instances.

Coverage:

- One persistent rational instance per variant runs all 8,346 authored cases
  (6,440 earlier public/cost, 1,840 wide, 66 degree/sign/shared), both policies,
  retained sources and one batch iteration: 16,692 queries per variant. Compare
  every complete report to its independently qualified native checkpoint-70
  record, including signed coefficients, interval/witness, metadata and guards.
- Rational cost controls use the 114 native cases under both policies, both
  lifecycles and batch sizes one/eight in fresh instances: 1,824 observations
  across variants. Compare against native full records, not checksums alone.
- History bridge controls use 48 cases, both policies, four histories, both
  lifecycles and batch sizes one/eight, each observation in a fresh instance:
  3,072 observations across variants. Compare complete source/report records to
  native checkpoint 70 and replay the independent extended-field oracle. This
  reused bridge retains the final batch report until finish; no timing
  equivalence with the rational drop-all bridge is claimed.

Decode output only within checked pointer/length bounds and before another
export can resize it. Full state/order violations and malformed/truncated
payloads are diagnostic failures, not permission to weaken report equality.
Preserve failed captures and source versions if correction is required.

After qualification, separately predeclare any WASM cost campaign, its instance/
history model, common counts, balanced orders, host boundary and corrected
statistics. Native timing ratios do not transfer to WASM by assumption. Consumer
regressions and representative stripped sizes remain before retention. No new
donor-line credit or full ecosystem completion follows from this checkpoint.
