# Checkpoint 56 — matched native endpoint/history costs

The guarded point-image repair remains unselected. Its completeness gain is
independently verified, but the broader nonrational/Unknown corpus exposes
unchanged-result runtime and allocation costs that the earlier rational corpus
did not. Investigate demand-gated recovery before retaining this implementation.
This checkpoint closes the stated native campaign, not the full ecosystem audit,
WASM cost qualification, or a production transfer.

## Source and qualification

The retained and guarded 956-file snapshots are reused without another source
copy. The sole production-code difference is still the isolated solver repair;
live sources remain unchanged. The checkpoint-55 corpus prefix is mechanically
reused, removing its unnecessary `mut` and unused timing import. The old draft
timing dispatcher remains preserved and unqualified. New CPU and allocation
wrappers share constructors, histories, queries and complete serialization.

Both release builds and all-target Clippy with warnings denied pass; the outer
build ends 2026-09-10T21:41:00.627Z. Four frozen executables occupy 12,772,864 bytes
in `/tmp/calcium-point-history.L38XCJ`. Native CPU size grows 1,128 bytes and the
instrumented binary grows 1,176 bytes; these are harness/link-layout observations,
not algorithm-only or representative application-size deltas.

All 48 cases × two policies × four histories × two lifecycles are included:
768 groups, each checked in both variants and both instrumentation modes.
Qualification passes 3,072 observations and 84,864 independent exact checks,
ending 21:55:25.081Z. CPU qualification uses one measured iteration; allocation
uses eight. The exact interpreter checks the final actual full result, including
its computational graph, selected value, full polynomial up to nonzero rational
scale, bounds, multiplicities and witness. It does not use Hyper equality or
floating-point overlap as its oracle. Intermediate iterations have only a
polynomial-length checksum, not independent full mathematical checks.

There are 256 improved groups and 512 unchanged groups. These are repeated
policy/history/lifecycle instances of the existing lost-witness gap, not 256 new
independent defects or a disjoint addition to prior counts. Nine preconditioning
calls leave every complete input/report identical to the checkpoint-55 record.
CPU and instrumented observations agree in full. Deep-history availability
differences remain separate groups, never averaged into a single state.

## Measurement protocol

- Native x86-64, Ryzen 7 5800X3D, core 6 affinity; its SMT sibling is core 14.
  The host uses the powersave governor. Before/after environment snapshots are
  preserved; neither fixed frequency nor an otherwise idle host is claimed.
  No own compilation, regression or Memcheck workload overlaps the CPU run.
- Each group uses a 16-iteration pilot per variant. A common iteration count
  targets 4 ms at the slower pilot rate, clamped to 4–4,096 iterations. All
  12 alternating ABBA/BAAB blocks are retained without outlier deletion.
- CPU: 36,864 observations plus 1,536 pilots, 21:57:01.521–22:03:03.195Z.
  Allocation: 4,608 observations, three paired blocks of eight iterations,
  22:03:03.289–22:03:33.214Z. Instrumented times are not CPU evidence.
- Every child makes nine preconditioning calls. Fresh rebuilds, applies the
  selected history and drops input graphs inside each iteration; retained reuses
  those inputs. Neither lifecycle means cold process-wide scalar constants.
- Timing includes drops of earlier results but not the final result. The final
  report remains live at the counter snapshot. Serialization, full-result
  validation and held-input serialized-record checks happen outside that window.
  Net live/peak counts include cache activity as well as result storage.
- Every saved final full result is compared with its independently qualified
  record. Offline verification also replays qualification, checks actual pairing
  order, timing ranges and common calibration counts, and recomputes statistics.
  It passes at 22:11:50.280Z with empty stderr and no child failures.

## Results and tradeoffs

For the 512 same-result groups, paired median candidate/baseline ratios range
from **0.902928 to 1.348403**. Fourteen individual bootstrap intervals lie wholly
below one; 192 lie wholly above. These 95% intervals are per-group, not adjusted
for 768 comparisons and not a universal performance claim.

The largest paired slowdown is retained case 37, approximate policy, serialized
round-trip history: median per-query samples are 18.242 versus 24.587 microseconds,
paired ratio 1.348403, interval [1.277726, 1.378328]. Both versions return the same
NonIsolatingImageInterval. Related constructed/coarse/round-trip cases 36/37 show
similar costs; no extra answer is obtained there.

Same-result allocation requests/bytes are unchanged in 324 groups and higher in
188, with no lower or variable-delta groups. The largest eight-query request
increase is 456, and the largest requested-byte increase is 46,848; these maxima
need not belong to the same group. Return-live delta is identical in all 512.
Peak demand is unchanged in 496 and 176 bytes higher in 16 retained-input groups
(cases 20/21, all policies/histories). Request totals can be normalized by eight;
net live and peak deltas must not be treated as per-query allocation totals.

The 256 gained-answer groups have paired ratios 0.914530–3.506376. They perform
additional certified work and are not equal-work speedups or regressions. In
those groups, request/byte totals decrease in 80 and increase in 176. Eight-query
deltas range from −8 to +1,600 requests and −456 to +82,168 requested bytes.
Return-live increases 143–1,663 bytes with the final successful result alive;
peak is unchanged in 165 groups and higher in 91, at most 1,104 bytes. This does
not establish a leak, bounded lifetime memory, native allocator overhead or RSS.
No new Memcheck campaign is claimed; checkpoint 55's whole-collector reachable
byte difference remains distinct from these marginal measurements.

## Decision and next experiment

Do not retain the current eager witness probe at this checkpoint. Exactness and
completeness take priority, but there is a plausible way to preserve the recovery
without charging its strict endpoint comparison to every nonpoint/Unknown image.
Static inspection shows `binary_image_interval` performs that new comparison
before the refiner, while the refiner already compares endpoints and specifically
rejects collapsed intervals lacking a witness. This suggests a demand-gated
recovery path. It is a hypothesis, not a measured optimization or permission to
weaken the refiner's witness, vanishing, uniqueness or whole-image obligations.
The approximate-policy multiplication/division strict replay must remain sound.

Next: implement and independently qualify an isolated demand-gated revision,
compare it against these preserved controls, then execute the relevant extended
WASM checks/costs and decide retention. No new variant exists at this checkpoint.
The separate power-sum experiment and the original full reference inventory
remain in scope.

## Evidence and resource accounting

`point-history-manifest.json` binds the sources, qualification, full raw rows,
summaries, command captures and this note. `verify-point-history.mjs` rechecks
956 live identities and imports 55 through 48, preserving earlier failed gates.
It does not rerun the full historical 47-checkpoint chain; that latest full
capture remains 2026-09-10T04:56:34.285Z.

Raw qualification/CPU/pilot/allocation files total 158,331,284 workspace bytes.
At the captured capacity check, `/tmp` has 16,467,509,248 available bytes. The four
dedicated executable bytes above exclude changes within the reused Cargo cache.
No cleanup, deletion, production/donor edit, commit, push or external report.
All five retained continuation transfers are unchanged. No donor-read coverage
is added: 1,415 complete files, 20 partial files, 183,653 uniquely counted lines
remain Calcium/FLINT continuation totals, not whole-ecosystem completion.
