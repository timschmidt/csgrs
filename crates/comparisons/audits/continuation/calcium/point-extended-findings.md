# Nonrational / Unknown endpoint qualification — checkpoint 55

The guarded point-image repair passes the extended mathematical and focused
memory checks below. It remains **isolated, not retained**: matched endpoint/
history costs and relevant WASM performance are still pending. No production,
donor or previously bound source was changed, and no new donor-read coverage
is claimed. The entire original ecosystem inventory remains open.

## Corpus and exact independent checks

The two existing full snapshots are reused without another source copy. A new
serde-enabled release collector emits complete operands, reports, polynomial
coefficients, interval endpoints and witnesses. Serialization omits accelerator
caches; no status/checksum alone stands in for a value check. Inputs are checked
unchanged around every query in the Rust collector.

There are 48 operation cases (12 operand families × add/subtract/multiply/divide),
two policies, and four explicitly ordered histories: freshly constructed inputs,
endpoints refined to −64, endpoints refined to −4096, and a serialization round
trip. Each variant emits 384 queries plus one terminal record. Process-wide
constants can retain caches across the corpus; these are not 384 cold processes.

Families cover singleton sqrt(2)/sqrt(3), scaled radicals, radical interval
endpoints, wider witnessed roots, zero times a wide interval, algebraically
obscured radius 2^-3000, and intervals containing sin(e)^2+cos(e)^2−1. A final
family has mathematically ordered width 2^-2999 that the bounded Hyper input
validator cannot establish. Failure to establish that fact is not a wrong value.

The independent JavaScript interpreter evaluates the **serialized computation**,
not merely the class certificate or a Hyper comparison. It uses exact BigInt
rationals in Q(sqrt(2),sqrt(3)), symbolic nonzero pi/e monomials, and the explicit
identity sin(t)^2+cos(t)^2=1 for the shared finite argument t=e−pi. Exact algebraic
signs use field conjugation/squaring, not floating-point sampling or finite
interval overlap. Arbitrary transcendental nodes/divisors are rejected.

Checks cover the authored exact source bounds and witnesses, rational carrier
representation, every polynomial coefficient after monic normalization (up to
a nonzero rational factor, preserving multiplicities), selected values,
polynomial vanishing, interval image containment, exact witnesses and distinct
root counts. Half-open ownership is respected for witness-free intervals;
certified point witnesses are handled separately. This is not a new numerical
accuracy qualification of the unchanged scalar approximation kernels.

The first decoder omitted shared `InvPi` and `Sqrt2` tags. Its preserved code-1
capture stops at baseline case 40 / STRICT / constructed history, after 4,236
decoded values. The source-defined constant cases were then added; no observation
or numerical result was changed. The completed decoder checks 10,664 serialized
values. This was a checker-coverage failure, not a Hyper mathematical defect.

An initial oracle passes 17,640 checks. Its exact source and successful capture
remain preserved. Additional authored-interval and rational-carrier checks
strengthen the gate to **21,216 checks, all passing**: 9,968 baseline and 11,248
candidate. The strengthened gate completes at 2026-09-10T21:23:32.789Z. In-memory
negative controls reject a changed witness, a changed polynomial and an invalid
endpoint without altering any captured observation.

## Results and history effects

| Outcome | Baseline queries | Guarded queries |
| --- | ---: | ---: |
| Transformed | 188 | 316 |
| Lost point-witness rejection | 128 | 0 |
| Denominator guard | 24 | 24 |
| Undecided | 6 | 6 |
| Nonisolating report | 6 | 6 |
| Input evidence not established | 32 | 32 |

Exactly 128 query records gain certified answers; the other 256 query records
and the terminal record remain byte-identical. The candidate has 252 checked
exact witnesses versus 124 baseline witnesses. These repetitions span policies
and histories and include controls related to the earlier corpus: do not add
128 to 825 as a count of independent defects or disjoint mathematical cases.
The repair still addresses the same lost-witness completeness gap.

Deep endpoint refinement enables addition/subtraction in the tiny Unknown-offset
family under both policies, in **both** variants. Constructed, coarse-refined and
deserialized histories return Undecided under STRICT or NonIsolatingImageInterval
under approximate policy; the −4096 history returns Transformed. There are four
case/policy combinations, recorded separately for both variants. Values do not
change; the available evidence changes the bounded decision. This is why matched
costs must not combine histories or treat gained answers as equal-work speedups.

## Focused memory and storage

Native and Memcheck outputs match exactly per variant: 1,141,669 baseline bytes
and 1,254,165 candidate bytes. Both Memcheck runs report zero errors and zero
definite/indirect/possible lost blocks. They are **not zero-live**:

- Baseline: 47,632 reachable bytes in 346 blocks; 675,746 allocations,
  675,400 frees, 123,272,240 cumulative requested bytes.
- Guard: 48,648 reachable bytes in 353 blocks; 729,203 allocations,
  728,850 frees, 131,069,337 cumulative requested bytes.

The +1,016 reachable bytes / seven blocks and whole-collector allocation totals
include construction, serialization, histories and additional certified output.
They are not marginal solver allocation costs, peak RSS, an ownership leak
diagnosis or a general boundedness proof. Both memory captures complete by
21:19:27.205Z. No suppression is introduced.

Only two dedicated executables are added, totaling 6,379,848 bytes under
`/tmp/calcium-point-extended.ejZUdW`; the existing Cargo cache and source snapshots
are reused. Four native/Memcheck output files total 4,791,668 workspace bytes.
About 16 GB remains free on /tmp. No cleanup or deletion occurred.

The release builds retain one unnecessary-mut warning in the collector closure.
No new Clippy, debug/full-suite or WASM qualification of this harness is claimed.
The collector contains draft CPU/allocation modes, but they have **not** been
exercised or accepted as benchmark evidence. In particular, the final measured
full report and cache history still need explicit qualification before timings.

## Integrity and next actual action

The new manifest binds the source versions, failed decoder capture, successful
initial/strengthened oracles, native/Memcheck outputs and two executable hashes.
Its verifier imports checkpoint 54's chain through 48 and rechecks all 956 live
source identities. It does not rerun the full historical 47-chain check.

Next: finalize a matched benchmark harness that validates the final actual full
report outside the timed window, preserves cold/preconditioned distinctions,
and measures CPU and allocation separately across the endpoint/history matrix.
Then execute relevant WASM controls/costs and decide retention explicitly. Existing
checksum-only draft modes must not be treated as completed cost qualification.
The separate power-sum optimization remains unselected/untimed. All five earlier
retained continuation transfers are unchanged; the full inventory is incomplete.
