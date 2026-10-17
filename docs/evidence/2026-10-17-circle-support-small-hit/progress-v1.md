# Direct small-cache-hit return: rejected at index activation

**Reject this proposal as written.** The new small-hit early return passes its
focused semantic checks but fails the predeclared repeated cold-batch cost gate.
The main implementation is unchanged. The earlier 35-file native integration
remains rejected; its independently admitted fixes remain intact. E44 remains
an unadmitted historical proposal with its own measurements, not a retained
fallback. Neither evaluation establishes general exact KiCad stroke fidelity.

The owner is Hypercurve's operation-local circular-support relation cache,
under P4, P10, P19, P20 and P36. The candidate preserves E44's exact keys,
ordered matching priority, original retained witnesses, symbolic and very-large
coefficient fallback, single-policy batch lifetime and 32-entry activation
threshold. It moves small retained hits before index bookkeeping, with one scan
on a small miss. The candidate was isolated throughout; review the complete
proposal patches and the incremental patch against E44 in the evidence archive.

A read-only replay of the E44 baseline and candidate completed on the current
production revision before executable qualification. Those two short shared-hit
observations are diagnostic reproduction, not a new matched performance claim.
The new candidate passed metadata type checking and strict direct Clippy, then
compiled successfully with the same optimized flags, toolchain, dependencies,
features, tests and diagnostic modules as the hash-verified E44 baseline.
The baseline executable was reused read-only with its source and build proof.
Both binaries passed all nine focused cache contracts. The candidate diagnostic
test harness also passed strict direct Clippy. These are not Cargo-matrix or
full-library passes for this revision.

All 20 preflight processes and all **140 matched processes / 280 timed phases**
completed. Every phase contains at least 65,536 queries; cold uses a fresh
batch cache including its creation and destruction, while warm uses one prepared
cache. Each query validates independently derived 3-4-5 intersections and both
circle equations, with complete ordered rows and a strict Certified receipt.
Input construction, reference preparation, printing and process startup stay
outside the phase timers and inside process observations. Cold here means the
batch cache, not a reset of every scalar or process cache. The ten original
sizes and all seven paired repetitions are preserved, with deterministic case
shuffling and alternating variant order.

Median seconds per complete repeated phase:

| Pairs per batch | Baseline cold | Candidate cold | Baseline warm | Candidate warm |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.096354464 | 0.096374713 | 0.049691138 | 0.048456693 |
| 4 | 0.113084803 | 0.113023576 | 0.065013752 | 0.064983254 |
| 8 | 0.132276100 | 0.132736609 | 0.077647523 | 0.078244542 |
| 9 | 0.135333100 | 0.134795206 | 0.079404962 | 0.080783107 |
| 16 | 0.171458282 | 0.170313101 | 0.100205579 | 0.101741273 |
| 31 | 0.215920700 | 0.211437829 | 0.137983327 | 0.137300524 |
| 32 | 0.213163810 | 0.212096263 | 0.137654290 | 0.090976975 |
| 33 | 0.215810937 | 0.238608847 | 0.139333005 | 0.090868642 |
| 64 | 0.289479904 | 0.206200529 | 0.210934034 | 0.090374477 |
| 128 | 0.436470671 | 0.191875476 | 0.356440213 | 0.093358681 |

At 33 pairs the cold median rises from 0.215810937 s to 0.238608847 s:
**10.5638344% slower**. All seven paired differences are positive, and the
candidate minimum (0.234221130 s) exceeds the baseline maximum (0.219813662 s).
This satisfies every part of the predeclared rejection criterion. Favorable
large-batch results and all small differences remain in the raw evidence;
none cancels the stable supported-workload regression. The source initializes
the index from retained relations on the first eligible lookup at length 32.
That is an implementation observation, not a measured allocation attribution.

The subsequent support-matrix preflight invocation was refused by its existing
assertion because the small-batch result contains a decisive rejection. The
script exited 1 before creating an output directory or launching a benchmark.
Its tool-reported assertion is recorded in the decision; no raw redirected
process log exists for that invocation. Do not count this as a completed support
case or silently bypass the guard. The prepared support, allocation, full-library,
all-feature and grouped-comparison helpers remain unexecuted, except for that
refused support preflight. No new allocation or package-size claim is made.
The required downstream, native/report, binding and release gates remain open;
continuing them cannot retain this already rejected revision.

Preparation first selected the repository-root rustfmt style because the draft
lived under target/. That check failed and its outputs remain preserved. The
byte-identical candidate passed with the owning crate's explicit rustfmt config;
no source edit or validation weakening was used to obtain that pass.

Storage was part of the declared execution contract. Metadata checks used a
fresh runtime directory; executable checks used a fresh /dev/shm directory with
an explicit user-quota guard. Each compiler/test command retained a 1,800-second
wall and 8 GiB address-space limit, at least 2 GiB host available memory and
256 MiB runtime/quota reserve, with a separately declared 512 MiB repository
reserve for small controls/evidence. These are new experiment contracts, not
retroactive changes to prior native CPU/wall gates. No closed Cargo target was
reopened. The requested directory on Disk A is still absent: OS ownership and
password-required sudo blocked creation. There was no automatic-review denial.
A writable larger volume remains needed for fresh full downstream validation.

Only current_work changes in the execution ledger. Every new physical file
path is independently hashed, including metadata and executable roots; the
compressed Git archive is verified byte-for-byte against its originals.
The inherited 27,093 missing paths remain disclosed. This is not a successful
whole-history preservation audit or completion of the properties plan.

A successor must address index activation cost with the original sizes, exact
witnesses and full-scale workloads intact. Deferring construction until its
cost can be amortized is an untested hypothesis, not an admitted optimization.
