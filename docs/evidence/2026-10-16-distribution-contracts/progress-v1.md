# Distribution and concatenation progress

Baseline `00e612307`. This milestone restores explicit public failure and
validates sources before distribution shortcuts, with checked native
concatenation and reusable qualified PWN facts. It is focused acceptance,
not completion of properties.md, full CI, or a shipped release.

The owning-layer/domain contract is in [contract-v1.md](contract-v1.md);
the deliberate Result-returning API and invalid-source migrations are in
[implementation-contract-v1.md](implementation-contract-v1.md). Reproduction
established empty-on-error wrappers, erased native causes, unchecked huge
copy counts, invalid source shortcuts, and release-mode index rebasing that
could wrap an invalid index into another input.

Kept: typed strict wrappers and native causes; native checked concatenation;
shared closed-PWN validation with combined face/edge certainty; checked
copy capacities; exact full-workload semantic and preservation evidence.
Already satisfied and preserved: canonical-empty support and the successful
original 8/16/12 distributions and 22-copy/duplicate concatenations.

Independent acceptance covers exact membership and per-face volume proofs,
strict topology, empty accumulation, self-subtraction, touching/overlapping
layouts, signed spacing, arc midpoint placement and winding multiplicity.
Every rational-box component also passes an independent exact six-face
coverage oracle, including the full 22-copy workload. Structural copying
of an open mesh makes no closed-topology claim.

The original expanded arc-volume scalar query declines on both revisions.
The [volume witness](volume-oracle-v1.md) proves every face contribution
before adding exact rationals; all copies and downstream tests remain.
This does not extend general scalar expression recognition. All failed
builds and diagnostics remain in [diagnostic-findings-v1.md](diagnostic-findings-v1.md)
and the [complete summary](qualification-summary-v1.json).

## Validation

The final `qualification-v1` phase passes all 30 commands on frozen source.
Counts below are per feature configuration and overlap; they are not a
count of distinct workflows or coverage.

| Command/configuration | Passed | Ignored |
| --- | ---: | ---: |
| native-mesh | 245 | 7 |
| root-minimal | 112 | 0 |
| root-curve | 268 | 0 |
| root-all-features | 385 | 1 |
| ffi-tests | 10 | 0 |
| root-doctests | 1 | 0 |
| native-doctests | 0 | 0 |

Formatting, native/root/minimal/FFI Clippy with warnings denied, public
documentation, both benchmark builds and all four original benchmark
consumers pass. Rebuilt C, C++, Python and Go consumers pass, including
Boolean failures and empty composition. JavaScript/WASM certainty, query,
geometry and invalid-input checks plus strict TypeScript declarations pass.
The additional direct all-feature release library build passes. A separate
benchmark build supplies matching baseline/candidate profiles and features
for the fixed cost phase; the preparation record verifies native dependencies.

## Preserved work and measured cost

All 60 timing and 20 allocation children complete. Every replay of each of
the five fixtures is byte-identical to its original exact coordinate/index
artifact; numerical semantics are established separately above. The fixed
three-pair cold/warm timing matrix includes input construction, and allocation
uses separate binaries. Serialization is an untimed replay after the
post-drop retention snapshot. These short single-host probes are not an
optimization or competitive speedup claim; all unfavorable samples remain.

| Fixture | State | Baseline median ns | Candidate median ns | Candidate / baseline |
| --- | --- | ---: | ---: | ---: |
| linear-8 | cold | 178218 | 181768 | 1.020 |
| linear-8 | warm | 940 | 3320 | 3.532 |
| grid-4x4 | cold | 256073 | 272131 | 1.063 |
| grid-4x4 | warm | 3220 | 3270 | 1.016 |
| arc-12 | cold | 802915 | 838043 | 1.044 |
| arc-12 | warm | 1130 | 1200 | 1.062 |
| feature-merge-22 | cold | 292560 | 308609 | 1.055 |
| feature-merge-22 | warm | 256113 | 224904 | 0.878 |
| merge-duplicate | cold | 37018 | 33088 | 0.894 |
| merge-duplicate | warm | 3060 | 3230 | 1.056 |

The complete timing/allocation rows, requested-byte peaks, post-drop
retention and driver/library sizes are retained in the summary and
`resources-v1/measurements/`. Toolchain, hardware, feature fingerprints and
source byte/line deltas are in `cost-host-v1.json`. Requested allocation
bytes exclude allocator overhead and do not bound total heap use.

The warm linear median ratio is 3.532, with raw baseline samples
890/940/4240 ns and candidate samples 3320/6810/830 ns. This spread does not
establish a stable latency change; all six samples remain retained. Its warm
allocation counts are zero on both revisions. Cold distribution validation
adds ten allocations per fixture (3,140–3,460 requested bytes); merge allocation
counts, distribution post-drop retention, and every warm allocation/retention
measurement are unchanged. Cold medians for linear/grid/arc increase by about
2.0/6.3/4.4 percent. These costs accompany the restored failure contract and
are not concealed by the favorable duplicate-merge and warm 22-copy samples.

## Remaining boundaries

Capacity checks precede copy construction, and overflow/failed reservations
are typed. They do not bound scalar/shared-buffer allocations, expression
growth, total work, retained heap bytes, or cancellation. Dangerous baseline
children used external containment; Node/WASM retains wall containment but
has no address-space cap. No internal resource-bound claim is made.

Local certainty and native PWN evidence do not reconstruct inherited
construction intent. General provenance, wider symbolic completion, clean
external package/release validation, full CI, formal/fuzz/coverage targets
and the 35 separately unadmitted native files remain open.

Closure requires all currently available inherited evidence and every new
build/overlay artifact to retain its hash. The 27,093 historical missing
artifact expectations remain recorded without recovery claims. Only the
protected ledger’s raw `current_work` value is updated; every other byte
and decoded key must match revision `9025de62`. No push is performed.
