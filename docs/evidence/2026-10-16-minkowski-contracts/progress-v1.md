# Represented-set Minkowski sums: local admission

The final implementation satisfies the focused contract in
[implementation-contract-v2.md](implementation-contract-v2.md). Hypermesh now
computes the regularized sum of represented nonzero-winding solids, including
disconnected components, concavity and cavities. Both operands validate before
an empty identity; unused positions do not change the solid. Explicit
`minkowski_hull` entry points preserve the previous point-set interpretation.
The migration permits different triangulation and vertex order for general
decomposed solids. This is a correctness and completeness change, with no
optimization claim.

Hyperreal supplies the two exact proofs required by the unchanged original
16×8 sphere workload: a bounded built-in root-square projection and selected
eighth-angle radical leaves. These are query projections; authoritative symbolic
coordinates and their serialization remain intact. CSGRS delegates to native
implementations, and WASM preserves typed errors and validates resource counts
before numeric conversion. Local certification does not establish the provenance
of an earlier construction or fidelity to an intended analytic sphere.

The semantic regressions use independently derived exact volumes and membership
witnesses: separated sums have volume 16 or 32 and retain their gaps; an L prism
sum has volume 36 and retains its notch; a hollow sum has volume 124 and retains
its cavity. Nested, doubled, reversed and canceling shells exercise nonzero
winding. Affine and repeated composition, exact tiny gaps, symbolic translations,
subsequent Booleans, closed-PWN validation, typed invalid/undecided outcomes,
small collection budgets, cancellation and recovery are checked separately.
The complete original README and sphere fixtures remain in the tests.

## Executed checks

[qualification-v1](qualification-v1/result.json) ran 48 serial commands: 47
passed, and the benchmark consumer helper failed its target-path assertion
before writing output. The identical helper content was placed in this evidence
directory, and the original benchmark inputs passed in
[benchmark-repair-v1](benchmark-repair-v1/result.json). That final phase passed
all 11 commands, including formatting, root all-feature tests, native and WASM
Clippy with warnings denied, docs, actual WASM/TypeScript consumers, benchmark
build and consumers, and the matched library build.

Only the three files pinned in
[wasm-count-adjustment-v1.json](wasm-count-adjustment-v1.json) changed after the
broad phase: the WASM wrapper, its JavaScript tests and README. The final phase
checks those changes. All scalar/native sources and their tests remain identical
to the broad passing checks. The actual generated WASM module first reproduced
60 incorrectly accepted or misclassified invalid-count cases. The final consumer
passes 96 invalid-count cases, valid zero and upper endpoints, both policies,
typed exhaustion and subsequent recovery.

| Check group | Passed | Existing ignored |
| --- | ---: | ---: |
| Hyperreal all features / default | 1,037 / 930 | 0 / 0 |
| Hyperlimit, Hyperlattice and Hypersolve | 1,545 | 0 |
| Hypertri | 193 | 0 |
| Hypermesh | 247 | 7 |
| Hypercurve | 2,496 | 33 |
| CSGRS minimal / curve / all features | 130 / 286 / 403 | 0 / 0 / 1 |
| FFI | 10 | 0 |
| Scalar / root / triangulation doctests | 28 / 1 / 4 | 0 / 0 / 0 |

These are per-command totals, not a count of distinct tests across configurations.
The broad phase also passed the recorded relevant Clippy/check/doc combinations
and real C, C++, Python and Go consumers, including empty composition and
construction-error regressions. Native mesh doctests contain no test cases.
The original benchmark consumers retain primitive catalog, profile
extrude/revolve/twist/sweep/loft, buffers/hull/Minkowski and cube-sum inputs.
No fixture resolution, policy or required certified outcome was weakened.
Every unsuccessful diagnostic/build phase remains recorded in
[diagnostic-findings-v1.md](diagnostic-findings-v1.md) and the summary.

## Costs and retained artifacts

[resources-v1](resources-v1/result.json) passed all 12 commands and all 104 child
trials: 78 timing samples and 26 separate allocation samples. The frozen pre-edit
cube driver was replayed against old and final libraries with matching Cargo unit
features, profile, compiler and flags. The raw samples, original pre-edit samples,
commands, fingerprints, locks, host/toolchain details and all unfavorable results
are retained in [qualification-summary-v1.json](qualification-summary-v1.json).
Timing medians below use three samples per variant and temperature.

| Completed workload | Cold median, ms | Warm median, ms |
| --- | ---: | ---: |
| Cube sum, baseline → final | 0.318828 → 0.439579 | 0.161929 → 0.193287 |
| Disconnected sum, final | 1.371336 | 0.996671 |
| Concave sum, final | 5.551349 | 4.760463 |
| Hollow sum, final | 301.264082 | 296.228218 |

The cube samples regress by about 38% cold and 19% warm. Its requested Rust
allocation bytes increase from 52,408 to 89,496 cold and 42,200 to 67,208 warm;
reallocation bytes are recorded separately. Peak additional requested bytes are
43,008 → 46,632 cold and 35,680 → 35,760 warm. Cold retained bytes after dropping
the output rise from 7,328 to 10,872; both warm cube samples retain zero additional
bytes. These costs are retained with the correctness repair. No improvement in a
lower priority is used to justify reduced meaning, evidence or supported inputs.

The general sum rows have no equivalent correct old workload. Their cold/warm
peak additional requested bytes are 133,322/101,946 (disconnected),
320,782/235,566 (concave), and 4,253,034/3,294,226 (hollow). Warm retained deltas
after output drop are 9,904, 4,400 and 111,696 bytes respectively; these samples
do not establish bounded cache retention. OS process peak RSS ranges across
timing/allocation trials are 8,988–9,512 KiB, 9,380–9,828 KiB and
13,924–14,512 KiB respectively. The allocator measures requested Rust bytes,
excluding native allocator internals; process RSS is a separate measurement.

The retrospective captured scalar queries change from Undecided to Certified.
Root-square cold medians are 2.114505 → 2.271774 ms; eighth-angle medians are
0.460208 → 0.546362 ms. Their final warm medians are 0.002690 and 0.002260 ms.
These are different completion categories, so they establish no same-completion
speedup. The rational control stays certified; the unresolved trigonometric
control stays undecided. The latter consumes more allocation bytes in the final
implementation, also retained in the raw record. Timer-scale control differences
are not claimed as optimizations.

For geometry trials, timing includes input construction and output
materialization; independent validation and serialization replay follow it.
Warm geometry trials repeat construction in the same process. Scalar parsing is
outside timing, and a warm scalar query reuses the same retained value. Cold means
a fresh process/proof state, not a cold filesystem. Each scalar query preserves
the exact serialized input. Every geometry case is deterministic across the
recorded runs. The original cube artifact is byte-identical to the pre-edit
artifact, independently of its volume/topology checks:
`71cccb19f8550dc9be88a31d53812d6cb223fc9c72b1d7a8ca49ed8ceff91215`.

Matched Hypermesh archives grow from 5,038,386 to 5,183,942 bytes; Hyperreal from
11,616,304 to 11,625,310 bytes. The cube timing executable grows from 22,903,088
to 23,995,048 bytes under this build profile. All library, executable and per-file
source sizes are recorded. Captured scalar serialization contains 309/55
computable-node occurrences, depths 16/11, and maximum rational
numerator-plus-denominator sizes of 107/213 bits. These describe serialized trees,
not unique DAG nodes, internal proof expansion peaks or general growth bounds.

## Preservation and remaining work

[checkpoint-v1.json](checkpoint-v1.json) records source/dependency pins and the
complete artifact audit. Admission permits no new missing artifact or hash
mismatch. The prior 27,093 missing paths retain their expected hashes in the
pinned parent audit; their historical loss is not repaired by this milestone.
All 35 separately unadmitted Hypercurve/Hyperdrc files retain their previous
hashes. Only the raw `current_work` value in the execution ledger may change;
all other bytes and decoded keys must match revision `9025de62`.

The pre-existing native 26-input/merged-shell union failure remains open.
Its complete analytic box fixture still fails on old libraries, while native
binary accumulation certifies volume 124; the final implementation uses those
binary unions. This does not repair batch-arrangement sharing or claim P21
completion. Bare-mesh inherited provenance also remains open.

Collection limits and cooperative cancellation do not bound total primitive
work, scalar growth, heap bytes or elapsed time. Native validation, hull and
Boolean calls cannot be interrupted internally; the existing radical tower's
structural limits are not a complete total-work/cancellation contract. Validation
uses external 1,800-second and 8-GiB address-space containment, except WASM/Node
commands use wall containment only. Cost children have 120-second wall limits.
These runs establish neither full internal resource bounds nor worst-case costs.

The clean external release/user path, full workflow matrix, formal/fuzz/coverage
targets, separate native changes and full properties plan remain open. No push
was performed. This record admits only the focused implementation and evidence
described above.
