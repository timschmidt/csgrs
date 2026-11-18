# Solver Jacobian context review

The required phase-7 review of `hypersolve/src/jacobian.rs` found a public
forward-difference bug at `557a349f9`. For `x²`, model value `1`, context value
`3`, and step `1`, the implementation returns `-5` instead of `7`: it evaluates
the baseline at the context but perturbs the model value. A weighted case with
fixed, unused and inactive roles also fails. Missing used bindings still return
the existing typed evaluation error.

All six original declarations were read, along with `EvaluationContext`,
residual evaluation and the immediate solver caller. They belong to the solver's
numerical proposal boundary, category 4 in plan section 7.1. None stores an
interval or supplies general interval arithmetic to extract. The ordinary solver
reconstructs its context from its current model each iteration; that synchronized
path is an unchanged control. The returned `f64` matrix is not a certificate.

Three isolated repairs remain unaccepted:

| Candidate | Change | Whole-call stack result against original |
| --- | --- | --- |
| `9e88db0ca` | Read the context binding before perturbing | +32 bytes in all 16 groups, with and without dispatch instrumentation |
| `89b58eb57` | Shift through one context-map entry | −80 bytes in 14 groups; +128 bytes in both empty-row groups |
| `4bd27bdb9` | Return the complete owned, perturbed context | −128 bytes in 14 groups; +176 bytes in both empty-row groups |

The eight-case corpus includes synchronized quadratic and linear inputs, a
mixed two-variable polynomial, weighted/fixed/unused/inactive roles, an empty
row set, and both inequality hinges. Four campaigns run 1,536 fresh processes,
including two stack-fill patterns and four repetitions. All 768 paired matrix
outputs match independent expected binary values; 256 allocation and output-drop
pairs match. The first candidate also preserves 64 full dispatch snapshots.
These are finite-corpus observations, not universal bounds or latency evidence.
The stack regressions prevent retention; binary-size permission does not waive
that gate. No timing campaign or size-account change follows from this review.

At `9e88db0ca`, the full all-feature owner library/integration suite, doctests,
strict docs, Clippy and formatting pass on Rust 1.95. Stable-toolchain focused
tests and all-target strict Clippy pass. The later ownership variants pass four
focused tests, including generation, source-context preservation and the
original independent regressions; they were stopped at the resource screen
before repeating full qualification. Tests assert scalar operation counts and
zero Hyperlimit decisions. The probe's initial dead-code warning is preserved;
an explicit statistics accessor resolves it without suppressing the warning.

The original source is restored clean. Three original Jacobian tests, strict
all-feature/all-target Clippy and formatting pass. Both restored probe binaries
are byte-identical to their baseline artifacts and pass their functional checks.
No production repair or new test is retained.
Six reviewed dispositions are added to the partial inventory. Zero/underflowed
step validation, missing numeric-estimate handling and signed symbolic
inequality conventions remain separate review questions. Neither this audit nor
the preserved failing fixtures establishes broader Jacobian correctness.

Evidence: `docs/evidence/2026-11-17-interval-jacobian-context`, local commit
`a099a4fd21fd8d106acfb874bb943dedc628794d`; 1,792 files / 1,280,121 bytes,
SHA-256 `33911caecf0a7cd0d30a440b434f9bbff6331f88c514a7a711b71a4dbaeb42ce`.
Commands, source patches, locks, failed witnesses and all raw observations are
retained. Earlier archive paths and manifest rows are unchanged. No push.

## Reassessment under the bounded memory policy

At `29e2bfdf3`, the original rebound-square and weighted mixed-role witnesses
both failed again. The missing-used-binding control passed. Every candidate in
the table was measured against that revision and the fixed `c9f1f9ac5`
baseline. Twelve workloads include the original eight, three rebound cases with
signed steps, and a dense 8 × 8 polynomial matrix. The older arithmetic
cannot produce correct results on the independently rebound inputs. Its cost
control uses the same center, step and expected matrix with synchronized
model and context coordinates. The wrong old results are recorded separately.

Each workload ran cold and warm in both native feature configurations. Four
fresh repetitions and two fill patterns measured byte-resolution callee stack
on a guarded 2 MiB worker. The callback inspected and destroyed its matrix.
Heap measurements separately captured allocation events, peak requested bytes,
the live result and its destruction. Original inputs and caller frames were
outside the callee boundary. All candidate matrices matched the independent
expectations. The trace-enabled observations matched the preceding revision's
full snapshots on equivalent completed work.

| Repair | Groups outside the memory caps, out of 48 | Largest incremental stack charge |
| --- | ---: | ---: |
| Original context lookup with cloned operands | 15 | 32 B |
| Same lookup with borrowed operands | 0 | 0 B |
| One entry lookup with borrowed operands | 2 | 32 B |
| Returning an owned shifted context | 2 | 80 B |
| Direct iteration of the perturbed residuals | 44 | 112 B |
| Shared dense-row extraction with separate lookups | 22 | 160 B |
| Shared dense-row extraction with one entry lookup | 2 | 16 B |
| Separate occupied and vacant entry handling | 2 | 32 B |
| Mutate an existing binding, insert when missing | 2 | 80 B |
| Drop the scratch context before extracting rows, separate lookups | 8 | 32 B |
| Drop the scratch context before extracting rows, mutable binding | 3 | 112 B |
| Reuse one working binding map across columns | 48 | 240 B |

The native caps were 128 bytes and 1% incrementally, 256 bytes and 2%
cumulatively, a 16 KiB callee ceiling, and at least 1 MiB free after the guard
and caller. The original lookup took the warm empty-row call from 1,329 to
1,361 bytes. The borrowed lookup at `9268e4371` preserved stack usage,
requested allocation events, peak heap and output-drop measurements in all
48 groups. Its largest callee measurement was 12,376 bytes and its minimum
headroom was 2,074,472 bytes. Peak requested heap reached 2,361 bytes on the
ordinary corpus and 19,444 on the dense fixture. These comparisons cover one
native worker. There is no allocator-usable or whole-process RSS claim.

The borrowed implementation passed the standalone production-client size caps.
Its native file grew by 376 bytes and its union of defined function ranges by
nine bytes. Both the linked WASM file and its code section shrank by 25 bytes.
The uninstrumented public WASM client checked 1,568 entries per instance in
forward and reverse order. Independent native checks also distinguished the
old −5 from the corrected 7. In the separate native measurement executable,
the complete file shrank by 928 bytes, but allocated RELRO padding grew from
992 to 1,832 bytes. The 840-byte incremental charge exceeds the section's 1%
cap. Its fixed baseline was 800 bytes. No growth was charged to the production
memory or size accounts for this isolated repair.

Runtime also prevents retention. On the weighted fixed/unused/inactive case,
a ten-pair crossover found the trace-enabled binary slower on all three
physical cores. A second crossover reversed pair order and collected 24
more process pairs per core. Median paired slowdowns were 4.22%, 5.15% and
4.96% on CPUs 3, 5 and 15. All three paired-median bootstrap intervals stayed
above zero. The default-feature build was faster on this case. The timings
include proposal construction and output destruction. Trace recording was
off during timing and the full snapshots were collected separately.

Primary `3cd22cc86` adds 392 supported public matrix cases, comprising 2,352
bitwise entry checks, and preserves the typed error for an unbound active
symbol. A fixed coordinate can be rebound independently and still alter the
derivative of another coordinate. Free and unused columns preserve IEEE signed
zero for negative steps, and fixed columns retain positive zero. Restoring all
authored bindings, forcing every zero positive and perturbing a fixed column
each failed the new tests. Public documentation gives the reproducible
free-coordinate defect and the model/context update that supplies the intended
center. [Raw measurements and reproduction material](../evidence-archive.tsv)
