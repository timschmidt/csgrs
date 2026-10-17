# Containment input validation acceptance

The native query now reuses the existing closed-PWN validator before bounds or boundary shortcuts. Canonical empty geometry remains a decided, Certified false. The modeling and WASM entry points share this implementation; the latter preserves typed GeometryError failures and its existing Unknown representation.

This is the input/evidence migration in [contract-v2.md](contract-v2.md), not a claim that every valid PWN containment answer has correct nonzero-winding semantics. Strict queries now remain undecided when source validity cannot be proved, even if bounds could exclude the point. Approximate validation keeps its qualification. No fixture, policy, assertion, native winding algorithm, or prior certified-success test was weakened.

The frozen baseline ran ten native tests: four passed and six failed. The actual baseline WASM consumer ran thirteen tests: ten passed and three failed. A separate baseline invocation under an 8-GiB address-space limit failed during WASM instantiation before tests; it supplies no test evidence. Its raw output and the successful 64-GiB reproduction are retained. The first preparation script also stopped on a documentation-text mismatch, and its premature validation startup failed before running Cargo. Both attempts and their unused snapshots/targets remain preserved.

Candidate checks (each command and full output is retained):

| Command group | Exit | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: | ---: |
| format | 0 | — | — | — |
| containment-inputs | 0 | 10 | 0 | 0 |
| mesh-and-modeling-all-features | 0 | 688 | 0 | 8 |
| modeling-minimal | 0 | 142 | 0 | 0 |
| all-feature-all-target-clippy | 0 | — | — | — |
| minimal-all-target-clippy | 0 | — | — | — |
| all-feature-docs | 0 | — | — | — |
| cost-libraries | 0 | — | — | — |
| shell-syntax | 0 | — | — | — |
| real-consumers | 0 | — | — | — |
| wasm-clippy | 0 | — | — | — |

The real WASM script passed all 13 mesh and 12 IO tests, both earlier JavaScript consumers, geometry-certainty TypeScript typechecking, and the compiled/executed IO and mesh TypeScript consumers. Independent tetrahedron halfspaces and invalid edge/index counts are the semantic oracles; agreement between wrappers only verifies propagation. The cache tests cover clones, concurrent readers, mutation and strict/approximate/strict source validity.

Five alternating fresh-process pairs completed 2,560 Certified queries with the same full expected bool rows. Cold measurements include source construction, all queries and result materialization; warm measurements reuse the same meshes. Allocator counters report requested Rust bytes, not total heap capacity. Timings are instrumented observations, not a production speedup claim. All adverse samples, drop measurements, whole-process RSS, binary sizes, compiler/hardware/lock/source pins and external limits are retained in [cost-v1/result.json](cost-v1/result.json).

- strict / cold_construct_query: median candidate/baseline elapsed ratio 1.038; allocation-call deltas [128, 128, 128, 128, 128]; allocated-byte deltas [15872, 15872, 15872, 15872, 15872].
- strict / warm_query: median candidate/baseline elapsed ratio 0.822; allocation-call deltas [0, 0, 0, 0, 0]; allocated-byte deltas [0, 0, 0, 0, 0].
- approximate / cold_construct_query: median candidate/baseline elapsed ratio 1.013; allocation-call deltas [128, 128, 128, 128, 128]; allocated-byte deltas [15872, 15872, 15872, 15872, 15872].
- approximate / warm_query: median candidate/baseline elapsed ratio 0.884; allocation-call deltas [0, 0, 0, 0, 0]; allocated-byte deltas [0, 0, 0, 0, 0].

The unchanged winding diagnostic still disagrees with nonzero-winding expectations in both fixtures and all six policy observations: nested outward shells and coincident opposite shells. This is an open correctness gap, preserved in [winding-followup-v1/stdout](winding-followup-v1/stdout); the validation guard does not repair it.

All 2,827 final source inputs match the qualified candidate before the admission-only ledger update. Main-tree formatting passed after applying the five files. No completed Cargo target was reopened. No source/fixture/dependency exception was needed.

Remaining limits: no internal total-work, heap-byte or cancellation guarantee; no complete set-membership, end-to-end provenance, release, full CI/pre-push, coverage or whole-history preservation claim. The 27,093 inherited missing artifact paths remain unresolved. The rejected 35-file native integration and its 22 failing regression tests remain preserved. Nothing was pushed.

Next: repair the retained winding mismatch at the native layer, then continue the full properties plan.
