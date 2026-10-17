# Nonzero-winding containment acceptance

Hypermesh now classifies strict interior of the represented regularized nonzero-winding set. Signed crossings preserve multiplicity and cancellation; a finite exact direction family avoids tangential edge/vertex counts. Source-face queries use the existing native Boolean arrangement to distinguish an internal sheet from the represented set boundary. CSGRS and WASM consume that same native result and its certainty. Canonical-empty and typed invalid-source checks remain in force.

This is the deliberate semantic migration in [contract-v1.md](contract-v1.md), not a performance-only change. The signed-ray implementation is shared with Minkowski cell winding. Its checked arithmetic, retry order and cancellation call sites are preserved; diagnostic wording now says mesh winding rather than Minkowski winding. No new cache or float conversion is introduced.

The initial candidate repaired the original winding cases but rejected a previously successful cancelled flat-source query. The corrected arrangement admits one geometric ray only for one canonical facet bundle with zero aggregate transition in each operand. It retains the existing sheet strata, absolute winding proof and output consistency checks. Open or uncancelled sheets, including cancellation across different operands, remain rejected. The new lower-level guard test supplements the existing open-surface regression.

The current committed baseline ran 15 native tests: five passed and ten failed. Its real WASM consumer passed four of nine cases. The corrected candidate passes all 15 new native tests, the ten preceding source-validation controls and all nine WASM winding cases. The independent oracle evaluates exact signed box membership in every adjacent open octant. Tests separately establish source-plane support, exact volume, closed PWN topology, affine invariance, symbolic pi translation, flat multiplicity, touching/crossing/disconnected composition and unchanged source buffers. Wrapper agreement alone is not the oracle.

Candidate commands and complete raw output are retained:

| Command group | Exit | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: | ---: |
| format | 0 | — | — | — |
| containment-regressions | 0 | 25 | 0 | 0 |
| mesh-and-modeling-all-features | 0 | 704 | 0 | 8 |
| modeling-minimal | 0 | 157 | 0 | 0 |
| all-feature-all-target-clippy | 0 | — | — | — |
| minimal-all-target-clippy | 0 | — | — | — |
| all-feature-docs | 0 | — | — | — |
| cost-libraries | 0 | — | — | — |
| shell-syntax | 0 | — | — | — |
| real-consumers | 0 | — | — | — |
| wasm-clippy | 0 | — | — | — |
| ffi-tests | 0 | 10 | 0 | 0 |
| ffi-clippy | 0 | — | — | — |
| ffi-docs | 0 | — | — | — |
| ffi-library | 0 | — | — | — |
| empty_composition-c-compile | 0 | — | — | — |
| empty_composition-c | 0 | — | — | — |
| empty_composition-cpp-compile | 0 | — | — | — |
| empty_composition-cpp | 0 | — | — | — |
| empty_composition-python | 0 | — | — | — |
| boolean_failures-c-compile | 0 | — | — | — |
| boolean_failures-c | 0 | — | — | — |
| boolean_failures-cpp-compile | 0 | — | — | — |
| boolean_failures-cpp | 0 | — | — | — |
| boolean_failures-python | 0 | — | — | — |
| polyhedral_construction-c-compile | 0 | — | — | — |
| polyhedral_construction-c | 0 | — | — | — |
| polyhedral_construction-cpp-compile | 0 | — | — | — |
| polyhedral_construction-cpp | 0 | — | — | — |
| polyhedral_construction-python | 0 | — | — | — |
| cancelled_flat-c-compile | 0 | — | — | — |
| cancelled_flat-c | 0 | — | — | — |
| cancelled_flat-cpp-compile | 0 | — | — | — |
| cancelled_flat-cpp | 0 | — | — | — |
| cancelled_flat-python | 0 | — | — | — |

Real WASM consumers additionally preserve the 13 mesh and 12 IO regressions, both earlier JavaScript consumers, geometry-certainty TypeScript typechecking and compiled/executed IO and mesh TypeScript clients. Native all-feature tests include the affected Minkowski cancellation and resource cases. Real C, C++ and Python consumers also pass the existing empty-composition, Boolean-failure and polyhedral-construction contracts. New C and Python consumers each exercise 72 cancelled-sheet Boolean cases plus downstream composition; the C++ wrapper exercises all four operations and a continuation. These adapters expose status/errors, not an end-to-end certainty receipt; that interface remains open.

The unchanged allocator control completed 2,560 Certified queries. The expanded 21-case matrix ran 6,720 attempts across five alternating fresh-process pairs; every candidate query completed Certified (3,360). Baseline wrong values and typed failures remain separate categories, not faster successes. Both policies, cold construction plus queries, warm queries, result materialization, retained and peak requested bytes, release-at-drop, process RSS and binary sizes are retained. These are allocator-instrumented observations, not a production speedup claim.

Matrix completion categories: `{"baseline": {"certified_completion": 2080, "incorrect_result": 1280}, "candidate": {"certified_completion": 3360}}`.

Unchanged-control observations (all five pairs):

- strict / cold_construct_query: median elapsed ratio 1.922; allocation-call deltas [1044, 1044, 1044, 1044, 1044]; requested-byte deltas [180256, 180256, 180256, 180256, 180256].
- strict / warm_query: median elapsed ratio 1.593; allocation-call deltas [1008, 1008, 1008, 1008, 1008]; requested-byte deltas [169056, 169056, 169056, 169056, 169056].
- approximate / cold_construct_query: median elapsed ratio 2.240; allocation-call deltas [1072, 1072, 1072, 1072, 1072]; requested-byte deltas [182112, 182112, 182112, 182112, 182112].
- approximate / warm_query: median elapsed ratio 2.021; allocation-call deltas [1008, 1008, 1008, 1008, 1008]; requested-byte deltas [169056, 169056, 169056, 169056, 169056].

Largest comparable boundary-query regressions (instrumented medians; all rows remain available):

- opposite_face / approximate / warm_query: 59.728 times baseline latency.
- opposite_face / strict / warm_query: 57.035 times baseline latency.
- symbolic_vertex / strict / warm_query: 36.347 times baseline latency.
- symbolic_vertex / approximate / warm_query: 34.265 times baseline latency.
- opposite_face / strict / cold_construct_query: 34.183 times baseline latency.
- opposite_face / approximate / cold_construct_query: 33.199 times baseline latency.
- symbolic_face / strict / warm_query: 16.945 times baseline latency.
- symbolic_face / approximate / warm_query: 16.887 times baseline latency.

Native control probe bytes: 7,071,640 before, 8,341,504 after. Matrix probe bytes: 7,098,368 before, 8,367,392 after. These linked probes newly retain the arrangement code; they are not the entire product size.

Every expanded-case pair and any unfavorable observation is retained in [cost-v1/result.json](cost-v1/result.json). Latency ratios are reported only where both variants return the independently expected Certified result. The boundary arrangement is additional work and its increases are part of this admission.

Failed link/fixture/compile attempts and the rejected first candidate remain intact. No tests ran in the link or compile failures. The wrong-fixture attempt reran the older 12-test probe; it supplies no expanded-test evidence. The first candidate WASM script stopped at its flat-source test, so its TypeScript stages were not run. Candidate v2 passed focused native and complete WASM consumers but its new unit test failed to compile because unwrap_err required Debug on the success type. Candidate v3 changes only the error-extraction syntax, retaining the expected error assertion and all production code. No prior build target was reopened or old attempt rewritten.

All 2,829 final inputs match the qualified snapshot before the admission-only current_work ledger update. Main formatting passed after applying exactly nine implementation/test paths. No dependency, source-policy, fixture-resolution or validation exception was used.

Remaining limits:

- No total-work, heap-byte, expression-growth or cancellation bound for containment or its native boundary arrangement; expression depth and coefficient growth were not instrumented in the focused cost probe.
- Source-boundary queries can now require arrangement work; all measured increases are retained, with no optimization claim.
- The tested rational, affine and pi-translated domain is evidence, not a theorem establishing all possible closed-PWN symbolic inputs.
- Published-package pipeline, full pre-push/CI matrix, full provenance chain and coverage remain open.
- The inherited artifact audit remains failed/incomplete with 27093 missing paths; new physical files are audited separately.
- The rejected 35-file integration remains archived and excluded from runtime source.
- The combined package test command retains the pre-existing Cargo output-name collision warnings for the two basic examples; the same warnings are present in the qualified E31 baseline. They are not caused by this patch.

The next cost opportunity is to reuse certified global bounds for non-interior boundary rejection. It remains unimplemented and uncredited; any optimization must preserve this corrected completion corpus.

Nothing was pushed. The full properties goal remains active.
