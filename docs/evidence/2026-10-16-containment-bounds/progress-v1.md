# Certified containment bounds acceptance

Hypermesh now uses a certified global supporting plane to prove that a query cannot be strictly inside a finite closed PWN solid. Source validation still runs first. The aggregate certainty of validation, bounds and equality must be Certified; approximate equality continues through the existing fallback. The public Rust and WASM paths consume the same native result.

This changes proof search without changing represented sets, source buffers, public policy, serialized geometry or the nonzero-winding contract. There is no new cache, representation or float conversion. The theorem supplies the shortcut: every neighborhood of a supporting-plane point contains points outside the source bounds and hence outside the represented solid.

Before implementation, the new four-test native probe and ten-test actual JavaScript consumer both passed against the qualified E32 libraries. The tests independently check all six planes with exact 1/10^12 offsets and pi translation, an enlarged AABB that contains exterior tetrahedron points, invalid triangle mutation after warming bounds, and unresolved trigonometric coordinates that preserve qualified receipts. JavaScript checks distinguishable binary64 neighbors. These supplement the entire existing source-validation and winding corpus; no policy, fixture or assertion was weakened.

| Command group | Exit | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: | ---: |
| format | 0 | — | — | — |
| containment-regressions | 0 | 29 | 0 | 0 |
| mesh-and-modeling-all-features | 0 | 708 | 0 | 8 |
| modeling-minimal | 0 | 161 | 0 | 0 |
| all-feature-all-target-clippy | 0 | — | — | — |
| minimal-all-target-clippy | 0 | — | — | — |
| all-feature-docs | 0 | — | — | — |
| cost-libraries | 0 | — | — | — |
| shell-syntax | 0 | — | — | — |
| real-consumers | 0 | — | — | — |
| wasm-clippy | 0 | — | — | — |

The real WASM suite passes 35 JavaScript tests, geometry-certainty TypeScript typechecking, and the compiled/executed IO and mesh TypeScript clients. Clippy denies warnings for affected all-feature/all-target, minimal/all-target and WASM configurations; affected docs deny warnings. All 2,830 final inputs match the qualified snapshot before the admission-only current_work update, and main formatting passes after applying the four paths.

The unchanged control and 21-case matrix compare against the corrected E32 implementation. Five alternating fresh-process pairs complete 2,560 control and 6,720 matrix queries with independently expected values and Certified receipts on both sides. Cold construction plus query, warm queries, result materialization, source drop, allocation calls/bytes, live/peak requested bytes, process resources and binary sizes are retained. Process scalar caches are not reset between cases or policies, as in the unchanged probe.

All 16 preregistered allocation gates pass. Targeted warm matrix rows require at least 50% fewer allocation calls in every pair; the tetrahedron control requires fewer calls and requested bytes in every cold/warm policy pair.

| Targeted warm case | Policy | Allocation-call reduction over five pairs |
| --- | --- | ---: |
| simple_face | strict | 98.81%–98.81% |
| simple_vertex | strict | 98.81%–98.81% |
| opposite_face | strict | 99.90%–99.90% |
| symbolic_face | strict | 99.95%–99.95% |
| symbolic_vertex | strict | 99.95%–99.95% |
| flat_face | strict | 99.14%–99.14% |
| simple_face | approximate | 98.81%–98.81% |
| simple_vertex | approximate | 98.81%–98.81% |
| opposite_face | approximate | 99.90%–99.90% |
| symbolic_face | approximate | 99.95%–99.95% |
| symbolic_vertex | approximate | 99.95%–99.95% |
| flat_face | approximate | 99.14%–99.14% |

Control observations (candidate divided by baseline for timing):

- strict / cold_construct_query: median instrumented elapsed ratio 0.270; allocation-call deltas [-1711, -1711, -1711, -1711, -1711]; requested-byte deltas [-234464, -234464, -234464, -234464, -234464].
- strict / warm_query: median instrumented elapsed ratio 0.224; allocation-call deltas [-1616, -1616, -1616, -1616, -1616]; requested-byte deltas [-219360, -219360, -219360, -219360, -219360].
- approximate / cold_construct_query: median instrumented elapsed ratio 0.201; allocation-call deltas [-1680, -1680, -1680, -1680, -1680]; requested-byte deltas [-232416, -232416, -232416, -232416, -232416].
- approximate / warm_query: median instrumented elapsed ratio 0.223; allocation-call deltas [-1616, -1616, -1616, -1616, -1616]; requested-byte deltas [-219360, -219360, -219360, -219360, -219360].

Across the complete measured matrix phases, every pair uses 559,048 allocation calls / 50,968,144 requested bytes before and 242,170 calls / 26,367,512 bytes after: reductions of 56.7% and 48.3%. The summed instrumented elapsed ratios range from 0.542 to 0.607. These phase sums exclude query-input preparation, source drop and process startup; they are not an end-to-end application timing claim.

Five cold rows incur the following increases in every pair. The unchanged probe shares process scalar caches across cases and policies; these observations do not isolate a per-case local cause.

| Strict cold case | Extra allocation calls | Extra requested bytes | Extra retained bytes | Extra peak requested bytes |
| --- | ---: | ---: | ---: | ---: |
| nested_inside | 3 | 144 | 144 | 144 |
| nested_internal_face | 9 | 504 | 400 | 400 |
| opposite_inside | 4 | 296 | 296 | 296 |
| shared_face | 25 | 1784 | 1784 | 1784 |
| tangent_origin | 1 | 176 | 176 | 176 |

Largest instrumented timing ratios, including every adverse row in decision-v1.json:

- simple_inside / strict / warm_query: 1.191 times baseline.
- cavity_inside / strict / cold_construct_query: 1.143 times baseline.
- simple_inside / strict / cold_construct_query: 1.140 times baseline.
- simple_outside / approximate / warm_query: 1.046 times baseline.
- shared_face / approximate / cold_construct_query: 1.040 times baseline.
- nested_internal_vertex / approximate / cold_construct_query: 1.035 times baseline.
- nested_internal_vertex / approximate / warm_query: 1.034 times baseline.
- shared_face / strict / warm_query: 1.034 times baseline.

Rows with any increase in allocation calls/requested bytes: 5. Rows with any increase in retained/peak requested bytes: 5. All pairs, source-drop observations and raw process measurements remain in [cost-v2/result.json](cost-v2/result.json).

Native control probe bytes: 8,341,528 before, 8,341,592 after. Matrix probe bytes: 8,367,416 before, 8,367,480 after. These are linked probe sizes, not the complete product.

The raw and processed WASM modules each grow by 103 bytes. Generated JavaScript glue and both declaration files are byte-identical to E32; [wasm-size-v1.json](wasm-size-v1.json) retains sizes and hashes.

This admission is a measured allocation reduction for the same completed containment contract. Instrumented latency is reported with all unfavorable samples and is not an unqualified production speedup claim. Separately retained baseline/candidate observations show the inverse-rotated cube query at (0.5, 0, 0.5) strengthening from strict Unknown / approximate qualified false to Certified false under both policies. Its certified lower supporting plane proves non-interiority of the retained set. This does not prove fidelity to the original cube or certify prior approximate intent. The original query and geometry remain in the consumer with stronger assertions; an added upper-face query preserves strict Unknown and qualified approximate false coverage. This capability gain is not included in the cost comparison.

The first WASM attempt stopped at the old fixed-Unknown assertion, so subsequent consumer and TypeScript stages did not execute in that attempt. It remains preserved. Final v2 changes only that JavaScript consumer; every native source, dependency, manifest and test is byte-identical to the complete candidate-v1 native qualification. WASM v2 uses a fresh target and executes the final consumer suite. No completed Cargo target was reopened.

Remaining limits:

- Containment and fallback arrangement still have no internal total-work, heap-byte, expression-growth or cancellation bound. The focused probe counts allocator requests and peak live requested bytes; it does not instrument scalar expression depth or coefficient growth.
- Five allocator-instrumented pairs establish these completed-query allocation changes; they do not establish uninstrumented production latency, a universal speedup or cross-library superiority.
- No new claim of end-to-end construction provenance, manifoldness or fidelity to an analytic source follows from the local containment receipt.
- C, C++, Python and Go do not expose this native containment query and were not rerun for this change. Real JavaScript and compiled TypeScript consumers were run.
- The published-package pipeline, full pre-push/CI matrix, coverage and broader properties plan remain open; nothing was pushed.
- The inherited artifact audit remains failed/incomplete with 27093 missing paths; fresh physical files are audited separately.
- The rejected 35-file native integration remains archived and excluded from runtime source; its original workload gates remain in force.
- The combined native package test command retains the pre-existing Cargo output-name collision warnings for the two basic examples. The same warnings occur in the qualified E32 baseline.

Next:

- Continue ordinary strict completion and typed failure repairs from the preserved native regressions, using smaller changes with independent exact expectations and unchanged original workload gates.
- Complete end-to-end certainty/provenance and the clean external documented distribution pipeline, then the remaining properties, workflow matrix and measured coverage.
