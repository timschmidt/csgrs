# Native STL import and typed format failures in WASM

This is a narrow source admission under [the declared contract](contract-v1.md), `properties.md` and `docs/admission.md`. The full properties goal remains active. It improves the supported public pipeline without changing the native parser, geometry kernels, predicate policies, fixtures, or successful export bytes.

On parent `3d335db43eac00d003bc2fb0ed1beb2a9c19d79b`, the new real-JavaScript matrix passed one test and failed eleven; TypeScript lacked the importer and error types. Both existing consumers passed. [Baseline commands and raw outputs](baseline-v1/commands.json) preserve these findings.

`MeshJs.fromSTL(Uint8Array)` now delegates to native strict closed-PWN validation. Binary and ASCII encodings accept canonical empty, disconnected, nested, inverted and winding-multiplicity input within that native domain. Coordinates parse as binary32 and are promoted exactly. These facts concern the parsed mesh; they do not establish manifoldness, exact-source preservation, analytic fidelity, or inherited provenance.

STL import/export failures are ordinary `GeometryIoError` objects with a native `code`, format context, variant fields, and retained validation `cause` where present. Malformed `fromArrays` input uses `GeometryError`/`invalid_input`. This intentionally changes former string exceptions: use `message` for text, `code` for classification, and `cause.code` for native construction categories. Host errors require no `free()` and outlive mesh handles. Generated TypeScript declarations describe these fields.

The final shared source snapshot contains 2,830 pinned inputs. All six [native commands](native-validation-v1/commands.json) passed: all-feature tests (429 passed, one existing README-image test ignored), wasm-feature tests (346 passed), both all-target Clippy combinations with `-D warnings`, formatting, and docs with warnings denied. All three [WASM commands](wasm-validation-v1/commands.json) passed, including actual WASM Clippy and the real binding harness. All twelve new JS tests passed; both prior JS consumers and the old TypeScript check passed; the new TypeScript consumer compiled and executed. Main-tree formatting passed after byte-identical application.

The [broad-domain/resource probe](import-resources-v1/result.json) retained eight observations: 128 hostile maximum-count headers failed typed, minimum-subnormal/minimum-normal/maximum binary32 cubes imported and composed strictly, and disconnected/hole/inverted/double-winding fixtures preserved their independently encoded oriented facets and signed volumes. These externally contained probes do not establish a public work, heap or cancellation bound.

The [cost contract](cost-contract-v1.md), [95 process records](cost-v1/commands.json), [190 raw rows](cost-v1/rows.json), and [full summary](cost-v1/summary.json) retain favorable and unfavorable results. Existing exports use five alternating pairs per fixture, complete retained API outputs, independent integer geometry oracles, and full byte comparisons. Every process succeeded and every matched complete input/output agreed. First calls include lazy runtime compilation. Module loading, setup and verification are outside timing; host load was not controlled. No optimization is claimed.

| Matched export | Cubes | First baseline µs | First candidate µs | Paired ratio | Warm baseline µs | Warm candidate µs | Paired ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| binary | 0 | 1190.038 | 1007.401 | 0.8565 | 1.842 | 1.817 | 0.9941 |
| binary | 1 | 2227.277 | 1844.564 | 0.8380 | 2.593 | 2.723 | 1.0199 |
| binary | 256 | 5062.913 | 5416.128 | 1.0141 | 68.918 | 68.123 | 0.9189 |
| ascii | 0 | 923.537 | 1162.850 | 1.0832 | 3.227 | 3.111 | 1.0358 |
| ascii | 1 | 1816.295 | 1640.908 | 0.8973 | 53.138 | 52.186 | 1.0068 |
| ascii | 256 | 13110.461 | 12859.658 | 0.9796 | 8810.511 | 8686.634 | 0.9788 |

New native import pipelines have no equivalent completed baseline API. Their absolute observations retain strict receipts and every output mesh. The typed-error path likewise has a different contract from old string failures.

| New candidate path | Cubes | First µs/operation | Warm µs/operation | Warm retained Wasm linear bytes | Peak process RSS KiB |
| --- | ---: | ---: | ---: | ---: | ---: |
| pipeline binary | 0 | 2418.074 | 8.259 | 1507328 | 76152 |
| pipeline binary | 1 | 14916.977 | 222.109 | 1572864 | 93700 |
| pipeline binary | 256 | 60521.220 | 26800.692 | 10027008 | 115236 |
| pipeline ascii | 0 | 2300.392 | 9.504 | 1507328 | 76364 |
| pipeline ascii | 1 | 14869.681 | 299.991 | 1572864 | 90420 |
| pipeline ascii | 256 | 74814.239 | 36616.811 | 9895936 | 135264 |
| errors none | 0 | 512.647 | 18.810 | 1507328 | 76672 |

The error observation averages eight different failure triggers, retaining 520 host errors per process; it is not a per-variant latency claim. Wasm linear memory is reserved/high-water storage, not measured live Rust heap. The complete per-phase Node memory values and maximum observations remain in the raw rows.

| Artifact | Baseline bytes | Candidate bytes | Increase |
| --- | ---: | ---: | ---: |
| csgrs.js | 78724 | 80582 | 1858 |
| csgrs.d.ts | 12948 | 15809 | 2861 |
| csgrs_bg.wasm | 11161961 | 11229889 | 67928 |
| csgrs_bg.wasm.d.ts | 14645 | 14726 | 81 |

Only reachable STL failure variants receive runtime coverage here; all-feature compilation checks the remaining native match arms. The declared error union does not itself prove coverage. Existing validation categories survive, but deeper native proof witnesses and end-to-end construction provenance remain open. Other legacy WASM string errors, full release/distribution validation, and the complete pre-push workflow matrix also remain open. No push was performed.

The 35 pre-existing native files remain byte-identical and separately unadmitted: their original full-board and copper-neck runtime gates have not passed. The historical artifact audit remains failed/incomplete with 27,093 missing inherited paths. This checkpoint adds independently hashed physical evidence to 934,961 inherited expectations; it does not reverify that history or claim zero loss. Only the raw `current_work` ledger value changes; bytes and decoded keys outside it remain bound to revision `9025de62`.

See [the decision](decision-v1.json), [exact application](application-v1.json), and [checkpoint](checkpoint-v1.json) for source, command, artifact and limitation records.
