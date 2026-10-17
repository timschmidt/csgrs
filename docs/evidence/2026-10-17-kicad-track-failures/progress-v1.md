# Typed KiCad track failures and coincident disks

Retained a six-file change after current-revision reproduction and complete final HyperDRC qualification. The KiCad adapter returns typed source, unsupported-geometry, native-geometry, or budget failures through the existing loader Result. It publishes no partial board. Exact zero width remains empty after all required source fields validate.

The original HVP109A workload exposed five positive-width tracks with exactly coincident endpoints. They now reuse the native circle constructor. Independent Python Fraction extraction and 15 strict→approximate→strict checks establish their exact centers, radii, areas, point membership and native bounds. Previously materialized feature metadata and finite report coordinate bits remain identical across all measured inputs. This is a deliberate geometry correction and version-21 evidence migration, not a performance-only patch.

Final validation: 39 new public tests; 1,653 tests in the complete native run; all three original full-board, copper-neck and Gerber gates separately pass; warning-denied all-target Clippy, docs and formatting pass. The 39 focused tests overlap the full run. All 96 malformed-source observations now fail typed; 32 actual CLI cases reject input even with --allow-findings and emit no success report.

The unchanged original 26 stroke regressions move from 4 passed/22 failed to 5 passed/21 failed. Only the coincident-track disk test is newly completed. Failed-region versus load-error expectations are explicitly different contracts; remaining failures stay archived.

Five alternating pairs retain both load phases, all allocation requests, retained/peak requested bytes, process resources, output snapshots and raw CLI reports. Only unchanged-output fixtures support matched comparisons. HVP109A includes five new disks; its ratios are descriptive changed-workload observations. No speedup is claimed.

| Input | Phase | Matched contract | Median candidate/baseline time | Allocation-call deltas (all pairs) |
| --- | --- | --- | ---: | --- |
| ordinary64 | fresh_process_load | True | 1.0417 | 0, 0, 0, 0, 0 |
| ordinary64 | repeat_after_projection_and_drop | True | 1.0534 | 0, 0, 0, 0, 0 |
| cparti | fresh_process_load | True | 0.9906 | 9528, 9528, 9528, 9528, 9528 |
| cparti | repeat_after_projection_and_drop | True | 0.9986 | 9528, 9528, 9528, 9528, 9528 |
| hvp109a | fresh_process_load | False | 0.9991 | 11776, 11776, 11777, 11779, 11777 |
| hvp109a | repeat_after_projection_and_drop | False | 1.0063 | 11777, 11778, 11778, 11778, 11776 |

Observed CLI size delta: +124640 bytes; allocator-probe delta: +45328 bytes. Every adverse row is retained in decision-v1.json and comparison-v1/result.json.

Retain for explicit malformed-source failure and exact coincident-disk completion, above performance, memory and size in the governing ladder. The unchanged ordinary64 instrumented samples show slower median loading; the unchanged CPArti samples require more allocation calls and bytes. All adverse observations and artifact growth are retained. Previously materialized output is preserved and both original full-board resource gates pass. The additional HVP109A geometry changes the completed workload. No speedup or lower-resource claim is made.

The first candidate rejected the HVP109A disks and failed the original workload gates. The second introduced the disk construction but missed private-test imports. Both full attempts remain frozen. The final target reruns all affected checks; no completed Cargo target was reopened.

Limits and remaining work:

- Noncoincident tracks still use the legacy finite rectangular adapter. General round caps, exact source-coordinate fidelity and extreme noncoincident strokes remain open; local predicate certainty does not certify that source fidelity.
- The new typed failure boundary covers straight-track start/end/width and their construction. Other object families, layer/net validation, the full error taxonomy, end-to-end provenance, and structured machine-readable CLI errors remain open.
- Input parsing, exact coefficient growth, total work, heap bytes and cancellation have no new internal bound. Shared work-budget checks preserve existing denial, but do not bound parsing. Runs use external time/address-space limits; allocator requests and process RSS do not measure expression depth or prove a heap-byte bound.
- HVP109A loads now include five additional disks. Their costs are retained as changed-workload observations, not a same-contract speed comparison. Instrumented matched samples for unchanged inputs do not establish uninstrumented production latency or a universal performance claim.
- The archived native-stroke tests stop at their first failed assertion; no credit is claimed for later unexecuted checks. Their remaining failures and the rejected 35-file integration remain preserved.
- The published-package user path, minimum compiler, complete pre-push/CI matrix, all-property audit and measured coverage remain incomplete. No push was performed; KiCad loading has no affected C/JS/WASM binding consumer in this patch.
- The inherited physical artifact audit remains failed/incomplete with 27093 missing paths. Fresh paths are audited independently without a whole-history or zero-loss claim.
- prepare_candidate_v2.py printed only its two test-path variables in the short terminal summary; its result.json correctly records all five changed paths and the complete source hashes. The inaccurate short summary is preserved, not used as a source inventory.

- Continue ordinary strict native-stroke completion and exact source fidelity using the preserved original regressions and unchanged workload gates; do not restore the rejected integration wholesale.
- Extend visible failures and provenance through the remaining modeling/format boundaries, then complete clean external distribution, release/workflow validation and the remaining properties.
