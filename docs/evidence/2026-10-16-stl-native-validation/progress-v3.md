# STL import admission

Keep the narrow STL correction and its example feature declaration. The full properties goal remains active. CSGRS now promotes parsed finite binary32 coordinates exactly, then uses Hypermesh's strict closed-PWN validation. This removes the absolute floating area cutoff and the edge map that lost multiplicity, retains native validation facts for later operations, and preserves native errors through `IoError::Construction`. Canonical empty remains successful. The proof concerns the parsed mesh, not an original analytic surface, manifoldness in general, or preservation of earlier construction intent.

The baseline is commit `7a1f0c7e1f4f69499feb3b471f739dfb50f4776c` plus the unchanged 35 pending native files. Read-only qualified libraries reproduced the original format probe (7 passed, 5 failed), original topology probe (2 passed, 2 failed), and new acceptance tests (3 passed, 5 failed). The final implementation passes all eight new tests and all four original topology tests. The original format probe now has 9 passes and the same three separately identified export failures: tiny-coordinate text loss in OBJ and ASCII STL, and binary STL quantization collapsing faces. The original probes remain unchanged.

The new tests use independently specified binary32 faces, exact coordinates, box boundaries, winding integrals, topology and subsequent composition. They cover both encodings, the smallest subnormal through maximum finite scale, empty and malformed input, typed native causes, disconnected/nested/inverted/multiply wound surfaces, signed zero, strict→approximate→strict proof reuse, and mutation after validation. Successful empty results enter later union operations; malformed meshes fail before those operations.

Candidate v1 passed all-feature tests, Clippy and docs but exposed an existing example feature mismatch in the STL-only configurations. `basic_shapes` calls the curve-gated torus constructor while declaring only `stl-io`. The unchanged baseline independently reproduced E0425. Candidate v2 adds the missing `curve` requirement; the STL Rust source and all eight regressions are byte-identical to v1. Both physical targets and all failed logs remain preserved. The default documented example command already enables both features.

Final validation used Rust 1.98.1, Cargo offline with the existing lockfile, two build jobs, and a new empty physical ext4 target. `candidate-validation-v2/commands.json` records all 13 commands and their raw output. Every command met its expected exit; the original format diagnostic intentionally exits 101 for the three remaining export defects.

| Check | Final result |
| --- | --- |
| Workspace formatting | Passed in snapshot and applied main worktree |
| Minimal OBJ/STL regression build | 8 passed |
| Root all features | 420 passed, 1 pre-existing ignore |
| Root STL only | 144 passed, no ignores |
| Original topology probe | 4 passed |
| All-feature/all-target Clippy, `-D warnings` | Passed |
| STL-only/all-target Clippy, `-D warnings` | Passed |
| `basic_shapes`, minimal `curve,stl-io` execution and Clippy | Passed |
| All-feature rustdoc, `-D warnings` | Passed |

The full-scale sphere and text-to-STL consumers remain in the all-feature suite. The unchanged example generated cube, sphere, cylinder, ellipsoid and torus at its original resolutions. Cargo's backtick-delimited run log was outside the initial executable-retention parser, so `example_replay_v1.py` performs a fresh replay from a hashed, retained executable. All five complete STL outputs match the earlier run byte for byte; the replay does not claim that the hash was recorded at the earlier execution. No completed target was reopened. There is no inspected C/WASM STL import wrapper; WASM export entry points are unchanged.

The first cost attempt (`cost-v2`) stopped on a faulty input-row-order oracle after union with empty. Public Boolean operations regularize their operands and may retriangulate or reorder them. The corrected version independently verifies integer box corners, outward boundary faces, manifoldness and volume one for every component, while separately retaining complete byte-for-byte baseline/candidate output comparisons. The production implementation and acceptance tests did not change. The failed cost attempt remains unqualified and preserved.

The completed comparison (`cost-v3`) has five alternating pairs for each encoding and each of empty, unit cube and 256 disconnected cubes: 60 fresh processes and 120 first/repeated-call rows. Each measured operation imports, validates nonempty topology strictly, unions with empty and retains its complete native result. Inputs and comparison-file writes are outside timing. Every complete input/output comparison agrees. The allocator is instrumented; these are focused measurements, not an optimization or statistical performance claim.

| Encoding / cubes | Repeated pipeline median paired ratio, candidate/baseline | Slower pairs |
| --- | ---: | ---: |
| Binary / 0 | 0.908753 | 0/5 |
| Binary / 1 | 0.843751 | 0/5 |
| Binary / 256 | 0.932009 | 1/5 |
| ASCII / 0 | 1.063479 | 3/5 |
| ASCII / 1 | 0.899009 | 1/5 |
| ASCII / 256 | 0.933458 | 0/5 |

Unfavorable results remain in the record: first empty-pipeline ratios are 1.030826 binary and 1.097213 ASCII; first unit import-only ratios are 1.061156 and 1.076451. The sub-microsecond empty timings do not establish a general speed change. Nonempty repeated pipelines make three fewer observed allocation/reallocation requests and request 1,196 fewer bytes per operation. Their retained requested-byte deltas and phase peak requested-byte measurements agree between variants. These counters exclude allocator metadata and non-Rust heap use; raw GNU-time process RSS is retained separately. The instrumented binaries are 9,766,544 baseline and 9,747,880 candidate bytes as built, with source-path differences recorded in their provenance.

`apply_v3.py` installed only the exact tested `Cargo.toml`, `src/io/stl.rs` and `tests/stl_import_contracts.rs` bytes and confirmed all other captured build inputs against the snapshot. Main-worktree formatting passed. `decision-v3.json` records this narrow admission. The checkpoint changes only the raw `current_work` ledger value and verifies every byte outside it, plus every other decoded key, against `9025de62`.

No public total-work, heap-byte or cancellation budget is added. Tests/builds use external elapsed/address-space limits; parser allocations remain internally unbounded. The 35 native files remain unadmitted: the earlier original full-board and copper-neck runtime gates still fail. The inherited artifact audit remains failed/incomplete with 27,093 missing historical paths; fresh physical hashes do not establish whole-history preservation. Clean external-package/release validation, the complete pre-push matrix, the three export defects and the other properties remain open. No push was performed.
