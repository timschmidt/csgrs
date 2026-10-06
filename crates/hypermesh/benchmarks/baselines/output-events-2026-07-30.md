# Hypermesh batched output-event checkpoint

Date: 2026-07-30

Direct parent: `383f01ef` (`Record compact mesh carrier benchmark`)

## Change

Output resolution now discovers every proper edge crossing in one deterministic
broad-phase traversal before applying the finite split-event batch. The prior
path returned after its first crossing, rebuilt all unique edges and exact
bounds, and repeated the complete candidate scan for every subsequent event.

Malformed Boolean meshes are rejected before mutation when triangle indices are
out of range or triangle/provenance row counts differ. Event and intersection
growth use checked capacity operations.

The retained 17-by-17 exact crossing fixture contains 289 independent proper
intersections. One discovery traversal retains all 289 events, exceeding the
old 256-pass boundary without an ordering-dependent terminal.

## Runtime

The existing release Criterion row was measured serially in the direct-parent
worktree and then in the current tree with the same target directory:

| Row | Direct parent | Current | Movement |
| --- | ---: | ---: | ---: |
| `output/cube_union_triangulate_certified` | 136.18 µs | 134.81 µs | -1.01% |

Parent confidence interval: 135.48–137.02 µs. Current confidence interval:
134.46–135.22 µs. Criterion classified the paired -0.96% estimate as within
its noise threshold. This ordinary cube row has few crossing events; it is a
regression guard rather than the asymptotic workload.

For a crossing batch of size `k`, discovery changes from up to `k` complete
edge-pair traversals to one. The event queue retains five `usize` values per
event (40 bytes on this host); intersection vertices themselves are required by
the resolved output. The 289-event regression therefore adds about 11.3 KiB of
transient queue storage while removing repeated exact-predicate scans.

## Native and WASM linked size

The compact-carrier checkpoint at the direct parent is the comparison row.
Native code is `.text`; WASM code is `wasm-opt -Oz`.

| Consumer | Profile | Target | Parent code | Current code | Movement |
| --- | --- | --- | ---: | ---: | ---: |
| General | Release | Native | 3,701,085 | 3,702,133 | +0.0283% |
| General | Release | WASM | 2,591,488 | 2,592,437 | +0.0366% |
| Immediate | Release | Native | 3,734,901 | 3,735,941 | +0.0278% |
| Immediate | Release | WASM | 2,606,677 | 2,607,622 | +0.0363% |
| General | Size | Native | 1,672,707 | 1,674,267 | +0.0933% |
| General | Size | WASM | 1,084,991 | 1,086,081 | +0.1005% |
| Immediate | Size | Native | 1,684,711 | 1,686,255 | +0.0916% |
| Immediate | Size | WASM | 1,095,173 | 1,096,268 | +0.1000% |

The correctness checkpoint remains below the one-percent phase gate on every
measured artifact. Final packaging still requires recovering all growth
against the Phase 0 authoritative baseline through later consolidation.

## Verification

- `cargo test --all-features`: 1,159 executed tests passed; 7 ignored.
- `cargo test --no-default-features`: 1,157 executed tests passed; 7 ignored.
- The 289-event exact crossing and malformed-output regressions passed under
  both feature configurations.
- `cargo check --all-targets --all-features` passed.
- all-target no-default checking, all-feature Clippy with warnings denied,
  fuzz-target checking, and rustdoc passed.
- `cargo fmt --all -- --check` and `git diff --check` passed.
- The default native/WASM release and size consumers were rebuilt and measured.

Machine-readable values are in `output-events-2026-07-30.toml`.
