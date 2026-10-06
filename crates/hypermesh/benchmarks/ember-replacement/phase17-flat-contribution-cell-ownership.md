# Phase 17/18 checkpoint: flat contribution-cell ownership

Date: 2026-08-05

Implementation parent: Hypermesh `1b253caf`

Implementation commit: Hypermesh `a59bc843`

Companion data: `phase17-flat-contribution-cell-ownership.toml`

Status: correctness, runtime, heap, and size checkpoint; Phases 17 and 18 remain open

## Outcome

The contribution-complete surface arrangement now keeps the disjoint-set
result's flat `Vec<u32>` as the sole owner of authored sheet side-cell IDs.
The former temporary `SurfaceSheet { vertices, cells, bundle }` vector copied
the same cells, repeated each bundle's triangle and bundle ID for every authored
contribution, and survived until global winding classification finished. It is
gone.

The existing monotone contribution-offset table now maps a sheet back to its
geometric bundle when a component seed needs that relation. Bundle-ordered
output recovery slices the flat side-cell owner directly and reuses the
triangle already owned by `PendingFacet`. A geometric bundle with one authored
sheet returns its two outer cells directly; multi-sheet bundles retain the
complete directed-balance algorithm and every malformed-stack check.

The aggregate winding transition is also the sheet transition whenever every
geometric bundle has exactly one contribution. A second transition owner is
allocated only when coincident contributions actually require independent
authored-sheet topology. Seed classification borrows its already checked
contribution row, triangle, and side cells across direction attempts rather
than rediscovering them.

These are representation and ownership changes to the one complete algorithm.
They do not collapse coincident sheets, bypass exact radial ordering, recognize
a fixture or competitor, or alter the Boolean expression being materialized.
Compact radial flags and alternate multiplicity schedules were measured during
the checkpoint and fully removed because their broad controls regressed.

## Exactness and policy

- `hyperreal::Real` remains the construction scalar.
- `STRICT` has no approximate terminal.
- `APPROXIMATE_512` can terminate only through Hyperlimit's 512-bit terminal.
- The operation-local decision context still aggregates terminal use into one
  `MeshCertainty` value.
- All large rows below are byte-identical across the two policies and remain
  `Certified`; no approximate terminal was consumed.
- The contribution-level radial stack, retained component-balance rule, exact
  decline paths, overflow checks, and typed malformed-topology failures are
  unchanged.

## Validation and sanitizer

The final matrix passes:

- `cargo test --all-features`: 214 executed tests, seven documented manual or
  opt-in ignores, no failures;
- the public dense-crossing arrangement under both policies;
- all 17 corpus/manifest tests over the monotone 59-case registry;
- `cargo check --no-default-features`;
- `cargo clippy --all-targets --all-features -- -D warnings`;
- `cargo doc --all-features --no-deps`;
- `cargo fmt --all -- --check` and `git diff --check`;
- AddressSanitizer/libFuzzer `boolean_pipeline`: 2,082 seed files discovered,
  3,711 executions in 31 seconds, 490 MiB maximum sanitizer RSS, no crash.

LeakSanitizer alone remained disabled with `ASAN_OPTIONS=detect_leaks=0`
because the managed environment prevents its final ptrace scan. AddressSanitizer
and libFuzzer remained enabled. Generated fuzz corpus and artifacts are not
part of this checkpoint.

## Deterministic retained-work controls

The table reports five-run means of retired user instructions and branches for
1,000 strict arrangements that materialize union, intersection, difference,
and reverse difference together. The parent and current binaries were run
adjacently on pinned CPU 11. Counters varied by at most 0.04% within a row.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 2,878,901,557 | 2,856,327,195 | -0.784% | 487,625,739 | 483,882,610 | -0.768% |
| overlapping boxes | 3,650,113,562 | 3,627,124,212 | -0.630% | 619,748,164 | 616,389,691 | -0.542% |
| affine boxes | 6,981,825,352 | 6,970,739,480 | -0.159% | 1,187,730,893 | 1,187,361,607 | -0.031% |
| identical boxes | 3,767,044,285 | 3,764,607,382 | -0.065% | 639,690,479 | 639,555,510 | -0.021% |

Against the earlier `d17f1ec2` control, the complete current engine is now
2.348%/4.846%/3.671% lower in instructions on crossing, overlapping, and
identical geometry. Affine remains 1.066% higher in instructions and 0.619%
higher in branches. Profiling attributes that remaining difference to the
additional exact radial rings needed to retain coincident authored sheets; it
is an open general optimization target, not a candidate for topology collapse.

The frequency-sensitive five-run wall medians were 289,883 ns crossing,
368,025 ns overlapping, 718,268 ns affine, and 369,320 ns identical. Using the
established exact-input CGAL 6.0.3 EPECK medians gives open runtime ratios of
2.463x, 2.829x, and 1.994x for crossing, overlapping, and affine respectively.
The retired counters, rather than those frequency-sensitive clocks, are the
checkpoint regression authority.

## Full-resolution historical hard case

The checksum-pinned 11,894-by-11,894-triangle rotated YeahRight intersection
still produces the exact empty result with `Certified` certainty under both
policies. Fresh direct samples took 984.905 ms strict and 984.765 ms
approximate-512 at 70,716/70,660 KiB maximum process RSS.

The strict three-run counter mean is 10,342,992,427 instructions,
1,762,017,725 branches, and 24,135,762 branch misses. Relative to the immediate
parent, instructions/branches fall 0.124%/0.123%; relative to `d17f1ec2`, they
fall 3.780%/3.585%.

Historical EMBER required 3,312.66 seconds and 329,352 KiB. The current
approximate-512 direct sample is about 3,364x faster and lowers maximum RSS
78.55%. Historical CGAL EPECK required 0.09 seconds and 15,516 KiB, leaving an
open 10.94x runtime and 4.55x process-RSS deficit.

## Large-fixture heap

These are global-allocator requested-payload measurements. `Incremental peak`
subtracts retained decoded input. Every strict/approximate-512 pair is
identical and certified.

| Fixture | Input triangles | Parent incremental peak | Current incremental peak | Current allocations | Added-byte change |
| --- | ---: | ---: | ---: | ---: | ---: |
| general subdivided boxes | 6,144 | 12,132,042 B | 12,132,042 B | 35,729 (-3) | -197,664 B (-1.087%) |
| dense coplanar boxes 32 | 24,576 | 55,759,202 B | 55,759,202 B | 926,776 (-1) | -1,179,648 B (-0.532%) |
| transverse self-PWN clusters 512 | 4,100 | 11,451,636 B | 11,451,636 B | 197,040 (-3) | -327,840 B (-1.049%) |
| full rotated YeahRight | 23,788 | 54,471,340 B | 53,217,804 B | 10,804,019 (-3) | -1,253,536 B (-0.205%) |

The ordinary large-fixture peaks remain governed by other simultaneous owners,
but cumulative traffic falls on every row. Full YeahRight did retain the
temporary contribution topology at its peak, so eliminating that duplicate
owner removes 1,253,536 bytes, or 2.301%, from its incremental kernel peak.

## Source and linked size

The explicit checked flat-slice plumbing adds 51 Tokei production code lines
relative to the immediate parent, from 20,300 to 20,351. Despite that source
movement, every canonical linked row shrinks:

| Consumer/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release/general | 2,025,170 | 2,024,242 | -928 | 1,442,248 | 1,441,682 | -566 |
| all-feature release/general | 2,160,379 | 2,159,459 | -920 | 1,517,400 | 1,516,808 | -592 |
| default size/general | 1,116,183 | 1,115,895 | -288 | 700,085 | 699,921 | -164 |
| all-feature size/general | 1,118,247 | 1,117,975 | -272 | 700,018 | 699,916 | -102 |

Relative to `d17f1ec2`, native/optimized-WASM growth is now
0.796%/1.212%, 0.720%/1.180%, 1.447%/1.816%, and 1.420%/1.844% across those
four rows. Further source and linked recovery remains open.

## Call graph and removal audit

The workspace call-graph utility was rerun after removing the temporary clean
profiling worktree, over exactly Hyperreal, Hyperlattice, Hyperlimit, Hypertri,
and Hypermesh:

- production: 15,460 function nodes / 25,840 edges;
- tests, examples, benches, and fuzz included: 21,863 nodes / 35,352 edges;
- 49 direct static Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- exactly one `build_surface_arrangement -> assemble_surface_cells` route;
- zero `SurfaceSheet`, EMBER, `segment_trace`, or `local_bsp` nodes.

Static graph resolution remains navigation and removal evidence, not a
replacement for the passing runtime, policy, sanitizer, and corpus gates.

## Open work

This checkpoint does not declare Phase 17 or Phase 18 complete. Every CGAL
runtime gap, full-case RSS gap, affine constant factor, ordinary large-fixture
peak regression against `d17f1ec2`, retained-query allocation traffic,
source/native/WASM growth, broader real-world/generated pathology admission,
fuzz-mutation-source audit, and the final requirement-by-requirement release
audit remain explicit.

Any later optimization must preserve contribution-level topology, the retained
component-balance invariant, the one exact decline path, both Hyperlimit policy
semantics, and the prohibition on fixture- or competitor-aware dispatch.
