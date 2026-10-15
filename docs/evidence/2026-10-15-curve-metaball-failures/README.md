# Planar metaballs and exact profile composition

This is local acceptance evidence under `properties.md` and `docs/admission.md`,
not a new policy or a claim that the full properties plan is complete.
Implementation commits are `77053db06` (predicate/filter composition) and
`4af09da67` (planar modeling and bindings). The base was `f82329124` plus the
original 35 pending native files. Those files remain byte-for-byte unchanged
and require their separate admission. These are worktree validations, not a
clean-checkout, published-package, release, or MSRV qualification.

## Contract and owning layers

The relevant properties are P1, P3–P4, P6, P8–P9, P13, P17–P20, P25–P30,
P35–P37 and P39. They identify requirements; this record does not mark them
fully satisfied.

Hypersdf owns exact inverse-square influence and conservative bounds. CSGRS
now samples that same native carrier for planar metaballs, with positive radii
and iso, nonnegative padding, and dimensions of at least two, even for empty
input. `curve::metaballs` returns `Result`; `metaballs_with_context` preserves
the certainty actually consumed. Invalid input, undecided construction and
checked capacity/allocation failures no longer become empty geometry.

This is an explicit API and geometry migration. Sampling uses
`F=(iso-I)/(iso+I)`, where `I=sum(r²/distance²)`, continuously extended to -1
at centers. The cutoff and finite pole sentinel are removed. The intended zero
set is unchanged, but interpolated coordinates and serialized artifacts change.
The existing 2D core grid is retained; it does not gain the 3D sampler's border.

Hypercurve owns exact edge crossings and the polygonal marching-squares
proposal. Ambiguous cells retain the center-sample-sum sign rule, with zero
inside. Existing native arrangement supplies material, holes and disconnected
components. Boundary samples must be exterior, and unsupported arrangements
remain errors. These chords are neither the exact bilinear level curve nor a
certified approximation of the continuous field. Numerical certainty, region
topology and analytic fidelity are separate claims.

The next supported operations exposed a second failure: extrusion quantized
the exact polygon through `f64`. Hypercurve now exposes certified native line
profiles, reusing its existing line-image and retained-endpoint authority,
including authored lines stored as Bézier curves. Hypertri triangulates those
contours without coordinate projection. CSGRS triangulation and linear/vector
extrusion consume that shared path and preserve exact vertices. This also
changes formerly quantized output artifacts. Other profile representations
retain the existing finite sampled path; its fidelity/provenance limitations
remain open.

The unchanged strict gear catalog then exposed two lower-layer blockers.
Hyperlimit now accepts any certified unequal coordinate as a complete point
inequality witness; equality still needs every coordinate, and lexicographic
ordering is unchanged. Hypertri's triangle AABB is now a conservative rejection
filter: Unknown proceeds to the final triangle predicate, without consuming an
approximate filter. Both changes retain strict completion of the original gear
workload.

Extrusion adapters retain native failure categories and sources. WASM
extrusion and triangulation expose native `GeometryError` categories, including
invalid non-finite extrusion heights through the existing shared ingress helper.

## Semantic evidence

- The normalized unit-source 3×3 proposal is a diamond of independently derived
  area `2*(20201/20200)^2`. Its extrusion has independently checked exact volume
  and closed manifold topology. Empty accumulation and subtraction-to-empty
  outputs remain consumable.
- Four sources around a ring at the original 25×25 resolution preserve an
  excluded center and material bridges. Native grids separately exercise
  holes, disconnected components, zero vertices, all diagonal pairings,
  malformed axes/dimensions and open sample boundaries.
- A rational profile with a hole has exact area `23/3` and height-two volume
  `46/3`, also after translation by `10^400`. Vertex membership, signed area,
  signed volume and closed topology are tested separately.
- Strict→approximate→strict sequences retain undecided evidence, while merely
  selecting approximate policy for certified inputs still returns Certified.
  The point-equality regression independently proves inequality using another
  coordinate, while its ordering control remains Unknown.
- The original README 24×24 and benchmark 48×48 planar inputs complete under
  strict policy at their original resolutions. The original strict solid
  catalog, including involute and cycloidal gear extrusion, remains enabled.
- Real C, C++, Python and Go consumers retain integer width `2^53+1` through
  extrusion and bounds, reject zero extrusion and recover afterward. WASM
  translates a unit square by `2^53`, extrudes it, translates back and checks
  width one. It also checks typed invalid inputs and certainty. Finite display
  arrays remain quantized; their local predicate receipt is not a claim of
  topology or fidelity after quantization.

## Trials, including failures

Every executed runner and trial remains immutable. Command lists, source
patches/archives, hashes, raw stdout/stderr and terminal results are retained.

| Trial | Result |
| --- | --- |
| baseline-v1 | All nine planar failure/geometry regressions failed on the target worktree. |
| candidate-v1, candidate-v2 | Seed copying hit storage quota before compilation. The first full copy stderr was not retained; its truncated-tool-output limitation is recorded. The second preparation preserves full stderr. |
| candidate-v3 | Compile failure: `Problem` does not implement `Eq`. |
| candidate-v4 | Ten planar tests passed; exact extrusion volume failed through the finite projection. |
| candidate-v5 | New test compilation exposed incorrect test API calls. |
| candidate-v6 | Native grids, planar tests and full-size workloads passed; two authored-line exact-profile tests failed because lines were stored as Bézier curves. |
| candidate-v7 | All 22 focused cases passed after reusing native line-image authority. |
| wrappers-baseline-v1 | Reproduced loss of a native undecided error into an adapter string. |
| native-v1 | Formatting and Clippy passed; the original strict gear catalog failed at point equality. |
| predicate-baseline-v1 | Reproduced an unknown X comparison hiding a definite unequal Y coordinate. |
| candidate-v8 | Predicate regressions passed; the gear reached an undecided triangle AABB filter. |
| candidate-v9 | Predicate/filter regressions, the unchanged strict gear catalog and exact-profile/adapter tests passed. |
| native-v2 | All 22 native qualification commands passed. |
| bindings-v1 | 36 of 37 commands passed. All WASM semantic assertions passed, but the new test freed the same point twice during cleanup; TypeScript had not run. The failure and generated module remain preserved. |
| bindings-v2 | Corrected only the test's duplicate ownership registration. A fresh copy of the unchanged module passed Node, cleanup and TypeScript. |

Only the WASM ingress helper/call and its JS regression changed after
`native-v2`; final host all-target/all-feature Clippy, WASM-target Clippy and
real binding consumers cover those edits. Only the JS ownership bookkeeping
changed between the two binding phases. The closure checks these source deltas.

## Passing validation coverage

| Suite | Passed | Existing ignored |
| --- | ---: | ---: |
| CSGRS all-feature library/integration | 238 | 1 |
| Hypercurve all-feature library/integration | 2,493 | 33 |
| Hyperlimit all-feature library/integration | 365 | 0 |
| Hypertri all-feature library/integration | 188 | 0 |
| Hypermesh all-feature library/integration | 236 | 7 |
| Hypersdf all-feature library/integration | 115 | 0 |
| FFI | 10 | 0 |
| Minimal CSGRS | 38 | 0 |
| Curve + metaballs focused cases/workloads | 16 | 0 |
| Metaballs-only | 15 | 0 |
| Default-feature native marching squares | 6 | 0 |
| Combined doctests | 6 | 0 |

Feature-configuration counts overlap. No ignore marker was added. Formatting,
strict Clippy for the declared native/feature/fuzz/WASM configurations,
warning-free docs, C/C++ compilation, five consumer fixtures per C/C++/Python,
two Go tests, generated Node.js assertions and TypeScript checking passed.
Exact commands and tool versions are in the protocols and raw logs.

## Resources and remaining limits

Commands run serially with two Cargo jobs, offline dependency resolution,
1800-second external wall containment, and an 8-GiB per-process address-space
limit. Node/WASM and the final TypeScript command explicitly omit the address
limit because of V8 virtual reservations; wall containment remains. GNU time
records per-command timing and maximum resident memory for later phases,
including compilation. Those are not per-operation allocation measurements or
matched benchmarks. No performance improvement is claimed.

Primary grid capacities/reservations are checked. Scalar expression depth,
coefficient growth, secondary native allocations, total work, heap-byte limits
and cancellation remain internally unbounded or unmeasured. Cleanup manifests
map deleted non-executable duplicate compiler files to preserved identical
originals. Executed artifacts, unique compiler output, logs and source snapshots
remain. The third duplicate cleanup ran during doctests but touched only named
closed targets, never the active target.

The full CI/pre-push matrix, release/MSRV/clean external installation, image and
export regeneration, directed fuzz/sanitizer campaigns, analytic fidelity
bounds, end-to-end provenance and the original scalar/native admission gaps
remain open. No push occurred. The overall properties goal remains active.
