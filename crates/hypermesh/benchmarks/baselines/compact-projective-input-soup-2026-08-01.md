# Compact projective input soup — 2026-08-01

This is Phase 7 checkpoint 25 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hypermesh
`d7dd9e492da6a4eeb8725d478cc7e7bf7e743a4e`, based on checkpoint 24
`c45cc4a105fe87388eca379f177b15d1e2313072`. The scalar base remains
Hyperreal `a90fd36aca8df4aab4661c068f2b29961d657da2`.

## Outcome

Certified two-convex Boolean inputs no longer materialize one complete
`ConvexPolygon` (336 bytes on this x86-64 host, plus owned fields) for every
source triangle before merging coplanar triangles into projective faces. A
32-byte triangle descriptor now retains three source indices and one
mesh-local support-plane index; positions are shared once per operand, support
planes are reused during admission, and full polygons are built only for the
collapsed faces that projective clipping consumes.

On the generated 13,452-triangle fixture this removes 10.14% of retired
instructions and about 12.1% of task clock. On the dense 6,144-triangle box
fixture it removes 35.65% of instructions and at least 38.8% of task clock.
Peak heap falls 29.84% and 45.07%, respectively. The retained-arrangement
control deliberately continues through the full retained-polygon path; even
there, instructions improve 0.53% and peak heap improves 5.74% because the
shared projective core no longer carries unused normalized-plane caches.

The cost is bounded linked code growth: 0.62% release-native text, 0.90%
optimized release WASM, 0.84% size-native text, and 1.09% optimized size WASM.
Runtime has priority, and the large runtime and heap reductions retain the
change.

## Representation and execution shape

`ProjectiveInputSoup` owns exact bounds and one `ProjectiveInputMesh` per
operand. Each mesh owns or shares one `Arc<[Point3]>`, a deduplicated admission
support list, 32-byte `ProjectiveInputTriangle` rows, and one checked polygon
offset. Native `TriangleMesh` positions reuse their existing `Arc`; borrowed
views are copied once because the projective result must outlive the input
view.

Admission performs the same required validation as the full builder:

1. mesh, triangle, optional-plane, and vertex-index shapes are validated;
2. exact bounds are computed through the operation `DecisionContext`;
3. exact dyadic axis supports and exactly certified adjacent coplanar supports
   are reused when available;
4. every retained support is validated through the selected policy; and
5. degenerate triangles, invalid indices, arithmetic overflow, and
   non-retryable errors remain typed failures.

The projective preparation then deduplicates oriented support planes exactly,
partitions triangles stably by mesh and support, identifies source-boundary
edges by sorted undirected endpoint identity, traces each oriented face cycle,
preserves supplied boundary planes, and removes only exactly certified
collinear vertices. Singleton groups materialize an indexed deferred triangle;
multi-triangle groups materialize one certified face sharing the operand's
position arena. The existing clipping/classification body is factored into one
`compute_projective_convex_faces` core used by both compact and full inputs.

## Policy and path-completeness proof

No floating approximation certifies topology. Binary64 position and plane
values remain scheduling keys whose collisions are resolved exactly. Support
validity, coplanarity, collinearity, plane identity, clipping, output closure,
and nondegeneracy all use the immutable operation context.

Under `STRICT`, an unresolved predicate remains a typed indeterminate result.
Under `APPROXIMATE_512`, only Hyperlimit's terminal 512-bit interpretation can
resolve an otherwise unresolved equality/sign. The compact carrier adds no
policy, terminal, or independent equality implementation. Final YeahRight
results for all four Boolean operations are exact `BooleanMesh`-equal between
policies and report `Certified` in both modes.

The compact path is attempted only for exactly two operands whose convexity
facts have already been certified and only when retained source polygons are
not being substituted. Any retryable compact preparation, face-collapse, or
projective-candidate failure falls through to a fresh ordinary full polygon
soup, the previous full projective candidate, and then the complete general
subdivision path. A projective `None` likewise falls through. Non-retryable
input and arithmetic errors propagate unchanged. Retained-polygon operations
never enter the compact builder. This is a restructuring of owned internals,
not a compatibility shim or parallel public API.

## Exact output gates

Both policies produce the following certified immediate results:

| Fixture | Input triangles | Output vertices | Output triangles |
| --- | ---: | ---: | ---: |
| Generated projective | 13,452 | 154 | 304 |
| Retained arrangement | 4,524 | 625 | 1,246 |
| Dense subdivided boxes | 6,144 | 27 | 50 |

The opt-in YeahRight gate now runs union, intersection, difference, and
symmetric difference under both policies. It requires policy-paired exact mesh
equality, exact directed closure, exact nondegeneracy, and polygon/immediate
API agreement. Its auxiliary binary64 summary is not used as the closure
oracle for symmetric difference: that summary deliberately quantizes vertices
and can merge exact distinct crossings with the same key; Hypermesh's exact
directed closure evidence remains authoritative.

## Serialized CPU work

Checkpoint-24 and candidate repeated-operation executables were pinned to
logical CPU 9 in parent/candidate/candidate/parent order. Each process builds
the fixture once and repeats the complete immediate union. Retired
instructions are the primary deterministic gate; task clock is reported from
the same isolated brackets.

| Fixture / policy | Repetitions | Parent ms/op | Current ms/op | Task | Parent instructions | Current instructions | Instructions |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | 10.544760 | 9.282874 | -11.967% | 61,012,877,057 | 54,826,805,041 | -10.139% |
| Generated / `APPROXIMATE_512` | 501 | 10.610499 | 9.319950 | -12.163% | 61,012,870,068 | 54,826,449,857 | -10.140% |
| Boxes / `STRICT` | 10,001 | 1.393489 | 0.798115 | -42.725% | 157,412,232,409 | 101,295,542,768 | -35.650% |
| Boxes / `APPROXIMATE_512` | 10,001 | 1.488684 | 0.814832 | noisy bracket | 157,407,537,346 | 101,302,379,261 | -35.643% |
| Retained / `STRICT` | 51 | 35.157157 | 34.721863 | -1.238% | 20,906,862,803 | 20,796,862,642 | -0.526% |
| Retained / `APPROXIMATE_512` | 51 | 35.202157 | 34.695490 | -1.439% | 20,907,896,309 | 20,797,861,341 | -0.526% |

One approximate-box parent process was a clock outlier (16,157.12 ms versus
13,619.53 ms in its pair), while its instructions remained stable. Comparing
the slowest candidate to the fastest parent still gives a conservative 38.83%
clock reduction; no clock claim depends on the outlier-inflated mean.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Strict and approximate recordings have identical allocation and peak-heap
rows. `Temporary allocations` is Heaptrack's reconstructed-temporary count.

| Fixture | Parent allocations | Current allocations | Parent temporaries | Current temporaries | Parent peak | Current peak | Peak movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated projective | 200,753 | 200,756 | 10,359 | 10,359 | 10.69 MiB | 7.50 MiB | -29.84% |
| Dense boxes | 27,209 | 27,212 | 81 | 81 | 4.26 MiB | 2.34 MiB | -45.07% |
| Retained arrangement | 454,001 | 454,005 | 28,735 | 28,735 | 12.38 MiB | 11.67 MiB | -5.74% |

Allocation-call movement is effectively neutral (+3, +3, and +4). The peak
reduction comes from replacing eager, simultaneously live polygon families
with compact source rows and one collapsed face per support.

## Cycle profile

The final frame-pointer profile covers 501 generated unions on CPU 9. It has
about 6,000 samples and zero lost samples. Largest self owners are
`Rational::to_f64_lossy` 6.57%, compact input construction 5.67%, four-by-two
signed-product ordering 4.98%, six-by-two ordering 4.97%, crossing-event
splitting 3.07%, point predicate evidence 2.77%, mixed-width GCD 2.48%, the
rational filter 2.47%, allocator work 2.41%, compact projective preparation
2.10%, exact normalization 2.04%, line sign 1.97%, exact rational coordinate
construction 1.95%, exact-dyadic export 1.93%, the shared projective-face core
1.64%, and `memmove` 1.33%.

Checkpoint 24 sampled full polygon-soup construction at 7.80% and `memmove` at
4.89%. Attribution varies between profiles, but their disappearance/reduction
matches the deterministic instruction and heap changes.

## Competitive and historical controls

One CPU-9 Criterion session reports equivalent throughput workloads:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.2614–6.3339 ms (6.2863 center) | 756.44–764.16 us (759.75) | 665.75–683.70 us (674.20) |
| Dense exact-cell boxes | 830.91–846.68 us (837.17 center) | 7.1308–7.3209 ms (7.2230) | 4.3569–4.3871 ms (4.3685) |

The projective Hypermesh row improves 3.12% versus its prior Criterion sample
but remains about 8.27x slower than Boolmesh and 9.32x slower than Manifold.
On dense exact cells, Hypermesh improves 39.28% and is now about 8.63x faster
than Boolmesh and 5.22x faster than Manifold. Competitors are throughput
comparators, not exactness oracles.

The directional retained historical baseline is 944.8 ms, 67.74 MiB, and
5,020,891 allocations. Current strict direct work is 34.7219 ms, 11.67 MiB,
and 454,005 allocations: 96.32%, 82.77%, and 90.96% below those historical
values. Fixture and implementation evolution make this a trend rather than a
direct A/B.

## Linked code and call graph

The implementation changes 831 production insertions / 261 deletions and 143
test insertions / 88 deletions. It adds no public API or compatibility shim.

| Consumer | Profile / format | Parent | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,040,012 | 4,065,132 | +25,120 (+0.622%) |
| Immediate | Release native text | 4,073,628 | 4,098,796 | +25,168 (+0.618%) |
| General | Release WASM `wasm-opt -Oz` | 2,703,302 | 2,727,644 | +24,342 (+0.901%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,718,337 | 2,742,692 | +24,355 (+0.896%) |
| General | Size native text | 1,855,338 | 1,870,978 | +15,640 (+0.843%) |
| Immediate | Size native text | 1,867,846 | 1,883,454 | +15,608 (+0.836%) |
| General | Size WASM `wasm-opt -Oz` | 1,153,145 | 1,165,790 | +12,645 (+1.097%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,163,505 | 1,176,169 | +12,664 (+1.088%) |

The equal-length repeated executable grows 10,944 file bytes (0.172%), 7,744
text bytes (0.153%), and 8,176 aggregate text/data/BSS bytes (0.154%). Its BSS
falls from 1,067 to 883 bytes.

The final call-graph utility reports 8,028 nodes / 19,820 edges for Hypermesh
and 19,718 / 39,444 across Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and
Hypermesh. Relative to checkpoint 24 this is +10/+150 and +35/+156. New named
production nodes are the compact builder, compact projective preparation,
face collapse, shared projective-face core, and checked polygon-index helper;
the remaining node movement is test coverage. No policy or terminal spine was
added.

## Rejected and corrected alternatives

- An initial 40-byte triangle row stored every polygon index. Moving one
  checked polygon offset to each input mesh reduced it to 32 bytes.
- Initial boundary grouping used directed source endpoints. It saw 192 unique
  edges in a 64-triangle coplanar face and forced the complete fallback, making
  the generated case roughly 11x slower. Sorted undirected endpoint identity
  is the existing exact source-edge invariant; the corrected implementation
  leaves only the true oriented boundary cycle.
- A first compact version deduplicated and collapsed support faces, then passed
  them through the old second dedup/collapse stage. It raised retained
  instructions about 0.55% and retained peak heap to 12.45 MiB. Extracting one
  shared projective-face core removes that redundant work; the final retained
  row is -0.53% instructions and 11.67 MiB.
- Explicit `ManuallyDrop` field moves compiled to the same `memmove`; the
  clearer ordinary ownership form was restored.
- Fixture-size threshold gating was unnecessary after all three final workload
  families improved. No threshold or duplicated implementation remains.

All temporary diagnostics, repetition hooks, and losing variants were removed
before final validation.

## Validation

The final implementation passes:

- default, no-default, and all-feature tests for Hyperreal, Hyperlattice,
  Hyperlimit, Hypertri, and Hypermesh;
- 1,059/1,059 Hypermesh library tests after the final test edit;
- warning-denied all/no-default Clippy and rustdoc across all five crates;
- all Hyperreal and Hypermesh benchmark targets and every Hypermesh fuzz bin;
- the complete nightly AddressSanitizer Hypermesh library suite (1,059 tests);
- all-family dispatch tracing with zero unknown-fact, fallback, or abort events
  on the generated row;
- opt-in release competitor adapters, both-policy/every-operation YeahRight
  exact closure and nondegeneracy, policy-paired exact mesh equality,
  polygon/immediate agreement, 3,360/13,440-triangle stress, and the full
  11,894-triangle input-validation gate;
- all six final one-shot large-fixture output checks, Heaptrack recordings,
  native/WASM size consumers, the call graph, final profile, and competitive
  controls; and
- formatting and diff checks.

The approximately 56-minute full-resolution rotated Boolean was not rerun. It
enters the ordinary non-certified-input path, so this certified two-convex
carrier cannot execute for it; the full path used by that test is unchanged.
