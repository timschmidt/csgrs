# Adjacency neighbor-presence filter

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 39

Revisions:

- Hypermesh parent: `c9e967cfbeb8c91ff900c7674f83f4fe542e37c3`
- Hypermesh implementation: `8d27c24a4a68caec9b3b72e1bdc0bf6143a6de4c`
- Hyperreal: `33e7e4d620c0c95620c1045d09d0089ded105030`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

Hypermesh's projective input-soup construction now gives every adjacency head
a monotonic 16-bit neighbor-presence filter. A missing bit proves that the
requested undirected edge has never been inserted, so the lookup returns
without following the head's dependent linked list. A present bit is only a
candidate: colliding neighbors retain the complete equality scan.

The optional adjacency carrier is boxed. This keeps the disabled
axis-aligned and supplied-plane paths at one word while the enabled path owns
the filter arena. The retained layout adds no public API, dependency,
persistent cache, compatibility shim, policy branch, or fallback path. Source
changes by 15 insertions and two deletions in one file; two insertions extend
the focused unit test.

Across both Hyperlimit policies, serialized parent/candidate brackets improve
task clock by 1.35%, 1.10%, and 0.61% on generated-projective, retained, and
dense fixtures. Cycles improve 1.29%, 1.11%, and 0.65%. The filter adds simple
bit-test instructions, but removes dependent pointer scans and lowers branch
misses on the generated hot path. Performance is the retention priority.

Large-fixture Heaptrack peaks remain 7.50, 11.66, and 1.14 MiB. Direct Massif
peaks move -16, +520, and +32 bytes; these are allocator-layout movements at
otherwise unchanged useful-heap frontiers. Native loaded aggregate grows four
bytes, a stripped production probe grows 56 bytes, and canonical optimized
WASM grows 217--522 bytes.

## Exactness and complete-path invariant

For a normalized edge `(head, other)`, insertion executes
`filters[head] |= 1 << (other mod 16)` before the edge can be observed by a
later lookup. Entries are never removed and the filter is never cleared.
Therefore:

- a zero bit is an exact proof that no matching entry exists;
- a one bit makes no equality claim and retains the entire owner chain;
- modulo collisions can add work but cannot suppress a match;
- entry order, first-owner selection, stored direction, triangle identity,
  and orientation inversion are unchanged; and
- self-edges and reversed directed uses retain their previous behavior.

The focused unit test covers first-owner retention, reversal, a self-edge, a
definite absent bit, and an absent neighbor colliding with an inserted bit.
The exhaustive existing mesh suites cover the same carrier through all
projective input-soup consumers.

This optimization does not inspect or approximate `hyperreal::Real`. It is a
topological rejection before support-plane reuse, and any surviving candidate
continues through the identical exact equality and predicate paths.

## STRICT and APPROXIMATE_512 policy proof

No Hyperlimit resolver, terminal, certainty aggregation, predicate, or
`MeshContext` dispatch changes. `STRICT` still consumes only structural,
filtered, or exact decisions. `APPROXIMATE_512` still differs only if
Hyperlimit reaches its terminal 512-bit equality/sign interpretation. The
neighbor filter is identical under both policies and cannot manufacture an
equality result.

Both policies return `Certified` with identical output topology:

| Fixture | Input triangles | Output vertices | Output triangles | STRICT | APPROXIMATE_512 |
| --- | ---: | ---: | ---: | --- | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` | `Certified` |

The ignored release gates additionally compare both policies across union,
intersection, difference, and symmetric difference, verify boundaryless exact
outputs, compare polygon and triangle-immediate APIs, exercise the 13,440
triangle memory-pressure fixture, and validate the full-resolution retained
input.

## Measured opportunity and retained layout

A temporary diagnostic on the generated left input recorded 6,722 vertices,
13,440 triangles, 20,160 inserted adjacency entries, and 40,320 lookups. The
unfiltered carrier followed 68,596 linked-list entries: 36,878 on hits and
31,718 on misses, with a maximum chain length of ten.

Simulated filters retained these linked-list visits:

| Filter width | Linked-list visits | Reduction from unfiltered |
| ---: | ---: | ---: |
| 8 bits | 43,334 | 36.82% |
| 16 bits | 39,546 | 42.35% |
| 32 bits | 37,936 | 44.70% |
| 64 bits | 37,379 | 45.51% |

Sixteen bits captures most of the available rejection for 13,444 bytes on
this input. With the 72-byte boxed carrier, the enabled-path live addition is
13,516 bytes and does not own the process peak. Wider filters spend materially
more arena for diminishing scan removal.

Boxing is important to path balance. A separate unboxed `u16` filter made the
always-present optional carrier larger and moved the dense STRICT clock about
1% backward. Packing a filter into a 16-byte head avoided an allocation but
doubled the head arena and was about 1% slower in balanced controls. The boxed
separate arena produced the best measured runtime/memory combination while
keeping disabled paths at one word.

All temporary counters, repetition hooks, and experimental layouts were
removed before the production commit.

## Serialized CPU A/B

Parent and candidate use equal native release flags and the same temporary
operation-repetition hook. Runs are serialized on CPU 9 in
parent/candidate/candidate/parent brackets. Both policies produce the same
certified topology. Percentages below are candidate relative to parent;
negative is better for work/time counters.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches | Branch misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,001 | -1.691% | -1.559% | +0.4554% | -0.0035% | about -2.50% |
| Generated / `APPROXIMATE_512` | 1,001 | -1.0036% | -1.017% | +0.4579% | +0.0022% | about -2.17% |
| Retained / `STRICT` | 201 | +0.309% | +0.122% | +0.0280% | -0.0091% | noise |
| Retained / `APPROXIMATE_512` | 201 | -2.516% | -2.334% | +0.0271% | -0.0089% | noise |
| Dense boxes / `STRICT` | 10,001 | -0.771% | -0.806% | +0.1407% | -0.0123% | noise |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -0.452% | -0.488% | +0.1363% | -0.0251% | noise |

Arithmetic means across the two policies:

| Fixture | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated 13,452 | -1.347% | -1.288% | +0.4567% | approximately unchanged |
| Retained 4,524 | -1.104% | -1.106% | +0.0275% | -0.0090% |
| Dense boxes 6,144 | -0.612% | -0.647% | +0.1385% | -0.0187% |

The generated `APPROXIMATE_512` parent/candidate means were 8,564.46 /
8,478.51 ms, 36.2230 / 35.8546 billion cycles, and 100.44614 / 100.905995
billion instructions. Dense STRICT means were 6,842.495 / 6,789.765 ms and
28.9861 / 28.7526 billion cycles. Retained STRICT means were 6,862.33 /
6,883.555 ms; its small adverse clock movement is bounded by near-identical
deterministic work and the opposite-policy bracket.

## Profile movement

Equal-flag 501-operation CPU-9 profiles record 17,932 parent and 17,436
candidate cycle samples with zero loss. The projective input-soup construction
head falls from 5.94% to 5.42% sampled self. The candidate's next leading heads
are the four-by-two and six-by-two signed product sums at 4.63% each,
`Rational::to_f64_lossy` at 3.10%, allocator internals at 2.96%, word GCD at
2.82%, and crossing-event splitting at 2.54%. Sampling is explanatory; the
serialized counters above are the retention gate.

## Large-fixture heap

Production, no-hook Heaptrack recordings include fixture construction and one
complete immediate union. The retained OBJ is `yeahright_boolean_hull.obj`,
SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Parent allocations | Candidate allocations | Parent peak | Candidate peak | Parent RSS | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,641 | 200,644 | 7.50 MiB | 7.50 MiB | 17.89 MiB | 17.81 MiB |
| Generated / `APPROXIMATE_512` | — | 200,644 | — | 7.50 MiB | — | 17.77 MiB |
| Retained / `STRICT` | 453,853 | 453,856 | 11.66 MiB | 11.66 MiB | 21.10 MiB | 20.99 MiB |
| Retained / `APPROXIMATE_512` | — | 453,856 | — | 11.66 MiB | — | 20.99 MiB |
| Dense boxes / `STRICT` | 2,147 | 2,148 | 1.14 MiB | 1.14 MiB | 9.13 MiB | 9.05 MiB |
| Dense boxes / `APPROXIMATE_512` | — | 2,148 | — | 1.14 MiB | — | 9.06 MiB |

Heaptrack reports three additional allocation calls on enabled generated and
retained paths and one on the disabled dense path; the latter demonstrates
that one-call differences include process/layout noise. The actual enabled
carrier adds a box and filter arena. Rounded peaks are unchanged.

Direct byte-level Massif A/B:

| Fixture | Parent total | Candidate total | Movement | Parent useful | Candidate useful |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated | 8,245,736 B | 8,245,720 B | -16 B | 7,420,614 B | 7,420,614 B |
| Retained | 12,691,048 B | 12,691,568 B | +520 B | 11,583,215 B | 11,582,695 B |
| Dense boxes | 1,064,976 B | 1,065,008 B | +32 B | 1,063,718 B | 1,063,718 B |

The filter arena is live during input-soup construction but is released before
the retained and generated whole-operation peak. The byte-level results and
lower candidate RSS preserve checkpoint 38's memory frontier.

## Dispatch, linked size, and call graph

The generated dispatch trace is byte-for-byte unchanged in its counters:
97,321 dispatch events, 676 predicates, 1,411 linear-algebra events, 6,341
cache events, and 12,775 rational temporaries, with zero unknown facts and
zero fallback/abort events.

Equal-flag repeated-probe linked size:

| Measure | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| `.text` section | 3,817,878 B | 3,818,598 B | +720 B |
| GNU text | 4,635,789 B | 4,635,777 B | -12 B |
| GNU aggregate | 4,887,160 B | 4,887,164 B | +4 B |
| File | 5,586,624 B | 5,586,976 B | +352 B |

The no-hook production probe has the same +720-byte `.text`, -12-byte GNU
text, and +4-byte loaded aggregate movement. Its unstripped file comparison is
distorted by a 49 KiB non-loaded `.strtab` name-layout change; stripping both
artifacts reduces the comparable movement to +56 bytes.

Canonical default-feature consumer movement from checkpoint 38:

| Consumer | Native text | Native aggregate | File | Optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| General release | +456 B | -8 B | +592 B | +461 B |
| Immediate release | +472 B | -8 B | +608 B | +522 B |
| General size profile | +176 B | 0 B | +224 B | +219 B |
| Immediate size profile | +192 B | 0 B | +240 B | +217 B |

All-feature canonical consumers were also built in fresh directories. Their
native aggregate / optimized-WASM sizes are 4,461,340 / 2,813,291 bytes for
general release, 4,498,204 / 2,828,755 for immediate release, 2,114,124 /
1,197,088 for general size, and 2,126,416 / 1,207,835 for immediate size.

The Hypermesh per-library call graph moves from 8,051 nodes / 19,877 edges to
8,052 / 19,878, SHA-256
`b70b00321ab4f57c6f9f595ed605f966ba6ec0d886b102bd5769c9b2431355ac`.
The only additions are `hypermesh::mesh::Box::new` and the call from
`build_projective_input_soup`.

The five-crate graph moves from 19,774 nodes / 39,555 edges to 19,775 /
39,556, SHA-256
`47eb9c1eb36106e90366b1e0f3501eac4df0b633ba98934ae1cc4e33d80fc5a5`.
Hyperreal, Hyperlattice, Hyperlimit, and Hypertri per-library graphs are
byte-identical to checkpoint 38. No policy terminal, predicate, exact fallback,
or mesh operation edge changes.

## Competitive and historical controls

Fresh CPU-9 Criterion centers preserve the competitive shape. The first union
samples in both groups had visibly wide intervals and were replaced by clean
single-row reruns. Criterion clocks are orientation; serialized revision A/B
is the performance gate.

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.3147 ms | 759.72 us | 669.39 us | 8.31x / 9.43x slower |
| Projective / intersection | 4.5347 ms | 740.95 us | 653.51 us | 6.12x / 6.94x slower |
| Projective / difference | 4.1457 ms | 762.60 us | 665.64 us | 5.44x / 6.23x slower |
| Dense boxes / union | 729.09 us | 7.6176 ms | 4.8336 ms | 10.45x / 6.63x faster |
| Dense boxes / intersection | 514.79 us | 3.8713 ms | 3.3860 ms | 7.52x / 6.58x faster |
| Dense boxes / difference | 645.30 us | 6.5600 ms | 4.0247 ms | 10.17x / 6.24x faster |

The retained STRICT bracket averages 34.2465 ms per union. Against the
directional historical 944.8 ms row, that is 96.375% lower or 27.59x faster.
The candidate's 11.66 MiB peak, 453,856 allocations, and 20.99 MiB strict RSS
are 82.79%, 90.96%, and 74.56% below the historical 67.74 MiB, 5,020,891, and
82.5 MiB. Fixture and implementation evolution make these trend controls, not
direct revision A/B.

## Rejected implementations

- An exact-word digit-iterator proof in Hyperreal added 0.13% instructions,
  0.31% branches, and 64 bytes of text/file. It was restored.
- An unboxed separate `u16` filter improved generated time about 2.4% but made
  the disabled dense STRICT path about 1% slower by enlarging its optional
  carrier.
- An 8-bit filter saved only 6.6 KiB on the generated fixture while adding
  0.036% instructions, 0.068% branches, and 0.39% cycles versus 16 bits.
- Packing filter/orientation data into each edge was neutral-to-slower; the
  initial and peeled variants added 0.42% / 0.96% instructions.
- A 16-bit combined-head layout avoided one allocation but doubled the head
  arena, used about 40 KiB more heap, and was about 1% slower.
- A native-word combined-head filter removed 0.20% instructions versus the
  16-bit combined head but remained about 1% slower in balanced orders.
- Wider 32- and 64-bit filters rejected only another 4.1% and 4.6% of original
  linked visits while multiplying filter-arena size.

## Validation

The retained source passed:

- Hypermesh's default, minimal, and all-feature suites (1,063 / 1,063 / 1,064
  library tests plus every integration and doc test);
- Hyperreal's default, minimal, and all-feature suites (564 / 564 / 641
  library tests plus every integration and doc test);
- Hyperlattice, Hyperlimit, and Hypertri default, minimal, and all-feature full
  suites;
- warning-denied all-target Clippy, warning-denied rustdoc, and format checks
  for all five crates under minimal and all-feature configurations;
- all 1,063 Hypermesh library tests under nightly ASAN, with leak detection
  disabled for the allocator harness;
- every Hypermesh fuzz-bin check, all-feature benchmark compilation, and
  minimal/all-feature `wasm32-unknown-unknown` library checks;
- both-policy dispatch tracing and the five bounded ignored competitive
  release gates described above; and
- focused collision/first-owner coverage and diff whitespace checks.

The known approximately 56-minute
`full_resolution_yeahright_rotated_intersection_certifies_empty` control was
not rerun: the patch does not alter its operation, predicate, policy, or
fallback graph, and the bounded full-resolution validation gate passed.

Representative commands:

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  yeahright_exact_hypermesh_outputs_remain_boundaryless_for_every_operation \
  -- --ignored --exact --test-threads=1
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace \
  --features dispatch-trace
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-adjacency-filter-size-default \
  ./benchmarks/size-harness/measure.sh default
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-adjacency-filter-size-all \
  ./benchmarks/size-harness/measure.sh all

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json --per-library \
  --out-dir /tmp/hypermesh-adjacency-filter-callgraph
```

Hyperlimit's pre-existing untracked `hyperlimit` executable is untouched.
Machine-readable raw and derived values are in
`adjacency-neighbor-presence-filter-2026-08-02.toml`.
