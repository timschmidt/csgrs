# Inline-first exact-verified support-plane index

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 43

Revisions:

- Hypermesh parent/evidence baseline: `35c4ebabef93c2c81149c785dcfae3d5a7309353`
- Hypermesh implementation: `72b037f4`
- Hyperreal: `6302bbd848ad99cf192419c76e399f6e45cbdba3`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

The two projective-input builders no longer store a separately allocated
`Vec<usize>` in every finite-float support-plane hash bucket. One private
`ApproximateSupportPlaneIndex` now owns:

- a two-word inline bucket containing the first support and a collision head;
- one shared two-word collision arena per input mesh; and
- one `HashMap::entry` probe that either returns an exact match or installs the
  new support.

The common unique-key case allocates no child vector and hashes the key once.
True binary64-key collisions remain complete: the first support and every
collision node are checked by the same exact `Plane` equality used by the
parent. No public API, dependency, feature, compatibility shim, retained
policy state, or alternate implementation was added.

Across `STRICT` and `APPROXIMATE_512`, generated/retained/dense policy-mean
instructions fall 0.144%/0.138%/0.216% and branches fall
0.156%/0.166%/0.160%. Clock is order-sensitive at this scale; policy-mean
task/cycle movement is +0.757%/+0.263%, +0.569%/+0.874%, and
-0.110%/+0.058%, all inside the retention gate. Performance remains the
primary criterion, with deterministic work providing the stable direction.

Production allocation calls fall by 215, 1,133, and 11 on the three large
fixtures. The retained-arrangement Heaptrack peak falls from 11.66 to 11.60
MiB; generated and dense peaks stay in their 7.50 and 1.14 MiB classes. The
production ELF `.text` section is 800 bytes smaller, canonical release native
`.text` is flat-to-smaller, and optimized release WASM is about 3.9 KiB
smaller.

## Exactness and complete-path invariant

The binary64 tuple is only a candidate-bucket key. It does not certify plane
equality or inequality. For an occupied key, `intern`:

1. compares the inline first support by exact `Plane` equality;
2. walks every shared-arena collision and performs the same exact equality;
3. returns an identity only after such a comparison succeeds; and
4. otherwise appends the new exact support and collision node.

A vacant floating bucket records a new support identity; a miss is never used
as proof that two geometric planes differ. The surrounding parent paths are
unchanged:

- exact component-storage identities are still checked first;
- unavailable/non-finite filter coefficients still enter the complete
  nonexact/general scan;
- support insertion order and per-polygon finite coefficient storage are
  unchanged; and
- later policy-aware geometric support canonicalization remains in place.

Both retained node types are asserted to occupy exactly two machine words.
`usize::MAX` is the absent-link sentinel. Because the collision element is a
non-zero-sized two-word value, a `Vec` cannot reach that index within Rust's
allocation bound, so the sentinel cannot alias a valid collision.

The focused regression constructs three distinct exact planes whose first
coefficients are `1`, `1 + 2^-54`, and `1 + 2^-55`. All round to the same
binary64 key. It proves that the unique-key insertion has zero collision-arena
capacity, duplicates reuse the correct exact identity, every collision-chain
member remains reachable, and only three supports are stored.

## STRICT and APPROXIMATE_512

The new index does not inspect or choose a policy. It consumes the existing
certified finite-float proposal from `exact_plane_f64` and preserves the exact
verification that follows it. `STRICT` therefore still accepts only
structural, certified-filter, or exact decisions. `APPROXIMATE_512` can still
consume an approximate decision only at Hyperlimit's terminal 512-bit
equality/sign interpretation.

Every large run completed as `Certified` with identical topology:

| Fixture | Input triangles | Output vertices | Output triangles | STRICT | APPROXIMATE_512 |
| --- | ---: | ---: | ---: | --- | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` | `Certified` |

The generated dispatch trace remains 97,321 events: 676 predicate events,
1,411 linear-algebra events, 6,341 cache events, 12,775 rational temporaries,
zero unknown facts, and zero fallback/abort events. This is byte-for-byte the
checkpoint-40 event summary.

## Serialized CPU A/B

Parent and candidate use `-C target-cpu=native -C codegen-units=1`, the same
temporary operation-repetition hook, CPU 11, and identical fixture setup.
Hooks were removed before the implementation commit. Percentages are
candidate relative to parent; negative is better.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,001 | +1.040% | +0.660% | -0.1475% | -0.1593% |
| Generated / `APPROXIMATE_512` | 1,001 | +0.474% | -0.134% | -0.1411% | -0.1521% |
| Retained / `STRICT` | 101 | +0.598% | +1.032% | -0.1401% | -0.1681% |
| Retained / `APPROXIMATE_512` | 101 | +0.540% | +0.716% | -0.1368% | -0.1639% |
| Dense boxes / `STRICT` | 10,001 | +0.559% | +0.762% | -0.2221% | -0.1690% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -0.780% | -0.647% | -0.2109% | -0.1511% |

Arithmetic means across policies:

| Fixture | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated 13,452 | +0.757% | +0.263% | -0.144% | -0.156% |
| Retained 4,524 | +0.569% | +0.874% | -0.138% | -0.166% |
| Dense boxes 6,144 | -0.110% | +0.058% | -0.216% | -0.160% |

Generated STRICT uses balanced parent/candidate/candidate/parent and reverse
candidate/parent/parent/candidate brackets. Parent/candidate means are
8,608.583/8,698.115 ms, 35.9797/36.2172 billion cycles,
100.2846/100.1366 billion instructions, and 17.3154/17.2878 billion branches.
The opposite-order bracket alone moves only +0.350% clock/+0.117% cycles,
demonstrating the clock sensitivity; both orders retain the same instruction
and branch reductions.

## Large-fixture heap

Production no-hook Heaptrack recordings include fixture construction and one
complete union. The retained input is `yeahright_boolean_hull.obj`, SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Parent allocations | Candidate allocations | Parent peak | Candidate peak | Parent RSS | Candidate RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,643 | 200,428 | 7.50 MiB | 7.50 MiB | 17.87 MiB | 17.83 MiB |
| Generated / `APPROXIMATE_512` | 200,643 | 200,428 | 7.50 MiB | 7.50 MiB | 17.93 MiB | 17.76 MiB |
| Retained / `STRICT` | 453,855 | 452,722 | 11.66 MiB | 11.60 MiB | 21.00 MiB | 20.97 MiB |
| Retained / `APPROXIMATE_512` | 453,855 | 452,722 | 11.66 MiB | 11.60 MiB | 20.99 MiB | 20.97 MiB |
| Dense boxes / `STRICT` | 2,147 | 2,136 | 1.14 MiB | 1.14 MiB | 9.07 MiB | 9.20 MiB |
| Dense boxes / `APPROXIMATE_512` | 2,147 | 2,136 | 1.14 MiB | 1.14 MiB | 9.17 MiB | 9.11 MiB |

Heaptrack's global peak is the heap gate. Converted Massif snapshots agree on
the retained and dense direction: retained falls by 51.8--55.6 KiB and dense
by 512--544 bytes. Generated snapshot placement varies with trace phase
(STRICT falls 29,984 bytes; APPROXIMATE_512's sampled snapshot rises), while
Heaptrack's global generated peak remains 7.50 MiB under both policies. RSS is
reported but treated as process/profiler noise at these differences.

## Native and WASM size

The matched production probe moves as follows:

| Measure | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| ELF `.text` | 3,818,934 B | 3,818,134 B | -800 B |
| GNU text | 4,635,873 B | 4,635,033 B | -840 B |
| GNU aggregate | 4,887,132 B | 4,887,156 B | +24 B |
| Unstripped file | 5,587,104 B | 5,586,536 B | -568 B |
| Stripped file | 4,887,304 B | 4,886,472 B | -832 B |

Canonical consumer deltas from checkpoint 40:

| Features / consumer | ELF `.text` | GNU aggregate | File | Optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| Default general release | -80 B | -8 B | +392 B | -3,922 B |
| Default immediate release | -32 B | -8 B | +440 B | -3,927 B |
| Default general size | +608 B | +12 B | +496 B | +354 B |
| Default immediate size | +224 B | +4,104 B padding | +112 B | +341 B |
| All-feature general release | -48 B | +8 B | +408 B | -3,905 B |
| All-feature immediate release | -48 B | 0 B | +408 B | -3,860 B |
| All-feature general size | +464 B | +4 B | +368 B | +846 B |
| All-feature immediate size | +224 B | 0 B | +112 B | +342 B |

The release consumers that matter for runtime are flat-to-smaller in native
`.text` and materially smaller in optimized WASM. The isolated 4 KiB GNU
aggregate movement is section padding; the actual ELF `.bss` section remains
202 bytes in both default size-profile immediate artifacts.

## Call graph

The five-crate source graph moves from 19,778 nodes / 39,568 edges to 19,792 /
39,586, SHA-256
`a34a61ede76b7c11e7f286684ad403c50a8a9d8dd5840f67312a0c78f70d4efe`.
Hypermesh moves from 8,052 / 19,878 to 8,066 / 19,896, SHA-256
`f4442b58f89903f92a21acbb1127285cb99c1301001bf9163b4e0f7110a22c20`.

The additions are the private index/bucket methods, entry operations, and the
focused collision test. Hyperreal, Hyperlattice, Hyperlimit, and Hypertri are
byte-identical to checkpoint 40. No policy terminal, general support route,
mesh operation, or exact verification edge is removed.

## Competitive and historical controls

Fresh Criterion centers on CPU 11 orient the current implementation:

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.1516 ms | 785.88 us | 675.15 us | 7.83x / 9.11x slower |
| Projective / intersection | 4.5280 ms | 746.85 us | 665.12 us | 6.06x / 6.81x slower |
| Projective / difference | 4.2462 ms | 772.92 us | 673.86 us | 5.49x / 6.30x slower |
| Dense boxes / union | 702.70 us | 6.8482 ms | 4.5875 ms | 9.75x / 6.53x faster |
| Dense boxes / intersection | 512.08 us | 3.9220 ms | 3.4277 ms | 7.66x / 6.69x faster |
| Dense boxes / difference | 645.32 us | 6.4082 ms | 4.0334 ms | 9.93x / 6.25x faster |

Criterion classifies Hypermesh projective union as unchanged (+0.161%),
intersection as unchanged (+0.511%), and difference as within its noise
threshold (+2.341%) relative to the immediately stored session. Dense union
moves -1.794% within noise; intersection and difference move +1.513% and
+1.336% within noise. Cross-session competitor movement is larger than this
checkpoint, so the serialized same-binary counters above remain the retention
gate.

The retained union averages about 34.62 ms. Against the directional historical
944.8 ms row, it is 96.34% lower or 27.29x faster. Its 11.60 MiB peak, 452,722
allocations, and 20.97 MiB RSS are 82.88%, 90.98%, and 74.58% below the
historical 67.74 MiB, 5,020,891, and 82.5 MiB controls. Fixture and
implementation evolution make these trend controls, not revision A/B.

## Rejected implementations

- Moving support canonicalization into `ProjectiveInputMesh` with an
  insertion-time certified-float carrier increased matched generated
  instructions about 3.48% because it moved filter/cache timing into input
  construction.
- Exact-rational hashing at insertion increased instructions about 2.93%; a
  bounded exact scan increased them about 5.29%; tail preparation plus direct
  collapse increased them about 2.67%.
- A child `Vec` per floating key retained one allocation per unique support.
  `Option<usize>` collision links occupied three words rather than two.
- A separate `get` followed by `entry` preserved exactness and removed child
  allocations, but performed two map probes for every unique key and retired
  less deterministic work than the retained single-probe index.

All diagnostic support counters and operation-repetition hooks were removed.

## Verification

- default, no-default, and all-feature `cargo test --no-fail-fast` matrices;
- 1,064/1,064/1,065 library tests plus enabled integration and doc tests;
- all-target all-feature Clippy with warnings denied;
- all-feature rustdoc with warnings denied;
- complete fuzz-target build;
- 1,064-test AddressSanitizer library run;
- formatting and diff checks;
- both-policy large-fixture topology and Heaptrack;
- native/WASM size harness;
- dispatch trace, five-crate call graph, and fresh competitive controls.

The pre-existing untracked Hyperlimit `hyperlimit` executable was untouched.
