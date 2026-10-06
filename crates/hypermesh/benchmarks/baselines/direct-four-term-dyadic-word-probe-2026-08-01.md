# Direct four-term dyadic word probe — 2026-08-01

This is Phase 7 checkpoint 24 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hyperreal
`a90fd36aca8df4aab4661c068f2b29961d657da2`, based on
`9b171c1231a993a65110cee06fa67ff655f7a4ed`. Measurements use Hypermesh
`0548ac30682f9be7107386c1fd6e39610b8ba1a4` as the unchanged mesh-side base.

## Outcome

Expanded four-term/two-factor affine determinants can now enter the exact
one-pass dyadic word accumulator before paying for the generic bit-width plan.
The generated 13,452-triangle projective union retires 3.791–3.794% fewer
instructions and 3.294–3.297% fewer branches under both policies. Task clock
improves 2.619% under `STRICT` and 3.245% under `APPROXIMATE_512`.

The retained 4,524-triangle arrangement is deliberately protected from its
known-unprofitable four-term probes. Instructions move +0.0029% under `STRICT`
and +0.00004% under `APPROXIMATE_512`, while task clock improves 0.212% and
0.469%. The 6,144-triangle box control is also instruction-neutral-to-better;
its policy-paired clock improves 0.216%.

All six final-source Heaptrack rows are identical to checkpoint 23. Canonical
consumer growth is 0.053–0.092%, and the equal-length repeated-operation text
grows 0.098%. Runtime has priority, so the generated-mesh reduction justifies
this bounded code-size cost.

## Exact path and policy proof

`Rational::signed_product_sum_ordering` already uses the unplanned exact word
accumulator for six-term/two-factor orientation expansions. It now admits a
four-term/two-factor expansion when the numerator of the first factor in the
fourth term converts to `u128`:

1. `FACTORS == 2` is checked before any four-term indexing.
2. `TERMS == 6` preserves the existing direct path.
3. `TERMS == 4` requires `terms[3][0].numerator.to_u128().is_some()`.
4. A successful unplanned accumulation compares exact nonnegative `u128`
   totals on a common power-of-two grid.
5. Any rejected admission or `None` from the accumulator continues to the
   unchanged complete bit-width plan, 384-bit stack accumulator, and arbitrary
   exact reducer.

The admission check cannot certify a sign or equality. If it rejects, the
unplanned accumulator would necessarily reject the same oversized numerator
when it reached that factor, so no former success is skipped. If it admits,
every factor conversion, product, alignment shift, rescale, and signed total
addition remains checked. The new 512-case differential regression exercises
both four- and six-term forms against the retained planned/two-pass alignment,
including required narrow successes and full-word overflow fallbacks.

Within the unplanned helper, exact dyadic-denominator validation now precedes
numerator conversion for each factor. Both operations remain fallible and
their conjunction is unchanged. The only observable internal difference on a
failed probe is that an exact denominator fact can be learned before a later
wide numerator rejects the word path. This ordering is cheaper for generated,
retained, and box controls.

The accumulator is exact under both policies. `STRICT` permits no terminal
approximation. `APPROXIMATE_512` still changes only Hyperlimit's terminal
interpretation after unresolved refinement reaches 512 bits. Predicate
dispatch, certainty aggregation, topology, errors, and all mesh fallback paths
are unchanged. Every measured result is `Certified` with identical vertices
and triangles under both policies.

## Admission evidence and dispatch

Temporary diagnostics, removed before validation, measured the complete
four-term opportunity set:

| Fixture | Potential 4x2 probes | Exact word successes | Would fall through | Final guard result |
| --- | ---: | ---: | ---: | --- |
| Generated projective | 13,230 | 11,557 | 1,673 | admits 12,852; rejects 378 wide fourth factors |
| Retained arrangement | 1,128 | 0 | 1,128 | rejects all 1,128 wide fourth factors |

The generated fourth admission numerator is 97–128 bits in 12,852 cases and
at least 129 bits in 378 cases. Every retained occurrence is at least 129 bits.
The final dispatch trace moves 95 decisions from the dyadic stack accumulator
to the exact word accumulator, reports zero generated unknown-fact events and
zero fallback/abort events, and leaves every output family certified.

## Serialized CPU work

Parent/candidate/candidate/parent processes were pinned to logical CPU 9. Each
process constructs its fixture once and repeats a complete immediate union.
Counters are task clock, cycles, instructions, branches, branch misses, and
cache misses. Instructions are the primary retention gate; branch-miss and
cache-miss percentages remain secondary host/layout observations.

| Fixture / policy | Repetitions | Parent ms/op | Current ms/op | Task | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | 10.913653 | 10.627804 | -2.619% | -2.544% | -3.791% | -3.294% | +0.981% | +2.987% |
| Generated / `APPROXIMATE_512` | 501 | 10.928703 | 10.574052 | -3.245% | -3.117% | -3.794% | -3.297% | +0.646% | +3.145% |
| Retained / `STRICT` | 51 | 35.372941 | 35.297941 | -0.212% | -0.088% | +0.0029% | -0.020% | -0.840% | +1.192% |
| Retained / `APPROXIMATE_512` | 51 | 35.297059 | 35.131569 | -0.469% | -0.336% | +0.00004% | -0.025% | -0.702% | +2.198% |
| Boxes / `STRICT` | 10,001 | 1.384720 | 1.375967 | -0.632% | -0.526% | -0.0131% | -0.060% | -1.690% | -7.176% |
| Boxes / `APPROXIMATE_512` | 10,001 | 1.383168 | 1.385932 | +0.200% | +0.114% | -0.0011% | -0.052% | -0.904% | -7.020% |

The approximate box row uses six processes per revision across three balanced
brackets because one earlier candidate process was an instruction outlier.
Across policies, task clock improves 2.932% generated, 0.340% retained, and
0.216% boxes. The box instruction mean improves 0.0071%.

Outputs remain 154 vertices / 304 triangles generated, 625 / 1,246 retained,
and 27 / 50 boxes for every process and policy.

## Large-fixture heap

Heaptrack records fixture construction plus one complete immediate union.
Strict and approximate recordings match each other and checkpoint 23 exactly:

| Fixture | Input triangles | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 13,452 | 200,753 | 10,359 | 10.69 MiB |
| Retained arrangement | 4,524 | 454,001 | 28,735 | 12.38 MiB |
| Subdivided boxes | 6,144 | 27,209 | 81 | 4.26 MiB |

The retained fixture SHA-256 remains
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.
The source change adds no allocation, cache, carrier, or retained mesh state.

## Cycle profile

The final frame-pointer profile covers 100 complete generated-8 unions on CPU
9 at 1,999 Hz. It contains 2,314 samples, approximately 4,784,692,370 cycle
events, and zero lost samples. Largest self owners are polygon-soup
construction 7.80%, memmove 4.89%, projective construction 4.72%, four-by-two
signed-product ordering 4.42%, six-by-two ordering 3.80%, lossy rational
conversion 3.66%, crossing-event splitting 3.10%, and mixed-width GCD 2.06%.
The remaining four-by-two generic dyadic plan is 0.66% self. Sampling
attribution moves substantially between recordings; serialized counters are
the quantitative gate.

## Source, linked code, and call graph

The production change is 9 insertions / 7 deletions; the differential test is
16 insertions / 1 deletion. There is no public API or compatibility shim.

| Consumer | Profile / format | Parent | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| General | Release native text | 4,036,956 | 4,040,012 | +3,056 (+0.0757%) |
| Immediate | Release native text | 4,070,572 | 4,073,628 | +3,056 (+0.0751%) |
| General | Release WASM `wasm-opt -Oz` | 2,701,864 | 2,703,302 | +1,438 (+0.0532%) |
| Immediate | Release WASM `wasm-opt -Oz` | 2,716,899 | 2,718,337 | +1,438 (+0.0529%) |
| General | Size native text | 1,854,346 | 1,855,338 | +992 (+0.0535%) |
| Immediate | Size native text | 1,866,846 | 1,867,846 | +1,000 (+0.0536%) |
| General | Size WASM `wasm-opt -Oz` | 1,152,087 | 1,153,145 | +1,058 (+0.0918%) |
| Immediate | Size WASM `wasm-opt -Oz` | 1,162,447 | 1,163,505 | +1,058 (+0.0910%) |

The equal-length repeated-operation executable grows from 6,371,744 to
6,377,360 file bytes, 5,055,578 to 5,060,542 text bytes, and 5,313,141 to
5,317,241 aggregate text/data/BSS bytes. BSS falls from 1,931 to 1,067 bytes.

The Hypermesh-only graph remains 8,018 nodes / 19,670 edges. The five-crate
graph remains 19,683 nodes and moves from 39,286 to 39,288 edges. The only net
production additions are the `to_u128` admission call and its `is_some` check.
The renamed/extended differential test replaces its old test-only edges; there
is no new policy, terminal, fallback, allocation, ownership, or topology spine.

## Competitive and historical controls

One CPU-9 Criterion session reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Exact-cell union, 3,072 triangles/operand | 1.3811–1.3883 ms (1.3852 center) | 8.5163–11.852 ms (9.7067) | 4.3654–4.3822 ms (4.3731) |
| Projective generated union | 6.4138–6.9092 ms (6.5581 center) | 762.66–766.49 us (764.75) | 695.96–742.98 us (713.93) |

An isolated Hypermesh projective repeat tightens to 6.4637–6.5229 ms with a
6.4932 ms center. The full session moved slower for all three engines relative
to checkpoint 23, so it is directional rather than a candidate A/B. Hypermesh
is 7.01x faster than Boolmesh and 3.16x faster than Manifold on the exact-cell
control. It remains about 8.58x and 9.19x slower in the same projective session;
that general-projective gap remains a primary target.

The directional retained historical baseline remains 944.8 ms, 67.74 MiB, and
5,020,891 allocations. Current strict direct work is 35.2979 ms, 12.38 MiB,
and 454,001 allocations: 96.26%, 81.72%, and 90.96% below those historical
values. Fixture and implementation evolution make this a trend, not a direct
A/B.

## Rejected alternatives

- An unguarded four-term probe improved generated instructions about 3.44% but
  raised retained instructions about 0.028% and grew representative text
  3,944 bytes.
- A retained-fact denominator prefilter added about 0.216% generated
  instructions relative to the unguarded form, grew text, and did not protect
  the retained case.
- Raw odd-denominator inspection did not discriminate the retained failures.
- A numerator `bits() <= 128` guard worked, but `to_u128().is_some()` measured
  better and directly matches the accumulator's conversion requirement.
- Reordering terms to fail the retained case earlier reduced the generated
  improvement from about 3.70% to 3.41%.
- Reusing the prechecked fourth numerator shrank text 512 bytes but regressed
  generated instructions about 0.45% and branches about 0.19%.

All rejected code, diagnostic counters, labels, temporary assertions, and the
repetition hook were removed before final validation.

## Validation

The final implementation passes:

- default, no-default, and all-feature tests for Hyperreal, Hyperlattice,
  Hyperlimit, Hypertri, and Hypermesh;
- 560/560/637 Hyperreal library tests and 1,057/1,057/1,058 Hypermesh library
  tests, plus all integration and doc tests;
- warning-denied all-target Clippy and warning-denied rustdoc on all/no-default
  feature surfaces in all five crates;
- formatting and diff checks in all five crates;
- every Hyperreal and Hypermesh benchmark target and every Hypermesh fuzz bin;
- the final 38-test nightly AddressSanitizer dyadic sweep;
- the all-family dispatch trace;
- opt-in release YeahRight competitor-input, every-operation closure and
  nondegeneracy, polygon/immediate consistency, 3,360/13,440-triangle stress,
  and full 11,894-triangle input-validation gates; and
- final Heaptrack, native/WASM size, call-graph, competitive, and profile runs.

The approximately 56-minute full-resolution rotated Boolean was not rerun:
this change only schedules an already-proved exact scalar accumulator earlier,
all failure paths fall through unchanged, and the larger exact release gates
exercise both successful and rejected four-term probes. Its prior certified
empty result took 3,357.09 seconds and peaked at 319.07 MiB.
