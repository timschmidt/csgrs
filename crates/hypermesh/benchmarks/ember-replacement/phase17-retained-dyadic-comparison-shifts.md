# Phase 17 retained dyadic comparison shifts

Date: 2026-08-05

Status: accepted performance checkpoint; Phases 11, 17, and 18 remain open

Implementation: Hyperreal `e238dc1`; Hypermesh remains `63a7bc4e`

Direct parent: Hyperreal `14258dc`; Hypermesh `63a7bc4e`

## Result

Exact rational comparison repeatedly rediscovered that the denominator of a
canonical dyadic value was a power of two. That work was disproportionately
expensive for the wide exact coordinates exposed by the permanent thin-dyadic
mesh family. Hyperreal already owns a packed, monotonic denominator-shift fact
on every immutable `Rational`; comparison now asks that canonical owner for the
shift instead of rescanning the same `BigUint` denominator.

The change is deliberately local and general:

- both comparison operands call
  `Rational::dyadic_denominator_shift_if_reduced`;
- the comparison-local `power_of_two_shift` duplicate is deleted;
- a reduced dyadic learns its shift once and later comparisons reuse it;
- a known non-dyadic returns immediately from the same retained fact;
- an internally unreduced value declines this fast path and enters the
  unchanged complete exact comparison fallbacks.

There is no coordinate-width threshold, fixture name, mesh size, operation,
topology, expected-result, policy-name, benchmark, or competitor branch. No
field, allocation, dependency, public API, compatibility shim, or alternate
engine is added. This is Hyperreal-style scheduling: retain an exact structural
fact at its scalar owner, exploit it when proved applicable, and otherwise fall
through to the complete exact path.

## Exactness and policy

The regression constructs exact `5/2^512` and `3/2^511` values, proves their
order, observes the packed shift encodings, repeats the comparison 128 times,
and proves the retained facts do not change. It also compares an internal raw
`10/2^513` representation with canonical `5/2^512`; because the raw value is
not reduced, the retained-shift route declines and the existing exact fallback
proves equality.

Rational comparison itself has no approximate result. Consequently:

- every exact thin-dyadic output remains 2,410 vertices and 4,816 union
  triangles;
- `STRICT` and `APPROXIMATE_512` remain byte-identical and `Certified` for the
  512-bit large fixture and the full YeahRight control;
- `STRICT` remains exact-only;
- the general depth-128 symbolic control still reports
  `Approximate512Consumed` only at Hyperlimit's terminal 512-bit equality;
- Hypermesh still absorbs terminal certainty into the one operation context.

## Paired thin-dyadic performance

CPU-11-pinned `perf stat -r 3` measurements wrap one fresh release process and
one union. Parent and current executables were measured in the same session.
Setup remains included. Retired work falls at every exact width, while output
and certainty remain identical.

| Shift / `STRICT` | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Parent cycles | Current cycles | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 1,195,582,867 | 1,175,327,260 | -1.6942% | 205,341,264 | 199,169,890 | -3.0054% | 465,413,934 | 463,136,056 | -0.4894% |
| 512 | 1,644,102,551 | 1,564,149,168 | -4.8630% | 296,813,019 | 270,788,627 | -8.7679% | 639,851,116 | 602,185,235 | -5.8867% |
| 2,048 | 2,331,027,131 | 2,145,069,155 | -7.9775% | 445,853,271 | 384,497,692 | -13.7614% | 871,713,329 | 790,771,069 | -9.2854% |

The monotonic width trend is expected: the retained fact replaces a scan whose
cost grows with the exact denominator, while the packed atomic query stays
bounded.

## Broad controls and honest tradeoffs

The same final source was compared with the committed parent across ordinary,
dense, wide, full-mesh, and symbolic policy controls.

| Workload | Instruction change | Branch change | Certainty |
| --- | ---: | ---: | --- |
| crossing octahedra, all four x1000 | -0.1089% | -0.3597% | `Certified` |
| affine boxes, all four x1000 | +0.1757% | +0.0300% | `Certified` |
| sparse 512 shells, all four x5 | +0.3668% | -0.1099% | `Certified` |
| dense coplanar 32, all four x1 | -1.3216% | -2.3316% | `Certified` |
| clipped voxel torus 33, all four x3 | +0.5052% | +0.1369% | `Certified` |
| 2,049-bit wide boxes, union x5 | -9.9542% | -17.8183% | `Certified` |
| full YeahRight instrumented kernel x1 | -0.4375% | -0.8332% | `Certified` |
| symbolic depth 1, `STRICT`, all four x20 | +0.0344% | +0.0202% | `Certified` |
| symbolic depth 128, `APPROXIMATE_512`, all four x5 | +0.0016% | -0.0014% | `Approximate512Consumed` |

Full-control cycles fall 1.8307%. The largest ordinary instruction movement is
an openly reported +0.5052%; it is retained because the same clean scalar rule
removes 7.98--9.95% of instructions and 13.76--17.82% of branches from the
wide exact controls, improves the dense and full controls, and deletes shipped
code. No special case was introduced to hide the small ordinary lookup cost.

## Runtime profile

Twenty-one 512-bit unions were sampled at 999 Hz with DWARF call stacks. In the
parent profile, `BigUint::trailing_zeros` accounted for 3.94% self time and
`Rational::partial_cmp` for 8.15%. In the final profile those rows fall to
0.24% and 6.09%. `compare_shifted_biguints` remains at 5.13%, demonstrating
that the exact borrowed-digit comparison is still the active decision path;
only repeat structural discovery was removed.

The final profile is stored at
`target/phase17-retained-dyadic-comparison-512-21.data`. Sampling percentages
are diagnostic rather than deterministic scorecard values; the paired retired
counters above are authoritative.

## Large-fixture heap and RSS

The instrumented 512-bit large fixture is exactly unchanged under both
policies:

| Requested-payload metric | Parent | Current |
| --- | ---: | ---: |
| input payload | 798,040 | 798,040 |
| total peak | 23,610,989 | 23,610,989 |
| incremental kernel peak | 22,812,314 | 22,812,314 |
| post-Boolean incremental | 548,712 | 548,712 |
| output-live payload | 520,472 | 520,472 |
| input fact growth | 28,240 | 28,240 |
| post-input-drop residual | 122,584 | 122,584 |
| allocation calls | 400,128 | 400,128 |
| deallocation calls | 399,851 | 399,851 |
| reallocations | 6,081 | 6,081 |
| cumulative added bytes | 46,478,238 | 46,478,238 |
| cumulative removed bytes | 45,929,526 | 45,929,526 |

The full 23,788-input-triangle YeahRight control is also byte-identical between
parent/current and `STRICT`/`APPROXIMATE_512`: 59,440,454-byte total peak,
52,317,092-byte incremental kernel peak, 9,906,413 allocations, 1,612,899
reallocations, and 582,162,615 cumulative added bytes. Reusing a packed fact
therefore shifts neither live ownership nor allocation traffic.

Three fresh-process RSS runs produce medians of 29,156 KiB under `STRICT`,
29,168 KiB under `APPROXIMATE_512`, and 10,076 KiB for CGAL EPECK. The current
2.894x/2.895x losses remain open. RSS is noisy at this scale; the allocator
boundary above is the exact memory result.

## Pinned CGAL EPECK boundary

The same reduced-rational OFF inputs and CGAL 6.0.3 EPECK adapter produce the
same valid, closed 2,410-vertex/4,816-triangle union. One process performs 21
unions; Hypermesh includes exact fixture construction once and CGAL includes
OFF parsing once.

| Engine / revision | Instructions | Branches | Cycles |
| --- | ---: | ---: | ---: |
| Hypermesh parent | 33,127,108,506 | 5,997,331,669 | 12,612,964,143 |
| Hypermesh current | 31,476,829,077 | 5,460,394,310 | 12,055,899,202 |
| CGAL EPECK | 4,854,933,895 | 893,866,367 | 1,828,469,164 |

Current Hypermesh removes 4.9817% of parent instructions, 8.9529% of branches,
and 4.4166% of cycles. It remains 6.483x, 6.109x, and 6.593x above CGAL.
Hypermesh's measured internal mean is 136.744 ms versus CGAL's 6.391 ms median,
a still-open 21.40x wall-time loss. Frequency-sensitive clocks are advisory;
no parity claim is made.

## Code and binary size

Production replaces two calls and deletes the five-line duplicate scanner.
The remaining 52 inserted lines are the exact regression test. Linked release
native and optimized WASM shrink in every row. Size-profile native text grows
224--344 bytes, while optimized size WASM shrinks 199 bytes in every row.

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,957,890 | 1,957,714 | -176 |
| default release native immediate text | 1,961,034 | 1,960,858 | -176 |
| default release optimized WASM general | 1,381,629 | 1,381,349 | -280 |
| default release optimized WASM immediate | 1,383,470 | 1,383,190 | -280 |
| default size native general text | 1,069,047 | 1,069,271 | +224 |
| default size native immediate text | 1,070,003 | 1,070,235 | +232 |
| default size optimized WASM general | 662,037 | 661,838 | -199 |
| default size optimized WASM immediate | 662,450 | 662,251 | -199 |
| all-feature release native general text | 2,093,367 | 2,093,127 | -240 |
| all-feature release native immediate text | 2,096,223 | 2,095,983 | -240 |
| all-feature release optimized WASM general | 1,455,608 | 1,455,434 | -174 |
| all-feature release optimized WASM immediate | 1,457,580 | 1,457,406 | -174 |
| all-feature size native general text | 1,071,287 | 1,071,631 | +344 |
| all-feature size native immediate text | 1,072,259 | 1,072,595 | +336 |
| all-feature size optimized WASM general | 662,295 | 662,096 | -199 |
| all-feature size optimized WASM immediate | 662,334 | 662,135 | -199 |

## Call graph and rejected alternatives

Fresh exact-scope graphs for Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and
Hypermesh contain 15,152 production nodes / 25,274 edges, 16,606 nodes /
27,566 edges with tests, and 21,534 / 34,715 with tests, examples, benches,
and fuzz targets. Hypercurve and HyperSolve are excluded. The comparison node
reaches the two operand calls to the one retained dyadic-shift accessor and the
unchanged shifted-`BigUint` comparison. The removed comparison-local scanner
has no node. There are zero actual `ember` namespace nodes.

Three measured alternatives were removed completely:

1. A Hypermesh recent-entry hint in the rational linear-form cache raised the
   512-bit target to 1,654,012,677 instructions / 298,176,165 branches.
2. A helper that admitted raw unreduced dyadics raised it to
   1,567,335,544 / 272,413,059 and did not repair ordinary controls. The clean
   exact fallback is both simpler and faster.
3. A one-word-denominator hybrid raised it to 1,693,692,758 / 300,573,851,
   worse than the committed parent.

None of those caches, width branches, or duplicate helpers remain in either
repository.

## Validation and reproduction

Hyperreal passes 653 all-feature and 575 no-default unit tests, all integration
suites, and 24/19 respective doctests. Hypermesh passes 203 default tests with
six ignores, 204 all-feature tests with six ignores, and 154 minimal library
tests. Warning-denied all-target/all-feature Clippy and rustdoc, fuzz-target
checks, bench/example builds, formatting, diff checks, both size matrices,
both-policy large heap probes, the full large-mesh control, CGAL validation,
profiling, and all three call graphs pass.

Representative commands:

```sh
cargo test --locked --all-features
cargo test --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run --all-features
cargo check --locked --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

Phases 11, 17, and 18 remain open. This checkpoint improves exact rational
comparison scheduling and deletes duplicate work; it does not close the
remaining per-case CGAL runtime/RSS deficits, external real-world corpus work,
remaining arrangement/corefinement ownership, or final requirement audit.
