# Normalized rational relative filter conversion

Date: 2026-08-02

Status: retained as Phase 7 checkpoint 44

Revisions:

- Hypermesh evidence parent: `1af84f4d3907d1bf4389644417f4718189eab9f32`
- Hyperreal implementation: `249119bc6bbb67899da2e2cb93282c43475f92cb`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`

## Outcome

Hyperreal's normalized four-term rational filters no longer construct and
validate an absolute binary64 conversion-error radius that they never
consume. `RationalLinearForm4Filter` and `RationalLinearForm4Query` now admit
only a finite normal approximation (or exact zero), normalize each four-lane
vector by an exact positive power of two, and use the existing proved relative
conversion bound in the existing 82-epsilon sign certificate.

Interval filters still call `rational_f64_with_error` and still require the
same representable absolute-error radius. The shared wrapper now calls the
relative conversion helper before constructing that radius, removing
duplicated admission code. No public API, dependency, feature, compatibility
shim, policy state, or mesh path was added. The production source change is a
net ten lines and the matched production probe is 320 bytes smaller in both
ELF `.text` and stripped file size.

Across `STRICT` and `APPROXIMATE_512`, generated/retained/dense policy-mean
instructions fall 0.229%/0.083%/0.042% and branches fall
0.768%/0.322%/0.096%. Policy-mean task clock moves
-0.036%/-0.931%/-0.157%, and cycles move -0.163%/-0.856%/-0.083%.
Allocation counts and peak heap are byte-class neutral on all six large
fixture/policy rows.

## Exactness and complete-path invariant

`Rational::to_f64_lossy` already provides the conservative relative
conversion premise used by the four-term certificate. The retained schedule
is sound because:

1. a nonzero input is admitted only when its approximation is finite and
   normal; exact zero is the only input admitted with a zero approximation;
2. each coefficient vector and query vector is normalized independently by
   an exact positive power of two, so the exact linear-form sign is unchanged;
3. the normalizer rejects every nonzero normalized lane below exponent -500;
4. every nonzero coefficient/query product is therefore at least exponent
   -1000, safely above binary64's minimum normal exponent -1022;
5. the existing 82-epsilon bound covers both 32-epsilon relative conversions,
   rounded products, and rounded additions; and
6. an unavailable conversion, unsafe exponent span, non-normal product,
   non-normal error bound, or unresolved sign still returns `None` to the
   unchanged exact rational fallback.

The minimum-normal boundary now has direct regression coverage. The old
absolute-radius wrapper rejects `f64::MIN_POSITIVE` because its radius is
subnormal, while the relative four-term path safely normalizes it to one. The
smallest subnormal remains rejected. A second oracle scales randomized exact
rationals across six coefficient/query exponent pairs in the newly admitted
low-normal range and obtains more than 1,500 floating certificates without a
single disagreement with exact `signed_product_sum_ordering`. The existing
4,096-case ordinary-magnitude oracle and exhaustive 2,046-normal-exponent
normalizer test remain green.

## STRICT and APPROXIMATE_512

The conversion helper receives no policy and makes no equality or sign
decision. It can only make the existing floating certificate available; that
certificate either proves a sign under its conservative error bound or
declines to exact arithmetic. `STRICT` therefore remains entirely certified.
`APPROXIMATE_512` can still consume approximation only where Hyperlimit
performs the terminal 512-bit equality/sign interpretation.

Every final-source large run completed as `Certified` with identical topology:

| Fixture | Input triangles | Output vertices | Output triangles | `STRICT` | `APPROXIMATE_512` |
| --- | ---: | ---: | ---: | --- | --- |
| Generated projective | 13,452 | 154 | 304 | `Certified` | `Certified` |
| Retained arrangement | 4,524 | 625 | 1,246 | `Certified` | `Certified` |
| Dense boxes | 6,144 | 27 | 50 | `Certified` | `Certified` |

The generated dispatch trace remains exactly 97,321 events: 676 predicate
events, 1,411 linear-algebra events, 6,341 cache events, 12,775 rational
temporaries, zero unknown facts, and zero fallback/abort events.

## Serialized CPU A/B

Checkpoint 43 and checkpoint 44 use `-C target-cpu=native -C codegen-units=1`,
CPU 11, identical fixtures, and the same temporary optional repetition
argument. Generated rows use parent/candidate/candidate/parent brackets;
retained and dense rows use parent/candidate/parent brackets. The hook was
removed before commits. Percentages are candidate relative to parent;
negative is better.

| Fixture / policy | Repetitions | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 1,001 | -0.326% | -0.288% | -0.2283% | -0.7674% |
| Generated / `APPROXIMATE_512` | 1,001 | +0.253% | -0.037% | -0.2288% | -0.7681% |
| Retained / `STRICT` | 101 | +0.003% | +0.193% | -0.0844% | -0.3241% |
| Retained / `APPROXIMATE_512` | 101 | -1.865% | -1.906% | -0.0816% | -0.3204% |
| Dense boxes / `STRICT` | 10,001 | -1.210% | -1.205% | -0.0411% | -0.0973% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | +0.897% | +1.039% | -0.0419% | -0.0952% |

Arithmetic means across policies:

| Fixture | Task clock | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: |
| Generated 13,452 | -0.036% | -0.163% | -0.229% | -0.768% |
| Retained 4,524 | -0.931% | -0.856% | -0.083% | -0.322% |
| Dense boxes 6,144 | -0.157% | -0.083% | -0.042% | -0.096% |

The generated `STRICT` parent/candidate means are 8,500.335/8,472.665 ms,
35.7110/35.6081 billion cycles, 100.1407/99.9122 billion instructions, and
17.2887/17.1561 billion branches. `APPROXIMATE_512` clocks reverse within the
bracket while cycles still fall; deterministic instruction and branch
directions are essentially identical between policies. Retained and dense
show the same order sensitivity, so retired work is the primary small-change
retention gate.

## Large-fixture heap

Six final-source, no-hook Heaptrack recordings include fixture construction
and one union. The retained OBJ SHA-256 is
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

| Fixture / policy | Allocations | Temporary allocations | Peak heap | Peak RSS |
| --- | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 200,428 | 10,363 | 7.50 MiB | 17.81 MiB |
| Generated / `APPROXIMATE_512` | 200,428 | 10,363 | 7.50 MiB | 17.88 MiB |
| Retained / `STRICT` | 452,722 | 28,731 | 11.60 MiB | 21.12 MiB |
| Retained / `APPROXIMATE_512` | 452,722 | 28,731 | 11.60 MiB | 20.89 MiB |
| Dense boxes / `STRICT` | 2,136 | 65 | 1.14 MiB | 9.14 MiB |
| Dense boxes / `APPROXIMATE_512` | 2,136 | 65 | 1.14 MiB | 9.10 MiB |

Allocation, temporary-allocation, and peak values are identical to checkpoint
43. RSS moves only inside profiler/process noise. This scalar admission change
adds no storage and does not extend any mesh lifetime.

## Native and WASM size

The matched production probe moves as follows:

| Measure | Checkpoint 43 | Checkpoint 44 | Movement |
| --- | ---: | ---: | ---: |
| ELF `.text` | 3,818,134 B | 3,817,814 B | -320 B |
| GNU text | 4,635,033 B | 4,634,713 B | -320 B |
| GNU aggregate | 4,887,156 B | 4,887,156 B | 0 B |
| Unstripped file | 5,586,536 B | 5,586,216 B | -320 B |
| Stripped file | 4,886,472 B | 4,886,152 B | -320 B |

Canonical consumer deltas from checkpoint 43:

| Features / consumer | Native text | Native file | Optimized WASM |
| --- | ---: | ---: | ---: |
| Default general release | -272 B | -280 B | +234 B |
| Default immediate release | -272 B | -288 B | +234 B |
| Default general size | +232 B | +240 B | +753 B |
| Default immediate size | +224 B | +224 B | +748 B |
| All-feature general release | -736 B | -800 B | +510 B |
| All-feature immediate release | -736 B | -800 B | +510 B |
| All-feature general size | +224 B | +224 B | +753 B |
| All-feature immediate size | +232 B | +240 B | +748 B |

Runtime-oriented native consumers shrink. The worst optimized-WASM movement
is 753 bytes (0.064% of the smallest affected optimized artifact), which is
accepted under the performance-first size gate.

## Profile and call graph

The final generated profile contains 34,000 cycle samples with zero loss.
`RationalLinearForm4Query::from_affine_point3` falls from 2.26% to 1.53% self;
`Rational::to_f64_lossy` falls from 3.15% to 2.99%. The shared normalizer is
2.31%, while the next heads remain projective input construction (5.51%),
four- and six-term exact signed product sums (4.64%/4.45%), word/fixed GCD,
crossing splitting, and allocator traffic. The removed absolute-radius work
no longer appears as a separate prerequisite for normalized queries.

The five-crate graph moves from 19,792 nodes / 39,586 edges to 19,800 / 39,607,
SHA-256
`e8c093e80e72adb86404577816be4c333a0e8e8a5241279ff3c5441cbfbf6eb3`.
Hyperreal moves from 7,156 / 12,287 to 7,164 / 12,308, SHA-256
`134bdcb68630c798f8de1ccc9de30990d196e3cb4dd62abbb12a71139273b15a`.
The additions are the private conversion helper and exactness tests.
Hypermesh remains byte-identical at 8,066 / 19,896 with SHA-256
`f4442b58f89903f92a21acbb1127285cb99c1301001bf9163b4e0f7110a22c20`;
Hyperlattice, Hyperlimit, and Hypertri are also byte-identical. No terminal
policy edge, fallback edge, or mesh operation was removed.

## Competitive and historical controls

Fresh Criterion centers on CPU 11:

| Fixture / operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Projective / union | 6.1917 ms | 774.30 us | 672.97 us | 8.00x / 9.20x slower |
| Projective / intersection | 4.4814 ms | 744.44 us | 685.52 us | 6.02x / 6.54x slower |
| Projective / difference | 4.1294 ms | 770.00 us | 673.42 us | 5.36x / 6.13x slower |
| Dense boxes / union | 711.03 us | 7.0141 ms | 4.3721 ms | 9.86x / 6.15x faster |
| Dense boxes / intersection | 506.31 us | 3.9744 ms | 3.4638 ms | 7.85x / 6.84x faster |
| Dense boxes / difference | 659.21 us | 6.2095 ms | 4.0469 ms | 9.42x / 6.14x faster |

Criterion classifies every Hypermesh projective operation as unchanged.
Dense union/difference are within the noise threshold and dense intersection
improves 1.287% within that threshold. Cross-engine session movement remains
larger than this scalar checkpoint, so the serialized same-binary counters
above are the retention gate.

The retained union's final repeated policy mean is about 33.89 ms. Against the
directional historical 944.8 ms row, it is 96.41% lower or 27.88x faster. Its
11.60 MiB peak and 452,722 allocations remain 82.88% and 90.98% below the
historical 67.74 MiB and 5,020,891 controls. These are implementation-evolution
trend controls, not revision A/B.

## Verification

The retained implementation passes:

- Hyperreal default/minimal/all-feature suites (567 / 567 / 644 library tests
  plus every enabled integration and doc test);
- Hyperlattice, Hyperlimit, Hypertri, and Hypermesh default, minimal, and
  all-feature suites; Hypermesh has 1,064 / 1,064 / 1,065 library tests;
- warning-denied all-target Clippy, warning-denied minimal/all-feature rustdoc,
  and formatting for all five crates;
- eight focused Hyperreal tests and all 1,064 Hypermesh library tests under
  nightly AddressSanitizer with leak detection disabled;
- every Hypermesh fuzz-bin check, all-feature benchmark compilation, and
  minimal/all-feature `wasm32-unknown-unknown` library checks;
- both-policy large topology/Heaptrack, generated dispatch tracing, canonical
  native/WASM size, final profile, and five-crate/per-library call graphs; and
- fresh projective/dense competitive controls plus diff whitespace checks.

The known approximately 56-minute ignored full-resolution rotated-intersection
control was not rerun. Its bounded full-resolution gate is unchanged, this
scalar filter retains every exact fallback, and both large projective policy
runs pass.

The pre-existing untracked Hyperlimit `hyperlimit` executable was untouched.
No diagnostic counter, temporary repetition hook, or measurement-only source
remains in a production tree. Machine-readable raw and derived values are in
`normalized-rational-relative-filter-2026-08-02.toml`.
