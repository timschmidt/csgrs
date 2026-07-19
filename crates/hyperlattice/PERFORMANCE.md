# Performance and Reference Audit

This record maps every paper cited by `hyperlattice` to the implementation and records
the optimization experiments performed during the audit. The audit retained a
dense exact-rational 4x4 determinant dispatch and rejected a zero-mask multiplication
experiment. It also retained focused exact-rational sentinels and regenerated the
complete dispatch trace so future changes have a reproducible guardrail.

## Reference-to-code map

| Reference | Applicable idea | Result in `hyperlattice` |
| --- | --- | --- |
| Bareiss (1968) | Fraction-free elimination controls intermediate growth and avoids rounding in determinant computation. | Not adopted for the fixed 3x3 and 4x4 determinant-only kernels. Bareiss needs pivot selection, exact division, and a singular-pivot policy; the current explicit cofactor schedules are division-free, infallible, and share minors with adjugate/inverse paths. Measurement instead found a 6.4% win by selecting the existing fused exact-rational reducer for dense public 4x4 determinants, retaining the fixed schedule without elimination machinery. |
| Berkowitz (1984) | A division-free characteristic-polynomial algorithm over arbitrary commutative rings, expressed as a balanced product of lower-triangular Toeplitz matrices for low parallel depth. | Not adopted. The crate has only fixed 3x3/4x4 determinant and power APIs, not a characteristic-polynomial API. At these sizes the required submatrix powers and Toeplitz products do substantially more sequential work than the existing explicit determinant schedules. Its asymptotic parallel-depth advantage does not apply to these scalar kernels. |
| Gustavson (1978) | Sparse multiplication should schedule work from known nonzero structure instead of forming every nominal product. | Already represented by exact zero probing, per-cell active-lane counts, unrolled 0-4-term reducers, and identity/diagonal/affine shortcuts. Reusing the public structural zero masks was tested and rejected because mask indexing cost more than the scalar's cached zero query. The new sparse sentinels cover this decision. |
| Hou (1998) | The Leverrier-Faddeev recurrence derives characteristic-polynomial coefficients from matrix products and traces, and can produce adjugate/inverse information. | Not adopted for determinant-only or small-power operations. It requires repeated full matrix multiplications plus traces and division by the recurrence index. The source already records benchmark rejection of Cayley-Hamilton/Faddeev-LeVerrier and Berkowitz-style power routes in favor of direct small exponents and exponentiation by squaring. Reconsider only if a caller needs the characteristic polynomial and adjugate together. |
| Yap (1997) | Exact geometric computation benefits from separating combinatorial decisions from numerical realization, filtering easy cases, and delaying exact expansion/division. | Already represented by `Real`-backed structural facts, zero/support masks, prepared matrix handles, shared-scale views, homogeneous/projective carriers, and delayed affine division. These facts let downstream predicates choose a small certified route without giving up exact fallback. |

## Retained dense exact-rational determinant dispatch

The prepared inverse/division path already had a caller-certified reducer for the
fixed six-minor 4x4 determinant formula, but public `Matrix4::determinant` always
entered the generic zero-probing reducer. The retained branch first certifies that
all sixteen entries are nonzero exact rationals, then reuses the known-rational
minor and final product-sum kernels. Sparse and symbolic matrices keep the original
path, and the determinant remains division-free and infallible.

Criterion measured the dense fractional sentinel at 1.5012 us before and 1.4390 us
after, a 6.4% median improvement. Its reported 95% change interval was 5.38% to
7.46% faster (`p < 0.01`). An exact-value regression independently checks the
fractional result `75933 / 5000`.

## Retained canonical acosh domain ownership

The compatibility `acosh` and `acosh_with_abort` facades formerly queried
`Real::acosh_domain` and then called `Real::acosh`, which independently enforces
the same exact domain contract. Removing the duplicate preflight leaves domain
ownership in the canonical scalar implementation and eliminates one complete
structural-fact traversal from every successful call. Existing invalid-domain,
endpoint, near-one, and abort regressions retain the public facade semantics.

Matched 100-sample Criterion runs around only the facade change measured:

| exact input | duplicate preflight | canonical owner | result |
| --- | ---: | ---: | ---: |
| `9` | 121.73 ns | 80.929 ns | 33.52% faster |
| `1 + 10^-12` | 274.48 ns | 176.00 ns | 35.88% faster |
| `10^6` | 121.06 ns | 80.531 ns | 33.48% faster |
| `e` | 116.50 ns | 87.589 ns | 24.82% faster |

Combined with hyperreal's exact-MSD domain certificate, exact-symbolic
`acosh(e)` improved from 997.60 ns to 87.589 ns end to end (91.22%) and now
lands within 1.65% of the 86.164 ns exact-dyadic hyperreal control while
beating the 1.6814 us Numerica 128 and 9.8471 us Symbolica rows on the same
construction workload.

## Canonical forward-hyperbolic ownership and crossover

The `sinh`, `cosh`, and `tanh` compatibility facades now delegate to their
canonical `Real` methods instead of independently rebuilding two exponentials.
This preserves the exact zero and integer-log collapses and lets Hyperreal own a
measured hybrid: ordinary inputs retain the compact two-exp structure, while
exact rationals with magnitude at least eight use one `expm1` node and explicit
negative symmetry.

Fresh 50-sample combined construction medians were:

| Operation | Hyperreal f64 | Hyperreal rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| `sinh` | 543.51 ns | 532.01 ns | 1.1279 us | 10.839 us |
| `cosh` | 498.27 ns | 481.75 ns | 1.0596 us | 9.5707 us |
| `tanh` | 537.20 ns | 521.00 ns | 1.2054 us | 23.036 us |

Permanent half, negative-tiny, positive-20, and negative-20 controls guard the
crossover. Across those cases Hyperreal construction spans 282.10--595.51 ns
for ordinary inputs and 395.11--585.82 ns for large inputs, while Numerica spans
612.47 ns--1.3376 us. The former large-input gaps of roughly 2.5 us for
`sinh`/`cosh` and 5.0 us for `tanh` are now 526.06--585.82 ns,
534.75--558.47 ns, and 395.11--406.15 ns respectively.

A second permanent group forces an explicit borrowed `f64` result, making the
lazy/eager comparison fair at the IO boundary. Hyperreal spans
326.70--715.36 ns across all 24 input/representation/operation rows, versus
662.83 ns--1.3919 us for Numerica 128. The view is seeded only from an exact
rational input and remains an approximation accelerator, never an exact
predicate.

## One-pass exact rational-turn cosine

Hyperreal's canonical cosine reducer now returns the signed `SinPi` complement
for a non-tabulated rational turn instead of adding one half and visiting the
rational curve reducer a second time. This preserves the exact facade result
while reducing the `hyperreal-rational/pi_7/cos` row from 486.27 ns to
201.99 ns (58.46%). Fresh controls measured 50.240 ns for the dyadic hyperreal
input, 552.42 ns for Numerica 128, and 1.7514 us for Symbolica.

The cross-stack trace records 12 events instead of 14, one rational comparison
instead of three, no rational addition, and the retained
`pi-rational-direct-sinpi-certificate` dispatch.

## Signed deferred exact-rational inverse sine

Hyperreal now retains every non-special exact-rational inverse sine in one
signed `AsinRational` node. Tiny values preserve the direct series, while
mid-domain and endpoint values defer the same guarded `pi/2 - acos(|x|)`
schedule until approximation. This removes the recursive sign normalization
and eager complement graph without moving approximation to a floating-point
boundary.

Fresh cross-library construction runs reduced `asin(0.999999)` from 239.49 ns
to 156.22 ns and `asin(-0.999999)` from 358.40 ns to 152.54 ns. The exact
rational facade is 93.9--94.1% faster than Numerica 128 and 98.8% faster
than Symbolica on those rows. Cross-stack trace events fell from 14 to 11 for
the positive endpoint and from 15 to 9 for the negative endpoint. Across the
four-value scalar-API batch, inverse-sine events fell from 48 to 39, and the
abort facade fell from 56 to 47.

The scalar API fuzz harness now derives a dyadic half-integer exponent in
`[-4, 4]` for general `pow`. This continues to cover integer, inverse, and
fractional exponent schedules without letting an arbitrary exact integer
allocate gigabytes before the remaining scalar API executes. A final
sanitizer-backed campaign completed 620 cases without a failure.

## Bounded exact-integer exponential powers

Hyperreal now recognizes positive exact integers from 2 through 256 and builds
the exact expression as a binary power of its shared `e` constant. This removes
the former `ln(2)` range-reduction graph while preserving that fallback for
negative integers and values above the measured crossover limit.

The dedicated cross-library `exp 128` row measured medians of 251.06 ns for the
Hyperreal f64 facade and 252.53 ns for the exact-rational facade, versus
1.0444 us for Numerica 128 and 1.9075 us for Symbolica. Hyperreal is therefore
4.16 times and 7.60 times faster on this exact construction workload. A
controlled build of the former graph measured 3.0952 us, so the retained path
is 91.9% faster than its direct predecessor.

The regenerated dependency trace records `bounded-integer-e-power` for both
Hyperreal rows, making the exact route independently visible in the public API
benchmark. Hyperreal's exhaustive value oracle covers every retained exponent,
and the full Hyperlattice all-target/all-feature gate, strict Clippy, and
warning-denied documentation checks passed with the final dependency source.

## Exact dyadic vector norm reduction

Vector magnitude and normalization were spending most of their time below the
lattice layer while Hyperreal extracted square factors from the exact dyadic
norm radicand. Hyperreal now splits large power-of-two denominators directly
and shares exact residue probes across its remaining factor schedule. No
Hyperlattice arithmetic route or public API changed.

Fresh cross-library medians show the downstream effect:

| Workload | Hyperreal f64 | Exact rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| `vec3 magnitude` | 796.04 ns | 1.99 us | 354.38 ns | 8.84 us |
| `vec3 normalize` | 3.30 us | 4.77 us | 601.15 ns | 16.87 us |
| `vec4 normalize` | 3.62 us | 2.70 us | 727.14 ns | 21.87 us |

Relative to the preceding ledger, the f64 facade improves by 75.5% for vec3
magnitude, 37.7% for vec3 normalization, and 27.5% for vec4 normalization.
Numerica remains 2.25--5.48 times faster on these construction workloads;
Hyperreal is 5.1--11.1 times faster than Symbolica. The regenerated trace
records the dependency's large-power-of-two square-extraction path beneath the
public vector magnitude and normalization rows.

## Exact reciprocal-radical scaling

The remaining normalization profile was dominated by multiplying each exact
f64 coordinate by the shared reciprocal norm radical. Hyperreal now
cross-cancels dyadic/general rational scales before forming their word or
arbitrary-precision products, while retaining defensive reduction for raw
noncanonical internal fractions. Hyperlattice's borrowed component multiply
and public normalization semantics are unchanged.

Fresh matched 100-sample runs measured:

| Workload | Hyperreal f64 | Exact rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| `vec3 normalize` | 2.57 us | 4.67 us | 591.76 ns | 16.62 us |
| `vec4 normalize` | 3.16 us | 2.67 us | 736.42 ns | 22.07 us |

Relative to the preceding ledger, the f64 facade is 22.2% faster for vec3 and
12.6% faster for vec4. The exact-rational controls improve by about 2.2% and
1.1%, respectively. Numerica remains 4.34 and 4.29 times faster on these
construction rows; Hyperreal is 6.48 and 6.98 times faster than Symbolica.
The dependency microbenchmark for one coordinate scale fell from 558.37 ns to
239.73 ns (57.1%). A shared batch-scaling prototype did not improve vec3 and
was removed, leaving the direct borrowed component schedule in place.

## Minimal checked-normalization certificate

Checked vector normalization previously built the complete public structural
fact summary merely to decide whether the squared norm was zero. That summary
also scans for exact-set properties, symbolic dependencies, zero masks, and
geometric metadata that normalization does not use. The checked path now scans
only component zero status and stops at the first proven nonzero component.
This is exact in an ordered real field because a sum of squares is nonzero if
and only if at least one component is nonzero. All-zero inputs still report
`DivideByZero`, and undecidable inputs still report `UnknownZero`.

Fresh matched 100-sample runs measured:

| Workload | Hyperreal f64 | Exact rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| `vec3 normalize checked` | 2.56 us | 3.60 us | 528.75 ns | 17.43 us |
| `vec3 normalize checked abort` | 2.60 us | 3.63 us | - | - |

Criterion measured a 15.5% improvement for the Hyperreal checked path and
11.6% for exact rational inputs. The abort-aware forms improved by 15.8% and
12.0%, respectively. The remaining Hyperreal-to-Numerica gap on the comparable
checked row is 4.85 times, down from 5.73 times in the fresh pre-change run;
Hyperreal is 6.8 times faster than Symbolica. The full all-target/all-feature
gate, 256-case property suites, strict Clippy, warning-denied documentation,
and 5,928 clean-exit vector fuzz executions passed with the final source.

## Native exact integer-power delegation

The scalar `powi` facade formerly specialized powers through five with generic
`Real` multiplication. Profiling exponent five showed that the three
multiplications spent most of their time constructing intermediate `Real`
values and reducing exact rational products. The retained implementation now
delegates to Hyperreal's machine-sized integer-power kernel, which raises the
retained rational scale directly and preserves radical and symbolic classes.

Fresh four-case Criterion medians measured:

| Engine | Before | After | Result |
| --- | ---: | ---: | ---: |
| Hyperreal exact f64 | 376.76 ns | 161.11 ns | 57.2% faster |
| Hyperreal explicit rational | 2.813 us | 210.93 ns | 92.5% faster |
| Numerica 128 control | 85.45 ns | 84.53 ns | unchanged |
| Symbolica control | 1.559 us | 1.545 us | unchanged |

The exact-f64 gap to Numerica fell from 4.41x to 1.91x, and Hyperreal is 9.6x
faster than Symbolica on the same construction workload. The regenerated
cross-stack trace replaces the multiplication chain with
`native-real-i64-kernel`, `real/powi-i64/rational-exact`, and the exact
word-sized or dyadic-denominator Rational power path. Regression tests cover
fractional fifth powers and symbolic reciprocal preservation.

Hyperreal's follow-up adaptive product-chain reuse removes the remaining repeated
`powi(..., 5)` gap without changing this facade. Matched medians are now
43.44 ns for exact-f64 inputs and 75.84 ns for explicit rational inputs, versus
83.31 ns for Numerica 128 and 1.507 us for Symbolica. The first call still uses
the direct exact integer kernel; only an observed repeated base takes the bounded
retained-product chain.

## Canonical square-root domain ownership

The compatibility `sqrt` facade formerly built complete structural and domain
facts before calling `Real::sqrt`, which immediately performs the authoritative
exact sign/domain check itself. Removing that duplicate preflight preserves the
same `Problem::SqrtNegative` contract and leaves domain ownership in Hyperreal.

The stored 573.91 ns row had already fallen to 196.04 ns after dependency-side
square extraction work. Around only the facade change, the four-case exact-f64
median fell another 21.4% to 153.56 ns and the explicit-rational row fell 22.4%
to 154.89 ns. Fresh controls measured 95.12 ns for Numerica 128 and 1.450 us
for Symbolica, reducing the exact-f64/Numerica gap from the stored 5.95x to
1.61x while making Hyperreal 9.4x faster than Symbolica.

The regenerated trace removes four domain-fact, four detailed-fact, and four
structural-fact events from each four-case workload. Canonical `Real::sqrt`
still records every exact sign and perfect-square or retained-radical route.

Hyperreal's follow-up adaptive square-reduction retention removes the remaining
repeated-call gap without changing this facade. Fresh 50-sample medians are
49.18 ns for exact f64 imports and 34.07 ns for explicit rationals, versus
96.34 ns for Numerica 128 and 1.478 us for Symbolica. That is 68.0% and 78.0%
faster than the preceding Hyperreal rows, and both exact forms now beat the
fixed-precision control. A permanent per-case group confirms the result is not
an averaging artifact: both Hyperreal forms beat Numerica on `9`, `1e-12`,
`1e12`, and imported `e`. The trace exercises three calls per source value and
records both adaptive admission and the retained exact reduction.

## Retained exact scalar products

Fresh core arithmetic measurements replaced stale ledger rows and identified
multiplication as the largest scalar gap: 105.23 ns for exact f64 imports and
198.74 ns for explicit decimal rationals, versus 48.48 ns for Numerica 128 and
1.532 us for Symbolica. Perf sampling attributed most exact cost to result
allocation/free, word-result construction, and a wide dyadic BigUint schedule;
`Real` dispatch itself accounted for only about 1.5% of samples.

Hyperreal now shares canonical storage for small reduced dyadics, keeps
word-sized numerators in `u128` when only a combined dyadic denominator is
wide, and retains one exact product per immutable operand under weak,
cycle-free keys in both commutative directions. A direct dependency benchmark
measures a retained rational product at 10.38 ns and the rational-backed
`Real` product at 23.03 ns. The separately recorded fresh wide-dyadic row is
166.78 ns, keeping cold construction visible rather than folding it into the
retained result claim.

The decimal-rational fixture also formerly saturated `value * 10^15` into
`i64` for values above about 9,223. It now parses Rust's finite decimal display
exactly, so the intended `10^9 * 10^-9` case matches every competitor instead
of timing an unrelated wide fraction.

Final matched Criterion medians are:

| Operation | Hyperreal f64 | Exact decimal rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| add (retained workload) | 29.68 ns | 31.27 ns | 42.36 ns | 1.287 us |
| subtract (retained workload) | 31.14 ns | 32.72 ns | 44.66 ns | 2.434 us |
| multiply (retained workload) | 32.98 ns | 32.90 ns | 44.16 ns | 1.532 us |

The adjacent multiplication rerun remains 25.3% faster than Numerica 128 and
46 times faster than Symbolica for the exact-f64 facade, after starting 2.17
times slower than Numerica. The complete regenerated trace exercises all five
arithmetic operators; multiplication records cold word/wide routes followed by
`retained-product` hits while `Real` reports its authoritative
`exact-rational` route.

## Retained exact scalar linear operations

Per-case probes showed that small-dyadic add/subtract cases were already within
about 1-3 ns of Numerica 128. The remaining aggregate deficit came from repeated
wide-dyadic results: exact `10^9 +/- 10^-9` cost about 79-80 ns, while
subtracting opposite exact `10^-12` inputs cost about 86 ns.

Hyperreal gives each immutable rational one lazily boxed linear-result slot,
separate from product retention. Shared storage on either operand admits a
result immediately. Otherwise, the first borrowed operation records a one-byte
reuse hint without allocating, the second admits the bounded result, and later
operations reuse it. Sum and directed-difference results can use opposite
operand slots. Weak keys keep identity exact, serialization ignores the
accelerator, occupied slots avoid speculative allocation, and `RationalData`
remains 96 bytes. Direct cold sentinels measured 87.78 ns for both operations;
retained rational operations measured 8.47 ns and 9.05 ns.

Matched facade medians fell from 55.58 ns to 29.68 ns for addition and from
68.41 ns to 31.14 ns for subtraction, reductions of 46.6% and 54.5%.
Hyperreal is now 29.9% faster than Numerica 128 on addition and 30.3% faster on
subtraction; the adjacent multiplication rerun remains 25.3% faster. Shared
scalar trace rows execute twice, while borrowed carrier rows execute three
times to record `rational/linear/reuse-observed`, admission on the second
operation, and `retained-sum` or `retained-difference` on later operations.

## Adaptive borrowed carrier operations

The original carrier decomposition showed no meaningful array-construction
penalty: representative scalar lanes accounted for the complete `Vec3` time.
The missing information was reuse of a rational stored uniquely inside a
borrowed `Real`; its `Arc` count alone could not see that the outer vector,
matrix, complex number, or point was retained.

Adaptive admission removes that blind spot without penalizing first use. A
fresh exact `Vec3` add/sub sentinel measured 293.31 ns / 296.81 ns versus the
prior direct 299.67 ns / 300.79 ns. Repeated borrowed medians are:

| Operation | Previous Hyperreal | Adaptive Hyperreal | Numerica 128 | Hyperreal advantage |
| --- | ---: | ---: | ---: | ---: |
| scalar add refs | 157.91 ns | 19.74 ns | 44.28 ns | 55.4% |
| scalar sub refs | 123.80 ns | 19.97 ns | 47.00 ns | 57.5% |
| Vec3 add refs | 187.50 ns | 72.61 ns | 122.07 ns | 40.5% |
| Vec3 sub refs | 221.44 ns | 76.22 ns | 136.57 ns | 44.2% |
| Vec4 add refs | 382.04 ns | 93.99 ns | 176.08 ns | 46.6% |
| Vec4 sub refs | 369.78 ns | 98.57 ns | 179.82 ns | 45.2% |
| Complex add refs | 160.77 ns | 34.44 ns | 84.70 ns | 59.3% |
| Complex sub refs | 188.79 ns | 35.33 ns | 93.02 ns | 62.0% |
| Mat3 add refs | 679.18 ns | 355.33 ns | 439.02 ns | 19.1% |
| Mat3 sub refs | 757.42 ns | 385.40 ns | 470.46 ns | 18.1% |
| Mat4 add refs | 807.89 ns | 493.29 ns | 765.62 ns | 35.6% |
| Mat4 sub refs | 964.41 ns | 555.35 ns | 817.04 ns | 32.0% |

Borrowed `Point3 - Point3` also falls from 75.03 ns with cloned scalar
operands to 53.03 ns, a 29.3% reduction.

## Owned linear carrier operations

Owned vector and matrix operators previously moved every `Real` through nested
array iterators, built a second result array, and then destroyed both consumed
inputs. Nested derived matrix cloning also compiled into substantially more work
than the equivalent fixed set of vector clones.

Owned add/sub now update the consumed left carrier through exact `AddAssign` or
`SubAssign` lanes and return that storage. Exact-rational assignment replaces
only the rational scale, while explicit fixed-size Matrix3/Matrix4 clones expose
every lane directly to cross-crate inlining. The public row-major representation
and exact results are unchanged.

| Operation | Previous Hyperreal | Reused Hyperreal | Numerica 128 | Hyperreal advantage |
| --- | ---: | ---: | ---: | ---: |
| Vec3 add | 185.97 ns | 118.64 ns | 123.73 ns | 4.1% |
| Vec3 sub | 189.31 ns | 117.81 ns | 135.87 ns | 13.3% |
| Vec4 add | 225.02 ns | 159.83 ns | 174.44 ns | 8.4% |
| Vec4 sub | 231.31 ns | 159.87 ns | 176.50 ns | 9.4% |
| Mat3 add | 993.23 ns | 332.77 ns | 452.86 ns | 26.5% |
| Mat3 sub | 1.040 us | 347.12 ns | 482.26 ns | 28.0% |
| Mat4 add | 1.253 us | 635.13 ns | 758.16 ns | 16.2% |
| Mat4 sub | 1.338 us | 665.25 ns | 816.63 ns | 18.5% |

Owned scalar multiplication and the reciprocal-multiply phase of owned scalar
division now use the same consumed-carrier schedule. The exact multiplier updates
only each lane's rational scale; symbolic lanes retain the general borrowed
arithmetic fallback.

| Operation | Previous current path | Optimized current | Numerica 128 | Hyperreal / Numerica |
| --- | ---: | ---: | ---: | ---: |
| Vec3 mul scalar | 126.30 ns | 91.63 ns | 119.31 ns | 0.77x |
| Vec4 mul scalar | 150.85 ns | 108.07 ns | 158.24 ns | 0.68x |
| Mat3 mul scalar | 412.60 ns | 184.44 ns | 647.22 ns | 0.28x |
| Mat4 mul scalar | 553.19 ns | 305.44 ns | 1.124 us | 0.27x |
| Vec3 div scalar | 382.95 ns | 118.51 ns | 169.34 ns | 0.70x |
| Vec4 div scalar | 438.33 ns | 147.80 ns | 225.42 ns | 0.66x |
| Mat3 div scalar | 1.040 us | 229.99 ns | 787.66 ns | 0.29x |
| Mat4 div scalar | 1.527 us | 381.16 ns | 1.37 us | 0.28x |

Scalar multiplication and division now beat the comparison baseline at every fixed
carrier size. Shared exact divisors retain one reciprocal in Hyperreal's bounded
arithmetic cache. The forward edge owns that result and its reverse edge is weak, so
repeated division presents a stable factor identity to every lane's product cache
without an ownership cycle. The rational-fixture division rows are similarly
118.51--377.44 ns. Mat4 subtraction remained statistically unchanged at 671.87 ns
while the cache layout was varied.

Exact negation now retains one opposite-sign rational beside the existing reciprocal
without enlarging `RationalData` or displacing either linear-result slot. Fixed-size
owned and borrowed vector/matrix kernels make each lane explicit, and borrowed exact
`Real` negation constructs the result directly instead of cloning and then replacing
its rational scale. The clone-inclusive scalar row falls from 57.60 ns to 19.23 ns,
beating Numerica 128 at 20.38 ns and Symbolica at 1.06 us. Matched carrier medians are:

| Operation | Previous Hyperreal | Optimized Hyperreal | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| Vec3 neg (clone-inclusive owned row) | 195.68 ns | 89.59 ns | 50.34 ns | 3.13 us |
| Vec4 neg (clone-inclusive owned row) | 222.00 ns | 108.93 ns | 63.70 ns | 4.02 us |
| Mat3 neg (clone-inclusive owned row) | 775.02 ns | 209.49 ns | 423.07 ns | 9.02 us |
| Mat4 neg (clone-inclusive owned row) | 1.20 us | 357.86 ns | 728.99 ns | 13.98 us |
| Vec3 neg ref | 169.24 ns | 42.82 ns | 46.09 ns | 3.09 us |
| Vec4 neg ref | 214.43 ns | 50.90 ns | 63.57 ns | 4.04 us |
| Mat3 neg ref | 538.23 ns | 118.77 ns | 416.49 ns | 8.43 us |
| Mat4 neg ref | 766.60 ns | 197.26 ns | 748.76 ns | 13.80 us |

Borrowed negation and both matrix ownership forms now beat Numerica at every fixed
carrier size. The main owned vector row still times cloning the reusable fixture inside
the loop, unlike the external engines' reference-to-result API; that 42--47 ns setup
cost is recorded separately instead of being attributed to the negation kernel. The
clone-inclusive owned vector rows still improve by 51--54%, and all optimized rows
remain at least 34 times faster than Symbolica.

## Rejected zero-mask multiplication experiment

Matrix multiplication obtains `Matrix3StructuralFacts` or `Matrix4StructuralFacts`
before entering the sparse borrowed kernel. The experiment passed those facts' retained
zero masks into the kernel and replaced each `Real::definitely_zero()` probe with a bit
lookup. The arithmetic schedule and result construction were otherwise unchanged.

Criterion measurements used exact-rational upper-triangular by lower-triangular products:

| Sentinel | Existing cached zero probes | Retained-mask experiment | Effect |
| --- | ---: | ---: | ---: |
| `sentinel/matrix3/sparse_mask_product` | about 2.862 us | about 2.970 us | 3.8% slower |
| `sentinel/matrix4/sparse_mask_product` | about 4.667 us | about 4.735 us | 1.5% slower |

Dense borrowed multiplication also showed no statistically significant improvement:
the 3x3 sample moved from about 4.940 us to 4.785 us with a confidence interval spanning
no change (`p = 0.55`), while 4x4 moved from about 8.192 us to 8.153 us within noise.
The production experiment was therefore removed completely. The focused benchmark rows
remain because they directly guard the Gustavson-style sparse scheduling surface.

## Retained and cold exact complex multiplication

Complex multiplication now has one canonical component scheduler across all
owned and borrowed operators and integer powers. Exact rationals with retained
reuse evidence use Gauss's three-product identity, exposing stable sums and
products to Hyperreal's arithmetic caches. Isolated values instead enter one
paired scalar reducer that converts all four exact rational components once and
returns both canonical results. Wider or symbolic inputs retain the general
exact product-sum fallback.

The cold reducer recognizes word-sized dyadic and 2/5-smooth decimal
denominators without entering a general GCD. This is exact fraction reduction,
not decimal approximation. General denominators and overflowing word products
still fall through to the arbitrary-precision reducer.

| Workload | Hyperreal f64 | Hyperreal rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| retained owned multiply | 157.85 ns | 162.59 ns | 247.72 ns | 10.11 us |
| retained borrowed multiply | 138.77 ns | 140.11 ns | 235.87 ns | 10.11 us |
| cold varying multiply | 222.77 ns | 280.76 ns | 286.27 ns | 10.24 us |
| integer power five | 679.67 ns | 788.02 ns | 1.2294 us | 45.329 us |

The retained trace records `mul-components-three-product-exact-rational`.
Fresh varying inputs record `mul-components-fused-cold-exact-rational` followed
by the scalar-owned `paired-word-sized` reducer. The cold Criterion group uses
`iter_batched`, so input construction and decimal parsing are excluded from the
measured multiplication interval for every engine.

## Common-scale exact complex division

Owned, borrowed, and checked complex division now share one component helper.
When all four components are exact rationals, Hyperreal forms the conjugate
product and squared norm at common integer scales, then cross-cancels those
scales into the two final fractions. Dyadic inputs align exponents directly;
word-sized general denominators use LCM scaling with identity-GCD elision.
Overflow and wider arbitrary-precision rationals retain the exact general
fallback, while symbolic inputs retain the previous norm/inverse/product-sum
schedule. Exact zero norms still return `DivideByZero`; checked symbolic norms
still reject `UnknownZero`.

| Workload | Hyperreal f64 | Hyperreal rational | Numerica 128 | Symbolica |
| --- | ---: | ---: | ---: | ---: |
| reciprocal | 214.64 ns | 239.19 ns | 276.47 ns | 10.88 us |
| reciprocal checked | 218.43 ns | 248.39 ns | 275.94 ns | 10.89 us |
| complex division | 373.81 ns | 474.95 ns | 615.42 ns | 22.03 us |
| complex division checked | 375.42 ns | 470.50 ns | 632.26 ns | 22.41 us |
| borrowed complex division | 349.79 ns | 457.13 ns | 503.22 ns | 21.84 us |

The exact-rational object trace records
`div-components-fused-exact-rational`; the scalar trace distinguishes
`paired-dyadic-word-sized`, `paired-general-word-sized`, and the
arbitrary-precision fallback.

## Trace evidence

`dispatch_trace.md` was regenerated from the diagnostic build. It confirms that dense
exact-rational public determinant workloads enter
`matrix4-factors-dense-exact-known-rational` and
`determinant4-from-factors-known-rational`, while sparse rational determinant rows
retain `matrix4-factors` and zero pruning. Dense and sparse multiplication, inverse,
affine, diagonal, prepared-cache, and transform paths remain separately observable.
The trace is diagnostic rather than timing evidence because tracing hooks are compiled
into that build.

`api_dispatch_trace.md` complements the Criterion-integrated matrix report with
single-run workloads for the public algebra2, point, projective, and AABB families.
Each row fails if it records only its benchmark marker, preventing a syntactically
present workload from being mistaken for executed lattice/dependency work.

## Reproduction and validation

```sh
cargo bench --bench regression_sentinels -- 'sentinel/matrix3/sparse_mask_product'
cargo bench --bench regression_sentinels -- 'sentinel/matrix4/sparse_mask_product'
cargo bench --bench regression_sentinels -- 'sentinel/matrix4/determinant_fractional'
cargo bench --bench mathbench -- scalar_large_integer_exp
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
cargo bench --bench api_dispatch_trace --features hyperreal-dispatch-trace
cargo fmt --all -- --check
cargo test --locked
cargo check --benches --locked
cargo clippy --all-targets --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
```
