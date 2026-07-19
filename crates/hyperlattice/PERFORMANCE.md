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

Hyperreal now gives each immutable rational one lazily boxed linear-result slot,
separate from product retention. A shared left operand can retain a sum in its
slot; if that slot is occupied, a directed difference can use the paired
operand's slot. Weak operand keys keep cache identity exact without retaining
the key, serialization ignores the accelerator, and operands with no evidence
of shared ownership skip retention. Direct cold sentinels measured 91.24 ns for
wide-dyadic addition and 89.97 ns for subtraction, while retained rational
operations measured 8.77 ns and 8.99 ns.

Matched facade medians fell from 55.58 ns to 29.68 ns for addition and from
68.41 ns to 31.14 ns for subtraction, reductions of 46.6% and 54.5%.
Hyperreal is now 29.9% faster than Numerica 128 on addition and 30.3% faster on
subtraction; the adjacent multiplication rerun remains 25.3% faster. Trace
coverage executes each linear case twice and records the cold exact route
followed by `rational/linear/retained-sum` or
`rational/linear/retained-difference`, with the corresponding
`real/add|sub/same-symbolic-basis` route.

Borrowed `Vec3` construction is now the next measured carrier gap: addition is
187.50 ns versus Numerica 128's 126.64 ns (1.48x), and subtraction is
221.44 ns versus 139.03 ns (1.59x). The scalar kernels now win, so the next
cycle can isolate vector result construction and ownership overhead.

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
