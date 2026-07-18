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
