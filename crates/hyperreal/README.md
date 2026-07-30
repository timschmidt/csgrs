<h1>
  hyperreal
  <img src="./doc/hyper.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperreal` provides exact rational arithmetic, symbolic real values, lazy
computable-real approximation, and conservative certificates for Rust. It is
the scalar layer beneath the Hyper geometry, solver, physics, circuit,
routing, packing, voxel, and design-readiness crates.

The crate does not try to be a complete computer algebra system. Its job is to
preserve exact or refinement-ready structure until a caller can make a
certified decision—or report that the available evidence is insufficient.
Version `0.13.1` is the current crate release documented here.

## Why exact-aware real arithmetic?

Fixed-size floats are excellent interchange and rendering values, but rounding
them can change the sign of a small residual. In geometry or constraint code,
that can change a topology decision. Arbitrary-precision rationals avoid that
rounding, although eager reduction can make intermediate numerators and
denominators grow rapidly. A full symbolic algebra system is often more
machinery than a systems crate needs.

`hyperreal` occupies the useful middle ground:

```text
exact integers, fractions, and finite IEEE-754 values
                         │
                         ▼
                    Rational
                         │
          exact scale + retained expression
                         ▼
                       Real
              ┌──────────┴──────────┐
              ▼                     ▼
      structural certificates   Computable refinement
              │                     │
              └──────────┬──────────┘
                         ▼
             decision or explicit uncertainty
```

Primitive-float export is a named, lossy boundary. It is suitable for display,
I/O, and third-party interoperability, not for equality, ordering, predicates,
or topology.

## Primary types

| Type | Purpose |
| --- | --- |
| `Rational` | Arbitrary-precision exact fraction and the base of exact arithmetic. |
| `Real` | Public exact-aware scalar combining a rational scale with symbolic or computable structure. |
| `Computable` | Lazy expression graph evaluated to a requested binary precision. |
| `RealStructuralFacts`, `RealDetailedFacts` | Conservative facts about sign, zero, magnitude, representation, and domain. |
| `CertifiedRealSign`, `CertifiedRealOrdering`, `CertifiedRealEquality` | Results whose certainty and unresolved cases remain explicit. |
| `RealExactSetFacts` | Set-level summaries used to select dyadic, integer-grid, signed-unit, or shared-denominator schedules. |
| `Simple` | Optional small expression language and calculator surface. |
| `Problem` | Fallible arithmetic, domain, parsing, and refinement error. |

The source guides for [`Rational`](src/rational/README.md),
[`Real`](src/real/README.md), and [`Computable`](src/computable/README.md)
explain their representations in more depth.

## Quick start

Create a project and add Hyperreal:

```sh
cargo new exact-scalars
cd exact-scalars
cargo add hyperreal
```

Replace `src/main.rs` with:

<!-- quickstart:start -->
```rust
use hyperreal::{Problem, Rational, Real, RealSign};

fn main() -> Result<(), Problem> {
    let one_tenth = Rational::fraction(1, 10)?;
    let two_tenths = Rational::fraction(2, 10)?;
    let exact_sum = one_tenth + two_tenths;
    assert_eq!(exact_sum, Rational::fraction(3, 10)?);

    let diagonal = Real::new(Rational::new(2)).sqrt()?;
    assert_eq!(diagonal.refine_sign_until(-64), Some(RealSign::Positive));

    println!("sqrt(2) ≈ {:.12}", diagonal.to_f64_lossy().unwrap());
    Ok(())
}
```
<!-- quickstart:end -->

Run it:

```sh
cargo run
```

The same program is checked in as
[`examples/readme_quickstart.rs`](examples/readme_quickstart.rs), compiled by
the test suite, and compared byte-for-byte with this README block.

## API guide

### Construct and inspect exact rationals

| Task | API |
| --- | --- |
| Constants and integers | `Rational::zero`, `one`, `new`, `from_bigint` |
| Fractions | `fraction`, `from_bigint_fraction`, `inverse` |
| Parse exact text | `str::parse::<Rational>` for integers, decimals, and fractions |
| Import a finite float exactly | `Rational::try_from(f32)`, `Rational::try_from(f64)` |
| Inspect a value | `numerator`, `denominator`, `sign`, `is_zero`, `is_one`, `is_integer`, `is_dyadic`, `same_denominator` |
| Integer/fraction parts | `trunc`, `fract`, `to_big_integer` |
| Powers and roots | `powi`, `extract_square_reduced`, `perfect_nth_root`, `is_perfect_power` |

`Rational` implements the usual owned and borrowed arithmetic operators.
Finite float import decodes the exact IEEE-754 value; it does not reinterpret
the input as a decimal measurement.

### Construct and combine real values

| Task | API |
| --- | --- |
| Constants | `Real::zero`, `one`, `integer`, `pi`, `tau`, `e` |
| Lift an exact value | `Real::new`, integer and finite-float `From`/`TryFrom` implementations |
| Aggregate | `sum_owned`, `sum_refs`, `mean`, `sample_stddev`, `min`, `max` |
| Common transforms | `abs`, `inverse`, `inverse_ref`, `to_radians`, `to_degrees` |
| Certified integer operations | `floor_certified`, `ceil_certified`, `round_certified`, `trunc_certified`, `fract_certified`, `rem_euclid_certified` |

`Real` implements arithmetic operators while retaining recognized exact,
symbolic, and computable forms. `PartialEq` is structural and is deliberately
not a complete proof of mathematical equality.

### Elementary and stable functions

All entries below are methods on `Real`:

| Family | Methods |
| --- | --- |
| Roots and powers | `sqrt`, `cbrt`, `root_n`, `powi_i64`, `powi`, `pow_rational`, `pow` |
| Exponential and logarithmic | `exp`, `expm1`, `ln`, `ln_1p`/`log1p`, `ln_1m`/`log1m`, `log2`, `log10` |
| Stable probability transforms | `softplus`, `sigmoid`, `logit`, `logaddexp`, `logsubexp` |
| Trigonometric | `sin`, `cos`, `tan`, `sin_pi`, `cos_pi`, `tan_pi`, `sinc`, `sinc_pi`, `cosc` |
| Inverse trigonometric | `asin`, `acos`, `atan`, `atan2` |
| Hyperbolic | `sinh`, `cosh`, `tanh`, `asinh`, `acosh`, `atanh` |
| Cancellation helpers | `sqrt1pm1`, `sqrt1m1`, `hypot_minus` |
| Fused algebra | `mul_add`, `sum_products`, `diff_of_products`, `eval_poly`, `eval_rational_poly` |
| Vector-length helpers | `hypot2`, `hypot3`, `dot2_refs`, `dot3_refs`, `dot4_refs` |

Domain errors are returned as `Problem`; these operations do not silently
substitute a primitive-float result.

### Probability and special functions

| Family | Methods |
| --- | --- |
| Error and normal core | `erf`, `erfc`, `erfcx`, `dnorm`, `pnorm`, `normal_sf`, `pnorm_upper` |
| Stable normal logs and intervals | `log_dnorm`, `log_pnorm`, `log_normal_sf`, `normal_interval`, `pnorm_diff` |
| Inverses and parameterized normal | `erfinv`, `erfcinv`, `qnorm`, `qnorm_upper`, `normal_pdf`, `normal_cdf`, `normal_survival`, `normal_quantile` |
| Normal ratios and moments | `normal_mills`, `normal_hazard`, `normal_log_hazard`, `normal_inverse_mills`, `standard_normal_moment`, `normal_interval_moment` |
| Gaussian derivatives and truncation | `hermite_probabilists`, `dnorm_derivative`, `gaussian_derivative`, `truncated_normal_mean`, `truncated_normal_variance` |
| Gamma and beta | `gamma`, `lgamma`, `beta`, `ln_beta`/`lbeta`, `regularized_beta`, `regularized_beta_q` |
| Gamma distributions | `regularized_gamma_p`, `regularized_gamma_q`, `chi_square_cdf`, `chi_square_sf` |

These functions have deliberately documented domains and, in some cases,
bounded implemented parameter ranges. Consult their rustdoc before using them
as a general-purpose special-function library.

### Ask for facts and certificates

| Task | API |
| --- | --- |
| Cheap object-independent facts | `definitely_zero`, `definitely_one`, `zero_or_one`, `zero_one_or_minus_one`, `structural_facts`, `detailed_facts` |
| Recover exact content | `exact_rational`, `exact_rational_ref`, `is_exact_dyadic_rational`, `exact_set_facts` |
| Check a function domain | `domain_facts`, `reciprocal_domain`, `sqrt_domain`, `log_domain`, `asin_acos_domain`, `acosh_domain`, `atanh_domain` |
| Refine a sign | `zero_status`, `refine_sign_until`, `certified_sign_until` |
| Compare with bounded refinement | `certified_eq_until`, `certified_cmp_until`, `certified_dyadic_interval` |

Higher layers should first use retained facts, then request bounded refinement
only at a decision boundary. An unresolved certificate is meaningful; it must
not be replaced with an epsilon comparison.

### Work directly with computable reals

| Task | API |
| --- | --- |
| Constants and exact input | `Computable::zero`, `one`, `pi`, `tau`, `e`, `rational` |
| Expression nodes | `negate`, `inverse`, `square`, `multiply`, `add`, `sqrt`, `exp`, `expm1`, `ln`, trig and inverse/hyperbolic methods |
| Approximation | `approx`, `approx_signal` |
| Decisions | `structural_facts`, `zero_status`, `sign_until`, `try_compare_to_until`, `try_compare_to` |
| Cancellation | `abort` |

`Computable::approx(p)` returns a scaled integer approximation at binary
precision `p`; it does not promise a fixed decimal digit count.
`Computable::sign` is a compatibility best-effort query whose `NoSign` result
also means unresolved, and `compare_absolute` reports tolerance overlap as
`Equal`; neither is an exact decision API. Partial-function expression
constructors such as `inverse`, `sqrt`, and `ln` assume their mathematical
domains. Prefer the checked `Real` operations at external decision boundaries.

### Advanced exact reducers

Geometry and linear-algebra crates can use the public determinant filters,
signed-product sums, dot/linear/affine combinations, exact dyadic-line
carriers, and certified determinant-sign helpers re-exported from the crate
root. These are advanced stack-building APIs rather than ordinary scalar
entry points; their complete signatures and invariants are maintained in
rustdoc.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `simple` | no | Enables `Simple` and the `hyperreal` calculator binary. |
| `serde` | no | Enables JSON and CBOR serialization APIs and serializable expression types. |
| `cached-f32-approx` | no | Caches selected `f32` approximation paths. |
| `cached-f64-approx` | no | Retains selected `f64` views across exact-rational clones. |
| `dispatch-trace` | no | Records internal dispatch and rational-growth counters for development. |

With the optional expression language:

```toml
[dependencies]
hyperreal = { version = "0.13.1", features = ["simple"] }
```

The user-facing `Simple` API is `FromStr`, `Simple::evaluate`, and
`Simple::value`. Its grammar exposes the corresponding arithmetic,
elementary, stable, polynomial, probability, and special-function `Real`
operations. Run expressions with:

```sh
cargo run --features simple -- "(sqrt (/ 49 64))"
```

With `serde`, use `serde::to_json`, `from_json`, `to_bytes`, and `from_bytes`.
Serialization records expression structure, not transient caches or abort
signals.

## Guarantees and boundaries

- Integer, fraction, decimal-text, and finite-float imports preserve their
  exact represented value.
- `NaN` and infinities are rejected. IEEE negative zero imports as exact zero.
- Scientific notation is not an exact text format in the current parser.
- `Real::to_f32_lossy` and `to_f64_lossy` return `None` when no finite
  approximation can be produced.
- Structural equality is not algebraic equality. Use exact extraction or a
  certification API when a branch depends on the answer.
- Some real-number questions cannot be decided within a bounded refinement
  budget; APIs preserve that uncertainty.
- Hyperreal owns scalar facts. Coordinate, matrix, geometry, and topology facts
  belong to higher crates.

## Ecosystem and further documentation

[Hyperlattice](https://github.com/timschmidt/hyperlattice) builds exact-aware
points, vectors, and matrices over `Real`.
[Hyperlimit](https://github.com/timschmidt/hyperlimit) turns scalar and lattice
facts into certified geometric predicates. Higher geometry crates own curves,
triangulations, meshes, boundary representations, and CSG grammar.

- [`src/README.md`](src/README.md) introduces the source architecture.
- [`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
  optimization evidence.
- [`benchmarks.md`](benchmarks.md) contains generated benchmark summaries.
- Generate the complete API reference with `cargo doc --open`.

## References

- Boehm, Hans-Juergen, Robert Cartwright, Mark Riggle, and Michael J.
  O'Donnell. “Exact Real Arithmetic: A Case Study in Higher Order
  Programming.” *LFP '86*, 1986, pp. 162–173.
  [doi:10.1145/319838.319860](https://doi.org/10.1145/319838.319860).
- Boehm, Hans-J. “Towards an API for the Real Numbers.” *PLDI 2020*,
  pp. 562–576.
  [doi:10.1145/3385412.3386037](https://doi.org/10.1145/3385412.3386037).
- Brent, Richard P., and Paul Zimmermann. *Modern Computer Arithmetic*.
  Cambridge University Press, 2010.
  [doi:10.1017/CBO9780511921698](https://doi.org/10.1017/CBO9780511921698).
- Johansson, Fredrik. “Efficient Implementation of Elementary Functions in the
  Medium-Precision Range.” *ARITH 2015*, pp. 83–89.
  [arXiv:1410.7176](https://arxiv.org/abs/1410.7176).
- Payne, Mary H., and Robert N. Hanek. “Radian Reduction for Trigonometric
  Functions.” *ACM SIGNUM Newsletter*, vol. 18, no. 1, 1983, pp. 19–24.
  [doi:10.1145/1057600.1057602](https://doi.org/10.1145/1057600.1057602).
- Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*,
  vol. 18, 1997, pp. 305–363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational Geometry*,
  vol. 7, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

Boehm and colleagues establish the computable-real model and API concerns;
Brent, Zimmermann, Johansson, and Payne describe the multiple-precision
elementary-function techniques; Shewchuk and Yap motivate exact decision
boundaries for downstream geometry.

## Acknowledgements

Hyperreal descends from Nick Lamb's
[`realistic`](https://github.com/tialaramex/realistic) project. Jason Siefken's
contributions improved both `realistic` and Hyperreal. The repository history
also records contributions from Timothy Schmidt, TimTheBig, Sambit Nayak, and
the other credited commit authors.

## License and contributing

Hyperreal is distributed under the Apache License 2.0. Upstream-derived files
retain their applicable copyright and license notices; see
[`LICENSE.txt`](LICENSE.txt) and [`LICENSE-MIT`](LICENSE-MIT).

Changes should keep lossy conversions explicit and preserve uncertainty at
decision boundaries. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
```
