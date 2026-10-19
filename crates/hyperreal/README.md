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
Version `0.13.1` describes the unreleased source in this checkout.

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

From the repository root, run the checked-in example for this source version:

```sh
cargo run -p hyperreal --example readme_quickstart
```

The example is:

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

The program is checked in as
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
| Iterator products | owned or borrowed `Iterator::product::<Rational>()` |

`Rational` implements the usual owned and borrowed arithmetic operators.
Finite float import decodes the exact IEEE-754 value; it does not reinterpret
the input as a decimal measurement.

### Construct and combine real values

| Task | API |
| --- | --- |
| Constants | `Real::zero`, `one`, `integer`, `pi`, `tau`, `e` |
| Lift an exact value | `Real::new`, integer and finite-float `From`/`TryFrom` implementations |
| Aggregate | owned or borrowed `Iterator::product::<Real>()`, `sum_owned`, `sum_refs`, `mean`, `sample_stddev`, `min`, `max` |
| Common transforms | `abs`, `inverse`, `inverse_ref`, `to_radians`, `to_degrees` |
| Integer operations | total `near_integer`; certified directional `floor_certified`, `ceil_certified`, `round_certified`, `trunc_certified`, `fract_certified`, `rem_euclid_certified` |

`Real` implements arithmetic operators while retaining recognized exact,
symbolic, and computable forms. `PartialEq` is structural and is deliberately
not a complete proof of mathematical equality. When a caller specifically
needs to recognize opposite rational scales over one shared symbolic basis,
`is_structural_negation_of` certifies that relation without allocating a
temporary negated `Real`; `false` remains inconclusive for general identities.

Long `Real` iterator sums are combined through a streaming balanced tree once
their size hint proves enough terms, while short and size-unknown sums retain
the cheaper sequential construction path. Long homogeneous rational or
symbolic prefixes collapse to one scaled value before any balancing work.
Owned sums move their values into that reducer; borrowed sums clone only after
the proven-long path is selected.

Square roots of recognized quadratic surds also stay exact when the conjugate
norm is a nonnegative rational square. For example,
`sqrt(x + y + 2*sqrt(x*y))` reduces to a sum of rational square roots for
positive rational `x` and `y`; `certified_eq_until` can then prove equality
with `sqrt(x) + sqrt(y)` even when structural `PartialEq` sees the two terms in
opposite orders.

### Elementary and stable functions

All entries below are methods on `Real`:

| Family | Methods |
| --- | --- |
| Roots and powers | `sqrt`, `cbrt`, `root_n`, `powi_i64`, `powi`, `pow_rational`, `pow` |
| Exponential and logarithmic | `exp`, `exp2`, `exp10`, `expm1`, `ln`, `ln_1p`/`log1p`, `ln_1m`/`log1m`, `log2`, `log10` |
| Stable probability transforms | `softplus`, `sigmoid`, `logit`, `logaddexp`, `logsubexp` |
| Trigonometric | `sin`, `cos`, `tan`, `cot`, `sin_pi`, `cos_pi`, `tan_pi`, `cot_pi`, `sinc`, `sinc_pi`, `cosc` |
| Inverse trigonometric | `asin`, `acos`, `atan`, `atan2` |
| Hyperbolic | `sinh`, `cosh`, `tanh`, `asinh`, `acosh`, `atanh` |
| Cancellation helpers | `sqrt1pm1`, `sqrt1m1`, `hypot_minus` |
| Fused algebra | `mul_add`, `sum_products`, `diff_of_products`, `eval_poly`, `eval_rational_poly` |
| Vector-length helpers | `hypot2`, `hypot3`, `dot2_refs`, `dot3_refs`, `dot4_refs` |

Domain errors are returned as `Problem`; these operations do not silently
substitute a primitive-float result.
`cot` and `cot_pi` return `Problem::NotANumber` at certified integer-pi poles;
an opaque denominator that bounded refinement cannot separate from zero
returns `Problem::UnknownZero`.

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
| Cheap object-independent facts | `definitely_zero`, `definitely_one`, `zero_or_one`, `zero_one_or_minus_one`, `immediate_equality`, `immediate_sign`, `structural_facts`, `detailed_facts` |
| Recover exact content | `exact_rational`, `exact_rational_ref`, `is_exact_dyadic_rational`, `exact_set_facts` |
| Check an opposite structural scale | `is_structural_negation_of` |
| Check a function domain | `domain_facts`, `reciprocal_domain`, `sqrt_domain`, `log_domain`, `asin_acos_domain`, `acosh_domain`, `atanh_domain` |
| Refine a sign | `zero_status`, `refine_sign_until`, `certified_sign_until` |
| Compare with bounded refinement | `certified_eq_until`, `certified_cmp_until` |
| Enclose while retaining exact rational points | `certified_rational_interval` |
| Reuse an existing enclosure without refinement | `cached_rational_interval` |
| Enclose on a dyadic grid for interval arithmetic | `certified_dyadic_interval` |

`immediate_equality` consumes already known rational structure. `Some(true)`
proves equality and `Some(false)` proves inequality; `None` is inconclusive,
including for rational pairs that require wider arithmetic. It does not
construct a difference or arbitrary-width cross products. Symbolic inputs
keep their existing proof paths. Borrowed digit comparisons still depend on
input size, so this query does not supply a constant-time resource bound.

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
`Computable::sign_until` returns `None` when bounded refinement cannot prove a
sign, and `compare_absolute` reports tolerance overlap as `Equal`; neither
turns an unresolved comparison into an exact decision. Partial-function expression
constructors such as `inverse`, `sqrt`, and `ln` assume their mathematical
domains. Prefer the checked `Real` operations at external decision boundaries.
Supported rational-radical graphs may additionally receive a bounded
algebraic-norm zero certificate; unsupported graphs preserve the same `None`
result and caller-supplied precision floor.

### Checked scientific output

`Real::try_scientific` returns rounded scientific text or a typed failure. It
preserves the requested significant-digit precision and exponent case. Exact
halfway values round to an even last digit; non-rational values require both
certified enclosure endpoints to round to the same text.

```rust
use hyperreal::{Real, ScientificCase};

let value: Real = "1/8".parse().unwrap();
assert_eq!(
    value.try_scientific(6, -128, ScientificCase::Lower).unwrap(),
    "1.250000e-1"
);
```

An enclosure search that cannot separate zero returns `Problem::UnknownZero`.
Unresolved rounding, aborts and resource limits return `Problem::Exhausted`.
The precision floor limits requests made by this method, not every operation
inside an underlying expression. See its rustdoc for exact resource guards.
The returned decimal is a presentation value, not an exact serialization.

The existing formatting traits keep their behavior, including potentially
expensive leading-bit searches for unresolved zeros. Use the checked method
when those failures must be handled explicitly. `Real::fixed_decimal` remains
a separate view with absolute fractional-digit precision.

`Real::try_decimal` applies the same checked rounding contract to adaptive
output: magnitudes in `[2^-19, 2^60)` use fixed decimal places, while other
values use lowercase scientific notation. Its notation choice is independent
of cache history. At the window boundaries it may differ from the older
alternate formatter's leading-bit estimate.

For human diagnostics, `Real::diagnostic_decimal(min_precision)` returns a
borrowed display view. It respects the requested decimal precision and prints
explicit nonnumeric failure markers such as `[UnknownZero]` or `[Exhausted]`.
Use `try_decimal` to handle the typed error in code; diagnostic text must never
be used as numeric input to exact geometry or machine output.

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

With `serde`, use `Real::to_json`, `from_json`, `to_bytes`, and `from_bytes`.
Serialization records expression structure, not transient caches or abort
signals. An opaque scalar without its required computable payload is rejected
by Serde and returns `Problem::ParseError` through the convenience readers.
Validation of mathematical domains inside present expression graphs remains
incomplete.

## Guarantees and boundaries

Scalar certificates and refinement caches follow the stack-wide
[admission protocol](../../docs/admission.md), including exact construction,
honest structural facts, typed domain errors, and monotone refinement.

Verus verification is in progress. Production arithmetic kernels and an
unbounded rational model have checked proofs; the full crate is **not yet
formally verified**. See the [proof commands, current boundary, and remaining
obligations](verification/README.md).

- Integer, fraction, decimal/scientific-text, and finite-float imports preserve
  their exact represented value.
- `NaN` and infinities are rejected. IEEE negative zero imports as exact zero.
- Scientific notation is parsed directly rather than through floating point.
  `Rational` returns `Problem::Exhausted` before a short exponent-only input can
  force excessive eager storage; `Real` retains such extreme powers lazily.
- `Real::to_f32_lossy` and `to_f64_lossy` return `None` when no finite
  approximation can be produced.
- Structural equality is not algebraic equality. Use exact extraction or a
  certification API when a branch depends on the answer.
- Some real-number questions cannot be decided within a bounded refinement
  budget; APIs preserve that uncertainty.
- Hyperreal owns scalar facts. Coordinate, matrix, geometry, and topology facts
  belong to higher crates.

### Serialized certificate admission

With `serde`, JSON and CBOR retain the symbolic class, its rational scale and
the optional computable expression. Named classes are certificates: their
parameters must justify the sign, nonzero and irrationality facts they claim.
A supplied expression must also be proved equal to that certificate. The
decoder preserves accepted expression graphs and their serialized form.

`Real::from_json`, `Real::from_bytes` and conversions from CBOR `Value` return
`Problem::ParseError` for a proved invalid certificate. They return
`Problem::UnknownCertificate` when a claim or payload relationship remains
unproved. Generic Serde containers carry this diagnostic in the format's
deserialization error. Neither outcome publishes a `Real`, including when
its outer rational scale is zero.

This tightens earlier imports that trusted named tags without checking their
parameters or payloads. Applications must handle undecided imports explicitly;
an unsupported equality proof does not imply invalid mathematics. Re-export a
value constructed through the public scalar API to obtain its native
certificate and payload. Do not remove a supplied payload to bypass validation:
that would discard the relationship being checked. Valid lazy certificates
remain supported. The wire layout is unchanged; zero-pi-power constant products
now reconstruct with the mathematical identity `pi^0 = 1`.

Payload checks first compare the native recipe structurally, then use the
shared positive-product proof for powers, roots, reciprocals and exponentials.
That proof visits at most 128 nodes, retains at most 16 atoms and limits
coefficients to 1024 bits. It checks definedness before cancellation and does
not refine an untrusted payload. Noncanonical positive sums may use refinement
through 256 fractional bits on a recipe built from checked parameters.
These limits do **not** bound total decoding work, parameter sizes or graph
construction. General opaque-graph domain validation and comprehensive import
resource limits remain incomplete; externally contain hostile-input workloads.

Private trigonometric wire variants retain reduced parameters. The dedicated
`PrescaledCosRational`, `PrescaledSinRational` and `PrescaledTanRational` kernels
require `|r| < 1`; their half-pi-complement variants require `1 <= r < 3/2`.
Imports outside those stored parameter ranges return `ParseError`, including
when a zero outer factor would hide the invalid child. Small-angle `sinc` and
`cosc` nodes reject exact rational inputs outside `|x| <= 3/4` and shared
constants whose certified magnitude proves they exceed that bound. This
tightens earlier unchecked private representations; the public trigonometric
constructors retain their wider input domains through argument reduction.
The domain and small-kernel checks also recognize defined rational expressions
using the shared quotient evaluator: addition, multiplication, squares,
negation, powers-of-two scaling, three-term linear combinations and reciprocals
with proved nonzero denominators. For example, `inverse(1 + (-1))` now fails
import before multiplication by zero or a named certificate can conceal it.
Such invalid graphs previously accepted, or reported as `UnknownCertificate`,
now return `ParseError`. Valid graphs retain their wire representation.
This optional proof uses the existing limits of 512 nodes, depth 64, 8192
operations and 4096 coefficient bits; existing exact-leaf checks still apply
to larger leaves. Both operands are checked even under a zero factor. An
opaque operand or exhausted proof budget leaves the domain unproved; acceptance
does not certify an arbitrary graph's domain. General expression domains,
kernel ranges and whole-input resource limits remain incomplete.
Inverse-cosine endpoint facts distinguish `acos(1) = acosh(1) = 0`; an imported
`acosh` node needs evidence about its argument before claiming a positive sign.
The native rational reader recognizes these exact endpoints so later arithmetic
can consume them without numerical refinement. For `acosh`, the shared bounded
quotient evaluator requires rational leaves and checks every denominator before
discarding zero terms; an arbitrary cached zero is insufficient. Imported
non-rational `acosh` arguments may remain undecided until new evidence is found.

Normal-quantile seeds are evaluation hints, including in retained wire graphs.
Recognizable CDF expressions of exact rational arguments retain their inverse
without evaluating tiny tails: the shared scalar normalizer suggests an argument,
and structural equality with the complete reconstructed CDF proves the identity.
Seeds no longer supply certified values by themselves. Other results require
CDF enclosures, with monotone search when a Newton step or seed is insufficient.
The exact half-probability identity remains zero through import and downstream
arithmetic. Valid stored graph bytes are preserved; a native constructor may
canonicalize the exact half-probability case to zero and recognized rational
CDF inverses to their exact values. The native quantile
operation requires a defined probability strictly between zero and one;
general expression-domain validation and total-work bounds remain incomplete.

Hypercurve's exact SVG import exposes undecided scalar admission as
`SvgError::Scalar(Problem::UnknownCertificate)` with the scalar error as its
source. Consumers matching every `SvgError` variant must handle the new
`Scalar` variant. CSGRS classifies this scalar cause as `Undecided`.

## Ecosystem and further documentation

[Hyperlattice](https://github.com/timschmidt/hyperlattice) builds exact-aware
points, vectors, and matrices over `Real`.
[Hyperlimit](https://github.com/timschmidt/hyperlimit) turns scalar and lattice
facts into certified geometric predicates. Higher geometry crates own curves,
triangulations, meshes, boundary representations, and CSG grammar.

- [`src/README.md`](src/README.md) introduces the source architecture.
- [`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
  optimization evidence.
- [`COVERAGE.md`](COVERAGE.md) defines the exhaustive representation matrix,
  feature matrix, fuzzing, and executable-line measurement.
- [`benchmarks.md`](benchmarks.md) contains generated benchmark summaries.
- Generate the complete API reference with `cargo doc --open`.

The Cargo package includes the source, examples, tests, regression fixtures,
and checkpoint summaries. Full validation replay bundles under
`benchmarks/checkpoints/*.tar.gz` remain in the repository and are excluded
from the package; use a repository checkout to replay those archived runs.

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
