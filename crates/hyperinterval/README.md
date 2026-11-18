# Hyperinterval

Scalar sets with exact endpoint semantics for the Hyper stack. The current
surface provides checked integer/fixed arithmetic, exact rational intervals,
budgeted common-scale dyadic arithmetic, symbolic Real balls, and finite IEEE
enclosures on x86_64 and wasm32-unknown-unknown. Empty sets, sign sets, and set
relations are explicit.
Elementary function enclosures remain under development.

With `dyadic` and either float feature, `DyadicInterval::from_f32` and
`DyadicInterval::from_f64` import finite IEEE values exactly, including
subnormals. Both reject NaN and infinity and reserve work for the input
significand before allocation. A normal binary32 input needs 24 magnitude bits
and 48 work units. This conversion needs
no strict-float platform qualification or Hyperreal dependency. `0.1_f64` remains
its binary value, not 1/10; importing numerical estimates does not certify their
relationship to an earlier source.

With `dyadic` and `float-enclosure`, `F32Enclosure::from_dyadic` and
`F64Enclosure::from_dyadic` return tight outward finite IEEE bounds. The borrowed
exact source remains available: off-grid endpoints widen, tiny values enclose
between zero and the minimum subnormal, and finite overflow is a typed error.
Both endpoints consume one caller-owned dyadic work/size budget. This enables
exact IEEE → dyadic arithmetic → explicit IEEE enclosure chains without native
floating arithmetic. The conversion uses allocation; float-only arithmetic does
not acquire that dependency.

Finite integer, fixed, rational, dyadic and qualified IEEE carriers provide
exact `abs`, `min_image` and `max_image`. Min/max are images over independently
chosen operand members, so either empty operand gives empty. Absolute value
retains zero when its input crosses zero; a signed primitive minimum that
cannot be reflected returns Overflow. Dyadic methods use the existing shared
size/work budget. These total real functions do not clip their input domain.

Fixed-point and qualified IEEE intervals provide `sqrt()` with tight outward
bounds and explicit domain coverage. They use bounded primitive integer
arithmetic without allocation. Fixed-point upper rounding can return Overflow
at maximal fractional scales. IEEE square roots include subnormal inputs and
retain the certified backend's platform qualification gate.

With `dyadic`, finite intervals, extended intervals and two-component unions
provide `overlaps(&other, &mut budget)` and `disjoint(&other, &mut budget)`.
These queries include touching endpoints and preserve gaps in split results.
Empty inputs are disjoint from every set. Finite endpoint comparisons share
the caller’s budget and can decide separation across the full exponent range
without constructing a common-grid intersection.

With `dyadic`, `DyadicInterval::bisect(&mut budget)` returns two closed halves
sharing the exact midpoint. Empty and singleton inputs return `Ok(None)`. An
unrepresentable midpoint on the minimum exponent grid returns `Overflow`.

Rational and dyadic intervals provide `sqrt(precision, &mut budget)`, returning
`RangeEnclosure<DyadicInterval>` with tight outward endpoints on the grid
`2^precision`. Rational inputs require both features. Negative source points
are excluded with `Partial` coverage; strictly negative nonempty inputs return
`EmptyDomain`. Empty inputs return empty with `Complete` coverage. Keep the
coverage receipt when consuming the result in later arithmetic.

Rational and dyadic intervals also provide `exp(precision, &mut budget)` with
tight outward dyadic endpoints. Its domain is complete for every input, with
empty mapped to empty. Certified Taylor tails, directed range reconstruction
and endpoint refinement share the same work/size budget. A failed attempt
publishes no partial image; a new budget permits a fresh retry. This path
provides deterministic work accounting, not cancellation or actual heap-byte
limits.

`ExtendedInterval<I>` adds explicit closed rays and the entire real line over
sealed finite real backends. A ray requires a singleton finite bound; infinity
is never a member or a floating endpoint. Dyadic extended carriers support
exact hull, intersection, inclusion, membership, negation, validated replay,
and outward exponentiation. Primitive-backed carriers remain allocation-free
and Copy when their backend is Copy.

Rational and dyadic `log(precision, &mut budget)` returns an extended dyadic
image. Positive inputs have Complete coverage. Inputs reaching zero from above
have a lower-unbounded image and Partial coverage; wholly nonpositive inputs
return EmptyDomain. Keep this receipt when applying later operations. Finite
extraction of an unbounded result returns UnboundedImage.

Rational, dyadic and extended dyadic `atan(precision, &mut budget)` returns a
finite dyadic enclosure with Complete local coverage. It preserves exact zero,
uses certified limits at unbounded ends, and supports extreme dyadic scales
without requiring magnitude-sized integers. The same work budget covers all
series and refinement attempts. Keep earlier domain receipts when chaining
operations; for example, `log` followed by `atan` can still have inherited
Partial coverage.

Dyadic `asin`, `acos`, `acosh` and `atanh` use the same shared work budget and
retain Complete/Partial coverage for their real domains. The first three return
finite outward enclosures; `atanh` retains explicit unbounded limits at -1 and 1.
Inputs with no defined member return EmptyDomain. These composed enclosures are
not promised tight. Exact zero special values and empty inputs are preserved.

Rational, dyadic and extended dyadic `sin`/`cos` return outward dyadic images
with Complete local coverage. They certify phase reduction and include
interior extrema. A full period or unbounded connected source produces an
outward grid enclosure of the whole function range; narrow inputs with huge phases require enough size/work
budget to resolve those phases. The original exact endpoints remain the source
through refinement. No native floating transcendental supplies a certificate.

Rational, dyadic and extended dyadic `tan`/`cot` return closed split images with
certified pole incidence and domain coverage. Tangent on [1,2] retains two rays;
cotangent on [0,1] retains its one-sided unbounded image. Zero-only cotangent
returns EmptyDomain. Empty sources succeed with Complete coverage, while any
excluded pole gives Partial coverage. A full branch or unbounded connected
source produces the entire line. Coarse outward rounding can merge split
components. An unresolved numerical denominator triggers refinement or typed
exhaustion; it never establishes a pole or whole-range result.

Rational, dyadic and extended dyadic `pow_rational(&numerator, &denominator,
precision, &mut budget)` accepts arbitrary-size integer exponent components.
It reduces the exponent before selecting its real domain: odd denominators
admit negative bases, even denominators exclude them, and negative powers
exclude zero. A zero exponent uses the constant-one convention, including at
zero, with outward grid rounding. A zero denominator is invalid even for empty
sources. Exact integer relations certify each bound; perfect rational roots
remain compact. Split results retain poles and Partial domain receipts. Large
degrees or fine grids can exhaust the declared size/work budget.

`IntervalSet2<I>` retains up to two normalized closed components without a
separate union allocation. Dyadic construction and replay validate order and
separation; queries preserve real gaps. Extended dyadic `div` and `reciprocal`
return this carrier with domain coverage, including both branches around zero.
A zero-only denominator returns EmptyDomain. Existing finite division stays
strict. Outward rounding or closure can merge components at zero; infinity is
never a member. Split results support negation, membership, inclusion, an
explicit connected hull and further exponentiation. Keep earlier Partial
receipts when chaining operations.

Hyperinterval encloses. Hyperlattice carries set-valued linear structure.
Hyperlimit decides. Hypersolve certifies. Hyperdynamics evolves. Domain crates
assign meaning.

<!-- quickstart:start -->
```rust
use hyperinterval::{FixedInterval, IntervalError, SignSet};

fn main() -> Result<(), IntervalError> {
    type Q30Bounds = FixedInterval<i32, 30>;
    let range = Q30Bounds::new(-(1 << 29), 1 << 29)?;
    assert_eq!(range.signs(), SignSet::Mixed);
    assert!(range.contains_point(0)); // possible zero is not a scalar equality proof
    assert_eq!(range.square()?.bounds(), Some((0, 1 << 28)));
    assert_eq!(Q30Bounds::new(1, 0), Err(IntervalError::ReversedEndpoints));
    Ok(())
}
```
<!-- quickstart:end -->

Enable `fixed` for that example. `integer` enables exact discrete ranges of
all signed and unsigned primitive integer widths. Fixed endpoints represent
`bits / 2^FRAC`; i32 and i64 are supported with `FRAC` less than the word size.
Integer ranges are discrete sets; fixed intervals contain every real point
between their endpoints. Neither implements scalar ordering.

Integer arithmetic uses checked sum, difference, negation, product hull,
square and nonnegative integer powers. Integer multiplication can have gaps;
the result is the smallest containing integer range. Division is not silently
interpreted as truncated real arithmetic. Exact width/radius and explicitly
rounded midpoints support every primitive integer width, including i128/u128. Fixed and integer midpoints share the same exact numerator kernel;
fixed widths retain the full unsigned endpoint distance.
`scale` returns the smallest integer hull of multiplication by one exact
scalar. `affine(factor, offset)` encloses the integer image in a single step
and checks the final endpoints. For example, doubling the i8 range [62, 65]
and adding −8 gives [116, 122], even though the intermediate 130 exceeds i8.
With either `integer` or `fixed`, `integer_quotient_bounds` returns the
integer floor and ceiling of an i128 ratio, with typed zero-denominator and
overflow errors; it does not replace the ratio by a discrete integer set.

Fixed products and fused arithmetic use i128 intermediates. Products, quotients, reciprocals,
squares, ratio ingress and fused affine images round outward; representational
overflow is an error, never saturation. Integer powers use a bounded sequence
of outward-rounded products and may be wider than the tight hull. Storage
conversion is exact or fails; scale coarsening rounds outward. Strict division
rejects any zero-containing denominator, even with an empty numerator.
`div_integer` divides by an exact signed integer without requiring that integer
to fit as a fixed-point value. It retains the source grid, rounds outward, and
rejects zero even for an empty numerator.

With `fixed,dyadic`, `to_dyadic(&mut budget)` converts the two lattice
numerators at their shared exponent exactly. With `fixed,rational`,
`to_rational()` converts the same endpoints directly using the denominator
`2^FRAC`. Both preserve empty intervals and support all valid i32/i64 scales.

No default feature enables allocation, `std`, Hyperreal or serialization.
The `integer` and `fixed` builds are allocation-free `no_std` and target Rust
1.88. Optional `serde` uses null for empty and an ordered endpoint pair for
nonempty sets, with scale and endpoint validation on import. This is an
initial encoding, not a migration of any domain's canonical evidence.

The `rational` feature adds `RationalInterval` over Hyperreal's exact rationals
and requires Rust 1.95. It supplies exact sum, difference, product hull,
square, strict quotient, affine images and positive arithmetic helpers. The
`checked_pow(i64)` operation gives exact signed-integer images, rejecting zero
in a negative-power domain. It inherits Hyperreal's eager-growth guard; scalar
refusals retain the typed `Problem` in `IntervalError::Scalar` and `source()`.
This guard is not a cumulative work, memory-byte or cancellation budget. The
`dyadic` feature is allocation-enabled `no_std` on Rust 1.88: its BigInt
mantissas share an exponent, with explicit budgets for arithmetic and scale
changes. Common-grid arithmetic, set operations and comparisons keep exact
zero on the other operand's grid; zero copies consume work without reserving
bits for a shift. An interval that merely contains zero still uses ordinary
alignment accounting. `hyperreal` enables both carriers and their certified Real bridge;
`real-ball` adds exact symbolic centers with nonnegative rational radii.

Dyadic width, midpoint and radius return exact singleton intervals, or empty
for an empty input. They never round an unrepresentable half. Pointwise square
preserves dependence across zero, while multiplication treats its operands as
independent sets. Reciprocal accepts an explicit output grid and rejects inputs
containing zero. These operations share the caller's size and work budget.
Nonnegative integer powers compute the exact image using bounded endpoint
exponentiation and parity, with explicit size reservations before growth.
Negative powers compose with reciprocal at a caller-selected grid.
The dyadic `scale` and `affine` methods take their scalar arguments as borrowed
`(mantissa, exponent)` pairs. They return the exact image, including when a
negative factor reverses the endpoints. Scaling with zero offset needs at
most two endpoint products and no translation work.

`float-proposal` adds `F32ProposalInterval` and `F64ProposalInterval`, with
no dependencies or allocation. Their native arithmetic produces endpoint
estimates for scheduling and candidate construction. It makes no containment
or exclusion guarantee, and these types cannot convert into certified intervals.
NaN/infinity input and overflowing calculations return typed errors. Optional
serde encodes the endpoint IEEE bit patterns and validates them on import;
empty is null and both signed zeros canonicalize to positive zero.

The measurement replay example translates `1/2 ± 1/4` to another origin and
back. At an offset of 2⁵³ the default floating-point calculation loses the
measured width. The example checks the proposed endpoints against an exact
replay of the original measurement. Run it from the workspace root.

```sh
cargo run -p hyperinterval --features rational,float-proposal --example proposal_replay
```

`float-enclosure` independently adds `F32Enclosure` and `F64Enclosure` with
allocation-free integer arithmetic on IEEE encodings. Addition, subtraction,
product, quotient, square and fused affine images have tight outward finite
bounds. Nonnegative integer powers accumulate outward rounding and may be
wider; an unrepresentable intermediate or required endpoint returns Overflow.
These operations preserve subnormals independently of native rounding modes
and FTZ/DAZ. Nonempty ingress requires x86_64 or wasm32-unknown-unknown; other targets
return UnqualifiedEnvironment. Optional serde validates endpoint bit pairs.
Optional `rational` and `dyadic` features provide exact endpoint conversions;
the dyadic bridge takes an explicit size/work budget. IEEE ingress preserves
the exact binary value: `0.1_f64` is not 1/10.

Dyadic precision means grid spacing `2^precision`, not a bound on interval
width. Directed conversions preserve containment. `DyadicBudget` accounts for
reserved intermediate magnitude bits and deterministic work units; it does
not measure CPU cycles or heap bytes. Rational arithmetic and Hyperreal center
evaluation retain their existing resource contracts. In particular, a bridge's
conversion budget is not a total-work bound on computable Real evaluation.
These carriers add no independent scalar cache and do not choose a predicate
policy. An inconclusive ball-membership query remains unresolved.

The Real-to-dyadic example starts with an exhausted conversion budget, retries
with a shared budget, and refines the grid until it separates √2 from 181/128.

```sh
cargo run -p hyperinterval --features hyperreal --example real_to_dyadic
```

```sh
cargo test -p hyperinterval --no-default-features
cargo test -p hyperinterval --all-features
cargo run -p hyperinterval --example fixed_bounds --features fixed
```

From a workspace checkout, run the native production coverage gate with
`python3 crates/hyperinterval/scripts/coverage.py`. It uses the active Rust
toolchain's `llvm-tools-preview` component and runs the all-feature library
and integration tests. The printed artifact directory contains per-file counts
and the merged profile. If an instrumented metric falls below 100%, the
command writes the annotated production source to `uncovered.txt` and exits
with an error.
For branch counts, run
`RUSTUP_TOOLCHAIN=nightly RUSTFLAGS=-Zcoverage-options=branch python3 crates/hyperinterval/scripts/coverage.py`
with that nightly toolchain’s LLVM tools installed. A toolchain with no branch
counters reports branch coverage as unmeasured.

See the [API proposal](../../docs/research/interval-api-design.md) and
[progress record](../../INTERVAL_REFACTOR_PROGRESS.md) for acceptance evidence
and the remaining implementation scope.

Exact width measurements preserve values outside endpoint storage: `FixedWidth<F>`
uses an unsigned numerator with a binary scale, and `F32Width`/`F64Width` retain
checked endpoint differences. Float width ordering is exact integer arithmetic;
`enclosure` is an explicit, fallible projection. Rational and dyadic intervals
retain their exact width operations. `DyadicInterval::compare_singleton` shares
caller-owned work/size accounting and rejects non-singleton operands.

`midpoint_enclosure` and `radius_enclosure` explicitly enclose the exact center
and half-width. Rational results are exact singletons. Fixed and strict IEEE
results use tight outward endpoint neighbors when a half-step is not representable;
fixed radius storage can overflow. Dyadic results are exact except for outward
projection at the minimum stored exponent. All dyadic work uses the caller's
budget. A radius projection measures the original half-width, not a radius
about a separately rounded midpoint. These APIs do not select a point implicitly.
