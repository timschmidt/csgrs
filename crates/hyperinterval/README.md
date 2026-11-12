# Hyperinterval

Scalar sets with exact endpoint semantics for the Hyper stack. The current
surface provides checked integer/fixed arithmetic, exact rational intervals,
budgeted common-scale dyadic arithmetic, and symbolic Real balls. Empty sets,
sign sets, and set relations are explicit. Qualified float and elementary
function enclosures remain under development.

Hyperinterval encloses. Hyperlattice carries set-valued linear structure.
Hyperlimit decides. Hypersolve certifies. Hyperdynamics evolves. Domain crates
assign meaning.

```rust
use hyperinterval::{FixedInterval, SignSet};

type Q30Bounds = FixedInterval<i32, 30>;
let range = Q30Bounds::new(-(1 << 29), 1 << 29)?;
assert_eq!(range.signs(), SignSet::Mixed);
assert!(range.contains_point(0)); // zero is possible, not a scalar equality proof
assert_eq!(range.square()?.bounds(), Some((0, 1 << 28)));
# Ok::<(), hyperinterval::IntervalError>(())
```

Enable `fixed` for that example. `integer` enables exact discrete ranges of
all signed and unsigned primitive integer widths. Fixed endpoints represent
`bits / 2^FRAC`; i32 and i64 are supported with `FRAC` less than the word size.
Integer ranges are discrete sets; fixed intervals contain every real point
between their endpoints. Neither implements scalar ordering.

Integer arithmetic uses checked sum, difference, negation, product hull,
square and nonnegative integer powers. Integer multiplication can have gaps;
the result is the smallest containing integer range. Division is not silently
interpreted as truncated real arithmetic. Exact width/radius and explicitly
rounded midpoints support every primitive integer width, including i128/u128.

Fixed arithmetic widens intermediates to i128. Products, quotients, reciprocals,
squares, ratio ingress and fused affine images round outward; representational
overflow is an error, never saturation. Integer powers use a bounded sequence
of outward-rounded products and may be wider than the tight hull. Storage
conversion is exact or fails; scale coarsening rounds outward. Strict division
rejects any zero-containing denominator, even with an empty numerator.

No default feature enables allocation, `std`, Hyperreal or serialization.
The `integer` and `fixed` builds are allocation-free `no_std` and target Rust
1.88. Optional `serde` uses null for empty and an ordered endpoint pair for
nonempty sets, with scale and endpoint validation on import. This is an
initial encoding, not a migration of any domain's canonical evidence.

The `rational` feature adds `RationalInterval` over Hyperreal's exact rationals
and requires Rust 1.95. It supplies exact sum, difference, product hull,
square, strict quotient, affine images and positive arithmetic helpers. The
`dyadic` feature is allocation-enabled `no_std` on Rust 1.88: its BigInt
mantissas share an exponent, with explicit budgets for arithmetic and scale
changes. `hyperreal` enables both carriers and their certified Real bridge;
`real-ball` adds exact symbolic centers with nonnegative rational radii.

Dyadic width, midpoint and radius return exact singleton intervals, or empty
for an empty input. They never round an unrepresentable half. Pointwise square
preserves dependence across zero, while multiplication treats its operands as
independent sets. Reciprocal accepts an explicit output grid and rejects inputs
containing zero. These operations share the caller's size and work budget.
Nonnegative integer powers compute the exact image using bounded endpoint
exponentiation and parity, with explicit size reservations before growth.
Negative powers compose with reciprocal at a caller-selected grid.

Dyadic precision means grid spacing `2^precision`, not a bound on interval
width. Directed conversions preserve containment. `DyadicBudget` accounts for
reserved intermediate magnitude bits and deterministic work units; it does
not measure CPU cycles or heap bytes. Rational arithmetic and Hyperreal center
evaluation retain their existing resource contracts. In particular, a bridge's
conversion budget is not a total-work bound on computable Real evaluation.
These carriers add no independent scalar cache and do not choose a predicate
policy. An inconclusive ball-membership query remains unresolved.

```sh
cargo test -p hyperinterval --no-default-features
cargo test -p hyperinterval --all-features
cargo run -p hyperinterval --example fixed_bounds --features fixed
```

See the [API proposal](../../docs/research/interval-api-design.md) and
[progress record](../../INTERVAL_REFACTOR_PROGRESS.md) for acceptance evidence
and the remaining implementation scope.
