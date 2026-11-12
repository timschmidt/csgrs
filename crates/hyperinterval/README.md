# Hyperinterval

Scalar sets with exact endpoint semantics for the Hyper stack. The current
surface provides checked integer ranges and fixed-point interval arithmetic,
explicit empty sets, sign sets, and named set relations. Host rational/dyadic
bridges and qualified float enclosures are under development.

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

```sh
cargo test -p hyperinterval --no-default-features
cargo test -p hyperinterval --all-features
cargo run -p hyperinterval --example fixed_bounds --features fixed
```

See the [API proposal](../../docs/research/interval-api-design.md) and
[progress record](../../INTERVAL_REFACTOR_PROGRESS.md) for acceptance evidence
and the remaining implementation scope.
