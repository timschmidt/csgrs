# Hyperlattice interval algebra

Fixed-size Cartesian interval vectors, matrices and boxes. Hyperinterval owns
scalar arithmetic; this package owns Cartesian structure and linear products.
Hyperlattice re-exports the same types for host consumers. This companion
provides the Rust 1.88, allocation-free `no_std` dependency path for firmware.

```rust
use hyperlattice_interval::{IntervalMatrix, IntervalVector};
use hyperlattice_interval::hyperinterval::FixedInterval;

type I = FixedInterval<i32, 0>;
let matrix = IntervalMatrix::new([[I::point(2)?, I::point(-1)?]])?;
let vector = IntervalVector::new([I::new(1, 3)?, I::new(4, 5)?])?;
let image = matrix.mul_vector(&vector, &mut ())?;
assert_eq!(image.lanes().unwrap()[0].bounds(), Some((-3, 2)));
# Ok::<(), hyperlattice_interval::IntervalLatticeError>(())
```

Build and test with `cargo test -p hyperlattice-interval --features fixed`.
No scalar backend is enabled by default. `fixed` and `float-enclosure` require
no allocator. `dyadic` uses allocation with explicit Hyperinterval work/size
accounting. `rational` enables Hyperreal, std and its Rust 1.95 requirement.
`serde` validates scalar endpoints, dimensions and fixed scales on replay.

Implemented: canonical empty/unit products, component hull/intersection and
inclusion, sums/differences, dot products, matrix products, affine enclosures,
transpose and checked row/column projections. All loops have dimension bounds;
inline storage and stack use grow with those dimensions. Scalar overflow and
resource errors propagate; no partial result is returned. Rational operations,
explicit clones and serialization have no total-work or heap bound.

The outputs are Cartesian enclosures. They need not equal a correlated image;
an affine enclosure does not retain the defining affine relation. Exact
correlated affine carriers, midpoint/radius forms, norms, matrix measures and
measured specializations remain under development in Phase 6 of the
[implementation plan](../../docs/INTERVAL_REFACTOR_PLAN.md).
