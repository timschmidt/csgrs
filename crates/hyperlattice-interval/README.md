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
transpose and checked row/column projections. Vector and matrix one/infinity
norms include lower and upper bounds. Square matrices also supply logarithmic
one/infinity measures (with signed diagonals) and row/column diagonal-dominance
margin vectors. These supply interval evidence for downstream decisions. All loops have dimension bounds;
inline storage and stack use grow with those dimensions. Scalar overflow and
resource errors propagate; no partial result is returned. Rational operations,
explicit clones and serialization have no total-work or heap bound.

The vector/matrix outputs are Cartesian enclosures. They need not equal a
correlated image. `AffineMap::image` instead retains an exact source box and
point map; applying another map preserves the shared source parameters.
`AffineImage::enclosure` composes coefficient bounds before evaluating that box.
Rational/dyadic success gives its exact coordinate hull; primitive rounding
can widen the projection. The exact definition survives overflow or exhaustion
of a projection, off-grid results, parameter restrictions and validated replay.
Maps with empty or nonsingleton coefficients are rejected. Finite nesting lives
in the Rust type; inline storage and stack use grow with dimensions and depth.
`AffineChain<I, S, N, CAP>` retains a runtime-selected number of square maps
in inline storage. `push` reports typed capacity exhaustion without changing
the chain. Zero maps retain the source, and rectangular static images can
precede or follow a chain. Replay checks every map and the receiving capacity.

`widths` returns exact per-coordinate measurements, including full unsigned
fixed widths and float widths that cannot fit a finite float. `widest_axis`
compares exact widths and breaks ties by the lowest index. Empty boxes have no
widths; the zero-dimensional unit has an empty width array and no axis.

Midpoint/radius forms, contraction helpers and measured specializations
remain under development in Phase 6 of the
[implementation plan](../../docs/INTERVAL_REFACTOR_PLAN.md).

Consuming a vector or matrix with `midpoint_radius()` creates a
`MidpointRadiusVector<I, N>` or `MidpointRadiusMatrix<I, R, C>`. Each lane defines
its exact center `(a+b)/2` and radius `(b-a)/2` from checked bounds stored once.
`intervals()` borrows the source; `into_intervals()` reconstructs it exactly by
ownership transfer, including half-steps and cases where a numerical projection
overflows. `midpoint_enclosure` and `radius_enclosure` explicitly produce
endpoint-family enclosures with a shared context. They can be nonsingletons
and never replace the retained exact definition. Replay validates the source
intervals through the existing serializer; empty and unit products stay distinct.
