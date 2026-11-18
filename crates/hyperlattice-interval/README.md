# Hyperlattice interval algebra

Fixed-size Cartesian interval vectors, matrices and boxes. Hyperinterval owns
scalar arithmetic; this package owns Cartesian structure and linear products.
Hyperlattice re-exports the same types for host consumers. This companion
provides the Rust 1.88, allocation-free `no_std` dependency path for firmware.

<!-- quickstart:start -->
```rust
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};

fn main() -> Result<(), IntervalLatticeError> {
    type I = FixedInterval<i32, 0>;
    let matrix = IntervalMatrix::new([[I::point(2)?, I::point(-1)?]])?;
    let vector = IntervalVector::new([I::new(1, 3)?, I::new(4, 5)?])?;
    let image = matrix.mul_vector(&vector, &mut ())?;
    assert_eq!(image.lanes().unwrap()[0].bounds(), Some((-3, 2)));
    let scale = IntervalMatrix::new([[I::point(2)?]])?;
    let next = scale.mul_vector(&image, &mut ())?;
    assert_eq!(next.lanes().unwrap()[0].bounds(), Some((-6, 4)));
    Ok(())
}
```
<!-- quickstart:end -->

Build and test with `cargo test -p hyperlattice-interval --features fixed`.
No scalar backend is enabled by default. `fixed` and `float-enclosure` require
no allocator. `dyadic` uses allocation with explicit Hyperinterval work/size
accounting. `rational` enables Hyperreal, std and its Rust 1.95 requirement.
`hyperreal` enables the native certified Real-to-dyadic conversion bridge,
including std and the same Rust 1.95 requirement.
`serde` validates scalar endpoints, dimensions and fixed scales on replay.

Run the rectangular replay and multiplication tests with
`cargo test -p hyperlattice-interval --features fixed,serde --test shape_replay`.
From `crates/hyperinterval`, run the same oracle under libFuzzer with
`cargo +nightly fuzz run cartesian_shapes --features lattice-shapes`. The
target covers empty dimensions and shapes up to 33 lanes, malformed wire
lengths, outward rounding, and overflow in successive products.

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

Jacobian carriers, broader authored-form composition and measured
specializations remain under development in Phase 6 of the
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

`CenteredVector<I, N>` and `CenteredMatrix<I, R, C>` instead retain independently
authored point centers and nonnegative point radii. `new(center, radius)` checks
those conditions without computing bounds. Borrowed `center()` / `radius()` and
owned `into_parts()` retain the definition, including when `enclosure()` fails
with overflow or a resource error. The explicit enclosure uses shared scalar
subtraction, addition and hull; it can round outward and remains separate from
the exact definition. Empty or nonsingleton components are invalid; `empty()`
creates the canonical empty set. Serde revalidates both components. Both use
the same scalar backend. A vector’s `affine_source(context)` creates a
`TranslatedBox` with its exact center and signed offset parameters `[-r,r]`.
Affine images and chains consume this definition before any absolute endpoint
projection. Restrictions select offsets, not absolute coordinates. Replay
validates the center and parameter box. Matrix forms are not affine sources.

Square matrices provide `identity_minus()` for the Cartesian image of `Id - A`,
and `contraction_margin_one()` / `contraction_margin_infinity()` for the ranges
of `1 - ||A||`. These compose with matrix products and affine enclosures for
Krawczyk arithmetic. Positive lower margins supply norm evidence; consumers
still establish Jacobian validity, self-mapping and solver decisions. Empty
inputs stay empty. Nonempty queries require representable scalar one, except
zero-dimensional identity subtraction; fixed i32/F31 and i64/F63 therefore
report typed overflow. Scalar operations share the caller's context.

The standalone [Alumina FOC client](conformance/alumina-foc/README.md) exercises
Clarke/Park transforms through the shared fixed kernels, retaining Alumina's
rounding boundaries and typed errors. It includes independent exact corner
checks, transform chains and matched cost probes against the firmware routines.
Build it with its own manifest and the sibling `alumina-firmware` checkout.
