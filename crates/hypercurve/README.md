<h1>
  hypercurve
  <img src="./doc/hypercurve.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hypercurve` is an early-stage planar curve kernel for line and circular-arc
geometry. The current implementation focuses on exactness-aware topology:
segment intersections, closed-contour containment, signed region views,
region-pair event extraction, fragment splitting, and the first boolean
selection/traversal scaffolding.

The crate keeps Cavalier-compatible bulge semantics where they are useful, but
the native APIs expose explicit uncertainty for tangent, overlap, boundary, and
unsupported cases instead of silently resolving them through global epsilon
rules.

## Numeric Model

`hypercurve` stores and decides core geometry with hyperreal-backed
`hyperlattice::Scalar` values. Primitive `f64` appears only in named edge
conversions, tests, benchmarks, rendering/IO helpers, and compatibility
harnesses. There is no approximate scalar backend feature.

## Current Status

- Line and circular-arc segments with explicit intersection result types,
  including finite same-circle arc overlap intervals.
- Axis-aligned bounding boxes for points, segments, curve strings, contours, and
  regions, used as conservative broad-phase filters before exact curve
  intersections, contour/region event collection, point classification, and
  self-contact checks.
- Primitive left offsets for line and circular-arc segments, with explicit
  uncertainty when an arc offset would collapse or reverse radius.
- Raw open curve-string and closed-contour left offsets with line joins:
  line-line corners are mitered, while parallel, arc, and mixed joins use
  circular join arcs until the full offset trim/rebuild pipeline is implemented.
- Curve-string/contour self-contact detection and checked offset entry points
  that reject raw joined offsets requiring self-intersection trimming.
- Checked closed outlines for open curve strings using left/right offsets and a
  selectable `OffsetCap` style: circular, straight butt, or square end caps.
- Closed contours with winding/boundary classification.
- Region point classification skips contours whose bounding boxes are decidably
  missed before exact boundary and winding tests.
- Prepared borrowed curve-string, contour, and region views cache segment,
  contour, and whole-region boxes for repeated self-contact, curve-string
  intersection, contour/region point classification, and contour/region event
  queries, plus prepared region boolean-boundary loop, checked-contour, and
  region-result traversal. Mixed prepared-vs-region event and boolean wrappers
  reuse either prepared operand while transiently preparing the ordinary side,
  without changing exact boundary semantics.
- Owned and borrowed regions with material and hole contour bins.
- Region-pair event collection and point-bearing fragment splitting.
- Boolean fragment classification for union, intersection, difference, and xor.
- Directed boundary-fragment emission, endpoint-connected chain assembly, and
  closed-loop reconstruction into checked contours.
- Region-level boolean boundary pipeline for producing closed result loops or
  checked boundary contours when no unresolved topology remains.
- Resolved boolean boundary contours can be nested into material/hole bins to
  produce a `Region2` result.
- Exact contour-bin boolean identities, including reordered bins, rotated start
  vertices, and reversed traversal, are handled before general traversal so
  coincident boundaries do not force uncertainty for `A op A`.
- Empty-operand boolean identities are handled before traversal, preserving
  material/hole roles for the nonempty region.
- Region-result identity and empty-set fast paths clone explicit material/hole
  bins directly, so valid touching-bin regions are not reinterpreted by
  boundary-only nesting.
- Boolean fragment emission is role-aware: selected fragments from hole
  contours are oriented as negative-fill edges before chain assembly.
- Region-level xor is assembled from the two one-sided differences and merges
  their explicit signed bins, preserving boundary-touching components.
- Checked boundary-contour xor uses the same symmetric-difference region path
  before exposing unassigned boundary loops.
- Boundary-only contacts are certified before traversal: point contacts use
  regularized set identities, and external shared-edge contacts drop coincident
  zero-area edges for union/xor output.
- Boundary-touching containment identities are handled before traversal for
  union, intersection, and contained-minus-container difference.
- Container-minus-boundary-touching-subset difference is rebuilt for certified
  shared-edge containment by dropping the coincident zero-area boundary and
  assembling the remaining directed fragments into the notched result.
- Imported Cavalier deterministic and fuzz suites are present as compatibility
  references.

This crate is not yet a complete boolean or offset engine. Shared-boundary
fragments with positive-area overlap beyond the certified containment/contact
fast paths, point-touching containment branches, or otherwise ambiguous topology
are still reported as unresolved until the general overlap resolver is
implemented; joined offsets are not yet self-intersection trimmed.

## Documentation

Public APIs are written to build cleanly on docs.rs. Local verification uses:

```text
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
```

## UI Test Article

The `examples/hypercurve_ui` package recreates the Cavalier UI test article
around `hypercurve` operations. It keeps a bulge-vertex editor for convenient
interactive testing, then converts that data into `hypercurve` contours and
regions for booleans, intersections, slices, and offsets.

```text
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
cargo check --manifest-path examples/hypercurve_ui/Cargo.toml --target wasm32-unknown-unknown
trunk serve examples/hypercurve_ui/index.html
```

If Trunk 0.21 rejects `NO_COLOR=1` in the local environment, run the Trunk
command as `env -u NO_COLOR trunk serve examples/hypercurve_ui/index.html`.

## Benchmarks

Small no-dependency benchmark targets exercise the current ordinary, prepared,
and mixed-prepared boolean boundary pipeline, containment hot paths including
prepared contour and region repeated-query classifiers, segment-intersection
hot paths, ordinary and prepared curve-string intersections, ordinary and
prepared bounding-box filtered self-contact scans, ordinary and prepared
region-event scans, and primitive plus open-outline offsets:

```text
cargo bench --bench boolean_pipeline
cargo bench --bench containment
cargo bench --bench intersection
cargo bench --bench offset
```

## References

Foster, Erich L., Kai Hormann, and Romeo Traian Popa. "Clipping Simple Polygons
with Degenerate Intersections." *Computers & Graphics: X*, vol. 2, 2019,
article 100007. https://doi.org/10.1016/j.cagx.2019.100007.

Bentley, Jon Louis, and Thomas A. Ottmann. "Algorithms for Reporting and
Counting Geometric Intersections." *IEEE Transactions on Computers*, vol. C-28,
no. 9, 1979, pp. 643-647.

Greiner, Gunther, and Kai Hormann. "Efficient Clipping of Arbitrary Polygons."
*ACM Transactions on Graphics*, vol. 17, no. 2, 1998, pp. 71-83.
https://doi.org/10.1145/274363.274364.

Hormann, Kai, and Alexander Agathos. "The Point in Polygon Problem for
Arbitrary Polygons." *Computational Geometry*, vol. 20, no. 3, 2001, pp.
131-144. https://doi.org/10.1016/S0925-7721(01)00012-8.

Martinez, Francisco, Antonio J. Rueda, and Francisco R. Feito. "A New Algorithm
for Computing Boolean Operations on Polygons." *Computers & Geosciences*,
vol. 35, no. 6, 2009, pp. 1177-1185.
https://doi.org/10.1016/j.cageo.2008.08.009.

Tiller, Wayne, and Eric G. Hanson. "Offsets of Two-Dimensional Profiles." *IEEE
Computer Graphics and Applications*, vol. 4, no. 9, 1984, pp. 36-46.

Vatti, Bala R. "A Generic Solution to Polygon Clipping." *Communications of the
ACM*, vol. 35, no. 7, 1992, pp. 56-63.
https://doi.org/10.1145/129902.129906.
