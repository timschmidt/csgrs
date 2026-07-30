# HyperBREP

Hyper-native exact boundary representation for CAD.

HyperBREP uses `hyperreal::Real` as its only authoritative scalar and familiar
CAD vocabulary: `Vertex`, `Edge`, `EdgeUse`, `Wire`, `Face`, `Shell`, `Solid`,
`Curve3`, `Pcurve`, `Surface`, and `Model`.

The source model is an immutable typed arena. Geometry and topology are staged
through `ModelBuilder`; invalid references, disconnected wires, mismatched edge
endpoints, and unsupported edge/pcurve/surface agreement cannot be published as
a `Model`.

## Exactness contract

- Coordinates, parameters, domains, weights, knots, and transforms use `Real`.
- Geometric equality and ordering use Hyperlimit predicates, not structural
  `PartialEq` or an epsilon.
- An unresolved predicate is a typed error, never `false`.
- Numerical methods may propose candidates but cannot author topology without
  exact certification.
- Tessellations and other approximate outputs are derived data, not source
  BREP truth.

See
[`docs/architecture/0001-exact-decision-contract.md`](docs/architecture/0001-exact-decision-contract.md)
and
[`docs/architecture/0002-model-ownership-and-validity.md`](docs/architecture/0002-model-ownership-and-validity.md)
for the binding design decisions.

## Current exact vertical slice

The clean-break implementation currently provides:

- exact line, circular/elliptic arc, rational Bézier, and finite non-periodic
  NURBS `Curve3` carriers;
- arbitrary positive-order exact derivatives, certified represented parameter
  location, NURBS knot-insertion subdivision, supported reversal, and
  conservative exact curve bounds;
- parameterized planes plus sphere, cylinder, cone, and torus carriers with
  exact domains, evaluation, and partial derivatives;
- exact extrusion, revolution, tensor-product rational Bézier, and
  tensor-product NURBS surface carriers;
- Hypercurve-owned pcurves;
- private typed-ID arenas and retained adjacency;
- exact edge endpoint validation;
- per-edge-use exact parameter correspondence with affine and native
  directed-angular-sweep relations;
- certified line/plane, circular-arc/plane, axial-line/cylinder,
  circular-edge/cylinder, generator-or-circle/cone, circular-edge/torus, and
  line-or-circle/extrusion image agreement over complete parameter intervals;
  exact line/circle/rational-Bézier/NURBS meridian and latitude-circle
  agreement on revolution surfaces; plus rational Bézier/NURBS iso-boundary
  agreement on tensor surfaces;
- certified planar loop winding, inner-wire nesting, and exact positive shell
  volume;
- connected closed wires, connected shells, closed-manifold solid checks, and
  certified inward z-prism void shells;
- exact shared-edge `builder::cuboid`, native `builder::cylinder`,
  boundaryless-face `builder::sphere`, truncated `builder::cone_frustum`, and
  periodic-patch `builder::torus` constructors;
- native `builder::revolve` for exact simple off-axis radial/axial polygons,
  with four identity-stitched angular cells per profile edge;
- native `builder::{revolve_contour, revolve_contour_region}` for exact
  off-axis line/arc profiles and cavities; circular profile meridians remain
  native, exact contour first moments drive Pappus volume, and curved face
  area is integrated analytically;
- native `builder::{revolve_path, revolve_path_region}` for exact closed
  `CurvePath2` profiles and cavities; authored Bézier, polynomial B-spline,
  finite NURBS, and single-carrier periodic spline profiles remain native;
  periodic inputs are partitioned into exact clamped knot-span meridians,
  exact span injectivity and pairwise intersections certify simplicity,
  polynomial-equivalent, rational-quadratic, exact homogeneous degree-elevated
  conic, arbitrary-degree at-most-quadratic-weight, and nonuniform rational
  line-image moments drive exact volume, while higher weight-degree rational
  moments remain explicit unsupported measurements; transverse clipping
  decomposes multi-span NURBS meridians exactly and deduplicates contacts at
  shared knot seams;
- exact `builder::revolve_region` cavities as inward periodic shells with
  first-moment subtraction and radial/profile material classification;
- exact `builder::sphere_with_voids` construction with inward complete-sphere
  cavities, strict nesting, exact volume, and material classification;
- validated open-shell `builder::{rational_bezier_patch, nurbs_patch}`
  constructors with native spline boundary curves and exact parameter maps;
- `builder::tensor_patch_shell` for mixed collections of exact rational
  Bézier/NURBS patch specifications; complete projectively identical
  boundaries are identity-stitched across faces, including reversed traversal
  and globally scaled weights, while unmatched boundaries remain open;
- exact concave-simple-polygon `builder::extrude` plus native line/arc
  `builder::{extrude_contour, extrude_contour_regions}` construction with
  holes and disconnected solids;
- exact affine `builder::{sweep, sweep_region}` for polygonal linear-path
  sweeps in an explicit model-space profile frame, including through-holes
  and shear without a hidden moving-frame policy;
- exact `builder::sweep_curve` for fixed-frame polygon sweeps along rational
  Bézier paths whose normalized profile-plane progress is proven affine and
  strictly positive by Bernstein coefficient identity; lateral curvature is
  unrestricted, side faces are native tensor rational Bézier translation
  surfaces, and no moving frame or sampling policy is inferred;
- exact multi-section `builder::loft` for vertex-corresponding polygons:
  every span independently retains planar homothetic sides or certifies convex
  non-homothetic interpolation with native bilinear tensor patches;
  intermediate rings are identity-shared C⁰ seams, and piecewise quadratic
  section-area integration supplies exact volume without inferred
  correspondence or continuity;
- exact region, multi-region, through-hole, and closed-cavity extrusion
  construction;
- retained exact bounds plus exact planar/spherical/cylindrical/conical/
  toroidal/extrusion/revolution face-area and
  affine-prism/sphere/cylinder/frustum/full-or-planar-capped-torus/revolution
  solid-volume queries;
- cached exact solid point classification plus line/plane, plane/plane,
  circular-or-elliptic-arc/plane, circular-arc/sphere,
  transverse-circular-arc/cylinder-or-cone-or-torus, line/sphere, line/cylinder,
  line/cone, plane/sphere, and sphere/sphere intersections, plus certified
  authored-axis transverse plane/revolution profile sections,
  transverse plane/cone/torus and
  perpendicular/oblique/axial-parallel plane/cylinder cuts,
  parallel-cylinder intersections, and coaxial sphere/cylinder intersections
  with retained two-surface pcurves when their authored frames align,
  with retained multiplicity, overlaps, coincidence, tangent points, lines,
  circles, and exact ellipse curves;
- topology-safe exact family-preserving transforms, including rigid
  cylindrical/conical/toroidal/revolution translations and reflections,
  arbitrary affine line prisms, and rigidly reoriented curved prisms;
- copy-on-write stable-ID geometry/topology replacement transactions with
  complete validation on commit;
- exact canonical edge splitting with incident pcurve subdivision, native
  root-lineage angular subdivision, forward/reversed wire repair, and full
  certificate-preserving revalidation without nested rational-projection
  expression growth;
- exact planar face splitting along identity-shared line chords, including
  certified hole reassignment, owning-shell repair, and cap-region
  recertification after subdivision;
- exact curve-driven planar splitting that attaches retained intersection-line
  endpoints to existing boundary vertices or splits crossed canonical edges at
  their represented `Real` parameters before identity sharing;
- deterministic exact multi-trace planar arrangement and partitioning,
  independent of caller order and trace direction, with identity-shared
  crossing vertices and explicit duplicate, positive-length-overlap,
  ambiguous-region, and unsupported-curve outcomes;
- deterministic mixed straight/retained-curve face arrangements with exact
  pcurve crossing atomization, descendant-region routing, and wholly interior
  closed curves authored as identity-shared outer/inner wire pairs; nested
  closed loops and boundary-attached traces are invariant under input order
  and direction;
- regularized `boolean::{union, intersection, difference}` for certified exact
  z-prisms, preserving line/arc boundaries across disconnected and holed
  outputs such as native-cylinder lenses and annuli, with explicit empty,
  unsupported-operand, incompatible-slab, and unresolved outcomes;
- arbitrary-orientation parallel/antiparallel cylinder Booleans through an
  exact cylinder-local frame, including coaxial axial-interval regularization
  and native arc-bounded radial results;
- arbitrary-orientation coincident cone-frustum Booleans through exact
  slant-interval regularization, including two-solid interior cuts;
- exact identical-torus Booleans across equivalent periodic frames, including
  reversed carrier axes;
- exact transverse torus/slab intersection through retained concentric
  latitudes, one or two native annular caps, periodic parameter-cell
  certification, and analytic band measurement/classification;
- exact axis-containing plane/ring-torus carrier intersection as two native
  meridian circles with one authoritative spatial parameter and exact pcurves
  on both carriers. Parallel-to-axis planes retain exact outer tangency and
  certify strict outer separation, while an oblique axial cutter partitions
  all periodic torus cells plus both planar loops without tolerance sewing.
  Standard intersection and torus-minus-cutter difference publish the two
  complementary half-tori through one closed longitude-region certificate,
  with exact area, volume, classification, transforms, and persistence;
- exact coaxial polygonal-revolution Booleans through Hypercurve radial/axial
  regions, including reversed axes, disconnected outputs, and retained
  toroidal-profile cavities;
- strict certified-AABB disjoint Booleans across every current single-solid
  analytic family, with exact full-arena remapping for multi-solid unions;
- equal, strictly contained, and strictly partially overlapping sphere
  Booleans, including exact spherical-cavity difference and two-face periodic
  cap shells stitched on the exact intersection circle;
- exact authored-axis sphere/slab clipping through retained latitude pcurves,
  first-cut whole-sphere partitioning, native one-cap or two-cap spherical
  regions, and analytic segment measurement/classification;
- retained solid intersection graphs with certified per-face broad-phase
  rejection, exact analytic carrier relations, and explicit unsupported-pair
  evidence, plus exact two-face trim clipping for transverse planar carrier
  lines and exact rational-conic clipping against planar face regions, as the
  common input to general intersection-driven splitting; retained line,
  rational-Bézier, and NURBS fragments can be grouped by source face and
  applied through `partition_first_faces` or `partition_second_faces` as
  deterministic validated partitions on either operand. Unsupported transfer
  is a typed error and never silently skips a known exact trace;
- perpendicular plane/cylinder graph cuts retain one authoritative spatial
  latitude circle plus exact pcurves on both carriers: native angular-sweep
  correspondence on the planar circle and an affine constant-height line in
  cylinder parameters. Quarter traces coalesce into a certified closed planar
  loop, cylinder parameter rectangles tile exactly across axial subdivision,
  and axial slab clipping survives stitching, rigid orientation, reflection,
  operand reversal, and byte-identical persistence with exact `πr²h` volume;
- authored-frame-aligned coaxial sphere/cylinder cuts retain both exact
  latitude pcurves, clip each circle across the four bounded cylinder patches,
  coalesce those fragments back into one periodic sphere trace, and partition
  both complete carriers. Standard intersection stitches two spherical caps
  and the central cylindrical band with exact area, volume, classification,
  transforms, operand reversal, and byte-identical replay. Complementary
  difference stitches the periodic two-loop spherical band to the inward
  cylinder band as one exact genus-one solid with native area, volume,
  classification, transforms, and replay. Opposite cylinder-minus-sphere
  difference retains the two disconnected capped-cylinder ends, each with a
  native reversed spherical cap and exact exclusion-aware queries. Regular
  union joins the spherical band to both capped cylinder ends as one exact
  shell with true union classification and no false primitive profile.
  Strict cross-family containment regularizes the complete Boolean truth
  table in either direction; the nontrivial difference retains the contained
  finite cylinder or sphere as one native inward void with exact queries.
  The exact clearance fast path precedes carrier intersection, so strictly
  contained off-axis pairs do not pay for or depend on an unsupported
  sphere/cylinder surface relation. Finite coaxial intervals that cross only
  one sphere latitude retain the native one-cap/one-band/one-disk topology
  for intersection, union, and either directed difference, with one exact
  finite-cylinder region certificate rather than symmetric-clip assumptions;
- transverse plane/cone cuts use the same two-pcurve latitude abstraction,
  with cone slant parameter retained exactly. Frustum side descendants prove
  the same no-gap/no-overlap parameter grid, so slab clips remain native conic
  solids with exact frustum volume. The z-prism fast path now requires a real
  translation-family certificate and cannot misclassify a two-layer frustum
  from its vertices alone;
- authored-axis transverse plane/revolution cuts intersect the exact meridian
  profile first, then orbit every isolated positive-radius contact into one
  authoritative spatial circle with exact planar and constant-profile
  pcurves. Exact profile-range grids recertify the retained revolution sides;
  planar annular caps reconstruct the missing radial profile segments, so a
  revolution/slab intersection survives standard stitching with exact area,
  volume, classification, operand reversal, and persistence;
- axis-containing plane/cone cuts retain two native lower-bounded generator
  rays. Each ray owns one authoritative spatial `Real` parameter plus exact
  affine parameter rays on both carriers; face trimming clamps that parameter
  at the apex and publishes only finite two-pcurve fragments. An axial cuboid
  cutter partitions both operands and standard intersection or complementary
  difference publishes either longitudinal half-frustum with exact area,
  volume, halfspace classification, transforms, profile isolation, and
  persistence;
- all-face retained-graph selection through `select_first_faces` and
  `select_second_faces`, with exact pcurve-region witnesses for trimmed planar,
  analytic, and tensor carriers plus canonical parameter-domain witnesses for
  boundaryless faces; every face receives an operation-aware action or a typed
  ownership error;
- all-face selected-result transfer through `stitch_selected_faces`, preserving
  source-local curved edge identity and matching cross-operand edges only by
  exact restricted `Curve3` evidence. Whole spherical faces and intact curved
  shells transfer natively; exact affine Bézier lines and coplanar tensor
  carriers regularize to canonical line/plane geometry with rebuilt projected
  pcurves before ordinary shell validation. Planar faces in mixed curved shells
  likewise rebuild line pcurves from the authoritative spatial edge and plane
  frame instead of persisting arrangement-expression history. Subdivided
  curve-sweep connectors and active tensor subrectangles also recertify exact
  transverse path clips;
- `boolean::intersect_faces` as the same exact carrier-and-trim primitive for
  arbitrary validated faces, including faces in open shells; finite tensor
  iso-curves and plane/extrusion sections therefore do not require fabricated
  solid ownership before both pcurves can be clipped and an exact spatial
  subcurve published;
- exact coincident-plane support arrangements, oriented material-side
  ownership, and selected-face identity stitching for planar union,
  intersection, and difference, including reversed face-local pcurves,
  cross-operand edge-domain remapping, disconnected results, and fully
  revalidated prism-family shells; the standard `union`, `intersection`, and
  `difference` entry points use this route whenever the faster world-z prism
  specialization cannot certify the operands or result;
- an exact convex planar shell fast certificate based on convex line-face
  loops and oriented half-spaces, plus a general straight-planar certificate
  that compares every coincident face region, boundary segment, and transverse
  face-line material interval exactly; this publishes concave non-prismatic
  skew union and difference results, assigns contained inward planar
  components as exact void shells, rejects non-manifold line or point
  contacts, and survives untrusted persistence replay;
- deterministic exact JSON for every current `Curve3` and `Surface` carrier,
  plus native line/circular-arc and rational-Bézier pcurves, through an
  untrusted, fully
  revalidated `RawModel`;
- exact plane intersection for linear-extrusion rational Bézier and
  degree-1-v NURBS tensor patches when the plane selects one bounded native
  iso-curve, plus exact non-isoparametric plane sections of rational Bézier
  and NURBS translation tensors linear in either parameter axis when every
  rational graph coefficient is certified inside the bounded patch; the native
  spatial spline and exact rational graph pcurve are retained without fitting.
  Partial sections with `Real`-represented tensor-boundary roots are returned
  as one or more exact retained curves; non-representable algebraic boundaries
  remain explicit rather than approximated;
- exact `Surface::iso_curve` extraction in either tensor direction and
  `Model::split_face_by_surface_curve` transfer for one retained rational
  Bézier/NURBS iso-curve or complete rational-Bézier or NURBS
  translation-tensor graph sections in either axis; full homogeneous controls, native NURBS
  domains and knots, exact rational-Bézier or multi-span NURBS graph pcurves,
  curved-loop simplicity, same-boundary two-edge pockets, and orientation are
  recertified, interior endpoints split canonical boundary edges, the new curve
  is identity-shared by both descendants, and the complete open shell is
  revalidated before publication; `Model::split_face_by_surface_curves`
  applies every retained fragment with an explicit first/second operand
  selector, exact endpoint ordering, deterministic descendant selection, and
  caller-order-independent topology; represented transverse pcurve contacts
  are atomized through Hypercurve's certified curve-intersection evidence,
  while overlaps and unresolved algebraic contacts remain explicit;
- topology-derived curve-sweep certification that cancels internal edges
  across coplanar cap descendants, proves each rational-tensor side group
  tiles its complete exact unit parameter boundary, recovers the authored
  translated path/profile from current topology, and therefore preserves
  volume, classification, and untrusted replay after exact cap-edge,
  cap-face, and tensor-face subdivision;
- exact plane/extrusion-surface intersection as a projected native
  line/rational Bézier/NURBS curve for transverse extrusion directions, or
  exact lifted lines for supported parallel profile contacts; materializable
  retained line/rational-Bézier/NURBS pcurves are clipped in both face regions
  and mapped back to exact `Curve3` subcurves in the intersection graph;
  native finite fragments retain both clipped pcurves and compose exact affine
  parameter maps when a rational Bézier subcurve normalizes to `[0,1]` but a
  NURBS subcurve keeps its native knot interval;
- optional `tessellation` integration with two deliberately separate outputs:
  exact oriented HyperTRI triangulation for validated line-bounded planar
  faces, and explicitly lossy chordal output for line-trimmed rational
  Bézier/NURBS faces under a uniform parameter-subdivision policy; chordal
  artifacts retain every exact `Real` parameter and source-surface image and
  certify only exact vertex incidence, never an unproved interior error bound;
- immutable `Arc`-shared `Model` values.

The implementation intentionally rejects combinations whose proof path has not
landed yet. The workspace-root
[`HYPERBREP_REBUILD_PLAN.md`](../HYPERBREP_REBUILD_PLAN.md) is the execution
plan for analytic surfaces, intersections, editing, Booleans, spline surfaces,
tessellation, and adapters.

Enable the optional derived-output adapter with `--features tessellation`.
HyperTRI remains outside the default core dependency graph.

The current design and certified operation boundary are recorded in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) and
[`docs/SUPPORT_MATRIX.md`](docs/SUPPORT_MATRIX.md).

## Example

The runnable [`examples/basic.rs`](examples/basic.rs) constructs and measures
one exact cuboid:

```sh
cargo run --example basic
```

The essential construction flow is:

```rust
use hyperbrep::{Point3, Real, builder};

let (model, solid) = builder::cuboid(
    Point3::origin(),
    Point3::new(Real::from(2), Real::from(3), Real::from(5)),
)?;
let exact_volume = model.solid_volume(solid)?;
# let _ = exact_volume;
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Development checks

```sh
cargo fmt --all -- --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
```

## Non-goals

HyperBREP does not use triangle meshes, voxel fields, physics shapes, or
primitive floats as source topology. Those systems consume validated BREP data
through explicitly derived adapters.
