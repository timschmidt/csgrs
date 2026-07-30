# HyperBREP fuzzing

`model_builder` exercises exact line/plane construction and typed-arena
ownership transitions. `analytic_solid_roundtrip` varies cylinder, sphere,
cone-frustum, torus, off-axis polygonal and line/arc-profile revolutions,
affine linear-sweep, and
fixed-frame rational-Bézier curved-sweep paths, explicit positive
polynomial-taper moving frames, plus alternating
two-/three-section homothetic/convex-corresponding loft dimensions
plus partial
sphere, oriented coaxial cylinder, coincident cone-frustum, and coaxial
revolution union, intersection, and difference. It
then exercises
construction, retained measurement and classification, rigid translation,
exact JSON, full revalidation, and self/cross-model certified intersection
graph construction. Exact cuboid cohorts drive planar
partition/classification/selection for union, intersection, and difference;
the partial-overlap cohort additionally drives coincident support
arrangement, selected-face transfer, reversed pcurves, identity stitching,
solid validation, and exact result publication. A rationally rotated cuboid
also drives transverse support arrangements and convex/non-convex
non-prismatic shell publication through all standard Boolean operations. A
rotated frustum cohort drives axis-containing plane/cone rays, finite
two-pcurve graph clipping, standard longitudinal-half intersection or
difference, exact half-volume, compact mixed-shell JSON, and untrusted replay.
A fixed rationally rotated contained cuboid alternates strict planar-void
publication with exact point-contact rejection and persistence replay. The
same target varies a parallel plane across a certified linear rational tensor
patch, evaluates any retained native iso-curve, and clips that intersection
through an open-shell patch face and a trimmed solid-owned plane face. It also
varies an oblique plane across curved u- and v-linear rational translation
tensors,
exercising certified disjoint, complete non-isoparametric, and explicitly
represented or algebraically blocked partial-graph outcomes plus exact pcurve
materialization. Equal-weight bilinear rational tensors also vary their plane
offset across complete, represented-clipped, boundary-only, and strictly
disjoint outcomes;
retained rational-quadratic parameter graphs compose to exact
rational-quartic spatial curves, and complete or partial splits are replayed
byte-for-byte. Complete
rational-Bézier and single- or multi-span NURBS graph sections in both tensor
directions
also drive
identity-shared curved face splitting, native-domain and control-net
revalidation, rational-pcurve persistence, and untrusted replay. Retained
piecewise-linear NURBS graphs additionally drive multiple disjoint partial
fragments, deterministic all-fragment descendant partitioning, same-boundary
two-edge topology, exact curved-loop area orientation, and partial-profile
recertification.
interior patch curves are transferred into fully revalidated split-face
topology, and a two-face projectively stitched patch shell is persisted and
revalidated. The same exact patch is also sent through byte-varied explicit
boundary subdivision and shared-midpoint chordal derivation; every returned
index must address its retained exact parameter/source-image pair. It also
builds a byte-scaled complete affine tensor patch with separable positive
weights, checks its exact area before and after untrusted replay, and builds a
native-domain NURBS-bounded planar `CurvePath2` face in a byte-scaled skew
plane frame, checking its exact frame-scaled area and replaying the complete
open shell. The same path is extruded through a byte-scaled height and must
retain exact volume through untrusted solid replay. It then
intersects an exact rational Bézier extrusion with a transverse plane and a
line extrusion with a parallel plane, exercising the retained native
projected curve, both exact pcurve images, and lifted-line paths.
Invalid and degenerate byte-derived inputs must return structured errors
without panicking or publishing an invalid model.

`topology_edit_roundtrip` varies exact cuboids and cylinders, then splits an
edge midpoint, an existing-vertex planar cap diagonal, or a curve trace whose
endpoints require exact canonical boundary-edge splits. It also partitions a
cap with parallel, crossing, or three-way concurrent exact traces supplied in
noncanonical geometric order.
Certificate-backed volume and exact persistence must survive every complete
edit/republication pipeline. The retained regression corpus includes the
multi-trace cylinder case that previously exposed exponential serialization
of nested rational-projection expressions.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run model_builder --fuzz-dir fuzz -- -max_total_time=30
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run \
  analytic_solid_roundtrip -- -runs=100
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run \
  topology_edit_roundtrip -- -runs=100
```

`detect_leaks=0` is needed only in ptrace-managed environments where
LeakSanitizer cannot attach; address and coverage sanitization remain active.
