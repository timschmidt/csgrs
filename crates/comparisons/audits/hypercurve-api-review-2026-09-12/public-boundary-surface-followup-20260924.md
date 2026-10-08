# Public boundary surface follow-up (read-only inventory)

Production consumers in Hyperbrep, Hypermesh, Hypergraphics, Synaps CAD,
Hypertri, Hyperphysics, Hypersdf, csgrs and Hypervoxel no longer mention
BezierSplitFragment2, BezierSplitMaterialization2 or CurveRegionBoundaryLoop2.
Curve2 already supplies exact split_at and subcurve operations over the common
CurveParameter2. BoundaryLoop.curves exposes all retained exact curves; its
fragments accessor is crate-private.

Hypercurve still exports the construction-history fragment/materialization
carriers. BoundaryLoop::new and try_new_with_arrangement_sources accept them.
The latter also exposes arrangement source records whose mathematical role
must be distinguished from required boundary geometry. Older region tests,
benches/bezier_region.rs and fuzz/fuzz_targets/bezier_region.rs construct these
fragments directly and need direct migration to the common path/curve API.

This is not just an export change: BezierArrangementGraph2 publicly accepts
split materializations, five authored curve split methods return them, and
RationalBezierGeneral pair materializations expose them through first/second.
The dedicated split-materialization integration tests include invalid
partition/provenance checks. Preserve those mathematical checks at the owning
internal construction boundary or the new public exact split contract; do not
delete them merely because the old representation becomes private. A surviving
authoring/control-net conversion is a distinct operation, not a reason to keep
a second general exact curve API. Remove superseded public entry points and
update all callers together; no forwarding compatibility layer is needed.

This inventory changes no production source. Runtime simplification remains a
separate obligation: boundary storage still owns fragments and lazily creates
Curve2 views, while retained Curve2 owns a fragment. Hiding history-specific
public types alone would not consolidate that dual storage or its dispatch.
