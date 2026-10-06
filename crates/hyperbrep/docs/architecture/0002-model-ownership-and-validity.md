# 0002: Model Ownership and Validity

Status: Accepted
Date: 2026-07-29

## Context

The old API exposes mutable topology records and vectors. Edges have no curve,
edge uses have no pcurve, loops are nested values, and most operations rebuild
identifier maps before they can query adjacency. Invalid state is normal, so
ordinary callers must ask a family of readiness reports whether a value is
usable.

The replacement needs stable identity, compact lookup, immutable sharing,
transactional editing, and validity guarantees strong enough for conventional
modeling operations.

## Decision

### One immutable arena

`Model` owns geometry and topology in compact typed arenas:

```text
geometry: Curve3, Pcurve, Surface
topology: Vertex, Edge, EdgeUse, Wire, Face, Shell, Solid
```

Records have private fields. Public typed IDs are stable within one model and
index their arenas in O(1). IDs are not pointers, global identifiers, or
geometric hashes.

`Model` is a cheap clone over immutable shared data. Cached facts and indexes
are immutable or initialized once. There is no shared mutable topology.

### Required ownership

- `Vertex` owns a model-space point.
- `Edge` references start/end vertices, a `Curve3`, and an exact parameter
  domain.
- `EdgeUse` references an edge, direction, face-local `Pcurve`, and exact
  parameter correspondence.
- `Wire` owns an ordered sequence of edge uses.
- `Face` references one surface, one outer wire, and zero or more inner wires.
- `Shell` owns oriented faces.
- `Solid` owns one outer shell and zero or more void shells.

Model indexes retain all reverse adjacency required by ordinary queries.
Queries do not reconstruct maps.

### Three construction states

1. `RawModel` is an untrusted import carrier. It may be invalid and exposes no
   modeling or measurement API.
2. `ModelBuilder` stages normal construction. It rejects invalid local
   references immediately.
3. `Model` is immutable and globally validated.

`ModelBuilder::finish()` and `RawModel::validate()` return either a `Model` or a
structured `ValidationReport`. They never repair, sew, merge, or infer topology.

### Editing

`Model::edit()` creates a staged `Edit`. Commit validates the affected local
region and all impacted global invariants before returning a new `Model`.
Failure leaves the source model unchanged.

The initial implementation may copy full arenas. Copy-on-write and localized
validation are performance refinements that cannot alter edit semantics.

### Invariants

A `Model` guarantees:

- every typed ID resolves;
- every edge's curve evaluates to its endpoint vertices at the retained domain
  boundaries;
- every wire is ordered, connected, and closed by topological identity;
- every edge use has a pcurve on its owning face;
- the edge curve and surface-evaluated pcurve have a certified identical image
  over the edge-use domain;
- face boundaries have certified orientation and nesting;
- manifold shell edges have two opposite uses;
- shell and solid orientation/nesting are certified;
- unsupported geometry is not present as a supported canonical record.

During staged implementation, `finish()` rejects combinations for which these
proofs are not implemented. It does not return a weaker `Model`.

### Conventional successful API

Readiness reports are not the normal success surface. A successful constructor
or operation returns the conventional model object or measurement. Detailed
evidence is retained by validation and operation errors and may be requested
for diagnostics.

## Consequences

- The old public structs and report-driven workflow are deleted.
- Builders may reject geometry families temporarily while their agreement proof
  is being implemented.
- Import code has an explicit raw-to-validated transition.
- Stable typed IDs and retained adjacency enable predictable performance.
- Immutable model sharing makes parallel read-only operations possible once all
  retained geometry carriers are `Send + Sync`.

## Rejected Alternatives

### Mutable object graph with shared references

Rejected because mutation can invalidate distant topology, synchronization
becomes pervasive, and identity depends on allocation.

### Public arena vectors

Rejected because callers could reorder, replace, or remove records without
updating IDs and adjacency.

### Validation reports on every ordinary query

Rejected because canonical model validity belongs in construction and types.

### Automatic sewing or tolerance merging

Rejected because it changes authored topology without exact evidence.

### Separate shell-owned geometry collections

Rejected because shared geometry, stable IDs, cross-solid operations, and
transactional editing all benefit from one model-level ownership boundary.
