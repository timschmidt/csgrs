# 0002: Model Ownership and Validity

Status: Accepted
Date: 2026-07-29
Amended: 2026-10-14 (model-bound public references)

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

Records have private fields and compact typed internal slots. Public Rust
operations use borrowed `VertexRef`, `EdgeRef`, `FaceRef`, `SolidRef` and the
corresponding references for other entity families. Each carries its owning
immutable revision and a private slot. Traversal preserves that owner; callers
do not pair a bare slot with a separate model. Typed slots exposed by errors
are diagnostic positions, never resolution handles.

`Model` is a cheap clone over immutable shared data. Cached facts and indexes
are immutable or initialized once. There is no shared mutable topology.

Clones share revision identity. Transforms, committed edits and archive replay
create independent revisions, even when the resulting geometry is equal.
Within the process, identity is `Arc::ptr_eq` on the immutable arena allocation.
Borrowing keeps the allocation alive, preventing address reuse while a
reference exists. This identity is used only for ownership and reference
equality/hashing; it is not a geometric predicate, archive identity, or durable
feature name. No global counter or platform-specific identifier is required.
Rust documents allocation identity through
[`Arc::ptr_eq`](https://doc.rust-lang.org/std/sync/struct.Arc.html#method.ptr_eq)
and clone-on-write separation through
[`Arc::make_mut`](https://doc.rust-lang.org/std/sync/struct.Arc.html#method.make_mut).
Applying those contracts to immutable, borrowed revisions is this design's
ownership argument; neither operation establishes geometric equality.

`ModelBuilder` issues typed keys carrying an `Arc` construction scope. It
checks every input key before mutating its staged arenas. Finishing consumes
the builder and retains that scope in the published model, so keys can resolve
there and in clones. They do not resolve after transform, edit commit or
decode. Ordinary arena records remain compact and do not carry scope tokens.

The [public ownership layer](../../src/ownership/mod.rs) is the sole exported
topology API. The private arena algorithms retain their compact indexing;
there is no exported unscoped compatibility route. Constructor and Boolean
results own `BodySet` or `SheetSet`; their iterators return bound references.

### Required ownership

- `Vertex` owns an exact model-space `GeometricPoint3`, including a retained
  selected curve image when independent coordinates are unavailable.
- `Edge` references start/end vertices, a `Curve3`, and an exact parameter
  trim represented by `ParameterRange`. Increasing trim order is separate
  from an incident pcurve's traversal order.
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

`ModelBuilder::finish()` returns a `Model` or a structured `ValidationReport`.
`RawModel::validate()` also accounts for archive errors through
`PersistenceError`, which retains validation failures. Neither operation
repairs, sews, merges, or infers topology.

### Editing

`Model::edit()` creates a staged `Edit`. Commit validates the affected local
region and all impacted global invariants before returning a new `Model`.
Failure leaves the source model unchanged.

An edit retains its original source revision. All targets and related
references must belong to that revision, including references passed after a
staged transform. A foreign or stale revision is rejected before mutation.
Borrowed references to an old model remain valid for that old immutable model;
they do not become references to its edited result.

Topology-operation details borrow their result model and retain the source.
Multi-trace partitions also retain each step's input and output revision.
Later wire compaction cannot silently rebind an earlier split's positions.
This retains intermediate arenas and has a memory cost; persistent arena
sharing and composed history are later refinements. A wire-compaction map
describes its own step, not durable feature naming or a composed history.

The initial implementation may copy full arenas. Copy-on-write and localized
validation are performance refinements that cannot alter edit semantics.

An exact edge subdivision may inherit the validated source's image, wire and
solid proofs because it partitions the same boundary. Publication still checks
connectivity, ownership and the actual parameter correspondence. Replacing a
vertex, carrier or topology record invalidates those inherited proofs.
Selected points and ranges remain exact through subdivision and transforms;
unavailable stored views produce typed blockers in scalar-only consumers.
Selected graph archives and their untrusted replay are not yet implemented.

### Invariants

A `Model` guarantees:

- every internal slot and published borrowed reference resolves in its owner;
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

Explicit shared operation contexts, bounded resource accounting, detailed
decision receipts, mandatory boundary provenance and generated host-handle
registries remain unfinished WP01/WP17 work. This ownership amendment does not
claim those contracts are implemented.

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
