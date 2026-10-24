# Construction evidence on native meshes

`GeometryOutcome::certainty` and `MeshOutcome::certainty` describe decisions
consumed by that operation. `TriangleMesh::construction_evidence()` separately
retains qualifications of the construction dependencies. Extracting `.value`,
cloning, transforming, or composing a mesh keeps those qualifications. A strict
PWN or manifold validation proves a fact about the current geometry; it does
not clear an earlier approximate construction.

The summary exposes five independent facts:

| Query | Meaning |
| --- | --- |
| `source_history_missing()` | Some preceding construction history is unavailable. |
| `approximate_512_consumed()` | Available history records an actual terminal predicate decision. |
| `coordinates_projected()` | A coordinate projection occurred; its boundary API defines the loss model. |
| `source_sampled()` | Construction used samples; no continuous-source error bound is implied. |
| `buffers_changed()` | Geometry ceased to match a retained construction identity. |

`predicate_certainty()` is absent when history is missing. Otherwise it reports
the weakest retained predicate certainty since the declared coordinate source.
Selecting `APPROXIMATE_512` alone does not qualify an exact construction. A false
approximate flag cannot certify missing history. Predicate certainty, topology,
and fidelity remain separate contracts even when all retained predicates are
certified.

This is a bounded qualification summary, not a replayable proof or a complete
boundary-to-source mapping. Full curve lineage and persistent proof replay remain
open under `properties.md` P15/P19. Bare curve-to-mesh conversions expose missing
history; finite sampled-profile conversions additionally expose sampling and
projection. Mesh evidence cannot reconstruct a discarded curve receipt. A
caller that composes curve operations keeps the receipt of the whole chain
with `GeometryOutcome::and_then`, which combines each dependent operation's
certainty with the certainty it consumed; `map` cannot see the decisions made
inside its closure.

## Source and mutation boundaries

`TriangleMesh::new` imports raw coordinates with untracked preceding history.
`TriangleMesh::from_construction` explicitly starts a new claim with supplied
qualifications. `ConstructionEvidence::DECLARED_COORDINATES` declares the current
coordinates as the source; it does not prove their relation to earlier intent.
Primitive constructors and free point-set hull construction make this source
boundary explicit in their domain. The mesh hull method also inherits its mesh
input's evidence. Modeling parameters are declared scalar inputs.

`with_inherited_construction` only adds qualifications. Supported native
operations use that invariant and combine all consumed parents, including empty
outputs. A new proof of topology cannot upgrade a construction claim. Replacing
or editing public buffers exposes missing history while retaining known
approximate, sampling and projection qualifications. Reauthoring equal raw
coordinates starts a new source claim; it does not restore the earlier evidence.

Geometry equality still compares coordinates and indices. Compare evidence
separately. Modeling caches also key on construction qualifications, so two
meshes sharing geometry buffers cannot exchange differently qualified outputs.

### Boolean batch API migration

`BooleanMeshBatch` now uses `Arc<[Point3]>` and `Arc<[BooleanMeshResult]>` with
private weak identity witnesses. Replace raw struct literals with
`BooleanMeshBatch::new(vertices, results)`; use `.iter()` for iteration,
`.to_vec()` for an owned vector, and `Arc::make_mut` for intentional edits.
Editing either shared buffer invalidates its construction identity. Malformed
carrier validation and typed failures retain their existing contract.

`into_triangle_meshes` preserves the summary for every row, including canonical
empty results, and shares positions across nonempty results. Per-triangle
`TriangleSource` mappings still belong to the batch and are discarded by this
explicit conversion; the summary does not replace them.

## Complete in-process STL path

With the `stl-io` feature, preserve a bound artifact through export/import:

```rust
use csgrs::{GeometryContext, Real, solid};
use csgrs::io::stl::to_stl_binary_artifact;
use hyperlattice::Matrix4;

let source = solid::cube(Real::one())?;
let shifted = solid::try_transform_with_context(
    &source,
    &Matrix4::affine_translation([3.into(), 5.into(), 7.into()]),
    &GeometryContext::STRICT,
)?;
let model = solid::boolean_with_context(
    &shifted.value, &solid::empty(), hypermesh::BooleanOp::Union,
    &GeometryContext::STRICT,
)?.value;
let artifact = to_stl_binary_artifact(&model, "model")?;
let imported = artifact.import()?;
let local = imported.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())?;
assert_eq!(local.certainty, hypermesh::MeshCertainty::Certified);
let inherited = imported.construction_evidence();
assert_eq!(inherited, model.construction_evidence().with_coordinate_projection());
# Ok::<(), Box<dyn std::error::Error>>(())
```

`BinaryStlArtifact` binds immutable bytes to the source summary within the
process. It encodes the same bytes as `to_stl_binary`; coordinates cross the
existing binary32 projection, and the exporter checks encoded triangles for
nondegeneracy. Import validates the parsed closed-PWN geometry. Neither operation
promises manifoldness or a geometric error bound, and the authoritative exact
source remains unchanged.

`bytes()` borrows the artifact; `into_bytes()` deliberately extracts bytes without
the receipt. Plain `from_stl(bytes)` reports missing source history and projection.
Identical STL bytes can originate from differently qualified constructions.
There is no API to restore a certified receipt from an unverified serialized
flag. Persistent proof-preserving interchange remains open.

JavaScript exposes `mesh.constructionEvidence` separately from outcome certainty.
`mesh.toStlBinaryArtifact()` returns an artifact with a read-only receipt, a
copying `bytes` getter, and `intoMesh()` for receipt-preserving import. Editing
that byte copy cannot change the artifact. C exposes
`csgrs_triangle_mesh_construction_evidence` for every scalar family; C++ and
Python provide `construction_evidence()`. These summaries outlive their queried
handles. The C ABI currently exposes strict construction APIs; this addition
does not introduce an approximate-policy constructor or bound STL ABI.

## Resource and acceptance scope

The summary occupies one byte before carrier alignment. Mesh binding reuses
existing weak buffer identities; Boolean batches add two weak identities.
There is no dependency graph. Weak identities retain backing allocations after
their values are dropped: editing or replacing a batch buffer can retain its old
allocation until the batch and its identity witnesses are dropped. Thus a
constant-sized summary does not imply constant retained heap usage, and edits
can increase peak memory. The admission measurements include this cost.

An STL artifact owns encoded bytes and a summary. Existing geometry, expression,
cache-byte and total-work limitations remain; the metadata does not add an
internal work or heap budget.

The regression corpus retains the twelve E98 exact/approximate/outside-point,
integral/fractional, cold/warm pipelines, including the original `2^-1200` point
and 512-bit modeling policy. It separately checks current geometry, local proofs,
inherited evidence, binary32 endpoint expectations, and mutation/cache/empty
boundaries. A delayed stronger scalar proof does not retroactively certify the
earlier construction. Admission evidence records actual executed configurations;
this document alone makes no release or full P19 completion claim.
