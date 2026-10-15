# HyperBRep repository and reference audit

Audit date: 2026-10-14. Repository baseline:
`47ae4a170e1c233252b26cc410fa210c407e4b34`.

This audit supplies the implementation evidence for the
[development plan](DEVELOPMENT_PLAN.md). Existing capabilities below describe
the baseline, not promises made by the plan. Repository-wide
[properties](../../../properties.md) and their normative
[admission protocol](../../../docs/admission.md) govern extensions.
The plan's [property mapping](DEVELOPMENT_PLAN.md#properties-and-acceptance-obligations)
covers all 39 properties and 14 known open targets. Their implementation and
verification status remains in the existing
[execution ledger](../../../docs/properties-status.json); this documentation
review does not upgrade that status or replace the unfinished future-work
inventory with a completion claim.

The follow-up [algorithm research](ALGORITHM_RESEARCH.md) examines feasibility,
primary literature, performance experiments and solver applicability against
revision `4529450d6`. It supplements this audit without changing the baseline
test result or claiming that the proposed algorithms have landed.

## Baseline verification

The following command completed successfully on the local native host:

```sh
cargo test --locked -p hyperbrep --all-features --lib
```

Result: **233 passed, 0 failed, 0 ignored**. The test executable took 19.01
seconds. Cargo 1.99.0 used an isolated Cargo cache and target directory under
`/tmp`. This checks the existing library, including tessellation; it is not a
full workspace, WASM, fuzz, documentation, or pre-push validation result.
None of the five external reference projects was built or benchmarked in
this audit.

## Scalar and predicate foundations

### Hyperreal

Read the [crate overview](../../hyperreal/README.md),
[scalar implementation notes](../../hyperreal/src/real/README.md),
[rational implementation notes](../../hyperreal/src/rational/README.md), and
the public scalar surface. `Real` combines exact representations, structural
facts, computable expressions, and certified enclosures. Its primitive
approximation cache is not the source of mathematical truth.

`PartialEq` is a representation relation. Failure of structural equality is
not evidence of geometric inequality. The accepted
[exact-decision ADR](architecture/0001-exact-decision-contract.md) already
explains this distinction, including an equality that remains undecided by
the bounded implementation. General computable-real equality must not be
promised to terminate.

The rational layer already invests in borrowed and fused arithmetic,
deferred reduction, shared denominators, small exact kernels, and cache
lifetime control. BREP should consume these paths rather than create a
second scalar or force repeated rational normalization. Exact lifting of a
finite floating value preserves its binary value; exact decimal input must
retain the decimal rational instead of passing through a float.

### Hyperlimit

Read the [crate overview](../../hyperlimit/README.md), predicate outcomes,
resolution code, and orientation implementations. Outcomes preserve both
certainty and escalation stage. A certified filter result is authoritative;
an approximate terminal result is not admissible for BREP topology.

The cascade is operation-dependent. Orientation front doors can use certified
`Real` filters and specialized rational filters before constructing a generic
expression. Generic resolution uses structural evidence, certified filters,
exact callbacks, bounded refinement, and applicable algebraic identity
proofs. Describing every predicate as a fixed floating-point-to-big-rational
ladder would miss this design and encourage unnecessary expression creation.

The current policy exposes `STRICT` and `APPROXIMATE_512`; it is not a general
per-operation resource manager. BREP must preserve `Unknown`, exact/filtered
certainty, and useful stage information through its own error boundaries.
New scalar predicates belong here; topology composition belongs in BREP.

### Hypersolve

Read the [overview](../../hypersolve/README.md), public modules, selected
algebraic field implementation, and
[work budget](../../hypersolve/src/work_budget.rs). Existing machinery includes
univariate isolation, multiplicity, exact polynomial arithmetic, resultant
and fiber operations, common components, ordered fields, and recursive
quadratic representations. This is substantial infrastructure, but not a
complete general trimmed-surface intersection algorithm.

A selected algebraic field value retains its source root and polynomial
relation. An independently materialized `Real` is optional. That distinction
is essential for comparing coordinates from the same intersection and for
retaining branch identity after subdivision.

The existing synchronous, thread-local budget accounts for refinements and
promotions. Nested calls share charges. It does not by itself bound all
coefficient growth, allocation, elimination cells, execution time, or work
on other threads. Some error translations coarsen budget failure into
undecided outcomes; implementation must preserve the original distinction
where these become BREP dependencies.

### Hypercurve

Read the [overview](../../hypercurve/README.md), public API, retained point and
parameter types, selected scalar implementation, Boolean arrangement code,
subdivision, and offset code. Hypercurve owns planar curve topology, filled
regions, native line/circle and polynomial/rational spline carriers, retained
algebraic contacts and charts, overlaps, singularities, and offsets.

`CurveRegion2` is a filled region; an oriented `CurvePath2` is not a substitute
for region semantics. Its Boolean arrangement can supply all four Boolean
answers from shared work. Curve parameters and geometric points can retain
selected algebraic evidence when ordinary coordinates or split control
points cannot be materialized. These are the patterns BREP needs to extend
into 3D.

Exact parallel curves are not generally rational Béziers. Hypercurve already
separates retained analytic offsets from fitted approximations and their
verification. Surface offsets need the same distinction. An outer
provisional/approximate evaluation scope must never weaken a nested BREP
construction. Generic selected-root algebra belongs in Hypersolve; planar
chart and incidence logic stays in Hypercurve.

## HyperBRep implementation

The eight source files contain roughly 67,500 lines including embedded tests.
Most functionality lives in four large files. Existing code should be
retained, split into modules, and brought under more general certificates;
this is not an empty-crate implementation task.

| Source | Existing responsibility | Main constraint |
| --- | --- | --- |
| [geometry.rs](../src/geometry.rs) | Curves, surfaces, derivatives, bounds, parameter location, analytic and selected spline intersections | Ordinary edge parameters and vertex coordinates require materialized `Real` values; generic intersections are incomplete |
| [topology](../src/topology/mod.rs), `model.rs` at the audited baseline | Immutable arenas, adjacency, validation, classification, measurements, transactions and splitting | Solid publication depends on recognized shell families; general curved-shell validity is missing |
| [builder.rs](../src/builder.rs) | Primitives, extrusion, revolution, supported sweeps/lofts, spline patches | Many exact but deliberately restricted construction domains |
| [boolean.rs](../src/boolean.rs) | Specialized Booleans, intersection graph, face partition/classification/stitching | Curved coincidence, retained endpoints, general face partitioning and output validation limit closure |
| [persistence.rs](../src/persistence.rs) | Version 8 JSON and untrusted replay through validation | No general retained intersection/proof graph format or comprehensive ingestion budget |
| [tessellation.rs](../src/tessellation.rs) | Exact planar triangulation and explicit chordal approximation | Exact sample incidence does not certify interior approximation error or whole-body watertightness |
| [error.rs](../src/error.rs) and [lib.rs](../src/lib.rs) | Public error/type boundary and strict policy | Error families and resource diagnostics need a coherent public contract |

### Geometry and construction already present

Spatial carriers include lines, circular and elliptic arcs, rational Béziers,
and finite non-periodic NURBS. Surface carriers include planes, cylinders,
spheres, cones, ring tori, extrusion, revolution, rational Bézier tensors,
and NURBS tensors. Derivatives, selected exact bounds, subdivision, and
family-preserving transformations exist.

Constructors include shared-edge cuboids, native cylinders, complete spheres,
cone frusta, periodic torus patches, planar faces, spline patches and patch
shells, line/arc/spline extrusions with holes, and supported revolutions with
cavities. The current full-sphere representation can be a boundaryless face.

Sweeps already include affine linear sweeps, fixed-frame rational paths with
certified affine positive axial progress, and explicit rational moving frames
with restricted section planes and area behavior. Lofts already support
corresponding polygons, homothetic sides, and certified convex bilinear spans.
These are valuable vertical slices, not arbitrary-path sweep or arbitrary
section loft implementations. Full cone apices and general singular charts
need additional topology.

Measurements include many exact family-specific areas and volumes, sometimes
using sophisticated rational integration. A lack of elementary area or volume
must not prevent a valid general curved solid from existing.

### Ownership, identity and validity

The current `Model` is immutable and `Arc` shared, with private typed-index
arenas and retained adjacency. Edges own spatial carriers; edge uses retain
pcurves and parameter correspondence. Builder publication and edit commit
revalidate the result. These foundations should remain.

Raw typed indices do not carry model identity or generations. Some editing
paths compact or remap records. Consequently, indices are not durable feature
names and must not be advertised as stable references across arbitrary
recomputation. Model-bound references, explicit edit maps, and downstream
feature naming solve different problems.

Solid admission in `ModelBuilder::solid` tries certificates for spheres and
sphere combinations, revolutions, cylinders, frusta, tori, prisms, sweeps,
lofts, and planar shells. Failure to recognize a supported family can produce
`UnsupportedSolidShell` even when a shell is mathematically valid. General
curved-shell embedding, orientation, nesting, and material classification are
the main architectural bottleneck.

### Intersections and Boolean closure

There are substantial exact analytic cases, including transverse and axial
plane sections, aligned/coaxial quadrics, selected torus cuts, native
revolution-profile sections, extrusion sections, and selected rational tensor
sections. Many retain both surface pcurves, multiplicity, and
mixed-dimensional components. This should not be described as a complete
quadric-by-quadric or NURBS-by-NURBS intersection matrix.

The general graph path already partitions operands, classifies witnesses,
selects face pieces, and stitches identity-shared boundaries. Specialized
paths cover prisms, parallel cylinders, coincident frusta, identical tori,
spheres, and other restricted combinations. The public operations are union,
intersection, and difference; a complete symmetric-difference API remains
work.

The graph computes reusable face bounds, then scans face pairs with nested
loops and conservative rejection. A hierarchy can improve candidate
generation, but cannot decide contact from an uncertified box or replace the
intersection proof. Curved coincident regions and general resulting curved
shells remain important gaps after a contact has been found.

### Persistence and derived outputs

Version 8 JSON reconstructs through validation rather than trusting cached
certificates. Deserialization disables the normal recursion limit and uses
`serde_stacker`; the replacement format needs explicit depth, size, reference,
and work limits, including a WASM-compatible implementation.

Chordal tessellation reports `ExactAtVerticesOnly`. This is an honest,
limited guarantee. It does not establish a Hausdorff error bound, prevent
triangle inversion, or prove consistent sampling of a shared edge on all
incident faces. Preserve the distinction while adding certified meshing.

## Whole repository ownership

These packages and downstream modules were reviewed for ownership and
integration. A row assigning no new kernel work is still a deliberate scope
decision; the BREP effort must not pull unrelated domain policy into the
geometry stack.

| Package or module | Existing role and planned relationship |
| --- | --- |
| [hyperreal](../../hyperreal/README.md) | Sole authored scalar, exact arithmetic and certified enclosures; generic arithmetic improvements only |
| [hyperlattice](../../hyperlattice/README.md) | Exact fixed-dimensional vectors, matrices, transforms and structural facts; reusable coordinate kernels, no BREP topology |
| [hyperlimit](../../hyperlimit/README.md) | Certified predicates and cascades; generic incidence/separation/comparison extensions and instrumentation |
| [hypersolve](../../hypersolve/README.md) | Selected roots, polynomial and algebraic elimination, work accounting; generic certificates needed by 3D intersections |
| [hypercurve](../../hypercurve/README.md) | Planar arrangements, Boolean regions, offsets and retained contacts; authoritative surface-chart trimming |
| [hyperbrep](../README.md) | Spatial carriers, boundary topology, solid validity, modeling, geometric queries and exact/source-relative certificates |
| [hypertri](../../hypertri/README.md) | Straight planar triangulation; optional tessellation component, never the owner of curved trim semantics |
| [hypermesh](../../hypermesh/README.md) | Exact triangle arrangements and Booleans, exact spatial queries; explicit mesh adapters and candidate-index design reference |
| [hypersdf](../../hypersdf/README.md) | Implicit field semantics and certified field queries; explicit BREP/field adapters coordinated with its existing meshing plan |
| [hypervoxel](../../hypervoxel/README.md) | Exact grid occupancy and conservative spatial products; separate excluded workspace and derived adapter |
| [hypergraphics](../../hypergraphics/README.md) | Rendering and explicit float conversion; BREP tessellation, selection IDs and diagnostic overlays |
| [hyperphysics](../../hyperphysics/README.md) | Material/density, physical properties and contact; consume certified BREP geometric moments instead of presenting triangle moments as exact curved-body moments |
| [hyperpath](../../hyperpath/README.md) | Routing, paths, CAM and domain constraints; consume sections, offsets, projections and clearance facts |
| [hyperpack](../../hyperpack/README.md) | Packing and replay; use certified bounds and sections, then exact final interference checks |
| [hyperparts](../../hyperparts/README.md) | Sourced part knowledge, geometry handles, units and identity; reference BREP/document geometry without owning it |
| [hypercircuit](../../hypercircuit/README.md) | Circuit/PCB design semantics; use assemblies and enclosure/clearance queries |
| [hyperdrc](../../hyperdrc/README.md) | Domain validation and release policy; consume explicit geometric certainty and blockers |
| [hyperevolution](../../hyperevolution/README.md) | Proposal search, archive and replay; possible test-case minimization and design search, no authority over topology |
| [csgrs-ffi](../../csgrs-ffi/README.md) | Existing opaque-handle native bindings; add BREP handles and stable error/certainty encoding |
| [comparisons](../../comparisons/Cargo.toml) | Separate native comparison infrastructure and external dependencies; host optional BREP oracle programs in a separately runnable subproject |
| [csgrs source](../../../src/lib.rs) | Currently a mesh and planar-curve grammar; add an optional native BREP grammar and feature evaluation layer |
| [csgrs parts](../../../src/parts/mod.rs) | Existing assemblies, attachment frames, BOM, drawings and manuals; extend geometry references and placement to BREP |

The root crate does not currently depend on HyperBRep. `src/parts` already
has an assembly document and uses triangle geometry. Introducing a second
assembly system in the kernel would duplicate ownership. Likewise,
`hyperparts` is not an existing parametric CAD document implementation.

Hypermesh admits triangle-set arrangements broader than manifold CAD solids.
Mesh-to-BREP conversion must check the target body's manifold contract.
An SDF expression can be sign-equivalent without being a true metric distance;
adding a constant to an arbitrary field cannot be assumed to implement a CAD
offset. Voxel occupancy and display triangles remain derived representations.

## External references

### stl2step

Reviewed revision: `74a4546525814515349794a2a6fdaa5d26903e78`.

The public options expose units, scale, welding, sewing, unification,
reconstruction and validation controls. The result accounts for solids,
shells, face counts, warnings and reconstruction statistics. Its implementation
uses OCCT for construction, repair and STEP exchange. These are workflow and
diagnostic references, not a production dependency for the pure Rust kernel.
See the pinned [public header](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/include/stl2step/stl2step.hpp)
and [conversion implementation](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/src/stl2step.cpp).

Its reconstruction contracts address triangle-region ownership, shared boundary
chains, explicit closed-loop roles, deterministic junction splitting, and
signed volume changes from replacing chords by arcs. These are useful design
inputs for a future reconstruction layer. HyperBRep should retain full input
component accounting and certify the proposed replacement independently.
See the [reconstruction contracts](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/src/refit.hpp).

The source contains prismatic reconstruction, cone/chamfer work, and torus
blend mathematics; the older README limitation list does not fully describe
this revision. They use numerical fitting and restricted geometric
recognition, not proof of original design intent or a general freeform
reconstruction algorithm. Inspect the pinned
[prism interface](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/src/refit_prism.hpp),
[cone implementation](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/src/refit_chamfer_cone.cpp),
and [torus mathematics](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/src/refit_torus_math.hpp).

The local converter has warning/error paths that can omit failed components;
do not adopt an unconditional claim that every component survives. The planned
import API must distinguish accepted, rejected, and retained raw components,
and forbid a partial document from masquerading as a complete conversion.
Project source and OCCT have separate licenses; preserve provenance and
review notices before any code reuse. The
[project license](https://github.com/BlinkingSun/stl2step/blob/74a4546525814515349794a2a6fdaa5d26903e78/LICENSE)
does not license OCCT on its own terms.

### solidcore

Reviewed revision: `ebfe5921e4dfd3f927c37e20f35607e0407d8328`.

The progress ledger reports 145 of 261 units with tests, including 89 green
and 56 partial units. Unit counts are not general CAD readiness. The Boolean,
blend and offset modules at this revision are documentation stubs. Its useful
implemented material includes curve/surface geometry, NURBS work and
intersection machinery. See the pinned
[progress ledger](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/plan/PROGRESS.md),
[Boolean module](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/boolean.rs),
[blend module](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/blend.rs),
and [offset module](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/offset.rs).

Its explicit kernel context, deterministic testing, operation dependency
inventory and differential harness are useful organizational references.
Its `f64` tolerance policy and mutable topology interface do not satisfy
HyperBRep's contract. See the
[kernel context](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/kernel.rs),
[operation closure inventory](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/plan/closure.md),
and [differential harness](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/difftest/README.md).

Surface tracing and NURBS offset fitting can inform candidate generation, but
numerical marching and sampling do not certify that all components were found
or that an offset is exact. Compare the
[surface tracer](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/intersect/surface_tracer.rs)
and [offset fitter](https://github.com/bherbruck/solidcore/blob/ebfe5921e4dfd3f927c37e20f35607e0407d8328/crates/solidcore/src/nurbs/srf_offset.rs).
Track upstream NVIDIA source attribution when adapting algorithms from this
Apache-2.0 port; do not treat its oracle or its tests as mathematical ground
truth.

### NVIDIA USD BRep

Reviewed revision: `c9c5979f04cea8c38213004d4f04df0dcfb40d05`, also the
upstream revision identified by Solidcore's comparison work.

This repository contains the SMLib geometry/modeling implementation, a higher
level `SM_API`, Python bindings, USD conversion and schema code, staging,
utilities, and Hydra integration. Its public modeling surface includes
Booleans and imprinting, sweeps, fillets, offsets, shelling, projection,
stitching and tessellation. Unlike the Rust port's high-level stubs, there are
substantial implementations to study. See the pinned
[modeling API](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SM_API/inc/SmApiBrep.h).

The Boolean coordinator performs intersection/insertion/relationship work
before result selection. The fillet coordinator constructs edge and corner
geometry and integrates it with the target. The offset coordinator tracks
source faces, edges and vertices while creating displaced faces and gap
closures. These decompositions reinforce sharing one intersection,
arrangement and stitching infrastructure across modeling features. Their
mutable, tolerance-driven implementation must be redesigned around retained
certificates and atomic immutable publication. See
[SmMerge.cpp](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp),
[SmFilletExecutive.cpp](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmFilletExecutive.cpp),
and [SmOffsetExecutive.cpp](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmOffsetExecutive.cpp).

Topology-change callbacks distinguish additions, removals and geometry
changes, with explicit pointer-reuse caveats. The public shell wrapper checks
that selected faces belong to its operand, copies private working data, and
handles the lower-level ownership variants before returning. HyperBRep should
provide stronger model-scoped references and semantic split/merge history,
and make unchanged-on-failure universal. See
[change tracking](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/inc/SmTrackTopologyChanges.h)
and the [shell wrapper](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SM_API/src/SmApiBrep.cpp).

The USD schema packs topology counts, indices, face uses, edge uses, radial
relationships, analytic/NURBS geometry and UV domains into arrays. Staging
handles source traversal, explicit and natural boundaries, and index mapping.
This is an interchange layout, not an appropriate in-memory replacement for
HyperBRep's immutable certified model. Its scalar arrays also cannot directly
preserve arbitrary Hyperreal or selected algebraic values. See the
[schema](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/schema/omniSolid/resources/schema.usda)
and [source staging data](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/BREP_STAGING/inc/BrepSourceData.h).

The known-issues document supplies concrete regression targets: success
without requested fillet removal, failed healing reported as success,
periodic seam/pole damage, cavity merging, reversed mesh triangles, trimmed
measurement error, and malformed packed topology crashing import. These
reinforce independent result validation, exact seam semantics, bounded raw
parsing, and complete failure accounting. They also disqualify the reference
from serving as an unquestioned oracle. See the pinned
[known issues](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/KNOWN_ISSUES.md).

Use the operation inventory and selected independent constructions for native
validation. Keep OpenUSD, OCCT export, Python and Hydra out of the production
Rust dependency graph. Later USD exchange belongs in the IO/document layer;
display integration belongs downstream. Preserve source attribution and
inspect each reused component's terms, including the separate dependencies
listed in [third-party notices](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/THIRD_PARTY_NOTICES.md).

### Monstertruck

Reviewed revision: `d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652`.

Monstertruck is an Apache-2.0 Truck fork with separate topology, geometry,
modeling, Boolean, fillet, healing, assembly, exchange, meshing and rendering
crates. Its Rust implementation and WASM surface make it a useful integration
reference. The concrete geometry and tolerance contracts remain substantially
`f64` based; the scalar-generic trait work does not establish Hyperreal-style
exact decisions. See the pinned [workspace overview](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/README.md).

The default Boolean backend triangulates operand shells, derives intersection
polylines from mesh interference, recovers surface parameters numerically,
partitions faces, and classifies remaining faces with ray crossings against
the tessellations. Typed errors and preserved curve representations are
useful API patterns, but they do not make this pipeline an exact topology
oracle. HyperBRep must retain its complete certified intersection and
classification requirements. See the [classic Boolean pipeline](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-solid/src/transversal/classic/mod.rs)
and [intersection recovery](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-solid/src/transversal/classic/intersection_curve.rs).

The fillet API separates constant, variable and per-edge radii, profiles,
chain selection and topology edits. Its conversion layer refuses unsupported
curve-to-NURBS conversion but can sample a surface into a degree-one NURBS
grid; edge remapping there uses endpoint proximity. Chain processing also
contains skip paths after failed rematching or geometry construction. These
motivate explicit conversion guarantees, semantic identity maps and complete
requested-edge accounting in HyperBRep. Study the
[options](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-fillet/src/params.rs),
[conversion layer](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-fillet/src/convert.rs),
and [chain coordinator](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-fillet/src/edge_select.rs).

Healing separates closed-edge/face splitting, seam processing and pass-through
edge handling. The repeated seam-use fixture is especially useful: one
spatial edge can occur twice with opposite directions on the same periodic
face. HyperBRep should preserve its chart gluing explicitly rather than
adopt seam deletion as a universal repair. See the
[seam pass](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-healing/src/strip_seam_edges.rs).
Its [topology records](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-topology/src/lib.rs)
share mutable geometry with `Arc<Mutex<_>>` and also carry stable IDs;
HyperBRep should retain immutable revisions and its separate semantic history.

STEP conversion reports distinguish lost faces, wires, edge uses, edges,
vertices, representations and solids, with typed reasons. The ordinary APIs
can discard the reports; strictness is opt-in. HyperBRep should make complete
accounting mandatory and expose partial import through a distinct result.
See the [load reports](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-io/src/step/load/report/mod.rs).
The [manifest](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-io/Cargo.toml)
uses `step-p21`, a Ruststep fork worth including in the parser review.
The mapped [point and placement types](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-io/src/step/load/step_types/placement.rs)
use floating coordinates, so their end-to-end mapping cannot simply preserve
exact decimal authoring unchanged.

Assembly nodes hold shared shapes and attributes; occurrence edges hold
transforms and attributes. The STEP writer maps this structure into product
and representation relationships. Keep that definition-versus-occurrence
distinction in the existing csgrs assembly layer. See the
[assembly types](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-assembly/src/assy.rs)
and [STEP assembly writer](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-io/src/step/save/assembly/mod.rs).

Surface subdivision explicitly distinguishes recursion depth from total cell
work and exposes truncation. Legacy methods can return a coarser grid, while
fallible methods report refusal. Transfer the measurable-work lesson, but
never silently weaken a requested HyperBRep certificate. See the
[subdivision budget](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-traits/src/algo/surface/mod.rs).
The [continuity capability types](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-traits/src/surface_continuity/mod.rs)
distinguish degree, control-row, periodic and boundary limitations; this is
useful diagnostics vocabulary, separate from proving continuity of a join.
The [active B-spline basis window](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-geometry/src/nurbs/mod.rs)
is a performance reference for avoiding full control-net work.

The current [cadmpeg conversion code](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-io/src/cadmpeg/mod.rs)
contains actual body conversion and distinguishes solids and sheets; the root
README's older description of an unwritten IGES conversion is stale. Broader
T-spline/T-NURCC representation also exists in the
[geometry module](https://github.com/virtualritz/monstertruck/blob/d2ad1ed7aa420b7f32cd2c1e3ad319b46d593652/monstertruck-geometry/src/t_spline/mod.rs).
These inform later interchange and authoring adapters; neither is required
as a new authoritative HyperBRep carrier for the first release.

### rust-cad

Reviewed revision: `417e8fcea322d2727557bed6ea7b1e9f48820777`.

This repository combines a vendored Monstertruck kernel with a geometry
verification and experimental patch-search lab. Its checked-in kernel is the
same `d2ad1ed7` revision reviewed above. A local byte comparison found all 538
tracked vendored files compared identical to the Monstertruck checkout.
Consequently these are not independent kernel implementations or independent
differential votes. See the [repository description](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/README.md).

The probe emits geometry and topology observations; a separate evaluator
judges them. Its gate rejects regressions in previously passing cases, new
incorrect successes, new runtime failures and comparisons across evaluator
revisions. Kernel tests and release checks are additional gates. This is a
useful model for the BREP comparison harness, without adopting or running an
autonomous patch-search service as part of this plan. See the
[baseline comparator](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/lab/compare_baseline.py)
and [kernel gates](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/lab/gates.py).

The evaluator revision history demonstrates why the checker itself needs
tests. Judging each shell separately avoids welding valid touching bodies
together. Subdivision-aware boundary checks expose contacts hidden by raw
edge counts. Degenerate pole triangles and mesh conformance are distinct
from source-shell validity. The reported Euler and body/cavity expectations
are specific to the recipe families, not universal invariants for all CAD.
See the [evaluator changes](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/lab/EVALUATOR-CHANGES.md).

Its numerical oracle compares area, volume and centroid, allowing one-percent
error for several curved families, and checks shell incidence. Such metrics
are useful rejection tests but cannot prove shape equality or exact topology.
The standalone STEP-import check requires nonempty output; it does not prove
that every source component survived. HyperBRep needs stronger exact witnesses
and complete import accounting. See the
[oracle](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/lab/oracle.py)
and [evaluator](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/lab/verify.py).

The round-two research record reports 420/420 recipe passes and 351/351 fresh
sealed passes for its retained patch stack, alongside kernel gates. These
are reported results, not runs reproduced here; the patches are explicitly
not applied to the checked-in kernel. The record also notes a contact-audit
timeout followed by a successful unchanged retry and limited geometric
domains. See the pinned [round-two record](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/research/2026-10-13-round2/README.md).

Selected patch excerpts provide useful regression recipes: fillet end trims
should retain rational sections rather than chords, pole UV duplication
must retain the same spatial pole, small interior loops must survive, and
STEP products may reach solids through non-transforming representation links.
The patch still uses numerical proximity and sampled fallback paths; its
"exact" sections are not evidence for the Hyperreal decision contract.
Promote root-cause lessons through independent tests, not wholesale patch
application. See the [retained patch](https://github.com/BohnBawerick/rust-cad/blob/417e8fcea322d2727557bed6ea7b1e9f48820777/research/2026-10-13-round2/kept.patch).

### Algebraic and STEP design references

The CGAL algebraic-kernel manual illustrates retaining a polynomial plus an
isolating interval, and representing bivariate solutions through a selected
fiber instead of eagerly extracting independent coordinate values. This
supports the proposed representation approach; it is not a proposed CGAL
runtime dependency. See the [primary manual](https://doc.cgal.org/latest/Algebraic_kernel_d/index.html).

OCCT documents STEP geometry/topology and assembly translation separately
from associated names, colors and other document attributes. HyperBRep's
exchange layer should make that scope equally explicit. See the
[STEP translator guide](https://occt3d.com/dev/doc/overview/html/occt_user_guides__step.html).

Ruststep is a possible pure Rust syntax/schema dependency, not an adopted
kernel dependency. Its own README describes an experimental toolkit and
notes exceptions for development schema and fixture directories. Exact
decimal preservation, writing support, schema coverage, WASM behavior and
redistribution rights must pass a dependency review before reuse. See the
[project overview](https://github.com/ricosjp/ruststep).

## Validation ownership and current CI coverage

All twelve workflow files matter to the repository's pre-push contract.
Commands below are an execution inventory, not a substitute for reading the
current workflow, environment, working directory and called scripts.

| Workflow | Locally reproducible obligations |
| --- | --- |
| [rust](../../../.github/workflows/rust.yml) | Root no-default/default/all-feature builds and tests, formatting, strict Clippy, typos, WASM release-profile build, matching wasm-bindgen, Node smoke tests and strict TypeScript checks |
| [workspace](../../../.github/workflows/workspace.yml) | Workspace all-target check; individual package tests including BREP; release DRC tests; excluded Hypervoxel workspace tests and formatting |
| [adversarial](../../../.github/workflows/adversarial.yml) | Both adversarial scripts, including feature subsets and documentation tests; honor configured optional fuzz duration |
| [csgrs-ffi](../../../.github/workflows/csgrs-ffi.yml) | Locked check/test, formatting, strict Clippy and docs, C header and C++ header/binding compilation |
| [hyperreal](../../../.github/workflows/hyperreal.yml) | Pinned Verus installation and verification, rejection and acceptance guards, Rust check/test/lint/format, fuzz-bin checks |
| [hyperlattice](../../../.github/workflows/hyperlattice.yml) | Default/all-feature checks, tests and Clippy, formatting, fuzz-bin check |
| [hypercurve](../../../.github/workflows/hypercurve.yml) | All-feature Clippy; release all-feature tests with two test threads; demo tests, Clippy and release WASM build |
| [hypersolve](../../../.github/workflows/hypersolve.yml) | All-target/all-feature and no-default tests, representation script, allocation script with configured iteration count, fuzz check and specific fuzz Clippy, formatting, docs and coverage script |
| [hypertri](../../../.github/workflows/hypertri.yml) | Cargo-hack feature powersets, explicit all-algorithm/all-feature tests, representation script, fuzz check, docs, UI check and coverage script |
| [hypermesh](../../../.github/workflows/hypermesh.yml) | Default/no-default/all-feature checks, tests, Clippy and docs; bench compilation; fuzz check; native/WASM size harness builds; Trunk UI build |
| [hypergraphics](../../../.github/workflows/hypergraphics.yml) | Default/all-feature and no-default test/lint/doc matrices, WASM checks, fuzz format/check/lint, package listing, MSRV checks |
| [pages](../../../.github/workflows/pages.yml) | Release Trunk builds for all three demos and site staging; hosted deployment itself is an external action |

There is no dedicated HyperBRep workflow at this baseline. Its current
workspace coverage does not provide a complete BREP feature, WASM, fuzz,
documentation and performance matrix. The development plan adds that matrix
without weakening any existing workflow.

Before a future push, [AGENTS.md](../../../AGENTS.md) additionally requires
Clippy with `-D warnings` for every feature/target/manifest combination
exercised by CI, even combinations whose workflow step only builds or tests.
CI or pre-commit edits require `zizmor --pedantic --strict-collection .`.
All intended fixes must be committed and the worktree clean before pushing.
Only genuinely unavailable hosted, secret-dependent or foreign-OS steps may
be excluded, with explicit reporting.
