# Named modeling workflow corpus

This catalog identifies regression gates and their finite input domains.
The [admission protocol](admission.md) remains normative. Admission records in
the [execution ledger](properties-status.json) identify the revision, commands,
feature configurations and observed results; this catalog alone proves no
implementation claim. Tests require the stated completion and preserve the
original difficult fixtures.

| Workflow | Domain and semantic evidence | Regression gate |
| --- | --- | --- |
| Transformed box Booleans | Nine configurations under five invertible affine maps; exact represented-set equality, separate PWN validity, subsequent composition | [strict_boolean_workflows.rs](../tests/strict_boolean_workflows.rs) |
| Drilled bodies and removal chains | Inscribed polygonal drills with 8, 16, 24 and 32 angular samples; strict membership, certified manifoldness, independently derived volume enclosure, complete subtraction and reuse of empty | [strict_drilled_workflows.rs](../tests/strict_drilled_workflows.rs) |
| Empty results | Valid-set identities, disconnected and symbolic operands, invalid operands paired with empty, subsequent operations | [empty_composition.rs](../tests/empty_composition.rs) |
| Transformed primitives | Exact scaling, translation, rotation and reflection; volume and topology; finite projective transforms and typed unbounded/invalid/undecided outcomes | [transform_failures.rs](../tests/transform_failures.rs) |
| Profile extrusion with a hole | Rational line profiles, including a translation beyond binary64 range; exact area and volume, manifoldness, native diagnostics | [exact_linear_modeling.rs](../tests/exact_linear_modeling.rs) |
| Construction evidence through interchange | Twelve exact/qualified hull pipelines, including the original tiny outside point; local proofs, inherited qualifications and binary32 endpoint semantics checked independently | [construction_evidence.rs](../tests/construction_evidence.rs) |

## Transformed box domain

Each source box is an exact rational polyhedron. The named configurations are
overlap, coplanar faces, face touching, edge touching, point touching,
disconnected components, coincidence, strict nesting, and an overlap of exactly
`1/10^12` next to a coplanar face. Each runs under `STRICT` with:

- Identity.
- Rational shear and translation, with determinant one.
- Reflection and nonuniform scale, with determinant minus six.
- A 45-degree rotation with exact `sqrt(2)/2` coefficients.
- Translation by `pi`, `sqrt(2)` and `ln(2)` on the three axes.

Each of the 45 pairs checks 21 outputs: both transformed inputs; union,
intersection, difference and XOR before and after transformation; commuted
union and intersection; reunion of difference and intersection; union and
intersection idempotence; complete subtraction; and rebuilding from empty.
Every completed operation must report `Certified`. Nonempty output must
separately pass certified closed-PWN validation. Canonical empty is checked at
the modeling boundary because the native PWN kernel requires nonempty input.
Edge/point-touching and XOR outputs are not promised manifold by this gate.

## Equality criterion and independent oracle

The [oracle](../tests/support/strict_boolean_workflows/oracle.rs) uses explicitly
specified inverse affine charts to recover exact rational coordinates. It
never rounds coordinates and never calls native Boolean or containment code.
Failure to prove a rational chart coordinate is an oracle limitation, not
evidence that the geometry is incorrect.

Every nondegenerate output triangle must lie on a source coordinate plane and
inside source bounds. Those planes partition space into open cells without
output boundary. Exact signed ray crossings establish winding at one
nondegenerate point per cell; the winding is constant throughout that cell.
The expected membership comes directly from the two box inequalities and the
Boolean truth table. Equality on these open cells establishes equality of the
regularized sets, including their closures. Signed tetrahedral volume is also
checked against the independent sum of occupied-cell volumes.

The ray samples follow a rational curve in each cell. Any projected triangle
edge line intersects that curve at most twice, so `6*T+1` trials suffice for
`T` triangles. Controls in every coordinate frame reject a disjoint box with
the same volume. A hand-retriangulated face keeps the exact set while putting
an internal vertex on the first sample ray, requiring the retry path.

Semantic equality is independent of vertex order, triangle count and
serialization. These tests establish facts about their exact polyhedral sets;
they do not establish fidelity to an analytic surface or certify earlier
approximate construction intent.

## Scope and resource limits

The format-boundary workflow in
[`tests/gltf_empty_composition.rs`](../tests/gltf_empty_composition.rs) runs
under `gltf-io`: empty JSON/GLB scenes and named or mixed empty objects continue
through import, transform, strict Boolean identities, complete subtraction,
and export. Exact box boundary planes and signed volume supply independent
geometry expectations; closed-PWN and manifold checks remain separate.
Missing source history survives subsequent local certification. The suite
also preserves exact rows for implicit and unsigned indices, interleaved and
sparse positions, nested transforms, scene selection, and the original
10,000-node chain. Malformed accessor and graph fixtures must return errors,
including when no triangles could otherwise be published. Serialized byte
stability and independent format validation are recorded separately by the
change's admission evidence.

The PLY workflow in
[`tests/ply_projection_contracts.rs`](../tests/ply_projection_contracts.rs)
checks exact decimal spelling of finite binary64 projections, including every
finite exponent family, subnormals and small normal components. A third-party
PLY reader reconstructs a tiny closed box for separate exact bounds/volume,
topology, transform and strict Boolean checks. This is consumer-side
reconstitution, not a new CSGRS PLY importer or proof of source intent.
The serialization migration is listed in [`MIGRATIONS.md`](MIGRATIONS.md#serialized-output-and-geometry).

The strict success gates remain required successes. An undecided predicate,
unsupported representation, invalid input, resource exhaustion or incorrect
result is a separate outcome, never a substitute passing status. Approximate
completion is not accepted by changing this corpus's policy. Existing typed
failure tests remain separate from these valid-input success gates.

Admission runs retain external time and address-space containment. That does
not establish internal total-work, retained-heap or cancellation guarantees.
These finite fixtures do not exhaust all meshes, transformations, symbolic
expressions, cache histories or thread schedules. Full workflow coverage,
end-to-end provenance and the clean shipped-user-path validation remain open.
