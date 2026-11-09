# Indexed PWN closure admission

Hypermesh can certify closed piecewise-constant winding-number input from a
balanced directed indexed boundary without first proving which distinct
position rows are geometrically coincident. This extends a previously undecided
case; it is not a source-size refactor or a claim of faster Boolean completion.

The owner is `validate_source_pwn` in
[`mesh.rs`](../../crates/hypermesh/src/mesh.rs), under P4, P6, P8–P10, P19 and
P21–P22. The input domain is finite indexed triangle input with admitted exact
coordinates. Existing nonempty, checked-index, representable-index and exact
triangle-nondegeneracy validation runs first, in its original order. Then the
integer edge-use pass checks whether every directed edge has equal forward
and reverse multiplicity. A zero indexed boundary stays zero under the fixed
index-to-point map. Coincident vertices, intersecting components, or repeated
closed components do not invalidate this sufficient PWN closure proof.

This certificate does not establish manifoldness, embeddedness, bounds, source
fidelity or a Boolean result. If the indexed boundary does not cancel, the
original exact geometric canonicalization and edge-balance path still runs,
with its predicate order unchanged. Separately indexed adjacent faces remain
supported. No coordinate welding, rounding or new public type is introduced.
The reusable `valid_pwn` fact retains the complete validation's certainty and
the existing source-buffer and policy guards.

The new strict witness contains two indexed closed cubes related by a shared
symbolic translation. Triangle nondegeneracy is decidable, but equality across
components is outside the current scalar normal form. On published
`29d8b8ce69`, validation fails with `PredicateUndecided` for 3D coordinate
equality. The corrected validator returns `Certified` under both STRICT and
APPROXIMATE_512, without consuming an approximate terminal.

Acceptance also checks balanced nonmanifold multiplicity, out-of-range indices,
collapsed triangles and reversed winding. A dispatch test checks exactly one
indexed proof or one geometric fallback, followed by reuse of the retained
proof under each policy. All 218 owning-crate unit tests pass with dispatch
instrumentation. Existing cache-mutation tests and coincident-index fixtures
remain unchanged. Full workflow and downstream qualification is required
before publication; no earlier revision's pass qualifies this change.

| Capability | Evidence boundary |
| --- | --- |
| Representation and coordinates | Existing exact positions and indices retained |
| Triangle validation | Existing nondegeneracy and index checks retained |
| PWN admission | New indexed proof; original geometric fallback retained |
| Bounds and incidence ordering | Separate predicates; no new guarantee |
| Boolean/reconstruction/interchange | Existing consumers; full downstream checks required |
| Offset, corner edits, general curve closure | Unchanged; no support claim |

This extension intentionally avoids geometric equality decisions when the
indexed proof suffices. Predicate trace counts on that route therefore change;
they are not claimed invariant. The geometric fallback keeps the original
decision sequence. Raw red and green runs and source snapshots are retained in
`docs/evidence/2026-11-09-openscad-corpus-and-indexed-closure` in the local
[evidence archive](../evidence-archive.md). No end-to-end performance improvement
is established by these tests.
