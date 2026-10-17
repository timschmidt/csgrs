# Certified bounds before surface arrangement

Base revision: `311d04fbbe5635a29fe7e754b838cccc4b8add53`.

The owning layer is native Hypermesh containment. P4/P6/P8/P9/P10/P17/P18/
P22/P24/P25/P30/P36 apply. The public operation remains strict interior of
its regularized nonzero-winding set. Canonical empty, malformed-source,
qualified-source, symbolic-coordinate and internal-source-face behavior stays
in scope. Source validation must precede every bounds shortcut.

A finite closed PWN source has winding zero outside its position AABB. A point
on a certified supporting plane of that AABB has arbitrarily close exterior
points, hence cannot be strict interior. This proof needs source validity,
bounds and coordinate equality to be Certified. An approximate equality or
already-qualified dependency does not enable the new shortcut. Existing
outside-bounds behavior, certainty receipts and fallback algorithms remain.
No new retained cache, geometry conversion, representation or API is added.

Before implementation, rerun independent correctness controls and current
cost probes using the qualified E32 production libraries. Verify that every
current runtime input matches that snapshot; admission metadata is the only
permitted baseline difference. Reuse both existing allocation probes unchanged.
Both sides of the final comparison must complete the same Certified results.

Acceptance preserves the entire E32 strict-success corpus and adds all six
supporting planes with exact 1/10^12 offsets, retained pi translation, bounds
that contain points outside the actual tetrahedron, mutated invalid sources
and policy-qualified query coordinates. Real WASM clients exercise exact
binary64 near-boundary inputs. No epsilon, fixture reduction, new Unknown for
an old Certified result, or weaker validation is acceptable. Newly sufficient
proofs may strengthen an undecided query only with a certified non-interior
witness; such a gain is recorded separately from timing comparisons.

Use five alternating fresh-process pairs for the unchanged 21-case matrix and
tetrahedron control, with all raw observations retained. Require all candidate
queries to remain correct/Certified, a reduction of at least half the warm
allocation calls on each global-boundary matrix case, and reduced allocation
work in the tetrahedron control. Inspect all other rows and preserve adverse
samples; an isolated favorable latency is insufficient. Instrumented timing
is not an unqualified production speed claim. Preserve the geometry-output
and serialization contracts; this changes proof search, not represented sets.

Run full affected native/minimal suites, real WASM/TypeScript consumers,
Clippy, docs and formatting after the final edit. This does not complete the
full properties, historical artifact audit, release, workflow or coverage
requirements. No total-work, heap-byte, expression-growth or cancellation bound
is added; externally contain validation and report resource observations.
