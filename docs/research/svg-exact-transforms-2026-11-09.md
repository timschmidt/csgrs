# Exact SVG transform ingress — 9 November 2026

Hypercurve owns SVG numeric ingress and composition; the existing transform,
incidence and region arrangement kernels own geometry. The affected contracts
are P1, P2, P6, P12, P17–P20, P25, P27 and P39. This is a correctness extension.

Three maintained witnesses fail against production `4de752ea16`: rounded decimal
matrix coefficients, loss of a unit translation between large nested offsets,
and a rounded quarter-turn around a decimal center. The same assertions pass
with exact coefficients and composition under STRICT and APPROXIMATE_512.

The existing svgtypes validator retains authority over the admitted transform
grammar and finite-token range. The existing exact decimal span cursor supplies
authored values, including underflow-small nonzero translations. One native
exact affine carrier replaces the floating intermediate carrier. Rotations and
skews use exact rational-degree multiples of pi; scalar failures propagate.
The no-transform context remains implicit and adds no geometric predicate work.
There is no new arrangement, tolerance, fitting or authoring carrier.

The original decimal region/replay test is preserved and extended with an
inherited translate/scale input. Its independent rational rectangle, complete
XOR, fresh-process native replay, reflection, later Boolean and empty recovery
assertions remain. A root SVG-adapter witness independently checks nested
composition, native replay and further operations. Grammar errors and tangent
poles are typed separately. Four new capability rows retain both policies.

Qualification records 122 successful test reports across all 17 affected
Hypercurve and CSGRS SVG harnesses, with no failures or ignores, plus the
17 capability-runner tests and five selected capability witnesses. Production
files remained byte-identical while the transform test was strengthened with
independent square-root coordinates and fresh native replay for every rotation
and skew case. Final focused, MSRV 1.95.0 and strict Clippy reruns pass. Nine
unchanged maintained acceptance bodies also execute in three fresh pure Rust
WASM instances with no host imports and the original compiled stack. Native
subprocess replay remains guarded by its original target condition.

Three unchanged untransformed controls run under both policies against published
main and the candidate. Complete dispatch vectors, rational counters, certainty
receipts and all eight exported SVG artifacts match exactly. This is finite
behavioral evidence, not a controlled performance or universal closure claim.
Only the five selected catalog rows are newly qualified here; the catalog's
439 entries do not imply that all 439 were rerun in this admission.

Raw red/green logs, original and strengthened fixtures, source hashes, compiled
artifact hashes, capability records and the separate B-text trace baseline are
retained locally in `docs/evidence/2026-11-09-svg-exact-transforms`. The archive
index identifies availability and digest. Complete pre-push verification is
still required for a new publication; the previous `4de752ea16` qualification
does not qualify this implementation.

Primitive shape/style attributes and sampled circle/ellipse conventions remain
open. No total arithmetic-work, heap or cancellation bound is added. Authored
expression size and scalar complexity remain real costs. General SVG source
conformance and arbitrary algebraic operation closure are not established.

The separate B-text diagnostic now records nonempty dispatch and successful
native-region continuation under both policies. Its original offset performs
3,221,399 rational GCDs and reaches 67,179 operand bits in the STRICT recording.
Its roughly 59-second traced evaluations are attribution observations, not a
controlled speedup or downstream CI completion claim. The unresolved B-text
compile deadline remains unchanged; exactness work takes priority.
