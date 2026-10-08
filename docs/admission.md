# Hyper stack admission protocol

This is the normative protocol for extending a scalar certificate, predicate,
curve family, topology operation, cache, adapter, or machine lowering in the
Hyper stack. It consolidates the requirements in
[`properties.md`](../properties.md), P1–P39. A crate's API documentation supplies
its concrete types and algorithms; it must not weaken these requirements.
This protocol specifies admission criteria, not a claim that all existing
implementations already satisfy them. Outstanding work is tracked in the
[execution ledger](properties-status.json).

The governing order is **exactness > completeness > performance > memory >
binary size > source size**. A faster path that changes the represented set,
loses a reusable certificate, or rejects previously supported input fails
admission. A mathematically exact result that cannot be used in the next
supported operation is also incomplete.

## 1. State the extension's contract

Before implementation, identify the owning layer and record:

- The input domain, output carrier, and mathematical operation. For sampled
  geometry, specify the exact set represented by those samples; do not claim
  that a polygonal sphere is the analytic sphere.
- The supported representation tags and compositions, including degenerate,
  coincident, tangent, reversed, disconnected, and nested inputs.
- The predicate policy, work budget and unit, and typed exhaustion result.
  Distinguish a precision limit from a bound on total work. A fixed number of
  refinement steps alone does not bound coefficient growth or elapsed time.
- The witness that permits a definite answer, how it is validated, and what
  source/root/parameter identity is retained for replay.
- The failure vocabulary: invalid input, ordinary numeric domain error,
  unsupported representation, exhausted budget, unbounded result, or
  inconclusive predicate. None of these is an empty successful result.
- The dependent operations that must accept this output, including repeated
  composition, and the public API, serialization, and release impact.

Use established mathematical terminology: orientation, winding number,
regularized Boolean operation, rational, algebraic root, interval enclosure,
and triangulation. Distinguish structural equality from equality of values,
and certainty of a predicate from fidelity of a display projection. New names
must agree with the same operation's name elsewhere in the stack.

## 2. Own the proof at the earliest sufficient layer

| Layer | Required admission evidence | Properties |
| --- | --- | --- |
| Hyperreal | Exact input stays exact; a structural fact has a certificate; symbolic identities avoid needless refinement; undefined numeric operations return `Problem`; float ingress rejects NaN and infinity. Use shared reducers and stable primitives instead of duplicating unstable formulas. | P1–P7 |
| Hyperlimit | An explicit policy selects the decision cascade. Every decided answer has its certainty receipt and witness; unresolved work remains `Unknown`. Structural and rational proofs precede bounded refinement and any policy-authorized terminal. | P8–P11 |
| Hypercurve | `CurveRegion2` remains the filled-region carrier. Winding determines material and holes under the declared fill rule. Output retains source boundary, contact, selected-root, and parameter identity. Unsupported promotion returns a typed blocker; display chords are not exact evidence. | P12–P16 |
| Curve composition | Verify the intended regularized set and ownership at crossings, tangencies, overlaps, and degeneracies; preserve representations and evidence required by the next operation; measure repeated construction/refinement cost. | P17–P20 |
| Hypermesh | Validate the closed piecewise-constant winding-number (PWN) input contract; reuse one arrangement across expression roots. Invalid topology fails typed. Exterior-inside expressions fail with `UnboundedBooleanOutput`; coincidence uses exact predicates. | P21–P24 |
| CSGRS | Compose native carriers through adapters, preserve representable interchange values, and keep deterministic serialization. Modeling vocabulary must not introduce a parallel kernel. | P25–P27 |
| ABI and display | Validate input before native construction; preserve certainty, ownership, and typed failure across each boundary. Finite display buffers cannot be consumed as exact source evidence. | P28–P30 |
| Motion and machines | Each admitted demotion has an error budget and a certificate. The real-time core accepts only independently admitted bounded IR; canonical jobs and planning evidence replay. OpenSCAD numeric source enters `Real` without binary64 parsing. | P31–P34 |

The motion/machine rows apply to the downstream integration described in the
plan. Merely adding this specification to CSGRS is not evidence that firmware,
the OpenSCAD differential harness, or that integration has been implemented.

## 3. Keep certainty and identity explicit

1. A strict result must be certified. Refinement failure, lack of a recognized
   form, and unsupported geometry cannot be interpreted as false or zero.
2. `APPROXIMATE_512` authorizes the specified terminal interpretation; selecting
   it does not by itself make a result approximate. Consuming the terminal
   does. A terminal answer is not an exact proof of the original expression.
3. Composite operations propagate the weakest consumed certainty. Preserve
   `Provisional`, `Classification`, `GeometryOutcome`, or the corresponding
   kernel result until an explicitly named boundary handles it. Exported APIs
   must make this receipt queryable, including errors and uncertain queries.
4. A selected root needs its defining relation and isolating/selection
   evidence. A selected fiber also needs the identity of its base root. Two
   independently reconstructed coordinates are not a replacement for those
   identities.
5. Every output boundary element must identify its source elements and the
   decisions that retained, split, reversed, or removed them. Record contour
   roles and fill rule, not an authoring-orientation convention masquerading
   as a proof of interior.
6. An extension that resolves a former `Unknown` must preserve all previously
   certified results and their meaning. Representation changes may require
   an explicit format/API migration; they cannot silently invalidate replay.

## 4. Treat caches as part of the contract

Document each cache's key equivalence, retention budget and unit, eviction
order, lifetime, thread behavior, reset mechanism, and cold/warm semantics.
No pointer key may outlive its owned input identity. Do not reuse an
approximate fact as a strict proof. Cache identity includes policy and any
other inputs that affect semantics; precision-dependent entries retain enough
information to serve later higher-precision requests honestly.

Test empty, warm, evicted, disabled (where configurable), nested, and concurrent
states. Exercise both directions of policy changes and low/high/low precision
queries. Cache hits must retain the witness and certainty, and mutation of a
returned value must not modify a retained result. An exhausted speculative
budget is not a permanent proof that a value is undecidable.

For CSGRS modeling caches, [`solid::cache`](../src/solid/cache.rs) defines
entry limits and synchronous thread-local scopes. Those limits are not a
heap-byte bound and do not configure kernel/scalar caches.

## 5. Validate exact behavior before measuring cost

Each extension must supply the applicable evidence below, with commands,
feature sets, fixtures, and output locations recorded in its ledger entry.

| Evidence | Acceptance criterion |
| --- | --- |
| Unit and representation matrix | Every added finite representation tag and dispatch pair has a construction recipe and assertion that it reached the intended representation. Domain errors and boundary values are included. |
| Independent correctness oracle | Certified exact rows or independently derived identities agree. Comparing two wrappers around the same implementation alone is insufficient. |
| Topology and composition | Reversal, winding multiplicity, tangency, coincidence, slivers, disconnected components, holes, invalid indices, unbounded complements, and repeated outputs-to-inputs decide or diagnose with the specified semantics. |
| Near-coincidence | Exact offsets such as `1/10^12` remain distinct in classification and bounds; no tolerance merges them. |
| Fuzz regression | Each minimized, license-clean failure is retained as a deterministic fixture. Full-resolution hard cases remain full resolution; any additional reduced case supplements them. |
| Directed campaigns | Representation families have independent lexicase selection buckets. Logs include seeds, budget, selection scores, crashes/timeouts, minimized cases, ASan configuration, and dispatch paths. |
| Formal verification | Proof coverage names executable entry points, invariants, trusted assumptions, tool version, and proof results. Verification of a model is not credited as verification of unconnected production code. |
| Boundary exercise | Run actual C/C++, Python, and JavaScript/WASM consumers where affected. Check ownership, family mismatches, error lifetime, malformed lengths/indices, non-finite numbers, and certainty access. |
| Coverage | Publish the instrumented denominator, features, exclusions, and line/region/branch metrics actually supported by the tool. The 100% target remains open until measured; infinite input domains are addressed by proofs, generators, and adversarial matrices. |

## 6. Gate optimization on reproducible evidence

Establish the baseline before changing the measured implementation. Retain
source revisions or patches, compiler and target, build flags, feature sets,
dependency locks, hardware, seeds, iteration counts, warmup, and raw samples.
State whether a run is a focused probe, a matched benchmark, or a campaign.

For each retained optimization, compare exact output rows, topology, provenance,
certainty, serialized output size, and deterministic checksums. Checksum
agreement supports the comparison but does not replace exact equality checks.
Measure cold and warm behavior separately, and report allocation and binary
size evidence proportionate to the changed resource costs. Parallel or SIMD
execution must preserve the same policy, reduction semantics, and output order.

Maintain an idea ledger with `kept`, `rejected`, `already-satisfied`, or
`architecture-inapplicable`, decisive measurements, and interactions with
previously rejected ideas. Do not keep a lower-rung improvement with a
significant higher-rung regression. A timer-scale result is not evidence of
a speedup; a short probe is not a cross-library performance claim.

## 7. Complete the admission record

An admission record links the contract, implementation, tests, raw evidence,
public documentation, known blockers, and release effects. Mark incomplete
items explicitly. Document package feature compatibility, dependency versions,
license/provenance, examples, binding artifacts, and packaging checks before
calling a crate release-ready. A paper must distinguish proved statements,
measured results, and open questions and link each claim to reproducible data.

Run the applicable correctness, documentation, lint, benchmark, and boundary
checks after the final code change. Before any push, follow the complete
[repository pre-push verification rules](../AGENTS.md), including every locally
executable workflow command and all required Clippy combinations. CI changes
also require `zizmor --pedantic --strict-collection .`. Hosted-only or unavailable
platform steps require an explicit limitation record.

P35–P39 govern every stage: promote failures, gate retention on evidence,
state coverage precisely, direct adversarial work toward weak cases, and keep
the typed frontier and its open problems published. A ledger entry, green
narrow test, or this document itself does not prove a broader requirement.
