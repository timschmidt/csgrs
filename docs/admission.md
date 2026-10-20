# Hyper stack admission protocol

This is the normative acceptance contract for changes to a scalar certificate,
predicate, curve family, topology operation, cache, modeling operation, adapter,
distribution, or machine lowering in the Hyper stack. It consolidates the
requirements in
[`properties.md`](../properties.md), P1–P39. A crate's API documentation supplies
its concrete types and algorithms; it must not weaken these requirements.
This protocol specifies admission criteria, not a claim that all existing
implementations already satisfy them. Outstanding work is tracked in the
[execution ledger](properties-status.json).

> A result is not complete until the next supported operation can consume it
> without losing mathematical meaning, evidence, or control of resources.

The governing order is **exactness > completeness > performance > memory >
binary size > source size**, within a declared input domain and resource
contract. A faster path that silently changes the answer, loses evidence,
rejects previously supported input, or weakens policy fails admission. An
intentional contract change requires an explicit migration and cannot be
presented as a performance-only change.

## 1. State the extension's contract

Before implementation, reproduce review findings on the target revision; an
earlier assessment is not evidence of its current behavior. Identify the
owning layer, relevant property, affected consumers, and a failing or missing
acceptance test. Record:

- The input domain, output carrier, and mathematical operation. For sampled
  geometry, specify the exact set represented by those samples; do not claim
  that a polygonal sphere is the analytic sphere.
- The supported representation tags and compositions, including canonical
  empty results, holes, symbolic coordinates, qualified uncertainty, and
  degenerate, coincident, tangent, reversed, disconnected, and nested inputs.
- The predicate policy and distinct precision, total-work, retained-memory,
  and cancellation contracts, with units and typed exhaustion outcomes.
  A fixed refinement precision or step count alone does not bound expression
  growth, coefficient size, total computation, or elapsed time.
- The witness that permits a definite answer, how it is validated, and what
  source/root/parameter identity is retained for replay.
- The outcome vocabulary: successful empty geometry, invalid input, ordinary
  numeric domain error, unsupported representation, undecided predicate,
  exhausted budget, and unbounded result must remain distinguishable.
- The dependent operations that must accept this output, including repeated
  composition, and the public API, serialization, and release impact.

Use established mathematical terminology: orientation, winding number,
regularized Boolean operation, rational, algebraic root, interval enclosure,
and triangulation. Distinguish structural equality from equality of values,
and state what each construction or conversion establishes:

| Contract | Required claim |
| --- | --- |
| Numerical certainty | Decisions are justified by the evidence consumed under the selected policy. |
| Topological validity | The representation satisfies its declared topology, such as closed PWN input or manifoldness where required. |
| Geometric fidelity | The representation has the stated relationship to its intended source, with any quantization or approximation bound. |

A certified predicate alone does not establish manifoldness. A valid closed
mesh does not prove fidelity to an analytic surface. Certification of a sampled
field does not certify the underlying continuous field. Test these dimensions
separately. New names must agree with the same operation's name elsewhere in
the stack.

## 2. Own the proof at the earliest sufficient layer

Fix the invariant in the lowest layer that owns it, even when the failure first
appears in a consumer. Reuse native representations and shared implementations;
retain construction facts and provenance that downstream operations can use.
Demonstrate that affected public entry points share the enforcement. Do not
substitute duplicated formulas, caller-specific tolerances, a second kernel in
an adapter, or application-side reconstruction of discarded evidence.

| Layer | Required admission evidence | Properties |
| --- | --- | --- |
| Hyperreal | Exact input stays exact; a structural fact has a certificate; symbolic identities avoid needless refinement; undefined numeric operations return `Problem`; float ingress rejects NaN and infinity. Use shared reducers and stable primitives instead of duplicating unstable formulas. | P1–P7 |
| Hyperlimit | An explicit policy selects the decision cascade. Every decided answer has its certainty receipt and witness; unresolved work remains `Unknown`. Structural and rational proofs precede bounded refinement and any policy-authorized terminal. | P8–P11 |
| Hypercurve | `CurveRegion2` remains the filled-region carrier. Winding determines material and holes under the declared fill rule. Output retains source boundary, contact, selected-root, and parameter identity. Unsupported promotion returns a typed blocker; display chords are not exact evidence. | P12–P16 |
| Composition across layers | Verify the intended regularized set and ownership at crossings, tangencies, overlaps, and degeneracies; preserve representations and evidence required by the next operation; measure repeated construction/refinement cost. | P17–P20 |
| Hypermesh | Validate the closed piecewise-constant winding-number (PWN) input contract; reuse one arrangement across expression roots. Invalid topology fails typed. Exterior-inside expressions fail with `UnboundedBooleanOutput`; coincidence uses exact predicates. | P21–P24 |
| CSGRS | Compose native carriers through adapters, including documented bridges for kernel restrictions and empty results. Preserve typed failure, representable interchange values, and deterministic serialization. Format boundaries own quantization. Modeling vocabulary must not introduce a parallel kernel. | P25–P27 |
| ABI and display | Validate input before native construction; preserve certainty, ownership, and typed failure across each boundary. Finite display buffers cannot be consumed as exact source evidence. | P28–P30 |
| Motion and machines | Each admitted demotion has an error budget and a certificate. The real-time core accepts only independently admitted bounded IR; canonical jobs and planning evidence replay. OpenSCAD numeric source enters `Real` without binary64 parsing. | P31–P34 |

The motion/machine rows apply to the downstream integration described in the
plan. Merely adding this specification to CSGRS is not evidence that firmware,
the OpenSCAD differential harness, or that integration has been implemented.

Kernels need not accept every representation. The public modeling layer must
make each result consumable by the next supported operation whenever mathematical
composition is defined, without changing meaning or erasing qualified uncertainty.

## 3. Keep certainty and identity explicit

1. A successful strict decision must have a proof of its stated claim.
   Refinement failure, lack of a recognized form, and unsupported geometry
   cannot be interpreted as false or zero. Local certification does not certify
   the intent of an earlier approximate construction.
2. `APPROXIMATE_512` authorizes the specified terminal interpretation; selecting
   it does not by itself make a result approximate. Consuming the terminal
   does. A terminal answer is not an exact proof of the original expression.
3. Composite claims carry the certainty of the facts and decisions on which
   they depend. Keep operation-local certainty separate from end-to-end
   provenance where necessary. Preserve
   `Provisional`, `Classification`, `GeometryOutcome`, or the corresponding
   kernel result until an explicitly named boundary handles it. Exported APIs
   must make this receipt queryable, including errors and uncertain queries.
   Upgrading a claim requires a new proof of that claim, not a wrapper
   conversion, cache hit alone, serialization step, or discarded outcome object.
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

Exercise the entire dependency chain:

```text
approximate construction
→ transform
→ Boolean operation
→ export
→ import
→ validation
```

No step may silently promote the original claim to certified. Reimport can
establish facts about the quantized mesh; that is a different claim from
preserving the original exact geometry. Lossless interchange preserves
representable values. Lossy interchange must expose its loss model and validate
any promised topology or error bound. Rendering and export approximations must
not replace authoritative exact source geometry.

The primary modeling interface preserves typed failures. An error must never
be replaced by an empty mesh, zero coordinate, false classification, unchanged
input, or incomplete result presented as success. Deliberately lossy convenience
interfaces must name that behavior and require intentional use. Panics do not
replace ordinary domain errors. Exercise each outcome category through the
public API and affected wrappers and bindings, checking that diagnostics survive
and failed operations cannot enter a downstream pipeline as successful models.

## 4. Treat caches as part of the contract

Document each cache's key equivalence, retention budget and unit, eviction
order, lifetime, thread behavior, reset mechanism, and cold/warm semantics.
No pointer key may outlive its owned input identity. Reuse must account for
geometry identity, mutation, policy admissibility, precision, and proof scope;
stale geometry must never appear validated. A strict proof can serve multiple
policies when admissible; an approximate fact cannot become a strict proof.
Precision-dependent entries retain enough information to serve later requests
honestly, including reuse of proofs obtained at higher precision.

Test cold, warm, evicted, disabled (where configurable), mutated, nested, and
concurrent states, where applicable. Exercise strict→approximate→strict and
low→high→low precision sequences. Cache hits retain the witness and certainty;
mutation of a returned value must not modify a retained result. An exhausted
speculative budget is not a permanent proof that a value is undecidable.

Warming may turn `Unknown` into `Certified` through available evidence. Compare
mathematical results and definite facts, not identical execution paths or
failure histories. Retained state may reduce work or reuse a stronger proof;
it must never introduce contradictions or weaken an available certificate.

For CSGRS modeling caches, [`solid::cache`](../src/solid/cache.rs) defines
entry limits and synchronous thread-local scopes. Those limits are not a
heap-byte bound and do not configure kernel/scalar caches.

## 5. Validate exact behavior before measuring cost

Each change must supply the applicable evidence below, with commands,
feature sets, fixtures, and output locations recorded in its ledger entry.

### Composition and semantic equality

For valid represented sets `A`, test these regularized-set identities, then
feed their outputs into further supported operations:

```text
union(A, empty)        ≡ A
intersection(A, empty) ≡ empty
difference(A, empty)   ≡ A
difference(empty, A)   ≡ empty
difference(A, A)       ≡ empty
```

Here `≡` means equality of represented sets, not identical triangulations or
vertex ordering. Include union accumulation from empty and subtraction chains
that remove everything. Distinguish canonical empty geometry from malformed
geometry, including malformed operands paired with empty; identity shortcuts
must not conceal invalid inputs. Include disconnected components, holes,
retained symbolic coordinates, and qualified outcomes in composition coverage.

Correctness evidence must detect a shared implementation mistake. Agreement
between wrappers of one kernel is not an independent oracle; triangle counts
and checksums alone do not establish the intended set. Use independently
derived exact results, representation-aware generators, topology checks, and
mathematical relationships. For valid inputs and supported invertible affine
transforms `T`, applicable properties include:

```text
union(A, B)                 ≡ union(B, A)
intersection(A, B)          ≡ intersection(B, A)
union(A, A)                 ≡ A
T(boolean(A, B))            ≡ boolean(T(A), T(B))
union(difference(A, B),
      intersection(A, B))   ≡ A
```

State each property's input, representation, policy, and transform preconditions
and its equality criterion. Test semantic equality separately from deterministic
serialization; neither substitutes for the other contract.

### Certified completion of useful workloads

Maintain a named corpus of ordinary workflows: drilled bodies, transformed
primitives, profile extrusion, holes, repeated cuts, touching and coplanar
surfaces, and supported symbolic representations. The
[named modeling corpus](workflow-corpus.md) records fixtures, input domains
and semantic equality criteria; admission records identify the revisions and
configurations actually executed. Record separately:

```text
certified completion
qualified approximate completion
undecided
unsupported
invalid input
resource exhaustion
incorrect result
```

Capability is completed, validated work, not the count of constructors, tests,
or API entries. Strict mode need not decide every possible expression, but its
supported domain must be explicit. Preserve the certified-success corpus and
add regressions for expanded capability. Do not make a failure pass by changing
policies, accepting a new `Unknown`, removing difficult geometry, or reducing
fixture resolution. A deliberate contract change requires a recorded migration
and must not be credited as a fix under the original contract. Keep minimized
regressions alongside the original full-scale hard cases.

### Resource behavior

Input-controlled work must have an explicit resource contract. Distinguish
precision limits, total work, retained-memory limits, and cancellation; an
entry-count cache limit is not a heap-byte limit. Exhaustion produces a typed
outcome before publishing a result as complete. Useful partial results require
a separately identified provisional contract.

Run repeated compositions and adversarial growth cases under deliberately
small budgets. Check exhaustion, cancellation, retained-state cleanup, and
subsequent recovery. Measure expression depth, coefficient growth, allocations,
and peak memory where affected. Where internal bounds are absent, record that
limitation and use external containment for dangerous tests rather than
claiming bounded execution.

### Evidence matrix

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
dependency locks, hardware, input construction, seeds, iteration counts,
warmup, timing boundaries, policies, cache conditions, and raw samples.
State whether a run is a focused probe, a matched benchmark, or a campaign.

Compare the same completed contract. An operation returning an error sooner
is not a faster successful operation. Lower resolution, missing provenance,
weaker certainty, skipped validation, or excluded output materialization changes
the workload. Report completion rates and certainty beside latency and resource
use, including failures and unfavorable results on the supported corpus.

For each retained optimization, compare exact output rows, topology, provenance,
certainty, serialized output size, and deterministic checksums. Checksum
agreement supports the comparison but does not replace exact equality checks.
Preserve the declared exact-output and deterministic-artifact contracts;
intentional representation or serialization changes must be identified
separately from performance-only work and have an explicit migration.
Measure cold and warm behavior separately, and report allocation and binary
size evidence proportionate to the changed resource costs. Parallel or SIMD
execution must preserve the same policy, reduction semantics, and output order.

Maintain an idea ledger with `kept`, `rejected`, `already-satisfied`, or
`architecture-inapplicable`, decisive measurements, and interactions with
previously rejected ideas. Do not keep a lower-rung improvement with a
significant higher-rung regression. A timer-scale result is not evidence of
a speedup; a short probe is not a cross-library performance claim. An isolated
favorable measurement cannot justify retaining an optimization while concealing
supported-corpus regressions.

## 7. Complete the admission record

An admission record links the contract, implementation, tests, raw evidence,
public documentation, known blockers, and release effects. Mark incomplete
items explicitly. After the final edit, record the local and downstream checks
executed, what passed, what remains blocked, and the evidence supporting each
correctness or performance claim. A paper must distinguish proved statements,
measured results, and open questions and link each claim to reproducible data.

### Retention budget

Evidence must stay proportionate to the claim it supports. In this repository,
an admission record is a short Markdown summary. It gives the contract, the
commands run, their results and blockers, and the digests of the evidence it
relies on. Raw material goes to the evidence repository described in
[`evidence-archive.md`](evidence-archive.md). That covers command logs,
samples, traces, replay bundles, audit corpora and checkpoint archives. Add
each new record's digest to [`evidence-archive.tsv`](evidence-archive.tsv)
with `scripts/evidence_archive.py`. Cite a source revision by commit instead
of copying its tree. Record third-party package archives by name, version and
digest instead of redistributing them.

`scripts/check-repository-budget.sh` enforces the in-repository side of this
budget in CI. Archived evidence roots stay untracked, the tracked tree stays
within 256 MiB, and no tracked file exceeds 4 MiB except the allowances it
lists.

### The shipped user path

A capability is delivered only when an external consumer can use it through the
documented distribution. The release, dependency graph, examples, API
documentation, feature configurations, minimum supported compiler, and bindings
must describe a coherent system. A workspace build alone is insufficient.

In a clean external project, exercise:

```text
install documented package
→ construct
→ transform
→ compose
→ inspect certainty
→ validate required topology
→ export
→ reimport
→ check the format-specific contract
```

Run real binding consumers where affected. Verify that workspace patches, local
paths, undeclared assets, and development-only configuration are not making the
example work. Record package feature compatibility, dependency versions,
minimum supported compiler validation, license/provenance, examples, binding
artifacts, and packaging checks before calling a crate release-ready. An
unpublished or unavailable distribution leaves this boundary open; local source
validation does not establish that it shipped.

### Final checks and implementation order

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

For the review gaps, implement **empty-result composition and visible failures
first; ordinary strict-mode completion next; complete user-path and release
validation next**. Performance work proceeds against those preserved contracts,
not in place of them. Make the smallest coherent change that restores or
extends the contract, without weakening tests, policies, fixtures, or validation
to obtain a passing result.

For every patch: **does it let users complete more trustworthy work, or complete
the same trustworthy work at lower cost?**
