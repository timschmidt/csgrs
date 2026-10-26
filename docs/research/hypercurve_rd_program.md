# Hypercurve: research and development program for exact CAD/CAM geometry

**Implementation handoff — 25 October 2026**

**Repository:** https://github.com/timschmidt/csgrs\
**Inspected revision:** `3cca6a894858c8b98497a3cfe81cff5e17e1a7f2`, committed 24 October 2026\
**Governing priority:** **exactness > completeness > performance > binary size > code size**

## Read this first

Implement an evidence-driven program of small, independently reviewable changes. Investigate every work package below; retain only candidates that demonstrate an improvement under the stated priority order and preserve all higher priorities. A package is a hypothesis to evaluate, not a direction to force its proposed architecture into production.

The central architectural hypothesis is that authored and generated curves should share a semantic contract: **support, parameter chart, exact domain, traversal, and source-bound evidence**, with optional native images and specialized arithmetic backends. The hypothesis is not that every curve should have one numerical representation. Begin with an adapter over the existing span machinery, migrate a narrow consumer, measure it, and remove the superseded path only after admission.

Complete useful CAD/CAM operation domains and output-to-input workflows in parallel with consolidation. Do not defer concrete correctness and capability repairs until a large representation rewrite finishes. Do not declare the library complete because its constructors, individual operations, or finite tests all exist.

This document is a research and implementation specification, not a report of completed experiments. The repository was inspected statically. Rust build/test tools were unavailable in the review environment; no current suite or benchmark campaign was independently executed. Historical timings and unresolved cases below are leads that the implementing agent must reproduce. Rebase this program onto the actual execution revision before editing.

### Navigation

1. Mission, priorities, and admission rules
2. What “complete” means for this program
3. Current architecture and baseline corrections
4. Target contracts and ownership
5. Independent validation and workload corpus
6. Measurement and acceptance protocol
7. Research work packages P00–P29
8. Execution phases and dependency gates
9. Required experiment records and command anchors
10. Completion, release, and stopping criteria
11. Source map and primary research

---

## 1. Mission, priorities, and admission rules

### 1.1 Mission

Make Hypercurve easier to understand, maintain, extend, and use as an exact planar geometry kernel for CAD/CAM. Preserve the mathematical and operational guarantees that justify its complexity. Reduce duplicated authorities, conversions, predicates, topology rules, and generated-expression growth where measurements support doing so.

The desired result is a smaller number of well-defined semantic mechanisms supporting a larger, explicitly documented set of completed workflows. No target line count, number of types, or speedup is assumed in advance.

### 1.2 The priority order is lexicographic

Use the user's order exactly:

| Highest improved priority | Required protections | Lower-priority costs that may be accepted and must be disclosed |
|---|---|---|
| Exactness | No new false mathematical, topological, provenance, or certification claims; demonstrate the repaired defect or strengthened guarantee | Some requests may correctly become undecided; runtime and size may increase |
| Completeness | Exactness remains intact; preserve the already admitted completion domain while adding the new capability | Runtime and sizes may increase |
| Performance | Exactness and the admitted completion domain remain intact | Binary and source size may increase |
| Binary size | Exactness, completeness, and protected performance remain intact | Source size may increase |
| Code size | Exactness, completeness, performance, and binary size remain intact | No lower ranked production metric remains |

This is not a weighted score. A 30% line reduction cannot offset a lost exact case. A performance improvement cannot excuse a dropped contact, weakened closure test, approximate fallback, missing component, or new unsupported output.

Within a priority, do not trade away one protected capability or workload merely because an aggregate improves. A completion gain should extend the admitted success set, not exchange one success for another. A performance claim must state its protected workload set and cannot hide a known slowdown behind a geometric mean. If two candidates offer incomparable gains within one priority, keep an explicit comparison frontier until the evidence or a declared workload contract resolves the choice.

The repository currently includes memory in its own priority ladder. For this program, the user's five-part order controls. Preserve existing resource, allocation, cancellation, and failure contracts; measure memory and retention because they affect completion and runtime. Do not silently insert memory as an additional ranked priority, or reject a size improvement solely for a memory increase that breaches no contract and causes no higher-priority regression.

### 1.3 Exactness includes the claims made about a result

For each result, distinguish:

- The mathematical value or represented set.
- Numerical certainty of the predicates used.
- Topological validity required by the operation.
- Fidelity to the intended input/source geometry.
- Selected-root, parameter, chart, and source identity.
- Honest failure and resource status.
- Evidence needed for a supported downstream operation.

An exactly represented polygon may only approximate its intended smooth source. A certified projection of a provisionally closed path does not certify the earlier closure. A coordinate satisfying an eliminant may select the wrong root or an extraneous solution. None of these distinctions may disappear during simplification.

### 1.4 Admission rules

For every candidate:

1. State the precise domain and highest priority it improves.
2. Reproduce the baseline and write an independent expected-result specification.
3. Freeze the protected success set, policies, resources, features, targets, and measurements before observing the candidate's results.
4. Implement the smallest coherent experiment in isolation.
5. Verify semantics and higher-priority nonregression, including downstream reuse.
6. Measure the intended benefit and all material lower-priority costs.
7. Admit, reject, or hold it with evidence. Delete temporary competing production paths after admission unless an explicit fallback remains necessary.

A formerly incorrect baseline answer is not a valid success to preserve. Fix that defect, explain any resulting loss of apparent completion, and seek the narrowest sound repair. Do not manufacture an “exactness improvement” by broadly refusing valid work without a reproduced correctness problem.

Testing and research infrastructure may be necessary to establish these decisions. Keep it outside production where appropriate, account for its maintenance cost, and call it verification infrastructure. More tests alone are not proof that an algorithm became more exact or more complete.

Unverified, blocked, and statistically inconclusive candidates remain experimental. Neither a deadline nor an attractive design diagram is admission evidence.

Freezing protected cases prevents cherry-picking; it does not excuse a subsequently discovered, reproducible higher-priority regression within the declared supported domain. Investigate that concrete risk, add it to the corpus, and reject or repair the lower-priority candidate.

---

## 2. What “complete” means for this program

### 2.1 Define completion by domain, operation, and workflow

Maintain separate declarations for:

1. **Mathematical domain:** coefficient field, supported curve/support families, parameter domains, regularity, degeneracies, and permitted topology.
2. **Operation contract:** result semantics, required evidence, bounded/unbounded behavior, and exact versus explicitly approximate output.
3. **Practical execution envelope:** input/output complexity, resource policy, target platform, and measured completion limits.
4. **Composition contract:** which outputs can be passed to which subsequent operations without conversion that weakens their claims.

A bounded corpus can establish regression protection and practical coverage. It cannot prove termination or closure for every finite input of unbounded degree, coefficient size, or expression depth.

For finitely described rational/algebraic inputs within a declared class, exact decision procedures are a meaningful research goal, subject to mathematical assumptions and resources. For arbitrary constructive or transcendental `Real` values, total terminating equality/sign decisions are not a valid general promise. Preserve typed uncertainty and expand recognized exact identities carefully. See R4 in the bibliography.

### 2.2 Completion tiers

| Tier | Intended domain | Required deliverable |
|---|---|---|
| A: ordinary exact CAD profiles | Finite line and circular-arc boundaries, rational/algebraic coordinates admitted by each operation, holes/islands and open paths | Reliable Boolean, trim, offset/stroke, corner editing, containment, and reuse workflows, including documented degeneracies |
| B: rational curves and their generated results | Polynomial/rational Béziers, supported spline/NURBS spans, exact selected trims, analytic parallels, generated chords/circles and selected fibers | The same declared operations on equivalent authored and generated representations, with source/chart identity and output-to-input closure |
| C: algebraic branch expansion | Explicitly admitted implicit polynomial branches, maps, singular/vertical/repeated components and branch pairs | Staged certified relations, local sectors, point location, then filled arrangements; each stage has its own domain |
| Q: certified finite output and measurement | Chords/arcs/finite coordinates, length/distance/curvature enclosures, documented approximation policies | Proven bounds and qualification tied to source spans; downstream consumers can identify the exact guarantee |

These are milestones, not claims that the inspected tree already satisfies every cell. Preserve existing capabilities beyond these tiers as well; tiers prioritize additions rather than licensing deletions.

### 2.3 Operation matrix to instantiate

For every admitted representation and relevant ordered representation pair, inventory:

| Area | Operations and distinctions to record |
|---|---|
| Curves and charts | Evaluate, derivative/jet, split, trim, reverse, transform, native image, injectivity/regularity evidence, seams and one-sided limits |
| Relations | Intersections, tangency/multiplicity, endpoint contacts, overlaps, residual contacts, source-parameter correspondence, self-intersection |
| Filled geometry | Validation, fill rule, regularization, union/intersection/difference/xor, component/hole ownership, boundary extraction and reconstruction |
| Offsets and strokes | Signed normal parallels, regularized region offset, joins/caps, cusps, collapse/split/merge, open versus closed semantics |
| Corner editing | Fillet/chamfer constraints, all finite solutions, impossible/continuous families, trimming, re-editing generated corners |
| Queries and measures | Membership, conservative bounds, exact containment, extrema, nearest point/distance, arc length and inverse length, clearance and curvature |
| Specialized geometry | Current straight-skeleton domains, translation obstacles/no-fit regions, optional Minkowski extensions |
| Interoperability | Exact source replay, finite export/import, C/WASM/bindings, error and qualification propagation |
| Resources | Existing budgets, truthful exhaustion, cancellation, cache behavior, cleanup and recovery |

For each cell record supported coefficient domain, result family, transformation capability, policy, resource contract, proof obligations, status, fixture, oracle, and downstream consumers. A representable type is not evidence that every operation supports it.

### 2.4 Keep the kernel boundary clear

Hypercurve should provide geometric regions, curves, constraints, certified queries, and source/fidelity records useful to CAD/CAM. Toolpath sequencing, pocket strategy, rest-machining strategy, feed/speed scheduling, controller behavior, machine quantization, and G-code policy belong to downstream applications or existing Hyperpath layers.

Exercise those consumers in integration tests where relevant. Do not build a second CAM planner inside Hypercurve or describe a geometric offset candidate as a proven complete machining process.

---

## 3. Current architecture and baseline corrections

The following observations refer to the inspected revision. Recheck them before implementation. Repository source references S1–S12 are listed at the end.

### 3.1 The size problem must use consistent denominators

The static audit counted approximately:

| Scope | Raw physical Rust lines |
|---|---:|
| All Hypercurve Rust files | 479,887 |
| Hypercurve `src`, including embedded tests | 411,679 |
| Embedded test code identified within `src` | 135,896 |
| Non-test `src` remainder | 275,783 |
| `bezier_offset.rs` and its directory, including tests | 191,579 |
| That offset subsystem's non-test remainder | 127,491 |

These counts explain where to investigate; they are not a claim that those lines are unnecessary. The approximate embedded-test classifier and counting script must be pinned and rechecked for admission measurements. Count comments, tests, fixtures, generated code, examples, and production code separately.

Hyperreal, Hypersolve, and Hyperlimit are additional dependency layers. Moving a solver into one of them is not a total source reduction. Comparing all Hypercurve tests and specialized operations against only another library's production polygon code is not a useful simplification target.

### 3.2 Preserve and extend what already exists

- `Curve2` already wraps native, restricted, source-range, and polynomial carriers. Their difference is not simply “user authored” versus “generated”: authored algebraic lines may be retained chords, and materialized generated fragments may become native.
- `CurveSupport2`, `CurveSourceSpan2`, `CurveParameterRange2`, and `SelectedScalar2` already provide much of the proposed common model.
- `CurveRegion2` already is a common filled-region authority. The four Boolean operations already share an arrangement/split-topology implementation.
- `Curve2::source_restriction()` is already public, although some future-work prose still describes this as missing. Its coverage remains partial.
- Native images and native segments are optional exact views with their own parameter and traversal conditions. They are not universal replacements for source authority.
- Exact offsets/strokes, corner operations, box containment, selected measures, native skeleton machinery, and convex line translation obstacles already exist in varying domains.
- The numerical comparison and enclosure implementation has evolved beyond several older ledger descriptions. Reproducing a named failure is mandatory before accepting its documented explanation.

### 3.3 Reconcile these specific historical leads

| Lead | Required interpretation |
|---|---|
| “10 of 24” long compositions within 300 seconds | Repository-reported historical outcome; only some rows were recently refreshed. Reproduce the execution revision; preserve censored timeouts and raw samples. |
| Broad restricted-support relation migration | Previous claimed completion defects did not reproduce; some migration proposals were deferred. Treat a new pilot as a measured consolidation or an independently demonstrated capability fix. |
| H13 refilleting sign leaves | Recorded leaves were univariate and already had modular nonzero evidence; common-denominator enclosure work addressed their bottleneck. New lazy/modular work needs a genuinely different captured multi-source case. |
| H14 degree-192 equality | An identically-zero remainder is not a GCD target. Do not apply the common-fiber GCD proposal to this case. |
| Degree-214 NURBS fillet center | The selected factor reportedly has intrinsic degree 164. Substituting that factor downstream was measured worse and did not finish the stroke. Investigate construction-side selected fibers instead. |
| Two-sided field joins | A missing-source case was seen in an isolated internal test, not a traced production workflow. Require an explicit supported-operation need or declare a new capability domain. |
| Fixed 16-refinement represented-root explanation | Current source checks enclosures and delegates to full algebraic comparison. Reproduce the Pell-denominator case and identify its current cause. |
| Normal-form sharing/workspace proposals | Several variants were rejected or deferred after other improvements. Reopen only with a named changed premise and residual cost. |
| Shared all-four-Boolean or region span-product optimization | Sharing already exists and region carriers are already single spans. Do not optimize a nonexistent duplication. |

Reclassification to “already satisfied” or “architecture inapplicable” is a valid research result. It is preferable to adding speculative infrastructure.

---

## 4. Target contracts and ownership

### 4.1 Common curve semantics

Use this conceptual contract to evaluate designs; do not implement it verbatim as one oversized object:

```text
Curve/span semantics
    support authority
    parameter chart or finite atlas, period and seam ownership
    parameter transport maps
    spatial image map, distinct from parameter transport
    exact active domain with endpoint ownership
    traversal/orientation and occurrence identity
    source-bound endpoint/contact/regularity evidence
    optional exact native views and acceleration caches
```

The support describes the curve equation or exact construction. The chart identifies what a parameter means; a finite atlas is available when one chart is insufficient. Parameter transports act on charts, while spatial image maps act on the plane. The active domain limits the support. Traversal distinguishes orientation, repeated passes, and periodic seams. Evidence connects selected roots and endpoints to those exact objects.

Keep numerical implementations specialized: rational control nets, algebraic/selected parameters, analytic parallels with radical sheets, recursive local fields, and implicit branch selectors can coexist under the contract.

Inventory exact affine-image capability per family: preservation of the family, exact promotion, or a retained spatial image map. A nonuniformly scaled circle is a conic; an affine image of a Euclidean parallel is not generally the Euclidean parallel of the transformed source.

### 4.2 Non-negotiable representation invariants

1. Repeated trimming composes/normalizes chart maps and does not create an unbounded wrapper chain.
2. Reversal preserves the existing public parameter convention, or an explicit compatibility migration supplies the transport. Native and retained reversal currently have different internal conventions.
3. The same geometric point at several parameters remains several occurrences when the operation needs them.
4. Equal endpoints and matching support do not prove equal traced segments. Injectivity, interval, multiplicity, and traversal matter.
5. Sharing a parameter source does not establish geometric coincidence after different image transforms.
6. A source pole outside the active finite domain does not automatically invalidate the trim; a pole inside it remains a domain obligation.
7. A selected root retains its defining relation, base field, root selection, and certified isolator. Refinement narrows the same value.
8. A point's coordinates may share essential algebraic relations. Do not flatten them into unrelated scalar expressions merely to fit an interface.
9. Authority and dependencies needed to justify a published result remain owned by that result. Recomputable query proofs and acceleration may be evicted when doing so does not invalidate published evidence.
10. Approximation qualification cannot be upgraded by copying, restriction, rebuilding, serializing, or downstream local certification.

### 4.3 Ownership boundaries

| Owner | Responsibility |
|---|---|
| Hyperreal / numerical foundation | Exact scalar expression semantics, certified enclosures/sign logic, recognized symbolic identities, lifecycle and scalar caches |
| Hyperlimit | Reusable geometric predicate semantics, supporting exact constructions, policy/escalation, predicate outcomes, certainty/provenance, and explicit Unknown |
| Hypersolve | Polynomial/root selection, local fields/embeddings, exact system solving, root counts, multiplicity and algebraic topology witnesses |
| Hypercurve supports/charts | Curve construction, active domains, source correspondence, exact geometric relation interfaces |
| Hypercurve topology | Arrangement events, sectors, winding, regularization, region ownership, publication of valid outputs |
| Hypercurve adapters | Qualified finite projection, exact replay formats, API/binding translation, source-fidelity propagation |
| Downstream CAD/CAM | Application policy, machining strategy, finite machine/controller decisions |

Place shared arithmetic in its existing owner where it reduces duplicate authorities. Do not make a universal geometry enum the owner of polynomial algebra, or add a second scalar/region hierarchy.

Resource-budget infrastructure remains with its actual implementing owner; inventory that ownership rather than inferring it from a crate's name.

---

## 5. Independent validation and workload corpus

### 5.1 Four equality contracts

Choose the relevant contract for each test:

| Contract | What must be established |
|---|---|
| Scalar value equality | Same mathematical value, regardless of expression/field representation |
| Region/set equality | Same regularized material set under the declared fill rule |
| Parameterized-curve equality | `C(t) = D(phi(t))` with full claimed domain coverage, proved chart transport, endpoints/seams, direction, and occurrence multiplicity |
| Representation/replay compatibility | Promised source/root identities, receipts, serialization, ABI, and deterministic outputs remain valid |

Rust structural equality is not a general mathematical scalar oracle. Same area, bounds, component count, sampled points, renderings, or checksums cannot establish arbitrary set or parameterized-curve equality.

For ordinary span equivalence, the parameter map must be a certified monotone bijection between the claimed active domains, with direction stated. If a consumer accepts a covering map instead, specify its multiplicity and endpoint/seam ownership. Equality on only a subdomain cannot certify equality of the full curves.

Use exact independent arrangements/cell checks for the families they support, or independently derived family-specific boundary and local-topology certificates. A symmetric difference computed by Hypercurve itself is a useful metamorphic check, not an independent oracle.

Every selected solution needs its original equations, denominator/radical-sheet conditions, parameter restrictions, root isolator, and selection checked. Resultant membership alone is insufficient.

Verification must also establish exhaustiveness over the claimed domain: account for all relevant roots, events, active-domain pieces, and material cells. Checking only the returned witnesses can accept an incomplete result.

### 5.2 Oracle tiers

| Domain | Independent route | Restriction |
|---|---|---|
| Rational linear geometry | Small independent rational formulas; a qualified exact-construction arrangement implementation | Parse identical rationals; validate fill and topology, not just boundary coordinates |
| Polynomial Béziers with rational control coordinates | CGAL's documented Bézier traits and independent algebraic checks | These traits do not automatically cover weighted rational Béziers, arbitrary algebraic controls, or constructive coefficients |
| Weighted rational/implicit algebraic cases | Qualified algebraic-curve traits or independent elimination, substitution, isolation, and topology checks | Prove applicability to coefficient fields, branch selectors, poles, and multiplicity |
| Algebraic scalars and polynomial systems | Independent exact polynomial/number-field implementation, Sturm counts, exact residuals, verified GCDs | Preserve the selected conjugate and distinguish divisibility from maximality |
| Constructive/transcendental values | Independently justified identities and certified interval separation | Interval overlap and high-precision agreement do not prove equality |
| Finite approximation workloads | Clipper2, Cavalier Contours, Kurbo, or comparable qualified tools | Throughput/approximation comparators unless an exact common subdomain is separately established |
| Whole CAD/CAM chains | Analytic fixtures plus independent local certificates, topology, identity, and fidelity checks | Disclose shared numerical dependencies; another wrapper around Hypercurve is not independent |

Integer input endpoints do not imply integer intersections. Do not promote an integer clipping comparator into an exact construction oracle on that basis.

Reuse the repository comparison harness where appropriate, pin its native dependencies, and separate retained-native operation timing from parsing/FFI/serialization timing. The existing exactCore comparison notes report native concurrency problems; isolate that oracle in a worker and serialize calls until independently qualified.

Validate the checkers with deliberately wrong answers: wrong isolated root, deleted tiny hole, flipped branch, rounded rational, incorrect winding, dropped endpoint occurrence, misbound source witness, and a same-area wrong region. A checker that accepts these is not an admission gate.

### 5.3 Required corpus

Retain existing tests and exact full-resolution workloads. Add license-clean real designs and mathematical generators in these groups:

1. Line/arc part profiles: holes, islands, narrow slots, pocket-like inward offsets, component disappearance, many contours, and open strokes.
2. Polynomial/rational Béziers and splines/NURBS: exact splits, trims at irrational contacts, inflections, stationary derivatives, weights, poles, and nonuniform/periodic knots.
3. Corner editing: convex/reflex corners, radius/setback below/at/above limits, several valid solutions, impossible and continuous solution families, re-editing generated corners.
4. Composition: construct → transform → Boolean → offset → edit → stroke → rebuild → another operation → required export/reimport. Include Boolean-generated trim → nonuniform affine transform → containment/intersection/Boolean, using exact image equivalence rather than an invalid offset/affine commutation identity.
5. Topology: endpoint touch, crossing, high-order tangency, overlaps in both directions, shared endpoints plus later crossings, retracing, self-intersection, cusps, repeated components, zero-dimensional events.
6. Scalar stress: independent equivalent expressions, exact gaps such as `2^-k`, huge translations/tiny scales, close unequal roots, selected conjugates, coefficient growth, recognized and unrecognized transcendental zeros.
7. Resources and policy: cold/warm/evicted states, low/high budgets, strict/qualified policies, cancellation, deep expressions, concurrent ownership, serialization and replay.

Cover all finite admitted representation tags and ordered dispatch pairs. Use structured generators and covering arrays for other axes, then targeted higher-order combinations. Preserve separate fuzz buckets for difficult selected/offset/implicit representations so abundant easy line cases do not dominate selection.

Run meaningful chains of 1, 2, 4, 8, 16, and 32 operations where valid. Separate fixed-output-complexity chains from chains whose output necessarily grows. Parameterize segment count, intersection count, polynomial degree, coefficient bits, output size, and expression depth independently.

Maintain a frozen holdout corpus for admission. Newly discovered failures become permanent visible regressions, preserving both the minimized case and the original case that exposed real scale/composition behavior.

### 5.4 Metamorphic tests with stated hypotheses

- Booleans: commutativity where appropriate, idempotence, empty identities, and decomposition identities for valid regularized sets.
- Booleans commute with supported invertible affine maps.
- Euclidean-distance offsets commute with appropriate similarities, with distance scaled and signed-side orientation adjusted. General shear/nonuniform scale does not preserve that contract.
- Offset by `d` then `-d` is not generally the identity. Topology changes and regularization matter.
- Round dilation/erosion, join-based boundary offset, miter/bevel behavior, and raw analytic parallel tracing are different operations. Do not transfer an identity between them without proof.
- Fillet tangency/distance and chamfer setbacks require the declared branch/corner/trim assumptions.
- Refinement may strengthen Unknown to a certified result, but must not contradict a previous definite fact.

### 5.5 Typed outcomes and evidence

Record implementation outcome and oracle outcome separately:

```text
CertifiedComplete
QualifiedComplete
Undecided
Unsupported
InvalidInput
NumericDomainError
ResourceExhausted
Cancelled
UnboundedResult
ExternalTimeout / ExternalOOM / Crash
IncorrectResultOrEvidence

Oracle: Verified / Refuted / Inconclusive / NotApplicable
```

Successful empty geometry is a completed result. Faster refusal is not faster completion. External termination is not evidence of a typed internal resource contract. An inconclusive oracle is neither a pass nor a finding that the candidate is wrong.

Preserve the raw API result and its claimed certainty alongside the evaluator's classification, including when the evaluator records IncorrectResultOrEvidence. Do not erase the misclassification being investigated.

Let `S_base` be the set of independently validated baseline strict workflows under declared policies/resources. A change below completeness must preserve every member and its evidence. A completeness extension must enlarge the admitted capability without silently replacing successes. Protect qualified workflows under their separate contracts.

---

## 6. Measurement and acceptance protocol

### 6.1 Freeze the experiment

Pin baseline/candidate commits and patch hashes; toolchain; lockfiles; target/features; allocator; compiler/linker/profile flags; CPU/OS; runner settings; corpus/oracle revisions and digests; seeds; warmup; stopping rule; protected cases; and intended improvement.

Use a clean baseline and a clean candidate build. Preserve raw unsuccessful results. Do not change fixture resolution, precision, policy, observation/materialization, or resource limits on only one side.

### 6.2 Measure the whole promised operation

Run two distinct lanes:

- **Retained-input lane:** identical exact objects already constructed; operation plus all observations needed to discharge its contract.
- **User-workflow lane:** input construction/parsing, complete chain, required validation and receipts, materialization, export if promised, and teardown.

Report phase timings as diagnostics, but compare total promised work. Returning a lazy expression is not equivalent to completing a result whose downstream evaluation was timed on the baseline.

Record certified completion and failure status alongside times. Timeouts are censored observations; do not replace them with the timeout limit as a completed runtime or remove them from the outcome table.

### 6.3 Performance nonregression

Use a quiet runner and balanced randomized A/B and B/A process pairs. Record affinity, CPU topology/governor/thermal conditions, and build-layout assumptions. Inner iterations are not independent process replicates.

Pilot variance, then freeze sample counts or a justified sequential rule. Use enough independent observations for the promised precision; 30 process pairs can be a starting design for ordinary cases, not a universal sufficiency theorem. Long cases need an explicit feasible design. Do not quote p99 from a handful of observations.

Report absolute times, effect ratios, uncertainty, exclusions, and all samples. Use a paired/cluster-aware method. Freeze confidence level, estimator, protected metric family, and simultaneous-comparison treatment before final data collection. Run instrumentation separately when it changes wall-time behavior.

For a protected latency metric define `R = candidate / baseline`. A lower-priority candidate needs evidence that its upper confidence bound satisfies `U(R) <= 1` under the literal zero-regression rule. Failure to find a significant slowdown is not that evidence. Equal-speed noisy results may remain inconclusive; do not invent a 1% or 5% permitted slowdown after seeing the data. Any nonzero margin would require a separate explicit priority/contract decision.

This statistical rule is necessarily scoped to the protected workloads and platform. No finite benchmark campaign proves nonregression on all possible inputs. Bit-identical deployed code/data and unchanged contracts can provide stronger scoped evidence for a pure source refactor. Deterministic instruction/allocation counts help explain changes but do not alone prove elapsed-time improvement.

For a claimed performance win, require a predeclared target effect with uncertainty wholly on the improving side and protect the other designated workloads. A higher-priority exactness/completeness win can justify disclosed runtime cost; do not mislabel it a speedup.

### 6.4 Resources and caches

Measure allocation calls/traffic, live/retained heap, peak RSS/PSS where available, cache bytes, graph depth/nodes, maximum polynomial degree/coefficient bits, source/field count, refinements, promotions, and teardown.

Preserve actual resource contracts. A refinement cap is not a bound on BigInt work, heap, or elapsed time. An entry count is not a byte bound. State the largest uninterruptible unit when claiming cooperative cancellation.

For each changed cache test cold, warm, evicted, disabled where supported, independently rebuilt equivalent input, changed geometry, nested use, configured concurrency, strict → qualified → strict, and low → high → low resource/precision sequences. Stronger valid cached proof may complete an earlier Unknown. Budget-dependent failures must not poison future capability.

Use externally contained child processes for hostile or historically aborting inputs. Keep external kills separate from internal exhaustion and record platform-specific containment. Do not mistake sanitizer virtual-address reservation for ordinary retained-memory growth.

### 6.5 Binary and code size

Measure deployed artifacts, not only `rlib` archives:

- A minimal exact planar consumer.
- A representative Boolean + offset + editing consumer.
- The exported C ABI and a real linked C consumer.
- Supported WASM and relevant binding payloads.
- Default and documented minimal feature sets.

Fix compiler, LTO, codegen, panic, strip, link/GC, and compression settings. Report raw and deployed bytes, text/read-only/data/debug/symbol sections, and digests. Speed-oriented versus size-oriented build profiles are separate experiments with performance gates.

Count production source, tests, comments/docs, generated code, templates/generators, first-party dependency closure, and new third-party/build complexity separately. Track semantic owners, dispatch sites, public API count, maximum function/file size, and compile cost as diagnostics. Moving code, shortening names, hiding expansion in macros, or deleting valuable regressions is not eliminated semantic complexity.

### 6.6 Interactions and retention

For interacting candidates A and B, compare base, base+A, base+B, and base+A+B. Test the whole cumulative accepted stack and targeted three-way interactions where shared authorities/caches/dispatch suggest risk. Pairwise success is insufficient for the final stack.

Use dispositions: **kept, rejected, already satisfied, architecture inapplicable, inconclusive, blocked**. Each records assumptions, sources, evidence, adverse cases, and reason. Reconsider a rejection only with a named changed premise.

Stop broadening tests when the defined gates are satisfied, unless a concrete remaining risk or mandatory repository gate requires more. Keep long exploratory campaigns separate from routine per-edit feedback.

---

## 7. Research work packages

Every package inherits Sections 1–6. Its local checks supplement those requirements. “Keep” always means admission under the highest verified improvement, including all higher-priority protections. Every package must end in an evidence-backed disposition; research can succeed by rejecting an idea.

### P00 — Rebaseline the code, domains, costs, and unresolved cases

**Purpose:** establish the evidence infrastructure; avoid implementing stale plans.

**Work.** Read current `AGENTS.md` files, `properties.md`, admission/evidence/workflow documents, all three numerical/geometry future-work ledgers, and the applicable CI workflows. Pin the actual revision. Inventory public operations, all carrier/support/parameter/point tags, feature gates, conversions, proof owners, and downstream consumers. Remeasure source counts with inline tests separated.

Create capability and evidence matrices, reproduce existing certified-success cases, then reproduce named blockers and long compositions in contained workers. Trace where each failure occurs: construction, scalar sign/equality, field embedding, root solving, relation clipping, local topology, evidence publication, adapter, or resources. Capture exact algebraic leaves without expensive full-expression debug formatting.

**Minimum deliverable.** A machine-readable manifest and one representative workflow per existing family, with independent expected semantics, actual dispatch evidence, typed status, source/field growth, and raw observations. Import historical failures as unverified leads until rerun.

**Gate.** Correctly distinguish already fixed cases, genuinely unsupported domains, mathematical uncertainty, wrong output, and external timeout. Do not improve the baseline by lowering resolution or deleting failing fixtures. Baseline tests that cannot be run remain unverified.

**Sources/dependencies.** S1, S2, S9–S12; first package. Numerical and geometry workstreams may begin focused read-only investigation while the full baseline campaign runs, but production admission waits for relevant baseline evidence.

### P01 — Independent certificate and semantic checkers

**Hypothesis:** a small set of independent checkers can establish stronger correctness than many same-implementation differential tests, while exposing the exact proof obligations of shared abstractions.

**Pilot.** Implement independent rational line/arc fixtures and small polynomial root/order checks. Qualify an external arrangement oracle on its actual domain. Define an input/output interchange preserving exact integers/rationals, polynomial coefficients, root isolators, chart maps, and fill rules. Then check one rational-curve intersection and one Boolean region, including topology and provenance.

Use independent equations/algorithms where possible; label any shared numerical dependencies. Preserve original source equations so eliminating variables or squaring cannot hide invalid solutions.

**Gate.** Detect every wrong-answer control in Section 5.2. Demonstrate that checking all arrangement cells/branches is justified for the declared family. Distinguish an oracle timeout from verification. Native concurrency or serialization must not corrupt the checker.

**Measure/decision.** Checker coverage, false acceptance on controls, oracle domain, runtime, and reproducibility. Keep necessary assurance tooling separately from production. If no full independent arrangement is available, use narrow analytical families and explicit certificates; leave wider equivalence unverified rather than calling sampled agreement a proof.

**Dependencies/hotspots.** P00; `crates/comparisons`, exact fixture suites, external R1–R5. This package is a continuing service to all experiments, not a requirement to build a second full Hypercurve.

### P02 — Common exact span view: the authored/generated unification pilot

**Hypothesis:** common support/chart/domain/traversal/evidence access can remove origin-dependent dispatch and conversions without changing arithmetic storage.

**Alternatives.** Compare a borrowed/prepared view over current carriers with the current code. Do not begin by converting all curves to native control nets or replacing every enum with trait objects.

**Pilot.** Choose one rational Bézier in native, source-range, retained, and selected-fiber forms. Expose equivalent support, exact domain, chart map, traversal, and endpoint witnesses using existing `CurveSourceSpan2` machinery where possible. Move endpoint/evaluation/restriction and one pair-query consumer onto the view. Keep simple line/arc paths cheap and avoid allocating merely to create the view.

**Proof/tests.** Irrational trim ends; full-range versus equivalent restriction; repeated split/restrict/reverse; exact parameter replay; native image with a different chart; a pole outside versus inside the active interval; equal support with different transforms; retracing and multiple occurrences of the same point. Independently verify the parameter map, not only the image set.

**Measure/keep.** Count removed consumer branches, duplicate fields, conversions, preparations, allocations, virtual dispatch, binary bytes, and whole-workflow time. Keep only if a complete coherent pilot pays for the added abstraction under the priority rule.

**Stop.** Reject a permanent extra layer with no deletions, forced global promotion, chart incompatibility, or a higher-priority regression. The experiment may conclude current storage should remain.

**Dependencies/hotspots.** P00–P01; `curve.rs`, `curve_subdivision.rs`, `curve_evaluation.rs`, `curve_support.rs`, `bezier_split.rs`. Coordinate authority contracts with P04.

### P03 — Unified trims, traversal, native views, and canonicalization

**Hypothesis:** after consumer unification, equivalent trim storage and proof-based native views can be consolidated without erasing parameterization.

**Pilot.** Collapse one duplicated rational trim path behind P02. Preserve the source chart as authority; represent optional rebasing explicitly. Normalize nested affine transports. Extend to spline span decomposition only after the single-span experiment passes.

Audit exact simplifications such as rational curve → line/circle, full-range trim elimination, identity transform removal, shared support reuse, and exact collinear/degree-reduced images. Require a proof of the traced segment/domain and preserve the inverse/source parameter map where promised.

For a noninjective image, preserve a parameter correspondence relation rather than fabricating one inverse or deduplicating distinct source occurrences.

**Proof/tests.** `C(t)=(t,0)` and `D(t)=(t²,0)` can share an image with different charts. Equal endpoints do not exclude a loop or retracing. Include derivative chain rules; nonuniform knots; one-sided derivatives; periodic seams; major arcs; zero-length versus full traversal; reversed selected ends; poles; multiple exact parameter occurrences. A negative result from a sufficient injectivity test means “not proved,” not “self-intersecting.”

**Measure/keep.** Fewer trim/materialization owners and conversions, bounded wrapper depth, cheaper repeat restriction, stable native hot paths, and required size/runtime gates. Native extraction remains optional, lazy, and source-bound.

**Stop.** No silent public reparameterization, universal rationalization, eager control-net explosion, or loss of one-sided endpoint ownership. Do not count transformed shared lineage as geometric coincidence.

**Dependencies/hotspots.** P02 and P04; `curve_native_image.rs`, `curve_native_segment.rs`, `curve_scalar_chart.rs`, `curve_injectivity.rs`, spline and subdivision code.

### P04 — Shared selected-root authority and correlated point queries

**Hypothesis:** a common immutable authority can eliminate repeated root conversions/validation while preserving specialized local fields and coordinate correlation.

**Pilot.** Start with a borrowed view over defining relation, coefficient/base field, selected branch, isolator, and domain witness in Hypercurve/Hypersolve. Route one comparison and restriction through it. Separately express a selected point's affine/rational queries over its shared authority, without independently projecting x and y into unrelated `Real` values.

Only then consider shared owned storage/caches. Keep solver reports and geometric chart/source metadata in their respective owners. Immutable mathematical identity and refinable enclosures must not be conflated.

**Proof/tests.** Independent Sturm counts and radical examples; equal roots with different polynomials; distinct roots of one polynomial; repeated/near roots; rebuilt equivalent fields; reversed field-construction order; correlated coordinate substitution; changed image maps; malformed isolators/denominators; serialized replay. Structural `PartialEq` is not the oracle.

**Measure/keep.** Conversions, duplicate proof validation, global promotions, allocations, object layout, source retention, and downstream costs. Retain specialized backends and exact cold routes.

**Stop.** Reject eager primitive-element joins that introduce a higher-priority regression, persistent duplicate authorities without an admitted purpose, or replacement of selected-root identity by an approximate coordinate. Object layout/heap growth is diagnostic; it blocks a size-only candidate when it causes runtime/binary regression or violates a resource contract.

**Dependencies/hotspots.** P00–P02 contracts; `bezier_parameter.rs`, `selected_scalar.rs`, `curve_point.rs` and affine/rational/definition modules, Hypersolve `algebraic.rs` and represented-root types.

### P05 — S7 cusp-contact/chart consolidation

**Hypothesis:** selected rational and parallel contact maps share chart and evidence semantics even though their arithmetic payloads differ.

**Pilot.** Inventory current duplicated variants and call sites. Unify the common mapped-contact envelope: other parameter authority, circle/source identity, location, tangent/side evidence, and policy qualification. Retain rational versus two-normal/two-speed evaluators as backends. Migrate and delete one duplicate query family at a time.

The rational payload has isolated-incidence evidence/cache and two-term expressions; the parallel payload has different radical expressions. Compare an enum-backed common envelope against narrow helper extraction. Do not mechanically union every field into a large structure.

**Proof/tests.** Independent circle/line/conic examples, true tangency versus two nearby crossings, reversed/transformed contacts, exact overlap maps, endpoint inclusion, cached/replayed selected contacts, strict and qualified policies.

**Measure/keep.** Production source reduction, match-arm reduction, object size, monomorphized code, allocations, and all protected operation timings. This starts as a code-size hypothesis, so every higher priority, including binary size and performance, must pass.

**Stop.** Revert if source savings cause binary/runtime regression through generic expansion, larger payloads or repeated preparation, or cause new Unknowns or lost evidence. Payload size alone is a diagnostic under the five-rung priority.

**Dependencies/hotspots.** P04 authority contract and P02 chart semantics; `bezier_offset.rs` and `bezier_offset/cusp_semicircle/parameters.rs`. Recount references rather than relying on historical counts.

### P06 — S5 comparison ladder, local embeddings, and close roots

**Hypothesis:** one comparison authority can use retained facts and defining relations before global elimination or long blind refinement.

**Pilot sequence.** Separate storage equivalence from mathematical equality. Validate equivalent-base embeddings. Consolidate: identity/retained proof → separated enclosures → local defining-relation sign → certified embedding/join where supported → existing exact cold fallback → typed uncertainty.

Target a reproduced formula-root/polynomial-root comparison and Pell/near-zero case. For projective `n/d` values, evaluate defining polynomials homogeneously with the denominator's nonzero and sign obligations explicit. Two-sided field joins require a demonstrated supported workflow or an independently declared capability extension.

Defining-polynomial sign can order a query against a selected root only under proved conditions: for the existing simple-root route, isolate one simple root, certify opposite endpoint signs, order outside-bracket queries first, and certify an inside query's location before using the sign with its endpoint orientation. Repeated/even roots require a suitable square-free/Thom/Sturm authority or the existing exact fallback. Include negative polynomial rescaling and even-root controls.

**Proof/tests.** Exact small-field oracle; roots of `f` and `k*f`; `f` versus `f-2^-600`; different selected conjugates; reciprocal forms; positive/negative/zero denominators; equal-value transitivity; order antisymmetry; source-order independence; budget and cache sequences. Test exact zero separately from extremely small nonzero values.

**Measure/keep.** Number of comparisons decided per stage, refinements/promotions avoided, degree/coefficient growth, and complete public workflows. Eliminate a specific long route by proof, not by changing a magic bound.

**Stop.** Never delete a global fallback that still certifies baseline cases. “No global promotion” is a common-case design goal, not permission to lose completeness. Do not add a universal field-join tax to easy comparisons.

**Dependencies/hotspots.** P04, P20 accounting; `selected_scalar.rs`, `bezier_offset/recursive_field.rs`, Hypersolve recursive-quadratic, projective, and represented-root code.

### P07 — S6: remove unnecessary promotion at actual consumer boundaries

**Hypothesis:** widening consumers to the existing exact parameter abstraction can preserve local fields and avoid unnecessary global eliminants.

**Pilot sequence.** Reconcile the current S6 groups before editing:

1. Group C: selected-fiber two-radical zero queries for diameter/cusp expressions, retaining radical-sheet and sign conditions.
2. Group B: migrate Bézier-only readers to region/curve parameter operations before widening their producer type.
3. Group D: expose the actual retained fillet-center field, beginning with the existing recursive chord kernel, then rational/parallel kernels.

Retain genuinely global sites and deliberate cold fallbacks. Use exact evidence to show the conversion is removed rather than merely moved elsewhere.

**Proof/tests.** Low-degree radical formulas; original unsquared equations; all incident/complement branches; zero-distance/cusp/tangent controls; reversals/transforms; output → next stroke/offset/fillet. Check branch completeness and parameter/source identity.

**Measure/keep.** Global projections per public workflow, field degrees, coefficient sizes, exact-zero cost, conversion code, and completed compositions. Interface widening alone is not a performance result.

**Stop.** A downstream type change cannot fix a center already authored as a degree-214 global root. Route that case to P10. Reject new blockers or projection moved into another helper.

**Dependencies/hotspots.** P04–P06; S6 inventory in `FUTURE_WORK.md`, selected fiber/cusp code, `curve_corner_chain.rs`. Coordinate P10 without combining the experiments into one large patch.

### P08 — Preserve incidence, tangency, closure, and qualification

**Hypothesis:** retaining valid construction facts through rebuilds will both prevent false certification and complete operations that currently try to rediscover difficult exact zeros.

**Pilot A.** Reproduce `round_stroke_boundary_of_a_filleted_arc_corner_refillets_and_erodes`. Trace fillet → round stroke → boundary reconstruction → reversal → second fillet → erosion. Preserve the first lost support/chart-qualified endpoint or tangency witness, or demonstrate why it cannot be reused.

**Pilot B.** Reproduce the approximate-closure fixture described in future work. Distinguish genuine invalid topology from inability to admit a provisional construction under strict policy. Ensure downstream translation/profile extraction preserves qualification, and a locally certified projection does not upgrade the earlier closure.

**Pilot C.** Reproduce the reported separately transformed shared-support case: create one support and selected contact, apply equal transforms in separate calls to the consumers, then repeat contact/equality/Boolean queries. Compare transform-before-cut and deliberately different transforms. Only then decide whether stable support/image evidence is warranted; avoid speculative global interning.

Use typed, source-bound incidence/jet evidence. Do not use an unqualified “certified” boolean or attach an unbounded construction history to every curve.

**Proof/tests.** Original and minimized chains; line/line control; tiny exact nonzero-angle perturbation; reflection/scale; copy/split/reverse/transform/replay; changed support/map; tampered witnesses; strict → qualified → strict; cold/warm/evicted. Independently derive incidence/tangency equations.

**Measure/keep.** Correctly classified outcomes, new strict/qualified completions, witnesses lost/reused, exact-zero recomputation, evidence size/lifetime, and whole-chain cost.

**Stop.** Reject witness reuse on the wrong source/range/transform, approximation laundering, or relabeling a real invalid input as a policy blocker.

**Dependencies/hotspots.** P00–P01; P02/P04 assist but are not prerequisites for a narrow repair. Corner-chain, fillet, region publication/rebuild, offset joins, and `policy.rs`.

### P09 — Shared support-pair relations with explicit domain policies

**Hypothesis:** open intersections, filled arrangements, and corner-center queries can reuse geometric relations while retaining their distinct topology and selection policies.

**Pilot.** Identify one actually duplicated family, such as a retained chord/rational or rational/rational pair. Share prepared support equations and solve/clip relations on the admitted active or explicitly extended incident domains. This does not require enumerating entire unbounded supports. Publish isolated contacts, overlaps/components, paired parameter correspondence, applicable multiplicity/tangent evidence, and precise blockers.

Keep native line/arc accelerators and family-specific solvers. A fillet query may intentionally extend incident supports beyond the input trim; make this a domain policy, not a hidden exception.

Never reuse a domain-specific relation-completeness certificate outside its proved range.

**Proof/tests.** Both operand orders and reversals; crossing/touch/tangency; overlap clipped to empty, singleton, or interval; overlap plus a separate major-arc contact; noninjective/off-diagonal parameter contacts; adjacent shared endpoint plus a later crossing; residual contacts after common-component extraction. Preserve each API's documented singleton-overlap convention.

**Measure/keep.** Deleted consumer algorithms, preparations/resultants/root isolations reused, allocation/time, supported-pair matrix, and full Boolean/corner costs. Do not add another arrangement or an all-four-Boolean cache without a measured need.

**Stop.** Reject a bulk migration based on an unobserved completion defect, a universal solver that slows common pairs, empty-on-Unknown behavior, or lost topology-specific evidence.

**Dependencies/hotspots.** P02, P04, P06; `curve_support_intersection.rs`, `curve_intersection.rs`, `curve_region_boolean.rs` and pair kernels, `curve_fillet_centers.rs`.

### P10 — Construct normalized offsets and fillet centers in better local fields

**Hypothesis:** retaining the original incidence system and selected local sources at construction time avoids unnecessary global elimination in subsequent operations.

**Pilot A.** Reproduce the regular 40-gon inward-offset bounds/own-vertex case. Compare current unit-normal coordinates with homogeneous or unnormalized support relations and explicit norm/side evidence. An alternative is a narrowly justified scalar identity recognizing the positive square-root branch. Do not duplicate a line-fillet incidence improvement that has already landed.

**Pilot B.** Begin with two modest rational quadratic offset supports. Construct the selected center as a local fiber, preserve its source parameters and incidence equations, and feed it into trim, fillet replay, and stroke. Then target the difficult NURBS-pair composition. Ensure the center was never first globally projected and merely wrapped afterward.

**Proof/tests.** Independent small resultants, root counts and isolating boxes; back-substitution into unsquared distance/normal equations; nonzero direction and denominator proofs; radical sheets; reflection/sign; all finite valid centers; impossible and continuous center families; generated-center reuse.

**Measure/keep.** Global and relative degree at each stage, extraneous factors, coefficient bits, number of joined sources, promotions, runtime, and completed public chains. Preserve optional global projection as a cold view where necessary.

**Stop.** Local representation cannot remove an intrinsic degree-164 selected value. Do not repeat the rejected degree-214 → 164 substitution without a new premise. Reject lost branch signs, incomplete center enumeration, or costs merely shifted to every ordinary line/arc.

**Dependencies/hotspots.** P04/P06/P07, P08 evidence, possibly P12; `offset.rs`, `curve_fillet_centers.rs`, `curve_corner_chain.rs`, selected-fiber and recursive-field code.

### P11 — Lazy sign leaves and verified modular nonzero certificates

**Hypothesis:** genuinely multi-source expressions may spend more time expanding dense polynomial leaves than deciding their signs.

**Preflight.** Capture actual leaves from blocked/slow filleted-union, weighted, or chamfered multi-source workflows. Separate expansion, reduction, enclosure, zero-test, and field-GCD costs. The previously improved univariate H13 fixture is not sufficient justification.

**Pilot.** Retain the recursive radical sign rules. Change one leaf representation to a bounded expression DAG over reduced inputs. Try rigorous interval evaluation on the shared source box, then applicable modular nonzero evidence, then refinement/existing exact zero fallback.

Specify good-prime conditions: denominators invertible, leading terms valid, coefficient relations correctly reduced, and certificate applicable to the selected root rather than merely some conjugate. Modular nonzero is not a sign; certified real separation must determine positive/negative. Zero modulo several primes is not an exact zero proof.

**Proof/tests.** Independently expanded small towers; exact cancellation; repeated radicals; dependent sources; near-zero nonzero values; bad/unlucky primes; denominator poles; reducible defining polynomials and selected conjugates. Validate any norm/coprimality certificate.

**Measure/keep.** Dense expansion avoided, peak coefficient bits, work/allocations, exact fallback rates, and public-workflow completion/runtime. Retain the measured small-case dispatch protections already present.

**Stop.** Reject if no captured leaf matches the premise, exact zeros regress, cost moves to an equally expensive GCD, or unconditional modular work slows common cases. Previously rejected enclosure/field variants need changed assumptions before reconsideration.

**Dependencies/hotspots.** P00 profiles, P04/P06 authority, P20 resources; recursive-field/sign kernels and the owning polynomial arithmetic layer.

### P12 — Modular/CRT common-fiber GCD and separate root-count research

**Hypothesis:** recorded degree-36/40 fibers over a degree-14 selected field suffer coefficient swell that verified modular reconstruction can avoid.

**Pilot.** Extract actual inputs from the relevant fillet/stroke/erosion workflows. Implement a private GCD candidate generator with the current exact chain retained. Document the true coefficient field/localization and good-prime conditions; do not assume every selected polynomial defines an irreducible field.

Reconstruct via CRT/rational reconstruction. Verify exact divisibility into both inputs **and maximality**, for example using a valid good-reduction degree bound matching the verified common divisor. A verified common factor alone does not establish a GCD.

After that experiment is independently admitted or rejected, compare certified square-free Descartes/Bernstein root counting with the current Sturm route. Keep endpoint inclusion, repeated-root treatment, and selected-base semantics explicit.

**Proof/tests.** Independent small-field GCDs; planted factors with coprime cofactors; vanishing leading coefficients; zero polynomials/fibers; bad primes; repeated factors; selected conjugates; endpoint roots. Replay all reconstruction witnesses and independently count roots.

**Measure/keep.** Exact arithmetic/bit complexity, reconstruction attempts, fallback rate, peak coefficients, and full workloads. Evaluate small inputs separately to choose a justified dispatch threshold.

**Stop.** Do not apply this to an identically-zero remainder such as H14. Revert unverified maximality, field-division cost displacement, or speculative reconstruction with no end-to-end benefit.

**Dependencies/hotspots.** P04/P06 and P20; `count_bivariate_fiber_system_roots` and owning Hypersolve polynomial kernels. P10 may change future inputs; remeasure their interaction.

### P13 — Rigorous filter ordering and construction-time scalar compaction

**Hypothesis:** a cheap certified enclosure can decide common nonzero predicates before expensive expansion, while exact construction identities can prevent expansion entirely.

**Preflight.** Profile the current revision after existing scalar/coordinate compaction. The previously examined small binary64 filter could not handle the dominant radical/trigonometric graphs; moving it earlier unchanged is not a new experiment.

**Pilot.** Add one rigorously outward-enclosed node family at a time, such as admitted positive square roots or cached certified constants. Preserve overflow/underflow/domain handling and the complete existing exact cascade for unresolved cases. Alternatively simplify a construction using an already proved relation at its source.

Compare filters, construction-side reduction, and no change. Avoid a new public workspace or universal normal form without a measured residual workload.

**Proof/tests.** Independent exact dyadic or validated interval containment; ulp boundaries/subnormals; huge/tiny values; perfect and imperfect roots; exact zeros; severe cancellation; graph/work limits; recognized versus unrecognized transcendental identities. Testing proximity to a float value is insufficient.

**Measure/keep.** Filter hit rate and overhead by representation, certified decisions, exact-stage work avoided, full workflow runtime, and code/binary cost. A rigorous filter may only return a definite result when its enclosure proves it.

**Stop.** No epsilon zero, float authority, dropped exact fallback, or unconditional expensive normalization. Reopen prior rejected memo/workspace variants only with a named changed premise.

**Dependencies/hotspots.** P00 profiles and P20/P21; Hyperreal/Hyperlimit owners. R3 and R6 are research references with restricted applicability, not drop-in universal solutions.

### P14 — Exact extrema, containment, and source-location witnesses

**Hypothesis:** exact stationary-coordinate decisions will complete practical CAM queries that conservative control hulls cannot decide at tangency.

**Pilot.** Extend nonmonotone rational cubic box containment. For `x=P/W`, use roots of `P'W-PW'` on a certified pole-free active interval, with endpoint and constant-coordinate handling. Query each candidate against the box sides without requiring eager global Cartesian materialization. Then apply the same contract to retained/source-range trims and NURBS spans.

Keep `bounds()` as its documented conservative enclosure. Add an explicit tight-extremum/containment query only where useful. A valid loose enclosure is not a correctness bug.

**Proof/tests.** Interior irrational extrema, repeated derivative roots, constant coordinates, endpoint maxima, degree ≥4, bound exactly tangent and `2^-k` above/below, poles inside/outside trim, selected endpoints, transformed spans, periodic seams. Verify both a contained result and an outside witness with its source parameter.

**Measure/keep.** Newly certified cells, root isolation/reuse, witness transport, exact predicate cost, and ordinary enclosure fast-path cost. Use cheap enclosures first when they already prove containment/disjointness.

**Stop.** Hull subdivision is not a complete exact fallback for every tangential case. Do not silently turn every bounds call into expensive exact extremum solving or claim a witness in the wrong chart.

**Dependencies/hotspots.** P00–P01; P02/P04 help sharing. `curve_box_containment.rs`, `rational_bezier_general.rs`, `bezier_retained_measure.rs`.

### P15 — Certified distance, clearance, length, and curvature queries

**Hypothesis:** a coherent query contract over common spans can complete important CAD/CAM checks while distinguishing algebraic values, exact computable quantities, and certified enclosures.

**Pilot sequence.** Inventory current line/arc distance and Bézier length/inverse-length APIs. Add one demanded gap: closest point or distance-threshold decision on a finite rational span, followed by selected trims. Include stationary distance candidates, endpoints, poles, repeated minima, and branch/domain restrictions. Treat exact squared distance, certified distance intervals, and comparison against a threshold as distinct outputs.

A later demanded clearance milestone covers curve/curve, filled-region, and tool-footprint/material queries. Account for endpoint and stationary systems and positive-dimensional minimizing families. Combine boundary distance with overlap/containment: positive separation of two boundaries does not imply their filled regions are disjoint.

For length and inverse length, retain certified enclosures and parameter brackets where elementary/algebraic closed forms are unavailable. An exact constructive-real integral carrier is also a valid research option if its refinement and error bounds are verified; lack of an algebraic closed form does not make an exact computable representation impossible. Compare its consumer value and costs against the existing interval API, without promising total equality for such integrals. For curvature limits, specify regularity, derivative order, zero-speed/cusp behavior, signed versus unsigned curvature, and threshold semantics.

**Proof/tests.** Independent line/circle/parabola cases, symmetric multiple minima, flat/constant-distance families, exact tangency to clearance threshold, near-threshold separation, cusp and zero-length cases, reversed/trimmed/periodic spans. Demonstrate enclosure convergence or honest limits.

**Measure/keep.** New useful queries and composed clearance/length workflows, enclosure width versus work, source witness fidelity, and speed/size. Publish the exact guarantee returned.

**Stop.** No sampled “minimum,” unproved exact integral/equality promise, or arbitrary choice of one closest point when the contract requires all solutions. Do not add unused metric APIs solely to enlarge a feature checklist.

**Dependencies/hotspots.** P02/P04/P14, root solver; `segment_metric.rs`, `bezier_metric.rs`, differential and retained-measure modules.

### P16 — Exact offset/stroke completion and semantic separation

**Hypothesis:** clearer stages and preserved witnesses can reduce offset complexity while completing generated-output reuse.

**Pilot.** Map the existing pipeline: regular support analysis → parallel/cap/join candidates → singular/contact events → trimming/arrangement → regularized region → proof-qualified publication. Identify one duplicated or blocked invariant, not an entire replacement engine.

Define separately the raw signed parallel, open stroke with named caps/joins, join-based contour offset, and disk-based set dilation/erosion where offered. Radius sign, winding, fill, miter limit, stationary speed, and cusp behavior must be explicit.

**Proof/tests.** Line/arc controls; rational cusp onset; narrow necks; holes/islands; inward component disappearance and splitting; outward merges; round/bevel/miter joins; butt/round/square caps; self-crossings; zero radius; exact threshold radii; generated inputs; repeat offset/Boolean/corner edits. Verify all retained material and discarded branches.

**Measure/keep.** One formerly blocked workflow or duplicated semantic owner removed, complete set/topology oracle, field growth, and per-stage/whole-chain costs.

**Stop.** Do not replace analytic parallels by finite rational/chord approximations under an exact contract, or treat a candidate parallel collection as a valid region. Exact analytic offset representations can be essential even when an approximate rendering is easy.

**Dependencies/hotspots.** P08/P09, targeted P10–P12 as traces require; `bezier_offset`, `bezier_region/offset_spans.rs`, joins, region offset/stroke code.

### P17 — Fillet/chamfer solution completeness and editable generated corners

**Hypothesis:** common relation and local-field contracts can reduce large corner dispatch while preserving all valid finite solutions and subsequent editability.

**Pilot.** Choose one blocked generated-carrier corner. Specify whether incident supports may extend, which sides/branches are allowed, radius/setback constraints, trim feasibility, and solution ordering. Use existing center relations; separate candidate solving from corner selection and boundary surgery.

Admit exact finite solution sets; distinguish no solution, a discrete set, and a continuous family requiring an additional constraint. Preserve shared tangency/contact evidence through publication and the next edit.

Deduplicate by the promised solution identity, including contact occurrences and trims. Equal center coordinates may correspond to distinct valid solutions.

**Proof/tests.** Convex/reflex and reversed corners; line/arc/rational pairs; equal/tangent supports; all finite centers; radius at geometric limits; overlapping edits; zero-length trims; stationary endpoints; impossible and continuous families; fillet → stroke → fillet/chamfer → Boolean/offset. Validate original unsquared distance/tangency and exact setbacks.

**Measure/keep.** Completed corner/workflow cells, duplicated family dispatch deleted, candidate/root count, exact branch filtering, maximum field growth, and user-level latency. Keep simple line/arc paths specialized if they win higher priorities.

**Stop.** No arbitrary sampled center from a continuous family, hidden tolerance for contact, discarded difficult solution, or endpoint-only validation of the edited image.

**Dependencies/hotspots.** P08–P10 and P16; `curve_fillet_centers.rs`, `curve_fillet.rs`, corner chain/edit modules. Split long dispatch by semantic owner only after mapping actual duplication.

### P18 — Arrangement event, overlap, winding, and degeneracy invariants

**Hypothesis:** a compact set of shared local-topology rules can replace scattered degeneracy cases and improve closure under repeated Boolean operations.

**Pilot.** Inventory the existing authoritative arrangement, local sector ordering, overlap ownership, and publication checks. Pick one reproduced ambiguity involving tangency, overlap endpoints, shared events, or retracing. Formalize its event/sector invariant and migrate the relevant consumers.

Keep open-curve contacts, closed-set boundaries, and regularized filled material semantics distinct. Use event identity plus certified local behavior; equality of coordinates alone does not establish identical branches.

**Proof/tests.** Several curves meeting at one point; high-order tangency; cusp; transverse crossing beside an overlap; coincident reversed loops; repeated winding; nested holes/islands; zero-area collapse; mixed empty/nonempty operands; invalid input paired with empty; noninjective support; transformed copies. Check all sectors, face labels, boundary ownership, and canonical empty output.

**Measure/keep.** Eliminated duplicate topology decisions, new certified degenerate/composed cases, event sorting/sign cost, output complexity, and all four Boolean operations.

**Stop.** Do not build a second arrangement, special-case an unresolved predicate as “no event,” infer crossing from multiplicity-erased algebra, or let an empty shortcut bypass input validation.

**Dependencies/hotspots.** P08/P09; region Boolean modules, point location, split topology, publication/validation. P19 extensions must satisfy these same invariants before filled admission.

### P19 — Implicit polynomial branches, multiplicity, and vertical components

**Hypothesis:** staged solver-certified branch support can extend the common contract without forcing every algebraic curve into a rational parametrization.

**Stages.** First reproduce the currently admitted line/branch domain. Then:

1. Reuse cut witnesses instead of isolating the same event twice.
2. Admit line-component overlaps with residual finite contacts.
3. Admit selected/algebraic line coordinates and mapped branches by exact pullback.
4. Add branch pairs, repeated/vertical components, singular events, and certified adjacency in the solver.
5. Add point location/local sectors, then filled loops and Boolean closure.

Separate x-content `c(x)` from the primitive part before square-free reduction with respect to y. Publish multiplicity/source-role evidence before removing repeated-component blockers. Existing BRep consumers use these distinctions to avoid admitting tangency as a crossing. Vertical and boundary components require their own carriers/ownership.

Filled admission must specify oriented-path winding or explicit polynomial-inequality semantics. A zero locus alone has no chosen interior: `F` and `F²` have the same zero set but different sign domains. Preserve the chosen fill semantics through factor reduction and branch orientation; algebraic factor multiplicity is not automatically boundary winding multiplicity.

**Proof/tests.** Circle upper/lower branches; multiple branches at one projected root; even tangency; isolated points; vertical tangents; line component plus residual contacts; repeated factors; boundary factors; mapped source-x versus world-x; selected conjugates; malformed selectors. Use independent factorization/topology and downstream plane-section controls.

**Measure/keep.** Explicit newly admitted polynomial classes, relation/sector completion, repeated isolation, degree/coefficients, and practical CAD/CAM workflows.

**Stop.** No filled output before all necessary event/sector/ownership proofs exist. Square-free zero-set equality alone does not preserve multiplicity semantics. General implicit support is not an automatic replacement for native lines/circles/rational spans.

**Dependencies/hotspots.** P04/P06/P09/P18; P12 as needed; polynomial-curve modules and Hypersolve bivariate topology, plus affected HyperBRep consumers.

### P20 — Truthful resource contracts, cancellation, and safe lifetime

**Hypothesis:** accounting at the actual expensive operations and safe cleanup can improve practical completion and failure behavior without weakening exactness.

**Pilot A.** Inventory which current work units are charged. Add opt-in, clearly scoped limits for graph nodes, degree/products, coefficient bits, exact-work units, and cancellation checkpoints in one owner at a time. State which are hard bounds and which are proxies. Keep existing unlimited/default semantics unless an explicit contract migration is part of an admitted change.

**Pilot B.** Reproduce the deep-expression recursive-drop abort in a contained process. Evaluate the remaining depth/iterative-cleanup ideas against current ordinary construction/drop cost and shared/concurrent last-owner behavior. Existing rejected variants are counterevidence, not permission to repeat them unchanged.

**Proof/tests.** Limit just below/at/above threshold; large single promotion; nested/worker context propagation; cancellation during algebra/publication/cleanup; no partial certified result; small request after exhaustion; larger-budget recovery; no cached capability failure; shared-node release without cycles/stale identities. Update any production fact-word/cache-encoding proof if layout changes.

**Measure/keep.** Accounting coverage, maximum uninterruptible work, actual allocations, teardown, crash-free admitted envelope, and common-case overhead. A stack-safety completion gain may justify disclosed lower-priority cost; do not label every memory optimization an exactness improvement.

**Stop.** No hard heap claim without controlling relevant internal allocations, including BigInt dependencies. External containment remains honestly external. Reject new default limits used to make slow valid cases disappear.

**Dependencies/hotspots.** P00; numerical budgets, Hyperreal lifecycle, actual cache owners. Instrumentation can start early and support P06–P13.

### P21 — Prepared queries, cache ownership, and conservative spatial pruning

**Hypothesis:** reusable immutable preparation and rigorous broad-phase bounds can reduce repeated work without a new semantic authority.

**Pilot.** Profile repeated region/curve queries, pair preparation, and source normalization. Separate immutable authority from evictable derived data. Choose one repeated query or source preparation. Compare operation-local reuse, retained prepared objects, and existing caches; keep a public prepared API only if a measured consumer needs it.

For pair enumeration, prototype conservative hierarchical bounds or event indexing only where the current code demonstrably does unnecessary pair work. Prune solely on certified disjointness. An uncertain bound must preserve the candidate pair.

**Proof/tests.** Cache-state matrix from Section 6.4; same source under different transforms; equivalent independently constructed supports; changed policy/budget; nested/concurrent use; eviction; failed preparation followed by success; tight touching versus separated boxes; all true contacts retained. If parallelizing independent work, propagate contexts and preserve deterministic output contracts.

**Measure/keep.** Preparation cost plus amortization, one-shot latency, hit rate, retained bytes, pair count versus output complexity, and whole operation. Authority must outlive dependents safely; caches must not accumulate entire edit history.

**Stop.** Reject universal caching without reuse, failure poisoning, policy-unsafe keys, bounds that miss tangencies, or a preparation object that only moves existing work into an unmeasured phase. Memory is diagnostic unless a contract or higher priority is affected.

**Dependencies/hotspots.** P02/P04/P09, P20; region query caches, pair preparation, conservative bounds and spatial indexes. Recheck prior lazy-preparation improvements before adding another layer.

### P22 — Certified finite projection and per-span fidelity

**Hypothesis:** explicit per-output-span fidelity makes approximate output composable and can simplify adapters without contaminating exact topology.

**Pilot.** Extend one existing certified parallel/projection result beyond aggregate counts. For each output span retain source identity, source interval/chart, approximation type, proven bound, norm/metric, and qualification. Preserve it through path extraction, split, reverse, transform, and supported export/reimport.

Compare existing subdivision/candidate fitting with a researched approximation method only under the same certificate checker. Approximate algorithms may generate candidates; an independent bound must admit them. Keep this lane distinct from exact region operations.

**Proof/tests.** Hausdorff/set-distance versus same-parameter error; inflections/cusps; repeated/retraced spans; endpoint tangency; tiny features; round-trip lossless versus quantized formats; cumulative error propagation under composition. For example, `C(t)=(t³,0)` and `L(t)=(t,0)` trace the same set but differ by `3/8` at `t=1/2`. A set-distance certificate alone does not prove same-parameter accuracy.

A whole-span Hausdorff bound cannot simply be copied to equal-parameter restrictions: restricting this example to `[0,1/2]` gives image intervals `[0,1/8]` and `[0,1/2]`, whose Hausdorff distance is `3/8` despite the original zero bound. Split/trim must use a certified local correspondence to derive a valid bound, revalidate each new piece, or retain only the original whole-span claim. Source-interval metadata alone is insufficient.

Under affine transforms, propagate distance bounds with a certified operator-norm bound; under nonlinear operations derive a new bound or return appropriate qualification. Do not assume arbitrary error tolerances add through offset/Boolean topology.

**Measure/keep.** Certified output density, approximation work, evidence size, downstream recoverability, and same-guarantee runtime. Machine/controller quantization remains downstream.

**Stop.** No receipt that upgrades earlier provisional input, omitted source spans, changed error norm hidden under “tolerance,” or finite approximation inserted into an exact Boolean path.

**Dependencies/hotspots.** P08, P02/P04, P27; certified parallel/path projection and format adapters. R7 supplies candidate ideas, not certification.

### P23 — Skeletons, translation obstacles, and Minkowski scope

**Hypothesis:** selected CAD/CAM needs may justify completing existing specialized domains or deriving broader operations from exact decomposition and union.

**Pilot A.** Rebaseline current line/arc skeleton event, split, and simultaneous-collapse behavior. Select one demonstrated consumer gap. Specify whether the requested object is a straight skeleton, medial axis, or offset event structure; these are different contracts.

**Pilot B.** Rebaseline convex line-contour translation obstacles. For a named nesting/clearance use case, evaluate concave decomposition plus exact union with explicit hole/contact semantics. Treat curved Minkowski sums and general curved skeletons as later research with their own representation/solver requirements.

**Proof/tests.** Analytic rectangles/triangles/circles where applicable; simultaneous events; concave pockets; holes; zero-width contacts; reflected translation conventions; placement just inside/on/outside forbidden boundaries; independent exact small arrangements. Check coverage and overlap of decomposition pieces.

**Measure/keep.** Newly completed named consumers, exact topology, event count/output growth, runtime, and dependency/source cost. Prefer reuse of existing certified primitives when it pays.

**Stop.** No “generic offset” wrapper claimed to solve all skeleton/Minkowski problems, or breadth added without an admitted CAD/CAM requirement. If prerequisites are absent, record a bounded research result and precise blockers.

**Dependencies/hotspots.** P18 and relevant P14/P16/P19; straight-skeleton and translation-obstacle modules, downstream placement/CAM callers.

### P24 — Binary footprint, feature boundaries, and compilation strategy

**Hypothesis:** redundant instantiations, overly broad exports/dependencies, or forced optional facilities may contribute avoidable shipped code.

**Pilot.** Extend the existing size harness. Attribute dominant linked sections and instantiations. Compare narrow generic factoring, non-generic internal helpers, selective type erasure, visibility/inlining changes, and dependency/feature separation one at a time. Preserve specialized hot paths.

Evaluate an optional external algebra backend as a separate deployment experiment, including native libraries, bindings, license obligations, build/release complexity, supported platforms, and exact fallback. Do not count transferred code as eliminated complexity.

**Proof/tests.** Same API/ABI and result qualification; actual external consumers; default/minimal/all supported feature configurations; native/WASM targets; stripped and full ABI artifacts. Force result observation so dead-code elimination cannot erase the benchmark.

**Measure/keep.** Full deployed bytes and sections, dependencies, build time as a diagnostic, performance, and completion. A smaller configured product with fewer features is a different capability contract, not a same-capability size win.

**Stop.** No default capability removal, changed error behavior, slower protected workload, hidden system-library dependency, or `rlib`-only claim. Size profiles that slow execution do not pass a binary-only improvement gate.

**Dependencies/hotspots.** P00/P01 and Section 6.5; `benchmarks/size-harness`, crate features, public exports, C/WASM wrappers. Numerical backend replacement also requires P28.

### P25 — Semantic modules, concise names, and test corpus organization

**Hypothesis:** removing repeated semantic ownership and organizing tests by invariant can reduce maintenance cost without sacrificing regression evidence.

**Pilot.** Map the large offset/conversion test modules and long corner dispatch. Move one cohesive test family without changing semantics; preserve old-to-new test identification and discovered cases. Extract repeated fixture construction only when the oracle remains independent. In a separate patch, factor one private dispatch by mathematical owner or shorten a redundant private naming family.

Compare explicit helpers/tables with macros/code generation. Generated source may be useful, but count templates, generator, inputs, expanded code, debug quality, and toolchain complexity. Prefer fewer semantic cases over denser formatting.

**Proof/tests.** Same fixtures, exact policies, independent assertions, intended representation/dispatch, ignored hard cases, and full-scale originals. Organizational changes should not alter production behavior or serialized tags. Existing test discovery/diffs usually suffice for a purely mechanical move; do not add tautological tests that merely mirror the new file structure.

**Measure/keep.** Genuine source removed, repeated setup reduced, file/function size, duplicate owners, test discovery/compile cost, and all higher-priority gates. Use a ratchet on touched files with justified exceptions.

**Stop.** No fewer adversarial cases masquerading as consolidation, mass public renaming without migration value, macros that hide proof obligations, or a giant repository-wide reformat mixed with algorithms.

**Dependencies/hotspots.** P00 coverage inventory; `bezier_offset/conversion_tests.rs`, region tests, cusp families, corner dispatch. Organize near substantive work rather than blocking the program on cosmetic churn.

### P26 — Focused formal assurance of actual proof boundaries

**Hypothesis:** a few small verified boundaries can make consolidation safer and reduce duplicated defensive reasoning.

**Pilot.** Inventory existing verification models and their connection to production. Choose one critical boundary: interval sign implication, selected-root/domain validation, cache proof applicability, parameter-map composition, or local event ownership. State assumptions and model the actual implementation or mechanically checked correspondence.

Use proof assistants, bounded model checking, property-based generation, and mutation testing according to the boundary. Reuse current tooling before adding another verification language. A checked certificate protocol may be a more useful first target than attempting to prove the entire kernel.

**Proof/tests.** Invalid witness and policy controls; overflow/tag encoding; concurrent ownership where relevant; source/chart mismatch; nonzero denominators; repeated roots; finite-model assumptions. Connect every theorem to a named production entry point and concrete build configuration.

**Measure/keep.** Verified obligations, mutation rejection, trusted assumptions, production correspondence, and maintenance burden. A theorem about a toy model does not automatically strengthen production exactness.

**Stop.** No unsupported whole-library “formally verified” claim, proof that silently assumes the difficult zero decision, or new model maintained independently of implementation.

**Dependencies/hotspots.** P01 and owning package; existing `verification` models, fact/cache encodings, receipt and publication boundaries. Keep proofs focused enough to review alongside code.

### P27 — CAD/CAM integration, bindings, replay, and packaged consumers

**Hypothesis:** output usability and qualification across public boundaries are part of completion, and current workspace tests may miss distribution or adapter gaps.

**Pilot.** Select a representative planar CAD editing consumer and a downstream CAM geometry consumer already present in the repository. Exercise exact construction → transform/edit → Boolean/offset → receipt/topology inspection → required export/reimport → next operation.

Run a clean external consumer using the intended package installation, without workspace patches masking dependency/version issues. Cover supported C ABI/WASM/bindings and exact versus finite file formats. Preserve source/chart/fidelity records where the format promises them.

Across serialization/process boundaries, preserve or strictly reconstruct semantic source/root/chart identity; pointer identity is not an interchange contract. If a format cannot express selected evidence, expose the loss or return a typed blocker instead of calling a manufactured native reconstruction lossless.

**Proof/tests.** Exact scalar transport; empty and unbounded outcomes; all typed failures; provisional closure; selected generated boundaries; holes/winding; round-trip parameter identity; explicit loss in formats that cannot encode the exact source. No adapter-local tolerance may “repair” a kernel blocker under an exact contract.

**Measure/keep.** Completed deployed workflows, payload/binary size, parsing versus native runtime, and predictable error/qualification handling. CAM checks must name the geometry they establish, such as clearance or remaining material.

**Stop.** No claim of machine-safe complete toolpath planning from a region offset alone, no hidden quantization, and no workspace-only release claim. Keep planner/feed/controller policy downstream.

**Dependencies/hotspots.** P08/P16/P17/P22/P24 as relevant; root downstream profiles, Hyperpath CAM modules, wrappers, serialization, examples, and package manifests.

### P28 — Alternative algorithms and simplification of specialized fast paths

**Hypothesis:** some specialized branches can be replaced by a simpler complete method, while others are necessary to protect practical performance.

**Work.** For each hotspot, write an applicability sheet: theorem/domain, current overlap, alternatives, oracle, fallback, expected benefit, and licensing/platform cost. Evaluate candidates only against captured inputs and ordinary controls.

Candidate families include certified subdivision with exact singular fallback; interval Newton/Krawczyk-style isolation on domains where their assumptions hold; symbolic-numeric bivariate isolation/lifting; sparse versus dense polynomial layouts; fraction-free/subresultant versus verified modular elimination; and existing external exact backends. Read the original method before implementation.

**Pilot.** Replace one duplicated pair or polynomial backend behind the same exact result protocol. Keep the old implementation temporarily for differential evidence and independently check both. Separately experiment with removing one expensive-to-maintain fast path, measuring the whole domain it protects.

**Proof/tests.** Completeness of root/event enumeration, no generic-position assumption unless part of the contract, multiple roots/tangencies, positive-dimensional overlaps, poles/singularities, coefficient-field restrictions, source selection, and complete fallback. A uniqueness certificate in a box does not enumerate every box; a numerical root guess is not a solution proof.

**Measure/keep.** Full pipeline cost, dispatch-domain coverage, degree/bit/output scaling, binary/source change, and dependency burden. Use a justified dispatch crossover when no one method dominates.

**Stop.** No approximate solver substituted for an exact one, no new Unknown replacing a solved singularity, and no speed claim from a paper transferred to a different coefficient domain without measurement. If retaining two algorithms is required by higher priorities, keep both with clear ownership.

**Dependencies/hotspots.** P00/P01, P09/P11/P12, P20; R1–R6. This is an evidence-driven alternative-method queue, not a simultaneous rewrite of the numerical stack.

### P29 — Cumulative closure, adversarial campaigns, and accepted-stack validation

**Hypothesis:** individually sound improvements can interact through representations, caches, algebra, and topology; acceptance must cover the integrated library.

**Work.** After each retained change, extend the success ledger with new capabilities and failures. For interacting changes run the 2×2 matrix, then the cumulative stack and targeted higher-order combinations. Re-run the protected holdout, all existing long compositions under their recorded envelopes, and relevant downstream consumers.

Use stratified fuzzing driven by wrong witnesses, weak completion, unvisited dispatch, expression growth, and policy/cache interactions. Preserve exact seeds, original and minimized failures, and final-observation requirements. A fast Unknown receives no completion/performance credit.

**Proof/tests.** All Section 5 axes, plus returned outputs fed directly into the next supported operation, chain depth growth, copy/reverse/split/transform/replay, final teardown, and qualified-to-strict boundary checks. Verify branch coverage actually reaches each intended representation.

**Measure/keep.** Set difference in completed workflows, no incorrect claims, all protected higher-rung metrics, cumulative dependency/artifact/source cost, raw evidence, and reproducible release gates.

**Stop.** Revert or repair a candidate whose interaction breaks a higher priority even if it passed alone. Do not permanently retain shadow implementations, duplicated adapters, or experimental feature paths without an admitted production role.

**Dependencies/hotspots.** Every retained package; P00/P01 harness, all touched lower layers and downstream consumers. This runs throughout the program and is the final production gate.

---

## 8. Execution phases and dependency gates

### 8.1 Start with three concrete engineering bets

After the relevant P00/P01 baseline is usable, the strongest initial experiments are:

| Initial experiment | Why start here | Decisive result |
|---|---|---|
| P02 rational-span adapter | Directly tests authored/generated unification against existing infrastructure, with limited storage risk | One consumer family becomes simpler with identical semantics and all higher-priority gates satisfied, or the adapter is rejected |
| P08 tangency/closure evidence | Tests whether expensive downstream failures originate in lost facts or qualification | Reproduced chains complete with valid retained proofs, or the current failure is precisely reassigned |
| P14 rational cubic containment | A bounded operation gap with analytic/root-based independent checks and practical CAM value | Nonmonotone/tangent/selected-trim cases complete under an explicit domain while cheap bounds retain their contract |

These can run in parallel with non-overlapping owners. Preserve one accepted baseline and coordinate changes to shared authority types. P25 should organize only the files needed by these experiments; a large cosmetic refactor is not a prerequisite.

### 8.2 Phase gates

| Phase | Packages | Exit condition |
|---|---|---|
| 0: establish truth | P00, initial P01; inspect P20/P24 instrumentation | Reproducible baseline, typed status/evidence matrices, independent checkers for the first targets, frozen priority and measurement protocol |
| 1: small architectural and capability pilots | P02, P08, P14; focused P25 | Each pilot admitted or rejected with raw evidence; no permanent duplicate layer; first new closure/query capabilities recorded |
| 2: consolidate authority and consumers | P03–P07, one P09 family | Source/chart/root contracts stabilized; unnecessary conversions deleted; old exact success routes retained or independently superseded |
| 3: profile-driven numerical research | P10–P13, P20/P21, selected P28 | Actual bottlenecks reduced or completed without false proof; negative experiments archived; common workloads protected |
| 4: expand practical CAD/CAM closure | P15–P18, P22, P27 | Named Tier A/B chains and certified-output guarantees pass independent and downstream gates |
| 5: selective advanced domains | P19, P23, later P28 | Each polynomial/skeleton/placement extension has an explicit admitted domain and complete prerequisite evidence |
| 6: footprint and maintainability | P24/P25 where measured opportunities remain | Same-capability deployed/source reduction under every higher-priority gate |
| Throughout/final | P26 where useful, P29 | Verified boundary correspondence, cumulative interactions, holdout, release/consumer checks, and honest completion report |

Phases describe dependency logic rather than an all-or-nothing waterfall. A focused bounds repair need not wait for field GCD research. Binary/source experiments can proceed early if their higher-priority evidence is available. Advanced branches must not delay delivery of completed ordinary CAD/CAM improvements.

### 8.3 Work allocation

Use independent worktrees/branches for:

- **Geometry contracts:** span/charts, proof retention, relations, bounds/topology.
- **Numerical kernels:** local fields, comparisons, construction, sign/GCD/filter experiments.
- **Validation and integration:** independent oracles, reproducibility, consumer/binary/corpus checks.

With one CLI agent, execute these queues serially while keeping the same separation of hypotheses. With parallel agents, assign one owner for each shared interface, file family, and admission record. Agree on contracts before simultaneous edits. Avoid having two agents invent different selected-root or span abstractions.

### 8.4 One-hypothesis implementation loop

1. Confirm the selected case still exists on the current accepted revision.
2. Read the relevant source, tests, ledgers, and primary algorithm reference.
3. Write the experiment record before changing production code.
4. Freeze exact inputs and expected result, including required exhaustive and downstream evidence.
5. Capture an original and minimized public reproducer; retain both.
6. Implement the smallest independently testable prototype. Keep representation and solver-algorithm rewrites separate.
7. Run focused semantic/dispatch/policy tests first; stop on a higher-priority failure.
8. Measure the intended benefit and protected metrics with matched builds.
9. Integrate with the current accepted stack and run relevant interaction/consumer gates.
10. Admit or revert only the candidate's own changes; preserve user work and unrelated accepted work.
11. Remove superseded code and temporary comparison paths, rerun affected gates after deletion, and write the evidence-backed decision.
12. Update the capability ledger and next bottleneck. Newly admitted successes become protected baseline cases.

Maintain reviewable commits: fixture/oracle or reproduction; implementation; justified deletion; evidence/documentation. Do not combine mechanical movement, type migration, new elimination mathematics, and output semantics in one opaque patch.

### 8.5 Research stopping rules

Before a substantial experiment, name the decisive fixture, measured premise, prototype boundary, proof checker, and investigation budget. The budget limits experimental effort, not the mathematical resource contract of production APIs.

Stop or redirect when:

- The failure is already fixed or the alleged duplicate path does not exist.
- A prototype cannot remove any authority/branch and adds overhead without a higher-priority capability gain.
- Independent checking reveals an incorrect or incomplete result.
- The bottleneck moves into another equally expensive unavoidable stage with no workflow benefit.
- A necessary theorem/field assumption does not match current inputs.
- Measurements remain inconclusive after the predeclared campaign.
- A new prerequisite is a separate research problem; record it rather than burying it in a growing patch.

A deferred package must include a concrete next experiment and blocker. “Needs more work” is not a research result. No arbitrary calendar date can certify universal completion, and no negative experiment needs to remain in the default production path.

---

## 9. Required experiment records and command anchors

### 9.1 Reuse repository infrastructure before adding files

Place the program ledger and small manifests with existing admission/performance/workflow documentation. Reuse current test, benchmark, fuzz, size, and comparison facilities. Establish names and locations from the current tree; the example names below describe content, not a mandate for another framework.

Keep bulky raw evidence outside the source repository following `docs/evidence-archive.md` and existing archive practice. Check in small manifests, digests, reproduction commands, and findings. Respect current repository file/total-size limits. Do not add tens of megabytes of generated coefficient dumps or benchmark logs to each code commit.

### 9.2 Experiment manifest

Each candidate needs machine-readable fields equivalent to:

```yaml
schema_version: 1
experiment_id: P02.rational_span_view.001
state: planned
highest_claimed_improvement: code_size
base_sha: null
candidate_sha: null
patch_sha256: null

claim:
  hypothesis: null
  admitted_input_domain: null
  output_contract: null
  required_downstream_workflows: []
  proof_obligations: []
  changed_owner: null
  current_implementation_overlap: null
  previous_rejection_or_changed_premise: null

environment:
  toolchain: null
  lockfile_sha256: null
  targets: []
  features: []
  build_and_link_flags: []
  machine_and_os: null
  allocator: null
  corpus_sha256: null
  oracle_versions: []
  seeds: []

admission:
  baseline_certified_success_ids: []
  baseline_qualified_contract_ids: []
  protected_metrics_and_platforms: []
  performance_estimand: null
  confidence_level: null
  simultaneous_comparison_method: null
  sampling_and_stopping_rule: null
  allowed_higher_priority_regression: 0
  hard_resource_and_compatibility_contracts: []

evidence:
  raw_outcome_records: []
  independent_checker_records: []
  performance_samples: []
  binary_and_source_reports: []
  resource_reports: []
  interaction_reports: []
  artifact_uris_and_sha256: []

decision:
  disposition: null
  verified_highest_improvement: null
  higher_priority_gates: []
  lower_priority_costs: []
  adverse_results: []
  remaining_limitations: []
  removed_or_retained_fallbacks: []
  migration_notes: []
  reason: null
```

This is an unexecuted template. Populate real values; do not turn `null` into an implied pass. Allow the verified improvement to differ from the initial hypothesis only with a clear new claim and appropriate gates.

### 9.3 Per-case records

Each capability row should identify:

```text
case_id, revision, operation_chain, input_digest,
curve/support/parameter/point_tags, coefficient_domain,
degeneracy_class, chart_and_transform, policy, resources,
expected_domain_and_semantics,
raw_api_outcome, claimed_certainty_and_fidelity,
evaluator_classification, oracle_outcome,
root_event_exhaustiveness, topology, source_identity,
downstream_completion,
wall_and_cpu_phases, maximum_degree_and_coefficient_bits,
allocation_and_retention_diagnostics,
dispatch_trace_digest, evidence_digest, blocker_owner
```

Keep performance samples in separate raw records with process/build identifiers, randomized pair order, cold/warm state, and observation/materialization method. Do not store only medians.

### 9.4 Human admission report

For each package, provide:

1. Problem, exact contract, and why existing code did not already solve it.
2. Hypothesis, alternatives tried, changed owner, and source/algorithm applicability.
3. Independent proof/checker results, including negative controls and exhaustiveness.
4. Old/new outcome table and exact newly completed or repaired workflows.
5. Performance/resource/binary/source comparisons with uncertainty and raw-evidence digests.
6. Higher-priority nonregression, compatibility, and downstream/interaction evidence.
7. Kept/rejected/blocked decision; adverse cases; necessary remaining fallbacks; deleted paths.
8. Remaining limitations and the next smallest useful experiment.

An updated ledger is not a substitute for running the experiment or implementing the admitted capability.

### 9.5 Current command anchors

Read the current manifests and workflows before executing. These commands are anchors observed in the reviewed tree, not a complete immutable CI specification or a claim that they were run.

From `crates/hypercurve`, the reviewed workflow includes:

```sh
cargo fmt --check
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo test --release --locked --all-features --no-fail-fast -- --test-threads=2
```

From the repository root, useful targeted anchors include:

```sh
cargo test --locked -p hypercurve --all-features
cargo bench --locked -p hypercurve --bench curve_region_boolean_batch
cargo bench --locked -p hypercurve --bench bezier_split_materialization
cargo bench --locked -p hypercurve --bench bezier_tangent_order
cargo bench --locked -p hypercurve --bench offset
cargo bench --locked -p hypercurve --bench editing
cargo bench --locked -p hypercurve --features comparative-benchmarks --bench comparative
cargo test --locked --test successive_operations
cargo test --locked --release --test successive_operations -- --ignored
cargo bench --locked --bench successive_operations
```

Use filters and contained workers for individual long ignored cases before the full campaign. Enumerate current test/benchmark names rather than assuming every historical name survives.

From `crates/comparisons`, after qualifying its external dependencies:

```sh
cargo test --locked --all-features -- --test-threads=1
```

From `benchmarks/size-harness`, with its specified lock/toolchain:

```sh
./measure.sh all
./measure.sh perf
```

For changes below Hypercurve, run the affected owning crate and downstream tests, not only Hypercurve. Enumerate current feature/target/binding and locally executable workflow gates as required by `AGENTS.md`. Pre-push checks are not replaced by this abbreviated list. Tool or platform unavailability means an unverified gate, not a passing result.

Keep long campaigns monitored, interruptible, and progress-reporting. A short probe establishes a lead; it does not establish a completed benchmark or fuzz campaign.

### 9.6 First CLI work session

1. Read this entire document, current repository instructions, and relevant ledgers. Record the user-priority override.
2. Inspect the actual branch, dirty state, revision, toolchain, and available targets without disturbing existing work.
3. Create the P00 manifest and reproduce a small representative exact workflow in every currently relevant family.
4. Reproduce the three initial targets in Section 8.1, classifying already satisfied or changed failures honestly.
5. Qualify independent oracles for those targets; add missing wrong-answer controls.
6. Capture source/dispatch and runtime baselines for the P02 pilot; exact outcome/evidence baselines for P08/P14.
7. Choose the smallest ready implementation, create its experiment record, and proceed through Section 8.4.
8. Continue the queue using dependencies and measured bottlenecks. Avoid repeatedly asking for routine implementation decisions already covered by this program.

---

## 10. Completion, release, and stopping criteria

### 10.1 A work package is evaluated when

Its current premise has been checked, its prototype or explicit inapplicability has been tested, independent obligations have been assessed, and a disposition with raw evidence exists. A rejected idea is evaluated. An unimplemented idea merely copied to a future-work list is not.

A blocked experiment can be a legitimate research outcome if its exact missing theorem, oracle, platform, workload, or dependency is recorded. It does not count as completed production capability.

### 10.2 A production change is complete when

- Its claimed improvement is verified and all higher priorities pass the scoped admission gates.
- Every affected supported output can be reused by the declared next operations.
- Exactness, topology, source identity, qualification, and failure semantics survive relevant copy/trim/reverse/transform/replay/cache sequences.
- Required lower-layer, feature, target, binding, downstream, and package checks pass or the change remains explicitly unadmitted.
- Superseded code and temporary experiment machinery are removed or justified as necessary fallbacks.
- Documentation and capability tables describe the actual domain and resources, including limitations.
- Its interaction with the accepted stack passes P29.

### 10.3 Tier completion must be a scoped statement

A valid milestone statement has this form:

> At revision X, operations O on input class D, with listed degeneracies and resource policy B, return the declared certified or typed outcomes. Named workflows W and the documented holdout/coverage obligations pass. Remaining domains U are explicitly unsupported or undecided.

Provide both mathematical arguments for algorithm coverage and empirical evidence for practical operation. “All tests pass,” “all curve types share an enum,” “500,000 lines reduced,” and “24 workloads finish” are not substitutes for that declaration.

Keep exactness defects at zero among verified cases, preserve the cumulative strict/qualified success sets, and show which declared cells and workflows were added. Report the limits of proof and testing without claiming universal arbitrary-real decidability or unbounded composition performance.

### 10.4 Final program report

Produce a concise decision table for all P00–P29 packages, links/digests to evidence, admitted commits, rejected alternatives and reasons, domain/composition gains, performance distributions, deployed binary/source totals, resource limitations, and unresolved research dependencies.

Separate:

- Architecture that became simpler.
- Capabilities that became complete within a declared domain.
- Performance that improved under matched measurement.
- Footprint that shrank without higher-priority regressions.
- Essential complexity that remained because removing it failed the priority test.

The desirable endpoint is not one representation or the smallest possible codebase. It is one clear authority for each mathematical fact, explicit specialized backends where necessary, a larger useful certified domain, and enough evidence to know which complexity is earning its place.

---

## 11. Source map and primary research

### 11.1 Repository sources

All repository links below are pinned to the inspected revision. The implementing agent must inspect the current equivalents, reconcile differences, and record the execution revision. Local source inspection, rather than benchmark execution, supports the current-architecture observations in this document.

| ID | Source | Use |
|---|---|---|
| S1 | [AGENTS.md](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/AGENTS.md), [properties.md](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/properties.md), [admission](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/docs/admission.md) | Repository contracts and gates; user's five-rung priority overrides the differing local ladder |
| S2 | [Hypercurve FUTURE_WORK](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/FUTURE_WORK.md), [PERFORMANCE](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/PERFORMANCE.md) | S5/S6/S7, composition leads, rejected/deferred experiments; historical claims require reproduction |
| S3 | [Curve carrier](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve.rs), [source spans and trims](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_subdivision.rs), [support](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_support.rs) | Common semantic model and parameter/reversal hazards |
| S4 | [Selected scalar](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/selected_scalar.rs), [split/parameter families](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/bezier_split.rs), [curve points](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_point.rs) | Existing selected authority and essential point correlation |
| S5 | [Support relations](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_support_intersection.rs), [region Booleans](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_region_boolean.rs), [fillet centers](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_fillet_centers.rs) | Shared relation opportunity and existing authoritative arrangement |
| S6 | [Box containment](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_box_containment.rs), [retained measures](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/bezier_retained_measure.rs), [Bézier metrics](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/bezier_metric.rs) | Conservative bounds versus exact query/measurement contracts |
| S7 | [Native image](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_native_image.rs), [native segment](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/curve_native_segment.rs), [policy](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypercurve/src/policy.rs) | Optional views, injectivity/traversal and qualification |
| S8 | [Hypersolve FUTURE_WORK](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypersolve/FUTURE_WORK.md), [represented roots](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypersolve/src/represented_root.rs), [work budget](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hypersolve/src/work_budget.rs) | Local comparison, polynomial completion and accounting scope |
| S9 | [Hyperreal README](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hyperreal/README.md), [Hyperreal FUTURE_WORK](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hyperreal/FUTURE_WORK.md), [Hyperlimit README](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hyperlimit/README.md) | Scalar/predicate ownership, unknown semantics and lifecycle research |
| S10 | [Workflow corpus](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/docs/workflow-corpus.md), [successive operations](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/docs/successive-operations.md), [evidence archive](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/docs/evidence-archive.md) | Existing campaigns, historical limitations and evidence storage |
| S11 | [Comparison harness](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/comparisons/README.md), [size harness](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/benchmarks/size-harness/README.md), [Hypercurve workflow](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/.github/workflows/hypercurve.yml) | Reuse existing measurement and release gates |
| S12 | [Hyperpath CAM](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hyperpath/src/cam.rs), [rest-material planning](https://github.com/timschmidt/csgrs/blob/3cca6a894858c8b98497a3cfe81cff5e17e1a7f2/crates/hyperpath/src/cam/rest.rs) | Downstream consumer boundary; geometry completion is not full machine planning |

File names in work packages are navigation anchors into these subsystems. The implementation record must cite exact current symbols/lines; avoid treating old line numbers or comments as authoritative over current source.

### 11.2 Primary research and comparative methods

These references justify investigations, not claims of automatic applicability or performance. Documentation was consulted on 25 October 2026; pin the actual version used in any experiment.

| ID | Primary source | Role and limit |
|---|---|---|
| R1 | [CGAL 2D Arrangements manual](https://doc.cgal.org/latest/Arrangement_on_surface_2/index.html) and [Bézier traits](https://doc.cgal.org/latest/Arrangement_on_surface_2/classCGAL_1_1Arr__Bezier__curve__traits__2.html), documentation rendered as 6.2.1 | Reference architecture for separating topology and geometric traits; possible oracle on qualified domains. Bézier traits here use polynomial curves with rational control coordinates and algebraic intersections. |
| R2 | Berberich, Emeliyanenko, Kobel, Sagraloff, [Exact Symbolic-Numeric Computation of Planar Algebraic Curves](https://arxiv.org/abs/1201.1548) | Certified isolation/topology research for planar algebraic curves. Read full algorithms and assumptions before using them for P19/P28; arbitrary constructive coefficients are outside the general claim. |
| R3 | Shewchuk, [Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates](https://www.cs.cmu.edu/~quake/robust.html) | Adaptive exact predicates under specified input/arithmetic assumptions; a model for rigorous filters, not arbitrary-real construction/equality. |
| R4 | Boehm, [Towards an API for the Real Numbers](https://research.google/pubs/towards-an-api-for-the-real-numbers/) | Partial decision procedures and recursive-real comparison limitations; guards against an impossible universal equality requirement. |
| R5 | Johansson, [Calcium exact real and complex arithmetic documentation](https://fredrikj.net/calcium/ca.html); [FLINT QQbar documentation](https://flintlib.org/doc/qqbar.html) | Candidate independent algebraic/symbolic oracle tiers. Calcium documents noncanonical field representations and Unknown. Direct QQbar documentation retrieval returned HTTP 403 during this review; qualify the installed release and its documentation before relying on it. |
| R6 | [CGAL Lazy_exact_nt documentation](https://doc.cgal.org/latest/Number_types/classCGAL_1_1Lazy__exact__nt.html) | Lazy exact computation with interval filtering as a design reference. Its guarantees do not automatically apply to Hypercurve's selected-source or constructive domains. |
| R7 | Levien, [Parallel curves of cubic Béziers](https://raphlinus.github.io/curves/2022/09/09/parallel-beziers.html) | Approximation/candidate-generation research; requires a separate certificate and must remain in the qualified finite-output lane. |
| R8 | Kalibera and Jones, [Rigorous Benchmarking in Reasonable Time](https://kar.kent.ac.uk/33611/45/p63-kaliber.pdf), ISMM 2013 | Motivation for accounting for iteration/process/build variation and effect-size uncertainty. The exact admission thresholds in this program are project rules, not prescriptions attributed to the paper. |
| R9 | [Clipper2 documentation](https://angusj.com/clipper2/Docs/Overview.htm), [Cavalier Contours repository](https://github.com/jbuckmccready/cavalier_contours) | Qualify finite-coordinate throughput and approximation comparisons; normalize domain and guarantees before comparing size or speed. |

For each adopted method, archive a short applicability note: version, theorem assumptions, coefficient and curve domain, degeneracies, input/output guarantee, independence of checker, licensing/build implications, fallback, prior experiment overlap, and decisive measurement. Read full primary material when an abstract or API description does not establish the required proof.
