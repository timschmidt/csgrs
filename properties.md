# The Hyper Stack: Striven-For Properties

A precise statement of the properties the Hyper geometry stack strives for and the acceptance contract for changes. Each property names a requirement and a criterion for checking it; the normative evidence protocol is in [`docs/admission.md`](docs/admission.md). These requirements are not a claim that the implementation already satisfies them. Ordered by the governing preference ladder.

> A result is not complete until the next supported operation can consume it without losing mathematical meaning, evidence, or control of resources.

## 0. Governing ladder

**Exactness &gt; completeness &gt; performance &gt; memory &gt; binary size &gt; source size.**

A change may never improve a lower rung by demoting a higher one within the declared input domain and resource contract. Silently changing answers, reducing supported inputs, discarding evidence, or weakening policy is not an optimization. A deliberate contract change requires an explicit migration. Performance-only work must preserve the declared exact-output and deterministic-artifact contracts, including exact rows, output size, and checksums; memory work must not introduce approximation; size work must not change answers.

Every construction and conversion must distinguish three independent claims:

| Contract | Question |
| --- | --- |
| Numerical certainty | Were the decisions justified under the selected policy? |
| Topological validity | Does this representation satisfy the required topology? |
| Geometric fidelity | How does this representation relate to the intended source? |

A certified predicate does not establish manifoldness; a valid closed mesh does not establish fidelity to an analytic surface; certification of a sampled field does not certify the underlying continuous field. Test each promised claim separately. The closure requirements P17–P20 apply across supported modeling operations, not only within the curve kernel.

---

## 1. Scalar-layer properties (`hyperreal`)

**P1. Exactness by construction.** Exact rational values stay exact until an API *explicitly* requests approximation or float conversion. No hidden binary64 demotion anywhere in the core. *Criterion:* no code path demotes a scalar except at named boundary APIs.

**P2. Structure preservation.** Recognizable symbolic forms — π, e, roots, logs, trig endpoints, removable small-angle limits, quadratic surds `a + b√d` — stay symbolic while that avoids precision loss or repeated approximation. *Criterion:* an identity provable from the certificate must not require refinement; `hypot` of 3-4-5 stays exact.

**P3. Conservative, honest facts.** Structural facts are cheap-or-Unknown: "Unknown" means "not cheaply proven," never "false." Equality is structural, not theorem-proving, and says so. *Criterion:* no fact API returns a definite answer without a certificate behind it.

**P4. Monotone refinement caches.** Cached approximations and facts must preserve mathematical semantics and never weaken a later request, at any precision. Warming may strengthen Unknown into Certified and reuse prior proofs, including proofs obtained at higher precision; it must never introduce contradictions or weaken an available certificate. Reuse accounts for geometry identity, mutation, policy admissibility, precision, and proof scope; an approximate fact is never a strict proof. Exhaustion is not permanent evidence of undecidability. *Criterion:* cold, warm, evicted, disabled, mutated, and concurrent states, where applicable, preserve value and error contracts and agree on definite facts, including strict→approximate→strict and low→high→low precision sequences. Execution paths and failure histories need not be identical.

**P5. Bounded explosion.** Growth is prevented at the earliest layer with enough information: dyadic recognition, shared-denominator reducers, product-sum shapes, perfect-root collapsing. `Computable` is the fallback, never the default container. Input-controlled work has an explicit resource contract: precision, total work, retained memory, and cancellation are distinct. *Criterion:* repeated operand chains do not force GCD work per term or build avoidable expression trees; adversarial compositions exercise typed exhaustion, cleanup, and recovery. Missing internal bounds are disclosed and dangerous tests use external containment, not an unsupported claim of bounded execution.

**P6. Typed failure, never silent wrongness.** All known scalar domain errors return `Problem`; `expect` only for invariants proven by construction; invalid float imports (NaN/Inf) are rejected at conversion. Across the stack, successful empty geometry, invalid input, unsupported representation, undecided predicate, exhausted budget, and unbounded result are distinct outcomes. *Criterion:* ordinary domain failures neither panic nor become empty meshes, zero coordinates, false classifications, unchanged inputs, or incomplete successes; typed diagnostics survive the public API, wrappers, and bindings.

**P7. Stable forms built in.** Cancellation-prone compositions (`ln_1p`, `expm1`, `diff_of_products`, `sqrt1pm1`, `logsubexp`, `hypot2/3`...) are public primitives so callers cannot spell them unstably. *Criterion:* geometry code never hand-rolls a numerically unstable composition the scalar layer already owns.

---

## 2. Predicate-layer properties (`hyperlimit`)

**P8. Policy-explicit decisions.** Every predicate-bearing operation receives an explicit policy (`STRICT`, `APPROXIMATE_512`); no internal default changes predicate semantics. *Criterion:* every consumed fact is admissible under the selected policy and proof scope. A reusable strict proof may satisfy multiple policies; P4 permits a stronger cached proof to change the decision path without changing meaning.

**P9. Certainty receipts.** Results carry `Certified` vs. `Approximate512Consumed`, bound to the claim and the evidence actually consumed. Selecting an approximate policy does not itself make an answer approximate. Keep operation-local certainty separate from end-to-end provenance: a later strict operation does not certify the intent of an earlier approximate construction. *Criterion:* a consumer can distinguish local proof from inherited qualification, and upgrading a claim requires a new proof of that claim, never a wrapper conversion, cache hit alone, serialization step, or discarded outcome object.

**P10. Least-cost proof search.** The decision cascade (structural shortcut → rational reducer → bounded refinement → deterministic terminal) finds the cheapest sufficient certificate, JIT-like for repeated queries. *Criterion:* repeated queries amortize; prepared objects beat one-shot paths on measured benches.

**P11. Never wrong, only unsure.** Where a decision cannot be certified in budget, return Unknown — never a guess. *Criterion:* adversarial corpus contains zero false-positive/false-negative predicate answers at any policy.

---

## 3. Curve-layer properties (`hypercurve`)

**P12. One region carrier, many families.** `CurveRegion2` is the sole filled-region carrier; line/arc specializations are private fast paths inside the unified kernel, never parallel semantics. *Criterion:* a mixed line/arc/Bézier/NURBS boundary classifies consistently with the pure-line equivalent.

**P13. Certified interior from winding.** Interior sides are proven by arrangement from winding — authoring requires no orientation hints. Material/hole loops, nested islands, and fill rules (`NonZero`, `EvenOdd`) publish the regularized set. *Criterion:* same boundary with reversed author orientation yields the same region.

**P14. Uncertainty is a typed value.** `Classification<T>` (decided/uncertain), `Provisional<T>`, `evaluate_under`: approximate evaluation is explicitly qualified and certainty-carrying. *Criterion:* no approximate answer is ever unlabelled or mislabeled as exact.

**P15. Boundary provenance.** Curve operations emit evidence — retained roots, contact records, contour roles — not just coordinates. *Criterion:* every output boundary element can name the input elements that produced it.

**P16. Lossless promotion, typed blockers.** Exact line/arc promotes losslessly to downstream consumers; anything uncertifiable (e.g. a general cubic) fails with a typed blocker rather than borrowing lossy display chords — unless a distinct bounded certificate admits it. *Criterion:* no display approximation ever feeds an exact downstream decision.

**P17. Semantic closure.** Every supported exact operation returns the intended exact regularized set, with consistent winding, boundary ownership, tangency, overlap, and degeneracy semantics. Policy-qualified outcomes retain their declared uncertainty and do not replace a required exact success. *Criterion:* valid-set identities and independently derived expectations establish equality of represented sets, not just triangle counts, checksums, or agreement between wrappers of one kernel. Preconditions and the equality criterion are explicit.

**P18. Representational closure.** The output of a supported operation remains valid input to the next supported operation whenever the mathematical composition is defined, including empty results, disconnected components, holes, symbolic coordinates, and qualified uncertainty. The public modeling layer bridges documented kernel restrictions without changing meaning. *Criterion:* exercise empty-set identities, union accumulation from empty, and subtraction chains that remove everything, then consume their results in further operations. Canonical empty geometry remains distinct from malformed operands; identity shortcuts must not conceal invalid input.

**P19. Evidentiary closure.** Roots, selected fibers, parameters, signs, and topology decisions retain enough identity and proof information to be reused or replayed. Every claim carries the certainty of its dependencies. *Criterion:* approximate construction → transform → Boolean → export → import → validation never silently promotes the original claim to certified. A proof about a reimported quantized mesh is distinct from preservation of the original exact geometry.

**P20. Computational closure.** A mathematically valid output must remain consumable under the declared resource contract through repeated operations. Precision limits do not bound total computation or expression growth; cache entry limits do not bound heap bytes. Exhaustion is typed before publishing a complete result; useful partial results have a separate provisional contract. *Criterion:* small-budget compositions test exhaustion, cancellation, retained-state cleanup, and recovery, with expression depth, coefficient growth, allocation, and peak-memory evidence where affected.

---

## 4. Mesh-layer properties (`hypermesh`)

**P21. One exact arrangement.** Booleans validate closed PWN input, corefine shared faces, build one radial cell complex, propagate absolute windings, and select closure-certified boundaries — one `boolean()` entry point, shared intersections across expression roots. *Criterion:* additional expression roots do not repeat arrangement work.

**P22. Broad input contract, typed rejections.** Disconnected, nested, coincident, balanced-nonmanifold, and winding-multiplicity inputs are supported; open meshes, invalid indices, and degenerate triangles are typed failures — never hangs, never wrong output, never unbounded allocation. *Criterion:* adversarial Boolean matrix (tangent, coplanar, near-coplanar, sliver, inverted, duplicate, invalid) all decide or diagnose.

**P23. Unbounded sets never masquerade as finite.** Negation/exterior-containing expressions return `UnboundedBooleanOutput` rather than a finite mesh that misrepresents the result. *Criterion:* no finite output is ever emitted for an exterior-inside result.

**P24. Tolerance-free coincidence.** Near-coincident-but-distinct geometry stays distinct; there is no epsilon merge anywhere. *Criterion:* a point at 1e-12 outside a cube is outside; a 1e-12 translation is preserved in bounds.

---

## 5. Grammar-layer properties (`csgrs`)

**P25. Vocabulary, not a second kernel.** CSGRS composes native Hyper types — it never introduces another mesh, region, or polygon carrier. Enforce each invariant in its lowest owning layer and share that implementation across public entry points; modeling composition belongs here, scalar identities and predicate proofs in their respective layers, topology in the mesh kernel, and quantization at format boundaries. The primary modeling interface preserves typed failure; deliberately lossy convenience interfaces require explicit, intentional use. *Criterion:* adapters bridge documented representation restrictions without duplicated formulas, caller-specific tolerances, a hidden second kernel, or forcing applications to reconstruct discarded evidence.

**P26. Intent-preserving interchange.** IO and scalar adapters are lossy only at their documented boundaries; floats appear only at explicit rendering, serialization, adapter, FFI, and wasm edges. Lossless interchange preserves representable values. Lossy interchange exposes its loss model and validates any topology or error bound it promises. *Criterion:* format-specific round trips distinguish numerical certainty, topology, and fidelity to the source; reimport cannot certify preservation of exact source geometry merely by validating the quantized mesh.

**P27. Deterministic, checksum-auditable outputs.** Repeated serialized runs are deterministic for geometry and checksums; performance-only changes must preserve the declared rows, output size, and checksum contracts to be retained. Intentional representation or serialization changes require a separately identified migration. *Criterion:* every retained optimization in PERFORMANCE.md cites its preserved evidence; semantic equality is tested separately from deterministic serialization.

---

## 6. Boundary properties (FFI, wasm, JS, display)

**P28. FFI cannot violate the contract.** Null-checked, family-checked handles; panics caught at the boundary (`Panic = 255`); thread-local last-error with documented lifetime; matched `*_free` ownership; no UB on misuse. *Criterion:* hostile C/JS input (incomplete triples, out-of-range indices, non-finite coordinates) is rejected before native geometry is created.

**P29. Display is a projection, never a source.** Exact source, canonical machine, and lossy display values live in separate domains; GPU/export boundaries are the only f32/f64 demotion points and own their own checks. *Criterion:* changing display fidelity can never change a classified or computed result.

**P30. The certainty receipt survives every boundary.** `Certified`/`Uncertain`/`Unknown`, typed failures, and the provenance needed to interpret each claim remain queryable at each ABI — including wasm/JS. *Criterion:* real binding consumers can distinguish successful empty geometry from failure and inspect certainty through a downstream pipeline. Any boundary that currently flattens this evidence remains an open target.

---

## 7. Motion/machine-layer properties (alumina)

**P31. Certified end-to-end demotion.** Every float-shaped artifact in the chain (chord approximation, step rounding, timer rounding, time dilation) has an explicit position-error budget and a proof that the budget holds. *Criterion:* from sketch to microstep, every demotion either certifies or fails typed.

**P32. Fail-closed safety.** The safety state machine is fail-closed; bounded channels; one core for real-time work; firmware consumes only bounded, independently admitted machine IR. *Criterion:* no audit/evidence data path enters the real-time core.

**P33. Replayable canonical evidence.** Machine jobs are content-addressed, replay-verified against retained exact carriers, and their planning/lowering evidence is reconstructible from live state. *Criterion:* the same job always replays identically; policy changes change evidence even when bytes do not.

**P34. Lossless community inheritance.** OpenSCAD numerics parse directly into `Real` (no f64 bottleneck); compatibility is defined as *intent-preserving*, and diffs against upstream on the artifact corpus are triaged and published, not silently absorbed. *Criterion:* the differential harness runs the corpus through both engines and classifies every difference.

---

## 8. Process properties (how the properties are kept true)

**P35. Failure promotion.** Every minimized, license-clean fuzz/adversarial failure becomes a deterministic regression fixture alongside the original full-scale hard case. *Criterion:* no passing result is obtained by weakening policies, accepting a new Unknown, removing difficult geometry, or reducing fixture resolution; an intentional contract change is explicitly recorded as a migration, not a fix or optimization.

**P36. Evidence-gated retention.** A change is retained only after focused correctness tests plus proportionate benchmark/allocation/size evidence for the same completed contract. A faster error, lower resolution, missing provenance, weaker certainty, skipped validation, or omitted output materialization changes the workload. *Criterion:* ledgers name kept, rejected, already-satisfied, or architecture-inapplicable for every idea; retained source/dependency/toolchain/hardware/input/policy/cache/timing records and raw samples report completion and certainty beside cost. Keep unfavorable results and supported-corpus regressions, not only favorable isolated measurements. Retention has a budget: raw samples and other bulky records live in the evidence repository, and this repository keeps their digests under the [retention budget](docs/admission.md#retention-budget).

**P37. No overclaiming.** Completion claims state their coverage precisely; inventory is not credited as reading; probes are not claimed as matched benchmarks; a specification or ledger update does not establish implementation compliance. The shipped user path is the final acceptance boundary. *Criterion:* every public claim names its evidence artifact; a clean external project exercises the documented package through construction, transform, composition, certainty inspection, required topology validation, export, reimport, and format-specific checks. Releases, dependencies, examples, API docs, features, minimum supported compiler, and affected bindings describe one coherent system without hidden workspace patches, local paths, undeclared assets, or development-only configuration.

**P38. Selection-pressure verification.** Fuzzing is directed by lexicase worst-performer selection across the stack, with asan, path tracing, and competitive benches as gates. *Criterion:* the adversarial campaign demonstrably hunts its own weakest cases.

**P39. Legible frontier.** Capability is measured by completed, validated workflows in a named corpus: drilled bodies, transformed primitives, profile extrusion, holes, repeated cuts, touching and coplanar surfaces, and supported symbolic representations. Record certified completion, qualified approximate completion, undecided, unsupported, invalid input, resource exhaustion, and incorrect result separately. The strict supported domain and open problems are explicit and queryable; [`docs/admission.md`](docs/admission.md) defines how extensions earn acceptance. *Criterion:* preserve the certified-success corpus while resolving unsupported or undecided cases and add regressions for expanded capability. Counts of constructors, tests, and API entries do not establish completeness. Chains that feed results back into further operations are measured in [`docs/successive-operations.md`](docs/successive-operations.md).

---

## 9. Known open targets (from the review)

Implement the review gaps in this order: **empty-result composition and visible failures first; ordinary strict-mode completion next; complete user-path and release validation next**. Reproduce each finding on the target revision before changing code. Performance work proceeds against the preserved contracts and cannot substitute for these milestones.

For every patch, ask: **does it let users complete more trustworthy work, or complete the same trustworthy work at lower cost?**

1. **Cache policy as contract** - `solid.rs` thread-local caches should acquire the same explicitness as the scalar caches: caller-visible budget/semantics (P4 applied at the grammar layer).
2. **JS certainty surface** - audit the target revision for flattened `MeshCertainty`/`Classification` and expose any missing receipt and provenance through real consumers (P9/P19/P30).
3. **Refinement budget for curves** - make "undecidable in bounded time" an explicit, documented per-operation budget (the `APPROXIMATE_512` analog for curve queries) rather than emergent behavior (P11/P14).
4. **One normative admission spec** - keep [`docs/admission.md`](docs/admission.md) aligned with P1–P39 and apply its acceptance evidence to each change; documenting the protocol does not discharge implementation gaps.
5. **Industry standard terminology** - wherever possible the codebase should favor unambiguous industry standard naming and terminology.  Names should be regularized across the stack such that functions and types are consistent and can be referenced directly.
6. **Exhaustive testing** - 100% coverage with unit tests including adversarial inputs and all possible representations, benchmarks, competitive library benchmarks, asan, fuzzing including all representation types, and lexicase worst performers with independent buckets for each representation type, https://github.com/timschmidt/exactcore-hyper-comparison integrated under crates/comparisons with consolidated and re-validated logs and organized and complete code for all claims for reference in a future paper.  Verus coverage for hyperreal, hyperlattice, and hyperlimit.
7. **Idiomatic bindings** - For C / C++, Python, Javascript / Typescript via WASM
8. **CI checks** - should exercise as much testing as possible, deploy demos and a static site for the project as a whole.
9. **Documentation** - should synthesize the best aspects of current crates documentation, with hyperreal and csgrs being the flagship examples - the readme should explain what the crate does, why, and provide a quick entry into how along with an index of available types and functions which dive into deep educational-quality references in  rustdoc via good doc comments in the code and additional markdown documents in git where appropriate.
10. **Paper** - A scientific paper detailing important aspects of the project suitable for publication and education of others should be prepared.
11. **Optimization** - Opportunities for reducing the amount of code and complexity throughout the stack while preserving or enhancing the exactness > completeness > performance > memory > binary > code priority ladder should be logged, prototyped, evaluated, considered in combination with previously evaluated and rejected ideas, and so on.  Only keeping changes which improve without significant regression.  APIs should be as narrow and thin as possible while retaining honesty and the priorities.  Consider the use of parallelism through fearless_simd and multithreading.
12. **Release** - All crates in the monorepo should be prepared for release, including the clean external consumer pipeline, documented feature configurations, minimum supported compiler, package dependencies, examples, and real affected bindings required by P37 and the admission protocol.
    *Status (2026-10-20):* [`docs/RELEASE.md`](docs/RELEASE.md) prepares 18 crates; hypervoxel, hyperbrep, hyperbrep-io and hyperspatial are deferred. `scripts/release-check.sh` packages them, builds each package and its docs.rs documentation, then runs a consumer outside the repository against the unpacked packages. The consumer runs the csgrs user path, every README quick start, and C, C++, Python, Go and TypeScript binding checks, on stable and on the declared 1.95.0. Every crate declares that compiler, a semver-checked version and a changelog. Preparing this found and fixed a representation-dependent float conversion in hyperreal and two dependency advisories. Every example now completes under `STRICT` (target 15), and the [successive-operations corpus](docs/successive-operations.md) stays exact and certified; its growth and cost limits are listed under known limitations in `docs/RELEASE.md`. Still open: publishing `hypersdf`, `csgrs`, `csgrs-ffi`, `hyperdrc` and `hypercircuit` waits on `hypervoxel` and its vendored `voxelis` reaching crates.io, and publication and the docs.rs builds stay unverified until the crates are published.
13. **File organization** - Files should be organized, sized, and split / combined semantically such that finding relevant code is easy and no file is too long for reading.
14. **Future work** - each crate may have documentation discussing crate-specific future work which is also in scope.

### Review of 2026-10-20

An external review of the merged stack added these targets. Each was
reproduced on the merge revision. They sit inside the order above: the first
is ordinary strict-mode completion, the second milestone.

15. **Strict completion of the shipped examples** - Strict mode refused the
    repository's own examples; all of them now complete under `STRICT`, and
    `scripts/run-examples.sh` has no known failures. `boolean_operations` and
    `convex_hull` stopped on exact zeros of the 32×16 sampled sphere:
    collinear plane crossings of one quad, and four coplanar vertices that are
    not a quad (`cot(pi/16) = tan(7 pi/16)`). `hyperreal`'s cyclotomic zero
    proof now decides them;
    [`crates/hypermesh/FUTURE_WORK.md`](crates/hypermesh/FUTURE_WORK.md#convex-hulls-of-sampled-spheres)
    records the causes, the remaining cost and the hull row instability.
    `extrude` stopped because `try_sweep` reported an undecided frame-transport
    rotation as `InvalidArguments`, which also violated P6. The rotation was
    valid: its axis only lacked a structural nonzero certificate.
    `Matrix4::rotation_between_unit_vectors` now builds that minimal rotation
    algebraically, without normalizing the axis. The sweep decides path
    reversal and the initial axis alignment under the policy. It reports
    `PredicateUndecided` naming the sweep step, or the scalar cause, instead
    of `InvalidArguments`. A sweep with rational frames composes with a strict
    Boolean ([`tests/sweep_frames.rs`](tests/sweep_frames.rs)). Still open:
    the exact frames of a general sampled path lie in a tower of nested square
    roots. A strict Boolean on such a tube meets an exact zero it cannot prove;
    for the example's helix it ends `PredicateUndecided` after 37 to 48 s in
    release builds.
16. **Evidence retention** - Resolved in tree. Generated evidence (14 GB in
    552,370 files) moved to the evidence repository with per-record digests
    ([`docs/evidence-archive.md`](docs/evidence-archive.md)) and was removed
    from history, with a commit map for old IDs. P36 now has a
    retention budget, and CI enforces it with
    `scripts/check-repository-budget.sh`.
17. **Dependency direction** - `hyperdrc` (PCB design-rule checks) and
    `hypercircuit` (circuit simulation) are applications that depend upward
    on `csgrs`. Move them to an application area outside the core release
    graph, so no core crate's workspace build or release depends on them.
18. **Reviewable size** - The stack has about 1.43 million lines of
    first-party Rust across 22 crates, excluding vendored code and the
    comparison harness. `hypercurve` alone has 476k lines. That includes the
    58k-line `crates/hypercurve/src/bezier_offset/conversion_tests.rs` and
    identifiers such as
    `projected_selected_trivariate_candidate_has_subresultant_root_with_resultant_limit`.
    The month to 2026-10-20 had 1,367 commits; 246 of them begin with
    "Record" or "Freeze". At that rate no person can review the code. Add a
    per-file line budget enforced in CI (with target 13), an identifier
    naming rule (with target 5), and change sets small enough to review.
    Ledger-only commits do not advance a target (P37).
19. **One owner per core type** - `Point2` is defined in `hypercurve`,
    `hyperlattice` and `hypersolve`. `Aabb` is in `hyperlattice` and
    `hypermesh`, `Sign` in `hyperlimit` and `hypertri`, and `Classification`
    twice in `hypersolve` and once in `hypermesh`. This contradicts target 5.
    Keep each concept in its lowest sufficient layer and re-export it
    elsewhere, with explicit migrations.
20. **API surface** - 110 public `*_with_context` functions pair with
    context-free twins, doubling the surface. Choose one convention, such as
    one entry point taking the context plus a documented convenience layer,
    and migrate to it.
21. **README as a migration log** - Resolved: the README describes current
    behavior, and its accumulated migration and serialization-change notes
    moved to [`docs/MIGRATIONS.md`](docs/MIGRATIONS.md). New migrations are
    recorded there. Restructuring the README as target 9 describes remains
    open.
22. **Pre-existing CI gates** - `hypersolve`'s production line coverage gate
    requires 85%. It measured 76.24% on `main` before the merge and 77.52%
    after. Closing it needs new tests, not a lower threshold. Separately,
    `hypercurve.yml` runs only on `main` and pull requests, and on `main` its
    tests were skipped because the Clippy step failed first. Regressions from
    both merged branches therefore reached `hypercurve` without any CI run of
    its tests. Run the dependent crates' suites on every branch that changes
    a lower layer.
23. **HyperBREP generality** - Admission is by shape-family recognizers, and
    surface intersection is a pairwise table. See the findings and acceptance
    gates in
    [`crates/hyperbrep/docs/DEVELOPMENT_PLAN.md`](crates/hyperbrep/docs/DEVELOPMENT_PLAN.md#review-findings-2026-10-20).
