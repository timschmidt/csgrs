# The Hyper Stack: Striven-For Properties

A precise statement of the properties the Hyper geometry stack strives for, written to make each one targetable: every property has a name, a definition, and a criterion that future work can be checked against. Ordered by the governing preference ladder.

## 0. Governing ladder

**Exactness &gt; completeness &gt; performance &gt; memory &gt; binary size &gt; source size.**

A change may never improve a lower rung by demoting a higher one. Performance work must preserve exact rows, output size, and checksums; memory work must not introduce approximation; size work must not change answers.

---

## 1. Scalar-layer properties (`hyperreal`)

**P1. Exactness by construction.** Exact rational values stay exact until an API *explicitly* requests approximation or float conversion. No hidden binary64 demotion anywhere in the core. *Criterion:* no code path demotes a scalar except at named boundary APIs.

**P2. Structure preservation.** Recognizable symbolic forms — π, e, roots, logs, trig endpoints, removable small-angle limits, quadratic surds `a + b√d` — stay symbolic while that avoids precision loss or repeated approximation. *Criterion:* an identity provable from the certificate must not require refinement; `hypot` of 3-4-5 stays exact.

**P3. Conservative, honest facts.** Structural facts are cheap-or-Unknown: "Unknown" means "not cheaply proven," never "false." Equality is structural, not theorem-proving, and says so. *Criterion:* no fact API returns a definite answer without a certificate behind it.

**P4. Monotone refinement caches.** Cached approximations and facts must preserve mathematical semantics and never weaken a later request, at any precision. Warming may strengthen Unknown into Certified and reuse prior proofs, including proofs obtained at higher precision; it must never introduce contradictions or weaken an available certificate. *Criterion:* cold and warm queries preserve the same value and error contracts; every definite fact agrees, and warming retains or strengthens all available facts.

**P5. Bounded explosion.** Growth is prevented at the earliest layer with enough information: dyadic recognition, shared-denominator reducers, product-sum shapes, perfect-root collapsing. `Computable` is the fallback, never the default container. *Criterion:* repeated operand chains do not force GCD work per term or build avoidable expression trees.

**P6. Typed failure, never silent wrongness.** All known domain errors return `Problem`; `expect` only for invariants proven by construction; invalid float imports (NaN/Inf) are rejected at conversion. *Criterion:* no public API can panic on ordinary numeric domain failure.

**P7. Stable forms built in.** Cancellation-prone compositions (`ln_1p`, `expm1`, `diff_of_products`, `sqrt1pm1`, `logsubexp`, `hypot2/3`...) are public primitives so callers cannot spell them unstably. *Criterion:* geometry code never hand-rolls a numerically unstable composition the scalar layer already owns.

---

## 2. Predicate-layer properties (`hyperlimit`)

**P8. Policy-explicit decisions.** Every predicate-bearing operation receives an explicit policy (`STRICT`, `APPROXIMATE_512`); no internal default changes predicate semantics. *Criterion:* the same query under the same policy always answers through the same decision stratum.

**P9. Certainty receipts.** Results carry `Certified` vs. `Approximate512Consumed` — the *cost class of the proof* is a return value. *Criterion:* a consumer can always distinguish exact from fallback-decided without inspecting internals.

**P10. Least-cost proof search.** The decision cascade (structural shortcut → rational reducer → bounded refinement → deterministic terminal) finds the cheapest sufficient certificate, JIT-like for repeated queries. *Criterion:* repeated queries amortize; prepared objects beat one-shot paths on measured benches.

**P11. Never wrong, only unsure.** Where a decision cannot be certified in budget, return Unknown — never a guess. *Criterion:* adversarial corpus contains zero false-positive/false-negative predicate answers at any policy.

---

## 3. Curve-layer properties (`hypercurve`)

**P12. One region carrier, many families.** `CurveRegion2` is the sole filled-region carrier; line/arc specializations are private fast paths inside the unified kernel, never parallel semantics. *Criterion:* a mixed line/arc/Bézier/NURBS boundary classifies consistently with the pure-line equivalent.

**P13. Certified interior from winding.** Interior sides are proven by arrangement from winding — authoring requires no orientation hints. Material/hole loops, nested islands, and fill rules (`NonZero`, `EvenOdd`) publish the regularized set. *Criterion:* same boundary with reversed author orientation yields the same region.

**P14. Uncertainty is a typed value.** `Classification<T>` (decided/uncertain), `Provisional<T>`, `evaluate_under`: approximate evaluation is explicitly qualified and certainty-carrying. *Criterion:* no approximate answer is ever unlabelled or mislabeled as exact.

**P15. Boundary provenance.** Curve operations emit evidence — retained roots, contact records, contour roles — not just coordinates. *Criterion:* every output boundary element can name the input elements that produced it.

**P16. Lossless promotion, typed blockers.** Exact line/arc promotes losslessly to downstream consumers; anything uncertifiable (e.g. a general cubic) fails with a typed blocker rather than borrowing lossy display chords — unless a distinct bounded certificate admits it. *Criterion:* no display approximation ever feeds an exact downstream decision.

**P17. Semantic closure: every supported operation returns the intended exact regularized set, with consistent winding, boundary ownership, tangency, overlap, and degeneracy semantics.

**P18. Representational closure: every exact result needed by the next operation remains representable.

**P19. Evidentiary closure: roots, selected fibers, parameters, signs, and topology decisions retain enough identity and proof information to be reused or replayed.

**P20. Computational closure: the output may be mathematically valid, but repeated operations must not cause uncontrolled expression depth, coefficient growth, redundant refinement, allocation traffic, or reconstruction cost
---

## 4. Mesh-layer properties (`hypermesh`)

**P21. One exact arrangement.** Booleans validate closed PWN input, corefine shared faces, build one radial cell complex, propagate absolute windings, and select closure-certified boundaries — one `boolean()` entry point, shared intersections across expression roots. *Criterion:* additional expression roots do not repeat arrangement work.

**P22. Broad input contract, typed rejections.** Disconnected, nested, coincident, balanced-nonmanifold, and winding-multiplicity inputs are supported; open meshes, invalid indices, and degenerate triangles are typed failures — never hangs, never wrong output, never unbounded allocation. *Criterion:* adversarial Boolean matrix (tangent, coplanar, near-coplanar, sliver, inverted, duplicate, invalid) all decide or diagnose.

**P23. Unbounded sets never masquerade as finite.** Negation/exterior-containing expressions return `UnboundedBooleanOutput` rather than a finite mesh that misrepresents the result. *Criterion:* no finite output is ever emitted for an exterior-inside result.

**P24. Tolerance-free coincidence.** Near-coincident-but-distinct geometry stays distinct; there is no epsilon merge anywhere. *Criterion:* a point at 1e-12 outside a cube is outside; a 1e-12 translation is preserved in bounds.

---

## 5. Grammar-layer properties (`csgrs`)

**P25. Vocabulary, not a second kernel.** CSGRS composes native Hyper types — it never introduces another mesh, region, or polygon carrier. *Criterion:* every public type is a re-export or adapter of a kernel type.

**P26. Intent-preserving interchange.** IO and scalar adapters are lossy only at their documented boundaries; floats appear only at explicit rendering, serialization, adapter, FFI, and wasm edges. *Criterion:* round-trips through every supported format preserve exact geometry where the format allows.

**P27. Deterministic, checksum-auditable outputs.** Repeated serialized runs are deterministic for geometry and checksums; performance changes must preserve rows, output size, and checksums to be retained. *Criterion:* every retained optimization in PERFORMANCE.md cites its preserved evidence.

---

## 6. Boundary properties (FFI, wasm, JS, display)

**P28. FFI cannot violate the contract.** Null-checked, family-checked handles; panics caught at the boundary (`Panic = 255`); thread-local last-error with documented lifetime; matched `*_free` ownership; no UB on misuse. *Criterion:* hostile C/JS input (incomplete triples, out-of-range indices, non-finite coordinates) is rejected before native geometry is created.

**P29. Display is a projection, never a source.** Exact source, canonical machine, and lossy display values live in separate domains; GPU/export boundaries are the only f32/f64 demotion points and own their own checks. *Criterion:* changing display fidelity can never change a classified or computed result.

**P30. The certainty receipt survives every boundary.** `Certified`/`Uncertain`/`Unknown` must remain queryable at each ABI — including wasm/JS. *Target:* any boundary that currently flattens certainty (e.g. JS) should surface it.

---

## 7. Motion/machine-layer properties (alumina)

**P31. Certified end-to-end demotion.** Every float-shaped artifact in the chain (chord approximation, step rounding, timer rounding, time dilation) has an explicit position-error budget and a proof that the budget holds. *Criterion:* from sketch to microstep, every demotion either certifies or fails typed.

**P32. Fail-closed safety.** The safety state machine is fail-closed; bounded channels; one core for real-time work; firmware consumes only bounded, independently admitted machine IR. *Criterion:* no audit/evidence data path enters the real-time core.

**P33. Replayable canonical evidence.** Machine jobs are content-addressed, replay-verified against retained exact carriers, and their planning/lowering evidence is reconstructible from live state. *Criterion:* the same job always replays identically; policy changes change evidence even when bytes do not.

**P34. Lossless community inheritance.** OpenSCAD numerics parse directly into `Real` (no f64 bottleneck); compatibility is defined as *intent-preserving*, and diffs against upstream on the artifact corpus are triaged and published, not silently absorbed. *Criterion:* the differential harness runs the corpus through both engines and classifies every difference.

---

## 8. Process properties (how the properties are kept true)

**P35. Failure promotion.** Every minimized, license-clean fuzz/adversarial failure becomes a deterministic regression fixture; full-resolution hard cases are never replaced with reduced proxies. *Criterion:* no fixture shrinks without explicit justification.

**P36. Evidence-gated retention.** A change is retained only after focused correctness tests plus proportionate benchmark/allocation/size evidence; rejected experiments are recorded with decisive data. *Criterion:* the ledgers name kept, rejected, already-satisfied, or architecture-inapplicable for every idea.

**P37. No overclaiming.** Completion claims state their coverage precisely; inventory is not credited as reading; probes are not claimed as matched benchmarks. *Criterion:* every public claim names its evidence artifact.

**P38. Selection-pressure verification.** Fuzzing is directed by lexicase worst-performer selection across the stack, with asan, path tracing, and competitive benches as gates. *Criterion:* the adversarial campaign demonstrably hunts its own weakest cases.

**P39. Legible frontier.** The completeness boundary is typed (Unknown, blockers, receipts), the admission protocol for new certificates/families is written in one normative place, and the open-problem list is a published, queryable corpus. *Target:* extensions convert Unknowns to Certified; nothing else changes.

---

## 9. Known open targets (from the review)

1. **Cache policy as contract** - `solid.rs` thread-local caches should acquire the same explicitness as the scalar caches: caller-visible budget/semantics (P4 applied at the grammar layer).
2. **JS certainty surface** - wasm boundary currently flattens `MeshCertainty`/`Classification`; expose it (P26).
3. **Refinement budget for curves** - make "undecidable in bounded time" an explicit, documented per-operation budget (the `APPROXIMATE_512` analog for curve queries) rather than emergent behavior (P11/P14).
4. **One normative admission spec** - consolidate the distributed invariants (P1–P35) into the single documented protocol future extenders program against.
5. **Industry standard terminology** - wherever possible the codebase should favor unambiguous industry standard naming and terminology.  Names should be regularized across the stack such that functions and types are consistent and can be referenced directly.
6. **Exhaustive testing** - 100% coverage with unit tests including adversarial inputs and all possible representations, benchmarks, competitive library benchmarks, asan, fuzzing including all representation types, and lexicase worst performers with independent buckets for each representation type, https://github.com/timschmidt/exactcore-hyper-comparison integrated under crates/comparisons with consolidated and re-validated logs and organized and complete code for all claims for reference in a future paper.  Verus coverage for hyperreal, hyperlattice, and hyperlimit.
7. **Idiomatic bindings** - For C / C++, Python, Javascript / Typescript via WASM
8. **CI checks** - should exercise as much testing as possible, deploy demos and a static site for the project as a whole.
9. **Documentation** - should synthesize the best aspects of current crates documentation, with hyperreal and csgrs being the flagship examples - the readme should explain what the crate does, why, and provide a quick entry into how along with an index of available types and functions which dive into deep educational-quality references in  rustdoc via good doc comments in the code and additional markdown documents in git where appropriate.
10. **Paper** - A scientific paper detailing important aspects of the project suitable for publication and education of others should be prepared.
11. **Optimization** - Opportunities for reducing the amount of code and complexity throughout the stack while preserving or enhancing the exactness > completeness > performance > memory > binary > code priority ladder should be logged, prototyped, evaluated, considered in combination with previously evaluated and rejected ideas, and so on.  Only keeping changes which improve without significant regression.  APIs should be as narrow and thin as possible while retaining honesty and the priorities.  Consider the use of parallelism through fearless_simd and multithreading.
12. **Release** - All crates in the monorepo should be prepared for release.  With all that entails.
13. **File organization** - Files should be organized, sized, and split / combined semantically such that finding relevant code is easy and no file is too long for reading.
14. **Future work** - each crate may have documentation discussing crate-specific future work which is also in scope.
