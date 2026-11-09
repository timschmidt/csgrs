# Simplification and modeling-API program (November 2026)

Status: **implemented** (3 November 2026). Every package has an outcome in
the [Outcomes](#outcomes) table: accepted, accepted in part, rejected or
deferred, each with its evidence. This program records an external review
received on 3 November 2026, the claims that were checked against the tree,
and the work packages derived from it. It sits under the
same acceptance contract as every other change. That means
[`properties.md`](../../properties.md), [`docs/admission.md`](../admission.md)
and the repository [agent rules](../../AGENTS.md), with the governing order:

**exactness > completeness > performance > memory > binary size > source size**

Integration follow-up (4 November): `closure-integration` fast-forwarded to
`a08c698757`, bringing the implementation here. The
[integration record](consolidation-2026-11-04.md) distinguishes historical
package outcomes from fresh verification and remaining work. A new U06
regression reproduced lost construction ancestry on empty expression leaves;
lowering now retains that evidence before replacing their geometry with `false`.

The [5 November BRep continuation admission](brep-continuation-2026-11-05.md)
qualifies the retained integration on its finite corpus. General closure and
D03 source conformance remain open; package outcomes below are unchanged.

The [8 November selected graph follow-up](implicit-shared-sections-2026-11-08.md)
extends exact finite incidence beside unselected common supports and corrects a
historical trace-recording claim. It preserves existing measured dispatch while
retaining adverse costs of the new quintic chain. General operation closure and
total resource control remain open; this is not a source-size refactor.

The D03 continuation review reproduced binary64 rounding in standard SVG path
ingress. The [decimal-ingress follow-up](svg-decimal-ingress-2026-11-08.md)
retains original decimal tokens in Hypercurve before any comparison adapter.
This corrects a comparison prerequisite. The [9 November corpus admission](openscad-corpus-2026-11-09.md)
records all 524 prototype outcomes and the maintained bounded collector. There
are 198 emitted-model agreements and 188 differences; source conformance, causal
classification, and numeric attribute/transform ingress remain open. The
[indexed PWN follow-up](indexed-pwn-closure-2026-11-09.md) extends closure admission
without geometric coincidence decisions when indexed edges already cancel.
The [exact transform follow-up](svg-exact-transforms-2026-11-09.md) reproduces
rounded SVG coefficients and intermediate affine composition in Hypercurve.
Its candidate retains original transform tokens and exact trigonometry; primitive
attributes and broader source conformance remain separate work.

The [exact paint follow-up](svg-exact-paint-2026-11-09.md) separately reproduces
lost positive paint factors and rounded range validation. Its focused witnesses
cover exact sign selection, inheritance and native continuation; rectangle
attributes, sampled shape conventions and general source conformance remain
open. Its affected owner/downstream, strict lint, MSRV and WASM witnesses pass;
a new complete publication gate and immutable evidence closeout remain pending.

The [native attribute follow-up](svg-exact-native-attributes-2026-11-09.md)
separately reproduces rounded line attributes and polygon/polyline point lists.
Its focused independent-coordinate and filled-set witnesses pass through native
replay and later operations. Affected owner, strict lint, MSRV and WASM checks
pass; combined/downstream checks subsequently pass as recorded below.
Publication qualification remains pending.
Rectangle attributes, sampled shape fidelity and general source conformance
remain open.


The isolated numeric-ingress integration combines the two committed SVG
follow-ups while retaining their original witnesses and all 448 capability
rows. A new CSGRS adapter witness combines exact line/point-list coordinates,
tiny positive paint, inherited opacity and exact transforms with independent
rational geometry, native replay, later Booleans and empty recovery. Its first
combined run reproduced malformed hidden native coordinates becoming successful
empty geometry: the original five root witnesses passed and the new sixth
failed. A narrow owner correction now validates native lines and point lists
before hidden selection; a sixth paint witness adds independent typed-error and
valid-empty controls, with a 449th capability row. Actual combined source
`2639c68fee` passes 118 owner reports, six root region reports, ten selected rows,
17 runner tests, unchanged dispatch controls, nineteen MSRV bodies and 57 WASM
executions. Seven root SVG harnesses pass sixteen reports; full BRep/IO passes
996. Strict owner/root lint passes. These results qualify the finite functional
increment; a complete publication gate and immutable evidence closeout remain
pending. Parent-only checks are not credited as combined-source completion.


Most packages below reduce source size, which is the lowest priority. Such a
package is retained only when every higher priority is preserved, with
evidence. Two packages are higher-priority correctness work and are marked as
such: the hyperdrc tokenizer (K02) and the modeling session (U02).

The review was source-read and measured, not built or run. Its line savings are
estimates. Recount on the target revision before starting any package.

## Review summary

- Literal copy-paste is about 5% of the code; hypermesh has almost none. Most
  of the size is the real complexity of exact algebraic offsets, plus a very
  large test suite.
- The realistic production saving is 15-20k source lines from removing
  plumbing and duplicated helpers. There is also a large cut available in test
  boilerplate.
- One much larger, unproven opportunity exists: the selected polynomial field
  route versus the per-case cusp/semicircle kernels (K06).
- The public modeling API exposes good abstractions (exact `Real`, explicit
  policy, certainty on every result), but variants inflate it. The convenient
  path also discards per-step certainty.
- Downstream, SynapsCAD flattens typed failures and certainty at its boundary.
  The openscad-rs compatibility bar is low, and the alumina READMEs read like
  changelogs.

## Spot checks on `e83de9ee8c`/`2bc58a1589`

| Claim | Review figure | Checked | Result |
| --- | --- | --- | --- |
| `Uncertain(r) => return [Ok(]Uncertain(r)` pass-through arms | 2,790 workspace, 1,702 in `bezier_offset` | multi-line regex, any path prefix, with or without `Ok(` | 2,841 workspace, 2,079 in `bezier_offset*`. Only 578 match the strict `Classification::Uncertain(x) => return Ok(Classification::Uncertain(x))` spelling. |
| Local `decided` helpers reinvented per file | several | `fn decided`/`macro_rules! decided` in `crates/` | 86 definitions |
| `point_segment_distance_squared` in hyperpath `pcb_*` | 5 | definitions in `crates/hyperpath/src` | 5 files |
| `sort_reals_exact` in hypersolve | 4 | definitions | 4 |
| hyperdrc S-expression tokenizer never fails | yes | `crates/hyperdrc/src/sexp.rs` `tokenize` returns `Vec<String>`; `read_string` stops at end of input | Confirmed. An unterminated string is accepted silently. hypercircuit and hyperpath return typed errors. |
| `hyperlattice/src/matrix/core.rs` size | 11.7k lines | `wc -l` | 11,674 |
| `cusp_semicircle/` size | ~37k lines | all `.rs` under `*cusp_semicircle*` | 47,185, which is larger than the review states |
| `polynomial_curve_selected_circle.rs` | 553 lines | `wc -l` | 553 |
| hypercurve tests | 260k lines | files with `test` in their path | 182,704, of which 74,564 are under `tests/`. Inline `#[cfg(test)]` modules account for the rest of the review's figure. |
| hyperdrc depends on csgrs | 2 call sites | `csgrs::` uses and manifest | 6 `csgrs::` references; `csgrs` with `curve`, `gerber-io`, `offset` |
| hyperreal `simple` used only by comparisons | yes | manifests | Only `crates/comparisons` enables it, plus hyperreal's own examples |
| `cached-f32-approx` unused | ~28 `cfg` sites | manifests and `cfg` sites | Declared only in hyperreal; 22 source references; no manifest enables it |
| `_with_context` public functions in `solid.rs`/`curve/` | 57 operations | `pub fn *_with_context` | 58 (81 across `src/`) |
| `sdf` variants | eight | `pub fn sdf*` | eight: `sdf`, `sdf_expr`, and their `_with_context`, `_with_diagnostics` and `_with_diagnostics_and_context` forms |
| `HypermeshResult` in `solid.rs` | 28 | `grep -c` | 28 |
| SynapsCAD `Shape::Failed(String)` | yes | `src/compiler/geometry/mod.rs:45` | Confirmed |
| SynapsCAD Booleans fold pairwise | yes | `src/compiler/evaluator/booleans.rs` | Confirmed: `for child in rest` loops per operation |
| openscad-rs pass-rate bar | > 50% | `tests/openscad_compat.rs:77` | Confirmed: `passed / total > 0.5` |

The remaining figures were not checked. These include the 63 matrix twin
functions, the 1,659 trace-formatting lines, the 15 `A + B√S` sign copies, the
six circle-line classifiers, the 5 determinant kernels and the test-fixture
call counts. Recount them before scoping the package that uses them.

## Gates for every package

A package is merged only when it shows all of the following on the same
completed contract:

1. **Identical exact outputs.** This includes identical serialized geometry and
   checksums under both `STRICT` and `APPROXIMATE_512`, with identical certainty
   receipts.
2. **Identical `dispatch-trace` counts.** This is the guard most likely to catch
   a refactor that silently changes which fast path fires. Existing assertions,
   such as the 53 in `conversion_tests.rs` and the nested-image dispatch tests,
   must stay unchanged. Add a count assertion for any path the package touches
   that has none.
3. **No regression in benchmarks or binary size.** Cover the owning crate's
   `PERFORMANCE.md` benches, `successive_operations` and the size harnesses. A
   source reduction that grows monomorphized code or a payload fails P36.

The order of sign and predicate calls must not change. That order decides which
proofs are cached and reused (P4, P10). A refactor that reorders calls,
evaluates a hook eagerly, or merges two branches with different first
predicates is a behavior change, even when every test still passes.

**Do not touch** the following; they were judged essential rather than redundant:
- the cheapest-method dispatch order;
- the separate algebraic field kinds, including the four circle-frame variants;
- the hyperreal computable/rational core;
- hyperlimit's decision sequence;
- hyperdrc's 2D spatial index.

## Kernel simplification (K)

### K01 — A shared pass-through operator for `Classification`

The block

```rust
match x { Classification::Decided(v) => v, Classification::Uncertain(r) => return Ok(Classification::Uncertain(r)) }
```

and its unwrapped form are the largest repeated pattern: about 2,800 arms. Many
files define a private `decided` helper for it.

- Add one shared macro or `Try`-style adapter in `hypercurve` (and where needed
  in hypersolve/hyperlimit) for `Classification<T>` and
  `CurveResult<Classification<T>>`. Delete the per-file helpers it subsumes.
- Apply it with a codemod that rewrites only arms that are exactly this
  pass-through. Leave untouched any arm that inspects the reason, such as one
  that treats `BudgetExhausted` specially, maps it, or does work first.
- Start with `bezier_offset` alone, as a purely mechanical first PR.
- Estimate: 8-10k lines. No meaning changes. The gates still apply, because the
  early return must stay at the same point in the call sequence.

### K02 — Consolidate small duplicated helpers

- hyperpath `pcb_*`: one `point_segment_distance_squared` family (five copies).
- hypersolve: one `sort_reals_exact` (four copies), and one
  `derivative_coefficients` across crates (four copies).
- Polynomial trimming: seven functions. Some decide zero structurally and some
  through a policy. Keep two named versions, structural and policy-decided, and
  never merge them into one.
- hypersolve: replace the four private two-endpoint interval structs with
  `RealInterval` where their invariants match.
- 3×3 determinant kernels: five copies. Share them only where the kernel and its
  sign path are identical.
- Repeated per-crate predicate wrappers: consolidate them in the owning layer
  (P25).
- S-expression tokenizers: three copies (hyperdrc, hypercircuit, hyperpath). **The
  hyperdrc tokenizer is a P6 defect, fixed first and separately.**
  `crates/hyperdrc/src/sexp.rs` returns `Vec<String>`, and `read_string` returns
  whatever it has read when the input ends. So an unterminated string is
  accepted silently. The fix returns a typed error, with a regression test, before
  any sharing. After that, one shared tokenizer may serve all three if the
  borrowed-atom behavior of hyperpath's tokenizer is kept.
- Estimate: 2-2.5k lines.

### K03 — Const-generic `Matrix3`/`Matrix4` internals

`hyperlattice/src/matrix/core.rs` has 11,674 lines with function pairs that
differ only by dimension, plus multi-line `trace_dispatch!` formatting.
- Move the twins to const-generic internals behind thin `Matrix3`/`Matrix4`
  wrappers, and use single-line trace calls.
- Keep the hand-specialized dense-certificate paths as they are.
- Check monomorphized code size and the hyperlattice benches. A shared generic
  body that defeats a dimension-specific specialization fails the gate.
- Estimate: 2-3k lines.

### K04 — Shared skeletons inside `cusp_semicircle/`

`cusp_semicircle/` is a grid: one kernel per construction kind (semicircle,
chord, parallel curve) for each partner kind (chord, rational curve, parallel
curve, semicircle). Three procedures repeat inside the cells:
- **Sign of `A + B√S`.** About 15 near-identical copies in `fiber_sign.rs`,
  `bezier_offset.rs`, `selected_fiber.rs` and `parameters.rs`. One generic
  routine with per-field hooks, keeping the trivariate shortcuts as hooks.
  Estimate: 0.6-1k lines.
- **Circle-line contact classification.** Six sibling functions in
  `chord_kernel.rs` repeat the discriminant branch, side test, clip and
  parameter-map construction. One classifier that takes a signing closure per
  case. Estimate: 1-1.5k lines.
- **Angular ordering and component publishing.** In `parameters.rs` and
  `rational_kernel.rs`. Estimate: 0.8-1.2k lines.

This overlaps P05 (S7 cusp-contact/chart consolidation) in
[`hypercurve_rd_program.md`](hypercurve_rd_program.md#p05--s7-cusp-contactchart-consolidation).
Do it under P05's proof, measurement and stop rules. Preserving the order of
sign calls is the binding constraint.

### K05 — Remove hypercurve's second Bernstein root isolator

`bezier_parameter.rs` (around lines 4150-5330) reimplements polynomial division,
rational-root reconstruction and Bernstein subdivision that hypersolve already
provides (`root_isolation`, `isolate_ordered_field_polynomial_roots`).
Hypercurve already calls the hypersolve versions elsewhere.
- Precondition: show that the two paths produce identical isolating intervals
  and identical root ownership on the corpus, not just the same root counts.
- Estimate: 0.8-1.2k lines.

### K06 — Experiment first: the selected polynomial field route

`polynomial_curve_selected_circle.rs` (553 lines) does selected-circle work
through `SelectedPolynomialField`. That is the same kind of job `cusp_semicircle/`
does in about 47k lines. No benchmark compares them.

The work is a head-to-head benchmark on the existing cusp fixtures, under both
policies. It measures:
- completion;
- certainty;
- time, allocation and peak memory;
- whether the route avoids joining two algebraic fields into one.

This is an experiment, not a migration. Record the result either way in
`crates/hypercurve/PERFORMANCE.md` and in this ledger. If the route matches, a
migration plan follows under P05's gates. If it does not, record it as rejected
with its evidence.

**Outcome (2026-11-03): rejected.** The two routes share only finite chords on
rational lines (and circles with rational centers). Rational, parallel and
selected-endpoint partners need implicitization or a field join. On 18
line cases (three cusp fixtures, three lines, both policies) the contacts and
certainty are identical. The general route costs 31–61× more time on
rational-frame centers, and about 10⁴× on the ellipse-offset parallel-normal
center (≈1 min against 5–11 ms). It must also borrow the kernel's half-circle
selection. Evidence is in
[`crates/hypercurve/PERFORMANCE.md`](../../crates/hypercurve/PERFORMANCE.md#k06-selected-polynomial-field-route-versus-cusp-semicircle-kernels-2026-11-03).

### K07 — Dependency and build-time trimming

- **hyperdrc → csgrs.** hyperdrc uses csgrs for Gerber import and a thin wrapper
  over a hypercurve method. Moving `src/io/gerber.rs` into hypercurve (behind a
  feature) or into a small crate lets hyperdrc drop csgrs, and with it hypermesh
  and hypertri.
- **hyperreal `simple`.** A 2.4k-line Lisp parser enabled only by
  `crates/comparisons` and hyperreal's own examples. It could move to
  `crates/comparisons`.
- **`cached-f32-approx`.** No manifest enables it. Remove its 22 source
  references, or record why it is kept.

These packages save dependency-graph size and build time, not lines. Each needs
the release and MSRV consumer checks and a `docs/MIGRATIONS.md` entry for any
feature removed from a published crate.

### K08 — Hypercurve test-support helpers

Test boilerplate is the largest pile: repeated policy loops, `Real::from`
fractions, `Point2::from_values` and `.unwrap()`. Add one test-support module
(for example `tests/support/mod.rs`, plus a `#[cfg(test)]` counterpart):
- `for_each_policy`;
- `r!(3/7)` and `pt!(x, y)` macros;
- a `decided_ok()` helper.

Then migrate tests file by file. A migrated test must check exactly what it
checked before: no assertion may be dropped or weakened (P35). Migrate the
original hard cases with the same inputs.

## Public modeling API (U)

These packages change csgrs's public surface. Each one is additive first. Old
names are kept with `#[deprecated]` for one release and recorded in
[`docs/MIGRATIONS.md`](../MIGRATIONS.md). U02 is the most important, because it
removes a correctness hazard, not only a style problem.

### U01 — One name per operation

- Drop the `try_` prefix where `x` and `try_x` both exist with the same
  signature. Every operation already returns `Result`.
- Replace the `_with_context`, `_with_diagnostics` and
  `_with_diagnostics_and_context` variants:
  - optional outputs, such as diagnostics, always go inside the returned
    outcome;
  - optional inputs go in an options struct with a `Default`.
- Expected effect: well over a third fewer public functions. `sdf` alone has
  eight variants.

### U02 — A modeling session carries policy and certainty (P8, P9, P19)

Today the convenient path (plain functions plus `.into_value()`) discards each
step's certainty. Knowing whether a whole model is certified means threading
`GeometryOutcome::and_then` by hand. A session type should:
- choose the policy once, visibly (`Csg::strict()`,
  `Csg::new(GeometryContext::APPROXIMATE_512)`), never by an internal default
  (P8);
- keep the weakest certainty across every operation, and report it
  (`m.certainty()`). Inherited qualification must stay distinguishable from
  operation-local proof (P9, P19);
- make the `_with_context` variants unnecessary. Free functions can remain as
  thin `STRICT` wrappers.

`GeometryContext` is a one-byte `Copy` type, so pass it by value wherever it
remains a parameter.

The acceptance tests are three chains, each of which must report the correct
whole-chain certainty, with no wrapper able to upgrade it:
- a strict-only chain;
- a chain with one approximate step;
- a chain with an approximate step followed by strict steps.

### U03 — `impl IntoReal` at the modeling edge

Every example defines its own `fn r(f64) -> Real`. Public modeling entry points
should accept `impl IntoReal`:
- integers, ratios, `Real` and `&Real` convert exactly;
- `f64` converts exactly to its binary value;
- NaN and infinity become a `ValidationError` (P6).

Document clearly that `0.1_f64` is not 1/10. Offer `ratio(1, 10)` or
decimal-string parsing for decimal intent. No value is ever demoted (P1); only
call sites get shorter.

### U04 — One error type

Booleans return `HypermeshResult`, while nearly everything else returns
`ValidationError` (28 against 223 uses in `solid.rs` and `curve/native.rs`).
Provide a single `csgrs::Error` with `From` conversions, keeping the kernel error
as its `source()`. Typed diagnostics must survive (P6, P30).

### U05 — Methods or free functions, consistently

Constructors stay functions (`cube`, `circle`). Everything that transforms or
combines geometry becomes a method, so models read left to right. Today
`SolidExt` has the Booleans and `translated`/`transformed`. Rotate, scale,
mirror, center and the other transforms exist only as free functions.

### U06 — A lazy CSG expression tree compiled to one `BooleanProgram` (P21)

Follow-up (4 November): evaluation errors now retain a `SolidExprChild` path
and boxed native cause in `Error::Expression`. `SolidExpr::node_at` resolves
that path. Indexed input failures and failed transforms are localized; shared
arrangement errors and native failures without an operand index identify the
root evaluation. Failure categories and source chains remain available. The
[admission record](expression-node-errors-2026-11-04.md) records the reproduced
missing context, current checks, cost observations and remaining limits.

Hypermesh already has `BooleanProgram` and shares arrangement work across
expression roots, but the public API evaluates one Boolean at a time.

A lazy tree would:
- allow operators (`(a - b) | c`), because nothing fails before `evaluate`;
- let a validation error name the tree node that caused it;
- compile the whole model into one program, so intersections are shared rather
  than recomputed per step;
- give a natural place for subresult caching and provenance.

It is new surface, so it breaks nothing, and the eager API stays underneath for
kernel-level users. Prototype it against `successive_operations` and report the
saving on the same completed contract (P36).

## Downstream applications (D)

These packages live in their own repositories. They are listed here because
they consume the stack's contracts.

- **D01 — SynapsCAD keeps typed failures and certainty (P6, P30).**
  - Replace `Shape::Failed(String)` with a typed failure that carries the
    openscad-rs source span.
  - Show a certainty badge per part in the viewport.
  - alumina-interface already keeps evidence this way and is the reference.
- **D02 — SynapsCAD lowers Boolean subtrees into one program.**
  `eval_boolean_op` folds children pairwise, so `union()` over N children builds
  N−1 arrangements. The OpenSCAD syntax tree already is a CSG tree. Lower each
  `union`/`difference`/`intersection` subtree into one hypermesh `BooleanProgram`
  (through U06 once it exists), and benchmark the change.
- **D03 — openscad-rs compatibility ratchet.**
  - `tests/openscad_compat.rs` requires a pass rate above 50%. Record the
    current count and fail CI if it drops, so each fix is locked in.
  - Then evaluate the same suite in SynapsCAD and diff the results against
    OpenSCAD with its Manifold backend. That gives the SCAD corpus and the
    float-kernel comparison of P34.
- **D04 — Newcomer READMEs.**
  - The alumina-interface and alumina-firmware READMEs are dense accounts of
    recent certification work.
  - A README should state what the system is, show the architecture, explain how
    to build it and say what currently works.
  - Certification narratives move to `docs/`. The same applies, less strongly,
    to csgrs.

## Outcomes

| Package | Status | Outcome |
| --- | --- | --- |
| K01 (`Classification` pass-through) | accepted | `hypersolve::decided!(value)` unwraps a decided `Classification` or returns its uncertainty unchanged; `decided!(value, Ok)` returns it wrapped. It expands to the same match that was written by hand. A codemod rewrote only matches whose two arms were exactly the pass-through, in either order and with no comments: 1,135 in `bezier_offset` (commit `062a289ad4`, 3,778 lines removed), then 379 more in Hypercurve, hypersolve, HyperBRep and Hypercircuit (`e6d3facf71`), whose codemod accepted only `Classification` paths. There are now 1,516 uses. Arms that inspect, map or react to the reason stay hand-written, as do the five two-arm matches that return another enum's `Uncertain`. Hypercurve's release test binary `.text` was unchanged (29,892,736 bytes) and so was every dispatch-trace count assertion. No per-file `decided` helper is subsumed by the macro: each one turns uncertainty into an error, `None` or a panic. Byte-identical copies were handled under K02 and K08. Two production copies in `polynomial_curve.rs` and `polynomial_curve_pair.rs` have identical text but call file-local `blocked` helpers that attribute different operations (evaluation, intersection), so they stay. |
| K02 (hyperdrc tokenizer) | accepted | Tokens are now typed (`Open`, `Close`, `Atom`) and `tokenize` returns `Result`. Two defects were reproduced on the old code by new tests: an unterminated quoted string was accepted, taking the rest of the input as its text, even after a trailing escape; and a quoted `"("` or `")"` was read as a list delimiter. Both tests failed before the change and pass after. HyperDRC release `--all-features`: 1,672 passed (1,670 before, plus the 2 new tests), with the board corpus unchanged. Sharing one tokenizer with hypercircuit and hyperpath remains under K02. |
| K02 (3×3 determinant kernels) | rejected | Five kernels were compared: `hyperlattice` `projective::det3` and `matrix::determinant3`, `hyperlimit` `orient::det3_refs` and `halfspace::det3`, and `hyperphysics` `mass::determinant3`. No two compute the same way. They use a cofactor `dot3`, six-term `signed_product_sum` with different sign patterns and term orders, and an explicit `mul`/`sub` chain. Each builds a different exact `Real` expression, so merging them changes expression structure. The package's own rule is to share only identical kernels and sign paths. |
| K02 (S-expression tokenizer sharing) | rejected | The three tokenizers live in independent crates with different error contracts (`anyhow`, `String`, `SpecctraParseError`), and hyperpath's borrows atoms from the source. Sharing one would add a dependency edge between hyperdrc, hypercircuit and hyperpath to save under 100 lines. The correctness defect, hyperdrc accepting an unterminated string, is fixed in its own crate. |
| K02 (per-crate predicate wrappers) | accepted in part | An inventory of small sign and compare helpers found no helper identical across crates. Within crates, 26 groups (73 copies) were byte-identical. 23 groups are now one `pub(crate)` copy each, imported by the other files, in hypertri, hyperpath, hyperphysics, hyperlimit, hyperpack and hypersolve; 456 lines removed, 96 added. A group was merged only when every callee resolves to the same item in every file: same import path, no local definition, no glob import. Three groups stay: two `tensor_resultant.rs` trims with a local `zero`, and Hypercurve's `negate`, which sits in files with glob imports. Release `--all-features` tests: hyperlimit 376, hypertri 214, hyperpath 444, hyperphysics 73, hyperpack 155, hypersolve 1,332, with no failures. A later pass (commit `344dddb724`) found that the inventory had missed hyperlimit's decision-trace bookkeeping: `stage_rank`/`max_stage` in 10 predicate modules, `certainty_rank`/`max_certainty` in 7, and `DecisionTrace`, `UnknownDecision` and `decided` in 3-4, all byte-identical with identical callees. They now live once in `predicates/decision.rs` (423 lines removed, 90 added). Three files keep a `max_certainty` with a different body, and `order.rs` keeps its `#[inline(always)]` copies. hyperlimit release `--all-features`: 376 passed, unchanged; no hyperlimit symbol in the release bench binaries changes size (the only `.text` difference is a bench report writer that embeds the checkout path). |
| K07 (`cached-f32-approx`) | rejected (kept) | It is a published feature of hyperreal 0.13.0 on crates.io and is documented in hyperreal's README. Hypertri's verified matrix runs with each approximation cache on its own and with both (`crates/hypertri/PERFORMANCE.md`). Removing it would break published users to save about 22 `cfg` lines at the lowest priority. It is tested under `--all-features` in the hyperreal workflow. |
| K07 (hyperdrc → csgrs) | rejected | `cargo tree -p hyperdrc -e normal --prune csgrs` removes 18 of 147 crates. Eleven come with the Gerber importer itself (`gerber_parser`, `gerber-types`, the regex family, `lazy-regex`, `strum`, `uuid`) and would return wherever the importer moved. The net saving is 7 crates: csgrs, hypermesh, hyperphysics, `doc-image-embed`, `hashbrown`, `foldhash`, `allocator-api2`. hyperdrc also uses `csgrs::curve::try_finite_profiles`. Moving the 1,902-line importer, which depends on `csgrs::curve`, `GeometryContext` and csgrs's `IoError`, plus that function, across a crate boundary in a release-prepared stack costs more than 7 crates of build graph save. Revisit if hyperdrc's build time becomes a measured problem. |
| K07 (hyperreal `simple`) | rejected (kept) | `simple` enables the public `Simple` type, the published `hyperreal` calculator binary and the `library_perf` bench. Moving it to `crates/comparisons` would remove published API and a binary to save 2,363 source lines at the lowest priority. |
| K02 (hyperpath `pcb_*` helpers) | accepted | Fourteen private helpers were byte-identical in two to five of the `pcb_*` modules: the distance kernels, `cross`, `dot`, `squared_norm`, polygon edges and area, margin classification and report constructors. Every body was compared byte for byte, and so were the callees of every moved function. They now live once in `pcb_shared.rs`. `segment_segment_distance_squared` is shared only by the two modules whose copies matched; `pcb_obround_pad` keeps its different copy. The change is 705 lines removed and a 236-line shared module. |
| K02 (hypersolve `sort_reals_exact`) | accepted | Four identical insertion sorts, all over `hyperlimit::compare_reals`, became one `pub(crate)` function in `algebraic.rs`. |
| K02 (hypersolve interval structs) | accepted | Four private `{lower, upper}` structs of `Real` (`ValueInterval` ×3, `AlgebraicValueInterval`) had no methods and are replaced by `RealInterval`. Their policy-taking arithmetic stays local, because `RealInterval`'s own methods compare under STRICT only. `CertifiedRationalInterval` holds `HyperRational` endpoints and stays. |
| K02 (`derivative_coefficients`) | rejected | The four copies differ in failure contract: `Vec`, `Option` and `CurveResult`. A shared version would have to change one of them. |
| K02 (polynomial trimming) | rejected | Hypersolve has eight trimmers of `Real` coefficient vectors, and every pair differs in at least one contract. Zero is decided in four ways: structurally from an exact rational (`algebraic_polynomial_image` ×2, `algebraic_binary`), from `zero_status` (`algebraic_fiber`), by a policy comparison (`algebraic::trim_polynomial`), or by a policy comparison after an exact-rational shortcut (`trim_polynomial_for_evaluation`, `root_isolation::trim_polynomial_slice`, `algebraic_rational_image::trim_real_polynomial`). The last two kinds call predicates in a different order. Failure contracts also differ: empty input gives `None`, `Some(&[])` or a pushed zero; a non-rational coefficient fails or stops; one rejects constants. Merging any two would change a decision path or a failure contract, which the gates forbid. The structural/policy split the package asked for already exists in their names; the resultant trimmers with a precision fallback are a third kind. |
| K08 (test-support helpers) | accepted in part | `r!(3 / 7)` replaced 1,213 exact fraction literals (`e6d3facf71`). `pt!(x, y)` expands to `Point2::from_values(x, y)`, with `Point2` resolved where it is used. It replaced 1,180 calls in the 16 files where the shared macros are already in scope, the unit-test modules and integration tests with `#[macro_use] mod support`, and saved 896 lines. The other 47 integration files would each need a macro import, for about 100 lines in total, so they were left. `test_support::decided` and `support::decided` (`#[track_caller]`) replace 31 file-local decided-or-panic copies whose bodies differed only in panic text, and `decided_ok(result)` replaces 58 `decided(result.unwrap())` calls. Hypercurve's assertion macros (15,355), `.expect(` calls and every other check are unchanged. `.unwrap()` and `decided(` fall by exactly the 58 calls `decided_ok` absorbs, and `panic!` by the 31 removed copies, plus the 4 panics in the new helpers. Hypercurve release `--all-features`: 3,088 passed and 37 ignored, the same as at `1a0972a2fc`. `for_each_policy` is rejected: Hypercurve's 1,268 policy loops (1,079 over `CurveContext`, 189 over `PredicatePolicy`) are already one line each, and a closure helper would change the control flow of `?`, `return` and `continue` in their bodies. |
| U02, U03, U04 (session, exact numbers, one error type) | accepted | `csgrs::Csg` chooses the policy once and keeps the weakest operation-local certainty of every operation it runs, including receipts folded in through `run` and `record`. Inherited mesh qualification stays in `construction_evidence`, so the two claims are not conflated (P9). `IntoReal` converts integers, `Real`, `&Real` and `Rational` exactly, and `f32`/`f64` to their exact binary value. NaN, infinities, zero denominators and malformed decimal text are the new `Error::InvalidNumber`, classified as invalid input. `ratio` and `decimal` give decimal intent exactly. `csgrs::Error`/`csgrs::Result` alias `ValidationError`, which already wraps `HypermeshError`. `tests/session.rs` covers the plan's three chains (strict only; one approximate step; approximate then strict), shows that no later step or `record` can upgrade the chain, and checks number conversion. The quickstart now uses `Csg`; old and new quickstarts write byte-identical STL. Additive only (`docs/MIGRATIONS.md`). Follow-ups (commits `98dbce3628`, `d88e4b3374`): `GeometryContext` is now passed by value everywhere, as U02 asked. Every type position and constant borrow was rewritten, then every call-site borrow rustc named, in 100 files across csgrs, csgrs-ffi, hyperdrc, hypercircuit, tests, examples, benches and fuzz targets. No `&GeometryContext` remains, while borrows of a derived `MeshContext` stay. U03 now reaches the STRICT modeling edge: the 57 convenience functions of `solid` and `curve` without a `GeometryContext` parameter, plus `SolidExt::translated`/`rotated`/`scaled`, take `impl IntoReal`, bound by `into_real()?` at the top of the body. Functions with an explicit policy keep exact `Real`, as the precise kernel-level form. Five of the ten examples no longer define `fn r`. The other five keep it only for kernel types (`Point3`, `Vector3`, `Matrix4`, hypercurve offsets) and policy-taking calls, which are outside csgrs's modeling surface. All 13 examples write byte-identical files against `1a0972a2fc`. |
| K03 (const-generic `Matrix3`/`Matrix4`) | rejected | Measured on `matrix/*.rs`: 49 dimension-suffixed function pairs (3,703 lines), plus 17 same-named `impl Matrix3`/`impl Matrix4` methods. After normalizing 3/4, only 3 pairs (36 lines) are identical, and 12 are near-identical. The others are hand-unrolled per dimension, and their order of `Real` operations determines expression structure and dispatch paths. A const-generic loop body would change both, which the gates forbid. The review's count of 63 pairs reproduces, but its premise that they differ only in dimension does not. |
| K03 (one-line matrix trace calls) | accepted | A file-local `trace_matrix!(operation, path)` forwards the same triple to `trace_dispatch!` under the `hyperlattice_matrix` layer. All 427 calls in `matrix/core.rs` now fit on one line, so the file shrinks by 1,070 lines. The `mathbench` dispatch-trace report, which has 1,881 `hyperlattice_matrix` rows, and the `api_dispatch_trace` report are byte-identical before and after. Hyperlattice: 241 tests pass with all features. |
| K04 (sign of a two-term sum) | accepted in part | Eight functions in `bezier_offset.rs`, `fiber_sign.rs` and `parallel/point.rs` repeated the same combination: an early decision when a term's sign is zero or both agree, then a dominance sign selecting the first or second term. They now call `two_term_sign(first, second, dominance)`. The dominance closure runs only after both term signs are decided, so the order of sign evaluations is unchanged. A site is rewritten only when every `return` in the moved code yields `Decided(Zero)`, which the helper maps to itself. Saving: 67 lines. The other six early-return blocks feed different final mappings and stay. |
| K04 (circle-line contact walk) | accepted in part | `retained_offset_chord_intersections` and `selected_radial_chord_intersections` had identical contact walks, apart from their nested expression type, sign function, diameter field and error text. A generic `nested_chord_contacts` now serves both. The rewrite script asserted that the second walk equals the first after exactly those substitutions. Saving: 33 lines. The other circle-line kernels (122-624 lines each) are only 60-76% alike line for line, so a shared skeleton would need hooks that cost about as much as they save. |
| K04 (angular ordering and component publishing) | deferred | The `*_contact_order_to_real` family (10 functions) and the `*_replay_rational_circle_component` family (6) are at most 76% alike line for line. A shared skeleton would need per-case hooks around different field towers. Deferred under P05 until K06 shows whether a general route can replace them. |
| K05 (second Bernstein isolator) | rejected | Hypercurve's unit-interval isolator returns `BezierParameter2` with exact rational-root recognition (modular screening, reconstruction) and policy-aware classification, and it already delegates non-rational fields to hypersolve's `isolate_ordered_field_polynomial_roots`. hypersolve's rational-row tools (`isolate_univariate_polynomial_roots`, `subdivide_bernstein_univariate_polynomial_interval_roots`) work on expression-problem rows over the line and report intervals or subdivision evidence. They are not an equivalent isolator. Removing the copy would mean porting rational-root recognition into hypersolve and accepting different isolating intervals, which is a representation migration and fails the package's own precondition. |
| U01 (one name per operation) | accepted | The 11 duplicate `try_x`/`x` pairs, where `x` only called `try_x` (12 functions across `solid` and `curve`), are removed in favor of the plain names. The `try_` prefix is also dropped from their policy-taking variants: 192 references were rewritten, including Hypercircuit's. These names were never published (crates.io has 0.20.1), so they were removed instead of deprecated, and `docs/MIGRATIONS.md` lists the renames. Follow-up (commit `d88e4b3374`): the six `_with_diagnostics` and `_with_diagnostics_and_context` variants of `sdf`, `sdf_expr` and `metaballs` are removed. Diagnostics are an optional input (`SampleOptions`, default off) of the `_with_context` form, and an optional output inside its outcome (`SampledMesh { mesh, diagnostics }`, `Some` exactly when requested). They stay off by default because collecting them costs an exact sign and extremum comparison per sample. A test shows, under both policies, that requesting them changes neither the mesh nor the certainty. The remaining `_with_context` forms stay as the single explicit-policy form of each operation. An options struct with a `Default` would choose the policy implicitly, which P8 and U02 forbid, and `Csg::run` reaches every one of them. Each operation now has at most two names: the STRICT convenience form and the explicit-policy form. Public `_with_` functions in `src/`: 117 before this follow-up, 111 after, none `_with_diagnostics`. None of these names were published either. |
| U05 (methods or free functions) | accepted | `SolidExt` gains `rotated`, `scaled`, `mirrored` and `centered`, so every transform is a method alongside `translated` and `transformed`. Each one is the corresponding free function, and a test compares them. Constructors stay functions. |
| U06 (lazy CSG tree) | accepted | `SolidExpr` records unions, intersections, differences and symmetric differences (also as `|`, `&`, `-`, `^`) plus lazy transforms. It evaluates the whole model as one `hypermesh::boolean` truth program. Transforms are pushed to the leaves and applied with the eager functions, empty leaves are the constant false node, and shared subtrees are lowered once. Tests show equal sets to step-by-step evaluation (empty exact symmetric difference, equal exact volume, and the independent closed-form volume 1544), every operator matching its eager operation, pushed-down rotate/scale/translate, empty-leaf identities, and certainty recorded through `Csg::evaluate`. With `dispatch-trace`, a five-box union builds 1 arrangement instead of 4. Release timing for a 24-cube drilled by eight holes: 11.9 ms median for one expression against 14.6 ms step by step, with the same exact set (host load about 7). |
| K06 (selected polynomial field route) | rejected | Head-to-head on the cusp fixtures (`crates/hypercurve/PERFORMANCE.md`, K06 section). The routes share only chords on rational lines and circles with rational centers. Rational, parallel and selected-endpoint partners would need implicitization or a field join. On the 18 shared cases (3 fixtures × 3 lines × both policies), contacts, counts and certainty are identical. The general route costs 31-61× more time on rational-frame centers and about 10⁴× on the ellipse-offset center (about 1 min against 5-11 ms; 2,268 retained sign queries against 1), and it must borrow the kernel's half-circle selection. The per-case kernels stay; K04's deferred items keep their shape. |
| D01 (SynapsCAD typed failures and badge) | accepted in part (local `657e6b1`; migration note `4fcb1d3`) | Typed failures retain the kernel cause and nearest authored statement through deferred Boolean trees and transforms. Successful and failed parts preserve numbering, inherited qualification, AI context and preview palette. A live native UI check verified the badges and failure location. Canonical empty meshes remain distinct from malformed buffers, and `compile_views_only` rejects failed or partial compilations. Focused regressions and all 32 UI tests pass. The [native-source follow-up](native-source-boundaries-2026-11-04.md) at `1f1ec3e` retains native geometry through display/export; all 43 binary/UI tests pass. Full configured library CI now has one B-text failure; strict local proof does not certify inherited source fidelity. See the [current admission](downstream-api-admission-2026-11-04.md) for commands, source stages and limits. |
| D02 (SynapsCAD Boolean subtrees) | accepted in part (local `657e6b1`) | 3D Boolean subtrees and transforms use one shared `SolidExpr`; difference is first minus union of the rest, and 2D regions keep their own authority. Four-child tests retain independent exact volumes 880 and 60, exact bounds and one-arrangement dispatch evidence. Source-span metadata follows balancing and transforms. Batch triangulation/order changes are an explicit representation migration. Earlier 30.6–31.1 ms versus 27.9–28.3 ms timings (host load 11–22) are historical, not a fresh speedup claim. The initial CI run recorded 59 passes and two failures. The [native-source follow-up](native-source-boundaries-2026-11-04.md) resolves the near-coincident failure: current CI records 64 passes, one B-text failure and two ignores. Native normal construction has measured adverse cost; this is a capability/artifact migration. See the [current admission](downstream-api-admission-2026-11-04.md). |
| D03 (openscad-rs ratchet) | accepted (local commit `2b7700a` in openscad-rs) | The compat job never ran: `/vendor/` was gitignored, so the declared submodule had no gitlink, and the test skipped and passed. The OpenSCAD corpus is now pinned at `8e8df3dd`, and CI requires it. The >50% floor is replaced by an exact list of the 513 of 524 files that parse; the test fails on a regression or on an unrecorded new pass. All four failure modes were exercised. The 11 failures are classified: five deliberately invalid upstream, six real gaps. At the original checkpoint its manifest required an unresolvable `../hyperreal` 0.13.1. The later monorepo-layout row below records the local correction and qualification. |
| D03 (SynapsCAD against OpenSCAD with Manifold) | deferred (524 outcomes collected; source conformance incomplete) | The [9 November admission](openscad-corpus-2026-11-09.md) records all 524 original prototype cases: 198 emitted-model agreements, 188 differences, and 138 other outcomes. The maintained collector preserves exact OBJ/SVG boundary-model comparison, certainty, failures and continuation identities, with external wall and per-process address-space containment. Twenty-six Python tests, nine Rust tests, eight geometric controls and paired real-engine controls pass. A reference crash after an empty marker is rejected before empty-model adoption. These do not establish original-source fidelity or resolve every corpus outcome. A caller-context/cache repair is locally committed in SynapsCAD; its unchanged B-text CI deadline still fails, so it remains unpushed. |
| D03 (openscad-rs monorepo layout) | accepted (local commit `c0d92e6` in openscad-rs) | The maintainer chose the monorepo layout (2026-11-03). hyperreal now comes from `../csgrs/crates/hyperreal` at 0.14.0, both CI jobs fetch csgrs at `e83de9ee8c`, and `fuzz/Cargo.lock` follows. Format, Clippy, tests, the ratchet with the corpus required and the locked fuzz check pass in the real directory. zizmor reports no findings on `.github`. |
| D04 (alumina-interface README) | accepted (local `efd379f`) | The introductory README and detailed `docs/CURRENT-BASELINE.md` are committed. A replay against original base `2c49c81` verifies 553 narrative lines unchanged after relative-link rebasing and resolves every file target. This is document fidelity evidence, not fresh runtime or hardware qualification. Unrelated pending GUI planning edits remain untouched; see the [admission](downstream-api-admission-2026-11-04.md). |
| D04 (alumina-firmware README) | accepted (local `62d3f6f`) | The architecture introduction and detailed `docs/CURRENT-STATUS.md` are committed. A replay against original base `bdee397` verifies 962 narrative lines unchanged after relative-link rebasing and resolves every file target. Other pending contributor/planning documents remain untouched. No firmware behavior or hardware validation is claimed; see the [admission](downstream-api-admission-2026-11-04.md). |
| D04 (integrated BRep entry points) | locally qualified documentation; publication pending | The root README now identifies implemented selected STEP exchange. HyperBRep and IO introductions separate system/ownership, build/use and current boundaries from certification histories. Their 850/261 original lines are retained in explicitly historical snapshots with relative links rebased; the existing support/implementation and STEP mapping records remain the current authorities. All five original Rust example blocks are retained byte-for-byte, including IO's README-driven doctests. Preservation/link/budget and package-file checks pass. On committed `4b8f9e257a`, all four original IO doctests, both root README tests, both native examples and strict all-feature rustdoc completed with all 3,840 source inputs unchanged (consumer log SHA-256 `84db80e7fc29f19e5a64f6f3513d3bc86f71a3e2e45e639e0d974470950e83f8`). Archive/index closeout and a fresh full pre-push gate for the integrated source remain pending. This is documentation work, not a new geometry or performance claim. |
| D03 (exact SVG rectangle ingress) | locally qualified; integrated publication pending | The [rectangle continuation](svg-exact-native-attributes-2026-11-09.md#exact-unrounded-rectangle-attributes) retains exact unrounded fields and endpoint sums, validates dimensions/radii before empty selection, and preserves valid hidden rounded results without sampled materialization. Five original assertion failures and a further hidden-rounded failure are reproduced; all seven corrected owner tests and eight rounded controls pass in their stated finite scope. On `0963cec833`, the affected owner/root run passes 132 parent reports and seventeen selected rows, MSRV passes 26 maintained bodies, pure WASM passes 78 executions, and root/BRep/IO passes 1,013 reports; strict owner/root lint and formatting pass. The seven new rows preserve all 449 prior objects. These are finite local receipts with exact committed-source attribution. Archive/index closeout and a fresh full pre-push gate on the integrated source remain pending. Visible rounded analytic fidelity and general closure/resource bounds remain open. |

## Suggested order

1. Fix the hyperdrc tokenizer (K02, correctness) and run the K06 benchmark. The
   benchmark is cheap, and its answer changes the scope of K04.
2. K01 on `bezier_offset` alone. It is mechanical and easy to verify, and it
   makes K04 easier to see.
3. U02 and U03, additive alongside the existing API, then U01 and U04 with
   deprecations.
4. K02, K03, K07 and K08, independently.
5. K04 and K05 under P05's gates. U06 prototype with D02 as its first consumer.

Record each package's outcome here as **accepted**, **rejected**, **not
applicable** or **deferred**, with its evidence. This follows the
[research ledger](../research-program.md) convention, so a rejected lead is not
proposed again.
