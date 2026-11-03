# Simplification and modeling-API program (November 2026)

Status: **planning**. Nothing here is implemented. This program records an
external review received on 3 November 2026, the claims that were checked
against the tree, and the work packages derived from it. It sits under the
same acceptance contract as every other change. That means
[`properties.md`](../../properties.md), [`docs/admission.md`](../admission.md)
and the repository [agent rules](../../AGENTS.md), with the governing order:

**exactness > completeness > performance > memory > binary size > source size**

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
