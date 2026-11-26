# Repository Rustdoc audit

The requested scope is every line of every repository file, with accurate
Rustdoc for every public and private Rust API item. This audit is **in progress**;
no repository-wide coverage claim has been established.

Baseline: `8c22379b75982886697f607ffc09891f020794a2`, branch `rustdoc`.
`git ls-files` reports 4,492 tracked files, including 2,930 `*.rs` files at
baseline. These are inventory counts, not review or documentation coverage.
Nested workspaces, excluded packages, binaries, examples, tests, benches, fuzz
harnesses, macros, and configuration-dependent APIs remain in scope. Non-Rust
files must also be reviewed for their role in the codebase and documentation.

## Acceptance and review method

Owning layers retain their existing mathematical authority. This documentation
work follows P37 (accurate scope and API claims), with each batch naming the
specific properties whose contracts it explains. Missing or misleading Rustdoc
is the initial acceptance witness; source inspection and rendered private docs
are required, not merely a clean public `missing_docs` lint. Inventory and
linting cannot substitute for reading implementations and consumers.

Track whole-file reviews explicitly below; partial reads do not earn whole-file
credit. Review all declarations, including private fields and test helpers.
Check actual domains, failure paths, certainty, provenance, and resource limits.
Keep changes to documentation unless a separately admitted implementation fix
is needed. Check private-doc generation for affected configurations and inspect
rendered items. For this documentation-only session, the repository owner explicitly waived
full workflow checks on 26 November 2026. Use focused documentation and formatting
checks before committing and pushing; this exception does not change the standing
repository rules for later sessions or authorize implementation edits.

## Reviewed Rust files

| File | Review and change | Remaining validation |
| --- | --- | --- |
| `tests/sweep_frames.rs` | Entire file; documented all 21 function/constant declarations, exact rational frame invariants, finite helix/grouping oracles, typed failures and terminal certainty. | Source, formatting and comment-only checks passed; runtime assertions and timing diagnostics unchanged, not rerun. |
| `tests/exact_profile_sweeps.rs` | Entire file; documented all 50 function/constant/type declarations, exact prism fixtures, signed revolution phases, independent winding/sector oracles, policy/composition and qualified STL replay. | Source, formatting and comment-only checks passed; assertions and optional STL branch unchanged, not rerun. |
| `tests/mixed_modeling_profiles.rs` | Entire file; documented numeric/contour fixtures, consumer dispatch, exact-member measures, mixed holes/spans and repeated thin-wall cuts. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/exact_path_failures.rs` | Entire file; documented control-net helpers, native causes, display-count invariance, reversal, exact area and empty-region witnesses. Clarified the limited diagnostic evidence in the historical symbolic-endpoint test. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/hilbert_trimming.rs` | Entire file; documented exact run helpers, axis-aligned simplification, source intervals/contact replay, policy transitions and algebraic trim reuse. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/gear_airfoil_failures.rs` | Entire file; documented numeric fixtures, profile/volume helpers and every mechanical-profile failure, certainty, composition and original-resolution witness. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/curve_constructor_failures.rs` | Entire file; documented adapter errors, degenerate/invalid companions, exact area/volume witnesses, policy transitions and byte-capacity failures. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/sampled_profile_failures.rs` | Entire file; documented compatibility trait, helpers, fixture macro and generated tests, error/area/composition witnesses and optional artifacts/interchange. | Source, formatting and comment-only checks passed; tests and optional serialization branches not executed. |
| `tests/slice_projection_contract.rs` | Entire file; documented the compatibility trait, helpers and all section/projection witnesses. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/minkowski_contracts.rs` | Entire file; documented helpers, topology/composition/policy/resource witnesses and limits of volume/probe evidence. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `src/curve/mod.rs` | Entire file; native and optional module exports reviewed; existing module Rustdoc is accurate. No edit needed. | Curve private-doc build passed; optional implementations remain separately in scope. |
| `src/curve/native.rs` | **Partial:** file start through `extrude_twisted`, inclusive, plus the complete `native_boundary_source_probe` module and the sweep-related test/helper items recorded below; frame construction, closure seam, policy/error flow and twist validation documented. Resume at `metaballs`; most of the main test module remains pending. | Curve private-doc build passed; public/private pages inspected. The remainder is not credited as reviewed. |
| `src/solid.rs` | Entire file, cumulatively across the recorded primitive, transform, sampling, distribution and final projection/loft batches; all private helpers, trait implementation methods and tests reviewed/documented. | Private docs passed in recorded minimal and gated configurations, including `curve,attributed,bevymesh`; source-only review of tests, runtime checks waived. |
| `tests/polyhedral_construction.rs` | Entire file; documented all helpers/witnesses and corrected the twisted-square fixture's rotation comment to 45 degrees. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `crates/hypermesh/src/mesh/concatenation.rs` | Entire file including tests; documented capacity helper, fixtures and proof/identity witnesses; existing public contract reviewed. | Hypermesh private docs passed; helper page inspected; test bodies unchanged, not rerun. |
| `tests/distribution_contracts.rs` | Entire file; documented every helper and admission, placement, certainty, capacity, multiplicity and cache witness. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/native_queries.rs` | Entire file; documented ray parameter, copy-count and hull-count expectations with their scope. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `src/implicit/sdf.rs` | Entire file; documented diagnostic fields, strict preview storage, indexed sampling and private grid helpers. | Private docs passed with `sdf`, `metaballs` and both; generated public/private contracts inspected. |
| `src/implicit/metaballs.rs` | Entire file; documented report fields, bordered sampling, empty behavior and private transfer/bounds helpers. | Private docs passed with `metaballs` and `sdf,metaballs`; generated pages inspected. |
| `src/implicit/tpms.rs` | Entire file including local extraction helpers/constants and tests; documented field formulas, band units, sampling and cache invariants. | Private docs passed with `sdf` and `sdf,metaballs`; local items/tests reviewed in source. |
| `src/implicit/mod.rs` | Entire file; feature gates and existing module Rustdoc agree with the three owning implementations. No edit needed. | Individual and combined feature doc builds passed. |
| `tests/metaballs_diagnostics.rs` | Entire file; documented all fixture helpers and diagnostic/influence/grid witnesses. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/sdf_tpms_diagnostics.rs` | Entire file; documented exact plane expectations, preview comparisons, downstream admission and diagnostic limitations. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/transform_failures.rs` | Entire file; documented every helper and transform failure/composition witness. | Source review, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/primitive_failures.rs` | Entire file; documented all primitive-failure and empty-composition helpers/witnesses. | Source review, formatting and comment-only checks passed; test bodies unchanged, not rerun. |
| `src/solid/cache.rs` | Entire file; documented thread-local state, default implementation and local restoration guard; clarified receipt reuse. | Private docs passed with and without `curve`; local guard reviewed in source. |
| `src/solid/cache/storage.rs` | Entire file; documented all private keys, fields, retention helpers and completed-result contracts. | Private pages with `curve` checked; minimal build also passed. |
| `tests/solid_cache.rs` | Entire file; documented constants, comparison/uncertainty helpers and all cache regression witnesses. | Formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `src/attributed.rs` | Entire file including tests; documented alignment-only admission, equality, private fields/helpers and shared export cache. | Private docs passed with `attributed,curve`; generated sidecar page checked. |
| `src/hyper_math.rs` | Entire file; documented internal conversion implementations, strict predicate helpers and diagnostic-erasing convenience semantics. | Private docs passed with `attributed,curve` and no default features; generated private pages checked. |
| `src/solid_expr.rs` | Entire file; documented deferred operators, transform-dependent memoization, private lowering state, error paths and resource limits. | Private docs passed with `curve,dispatch-trace` and no default features; generated private/operator pages checked. |
| `tests/solid_expr.rs` | Entire file including dispatch and ignored timing witnesses; documented helpers and corrected independence claims. | Formatting and comment-only comparison passed; test bodies unchanged, not rerun. |
| `src/session.rs` | Entire file; documented conversion implementations/macro, private state, policy exceptions and receipt scope. | Private docs passed with and without `curve`; generated conversion docs checked. |
| `tests/session.rs` | Entire file; documented every test/helper according to actual assertions. | Comment-only comparison and formatting passed; tests source-reviewed, not rerun. |
| `tests/support/uncertainty.rs` | Entire file; existing module/helper Rustdoc matches retained witness and independent unresolved-zero assertion. No edits needed. | Source review only; no current execution claim. |
| `src/errors.rs` | Entire file and classification match arms; documented payload fields, categories and private classifiers. | Private docs passed with all error-gating features and no default features; formatting passed. |
| `src/lib.rs` | Entire file; checked feature overview and policy uses, added private API build guidance and private constant docs. | Crate page and private policy page checked; broader gated API review remains in owning modules. |
| `src/adapter/mesh.rs` | Entire file; documented scalar facade operations, graphics cache, exports and private helpers. | Private docs passed with and without `curve`, with broken links denied. |
| `src/adapter/curve.rs` | Entire file; documented region operations, sampled profile/revolution boundaries and private conversion helper. | Private docs passed with `curve`, with broken links denied. |
| `src/adapter/scalar.rs` | Entire file and conversion implementations; documented all declarations and native error distinctions. | Private docs passed with and without `curve`, with broken links denied. |
| `src/adapter/mod.rs` | Entire file including tests; documented bound conversion, fields and test intent. | Private docs passed with and without `curve`, with broken links denied. |
| `src/context.rs` | Entire file, including tests. Documented every previously undocumented private declaration; corrected approximate-policy wording and documented receipt loss at `into_value`. | Private docs passed with no default features and with `curve`; broader feature checks pending; full workflow checks waived for this docs-only session. |

### Context batch, 26 November 2026

Owner: CSGRS policy/receipt bridge, P8/P9/P19/P25/P30/P37. Domain: both predicate
policies, generic completed values, nested kernel receipts, and mesh dependency
qualifications. Missing witness at baseline: `GeometryDecisions`, its fields and
most methods, `GeometryOutcome::new`, and test declarations lacked Rustdoc.
`APPROXIMATE_512` said “certified topology” without explaining terminal consumption.

Read surrounding implementation in this file and inspected policy definitions in
`hyperlimit/src/predicate.rs`, construction evidence in
`hypermesh/src/construction.rs`, mesh evidence transport in
`hypermesh/src/mesh.rs`, `hypercurve::evaluate_under`, and Boolean/transform
consumers in `src/solid.rs`. Those supporting partial reads are not whole-file
audit credit. Also read `src/lib.rs`, `src/adapter/mod.rs`, and
`src/adapter/scalar.rs` to identify the next documentation gaps; their item-level
corrections and verification remain pending.

The diff changes comments only in Rust source. A comparison removing comment
lines confirms all non-comment lines in `src/context.rs` match baseline, so no
predicate ordering, runtime contract, dispatch, or representation change is
claimed. No performance or universal correctness claim is made.

Checks after edits:

- `cargo fmt --all -- --check`: passed.
- `git diff --check`: passed.
- Non-comment-line baseline comparison for `src/context.rs`: passed.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0 cargo doc --locked -p csgrs --no-default-features --document-private-items --no-deps --offline`: passed without warnings; log `/tmp/csgrs-rustdoc-doc.log`.

- The same documentation command with `--features curve` and
  `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`: passed without warnings;
  log `/tmp/csgrs-rustdoc-curve-doc.log`.
- Parsed generated HTML for `GeometryDecisions`, `GeometryContext`, and
  `GeometryOutcome`: confirmed private methods/fields and revised contract text
  appear in the pages.

Build artifacts are isolated in `/tmp/csgrs-rustdoc-target` for reuse and eventual
cleanup. No artifacts or raw logs are committed. Full pre-push workflow checks
were not run. The owner subsequently waived them for this docs-only session.

### Scalar adapter batch, 26 November 2026

Owner: CSGRS scalar boundary, P1/P3/P6/P25/P26/P29/P37. Domain: native `Real`,
finite/nonfinite primitive floats, signed integers, coordinate arrays, converted
bounds, and native error forwarding. Baseline for this batch: `782f2d9fc`.
Missing witness: most scalar declarations, trait members, implementation methods,
coordinate helpers, bounding-box fields and test helpers had no Rustdoc.

Read every line of `src/adapter/scalar.rs` and `src/adapter/mod.rs`. Cross-checked
native float import/export and rational-to-integer conversion in Hyperreal,
structural `exact_rational` recognition, modeling error variants, and mesh/curve
bound and graphics consumers. These supporting partial reads do not earn
whole-file credit. Documented exact binary float ingress, lossy egress, distinct
integer failures, preserved error sources, ordered component conversion, and
lack of outward-enclosure guarantees for rounded bounds. Test docs describe
actual assertions rather than inferring extra guarantees from test names.

Validation after edits:

- `git diff --check`: passed.
- Non-comment content comparison against the batch baseline: passed after
  normalizing whitespace and optional trailing tuple commas introduced to
  document variant fields. No executable behavior changed.
- Private Rustdoc with `curve` and broken links denied: passed without warnings.
  Command: `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`.
- Parsed generated HTML for the scalar trait, integer strategy, bounds type,
  private bound converter and private XY importer: expected contract text present.
- The same private-doc command without `--features curve`: passed without warnings.
- `cargo fmt --all -- --check`: passed.

Logs remain under `/tmp/csgrs-rustdoc-adapter-*.log`; reused artifacts remain under
`/tmp/csgrs-rustdoc-target`. Full workflow checks waived explicitly by the owner
for this documentation-only session; no runtime or performance claims are made.

### Mesh and curve adapter batch, 26 November 2026

Owner: CSGRS modeling and display adapters, P6/P8/P9/P19/P25/P26/P29/P37.
Baseline: `14ab38d45`. Domain: native mesh/region wrappers, primitive scalar
construction, Booleans, transforms, bounds, extrusion, sampled revolution,
finite profiles, graphics buffers and cached exports. Missing witness: most
facade methods and private storage/conversion declarations lacked Rustdoc;
context method summaries did not identify the operation or receipt scope.

Read every line of `src/adapter/mesh.rs` and `src/adapter/curve.rs`. Cross-checked
constructor validation, face triangulation, empty handling, transforms,
bounds, Boolean forwarding, finite-profile projection, extrusion and revolution
in the CSG grammar. Inspected Hypercurve's principal policy scope and Boolean
entry point, and Hypermesh's exact graphics export and normal construction.
Supporting reads are partial, not whole-file review credit.

Documented the distinction between primitive-scalar conversion and authoritative
native geometry, local predicate receipts and inherited qualification, winding
reversal and set complement, polygon triangulation and whole-mesh validation,
swept and regularized sampled revolution. The finite-profile path explicitly
projects to `f64` even with `RawReal`; converting those points back cannot recover
the original exact source. Bounds may be loose for algebraic trims and rounded
exports need not remain outward enclosures. The graphics cache documentation
specifies allocation/type keys, retained ownership, per-thread FIFO lifetime,
eight-entry capacity, failure handling, lack of a byte cap/reset, and the stable
conversion assumption for custom adapters.

Validation after final edits:

- Private Rustdoc with `curve`, broken links denied: passed without warnings.
  Command: `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`.
- The same command without `--features curve`: passed without warnings.
- Parsed generated HTML for both facade types, `RegionProfile`, private
  `finite2_to_scalar`, `point3_to_scalar`, `CachedGraphicsMesh`, and the
  macro-generated thread-local cache page: expected contract text present.
- Compared all nonblank, non-comment Rust lines against the batch baseline:
  identical in both files. No executable changes or performance claims.
- `git diff --check`: passed after removing trailing whitespace in new docs.
- Final `cargo fmt --all -- --check`: passed.

Logs: `/tmp/csgrs-rustdoc-mesh-curve-doc.log` and
`/tmp/csgrs-rustdoc-mesh-minimal-doc.log`. Reused build tree remains about 97 MiB;
no generated artifacts are tracked. Full workflow checks remain waived by the
owner for this documentation-only session.

### Error API and crate root batch, 26 November 2026

Owner: CSGRS error and binding boundary, P6/P8/P30/P37. Baseline: `e5e8487f1`.
Domain: every modeling error variant and classification branch, including optional
font and sampled-field diagnostics. Missing witness: variant payload fields,
most category variants, `kind`, and both private classifiers lacked Rustdoc.

Read every line of `src/errors.rs` and `src/lib.rs`. Inspected session aliases,
JavaScript category export, expression and scalar wrappers, allocation helpers,
curve diagnostics, and the installed thiserror source-chain expansion. Checked
crate-root feature declarations against Cargo configuration and boundary module
exports. Supporting files are not credited as fully audited. `src/session.rs`
has only a targeted alias-doc correction; its full review remains pending.

Documented actual classification semantics, including the coarse resource bucket
that currently includes cancellation, capacity overflow and several allocation
failures. `ResourceUnavailable` is used for explicit curve storage uncertainty,
not every failed allocation. Documented that transparent scalar wrappers forward
the payload's source chain and polynomial topology retains its payload without
exposing it as `Error::source()`. Corrected the session error-alias claim to agree.
These are existing implementation limits, not algorithm fixes or claims of
compliance with the stronger failure/resource target contract. Future changes to
classification or source-chain behavior require separate admission.

Crate-root docs now describe the benchmark feature bundle, private-documentation
command, strict helper policy, and test-only mesh context. Test-only constant
documentation was reviewed in source; ordinary Rustdoc builds exclude test cfg.

Validation after edits:

- `git diff --check`: passed.
- Code comparison against baseline, removing comments and normalizing whitespace
  and optional trailing record/tuple commas: unchanged for all three Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,truetype-text,hershey-text,sdf,metaballs
  --document-private-items --no-deps --offline`: passed without warnings.
  This enables every feature-gated variant in `src/errors.rs`.
- Generated HTML checked for error payloads/categories, both private classifier
  pages, private policy constant, root build guidance, and session error alias.
- The same private-doc command without the feature list: passed without warnings.
- `cargo fmt --all -- --check`: initial failure required splitting attributes onto
  their own lines beside documented tuple fields; final check passed after that
  formatting correction.

Logs remain in `/tmp/csgrs-rustdoc-errors-doc.log` and
`/tmp/csgrs-rustdoc-errors-minimal-doc.log`. Source behavior, category mapping and
error-source attributes are unchanged. Full workflow checks remain waived for
this docs-only session; no runtime or performance claim is made.

### Modeling session batch, 26 November 2026

Owner: CSGRS input conversion and receipt orchestration, P1/P6/P8/P9/P19/P25/P37.
Baseline: `8ca463916`. Domain: fixed-width integers, exact native scalars, finite
float imports, rational/scientific text, both policies, successful and failed
callbacks, native mesh/curve operations, and external receipts. Missing witness:
conversion implementations and macro-generated methods, private fields, and
regression helpers lacked Rustdoc. Existing session summaries claimed receipts
for every operation and policy selection once without identifying strict solid
constructor exceptions. Extrusion documentation described only +Z although signed
nonzero heights are accepted.

Read every line of `src/session.rs`, `tests/session.rs`, and
`tests/support/uncertainty.rs`. Inspected native scalar conversion/parsing,
strict primitive construction, reflection, extrusion, and expression lowering
and evaluation. Those supporting partial reads do not earn whole-file credit.
The uncertainty helper was already documented consistently with its assertions;
no change was needed. Existing tests compare wrapper outputs, receipt transitions,
input rational forms and typed errors; their source review is not independent
geometric completion evidence or a claim of execution on this revision.

Documented that `run` records only successful returned receipts; prior records
survive a later error. `record` trusts external receipts without policy checking
or value inspection. Native modeling paths carry mesh history separately, but
generic callbacks are responsible for their values' evidence. The four solid
constructors use strict grammar functions directly and do not update the session
accumulator. The docs identify this current limitation without modifying policy
behavior. Sampling fidelity, callback success, and accumulated numerical certainty
remain separate claims. Expression evaluation docs distinguish a shared nonempty
arrangement from the all-empty shortcut and transform-dependent subtree reuse.

Validation after edits:

- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated session module, `Csg`, `IntoReal`, private integer macro, and decimal
  parser pages contain the new contracts; macro-generated implementation docs
  appear on the trait page.
- `cargo fmt --all -- --check`: passed. After the last module wording edit,
  `rustfmt --check --edition 2024 src/session.rs tests/session.rs` also passed.
- `git diff --check`: passed. Nonblank, non-comment Rust lines match the baseline
  in both edited source files. Tests and example bodies remain unchanged.

Logs: `/tmp/csgrs-rustdoc-session-doc.log` and
`/tmp/csgrs-rustdoc-session-minimal-doc.log`. Reused build tree is about 124 MiB.
Full workflow/runtime checks remain waived for this docs-only session. Test-only
Rustdoc was reviewed in source, not claimed as rendered by the library doc build.

### Lazy expression batch, 26 November 2026

Owner: CSGRS expression lowering into the Hypermesh Boolean program,
P6/P8/P9/P19/P20/P21/P25/P37. Baseline: `48d00109e`. Domain: native mesh leaves,
shared subtrees, ordered transform chains, all four Boolean operators, canonical
empty and malformed leaves, both policies and located/unlocated failures.
Missing witness: private node variants/payloads, lowering state, path storage,
helpers, operator macro and generated members lacked Rustdoc. Existing summaries
omitted transform-dependent reuse and the all-empty no-arrangement shortcut.

Read every line of `src/solid_expr.rs` and `tests/solid_expr.rs`. Cross-checked
native transform admission and receipt handling, the kernel Boolean entry point,
and existing session/uncertainty support reviewed in prior batches. Supporting
kernel/grammar reads are not full-file audit credit.

Documented allocation identity versus set equality, memo lifetime and keys,
inner-to-outer transform application, left-first lowering, false-node reuse,
ancestry retention, and first-occurrence error paths. The affine distribution
identity is stated with its preconditions; projective leaf admission may fail
before later Boolean selection. Recorded the existing recursive/allocating
lowerer and its `u32` indexing panics without implying typed exhaustion or an
expression-level resource bound. No algorithm, predicate order or bound changed.

Documented each regression helper and test's actual role. Corrected the claim
that eager/lazy comparisons supply an independent set oracle: both paths and the
symmetric-difference helper use the native kernel. Matching absent rational normal
forms is not a volume proof. The drilled fixture has a separate closed-form
volume expectation; the dispatch fixture asserts one versus four starts. The
ignored timing fixture reports five-run median/min/max without a cache reset or
performance threshold. These are source observations, not new execution or
performance claims.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid_expr.rs tests/solid_expr.rs`: passed.
- `git diff --check`: passed.
- Compared edited Rust against baseline after removing comment lines and
  normalizing whitespace and optional trailing commas: executable content
  unchanged, including every assertion and ignored/feature gate.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,dispatch-trace --document-private-items
  --no-deps --offline`: passed without warnings. The same command without the
  feature list passed without warnings.
- Generated module, public expression, private node/transform/program/location,
  dispatch helper and operator macro pages contain the new contracts. Generated
  owned-operator method documentation appears on the public expression page.

Logs: `/tmp/csgrs-rustdoc-expressions-doc.log` and
`/tmp/csgrs-rustdoc-expressions-minimal-doc.log`; artifacts remain in the reused
`/tmp/csgrs-rustdoc-target`, with no generated output tracked.

Full workflow/runtime checks remain waived by the owner for this docs-only
session. Test-only Rustdoc is reviewed in source; a library documentation build
does not render the integration tests or execute their assertions.

### Attribute sidecar and scalar helpers batch, 26 November 2026

Owner: CSGRS boundary attributes and internal scalar conveniences,
P1/P3/P4/P6/P8/P19/P25/P26/P29/P37. Baseline: `4faf22d51`. Domain: aligned
metadata, optional authored normals, successful/failed/concurrent graphics
exports, native and primitive scalars, strict predicates and exact vector/point
construction. Missing witness: sidecar fields, equality, uncached exporter and
tests, plus every internal scalar-helper declaration, lacked Rustdoc. Existing
sidecar summaries did not distinguish count validation from geometric validity.

Read every line of `src/attributed.rs` and `src/hyper_math.rs`. Inspected native
mesh equality, point/vector conversion and interpolation, exact GPU row assembly,
OBJ sidecar import, helper call sites, and cache recovery/concurrency/authored-normal
regression witnesses in `tests/native_boundary_failures.rs`. Those partial reads
are not whole-file review credit or claims that the tests were run.

Documented unchanged geometry/attribute ownership, cache sharing by clone and
metadata mapping, first-success publication, retry and racing-success behavior,
lifetime, lack of eviction/reset/byte cap, and separation from modeling authority.
Authored normals are copied without normalization or degeneracy checks; geometry
validity is not implied by successful authored-row export. Equality ignores
construction evidence and the export cache, following native buffer equality.

Internal scalar docs distinguish exact ingress from lossy export, the private
conversion trait from the public session trait, and strict positive evidence from
absence of proof. `hreal_gt_f64` and triangle nondegeneracy return false for
undecided cases; comparison and arithmetic helpers erase particular diagnostics
into `None`; several wrappers always return `Some`. Their documentation now
states this without changing behavior or presenting the wrappers as complete
failure/provenance-preserving APIs. The test-only threshold is the exact value
of a binary float, not an exact decimal or modeling coincidence rule.

Validation after edits:

- `rustfmt --check --edition 2024 src/attributed.rs src/hyper_math.rs`: passed.
- `git diff --check`: passed.
- Nonblank, non-comment line comparison against baseline: identical for both
  edited Rust files, including test assertions and cache control flow.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features attributed,curve --document-private-items
  --no-deps --offline`: passed without warnings.
- Generated attributed sidecar page includes private state and exporter docs;
  private scalar module, conversion trait and representative predicate/arithmetic
  helper pages contain the revised contracts.
- The same private-doc command without the feature list: passed without warnings.

Logs remain at `/tmp/csgrs-rustdoc-attributes-math-doc.log` and
`/tmp/csgrs-rustdoc-math-minimal-doc.log`. Test-only helper documentation was
reviewed in source. Full runtime/workflow checks remain waived for this docs-only
session; no correctness or performance result beyond documentation validation is
claimed.

### Modeling cache batch, 26 November 2026

Owner: CSGRS completed-operation retention, P4/P5/P8/P9/P19/P20/P25/P37.
Baseline: `281c53cd0`. Domain: strict primitive keys, ordered merges, distribution
layouts, curve projections/slices, exact policy keys, scoped entry limits,
thread-local ownership, mutation/evidence changes and repeated requests. Missing
witness: private storage variants and fields, matching/retention helpers, local
scope guard, and test declarations lacked Rustdoc.

Read every line of `src/solid/cache.rs`, `src/solid/cache/storage.rs`, and
`tests/solid_cache.rs`. Cross-checked primitive/distribution key construction and
retention call sites in `src/solid.rs`, including early successful empty returns
that need not insert an entry. Those caller reads are partial and do not earn
whole-file credit.

Documented structural parameter equality, owned allocation identities and
construction snapshots, exact context matching, first matching FIFO lookup,
nonrefreshing hits, entry lifetimes, active insertion policy and no error caching.
Private insertion helpers trust caller-completed outcomes and do not revalidate
geometry. Zero limits skip entry construction; policy shrinking trims oldest
entries, with vector capacity released at zero and otherwise available for reuse.
No limit is a retained-byte or total-work budget. A scope guard restores prior
limits during unwinding, without resurrecting evicted entries. A cache hit
returns its stored receipt; rebuilding can use stronger scalar proofs acquired
since insertion, consistent with P4's distinction between proof reuse and value
semantics. This clarification does not authorize weaker policies or altered values.

Test docs describe row/receipt comparisons separately from construction ancestry
and set equivalence. Witnesses cover FIFO, nested unwinding, workers, source order
and mutation, policy separation, and all enabled cache categories. Source review
is not a claim of executing these tests or proving universal cache correctness.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid/cache.rs src/solid/cache/storage.rs
  tests/solid_cache.rs`: passed.
- `git diff --check`: passed.
- Baseline comparison after removing comment lines and normalizing whitespace
  and optional trailing commas: code identical in all three edited Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings.
- Generated private key enums, mesh storage, cache state, retention/lookup helpers,
  thread-local state and public policy scope pages contain the new contracts.
  Function-local guard and integration-test docs were reviewed in source rather
  than claimed as rendered by the library build.
- The same private-doc command without `--features curve`: passed without warnings.

Logs: `/tmp/csgrs-rustdoc-cache-doc.log` and
`/tmp/csgrs-rustdoc-cache-minimal-doc.log`; existing temporary build storage is
reused. Full workflow/runtime checks remain waived for this docs-only session.
No predicate ordering, cache key, capacity, test assertion or source behavior was
changed, and no new runtime/performance claim is made.

### Solid primitives, partial-file batch, 26 November 2026

Owner: CSGRS native primitive grammar, P1/P6/P17/P18/P19/P20/P25/P26/P37.
Baseline: `4a75b574b`. **Partial file:** reviewed `src/solid.rs` from its module
header through the end of `torus_uncached` (baseline lines 1–869). Whole-file
credit remains open; resume at `teardrop_cylinder`. Read every line of
`tests/primitive_failures.rs`. Inspected native orthonormal-frame construction;
that supporting read does not earn full kernel-file credit.

Missing witness: private strict sign, parameter, capacity, reservation, sampled
circle, indexed assembly and uncached primitive helpers lacked Rustdoc. Public
primitive summaries omitted important domains, sample conventions and failure
rules. Documented all declarations in this reviewed section, including source
validation before empty-collapse shortcuts, checked inline bytes versus nested
allocation cost, and `indexed_convex`'s absence of convexity validation.

Specified sphere Y-axis poles and exact angular samples, signed cylinder/frustum
height, cone tips, octahedron/icosahedron circumradius, arrow proportions and
mirrored-tip placement, and the sampled ring-torus domain. Exact coordinates,
polyhedral geometry, sampling history and analytic fidelity remain distinct.
Arrow orientation propagation is described as its actual construction sequence,
not a general topology certificate; secondary adjacency allocations remain
outside the primary reservation checks. No new resource guarantee is asserted.

Regression documentation describes negative/undecided input, resolution/capacity,
zero-size validation, typed adapter failures, signed-height volume, cache recovery
and further operations on canonical empties. Their source inspection establishes
what is asserted, not that this revision passed runtime tests or proves closure.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid.rs tests/primitive_failures.rs`: passed.
- `git diff --check`: passed; every nonblank, non-comment source line matches
  baseline in both edited files, including all assertions and orientation logic.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated sphere, cylinder, frustum, arrow, torus, private capacity, sample-table,
  indexed-convex and uncached icosahedron pages contain the revised contracts.

Logs: `/tmp/csgrs-rustdoc-primitives-doc.log` and
`/tmp/csgrs-rustdoc-primitives-minimal-doc.log`. Full workflow/runtime checks remain
waived for this docs-only session. No executable behavior or performance claim
changed; test-only docs were reviewed in source.

### Solid construction and transforms, partial-file batch, 26 November 2026

Owner: CSGRS native solid grammar and its Hypertri/Hypermesh policy bridges,
P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `f85b58f70`.
**Partial file:** reviewed `src/solid.rs` from `teardrop_cylinder` through `inverse`
(baseline lines 995–1528), including each body, guard and receipt path. Combined
with the previous batch, review reaches the sampled-field options boundary;
whole-file credit remains open. Read every line of `tests/transform_failures.rs`.
Supporting reads covered the relevant curve constructors/extruders, Hypertri
simple-ring contract, Hypermesh transforms, reflection, subdivision, finite
materialization, winding reversal and indexed edge check. Those reads do not
earn whole-file credit for their owning modules.

Missing witnesses: private planar projection/triangulation and orientation-aware
transform helpers lacked Rustdoc; several public summaries overstated or omitted
their actual behavior. `renormalized` clones the mesh without coordinate rebuild.
`materialize_finite` performs lossy binary64 projection before exact promotion,
retaining coordinate-projection provenance without an error or topology bound.
`is_closed_manifold` checks indexed edge pairing, without geometric tests or
vertex-link validation. Documented these limitations without changing behavior.

Documented teardrop placement and sampled profile, gear profile delegation,
signed nonzero extrusion thickness, and the use of `helix_angle_degrees` as total
twist rather than conversion from a tooth helix angle. Polyhedron docs distinguish
per-face admission from closed-solid admission, distinct closing indices from
repeated coordinates, and canonical empty geometry from position-only buffers.
Projection docs distinguish policy classifications from proofs and retain the
support-search/decision order. Boolean docs explain the two-empty shortcut and
inherited qualifications; transform docs specify determinant checks, index
validation, per-triangle projective scale signs, infinity failures, Euler order,
reflection and winding correction. Subdivision docs describe exact shared indexed
midpoints and the absence of a total resource budget.

Transform witness documentation describes actual independent coordinate/winding
and volume expectations, policy retry, typed error/source preservation, valid
empty composition and invalidated indices. No test assertion was changed and no
runtime execution or universal correctness claim follows from this source review.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid.rs tests/transform_failures.rs`: passed.
- `git diff --check`: passed. Every nonblank, non-comment source line matches
  baseline in both Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated teardrop, helical gear, polyhedron, transform, normalization,
  materialization and edge-check pages contain the revised contracts. Private
  triangulation, projection-axis, ring-projection and orientation-helper pages
  also contain the new Rustdoc. Integration-test docs were reviewed in source.

Logs: `/tmp/csgrs-rustdoc-transforms-doc.log` and
`/tmp/csgrs-rustdoc-transforms-minimal-doc.log`. The existing temporary build
directory is reused. Full workflow/runtime checks remain waived for this docs-only
session. No executable behavior, predicate order or performance claim changed.

### Sampled fields and bounds, partial-solid/full-implicit batch, 26 November 2026

Owner: CSGRS sampled implicit constructors and their Hypermesh/Hypersdf bridges,
P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `6beb18849`.
**Partial solid file:** reviewed `src/solid.rs` from `SampleOptions` through
`float_with_context`, including feature gates and each wrapper body. Resume at
`merge`; whole-file credit remains open. Read every line of `src/implicit/mod.rs`,
`sdf.rs`, `metaballs.rs`, `tpms.rs`, `tests/metaballs_diagnostics.rs` and
`tests/sdf_tpms_diagnostics.rs`. Supporting reads of Hypermesh bounds/Surface Nets,
Hypersdf metaball fields and selected failure-test sections do not earn whole-file
credit for those modules.

Missing witnesses: diagnostic fields, sampler/grid/preview helpers, TPMS caches,
local tetrahedral helpers/constants and test declarations lacked Rustdoc. Public
summaries blurred policy-selected counts with strict preview proofs and omitted
grid-border/empty rules. TPMS prose asserted mathematical surface properties not
established by its trigonometric formulas or implementation. Replaced those claims
with the modeled formulas and extraction behavior, without changing any algorithm
or reducing the repository's target contract.

Documented exact sample placement, callback order, unshifted extrema versus
iso-shifted sign buckets, and the possibility that diagnostic comparisons fail
or consume additional approximate certainty. Expression previews require strict
signs independently of meshing policy, retain unavailable values explicitly,
and use clamped F32 magnitudes stored as f64 without driving native geometry.
Compatibility counters remain zero on successful native extraction; detected
degeneracy and undecided samples return failures instead of partial output.

Metaball docs specify requested/bordered count validation, conservative bounds,
one-cell exterior samples, strict field evaluation and empty-source reports.
TPMS docs specify positive wavelength, field-value band thickness (not Euclidean
distance), bounding-box clipping, half-step exterior sampling, six-tetrahedron
extraction, negative/zero ownership, shared edge crossings and orientation errors.
Per-call trigonometric caches require a fixed coordinate/scale pair per index.
Neither Surface Nets nor the tetrahedral path is documented as independently
certifying continuous-field topology, exact minimality or embedded closed output.
Primary count/reservation checks are not presented as total resource bounds.

Bounds/placement docs cover unused position rows, index validation, certified
cache reuse, local receipt scope and the absence of source construction history
in a bounds value. Centering uses the box midpoint; floating uses minimum Z.
Diagnostic test docs distinguish independently expected rational planes/counts
from shared-path agreement, indexed closure and lossy volume-sign diagnostics.

Validation after edits:

- Targeted `rustfmt --check --edition 2024` on all six edited Rust files: passed.
- `git diff --check`: passed; every nonblank, non-comment source line matches
  baseline across all six files, including tests and local helper bodies.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features sdf,metaballs --document-private-items
  --no-deps --offline`: passed without warnings. Separate `sdf`, `metaballs`,
  and no-feature private-doc builds also passed.
- Generated diagnostic structs/fields, preview/grid storage, sampler, border,
  tetrahedral extraction, TPMS cache, public options/constructors and bounds
  contracts were inspected. Function-local items and test docs were reviewed in
  source; ordinary library Rustdoc does not render them as standalone items.

Logs use the prefix `/tmp/csgrs-rustdoc-sampling-` and suffix `-doc.log`, with
configuration names `sdf`, `metaballs`, `sdf,metaballs` and `minimal` between them.
The shared temporary build directory is about 150 MiB. Full workflow/runtime checks
remain waived for this docs-only session. No executable behavior, predicate order,
test assertion or performance claim changed. A generated-text check was adjusted
for Rustdoc's smart apostrophe; combined pages were regenerated after the minimal
build removed feature-gated output from the shared directory.

### Distribution, queries and concatenation batch, 26 November 2026

Owners: CSGRS distribution/query grammar, Hypermesh structural concatenation, and
the Hyperphysics mass-report boundary; P1/P4/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37.
Baseline: `55a0f4abb`. **Partial solid file:** reviewed `src/solid.rs` from `merge`
through `exact_mass_properties`, including every body and fallback/receipt path.
Resume at `flatten`; later sections were encountered but have not earned audit
credit. Read every line of `crates/hypermesh/src/mesh/concatenation.rs`, including
unit tests, and `tests/distribution_contracts.rs` and `tests/native_queries.rs`.
Supporting reads covered native mesh queries, ray/triangle classification, vector
normalization/angles, Hyperphysics integration/admission and distribution caches;
those owning files remain outside whole-file credit.

Missing witnesses: the distribution admission, buffer and union helpers lacked
Rustdoc, as did the native concatenation capacity helper and test declarations.
Public summaries omitted important placement, fallback and query/error semantics.
Documented row/column order, signed normalized linear spacing, global-Z arc
rotation, angular midpoint for one copy and unreduced sweeps. Canonical-empty and
zero-copy paths still validate their applicable source/direction requirements;
copy-capacity checks happen only after those shortcuts. Cache keys/receipts and
inherited mesh qualifications are described without asserting a total resource
bound.

Disjoint bounds permit structural concatenation, retaining source winding
multiplicity instead of normalizing each boundary. Failed/missing bounds and
undecided overlap hints trigger ordered Boolean fallback; successful hint receipts
already consumed remain in aggregate certainty. Concatenation preserves all rows,
validates indices before rebasing and transfers current closed-PWN facts only when
all nonempty operands supply one. The helper's inline-byte/count checks do not
bound nested scalar or later allocation costs.

Query docs specify ray parameterization, coplanar omission, sorted/deduplicated
contacts, per-segment polyline order and the short-polyline validation shortcut.
Dihedral measurement uses supplied indexed normals without adjacency validation;
its convenience form erases native failures into `None`. Mass-property docs now
state the caller's closed/oriented-geometry obligation, signed integration and
orientation normalization, absent source qualifications and conversion of physics
errors into debug-text `ValidationError::Geometry`. Exact scalar arithmetic alone
does not establish physical validity or preserve typed native error sources.

The first Hypermesh private-doc build reproduced two ambiguous `boolean` links
at this revision, in `src/lib.rs` and `src/winding.rs`. Inspected their surrounding
API context and disambiguated both as function links with `()`. Those targeted
repairs do not establish full-file semantic review of either file. Read the whole
`crates/hypermesh/AGENTS.md`; its full pre-push checks are covered by the owner's
session-specific docs-only waiver, with focused checks used here.

Validation after edits:

- `rustfmt --check --edition 2024 --config skip_children=true` on all six edited
  Rust files: passed. `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in all six Rust files,
  including test assertions, predicate order, fallback branches and cache behavior.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p hypermesh
  --document-private-items --no-deps --offline`: passed without warnings after the
  two function-link repairs.
- Equivalent private-doc builds for `-p csgrs --no-default-features`, with and
  without `--features curve`: passed without warnings.
- Inspected twelve generated public/private contracts for merge, distribution
  helpers/layouts, queries, mass properties and native capacity checking. Both
  repaired links resolve to function pages. Unit/integration-test Rustdoc was
  reviewed in source, not claimed as rendered by ordinary library builds.

Logs: `/tmp/csgrs-rustdoc-concatenation-doc.log`,
`/tmp/csgrs-rustdoc-distribution-minimal-doc.log` and
`/tmp/csgrs-rustdoc-distribution-curve-doc.log`. Shared temporary build storage
is about 160 MiB. Full workflow/runtime checks remain waived for this docs-only
session. No implementation, assertion or performance claim changed.

### Solid projection, hull, loft and fluent API completion, 26 November 2026

Owner: CSGRS solid-to-planar, hull/Minkowski, loft, renderer and fluent API
boundaries; P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `093ec2c88`.
Read every line of `src/solid.rs` from `flatten` through end of file, including
the `SolidExt` implementation and all module tests. Together with the prior
explicit section records, this completes the file's source review. Read every
line of `tests/polyhedral_construction.rs`. Supporting reads covered native
section records/assembly, hull and Minkowski entry paths, source preparation and
pair hulls, GPU normal construction/projection and feature configuration; those
owning modules do not receive whole-file credit here.

Missing witnesses: public summaries omitted projection/winding behavior,
incidence-record loss, hull degeneracy and renderer/visitor panic contracts.
Fluent implementation methods and smoke-test declarations lacked their own
Rustdoc. Documented `SliceResult` component meanings and its currently empty path
list; slicing consumes native incidence records without returning them. Flatten
orients every projected face as material, so reversed source faces do not cancel.
Both operations retain local certainty without carrying mesh construction history
into the planar result, and cache completed source/context-specific outcomes.

Hull docs distinguish all stored positions from represented triangle sets,
including unchecked triangle rows and degenerate empty/lower-dimensional inputs.
Minkowski docs distinguish the represented-solid sum from pairwise stored-position
convexification and explain native default limits, cooperative cancellation and
construction qualifications. Loft docs describe section equality/coalescing,
per-section plane validation, cap selection, fixed side diagonals, orientation,
checked primary buffers and closed-PWN output validation. Approximate coalescing
remains qualified in the receipt and mesh evidence.

Bevy export is documented as lossy `f32` display geometry with flat normals and
panic-on-error behavior. Triangle visitation clones exact positions in authored
order but directly indexes them, so malformed indices can panic. Trait docs
state strict convenience behavior, explicit-context receipts and deferred Boolean
composition; every native implementation method has a scoped delegation contract.
Uniform metadata, exact translation helpers and all unit-test declarations are
documented. Polyhedral test docs identify independent area/volume expectations,
policy transitions and downstream admission separately from universal set proofs.
The existing twisted-square comment incorrectly said a quarter turn; its literal
coordinates encode an eighth turn followed by scaling, so only the comment changed.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/solid.rs tests/polyhedral_construction.rs`:
  passed. `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in both files,
  including tests, feature gates, panic paths and predicate order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,attributed,bevymesh
  --document-private-items --no-deps --offline`: passed without warnings.
  The equivalent no-feature private-doc build passed as well.
- Inspected eleven generated contracts covering the slice alias, projection,
  slicing, Bevy panic/loss behavior, hull and Minkowski distinctions, loft,
  trait visitation, metadata and translation. Unit/integration-test docs remain
  source-reviewed; ordinary library documentation does not render test items.

Logs: `/tmp/csgrs-rustdoc-solid-completion-doc.log` and
`/tmp/csgrs-rustdoc-solid-completion-minimal-doc.log`. Reused build storage grew
to about 502 MiB for the optional Bevy dependency check; `/tmp` has over 125 GiB
available. No separate target tree or runtime test build was created. Full
workflow/runtime checks remain waived for this docs-only session. No executable
behavior, test assertion or performance claim changed. Completion here describes
this file's source audit, not repository-wide Rustdoc coverage or kernel closure.

### Slice/Minkowski witnesses and initial curve grammar, 26 November 2026

Owner: CSGRS section/projection and represented-solid Minkowski regression
contracts; native planar construction adapters and private predicate/cache
helpers. Relevant properties: P1/P6/P8/P9/P17/P18/P19/P20/P30/P37. Baseline:
`0665cc8f5`. Read every line of `tests/slice_projection_contract.rs`,
`tests/minkowski_contracts.rs` and `src/curve/mod.rs`. Read `src/curve/native.rs`
from its beginning through `regular_ngon`, inclusive. Supporting reads covered
side-wall helper consumers, native exact ring admission and bounded scalar
equality; these do not earn whole-file credit for their owning modules.

Missing witnesses: test helpers, test declarations, the compatibility trait and
private curve helpers lacked Rustdoc. Initial primitive summaries omitted
placement, sampling, validation order and collapse details. Documented the
actual assertions for holes, signed multiplicity, coincident faces, permutations,
open wires, canonical empties, invalid inputs, cache mutation/eviction, thread-local
policies, strict retries, native diagnostics, configured resource limits and
pre-set cancellation. Minkowski docs distinguish independent volume/decomposition
expectations from shared-kernel Boolean comparisons and analytic-ball claims.
The full sampled-sphere fixture remains unchanged. These are source-reviewed
witnesses, not claims of execution or universal geometric proof.

Curve docs now describe the strict-only 64-entry FIFO, coordinate equality keys,
non-refreshing hits and the distinction between entry bounds and memory/work
bounds. Sampled circle/ellipse docs give full-axis versus radius meanings,
starting points, orientation, straight edges, minimum samples, nonnegative
validation and qualified collapse. Polygon docs describe exact coordinates,
optional repeated closing points and native admission. Orientation helpers use
the segment-start polygon's policy-decided signed area; reversing endpoint pairs
does not reverse segment iteration. Exact ratio, polar and rotational-pattern
helpers document conversion errors, angular units, ordering and zero-copy behavior.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/slice_projection_contract.rs tests/minkowski_contracts.rs`: passed.
  `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in all three edited
  Rust files, including assertions, feature gates and predicate order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected thirteen rendered contracts, including the macro-generated private
  cache item, predicate/ring helpers, wall orientation, exact ratio/pattern
  construction and public polygon/circle/ellipse/triangle entry points.
  Integration-test Rustdoc was reviewed in source; ordinary library docs do not
  render test items.

Log: `/tmp/csgrs-rustdoc-curve-primitives-doc.log`. Reused temporary build storage
is about 498 MiB, with over 125 GiB available on `/tmp`. No runtime builds or
additional target trees were created. Full workflow/runtime checks remain waived
for this docs-only session. No executable behavior or performance claim changed.

### Sampled and compound planar shapes, 26 November 2026

Owner: CSGRS native planar construction grammar and its adapter/downstream
acceptance witnesses; P1/P4/P5/P6/P8/P9/P17/P18/P19/P25/P30/P37. Baseline:
`3412d69d1`. Read every line of `src/curve/native.rs` from `arrow` through
`circle_with_two_flats`, inclusive. Read every line of
`tests/curve_constructor_failures.rs` and `tests/sampled_profile_failures.rs`,
including generated fixtures, optional expression artifacts and OBJ branches.
Supporting reads covered the existing unit invalid-parameter and egg/heart-bound
witnesses; the later native test module is still awaiting its full audit.

Missing witnesses: planar summaries did not specify placement, sample formulas,
normalization, clipping sides, capacity/validation order or qualified branch
selection. Integration-test declarations, helpers and generated test items lacked
Rustdoc. Documented all 27 constructor entry points in this section and every
named helper/test declaration in both integration files without changing logic.

The docs distinguish exact polygon definitions from analytic curve fidelity.
Arrow shoulders can coalesce under an approximate comparison while retaining
qualification across a strict ring-cache hit. Teardrop length is attained by its
samples only with an even semicircle interval count; squircle axis extrema may
also be absent at coarse resolution. Egg bounds are centered, while heart bounds
start at the origin; both normalize sample extrema. Superformula docs give the
radial expression, scalar failure paths and direct closing edge without promising
a smooth periodic seam.

Compound docs specify positive-Y keyhole handles, positive-X keyway cuts,
Y-directed flat clipping and zero-width-strip regularization. Reuleaux construction
intersects equally oriented sampled disks with the documented center radius;
`diameter` supplies each disk radius, and the polygonal result is not certified
constant-width geometry. Annular thickness is a difference of circumradii, with
edge-line separation `thickness * cos(pi / segments)`. Full-turn sectors use a
regular polygon starting on positive X regardless of authored start or sweep sign.
Capacity checks are distinguished from a total Boolean-work bound.

Test docs state actual error, certainty, area, topology and composition assertions.
They preserve the original high-resolution example fixtures and distinguish
expression artifacts from semantic proof. OBJ replay coverage concerns a dyadic
fixture's positions and volume; it is not generalized to lossless OBJ interchange.
Macro-generated tests now carry Rustdoc in their expansion.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/curve_constructor_failures.rs tests/sampled_profile_failures.rs`: passed.
  `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in the three edited Rust
  files, including macro bodies apart from Rustdoc, assertions and feature gates.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Checked all 27 affected generated constructor pages for the documented formulas,
  placement, policy and failure details. Integration tests and generated test
  declarations remain source-reviewed; the library doc build does not render them.

Log: `/tmp/csgrs-rustdoc-planar-shapes-doc.log`. Reused temporary build storage
is about 499 MiB. No additional target tree or runtime build was created. Full
workflow/runtime checks remain waived for this docs-only session. No executable
behavior, assertion or performance claim changed.

### Mechanical profiles and private gear samplers, 26 November 2026

Owner: CSGRS mechanical-profile grammar and native planar admission adapters;
P1/P4/P5/P6/P8/P9/P17/P18/P19/P25/P37. Baseline: `f39e10019`. Read every line
of `src/curve/native.rs` from `involute_gear` through `sampled_cycloidal_gear`,
inclusive, and every line of `tests/gear_airfoil_failures.rs`. Supporting reads
covered the later parameter-sensitivity, cycloidal-rack contact and mechanical
single-component unit witnesses; the later test module still awaits full review.

Missing witnesses: public mechanical summaries omitted dimension conventions,
derived restrictions, placement, sample parameterization and airfoil closure.
Private gear samplers and integration-test helpers/witnesses lacked Rustdoc.
Documented all ten public entry points, both private samplers and every named
helper/test declaration in the integration file. The scalar, predicate and
native-region implementations retain authority; no algorithm was changed.

Gear docs now give pitch/base/tip/root formulas, clearance and backlash units,
positive-X tooth placement, radial feet and shared rotational construction.
Involute flanks sample radius; cycloidal flanks sample rolling parameter, with
principal inverse-cosine selection of first-lobe endpoints and explicit reach
and separation checks. Both tip and root circular lands are polygonally sampled.
Private docs give the involute deflection and epicycloid/hypocycloid formulas,
root cutoff, endpoint tests, mirroring and final native admission.

Rack docs distinguish straight involute flanks from sampled cycloidal parts and
give pitch, root/tip levels, backing depth, end-wall placement, positive land
requirements and checked storage. The airfoil contract states normalized digit
meanings without an integrality restriction, uniform chord stations, piecewise
camber and normal offsets. Thickness coefficients preserve their binary64 literal
values. The authored ring keeps all upper stations and only lower interior
stations: the upper trailing edge connects directly to the last lower interior
sample, and the lower trailing-edge endpoint is omitted. Documentation does not
promise analytic fidelity or exact chordwise bounds for this polygon.

Test docs preserve the original resolutions and identify independent rack area
and airfoil rational-ordinate expectations, selected membership contacts, manifold
and volume checks, downstream empty consumption, typed capacity/invalid/undecided
failures and qualified cache/solid composition. Admission counts and scalar totals
are not presented as universal set equality or performance measurements.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/gear_airfoil_failures.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both edited Rust files,
  including tests, validation order, sample loops and scalar coefficient literals.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected twelve generated public/private contracts for formulas, placement,
  capacity, branch selection and closure. Integration tests remain source-reviewed;
  ordinary library documentation does not render their declarations.

Log: `/tmp/csgrs-rustdoc-mechanical-profiles-doc.log`. Reused temporary build
storage remains about 499 MiB. No runtime build or additional target tree was
created. Full workflow/runtime checks remain waived for this docs-only session.
No executable behavior, assertion, coefficient or performance claim changed.

### Exact control nets and Hilbert clipping, 26 November 2026

Owner: CSGRS exact path/region constructors and retained Hilbert trim adapters;
P1/P4/P5/P6/P8/P9/P12/P15/P16/P17/P18/P19/P25/P30/P37. Baseline: `abccd1272`.
Read every line of `src/curve/native.rs` from `bezier_path` through
`hilbert_paths_with_context`, including both function-local grid helpers.
Read every line of `tests/exact_path_failures.rs` and `tests/hilbert_trimming.rs`.
Supporting reads covered native path admission, spline layout/endpoints and
parameter domains, general rational Bezier construction, and retained region
bounds. These supporting modules are not credited as whole-file reviews.

Missing witnesses: exact-path docs omitted carrier selection, parameter domains,
fill rules, unused display-count behavior and precise diagnostics. Hilbert docs
needed bounds/placement and conversion details; local integer helpers and test
helpers/declarations lacked Rustdoc. Documented all ten entry points in this
section, both local functions and all named declarations in the integration files.

Bezier docs specify line/quadratic/cubic specialization and unit-weight rational
storage for higher degrees, while preserving the polynomial meaning. Closed paths
do not imply filled regions; region construction checks closure and uses native
even-odd admission without adding a closing line. Spline docs give clamped integer
knots, retained source controls and evaluation on `[0, control_count - degree]`.
Positive display counts are validation-only and do not request sampling buffers.
Checked spline index overflow remains `InvalidArguments`; documentation does not
relabel it as resource exhaustion or change validation.

Hilbert docs give the cell-center formula, traversal orientation, pre-empty
parameter validation, padded bounding rectangle and possibly conservative native
envelope. The point-count cap does not bound native trim work. The exact path view
retains source intervals and contact evidence; the string view preserves represented
line geometry but drops those records and returns a typed unsupported result when
selected endpoints lack scalar coordinates. Local quadrant rotation/distance
helpers state their integer-domain preconditions.

Test docs distinguish independent interval/area/midpoint expectations from wrapper
agreement, conservative axis-aligned simplification from general curve simplification,
and contact replay from display approximation. The historical test named
`unresolved_closure_is_undecided_across_policy_reuse` constructs its endpoint with
`sin(e)^2 + cos(e)^2 - 1` directly and checks only the resulting native error
category/source. It does not separately test endpoint-equality uncertainty or
localize the failing stage. Its Rustdoc now states those limits. Its current
runtime outcome has not been reproduced in this docs-only batch; do not cite it
as proof of a specifically localized closure blocker. The fixture and all its
assertions remain unchanged.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/exact_path_failures.rs tests/hilbert_trimming.rs`: passed.
  `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in all three Rust files,
  including local functions, validation order, fixture expressions and assertions.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected all ten generated entry-point pages for carrier/domain, fill,
  validation, placement and evidence contracts. Function-local and integration-test
  documentation was reviewed in source; ordinary library docs do not render it.

Log: `/tmp/csgrs-rustdoc-exact-paths-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. No
executable behavior, assertion or performance claim changed.

### Profile conversion, meshing and extrusion, 26 November 2026

Owner: CSGRS curve-to-mesh modeling boundary, native triangulation adapters and
output validation; P1/P4/P6/P8/P9/P12/P16/P17/P18/P19/P25/P30/P37. Baseline:
`830b957e5`. Read every line of `src/curve/native.rs` from `try_triangulate`
through `emit_section_walls`, inclusive, and the complete separate
`native_boundary_source_probe` test module at the end of that file. Read every
line of `tests/mixed_modeling_profiles.rs`. Supporting reads covered native
linear-profile retention, mixed sampling/join selection, finite export ownership,
triangulation, geometric validation, scalar debug checks and construction-receipt
publication. Those kernel modules are not credited as completed Rustdoc audits.

Missing witnesses: private meshing helpers lacked Rustdoc, and public summaries
did not expose extrusion validation order, finite-history limitations or the
actual strength of output validation. Documented the public boundary, private
helpers, callback/ending types and relevant test declarations without changing
construction, predicate order or assertions.

Modeling profiles choose exact linear views independently; only absence of that
view selects mixed sampling, while native failures propagate. Sampling preserves
represented line members and native endpoints, including tiny members beside
curved components, and records sampling/projection qualifications on mesh output.
Endpoints without native coordinates may also be projected. The chord-error value
is a request, not a blanket topology/fidelity certificate. Finite export projects
all coordinates and retains native ownership with a local certainty receipt; its
generic outcome does not attach the internal mesh construction accumulator.

Extrusion docs specify signed height, full displacement vectors, nonzero-Z
validation before empty handling, cap/wall orientation and canonical empty-source
results. Validation checks unique nondegenerate face geometry and directed
geometric edge pairing; neither validator checks embedding or vertex links.
A native false result becomes a modeling `Geometry` error, while native failures
retain their causes. The allowed-empty closure shortcut inspects triangle count,
not positions. The separate source-probe tests document these distinctions.

The triangle emitter uses reverse structural position lookup, has a strict
nondegeneracy debug assertion, and returns `true` on every normal path, including
suppression of repeated-index faces when the assertion is disabled. Its Boolean
return is not an emission certificate. Public triangulation/extrusion docs expose
the debug panic condition. Cap/wall docs specify traversal order, the chosen quad
diagonal, caller-owned apex collapse, all-four-corner placement even for an apex,
and partial local buffers on placement errors; final publication remains a caller
responsibility.

Mixed-profile tests document independent exact component measures and retained
vertices for rational, irrational, tiny and far-translated inputs, plus native
closure checks and sampling qualifications across five consumers. Repeated thin-wall
Booleans are checked after dropping the source region. These witnesses remain
source-reviewed, not runtime completion or performance claims.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/mixed_modeling_profiles.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both edited Rust files,
  including debug assertions, emission returns, all tests and validation order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected sixteen generated public/private pages, including the sampling
  constant, placement alias, wall-ending enum, validators and emitters. Test
  declarations were inspected in source; the library build does not render them.

Log: `/tmp/csgrs-rustdoc-profile-meshing-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. No
executable behavior, assertion or performance claim changed.

### Signed revolution and exact profile sweep witnesses, 26 November 2026

Owner: CSGRS curve-to-mesh revolution, signed-region composition and output
validation; P1/P4/P6/P8/P9/P12/P16/P17/P18/P19/P25/P30/P37. Baseline:
`2b8e62e6b`. Read every line from `revolve` through
`split_at_revolution_axis` in `src/curve/native.rs`, and every line of
`tests/exact_profile_sweeps.rs`. Supporting reads inspected native closed-PWN,
unique-face and geometric edge-pairing validation in Hypermesh; that file is
not credited as a completed audit.

Missing witnesses: revolution mode/dispatch/validation helpers lacked Rustdoc;
public docs omitted segment minima, empty-input validation order and the
limited rational turn-count recognizer. The test module summary understated
its revolution oracles, and `exact_solid` claimed manifoldness beyond the
predicates it calls. Updated these contracts and documented all 50 test
function/constant/type declarations, including the function-local prism alias.

Revolution docs distinguish legacy overlapping partial shells from regularized
signed material, preserve odd-lattice phase and state the even full-turn fold
precondition. Oversized turns use `exact_rational` for integer recognition;
failure to recognize rationality is undecided, not exhaustion or a proved
fractional turn. Angular-table addressability is not a global resource budget.
Parameter validation precedes empty handling. Partial cap emission retains its
strict nondegeneracy debug assertion. Explicit-start revolution transports the
completed swept-boundary result and its qualifications through a transform.

Private docs state caller obligations for matching sign arrays and angular
samples, potential zip truncation/indexing on violated internal preconditions,
axis-row sharing, quad diagonals, cap orientation and native filled-region
authority for splitting/folding. Closed-PWN validation checks nondegenerate
faces and balanced edge multiplicities without proving manifoldness or unit
winding; the legacy check also lacks embedding/vertex-link validation. The
regularized allowed-empty path checks both buffers, unlike the legacy
triangle-only shortcut.

The test audit separates exact volume/vertex witnesses, deterministic buffer
comparisons, shared Boolean composition and independent sector/ray-winding
oracles. The retained winding-two mutation demonstrates why edge closure alone
does not establish regularized material. The 576-row matrix and finite query
grids are scoped regression evidence, not universal closure proofs. Optional
STL replay explicitly checks projection provenance and equality relative to the
imported mesh. No fixture, predicate, assertion or executable behavior changed.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/exact_profile_sweeps.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both Rust files.
  A declaration scan confirmed Rustdoc on all 50 function/constant/type items
  in the test file; that file-local scan is not a repository coverage metric.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected ten generated public/private pages, including the output enum and
  all revolution helpers. Integration tests and the local alias were reviewed
  in source; ordinary library docs do not render those items.

Log: `/tmp/csgrs-rustdoc-revolution-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. Test
execution, optional STL replay and performance were not revalidated here.

### Sweep frames and twisted extrusion, 26 November 2026

Owner: CSGRS profile placement/frame construction and native mesh validation;
P1/P4/P6/P8/P9/P16/P17/P18/P19/P25/P30/P37. Baseline: `943279e05`.
Read every line from `sweep_scalar_error` through `extrude_twisted` in
`src/curve/native.rs`, and every line of `tests/sweep_frames.rs`. Supporting
reads covered vector normalization, minimal unit-vector rotations, shared wall
and cap emission, the previous exact-profile fixtures, and scalar uncertainty
controls. The later optional/transform wrappers were read for context but are
not yet credited as completed item audits.

Also reviewed/documented these main-module items completely: the certified and
terminal zero helpers, `signed_volume`, `assert_exact_solid_boundary`,
`profile_solid_constructors_share_closed_indexed_topology`,
`sweep_rejects_revisited_open_path_vertices`,
`degenerate_profile_solid_requests_do_not_claim_solids`,
`approximate_sweep_reports_consumed_terminal_point_equality`, and
`sweep_scalar_failures_keep_their_outcome_category`. The remaining main-module
tests still require their own audits.

Missing witnesses: sweep/twist docs omitted the closed-frame seam, validation
before empty input, allocation scope and emitter debug assertions; twist docs
overstated manifold validation. Private frame docs omitted predicate order,
fallback conditions and which errors can be superseded by an independent
nonzero component. Test docs called approximate terminal resolution a
certificate, treated unnormalized dot products as basis coordinates and
overstated the nearest-center grouping's separation assumption.

The sweep retains one frame per distinct path vertex, using chord normals at
open endpoints and normalized unit-chord bisectors elsewhere. Closed paths
connect the final stored frame directly to the first, without redistributing
residual rotation. Scalar normalization retries only `UnknownZero`; a caller's
nonzero decision is not passed as a proof object into the normalizer. Reversal
and initial-axis helpers preserve component order, permit a later independent
nonzero component to resolve an earlier failure, and otherwise propagate the
last component error. Unknown scalar certificates acquire the sweep label and
policy; exhaustion and other scalar causes remain typed scalar failures.

Sweep path/frame construction and twist parameter-table construction precede
empty-profile handling. Neither entry point introduces a total-work/heap or
cancellation budget; twist has no angular-table addressability preflight.
Wall/cap debug assertions can precede final validation. The latter proves
unique nondegenerate faces and directed edge pairing, not vertex-link
manifoldness, embedding or unit winding. Negative twist-extrusion height
reverses wall/cap orientation; apex tapering has no top cap.

Rechecked the documented thin-strip counterexample independently with exact
Python `Fraction` arithmetic in Q(sqrt(3)): width 2^-20 at X=1, Y in [0,2],
height 1, twist 90 degrees, three equal bands and final scale (1/2,2/3).
Placed four sections using the documented linear scales and 0/30/60/90-degree
sines/cosines, split each wall on the implemented diagonal and added planar
caps. All 28 triangles were nondegenerate and every directed geometric edge
had one reverse. The exact scalar-triple-product sum (six-volume) was

```text
-1495330579284247/1068725302198272 + (205/169869312) * sqrt(3).
```

Its upper bound using sqrt(3) < 2 is negative. This checks the mathematical
construction formula and the limited meaning of edge closure; it is not a
runtime execution or completion claim for the Rust constructor.

Frame tests now distinguish stored-rational checks, exact distance/normal/axis
invariants, binary64 helix residuals, approximate nearest-center grouping,
Boolean volume identities and error-category witnesses. Printed timings have
no performance threshold. The local projected-volume helper skips failed
projections, and its Rustdoc no longer implies an exact volume certificate.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs tests/sweep_frames.rs`:
  passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both Rust files.
  A file-local declaration scan found Rustdoc on all 21 test functions/constants;
  this does not establish repository-wide documentation coverage.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected six generated public/private pages for sweep, twist and frame/error
  helpers. Integration and unit tests were inspected in source; ordinary
  library documentation does not render test-only items.

Log: `/tmp/csgrs-rustdoc-sweep-frames-doc.log`. Reused build storage remains
about 499 MiB. No additional target tree or runtime build was created. Full
workflow/runtime checks remain waived for this docs-only session. No predicate,
construction, assertion or executable behavior changed.

## Next work

Continue `src/curve/native.rs` at `metaballs`, including optional image/text
adapters, planar transforms, queries and the remaining main-module tests, then
review the remaining root modules and owning crates. Maintain review records with
each batch. Establish automated public/private item coverage across manifests
and features without confusing that metric with semantic review. Review every
remaining tracked file before claiming completion. Commit and push documentation
batches after focused checks under the owner's session-specific waiver.
