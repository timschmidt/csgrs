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

## Next work

Continue through the mesh and curve adapter implementations, then the remaining
root API and owning crates. Maintain review records with each batch. Establish
automated public/private item coverage across manifests and features without
confusing that metric with semantic review. Review every remaining tracked file
before claiming completion. Commit and push documentation batches after focused
checks under the owner's session-specific waiver.
