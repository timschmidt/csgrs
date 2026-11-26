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
rendered items. All workflow validation required by `AGENTS.md` remains a gate
before each push. Do not credit a narrow documentation build as that gate.

## Reviewed Rust files

| File | Review and change | Remaining validation |
| --- | --- | --- |
| `src/context.rs` | Entire file, including tests. Documented every previously undocumented private declaration; corrected approximate-policy wording and documented receipt loss at `into_value`. | Private docs passed with no default features and with `curve`; broader feature/workflow checks pending. |

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
have not yet run; no push is permitted on the basis of these narrow checks.

## Next work

Commit the context batch. Audit the scalar
adapter boundary (including exact float import, lossy export, and integer error
semantics), then continue through the remaining root API and owning crates.
Maintain review records with each batch. Establish automated public/private
item coverage across manifests and features without confusing that metric with
semantic review. Review every remaining tracked file and the complete workflow
matrix before claiming completion.
