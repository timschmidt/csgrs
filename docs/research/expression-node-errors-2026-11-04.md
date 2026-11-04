# Lazy expression diagnostics, 4 November 2026

Owner: CSGRS expression lowering, P6/P25/P30; U06 of the simplification/API
program. Source baseline: `c0db6b33bf`. Domain: lazy mesh expressions under
both explicit predicate policies, including shared subtrees, transforms and
canonical empty operands. No geometry kernel or scalar decision rule changes.

## Reproduction and contract

A nested zero-axis scale failed with only `invalid arguments`, without naming
the expression occurrence. The new acceptance witness failed on the baseline.
Evaluation now returns `Error::Expression { node_path, source }`:

- Transform failures identify the original transform, including after it is
  pushed down to several leaves.
- Native input diagnostics carrying an operand index identify that leaf's
  first lowered occurrence. Empty operands do not hide malformed partners.
- Shared-arrangement errors and native diagnostics without an operand index
  identify the root evaluation. An empty path does not establish a unique
  offending leaf. For example, `VertexIndexOutOfBounds` has no operand index.
- `SolidExpr::node_at` resolves a path relative to the original expression;
  incompatible steps return `None`. Paths are occurrence descriptions, not
  persistent identities across edits or allocation-based mathematical equality.
- The original error remains boxed and queryable through the public variant
  and standard source chain. `kind()` delegates to the cause. Invalid transform
  numbers still fail during construction, before an expression node exists.

The intentional diagnostic migration is in `docs/MIGRATIONS.md`. Successful
geometry, predicate order, certainty, inheritance and arrangement count are
unchanged. Parent links share path prefixes during lowering; only an error
materializes its path. This adds temporary metadata proportional to visited
lowering edges and operands. It introduces no persistent cache and supplies
no new total-work, heap, stack or cancellation bound for the existing evaluator.

## Verification

After the final Rust edit:

- `cargo test --offline --release --features dispatch-trace`: 681 passed
  including five doctests, seven existing ignores, zero failures. The new
  cases cover typed causes/categories, path resolution, shared occurrences,
  original transforms, invalid/empty operand pairs under every Boolean,
  unbounded projective maps, undecided arrangements and retries. Existing
  exact-set, volume, certainty, downstream composition and one-arrangement
  assertions remain intact.
- `cargo clippy --offline -p csgrs --all-targets --features dispatch-trace -- -D warnings`
  and its `--no-default-features` counterpart: passed. The all-target
  `dispatch-trace` Clippy check also passes on MSRV Rust 1.95.0.
- Warning-denied CSGRS rustdoc, workspace formatting, whitespace and repository
  budget checks: passed.

The actual SynapsCAD consumer initially failed on five stale by-reference
`GeometryContext` calls from the preceding API migration. Corresponding
source, test, benchmark and fuzz call sites in six files now pass the policy
by value. At this checkpoint its existing D01/D02 edits remained uncommitted; the
[subsequent downstream admission](downstream-api-admission-2026-11-04.md) records
their local commits and runtime qualification. With the corrected
calls, offline locked all-target checks, warning-denied all-target Clippy,
formatting and locked fuzz-target checks pass against this source. `CCACHE_DISABLE=1`
avoids the host C cache's read-only directory; the initial environmental failure
and the subsequent API failures are retained. This is compilation/lint evidence,
not fresh GUI, runtime application-suite or installed-package qualification.

## Cost observations

A matched focused probe runs the unchanged ignored drilled-block test on an
isolated baseline archive and this candidate, with identical release features,
compiler and lockfile. Three alternating process pairs each execute five eager
and five lazy evaluations and the fixture's unchanged exact-set comparison.
The lazy median-of-medians is 18.7 ms baseline versus 24.4 ms candidate, an
adverse observation. The unchanged eager control also moves from 20.8 to
27.3 ms. Host load is 16–17; this small probe cannot separate shared-host
variation from a regression. It establishes no latency nonregression or
speedup claim; all six process summaries are retained.

`ValidationError` remains 48 bytes and `Result<TriangleMesh>` remains 56 bytes
in both builds. The built cdylib grows from 3,304,192 to 3,305,752 bytes; its
ELF `.text` stays 232,627 bytes. A cdylib is not a representative monomorphized
Rust application. Retention is for the added diagnostic capability, not a
performance or size improvement.

Evidence: local evidence branch `expression-node-errors-2026-11-04`, commit
`34fb72de7b2eb4cb3a75074124287518d9b009c0`; record `docs/evidence/2026-11-04-expression-node-errors`
(30 files, 322,219 bytes), SHA-256
`326f3d6b42f7dfa528b0e26a8fbfda1e71dab909ca94ba7bc7cd620c8e5ea7a4`. The archive tree adds only this record and
preserves every previous path. The branch is unpushed. General
operation closure, uniquely localizing native errors lacking operand identity,
application admission, full pre-push checks and published distribution remain
open.
