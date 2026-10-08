# Checkpoint 77 — twelfth-turn proof candidate, not retained

The isolated candidate now proves all 32 twelfth-turn radical identities found
in checkpoint 76. It retains the compact trigonometric expression graph and
does not request or overwrite approximations during its proof. This is a
completeness improvement under initial qualification, not a production change
or a performance result. The seven retained continuation transfers are unchanged.

## Scope and design

Copied only the 181 current Hyperreal source/support files, 5,317,461 bytes,
with exclusive creation and source-hash verification. The candidate adds two
files and modifies only the private include facade and unresolved-sum sign
fallback. The 957-file live map and all checkpoint-76 donor/evidence identities
remain unchanged. No new donor read credit.

The normal form uses Q(sqrt(2),sqrt(3)); a structural parser proves pi/12 angles.
It admits only bounded, exact operations and rejects unsupported fields, domains,
zero divisors and oversized expressions. The proof runs before caching an
unresolved sum, including a sum visited as a descendant. No new persistent cache
or public representation variant is introduced. The new implementation is 307
lines / 11,266 bytes, and its test module 442 lines / 16,768 bytes, plus the small
include/hook diff. These are source counts, not linked-size measurements.

See the [protocol](twelfth-relation-protocol-v77.md),
[algebraic argument and independent checks](twelfth-relation-math-v77.md), and
[final source binding](twelfth-final-binding-v77.json).

## Verified results

- Independent oracle: 6,561 coefficient-grid signs; 512 complete polynomial
  multiplication/matrix-inversion cases; 256 near-cancelling Pell convergents
  plus 512 mixed-field cases; 336 rational-square-class cases. The oracle uses
  BigRational polynomial reduction, Gaussian elimination and exact dyadic
  enclosures, not the candidate sign/conjugation algorithm.
- The unchanged public probe produces 1,728 records per debug/release profile,
  byte-identical. Exactly 192 repeated Unknown results, representing 32 distinct
  identities, become Equal. The other 1,536 rows are unchanged: 552 periodic
  Equal results, 552 perturbed NotEqual results, previously known radical
  equalities and 72 repeated pole observations. Six stream corruptions rejected.
- Matched regressions: baseline 756 default / 859 all-feature tests; candidate
  763 default / 867 all-feature tests, in both debug and release. Every original
  name/outcome is preserved; seven/eight new private tests account for the
  difference. No failures or ignored tests in these runs. Warm/cold caches,
  prior Unknowns, descendant query order and JSON/CBOR are covered.
- All-target/all-feature Clippy, both formatting checks and all-feature release
  WASM compilation pass. The complete 126-package/node metadata graphs match
  after normalizing only the intended Hyperreal root paths. WASM execution is
  not qualified by a successful build.
- Memcheck reproduces the complete public output: zero errors and zero lost
  bytes. Whole-process counts are 44,002 allocations, 43,834 frees and 4,474,572
  requested bytes; 21,584 bytes / 168 blocks remain reachable at exit. This is
  not a matched allocation comparison, peak measurement or leak-free-all-memory
  claim, and it includes setup, output and shared caches.

## Failures and evidence integrity

Preserved three failed captures: the initial test helper used nonexistent
`Computable::sqrt2`; Clippy rejected two test-oracle index loops; and the first
regression driver stopped on that Clippy failure. The helper and loops were
corrected, with initial/pre-correction test sources retained. The production
candidate is byte-identical across the Clippy-only correction.

Four sandboxed Node completion captures had status zero but empty output and
are explicitly unusable as evidence of verifier/driver completion. Approved
reruns supply the required records, independently recompute the results and
reuse the already successful individual build/test captures. No capture was
overwritten and zero status alone was not accepted.

Combined evidence passes at 2026-09-11T23:31:55.908Z. The manifest binds 1,759
artifacts and 41 captures: 25 accepted final-source gates, nine historical
development successes, three failed captures and four unusable empty captures.

Final sealed verification passes 2026-09-11T23:34:29.411Z, code 0 / null
signal, one record / 1,532 bytes and empty stderr. All commands are terminal.
[Final capture](results/twelfth-verify-v77.json).

## Storage, decision and remaining gates

Shared build cache reused; no broad stack copy or executable snapshot copies.
The two new public probe binaries in that cache are 3,198,496 and 1,970,744
bytes. Recorded /tmp availability is 13,994,549,248 bytes. No cleanup, donor/live
edit, commit, push or external report was made.

Do not retain yet. Still required: matched intended-path and bypass timings,
allocation/peak and binary-size comparisons, WASM runtime/cost checks, relevant
consumer tests, and remaining bench/doctest/fuzz/CI checks. In particular, bound
the cold failed-proof tax, shared-tree behavior and repeated-query costs before
deciding whether the extra code is worthwhile. Preserve the existing compact
nodes and exact/Unknown semantics throughout.

Current entry point: `node verify-twelfth-relation-v77.mjs`. Checkpoints 75 and 76
also remain valid for their unchanged live/source states. Continuation coverage
stays 1,447 complete / 20 partial / 185,617 uniquely read lines. Remaining
references, other transfer candidates and full inventory reconciliation keep
the original ecosystem audit open.
