# Future work

## Structural sign facts expand every fresh predicate

`Computable::structural_facts` calls `exact_sign`, which falls back to
`exact_quadratic_surd_sign`. That expands the expression into a π-Laurent
polynomial with radical coefficients, up to 512 node visits. The memo lives
for one call, and the result is cached only on the queried node. Every new
predicate expression therefore re-expands the shared coordinates beneath it.
Hyperlimit's cascade runs this "structural" stage before cheap bounded
refinement, which decides most nonzero signs.

**Evidence.** Callgrind on three steps of the 6-gon drill chain in
`tests/successive_operations.rs` (release, 7.96 G instructions): 86% inclusive
in `Real::structural_facts`, 72% in `pi_laurent_polynomial_with_memo`, and
allocation and rational arithmetic dominate the self cost. The chain's
per-step time grows faster than its output because the expression graphs
under each predicate deepen (see
[`docs/successive-operations.md`](../../docs/successive-operations.md)).

**Options.** Retain a node's normal form, or its failure, across calls. That
cache needs a documented contract under P4. Alternatively, run one cheap
refinement before the expansion when the expansion is not cached. Exact zeros
would still reach the expansion, the quadratic tower and the radical tower.
Either change needs the hyperreal and hyperlimit benchmark suites as evidence,
since the cascade order is a stack-wide performance policy.

## Dropping a deep expression graph recurses

`Arc<Node>` drops its operands recursively. Building
`pi + sqrt(1) + … + sqrt(n)` works for any `n`, and so does
`Real::representation_size`. Dropping the value aborts the process with a
stack overflow at n = 20,000 on a 2 MiB test thread; n = 5,000 succeeds.
That violates P6. Repeated Booleans deepen derived coordinates by about ten
nodes per step, so the abort is far off in today's chains. **Option:** an
iterative `Drop` that moves uniquely owned operands to a work list. It runs on
every node drop, so it needs allocation and timing evidence on the hot scalar
benches.
