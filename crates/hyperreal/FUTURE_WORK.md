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

**Measured 2026-10-22.** Running the existing 32-node binary64 filter before
the expansion is not such an option yet. In the root chains, 99.8% of
structural sign walks reached the expansion, which took 39% of user CPU, or
52% in the long chains. The filter could evaluate none of those graphs,
because each contains a radical constant, a `Sqrt` node or a trigonometric
node. PERFORMANCE.md has the counts. A preflight would first need certified
enclosures for those node kinds.

## Dropping a deep expression graph recurses

`Arc<Node>` drops its operands recursively. Building
`pi + sqrt(1) + … + sqrt(n)` works for any `n`, and so does
`Real::representation_size`. Dropping the value aborts the process with a
stack overflow at n = 20,000 on a 2 MiB test thread; n = 5,000 succeeds.
That violates P6. Repeated Booleans deepen derived coordinates by about ten
nodes per step, so the abort is far off in today's chains.

**Tried, 2026-10-22, not kept.** On `3836df74fa` a child process that builds
the 80,000-term sum on a 2 MiB thread aborts with `SIGABRT`. An iterative
`Drop for Node` fixed it. It released operands through `Arc::into_inner` and
queued uniquely owned interior nodes in four inline slots, with a fallible
overflow vector. Shared operands were only decremented. The 80,000-level
chain then dropped with no scratch allocation, and its live bytes returned
exactly to their starting total. Shared roots, shared interior partial sums
and eight concurrent final owners all stayed valid.

Every variant made ordinary drops materially slower, so none was kept.
PERFORMANCE.md lists the variants and measurements. The best kept the derived
recursive order within 16 KiB of stack below the outermost node drop. That
variant cost 13–14% more instructions and 16–30% more drop time on fresh
`sqrt(2)`, mesh-coordinate, rotated-coordinate and 16-root values. It also
doubled teardown time per level once the iterative path took over.
Thread-local bookkeeping is the floor of that approach, because every value's
outermost node drop pays for it.

**Remaining option.** Store a saturating graph depth in the node's fact word,
which has five free bits (11–15), when the node is constructed. Drop would then
read one word and compare it to the limit, and only nodes beyond the limit
would be released iteratively. This moves a load per operand into `Node::new`.
It also changes the cache encoding modelled in
`verification/cache_encoding_model.rs`, so it needs Verus proof updates and
construction-bench evidence beside the drop evidence.
