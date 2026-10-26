# Future work

## Import resource contracts after R19

The scoped R19 change passes final repository validation. Bounded domain replay,
small-angle sign proofs and shared reciprocal construction restore the scoped
cold continuation cases; they do not supply a cumulative work or heap budget.
Scalar imports still have no caller node/bit/work/byte/cancellation contract.
Generic Laurent coefficient expansion and reciprocal/trig approximation can
still encounter enormous shifts or panic outside the repaired proof paths;
recursive graph teardown remains separately open below. BRep archive limits
bound input syntax and records, not scalar reconstruction arithmetic or heap.
Hypermesh sharing caps attempts, not each proof's cost, and has no operation
cancellation hook. A cumulative hook needs a measured owning path and typed
exhaustion, cleanup and retry tests. The suggested `quantile(Phi(1000))` control
already exhausted natively on this revision, before serialization; it does not
establish an import defect. See the [R19 evidence and costs](PERFORMANCE.md#cold-domain-continuation-and-resource-audit-r19-2026-10-25).

## Rotation relations after R18

The original binary64 four-angle cube union now has a bounded exact proof.
The final relation attempt can cancel common admitted factors before expanding
the remaining polynomial, then use its existing unit-circle identities.
Expanded degree/term limits and the shared work budget are unchanged; broader
compound rotations remain outside this completion claim. Five scalar controls
cover factor bounds, inverse obligations, nonzero residuals, exhaustion and
cancellation. Public tests check the independent cube oracle and continued
empty operations.

This is a completeness extension with disclosed costs. All 64 simultaneous
scalar latency upper bounds exceed 1, and the unresolved degree-14 control
requests 2,096 extra bytes per query. No speedup or latency nonregression is
claimed. See the [R18 record](PERFORMANCE.md#factored-compound-rotation-completion-r18-2026-10-25).
General recognition, cumulative operation work/heap bounds and cold large-graph
archive continuation remain open; S01 and a huge-degree field descriptor are
not adopted.

## Retained scalar memory after the R17 audit

The October 25 Nano reproduction attributes 16.9% of held-report allocations
to rational derived records and 9.2% to `Computable` evaluation caches. All
116,924 evaluation cells contain an approximation; 993 also contain a quadratic
tower. There are no decline-only cells in the sampled stages, so moving only
decline flags cannot reduce this workload's held cells.

A private layout allocating the fourth/fifth rational cache slots lazily keeps
the same exact entries and weak ownership. It reduces held report payload by
5.3% and full-release peak payload by 4.6%, but a protected wide-rational warm
negation query regresses, and none of the 64 simultaneous intervals proves
latency nonregression. The layout is not retained. See the
[measurements and validation](PERFORMANCE.md#retained-scalar-ownership-and-rejected-layout-r17-2026-10-25).

Full Nano post-drop payload grows across the three measured repetitions;
neither these samples nor the earlier small-region probe establish a byte
bound or plateau. Future retention changes need a new measured premise,
source/precision/policy/lifetime preservation and admitted re-query costs.
Internal work, retained-byte and cancellation contracts remain open. This is
not a proposal to evict source authority, restart the rejected per-call Laurent
memo or adopt S01.

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

**Measured 2026-10-23 (research S01).** Retaining completed atom-free forms
across the predicates of one Boolean operation saves 52-55% of the
instructions of the quadratic chains and 5% of the 16-gon and 10-degree
chains, and a verification run found no warm result that differed from the
cold one. Most of the reuse is in Hypertri's triangulation predicates, which
a workspace in Hypermesh's decision context does not reach; there it saves
only 4-5%. Compacting arrangement coordinates at construction (Hypermesh,
2026-10-23) now captures most of the same cost. The remaining 12-16% needs
workspace variants of Hyperlimit's 2D predicates and a workspace in
Hypertri's `PredicateEvaluator`. PERFORMANCE.md has the measurements and the
prototype's contract.

**Decision, 2026-10-24: not adopted for now.** The maintainer chose to leave
the APIs unchanged, because the coordinate compaction already captures most
of the cost without new public API. The design is kept here for a later
decision, for example if workloads dominated by quadratic fields become a
priority:

- Hyperreal: a public, opaque, operation-owned workspace type, and sign and
  normal-form entry points that take it by `&mut`.
- Hyperlimit: workspace variants of about eight 2D predicates (orientation,
  point-segment and segment-intersection classification, real comparison),
  next to the existing policy-only functions.
- Hypertri: the per-operation `PredicateEvaluator` holds a workspace and
  routes its predicates through those variants; the triangulation entry
  points accept or create one.
- Hypermesh: `DecisionContext` owns one workspace per Boolean operation,
  passes it to triangulation and to its own predicates, and drops it when
  the operation ends.

The workspace contract is the prototype's: it retains only completed
atom-free forms from calls that met no work or depth limit; it never retains
a decline, `Unknown` or budget-exhausted result and never refunds work; a hit
costs one node visit, so a warm call never visits more nodes than a cold one
and bounded success is unchanged; it is byte-bounded with deterministic
eviction, keyed by an identity that address reuse cannot confuse, and owned
by one operation or thread, with no global state. Expected benefit with the
compaction in place: a further 12-16% of instructions on the 6/8-gon and
30/45-degree chains and 5% on the 16-gon chain, against +0.4 to +1.1% on the
icosahedron unions and a higher peak live heap (0.6 MB to 11.7 MB for one
6-gon step with an operation-wide cache). A workspace reachable only from
Hypermesh's own predicates saves 2-3 points and is not worth its API.

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
