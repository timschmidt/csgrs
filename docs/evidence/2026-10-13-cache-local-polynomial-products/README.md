This experiment tests a typed borrowed multiplication operation inside the
original Box-backed polynomial cache. The inputs and fixed protocol are
recorded before preparation, compilation or measurement. No runtime is admitted.

The previous [direct-projection experiment](../2026-10-13-retained-polynomial-projection/README.md)
passed its scoped correctness checks but was rejected after fixed scalar costs.
Its source and samples remain frozen. This candidate starts from the unchanged
endpoint/P4 parent with twenty shared tests; it does not inherit the rejected
Arc payload, recursive result enum or projection changes.

The [source review](scope-v1.json) examines a narrower borrowing lifetime. A
private typed cache method accepts an owned left polynomial. On a cache hit,
it borrows the closed right coefficients under the existing read lock and
returns an owned product. On a miss, it returns the original left input
without cloning it. A cached-product decline is distinct from a miss and marks
certificate reuse, preserving the original outer retry.

The closed multiplication kernel follows the previously qualified operation
and insertion order. Its coefficient body uses rational arithmetic and
immutable structural comparisons; the reviewed helpers do not evaluate
expressions or call the expression cache. The closed right payload contains
no Computable owners. The typed entry point accepts no arbitrary callback.
The caller retains the right node, and the guard is released before recursive
normalization, cache publication or returning the product.

Holding a read lock across rational arithmetic can delay writers. Coefficient
bit sizes are not constant, and neither source inspection nor the planned
concurrency test proves a fixed lock-duration bound or wait-free behavior.
This is a scoped source review, not a whole-crate formal proof. Lock duration,
helper dispatch and code generation remain performance risks.

Left normalization still precedes the right lookup. A retained hit bypasses
the original depth/work counters, including zero budgets. A miss enters the
original recognizer body exactly once, without a second root cache probe.
Preparation recovers that body and all other baseline bytes after reversing
only the declared edits. Original memo insertion, opaque ownership, limits
and retry-after-memo-drop behavior remain. P4 permits stronger cached proofs,
never contradictions or weaker available facts.

Eight private controls cover exact products and buffer stability; term,
radical and power frontiers; unchanged left ownership on misses; zero-work
hits and hit declines; original fallback work/memo accounting; concurrent
product readers and exact approximation writers; poison recovery; and an
opaque left operand that owns the same node whose closed cache is borrowed.
Shared projection/storage/memo/alias tests and inherited cache/P4 controls
remain unchanged.

The planned qualification has 28 sequential commands across two fresh
targets: focused controls, default/all-feature scalar libraries, 28 doctests
per variant, formatting, strict default/all-feature/all-target Clippy and
warning-denied docs. Counts overlap. Existing layouts, memo counts, ordinary
copy sharing and the temporary product-lookup result size are recorded.

After qualification, three unchanged scalar suites will run twelve preflights
and sixty measurements in five fixed paired blocks. Timing and allocation
executables are separate, all builds finish before probes, and all samples
are retained. No earlier sample set is extended. Native correctness, costs,
reports and production size remain separate admission obligations. Both full
fixtures and the 120-second board worker-CPU and 60-second neck check-body-wall
assertions remain unchanged. The full properties plan is still open.
