# Opposite-product cancellation acceptance

Commit `775af8774` is retained as a scalar completion extension against baseline
`f1092edb1`. It proves `a*b-b*a = 0` before expanding structurally identical
factors. The original 16- and 32-segment strict drilled-body workflows still
return typed undecided outcomes; this record does not admit strict drilling.

The owning layer is Hyperreal (P2–P5/P7); predicate and mesh consumers reuse
the shared implementation through `diff_of_products`, `signed_product_sum`
and `active_signed_product_sum`. The input domain is valid `Real` operands in
exactly two opposite-sign two-factor products. Either factor ordering is
supported. Failed structural equality supplies no mathematical fact. Rational
reduction remains first; other factors, signs and arities retain their paths.

The independent oracle is commutativity over the reals. Near-distinct factors
offset by exact `1/10^12` remain nonzero. Eight scalar tests cover independent
reconstruction, both sign orders, same-sign sums, other arities, zero/high/zero
precision and concurrent readers. Two native tests establish the horizontal
normal of a symbolic vertical triangle and the nonzero normal component after
an exact displacement. Wrapper agreement alone is not the oracle.

## Final-source qualification

All 13 commands in [qualification-v2.json](qualification-v2.json) passed after
the last production/test edit. Counts overlap across configurations and do not
measure coverage or unique workload completion.

| Configuration | Passed | Existing ignored |
| --- | ---: | ---: |
| Hyperreal default | 886 | 0 |
| Hyperreal all features | 991 | 0 |
| Hyperlimit, Hyperlattice, Hypersolve all features | 1,544 | 0 |
| Hypermesh all features | 240 | 7 |
| Hypercurve all features | 2,493 | 33 |
| CSGRS curve feature | 231 | 0 |
| CSGRS minimal | 96 | 0 |
| CSGRS all features | 338 | 1 |
| Hyperreal doctests with warnings denied | 28 | 0 |

Workspace formatting and the recorded all-target Clippy configurations with
`-D warnings` also passed. [The assessment](scalar-assessment-v1.json) retains
commands, source and executable hashes, outcomes and limits. The original
drilling regression still explicitly tests its undecided strict outcome;
passing that regression does not mean the geometry completed strictly.

## Cost decision

[The decision](scalar-decision-v1.json) keeps the capability extension with its
cost tradeoffs. This is an instrumented scalar probe on a shared, unpinned host.
Both revisions use the same dependency artifacts, optimization flags, fixture,
oracle, APIs and timing boundaries. Timings include scalar construction where
specified, result materialization, proof query and drop. Serialization and mesh
output are outside this scalar workload. Constant caches are not globally cold.

The initial three process pairs use 128 measured operations per row. All 18
completed-control medians are slower, up to 31.29%. Because some windows lasted
only tens of microseconds, five additional fixed process pairs use the same
frozen binaries at the probe's existing maximum of 4,096 operations per row.
Both sets are retained; the later observations do not invalidate the earlier
ones. Each range below spans the three APIs and reports median latency change.

| Completed control | Boundary | Initial batches | Longer batches |
| --- | --- | ---: | ---: |
| Rational | Rebuilt | +22.27% to +25.75% | -7.71% to -1.43% |
| Rational | Prepared | +15.71% to +22.60% | -10.54% to +3.39% |
| Positive sum | Rebuilt | +6.20% to +7.38% | -3.43% to -2.05% |
| Positive sum | Prepared | +3.29% to +31.29% | +1.50% to +4.66% |
| Unequal factors | Rebuilt | +0.15% to +26.51% | -2.73% to -2.26% |
| Unequal factors | Prepared | +0.79% to +8.27% | +0.88% to +1.76% |

The probe's `same_sign` case spells a positive sum as `a*b-(-b)*a` so all three
APIs have the same workload. Actual equal-sign flag arrays are separately
covered by the scalar regression. Individual process-pair ratios include larger
adverse variation than these medians; all raw rows remain in the
[short-batch assessment](scalar-assessment-v1.json) and
[long-batch assessment](cost-confirmation-assessment-v1.json).
No statistical significance, uniform speedup or downstream speedup is claimed.

Long batches record 368,640 certified baseline operations and 122,880 undecided
cancellations; the candidate certifies all 491,520 operations. Cancellation
has no latency ratio because the baseline did not complete the same proof.
Prepared candidate cancellation allocates nothing; rebuilding factors takes
124 allocations per operation. Existing control allocation counts and peak
live bytes are unchanged; every measured row retains zero additional live
bytes. Probe executable size increases by 656 bytes and the scalar archive
by 4,382 bytes. These are focused artifacts, not released-package sizes.

## Representation and resource contracts

Newly constructed cancelling products are canonical rational zero, so their
expression serialization can change. No public signature or wire schema
changes. Previously encoded expressions remain valid; they are not promised
automatic recertification. Consumers comparing deterministic artifacts must
account for this explicit representation change and retain revision-specific
baselines. This change is not classified as a performance-only optimization.

There is no new cache, refinement precision, tolerance or native workaround.
The helper makes at most four structural equality calls. Their existing DAG
walk can allocate and has no total-work, heap-byte or cancellation bound.
External 8 GiB per-process address-space and 1,800-second wall containment do
not establish internal resource guarantees. That limitation remains open.

## Remaining workflow and preserved evidence

The original body is a 12×12×4 cuboid cut by the represented radius-2,
height-6, 16-segment polygonal prism translated by `(6,6,-1)`. It is not an
analytic cylinder. Eight- and 24-segment strict controls remain certified;
16 and 32 remain undecided, with qualified approximate completion. Exact volume
comparison for the original 16-segment result remains unresolved.

The next neighboring-triangle coplanarity expression has the independent ring
identity `h(v-o)u-h(u-o)v-[h(v-o)o-h(u-o)o]=0`, where
`u=6-2*sin(pi/8)`, `v=6-2*sin(3*pi/8)`, `o=6-sqrt(2)` and `h=6`.
[The reduced probe](affine-baseline-v1/frontier.stdout) returns no rational
normal form and `Unknown` at precision 0, -512 and 0; adding exact `1/10^12`
certifies positive. No trigonometric identity is required for the cancellation.
The native diagnostic and original full-resolution fixtures are retained.

Earlier failures remain recorded: mixed native dependency types, two erroneous
precision arguments in the initial test harness, the genuine reversed-product
regressions, disk quota during qualification, a pre-write formatting check,
ambiguous dependency selection, and the missing expected `cfg(test)` declaration
in the first direct-rustc cost build. Retries use fresh output directories;
no failed evidence or original limit was replaced. Unrelated formatting was
restored after checking identical formatted Rust; included production files
were already noncanonical under standalone rustfmt before this patch.

The 35 pending native curve/DRC files remain byte-identical and unadmitted.
Their original full-board cost failures and seven previously reproduced circuit
fixture failures remain open and were not rerun here. No whole-workspace CI,
released-package, MSRV, real binding-consumer, sanitizer, formal-proof or
whole-plan completion is claimed. No push was performed. The next implementation
must prove the remaining identity in its scalar owning layer, preserve the
existing supported domain, and then recheck the complete drilling contract.
