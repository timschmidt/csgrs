This experiment extracts the qualified direct same-radicand arithmetic into a
private `#[inline(never)]` helper. It compares three fresh sources: the unchanged
parent, the rejected direct version as a control, and the helper version.
No runtime is admitted.

The [source review](scope-v1.json) establishes the helper contract before
compilation. The single caller first applies the original opaque-product guard,
then proves singleton vectors and equal non-unit radicands. Neither input is
mutated before the helper, so both index-zero reads and later writes are in
bounds. The left coefficient moves into the helper and the right remains
borrowed. No additional owner or scalar comparison is introduced relative to
the direct version.

The arithmetic suffix is preserved exactly apart from indentation and moving
the `Some` wrapper to the caller. Operation order, zero skipping, left/right
radicand payload selection, vector reuse and cancellation behavior remain.
Removing the eligibility branch and helper recovers the original general
multiplier byte for byte. Cache fields, memo publication, traversal budgets,
mandatory retry, and the stronger-facts P4 contract remain inherited.

The earlier direct trial found eight consistently slower scalar phases while
saving allocations in six. Its coefficient symbol grew from 2,473 to 4,161
bytes, and the first stack subtraction from 248 to 264 bytes. Those static
observations motivate this experiment; they do not identify the cause of the
slowdowns or measure total stack use. A helper call and its bounds checks may
also add cost, and the direct version's early-comparison and capacity risks
remain.

All 28 shared test functions are unchanged and byte-identical across sources.
Each source will receive focused controls, 902 all-feature and 815 default
library tests, 28 doctests, formatting, strict default/all-feature all-target
Clippy, and warning-denied scalar documentation: 42 sequential commands.
Focused and configuration counts overlap.

If qualification passes, three unchanged scalar probes will run 18 preflights
and 90 measurements after all builds finish. Five fixed three-source blocks
use 4,096 timing inputs or 1,000 allocation inputs per phase, with separate
timing and allocation executables. All samples, witnesses, representation
inventories, requested-memory fields and probe sizes remain. The three
contrasts share samples and are not independent experiments. No earlier trial
is extended.

Native qualification, the original 120-second board worker-CPU and 60-second
neck check-body-wall limits, matched native costs, complete report bytes,
production CLI size and remaining workflows are separate retention gates.
All 39 properties, 14 targets and crate future work remain in scope. Existing
main runtime, the 35 pending native paths, and individual ledger statuses are
protected by source hashes.

Preparation passed. Each frozen source contains 10,136 files. The baseline
and direct sources match their previously qualified sources byte for byte;
only the declared algebra file differs across variants. Exact arithmetic and
fallback recovery, shared-test identity, manifest/lockfile identity, and all
parent/main/native source guards pass. The frozen maps and deltas are committed
before compilation.
