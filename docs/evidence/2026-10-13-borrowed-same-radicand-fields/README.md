This experiment tests a private same-radicand helper that borrows checked
coefficient fields. The controls are the unchanged endpoint/P4 parent and the
previously rejected outlined version. No runtime is admitted.

The [source review](scope-v1.json) establishes disjoint mutable borrows of the
left constant, checked radical pair and empty atom vector, plus shared borrows
of the right constant and pair. The caller keeps the original opaque-product
guard and singleton/equal/non-unit eligibility checks. The helper performs no
vector indexing and adds no scalar comparison, clone, unsafe access, public
API or cache field.

The helper returns `Some(Rational)` on cancellation without replacing input
fields. The caller then constructs the allocation-free rational carrier and
drops the consumed input. On survival, the helper assigns the selected
radicand, constant, coefficient and empty atom vector in the original order.
Releasing atom capacity inside the helper preserves its ordering before
remaining local drops. `None` means the borrowed fields contain the one-radical
product; the enclosing multiplier still returns `Some` for every eligible
product.

Declared substitutions map the outlined arithmetic suffix to the borrowed
one, preserving arithmetic order, zero skipping and operand forms. Removing
the helper/branch and the receiver's `mut` recovers the original general method
exactly. Cache storage, memo publication, mandatory retry, work/depth budgets
and the stronger-facts P4 contract remain inherited.

The previous outlined trial reduced the multiplier symbol but left its first
stack subtraction at 264 bytes and slowed cloned conjugate queries in every
pair. Those observations motivate this source review; they do not attribute
the timings to stack size or bounds checks. Borrowed argument passing and
result dispatch can also add cost. Early comparisons and reused vector
capacity retain the direct-product risks.

All 28 shared controls remain byte-identical. The three fresh sources will
receive 42 sequential qualification commands: 902 all-feature and 815 default
library tests per variant, 28 doctests, shared/cache/P4 controls, formatting,
strict default/all-feature all-target Clippy and warning-denied scalar docs.
These counts overlap.

If qualification passes, unchanged rational, quadratic and integer probes
will run 18 preflights and 90 measurements after all builds finish. Five fixed
three-source blocks use separate timing/allocation executables and 4,096 timing
or 1,000 allocation inputs per phase. Every sample, witness, representation
inventory, memory field and probe size remains. Comparisons share samples and
are not independent; earlier trials are neither extended nor erased.

Static inspection, if reached, will read the existing binaries and store
losslessly compressed disassembly plus raw-output hashes. It will introduce
no additional cost sample. Native correctness, original full-fixture gates,
matched native costs, full report bytes, production CLI size and the remaining
workflow matrix stay separate retention requirements. The original limits
remain 120 seconds of board worker CPU and 60 seconds of neck check-body wall.

The full plan remains active, with all 39 properties, 14 targets and crate
future work in scope. Main runtime, the original 35 pending native paths and
individual property/target statuses are protected by the source guards.

Preparation passed. Each frozen source contains 10,136 files. Baseline and
outlined match their previously qualified sources byte for byte; only the
declared algebra file differs across variants. Shared-test identity, reviewed
arithmetic substitutions, original general fallback recovery, manifest/lockfile
identity and all parent/main/native source guards pass. Source records are
committed before compilation.

All 42 qualification commands passed. The [independent assessment](correctness-v1.json)
verifies every command, log and source binding. Each variant passes 902
all-feature tests, 815 default tests, 28 doctests, all shared/cache/P4 controls,
formatting, both strict Clippy configurations and warning-denied docs. Counts
overlap and do not cover the complete workflow matrix.

All eight recorded layouts match. Memo inventories remain 1/1/2 and ordinary
copy sharing remains 0/0. Outlined and borrowed reuse the capacity-one input
buffer in the surviving-radical control and release radical capacity on
cancellation; the baseline capacities are four. Atom capacities are zero
throughout. These observations do not establish total-memory or timing gains.
