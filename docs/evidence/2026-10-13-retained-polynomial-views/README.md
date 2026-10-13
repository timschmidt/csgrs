This experiment starts from the frozen endpoint/P4 parent and tests immutable
views of an entire retained Laurent polynomial. The previous
[owned-radical trial](../2026-10-13-owned-radical-scratch/README.md) passed scalar
correctness but was rejected after fixed costs showed repeated slowdowns and
increased requested memory. Its source and samples remain frozen.

The new private result enum distinguishes an ordinary owned polynomial from an
Arc to a closed polynomial. Cache reads clone that Arc under the existing read
lock, then release the guard. Closed coefficients retain their original types:
only Rational values and radical vectors, with no Computable fields. Publishing
uses an Arc instead of a Box, so its reference-count header adds retained bytes.
No new shared allocation is introduced for ordinary scratch arrays.

Multiplication materializes its left operand. An owned right operand uses the
existing algorithm; a retained right operand lends its closed coefficient
slices. Both call the same coefficient multiplication body. Preparation checks
the borrowed polynomial loop against the original operation and insertion
sequence. Addition, scaling, inversion and square-root inspection materialize
owned values before the original algorithm. The existing projection wrapper
also materializes its result, so the rational, pi, quadratic and compacting
consumers remain unchanged. This version does not eliminate every copy.

The normalizer still computes owned results, inserts its original deep memo
copy and then publishes an eligible closed certificate. Cache hits return
early. Opaque node ownership, arithmetic order, all recognition limits and the
mandatory original retry after releasing the first memo remain requirements.
No additional recognition pass, approximation, dependency or public API is
introduced. P4 allows stronger facts after warming, never contradictions or
weaker facts.

Both variants run the same eight storage, three memo and three alias controls,
plus inherited cache and P4 checks. The only shared-test edit adds a cache-layout
observation; preparation recovers the original mathematical tests exactly.
The previous radical-sharing inventory should be 0/0 for both variants because
ordinary `to_polynomial` still copies. Five candidate controls exercise actual
retained views: shared buffers and graph release, exact arithmetic across all
ownership combinations, term/radical/power frontiers, concurrent mutation
isolation, and opaque-left ownership and declined products.

All 26 qualification commands passed in fresh targets. Baseline passes 888
all-feature and 801 default scalar library tests; candidate passes 893 and 806,
including the five retained-view controls. Both pass 28 doctests, focused shared
and inherited controls, formatting, strict default/all-feature/all-target Clippy
and warning-denied docs. Counts overlap and do not establish full-workflow
coverage. The [independent assessment](correctness-v2.json) verifies every log,
command, source binding and inventory.

The eight existing type layouts match, including 56-byte scratch coefficients,
32-byte closed coefficients, 24-byte owned polynomials and 64-byte cache values.
The new private result enum is 24 bytes. Original memo entries remain 1/1/2 for
retained/original/opaque controls, and ordinary radical-copy inventories remain
0/0 in both variants. Candidate tests separately verify actual shared closed
payloads and their release. Arc headers are additional heap costs and are not
accounted for by these private type sizes.

The next fixed scalar trial executes twelve preflights and 60 measurements
across the three unchanged probe suites. Builds precede every probe; timing and
allocation use separate executables. Every phase, sample, adverse result,
layout and probe size remains in scope. Neither the source design nor old
recursive-copy counts prove an improvement.

Version 1 preparation stopped before compilation because rustfmt folded three
calls and removed optional trailing argument commas. The exact-token guard
rejected those generated fragments. The failed run, both formatted source
trees and their hashes remain preserved. An independent
[failure assessment](preparation-failure-assessment-v1.json) recovers every
parent token after explicitly accounting for those three commas.

Version 2 changes only the preparation's expected call fragments. Preparation
passed and froze fresh 10,134-file baseline and 10,135-file candidate trees.
Their maps exactly match the preserved formatted version 1 trees, including
every Rust and mathematical test byte. Exact parent-token recovery and the
borrowed multiplication's operation sequence are verified. Baseline and
candidate differ only in algebra.rs, representation.rs and the new private
helper. Qualification passed; costs have not run. See
[preparation-v2/protocol.json](preparation-v2/protocol.json).
See [scope-v2.json](scope-v2.json) for the correction, contract and cost risks.
Native validation, original board and neck
limits, endpoint/broad/abort controls, complete report bytes and production CLI
size remain requirements before runtime admission. Main runtime and all 35
pending native entries remain unchanged. All 39 properties, 14 targets and
crate-specific future work remain in scope.
