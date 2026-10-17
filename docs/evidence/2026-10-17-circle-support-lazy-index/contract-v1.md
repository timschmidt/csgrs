# Lazy circular-support index admission experiment

Owner: Hypercurve's operation-local support-relation cache. Properties P4,
P10, P19, P20 and P36 apply. The E42 guarded index was rejected by E43:
complete cold batches of 4, 8 and 9 pairs regressed 16–19.5%. Main code still
uses ordered linear lookup. Reproduce that finding on the current revision
before changing the isolated proposal.

Hypothesis: a small batch should not initialize, populate or hash an index it
never uses. Keep its exact ordered lookup. After at least 32 retained relations,
an eligible exact rational query can lazily build an index from the retained
entries. Preserve the earliest matching entry, including unindexed entries
preceding an indexed hit. Symbolic and `VeryLarge` scalar supports retain exact
ordered fallback. The threshold is a cost choice, not an input restriction.
Index construction must retain the same owned support values and witnesses;
no source pointer, approximate key, float projection or discarded proof is used.

The cache still belongs to one synchronous operation under one policy. No
internal total-work or heap-byte bound is introduced. External containment is
1,800 seconds and 8 GiB address space per command, with repository, runtime and
host-memory reserves of 768 MiB, 256 MiB and 2 GiB. Preserve closed outputs and
source snapshots. New Git evidence may use a lossless archive with per-member
hash verification to fit storage; original physical logs remain intact.

Keep all eight existing cache contracts unchanged and add mixed symbolic,
million-bit rational and ordinary rational transitions through 80 retained
relations, rebuilding owners and checking changed geometry. Both diagnostic
variants receive identical new semantic tests and benchmark modules. Independent
3-4-5 secant equations and strict receipts remain the correctness oracle.

Repeat E43's complete small-batch experiment with every original size plus 31,
32, 33 and 64, exposing both sides of index activation. Each phase performs at
least 65,536 queries and includes exact validation, ordered output construction,
comparison and destruction. Cold includes per-batch cache construction and
destruction; warm excludes preparation and final cache destruction. Inputs,
reference preparation, printing and process startup are outside phase timing
and inside process resource observations. Process/scalar state is reused.

Run a preflight followed by seven matched repetitions per size/variant, with
deterministically shuffled case order and alternating variant order. Preserve
all samples. The same decisive rejection rule applies: a median regression above
10%, all seven paired differences positive, and separated sample ranges.
Absence of that signal is not proof of zero overhead or permission to conceal
other unfavorable observations.

If small-batch behavior fails, reject the proposal as written before spending
resources on broader qualification. Otherwise preserve the original rational,
symbolic, mixed, superrange, subrange and shared-large-coefficient corpus;
then require proportional allocation/size evidence, affected library and
downstream validation, original native workload/report gates and real bindings
before retention. This private-kernel experiment cannot establish release,
pre-push, coverage, campaign, proof or whole-plan completion.
