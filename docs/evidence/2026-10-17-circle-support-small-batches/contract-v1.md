# Repeated small circular-support batches

The owner is Hypercurve's private, operation-local circular-support relation
cache. P4, P10, P19, P20 and P36 apply. The guarded E42 proposal is unadmitted;
its exact rational index and its small-batch and coefficient-size guards are
unchanged in this experiment. Production remains unchanged.

The missing acceptance evidence is a stable comparison of completed small cold
batches. E42's single cold batch showed unfavorable microsecond differences;
those observations do not establish their steady-process cost. Both variants
will run identical fresh diagnostic source and use the same pinned dependencies,
compiler, flags and feature set. No completed target will be reopened.

The domain is pairs of full circles with rational centers `(32n, 0)` and
`(32n + 6, 0)`, both radius 5, under strict policy. Every query must return two
certified secant witnesses `(32n + 3, -4)` and `(32n + 3, 4)`, in the kernel's
retained order. Each witness is checked independently against those coordinates
and both circle equations. Ordered exact output rows are materialized on every
batch, compared across repetitions and phases, and preserved for cross-variant
comparison. Equality of wrappers or checksums alone is not the oracle.

Batch sizes 1, 4, 8, 9 and 16 exercise both sides of the small-batch guard; 128
is an indexed-workload control. Each measured phase performs at least 65,536
queries in the same process. Cold means a new cache per batch, including its
construction and destruction. Warm means one prepared cache repeatedly queried;
preparation and its final destruction are outside the warm timer. Both timers
include querying, all exact validation, complete ordered-row materialization,
row comparison and row destruction. Input construction, first-batch reference
preparation, output printing and process startup are outside both timers and
remain included in process resource observations. This is a private-kernel cost
comparison, not a public modeling or distributed-package benchmark.

Run a preflight and then seven repetitions per size and variant in deterministic
shuffled case order, alternating variant order. Preserve every sample, including
unfavorable ones. A median regression above 10% with all seven paired differences
positive and nonoverlapping sample ranges is a decisive rejection of this
proposal as written. Smaller or noisy differences do not prove zero overhead;
other acceptance gates remain required for retention. This criterion is set
before collecting the new measurements.

External containment remains 1,800 seconds and 8 GiB address space per command,
with root, runtime and host-memory reserves of 768 MiB, 256 MiB and 2 GiB.
There is no claim of an internal work or cache-byte bound. Stop on an external
resource guard; retain partial evidence without claiming completed validation.

Record source/dependency/toolchain/hardware hashes, raw commands, results and
timing boundaries. Verify unchanged production and frozen inputs afterward.
This experiment cannot establish the still-open downstream, Cargo matrix,
release, binding, coverage, fuzzing or formal-proof requirements.
