# Isolated executable qualification within the remaining shared-memory quota

This is a new bounded experiment after the metadata-only check completed.
It does not relax any completed experiment's contract or the original native
workload limits. Full downstream storage remains pending.

`/dev/shm` is executable and has 524,210 KiB of user-quota headroom. A fresh
candidate source copy and one approximately 47 MB diagnostic executable fit
there. Reuse the E44 baseline binary read-only only after verifying its source,
test/diagnostic modules, feature set, flags, compiler and dependencies. Rebuild
the candidate with the same diagnostic hooks and nine unchanged cache tests.

Use a new private output root, never a completed Cargo target. For every
command, enforce 1,800 seconds, 8 GiB address space, 2 GiB host MemAvailable,
and 256 MiB remaining `/dev/shm` user quota. Query quota explicitly; filesystem
free bytes alone overstate this user's capacity. Root-volume input code is
read-only during compiler/test execution. Declare a 512 MiB minimum root free
space for this new experiment's small control files and at most 4 MiB of new
repository controls/evidence. These bookkeeping allowances do not change any
geometry operation, precision policy, native CPU/wall-time gate or fixture.

First run the nine exact contracts and strict diagnostic Clippy. Then preserve
all ten E44 small-batch sizes, the full fifteen-case support matrix, and the
seven-group shared-support comparison. Every completed result must retain its
independent oracle, exact output rows and certainty. Keep every unfavorable
sample. The predeclared decisive rejection criterion remains median cost above
baseline by more than 10%, all seven paired differences positive and separated
sample ranges. No threshold is rounded down to obtain a desired decision.

This no-feature executable can also run its full library corpus from a fresh
source mirror if the quota reserve permits. An initial no-feature pass does
not establish all-feature, Cargo-matrix, downstream, native/report, binding or
release acceptance. Record every missing gate and leave the proposal unadmitted
until the required work actually completes.
