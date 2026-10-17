# Independent metadata qualification while full-build storage is pending

This supplements preparation; it does not replace any required execution,
performance, native workload, resource, downstream or binding gate.

A direct Rust/Clippy test-harness check with `--emit=metadata` can check the
actual new method and all nine retained semantic tests without creating an
executable or running a test. Use a fresh source snapshot and the same verified
no-feature dependency libraries, edition and compiler as E44. Keep main and
all completed source/build roots unchanged. The copied candidate uses the
reviewable proposal's production source and semantic-test module hooks; the
three ignored timing-module hooks remain absent.

Budget this distinct preparation operation before executing it: at most 32 MiB
of planned copied source, 1,800 seconds and 8 GiB address space per compiler
command, and at least 2 GiB host MemAvailable. The private runtime output volume
must retain 256 MiB free and is checked every five seconds. Reject preparation
if its planned copy cannot leave that reserve. Compiler temporary files and
metadata output all live in the fresh runtime directory. Repository input
files are read-only; preparation controls and the eventual compact Git record
must fit within a separately declared 2 MiB repository-write allowance, with
at least 512 MiB free on that volume before this preparation starts.

The previous 768 MiB repository-volume execution reserve is not changed for
any completed experiment. This is a newly declared, smaller-footprint metadata
operation with no executable, workload result or performance claim. The
original native CPU/wall-time gates, fixture resolutions and policies are
unchanged. Full downstream validation still needs the requested larger writable
build volume and a declared layout before it can run.

A compiler or Clippy pass establishes type/lint validity only. Do not credit
it as any semantic-test pass, removal of the measured regression, cache
admission or completion of the properties plan.
