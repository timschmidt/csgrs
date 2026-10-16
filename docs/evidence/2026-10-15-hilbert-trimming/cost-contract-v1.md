# Original caller cost observation

Run the unchanged `profile_primitives/constructor/hilbert_curve` fixture:
square side 8, order 3, padding 1. Baseline is the retained E14 all-feature
benchmark executable (`362e157c1` source graph); candidate is the E15 final
all-feature build. Both include `dispatch-trace`. Record executable hashes,
source/dependency pins, toolchain, host snapshot, commands, stdout CSV, stderr
dispatch counts and `/usr/bin/time -v` resources.

Fix five alternating baseline/candidate pairs for each of two conditions:

- Cold: fresh process, zero explicit warmups, one iteration per timed row.
- Warm: fresh process, one eight-iteration warmup, eight timed iterations.

Do not extend or filter the matrix based on results. Each timed closure
constructs the square, constructs and materializes the returned strings, reads
the run count, and drops the result. There is no export or rendering in this
timing boundary. A fresh process is not a claim that operating-system caches
are evicted. No core affinity or frequency isolation is applied.

Require identical work-unit/output-size/checksum fields for corresponding
conditions, and retain every result. These fields count strings; independent
grid/interval tests establish the actual geometry. `/usr/bin/time` peak RSS
covers the entire process, including untimed setup and later constructors;
it is not an isolated allocation count or a bound on native trimming.

This is a cost disclosure for a correctness migration. The square output was
already correct, but the new implementation computes exact trim evidence
before providing the narrower string view. Baseline midpoint selection does
not satisfy the general clipping contract. No faster-operation or cross-library
claim follows from these observations. Keep unfavorable latency and memory
results; the broader resource contract remains open.
