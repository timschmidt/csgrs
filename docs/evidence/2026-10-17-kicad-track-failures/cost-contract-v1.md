# Loading cost and output preservation

This is a correctness change with an explicit source-failure migration, not a
speed optimization. Preserve the original complete test suite and all three
full-scale workload gates. No new threshold replaces those limits.

After qualification, compare the current baseline and candidate from their
read-only release libraries and CLI executables. Use five alternating paired
processes for each of three identical inputs: 64 ordinary straight tracks and
the complete CPArti and HVP109A source boards. Each process measures a fresh
load and a repeat load after projecting and dropping the first board. No
policy, resolution, source, or geometry simplification changes between sides.

The measured interval includes file reading, parsing, and materialization of
the returned BoardModel. Report projection, output serialization, process
startup, and destruction are outside it and explicitly excluded. The counting
allocator reports calls, requested bytes, retained requested bytes and peak
additional live requested bytes. These are not heap-resident-byte or expression
growth measurements. Process resources are retained separately. No internal
total-work, heap-byte, parsing-depth or cancellation bound is claimed; use
external 1800-second / 8-GiB process containment.

Compare exact serialized source metadata and every finite report coordinate's
binary64 bits, including retained projection failures. This checks preservation
of the legacy representation, not exact fidelity to KiCad's intended stroke.
The public source tests provide independent expected failures and known point
membership; wrapper/report agreement is not an independent geometry oracle.
Every process must load successfully and preserve the same representation.
Count retained feature failures separately from successful projections.

Run actual baseline and candidate CLIs on those same full inputs with selected
checks. Preserve all raw reports and compare every JSON leaf. The sole declared
report migration is readiness check_version 20 to 21; any other difference
must be investigated and recorded rather than normalized away. Run malformed
CLI inputs separately; faster rejection never counts as successful geometry.

Keep every pair, adverse latency/allocation row and executable size. Measured
validation overhead can be retained under the governing exactness/completeness
priority if the original completed-workload gates remain satisfied. Do not
infer production latency or a universal speed claim from instrumented samples.
