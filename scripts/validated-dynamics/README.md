# Validated dynamics comparison

This standalone benchmark compares Hyperdynamics with CAPD::DynSys on the same
finite IVP, endpoint and whole-horizon tube contract. CAPD is an external process;
it is not a Hyper stack dependency or a replacement for its exact source/proof
receipts. No third-party implementation is vendored here.

The corpus has four autonomous fields: `x'=2`, `x'=x`, `x'=x²`, and
`(x',y')=(y,-x)`. Each uses a point initial state and a box initial state.
The scalar state starts at `1`, or `1±2^-10`; the oscillator starts at `(1,0)`,
or that point plus `[-2^-10,2^-10]²`. The horizon is exactly `[0,1/4]`.
All input values have identical exact meanings in both backends.

Success requires the complete certified horizon, an endpoint enclosure of width
at most `1/64` per coordinate, and a whole-horizon tube hull of width at most `1`
per coordinate. The retained native trajectory certificates supply the tube;
sampling is not used. `validate.py` checks every output against independent
rational analytic bounds. Exponential uses its positive series and a geometric
tail bound; sine/cosine use alternating-series bounds; Riccati has an exact
rational solution. Oscillator extrema over the complete initial rectangle and
time interval are checked, including the interior maximum of the first coordinate.
Square-root bounds for that extremum use integer square roots with exact rational
verification. Every actual endpoint and tube width is recorded.

Hyperdynamics uses prepared order 3 with the existing default enclosure policy.
CAPD uses order 12, `IOdeSolver`, `C0Rect2Set`, and an interval `SolutionCurve`.
Both cap a step at `1/16`. These are explicit configurations, not a claim of
optimal tuning or identical internal algorithms. Both must pass the same quality
limits; stronger output is reported rather than rounded to the limit.
CAPD's `TimeMap` replaces its solver's maximum-step setting, so its adapter
advances through four exact checkpoints. Both adapters check every retained
piece duration, and the independent validator checks the published time domains.

`reuse` excludes source parsing/compilation and solver/coefficient preparation.
`fresh` includes those costs. Both include integration, retained native evidence,
materialized endpoint/tube output, and destruction of that evidence/output.
The full Hyperdynamics source and rejected-attempt receipts remain in the report;
CAPD retains its native solution-curve certificate. These are different evidence
and resource APIs. Serialization and the independent oracle run outside timed
loops. Eight warmups precede each mode. Results must remain byte-identical across
fresh/reused preparation and all measured blocks within each adapter.

The adapters emit exact dyadic triples or exact IEEE hexadecimal endpoints.
The Python oracle converts both to rational numbers without decimal rounding.
Graph/scalar work counters describe Hyperdynamics integration only; they exclude
preparation and the separate tube-publication budget. CAPD has no equivalent
counters in this adapter. Neither counters nor the runner timeout are a universal
memory/total-work bound. The runner has a 600-second process timeout; failures are
not included as successful timing samples. No allocation comparison is implied.

## Build and run

Use Rust 1.95.0, Python 3 with assertions enabled, GCC with C++17, CMake, and Linux
`taskset`/`setarch`. Choose an output directory that does not already exist.
Build the official [CAPD repository](https://github.com/CAPDGroup/CAPD) at commit
`37057e5899bccaa6cd5701057b7e6ea5e232ef8b` (version 6.1.0):

```sh
cmake -S "$CAPD_SOURCE" -B "$CAPD_BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DCAPD_INTERVAL_TYPE=NATIVE -DCAPD_BUILD_TESTS=ON
cmake --build "$CAPD_BUILD" --parallel 20
ctest --test-dir "$CAPD_BUILD" --output-on-failure --parallel 20
```

The upstream tests require Boost headers and `unit_test_framework`. The recorded
run used Debian packages extracted into a local directory and explicit
`BOOST_ROOT`/`BOOST_LIBRARYDIR`; it did not install system packages. CMake 3.31
also used `CMAKE_POLICY_DEFAULT_CMP0167=OLD`, `CMAKE_POLICY_DEFAULT_CMP0144=NEW`,
`Boost_NO_SYSTEM_PATHS=ON`, and `Boost_NO_BOOST_CMAKE=ON`. The evidence records
exact commands and package hashes. Follow the upstream build documentation for
other environments; do not disable the tests to hide failures.

From the CSGRS root:

```sh
python3 scripts/validated-dynamics/run.py \
  --capd-source "$CAPD_SOURCE" --capd-build "$CAPD_BUILD" \
  --cargo-target "$COMPARISON_TARGET" --output "$COMPARISON_OUTPUT" \
  --iterations 100 --samples 5
python3 scripts/validated-dynamics/check_rejections.py \
  "$COMPARISON_OUTPUT/hyperdynamics-preflight.log"
cargo +1.95.0 clippy --locked --manifest-path scripts/validated-dynamics/Cargo.toml \
  --all-targets -- -D warnings
```

The runner keeps CAPD's required `-frounding-math` and native interval definition,
uses external headers as system headers, and compiles the adapter with warnings
denied. Separate processes isolate CAPD's rounding environment. It pins one CPU,
disables ASLR, and runs Hyperdynamics/CAPD/CAPD/Hyperdynamics blocks. Each block
validates the complete corpus before accepting its samples. Avoid other workloads
during measurement. The runner records compiler versions, source/binary hashes,
CPU information, affinity, commands, raw samples and exact certificate transcripts.

This small corpus does not establish broad competitiveness, long-time stability,
stiff-system behavior, high-degree symbolic support, or equal replay interfaces.
See the [recorded comparison](../../docs/research/validated-dynamics-comparison.md).
