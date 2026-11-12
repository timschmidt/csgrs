# Alumina measurement conformance client

This unpublished standalone package checks the exact arithmetic boundary between
Alumina's unit-bearing `BoundedMeasurement` and Hyperinterval. Keep `csgrs`,
`alumina-interface` and `alumina-firmware` beside each other. From the CSGRS root:

```sh
cargo +1.95.0 test --locked --manifest-path crates/hyperinterval/conformance/alumina-measurements/Cargo.toml --features trace -- --nocapture
cargo +1.95.0 clippy --locked --manifest-path crates/hyperinterval/conformance/alumina-measurements/Cargo.toml --all-targets --all-features -- -D warnings
cargo +1.95.0 run --locked --release --manifest-path crates/hyperinterval/conformance/alumina-measurements/Cargo.toml --features probes --bin cost
cargo +1.95.0 run --locked --release --manifest-path crates/hyperinterval/conformance/alumina-measurements/Cargo.toml --features probes --bin probe -- 6
cargo +stable build --locked --release --manifest-path crates/hyperinterval/conformance/alumina-measurements/wasm/Cargo.toml --target wasm32-unknown-unknown
```

The tests retain independent common-denominator expectations, both physical unit
types, negative-radius rejection, exact replay and subsequent arithmetic. Large
rational fixtures supplement the small grid. Original-formula and public-wrapper
trace counts, certainty receipts and allocations are compared in separate fresh
processes, with cold and explicitly warmed inputs. Recreating an equal small
rational in the same process can reuse interned facts; it does not establish
equal cache conditions.

The `probes` feature enables native release binaries using Alumina Interface's
release settings. Keep this feature out of release test builds: tests unwind,
whereas the application probes abort. `cost` times
complete repeated construction and destruction with prepared inputs and reports
allocation calls/bytes. `probe` prints full exact endpoint text for a runtime
fixture index. Use matching source and build settings for before/after comparisons.

The Wasm export hashes complete exact endpoint text. Zero denotes negative
uncertainty, `u64::MAX` denotes an invalid denominator, and `u64::MAX - 1` denotes
an unexpected kernel error that the oracle must reject. The independent Python
Fraction oracle and captured artifact measurements live in the evidence archive.
The raw module includes binding imports from Interface's dependencies; the
measurement path must execute without calling them. This does not substitute for
an application browser, schedule-latency or whole-UI memory qualification.

Production migration decisions, adverse measurements and remaining gates are
recorded in `INTERVAL_REFACTOR_PROGRESS.md` and its referenced evidence records.
