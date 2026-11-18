# Alumina differential client

`alumina_q30.rs` compares the shared kernel with the actual, unchanged
Alumina FOC implementation. It runs 20,736 endpoint-boundary combinations,
100,000 deterministic random cases and 81 extreme i64 ratios. Every result
endpoint or mapped typed error must agree. Independent mathematical enclosure
tests remain in Hyperinterval; agreement with a legacy implementation alone
is insufficient.

Run from the repository root:

```sh
python3 crates/hyperinterval/scripts/alumina_conformance.py \
  --firmware ../alumina-firmware --output /tmp/interval-alumina
```

This creates an isolated dev-only Cargo workspace under the selected output
directory. Firmware is not a production or normal dev dependency of the
published Hyperinterval package. The explicit output directory retains the
manifest, lockfile and log for evidence. By default the client uses Rust 1.88.

Add `--bench` to collect 30 paired alternating samples for five Q30 operations,
with exact output checks and separate warmup. The probe measures out-of-line
calls on four successful input cases, including construction and materialized
endpoints. It is not a full FOC, embedded-cycle or HIL benchmark. Raw samples
stay in the chosen output directory; do not compare a successful row with an
overflow/invalid row.

### Fixed-point Cartesian images

The interval WASM runner builds the standalone fixed-point Hyperlattice
client. It checks the affine, norm, width, midpoint/radius and contraction
exports in two fresh instances. The affine-chain oracle follows all four
source corners through each step. See the per-client results under
`fixed_lattice` in the runner's `report.json`.

### Real-to-dyadic cancellation and retry

The native and bare-WASM consumers run the same exact checks on
`sqrt(109) - 7/5`. They cover cold and warm cancellation, independent
readers of a shared result, conversion-budget exhaustion
and finer-precision retry. The native target also exercises concurrent
readers. The oracle translates the exported dyadic endpoints back and
compares their squares with 109 using exact rational arithmetic.

The interval WASM runner executes this corpus on toolchains meeting the
client’s declared Rust version. To run the client on its own, use the
following commands from the CSGRS root with Rust 1.95.0, the
`wasm32-unknown-unknown` target and Node.

```sh
cargo +1.95.0 test --locked -p hyperinterval --features hyperreal --test real_dyadic_lifecycle
client=crates/hyperinterval/conformance/wasm-real-bridge
cargo +1.95.0 build --locked --release --manifest-path "$client/Cargo.toml" \
  --target wasm32-unknown-unknown --target-dir target/interval-real-bridge
node "$client/check.mjs" target/interval-real-bridge/wasm32-unknown-unknown/release/interval_real_bridge_wasm_conformance.wasm
```
