# Alumina FOC interval conformance client

This standalone, unpublished client compares Hyperinterval fixed arithmetic and
Hyperlattice matrix products with Alumina's current Clarke/Park transforms. It
uses the public Alumina wrappers, validated rotations and error types. It does
not replace firmware code or supply a second geometry or scalar kernel.

Place `csgrs` and `alumina-firmware` next to each other. From the CSGRS root:

```sh
cargo +1.88.0 test --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml -- --nocapture
cargo +1.88.0 clippy --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --all-targets -- -D warnings
cargo +1.88.0 run --locked --release --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --bin cost
cargo +1.88.0 check --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --lib --target thumbv7em-none-eabihf
cargo +1.88.0 build --locked --release --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/wasm/Cargo.toml --target wasm32-unknown-unknown
```

The production dependency graph of Hyperinterval/Hyperlattice is unchanged. This
client has its own lockfile and only four dependency packages, all local: Alumina
FOC/protocol and Hyperinterval/Hyperlattice interval algebra. The library and Wasm
consumer are `no_std` and need no allocator; tests and native probes use `std`.

`clarke` keeps wide Q30 numerators until division/scaling, including differences
that exceed i32 but have representable outputs. Park and its inverse retain the
current sequence of rounded products and sums/differences. For example, the
current Park implementation can reject an intermediate `MIN * -1` even when the
mathematical final result fits. A signed matrix rewrite must not silently change
that failure contract. Separately rounding the Clarke coefficients `2/3` and
`-1/3` changes exact output endpoints. Both cases are maintained regressions.

The corpus compares full outputs and typed failures, then checks successful
outputs against independent exact integer corner expectations. It covers full
i32 endpoints, zero-crossing intervals, valid uncertain rotations, balance
rejection, seeded intervals, widened products/divisions and repeated transforms.
The cost probe first verifies equal completed results, then reports timing,
allocation and checksums for both implementations. `native-probe` and
`shared-probe` run identical workload bodies for executable-size comparison.
These probes do not establish firmware cycle, stack, HIL or performance gates.
Measurements and adoption decisions belong in the main progress/evidence ledger.
