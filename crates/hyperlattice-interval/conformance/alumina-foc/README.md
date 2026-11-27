# Alumina FOC interval conformance client

This standalone, unpublished client compares Hyperinterval fixed arithmetic and
Hyperlattice matrix products with the checked-out Alumina Clarke/Park transforms. It
uses the public Alumina wrappers, validated rotations and error types. It does
not replace firmware code or supply a second geometry or scalar kernel.

Place `csgrs` and a compatible `alumina-firmware` checkout next to each other.
The client's lockfiles require the Alumina FOC package's Hyperinterval dependency;
an older checkout with a different dependency graph cannot use these locked builds.
Local path dependency contents are not pinned by Cargo.lock. Record both source
revisions when comparing results. From the CSGRS root:

```sh
cargo +1.88.0 test --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml -- --nocapture
cargo +1.88.0 clippy --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --all-targets -- -D warnings
cargo +1.88.0 run --locked --release --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --bin cost
cargo +1.88.0 check --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --lib --target thumbv7em-none-eabihf
cargo +1.88.0 build --locked --release --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/wasm/Cargo.toml --target wasm32-unknown-unknown
node crates/hyperlattice-interval/conformance/alumina-foc/wasm/check.mjs
```

The Node runner accepts an explicit Wasm file path when using a custom Cargo
target directory, followed by an optional new JSON replay-file path. It checks
full tagged endpoints and range outcomes against the integer oracle in two fresh
instances. The guest panic handler spins; use an external timeout when running
the driver directly. Replay-file creation happens after validation and refuses
to overwrite an existing file.

Generate the library's public and private API documentation with:

```sh
cargo +1.88.0 doc --locked --manifest-path crates/hyperlattice-interval/conformance/alumina-foc/Cargo.toml --lib --no-deps --document-private-items
```

To execute the same calls as linked Cortex-M4 instructions, install the
`thumbv7em-none-eabihf` and `wasm32-unknown-unknown` Rust 1.88.0 targets.
With Node and `uv` on the host, run this command from the CSGRS root.

```sh
uv run crates/hyperlattice-interval/conformance/alumina-foc/check-thumb.py
```

The simulator builds both targets, replays every oracle-checked call on the
Thumb ELF, and checks the return value, stack pointer and r4–r11 registers
at the function boundary. Each call has a one-million-instruction limit; the whole
command has no internal wall-clock limit. Two fill patterns measure touched stack
spans on fresh and reused simulator states, not the minimum stack pointer or a
universal stack bound. The fixture has a 64 KiB read-only executable image slot
with 16 KiB reserved and an 8 KiB stack with a 4 KiB unmapped reserve.
Pass `--output <new-directory>` to retain the ELF and the checked call list.
The explicit directory must not exist and can retain partial files after failure.
Without it, ELF/replay files are temporary; Cargo build artifacts remain in
`CARGO_TARGET_DIR` or `wasm/target`. The script resolves its real location, so a
symlink to it does not change the required sibling-checkout layout.
The pinned Python dependencies are host tools. Physical board cycles,
interrupt stack use and deployment flash fit require separate measurements.

The production dependency graph of Hyperinterval/Hyperlattice is unchanged. This
client has its own lockfile and only four dependency packages, all local: Alumina
FOC/protocol and Hyperinterval/Hyperlattice interval algebra. The library and Wasm
consumer are `no_std` and need no allocator; tests and native probes use `std`.

`clarke` keeps wide Q30 numerators until division/scaling, including differences
that exceed i32 but have representable outputs. Its alpha path uses an exact
integer affine image and subtractions, followed by `div_integer(3)`. Park and
its inverse retain the
current sequence of rounded products and sums/differences. For example, the
current Park implementation can reject an intermediate `MIN * -1` even when the
mathematical final result fits. A signed matrix rewrite must not silently change
that failure contract. Separately rounding the Clarke coefficients `2/3` and
`-1/3` changes exact output endpoints. Both cases are maintained regressions.

The corpus compares full outputs and typed failures, then checks successful
outputs against independent exact integer corner expectations. It includes signed
i32 extremes, zero-crossing intervals, valid uncertain rotations, balance
rejection, seeded intervals, widened products/divisions and repeated transforms.
The cost probe first verifies equal completed results, then reports timing,
allocation and checksums for both implementations. `native-probe` and
`shared-probe` run identical workload bodies for executable-size comparison.
These are finite fixtures and seeded checks, not exhaustive coverage of all i32
intervals. Native/shared agreement can include common underlying arithmetic;
the separate integer expectations supply the independent numerical checks.
The WASM host also reads each output tag and signed lane endpoint. Its BigInt
oracle derives the two Clarke radical brackets and applies outward rounding
at the specified Q30 operations through the complete five-transform chain.
Noncardinal Park checks include both sides of the signed range boundary.
The fixture ABI is not a general typed-error interface: unexpected assertions
spin, `range_probe` returns only success/Range, and `output_word` maps a native
error to a reversed-endpoint sentinel even for tag queries. See its
[source contract](wasm/lib.rs) before using it outside the successful fixture corpus.
Measurements and adoption decisions belong in the main progress/evidence ledger.
