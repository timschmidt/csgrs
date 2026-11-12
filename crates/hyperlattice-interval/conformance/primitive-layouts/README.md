# Primitive interval layout experiment

This unpublished, standalone package compares Hyperlattice's Cartesian vector
with private separate lower/upper arrays. Both use Hyperinterval scalar kernels;
there is no SIMD or duplicate arithmetic. It covers i32/i64 fixed-point and
x86_64-qualified f32/f64 enclosures, empty sets, zero-dimensional unit products,
addition, subtraction, hull, intersection, dot products, and repeated chains.

Reconstruction validates through existing constructors. Canonical replay uses
the existing `IntervalVector` format. The prototype cannot provide the current
borrowed `lanes()` API. It is an experiment, not a production layout replacement.

From this directory:

```sh
cargo +1.88.0 test --locked
cargo +1.88.0 test --locked --release
cargo +1.88.0 clippy --locked --all-targets -- -D warnings
cargo +1.88.0 run --locked --release --bin cost
cargo +1.88.0 run --locked --release --bin native-probe -- 7
cargo +1.88.0 run --locked --release --bin soa-probe -- 7
cargo +1.88.0 build --locked --release --manifest-path wasm/Cargo.toml --target wasm32-unknown-unknown
```

The cost probe emits sizes and alternating samples for operations 0–7:
round trip/copy, add, subtract, hull, intersection, dot, imported chain, and
retained chain. Operations 0–6 include SoA ingress; vector outputs also include
canonical materialization. Operation 7 starts with each provider's native
storage and returns the same scalar endpoint enclosure. Each chain performs
four add/subtract pairs, intersection, hull, and dot. Exact outputs are compared
before timing, and every measured path asserts zero allocations.

The Wasm consumer exports full fixed-point result endpoints. IEEE ingress on
Wasm remains explicitly unqualified. The execution ledger and evidence archive
hold the independent Wasm oracle, raw measurements, adverse results, and the
adoption decision. Timing probes do not establish general performance bounds.
