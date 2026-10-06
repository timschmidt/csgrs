# Hypermesh size harness

These dependency-only binaries exercise overlapping exact Booleans without
Hypermesh's dev dependencies, so native and `wasm32-unknown-unknown` artifacts
measure the linked Hyper stack rather than Criterion, fuzzing, UI, or
competitor code.

Both binaries call the canonical shared-arrangement `boolean` API and
materialize its compact triangle batch. `hypermesh-size-harness` uses an exact
tetrahedron workload and consumes the value; `immediate` uses an exact box
workload and also observes aggregate certainty. The distinct consumers expose
link-retention differences without retaining a second Boolean engine or a
compatibility API. Each selects the operation from a runtime argument;
`immediate` also accepts `strict` as its second argument:

```sh
cargo run --release -- union
cargo run --release -- intersection
cargo run --release -- difference
cargo run --release -- symmetric-difference
cargo run --release --bin immediate -- union strict
```

`retention` repeatedly exercises the canonical arrangement with raw mesh views.
It is excluded from `measure.sh` because it is a long-lived-process memory
consumer rather than an artifact-size consumer:

```sh
cargo build --release --bin retention
heaptrack --record-only target/release/retention 512
```

`native_repeat` runs the same workload through owned native views, retaining
only the compact reusable proof facts on each input:

```sh
cargo run --release --bin native_repeat -- 512
```

`carrier_retention` holds 100,000 cold owned meshes to expose the fixed
per-mesh proof-header cost independently of position and triangle buffers:

```sh
cargo build --release --bin carrier_retention
heaptrack --record-only target/release/carrier_retention 100000
```

Measure the default or all-feature dependency graph:

```sh
./measure.sh default
./measure.sh all
```

Set `HYPERMESH_SIZE_TARGET_DIR` to keep build artifacts elsewhere. The script
reports both binaries' raw and compressed bytes, native sections,
`wasm-opt -Oz` output, and artifact hashes for the speed-oriented `release`
profile and size-oriented `size` profile.
