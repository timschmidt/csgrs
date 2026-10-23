# CSGRS size harness

These dependency-only consumers measure what root CSGRS workflows, the C ABI,
and the WASM bindings actually link. The package stays outside the workspace so
competitor kernels, Criterion, and proptest never enter a measured artifact,
and it declares its own `[workspace]` table so it resolves the same way from a
checkout, a git worktree, or an unpacked copy. `Cargo.lock` is seeded from the
repository's lockfile, so every shared dependency is measured at the version
the workspace resolves. After the root lockfile changes, reseed it rather than
resolving afresh:

```sh
cp ../../Cargo.lock . && cargo metadata --offline --format-version 1 >/dev/null
```

Every consumer completes a real workflow and consumes its result, so dead-code
elimination cannot reduce a row to metadata:

| Binary | Feature set | Workflow |
| --- | --- | --- |
| `primitive` | none | exact cube, translation, exact bounds, native triangle traversal |
| `composed` | `cad` | square profile, extrusion, regularized difference, exact bounds, certainty receipt, binary STL and OBJ materialization |
| `raster` | `raster` | grayscale buffer, `curve::from_image` contour tracing, exact bounds |
| `consumer/abi_consumer.c` | — | the same drilled-body workflow through the `csgrs-ffi` C ABI, including indexed buffers, construction evidence, and every `*_free` call |

Each binary takes a workload argument and a repeat count, and prints the
elapsed time of the timed loop after one warm iteration:

```sh
cargo run --locked --profile embedded --bin primitive -- 12 200
cargo run --locked --profile embedded --features cad --bin composed -- 12 200
cargo run --locked --profile embedded --features raster --bin raster -- 32 200
```

## Measuring

```sh
./measure.sh primitive
./measure.sh cad
./measure.sh raster
./measure.sh abi
./measure.sh wasm
./measure.sh wasm-documented
./measure.sh all
./measure.sh attribute
./measure.sh perf
```

`measure.sh` builds each artifact natively and for `wasm32-unknown-unknown`
under three matched configurations:

- `release` — the repository's speed-oriented profile (`opt-level = 3`).
- `embedded` with `opt-level = "z"` — the shipped size profile (fat LTO, one
  codegen unit, `panic = "abort"`).
- `embedded` with `opt-level = "s"` — the same profile with the alternate size
  level, selected through `CARGO_PROFILE_EMBEDDED_OPT_LEVEL` so no manifest
  changes between rows.

The harness profiles mirror the workspace root, so harness rows and root rows
are built with the same optimization, LTO, codegen-unit, and panic settings.
The `abi` scope builds `csgrs-ffi` from the workspace root and reports the
complete exported shared library and a real C consumer linked against the
static library, once as an ordinary link and once with `-Wl,--gc-sections`;
those are different quantities. The `wasm` scope builds the WASM bindings as
`scripts/test-wasm-bindings.sh` does (`--no-default-features --features wasm`),
and `wasm-documented` builds them as the README and `wasm-pack` command does
(`--features wasm` with the default features). When the installed
`wasm-bindgen` matches `Cargo.lock`, both also report the generated
`csgrs_bg.wasm`, JavaScript, and TypeScript bytes. `all` runs every size scope
but not `attribute` or `perf`.

Each row reports the artifact's file bytes, its SHA-256, `gzip -9` and
`brotli -q 11` bytes, and the allocated code and read-only data bytes
separately from symbol-table and debug bytes. Native rows also report the
deployed stripped size; WASM rows report the code, data, name, and debug
section bytes and `wasm-opt -Oz` output. Debug and symbol sections are
reported, never silently subtracted: a stripped size is a deployment number,
and the unstripped file is the diagnostic artifact.

`./measure.sh attribute` runs `cargo bloat --crates` (when installed) on the
embedded `z` consumers to attribute retained code to crates. Its attribution
is heuristic; for example, it can assign the standard library's vendored
decompressor to a dependency of the same name.

`./measure.sh perf` builds the native consumers at each embedded
optimization level, keeps a copy of each, and runs the copies in alternating
order, so a size comparison can be read beside the latency of the same
completed workload and host drift affects each level alike. Size ranks below performance in this
repository's preference ladder, so a smaller artifact that slows an ordinary
workload is not an improvement.

Environment variables:

- `CSGRS_SIZE_TARGET_DIR` — build directory root (default `./target`).
- `CSGRS_SIZE_OPT_LEVELS` — embedded optimization levels (default `z s`).
- `CSGRS_SIZE_PERF_REPEATS`, `CSGRS_SIZE_PERF_RUNS` — timed iterations per run
  and runs per configuration in the `perf` scope.
- `WASM_BINDGEN` — `wasm-bindgen` binary used for the bindings row.

Recorded results live in [`../../PERFORMANCE.md`](../../PERFORMANCE.md).
