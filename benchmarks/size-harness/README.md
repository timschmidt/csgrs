# CSGRS size harness

These dependency-only consumers measure what root CSGRS workflows, the C ABI,
and the WASM bindings actually link. The package stays outside the workspace so
competitor kernels, Criterion, and proptest never enter a measured artifact,
and it declares its own `[workspace]` table so it resolves the same way from a
checkout, a git worktree, or an unpacked copy. `Cargo.lock` is seeded from the
repository's lockfile, so every shared dependency is measured at the version
the workspace resolves. After the root lockfile changes, reseed it rather than
resolving afresh, running this command from `benchmarks/size-harness`:

```sh
cp ../../Cargo.lock . && cargo metadata --offline --format-version 1 >/dev/null
```

Each consumer exercises a real workflow and consumes its result. Runtime arguments
and output uses make the workload observable; retained code must still be checked in
the artifact. A successful build is not evidence that the workflow ran or completed:

| Binary | Feature set | Workflow |
| --- | --- | --- |
| `primitive` | none | exact cube, translation, exact bounds, native triangle traversal |
| `composed` | `cad` | square profile, extrusion, regularized difference, exact bounds, extrusion-only local receipt, binary STL and OBJ materialization |
| `raster` | `raster` | grayscale buffer, `curve::from_image` contour tracing, exact bounds |
| `consumer/abi_consumer.c` | — | drilled-body construction through the f64 C ABI, projected bounds/indexed buffers, an evidence query and success-path frees |

Each Rust binary takes a workload argument and repeat count. Native builds print
elapsed loop time after one warm iteration, including workflow allocation and
destruction. WASM builds omit the timer. The C consumer has neither warmup nor timer;
it checks statuses and prints final counts/height. It does not inspect evidence
fields, and error exits do not explicitly free handles acquired before failure.
No consumer independently checks geometric equivalence across optimization levels.

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

For `primitive`, `cad` and `raster`, `measure.sh` builds native and
`wasm32-unknown-unknown` artifacts under these default configurations:

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
those are different quantities. ABI builds are native Linux artifacts; consumer
links/runs are omitted if `cc` is absent. The `wasm` scope builds the WASM bindings as
`scripts/test-wasm-bindings.sh` does (`--no-default-features --features wasm`),
and `wasm-documented` builds them as the README and `wasm-pack` command does
(`--features wasm` with the default features). When the installed
`wasm-bindgen` matches `Cargo.lock`, both also report the generated
`csgrs_bg.wasm`, JavaScript, and TypeScript bytes. `wasm` measures the `wasm`
profile and each configured embedded level; `wasm-documented` measures only the
`wasm` profile. `all` runs every size scope but not `attribute` or `perf`.
Rust consumer and binding size scopes build/report without running those artifacts;
the ABI scope executes its linked C consumers.

Each artifact report includes file bytes, SHA-256 and `gzip -9` bytes; Brotli
requires an installed `brotli`. Native section reporting requires `readelf`, with
stripped sizes additionally requiring `strip`. The ELF parser's `allocated_bytes`
sums non-NOBITS sections with nonzero addresses; it is not runtime memory usage.
Its read-only grouping includes selected relocation/unwind sections. WASM reports
section payload bytes, excluding module headers and section framing; custom payloads
include their names. `wasm-opt -Oz` output is reported only when the tool exists and
is not executed or checked for equivalence by this script. Original file bytes,
stripped bytes and optimized bytes remain separate quantities.

`./measure.sh attribute` runs `cargo bloat --crates` (when installed) on the
embedded `z` consumers to attribute retained code to crates. Its attribution
is heuristic; for example, it can assign the standard library's vendored
decompressor to a dependency of the same name.

`./measure.sh perf` builds the native consumers at each embedded
optimization level, keeps a copy of each, and runs the copies in alternating
order within each round, so a size comparison can be read beside workload latency.
This reduces order separation without eliminating host drift. Only `primitive` and
`composed` are timed, both with width 12; output/receipts are not compared between
levels. Size ranks below performance in this
repository's preference ladder, so a smaller artifact that slows an ordinary
workload is not an improvement.

Environment variables:

- `CSGRS_SIZE_TARGET_DIR` — build directory root (default the harness directory's
  `target`, regardless of the caller's working directory).
- `CSGRS_SIZE_OPT_LEVELS` — embedded optimization levels (default `z s`).
- `CSGRS_SIZE_PERF_REPEATS`, `CSGRS_SIZE_PERF_RUNS` — timed iterations per run
  and runs per configuration in the `perf` scope.
- `WASM_BINDGEN` — `wasm-bindgen` binary used for the bindings row.

Reports stream to stdout; retain them explicitly. Build outputs, optimized WASM,
generated bindings and copied perf binaries remain under the chosen target directory.
The script has no cleanup trap or final manifest proving every requested row completed;
an error can leave partial reports and temporary files. Optional-tool skips must be
considered when comparing reports.

Recorded results live in [`../../PERFORMANCE.md`](../../PERFORMANCE.md).
