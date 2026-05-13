# hyperdrc Benches

This folder contains benchmark and smoke-performance entry points for
`hyperdrc`.

## Contents

- [`parser_geometry_smoke.rs`](parser_geometry_smoke.rs) exercises parser and
  geometry paths in a benchmark-style harness. It is intended to catch broad
  regressions in the hot paths rather than to prove detailed rule behavior.

## Usage

Run the benchmark target with Cargo:

```sh
cargo bench
```

Keep benchmark inputs small enough for routine local use. Detailed behavioral
coverage belongs in unit tests beside the owning module under
[`../src`](../src/README.md).

Return to the [repository README](../README.md).
