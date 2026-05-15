# hyperdrc Benches

This folder contains benchmark and smoke-performance entry points for
`hyperdrc`.

## Contents

- [`parser_geometry_smoke.rs`](parser_geometry_smoke.rs) exercises parser,
  geometry construction, local copper-density, and trace-junction acid-trap
  readiness paths plus dense-pad via-spacing and mask-bridge review in a
  benchmark-style harness. It also times the antenna and inductor copper
  keepout heuristics used by RF and power-converter readiness, plus clustered
  thermal-via distribution review, ESD clamp return-path proximity, and
  mixed-signal partition review. The harness also includes a many-island
  minimum-copper-neck case so morphology changes do not regress on split copper
  pours, plus a small production-artifact package covering BOM/centroid/README
  parity and package-level handoff checks. It is intended to catch broad
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
