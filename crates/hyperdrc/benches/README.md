# hyperdrc Benches

This folder contains benchmark and smoke-performance entry points for
`hyperdrc`.

## Contents

- [`parser_geometry_smoke.rs`](parser_geometry_smoke.rs) exercises parser,
  geometry construction, duplicate layer/island-geometry sanity, tiny and skinny
  layer-fragment review, local copper-density, and trace-junction acid-trap readiness paths plus dense-pad via-spacing and mask-bridge review in a
  benchmark-style harness. It also times RF keepout, antenna copper keepout,
  RF via-fence, switch-node, and inductor copper keepout heuristics used by RF
  and power-converter readiness, plus clustered
  thermal-via distribution and bucketed thermal copper-area/spacing/keepout review, ESD clamp return-path proximity,
  voltage/protective-earth spacing, ESD protection/return, surge-protection keepout review, mixed-signal partition and sensitive-net review, bucketed component-hole/component/connector/fiducial/process/pad-pair spacing, and bucketed testpoint accessibility spacing. The harness also includes same-net drill-break,
  different-net short, differential pair width/neck-down/skew/via-proximity/return,
  differential pair-to-pair spacing, split-plane crossing, return-path
  proximity, and drill-to-copper clearance probes, high-current pad-entry/via-return review, and a many-island
  minimum-copper-neck case so morphology changes do not regress on split copper
  pours, plus a small production-artifact package covering BOM/centroid/README parity,
  package-level handoff checks, waiver-governance metadata review, and waiver
  stub geometry-fingerprint generation. It is intended to catch broad
  regressions in the hot paths rather than to prove detailed rule behavior.
- [`fixture_smoke.rs`](fixture_smoke.rs) unpacks the repository board fixtures
  and times the same bounded KiCad/Gerber end-to-end smoke suites used by the
  fixture tests. It is intentionally small enough for routine local runs while
  still covering real fixture parsing, board loading, report emission, and
  minimum-copper-neck/drill-spacing execution.

## Usage

Run the benchmark target with Cargo:

```sh
cargo bench
```

Keep benchmark inputs small enough for routine local use. Detailed behavioral
coverage belongs in unit tests beside the owning module under
[`../src`](../src/README.md).

Return to the [repository README](../README.md).
