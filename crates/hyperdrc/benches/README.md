# hyperdrc Benches

This folder contains benchmark and smoke-performance entry points for
`hyperdrc`.

## Contents

- [`parser_geometry_smoke.rs`](parser_geometry_smoke.rs) exercises parser,
  geometry construction, duplicate layer/island-geometry sanity, tiny and skinny
  layer-fragment review, local copper-density, sparse paste overhang/coverage/ratio,
  paste/mask alignment, exposed-copper, mask coverage/expansion/overlap-clearance,
  mask-island, silkscreen blocker, and paste/mask aperture spacing, thermal-pad windowpane, tombstone paste-imbalance with sparse pad and aperture culling, paste-via exposure with sparse aperture culling, trace-junction acid-trap including sparse segment culling, via-in-pad, and teardrop readiness paths plus dense-pad local-fiducial, escape, via-spacing including sparse pad-candidate culling, and mask-bridge review in a
  benchmark-style harness, including sparse mask-bridge pad-candidate culling.
  It also times RF keepout, antenna copper keepout,
  RF via-fence, switch-node, and inductor copper keepout heuristics used by RF
  and power-converter readiness, plus clustered
  thermal-via distribution, clustered thermal-via spread, sparse thermal-via candidate culling, bucketed thermal relief, pad-via, sparse copper-width/net-intent review, copper-area/spacing/keepout review, sparse different-net spacing, sparse registration-tolerance review, and sparse net-class clearance/differential-pair spacing review, ESD clamp return-path proximity,
  voltage/protective-earth spacing, ESD protection/return, surge-protection keepout review, mixed-signal partition and sensitive-net review, rectangular component-edge/fiducial-edge culling, bucketed component-hole/component/connector/fiducial/process/pad-pair spacing, bucketed testpoint accessibility spacing, testpoint side-parity culling, and testpoint copper-clearance culling. The harness also includes same-net drill-break,
  different-net short, differential pair width/neck-down/skew/via-proximity/return
  including dense pair-side via and sparse ground-stitch lookup cases,
  differential pair return, intra-pair spacing sparse acceptance,
  differential pair-to-pair spacing including sparse pair-field culling, split-plane crossing including sparse ground-zone culling, sparse reference-plane void and orphaned-zone review, return-path
  proximity including sparse same-layer ground lookup, connector return-path,
  rectangular board-edge exposure/high-speed/high-voltage edge fast paths, edge/chassis stitching, high-speed via return-path stitching, and decoupling
  proximity sparse ground-field lookups,
  proximity, same-net island connectivity, same-net drill-break sparse-drill culling, plane-clearance, panelization-clearance, drill-to-copper clearance, plating-intent sparse copper lookup, rectangular-outline drill-clearance fast-path coverage, drill-spacing, drill-table consistency, IPC-D-356 annotation/coverage/drill-diameter matching,
  mounting-hole spacing/distribution/grounding/keepout/edge/plating, gold-finger spacing/drill-keepout, rectangular edge-plating intent, castellation pitch, sparse tooling-hole filtering, sparse mouse-bite row spacing, testpoint coverage over large critical-net sets, and conformal-coating keepout probes, controlled-impedance, differential-pair presence, reference-plane presence, high-current layer-change, high-current pad-entry/via-array/via-return review including sparse support/return culling, and a many-island
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
