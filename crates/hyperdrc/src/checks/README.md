# hyperdrc Checks

This folder contains the design-readiness checks that turn parsed geometry,
board context, and sidecar data into `hyperdrc` violations. Checks are grouped
by the data model they need.

## Module Map

- [`mod.rs`](mod.rs) exposes the check modules and documents the broad grouping.
- [`layer.rs`](layer.rs) contains checks over already-flattened 2D layer
  geometry, whether that geometry came from Gerber or from KiCad copper
  aggregation.
- [`board.rs`](board.rs) contains checks that need richer board context:
  KiCad nets, pads, vias, drills, panel graphics, Excellon drills, and
  IPC-D-356 points.
- [`manifest.rs`](manifest.rs) contains package-level readiness checks over the
  discovered input manifest and inferred Gerber roles.
- [`distance.rs`](distance.rs) contains geometric distance helpers shared by
  checks that need boundary-distance fallbacks in addition to polygon boolean
  operations.

## Layer Checks

[`layer.rs`](layer.rs) owns:

- `mask-island-keepout`
- `copper-overlap`
- `board-edge-clearance`
- `paste-overhang`
- `paste-aperture-coverage`
- `paste-aperture-ratio`
- `minimum-paste-aperture`
- `paste-mask-alignment`
- `exposed-copper`
- `solder-mask-opening-coverage`
- `solder-mask-overlap-clearance`
- `solder-mask-board-edge-clearance`
- `silkscreen-overlap`
- `silkscreen-board-edge-clearance`
- `silkscreen-min-width`
- `minimum-copper-neck-width`
- `solder-mask-sliver`
- `minimum-mask-opening`
- `acid-trap-candidate`
- `layer-sanity`
- `copper-balance-readiness`
- `mechanical-layer-geometry`
- `board-outline-sanity`
- `board-outline-fragments`

These checks mostly work by combining `csgrs` boolean operations with small
role-specific heuristics. Morphological checks use an erode-and-grow pattern to
detect thin copper, mask, and silkscreen features.

## Board Checks

[`board.rs`](board.rs) owns:

- `annular-ring-readiness`
- `plating-intent`
- `drill-to-copper-clearance`
- `board-outline-drill-clearance`
- `drill-spacing`
- `drill-aspect-ratio`
- `drill-table-consistency`
- `copper-net-intent`
- `different-net-spacing`
- `layer-registration-tolerance`
- `panelization-clearance`
- `ipc356-coverage`
- `ipc356-drill-diameter`

Board checks use the parsed KiCad model and sidecars. They can reason about
same-net versus different-net copper, plated versus non-plated drills, nearby
IPC-D-356 test records, and panel or route geometry that is not visible from a
single Gerber layer alone.

## Manifest Checks

[`manifest.rs`](manifest.rs) owns `file-manifest-readiness`. It classifies
Gerber-like input names into core manufacturing roles and warns when a package
is missing recognizable copper, outline/profile, drill data, or matching solder
mask layers. It also warns on duplicated core roles such as multiple top copper
files.

## Adding A Check

When adding a new `hyperdrc` check:

1. Put the implementation in the module matching its required data model.
2. Add focused passing and failing tests in the same module.
3. Add a variant to [`../cli.rs`](../cli.rs) and wire it into
   [`../app.rs`](../app.rs).
4. Add rule thresholds to [`../config.rs`](../config.rs) if the check needs
   tunable values.
5. Update the root [README](../../README.md) and the design readiness plan in
   [`../../docs`](../../docs/README.md).

Return to the [source tree README](../README.md).
