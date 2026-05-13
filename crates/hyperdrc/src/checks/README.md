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
- [`artifacts.rs`](artifacts.rs) contains BOM, centroid, netlist, README, and
  drawing sidecar checks for assembly/pre-production package readiness.
- [`excellon.rs`](excellon.rs) contains Excellon-sidecar readiness checks that
  validate tool tables, unit declarations, and drill hit integrity.
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
- `board-outline-cutout-clearance`
- `paste-overhang`
- `paste-aperture-coverage`
- `paste-aperture-ratio`
- `minimum-paste-aperture`
- `paste-aperture-spacing`
- `paste-mask-alignment`
- `exposed-copper`
- `solder-mask-opening-coverage`
- `solder-mask-expansion`
- `solder-mask-overlap-clearance`
- `solder-mask-board-edge-clearance`
- `silkscreen-overlap`
- `silkscreen-clearance`
- `silkscreen-board-edge-clearance`
- `silkscreen-min-width`
- `minimum-copper-neck-width`
- `solder-mask-sliver`
- `minimum-mask-opening`
- `solder-mask-opening-spacing`
- `acid-trap-candidate`
- `layer-sanity`
- `copper-balance-readiness`
- `mechanical-layer-geometry`
- `board-outline-sanity`
- `board-outline-fragments`
- `board-outline-self-intersection-readiness`
- `board-outline-notch-readiness`
- `board-outline-duplicate-readiness`
- `board-outline-nesting-readiness`

These checks mostly work by combining `csgrs` boolean operations with small
role-specific heuristics. Morphological checks use an erode-and-grow pattern to
detect thin copper, mask, and silkscreen features.

## Board Checks

[`board.rs`](board.rs) owns:

- `annular-ring-readiness`
- `annular-ring-tolerance`
- `plating-intent`
- `routed-slot-readiness`
- `castellation-intent`
- `castellation-hole-readiness`
- `via-in-pad-readiness`
- `drill-to-copper-clearance`
- `board-outline-drill-clearance`
- `drill-spacing`
- `drill-aspect-ratio`
- `drill-table-consistency`
- `copper-width-readiness`
- `copper-net-intent`
- `teardrop-readiness`
- `thermal-relief-readiness`
- `plane-clearance-readiness`
- `board-edge-exposure`
- `high-speed-edge-readiness`
- `edge-copper-pullback-readiness`
- `high-voltage-edge-readiness`
- `edge-stitching-readiness`
- `controlled-impedance-readiness`
- `differential-pair-readiness`
- `differential-pair-spacing-readiness`
- `differential-pair-via-symmetry-readiness`
- `reference-plane-readiness`
- `reference-plane-void-readiness`
- `orphaned-zone-readiness`
- `same-net-island-readiness`
- `return-path-readiness`
- `high-current-readiness`
- `power-via-array-readiness`
- `thermal-via-readiness`
- `power-plane-readiness`
- `high-current-neck-readiness`
- `voltage-clearance-readiness`
- `sensitive-net-spacing-readiness`
- `sensitive-return-readiness`
- `rf-keepout-readiness`
- `chassis-stitching-readiness`
- `gold-finger-readiness`
- `testpoint-coverage-readiness`
- `fiducial-readiness`
- `dense-pad-escape-readiness`
- `different-net-spacing`
- `layer-registration-tolerance`
- `panelization-clearance`
- `ipc356-coverage`
- `ipc356-drill-diameter`

Board checks use the parsed KiCad model and sidecars. They can reason about
same-net versus different-net copper, plated versus non-plated drills, nearby
IPC-D-356 test records, and panel or route geometry that is not visible from a
single Gerber layer alone.

## Artifact Checks

[`artifacts.rs`](artifacts.rs) owns `production-artifact-readiness`. It validates
common BOM, centroid, and netlist CSV/TSV content for required headers,
manufacturer/supplier procurement metadata, value/description and
footprint/package coverage, lifecycle/status review, approved alternate
coverage, quantity/refdes agreement, common grouped reference notation, DNP/DNI
parity handling, unusual reference designators, duplicate reference designators,
conflicting MPN value/footprint metadata, malformed centroid coordinates,
out-of-range rotations, invalid side values, duplicate centroid coordinates,
duplicate pin/net assignments, repeated netlist pin rows, one-pin net review,
reference parity between purchase, placement, and netlist artifacts, BOM versus
centroid assembly-side, value, footprint, and rotation parity, conflicting
centroid value/footprint/rotation metadata, polarity/MSL/component-height handoff
metadata for likely sensitive BOM rows, and DNP/DNI references that still appear
in placement data. It also checks README artifacts for basic revision/version,
manufacturing-note content, order parameters, contradictory order notes, rout
drawing parity for panelized jobs, release preflight evidence, assembly handoff
evidence for double-sided placement data, and conditional process notes for
selective/wave solder or conformal coating. It cross-checks README requests for
controlled impedance, edge plating, castellations, double-sided assembly, and
special assembly processes against the presence of fabrication or assembly
drawing sidecars. It validates text sidecar filenames/extensions for
recognizable BOM, centroid, netlist, and README roles, and checks
fabrication/assembly/rout drawing files for common extensions, empty or
placeholder-sized content, and role-specific filename tokens.

## Manifest Checks

[`manifest.rs`](manifest.rs) owns `file-manifest-readiness`. It classifies
Gerber-like input names into core manufacturing roles and warns when a package
is missing recognizable copper, outline/profile, drill data, or matching solder
mask layers. It also warns on duplicated core roles such as multiple top copper
files. In addition, `file-manifest-readiness` validates pre-production package
artifacts and now expects one of each: BOM, centroid, netlist, fabrication
drawing, assembly drawing, readme, and rout drawing. If KiCad input is provided
the check also compares the count of KiCad copper layers and an optional declared
manifest copper count against Gerber-recognized copper roles to catch probable
layer-stack mismatches before downstream checks. It reports inner copper without
both outer copper layers, odd recognized copper layer counts, side-specific
mask/paste/silkscreen files without matching copper, and single-copper packages
that also contain bottom-side outputs. It also compares recognizable revision
and generated-date tokens across Gerber and package artifact filenames, warns
when files appear to mix project/job name prefixes, and warns on stale-looking
backup/archive filename tokens.

## Excellon Checks

[`excellon.rs`](excellon.rs) owns `excellon-readiness`. It consumes parsed Excellon
reports and reports parser-level and data-integrity issues before geometry checks
consume drill hits.

- `excellon-readiness`

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
