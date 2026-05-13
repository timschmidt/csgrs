# hyperdrc Checks

This folder contains the design-readiness checks that turn parsed geometry,
board context, and sidecar data into `hyperdrc` violations. Checks are grouped
by the data model they need.

## Module Map

- [`mod.rs`](mod.rs) exposes the check modules and documents the broad grouping.
- [`layer.rs`](layer.rs) contains checks over already-flattened 2D layer
  geometry, whether that geometry came from Gerber or from KiCad copper
  aggregation.
- [`drill.rs`](drill.rs) contains hole, slot, castellation, drill-spacing,
  drill-to-copper, annular-ring, aspect-ratio, and drill-table checks across
  KiCad, Excellon, and IPC-D-356 sources.
- [`board.rs`](board.rs) contains checks that need richer board context:
  KiCad nets, pads, vias, panel graphics, Excellon drills, and
  IPC-D-356 points.
- [`assembly.rs`](assembly.rs) contains component, fiducial, testpoint, tooling,
  mouse-bite, and fine-pitch assembly-readiness checks.
- [`artifacts.rs`](artifacts.rs) contains BOM, centroid, netlist, README, and
  drawing sidecar checks for assembly/pre-production package readiness.
- [`surface_finish.rs`](surface_finish.rs) contains README/order-note surface
  finish compatibility heuristics used by `production-artifact-readiness`.
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
detect thin copper, mask, and silkscreen features. Paste checks also compare
paired paste/copper islands for basic aperture coverage and area ratio.

## Stencil Checks

[`stencil.rs`](stencil.rs) owns paste-printing checks that need stencil process
heuristics or KiCad via context:

- `thermal-pad-paste-windowpane-readiness`
- `stencil-area-ratio-readiness`
- `paste-aperture-aspect-ratio-readiness`
- `tombstone-paste-imbalance-readiness`
- `paste-via-exposure-readiness`

The module keeps IPC-7525-style paste printing heuristics out of generic layer
checks while still accepting flattened paste/copper geometry.

## Drill Checks

[`drill.rs`](drill.rs) owns fabrication checks where the primary geometry is a
hole, slot, castellation, or drill-table record:

- `annular-ring-readiness`
- `annular-ring-tolerance`
- `plating-intent`
- `routed-slot-readiness`
- `castellation-intent`
- `castellation-hole-readiness`
- `drill-to-copper-clearance`
- `board-outline-drill-clearance`
- `drill-spacing`
- `drill-aspect-ratio`
- `drill-table-consistency`
- `drills_to_sketch` shared geometry adapter for panel and drill keepout checks

These checks compare KiCad holes with sidecar Excellon and IPC-D-356 records,
review plated versus non-plated intent, catch edge/castellation ambiguity,
estimate annular-ring margin, and build conservative circular keepouts for slots
until exact routed-slot geometry is modeled.

## Board Checks

[`board.rs`](board.rs) owns board-context electrical, mechanical, and
pre-production checks that need nets, component-like copper, zones, outlines, or
panel features:

- `copper-width-readiness`
- `copper-net-intent`
- `via-in-pad-readiness`
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
- `gold-finger-edge-readiness`
- `gold-finger-spacing-readiness`
- `gold-finger-drill-keepout-readiness`
- `component-edge-clearance-readiness`
- `component-hole-clearance-readiness`
- `connector-rework-clearance-readiness`
- `pad-pair-asymmetry-readiness`
- `connector-return-path-readiness`
- `decoupling-proximity-readiness`
- `esd-protection-readiness`
- `switch-node-keepout-readiness`
- `thermal-pad-via-readiness`
- `thermal-copper-area-readiness`
- `hot-component-spacing-readiness`
- `thermal-mechanical-keepout-readiness`
- `different-net-spacing`
- `layer-registration-tolerance`
- `panelization-clearance`
- `ipc356-coverage`
- `ipc356-drill-diameter`

Board checks use the parsed KiCad model and sidecars. They can reason about
same-net versus different-net copper, nearby IPC-D-356 test records,
gold-finger edge, spacing, and drill keepout risk, connector return-path
signals, power decoupling proximity, ESD protection proximity, switching-node
keepout risk, thermal copper-area support, hot-component spacing,
thermal/mechanical hole keepouts, likely thermal-pad via coverage, and panel
geometry that is not visible from a single Gerber layer alone.

## Assembly Checks

[`assembly.rs`](assembly.rs) owns:

- `component-edge-clearance-readiness`
- `component-hole-clearance-readiness`
- `connector-rework-clearance-readiness`
- `pad-pair-asymmetry-readiness`
- `testpoint-coverage-readiness`
- `testpoint-accessibility-readiness`
- `tooling-hole-readiness`
- `mouse-bite-readiness`
- `fiducial-readiness`
- `local-fiducial-readiness`
- `dense-pad-escape-readiness`

These checks use KiCad pads, drills, board outlines, and IPC-D-356 points to
review assembly edge clearance, mechanical keepouts, two-terminal land-pattern
symmetry, fixture probe access, panel tooling, fiducials, and fine-pitch escape
signals.

## Artifact Checks

[`artifacts.rs`](artifacts.rs) owns `production-artifact-readiness`. It validates
common BOM, centroid, and netlist comma-, tab-, semicolon-, and
whitespace-delimited content for required headers,
manufacturer/supplier procurement metadata, value/description and
footprint/package coverage, lifecycle/status review, approved alternate
coverage, same-as-primary alternate detection, broader lifecycle-risk
vocabulary, optional unit-cost/price sanity, procurement consistency across
manufacturer/supplier/lifecycle fields, placeholder release metadata such as
`TBD` or `unknown`,
quantity/refdes agreement, zero-quantity population intent, assembly/build
variant handoff parity, common grouped reference notation, DNP/DNI parity
handling, unusual reference designators,
duplicate reference designators,
empty component/placement/netlist sidecars, conflicting MPN value/footprint/
procurement metadata, malformed centroid coordinates, unusually large placement
coordinates, placeholder centroid/netlist cells, out-of-range rotations,
invalid side values, duplicate centroid coordinates,
duplicate pin/net assignments, repeated netlist pin rows, one-pin net review,
reference parity between purchase, placement, and netlist artifacts, BOM versus
centroid assembly-side, value, footprint, and rotation parity, conflicting
centroid value/footprint/rotation metadata, polarity/MSL/component-height handoff
metadata for likely sensitive BOM rows, and DNP/DNI references that still appear
in placement data. It also checks README artifacts for basic revision/version,
manufacturing-note content, order parameters, contradictory fabrication,
layer-count, assembly, coating, programming, and test-fixture notes, rout
drawing parity for panelized jobs, release preflight evidence, assembly handoff
evidence for double-sided BOM or placement data, and conditional process notes for
selective/wave solder or conformal coating. It also infers likely through-hole,
BGA/CSP/LGA, and programmable BOM rows and expects README handoff notes for
solder process, X-ray/AOI/inspection, firmware/programming/test coverage,
firmware revision traceability, programming method, and test-acceptance
criteria. It cross-checks README requests for controlled impedance, edge
plating, castellations, fabrication markings, double-sided assembly, and special
assembly processes against the presence of fabrication or assembly drawing
sidecars. It also checks serialization/barcode handoff and packaging/ESD/
moisture notes when README release notes mention those workflows. It checks
surface-finish compatibility notes for edge contacts, fine-pitch packages,
press-fit hardware, and wire bonding. It checks
revision and generated/release date markers across sidecar filenames and README
content so mixed release packages are caught before handoff. It validates text
sidecar filenames/extensions for
recognizable BOM, centroid, netlist, and README roles, and checks
fabrication/assembly/rout drawing files for common extensions, empty or
placeholder-sized content, and role-specific filename tokens.

## Manifest Checks

[`manifest.rs`](manifest.rs) owns `file-manifest-readiness`. It classifies
Gerber-like input names into core manufacturing roles and warns when a package
is missing recognizable copper, outline/profile, drill data, or matching solder
mask layers. It also warns on duplicated core roles such as multiple top copper
files. In addition, `file-manifest-readiness` validates pre-production package
artifacts from explicit sidecar flags and from `--gerber-dir` sidecar discovery.
It expects one of each: BOM, centroid, netlist, fabrication drawing, assembly
drawing, readme, and rout drawing. If KiCad input is provided the check also
compares the count of KiCad copper layers and an optional declared manifest
copper count against Gerber-recognized copper roles to catch probable
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
5. Update this README, the root [README](../../README.md), the design readiness
   plan in [`../../docs`](../../docs/README.md), and the
   [test guide](../../docs/testing.md) so check ownership and coverage stay
   discoverable.

Return to the [source tree README](../README.md).
