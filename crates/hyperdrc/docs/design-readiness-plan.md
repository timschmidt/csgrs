# HyperDRC Design Readiness Plan

This project starts as a command line Gerber and KiCad checker built on `csgrs`
`Sketch` geometry. Gerber input covers layer-level polygon checks. KiCad input
adds net, drill, pad, via, track, zone, and board-outline context for richer
readiness checks. Excellon drill files are supported as an industry-standard
sidecar input where possible.

## Implemented Checks

- `mask-island-keepout`: treat each closed polygon or multipolygon island in a
  layer as the target island, offset that island and the remaining mask geometry
  by the requested keepout distance, intersect the two offset regions, and report
  the resulting coordinates as a violation when a non-empty shape remains.
- `copper-overlap`: intersect one copper layer with another copper layer and
  report overlapping geometry that may create unintended capacitance.
- `board-edge-clearance`: erode the board outline by the requested clearance,
  subtract that allowed region from copper, and report any remaining copper.
- `board-outline-sanity`: warn when an explicit board outline layer or KiCad
  `Edge.Cuts` data has no closed polygon area.
- `paste-overhang`: subtract paired copper, optionally expanded by a tolerance,
  from paste apertures.
- `paste-aperture-coverage`: subtract paste apertures from paired copper and
  report copper that does not have paste coverage.
- `exposed-copper`: intersect paired copper and solder mask opening geometry.
- `solder-mask-opening-coverage`: subtract solder mask openings from paired
  copper and report copper that would remain covered by mask.
- `silkscreen-overlap`: intersect silkscreen with explicitly paired blocker
  geometry such as copper, paste, or mask openings.
- `silkscreen-min-width`: approximate thin silkscreen strokes and text geometry
  by eroding and re-growing silkscreen with half the requested minimum width.
- `min-copper-neck`: approximate thin copper features by eroding and re-growing
  copper with half the requested minimum width, then reporting removed geometry.
- `solder-mask-sliver`: approximate thin solder mask webs by eroding and
  re-growing mask geometry with half the requested minimum mask web width.
- `acid-trap`: report copper polygon vertices whose angle is below a configured
  threshold.
- `layer-sanity`: report empty polygon geometry, missing bounds, and optional
  maximum-area excursions that can indicate polarity or layer-role problems.
- `mechanical-layer-geometry`: warn when polygon geometry appears on layers
  whose names look mechanical, fabrication, ECO, margin, or user-defined.
- `annular-ring`: compare KiCad plated drill diameter against nearby same-net
  copper and report rings below the configured threshold.
- `drill-to-copper-clearance`: offset KiCad and Excellon drill holes by a
  configured clearance and intersect against other-net KiCad copper.
- `drill-spacing`: compare KiCad and Excellon drill edge-to-edge clearance and
  report holes closer than the configured drill clearance.
- `different-net-spacing`: offset same-layer KiCad copper features by a
  configured clearance and report different-net proximity.
- `layer-registration-tolerance`: compare KiCad copper layers with an offset
  tolerance to identify features vulnerable to registration shifts.
- `panelization-clearance`: check copper against KiCad panel graphics, KiCad
  NPTH drills, and Excellon drill hits.
- `ipc356-coverage`: warn when IPC-D-356 electrical test records do not have
  nearby parsed KiCad copper.
- `ipc356-drill-diameter`: warn when IPC-D-356 drill diameter records conflict
  with nearby parsed KiCad drill diameters.

## Supported Inputs

- Gerber RS-274X through `csgrs` `gerber-io`.
- KiCad `.kicad_pcb` S-expression files for common board objects: nets, pads,
  oval pads, oval/rectangular drill declarations, common custom pad primitives,
  tracks, vias, zones, `Edge.Cuts` lines/arcs/rectangles/circles, and panel
  graphics.
- Excellon drill files with common `METRIC`/`INCH` tool definitions and drill
  hit coordinates.
- IPC-D-356 electrical test netlists with common test records. Parsed records can
  annotate nearby KiCad copper and drills by net name.

## Remaining Checks Requiring Deeper Inputs

- Solder mask sliver detection: identify thin residual solder mask webs between
  pads after mask expansion.
- Full KiCad custom pad primitive semantics, including subtractive primitives,
  exact roundrect radii, text primitives, and all graphic variants.
- Broader IPC-D-356 fixed-column dialect coverage and richer use of access-side,
  feature type, and soldermask flags.
- ODB++ or IPC-2581 ingestion for richer manufacturing stackup and fabrication
  rules.
- Fabricator-specific rule decks, class-based constraints, and board stackup
  tolerances.

## Test Ideas From Fabrication DRC Guidance

PCBWay's public DRC overview and linked engineering-question topics are useful
as a fabrication-facing checklist for tests. The strongest near-term value is to
turn those topics into small synthetic Gerber/KiCad fixtures with one clear
violation and one matching non-violation.

### Existing-check regression fixtures

- Implemented in unit tests: 3 mil minimum trace-width violation coverage, 6 mil
  preferred trace pass coverage,
  KiCad via annular-ring fail/pass coverage, pad-to-via, via-to-via,
  trace-to-trace, trace-to-pad, trace-to-via, hole-to-trace, hole-to-hole,
  conservative slot-to-slot, and slot-like drill-to-trace spacing coverage,
  0.20 mm board-edge trace
  clearance, pad-crossing-outline coverage, oversized solder-mask opening
  coverage, undersized/missing paste and solder-mask opening coverage,
  silkscreen-over-pad/blocker and silkscreen-over-V-score coverage, thin
  silkscreen stroke coverage,
  Excellon/NPTH-style panel drill clearance coverage, tab-route, V-score,
  castellated, and edge-plating panel-layer recognition coverage, same-net
  plated drill clearance suppression, same-net NPTH drill-to-trace clearance
  coverage, layer-sanity empty/bounds/area coverage,
  layer-registration tolerance coverage, IPC-D-356 missing-copper coverage, and
  IPC-D-356 drill net/diameter recovery and conflict coverage, KiCad
  oval/rectangular drill parsing coverage, no-input rejection coverage, missing
  Gerber/KiCad/Excellon/IPC-D-356 file coverage, and mechanical/user layer
  geometry, board-outline sanity, duplicate explicit layer-pair rejection, and
  duplicate explicit layer-role validation coverage, including
  all-layers-as-silkscreen rejection and board-outline versus explicit
  copper/mask/silkscreen role conflict rejection.
- Remaining spacing refinements: add exact routed slot-to-slot and plated-slot
  fixtures once slots preserve exact routed geometry and plated-slot semantics.
- Remaining drill/open-circuit fixtures: add through-hole containment fixtures
  where a drill cuts through a same-net trace so future open-circuit logic has
  expected behavior captured.
- Remaining paste and mask opening geometry: add per-pad paste exemptions once
  KiCad pad attributes are preserved through rule checks.
- Remaining silkscreen manufacturability: add fixtures for mirrored bottom
  silkscreen and slot geometry drawn on silkscreen once silkscreen side/text
  semantics are modeled.
- Remaining panelization features: add richer plated-half-hole and edge-plating
  copper treatment once side-plating semantics are modeled separately from
  generic panel graphics.
- Remaining layer sanity and file completeness: add CLI/app-level fixtures for
  missing layer-order metadata and ambiguous one-layer versus two-layer inputs.
- Remaining IPC-D-356 coverage: add access-side, feature-type, and soldermask
  flag semantics as broader fixed-column dialect support improves.

### Future-rule fixture backlog

- Power and ground trace sizing: fixtures with named power nets whose trace
  widths differ from a configured per-net or net-class minimum.
- Differential pair and critical signal constraints: fixtures for pair length
  mismatch, intra-pair spacing violations, missing/insufficient guard traces,
  and impedance-rule metadata once stackup or net classes exist.
- Region separation rules: fixtures that intentionally mix analog/digital,
  input/output, high-frequency/low-frequency, or high-power/low-power placement
  zones once board regions are modeled.
- Gold fingers and beveling: fixtures for no gold-finger design despite order
  metadata, missing mask opening on fingers, copper/text on fingers, and bevel
  keepout violations.
- BGA-specific checks: fixtures for BGA pad opening, solder-mask bridge, via
  escape, and pad/via spacing at dense pitch.
- NPTH and slot semantics: fixtures that distinguish cutouts, round holes,
  oval slots, rectangle slots, NPTH holes, plated slots, and route slots drawn on
  the wrong layer.

## Implemented Reporting

- Stable violation IDs derived from check name, layer names, island index, and
  rounded coordinates.
- Text, JSON, and GeoJSON output.
- SVG violation overlays for quick local or CI artifact review.
- Severity on each violation.
- JSON waiver files that suppress findings by ID, check name, layers, or message
  text.
- Compact JSON CI summaries with error, warning, waiver, and per-check counts.
- JSON rule configuration with CLI overrides for clearance thresholds, area
  thresholds, and KiCad copper layer selection.

## Reporting Roadmap

- Export violation overlays as Gerber for direct review in board viewers.
- Record parser warnings separately from DRC violations.

## Input Roadmap

- Infer layer roles from file extensions and X2 attributes.
- Accept explicit layer-role flags for ambiguous files. Initial flags are present
  for board outline, copper, paste/copper pairs, copper/mask pairs, and
  silkscreen/blocker pairs.
- Add stackup and net-class sections to the project config file.
- Add IPC-356 netlist ingestion so copper overlap can distinguish intended from
  unintended coupling.
- Add IPC-2581 and ODB++ importers if licensing and available crate support make
  that practical.
- Preserve Gerber units and report normalized units explicitly.
