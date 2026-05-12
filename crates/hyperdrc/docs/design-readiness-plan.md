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
- `paste-overhang`: subtract paired copper, optionally expanded by a tolerance,
  from paste apertures.
- `exposed-copper`: intersect paired copper and solder mask opening geometry.
- `silkscreen-overlap`: intersect silkscreen with explicitly paired blocker
  geometry such as copper, paste, or mask openings.
- `min-copper-neck`: approximate thin copper features by eroding and re-growing
  copper with half the requested minimum width, then reporting removed geometry.
- `solder-mask-sliver`: approximate thin solder mask webs by eroding and
  re-growing mask geometry with half the requested minimum mask web width.
- `acid-trap`: report copper polygon vertices whose angle is below a configured
  threshold.
- `layer-sanity`: report empty polygon geometry, missing bounds, and optional
  maximum-area excursions that can indicate polarity or layer-role problems.
- `annular-ring`: compare KiCad plated drill diameter against nearby same-net
  copper and report rings below the configured threshold.
- `drill-to-copper-clearance`: offset KiCad and Excellon drill holes by a
  configured clearance and intersect against other-net KiCad copper.
- `different-net-spacing`: offset same-layer KiCad copper features by a
  configured clearance and report different-net proximity.
- `layer-registration-tolerance`: compare KiCad copper layers with an offset
  tolerance to identify features vulnerable to registration shifts.
- `panelization-clearance`: check copper against KiCad panel graphics, KiCad
  NPTH drills, and Excellon drill hits.
- `ipc356-coverage`: warn when IPC-D-356 electrical test records do not have
  nearby parsed KiCad copper.

## Supported Inputs

- Gerber RS-274X through `csgrs` `gerber-io`.
- KiCad `.kicad_pcb` S-expression files for common board objects: nets, pads,
  oval pads, common custom pad primitives, tracks, vias, zones, `Edge.Cuts`
  lines/arcs/rectangles/circles, and panel graphics.
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
