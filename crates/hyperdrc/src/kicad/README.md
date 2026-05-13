# hyperdrc KiCad Parser

This folder contains the KiCad-specific model and graphics helpers used by the
top-level [`../kicad.rs`](../kicad.rs) loader. The loader parses common
`.kicad_pcb` S-expression objects into a simplified board model for
design-readiness checks.

## Module Map

- [`../kicad.rs`](../kicad.rs) is the parser facade. It reads a `.kicad_pcb`
  file, parses S-expressions, extracts nets, footprints, pads, drills, tracks,
  vias, zones, board outlines, and panel features, then returns a `BoardModel`.
- [`model.rs`](model.rs) defines the semantic data model used by checks:
  `BoardModel`, `CopperFeature`, `CopperKind`, and `DrillFeature`.
- [`graphics.rs`](graphics.rs) parses KiCad graphical objects for `Edge.Cuts`
  and panel-related layers such as V-score, tab-route, castellated, and
  edge-plating style names.

## Parsed Board Data

The current `hyperdrc` KiCad loader handles the common objects needed by the
implemented checks:

- Nets and net names.
- Footprint pads, including circular, rectangular, oval, and common custom pad
  primitives.
- Pad drill declarations, including conservative handling of oval and
  rectangular drill declarations.
- Tracks, vias, and zones.
- `Edge.Cuts` lines, arcs, rectangles, and circles.
- Panel graphics on common panelization-related layer names.

The model intentionally stores copper as independent features with layer, net,
kind, sketch, and location. This keeps board-level checks simple: they can
filter by layer, compare nets, aggregate copper by layer, and report point
locations without returning to raw KiCad syntax.

## Current Limits

The parser is not a lossless KiCad implementation. It focuses on readiness data
used by the checks. Known limits include exact routed-slot geometry, every
custom pad primitive semantic, complete text rendering, all KiCad graphical
variants, full stackup/rule parsing, and schematic/PCB parity.

For typed or lossless KiCad support, the roadmap in
[`../../docs/design-readiness-plan.md`](../../docs/design-readiness-plan.md)
tracks possible future integrations such as `kiutils-rs`, `kiutils_kicad`, and
`kicad-cli`.

Return to the [source tree README](../README.md).
