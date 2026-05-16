# hyperdrc KiCad Parser

This folder contains the KiCad-specific model and graphics helpers used by the
top-level [`../kicad.rs`](../kicad.rs) loader. The loader parses common
`.kicad_pcb` S-expression objects into a simplified board model for
design-readiness checks.

## Parser Choices

The KiCad loader is intentionally semantic and partial. It does not attempt to
round-trip a `.kicad_pcb` file. Instead, it extracts the objects that current
readiness checks can use: copper features, nets, pads, drills, vias, zones,
board outline geometry, and panel graphics.

Important consequences:

- Copper is stored as independent features with layer, net, kind, sketch, and
  location so checks can filter and report without returning to raw
  S-expressions.
- Wildcard copper layers are expanded from the board layer table when that
  evidence is present, because readiness checks need the fabricated copper
  picture rather than only the source token.
- Text and some graphics are approximated conservatively as geometry proxies.
  Exact rendering remains a known gap rather than a hidden promise.
- Unsupported KiCad semantics should be documented as limits and added with
  focused tests when a readiness check needs them.

## Module Map

- [`../kicad.rs`](../kicad.rs) is the parser facade. It reads a `.kicad_pcb`
  file, parses S-expressions, extracts nets, footprints, pads, drills, tracks,
  vias, zones, board outlines, and panel features, then returns a `BoardModel`.
- [`model.rs`](model.rs) defines the semantic data model used by checks:
  `BoardModel`, `CopperFeature`, `CopperKind`, and `DrillFeature`.
- [`arcs.rs`](arcs.rs) reconstructs circular arcs from KiCad start/mid/end
  points for board, footprint, and custom-pad graphics.
- [`custom_pad.rs`](custom_pad.rs) parses additive custom-pad graphics such as
  polygon, rectangle, circle, line, arc, Bezier, and conservative text bounding
  primitives.
- [`footprint_graphics.rs`](footprint_graphics.rs) parses copper-layer
  footprint graphics into unnetted copper features for checks that need the
  complete fabricated copper picture.
- [`graphic_primitives.rs`](graphic_primitives.rs) keeps shared filled versus
  stroked rectangle, circle, and polygon interpretation in one place.
- [`text.rs`](text.rs) provides conservative text bounding boxes shared by
  custom-pad and footprint-graphics parsing.
- [`graphics.rs`](graphics.rs) parses KiCad graphical objects for `Edge.Cuts`
  and panel-related layers such as V-score, tab-route, castellated, and
  edge-plating style names.

## Parsed Board Data

The current `hyperdrc` KiCad loader handles the common objects needed by the
implemented checks:

- Nets and net names.
- Board-level copper layer declarations for expanding `"*.Cu"` pad, via, and
  footprint-graphics wildcards across inner copper layers when a KiCad layer
  table is present.
- Footprint pads, including circular, rectangular, trapezoid,
  rounded-rectangle, chamfered rounded-rectangle, oval, and common custom pad
  primitives such as polygon, rectangle, circle, line, arc, Bezier strokes, and
  conservative text bounding boxes. Explicit unfilled rectangle, circle, and
  polygon primitives are preserved as stroked outlines instead of solid copper
  fills.
- Copper-layer footprint graphics, including line, rectangle, circle, arc,
  polygon, Bezier/curve aliases, and conservative text bounding primitives.
  Explicit unfilled rectangle, circle, and polygon graphics are preserved as
  stroked outlines.
- Pad drill declarations, including offset drill centers and conservative
  handling of oval and rectangular drill declarations.
- Tracks, vias, and zones.
- `Edge.Cuts` lines, arcs, rectangles, circles, polygons, and Bezier strokes.
- Panel graphics on common panelization-related layer names.
- Legacy `(width ...)` and newer `(stroke (width ...))` graphical width
  declarations where line or arc stroke width affects the simplified geometry.

The model intentionally stores copper as independent features with layer, net,
kind, sketch, and location. This keeps board-level checks simple: they can
filter by layer, compare nets, aggregate copper by layer, and report point
locations without returning to raw KiCad syntax.

## Current Limits

The parser is not a lossless KiCad implementation. It focuses on readiness data
used by the checks. Known limits include exact routed-slot geometry,
subtractive custom pad primitive semantics, glyph-accurate text rendering,
remaining non-copper KiCad graphical variants, full material stackup/rule
parsing beyond copper-layer wildcard expansion, and schematic/PCB parity.

For typed or lossless KiCad support, the roadmap in
[`../../docs/design-readiness-plan.md`](../../docs/design-readiness-plan.md)
tracks possible future integrations such as `kiutils-rs`, `kiutils_kicad`, and
`kicad-cli`.

Return to the [source tree README](../README.md).
