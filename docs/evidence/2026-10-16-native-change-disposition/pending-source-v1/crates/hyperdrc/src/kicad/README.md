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

- Copper is stored as independent features with layer, net, kind, region, and
  location so checks can filter and report without returning to raw
  S-expressions.
- Via copper retains its exact decimal center and diameter as a native circle
  on every expanded copper layer. Invalid diameters retain a source-construction
  error so geometry checks can diagnose them. Report tessellation does not define
  the via boundary.
- Root `segment` tracks retain exact decimal endpoints and widths. Their native
  strokes include both round caps; coincident endpoints produce a disk and zero
  width is certified empty. Missing, malformed or negative-width source fields
  retain failed geometry for diagnostics. The midpoint remains exact until a
  report needs it. See the [track source references](../../../../docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/reference-sources.json).
- Primitive pad centers, sizes, angles, trapezoid deltas, and rounding/chamfer
  ratios are parsed as exact source decimals. Diameter halving, ratio products,
  oval decomposition and placement precede any report projection. A nonzero
  source dimension outside binary64 range remains native geometry. A present
  malformed numeric field retains a source failure instead of selecting an
  absent-field default.
- Pad positions are footprint-relative; their angles are absolute in the
  board frame, with zero when omitted. KiCad positive rotation maps `(x, y)`
  to `(y, -x)` at 90 degrees. Selected chamfers replace their corners while
  the remaining corners keep their rounding. The pinned source references
  and independent regression witnesses are in the
  [pad-source record](../../../../docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/reference-sources.json).
  These exact source formulas do not reproduce KiCad's integer-unit rounding.
- Wildcard copper layers are expanded from the board layer table when that
  evidence is present, because readiness checks need the fabricated copper
  picture rather than only the source token.
  `F&B.Cu` expands to front and back copper only, including on multilayer boards.
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
- [`pads.rs`](pads.rs) retains pad source fields and constructs native pad
  regions with board-frame placement. An oval keeps its body and two cap
  features, or one circle when its axes are equal.
- [`tracks.rs`](tracks.rs) parses exact linear-track source fields and delegates
  round-capped strokes to the native kernel.
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
- Straight tracks, vias, and zones.
- `Edge.Cuts` lines, arcs, rectangles, circles, polygons, and Bezier strokes.
- Panel graphics on common panelization-related layer names.
- Legacy `(width ...)` and newer `(stroke (width ...))` graphical width
  declarations where line or arc stroke width affects the simplified geometry.

The model intentionally stores copper as independent features with layer, net,
kind, region, and location. This keeps board-level checks simple: they can
filter by layer, compare nets, aggregate copper by layer, and report point
locations without returning to raw KiCad syntax.

## Current Limits

The parser is not a lossless KiCad implementation. It focuses on readiness data
used by the checks. Known limits include exact routed-slot geometry,
subtractive custom pad primitive semantics, glyph-accurate text rendering,
remaining non-copper KiCad graphical variants, full material stackup/rule
parsing beyond copper-layer wildcard expansion, and schematic/PCB parity.
Root `arc` tracks are currently omitted; the complete CPArti fixture contains
1,785 such objects. Custom-pad local primitives and graphics still use finite
source coordinates or sampled curves. Their individual finite line strokes now
import endpoints and width before native arithmetic, but that does not recover
source decimals or replace sampling. Custom-pad placement uses the exact parsed pad frame,
but footprint graphics still use their legacy placement adapter. Incomplete
objects in those paths, including malformed drill fields, can still be skipped
or defaulted. A failed pad placement retains failed copper geometry and does
not emit a drill with a fabricated center. Complete source diagnostics and
exact routed-slot geometry remain open.

Required copper/drill marker projections return typed uncertainty when no
finite location exists. Copper markers also reject retained shape/placement
failures; placeholder metadata cannot become an ordinary origin finding.
Net-class windows compare exact anchors, and net-class aggregation retains
source features until a finding needs markers. These boundaries do not certify
the remaining finite narrow predicates or legacy spatial-index heuristics.

For typed or lossless KiCad support, the roadmap in
[`../../docs/design-readiness-plan.md`](../../docs/design-readiness-plan.md)
tracks possible future integrations such as `kiutils-rs`, `kiutils_kicad`, and
`kicad-cli`.

Return to the [source tree README](../README.md).
