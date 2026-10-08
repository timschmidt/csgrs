# hyperdrc Geometry

This folder contains the geometry helpers that make `hyperdrc` checks readable.
The rest of the crate should describe PCB concepts; this module handles the
repeatable polygon and profile mechanics underneath those concepts.

## Geometry Choices

`Polygon` retains an authoritative `CurveRegion2` and finite report rings.
`circle_polygon` imports its finite center/radius exactly and constructs a native
circle; its segment count controls only the report ring. KiCad vias construct
their native circles directly from exact decimal source values. Other parser
primitives still have documented approximation boundaries and need further
source-fidelity work.

Exact polygon area and bounds come from the retained region. Profile construction
combines those native bounds; report samples cannot establish a conservative
curve bound. Missing area certificates remain unavailable instead of borrowing
report-ring area. Some readiness consumers still need migration to propagate
that unavailability, and existing finite spatial indices require a separate
conservative-enclosure audit.

This split keeps geometry helpers small and predictable:

- Geometry functions should be deterministic for degenerate, tiny, signed, or
  rotated inputs.
- Source-specific interpretation belongs in parser modules, not here.
- Filtering is caller-controlled so checks can choose their own reportable-area
  thresholds.
- Metadata should survive profile conversion whenever a helper accepts a full
  profile instead of plain polygons.

## Module Map

- [`../geometry.rs`](../geometry.rs) is the public module facade. It exposes the
  helpers from this folder and keeps the external API compact.
- [`primitives.rs`](primitives.rs) builds common PCB shapes: circles,
  rectangles, trapezoids, rounded and chamfered rectangles, traces, arcs,
  Bezier strokes, transforms, and polygons from point lists.
- [`circle.rs`](circle.rs) constructs native circles under STRICT policy and
  retains construction or budget failures for source adapters to report.
- [`region.rs`](region.rs) combines exact-backed polygon inputs into native
  [Hypercurve](https://github.com/timschmidt/hypercurve) filled regions while
  preserving layer metadata.
- [`violations.rs`](violations.rs) projects exact-backed multipolygons into
  reportable violation shapes, including exact area filtering and hole
  preservation.

## Responsibilities

Geometry code is deliberately low-level. It should not know about KiCad nets,
Gerber file roles, waivers, reports, or CLI flags. It should provide predictable
operations that higher-level modules can compose into design-readiness checks.

The geometry tests are intentionally antagonistic. They cover degenerate line
segments, signed dimensions, tiny nonzero features, closed and open rings,
holes, rotations, trapezoid corner deltas, clamped rounded-rectangle radii,
selected chamfered-rectangle corners, clockwise and counterclockwise arcs,
Bezier sampling, zero-radius circles, and property-generated shapes. This is
important because PCB data frequently contains small fragments and
vendor-specific geometry edge cases.

## Working With Profiles

`hyperdrc` uses `PcbRegion`, an exact
[Hypercurve](https://github.com/timschmidt/hypercurve) filled-region carrier
with PCB metadata, as the common geometry container.
[CSGRS](https://github.com/timschmidt/csgrs) remains an explicit import and
adapter boundary. Parser modules build regions, check modules combine them with
offsets and booleans, and report modules convert finite boundary views into
stable violation records.

When adding geometry helpers:

- Keep functions deterministic and unit-test edge cases directly.
- Prefer structured geometry operations over string or coordinate hacks.
- Preserve metadata where the helper accepts or returns a full profile.
- Filter only when the caller supplies an explicit threshold.

Return to the [source tree README](../README.md).
