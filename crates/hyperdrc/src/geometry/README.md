# hyperdrc Geometry

This folder contains the geometry helpers that make `hyperdrc` checks readable.
The rest of the crate should describe PCB concepts; this module handles the
repeatable polygon and profile mechanics underneath those concepts.

## Geometry Choices

`Polygon` retains an authoritative `CurveRegion2` and finite report rings.
`circle_polygon` imports its finite center/radius exactly and constructs a native
circle; its segment count controls only the report ring. KiCad vias construct
their native circles directly from exact decimal source values. `rect_polygon`
imports its finite center, dimensions and degree angle before arithmetic and
uses native rotation and translation. Zero dimensions are certified empty;
invalid inputs and exhausted construction remain failures. Finite report corners
are separate, even when they collapse or exceed their representable range.
Other parser primitives and token-to-float paths still need source-fidelity work.

Rounded/chamfered rectangles and trapezoids also build their native outlines
before computing report corners. Rounded rectangles retain exact line/arc
boundaries regardless of report sampling. Native clamping preserves subnormal
dimensions and limiting capsules, circles and chamfer diamonds. Trapezoid
deltas follow KiCad's half-delta convention; excessive deltas remain failures.

`Polygon::try_area` and `try_bounds` measure the retained region under STRICT
policy and preserve source, measurement and shared-budget failures. Only
certified empty geometry has no bounds. Completed values may be cached, but
warm caches still respect source and budget failures. Required area, bounds and
dimension consumers propagate uncertainty to readiness records.

`transform_polygon` transforms the retained native carrier under the same
STRICT and shared-budget guard. Source or transform failures remain attached
to its result, including identity and empty fast paths. Retry from the original
polygon in a fresh scope. Report rings are transformed separately and cannot
replace unavailable native geometry or measurements.

Profile construction combines native bounds; report samples cannot establish a
conservative curve bound. The layer-polygon grid uses certified native
enclosures and retains unknown or oversized boxes for exact predicates.
`bounding_rect` remains a diagnostic report view with a finite-ring fallback.
Stencil native point candidates share that grid and retain unrepresentable
centers or radii for checked native distance predicates. Other point/feature
indices, report-chord width shortcuts and finite source adapters
still need a conservative-enclosure and source-fidelity audit.

Drill spacing also uses this enclosure grid, with native circular bounds and a
checked signed edge gap. Native point queries preserve IPC/drill source indices
even when report coordinates are unavailable. Hardware selection and spacing
retain unresolved comparisons, and castellation selection uses the authoritative
outline boundary. Other drill checks and point/copper indices still need audit.

IPC net annotation stages native center matches and diameter updates, then
publishes only after the shared guard succeeds. Failures leave metadata intact
and retain their typed cause. Native IPC coverage distinguishes an unresolved
candidate from certified absence and accepts any certified matching center.

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
- [`primitives.rs`](primitives.rs) builds circles, finite trace/stroke adapters,
  transforms and polygons from point lists.
- [`pad.rs`](pad.rs) constructs native rounded/chamfered rectangles and
  trapezoids under the shared guard. [`pad/report.rs`](pad/report.rs) holds their
  separate finite drawing calculations.
- [`rectangle.rs`](rectangle.rs) constructs native rectangles before generating
  their separate finite report corners, under the STRICT/shared-budget guard.
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
