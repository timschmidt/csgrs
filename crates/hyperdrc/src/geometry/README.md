# hyperdrc Geometry

This folder contains the geometry helpers that make `hyperdrc` checks readable.
The rest of the crate should describe PCB concepts; this module handles the
repeatable polygon and sketch mechanics underneath those concepts.

## Module Map

- [`../geometry.rs`](../geometry.rs) is the public module facade. It exposes the
  helpers from this folder and keeps the external API compact.
- [`primitives.rs`](primitives.rs) builds common PCB shapes: circles,
  rectangles, traces, arcs, transforms, and polygons from point lists.
- [`sketch.rs`](sketch.rs) converts between `geo` polygons and `csgrs`
  `Sketch` values while preserving layer metadata.
- [`violations.rs`](violations.rs) converts `geo` multipolygons into reportable
  violation shapes, including area filtering and hole preservation.

## Responsibilities

Geometry code is deliberately low-level. It should not know about KiCad nets,
Gerber file roles, waivers, reports, or CLI flags. It should provide predictable
operations that higher-level modules can compose into design-readiness checks.

The geometry tests are intentionally antagonistic. They cover degenerate line
segments, signed dimensions, tiny nonzero features, closed and open rings,
holes, rotations, clockwise and counterclockwise arcs, zero-radius circles, and
property-generated shapes. This is important because PCB data frequently
contains small fragments and vendor-specific geometry edge cases.

## Working With Sketches

`hyperdrc` uses `PcbSketch`, an alias around `csgrs::Sketch<LayerMetadata>`, as
the common geometry container. Parser modules build sketches, check modules
combine them with offsets and booleans, and report modules convert resulting
polygons into stable violation records.

When adding geometry helpers:

- Keep functions deterministic and unit-test edge cases directly.
- Prefer structured geometry operations over string or coordinate hacks.
- Preserve metadata where the helper accepts or returns a full sketch.
- Filter only when the caller supplies an explicit threshold.

Return to the [source tree README](../README.md).
