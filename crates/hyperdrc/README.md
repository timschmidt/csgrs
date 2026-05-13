<h1>
  hyperdrc
  <img src="./docs/hyperdrc.png" alt="hyperdrc logo" width="144" align="right">
</h1>

`hyperdrc` is a Rust command line tool for PCB design-readiness checks over
Gerber, KiCad, Excellon, and IPC-D-356 inputs. It uses the latest git version
of [`csgrs`](https://github.com/timschmidt/csgrs) for Gerber parsing, polygon
offsets, and boolean geometry.

## Current Status

`hyperdrc` is an active prototype with a broad regression suite for
fabrication-readiness rules. It supports layer-level Gerber checks, net-aware
KiCad checks, Excellon drill sidecars, IPC-D-356 electrical-test sidecars,
JSON/GeoJSON/text reports, SVG review overlays, JSON waivers, JSON rule
configuration, TransJLC conversion, and structured input provenance.

The implemented checks are useful for CI and local design review, but they are
not a replacement for a fabricator's final DFM/DRC pass. Some geometry is still
conservative: KiCad oval and rectangular drill declarations are treated as
circular keepouts using their larger dimension until exact routed-slot geometry
is modeled, and IPC-D-356 parsing focuses on common test records rather than the
full fixed-column dialect.

## Quick Start

Run all default checks against one or more Gerber layers:

```sh
cargo run -- path/to/top.gbr path/to/bottom.gbr
```

Load every Gerber-like file from a directory:

```sh
cargo run -- --gerber-dir path/to/gerber-package
```

Convert a Gerber package with
[`TransJLC`](https://github.com/HalfSweet/TransJLC) before loading the converted
outputs:

```sh
cargo run -- \
  --convert-input path/to/source-gerbers \
  --conversion-output-dir build/hyperdrc-converted \
  --source-eda kicad \
  --transjlc-bin TransJLC
```

Run KiCad-aware checks against a `.kicad_pcb` file:

```sh
cargo run -- \
  --config examples/hyperdrc-config.json \
  --kicad-pcb board.kicad_pcb \
  --kicad-copper-layer F.Cu \
  --kicad-copper-layer B.Cu \
  --excellon panel-holes.drl \
  --ipc356 board.ipc \
  --format geojson
```

Run a specific check sequence:

```sh
cargo run -- \
  --check mask-island-keepout \
  --check copper-overlap \
  --keepout 0.2 \
  --pair 0:1 \
  --format json \
  path/to/mask.gbr path/to/copper.gbr
```

Layer roles are explicit zero-based indexes into the Gerber input list:

```sh
cargo run -- \
  --board-outline 2 \
  --copper-layer 0 \
  --copper-layer 1 \
  --paste-pair 3:0 \
  --mask-pair 0:4 \
  --silk-layer 5 \
  --silk-pair 5:4 \
  top.gbr bottom.gbr edge.gbr paste.gbr mask.gbr silk.gbr
```

Output formats are `text`, `json`, and `geojson`. JSON reports include stable
violation IDs, severity, layers, polygon coordinates, point locations where
applicable, short messages, and a structured input manifest. SVG review overlays
can be written with `--svg-overlay violations.svg`.

Rule thresholds can be placed in a JSON config file and loaded with `--config`.
CLI flags override config values. See
[examples/hyperdrc-config.json](examples/hyperdrc-config.json).

## Default Readiness Checks

The default suite includes geometry, drill, sidecar, package, and provenance
checks:

- `mask-island-keepout`, `copper-overlap`, `board-edge-clearance`,
  `board-outline-sanity`, `board-outline-fragments`, `paste-overhang`,
  `paste-aperture-coverage`, `paste-aperture-ratio`,
  `minimum-paste-aperture`, `paste-aperture-spacing`,
  `paste-mask-alignment`, `exposed-copper`, `solder-mask-opening-coverage`,
  `solder-mask-expansion`, `solder-mask-overlap-clearance`,
  `solder-mask-board-edge-clearance`, `silkscreen-overlap`, `silkscreen-clearance`,
  `silkscreen-board-edge-clearance`, `silkscreen-min-width`,
  `min-copper-neck`, `acid-trap`, `layer-sanity`, `copper-balance`,
  `mechanical-layer-geometry`, `solder-mask-sliver`, and
  `minimum-mask-opening`, and `solder-mask-opening-spacing`.
- `annular-ring`, `annular-ring-tolerance`, `plating-intent`, `routed-slot-readiness`,
  `castellation-intent`, `castellation-hole-readiness`,
  `drill-copper-clearance`, `via-in-pad-readiness`,
  `board-outline-drill-clearance`, `drill-spacing`, `drill-aspect-ratio`,
  `drill-table-consistency`, `copper-width-readiness`, `copper-net-intent`,
  `teardrop-readiness`, `thermal-relief-readiness`, `plane-clearance-readiness`,
  `board-edge-exposure`, `high-speed-edge-readiness`,
  `high-voltage-edge-readiness`,
  `controlled-impedance-readiness`,
  `differential-pair-readiness`, `differential-pair-spacing-readiness`,
  `reference-plane-readiness`,
  `reference-plane-void-readiness`, `orphaned-zone-readiness`,
  `same-net-island-readiness`, `return-path-readiness`,
  `high-current-readiness`, `power-via-array-readiness`, `thermal-via-readiness`,
  `power-plane-readiness`, `high-current-neck-readiness`, `voltage-clearance-readiness`,
  `sensitive-net-spacing-readiness`, `gold-finger-readiness`, `net-spacing`,
  `registration-tolerance`, and
  `panelization-clearance`.
- `ipc356-coverage`, `ipc356-drill-diameter`, and
  `file-manifest-readiness`.

Important tunables include `--keepout`, `--clearance`, `--min-width`,
`--min-mask-width`, `--acid-trap-angle`, `--annular-ring`,
`--drill-clearance`, `--board-thickness`, `--max-drill-aspect-ratio`,
`--min-paste-area-ratio`, `--max-paste-area-ratio`,
`--max-copper-imbalance-ratio`, `--net-clearance`,
`--registration-tolerance`, `--panel-clearance`, `--ipc356-tolerance`,
`--min-area`, and `--max-layer-area`.

## Waivers And CI

Waiver files are JSON and can suppress findings by `id`, `check`, `layers`, and
message text. A compact CI summary can be written with `--summary-file`:

```json
{
  "waivers": [
    {
      "check": "acid-trap-candidate",
      "layers": ["F.Cu"],
      "message_contains": "below 30",
      "reason": "accepted connector footprint geometry"
    }
  ]
}
```

```sh
cargo run -- \
  --kicad-pcb board.kicad_pcb \
  --waiver waivers.json \
  --summary-file summary.json \
  --svg-overlay violations.svg
```

## Repository Map

Each folder has its own local README with the hyperdrc-specific ownership
details for that part of the tree:

- [src](src/README.md): Rust crate structure, runtime pipeline, parsers,
  reports, configuration, and submodule map.
- [src/checks](src/checks/README.md): all design-readiness checks grouped by
  layer, board, manifest, and geometry-helper responsibilities.
- [src/geometry](src/geometry/README.md): polygon construction, sketch
  conversion, shape extraction, and geometry-test expectations.
- [src/kicad](src/kicad/README.md): KiCad board model, S-expression parsing,
  graphics parsing, and current parser scope.
- [docs](docs/README.md): roadmap, design-readiness backlog, and visual assets.
- [examples](examples/README.md): runnable configuration examples.
- [benches](benches/README.md): benchmark and smoke-performance entry points.
- [proptest-regressions](proptest-regressions/README.md): persisted fuzz and
  property-test regression seeds.

## Known Gaps

Not yet modeled: exact routed slot shapes, plated-slot/edge-plating electrical
semantics, KiCad silkscreen text side/mirroring, per-pad paste or mask
attributes, fabricator-specific rule decks, stackup/net-class constraints, and
ODB++/IPC-2581 input.

See [docs/design-readiness-plan.md](docs/design-readiness-plan.md) for the
long-form design-readiness roadmap.
