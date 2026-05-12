<h1>
  hyperdrc
  <img src="./doc/hyperdrc.png" alt="Hyper, a clever math thief" width="144" align="right">
</h1>

`hyperdrc` is a Rust command line tool for design readiness checks on PCB
Gerber and KiCad board files. It uses the latest git version of
[`csgrs`](https://github.com/timschmidt/csgrs) for Gerber parsing, polygon
offsets, and boolean geometry.

## Usage

Run all checks against one or more Gerber layers:

```sh
cargo run -- path/to/top.gbr path/to/bottom.gbr
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

Available checks:

- `mask-island-keepout`: offsets each mask island and the remaining mask
  geometry by the keepout distance, intersects them, and reports resulting
  violation polygons.
- `copper-overlap`: intersects copper layers and reports overlapping regions.
- `board-edge-clearance`: subtracts the board outline eroded by `--clearance`
  from selected copper layers.
- `paste-overhang`: reports paste outside paired copper, allowing
  `--paste-tolerance`.
- `exposed-copper`: reports copper intersecting paired solder mask openings.
- `silkscreen-overlap`: reports silkscreen intersecting paired blocker geometry
  such as copper or mask openings.
- `min-copper-neck`: uses an erode-and-grow morphology check to flag copper
  features removed by `--min-width`.
- `acid-trap`: reports copper polygon vertices below `--acid-trap-angle`.
- `layer-sanity`: warns on empty layers, missing bounds, and optional
  `--max-layer-area` excursions.
- `solder-mask-sliver`: uses an erode-and-grow morphology check to flag mask
  webs removed by `--min-mask-width`.
- `annular-ring`: checks KiCad plated drills against nearby same-net copper.
- `drill-copper-clearance`: offsets KiCad and Excellon drills by
  `--drill-clearance` and intersects other-net copper.
- `net-spacing`: checks different-net KiCad copper features on the same layer
  using `--net-clearance`.
- `registration-tolerance`: reports cross-layer KiCad copper features within
  `--registration-tolerance`.
- `panelization-clearance`: checks copper against KiCad panel graphics, KiCad
  NPTH drills, and Excellon drill features using `--panel-clearance`.
- `ipc356-coverage`: warns when IPC-D-356 test records do not have nearby parsed
  KiCad copper within `--ipc356-tolerance`.

Layer roles are explicit zero-based indexes into the input file list:

```sh
cargo run -- \
  --board-outline 2 \
  --copper-layer 0 \
  --copper-layer 1 \
  --paste-pair 3:0 \
  --mask-pair 0:4 \
  --silk-pair 5:4 \
  top.gbr bottom.gbr edge.gbr paste.gbr mask.gbr silk.gbr
```

Output formats are `text`, `json`, and `geojson`. The JSON reports include
stable violation IDs, severity, layers, polygon coordinates, point locations
where applicable, and a short message. SVG review overlays can be written with
`--svg-overlay violations.svg`.

Rule thresholds can be placed in a JSON config file and loaded with `--config`.
CLI flags override config values. See
[examples/hyperdrc-config.json](examples/hyperdrc-config.json).

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

KiCad support currently reads the common `.kicad_pcb` S-expression objects used
for readiness checks: nets, pads, segments, vias, zones, `Edge.Cuts` lines,
rectangles, circles, arcs, oval pads, and common custom pad primitives. Simple
panel feature graphics are detected on layers containing `Panel` or `VScore`.
Excellon support reads common `METRIC`/`INCH` tool definitions and drill hits.
IPC-D-356 support reads common test records and uses them to annotate nearby
KiCad copper or drills with net names when source board objects are missing net
data.

See [docs/design-readiness-plan.md](docs/design-readiness-plan.md) for suggested
future checks and reporting improvements.
