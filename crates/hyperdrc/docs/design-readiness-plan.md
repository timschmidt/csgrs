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
- `board-outline-fragments`: warn when an explicit board outline layer or KiCad
  `Edge.Cuts` data parses to multiple disconnected outline regions.
- `paste-overhang`: subtract paired copper, optionally expanded by a tolerance,
  from paste apertures.
- `paste-aperture-coverage`: subtract paste apertures from paired copper and
  report copper that does not have paste coverage.
- `paste-aperture-ratio`: compare paste area over each paired copper island
  against configured minimum and maximum paste-to-copper ratios.
- `minimum-paste-aperture`: warn when explicit paste layers contain apertures
  whose minimum bounding dimension is below the configured minimum width.
- `paste-mask-alignment`: join explicit paste/copper and copper/mask pairs by
  their shared copper layer and warn when paste extends outside the matching
  solder mask opening.
- `exposed-copper`: intersect paired copper and solder mask opening geometry.
- `solder-mask-opening-coverage`: subtract solder mask openings from paired
  copper and report copper that would remain covered by mask.
- `solder-mask-overlap-clearance`: expand paired solder mask openings by the
  configured clearance, subtract the intentional opening, and warn on covered
  copper in that vulnerable band.
- `solder-mask-board-edge-clearance`: warn when explicit mask-opening layers
  fall outside the board outline eroded by the configured clearance.
- `silkscreen-overlap`: intersect silkscreen with explicitly paired blocker
  geometry such as copper, paste, or mask openings.
- `silkscreen-board-edge-clearance`: warn when explicit silkscreen layers fall
  outside the board outline eroded by the configured clearance.
- `silkscreen-min-width`: approximate thin silkscreen strokes and text geometry
  by eroding and re-growing silkscreen with half the requested minimum width.
- `min-copper-neck`: approximate thin copper features by eroding and re-growing
  copper with half the requested minimum width, then reporting removed geometry.
- `solder-mask-sliver`: approximate thin solder mask webs by eroding and
  re-growing mask geometry with half the requested minimum mask web width.
- `minimum-mask-opening`: warn when explicit mask-opening layers contain
  openings whose minimum bounding dimension is below the configured mask width.
- `acid-trap`: report copper polygon vertices whose angle is below a configured
  threshold.
- `layer-sanity`: report empty polygon geometry, missing bounds, and optional
  maximum-area excursions that can indicate polarity or layer-role problems.
- `copper-balance`: compare selected Gerber copper layers or parsed KiCad
  copper layers and warn when largest-to-smallest copper area exceeds a
  configured ratio.
- `mechanical-layer-geometry`: warn when polygon geometry appears on layers
  whose names look mechanical, fabrication, ECO, margin, or user-defined.
- `annular-ring`: compare KiCad plated drill diameter against nearby same-net
  copper and report rings below the configured threshold.
- `plating-intent`: warn when a KiCad NPTH drill has nearby copper or when a
  plated drill has no nearby same-net pad or via copper.
- `drill-to-copper-clearance`: offset KiCad and Excellon drill holes by a
  configured clearance and intersect against other-net KiCad copper.
- `board-outline-drill-clearance`: offset KiCad and Excellon drill holes by
  their radius plus the configured drill clearance and report keepout area that
  is not contained by the KiCad board outline.
- `drill-spacing`: compare KiCad and Excellon drill edge-to-edge clearance and
  report holes closer than the configured drill clearance.
- `drill-aspect-ratio`: compare finished board thickness against KiCad and
  Excellon drill diameters and warn when the configured aspect-ratio limit is
  exceeded.
- `drill-table-consistency`: compare nearby KiCad, Excellon, and IPC-D-356
  drill records and warn when sidecar drill diameters conflict.
- `copper-net-intent`: warn when parsed KiCad copper still has no net after
  native board parsing and IPC-D-356 annotation.
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
- `file-manifest-readiness`: warn when a Gerber package is missing recognizable
  copper, board outline, drill data, or matching solder mask layers, and warn on
  duplicate core manufacturing roles.

## Supported Inputs

- Gerber RS-274X through `csgrs` `gerber-io`.
- Gerber directories and converter-produced Gerber directories. The first
  converter backend shells out to TransJLC to normalize common EDA Gerber
  packages into JLCEDA/JLCPCB-style Gerber names before loading them through the
  same layer pipeline.
- KiCad `.kicad_pcb` S-expression files for common board objects: nets, pads,
  oval pads, oval/rectangular drill declarations, common custom pad primitives,
  tracks, vias, zones, `Edge.Cuts` lines/arcs/rectangles/circles, and panel
  graphics.
- Excellon drill files with common `METRIC`/`INCH` tool definitions and drill
  hit coordinates.
- IPC-D-356 electrical test netlists with common test records. Parsed records can
  annotate nearby KiCad copper and drills by net name.

## Remaining Checks Requiring Deeper Inputs

- Full KiCad custom pad primitive semantics, including subtractive primitives,
  exact roundrect radii, text primitives, and all graphic variants.
- Broader IPC-D-356 fixed-column dialect coverage and richer use of access-side,
  feature type, and soldermask flags.
- ODB++ or IPC-2581 ingestion for richer manufacturing stackup and fabrication
  rules.
- Fabricator-specific rule decks, class-based constraints, and board stackup
  tolerances.

## Research-Derived Readiness Backlog

The backlog below combines PCB fabricator DRC/DFM guidance, KiCad preflight
concepts, and assembly-oriented DFA review items. Each item should eventually
have one failing fixture, one passing fixture, and a rule-deck configuration
example before it is considered production-ready.

### Data Model and Rule Deck Foundations

- Unit model: preserve source units, normalize internal distances, and report
  both source and normalized units. Avoid comparing mil/mm thresholds against
  untagged parser output.
- Board stackup model: layer count, copper weight, dielectric thickness,
  material family, finish, soldermask process, soldermask color, board
  thickness, controlled-impedance requirements, and target IPC/fabricator class.
- Net-class model: per-net and per-class trace width, clearance, differential
  pair, length, impedance, via, current, and voltage constraints.
- Manufacturing capability profiles: JLCPCB, PCBWay, Eurocircuits-style
  service classes, and custom JSON decks. Profiles should separate hard minimum,
  preferred minimum, and cost-escalation thresholds.
- Assembly profile: hand assembly, prototype SMT, production SMT, double-sided
  SMT, reflow constraints, selective solder, wave solder, press-fit, and
  conformal coating assumptions.
- Mechanical model: routed outline, V-score lines, mouse bites, tabs, slots,
  cutouts, countersinks, castellations, plated edges, bevels, tooling holes,
  fiducials, keepout zones, stiffeners, and enclosure constraints.
- BOM/position model: component outlines, package classes, rotations, heights,
  polarity marks, fiducials, centroid files, assembly-side data, and alternate
  parts.
- File manifest model: expected copper/mask/paste/silk/drill/fab/drawing files,
  generated timestamp, source EDA, board revision, order parameters, and
  declared layer count. Initial role inference and missing/duplicate core-file
  checks are implemented by `file-manifest-readiness`.

### Fabrication Geometry Checks

- Minimum copper feature width by layer, copper weight, and service class.
- Copper spacing matrix: trace-to-trace, trace-to-pad, trace-to-via,
  pad-to-pad, pad-to-via, via-to-via, copper-to-hole, copper-to-slot,
  copper-to-cutout, and copper-to-board-edge.
- Voltage-aware clearance: compare net voltage classes against internal,
  external-coated, external-uncoated, slot, and creepage/clearance rules.
- Annular ring with tolerance: compute nominal, worst-case drill-wander, and
  registration-adjusted ring for vias, component holes, press-fit holes, and
  plated slots.
- Drill aspect ratio: board thickness divided by finished drill diameter, with
  separate thresholds for standard through holes, microvias, blind vias, buried
  vias, and backdrills. Initial through-hole style checks are implemented for
  KiCad and Excellon drill features with configurable board thickness and
  maximum aspect ratio.
- Drill table consistency: compare Excellon tools, KiCad drill declarations,
  IPC-D-356 drill data, fabrication drawing notes, and order metadata. Initial
  sidecar consistency checks compare nearby KiCad/Excellon and Excellon/IPC-D-
  356 drill records.
- Plated versus non-plated intent: detect copper pads around NPTH holes,
  missing pad stacks around PTH holes, ambiguous slot plating, and NPTH copper
  clearance violations. Initial KiCad checks warn on NPTH drills with nearby
  copper and plated drills without nearby same-net pad or via copper.
- Routed slot geometry: exact obround/rectangular slot outlines, slot width,
  slot end radius, slot-to-copper spacing, slot-to-slot spacing, and minimum
  routed cutter diameter.
- Board outline validity: closed contour, self-intersections, duplicate
  outlines, nested cutouts, tiny outline fragments, notches below router
  diameter, inside corners below router radius, and outline-to-hole tolerance.
  Initial outline-fragment checks warn when the parsed outline has multiple
  disconnected regions. Initial outline-to-hole checks report KiCad and
  Excellon drill keepouts that are not contained by the parsed board outline.
- Copper balance: layer copper area, local copper density, plane void islands,
  sparse inner layers, high copper imbalance across the stack, and bow/twist
  risk flags. Initial layer-area imbalance checks are implemented for KiCad
  copper layers and explicitly selected Gerber copper layers.
- Isolated copper: floating copper islands, unconnected pours, orphaned zone
  remnants, and copper slivers below etchable area. Initial KiCad net-intent
  checks warn on parsed copper that remains unnetted after optional IPC-D-356
  annotation.
- Acid traps: acute polygon vertices, acute trace junctions, narrow wedge voids,
  and trapped etchant pockets inside plane pours.
- Teardrop recommendations: narrow trace-to-pad and trace-to-via junctions below
  a configured width or annular-ring margin.
- Thermal relief: starved thermals, missing thermal spokes where required,
  excessive spoke width for solderability, asymmetric reliefs, and direct plane
  connections on hand-soldered through-hole pads.
- Plane clearances: antipad size on inner layers, missing clearance pads for
  through-hole pins, copper pour clearance around mechanical holes, and shorts
  from insufficient antipads.
- Board edge exposure: exposed copper at routed edges, unintentional edge
  plating, copper too close to V-score, copper too close to tab routes, and
  missing edge-plating declarations when copper intentionally reaches an edge.
- Gold fingers: finger width/spacing, bevel keepout, mask opening, plating
  finish requirement, no vias/text/silk/paste on fingers, and consistent finger
  length.
- Castellations and half holes: minimum hole diameter, annular ring, edge
  registration, plating intent, pad pitch, and copper pullback around routed
  board edge.
- High-current copper: trace width, neck-down length, via array current sharing,
  copper pour bottlenecks, thermal vias, and connector/pad current-density
  warnings.
- Controlled impedance readiness: impedance-rule presence, stackup completeness,
  reference-plane continuity, coplanar ground spacing, return-path voids, and
  layer-change via stitching.

### Solder Mask, Paste, and Finish Checks

- Solder mask expansion: mask opening larger than copper pad by the selected
  process margin; flag mask-on-pad and excessive exposure.
- Solder mask web/sliver: residual mask dam width between pads, vias, and
  openings; classify by mask process and color where known.
- Mask overlap clearance: copper track or plane too close to mask opening to
  remain reliably covered. Initial paired copper/mask checks warn when covered
  copper falls within the configured mask-opening clearance band.
- Minimum mask opening: openings too small to resolve in the selected mask
  process. Initial explicit mask-layer checks flag openings whose minimum
  bounding dimension is below the configured mask width.
- Mask-to-board-edge behavior: required pullback near routed edges, V-scores,
  panel tabs, and specified exposed-edge areas. Initial explicit mask-layer
  checks warn when openings violate the configured board-edge clearance.
- Via tenting/filling: vias under components, exposed vias in pads, tented-via
  intent versus mask openings, plugged/capped/fill requirements, and mismatch
  between design and order notes.
- BGA mask rules: NSMD/SMD consistency, mask bridge width, opening ratio, escape
  via proximity, dogbone geometry, and via-in-pad treatment.
- Paste reduction/expansion: paste aperture area ratio versus copper, per-pad
  paste overrides, excessive paste on thermal pads, and missing paste on SMD
  pads. Initial paired-layer island ratio checks are implemented with
  configurable minimum and maximum paste-to-copper area ratios.
- Stencil manufacturability: aperture minimum width, aperture area ratio,
  aspect ratio, home-plate apertures, windowpane thermal pads, fine-pitch bridge
  risk, and tombstoning imbalance on two-terminal parts. Initial explicit paste
  layer checks flag apertures whose minimum bounding dimension is below the
  configured minimum width.
- Paste-to-mask/copper alignment: paste outside mask opening, paste outside pad,
  paste over vias, paste bridging between adjacent pads, and paste aperture
  slivers below stencil process capability. Initial explicit triple checks warn
  when paste extends outside the solder mask opening associated with the same
  copper layer.
- Surface finish compatibility: ENIG/ENEPIG/hard gold/HASL constraints for
  fine-pitch, press-fit, gold fingers, wire bonding, and high-voltage creepage.

### Silkscreen, Legend, and Marking Checks

- Legend line width and text height by fabrication capability.
- Silkscreen overlap with exposed copper, mask openings, paste, vias, holes,
  slots, board edges, V-score, tab routes, and gold fingers. Initial explicit
  silkscreen plus board-outline clearance checks are implemented.
- Silkscreen clipping risk: legend too close to mask cutbacks or pads where the
  fabricator will clip text fragments.
- Bottom-side mirroring and side intent for silkscreen text.
- Polarity and pin-1 indicators present and visible for polarized parts,
  connectors, ICs, diodes, LEDs, electrolytics, and batteries.
- Reference designator completeness, duplicate refdes detection, refdes outside
  board outline, unreadable refdes, and assembly drawing consistency.
- Fabrication marking checks: date code, UL mark, impedance coupon label,
  serialization, revision text, and customer-required markings in allowed zones.
- Fiducial label and keepout clarity: global/local fiducials not covered by
  silk, mask, copper clutter, or components.

### Assembly and DFA Checks

- Component-to-component clearance using courtyard, body, and height data.
- Component-to-board-edge clearance for pick-and-place, depanelization, clamps,
  rework, and hand soldering.
- Component-to-hole/slot/mechanical clearance for screws, standoffs, chassis,
  keepout volumes, and connector mating envelopes.
- Orientation consistency for polarized packages and same-package arrays.
- Tombstoning risk: asymmetric pad sizes, thermal imbalance, paste imbalance,
  copper imbalance, and unequal trace connections on small passives.
- Fine-pitch bridging risk: pad pitch, paste aperture spacing, mask dam absence,
  and soldermask-defined/NSMD mismatch.
- QFN/DFN thermal pad rules: paste windowpane, via-in-pad fill/tent intent,
  solder voiding risk, and exposed pad copper balance.
- BGA assembly risk: pitch class, escape route feasibility, dogbone via size,
  microvia requirements, soldermask web, inspection accessibility, and X-ray
  requirement flag.
- Connector/rework access: soldering iron access, cable insertion direction,
  latch clearance, mating height, and keepout zones.
- Test point readiness: minimum probe diameter, soldermask opening, net
  coverage, side accessibility, spacing, height clearance, and no soldermask or
  silkscreen obstruction.
- Fiducials: global/local count, symmetry, copper diameter, mask opening,
  keepout, edge clearance, and side coverage.
- Panel fiducials/tooling: panel-level fiducials, tooling holes, rails, bad-board
  marks, breakaway tabs, mouse bite hole size/spacing, and V-score residual web.
- Double-sided assembly: heavy parts on second side, reflow shadowing, adhesive
  requirements, and through-hole/wave conflicts.
- Selective or wave solder readiness: component side, solder thieves, keepouts,
  orientation, shadowing, and thermal relief around through-hole pins.
- Moisture/cleanliness/coating: conformal coating keepouts, no-clean flux risk
  under low-standoff packages, and unmasked test pads in coated areas.

### Electrical and Functional Validation Checks

- Netlist parity: schematic-to-PCB net consistency, missing nets, extra copper
  islands, unconnected pads, and intentional no-connect handling.
- Same-net continuity: broken traces, missing zone refill, unstitched plane
  islands, disconnected thermal spokes, and trace severed by holes or slots.
- Different-net shorts: copper overlap with net awareness, inner-layer antipad
  shorts, drill breakout shorts, and paste/mask-induced assembly short risk.
- Differential pairs: pair membership, skew, intra-pair spacing, width,
  neck-downs, pair-to-pair spacing, layer-change symmetry, and reference plane
  continuity.
- High-speed return paths: reference-plane void crossings, split-plane
  crossings, missing stitching vias at layer changes, and loop-area excursions.
- Power integrity: decoupling capacitor proximity/orientation, plane neck-downs,
  via count per rail, high-current bottlenecks, and starved regulator thermal
  pads.
- Analog/digital/RF segregation: region keepouts, noisy-net proximity, guard
  traces, via fences, antenna keepouts, and copper-free regions under inductors
  or antennas.
- ESD/safety: creepage and clearance by voltage class, slot barriers, spark-gap
  geometry, protective earth spacing, fuse/MOV keepouts, and high-voltage
  silkscreen warnings.
- Thermal validation: thermal via arrays, copper area under heat-generating
  parts, thermal relief versus heat spreading, hot component spacing, and
  heatsink/mechanical keepouts.
- EMC readiness: edge-rate nets near board edge, connector return pins, chassis
  stitching, ground moat mistakes, cable shield connection intent, and loop
  antenna risk.

### Manufacturing File and Pre-Production Workflow Checks

- File completeness: expected copper, soldermask, paste, silkscreen, drill,
  rout, fab drawing, assembly drawing, centroid, BOM, netlist, and readme files.
- Layer count parity: declared order layer count, stackup, Gerber set, KiCad
  stackup, and filename conventions agree.
- Layer role inference: Gerber X2 attributes, file extensions, JLC-style names,
  KiCad names, and explicit CLI role overrides resolve to a coherent stack.
- Polarity and mirroring: negative planes, bottom-layer mirroring, text
  orientation, drill origin, and coordinate origin consistency.
- Drill file checks: plated/non-plated split, duplicate drill files, missing
  tools, unsupported units/zeros, route slots versus drill hits, and tool
  diameter outliers.
- Gerber sanity: empty layers, tiny aperture flashes, unbounded fills, huge
  areas, malformed regions, duplicate layers, stale plot files, and mixed
  revisions.
- Order-parameter parity: board thickness, copper weight, soldermask color,
  surface finish, impedance, castellations, edge plating, via fill, controlled
  depth, and panelization options match file content.
- Revision consistency: board revision appears consistently in fab drawing,
  silkscreen, README, filename, BOM, placement file, and source metadata.
- Preflight sequence: refill zones, run EDA DRC/ERC, generate fresh fabrication
  outputs, reload outputs into independent viewer/parser, run HyperDRC, generate
  overlay artifacts, review waiver diff, and archive exact submitted package.
- Waiver governance: require reason, owner, expiry/review date, source link,
  and unchanged geometry hash for production waivers.
- CI gating: fail on errors, warn on fabricator-cost escalations, attach SVG and
  GeoJSON artifacts, compare violation count against baseline, and block stale
  generated outputs.
- Engineering review packet: checklist summary, stackup, rule deck, plots,
  DRC/ERC reports, BOM/centroid checks, and open manufacturing questions.

### Data Sources to Consider Next

- Gerber X2/X3 attributes for layer role, net, component, and aperture intent.
- IPC-D-356 access-side, feature type, soldermask flags, net names, and drill
  diameter records.
- KiCad board setup: net classes, constraints, custom DRC rules, stackup,
  differential pair settings, pad properties, via properties, zones, component
  footprints, 3D/body boxes, and fabrication output settings.
- Pick-and-place and BOM files for assembly-side, package, refdes, value,
  population, rotation, and centroid consistency.
- Fabrication and assembly drawings, parsed initially as metadata sidecars or
  structured JSON notes.
- ODB++ or IPC-2581 for richer stackup, components, nets, drill spans, materials,
  and manufacturing intents.

## Test Ideas From Fabrication DRC Guidance

Fabrication-facing guidance from PCBWay, JLCPCB, Eurocircuits, KiCad, Sierra
Circuits, and other DFM references should be turned into small synthetic
Gerber/KiCad fixtures with one clear violation and one matching non-violation.

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
- Soldermask process fixtures: LDI versus conventional/PI mask examples with
  different minimum web, mask annular ring, and mask-overlap clearances.
- Voltage clearance fixtures: same geometry passing at low voltage and failing
  at high voltage, with internal/external layer variants and optional coating.
- Aspect-ratio fixtures: board thickness and drill diameter combinations that
  pass/fail standard via, microvia, blind via, and through-hole limits.
- Copper-balance fixtures: large pour imbalance, isolated local copper density
  islands, and dense copper regions near sparse opposite-side geometry.
- Board-outline fixtures: unordered outlines, self-crossing outlines, duplicate
  outlines, tiny route notches, nested cutouts, and route slots touching copper.
- File-package fixtures: stale generated Gerber files, missing paste/mask/drill
  files, mismatched declared layer count, duplicate layers, mixed units, and
  mismatched revision strings.
- DFA fixtures: component courtyard collisions, component too close to edge,
  missing global fiducials, testpoint spacing failures, QFN thermal pad paste
  overfill, and via-in-pad without fill/cap metadata.
- Electrical-readiness fixtures: missing zone refill, unconnected copper island,
  split-plane crossing, differential-pair skew, missing return stitching via,
  and high-current bottleneck.

## Implemented Reporting

- Stable violation IDs derived from check name, layer names, island index, and
  rounded coordinates.
- Text, JSON, and GeoJSON output.
- SVG violation overlays for quick local or CI artifact review.
- Severity on each violation.
- Structured JSON input manifest entries with adapter, role, path, and
  conversion-origin provenance.
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

## IO Sources and Sinks Roadmap

The project should treat IO as a set of adapters around a shared internal board
and layer model. Direct Rust crates are preferable for deterministic CI behavior,
but command-line adapters are still valuable when an EDA tool already provides a
well-maintained exporter.

### Highest-Value Sources

- KiCad native files via `kiutils-rs` or `kiutils_kicad`: replace or augment the
  hand-rolled KiCad parser with a typed, lossless KiCad API. Useful inputs:
  `.kicad_pcb`, `.kicad_pro`, `.kicad_dru`, `.kicad_sch`, `.kicad_mod`,
  symbol/footprint library tables, and worksheet/title-block files. This would
  unlock native net classes, custom DRC rules, stackup, title/revision data,
  footprint metadata, pad properties, and schematic/PCB parity checks.
- KiCad CLI as a conversion/check backend: shell out to `kicad-cli` for DRC,
  Gerber, drill, DXF, SVG, PDF, IPC-D-356, IPC-2581, ODB++, STEP, GLB, PLY,
  position-file, and statistics exports. This is the fastest path to high
  fidelity for formats that KiCad already owns.
- Gerber X2/X3 and Gerber job files through `gerber_parser` / `gerber-types`:
  mine attributes for layer role, net/component metadata, file function,
  polarity, board profile, drill map, and job-level manifest data. Keep `csgrs`
  for geometry if it remains best for boolean operations, but use parser-level
  metadata to reduce manual layer-role flags.
- Gerber package libraries through Gerbonara as an optional Python adapter:
  useful for robust folder-level file-role inference across real-world CAD
  packages from KiCad, Altium, Eagle, Allegro, gEDA, Fritzing, PADS, and
  Target3001. Rust can drive this through a command-line or Python subprocess
  bridge if direct Rust coverage is insufficient.
- GenCAD via the `gencad` crate: import a richer manufacturing/test model with
  components, nets, routes, and part data. This is attractive for DFT/testpoint
  readiness and assembly checks because GenCAD was designed around PCB
  fabrication and testing interchange.
- DXF/DWG mechanical data via `dxf` or `acadrust`: import enclosures,
  panel drawings, mechanical keepouts, board outlines, slots, fab notes, and
  assembly fixture constraints. `dxf` is a mature read/write path; `acadrust`
  broadens scope to DWG and ACIS/SAT but should be evaluated for stability and
  license fit.
- SVG drawings via `usvg`: import laser/mechanical outlines, vendor templates,
  board-edge graphics, and review annotations. `usvg` resolves CSS, inherited
  attributes, basic shapes, and path commands into simpler absolute paths.
- BOM and placement files via `csv`, `serde`, and `calamine`: parse CSV/TSV,
  JSON, XLS/XLSX/XLSM/XLSB/ODS BOMs, centroid files, AVL/AML sheets, and
  assembly notes. This supports refdes completeness, DNP variants, centroid
  sanity, side/rotation checks, and manufacturer order-parameter parity.
- ZIP/TGZ/package ingestion via Rust archive crates: accept manufacturer upload
  packages directly, unpack to a temp workspace, detect Gerber/drill/BOM/PNP
  roles, and preserve a manifest of exactly what was checked.

### Medium-Value or Specialized Sources

- IPC-2581 / IPC-DPMX through XML parsing: IPC-2581 is an open, bidirectional
  PCB design/manufacturing exchange format and includes fabrication and assembly
  information. There does not appear to be a mature Rust-native parser today,
  but it is XML, so an incremental parser using `quick-xml`, `roxmltree`, or
  `xml-rs` is realistic. Existing web tooling such as BoardUI can serve as a
  reference for object modeling.
- ODB++ through external services or adapters: ODB++ is data-rich and widely
  used for CAM/assembly, but the format is proprietary. Options include parsing
  a practical subset, driving KiCad's ODB++ exporter, using an external
  extractor, or integrating a service such as OdbDesign. Note licensing risk:
  OdbDesign is AGPL for the open-source edition, so use it as a subprocess or
  optional service only if license constraints are acceptable.
- STEP/STEPZ for enclosure and mechanical validation: use KiCad CLI export,
  OCCT-based tooling, `ruststep`, or `vcad` depending on whether the goal is
  metadata, B-rep geometry, or simple collision envelopes. Rust-native STEP
  support exists but may be limited for full AP242 assembly/GD&T workflows; OCCT
  remains the mature path through C++ or external tools.
- 3D mesh formats: STL, OBJ, PLY, GLB/glTF, and U3D can be useful sinks for
  review artifacts and collision envelopes. Rust has `stl_io`, glTF ecosystem
  crates, and `vcad` export paths; these are best treated as visualization or
  approximate mechanical-clearance sinks rather than authoritative CAD inputs.
- PDF/PostScript/HPGL: useful mostly as human review sinks or fab-drawing
  sources. Direct semantic extraction is weak; prefer producing PDFs from
  structured report/overlay data and using PDF parsing only for rough metadata
  or text-note extraction.
- Image inputs: PNG/TIFF renderings from Gerber, KiCad, or vendor viewers can be
  used for visual regression tests, ML-assisted review, or review overlays.
  They should not be the source of truth for dimensional checks.
- Pick-and-place variants and assembly drawings: many CMs require only refdes,
  X, Y, rotation, and side for centroid data. Add explicit parsers for common
  KiCad `.pos`, Altium CSV, JLCPCB CPL, and generic centroid schemas.
- Test and inspection formats: IPC-D-356, GenCAD, boundary-scan netlists,
  flying-probe reports, AOI placement exports, and bed-of-nails fixture formats
  can improve DFT coverage and production feedback loops.
- Vendor/order APIs: JLCPCB, PCBWay, Eurocircuits, DigiKey, Mouser, Octopart,
  Nexar, and internal PLM/ERP APIs could provide capabilities, stackup options,
  BOM availability, lifecycle status, pricing, and order-parameter parity. Keep
  these behind optional network-enabled adapters and cache results for CI.

### Highest-Value Sinks

- Gerber/Excellon output: export violation overlays or generated keepout layers
  as Gerber so designers can load findings in existing PCB viewers.
- KiCad markers or rule areas: write warnings back into `.kicad_pcb` as markers,
  graphics, zones, or generated user layers once lossless KiCad writing is
  available.
- KiCad `.kicad_dru`: generate or update custom rule files from HyperDRC rule
  decks so designers can catch issues earlier in KiCad.
- IPC-D-356 output: emit an annotated electrical-test netlist or diff showing
  expected test access, uncovered nets, and drill/net mismatches.
- IPC-2581 or ODB++ export: longer-term single-package manufacturing handoff
  with embedded DRC/DFM annotations, if library/tool support matures.
- GenCAD output: useful for DFT/test fixture workflows and integration with
  test engineering tooling.
- DXF/SVG/PDF overlays: produce human-review artifacts for mechanical, fab, and
  assembly teams. SVG already exists; DXF/PDF would make review easier in CAD
  and document workflows.
- HTML report bundle: self-contained interactive report with SVG/canvas
  overlays, layer toggles, waiver state, and source file manifest.
- SARIF/JUnit/GitHub annotations: CI-native sinks so DRC findings appear in code
  review tools, dashboards, and quality gates.
- JSON Lines / SQLite / Parquet: structured machine-readable sinks for trend
  analysis across many boards, vendors, revisions, and rule decks.
- Waiver and baseline update sinks: generate proposed waiver stubs, expired
  waiver reports, and geometry-hash baselines for controlled production
  exceptions.

### Adapter Architecture Notes

- Keep `IoAdapter` boundaries explicit: `discover`, `read`, `convert`, `write`,
  and `capabilities`. A format may support only one operation.
- Separate semantic richness from geometry richness. A Gerber may have excellent
  polygon geometry but poor design intent; KiCad/IPC-2581/ODB++ may carry nets,
  components, stackup, and manufacturing intent.
- Preserve provenance for every loaded object: source file, adapter, layer,
  units, original identifier, and transformation history. Initial report-level
  provenance is implemented for direct Gerbers, Gerber directories, converted
  Gerbers, KiCad boards, Excellon drills, IPC-D-356 netlists, and waivers.
- Treat external tools as hermetic conversion steps with captured command line,
  version output, stdout/stderr, input hash, and output manifest.
- Prefer optional Cargo features for heavy or license-sensitive integrations:
  `io-kicad-typed`, `io-dxf`, `io-svg`, `io-xlsx`, `io-gencad`,
  `io-ipc2581`, `io-step`, and `io-archives`.
- Provide fixture corpora by adapter: tiny synthetic files for unit tests, plus
  opt-in real-world packages for integration tests.

## Research Sources

- KiCad PCB Editor documentation: DRC should be run before generating
  manufacturing files, with zone refill and board/schematic parity checks
  considered part of preflight; KiCad also documents DFM-oriented DRC classes
  such as board-edge and hole clearances.
- PCBWay DRC and layout guidance: common checks include minimum line width,
  spacing between traces/pads/vias, via size, power/ground trace sizing,
  critical-signal routing, differential-pair/guard-trace constraints, and
  analog/digital/high-power/high-frequency separation.
- JLCPCB/TransJLC ecosystem: TransJLC normalizes Gerber packages from common
  EDAs to JLCEDA/JLCPCB-style naming; JLCPCB capability discussions emphasize
  via diameter, via hole size, and annular-ring interpretation.
- Eurocircuits PCB Checker and tolerances: DRC compares measured values against
  configured track, isolation, and annular-ring limits; DFM adds plating index,
  copper area, solderpaste surface, exposed copper, and fiducial-like pad
  information. Their tolerances also highlight bow/twist, drill tolerances,
  hole-to-hole, NPTH-to-copper, soldermask web/opening/overlap, legend, edge
  clearance, slot width, and profile tolerance.
- Eurocircuits soldermask guidance: soldermask openings are usually negative
  data and larger than copper pads; mask annular ring, mask segment/web, mask
  overlap clearance, minimum opening, via tenting/filling, NPTH mask openings,
  and board-outline mask behavior all affect manufacturability.
- Sierra Circuits DFM/DFA guidance: annular-ring breakout, drill spacing,
  via-in-pad filling/capping, solder wicking, component footprint correctness,
  and soldermask bridge sizes are common production-delay causes.
- General DFM references: checklist themes repeatedly include trace/space,
  annular ring, aspect ratio, soldermask/paste alignment, silkscreen clipping,
  component spacing, thermal relief, copper balance, test points, fiducials,
  documentation completeness, and pre-production review packets.
- KiCad CLI documentation: exposes useful source/sink conversions for Gerber,
  Excellon, DRC, DXF, GenCAD, IPC-D-356, IPC-2581, ODB++, PDF, position files,
  statistics, STEP, SVG, and 3D formats.
- Rust IO ecosystem notes: `kiutils-rs`/`kiutils_kicad` cover typed/lossless
  KiCad files; `gerber_parser` covers Gerber parsing with `gerber-types`;
  `gencad` parses GenCAD; `dxf` reads/writes DXF/DXB; `acadrust` targets
  DXF/DWG/ACIS; `usvg` parses SVG; `calamine` reads Excel/OpenDocument
  spreadsheets; `ruststep` and `vcad` are possible STEP/CAD paths.
