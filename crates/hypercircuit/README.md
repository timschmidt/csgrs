# hypercircuit

`hypercircuit` is the Rust-native declarative circuit and PCB semantic layer for
the Hyper ecosystem. It retains components, typed pins, nets, buses, ports,
footprints, placements, stackups, routing constraints and PCB feature identity,
alongside Modified Nodal Analysis (MNA), residual replay and multiphysics
coupling reports. Circuit truth uses `hyperreal::Real`; numerical solvers are
proposal engines whose candidates can be checked against authored equations.

The ownership boundary is intentional: `hyperpath` supplies exact routing/path
carriers, `csgrs` materializes profiles and solids, and `hyperdrc` verifies
constraints and release readiness. Circuit and PCB semantics do not belong in
the geometry engine. See the [capability matrix](CAPABILITY_MATRIX.md) for the
tscircuit/via-rs equivalence ledger and remaining acceptance gates.

With `geometry`, `LegacyCsgrsElectronicsImport` can still read the versioned
JSON handoffs captured before csgrs's package/electrical marker removal. The
live csgrs metadata API no longer contains those variants or electrical
terminal roles. Persisted claims retain typed authoring omissions and never
infer nets, connectivity, pins, pad geometry, placement, or manufacturing
policy from a mesh. This migration-only reader is scheduled for removal in
HyperCircuit 0.4.0; users must resolve its omissions into ordinary authored
HyperCircuit semantics and persist a current `SemanticDocument` before then.

The crate is not a complete SPICE simulator. Its current executable path builds
small dense linear DC and complex AC MNA systems, certifies proposed solutions,
and performs exact trapezoidal or backward-Euler capacitor/inductor companion
solves. Linear AC analysis stamps exact rectangular phasors for R/L/C,
independent voltage/current sources, and VCCS devices at caller-authored exact
positive angular frequencies. Ordered sweeps retain the shared `MnaUnknown`
identity order and replay every real and imaginary residual exactly. AC source
amplitudes are explicit component-addressed `AcExcitation` directives; a source
without one has zero small-signal amplitude rather than implicitly reusing its
DC value.
Transient steps compose into bounded fixed or adaptive exact time-series runs
driven by retained constant, step, piecewise-linear, indefinitely periodic
pulse, damped sine, or dual-edge exponential independent-source stimuli. Exact
source breakpoints truncate coarser proposed steps.
Retained Shockley diodes additionally use bounded primitive-float Newton
linearization proposals whose imported coefficients and exact MNA candidates
are audited; acceptance replays the true exponential law against exact
voltage/current tolerances. Fixed-step mixed R/C/L/diode transient runs retain
stable waveforms and per-endpoint convergence evidence. Retained three-terminal
N/P MOSFETs additionally execute a body-tied-source Shichman-Hodges square law
for DC: cutoff, triode and saturation regions, exact derivatives, every affine
Newton proposal and final true-law residual replay are retained. Accepted
diode and MOSFET Newton reports can be promoted into an opaque
`AcOperatingPoint`; small-signal sweeps then stamp the diode incremental
conductance or exact MOSFET `gm`/`gds`, retain operating-region and derivative
evidence, and exactly replay every complex residual. The diode exponential
slope remains an explicit primitive-float proposal/import boundary. Dynamic
MOSFET capacitance, body effect, subthreshold and reverse-channel behavior,
sparse matrices, adaptive nonlinear integration, and coupled-field solvers
remain explicit adapter boundaries. See
[`examples/ac_sweep.rs`](examples/ac_sweep.rs) for a declarative RC sweep and
[`examples/nonlinear_ac.rs`](examples/nonlinear_ac.rs) for a checked
common-source DC operating point and exact small-signal sweep.

## Optional PCB layers

Enable `layout` for declarative PCB types and `hyperpath` lowering. Enable
`geometry` to additionally materialize substrate, source-addressable copper and
drill records through `csgrs`. Enable `drc` for a direct, audited conversion to
`hyperdrc` board, stackup and net-class inputs. Enable `interchange` for the
versioned, exact-value-preserving circuit/schematic JSON document. Enable
`lceda` for the audited EasyEDA/LCEDA Pro project adapter:

```toml
[dependencies]
hypercircuit = { version = "0.3.0", features = ["layout"] }
```

## Project command line

With `drc` and `interchange`, the `hypercircuit` executable consumes the same
versioned `SemanticDocument` used by the Rust API. It does not construct a
second connectivity or layout model:

```text
cargo run --features drc,interchange --bin hypercircuit -- check
cargo run --features drc,interchange --bin hypercircuit -- ir --out design.json
cargo run --features drc,interchange --bin hypercircuit -- snapshot --out snapshot.json
cargo run --features drc,interchange --bin hypercircuit -- bom --out bom.csv
cargo run --features drc,interchange --bin hypercircuit -- \
  export-kicad main review/board.kicad_pcb
cargo run --features drc,interchange --bin hypercircuit -- \
  export-svg main review/board.svg
cargo run --features drc,interchange --bin hypercircuit -- \
  release main release/
```

Project commands search the current directory and its ancestors for a
versioned `hypercircuit.toml`. A project can expose several named designs and a
default. A command provider is an explicit argv vector; a Cargo provider runs
one package binary with optional features. Both must print exactly one
`SemanticDocument` JSON value to standard output:

```toml
schema = "org.hypercircuit.project"
version = 1

[project]
name = "controller"
version = "0.1.0"
default-design = "main"

[designs.main]
provider = "cargo"
package = "controller-designs"
binary = "main-board"
features = ["production"]

[designs.prototype]
provider = "command"
command = ["./target/debug/prototype-board", "--semantic-json"]

[[pcb-materials]]
handle = "hyperphysics:FR4"
relative-permittivity = "21/5"
loss-tangent = "0.018"
source-authority = "laminate-manufacturer"
source-locator = "datasheet/revision-a"
source-freshness = "2026-06"
condition = "1 MHz nominal"
```

The Cargo provider's `main` can author with the ordinary Rust API and finish
with `println!("{}", document.to_json_pretty()?)`. Passing an existing JSON
path instead of a design name remains supported for direct interchange and CI
fixtures. `ir` and `snapshot` canonicalize provider output through parse,
schema migration/validation, and deterministic serialization before writing or
printing it. `bom` resolves retained placement constraints before deriving
assembly rows. Optional `[[pcb-materials]]` entries use exact decimal or
rational strings and required source provenance. `check` and `release` resolve
them into the same HyperPhysics property graphs accepted by
`ReleasePreparationOptions::pcb_materials`; duplicate handles, empty
provenance, nonpositive relative permittivity and negative loss tangent fail
manifest loading. Direct JSON paths intentionally receive no implicit material
catalog.

`check` replays semantic validation and ERC; when PCB intent is attached it
also builds the full placement, csgrs materialization, HyperDRC, fabrication,
CAM re-import, and assembly evidence in memory. `release` writes the audited
Gerber X2, Excellon, IPC-D-356, manifest, BOM, pick-and-place, and DNP bytes.
Both commands exit with status 2 when typed release blockers remain, while
malformed input or impossible evidence construction exits with status 1.
KiCad export writes same-stem `.kicad_pro` and `.kicad_dru` companions when
retained rules require them and reports any typed omissions.

`Design` is the concise authoring front door over the retained `Circuit`,
`SchematicLayout`, and `PcbLayout` IRs. Typed handles prevent cross-design
net/pin connections. `Design::define_part` lowers a reusable `PartDefinition`
exactly once into a device model, optional multipart symbol definition, and
optional land pattern. `Design::instantiate` then adds only an instance's
parameter values, waveform, selected symbol-unit placements, and board
placement; every resulting `PartDefinitionHandle` is design-scoped and exposes
the stable retained library identities. Two or two thousand instances can
therefore share one electrical interface, exact drawing library, footprint,
and pin-to-pad map. `PartSymbolUnit` keeps definition-local pins and exact
graphics separate from `SymbolUnitPlacement`, while `PartInstance` keeps
reference/value/transform data separate from the reusable definition. See
[`examples/reusable_parts.rs`](examples/reusable_parts.rs).

With `interchange`, `Design::export_part` and `Design::import_part` move that
same retained definition through a `PortablePartDefinition`; they do not copy
it into a package-specific circuit model. `PartLibraryArtifact` binds named
definitions, exact semver, and dependency requirements into versioned canonical
JSON. `CircuitPackageStore` publishes immutable SHA-256-addressed files,
recognizes identical cache hits, loads only exact lock coordinates, replays
artifact validation after parsing, and rejects digest or package/version
disagreement. The catalog can therefore resolve and verify a lock before a
fresh `Design` imports and instantiates the selected definition. Network
registry authentication and transport remain external adapters over these
verified bytes. See
[`examples/part_library.rs`](examples/part_library.rs).

`Design::add` remains the concise one-off convenience and atomically lowers one
fluent `Part` into a private model, instance, optional schematic symbol
definition and placement, embedded footprint, pin-to-pad map, and placement.
`connect` remains the sole logical binding operation and also chains every
visible endpoint with typed schematic wires; it does not create a second
connectivity graph. `finish` returns only after the circuit, schematic, and PCB
validators pass. Conventional same-name pins and pads map automatically;
explicit `PartPin::pad` calls retain multi-land mappings. Exact primitive
constructors in `parts` lower directly into the existing simulation models:

```rust
let mut d = Design::new(
    "divider",
    BoardOutline::rectangle(Real::from(20), Real::from(12)),
    PcbStackup::single_layer(Real::one(), None),
)?;
let vcc = d.rail("VCC", Some(Real::from(5)), None, RailKind::Power)?;
let gnd = d.ground("GND")?;
let r1 = d.add(
    parts::resistor("R1", Real::from(1_000))
        .symbol(Symbol::two_pin_horizontal(
            SchematicPoint::new(Real::from(10), Real::from(4)),
            Real::from(2), Real::from(3), Real::from(2),
        ))
        .footprint(Footprint::two_pad_smd(
            Real::one(), Real::one(), Real::from(2), vec![TraceLayer(0)],
        ))
        .at(Point2::new(Real::from(10), Real::from(6))),
)?;
d.connect(&vcc, [r1.pin("1")?])?;
d.connect(&gnd, [r1.pin("2")?])?;
let checked = d.finish()?;
```

Parts do not need manually authored drawing coordinates. On a checked design,
`replace_schematic_with_auto_layout` derives generic symbol units, pin sides,
typed orthogonal wires, boundary-port placements, and net labels directly from
the authoritative `Circuit`. Non-ground connectivity drives deterministic
breadth-first columns; independent sources and driving pins seed each connected
component, while global ground is excluded from placement adjacency so it
cannot collapse an entire design into one column. Exact spacing and body policy,
a hard instance bound, every column/row choice, edge count, and generated
artifact count remain inspectable in `SchematicAutoLayoutReport`. Replacement
is explicit so a hand-authored drawing is never overwritten implicitly. See
[`examples/auto_schematic.rs`](examples/auto_schematic.rs).

`SchematicLayout::export_kicad_schematic` lowers a flat, circuit-bound drawing
to a native KiCad 9 `.kicad_sch` file. Reusable `SchematicSymbolDefinition`
records own model binding, independently placeable multipart units, exact
line/rectangle/circle/arc/polyline/text graphics and logical pin presentation;
`SchematicSymbol` placements retain only instance, definition, unit and
transform identity. Each used definition is embedded once per native file and
shared by every placement. Electrical pin kinds, symbol/port placement, local
net labels and typed polylines are emitted without introducing a KiCad-owned
circuit model. Exact
coordinates cross a recorded finite-decimal boundary; each circuit wire is
simplified and split into KiCad's native two-point wire records with an ordered
`KiCadSchematicWireProjection`. `KiCadSchematicImportReport::from_str` and
`from_path` reconstruct an editable `SchematicLayout` against a caller-supplied
authoritative `Circuit`, retain symbol/instance identities through hidden
properties, import native coordinates and reusable graphics as exact decimals,
coalesce native two-point segments back into semantic wires, and reject any
resulting circuit disagreement. Native wire/label UUIDs become audited new
drawing identities. See
[`examples/multipart_symbol.rs`](examples/multipart_symbol.rs) for a shared
two-unit definition rendered through SVG and KiCad.

Standalone native libraries can enter the same retained path.
`KiCadPartLibraryImportReport::from_str`/`from_paths` pairs one named
`.kicad_sym` entry with one `.kicad_mod` footprint and produces a
`PortablePartDefinition`. The adapter resolves symbol inheritance, merges
unit-zero common pins/graphics with independently placeable multipart units,
maps native electrical pin kinds, retains exact line/rectangle/circle/arc/
polyline/text symbol graphics, imports exact pads, round and slotted drills,
pin maps, mask/paste margins, and supported footprint artwork, and records
every native decimal. Pad-local rotation is retained exactly through
materialization, DRC-visible geometry, SVG/LCEDA review, semantic JSON and
KiCad board/library round trips. Every external package-model URI and exact
offset/scale/rotation is retained independently of the optional fallback body,
including multiple models per footprint. Alternate De Morgan bodies, pin
graphic shapes, custom pad primitives, unsupported artwork/layers, and paste
ratios remain typed `KiCadLibraryImportOmission`s rather than disappearing.
Controlled fixtures plus installed official `Device:R`/0603 and multipart
`74LS00`/DIP-14 regressions exercise the adapter. The command-line
[`examples/kicad_library_import.rs`](examples/kicad_library_import.rs) writes
the result directly as a versioned portable part-library artifact.

Explicit `SchematicSheet` trees use
`export_kicad_schematic_book`: every page receives one native file, stable
filename/file UUID/page/path evidence, and each parent-side
`SchematicSheetPort` becomes a sheet pin paired with the linked child-side
hierarchical label. Hidden length-prefixed link metadata retains sheet, link,
parent/child port, original display-name and net identities without a sidecar.
`KiCadSchematicBookImportReport::from_files` recursively follows `Sheetfile`
references, rejects cycles/missing files/duplicate sheet claims, imports every
page in endpoint-safe passes, restores page membership, and validates the whole
book against the supplied `Circuit`. KiCad-required shared boundary names have
typed projection omissions when parent and child display names differ. KiCad 9
CLI regressions render empty, symbol, port, label, complete flat, and generated
two-page hierarchy files. Arbitrary third-party symbol libraries remain outside
the self-emitted semantic subset. See
[`examples/kicad_schematic.rs`](examples/kicad_schematic.rs) and
[`examples/hierarchical_schematic.rs`](examples/hierarchical_schematic.rs).

```text
cargo run --example kicad_schematic -- divider.kicad_sch
kicad-cli sch export svg --output review divider.kicad_sch
cargo run --example hierarchical_schematic -- hierarchy-output
cargo run --features interchange --example kicad_library_import -- \
  /usr/share/kicad/symbols/Device.kicad_sym R \
  /usr/share/kicad/footprints/Resistor_SMD.pretty/R_0603_1608Metric.kicad_mod \
  resistor-0603 resistor-0603.json
```

The same `parts` module exposes executable `diode`, `mosfet`, `nmos`, and
`pmos` declarations. Their exact Shockley or body-tied-source square-law
parameters and typed A/K or D/G/S terminal roles lower directly into
`solve_diode_dc` and `solve_mosfet_dc`; they are not display-only library
aliases.

Independent voltage/current sources accept `.waveform(SourceWaveform::...)`
during part construction, or a waveform can be attached later with
`Design::stimulus(&source_handle, ...)`. Both paths retain constant, step,
piecewise-linear, pulse, sine, and exponential intent; transient runners use
the exact authored breakpoints rather than an authoring-time sample table.

`Design::bus`, `Design::bus_slice`, and `Design::port` expose reusable circuit
interfaces without falling back to stringly raw vectors. Their handles retain
design scope and authored bit order; slices can preserve or reverse a checked
range, and boundary ports retain direction and optionality.

`Design::route`, `Design::via`, `Design::zone`, and `Design::keepout` provide
the same typed boundary for physical copper and exclusion intent. Fluent
`Route`, `Via`, `Zone`, and `Keepout` values accept exact hyperpath segments,
layer spans, mask/plating policy, rich pour policy, and typed keepout scopes.
Their design calls bind nets through `NetHandle`, reject duplicate, malformed,
foreign-handle, and absent-stackup-layer requests atomically, capture source
locations, and write directly into the retained PCB collections. The returned
`RouteHandle`, `ViaHandle`, `ZoneHandle`, and `KeepoutHandle` preserve stable
identity for downstream declarations.

`Design::constrain` accepts typed `PlacementRule` declarations over
`InstanceHandle`s. Fixed and relative equations, X/Y alignment, inclusive
regions, exact rotation sets, and board-side sets lower atomically into
`PlacementConstraint`; duplicate identities, foreign or unplaced instances,
invalid arity/regions, and multiple position drivers are rejected before
mutation. `PlacementConstraintHandle` and the source map retain the resulting
identity and call site. Structural `finish` preserves these equations; callers
can explicitly resolve or search placement, while hierarchical compilation
resolves child-local rules before applying parent transforms.

Routing and release policy has an equally typed path.
`Design::via_style` retains named plated constructions and checked stackup
spans; `Design::net_class` binds `NetClassRule` to typed nets, parent classes,
and preferred via styles; and `Design::differential_pair` couples two typed
nets with exact spacing and skew plus an optional reduced-width/spacing,
bounded terminal neck-down. The builders cover preferred/minimum width, clearance, via
geometry/count, length, impedance, reference-plane, mask, span, and pair-fanout
intent. `ViaStyleHandle`, `NetClassHandle`, and
`DifferentialPairHandle` prevent foreign-design references, while authoritative
rule validation rejects inheritance, geometry, duplicated-net, and pair errors
before mutation.

With the `drc` feature, `CheckedDesign::prepare_release` and
`CheckedProject::prepare_release` are the cohesive
verification/manufacturing handoffs for flat and compiled hierarchical work.
They resolve retained placement constraints, apply the solution to a reviewable
`resolved_layout`, and compose the existing authoritative ERC, csgrs
materialization, HyperDRC, fabrication, CAM re-import, BOM/pick-and-place, and
assembly CSV audit APIs. Hierarchical projects use their validated
path-qualified composed `Circuit`/`PcbLayout` while retaining reusable
definitions, schematics, scope maps, and source maps beside the report. The
resulting `ReleasePreparationReport` preserves every intermediate artifact and
reports typed `ReleaseBlocker`s; rule failures therefore remain inspectable
reports, while only unsatisfied placement or an impossible
materialization/fabrication stage returns `ReleasePreparationError`. This is
orchestration over retained containers, not another circuit or board IR.

`DesignModule` wraps a `CheckedDesign` as a reusable circuit/PCB definition.
`instantiate` accepts only design-scoped child `PortHandle`/parent `NetHandle`
pairs, rejects missing required ports and conflicting bindings atomically, and
retains exact parameter overrides and `LayoutTransform`s. Arbitrarily nested
modules compile through the authoritative `CircuitLibrary` and
`LayoutAssembly` carriers into a validated, path-qualified flat
`CheckedProject` ready for routing, DRC, materialization, and export. Nested
front/back mirroring, rotation, and translation compose exactly.
`LayoutModule` retains child-local placement constraints. Composition applies
placement-group transforms, validates and resolves the constraints in their
local coordinate frame, and only then transforms the resolved placements into
the parent; unsatisfied local predicates reject the complete composition
without emitting a partial flat board.

Advanced callers continue through `circuit_mut`, `schematic_mut`, and
`layout_mut`; the fluent
surface never hides or duplicates routes, zones, rules, hierarchy, simulation,
or manufacturing intent. Every fluent declaration, placement and connection
automatically captures its Rust `file:line:column`. `Design::check` correlates
authoritative circuit/schematic/layout findings into `DesignDiagnostic` records, and
also records escape-hatch mutation call sites as fallbacks for advanced
objects. `CheckedDesign::source_map` keeps that CI/editor provenance beside,
not inside, the portable `Circuit`, `SchematicLayout`, and `PcbLayout`; with `interchange`, the
source map can be serialized independently. See `examples/fluent_design.rs`.
`examples/source_diagnostics.rs` is a runnable intentionally-invalid design
that prints the correlated Rust call sites.

`LcedaProExportReport::from_design` emits a deterministic `.epro2` ZIP with
`project2.json` and a parseable JSON-record `.epru` stream. The mapping includes
reusable footprints, pads and pin/net assignments, authored schematic wires,
PCB placements, tracks, vias, copper zones, board outlines and cutouts. Callers
must declare whether PCB lengths are authored in millimetres, mils or inches;
every exact-to-decimal projection and every unsupported or approximated detail
is returned in the report. The official Pro documentation currently labels its
detailed format incomplete and announces a new V3 format, so every archive also
carries an editor-conformance review marker. Typed keepouts, detailed material semantics,
arbitrary pad polygons, routed slots and native hierarchical sheet-symbol
relationships remain explicit review items.
`LcedaProImportReport::from_archive` applies the supported PCB editor subset
against caller-supplied authoritative circuit/layout truth. Stable extension
identities restore placement transforms, straight route segments, authored
vias, zone boundaries and straight exterior/cutout segments; every mil token
is reconstructed as an exact value in the declared source unit. Circuit
connectivity plus footprint, pad, stackup, constraint and keepout definitions
remain explicitly baseline-owned, curved contours retain their exact baseline
geometry, and ZIP/record/target/result failures are typed. The importer requires
one complete, duplicate-free set of stable placement, supported route-segment,
authored-via, zone and outline identities; deleted records, duplicate records,
net reassignment and footprint reassignment fail atomically. See
`examples/lceda_pro_export.rs` for a runnable millimetre-authored
export/import/re-export cycle.
`LcedaSchematicImportReport::from_archive` independently restores the supported
schematic presentation subset: stable symbol position/quarter-turn transforms,
label position/text, free junction positions and complete wire polylines.
Circuit nets, typed pin/port endpoints, sheet membership and reusable symbol
definitions remain authoritative in the baseline. Imports require a complete,
duplicate-free rendering of every retained symbol, label and wire; connectivity
edits, missing records, discontinuous segments and invalid results fail
atomically. Exact schematic-coordinate reconstructions and preserved anchored
endpoints are reported as evidence, and the runnable example exercises both
PCB and schematic import before deterministic re-export.

The retained PCB model includes exact mixed line/circular-arc/cubic-Bezier
`BoardContour` outlines and cutouts, ordered stackups, land patterns,
pin-to-pad maps, front/back placements, traces, vias, zones, keepouts, net
classes and differential pairs. Contour curves remain hyperpath carriers and
materialize directly into Hypercurve/csgrs regions; they are never recovered
from a tessellation. `BoardOutline::boundary_geometry` is the canonical
exterior-minus-cutouts lowering: its `BoardBoundaryGeometry` retains exact
Hypercurve paths, the filled region, curve-extrema bounds, point/disc/segment/
axis-aligned-envelope predicates, and cached exact insets. Materialization,
placement, negotiated routing, stitching and the HyperDRC handoff consume that
same representation. Line/circular-arc clearance uses exact point/segment
distance predicates; certified region insets are retained as the general path.
A general cubic parallel offset is reported as indeterminate instead of being
silently tessellated. `PcbLayout::validate` performs structural checks; it is
not a substitute for `hyperdrc`.
Fabrication preserves exact line and circular-arc profile primitives. Cubic
exteriors and cutouts remain exact in the retained layout, while
`FabricationContourProjectionPolicy` makes their CAM boundary explicit: the
default rejects them, and `CubicBezierPolyline` adaptively emits line segments
under a caller-selected finite positive chord error in the declared source
unit. Manifest evidence records the contour identity, retained segment index,
requested bound, and generated segment count. See
`examples/curved_fabrication.rs`.

Placement intent can retain fixed origins, exact relative offsets, row/column
alignment, inclusive board regions, allowed exact rotations, and allowed board
sides under stable constraint identities.
`PcbLayout::resolve_placement_constraints` deterministically applies the exact
fixed/relative equations and reports cycles or unsatisfied predicates.
`PcbLayout::solve_placement` then preserves legal authored origins and searches
a caller-bounded, row-major exact grid for unconstrained packages. Fixed and
relative groups remain locked; same-side courtyard, body, or pad fallback
envelopes receive conservative clearance. Exterior and cutout containment use
exact point/segment predicates over the transformed conservative envelope.
Legal candidates are ranked by an exact retained score. Opt-in
`pin_access` first audits every mapped terminal on every physical copper layer
in all four cardinal directions at a caller-authored exact probe distance,
minimum trace width, and minimum clearance. It shares the negotiated router's
board/cutout, copper-keepout, fixed straight-route/via, transformed foreign-pad,
net/regional-width, route-constraint, and terminal escape-policy legality.
`PlacementPinAccessReport` retains instance/pin/pad/net/center and every
layer/direction result as accessible, blocked, or indeterminate; incomplete
final evidence becomes a typed placement issue. Candidate ranking first
minimizes fully blocked terminals and blocked probes. Opt-in
`minimize_routing_congestion` then reduces overlap pressure between the
axis-aligned pad-terminal boxes of distinct logical nets. An optional exact
`density_radius` penalizes same-side origins inside that caller-authored
Manhattan radius. Exact ratsnest length derived from circuit pin nets and
footprint pin-to-pad mappings, origin displacement, and orientation changes
complete the lexicographic score. Disabled pin-access/routability/density terms
retain exact zero costs, preserving connectivity-first behavior. Authored legal
origins remain fixed unless optimization is explicitly enabled. Fixed/relative
origins remain locked while retained rotation and side choices can still
resolve their envelopes. The result records each position, rotation, or side
move, both full scores, envelope source, search count, final access evidence,
and any indeterminate predicate.
`tests/placement.rs` applies a congestion-reducing proposal and completes both
nets through the ordinary negotiated HyperPath router; independent access
fixtures move a legal board-edge-trapped terminal and prove composed
keepout/foreign-pad/escape-policy blocking. The score is still a
placement-stage probe rather than route or release proof, so applied results
must pass routing and the exact component/board readiness checks in
`HyperDrcHandoff`.

Land patterns retain source-addressable silk, fabrication, courtyard, mask,
paste, copper and custom line/circle/polygon/text artwork. The KiCad writer
emits these primitives on side-aware layers and reports any defaulted stroke
policy explicitly. `LandPatternBody` is an optional exact native review/DRC
fallback envelope with retained standoff and height. Independently,
`LandPattern::models` retains zero or more typed `Pcb3dModelReference` values,
so imported or authored package models neither require nor masquerade as body
envelopes. Each reference carries a resolver URI, explicit OBJ/STEP/VRML/glTF
container and exact scale/rotation/offset. KiCad board and standalone footprint
import/export preserve every model URI and transform. Pad shape dimensions are
centered on the retained pad origin; exact pad-local rotation is applied before
footprint side/rotation/translation to copper, process openings, routed slots,
placement envelopes, DRC and review output. KiCad's centered horizontal and
vertical oval drills map exactly to retained centerline slots in both board and
standalone-footprint import, and board export reconstructs the native oval
dimensions. Non-centered or diagonal slots remain valid HyperCircuit intent;
KiCad export projects only those to a round cutter-diameter drill and returns a
typed `RoutedSlotProjectedToRound` omission.

Copper zones retain exact foreign-net clearance, stable priority, solid or
angled hatch fill, and solid, isolated, or parameterized thermal-relief
connections. Materialization clips pours to the substrate, subtracts scoped
keepouts and expanded foreign-net copper, resolves zones in priority order, and
constructs same-net thermal spokes through csgrs profile booleans. Retained
island policy can keep every component, remove every unconnected component, or
remove only unconnected components below an exact filled-area threshold.
Materialization certifies contour/hole ownership and records initial, retained,
unconnected-pruned, and area-pruned island counts. Native SVG exposes the
policy; KiCad receives its native island mode/area fields plus mapped
fill/clearance/thermal fields, while LCEDA carries audited extension fields
pending native-editor conformance.

Zones may also retain a bounded plated stitching-via grid with exact pitch,
edge clearance, layer span, land/drill dimensions and surface-mask intent.
Deterministic row-major realization uses exact zone/keepout predicates plus the
canonical mixed-curve board inset, rejects board/cutout/via-keepout and
via-collision candidates, and
feeds the same derived vias to hyperpath routing, SVG, KiCad, LCEDA, csgrs
materialization, HyperDRC and fabrication. KiCad receives the realized vias and
an explicit policy-lowered omission because it cannot reconstruct the
declarative generator. Candidate/rejection/truncation evidence is retained in
routing and fabrication reports. Thermal spokes currently terminate against
the exact-aware land-clearance bounding box when curved profile intersection
is uncertifiable; each such projection is counted in fabrication evidence.

Geometry materialization always retains individual source/net-tagged copper
features. A per-layer boolean image is present only when csgrs certifies the
union; otherwise `LayerImage::blocker` records the exact uncertainty, so an
unresolved topology decision never silently becomes fabrication geometry.
Each feature also carries a `MaterializedCopperIdentity` naming its semantic
pad/instance/pin, route, via, zone, or placed-artwork owner. DRC and
manufacturing adapters therefore do not recover circuit identity by parsing a
diagnostic string or inspecting csgrs geometry.
`HyperDrcHandoff` returns typed board, stackup, net-class, differential-pair,
authored keepout, exact routed-slot, side-aware placed courtyard/body, certified
surface-copper, and mask/paste/legend inputs plus explicit geometry omissions.
For impedance-controlled single-ended classes and differential pairs,
`HyperDrcHandoff::from_materialization_with_materials` resolves each retained
dielectric material handle through `PcbMaterialPropertyLibrary`, whose values
are source-attributed `hyperphysics::MaterialPropertyGraph`s. Assertions use
the dimensionless custom keys `PCB_RELATIVE_PERMITTIVITY_PROPERTY` and
`PCB_LOSS_TANGENT_PROPERTY` with unit `1`. Only exact, physically valid,
consistent values are lowered into HyperDRC's current board-wide laminate
model, while `DrcDielectricMaterialEvidence` retains each successful layer,
exact value and contributing `SourceSpec`. Missing handles,
unknown/interval/proposal/conflicting values, invalid units or signs, and
heterogeneous layer values remain typed `DrcHandoffOmission`s.
`ReleasePreparationOptions::pcb_materials` carries the same library into the
cohesive release path, so incomplete controlled-impedance material evidence is
a release blocker instead of an implicit FR-4 default.
`run_readiness` invokes HyperDRC's stackup, net constraint, native-authoring,
component-envelope, mask opening/spacing/expansion, paste overhang, and legend
width/overlap/board-edge checks directly. Net classes own single-ended
target/tolerance intent; `DifferentialPair` owns differential target/tolerance
intent independently of those member classes. Supported single-ended and
equal-width edge-coupled outer-microstrip and centered-stripline routes are
screened by HyperDRC's audited analytical models against those retained values.
HyperDRC assigns target metadata, unsupported model applicability and
out-of-tolerance results the stable `net-impedance-target-readiness` identity. By default
`ImpedanceTargetPolicy::ReleaseBlocking` promotes those analytical findings to
errors; `Advisory` is an explicit preliminary-design opt-out.
`HyperDrcReadinessReport` retains the policy and promotion count. Unequal-width,
asymmetric or coplanar differential geometry, mask/conductor thickness,
frequency-dependent loss/roughness and general heterogeneous field solving
remain explicit future boundaries. Error-severity findings block
`is_release_clean`. HyperDRC profile offsets are fallible at this boundary:
exact topology indeterminacy becomes an error-severity
`geometry-uncertainty` finding that names the requested check and source
layers. Release preparation therefore remains inspectable instead of unwinding
or substituting approximate geometry.
`CheckedDesign::prepare_release` carries resolved placement through this
handoff, fabrication generation, and both CAM and assembly re-import audits.
Its report's `is_release_clean` requires clean ERC, lossless DRC/fabrication
handoffs, error-free HyperDRC, byte-integrity, CAM, connectivity, and assembly
evidence.

`PcbMaterializationReport::stackup_3d` extrudes decided copper images and
dielectric/mask/custom board layers into independently inspectable exact-Z
`csgrs` meshes with layer metadata, alongside placed package-body envelope
solids. Round drills and exact centerline slots are subtracted from every
decided physical layer; certified front/back solder-mask opening unions are
subtracted from the corresponding ordered mask layer. Each successful
subtraction has typed layer/source/plating/process evidence, while invalid
tools, blocked images and exact-boolean failures remain explicit omissions.
`Pcb3dAssemblyReport::to_gltf` then emits one self-contained glTF 2.0 scene
whose independently named nodes preserve stable layer or circuit-instance
identity in an adjacent audit report. The glTF binary32 coordinate boundary is
explicit; exact source solids remain authoritative.
`stackup_3d_with_model_resolver` lets filesystem, package-store, embedded-asset,
or authenticated registry policy supply exact source bytes without putting I/O
policy in the semantic layer. Every declared Wavefront OBJ model, VRML/WRL
indexed-face scene, or self-contained glTF/GLB triangle scene is parsed through
csgrs, transformed independently into front/back board placement, SHA-256
audited, and named by logical instance plus stable model index. VRML import
flattens transform/group/shape graphs with `DEF`/`USE`, triangulates exact
decimal polygons, and counts omitted line/point or degenerate geometry. glTF
import flattens the selected scene's node hierarchy and retains source scene,
mesh-node, primitive and triangle counts; external buffer URIs, skins, morph
targets and non-triangle topologies fail explicitly. One or more resolved
models replace the optional fallback envelope in the scene; model loading works
even when no envelope exists. STEP references remain typed unsupported-format
omissions until a real B-rep loader exists.

[`examples/review_bundle.rs`](examples/review_bundle.rs) demonstrates the
shared-source review path end to end:

```bash
cargo run --features geometry --example review_bundle -- /tmp/hypercircuit-review
```

It writes `schematic.svg`, `pcb.svg`, `board.gltf`, and a typed
`review-manifest.json` from one checked declarative circuit/layout, including
simulation-stamp, scene-object, drill/mask-subtraction, finite-coordinate and
omission evidence.

`PcbLayout::export_kicad` produces a standalone review board and returns every
exact-to-decimal projection plus unsupported-detail omissions. With `geometry`,
`FabricationPackage::from_materialization` requires decided layer unions before
emitting X2-attributed copper Gerbers and plating-separated Excellon round
drills/routed slots. The same source-addressable materialization now derives
side-aware solder-mask openings and SMD paste apertures from exact per-pad
margins, lowers per-surface via `Tented`/`Open { margin }` intent and
filled/stroked mask/paste/legend artwork, and emits X2
`Soldermask`, `Paste`, `Legend`, and authoritative `Profile,NP` files. Back-side
placement mirrors geometry and swaps process sides. Production packages declare
the retained source unit, normalize CAM output to millimetres, and preserve
unspecified via-mask intent plus unresolved custom-layer, package-edge-cut, and
silkscreen-clipping policy as manifest omissions rather than silently choosing
fabrication semantics.
Exact cubic board profiles likewise require
`FabricationExportOptions::with_cubic_contour_chord_error`; no default
tessellation is inferred. The bounded projection is used only for the profile
Gerber, leaving HyperCircuit's exact `BoardContour` authoritative, and every
projected exterior/cutout segment is retained in the fabrication manifest.
Production text is lowered only when the caller supplies a
`ProductionTextPolicy` containing the exact OpenType/TrueType bytes and a stable
font name. The selected font's SHA-256 is retained in materialization and the
fabrication manifest; absent or rejected fonts remain explicit per-source
omissions.
Every package also contains a versioned deterministic JSON manifest with file
roles, byte counts, copper/process/drill source-feature accounting, source/output
units, production omissions and SHA-256 digests;
`verify_integrity` replays that evidence against the retained bytes.
With `drc`, `audit_cam_round_trip` additionally re-imports each Gerber through
csgrs geometry parsing and HyperDRC X2 metadata parsing, and each Excellon
program through HyperDRC's unit, fixed-coordinate, plating, tool, round-hit and
routed-slot parser. Connected pads and vias additionally produce an IPC-D-356
electrical-test sidecar from their typed semantic owners. HyperDRC re-imports
that sidecar and reconciles net, reference, pin, exact projected coordinate and
feature class against the versioned manifest ledger. The typed report blocks
release on archive-integrity, required CAM setup, nonempty geometry,
route-structure, drill-count or electrical-test mismatches while retaining
parser warnings and explicitly omitted netless pads as review evidence.
The manifest also records each zone's realized fill/connection class and counts
of cleared foreign features, treated same-net lands, and applied keepouts.
`PcbLayout::to_svg` provides a native all-layer or per-copper-layer review view
with source/net data attributes, transformed placed pad and drill/slot shapes,
and an exact-to-finite projection audit.

The checked curved-profile release demonstration is runnable with:

```bash
cargo run --features drc --example curved_fabrication
```

`KiCadImportReport::from_str` and `from_path` reconstruct editable circuit nets,
generic footprint interfaces, pad mappings, placements, exact route segments,
including certified circular arcs, vias, zones and mixed line/arc board
contours from the supported PCB subset. Common centered, axis-aligned KiCad
oval drills round-trip as exact routed-slot centerlines; shapes outside that
native representation are never dropped silently and receive typed projection
evidence. The board writer now emits KiCad's
physical `setup/stackup` records for every retained copper, dielectric and
surface-mask layer, preserving exact thickness and material through semantic
re-import. `KiCadExportReport::project` optionally carries a matching
`.kicad_pro` with minimal native net-class objects, exact-name
`netclass_assignments`, preferred track width, clearance, and preferred via
land/drill sizes. `preferred_trace_width` is intentionally distinct from
`min_trace_width`: KiCad native class widths guide interactive routing, while
`KiCadExportReport::design_rules` carries a matching `.kicad_dru` with direct
net-name conditions that enforce minimum track width/clearance and maximum
routed length/via count. Projection records audit both mappings.
`from_str_with_companions` and `from_project_paths` re-import all three files;
the older board-only and board-plus-rule entry points remain available.
Missing stackup data still uses the caller's explicit conductor-thickness
fallback; missing companions, wildcard/composite project assignments, named
via-style identity, signal-integrity intent, and regional policy remain typed
omissions rather than defaults. This follows KiCad's current format boundary:
physical stackup is in the
[board file](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/), project
settings are versioned
[JSON settings](https://dev-docs.kicad.org/en/components/settings/index.html),
and enforceable custom constraints use
[`.kicad_dru`](https://docs.kicad.org/master/en/pcbnew/pcbnew.html#custom_design_rules).
Cubic Bezier route and board-contour intent remains native in semantic JSON,
hyperpath, SVG and csgrs materialization; KiCad/LCEDA boundaries report typed
chord omissions and cubic fabrication profiles are refused until the caller
selects an explicit projection policy. Generated ids and unsupported
graphics/shapes are returned as `KiCadImportOmission` values. Regression
fixtures perform export/import/edit/re-export/re-import cycles without losing
the edited exact route width, physical stackup, or supported custom-rule
policy. Flat native schematic files independently round-trip circuit-bound
generic symbols, ports, labels and segmented wires through
`KiCadSchematicImportReport`; symbol coordinate edits are reconstructed without
changing circuit topology. See `examples/kicad_stackup_rules.rs` and
`examples/kicad_schematic.rs`.

```text
cargo run --features layout --example kicad_stackup_rules -- \
  review.kicad_pcb
```

The `release_workflow` integration test exercises one retained two-layer board
across exact simulation/ERC, schematic and PCB SVG, hyperpath routing handoff,
csgrs materialization and 3D stackup review, native HyperDRC, fabrication and
assembly outputs, semantic JSON, and KiCad re-import.
The `workflow` integration test begins at the concise fluent `Design` and
`DesignModule` APIs and proves both `CheckedDesign::prepare_release` and
`CheckedProject::prepare_release` through exact simulation, resolved placement,
path-qualified source identity, csgrs/HyperDRC, manufacturing package re-import,
and assembly reconciliation. It also proves release-rule failures remain typed,
inspectable evidence rather than construction errors.

`SchematicLayout` keeps symbol units, circuit ports, net labels and typed wire
endpoints bound to the authoritative circuit model. Explicit sheet trees own
page-local content membership; typed sheet boundary ports and direct
parent/child links retain the authoritative crossing net and reject cycles,
cross-page wires, or link-net disagreement. `to_svg_book` emits one standalone
review page per sheet with an audit record for every exact-to-finite coordinate
projection; flat layouts remain a backward-compatible implicit one-page book.
See `examples/hierarchical_schematic.rs` for a minimal two-page boundary-link
workflow.

With `interchange`, `SemanticDocument` provides a versioned JSON boundary that
round-trips exact `Real` values across circuit, schematic and PCB intent.
Revisions 8 through 25 migrate to v26 with an explicit
`SemanticMigrationReport`; other schema families or revisions, invalid
circuits, and drawing/layout references that disagree with the circuit graph
are rejected. `DesignEditBatch` adds atomic, optimistic-concurrency editor
deltas over stable net, bus, bus-slice, circuit-port, device-model,
circuit-instance, rail, module-parameter, source-stimulus, child-circuit,
schematic symbol/port/wire/label/sheet/boundary/link, land-pattern, placement,
keepout, constraint, route, via and zone identities. They replace ordered bus membership,
slice definitions, circuit-port and rail definitions, module parameter targets,
source waveforms, child bindings/overrides, simulation policy, the ordered
manual-stamp vector, complete schematic symbol/port/wire/label/sheet/boundary/
link definitions, pin bindings, model/footprint assignments and copper-net
assignments, board outline/stackup/rules, land-pattern definitions and keepout
boundaries/scopes, and complete model/instance/constraint/route/via/zone
definitions alongside exact line/arc/Bezier route geometry, and insert or
remove complete semantic objects. The optional schematic and PCB containers are
themselves reversible, so a circuit-only document can acquire a complete flat or
hierarchical drawing and a complete retained board in transactions that undo
exactly back to no drawing or board. Mixed batches validate circuit, schematic
and PCB references together.
Structural edits have presence-aware merge addresses: adding or deleting an
object conflicts with every stale field edit to that identity but commutes with
edits to other objects. A batch is replayed on a clone, checked against the
expected `DesignRevision`, ordinary
semantic validation and placement-constraint resolution, and committed only if
every edit succeeds. `DesignHistory` schema v14 generates position-preserving
inverse batches and persists undo/redo stacks while every commit, undo, and redo
advances the monotonic revision.
Its contiguous replay log permits field-aware concurrent commits: disjoint
writes rebase onto the current revision, including route width versus
centerline edits on one route, while overlapping writes return a typed
`MergeConflict`. Explicit log compaction bounds retained evidence and refuses
older unprovable rebases instead of guessing.
Stale or invalid batches leave the document unchanged.
`examples/semantic_edit.rs` imports a KiCad board, commits an exact route-width
edit, persists it after undo, restores and redoes it, then re-exports it.
Both this workflow and `examples/kicad_roundtrip.rs` automatically read and
write same-stem `.kicad_pro` and `.kicad_dru` companions when present.

```text
cargo run --features interchange --example semantic_edit -- \
  input.kicad_pcb output.kicad_pcb route-1 3 2
```

## Installation

```toml
[dependencies]
hypercircuit = "0.3.0"
```

Use a sibling checkout during Hyper-stack development:

```toml
[dependencies]
hypercircuit = { path = "../hypercircuit" }
```

## Quick start

Build a one-node conductance problem and replay an exact candidate:

```rust
use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitResult, ComponentId, LinearStamp,
    Net, NetId, Real, TransientPolicy,
};

fn main() -> CircuitResult<()> {
    let out = NetId::new("out")?;
    let circuit = Circuit::new(
        CircuitId::new("conductance")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: out.clone(),
        is_ground: false,
    })
    .with_stamp(LinearStamp::Conductance {
        component: ComponentId::new("g1")?,
        part: None,
        pos: Some(out),
        neg: None,
        conductance: Real::from(2),
    });

    let system = circuit.linear_mna_system()?;
    let replay = system.replay_candidate(&[Real::zero()])?;
    assert!(replay.accepted);
    Ok(())
}
```

## Core API

- `Circuit`, `Net`, `CircuitInstance`, `DeviceModel`, and the ID types preserve
  topology, model parameters, and stable identity before solver lowering.
- `LinearStamp`, `MnaProblem`, `MnaUnknown`, and `LinearMnaSystem` build exact
  dense MNA equations. `LinearMnaSystem::replay_candidate` computes `A*x - b`
  with exact values and returns a `ResidualReplayReport`.
  `LinearMnaSystem::solve_exact` performs certified-pivot Gauss-Jordan
  elimination and refuses singular or indeterminate pivots before replaying its
  own candidate.
- `Circuit::linear_mna_from_devices` lowers retained resistor, independent
  source, controlled-source, and DC capacitor/inductor behavior into executable
  stamps, with instance parameters overriding model defaults.
- `SourceStimulus`, `SourceWaveform`, and `SourceWaveformPoint` retain exact
  constant, ideal-step, piecewise-linear, indefinitely periodic pulse, damped
  sine, and dual-edge exponential voltage/current-source intent by stable
  component identity. Pulse timing, sine frequency/damping/phase, and
  exponential delays/time constants remain exact.
  `Circuit::lower_linear_devices_at` evaluates a source at an exact time,
  records its value in the lowering report, and lets a waveform replace an
  otherwise absent scalar source parameter.
- `Circuit::transient_step_at` lowers exact capacitor and inductor companions
  plus endpoint-evaluated sources for trapezoidal or first-order Gear/BDF
  policy, solves and replays the step, and returns component-addressed
  voltage/current history for the next step. `transient_step` remains the
  time-zero convenience entry point.
- `Circuit::transient_run` advances that replayed kernel to an exact stop time
  under caller-authored bounds. Fixed runs retain every endpoint directly.
  Proposed steps are truncated at the next retained step, PWL knot, or
  repeating pulse-phase boundary, sine onset, or exponential rise/fall delay,
  so a coarse caller grid cannot silently skip authored source events.
  Adaptive runs compare one full step with two half steps, accept only the
  twice-replayed refined state when every MNA unknown and reactive voltage/
  current meets the exact absolute/relative tolerance, and retain every
  accepted or rejected attempt with its maximum exact error ratio.
  `TransientRunReport` exposes stable unknown, reactive-state, and evaluated
  source waveforms plus explicit accepted-step, rejected-step, and
  minimum-timestep terminal states; a bounded partial series is never
  mislabeled complete. See `examples/transient_run.rs` for an exact PWL-driven
  adaptive capacitor waveform and `examples/analytic_sources.rs` for an exact
  delayed sinusoid lowered through a voltage source and load.
- `NonlinearDeviceReport`, `PiecewiseLinearSegment`, `EventPolicy`, and
  `SwitchState` record device domains and event decisions without hiding them in
  a numerical tolerance.
- `solve_piecewise_linear` deterministically enumerates a caller-bounded set of
  exact active regions, skips singular/inconsistent candidates, and returns only
  a solution whose device voltages are certified inside every selected region.
- `MosfetPolarity` and `DeviceModelKind::Mosfet` retain explicit drain, gate and
  source roles. `SquareLawMosfet`, `MosfetRegion`, and `MosfetNewtonPolicy`
  execute the body-tied-source level-one law for N- or P-channel DC circuits.
  `Circuit::solve_mosfet_dc` retains every exact active-region derivative,
  affine Newton stamp, candidate and exact true-law KCL/branch replay; it never
  labels a tolerance-only linear proposal as proof. See
  `examples/mosfet_dc.rs` for a declarative common-source stage whose exact
  saturation solution is replayed with zero residual.
- `Bus`, `BusSlice`, `CircuitPort`, `DevicePin`, and `PinElectricalKind` retain
  reusable interfaces and pre-layout electrical classes. `RailIntent` adds an
  exact nominal voltage, maximum-current envelope and ground/power/reference
  role without encoding supply semantics in a net name.
- `Circuit::electrical_rule_check` detects actively driven net conflicts,
  undriven signal inputs, missing power sources, bound no-connect pins, and
  ground ports attached to signal nets. `ErcRuleDeck` assigns stable per-rule
  info, warning, error, or ignore policy for review and release gates.
- `CircuitLibrary` and `SubcircuitInstance` validate reusable definitions,
  reject recursive hierarchy, require declared child ports, and flatten to
  stable path-qualified component/model/net/stamp/source identities for
  simulation. `flatten_with_scopes` additionally retains deterministic local
  net and instance maps for downstream semantic composition.
- `CircuitModuleParameter`, `CircuitModuleParameterTarget`, and
  `CircuitModuleParameterOverride` declare exact, unit-checked reusable
  interfaces instead of mutating flattened strings. Defaults or parent
  overrides target retained instance/model parameters and can forward through
  nested subcircuits; elaboration applies them before namespacing and preserves
  value provenance. Parameterized definitions refuse manual stamps that could
  become stale and lower device models after substitution. See
  `examples/parameterized_module.rs`.
- `LayoutModule`, `PlacementGroup`, `LayoutModuleInstance`, and
  `LayoutAssembly` bind reusable PCB fragments to those elaborated circuit
  scopes. Exact module/group transforms compose front/back placement,
  line/arc/Bezier routes, vias, zones and keepouts; land patterns and rule
  identities are path-qualified, electrical nets are remapped from the circuit
  scope, and the result must pass ordinary `PcbLayout` validation. See
  `examples/layout_module.rs`.
- `CircuitPackageCatalog` resolves semver-compatible circuit/model/footprint/
  symbol libraries with deterministic highest-compatible backtracking.
  `CircuitPackageLock` records a versioned lock schema, exact sources and
  content digests, and can verify the complete transitive lock before loading.
  `PartLibraryArtifact` packages unified model/multipart-symbol/land-pattern
  definitions as canonical versioned JSON, while `CircuitPackageStore`
  publishes and loads immutable SHA-256-addressed artifacts by exact lock
  coordinate. `Design::export_part`/`import_part` bridge those artifacts to the
  retained IR without a second component representation.
- `SchematicLayout`, `SchematicSymbol`, and `SchematicWire` retain authored
  drawing placement without duplicating logical connectivity; typed endpoints
  are checked against instance pin and circuit-port net bindings.
- With `layout`, `PcbLayout`, `LandPattern`, `PcbRoute`, `PcbVia`, `NetClass`,
  `DifferentialPair`, and `DifferentialPairNeckdown` retain PCB intent and lower route predicates to
  `hyperpath` without surrendering circuit net identity.
  `PcbRouteSegment` orders straight, exact directed circular-arc, and exact
  cubic-Bezier carriers in one connected path. Curve parameters stay
  authoritative in hyperpath, semantic JSON and SVG. KiCad retains circular
  arcs and explicitly omits unsupported Beziers; polygonal manufacturing
  materialization records each caller-controlled arc/Bezier chord-error
  projection in the fabrication manifest.
  Layout validation requires every connected pin on a placed instance to map
  to at least one nonduplicated physical pad declared by its device model.
- With `geometry`, `MaterializedProcessFeature` and `ProcessLayerImage` retain
  exact source identity across mask, paste, and legend unioning just as
  `MaterializedCopperFeature` and `LayerImage` do for copper. Copper artwork has
  its own HyperDRC kind instead of being mislabeled as a pad or zone.
- `RoutingProblemReport::from_layout` derives exact placed-pad terminals,
  stable net/layer aliases, existing copper, complete net-class rules, and
  orthogonal keepouts for `hyperpath`. Circuit-owned
  `RouteConstraintRegion`, `EscapePolicy`, `LengthTuningPattern`, and
  `PhaseTuningGroup` values remain attached to the routing problem beside
  those geometry carriers.
  `RoutingSolution::from_hyperpath` maps accepted straight traces, circular
  arcs, cubic Beziers and drilled vias back to semantic PCB identities before
  replace-or-append editing.
  `RoutingProblemReport::export_tscircuit_simple_route_json` additionally emits
  the current [`SimpleRouteJson`](https://github.com/tscircuit/tscircuit-autorouter#input-format-simpleroutejson)
  solver protocol with stable logical-net and physical-layer names, placed-pad
  terminals, board bounds, and conservative pad/cutout/keepout/zone/
  existing-copper obstacles. Every exact number crossing that finite JSON
  boundary is audited. Non-rectangular geometry, multilayer terminal reduction,
  and global-width projection remain typed `TscircuitRoutingOmission` values.
  `TscircuitRoutingImportReport::from_str` validates returned trace/net/layer
  records, reconstructs every decimal exactly, requires caller-owned via land,
  drill, and plating policy absent from the protocol, and passes all returned
  wires/vias through HyperPath before producing a `RoutingSolution`. See
  [`examples/tscircuit_router_handoff.rs`](examples/tscircuit_router_handoff.rs):

  ```text
  cargo run --features layout --example tscircuit_router_handoff
  ```
- `PcbLayout::negotiated_autoroute` supplies a deterministic native routing
  proposal for selected nets. It builds an exact-coordinate planar grid,
  connects multi-pad nets with multilayer A*, and repeats complete rip-up/
  reroute passes using present and historical congestion costs in the style of
  [PathFinder](https://janders.eecg.utoronto.ca/1387/readings/pathfinder.pdf).
  `NegotiatedGridMode::Uniform` preserves a caller-pitched global lattice.
  `FeatureAligned` instead retains coarse global coverage while injecting exact
  terminal, pad, board/cutout, existing-copper, via, zone, keepout and regional
  rule coordinates plus bounded fine-pitch halos. Physical pitch-unit costs
  keep coarse and fine edges comparable, differential-pair coordinate closure
  preserves its exact constant translation, and `NegotiatedGridEvidence`
  records the final/injected coordinate inventory.
  `LocallyRefined` instead activates `grid_pitch` capacity only inside exact
  axis-aligned `NegotiatedGridRefinementRegion` values over a coarse global
  mesh. Traversal skips inactive coordinate intersections when following a
  coarse edge, while refined regions expose every exact local crossing. Each
  region must contain a coarse X/Y crossing and have pitch-aligned bounds, so
  disconnected or under-spaced meshes fail before search. Grid evidence
  separately retains active planar nodes and nodes added by local refinement;
  the executable dense-channel fixture uses 59 searchable nodes instead of the
  equivalent 169-node global fine mesh.
  `PcbLayout::adaptive_negotiated_autoroute` closes the caller-authored-region
  loop with bounded deterministic synthesis. Round zero uses a feature-aligned
  coarse mesh; each incomplete report contributes exact conflict geometry and
  failed-net terminal envelopes, which are padded, snapped to the global
  fine-pitch lattice, expanded to a coarse crossing, merged, and rerouted as
  sparse local capacity. `NegotiatedAdaptiveRoutePolicy` independently bounds
  refined reruns and merged regions. `NegotiatedAdaptiveRouteReport` retains
  every ordinary routing report plus the exact region set proposed after each
  round and distinguishes completion, refinement-limit, region-limit, and
  no-progress termination. The executable channel fixture proves a coarse
  failure, one synthesized region, deterministic refined completion, semantic
  application, validation, and geometry materialization.
  `NegotiatedPlanarTopology::Orthogonal` retains the four-neighbor default;
  `Octilinear` additionally admits exact equal-span 45-degree edges.
  `AnyAngle { maximum_neighbors_per_node }` constructs a deterministic bounded
  visibility graph from the nearest active planar nodes. Exact segment
  legality and fixed-feature clearance still gate every candidate, while
  `RouteDirection::Arbitrary` lets regional and terminal-escape intent admit
  only motions outside the axis/45-degree families and survives hierarchical
  quarter-turn/mirror transforms. After every pass, exact segment-to-segment
  distance checks compare emitted half-widths plus pairwise clearance for all
  unrelated same-layer nets. Crossing or too-close visibility edges therefore
  claim both underlying segment resources, feed present/history negotiation,
  and remain ordinary source-addressable conflict evidence. Differential pairs
  use a separately bounded translated visibility set and retain their existing
  joint certification.
  `RouteDirection::DiagonalRising` and `DiagonalFalling` make those motions
  explicit in regional and terminal-escape policy and transform correctly
  through mirrored hierarchical modules. Opposing diagonals claim one
  `DiagonalCell` resource, so a geometric crossing remains a typed negotiated
  conflict instead of becoming two unrelated edges. The minimum available
  diagonal track spacing is checked exactly against every selected width,
  clearance and via construction before search.
  Exact predicates enforce board, cutout and layer-scoped keepout clearance.
  Every placed copper pad—including electrically unmapped lands—is a fixed
  obstacle on its transformed physical layers. Circle, rectangle, rounded
  rectangle, obround and arbitrary polygon pads use their exact retained
  envelope, pad rotation, placement rotation and board-side mirroring instead
  of a conservative bounding disc. Retained straight routes, authored vias and
  zone-stitching vias are fixed copper obstacles. Accepted traces and
  adjacent-layer vias cross the `hyperpath` carrier before becoming
  `RoutingSolution`, and `NegotiatedRouteReport` retains expansion, conflict,
  failure and iteration-limit evidence. Every pass also retains exact
  `NegotiatedRouteIterationState`: stable logical-net path/node sequences,
  typed node/segment/diagonal-cell/via conflict geometry with all claiming
  nets, and typed failures. Re-running the same problem/policy produces
  identical state, so
  debuggers and viewers can replay the pipeline without accessing private grid
  indices or becoming a second layout IR.
  `NegotiatedRouteReport::iteration_svg` turns any retained pass into a
  standalone, layer-filterable SVG with exact-coordinate projection evidence,
  provisional net paths, layer transitions, typed conflict geometry, and
  failure metadata. `NegotiatedRouteReport::replay_html` packages every such
  pass into a self-contained interactive document with a pass slider,
  previous/next and keyboard navigation, and live edge/conflict/failure
  metrics; the HTML report aggregates the same projection audit rather than
  inventing a viewer-owned IR. `examples/advanced_routing.rs` writes
  `index.html` and every individual SVG pass to
  `target/advanced-routing-replay/`:

  ```text
  cargo run --features layout --example advanced_routing
  ```

  `apply_to` replaces only selected-net authored routes/vias. Retained
  two-terminal differential pairs with
  compatible translated endpoints are searched atomically: both traces take
  every bend and layer transition together, pair-member pads use the authored
  edge gap rather than generic foreign-net clearance, and
  `NegotiatedDifferentialPairEvidence` certifies exact center spacing, planar
  lengths, skew and synchronized-via count. Optional
  `DifferentialPairNeckdown` intent admits narrower traces and tighter
  axis-aligned terminal spacing, symmetrically fans both members out to nominal
  coupled spacing within the exact caller bound, collision-checks every fanout
  edge at its authored terminal width, emits width-split semantic routes,
  includes the edges in route resources and skew, and returns exact
  width/spacing/transition/use evidence.
  Feature-aligned grids inject the required fanout coordinates and close them
  under nominal pair translation. Uniform-grid terminals and fanout points
  must lie on the caller's lattice; feature-aligned mode admits exact
  off-lattice terminals. Fixed curved routes are refused explicitly.
  `RoutingSolution::quality_report` supplies deterministic benchmark evidence
  against the same retained problem: per-net and aggregate exact straight-line
  centerline length, obstacle-free Euclidean spanning-tree lower bound, excess
  length, stretch and via count. Diagonal lengths remain exact algebraic
  values; missing routes, curves and indeterminate comparisons remain typed
  issues rather than implicit numeric projections.
  `NegotiatedRouteReport` also retains the complete caller policy and
  `NegotiatedRouteWorkEvidence`: exact grid-state count, configured pass and
  per-connection expansion budgets, executed passes, total/peak expansions,
  expansion-limit failures and pass-budget exhaustion. These machine-neutral
  work units make corpus and CI comparisons reproducible without treating wall
  time as semantic evidence.
  `tests/routing_corpus.rs` retains an original, license-clean workload shaped
  after the public ts-circuit dataset categories: a 12-net single-layer bus,
  four nets crossing independently across two layers, and a three-net
  non-axis fanout. Every case exports through the current `SimpleRouteJson`
  adapter, routes twice with exact-equal retained reports, stays
  below a pinned machine-neutral expansion ceiling, produces complete exact
  quality evidence, applies, and revalidates. The corresponding
  `cargo bench --bench routing --all-features` executable reports elapsed time,
  grid states, passes, expansions, exact routed length, and stretch without
  imposing a machine-dependent time threshold. One July 2026 optimized run
  observed 639 ms/516 expansions for the 12-net bus, 130 ms/597 expansions for
  the four-net multilayer crossing case, and 81 ms/36 expansions for the
  three-net any-angle case; those elapsed values are illustrative, while the
  retained work and quality values are the reproducible evidence.
  Polygonal `RouteConstraintRegion` intent restricts selected nets to retained
  layer/direction/via sets whenever a candidate edge touches the region.
  `EscapePolicy` similarly constrains package-terminal fanout inside an exact
  Manhattan distance, and both produce per-policy accepted-edge/via evidence.
  `ViaStyle` retains named land, finished-drill, plating, surface-mask, and
  optional allowed-layer-span intent. Net classes select those constructions;
  negotiated search uses their physical radius and span legality, emits them
  through hyperpath, and returns per-net `NegotiatedViaStyleEvidence`. Legacy
  paired preferred land/drill values and router defaults remain explicit
  fallbacks.
  Net classes may inherit from one retained base class. Optional scalar/style
  fields inherit until overridden, reference-plane requirements accumulate,
  and assigned nets remain local to the derived class.
  `resolve_net_classes` rejects missing parents and cycles and returns
  `ResolvedNetClass` values with base-to-derived lineage; routing and HyperDRC
  consume those same resolved policies.
  `RouteRuleRegion` applies stricter exact width and clearance floors whenever
  a selected route edge touches its polygon. Overlapping regions combine by
  exact maximum; search uses the local envelope, output routes split at width
  boundaries, and `NegotiatedRouteRuleRegionEvidence` counts every governed
  planar edge and via.
  After stable route identities exist,
  `PcbLayout::realize_length_tuning` can replace one deterministic orthogonal
  span with a bounded exact serpentine inside an authored region. It proves
  original/realized/target length, cycle count and idempotent replay before the
  result proceeds to ordinary geometry and HyperDRC checks.
  `Design::length_tuning` and `Design::phase_tuning_group` expose the same
  retained intent through design-scoped typed handles. A phase group evaluates
  all member patterns against a private layout, proves final exact skew,
  checks edge clearance against foreign straight routed copper, retained and
  zone-stitching via lands, exact transformed placed-pad envelopes, copper-zone
  source boundaries and applicable copper keepouts,
  optionally certifies that a differential pair remains one constant
  translation at its required center spacing, and exposes route replacements
  only as one atomic set.
  With `geometry`, `PcbLayout::realize_phase_tuning_with_realized_zones`
  instead repours the candidate layout and checks the final source-addressable
  zone fill. `PhaseTuningZoneCollisionMode`,
  `PhaseTuningRealizedZoneEvidence`, and
  `PhaseTuningRealizedZoneStatus` distinguish authored-boundary checks from
  realized-fill proof, retain exact required clearance and island count, and
  reject any extra-clearance relationship that exact point probes cannot
  certify. Generated multi-segment orthogonal serpentines materialize as one
  retained route feature by exactly unioning their swept line segments.
  `PcbLayout::synthesize_phase_tuning` can instead derive stable member
  patterns, route selectors, exact target lengths and tight board-contained
  regions from retained orthogonal routes. It deterministically enumerates
  both excursion sides under a caller-owned assembly bound and returns intent
  only after the same atomic skew, differential-coupling, board, copper and
  keepout certification succeeds. The report can add the retained intent or
  apply its already-certified route set without partial mutation. See
  `examples/advanced_routing.rs` and `examples/phase_tuning.rs`.
  Automated thousand-sample upstream dataset runs, incremental-route and
  memory measurements, standardized benchmark-machine history, and materially
  denser BGA workloads remain future performance work. The current
  [tscircuit autorouter](https://github.com/tscircuit/tscircuit-autorouter) and
  its archived
  [dataset/benchmark taxonomy](https://github.com/tscircuit/autorouting) are
  the comparison targets for those later stages.
- `KiCadImportReport` restores the supported KiCad PCB semantic subset into the
  same retained API, while `KiCadExportReport` audits finite projection,
  carries an optional custom-rule companion, and reports unsupported outbound
  intent. `KiCadDesignRuleProjection` records every retained net-class rule
  lowered to direct KiCad net-name conditions.
- `DesignEdit`, `DesignEditBatch`, `DesignRevision`, `EditTarget`, and
  `EditReplayReport` provide a serializable editor transport over retained
  semantic identities. Current operations replace net ground intent, ordered
  bus/slice, boundary-port, rail, module-parameter and child-circuit definitions,
  source waveforms, simulation policy, manual stamps, schematic object
  definitions, board outline/stackup/rules, land-pattern and keepout definitions,
  complete model/instance/constraint/route/via/zone definitions, instance pin
  bindings or model, placement transform or footprint, route
  width/geometry/net, via center/net, or zone boundary/net. They insert or remove complete nets, buses,
  slices, ports, rails, module parameters, stimuli, child circuits, device
  models, circuit instances, schematic layouts, symbols, port placements, wires,
  labels, sheets, sheet ports and sheet links, complete PCB layouts, land patterns, keepouts,
  placements, constraints, routes, vias, and zones atomically. They refuse stale
  revisions, duplicate or missing targets,
  invalid results, and placement-constraint conflicts without partial mutation. `DesignHistory`
  retains generated inverse batches and serializable undo/redo stacks without
  rolling revisions backward. `EditAddress` and `ConcurrentCommitReport`
  provide field-level optimistic rebasing with typed overlap conflicts and a
  compactable append-only replay log.
- `LcedaProExportReport` produces a deterministic EasyEDA/LCEDA Pro archive
  from the same circuit, schematic and PCB source, including real authored
  copper rather than generic draft placement, with a versioned unit/loss audit.
  Baseline-relative import APIs restore the supported PCB and schematic
  presentation edits while refusing changes to authoritative connectivity,
  libraries and topology.
- `AssemblyOutputs` derives deterministic grouped BOM and exact-coordinate
  pick-and-place CSV views without inventing identities from geometry.
  `AssemblyVariant` adds validated DNP lists, stable alternate-part handles and
  explicit DNP CSV while procurement facts remain owned by `hyperparts`.
  `audit_csv_round_trip` independently parses all three documents and exactly
  reconciles fitted/DNP references, part/model/land-pattern identities,
  coordinates, rotations, sides, variants, quantities and canonical grouping;
  unplaced fitted instances plus syntax, schema and semantic drift are retained
  as typed release findings.
- `PhysicalElectricalPort`, `ThermalPort`, `CoupledResidualBlock`, and
  `ElectrothermalRcReport` describe circuit/physics boundaries. Call
  `CoupledResidualBlock::to_hypersolve_problem` to hand residual rows to
  `hypersolve`.
- `CircuitAdapterReport` records the solver, tolerance policy, and exact-replay
  result of an external numerical adapter.

## Precision and performance

Primitive floats belong only in named import, diagnostic, or solver-adapter
boundaries. Exact MNA rows, unknown ordering, model parameters, domains, and
coupling residuals remain structured so they can be replayed without treating a
solver tolerance as proof. An unknown replay is reported as uncertainty rather
than silently accepted.

The dense implementation is intended for small fixtures and certification
paths. Stable IDs and the separation between stamp construction and replay let
future sparse, transient, and DAE adapters reuse semantic context without
rebuilding it. The crate does not eagerly expand nonlinear devices or transient
policies into a global expression tree.

Currently implemented: linear conductance, current-source, voltage-source,
VCCS, and transient-companion stamps; exact dense residual replay; replayed
trapezoidal/backward-Euler C/L steps; retained exact
constant/step/PWL/pulse/sine/exponential source stimuli and event-aligned
bounded fixed/adaptive time series;
bounded exact piecewise-linear
active-region solving; retained two-terminal Shockley diode models; bounded
Newton DC solves with exact true-law replay; fixed-step mixed
linear/reactive/diode time series; retained three-terminal N/P square-law
MOSFET models with exact cutoff/triode/saturation derivatives, bounded DC
Newton solving and exact true-law replay; certified diode/MOSFET DC
operating-point promotion and exact complex small-signal sweeps retaining every
incremental derivative; nonlinear and switch report carriers;
electrothermal RC reports; and `hypersolve` coupling handoff. Every diode
iteration retains its exact input voltage, imported dyadic conductance/current
intercept, exact linear proposal replay, voltage update and true-law residual.
See `examples/diode_transient.rs`, `examples/mosfet_dc.rs`, and
`examples/nonlinear_ac.rs`. Dynamic MOSFET capacitance, body effect,
subthreshold and reverse-channel behavior, sparse matrices, adaptive
nonlinear/multistep integration, and field solvers are not yet implemented
here.

Duplicate net-voltage and branch-current unknowns are rejected before assembly.
See [the reference and performance audit](PERFORMANCE.md) for the source-by-source
mapping, retained benchmark results, rejected trials, and validation evidence.

## References

- Ho, Ruehli, and Brennan, [*The Modified Nodal Approach to Network
  Analysis*](https://doi.org/10.1109/TCS.1975.1084079), 1975.
- Nagel, [*SPICE2: A Computer Program to Simulate Semiconductor
  Circuits*](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1975/9602.html),
  UCB/ERL M520, 1975.
- Yap, [*Towards Exact Geometric
  Computation*](https://doi.org/10.1016/0925-7721(95)00040-2), 1997.
- Cortes Garcia, De Gersem, and Schoeps, [*A Structural Analysis of
  Field/Circuit Coupled Problems Based on a Generalised Circuit
  Element*](https://doi.org/10.1007/s11075-019-00686-x), 2020.
- LLNL, [SUNDIALS IDA](https://computing.llnl.gov/projects/sundials/ida), the
  DAE/BDF adapter model referenced by `TransientPolicy::IdaDaeAdapter`.

Direct dependencies: [hyperreal](https://github.com/timschmidt/hyperreal) ·
[hypersolve](https://github.com/timschmidt/hypersolve). Related Hyper crates:
[hyperparts](https://github.com/timschmidt/hyperparts) ·
[hyperpath](https://github.com/timschmidt/hyperpath) ·
[hyperphysics](https://github.com/timschmidt/hyperphysics) ·
[hyperevolution](https://github.com/timschmidt/hyperevolution)

## Development

```sh
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cargo bench --bench mna
cargo bench --bench routing --all-features
```
