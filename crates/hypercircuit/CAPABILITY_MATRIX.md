# Declarative circuit and PCB capability matrix

This matrix is the acceptance ledger for practical equivalence with
[tscircuit](https://github.com/tscircuit/tscircuit) and
[via-rs](https://github.com/jz315/via-rs). It records user-visible workflows,
not type-count parity. `Implemented` means a tested retained representation or
lowering exists; `partial` names the remaining workflow gap.

## Ownership boundaries

| Concern | Authoritative crate | Boundary |
| --- | --- | --- |
| Circuit hierarchy, instances, typed pins, nets, buses, ports, simulation intent | `hypercircuit` | Retained semantic source of truth |
| Footprints, pin-to-pad maps, placement, stackup, net classes, PCB feature identity | `hypercircuit` | Retained layout source of truth |
| Trace/via path primitives and route certification | `hyperpath` | Receives stable net aliases and exact route carriers |
| Net scheduling, placement and negotiated PCB route search | `hypercircuit` | Owns electrical intent, fixed-feature policy and audited proposal/application |
| Profile/solid construction, booleans, offsets and tessellation | `csgrs` | Materialization engine; no circuit ownership |
| Constraint decks, geometric DRC and release-readiness evidence | `hyperdrc` | Consumes source-addressable features and authored rules |
| Parts and material libraries | `hyperparts`, `hyperphysics` | Referenced by stable handles; not copied into geometry |

This is deliberately closer to tscircuit's separation of core IR, footprint,
routing, viewing and export packages than to a monolithic CAD object. It also
preserves via-rs's Rust-native typed design and validation model while keeping
autorouting and fabrication sign-off in specialist crates.

## Capability matrix

| Workflow | tscircuit | via-rs 0.1.1 | hypercircuit status | Acceptance gap |
| --- | --- | --- | --- | --- |
| Stable circuit/component/net identities | Yes | Yes | Implemented | — |
| Fluent declarative authoring | Yes | Yes | Implemented | `Design` owns typed design-scoped part-definition/net/instance/pin and route/via/zone/keepout handles. `define_part` lowers one reusable electrical interface, multipart exact symbol library, footprint and pin map exactly once; `instantiate` adds only reference, value/waveform, selected symbol-unit transforms and PCB placement, so many instances share one retained `DeviceModel`, `SchematicSymbolDefinition` and `LandPattern`. One-off fluent parts, exact linear/nonlinear primitive models, schematic wires, rails, exact mixed-segment routes, layer transitions, rich copper pours and scoped keepouts also lower atomically into the authoritative `Circuit`/`SchematicLayout`/`PcbLayout`, retain advanced escape hatches, and finish only after all three validators pass |
| Typed device pins and instance bindings | Yes | Yes | Implemented | — |
| Ground, rails, named signals | Yes | Yes | Implemented | — |
| Buses, named slices and boundary ports | Yes | Limited | Implemented | `Design::bus`, `Design::bus_slice`, and `Design::port` return design-scoped typed handles, preserve/reverse exact authored member order, reject empty/duplicate/out-of-range/cross-design requests atomically, and lower directly into retained `Bus`, `BusSlice`, and `CircuitPort` records |
| Reusable hierarchical modules | Yes | Yes | Implemented | `DesignModule::instantiate` checks design-scoped child-port/parent-net bindings, required interfaces and exact parameter overrides atomically; arbitrary nesting compiles directly through `CircuitLibrary` and `LayoutAssembly` into a source-mapped `CheckedProject`. Stable local-to-flat scope maps compose exact nested front/back transforms, native line/arc/Bezier routes, vias, zones, keepouts and net classes into validated path-qualified flat boards. `LayoutModule` retains child-local placement constraints, resolves them after local placement-group transforms and before parent transforms, and rejects unsatisfied predicates before partial composition |
| Structural connectivity validation | Yes | Yes | Implemented | Automatic Rust `file:line:column` traces cover fluent board/net/part/pin/symbol/wire/footprint/placement/connect declarations and advanced circuit/schematic/layout mutation boundaries; `DesignDiagnostic` correlates every authoritative circuit/schematic/layout finding to the most specific known call sites, and the independently serializable `DesignSourceMap` survives checked authoring without polluting semantic IR |
| Electrical-rule checking | Yes | Yes | Implemented | — |
| Exact linear MNA solve and replay | Limited | Out of scope | Implemented | — |
| Primitive-device simulation | Limited | Out of scope | Partial | R/V/I/VCCS, DC and exact complex small-signal AC R/L/C, retained exact constant/step/PWL, indefinitely periodic pulse, damped sine and dual-edge exponential independent-source stimuli, endpoint-aware lowering, exact source-event step truncation, replayed trapezoidal/backward-Euler C/L steps, bounded fixed/adaptive step-doubling linear runs, bounded exact piecewise-linear active-region solving, retained Shockley diodes and retained three-terminal N/P MOSFETs now exist. Component-addressed AC excitations drive ordered exact angular-frequency sweeps; rectangular phasor MNA, shared unknown identities, exact squared magnitude and real/imaginary residual replay are retained, with zero small-signal amplitude for unexcited independent sources. A checked fluent RC fixture proves the same declarative connectivity through the sweep, while an inductor/VCCS fixture covers reactive and controlled-source stamps. Fluent source parts accept retained waveforms directly or through typed `Design::stimulus`; a checked current-source/capacitor fixture executes its authored PWL breakpoints through the exact transient runner. Fluent `parts::diode`, `parts::mosfet`, `parts::nmos`, and `parts::pmos` declarations retain exact model parameters and typed A/K or D/G/S roles; checked authoring fixtures extract and execute both nonlinear laws. Diode DC and fixed-step mixed R/C/L/diode transient runs retain every lossy Newton linearization as imported exact coefficients, solve each linear proposal exactly, and accept endpoints only after true exponential-law residual replay satisfies authored exact voltage/current tolerances. The body-tied-source Shichman-Hodges MOSFET DC path retains explicit drain/gate/source roles, exact cutoff/triode/saturation derivatives, affine Newton proposals and exact true-law KCL/branch replay. Accepted diode/MOSFET Newton reports promote to opaque `AcOperatingPoint` values; exact small-signal sweeps retain DC solver/replay provenance, the diode's imported incremental conductance/intercept boundary or exact MOSFET region/`gm`/`gds`, and replay every complex residual. A checked declarative common-source example proves exact −1 gain at multiple frequencies. Arbitrary expression stimuli, dynamic MOSFET capacitance, body effect, subthreshold and reverse-channel behavior, adaptive nonlinear stepping, higher-order BDF and sparse large-system integration remain |
| Reusable land patterns and pads | Yes | Yes | Implemented | — |
| Pin-to-pad maps | Yes | Yes | Implemented | — |
| Component placement | Yes | Draft export | Partial | Typed `Design::constrain`/`PlacementRule` declarations atomically retain fixed/relative/alignment/region/rotation/side constraints over design-scoped instance handles. Reusable exact placement-group/module transforms, exterior/cutout-aware bounded search with conservative courtyard/body/pad clearance, and release-blocking HyperDRC certification exist. Candidate evidence retains opt-in exact cardinal pin-access cost before distinct-net routing-box overlap, caller-radius same-side density, connectivity, displacement and orientation. Every mapped terminal is probed on every physical copper layer at a caller-authored exact distance/minimum width/clearance; board/cutout and copper-keepout envelopes, fixed straight routes and vias, transformed foreign pads, net/regional rules, route-constraint regions and terminal escape policy share negotiated-router predicates. Reports retain instance/pin/pad/net/center plus layer/direction accessible/blocked/indeterminate evidence and make incomplete final audits typed solve failures. Regressions move a legal board-edge-trapped package from two blocked probes to none, prove keepout/foreign-pad/escape-policy composition, and separately complete congestion-improved nets through the negotiated HyperPath router. Multi-pass/global optimization, thermal objectives and large-board placement benchmarks remain |
| Board outline and cutouts | Yes | Draft export | Implemented | Exact retained `BoardContour` line/circular-arc/cubic-Bezier segments round-trip through semantic JSON and SVG and lower once into the canonical `BoardBoundaryGeometry` exterior-minus-cutouts carrier. The same exact Hypercurve region drives csgrs materialization, mixed-curve placement containment and HyperDRC; exact primitive-distance predicates drive line/arc routing and stitching clearance, with cached region insets available where certified. General cubic parallel offsets remain explicitly indeterminate instead of silently approximated. Circular edges export/re-import through KiCad and emit Gerber G02/G03 profile arcs. Cubic CAM defaults to refusal but an explicit finite positive chord-error policy adaptively emits exterior/cutout profile segments and records contour identity, segment index, requested bound and generated count in the fabrication manifest; the checked-release fixture independently re-imports the projected Gerber. LCEDA/KiCad cubic substitutions remain typed adapter omissions |
| Physical stackup/material handles | Partial | Rules | Partial | Exact ordered thickness/material handles now resolve through source-attributed HyperPhysics property graphs using HyperCircuit-owned relative-permittivity/loss-tangent vocabulary. Exact positive/nonnegative, dimensionless and board-wide-consistent values feed HyperDRC's audited single-ended and equal-width edge-coupled outer-microstrip/centered-stripline target screens; missing, uncertain, conflicting, invalid and heterogeneous properties are typed release omissions. Rust callers supply the property library directly, while exact decimal/rational `[[pcb-materials]]` declarations with authority/locator/freshness/condition provenance make the same evidence available to manifest-driven `check` and `release`. Per-layer anisotropy, frequency/temperature dependence, conductor thickness/roughness/loss, unequal-width/asymmetric/coplanar coupling and general field solving remain |
| Net classes and differential-pair intent | Yes | Electrical classes | Implemented | Typed `ViaStyleRule`, `NetClassRule`, `DifferentialPairRule`, `LengthTuningRule`, and `PhaseTuningGroupRule` declarations bind design-scoped nets, routes, inheritance parents, preferred via constructions, exact width/clearance/length/via-count/single-ended-impedance/reference-plane policy, pair-owned differential target/tolerance, nominal width/spacing, reduced terminal width/spacing, bounded neck-down transitions, skew and atomic phase groups; returned handles prevent cross-design rule references |
| Traces and vias with retained net identity | Yes | Out of scope | Partial | Ordered exact line/circular-arc/cubic-Bezier segments lower through hyperpath and semantic JSON; SVG preserves all three, KiCad round-trips arcs and explicitly omits Beziers, and CAM records audited curve projections; broader curve and editor interchange remain |
| Copper zones and keepouts | Yes | Out of scope | Implemented | Exact clearance, priority, solid/hatch fill, solid/isolated/thermal land connections, scoped keepout subtraction, exact-area/unconnected island pruning and bounded exact-predicate stitching-via grids lower through routing/DRC/CAM with manifest evidence |
| Routing and router handoff | Capacity autorouter plus solver protocol | Out of scope | Partial | Deterministic multilayer negotiated-congestion A* routes multi-terminal nets, rips up selected copper, avoids exact board/cutout/scoped-keepout boundaries plus every placed copper pad, retained straight routes and vias, emits adjacent-layer vias through hyperpath, and returns per-pass evidence before selective application. Connected pads retain net/class policy; electrically unmapped pads remain physical obstacles. Circle, rectangle, rounded-rectangle, obround and polygon pads use exact placement-side/pad/placement transforms and exact point/capsule clearance rather than conservative discs; executable channel and rotation fixtures cover every non-circular family. Every pass retains deterministic visualization/replay state: exact logical-net path nodes, typed node/segment/diagonal-cell/via conflicts with stable claiming-net sets, and typed failures; an executable rerun regression proves state equality without exposing private grid indices or creating a viewer-owned IR. Callers select a uniform exact lattice or a feature-aligned mode that combines coarse global coverage with exact terminal/pad/boundary/existing-copper/via/zone/keepout/rule coordinates and bounded fine-pitch halos; physical pitch-unit costs keep irregular edges comparable, coordinate closure preserves differential-pair translation, and final/injected coordinate counts remain audited. `NegotiatedPlanarTopology` selects the orthogonal default or exact equal-span 45-degree edges; rising/falling diagonal directions survive hierarchical transforms, opposing diagonals claim one crossing-cell resource, and exact minimum diagonal track spacing is checked against selected copper policy before search. An executable regression proves that exact off-lattice terminals refused by uniform mode route under feature alignment. The current ts-circuit `SimpleRouteJson` protocol now has an audited bidirectional adapter: exact circuit-net/physical-layer aliases and placed terminals plus conservative board/pad/cutout/keepout/zone/existing-copper obstacles export to finite JSON; returned variable-width wires and layer-changing vias are parsed as exact decimals, supplied explicit caller via-process policy, certified through HyperPath, and restored as semantic features. Typed omissions cover protocol-wide global width, multilayer-terminal and rectangular-obstacle reductions. Exact polygon-touch constraint regions enforce layer/direction/via sets, including retained rising/falling diagonal direction intent; regional rule polygons raise width/clearance by exact maximum, affect search envelopes, split emitted widths and retain use evidence. Terminal escape policies enforce layer/direction/via sets inside retained Manhattan radii. Net-class-selected named via styles govern land/drill/plating/mask/span legality and retain per-net use evidence through materialization. Bounded exact post-route serpentine proposals meet retained total-length targets inside authored regions and replay idempotently. Constant-spacing two-terminal differential pairs route atomically with translated bends, synchronized vias and exact length/skew evidence. Optional retained neck-down intent accepts reduced trace width and axis-aligned terminal spacing, symmetrically fans both lanes to nominal spacing under an exact transition bound, collision-checks each edge at its emitted width, resource-accounts the fanout, closes feature-aligned grids over required coordinates, emits width-split semantic routes and retains width/spacing/length/edge-count evidence. Retained `PhaseTuningGroup` assemblies now evaluate two or more patterns privately, prove exact final skew, reject foreign straight-route, retained/zone-stitching via-land, exact transformed pads, copper zones and copper keepouts without partial application, and optionally certify that a differential pair remains an exact constant translation at required center spacing. A geometry-enabled realized-zone mode repours the candidate board, certifies requirements covered by each zone's exact configured clearance, retains route/zone/clearance/island/status evidence, proves stricter-band collisions with exact point classification, and rejects uncertified relationships atomically. `RoutingSolution::quality_report` adds exact per-net/aggregate straight-line length, Euclidean spanning-tree lower bound, excess, stretch and via-count evidence with typed missing/curved/unmeasurable cases. Automatic tuning-intent synthesis and the packaged interactive replay viewer are separately implemented below. A retained 19-net public-taxonomy corpus and non-gating wall-clock benchmark now cover single-layer bus, multilayer crossing and bounded any-angle workloads; automated thousand-sample upstream datasets, standardized history, incremental/memory metrics and dense BGA workloads remain |
| Routing reproducibility and exact pad certification | Pipeline diagnostics | Out of scope | Implemented | Every negotiated report retains its complete policy plus exact grid-state count, pass and per-connection budgets, executed passes, total/peak expansions, typed expansion-limit count and pass-budget exhaustion. Uniform and feature-aligned fixtures plus an original 19-net public-taxonomy corpus pin machine-neutral work ceilings, complete exact quality and exact replay equality. The corpus also exports every problem through the current SimpleRouteJson adapter, applies and revalidates each result; a non-gating optimized benchmark reports elapsed time beside grid, pass, expansion, length and stretch evidence. Routing and atomic phase tuning share exact transformed circle/rectangle/rounded-rectangle/obround/polygon pad predicates; connected and electrically unmapped placed pads remain physical obstacles, with channel, rotation, board-side and tuning regressions. This supersedes the earlier routing-row references to conservative tuning-pad envelopes, missing runtime-budget evidence, and absent wall-clock observations; thousand-sample upstream automation, standardized benchmark history, incremental/memory metrics and dense BGA corpora remain. |
| Routing iteration visualization | Pipeline visualization state | Out of scope | Implemented | `NegotiatedRouteReport::iteration_svg` renders any retained pass as a deterministic standalone, layer-filterable SVG without reconstructing private grid indices. `replay_html` packages all of those frames into one self-contained interactive viewer with pass slider, previous/next and keyboard navigation, live edge/conflict/failure metrics, terminal run status, and an aggregate exact-to-finite projection audit. Exact provisional paths and layer transitions, source-addressable node/segment/via conflicts, and typed failure metadata remain source-addressable. Conflict, completion, filtering, absent/empty-pass, expansion-limit and byte-identical rerun regressions plus `advanced_routing`'s HTML and per-pass SVG artifacts keep the viewer executable. This supersedes the earlier routing-row reference to a missing ready-made interactive viewer. |
| Automatic tuning-intent synthesis | Manual tuning declarations | Out of scope | Implemented | `PhaseTuningSynthesisPolicy` selects distinct routed nets, an explicit or inferred longest-member target, exact amplitude/pitch/tolerance/region padding, cycle and complete-assembly bounds, skew/clearance, and optional differential-pair coupling. `PcbLayout::synthesize_phase_tuning` derives stable member identities, route selectors and tight exact regions, filters generated copper through certified board containment, deterministically enumerates both excursion sides, and exposes intent only when the existing atomic phase tuner certifies the whole assembly against foreign routes, vias, exact transformed pads, zones, keepouts, final skew and pair translation. Reports distinguish invalid/duplicate/unknown/unsupported/unreachable/no-span/indeterminate-boundary/no-certified/candidate-limit outcomes, retain available/considered work, and separately apply intent or already-certified copper without partial mutation. Coupled-pair, exact deterministic replay, duplicate identity, already-satisfied replay and hard candidate-limit regressions plus `phase_tuning` keep the workflow executable. This supersedes the earlier routing-row reference to missing automatic tuning-region/pattern synthesis. |
| `csgrs` materialization | Internal render IR | Out of scope | Partial | Source-addressable exact mixed-curve substrate/cutouts, copper and drill plus exact-margin, side-aware pad/via mask, paste/legend and caller-font text features with certified process-layer unions exist. Every copper feature retains a typed pad/instance/pin, route, via, zone or placed-artwork owner instead of deriving semantics from geometry or display strings. Generated multi-segment orthogonal routes retain one semantic owner while materialization exactly unions their swept line-segment profiles; non-axis-aligned straight routes construct one exact-tangent capsule contour with caller-bounded round-cap sampling instead of relying on an uncertifiable higher-order boolean. Hypercircuit's canonical boundary carrier now feeds materialization and HyperDRC directly; only unsupported higher-order offset construction remains indeterminate in clearance search |
| Source-addressable DRC handoff | Internal checks | External KiCad | Partial | Direct board/stackup/net-class/differential-pair plus native authored-keepout, exact-slot, component-envelope and certified mask/paste/legend checks run in HyperDRC. Controlled-impedance material handles resolve through HyperPhysics graphs without defaults; exact homogeneous Dk/Df values activate HyperDRC's supported single-ended and equal-width differential analytical route estimates, while every unsupported resolution state remains release-auditable. Target metadata failures, unsupported analytical applicability and out-of-tolerance estimates share the stable `net-impedance-target-readiness` identity. HyperCircuit defaults to release-blocking promotion, offers an explicit advisory policy, and retains the applied policy plus promotion count; CLI tests prove material-missing direct documents and physically mismatched single-ended or differential routes block while in-target routes release cleanly. Uncertifiable profile offsets return an error-severity `geometry-uncertainty` finding with the requested rule and source layers instead of unwinding or using approximate fallback geometry |
| Checked release workflow | Yes | Validation/export | Implemented | `CheckedDesign::prepare_release` and `CheckedProject::prepare_release` resolve retained placement and compose ERC, csgrs materialization, HyperDRC, Gerber/Excellon/IPC-D-356 generation and re-import, package integrity, BOM/pick-and-place and assembly CSV reconciliation without introducing a second IR. Compiled hierarchy uses the validated path-qualified flat view while reusable definitions, scopes, schematics, and sources remain available. `ReleasePreparationReport` preserves every artifact and returns deterministic typed blockers, while only impossible evidence construction is an error |
| Project CLI | Dev/render/export commands | Manifest providers, `via ir` / `check` / `snapshot` / `bom` / exports | Implemented | A versioned `hypercircuit.toml` selects named/default designs produced by explicit argv or Cargo package/bin providers and can declare exact source-attributed PCB dielectric properties. Every command parses and validates the provider's `SemanticDocument`; direct JSON paths remain supported without guessing an implicit material catalog. `ir` and `snapshot` emit canonical exact semantic JSON, `bom` resolves retained placement before CSV, and `check`, `export-kicad`, `export-svg`, and `release` consume the same document. Check executes ERC plus the complete PCB release-evidence pipeline in memory; release persists audited CAM and assembly documents, reports typed blockers, and uses distinct clean/blocked/error statuses. External-process integration tests prove ancestor discovery, provider execution, a blocked direct controlled-impedance document, and the same document's clean manifest-material check/release |
| Schematic layout/rendering | Yes | Yes | Partial | Reusable model-bound `SchematicSymbolDefinition` libraries own independently placeable multipart units, logical pin presentation, nominal exact extents and exact line/rectangle/circle/arc/polyline/text graphics; lightweight `SchematicSymbol` placements reference definition/unit identity. Validation covers library/model/unit/pin/graphic integrity and placement compatibility, SVG renders the retained primitives, automatic layout shares one generated definition per device model, and atomic editor deltas insert/remove/replace definitions independently of placements. Fluent `Symbol`/`SymbolPin` declarations validate required pin coverage atomically, and `Design::connect` chains visible endpoints into retained net-checked wires from the same connectivity declaration. `Circuit::auto_schematic` derives bounded deterministic generic symbols, model-style pin sides, boundary-port placements, typed orthogonal wires and net labels from the authoritative graph. Native KiCad export embeds each used definition once, preserves multipart unit selection and exact graphics, coalesces native two-point segments on import, and validates reconstructed presentation against caller-supplied circuit truth. Standalone `.kicad_sym` import resolves inheritance and unit-zero common content, maps native ERC pin types, preserves supported exact graphics/multipart units, and emits typed loss for alternate bodies and unsupported primitives. Explicit sheet trees round-trip as root/child native files with stable sheet/file/path/page evidence, parent sheet pins, matched child hierarchical labels, restored link/port/net/original-name identity and typed missing-file/cycle/duplicate failures. Schematic topology optimization and broader exotic third-party primitives remain |
| KiCad import/export | Export | Draft export | Partial | Audited PCB export plus semantic net/footprint/pad/line-and-midpoint-arc placement/route/via/zone/contour import supports revision-checked edit/re-export/re-import. Pad-local rotation is retained exactly through copper/drill materialization, placement bounds, SVG/LCEDA review and board/standalone-library round trips. Centered horizontal/vertical oval drills round-trip exactly as retained centerline slots through board and standalone `.kicad_mod` paths; non-centered or diagonal slots export as a valid round cutter-diameter drill with typed projection evidence. Board and standalone `.kicad_mod` paths retain every independent package-model URI plus offset/scale/rotation, including multiple models without a fallback body envelope. Ordered copper/dielectric/surface-mask thickness and material round-trip through native `setup/stackup` records. A minimal `.kicad_pro` companion round-trips native class identities, exact-name assignments, preferred trace width, clearance and via land/drill defaults; a `.kicad_dru` companion independently enforces minimum width/clearance and maximum routed length/via count. Both retain numeric projections and typed loss for flattened inheritance, wildcard/composite assignments, named via-style identity, signal-integrity and regional policy. Independent native `.kicad_sch` export/import embeds shared multipart definitions once, covers exact symbol graphics, pins, ports, labels, coalesced wires and recursive sheet hierarchy without transferring circuit ownership. Standalone `.kicad_sym` plus `.kicad_mod` import produces one portable unified part with inherited/multipart symbols, native pin kinds, pads/pin maps/drills/margins, supported artwork and package models; exact decimals and explicit projection/omission evidence cover every supported or dropped fact. Official installed resistor/0603 and multipart logic/DIP fixtures validate real library compatibility. Stable hidden HyperCircuit metadata retains definition/model/name/body/symbol/instance identity while native-safe library names remain an adapter detail. KiCad 9 CLI accepts and renders generated flat subsets, follows generated child `Sheetfile` references and emits both hierarchy pages, and accepts the same-stem PCB three-file output. Broader exotic library graphics/pad shapes and PCB/rule primitives remain |
| LCEDA export/import | No | Draft export | Partial | Deterministic Pro `.epro2` export maps footprints/pads, devices, schematic pages/wires, PCB placements/pad nets/tracks/vias/zones/outlines with explicit source units and projection/omission evidence. Stable HyperCircuit identities support audited baseline-relative PCB import of placement transforms, straight route segments, authored vias, zone boundaries/policy-neutral geometry and straight exterior/cutout segments. An independent schematic-presentation importer restores symbol position/quarter-turn transforms, label position/text, free junctions and complete wire polylines while preserving typed pin/port endpoints, page topology and reusable definitions. Both paths reconstruct finite editor decimals exactly, require complete duplicate-free stable-identity sets and structurally valid retained results, and keep circuit connectivity authoritative; PCB footprint assignments, libraries, stackup, constraints and keepouts remain baseline-owned and curved contours are preserved. Regressions edit each archive domain, import, validate, re-export and prove stable second import; they also reject connectivity tampering, missing documents and duplicate physical identities. The runnable example executes the combined cycle. Native-editor conformance, third-party archive discovery and broader library/native record import remain |
| Gerber X2, Excellon and IPC-D-356 | Yes | Out of scope | Partial | X2 copper, pad/via mask, paste, caller-font legend and authoritative exact-line/arc or explicitly bounded cubic-projection profile files plus plating-separated round/routed-slot programs, connected-pad/via electrical-test records and a versioned unit/font/geometry-projection/connectivity-omission/SHA-256 manifest are authored. Emitted files are independently re-imported through csgrs and HyperDRC for nonempty geometry, X2 setup, Excellon 4:6 scaling, tool/plating/round/route structure, exact IPC net/reference/pin/coordinate/class reconciliation, integrity and count evidence; broader third-party CAM conformance remains |
| BOM and pick-and-place | Yes | Partial | Implemented | Deterministic fitted BOM, exact-coordinate placement and variant DNP CSV are independently parsed and reconciled by logical reference across part/model/land-pattern, quantity/grouping, position, rotation, side and variant fields; unplaced fitted instances block the audit, while procurement enrichment remains owned by hyperparts |
| 2D/3D preview | Yes | Review artifacts | Partial | Audited native schematic/PCB SVG plus exact-Z stackup/copper and optional placed component-body fallback envelopes exist. Decided physical layers receive retained round-drill/exact-slot subtraction, ordered front/back mask layers receive certified opening-union subtraction, and every success/failure retains layer/source/process evidence. Land patterns retain zero or more external model references independently of fallback geometry, each with resolver URI, declared OBJ/STEP/VRML/glTF container and exact local transform. A caller resolver supplies exact bytes without transferring I/O policy; native OBJ, VRML/WRL indexed-face scenes and self-contained glTF/GLB triangle scenes record per-instance/model-index SHA-256/source-node/primitive/triangle evidence, apply front/back placement and suppress the fallback envelope whenever at least one model resolves. VRML transform/group/shape graphs, `DEF`/`USE` reuse and exact polygon triangulation are supported; omitted line/point and degenerate geometry is counted. glTF node transforms are flattened, while external buffers, skins, morph targets and non-triangle modes fail explicitly. Model loading does not require an envelope. Unsupported STEP loading and malformed/resolution failures remain typed omissions. A generic csgrs named-mesh adapter emits one self-contained glTF 2.0 scene while HyperCircuit reports stable layer/instance/model identity and the explicit binary32 coordinate boundary. `examples/review_bundle.rs` embeds and resolves an OBJ package, then writes schematic SVG, PCB SVG, named glTF and a review manifest from one checked design. STEP/B-rep assembly remains |
| JSON/editor interchange | Circuit JSON | JSON snapshots | Partial | Versioned circuit/schematic/PCB JSON provides exact-value round trip and reported migration of v8–v25 inputs to v26, including point-ring promotion to mixed curve segments, exact pulse/sine/exponential source parameters, MOSFET polarity plus drain/gate/source roles, preferred-versus-minimum trace width, promotion of legacy per-placement symbol bodies/pins into reusable definitions, promotion of untyped package-model handles into explicit format/transform references, separation of model lists from optional fallback body envelopes, exact pad-local rotation, pair-owned differential target/tolerance and neck-down intent, and atomic phase-tuning groups. Whole-document atomic batches work with or without a PCB; attach/remove complete schematic or PCB containers; and create/remove nets, ordered buses/slices, circuit ports, rails, module parameters, source stimuli, child-circuit instances, models, instances, reusable schematic definitions and placements, retained schematic objects, land patterns, keepouts, placements, constraints and copper. Field edits cover circuit interface/hierarchy/simulation policy and stamps, complete schematic definitions/objects, board outline/stackup/rules, and full land-pattern/keepout/model/instance/constraint/route/via/zone definitions alongside narrower pin, assignment, geometry and net edits. Circuit/schematic/PCB rollback and placement-constraint replay remain atomic. Container-presence addresses conflict with every stale nested write; complete-definition writes conflict with narrower writes to the same object; independent identities and board fields rebase. Persisted position-preserving inverse batches provide monotonic undo/redo; a contiguous replay log reports conflicts and refuses compacted-away history. Distributed session transport and richer operation transforms remain |
| Library/registry workflow | Yes | Parts crates | Partial | Deterministic semver catalog resolution, versioned JSON lockfiles, exact source pins and digest/dependency verification exist. Versioned `PartLibraryArtifact` JSON unifies model, multipart symbol, land pattern, pin map and external part identity; `CircuitPackageStore` publishes immutable SHA-256-addressed artifacts, recognizes exact cache hits and loads every lock coordinate with digest/schema/semantic/package/version verification. `Design::export_part`/`import_part` use the retained definitions directly. Authenticated network registry discovery/fetch/push remains an adapter boundary |
| End-to-end representative-board tests | Yes | Yes | Implemented | — |
| Local routing-capacity refinement | Adaptive capacity depth | Out of scope | Implemented | `NegotiatedGridMode::LocallyRefined` retains exact axis-aligned `NegotiatedGridRefinementRegion` values over a coarse global mesh. Only coarse crossings and nodes inside those regions are searchable; orthogonal traversal skips inactive coordinate axes, octilinear traversal finds the nearest active equal-span neighbor, and generalized diagonal-cell resources preserve crossing conflicts across sparse index ranges. Policy validation rejects empty, out-of-board, disconnected, non-pitch-divisible and under-spaced refinement regions before search. `NegotiatedGridEvidence` distinguishes coordinate inventory, active planar nodes and nodes added beyond the coarse mesh. An executable two-net channel fixture completes, validates and materializes with 59 active nodes versus 169 for the equivalent global fine mesh. This supersedes the routing row's earlier reference to missing locally capacity-refined meshes. |
| Automatic routing-capacity synthesis | Adaptive capacity depth and pipeline diagnostics | Out of scope | Implemented | `PcbLayout::adaptive_negotiated_autoroute` starts with an audited feature-aligned coarse pass, derives exact hotspot envelopes from retained conflict geometry and failed-net terminals, pads and snaps them to one global fine-pitch lattice, expands them to a coarse crossing, merges overlaps deterministically, and reruns through `LocallyRefined`. `NegotiatedAdaptiveRoutePolicy` separately bounds reruns, regions, and padding; `NegotiatedAdaptiveRouteStatus` distinguishes complete, refinement-limit, region-limit, and no-progress outcomes. Every `NegotiatedAdaptiveRouteRound` retains its full ordinary routing report and exact proposed next region set. An executable same-layer channel fixture proves coarse failure, one synthesized `[0,4]–[12,8]` region, deterministic refined completion, semantic application, validation, and csgrs materialization. This supersedes the routing row's earlier automatic-refinement gap. |
| Bounded any-angle routing | Pipeline visibility routing | Out of scope | Implemented | `NegotiatedPlanarTopology::AnyAngle` retains a caller-bounded nearest-active-node visibility graph. Candidate segments still pass exact board/cutout/keepout, transformed-pad, fixed-route/via, regional-rule and escape-policy predicates. `RouteDirection::Arbitrary` distinguishes non-axis/non-45° intent and remains invariant under supported hierarchical transforms. A post-pass exact segment-distance audit applies each edge's emitted half-width and maximum pairwise clearance, marks both segment resources for negotiation, and skips only atomically certified retained differential pairs. Collinear emission now uses exact cross/dot products rather than merging unrelated arbitrary slopes. An executable crossing fixture proves that non-45° edges with no shared grid resource become typed segment conflicts; selected direct routing then reports exact Euclidean quality, validates, crosses HyperPath, and materializes through HyperCircuit's arbitrary-line capsule boundary. This supersedes the routing row's earlier angles-beyond-octilinear gap. |

## Workspace ownership audit

Circuit and PCB authoring vocabulary is owned by hypercircuit. csgrs retains
only exact geometry/materialization and format-level profile codecs. Its two
legacy electronics metadata markers and `pin`/`pad` role semantics have been
removed under the workspace breaking-change policy.
`LegacyCsgrsElectronicsImport` remains the versioned persisted migration reader:
it reads already-captured claims, rejects other schema revisions, and reports
every missing `DeviceModel` or `LandPattern` fact instead of deriving semantics
from geometry. Its exported
`LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION` schedules deletion for HyperCircuit
0.4.0, after callers have resolved every omission and persisted ordinary
semantic documents. A csgrs source-level regression rejects new public
circuit/PCB vocabulary, a reverse hypercircuit dependency, or restoration of
the retired marker and terminal-role vocabulary.

## Vocabulary placement

PCB code-CAD needs these retained nouns: `Circuit`, `DeviceModel`, `DevicePin`,
`CircuitInstance`, `Net`, `RailIntent`, `Bus`, `BusSlice`, `CircuitPort`,
`SourceStimulus`, `SourceWaveform`, `SourceWaveformPoint`,
`CircuitModuleParameter`, `CircuitModuleParameterTarget`,
`CircuitModuleParameterOverride`,
`ShockleyDiode`, `DiodeNewtonPolicy`, `DiodeNewtonIteration`,
`DiodeResidualReplayReport`,
`MosfetPolarity`, `SquareLawMosfet`, `MosfetRegion`,
`MosfetNewtonPolicy`, `MosfetOperatingPoint`,
`MosfetResidualReplayReport`, `AcOperatingPoint`,
`AcOperatingPointDeviceKind`, `AcOperatingPointProvenance`,
`AcNonlinearLinearization`,
`AcSmallSignalSweepReport`,
`Design`, `CheckedDesign`, `DesignCheckReport`, `PartDefinition`,
`PartDefinitionHandle`, `PartInstance`, `PartSymbolUnit`,
`SymbolUnitPlacement`, `PortablePartDefinition`, `PartLibraryArtifact`,
`CircuitPackageStore`, `PublishedPartLibrary`,
`KiCadPartLibraryImportOptions`, `KiCadPartLibraryImportReport`,
`KiCadLibraryImportOmission`, `NetHandle`, `BusHandle`,
`BusSliceHandle`, `PortHandle`, `InstanceHandle`,
`PinHandle`, `Part`, `PartPin`, `Footprint`, `SourceLocation`,
`AuthoringTarget`, `AuthoringAction`, `AuthoringTrace`, `DesignSourceMap`,
`DesignValidationIssue`, `DesignDiagnostic`, `Symbol`, `SymbolPin`,
`SchematicSymbolDefinitionId`, `SchematicSymbolDefinition`,
`SchematicSymbolUnit`, `SchematicGraphic`, `SchematicGraphicFill`,
`Route`, `RouteHandle`, `Via`, `ViaHandle`, `Zone`, `ZoneHandle`, `Keepout`,
`KeepoutHandle`, `PlacementRule`, `PlacementConstraintHandle`,
`ViaStyleRule`, `ViaStyleHandle`, `NetClassRule`, `NetClassHandle`,
`DifferentialPairRule`, `DifferentialPairHandle`, `LengthTuningRule`,
`LengthTuningPatternHandle`, `PhaseTuningGroupRule`, `PhaseTuningGroupHandle`,
`DesignModule`, `DesignModuleInstance`, `CheckedProject`, `ModuleBuildError`,
`LandPattern`, `LandPatternBody`, `LandPatternPad`, `PadPinMap`, `PcbPlacement`, `BoardOutline`,
`BoardContour`, `BoardContourSegment`,
`PcbStackup`, `NetClass`, `ResolvedNetClass`, `DifferentialPair`,
`DifferentialPairNeckdown`, `PcbRoute`, `PcbVia`,
`CopperZone`, `PcbKeepout`, `RouteConstraintRegion`, `RouteRuleRegion`, `EscapePolicy`,
`ViaStyle`, `ViaStyleSpan`, `LengthTuningPattern`, `PhaseTuningGroup`,
`PlacementPinAccessPolicy`, `PlacementPinAccessDirection`,
`PlacementPinAccessStatus`, `PlacementPinAccessProbeEvidence`,
`PlacementPinAccessTerminalEvidence`, `PlacementPinAccessIssue`,
`PlacementPinAccessScore`, `PlacementPinAccessReport`,
`PhaseTuningReport`, `PhaseTuningStatus`, `PhaseTuningIssue`,
`PhaseTuningObstacle`, `Pcb3dModelFormat`,
`Pcb3dModelReference`, `Pcb3dModelTransform`, `Pcb3dModelResolver`,
`Pcb3dComponentModel`, `Pcb3dModelResolutionEvidence`, `Pcb3dAssemblyReport`,
`Pcb3dSubtractionEvidence`, `Pcb3dSubtractionKind`, `Pcb3dSceneObject`,
`Pcb3dSceneObjectKind`, `Pcb3dGltfReport`, and `Pcb3dCoordinateEncoding`.
Route generation additionally needs retained policy/evidence nouns:
`NegotiatedGridMode`, `NegotiatedGridRefinementRegion`,
`NegotiatedPlanarTopology`, `RouteDirection`, `NegotiatedGridEvidence`,
`NegotiatedAdaptiveRoutePolicy`, `NegotiatedAdaptiveRouteStatus`,
`NegotiatedAdaptiveRouteRound`, `NegotiatedAdaptiveRouteReport`,
`NegotiatedRoutePolicy`, `NegotiatedRouteIteration`,
`NegotiatedRouteNodeState`, `NegotiatedRouteNetState`,
`NegotiatedRouteConflictGeometry`, `NegotiatedRouteConflictState`,
`NegotiatedRouteIterationState`, `NegotiatedRouteWorkEvidence`,
`NegotiatedRouteFailure`, `NegotiatedDifferentialPairEvidence`,
`NegotiatedDifferentialPairNeckdownEvidence`,
`NegotiatedRouteConstraintEvidence`, `NegotiatedRouteRuleRegionEvidence`,
`NegotiatedEscapePolicyEvidence`, `NegotiatedViaStyleEvidence`, and
`NegotiatedRouteReport`. Deterministic comparison additionally uses
`RoutingQualityStatus`, `RoutingQualityIssue`, `RoutingQualityNetEvidence`, and
`RoutingQualityReport`.
They belong in `hypercircuit`, because they carry electrical or manufacturing
meaning. `csgrs` should expose geometry vocabulary only: profiles, paths,
regions, solids, transforms, offsets, booleans, meshes and import/export
geometry adapters. A pad becomes a `csgrs::Profile` only during materialization;
the result retains a hypercircuit source id and net rather than teaching csgrs
what a pad or net means.

Current tscircuit and via-rs organization reinforces retained authoring, editor,
and routing vocabulary additions. Their concise component trees/design builders
lower into independent retained IRs rather than making render geometry the
source of truth. Hypercircuit's `Design` follows that boundary while emitting
its existing richer circuit/layout carriers directly.
The current [via-rs 0.1.1 package](https://docs.rs/crate/via-rs/0.1.1)
documents typed nets/rails/pins/footprints, checked read-only `Board` output,
project checks/snapshots/BOM and KiCad/LCEDA export; HyperCircuit maps those
roles to `Design`, `CheckedDesign`, its project CLI and retained adapters while
also owning routing and simulation. The current
[tscircuit autorouter](https://github.com/tscircuit/tscircuit-autorouter)
accepts `SimpleRouteJson`, iterates an inspectable pipeline, adaptively chooses
capacity depth and exposes visualization state. HyperCircuit now covers the
protocol, deterministic quality evidence, deterministic retained-feature
coordinate refinement, explicit sparse local-capacity regions, bounded
conflict/failure-driven refinement synthesis, bounded exact any-angle visibility,
exact inter-route clearance accounting, replayable solver-state data, and a
standalone per-iteration SVG plus packaged interactive HTML replay viewer.
Larger public performance corpora remain the distinct router gap.
Tscircuit's core produces a retained Circuit JSON rather than rendering
directly; its viewers consume that IR independently, and its PCB viewer exposes
replayable edit events. Hypercircuit now owns `DesignEdit`, `DesignEditBatch`,
`DesignEditId`, `EditTarget`, `EditAddress`, `EditReplayReport`,
`ConcurrentCommitReport`, `DesignRevision`,
`ReversibleDesignEdit`, `DesignHistory`, `DesignHistoryReplayReport`,
`SemanticMigrationStep`, and `SemanticMigrationReport` for editor-safe deltas,
monotonic undo/redo, presence-aware structural net/bus/slice/port/rail/
module-parameter/stimulus/subcircuit/device-model/instance/schematic/symbol/wire/
label/sheet/sheet-port/sheet-link/land-pattern/keepout/placement/constraint/
route/via/zone insertion and removal, plus atomic board/policy/stamp fields and explicit schema
upgrades instead of mutating exported geometry.
It also owns
`LayoutModule`, `LayoutModuleInstance`, `LayoutTransform`, `PlacementGroup`,
`LayoutAssembly`, and composition evidence for layout hierarchy. The checked
`DesignModule` authoring layer binds typed circuit ports to parent nets and
emits those retained carriers directly, so recursive layout is not a separate
connectivity graph.
`RouteConstraintRegion`, `RouteRuleRegion`, `EscapePolicy`,
`LengthTuningPattern`, and `PhaseTuningGroup` now retain advanced routing
intent across semantic JSON, routing handoff and hierarchical layout
composition; negotiated search enforces the first three, exact bounded
realization applies the fourth, and atomic phase realization privately applies
and certifies the fifth.
`ViaStyle` and `ViaStyleSpan` now retain
net-class-selected layer-transition construction, while negotiated search and
materialization retain exact use evidence. None are csgrs geometry concepts.
`DifferentialPairNeckdown` and
`NegotiatedDifferentialPairNeckdownEvidence` now own bounded terminal fanout
intent and its exact accepted-use proof. `PlacementPinAccessPolicy` and its
source-addressable report/evidence vocabulary now own exact placement-stage
fanout feedback while reusing router legality; this is circuit/PCB policy, not
geometry-engine state. `PhaseTuningZoneCollisionMode`,
`PhaseTuningRealizedZoneEvidence`, and `PhaseTuningRealizedZoneStatus` now keep
candidate-repour policy and proof in hypercircuit rather than adding that
concern to csgrs.

## Completion gates

Practical equivalence requires all of the following, not merely public structs:

1. Author a hierarchical representative board with typed pins, buses, rules,
   footprints, placements, mixed-curve board contours, routes, zones and
   keepouts in Rust.
2. Run structural ERC, lower routing work to `hyperpath`, materialize geometry
   through `csgrs`, and verify the result through `hyperdrc` without losing net
   or source identity.
3. Export and re-import at least KiCad plus a fabrication job (Gerber X2,
   Excellon, BOM and pick-and-place) with an explicit loss report. The supported
   KiCad PCB subset now passes exact edit/re-export/re-import fixtures for
   copper geometry, centered axis-aligned oval slots, physical stackup, native
   `.kicad_pro` class defaults and assignments, and an enforceable `.kicad_dru`
   rule companion; broader source coverage remains. The KiCad schematic subset
   emits native embedded generic
   symbols, ports, electrical labels, segmented wires and recursive Sheetfile
   hierarchy, passes flat and two-page KiCad 9 CLI rendering, and re-imports
   symbol coordinate edits plus nested sheet/link/port identity against
   retained circuit truth; arbitrary third-party libraries remain. The
   supported fabrication subset now re-imports
   Gerber geometry/X2 metadata and Excellon fixed coordinates, tool/plating,
   round-hit and routed-slot structure plus IPC-D-356 electrical connectivity
   with manifest/integrity/count evidence;
   independent third-party CAM conformance remains. BOM, pick-and-place and DNP
   CSV now independently re-import with exact design-owned field reconciliation.
   Deterministic LCEDA Pro export plus baseline-relative supported PCB and
   schematic-presentation import adds a second review/editor round trip with
   authored copper, symbol transforms, labels and wire bends plus
   machine-readable unit/loss audits; native-editor conformance remains.
4. Simulate a meaningful mixed linear/nonlinear/transient circuit with retained
   source stimuli and replay accepted results against retained equations.
   `tests/simulation.rs::mixed_linear_reactive_diode_run_replays_every_nonlinear_endpoint`
   now executes this gate with a step-driven R/C/Shockley-diode circuit; every
   accepted sample retains Newton and true-law replay evidence.
   `tests/mosfet.rs::common_source_dc_solve_replays_the_exact_saturation_law`
   separately executes declarative model extraction, exact square-law Newton
   solving and exact-zero replay for a common-source N-channel stage.
5. Produce reviewable schematic, PCB and 3D outputs from the same source model.
   `examples/review_bundle.rs` executes this gate directly: one checked fluent
   design produces schematic/PCB SVG, a named multi-object glTF scene, exact
   drill/mask-subtraction evidence, simulation-stamp evidence, explicit
   binary32 projection policy and a machine-readable review manifest.
6. Keep representative-board round trips, DRC, connectivity and simulation in
   CI, with no PCB-domain ownership reintroduced into `csgrs`.

`tests/release_workflow.rs` is the executable representative-board gate: one
retained design runs structural validation/ERC and exact DC simulation, emits
schematic/PCB/3D review artifacts, crosses the hyperpath routing boundary,
materializes through csgrs, runs native HyperDRC readiness, produces Gerber X2,
Excellon, BOM and pick-and-place outputs, independently re-imports the emitted
CAM and assembly documents, and round-trips semantic JSON plus the supported
KiCad PCB subset.
`tests/workflow.rs` independently proves that the concise typed `Design`
and `DesignModule` surfaces reach the same release machinery through
`CheckedDesign::prepare_release` and `CheckedProject::prepare_release`,
including exact simulation, placement resolution, path-qualified
source-addressable geometry, HyperDRC, fabrication/assembly re-import, clean
release aggregation, and inspectable typed blockers.
`tests/cli.rs` invokes the compiled project executable as an external process,
discovers a manifest from a nested directory, runs its default command provider,
and feeds one retained semantic document through check, canonical IR/snapshot,
BOM, KiCad export, SVG review, and release. It then inspects the native board,
preview, fabrication manifest, and assembly files.
`tests/curved_outline.rs` separately proves mixed line/arc/Bezier contour
retention, semantic migration, exact placement predicates, csgrs materialization
and HyperDRC handoff, SVG review, KiCad arc round trip, Gerber arc re-import,
default cubic CAM refusal, and explicit bounded exterior/cutout projection
through checked release. `examples/curved_fabrication.rs` exposes the same
policy as a runnable board-to-CAM demonstration.
`tests/autoroute.rs` and `tests/stitching.rs`
exercise exact circular-boundary clearance in both search paths.

## Practical-equivalence verdict

As of 2026-07-24, all six completion gates above pass and HyperCircuit has
practical workflow equivalence to the scoped public workflows of tscircuit and
via-rs 0.1.1. The verdict is based on executable end-to-end behavior, not type
count: one retained Rust model can be authored hierarchically, validated,
simulated, placed and routed, materialized through csgrs, checked through
HyperDRC, reviewed in 2D/3D, exported to KiCad/LCEDA and fabrication/assembly
documents, independently re-imported, and driven through the project CLI.
The ownership regression and the scheduled migration-reader deletion make
csgrs's geometry-only boundary explicit.

Rows still marked `Partial` identify breadth beyond this practical gate:
larger hosted/public benchmark automation, authenticated registry transport,
exotic third-party editor records, general STEP/B-rep loading, broader device
physics and sparse large-system simulation. They remain useful roadmap work,
but do not remove any representative-board workflow that either comparison
target puts in scope. Any future regression in gates 1–6 invalidates this
verdict even if individual feature rows continue to compile.
