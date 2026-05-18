<h1>
  hyperdrc
  <img src="./docs/hyperdrc.png" alt="hyperdrc logo" width="144" align="right">
</h1>

`hyperdrc` is a Rust library and thin command line tool for PCB design-readiness review.
It loads Gerber, KiCad, Excellon, IPC-D-356, package sidecars, and converted handoff
artifacts, then emits evidence-rich findings for local review and CI.

The project is a readiness reviewer, not a fabricator-certified CAM engine. Its job is
to make release-package risk visible before upload and to preserve the evidence behind
each finding.

## Hyper Ecosystem

`hyperdrc` is a domain crate that bridges current CAM/parser tooling and the exact
Hyper stack.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact source-grid, coordinate,
  stackup, and rule values where checks can preserve them.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy for
  future geometry checks that should not use local epsilon rules.
- [hypercurve](https://github.com/timschmidt/hypercurve) and
  [hypertri](https://github.com/timschmidt/hypertri): exact-aware planar geometry and
  triangulation surfaces for future CAM pipelines.
- [hyperpath](https://github.com/timschmidt/hyperpath): routing, clearance, tangent,
  and path provenance carriers.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, package, interface,
  process, BOM, and compatibility evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit) and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): future electrical,
  thermal, material, and coupled readiness checks.
- [csgrs](https://github.com/timschmidt/csgrs): current Gerber parsing, polygon offset,
  and boolean interop target used by the prototype.

## Typical PCB Readiness Problems

PCB release packages are not just copper polygons. Boards fail review because layer
roles are ambiguous, drill and test files disagree, stackup evidence is incomplete,
sidecars are stale or malformed, generated outputs do not match source intent, or
CAD-only assumptions disappear in Gerber handoff. Traditional tools often collapse
parser assumptions, CAM polygon approximations, rule policy, and manufacturing evidence
into one report.

`hyperdrc` treats readiness as evidence preservation. Inputs, parser diagnostics, source
grids, conversion history, waivers, baselines, rule configuration, and report formats
are first-class data. Geometry findings are conservative review prompts, not fabricator
guarantees, and lossy adapter status remains visible until exact Hyper geometry covers
more of the CAM pipeline.

## Main Types And Surfaces

- The library `run` pipeline returns a serializable `Report` through `RunOutcome`
  instead of terminating the process.
- Parser modules load Gerber, KiCad, Excellon, IPC-D-356, sidecar tables, drawings,
  converter outputs, and handoff artifacts with structured diagnostics.
- Check modules emit stable finding IDs, severity, source context, messages, geometry,
  and active/waived status.
- Rule and policy types cover CLI/config thresholds, package profiles, assembly
  profiles, stackup metadata, net classes, fabricator capability profiles, waivers, and
  baselines.
- Report writers produce text, JSON, JSON Lines, GeoJSON, SARIF, GitHub annotations,
  HTML, JUnit, SQLite, Arrow IPC, Parquet, overlays, and review companions.
- Repository-local READMEs under `src`, `src/checks`, `src/geometry`, `src/kicad`,
  `docs`, `examples`, and `benches` document the larger internal map.

## Precision Model

`hyperdrc` currently combines exact-aware metadata with pragmatic CAM adapters.
Parser diagnostics preserve source units, grid declarations, file roles, X2 attributes,
sidecar schema evidence, and converter provenance. Geometry checks are conservative
review prompts, not proof that a board is manufacturable.

Where exact Hyper geometry is not yet wired through the full CAM path, the README and
reports should keep that boundary visible. Decimal/source-grid facts should be retained
at import, finite geometry should be lifted into Hyper crates where practical, and
lossy `geo`/`csgrs` adapters should remain explicit.

## Performance Model

The crate is intended to run in local preflight and CI. It keeps the CLI thin over the
library pipeline, streams progress for long-running checks to stderr, and supports
machine-readable output formats for downstream tooling. Rule configuration, sidecar
discovery, parser diagnostics, baselines, and waivers are resolved once and then reused
by checks and reports.

Performance work should keep expensive geometry checks observable, avoid hiding parser
or adapter costs, and prefer reusable source/provenance records over reparsing release
packages for each output format.

## Current Status

`hyperdrc` is an active prototype with broad regression coverage. Implemented today:

- Gerber directory/package loading, KiCad board parsing, Excellon sidecars, IPC-D-356
  sidecars, package archives, common manufacturing sidecars, and converter output
  discovery;
- parser diagnostics for Gerber/X2, KiCad, Excellon, IPC-D-356, BOM/centroid/netlist
  tables, spreadsheets, drawings, IPC-2581, ODB++, STEP/mesh/image/test artifacts, and
  converter manifests;
- readiness checks for copper/layer geometry, board outline, mask/paste/silkscreen,
  drills, IPC-D-356, KiCad net/board context, stackup/net-class policies, assembly/test
  features, generated-output freshness, package completeness, and waiver governance;
- JSON rule configuration, CLI overrides, package/assembly/fabricator profiles, stackup
  metadata, net-class constraints, waivers, baselines, and baseline diffs;
- reports in text, JSON, JSON Lines, GeoJSON, SARIF, GitHub annotations, HTML, JUnit,
  SQLite, Arrow IPC, and Parquet;
- review overlays and companions for SVG, Gerber, Excellon, DXF, PDF, KiCad marker
  boards/rules, IPC-D-356, GenCAD, and IPC-2581.

Known limits: findings are conservative preflight evidence, not a replacement for a
fabricator DFM/DRC pass. Routed-slot geometry, full ODB++/IPC-2581 import,
glyph-accurate text, custom pad booleans, rich impedance solving, and several
format-specific dialects remain incomplete.

## Installation

`hyperdrc` is primarily used from a checkout while the Hyper crates are developed
together:

```sh
cargo run -- --help
```

As a library dependency from sibling checkouts:

```toml
[dependencies]
hyperdrc = { path = "../hyperdrc" }
```

## Library And CLI

The reusable API lives in `src/lib.rs` and exposes parser modules, checks, report types,
policies, and the `run` pipeline. The binary in `src/main.rs` parses CLI flags, calls
the library, and maps active findings to process status. Use `--allow-findings` for
report-only automation.

## Quick Start

Run default checks over files or a Gerber package directory:

```sh
cargo run -- path/to/top.gbr path/to/bottom.gbr
cargo run -- --gerber-dir path/to/gerber-package
```

Run KiCad-aware checks with a config file and sidecars:

```sh
cargo run -- \
  --config examples/hyperdrc-config.json \
  --kicad-pcb board.kicad_pcb \
  --excellon board.drl \
  --ipc356 board.ipc \
  --format geojson
```

Generate review artifacts without failing the surrounding shell recipe:

```sh
cargo run -- \
  --allow-findings \
  --kicad-pcb board.kicad_pcb \
  --format html \
  --svg-overlay violations.svg \
  --summary-file summary.json
```

Useful converter entry points include `--converter kicad-cli`, `--convert-input`,
`--conversion-output-dir`, and sidecar export flags for handoff/review artifacts.

## Readiness Coverage

The default suite covers five broad surfaces:

- layer geometry: copper overlap, edge clearance, mask/paste alignment, silkscreen
  clearance, feature width, acid traps, copper balance, density, and outline sanity;
- drills and fabrication context: annular ring, drill spacing/clearance, routed-slot
  readiness, castellation intent, aspect ratio, Excellon evidence, and cross-source
  drill consistency;
- KiCad board context: net intent, high-speed/current/RF/ESD/thermal heuristics,
  differential-pair review, panelization, keepouts, grounding, stackup and net-class
  constraints;
- assembly and test readiness: component clearances, fiducials, tooling holes, mouse
  bites, testpoint coverage/accessibility, selective/wave/press-fit/conformal-coating
  evidence, and IPC-D-356 coverage;
- package readiness: required artifacts, sidecar discovery, BOM/centroid/netlist
  structure, drawings, generated-date freshness, polarity/MSL/surface-finish handoff
  notes, overlays, waivers, and baselines.

The check ownership map is in [src/checks](src/checks/README.md), and the long-form
roadmap remains in [docs/design-readiness-plan.md](docs/design-readiness-plan.md).

## Repository Map

- [src](src/README.md): library structure, runtime pipeline, parsers, reports,
  configuration, and modules.
- [src/checks](src/checks/README.md): readiness checks grouped by ownership.
- [src/geometry](src/geometry/README.md): polygon construction and geometry-test
  expectations.
- [src/kicad](src/kicad/README.md): KiCad model and parser scope.
- [docs](docs/README.md): roadmap and visual assets.
- [docs/testing.md](docs/testing.md): test-suite guide.
- [examples](examples/README.md): runnable configuration examples.
- [benches](benches/README.md): benchmark and smoke-performance entry points.
- [proptest-regressions](proptest-regressions/README.md): persisted property-test
  regression seeds.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench parser_geometry_smoke
cargo bench --bench fixture_smoke
```

## References

hyperdrc comments and readiness heuristics cite these design and manufacturing
references where the code implements related checks. Entries are kept in MLA
style so they can be copied into engineering review notes.

- Areny, F. A., et al. "A Study of SnAgCu Solder Paste Transfer Efficiency and Effects of Optimal Reflow Profile on Solder Deposits." *Microelectronic Engineering*, 2011, https://doi.org/10.1016/j.mee.2011.02.104.
- Andrew, A. M. "Another Efficient Algorithm for Convex Hulls in Two Dimensions." *Information Processing Letters*, vol. 9, no. 5, 1979, pp. 216-219, https://doi.org/10.1016/0020-0190(79)90072-3.
- Becerra, Jose, Dennis Willie, and Murad Kurwa. "Press Fit Technology Roadmap and Control Parameters for a High Performance Process." *IPC APEX EXPO Conference Proceedings*, Flextronics, https://www.circuitinsight.com/pdf/press_fit_technology_roadmap_control_parameters_ipc.pdf. Accessed 14 May 2026.
- Bentley, Jon Louis. "Multidimensional Binary Search Trees Used for Associative Searching." *Communications of the ACM*, vol. 18, no. 9, 1975, pp. 509-517, https://doi.org/10.1145/361002.361007.
- Bhargava, Ankit, et al. "DC-DC Buck Converter EMI Reduction Using PCB Layout Modification." *IEEE Transactions on Electromagnetic Compatibility*, vol. 53, no. 3, 2011, pp. 806-813, https://doi.org/10.1109/TEMC.2011.2145421.
- Black, J. R. "Electromigration--A Brief Survey and Some Recent Results." *IEEE Transactions on Electron Devices*, vol. 16, no. 4, 1969, pp. 338-347, https://doi.org/10.1109/T-ED.1969.16754.
- Chen, Fen, and Ning-Cheng Lee. "A Novel Solution for No-Clean Flux Not Fully Dried Under Component Terminations." *Indium Corporation Technical Paper*, 2015, https://www.electronics.org/system/files/technical_resource/E39%26S13_03%20-%20Ning%20C.%20Lee.pdf. Accessed 14 May 2026.
- Chesser, Kevin, and May Porley. "What Are the Basic Guidelines for Layout Design of Mixed-Signal PCBs?" *Analog Dialogue*, vol. 56, no. 3, 2022, https://www.analog.com/en/resources/analog-dialogue/articles/what-are-the-basic-guidelines-for-layout-design-of-mixed-signal-pcbs.html. Accessed 14 May 2026.
- Cohn, S. B. "Characteristic Impedance of the Shielded-Strip Transmission Line." *IRE Transactions on Microwave Theory and Techniques*, vol. MTT-2, no. 2, 1954, pp. 52-57, https://doi.org/10.1109/TMTT.1954.1124875.
- Eurocircuits. "Tombstoning." *Eurocircuits Technical Guidelines*, https://www.eurocircuits.com/technical-guidelines/pcb-assembly-guidelines/tombstoning/. Accessed 13 May 2026.
- Ericson, Christer. *Real-Time Collision Detection*. CRC Press, 2005.
- Farin, Gerald. *Curves and Surfaces for CAGD: A Practical Guide*. 5th ed., Academic Press, 2002.
- FixturFab. "Design for Test: How to Design Test Points for PCB Testing." *FixturFab Resources*, https://fixturfab.com/resources/how-to-test/design-for-test. Accessed 13 May 2026.
- GitHub. "Workflow Commands for GitHub Actions." *GitHub Docs*, https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands. Accessed 13 May 2026.
- Hammerstad, E., and O. Jensen. "Accurate Models for Microstrip Computer-Aided Design." *1980 IEEE MTT-S International Microwave Symposium Digest*, 1980, pp. 407-409, https://doi.org/10.1109/MWSYM.1980.1124303.
- Harter, Stefan, et al. "The Effect of Area Shape and Area Ratio on Solder Paste Printing Performance." *SMTA International*, 2016, https://www.circuitnet.com/programs/55115.html.
- Hinnant, Howard. "chrono-Compatible Low-Level Date Algorithms." *Howard Hinnant's Date Algorithms*, https://howardhinnant.github.io/date_algorithms.html. Accessed 13 May 2026.
- Hollstein, K., X. Yang, and K. Weide-Zaage. "Thermal Analysis of the Design Parameters of a QFN Package Soldered on a PCB Using a Simulation Approach." *Microelectronics Reliability*, vol. 120, 2021, article 114118, https://doi.org/10.1016/j.microrel.2021.114118.
- IPC. *Generic Standard on Printed Board Design: IPC-2221B*. IPC, https://www.ipc.org/TOC/IPC-2221B.pdf. Accessed 13 May 2026.
- IPC. *Standard for Determining Current Carrying Capacity in Printed Board Design: IPC-2152*. IPC, 2009, https://shop.ipc.org/ipc-2152/ipc-2152-standard-only.
- IPC. *Bare Substrate Electrical Test Data Format: IPC-D-356B*. IPC, 1 Oct. 2002, https://shop.electronics.org/ipc-d-356/ipc-d-356-standard-only.
- IPC. *Computer Numerical Control Formatting for Drillers and Routers: IPC-NC-349*. IPC, 1985, https://www.electronics.org/TOC/IPC-NC-349.pdf. Accessed 16 May 2026.
- IPC. *Generic Requirements for Surface Mount Design and Land Pattern Standard: IPC-7351B*. IPC, 2010, https://shop.ipc.org/ipc-7351/ipc-7351-standard-only.
- KiCad. "S-Expression Format." *KiCad Developer Documentation*, https://dev-docs.kicad.org/en/file-formats/sexpr-intro/. Accessed 15 May 2026.
- IEC. *IEC 60352-5: Solderless Connections, Part 5: Press-In Connections, General Requirements, Test Methods and Practical Guidance*. International Electrotechnical Commission, https://webstore.iec.ch/publication/23286.
- IEC. *IEC 61000-4-5: Electromagnetic Compatibility (EMC), Part 4-5: Testing and Measurement Techniques, Surge Immunity Test*. International Electrotechnical Commission, https://webstore.iec.ch/publication/4184.
- IEEE. *IEEE Standard for Configuration Management in Systems and Software Engineering: IEEE Std 828-2012*. IEEE, 2012, https://doi.org/10.1109/IEEESTD.2012.6170935.
- IPC. *Press-Fit Standard for Automotive Requirements and Other High-Reliability Applications: IPC-9797*. IPC, May 2020, https://www.ipc.org/TOC/IPC-9797-toc.pdf.
- IPC. *Requirements for Soldered Electrical and Electronic Assemblies: IPC J-STD-001H*. IPC, Sept. 2020, https://shop.ipc.org/ipc-j-std-001/ipc-j-std-001-standard-only.
- IPC. *Requirements for Electrical Testing of Unpopulated Printed Boards: IPC-9252B*. IPC, 2016, https://shop.ipc.org/ipc-9252/ipc-9252-standard-only.
- IPC. *Guidelines for Temperature Profiling for Mass Soldering Processes (Reflow and Wave): IPC-7530*. IPC, https://shop.ipc.org/ipc-7530/ipc-7530-standard-only.
- IPC. *Performance Specification for Electroless Nickel/Immersion Gold (ENIG) Plating for Printed Boards: IPC-4552B*. IPC, Apr. 2021, https://www.ipc.org/TOC/IPC-4552B-toc.pdf.
- IPC. *Qualification and Performance Specification for Rigid Printed Boards: IPC-6012D*. IPC, https://www.ipc.org/TOC/IPC-6012D.pdf. Accessed 13 May 2026.
- IPC. *Specification for Electroless Nickel/Electroless Palladium/Immersion Gold (ENEPIG) Plating for Printed Circuit Boards: IPC-4556*. IPC, 5 Feb. 2013, https://shop.electronics.org/ipc-4556/ipc-4556-standard-only/Revision-0/english.
- Ucamco. *The Gerber Layer Format Specification, Revision 2024.05*. Ucamco NV, 2024, https://www.ucamco.com/en/gerber/downloads. Accessed 16 May 2026.
- IPC. *Specification for Immersion Silver Plating for Printed Boards: IPC-4553A*. IPC, 16 June 2009, https://webstore.ansi.org/standards/ipc/ipc4553a2009.
- IPC. *Stencil Design Guidelines: IPC-7525B*. IPC, https://www.ipc.org/TOC/IPC-7525B.pdf. Accessed 13 May 2026.
- Kirschning, M., and R. H. Jansen. "Accurate Wide-Range Design Equations for the Frequency-Dependent Characteristic of Parallel Coupled Microstrip Lines." *IEEE Transactions on Microwave Theory and Techniques*, vol. 32, no. 1, 1984, pp. 83-90, https://doi.org/10.1109/TMTT.1984.1132616.
- Oezkoek, Mustafa, Joe McGurran, Dieter Metzger, and Hugh Roberts. "Wire Bonding and Soldering on ENEPIG and ENEP Surface Finishes with Pure Pd-Layers." *IPC Technical Resource*, Atotech, https://www.ipc.org/system/files/technical_resource/E5%26S34_01.pdf. Accessed 15 May 2026.
- Parnas, D. L. "On the Criteria To Be Used in Decomposing Systems into Modules." *Communications of the ACM*, vol. 15, no. 12, 1972, pp. 1053-1058, https://doi.org/10.1145/361598.361623.
- Paterson, Donald G., and Miles A. Tinker. "Studies of Typographical Factors Influencing Speed of Reading. II. Size of Type." *Journal of Applied Psychology*, vol. 13, no. 2, 1929, pp. 120-130, https://doi.org/10.1037/h0074167.
- Chin, Cheng-Hao, and Gnyaneshwar Ramakrishna. "Impact of BGA Escape Trace Design on Performance of Solder Joint." *SMTA International*, Cisco Systems, https://www.circuitnet.com/programs/56311.html. Accessed 14 May 2026.
- Jonnalagadda, K. "Reliability of Via-in-Pad Structures in Mechanical Cycling Fatigue." *Microelectronics Reliability*, vol. 42, no. 2, 2002, pp. 253-258, https://doi.org/10.1016/S0026-2714(01)00136-6.
- Lee, Jae-Hun, et al. "Effect of Pulse-Reverse Plating on Copper: Thermal Mechanical Properties and Microstructure Relationship." *Microelectronics Reliability*, vols. 100-101, 2019, article 113383, https://doi.org/10.1016/j.microrel.2019.06.062.
- Lee, D. T., and Franco P. Preparata. "Computational Geometry - A Survey." *IEEE Transactions on Computers*, vol. C-33, no. 12, 1984, pp. 1072-1101, https://doi.org/10.1109/TC.1984.1676388.
- Lin, Ming C., and John F. Canny. "A Fast Algorithm for Incremental Distance Calculation." *Proceedings. 1991 IEEE International Conference on Robotics and Automation*, 1991, pp. 1008-1014, https://doi.org/10.1109/ROBOT.1991.131723.
- OASIS. *Static Analysis Results Interchange Format (SARIF) Version 2.1.0*. Edited by Michael C. Fanning and Laurence J. Golding, OASIS Committee Specification 01, 23 July 2019, https://docs.oasis-open.org/sarif/sarif/v2.1.0/cs01/sarif-v2.1.0-cs01.html.
- STMicroelectronics. *AN576: Influence of the PCB Layout on the ESD Protection*. STMicroelectronics, DocID3588 Rev. 3, https://www.st.com/resource/en/application_note/an576-pcb-layout-optimisation-stmicroelectronics.pdf. Accessed 14 May 2026.
- Sun, Yanhui, et al. "Multi-Physics Coupling Aid Uniformity Improvement in Pattern Plating." *Applied Thermal Engineering*, vol. 108, 2016, pp. 1197-1206, https://doi.org/10.1016/j.applthermaleng.2016.07.182.
- Tang, Yinggang, et al. "Study on Wet Chemical Etching of Flexible Printed Circuit Board with 16-um Line Pitch." *Journal of Electronic Materials*, vol. 52, 2023, pp. 4030-4036, https://doi.org/10.1007/s11664-023-10368-z.
- Toussaint, Godfried T. "Solving Geometric Problems with the Rotating Calipers." *Proceedings of IEEE MELECON '83*, 1983.
- Wilcoxon, Ross, Tim Pearson, and David Hillman. "Modeling the Effects of Thermal Pad Voiding on Quad Flatpack No-Lead (QFN) Components." *Journal of Surface Mount Technology*, vol. 36, no. 2, 2023, https://doi.org/10.37665/smt.v36i2.37.
- Wheeler, H. A. "Transmission-Line Properties of a Stripline Between Parallel Planes." *IEEE Transactions on Microwave Theory and Techniques*, vol. 26, no. 11, 1978, pp. 866-876, https://doi.org/10.1109/TMTT.1978.1129505.
- Wong, Hang, et al. "Small Antennas in Wireless Communications." *Proceedings of the IEEE*, vol. 100, no. 7, 2012, pp. 2109-2121, https://doi.org/10.1109/JPROC.2012.2188089.
- Xu, Jun, and Shuo Wang. "Investigating a Guard Trace Ring to Suppress the Crosstalk Due to a Clock Trace on a Power Electronics DSP Control Board." *IEEE Transactions on Electromagnetic Compatibility*, vol. 57, no. 3, 2015, pp. 546-554, https://doi.org/10.1109/TEMC.2015.2403289.
