# HyperDRC reference and performance audit

This audit maps every README reference to a current implementation boundary and
records only measured performance work. HyperDRC remains a conservative release
reviewer: a spatial index may propose candidates, but exact or explicitly lossy
narrow-phase geometry still decides findings.

## Retained optimization

The regular-grid broad phases in `checks::spatial` previously stored exact cell
keys in `BTreeMap`s. Bentley establishes the value of multidimensional search
structures, while Teschner et al. show that a regular spatial grid can be
compressed efficiently with hashing. HyperDRC already had the conservative grid,
cell-size inflation, and exact narrow phase; the retained change substitutes
`HashMap` only for exact bucket-key lookup.

Candidate report order does not depend on hash iteration. Queries probe cells in
explicit coordinate order, bucket members retain insertion order, and every
returned candidate vector is sorted and deduplicated before use. A regression test
rebuilds randomized hash tables and checks the same sorted IDs. Rust's map also
resolves hash collisions by key equality, so unrelated cells cannot alias at the
API boundary.

The isolated release benchmark constructs 10,003 drills, including a populated
nearby cluster, and runs `drill_spacing` 200 times. Five sequential runs were used
for each median on 2026-07-15:

| Storage | Runs (ms) | Median | Change |
| --- | --- | ---: | ---: |
| Ordered grid buckets | 1615.162, 1618.816, 1605.697, 1593.236, 1605.558 | 1605.697 ms | baseline |
| Hashed exact grid buckets | 1268.479, 1261.417, 1260.223, 1257.716, 1272.318 | 1261.417 ms | 21.44% faster |

Reproduce with `cargo bench --bench spatial_index_audit`. The benchmark is kept
separate from `parser_geometry_smoke` so parser, CSG, and report costs cannot hide
the bucket-storage effect. During validation the broad target exposed uncertified
mask offset-ring difference and intersection cases. Those helpers now retain the
expanded cover or subject island conservatively instead of aborting; that safety
hardening is not counted as a speedup.

## Reference-by-reference disposition

### Algorithms, geometry, data, and reporting

| Reference | Current target and disposition |
| --- | --- |
| Andrew, 2D convex hull | Directly implemented as the exact monotone-chain reduction in `checks::spread`; sorting and orientation predicates retain exact `Scalar` coordinates. |
| Apache Arrow columnar format | `arrow_report` writes the shared report model; it is an output adapter and does not justify duplicating analysis or geometry. |
| Apache Parquet | `parquet_report` serializes the Arrow-shaped report; compression/encoding belongs to the library and was not reimplemented. |
| Bentley, multidimensional binary search trees | The applicable associative-search idea is retained as private spatial broad phases. A k-d tree was not added alongside the existing conservative variable-radius grid because it would duplicate indexing without removing the exact narrow phase. |
| Ericson, *Real-Time Collision Detection* | AABB rejection, uniform-grid candidates, and exact narrow-phase replay are retained throughout `checks::spatial`, `distance`, and geometry checks. |
| Farin, CAGD | Bezier and arc construction is isolated in geometry/KiCad adapters; exact curve ownership is delegated to `hypercurve`. Flattened CAM checks do not reconstruct curves from polygons. |
| GitHub workflow commands | `github_annotations` emits CI annotations from the already-built report; no analysis pass is repeated per sink. |
| Hinnant, date algorithms | Date parsing and freshness evidence live in `date`/artifact checks. Date arithmetic is not a geometry hot path. |
| KiCad S-expression format | `sexp` and `kicad` retain parsed board intent and exact decimal scalars. Parser throughput is measured, but replacing the format model with an ad-hoc scanner was rejected as an evidence loss. |
| Lee and Preparata, computational geometry survey | Supports the separation of hull, distance, intersection, and spatial-search primitives. Those remain private helpers behind PCB-specific findings. |
| Lin and Canny, incremental distance | Temporal coherence is not present in a static release-package run. Cached simplex state would need a stable object lifecycle that the current immutable report pipeline does not expose, so it is deferred. |
| OASIS SARIF 2.1.0 | `sarif` maps stable findings into the standard. It consumes the shared report and does not rerun checks. |
| Parnas, module decomposition | Parser, policy, check, geometry, and report ownership are kept in separate modules; the spatial implementation remains private so public behavior is PCB-domain behavior. |
| Teschner et al., spatial hashing | Newly researched and directly tested. Exact tuple-key hashing for the existing regular grid is retained by the 21.44% result above; custom modulo tables and collision chains were unnecessary because `HashMap` already preserves key identity. |
| Toussaint, rotating calipers | Directly implemented after Andrew hull reduction in `checks::spread`, replacing all-pairs point-set diameter work while retaining exact distances. |
| Yap, exact geometric computation | Exact source decimals, `Scalar` decisions, certified comparisons, and named float projections define the proof boundary. The retained hash change only changes candidate storage. |

### PCB, assembly, electrical, and manufacturing research

| Reference | Current target and disposition |
| --- | --- |
| Areny et al., solder-paste transfer | Paste area/coverage and stencil readiness retain configurable evidence. Reflow physics is a process input, not inferred from flattened geometry. |
| Becerra et al., press-fit process | Press-fit keepout and package-handoff checks retain the relevant geometry and process declarations; force/compliance simulation is outside this reviewer. |
| Bhargava et al., buck-converter EMI layout | Switch-node, inductor, return-path, and power-converter geometry checks implement review prompts. Electromagnetic field solving is deferred to `hyperphysics` or external tools. |
| Black, electromigration | High-current width/neck and current-policy checks expose geometry risk. Lifetime prediction needs material, temperature, and current-density evidence not present in Gerber alone. |
| Chen and Lee, trapped no-clean flux | Paste/via exposure and production-artifact checks retain review evidence; chemical drying is not claimed from planar polygons. |
| Chesser and Porley, mixed-signal PCB layout | Mixed-signal partition, sensitive-net spacing, guard, and return-path checks are implemented as conservative layout prompts. |
| Cohn, shielded strip transmission line | The current impedance screen does not model shielded-strip geometry. Adding the equation without complete stackup/shield evidence would create false authority, so it is deferred. |
| Eurocircuits, tombstoning | Neighbor-pad and paste-imbalance checks directly cover asymmetric deposits while retaining configurable thresholds. |
| FixturFab, design for test | Testpoint diameter, spacing, accessibility, edge clearance, and IPC-D-356 coverage checks are implemented. Fixture mechanics remain external evidence. |
| Hammerstad and Jensen, microstrip | The quasi-static single-ended outer-microstrip estimate is directly implemented in `checks::impedance` with exact inputs and an explicit first-pass limitation. |
| Harter et al., stencil area shape/ratio | Stencil area-ratio, aspect-ratio, minimum aperture, and windowpane checks retain the applicable planar metrics. |
| Hollstein et al., QFN thermal analysis | Thermal-pad via, via distribution, copper-area, paste-windowpane, and mechanical keepout checks expose layout inputs; coupled simulation is not approximated. |
| Kirschning and Jansen, coupled microstrip | Frequency-dependent coupled-line equations require differential geometry, frequency, loss, and stackup detail beyond the current single-ended screen. Deferred rather than exposed as a universal rule. |
| Oezkoek et al., ENEPIG/ENEP bonding | Surface-finish compatibility and handoff evidence are checked; bond-process qualification remains a manufacturer decision. |
| Paterson and Tinker, type size | Silkscreen text-height readiness uses disconnected-island bounds only as a documented readability proxy because flattened Gerber has lost font/glyph semantics. |
| Chin and Ramakrishna, BGA escape traces | Dense-pad escape, via proximity, pad/via spacing, mask-web, and local-fiducial checks are implemented. Solder-joint simulation is deferred. |
| Jonnalagadda, via-in-pad fatigue | Via-in-pad exposure/plating-intent checks identify the configuration; fatigue life requires mechanical/process evidence and is not synthesized. |
| Lee et al., pulse-reverse copper plating | Copper density/balance and fabrication handoff checks expose plating-uniformity inputs. Process waveform and material-property simulation are external. |
| ST AN576, PCB layout and ESD | ESD protection distance, edge/keepout, and return-path checks implement conservative placement prompts. Surge waveform simulation is not inferred. |
| Sun et al., pattern-plating uniformity | Local copper-density windows and whole-layer balance retain the applicable geometric proxies. Multiphysics plating simulation is deferred. |
| Tang et al., fine-line wet etching | Minimum width, neck, acid-trap, and local-density checks expose etch-risk geometry with explicit fabricator thresholds. |
| Wilcoxon et al., QFN thermal-pad voiding | Thermal-pad paste windowpane, via exposure, and coverage checks retain the applicable stencil/layout evidence; void fraction is not predicted. |
| Wheeler, centered stripline | The zero-thickness centered single-ended stripline estimate is directly implemented and named as a first-pass readiness model. |
| Wong et al., small antennas | Antenna copper keepout, RF launch, and via-fence checks provide geometry review. Radiation efficiency requires a field solver. |
| Xu and Wang, guard trace ring | Sensitive-net guard/return proximity and mixed-signal checks implement placement evidence without claiming crosstalk magnitude. |

### Standards and interchange specifications

| Reference | Current target and disposition |
| --- | --- |
| IPC-2221B | Governs configurable clearance, stackup, conductor, and general board-design review; values stay policy inputs rather than hidden universal constants. |
| IPC-2152 | Current-carrying readiness uses declared current/width/stackup evidence. Thermal-current solving is deliberately not reconstructed from copper alone. |
| IPC-D-356B | `ipc356` parses electrical-test points, nets, diameters, access, and diagnostics; cross-source coverage and drill checks consume the retained report. |
| IPC-NC-349 | Excellon/router command and routed-slot readiness preserve parsing diagnostics and unsupported-command evidence. |
| IPC-7351B | Land-pattern, dense-pad, courtyard/component, paste, and assembly policies use configurable thresholds; package-specific qualification remains external. |
| IPC-2581C | Review/export companions retain manufacturing-description evidence. Full authoritative IPC-2581 import remains a documented boundary. |
| IEC 60352-5 | Press-in connection declarations and keepouts are reviewed; mechanical insertion qualification is deferred. |
| IEC 61000-4-5 | Surge protection placement/keepout evidence is reviewed; immunity testing is not replaced by geometry. |
| IEEE 828-2012 | Provenance, baselines, waivers, manifests, and generated-output freshness retain configuration/release evidence. |
| IPC-9797 | Automotive/high-reliability press-fit readiness consumes explicit profile and keepout policy; it does not certify the process. |
| IPC J-STD-001H | Soldering, polarity, paste, coating, and assembly-handoff checks expose missing evidence without asserting workmanship acceptance. |
| IPC-9252B | Bare-board electrical-test readiness is represented through IPC-D-356 coverage, net continuity, and package evidence. |
| IPC-7530 | Reflow-profile readiness remains artifact/process evidence. HyperDRC does not optimize or simulate an oven profile. |
| IPC-4552B | ENIG selection and compatibility are retained in surface-finish/handoff checks. Plating qualification is external. |
| IPC-6012D | Rigid-board capability and package-readiness thresholds remain declared fabricator policy, not baked-in geometry truth. |
| IPC-4556 | ENEPIG selection/bonding compatibility is reviewed from handoff evidence; deposition quality is not inferred. |
| Ucamco Gerber 2024.05 | Gerber/X2 metadata, polarity, units, roles, and parser diagnostics are retained; lossy polygon conversion stays explicit. |
| IPC-4553A | Immersion-silver selection and handoff compatibility are reviewed; process performance remains external. |
| IPC-7525B | Stencil area/width/aspect, paste coverage, windowpane, and tombstone heuristics are implemented with configurable policy. |

## Considered but not retained

- A second k-d tree alongside the regular grid would duplicate construction for
  checks whose radii and feature spans vary, while all candidates would still
  need exact replay. The measured hashed grid is simpler and faster on the
  representative sparse workload.
- Custom integer hash functions and fixed modulo tables from graphics-oriented
  spatial hashing were not retained. Exact tuple keys in the standard map avoid
  exposing table collisions and already deliver the measured gain.
- Incremental closest-feature caching from Lin-Canny was not introduced because
  independent static checks do not provide coherent successive configurations or
  stable simplex ownership.
- Frequency-dependent coupled-line, field, thermal, plating, fatigue, reflow,
  and electromagnetic models remain explicit delegated boundaries. Adding a
  partial formula without the referenced paper's required physical inputs would
  change a review prompt into an unsupported certification claim.

## Validation protocol

The retained path is covered by all spatial-index unit tests, all drill-spacing
tests, a hash-randomization ordering regression, the isolated release benchmark,
formatting, Clippy with warnings denied, all targets/features, and rustdoc.
`cargo test --all-targets --all-features` passed 1,082 library tests, both board
fixture benchmarks, the full debug parser/geometry smoke target, and the new
spatial benchmark. `cargo check --all-targets --all-features`, Clippy, generated
rustdoc, and `git diff --check` also passed. One executable Gerber-metadata
doctest could not be linked on this host: GNU `ld` repeatedly terminated with
`SIGBUS` while swap was saturated, including when run alone and with an alternate
linker request. Its parser assertions are independently covered by the passing
unit suite; this environmental linker failure is not counted as a passing test.
