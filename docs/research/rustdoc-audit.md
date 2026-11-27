# Repository Rustdoc audit

The requested scope is every line of every repository file, with accurate
Rustdoc for every public and private Rust API item. This audit is **in progress**;
no repository-wide coverage claim has been established.

Baseline: `8c22379b75982886697f607ffc09891f020794a2`, branch `rustdoc`.
`git ls-files` reports 4,492 tracked files, including 2,930 `*.rs` files at
baseline. These are inventory counts, not review or documentation coverage.
Nested workspaces, excluded packages, binaries, examples, tests, benches, fuzz
harnesses, macros, and configuration-dependent APIs remain in scope. Non-Rust
files must also be reviewed for their role in the codebase and documentation.

## Acceptance and review method

Owning layers retain their existing mathematical authority. This documentation
work follows P37 (accurate scope and API claims), with each batch naming the
specific properties whose contracts it explains. Missing or misleading Rustdoc
is the initial acceptance witness; source inspection and rendered private docs
are required, not merely a clean public `missing_docs` lint. Inventory and
linting cannot substitute for reading implementations and consumers.

Track whole-file reviews explicitly below; partial reads do not earn whole-file
credit. Review all declarations, including private fields and test helpers.
Check actual domains, failure paths, certainty, provenance, and resource limits.
Keep changes to documentation unless a separately admitted implementation fix
is needed. Check private-doc generation for affected configurations and inspect
rendered items. For this documentation-only session, the repository owner explicitly waived
full workflow checks on 26 November 2026. Use focused documentation and formatting
checks before committing and pushing; this exception does not change the standing
repository rules for later sessions or authorize implementation edits.

## Reviewed Rust files

| File | Review and change | Remaining validation |
| --- | --- | --- |
| `crates/csgrs-ffi/src/lib.rs` | Entire file; documented all direct/generated ABI operations, records/fields, family dispatch, private conversion/ownership helpers and regression tests. | All-target compilation and private docs with missing public docs/broken links denied passed; 176 C function pages inspected. Tests were compiled, not executed. |
| `benchmarks/size-harness/src/bin/primitive.rs` | Entire file; documented argument/default handling, timing/drop boundaries and count-only result. | Standalone all-feature check/private docs passed; workload not executed. |
| `benchmarks/size-harness/src/bin/composed.rs` | Entire file; corrected extrusion-only receipt description and documented fixed tool, bounds assertion, exports and timing. | Standalone all-feature check/private docs passed; workflow not executed. |
| `benchmarks/size-harness/src/bin/raster.rs` | Entire file; documented thresholded-cell model, pixel pattern, lifetime, defaults and native/WASM timer split. | Standalone all-feature check/private docs passed; raster not generated/executed. |
| `benchmarks/rust/kernel_comparison.rs` | Entire file; documented all helpers/local items, winding normalization, lossy fingerprints, setup/gating and actual timed work. | Compilation/private docs pass with curve added; manifest-declared minimal feature set fails on existing curve/torus imports. No benchmark executed. |
| `benchmarks/rust/solidean_comparison.rs` | Entire file; documented opt-in gates, timer boundaries, finite/tolerant mass checks and nominal empty expectation. | Minimal-feature compilation/private docs passed; acquisition and benchmark not run. |
| `benchmarks/support/solidean.rs` | Entire file; documented every instruction/field/helper, binary64 OBJ parsing, single-consumption execution and cache validation limits. | Benchmark compilation/private docs passed; tuple-field expansion adds only layout/trailing commas. No fixtures downloaded or executed. |
| `benchmarks/rust/profile_primitives.rs` | Entire file; documented constructor/path counts, receipt loss, finite projection and disjoint cubic/square union timing. | Feature-bundle compilation/private docs passed; benchmark not run. |
| `benchmarks/rust/feature_pipeline.rs` | Entire file; documented all helpers, nominal work units, immediate Boolean order, length-only cases and export/import timing boundaries. | Declared feature-bundle compilation/private docs passed; runtime catalog and numerical outcomes not executed. |
| `benchmarks/rust/adapter_graphics_export.rs` | Entire file; documented projected-buffer equality, native intermediate allocation, warmup order and timer/drop boundaries. | Minimal-feature compilation/private docs passed; benchmark unrun. |
| `benchmarks/rust/mesh_boolean_context.rs` | Entire file; documented historical failure/approximate outcome assertions, upper-median timing and per-policy storage rebuilding. | Minimal-feature compilation/private docs passed; assertions unchanged, runtime outcomes not reproduced. |
| `benchmarks/rust/mesh_pipeline.rs` | Entire file; documented six loops, length-only buffer row, override defaults, output destruction and receipt loss. | Minimal-feature compilation/private docs passed; benchmark unrun. |
| `tests/successive_operations.rs` | Entire file; documented observer ordering, every finite/ignored chain gate, rational row comparison and representation limits. | Focused compilation/private docs passed; all counts, graph rows and assertions unchanged and unrun. |
| `benchmarks/rust/successive_operations.rs` | Entire file; documented allocator methods/state, requested-byte limitations, snapshots, timing boundaries and raw journal lifecycle. | Focused compilation/private docs passed; allocator, file I/O and chain execution unchanged and unrun. |
| `tests/support/successive_operations.rs` | Entire file; documented step lifecycle, receipts/errors, timings, coordinate metrics, rational-box oracle and every chain factory. | Test/benchmark compilation and private docs passed; failure tuple layout adds only a trailing comma; runtime chains unchanged and unrun. |
| `tests/competitive.rs` | Entire file; documented tolerant three-engine comparison, quantized half-edge validation and input-only large-fixture gate. | Focused compilation/private docs passed; tests unchanged and unrun. |
| `benchmarks/support/competitive.rs` | Entire file; documented all adapters, records, fields, variants, projected measures, hull fallback and tolerance limits. | Test and benchmark consumers compile; private descriptions/fields render; no measurements executed. |
| `benchmarks/support/yeahright.rs` | Entire file; documented opt-in parsing, pinned archive/control checks, existence-only derived hull reuse and staging lifecycle. | Both benchmark consumers compile; private docs passed; no download/cache mutation executed. |
| `benchmarks/rust/competitive.rs` | Entire file; documented timed boundaries, setup/filter behavior, count measurements and optional acquisition. | Focused compilation/private docs passed with explicit dev-dependency metadata; benchmark not run. |
| `tests/generated_benchmark_corpus.rs` | Entire file; documented import/byte-stability checks, same-process scope and retained generated files. | Focused compilation/private docs passed with OBJ enabled; no fixture generation executed. |
| `benchmarks/support/generated_corpus.rs` | Entire file; documented voxel domains, foam contact filling, deterministic boundary order, local face constant and file lifecycle. | Test and benchmark consumers compile; private docs passed; no serialized assets generated. |
| `tests/enclosure_workflows.rs` | Entire file; documented wall/base and cut helpers, dyadic/collapsed/split offsets, OBJ continuation, exact measures and sampled round-plan limits. | Focused compilation/private docs passed with OBJ enabled; all assertions unchanged and unrun. |
| `tests/downstream_solids.rs` | Entire file; documented rotation equivalence, convex probes, required success matrices, authored planter parts and exact/enclosed measures. | Focused compilation/private docs passed; all assertions and ignored full workloads unchanged and unrun. |
| `tests/downstream_exact_inputs.rs` | Entire file; documented exact ingress, bounded integer line oracle, overlap payloads, full-circle support relations and all tests. | Focused compilation/private docs passed; tuple layout adds only trailing commas; assertions unchanged and unrun. |
| `tests/downstream_profiles.rs` | Entire file; documented parabola/offset/twist fixtures, area enclosures, sampled smoke bounds, side/cap oracles and continuation limits. | Focused compilation/private docs passed; comments corrected without changing formulas, expected blockers or assertions. |
| `tests/strict_boolean_workflows.rs` | Entire file; documented corpus module, nested generator macros, generated modules and all test definitions. | Focused compilation/private docs passed; 45 workflows and five controls remain unchanged and unrun. |
| `tests/strict_drilled_workflows.rs` | Entire file; documented polygonal drill construction, signed six-volume enclosure, membership and empty reuse assertions. | Focused compilation/private docs passed; corrected enclosure units in comments only; runtime assertions not run. |
| `tests/support/strict_boolean_workflows/mod.rs` | Entire file; documented truth tables, strict operations, check sequencing, 21-output chains and oracle controls. | Consumer compilation/private docs passed; call order, counters and assertions unchanged. |
| `tests/support/strict_boolean_workflows/fixtures.rs` | Entire file; documented rational box fields, nine relations, five forward/inverse maps and rational-normalization limits. | Consumer compilation/private docs passed; all private fields/methods rendered. |
| `tests/support/strict_boolean_workflows/oracle.rs` | Entire file; documented rational kernels, counters, source-plane domain, winding proof, retry bound and separate topology obligation. | Consumer compilation/private docs passed; exact arithmetic and ray ordering unchanged. |
| `tests/stl_import_contracts.rs` | Entire file; documented independent encoders, binary32 scale/row expectations, winding integrals, typed topology errors and continuation. | Focused compilation/private docs passed; assertions and encoded fixtures unchanged, not run. |
| `tests/sampled_sphere_strict.rs` | Entire file; documented private ball fields/helpers, finite oracle limits, exact local receipts, projected checks and timing scope. | Focused compilation/private docs passed; test thresholds and ignored workload unchanged, not run. |
| `tests/curve_metaball_failures.rs` | Entire file; documented result traits, validation categories, sampled diamond/ring witnesses and Boolean/extrusion continuation. | Focused compilation/private docs passed; assertions and source/grid fixtures unchanged, not run. |
| `tests/curve_metaball_workloads.rs` | Entire file; documented original renderer/benchmark inputs, loop-role, area and membership assertions. | Focused compilation/private docs passed; workload execution/timing not performed. |
| `tests/metaball_failures.rs` | Entire file; documented result traits, singular influence, normalized-field coordinates, bordered diagnostics and recovery. | Focused compilation/private docs passed; assertions unchanged, not run. |
| `tests/metaball_workloads.rs` | Entire file; documented exact float ingress, retained workload sizes, edge/volume and implementation-derived bounds checks. | Focused compilation/private docs passed; workload execution/timing not performed. |
| `tests/sdf_failures.rs` | Entire file; documented field/domain/branch failures, sample indices, capacity preflight, preview and diagnostic-on/off comparisons. | Focused compilation/private docs passed; assertions and callback order unchanged, not run. |
| `tests/native_boundary_failures.rs` | Entire file; documented typed polyhedron/point/graphics failures, exact normals, scalar projection, mutation and attributed cache witnesses. | Focused compilation/private docs passed with attributed cases enabled; runtime assertions unchanged, not run. |
| `tests/planar_operation_failures.rs` | Entire file; documented compatibility trait/members, homogeneous matrix admission, typed query failures, empty/holed continuation and adapter semantics. | Focused compilation/private docs passed; trait implementation docs rendered and assertions unchanged, not run. |
| `crates/hypermesh/src/construction.rs` | Entire file; documented private bit storage/flags, default/mutation semantics and missing-history precedence. | Owning private docs and downstream compilation passed; tuple-field formatting adds only a trailing comma. No geometry behavior changed. |
| `tests/empty_composition.rs` | Entire file; documented box/empty witnesses, typed-invalid matrices, qualified validation, batch roots and continued empty operations. | Minimal and curve-enabled compilation/private docs passed; assertions unchanged, not executed. |
| `tests/receipt_composition.rs` | Entire file; documented retained versus adopted controls, missing history, inherited blockers and map/and_then scope. | Minimal and curve-enabled compilation/private docs passed; assertions and proof-query order unchanged, not executed. |
| `tests/construction_evidence.rs` | Entire file; documented approximate hull fixture, derived paths, mutation/views, cache policies, workers and pipeline gate. | Minimal and curve/STL compilation/private docs passed, including the existing downstream pipeline. Runtime witnesses not executed. |
| `tests/containment_bounds.rs` | Entire file; documented rational near-plane queries, symbolic translation, unused bounds, mutation and qualification witnesses. | Focused compilation/private docs passed; geometry assertions unchanged, not run. |
| `tests/containment_inputs.rs` | Entire file; documented typed pre-shortcut validation, empty carriers, mutation, winding reversal, workers and qualified source facts. | Focused compilation/private docs passed; concurrency/geometry tests unchanged, not run. |
| `tests/containment_winding.rs` | Entire file; documented box descriptor/fields, independent octant oracle, all query/regularization matrices and local helpers. | Focused compilation/private docs passed; fields and 18 rendered declarations across containment files inspected. Test/local functions source-reviewed. |
| `tests/mesh_boolean_context.rs` | Entire file; documented dispatch helpers, projected-volume tolerances and strict/approximate receipt witnesses. | Focused compilation/private docs passed; all assertions unchanged, not run. |
| `tests/qualified_region_queries.rs` | Entire file; documented positive-cap collapse, delayed sign proof, inherited qualification and exact controls. | Focused compilation/private docs passed; all assertions and proof-query order unchanged, not run. |
| `tests/exact_linear_modeling.rs` | Entire file; documented rational hole/area/volume/coordinate witnesses, huge exact shift and adapter failure distinctions. | Focused compilation and private helper docs passed; geometry assertions unchanged, not run. |
| `tests/batch_common_cells.rs` | Entire file; documented 26-box construction, cavity invariants, shell concatenation/regularization and empty continuation. | Focused compilation and private helper docs passed; geometry assertions unchanged, not run. |
| `tests/hull_determinism.rs` | Entire file; documented ordered rows/evidence, source matching, cache warmups and temporary-file child protocol. | Focused compilation and private helper docs passed; hull and child-process geometry tests not run. |
| `tests/compound_rotations.rs` | Entire file; documented binary64/decimal angle families, no-prewarming order, independent cube-support oracle and continuation. | Focused compilation/private docs passed; assertions and predicate order unchanged, not run. |
| `tests/support/downstream_workloads.rs` | Entire file; documented all helpers including local disjoint-set root, exact comparisons/enclosures, flags, topology and triangle-row matching limits. | All five consuming test targets compiled; private helpers rendered through compound rotations. Function-local root source-reviewed. |
| `tests/adversarial.rs` | Entire file; documented exact float ingress, boundary-point witness, Boolean/transform assertions and optional offset extrusion. | Focused compilation passed with and without offset; private helper docs inspected. Geometry tests unchanged, not run. |
| `tests/adversarial_deep.rs` | Entire file; documented three-step box chain and sampled-circle union/extrusion at actual assertion strength. | Focused compilation and private docs passed; geometry tests unchanged, not run. |
| `tests/adversarial_extrusions.rs` | Entire file; documented face/edge predicate helper and segmented revolution, sweep and loft fixtures. | Focused compilation and private docs passed; geometry tests unchanged, not run. |
| `tests/adversarial_fixtures.rs` | Entire file; documented lexical label checks, deterministic path ordering and lack of geometric replay. | Focused compilation/private module docs passed; existing metadata test compiled directly and passed. |
| `tests/adversarial_stress.rs` | Entire file; documented eight-box fold and three sampled-profile extrusions without resource/completeness claims. | Focused compilation and private docs passed; geometry tests unchanged, not run. |
| `tests/adapter_boolean_failures.rs` | Entire file; documented operation variants, dispatch/context matrix, native wrapping and typed cause assertions. | Focused test compilation and private helper docs passed; assertions unchanged, not executed. |
| `tests/adapter_curve_failures.rs` | Entire file; documented failure transport, synthetic policy error, finite rectangle composition and revolution receipt/volume scope. | Focused test compilation and private helper docs passed; assertions unchanged, not executed. |
| `tests/scalar_certificate_failures.rs` | Entire file; documented inconsistent/undefined scalar wire fixtures, constructor guards and scalar source chains. | Focused test compilation and module docs passed; test-only items source-reviewed, not executed. |
| `tests/bounds_failures.rs` | Entire file; documented empty/invalid/undecided bounds, cache mutation, placement invariants and TPMS/sample consumers. | Focused test compilation and private helper docs passed; assertions unchanged, not executed. |
| `tests/storage_failures.rs` | Entire file; documented authored error classification/source transport without claiming allocator or budget exercise. | Focused test compilation and module docs passed; test-only item source-reviewed, not executed. |
| `examples/readme_renders.rs` | Entire file; documented fixtures, exact ingress/finite display, all constants/helpers/fields, raster math, endpoint markers, receipts and output behavior. | Private docs and focused example check passed with actual required features; 69 pages inspected. Missing-Hershey configuration reproduced; renderer/geometry unit witness not executed. |
| `tests/readme_render_regression.rs` | Entire file; documented lexical asset discovery, retained output directories, subprocess/environment behavior and byte-comparison scope. | Directly compiled existing tests and ran only the PNG-reference existence test: passed. Ignored renderer/byte-comparison test unchanged and not run. |
| `tests/readme.rs` | Entire file; documented embedded snapshots, exact quickstart synchronization and selected metadata/layout assertions. | Both existing tests compiled directly and passed after restoring README synchronization; assertions unchanged. |
| `examples/multi_format_export.rs` | Entire file; documented independently gated writers, output/truncation order, text statistics and all eight test intentions/limits. | Minimal and all-three-format private docs passed. Metadata-only test compilation reproduced eleven existing baseline/current errors; tests remain blocked and unchanged. |
| `examples/basic.rs` | Entire file; documented strict-session receipt scope, through opening, terminal STL and working-directory output. | Private example docs/rendering passed; assertions unchanged, example not run. |
| `examples/scalar_adapter.rs` | Entire file; documented adapter alias, exact finite ingress, projected bounds and fixture-specific equality assertions. | Private example docs/rendering passed; assertions unchanged, example not run. |
| `examples/basic_shapes.rs` | Entire file; documented sampled primitive scope, output constant, construction and writer failure behavior. | Private example docs/rendering passed; no STL files generated. |
| `examples/boolean_operations.rs` | Entire file; documented separate strict operations, sampled sphere, output files and absence of an independent set oracle. | Private example docs/rendering passed; operations not executed. |
| `examples/transformations.rs` | Entire file; documented independent transform demonstrations, degree angles, matrix order, exact numeric ingress and file replacement. | Private example docs/rendering passed; operations not executed. |
| `examples/adjacency_demo.rs` | Entire file; documented directed adjacency count, original-mesh smoothing runs and vertex-zero displacement assertion scope. | Private example docs/rendering passed; assertion unchanged, not rerun. |
| `examples/convex_hull.rs` | Entire file; documented union-position hull, sampled operand, sequential exports and failure behavior. | Private example docs/rendering passed; geometry not executed. |
| `examples/minkowski_sum.rs` | Entire file; documented represented-set sum versus analytic-ball offset and terminal export. | Private example docs/rendering passed; geometry not executed. |
| `examples/basic2d_shapes_and_offsetting.rs` | Entire file; documented policy/receipt extraction, sampled profiles, optional join conventions and flat triangulated exports. | Private example docs passed with and without `offset`; no modeling or exports run. |
| `examples/extrude.rs` | Entire file; documented sampled profiles/revolution/path, binary64 helix ingress, helpers and output behavior; corrected overbroad axis-side comment. | Private example docs/rendering passed; no modeling or exports run. |
| `benchmarks/rust/part_blueprint.rs` | Entire file; documented fixture construction, authored metadata, fixed timing boundary and edge-count limitations. | Formatting/comment-only comparison passed; benchmark source-reviewed, not executed or newly compiled. |
| `benchmarks/support/harness.rs` | Entire file; documented every setting, timing/selection method, field, measurement helper, CSV formatter and trace helper. | Standalone private Rustdoc passed with and without dispatch tracing; 13 rendered pages inspected. Benchmark workloads not run. |
| `fuzz/fuzz_targets/fuzz_part_blueprint.rs` | Entire file; documented exact fixture ingress, byte grammar, witness scope and existing short-record indexing gap. | Formatting/comment-only comparison passed; independent byte-chunk domain check executed. No fuzz runtime/compile result claimed. |
| `fuzz/fuzz_targets/support/mod.rs` | Entire file; documented fixed policy, assertion order, empty behavior, receipt extraction and limits of mesh validation. | Source/formatting/comment-only comparison passed; callers/kernel contracts inspected in context, fuzz validation not executed. |
| `src/parts/blueprint.rs` | Entire file; documented all box projection/splitting/ordering helpers, fields/variants, metadata gates, certainty limits and lossy SVG behavior. | Minimal and attributed/Hershey private docs passed; all rendered items inspected. Tests source-reviewed, unchanged and not rerun. |
| `src/parts/occlude.rs` | Entire file; documented index adjacency, dihedral tests, midpoint coverage, boundary pairing, omitted depth events, partial reports and evidence loss. | Private pages, tuple payloads and material contracts inspected; local rank/test helpers source-reviewed. No geometry behavior changed. |
| `src/parts/manual.rs` | Entire file; documented verbatim Markdown, callback/section order, table aggregation, anchor rules, private helpers and the unit witness. | Minimal and Hershey private docs passed; all rendered items/fields inspected. Test assertions unchanged, not rerun. |
| `src/parts/dimension.rs` | Entire file; documented component admission, public-field consistency, optional lossy labels, glyph placement, error atomicity and all helpers/tests. | Minimal label stub and Hershey private pages inspected; formatting and comment-only comparisons passed. Tests unchanged, not rerun. |
| `src/parts/mod.rs` | Entire file; documented module roles and authored metadata versus geometric evidence. | Minimal and attributed private docs passed; module page inspected. |
| `src/parts/scalar.rs` | Entire file; documented strict predicates, diagnostic-erasing comparisons, translation and unconstrained interpolation. | Private helper pages inspected; implementation unchanged. |
| `src/parts/metadata.rs` | Entire file; documented every record/payload, unchecked identity/geometry labels, installation admission and presence-only queries. | Private docs and payload rendering passed; no validation behavior changed. |
| `src/parts/attachment.rs` | Entire file; documented frame admission, normalized-axis mating, roll selection, private basis state and unit witnesses. | Private pages inspected; test bodies source-reviewed, not rerun. |
| `src/parts/assembly.rs` | Entire file; documented all fields, constructors, traversal, pose roots, explode propagation, partial blockers and count/tree assumptions. | Private pages inspected; test assertions unchanged, not rerun. |
| `src/parts/bom.rs` | Entire file; documented step traversal, keys/order, merge columns, global counts and zero/omission/saturation behavior. | Private pages inspected including final root-merge clarification; unit witness unchanged, not rerun. |
| `examples/assembly_manual.rs` | Entire file; documented stand-in geometry, exact placement/BOM assertions, drawing/manual output and fixed output directory. | Source and formatting checked; example not executed and no generated artifacts written. |
| `src/wasm/curve_js.rs` | Entire file; documented profile records, all modeling/query/constructor methods, outcome fields/conversions, finite arrays and legacy string-error boundaries. | Private docs with `wasm` passed; all eight module/type/helper pages and method summaries inspected. No executable changes. |
| `src/wasm/mesh_js.rs` | Entire file; documented raw-buffer admission, display normal/coordinate behavior, every modeling/constructor method, categorized errors, count limits and STL artifact ownership. | Private docs with `wasm` passed; all nine module/type/helper pages and method summaries inspected. No executable changes. |
| `src/wasm/mod.rs` | Entire file; documented modules, exact numeric ingress, lossy/NaN egress, policy selection and unvalidated native matrix wrapping. | Private WASM-feature docs passed; shared helper pages inspected. Test-only threshold remains source-reviewed. |
| `src/wasm/point_js.rs` | Entire file; documented storage, getters, display, Rust conversions and scope of the host test. | Private docs passed; type, fields and methods inspected. Test unchanged, not rerun. |
| `src/wasm/matrix_js.rs` | Entire file; documented positional row layout despite irregular argument labels, finite ingress, projection limits and native-copy witnesses. | Private docs passed; helper/type pages and methods inspected. Tests unchanged, not rerun. |
| `src/wasm/vector_js.rs` | Entire file; documented native arithmetic, normalization/rotation domains, strict decision order, flattened errors, finite egress and all tests/helpers. | Private docs passed; helper/type pages and methods inspected. Tests unchanged, not rerun. |
| `src/wasm/plane_js.rs` | Entire file; documented additive-offset convention, native orientation, validation order, local receipts, extraction loss and reflection. | Private docs passed; helper/type pages and methods inspected. No implementation changes. |
| `src/wasm/context_js.rs` | Entire file; documented all receipt conversions, fields/accessors, unresolved status preconditions, construction-summary scope and consuming extraction. | Private docs passed; all seven type/enum pages inspected, including private fields/methods. |
| `src/wasm/io_error_js.rs` | Entire file; documented TS union scope, property assignment assumptions, message-only causes and retained construction categories. | Private docs passed including optional DXF/Gerber arms; helper pages inspected. Macro-consumed TS constant source-reviewed. |
| `tests/wasm_certainty.rs` | Entire file; documented host-only receipt/evidence, successful boundary/empty and strict–approximate–strict query witnesses. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `src/voxels/mod.rs` | Entire one-line file; existing module description accurately identifies an empty placeholder for future experiments/hooks. No edit needed. | Module docs render; no implemented voxel API is claimed. |
| `src/io/gerber.rs` | Entire file; documented modal state, all fields/helpers, sampled construction, strict finite predicates, exposure ordering, unsupported/ignored commands and quantized export limits. | Private docs with `gerber-io` passed; 53 rendered pages inspected, including private methods/fields. No implementation changes. |
| `tests/rounded_rectangle_fidelity.rs` | Entire file; documented exact polygon areas/tangent vertices, clamp/control cases, composition/volume, policy retry, capacity and optional OBJ/Gerber witnesses. | Source, formatting and comment-only comparison passed; all assertions unchanged, not rerun. |
| `src/io/vrml.rs` | Entire file; documented public omission/nonempty contracts, every parser/helper/state/variant field, exact transform order, recursive ownership and all unit witnesses. Corrected aggregate-area evidence claims. | Private docs with `vrml-io` passed; all seventeen API/type/helper pages inspected, including private methods, fields and payloads. Tests unchanged, not rerun. |
| `tests/geometry_context.rs` | Entire file; documented all helpers, policy constants and witnesses across native/scalar/curve/sample/JS/interchange boundaries. Clarified the scope of count, area, finite-view and local-certainty assertions. | Source, formatting and documentation-only comparison passed; all gated test bodies unchanged and source-reviewed, not compiled or rerun as tests. |
| `src/io/gltf.rs` | Entire file; documented all private staging/cache fields, parse/layout/forest checks, transform arithmetic, scene capture, finite output limits and writer behavior. | Private docs with `gltf-io,obj-io,ply-io` passed; all 27 type/helper/public API pages inspected, including fields and constructor. |
| `tests/gltf_empty_composition.rs` | Entire file; documented independent GLB/accessor bytes, box expectations, empty/qualified reuse, malformed graphs/layouts, cache behavior and full-depth witness. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/interchange_failures.rs` | Entire file; documented native cause chains, independent projected rows, cache/retry/error witnesses and downstream empty/solid reuse. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `src/io/dxf.rs` | Entire file; documented timestamp scope, exact finite promotion, OCS basis, polygon preconditions, supported entity interpretation, projection/qualification and export index panics. | Private docs with `dxf-io,svg-io` passed; all ten constant/helper/public-function pages inspected. Unit witness unchanged, not rerun. |
| `src/io/svg.rs` | Entire file; documented error mapping, native options re-export, carrier tuple, strict replay, sampled standard primitives and exact-extension/display distinction. | Private docs with `dxf-io,svg-io` passed; three function pages and the options re-export page inspected. |
| `tests/svg_hidden_path_admission.rs` | Entire file; documented hidden grammar/extension failures, certified empty replay and downstream reuse. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_sampled_primitive_admission.rs` | Entire file; documented valid unselected emptiness versus invalid input and visible sampler underflow. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_chord_replay.rs` | Entire file; documented selected seventh-degree root, retained chord identity, midpoint incidence and repeated split replay. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_circle_replay.rs` | Entire file; documented selected-coordinate fillet family/contacts, interior incidence, reversal and byte stability. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_point_replay.rs` | Entire file; documented supporting-line intersection retention and subsequent exact subdivision/replay. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_parallel_replay.rs` | Entire file; documented rational helper and independent parabola normal-shift/incidence expectation. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_selected_replay.rs` | Entire file; documented retained source restriction, unused invalid selector and typed exhaustion/retry witnesses. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_polynomial_replay.rs` | Entire file; documented graph-byte helper, native error categories, vertical image/chart distinction and retained parabola subdivision. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/svg_region_replay.rs` | Entire file; documented all eight region witnesses and shared macro module, exact independent expectations, qualified empty rejection and fixture-specific offset identities. | Source, formatting and comment-only comparison passed; assertions and archived fixture unchanged, not rerun. |
| `src/io/stl.rs` | Entire file; documented byte-cache fields, artifact ownership/evidence, finite normal calculation, validation order, import qualifications and all unit witnesses. | Private docs with `stl-io,obj-io` passed; all eleven helper/type/export/import pages inspected, including private fields and artifact methods. |
| `tests/export_projection_contracts.rs` | Entire file; documented OBJ decimal replay, STL independent byte/normal expectations, projection collapse and cache/Boolean reuse. | Source, formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `tests/support/construction_evidence_pipeline.rs` | Entire file; documented rational/uncertainty/box helpers and all twelve policy/placement/cache pipeline cases. Clarified fresh-process selection and printed intent-volume evidence. | Source, formatting and comment-only comparison passed; tests unchanged, not rerun. |
| `src/io/mod.rs` | Entire file; documented every error payload, shared triangulation/projection/metadata helper, local formatter and test. | Gated and minimal private docs passed; error and seven helper pages inspected. No implementation change. |
| `src/io/amf.rs` | Entire file; documented unit labels, projected color bounds, serialization order, validation scope and writer behavior. | Gated private docs passed; all six function pages inspected. |
| `src/io/obj.rs` | Entire file; documented parsers, policy qualification, attributed corner selection, validation order and all unit witnesses. | Gated private docs passed including attributed readers; all eleven function pages inspected. Tests unchanged, not rerun. |
| `src/io/ply.rs` | Entire file; documented cache fields/lifetime/order, generated flat-normal rows, writer behavior and external-parser witness. | Gated private docs passed; cache type/state and both function pages inspected. Tests unchanged, not rerun. |
| `tests/amf_projection_contracts.rs` | Entire file; documented independent tag reader, exponent-family corpus, exact projection, tiny-cube and failure witnesses. | Source, formatting and documentation-only comparison passed; assertions unchanged, not rerun. |
| `tests/obj_export_failures.rs` | Entire file; documented native diagnostic checks, priority, writer preservation, exact fixtures and empty/solid reuse. | Source, formatting and documentation-only comparison passed; assertions unchanged, not rerun. |
| `tests/ply_projection_contracts.rs` | Entire file; documented external reader, independent normal/volume expectations, qualified replay and cache/error/composition witnesses. | Source, formatting and documentation-only comparison passed; assertions unchanged, not rerun. |
| `src/curve/image.rs` | Entire file; documented report/error payloads, exact grid identity, directed-edge traversal, turn selection, simplifier assumptions and all tests. Corrected the documented scope of the rectangle witness. | Private docs with image/text features passed; all eleven public/private pages inspected. Runtime tests unchanged, not rerun. |
| `src/curve/truetype.rs` | Entire file; documented table checks, blank-glyph proof cases, limited layout/kerning, every collector field/helper/callback and both unit witnesses. | Private docs with image/text features passed; all eight helper/type pages inspected, including private fields and trait overrides. Runtime tests unchanged, not rerun. |
| `tests/text_contracts.rs` | Entire file; documented helpers, gated modules, independent outline/placement expectations, errors, policy/scale/composition and qualified STL replay. | Source, formatting and documentation-only comparison passed; assertions and optional STL branch not rerun. |
| `tests/support/text_font.rs` | Entire file; documented synthetic glyph/table encoding, size assumptions, class-zero coverage and deliberate length corruption. | Source, formatting and documentation-only comparison passed; fixture bytes and behavior unchanged. |
| `src/curve/metaballs.rs` | Entire file; documented strict field construction versus policy-selected extraction, empty validation, indexed sample errors, checked buffer scope and axis preconditions. | Private docs passed with `curve,metaballs,image-io,truetype-text,hershey-text`; all three helper pages inspected. Runtime checks waived. |
| `tests/sweep_frames.rs` | Entire file; documented all 21 function/constant declarations, exact rational frame invariants, finite helix/grouping oracles, typed failures and terminal certainty. | Source, formatting and comment-only checks passed; runtime assertions and timing diagnostics unchanged, not rerun. |
| `tests/exact_profile_sweeps.rs` | Entire file; documented all 50 function/constant/type declarations, exact prism fixtures, signed revolution phases, independent winding/sector oracles, policy/composition and qualified STL replay. | Source, formatting and comment-only checks passed; assertions and optional STL branch unchanged, not rerun. |
| `tests/mixed_modeling_profiles.rs` | Entire file; documented numeric/contour fixtures, consumer dispatch, exact-member measures, mixed holes/spans and repeated thin-wall cuts. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/exact_path_failures.rs` | Entire file; documented control-net helpers, native causes, display-count invariance, reversal, exact area and empty-region witnesses. Clarified the limited diagnostic evidence in the historical symbolic-endpoint test. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/hilbert_trimming.rs` | Entire file; documented exact run helpers, axis-aligned simplification, source intervals/contact replay, policy transitions and algebraic trim reuse. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/gear_airfoil_failures.rs` | Entire file; documented numeric fixtures, profile/volume helpers and every mechanical-profile failure, certainty, composition and original-resolution witness. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/curve_constructor_failures.rs` | Entire file; documented adapter errors, degenerate/invalid companions, exact area/volume witnesses, policy transitions and byte-capacity failures. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/sampled_profile_failures.rs` | Entire file; documented compatibility trait, helpers, fixture macro and generated tests, error/area/composition witnesses and optional artifacts/interchange. | Source, formatting and comment-only checks passed; tests and optional serialization branches not executed. |
| `tests/slice_projection_contract.rs` | Entire file; documented the compatibility trait, helpers and all section/projection witnesses. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/minkowski_contracts.rs` | Entire file; documented helpers, topology/composition/policy/resource witnesses and limits of volume/probe evidence. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `src/curve/mod.rs` | Entire file; native and optional module exports reviewed; existing module Rustdoc is accurate. No edit needed. | Curve private-doc build passed; optional implementations remain separately in scope. |
| `src/curve/native.rs` | Entire file, cumulatively across the recorded constructor, path, meshing, revolution, sweep and final adapter/query/test batches. All 162 detected declarations have Rustdoc; enum variants and local items were included in their owning batches. | Private docs passed with `curve` and with `curve,metaballs,image-io,truetype-text,hershey-text`; affected public/private pages inspected. Unit/integration witnesses remain source-reviewed, not rerun. |
| `src/solid.rs` | Entire file, cumulatively across the recorded primitive, transform, sampling, distribution and final projection/loft batches; all private helpers, trait implementation methods and tests reviewed/documented. | Private docs passed in recorded minimal and gated configurations, including `curve,attributed,bevymesh`; source-only review of tests, runtime checks waived. |
| `tests/polyhedral_construction.rs` | Entire file; documented all helpers/witnesses and corrected the twisted-square fixture's rotation comment to 45 degrees. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `crates/hypermesh/src/mesh/concatenation.rs` | Entire file including tests; documented capacity helper, fixtures and proof/identity witnesses; existing public contract reviewed. | Hypermesh private docs passed; helper page inspected; test bodies unchanged, not rerun. |
| `tests/distribution_contracts.rs` | Entire file; documented every helper and admission, placement, certainty, capacity, multiplicity and cache witness. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/native_queries.rs` | Entire file; documented ray parameter, copy-count and hull-count expectations with their scope. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `src/implicit/sdf.rs` | Entire file; documented diagnostic fields, strict preview storage, indexed sampling and private grid helpers. | Private docs passed with `sdf`, `metaballs` and both; generated public/private contracts inspected. |
| `src/implicit/metaballs.rs` | Entire file; documented report fields, bordered sampling, empty behavior and private transfer/bounds helpers. | Private docs passed with `metaballs` and `sdf,metaballs`; generated pages inspected. |
| `src/implicit/tpms.rs` | Entire file including local extraction helpers/constants and tests; documented field formulas, band units, sampling and cache invariants. | Private docs passed with `sdf` and `sdf,metaballs`; local items/tests reviewed in source. |
| `src/implicit/mod.rs` | Entire file; feature gates and existing module Rustdoc agree with the three owning implementations. No edit needed. | Individual and combined feature doc builds passed. |
| `tests/metaballs_diagnostics.rs` | Entire file; documented all fixture helpers and diagnostic/influence/grid witnesses. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/sdf_tpms_diagnostics.rs` | Entire file; documented exact plane expectations, preview comparisons, downstream admission and diagnostic limitations. | Source, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/transform_failures.rs` | Entire file; documented every helper and transform failure/composition witness. | Source review, formatting and comment-only checks passed; assertions unchanged, not rerun. |
| `tests/primitive_failures.rs` | Entire file; documented all primitive-failure and empty-composition helpers/witnesses. | Source review, formatting and comment-only checks passed; test bodies unchanged, not rerun. |
| `src/solid/cache.rs` | Entire file; documented thread-local state, default implementation and local restoration guard; clarified receipt reuse. | Private docs passed with and without `curve`; local guard reviewed in source. |
| `src/solid/cache/storage.rs` | Entire file; documented all private keys, fields, retention helpers and completed-result contracts. | Private pages with `curve` checked; minimal build also passed. |
| `tests/solid_cache.rs` | Entire file; documented constants, comparison/uncertainty helpers and all cache regression witnesses. | Formatting and comment-only comparison passed; assertions unchanged, not rerun. |
| `src/attributed.rs` | Entire file including tests; documented alignment-only admission, equality, private fields/helpers and shared export cache. | Private docs passed with `attributed,curve`; generated sidecar page checked. |
| `src/hyper_math.rs` | Entire file; documented internal conversion implementations, strict predicate helpers and diagnostic-erasing convenience semantics. | Private docs passed with `attributed,curve` and no default features; generated private pages checked. |
| `src/solid_expr.rs` | Entire file; documented deferred operators, transform-dependent memoization, private lowering state, error paths and resource limits. | Private docs passed with `curve,dispatch-trace` and no default features; generated private/operator pages checked. |
| `tests/solid_expr.rs` | Entire file including dispatch and ignored timing witnesses; documented helpers and corrected independence claims. | Formatting and comment-only comparison passed; test bodies unchanged, not rerun. |
| `src/session.rs` | Entire file; documented conversion implementations/macro, private state, policy exceptions and receipt scope. | Private docs passed with and without `curve`; generated conversion docs checked. |
| `tests/session.rs` | Entire file; documented every test/helper according to actual assertions. | Comment-only comparison and formatting passed; tests source-reviewed, not rerun. |
| `tests/support/uncertainty.rs` | Entire file; existing module/helper Rustdoc matches retained witness and independent unresolved-zero assertion. No edits needed. | Source review only; no current execution claim. |
| `src/errors.rs` | Entire file and classification match arms; documented payload fields, categories and private classifiers. | Private docs passed with all error-gating features and no default features; formatting passed. |
| `src/lib.rs` | Entire file; checked feature overview and policy uses, added private API build guidance and private constant docs. | Crate page and private policy page checked; broader gated API review remains in owning modules. |
| `src/adapter/mesh.rs` | Entire file; documented scalar facade operations, graphics cache, exports and private helpers. | Private docs passed with and without `curve`, with broken links denied. |
| `src/adapter/curve.rs` | Entire file; documented region operations, sampled profile/revolution boundaries and private conversion helper. | Private docs passed with `curve`, with broken links denied. |
| `src/adapter/scalar.rs` | Entire file and conversion implementations; documented all declarations and native error distinctions. | Private docs passed with and without `curve`, with broken links denied. |
| `src/adapter/mod.rs` | Entire file including tests; documented bound conversion, fields and test intent. | Private docs passed with and without `curve`, with broken links denied. |
| `src/context.rs` | Entire file, including tests. Documented every previously undocumented private declaration; corrected approximate-policy wording and documented receipt loss at `into_value`. | Private docs passed with no default features and with `curve`; broader feature checks pending; full workflow checks waived for this docs-only session. |

### Context batch, 26 November 2026

Owner: CSGRS policy/receipt bridge, P8/P9/P19/P25/P30/P37. Domain: both predicate
policies, generic completed values, nested kernel receipts, and mesh dependency
qualifications. Missing witness at baseline: `GeometryDecisions`, its fields and
most methods, `GeometryOutcome::new`, and test declarations lacked Rustdoc.
`APPROXIMATE_512` said “certified topology” without explaining terminal consumption.

Read surrounding implementation in this file and inspected policy definitions in
`hyperlimit/src/predicate.rs`, construction evidence in
`hypermesh/src/construction.rs`, mesh evidence transport in
`hypermesh/src/mesh.rs`, `hypercurve::evaluate_under`, and Boolean/transform
consumers in `src/solid.rs`. Those supporting partial reads are not whole-file
audit credit. Also read `src/lib.rs`, `src/adapter/mod.rs`, and
`src/adapter/scalar.rs` to identify the next documentation gaps; their item-level
corrections and verification remain pending.

The diff changes comments only in Rust source. A comparison removing comment
lines confirms all non-comment lines in `src/context.rs` match baseline, so no
predicate ordering, runtime contract, dispatch, or representation change is
claimed. No performance or universal correctness claim is made.

Checks after edits:

- `cargo fmt --all -- --check`: passed.
- `git diff --check`: passed.
- Non-comment-line baseline comparison for `src/context.rs`: passed.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0 cargo doc --locked -p csgrs --no-default-features --document-private-items --no-deps --offline`: passed without warnings; log `/tmp/csgrs-rustdoc-doc.log`.

- The same documentation command with `--features curve` and
  `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`: passed without warnings;
  log `/tmp/csgrs-rustdoc-curve-doc.log`.
- Parsed generated HTML for `GeometryDecisions`, `GeometryContext`, and
  `GeometryOutcome`: confirmed private methods/fields and revised contract text
  appear in the pages.

Build artifacts are isolated in `/tmp/csgrs-rustdoc-target` for reuse and eventual
cleanup. No artifacts or raw logs are committed. Full pre-push workflow checks
were not run. The owner subsequently waived them for this docs-only session.

### Scalar adapter batch, 26 November 2026

Owner: CSGRS scalar boundary, P1/P3/P6/P25/P26/P29/P37. Domain: native `Real`,
finite/nonfinite primitive floats, signed integers, coordinate arrays, converted
bounds, and native error forwarding. Baseline for this batch: `782f2d9fc`.
Missing witness: most scalar declarations, trait members, implementation methods,
coordinate helpers, bounding-box fields and test helpers had no Rustdoc.

Read every line of `src/adapter/scalar.rs` and `src/adapter/mod.rs`. Cross-checked
native float import/export and rational-to-integer conversion in Hyperreal,
structural `exact_rational` recognition, modeling error variants, and mesh/curve
bound and graphics consumers. These supporting partial reads do not earn
whole-file credit. Documented exact binary float ingress, lossy egress, distinct
integer failures, preserved error sources, ordered component conversion, and
lack of outward-enclosure guarantees for rounded bounds. Test docs describe
actual assertions rather than inferring extra guarantees from test names.

Validation after edits:

- `git diff --check`: passed.
- Non-comment content comparison against the batch baseline: passed after
  normalizing whitespace and optional trailing tuple commas introduced to
  document variant fields. No executable behavior changed.
- Private Rustdoc with `curve` and broken links denied: passed without warnings.
  Command: `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`.
- Parsed generated HTML for the scalar trait, integer strategy, bounds type,
  private bound converter and private XY importer: expected contract text present.
- The same private-doc command without `--features curve`: passed without warnings.
- `cargo fmt --all -- --check`: passed.

Logs remain under `/tmp/csgrs-rustdoc-adapter-*.log`; reused artifacts remain under
`/tmp/csgrs-rustdoc-target`. Full workflow checks waived explicitly by the owner
for this documentation-only session; no runtime or performance claims are made.

### Mesh and curve adapter batch, 26 November 2026

Owner: CSGRS modeling and display adapters, P6/P8/P9/P19/P25/P26/P29/P37.
Baseline: `14ab38d45`. Domain: native mesh/region wrappers, primitive scalar
construction, Booleans, transforms, bounds, extrusion, sampled revolution,
finite profiles, graphics buffers and cached exports. Missing witness: most
facade methods and private storage/conversion declarations lacked Rustdoc;
context method summaries did not identify the operation or receipt scope.

Read every line of `src/adapter/mesh.rs` and `src/adapter/curve.rs`. Cross-checked
constructor validation, face triangulation, empty handling, transforms,
bounds, Boolean forwarding, finite-profile projection, extrusion and revolution
in the CSG grammar. Inspected Hypercurve's principal policy scope and Boolean
entry point, and Hypermesh's exact graphics export and normal construction.
Supporting reads are partial, not whole-file review credit.

Documented the distinction between primitive-scalar conversion and authoritative
native geometry, local predicate receipts and inherited qualification, winding
reversal and set complement, polygon triangulation and whole-mesh validation,
swept and regularized sampled revolution. The finite-profile path explicitly
projects to `f64` even with `RawReal`; converting those points back cannot recover
the original exact source. Bounds may be loose for algebraic trims and rounded
exports need not remain outward enclosures. The graphics cache documentation
specifies allocation/type keys, retained ownership, per-thread FIFO lifetime,
eight-entry capacity, failure handling, lack of a byte cap/reset, and the stable
conversion assumption for custom adapters.

Validation after final edits:

- Private Rustdoc with `curve`, broken links denied: passed without warnings.
  Command: `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`.
- The same command without `--features curve`: passed without warnings.
- Parsed generated HTML for both facade types, `RegionProfile`, private
  `finite2_to_scalar`, `point3_to_scalar`, `CachedGraphicsMesh`, and the
  macro-generated thread-local cache page: expected contract text present.
- Compared all nonblank, non-comment Rust lines against the batch baseline:
  identical in both files. No executable changes or performance claims.
- `git diff --check`: passed after removing trailing whitespace in new docs.
- Final `cargo fmt --all -- --check`: passed.

Logs: `/tmp/csgrs-rustdoc-mesh-curve-doc.log` and
`/tmp/csgrs-rustdoc-mesh-minimal-doc.log`. Reused build tree remains about 97 MiB;
no generated artifacts are tracked. Full workflow checks remain waived by the
owner for this documentation-only session.

### Error API and crate root batch, 26 November 2026

Owner: CSGRS error and binding boundary, P6/P8/P30/P37. Baseline: `e5e8487f1`.
Domain: every modeling error variant and classification branch, including optional
font and sampled-field diagnostics. Missing witness: variant payload fields,
most category variants, `kind`, and both private classifiers lacked Rustdoc.

Read every line of `src/errors.rs` and `src/lib.rs`. Inspected session aliases,
JavaScript category export, expression and scalar wrappers, allocation helpers,
curve diagnostics, and the installed thiserror source-chain expansion. Checked
crate-root feature declarations against Cargo configuration and boundary module
exports. Supporting files are not credited as fully audited. `src/session.rs`
has only a targeted alias-doc correction; its full review remains pending.

Documented actual classification semantics, including the coarse resource bucket
that currently includes cancellation, capacity overflow and several allocation
failures. `ResourceUnavailable` is used for explicit curve storage uncertainty,
not every failed allocation. Documented that transparent scalar wrappers forward
the payload's source chain and polynomial topology retains its payload without
exposing it as `Error::source()`. Corrected the session error-alias claim to agree.
These are existing implementation limits, not algorithm fixes or claims of
compliance with the stronger failure/resource target contract. Future changes to
classification or source-chain behavior require separate admission.

Crate-root docs now describe the benchmark feature bundle, private-documentation
command, strict helper policy, and test-only mesh context. Test-only constant
documentation was reviewed in source; ordinary Rustdoc builds exclude test cfg.

Validation after edits:

- `git diff --check`: passed.
- Code comparison against baseline, removing comments and normalizing whitespace
  and optional trailing record/tuple commas: unchanged for all three Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,truetype-text,hershey-text,sdf,metaballs
  --document-private-items --no-deps --offline`: passed without warnings.
  This enables every feature-gated variant in `src/errors.rs`.
- Generated HTML checked for error payloads/categories, both private classifier
  pages, private policy constant, root build guidance, and session error alias.
- The same private-doc command without the feature list: passed without warnings.
- `cargo fmt --all -- --check`: initial failure required splitting attributes onto
  their own lines beside documented tuple fields; final check passed after that
  formatting correction.

Logs remain in `/tmp/csgrs-rustdoc-errors-doc.log` and
`/tmp/csgrs-rustdoc-errors-minimal-doc.log`. Source behavior, category mapping and
error-source attributes are unchanged. Full workflow checks remain waived for
this docs-only session; no runtime or performance claim is made.

### Modeling session batch, 26 November 2026

Owner: CSGRS input conversion and receipt orchestration, P1/P6/P8/P9/P19/P25/P37.
Baseline: `8ca463916`. Domain: fixed-width integers, exact native scalars, finite
float imports, rational/scientific text, both policies, successful and failed
callbacks, native mesh/curve operations, and external receipts. Missing witness:
conversion implementations and macro-generated methods, private fields, and
regression helpers lacked Rustdoc. Existing session summaries claimed receipts
for every operation and policy selection once without identifying strict solid
constructor exceptions. Extrusion documentation described only +Z although signed
nonzero heights are accepted.

Read every line of `src/session.rs`, `tests/session.rs`, and
`tests/support/uncertainty.rs`. Inspected native scalar conversion/parsing,
strict primitive construction, reflection, extrusion, and expression lowering
and evaluation. Those supporting partial reads do not earn whole-file credit.
The uncertainty helper was already documented consistently with its assertions;
no change was needed. Existing tests compare wrapper outputs, receipt transitions,
input rational forms and typed errors; their source review is not independent
geometric completion evidence or a claim of execution on this revision.

Documented that `run` records only successful returned receipts; prior records
survive a later error. `record` trusts external receipts without policy checking
or value inspection. Native modeling paths carry mesh history separately, but
generic callbacks are responsible for their values' evidence. The four solid
constructors use strict grammar functions directly and do not update the session
accumulator. The docs identify this current limitation without modifying policy
behavior. Sampling fidelity, callback success, and accumulated numerical certainty
remain separate claims. Expression evaluation docs distinguish a shared nonempty
arrangement from the all-empty shortcut and transform-dependent subtree reuse.

Validation after edits:

- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated session module, `Csg`, `IntoReal`, private integer macro, and decimal
  parser pages contain the new contracts; macro-generated implementation docs
  appear on the trait page.
- `cargo fmt --all -- --check`: passed. After the last module wording edit,
  `rustfmt --check --edition 2024 src/session.rs tests/session.rs` also passed.
- `git diff --check`: passed. Nonblank, non-comment Rust lines match the baseline
  in both edited source files. Tests and example bodies remain unchanged.

Logs: `/tmp/csgrs-rustdoc-session-doc.log` and
`/tmp/csgrs-rustdoc-session-minimal-doc.log`. Reused build tree is about 124 MiB.
Full workflow/runtime checks remain waived for this docs-only session. Test-only
Rustdoc was reviewed in source, not claimed as rendered by the library doc build.

### Lazy expression batch, 26 November 2026

Owner: CSGRS expression lowering into the Hypermesh Boolean program,
P6/P8/P9/P19/P20/P21/P25/P37. Baseline: `48d00109e`. Domain: native mesh leaves,
shared subtrees, ordered transform chains, all four Boolean operators, canonical
empty and malformed leaves, both policies and located/unlocated failures.
Missing witness: private node variants/payloads, lowering state, path storage,
helpers, operator macro and generated members lacked Rustdoc. Existing summaries
omitted transform-dependent reuse and the all-empty no-arrangement shortcut.

Read every line of `src/solid_expr.rs` and `tests/solid_expr.rs`. Cross-checked
native transform admission and receipt handling, the kernel Boolean entry point,
and existing session/uncertainty support reviewed in prior batches. Supporting
kernel/grammar reads are not full-file audit credit.

Documented allocation identity versus set equality, memo lifetime and keys,
inner-to-outer transform application, left-first lowering, false-node reuse,
ancestry retention, and first-occurrence error paths. The affine distribution
identity is stated with its preconditions; projective leaf admission may fail
before later Boolean selection. Recorded the existing recursive/allocating
lowerer and its `u32` indexing panics without implying typed exhaustion or an
expression-level resource bound. No algorithm, predicate order or bound changed.

Documented each regression helper and test's actual role. Corrected the claim
that eager/lazy comparisons supply an independent set oracle: both paths and the
symmetric-difference helper use the native kernel. Matching absent rational normal
forms is not a volume proof. The drilled fixture has a separate closed-form
volume expectation; the dispatch fixture asserts one versus four starts. The
ignored timing fixture reports five-run median/min/max without a cache reset or
performance threshold. These are source observations, not new execution or
performance claims.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid_expr.rs tests/solid_expr.rs`: passed.
- `git diff --check`: passed.
- Compared edited Rust against baseline after removing comment lines and
  normalizing whitespace and optional trailing commas: executable content
  unchanged, including every assertion and ignored/feature gate.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,dispatch-trace --document-private-items
  --no-deps --offline`: passed without warnings. The same command without the
  feature list passed without warnings.
- Generated module, public expression, private node/transform/program/location,
  dispatch helper and operator macro pages contain the new contracts. Generated
  owned-operator method documentation appears on the public expression page.

Logs: `/tmp/csgrs-rustdoc-expressions-doc.log` and
`/tmp/csgrs-rustdoc-expressions-minimal-doc.log`; artifacts remain in the reused
`/tmp/csgrs-rustdoc-target`, with no generated output tracked.

Full workflow/runtime checks remain waived by the owner for this docs-only
session. Test-only Rustdoc is reviewed in source; a library documentation build
does not render the integration tests or execute their assertions.

### Attribute sidecar and scalar helpers batch, 26 November 2026

Owner: CSGRS boundary attributes and internal scalar conveniences,
P1/P3/P4/P6/P8/P19/P25/P26/P29/P37. Baseline: `4faf22d51`. Domain: aligned
metadata, optional authored normals, successful/failed/concurrent graphics
exports, native and primitive scalars, strict predicates and exact vector/point
construction. Missing witness: sidecar fields, equality, uncached exporter and
tests, plus every internal scalar-helper declaration, lacked Rustdoc. Existing
sidecar summaries did not distinguish count validation from geometric validity.

Read every line of `src/attributed.rs` and `src/hyper_math.rs`. Inspected native
mesh equality, point/vector conversion and interpolation, exact GPU row assembly,
OBJ sidecar import, helper call sites, and cache recovery/concurrency/authored-normal
regression witnesses in `tests/native_boundary_failures.rs`. Those partial reads
are not whole-file review credit or claims that the tests were run.

Documented unchanged geometry/attribute ownership, cache sharing by clone and
metadata mapping, first-success publication, retry and racing-success behavior,
lifetime, lack of eviction/reset/byte cap, and separation from modeling authority.
Authored normals are copied without normalization or degeneracy checks; geometry
validity is not implied by successful authored-row export. Equality ignores
construction evidence and the export cache, following native buffer equality.

Internal scalar docs distinguish exact ingress from lossy export, the private
conversion trait from the public session trait, and strict positive evidence from
absence of proof. `hreal_gt_f64` and triangle nondegeneracy return false for
undecided cases; comparison and arithmetic helpers erase particular diagnostics
into `None`; several wrappers always return `Some`. Their documentation now
states this without changing behavior or presenting the wrappers as complete
failure/provenance-preserving APIs. The test-only threshold is the exact value
of a binary float, not an exact decimal or modeling coincidence rule.

Validation after edits:

- `rustfmt --check --edition 2024 src/attributed.rs src/hyper_math.rs`: passed.
- `git diff --check`: passed.
- Nonblank, non-comment line comparison against baseline: identical for both
  edited Rust files, including test assertions and cache control flow.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features attributed,curve --document-private-items
  --no-deps --offline`: passed without warnings.
- Generated attributed sidecar page includes private state and exporter docs;
  private scalar module, conversion trait and representative predicate/arithmetic
  helper pages contain the revised contracts.
- The same private-doc command without the feature list: passed without warnings.

Logs remain at `/tmp/csgrs-rustdoc-attributes-math-doc.log` and
`/tmp/csgrs-rustdoc-math-minimal-doc.log`. Test-only helper documentation was
reviewed in source. Full runtime/workflow checks remain waived for this docs-only
session; no correctness or performance result beyond documentation validation is
claimed.

### Modeling cache batch, 26 November 2026

Owner: CSGRS completed-operation retention, P4/P5/P8/P9/P19/P20/P25/P37.
Baseline: `281c53cd0`. Domain: strict primitive keys, ordered merges, distribution
layouts, curve projections/slices, exact policy keys, scoped entry limits,
thread-local ownership, mutation/evidence changes and repeated requests. Missing
witness: private storage variants and fields, matching/retention helpers, local
scope guard, and test declarations lacked Rustdoc.

Read every line of `src/solid/cache.rs`, `src/solid/cache/storage.rs`, and
`tests/solid_cache.rs`. Cross-checked primitive/distribution key construction and
retention call sites in `src/solid.rs`, including early successful empty returns
that need not insert an entry. Those caller reads are partial and do not earn
whole-file credit.

Documented structural parameter equality, owned allocation identities and
construction snapshots, exact context matching, first matching FIFO lookup,
nonrefreshing hits, entry lifetimes, active insertion policy and no error caching.
Private insertion helpers trust caller-completed outcomes and do not revalidate
geometry. Zero limits skip entry construction; policy shrinking trims oldest
entries, with vector capacity released at zero and otherwise available for reuse.
No limit is a retained-byte or total-work budget. A scope guard restores prior
limits during unwinding, without resurrecting evicted entries. A cache hit
returns its stored receipt; rebuilding can use stronger scalar proofs acquired
since insertion, consistent with P4's distinction between proof reuse and value
semantics. This clarification does not authorize weaker policies or altered values.

Test docs describe row/receipt comparisons separately from construction ancestry
and set equivalence. Witnesses cover FIFO, nested unwinding, workers, source order
and mutation, policy separation, and all enabled cache categories. Source review
is not a claim of executing these tests or proving universal cache correctness.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid/cache.rs src/solid/cache/storage.rs
  tests/solid_cache.rs`: passed.
- `git diff --check`: passed.
- Baseline comparison after removing comment lines and normalizing whitespace
  and optional trailing commas: code identical in all three edited Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings.
- Generated private key enums, mesh storage, cache state, retention/lookup helpers,
  thread-local state and public policy scope pages contain the new contracts.
  Function-local guard and integration-test docs were reviewed in source rather
  than claimed as rendered by the library build.
- The same private-doc command without `--features curve`: passed without warnings.

Logs: `/tmp/csgrs-rustdoc-cache-doc.log` and
`/tmp/csgrs-rustdoc-cache-minimal-doc.log`; existing temporary build storage is
reused. Full workflow/runtime checks remain waived for this docs-only session.
No predicate ordering, cache key, capacity, test assertion or source behavior was
changed, and no new runtime/performance claim is made.

### Solid primitives, partial-file batch, 26 November 2026

Owner: CSGRS native primitive grammar, P1/P6/P17/P18/P19/P20/P25/P26/P37.
Baseline: `4a75b574b`. **Partial file:** reviewed `src/solid.rs` from its module
header through the end of `torus_uncached` (baseline lines 1–869). Whole-file
credit remains open; resume at `teardrop_cylinder`. Read every line of
`tests/primitive_failures.rs`. Inspected native orthonormal-frame construction;
that supporting read does not earn full kernel-file credit.

Missing witness: private strict sign, parameter, capacity, reservation, sampled
circle, indexed assembly and uncached primitive helpers lacked Rustdoc. Public
primitive summaries omitted important domains, sample conventions and failure
rules. Documented all declarations in this reviewed section, including source
validation before empty-collapse shortcuts, checked inline bytes versus nested
allocation cost, and `indexed_convex`'s absence of convexity validation.

Specified sphere Y-axis poles and exact angular samples, signed cylinder/frustum
height, cone tips, octahedron/icosahedron circumradius, arrow proportions and
mirrored-tip placement, and the sampled ring-torus domain. Exact coordinates,
polyhedral geometry, sampling history and analytic fidelity remain distinct.
Arrow orientation propagation is described as its actual construction sequence,
not a general topology certificate; secondary adjacency allocations remain
outside the primary reservation checks. No new resource guarantee is asserted.

Regression documentation describes negative/undecided input, resolution/capacity,
zero-size validation, typed adapter failures, signed-height volume, cache recovery
and further operations on canonical empties. Their source inspection establishes
what is asserted, not that this revision passed runtime tests or proves closure.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid.rs tests/primitive_failures.rs`: passed.
- `git diff --check`: passed; every nonblank, non-comment source line matches
  baseline in both edited files, including all assertions and orientation logic.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated sphere, cylinder, frustum, arrow, torus, private capacity, sample-table,
  indexed-convex and uncached icosahedron pages contain the revised contracts.

Logs: `/tmp/csgrs-rustdoc-primitives-doc.log` and
`/tmp/csgrs-rustdoc-primitives-minimal-doc.log`. Full workflow/runtime checks remain
waived for this docs-only session. No executable behavior or performance claim
changed; test-only docs were reviewed in source.

### Solid construction and transforms, partial-file batch, 26 November 2026

Owner: CSGRS native solid grammar and its Hypertri/Hypermesh policy bridges,
P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `f85b58f70`.
**Partial file:** reviewed `src/solid.rs` from `teardrop_cylinder` through `inverse`
(baseline lines 995–1528), including each body, guard and receipt path. Combined
with the previous batch, review reaches the sampled-field options boundary;
whole-file credit remains open. Read every line of `tests/transform_failures.rs`.
Supporting reads covered the relevant curve constructors/extruders, Hypertri
simple-ring contract, Hypermesh transforms, reflection, subdivision, finite
materialization, winding reversal and indexed edge check. Those reads do not
earn whole-file credit for their owning modules.

Missing witnesses: private planar projection/triangulation and orientation-aware
transform helpers lacked Rustdoc; several public summaries overstated or omitted
their actual behavior. `renormalized` clones the mesh without coordinate rebuild.
`materialize_finite` performs lossy binary64 projection before exact promotion,
retaining coordinate-projection provenance without an error or topology bound.
`is_closed_manifold` checks indexed edge pairing, without geometric tests or
vertex-link validation. Documented these limitations without changing behavior.

Documented teardrop placement and sampled profile, gear profile delegation,
signed nonzero extrusion thickness, and the use of `helix_angle_degrees` as total
twist rather than conversion from a tooth helix angle. Polyhedron docs distinguish
per-face admission from closed-solid admission, distinct closing indices from
repeated coordinates, and canonical empty geometry from position-only buffers.
Projection docs distinguish policy classifications from proofs and retain the
support-search/decision order. Boolean docs explain the two-empty shortcut and
inherited qualifications; transform docs specify determinant checks, index
validation, per-triangle projective scale signs, infinity failures, Euler order,
reflection and winding correction. Subdivision docs describe exact shared indexed
midpoints and the absence of a total resource budget.

Transform witness documentation describes actual independent coordinate/winding
and volume expectations, policy retry, typed error/source preservation, valid
empty composition and invalidated indices. No test assertion was changed and no
runtime execution or universal correctness claim follows from this source review.

Validation after edits:

- `rustfmt --check --edition 2024 src/solid.rs tests/transform_failures.rs`: passed.
- `git diff --check`: passed. Every nonblank, non-comment source line matches
  baseline in both Rust files.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps --offline`:
  passed without warnings. The same command without `--features curve` passed.
- Generated teardrop, helical gear, polyhedron, transform, normalization,
  materialization and edge-check pages contain the revised contracts. Private
  triangulation, projection-axis, ring-projection and orientation-helper pages
  also contain the new Rustdoc. Integration-test docs were reviewed in source.

Logs: `/tmp/csgrs-rustdoc-transforms-doc.log` and
`/tmp/csgrs-rustdoc-transforms-minimal-doc.log`. The existing temporary build
directory is reused. Full workflow/runtime checks remain waived for this docs-only
session. No executable behavior, predicate order or performance claim changed.

### Sampled fields and bounds, partial-solid/full-implicit batch, 26 November 2026

Owner: CSGRS sampled implicit constructors and their Hypermesh/Hypersdf bridges,
P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `6beb18849`.
**Partial solid file:** reviewed `src/solid.rs` from `SampleOptions` through
`float_with_context`, including feature gates and each wrapper body. Resume at
`merge`; whole-file credit remains open. Read every line of `src/implicit/mod.rs`,
`sdf.rs`, `metaballs.rs`, `tpms.rs`, `tests/metaballs_diagnostics.rs` and
`tests/sdf_tpms_diagnostics.rs`. Supporting reads of Hypermesh bounds/Surface Nets,
Hypersdf metaball fields and selected failure-test sections do not earn whole-file
credit for those modules.

Missing witnesses: diagnostic fields, sampler/grid/preview helpers, TPMS caches,
local tetrahedral helpers/constants and test declarations lacked Rustdoc. Public
summaries blurred policy-selected counts with strict preview proofs and omitted
grid-border/empty rules. TPMS prose asserted mathematical surface properties not
established by its trigonometric formulas or implementation. Replaced those claims
with the modeled formulas and extraction behavior, without changing any algorithm
or reducing the repository's target contract.

Documented exact sample placement, callback order, unshifted extrema versus
iso-shifted sign buckets, and the possibility that diagnostic comparisons fail
or consume additional approximate certainty. Expression previews require strict
signs independently of meshing policy, retain unavailable values explicitly,
and use clamped F32 magnitudes stored as f64 without driving native geometry.
Compatibility counters remain zero on successful native extraction; detected
degeneracy and undecided samples return failures instead of partial output.

Metaball docs specify requested/bordered count validation, conservative bounds,
one-cell exterior samples, strict field evaluation and empty-source reports.
TPMS docs specify positive wavelength, field-value band thickness (not Euclidean
distance), bounding-box clipping, half-step exterior sampling, six-tetrahedron
extraction, negative/zero ownership, shared edge crossings and orientation errors.
Per-call trigonometric caches require a fixed coordinate/scale pair per index.
Neither Surface Nets nor the tetrahedral path is documented as independently
certifying continuous-field topology, exact minimality or embedded closed output.
Primary count/reservation checks are not presented as total resource bounds.

Bounds/placement docs cover unused position rows, index validation, certified
cache reuse, local receipt scope and the absence of source construction history
in a bounds value. Centering uses the box midpoint; floating uses minimum Z.
Diagnostic test docs distinguish independently expected rational planes/counts
from shared-path agreement, indexed closure and lossy volume-sign diagnostics.

Validation after edits:

- Targeted `rustfmt --check --edition 2024` on all six edited Rust files: passed.
- `git diff --check`: passed; every nonblank, non-comment source line matches
  baseline across all six files, including tests and local helper bodies.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features sdf,metaballs --document-private-items
  --no-deps --offline`: passed without warnings. Separate `sdf`, `metaballs`,
  and no-feature private-doc builds also passed.
- Generated diagnostic structs/fields, preview/grid storage, sampler, border,
  tetrahedral extraction, TPMS cache, public options/constructors and bounds
  contracts were inspected. Function-local items and test docs were reviewed in
  source; ordinary library Rustdoc does not render them as standalone items.

Logs use the prefix `/tmp/csgrs-rustdoc-sampling-` and suffix `-doc.log`, with
configuration names `sdf`, `metaballs`, `sdf,metaballs` and `minimal` between them.
The shared temporary build directory is about 150 MiB. Full workflow/runtime checks
remain waived for this docs-only session. No executable behavior, predicate order,
test assertion or performance claim changed. A generated-text check was adjusted
for Rustdoc's smart apostrophe; combined pages were regenerated after the minimal
build removed feature-gated output from the shared directory.

### Distribution, queries and concatenation batch, 26 November 2026

Owners: CSGRS distribution/query grammar, Hypermesh structural concatenation, and
the Hyperphysics mass-report boundary; P1/P4/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37.
Baseline: `55a0f4abb`. **Partial solid file:** reviewed `src/solid.rs` from `merge`
through `exact_mass_properties`, including every body and fallback/receipt path.
Resume at `flatten`; later sections were encountered but have not earned audit
credit. Read every line of `crates/hypermesh/src/mesh/concatenation.rs`, including
unit tests, and `tests/distribution_contracts.rs` and `tests/native_queries.rs`.
Supporting reads covered native mesh queries, ray/triangle classification, vector
normalization/angles, Hyperphysics integration/admission and distribution caches;
those owning files remain outside whole-file credit.

Missing witnesses: the distribution admission, buffer and union helpers lacked
Rustdoc, as did the native concatenation capacity helper and test declarations.
Public summaries omitted important placement, fallback and query/error semantics.
Documented row/column order, signed normalized linear spacing, global-Z arc
rotation, angular midpoint for one copy and unreduced sweeps. Canonical-empty and
zero-copy paths still validate their applicable source/direction requirements;
copy-capacity checks happen only after those shortcuts. Cache keys/receipts and
inherited mesh qualifications are described without asserting a total resource
bound.

Disjoint bounds permit structural concatenation, retaining source winding
multiplicity instead of normalizing each boundary. Failed/missing bounds and
undecided overlap hints trigger ordered Boolean fallback; successful hint receipts
already consumed remain in aggregate certainty. Concatenation preserves all rows,
validates indices before rebasing and transfers current closed-PWN facts only when
all nonempty operands supply one. The helper's inline-byte/count checks do not
bound nested scalar or later allocation costs.

Query docs specify ray parameterization, coplanar omission, sorted/deduplicated
contacts, per-segment polyline order and the short-polyline validation shortcut.
Dihedral measurement uses supplied indexed normals without adjacency validation;
its convenience form erases native failures into `None`. Mass-property docs now
state the caller's closed/oriented-geometry obligation, signed integration and
orientation normalization, absent source qualifications and conversion of physics
errors into debug-text `ValidationError::Geometry`. Exact scalar arithmetic alone
does not establish physical validity or preserve typed native error sources.

The first Hypermesh private-doc build reproduced two ambiguous `boolean` links
at this revision, in `src/lib.rs` and `src/winding.rs`. Inspected their surrounding
API context and disambiguated both as function links with `()`. Those targeted
repairs do not establish full-file semantic review of either file. Read the whole
`crates/hypermesh/AGENTS.md`; its full pre-push checks are covered by the owner's
session-specific docs-only waiver, with focused checks used here.

Validation after edits:

- `rustfmt --check --edition 2024 --config skip_children=true` on all six edited
  Rust files: passed. `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in all six Rust files,
  including test assertions, predicate order, fallback branches and cache behavior.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p hypermesh
  --document-private-items --no-deps --offline`: passed without warnings after the
  two function-link repairs.
- Equivalent private-doc builds for `-p csgrs --no-default-features`, with and
  without `--features curve`: passed without warnings.
- Inspected twelve generated public/private contracts for merge, distribution
  helpers/layouts, queries, mass properties and native capacity checking. Both
  repaired links resolve to function pages. Unit/integration-test Rustdoc was
  reviewed in source, not claimed as rendered by ordinary library builds.

Logs: `/tmp/csgrs-rustdoc-concatenation-doc.log`,
`/tmp/csgrs-rustdoc-distribution-minimal-doc.log` and
`/tmp/csgrs-rustdoc-distribution-curve-doc.log`. Shared temporary build storage
is about 160 MiB. Full workflow/runtime checks remain waived for this docs-only
session. No implementation, assertion or performance claim changed.

### Solid projection, hull, loft and fluent API completion, 26 November 2026

Owner: CSGRS solid-to-planar, hull/Minkowski, loft, renderer and fluent API
boundaries; P1/P6/P8/P9/P17/P18/P19/P20/P25/P30/P37. Baseline: `093ec2c88`.
Read every line of `src/solid.rs` from `flatten` through end of file, including
the `SolidExt` implementation and all module tests. Together with the prior
explicit section records, this completes the file's source review. Read every
line of `tests/polyhedral_construction.rs`. Supporting reads covered native
section records/assembly, hull and Minkowski entry paths, source preparation and
pair hulls, GPU normal construction/projection and feature configuration; those
owning modules do not receive whole-file credit here.

Missing witnesses: public summaries omitted projection/winding behavior,
incidence-record loss, hull degeneracy and renderer/visitor panic contracts.
Fluent implementation methods and smoke-test declarations lacked their own
Rustdoc. Documented `SliceResult` component meanings and its currently empty path
list; slicing consumes native incidence records without returning them. Flatten
orients every projected face as material, so reversed source faces do not cancel.
Both operations retain local certainty without carrying mesh construction history
into the planar result, and cache completed source/context-specific outcomes.

Hull docs distinguish all stored positions from represented triangle sets,
including unchecked triangle rows and degenerate empty/lower-dimensional inputs.
Minkowski docs distinguish the represented-solid sum from pairwise stored-position
convexification and explain native default limits, cooperative cancellation and
construction qualifications. Loft docs describe section equality/coalescing,
per-section plane validation, cap selection, fixed side diagonals, orientation,
checked primary buffers and closed-PWN output validation. Approximate coalescing
remains qualified in the receipt and mesh evidence.

Bevy export is documented as lossy `f32` display geometry with flat normals and
panic-on-error behavior. Triangle visitation clones exact positions in authored
order but directly indexes them, so malformed indices can panic. Trait docs
state strict convenience behavior, explicit-context receipts and deferred Boolean
composition; every native implementation method has a scoped delegation contract.
Uniform metadata, exact translation helpers and all unit-test declarations are
documented. Polyhedral test docs identify independent area/volume expectations,
policy transitions and downstream admission separately from universal set proofs.
The existing twisted-square comment incorrectly said a quarter turn; its literal
coordinates encode an eighth turn followed by scaling, so only the comment changed.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/solid.rs tests/polyhedral_construction.rs`:
  passed. `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in both files,
  including tests, feature gates, panic paths and predicate order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve,attributed,bevymesh
  --document-private-items --no-deps --offline`: passed without warnings.
  The equivalent no-feature private-doc build passed as well.
- Inspected eleven generated contracts covering the slice alias, projection,
  slicing, Bevy panic/loss behavior, hull and Minkowski distinctions, loft,
  trait visitation, metadata and translation. Unit/integration-test docs remain
  source-reviewed; ordinary library documentation does not render test items.

Logs: `/tmp/csgrs-rustdoc-solid-completion-doc.log` and
`/tmp/csgrs-rustdoc-solid-completion-minimal-doc.log`. Reused build storage grew
to about 502 MiB for the optional Bevy dependency check; `/tmp` has over 125 GiB
available. No separate target tree or runtime test build was created. Full
workflow/runtime checks remain waived for this docs-only session. No executable
behavior, test assertion or performance claim changed. Completion here describes
this file's source audit, not repository-wide Rustdoc coverage or kernel closure.

### Slice/Minkowski witnesses and initial curve grammar, 26 November 2026

Owner: CSGRS section/projection and represented-solid Minkowski regression
contracts; native planar construction adapters and private predicate/cache
helpers. Relevant properties: P1/P6/P8/P9/P17/P18/P19/P20/P30/P37. Baseline:
`0665cc8f5`. Read every line of `tests/slice_projection_contract.rs`,
`tests/minkowski_contracts.rs` and `src/curve/mod.rs`. Read `src/curve/native.rs`
from its beginning through `regular_ngon`, inclusive. Supporting reads covered
side-wall helper consumers, native exact ring admission and bounded scalar
equality; these do not earn whole-file credit for their owning modules.

Missing witnesses: test helpers, test declarations, the compatibility trait and
private curve helpers lacked Rustdoc. Initial primitive summaries omitted
placement, sampling, validation order and collapse details. Documented the
actual assertions for holes, signed multiplicity, coincident faces, permutations,
open wires, canonical empties, invalid inputs, cache mutation/eviction, thread-local
policies, strict retries, native diagnostics, configured resource limits and
pre-set cancellation. Minkowski docs distinguish independent volume/decomposition
expectations from shared-kernel Boolean comparisons and analytic-ball claims.
The full sampled-sphere fixture remains unchanged. These are source-reviewed
witnesses, not claims of execution or universal geometric proof.

Curve docs now describe the strict-only 64-entry FIFO, coordinate equality keys,
non-refreshing hits and the distinction between entry bounds and memory/work
bounds. Sampled circle/ellipse docs give full-axis versus radius meanings,
starting points, orientation, straight edges, minimum samples, nonnegative
validation and qualified collapse. Polygon docs describe exact coordinates,
optional repeated closing points and native admission. Orientation helpers use
the segment-start polygon's policy-decided signed area; reversing endpoint pairs
does not reverse segment iteration. Exact ratio, polar and rotational-pattern
helpers document conversion errors, angular units, ordering and zero-copy behavior.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/slice_projection_contract.rs tests/minkowski_contracts.rs`: passed.
  `git diff --check`: passed.
- Every nonblank, non-comment source line matches baseline in all three edited
  Rust files, including assertions, feature gates and predicate order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected thirteen rendered contracts, including the macro-generated private
  cache item, predicate/ring helpers, wall orientation, exact ratio/pattern
  construction and public polygon/circle/ellipse/triangle entry points.
  Integration-test Rustdoc was reviewed in source; ordinary library docs do not
  render test items.

Log: `/tmp/csgrs-rustdoc-curve-primitives-doc.log`. Reused temporary build storage
is about 498 MiB, with over 125 GiB available on `/tmp`. No runtime builds or
additional target trees were created. Full workflow/runtime checks remain waived
for this docs-only session. No executable behavior or performance claim changed.

### Sampled and compound planar shapes, 26 November 2026

Owner: CSGRS native planar construction grammar and its adapter/downstream
acceptance witnesses; P1/P4/P5/P6/P8/P9/P17/P18/P19/P25/P30/P37. Baseline:
`3412d69d1`. Read every line of `src/curve/native.rs` from `arrow` through
`circle_with_two_flats`, inclusive. Read every line of
`tests/curve_constructor_failures.rs` and `tests/sampled_profile_failures.rs`,
including generated fixtures, optional expression artifacts and OBJ branches.
Supporting reads covered the existing unit invalid-parameter and egg/heart-bound
witnesses; the later native test module is still awaiting its full audit.

Missing witnesses: planar summaries did not specify placement, sample formulas,
normalization, clipping sides, capacity/validation order or qualified branch
selection. Integration-test declarations, helpers and generated test items lacked
Rustdoc. Documented all 27 constructor entry points in this section and every
named helper/test declaration in both integration files without changing logic.

The docs distinguish exact polygon definitions from analytic curve fidelity.
Arrow shoulders can coalesce under an approximate comparison while retaining
qualification across a strict ring-cache hit. Teardrop length is attained by its
samples only with an even semicircle interval count; squircle axis extrema may
also be absent at coarse resolution. Egg bounds are centered, while heart bounds
start at the origin; both normalize sample extrema. Superformula docs give the
radial expression, scalar failure paths and direct closing edge without promising
a smooth periodic seam.

Compound docs specify positive-Y keyhole handles, positive-X keyway cuts,
Y-directed flat clipping and zero-width-strip regularization. Reuleaux construction
intersects equally oriented sampled disks with the documented center radius;
`diameter` supplies each disk radius, and the polygonal result is not certified
constant-width geometry. Annular thickness is a difference of circumradii, with
edge-line separation `thickness * cos(pi / segments)`. Full-turn sectors use a
regular polygon starting on positive X regardless of authored start or sweep sign.
Capacity checks are distinguished from a total Boolean-work bound.

Test docs state actual error, certainty, area, topology and composition assertions.
They preserve the original high-resolution example fixtures and distinguish
expression artifacts from semantic proof. OBJ replay coverage concerns a dyadic
fixture's positions and volume; it is not generalized to lossless OBJ interchange.
Macro-generated tests now carry Rustdoc in their expansion.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/curve_constructor_failures.rs tests/sampled_profile_failures.rs`: passed.
  `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in the three edited Rust
  files, including macro bodies apart from Rustdoc, assertions and feature gates.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Checked all 27 affected generated constructor pages for the documented formulas,
  placement, policy and failure details. Integration tests and generated test
  declarations remain source-reviewed; the library doc build does not render them.

Log: `/tmp/csgrs-rustdoc-planar-shapes-doc.log`. Reused temporary build storage
is about 499 MiB. No additional target tree or runtime build was created. Full
workflow/runtime checks remain waived for this docs-only session. No executable
behavior, assertion or performance claim changed.

### Mechanical profiles and private gear samplers, 26 November 2026

Owner: CSGRS mechanical-profile grammar and native planar admission adapters;
P1/P4/P5/P6/P8/P9/P17/P18/P19/P25/P37. Baseline: `f39e10019`. Read every line
of `src/curve/native.rs` from `involute_gear` through `sampled_cycloidal_gear`,
inclusive, and every line of `tests/gear_airfoil_failures.rs`. Supporting reads
covered the later parameter-sensitivity, cycloidal-rack contact and mechanical
single-component unit witnesses; the later test module still awaits full review.

Missing witnesses: public mechanical summaries omitted dimension conventions,
derived restrictions, placement, sample parameterization and airfoil closure.
Private gear samplers and integration-test helpers/witnesses lacked Rustdoc.
Documented all ten public entry points, both private samplers and every named
helper/test declaration in the integration file. The scalar, predicate and
native-region implementations retain authority; no algorithm was changed.

Gear docs now give pitch/base/tip/root formulas, clearance and backlash units,
positive-X tooth placement, radial feet and shared rotational construction.
Involute flanks sample radius; cycloidal flanks sample rolling parameter, with
principal inverse-cosine selection of first-lobe endpoints and explicit reach
and separation checks. Both tip and root circular lands are polygonally sampled.
Private docs give the involute deflection and epicycloid/hypocycloid formulas,
root cutoff, endpoint tests, mirroring and final native admission.

Rack docs distinguish straight involute flanks from sampled cycloidal parts and
give pitch, root/tip levels, backing depth, end-wall placement, positive land
requirements and checked storage. The airfoil contract states normalized digit
meanings without an integrality restriction, uniform chord stations, piecewise
camber and normal offsets. Thickness coefficients preserve their binary64 literal
values. The authored ring keeps all upper stations and only lower interior
stations: the upper trailing edge connects directly to the last lower interior
sample, and the lower trailing-edge endpoint is omitted. Documentation does not
promise analytic fidelity or exact chordwise bounds for this polygon.

Test docs preserve the original resolutions and identify independent rack area
and airfoil rational-ordinate expectations, selected membership contacts, manifold
and volume checks, downstream empty consumption, typed capacity/invalid/undecided
failures and qualified cache/solid composition. Admission counts and scalar totals
are not presented as universal set equality or performance measurements.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/gear_airfoil_failures.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both edited Rust files,
  including tests, validation order, sample loops and scalar coefficient literals.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected twelve generated public/private contracts for formulas, placement,
  capacity, branch selection and closure. Integration tests remain source-reviewed;
  ordinary library documentation does not render their declarations.

Log: `/tmp/csgrs-rustdoc-mechanical-profiles-doc.log`. Reused temporary build
storage remains about 499 MiB. No runtime build or additional target tree was
created. Full workflow/runtime checks remain waived for this docs-only session.
No executable behavior, assertion, coefficient or performance claim changed.

### Exact control nets and Hilbert clipping, 26 November 2026

Owner: CSGRS exact path/region constructors and retained Hilbert trim adapters;
P1/P4/P5/P6/P8/P9/P12/P15/P16/P17/P18/P19/P25/P30/P37. Baseline: `abccd1272`.
Read every line of `src/curve/native.rs` from `bezier_path` through
`hilbert_paths_with_context`, including both function-local grid helpers.
Read every line of `tests/exact_path_failures.rs` and `tests/hilbert_trimming.rs`.
Supporting reads covered native path admission, spline layout/endpoints and
parameter domains, general rational Bezier construction, and retained region
bounds. These supporting modules are not credited as whole-file reviews.

Missing witnesses: exact-path docs omitted carrier selection, parameter domains,
fill rules, unused display-count behavior and precise diagnostics. Hilbert docs
needed bounds/placement and conversion details; local integer helpers and test
helpers/declarations lacked Rustdoc. Documented all ten entry points in this
section, both local functions and all named declarations in the integration files.

Bezier docs specify line/quadratic/cubic specialization and unit-weight rational
storage for higher degrees, while preserving the polynomial meaning. Closed paths
do not imply filled regions; region construction checks closure and uses native
even-odd admission without adding a closing line. Spline docs give clamped integer
knots, retained source controls and evaluation on `[0, control_count - degree]`.
Positive display counts are validation-only and do not request sampling buffers.
Checked spline index overflow remains `InvalidArguments`; documentation does not
relabel it as resource exhaustion or change validation.

Hilbert docs give the cell-center formula, traversal orientation, pre-empty
parameter validation, padded bounding rectangle and possibly conservative native
envelope. The point-count cap does not bound native trim work. The exact path view
retains source intervals and contact evidence; the string view preserves represented
line geometry but drops those records and returns a typed unsupported result when
selected endpoints lack scalar coordinates. Local quadrant rotation/distance
helpers state their integer-domain preconditions.

Test docs distinguish independent interval/area/midpoint expectations from wrapper
agreement, conservative axis-aligned simplification from general curve simplification,
and contact replay from display approximation. The historical test named
`unresolved_closure_is_undecided_across_policy_reuse` constructs its endpoint with
`sin(e)^2 + cos(e)^2 - 1` directly and checks only the resulting native error
category/source. It does not separately test endpoint-equality uncertainty or
localize the failing stage. Its Rustdoc now states those limits. Its current
runtime outcome has not been reproduced in this docs-only batch; do not cite it
as proof of a specifically localized closure blocker. The fixture and all its
assertions remain unchanged.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/exact_path_failures.rs tests/hilbert_trimming.rs`: passed.
  `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in all three Rust files,
  including local functions, validation order, fixture expressions and assertions.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected all ten generated entry-point pages for carrier/domain, fill,
  validation, placement and evidence contracts. Function-local and integration-test
  documentation was reviewed in source; ordinary library docs do not render it.

Log: `/tmp/csgrs-rustdoc-exact-paths-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. No
executable behavior, assertion or performance claim changed.

### Profile conversion, meshing and extrusion, 26 November 2026

Owner: CSGRS curve-to-mesh modeling boundary, native triangulation adapters and
output validation; P1/P4/P6/P8/P9/P12/P16/P17/P18/P19/P25/P30/P37. Baseline:
`830b957e5`. Read every line of `src/curve/native.rs` from `try_triangulate`
through `emit_section_walls`, inclusive, and the complete separate
`native_boundary_source_probe` test module at the end of that file. Read every
line of `tests/mixed_modeling_profiles.rs`. Supporting reads covered native
linear-profile retention, mixed sampling/join selection, finite export ownership,
triangulation, geometric validation, scalar debug checks and construction-receipt
publication. Those kernel modules are not credited as completed Rustdoc audits.

Missing witnesses: private meshing helpers lacked Rustdoc, and public summaries
did not expose extrusion validation order, finite-history limitations or the
actual strength of output validation. Documented the public boundary, private
helpers, callback/ending types and relevant test declarations without changing
construction, predicate order or assertions.

Modeling profiles choose exact linear views independently; only absence of that
view selects mixed sampling, while native failures propagate. Sampling preserves
represented line members and native endpoints, including tiny members beside
curved components, and records sampling/projection qualifications on mesh output.
Endpoints without native coordinates may also be projected. The chord-error value
is a request, not a blanket topology/fidelity certificate. Finite export projects
all coordinates and retains native ownership with a local certainty receipt; its
generic outcome does not attach the internal mesh construction accumulator.

Extrusion docs specify signed height, full displacement vectors, nonzero-Z
validation before empty handling, cap/wall orientation and canonical empty-source
results. Validation checks unique nondegenerate face geometry and directed
geometric edge pairing; neither validator checks embedding or vertex links.
A native false result becomes a modeling `Geometry` error, while native failures
retain their causes. The allowed-empty closure shortcut inspects triangle count,
not positions. The separate source-probe tests document these distinctions.

The triangle emitter uses reverse structural position lookup, has a strict
nondegeneracy debug assertion, and returns `true` on every normal path, including
suppression of repeated-index faces when the assertion is disabled. Its Boolean
return is not an emission certificate. Public triangulation/extrusion docs expose
the debug panic condition. Cap/wall docs specify traversal order, the chosen quad
diagonal, caller-owned apex collapse, all-four-corner placement even for an apex,
and partial local buffers on placement errors; final publication remains a caller
responsibility.

Mixed-profile tests document independent exact component measures and retained
vertices for rational, irrational, tiny and far-translated inputs, plus native
closure checks and sampling qualifications across five consumers. Repeated thin-wall
Booleans are checked after dropping the source region. These witnesses remain
source-reviewed, not runtime completion or performance claims.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/mixed_modeling_profiles.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both edited Rust files,
  including debug assertions, emission returns, all tests and validation order.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected sixteen generated public/private pages, including the sampling
  constant, placement alias, wall-ending enum, validators and emitters. Test
  declarations were inspected in source; the library build does not render them.

Log: `/tmp/csgrs-rustdoc-profile-meshing-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. No
executable behavior, assertion or performance claim changed.

### Signed revolution and exact profile sweep witnesses, 26 November 2026

Owner: CSGRS curve-to-mesh revolution, signed-region composition and output
validation; P1/P4/P6/P8/P9/P12/P16/P17/P18/P19/P25/P30/P37. Baseline:
`2b8e62e6b`. Read every line from `revolve` through
`split_at_revolution_axis` in `src/curve/native.rs`, and every line of
`tests/exact_profile_sweeps.rs`. Supporting reads inspected native closed-PWN,
unique-face and geometric edge-pairing validation in Hypermesh; that file is
not credited as a completed audit.

Missing witnesses: revolution mode/dispatch/validation helpers lacked Rustdoc;
public docs omitted segment minima, empty-input validation order and the
limited rational turn-count recognizer. The test module summary understated
its revolution oracles, and `exact_solid` claimed manifoldness beyond the
predicates it calls. Updated these contracts and documented all 50 test
function/constant/type declarations, including the function-local prism alias.

Revolution docs distinguish legacy overlapping partial shells from regularized
signed material, preserve odd-lattice phase and state the even full-turn fold
precondition. Oversized turns use `exact_rational` for integer recognition;
failure to recognize rationality is undecided, not exhaustion or a proved
fractional turn. Angular-table addressability is not a global resource budget.
Parameter validation precedes empty handling. Partial cap emission retains its
strict nondegeneracy debug assertion. Explicit-start revolution transports the
completed swept-boundary result and its qualifications through a transform.

Private docs state caller obligations for matching sign arrays and angular
samples, potential zip truncation/indexing on violated internal preconditions,
axis-row sharing, quad diagonals, cap orientation and native filled-region
authority for splitting/folding. Closed-PWN validation checks nondegenerate
faces and balanced edge multiplicities without proving manifoldness or unit
winding; the legacy check also lacks embedding/vertex-link validation. The
regularized allowed-empty path checks both buffers, unlike the legacy
triangle-only shortcut.

The test audit separates exact volume/vertex witnesses, deterministic buffer
comparisons, shared Boolean composition and independent sector/ray-winding
oracles. The retained winding-two mutation demonstrates why edge closure alone
does not establish regularized material. The 576-row matrix and finite query
grids are scoped regression evidence, not universal closure proofs. Optional
STL replay explicitly checks projection provenance and equality relative to the
imported mesh. No fixture, predicate, assertion or executable behavior changed.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs
  tests/exact_profile_sweeps.rs`: passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both Rust files.
  A declaration scan confirmed Rustdoc on all 50 function/constant/type items
  in the test file; that file-local scan is not a repository coverage metric.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected ten generated public/private pages, including the output enum and
  all revolution helpers. Integration tests and the local alias were reviewed
  in source; ordinary library docs do not render those items.

Log: `/tmp/csgrs-rustdoc-revolution-doc.log`. Reused temporary build storage
remains about 499 MiB. No runtime build or additional target tree was created.
Full workflow/runtime checks remain waived for this docs-only session. Test
execution, optional STL replay and performance were not revalidated here.

### Sweep frames and twisted extrusion, 26 November 2026

Owner: CSGRS profile placement/frame construction and native mesh validation;
P1/P4/P6/P8/P9/P16/P17/P18/P19/P25/P30/P37. Baseline: `943279e05`.
Read every line from `sweep_scalar_error` through `extrude_twisted` in
`src/curve/native.rs`, and every line of `tests/sweep_frames.rs`. Supporting
reads covered vector normalization, minimal unit-vector rotations, shared wall
and cap emission, the previous exact-profile fixtures, and scalar uncertainty
controls. The later optional/transform wrappers were read for context but are
not yet credited as completed item audits.

Also reviewed/documented these main-module items completely: the certified and
terminal zero helpers, `signed_volume`, `assert_exact_solid_boundary`,
`profile_solid_constructors_share_closed_indexed_topology`,
`sweep_rejects_revisited_open_path_vertices`,
`degenerate_profile_solid_requests_do_not_claim_solids`,
`approximate_sweep_reports_consumed_terminal_point_equality`, and
`sweep_scalar_failures_keep_their_outcome_category`. The remaining main-module
tests still require their own audits.

Missing witnesses: sweep/twist docs omitted the closed-frame seam, validation
before empty input, allocation scope and emitter debug assertions; twist docs
overstated manifold validation. Private frame docs omitted predicate order,
fallback conditions and which errors can be superseded by an independent
nonzero component. Test docs called approximate terminal resolution a
certificate, treated unnormalized dot products as basis coordinates and
overstated the nearest-center grouping's separation assumption.

The sweep retains one frame per distinct path vertex, using chord normals at
open endpoints and normalized unit-chord bisectors elsewhere. Closed paths
connect the final stored frame directly to the first, without redistributing
residual rotation. Scalar normalization retries only `UnknownZero`; a caller's
nonzero decision is not passed as a proof object into the normalizer. Reversal
and initial-axis helpers preserve component order, permit a later independent
nonzero component to resolve an earlier failure, and otherwise propagate the
last component error. Unknown scalar certificates acquire the sweep label and
policy; exhaustion and other scalar causes remain typed scalar failures.

Sweep path/frame construction and twist parameter-table construction precede
empty-profile handling. Neither entry point introduces a total-work/heap or
cancellation budget; twist has no angular-table addressability preflight.
Wall/cap debug assertions can precede final validation. The latter proves
unique nondegenerate faces and directed edge pairing, not vertex-link
manifoldness, embedding or unit winding. Negative twist-extrusion height
reverses wall/cap orientation; apex tapering has no top cap.

Rechecked the documented thin-strip counterexample independently with exact
Python `Fraction` arithmetic in Q(sqrt(3)): width 2^-20 at X=1, Y in [0,2],
height 1, twist 90 degrees, three equal bands and final scale (1/2,2/3).
Placed four sections using the documented linear scales and 0/30/60/90-degree
sines/cosines, split each wall on the implemented diagonal and added planar
caps. All 28 triangles were nondegenerate and every directed geometric edge
had one reverse. The exact scalar-triple-product sum (six-volume) was

```text
-1495330579284247/1068725302198272 + (205/169869312) * sqrt(3).
```

Its upper bound using sqrt(3) < 2 is negative. This checks the mathematical
construction formula and the limited meaning of edge closure; it is not a
runtime execution or completion claim for the Rust constructor.

Frame tests now distinguish stored-rational checks, exact distance/normal/axis
invariants, binary64 helix residuals, approximate nearest-center grouping,
Boolean volume identities and error-category witnesses. Printed timings have
no performance threshold. The local projected-volume helper skips failed
projections, and its Rustdoc no longer implies an exact volume certificate.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs tests/sweep_frames.rs`:
  passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both Rust files.
  A file-local declaration scan found Rustdoc on all 21 test functions/constants;
  this does not establish repository-wide documentation coverage.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features curve --document-private-items --no-deps
  --offline`: passed without warnings.
- Inspected six generated public/private pages for sweep, twist and frame/error
  helpers. Integration and unit tests were inspected in source; ordinary
  library documentation does not render test-only items.

Log: `/tmp/csgrs-rustdoc-sweep-frames-doc.log`. Reused build storage remains
about 499 MiB. No additional target tree or runtime build was created. Full
workflow/runtime checks remain waived for this docs-only session. No predicate,
construction, assertion or executable behavior changed.

### Final native curve adapters, transforms, queries and tests, 26 November 2026

Owner: CSGRS curve modeling/conversion boundary and planar metaball adapter;
P1/P4/P6/P8/P9/P12/P16/P17/P18/P19/P25/P30/P37. Baseline: `f187abd82`.
Completed the item audit from `metaballs` through `translated_solid` and every
remaining main-module test in `src/curve/native.rs`. Together with prior
contiguous batches and the already reviewed final validation-probe module,
this completes the file's line-by-line review. Also read and documented every
line/item in `src/curve/metaballs.rs`.

Supporting reads inspected image cell tracing and coordinate conversion,
TrueType text assembly/layout, Hershey stroke assembly, native region affine
transforms/classification/envelopes, Hypersdf field validation/evaluation/bounds,
solid translation and Cargo feature wiring. The image/TrueType implementation
files and supporting kernel files are not credited as completed audits here.

Missing witnesses: raster and convenience-wrapper summaries omitted the model
being adopted, discarded report/receipt details and empty-input validation;
generic transform summaries omitted planar homogeneous semantics and native
representation blockers. Planar metaball internals and remaining unit witnesses
lacked Rustdoc. Documented these without changing code, tests or feature wiring.

Raster tracing adopts thresholded integer unit cells, keeps image Y coordinates
without reflection and drops the internal trace report when returning a region.
TrueType preserves parser-emitted line/Bézier outlines with nonzero fill, selects
face zero, validates even empty text and uses explicit newline/carriage/tab
behavior. Its receipt does not repair upstream parser rounding. Hershey returns
open strokes after an exact Y reflection; all whitespace, including newlines,
advances horizontally. Entire-font record and size validation also precedes
empty-text success. Partial text is not published after failures.

Planar metaballs sample `(iso-I)/(iso+I)`, including the continuous -1 value at
centers. The native chord region is a discrete model, not a continuous-field
fidelity certificate. Grid count, field parameters and padding precede empty
handling. X varies fastest, without the 3D sampler's extra border. Sampling
errors retain grid index and source; buffer addressability/reservation checks
do not establish a global work/memory budget. Field construction/evaluation and
bounds remain strict while extraction uses the selected curve policy.

Planar matrix docs specify the restriction to `(x,y,0,1)`, irrelevant third
column, constant-weight normalization, pre-empty validation order and native
determinant/topology authority. A singular full 4x4 matrix can still have an
admissible planar restriction. Point queries distinguish boundary from missing
evidence; bound queries distinguish a certified envelope from tight retained
algebraic-fragment extrema. The legacy solid-translation bridge remains a 3D
mesh transform with construction history, not a planar conversion.

Remaining tests now describe cache/policy isolation, certified versus terminal
zero controls, finite-export qualifications, constructor/catalog smoke tests,
structural inequality, exact sampled extents, rack contacts, Hilbert centers and
Hershey coordinates according to their actual assertions. They do not upgrade
projected measurements, wrapper agreement or structural inequality to general
geometric-set proofs. A declaration scan found Rustdoc on all 162 named
function/type/enum/module/static/constant declarations in `native.rs` and all
three functions in planar `metaballs.rs`. These local counts are not a
repository-wide coverage metric or a substitute for the recorded source reads.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/native.rs src/curve/metaballs.rs`:
  passed. `git diff --check`: passed.
- All nonblank, non-comment source lines match baseline in both edited Rust
  files; assertions, decision order, geometry and feature wiring are unchanged.
- With `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0` and
  `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`, both commands passed
  without warnings:
  - `cargo doc --locked -p csgrs --no-default-features --features
    curve,metaballs,image-io,truetype-text,hershey-text --document-private-items
    --no-deps --offline`.
  - `cargo doc --locked -p csgrs --no-default-features --features curve
    --document-private-items --no-deps --offline`.
- Inspected twenty rendered adapter/transform/query pages and all three planar
  metaball helper pages in the combined-feature build. Test-only items remain
  source-reviewed; ordinary library docs do not render them.

Logs: `/tmp/csgrs-rustdoc-curve-adapters-doc.log` and
`/tmp/csgrs-rustdoc-curve-tail-minimal-doc.log`. The shared temporary build tree
is about 542 MiB after checking the additional image/font feature dependencies.
No separate target tree or runtime build was created. Full workflow/runtime
checks remain waived for this docs-only session; no performance claim is made.

### Image tracing and TrueType internals, 26 November 2026

Owner: CSGRS raster/font conversion boundaries and their private topology/layout
adapters; P1/P6/P8/P9/P12/P19/P25/P30/P37. Baseline: `98ec9042e`.
Read every line of `src/curve/image.rs`, `src/curve/truetype.rs`,
`tests/text_contracts.rs` and `tests/support/text_font.rs`. Supporting reads
checked the locked ttf-parser 0.25.1 CFF completion/`ZeroBBox` path and existing
uncertainty helpers. Third-party parser reads and the real-font workload's
asset reference are not credited as separate repository-file audits.

Missing witnesses: raster report/grid fields, tracing helpers and tests lacked
complete Rustdoc; the TrueType collector and its trait overrides/fields were
undocumented. The raster rectangle test's historical name overstates what its
assertions inspect, so this batch describes exactly which quantities and
decisions each test asserts.

Raster docs distinguish exposed unit-edge work counts from final simplified
segments, preserve direct numeric Y coordinates and explain the positive-turn
rule separating diagonal contacts. The outgoing-edge walk assumes the emitter's
oriented grid graph; the collinear simplifier assumes a nonbacktracking boundary
and does not check betweenness on arbitrary paths. Native contour/region
admission remains the topology authority. Zero/unavailable/undecided orientation
share the existing explicit error, and ordinary collection growth is not a
typed resource-budget contract. Two tuple error variants were expanded solely
to attach Rustdoc to their retained native error payloads.

The raster rectangle witness checks six selected cells, ten exposed edges and
one material loop without holes. Despite its historical name, it does not
assert four final segments. Its assertions are preserved, and the documentation
states that evidence limit rather than claiming stronger validation.

Font docs specify required versus declared-but-unparsed tables and lazy glyph
validation. Blank glyf evidence requires valid loca ranges and either no bytes
or a complete zero-contour header; CFF accepts only the parser's completed
charstring with untouched bounds. CFF2 absence remains explicitly unsupported.
Callbacks preserve exact parser-coordinate promotion and offset-before-scale
placement, retain line/quadratic/cubic families, and save the first failure.
They may leave partial local state, but text assembly rejects it before any
successful geometry is published. Private fields and all callback overrides
are documented and visible in the generated private API pages.

Layout docs describe exact font/point units, control-character cursor behavior,
default space fallback and the adapter's limited pair handling. A nonzero GPOS
sum wins; zero, including cancellation, falls back to eligible kern tables.
All parsed GPOS lookups are visited without script/feature selection, and class
zero still requires left-glyph coverage. This is not a full shaping engine or a
certificate of source-font fidelity before parser coordinate rounding.

Text tests document independent square/arch area and membership expectations,
extreme and symbolic scales, native error causes, empty/Boolean/extrusion reuse,
Hershey placement and retained real-font/catalog workloads. Optional STL checks
compare binary32 position sets and local edge/face validation, not original
exact curved geometry. Synthetic sfnt helpers document their small-count/delta
assumptions, intentionally zero checksum/search metadata, R-only GPOS coverage
and directory-length corruption without payload removal.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 src/curve/image.rs src/curve/truetype.rs
  tests/text_contracts.rs tests/support/text_font.rs`: passed.
  `git diff --check`: passed.
- Nonblank, non-comment source matches baseline after accounting explicitly for
  the two tuple-field layout expansions. No expressions, assertions, fixtures,
  callback order or control flow changed.
- Local declaration scans found Rustdoc on 26 image declarations, 31 TrueType
  declarations, 18 text-test declarations and seven synthetic-font helpers;
  fields were checked separately. These are file-local checks, not a general
  repository coverage metric.
- `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target CARGO_INCREMENTAL=0
  RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links' cargo doc --locked -p csgrs
  --no-default-features --features
  curve,metaballs,image-io,truetype-text,hershey-text --document-private-items
  --no-deps --offline`: passed without warnings.
- Inspected nineteen rendered image/TrueType pages, including private fields,
  tuple error payloads and trait callback documentation. A rendered-text check
  initially searched for “suppress” instead of the actual “suppresses”; correcting
  that check confirmed the existing generated text. Test-only items remain
  source-reviewed; ordinary library docs do not render them.

Log: `/tmp/csgrs-rustdoc-image-text-doc.log`. Reused temporary build storage is
about 543 MiB. No runtime build or separate target tree was created. Full
workflow/runtime checks remain waived for this docs-only session; no runtime
completion or performance claim is made.

### Shared interchange, AMF, OBJ and PLY batch, 26 November 2026

Baseline: `b5da7bf6e`. Owner: CSGRS interchange boundary, with native scalar,
triangulation and mesh authority retained. Relevant properties: P6/P8/P9,
P18–P20, P25–P27, P29/P30 and P37. Domain: shared format diagnostics and finite
projections; AMF unit/color/metadata serialization; OBJ exact scalar parsing and
face triangulation, with optional authored normals; ASCII PLY generated normals
and successful-output caching. Read all seven files listed above in full.
Supporting reads of native graphics row assembly, geometry qualification and
projection-axis selection do not count as additional whole-file kernel audits.

Missing documentation witnesses included shared error payloads/helpers, private
format parsers/cache fields and integration-test declarations. The shared face
helper's old summary described only planar earcut, omitting triangle pass-through,
nonplanar fans and the locally convex fan path. AMF color documentation needed
to distinguish projected range checks from exact source comparisons.

The shared face docs now describe support-search error order, every-point
planarity decisions, strict dominant-axis preference with policy fallback,
collinear retention and earcut winding restoration. A three-corner face is not
geometrically validated here; local turn agreement is not a general simplicity
proof. Textual conversion diagnostics are distinguished from typed native
construction sources. Finite projection does not establish source equality or
topology; binary32 uses the cached binary64 view, and the decimal formatter
preserves that selected finite value rather than an arbitrary exact source.

OBJ docs describe ignored records/fields, already-read index domains and exact
decimal parsing. Attributed import chooses the first normal for a repeated
position within a face, omits the entire sidecar if any corner lacks a normal,
and ignores an eager graphics-conversion failure. Its success is not a graphics
validity certificate. AMF emits all stored vertices and checks projected color
bounds; PLY emits separate triangle-corner rows with generated flat normals.
PLY cache identity is allocation-based reuse, not geometric equality; four
entries per thread do not bound retained bytes. All three writers finish
conversion before writing but cannot promise rollback after an I/O failure.

Test docs describe finite exponent-family coverage without claiming exhaustive
binary64 coverage. Independent decimal/bit checks, box support planes and signed
volume remain separate from local topology and inherited provenance. Existing
fixtures and assertions, including cache mutation/eviction/concurrent use and
empty-result composition, were preserved rather than rerun in this docs session.

Validation after the final Rust edit:

- `rustfmt --check --edition 2024 --config skip_children=true` on the seven
  edited Rust files: passed. `git diff --check`: passed.
- Nonblank, non-comment source matches baseline after explicit normalization
  of six error-payload tuple layouts and the local formatter's tuple layout.
  No expressions, control flow, ordering, fixtures or assertions changed.
- File-local declaration scans covered 12 shared-I/O declarations, six AMF,
  seventeen OBJ, seven PLY, and 10/10/13 declarations in the corresponding
  integration files. Two multiline feature attributes required manual scan
  follow-up; both helpers have attached Rustdoc. Fields and variant payloads
  were separately checked. These counts are not repository coverage metrics.
- With `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target`, `CARGO_INCREMENTAL=0`
  and `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`, ran
  `cargo doc --locked -p csgrs --no-default-features --features
  obj-io,ply-io,amf-io,stl-io,dxf-io,gerber-io,attributed
  --document-private-items --no-deps --offline`: passed without warnings.
- The same private-doc command without features passed. Inspected 29 rendered
  pages in the gated build, including error payloads and cache fields. Local
  formatter items and tests remain source-reviewed because ordinary library
  docs do not render them.

Logs: `/tmp/csgrs-rustdoc-io-common-doc.log` and
`/tmp/csgrs-rustdoc-io-minimal-doc.log`. Temporary shared build storage is about
693 MiB. No second build tree or runtime build was created. Full workflow and
runtime checks remain waived by the owner for this docs-only session; no
runtime correctness or performance claim is made.

### STL boundary and construction-evidence pipeline batch, 26 November 2026

Baseline: `3e608bbff`. Owner: CSGRS STL interchange boundary and its qualification
witnesses, P6/P8/P9/P18–P20/P25–P27/P29/P30/P37. Domain: ASCII binary64 spelling,
binary32 STL projection, encoded-face nondegeneracy, closed-PWN import, immutable
bound artifacts and geometry-byte caching. Read the three files above in full.
Supporting native closed-PWN cache validation, the parent test-module declaration
and installed `stl_io` parsing/writing were inspected without additional
whole-file audit credit. SVG adapter/kernel exploratory reads remain pending
documentation review and are not credited in this batch.

Missing witnesses: private cache and artifact fields, local count bound, triangle
helpers, construction-aware import and test declarations lacked complete Rustdoc.
The public descriptions omitted several distinctions between export and import.

Documented that ASCII uses finite binary64 normal arithmetic, which can fail by
underflow/overflow even for nondegenerate exact input. Binary export computes
normals before binary32 coordinate projection, then separately validates encoded
triangle nondegeneracy under strict native decisions. It allows open surfaces;
artifact import still requires closed PWN geometry or canonical empty output.
Names are validated but absent from the zero-filled binary header. Unused source
positions and file normals are not retained by the respective conversions.

The byte cache owns both source allocations, retains only complete successes,
evicts in insertion order and returns independent vectors. The entry cap is not
a memory/work budget. Artifact evidence comes from the current source mesh even
on a geometry cache hit. Plain import records projected coordinates and missing
source history; artifact import keeps its bound qualification while proving only
local parsed topology. Artifact byte extraction copies the shared bytes and
deliberately loses the receipt.

Test docs preserve independent binary record construction, tiny-normal and box
volume expectations, exact-vs-projected geometry, collapsed-face rejection and
cache retry/eviction/mutation. The construction-evidence matrix distinguishes
exact fractional placement from its later lossy STL projection. Its twelve names
permit fresh-process invocation but the normal test harness does not guarantee
it. The outside-point case delays stronger sign proof until after observation;
the printed missing-pyramid-volume formula is a mathematical explanation, not
an independently constructed reference hull assertion.

Validation:

- `rustfmt --check --edition 2024 src/io/stl.rs
  tests/export_projection_contracts.rs
  tests/support/construction_evidence_pipeline.rs`: passed after final edits.
  `git diff --check`: passed.
- Nonblank, non-comment source matches baseline exactly in all three files.
  File-local scans found Rustdoc on 21 STL declarations, 15 export-projection
  declarations and 16 pipeline declarations; private fields were separately
  checked. No behavior, fixtures or assertions changed.
- After the final library edit, with the shared temporary target,
  `CARGO_INCREMENTAL=0` and broken Rustdoc links denied, ran
  `cargo doc --locked -p csgrs --no-default-features --features stl-io,obj-io
  --document-private-items --no-deps --offline`: passed without warnings.
  Inspected eleven rendered STL pages, including fields and artifact methods.
  Local constants and test items remain source-reviewed.

Log: `/tmp/csgrs-rustdoc-stl-doc.log`. Shared temporary build storage remains
about 693 MiB. Full workflow/runtime checks remain waived for this docs-only
session. No runtime completion or performance claim is made.

### DXF and SVG adapters and SVG witness corpus, 26 November 2026

Baseline: `aba5b96b6`. Owners: CSGRS interchange adapters, with geometry decisions
owned by native scalar, mesh and curve layers. Relevant properties: P6/P8/P9,
P12/P15–P20, P25–P27/P29/P30/P37. Domain: DXF surface-entity interpretation and
triangle output; SVG default-options native import/export, selected definitions,
qualified empty geometry and downstream exact operations. Read both adapter
files and all nine `tests/svg*.rs` files in full. Supporting reads of Hypercurve
SVG options/geometry/replay paths, checked vector normalization, the locked DXF
library's date conversion and the DXF context integration witness are not
credited as additional whole-file audits. The referenced archived selected-circle
fixture and shared macro source remain separate audit work.

Missing documentation witnesses included all private DXF helpers, the SVG error
mapper and SVG test declarations. Public adapter summaries did not describe
important conversion and validation limits. These findings were reproduced by
reading the current implementations; this session's docs-only waiver does not
turn that source review into a new runtime execution claim.

DXF docs now state that OCS axis selection compares normalized components against
1/64 in short-circuit order, while checked normalization has its own failure
path. A polygon's first three corners must be nondegenerate before the shared
triangulator runs. Closed polylines use straight stored-vertex loops without
bulge/width/special-mode interpretation, and circles use 64 finite trigonometric
samples. Signed SOLID thickness uses a fixed face assembly without reorienting
negative extrusion. Import retains coordinate-projection/missing-history
qualification and does not validate whole-mesh closure.

The DXF exporter directly indexes positions before fallible conversion: malformed
triangle references can panic. This is documented under `Panics`, not relabeled
as a typed failure or fixed in this docs-only batch. The finite normal calculation
also remains distinct from exact nondegeneracy. Fixed timestamps remove call-time
variation, but local date fields still depend on host timezone; no cross-timezone
determinism or exact decimal-source replay is claimed.

SVG docs identify the tuple's filled region, native stroke wires and remaining
stroke paths. Strokes are centerlines, not width-expanded material. The wrapper
uses native defaults and strict topology/replay; it does not return a separate
aggregate certainty outcome. Standard circle/ellipse and rounded-rectangle
primitives use the current sampled model. Exact extensions retain supported
definitions and selections, while standard display paths alone are not native
replay. Hidden/unused definitions remain subject to validation. Scalar, exact
curve and polynomial topology causes remain typed through the adapter.

SVG witnesses now explain independent algebraic contacts, rational rectangles,
parabola normal shifts, selected-root/chord/circle definitions, chart-aware
polynomial identity and subsequent restriction/split/reversal/Boolean/offset
operations. Semantic XOR checks are distinct from stable bytes. The annulus and
selected-circle grow/shrink equalities apply to those fixtures, not arbitrary
regularized regions. Qualified empty export rejection, invalid unused selectors,
zero-work exhaustion and larger-budget retry retain their actual assertion scope.

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on both
  adapters and all nine SVG integration files: passed. `git diff --check`: passed.
- Nonblank, non-comment source matches baseline exactly in all eleven files.
  No expressions, fixtures, assertions, predicate order or behavior changed.
- File-local scans found Rustdoc on twelve DXF declarations, three SVG functions
  and 25 declarations across the SVG integration files. The options re-export
  was checked separately. These are local checks, not repository coverage.
- With `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target`, `CARGO_INCREMENTAL=0`
  and `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`, ran
  `cargo doc --locked -p csgrs --no-default-features --features dxf-io,svg-io
  --document-private-items --no-deps --offline`: passed without warnings.
  Inspected fourteen rendered pages, including private helpers, export panic
  documentation and the options re-export. Test items remain source-reviewed.

Log: `/tmp/csgrs-rustdoc-dxf-svg-doc.log`. Shared temporary build storage is about
789 MiB. Full workflow/runtime checks remain waived for this documentation-only
session; no new runtime completion or performance claim is made.

### glTF import/export and downstream witnesses, 26 November 2026

Baseline: `5d8e6a2af`. Owner: CSGRS glTF interchange boundary, preserving native
mesh/scalar authority. Relevant properties: P6/P8/P9/P18–P20/P25–P27/P29/P30/P37.
Domain: selected self-contained JSON/GLB scenes, validated triangle accessors,
flattened finite node transforms, exact graphics-row staging, projected named
scene export and canonical empty geometry. Read all three files above in full.
The installed glTF dependency's binary32 matrix/TRS conversion was inspected as
supporting evidence; no additional kernel or dependency whole-file credit is
claimed.

Missing witnesses: the matrix alias, cache/staging fields, private conversion and
validation helpers, single-object writer and test declarations lacked Rustdoc.
Existing public descriptions omitted projection/normal checks' timing and
described render-safe names without an implemented character-restriction policy.

Import docs now distinguish document-wide graph/buffer validation from selected
scene traversal. Graph checks reject multiple parents, cycles and duplicate or
non-root scene entries; an explicit stack avoids recursion but is not a total
resource budget. Accessor docs state the prior document/buffer invariants,
component alignment, range arithmetic and strictly increasing sparse indices.
Only consumed geometry receives layout/triangle checks beyond schema validation.
All declared buffers must still be self-contained and sufficiently long.

The parser forms matrices/TRS in binary32; the adapter widens and composes in
binary64 before exact promotion. That promotion does not restore exact authored
transform arithmetic. Reflections keep index order; singular maps must satisfy
the final triangle normal check rather than a separate determinant rejection.
Import discards normals/materials/textures/animations, flattens static node
transforms and carries missing-history/projection qualification. Closed topology
and an aggregate certainty receipt are not established by the import report.

Export staging is an owned snapshot of native generated flat-normal rows, not a
second modeling authority. Scene capture defers finite conversion and duplicate
name checks. JSON escaping handles names without a separate metadata filter.
Projection can collapse a triangle; unlike STL's encoded-face check, this path
does not revalidate projected nondegeneracy. Normals are projected independently,
not recomputed from encoded vertices. Empty objects retain named meshless nodes,
and no-object scenes omit node arrays. Exact array identity/name keys drive the
four-entry success cache without defining geometric equality or a byte budget.
Writers complete conversion before writing but cannot roll back I/O failure.

Test docs describe independently assembled GLB/accessor/binary rows and exact
3-4-5 normals/box volumes, separately from error comparison with the native GPU
path. The tiny-height retry proves normal certification and retry, not coordinate
survival after projection. Reimported empty geometry has explicit missing
history, not preservation of the original source's approximation receipt. The
10,000-node chain witness checks completion without a timing/memory claim.

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on the
  three edited Rust files and `git diff --check`: passed.
- Nonblank, non-comment source matches baseline exactly. File-local scans found
  Rustdoc on 29 glTF declarations, 18 empty-composition declarations and 22
  interchange-failure declarations; private/public fields were checked separately.
  No implementation, assertion, fixture or decision-order changes occurred.
- With the shared temporary target, `CARGO_INCREMENTAL=0` and broken Rustdoc
  links denied, ran `cargo doc --locked -p csgrs --no-default-features
  --features gltf-io,obj-io,ply-io --document-private-items --no-deps --offline`:
  passed without warnings. Inspected 27 rendered pages. One text-check expectation
  initially used “correct item width” instead of the documented “actual item width”;
  correcting that check confirmed the existing rendered contract. Local constants
  and test items remain source-reviewed.

Log: `/tmp/csgrs-rustdoc-gltf-doc.log`. Shared temporary build storage is about
815 MiB. Full workflow/runtime checks remain waived for this docs-only session;
no new runtime completion or performance claim is made.

### VRML import and geometry-context witnesses, 26 November 2026

Baseline: `f6cbab622`. Owners: CSGRS VRML parser/interchange adapter and explicit
policy/receipt consumers, P6/P8/P9/P19/P20/P25/P26/P30/P37. Domain: UTF-8 VRML
2.0 polygonal scenes, recursive node instances, exact numeric transforms,
selected-policy triangulation and omission diagnostics. Read both files above
in full. Supporting reads of native axis-angle construction and the already
audited shared face triangulator do not add whole-file kernel audit credit.

Missing witnesses: private tokens/nodes, payloads/fields, parser methods,
transform/flattening helpers and nearly all unit/context-test declarations lacked
Rustdoc. The module summary said unsupported geometry fails without explaining
the deliberately counted line/point and degenerate omissions. An area-test
comment overstated aggregate vector-area equality as proof of complete coverage.

Public docs now distinguish omitted line/point instances, degenerate rings and
degenerate emitted triangles. Indeterminate checks fail; a scene with no
surviving triangles remains a geometry error, not successful empty geometry.
This documents current behavior without claiming the repository's empty-result
target is thereby satisfied. Imported meshes retain missing source history and
any consumed predicate qualification; no whole-mesh closure claim is made.

Parser docs describe case-sensitive words, exact scalar tokens, string escaping,
last-value replacement and NULL fields that leave earlier non-NULL geometry or
coordinates intact. DEF/USE uses owned subtree clones and a document-wide name
table; definitions inside skipped appearance fields are not registered and
skipped USE names are not resolved. Brace-skipped rendering bodies do not receive
full grammar/semantic validation. These limits remain visible rather than being
presented as complete VRML validation.

Transform docs give `T C R SR S SR⁻¹ C⁻¹`, native checked rotation behavior,
unrestricted zero/negative scales at assembly and preserved index order under
reflections. Flattening appends every instance's positions, including unused
ones, and applies ccw reversal before shared triangulation. Larger rings search
fan triples in order, stopping on uncertainty; final emitted triangles receive
another degeneracy pass. Partial scratch output is discarded on failure.
Recursive parsing/flattening and cloned subtrees have no declared total resource
bound. Six tuple variants were expanded only to attach payload Rustdoc.

The full geometry-context file now documents actual evidence: strict/terminal
policy transitions, cache admissibility, local receipts, selected membership and
count checks, sampled-field output, Rust calls into JS-facing wrappers and
interchange continuation. DXF count/certainty does not prove source fidelity;
VRML's narrow area enclosure does not alone prove coverage/nonoverlap. The unit
near-pi/4 witness compares aggregate oriented vector area and certified outcomes;
it does not directly assert the normal-order predicate remains unresolved.
No assertions or fixture precision were changed to obtain these descriptions.

Validation after final edits:

- `rustfmt --check --edition 2024 --config skip_children=true src/io/vrml.rs
  tests/geometry_context.rs` and `git diff --check`: passed.
- Nonblank, non-comment source matches baseline after explicitly normalizing
  six tuple-payload layout expansions. No expressions, decision order, fixture
  values or assertions changed.
- File-local scans found Rustdoc on 49 VRML declarations and 23 geometry-context
  declarations. Variant payloads and fields were separately checked; this is not
  a repository coverage metric.
- With `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target`, `CARGO_INCREMENTAL=0`
  and `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`, ran
  `cargo doc --locked -p csgrs --no-default-features --features vrml-io
  --document-private-items --no-deps --offline`: passed without warnings.
  Inspected seventeen rendered pages, including parser methods and private
  variant/struct fields. Test-only declarations and local constants remain
  source-reviewed; the feature-gated context tests were not rerun.

Log: `/tmp/csgrs-rustdoc-vrml-doc.log`. Shared temporary build storage is about
816 MiB. Full workflow/runtime checks remain waived for this docs-only session;
no new runtime completion or performance claim is made.

### Gerber interchange and rounded-profile witnesses, 26 November 2026

Baseline: `d3edf129d`. Owners: CSGRS Gerber interchange adapter and sampled
profile consumers, P6/P8/P9/P19/P20/P25/P26/P30/P37. Domain: parsed Gerber modal
commands, standard aperture exposures, region contours, polarity/repetition and
finite output encoding. Read both files above in full. Supporting reads of
native contour/region admission, already audited modeling constructors/context
witnesses and the locked `gerber-types` coordinate serializer do not add
whole-file kernel or dependency audit credit.

Missing witnesses: private state, transform/coordinate records, fields and most
helpers lacked Rustdoc. Public import descriptions did not distinguish sampled
geometry from native decision certainty. Circular sweep comments overstated
topology preservation, and the radial allowance comment asserted a rounding
bound without supporting analysis. Documentation now describes the implemented
sampling and allowance without claiming an analytic-source certificate.

Import docs explain empty string/path channels, finite coordinate/image
arithmetic, sampled flashes/strokes, ignored stroke holes, ignored quadrant/EOF
commands, macro/block failures and parser-only unwind protection. Region closure
uses EvenOdd native admission; clear exposure flushes preceding balanced dark
unions before subtraction. Modal transform changes do not retroactively remap
stored positions. Strict hull predicates operate on promoted finite values;
image orientation classifies an already rounded determinant. Equal zero image
scales satisfy the circle-scale comparison without proving nonsingularity.

Export docs distinguish native finite projection from subsequent fixed-point
quantization. The locked serializer truncates to millionths before possible
coarser rounding. Duplicate suppression does not validate surviving topology,
area or vertex count; invalid raw digit formats are not comprehensively
prevalidated. Import/export have no total work/allocation bound. These are
recorded implementation limits, not changes to the repository's exactness,
closure, resource or explicit-failure targets. No algorithm was changed here.

The test documentation describes independent polygon areas and tangent points,
the translated hole/extrusion pipeline, exact dyadic OBJ positions, strict
retry after approximate clamp consumption, and capacity priority. The Gerber
roundtrip witness checks selected boundary/inside/outside points; it does not
prove full source equality or an analytic error bound.

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on both
  Rust files and `git diff --check`: passed.
- Removing only blank/comment lines gives identical source to baseline for both
  files. No executable expression, fixture, assertion or predicate order changed.
- File-local scans found Rustdoc on 72 Gerber declarations and 15 test/helper
  declarations. Fields were checked separately; this is not repository coverage.
- With `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target`, `CARGO_INCREMENTAL=0` and
  `RUSTDOCFLAGS='-D rustdoc::broken_intra_doc_links'`, ran
  `cargo doc --locked -p csgrs --no-default-features --features gerber-io
  --document-private-items --no-deps --offline`: passed without warnings.
  Inspected 53 rendered pages, including private methods and fields. Initial
  text-check mismatches came from inline-code spacing and Rustdoc's typographic
  apostrophes; normalizing those presentation differences passed all checks.
  Test-only items remain source-reviewed, not rerun.

Log: `/tmp/csgrs-rustdoc-gerber-doc.log`. Shared temporary build storage is about
831 MiB. Full workflow/runtime checks remain waived for this docs-only session;
no new runtime completion or performance claim is made. All root `src/io/*.rs`
files now have recorded full-file reviews; this does not cover every downstream
consumer, dependency or kernel interchange implementation.

### WASM values, query receipts and interchange diagnostics, 26 November 2026

Baseline: `38b5982d1`. Owners: CSGRS JS value/receipt/interchange boundaries,
P1/P6/P8/P9/P19/P26/P28/P30/P37. Domain: finite JS numeric ingress, native
symbolic values entering through Rust conversions, point/vector/matrix/plane
operations, completed/unresolved queries and format errors. Read the seven
edited WASM files and the Rust certainty test in full, plus the entire empty
voxel module. Supporting reads of native vector normalization/angle, native
plane construction/reflection, construction evidence, mesh/curve query callers
and the TypeScript certainty consumer do not add whole-file kernel, mesh/curve
wrapper or TypeScript audit credit.

Missing witnesses: value types/fields, most exported operations, private scalar
and quaternion helpers, receipt conversions/accessors and tests lacked Rustdoc.
Existing certainty wording conflated predicate decisions with topology proof;
a vector comment said native arithmetic returned finite components. Both now
describe the implemented boundary precisely.

The docs distinguish exact binary64 ingress from lossy getters, including NaN
fallback when no finite view is available. Native Rust conversions bypass finite
projection and do not validate arbitrary native carriers. Matrix construction
documents the actual positional row order despite irregular historical parameter
names; it imposes no affine/invertibility constraint. Vector equality continues
after an unequal component until all comparisons finish or one is undecided.
Rotation docs state radians, scalar-first quaternion normalization, the exact
antipodal branch, and finite quaternion export without re-normalization.

Plane docs specify `n·p + offset = 0`, non-unit normals, point-order orientation,
normal validation before offset ingress and native reflection. Local receipts
are exposed by context results but discarded by bare plane/matrix extraction;
those value wrappers store no inherited evidence. Query docs distinguish
boundary, outside/empty, curve uncertainty and mesh Unknown. Construction flags
remain a dependency summary rather than a replayable proof or source-error bound.

Interchange docs describe category/field preservation and the narrower named
message causes; nested native kernel payloads are not all exported. TypeScript
readonly declarations do not freeze host Error properties. Reflection exceptions
panic in the property helper, while a false assignment result is unchecked.
These flattened-error, NaN and receipt-loss behaviors remain implementation gaps
against the broader targets; documenting them does not make them compliant.

Unit/integration witness docs separate host Rust calls from generated JavaScript
execution. Historical test names mentioning hostile axes or cross products now
state which promotion checks they actually perform. Manually supplied receipt
tests are distinguished from the hull witness that actually consumes a terminal.
No runtime implementation, assertions, signatures or TypeScript declarations changed.

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on all
  eight edited Rust files and `git diff --check`: passed.
- Removing only blank/comment lines gives identical source to baseline for each
  Rust file. Declaration scans found documented counts of 16/10/9/35/16/36/5 in
  module/point/matrix/vector/plane/context/IO files and seven in the Rust test.
  Private fields and enum variants were checked separately.
- With the shared temporary target, incremental compilation disabled and broken
  Rustdoc links denied, ran `cargo doc --locked -p csgrs --no-default-features
  --features wasm,dxf-io,gerber-io --document-private-items --no-deps --offline`
  and the same command with `--features wasm`: both passed without warnings.
- Inspected 34 rendered module/type/helper pages, including private fields,
  methods, exported contracts and ordinary-feature output. The voxel placeholder
  module renders as supplied. An initial attempt to read that page occurred while
  Rustdoc was regenerating output; the read passed after the build completed.
  The macro-consumed TypeScript-section constant and test-only items remain
  source-reviewed rather than ordinary rendered Rustdoc items.

Logs: `/tmp/csgrs-rustdoc-wasm-values-doc.log` and
`/tmp/csgrs-rustdoc-wasm-values-minimal-doc.log`. Shared temporary build storage
is about 1016 MiB. Full workflow/runtime checks remain waived for this docs-only
session. These host documentation builds do not establish wasm32 execution,
generated-package TypeScript validation, browser behavior or a new performance claim.

### WASM mesh/curve modeling and generated-binding consumers, 26 November 2026

Baseline: `3826b2051`. Owners: CSGRS JS modeling, finite display/interchange and
receipt boundaries, P1/P6/P8/P9/P19/P20/P26/P28/P30/P37. Domain: native filled
regions and triangle meshes, exact finite ingress, sampled constructors,
Boolean/transform/meshing chains, finite arrays/profiles, bounded closed-PWN
sums and immutable STL artifacts. Read both edited Rust files completely.
Supporting reads of already audited modeling APIs and native contour admission,
mesh construction history and Minkowski options do not add whole-file kernel
credit.

Missing witnesses: most exported methods and all private helper/record fields
lacked Rustdoc. The region closure description named the wrong evaluation
entry point; it now describes the actual selected-policy `evaluate_under` path.
The STL binding description now states exactly which category/message survives
in its cause, rather than implying every native typed payload crosses JS.

Curve docs state native region authority, exact finite coordinate promotion,
flattened material/hole roles, contour validation before the selected-policy
region closure, empty results and string-only errors in legacy paths. Display
profile conversion drops chord-error metadata/history. Triangle arrays expose
local certainty but not the native mesh's construction summary; finite position
views can round or become NaN, and index casts are unchecked on wider hosts.
No post-projection topology/fidelity proof is supplied. Constructors document
actual polygon families, sample minima, dimension validation and origin/phase.
Extrusion, sweep, revolution and twisted extrusion link their native domains,
including signed radii, axis crossings, frame seams, non-embedding possibilities
and resource limits. Outcome extraction consumes its handle and drops the
separate local receipt while moving existing native evidence unchanged.

Mesh docs distinguish raw-array admission from closed-solid validation and
triangle-buffer emptiness from canonical empty geometry. Bounds include unused
positions. Display normals are finite normalized sums of unit face normals;
failed face normalization is skipped, and display arrays perform no topology
admission. Count getters saturate on wider hosts while index export casts do
not. Transform docs explain singular/infinity rejection and reflected winding;
inverse reverses orientation without constructing an unbounded complement.
Minkowski represented-set sums remain distinct from stored-position hulls, with
count units and the absence of total work/byte bounds or exposed cancellation.
STL artifact docs distinguish immutable bound receipts from copied raw bytes.

Additional complete consumer reviews (non-Rust files):

| File | Review and change | Validation scope |
| --- | --- | --- |
| `tests/wasm/geometry_certainty.cjs` | Entire file; documented handle helpers and witness limits; replaced assertions about internal proof dispatch/resource behavior with the actual observed rotation/query contracts. | Comment-only comparison passed; fixtures/assertions unchanged. Node syntax/runtime execution unavailable in this session's PATH. |
| `tests/wasm/geometry_certainty.ts` | Entire file; documented exhaustive query formatter and generated declaration/ownership consumer scope. | Comment-only comparison passed; generated-package type checking not rerun. |
| `tests/wasm/mesh_errors.cjs` | Entire file; documented failure capture, host error assertions and raw open/collinear fixtures. Reviewed every invalid/empty, membership, lifetime and STL continuation witness. | Comment-only comparison passed; Node syntax/runtime execution unavailable in PATH. |
| `tests/wasm/mesh_errors.ts` | Entire file; documented assertion narrowing and typed error capture; reviewed lifetime and empty STL continuation. | Comment-only comparison passed; generated-package type checking not rerun. |

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on both
  Rust files and `git diff --check`: passed.
- Removing only blank/comment lines gives identical source to baseline for all
  six edited source/consumer files. No assertion, fixture, signature, expression
  or decision ordering changed. Local scans found 54 documented curve-binding
  declarations and 69 mesh-binding declarations; private fields were checked
  separately.
- With the shared temporary target, incremental compilation disabled and broken
  Rustdoc links denied, ran `cargo doc --locked -p csgrs --no-default-features
  --features wasm --document-private-items --no-deps --offline`: passed without
  warnings. Inspected seventeen rendered pages and all 123 declaration summaries
  plus field/method contract text. The text checker was adjusted for bracketed
  external-link rendering; source contracts did not change to satisfy it.
- Exact rational conversion of the binary64 literals confirms the documented
  `13.1 + 0.5` fixture sum equals the exact sum of those two finite input values.
  This arithmetic check does not establish native angle-addition completion.
- Attempted `node --check` on both edited CJS files: unavailable because `node`
  was not found; `nodejs` was also absent from PATH. No JS runtime, TypeScript
  package, wasm32 or browser result is claimed. Runtime/full workflow checks
  remain waived for this docs-only session.

Log: `/tmp/csgrs-rustdoc-wasm-modeling-doc.log`. The same approximately 1 GiB
temporary build directory was reused. All root `src/wasm/*.rs` files now have
recorded whole-file reviews; remaining JS consumers and lower-layer binding
implementations are still in scope. No new performance claim is made.

### Remaining WASM consumer corpus and runner, 26 November 2026

Baseline: `5f720a75b`. Owners: generated-binding acceptance witnesses and their
test harness, P6/P9/P19/P20/P26/P28/P30/P37. Domain: binary32/decimal STL
boundaries, retained construction summaries, raw buffer rejection, host error
lifetime and strict interior of regularized nonzero-winding box fixtures.
Read every file listed below in full. Supporting reads of already audited STL
artifact methods and the workflow's WASM job do not add whole-file workflow
or kernel credit.

Missing witnesses: private test helpers and runner tool/ownership assumptions
lacked explanatory documentation. Added JSDoc/comments describing actual
assertions and limits without changing test behavior. This is non-Rust source
coverage; it does not contribute to a Rust item-coverage percentage.

| File | Review and change | Validation scope |
| --- | --- | --- |
| `tests/wasm/construction_evidence.ts` | Entire file; documented the predicate-history helper and distinction among local receipts, missing history, projected artifacts and analytic fidelity. Reviewed all extraction, mutation-of-copy, reimport and continuation checks. | Comment-only comparison passed; generated TypeScript/runtime execution not rerun. |
| `tests/wasm/export_projection.cjs` | Entire file; documented owned-handle transfer, exact BigInt decimal oracle, textual row extraction and finite geometry witnesses. Reviewed immutable copies, collapse/collinearity rejection, cache churn and tiny-cube continuation. | Comment-only comparison passed; Node execution remains unavailable in PATH. |
| `tests/wasm/containment_winding.cjs` | Entire file; documented signed-shell construction and the independent eight-cell nonzero-winding oracle for integer box planes and half-grid queries. Reviewed all cancellation, cavity, overlap, tangent and nearby-boundary witnesses. | Comment-only comparison passed; Node execution not rerun. |
| `tests/wasm/io_contracts.cjs` | Entire file; documented scoped ownership, capture/error helpers, raw-triangle fixture and normalized cube oracle. Reviewed every format/payload/lifetime/empty/retry witness. | Comment-only comparison passed; Node execution not rerun. |
| `tests/wasm/io_contracts.ts` | Entire file; documented exhaustive discriminant handling, assertion narrowing, contract-based error cast and consuming handle transfer. | Comment-only comparison passed; generated TypeScript/runtime execution not rerun. |
| `tests/wasm/package.json` | Entire file; private test package pins only TypeScript 5.7.3; no scripts or publishing behavior. No edit needed. | JSON parsed; package identity/version/dependency pin matches lockfile. |
| `tests/wasm/package-lock.json` | Entire file; reviewed root package, single dev dependency, registry/integrity record, executable mappings, license and Node engine metadata. No edit needed. | JSON parsed and pinned version agrees with manifest; no registry download/integrity verification or dependency upgrade claimed. |
| `scripts/test-wasm-bindings.sh` | Entire file; documented prerequisites/overrides, repo-relative paths, Cargo target-directory selection, generated Node artifact and the type-check-only consumer. Reviewed all build, CJS, TS compile and emitted-JS execution commands. | `bash -n` and comment-only comparison passed; full runner not executed. |

The winding oracle's finite offsets are appropriate to this fixture domain:
queries lie on the half grid and all box planes have integer coordinates, so
query-plus/minus-one-eighth samples inspect every incident octant without
crossing an unrelated plane. This explains the independent expected membership;
it is not a generic sampled test for arbitrary solid boundaries. Cube checks
separately require finite corner values, outward axis faces, edge pairing and
integer normalized volume. Neither volume nor selected membership alone is
described as a universal geometry proof.

The runner changes directory before interpreting relative tool paths, requires
the bindgen CLI version from Cargo.lock, uses Cargo metadata for build storage,
and generates the actual Node binding. It runs five CJS consumers, only
type-checks `geometry_certainty.ts`, and compiles/executes the other three TS
consumers. Documentation now makes that distinction visible. No command,
assertion, fixture, package version or workflow configuration was modified.

Validation after final edits:

- `bash -n scripts/test-wasm-bindings.sh` and `git diff --check`: passed.
- Comment-only comparisons against baseline passed for all six edited consumer/
  runner files, including the embedded Python and all shell commands unchanged.
- Parsed both package JSON files and verified matching names, versions and the
  exact TypeScript 5.7.3 pin. The workflow's bindgen CLI version matches the
  locked Rust crate version 0.2.126. This is local consistency evidence only.
- Node/TypeScript/generated-WASM runtime checks remain unexecuted; Node was
  already confirmed absent from PATH in the preceding batch. Full workflow
  checks remain waived for this docs-only session. No Rust source changed, so
  the preceding private Rustdoc build was not repeated for this comment batch.

All eleven tracked files directly in `tests/wasm/` now have recorded full-file
reviews across the two batches. The shell runner has its own full-file record;
the rest of the workflow and repository remain in scope. No build trees or
generated artifacts were added, and no new runtime or performance result is claimed.

### Parts metadata, attachment, assembly and BOM, 26 November 2026

Baseline: `affc4070a`. Owners: csgrs parts metadata and documentation assembly,
P3/P6/P8/P9/P19/P25/P30/P37. Domain: authored node trees, exact native frame
arithmetic, selected documentation poses, explode guides and finite BOM counts.
Read the seven files recorded above in full, including private helpers and unit
tests. Partial reads of blueprint/manual consumers provide context but do not
earn whole-file review credit for those modules.

Missing witnesses were absent private Rustdoc and claims stronger than the
implementation: frame mating matches normalized axis directions, certainty
labels are authored metadata rather than geometric proofs, and installation
presence is not replay validation. Public mutation and cloned nodes can violate
the intended acyclic tree. Root poses start a selected documentation frame;
ancestor placements and the selected node's placement are not reapplied. Counts
affect BOM quantities rather than geometric instance expansion. Explode blockers
can accompany retained partial output and do not constitute complete geometry
validation. These implementation limits are documented, not repaired or accepted
as satisfying the wider geometry contract.

BOM traversal follows step children only. Missing/overflowed step quantities
become zero, failed part counts omit lines, and flattened sums saturate. Hidden
steps remain; references and hidden parts are omitted. Merged columns concatenate
lines without aggregation, and a merged root has no column. The example uses
cuboid stand-ins, writes SVG/Markdown into the manifest-root `target` directory,
and does not prove hole construction, STL export or general attachment behavior.

Validation after edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on all
  seven Rust files and `git diff --check`: passed.
- Documentation-only comparisons against baseline passed. Three tuple
  declarations were expanded solely to document payloads; their new trailing
  commas/layout were explicitly normalized. No executable expression, signature,
  assertion, fixture or decision order changed.
- Local declaration scans found documentation for all 111 declarations in the
  seven files; fields and variants were checked separately. Test-only items were
  inspected in source. This is batch evidence, not repository-wide coverage.
- Reused the shared temporary target with incremental compilation disabled and
  `-D rustdoc::broken_intra_doc_links`: `cargo doc --locked -p csgrs
  --no-default-features --document-private-items --no-deps --offline` passed, as
  did the same command with `--features attributed`. The latter ran after the
  final BOM root-merge clarification; neither build emitted warnings.
- Inspected 66 rendered module/type/helper pages, all 82 non-test declaration
  summaries, private payloads/fields and material contract text. The final BOM
  sentence was checked again in attributed output. An initial follow-up used
  the wrong function filename; correcting the inspector path required no source
  change.
- Runtime tests, the example and full workflows were not run under the owner's
  docs-only waiver. No performance, runtime completion or universal proof claim
  is made. One approximately 1.1 GiB temporary build tree remains in use.

Logs: `/tmp/csgrs-rustdoc-parts-assembly-doc.log` and
`/tmp/csgrs-rustdoc-parts-attributed-doc.log`.

### Dimension annotations and Markdown manual, 26 November 2026

Baseline: `681b832fe`. Owners: parts drawing annotations and manual formatting,
P3/P6/P8/P9/P19/P25/P29/P30/P37. Domain: native measured segments, orthographic
projection, optional display glyphs and authored step/BOM Markdown. Read both
files above in full, including all feature branches and tests. Supporting reads
of projection/edge helpers, reserve logic, Hershey entry points and release
overflow configuration do not count as whole-file audits of those dependencies.

Missing acceptance witnesses were undocumented private helpers and incomplete
public contracts. Dimension admission checks each component sign, rather than
only the resulting length. Public fields can violate the constructor's distance
relation. Annotation does not revalidate it; returned errors precede appending
edges. Optional labels use lossy finite binary64 text, then exact display-stroke
placement. A missing glyph center silently omits the label; undecided extrema
comparisons retain earlier candidates. These paths are documented without
claiming a general certified bounds result or altering their implementation.

Manual output embeds text and callback paths verbatim without filesystem access,
escaping or anchor collision resolution. Blank step names remain in the contents
but receive no section. Merged steps still receive sections. The global table
uses retained columns, not the flattened BOM: duplicate sums within a column use
ordinary integer arithmetic, while totals across columns saturate. Neither the
blocker list nor generated text validates counts, geometry, pictures or Markdown.
Existing tests cover selected strings/edges and one label-failure atomicity case;
they do not establish all these domains or a complete independent output oracle.

Validation after final source edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` and
  `git diff --check`: passed.
- Removing only blank/comment lines yields identical Rust source to baseline
  for both files. No expressions, assertions, fixtures, signatures or predicate
  ordering changed. Local scans found 9 documented manual declarations and 22
  dimension declarations, including feature alternatives/tests; fields were
  separately inspected.
- With the shared temporary target, incremental compilation disabled and broken
  Rustdoc links denied, `cargo doc --locked -p csgrs --no-default-features
  --document-private-items --no-deps --offline` passed, as did the same command
  with `--features hershey-text`. Neither emitted warnings. Builds were repeated
  only to inspect both feature-dependent versions in the reused output tree.
- Inspected 23 rendered module/type/function pages, 21 enabled declaration
  summaries, all record fields and material contract text, plus the minimal
  no-text label stub. The text inspector initially introduced whitespace around
  inline code punctuation; normalizing that presentation difference fixed the
  check without changing source documentation.
- Unit tests and full workflows remain unexecuted under the owner's docs-only
  waiver. No runtime, performance or mathematical completion claim is made.

Logs: `/tmp/csgrs-rustdoc-dimension-manual-minimal-doc.log` and
`/tmp/csgrs-rustdoc-dimension-manual-hershey-doc.log`. No generated artifacts were
added to the repository; the same temporary build tree remains in use.

### Bounds blueprints and posed-mesh occlusion, 26 November 2026

Baseline: `48bbe34d5`. Owners: csgrs parts drawing projection/visibility,
P3/P6/P8/P9/P19/P20/P25/P26/P29/P30/P37. Domain: authored axis-aligned bounds,
posed indexed triangle meshes, orthographic feature/guide segments and terminal
SVG display. Read both modules in full, including every helper, field, variant
and unit witness. Supporting reads of native mesh construction/dihedral,
homogeneous transforms, AABB and the already-reviewed example do not add
whole-file kernel audit credit.

Missing acceptance witnesses were undocumented private items and overbroad
certification descriptions. The bounds path draws projected rectangle outlines,
requires ordered bounds/distinct handles without validating them, and uses
non-strict depth separation despite its existing "strictly nearer" note. The
metadata adapter uses only the first face's metadata and drops all output on any
input blocker. Successful bounds reports hard-code `NativeExactCsg` independently
of authored certainty and unknown edges. SVG exports finite lossy coordinates,
but its padded view-box arithmetic is unchecked and it omits exact definitions,
notes and certainty. Documentation describes these facts without changing output.

The mesh path uses vertex-index adjacency, not geometric welding. Local transformed
meshes are constructed with untracked history, native angle receipts are extracted
without retention, and report certainty ranks authored node labels separately.
Partial output survives missing nodes/transforms/indices; not every unknown edge
adds a blocker. Only one pose slot is populated. Boundary hits use shared-sample
incidence and opposite triangle sides; they do not require identical edge endpoints.

Most significantly, projected triangle-boundary crossings are the only split events:
the code adds no interior depth-equality event, and collinear segments report no
intersection. Extending a midpoint decision to a whole piece assumes constant
coverage/depth order between cuts, which is not validated. This is established by
inspection of the current event construction and classifier; no new executed Rust
counterexample or universal visibility theorem is claimed. The documentation now
states this implementation gap rather than describing every emitted `ExactMesh`
status as whole-piece certification. The existing box fixtures do not cover that
general intersecting-surface domain. No algorithm, fixture, assertion or policy
was weakened or repaired in this documentation-only batch.

Validation after final source edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` and
  `git diff --check`: passed.
- All non-comment lines match baseline after explicitly collapsing only three
  tuple-payload layouts expanded for their field documentation. A second
  whitespace-normalized comparison also passed. No expressions/signatures or
  predicate order changed. Local scans found 45 documented blueprint declarations
  and 33 occlusion declarations, including tests and the nested rank helper.
- With the shared temporary target, incremental compilation disabled and broken
  Rustdoc links denied, `cargo doc --locked -p csgrs --no-default-features
  --features attributed,hershey-text --document-private-items --no-deps --offline`
  passed, as did the same command without the feature argument. The first build
  exposed an incorrect relative module-doc link; it was corrected to the public
  `crate::parts` path before both final builds passed without warnings.
- Inspected 61 rendered pages and 61 non-test declaration summaries, plus private
  fields, variants/payloads and material contract text. Nested rank/test items
  remain source-reviewed. Source comparison and formatting passed after final edits.
- Runtime tests/full workflows remain waived for this docs-only session. No new
  runtime, performance or mathematical completion claim is made. The same temporary
  build directory was reused, without repository build artifacts.

Logs: `/tmp/csgrs-rustdoc-blueprint-occlude-feature-doc.log` and
`/tmp/csgrs-rustdoc-blueprint-occlude-minimal-doc.log`.

All 53 tracked files under the root package's `src/` now have whole-file review
rows in this ledger. This was checked against `git ls-files src`; it is review
coverage for that directory, not a repository-wide item-coverage percentage or
proof that every feature combination has been built. Remaining crates, tests,
examples, build/configuration files and documentation are still in scope.

### Blueprint benchmark/fuzz consumers and shared harnesses, 26 November 2026

Baseline: `66d0dc12d`. Owners: development workload/validation consumers,
P6/P8/P9/P19/P20/P29/P30/P35/P36/P37. Domain: integer cube bounds reports,
environment-selected benchmark repetitions, measurement counters and native mesh
assertions. Read all four files listed above in full. Supporting reads of target
manifest entries, benchmark/fuzz callers, kernel mesh-validation methods and
dispatch reset/correlation code provide context only, not additional whole-file
review credit.

Missing acceptance witnesses were absent private helper/field docs and incomplete
timing/input-domain descriptions. The part benchmark uses a separate fixed loop,
not the shared configurable harness: 64 cubes are constructed before timing and
512 front-view calls count both output slots. No independent visibility oracle,
blocker check, certainty check or sample distribution is asserted. The fuzz target
checks counts/handles/notes and an exhaustive style match, not geometric visibility.
Its `chunks(6)` loop indexes incomplete final records for nonmultiples of six
between 6 and 71 bytes; this is an existing harness gap. A standalone byte-chunk
calculation confirmed all 55 affected lengths below 72, without running Rust or
geometry. The target/fixtures/assertions were preserved; fixing that gap is outside
this docs-only batch.

The shared benchmark harness now documents live environment selection versus
captured counts, temperature labels without cache control, ordinary/stress sample
plans, warmup inclusion in traces, caller-timed versus wall-clock timing boundaries,
saturating counters and wrapping checksums. The trace assertion includes a synthetic
entry event and alone does not establish native geometry dispatch. CSV labels are
unescaped. Shared fuzz validation asserts finite positions, valid indices and
unique/nondegenerate triangles, with optional nonempty edge-pairing checks; it does
not establish source fidelity, non-self-intersection or intended Boolean semantics.

Validation after final edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on all four
  files and `git diff --check`: passed.
- Removing only blank/comment lines yields identical source to baseline in all
  four files. No fixture, assertion, timing loop, environment interpretation or
  predicate order changed. Scans found all 28 declarations documented, with
  record fields separately inspected and macro-body intent in module docs.
- Standalone `rustdoc --edition 2024 --crate-name csgrs_bench_harness
  --document-private-items -D rustdoc::broken_intra_doc_links
  benchmarks/support/harness.rs -o /tmp/csgrs-rustdoc-target/doc` passed.
  Repeated with `--cfg 'feature="dispatch-trace"'`, the existing dispatch-enabled
  hyperreal metadata and cached dependency search paths: passed. No dependency
  rebuild or extra build tree was needed. Both final logs are empty (no warnings).
- The initial builds warned about a public constant linking to private `emit`;
  its prose was corrected and both configurations rebuilt. Inspected 13 rendered
  pages, all 20 harness declaration summaries, private fields and timing/trace
  contracts; checked the final constant prose again after that edit.
- Benchmarks/fuzz targets were not executed or newly compiled, and standalone
  helper documentation does not establish those target configurations build or
  complete. Full workflows remain waived. No new performance/fuzz-completion
  claim is made.

Logs: `/tmp/csgrs-rustdoc-bench-harness-doc.log` and
`/tmp/csgrs-rustdoc-bench-harness-trace-doc.log`.

### Primitive, transform and profile examples, 26 November 2026

Baseline: `284f4ada4`. Owners: root-package executable API examples,
P3/P6/P8/P9/P19/P25/P26/P29/P30/P37. Domain: strict-session construction,
scalar-adapter bounds, sampled primitives/profiles, native transforms and
Boolean/hull/Minkowski composition, followed by terminal binary STL. Read all ten
files above in full, including every constant, alias and helper. Supporting reads
of manifest feature requirements, already-reviewed native wrappers and kernel
adjacency/offset contracts do not add whole-file review credit elsewhere.

Missing acceptance witnesses were absent main/helper/module docs and incomplete
claims about mathematical source and output behavior. Documentation now separates
sampled circles/spheres/revolutions from analytic geometry, exact binary64 ingress
from decimal interpretation, local/session receipts from terminal export, and
fixture-specific assertions from general shape correctness. Ring/keyhole examples
permit `APPROXIMATE_512` and extract values without reporting receipts; later strict
triangulation does not independently certify earlier decisions. The sweep path is
computed in binary64 before native promotion. The prior revolution comment asserted
a universal one-sided-axis prerequisite inconsistent with the current full-turn
implementation; it now explains the actual torus fixture's placement.

Output paths are relative to the process working directory. Writers replace files,
do not atomically publish a whole example's outputs and leave earlier artifacts
after a later panic. The strict-session example propagates modeling/export/IO
errors, while most other examples unwrap them. No executable behavior changed.

Validation after source edits:

- Focused `rustfmt --check --edition 2024 --config skip_children=true` on all ten
  files and `git diff --check`: passed after the final edit.
- Removing only blank/comment lines gives identical source to baseline in all
  ten files. All 31 constants/aliases/functions have Rustdoc; no expressions,
  assertions, fixture values, policies or feature gates changed.
- Reused `/tmp/csgrs-rustdoc-target` with incremental compilation disabled and
  broken Rustdoc links denied. `cargo doc --locked -p csgrs --no-default-features
  --features curve,stl-io,offset --document-private-items --no-deps --offline -j 2`,
  selecting all ten files by repeated `--example NAME`, passed. The same selection
  with `--features curve,stl-io` passed after the last comment edit. Both logs were
  checked and contain no warnings. The rendered source includes the corrected
  revolution comment.
- Inspected 41 rendered pages (ten example roots and 31 declarations), including
  feature, sampling, output, matrix-order and receipt contracts. This verifies
  documentation generation, not execution of modeling operations or assertions.
- No examples, runtime tests or full workflows were run under the owner's
  docs-only waiver. No STL output was generated. Cached development dependencies
  increased the single temporary build tree to approximately 1.3 GiB; no duplicate
  tree or repository build artifacts were added.

Logs: `/tmp/csgrs-rustdoc-small-examples-doc.log` and
`/tmp/csgrs-rustdoc-small-examples-no-offset-doc.log`.

### Multi-format example and embedded format witnesses, 26 November 2026

Baseline: `039069bc0`. Owner: root executable export example and its embedded
test module, P6/P19/P25/P26/P29/P30/P37. Domain: sampled mesh/Boolean fixtures,
independently enabled OBJ/PLY/AMF writers and selected textual-format assertions.
Read the entire file, including all feature alternatives and eight tests; checked
the existing AMF unit/export contracts for context. Missing witnesses were absent
main/helper/test Rustdoc and overbroad claims about output and format correctness.

Documentation now states that geometry runs even with every format disabled,
the final summary is a list of possible artifacts/applications rather than
verified files/compatibility, and each enabled path creates/truncates its file
before invoking the writer. Subsequent statistics generation repeats serialization
and may fail after the file has been written. Row/tag counts are not independent
geometry/format parsing. The AMF unit is a label without coordinate rescaling.
Test comments claiming parsed XML hierarchy or general Boolean export correctness
were corrected to describe the actual selected-token assertions.

The enabled test module has an existing compilation gap: eight method-style
export calls (`to_obj`, `to_ply`, `to_amf`, `to_amf_with_color`) are unavailable on
the current `TriangleMesh`, and the color test references undefined helper `r`
three times. Reproduced on both baseline source and current comments using
`rustc --edition 2024 --test --emit=metadata`, all three format cfgs, the exact
cached csgrs metadata from the feature-enabled doc build, and cached dependency
search paths. Both produced the same eleven diagnostics: eight E0599 and three
E0425. No executable was linked or run. The first direct compiler attempt lacked
dependency directories containing non-rmeta artifacts and stopped at E0463;
adding all cached dependency output directories resolved that inspection setup
issue and exposed the matching source errors. These test blockers are preserved
and documented under the owner's docs-only constraint, not repaired or reported
as passing witnesses.

Validation:

- Focused formatting and `git diff --check` passed after final edits. Removing
  blank/comment-only lines gives identical source to baseline. All eleven
  function/module declarations are documented; no expressions, assertions,
  feature gates, format strings or sample values changed.
- With the existing target and incremental compilation disabled, broken-link-denied
  `cargo doc --locked -p csgrs --no-default-features --features obj-io,ply-io,amf-io
  --example multi_format_export --document-private-items --no-deps --offline -j 2`
  passed. The same command without the feature argument also passed. The enabled
  build was repeated after the final test-module documentation edit; no warnings
  were emitted. Normal Rustdoc excludes the blocked test module.
- Inspected all three rendered example/main/helper pages and their feature,
  output, unit and statistics contracts. Test documentation was inspected in
  source, with its build blockers explicitly stated. Text inspection normalized
  typographic apostrophes; no source wording was altered to satisfy presentation.
- Runtime examples/tests and full workflows remain unexecuted under the owner's
  waiver. No model files or additional build tree were created. Documentation
  checks passed; feature-enabled test compilation remains a reproduced baseline
  failure, not a new failure caused by these comments.

Logs: `/tmp/csgrs-rustdoc-multi-format-doc.log`,
`/tmp/csgrs-rustdoc-multi-format-minimal-doc.log`, and
`/tmp/csgrs-rustdoc-multi-format-test-{baseline,current}.log`.

### README quickstart synchronization correction, 26 November 2026

Baseline: `56466ea99`. Owner: README/example consistency witness, P37. Read
`tests/readme.rs` in full and the marked README quickstart block in context.
This is not whole-file review credit for `readme.md`. The earlier example audit
added Rustdoc to `examples/basic.rs` but missed its downstream byte-equality
contract with the README snippet. Reproduced the resulting failure: quickstart
equality failed while the release-metadata test passed. This regression was
caused by the documentation batch, not a pre-existing implementation blocker.

Copied the complete documented example into the existing marked fence. Verified
the README prefix/suffix are byte-identical to baseline, so no text outside that
block changed. Added Rustdoc to the two test constants and two test functions;
their executable lines and assertions remain identical to baseline.

Compiled `tests/readme.rs` directly with `rustc --edition 2024 --test`, supplying
`CARGO_PKG_VERSION` from the current manifest, then ran its two tests. Both passed.
This dependency-free check executes the unchanged source assertions without
building the broader Cargo test graph. Focused Rustfmt and `git diff --check`
also passed. No model rendering, geometry operations, asset writes or full
workflow checks were run. Future edits to `examples/basic.rs` must repeat this
downstream synchronization check even when only comments change.

Logs: `/tmp/csgrs-rustdoc-readme-before.log` and
`/tmp/csgrs-rustdoc-readme-after.log`.

### README renderer and image-regression consumers, 26 November 2026

Baseline: `38346b4b4`. Owners: display-only example and asset-regression consumers,
P3/P6/P8/P9/P19/P25/P26/P29/P30/P37. Domain: fixed primitive/profile/text/implicit
fixtures, finite projection, PNG rasterization and filename/byte comparisons.
Read the entire 1,313-line baseline example and the entire regression test file.
Supporting reads of finite projection/triangulation, scalar conversion and feature
declarations do not add whole-file kernel or manifest credit.

Missing acceptance witnesses were undocumented private declarations and stronger
display claims than the implementation establishes. Finite fixture floats enter
exact native scalars; binary64 raster math follows explicit projection. Several
builders permit approximate predicates and discard operation receipts. Projection
retains material/hole decisions before rasterization, but pixels and depth-biased
overlays are not native visibility certificates. Documented every raster threshold,
marker/edge selection, camera map, color/coverage calculation and private record.
The normalizer's finite-value limits and the display conversion's lack of an extra
finite filter are explicit. Selected endpoints lacking coordinates remain unmarked;
display subdivision vertices do not become authored marker identities.

Output directory overrides are interpreted relative to the working directory;
text-only selection depends on presence, not value. Saves replace files individually
and can leave partial batches. The regression harness separately writes to a
repository-root `target/readme-render-regression/PID-NANOS`, ignores
`CARGO_TARGET_DIR`, retains its outputs and inherits environment/default features.
Its existence/lexical-name/byte checks do not prove geometric fidelity or parse
Markdown/PNG semantics. No renderer logic, fixture, threshold or assertion changed.

Feature finding: the example's manifest lists six required features but omits
`hershey-text`, used unconditionally by text rendering. With defaults disabled and
only the listed six, a focused `cargo check --example readme_renders` reproduced
E0433 for the gated Hershey module and E0425 for `hershey_strings`. The same check
with `hershey-text` passed. This is an existing source/manifest mismatch; comment-only
comparison confirms unchanged executable source. The manifest was not changed in
this docs-only worktree. A Rustdoc build with the six features had succeeded,
demonstrating why documentation generation alone must not be reported as a full
type-check of example bodies.

Validation after final edits:

- Focused Rustfmt and `git diff --check`: passed. Removing blank/comment-only lines
  yields identical source to baseline for both files. Scans found all 73 renderer
  declarations and all six regression declarations documented; fields were checked
  separately. Test bodies and all numeric constants remain unchanged.
- With the existing temporary target, incremental compilation disabled and broken
  Rustdoc links denied, `cargo doc --locked -p csgrs --no-default-features
  --features curve,image-io,truetype-text,metaballs,sdf,offset,hershey-text
  --example readme_renders --document-private-items --no-deps --offline -j 2`
  passed without warnings. Focused `cargo check` with the same feature/target
  selection also passed. The six-feature check remains a reproduced existing gap.
- Inspected 69 rendered pages, 71 non-test declaration summaries, private fields
  and material contracts. The endpoint-marker unit witness remains source-reviewed;
  no new native geometry or rendering execution is claimed.
- Compiled `tests/readme_render_regression.rs` directly with `rustc --test`, the
  file's six cfg features and `CARGO_MANIFEST_DIR` set to the repository root.
  Ran only `readme_referenced_png_baselines_exist`: passed (one other test filtered).
  This executes the unchanged existence assertions without invoking Cargo rendering.
- The ignored byte-for-byte render test, example execution and full workflows were
  not run under the owner's docs-only waiver. No PNGs or repository output directory
  were created. Cached artifacts stayed in the single temporary build tree.

Logs: `/tmp/csgrs-rustdoc-render-missing-hershey.log`,
`/tmp/csgrs-rustdoc-render-missing-hershey-check.log`,
`/tmp/csgrs-rustdoc-readme-renders-doc.log`,
`/tmp/csgrs-rustdoc-readme-renders-check.log`, and
`/tmp/csgrs-rustdoc-render-references.log`.

All 13 tracked Rust files under the root `examples/` directory now have whole-file
review rows, checked against `git ls-files examples`. This is directory review
coverage, not proof of all example/test feature combinations or repository-wide
Rustdoc completeness. The recorded multi-format test and renderer feature gaps remain.

### Adapter, scalar, bounds and storage failure witnesses, 26 November 2026

Baseline: `d37124484`. Owners: root modeling acceptance consumers,
P3/P6/P8/P9/P19/P20/P25/P30/P37. Domain: malformed native meshes, unresolved
scalar identities, four adapter scalar families, finite rectangle operations,
sampled revolution, scalar wire admission, bounds, empty TPMS inputs and authored
resource errors. Read all five files in full, including feature-gated test bodies.
Supporting reads of the shared uncertainty fixture, owning adapters/errors and
bounds consumers do not add new whole-file kernel credit.

Missing witnesses were undocumented private helpers/tests, an operation enum and
its variants, and statements stronger than the assertions. The shared unresolved
zero is a sine-addition identity; the supplied Pythagorean expression is its
separate proved control. Adapter fixtures wrap native geometry without testing
coordinate export through each scalar type. Authored incompatible-policy and
storage errors check transport, not actual retries or resource exhaustion.
Documented exact area/volume, finite point membership, edge pairing and constructor
guards at their actual strength. Recovery with a different sampled field is not
a larger-budget retry. No executable line, fixture or assertion changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline in all five files. All 46 declarations have
  Rustdoc; the four operation variants were checked separately.
- `cargo check --locked -p csgrs --no-default-features --features curve,sdf,svg-io`
  with the five explicit `--test` targets, `--offline -j 2`, passed without warnings.
  This compiles all affected conditional test bodies, including the field-error
  arm shared by SDF and metaballs. No runtime assertion result is claimed.
- `cargo rustdoc` for each selected test target with the same features,
  `--document-private-items` and denied broken intra-doc links passed after
  explicitly supplying the checked `csgrs` metadata and cached dependency search
  directories. The initial ordinary Cargo invocation could not link its own
  library; adding only `--extern` still lacked transitive dependency paths.
  The final invocation resolves both without changing repository configuration.
- Inspected all 22 top-level generated pages and verified the complete text of
  all 17 rendered declaration summaries plus four enum variants. Rustdoc omits
  `#[test]` functions; those remain source-reviewed, not claimed as rendered API.
- Geometry execution and full workflows were not run under the docs-only waiver.
  Builds reused `/tmp/csgrs-rustdoc-target` with incremental compilation disabled;
  its total size remained approximately 1.3 GiB.

Logs: `/tmp/csgrs-rustdoc-failure-tests-check.log`,
`/tmp/csgrs-rustdoc-failure-tests-doc.log` (initial linkage failure), and
`/tmp/csgrs-rustdoc-failure-tests-private.log` (successful private docs).

### Adversarial modeling and fixture consumers, 26 November 2026

Baseline: `130422d9f`. Owner: root modeling regression consumers,
P1/P3/P8/P9/P17–P20/P24/P25/P35/P37. Domain: finite binary64 fixture ingress,
native boxes, short Boolean/transform chains, sampled profiles, extrusion,
segmented revolution, sweep, loft and lexical fixture records. Read all five
test files in full. Supporting reads of native constructors, containment,
face/edge predicates and manifest gates do not add whole-file owner credit.

Missing witnesses: all 21 declarations lacked Rustdoc, and several names imply
more than their assertions establish. The near-origin point `(1e-12, 0, 0)` is
on an edge of `[0,1]^3`; its exclusion from strict interior alone cannot detect
vertex merging. The separate translation witness compares the exact bound.
`is_exact_solid` checks triangle validity/uniqueness and geometric edge pairing,
not vertex links or embedding. Circle fixtures are sampled polygons. Nonempty
output, positive extent and fixed iteration counts do not prove arbitrary set
equality or a resource bound. These limits are now explicit without changing
any fixture, assertion, predicate order or feature gate.

The metadata test requires a nonempty directory and literal labels in immediate
`.txt` entries. It neither requires a text entry nor parses/replays a described
operation. Read all four current records completely; preserved their historical
text rather than treating legacy `f64`/polygon/contained-panic descriptions as
current native API acceptance rules:

| File | Whole-file review | Evidence limit |
| --- | --- | --- |
| `tests/fixtures/adversarial/difference_metadata_sources.txt` | Source-label preservation narrative for attributed difference; unchanged. | No replay or current metadata correctness claim. |
| `tests/fixtures/adversarial/zero_by_zero_image_import.txt` | Empty raster narrative with permissive historical expected behavior; unchanged. | Current admission forbids replacing ordinary failure with a contained panic; this text is not that contract. |
| `tests/fixtures/adversarial/nonfinite_curve_shape_parameters.txt` | Legacy invalid-parameter/NaN filtering narrative; unchanged. | Does not establish current constructor behavior or authorize filtering authoritative geometry. |
| `tests/fixtures/adversarial/svg_path_order_roundtrip.txt` | Path-order-independent polygon/triangle roundtrip narrative; unchanged. | No semantic replay or triangle-count verification performed in this batch. |

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Comment/blank-line removal gives
  identical executable source to baseline in all five files; all 21 declarations
  are documented.
- Focused `cargo check` for all five test targets passed with defaults disabled
  and `curve,sdf,svg-io`; the `adversarial` target also passed with
  `curve,offset,stl-io` to compile its optional offset block. Extra features reuse
  existing metadata. Both checks used `--locked --offline -j 2` and produced no warnings.
- Private Rustdoc for all five test targets passed with broken links denied,
  using the checked own-library metadata and dependency paths as in the previous
  batch. Inspected all 14 top-level pages, all nine rendered declaration descriptions
  and the module descriptions. Test-function Rustdoc remains source-reviewed.
- Compiled `tests/adversarial_fixtures.rs` directly with `rustc --edition 2024 --test`
  and repository `CARGO_MANIFEST_DIR`, then ran its single existing test: passed.
  This establishes label presence only, not fixture semantics.
- Geometry execution and full workflows remain waived. The same temporary target
  was reused with incremental compilation disabled; no model or fixture output was written.

Logs: `/tmp/csgrs-rustdoc-adversarial-check.log`,
`/tmp/csgrs-rustdoc-adversarial-offset-check.log`,
`/tmp/csgrs-rustdoc-adversarial-private.log`, and
`/tmp/csgrs-rustdoc-adversarial-fixtures-run.log`.

### Exact downstream witnesses and shared helpers, 26 November 2026

Baseline: `9ab43450b`. Owners: root exact modeling acceptance consumers and their
shared test helper layer, P1/P3/P4/P8/P9/P17–P20/P24/P25/P35/P37. Domain: rational
line regions with holes, exact shifts beyond binary64 range, overlapping box
shells, sampled-sphere hulls, same-axis rotation identities, scalar comparisons,
finite enclosures, construction flags and indexed triangle topology. Read all
five changed files in full. Supporting reads of scalar sign certification,
filled-area semantics, mesh validation, sphere/hull/merge consumers and downstream
call sites do not add whole-file kernel or downstream-test credit.

Missing acceptance witnesses were undocumented declarations and overly broad
helper descriptions. Documented the rational profile's exact `23/3` area and 92
signed six-volume separately from flags and native edge validation. The 26-box
workload distinguishes concatenated shells from regularized material; its volume
and three point probes are not a general surface-equivalence oracle. Hull tests
separately check deterministic storage/evidence, closed-PWN validity and retained
source vertices, including a temporary signature in a fresh child process.
Compound rotations retain an independent six-support-plane/outward-orientation
and volume oracle. No intermediate query was inserted before the original union.

Shared helper corrections preserve mathematical distinctions: face uniqueness
and geometric edge pairing do not check vertex links or embedding; construction
flags alone do not certify source fidelity; a strict `1e-30` enclosure is not
equality; index Euler counts do not merge equal-coordinate rows. The oriented
triangle comparator maps every right position to the first matching left row,
including unused right rows, but does not canonicalize duplicate left rows.
Thus it can reject geometrically equivalent input and is not a general set test.
Documented its exact structural/strict-Z/X/Y comparison order and the validation
helper's eager pair of queries; all executable ordering remains unchanged.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Comment/blank-line removal gives
  identical executable source to baseline for all five files. All 46 declarations,
  including the function-local disjoint-set root, now have Rustdoc.
- Focused `cargo check --locked -p csgrs --no-default-features --features
  curve,sdf,svg-io` passed for the four edited root test targets and
  `downstream_exact_inputs`, `downstream_profiles`, `downstream_solids`, and
  `enclosure_workflows`. These include all five consumers of the shared helper.
  A separate `curve,obj-io` check of `enclosure_workflows` also passed to include
  its conditional OBJ consumer. Checks used `--offline -j 2`, without warnings.
- Private Rustdoc for the four edited root targets passed with broken links
  denied, checked own-library metadata and cached dependency search paths.
  Inspected 39 pages across those targets and the shared module, verifying all
  34 rendered declaration descriptions and five module descriptions. `#[test]`
  functions and the nested root helper remain source-reviewed rather than
  credited as standalone rendered API pages.
- No geometry, hull child process or full workflow execution was performed under
  the owner's docs-only waiver. Existing assertions/fixtures remain unchanged.
  Builds reused the single temporary target with incremental compilation disabled.

Logs: `/tmp/csgrs-rustdoc-exact-consumers-check.log`,
`/tmp/csgrs-rustdoc-exact-consumers-obj-check.log`, and
`/tmp/csgrs-rustdoc-exact-consumers-private.log`.

### Containment, winding and inherited query receipts, 26 November 2026

Baseline: `2791f35ee`. Owners: root containment/Boolean/curve-query acceptance
consumers, P3/P4/P6/P8/P9/P17–P19/P22/P24/P25/P37. Domain: exact integer/rational
and symbolically translated polyhedra, malformed topology, nonzero winding,
cancelled sheets, qualified seams/contacts, sampled cylinder subtraction and
qualified empty/nonempty curve queries. Read all five files in full, including
all helper bodies and local functions. Supporting reads of native containment,
modeling query wrappers, the uncertainty fixture, properties and admission rules
do not add new whole-file owning-layer credit.

Missing witnesses: undocumented private items, fields and tests, with names that
could overstate the scope of finite probes or native validators. The independent
box oracle uses half-grid queries and eight ±1/8 octant samples, with integer
support planes; documented its small-i32 domain assumptions. Source-face/edge/
vertex membership is distinct from boundary membership of the regularized set.
The side-two box helper checks support planes and signed volume but does not
assert each face normal. One cancelled-sheet continuation checks the receipt
without independently asserting its final geometry empty. Four spawned workers
do not force a particular simultaneous interleaving.

Documented validation before bounds shortcuts, strict/approximate/strict fact
reuse, and mutated-source rejection. Boolean projected volumes use explicit
binary64 tolerances, independently of local certainty and native edge checks.
The qualified cap has positive source area despite approximate collapse; its
stronger height-sign query remains after construction. Qualified query failures
and exact-square controls retain their existing order and assertions. No fixture,
policy, predicate, implementation or test assertion changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  yields identical executable source to baseline for all five files. All 65
  declarations, including two local functions, have Rustdoc; all three descriptor
  fields were separately reviewed.
- Focused `cargo check --locked -p csgrs --no-default-features --features
  curve,sdf,svg-io` passed for the three containment test targets, then the two
  receipt/query targets after their edits. Used `--offline -j 2`; no warnings.
  Extra features reuse existing metadata, not an expanded behavioral claim.
- Private Rustdoc passed for all five targets, with broken links denied and the
  checked own-library metadata plus dependency paths supplied explicitly. Inspected
  29 top-level pages, all 24 rendered declaration descriptions, five module
  descriptions and three private fields. `#[test]` and function-local declarations
  remain source-reviewed rather than credited as standalone rendered API pages.
- Geometry, worker execution and full workflows were not run under the docs-only
  waiver. Builds reused the existing temporary target with incremental compilation
  disabled; no new fixture or model outputs were created.

Logs: `/tmp/csgrs-rustdoc-containment-check.log`,
`/tmp/csgrs-rustdoc-containment-private.log`,
`/tmp/csgrs-rustdoc-query-receipts-check.log`, and
`/tmp/csgrs-rustdoc-query-receipts-private.log`.

### Empty composition and construction qualification, 26 November 2026

Baseline: `16fab2681`. Owners: Hypermesh's construction-qualification bitset and
root modeling acceptance consumers, P3/P4/P6/P8/P9/P17–P19/P22/P25/P30/P37.
Domain: monotone qualification flags, available/missing source history, buffer
identity changes, exact/qualified Boolean outputs including canonical empty,
curve conversion, explicit retained-geometry adoption and chained outcomes.
Read the complete owning 101-line module and all three test files, including
curve-gated nested tests. Supporting reads of mesh construction/equality,
modeling empty bridges, outcome combinators and the previously reviewed STL
pipeline add no whole-file credit elsewhere.

Missing witnesses were undocumented test helpers/cases and private bitset items.
Documented every flag and the private tuple field: missing history suppresses
aggregate predicate-certainty claims without clearing known approximation;
projection/sampling are separate qualifications. Mutation adds flags but does not
itself detect buffer changes. The default claim is untracked. These are bounded
qualifications rather than source relations or replayable proofs.

The empty tests distinguish both-empty-buffer success from malformed carriers,
preserve original error indices, regularize shell multiplicity and consume results
again. Volume/support-plane/edge checks, finite probes and adapter error strings
are documented at their actual strength. Receipt tests distinguish certified
local work from approximate ancestry, qualified curve topology and missing
curve-to-mesh history. Rebuilt controls explicitly adopt coordinates/boundaries;
matching their geometry does not alone prove cache admissibility or source
fidelity. Documented the difference between receipt-combining `and_then` and `map`
whose value may itself contain an error or another qualified outcome.

Also read this local rule file in full:

| File | Whole-file review | Application |
| --- | --- | --- |
| `crates/hypermesh/AGENTS.md` | Contains the standard full pre-push verification list; unchanged. | The owner's explicit session-wide docs-only waiver applies here as at the root. Focused checks below replace full workflows for this session only. |

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. All three test files have identical
  non-comment/nonblank lines to baseline. In the owning module, removing comments
  and formatting leaves identical tokens except the optional trailing comma added
  when documenting the tuple field. Its `u8` type, constants, bit operations,
  dispatch and decision order remain unchanged.
- All 59 declarations across these files have Rustdoc; the tuple field was checked
  separately. Earlier focused checks of the two composition targets passed; after
  the owning-module edit, all three targets were compiled again with defaults
  disabled, both without features and with `curve,stl-io`, including the pipeline.
  `cargo check --locked -p csgrs ... --offline -j 2` passed without warnings.
- `cargo doc --locked -p hypermesh --no-default-features --document-private-items
  --no-deps --offline -j 2` passed with broken links denied. Inspected the owning
  type page's full descriptions, all private flags, tuple field, mutation helper,
  default implementation and aggregate-certainty precedence.
- Private Rustdoc for all three root test targets passed in minimal and curve/STL
  configurations, supplying checked own-library metadata and cached dependency
  paths. Inspected 17 direct/helper pages, all 13 rendered declaration descriptions
  and linked module descriptions. Test functions remain source-reviewed rather
  than claimed as rendered standalone API pages.
- Geometry, concurrency, STL execution and full workflows were not run under the
  docs-only waiver. No model files were created. The reused temporary target remains
  approximately 1.3 GiB with incremental compilation disabled.

Final-edit logs: `/tmp/csgrs-rustdoc-evidence-downstream-check.log`,
`/tmp/csgrs-rustdoc-evidence-minimal-check.log`,
`/tmp/csgrs-rustdoc-construction-owner-doc.log`, and
`/tmp/csgrs-rustdoc-evidence-private.log`.

### Native graphics boundaries and planar matrix admission, 26 November 2026

Baseline: `03218cd2e`. Owners: root native/scalar/attribute boundary and planar
modeling acceptance consumers, P1/P3/P4/P6/P8/P9/P17–P19/P24/P25/P29/P30/P37.
Domain: self-crossing faces, homogeneous division, degenerate/unresolved normals,
four scalar adapters, exact/authored graphics rows, stronger scalar proofs,
shared/mutated caches and homogeneous maps restricted to the XY plane. Read both
files in full, including attributed feature cases and all compatibility trait
implementations. Supporting reads of graphics caching, attributed export and
planar matrix admission do not add whole-file owner credit.

Missing witnesses were 44 undocumented declarations/members and the absence of
scope descriptions for several tests. Documented exact versus integer-projected
normal values, preserved source chains, repeatable success caches and buffer
mutation isolation. Authored nonunit normals can export degenerate geometry;
that does not certify source validity. The stronger-proof witness deliberately
allows the first export to succeed and does not claim a reproduced cold failure.
The attributed worker test uses a barrier and checks shared successful addresses.

The planar compatibility trait preserves legacy successful empty/None values;
it converts only existing errors. Homogeneous gauges are normalized on XY even
when the full 4D matrix is singular. Perspective/out-of-plane, zero-weight
nonzero-image, all-zero-image and undecided constraints retain distinct categories.
Documented which cases assert only error presence, area/bounds or receipt values,
and which compare an empty symmetric difference. Input restoration is distinct
from a larger-budget retry. No executable line, fixture, policy, assertion or
predicate order changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline in both files. All 44 declarations/members
  have Rustdoc, including the trait method and its three implementations.
- `cargo check --locked -p csgrs --no-default-features --features curve,attributed
  --test native_boundary_failures --test planar_operation_failures --offline -j 2`
  passed without warnings, including every feature-gated case in these files.
- Private Rustdoc for both targets passed with broken links denied, using checked
  own-library metadata and dependency paths. Inspected all 11 top-level pages,
  both module descriptions and all 13 rendered declaration/member descriptions.
  Test-function docs remain source-reviewed rather than credited as rendered pages.
- Geometry, concurrency, cache eviction/proof recovery and full workflow execution
  were not run under the docs-only waiver. The existing temporary target was reused
  with incremental compilation disabled, and no model or graphics artifacts were emitted.

Logs: `/tmp/csgrs-rustdoc-native-planar-check.log` and
`/tmp/csgrs-rustdoc-native-planar-private.log`.

### Implicit-field acceptance and retained workloads, 26 November 2026

Baseline: `fb48e8eac`. Owners: root planar/3D metaball and sampled-field
acceptance consumers, P1/P3/P4/P6/P8/P9/P17–P20/P25/P30/P37. Domain: positive
inverse-square sources and normalized sampling, invalid/undecided parameters,
finite grids, scalar expression domains/branches, diagnostics, sampled topology
and continued modeling. Read all five files completely, including private trait
methods and implementations. Supporting reads of field sampling/borders, planar
wrappers, renderer/benchmark fixture excerpts, properties and admission rules do
not add new whole-file owner or benchmark credit.

Missing witnesses were undocumented declarations and missing limits on assertion
scope. Exact diamond area and Surface Nets coordinates describe the adopted
sampled models, not their continuous level sets. The 3D 3-cubed core uses a
one-cell exterior border and therefore 5-cubed diagnostics. Raw inverse-square
influence remains singular at a center, while the normalized sampling field has
its declared continuous extension. A ring's selected memberships do not alone
count holes. Published workload sources/resolutions match the inspected renderer
and feature-benchmark excerpts; their bounds assertions use the field's own
conservative bounds, not an independent analytic oracle.

Documented strict field evaluation separately from policy-selected sample signs
and extraction. Approximate meshing does not authorize an undecided expression
branch. Indexed scalar failure retains `[2,0,0]` and `SqrtNegative`; later recovery
changes field inputs rather than increasing a budget. Huge-grid tests exercise
capacity preflight/category mapping, not measured allocator or total-work
exhaustion. Diagnostic comparisons check stored mesh equality and local receipts,
not every evidence field. No code, policy, source count, resolution or assertion
was changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline in all five files. All 71 declarations and
  trait members have Rustdoc.
- `cargo check --locked -p csgrs --no-default-features --features curve,metaballs,sdf`
  with all five explicit test targets and `--offline -j 2` passed without warnings.
  Every file's feature gate was enabled.
- Private Rustdoc for all five targets passed with broken links denied, supplying
  the checked own-library metadata and dependency search paths. Inspected all 22
  top-level pages, five module descriptions and 29 rendered declaration/member
  descriptions. Test-function docs remain source-reviewed rather than credited
  as standalone rendered pages.
- Geometry, sampling workloads, benchmarks, rendering and full workflows were not
  executed under the docs-only waiver. The existing temporary target was reused
  with incremental compilation disabled; no model or image outputs were created.

Logs: `/tmp/csgrs-rustdoc-implicit-tests-check.log` and
`/tmp/csgrs-rustdoc-implicit-tests-private.log`.

### STL imported-model and sampled-sphere witnesses, 26 November 2026

Baseline: `b08c2a34e`. Owners: root interchange and sampled-geometry acceptance
consumers, P1/P3/P4/P6/P8/P9/P17–P19/P22/P25/P26/P29/P30/P37. Domain: independently
encoded ASCII/binary STL across finite binary32 scales and malformed topology,
sampled rational-angle spheres, strict Boolean/hull composition and finite
expectation probes. Read both files in full, including the ignored hull test and
all timing/query bodies. Supporting reads of STL parsing/validation and sphere
construction do not add whole-file owner credit.

Missing witnesses were undocumented private declarations/fields and descriptions
stronger than the implemented checks. STL rows compare promoted scalar values;
mixed signed zero does not assert bitwise identity. Independent encoders bypass
the production exporter, and winding integrals retain multiplicity rather than
proving arbitrary regularized-set volume. Parsed geometry validation is distinct
from fidelity to an earlier exact source. The cube helper's support-plane/volume
and native edge checks do not independently inspect every outward face normal.

Corrected the sphere module's overbroad oracle narrative. Its `Ball` helper uses
binary64 distances, an assumed `0.8 * radius` inner ball and `1e-9` margins. The
implementation contains neither a certified inradius check nor a floating-error
enclosure for those comparisons. Finite probe agreement therefore must not be
reported as an exact universal set oracle. This records an evidence limitation;
it does not modify the fixture or claim an observed wrong answer. Cube helper
`None` also covers points on support planes outside finite faces. Hull face-side
cross-checks use unnormalized projected normals. Timing messages and historical
ignore-reason costs remain diagnostics, not fresh performance evidence.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline. All 37 declarations/members are documented,
  with the two private ball fields checked separately. No threshold, input, test
  assertion, ignore annotation or executable ordering changed.
- `cargo check --locked -p csgrs --no-default-features --features curve,stl-io
  --test stl_import_contracts --test sampled_sphere_strict --offline -j 2` passed
  without warnings. The extra curve feature reuses existing checked metadata.
- Private Rustdoc for both targets passed with broken links denied, using checked
  own-library metadata and dependency paths. Inspected all 24 top-level pages,
  both module descriptions, 23 rendered declaration/member descriptions and two
  private fields. Test functions remain source-reviewed rather than credited as
  standalone rendered API pages.
- STL parsing/geometry execution, the ignored hull workload, timing measurements
  and full workflows were not run under the docs-only waiver. The temporary target
  was reused with incremental compilation disabled; no STL/model files were emitted.

Logs: `/tmp/csgrs-rustdoc-stl-sphere-check.log` and
`/tmp/csgrs-rustdoc-stl-sphere-private.log`.

### Strict Boolean and polygonal-drill workflow witnesses, 26 November 2026

Baseline: `cd84a54ce`. Owners: root modeling acceptance harness and its independent
rational oracle, P1/P3/P4/P8/P17–P21/P23/P37. Domain: nine rational box relations
under five invertible affine maps, plus inscribed 8/16/24/32-sided through-drills
and subsequent empty-set composition. Read all five Rust files completely,
including nested macro bodies, included helpers and all controls. Also read all
107 lines of `docs/workflow-corpus.md`; cross-checked its transformed-box and drill
entries against these implementations. The remaining catalog entries retain
their separate source-review obligations.

Missing witnesses were undocumented private declarations/fields, generated test
items and the distinction between assertion scopes. Documented the box oracle's
source-plane and bound restrictions, exact inverse charts, reflection orientation,
cell truth tables, rational ray retry argument and separately required closed PWN
validation. The 21-output chains compare sets independently of triangulation and
include reuse after complete removal. Equal-volume wrong-set and forced-retry
controls remain unchanged. Counts do not establish total resource bounds.

Corrected a drill comment: the certified width `2/10^24` bounds the error in signed
six-volume, not directly the volume. The formula describes the inscribed polygonal
hole. Finite membership probes and a volume enclosure do not by themselves prove
general set equality or analytic-cylinder fidelity. This is a documentation
correction, not a newly observed geometry failure or a changed acceptance gate.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline for all five Rust files, including macro
  bodies. No test, predicate order, geometry or assertion changed.
- `cargo check --locked -p csgrs --no-default-features --features curve,stl-io
  --test strict_boolean_workflows --test strict_drilled_workflows --offline -j 2`
  passed without warnings; features reuse the existing checked dependency set.
- Private Rustdoc for both targets passed with broken links denied, using checked
  own-library metadata and dependency search paths. Inspected 48 HTML pages
  (including indexes and macro redirects); matched 45 rendered declaration,
  method and field descriptions. Generated frame modules/macros render privately;
  seven test definitions, including the two macro templates, are source-reviewed
  rather than credited as rendered test-function pages. An initial text-match
  check needed whitespace normalization across inline-code markup, then passed.
- Geometry workflows, runtime controls, benchmarks and full CI were not run under
  the docs-only waiver. The existing temporary target was reused with incremental
  compilation disabled; no models or images were emitted.

Logs: `/tmp/csgrs-rustdoc-strict-workflows-check.log` and
`/tmp/csgrs-rustdoc-strict-workflows-private.log`.

### Downstream exact inputs and profile witnesses, 26 November 2026

Baseline: `add25e883`. Owners: root downstream acceptance consumers and independent
line/circle/section oracles, P1/P3/P4/P6/P8/P17–P19/P24/P25/P34/P37. Domain: exact
fraction/decimal boxes, small nonzero integer segments, positive-radius circle
supports, an algebraically trimmed parabola, rounded polygon offsets/collapse,
and a twelve-slice anisotropically scaled holed extrusion. Read both files
completely and checked the shared helpers' relevant bodies and native circle
relation/arc construction entries. Native owner excerpts and external historical
CAD snapshots receive no whole-file or fresh execution credit.

Missing witnesses were undocumented private declarations, enum variants/payloads
and scope distinctions. The integer line oracle requires intermediate i64
arithmetic to fit; overlap ranges are compared independently with endpoint replay
on the first segment, while point contacts replay both parameters. Circle helpers
construct semicircles but query full supports. Wrapper agreement is dispatch
evidence, not an independent oracle. Exact decimal parsing remains distinct from
binary64 import, and the test does not execute the original external CAD parser.

Profile documentation separates exact area/coordinate assertions, certified
`10^-30` enclosures, retained sampling flags and projected smoke bounds. Existing
ordering limitations remain expected diagnostic-text assertions. Section-side
prismatoid and cap integrals are signed sums; matching them does not identify
individual triangles. Corrected an overbroad comment that promoted empty-union
triangle preservation into an independent embeddedness proof. No geometry failure
is claimed; face/edge, index topology, row preservation and embedding are distinct
evidence. All fixtures, blockers, thresholds and assertions remain intact.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Source comparison after removing
  blank/comment-only lines is identical for the profile file. Exact-input changes
  additionally lay out two tuple variants to document all four payloads; comparison
  verifies their identical tokens except the two formatting trailing commas.
- All 39 declarations, seven enum variants and four tuple payloads have Rustdoc.
  Private docs matched 36 rendered descriptions across 27 top-level HTML pages
  and both support module pages; 14 test bodies remain source-reviewed because
  Rustdoc omits test-function pages.
- `cargo check --locked -p csgrs --no-default-features --features curve,stl-io
  --test downstream_exact_inputs --test downstream_profiles --offline -j 2`
  passed without warnings. Private Rustdoc for both targets also passed with
  broken links denied and checked own-library/dependency metadata supplied.
- Runtime geometry, external CAD programs, benchmarks and full workflows were not
  run under the docs-only waiver. Reused target remains approximately 1.3 GiB with
  incremental compilation disabled; no geometry/renderer artifacts were generated.

Logs: `/tmp/csgrs-rustdoc-downstream-inputs-profiles-check.log` and
`/tmp/csgrs-rustdoc-downstream-inputs-profiles-private.log`.

### Downstream rotated-solid and planter witnesses, 26 November 2026

Baseline: `d3daed290`. Owner: root downstream solid acceptance consumer,
P1/P3/P4/P8/P17–P19/P24/P25/P37. Domain: exact equivalent angle spellings,
near-coincident rotations of cubes and four/eight-sided prisms, and the full
algebraic/polyhedral planter with polygonal rim, cavity, four 48-sided drains,
below-floor chamfer and separate hexagonal tray. Read all 687 baseline lines,
including both ignored workloads. Historical kernel notes and current adapter
contracts were consulted as excerpts, not credited as whole-file reviews.

Missing witnesses were undocumented declarations and overbroad statements about
what finite checks establish. Documented ordered-coordinate equality separately
from triangle/provenance equality, the overlap formula's angle domain, convex
operand assumptions, and the ignored matrix's actual eighteen operations.
Successful fixtures remain required successes; documentation does not claim
fresh execution or predicate-stage trace evidence.

Boundary probes use the centroid plus/minus `2^-60` times an unnormalized normal.
The implementation has no minimum-separation proof; finite exact point decisions
neither validate every point on a face nor find a wholly missing component.
Removed the stronger clearance/universal-face claim while preserving both probes
and every assertion. Empty input passes that helper vacuously, whereas its caller
separately requires nonempty output. This records evidence scope, not an observed
wrong geometry result.

Planter documentation preserves inward authored winding and distinguishes exact
tray measures from certified `10^-30` signed-six-volume enclosures for the body,
drains and scene. The chamfer is a retained source child whose placement implies
no removal. Full-source construction, selected memberships, exit segments, bounds,
index components and native face/edge checks remain separate witnesses. Historical
ignore-reason timings remain untouched and are not current performance evidence.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline, including ignored annotations and all
  arithmetic, predicate order, geometry inputs and assertions.
- All 24 declarations have Rustdoc. Private docs matched 17 rendered summaries
  across 18 top-level HTML pages and the support module page. Seven test bodies
  were source-reviewed; Rustdoc does not render them as test-function pages.
- `cargo check --locked -p csgrs --no-default-features --features curve,stl-io
  --test downstream_solids --offline -j 2` passed without warnings. Private
  Rustdoc passed with broken links denied and checked dependency metadata supplied.
- No runtime geometry, ignored workload, external CAD program, benchmark or full
  CI execution under the docs-only waiver. The existing temporary target was
  reused with incremental compilation disabled; no scene exports were generated.

Logs: `/tmp/csgrs-rustdoc-downstream-solids-check.log` and
`/tmp/csgrs-rustdoc-downstream-solids-private.log`.

### Constant-section enclosure witnesses, 26 November 2026

Baseline: `559e5fb5b`. Owner: root enclosure acceptance consumer,
P1/P3/P4/P8/P17–P19/P25/P26/P29/P37. Domain: rational rectangular plans,
constant-height walls/base, repeated window/pocket/restoration cuts, dyadic
thickness down to `2^-80`, empty/split inward offsets, algebraic bevel corners,
rounded plans sampled by extrusion, and integer-coordinate OBJ continuation.
Read the full 423-line baseline file. Shared helper contracts and the historical
performance paragraph were inspected in context without new whole-file credit.

Missing witnesses were undocumented private declarations and imprecise certainty
and measurement descriptions. The `certified` helper receives a mesh after a
strict convenience operation, then checks certified native face/edge validity;
it cannot inspect the already-discarded operation receipt. Extrusion separately
asserts its local receipt. Empty production XORs compare modeling paths through
the kernel and are distinct from independent hand-derived measures. No claim of
vertex-link or general embedding validation is made.

Corrected the module narrative: round-plan area checks are certified `10^-30`
enclosures, while projected solid-volume error below 0.05 is a smoke bound.
Sampling/projection flags remain separate. Repeated-generation triangle counts
are deterministic representation checks, not memory or total-work bounds. The
historical sampled-cap union/subtraction timings are explicitly historical;
this test neither times nor replays that alternative. Integer OBJ round trips
exercise the reimported coordinate model without asserting original provenance.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline: inputs, feature gates, callback/predicate
  order, thresholds, assertions and convenience paths were preserved.
- All 23 declarations have Rustdoc. Private docs matched 16 rendered summaries
  across 17 top-level pages and the support module page. Seven test definitions,
  including the OBJ-gated body, remain source-reviewed rather than credited as
  rendered test-function pages.
- `cargo check --locked -p csgrs --no-default-features --features curve,obj-io
  --test enclosure_workflows --offline -j 2` passed without warnings. Private
  Rustdoc passed with broken links denied and checked dependency metadata supplied.
- Geometry, OBJ round-trip execution, performance and full workflows were not run
  under the docs-only waiver. Reused the existing curve/OBJ target metadata with
  incremental compilation disabled; no model or renderer outputs were created.

Logs: `/tmp/csgrs-rustdoc-enclosure-check.log` and
`/tmp/csgrs-rustdoc-enclosure-private.log`.

### Generated benchmark fixture construction and consumer, 26 November 2026

Baseline: `be588c902`. Owners: deterministic benchmark asset generator and root
import/stability test, P1/P6/P17/P25/P26/P29/P37. Domain: the fixed 31-by-31-by-6
slotted voxel labyrinth and level-three ternary foam after explicit diagonal-edge
contact filling. Read both files completely, including the local face constant
and filesystem installation path. Kernel-comparison import/call-site excerpts
were checked but receive no whole-file benchmark review credit.

Missing witnesses were undocumented private declarations and absent domain/file
lifecycle limits. Foam contact filling adds cells and changes the raw fractal set;
it is not a topology-preserving identity or universal vertex-link repair. Boundary
extraction emits fixed outward diagonals with shared corner keys, assuming corner
arithmetic and u32 coordinate conversion fit the small fixture domain. Generic
large inputs have no checked work/memory budget. Native edge checks, fixed counts,
byte determinism and mathematical set fidelity remain separate evidence.

Documented runtime target-directory resolution, byte-equal reuse, process-ID
staging followed by rename, retained outputs and failure behavior. There is no
explicit synchronization, same-process concurrent-writer isolation, durability
sync or cancellation cleanup; rename failure may leave the temporary file. These
are implementation descriptions, not newly executed failure or race findings.
The test repeats generation in one process and checks bytes/imported validity;
it does not establish fresh-process replay or independent set equality.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline for both files; cells, order, counts, paths,
  casts, file handling and assertions remain unchanged.
- All 13 declarations have Rustdoc. Private docs matched 11 rendered descriptions
  across 13 HTML pages. The test body and function-local `FACES` constant were
  source-reviewed; neither receives standalone rendered API-page credit.
- `cargo check --locked -p csgrs --no-default-features --features curve,obj-io
  --test generated_benchmark_corpus --offline -j 2` passed. Private Rustdoc also
  passed with broken links denied and checked own-library/dependency metadata.
- Downstream `cargo check --locked -p csgrs --no-default-features --features
  attributed,curve,obj-io,stl-io --bench kernel_comparison --offline -j 2` passed
  without warnings. This compilation does not credit that whole benchmark as read.
- No fixture generation, filesystem installation, geometry runtime or benchmark
  execution under the docs-only waiver. Incremental compilation remained disabled
  and the shared target remained approximately 1.3 GiB; no OBJ assets were emitted.

Logs: `/tmp/csgrs-rustdoc-generated-corpus-check.log`,
`/tmp/csgrs-rustdoc-generated-corpus-private.log` and
`/tmp/csgrs-rustdoc-generated-corpus-bench.log`.

### Competitive adapters, acquisition and timing consumers, 26 November 2026

Baseline: `ce429f009`. Owners: benchmark comparison adapters, root acceptance
consumer, optional fixture acquisition and timing registration, P1/P6/P8/P19/P25/
P26/P29/P35/P37. Domain: four binary64 box/tetrahedron cases, 3,072-face subdivided
boxes, optional control-hull/full-control workloads, projected output and tolerant
summaries. Read all four files completely (149/702/210/362 baseline lines). Kernel
comparison call sites and shared harness contracts were consulted; this does not
add whole-file kernel-comparison review credit.

Missing witnesses were undocumented declarations/records and hidden comparison
qualifications. Native import preserves the binary coordinate value, while output
projection, floating subdivision and 10^-9 keys are approximation boundaries.
Quantized closure/half-edge validation concern that adopted welded model. Floating
volume/area/bounds, vertex counts and a shared kernel are not exact universal set
oracles. Summary empty handling and unchecked finite-range assumptions are explicit.
The generic tolerance helper does not independently exclude every nonfinite case.

Documented the CSGRS-only convex fallback: any convex-certification failure can
trigger hull reconstruction and conditional subdivision; no fixed reconstructed
face count is asserted. This is a potential operand-comparability limitation,
not proof of a newly observed wrong benchmark. Boolmesh's exact-string empty-error
mapping is an adapter convention, not independent emptiness evidence. Operation
receipts and native source evidence do not survive the raw projected output.
All these existing implementations are unchanged.

Acquisition documentation separates archive/control hash checks from existence-only
reuse of a derived hull, and states opt-in values, runtime paths, curl/unzip/hash
commands, staging, failures and retained files. No network request or acquisition
command was executed. Benchmark setup remains outside Boolean timing; adapter
output materialization is inside. Count-based checksum fields do not certify
geometry, and filters do not suppress every setup step.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline across all four files, including tolerances,
  operation order, fallback/error branches, external commands and assertions.
- All 77 declarations/members, 23 record fields and three enum variants have
  Rustdoc. Inspected 69 test-configuration HTML pages and matched 95 descriptions;
  four test definitions remain source-reviewed. Separately inspected the benchmark
  root, two modules and two function pages, matching its four declaration summaries
  and module overview. Shared private field/variant descriptions render.
- `cargo check --locked -p csgrs --no-default-features --features curve,obj-io
  --test competitive --offline -j 2` passed. Downstream checks with features
  `attributed,curve,obj-io,stl-io`, `--bench competitive --bench kernel_comparison`
  and the same locked/offline/jobs options passed without warnings.
- Private Rustdoc for the test and benchmark passed with broken links denied.
  The first benchmark invocation lacked Cargo-supplied dev-dependency externs;
  retrying with the already checked boolmesh/manifold-rust/three-d-asset/tri-mesh
  metadata supplied explicitly passed. No manifest or implementation edit was needed.
- Runtime comparisons, benchmarks, acquisition and full workflows were not run
  under the docs-only waiver. Reused the temporary target without incremental
  compilation; no archive, hull, model or renderer outputs were generated.

Logs: `/tmp/csgrs-rustdoc-competitive-check.log`,
`/tmp/csgrs-rustdoc-competitive-private.log`,
`/tmp/csgrs-rustdoc-competitive-benches.log` and
`/tmp/csgrs-rustdoc-competitive-bench-private.log`.

### Successive-operation shared harness and oracle, 26 November 2026

Baseline: `4fbd02563`. Owner: shared root test/benchmark chain harness,
P1/P4/P6/P8/P17–P20/P25/P26/P35/P37/P39. Domain: the finite rational/symbolic drill,
bar, rotation, union, feedback, interchange and profile chains. Read all 783
baseline lines and both consuming test/benchmark files in full for context; their
own declaration documentation remains a subsequent batch and is not yet credited
as completed review in the whole-file table.

Missing witnesses were undocumented private functions, record fields, payloads,
factory preconditions and an incorrect units claim. Corrected `exact_volume`:
it divides the signed determinant sum by six and returns volume, not six-volume.
Its rational recognition applies to the sum; rational input coordinates suffice
but are not individually required by the code. Box-grid volume is independently
computed but one volume scalar does not identify a regularized set.

Documented one-based step ordering, local receipts versus inherited source facts,
returned-error records versus uncaught validation/graph-limit/callback panics,
empty/untouched handling and the observer's persistence responsibility. False
edge-validation values are recorded rather than rejected by the runner. Operation,
validation and replacement/drop timers exclude initial construction, coordinate
measurement/check callbacks, observer work and final/failure teardown. Coordinate
maxima are per component, not total retained graph/heap size or resource bounds.

Factories retain their finite dimensions, unchecked large-count arithmetic and
receipt-extracting convenience paths. Interchange adopts projected binary32/binary64
models and preserves the cut receipt while OBJ import's local receipt is discarded.
Construction errors retain their source category; other I/O errors flatten to a
Geometry message. None of these behaviors, orders, formulas or expected outcomes
was changed by the documentation work.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Source comparison removing
  comments/blanks and normalizing only the documented Outcome tuple layout shows
  identical tokens except its added trailing comma.
- All 45 declarations/members, 23 fields, five variants and two tuple payloads
  have Rustdoc. Matched all 75 descriptions in 40 generated private helper pages.
- `cargo check --locked -p csgrs --no-default-features --features
  attributed,curve,obj-io,stl-io --test successive_operations --offline -j 2`
  passed; the same configuration with `--bench successive_operations` passed.
  Private test Rustdoc passed with broken links denied and checked own-library
  metadata/dependency paths supplied. No warnings were emitted.
- Runtime chains, observer tests, allocation/timing probes and full workflows
  were not executed under the docs-only waiver. Existing target artifacts were
  reused with incremental compilation disabled; no raw journals/models were emitted.

Logs: `/tmp/csgrs-rustdoc-successive-support-check.log`,
`/tmp/csgrs-rustdoc-successive-support-private.log` and
`/tmp/csgrs-rustdoc-successive-support-bench.log`.

### Successive-operation tests and allocation/journal consumer, 26 November 2026

Baseline: `748d61a1d`. Owners: root chain acceptance tests and standalone timing/
allocation consumer, P4/P6/P8/P17–P20/P25/P26/P35/P37/P39. Domain: the retained
finite and ignored long chain matrices, validation-cache comparisons, synchronous
observer ordering, process-global requested-byte counters and optional raw TSV
journaling. Completed the declaration audit of both files read in the preceding
batch (599 and 292 baseline lines); all formulas, ignored annotations and size
rows remain intact. The external runner script and measurement document remain
separate review work, not newly credited whole-file sources.

Test docs distinguish independent volume from set equality, native edge checks
from embedding, exact output-coordinate comparisons from trace/receipt equality,
and finite graph/count rows from universal resource/completeness claims. Rational
triangle canonicalization uses BTreeSet and therefore discards multiplicity.
Corrected the ambiguous radical-tower comment: cubic sin(pi/18) is not expressible
solely through iterated square roots; no current proof-stage trace is asserted.

Allocator docs state that counters track requested sizes before System calls,
without failure rollback, overhead/RSS coverage or full-lifetime accounting.
Global relaxed snapshots can include other threads. Allocation deltas exclude
record-vector growth after the snapshot, while the runner's step timer includes
wrapper overhead. Initial/final lifetimes, validation and journal I/O are distinct.
Raw header/records sync before continuing, but `# completed` means normal harness
termination even after a chain returned a typed failure. None of these existing
behaviors was changed or executed as a new performance measurement.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source in both consumers and the shared helper's one link edit.
  All 28 test declarations and 21 benchmark declarations/methods plus four fields
  have Rustdoc; unsafe allocator contracts are documented in their implementations.
- Private test docs matched eight rendered summaries in nine top-level pages plus
  the chains module; twenty test definitions remain source-reviewed. Benchmark
  docs matched all 25 declaration/member/field summaries in seventeen explicit
  current pages plus the chains module. Stale pages from the identically named
  test target were not credited to the benchmark.
- `cargo check --locked -p csgrs --no-default-features --features
  attributed,curve,obj-io,stl-io --test successive_operations --offline -j 2`
  and the same configuration with `--bench successive_operations` passed without
  warnings. Both private Rustdoc targets passed with broken links denied and
  checked own-library/dependency metadata supplied.
- The first private build exposed a scope-sensitive shared module link after the
  consumer gained outer module docs. Fully qualifying it as
  `csgrs::GeometryContext::STRICT` fixed the link; final builds passed.
- No chain execution, allocator/timing measurement, filesystem journal or full CI
  was run under the docs-only waiver. Reused target remained approximately 1.3 GiB.

After adding this batch, every tracked Rust file under the root `tests/` inventory
has a whole-file ledger row. This is a bounded review milestone, not automated
all-feature documentation coverage or repository-wide completion; owning crates,
remaining benchmarks, nested packages and non-Rust sources remain substantial work.

Logs: `/tmp/csgrs-rustdoc-successive-consumers-check.log`,
`/tmp/csgrs-rustdoc-successive-consumers-private.log`,
`/tmp/csgrs-rustdoc-successive-consumers-bench.log` and
`/tmp/csgrs-rustdoc-successive-consumers-bench-private.log`.

### Probe driver, integration-test documentation and measurement narrative, 26 November 2026

Baseline: `32426592a`. Owners: research evidence tooling and its documentation,
P6/P20/P35–P37/P39. Read all three files completely and reconciled current harness
semantics with the scripts and historical record. This non-Rust batch uses Python
docstrings/comments and Markdown rather than claiming Rustdoc coverage for Python.

| File | Review and change | Validation |
| --- | --- | --- |
| `scripts/successive-operations.py` | Entire 300-line baseline; documented all functions including the nested signal handler, snapshots, attribution, journal parsing and exit/cleanup limits. | AST-identical excluding docstrings; help and Pydoc inspected; build/run not executed. |
| `scripts/test-successive-evidence.py` | Entire file; documented all three functions, retained setup, real geometry/termination behavior and row-parser limits. | AST-identical excluding docstrings; help and Pydoc inspected; integration test not executed. |
| `docs/successive-operations.md` | Entire 325-line baseline; corrected current validation/allocator terminology and separated historical finite measurements from fresh or universal claims. | Local link target files exist; historical measurement table and command blocks preserved; no performance replay. |

Missing witnesses were undocumented script helpers and prose stronger than the
current evidence. Snapshot hashes identify selected files/link text and metadata,
not a hermetic or signed build. The production journal summary accepts complete
18-column lines without validating each numeric field/outcome label; the integration
test's simpler parser can include a partial line. Run completion and certified
geometry completion remain separate, as do driver and child exit codes. Generic
errors may leave incomplete retained directories; cleanup is not universal.

The measurement narrative now names local receipts, native edge pairing, signed
volume checks, per-coordinate metrics and requested-byte counters separately.
Historical rows, commands and numerical expectations were preserved; no source
claim was replaced by a lower-resolution or failing acceptance fixture. Unarchived
raw samples were not recreated, and historical speed/heap conclusions were not
revalidated by this documentation audit. The standing release target is unchanged.

Validation after final edits:

- Parsed both scripts with Python's AST and compared them with baseline after
  removing only module/function docstrings: identical. All twelve driver functions
  and three integration-test functions, including nested functions, have docstrings.
- Imported modules under non-main names and inspected plaintext Pydoc for every
  top-level function. Nested functions were source-reviewed. Both `--help` commands
  succeeded without invoking build/run or the integration workload.
- `git diff --check` passed. All nine local Markdown link target files exist;
  fragment anchors were not revalidated. Historical measurement table rows are
  byte-identical, and command blocks were left unchanged.
- No geometry, Cargo build, process-termination test, acquisition or evidence-run
  directory was generated. Focused checks follow the session's docs-only waiver.

Temporary documentation outputs: `/tmp/csgrs-rustdoc-successive-operations-pydoc.txt`,
`/tmp/csgrs-rustdoc-test-successive-evidence-pydoc.txt`,
`/tmp/csgrs-rustdoc-successive-driver-help.txt` and
`/tmp/csgrs-rustdoc-successive-evidence-help.txt`.

### Mesh operation, policy-outcome and graphics timing boundaries, 26 November 2026

Baseline: `6f616289f`. Owners: three standalone root benchmark consumers,
P8/P19/P25/P35–P37/P39. Domain: sampled spheres/cylinders, translated boxes,
native hulls/Minkowski sums and f32/f64 graphics exports. Read all three files
completely (95/79/120 baseline lines); existing adapter and shared harness contracts
provide context but no new whole-file owner credit.

Missing witnesses were undocumented declarations and misleading workload labels.
The mesh_buffers row only reads lengths and sums nominal coordinate/index counts;
it does not convert, export or materialize buffers. Aggregate operation loops do
include output destruction, while graphics export and Boolean-context timers stop
before output destruction. Projected-buffer equality tests compare the two export
paths, not analytic-source fidelity; warmups/order and count fingerprints are explicit.

The retained Boolean-context assertions require all STRICT attempts to be undecided
and all APPROXIMATE_512 attempts to consume the terminal. They compare different
completion outcomes, not the same completed contract, and would reject newly
certified completion. This audit records the code's expectations without changing
them or claiming a reproduced current runtime failure. New per-policy mesh buffers
do not guarantee globally cold scalar caches and do not transport old construction
evidence. Timings include undecided attempts and report the last successful face
count. No expectation, input, policy or measurement loop was changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only lines
  gives identical source to baseline in all three files.
- All eleven declarations have Rustdoc. Private builds passed with broken links
  denied; inspected seventeen HTML pages and matched all declaration summaries
  plus the three module descriptions.
- `cargo check --locked -p csgrs --no-default-features --bench adapter_graphics_export
  --bench mesh_boolean_context --bench mesh_pipeline --offline -j 2` passed without
  warnings. Private Rustdoc used the matching checked minimal-library metadata
  and dependency search paths.
- No geometry benchmark, heap trace, timing measurement or full workflow was run
  under the docs-only waiver. Existing target artifacts were reused with incremental
  compilation disabled; no geometry/renderer exports were created.

Logs: `/tmp/csgrs-rustdoc-small-benches-check.log` and
`/tmp/csgrs-rustdoc-small-benches-private.log`.

### Profile constructors and broad feature timing catalog, 26 November 2026

Baseline: `a661af52a`. Owners: two root benchmark consumers, P8/P9/P19/P25–P27/
P36/P37/P39. Read all 307/846 baseline lines and checked shared-harness and relevant
constructor/Boolean implementation context. Domain: the fixed profile, mesh, scalar,
implicit, text, adapter, blueprint and interchange fixtures in these executables.
Missing witnesses were undocumented helpers and labels that suggested more than the
current closures perform. This batch changes comments only.

The profile benchmark's final union is a closed cubic plus a disjoint square,
despite the historical line/arc label. Compound constructors discard local receipts;
strict extrusion also returns only the value to measurement. Native source semantics,
sampled modeling and finite projection remain distinct. Output counts do not certify
geometry or source fidelity.

The broad catalog performs separate immediate Booleans in a fixed order; it does
not submit a shared Boolean program. Buffer cases that only read lengths are now
identified, as are grid-only distribution cases, imports from prebuilt exports,
fresh vector writer timing, nominal work units and lossy scalar-bit fingerprints
that substitute zero on conversion failure. No runtime branch or failure convention
was repaired or reclassified. Setup outside closures still occurs when its timed
case is filtered out. Cache reuse, output destruction and discarded receipts are
explicit rather than inferred from the benchmark labels.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Nonblank, non-comment source is
  byte-identical to baseline in both files.
- `cargo check --locked -p csgrs --no-default-features --features
  bench-feature-pipeline --bench profile_primitives --bench feature_pipeline
  --offline -j 2` passed without warnings, covering the declared feature bundle.
- Private Rustdoc passed with broken links denied and matching library metadata.
  Inspected sixteen rendered pages: all fourteen declaration summaries plus both
  module introductions. Every declaration in these two files is documented.
- No geometry cases, measurements, acquisition or full workflows were executed
  under the owner's docs-only waiver. The existing temporary target was reused
  with incremental compilation disabled; no repository-local build tree was added.

Logs: `/tmp/csgrs-rustdoc-feature-profiles-check.log` and
`/tmp/csgrs-rustdoc-feature-profiles-private.log`.

### Optional Solidean request interpreter and benchmark, 26 November 2026

Baseline: `15b18c971`. Owners: benchmark acquisition/interpreter and its timing
consumer, P6/P19/P25–P27/P36/P37/P39. Read all 733/138 original lines. Domain:
the pinned version-one, single-run JSON workloads, OBJ payloads and five admitted
dome levels. Missing witnesses were undocumented private/public items and implicit
assumptions about parsing, cache integrity, graph execution and geometric validation.

The interpreter converts OBJ decimal text to binary64 before exact native ingress,
fans polygon faces without convexity/planarity checks, and validates final index
bounds. Zero OBJ indices are not explicitly rejected: they map to the current
position count and can survive if later vertices exist. Backward instruction
references are checked while loading; aliasing and repeated consumption panic only
at execution. These are documented implementation limits, not repaired behavior or
proof of general OBJ/request support. No predicate calls or operation order changed.

Acquisition hashes compressed archive bytes and selected control files. Repository
reuse does not hash every OBJ, and derived dome reuse checks only three files for
presence. The lexical path guard does not confine symlinks. External-command/I/O
failures can leave staging; no lock, cancellation cleanup or durability contract
is claimed. This audit invoked none of the acquisition or mutation paths.

Execution timing includes native conversion, intermediate destruction and optional
diagnostics. Validation/checksums/final-output destruction are outside that duration.
Projected area and absolute signed volume are checked for finiteness and against
retained tolerant expectations; these are not independent topology or set-equality
oracles. The checker-grid `(0, 0)` reference can admit tolerance-sized nonempty
geometry and does not assert canonical emptiness. Workloads and thresholds remain
unchanged and were not executed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing comments/blank lines
  leaves identical benchmark code. Support code is identical except layout and
  trailing commas needed to document individual tuple fields; scoped enum token
  comparison verified unchanged variants, field types and order.
- Minimal-feature `cargo check --locked -p csgrs --no-default-features --bench
  solidean_comparison --offline -j 2` passed without warnings.
- Private Rustdoc passed with broken links denied. Its first invocation lacked
  the dev-dependency `serde_json`; the successful rerun supplied the exact metadata
  identified by the checked benchmark's dependency fingerprint, alongside the
  matching csgrs library. No manifest changes were needed.
- Checked all seventy declaration/field/variant summary groups in rendered docs,
  including private instruction payloads and request state, plus both module
  introductions. Source review covered every declaration and execution branch.
- No downloads, extraction, geometry measurements or full workflows ran under
  the docs-only waiver. Existing temporary build output was reused.

Logs: `/tmp/csgrs-rustdoc-solidean-check.log` and
`/tmp/csgrs-rustdoc-solidean-private.log`.

### Cross-kernel Rust workload consumer, 26 November 2026

Baseline: `020873fa8`. Owner: root kernel-comparison benchmark, P8/P19/P25/P36/
P37/P39. Read all 1018 baseline lines plus relevant native edge-pairing/finite
projection and shared-harness implementations. Domain: fixed sampled primitives,
exact rational sliver, generated OBJ corpus, optional control/hull and transformed
operands. Missing witnesses were undocumented helpers/local declarations and
claims stronger than the measured operations or fingerprints.

Coordinate fingerprints first allocate a projected position vector, then hash
rounded triangle corners. Failed aggregate conversion triggers per-coordinate
fallback with zero for failed/nonfinite values; large casts saturate. These are
diagnostic hashes, not equality proofs or certified sub-grid errors. The triangulate
case clones an already triangular source. Other count-only rows, timed normal search,
unconditional generated-corpus setup and dispatch-feature prewarming are explicit.
The optional control's hull proxy and original geometry remain distinct workloads.

The winding helper pairs indexed edges, propagates face flip parity and chooses
positive signed six-volume for each component. It is not a general cavity-preserving
normalizer, vertex-link validator or embeddedness proof. It preserves shared native
storage/evidence on an unchanged clone, but rebuilding after flips does not transport
old construction evidence. All eight function-local item/field/method doc groups
were source-reviewed; Rustdoc does not publish their standalone pages.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing comments/blank lines
  yields identical source, including predicate order and all workload assertions.
- The manifest-declared required set `attributed,obj-io,stl-io`, with defaults
  disabled, fails compilation: E0432 for `csgrs::curve` and E0425 for `solid::torus`.
  This unchanged source/configuration gap was reproduced and is now explained in
  module Rustdoc. The docs-only batch does not change Cargo feature requirements.
- Adding `curve` makes `cargo check --locked -p csgrs --no-default-features
  --features attributed,curve,obj-io,stl-io --bench kernel_comparison --offline -j 2`
  pass without warnings. Private Rustdoc passes with matching library metadata and
  broken links denied. All nineteen top-level summaries and the module introduction
  were checked in rendered pages; local declarations were checked in source.
- No geometry benchmark, generated fixture, optional acquisition, dispatch count or
  cross-engine timing was run. The owner waived full checks for this docs-only
  session; the pre-existing minimal-feature failure remains recorded, not fixed or
  counted as a pass. Existing temporary artifacts were reused (~1.5 GiB at review).

Logs: `/tmp/csgrs-rustdoc-kernel-declared-check.log`,
`/tmp/csgrs-rustdoc-kernel-check.log` and
`/tmp/csgrs-rustdoc-kernel-private.log`.

### Standalone size consumers and reporting tools, 26 November 2026

Baseline: `82d6f218c`. Owners: standalone benchmark package, C consumer and size
reporting tool, P8/P9/P19/P25–P30/P36/P37. Read the three Rust binaries (56/76/57
baseline lines), all 425 script lines, all 94 C lines, the complete 100-line README,
61-line manifest and 358-line lockfile. Missing witnesses were undocumented entry
points and prose overstating aggregate certainty, executed coverage, cleanup or
section metrics. The manifest and generated lockfile were reviewed but not edited.

The composed consumer returns the extrusion's local receipt; convenience translation,
difference and bounds do not contribute separate receipts to that value. Raster
exactness concerns thresholded unit cells. Rust timers include construction and
destruction after one warmup, but are absent on WASM. The C consumer reads projected
f64 bounds/buffers and queries evidence without asserting its fields; only success
paths free every acquired handle. Failure exits are not explicit handle cleanup.

The script builds/reports Rust and WASM artifacts without running them. ABI/perf
scopes execute native consumers, while perf includes primitive/composed only.
Documented-feature WASM has one profile row, unlike the binding-only profile sweep.
Native section grouping is a nonzero-address/non-NOBITS heuristic, and WASM counts
payloads without headers/framing. Optional-tool omissions, retained optimized/copied
artifacts, lack of an error cleanup trap and absence of cross-configuration output
comparison are now explicit. No measurement results or historical performance claims
were created by this review.

Non-Rust whole-file review records:

| File | Review and validation |
| --- | --- |
| `benchmarks/size-harness/measure.sh` | All seven shell functions, inline parsers and dispatch read; comments corrected; Bash syntax and all three embedded Python programs parse. No build/measurement commands executed through this script. |
| `benchmarks/size-harness/consumer/abi_consumer.c` | Both functions and all status/ownership paths read; comments corrected; strict C11 warnings-as-errors syntax check passed against the real ABI header. No link/run. |
| `benchmarks/size-harness/README.md` | Entire guide reconciled with executable scopes, timers, receipts, metrics and optional tools; command blocks preserved. |
| `benchmarks/size-harness/Cargo.toml` | Complete manifest reviewed unchanged; isolated workspace/feature/binary mapping checked; all three profile tables equal root tables. |
| `benchmarks/size-harness/Cargo.lock` | Complete generated file read unchanged; TOML parsed and all 38 shared package identities/checksums match root. This is dependency consistency, not a source/dependency security audit. |

Validation after final edits:

- Rustfmt and `git diff --check` passed. All five changed executable sources are
  identical after stripping comments/blank lines; Rust/C/shell operations are intact.
- `cargo check --locked --manifest-path benchmarks/size-harness/Cargo.toml
  --all-features --bins --offline -j 2` passed without warnings. This checks the
  standalone package with both optional features enabled, not each minimal feature
  combination, release linking, or WASM execution.
- All three private binary Rustdoc builds passed with broken links denied. Checked
  six private function summaries and three module introductions in rendered HTML.
- `cc -std=c11 -Wall -Wextra -Werror -fsyntax-only -Icrates/csgrs-ffi/include
  benchmarks/size-harness/consumer/abi_consumer.c`, `bash -n` on the measurement
  script and parse-only compilation of its three embedded Python blocks passed.
- No workflows, consumers, downloads, ABI links, size/latency measurements or WASM
  optimizer runs were executed under the docs-only waiver. Temporary compilation
  reused the shared target with incremental compilation disabled.

Logs: `/tmp/csgrs-rustdoc-size-consumers-check.log` and the three
`/tmp/csgrs-rustdoc-size-{primitive,composed,raster}-private.log` files.

### Benchmark orchestration and CSV summarization, 26 November 2026

Baseline: `5b9212fb5`. Owners: benchmark execution/evidence tools, P6/P20/P36/
P37/P39. Read all 140 runner lines and 265 summarizer lines. Both receive only
comments/docstrings; this non-Rust batch does not claim Rustdoc coverage for Python
or shell. Missing witnesses were undocumented parsing/statistical functions and
implicit claims about complete parity, coldness, result validation and cleanup.

| File | Whole-file review and validation |
| --- | --- |
| `benchmarks/run.sh` | Documented execution/partial-output lifecycle, quick/cold settings, relative-output path caveat, optional gate spellings and omitted feature requirement. Bash syntax passes; non-comment source identical. |
| `benchmarks/summarize.py` | Documented module, all five functions and four header/label constants; parsed AST identical excluding docstrings, CLI help and Pydoc inspected. No sample files or reports were generated through its execution path. |

The loader accepts negative numeric counters except iterations, duplicate sample
indices and extra row columns; short rows can cause an uncaught TypeError. Summary
times/output divide by iterations, not work units; raw checksums are not normalized.
Parity requires engine sets only for observed keys. It cannot see a workload missing
from every engine or prove a full workload/temperature Cartesian product. Markdown
ratios can fail on zero medians; output-write errors lie outside the main handler.
No parser, validation gate or reporting calculation was changed.

Fresh command processes do not make every case globally cold: setup/earlier cases
can warm proofs, and cold mode does not override the stress-family sample settings.
The runner retains partial files after failure and can resolve relative output paths
against different directories before/after its `cd`. Its cold/warm Cargo calls omit
`attributed`. Reproduced target selection with `cargo check --locked --bench
kernel_comparison --offline -j 2`: Cargo rejects the target because required features
are not enabled. The ordinary branch explicitly adds attributed; the cold/warm branch
remains an unchanged configuration gap. No benchmark execution was attempted.

Validation after final edits: Bash syntax, Python parsing, CLI help, AST equality,
all five rendered Pydoc function descriptions and `git diff --check` passed. The
recorded required-feature error is not a successful build or a repaired workflow.
No CMake/native builds, fixture acquisition or performance measurements ran under
the owner's docs-only waiver. Outputs: `/tmp/csgrs-rustdoc-summary-help.txt`,
`/tmp/csgrs-rustdoc-summary-pydoc.txt` and
`/tmp/csgrs-rustdoc-runner-features-check.log`.

The complete benchmark README was read, but its cross-native claims still require
the native consumer review; reading alone is not recorded as completed qualification.

### Shared native benchmark harness and CMake setup, 26 November 2026

Baseline: `5604ae64b`. Owners: native benchmark support/build configuration,
P6/P19/P25/P36/P37. Read every line of `benchmarks/native/common.hpp` (376 baseline
lines) and `benchmarks/native/CMakeLists.txt` (94). Both are whole-file non-Rust
reviews. Added documentation to every header type, field, method, free helper and
local incidence alias, and explained build/fixture assumptions in CMake comments.

The OBJ reader rejects zero/forward indices, parses binary64 coordinates, fans
faces and normalizes component winding. It does not establish source fidelity,
polygon convexity/planarity, vertex-link topology or embeddedness. Positive shell
orientation can change cavity semantics. Its signed six-volume sign uses ordinary
double comparison and does not reject zero/NaN, unlike the strict Rust counterpart.

The native harness's unsigned stream parser accepts prefixes; filters do not trim
and an empty filter selects all. Counters, iteration products and sample offsets
wrap rather than saturate. These differences from the Rust harness remain explicit;
matching CSV column names do not imply identical selection/resource semantics.
Output destruction and counter accumulation are timed; printing/sink updates are
outside timing. No callback correctness or receipt validation is added by the harness.

CMake installs fixture paths as strings, without acquisition or hash verification.
Missing kernels may be skipped only when REQUIRE_ALL is off, but missing STL support
for a found OCCT remains fatal. Compiler-specific flags and NDEBUG behavior are
documented. No option, flag, path or build condition changed.

Validation after final edits: nonblank/non-comment source comparison is identical
for both files; `git diff --check` passed. A C++20 syntax-only translation unit
including the real header and instantiating `Harness::run` compiled without warnings.
All declarations were source-reviewed. Doxygen is unavailable, so no rendered C++
documentation is claimed. No CMake configure, native kernel build, fixture load or
benchmark execution ran under the docs-only waiver; this is not a geometry or
cross-engine performance qualification.

### CGAL native comparison implementation, 26 November 2026

Baseline: `a8c09557b`. Owner: CGAL benchmark consumer, P19/P25/P36/P37/P39.
Read all 1044 original lines of `benchmarks/native/cgal.cpp`; this is a whole-file
non-Rust review. Added 35 documentation groups covering file purpose, aliases, all
functions and named local helpers/types, plus call-site timing/representation notes.
Fixed fixtures and all library calls, success gates and assertions remain unchanged.

EPECK operates on admitted coordinates, while round primitive samples, OBJ ingress,
icosahedron normalization, projected fingerprints and XYZ rotation have explicit
binary64 boundaries. Rotation projects existing points before double arithmetic;
it is not merely exact arithmetic with pre-rounded trigonometric coefficients.
The geometry fingerprint hashes projected bits without the Rust runner's extra grid
quantization. Soup orientation return values are ignored; this adapter does not
independently certify topology or source identity after conversion.

Recorded measurement limits: graphics rows count corners without output buffers;
mass-properties measures volume alone despite a nominal output count of ten;
identical-box intersection only measures existing geometry; disjoint union copies
face graphs. Ray/segment rows construct trees and count intersected primitives,
not deduplicated intersection points. Normal recalculation and approximate-angle
selection have their actual work described. The subdivision helper assumes triangle
fixtures, skips shorter faces and uses only the first three vertices of larger ones.
These are implementation observations, not repaired behavior or evidence of matched
cross-engine completed contracts. Exact sliver volume remains a specific assertion.

Validation after final edits: all nonblank/non-comment source is identical to the
baseline; `git diff --check` passed. Declaration/call-site documentation was reviewed
against the complete source. Direct C++20 syntax checking stops at missing
`CGAL/Exact_predicates_exact_constructions_kernel.h` on this compiler's include path
(`/tmp/csgrs-rustdoc-cgal-syntax.log`). This is a blocked compilation check, not a
passing native build. Doxygen is unavailable, so no rendered C++ coverage is claimed.
No libraries were installed, fixtures loaded, CMake build performed or geometry
benchmark executed under the docs-only waiver.

### OpenCascade native comparison implementation, 26 November 2026

Baseline: `63e31c759`. Owner: OCCT benchmark consumer, P19/P25/P36/P37/P39.
Read all 957 original lines of `benchmarks/native/opencascade.cpp`, a whole-file
non-Rust review. Added 32 documentation groups for the file, alias, policies/fields,
all helpers and named corpus callback. No executable source or validation gate changed.

The runner mixes analytic B-reps, polygon prisms, unsewn polyhedral face compounds
and tolerance-sewn OBJ solids. Zero added Boolean fuzz does not remove stored shape
tolerances. Boolean constructors precede explicit configuration/build, and meshing
requests parallel execution even though explicit Boolean Build requests serial work.
Geometry fingerprints use bounds bits/orientation flags rather than every coordinate.
No exact source-fidelity or inherited certainty receipt is produced by these paths.

Documented case limits include finer remeshing under the subdivide label, remesh/count
graphics cases without exported buffers, and an identical-intersection shortcut with
no Boolean call. Connectivity does not weld nodes across faces. Materialized normals
apply locations but not face orientation flags. Ray and polyline cases both use the
same finite line interval; containment maps every non-OUT classifier state to true.
The sliver's tolerant mass comparisons do not explicitly reject NaN. Fixed-file STL
writing includes filesystem work without concurrency isolation or error cleanup.
These are source observations, not new geometry tests or benchmark measurements.

Validation after final edits: `git diff --check` and full-file comparison after
removing comments/blank lines passed. Every declaration and case was source-reviewed.
Direct C++20 syntax checking, including the conventional `/usr/include/opencascade`
path, is blocked by missing `BRepAlgoAPI_Common.hxx`; see
`/tmp/csgrs-rustdoc-occt-syntax.log`. No successful native compile is claimed, and
Doxygen remains unavailable. No dependencies were installed, no fixtures acquired,
and no CMake build, meshing, Boolean or export ran under the docs-only waiver.

### Benchmark guides and ignore rules, 26 November 2026

Baseline: `0d37f0f28`. Owner: benchmark documentation, P19/P25/P36/P37/P39.
The missing acceptance witness was agreement between the guides' claims and the
already reviewed executable runners. Read every line of these seven files:

| File | Baseline lines | Whole-file result |
| --- | ---: | --- |
| `benchmarks/README.md` | 292 | Corrected measurement, filtering, feature-selection, validation and cross-engine scope claims. |
| `benchmarks/WORKLOADS.md` | 60 | Documented actual per-engine operands and implemented work; retained the matched-contract requirement. |
| `benchmarks/CROSS_KERNEL_COVERAGE.md` | 56 | Separated required portable coverage from observed-label parity and scenario inventory. |
| `benchmarks/SOLIDEAN.md` | 110 | Qualified local cache, timing, certainty and mass checks; preserved published numerical tables. |
| `benchmarks/mesh-boolean-context-2026-09-05.md` | 41 | Identified historical evidence and distinct strict-failure/approximate-completion contracts. |
| `benchmarks/results/.gitignore` | 2 | Unchanged: ignores result files while retaining this rule file. |
| `benchmarks/size-harness/.gitignore` | 1 | Unchanged: ignores this standalone workspace's target directory. |

Source inspection of `src/solid.rs` and the reviewed native builders establishes
that csgrs boxes start at the origin while native boxes are centered. The shared
sphere/box translation therefore gives csgrs box bounds `[3,17] × [2,16] × [1,15]`
versus native bounds `[-4,10] × [-5,9] × [-6,8]`. The contained-box difference removes
a corner in csgrs and an enclosed cavity in the native runners. Sphere construction,
graphics output, mass queries, subdivision, ray intervals and identity shortcuts
also differ. This is source evidence of unmatched scenarios, not a new geometry
execution. Neither labels, counts nor fingerprints establish equivalent completed
contracts. The target comparison requirement remains; no fixture or gate changed.

Checked all four linked Solidean publisher articles against the retained numerical
reference tables. Read the pinned upstream README license notice and parsed its
primitive, terrain and cube-grid request JSON: respectively nine differences,
224 differences, and 999 unions followed by 1000 differences. Recorded the terrain
article/request discrepancy and the grid publisher's per-step validation versus
the local final tolerant mass check. Links identify the primary sources. No mesh
assets or archives were acquired, and none of these requests was executed.

Validation after final edits: `git diff --check` passed; all fenced command blocks
are byte-identical, all five guides' relative Markdown link targets exist, and the
Solidean/context historical table rows are byte-identical. Only Markdown changed,
so no new Cargo compilation or runtime check was needed under the docs-only waiver.
Historical pass statements were not rerun and are not current validation claims.
All 36 tracked files below `benchmarks/` now have explicit whole-file review records
across these batches; this scoped milestone does not establish repository-wide
coverage, native compilation, geometric correctness or performance equivalence.

### Native C ABI implementation, 26 November 2026

Baseline: `98855ed0f`. Owner: C ABI above the scalar adapters, P6/P8/P9/P19/P25/P28–P30/P37.
Read all 2742 original lines of `crates/csgrs-ffi/src/lib.rs`, including all macro
expansions' source definitions/invocations and sixteen test/helper functions.
The missing acceptance witness was per-item public/private Rustdoc agreeing with
the pointer, scalar, geometry and failure behavior. Added docs for records and
fields, enum variants/payloads, associated types/methods, private aliases/helpers,
thread-local diagnostics, macros and every generated/direct C entry point.

The domain is live, correctly owned ABI handles and valid C memory, with family
checks and adapter/native validation of geometric values. Non-null pointer validity
is a caller obligation, not established by the null checks or panic catcher.
Documented borrowed input versus owned output, row-major matrices, split-word
integers, buffer count units, two-level profile offsets, empty-query ownership,
diagnostic lifetime and destructor behavior. Catching unwinds is not an abort or
memory-safety recovery mechanism, and error formatting occurs outside the catcher.

Documented finite binary-value ingress, lossy float egress, restricted integer
recognition and native scalar cloning. Region profiles first sample/project through
binary64 even in the real family; they cannot replay the exact source curve.
The ABI has no policy selector or separate local receipt. Region Booleans use the
active Hypercurve scope (normally strict); extrusion/revolution/profile extraction
discard separate outcomes, while mesh construction history remains queryable.
Sampling history is not analytic fidelity or a topology certificate.

Source tracing also corrected `src/adapter/mesh.rs`'s cylinder summary: height is
signed, with endpoints at Z zero and the imported height, not necessarily along +Z.
The existing `solid::cylinder` → `frustum` → `frustum_between` path establishes this
domain without an implementation change. This was a narrow follow-up to the earlier
whole-file adapter review, not a new whole-file kernel review.

Known implementation limits remain unchanged: generated buffer/profile exports
construct raw owning descriptors on the assignment's right-hand side before the
output-pointer check on the left. A null output can therefore leave internal raw
allocations unreclaimed even though no output record is published. This source-level
ownership observation is not an allocation measurement or a repaired failure path.
No ABI-wide work/memory/cancellation budget is implemented by these wrappers.

Validation after final edits:

- `cargo check --locked -p csgrs-ffi --all-targets --offline -j2` passed.
- `cargo rustdoc --locked -p csgrs-ffi --lib --offline -j2 -- --document-private-items -D rustdoc::broken_intra_doc_links -D missing_docs` passed.
- `cargo rustdoc --locked -p csgrs --lib --no-default-features --features curve --offline -j2 -- --document-private-items -D rustdoc::broken_intra_doc_links` passed for the adapter wording change.
- Focused Rustfmt and `git diff --check` passed. Executable tokens are identical
  after normalizing only whitespace/trailing commas introduced by nine documented
  tuple payloads; the adapter's non-comment lines are identical.
- Inspected 318 source documentation-group summaries across 259 rendered pages,
  including forty generated function templates and all 176 C function pages.
  Every unsafe exported function has a rendered Safety section. Private fields,
  variants, methods and aliases render; the sixteen test/helper functions and
  their cfg-test module were source-reviewed, not claimed as normal Rustdoc pages.
  The corrected adapter cylinder summary also renders.

Logs: `/tmp/csgrs-rustdoc-ffi-{check,private,adapter-private}.log`; shared target
directory `/tmp/csgrs-rustdoc-target` remains about 1.5 GiB with incremental builds
disabled. No tests, geometry operations, shared-library link smoke tests or full
workflow checks were executed under the documentation-only waiver. Successful
compilation/rendering is not a claim of complete ABI safety or runtime conformance.

### C header and FFI package documentation, 26 November 2026

Baseline: `bdadad40d`. Owner: ABI declarations/package guide, P19/P25/P28–P30/P37.
Completed whole-file non-Rust review of the package's remaining seven files:

| File | Original lines | Whole-file result |
| --- | ---: | --- |
| `crates/csgrs-ffi/include/csgrs.h` | 206 | Documented pointer/ownership rules, status and scalar layouts, buffers and all 56 direct/generated declaration templates, reusing the reviewed Rust operation contracts. |
| `crates/csgrs-ffi/README.md` | 153 | Corrected scalar/profile, ownership, receipt and panic scope; described current behavior instead of migration history; identified separate release-consumer checks. |
| `crates/csgrs-ffi/CHANGELOG.md` | 39 | Qualified initial-release scalar projection and unwind-catching claims; no release/version change. |
| `crates/csgrs-ffi/Cargo.toml` | 20 | Unchanged: workspace path dependencies, curve feature, three library artifact kinds and packaged source/header/docs/license include rules reviewed. |
| `crates/csgrs-ffi/.rustfmt.toml` | 6 | Unchanged: local formatting widths and shorthand/match-arm options. |
| `crates/csgrs-ffi/.gitignore` | 3 | Unchanged: local target directory, swap files and macOS metadata excluded. |
| `crates/csgrs-ffi/LICENSE` | 22 | Unchanged: MIT terms, notices, attribution and disclaimer reviewed. |

The header's continued macros retain every executable token. Expanded declarations
before/after are identical under both C and C++ preprocessing after whitespace
normalization. The 176 expanded export names exactly match the reviewed Rustdoc
function pages; name parity is not an independent cross-language layout proof.
The copied per-function comments retain the actual operation domain and caller
obligations, including signed cylinder height, family checks, empty bounds,
sampled revolution/profile semantics and separate receipt loss.

Validation after final edits: C11 and C++17 `-Wall -Wextra -Werror -fsyntax-only`
header checks passed, as did `git diff --check`. README/CHANGELOG fenced commands
are unchanged and relative Markdown targets exist. Inspected the relevant
`scripts/release-check.sh` consumer build/run excerpts to correct the guide's
validation location; this is not whole-file credit for that script. No Doxygen
renderer is installed, so no rendered C-documentation claim is made. No library
link/runtime smoke checks or packaging were run under the docs-only waiver.
Together with the preceding Rust implementation batch, all eight tracked files
in `crates/csgrs-ffi/` now have whole-file review records. Language wrappers remain
separate pending work; this milestone does not establish full repository coverage.

### C++ ownership wrapper, 26 November 2026

Baseline: `6250c231f`. Owner: C++ native binding, P6/P19/P25/P28–P30/P37.
Read all 279 original lines of `bindings/cpp/csgrs.hpp`, including the trait macro's
forty forwarding methods and four invocations, both geometry templates, all
scalar/error methods, private fields/helpers and public aliases. Added source
documentation throughout without changing any declaration or executable token.

Documented Real's cloning copies and transferring moves, move-only geometry,
unchecked raw-handle adoption, borrowed get() pointers and Real::release ownership.
Default/moved-from wrappers are null owners rather than canonical empty models.
The assignment body for Real is noexcept, but cloning its by-value argument can
throw before entry. Geometry operations borrow their inputs, throw typed errors
on failure and adopt new handles on success; raw trait forwarders return statuses.
The exception copies the same-thread diagnostic and retains its numeric category.

Bounds map NO_BOUNDS to std::nullopt, but a present RawReal bounds record still
owns six manually freed scalar handles. Raw mesh/graphics/profile descriptors
likewise have no C++ destructor: the corresponding trait free is required.
Copying an optional or descriptor does not clone ownership. Documented projected
profiles even for RawReal, float/integer export limits, and inherited C operation
domains without adding policy, receipt or resource guarantees.

Validation after final edits: a C++17 translation unit including the actual header
and explicitly instantiating all eight mesh/region family templates passed
`-Wall -Wextra -Werror -fsyntax-only`. Source tokens and complete preprocessed C++
output match the baseline after comment/whitespace normalization. Declaration
inspection found no undocumented method/alias candidates; `git diff --check`
passed. No linked execution, allocation test, Doxygen rendering or full workflow
check ran under the docs-only waiver. Source review and compilation establish the
documentation-only scope, not exception-safe cleanup for manually owned outputs.

### Go binding and regression sources, 26 November 2026

Baseline: `df8194b82`. Owner: Go/cgo binding, P6/P19/P25/P28–P30/P37.
Completed whole-file reviews of all seven tracked files under `bindings/go/`:

| File | Original lines | Review/change |
| --- | ---: | --- |
| `bindings/go/csgrs.go` | 451 | Documented package constraints, 132 function/type declarations, constants, private fields/helpers, family dispatch and all exported ownership/conversion paths. |
| `bindings/go/boolean_failures_test.go` | 82 | Documented inverse-rotation recovery, composed-angle undecided expectations across four Booleans, and empty reuse. |
| `bindings/go/bounds_test.go` | 81 | Documented absence/placement, empty continuation, wrong-family and invalid constructor checks. |
| `bindings/go/curve_constructor_failures_test.go` | 37 | Documented invalid companion dimension/resolution, zero-square absence and recovery. |
| `bindings/go/exact_linear_modeling_test.go` | 33 | Documented 2^53+1 integer ingress, rejected zero extrusion and exact exported bounds. |
| `bindings/go/planar_operation_failures_test.go` | 47 | Documented unsupported/NaN Z controls and empty/nonempty planar continuation, including local case fields. |
| `bindings/go/polyhedral_construction_test.go` | 24 | Documented crossed-face failure and concave-face bounds recovery; no closed-solid proof is asserted. |

Documented explicit Free without finalizers, unsafe ownership copies, null/freed
versus nil Go receiver behavior, unchecked signed-count casts to size_t, and
mutable family tags that do not convert geometry. Unknown dispatch tags route
to the real ABI, where actual native families remain checked. Family-suffixed
methods select that ABI directly rather than converting the receiver. Buffer
exports transfer ownership only without error; real bounds lack a Go free wrapper.
Graphics/profile layout aliases have no corresponding exporter/free wrappers here.

The source fetches the C thread-local diagnostic in a later cgo call without
pinning the goroutine. This does not guarantee diagnostic identity/lifetime across
those calls, even though the numeric status is retained. This is a source-level
finding informed by Go's [LockOSThread contract](https://pkg.go.dev/runtime#LockOSThread),
not a reproduced scheduling failure. Lowercase aliased C fields are not external
Go accessors under the [exported-identifier rule](https://go.dev/ref/spec#Exported_identifiers);
the existing tests are in the same package and do not establish external-package
usability. No implementation fix or synchronization change was made.

Validation after final edits: `git diff --check` and source-identity checks passed.
All non-comment lines match, with only the Real struct's single field expanded to
attach its comment; the complete cgo preamble is byte-identical. Declaration
inspection finds docs on all 132 binding functions/types and six test functions.
No go/gofmt executable was found on PATH or in the checked conventional installation
directories, so Go parsing/type compilation, gofmt and rendered Go documentation
remain unverified. No toolchain was installed and no tests/link smoke checks ran
under the docs-only waiver. Documentation and source equivalence do not establish
the runtime outcomes asserted by the retained regression tests.

### Python ctypes binding, 26 November 2026

Baseline: `c7fb1d814`. Owner: Python native binding, P1/P6/P19/P25/P28–P30/P37.
Read all 655 original lines of `bindings/python/csgrs.py`. Added docstrings for
all 118 authored classes/functions and module documentation, plus `__doc__`
metadata in four dynamic record factories (sixteen concrete ctypes types).
Documented aliases, strategy attributes, native field layouts/count units,
private dispatch/load helpers and ownership fields. Single-line suites were
expanded only to attach their docstrings; no call or conversion changed.

Reproduced existing pure Python conversion behavior without loading a native
library: I128 wraps modulo 2^128 (`2^127` → `-2^127`, `2^128+7` → `7`), integer
family scalar coercion maps `1.9` through `int` to `1`, and negative size_t counts
wrap on this host. Matrix packing bypasses scalar(): integer records or `(hi,lo)`
initializers work while ordinary integers fail; raw scalar pointers/None work
while Real wrappers fail. Null pointer acceptance by ctypes packing does not
establish valid native scalar coefficients. Original Real owners must remain live
while point/matrix records borrow their pointers.

Documented unchecked raw adoption, finalizer-based ownership without explicit
close/context managers, and real-valued bounds returned as raw records without
a cleanup wrapper. Buffer owners free nested real handles, but borrowed array
views cannot outlive them. Source inspection shows the loader caches its first
selection, searches legacy `ffi/target/release` paths rather than the workspace
target directory, and assigns the cache before diagnostic-symbol configuration.
No argument prototypes are installed; polygon/polyhedron lengths remain plain
Python integers rather than explicit size_t arguments. These are existing gaps,
not repaired behavior. The [ctypes calling contract](https://docs.python.org/3/library/ctypes.html#calling-functions)
and [Python finalization contract](https://docs.python.org/3/reference/datamodel.html#object.__del__)
support the documentation's limits on signature checking and cleanup guarantees.

Validation after final edits: Python parsing and in-memory compile passed.
Baseline/current ASTs are identical after removing only docstrings and the four
literal dynamic `__doc__` entries. Baseline/current pure conversion probes agree,
including wrapping, coercion and accepted/rejected matrix initializer forms.
Inspected Pydoc summaries for 135 objects (module, 118 authored definitions and
sixteen dynamic types), including private methods/helpers; output is in
`/tmp/csgrs-rustdoc-python-pydoc.txt`. Import, rendering and probes leave `_lib`
unset and write no bytecode caches. `git diff --check` passed. No native geometry,
library load/link smoke test, destructor allocation test or full workflow check
ran under the docs-only waiver. Helper probes do not establish native binding
safety, geometry completion or reliable finalization.

### Native TypeScript facade and binding guides, 26 November 2026

Baseline: `a680824bc`. Owner: native JS loader facade and binding documentation,
P6/P19/P25/P28–P30/P37. Read all 328 original lines of `bindings/js/csgrs.ts`,
all 134 lines of `bindings/README.md`, and all 96 lines of `bindings/js/WASM.md`.
Added source documentation for all 83 TypeScript declaration/member/field
candidates, including private dispatch, constructors, handle fields and aliases.

The native TypeScript file supplies no loader implementation. Its logical calls
omit C output pointers and array lengths and expect decoded outputs rather than
statuses. Documented required loader responsibilities: scalar/struct marshalling,
Real unwrapping, output slots, typed status handling, bounds absence and output
ownership. In particular, real integer egress expects a decimal string whereas
the C ABI writes two words. Generic/tuple types do not validate runtime families,
counts or pointer provenance. Free leaves handles intact with no disposed guard;
replacing the module-level loader does not migrate outstanding owners.

Reconciled the native guide with the reviewed C++/Python/Go/TypeScript lifetimes,
integer coercions, partial API surface, profile projection and diagnostic limits.
Separated those current implementation limits from the generated WASM API and
removed migration narration from the README. Rechecked the relevant already-reviewed
WASM source excerpts and runner: local certainty differs from inherited mesh history,
plain export records omit history/chord metadata, and each bounds endpoint getter
returns a new owned point wrapper. Updated the guide's ownership and receipt scope
while retaining its status table and code examples.

Validation after final edits: TypeScript non-comment source lines are identical;
all declaration/member/field candidates have adjacent documentation. Both guides'
fenced commands/examples are byte-identical and relative Markdown targets exist.
`git diff --check` passed. No Node/npm/TypeScript executables or local installed
test TypeScript dependency were found, so compilation, rendered TypeScript docs,
generated WASM declarations/runtime and loader integration were not run. No tools
were installed under the docs-only waiver. Source checks do not certify a consumer
loader, runtime outcomes or generated declaration support on this host.

All twelve tracked files under `bindings/` now have whole-file review records.
This is a scoped source/documentation milestone, not repository-wide Rustdoc
coverage or a cross-language conformance claim. The runtime limitations recorded
for individual wrappers remain unresolved implementation work.

### Hyperlimit outcome and sign-resolution contracts, 26 November 2026

Baseline: `0c6cfffd3`. Owner: Hyperlimit's predicate policy, outcome and shared
sign-resolution layer, with Hyperreal retaining scalar arithmetic authority.
Properties: P3–P6/P8–P11/P19/P20/P30. Domain: structural and refined Real signs,
trusted predicate callbacks, Boolean outcome composition and the two existing
policy settings. Missing acceptance witnesses were undocumented private helpers,
policy tuple storage and test functions, plus public text that did not explain
receipt extraction, terminal zero classification or resource limits. Reproduced
these documentation gaps against the current source before editing; no runtime
failure is asserted by this batch.

| File | Whole-file review and documentation change |
| --- | --- |
| `crates/hyperlimit/src/lib.rs` | All 262 baseline lines, including every re-export; corrected the crate-level strict/approximate distinction and documented module roles, feature scope and private-doc invocation. Re-export review does not imply review of the exported implementations. |
| `crates/hyperlimit/src/predicate.rs` | All 481 baseline lines; documented local receipts, trusted public constructors, extraction loss, eager Boolean operands and left-biased sufficient evidence, stage ranking, hint limits, private policy byte and every test. |
| `crates/hyperlimit/src/resolve.rs` | All 584 baseline lines; documented every helper and test, callback contracts/order, rational recognizer declines, composite replay, terminal enclosure rule and structural term-filter storage/decline behavior. |
| `crates/hyperlimit/src/real.rs` | All 118 baseline lines; documented scalar fact precedence, certificate metadata loss, both trait implementation methods and the test. Arithmetic wrappers retain their existing traced boundaries. |
| `crates/hyperlimit/src/error.rs` | All 29 baseline lines; documented static diagnostic payloads, display and standard-error integration, and separation from classifier Unknown outcomes. |
| `crates/hyperlimit/src/trace.rs` | All 13 baseline lines; documented enabled event recording and disabled expansion that does not evaluate arguments. |
| `crates/hyperlimit/src/test_support.rs` | All 28 baseline lines; documented the shared exact positive fixture's cancellation and scalar-domain assumptions; retained the angle-addition terminal control. |
| `crates/hyperlimit/Cargo.toml` | Entire file; reviewed package/docs metadata, dependencies, feature wiring, example and bench declarations. Unchanged. |

Supporting reads of Hyperreal's structural-sign, rational-enclosure and normal-form
methods and Hyperlimit's enclosure classifiers establish the referenced behavior;
these are excerpts, not whole-file kernel review. `classify.rs` was initially
partially displayed and receives no whole-file credit here. The earlier ledger
already covers all eleven files under `tests/wasm/`; confirming those records
adds no new coverage and supersedes the stale suggestion to repeat that batch.

The documentation distinguishes certified Exact/Filtered decisions from an
Approximate terminal. The shared terminal requests a certified rational enclosure
at computable precision -512, classifies a separating enclosure by its sign and
otherwise reports zero, always with the approximate receipt. Zero in that result
is not an equality proof. The exponent is neither a uniform geometric tolerance
nor a total work or memory bound. Exact recognition and field algorithms remain
separate stages after refinement, and an unresolved hint is not an exhaustion
receipt or proof of undecidability.

The general resolver trusts a supplied filter outcome, including Some(Unknown),
and returns it without checking policy or evidence; the exact hook is also
trusted. The direct path preserves scalar proof-route distinctions. Composite
policy resolution may invoke the same callback twice and conservatively marks
any successful approximate replay Approximate/Refined. It does not roll back
side effects or aggregate a work budget. Boolean combinators take already
computed values, retain sufficient left-biased evidence, and do not provide
lazy child evaluation or a complete computation history. `value()` and
`SignKnowledge::sign()` intentionally discard receipt metadata. No decision
sequence, callback invocation, assertion or dispatch count changed.

Validation after the final source edit:

- Focused Rustfmt checks on all seven edited Rust files and `git diff --check`
  passed. Comment-only comparison against the baseline passed: five files have
  identical nonblank/noncomment lines, and the policy tuple plus two error tuple
  payloads differ only by documentation layout and optional trailing commas.
  Whitespace/trailing-comma normalization was restricted to those two files.
- A local declaration scan checked 87 documented named declarations, including
  private/test declarations. Fields and enum payloads were separately reviewed.
  This finite source scan is not a repository-wide Rustdoc percentage.
- With shared `CARGO_TARGET_DIR=/tmp/csgrs-rustdoc-target`, `CARGO_INCREMENTAL=0`
  and offline locked dependencies, `cargo rustdoc -p hyperlimit --lib
  --all-features -- --document-private-items -D rustdoc::broken_intra_doc_links`
  passed. Inspected 105 summary groups across 46 rendered pages and nine
  explicitly selected contract pages, including private resolver functions,
  tuple fields and the Real trait implementation on the re-exported Real page.
- The same private-doc build with `--no-default-features` passed; inspected the
  disabled tracing macro's different rendered contract. Normal library docs
  omit cfg(test) helpers, which remain source-reviewed.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling affected test declarations and integration consumers without
  executing tests. Downstream `cargo rustdoc --locked -p csgrs --lib
  --no-default-features --features curve --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links` passed.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-core-{doc,minimal-doc,check,downstream-doc}.log`.
The shared temporary target is approximately 1.6 GiB. Full workflow/runtime and
performance checks remain waived by the owner for this docs-only session; no
new correctness, timing or universal closure claim is made. The 100% public and
private Rustdoc mandate remains in root AGENTS.md. Most owning-crate sources and
repository support files still require whole-file review.

### Hyperlimit classification vocabulary and facades, 26 November 2026

Baseline: `610f17c2c`. Owner: Hyperlimit's relation labels and variant helpers;
producing geometric classifiers remain in their existing predicate modules.
Properties: P3/P6/P9/P17/P19/P25/P30. Domain: signed line/plane sides, simplex
locations, closed-segment contacts, circle endpoint partitions, closed balls,
intervals, box/DOP candidates and ring turn summaries. Missing documentation
witnesses included private test/conversion methods and several descriptions
that asserted more geometric information than their enum/helper retains.

Whole-file review: `crates/hyperlimit/src/classify.rs` (649 baseline lines),
`crates/hyperlimit/src/orient.rs` (18), and `crates/hyperlimit/src/plane.rs` (21).
The two facades re-export existing carriers, evidence and policy-taking functions;
added module documentation explains that they introduce no second implementation
or implicit policy. Excerpts of `predicates/{distance,segment,triangle,ring,
interval,halfspace}.rs` establish producer behavior and existing fixture
expectations, but those larger files still lack whole-file audit credit.

Current behavior reproduced in source, with the enum helper paths also compiled
and executed in a temporary standalone probe:

- `SegmentIntersection::Identical` includes identical zero-length point-segments,
  and `has_positive_length_overlap()` unconditionally accepts that variant.
  Positive-length use therefore needs separate source degeneracy evidence.
  The existing point-segment fixture and classifier branch retain this behavior;
  no fixture or assertion was changed.
- Segment/triangle and ray/triangle `intersects()` accept Coplanar. Their producers
  can return Coplanar before any planar overlap test, so those true values are
  retained candidates rather than proofs of a common point. For example, the
  triangle with vertices (0,0,0), (1,0,0), (0,1,0) is disjoint from the segment
  (2,2,0)–(3,2,0) and the ray starting at (2,2,0) in direction (1,0,0), despite
  the shared supporting plane. This fixture geometry is an independent source
  argument; the full geometric classifiers were not executed in this batch.
- Circle/segment labels describe an endpoint partition, not an intersection
  count or differential tangency proof. For a unit circle, (1,0)–(2,0) receives
  Secant despite one boundary point, while (1,0)–(0,0) receives Tangent despite
  a radial supporting line. Both expectations already exist in the exact-kernel
  fixture. A point-segment inside the disk receives ContainedInside; a degenerate
  infinite-line input returns DegenerateLine without classifying its point.
- Sphere intersection classifies closed balls by center distance versus the sum
  of radii; it does not classify surface-sphere intersections or internal
  tangency separately. AABB/ball labels use nearest distance: a zero-radius ball
  centered inside a box receives Touching, and zero-volume boxes can receive
  Overlapping. Neither label alone certifies positive intersection volume.
- The ring-turn classifier calls `.value()` on policy-controlled orientations
  and returns a bare RingConvexity. Its approximate-policy result has no child
  certainty receipt; this remains an evidentiary gap, not certified support.
  MixedTurns can short-circuit the scan. Interval endpoint equality gives
  AtLowerEndpoint precedence when both endpoints coincide.

These are documentation corrections and explicit implementation limitations,
not migration of the normative geometry/evidence contract. Missing overlap or
certainty evidence is not supplied by wording, and no completed closure claim
is added. Other enum docs retain their mathematical relation definitions with
an explicit requirement to preserve the producer's certainty and input contract.

Validation after final source edits:

- Focused Rustfmt, `git diff --check` and byte comparison of all nonblank,
  noncomment source lines against baseline passed for all three files.
- Offline locked private Rustdoc for `hyperlimit --lib --all-features`, with
  broken intra-doc links denied, passed. Inspected 180 classification doc groups
  across 32 rendered classification pages, nine selected limitation contracts and
  both facade summaries. Test-only declarations were reviewed in source.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed. This compiles the affected tests and integration consumers without
  running their geometry fixtures.
- Compiled and ran `/tmp/csgrs-rustdoc-classify-probe.rs` with Rust 2024, importing
  the actual current predicate/classification source modules. Its seven assertions
  verify Identical's overlap flag, the two Coplanar/Disjoint helper pairs and two
  sign conversions. It does not construct geometry or call the geometric
  classifiers. No probe was added to the repository.
- Downstream private docs for `csgrs --lib --no-default-features --features curve`
  passed with locked offline dependencies, the shared target and broken intra-doc
  links denied.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-classify-{doc,check,downstream-doc}.log`.
Shared build storage remains about 1.6 GiB; no extra dependency installation or
build directory was introduced. Full workflow, runtime corpus and performance
checks remain waived for this docs-only session. Hyperlimit's batch and geometric
implementation modules, tests and supporting files still need full review.

### Hyperlimit batch scheduling and consumer witnesses, 26 November 2026

Baseline: `811e2d541`. Owners: the Hyperlimit batch wrappers and their integration
consumer. Properties: P4/P8–P10/P19/P20/P30. Domain: the twelve scalar predicate
families represented by ten owned case-tuple aliases, borrowed as slices under
one selected policy, with optional Rayon execution. Reproduced missing tuple
argument-role documentation and overstated batch provenance language against the
current implementation before editing.

Whole-file review: `crates/hyperlimit/src/batch.rs` (397 baseline lines) and
`crates/hyperlimit/tests/batch.rs` (360). Documented all ten tuple roles, twenty-four
sequential/parallel functions, the private parallel module, the test module's
scope, its policy/type aliases, five point/scalar helpers and three tests. No
signature, fixture, predicate order, trace call or assertion changed.

Batches return one unchanged scalar outcome per input position. Unknown does not
stop other queries; empty input collects an empty vector. There is no aggregate
certainty, construction provenance, shared arrangement or cumulative resource
budget. Ray cases pass a direction vector, not a second endpoint; circle cases
pass a squared radius. Parallel results preserve input positions, while worker
execution and cache/trace timing may differ. The implementation does not catch
panics or turn allocation failure into Unknown. Indexed collection ordering was
checked against the locally installed, locked Rayon 1.12.0 collection consumer's
index-based split and contiguous left/right reduction; this dependency excerpt
is not repository file-audit credit or a performance measurement.

The test corpus compares wrappers of the same scalar predicates, including full
outcome metadata, rather than independent geometric expectations. Despite its
historical name, `batch_uses_the_strict_scalar_path_for_each_case` passes
APPROXIMATE_512 and does not inspect dispatch. The 2048-case integer parallel
orientation corpora and smaller other-family fixtures remain unchanged. Their
source does not establish cold-cache equivalence, total resource bounds or
unresolved-symbolic coverage.

Validation after final source edits: focused Rustfmt and `git diff --check`
passed; both files have byte-identical nonblank/noncomment lines relative to
baseline. Private `cargo rustdoc --locked -p hyperlimit --lib --all-features
--offline -j2 -- --document-private-items -D rustdoc::broken_intra_doc_links`
passed. Inspected all 35 complete rendered item-doc groups on their corresponding
alias/function/module pages, including the twelve parallel links. Compiled the
affected integration consumer using `cargo check --locked -p hyperlimit --test
batch --offline -j2` with both `--all-features` and `--no-default-features`; both
passed. Test-only helper documentation was source-reviewed, not rendered by the
normal library build, and runtime fixtures were not executed under the owner's
full-check waiver.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-batch-{doc,check,minimal-check}.log`.
The same temporary target is reused. Read-ahead of geometric fact/homogeneous
modules is not yet a completed audit record; their documentation and validation
remain the next batch.

### Hyperlimit structural facts and homogeneous adapters, 26 November 2026

Baseline: `6a784db3d`. Owners: predicate-facing geometric summaries and homogeneous
incidence, with Hyperlattice retaining point/projective storage and construction
arithmetic. Properties: P3/P4/P6/P8/P9/P15/P19/P25/P30. Domain: planar displacement,
segment, triangle and box zero-support metadata; exact homogeneous point/line
construction and point/plane linear-form incidence, including zero weight and
unvalidated degenerate tuples. Missing acceptance witnesses were undocumented
private helpers/tests and incomplete explanations of trusted snapshots, unknown
flags and construction-versus-validity distinctions. Reproduced against current
source before changes.

| File | Whole-file review and change |
| --- | --- |
| `crates/hyperlimit/src/geometry/mod.rs` | All 15 baseline lines; documented owning-layer separation and all four module roles. |
| `crates/hyperlimit/src/geometry/point.rs` | All 7 lines; existing module docs correctly identify the Hyperlattice aliases and single storage/fact authority. Unchanged. |
| `crates/hyperlimit/src/geometry/facts.rs` | All 759 baseline lines; reviewed every public type/field/method and test. Documented six private helpers, test module, two test helpers and five witnesses; clarified false/unknown meanings, trusted record construction, determinant cancellation and reversed-edge zero evidence. |
| `crates/hyperlimit/src/geometry/homogeneous.rs` | All 228 baseline lines; documented all three construction wrappers, incidence policy/domain limits, private rational helper, both coefficient-trait methods and all test declarations. |

Supporting reads: Hyperlattice `projective.rs` through the construction and
incidence definitions, Hyperreal's sparse rational homogeneous-intersection
adapter, and the beginning of Hyperlimit's plane-carrier module. These are
excerpts and do not establish whole-file review of those implementations.

Structural facts are Copy records with public fields, not automatically
source-bound proofs. Geometry constructors collect exact zero knowledge;
constructors from fact records trust consistency and do not validate source
identity. A false known-zero/known-point flag need not prove the opposite.
One-hot displacement requires a known nonzero component and a known zero other
component. Triangle determinant structure proves degeneracy for two zero
products and nondegeneracy for one zero plus one nonzero product; two nonzero
products still need a cancellation decision. The ca edge slot reuses ac facts
because zero support is unchanged by reversal, not because it stores signed
traversal. Changed source geometry requires recomputation or proven transport.

Homogeneous construction forwards exact polynomial expressions without dividing
by weight or proving a unique affine result. Three-plane minors can give a valid
point at infinity or an all-zero tuple when rank is insufficient. Two-plane and
line/plane wrappers likewise do not reject zero-direction, contained or otherwise
invalid carriers. Incidence tests `a*x+b*y+c*z+d*w == 0` without affine division
or carrier validation; all-zero data can therefore yield true, and an approximate
true is not an exact incidence proof. These are documented validation/evidence
limits, not newly completed support or relaxed normative requirements. Derived
coordinate equality is distinguished from projective equality up to scale.

Existing tests retain independent rational coordinates, direction/moment
expectations, symbolic incidence checks, affine rejection at infinity and
conservative zero-fact expectations. Their comments distinguish value-only
assertions from certainty/dispatch evidence. The sin(e)^2+cos(e)^2-1 helper is a
mathematical zero used by structural-only unknown expectations; stronger scalar
recognition is not contradicted. No test expectation was changed or rerun.

Validation after final source edits:

- Focused Rustfmt, `git diff --check` and comparison of nonblank/noncomment lines
  passed for all three edited files; executable source is byte-identical to the
  baseline after removing comments/blank lines.
- A scoped source scan found 85 documented named declarations including private
  and test items; fields and variants were reviewed separately. This is not a
  repository-wide coverage percentage.
- Offline locked `cargo rustdoc -p hyperlimit --lib --all-features -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links` passed. Inspected
  83 summary groups and eight selected full-contract passages across 27 rendered
  pages, including the private rational helper and Plane3's coefficient-trait
  implementation. Normal library Rustdoc omits cfg(test) modules.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling affected unit and integration consumers without executing
  their fixtures. Downstream private csgrs docs with `--no-default-features
  --features curve` also passed with locked offline dependencies and broken-link
  denial. Shared build storage remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-geometry-{doc,check,downstream-doc}.log`.
The shared temporary target remains in use. Full workflow/runtime/performance
checks remain waived for this docs-only session. The large geometric plane module,
actual predicate implementations and most kernel crates remain unaudited in full;
this batch completes neither Hyperlimit coverage nor the repository goal.

### Hyperlimit plane inputs, evidence and report validation, 26 November 2026

Baseline: `cb0673445`. Owner: Hyperlimit's geometric plane predicates and retained
reports, with scalar filters/arithmetic in Hyperreal and point facts in
Hyperlattice. Properties: P3/P4/P6/P8–P10/P19/P25/P30. Domain: explicit Real plane
coefficients, oriented triples, point/segment/triangle queries, closed boxes with
independently reversible bounds, retained filters and structural/source replay.
Missing documentation witnesses were private evidence fields, helpers/tests and
public descriptions that omitted receipt loss, validator limits and orientation
conventions. Reproduced these gaps against current source before editing.

Read all 1,771 baseline lines of `crates/hyperlimit/src/geometry/plane.rs`,
including every type, field, method, helper and test. Added documentation to all
previously undocumented named declarations, including the test-local static lock.
Supporting excerpts of Hyperlattice Point3Facts, Hyperreal filter definitions and
the orientation dispatcher establish the referenced roles; those excerpts do not
add whole-file review credit for their owning implementations.

The revised contracts document:

- Explicit planes retain unnormalized exact coefficients and accept zero normals.
  Classifiers evaluate the resulting constant expression without validating an
  ordinary geometric plane. Derived equality compares stored representations,
  not equality of oriented planes under coefficient scaling.
- Plane3Facts is a public-field snapshot. Its sparse-normal query includes the
  zero normal, and false known-zero is not proof of nonzero. Plane3Evidence stores
  filters/facts without source identity and relies on unchanged matching
  coefficients. The retained dyadic filter can answer before inspecting the
  supplied plane, and trusted exactness facts can select a known-exact scalar
  path; callers must rebuild after mutation/transform. No stale-evidence check
  or proof transport was added.
- OrientedPlane3Evidence owns its derived plane and exposes only shared access,
  but does not retain/validate the original point triple. Its normal is
  `-(b-a) x (c-a)`, with offset `((b-a) x (c-a)) . a`: positive z is Below for
  the standard counterclockwise xy triple. A collinear triple is not rejected.
  The checked i128 filter is prepared only when the dyadic filter is unavailable.
- The internal no-filter point-plane path also skips the public direct rational
  route; it uses the generic exact Real expression/sign pipeline. Prior wording
  about an identical fallback ladder was inaccurate. No dispatch ordering or
  trace count changed.
- Segment classification stops before the end query if the start is Unknown;
  explicit triangle classification eagerly computes all three side queries before
  checking outcomes. Both completed relation outcomes join consumed receipts.
  TrianglePlaneReport instead extracts bare side values, losing certainty,
  escalation and Unknown diagnostics. Its approximate-policy reports therefore
  remain an evidentiary gap rather than certified support.
- Triangle report validation checks the side/relation truth table. Replay compares
  those fields under the selected policy, and matching Unknown reports can pass.
  Indexed replay checks bounds but not distinctness, nondegeneracy or original
  source identity. Success is not a new geometry/receipt certificate.
- AABB support selection compares x/y/z products, keeps the authored endpoint
  pairing on ties, then resolves the upper expression before the lower. Both
  signs are required. Exact point expressions survive, but support selection and
  signs can be policy-qualified; the outer outcome owns their aggregate receipt.
- AABB validation checks value order, signs and relation, not support points or
  axis-ordering fields. Unknown comparisons are folded into existing mismatch
  errors; approximate validation may return Ok without retaining its certainty.
  Source replay compares all stored fields after local validation, discards replay
  certainty and maps unresolved replay to SourceReplayMismatch. Representation or
  tie-corner differences can fail this equality despite equivalent geometry.

These are current behavior and implementation-limit descriptions, not a normative
contract relaxation or completed closure claim. Source/receipt gaps are not fixed
by documenting them. All seventeen tests retain their original assertions,
including the dispatch witness's one rational-filter hit and two zero-hit counts.
Test docs distinguish independent rational/geometric expectations, value-only
agreement, full receipt comparisons, unknown paths and actual trace evidence.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing only blank/comment lines
  gives byte-identical source to baseline; no expression, fixture, signature,
  field layout, call order or assertion changed.
- A scoped scan checked 83 documented named declarations including private items,
  tests and the local static; fields/variants were separately reviewed. This is
  not a repository-wide coverage metric.
- Private Rustdoc with `--locked -p hyperlimit --lib --all-features --offline -j2`
  and `--document-private-items -D rustdoc::broken_intra_doc_links` passed.
  Inspected 94 rendered documentation groups across 42 module pages and fourteen
  specifically selected contract pages, including private fields and helpers.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling the affected private and integration tests without executing
  them. Normal library docs omit cfg(test), whose comments were source-reviewed.
- Downstream private csgrs docs with `--no-default-features --features curve`
  passed with locked offline dependencies and broken intra-doc links denied.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-plane-{doc,check,downstream-doc}.log`.
The shared target remains about 1.6 GiB. Full workflow/runtime/performance checks
remain waived for this docs-only session. All eleven Rust files directly under
Hyperlimit's src and all five under src/geometry now have whole-file records;
actual predicate implementation modules, most tests and other crate files remain
substantial work. This is a scoped review milestone, not complete crate coverage.

### Shared decision receipts and composite-policy witnesses, 26 November 2026

Baseline: `140ee89e7`. Owner: Hyperlimit's private composite-decision bookkeeping,
predicate module declarations and the composite-policy integration consumer.
Properties: P4/P6/P8–P11/P19/P30. Domain: Decided/Unknown child outcomes, local
certainty/stage accumulation, strict and terminal-permitted composite reports.
The missing witnesses were undocumented private records/fields/functions and
test helpers; the old module introduction incorrectly said strongest certainty
and implied retention of the first unknown. Reproduced against the current
match branches and rank tables before editing.

Whole-file reviews: `crates/hyperlimit/src/predicates/decision.rs` (90 baseline
lines), `crates/hyperlimit/src/predicates/mod.rs` (18), and
`crates/hyperlimit/tests/composite_policy.rs` (234). All named declarations in
these files now have explanatory docs: nine decision declarations with four
fields, eighteen predicate-module declarations and nine test/helper functions.
The module index describes operation roles without claiming their implementations
have all been reviewed. Supporting reads of the first part of `predicates/exact.rs`
and the headers of `nd.rs` and `order.rs` are excerpts only.

DecisionTrace joins the weakest consumed qualification and highest ranked stage,
not the strongest proof or a chronological trace. Its Exact/Structural default
is an aggregation identity, not a geometry proof. `decided` updates it only on
Decided; Unknown returns its own hint/stage without mutation, and the caller
controls whether to stop. UnknownDecision reconstructs an Unknown outcome without
a value or earlier receipts. No policy-admissibility check, source provenance,
worker aggregation or resource counter is supplied by these helpers.

The six composite-policy fixtures preserve both the original now-recognized
unit-circle identity and separate angle-addition terminal controls. Docs explain
exactly which tests assert receipts, relation/location, zero ray parameter and
strict source replay. The constant-halfspace terminal witness checks qualification,
not its returned feasibility status. Trigonometric proper-hit and vertex-hit
comments no longer claim that an identity is unprovable or that the fixture
instruments absence of a coplanarity retest. The geometry, policy sequence,
assertions and historical fixture names remain unchanged.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Nonblank/noncomment lines are
  byte-identical to baseline in every edited file.
- A scoped declaration scan checked 36 documented names across the three fully
  reviewed files. Private fields were reviewed separately. This is not a global
  Rustdoc coverage percentage.
- The first all-feature private Rustdoc attempt failed on three unresolved links
  in existing module introductions after adding outer module documentation.
  Fixed only those links: Real and PredicateOutcome in `predicates/nd.rs`, and
  PredicateOutcome in `predicates/order.rs`, now use explicit targets. These
  two narrowly edited files remain pending whole-file review.
- Repeated `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline
  -j2 -- --document-private-items -D rustdoc::broken_intra_doc_links`: passed.
  Inspected 31 declaration/field/module documentation groups on rendered pages
  and confirmed the three corrected links have rendered type-page targets.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed. This compiles the composite-policy consumer and private-helper users
  without executing the runtime corpus. Test-only comments were source-reviewed.
- Downstream private csgrs docs with no default features and `curve` passed with
  locked offline dependencies and broken intra-doc links denied. Shared build
  storage remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-decision-{doc,check,downstream-doc}.log`.
The shared temporary target is reused. The owner continues to waive full workflow,
runtime and performance checks for this docs-only session. No implementation,
fixture, gate, predicate order or dispatch count changed; most predicate module
bodies and the remaining repository still need whole-file review.

### Rational and general-dimensional determinant kernels, 26 November 2026

Baseline: `28ffd9a7b`. Owner: Hyperlimit's fixed rational determinant schedules
and general-dimensional predicate implementation. Properties: P1/P3/P5/P6/P8–P11
and P37. Domain: structurally rational fixed-dimensional point tuples and
positive-dimensional coordinate arrays, including degenerate tuples, symbolic
fallbacks and malformed shapes. Before editing, reproduced missing private
helper/test docs, an incorrect lifted-row coordinate index and the subset-state
complexity claim by reading the implementations on this revision. The acceptance
witness is documented declarations plus rendered, linked contracts checked
against the unchanged expressions; this batch extends no geometric capability.

Whole-file reviews: `crates/hyperlimit/src/predicates/exact.rs` (287 baseline
lines) and `crates/hyperlimit/src/predicates/nd.rs` (527). All 42 named
declarations, including test modules, helpers and constants, have Rustdoc.
Supporting excerpts in Hyperreal's `real/arithmetic/facts.rs`, Hyperlimit's
`predicates/orient.rs` and the public re-export index do not establish new
whole-file review credit for those implementation files.

The fixed kernels borrow only structurally rational coordinates; a declined
path is not an irrationality or invalid-geometry proof. Docs specify the raw
orientation and lifted circle/sphere matrices, including degenerate determinant
zeros that do not establish a unique circle/sphere. Fixed 3D orientation uses
rows a-d, b-d, c-d, whereas general-dimensional orientation uses columns
points[i+1]-points[0]; the ordered standard tetrahedron consequently has opposite
signs. Public cross-references make this convention difference explicit.
Rational squared lifts and product-order cofactor helpers now explain their
mathematical role without moving scalar arithmetic into a new authority.

General-dimensional docs distinguish shape validation from affine independence,
raw lifted signs from inside/outside interpretation, Exact/Structural rational
outcomes from policy-qualified symbolic outcomes, and an approximate zero from
an exact dependence proof. The reduced rational sphere matrix uses coordinate
differences and a difference of squared norms, preserving the full determinant
by query-row subtraction and last-column expansion. Bareiss pivot selection,
swap parity and the empty determinant convention are documented. Its cubic
rational-operation count is not a bit-complexity or memory-byte bound. Removed
the unmeasured blanket intermediate-growth comparison with recursive minors.
Subset expansion has 2^n table slots and O(n 2^n) scalar arithmetic steps;
the size-sixteen dispatch threshold does not bound the recursive fallback's
factorial work or total expression size. No total resource guarantee is added.

The unchanged tests compare exact schedules on finite integer corpora, retain
symbolic rational declines and policy controls, and cover invalid shape, the
empty determinant and a sparse size-seventeen fallback. Their docs state what
is asserted and distinguish shared-arithmetic comparisons from independent
universal proofs. Runtime fixtures were not executed in this batch.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing only blank/comment
  lines yields identical source to baseline; signatures, expressions, call
  order, fixtures and assertions are unchanged.
- Scoped declaration checks found 15 documented names in exact.rs and 27 in
  nd.rs. Inspected all 28 non-test function documentation blocks in rendered
  private pages and seven selected sign/layout/resource contract pages.
  Test-only docs were source-reviewed because library Rustdoc omits cfg(test).
- Initial private Rustdoc passed but warned about a public link to a private
  kernel. Replacing it first exposed an incorrect guessed public name; changed
  it to the actual exported `orient3` and inspected the rendered target.
  Final `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline
  -j2 -- --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed after the final source edit, compiling affected test consumers.
- Downstream private csgrs Rustdoc with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed. Shared target storage remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-determinants-{doc,check,downstream-doc}.log`.
Full workflow, runtime and performance checks remain waived by the owner for
this docs-only session. No measured performance or implementation-completeness
claim follows from this documentation batch. Most predicate modules and the
remaining repository still need whole-file review; the existing future-agent
100% public/private Rustdoc mandate remains in force.

### Interval classification and enclosure-filter receipts, 26 November 2026

Baseline: `9712bd2c5`. Owner: Hyperlimit's interval normalization/classification
and enclosure-sign filters, with the shared relation vocabulary. Properties:
P3/P4/P6/P8–P11/P37. Domain: exact Real endpoints in either order, positive- and
zero-length intervals, center/radius enclosures, strict Unknown and approximate
terminal outcomes. The missing acceptance witnesses were undocumented private
helpers/tests and public descriptions claiming all decisions were certified.
Read current producer branches and comparison/receipt helpers before editing.

Whole-file reviews: `crates/hyperlimit/src/predicates/filters.rs` (333 baseline
lines) and `crates/hyperlimit/src/predicates/interval.rs` (527). Both now document
all eighteen named declarations apiece, including test modules, constants and
the nested Unknown helper. Also documented all three private BallFilterResult
variants and its tuple payload. Corrected the ClosedIntervalIntersection docs
in the previously reviewed `classify.rs`; this narrow revisit does not count as
another whole-file pass. AABB consumer excerpts establish why zero-extent
information matters but do not earn whole-file review credit.

Filters compare supplied endpoint values under the policy and can return
Approximate even through a `certified_*` function. Their module and public docs
now require inspection of certainty. No target value or source relation is
passed, so proving that the supplied enclosure contains the intended target
remains the caller's obligation. Endpoint comparisons run first then second,
stop on Unknown, and decide only matching positive/negative/zero signs. A single
zero endpoint declines. Filter stage replaces child stages while the weakest
certainty survives. The ball radius is compared before endpoint construction;
negative radius becomes Unsupported and unresolved/crossing cases become
RealRefinement, both at Filter stage. The Option wrapper merges those failures.
These wrappers do not retain child failure details or full source provenance.

Interval classification normalizes first, tests the lower endpoint before the
upper, and gives AtLowerEndpoint precedence for a matching singleton. Intersection
normalizes both inputs, tests first upper against second lower, then reverses
the separating comparison only if needed. The extent Boolean records either
normalization equality and has the same qualification as the outcome.

A newly recorded limitation: the strict rational input `[1,1]` versus `[0,2]`
produces Overlapping, despite the intersection being a singleton. Source replay
of its comparisons is Equal, Less, Greater, Greater, selecting that branch;
the extent-bearing internal report additionally returns true, but the public
relation discards that flag. The same holds with operand order reversed. Enum
and function docs now describe this degenerate case and require separate extent
evidence before inferring positive length. This is an implementation limitation,
not a migration weakening the normative exact classification contract. No new
runtime regression test was added or executed in this docs-only batch; the
existing positive-length fixtures do not cover this contained-singleton gap.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Nonblank/noncomment source is
  identical to baseline except the documented tuple payload's whitespace and
  optional trailing comma; normalizing that one variant makes it identical too.
  No expressions, signatures, call order, fixtures or assertions changed.
- Scoped source checks found 36 documented named declarations. Inspected all
  eighteen non-test function documentation blocks in private rendered pages
  and seven selected contract/enum pages, including the private tuple payload.
  Test-only comments were source-reviewed; normal library docs omit cfg(test).
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed, compiling consumers and existing test fixtures without running them.
- Downstream private csgrs Rustdoc with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed. The reused target remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-interval-filters-{doc,check,downstream-doc}.log`.
The owner's docs-only waiver remains in effect for full workflow, runtime and
performance checks. No implementation-completeness or performance claim is
made. The repository-wide audit and public/private documentation mandate remain
active; the remaining predicate modules and other files still need review.

### Scalar ordering, coordinate witnesses and selection, 26 November 2026

Baseline: `47db1d795`. Owner: Hyperlimit's scalar order/equality/selection
predicates and their point-equality integration consumer. Properties:
P1/P3/P4/P6/P8–P11/P37. Domain: exact Real values and 2D/3D coordinates,
rational and symbolic comparisons, strict Unknown, terminal-qualified results,
ordered clamp bounds and reciprocal construction. Missing witnesses included
private helper/test docs, a paired-sign description incorrectly claiming one
cascade, and undocumented receipt/error boundaries. Reproduced dispatch and
return branches on this revision before editing; no implementation changes.

Whole-file reviews: `crates/hyperlimit/src/predicates/order.rs` (891 baseline
lines) and `crates/hyperlimit/tests/point_equality.rs` (100). They now document
all 37 and five named declarations respectively. A local fixture-building
closure also has an explanatory source comment. Supporting excerpts of
Hyperreal's `arithmetic/facts.rs` and `arithmetic/inversion.rs` establish the
called comparison and assuming-nonzero contracts; those files remain outside
this batch's whole-file credit. Previously reviewed PredicateOutcome conjunction
semantics were checked again for the coordinate witness explanation.

Scalar comparison tries direct rational ordering, certified scalar comparison
down to precision -512, then a policy-controlled difference-sign resolver.
Docs separate precision from total work and preserve qualification on terminal
Equal. Paired signs use a rational shortcut or evaluate both scalar cascades
left then right, forwarding the first Unknown if needed. Receipt ranks describe
the weakest consumed certainty and highest stage kind, not execution chronology.

Coordinate equality searches under STRICT first and continues past Unknown to
find a later unequal coordinate. A false witness supplies its own sufficient
receipt; true requires all coordinates and composes their receipts. Only a still
unresolved strict pass is retried under a different requested policy. Rational
2D shortcuts retain their existing order. Lexicographic comparison instead
stops on Unknown or a non-equal earlier coordinate, and joins later receipts
only after Equal. Coordinate equality does not establish construction/source
identity. Private helpers explain the empty conjunction and replay behavior.

Min/max select the left reference on ties and retain qualification. Clamp first
checks bound order, rejects reversed bounds with Unknown/Unsupported at
Undecided stage, then checks the lower and upper comparisons in order; equality
with the lower bound still requires the upper comparison. It returns a bound
clone or the original owned value with the consumed decisions' receipt.

Reciprocal construction forwards Unknown, uses the scalar assuming-nonzero
constructor on a decided nonzero sign and preserves its receipt on success.
Any decided Zero, including Approximate, becomes Problem::DivideByZero without
its certainty receipt. Documentation now states that this error alone cannot
prove mathematical zero under a terminal-permitting policy. This information
loss is recorded as a current API limitation, not repaired by documentation.
The private quotient helper still decides the denominator for a zero numerator.
No new error or zero-handling behavior is introduced.

Test documentation identifies finite value assertions, selected receipt/rank
checks and the 2^-600 symbolic residual proof retained in scalar facts. Existing
historical test names are unchanged; their docs do not claim dispatch or reuse
counts that assertions do not check. The point-inequality policy loop creates
fresh inputs each iteration; the original unit-circle equality fixture reuses
its objects through strict/approximate/strict queries. The unresolved
angle-addition fixture is described as a current proof-frontier control, not
an impossibility claim. The original certified identity remains intact.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing only comment/blank
  lines yields byte-identical source to baseline in both files. No calls,
  signatures, branches, fixtures, assertions or dispatch counts changed.
- Scoped declaration checks found 42 documented names. Inspected all 22
  non-test function documentation blocks in rendered private pages and seven
  selected reciprocal, paired-sign, ordering, equality and clamp contract pages.
  Test-only comments were source-reviewed, not claimed as library-rendered.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed, compiling the affected integration and private fixtures.
- Downstream private csgrs documentation with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed. The shared build target remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-order-{doc,check,downstream-doc}.log`.
Runtime fixtures, full workflow and performance checks remain waived for this
docs-only session; no runtime or universal-completeness claim is made. The
repository-wide audit remains active, with most predicate modules, tests and
other repository files still requiring whole-file review and documentation.

### Orientation and retained lifted-predicate evidence, 26 November 2026

Baseline: `b50c2c58f`. Owner: Hyperlimit's orientation/lifted-sign predicates,
retained line/circle/sphere evidence and scheduling snapshots. Properties:
P1/P3/P4/P5/P6/P8–P11/P37. Domain: ordered exact Real source/query coordinates,
rational and symbolic representations, degenerate source tuples, retained
queries, explicit policies and filter declines. Missing acceptance witnesses
were undocumented private helpers/fields/tests, vague spatial sign conventions,
and insufficiently explicit source-association and fact-versus-proof boundaries.
All findings were checked against current dispatch branches before editing.

Whole-file review: `crates/hyperlimit/src/predicates/orient.rs`, all 2,588
baseline lines. All 118 named declarations now have Rustdoc, including private
functions/types, test functions inside proptest, the test module, constant and
nested static. All 47 fields were reviewed and documented, together with the
point re-export and module contract. A narrow LineSide clarification in the
previously reviewed `classify.rs` does not count as a new whole-file review.
Supporting reads of Hyperreal's `arithmetic/linear_algebra.rs`,
`arithmetic/facts.rs` and `exact_set.rs` were excerpts only and earn no whole-file
credit. They establish the called scalar filter/storage and scheduling roles.

Docs specify planar determinant orientation, the spatial rows a-d/b-d/c-d and
its negative standard-tetrahedron sign, and the full lifted circle/sphere row
layouts. For a nondegenerate tetrahedron the interior sphere sign matches its
spatial orientation, so the standard tetrahedron's interior sign is negative.
Zero determinants can result from repeated or dependent defining points;
construction of a retained polynomial is not a circle/sphere existence proof.
Line classification preserves the outcome receipt, but mapping a decided zero
to On does not validate distinct line endpoints. Approximate terminal outcomes
remain qualified, and fixed schedules do not add a total resource contract.

Retained line evidence owns dyadic/checked-word arithmetic data without binding
later source arguments. Circle/sphere evidence owns exact lifted coefficients
and optional filters without retaining the authoritative source Point objects.
Those paths can answer before the supplied source points are read. The caller
must preserve the same ordered exact source coordinates and rebuild after
changes/transforms; no source comparison, invalidation or replay validation is
provided here. Retained query coordinates have the same unchecked association
requirement. An empty supplied word query skips that path instead of rebuilding
it from the current point. The fallback can use current source coordinates,
so stale evidence is not uniformly detected or made safe by fallback behavior.

Public PredicateFacts and LiftedPolynomialFacts are mutable-value snapshots,
not source-bound certificates. Their counts/masks and schedule priorities are
documented; inconsistent supplied facts are not validated. Current retained
query paths inspect scalar values/filters rather than dispatching from the
coefficient-fact snapshot. The sparse-coefficient hint includes an all-zero
polynomial and does not certify geometry. Private mask builders preserve
known-zero/nonzero/unknown distinctions and require at most 128 entries. Scalar
representation and zero-status authority remain in their existing lower layers.

Private arithmetic docs describe exact norms, unit-column reduction, cofactor
signs and fixed product sums without changing their order. The retained rational
circle filter produces Exact/Filter, followed by Exact/Exact signed-sum fallback;
the sphere's retained rational sum produces Exact/Exact. The private
exact_outcome wrapper calls its callback once and attaches Exact/Exact, while
its policy and kernel-tag arguments are unused. It does not validate that proof
or preserve the kernel tag in the result. No new arithmetic authority was added.

Test docs distinguish value comparisons from full receipts and exact trace
counts. Existing word-filter witnesses retain their one-hit/zero-fallback
assertions; the overflow witness retains zero word hits/one rational hit;
retained line routing retains one hit for each of three query representations.
The five bounded property generators still request 256 cases each. The wide
orientation generator uses an independent i128 determinant before positive
scaling; the other equivalence fixtures share implementation/scalar machinery.
Repeated-source-row zeros do not certify nondegenerate circles/spheres. Private
tests that deliberately disable filters or change a coefficient exercise source
callback reachability, not stale-evidence safety. Binary64 fixture literals
remain exact binary values rather than their displayed decimal ratios.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing only comment/blank
  lines yields byte-identical source to baseline in both edited Rust files.
  No signatures, expressions, branches, fixture values, assertion counts or
  dispatch order changed.
- Scoped declaration scan checked all 118 names. Inspected 124 rendered
  documentation groups across 54 private module pages, including all 47
  fields, plus twelve selected sign/evidence/dispatch contracts and the updated
  public LineSide limitation. Test-only docs were source-reviewed rather than
  claimed as rendered by normal library Rustdoc.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed after the final source edit, including compilation of trace and
  proptest fixtures. These runtime fixtures were not executed in this batch.
- Downstream private csgrs Rustdoc with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed. The reused build target remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-orient-{doc,check,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived for this docs-only
session. No timing, stale-evidence validation or universal closure claim is
established by these documentation changes. The audit remains active; the
remaining predicate modules, most tests and other repository files still need
whole-file review. The existing future-agent public/private coverage mandate
remains in force.

### Convex carriers and ring classification/report boundaries, 26 November 2026

Baseline: `e20c1cfae`. Owner: Hyperlimit's convex halfspace composition, polygonal
ring signs/turns/even-odd queries and retained report validation. Properties:
P3/P4/P6/P8–P11/P17/P19/P37. Domain: cyclic exact Real vertex lists, indexed
lists with repetitions/invalid indices, oriented plane lists, degenerate
carriers, strict Unknown and terminal-qualified decisions. Missing witnesses
were private helper/test docs and descriptions overclaiming certification,
source validation and geometric meaning. Reviewed the current branches before
editing, including their unchanged early-exit and predicate order.

Whole-file reviews: `crates/hyperlimit/src/predicates/convex.rs` (326 baseline
lines) and `crates/hyperlimit/src/predicates/ring.rs` (1,167). All twelve convex
and 52 ring named declarations have Rustdoc. Reviewed all eighteen ring fields
and eight validation-error variants. A narrow ConvexPointLocation correction
in the previously reviewed `classify.rs` receives no new whole-file credit.
Supporting reads of the collinear segment helper and the first 150 lines of
`tests/retained_report_contracts.rs` are excerpts only. The previously reviewed
plane implementation's no-filter wrapper was checked for the convex caller.

Convex polygon classification assumes ordered convex input but does not check
simplicity or nonzero edges. It decides the shoelace sign, then compares each
cyclic edge orientation, stopping at opposite sign or Unknown. A repeated
closing vertex adds a zero-length edge and marks otherwise interior points
Boundary. The concrete source witness is square (0,0),(4,0),(4,4),(0,4) queried
at (2,2): four positive edge tests give Inside; appending (0,0) adds a zero
orientation and gives Boundary. This is a documented implementation limitation,
not an intentional narrowing of the normative geometry contract or a runtime
test result. Existing convex fixtures do not cover this repeated-closing case.

Plane composition uses the generic point-plane route without front filters,
Below/On as admissible sides, and an empty-list Degenerate convention rather
than whole-space membership. It does not validate normals, feasibility,
boundedness or dimension. A constant zero constraint contributes On and can
turn an otherwise interior classification into Boundary. Both convex paths
retain consumed qualification and highest stage; their first Unknown can stop
before a later exclusion witness. The enum docs now reflect producer limits.

Ring area is the shoelace algebraic area sign, not the even-odd filled area for
a self-intersecting walk. Rational inputs use sign-only exact accumulation;
symbolic inputs retain an expression and signed terms for policy resolution.
Short rings give Exact/Structural Zero. Ring2Facts computes area and turns
before raw cyclic-edge counts, discarding the area and turn receipts. Those
fields cannot certify an approximate-policy result. Local turns remove only
one structurally equal closing vertex, stop on Unknown or mixed turns, and do
not prove simplicity/global convexity. These are existing evidence gaps, not
fixed by documentation.

Even-odd classification processes each edge's orientation and possible segment
boundary before its y-straddle, rather than globally testing all boundaries
first. It compares a.y then b.y with query y, derives upward from b_above on a
straddle and uses the orientation sign for positive-x crossing. It constructs
no intersection coordinate. A boundary returns a retained prefix; Unknown
returns no partial report. Short rings return Outside even at their supplied
point/segment. Indexed entry points check all indices first and keep ring-order
positions in reports. Wrapper projections preserve outer certainty/stage.

Report fields retain decision values without their own certainty, source
coordinates or identity. Metadata validation checks count constraints, bounded
indices, edge shapes, at most one boundary, crossing count and coarse location.
It does not check index uniqueness/order, boundary-last traversal, upward equal
to b_above, or agreement of a non-boundary segment label with orientation.
SegmentLocationMismatch exists but is not emitted by these validators.
Source-level acceptance witnesses for these limits (not executed runtime tests):

- Three identical nonstraddling records with edge index zero, edge_count three,
  no boundary, zero crossings and Outside satisfy every current validation
  branch despite omitting two edge indices.
- A straddle with a_above false, b_above true, upward false, Negative orientation
  and crosses_right true satisfies the stored crossing truth table despite
  contradicting the producer's upward rule; a matching aggregate parity passes.

These examples are missing validation guarantees, not authorized proof fixtures.
Source replay first checks metadata, then compares the entire newly decided
report; Unknown or differing values becomes SourceReplayMismatch. Its receipt
is discarded, so matching approximate replay can return Ok without exact
certification. Reordered but geometrically equal sources need not reproduce the
same report. Indexed reports require replay vertices in their original ring order.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Nonblank/noncomment source is
  byte-identical to baseline in every edited Rust file; all fixtures,
  assertions, signatures, branches and predicate order remain unchanged.
- Scoped scans checked 64 named declarations. Inspected 58 rendered doc groups
  across 28 private module pages, including eighteen fields/eight variants,
  ten selected qualification/validation/degeneracy pages and the final error
  type summary. Test-only docs were source-reviewed, not library-rendered.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed, compiling affected fixtures and report consumers without running them.
- Downstream private csgrs Rustdoc with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed after the final source edit. Shared build storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-ring-convex-{doc,check,downstream-doc}.log`.
The owner continues to waive full workflow/runtime/performance checks for this
docs-only session. No implementation repair, runtime completeness or cost claim
is made. The full repository audit remains active; other predicate modules,
most tests and remaining repository files still need whole-file review.

### Segment/plane construction events and validation, 26 November 2026

Baseline: `966dedc0b`. Owner: Hyperlimit's segment/plane relation construction,
retained ratio/event fields and local/source validation. Properties:
P1/P3/P6/P8–P11/P17/P19/P30/P37. Domain: point-defined or explicit planes,
segment endpoints, supplied plane values/side labels, exact affine parameters,
strict Unknown and policy-terminal decisions. Missing witnesses included
private helper/field/test docs and public descriptions calling receipt-free
events certified. Reproduced the current projections, constructors and validation
branches before editing; the existing forged-point fixture directly exposes
the local/source-validation distinction.

Whole-file review: `crates/hyperlimit/src/predicates/segment_plane.rs`, all 996
baseline lines. All 36 named declarations, fourteen fields and 24 enum variants
have explanatory Rustdoc. The previously reviewed ordering/division helper was
revisited to verify Unknown, scalar-error and qualification handling. No other
file receives new whole-file review credit in this batch.

The point-defined entry point evaluates both endpoint orientations before
deriving its plane and both plane values, even if a side is already Unknown
or interpolation will not be needed. The explicit-plane entry point builds
both values before its two comparisons. Both discard the side predicates'
certainty/stages and unresolved hints. Events are therefore exact expressions
plus labels, not self-contained certified constructions. Degenerate planes
and segments are not independently rejected. Documentation no longer implies
that the returned record establishes source validity or qualification.

The values-based entry point trusts the supplied side labels and their
association with d0/d1/endpoints. Missing labels select Unknown; On/On selects
Coplanar; one On copies an endpoint; equal strict labels select Disjoint;
opposite labels attempt t=d0/(d0-d1). The crossing constructor does not verify
opposite signs, source incidence or the open-unit parameter bound. It can
construct extrapolated points when called directly with another nonzero ratio.
Successful division discards its receipt; DivideByZero maps to ZeroDenominator,
including a possible terminal Zero, while Unknown and other errors collapse
to ParameterDivisionFailed. Existing tests deliberately supply inconsistent
side/value pairs to exercise these branches; they do not prove those inputs
describe geometric crossings.

Interpolation preserves start+t*(end-start) for arbitrary Real t. Plane
evaluation is the unnormalized linear form, not generally signed Euclidean
distance, and a zero normal is evaluated as a constant. Axis-parameter extraction
tests only its one denominator; it merges errors/Unknown into None and loses
successful certainty. Neither a unit-interval bound nor agreement with other
coordinates is established by that scalar parameter.

Local validation checks relation/field shapes and retained side labels. Endpoint
events additionally compare t with the selected 0/1 index; proper crossings
require decided 0<t<1 and agreement with a quotient rebuilt from retained ratio
fields. It does not inspect point coordinates or establish the ratios as source
plane values. Unknown comparisons/division become mismatch errors, and accepted
approximate decisions have no returned receipt. A failure record's reason is
required but not independently reproduced. The unchanged fixture explicitly
replaces a proper-crossing point with (9,9,0), accepts local validation, then
rejects source replay.

Source validation first runs those checks, then uses derived field equality
against a fresh point-defined event. This includes Real structural equality,
not universal geometric equality, and does not bind a unique source. Matching
Unknown/failure events may pass; replay cannot restore missing certainty or
certify approximate history. Enum/error docs now distinguish stored label
patterns, missing fields, unresolved scalar checks and actual replay mismatch.
The private field-bundle constructors only organize data; they do not add proofs.

Test docs state the finite assertions: successful relation/side controls,
midpoint ratio replay, selected forged-field failures, a proved tiny positive
denominator, unresolved tiny symbolic divisions, and rational/symbolic axis
parameters. Historical test names remain unchanged, without claiming all
inconsistencies are caught or all decided relation classes occur in one fixture.
All original fixtures, assertions and expressions are preserved.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment lines
  gives byte-identical source to baseline; signatures, fields, branches,
  predicate order, fixtures and assertions are unchanged.
- A scoped scan checked 36 documented names. Inspected 65 rendered doc groups
  across nineteen private module pages, including fourteen fields/24 variants,
  and eight selected construction/receipt/validation contracts. Test-only docs
  were source-reviewed rather than claimed as rendered by library Rustdoc.
- The first private Rustdoc attempt rejected three unformatted `[0,1]` literals
  as unresolved links. Changed them to inline code. Final `cargo rustdoc --locked
  -p hyperlimit --lib --all-features --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links -D rustdoc::private_intra_doc_links` passed
  without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed, compiling affected fixtures/consumers without executing them.
- Downstream private csgrs Rustdoc with locked offline dependencies,
  `--no-default-features --features curve` and broken intra-doc links denied
  passed. Shared target storage remains approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-segment-plane-{doc,check,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived for this docs-only
session. Documenting the receipt and validation gaps does not repair them or
establish completed geometric support. The repository-wide audit remains
active, with remaining predicate modules, tests and other files still to review.

## Hyperlimit coplanar reports and projection helpers

Reviewed all 1,359 baseline lines of
`crates/hyperlimit/src/predicates/coplanar.rs` at `0ee79cde6`, including all
private helpers and test fixtures. Also revisited the relevant segment-label
helper and shared symbolic-zero fixture; those excerpts do not add whole-file
credit. Ownership is Hyperlimit's projected predicate/construction boundary.
The affected properties are policy-explicit decisions and receipts (P8/P9),
least-cost proof order (P10), explicit uncertainty (P11), and source-bound
validation. Inputs are exact Real-coordinate triples, indexed triangles,
projected rings/vectors, and retained reports under explicit predicate policies.
The acceptance witness was missing private/test documentation plus claims of
certification contradicted by the current return types and value extraction.
This is a documentation correction, not a change to the required contracts.

The module now distinguishes receipt-bearing triangle degeneracy from bare
coplanar reports. Degeneracy searches XY/XZ/YZ by arithmetic stage: dyadic
filter, checked-word kernel, rational filter, full rational kernel, then generic
policy-driven orientation for unresolved planes. Exact zeros suppress later
work on that projection. A nonzero witness alone suffices and returns its own
receipt, even after an unresolved projection. All-zero completion joins zero
certainty and stage from an Exact initial receipt. Without a nonzero witness,
an unresolved projection produces the first retained Unknown. Documentation
preserves this order and the approximate-degeneracy possibility.

Coplanar classification assumes rather than verifies 3D coplanarity. Projection
selection uses the first left-triangle plane with a policy-decided nonzero
orientation, dropping its receipt; the right triangle is not independently
checked for degeneracy. Invalid indices and unsuccessful projection selection
both produce projectionless Unknown. All nine edge pairs are evaluated in
left-edge-major order unless one is unresolved; a known contact does not stop
that work. Then all right vertex queries precede all left queries. Any missing
vertex location clears every vertex label from the Unknown report, while
retaining its edges. Overlap witnesses take precedence over touch; Degenerate
vertex locations are ignored. These labels alone do not prove positive area
for degenerate input. The general label reducer even maps empty/missing facts
to Disjoint, never Unknown, and the segment helper accepts Identical point
segments as overlap without establishing positive length.

Local report validation accepts every Unknown carrying a projection without
checking the remaining fields. Without a projection, only fact-free Unknown
passes. Decided reports require nine edge labels, six present vertex labels,
and agreement with the reducer. No source incidence, projection validity,
fact ordering, coplanarity, or certainty is checked. Replay first performs
these checks and bounds checks, then requires field equality with a fresh
report. Matching Unknown can pass; this does not restore dropped receipts or
bind a unique source. Public fields remain mutable. The docs now state those
limits rather than claiming a certified handoff.

Projection clones coordinates without establishing faithful source incidence.
Projected area is signed algebraic shoelace area, not general 3D Euclidean or
filled-set area; the sign helper preserves its outcome. The absolute-value
helper drops the comparison receipt and retains the signed expression on Equal,
so a terminal Equal need not produce a nonnegative exact expression. Turn
ordering relies on unchecked nonzero vectors, uses left bucket before right,
and drops comparison receipts. Point-in-projected-triangle preserves its 2D
receipt but does not establish 3D coplanarity.

Parameter recovery tries projected x before y, falling back on any None,
including uncertainty or errors. It establishes neither cross-coordinate
agreement nor a unit-interval bound. Projected line intersection uses
`d0/(d0-d1)` and interpolates all original 3D coordinates; skew 3D lines can
still intersect in projection. Division errors and Unknown collapse to None,
and successful certainty is lost. Midpoint, vector, raw determinant,
interpolation, cross/dot, bucket, indexing, edge ordering, report reduction,
and receipt-join helpers now document their specific roles and limits.

Test docs describe their actual finite assertions without upgrading names to
coverage claims. In particular, the Unknown-completion fixture checks the
relation rather than retained vertex labels, the strict unresolved-edge fixture
does not inspect its edge-prefix content, and the representation-stage fixture
does not count dispatches. Existing fixtures, assertions, predicate order, and
implementation expressions are unchanged.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing blank/comment-only
  lines leaves byte-identical source to the baseline.
- A scoped declaration scan found 55 documented names. Checked 60 complete
  rendered documentation groups across 37 private-module pages, including all
  five fields and fifteen enum variants. Test-only documentation was reviewed
  in source rather than claimed as rendered by library Rustdoc.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed; fixtures compiled but were not executed.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed. Shared target storage remains
  approximately 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-coplanar-{doc,check,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived for this docs-only
session. These records expose existing qualification/validation gaps; they do
not repair them or establish completed geometric support. Repository-wide
coverage and the line-by-line audit remain incomplete.

## Hyperlimit triangle-pair report composition

Reviewed all 1,141 baseline lines of
`crates/hyperlimit/src/predicates/triangle_triangle.rs` at `f68fc6c10`, including
private aliases/helpers and every test. Revisited the composite-policy wrapper,
preclassified segment/triangle implementation, shared relation vocabulary,
and triangle-pair subsection of `tests/retained_report_contracts.rs`; these
supporting excerpts do not constitute additional whole-file review. Ownership
is Hyperlimit's 3D predicate composition and retained-report boundary. Relevant
properties are policy and receipts (P8/P9), dispatch order (P10), uncertainty
(P11), and source validation. Inputs are exact Real-coordinate closed triangle
pairs, including degenerate/coplanar pairs and public mutable reports.
Acceptance witnesses were undocumented private/test declarations and overly
strong claims that every accepted relation was replayable from retained facts.
Implementation inspection on this revision identifies missing coplanar-edge
contact data and unchecked report shapes; documenting them does not change the
contract or repair the implementation.

Public classification uses strict-first composite replay. Under APPROXIMATE_512,
a decided strict pass remains unchanged; strict uncertainty triggers an
approximate replay whose decided outcome is conservatively Approximate/Refined.
The internal pass discards child receipts and attaches Exact/Exact to success,
so its stage does not represent the deepest child escalation. Its direct use
under approximate policy does not independently preserve qualification. The
report alone has neither certainty, source identity, nor source coordinates;
consumers must retain the public outcome. Replay supplies no shared total-work
budget or rollback across evaluations.

Evaluation order is left degeneracy, right degeneracy, then the degeneracy
branch. Even a decided degenerate left input still queries right degeneracy;
an unresolved right result returns Unknown before publishing a degenerate
report. The two plane queries follow right-against-left then left-against-right.
Both Coplanar selects projected classification; either strict separating plane
selects Disjoint without edges. Remaining cases query left edges before right,
using directed 01/12/20 order and reusing endpoint sides. Proper crossing beats
boundary contact. Success retains all six edge labels; uncertainty publishes
no partial arrays, although private accumulated flags can already be updated.
An unresolved coplanar report becomes Unsupported/Undecided without preserving
its original reason. Other child Unknown need/stage pairs are forwarded.

A Coplanar segment label means the edge lies in the supporting plane, not that
it meets the triangle. Classification therefore checks projected endpoints
first and then triangle edges, returning on a contact witness or first Unknown.
Only completed misses return false. The final report retains the Coplanar label
but not that additional contact boolean. Local reduction instead treats every
Coplanar edge as a touch. Source inspection therefore exposes a validation gap:
a classifier-produced Disjoint report with coplanar edges missing the opposite
triangle can be rejected by local validation. A targeted missing regression
candidate is left `(0,0,0),(4,0,0),(0,4,0)` and right
`(6,0,0),(7,0,0),(6,0,1)`: nondegenerate disjoint triangles on intersecting planes,
with disjoint edges along the planes' intersection. This candidate was derived
from the branch logic, not executed in this docs-only batch. A future behavior
fix should reproduce it and retain the needed contact evidence, not weaken
validation or label meaning.

Other validator limits are now explicit. A retained Degenerate label requires
only a matching final relation and bypasses all remaining fields. The coplanar
branch checks the nested label/report but ignores plane and edge fields.
Noncoplanar non-Disjoint reports need six labels; Disjoint is exempt. Plane
separation overrides even contradictory edge labels, and absent facts are
skipped, so a nondegenerate Disjoint report with no plane or edge facts passes.
No source incidence or certainty is verified. Source validation first runs
these local checks, then requires a decided field-equal replay while dropping
its receipt. It cannot restore qualification or establish unique source identity,
and it cannot bypass a local rejection caused by missing coplanar-edge facts.

Private aliases document the ordering and optional entries of their carriers;
the Unknown alias remains a full PredicateOutcome type by convention. Side
aggregation, edge pairing, outcome extraction, relation translation, strict
plane-separation recognition, and success packaging now state their roles and
preconditions. All eight fields and seven error variants are documented.
Test docs describe the finite controls: plane separation, crossing, boundary
contact, planar labels, degeneracy, six-edge disjointness, selected forged
reports, and symbolic uncertainty. They do not promote historical fixture names
to exhaustive path coverage or claim tests of the public approximate receipt
where the fixtures call internal helpers and only assert Unknown.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Non-comment, nonblank source
  remains byte-identical to baseline; signatures, predicate order, fixtures,
  assertions, and lint configuration are unchanged.
- Scoped declaration scan found 42 documented names. Checked 41 complete
  rendered doc groups across 23 private-module pages, including eight fields,
  seven variants, and the three private aliases. Test-only docs were reviewed
  in source; library Rustdoc does not render cfg(test) items.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed without warnings.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed. Existing fixtures compiled but were not executed.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed. Shared target remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-triangle-pair-{doc,check,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived by the owner for this
session. These source-level findings are not measured performance claims or
proof of completed geometry support. The repository-wide audit remains active.

## Hyperlimit retained-report and helper contract consumers

Baseline: `bba4fcc56`. Read all lines of three integration-test files:
`crates/hyperlimit/tests/retained_report_contracts.rs` (397),
`crates/hyperlimit/tests/classification_contracts.rs` (293), and
`crates/hyperlimit/tests/promoted_mesh_helpers.rs` (145). Earlier retained-report
excerpts are now superseded by whole-file review. Revisited ring and triangle/
plane validator implementations, public aliases, and the already reviewed
triangle-pair/coplanar report contracts for context; this does not add fresh
whole-file credit for those supporting reads.

Ownership is Hyperlimit's public classification/report consumer fixtures.
Relevant properties are exact input meaning, typed failures, policy/receipt
preservation, predicate truth tables, and source replay (P1/P3/P6/P8–P11/P19).
Domain: directly authored classification labels/report metadata, finite integer
and rational geometry, changed source data, and the optional std error-source
branch. Missing acceptance witnesses were module/declaration Rustdoc and precise
statements of what these tests actually assert. Several test names suggest
exhaustive validation or coverage, but their current finite tables and mutations
do not establish those broader claims. Only documentation changed; no assertion,
fixture, policy, feature gate, helper implementation, or predicate order changed.

The retained-report fixture docs distinguish accepted synthetic shapes from
geometric evidence. Ring cases cover a short Outside report, complete
nonstraddling metadata, partial boundary metadata, selected malformed counts,
indices, straddle facts and parity, and changed-query replay failure. They do
not establish index uniqueness/order or source truth for the synthetic labels.
Coplanar cases cover projectionless Unknown, one partial projected Unknown,
a complete synthetic Disjoint report, selected missing/inconsistent fields,
and an out-of-range replay index. They do not test every accepted Unknown shape
or successful coplanar source replay.

Triangle/plane controls assert five side-array reductions, successful borrowed
source replay, a changed-relation failure, then indexed failures before and
after an out-of-range index becomes a real point on the opposite side. This
fixture does not assert successful indexed replay; the promoted-helper fixture
does, for its uniform-side control. Triangle-pair controls start with crossing,
coplanar and degenerate reports, assert six crossing edge labels and selected
input-degeneracy labels, then reject specific mutations and replay against a
separated pair. They do not exhaust all publicly forgeable report fields. The
shared `decided` test helper extracts values and discards receipts; choosing
APPROXIMATE_512 alone does not establish terminal consumption.

Classification-helper docs distinguish enum truth tables from geometric
predicate tests. Coplanar segment/ray labels and conservative support-DOP overlap
are retained candidates, not newly proved intersections. Identical segment
labels satisfy the positive-length helper even though these label-only tests
cannot distinguish point-segments. The seven triangle-pair labels receive
explicit intersection/construction expectations. Four coplanar labels and three
synthetic ring-edge records exercise selected helper states, not every possible
metadata combination. Sign conversion/reversal, known-sign extraction, filtered
zero qualification, both error display prefixes, and the std-only absence of
nested source errors are documented separately.

Promoted mesh-helper docs state exact finite expectations: orthogonal x/y
support witnesses, converted x witness indices, one changed vertex and refreshed
x maximum six, source validation, and zero-axis rejection. Refresh scan counts,
complexity, all z ties, and certainty are not asserted. The authored XY plane's
positive-z queries yield Below under the repository's orientation convention;
its mixed-side control preserves On/Below/Above. Projected helpers check doubled
area 24, midpoint `(2,3/2,0)`, vector `(4,3)`, one inside-point label, a line
crossing at `(2,0,0)`, and an upper-before-lower counterclockwise turn. These
value assertions do not cover skew 3D sources or unresolved predicates.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves all three files byte-identical to the baseline. A scoped scan
  found all 24 declarations documented: nine retained-report names, seven
  classification tests, and eight promoted-helper names.
- `cargo check --locked -p hyperlimit --test retained_report_contracts --test
  classification_contracts --test promoted_mesh_helpers --all-features
  --offline -j2` passed. The same focused targets with `--no-default-features`
  passed, including compilation both with and without the std-only error-source
  assertions. Runtime tests were not executed.
- The initial direct `cargo rustdoc --test retained_report_contracts` lacked the
  owning hyperlimit import and failed with E0432. Supplying the exact checked
  all-feature hyperlimit rmeta via `--extern hyperlimit=...` and cached dependency
  directories fixed the inspection setup without repository configuration edits.
- Private Rustdoc for all three targets passed with all features, locked offline
  dependencies, `--document-private-items`, `-D warnings`, and both broken/private
  intra-doc-link lints denied. Render inspection initially caught an old module
  description retained by a fresh-cache Cargo result after a comment edit.
  Changing Rustdoc arguments to deny warnings forced new rendering; all final
  rendered prose matches the current source.
- Inspected thirteen complete rendered documentation groups: three crate
  introductions plus ten private helper/constant pages. The fourteen #[test]
  functions remain source-reviewed and compiled, not claimed as rendered pages.
  No library API implementation changed; these three test targets are the
  affected downstream consumers. Shared temporary build storage remains ~1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-report-tests-{doc,check,minimal-check,private}.log`;
checked artifact metadata is in the corresponding `artifacts.jsonl`. Full
workflow/runtime/performance checks remain waived by the owner for this docs-only
session. Coverage here is scoped evidence; the repository-wide audit is ongoing.

## Hyperlimit halfspace feasibility and Farkas replay

Baseline: `9207b87e9`. Read all 1,440 lines of
`crates/hyperlimit/src/predicates/halfspace.rs` and all 298 lines of
`crates/hyperlimit/tests/halfspace_feasibility.rs`, including private states,
helper arithmetic, every branch, and all fixtures. Revisited the composite
policy wrapper and acceptance rules; supporting excerpts add no whole-file
credit. Ownership is Hyperlimit's active-set feasibility and certificate replay.
Relevant properties are exact construction, explicit failure, policy/receipts,
decision order, and composable evidence (P1/P3/P6/P8–P11/P19). Inputs are finite
3D systems `n.x+d <= 0` over Real coordinates, including constant inequalities,
empty systems, singular plane combinations, rational data, and undecided symbolic
expressions. Missing witnesses were private/member/test Rustdoc and unqualified
claims that candidates/certificates and their replay were always exact.

Documentation now separates the mathematical motivation from the implemented
search. A closest-point active basis motivates origin, plane, line, and vertex
candidates in three dimensions, but search returns the first accepted candidate,
not necessarily a globally closest point. It visits singles, increasing-index
pairs and triples, replaying each candidate against planes in input order. A
violation advances search; an Unknown stops immediately, even if a later
candidate or constraint might suffice. Empty input accepts the origin with
Exact/Structural. Zero-normal planes remain constant inequalities. No shared
work/memory/cancellation contract or uniform practical completion bound is
provided by finite subset enumeration.

The public classifier uses strict-first replay for APPROXIMATE_512. Strict
completion preserves its outcome; an approximate replay's decided result is
conservatively Approximate/Refined. Candidate construction and certificate
helpers lose child receipts, so bare report data cannot recover the original
qualification. Feasible membership replay joins consumed certainty/stages;
certificate-result packaging attaches Exact/Exact and relies on the surrounding
public policy wrapper. A completed certificate search with NotFound still yields
Infeasible with no certificate, relying on prior active-set rejection. The
report does not retain all rejected-candidate evidence for standalone replay.

Farkas search visits constant, pair, triple and quadruple candidates in order,
stopping at the first accepted contradiction or Unknown. Pair nomination cancels
only the first coordinate on which both normals decide nonzero; full normal
and cone checks occur later. Triple nomination tries YZ/XZ/XY minors, checks
nonnegative orientation and full normal sum, but leaves offset positivity to
acceptance. Four-normal nomination uses alternating 3x3 minors and can produce
an all-zero vector at lower rank. Orientation scans all four signs, so a later
Unknown overrides already observed mixed signs. Successful sign receipts are
not retained. Dependency acceptance skips absent slots without itself requiring
their multipliers to be zero; internal callers supply that condition, whereas
public certificate replay checks it.

Report constructors only arrange fields; `is_feasible` reads the label alone.
Feasible validation checks the point against planes but ignores active indices
and any attached infeasibility certificate. Infeasible without a witness and
without a certificate returns true/Exact/Structural without inspecting planes.
The unchanged integration fixture explicitly expects true for such a report
against the feasible halfspace `x <= 0`. This is structural acceptance, not a
proof of infeasibility; docs no longer imply otherwise.

Direct certificate validation checks multiplier signs and selected-index bounds,
requires zero multipliers in absent slots and a positive multiplier somewhere,
recomputes weighted sums, compares the stored offset structurally, and then
checks zero normal and positive offset. Repeated indices are allowed. Structural
Real inequality can reject mathematically equal representations. Every decided
boolean is labeled Exact/Exact while child sign receipts are dropped; the method
has no strict-first wrapper. The source branches therefore do not justify
interpreting that label as independent exact certification under approximation.
The rational forged-component fixture does not cover this receipt gap. These
are documented implementation limits, not changes to the acceptance contract.

Projection helpers now specify `-d*n/(n.n)`, the 2x2 normal Gram solve, and
homogeneous affine recovery. DivideByZero skips a candidate, including a terminal
zero decision; other construction errors become Unsupported/Undecided. An
unresolved homogeneous reciprocal may fall back to direct affine conversion.
Successful reciprocal receipts are discarded and every constructed point still
needs system membership replay. Private determinant/vector/scalar helpers,
certainty joins, result enums, tuple payloads and Unknown fields are documented.

Test docs retain the finite scope of value/branch assertions rather than claiming
exhaustive search, cache reuse counts, or cost evidence. The nominal unit-box
fixture actually uses side length four. Rational controls assert origin/single/
pair/triple witnesses, selected active indices, one/two/four-plane certificates,
replay booleans and selected malformed components. Private tests cover deep
positive constructions, symbolic Unknown, dependency orientation and receipt
ranks. All original fixtures and assertions are unchanged; no runtime tests were
executed in this batch.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comments/blank lines
  and normalizing only the four expanded tuple-field layouts gives identical
  source to baseline. Tuple fields were expanded solely to attach field Rustdoc;
  no signature, branch, predicate ordering, fixture, assertion, or lint changed.
- Scoped scans found 53 documented owning-module declarations and sixteen
  integration-test declarations. Checked 71 complete rendered module doc groups
  across 35 pages, including nineteen named/tuple fields and twelve variants.
- Private all-feature Hyperlimit Rustdoc passed with locked offline dependencies,
  warnings denied, and broken/private intra-doc links denied. `cargo check
  --locked -p hyperlimit --lib --tests --all-features --offline -j2` passed,
  compiling affected local and integration consumers without executing them.
- Private Rustdoc for the halfspace integration target passed using the checked
  own-library metadata and cached dependency paths. The metadata selection
  initially encountered both library and test-harness artifacts; selecting the
  non-test library fixed the inspection script. All eight rendered module/helper/
  constant blocks match current source. Its nine test functions, and owning
  cfg(test) docs, remain source-reviewed rather than claimed as rendered pages.
- Downstream private csgrs Rustdoc with `--no-default-features --features curve`,
  locked offline dependencies, and broken links denied passed. Shared temporary
  target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-halfspace-{doc,check,test-private,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived for this documentation-only
session. Recording gaps does not repair them or establish universal geometry
completion. The repository-wide audit remains active.

## Hyperlimit axis-aligned box predicates

Baseline: `40ffaf5a2`. Read all 1,822 lines of
`crates/hyperlimit/src/predicates/aabb.rs`, including every public/private query,
the relation enum, rational-borrowing helpers, receipt bookkeeping, all eighteen
tests, and five nested test helpers. Revisited structural extent facts, interval
classification, and Real's borrowed-rational accessor. Those supporting excerpts
do not add whole-file review credit. Ownership is Hyperlimit's box predicates
and their broad-phase consumers. Relevant properties are exact meaning,
structural proof scope, policy/receipts, dispatch order and uncertainty
(P1/P3/P4/P8–P11/P19). Domain: borrowed Real coordinates for closed 2D/3D boxes,
reversed corners in general APIs, ordered corners in specialized APIs, collapsed
axes, rational/symbolic data and cached extent facts.

Acceptance witnesses were absent private/test Rustdoc, a module description
that named only 2D despite containing 3D queries, and claims that every bound/
interval decision was exact. Current branches preserve qualified approximate
comparisons, and the 2D extent path has a narrower zero-area recognition contract
than the previous positive-area wording implied. Documentation now describes
those implementation limits without changing the acceptance contract.

General point and intersection queries normalize endpoints per axis. Ordered
queries require the caller to supply ordered bounds and neither normalize nor
validate that precondition. Collapsed boxes remain sets rather than an empty-box
encoding. Closed point queries classify a point on any endpoint axis as Boundary;
relative interior uses open intervals on positive-width axes and equality on
collapsed axes. Thus a point box contains its point in relative interior. All
paths retain their stated axis order and stop at the first rejection or Unknown.
An unresolved earlier axis is not bypassed for a later possible separation.

Selected paths borrow structurally available rational coordinates for an
Exact/Exact result. Borrowing does not invoke general symbolic rational
reduction, clone or round coordinates, or force denominator normalization;
actual Rational comparisons can still have variable cost. Generic paths join
consumed certainty/stage and forward Unknown, without a strict-first composite
replay. Ordered overlap compares first maximum/second minimum before second
maximum/first minimum; containment compares corresponding minima before maxima.
The shared non-inlined scalar-comparison boundary is documented as a code-layout
choice, not a newly measured performance result.

Triangle bounding queries select x extrema and test x membership before y.
Extrema selection visits the second point then the third, each against current
minimum before maximum, retaining earlier references on ties. This tests the
box of the triple, including degenerate triples; it does not test point/triangle
incidence. Policy qualification applies to both bound selection and membership.
A box rejection is admissible for an enclosing object only with suitable bounds
and certainty; a box overlap does not establish contact of the bounded objects.

The 2D intersection wrapper eagerly computes both structural extent summaries
before testing intervals. Its facts-bearing entry point normalizes/classifies
x then y, stopping on separation/Unknown, then selects Touching for an interval
Touching label or either `known_zero_area == Some(true)`. Only that fact field
is consumed. Its truth and association with current corners are not validated;
stale or caller-forged facts are not certified by the returned child receipts.

The interval relation can be Overlapping for a contained singleton, with zero
extent separately recorded. The 2D path discards that flag and instead uses
structural facts; unknown zero-area facts take the same branch as false even
though unknown is not proof of positive extent. Therefore an Overlapping label
alone does not certify positive area when those facts are unavailable. This is
a source-level implementation gap, not a measured failing automatic-facts input
in this batch. A future acceptance witness should exercise policy-decided zero
extent with unavailable structural zero-area facts and compare the label to an
independent zero-area expectation. The 3D path retains the interval zero-extent
flags (or directly tests rational widths), so a contained flat box is Touching
with the producing receipt. No implementation or fixture was changed here.

Tests now document actual finite assertions. Rational controls cover location,
intersection, ordered containment, and planar relative interior. The symbolic
ordered-overlap fixture checks strict Unknown versus approximate true with an
Approximate receipt; other pi-translated tables mostly check values rather than
dispatch counts. The fixture named for cached extent facts inspects facts but
does not pass them to its point predicates. The borrowed-storage fixture has no
allocation/copy measurement. Unknown helpers inspect only the outcome variant,
and the receipt-rank fixture checks all finite rank pairs but not trace state
after an Unknown. Original fixtures, assertions and historical names remain.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only/blank
  lines gives identical source to baseline; no signatures, branches, predicate
  order, inline attributes, fixtures, assertions, or lint configuration changed.
- A scoped scan found 55 documented declarations, including the nested helpers.
  Inspected 26 complete rendered doc groups on 24 private-module pages, including
  both relation variants. All cfg(test) documentation was reviewed in source,
  not claimed as rendered by library Rustdoc.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline
  -j2` passed, compiling affected fixtures and consumers without execution.
- Downstream private csgrs Rustdoc with `--no-default-features --features curve`,
  locked offline dependencies, and broken intra-doc links denied passed. Shared
  temporary target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-aabb-{doc,check,downstream-doc}.log`.
Full workflow/runtime/performance checks remain waived for this docs-only
session. The documented limitations remain implementation work; the full
repository audit remains active.

## Hyperlimit determinant, reference-parity, and distance controls

Baseline: `8776c78a7`. Whole-file reviews:
`crates/hyperlimit/tests/nd_predicates.rs` (75 lines),
`crates/hyperlimit/tests/robust_parity.rs` (310),
`crates/hyperlimit/tests/exactcore_xinwei.rs` (51), and
`crates/hyperlimit/tests/predicate_filter_frontier.rs` (229). Revisited the
already reviewed dimension-generic owner and an excerpt of the point/triangle
distance implementation; the latter does not earn whole-file distance-module
credit. Ownership is Hyperlimit's determinant/reference/receipt acceptance
consumers. Relevant properties are exact input meaning, typed invalid/undecided
outcomes, certainty, dispatch order, and fresh-process evidence
(P1/P3/P6/P8–P11/P19). Domains are small rational simplices, finite represented
binary f64 coordinates, nine rational/radical/transcendental orientation triples,
and two rational distance fixtures.

The missing witnesses were module/helper/test Rustdoc, including functions
inside proptest and a nested process-guard constant, plus precise limits on
what the tests establish. All 36 named declarations now have documentation.
No input, expression, assertion, range, case count, feature gate, dispatch-count
expectation, environment guard, or subprocess argument changed.

Dimension-generic docs distinguish raw determinant conventions from general
geometric membership. The origin followed by the standard basis is positive in
3D/4D for orient_d; its 3D convention differs from orient3. The collinear case
asserts a complete false/Exact/Structural affine-independence outcome. The 2D
sphere cases assert raw Positive and Zero for one fixed simplex, not arbitrary
orientation-independent membership. Invalid arity is checked through imported
and qualified spellings of the same function, not independent implementations.
Most sign assertions discard the producing receipt.

Reference-parity docs identify robust as the expected-sign source,
geometry-predicates as the comparison across all four families, and apfp for
the planar families. Hyperlimit consumes exact represented binary f64 values;
decimal spellings and additions do not make ideal decimal coordinates. Fixed
controls cover selected boundaries, near boundaries, and one large orient2
case. Four property tests retain their 512-case configuration and bounded ranges
(orientation2 ±1e6, orientation3 ±1e4, incircle ±1e3, insphere ±100). Comparative
agreement is not a universal correctness certificate or exhaustive overflow/
underflow coverage. The reference-result sign helper maps NaN to Zero through
its fallback; it does not independently validate reference outputs. Coordinate
adapters document slice-length and finite-input assumptions where checked.

The rational distance fixture's vertices all satisfy `x+y+z=5/2`. Independent
Python Fraction arithmetic confirmed their centroid `(5/6,5/6,5/6)` and squared
norm `25/12`, strictly between two and three. The centroid lies in the triangle
and is parallel to the plane normal from the origin, explaining the closest
point. Actual Rust assertions compare against two and three rather than assert
25/12 directly. The other fixture checks zero distance to one collinear closed
segment hull. Replaced the unverified planner-history inline comment with the
explicit squared corner-radius comparison for boxes of unit half-width. These
are direct predicate tests, not planner execution or universal singular-case
coverage. No runtime Rust tests were run for this batch.

The frontier docs state the nine independent determinant expectations:
1, 1/9, 2^-100, sqrt(6), -1, e^2-pi^2, -2, e*2^-100, and zero. The table carries
case name, sign, stage, and dispatch path; every row expects Exact certainty,
even Refined-stage rows. Reconstruction occurs for each policy, without claiming
all shared scalar caches are cold. Translation/reversal checks signs and Exact
certainty but intentionally leaves stage unconstrained. The dispatch fixture
resets tracing after construction and checks selected path counts and absence
of terminal calls, not every internal event or elapsed time.

Fresh-process replay checks for presence of an environment guard, not a specific
value. With any guard value already set, it checks the current process and does
not spawn. Otherwise it starts the same test executable with the exact test-name
filter and guard set, inherits the remaining environment, and requires successful
exit. The corpus is reconstructed rather than serialized/reloaded. The nested
guard constant and all macro-contained property functions now have Rustdoc.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Non-comment/nonblank source is
  identical to baseline in all four files. Scoped counts are 8, 15, 5, and 8
  documented declarations respectively, including the four macro-contained
  property functions and the nested constant.
- `cargo check --locked -p hyperlimit --test nd_predicates --test robust_parity
  --test exactcore_xinwei --test predicate_filter_frontier --all-features
  --offline -j2` passed. The same focused targets with `--no-default-features`
  passed, covering compilation with and without the dispatch-trace fixture.
- Private Rustdoc for all four targets passed with all features, locked offline
  dependencies, checked own-library metadata and cached dependency paths,
  `--document-private-items`, warnings denied, and broken/private links denied.
  Inspected 24 complete rendered module/helper/constant doc groups. Fifteen test
  functions and the nested guard constant remain source-reviewed/compiled;
  ordinary Rustdoc does not expose them as standalone pages.
- The exact Fraction calculation verifies fixture arithmetic only, not the Rust
  predicate implementation. No random, subprocess, dispatch-count, or geometry
  test was executed. These test targets are the affected downstream consumers;
  no library API code changed. Shared target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-predicate-controls-{check,minimal-check,private}.log`;
checked artifact metadata is in the corresponding `artifacts.jsonl`. Full
workflow/runtime/performance checks remain waived by the owner. The complete
repository audit remains active; this is scoped coverage evidence only.

## Hyperlimit symbolic predicate fixtures and quickstart

Baseline: `fa953fd57` on `rustdoc`. Read every line of
`crates/hyperlimit/tests/cyclotomic_zero_predicates.rs` (120 baseline lines),
`extended_normal_form_predicates.rs` (87), `lazy_scalar_certificates.rs` (33),
`repeated_symbolic_products.rs` (141), `readme.rs` (41), and
`crates/hyperlimit/examples/readme_quickstart.rs` (11). Their owning layer is
Hyperlimit's predicate qualification corpus and package documentation. Relevant
contracts are P1/P3/P4 and P8–P11/P19: exact scalar meaning, reusable construction
identity, policy-sensitive decisions, and explicit certainty. The missing
acceptance witness was documentation for private helpers, constants, tests and
example entrypoint, plus misleading explanations of existing assertions.
Implementation and test assertions remain unchanged.

Added or reviewed Rustdoc for all 22 named declarations in these six files
(counts 6, 5, 1, 5, 4, 1). Sphere samples retain rational-angle trigonometric
coordinates; crossing uses a supporting line and does not assert its parameter
lies in `[0, 1]`. The translated fixture uses exact binary f64 values. Zero and
On checks require Exact certainty, while adjacent-quad nonzero controls inspect
values only. These fixtures isolate mesh-consumer predicates, without running
hulls or Booleans.

The degree-ten fixture asserts ordinary normal-form decline and subsequent
exact strict results, without asserting the successful stage. Replaced the
incorrect description of `2^-60` offsets as below refinement precision. Its
angle-addition control checks strict Unknown and an approximate decided receipt,
not the approximate value. The retained-product control does assert approximate
Zero, with strict Unknown before and after. Retained replay here means reuse in
one process, with no cache-count or serialization assertion. Documented the
unit chord normal and independent normal/determinant formulas used by the
symbolic geometry fixtures.

The serde fixture removes the `computable` member if present, without asserting
that removal found one. It reloads pi, sqrt(2), and ln(3), then checks strict
scalar/orientation values; it does not separately inspect certainty or prove
whole-value equivalence through a fresh process. Read supporting excerpts of
Hyperreal's `serde.rs` (through line 75), arithmetic facts and relation-normal-form
code, plus previously reviewed Hyperlimit resolution code; these excerpts do
not establish full review of the Hyperreal files.

Read the complete Hyperlimit README for context, but its broader claims about
coverage, bounds, and replay still require semantic review. This batch changes
only its marked quickstart block to match the newly documented runnable example.
The README tests check source equality after outer whitespace trimming, package
version presence, and eight heading substrings, not all documentation semantics.
The example asserts positive orientation and prints the full outcome; its
assertion itself extracts only the value.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. All six Rust files are identical
  to baseline after removing comment-only and blank lines. The README change
  consists solely of synchronized example comments.
- `cargo check --locked -p hyperlimit --test cyclotomic_zero_predicates --test
  extended_normal_form_predicates --test lazy_scalar_certificates --test
  repeated_symbolic_products --test readme --example readme_quickstart
  --all-features --offline -j2` passed. The same target list with
  `--no-default-features` passed; the serde fixture is excluded in that mode.
- Private Rustdoc passed for all five test targets and the example with all
  features, checked own-library metadata and cached dependency search paths,
  `--document-private-items`, warnings denied, and broken/private links denied.
  Verified 18 complete rendered module/helper/constant/main doc groups against
  source. Ten test functions remain source-reviewed and compiled rather than
  exposed on standalone Rustdoc pages.
- Independently checked the README/source text equality, manifest version
  substring, and all eight heading substrings in Python. These are static
  checks, not Rust test execution. No runtime scalar, geometry, or example tests
  ran. The focused targets are the affected downstream consumers; no library
  API or implementation changed. Shared target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-symbolic-{check,minimal-check,private}.log`;
checked artifact metadata is in `symbolic-artifacts.jsonl` under the same prefix.
The owner's full-check waiver applies. The future-agent public/private 100%
Rustdoc mandate remains in root AGENTS.md. Overall repository coverage and the
complete-file semantic audit remain unfinished.

## Hyperlimit adversarial predicate and property corpus

Baseline: `bd177a4c0` on `rustdoc`. Read every line of
`crates/hyperlimit/tests/adversarial_degeneracies.rs` (347 baseline lines),
`adversarial_predicates.rs` (587), and `adversarial_props.rs` (496). The owning
layer is Hyperlimit's qualification corpus, with affected scalar imports,
predicate outcomes, retained structural facts, homogeneous constructions,
intersections, and distance/classification consumers. Relevant properties are
P1/P3/P4 and P8–P11/P19: exact inputs/construction, policy scope, certainty,
retained evidence, and explicit uncertainty. Missing acceptance witnesses were
Rustdoc for all private declarations and descriptions of assertion scope.

Added documentation for all 75 declarations: 20, 23, and 32 respectively,
including 19 macro-contained property functions and the nested `evaluated_order`
helper. The 43 test functions and nested helper are source-documented even though
ordinary Rustdoc omits their standalone pages. Reviewed the previously audited
`predicate.rs` outcome definition again to confirm that derived equality compares
complete outcomes, including certainty/stage or uncertainty/stage, for the two
batch/scalar fixtures. Other predicate checks generally extract values.

The tests select APPROXIMATE_512, including the one historically named
`strict_predicates_decide_scalar_signs_and_ordering`; they do not independently
assert Exact receipts. Repeated queries do not assert cache behavior. The
unknown-zero helper requires structural uncertainty for `sin(1)^2+cos(1)^2-1`,
which does not imply policy-level undecidability. The 17-value scalar comparison
corpus skips pairs whose precision -256 enclosure fails or contains zero;
it is not an independent scalar kernel or exhaustive comparison-completion
witness. Structural facts and advisory scheduling enum tests do not measure
actual dispatch, storage allocation, leaks, or performance.

Documented input conversion and specific mathematical scope: finite floats are
imported exactly after any float arithmetic in fixture construction. The batch
row containing `1e150+1` therefore collapses to identical points before import.
The subnormal orientation determinant is exactly 2^-2148 although primitive
binary64 multiplication underflows. Unit-circle perturbations are formed in
f64; the sphere portion checks only its center. Three-point plane orientation
has the opposite sign to the chosen coefficient normal. The Pappus fixture
checks explicit rational intersections, and the pentagon seed is an authored
rational polygon, not an exact regular pentagon. Its five-round construction
replay is in-process geometry, not serialization.

The property corpus requests 256 cases per property over bounded integers,
including exact integer-valued f64 adapters. Conditional assertions may accept
unknown outcomes; comparisons of optional values may accept two unknowns.
Documented each such boundary rather than changing assertions. The vertical
segment/ray domain `[0,20]^2` always satisfies x+y <= 40, so it covers hits only.
The homogeneous coordinate-plane fixture checks affine conversion before
homogeneous incidence; corrected the inline comment that claimed the reverse
order. Also corrected the segment symmetry comment to acknowledge optional
unknown results. No fixture, helper implementation, assertion, policy, or
predicate order changed.

Independent arithmetic review used Python Fraction to solve the Pappus line
pairs, obtain `(8/5,2/5)`, `(35/11,5/11)`, `(9/2,1/2)`, and check their zero
determinant. A separate rational line-intersection calculation verified all
five inner/outer rounds on the exact affine pentagon coordinates. Binary64
checks confirmed `1e150+1 == 1e150`, the smallest positive float's exact value
2^-1074, and multiplication underflow; Fraction verified its square 2^-2148.
Enumerating all 441 vertical-hit x/y pairs confirmed the stated domain limit.
These calculations validate fixture arithmetic, not the Rust implementation.

Validation after the final source edit:

- Focused Rustfmt and `git diff --check` passed. All three files are identical
  to baseline after removing only comment-only and blank lines. A scoped scan
  confirmed Rustdoc on all 75 named declarations, including macro and nested
  functions.
- `cargo check --locked -p hyperlimit --test adversarial_degeneracies --test
  adversarial_predicates --test adversarial_props --all-features --offline -j2`
  passed. The same targets with `--no-default-features` also passed.
- Private Rustdoc for all three targets passed with all features, checked
  own-library metadata and cached dependency paths, `--document-private-items`,
  warnings denied, and broken/private links denied. Verified 34 complete
  rendered module/helper/constant/type-alias doc groups against source. The 43
  test functions and one nested helper remain source-reviewed and compiled.
- No runtime Rust tests, random property executions, or performance checks ran.
  These test targets are the affected downstream consumers. Shared target
  storage remains about 1.6 GiB; no additional target directory was created.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-adversarial-{check,minimal-check,private}.log`;
checked artifact metadata uses the same prefix and `artifacts.jsonl` suffix.
The independent arithmetic script is `/tmp/csgrs-adversarial-arithmetic.py`.
Full workflow checks remain waived by the owner. The complete repository audit
remains active; the existing 100% public/private mandate in AGENTS.md is retained.

## Hyperlimit support-DOP and scalar-representation consumers

Baseline: `d5e176944` on `rustdoc`. Read every line of
`crates/hyperlimit/tests/support_dop.rs` (775 baseline lines) and
`crates/hyperlimit/tests/real_representations.rs` (1280). The owning layer is
Hyperlimit's support-bound/report and scalar-representation qualification corpus.
Relevant contracts are P1/P3/P4 and P8–P11/P19: exact imported/construction data,
retained source identities, policy/receipt boundaries, and replay. Missing
acceptance witnesses were documentation for all private items and accurate
limits on the claims established by the existing tests.

Documented all 52 named declarations (24 support-DOP and 28 representation),
including two nested tag-inventory constants, plus all three RepresentationCase
fields. The support fixtures test exact integer/rational projections, extrema
indices, boundary classifications, coarse directional box relations, closed
halfspace side reports, selected forgery failures, and refresh counters. Queries
select APPROXIMATE_512 and extract values without certainty assertions. Tests
with names claiming every forgery cover only their listed finite mutations;
no test was broadened or weakened. Axis reversal compares optional values for
four samples. Refresh counters do not establish time or allocation cost.

Read supporting DOP validator/replay excerpts at baseline lines 557–690 and
739–845; these are not full review of `predicates/dop.rs`, which remains pending.
Source replay compares the newly produced report value after local validation
and discards the new outcome receipt. This batch documents test expectations;
it does not certify universal validator soundness. Previously recorded limits
of halfspace feasibility replay still apply to its consumers.

The scalar corpus has 22 named-class recipes, eight public structural kinds,
four rational storage classes, and five primitive-range status categories.
Its common helper exercises seven predicates whose shared translation carrier
cancels mathematically, including a collapsed triangle distance check. Therefore
successful consumption after reload does not independently establish equality
of original/restored scalar values. JSON/CBOR tests check selected tags and
kinds; they do not assert cache absence, deterministic bytes, fresh-process
replay, or general value preservation. The 70 node and 18 constant inventories
are checked against serde diagnostic wording at runtime; this batch reviewed
the assertions but did not execute them or claim current runtime success.

Documented private cache probes as Debug substring checks, the exact
HYPERLIMIT_EXPECT_F32_CACHE present/absent/unavailable behavior, neighboring-float
bounds, unconditional F64 markers, and the limits of abort-signal checks. The
large overflow views use clones; their subsequent translation fixture consumes
the original huge scalar. Triggered signals preserving known positive signs and
one opaque Unknown do not prove general cancellation cleanup, retry, or resource
bounds. The unit-circle opaque zero accepts Exact or Filtered certainty; the
angle-addition approximate Zero assertion extracts only its value.

Read Hyperreal's Carlson approximation excerpt through line 115 and the complete
small `computable/node/elliptic_integrals.rs` constructor for supporting context.
These excerpts explain why the root-node corpus's `(1/8,0,0)` internal tuple
encodes mathematical Carlson arguments `(1/64,1,1)`: the first child is squared
and the other two exponentiated. It is not a raw singular Carlson input. The
constructor itself has not yet received its own Rustdoc audit/coverage edits;
no full approximation-file review is claimed. Also checked the `Computable::approx`
signature and previously reviewed PredicateOutcome value extraction. Node reload
assertions compare approximation integers at precision -24, not an independent
exact-value oracle or arbitrary expression-composition coverage.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Both files are identical to
  baseline after removing only comment-only and blank lines. Scoped scans
  verified Rustdoc on all 52 declarations and three fields, including nested
  constants and serde-gated helpers.
- `cargo check --locked -p hyperlimit --test support_dop --test
  real_representations --all-features --offline -j2` passed. The same targets
  with `--no-default-features` passed; serde-gated tests/helpers are excluded
  in that configuration.
- Private Rustdoc passed for both targets with all features, checked own-library
  metadata and cached dependency paths, `--document-private-items`, warnings
  denied, and broken/private links denied. Verified 31 complete rendered
  module/helper/constant/struct/field doc groups against source. Twenty-four test
  functions and two nested constants remain source-reviewed and compiled, not
  standalone Rustdoc pages. Private Rustdoc was not run without default features.
- Independent Python integer/Fraction checks verified the first support fixture's
  min/max witness indices and x+y values, and strict separation of 1/2 from
  1/2+2^-1074. These validate fixture arithmetic, not Rust predicate execution.
  No runtime tests, cache-environment matrix, serialization replay, or abort
  executions ran in this batch. These test targets are the affected downstream
  consumers; no library implementation changed. Shared target remains 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-support-representations-{check,minimal-check,private}.log`;
checked artifact metadata uses the same prefix and `artifacts.jsonl` suffix.
Full workflow checks remain waived by the owner. The repository-wide semantic
review and coverage mandate remain active and incomplete.

## Hyperlimit directional support implementation

Baseline: `f5a4d040a` on `rustdoc`. Read every line of
`crates/hyperlimit/src/predicates/dop.rs` (2203 baseline lines), including all
inline tests. Revisited the preceding batch's full support-DOP integration-test
review and current halfspace validation delegation. Owning layer: Hyperlimit
support projections, source witnesses, and report validation. Relevant
properties: P1/P3/P4 and P8–P11/P19, especially exact stored expressions versus
policy-selected extrema, inherited qualification, mutation, and receipt loss.
Missing acceptance witnesses were private-item documentation and overbroad
public statements about exactness, replay, and bounded refresh work.

Reviewed/documented all 88 named declarations, 43 fields, and 44 enum variants.
The source is unchanged apart from comments and blank lines. The 12 declarations
inside the cfg(test) section include its module, policy constant, three helpers,
and seven tests; their Rustdoc is present in source but ordinary library docs
exclude that module. Existing assertion and predicate order are preserved.

Expansion metadata is now described as caller-declared unnormalized projection
margins. Integer axes are not normalized; changing axis scale changes the units
of both bounds and margins. A LossyAdapter tag supplies zero numeric margin and
does not itself prove a loss bound. Exact finite binary-float import alone does
not imply geometric loss. Expansion validation checks sign and kind/counts,
accepts decided approximate comparisons, drops receipts, and maps unresolved
nonnegativity to NegativeExpansion. The direct conservative-bound accessors
apply the scalar regardless of declared kind/count, without validation.

Witnessed construction scans extrema then fully validates every point per axis.
Its candidate helper ignores unknown comparisons during selection; final
validation can reject them. Witness point identity uses Point3 PartialEq;
projection equality is policy-driven, with unresolved equality mapped to
WitnessDistanceMismatch. Result-returning APIs discard certainty/stage even
when a comparison decided approximately. Raw conversions omit point copies and
expansion metadata, apply no margin, and reset the general carrier's source count
to zero. This is not certified transport of source fidelity or expansion bounds.

Refresh checks counts/indices, mutates slabs in order, then validates the complete
point set. An invalidated extremum rebuilds an axis; other listed points can
extend it. Duplicate changed indices are counted/revisited. There is no rollback
on later error and final validation still scans all points for every slab. This
supplies source-based evidence for the documented work structure, not a runtime
performance claim or a changed-subset work bound. Expansion consistency can fail
after mutation because its full validation occurs at the end.

General carriers accept zero axes, validate only visited slab bound orders, and
short-circuit on Outside, separation, or inverted bounds before later slabs.
Empty slabs classify as Degenerate rather than a universal-set identity.
Projection handles reversed AABB corner coordinates by selecting coordinate
product extrema. BoundaryTouch/nonseparation is an interval test, not a common
spatial witness. General outcomes accumulate current-query receipts but the
stored carrier/report does not preserve constructor qualification. A zero-axis
slab with bounds zero/zero yields constraint contact everywhere; source inspection
therefore does not support calling that label the intrinsic topological boundary.
This consequence was traced through the current code, not run as a new test.

Local AABB validation checks stored interval consistency and prefix scheduling,
not projection derivation or source identity. Source replay first validates,
then compares fresh report values while discarding fresh certainty/stage. Plane
reports lower slabs to closed halfspaces and check carrier, below, above in
that order; tangency can make both sides feasible. Empty/invalid/infeasible
carriers use Degenerate. Neither axes nor plane normals are separately required
to be nonzero at this layer.

Plane validation inherits the current HalfspaceFeasibilityReport behavior:
Infeasible with no witness/certificate is accepted as structurally valid. A
hand-constructed nonempty plane report with two valid slab halfspaces, such an
infeasible carrier, Degenerate relation, and no sides can therefore pass local
validation without proving infeasibility. The previously audited halfspace
implementation and current DOP delegate were inspected again; this is a
source-derived limitation, not a newly executed regression. Source replay is a
separate recomputation and does not turn a missing standalone certificate into
one. Local validation also accepts the empty-halfspace Degenerate sentinel
without proving an invalid source slab. The public slab-count multiplication
uses ordinary usize arithmetic and can overflow for forged counts; documented
that overflow-checking builds can panic. No implementation changes were made.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Non-comment/nonblank source is
  identical to baseline. Scoped scans found Rustdoc on every named declaration,
  field, and variant in this file.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed. This includes the inline controls and existing downstream integration
  tests without executing them.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed. Verified 164 complete rendered doc
  groups across 44 pages, including all 43 fields and 44 variants.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed.
- No runtime, mutation/rollback, forgery, performance, or resource-exhaustion
  tests ran. The source-derived limitations above remain implementation facts
  to address in a separately authorized code change, not conformance successes.
  Shared target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-dop-{doc,check,downstream-doc}.log`.
The owner's full-workflow waiver remains in effect. The complete repository
semantic/coverage audit is still unfinished; existing global instructions remain.

## Hyperlimit closed-segment predicates and line construction

Baseline: `f0aa9eef4` on `rustdoc`. Read every line of
`crates/hyperlimit/src/predicates/segment.rs` (2461 baseline lines), including
all inline tests. Owning layer: Hyperlimit segment incidence, intersection
classification, and supporting-line construction. Relevant properties are
P1/P3/P4 and P8–P11/P19: exact input expressions, retained source associations,
policy receipts, collinearity preconditions, and downstream construction use.
Missing acceptance witnesses were private-item Rustdoc and accurate descriptions
of trust boundaries, receipt accumulation, and Option failure semantics.

Reviewed/documented all 83 declarations and five private fields. The 31 cfg(test)
declarations are the module, policy constant, five helpers, and 24 test functions;
the remaining 52 declarations render in private library docs. All code, fixtures,
assertions, dispatch calls, and predicate order remain identical after removing
comment-only and blank lines. The whole file received semantic review, not just
a declaration scan.

Planar classification evaluates orientation then inclusive coordinate intervals
and endpoint equality. Spatial classification checks collapse first, then all
three cross-product signs before deciding off-line status. An earlier nonzero
component does not short-circuit later unresolved components. The between-closed
helper similarly evaluates both scalar comparisons even if the first is zero.
Documented all helper assumptions and order, including no collinearity check in
the collinear entrypoint and no zero-denominator rejection in the unit-parameter
helper. The vector accessor asserts axis < 3 in debug builds but otherwise
maps any index other than zero/one to z.

Retained line/query associations are caller preconditions; stale compiled filters
can decide before current coordinates are checked. Revisited the already audited
orientation owner at baseline lines 650–754 to confirm this path. Known-sign
entrypoints check only policy acceptance of certainty, trusting sign, stage, and
current source association. Facts marking collapse bypass endpoint revalidation.
Revisited the Segment2Facts accessor to confirm it returns the retained snapshot.
Neither kind of trusted evidence can silently be described as independently
verified by these wrappers. The internal collinear helper starts a fresh trace;
callers must transport any inherited collinearity qualification themselves.

The spatial intersection classifier's exactly-one-collapsed-operand branches
return point-classifier outcomes directly, without merging the trace from the
two preceding endpoint-pair equality checks. Source inspection reproduces this
receipt gap on the current revision; no runtime approximate-collapse witness
was added or executed in this docs-only session. Other classification branches
accumulate consumed receipts and propagate the encountered Unknown. Collinear
relations count unique shared endpoints after checking identity in either order;
proper/endpoint/skew relations return labels without constructing geometry.

Supporting-line construction tries eligible small/wide dyadic and fused rational
paths before general Real expressions. In the general path it drops the
policy denominator comparison receipt, returns None for decided zero, uses the
assuming-nonzero inverse hook for decided nonzero, and still tries ordinary
inversion on Unknown. Thus Unknown does not itself mandate failure. Any inverse
error maps to None; failure does not distinguish parallel/coincident/degenerate
geometry from scalar failure. A successful Option carries no comparison receipt
or explicit source-incidence record, and no closed-segment parameter bound is
checked. Revisited Hyperreal inversion excerpts at 570–595 and 807–843, including
the external nonzero hook; these excerpts are not a full inversion-file audit.
Documented receipt loss under approximate policy without changing the API.

The tests now distinguish complete retained/direct outcome comparisons from
value-only checks and names that overstate scope. The purported skew planar
control is actually two disjoint 2D arrangements; wide/nondyadic constructor
fixtures check outputs without asserting successful dispatch paths; the endpoint
collection control covers three arrangements rather than every permutation.
Replaced the claim that no proof decides a fixture zero with the observed
expectation that the current strict query remains unresolved. Existing hard
fixtures and all assertions are preserved.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Non-comment/nonblank source
  equals baseline; scoped scans confirm Rustdoc on all 83 declarations and five
  fields, including tests and helpers omitted from ordinary library docs.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling inline and integration consumers without executing them.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed. Verified 58 complete rendered doc
  groups across 53 pages, including all five fields.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed.
- Independent Python Fraction line-parameter calculations verified four fixture
  expectations: `(2,2)`, supporting-line intersection `(2,0)` outside the first
  segment, `(1/3,1/3)`, and the midpoint with extent `(2^63-1)^2`. These establish
  fixture arithmetic only, not Rust algorithm execution or dispatch selection.
- No runtime tests, approximate-collapse repro, cache/dispatch measurements, or
  performance tests ran. Shared target storage remains about 1.6 GiB.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-segment-{doc,check,downstream-doc}.log`.
Full workflow checks remain waived by the owner. The repository-wide audit is
still active and incomplete; documentation of implementation gaps does not
establish conformance to the target contract.

## Hyperlimit triangle containment and retained reports

Audited every line of `crates/hyperlimit/src/predicates/triangle.rs` (3,044 lines
at baseline `61c931768`). Owning layer: Hyperlimit orientation, incidence, retained
evidence, and policy-controlled classification. Relevant contracts: P4, P6,
P8–P10, and preservation of qualification for downstream consumers. The domain
is planar/spatial Real-coordinate triangles, points, closed segments, forward
rays, tetrahedra, and retained normal/report records under both policies.
Degenerate inputs, zero directions, coplanar support relations, mutable report
fields, stale source associations, and unresolved scalar signs all matter here.
The missing acceptance witness was complete accurate private/public Rustdoc and
rendered coverage; this docs-only change does not repair implementation gaps or
claim new geometric support.

Documented all 99 declarations, 23 named/tuple fields, and 13 error variants.
The 34 test-only declarations are the module, policy constant, four helpers, and
28 test functions. The other 65 declarations render in private library docs.
Only comments and the layout of one enum tuple payload changed; all executable
source, fixtures, assertions, predicate order, and dispatch calls are preserved.
The tuple field now has its own Rustdoc without changing its type or variant.
Confirmed the existing root AGENTS.md public/private 100% coverage mandate.

Reproduced the following distinctions by reading current implementation paths:

- Retained planar orientation accepts Unknown unchanged, as does spatial normal
  evidence. A different policy therefore does not itself retry cached Unknown.
  Disallowed decided certainty is recomputed; spatial recomputation uses the
  stored normal, not current vertices. Neither form checks source identity.
  Known-degenerate planar facts similarly bypass vertex checks and supply an
  Exact/Structural zero orientation. Documented these caller preconditions and
  the unresolved retry limitation without changing them.
- Spatial point containment runs all three edge-halfspace calls before reducing
  their results; tetrahedron containment similarly evaluates four replacement
  determinants. Later uncertainty can prevent a definite Outside result.
  Planar edge calls are sequential until Unknown, but do not short-circuit on
  the first outside sign. Documentation preserves these distinct schedules.
- Both public report builders use the strict-first composite-policy wrapper.
  Internal attempts discard child certainty/stages and label success Exact/Exact;
  an approximate replay is conservatively marked Approximate/Refined by the
  wrapper. Revisited the previously audited resolver at lines 248–280 to confirm
  that retry applies specifically to APPROXIMATE_512 and only after Unknown.
  Bare reports carry no aggregate receipt; validators discard successful check
  and replay receipts as well.
- The coarse segment classifier is not a projection of the report classifier.
  A transverse coarse path requires successful homogeneous-to-affine conversion,
  discards that point, then uses input line/edge triple products. The report path
  classifies its constructed candidate. The preclassified-side helper trusts its
  side labels and retained plane and starts Exact/Exact without a receipt input.
- Coplanar is a support-plane relation, not finite triangle overlap. Segment/ray
  paths do not reject triangle degeneracy before those support tests. A zero ray
  direction is classified through the parallel-plane branch, including Coplanar
  for an on-plane origin. A transverse ray outside the finite triangle still
  retains its candidate and parameter in a Disjoint report.
- Ray vertex contacts reuse the source vertex and a dot-product parameter while
  retaining the original plane ratio. Local validation must compare those
  parameter representations. Reciprocal hooks are followed by `expect`; their
  panic conditions are now explicit. No new runtime failure was asserted or
  manufactured from the mere presence of those invariant checks.
- Local report validation checks stored consistency, not candidate containment
  or source fidelity. In particular, ray Disjoint-without-candidate accepts its
  side/sign labels without checking them, and the ratio branch does not reject
  missing side/sign labels. Ray validation does not establish the point's ray
  equation or the ratio's source derivation. Unknown parameter checks map to
  validation errors; source replay maps Unknown to SourceReplayMismatch and
  compares report values without preserving the replay receipt. These are
  source-confirmed limitations, not completed proof obligations.

Test Rustdoc describes the actual finite assertions. The location-collapse loop
asserts permitted result sets rather than each individual expected mapping.
The corruption and optional-shape controls are not exhaustive despite their
names. Cached-sign and nonzero-direction controls do not assert dispatch counts;
most geometry tests inspect values without asserting the outer certainty/stage.
The original unit-circle zero normalization remains asserted separately from
the shared terminally unresolved control. No assertion or hard fixture changed.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Source identity comparison strips
  only comment-only/blank lines and normalizes only the documented
  `PlaneEventInvalid(SegmentPlaneValidationError)` tuple layout. The result is
  identical to baseline. Scoped scans confirm all item, field, and variant docs.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling library, inline tests, and integration consumers.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed. Verified all 102 complete rendered
  doc groups across 58 pages, including 23 fields and 13 variants. Test-only
  declarations were source-reviewed and compiled, not rendered in library docs.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed.
- Independent Python Fraction arithmetic verifies the finite ray fixture's
  t = 2, point (1,1,0), and strictly positive barycentric weights (1/2,1/4,1/4).
  This verifies fixture mathematics, not execution of the Rust implementation.
- No runtime tests, dispatch/cache measurements, or performance tests ran.
  Shared target storage remains approximately 1.6 GiB; incremental compilation
  remains disabled. Full workflow validation is waived by the owner this session.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-triangle-{doc,check,downstream-doc}.log`.
The repository-wide audit remains incomplete. The next remaining whole
predicate implementation in Hyperlimit is `distance.rs`; the existing tests and
other completed predicate records should not be re-counted as new coverage.

## Hyperlimit squared distances and circle/ball contacts

Audited every line of `crates/hyperlimit/src/predicates/distance.rs` (3,103 lines
at baseline `a9227a641`). Ownership is Hyperlimit's scalar comparisons, closest
feature selection, and contact labels. Relevant contracts are P5, P6, P8–P10,
and downstream qualification preservation. Inputs include Real-coordinate points,
lines, closed segments/triangles, plane equations, boxes, circles, and closed
balls under both predicate policies. Scalar distance thresholds may be negative;
geometric squared radii require caller validation at most entrypoints. The
missing acceptance witness was accurate complete public/private Rustdoc with
rendered coverage. This documentation-only batch changes no algorithms, branch
order, tests, fixtures, policies, or dispatch assertions.

All 85 declarations and five private vector fields now have Rustdoc. Of those,
36 declarations are test-only: the module, policy constant, eight helpers, and
26 test functions. The remaining 49 declarations render in private library docs.
Source after removing comment-only and blank lines is identical to baseline;
no formatting normalization of executable code was needed.

The review distinguishes these current behaviors from stronger target claims:

- Rational kernels use borrowed `exact_rational_ref` data, not arbitrary exact
  rational normal forms. Point-distance ordering and triangle distance try them
  unconditionally; several other entrypoints gate them on a nondyadic rational
  threshold/radius. The helpers themselves also accept dyadic data. None means
  the needed representation was unavailable, not a geometric decision. Rational
  successes attach Exact/Exact; generic paths follow the requested policy.
- Circle-line, point-line, point-plane, planar/spatial point-segment, and
  circle-segment paths discard some successful norm/projection guard receipts.
  Sphere/sphere classification discards successful radius-validation receipts.
  Final outcomes may therefore omit approximate branch qualification. These
  are source-confirmed omissions; no runtime witness for the omitted-receipt
  cases was added or executed in this docs-only session. Documentation does not
  repair them or establish P9 conformance.
- Triangle distance aggregates its own normal and decided barycentric checks,
  but cannot recover qualification discarded inside edge helpers. It evaluates
  all three projection numerators; one negative suffices to select edge distance
  even if another is unresolved. With no negative, unresolved projection remains
  Unknown. The edge-minimum helper may return a later Less with only that edge's
  receipt, because that single comparison suffices to show the minimum is Less.
  If no edge is Less, any encountered Unknown prevents Equal/Greater completion.
  This sufficient-witness behavior is distinct from missing branch receipts.
- Zero-normal planes compare offset squared with the threshold. For example,
  offset 5 yields the scalar equality 25 = 25 even though the equation has an
  empty zero set. The fallback is not a geometric distance or typed invalid-plane
  result. Collapsed lines/segments instead use point distance, and degenerate
  triangles use closed-edge minima. The triangle path alone explicitly maps a
  decided negative squared normal to Unsupported at the decision's stage.
- Circle/segment Tangent and Secant follow the endpoint partition, not a
  differential-tangency or intersection-count guarantee. Revisited the already
  audited relation definitions at `classify.rs` lines 155–205 and 294–339; those
  definitions already state these conventions and closed-ball semantics. The
  producer docs now agree. Sphere/sphere tests concern balls including interiors,
  not surface intersection. A negative radius becomes Unknown/Unsupported rather
  than a distinct invalid-input result; other squared-radius APIs do not check
  nonnegativity.
- AABB/ball classification sorts coordinate endpoints under policy, accepts
  degenerate boxes, and preserves all consumed axis and final-comparison receipts.
  A zero-radius ball anywhere in the box receives Touching. Overlapping does
  not prove positive volume for a degenerate box. All routines avoid square-root
  construction but provide no separate total-work/coefficient-growth budget.

The AABB approximate-axis test creates fresh inputs for each STRICT/APPROX/STRICT
iteration, so its sequence is not a retained-object cache-transition witness.
Its feature-gated dispatch checks remain unchanged: ten comparisons on completed
approximate cases and the specified strict prefix at unresolved axes. The
rational AABB grid contains 216 cases with independent integer expectations and
complete Exact/Exact assertions. Most other tests check values only. Docs no
longer imply exhaustive partitions, asserted dispatch selection, or absence of
square-root calls based on those value-only tests. The exact-normal identity
control and separate terminally unresolved expression remain intact.

Validation after final source edits:

- Focused Rustfmt, `git diff --check`, scoped declaration/field documentation scan,
  and comment-stripped source identity comparison passed.
- `cargo check --locked -p hyperlimit --lib --tests --all-features --offline -j2`
  passed, compiling the library, inline tests, and integration consumers.
- `cargo rustdoc --locked -p hyperlimit --lib --all-features --offline -j2 --
  --document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links` passed. Verified 55 complete rendered doc
  groups across 50 pages, including all five private fields. The 36 test-only
  declarations were reviewed and compiled, not rendered by library Rustdoc.
- Downstream `cargo rustdoc --locked -p csgrs --lib --no-default-features
  --features curve --offline -j2 -- --document-private-items
  -D rustdoc::broken_intra_doc_links` passed.
- Independent integer/Fraction arithmetic checked the 25, 9, and 1/9 squared
  distance expectations, triangle face/edge/vertex residual squares 9/25/1,
  and the single admitted boundary root for the outward unit-circle segment
  `(1+t,0)`, t in [0,1]. These are fixture-mathematics checks, not execution of
  the Rust kernels or their dispatch paths.
- No runtime tests, dispatch measurements, or performance tests ran. Incremental
  compilation remains disabled and shared target storage remains about 1.6 GiB.
  Full workflow checks remain waived by the owner for this docs-only session.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-distance-{doc,check,downstream-doc}.log`.
The current inventory has 19 files under Hyperlimit's predicates directory;
with the prior whole-file records, each has now received implementation and
Rustdoc review. This is not a crate-wide or repository-wide completion claim:
Hyperlimit's remaining examples, benches, fuzz programs, and support/documentation
files still require review, as does the much larger remaining repository.

## Hyperlimit allocation-profile example

Audited all 317 baseline lines of
`crates/hyperlimit/examples/allocation_profile.rs` at `17c6b1fe8`. Ownership is
development measurement instrumentation, not a predicate or allocator replacement
for library consumers. Relevant contracts are P5 and the admission protocol's
requirement to distinguish requested allocation traffic, retained/temporary
memory, complete lifetime, and completed workload. The missing witness was
accurate documentation of every private counter, allocator hook, snapshot,
guard, fixture, and entrypoint, with private example Rustdoc rendering.

Documented all 29 declarations and seven fields, including the three unsafe
GlobalAlloc methods and their caller safety requirements. Corrected the module's
workspace-root command to select `-p hyperlimit` explicitly. Preserved every
executable line, operation, output string, fixture, and feature branch.

The profiler forwards to System and observes process-wide allocator calls,
including worker activity. It counts successful allocations/reallocations, but
all observed frees regardless of allocation origin. Resizing counts full old/new
sizes as byte traffic and changes live balance only by the size difference.
This does not observe any simultaneous old/new storage internal to System.
Counters begin at zero after fixture construction and one warm-up per row;
pre-window allocations are not inventoried. Saturating subtraction can therefore
lose exact balance information when older allocations are released. Peak and
retained columns are estimates for the observation window, not complete-lifetime
or resident memory, allocator overhead, or per-predicate ownership accounting.

Documented relaxed independent snapshots, non-overlapping/non-nested measurement
preconditions, reset/in-flight-event limitations, unchecked count growth, and
guard behavior during ordinary unwinding. Returned report/vector destruction is
inside the measured loop; fixtures and warm-up are outside it. The optional
positive iteration argument defaults to 256, and each batch invocation counts
as one operation. Parallel builds add an eleventh row to ten ordinary rows.
Results are black-boxed and dropped, with no geometry-success or certainty
assertion. Thus printed totals alone do not prove exact workload completion or
resource-contract compliance; this audit makes no new performance claim.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. A scoped scan confirms docs on
  all declarations/fields; comment-only/blank-line removal gives source identical
  to baseline without other normalization.
- `cargo check --locked -p hyperlimit --example allocation_profile --all-features
  --offline -j2` and the same command with `--no-default-features` passed.
- `cargo rustdoc --locked -p hyperlimit --example allocation_profile
  --all-features --offline -j2 -- --document-private-items -D warnings
  -D rustdoc::broken_intra_doc_links -D rustdoc::private_intra_doc_links` passed,
  as did the corresponding no-default-features command. Verified all 37 complete
  doc groups across 24 pages in each build, including seven fields and all
  allocator/Drop implementation methods. The render checker excludes Markdown
  code-fence delimiters when comparing the module command with HTML text.
- No profiler invocation or runtime test ran; no allocation figures were
  collected. No library implementation changed, so the affected consumers are
  the two compiled example configurations rather than downstream geometry.
  Shared target storage remains about 1.6 GiB with incremental compilation off.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-allocation-{check,minimal-check,doc,minimal-doc}.log`.
The owner has waived full workflow validation for this docs-only session.
Remaining Hyperlimit examples include `perf_atlas.rs` and
`write_benchmarks_md.rs`; their surrounding benchmark/report modules also need
whole-file review. The broader repository goal remains active and incomplete.

## Hyperlimit stored benchmark-report writer

Audited all 472 baseline lines of
`crates/hyperlimit/benches/benchmark_report.rs` and all 15 baseline lines of
`crates/hyperlimit/examples/write_benchmarks_md.rs` at `eab87de1e`. Ownership is
development evidence/reporting rather than geometric computation. Relevant
contracts are the admission protocol's completed-workload and evidence-matrix
requirements, and P5's distinction between actual resource bounds and unbounded
input-driven work. Input is stored Criterion JSON, selected filesystem trees,
the crate manifest/source text, environment/argv, and existing Markdown. The
missing witness was accurate documentation of private parsing, grouping, and
replacement behavior, plus rendered coverage. No report-generation execution
or benchmark measurement was needed for this documentation-only edit.

Documented all 33 writer declarations and 17 fields, plus the example's two
declarations and module introduction. Three writer declarations are test-only
(the module and two tests); 30 render in private docs. Executable source is
identical after removing comment-only/blank lines. Existing lint attributes,
output strings, tests, manifest entries, and all file-writing behavior remain
unchanged. The example docs give an explicit workspace-root package selection.

The writer is an inventory of stored results, not a certificate of completed
equivalent workloads. It selects CRITERION_HOME verbatim before CARGO_TARGET_DIR
and executable ancestry/fallback logic. Relative paths have different bases in
those branches. Collection traverses directories without crate/revision/feature
filters or cycle/depth limits. A full-ID duplicate is replaced in filesystem
visit order; sorting the final IDs does not make that winner selection stable.
Missing required files/fields skip rows, required read/JSON errors propagate,
and optional change-estimate errors are suppressed. Missing confidence bounds
and median are replaced with the mean. No source confidence level is read even
though the table heading is always 95% CI. Throughput text is a workload count,
not a computed per-second rate; invalid count conversion yields zero.

The manifest inventory is deliberately described as the implemented line scan,
not a complete TOML parser: explicit bench blocks only, name before required
features, same-line quote extraction, unconditional # stripping, conventional
source paths, and substring-based Criterion detection. This crate currently has
two explicit benches with autobenches disabled. Comparison groups use exact
group/input strings, multiple function IDs, and input/name eligibility rules.
Their count is participating rows, not groups or pairwise comparisons. Ratios
use the fastest stored mean without positivity or semantic-equivalence checks;
linked reports are not validated. None of those limitations was repaired or
presented as proof that the performance admission contract is satisfied.

Automatic callers use the environment/argv skip gate; the direct example does
not. Callback errors print without failing the benchmark, whereas the example
propagates I/O errors. Failure to read existing benchmarks.md substitutes a new
header. Replacement edits the first marker pair; a missing END drops the old
tail, while multiple/nested sections are not validated. The write is direct,
without an atomic replacement or concurrent-writer protocol. Documented these
actual behaviors without running the writer or modifying any stored report.

Revisited the current manifest and the two callers at predicates.rs lines
3570–3599 and retained_fuzz.rs lines 1–32/108–126. These excerpts do not establish
whole-file review of either benchmark workload. Read all 162 lines of
dispatch_trace.rs as next-batch context; its Rustdoc/consumer audit remains
pending. The existing replacement test checks changed payload and one opening
marker, not repeated identical replacement or malformed-section behavior. The
manifest test checks two entries and only the first entry's name/heuristic.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, scoped declaration/field scans, and exact
  comment-stripped source identity checks passed for both edited files.
- `cargo check --locked -p hyperlimit --bench predicates --bench retained_fuzz
  --example write_benchmarks_md --all-features --offline -j2 -v` passed, as did
  the corresponding `--no-default-features` command. These are the direct
  consumers of the shared module; no geometry library changed.
- Verbose commands showed the harness=false benches use `--cfg test` without
  `--test`. To check the actual test bodies explicitly, replayed each checked
  example rustc invocation with `--test`, the same dependency metadata/features,
  compile-time manifest root, and a separate temporary output directory. Both
  metadata-only test-harness compilations passed; neither harness executed.
- `cargo rustdoc --locked -p hyperlimit --example write_benchmarks_md
  --all-features --offline -j2 -- --document-private-items -D warnings
  -D rustdoc::broken_intra_doc_links -D rustdoc::private_intra_doc_links` passed,
  as did the no-default-features version. Verified 51 complete rendered doc
  groups across 33 pages in each configuration: 48 from the shared module,
  including all 17 fields, and three from the example. Test-only docs were
  source-reviewed and test-harness compiled, not rendered in ordinary docs.
- No runtime tests, benchmark workloads, or report writers ran. Shared target
  storage remains about 1.6 GiB; metadata harness outputs use separate temporary
  directories. Incremental compilation remains disabled. Full workflow checks
  remain waived by the owner for this docs-only session.

Logs: `/tmp/csgrs-rustdoc-hyperlimit-report-{check,minimal-check,doc,minimal-doc}.log`
and `/tmp/csgrs-rustdoc-hyperlimit-report-harness-{check,minimal-check}.log`.
This advances the complete-repository audit without closing the remaining
benchmark, trace, fuzz, example, support-file, or wider kernel work.

## Hyperlimit benchmark dispatch-trace helper

Audited all 162 baseline lines of `crates/hyperlimit/benches/dispatch_trace.rs`
at `eeb18f2e4`, including both feature implementations and crate-visible aliases.
Ownership is diagnostic sampling/reporting for the predicate benchmark. Relevant
contracts are P5 and the admission protocol's distinction between observed
dispatch paths, resource measurements, correctness, and completed equivalent
workloads. Input is caller-created names/case slices/closures, feature selection,
thread-local trace state, retained process-global rows, and the output directory.
The missing witness was accurate public/private/conditional Rustdoc with checked
feature-specific rendering. All 17 declarations/re-exports now have docs; only
comments changed, with executable source identical after comment/blank removal.

The process-global enable flag is sticky and begin-run does not clear retained
rows. Sampling resets, records, and drains the calling thread's scalar trace
state. It does not automatically collect worker threads, and nesting can erase
an enclosing trace's counters even though the scalar guard restores the previous
recording flag. Samples with an empty dispatch vector are discarded even when
rational counters exist; an older same-name row remains. Nonempty samples replace
same-name rows. Sample/name conversion panics propagate, and poisoned row mutexes
panic; there is no rollback or recovery protocol here. Disabled-feature shims do
not invoke callbacks or convert names, but arguments/captured values are still
constructed by the caller and dropped normally.

The writer holds the global map mutex through formatting and direct file I/O,
keeps the rows afterward, and writes dispatch_trace.md relative to process cwd.
An empty map leaves an existing report untouched. Errors print without returning
failure; writes are not atomic. Row labels are not Markdown-escaped. Correlation
categories can overlap, and rational peak operand bits are not heap-memory usage.
Neither trace counts nor this report establish correctness, certainty, completed
workload, or non-instrumented timing. No row-count/retained-memory bound exists.

Supporting reads of `hyperreal/src/dispatch_trace.rs` at lines 100–225,
430–550, and 700–765 confirm thread-local recording, reset/drain, guard behavior,
and summary delegation. These are excerpts, not a whole-file scalar trace audit.
Revisited predicate-benchmark imports, selected sample call sites, and the
previously read trace-mode main path to confirm this helper is called outside
the Criterion timing loops. The complete benchmark workload file still awaits
semantic/Rustdoc review.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, scoped docs on all 17 declarations/aliases,
  and comment-stripped source identity passed. Existing lint attributes and
  diagnostic behavior remain untouched.
- `cargo check --locked -p hyperlimit --bench predicates --offline -j2` passed
  with `--all-features`, `--no-default-features`, and
  `--no-default-features --features parallel`. These exercise the recorder,
  minimal shims, and parallel-only row shim respectively.
- Bare `cargo rustdoc --bench predicates` omitted development/self dependencies
  in this environment and failed with unresolved imports. Supplying direct
  dependency metadata alone still lacked transitive search paths. Recovered
  without code/manifest changes: collect compiler-artifact records from the
  corresponding successful `cargo check --message-format=json`, pass checked
  rmeta files for the library/dev dependencies via `--extern`, and add dependency
  search paths for their artifact directories. Metadata here is distributed
  under debug/build/package/hash/out, not solely debug/deps.
- With those matching inputs, `cargo rustdoc --locked -p hyperlimit --bench
  predicates` passed for all three feature configurations, using
  `--document-private-items -D warnings -D rustdoc::broken_intra_doc_links
  -D rustdoc::private_intra_doc_links`. Exact augmented command arrays are saved
  in the per-configuration doc-command JSON files alongside the artifact lists.
  Verified 11 complete doc groups/pages with all features, seven with minimal
  features, and eight with parallel only. Rustdoc omits crate-visible import
  re-exports even in private docs: their one/two comments are source-reviewed and
  compiled, not counted as rendered; underlying function docs render correctly.
- No trace workload, benchmark, test, or report writer ran. No dispatch counts,
  memory/performance measurements, or implementation correctness claims were
  newly established. Shared target storage remains approximately 1.6 GiB with
  incremental compilation disabled. Full workflow checks remain owner-waived.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-bench-trace-{all,minimal,parallel}`
with `-check.log`, `-artifacts.jsonl`, `-doc-command.json`, and `-doc.log` suffixes.
The initial bare-command failure is retained in the separate `bench-trace-doc.log`.
The helper review is complete; remaining benchmark workloads, retained-fuzz
support, perf-atlas example, fuzz targets, and wider repository work remain open.

## Hyperlimit retained-case benchmark and support

Audited every baseline line of `crates/hyperlimit/benches/retained_fuzz.rs` (126)
and `crates/hyperlimit/benches/support/retained_fuzz.rs` (588) at `e496c81ec`.
Ownership is benchmark generation, timing-based selection, and persistence, not
geometric predicates or libFuzzer execution. Relevant contracts are P5 and the
admission protocol's evidence, resource, and same-completed-workload requirements.
The domain includes target/seed callbacks, environment/argv, historical Markdown,
promoted TSV, score metadata, and the compiled fuzz-target inventory. The missing
witness was accurate documentation of every authored item and private field,
including skipped work, malformed history, and measurement limitations.

Documented all 48 support declarations and 13 fields, plus ten authored benchmark
declarations and both module introductions. Executable source is identical after
removing comment-only/blank lines: generators, retention, macros, output strings,
policies, sampling parameters, and persistence behavior were not changed.

The benchmark dispatches by two fuzz-target names but constructs its own bounded
rational and symbolic fixtures. It does not decode saved fuzz bytes, invoke the
fuzz targets, assert invariants, or check predicate certainty/success. Rational
numerator magnitudes are 1–2001 and denominators 1–1009. The representation branch
uses either pi times a rational or sqrt(offset squared plus one), then an
axis-aligned unit triangle on those offsets and scalar comparisons. These are
two constructions, not exhaustive representation coverage. Each timed call
includes construction and destruction. Criterion registration follows initial
collection/refresh, so row filtering does not suppress that pre-registration work.

The configured default is 20,000 cases total across targets, not per target.
Round-robin selection gives 10,000 seeds per each of the two current targets,
and five immediate probes per case, if all calls return normally. These counts
follow the source loops, not a measured run. Invalid/zero/non-Unicode environment
counts fall back to the default. A smaller configured count can leave targets
unsampled. Fresh case timing is the minimum wall-clock duration, clamped to at
least one nanosecond; no separate warm-up, timeout, isolation, success assertion,
or total-work/memory bound exists for collection or re-probing.

History deduplicates by target/seed and retains the maximum recorded probe value,
then truncates to 1,000 rows. Human-readable durations round through f64 and may
lose precision; parser inputs are not checked for nonnegativity/finiteness/range.
Promoted-file loading filters current targets but keeps duplicates and zero times.
Rotation re-times all accepted existing rows before truncation, preserves their
historical maxima, releases the fastest one, excludes that key from same-refresh
additions, and backfills toward at most 100 cases. Historical candidates are not
filtered by current target names or re-timed during selection. Stale names can
therefore reach a later callback and panic. The fixed replay_promoted_100 row
name does not guarantee that 100 cases were available.

The report-skip variable suppresses writes and score probes, not collection or
promotion re-probes. With writes enabled, promoted TSV and history are written
before scoring; errors log without being returned or rolled back. A score panic
can occur after earlier files changed. Existing promoted-path presence alone
enables previous-score lookup, regardless of whether promoted contents parsed.
The score is an integer arithmetic mean of fresh minima, with saturating sum;
delta and change in delta use unchecked signed-range conversions/arithmetic.
Despite output terminology it is not lexicase selection, deterministic wall time,
or a time-normalized derivative. Prior metadata cannot establish comparable
case sets, revisions, policies, or machines. A missing score closing marker
prepends a new section while leaving the old unterminated section intact.

Read the complete fuzz Cargo.toml as configuration context: it is an isolated
cargo-fuzz workspace, declares the two callback target names, and marks both
bins test=false/doc=false. This does not review their Rust implementations or
exempt those targets from the documentation goal. Also inspected the locked
Criterion 0.5.1 macro definitions at macros.rs lines 65–82 and 120–133 in the
local dependency cache. They generate public benches and private main without
item Rustdoc. Their behavior is now described accurately in benchmark module
prose, but their individual rendered pages still lack attached documentation.
This is an explicit remaining generated-item coverage gap, not a 100% coverage
claim or an exception to the repository mandate. The macros were not rewritten
or the dependency patched during this documentation-only batch.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, authored-item/field scans, and exact
  comment-stripped source comparisons passed for both files.
- `cargo check --locked -p hyperlimit --bench retained_fuzz --offline -j2`
  passed with `--all-features` and `--no-default-features`. No new test source
  was added; these modules contain no authored test functions.
- Private benchmark Rustdoc passed in both configurations with `-D warnings`,
  broken/private intra-doc links denied, and the same checked dependency-rmeta
  and transitive search-path method established in the previous trace batch.
  Verified all 73 authored doc groups across 57 pages: 62 support groups,
  including 13 fields, and 11 benchmark groups. Separately inspected generated
  benches/main pages and confirmed the unresolved attached-item-doc gap above.
- No benchmark, timing probe, fuzz target, test, or report writer ran. Stored
  history, promotion, score, and Criterion output files were not refreshed.
  Shared target storage remains about 1.6 GiB with incremental compilation off.
  Full workflow validation remains waived by the owner this docs-only session.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-retained-fuzz-{all,minimal}` with
`-check.log`, `-artifacts.jsonl`, `-doc-command.json`, and `-doc.log` suffixes.
The whole-file reading and authored documentation are complete for this batch;
generated-item coverage and the wider repository audit remain open. Remaining
Hyperlimit work includes predicates.rs workloads, the perf-atlas example, fuzz
targets, and support/documentation artifacts.

## Hyperlimit fuzz targets and workspace documentation

Read every baseline line of both fuzz targets at `d110cd03f`:
`predicate_invariants.rs` (848) and `hyperreal_representations.rs` (187).
Also read the entire fuzz README, Cargo.toml, and 310-line Cargo.lock. This
completes whole-file reading of the five tracked files under Hyperlimit's fuzz
directory, not proof that the fuzz assertions pass or that generated-item
documentation is complete. Ownership is the predicate qualification harness;
relevant contracts are P5 and P8–P11, with P4 for retained evidence. The domain
is byte-derived bounded rational coordinates, fixed scalar recipes, and short
opaque graph constructions under explicit policies. Missing acceptance witnesses
were private-item/field documentation and accurate descriptions of asserted
coverage, uncertainty, and generated harness behavior.

Documented all 18 authored declarations (including two conversion methods and
one function-local helper), 26 fields, and both module introductions. Executable
source is identical after removing comment-only and blank lines. No generator,
assertion, policy, macro invocation, dependency, manifest flag, or lock entry
changed. The README now distinguishes actual target coverage from integration
coverage and includes verified commands for explicitly documenting doc=false bins.

The invariant target decodes four planar and five spatial points with i16
numerators and denominator-byte modulo 16 plus one. Its shared-scale helper maps
each numerator into 1–16 over prime denominator 17; the nested nonzero_mod17 name
does not describe its actual modulo-16 calculation. Most predicates use
APPROXIMATE_512; planar orientation separately compares STRICT if the former
decides. Orientation/point-line sequential and parallel batches compare complete
outcomes, while most remaining assertions compare values only. Several reversal,
boundary-zero, and report branches accept uncertainty by skipping assertions.

Documented the actual geometric fixtures: ring queries at source vertices,
repeated circle/sphere sites, rational coefficient facts, interval/ball signs,
coordinate-plane point recovery, segment symmetry, endpoint zero distances,
point-sized sphere/AABB contacts, and segment/ray triangles sharing their origin
with a triangle vertex. That last relation does not state a general ray/segment
equivalence. Segment/plane and coplanar records require validation; lifted
triangle pairs are planar, and relation/report checks do not independently prove
the validators' geometric sufficiency. Fixed half-parameter, circle, square/cube,
support-DOP, and feasible/infeasible halfspace cases supplement generated inputs.
Source-derived expectations are documented without claiming a newly executed run.

The representation target has 20 fixed recipes spanning eight public structural
kinds, plus four opaque recipes, not 22 fixed certificate recipes. Public-kind
assertions precede nonzero rational scaling and do not check each scaled class.
The scale helper receives denominator byte one, so even the zero-substituted
scale numerator is divided by two. Offset denominators range 1–256; signed step
numerators range 1–7 with denominators 1–5. Rational offsets enter edge deltas,
not every carrier. The stride byte jointly controls policy, partner rotation
modulo 24, and recursive depth modulo eight; shear also supplies graph opcodes.
All pair rotations are selectable across inputs, but no particular fuzz run is
proved to visit all pairs. Orientation/batch comparisons accept matching Unknown;
the source point must classify On, while sign and ordering results are discarded.
Certainty receipts are not asserted. Graph updates are limited to 1–8 rounds;
this does not bound the complete target's work, memory, or elapsed time.

Revisited relevant excerpts of the already reviewed representation integration
test: it contains the additional Pow10/Pow2 recipes and serde-gated inventories
of 70 node tags and 18 shared-constant tags. Corrected the README's conflation
with fuzz-target coverage and stale 57-node count. Its retained-performance
section now reflects the preceding benchmark audit: independent generators,
20,000 total initial cases, at most 100 promotions, arithmetic-mean timing score,
unchecked predicate outcomes, and best-effort nontransactional writes. This
corrects documentation, not the target coverage, runtime behavior, or report text.

The fuzz manifest is an isolated workspace enabling Hyperlimit std/parallel with
two test=false/doc=false bins. Read all 35 lockfile package records and dependency
edges; the lock selects libfuzzer-sys 0.4.12. Inspected that cached version's typed
macro arms at lib.rs lines 298–374 (also initially viewed equivalent 0.4.13
context before checking the lock). The initializer calls library initialization;
the input function rejects insufficient/undecodable bytes or writes debug input
and rejects it in diagnostic mode; otherwise the inner runner executes the body.
All three functions are nested in an anonymous constant. Only the initializer
has a short generated doc comment; the input and runner lack attached docs.
Module prose describes their behavior, but this remains a generated-item coverage
gap. No dependency or macro expansion was rewritten. Derived traits likewise
are not counted as authored declarations or asserted fully audited here.

Validation after final source edits:

- `git diff --check`, comment-stripped source identity, authored-item/field scans,
  and local README link resolution passed. Focused Rustfmt passes for the
  invariant target. The representation target has one pre-existing assertion
  formatting difference: baseline and current Rustfmt checks both exit 1 and
  request exactly the same changed lines. It was left untouched in this docs-only
  worktree; this is a recorded baseline finding, not a passing format claim.
- The first offline check lacked locked cc 1.2.62. After authorized dependency
  retrieval, compilation passed; the final offline command also passed:
  `cargo check --locked --manifest-path crates/hyperlimit/fuzz/Cargo.toml
  --bins --offline -j2`.
- `cargo rustdoc --locked --manifest-path crates/hyperlimit/fuzz/Cargo.toml
  --bin TARGET --offline -j2 -- --document-private-items -D warnings
  -D rustdoc::broken_intra_doc_links -D rustdoc::private_intra_doc_links`
  passed for both targets. Verified 45 complete doc groups across 17 rendered
  pages: 13 representation groups and 32 invariant groups, including all fields.
  The nested nonzero_mod17 helper has attached source Rustdoc but no item page;
  the three macro-generated functions also have no standalone pages. Rendering
  therefore does not demonstrate their item-documentation coverage.
- No fuzz campaign, test, benchmark, or report writer ran. No completion,
  certification, timing, or resource-bound claim was newly established. Shared
  build storage is about 1.7 GiB with incremental compilation disabled. Full
  workflow validation remains owner-waived for this docs-only session.

Evidence: `/tmp/csgrs-rustdoc-hyperlimit-fuzz-check-final.log`,
`/tmp/csgrs-rustdoc-hyperlimit-fuzz-{invariants,representations}-doc.log`,
`/tmp/csgrs-rustdoc-fuzz-format-{baseline,current}.log`, and the render checker
`/tmp/csgrs-fuzz-render.py`. The whole-file reading and authored documentation
are complete for these targets; generated-item coverage and the wider audit
remain open. Hyperlimit's predicates benchmark workloads, perf-atlas example,
and other support/documentation artifacts still require review.

## Hyperlimit predicate benchmark workloads

Audited all 3,599 baseline lines of `crates/hyperlimit/benches/predicates.rs` at
`8ec8ae9ef`, including every fixture, timed closure, score adapter, feature gate,
and the custom main. The previously reviewed benchmark_report and dispatch_trace
modules remain separately recorded. Ownership is predicate cost/qualification
instrumentation, with P1, P5, P8–P11 and the admission protocol's same-completed-
workload requirements. Inputs include generated binary64 values imported exactly,
explicit rational fixtures, and one symbolic pi-scaled simplex. Missing witnesses
were accurate private-item docs and clear separation of timing labels from the
geometry and validation actually performed.

Documented 118 authored declarations, two Workload variants, and the module.
This includes 99 functions/methods, twelve aliases, four constants, the enum,
and two module declarations. One constant is local to the parallel registration
function. There are no authored struct fields. Corrected narrative comments
that described reused-input rows as cold or suggested timing scores establish
exact-output preservation. Executable source remains identical after removing
comment-only/blank lines; no fixtures, assertions, APIs, scoring, policies,
registration order, timed calls, or pre-existing lint allowances changed.

The 512-case base, exact binary64 ingress versus explicit fractions, constructor
preconditions, tuple order, transforms, and mathematical fixture domains are now
documented individually. Immediate rows time one prebuilt query; most generated
rows traverse a complete vector. Other rows perform mixed work: 64 area queries,
two sets of 512 convex-membership queries, or two feasibility classifications.
Only the parallel helper publishes element throughput, requests 20 samples, and
times allocation/destruction of result vectors. Its 8,192 inputs repeat the base
sixteen times; it does not assert sequential/Rayon equality. Prepared evidence
is generally constructed outside query timing, while evidence-derivation and
support-DOP construction rows deliberately include preparation.

Most outcomes are reduced to unasserted scores that discard certainty/reasons.
Unknown has a distinct score in some adapters but is conflated with false or
failed validation in other rows. The infeasible-halfspace flag defaults to true
on Unknown. Floating determinant scoring maps NaN to zero. Summed scores are not
collision-free checksums. A few filter and support-DOP constructors expect success
and can panic, but this does not turn the whole harness into a completion corpus.
No cold-cache reset or target-wide work/memory/cancellation/time budget exists.

Independently checked three source-derived geometric observations with exact
Python Fraction arithmetic, saved in the witness log below:

- `large_rational_near_degenerate_incircle2d_cases` constructs x=(2tD)/D=2t,
  with D=1,000,003, while y uses denominator (D²+t²)D. It does not use the
  usual rational-circle denominator for x. At index zero, t=-501 and x=-1002;
  squared radius is strictly greater than one. The label cannot establish a
  near-unit-circle fixture. The generator and benchmark row were preserved.
- `oriented_plane_cases` uses near-degenerate height 0.38t+0.24u±1e-13, which
  is not the plane through its fixed sites. For the first generated query,
  exact arithmetic on the resulting binary64 coordinates gives a vertical
  residual whose absolute value exceeds 1/100. This verifies the documentation
  limitation without executing the predicate or claiming a timing measurement.
- The port helper's projected-line row sends the endpoints of a vertical
  segment through XY projection. Both become (0,0) on the supporting x-axis;
  both orientation determinants and their difference are zero. Inspection of
  the previously reviewed projected-ratio API confirms this exercises an absent
  ratio, not a transverse projected crossing. The companion midpoint segment
  ratio has a different, noncollapsed fixture.

The triangle rows named report_replay construct reports and score relations;
they do not call source replay validators. Selected DOP/plane/triangle/ring rows
do call internal validators, sometimes only after an intersecting/inside relation,
and score failures without asserting success. The DOP rows include building the
source object on each iteration. Revisited the relevant projection and
segment/plane API signatures to verify caller argument meaning. That inspection
also corrected the previous batch's two fuzz Input field comments: s/t are the
segment endpoints for the segment/plane event, while p/q/r define its plane.
This correction changes only those comments, not the fuzz code or prior results.

Custom main is authored and now documented, unlike the retained-fuzz Criterion
macro-generated entry points. Either exact trace flag selects a never-matching
Criterion filter and bypasses normal Criterion argv configuration, but all
fixture construction and registration still happen. Sampling requires the
dispatch-trace feature and the enabled run switch; its report path bypasses the
benchmark-summary skip flag. Without that feature the trace-only path samples
and writes nothing. Normal mode may write stored Criterion summaries after its
final summary; write errors are logged rather than returned as failure status.

Validation after final source edits:

- Focused Rustfmt, `git diff --check`, authored-item/variant scans, and exact
  comment-stripped source comparisons passed for both changed Rust files.
- `cargo check --locked -p hyperlimit --bench predicates --offline -j2` passed
  with all features, no default features, and no default features plus parallel.
- Private benchmark Rustdoc passed for all three configurations, denying warnings
  and broken/private intra-doc links, using matching checked rmeta and transitive
  dependency search paths as established in the earlier benchmark-helper audit.
  Initial interval notation was parsed as links; wrapping it in code spans fixed
  the five reported link errors before the final successful runs.
- Verified complete doc text for 120 groups across 116 pages in all-feature and
  parallel-only docs, and 117 groups across 113 pages in minimal docs. The local
  PARALLEL_BATCH constant has source Rustdoc but no standalone page; the minimal
  build additionally omits three parallel-gated functions. Derived Clone/Copy
  APIs are not counted as authored declarations or independently re-audited here.
- Rechecked and regenerated private docs for the corrected fuzz invariant target;
  its rendered fields were inspected with the existing fuzz render checker.
  No benchmark, dispatch collector, report writer, test, or fuzz campaign ran.
  No timing, certified-completion, dispatch-count, or correctness claim for the
  geometry implementation was newly established. Full workflow checks remain
  owner-waived for this documentation-only session.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-predicates-bench-{all,minimal,parallel}`
with `-check.log`, `-artifacts.jsonl`, `-doc-command.json`, and `-doc.log` suffixes.
Exact fixture observations are in `predicates-bench-witnesses.log` under the same
`/tmp/csgrs-rustdoc-hyperlimit-` prefix. The fuzz correction uses
`/tmp/csgrs-rustdoc-hyperlimit-fuzz-argument-doc{,-check}.log`. Whole-file reading
and authored documentation of this benchmark are complete; the perf-atlas
example, other Hyperlimit support/documentation artifacts, generated-item gaps,
and the wider repository audit remain open.

## Hyperlimit performance-atlas lattice and input foundations

Reviewed and documented the foundation section of
`crates/hyperlimit/examples/perf_atlas.rs`, baseline lines 1–1073 at `fe61cfa4e`:
axis decoding, integer arithmetic/search helpers, lattice fixtures, representation
recipes, term evaluation, geometry decoding, and input retention/reconstruction.
This is a partial-file batch; the 3,090-line example's workload registrations and
later helpers are not yet fully audited or documented. Read baseline 1074–1455,
1784–1835, 2529–2700, and 3074–3090 as consumer context, not completed item-review
coverage. Also inspected shared perf-atlas RNG and Outcome definitions around
lib.rs lines 78–228; that shared library still needs its own complete audit.

Ownership is measurement-fixture construction and the scalar/predicate boundary.
Relevant contracts are P1, P4, P5, P8/P9, and the admission protocol's explicit
domain/resource/evidence requirements. The domain includes arbitrary-size lattice
integers subject to helper preconditions, rational scales/translations, selected
symbolic recipes, and flat geometry streams. Missing witnesses were complete
private documentation and accurate mathematical/resource qualifications for the
existing helper claims. Documented 86 named declarations, 35 fields (including
two tuple payloads), 18 enum variants, and the module introduction. Three of the
declarations are function-local static caches.

Only comments changed, plus expanding the Ready and Sum enum payloads to attach
field documentation. Exact comment-stripped source comparison passes after
normalizing those two named tuple layouts back to their original single lines;
no other code normalization is needed. No arithmetic, generator, range, search,
predicate, ordering, fixture, assertion, or policy changed.

The RNG helper's top-bit-set limb contract makes big/signed nonzero even for a
requested width of zero. Odd forces the low bit without proving primality. The
small-prime, radius-65 circle, and radius-nine sphere tables are initialized once
and retained for process lifetime. Bounded draws use remainder reduction, so the
example no longer implies a proved uniform distribution. Distinct-index and
separated-angle helpers have no attempt bound; impossible requests can loop,
and n=0 is not a valid index domain despite the RNG's zero fallback.

Fixed-base Miller–Rabin screening is documented as candidate evidence, not a
general primality certificate. Two-squares search asserts its final exact norm;
three-squares search inherits that check but has no termination/resource contract
for all integers matching its residue precondition. Bounded arithmetic inspection
shows n=1 and n=2 always produce remainder one and cannot pass the prime screen.
This checks the branch arithmetic without running a nonterminating loop or
claiming that current large atlas fixtures fail.

The planar determinant helper controls an exact determinant, not universally an
orthogonal projection into the segment. Its vx modular interval alone is
insufficient: u=(5,4), v=(4,16) satisfies the l=3,k=0 determinant equation with
vx in range but has projection fraction 84/41. The exact-degenerate branch does
use two positive multiples with the smaller point between the endpoints. Spatial
fixtures apply a unit-lower times unit-upper matrix: determinant +1 preserves
orientation/volume/incidence, not Euclidean distances or all coordinate structure.
Only the factors' entries are bounded by one; their product can have a diagonal
entry three. Inside/Crossing names describe base combinations before vertical
lift, not a normal-displacement/projection certificate. Circle/sphere queries
retain checked radial identities on successful decomposition, while stepping
the initial perturbation and unbounded candidate searches prevent an exact
proximity or total-work claim. Explicit plane t is an equation value, not
unit-normal distance, and finite levels saturate at magnitude two.

Frame docs now state each actual mapping. Integer ignores dyadic magnitude
metadata; rational and symbolic coordinates use distinct per-axis translation
rules; widening adds a positive factor and translation without enforcing the
requested output bit length. Coordinate f64 conversion checks width and a lower
shift bound but not upper overflow; length lacks that lower guard and can
underflow; squared/offset use exact dyadics even for the F64 kind. Trig kinds are
not admitted to framed coordinate construction, although squared's fallback
would still produce PiSquared. Ratio divides by absolute denominator magnitude,
not a signed denominator. Large shifts and allocations have no independent cap.

Term sums retain stored addition order and skip zero coefficients; trig factors
use rational radians with ordinary sin/cos. No strict refusal or cost trend is
asserted as a mathematical consequence of these recipes. Input readiness depends
on whether every term is Ready, not on Kind: ready geometry is built once;
otherwise each run creates local atom cells, evaluates terms, builds geometry,
calls the callback, and destroys the temporary. Cloned Ready values can retain
shared scalar state, and fresh local cells do not reset all underlying caches.
Geo's flat decoder panics for too few scalars and retains extras. The revised
module introduction removes claims of exhaustive representation coverage,
universally cold symbolic calls, and uniform typed refusal flags. Shared Outcome
behavior distinguishes Option presence, bool truth, and unconditional numeric
success; none alone proves geometry, certainty, or benchmark equivalence.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, declaration/field/variant review, and the
  narrowly normalized source-identity comparison passed.
- `cargo check --locked -p hyperlimit --example perf_atlas --offline -j2`
  passed with all features and no default features.
- `cargo rustdoc --locked -p hyperlimit --example perf_atlas --offline -j2`
  passed in both configurations with `--document-private-items -D warnings
  -D rustdoc::broken_intra_doc_links -D rustdoc::private_intra_doc_links`.
  Verified 137 complete doc groups across 61 pages in each configuration,
  including tuple fields and both FromReals implementation methods on the trait
  page. The three local static caches have attached source docs but no standalone
  Rustdoc pages. This count excludes all unaudited downstream declarations.
- Exact bounded arithmetic witnesses check the small three-squares limitation,
  projection counterexample, and unimodular product-entry distinction. They do
  not execute the atlas or certify completion of its intended workloads.
  No benchmark, fuzz campaign, report writer, or unbounded search ran. No new
  performance, whole-cache, primality, or implementation-completeness claim is
  made. Incremental compilation remains disabled; full workflows remain waived
  by the owner for this documentation-only session.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-atlas-foundation-{all,minimal}`
with `-check.log` and `-doc.log` suffixes. Bounded observations are in
`/tmp/csgrs-rustdoc-hyperlimit-atlas-foundation-witnesses.log`; the render checker
is `/tmp/csgrs-atlas-foundation-render.py`. Continue with the determinant registry
and workload methods after In2/In3, then all remaining example sections. This
batch does not complete the perf-atlas example or the wider repository goal.

## Hyperlimit atlas determinant and N-dimensional workloads

Audited baseline lines 1353–2057 of
`crates/hyperlimit/examples/perf_atlas.rs` at `4f3844611`, from the determinant
registry through nd_predicates. This follows the foundation batch and still
does not complete the composite, scalar, polygon, construction, batch, or main
sections. Ownership is atlas workload assembly and predicate-consumer measurement;
relevant contracts are P1, P4, P5, P8/P9 and the admission protocol's equivalent-
workload/evidence requirements. The domain is the four lattice determinant
families and dimensions 2–9 with framed scalar inputs. Missing witnesses were
private registry/field documentation and accurate claims about paired inputs,
timing boundaries, outcome checking, and generated degeneracy.

Documented 35 declarations (29 functions/methods, four constants, a lattice
callback alias, and Det) plus all eight Det fields. Clarified the module's timing
description using the shared harness implementation. Exact comment-stripped
source is identical; no layout normalization is needed in this batch. All
generators, callbacks, strings emitted into atlas reports, policy choices,
registration order, and result conversions remain unchanged.

Read shared perf-atlas lib.rs context for configuration/argument construction,
RNG stream hashing, operation registration, measurement, sweep/fuzz/grid behavior,
and powers_of_two (roughly lines 1–66, 300–723, 883–885). These are contextual
reads, not a completed shared-library documentation audit. The example does not
call reuse_inputs; default measurement constructs new input objects from a
cloned per-point RNG state outside each timed call. Thus ready geometry/evidence
is retained within Input but regenerated for each call by the surrounding
harness. Symbolic Input::run still reconstructs terms inside timing. No global
cache reset follows from either strategy. The first probe alone sets success;
later timed-loop results are black-boxed without rechecking Outcome. Timing
ceilings and point budgets are checked after calls or between batches, not by
preempting a stuck generator/predicate.

Direct/prepared labels hash to different RNG seeds. The robust reference has its
own label and also draws frame parameters before lattice sites, reversing the
ordinary sweep's construction order. These are comparable recipe families, not
paired measurements on identical samples. Source-level seed reproduction with
the default seed confirmed different streams for f64 direct/prepared under
`orient2 prepared evidence`, and f64/robust under `orient2 by degeneracy`.
Prepared orientation queries use line/plane sidedness APIs rather than the direct
Sign return type, and no corresponding outputs or certainty receipts are asserted.
Numeric robust returns always satisfy the shared Outcome implementation, including
the absence of a finiteness/sign comparison in this example.

Det trusts its private registry: make2 wins when both generators exist, neither
means no sweep/grid/fuzz registration, decoder size comes from exact name checks,
and inactive callbacks panic. Callbacks keep only Option<Sign>, dropping receipts
and Unknown reasons. Family kmax values are capped further by axis/fuzz helpers.
Fuzz is seeded atlas sampling with 0–7 upward level jitter, not a libFuzzer target.
The documented axes distinguish full exponent/bit ladders from smaller grids,
including the transcendental grid's 512-bit endpoint. Atlas stopping/prediction
can omit later samples/cells, so registration does not prove complete sweeps.
The emitted report strings still make unconditional timing/refusal statements
such as a cliff near k≈50, microsecond costs, and STRICT refusing trig degeneracy.
Rustdoc now explicitly identifies these as unverified here; the strings were not
altered in this comment-only batch, and no such claims are adopted as evidence.

N-dimensional decoders infer a dimension using an inequality rather than exact
length validation. Bounded arithmetic inspection confirms seven orientation
scalars produce inferred d=3 with row lengths 3,3,1,0; nine sphere scalars give
3,3,3,0 and an empty query. These malformed private inputs are not described as
validated geometry. Affine-combination delta zero gives an exact dependence,
but delta one is not a universal nondegeneracy witness: two 44-bit points sharing
x and a further unit change along y remain collinear. The fixture docs now make
that distinction. Coordinates after axis two reuse the third frame translation,
still a fixed translation vector but not independent random shifts in all axes.

The dimension sweep uses 2–9. Its affine-independent series returns Option<bool>,
so Some(false) counts as atlas success. Sphere fixtures use an odd 40-bit radius
around a 44-bit center, with interior, on-sphere, and one-unit-outside queries.
Their reused k≈44 label does not describe the radius-relative one-unit gap,
which is on the scale of 2^-40. No expected sign, relation, or receipt is asserted.
These are source-derived limitations and bounded arithmetic observations, not
runtime failures or performance measurements of the intended atlas workloads.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, declaration/field review, and exact
  comment-stripped source comparison passed.
- `cargo check --locked -p hyperlimit --example perf_atlas --offline -j2`
  passed with all features and no default features.
- Private example Rustdoc passed for both configurations with warnings and
  broken/private intra-doc links denied. A generic type initially parsed as HTML
  was changed to a code span. An earlier successful Cargo invocation reused stale
  rendered pages; full-text inspection caught that. Regeneration with the additional
  `-D rustdoc::invalid_codeblock_attributes` flag produced the current comments.
- Verified all 44 groups across 28 pages for this section plus the changed module
  introduction, in each configuration. Rechecked all 137 rendered foundation
  groups too; its three documented local statics remain source-only. No claim is
  made for unaudited sections or generated/derived items outside these counts.
- No atlas measurement, benchmark, fuzz campaign, report write, or unbounded
  fixture search ran. No new completed-workload, performance, or certainty claim
  was established. Full workflow checks remain owner-waived for this docs-only
  session, and incremental compilation remains disabled.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-atlas-determinants-{all,minimal}`
with `-check.log` and `-doc.log` suffixes. Bounded seed/shape observations are in
`/tmp/csgrs-rustdoc-hyperlimit-atlas-determinants-witnesses.log`; the render checker
is `/tmp/csgrs-atlas-determinant-render.py`. Continue at the composite-predicate
section, retaining the distinction between complete item review and context reads.

## Hyperlimit atlas composite fixtures (2026-11-26)

Reviewed every line of the composite section in
`crates/hyperlimit/examples/perf_atlas.rs`, baseline `e0bdd0e7b` lines 2212–2920.
This completes that section, not the remaining scalar/polygon/construction/batch
sections or the whole repository. The owning layer is Hyperlimit's performance
example; relevant contracts are P8–P11's policy, receipt, and decision semantics,
with P5/P6's resource and failure distinctions. The input domain is the selected
planar/spatial lattice recipes under the four KINDS frames. Missing acceptance
witnesses were private item/field docs and accurate descriptions of fixture
geometry, decoding, callback success, and construction validity.

Documented nine declarations (eight functions/methods and Shape), all four Shape
fields, and the composite registrations' formulas and limits. Read the generators,
Frame methods, and called segment, distance, AABB, and homogeneous API context to
check argument order, scaling powers, returned relations, and validity behavior.
All source changes are comments; exact comment-stripped source matches the
baseline without layout normalization. Report strings, fixture generation,
callback ordering, policy, and output handling are unchanged.

Derived serializes points, normal/offset plane tuples, squared quantities, then
linear quantities. It does not validate the decoder's layout or geometry. The
planar segment/triangle fixtures inherit determinant control, not guaranteed
interior Euclidean projection. Their sentinel does give exact between-point,
T-junction, and edge relations. Circle-line/segment tangent supports are built
correctly, but still generate an unused perturbed query with an unbounded search.
The distance-ordering axis omits finite level 96.

Spatial documentation distinguishes determinant-one shears from isometries,
positive normal pairing from a ray pointing away from the query's side, and
closed-ball intersection from surface-sphere intersection. Checked the current
sphere API: it accepts ordinary radii, and the example's two unperturbed radii
sum to the center distance as intended. Point/sphere and sphere/sphere still
construct an unused perturbed query. The small quaternion helper samples all
radius-nine lattice sites, including axes, not solely (1,4,8) permutations.
Its exact length follows from RᵀR=Q²I for squared quaternion norm Q.

Source inspection and bounded exact arithmetic establish these additional limits:

- Spatial collinearity lifting has cross product delta*(-alpha,-beta,1). Checked
  all 49 slope pairs on the previously retained planar projection counterexample;
  lifting does not turn determinant control into a projection guarantee.
- Admissible 42-bit h and 8-bit r can be parallel (r=(128,128,128), h=2^34*r).
  The point/segment distance is then zero; the threshold perturbation can be -1.
  This fixture does not enforce nonnegative thresholds.
- The three homogeneous-construction plane normals can coincide, even with the
  required 24-bit magnitudes. Identical plane rows give zero minors and an invalid
  all-zero projective carrier. No rank/validity rejection occurs. The callback
  tests Boolean incidence, despite the emitted description calling it sidedness;
  both Some(true) and Some(false) count as atlas success.
- Box queries use the second box's minimum, which always shares the first box's
  minimum z. Exact inequalities at x gaps -1,0,+1 give boundary,boundary,outside,
  not interior,boundary,outside. Plane/box growth preserves the selected corner's
  nearest signed residual; that expression is not unit-normal distance.
- Three bounded quaternion samples preserve all 102 radius-nine table points and
  the ordinary-radius sum. These observations illustrate the defining identities;
  they are not runtime predicate tests or a universal generator qualification.

All callbacks retain only values, dropping receipts and Unknown reasons. Atlas
checks Option presence for its initial probe; it does not assert classifications,
validate intersection reports, or replay construction evidence. No workload
completion, performance, or certification claim follows from this batch.

Validation after final source edits:

- Focused Rustfmt, `git diff --check`, and exact comment-stripped identity passed.
- Offline locked example checks passed with all features and no default features.
- Private example Rustdoc passed with warnings and broken/private links denied in
  both configurations. An initial build diagnosed bare array notation as broken
  links; code spans fixed the comments without suppressing diagnostics.
- Full-text rendered inspection found all 13 new groups across eight pages in both
  configurations. Rechecked the prior 44 determinant groups and 137 foundation
  groups; three previously recorded local statics remain source-only.
- No atlas execution, benchmark, fuzz campaign, report write, or unbounded random
  search ran. Full workflow checks remain owner-waived for this docs-only session.
  The existing shared target directory is reused with incremental compilation off.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-atlas-composites-{all,minimal}`
with `-check.log` and `-doc.log` suffixes. Bounded arithmetic observations are in
`/tmp/csgrs-rustdoc-hyperlimit-atlas-composites-witnesses.log`; render inspection
uses `/tmp/csgrs-atlas-composite-render.py`. Continue at Scalar and Pair.

## Hyperlimit atlas scalar, polygon, construction, and batch workloads (2026-11-26)

Reviewed every remaining line of `crates/hyperlimit/examples/perf_atlas.rs`,
from Scalar through main (baseline `1266fcc0b` lines 3040–3644). Together with
the three preceding atlas records this completes line-by-line review of this
entire authored file, originally 3090 lines at `fe61cfa4e`. The owning layer is
Hyperlimit's performance example. Relevant contracts are P1's conversion edge,
P8–P11's policy/receipt/decision semantics, and P5/P6's work/failure distinctions.
The domain is the named scalar recipes, rounded lattice rings, integer halfspaces,
widened constructions, and generated orientation/sphere batches. Missing witnesses
were attached private docs and accurate explanations of generation, transformations,
validation, success criteria, and timing. No kernel behavior or support claim changed.

Added docs for 19 authored declarations, all six Pair fields, and all five Scalar
variants. This includes both feature branches of BATCH_MODES and the function-local
Run/RingRun aliases. Reviewed relevant shared atlas RNG, Outcome, argument, timing,
and output code and Hyperlimit filter, reciprocal, homogeneous, line-construction,
segment-plane, halfspace, and support-DOP entry points. These are context reads,
not a completed audit of the shared perf-atlas crate. No generated output string,
fixture, callback, arithmetic, or policy changed. Exact comment-stripped source
matches the baseline without normalization.

Scalar docs now distinguish the rational source gap from the transformed root/log
gap and rounded binary64 arithmetic. A 40-bit positive numerator gives rational
x in [3/2,2), while 1+unit can round to binary64 2. A private request at k=53 can
collapse the floating separation, although the F64 sweep stops at 52. Equality
fixtures retain the positive-domain square-root/product and logarithm/sum
identities, without promising that a selected policy proves them. Roots/logarithms
are rebuilt inside timing. Only SqrtStrict selects STRICT; other recipes permit
the approximate terminal. The certified_interval_sign callback discards a receipt
that may be approximate; reciprocal callbacks collapse scalar errors and Unknown
to None. Nominal reciprocal operand widths span 8..8192 and may shrink on reduction.

Polygon generation uses rounded binary64 trigonometry, so the integer vertices
are the model tested by exact predicates. They are not exact circle sites or a
certified approximation with an error/topology receipt. There is no convexity or
distinctness validation in generation; the helper also accepts n=0,1,2. The atlas
sweeps 4..16384 vertices. Its convexity row checks LocallyConvex; the other rows
check only presence of a decision, not an expected sign or containment relation.

Halfspace offsets use L=floor(sqrt(n·n))+1, giving the retained center strictly
negative residual and a supporting-plane distance greater than 2^40 before
framing. This proves feasibility, not the boundedness asserted by the emitted
feasibility description. Repeated allowed positive normals admit the unbounded
ray center-s*(1,1,1), s>=0. Feasibility callbacks ignore the known center and
accept any returned report without inspecting its witness or relation. The DOP
row reconstructs thirteen unnormalized axes inside timing and does not validate
extrema or receipts. Repeated timing loops include destruction of returned DOP
carriers; the initial probe stops timing before that destruction.

Line construction does not exclude degenerate/parallel inputs, nor replay returned
incidence. Three-plane construction unconditionally wraps the homogeneous carrier
in Some without validating rank, finite affine existence, or nonzero coordinates.
Segment/plane generation does establish a strict crossing: writing |t|=q*n2+r
with 0<=r<n2, the other residual is sign(t)*(r-2*n2), hence strictly opposite.
The callback checks ProperCrossing but does not inspect the point, parameter, or
construction evidence. No chain-closure completion claim follows from these rows.

Batches use approximate policy, already-built F64/Rational geometry, and fresh
input vectors under the default atlas harness. Orientation counts span 1..16384,
sphere counts 1..4096. Level selection uses remainder reduction rather than a
proven uniform draw. Output allocation, scanning, and destruction occur in the
callback. All values must be present for the first probe to succeed, but no signs
or receipts are checked. Sequential and parallel labels seed different streams;
these rows do not establish agreement on identical cases. Main delegates argument
handling and direct, non-atomic report output to Atlas, without a total timeout.

Validation after final source edits:

- Focused Rustfmt, `git diff --check`, and exact comment-stripped identity passed.
- Offline locked example checks and private Rustdoc passed with all features and
  no default features, with warnings and broken/private links denied.
- Full-text rendering found all 27 active renderable groups across 12 pages in this
  batch for each configuration. Run/RingRun are documented in source but Rustdoc
  emits no separate local-alias pages. Both BATCH_MODES definitions were checked
  in their applicable configurations.
- Rechecked all three preceding sections. The complete authored file has attached
  docs on 149 declarations (110 functions/methods, 14 constants, six aliases, three
  local statics, five enums, ten structs, one trait), plus 53 fields/payloads and
  23 variants. The declaration adjacency scan has no missing entries. There are
  220 distinct rendered doc groups across 108 pages per configuration, after
  deduplicating the module introduction. Five local items remain source-only;
  one of the two documented BATCH_MODES definitions is inactive in each build.
  This inventory/render evidence covers authored items, not every generated
  derive implementation or an assertion of repository-wide coverage.
- Bounded arithmetic checks reproduced the scalar bounds/rounding examples, the
  unbounded-halfspace witness, and 16 crossing-displacement cases with parameters
  strictly between zero and one. These are not runtime predicate/atlas tests.
- No benchmark, fuzz campaign, atlas report, or unbounded fixture search ran.
  Full workflow checks remain owner-waived. Shared target storage is reused with
  incremental compilation off; no new build tree or release artifacts were created.

Evidence prefix: `/tmp/csgrs-rustdoc-hyperlimit-atlas-tail-{all,minimal}` with
`-check.log` and `-doc.log` suffixes. Bounded observations are in
`/tmp/csgrs-rustdoc-hyperlimit-atlas-tail-witnesses.log`; render inspection uses
`/tmp/csgrs-atlas-tail-render.py`. All Hyperlimit Rust files now have whole-file
review records; remaining repository files and generated API gaps still require
review. Do not infer crate-wide specification compliance from documentation.

## Next work

Continue with owning-crate implementations and remaining build/support files.
Consult existing whole-file records before selecting the next batch, and maintain
review records with each batch. Establish automated public/private item coverage across manifests
and features without confusing that metric with semantic review. Review every
remaining tracked file before claiming completion. Commit and push documentation
batches after focused checks under the owner's session-specific waiver.
