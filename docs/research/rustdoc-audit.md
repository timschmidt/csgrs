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

## Shared performance-atlas harness (2026-11-26)

Reviewed all 975 lines of `crates/perf-atlas/src/lib.rs` at `9514cf3da` and its
complete Cargo.toml. The crate is a dependency-free, unpublished workspace utility
with no feature matrix of its own. This completes authored source review for
these two files. The owning layer is the shared measurement/report harness, not a
geometry kernel. Relevant contracts are P37's claim/evidence scope, P5/P6's resource
and failure distinctions, and P9/P10's receipt/completed-workload requirements for
performance consumers. Missing witnesses were accurate public timing claims and
private data, helper, macro, generated method, and test documentation.

Documented 62 authored declarations: seven structs, 47 functions/methods (including
the generated-method source), one trait, one alias, three constants, one macro,
one enum, and one test module. Also documented all 41 fields/tuple payloads and
three variants, with an attached crate introduction. The source adjacency inventory
has no missing declarations. The always_succeeds macro now supplies documentation
to all 16 emitted Outcome::succeeded methods. Ordinary derive implementations are
not counted as separately audited generated source by this inventory.

Only documentation and layout needed to attach tuple/macro member comments changed.
Comment-stripped source is identical after normalizing the Rng tuple, three
Measured tuple variants, and one macro-body layout. No algorithm, callback order,
status, serializer, fixture, assertion, or lint configuration changed. P37 coverage
is distinguished from complete mathematical/resource compliance throughout.

Timing and resource documentation now states the actual sequence:

- One initial call estimates a fixed batch count; batches are not extended to meet
  a minimum duration. Counts are capped at MAX_FRESH=4096 in fresh mode, but this
  is neither a byte limit nor a total work bound. Reuse mode has a much larger cap.
- Only the initial Outcome is inspected. Repeated result drops occur inside timing;
  the initial result's inspection/drop and all ordinary input setup/drop occur
  outside the respective call/batch timer. A fresh factory invocation does not
  prove independent equal-valued inputs or reset shared caches/external state.
- The initial ceiling is checked after completion. Its record drops the Outcome
  status/reason, even for a failed callback. Point budgets are checked between
  batches; they do not preempt calls or setup. At least one batch runs when samples
  is positive, even if setup already consumed the budget. Zero samples can panic
  on the empty statistics vector, then become a caught error record.
- Ordinary failed probes retain timings and do not stop sweeps/fuzz/grid sampling.
  Their timings can feed prediction. Ceiling/panic stopping and two-point growth
  prediction omit later work; grid blocking propagates rightward and into later
  rows. Fuzz stops after two consecutive magnitudes hit either a ceiling or panic.
- Unwinding panics are caught around measurement, not all outer registration or
  output code. Aborts/nontermination are not contained, and external side effects
  are not rolled back. No aggregate work, memory, or cancellation contract exists.

Outcome docs distinguish shallow carrier status from semantic completion:
Some(Err(...)), Ok(false), and NaN numeric results succeed. Tuple/vector wrappers
inspect their child statuses, while their reason selection does not identify which
child failed. Short messages retain at most 160 UTF-8 prefix bytes plus a three-byte
ellipsis; prior formatting/allocation is not bounded by that truncation.

RNG docs now describe remainder reduction, zero-bound/seed aliases, reversed-range
behavior, and the full-i64-range saturation omission of i64::MAX. Limbs and decimal
requests at width zero mean one bit. Decimal conversion repeatedly divides the
whole retained limb vector without trimming high zeros. Exact powers/ladder claims
are bounded by binary64 overflow, underflow, rounding, and endpoint conversion.

The serializer trusts its records, maps nonfinite floats to null, retains coarse
rather than typed failure information, and assembles the complete document with
intermediate allocations. finish writes directly to its destination, without
parent-directory creation or atomic publication. Report metadata omits toolchain,
hardware, filter, and reuse settings; it is not a complete reproducibility manifest.

Read all 100 baseline lines of `scripts/perf-atlas/README.md` and all 35 lines of
run.sh for consumer context. Corrected the README's measurement, status, budget,
seed, and callback guidance to agree with the harness. Renderer chart behavior and
archived-run claims remain to be checked against their owners; this batch does not
claim a complete renderer or workflow audit. The shell runner was not executed.
Searches identified nine crate consumers; the previously audited Hyperlimit example
is the representative downstream compile check, not a review of the other examples.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, declaration adjacency, and normalized
  comment-stripped identity passed.
- `cargo test --locked -p perf-atlas --lib --offline -j2` passed both existing tests.
  These cover small timing fixtures and do not establish performance envelopes.
- Direct private Rustdoc passed with warnings, public missing_docs, and broken/
  private links denied, both ordinarily and with `--cfg test`. This dependency-free
  crate was documented directly into separate small /tmp directories to avoid
  overwriting Hyperlimit's same-named example pages in the shared target directory.
- Full-text render checks verified 103 source doc groups across 22 pages normally,
  and 104 across 23 pages with cfg(test). All 16 macro-generated method docs render
  on the Outcome trait page. The local LIMIT and two #[test] functions have source
  docs but no separate pages, even under cfg(test); the test module itself renders.
  No test attributes or lint settings were changed to conceal those limits.
- Hyperlimit's perf_atlas example checks passed with all features and no default
  features after the final source edits. The shared target remains about 1.7 GiB;
  the two standalone doc trees are about 2.7 MiB each. No new Cargo target tree,
  dependency download, release build, or full atlas workload was needed.
- Eight bounded public-API probe reports parsed successfully. They reproduced
  ordinary-error continuation, first-probe-only status, caught zero-sample panic,
  ceiling precedence over failure, fresh/reused factory counts, grid block/error
  propagation, two-magnitude fuzz panic stopping, and escaped JSON labels. Direct
  status/RNG assertions covered nested wrappers, NaN, empty vectors, zero seed/
  width/bound, and reversed ranges. Expected caught panic hooks printed diagnostics;
  the probe process completed successfully. These are documentation witnesses,
  not benchmark results, universal distribution proofs, or geometric qualification.
- Full workflow checks remain owner-waived for this docs-only session.

Evidence: `/tmp/csgrs-rustdoc-perf-atlas-{test,doc,test-doc}.log`, downstream
`/tmp/csgrs-rustdoc-perf-atlas-downstream-{all,minimal}.log`, and
`/tmp/csgrs-rustdoc-perf-atlas-probe.log` with eight corresponding JSON fixtures.
Source/render checkers are `/tmp/csgrs-perf-atlas-{identity,render}.py`. Remaining
shared-atlas work includes renderer, workflow, and historical artifact validation;
repository-wide and generated-API coverage remain open.

## Performance atlas renderer and runner (2026-11-26)

Reviewed every line of `scripts/perf-atlas/render.py` (499 baseline lines), run.sh
(35), README.md (122), and `.github/workflows/perf-atlas.yml` (61), at `8770db430`.
The owning layer is performance-report presentation and orchestration, with P37's
scope/evidence rules and P5/P6/P9's resource, failure, and qualification distinctions.
The input domain is trusted perf-atlas/1 harness documents with the nested records
expected by the renderer. Missing witnesses were helper documentation and accurate
claims about axis transforms, data reduction, failure displays, highlighted scope,
runner behavior, and archived evidence. The workflow was reviewed but not changed.

Added or corrected all 23 Python documentation bodies: module, Scale class, and
21 functions/methods including the nested rows helper. Added comments explaining
constants, scale attributes, five embedded JavaScript helpers and the chart/theme/
filter handlers. Read all embedded CSS/JavaScript and all report templates. The
runner now describes its fixed crate list, sequential invocation, skipped examples,
fail-fast behavior, stale outputs, option failures, and lack of a local call timeout.

Only docstrings, comments, and explicitly enumerated explanatory HTML literals
changed. Python ASTs are identical after removing docstrings, normalizing added
JavaScript comment lines, and applying the recorded prose replacements to the
baseline. Embedded executable JavaScript, CSS, shell commands, chart calculations,
classification, data payloads, and file-selection logic are unchanged. The report
introduction/captions intentionally change to correct claims; full HTML byte identity
is therefore not asserted. Chart-specific output identity is checked separately.

The renderer now documents these source-derived distinctions:

- Time axes are always logarithmic. Magnitude axes automatically use logs only
  for positive finite values with max/min >=16. Tick sampling is count-based;
  the x_ticks scale parameter is unused. Padded time decades can still fail at
  floating extremes even when the original duration is finite and positive.
- Ordinary lines connect retained non-error measurements across gaps. Cliffs and
  valleys compare successive retained ok/single rows, not necessarily adjacent
  original magnitudes. Only ordinary series contribute highlights; fuzz/grid
  failures are excluded from the displayed totals. Lists cap at forty cliffs,
  twenty valleys, and forty errors/ceilings, in their documented sort/order.
- Tooltip timings are rounded to three significant figures before browser
  formatting. Duplicate magnitudes select the last row for line hover but the
  first row for the ordinary table. Summary tables are not lossless exports:
  ordinary min/max/iteration counts and grid error flags are omitted.
- Fuzz medians include measured failures and ceilings; unmeasured panic rows have
  no dot. A missing-duration heatmap error has an empty outline without a cross,
  and its hover still says skipped alongside the error flag. All-None grids have
  no heatmap. These presentation limitations remain in the implementation.
- Heatmap colours reverse in dark mode, so the old universal darker-is-slower
  statement was incorrect. RAMP's length controls bin count; its colour strings
  do not supply the actual CSS fills. Equal cells represent array indices rather
  than metric axis spacing.
- IDs can collide after name normalization; duplicate documents/names are not
  rejected. Only the format tag is asserted at CLI load, and Python optimization
  can disable that assertion. Shapes, numeric domains, metadata, and semantic
  workload comparability are not fully validated. Some top-level metadata is
  interpolated without escaping, so the trusted-input restriction is material.
- Rendering assembles complete strings/data in memory before direct file output.
  Render failures leave an earlier index intact; I/O failures can leave a partial
  new file. No atomic publication or aggregate resource limit was added.

Updated the README and generated explanatory text to match these behaviors. The
existing workflow sets a six-hour job timeout, uses the runner, installs moving
stable Rust, and attempts artifact upload even after failure. Its fixed action
references and arguments were inspected for consistency. Hosted execution, action
implementation, retention enforcement, browser behavior, and hardware conditions
were not qualified here. No CI configuration changed and no CI commands were run.

Historical archive claim retained for provenance, not adopted as measurement:
the prior README described a first full run on 24 November 2026 with 442 operations,
about 110 minutes on a 40-thread Xeon E5-2680 v2, and cited
`docs/evidence/2026-11-24-perf-atlas` in `timschmidt/csgrs-evidence`. The current
local evidence manifest/index search has no matching entry. Read-only GitHub
connector queries confirmed the public repository's default branch is main; that
exact path returned 404 there, and the returned docs/evidence directory listing
had 200 entries with no matching atlas/date name. No archive payload was cloned
or downloaded. Other branches/commits were not exhaustively searched, so this is
an unverified citation, not proof that the artifact never existed. The README now
links here for historical verification status instead of presenting those figures
as established evidence. The large evidence index was consulted only by targeted
search/context reads; it does not receive whole-file review credit in this batch.

Validation after final edits:

- Python syntax/AST checks, complete docstring inventory, shell comment-stripped
  identity, `bash -n`, and `git diff --check` passed.
- Pydoc rendered 22 complete module/class/function docstrings. Function-local rows
  retains its attached source docstring and has no separately reflected Pydoc item.
  This is Python documentation evidence, not additional Rustdoc coverage.
- Bounded probes confirmed anomaly comparisons across failed/missing rows, the
  16x scale threshold, rounded hover data, duplicate first/last selection, fuzz
  aggregate behavior, heatmap missing-error behavior, and ID collision examples.
  Zero-duration heatmaps, tiny decade padding, and huge decade padding reproduced
  their retained ValueError/OverflowError restrictions. No failure was suppressed
  or relabeled as completed support.
- Baseline/current line, fuzz, heatmap, and table output matched byte-for-byte for
  the retained fixture. Complete report output matched after exactly the intended
  explanatory copy and JavaScript comments. No measured data was changed.
- End-to-end runner probes used a temporary cargo stub and the real renderer.
  They checked sequential crate order, release/locked/all-feature flags, forwarded
  quick/filter options, missing examples, unknown/missing options, empty selection,
  fail-fast behavior, retained stale files, and CLI crate sorting. No actual Cargo
  build, benchmark workload, or hosted workflow ran in those probes.
- No browser execution was performed; Node was unavailable. Executable embedded
  JavaScript remained text-identical after removing new comment-only lines. This
  is syntax/source preservation evidence, not a browser interaction test.
- Full workflow checks remain owner-waived for this docs-only session. Shared
  Cargo storage remains about 1.7 GiB; runner fixtures use about 52 KiB under /tmp.

Evidence: `/tmp/csgrs-rustdoc-atlas-renderer-check.log`,
`/tmp/csgrs-rustdoc-atlas-runner-check.log`, and
`/tmp/csgrs-rustdoc-atlas-archive-check.json`. The Pydoc page, synthetic input/report,
AST/render checker, and runner stub/probes are under `/tmp/csgrs-atlas-*`.
Historical artifacts, other atlas workload examples, and the broader repository
remain open; this batch does not establish benchmark or mathematical completeness.

## Hyperlattice projective carriers and scalar guards (2026-11-26)

Reviewed every line of these seven files at baseline `d4522357c`, including
private helpers, test bodies, attributes, fields, and local constants:

| File under `crates/hyperlattice/` | Baseline lines | Current lines |
| --- | ---: | ---: |
| `src/projective.rs` | 559 | 690 |
| `src/scalar.rs` | 64 | 91 |
| `src/error.rs` | 18 | 26 |
| `src/trace.rs` | 13 | 23 |
| `src/aabb.rs` | 30 | 37 |
| `tests/common/mod.rs` | 27 | 40 |
| `tests/scalar.rs` | 264 | 298 |

Owning layer: Hyperlattice coordinate carriers, fixed-dimensional construction
formulas, scalar-domain guards, native result aliases, and trace forwarding.
Relevant contracts include P1/P2/P3/P4/P6/P9/P10/P17/P37: exact meaning, explicit
failure, scalar authority, preserved decision/dispatch order, and non-certifying
structural metadata. The missing acceptance witness was accurate documentation
of carrier validity, scale identity, domain checks, and the limits of existing
regressions; many private/test declarations lacked attached Rustdoc. No geometric
capability, certainty receipt, or resource bound was added.

The source inventory now finds comments on all 80 authored declarations and ten
fields in these seven files, including both feature alternatives of the trace
macro, trait/implementation methods, test helpers, 23 tests, and two function-local
constants. Imports/re-exports inherit the original API documentation; generated
compiler/derive items are not counted as authored declarations. This is a bounded
source inventory and semantic review, not repository-wide coverage.

Contracts clarified:

- Plane/point/line carriers accept degenerate coefficients. Projective points and
  lines require nonzero tuples; Pluecker lines also require direction·moment=0.
  Those conditions are not validated by constructors. Coefficientwise Real
  equality is not equality of geometric images up to a common nonzero scale.
  Public mutation changes current facts without transporting incidence proofs.
- Three-plane intersection constructs signed cofactors. Sparse rational and
  expanded paths retain the same tuple, not just a proportional representative.
  Rank below three yields the invalid all-zero tuple; rank-three output can have
  zero weight and represent a point at infinity. Two-plane and line-plane formulas
  likewise construct algebraic carriers without proving a unique finite result.
- Homogeneous incidence returns an expression, not a decision. Invalid all-zero
  inputs also satisfy it. Representative scale affects its sign, so it is not an
  affine sidedness classification. Coordinate fact sets do not validate geometry.
- Affine conversion first handles all-rational unit weight, then all-dyadic
  quotients, then general borrowed division, with x/y/z order unchanged. General
  division can require scalar nonzero reasoning. Failure propagates before a
  Point3 is returned; no independent work budget or certainty receipt is supplied.
- The definite-zero guard admits unknowns, whereas the known-nonzero guard maps
  Zero to DivideByZero and Unknown to UnknownZero without starting refinement.
  Native inversion can resolve the near-zero sine fixture that the checked
  reciprocal rejects. The fixture is mathematically negative/nonzero; its name
  describes unavailable cheap knowledge rather than mathematical equality to zero.
- Abort attachment shares a cooperative flag and changes scalar payload/cache
  state; it neither polls immediately nor preempts all exact shortcuts. Result
  aliases are the native scalar Result type, not stronger checked/certainty types.
- Aabb stores unsorted, unvalidated corners. Origin creates coincident zero bounds,
  not a canonical empty sentinel. Enabled trace arguments are evaluated even when
  recording is inactive; the disabled macro erases their evaluation entirely.
- The projective proportionality helper's six mixed minors also vanish for an
  all-zero tuple against any tuple. Tests requiring validity need a separate
  nonzero assertion. Existing sparse fixtures supply one where documented; the
  degenerate fixture explicitly expects zero. The wide dyadic comparison shares
  the scalar implementation and does not constitute an independent arithmetic or
  performance proof. The parallel fixture only asserts zero weight and an error.

Context reads followed these claims into Hyperreal's rational/fact, inversion,
abort, computable zero-knowledge, and dispatch implementations, Hyperlimit's
homogeneous adapter, and Hypermesh's bounds consumer. Those were targeted excerpts,
not new whole-file reviews. The complete Hyperlattice manifest and root library
were read for exports/features; root API claims remain subject to the unreviewed
implementation modules. Point, README, geometric/API-surface tests, and the trace
benchmark received context reads only, not whole-file completion credit.

Private Rustdoc initially failed on six existing redundant explicit links across
`kernels.rs`, `complex.rs`, `vector.rs`, `matrix/core.rs`, and feature-gated
`arbitrary_impls.rs`. Inspected their module/import context and removed only the
redundant destinations. The resolved targets remain the same. These five files
remain open for whole-file semantic review; no lint was disabled.

Validation after documentation edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical source in all twelve changed Rust files against the
  baseline. No executable layout, branch, predicate call, assertion, fixture,
  feature, visibility, or lint policy changed.
- Hyperlattice library builds succeeded with no default features and all features,
  using locked/offline Cargo, two workers, one shared target, and no incremental
  storage. Private library Rustdoc passed with warnings and broken/private links
  denied in both configurations. Full normalized text of 56 documentation groups
  on 31 pages matched the reviewed source in each configuration, including the
  corresponding trace macro alternative.
- Separate private `cfg(test)` documentation rendered 15 complete additional
  groups across 14 pages: test-module documentation, seven projective helpers,
  scalar-test overview, and common fixtures. Rustdoc omits individual `#[test]`
  function pages and the two function-local constants: all 25 retain reviewed
  source comments. Test attributes were not changed to manufacture rendered pages.
- The seven existing projective tests and sixteen scalar integration tests passed
  in both feature configurations. A temporary rustc harness includes the actual
  projective source unchanged and links actual Hyperlattice/Hyperreal libraries;
  scalar integration tests compile directly from their original source. This
  avoids unrelated Criterion/Rug/Symbolica development dependencies. The harness
  is focused validation, not execution of the full crate test suite.
- Hyperlimit's downstream library checks passed with no default features and all
  features. No dispatch-count or performance claim is inferred from these tests;
  executable-source identity preserves the existing dispatch statements.
- Initial harness-only unused-public-item warnings were removed by exposing the
  included module in the temporary harness. A subsequent feature-switch attempt
  found that Cargo's top-level rlib was replaced by the other configuration;
  rebuilding and testing each configuration sequentially resolved that harness
  setup issue. Final focused test/build/doc logs have no warnings or failures.
- Full workflow checks remain owner-waived for this docs-only session. The shared
  Cargo target is about 1.9 GiB. Temporary test binaries are removed after checks;
  documentation, source harnesses, and logs remain under /tmp for review.

Evidence: `/tmp/csgrs-rustdoc-lattice-{min,all}-{build,doc,tests,render}.log`,
`/tmp/csgrs-rustdoc-lattice-downstream-{min,all}.log`, and
`/tmp/csgrs-rustdoc-lattice-test-{docs,render}.log`. Cargo artifact manifests and
checkers are `/tmp/csgrs-rustdoc-lattice-*-build.json` and
`/tmp/csgrs-lattice-*.py`; private test docs are under
`/tmp/csgrs-lattice-harness/doc`. Temporary evidence is not a durable repository
artifact. Hyperlattice's remaining implementations, tests, benches, support files,
and the broader codebase remain open. The root AGENTS.md public/private coverage
mandate remains in force.

## Hyperlattice Cartesian points and fact snapshots (2026-11-26)

Reviewed all 777 baseline lines of `crates/hyperlattice/src/point.rs` at
`c29a3b817`, including six structs, fields, trait implementations, operators,
private bit/fact helpers, and both unit tests. The file now has 978 lines. All
104 authored declarations (81 functions/methods, sixteen associated output types,
six structs, and one test module) and thirty fields have attached documentation.
Compiler-generated derive members are not counted as authored declarations.

Owning layer: Hyperlattice's Cartesian coordinate carriers and structural fact
snapshots. Relevant properties are P1/P2/P3/P4/P6/P9/P10/P17/P37: exact coordinate
meaning, conservative facts, cache scope, failure vocabulary, preserved dispatch
order, explicit equality criteria, and consumable API documentation. Missing
acceptance witnesses included private/trait item comments and accurate domains
for shared-denominator eligibility, one-hot support, mask dimensions, collection
failure, and float conversion. This remains a documentation-only change.

Reproduced the following on the baseline before editing:

- Shared-scale views require a nonempty set of recognized exact rationals with
  equal reduced denominators. `(1/3,2/3)` qualifies; `(1/2,1/4)` and `(0,1/3)` do
  not. The constructor does not rewrite values onto a mathematical common
  denominator or resolve a symbolic value into a rational representation.
- Public view metadata can be changed while its borrowed coordinates remain the
  same. Detached fact packets are likewise freely constructible/editable. Counts
  use every set bit, including bits outside the point dimension; other queries
  trust stored flags/index fields without rechecking coordinates or consistency.
  These carrier APIs alone do not make user-supplied metadata a proof.
- `Point3(0,-7,0)` is one-hot with axis index 1 and one_mask 0. One-hot means one
  known nonzero coordinate, not a coordinate equal to one. `(0,2,3)` does not meet
  has_sparse_support's zero-or-single-coordinate shortcut despite being sparse in
  the ordinary broader sense. Family dependency flags are not correlated-root IDs.
- N=0 view construction returns None. A manually constructed zero-dimensional
  facts packet with zero masks reports both known-zero and known-dense. N=128
  all-zero input succeeds with all 128 mask bits set. Eligible N=129 input panics
  during the shift in the tested overflow-checked build. The public generic type
  has no enforced size bound or typed oversize error. Documenting this existing
  implementation restriction does not fix it or establish P6 conformance; release
  overflow-disabled behavior was not qualified, and no assertion was weakened.
- Float arrays import exact binary values; `0.1_f64` differs from exact `1/10`.
  NaN/Infinity return their native scalar error variants. Interpolation accepts
  parameters outside [0,1], and weighted sums accept negative/non-unit weights
  without normalization. Empty/mismatched weighted collections return None.

The comments also explain ownership and delegation: point-to-vector consumption
moves coordinates, while consuming a vector into a point currently clones its
lanes. Translation returns a point; ordered point subtraction returns a displacement.
Collection methods clone points into an allocated vector collection before the
vector implementation validates shape. Centroid uses an ordered sum and integer
count; its delegated Option path also collapses count-conversion/division failures.
Lossy exports stop at the first missing/nonfinite result and carry no certified
geometric error or topology guarantee. Detached facts require recomputation after
coordinate mutation; scalar cache changes can strengthen later observations.
No arithmetic, call order, ownership path, or failure behavior was changed.

Context inspection followed the actual vector lerp/mean/weighted_sum methods,
Hyperreal exact-set eligibility and scalar float conversion/fact methods, and the
Hyperlimit re-exports/plane fact usage. Those files received targeted reads only.
Read the complete point-specific API-surface test and its imports for validation;
the rest of that integration file remains unaudited. The test calls several
operator/fact methods without asserting their results; execution alone is not a
semantic oracle for those calls. The two unit tests also use lossy float assertions,
which are now explicitly distinguished from exact coordinate proofs.

Validation after final edits:

- Comment/blank-line removal produces byte-identical remaining source against
  `c29a3b817`; no executable line, fixture, attribute, assertion, or policy changed.
  Focused Rustfmt and `git diff --check` passed.
- Hyperlattice library builds and private Rustdoc passed with no default features
  and all features. Rustdoc denied warnings and broken/private intra-doc links.
  Initial bracketed formulas were interpreted as links; code formatting corrected
  them without suppressing lints. Full normalized text of 131 source documentation
  groups matched fourteen item pages in each configuration, including From impl
  documentation rendered on the vector pages.
- Separate private cfg(test) Rustdoc rendered the test module and private `real`
  fixture. Both #[test] functions retain reviewed source comments but have no
  individual item pages under Rustdoc. No test attributes or visibility changed
  in the repository to influence that result.
- Both original point unit tests and the selected original point API-surface test
  passed in each feature configuration. A temporary harness includes the actual
  point source unchanged and links actual scalar/vector libraries; the original
  integration source is compiled directly, with only its point test selected.
  Unrelated development dependencies and tests were not built/run through Cargo.
- Bounded independent exact expectations reproduced denominator eligibility,
  editable metadata, N=0/128/129 behavior, nonunit one-hot support, binary float
  ingress/errors, extrapolation, and weighted-sum shape/sign behavior before and
  after edits. The N=129 panic is an observed implementation gap, not geometric
  success or resource-exhaustion evidence. The probe does not certify arbitrary
  symbolic inputs or general completeness.
- Hyperlimit downstream library checks passed with no default features and all
  features. Full workflow checks remain owner-waived for this docs-only session;
  no benchmark, dispatch-count change, or performance improvement is claimed.
- Used one shared Cargo target with incremental compilation disabled and two
  workers, locked/offline. Temporary test/probe executables are deleted after
  validation; source probes, rendered docs, and logs remain available under /tmp.

Evidence: `/tmp/csgrs-rustdoc-point-{min,all}-{build,doc,tests,render}.log`,
`/tmp/csgrs-rustdoc-point-downstream-{min,all}.log`, and
`/tmp/csgrs-rustdoc-point-test-docs.log`; Cargo artifact manifests are alongside
these logs. Scripts are `/tmp/csgrs-point-*.py`, the exact probe source is
`/tmp/csgrs-point-probe.rs`, and private test docs are under
`/tmp/csgrs-point-harness/doc`. These temporary artifacts are not a durable evidence
archive. Remaining Hyperlattice modules/tests/support files and the wider
repository still require review; this entry does not claim complete crate coverage.

## Hyperlattice 2D algebra and private scalar adapters (2026-11-26)

Reviewed every baseline line of `crates/hyperlattice/src/algebra2.rs` (615) and
`crates/hyperlattice/src/kernels.rs` (250) at `fab9b68b3`. Current lengths are 625
and 368. This includes public fact fields, all arithmetic formulas, trait and
implementation methods, private helpers, enum variants, test alias/helper, and
all nine unit tests. These two whole-file reviews supersede the earlier
module-link-only context record for kernels.rs.

Owning layer: lattice coordinate algebra and scalar scheduling adapters, with
Hyperreal retaining scalar authority. Relevant properties include P1/P2/P3/P4/P6/
P9/P10/P17/P37: exact construction, conservative facts, cache/proof scope, explicit
failure and certainty boundaries, preserved call order, and consumable docs.
Missing acceptance evidence was attached private/implementation/test documentation
and precise distinctions between zero knowledge, expression construction, masks,
counts, and unchecked representation assumptions. No implementation or public
contract was changed to resolve the documented limitations.

Source inventory: algebra2 has 54 authored declarations (48 functions/methods,
four structs, one test module, one alias) and ten fields. Kernels has 33 authored
declarations (31 functions/methods, one enum, one trait) and three enum variants.
All have attached comments. Compiler-generated items are not counted as authored
source. Scope is these files, not the entire crate.

Contracts documented and checked against the target revision:

- Displacement uses to-from in x/y order. Wedge is the oriented parallelogram
  determinant, orientation is wedge(b-a,c-a), and squared distances/norms avoid a
  square root. These functions construct Real expressions, not sign decisions,
  topology/incidence certificates, or predicate certainty receipts. Positive sum
  means all addition signs, not necessarily a nonnegative scalar value.
- Product facts query both factor zero_status values, in left/right order, even
  when the first factor is zero. Zero dominates, two NonZero factors imply a
  NonZero product, and the other cases stay Unknown. No product is constructed.
  Existing scalar proof caches can affect later snapshots without a new local
  refinement loop. Orientation facts query displacement facts first, then revisit
  component knowledge for the determinant terms.
- Two zero determinant terms imply zero; one zero and one nonzero imply nonzero;
  two nonzero terms remain unclassified by support alone. A dense rational
  collinear example produces known_zero=None while its exact wedge is zero.
  Unknown is not a false classification, and known-nonzero gives no orientation
  sign. Axis pairs need not be different.
- All fact fields are public. Query methods trust their stored flags/statuses
  and do not bind them to original coordinates or validate consistency. Edited
  product/determinant flags can contradict the underlying factor packet. This
  metadata carrier is not itself a validator of caller-provided evidence.
- Product-sum masks inspect only indices below min(TERMS,64). Counts inspect the
  full array but cast usize to u32; beyond u32::MAX the count truncates, and the
  all-zero method cannot recognize a larger all-zero array. Empty packets report
  all terms zero. The 65-term mask/count distinction was exercised; counts above
  u32::MAX were established from the source conversion, not by allocating a
  multi-billion-entry fixture. This restriction is not a completeness claim.
- The pairwise expression builder uses definitely_zero, a narrower structural
  query than zero_status. It short-circuits the right-factor query when the left
  is definitely zero, yet scans all supplied terms. Its retained count includes
  unknown/unrecognized zeros. Zero/one/two retained terms get specialized paths;
  three or more forward the original array, including skipped zero terms.
- The active pairwise reducer tries the exact-rational aggregate, then performs
  products and signed accumulation in original order. It has no separate local
  pruning or identity search. It is not Hyperreal's generic signed_product_sum
  recognizer. Individual scalar operations may still simplify their inputs.
- Active means retained for arithmetic, not certified nonzero. Known-exact and
  known-dyadic adapters require current representation evidence; they bypass
  factor-class validation and forward to routines that consume stored rational
  components. Incorrect or stale evidence is not reported as a typed mismatch.
  No invalid-precondition probe was treated as supported operation success.
- Cached arithmetic methods reuse a borrowed operand but maintain no lattice
  cache. Dot and linear-combination wrappers delegate directly to Hyperreal.
  Self-dot wrappers pass identical references, permitting scalar-owned reuse but
  guaranteeing neither a hit nor a measured speedup. NonRational means no
  recognized rational representation, not proved mathematical irrationality.

Context reads followed the scalar zero_status/definitely_zero implementations,
exact-rational aggregate/unchecked entry points, dot/linear-combination delegation,
trace recorder, Axis2 index/bit mapping, and consumer references. Those excerpts
receive no whole-file completion credit. Read the complete algebra-specific
API-surface test and relevant imports, but the rest of that file remains open.
Existing performance commentary encountered in scalar context is not adopted as
new measurement evidence here.

Validation after documentation edits:

- Removing comment-only/blank lines leaves identical executable source in both
  files against `fab9b68b3`. Focused Rustfmt and `git diff --check` passed. No
  predicate ordering, expressions, branch conditions, assertions, visibility,
  features, fixtures, or lint policies changed.
- Hyperlattice library builds and private Rustdoc passed with no default features
  and all features, with warnings and broken/private intra-doc links denied.
  Full text from 90 source comment groups matched 23 item pages. Fifteen trait
  and implementation method comments share identical text: these are 75 distinct
  rendered texts, not 90 independently rendered items. The trait page renders a
  representative repeated method summary once; implementation comments also
  remain attached in source. Normal-source coverage is not inferred merely from
  Rustdoc's public missing_docs lint.
- Separate private cfg(test) docs rendered three additional groups: test module,
  scalar alias, and fixture helper. Nine #[test] functions retain source comments
  but no individual Rustdoc item pages. No attributes were changed to alter this.
- Nine original unit tests, one selected original API-surface integration test,
  and a separate bounded contract probe passed at baseline and after edits.
  Final runs passed in both feature configurations. The temporary harness
  includes the original algebra/kernel source unchanged and links actual scalar
  libraries; the integration test compiles directly from its original source.
- The probe checks all nine factor-status combinations, empty and 65-term facts,
  dense cancellation, edited metadata, signed/positive sums, each private kernel
  wrapper, and rational/dyadic/unrecognized representation tags using exact
  expectations. With tracing enabled, a zero-left product fact records two Real
  zero_status calls while a zero-left one-term expression records one
  definitely_zero call. The same assertions passed before and after edits; this
  is a bounded call-order witness, not the entire dispatch matrix or a benchmark.
- Hyperlimit downstream library checks passed with no default features and all
  features. Full workflow checks remain owner-waived for this docs-only session.
  Used locked/offline Cargo, two workers, one shared target, and no incremental
  storage. Temporary test executables are removed after checks; no performance
  improvement or arbitrary-degree/resource completeness is claimed.

Evidence: `/tmp/csgrs-rustdoc-algebra-baseline.log`,
`/tmp/csgrs-rustdoc-algebra-{min,all}-{build,doc,tests,render}.log`,
`/tmp/csgrs-rustdoc-algebra-downstream-{min,all}.log`, and
`/tmp/csgrs-rustdoc-algebra-test-docs.log`. Cargo artifact manifests and
`/tmp/csgrs-algebra-*.py` scripts support those checks; harness/probe source and
private test docs remain in `/tmp/csgrs-algebra-harness`. Temporary evidence is
not a durable archive. Remaining implementations, tests, and support files across
Hyperlattice and the repository still require review.

## Hyperlattice generators and fuzz-target contracts (2026-11-26)

Reviewed every line of the following Rust files at `4582c67ca`, including input
fields, manual/derived generators, complete callbacks, discarded operations,
assertions, helpers, and existing explanatory comments:

| File under `crates/hyperlattice/` | Baseline lines | Current lines |
| --- | ---: | ---: |
| `src/arbitrary_impls.rs` | 103 | 143 |
| `fuzz/fuzz_targets/scalar_ops.rs` | 205 | 241 |
| `fuzz/fuzz_targets/complex_ops.rs` | 193 | 213 |
| `fuzz/fuzz_targets/vector_ops.rs` | 225 | 249 |
| `fuzz/fuzz_targets/matrix_ops.rs` | 462 | 497 |
| `fuzz/fuzz_targets/hyperreal_representations.rs` | 275 | 324 |

Also read the complete fuzz Cargo manifest (48 lines), README (41), run guide (6),
and .gitignore (6). Manifest and ignore rules remain unchanged. Rewrote the README
and short run guide around the demonstrated input/oracle scope and explicit
monorepo commands. The earlier arbitrary_impls module-link-only record is now
superseded by whole-file review.

Owning layers: feature-gated lattice seed generation and downstream fuzz consumers.
Relevant contracts are P1/P3/P6/P8/P9/P10/P17/P37: exact scalar ingress, conservative
knowledge, honest failure/equality claims, retained certainty, and reproducible
consumer documentation. Missing witnesses were undocumented private inputs/helpers
and inaccurate statements about interval arithmetic, private-class coverage,
equality oracles, cancellation, and resource bounds. The change documents current
behavior; it does not implement stronger oracles or resource enforcement.

The six Rust files now document all 31 authored declarations and 22 fields.
Fuzz callback bodies are described by their enclosing module documentation.
LibFuzzer entry functions and derive-generated methods are not independently
attached authored items and are not counted toward this source inventory.

Findings and preserved limitations:

- Each library coordinate generator selects a sanitized finite binary64 or i128.
  The former maps nonfinite bit patterns to positive zero before exact Real
  import. Seeds are dyadic rationals/integers, including finite-float extremes,
  not general symbolic/algebraic or non-dyadic sources. This sanitization is a
  fuzz distribution, not the public scalar conversion policy. Mathematical
  floating zero remains scalar zero, regardless of its primitive sign bit.
- The six aggregate Arbitrary implementations have default (0,None) size hints.
  With arbitrary 1.4.2's zero-padding integer decoder, empty input yields zero
  complex/vector/matrix values. The four manual fuzz Input decoders likewise
  have a zero lower size hint. The derived representation Input has lower hint
  eight, which libfuzzer-sys checks before invoking its callback. Neither shape
  implies uniform sampling or a total downstream work bound.
- Scalar powers use a fixed list plus the full input i8, and general powers use
  half-integers from -4 to 4. Vector/matrix abort flags remain false. Many Results
  are discarded; executing those calls would check panic resistance rather than
  successful computation or the correctness of every returned error.
- Cancellation assertions that reject only NonZero accept Unknown. Complex
  exponent-zero assertions can skip errors. Matrix schedule-helper equivalences
  do not independently prove the shape-driven and exact-rational-driven classes
  disjoint. Fact helper assertions test internal consistency, not coefficients
  against an independent classification oracle. Legacy assertion diagnostic
  strings remain unchanged; misleading ordinary comments and module claims were
  corrected without editing the assertions themselves.
- The representation target creates 22 fixed recipes with an ordered public-kind
  assertion spanning eight categories, then four opaque recipes. It does not
  assert 22 distinct private classes or exhaust arbitrary expressions. Nonzero
  scaling occurs after kind assertions. Stride modulo 26 makes all ordered recipe
  pairs reachable across inputs, not covered in every execution or campaign.
- Opaque construction performs one to eight updates, including exp and squaring.
  Finite recipe depth does not bound evaluation work, coefficient size, expression
  cost, memory, or cancellation latency. Existing expect messages describing
  bounded values are not resource certificates and were not altered.
- assert_real_equal returns on CertifiedRealEquality::Equal; every other result
  falls through to interval overlap at precision -128. Overlap is not equality.
  A bounded probe confirmed the helper accepts pi and pi+2^-256 after a non-Equal
  equality result, while a separate certified equality proves their difference
  equals the positive exact step. This is an existing oracle limitation, not an
  exact success or a newly introduced approximation policy. Complex/point helper
  calls inherit it. No fixture or assertion was weakened to obtain this result.
- The old Apache-2.0 README reference contradicted Hyperlattice's manifest and
  local MIT LICENSE; the README now links the actual license. Retained benchmarks
  run separately implemented target/seed recipes, not raw corpus or full fuzz
  callbacks. Their measurements and campaign coverage were not certified here.

Context reads included arbitrary's integer, ratio, zero-padding, and default
size-hint implementation; libfuzzer-sys's typed minimum-length gate; scalar exact
conversion/enclosure behavior; and retained_fuzz.rs lines 1–90 showing its separate
recipes. These dependency/consumer excerpts and the LICENSE header receive no
additional whole-file completion credit. No external source or benchmark result
was substituted for target-revision evidence.

Validation after final edits:

- Comment/blank-line removal leaves identical remaining source in all six Rust
  files against `4582c67ca`. Assertions, diagnostic strings, input distributions,
  exponents, operations, feature configuration, and predicate order are unchanged.
  Focused Rustfmt and `git diff --check` passed.
- The all-feature Hyperlattice library built and generated private Rustdoc with
  warnings and broken/private links denied. The actual fuzz dependency configuration
  (no lattice defaults plus arbitrary) compiled all five bins. Explicit per-bin
  cargo rustdoc commands succeeded despite ordinary manifest doc=false selection.
  Full text of 59 source documentation groups matched 33 item pages, including
  private inputs/helpers and aggregate trait implementation docs. Generated macro
  entry items are not claimed as independently documented by that count.
- Generator probes checked six empty-input zero aggregates and default hints,
  eight primitive float classes/extremes, four i128 cases, matrix row-major order,
  and vector lane order. They link the real library and arbitrary dependency.
- A temporary probe copied the actual representation Input and four helper bodies
  unchanged. It checked the eight-byte lower hint, all 22 ordered public-kind
  assertions, one four-opaque-value graph fixture, the rational denominator endpoint,
  and the unequal-overlap example above. An initial probe mistakenly compared
  the difference using Real structural PartialEq; it exposed distinct computable
  and rational carriers. The final probe uses certified equality for that value
  claim, preserving the distinction between structural and mathematical equality.
- No libFuzzer campaign, complete callback execution, sanitizer campaign, benchmark,
  or extreme graph-evaluation stress run occurred. Compilation/rendering and the
  bounded probes do not establish fuzz completion or general operation closure.
- Full workflow checks remain owner-waived. Used locked/offline Cargo, two workers,
  shared storage, and disabled incremental compilation. The absent standalone
  fuzz lockfile was generated offline for validation (arbitrary 1.4.2,
  libfuzzer-sys 0.4.12), copied to /tmp as evidence, then removed from the worktree.
  No dependency manifest or lockfile is committed. Temporary probe binaries are
  removed after validation; the shared target is about 2.0 GiB.

Evidence: `/tmp/csgrs-rustdoc-lattice-generators-{build,doc,probe}.log`,
`/tmp/csgrs-rustdoc-lattice-fuzz-{check,render,representation-probe}.log`, and
`/tmp/csgrs-rustdoc-lattice-fuzz-{scalar,complex,vector,matrix,representations}-doc.log`.
Generator build metadata and `/tmp/csgrs-lattice-generators/` retain the checker,
probe sources, and temporary fuzz lockfile. These are temporary review artifacts,
not a durable evidence archive. Other Hyperlattice implementations, tests,
benchmarks, and support files, and most of the wider repository remain open.

## Hyperlattice allocation observer and coverage wrappers (26 November 2026)

Whole-file review on `caf926a6f` covered every line of
`crates/hyperlattice/examples/allocation_profile.rs` (288 → 409 lines),
`scripts/allocation_profile.sh` (8 → 15),
`scripts/representation_coverage.sh` (19 → 23), and `scripts/coverage.sh`
(201 → 220), with script paths relative to that crate. Added accurate module,
private allocator/counter/guard/statistics/helper Rustdoc and shell comments.
The Rust file has 25 authored const/static/struct/function declarations and six
fields. All have documentation; its 32 documentation groups include the module.
No implementation, output string, assertion, command, AWK program, or feature
selection changed.

Owning layer: development allocation instrumentation and execution-coverage
reporting. Relevant properties are P4 (cache scope), P5 (resource limits),
P36 (same completed workload and complete-lifetime memory evidence), and P37
(no overclaiming). The input domain is a positive usize iteration count over
22 fixed scalar recipes, plus shell environment/toolchain configuration and
LLVM text reports. The missing acceptance witness was accurate documentation
of private measurement machinery and the distinction between its counters and
complete memory evidence. Earlier text asserted steady-state/all-class coverage
and generalized execution/feature coverage beyond what the source establishes.

Reproduced semantics before editing the example:

- The allocator delegates to System. Successful allocations/reallocations count;
  realloc traffic adds the full new size, while live bookkeeping adds/subtracts
  only the size difference. Frees are counted regardless of allocation origin.
  Live balance resets to zero and saturates on subtraction. It cannot represent
  negative deltas, account for pre-existing storage, or observe transient storage
  inside System::realloc. No RSS, allocator-overhead, or total-memory claim follows.
- Six independent relaxed loads do not form a coherent concurrent snapshot.
  Reset/enable does not quiesce other threads. Guards are not nestable: dropping
  an inner guard disables the outer one's gate. Counter growth is unchecked;
  overflow and concurrent stress were reviewed but not executed in the allocator.
- Each row warms once, then includes counted result destruction. Closure fixtures
  precede the window; closure-environment destruction follows the explicit gate
  drop. One warm-up is not steady-state evidence. The private zero-iteration path
  still warms; main rejects zero and parses only the first argument.
- The 22 labels are recipes, not verified private class tags. The example asserts
  only their count. Four rows per recipe measure complex self-product, combined
  vector dot/cross, Matrix3 determinant, and Matrix4 affine point transformation.
  Printed inline type sizes exclude heap referents. There is no independent exact
  geometric oracle or certainty receipt. Peak/end values are whole-window balances;
  the other columns are per-iteration averages.
- The allocation wrapper inherits Cargo/Rust environment, chooses release/all
  features, reads iterations from an environment variable with unset/empty default
  128, and ignores positional arguments. The representation wrapper stops at the
  first failed command and selects four explicit cache/feature configurations;
  these are not every lattice-feature/cache combination.
- The coverage wrapper clears previous coverage files and the selected target's
  html contents before building. Relative target overrides are crate-relative.
  It selects no-default/all-feature tests and all-feature Cargo test targets;
  compiling an example harness need not execute main. Its environment variable
  requests report suppression only from targets that honor it. Its path filter
  does not exclusively select this crate's source.
- The production-line table is an AWK heuristic: standalone cfg(test), optional
  blanks, then mod tests. Intervening docs/attributes and compound cfg forms defeat
  recognition; later production after a recognized boundary is also excluded.
  The no-boundary sentinel excludes displayed line 999999 and above. Nonempty
  displayed count fields contribute, and anything except literal zero counts hit.
  There is no threshold or total-zero guard. On this host, empty input printed
  `TOTAL ... -nan%` and exited successfully. This is an existing reporting
  limitation, not Rustdoc coverage or correctness evidence.

Targeted context corrections in README's validation-tooling section and
PERFORMANCE's representation/coverage/allocation section now describe these
limits and the already-reviewed fuzz helper's overlap fallback. Those two files
receive **partial context review only**, not whole-file credit. Their adjacent
integration-test and benchmark claims remain for the corresponding full audits.
Feature-default manifest searches and the relevant property/admission passages
were context reads, not new whole-file completion claims.

Validation after final edits:

- Focused Rustfmt, individual `bash -n` checks for all three scripts, and
  `git diff --check` passed. Removing comment/blank lines gives identical remaining
  Rust and shell source against `caf926a6f`, including the embedded AWK program.
- Direct rustc compiled the actual example inside a temporary observer module
  against the previously built all-feature libraries. Before/after probes passed:
  allocation 8, resize 24, resize 4, free yielded 1 allocation, 2 reallocations,
  1 free, 36 traffic bytes, 24 peak bytes, zero end balance; freeing a pre-window
  32-byte allocation counted a free with zero peak/end balance; two measured
  16-byte allocations/drops plus one uncounted warm-up yielded the expected three
  calls and measured traffic/balances; nested-guard interference was confirmed.
- The original main completed all 88 rows at one counted iteration plus warm-up
  per row. Its complete printed report matched the pre-edit report byte for byte.
  This bounded debug invocation is neither a release benchmark nor a geometry
  qualification or proof of behavior at every iteration count.
- Private Rustdoc generation for the actual example passed with warnings and
  broken/private links denied. Complete normalized text of all 32 documentation
  groups matched generated content across 21 pages, including unsafe allocator
  methods, private statistics fields, helpers, and guard behavior.
- Actual allocation/representation wrappers ran against a temporary Cargo logging
  stub: verified crate cwd, default/empty/custom/invalid iteration passthrough,
  ignored positional arguments, all four exact configuration commands, and
  second-command fail-fast. This checks orchestration, not those Cargo workloads.
- The actual extracted AWK program ran against synthetic source/report fixtures:
  recognized boundary, later-production exclusion, docs/extra-attribute/compound
  cfg nonrecognition, sentinel exclusion, and empty-input behavior. No full LLVM
  coverage pipeline, cleanup on the shared target, or report-format compatibility
  claim was exercised.
- Full workflow checks remain owner-waived. No release profiler run, full
  representation matrix, benchmark suite, or coverage pipeline ran. No geometry
  implementation changed, so downstream geometry checks were not repeated.
  Reused the shared approximately 2.0 GiB target without new Cargo build trees;
  removed the temporary observer executable after validation.

Evidence: `/tmp/csgrs-rustdoc-lattice-allocation-{before,after,validate}.log`
and `/tmp/csgrs-lattice-allocation/` contain source probes, before/after report
text, the checker, synthetic AWK inputs/output, and generated private docs.
These are temporary evidence, not a durable archive. The other Hyperlattice
examples, implementations, tests, benchmarks, and support files remain subject
to their individual review records; most of the wider repository remains open.

## Hyperlattice quick start and stored benchmark report (26 November 2026)

Reviewed every line at `c749b8f4a` of these files under `crates/hyperlattice/`:

| File | Baseline lines | Current lines |
| --- | ---: | ---: |
| `examples/readme_quickstart.rs` | 16 | 29 |
| `examples/write_benchmarks_md.rs` | 15 | 29 |
| `tests/readme.rs` | 41 | 56 |
| `benches/support/benchmark_report.rs` | 473 | 635 |

Added all missing module/declaration/field Rustdoc: 41 authored declarations
including test modules/functions, 17 fields, and four module introductions.
The README's marked quick-start block was synchronized with the entire example,
including its documentation, as required by its unchanged byte-comparison test.
That README block is partial context review, not whole-file README completion.
No implementation, output string, assertion, predicate order, lint attribute,
feature selection, or benchmark result changed.

Owning layers and acceptance witnesses:

- Quick start: exact scalar ingress and lattice arithmetic demonstration, P1/P2,
  P3's structural-versus-semantic equality distinction, P6's native error path,
  and P37's documented consumer. The domain is the fixed integer vector (3,4,0)
  and 3x3 identity matrix. Missing private helper/main docs and README synchronization
  were the acceptance gap. The original example and both README tests passed before
  edits. The assertions check self-dot 25, principal square root 5, identity action,
  and identity inversion, using structural PartialEq for exact rational results.
  BlasResult propagates native scalar errors and adds no certainty receipt.
- Reporting: stored development measurements and manifest/source inventory, P5's
  resource-scope distinction, P27's output determinism, and P36/P37's equivalent
  completed workload and no-overclaiming requirements. Input is stored Criterion
  JSON, directories, environment/argv, a line-scanned manifest, and Markdown.
  Missing private parsing/grouping/replacement docs and their rendered coverage
  were the gap. Two original helper tests and an actual-source synthetic probe
  passed before editing. The writer records stored evidence, not completed or
  semantically equivalent current-revision workloads.

Report behavior documented from the target revision:

- CRITERION_HOME is used verbatim, including relative paths; a relative
  CARGO_TARGET_DIR is joined to the compile-time crate root. Otherwise executable
  ancestry selects an existing criterion directory, with a crate-target fallback.
  Recursive directory traversal follows symlink directory tests and lacks
  crate/revision filters, cycle detection, and work/depth/memory limits.
  Duplicate full IDs replace earlier visits; filesystem enumeration determines
  the winner even though output IDs are sorted.
- Missing required files/fields skip rows; required read/JSON errors propagate.
  Optional change-file errors are suppressed. Mean confidence bounds and median
  default to the mean, while the table always labels intervals 95% without reading
  their confidence level. Invalid throughput counts become zero; throughput text
  is workload size, not rate. These are existing reporting semantics, not statistical
  validation or approval of the underlying performance claims.
- Explicit bench inventory is a line scan, not TOML parsing or automatic target
  discovery. Features before the name are missed; # stripping ignores string
  context; multiline arrays, custom source paths, and general escaping are not
  supported. Conventional source substring detection can classify a comment as
  Criterion. This manifest currently has seven explicit benches. The existing
  inventory test checks seven, two names, and some non-Criterion classification,
  not every name/feature/source classification.
- Comparisons require distinct nonempty function IDs sharing exact group/input
  strings, with nonempty input or a competitive/comparison group-name substring.
  Participating-row count is not a count of pairs or groups. Ratios have no
  positive-time check or proof of equivalent semantics. A zero fastest row yielded
  `infx` and `NaNx` in the bounded synthetic probe. Duration/change/Markdown helpers
  format their inputs rather than validate measurements or sanitize arbitrary text.
- Automatic callbacks inspect exact argv strings and skip-variable presence,
  ignore the supplied Criterion object, and log write failures without failing
  the benchmark. The standalone example bypasses that gate and propagates I/O
  errors. It writes directly to the compile-time crate's benchmarks.md; failure
  to read existing Markdown substitutes a new header. Replacement affects the
  first marker pair, appends when no opening marker exists, and discards the old
  tail when its closing marker is absent. No atomic/concurrent-write protocol is
  provided. The original replacement test does not establish general idempotence.

Read the full crate manifest as context for its feature defaults, package example
selection, dev dependencies, and seven bench declarations. Read the three shared
writer call sites and Matrix3 inverse/error-alias excerpts. These receive no new
whole-file implementation/benchmark credit. The already-reviewed Hyperlimit
writer was reread as a documentation source; comment-stripped comparison showed
only crate constants, the fixed trace-report row, and manifest-test expectations
vary. Reused its applicable prose with those differences explicitly adapted;
no shared implementation refactor or mathematical authority was introduced.

Validation after final edits:

- Focused Rustfmt, `git diff --check`, declaration/field inventory, and exact
  comment/blank-line-stripped source identity passed for all four Rust files.
- The actual quick-start binary ran against the built all-feature lattice library.
  Both original README tests passed, including the unchanged full-block equality
  assertion. Their version/heading test is substring validation, not publication
  or semantic README validation.
- Both original report tests and a temporary probe appended to the unchanged
  actual module passed before/after. The probe exercised missing input, malformed
  required versus optional JSON, mean/median fallback, invalid throughput count,
  zero-time ratios, grouping eligibility/order, manifest feature order, comment
  classification, absent source, unterminated section replacement, and formatting.
  The synthetic rendered report matched byte for byte before/after. Probe files
  live only in /tmp; no checked-in report writer or timing workload was executed.
- Private Rustdoc with warnings and broken/private links denied passed for both
  examples and the README-test source. Complete text of 57 documentation groups
  matched 42 generated HTML pages. A cfg(test) doc build also rendered the helper's
  test-module prose. Rustdoc omits the four #[test] functions; their docs were
  source-reviewed and their bodies compiled/executed without modifying attributes.
- Cargo check passed for all three direct benchmark consumers (mathbench,
  regression_sentinels, retained_fuzz), both examples, and the README test, under
  all features and no default features. Commands used locked/offline resolution,
  two workers, the shared target, and disabled incremental compilation.
- Directly selecting Criterion for a workspace dependency-only build first hit
  version ambiguity, then a Cargo feature-resolver panic. A temporary external
  dependency crate pinned Criterion 0.7.0 and serde_json 1.0.149 and built offline
  into the same target for executable probes. Its initial unused-dependency
  warning was resolved by explicit re-exports; its final locked build passed.
  No repository dependency/lockfile changed. The normal consumer checks above
  use the repository manifest and lockfile and passed independently.
- Full workflows remain owner-waived. No timing/dispatch benchmark, general
  geometry qualification, symlink-cycle traversal, concurrent writer, or failure
  injection against repository reports ran. Temporary binaries are removed after
  validation; shared build storage is approximately 2.4 GiB, retained for reuse.

Evidence: `/tmp/csgrs-rustdoc-lattice-quickstart-{before,after}.log`,
`/tmp/csgrs-rustdoc-lattice-report-{before,after,render,consumers,consumers-minimal}.log`,
`/tmp/csgrs-rustdoc-lattice-report-deps-final.log`, and
`/tmp/csgrs-lattice-report/` retain checkers, probe source/fixtures, rendered sample,
dependency metadata/lockfile, and private docs. They are temporary evidence, not
an archival benchmark record. Hyperlattice's perf_atlas example remains open,
as do other unrecorded implementation/test/benchmark/support files and most of
the wider repository. This batch does not establish repository-wide coverage.

## Hyperlattice performance-atlas workload example (27 November 2026)

Reviewed all 1190 baseline lines of
`crates/hyperlattice/examples/perf_atlas.rs` at `7d125a8a4`; current length is
1423 lines. This completes whole-file review records for all four currently
tracked Hyperlattice Rust examples, not the crate or wider repository.
Documented 96 authored declarations including private traits, macro definitions
and methods, associated types, local aliases, and a nested helper; also documented
ten named fields, four enum variants, six tuple fields, and the module contract.
Macro-generated instances inherit their template documentation. No fixture,
operation, assertion, typed API, output description string, or dispatch order changed.

Ownership: development workload generation and timing, with scalar construction,
terminal formatting, and lattice/projective operations delegated to their existing
owners. Relevant contracts are P1/P3 (exact ingress and meaning of facts), P4
(cache scope), P5 (precision versus total resources), P6/P9 (failure/certainty
preservation), P27 (unchanged output contract), and P36/P37 (completed-workload
and no-overclaiming evidence). The domain is seeded finite recipe trees, fixed
2D/3D/4D carriers, bit/count/exponent axes, optional CLI selection, and the shared
atlas harness. Missing acceptance witnesses were full private docs and accurate
statements of generated representations, ranges, completion tokens, and timing
scope. Read the source and ran bounded actual-source probes before editing.

Findings and documented mathematical/operational scope:

- E holds rational seeds, signed principal roots, ordered sums, and rational scales.
  Real values are reconstructed in callbacks; recipe factories and their rational
  work occur outside timing. This does not reset global caches or establish cold
  execution. Sums rebuild each child visit without graph memoization. Reduced
  rational sizes can shrink; the root recipe does not exclude square radicands.
  At seed 1 and width 2, its generated radicand is exactly one. Labels therefore
  describe recipes rather than guaranteed irrational values or storage classes.
- The odd-decimal helper preserves positive integer bit width. Root signs are
  applied after taking the principal root. Empty sums produce exact zero; invalid
  root domains panic through expect. There is no recipe depth/size/work budget.
  Native adapters simply delegate operations; short input slices panic, longer
  slices are ignored beyond the carrier width. Dimension dispatch falls back to
  Vector4 for dimensions other than 2/3, and Matrix4 for dimensions other than 3.
- Output forcing first asks for an exact sign at binary precision -2048. An
  undecided sign returns None, a certified zero returns length token one without
  formatting, and a nonzero value is formatted with one leading plus sixteen
  fractional digits. Formatting is an approximate terminal and its string is
  discarded. Length sums are not value checksums; empty forcing returns zero and
  failure short-circuits later lanes. The sign precision does not bound total
  arithmetic work or subsequent formatting. Structural-facts rows return a Boolean
  token and orientation rows a sign discriminant, bypassing the blanket formatting
  claim in the previous module introduction.
- Every Problem passed through ok becomes None, losing its typed reason. Callbacks
  do not return policy/certainty receipts or independently check expected values,
  inverses, rotation equations, unit frames, branch identities, or incidence.
  The shared harness checks only the first probe's Outcome and does not preempt
  a stuck call. Its seeded fuzz series are timing samples, not correctness-directed
  fuzzing. Failed probes and timing stops have the already-documented shared-harness
  semantics; this audit adds no claim of a completed performance envelope.
- Thirteen vector/point families include unbounded-in-t lerp and a two-vector frame
  result; centroid factories/timing boundaries are explicit. Seventeen matrix
  families distinguish generic operations, four rational masks, four conditioning
  callbacks, and positive power exponents. Structural masks retain their exact
  behavior, including the unknown-string last-row-zero fallback. The near-singular
  last row is an exact sum at sentinel 512; finite noise need not prove nonsingularity
  or a condition number. Mixed conditioning chooses one kind for a whole matrix.
- The three transform families distinguish affine batch sweeps from projective
  mixed samples. The actual angle is an odd 53-bit integer divided by 2^54, hence
  in [1/4,1/2), contrary to the unchanged report description's [1/2,1). Sixty-four
  exact-rational fixtures confirmed that range; the width formula supplies the
  general argument. Axis-angle forces the point output, while between-vector
  rotation forces the matrix. No independent rotation oracle is present.
- Eight complex families include reciprocal, which still constructs its unused
  second operand. Unary shifted inputs are 1+positive recipe, scaled by 2^k.
  Dyadic shifted parts lie in [1.5,2), but rational/root parts can exceed two;
  among 64 seeded 53-bit cases, 30 rational and 30 root radicands exceeded one,
  establishing shifted values above two. The unchanged general [1,2) label is
  therefore too narrow. Near-cut x is greater than one but not bounded above by
  two; its callback cannot report the distinct error categories named in the
  retained description because ok erases them. Rustdoc explicitly records these
  existing report-label limitations without changing serialized descriptions.
- The three geometry families force homogeneous/affine intersection coordinates
  or raw line direction/moment without a rank/incidence oracle. Orientation uses
  c=(2a+b)/3 + epsilon*left_perp(b-a), so its determinant is epsilon*|b-a|^2.
  Coincident a,b can give zero before the zero-noise sentinel. The callback only
  returns a sign cast and does not assert that independent identity. These are
  expression/forcing workloads, not general region or topology qualification.

Context reads revisited shared perf-atlas RNG, first/repeated-probe timing,
selection, stopping, and serialization code; Hyperreal sign/LowerExp formatting;
and vector normalization, angle, and frame delegates. These excerpts receive no
new whole-file credit. Their owning sources and existing whole-file records
remain separate from this example's review.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing doc attributes and
  comment-only lines, then applying canonical Rustfmt, produces identical source
  against `7d125a8a4`. Tuple-field docs required only formatting/trailing-comma
  normalization. All executable expressions and operation-description strings
  are unchanged. No output fixture or assertion was weakened.
- Direct rustc compiled the actual whole example plus a temporary probe before
  and after. Bounded probes checked odd conversion, square-root collapse, empty
  sum/force, scaling, zero/nonzero formatting tokens, Problem erasure without
  invoking its terminal closure, vector/matrix dispatch fallbacks, row-major
  identity, all structural masks, and exact zero determinants of four generated
  singular 3x3 matrices (one per kind). The root/angle/range probes above also
  passed. Probe stdout matched byte for byte; these are finite witnesses, not
  universal operation closure or benchmark measurements.
- The complete main ran with an unmatched filter. It registered the same 44
  operation names in the same order and wrote a valid empty-operation JSON report.
  finish omits unmeasured operations, so this is CLI/registration smoke evidence,
  not serialized coverage of all descriptions or any timed workload execution.
  Invocation timeouts were external (25 seconds for probes, 15 for main).
- Private Rustdoc generated with warnings and broken/private links denied. Three
  bracket expressions initially parsed as links were corrected to code spans.
  Complete text of 102 source doc groups (80 distinct texts) and all six tuple-field
  docs matched 47 generated HTML pages. Nine function-local/template-local groups
  are source-reviewed and compiled but absent as independent item docs: five
  dimension aliases, three callback type aliases, and nested force_parts. Duplicate
  trait/impl prose is not counted as unique rendered text.
- Cargo check of the actual example passed under all features and no default
  features. The shared perf-atlas library built with the repository lockfile;
  checks used offline resolution, two workers, shared target storage, and disabled
  incremental compilation. The example has no feature-gated local declarations.
- No full atlas timing run, release measurement, competitive comparison, forced
  expensive transcendental workload, or broad geometry tests ran. Full workflow
  checks remain owner-waived. Temporary probe/example executables were removed;
  shared target storage remains approximately 2.4 GiB for reuse.

Evidence: `/tmp/csgrs-rustdoc-lattice-atlas-{before,after,render,check,minimal-check,deps}.log`,
`/tmp/csgrs-rustdoc-lattice-atlas-deps.json`, and
`/tmp/csgrs-lattice-atlas/` retain sources/checkers, canonical source comparison,
probe output, empty JSON smoke reports, registration logs, and private docs.
These are temporary audit artifacts, not a durable performance archive. Remaining
Hyperlattice implementations, integration tests, benchmarks and support prose,
and most of the wider repository, still require their own review records.

## Hyperlattice complex arithmetic core and diagnostic tests (27 November 2026)

Whole-file review at `350f5452d` covered `crates/hyperlattice/src/complex.rs`
(563 → 709 lines), `tests/complex.rs` (89 → 101), and
`tests/diagnostic_display.rs` (89 → 102), with test paths relative to the crate.
Documented all 66 core declarations, including module declarations, private helpers,
operator methods and associated output types, plus both public component fields.
Documented seven declarations in the arithmetic test file and three in the display
file, including their test functions and private unresolved-zero fixture.
Implementation, scalar-call order, trace labels, assertions, fixtures, and display
strings remain unchanged.

Owning layer: complex carriers and operation scheduling above native scalar
arithmetic; diagnostic display is a presentation boundary. Relevant contracts are
P1 (exact components), P3 (structural facts/equality), P4/P10 (reuse evidence and
call order), P5 (resource scope), P6 (native typed errors), P9/P30 (no implied
certainty receipt), and P36/P37 (evidence and precise coverage). The domain is
native Real component pairs, all provided owned/borrowed arithmetic forms, signed
i64 powers, and ordinary/alternate diagnostic formatting. Missing private docs and
incomplete public descriptions of zero powers, checked rational specialization,
reuse scheduling, and formatting flags were the acceptance gap. Before edits,
the eight original tests and bounded exact/trace probes passed on the target
revision, reproducing the documented behavior rather than assuming older reviews.

Clarified contracts:

- Complex stores editable re/im expressions. Construction does not normalize,
  refine, or validate a geometric carrier. Derived equality compares scalar
  structure, not arbitrary expression values; Clone preserves native sharing and
  cache semantics. Result aliases retain Problem without adding predicate policy,
  certainty/provenance, total-work, or independent cancellation guarantees.
- Multiplication is (ac-bd)+(ad+bc)i. It observes reuse evidence for all four
  components in lhs.re, lhs.im, rhs.re, rhs.im order. All Some(true) selects the
  three-product formula; all exact but not fully reused selects the fused rational
  kernel; otherwise active signed-product reducers form the components. Evidence
  can record an isolated rational's first observation. It is advisory scheduling,
  not a numerical approximation or mathematical-equality test.
- Division classifies all four components in the same fixed order. Exact-rational
  quotient construction directly owns its zero-denominator check even in checked
  mode. The general path constructs the rhs squared modulus, applies the optional
  cheap nonzero guard, inverts before forming the two numerators, then shares that
  inverse. The public norm query's fused sum intentionally differs from inline
  reciprocal/division denominator construction. No calls were consolidated or
  reordered to make documentation simpler.
- Exponent zero requires cheap proof that at least one component is nonzero.
  Known zero returns NotANumber; unresolved knowledge returns UnknownZero. The
  real exact-rational shortcut and remaining real/imaginary fact order are explicit.
  Positive exponents admit zero. Exponent -1 calls the reciprocal directly; other
  negative exponents invert their already-computed positive power. Checked powers
  use the corresponding checked inverse at that point. unsigned_abs admits
  i64::MIN without signed-negation overflow. Small powers and binary squaring keep
  their exact clone/multiply schedules; logarithmic multiplication count is not a
  bound on expression/coefficient growth. The private squaring helper requires
  a nonzero exponent and would panic on zero; public methods handle it first.
- Every owned/borrowed operator and Output type has documentation of its carrier,
  operand use, shared scheduler, or native failure behavior. Cached scalar adapters
  describe borrowing/reuse, not a guaranteed cache hit. BitXor is the integer-power
  alias and retains powi's domain semantics.
- Display retains literal parentheses/plus/i spelling, including `+ -2i` for a
  negative imaginary coefficient. Alternate mode uses diagnostic_decimal(-256)
  per component and may emit nonnumeric failure markers. It does not forward
  outer width/precision/sign flags to nested formatting. This is presentation,
  not exact interchange or a complete time bound.
- The display test's sin(e)^2+cos(e)^2-1 is mathematically zero; its name denotes
  unavailable cheap knowledge. Its unprepared/prepared passes reuse scalar state,
  and the -512 interval preparation result is discarded. Marker-count assertions
  are not independent cold/warm measurements, a full-string oracle for every
  output, or numeric-error bounds. The arithmetic tests' formula expectations
  remain limited to their rational fixtures and selected ownership/domain cases.

Read the 61-line elementary child and 161-line Jacobi child as context for module
roles, plus scalar guards, rational reuse/product/quotient primitives, diagnostic
formatting, shared test fixtures, and selected API-surface assertions. The two
child files still need their dedicated branch/continuation and documentation
review before receiving completion credit. Other excerpts receive no additional
whole-file credit. No mathematical kernel or approximation policy was changed.

Validation after final edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only/blank lines
  leaves identical source in all three files against `350f5452d`.
- The actual all-feature library built. Direct rustc harnesses compiled and ran
  all six existing complex tests and both diagnostic tests against that library,
  then repeated all eight against the no-default-feature build. The original
  assertions and inputs were unchanged; each process had a 30-second external
  containment timeout rather than an inferred internal resource guarantee.
- A bounded public-API probe checked independently specified rational product,
  quotient, reciprocal and norm results; integer powers 1..8, -2 and i64::MIN;
  ordinary/checked zero-exponent UnknownZero; a known nonzero imaginary component
  overriding an unknown real component; 0^0; and negative-imaginary/outer-precision
  formatting. Both feature configurations passed. Before/after all-feature probe
  stdout matched exactly.
- Exact trace assertions before/after observed one cold fused-rational product,
  one repeated three-product rational path, and one mixed-symbolic path. Each
  required its named count to equal one. This supports these specific scheduling
  witnesses, not every possible dispatch path or a performance equivalence claim.
- All-feature and no-default-feature private library Rustdoc passed with warnings
  and broken/private links denied. In each configuration, complete text of all
  69 core documentation groups (50 distinct texts) matched 12 HTML pages. Four
  further module/helper groups from the edited test files matched their private
  docs; the eight #[test] functions are source-reviewed and executed but omitted
  by Rustdoc. Repeated Output prose is not counted as unique rendered text.
- Hyperlimit downstream library checks passed under all features and no default
  features. Cargo used the repository lockfile, offline resolution, two workers,
  disabled incremental compilation, and the shared target. Full workflow checks
  remain owner-waived; no benchmark, full symbolic/transcendental corpus, sanitizer,
  or release-size measurement ran. These docs-only edits make no speed/memory claim.
- Removed the three temporary test/probe executables after validation. Shared target
  storage remains approximately 2.4 GiB; no duplicate Cargo target tree was created.

Evidence: `/tmp/csgrs-rustdoc-complex-{before,after,build,doc,render,downstream}.log`,
`/tmp/csgrs-rustdoc-complex-minimal-{build,test,doc,render,downstream}.log`, build JSON,
and `/tmp/csgrs-complex-audit/` retain the probe/checkers and generated test docs.
These are temporary review artifacts, not a durable performance archive. The
remaining complex continuation files, other library implementations/tests,
benchmarks/support material, and most of the wider repository remain open.

## Hyperlattice complex atanh and reference qualifications (27 November 2026)

Whole-file review at `3ce14924b` covered `crates/hyperlattice/src/complex/elementary.rs`
(61 → 81 lines), `tests/complex_atanh.rs` (279 → 332), and
`tests/support/complex_atanh_oracle.rs` (227 → 300), with test paths relative to
Hyperlattice. Documented the public operation and module role, all 16 test-file
declarations (including ten tests, three helpers, and three imported modules),
and all 21 oracle declarations, two named fields, and two tuple fields. The
oracle's three impl blocks for Interval share mathematical authority; no helper,
formula, assertion, precision, fixture, or execution order was changed.

Owning layers: exact complex continuation over Hyperreal, and test/benchmark
reference qualification. Relevant contracts are P1/P3 (exact construction and
honest knowledge), P4 (refinement/cancellation observations), P5/P6 (resource scope
and typed domain failures), P7 (stable scalar forms), P9 (scope of certainty),
and P36/P37 (completed-workload evidence and coverage). The operation domain is
native Real pairs with branch decisions sufficient to exclude the real cuts.
The reference has a smaller local/reciprocal rectangle domain. Missing private
docs, incomplete branch descriptions, and an overstated reference-enclosure claim
were the acceptance gaps. All ten existing tests passed on the baseline, while
bounded probes reproduced the reference rounding problem on that same revision.
The root AGENTS.md public/private 100% Rustdoc mandate remains in force.

Clarified operation and test contracts:

- For z=x+iy, L=(1-x)^2+y^2 and U=(1+x)^2+y^2, the general result is
  ln(U/L)/4 + i*atan2(2y,1-x^2-y^2)/2. The immediately known sign of x selects
  ln_1p(4x/L), -ln_1p(-4x/U), or the exact quotient-log fallback. No extra sign
  refinement is performed solely to choose that stable expression. The angle
  uses the native difference-of-products constructor. This follows the
  [NIST principal logarithmic definition](https://dlmf.nist.gov/4.37.E24).
- Definitely-zero imaginary and real components select the real-atanh and
  imaginary-atan paths before imaginary-sign refinement. An unresolved imaginary
  sign still permits the strict strip 1-x^2>0 when that sign is certified; otherwise
  it returns Exhausted. Certified real endpoints ±1 return Infinity and certified
  exterior cut points return NotANumber. No signed-zero convention selects a cut
  bank. Native failures propagate without a new policy, certainty receipt, or
  total-work/retained-memory bound.
- Existing tests separately cover axis identities/errors; nine small and ten large
  oracle fixtures; the shared 22 labeled scalar recipes; conjugation, parity and
  forward composition; tiny offsets at poles/cuts; unresolved zero knowledge;
  cancellation/retry and reader availability; same-process JSON replay; an opaque
  unit circle; and tiny/huge scaling. The recipe loop does not assert its count or
  distinct private representation tags. Native forward formulas share scalar
  machinery, and finite tolerance checks are not exact-equality proofs.
- The cancellation test checks successive interval overlap, not nesting, in the
  32→512→64→256 request sequence. It checks interval availability in four threads,
  not equality between every reader. Serialization checks component closeness and
  deterministic text in the same process; it is not fresh-process replay of the
  originating complex operation and branch-decision evidence. Tiny-offset fixtures
  compare normalized analytic leading coefficients, not a uniform remainder bound.
- The oracle converts native -850 input enclosures to 768-bit MPFR endpoints.
  Its local sum has 385 terms through z^769, with intended conservative tail
  q^771/(1-q^2), where q must upper-bound the L1 radius. If the original computed
  radius exceeds 3/4 it uses a reciprocal and requires the selected computed
  radius <=3/4. Reciprocal continuation requires a strictly signed original
  imaginary interval and adds the corresponding ±i*pi/2. These formulas agree
  with the [NIST local series](https://dlmf.nist.gov/4.38.E5) and
  [reciprocal series](https://dlmf.nist.gov/4.38.E6); formula validity does not
  establish the implementation's rounding guarantees.

Open reference-qualification finding, reproduced without changing implementation:

- Pair::radius passes an already-computed owned Float sum to with_val_round(Up).
  Rug 1.30.0 owned arithmetic uses nearest rounding; converting that value at the
  same precision cannot recover a lost bit. At 768 bits, actual Pair components
  exactly 1 and 2^-800 produce radius 1, strictly below their exact L1 sum. The
  corresponding borrowed addition with upward rounding is strictly greater than 1.
- The tail denominator similarly computes owned `1 - q2` before Down conversion.
  For q2=2^-800, that expression returns 1, above the directed lower subtraction.
  The final owned quotient likewise precedes Up conversion: at 768 bits, 1/11
  rounds below the directly upward-rounded quotient. These are mechanism witnesses;
  the latter two are not a demonstration that the full reference reaches those
  exact intermediate values for its existing fixtures.
- agrees_with computes nearest-rounded widths rather than certified upper widths.
  It tests overlap and small widths, assuming valid ordered input rectangles. It
  cannot by itself certify exact equality or repair earlier reference-bound gaps.
- Both original reference comparisons exercised by the probe and all existing
  tests still pass. No end-to-end erroneous atanh fixture result was demonstrated.
  The universal certified-oracle claim is unsupported; the numerical kernel and
  reference remain unchanged under the owner's docs-only scope. Future work must
  repair directed arithmetic in the reference owner, preserve the hard fixtures,
  and establish enclosure/tail evidence before treating benchmark or test overlap
  as uniformly certified. This documentation is not that repair or qualification.

Read the shared Hyperreal MPFR cleanup and representation recipe files, relevant
scalar atanh/atan2 and rational sign/magnitude code, Rug's owned/borrowed arithmetic
implementation, and the direct benchmark's oracle import as context. They receive
no new whole-file completion credit in this batch. The remaining Jacobi child,
benchmark implementation, and wider library/test corpus still require their own
review. The previously incomplete elementary child now has whole-file credit.

Validation after the documentation edits:

- Focused Rustfmt and git diff --check passed. After removing documentation and
  canonical Rustfmt normalization, all three files are identical to `3ce14924b`.
  Normalization accounts for multiline tuple-field doc attributes/trailing commas;
  executable expressions, assertions, fixtures, and literal messages are preserved.
- Actual all-feature and no-default-feature library builds passed. Direct rustc
  harnesses ran all ten original tests with serde and all nine without it against
  their corresponding built libraries. A 60-second external process timeout
  contained each harness. The bounded reference probe used a 20-second external
  timeout and also checked negative-rational endpoint order and zero multiplication.
  Before/after all-feature and no-default-feature probe stdout matched exactly.
- Both feature configurations generated private library Rustdoc with warnings and
  broken/private links denied. Explicit private test Rustdoc compiled under each
  matching serde configuration. Complete text of 33 documentation groups plus
  two tuple-field docs matched the rendered output across 32 HTML pages. The ten
  test-function doc groups were source-reviewed; Rustdoc omits #[test] functions,
  and the serde test is executed only when enabled. Linked source URLs are omitted
  from HTML visible text, so the checker normalizes Markdown link destinations.
- Cargo checks of the direct complex_atanh integration test and benchmark passed
  with all features and no defaults; these were repeated after final test/oracle
  wording edits. Hyperlimit downstream library checks passed in both configurations.
  All Cargo work used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared
  target. The temporary direct harness selected serde_json from the matching
  production Cargo artifact graph after an initial temporary dependency-feature
  mismatch; no repository dependency or lockfile change was needed.
- Full workflow checks remain owner-waived. No timing benchmark, exhaustive branch
  proof, sanitizer, release-size comparison, or full repository suite ran. This is
  a docs-only audit with an explicitly open oracle finding, not a performance or
  universal complex-transcendental correctness claim.
- Removed the two temporary test/probe executables after validation. The shared
  Cargo target is approximately 2.5 GiB; no duplicate target tree was introduced.

Evidence: `/tmp/csgrs-rustdoc-atanh-{before,after,build,doc,render,consumers,downstream}.log`,
`/tmp/csgrs-rustdoc-atanh-minimal-{build,test,doc,render,consumers,downstream}.log`,
corresponding Cargo artifact JSON, and `/tmp/csgrs-atanh-audit/` contain temporary
probes/checkers and generated private test docs. Temporary artifacts are review
evidence, not a durable performance archive. Most of the repository audit remains
open; declarations counted here do not establish overall 100% coverage.

## Hyperlattice Jacobi continuation, oracle, tests and timing consumers (27 November 2026)

Whole-file review at `86a8dcffa` covered these Hyperlattice files:

| File relative to crates/hyperlattice | Original → documented lines | Declarations |
| --- | ---: | ---: |
| src/complex/jacobi.rs | 161 → 210 | 4, plus five fields |
| tests/complex_jacobi.rs | 370 → 424 | 20 |
| tests/jacobi_epsilon.rs | 287 → 323 | 16 |
| tests/support/complex_jacobi_oracle.rs | 223 → 268 | 14, plus two fields |
| benches/complex_jacobi.rs | 70 → 88 | 3 |
| benches/jacobi_epsilon.rs | 69 → 87 | 3 |
| benches/complex_atanh.rs | 72 → 90 | 3 |

All seven files were read in full, including the existing assertion bodies and
benchmark timing/validation loops. Every authored item in this slice now has
Rustdoc, including private helpers, imported/nested modules, constants, fields
and test functions. This completes the remaining Jacobi child of the previously
reviewed complex core. It is not a repository-wide coverage claim. The root
AGENTS.md mandate for 100% public/private documentation remains unchanged.

Owning layer: Hyperlattice complex assembly over native Hyperreal amplitudes,
with a separate development oracle and timing consumers. Relevant properties:
P1/P2/P3 (exact meaning, retained relations and honest knowledge), P4/P5/P6
(cache observations, resource scope and native errors), P7 (stable denominator),
P17/P18/P19 (output relations and replay scope), and P36/P37 (completed-workload
and coverage evidence). The production domain is finite native Real component
pairs and an admitted real squared modulus m<1, including negative m; known
poles fail explicitly. The oracle uses a smaller local analytic domain. Missing
private contracts and insufficient distinctions between exact identities,
interval comparisons, cache observations and serialization evidence were the
documentation acceptance gaps. No implementation, predicate order, assertion,
fixture, benchmark mode/precision, trace label or dependency was changed.

Clarified mathematical and practical contracts:

- ImaginaryJacobi stores complementary real functions (S,C,D) at y|1-m, not
  the complex triplet at i*y. It also retains one inverse depending on the
  real-axis triplet and optionally the epsilon vertical correction. The helper
  trusts the caller's certified nonzero parameter sign and matching real_sn/
  real_cn inputs; it does not independently prove their common origin.
- Positive m uses the complementary parameter h=1-m. Negative m uses r=sqrt(h),
  argument r*y and parameter 1/h, then maps the reciprocal triplet (s,c,d) to
  (s/r,d,c), consistent with the
  [reciprocal-modulus identities](https://dlmf.nist.gov/22.17). The API uses m=k²,
  whereas DLMF uses modulus k. The stable negative-parameter denominator
  c²+(-m/h)*real_cn²*s² equals complementary Q=C²+m*real_sn²*S².
- One shared inverse of Q serves all six triplet components, or both epsilon
  components. Avoiding a separate divisor C admits finite quarter-period values
  when C=0 but Q!=0; complex dn can be zero. The
  [addition formulas](https://dlmf.nist.gov/22.8) explain the assembly. Real
  evaluation validates the parameter before the triplet's real-axis shortcut.
  Remaining zero-parameter input uses circular/hyperbolic continuation.
- Epsilon's real-axis path retains the native unwrapped composition. When
  imaginary zero is not cheaply known, ln(1-m) precedes sign certification and
  the m=0 identity. The nonzero path requests a shared real amplitude for triplet
  plus epsilon, and similarly shares complementary data. The vertical correction
  is y-epsilon(y|h), or r*(r*y-epsilon(r*y|1/h)) after reciprocal transformation.
  The [epsilon addition law](https://dlmf.nist.gov/22.16.E27) supplies the two
  rectangular components without a separate C divisor. The final expect checks
  the internal with_epsilon invariant, not a user-domain condition.
- NotANumber, Exhausted, DivideByZero and UnknownZero retain their distinct scalar
  boundaries. Exact expressions retain native relation/domain obligations without
  a separately added complex proof node, certainty receipt, or total-work/
  retained-heap/wall-time bound. Unknown imaginary zero is not a proof of nonzero.
- The local oracle derives coefficients from the
  [Jacobi differential system](https://dlmf.nist.gov/22.13.T1), with epsilon'=dn².
  For A=max(1,|m|), the convolution recurrence bounds degree-n coefficients by A^n;
  epsilon's positive-degree coefficient is bounded by A^(n-1). Degree 512 and
  q=A*(|x|+|y|)<=3/4 give geometric tail q^513/(1-q). Directed interval arithmetic
  bounds coefficients, Horner evaluation and tail for valid finite intervals.
  The radius sum, denominator, quotient and width differences use borrowed MPFR
  arithmetic with directed rounding; the preceding atanh oracle's owned-rounding
  finding was not assumed to apply here.
- Oracle input bounds still use production native evaluation at -850. It is an
  independent differential formula, not independent input conversion or global
  continuation. The helper's local domain does not itself require m<1, while
  compare first requires production admission. It returns six ordered triplet
  intervals or two epsilon intervals. O(DEGREE²) interval operations and
  O(DEGREE) retained coefficient endpoints do not bound native refinement or
  all MPFR memory/work. Bad domains assert and unavailable bounds unwrap-panic.
- The triplet relation helper consumes exact rational normal forms for three
  square-sum expressions; the third is algebraically dependent but tests another
  expression shape. These identities alone do not select a unique value/branch.
  Other tests use explicitly documented finite tolerances or independent local
  enclosure overlap. Shared recipe loops assert public kinds, not distinct
  private tags or a coverage count. Negative-parameter quarter-period sn/cn
  assertions test squared magnitudes without independently asserting their signs.
- Cancellation tests require successive overlap, not nested intervals. Thread
  checks require interval availability without comparing every reader's value.
  The triplet retry checks closeness; the epsilon retry checks availability.
  Serialization independently evaluates original/decoded components before
  structural cancellation, but remains same-process evidence. Triplet replay
  additionally checks closeness and exact relations; epsilon replay checks
  overlap and deterministic text, without an explicit width/equality assertion.
- Both benchmark programs prepare native/reference data before timing. Each
  timed batch retains outputs until validation, which is excluded from elapsed_ns.
  Native construction/cloning can share warmed inputs/global scalar state.
  --test selects nine rows (three parameters times three modes); normal operation
  has five trials, five native repeats or one oracle repeat. Every printed row
  is followed by an accepted==repeats assertion. Precision differences and lack
  of isolated cold caches preclude an inferred comparative speedup claim.

The atanh benchmark was also read in full as the remaining direct consumer of
the preceding batch's qualified oracle. Its module, reference import, component
helper and main loop now document the radius/tail/width limitations explicitly.
The printed contract and all executable row/acceptance logic remain unchanged;
accepted overlap is not proof that the reference-rounding gap has been resolved.
Its local, reciprocal and imaginary-axis regimes use the same timing/validation
separation as the Jacobi programs, with mpfr_series as the reference mode.

Read the two local admission narratives, shared scalar fixtures/MPFR cleanup,
real Jacobi evaluator, and scalar logarithm/inversion context. The admission
narratives' historical sanitizer/coverage/timing reports were not reproduced;
those references receive no new evidence or whole-file qualification credit.
No unrelated scalar implementation was edited.

Validation and limits:

- Focused Rustfmt and git diff --check passed. Removing comment-only and blank
  lines leaves identical source in all seven files against `86a8dcffa`, including
  all expressions, assertions, literals and call order. A declaration scan found
  preceding Rustdoc on all 63 declarations; all seven named fields were separately
  reviewed and rendered. This inventory is limited to these files, not macros or
  generated APIs elsewhere in the repository.
- The initial unoptimized all-feature triplet test process passed nine tests
  before its external 180-second limit expired during serialization. A subsequent
  300-second-contained process completed all twelve unchanged triplet tests in
  241.25 seconds. The initial timeout is retained as incomplete evidence, not a
  native typed-exhaustion result or a passing test. These elapsed times describe
  development validation, not a benchmark or promised resource envelope.
- The all-feature epsilon process passed its scalar-family ODE, near-pole,
  exact quarter-period and huge-quasi-period tests, then reached its external
  300-second limit during independent_ode_epsilon_enclosures. Its completed tests
  are recorded separately from the incomplete suite. The seven not-yet-completed
  tests were selected unchanged in a separate process with a 180-second limit;
  only the four already-passed names were filtered out.
  That process passed the ODE, invalid/unknown-error and parity/quasi-addition
  tests, then timed out during precision_order_cancellation_and_concurrency_preserve_values.
  Thus seven distinct all-feature epsilon tests completed across the two runs;
  cache/concurrency, real-axis inverse composition, serde replay and the huge
  zero-parameter case did not complete in these all-feature runs. The last case
  did pass in the selected no-default-feature run. No assertion failure was
  observed, but the complete all-feature epsilon suite is unverified. Source
  identity and documentation checks support this docs-only patch; they do not
  turn those timeouts into passed runtime checks.
- No-default-feature builds compiled both complete test harnesses. Seven selected
  original tests passed: triplet invalid parameters, unresolved parameter/pole
  distinctions, quarter periods and the large imaginary zero-parameter case;
  epsilon invalid/unknown errors, finite quarter periods/poles and huge-argument
  zero-parameter identity. The full no-default-feature runtime suites were not
  executed. Each selected test process had an external 30-second timeout.
- All-feature and no-default-feature private library Rustdoc passed with warnings
  and broken/private links denied. Explicit private test/benchmark Rustdoc passed
  under the corresponding serde settings. Full-text checks matched all 54
  non-test documentation groups across 98 generated HTML pages; repeated reference
  modules/text are not counted as new unique APIs. All 23 test-function doc groups
  are source-reviewed, since Rustdoc omits #[test] functions. The atanh bench has
  no feature-gated item and its explicit private docs were generated against the
  no-default-feature graph; Cargo checked it in both configurations.
- The two Jacobi benchmark --test runs each emitted nine rows and required every
  repeat to be accepted, under a 90-second external limit per program. The atanh
  benchmark emitted nine accepted rows both before and after its documentation
  edit under 30-second limits; removing elapsed_ns leaves identical JSON rows.
  The known atanh oracle qualification remains. Smoke acceptance neither measures
  performance nor certifies the intended bounds beyond each oracle's contract.
- Cargo checked both Jacobi integration tests and all three benchmark consumers
  with all features and no defaults. Hyperspatial downstream library checks passed
  in both configurations, including their Hyperlimit dependencies. Cargo used
  --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared target directory.
  Direct rustc harnesses used the actual matching library/Rug/serde artifacts;
  no repository dependency or lockfile changed.
- Full workflow checks remain owner-waived. No sanitizer campaign, primitive-cache
  feature matrix, fresh-process replay proof, statistical timing comparison,
  release-size measurement, or whole-repository runtime suite was attempted.
  Documentation and finite witnesses do not establish universal elliptic-function
  or downstream modeling closure.

- Removed the six temporary test/benchmark executables after the processes
  terminated. The shared target remains approximately 2.5 GiB; no duplicate Cargo
  target tree or release build was introduced.

Evidence: `/tmp/csgrs-rustdoc-jacobi-{before,after,epsilon-remaining,build,doc,render,consumers,downstream,smoke}.log`,
`/tmp/csgrs-rustdoc-jacobi-minimal-{build,test,doc,render,consumers,downstream}.log`,
`/tmp/csgrs-rustdoc-jacobi-atanh-bench-{before,after,check,all-check}.log`, Cargo
artifact JSON, and `/tmp/csgrs-jacobi-audit/` retain temporary harness/checker source,
private docs and smoke JSON. These are review artifacts, not a durable performance
archive. Vector/matrix implementations, remaining tests/benchmarks/support files
and most of the wider repository still require audit.

## Hyperlattice vectors, shared-scale facts and ownership tests (27 November 2026)

Whole-file review at `e5779844d` covered `crates/hyperlattice/src/vector.rs`
(2190 → 2458 lines), `tests/vector.rs` (678 → 715), and
`tests/borrowed_ops.rs` (62 → 73), with test paths relative to Hyperlattice.
All source lines, branches, macro arms and existing assertions were read. The
vector file now documents all 196 named declarations, 40 fields (including three
tuple fields), nine variants and its module. Counts include the authored macro
items once; expansion is checked separately. The vector test has 23 declarations
and three local layout-fixture fields; the ownership test has five declarations.
The root AGENTS.md public/private Rustdoc mandate remains unchanged.

Owning layer: native vector storage, structural scheduling and dimension-specific
algebra over Hyperreal. Relevant properties are P1/P2/P3 (exact components and
honest facts), P4/P10 (snapshot identity and dispatch order), P5/P6 (resource,
cancellation and error boundaries), P7 (scalar-owned kernels), P9/P19/P30
(certainty/provenance scope), and P36/P37 (evidence and coverage). Domains include
mutable 2D/3D/4D Real arrays, generic equal-reduced-denominator carriers, binary
float ingress/export and scalar/vector ownership forms. Missing private/helper/
operator Rustdoc and inaccurate cancellation/shared-denominator descriptions
were the acceptance gaps. Baseline tests and bounded probes reproduced current
behavior before annotation. No algorithm, call order, assertion, fixture, trace
label, lint policy, public name or dependency changed.

Clarified contracts and reproduced limitations:

- Ordinary vectors expose mutable coordinate arrays and IndexMut; derived equality
  is structural. Constructors store coordinates without normalization or projective
  validity checks. Fact packets are snapshots, not mutation-tracked proofs.
  Their public fields can be edited or made inconsistent, and methods trust them.
  Vector3/4 count methods count all stored mask bits without masking to dimension.
  A future vector mutation requires a fresh query when current facts are needed.
- Shared-scale construction accepts nonempty exact-rational inputs with identical
  reduced denominators, including integer and dyadic denominators. It does not
  rescale disparate denominators. The old has_shared_denominator_schedule wording
  incorrectly excluded dyadics: quarters satisfy both queries on this revision.
  Zero and one-third fail admission together because their reduced denominators
  differ; an all-zero integer grid is admitted. Owned conversion returning None
  consumes its vector without returning an unchanged fallback.
- The owner's coordinates and retained fact packet are private. Borrowed views
  reuse the packet without rescanning; exposed copies do not mutate the owner.
  A view's public masks can be overwritten while its coordinate references remain
  private. A probe changed a view of (1,1) to report an all-zero mask: its mask
  query then reports zero, while its dot product remains exactly two. This
  demonstrates the trust boundary, not a new geometry or equality certificate.
- Open generic-dimension limitation: u128 masks have only 128 lanes, without a
  typed oversized-constructor guard. SharedScaleVec::<0> rejects empty input;
  128 exact ones construct successfully, count 128 nonzeros and self-dot to 128.
  SharedScaleVec::<129>::from_components(std::array::from_fn(|_| Real::one()))
  panics in the debug build during the oversized shift. The probe catches that
  panic. Release-mode oversized behavior was not tested. Private mask helpers
  now state their size/precondition limits; this disclosure does not establish
  safe arbitrary-N support or narrow the mathematical contract. Typed dimension
  handling or a larger mask representation remains implementation work.
- Scalar zero/one/signed-unit and dependency queries remain at their owning layer.
  The 4D geometric helper reads only w: zero is Direction, one is Point, and
  minus one/other/unresolved is Unknown. Direction includes the all-zero tuple;
  this classification alone does not admit a nonzero projective representative.
  The private packet exists so matrix transforms can avoid a full structural scan.
- The checked norm scan observes lanes in order and returns at the first known
  nonzero, including after an earlier unknown. Otherwise unknown dominates all
  known zeros. Checked normalization rejects Zero/Unknown as DivideByZero/
  UnknownZero before constructing products. The fact-packet norm methods use
  retained counts instead of this live scalar scan; their docs preserve that
  distinction. No predicate calls were merged or reordered.
- Ordinary and checked normalization use scalar denominator clearing when every
  lane exposes an exact rational and at least one is non-dyadic. This route can
  combine differing denominators; it is not the narrower shared-scale-carrier
  admission test. The other route uses dimension-specific self-dot kernels,
  square root, one inverse, and direct borrowed lane multiplication. The abort
  normalization route deliberately omits that rational specialization. norm()
  unwraps magnitude() and can panic on a returned Problem; magnitude preserves
  the scalar failure. No total-work or retained-memory guarantee was added.
- dot_with_abort samples the flag with a relaxed load at entry. A clear flag
  delegates directly to ordinary dot, without attaching that supplied flag for
  future changes. In the probe, a dot of (e,pi,ln(2)) constructed with a clear
  flag still supplies a -128 enclosure after that flag is set. With a set flag,
  definitely-zero pruning is lane-ordered and short-circuits left then right;
  unknown factors survive. An empty active support returns exact zero without
  attachment. Retained operands are cloned/attached once each, then grouped into
  one scalar product-sum. Cancellation is cooperative and exact shortcuts can
  finish; the method returns Real, not a cancellation Result.
- magnitude_with_abort additionally attaches the flag to the completed dot
  expression even when the dot's entry sample was clear. Checked normalization
  performs its structural norm gate before attachment. Checked scalar division
  attaches only the divisor before its zero-status gate/inverse. Vector3 scaling
  uses direct borrowed multiplication where Vector2/4 use mul_cached; owned
  ordinary division retains its assignment schedule. These differences are
  documented rather than consolidated.
- Binary32/binary64 ingress converts every lane through the native exact binary
  rational boundary before collecting the first error. NaN/infinity retain typed
  scalar failures. In particular, 0.1_f64 equals 3602879701896397/36028797018963968,
  not 1/10; exact ratios supply intended decimal rationals. Lossy export stops at
  a missing/nonfinite conversion and provides no independent topology/fidelity
  certificate. Display uses bracketed scalar formatting or diagnostic_decimal(-256)
  in alternate mode, without forwarding other outer format flags. A probe confirms
  that alternate precision formatting is not forwarded.
- lerp permits extrapolation; step does not normalize direction. mean sums from
  zero in slice order and collapses empty/length-conversion/division failures to
  None. weighted_sum rejects empty or mismatched input without normalizing weights.
  Orthonormal basis selection tries X/Y/Z seeds after checked input normalization,
  discards every first-candidate error, then propagates the second normalization's
  error immediately; exhausted seed selection returns UnknownZero. Angle uses
  checked normalization followed by acos without clamping.
- The tests retain independent rational dot/wedge/cross/distance expectations,
  exact fact assertions and output re-admission. Ownership comparisons remain
  comparisons between wrappers of the same native kernel, not independent proofs
  or dispatch/performance equality. The local layout fixture measures current-target
  inline sizes and bounded padding, not total heap or a stable ABI. Float export
  checks concern their finite fixtures, not universal exact round-trip guarantees.

Context reads included scalar zero guards/abort attachment, exact-set schedule
queries, float conversion, rational normalization, shared test helpers, and the
matrix batch/geometric-dispatch consumers. These do not give the 10604-line matrix
core whole-file credit; its review remains open. Existing historical performance
remarks in ordinary implementation comments were not re-benchmarked or promoted
into new performance claims.

Validation after final source edits:

- Focused Rustfmt and git diff --check passed. Removing comment-only and blank
  lines leaves identical executable source in all three files against `e5779844d`,
  including macro bodies, literals, predicates and assertions.
- Actual Hyperlattice library builds passed with all features and no defaults.
  Direct rustc harnesses ran all 21 original vector tests and all four ownership
  tests in both configurations; all passed. The all-feature baseline passed the
  same 25 tests before edits. Each test process had a 60-second external limit.
- The 30-second-contained probe passed in both configurations, reproducing empty/
  dyadic/mixed-denominator admission, 128-lane success, the 129-lane debug panic,
  editable public masks, inactive-flag behavior, exact binary ingress and display
  flag behavior. The panic was an expected observed limitation, not typed success.
  Before/after all-feature stdout matched exactly; the no-default probe repeated
  the semantic checks with tracing compiled out.
- Exact trace assertions covered every surviving-support cardinality for active
  2D/3D/4D dots: twelve named branches total. Each selected branch occurred exactly
  once and clone-and-attach counted exactly twice the surviving lane count. No
  existing count was relaxed. This finite witness does not claim every vector
  path was profiled or establish a performance improvement.
- Private library Rustdoc passed with warnings and broken/private links denied in
  both feature configurations. Explicit private test docs also compiled. Full-text
  checks matched all 246 vector documentation groups (197 distinct texts) across
  the generated docs; each of 61 macro-template groups was also required to occur
  on all three expanded Vector2/3/4 pages. Four test module/helper groups rendered.
  Across 52 inspected HTML pages, repeated operator/trait text was not counted as
  unique prose. The 25 test-function doc groups and local layout type/three fields
  are source-reviewed; Rustdoc omits those function-local/test items.
- Cargo checks passed for vector/ownership integration tests and the mathbench
  consumer with all features and no defaults. Hyperspatial downstream library
  checks passed in both configurations, including Hyperlimit. Cargo used --locked
  --offline -j 2, CARGO_INCREMENTAL=0 and the shared target directory.
- Full workflow checks remain owner-waived. No timing benchmark, sanitizer,
  release-mode oversized-dimension test, binary-size measurement or full repository
  suite ran. These documentation edits neither repair the generic mask limit nor
  establish universal predicate/closure support.
- Removed the three temporary test/probe executables after validation. The shared
  Cargo target remains approximately 2.5 GiB; no duplicate target tree was created.

Evidence: `/tmp/csgrs-rustdoc-vector-{before,after,build,doc,render,consumers,downstream}.log`,
`/tmp/csgrs-rustdoc-vector-minimal-{build,test,doc,render,consumers,downstream}.log`,
Cargo artifact JSON and `/tmp/csgrs-vector-audit/` retain temporary probes/checkers
and private docs. These are review artifacts, not a durable performance archive.
The remaining matrix code, other tests/benchmarks/support material and most of
the wider repository still require audit.

## 2026-11-27: Hyperlattice matrix facts and module navigation

Base revision: `a1f226535`. This is a partial matrix-core audit, not a completed
matrix implementation audit. Owning layer: Hyperlattice's fixed-size matrix
storage and conservative scalar-fact aggregation, under P1, P3, P4 and P10;
private documentation availability also supports the P37 distribution boundary.
The domain reviewed is 3×3/4×4 `Real` matrices, their editable structural-fact
packets, and the private array/mask helpers used to construct those packets.
Before editing, source review found missing private-helper/field Rustdoc and an
inaccurate public single-scan description. The existing eight fact tests and an
additional temporary snapshot/precedence probe passed on the target revision.
No implementation, dispatch order, fixture, assertion, policy or dependency changed.

Coverage in this batch:

- Read and audited original `src/matrix/core.rs` lines 1–1217, through the end of
  `matrix4_facts`, stopping before `Matrix4TransformDispatchFacts`. This prefix is
  now 1381 lines, with 82 named declarations, 53 fields (including two tuple
  fields), 19 variants and its module introduction documented. Full-text checks
  found all 155 documentation groups in private HTML. These counts do not cover
  the remaining core file, whose original total was 10604 lines.
- Read all of `src/matrix/mod.rs` and the six navigation-only siblings `types.rs`,
  `batch.rs`, `determinant.rs`, `inverse.rs`, `ops.rs`, and `transform.rs`. Added
  documentation for all seven module declarations and made module links absolute.
  Adding outer module comments initially changed inner-doc link resolution;
  warnings-as-errors caught the broken links, and the final build passes. These
  navigation pages do not establish correctness of the later kernels they index.
- Read and documented the eight fact tests in `tests/matrix.rs`, original lines
  134–547: `matrix_exact_facts_expose_dyadic_and_shared_denominator_schedules`,
  the six `matrix_structural_facts_*` tests in that interval, and
  `matrix4_structural_facts_certify_signed_permutation_rows`. The surrounding
  public fact accessors and signed-axis mapping were read as consumer context.
  Neither the entire test file nor later public methods receive whole-file credit.

Documented behavior and limits:

- Matrix entries are public and structural equality delegates to `Real`. Clones
  preserve scalar sharing. Owned transposes move entries; borrowed transposes
  clone in output row-major order. Identity array construction admits dimension
  zero without constructing scalars.
- Fact packets are independently editable snapshots with no source reference or
  mutation tracking. Queries trust stored masks/flags. Changing a matrix leaves an
  older packet unchanged; altering a packet can make its zero-lane hint contradict
  its identity flag. These observations explain the validity preconditions, not
  an authorization for callers to consume invented mathematical certificates.
- Entry masks use row-major bits. Lane counts ignore high bits, while `is_zero`
  compares the whole entry mask. Indexed queries return `None` out of bounds;
  absent zero/one facts are not nonzero/inequality proofs. Sparse support permits
  at most one entry without a zero fact, including all-zero lanes.
- The determinant hint checks zero rows before zero columns, then diagonal,
  triangular, sparse rows/columns, shared denominator, dyadic, exact rational,
  generic fallback. Dense integer ones therefore select shared denominator even
  though they are also dyadic. Valid zero-lane evidence proves singularity;
  computing a full determinant is not always necessary to establish it.
- Transform classification prefers identity over signed permutation in 4D. The
  `Projective` fallback is not an invertibility proof: an all-zero 4×4 matrix
  receives that label and a structurally-zero determinant hint. Linear-block
  diagonal flags do not independently require an affine bottom row. Direction
  flags also require bottom-row xyz zeros, with translation and bottom-right
  entries unrestricted for homogeneous weight zero.
- Shape flags reuse a fixed scan, but complete packets collect rational-set facts,
  zero/one masks, dependencies and other recognizers in separate passes. The 4×4
  nonzero-diagonal scan occurs independently of affine/diagonal shape. Uniform
  scale requires equal exposed rational diagonals, including the homogeneous
  entry; symbolic equality is not attempted. Zero scale is allowed. Signed-axis
  recognition stops at a failing row and then checks column uniqueness.
- Private mask dimensions are preconditions, not typed validation. A temporary
  probe compiled the unchanged seven mask helpers extracted from this revision:
  zero/3/4-dimensional cases passed, while `matrix_lane_known_zero_count::<8>`
  panicked in a debug build despite its `N <= 8` assertion. Current concrete
  matrix callers use only 3 and 4. This is a latent private generic-bound gap,
  not evidence of a failing supported 3×3/4×4 operation. Release behavior and
  arbitrary larger generic instantiations were not tested or repaired.

Validation after the final source edit:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source against `a1f226535` in all nine
  edited Rust files, including every existing test assertion and predicate call.
- Actual Hyperlattice library builds passed with all features and no defaults.
  Direct rustc harnesses linked their matching Cargo artifacts and ran all eight
  original fact tests in both configurations. The all-feature baseline passed
  the same tests. Each process had a 60-second external timeout.
- The 30-second-contained probe passed before/after and with no defaults:
  snapshot mutation, forged masks, ignored lane high bits, whole-mask equality,
  invalid indices, precedence rules, singular fallback and clones. Before/after
  all-feature output matched exactly; the extracted-helper boundary probe passed
  with the expected caught debug panic. These probes are finite witnesses.
- Private library Rustdoc passed with warnings, broken links and private links
  denied in both configurations. Explicit private integration-test Rustdoc also
  passed. Full-text checks verified 155 prefix groups and 14 module/navigation
  groups across 298 inspected HTML pages. The eight newly documented `#[test]`
  functions are source-reviewed and executed; Rustdoc omits their function docs.
- Cargo checks passed for the matrix integration-test target and mathbench, plus
  the Hyperspatial downstream library (including Hyperlimit), with all features
  and no defaults. Commands used `--locked --offline -j 2`, `CARGO_INCREMENTAL=0`
  and the shared `/tmp/csgrs-rustdoc-target`.
- Full workflow checks remain owner-waived. No full matrix runtime suite,
  benchmark timing, release-mode generic-bound probe, binary-size measurement,
  sanitizer or universal matrix correctness/closure claim was added. No runtime
  performance improvement is claimed from these documentation changes.
- Removed the temporary matrix test, semantic probe and mask probe executables.
  The shared Cargo target remains approximately 2.5 GiB; no duplicate target tree
  was created. The existing root AGENTS.md 100% public/private Rustdoc mandate
  remains in force; this partial batch does not satisfy the repository-wide goal.

Evidence: `/tmp/csgrs-rustdoc-matrix-facts-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-facts-audit/` retain temporary checkers, probes and private test
docs. The next core boundary is `Matrix4TransformDispatchFacts`; subsequent
transform dispatch, array mapping, traits, kernels and public operations remain
open. Most of the wider repository also remains unaudited.

## 2026-11-27: Hyperlattice matrix dispatch, powers and generated solves

Base revision: `0f2c988d0`. The previous turn made progress by committing and
pushing the matrix-facts prefix; this batch continues from its recorded boundary.
Owning layer: Hyperlattice's matrix arithmetic and structural dispatch, using
Hyperreal's scalar operations and knowledge. Relevant contracts are P1/P3/P4
(exact arithmetic and scoped conservative facts), P6 (distinct zero/unknown
failures), P10 (predicate/dispatch order), and private documentation availability.
Before editing, review found undocumented dispatch packets, mapping/negation
helpers, power and pivot schedules, generated solve functions, and density
recognizers. Existing tests and the actual-library power/ownership probe passed
on the target revision. This remains documentation-only: no arithmetic, trace
call, predicate order, assertion, fixture, policy, dependency or public signature
changed.

Coverage in this batch:

- Read and audited `src/matrix/core.rs` original lines 1382–2087, from
  `Matrix4TransformDispatchFacts` through
  `multiply_arrays4_rhs_ref_with_dense_certificate`, stopping before
  `invert_matrix4_affine_linear_diagonal`. The block grew from 706 to 889 lines.
  Its 40 named declarations include three macro-generated function templates;
  six fields and three variants also have Rustdoc. All 49 doc groups rendered.
- Read and documented the three `impl_matrix!` power-method templates (`powi`,
  `powi_checked`, `powi_checked_with_abort`) and verified each on both generated
  Matrix3/Matrix4 pages. Consumer context included transform dispatch, transpose
  solve routing, density rechecks and operator hooks. This does not credit the
  entire public API macro, inverse implementation or remaining matrix core.
- Read and documented eleven original matrix tests: `matrix3_inverse_and_power`,
  both `matrix{3,4}_negative_power_matches_repeated_inverse_product` tests,
  `matrix4_identity_and_vector_multiply`,
  `matrix4_translated_diagonal_affine_preserves_point_direction_semantics`,
  `matrix4_negative_homogeneous_weight_stays_on_generic_projective_path`,
  both `matrix{3,4}_division_solves_right_division` tests,
  `checked_matrix3_division_matches_ordinary_right_division`,
  `checked_matrix_inverse_rejects_unknown_zero_pivots`, and
  `ordinary_matrix_inverse_prefers_known_nonzero_pivots`.
  The existing borrowed-matrix test was also used as a consumer witness; its
  earlier whole-file audit is not counted again. The matrix test file remains
  only partially audited.

Documented behavior and evidence:

- Immediate transform dispatch gathers only the needed shape/translation flags.
  Its generic signature requires dimension four, guarded only by a debug assert.
  Predicate groups short-circuit in their existing order. Direction-diagonal
  shape ignores translation and the bottom-right scalar for homogeneous weight
  zero; affine-diagonal shape additionally requires the canonical unit bottom
  row. Neither diagonal recognition nor an absent fact proves invertibility.
- Mapping helpers preserve row-major callback order and left/right ownership.
  Array sizes justify iterator `expect` calls. Empty arrays make no callbacks.
  Negation uses the explicit owned/borrowed row schedules, with scalar authority
  retained in Real.
- Binary powers special-case zero/one/four, and fixed-size wrappers specialize
  two/three. For positive e, the generic helper invokes multiplication
  floor(log2(e)) + popcount(e) - 1 times. This bounds callback count, not scalar
  work or coefficient/expression growth. A temporary extraction of the unchanged
  control flow verified e=0..16,31,32,33 and u32::MAX using empty arrays; it did
  not materialize huge scalar powers. Actual public J matrices (all ones)
  matched independent J^e = n^(e-1) J expectations for e=1..8 and identity at zero.
  Identity at i32::MIN verified unsigned-magnitude handling without scalar growth.
- Dense multiplication requires every entry known nonzero; it does not require
  rationality or imply nonsingularity. Products must be rechecked: the dense
  3×3 matrix with identical rows [1,1,-2] and dense 4×4 matrix with identical rows
  [1,1,-1,-1] both square to zero. Powers two through five matched exact zero.
  Trace assertions verified each dense square/cube branch once and exactly one
  dense dot kernel per nilpotent square/cube, preserving the sparse second step.
- The inverse helpers named "dense" only inspect three nonzero entries to exclude
  triangular/affine shortcuts. They do not establish full support. The
  shared-adjugate heuristic inspects the divisor first: rational 4×4 divisors
  and non-dyadic rational 3×3 divisors can skip the dividend's representation
  scan. This is neither a shared-denominator certificate nor a singularity proof.
  Replaced an obsolete dyadic-only explanation with the actual branch contract.
- Ordinary pivot selection prefers a later known nonzero over an earlier unknown,
  retaining the first unknown only if no known nonzero exists. Checked selection
  instead reports UnknownZero or DivideByZero. It does not start refinement.
  The ordinary selector requires col<N; the checked selector's empty candidate
  range returns DivideByZero, not a separate index error.
- Generated solvers implement coefficients * X = rhs with row swaps, pivot
  normalization and elimination, publishing only the final right buffer. Mutations
  are local to consumed inputs. The pivot slot receives one, not zero; corrected
  the old explanatory comment. Scalar errors propagate without partial success.
  The abort variant checks pivot knowledge before attaching the signal to its
  reciprocal. It does not poll at each row or attach to every matrix entry.
- Public nonnegative `powi_checked_with_abort` does not attach or poll its signal.
  Negative exponents use abort-aware inversion before ordinary power scheduling.
  Corrected the public claim of unconditional attachment. An active-signal
  identity power succeeded with zero `attach-owned-real` trace events. This
  records the implementation's cancellation limitation rather than fixing it or
  promising a deadline. Historical timing comments are not newly reproduced
  performance evidence; no speedup claim is made by this batch.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source against `0f2c988d0` in both edited
  Rust files, including all macro bodies, literal values and test assertions.
- Actual Hyperlattice library builds and twelve selected original tests (eleven
  matrix tests plus borrowed-matrix ownership) passed with all features and no
  defaults. The all-feature baseline passed the same tests. Test processes had
  60-second external limits; semantic probes had 30-second limits.
- The public-library power/ownership probe passed before and after edits with
  identical stdout and exact trace assertions. The no-default build repeated
  semantic assertions with tracing compiled out. Independent expected integer
  matrices covered mixed ownership add/subtract and owned/borrowed negation.
- A temporary probe compiled unchanged extracted mapping/power/pivot/solve
  functions and the original scalar attachment functions, using the actual
  kernels.rs/trace.rs modules and shared test fixtures. Its initial import of
  the crate-private RealKernelExt through the public library failed; corrected
  the probe to include the owning module without changing repository visibility.
  Final all-feature/no-default probes passed callback ordering, empty mapping,
  pivot preference, all six generated solves on permutation systems with
  independent row-order expectations, and UnknownZero before signal attachment.
  These direct helper witnesses do not prove every public division dispatch path.
- Private library Rustdoc and explicit integration-test Rustdoc passed with
  warnings and broken/private links denied in both configurations. Full-text
  verification matched the 49 block groups, each solve template on both generated
  solver pages, and the three public power templates on both matrix pages.
  The eleven new test-function docs are source-reviewed and executed; Rustdoc
  omits #[test] functions. The 298 HTML pages inspected are not 298 audited APIs.
- Cargo checks passed for matrix/borrowed integration targets, mathbench, and the
  downstream Hyperspatial library including Hyperlimit, with all features and no
  defaults. Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared target.
- Full workflow checks remain owner-waived. No full matrix runtime suite,
  benchmark timing, sanitizer, release-mode resource study or code-size
  measurement was run. No universal matrix correctness/composition claim follows.
- Removed four temporary test/probe executables. Shared target storage remains
  about 2.5 GiB, with no second target tree. Root AGENTS.md's 100% public/private
  Rustdoc mandate remains; this batch leaves the repository-wide goal open.

Evidence: `/tmp/csgrs-rustdoc-matrix-solve-*.log`, artifact JSON and temporary
`/tmp/csgrs-matrix-solve-audit/` scripts/probes/private docs. The next contiguous
boundary is `invert_matrix4_affine_linear_diagonal`, currently at line 2272 of
core.rs. Later inverse/division/multiplication/transform kernels, much of the
public macro, remaining tests and most of the wider repository still need audit.

## 2026-11-27: Hyperlattice 4×4 affine inverse and division helpers

Base revision: `855daaf31`. The previous turn made progress by pushing the
power/solve batch. This batch continues at its recorded affine-inverse boundary.
Owning layer: Hyperlattice's exact matrix arithmetic, using Hyperreal scalar
operations and structural knowledge. Relevant properties: P1/P3/P4 for exact
values and valid retained shape facts, P6 for zero/unknown failure, P10 for
unchanged predicate schedules, and P5 for honest resource/cancellation limits.
Before editing, review found 43 undocumented private helpers and misleading
comments about translation, arithmetic schedules and cancellation. Existing
focused tests and an independent rational affine probe passed on this revision.
No executable source, assertion, fixture, policy, trace call, dependency, or
signature changed.

Coverage in this batch:

- Read and audited original `src/matrix/core.rs` lines 2271–3598, from
  `invert_matrix4_affine_linear_diagonal` through
  `divide_matrix4_affine_by_affine_ref_translation`. All 43 functions now have
  Rustdoc; the block grew from 1328 to 1553 lines. This includes ordinary,
  checked, signal-accepting and borrowed adapters, their general/diagonal/
  translation-only formulas, and the shared translation/dense-product helpers.
- Read the relevant ordinary/checked/abort inverse and right-division dispatch
  consumers to establish how their retained shape flags reach these helpers.
  Those later dispatch functions receive context-only credit, not a completed
  public-API or whole-file audit.
- Read and documented four matrix tests:
  `targeted_fractional_matrix_forms_round_trip_exactly`,
  `checked_matrix_inverse_rejects_singular_matrices`,
  `matrix4_known_affine_translation_matches_generic_paths`, and
  `matrix4_known_affine_orthonormal_matches_generic_paths`.
  Also documented the shared `assert_singular_error` helper and `common` module
  declaration. The helper permits both DivideByZero and UnknownZero; its name
  does not mean the test requires a proved singularity. Other matrix tests remain
  only partially audited; previously covered tests are not credited again.

Documented formulas, preconditions and limits:

- Affine inversion uses `[R t; 0 1]⁻¹ = [R⁻¹ -R⁻¹ t; 0 1]`. Diagonal-linear
  helpers compute three reciprocals in axis order; checked helpers finish all
  three known-nonzero guards before constructing those reciprocals. A first
  unknown diagonal may return UnknownZero before a later known zero is inspected.
- The caller must establish affine bottom-row shape and the truth of positive
  retained diagonal/translation flags. Helpers do not revalidate these facts.
  General affine inversion and affine-by-affine division reconstruct the bottom
  row; the unrestricted-dividend helpers compute all four rows. Supplying false
  shape evidence is outside these helpers' preconditions, not certified input.
- Names containing `without_translation` or `no_translation` designate the path
  beyond the translation-only shortcut. They still consume the translation
  column. General affine right division uses `[L a;0 1]/[R b;0 1]` with linear
  block `L R⁻¹` and translation `a - L R⁻¹ b`. The block implementation computes
  the inverse, inverse translation, linear product, and final translation in its
  existing order. Scalar failures precede publication of completed geometry.
- For diagonal-affine divisors, the first three output columns are scaled by
  the diagonal reciprocals and the fourth is corrected by `-D⁻¹ t`. Owned and
  borrowed variants retain their existing cloning and expression association;
  ownership does not imply every consumed coordinate is moved without cloning.
- Pure translations have determinant one, so their shortcuts need no reciprocal.
  Owned division subtracts an active three-term dot of the original translation.
  Borrowed division first negates the translation and adds an ordinary three-term
  linear combination. These are the same algebraic right product with different
  scalar schedules and traces; corrected the old same-schedule comment.
- The active-dot helper forwards retained terms without a local zero scan or
  rationality test. The trace label's "exact" suffix does not restrict inputs
  to rationals. Similarly, affine linear dense multiplication checks known
  nonzero support, not rationality, and rechecks both new operands.
- The signal-accepting affine inverse ignores the signal on diagonal-linear and
  translation-only branches. General block paths forward it to 3×3 inversion;
  later products/sums add no local attachment or polling. Both translation-only
  checked division adapters explicitly ignore the supplied flag. Removed claims
  that determinant one makes abort checks unnecessary: it removes reciprocal
  work, not scalar-expression growth or the need for an explicit work contract.
  No cancellation behavior was changed or promoted into a deadline guarantee.
- Test documentation distinguishes independent inverse expectations from wrapper
  agreement, and authored tenths from a shared reduced denominator. Removed an
  unsupported historical citation and malformed fraction-free attribution from
  local explanatory comments. Existing performance comments are not newly
  reproduced benchmark evidence; this batch makes no performance claim.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source against `855daaf31` in both edited
  Rust files, including all existing assertions and predicate/trace calls.
- Actual Hyperlattice library builds passed with all features and no defaults.
  Nine selected original matrix tests passed before edits with all features and
  after edits in both configurations. They cover the four newly documented tests,
  translated point/direction behavior, 3×3/4×4 division, checked division and
  unknown-zero rejection. Test processes had 60-second external limits.
- The 30-second-contained actual-library probe compared general affine,
  diagonal-affine and translation inverses with explicit rational matrices.
  General linear block [[1,2,3],[0,1,4],[5,6,0]] used its integer inverse and
  independent translated offsets [-95,77,-22]. Diagonal [2,4,8] used an explicit
  denominator-eight inverse; pure translation used negated offsets.
- For each divisor, affine, non-affine and all-zero dividends were compared with
  an independent integer-dot/rational-output oracle: 36 quotient checks across
  owned, borrowed, checked and inactive-signal paths. The probe also checked
  three inversion modes per divisor. This is finite rational evidence, not a
  universal symbolic-equality or affine-closure proof.
- Zero and unknown diagonal failures remained distinct; an all-zero dividend did
  not bypass a singular divisor. Active flags did not prevent the demonstrated
  exact diagonal/translation inverse and translation-division results. Four
  exact trace assertions selected the intended branches once each and observed
  zero `attach-owned-real` events. Before/after all-feature stdout matched;
  no-default mode repeated semantic checks with tracing compiled out.
- Private library and explicit integration-test Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks verified
  each of the 43 helper docs on its own private function page, plus the shared
  test helper/module. The four new #[test] docs are source-reviewed and executed;
  Rustdoc omits test-function documentation. Inspection of 298 HTML pages was a
  rendering check, not a claim to have audited all APIs on those pages.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial library (including Hyperlimit), with all features and no defaults.
  Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared target directory.
- Full workflow checks remain owner-waived. No full matrix test suite, benchmark
  timing, sanitizer, code-size study, arbitrary symbolic-cost envelope or
  universal correctness claim was added. No policy or cancellation test weakened.
- Removed the two temporary test/probe executables. The shared Cargo target
  remains approximately 2.5 GiB; no duplicate target tree was created. The root
  AGENTS.md 100% public/private Rustdoc mandate remains unchanged and unmet for
  the repository as a whole.

Evidence: `/tmp/csgrs-rustdoc-matrix-affine-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-affine-audit/` temporary probes/checkers/private docs. Next
contiguous boundary: `invert_matrix3_by_diagonal`, currently core.rs line 3824.
The remaining 3×3/triangular helpers, later kernels/public APIs, remaining matrix
tests and most of the wider repository still require audit.

## 2026-11-27: Hyperlattice diagonal and triangular matrix schedules

Base revision: `4e8c5b733`. The previous turn made progress by pushing the 4×4
affine-helper audit. This batch continues at `invert_matrix3_by_diagonal`.
Owning layer: Hyperlattice matrix arithmetic with Hyperreal scalar equality,
knowledge and operations. Relevant contracts: P1/P3 for exact structural facts,
P4/P10 for retained evidence and predicate/dispatch order, P6 for distinct scalar
failures, and P5 for accurate work/cancellation claims. Before editing, review
found 38 undocumented helpers, incorrect full-inverse complexity comments, and
an incorrect reciprocal count in a 2D affine explanation. Focused original
tests and the independent rational probe passed on the target revision.
Implementation, signatures, predicates, trace calls, fixtures and assertions
remain unchanged.

Coverage:

- Read and audited core.rs original lines 3824–4726, through
  `divide_matrix3_by_lower_triangular_checked`. The 903-line block is now 1106
  lines and has 38 documented functions. It covers 3×3 diagonal/triangular/2D
  affine inverses, diagonal-affine division, 4×4 diagonal/triangular inverses,
  left/right diagonal multiplication, and 3×3 diagonal/triangular row solves.
- Read supporting scalar-scale and signed-product helpers plus inverse dispatch
  as consumer context. Those later definitions do not receive whole-file or
  completed-public-API credit from this batch.
- Read and documented sixteen original tests: the 4×4 diagonal and uniform-scale
  inverse tests; upper/lower triangular inverse agreement and singular-rejection
  tests in both sizes; and the six 3×3 known-uniform inverse, known-diagonal
  inverse, upper/lower triangular division, checked-upper division and diagonal
  division tests. Result-only tests now explicitly avoid claiming that their
  names establish reciprocal counts. The matrix test file remains partially
  audited; prior test documentation is not counted again.

Documented behavior and corrections:

- Diagonal inverse helpers trust caller-established off-diagonal zeros. They
  compare diagonal values structurally, in index order, to reuse a single
  reciprocal. Equal symbolic representations, including π, can take this route;
  it is distinct from the public rational-only uniform-scale fact. Otherwise
  reciprocals are constructed in order. Checked wrappers finish all ascending
  known-nonzero guards before equality checks or reciprocal construction.
- The explicit 3×3 upper inverse computes off-diagonal entries (0,1), (1,2),
  (0,2); the lower computes (1,0), (2,0), (2,1). The documentation records the
  corner formulas and trusted shape, without changing this arithmetic order.
- The 4×4 triangular inverse loops solve rows of X*A=I. Upper columns increase;
  lower columns decrease, with each inner k loop ascending. Each fixed-size
  helper performs four reciprocal constructions, ten cached result scalings,
  and ten product/subtraction updates. Generalizing the three-index loop to n
  gives cubic scalar-operation work, not a quadratic full inverse. Corrected
  the old O(n²) claims; no scalar bit-complexity or practical-time bound follows.
- Two-dimensional affine inversion forms `[R⁻¹ -R⁻¹t;0 1]`; the general path
  shares one inverse of ad-bc, while the diagonal path uses two reciprocals.
  Positive retained flags and affine shape are caller preconditions. Corrected
  the old three-scale-factor claim for diagonal-affine division and the
  inaccurate rank-one wording for the inverse translation vector.
- Abort-aware 3×3 diagonal-affine inversion attaches both pivot clones before
  either guard, even when the first guard fails. The general affine path
  attaches the determinant before guarding/inverting it. These helpers do not
  explicitly poll the flag or separately attach translations. This differs from
  the 4×4 shortcut behavior documented in the preceding batch; no paths were
  unified or reordered to make their behavior match.
- Left diagonal multiplication scales rows; right multiplication scales columns.
  The helpers clone factors/entries and perform nine or sixteen row-major
  cached multiplications. Local names beginning `inv` do not indicate an inverse:
  factors are used directly and may be zero. Diagonal division constructs
  reciprocals before mutating the consumed dividend buffer.
- Right triangular division solves x*U=b in increasing coordinate order and
  x*L=b in decreasing order. Quadratic work for one variable-size row solve
  becomes cubic for all n rows. The last 3×3 numerator uses an active signed
  product sum. The affine-upper row helper instead uses the zero-pruning
  `mul_sub_add` path and trusts two supplied reciprocal relations and a scalar
  one. Affine-by-affine division reconstructs the canonical bottom row; arbitrary
  dividends require the all-row formula.
- All shape-specific kernels rely on caller-established shape. Checked scalar
  guards do not revalidate it. The first unknown guard may return UnknownZero
  before a later known zero is considered; no general sign refinement or partial
  successful matrix is introduced. Removed misleading fraction-free/dangling
  local commentary without altering the operations it described.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source against `4e8c5b733` in both edited
  Rust files, including attributes, existing assertions and trace/predicate calls.
- Actual Hyperlattice builds and all sixteen selected original tests passed with
  all features and no defaults. The all-feature baseline passed those tests too.
  Tests ran in direct rustc harnesses linked to matching Cargo artifacts, with
  60-second process limits; probes had 30-second external limits.
- A Python standard-library Fraction Gauss–Jordan oracle generated exact expected
  inverses and products independently of the fixed matrix helpers. Actual public
  APIs matched it for 3×3/4×4 upper/lower matrices and general/diagonal 2D affine
  matrices: three inverse modes and four right-division modes per fixture.
  This is finite rational evidence, not a universal inverse or symbolic proof.
- Additional actual-library assertions checked row/column scaling against
  independently formed integer entries, uniform symbolic π inversion, and an
  unknown x pivot before a zero y pivot. Exact trace assertions counted ten
  `mul-cached` events for each named 4×4 triangular inverse; one general-affine
  determinant attachment; two diagonal-affine attachments on success and on
  first-guard failure; and one symbolic uniform-scale branch in each dimension.
  The ten inner product/subtraction updates follow from the inspected finite
  loops, not from a separate runtime counter. No existing count was loosened.
- Before/after all-feature public-probe stdout matched exactly. No-default mode
  repeated all semantic checks with tracing compiled out. These witnesses do not
  cover every shape-specific helper or all cancellation/cache states.
- Private library and explicit integration-test Rustdoc passed with warnings and
  broken/private links denied in both configurations. The initial documentation
  check interpreted two plain matrix indices as links; backtick formatting fixed
  both without changing code. Full-text verification matched all 38 doc groups
  on their individual private function pages. The sixteen new #[test] docs were
  source-reviewed and executed; Rustdoc omits those functions. The 298 inspected
  HTML pages are rendering evidence, not 298 completed API audits.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial library including Hyperlimit, with all features and no defaults.
  Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared target directory.
- Full workflow checks remain owner-waived. No full matrix suite, timing benchmark,
  code-size study, sanitizer, total-work bound or universal closure claim was
  added. Fixed operation counts do not establish a timing or memory improvement.
- Removed the two temporary test/probe executables. The shared Cargo target stays
  near 2.5 GiB; no duplicate target tree was created. The root AGENTS.md mandate
  for 100% public/private Rustdoc remains active; repository-wide completion is
  still unproven and most of the wider repository remains open.

Evidence: `/tmp/csgrs-rustdoc-matrix-tri-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-tri-audit/` temporary oracle, probes, checkers and private docs.
The next contiguous boundary is `affine_inverse_translation2`, currently core.rs
line 4931. Later 2D affine division, 4×4 row solves, multiply/transform/cofactor
kernels, public APIs and remaining tests still need audit.

## 2026-11-27: Hyperlattice affine and triangular right division

Base revision: `8482c599d`. This batch continues at
`affine_inverse_translation2`. Owning layer: Hyperlattice matrix arithmetic,
with Hyperreal scalar arithmetic and structural knowledge. Relevant contracts:
P1/P3 for exact values and conservative facts, P4/P10 for preserved predicate
and dispatch order, P6 for zero/unknown failures, and P5 for work and signal
claims. The domain is exact 3×3/4×4 matrices, with caller-established affine,
diagonal or triangular shape for specialized helpers; dividends may be singular.
Before editing, the target revision had 28 undocumented helpers and incorrect
whole-matrix complexity/row-dependency commentary. Nine selected original tests
and an independent rational probe passed on that revision. Implementation,
signatures, fixtures, assertions, predicates and trace calls remain unchanged.

Coverage:

- Read the contiguous core.rs block originally at lines 4930–5850, from the
  2D inverse-translation helper through
  `divide_matrix4_by_lower_triangular_checked`. The 921-line block is now 1075
  lines, including Rustdoc for all 28 private functions. Neither this matrix
  implementation nor its test file receives whole-file credit yet.
- Read the later checked right-division dispatcher, `mul_add`, and scalar adapter
  context to verify callers, structural-zero pruning, and signal attachment.
  These contextual reads do not complete the later definitions' own audit.
- Read and documented four original tests: known-diagonal 4×4 division,
  upper/lower triangular 4×4 division agreement, and checked upper-triangular
  division agreement. Their docs distinguish shared-kernel expectations and
  inactive-signal dispatch from independent evidence and cancellation tests.

Documented behavior and corrections:

- The small 2D helpers distinguish original translation t from inverse
  translation -R⁻¹t, retain the ordered two-product `mul_add` schedule, and do
  not establish shape or inverse relations themselves. Unknown zero status
  retains a product; structural-zero knowledge can omit it.
- Pure-translation division copies the first two columns and subtracts their
  action on t from the final column. Arbitrary dividends require all three
  rows; known affine dividends update two offsets and reconstruct `[0,0,1]`.
  Owned and borrowed 3×3 paths use the same subtraction helper. No reciprocal
  is needed, but determinant one does not bound expression work. Corrected a
  comment that inaccurately described changes to the third row and column.
- General 2D affine division forms ad-bc once, shares its reciprocal across the
  adjugate, and constructs inverse translation before row products. Borrowed
  helper names ending `no_translation` exclude the translation-only shortcut;
  they still handle nonzero translations. Affine-by-affine paths reconstruct
  the canonical bottom row, while unrestricted paths compute every row.
- Checked affine division guards the determinant before inverting it. Signal
  variants attach the determinant before the guard, including guard failures;
  later products do not explicitly poll or add attachments. Scalar propagation
  determines cancellation. Shape remains a precondition, and a checked scalar
  guard is not a shape proof or a total-work bound.
- Four-dimensional diagonal division constructs four reciprocals in index order
  and then uses sixteen row-major cached multiplications. Upper/lower row solves
  share four reciprocals across all rows and solve increasing/decreasing
  coordinates respectively. Each row is independent of other dividend rows.
  Their final two numerators use three/four-term active signed sums without a
  local pruning pass. Corrected old whole-matrix O(n²), one-inverse-per-row and
  inter-row dependency claims: generalized work is quadratic per row and cubic
  for n rows, without a scalar bit-complexity or elapsed-time guarantee.
- Upper-affine 4×4 division shares three reciprocals. The unrestricted path uses
  twelve cached scalings; the affine dividend uses nine and reconstructs its
  bottom row. The final homogeneous coordinate needs no scaling by a reciprocal
  of one. The row helper trusts supplied reciprocals and the supplied scalar one.
- Checked diagonal/triangular wrappers finish ascending known-nonzero guards
  before reciprocal construction, even when the subsequent solve descends.
  First zero/unknown errors propagate without a partial successful quotient.
  No predicates, guards, arithmetic order or dispatch paths were changed.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Stripping comment-only and blank
  lines leaves identical executable source in both edited Rust files against
  `8482c599d`, including attributes, assertions, trace calls and scalar schedules.
- Hyperlattice builds and all nine selected original tests passed with all
  features and no defaults; the all-feature baseline passed the same tests.
  Direct rustc harnesses used matching Cargo artifact graphs, with 60-second
  per-test and 30-second public-probe external limits.
- A Python standard-library Fraction Gauss–Jordan oracle independently generated
  expected quotients for seven rational divisors: general affine, translation
  and upper-affine 3×3 matrices, and upper/lower/upper-affine/diagonal 4×4 matrices.
  Two dividend shapes and four modes (owned, borrowed, checked, inactive-signal
  checked) produced 56 exact quotient comparisons in each configuration. These
  are finite rational witnesses, not a universal symbolic or cancellation proof.
- Additional actual-library checks preserved DivideByZero for a singular affine
  divisor even with a zero dividend, and UnknownZero for an unproved determinant.
  Exact all-feature trace assertions counted sixteen `mul-cached` events for
  full 4×4 upper/lower/diagonal division, twelve for full upper-affine division,
  and nine for affine-by-upper-affine division. Translation division recorded
  two or three column updates for affine or unrestricted dividends, in both
  ownership forms. General 2D signal-aware affine division recorded exactly one
  determinant attachment for both successful dividend shapes and zero/unknown
  determinant failures. No existing count assertion was loosened.
- Before/after all-feature probe stdout matched exactly. No-default mode repeated
  semantic checks with trace assertions compiled out. Both final probe builds
  were warning-free; temporary trace-only bindings were scoped to that feature.
- Private library and integration-target Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks found
  all 28 new function docs on their individual private pages. Four newly
  documented #[test] functions were source-reviewed and executed; Rustdoc omits
  them. Shared test module/helper rendering was rechecked without new coverage
  credit. The 298 inspected HTML pages do not imply 298 completed API audits.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial library, including Hyperlimit, with all features and no defaults.
  Commands used --locked --offline -j 2, CARGO_INCREMENTAL=0 and one shared target.
- Full workflow checks remain owner-waived. No full matrix-suite, benchmark,
  sanitizer, code-size, total-work or universal-closure claim is made. No focused
  check remains blocked. Removed the two temporary harness/probe executables;
  the shared target remains near 2.5 GiB and no duplicate target tree was created.
- The root AGENTS.md 100% public/private Rustdoc mandate remains active.
  Repository-wide completion is still unproven; most wider repository work is open.

Evidence: `/tmp/csgrs-rustdoc-matrix-div-*.log`, matching Cargo artifact JSON,
and `/tmp/csgrs-matrix-div-audit/` temporary oracle, probe source, checkers and
private docs. The next contiguous boundary is `right_divide_matrix3`, currently
core.rs line 6005. Remaining dispatch, multiplication, transforms, cofactors,
public APIs and tests still require audit.

## 2026-11-27: Hyperlattice right-division dispatch and shared adjugates

Base revision: `de2ce251c`. The preceding goal turn made progress by committing
and pushing affine/triangular row-helper documentation. This batch starts at
`right_divide_matrix3`. Owning layer: Hyperlattice matrix dispatch over Hyperreal
scalar facts/arithmetic. Relevant contracts: P1/P3 for exact representation and
conservative structural knowledge, P4/P10 for cache state and dispatch/predicate
order, P5 for cancellation/work limits, and P6 for typed scalar failures. Inputs
are arbitrary exact 3×3/4×4 matrices; specialized shape/rational paths establish
their prerequisites through the existing selectors. The dividend need not be
invertible. Before editing, twelve helpers lacked Rustdoc, a 4×4 comment still
restricted shared adjugates to dyadics, and a triangular comment incorrectly
assigned quadratic work to a whole matrix quotient. The original tests and
independent baseline probes reproduced the current paths before source edits.

Coverage:

- Read core.rs original lines 6005–6893, all 889 lines of the eight owned,
  borrowed, checked and signal-aware dispatchers, the early 4×4 rational-route
  predicate, and three shared-adjugate helpers. The block is now 984 lines;
  all twelve functions have Rustdoc. The matrix core remains partially audited.
- Re-read rational-class selectors, three-entry density witnesses, borrowed
  multiplication certificate handling, affine guard/translation/signal helpers,
  and the rational-minor/product kernels as supporting context. These reads do
  not complete later kernels' own documentation audit.
- Re-read the 3×3/4×4 quotient-identity tests, checked-division agreement and
  unknown-pivot fixtures. They were already documented; this batch changes no
  test files and claims no additional whole-file or test-documentation coverage.

Documented behavior and corrections:

- Right division solves X*B=A. Non-specialized Gauss–Jordan works on Bᵀ*Xᵀ=Aᵀ;
  shared-adjugate paths construct one determinant reciprocal and scale the
  adjugate product last. The 3×3 path forms its adjugate with the determinant;
  4×4 paths invert the determinant before building the adjugate.
- Shape priority is identity, diagonal, translation, upper affine with
  nondiagonal linear block, other upper triangular, lower triangular, and
  remaining affine. Affine diagonal-linear divisors deliberately bypass general
  upper-triangular division. Left shape facts are requested only for affine
  specialization; rational-class inspection is a separate operation.
- The 4×4 early route first tests entries (1,0), (0,1), (3,0) for known nonzero
  status, then requires rational exposed representations. Those witnesses
  exclude shape shortcuts but do not prove full entry density or invertibility.
  The known-rational minor kernels accept zeros elsewhere. Rational dividends
  select a known-rational product; symbolic dividends use fresh product-density
  certification and the ordinary fallback as needed.
- Remaining 4×4 shared-adjugate selection accepts any rational divisor, including
  non-dyadics and symbolic dividends. For 3×3, non-dyadic rational divisors qualify
  directly; dyadic divisors also require a rational dividend. Corrected the old
  dyadic-only statement. Unsupported timing comparisons and generic literature
  attributions were replaced with the local scheduling facts they described.
- Corrected whole-matrix triangular complexity: the variable-size schedule is
  quadratic in scalar operations per row and cubic across n rows. No scalar
  bit-complexity, timing or memory improvement follows from these fixed schedules.
- Checked paths guard pivots/determinants unless existing nonzero facts select
  an ordinary specialization. Zero/unknown failures propagate without a partial
  successful quotient. A zero dividend does not bypass divisor validation.
- Owned ordinary 4×4 division of an unrestricted dividend by a diagonal-linear
  affine divisor uses checked guards when nonzero diagonal knowledge is missing.
  Borrowed ordinary division and the ordinary affine-dividend branch do not add
  those guards. The documentation preserves this existing behavioral distinction.
- Cold-process probes with the shared near-zero sine fixture reproduced
  UnknownZero for owned unrestricted, checked unrestricted, checked affine, and
  signal-aware checked division. Borrowed unrestricted and ordinary affine-left
  division constructed results. In a separate warm sequence, borrowed arithmetic
  strengthened shared scalar knowledge and a subsequent checked call succeeded.
  This is knowledge-dependent dispatch, not proof that the cold unknown was zero.
- An initial combined temporary probe incorrectly expected the checked call to
  remain UnknownZero after borrowed arithmetic had warmed the shared data. Kept
  its source as `edge-initial.rs`, separated cold cases into fresh processes,
  and added an explicit warm-case assertion. No repository fixture or assertion
  was weakened. These probes check construction/error behavior; they do not claim
  full evaluation or a general completion bound for arbitrary symbolic inverses.
- Identity, diagonal, translation, triangular and diagonal-linear affine
  signal-aware shortcuts do not consume the flag (translation adapters may
  accept and ignore it). General affine/solve branches forward to their scalar
  routines; shared adjugates attach the determinant after construction and before
  guarding/inverting it. Accepting a flag does not promise immediate rejection,
  preemptive cancellation, or a bound on pre-attachment work.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source against `de2ce251c`; all attributes,
  scalar calls, guards and dispatch traces remain unchanged.
- Hyperlattice built with all features and no defaults. Ten selected original
  integration tests passed in both configurations and on the all-feature
  baseline, using direct rustc harnesses linked to matching Cargo artifacts.
- An independent Python Fraction Gauss–Jordan oracle supplied 176 exact rational
  quotient expectations: eleven divisor families, two dimensions, two dividend
  shapes and four ownership/checking modes. Every all-feature case asserted one
  event for its expected selected route. Both final feature configurations passed
  the semantic cases; all-feature baseline/final probe stdout matched exactly.
- Symbolic monomial divisors returned identity under four Gauss–Jordan modes in
  both dimensions. Rational all-ones singular divisors returned DivideByZero for
  zero dividends in all four modes. Additional edge probes checked sixteen exact
  symbolic-dividend quotients against independently formed rational coefficients
  times π, including dyadic/non-dyadic 3×3 selection and both 4×4 adjugate routes.
  All-feature edge cases also asserted their expected route count; no-default
  mode repeated semantics with tracing compiled out.
- For the 44 rational signal-aware cases, trace assertions counted zero
  attachments on 28 shape shortcuts and one on sixteen general-affine/shared
  routes. The 28 shortcuts also returned the exact oracle quotient with an
  already-set flag. This validates the documented limited signal behavior,
  not cancellation of arbitrary scalar work. No existing count was loosened.
- Seven fresh-process edge invocations per configuration covered the six cold
  outcomes and one deliberately warm sequence above. External limits were
  60 seconds per original test and 30 seconds per probe invocation. The final
  temporary probes compiled warning-free after removing redundant parentheses.
- Private library and integration-target Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks located
  all twelve new doc groups on their individual private function pages. The
  298 inspected HTML pages are rendering evidence, not completed API audits.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial library including Hyperlimit, with all features and no defaults.
  Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared target directory.
- Full workflows remain owner-waived. No full matrix suite, benchmark, sanitizer,
  binary-size, total-work or universal closure claim was added. No focused check
  remains blocked. Removed three temporary executables; the shared target remains
  near 2.5 GiB and no duplicate target directory was created.
- The root AGENTS.md mandate still requires 100% public/private Rustdoc. This
  batch extends demonstrated coverage; repository-wide completion remains unproven.

Evidence: `/tmp/csgrs-rustdoc-matrix-routing-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-routing-audit/` temporary sources, oracles, checkers and private
Rustdoc. The next contiguous boundary is `multiply_arrays3_borrowed`, currently
core.rs line 6990. Multiplication/transform/cofactor kernels, public APIs and
remaining matrix tests still require audit.

## 2026-11-27: Hyperlattice matrix multiplication and support facts

Base revision: `079719a29`. The preceding goal turn made progress by pushing the
right-division dispatcher audit. This batch continues at
`multiply_arrays3_borrowed`. Owning layer: Hyperlattice matrix multiplication
with Hyperreal structural facts and exact scalar reducers. Relevant contracts:
P1/P3 for exact arithmetic and conservative zero knowledge, P4/P10 for valid
retained evidence and unchanged predicate/dispatch order, P5 for honest resource
claims, and P6 for explicit caller preconditions. The domain is arbitrary Real
3×3/4×4 matrices, plus a private const-generic structural-fact helper. The
known-rational specialization additionally requires current rational factor
representations. Before editing, fifteen functions, a private type and its two
fields lacked Rustdoc. Original tests and independent mask/product probes passed
on the target revision. No implementation, API, fixture, assertion or policy changed.

Coverage:

- Read all of core.rs original lines 6989–7611, including the initial attribute:
  623 lines covering borrowed sparse/dense products, three 3×3 certificate
  wrappers, the known-rational 4×4 product, identity/diagonal facts, and six
  ownership dispatchers. The block is now 743 lines. Eighteen Rustdoc groups
  document fifteen functions, one type and two fields.
- Read scalar dot/active-sum adapters and the known-rational scalar reducer,
  public ownership-operator expansion, cubing/division call sites and existing
  product/inverse/ownership tests as context. These reads do not complete later
  public APIs or the remaining matrix implementation. No test documentation is
  newly counted and neither matrix source nor test file receives whole-file credit.

Documented behavior and corrections:

- Ordinary borrowed kernels scan all left entries, then all right entries, in
  row-major order using `!definitely_zero()`. These flags retain unknown lanes;
  they are not positive nonzero proofs. The ordinary dense branch means no
  definitely-zero lane was found, not full support certification or invertibility.
- Each sparse cell intersects its row/column flags and keeps active lane order.
  No products gives exact zero; one uses scalar multiplication; two uses an
  ordered active sum; the 3×3 full cell uses dot3. Four-dimensional cells use
  active sums for two, three or four terms. All output cells are constructed
  row-major. The unreachable two-term arm is excluded by the four counted
  booleans, not by a mathematical sign assumption.
- The 4×4 two-term case for lanes zero and one uses exactly those lanes. Replaced
  historical/performance commentary with this invariant and its consequence for
  sparse inverse-product checks. The finite mask corpus retains all subsets,
  including the pair singled out by the old comment.
- Dense kernels do not perform matrix-level sparsity checks; scalar-owned dot or
  active-sum behavior still applies. The three certificate wrappers instead
  require current known-nonzero status for every left entry, then every right
  entry if the left passes. Unknown rejects this shortcut while preserving the
  product via ordinary dispatch. The names do not imply rationality checks.
- Product support must be re-established for subsequent operations: dense inputs
  can multiply to exact zero. The cubing wrapper checks the intermediate square
  rather than reusing the source's support. None of these schedules proves a
  scalar-work, expression-growth or cancellation bound.
- The known-rational 4×4 kernel accepts zero lanes but requires both operands'
  current factors to expose exact rational representations. Its scalar reducer
  reads stored rational components without class checks; symbolic factors or
  stale evidence can change the value rather than return a typed mismatch.
  Documented this caller obligation without adding a check or broadening a claim.
- The small fact helper checks off-diagonal zeros row-major and short-circuits,
  then checks diagonal ones only after diagonal structure succeeds. Its N=0
  tests are vacuously true; N=1 is always diagonal. The Copy type is an unbound
  snapshot, so mutation requires recomputation. False means this recognition
  failed, not necessarily proved mathematical inequality.
- All six ordinary ownership dispatchers collect left and right fact snapshots
  before branching. Priority is left identity, right identity, left diagonal,
  right diagonal, then the borrowed active-cell kernel. Identity returns or
  clones the opposite array according to ownership; diagonal action scales rows
  or columns. Borrowed-left/owned-right operators use the same helper as two
  borrowed operands, as confirmed in the generated public implementations.
- Borrowed dispatch records its specialized event on entry, even for identity
  or diagonal products. Owned and owned-left/borrowed-right paths record their
  specialized event only on fallback. These distinctions and all scalar schedules
  remain unchanged. Removed unqualified historical benchmark-win commentary;
  this docs batch makes no new performance claim.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Stripping comment-only and blank
  lines leaves identical executable source against `079719a29`, preserving all
  attributes, predicates, reductions, constants, traces and assertions.
- Actual Hyperlattice builds and nine selected original tests passed with all
  features and no defaults, as well as on the all-feature baseline. Eight matrix
  tests cover inverse/product identities, negative powers, fractional fixtures,
  triangular inverses and division identities; one borrowed-ops test checks
  ownership agreement. Direct rustc harnesses used matching Cargo artifact graphs.
- The independent integer triple-sum oracle checked every pair of row/column
  support masks: 64 for 3×3 and 256 for 4×4, in four ownership forms, for 1280
  complete product comparisons. Other rows/columns retained nonzero entries so
  each pair reached the borrowed cell kernels. All-feature assertions required
  one expected specialized event and one dense/sparse event for each case.
- Eight identity/diagonal priorities per dimension, including two identities,
  two diagonals and zero operands, passed in four ownership forms: 64 more exact
  products with exact branch and specialized-event counts. No count was loosened.
- An isolated unknown lane survived exactly in a one-product cell in each size.
  Separate ordinary dense cases retained unknown entries and confirmed route
  selection; these cases test construction/dispatch, not full symbolic evaluation.
  Cubing matched an independent integer cube for a dense matrix and exact zero
  for the dense rank-one fixture whose row is [1,1,-2]. The latter asserted zero
  uses of the owned-ref dense-certificate branch after the square collapsed.
- A 3×3 shared-adjugate quotient with all-one dividend and diagonal-five/off-
  diagonal-one divisor returned exact 1/7 in every cell, exercising owned and
  borrowed dense-certificate multiplication. Additional final 4×4 quotient probes
  used a Python Fraction Gauss–Jordan inverse oracle, zero-containing operands,
  four division modes and exactly one known-rational product event per mode.
- Extracted the actual private fact type/helper for direct N=0, N=1 identity/zero
  and mutation checks. These assertions exercise the existing source, not an
  independently rewritten implementation. The empty dimension is private-helper
  evidence, not a newly supported public matrix dimension.
- Before/after all-feature public and private probe stdout matched exactly.
  No-default mode repeated all semantic checks with traces compiled out; all
  final probes were warning-free. Original tests had 60-second process limits,
  probes 30-second limits. The finite corpus is not universal symbolic closure.
- Private library and integration-target Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks found
  all eighteen groups on the individual function pages and the type page,
  including both fields. The 298 inspected HTML pages are rendering evidence, not completed API audits.
- Cargo checks passed for matrix and borrowed_ops integration targets, mathbench,
  and downstream Hyperspatial including Hyperlimit, with all features and no
  defaults. Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and one shared target.
- Full workflows remain owner-waived. No full matrix suite, runtime benchmark,
  sanitizer, code-size or total-resource study was claimed. No focused check
  remains blocked. Removed five temporary executables; the shared target remains
  near 2.5 GiB and no second target tree was created.
- The root AGENTS.md mandate for 100% public/private Rustdoc remains in effect.
  Repository-wide completion remains unproven and the wider audit remains open.

Evidence: `/tmp/csgrs-rustdoc-matrix-multiply-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-multiply-audit/` temporary generators, source extracts, probes,
checkers and private docs. The next contiguous boundary is
`transform_vector_rhs_ref`, currently core.rs line 7732. Transform/cofactor
kernels, public APIs and remaining matrix tests still require audit.

## 2026-11-27: Hyperlattice vector transforms and immediate batch handles

Base revision: `3c077382e`. The preceding turn made progress by pushing the
matrix-multiplication audit. This batch continues at `transform_vector_rhs_ref`.
Owning layer: Hyperlattice matrix/vector transforms over Hyperreal scalar facts
and linear forms. Relevant contracts: P1/P3 for exact values and conservative
knowledge, P4/P10 for retained facts and predicate order, P5 for allocation/work
claims, and P6 for reciprocal failures and assumed-input preconditions. Inputs
are exact 3D vectors and 4D homogeneous vectors; named point/direction helpers
require the declared w=1/w=0 facts, while general paths admit other weights and
projective matrices. Before editing, 31 items lacked Rustdoc, one batch method's
docs omitted material preconditions, and comments overstated nonzero evidence,
affine-only scope and allocation avoidance. The selected original tests and
independent public probes passed on the target revision before edits.

Coverage:

- Read all of core.rs original lines 7732–8642, through both immediate batch
  handles: 911 lines, now 1032. Reviewed/documented 13 free functions, nine
  methods, two private types and eight fields, for 32 Rustdoc groups. One
  existing point-batch doc was expanded; the other 31 groups were missing.
- Read public transform, Cartesian conversion, diagonal-divisor prescaling,
  vector-kind and scalar linear-form consumers as context. These contextual
  reads do not complete the later public API's own audit. The matrix source
  and test file remain only partially audited.
- Added docs to twelve original tests: assumed homogeneous batches; 3D diagonal
  divisor/vector composition; two 4D point/divisor compositions; seven exact
  rational/sign/zero/homogeneous transform fact tests; and pointwise/batch
  agreement. The existing translated-diagonal and negative-weight test docs
  were not counted again. Tests now distinguish literal expectations from
  shared-implementation agreement and identify the no-translation fixture
  whose name suggests translation.

Documented behavior and corrections:

- The const-generic one-shot helper serves N=3/N=4 callers; the signature is not
  a general-dimensional transform contract. Identity and diagonal shortcuts
  precede homogeneous classification in 4D. Directions omit the final column,
  points add it as offsets, and minus one/other weights remain general. All
  output rows are computed; a projective matrix may turn input w=0 into nonzero w.
- Cached zero flags authorize omission only when true. The field and helper
  names containing all-nonzero mean every definite-zero flag is false, so they
  also admit unknown coefficients. Corrected comments that claimed guaranteed
  nonzero offsets. Active linear forms describe scheduled arithmetic, not
  positive nonzero or rationality proofs.
- The separate scaled-weight helper computes each row's spatial linear form plus
  the final-column coefficient times the supplied w scale. It works for projective
  matrices, contrary to the old affine-only comment. It does not read right[3];
  its caller must correlate the separate scale and one flag with the prescaled
  point. All-retained offsets are precomputed before row forms, with the active
  three-term kernel; partial offsets retain ordinary linear forms and row checks.
- Direction-diagonal facts require both a diagonal top-left block and zero
  bottom xyz entries, permitting three scales and output zero w. Missing shortcut
  evidence does not prove the opposite shape. Assumed direction batches choose
  the schedule once, preserve input order, and do not classify individual weights.
- Affine diagonal-linear single-point transforms clone the input w=1 scalar;
  the corresponding point-batch loop creates fresh ones. Both require valid point
  assumptions. Cached point helpers can also receive non-affine diagonal matrices;
  corrected the claim that all diagonal cases had already been excluded.
- The 3D batch helper's dense label only means the identity/diagonal shortcuts
  were bypassed; it does not prove dense support. Both batch handles hold shared
  references, preventing ordinary geometric mutation during their lifetime.
  The 4D alternate constructor trusts its supplied report, reuses xyz column-zero
  facts, queries m33 and derives summaries without validating report origin.
- Generic 4D batch dispatch treats identity and diagonal separately. Other
  batches scan until two homogeneous kinds appear; that early stop proves a
  mixture rather than recording every later kind. Uniform batches reuse a
  schedule; mixed batches allocate per-vector facts and classify every input
  again, so a third kind later in the slice still dispatches from its own facts.
- Corrected the allocation-free claim: uniform batches avoid the per-vector
  fact array but still construct owned output storage. Empty inputs return empty
  output. Fixed arity/input count is not a scalar-expression or total-memory bound.
- Cartesian point conversion moves xyz directly for definite w=1; otherwise it
  takes one ordinary w reciprocal and scales xyz in order. Known zero returns
  the inverse error. Unknown status is not checked here and can produce a symbolic
  construction; success is not a separately established nonzero certificate or
  completed numerical evaluation. No epsilon or point-at-infinity substitute
  is introduced. Public Cartesian batches return the first conversion failure
  rather than a successful partial point list.
- Removed vague policy/literature references and unsupported local performance
  assertions where the actual retained-fact or cloning schedule suffices. No
  executable statements, predicates, trace events, assertions or fixtures changed.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Removing comment-only and blank
  lines leaves identical executable source in both edited Rust files against
  `3c077382e`, including guards, traces, branches and existing assertions.
- Hyperlattice builds and all fourteen selected original matrix tests passed
  with all features and no defaults; the all-feature baseline passed the same
  tests. Direct rustc harnesses used matching Cargo artifact graphs and
  60-second process limits; public probes had a 30-second external limit.
- Independent Python integer/rational row sums supplied expectations for eight
  4×4 matrix shapes: identity, diagonal, affine diagonal, all-zero/fully-retained/
  partial final columns, direction-diagonal projective and general affine.
  Weights 0, 1, -1 and 2 were checked through four ownership operators and the
  one-shot method, with exact selected-route counts in all-feature mode.
- Each shape also passed singleton generic batches, valid assumed point/direction
  calls, uniform directions/points/general weights, both orders of mixed batches,
  and empty batches. Mixed [0,1,-1] cases exercise the later third kind after the
  first-pass stop; [-1,0,1] exercises the alternate mixture. Exact top-level
  mixed/general-weight trace counts were asserted on the selected paths.
- Six independent scaled-point expectations covered zero/retained/partial final
  columns with divisor d3 equal to one and seven, including projective bottom
  rows. The expected scaled-weight branch count was one in each case.
- Identity, diagonal, permutation and general 3×3 transforms matched independent
  integer values through four ownership modes and nonempty/empty batches.
  Existing scalar-fact tests separately cover exact rationality, signs, zero
  lanes and symbolic π/e coefficients; sampled membership is not used as an oracle.
- Cartesian point probes checked literal rational normalization, DivideByZero
  at output w=0, a batch with an earlier valid point followed by that failure,
  empty input, and successful construction with the existing unknown-zero-status
  sine fixture as weight. The last check does not claim arbitrary symbolic
  termination or verify a general nonzero certificate.
- Before/after all-feature probe stdout matched exactly. No-default mode repeated
  all semantics with tracing compiled out. Final tests/probes were warning-free;
  no existing trace-count assertion or validation fixture was weakened.
- Private library and integration-target Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks found
  the 32 core groups on their function/type pages, including all methods and
  fields. Twelve newly documented #[test] functions were source-reviewed and
  executed; Rustdoc omits them. The 298 inspected HTML pages are rendering
  evidence, not 298 newly completed API audits.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial library including Hyperlimit with all features and no defaults.
  Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and one shared target directory.
- Full workflows remain owner-waived. No full matrix suite, benchmark, sanitizer,
  allocator profile, binary-size or universal symbolic closure claim was added.
  No focused check remains blocked. Removed the two temporary executables;
  the shared target remains near 2.5 GiB and no duplicate target tree was created.
- The root AGENTS.md 100% public/private Rustdoc mandate remains active.
  Repository-wide completion remains unproven; the wider audit is still open.

Evidence: `/tmp/csgrs-rustdoc-matrix-transform-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-transform-audit/` temporary generator, probes, checkers and
private docs. The next contiguous boundary is `scale_by_shared_factor`, currently
core.rs line 8764. Scalar combination/cofactor/inverse kernels, public APIs and
remaining matrix tests still require audit.

## 2026-11-27: Hyperlattice signed products and 3×3 cofactor inversion

Base revision: `6f898839f`. The preceding goal turn made progress by pushing the
vector-transform/batch audit. This batch continues at `scale_by_shared_factor`.
Owning layer: Hyperlattice scalar schedules and matrix cofactors, delegating
arithmetic/representation decisions to Hyperreal. Relevant contracts: P1/P3 for
exact polynomials and conservative zero knowledge, P4/P10 for retained evidence
and predicate/dispatch order, P5 for work/cache claims, and P6 for reciprocal
failures and unchecked representation preconditions. Inputs are Real factors
and exact 3×3 matrices; known-rational/dyadic helpers require their stated
current representations. Before editing, 22 functions lacked Rustdoc, several
comments contained incomplete references or unsupported performance comparisons,
and a test description incorrectly suggested pivot selection. Baseline original
tests and public/private probes reproduced current behavior before source edits.

Coverage:

- Read core.rs original lines 8764–9396: all 633 lines from shared scaling
  through `invert_matrix3_checked_with_abort`, now 749 lines. All 22 private
  functions have Rustdoc: shared scale passes, pruned/active signed products,
  determinant/adjugate construction, scaled adjugates and three inverse dispatchers.
- Read the later rational aggregate adapter, scalar known-rational inverse
  entry points, existing arithmetic adapters/trace forwarding, public determinant
  consumer and selected matrix tests as context. Those reads do not complete
  later definitions' own audits. Matrix source and tests remain partially audited.
- Corrected the existing doc for
  `ordinary_matrix_inverse_prefers_known_nonzero_pivots`: the current fixture
  reaches `matrix3_scaled_adjugate`, not a pivot search. The assertion checks
  construction success only. No test names, bodies or expectations changed;
  this is a correction to earlier documented coverage, not a newly counted test.

Documented behavior and corrections:

- Shared scaling consumes entries and borrows one factor, with nine or sixteen
  row-major `mul_cached` calls. The factor can be zero or any Real; these helpers
  neither invert it nor prove a determinant relation. The adapter owns no cache.
  Replaced incomplete fraction-free-elimination references with this actual
  elementwise schedule and scalar-owned sharing semantics.
- Pruned two/three-product helpers inspect pairs in source order, left before
  right within each short-circuit pair, and finish the status checks before
  arithmetic. Unknown factors are retained. Zero/one/two/three active subsets
  preserve original lane order and signs. The unreachable pruned match arms
  follow from small integer counts, not from scalar sign assumptions.
- `mul_sub_add` and both corresponding dense variants compute A-B-C, including
  a negative third-only term. Their names must not be read as A-B+C. Documented
  the exact polynomial explicitly without renaming or changing any caller.
  `mul_add_sub` instead uses A+B-C; two-term helpers use A-B or A+B.
- Dense-exact names without a known-representation suffix still use active
  scalar selection and admit symbolic factors. Known-rational/dyadic variants
  skip local representation checks and accept zeros, but require valid current
  caller evidence. Invalid evidence is not guaranteed to yield a typed error;
  the scalar reducer can read only stored rational components.
- The 3×3 determinant uses three signed minors and first-row dot expansion,
  without reciprocal or pivot selection. The adjugate helpers construct all nine
  transposed cofactors row-major, then pair the source first row with the adjugate
  first column for the determinant. Singular matrices still have these outputs.
- Dense adjugate construction trusts a true known-dyadic flag for every entry;
  false selects ordinary active arithmetic. Scaled adjugates take one ordinary
  determinant reciprocal after cofactor construction, then scale the output.
  Unknown determinant status is not a checked rejection on ordinary paths.
- The dense scaled helper scans rational representations row-major until the
  first missing view. A non-dyadic entry alone does not stop this scan: the
  aggregate branch requires a completed rational scan. All-dyadic data uses
  known-dyadic minors/determinant; missing representation selects ordinary
  active reducers. The ordinary dispatcher already tries its rational aggregate
  before this fallback; direct helper probes also exercise its independent branches.
- Inversion first tries three nonzero witnesses excluding triangular/affine
  shortcuts, not certifying full density or nonsingularity. Ordinary/checked
  paths then try a rational aggregate whose error is final. Otherwise shape
  priority is identity, diagonal, upper triangular, lower triangular, affine,
  then general cofactors; upper handling excludes affine diagonal-linear cases.
- The signal-aware dense path skips the rational aggregate, constructs its
  adjugate/determinant and attaches the signal before the determinant guard.
  General fallback does the same. Identity/diagonal/triangular shortcuts do not
  consume the flag; affine paths forward it to their own attachment schedule.
  Construction before attachment is not covered by a preemptive work bound.
- Removed unsupported timing-win statements and broken citations from this block.
  No representation, arithmetic, scalar-call order, predicates or traces changed.

Validation after final source edits:

- Focused Rustfmt and `git diff --check` passed. Stripping comment-only and blank
  lines leaves identical executable source in both edited Rust files against
  `6f898839f`, including attributes, trace events and assertions.
- Hyperlattice builds and eleven selected original matrix tests passed with all
  features and no defaults; the all-feature baseline passed the same tests.
  Tests cover inverse/product composition, fractional inputs, triangular and
  diagonal inverse agreement, singular/unknown rejection and right division.
  Direct rustc harnesses used matching Cargo artifact graphs.
- Independent Python Fraction Gauss–Jordan inversion and signed permutation
  determinant sums supplied expectations for nine rational fixtures: identity,
  diagonal, upper/lower triangular, diagonal-linear affine, general affine,
  general fallback, dense dyadic and dense non-dyadic rational. Ordinary,
  checked and signal-aware inverses matched, with selected-route counts and
  rational-aggregate usage asserted in all-feature mode.
- Public symbolic probes verified a literal π-dependent dense inverse and
  determinant, plus successful construction and exactly one scaled-adjugate
  event for the unknown-diagonal test fixture. All-ones singular matrices
  returned DivideByZero in all three inverse modes. The signal-aware failure
  attached its determinant exactly once before rejection. Unknown construction
  is not a completed-evaluation or general nonzero-certification claim.
- Extracted the actual first nineteen helper implementations and included the
  repository's actual scalar adapter and trace modules in a temporary harness.
  Independent signed integer arithmetic checked every 16-mask two-product and
  64-mask three-product factor-zero combination across the applicable pruned,
  dense, known-rational and dyadic variants. Pruned-event counts were exactly
  zero or one as prescribed; no existing count was weakened.
- Direct private probes checked retained unknown factors, row-major scale values
  and nine/sixteen scaling events. Three independently generated rational
  adjugate/inverse fixtures exercised ordinary, dense, dyadic and aggregate
  helper paths, including singular failure. These source extracts preserve
  the real helper bodies and scalar adapters; no mock arithmetic was substituted.
- All-feature before/after public and private stdout matched exactly. No-default
  mode repeated semantic checks with trace assertions compiled out. Final
  probes were warning-free. Original tests had 60-second process limits; probes
  had 30-second limits. This finite corpus is not a universal symbolic proof.
- Private library and integration-target Rustdoc passed with warnings and
  broken/private links denied in both configurations. Full-text checks matched
  all 22 doc groups on their individual private function pages. The corrected
  #[test] doc was source-reviewed and its original assertion executed; Rustdoc
  omits test functions. The 298 HTML pages inspected do not imply 298 new audits.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial including Hyperlimit, with all features and no defaults. Used
  --locked --offline -j 2, CARGO_INCREMENTAL=0 and one shared Cargo target.
- Full workflows remain owner-waived. No full matrix suite, timing benchmark,
  sanitizer, allocator profile, binary-size or total-work claim was added.
  No focused check remains blocked. Removed three temporary executables;
  the shared target remains near 2.5 GiB and no duplicate target tree was created.
- The root AGENTS.md 100% public/private Rustdoc mandate remains active.
  Repository-wide completion remains unproven and the wider audit is open.

Evidence: `/tmp/csgrs-rustdoc-matrix-cofactor3-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-cofactor3-audit/` temporary generators, source extracts,
probes, checkers and private docs. The next contiguous boundary is
`matrix4_factors`, currently core.rs line 9514. The 4×4 cofactor/inverse kernels,
public matrix APIs and remaining tests still require audit.

## 2026-11-27: 4×4 minors, adjugates and inverse dispatch

Scope and acceptance witness:

- Continued from `8bf6b5126`, reviewing every line of core.rs's original
  9513–10116 block: fifteen private functions, from `matrix4_factors` through
  `matrix4_adjugate_from_factors_dense_exact_known_rational`. The original
  604-line block lacked Rustdoc for all fifteen declarations. Added fifteen
  groups; the resulting block is 713 lines excluding its trailing separator.
- Read the representation classifiers, three-witness guard, scalar reducers,
  scalar aggregate inverse entry points, affine signal routing and right-division
  callers needed to explain the contract. Those contextual reads do not claim
  new whole-file coverage. Reviewed and documented two existing matrix tests:
  the general integer inverse/determinant and dense fractional determinant.
- Owner: Hyperlattice fixed-size matrix arithmetic over Hyperreal. Relevant
  properties are P1 exact values, P4 preserved fact/query order and P6 distinct
  singular/unknown errors. Inputs include rational, symbolic, sparse and singular
  matrices; unchecked known-rational helpers require current representation
  evidence. Adjugates remain defined for singular matrices; inversion does not.
- Missing acceptance witness: private rendered documentation for minor layout,
  transposed cofactor signs, arbitrary scaling, representation assumptions,
  aggregate fallback/error distinctions and branch-specific signal use.
  Reproduced on the target revision before editing repository sources.

Contracts clarified without implementation changes:

- The two minor arrays use row pairs (0,1)/(2,3), with column pairs
  (0,1), (0,2), (0,3), (1,2), (1,3), (2,3). Their determinant combination is
  `s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0`, the two-row Laplace expansion.
  Minor/source consistency is a caller invariant, not an attached certificate.
- Ordinary helpers prune definitely-zero products and retain unknown factors.
  Dense-exact helpers without a known-rational suffix use active scalar
  selection and admit symbolic values and zeros. Known-rational helpers require
  current rational storage; invalid evidence need not produce a typed mismatch.
- Determinant dispatch uses its unchecked rational path only after all sixteen
  entries expose nonzero rational values. This full-density test differs from
  the inverse's three nonzero witnesses, which merely exclude shape shortcuts.
- Adjugate entries are transposed cofactors. Scaled helpers immediately apply
  one caller-supplied factor to each of sixteen row-major outputs. They accept
  zero or other arbitrary factors; only a valid determinant reciprocal gives
  an inverse. Unscaled adjugates permit right division to delay the common scale.
- The 3×3 and 4×4 rational adapters classify current storage before selecting
  dyadic/general rational aggregates. Missing rational storage returns None;
  rational singularity returns Some(Err(DivideByZero)), a final outcome rather
  than permission to fall back. Neither adapter requires full entry density.
- The inverse dispatchers retain early aggregate priority, then identity,
  diagonal, upper triangular, lower triangular, affine and general cofactors.
  Upper routing excludes affine diagonal-linear cases. Ordinary determinant
  inversion has no explicit unknown-status rejection; checked cofactor paths
  guard before reciprocal construction and scaled-adjugate assembly.
- Unlike the 3×3 signal-aware dense route, the 4×4 route still tries the rational
  aggregate and returns without signal attachment. Symbolic dense and general
  cofactor paths attach after minor/determinant construction and before guarding.
  Shape shortcuts retain their existing branch-specific signal behavior.
- Removed historical timing-win/layout statements from the audited block;
  their old text remains in git history, not current performance evidence.
  Retained every formula, attribute, predicate order, trace and assertion.

Validation after final source edits:

- Focused Rustfmt and git diff --check passed. Removing only comment-only and
  blank lines leaves identical executable source in both edited Rust files
  against `8bf6b5126`.
- Hyperlattice library builds and ten selected original matrix tests passed
  with all features and no defaults. The all-feature baseline passed the same
  tests. Tests cover general/fractional determinants, negative powers,
  inverse/product composition, triangular/diagonal inverse agreement, singular
  and unknown rejection, and right division. Direct rustc harnesses used
  matching Cargo artifact graphs; original assertions were unchanged.
- Independent Python Fraction Gauss–Jordan inverse and permutation determinant
  oracles checked nine rational public fixtures: identity, diagonal, upper/lower
  triangular, diagonal-linear affine, general affine, general fallback, sparse
  dyadic satisfying the early witnesses, and fully dense non-dyadic rational.
  All three inverse modes matched, and results composed with their inputs to
  identity. All-feature probes asserted selected route counts, aggregate use,
  determinant specialization and signal-attachment counts.
- A symbolic dense-route fixture with determinant 25π matched literal inverse
  entries in all three modes. Fresh unknown determinants allowed ordinary
  construction but returned UnknownZero through checked modes. With an already
  set flag, the rational aggregate returned both nonsingular success and
  singular DivideByZero with exactly zero attachments. These observations do
  not establish complete evaluation of unknown expressions or a work bound.
- Temporary private probes extracted actual helper bodies and included the
  repository scalar-adapter and trace modules. Twenty-eight fixtures included
  structural shapes, rational denominator variation, zero/all-ones/rank-three
  singular matrices and sixteen deterministic generated matrices. Independent
  expectations checked twelve minors, determinant and every adjugate entry;
  cofactors used smaller permutation determinants rather than the implementation's
  six-minor formula. Both scaled variants matched factors zero and -2/3 and
  each made exactly sixteen scaling calls in trace mode. Rational inverses
  matched the independent oracle or the singular error as appropriate.
- Direct aggregate probes also covered sparse dyadic/non-dyadic 3×3 inputs,
  singular Some(error), and symbolic None for both sizes. Corrected a temporary
  harness import and removed an unused extracted helper; final probes compiled
  without warnings. All-feature before/after stdout matched. No-default mode
  repeated semantic checks with trace assertions compiled out. Original test
  processes had 60-second limits and probes 30-second limits; all completed.
- Library and integration-target private Rustdoc passed with warnings and
  broken/private links denied under both feature configurations. Full-text
  checks found every new function doc on its individual private page. The two
  #[test] comments were source-reviewed and their tests executed; Rustdoc omits
  test functions. The 298 HTML pages inspected do not represent new whole-file
  or repository-wide coverage.
- Cargo checks passed for the matrix integration target, mathbench and downstream
  Hyperspatial including Hyperlimit, with all features and no defaults. Commands
  used --locked --offline -j 2, CARGO_INCREMENTAL=0 and one shared target tree.
- Full workflows remain owner-waived. No full runtime suite, performance,
  allocator, sanitizer, code-size or total-work claim was added. No focused
  check remains blocked. Removed three temporary executables; the shared target
  remains near 2.5 GiB. The root AGENTS.md public/private 100% Rustdoc mandate
  remains active, and the repository-wide audit remains incomplete.

Evidence: `/tmp/csgrs-rustdoc-matrix-cofactor4-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-cofactor4-audit/` generators, source extracts, probes, checkers
and private docs. The next contiguous boundary is `macro_rules! impl_matrix`,
currently core.rs line 10227. Macro-generated public APIs, later matrix
implementations and remaining tests still require audit.

## 2026-11-27: common matrix API macro and generated operator contracts

Scope and acceptance witness:

- Continued from `20712b69a`, reading every line of the original core.rs
  10227–10793 block, including `impl_matrix!` and both invocations. The 567-line
  block becomes 833 lines including documentation. It contains one macro,
  35 generated impl blocks, 47 method templates and 32 associated-type
  templates. Of 115 resulting doc groups, 102 were missing, ten existing groups
  were expanded/corrected and three previously audited power groups retained.
- Added Rustdoc to three existing matrix tests: scalar broadcasting, matrix
  display and checked scalar division with an unset abort flag. Read surrounding
  helpers, both right-division dispatchers, scalar formatting/guards and existing
  borrowed-operator/diagnostic-display tests as consumer context. This does not
  claim new whole-file coverage of those files or the remaining matrix APIs.
- Owner: Hyperlattice's public fixed-size arithmetic wrappers over Hyperreal;
  properties P1 exact arithmetic, P4 preserved query/dispatch order and P6 typed
  divisor errors. Domain: Matrix3/Matrix4 and corresponding column vectors with
  rational/symbolic entries, scalar broadcasts, fallible matrix/scalar division,
  integer powers, array indexing and diagnostic formatting.
- Missing acceptance witness: authored docs for generated impls/methods/output
  types, plus accurate right-division, display, ownership, indexing and checked
  scalar contracts. Reproduced the missing groups and misleading inversion/display
  descriptions on the starting revision. Baseline library probes and original
  tests establish existing behavior; repository edits contain comments only.

Contracts clarified:

- Constructors retain row-major Real entries without validating invertibility
  or transform kind. Zero/identity use exact literals; transpose clones handles
  without arithmetic or conjugation and does not promise independent scalar caches.
- Scalar addition/subtraction broadcast to every entry, including off-diagonals.
  Matrix addition/subtraction are entrywise; multiplication is composition with
  the right transformation acting first on column vectors. All generated
  ownership forms describe the actual consumed/borrowed operands and schedules.
- Mutable row indexing does not transport previously collected facts across
  mutation. Both indexing methods panic at an out-of-range row. The row output
  type and all arithmetic result types now have their own documentation.
- Matrix division solves `X * right = left`; structural formulas, shared
  adjugates or a transposed solve need not materialize an inverse. Mixed-ownership
  ordinary division clones the borrowed matrix handles and forwards to the
  owned/owned operator, adding that operator's event; fully borrowed division
  invokes its dedicated dispatcher directly.
- Ordinary scalar division rejects definitely-zero divisors before one shared
  reciprocal. Unknown status is not itself rejected there. Checked division
  requires known nonzero status, returning DivideByZero/UnknownZero before
  scaling. Signal-aware scalar division attaches before that guard. A zero
  matrix does not authorize invalid scalar division. Borrowed scalar division
  retains the distinct Matrix3 borrowed-product and Matrix4 adapter schedules.
- Reciprocal aliases preserve matrix-inverse semantics and add their own dispatch
  event. The caret operator delegates to integer power. Existing power docs
  retain negative inversion, unsigned i32::MIN handling, singular exponent-zero
  identity and branch-specific cancellation behavior. Removed the macro's
  historical percentage-win comment; no current performance claim replaces it.
- Matrix-vector operators use the shared borrowed-input transform dispatcher.
  Matrix4 retains the supplied homogeneous weight; output is neither normalized
  by that weight nor converted to a point by this operator.
- Display writes nested row-major brackets. Ordinary entries use Real display;
  alternate entries use diagnostic_decimal(-256) with default precision and
  failure markers. Outer precision/width/alignment are not forwarded or applied
  to the complete matrix. Writer errors propagate. Display is not exact replay.

Validation after final source edits:

- Focused Rustfmt and git diff --check passed. Stripping only comment-only and
  blank lines leaves identical executable source in core.rs and tests/matrix.rs
  against `20712b69a`, preserving formulas, attributes, predicates and assertions.
- Hyperlattice library builds passed with all features and no defaults. Twelve
  original tests passed in each final configuration: nine selected matrix tests,
  the borrowed matrix operator test and both diagnostic-display tests. The
  all-feature baseline passed the first ten. Direct rustc harnesses used the
  matching Cargo artifact graph and retained every original assertion.
- Independent Fraction arithmetic and Gauss–Jordan inverse expectations checked
  all four matrix add/subtract/product/quotient ownership forms in both sizes,
  both scalar ownership forms for all four arithmetic operators, both negations
  and all four matrix-vector ownership forms. Literal expectations also checked
  constructors, transpose, mutation and reciprocal aliases. Trace-mode probes
  asserted each selected operator label, mixed-ownership division forwarding,
  scalar scaling counts and checked scalar attachment counts.
- Scalar tests distinguished exact-zero rejection and fresh unknown status in
  ordinary/checked/abort-aware paths. Checked failures stopped before scaling.
  Checked matrix division matched independent right-quotient expectations.
  Ordinary unknown reciprocal construction is not a completed-evaluation claim.
- For `A = I + N` with `N² = 0`, independent expectations `A^e = I + eN`
  checked ten exponents including i32::MIN, negative one, zero and positive
  square/cube/general paths, through ordinary/checked/signal-aware/caret APIs
  in both sizes. A set signal did not prevent these exact shortcuts. Singular
  zero matrices returned identity at exponent zero and DivideByZero at -1.
- Formatting probes checked literal rational/decimal strings, ignored outer
  precision/width, writer failure and an unresolved sin(e)²+cos(e)²-1 marker.
  Corrected an initial temporary-probe expectation: the shared unknown_zero
  fixture is a nonzero sine difference whose decimal refinement can succeed,
  so lack of cheap zero knowledge does not require an UnknownZero display
  marker. Preserved the initial failed log; no repository assertion was weakened.
- Both immutable and mutable out-of-bounds row probes caught the documented
  panic in each dimension. Their expected panic-hook stderr is retained;
  probe processes returned success. Final probes compiled without warnings,
  completed within 30 seconds, and all-feature before/after plus no-default
  stdout matched. Original test processes used 60-second limits.
- Library and integration-target private Rustdoc passed with warnings and
  broken/private links denied under both feature configurations. Full-text
  validation checked all 114 generated doc groups inside their own impl
  sections on each concrete matrix page, plus the private macro page: 229
  checks for 115 source templates. This verifies both expansions rather than
  only the macro source. The three #[test] docs were source-reviewed; Rustdoc
  omits test functions. Coverage of this block is not whole-crate coverage.
- Cargo checks passed for matrix, borrowed_ops, diagnostic_display, mathbench
  and downstream Hyperspatial including Hyperlimit with all features and no
  defaults. Used --locked --offline -j 2, CARGO_INCREMENTAL=0 and the shared
  target. Full workflows remain owner-waived; no performance, size, memory,
  sanitizer or universal symbolic-completion claim is made. No focused check
  remains blocked. Removed four owned temporary executables; the shared target
  remains near 2.5 GiB, without another Cargo target tree.
- The root AGENTS.md public/private 100% Rustdoc mandate remains active.
  The wider repository audit is incomplete; generated APIs must continue to be
  checked in their concrete rendered expansions as well as their source templates.

Evidence: `/tmp/csgrs-rustdoc-matrix-macro-*.log`, Cargo artifact JSON and
`/tmp/csgrs-matrix-macro-audit/` temporary generators, probes, checkers and private
docs. The next contiguous boundary is the dedicated `impl Matrix3` beginning
with `exact_facts`, currently core.rs line 11061. Those public APIs and the
remaining Matrix4 APIs/tests still require audit.

## Next work

Continue with owning-crate implementations and remaining build/support files.
Consult existing whole-file records before selecting the next batch, and maintain
review records with each batch. Establish automated public/private item coverage across manifests
and features without confusing that metric with semantic review. Review every
remaining tracked file before claiming completion. Commit and push documentation
batches after focused checks under the owner's session-specific waiver.
