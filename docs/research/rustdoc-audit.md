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

## Next work

Continue with remaining test/example/build sources and owning crates. Maintain review records with
each batch. Establish automated public/private item coverage across manifests
and features without confusing that metric with semantic review. Review every
remaining tracked file before claiming completion. Commit and push documentation
batches after focused checks under the owner's session-specific waiver.
