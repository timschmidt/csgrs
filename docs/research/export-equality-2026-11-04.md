# Projected-vertex export equality, 4 November 2026

SynapsCAD commit `124c31df8aa55bfcaa93582a4210b39a79bcf0e3` corrects a
reproduced export defect on baseline `4fcb1d3`. CSGRS is `75b7cbd2c6`, whose
Rust source remains identical to `d9b06483a3`. This follows the
[downstream API admission](downstream-api-admission-2026-11-04.md).

The owning layer is the application's export boundary (P6, P27, P30, P35).
Its input is the existing finite binary32 renderer positions and valid triangle
indices. The contract is to remove duplicate positions without merging distinct
values. It does not establish fidelity to the original exact source, general
topology preservation of projection/welding, or exact real-valued text replay.

A missing production export witness reproduces the defect: an exact cuboid of
width `1 / 16,777,216`, length one and height one passes renderer conversion.
The export weld's rounding grid merges its eight corners into four. OBJ loses
one dimension; the generated 3MF fails to reload because triangles reference
repeated vertices. Both tests fail before the implementation change. This
defect is separate from the near-coincident f32 rendering failure.

The private shared export helper now keys finite positions by their binary32
values, normalizing signed zero, and retains the first occurrence. It removes
the `1e6` tolerance grid and its saturating integer conversion. OBJ and 3MF
share the correction; STL does not use this helper. No modeling, predicates,
renderer projection, normal generation, source policy or original hard fixture
changed. The helper's documentation no longer claims that welding establishes
manifoldness.

Both production write/read regressions now preserve the two planes, eight
corners, twelve triangles, closed manifoldness and exact nondegeneracy of the
recovered mesh. File coordinates are recovered as binary32: OBJ uses the test's
coordinate/index reader, and 3MF uses lib3mf's reader. These checks do not claim
exact native interchange. A signed-zero control confirms duplicate removal.

After the final Rust edit, all 35 binary/UI tests passed with release
`dispatch-trace`; offline locked all-target Clippy with warnings denied passed;
the WASM binary compiled; formatting and whitespace checks passed. Commands and
logs are retained below. No rendering code changed, so the previous bounded
visual check is not presented as fresh export qualification. The full library
CI run was not repeated for this private helper change: its two known failures,
kernel cancellation limits and lack of whole-suite green remain explicit in
the prior admission. No performance claim or push qualification is made.

Authoritative native-carrier export, the existing f32 projection, normal
fallbacks and source-evidence transport remain open. This local correction
prevents an additional export merge; it does not complete those contracts.

Evidence branch `export-equality-2026-11-04`, commit
`0c0f10d4e10c1366018689e4cfefe57292779d5a`, adds only
`docs/evidence/2026-11-04-export-equality`: eight files, 13,718 bytes, SHA-256
`eaf1fceaf66a0231d46df4ee91e5ee1f86977ae1a6c2830de9038f7b5ad64afb`.
The source patch, baseline failures, all-test success, Clippy/WASM/format logs,
source revisions and replay commands are retained. Earlier archive paths are
unchanged, and the branch is unpushed. This CSGRS update is receipt-only; affected
checks are formatting, links, whitespace, archive integrity and retention budget.

The subsequent [native-source admission](native-source-boundaries-2026-11-04.md)
at SynapsCAD `1f1ec3e` supersedes renderer-buffer export and resolves the
near-coincident failure. Configured CI now has 64 passes, one B-text failure and
two ignores; conversion costs and remaining boundary/visual limits are retained.
