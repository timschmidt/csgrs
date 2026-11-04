# Native source through display and export, 4 November 2026

SynapsCAD `1f1ec3e66d8e3a4c87b52146bb7369b26fe7b040` extends baseline
`124c31d`. CSGRS is `25fafc4bc9`, whose Rust remains identical to `d9b06483a3`.
This follows the [export equality admission](export-equality-2026-11-04.md).

## Contract and witnesses

The application display/export boundary owns this change (P6, P25–P30, P35).
Its domain is currently renderable native triangle models, including exact
vertices that share a finite display position after projection, qualified
construction history and open triangulated surfaces. It preserves the native
source through the compiler, application part cache and export. It does not
claim general projected-topology preservation or exact native file replay.

The unchanged small-angle cylinder difference reproduces four triangles whose
distinct vertices coincide in binary32 display coordinates. The former renderer
rejects triangle 25; Hypermesh's native GPU API constructs finite normals from
the exact source. The adapter now reuses that implementation and retains typed
GPU causes. Native buffers remain attached to each part in authored Z-up
coordinates; Y-up display buffers establish no source topology/fidelity claim.

A second baseline witness edits display positions and changes OBJ output. Final
STL, OBJ and 3MF exports instead consume the retained source; all three have
display-edit independence tests. Native STL owns binary32 projection and rejects
triangle collapse before file replacement. Native OBJ owns vertex/face records;
the app adds only material/group records. 3MF uses binary64 projection and native
triangle predicates. Exact source validation precedes format conversion: removing
it reproduced false success for the exactly collinear points `(0,0)`,
`(1/2,1/5)`, `(3/2,3/5)`, which appear noncollinear after rounding. Orphan native
positions fail explicitly. Tests verify existing files survive numerical/input
failures; no general filesystem transaction guarantee is claimed.

The full thin-box fixtures are retained. Multi-part OBJ replay checks authored
axes, native indices/winding, material ownership and exact volumes 25 then 24
after feeding the imported model into another Boolean difference. Shared source
storage and unchanged construction evidence are exercised independently from
display collapse. The removed renderer-weld helper's signed-zero control is
replaced by native-source independence under display-zero edits; source vertex
identity now comes directly from the native mesh.

This is an explicit API/artifact migration, not a performance-only refactor.
External `MeshData` literals need `source`. Exports use authored coordinates and
native winding; renderer welding and fallback normals are removed. OBJ requests
flat shading without derived normal records. Native GPU projection can change
float display rows. Plain files remain lossy and omit native source receipts;
cross-revision byte identity and original-source fidelity are not claimed.

## Executed qualification

Final source passed 43 binary/UI tests, four conversion tests, warning-denied
all-target `dispatch-trace` Clippy, locked WASM binary and fuzz-bin compilation,
warning-denied library rustdoc, native release build, formatting and whitespace
checks. The unchanged fifteen-case rotation witness passes. The unchanged bench
driver completes `default:all`, `$fn=16`, in one 479.84 ms smoke run.

The unchanged configured `.github/scripts/test-ci.sh` was executed under the
same external 1,200-second containment limit as the preceding admission. It
terminated with exit 101: **64 passed, one failed, two ignored, 53 filtered**,
935.18 seconds. B-text extrusion exceeds its existing deadline; the former
near-coincident failure passes. No hard input, resolution, assertion or workflow
exclusion changed. This library run predates final test-only/documentation edits;
current conversion witnesses were rerun. Later export edits have their complete
binary-suite check. A separately labelled diagnostic run excluding B passed
64 tests, with two ignored and 54 filtered, in 32.19 seconds. It does not replace
the failing CI command. Full pre-push qualification remains unavailable.

Fresh production previews of the ordinary through-hole and disconnected cube
match installed OpenSCAD 2021.01 visually. Tiny-angle references differ, including
a viewing-direction control; OpenSCAD warns its result is non-manifold at both
angles. Images and warnings are retained. General visual conformance for these
cases remains unqualified. A live CI snapshot reaches scalar/orientation work
but truncates before the owning operation; the later debugger attempt finds the
process already gone. It establishes no operation-level causal attribution.

## Cost and open work

A focused warm probe compares a source-carrying shim of the old normal arithmetic
with the native GPU route on identical exact inputs. It retains 20 alternating
pairs per fixture from one shared-host process. Median conversion costs:

| Fixture | Float-normal shim | Native GPU |
| --- | ---: | ---: |
| Cube, 12 triangles | 5.55 µs | 22.20 µs |
| Sphere, 528 triangles | 11.23 ms | 83.20 ms |
| Drilled body, 32 triangles | 9.54 µs | 50.59 µs |
| Rotated cylinder, 32 triangles | 248.39 µs | 1.34 ms |

These costs are adverse. Source geometry, indices and inherited evidence agree;
display outputs intentionally differ. The initial strict position-equality
probe fails and is retained: 176 sphere coordinate components change, maximum
observed difference `4.7683716e-7`; normals also differ. This is no identical-
artifact performance admission. Three candidate default compilations complete
in 288–320 ms with 20 parts, 2,632 native triangles, five views and no failures
or warnings; no paired end-to-end speedup is inferred.

Part layout grows from 104 to 152 bytes, including the 48-byte native carrier.
Geometry storage is shared, but its lifetime extends and retained memory grows
beyond that value-size change. Total heap/allocation cost is unmeasured. Native
normal construction/reuse, B-text completion/cancellation, models that cannot
produce any finite display buffer, projected topology, native replay, installed
distribution and the remaining research queue stay open. No push was made.

Evidence branch `native-source-boundaries-2026-11-04`, commit
`ca3f86e84fa0531d0493597f89f7e8b9672f4ce2`, adds only record
`docs/evidence/2026-11-04-native-source-boundaries`: 75 files, 273,076 bytes,
SHA-256 `0617a293285b952868bf5037a3fa85d7ed2159df176152f5f4fb55ac189e40a2`.
It retains source stages, patches, failed and passing witnesses, full/diagnostic
checks, visuals, cost samples, source revisions and commands. Earlier evidence
is unchanged. This CSGRS update is receipt-only: its affected checks are format,
links, whitespace, archive integrity and retention budget.
