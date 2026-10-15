# Optional bounds and visible modeling failures

Implementation: `6095f177e`. The authoritative qualification is
[`closure-v1.json`](closure-v1.json), produced by the retained
[`close_v1.py`](close_v1.py). It verifies 230 retained executable/artifact paths,
the final source identities and logs, the unchanged 35 pending native files,
and the ledger restriction to `current_work`. The overall plan remains open.

The owning layers are Hypermesh indexed bounds, the CSGRS modeling and native
curve bridges, TPMS extraction, scalar/language adapters, and HyperCircuit
thermal-clearance bounds. The declared domain includes valid indexed point
sets, open/closed meshes, canonical empty geometry, and native curve regions.
Bounds do not establish closed-solid topology or fidelity to an analytic source.

Empty bounds are absent; an origin point has nonempty zero-size bounds. Invalid
indices and undecided comparisons retain typed failures. Centering/floating
preserve empty composition. TPMS constructors propagate bounds, parameter,
capacity, sample, interpolation, and extraction failures. Resolution below two
is deliberately rejected instead of clamped; valid formulas and fixture
resolutions are preserved. The root README and binding documents describe the
API migration, including C `NO_BOUNDS=10` and optional language-level results.

Independent acceptance checks derive exact cuboid extrema and signed volume,
validate required topology, then compose the results. Other checks distinguish
empty geometry from a point, mutate warmed index buffers, replay
strict/approximate/strict queries, exercise empty TPMS inputs and invalid or
undecided parameters, retain native error payloads, and verify recovery.

| Trial | Result and scope |
| --- | --- |
| `baseline-v1` | Eight regressions fail on `542871237` plus the original in-scope native work. |
| `candidate-v1` | Compilation fails during API migration; retained without revision. |
| `candidate-v2` | 133 focused tests pass; strict Clippy rejects a missing WASM annotation. |
| `native-v1` | Commands exit successfully, but a minimal-feature unused-import warning remains. Predates the final downstream and WASM fixes; historical evidence only. |
| `native-v2` | 202 CSGRS tests, 38 minimal-feature tests, 235 Hypermesh tests, 10 FFI tests and one doctest pass. Formatting, strict all-feature/minimal/fuzz Clippy and strict docs pass. Minimal-feature tests overlap the all-feature suite. |
| `downstream-v1` | Ten materialization units, all three materialization integration tests, two package-model release tests, and strict all-target Clippy pass with `drc,interchange`. Other unit/release cases are filtered explicitly. |
| `bindings-v1` | All 29 commands pass: compiler/runtime versions, native library build, actual f64 C/C++/Python consumers, a compiled cgo test, generated Node/WASM behavior, and strict generated TypeScript checks. |

Each phase retains its command list, environment, source hashes, tracked patch,
untracked-source archive, raw outputs and terminal result. Build targets are
separate copies; earlier observed executables are preserved. `validate_v2.py`
adds Go sources to the guard used by `validate_v1.py`; Go was unchanged in the
baseline. The isolated Go toolchain's official archive checksum and release
metadata are in `go-toolchain.json`. Documentation and the WASM runner are
additionally hashed in `documentation-and-runner-inputs.json`.

Eight existing tests remain ignored: one CSGRS rendering case and seven
Hypermesh external-fixture/manual scale or memory cases. This qualification
does not run the full CI matrix, sanitizers, clean external package/MSRV tests,
or a performance comparison. The consumer-supplied native JavaScript loader
remains unqualified; the Go test uses the local cgo wrapper and actual library.
The thermal-spoke placement formulas retain their earlier behavior; independent
coverage of their full geometric fidelity remains follow-up work.

Commands have an external 1800-second execution limit. Checked primary sample
reservations do not bound secondary allocations, scalar expression growth,
total work or peak heap use; no internal cancellation contract is claimed.
Legacy SDF and other convenience fallbacks, end-to-end provenance, ordinary
strict workflow completion, release validation and prior scalar/native
admission requirements remain open. No latency or allocation improvement is
claimed. Use fresh named trials for further changes; do not overwrite this
record or rerun the closure into its existing output.
