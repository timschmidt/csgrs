# Native mesh failures through real WASM consumers

Native mesh Boolean, membership, hull and subdivision failures now throw ordinary `GeometryError` objects with the existing validation category in `code` and the native diagnostic in `message`. Zero subdivision level uses that same contract. The adapter reuses `ValidationError::kind`; STL continues to use `GeometryIoError`.

**Migration:** these paths previously threw strings. Callers should inspect `error.code` or `error.message`. Error objects belong to JavaScript, need no `free()`, and remain usable after native geometry handles are released. The generated class documentation explains this change.

The [declared contract](prototype-v1/contract.json) and [initial real-JS probe](preflight-v1/result.json) establish the owning layer and missing acceptance before implementation. The probe observed 28 string exceptions. The unchanged new acceptance suite [failed seven of nine tests on the baseline](baseline-v1/real-js.stdout); its TypeScript consumer compiled and failed at runtime on the same issue.

The qualified candidate [passed all nine new JS tests](wasm-validation-v1/real-consumers.stdout), both earlier JS consumers, all twelve earlier IO tests, and the new and previous compiled TypeScript consumers. Invalid operands beside empty fail visibly in either order and both policies; successful empty results retain certainty through Boolean composition, STL import and further composition. Diagnostics survive native-handle cleanup. Valid membership retains its own status and certainty.

WASM Clippy, native all-feature/all-target Clippy, all-feature docs with warnings denied, and formatting passed. Main-tree formatting passed after application. These native checks establish compilation/lint/documentation; the runtime boundary evidence comes from actual generated bindings.

The [ten retained process samples](cost-v1/result.json) completed 50 geometry pipelines and 250 diagnostic observations. Full success values and STL bytes agree; original diagnostic messages agree. Successful geometry and the intentionally changed error surface have separate timing observations, and whole-process CPU/RSS records are retained. The old and new diagnostic contracts perform different work. No speedup or performance-only optimization is claimed.

Processed WASM sizes were 11,198,949 bytes for the baseline and 11,195,752 bytes for the candidate. Source, dependency/tool versions, generated artifacts, raw output and all unfavorable samples are retained.

The source snapshot was prepared while the native disposition was running, from its qualified committed-code snapshot. Application waited for the disposition commit. All runtime, policy, fixture and dependency bytes match; the only source-manifest difference is the admission ledger’s `current_work` value. The [application record](application-v1.json) verifies that protected raw bytes and all other decoded ledger keys are identical.

This is a narrow boundary improvement. It adds no internal resource bound, native geometry capability or certification of earlier approximate intent. New tests cover the stated invalid-input cases, not new public triggers for every native error variant or all six categories. Remaining provenance, query, distribution, release, pre-push and other properties work stays open. The historical artifact audit remains failed/incomplete; inherited expectations and missing paths remain preserved.

The 35-file native integration remains rejected and preserved by the preceding disposition; no native edit is reintroduced. See the [decision](decision-v1.json) and [checkpoint](checkpoint-v1.json).
