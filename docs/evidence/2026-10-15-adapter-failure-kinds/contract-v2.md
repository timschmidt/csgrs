# Correct native error bridge

The first standalone probes in `reproduction-v1` fail to compile: `SolidExt`
Booleans return `HypermeshError`, not `ValidationError`. Their attempted launches
also fail because compilation produced no executables. These are diagnostic
scaffolding failures, not a reproduction of the intended adapter assertion.
The first contract's native-type description is corrected here; its input
domain and acceptance criteria remain.

`ValidationError` already wraps `HypermeshError` in its `Mesh` variant and
classifies it with `kind()`. `AdapterError` already wraps `ValidationError` in
`Construction`, preserving display and source. The appropriate shared bridge
is a `From<HypermeshError> for AdapterError` implementation that uses those
existing conversions, then the eight Boolean methods can use `map_err` with
that bridge. No new error classifier, enum variant or geometric code is needed.

The v2 probes explicitly wrap the native error for category comparison and
require both source links: AdapterError → ValidationError → HypermeshError.
Run them against the unchanged target graph before production edits. A compile
success alone does not reproduce the loss, and a deliberately failing typed
assertion is not a passing implementation check.
