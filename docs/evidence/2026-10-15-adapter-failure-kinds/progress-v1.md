# Typed mesh Boolean adapter failures

The scalar adapter now preserves native mesh Boolean errors instead of
converting them to strings. All eight `ScalarMesh` entry points share
`From<HypermeshError> for AdapterError`, which reuses
`ValidationError::Mesh` and `AdapterError::Construction`. Diagnostic text and
both `Error::source` links survive. No geometry algorithm, predicate policy,
certainty receipt, supported input, fixture or resolution changes.

The owning layer is the CSGRS scalar adapter; the affected consumers are its
four scalar families and the C ABI's C++, Python and Go wrappers. This restores
the typed-failure and empty/malformed distinction required by P6, P18, P25 and
P30. The public migration is documented in the root README, adapter rustdoc and
both binding READMEs: undecided mesh Booleans now expose the existing status 5
instead of generic status 4. Invalid input remains status 4. Failed operations
publish no model handle.

## Reproduction and acceptance

- `reproduction-v1` preserves a probe compilation mistake: the native Boolean
  error is `HypermeshError`, not `ValidationError`. It is not a behavioral
  reproduction. `contract-v2.md` corrects that type description.
- `reproduction-v2` reproduces the loss on revision `5080ba4b4`: 0/192 invalid
  inputs and 0/32 undecided cases retain typed causes. Invalid-index inputs are
  paired with empty and nonempty geometry in both orders, across all four
  operations, all four scalar families and default/strict/approximate contexts.
  The undecided cases cover default and strict entry points.
- `baseline-bindings-v1` builds the actual shared library and C/C++/Go consumers.
  All four operations lose their undecided status in each of C, C++, Python and
  Go. The original four Go tests pass. Diagnostic text and failure remain
  visible; the missing category is the reproduced defect.
- `candidate-v1` passes all 224 Rust cases, the full minimal/curve/all-feature
  modeling suites (104/239/346 passed; one existing ignored test with all
  features), ten FFI tests, FFI Clippy, documentation generation and the root
  doctest. The new C/C++/Python consumers each retain 4/4 undecided statuses;
  all five Go tests pass. All existing C/C++/Python consumers for empty
  composition, primitive failures, transforms, bounds, curve constructors,
  planar operations and exact linear modeling also pass.
- That phase remains failed because root Clippy rejects `.err().expect()` in
  the new native-error test helper. The final edit changes only that expression
  to `expect_err()`. Its input and assertions are unchanged.
- `candidate-v2` stops at a `/tmp` user quota during preparation, before any
  validation command. Its complete diagnostics and partial build copy remain.
- `candidate-v3` passes explicit copy-on-write storage setup, workspace format
  checking, the 224 cases under minimal/curve/all-feature configurations and
  all-feature/all-target root Clippy with `-D warnings`. Every production,
  other test, binding, documentation and dependency source matches candidate-v1;
  its unaffected successful checks are reused rather than represented as rerun.

The typed-error tests compare native error values to establish propagation,
not as an independent geometry oracle. Invalid indices independently establish
malformed input; rotation/translation identities define the uncertain fixtures.
The existing empty-composition consumers separately check regularized-set
identities, volume/bounds, malformed operands and downstream recovery. This
patch does not extend the kernel's certified geometry domain.

## Resources, preservation and limits

`storage-contract-v1.md` records the quota failure and private overlay. Before
mounting, every lower artifact is hashed; closure must verify those hashes
again. New artifacts reside in the private upper directory. Earlier targets
and the failed partial copy remain intact. The overlay view must remain mounted
while its paths are part of retained evidence. Go 1.27.2 was restored from the
previously pinned archive after the earlier tool directory was found absent;
its size and SHA-256 were checked before extraction.

Commands retain their source snapshots, exact invocation, raw output,
executable hashes and resource records. The 1,800-second command limit and
8-GiB per-process address-space limit are external containment, not an internal
full-query work, retained-memory or cancellation contract. Replacing a formatted
error string with an existing owned variant introduces no new cache or geometry
work. No latency, allocation or binary-size improvement is claimed; command
timings are not matched performance measurements.

The lower geometry/scalar sources and WASM implementation/consumer sources
match the closed E12 qualification. That earlier evidence is inherited, not
presented as rerun or as validation of a shipped package. The closure carries
forward all prior artifact expectations, including 27,093 previously missing
paths; it does not claim historical recovery. The 35 pending native worktree
files remain unchanged and separately unadmitted.

Only the `current_work` value in `docs/properties-status.json` is updated; exact
bytes outside it and all other decoded keys must still match revision
`9025de62`. Other text-only adapter paths, remaining silent-failure convenience
APIs, broader provenance/resource guarantees, clean external package and
release validation, formal/fuzz/coverage and the full properties plan remain
open. No push or full-CI admission is part of this change.
