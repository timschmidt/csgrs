# Self-intersection fuzz-driver compiler compatibility

The fixed-size coordinate iterator in
`crates/hypercurve/fuzz/fuzz_targets/curve_self_intersection.rs` now passes
strict Clippy on Rust 1.99 and 1.95. This changes a qualification driver,
owned by Hypercurve under P36/P37, while preserving its P17–P19 witnesses.
It does not change a production kernel or establish additional curve support.

On parent `a1adc106811c8b4f9bb892fabb80f73c870d85d2`, strict full-bin
Clippy reproduces `chunks_exact_to_as_chunks`. The admitted input remains
every byte slice: the existing guard returns below nine bytes; the first
eight bytes supply four ordered coordinate pairs; byte eight selects the
split. Fixed array chunks are viewed as slices before the existing coordinate
map. Every conversion, policy, geometry call, uncertainty branch and assertion
is unchanged. The slice view preserves the previous optimized code generation.

## Executed checks

- Full-bin strict Clippy passes on stable Rust 1.99 and Rust 1.95.
- Workspace formatting and diff checks pass.
- Same-path stable release builds produce identical final executables:
  17,834,640 bytes, SHA-256
  `9472982e01eaa670eea7c2c40dbdedda3a04c67d2339c4d91e10aa5aebb085dc`.
  All ELF sections match; `.text` remains 10,943,358 bytes. The baseline was
  rebuilt and reproduced exactly before the final comparison.
- Both executables replay seven fixed inputs: two rejected short inputs,
  constant, straight, looped, closed and retraced curves. Existing assertions
  pass. This is fixed-input replay, without sanitizer instrumentation or a
  mutation campaign.
- Derived diagnostic harnesses preserve the driver body and compare exact
  dispatch-count vectors in fresh processes under `STRICT` and
  `APPROXIMATE_512`. All fourteen comparisons pass. Each policy records 36,709
  events for the closed curve, 1,265 for the constant, 1,915 for the straight
  curve, 59,335 for the loop and 243,885 for retracing; short inputs record zero.
  The harness asserts the complete vector, not just these totals.

The production dependency sources and manifests are unchanged. Identical
optimized executable bytes establish unchanged executable cost for this native
driver configuration; no cross-platform speedup, sanitizer-cost or universal
closure claim follows. Kernel benchmarks and downstream binaries do not consume
this standalone fuzz source. No size exception or cumulative allowance is used.

The initial array-reference candidate passed Clippy but added 128 executable
bytes and 64 text bytes. Its measurements are retained; it is superseded by
the slice-view candidate. Initial evidence-tool mistakes are also retained:
an objcopy invocation normalized a diagnostic copy, raw copies initially lacked
executable mode, and the first trace wrapper omitted the recording scope. The
positive-event assertion caught the last error. Corrected raw extraction,
replay and count assertions supply the accepted evidence; no assertion was
weakened.

Raw evidence is staged under
`/tmp/interval-refactor/coordinated-ci/fuzz-chunks-*`, including command records,
source patches, executable and section hashes, both diagnostic harnesses and
their complete trace vectors. Coordinated stage 92 is still running and has
not been frozen or archived. The broader workflow qualification remains open,
including the independently reproduced Hyperdrc 120-second CPU-time failure.
