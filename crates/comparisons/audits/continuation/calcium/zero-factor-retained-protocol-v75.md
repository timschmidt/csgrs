# Exact zero-factor live integration — checkpoint 75

Before editing live Hyper, freshly verify checkpoint 74, current 956-file live
identity, the 176-file candidate and 355-file consumer. Freeze those source maps
and the existing successful/failed evidence, including checkpoints 70–74. The
main candidate file and seven-test module must be promoted byte-for-byte, with
no opportunistic refactor, dependency change or unrelated solver edit.

Expected live changes: hypersolve/src/algebraic_binary.rs and the new
hypersolve/src/algebraic_binary/zero_factor_tests.rs. All other 955 previously
recorded files remain unchanged; the new live map contains 957 files. Preserve
pre-existing resultant.rs, root_isolation.rs and root_isolation_monic_tests.rs
changes. No commit, cleanup or donor edit. Use apply_patch for the live edits.

Run offline locked default tests and all-feature debug/release tests with
no-fail-fast and two test threads. Require every name/outcome to match the
qualified candidate: 817 default, 818 each all-feature profile, eight suites
per configuration, no failures/ignored tests. The default build intentionally
omits one existing feature-gated tensor test; do not call it a missing regression.
Run all-target/all-feature Clippy with warnings denied, formatting, and an
all-feature release WASM library build. Compare complete candidate/live
all-feature Cargo metadata after only intended solver/scalar path normalization;
check unique Hyper crates, lockfile and recorded tool versions.

The new source verifier must be independent of historical pre-adoption live
checks. Bind both changed files to the qualified candidate and every other live
file to the prior retained map. Recheck the frozen solver/consumer and archived
evidence without pretending that old runs used the new live path. Prior native/
WASM costs, consumer runs and representative sizes are inherited through exact
source/configuration identity, not relabelled as new measurements.

Record a seventh retained continuation transfer only after successful live
gates and a final sealed verification. No new dedicated binaries or source copy
are needed; reuse the nonincremental two-job build cache. Global donor coverage,
separate power-sum work and the complete remaining reference inventory stay open.
