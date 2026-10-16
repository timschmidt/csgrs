# Retained diagnostics

All executed phases are immutable and appear in the final summary, including
nonzero commands. No failed phase is resumed, overwritten, or counted as a
qualified workload.

- `reproduction-v1`: the baseline helper used a nonexistent free translation
  function, so compilation and subsequent invocations failed. The v2 helper
  uses the documented extension method.
- `reproduction-v2`: ordinary reproduction and the five exact full-size
  artifacts succeeded. The isolated enormous-count linear/arc children failed
  under containment without returning typed exhaustion. This is the expected
  reproduced defect, not a successful resource test; details and limits are in
  `baseline-findings-v1.md`.
- `candidate-v1`: native concatenation tests and formatting passed. The new
  full-workload test moved a scalar into `cos` and then tried to use it for
  `sin`, so root consumers and Clippy did not compile. The candidate-v2 test
  clones that scalar. A misplaced README table row was also corrected.
- `candidate-v2`: native concatenation tests, all-target root Clippy and the
  existing context/cache/query/slice consumers passed. Seven of eight new
  public tests passed. The expanded exact arc-volume assertion declined.
- `volume-witness-v1`: both unchanged baseline and candidate libraries decline
  the same expanded volume query on the original arc-12 fixture. Both pass
  linear/grid volume and subsequent composition.
- `volume-witness-v2`: an incorrect vector accessor prevents the new helper
  from compiling. Its failed invocations establish no geometric behavior.
- `volume-witness-v3`: local-edge divergence terms still decline when accumulated
  before normalization, on both revisions.
- `volume-witness-v4`: every face term is proved rational before accumulation;
  both revisions pass exact total volume, strict topology, union from empty,
  and complete self-subtraction for all original 8/16/12-copy fixtures. The
  earlier expanded queries remain recorded as undecided. See
  `volume-oracle-v1.md` for the derivation and precise claim.

The final regression uses the last proof without changing any input, expected
volume, policy, face count, membership witness, topology check, or downstream
operation. This qualifies a new acceptance oracle; it does not extend general
scalar expression recognition. Final production source is frozen across the
qualification, cost-library build, and resource measurement phases.

During qualification, the combined documentation command completed successfully
in about 200 seconds. A five-second observer recorded predominantly `statx`
calls while Cargo scanned the preserved build tree. A subsequent stack observer
found that Cargo had already exited. Both observer commands and outputs are
retained beside the qualification logs. Neither command changed source or
terminated Cargo; no documentation failure or precise bottleneck proof is
claimed. These observations are separate from the later fixed cost trials.

The direct all-feature release library build also completes. Its dependency
configuration differs from the retained benchmark build, so cost-build-v2
restores the original benchmark configuration in another private target. Before
resource measurements, the preparation helper requires matching root/native
features, profiles, compiler flags and native dependency fingerprints, and
checks that each root library references the selected native library. Both
build phases remain retained; no cost comparison uses mismatched configurations.
