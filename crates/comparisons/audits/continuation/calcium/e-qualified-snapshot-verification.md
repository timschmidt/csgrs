# Checkpoint 43: historical and current source identity

The original 43-checkpoint draft verifier passed before the e planner was
retained. The retention then legitimately changed `constants.rs`, added four
test-registration lines to `approximation.rs`, and added a test module.
Several older verifiers unconditionally read historical Hyper ranges from the
current workspace; the two complex-product source helpers also checked the
entire former live map. Their original paths are no longer appropriate for
checking historical evidence after this production change.

No original verifier, result, manifest or frozen source is replaced. The
versioned modules generated from `snapshot-v43-spec.mjs` change only explicit
historical source paths to the existing `derivative-demand-candidate` snapshot
and imports needed to reach those modules. The remaining source text, including
every numerical assertion, benchmark calculation, captured command path, failure
expectation and hash check of original artifacts, is identical. No filesystem
interposition, assertion suppression or skipped historical stage is used.

`e-qualified-retention-verification.json` records each original and derived hash,
every exact substitution, the 955-file historical map and the 956-file retained
map. The new entry point re-derives and byte-checks all versioned modules before
running the complete chain. Current live hashes are checked before and after;
the qualified 43rd verifier also checks them directly. Historical `--*-live`
flags are deliberately not accepted by this entry point.

Run from this directory:

```sh
node verify-e-qualified-retained.mjs --e-plan-live
```

This is an evidence/source-integrity rerun, not a new numerical campaign or a
44th source-audit checkpoint. It preserves the pre-retention successful draft
and all earlier failed numerical, memory and audit-harness evidence. The original
`verify-e-qualified.mjs` remains bound to the qualification manifest unchanged;
its direct historical-current-source assumption was identified by inspection,
not recorded as a fabricated failed command.
