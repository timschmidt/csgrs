# Metadata trial: qualification withheld after storage failure

The metadata-order candidate and the 35 pending native files remain unadmitted.
The trial's command outputs, cost samples and report bytes are retained, but
compiler-input identity is not qualified: the final artifact audit failed and a
fresh reproduction exposed lookup-order aliasing in nested FUSE overlays.

This report supersedes the admission interpretation of [progress-v1.md](progress-v1.md),
[decision-v1.json](decision-v1.json), and the earlier assessment summaries. Those
files remain unchanged as records made before the integrity failure. In
particular, the final checkpoint mentioned in progress-v1 was never produced.
[checkpoint-v2.json](checkpoint-v2.json) is a diagnostic checkpoint, not a passing
artifact audit or implementation admission.

## Reproduced integrity failure

The owning layer is validation infrastructure; P19, P27, P36 and P37 require
identity, reproducible evidence and claims limited to what was established.
The input domain is Cargo artifacts served through nested fuse-overlayfs mounts.
The missing acceptance check was preservation of distinct file identities under
both lookup orders, including hard-link replacement inherited from parent layers.

The attempted full audit exited 1 on the retained
`polyhedral-construction/qualification-v1/release/libcsgrs.rlib` path. Its expected
SHA-256 is `8131217ef1e1b510d615d0963966bfe849f3fc99f3f53fdfd096a4a7f5756e53`;
uncached reads returned `0b334878584bb5f993bf74ccdf85135eb2133f4726bac4c6b5e18a41f3255c74`.
The unexpected bytes were retained separately. Direct reads, including O_DIRECT,
reproduced the mismatch; it was not solely an audit memoization error.

Fresh read-only overlays over the same historical parent reproduced namespace
conflation. Reading the root library first made both library paths return that
file's bytes. Reading the deps library first made both paths return the deps
file's bytes. The physical source files stayed unchanged. The eight observations
(two paths, two orders, two repeats) are in
[the nested reproduction](closure-diagnostic-v1/overlay-probe-v1/observations.json).

The equivalent eight reads passed using independent physical copies as a plain
lower directory: [flat-copy control](closure-diagnostic-v1/flat-probe-v1/result.json).
This is a bounded control, not qualification of an entire build filesystem.
No existing target, mount, source or historical expectation was reset or repaired.

The [diagnostic](closure-diagnostic-v1/digests-v1.json) retains expected and observed
hashes separately. The original audit stopped before completion. Its printed
progress count is not a count of independently verified current artifacts.
Inherited missing paths retain their original expectations; there is no new
whole-history completeness, zero-loss or zero-mismatch claim.

## Observations retained, without admission

All 191 distinct executable references cited by the trial have matching bytes
retained in physical files or independent copies; see
[reference preservation](closure-diagnostic-v1/references-v1.json.gz).
This preserves the observed executables. It does not establish which dependency
bytes a compiler consumed through a nested view.

The baseline recorded 6/6 passing commands; the candidate recorded 14/16.
The two original failures remain: full-board worker CPU was 176.419625977 seconds
against 120, and copper-neck check-body wall time was 86.132483507 seconds against
60. No assertion, fixture, precision or policy was weakened. The complete raw
commands and the original assessment remain available in this directory.

The local cost trial completed 20 processes and 360 rows. Warm duplicate rejection
was 6.9% slower in all five pairs; all favorable and unfavorable rows remain in
[the original cost assessment](cost-assessment-v1.json). Full board report bytes
were identical across the variants, as recorded by
[the report replay](report-replay-v1/result.json). These are retained observations,
not qualified application speedups or independent geometry proofs.

## Checks and next work

The diagnostic checkpoint verifies the main source guards, all 35 pending native
files, local evidence bytes, retained executable references, and newly inventoried
physical trial artifacts. Its schema explicitly marks the inherited view audit
failed/incomplete. Parent expectations are referenced without changing their
hashes. Only the `current_work` value in the status ledger changes; the remaining
raw bytes and decoded keys stay bound to revision `9025de62`.

Continue with a new physical build target, exercise file creation, hard links,
atomic replacement and executable loading, then rebuild and rerun the affected
checks without nested overlay inputs. Earlier validation that relied on these
views requires requalification before supporting a current acceptance claim.
The discovered identity defect does not by itself prove production code wrong.

The format-boundary cases in [the inspection note](followup-inspection-v1.md) are
still hypotheses and must be reproduced before a runtime fix. Release, external
consumer, complete workflow, formal verification, coverage and resource-contract
work remain open. No runtime source change or push is part of this checkpoint.
The full properties plan remains active and incomplete.
