# Repository Agent Rules

## Change Acceptance

Apply the acceptance contract in [`properties.md`](properties.md) and the
normative protocol in [`docs/admission.md`](docs/admission.md) to every change.

Before changing code, identify the owning layer, the relevant property, its
input domain, and a failing or missing acceptance test. Reproduce review
findings on the target revision; do not assume an earlier assessment is still
current.

Make the smallest coherent change that restores or extends the contract.
Preserve exact meaning, supported composition, provenance, certainty, and
explicit failure. Do not weaken tests, policies, fixtures, or validation to
obtain a passing result.

After the final edit, run the affected local and downstream checks. Record
what was executed, what passed, what remains blocked, and the evidence
supporting any correctness or performance claim. Updating a specification or
ledger does not establish that the implementation satisfies it.

## Pre-Push Verification

Before every `git push`, agents must:

1. Run `cargo fmt --all -- --check`.
2. Run Cargo Clippy with `-D warnings`, including every feature, target, and manifest combination exercised by this repository's GitHub Actions workflows.
3. Run every locally executable validation command from every file in `.github/workflows/`, including tests, checks, docs, benches, fuzz-target checks, release builds, and WASM/demo builds where configured.
4. Whenever CI configuration changes (`.github/workflows/**`, `.github/actions/**`, `.github/dependabot.yml`, or pre-commit configuration), run `zizmor --pedantic --strict-collection .` from the repository root and fix every finding.
5. Fix every failure or warning caused by the pending changes. Repeat the complete validation set until it passes.
6. Commit all intended fixes before pushing, then confirm the worktree contains no uncommitted changes from the task.

A push is not allowed while any required check is failing. Hosted deployment actions, secret-dependent publication, and packaging for operating systems unavailable on the current host may be excluded only when they cannot be reproduced locally; inspect those workflow steps for consistency and report the limitation explicitly.
