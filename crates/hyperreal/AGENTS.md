# Repository Agent Rules

## Pre-Push Verification

Before every `git push`, agents must:

1. Run `cargo fmt --all -- --check`.
2. Run Cargo Clippy with `-D warnings`, including every feature, target, and manifest combination exercised by this repository's GitHub Actions workflows.
3. Run every locally executable validation command from every file in `.github/workflows/`, including tests, checks, docs, benches, fuzz-target checks, release builds, and WASM/demo builds where configured.
4. Fix every failure or warning caused by the pending changes. Repeat the complete validation set until it passes.
5. Commit all intended fixes before pushing, then confirm the worktree contains no uncommitted changes from the task.

A push is not allowed while any required check is failing. Hosted deployment actions, secret-dependent publication, and packaging for operating systems unavailable on the current host may be excluded only when they cannot be reproduced locally; inspect those workflow steps for consistency and report the limitation explicitly.

