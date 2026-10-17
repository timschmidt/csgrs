# Preserve the demo while correcting validation workspace discovery

All three `ui-validation-v1` commands exit before compilation. Cargo excludes
the demo from the copied workspace, then discovers the outer live repository
workspace and reports that the nested snapshot is not its member. Preserve
those exact failures and the tail-v4 prerequisite stop. No test executed and
no cost measurement began in that attempt.

Reproduce workspace discovery with offline, locked Cargo metadata at the live
unchanged demo manifest. Copy all 2,835 final v6 inputs byte-for-byte outside
the repository into a fresh `/tmp` directory, then verify that Cargo resolves
that demo as its own workspace. Do not edit manifests, locks, policies,
fixtures, tests or implementation to accommodate the harness location.

Use a new ext4 build target and the same three release validation commands,
limits and dependency cache. Verify the complete source manifest before and
after each command. The external source copy is a retained physical artifact
and must be included independently in the final physical-path audit. Keep the
original nested snapshot and completed failed target unchanged.

Only after these checks pass may the unchanged matched cost protocol run.
The source-location correction does not establish a shipped-package or browser
interaction contract, and does not relabel any earlier command as successful.
