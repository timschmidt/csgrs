# hyperdrc Proptest Regressions

This folder stores persisted regression seeds for `hyperdrc` property tests.
The files are generated and consumed by `proptest` so previously minimized
parser failures remain covered.

## Contents

- [`excellon.txt`](excellon.txt) stores regression cases for Excellon parser
  property tests.
- [`ipc356.txt`](ipc356.txt) stores regression cases for IPC-D-356 parser
  property tests.

## Maintenance

Do not hand-edit these files unless you are intentionally pruning or
investigating a stale regression seed. When property tests discover a new
minimal failing input, `proptest` may update these files. Review those updates
like source changes because they represent permanent parser coverage.

Return to the [repository README](../README.md).
