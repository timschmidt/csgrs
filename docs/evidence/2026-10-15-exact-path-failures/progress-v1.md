# Exact path construction preserves failure

`bezier_path`, `bspline_path` and `bezier_region` now return typed results.
Bezier path construction has a shared policy-taking implementation; region
construction consumes its local certainty and retains native closure/topology
errors. B-spline construction preserves the existing native invalid-degree
cause. Failed construction no longer becomes an absent path or apparently
successful empty region. The README, render example and benchmark callers
describe and handle the public Rust migration.

The target is revision `958d2fa76` plus the existing 35 native worktree files,
whose hashes remain unchanged. `baseline-findings-v1.md` distinguishes actual
behavioral reproductions from the retained diagnostic compiler failure. Valid
inputs, display counts and original benchmark/render fixture geometry remain.

All ten `candidate-v1` commands pass on frozen final source: six new acceptance
tests, the full curve-enabled suite (245 passed), the full all-feature suite
(352 passed, one existing ignored test), all-feature/all-target Clippy with
warnings denied, workspace formatting, documentation generation, the root
doctest, both affected benchmark builds and the README render example build.
The example was built, not executed as a complete rendering campaign.

The semantic regressions independently check exact Bernstein midpoint values
for line, quadratic, cubic and degree-four Bezier paths and matching clamped
single-span B-splines. Counts 1, 16 and `usize::MAX` preserve the same exact
values; both policies certify these known inputs. Reversal consumes the native
path correctly. A closed cubic retains area `27/20`, inside/boundary/outside
membership, union with empty and affine-scaled area `81/10`. The retraced
quadratic remains a successful Certified empty set and feeds further transforms
and Booleans. Invalid input and unresolved closure remain errors, with native
causes; the unresolved fixture stays Undecided through strict/approximate/strict.

`workloads-v1` executes the actual changed benchmark callers on five unchanged
fixtures: Bezier construction, B-spline construction, cubic finite projection,
cubic region extrusion and the feature profile catalog. Each runs once and
completes. Their raw rows and dispatch records are retained. Counts/checksums
are caller observations, not an independent geometric oracle; the mixed catalog
still includes legacy constructors with separate open contracts. This is not a
matched timing study, whole-catalog certification or performance claim.

The final target uses a private copy-on-write view of the preserved E13 build.
Its setup hashes every lower artifact and verifies copy up; closure checks those
hashes again and retains the mount specification, view and upper artifacts.
The `/tmp` user quota remains full, so private writes are under `/run/user/1000`
and the mounted target is under the repository's ignored build directory.
Earlier artifacts and partial failed copies remain. The 1,800-second command
and 8-GiB per-process address-space limits are external containment, not new
internal query, retained-memory or cancellation bounds.

The evidence audit inherits all prior expected hashes, including 27,093 missing
paths from earlier external cleanup. It must establish no new loss or mismatch;
it cannot establish historical recovery. The status ledger changes only
`current_work`, with the exact outside bytes and other keys guarded against
`9025de62`. The benchmark harness and example font are unchanged target-revision
inputs pinned at closure; the shared runner did not watch those two paths at
every command. Their retained Git content supports reconstruction.

This is workspace qualification of the declared exact-path contract. No
affected C-family/WASM constructor exists for these APIs. No release, clean
external-package, full-CI, rendering campaign or full-plan admission is claimed.
Hilbert clipping and its lossy wrapper, legacy sampled gears/airfoil/text,
remaining adapters, wider provenance/resource guarantees and the full
properties plan remain open. The 35 pending native files remain separately
unadmitted.
