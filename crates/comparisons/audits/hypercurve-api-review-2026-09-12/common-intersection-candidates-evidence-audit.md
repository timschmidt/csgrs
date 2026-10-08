# Common curve intersection candidate evidence

The full implementation goal remains active. This migration is a prerequisite for common analytic support-pair dispatch.

The rational/rational, parallel/rational and parallel/parallel candidate enums had identical mathematical payloads: no isolated intersection, two ordered lists of exact scalar/algebraic parameters, or degenerate elimination requiring further replay. Their distinct public names required two conversion functions and blocked common intersection diagnostics from accepting the existing analytic authority directly.

`CurveIntersectionCandidates2` now owns that common evidence in the curve intersection module. The three former names and both conversion functions are removed. Kernel signatures, incomplete supplements, common blockers, tests and benchmarks use the new type directly. Parallel/rational candidate fields use operand order (`first_parameters`, `second_parameters`). Swapping operands moves the two vectors without rebuilding or re-isolating roots. No compatibility alias remains.

This change does not alter projection, isolation, root identity, replay, normal selection, source-pole exclusions, saturation, overlap extraction, decision policy or retained contact payloads. The existing regression suite includes independent parameter-chart ordering, false normal-branch rejection, boundary fibers, constant images, residual contacts beside components and selected finite/exterior domain replay. All controlled production callers were searched; archived source snapshots in the comparison repository remain historical evidence.

The first compile attempt exposed missing imports from the scripted migration; these were corrected before qualification. The initial failure and source manifest are preserved under the `common-intersection-candidates-attempt0` prefix. Final sources stay frozen through the build and regression sequence.

This removes two enum definitions and conversion machinery. It makes no new runtime or memory claim. Common analytic support dispatch, publication of point-image parameter components, finite-domain clipping and public region overlap authority remain required follow-up work. The baseline `analytic-common-dispatch-baseline.json` records the still-missing dispatch independently of this migration.

Final counts, hashes, dependent checks and commit are recorded in `common-intersection-candidates-qualification.json`.

Committed as Hypercurve `d5e1b6afb8a95c9dfbf753529fcaa0f2431c12d8`. Qualification: 49 targets, 2,210 passes (1,978 Hypercurve + 232 HyperBREP), five unchanged known failures, nine ignored, eight unchanged expensive exclusions, zero new failures or timeouts. Both all-target checks, fuzz and UI checks, formatting, whitespace and all 396 frozen source hashes pass. All 30 repositories are clean.
