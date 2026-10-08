# Explicit finite domains through selected-circle geometry

Committed Hypercurve `3eabc047e4e826385aa6ab7720cdc966dc06f1a1`.

Parent: Hypercurve `72d831c46241929d0621969c372262c78a4cd88f`. HyperBREP remains `878fcb441b281ab1b44051ca4420b28092c3c2ed`. This is a bounded implementation step within the active architecture goal.

## Change

Eleven private geometric signatures now require `CurveParameterRange2`. The three selected-circle/parallel entry points are replaced by one worker with an explicit finite range and optional incident extension. Public pair dispatch, Boolean construction, corner operations and retained tangent replay preserve the actual source range. They no longer substitute the unit interval when endpoints retain selected-fiber evidence. Controlled callers are migrated directly; no compatibility interface is retained.

Circle bounds reuse the existing finite support authority. Optional PH substitution certifies the consumed range and retains the caller's policy inside exact predicate scope. Native whole-unit shortcuts require coverage. The affine-line adapter reuses the full circle/chord authority and clips its result to the requested domain. Projection and overlap replay use the common finite/ray scheduler, retain finite ownership of shared roots, and retain original selected overlap endpoints. Inverse maps derive their search interval from that retained range. General selected-normal frames can use rational substitution; other frames preserve their compact direct analytic equations when nonlinear rationalization would enlarge the coefficient field.

Mapped complementary-cut replay now shares direction matching across rational/rational, analytic/analytic and mixed pairs. Reversal transforms both the diameter and its positive denominator before the existing unsquared relation is tested. It promotes the matching shared cut once and preserves the original policy. The selected-normal affine frame uses a finite ordered bracket around its selected center, rather than a unit-only interval constructor.

## Independent expectations and regressions

The public fixture and its independent transverse-contact proof are preserved in `finite-circle-analytic-domains-public.rs` and `finite-circle-analytic-domains-certificate.md`. The committed parent executable completed 16 native cases and blocked 16 exterior cases with Unsupported; `finite-circle-analytic-domains-projection-check.json` binds that result to parent library and source hashes. The new integration regression exercises the same 32 combinations of policies, charts, operand order and reversals. Expected uniqueness and transversality come from the independent sign inequalities; public location-to-point replay verifies retained result evidence.

The new selected-overlap regression covers 12 finite solves and 48 endpoint map replays, with selected endpoints `sqrt(2)/4` and `1/2+sqrt(2)/4`, both policies and orientations, an already-consumed approximation terminal, and zero/nonzero PH displacement. The new mixed-map regression reflects a rational quarter-circle across the vertical diameter of an analytic quarter, checking complementary cuts in both operand orders and source orientations, and rejecting self-complementarity at the noncentral cut. An existing compact affine selected-normal regression now includes negative and exterior selected centers, both finite orientations, both halves and both policies.

## Failed candidates and computational behavior

Two compile-only attempts are retained under `finite-circle-domains-check-attempt1/` and `finite-circle-domains-check-attempt2/`. The first runtime candidate is archived under `finite-circle-domains-initial-attempt1/`: its public matrix passed, but broad validation found a unit-only selected-frame bracket, reversed mixed-map replay failure and an unexpected change of nonlinear map representation. The broad library run also exceeded 300 seconds. It is not a qualified run.

The second runtime candidate is archived under `finite-circle-domains-replay-attempt2/`. It repaired bracket and mixed-map replay, and 12 of 13 focused tests passed. The existing direct-Bezier fillet/classification test still exceeded 120 seconds. Bounding optional sign work alone did not solve that stall. A debugger attempt could not start because ptrace was unavailable in the sandbox; no stack trace was obtained.

The final candidate restores compact analytic dispatch for the affected nonlinear rational image. All 14 focused tests pass, including the previously stalled test in approximately 0.016 seconds, two long mapped-cut replay tests and the 32-case public integration regression. This is a local regression repair under uncontrolled other-session system load, not a general throughput or memory claim. Final qualification has 2,299 passes across 49 targets (2,067 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, eight existing expensive exclusions and no new failures or broad-run timeouts. All 35 public probes and 14 focused tests pass. The no-default-feature caller checks, all-feature release builds, formatting and source/artifact hashes verify; see `finite-circle-domains-qualification.json`. The successful qualification writer must not be rerun on changed sources. The archived normal library has SHA-256 `8eb7cd1fbaa241bf01e599e07ca251959c800c77c62a8091232bad9d52006638`, and the libtest SHA-256 is `16887002f46969544cb2afaef9362762df6d645e98651f5bc9cd08b46f042de5`, under `finite-circle-domains-libraries/`.

## Remaining scope and ownership

Some lower selected-circle equation builders still require native whole-unit source finiteness or regularity. Rational curves with unused native poles therefore remain separate closure work; the public exterior polynomial successes do not prove that case. Selected-point finite incidence, one-sided source-cusp frames, normalized public-region construction, independent inverse replay, five known failures/eight expensive exclusions and the broader algebraic/computational consolidation remain unfinished.

Builds use seven pinned committed repositories. The other session's Hyperreal verification and associated changes are untouched. The qualification checks the 396 Hypercurve/HyperBREP working files, 1,005 isolated source files, toolchain and executable/library hashes. No global workspace-clean claim is made.
