# Explicit domains in selected circle projection

Committed Hypercurve `72d831c46241929d0621969c372262c78a4cd88f`.

Parent: Hypercurve `8228394f174be2ae607694682defb167f030f540`. HyperBREP remains `878fcb441b281ab1b44051ca4420b28092c3c2ed`. This is a bounded implementation step within the active architecture goal; it does not complete the selected-circle public API migration.

## Change

Seven private selected-axis signatures replace optional narrow ranges and geometric incident descriptors with the existing borrowed `CurveParameterDomain2`. Dense projection, cached univariate isolation, recursive systems, represented-center scheduling, represented circle systems and direct-pair projection now pass that domain to one shared scheduler. The scheduler isolates the actual finite interval, then the optional regular ray, and gives the finite interval ownership of shared roots. The old unit-root allocation/copy/filter loop and its former helper name are removed. Unit-interval cached projection remains guarded by an actual unit interval.

The selected-fiber interval worker runs exact decisions inside a strict predicate scope while retaining the original context for selected-endpoint comparisons. It no longer substitutes a strict context that rejects the caller's retained boundary evidence. No new algebraic solver, compatibility alias, public wrapper or dependency is introduced.

Three geometric workers still convert their existing optional ordinary ranges at the boundary of this migrated scalar layer. Removing those remaining optional ranges, consolidating the three selected-circle entry points, migrating bounds and rational shortcuts, and removing the public exterior-domain guard remain subsequent work. Positive-dimensional mapped overlap storage and inverse replay also retain narrow endpoint assumptions that need a careful migration.

## Parent reproductions

`finite-circle-projection-parent-results.json` records two failing regressions on the committed parent plus test-only additions. The corresponding executable, source patch and 1,005-file source manifest are retained.

- The first factored-polynomial exterior case requested the closed interval `[-3,-2]`, whose three independently specified roots are `-3`, `-5/2`, and `-2`. The parent scheduler returned zero roots.
- Selected-fiber projection rejected a selected boundary with `Topology("a selected-fiber scalar crossed predicate policies")` after the test modeled an earlier policy terminal in the same composite operation. The exact scalar values and root relations remain fixed; the failure concerns retained policy identity.

The initial harness used the wrong Rust test module, matched zero tests, and is archived under `finite-circle-projection-parent-zero-filter-attempt1/`. It is not defect evidence. The corrected execution matched exactly one test per command and both failed for the reasons above.

## Candidate checks

The factored-polynomial test covers 72 combinations of both policies, unit/negative/exterior translated charts, traversal reversal, finite-only/increasing-ray/decreasing-ray domains, and both square-free and ordinary isolation paths. Expected roots come from the explicit factorization. Closed finite endpoints have one owner and open ray barriers are excluded.

The selected-endpoint test covers 12 projections across both traversal directions, both policies, and an already-consumed policy terminal. Its endpoint values are `1+sqrt(2)` and `3+sqrt(2)`, represented as affine images of roots of `(2u-alpha)(2u-alpha-1)` with `alpha=sqrt(1/2)`. Deliberately coarse certified endpoint brackets give an outward envelope `[1,5]`; projection of `(t-2)(t-3)(t-4)(t-5)` must retain only `3` and `4`. Endpoint global-polynomial caches must remain empty. The first compile-only candidate used the fixture's automatically refined endpoint brackets; it is archived under `finite-circle-projection-compile-attempt1/`. The final test deliberately widens them to make the clipping requirement explicit.

Final qualification is recorded in `finite-circle-projection-qualification.json`: 2,296 passes across 49 targets (2,064 Hypercurve and 232 HyperBREP), five unchanged known failures, nine ignored tests, eight existing expensive exclusions, and no new failures or broad-run timeouts. All eight focused regressions and all 34 existing public probes pass. The no-default-feature caller checks, all-feature release builds, formatting, 396 working-source hashes, 1,005 isolated-source hashes and artifact hashes verify. The successful qualification writer is complete and must not be rerun against changed sources.

The archived final normal library has SHA-256 `6c28d291cd5328445f25eacc8c02c951baf90fd360386492128e9e4ae5795757`; the libtest artifact has SHA-256 `358b2e855b707f281aa5e3c96ba8b064a48533085c94c1d77709e99b8bfa2cff`. Both are retained under `finite-circle-projection-libraries/`.

## Scope and ownership

The independently derived public selected-circle/non-PH matrix remains a separate 32-case fixture. Its parent baseline completes 16 native-chart cases and blocks 16 exterior-chart cases. The final candidate repeats exactly those counts in `finite-circle-analytic-domains-projection-check.json`, bound to the final normal-library and public-source hashes. The outer guard is unchanged; this expected remaining limitation is not counted among passing public qualification probes.

All builds use pinned committed dependencies. The other session's Hyperreal verification and associated changes are untouched. No global workspace-clean claim, general throughput improvement, or measured memory/binary-size improvement is made. The broader architecture goal remains active, including the remaining circle domains, selected-point incidence, source-cusp endpoint frames, normalized region admission, independent inverse replay, known failures/exclusions and algebraic/computational consolidation.
