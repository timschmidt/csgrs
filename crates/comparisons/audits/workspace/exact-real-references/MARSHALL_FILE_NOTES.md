# Marshall audit notes

Repository: `andrejbauer/marshall`, pinned at `c9f1f6466e879e8db11a12b9bc030e62b07d8bd2`.

## Coverage

All tracked text/source files were read completely in bounded, numbered reads. This includes the OCaml executable, parser, evaluator, interval/region and dyadic backends; the Haskell `Staged`, `Dyadic`, `Interval`, `Reals`, `Space`, `Searchable`, `Lipschitz`, monad tutorial and limit test; README/license/build metadata, examples, the 951-line topology notes and 557-line SCAN slides. EPS/FIG/PDF figures were classified by `file`/repository inventory (generated explanatory artwork, not source lines); their companion TeX include file was read. There are no generated binaries in the checkout.

Native validation was attempted. `ocaml`, `dune`, `ghc`, and `runghc` are unavailable in the execution environment, so the OCaml/Haskell test programs could not be compiled or run. `src/dyadic_test.ml` is an output smoke test for MPFR and bignum backends; `etc/haskell/test_lim.hs` is an unfinished/obsolete limit experiment. No benchmark result is claimed.

## Architecture and transferable ideas

* Marshall models a real as a Dedekind cut: paired lower/upper predicates, with bounded cuts refined by thirds and interval Newton estimates. `approximate.ml` computes lower/upper approximants with directed rounding; `eval.ml` repeats refinement until a requested interval width is reached. Equality is intentionally partial; `<` is semidecidable and propositions use conservative lower/upper truth values.
* The Haskell prototype makes the staging contract explicit: `Staged (Interval q)` is a reader-like precision/rounding function, and `Stage` carries `RoundDown`/`RoundUp` plus precision. This cleanly separates requested precision from scalar storage and is conceptually useful for Hyper's enclosure APIs.
* `Interval.hs`/`interval.ml` implement Kaucher (back-to-front) intervals, including sign-table multiplication and reciprocal/division. `Region` adds Boolean algebra of open/closed interval regions. These are useful semantic references for unknown/indeterminate results, but Hyper already has stronger structured enclosures and explicit domain contracts.
* Searchable/compact/overt abstractions (`Searchable.hs`, `Space.hs`, `Reals.hs`) express existential witness search and universal verification as Sierpinski-valued computations. This supports a principled partial-comparison API, but the prototype's compact search is a simple queue/bisection strategy and its `exists` implementation is explicitly unimplemented.
* `newton.ml` and the slides describe derivative/Lipschitz-assisted interval Newton refinement. Hyper could consider this only as an opt-in refinement hint after a measured workload demonstrates benefit; no safe drop-in change was identified.
* The scalar backends are not production-ready: `dyadic_num.ml` has a no-op normalization and a likely denominator typo in `halve` (`numerator_ratio` used for `q`), while the Haskell dyadic implementation normalizes by first doing exact big-integer arithmetic. `Dyadic_mpfr` is a thin MPFR wrapper and the repository itself says MPFR integration is not configured.
* `eval.ml` uses repeated AST substitution with alpha-renaming and heuristic precision growth. This is simpler but can duplicate terms and grow exponentially; it is weaker than Hyper's graph/caching approach for performance and memory.

## Decision

No Marshall code was copied into Hyper. The worthwhile lessons are already represented by Hyper's structured interval/unknown semantics: preserve directed lower/upper evaluation, make precision a first-class request, and treat comparisons/quantifiers as partial. The Kaucher sign-table and symbolic Newton ideas remain documented candidates, but without native tooling or benchmark evidence they do not meet the exactness-first “worthwhile change” bar.

## File-level inventory

See `MARSHALL_FILE_INVENTORY.tsv` for every tracked path, line count, and classification. Binary/generated figures are classified rather than line-audited.
