# Haskell exact-real / Data.CReal per-file audit

Snapshot: `7bb2abaed01ac874d914c99919e81cfd00dd7d6d`, 2026-06-11.
All36 tracked text files /2,466 physical lines have been read. The inventory
and read-coverage TSVs give exact lengths, hashes and ranges. The unchanged
499-test native suite and29 doctests pass on GHC9.6.7. Independent337,456
Rational arithmetic and15,379 directed MPFR elementary checks pass. Separate
precision-history tests demonstrate invalid retained signum/atan2 values and
a false convergence result. This is not a universal correctness claim.

| File | Comparison disposition |
| --- | --- |
| `.github/workflows/bump-command.yml` | Version-bump automation; no scalar idea. Reviewed only, no external actions invoked. |
| `.github/workflows/ci.yml` | GHC8.8–9.2, Stack8.8.4 and Nix gates; modern-host qualification must name its compiler. |
| `.github/workflows/command-dispatch.yml` | PR command routing; no numerical transfer. |
| `.github/workflows/release.yml` | Source/docs packaging and release publication; no scalar transfer or publication invoked. |
| `.github/workflows/tag-release.yml` | Tag automation; no scalar transfer. |
| `.gitignore` | Build/cache exclusions read; no algorithm. |
| `LICENSE` | MIT conditions read; no donor implementation copied into Hyper. |
| `Setup.hs` | Cabal doctest hook; installed test dependencies must be qualified separately. |
| `bench/Bench.hs` | Four retained expressions at six precisions, WHNF integer demand. Repetition can primarily measure a warm cache, not fresh evaluation; independent cold/warm drivers are necessary. |
| `changelog.md` | Reports a large memoization speedup in0.12.4; historical claims are not current measurements. |
| `default.nix` | Pinned nixpkgs development environment; not numerical code. |
| `exact-real.cabal` | Generated manifest read anyway; exposes internal representation and bounded helpers, custom doctest setup, native tests and Criterion suite. |
| `package.yaml` | Source package description and broad dependency bounds; agrees with the current Cabal structure. |
| `readme.md` | Precision-parameter conversions and approximate Eq explicitly documented. Euler identity's displayed zeros are not symbolic equality proofs. |
| `release.nix` | Source/docs build checks; no scalar transfer. |
| `src/Data/CReal.hs` | Small safe-facing export surface, with Internal still separately public. Hyper has a richer proof/query boundary. |
| `src/Data/CReal/Converge.hs` | Precision-adapted adjacent-equality/error-stagnation stopping without a convergence modulus or tail enclosure. Finite-list fallback returns the last element. The independent eventually-constant plateau counterexample returns the wrong limit. Reject as a general certified limit constructor. |
| `src/Data/CReal/Internal.hs` | Precision-indexed integer functions, one finest MVar cache, nonblocking unary cache seeds, guard-bit multiplication, Newton integer sqrt, Machin pi, bounded series and binary range reduction. Hyper already has finest caches, specialized squares/scales and magnitude-aware kernels. Donor computes while holding the cache lock, unlike Hyper's short publication lock. Independent elementary controls pass, but retained signum/atan2 and display contracts fail as detailed below. Eager Hyper cache seeding is experimentally rejected; no direct transfer retained. |
| `stack.yaml` | LTS16.20 resolver; current host's existing LTS22.44 is a separate compatibility environment if used. |
| `stack.yaml.lock` | Stale LTS11.22 snapshot and memoize package, inconsistent with current resolver/manifest; not a reliable current lock. |
| `test/BoundedFunctions.hs` | Compares specialized functions to general functions built from related kernels, using approximate equality; not an independent enclosure oracle. |
| `test/Data/CReal/Extra.hs` | Random rational-like inputs and approximate EqProp; input bias and equality precision affect coverage. |
| `test/Doctests.hs` | Generated doctest runner; no numerical algorithm. |
| `test/Floating.hs` | Broad identities and inverse relations, with shared kernels and relaxed comparisons. These cannot prove the absolute integer-approximation invariant. |
| `test/Fractional.hs` | Algebraic laws and reciprocal/rational consistency; zero exclusions inherit approximate order. |
| `test/Num.hs` | Algebraic and abs-signum identities use approximate equality, potentially hiding an invalid retained signum object. |
| `test/Ord.hs` | Non-strict total-order tests intentionally disabled because approximate equality breaks their laws. Do not adopt this as exact order. |
| `test/Random.hs` | Range checks at the phantom precision; equal bounds exercise log2 zero indirectly. |
| `test/Read.hs` | Read/show round trip only; independent16665-case check finds128 composed display-bound failures on valid one-sided Cauchy procedures. The standalone rational decimal kernel passes10945 exact rounding checks. |
| `test/Real.hs` | Rational conversion error check is one-sided and uses1/max(1,p), not2^-p. Far weaker than the claimed absolute error contract. |
| `test/RealFloat.hs` | Conversion/scaling identities and weak atan2 range checks; no independent quadrant or small-input angle oracle. |
| `test/RealFrac.hs` | Relaxed comparisons for integer/fraction laws; exact directional rounding boundaries require independent checks. |
| `test/Test.hs` | Five phantom precisions, algebraic/convergence identities and decimal properties; no independent Cauchy-history oracle. Collatz is a test input family, not a convergence proof. |
| `test/Test/QuickCheck/Classes/Extra.hs` | Algebraic property tree and relation helpers; useful test organization, not a new exact-real method. |
| `test/Test/QuickCheck/Extra.hs` | Numeric wrappers, bounded random generators and shrinkers; most validation compares through donor Eq. |
| `test/Test/Tasty/Extra.hs` | TestBatch adapter only. |

The independent plateau stream `[0,0,1,1,...]` has limit1 but Converge returns0.
For signum(1/1024) and atan2(1/1024,1/1024), a2-bit request initially yields
the interval[-1/4,1/4], disjoint from the later16-bit enclosure. These are
retained-value invariant failures, not just documented approximate Eq/Ord.

A64-observation donor diagnostic compares seeded unary result caches with
explicitly cleared result caches at128/512 bits. Seeding benefits this donor,
whose negate/abs/integer-add callbacks call the underlying precision function
directly. Clearing also allocates an extra MVar, so it is NOT a fair isolated
policy A/B or evidence of the same gain in Hyper. Hyper's retained Negate path
already queries its child's cache. See the root ledger and CacheBench.hs for
measurement boundaries. The subsequent isolated Hyper A/B passes432 MPFR
controls and900 exact seed endpoint checks, but480 paired CPU timings and120
allocation controls reject generic eager seeding: warmed unqueried negations
cost about80% more and squares4–6x more, despite11–23% warm-query improvements.
The entire prototype remains outside production.

Hyper baseline counterparts pass2345 exact-rational decimal checks,120 MPFR
tiny-coordinate quadrant/history checks through scale2^-2500, and20 certified
signs. These are scoped controls, not proofs for every formatted expression.

Current status: complete source read, numerical qualification and targeted
transfer comparison. No new production change retained. Root ledger holds
experiment chronology, rejected prototypes and all explicit limits.
