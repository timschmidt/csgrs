# ireal per-file audit dispositions

Pinned source: `a7c83281362c5dc857e97d98b424ec314a216b56`.
Every tracked text line is accounted for in `IREAL_READ_COVERAGE.tsv`; exact
file lengths/hashes are in `IREAL_FILE_INVENTORY.tsv`. This document records
the comparison disposition, not a claim that every possible input was tested.
Numerical evidence and current transfer candidates are in the workspace-root
`EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md` and the `/tmp/ireal-audit.Sc4TzO` probes.

| File | Ideas and disposition |
| --- | --- |
| `Data/Number/IReal.hs` | Unified point/interval API, partial comparison and redundant decimal output. Hyper deliberately separates exact values from evidence/enclosures; do not erase that distinction. |
| `Data/Number/IReal/Auxiliary.hs` | Logarithmic guard-bit estimates and decimal-to-binary conversion. Similar machinery exists in Hyper; no new checked conversion or precision representation selected. |
| `Data/Number/IReal/FAD.hs` | Binomial convolution avoids exponential symbolic product differentiation. Exact coefficient/derivative probes pass. Hypercurve already has quotient/Horner recurrences; its demonstrated machine-word coefficient and endpoint-factorial limits are repaired in retained commit d978852, with exact dense/sparse high-order closed-form regressions and full proportional qualification. Finite polynomial tails become infinite under addition in this donor. |
| `Data/Number/IReal/FoldB.hs` | Balanced reduction and one-final-rounding n-ary sum. The generic carry fold reverses inputs; Hyper's carry reducer preserves order. Controlled donor and pinned Hyper tests show strong dependency sensitivity. Raw-depth scheduling fixes the shared-prefix gap but regresses decaying prefixes4.5x and is rejected. Signed scale-aware demand ordering is retained in21e76ea after150 independent MPFR controls,96 paired scaled timings,108 construction counters,8 Memcheck controls, full proportional gates and layout/size checks. The512-prefix case improves about99x without the scaled counter-regression. It changes neither mathematical precision nor numerical facts. Blanket reversal and wholesale n-ary replacement remain unjustified. |
| `Data/Number/IReal/Generators.hs` | Signed-bit random values, expression generators, approximate consistency checks and compact-space search. Highest-first memoization can mask a broken precision function in the Cauchy test; independent references are essential. Search is not a general total decision procedure. |
| `Data/Number/IReal/IReal.hs` | Precision-indexed endpoint functions and shared memoization wrapper. Hyper already retains demand-driven immutable computations. |
| `Data/Number/IReal/IRealOperations.hs` | Range reductions, guard bits, endpoint handling, interval constructors/selectors, comparison and output. MPFR finds inward-rounded interval sqrt; zero-derived sign tests can claim positive. Point kernels pass sampled independent tests. Frozen enclosures and boundary nontermination are not a replacement for Hyper exact closure. |
| `Data/Number/IReal/IntegerInterval.hs` | Small sign-case endpoint kernels. Exact corner oracle finds wrong crossing-left/negative-right multiplication upper bound. Reject direct kernel transfer. |
| `Data/Number/IReal/Powers.hs` | Separate correlated square/power operations avoid interval dependency inflation. Hyper already retains square structure. |
| `Data/Number/IReal/Rounded.hs` | Type-level decimal enclosure budget can bound repeated refinement/storage. This freezes a potentially wide interval, not a computable point. It inherits interval defects and incomplete RealFloat support. |
| `Data/Number/IReal/Scalable.hs` | Binary shifts with outward endpoint rounding and nearest integer rounding. Requested accuracy conventions differ from Hyper; no novel proven improvement selected. |
| `Data/Number/IReal/UnsafeMemo.hs` | MVar stores one finest approximation; lower precision downscales. Non-atomic check/update can replace a finer entry with a coarser one under concurrency; thunks/exceptions remain relevant. Hyper already uses a synchronized finest cache; no concurrency stress proof or donor transplant claimed. |
| `LICENSE` | BSD-style redistribution conditions read; no donor implementation copied into Hyper. |
| `README.md` | Minimal package pointer; no additional implementation claims. |
| `Setup.hs` | Standard Cabal setup; no scalar transfer. |
| `applications/ClenshawCurtis.hs` | DCT-generated quadrature weights and precision-capped intermediates. Exact monomials expose the two-point/one-weight n=0 mismatch. Truncation proofs are explicitly left to the caller. |
| `applications/ClenshawRounded.hs` | Fixed-precision variant; same quadrature contract limitations. No new Hyper quadrature layer justified. |
| `applications/Erf.hs` | Generic power-series extension with an explicit finite input range. Sampled point/interval MPFR controls pass, but no new arbitrary-range special-function implementation is warranted from this example alone. |
| `applications/FAD.hs` | Older convolution-based AD version, separate from the installed module. Same reuse idea; library version was numerically qualified. No independent full legacy AD suite claimed. |
| `applications/FFT.hs` | Decimation-in-frequency, power-of-two preconditions and DCT/DST variants. Modern Complex division reaches undefined IReal RealFloat methods, breaking unchanged inverse-FFT controls. Inverse DCT length2 gives infinities even with Double. |
| `applications/FFTRounded.hs` | Precision-wrapper FFT variant; same length2 inverse-DCT issue and partial RealFloat concerns. No wholesale transform transfer. |
| `applications/Integrals.hs` | Adaptive Taylor integration with interval remainder and bisection. Exact polynomial controls pass; no global termination or arbitrary-integrand completeness claim. |
| `applications/IntegralsRounded.hs` | Success branch literally returns1; executable constant-integrand example confirms wrong integral. Reject this unfinished implementation. |
| `applications/LinAlg.hs` | Column-oriented LQ/Householder factorization, balanced dot products and hardcoded precision cap. Four small exact systems pass independent closed-form checks. Hyper's exact elimination/certification already addresses relevant consumers without this fixed cap. |
| `applications/ListNumSyntax.hs` | Zip-based list arithmetic; silently truncates mismatched shapes and uses infinite constant lists. Hyper's typed/checked vector and matrix dimensions are preferable. |
| `applications/Logistic.hs` | Manual Lipschitz precision budget avoids nested refinement explosion. A frozen enclosure must still prove achieved width; do not replace exact values with a guessed fixed precision. |
| `applications/MoreDigits.hs` | Historical benchmark recipes and explicit unsolved/heuristic cases. No historical timing accepted as a present benchmark, and no digit agreement accepted as a truncation proof. |
| `applications/MoreDigitsRounded.hs` | Fixed-budget speedups for differentiation, matrices and quadrature. Useful workload ideas, not evidence that a frozen interval meets Hyper's scalar contract. |
| `applications/Newton.hs` | Precision-staged interval Newton and recursive root enumeration. Affine-root controls pass. Equality, multiple roots and proof failure need explicit unresolved outcomes; Hyper already exposes such policy/evidence distinctions. |
| `applications/ODE.hs` | Taylor/Picard-Lindelof enclosure recipe with ad-hoc precision and step controls. Endpoint-flow assumptions and interval substrate would require separate proofs; not transferred. Optional plotting process not executed. |
| `applications/Para.hs` | Sparked independent blocks and bounded sum demands. The forcing strategy only reaches weak-head normal form; published dual-core timing is not current evidence. No automatic threading addition justified. |
| `applications/Plot.hs` | Explicitly experimental unsafe process-backed plotting adapter. No scalar idea; no GUI/process side effects invoked. |
| `applications/Taylor.hs` | Factorial-scaled derivative series and Lagrange remainder. Standard finite-jet machinery; requires valid derivative interval enclosures. |
| `applications/Zeta.hs` | Accelerated zeta(5) identity and warning about shared-prefix evaluation order. Independent synthetic dependent-sum benchmarks corroborate the scheduling warning. No new infinite-series object or tail proof imported. |
| `changelog.txt` | Version chronology, memoization, convolution AD and fixed-precision wrappers reviewed against source. |
| `doc/ireal.tex` | All977 lines, including material after end-document, read. Strict enclosure claims conflict with some producers; proof details are omitted. Historical sum tables are not qualification data. |
| `ireal.cabal` | All12 installed modules build unchanged on GHC9.6.7. Native tests/examples are standalone, not Cabal test components. Cabal-version warning preserved. |
| `tests/IRealTests.hs` |46 properties /4,303 native cases pass. Mostly random positive point cases, identities and memoized consistency; independent interval/cold-reference testing adds material coverage. |

Binary artifact: all9 pages of `doc/ireal.pdf` were rendered and visually read,
including its formulas and two historical sum tables. No missing figures or
unreviewed binary executable is present in this donor tree.

Transfer closure: complete for the ideas identified in this donor. Retained
changes are Hypercurve d978852 (exact high derivatives) and Hyperreal21e76ea
(scale-aware additive scheduling). Numerical failures and rejected variants
are preserved in the root ledger; unrelated cross-reference candidates remain
open and the overall ecosystem audit is not complete.
