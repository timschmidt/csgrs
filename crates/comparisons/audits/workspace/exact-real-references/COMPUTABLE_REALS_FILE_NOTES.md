# Common Lisp computable-reals audit — 2026-09-07

Source: `stylewarning/computable-reals`, pinned commit `607a5d5b95387c06f92a661aa5562a7be0f5cad8`.
All 8 tracked files and 877 physical lines were read completely, with hashes recorded in the workspace audit ledger. No local Common Lisp runtime is installed, so no native execution claim is made.

| File | Findings |
| --- | --- |
| LICENSE | BSD-3-Clause text; no algorithmic content. |
| README.md | Defines a Cauchy-style integer-at-binary-precision contract, rational approximation, continued-fraction simplification, and explicitly non-decidable external comparisons. The dynamic tolerance is an approximate internal policy, not an exact equality result. Hyper already exposes explicit bounded/unknown decisions and one-integer approximations. |
| computable-reals.asd | Serial five-file system, version 1.1.0; no test system or dependency pin. |
| package.lisp | Exports arithmetic, elementary functions, approximation, formatting, and rounding APIs; no separate certified/unknown comparison type. |
| constants.lisp | Forward-declared mutable global constant cells. |
| get-approximations.lisp | Initializes log2 and pi-family values through Machin-style arctangent expressions, then eagerly asks for 200-bit approximations at load time. This creates global startup work and mutable shared state; no transfer to Hyper, whose constants are immutable/lazy and cached per node. |
| reals.lisp 1–220 | `c-real` stores one cached integer/precision pair and a callback. `get-approx` reuses coarser cached values with arithmetic right shift rather than nearest rounding; for negative values this can spend the full extra unit and is weaker than Hyper’s retained nearest/ties-up coarsening. `rationalize-r` uses continued fractions to choose a simple rational, but comparisons remain tolerance-based. Formatting uses a decimal estimate and approximate sign, so it is intentionally output-only. |
| reals.lisp 221–420 | Addition collects rationals and real operands and allocates one callback; multiplication performs raw-magnitude budgeting and caches only one finest approximation. Division, roots, and round/floor/ceiling/truncate use heuristic guard formulas and throw on unresolved zero. `ash-r` handles negative shifts by coarse rounding. Hyper already has exact structural reduction, explicit unknown outcomes, and synchronized immutable-node caches; no implementation transfer is justified. |
| reals.lisp 421–646 | Log/exp/trig use range reduction plus rounded series. `atan-r` quadrant dispatch is driven by finite approximations, while `tan-r` is a raw sine/cosine quotient and does not expose certified pole uncertainty. The API has no domain certificate or partial comparison result. These are semantic limitations, not safe Hyper patches. |
| realstst.lisp | Demonstration script only: prints constants, radicals, transcendental values, and continued-fraction coefficients; it is not an oracle or regression suite. |

Disposition: source and architecture audit complete; no worthwhile production change. The useful design lesson—one recomputable approximation callback with explicit approximation requests—is already subsumed by Hyper’s immutable DAG and synchronized finest-value cache. The donor’s mutable globals, approximate tolerance comparisons, arithmetic-shift coarsening, and eager startup constants are rejected on exactness/completeness grounds.
