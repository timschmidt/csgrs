# OCaml Creal audit — 2026-09-07

Source: `backtracking/creal`, pinned commit `0520c6351fdc431db3c0941b5cf111211826f88a`.
All 13 tracked files and 1,850 physical lines were read completely. No OCaml/dune/opam runtime is installed, so native build/test results are not claimed.

| File | Findings |
| --- | --- |
| CHANGES.md | Two historical fixes (`exp`, `mul_Bexp`); no new contract evidence. |
| CReal.mli | Base-4 integer approximation contract, mutable-cache implementation hidden behind an abstract type, explicit nonterminating `compare`, bounded `rel_cmp`, broad elementary API, and decimal formatting contract. The API exposes no explicit Unknown/domain certificate. |
| CReal.opam/Makefile/dune/dune-project | Small Zarith/Dune library and one graphics example; no pinned compiler or reproducible benchmark harness. |
| README.md | Points to Ménissier-Morain’s thesis; no operational guarantees beyond the interface. |
| CReal.ml 1–220 | Each value stores a mutable `(precision,integer)` cache and mutable MSD. `approx` coarsens cached values through arithmetic right shift (`fdiv_Bexp`) rather than nearest rounding, so an admissible callback approximation can lose the strict one-unit enclosure at a coarser demand. Mutable caches are unsynchronized. Addition/negation/abs are lazy closures; MSD search intentionally does not terminate at zero. |
| CReal.ml 221–440 | Multiplication and inverse use MSD-derived guard formulas; inverse and roots raise on unresolved/negative cases. `arctan` branch selection uses a single coarse approximation, and arcsin/arccos derive through divisions whose endpoint uncertainty can remain unresolved. Exact rational square roots are recognized. |
| CReal.ml 441–660 | Trig range reduction, logarithm, exponential and hyperbolic functions use rational series and global mutable `e`/`pi` caches. `tan` is a raw `sin/cos` quotient with no certified pole result. The exponential path includes empirical constants and precision heuristics rather than a proof-bearing domain/error object. |
| CReal.ml 661–820 | Absolute comparison may diverge; `rel_cmp` returns 0 for overlapping intervals by design. `of_float` imports the host float as an exact Zarith rational, while formatting uses a single low-precision decimal decision and may emit p+1 digits. `min`/`max` memoize the first separated operand in mutable closures. |
| test.ml/test.expected/draw_sin.ml | Sanity/benchmark output and a GUI plotter, not independent enclosure or concurrency tests. The expected file records historical decimal outputs only. |
| LICENSE | LGPL-2.1 text plus linking exception; no algorithmic content. |
| .gitignore | Ignores build artifacts; no algorithmic content. |

Disposition: no production transfer. Hyper already uses immutable DAG nodes, synchronized monotone finest-value caches, explicit bounded/unknown decisions, exact rational import, and proof-bearing elementary kernels. Creal’s mutable global/cache state, arithmetic-shift coarsening, coarse branch heuristics, and raw pole division are weaker exactness/completeness contracts. The only reusable idea—base-4 integer approximants with explicit requested precision—is already subsumed by Hyper’s integer approximations.
