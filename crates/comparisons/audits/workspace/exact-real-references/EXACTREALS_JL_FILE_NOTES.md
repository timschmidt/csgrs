# ExactReals.jl audit — 2026-09-07

Source: `dpsanders/ExactReals.jl`, pinned commit `19d32155e69ad988cd68282d4e88b859552033e3`.
All 11 tracked files and 382 physical lines were read. Julia 1.11 is available; the package-registry test command was quota-blocked, so tests were run directly by loading `src`.

| File | Findings |
| --- | --- |
| LICENSE | MIT; no algorithmic content. |
| .appveyor.yml, .travis.yml, .gitignore | CI/build metadata and ignore rules; no scalar algorithm. |
| Project.toml | Julia 1.x package with stdlib Test only; no lockfile or numerical dependency. |
| README.md | Describes fast Cauchy/ Böhм-style scaled integers and only `+`, `-`, `*`; examples display Float64 intervals. The logistic example is illustrative, not an oracle. |
| src/ExactReals.jl | Exports `ExactReal` and `value_range`; includes two implementation files. |
| src/exact_reals.jl | Mutable struct stores one function, one finest `(n,BigInt)` cache and an MSD hint. Coarser queries use arithmetic right shift, with no synchronization or nearest-rounding proof. Rational construction uses integer division; `value_range` converts directed endpoints to Float64. `show` is an approximate display. |
| src/arithmetic.jl | Addition and multiplication use fixed guard formulas and cached MSD. `msd` loops forever for exact zero. Consequently the valid public expression `value_range(ExactReal(0) * ExactReal(1), 16)` did not terminate within 3 seconds (Julia terminated with a stack trace in `msd`/`BigInt` growth, exit 124). `sqrt(ExactReal(-1))` raises `DivideError`, not an explicit domain result. Comparison is an unbounded loop and `<` has no unknown/timeout result. |
| examples/logistic.jl | Compares one exact-real logistic trajectory with Float64 after 54 iterations; no independent interval oracle. |
| test/runtests.jl | Eight tests cover constructors, simple arithmetic and comparisons only; no zero multiplication, negative roots, concurrency, precision history, or independent exact oracle. |

Disposition: no production transfer. Hyper already has immutable DAG nodes, synchronized caches, explicit unknown/domain errors, and proof-bearing integer approximation. The only useful Cauchy-cache idea is already subsumed; this package’s zero nontermination, mutable state and Float64-facing contract are weaker.
