# Rocq exact-real-arithmetic audit

Repository: `rocq-community/exact-real-arithmetic`, shallow checkout pinned at `d6bc1fee859c5b773dc46ef8efd4d43ba29697c5`.

## Coverage and validation

All 35 tracked files (5,471 physical lines) were read completely in bounded numbered reads: every `.v` definition/proof, README, build metadata, description, `.gitignore`, and the full 504-line LGPL license. No generated artifacts are present. `coqc`, `rocq`, and `coq_makefile` are unavailable, so the declared `make` validation cannot run; no proof result is claimed from source inspection alone.

## Representation and algorithms

The development represents a real by a function `Reelc := Z -> Z`; at index `n`, the integer approximates `x * B^n` within one unit (`encadrement`). The base `B` is an abstract natural parameter, constrained axiomatically by `B >= 4`. Addition shifts both inputs by one index and rounds with `nearest_Int`; multiplication chooses operand-dependent indices from `msd` and `p_max`, multiplies integer approximants, then rounds at a compensating scale. Inverse uses sign-aware integer division with a ceiling helper, and square root uses `Z.sqrt` at doubled indices. Correctness is expressed as Coq lemmas over the standard real field.

The proofs contain useful design facts but not a production scalar implementation: precision is a logarithmic integer index, multiplication demand is magnitude-sensitive, and all error budgets are explicit. This resembles Hyper's dyadic approximation planning and supports the existing “evaluate difficult/large-magnitude operand first” strategy. The representation is function-based and recomputes every requested index; it has no memoization, DAG sharing, concurrency model, or performance measurements.

## Defects and trust boundary

`README.md` explicitly marks the development unsafe: many operations are abstract parameters/axioms and `Axiomes.v` proves `False` from `msd_c` by applying it to the all-zero function. `Inverse.v` ends with `Abort`, so its inverse proof is unfinished despite earlier lemmas. Several files rely on classical axioms and old Coq-era tactics/names. Consequently the source is valuable as a specification of index/error formulas, not as a sound proof certificate or code donor.

## Transfer decision

No Hyper production change was selected. The only plausible transferable concept—magnitude-aware index planning with a fixed rounding/error budget—is already implemented more strongly in Hyper's exact dyadic scheduler, which caches a finest approximation and derives coarser requests without recomputing the whole function. Porting this function-valued representation would lose immutable DAG sharing, cache identity, thread-safe refinement, and explicit unknown/domain contracts. The repository's unsound axioms and unfinished inverse proof make it unsuitable for strengthening Hyper's exactness or completeness.

See `ROCQ_EXACT_REAL_ARITHMETIC_FILE_INVENTORY.tsv` for the file-by-file ledger.
