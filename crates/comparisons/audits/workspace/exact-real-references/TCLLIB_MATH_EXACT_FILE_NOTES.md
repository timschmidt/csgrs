# Tcllib `math::exact` audit notes

Repository pin: `6093f8d6246572ae461bf344333483ca329413bc`.

## File coverage

All six exact-real package artifacts were read completely, line by line, in bounded numbered ranges: `modules/math/exact.tcl` (4060 lines), `modules/math/exact.test` (2255), `modules/math/exact.man` (219), generated Markdown (265), generated nroff (489), and generated HTML (307). The generated documentation agrees with the source manual apart from inherited documentation typo `pi = 3.15159...`.

## Architecture

`math::exact` is a TclOO lazy computable-real evaluator. Integer rationals are normalized as two-element vectors; 2x2 matrices are Mobius transformations `(a*x+b)/(c*x+d)`, and rank-3 tensors compose left/right transformations. Refinement emits continued-fraction/digit information only when `asPrint`/`asFloat` requests it. Workers implement sqrt, exp, log, trigonometric and inverse/hyperbolic functions. Expression nodes memoize sign/magnitude, leading digit/rest, and absorption results.

The implementation uses explicit intrusive reference counts (`ref`/`unref`) and a deferred deletion stack to prevent recursive destruction of deeply nested expression graphs. This is the clearest memory/stack idea for comparison with Hyper, although Hyper's immutable DAG ownership and synchronized caches provide a stronger concurrency model.

## Tests and runtime probes

`tclsh modules/math/exact.test` on Tcl 9.0.2: **198 passed, 0 skipped, 0 failed**. A documented reference-counted probe for `sqrt(2)+1/3` produced `1.74754689570642838e0` and released cleanly. A probe omitting the documented initial `ref` caused an expected invalid-object error after use, confirming the manual's ownership contract.

## Findings / transfer decision

* Useful concepts: Mobius/tensor refinement state is compact; deferred destruction avoids recursive destructor stack overflow; exact rational fast paths and memoized digit generation reduce repeated work; explicit tolerance-style `signum1`/`abs1` APIs make their non-exact classification contract visible.
* Not transferable as-is: Tcl integer/object overhead, single-threaded mutable caches, manual ownership, and parser-only decimal integers are unsuitable for Hyper. Domain failures are allowed to loop or overflow (e.g. `atan(1/0)`, inverse-trig endpoints/out-of-domain values, and some real powers), weaker than Hyper's structured unknown/domain contracts.
* Static issue: documentation says `pi = 3.15159...` (typo). TclOO class commands support implicit construction, so `T.applyTLeft`'s `Mstrict` call is valid. `rat**int` delegates exponentiation to Tcl integer arithmetic, so its range/performance follows Tcl's bignum implementation.
* **No production Hyper change selected.** The deferred-destruction pattern is already subsumed by Hyper's ownership architecture and adding a second reclamation mechanism would increase complexity without measured benefit. Mobius state remains an architectural candidate only for a future specialized stream kernel.

