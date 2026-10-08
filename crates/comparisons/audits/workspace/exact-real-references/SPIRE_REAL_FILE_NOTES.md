# Spire exact-number audit notes

Repository pin: `0fe5a6a9714181a20fc9cef4c8b2af088ff2b4c9`.

## Coverage and tests

The five relevant source/test files were read completely, line by line, in bounded numbered ranges: `Real.scala` (688 lines), `Algebraic.scala` (1604), `IsReal.scala` (87), and the two ScalaCheck suites (240 and 340), for 2,959 lines total. A local `sbt` executable is unavailable, so the Scala test suites could not be executed in this environment; their complete source and property coverage were audited instead.

## Architecture

`spire.math.Real` is a signed-binary, precision-indexed function `Int => SafeLong`. `Exact(Rational)` is the fast path; `Inexact` memoizes one highest-precision approximation in a volatile `(bits,value)` pair and rescales it for lower requests. Arithmetic computes guard bits from operand magnitudes, and elementary functions use range reduction plus lazy power series. `Algebraic` is a persistent expression AST over rationals, roots, arithmetic, and integer powers. Exact sign/equality uses cached degree/separation bounds (Li–Yap and BFMSS), then adaptive decimal refinement. Polynomial roots are isolated into rational intervals and refined with an atomic cached root approximator. `evaluateWith` replays an AST into another exact field (notably `Real`).

## Risks / observations

* `Real.signum` and `compare` use a fixed default precision for inexact values, so equality/order can be wrong near zero; `atan2` loops without a precision cap when both coordinates remain unresolved.
* `Real.reciprocal` recursively searches for a nonzero approximation and has no zero/termination contract. `pow(Int.MinValue)` negates the minimum exponent and can overflow. `fpow` truncates rational exponents with `limitToInt`, silently narrowing large exponents.
* `Inexact`'s single mutable cache is race-safe only at the value publication level and can recompute under contention; it does not preserve a multi-precision cache. Expression nodes retain full trees, while per-node `TrieMap` bound caches can grow without eviction.
* `Algebraic` explicitly rejects unsupported/undefined cases (`0^0`, divide by zero, negative even roots), and its separation-bound strategy is a strong exact-sign design. Root refinement uses `AtomicReference`, but bound arithmetic is checked `Long` arithmetic and throws on overflow rather than returning an unknown result.
* Tests include high-precision constants, root isolation, tricky exact zeros, approximation modes, and randomized polynomial/root properties. One Real property is mislabeled (`x * 1 = x`) but tests addition by zero.

## Transfer decision

Separation-bound-guided exact sign evaluation and immutable AST replay are relevant ideas, but Hyper already has structured unknown/domain outcomes, immutable DAG nodes, and stronger precision-aware cache publication. Spire's fixed-precision comparison and unbounded recursion are unsafe for Hyper. **No production change selected.**

