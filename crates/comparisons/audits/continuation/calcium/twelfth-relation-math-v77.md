# Twelfth-turn proof and independent checks

The implementation is a proof-only subfield, not a replacement approximation
representation. Its ordered basis is (1, sqrt(2), sqrt(3), sqrt(6)); the two
square roots are positive. Distinct square classes 1, 2, 3, 6 give an independent
rational basis. Arithmetic uses X^2=2 and Y^2=3. A denominator is admitted only
after its rational field norm is proved nonzero. This also rejects tangent and
cotangent poles; cancellation outside an inverse cannot authorize dividing by
zero inside it.

For a+b sqrt(2), equal coefficient signs determine the sign. For opposite signs,
compare a^2-2b^2 and keep the appropriate original sign. For A+B sqrt(3), first
determine A and B in the quadratic subfield; only when their signs oppose, use
the quadratic sign of A^2-3B^2. This retains the chosen positive embedding,
unlike accepting an arbitrary root of a polynomial or blindly squaring an
equation. Intermediate size is bounded by a fixed number of rational operations
on admitted 1,024-bit coefficients. The proof does not infer rational equality
from a numerical enclosure.

For sqrt(r), r must be an exact nonnegative rational leaf. Trying square classes
1, 2, 3, 6 and exactly squaring both integer roots verifies membership without
factoring. This parser is deliberately incomplete: it does not claim arbitrary
nested radicals or arbitrary algebraic fields.

Angle parsing retains both a rational constant and a pi coefficient. Addition,
rational scaling, rational inverse, negation, binary offset and the exact pi/tau
constants are admitted with the same shared node budget. Products with two
nonzero pi coefficients and inverse(pi) are not admitted. A nonzero rational
residual prevents the pi/12 certificate. Multiplying the exact coefficient by
12 must yield an integer, reduced modulo 24 by integer arithmetic. The table
uses the addition/subtraction formulas for pi/4 +/- pi/6 and quadrant symmetries.
All public branches and enormous period shifts are separately exercised by the
unchanged checkpoint-76 probe against both source versions.

The candidate test oracle does not invoke Hyper's arithmetic to calculate
expected coefficients: it uses num::BigRational, a bivariate polynomial array
reduced by X^2=2/Y^2=3, and rational Gaussian elimination of the multiplication
matrix for inverses. Signs are independently separated by exact dyadic rational
enclosures of the three square roots, not the candidate norm/sign routine.

- 6,561 rational-coefficient tuples cover all signs and zero in a small grid.
- 512 ordered operand pairs check complete product coefficients, inverse
  coefficients from the matrix solve, and independent signs.
- 256 Pell convergents test alternating signs beyond floating-point separation,
  also multiplied by 1+sqrt(3) and 1-sqrt(3) to exercise the outer sign norm.
  Those checks use 1,024-bit dyadic root enclosures.
- 336 rational-square-class cases check positive roots and exact squared values.
- Structural tests cover valid and unproved angle expressions, nonzero rational
  residuals, unsupported fields, negative roots, zero divisors/poles, excessive
  coefficient sizes, shifts and depth, cold/warm numeric caches, prior failed
  queries, descendant-before/after queries, and JSON/CBOR reconstruction.

This is finite executable qualification plus a reviewed algebraic argument,
not a machine-checked universal theorem. Complete cost/consumer qualification
and retention are outstanding. Unknown remains a legitimate outcome outside
the admitted field and resource budgets.
