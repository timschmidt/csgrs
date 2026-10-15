# Zero quotient composition baseline

Production revision: `7cf9d82acee6f152047eb37e5ff2e6dcece51a0d`.
The added tests change no policy, fixture, proof limit, or implementation.

`diagnostic-v4` confirms that the production quotient recognizer declines at
degree-9 products for both 16/32 empty unions and at degree-9/10 products for
the respective direct subtractions. Node and work budgets remain available.
Instrumentation uses source copies and returns the original predicate outcome;
its additional diagnostic query publishes no root proof. The commands succeed
as observations, while the 16/32 geometry compositions remain undecided.

All diagnostic failures are retained: v1 exhausted temporary storage; v2/v3
could not resolve the rebuilt transitive crate graph because the relocated
command retained a stale dependency search directory. V4 corrects that path
and reuses the frozen successful v2 builds and v3 external-library view.

The independent conditional algebra experiment `zero-analysis-v1` reduces the
empty-union captures' largest product degree from 9 to 4 by discarding a
denominator after its numerator proves zero. Direct-subtraction captures still
reach degree 10. This experiment assumes nonzero inverse operands; it is not
a production domain proof or a completed-workflow claim.

`scalar-zero-baseline-v1` passes workspace formatting and compiles both test
binaries with warnings denied. Its complete quotient unit group reports
8 passed / 2 failed; the public quotient tests report 6 passed / 1 failed.
The missing identities are `(1/d - 1/d) / d^8 = 0`, zero addition followed by
cancellation, `1/d^4 - d^4/d^8 = 0`, and composition of independently expanded
equal public quotients with a degree-8 reciprocal. Here `d = 4 + sin(1/3)`
has a retained strict nonzero proof. The public positive divisor uses
`4 + sin(e)`. Existing budgets and the original complete drilled workflows
are unchanged. Zero absorption must continue to reject a zero or unproved
divisor, including when it occurs in the other operand.

Hyperreal owns this invariant (P2–P5/P7); Hypermesh and CSGRS consume its
certificates. The next change should normalize proved zero quotients without
expanding an irrelevant denominator, after validating both child expressions.
It must preserve the authoritative expression and must not bypass inverse
guards, cancellation, or proof budgets. Full downstream admission and the
separate subtraction growth issue remain open.
