# Shared-factor polynomial baseline

Target revision: `045621eba`. The previous goal turn made progress: the
polynomial quotient candidate passed focused scalar qualification and restored
the original full 16-segment chain. The remaining original 32-segment
subtraction still failed with typed predicate uncertainty. Every original
workflow assertion and resolution is retained.

The fresh diagnosis and independent algebra are in
`../2026-10-15-polynomial-quotient-reduction/diagnostic-v3` and
`../2026-10-15-polynomial-quotient-reduction/remaining-algebra-v1.json`.
The current coplanar clipping predicate is a degree-eight polynomial over four
opaque sine atoms, with no inverse operands. Its two large opposite products
each have 145 terms, exceeding the existing 64-term limit. They share factors;
the remaining sum is identically zero. No transcendental identity is needed.

The invariant belongs in Hyperreal. The relevant requirements are P2–P5,
P7–P9, P20, P24–P27, P35–P37 and P39: exact scalar proofs, honest uncertainty,
resource limits, strict composition and independently justified acceptance.
A proposed recovery must preserve the same remaining recognition budget and
every operand's domain, and must not raise the degree or term limits. It may
prove a zero residual before expanding a common product. Common factors need
not be nonzero, but they must be valid operands. A nonzero residual does not
justify certifying the original expression.

The new private regression preserves the captured algebraic construction with
four independent opaque sine inputs. It tests the zero identity, signed exact
displacements, invalid and unproved inverse factors, degree-nine rejection,
work exhaustion, cancellation and recovery. `baseline-v1` compiles with
warnings denied and passes formatting, but its launch command mistakenly
names `unit` instead of the generated `units` executable. Its exit 127 is
retained. The fresh `baseline-v2` pins and executes that unchanged binary:
the domain/degree control passes; the exact identity and post-cancellation
recovery fail at the expected `None` versus exact zero assertion.

The user confirmed that another terminal had been clearing the earlier `/tmp`
artifacts and then confirmed `/tmp` is available for subsequent validation.
New builds therefore use fresh `/tmp/csgrs-factored-product-proofs` targets.
The earlier 27,093 missing recorded paths, including diagnostic source copies,
remain explicitly recorded in the previous compressed artifact audit. Storage
availability does not restore those artifacts or establish historical
preservation. Frozen targets are not resumed or overwritten.
