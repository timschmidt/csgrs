# Direct interval Horner replay (read-only candidate, not implemented)

The retained local polynomial authority currently evaluates defining signs at
rational bisection points by allocating a complete recursive field value for
every Horner multiply/add, then asks that value for its sign. The V25 late stack
lands in these dense coefficient scales. V28 first normalizes the defining row
once; measure this before extending the change.

If coefficient scaling remains dominant, the existing
recursive_quadratic_polynomial_interval helper can evaluate the same polynomial
at a point interval and certify a nonzero sign without allocating that field
value. Such a filter must decline zero-spanning/unknown intervals and retain the
exact field fallback for zero or tightly separated roots. It must preserve the
selected source tuple and positive radical sheets, and should prepare source
bounds once per polynomial rather than independently per coefficient. No cache
or broader interval API is justified until the measured remaining path needs it.

A filter is useful only if its bounded overhead is low on small roots and exact
endpoint equalities. Existing wide-gauge, near-zero/nonrational, proper-factor,
repeated-root and chart-projection tests must still pass. Source edits are frozen
while the V28 qualification is live.
