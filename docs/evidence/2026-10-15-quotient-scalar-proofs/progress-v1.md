# Guarded quotient proof progress

Baseline `a9e3fbb29` preserves three failing rational-projection regressions and
the passing `UnknownZero` denominator control. The candidate adds a separate
quotient attempt after the existing Laurent and polynomial proofs. It shares
polynomial arithmetic, coefficient admission and structural atom equality.

Every expanded inverse needs an exact nonzero witness: a retained immediate
strict sign, or a nonzero rational projection proved by the same attempt.
An unknown or zero denominator cannot enter fraction reduction. Addition and
multiplication preserve the nonzero-denominator invariant; proportional
numerator and denominator polynomials prove a rational result. No independence
of transcendental atoms is assumed. Only the completed root sign is published;
the authoritative expression is unchanged and declines are not cached.

This attempt separately permits 512 node visits, depth 64, 32 atoms,
64 monomials, degree 8, 4096 stored coefficient bits and 8192 work charges.
Temporary rational arithmetic can use twice the coefficient bits before a
check, plus a carry for addition. These limits do not bound the existing opaque
DAG matcher or whole-query time/heap use. Tests and diagnostics use external
8 GiB per-process address-space and 1800-second per-command wall containment.

The final-source [scalar-and-drill-v1 trial](scalar-and-drill-v1/result.json)
passes all 15 commands: workspace formatting; 906 default and 1013 all-feature
scalar tests; all-feature/all-target scalar Clippy with warnings denied; the
CSGRS curve library build; eight original drill observations; and 28 scalar
doctests with warnings denied. Counts overlap across configurations. The earlier
focused candidate passed all five public tests then present; the final trial
also covers different common denominators and unchanged serialization.

Controls establish exact positive/negative `1/10^12` offsets, small nonzero
divisors, typed unproved-denominator failure, zero/unknown inverse rejection,
local nonzero proofs without refinement, genuine proof warming after a decline,
precision changes, concurrent queries, cancellation, separate limits and
subsequent recovery. Existing Laurent and polynomial successful tests pass.

The unchanged full drilling matrix preserves certified 8/24-segment results
and qualified approximate 16/32-segment results. Strict 16/32 cases advance
past coordinate equality and now fail with typed `PredicateUndecided("surface
arrangement retained line orientation")`. A successful probe exit records that
observation; it does not certify completion of the Boolean operation.

The native diagnostic instruments a source copy, pins the exact Cargo dependency
graph and preserves the same outcomes. [Independent rational-function expansion](orientation-algebra-v1.json)
finds zero numerators in both captured determinants, conditional on their four
nonzero-denominator obligations. Each has 648 arithmetic visits and three opaque
atoms, exceeding the production attempt's 512-node limit. This diagnostic is
not a production certificate. Investigate query-local reuse of completed
subexpression proofs before increasing any limit; reproduce the missing reuse
test first and retain the original full fixtures.

This is a tested candidate, not final admission. Broad downstream qualification,
matched completed-contract cost evidence, strict full-drill completion,
independent semantic/topology/composition acceptance and release/user-path
validation remain open. The 35 pending native files are unchanged and remain
unadmitted. No push was performed; the properties plan remains active.
