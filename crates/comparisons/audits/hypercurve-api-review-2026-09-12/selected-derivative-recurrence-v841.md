# Exact selected-root derivative recurrence

Let the source have homogeneous polynomials X,Y,D, and let alpha be the selected root of P. Work first with one coordinate C=N/D at any alpha for which D(alpha) is certified nonzero. Differentiating N=D*C k times gives N_k=sum_{j=0}^k binomial(k,j) D_j C_(k-j), where subscripts denote derivatives of the original polynomials/functions. Set C_k(alpha)=A_k(alpha)/D(alpha)^(k+1), A_0=N. Multiplication by D^k gives

    A_k = N_k D^k - sum_{j=1}^k binomial(k,j) D_j D^(j-1) A_(k-j).

Only N_k and D_j are differentiated from the original source. A_k, D_j D^(j-1), and powers of D are values at alpha and may be replaced by certified remainders modulo P after each step. They are never differentiated. Thus their polynomial sizes remain below deg(P) when remainder reduction is certified; original source derivative vectors decrease in size. Uncertain reduction retains the exact unreduced expression, as before.

The algorithm uses no inverse of D modulo P. It therefore supports a reducible P sharing a factor with D as long as the selected alpha is finite. The published denominator remains the exact value D^(k+1), and the existing rational image factory retains the original finite-domain requirement; it cannot erase a source pole by canceling a common homogeneous factor. Existing field/root publication, lazy materialization and transformed-only parameter caches are unchanged.

The denominator derivative list ends only above its original certified degree. A contribution is skipped only if all coefficients of its selected value are definitely zero. Rational derivatives are not truncated at the source degree. Binomial coefficients use the existing checked-u64/exact-Real routine rather than a new bounded integer limit. A zero-order request still returns immediately.

Independent checks: the common-factor line quotient test compares eight derivatives to (-1)^k*k!/(1+t)^(k+1) at sqrt(1/2), including a reducible source P with a different root excluded as a pole. The new sparse rational test compares all orders through80 to Taylor coefficients of (t/(1+t^n),1/(1+t^n)) at0; its n40/order80 case needs binomial(80,40)>u64::MAX and nonzero derivatives beyond the source degree. Both policies are covered. Existing nonrational coefficient/source tests and downstream topology/fillet/offset cases remain in the qualification inventory.

This is a computational recurrence change; it is not evidence of universal closure for every curve operation or uniform performance improvement. Unchanged-source allocation/timing probes must be compared against the immediately preceding normal-release baseline.

Source review also checked exhausted numerator derivatives: certified empty/zero remainders use the existing canonical zero polynomial; denominator powers are never exhausted. For k<=degree(D), the stored denominator derivative at index j-1 is D_j*D^(j-1); the numerator history at index k-j is A_(k-j). The denominator loop state starts (D^0,D^1) and advances to (D^k,D^(k+1)) before publication, matching the recurrence for every requested order. The exact scalar derivative path changes only helper visibility; native one- and two-derivative specialized formulas remain unchanged.
