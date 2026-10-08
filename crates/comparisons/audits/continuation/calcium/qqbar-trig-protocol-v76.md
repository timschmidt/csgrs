# Roots of unity / forward rational-angle qualification — checkpoint 76

Read all eight implementation files and their eight tests independently in both
pinned qqbar trees (32 files, 1,964 new lines). Re-read the documented word-boundary
restriction; do not classify excluded inputs as donor correctness failures.
Compare selected live Hyper reduction/dispatch ranges, without claiming complete
Hyper source coverage or replacing its computable-real representation.

Use a small in-contract deterministic corpus: denominators 1,2,3,4,6,12;
numerators -2q through 2q; unreduced scale factors 1 and 3; root-of-unity, exp(pi*i*x),
sin/cos/tan/cot/sec/csc. Observe initial and 256-bit cached states. Check poles
directly (not only when an approximation is finite), full minimal polynomials,
both complex components' exact dyadic enclosures, and root-of-unity recognition.
Include five non-root recognition controls in both states. This is 3,776 primary
records plus ten controls and one terminal record. Failed-call output values
have no documented contract and must not be compared.

The independent oracle uses Q(sqrt(2),sqrt(3)) field arithmetic and exact sign
comparisons, not FLINT/Arb, approximate agreement, or upstream self-identities.
Sine/cosine use independently authored pi/12 radicals and quadrant identities.
Distinct Galois images give real minimal polynomials. Root-of-unity polynomials
come independently from exact division of X^n-1 by lower cyclotomic factors.
Check complete record membership, 128-bit output containment and width <=2^-96.
Exercise corrupted records/streams to test rejection. No out-of-contract
machine-limit inputs, archived execution, arbitrary denominator closure or speed claim.

Freeze source/library/configuration identities before compiling; reuse existing
FLINT/GMP/MPFR libraries. Run a warning-clean native collector and the same bounded
corpus under Memcheck, preserving complete outputs. Use one small dedicated binary,
not a library rebuild or broad source copy. Compare Hyper's actual corresponding
capabilities before deciding whether a new candidate is justified. The complete
ecosystem inventory and other open transfers remain in scope.
