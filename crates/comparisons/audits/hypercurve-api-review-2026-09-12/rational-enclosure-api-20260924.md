# Rational and dyadic enclosure contracts

V16's independent tensor enclosure regression exposed an existing Hyperreal
contract mismatch: `certified_dyadic_interval` returned an exact point interval
for any rational value and multiplied computable bounds by an arbitrary
rational scale. Both cases can retain nondyadic denominators. These bounds were
conservative, but their name did not guarantee the representation required to
control denominator growth in tensor Horner evaluation.

V17 names the existing behavior `certified_rational_interval`. Its arithmetic,
abort handling, computable precision schedule and exact rational points are
unchanged. Existing active callers migrate directly, including examples,
benchmarks and fuzz targets. Historical source snapshots and benchmark receipts
remain immutable. The mechanical-caller receipt verifies 33 files against the
parent text with only the exact identifier replacement and rustfmt. Hypercurve's
two active implementation files and the changed scalar contract/tests receive
separate review and runtime qualification.

`certified_dyadic_interval` now has the stronger grid contract. For a rational
bound x = s n/d, d > 0, and grid g = 2^p, divide n/g by d when p < 0, or n by d g
when p >= 0. Unsigned quotient and remainder give q and q+1 (q when exact).
Restore the dyadic grid and reverse the two signed endpoints for s < 0. Thus
the resulting interval is exactly [g floor(x/g), g ceil(x/g)]. This proof does
not require reduced input storage. Exact zero has the point enclosure [0,0].

For a general Real, first obtain its certified rational bounds [l,u]. Round l
down and u up on the grid. Containment follows by transitivity. The rational
scale still controls the initial computable enclosure width; the grid spacing
is not presented as a width bound. Aborted evaluation still provides no
certificate. Neither method changes exact scalar meaning or retained evidence.

The private Rational rounding operation is shared by Real's grid enclosure.
There is no compatibility alias: the two public Real queries have distinct
representation guarantees. Hypercurve's coefficient-precision tensor filter
uses the dyadic contract for coefficients and active source axes. Whole-value
exact witness replay remains available, and inactive axes require no bound
refinement.

Tests compare signed, reduced and unreduced rational bounds to independent
BigRational floor/ceiling arithmetic, including coarse grids and 1025-bit common
factors. Real tests cover exact rational points and rationally scaled square
roots and pi. The geometric regression independently checks polynomial
containment for signed rational/surd axes and a nontrivial source interval.

V17 qualification is in progress. The unchanged quartic chamfer 75-second gate
and the complete curve regression gate remain open. Review follow-up: the
stack-exactness audit script's search expression should recognize both enclosure
methods now that they have distinct contracts; change it after the current
owned source freeze is released.

V17 completes successfully; exact outer handle 23219 reaped. Both feature configurations pass formatter/Clippy, the three focused arithmetic cases and the all-feature public API coverage and scalar representation targets (13 test executions total). All six downstream library caller builds and all six changed fuzz-target builds pass. V18 updates the exactness audit search to cover both enclosure methods (shell syntax and both pattern arms checked) and reuses a reduced rational payload already on the requested nonpositive dyadic grid instead of shifting/dividing it again. The existing independent grid oracle covers this branch. The initial V18 copy failed with tmpfs EDQUOT before a build began and was removed. Completed V14–V17 snapshots were relocated with every file hash checked and original-path aliases retained; relocation outer handle 84901 reaped, receipt source-archive-relocation-20260924-4.json. The expanded V18 physical snapshot binds 2844 source/configuration inputs, excluding 666 tracked decoder build artifacts under an unrelated Alumina tool target directory; all 2044 curve inputs remain bound separately. The complete scalar/caller validation will be repeated before any commit.
