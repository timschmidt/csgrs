# Scalar proof required by the full sphere fixture

`convexity-baseline-v1/v2` reproduce this on the pinned pre-edit libraries:
the original cube(1.5)+sphere(0.4,16,8) example already returns a strict hull
predicate error. The sphere itself has certified closed-PWN topology, but native
convexity has 64 undecided point-plane queries. The first captured four points
and complete scalar expression are retained in the v2 target.

Ignoring the common exact radius, those points are
`(xy,y,x²)`, `(qx/2,y,qx/2)`, `(1/2,q/2,1/2)`, `(qy/2,q/2,qx/2)`, where
`x=sin(pi/8)`, `y=sin(3pi/8)` and `q=sqrt(2)`. Their oriented plane expression
factors as `-x(-q+2y)(q²-2)(x-y)/8`. Thus the missing identity is scalar:
`sqrt(2)²=2` in a polynomial with otherwise opaque real atoms. No trigonometric
identity, tolerance, mesh-specific formula or stronger refinement policy is
needed. The analogous built-in `sqrt(3)²=3` relation belongs to the same domain.

Hyperreal owns the fix (P1–P5/P7), with Hyperlimit/native convexity/hull and the
full unchanged Minkowski example as downstream consumers. Preserve the existing
successful Laurent, polynomial and quotient paths. Only after those decline,
try the same domain-checked quotient machinery with the two built-in constant
square relations. Keep node/depth/atom/term/degree/coefficient limits, charge
additional reductions, check cancellation and publish only a completed rational
proof. No negative result may block a later stronger proof. This does not add
arbitrary root assumptions or an unbounded algebraic-number solver.

Acceptance needs independently expanded mixed-root/opaque-atom identities,
positive and negative rational perturbations, an unrelated atom whose square
must remain unknown, invalid/unknown inverse domains, small budgets and recovery,
cancellation, and the original full sphere construction. Check existing scalar
and downstream representation matrices; do not turn the full example into an
accepted Unknown or reduce its sampling. This is a correctness/completeness
extension, with additional fallback cost reported rather than a speedup claim.
