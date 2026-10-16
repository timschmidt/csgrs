# Retained eighth-angle proof extension

After the built-in root-square projection, the unchanged full 16×8 sphere
example still fails strictly in `hull-proof-v1`. The captured hull points are
`(0,r,0)`, `(a,a,-r)`, `(0,-rx,-ry)`, `(a,a+ry,-rx)`, with the exact imported
radius `r`, `a=3/2`, `x=sin(pi/8)` and `y=sin(3pi/8)`. Independently expanding
their determinant gives `a*r²*(x²+y²-1)`. This is a second scalar proof gap,
not evidence against the first root-square identity.

Hyperreal owns this extension (P1–P5/P7/P11); native hulls and the full
Minkowski example consume it. Extend the existing query-only radical tower
with precisely the retained positive `sin(pi/8)` and `sin(3pi/8)` leaves.
The half-angle identities give the principal roots of `(2-sqrt(2))/4` and
`(2+sqrt(2))/4`; the arguments lie in `(0,pi/2)`, fixing root selection.
Use the existing bounded recognition of rational multiples of pi and the
existing tower arithmetic, positivity proof, generator/node/term/coefficient
limits. Preserve authoritative SinPi values and serialization, and leave
other trigonometric arguments unsupported by this proof path.

This is an exact proof-domain extension, not an optimization. The tower's
existing structural limits are not a total-work, heap-byte, or cancellation
contract; external time/address-space containment remains necessary. No new
cache is introduced. No declined proof may become a false or zero fact.

Acceptance includes raw-node half-angle and product identities, signed
rational perturbations, the independently expanded captured determinant,
invalid inverse domains and unsupported angles, repeated proof queries and
unchanged symbolic serialization. Retain the original full-resolution example
and the earlier root-square/opaque-atom regressions. Qualify the scalar and
downstream predicate/geometry suites after the final edit, and disclose costs.
