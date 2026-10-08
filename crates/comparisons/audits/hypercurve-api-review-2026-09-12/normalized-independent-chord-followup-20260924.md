# Independent endpoint corner oracle

Implemented and fully qualified in Hypercurve 91ddad8. Its V5 frozen
regression passes all four policy/reversal combinations, and all 121 public
integration cases pass. The V1–V4 failure and diagnosis, and the exact radial replay fix, are
recorded in selected-circle-containment-20260924-proof.md.

The fixture independent_field_algebraic_chord_region is the exact triangle
O=(0,0), A=(sqrt(1/2),0), B=(0,sqrt(1/3)), retaining A and B in independent
fields. Its corner test always edits vertex index one: A in forward order,
B in reverse order. Preserve that coverage, both policies and candidate counts.

Set s=sqrt(5/6). At the edited vertex V let e be the increasing axis unit
vector, and d the outgoing unit vector toward the other nonzero vertex.
The two exterior setback-1/10 contacts are V+(1/10)e and V-(1/10)d.
Find one candidate with both contacts across all normalized boundaries.
Its additional small triangular lobe contains V+(e-d)/30. Every candidate
keeps (1/8,1/8) inside and (2,2) outside, with early portions of both axis
edges still on the boundary. Those are exact geometric checks; a count of
AlgebraicChord payloads on loop zero is not.

For radius r=1/100, write a for the edited axis length and b for the other
axis length. The corresponding exterior tangency distance is L=r*(s+a)/b;
contacts are V+L*e and V-L*d. The circle center is V+r*s/b*(e-d).
A point one quarter of the way from V to that center lies in the small
exterior wedge before the nearest arc, for either connecting circular arc.
Use independently derived contacts and membership plus boundary ownership,
not concrete chord/circle variants. The regression finds both contacts in one
candidate and verifies the expected exact set before accepting that branch.

The directed exterior fillet uses the major arc: in forward traversal its
center is below the horizontal edge, so the arriving positive-x tangent
requires clockwise traversal from the upper radial point to the diagonal
contact. This arc includes the extreme point center+r*e. That point is outside
both the old triangle and the short wedge and must remain Boundary. Under
reversal, exchange coordinate axes and the same formula proves the extreme
point on the other edited leg. This gives a circle-specific geometric oracle
without inspecting the carrier enum. The chamfer's contact midpoint similarly
provides a straight-connector boundary oracle. The quarter-center interior
sample is before the nearest circle point for both a=sqrt(1/2),b=sqrt(1/3)
and their exchanged values: |center-V|>4r/3 in both cases.
