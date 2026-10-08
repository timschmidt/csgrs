# Reusable recursive point incidence (V578, design only)

No production implementation or additional diagnostic run. V577 owns the live
sources. V575 will first test the suspected point-representation gap.

Available machinery already imports every CurvePointData2 alternative through
`recursive_projective_point_source` / `recursive_projective_evidence_points`.
The resulting point retains correlated projective coordinates X/D, Y/D in one
recursive coefficient field. `positive_recursive_projective_point` certifies
and normalizes D>0. This avoids forcing two standalone coordinate images or
manufacturing synthetic chords solely to ask point membership.

For a source N/W, oriented source tangent T=(Tx,Ty), squared speed
S=Tx²+Ty² and signed distance d, retain the two equations

    Ax = D*Nx - X*W       Bx = -d*D*W*Ty
    Ay = D*Ny - Y*W       By =  d*D*W*Tx
    Ax*sqrt(S) + Bx = 0   Ay*sqrt(S) + By = 0.

Coefficients belong to the point's existing field; the source polynomials are
exact Real coefficients embedded there. A zero distance uses Ax and Ay directly
and does not require a normal. Otherwise the actual finite/incident source
domain must prove W nonzero and the selected S positive, including the owned
primitive frame at stationary boundaries.

For nonzero distance, enumerate one nonzero squared polynomial A²*S-B².
The existing recursive polynomial root solver retains root authority, including
linear/quadratic construction evidence and local ordered-field isolation.
Each published root is already a root of its authoritative equation; avoid
discarding that identity by redundantly reconstructing its zero. Replay the
unsquared branch using opposite A/B signs (or both zero). Check the other
coordinate's squared equation and its unsquared branch at the same root.
Source and point denominator signs do not change the zero-incidence test.

Important cases:

- One identically-zero squared coordinate equation needs the other coordinate
  as its enumerator; it is not evidence that the point is on the parallel.
- If both squared equations are identities, summing them proves the original
  source lies on the radius-|d| circle about the query point, once W and S are
  certified nonzero. On a connected regular source cell its oriented normal
  sheet cannot change. An exact interior branch test can then distinguish an
  entirely collapsed parallel from the opposite sheet. A sample alone, without
  these polynomial identities and connected-sheet proof, is insufficient.
- An all-parameters incidence should retain the existing Any/visitor(None)
  meaning. It must not choose a representative contact for an underconstrained
  fillet family.
- Closed finite ownership, algebraic endpoint-to-anchor bridges and the open
  incident ray remain separate admission evidence. Reuse the incident owner's
  membership and first-barrier checks. Do not infer a ray's normal from an
  arbitrary finite midpoint.
- `recursive_projective_polynomial_parameters` currently accepts a finite
  range. Its existing chord/parallel counterpart handles finite/incident
  domains; inspect and share the root-enumeration portion before creating
  another compactification or global projection implementation.
- Winding needs ordered parameters and endpoint ownership beyond membership.
  Do not replace its proven schedule merely because the membership equations
  can share a field. Qualify any eventual common replacement independently.

The first V575 fixture uses an independent procedural displacement with known
Cartesian meaning. Passing it alone would not establish coverage for every
deep selected field. A repair also needs retained non-Cartesian authority,
opposite-sheet, overlap/constant-coordinate, pole, stationary-limit and repeat
operation coverage, using existing regressions where they already apply.
