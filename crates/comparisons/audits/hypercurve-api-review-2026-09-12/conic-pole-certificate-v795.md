# Independent conic contact calculation

For affine controls (0,0), (1,1), (2,0) and weights [1,w,1], the homogeneous power polynomials are:

- X(t) = 2wt + (2-2w)t²
- Y(t) = 2wt - 2wt²
- W(t) = 1 + (2w-2)t + (2-2w)t²

Their conic identity is F(X,Y,W) = w²X² - 2w²XW + 2w²YW - (w²-1)Y² = 0.

Translate a second copy by (1,0). Substitution into the first conic gives F(X+W,Y,W) = w² W(2X-W). The W roots are projective intersections at infinity; finite contacts instead satisfy X/W=1/2 on the translated source, hence x=3/2 in world coordinates. For w=2 or w=-2, the two supporting-conic candidates have y=(4±√7)/3. The positive-weight unit trace includes the minus branch, and the negative-weight unit trace includes the plus branch. Each translated pair therefore has one finite contact.

For a translation by (0,1), substitution gives F(X,Y+W,W) = W((w²+1)W-2(w²-1)Y). For w=2 the non-pole factor is 5-14t+14t²; for w=-2 it is 5-6t+6t². Both quadratics have negative discriminant, so neither translated pair has a finite contact.

For w=-2, W=1-6t+6t² vanishes at (3±√3)/6. At either root, Y=-2/3 is nonzero, so neither homogeneous point admits an affine interpretation. The original probe published these as transverse contacts under both policies: two extra contacts in each horizontal pair and two false contacts in each vertical pair, eight in total.

For w=-1, W=(1-2t)². The horizontal non-pole factor is 4t²-1, whose only unit root is also the pole at 1/2; the vertical substitution is 2W². Both finite contact sets are empty. The original probe instead returned incomplete sets for both translations under both policies.

The runtime regression checks complete contact counts and exact algebraic coordinates, and evaluates both published parameters through the public point factories. The standalone probe has 24 independent finite evaluations at t=1/4 across its twelve cases.
