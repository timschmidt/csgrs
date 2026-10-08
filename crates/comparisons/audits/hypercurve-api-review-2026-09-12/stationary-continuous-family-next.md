# Next independent continuous-family regression

After V536 and a verified commit, test the existing rational joined-offset counterexample after the stationary reparameterization t=u². Do not edit frozen production or live driver inputs for this fixture.

Base P(u)=(3u²/8,9u⁴/64), degree-four rational Bezier with all weights1 and controls (0,0),(0,0),(1/16,0),(3/16,0),(3/8,9/64). The original offsets are41/64 and5/8. Reverse both finite unit fragments; they still join at(0,41/64). Construct retained endpoint images independently: at0 the one-sided point is(0,d); at1 it is(3/8−3d/5,9/64+4d/5). Avoid raw speed-zero point evaluation when creating the fixture.

Radius1/128 has the same continuous exact center family as the committed quadratic counterexample. Select u=sqrt(5)/3 (so t=5/9), center(-175/4992,4699/7488), contacts(-95/2496,4753/7488) and(-5/156,4645/7488). Check radius-only returns FilletConstraintRequired, center-only/contact-point/contact-parameter selects one, both axes, reversed path and both exact policies, independently fixed circle radius and contact images. This crosses source endpoint stationarity, original-offset cusp partitions, arbitrary exact parameters, and the additional exact constraint API. A retained center/contact witness should be replayed if feasible without Cartesian reconstruction. It is an unrun design, not a passing claim.

Other open cases: generic non-Cartesian point incidence at stationary parameters still falls back to raw point visitor; normal constraints on an incident ray cannot be discharged from the finite cell certificate; original offset cusps on rays still need their own one-sided domain ownership; selected-center component replay cost needs broad timing checks. Full implementation goal remains active.
