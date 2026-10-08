# Explicit finite parallel regularity

Follow-up to the analytic self-incidence migration. The implementation goal stays active.

Replace the implicit-unit `BezierParallel2::singularity_analysis(policy)` contract with one required `CurveParameterRange2` argument. All authored full-span callers pass the unit range explicitly; retained offset composition and tangent certification pass their actual range. Use the existing finite-domain root authority, preserving the original polynomial and endpoint identities. Extend the exact quadratic cusp formula with range membership rather than hiding an unbounded result behind a unit filter. The report should retain its certified range. Add the minimal public exact-range constructor needed to call this API, without an alias or compatibility method.

Validation should distinguish source singularities, poles, and true normal-sheet cusps on exterior, reversed, proper-unit, and selected-endpoint ranges. A pole outside the requested range must not block it. Roots on closed endpoints must remain available; opposite-sheet squared roots must remain excluded. Compare equivalent native and shifted charts using independently known equations.

Do not relax analytic fragment admission until range-sensitive pruning has the proper premise. The source-axis helpers currently assume the native control polygon. Cross-fragment point distinctness additionally needs injectivity on the interval joining both contact parameters; individual fragment regularity does not certify a union that crosses cusps. The regularity API change supplies the missing reusable proof for this next stage. Affine analytic first derivatives, finite pair incidence, the public constructor baseline, and general normalized region construction remain open.
