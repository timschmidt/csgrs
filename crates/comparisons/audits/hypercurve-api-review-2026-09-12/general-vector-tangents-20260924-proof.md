# General retained vector-tangent parameters

Parent Hypercurve: 8de08b874db388e25b2dae69cf93622cded79006.
All prior owned processes are reaped. V1 migrates the four existing vector/source
tangent predicate methods from BezierParameter2 to CurveParameter2. Their two
linear forms use the existing common polynomial-sign authority; derivative
orientation already consumes that parameter. All callers are updated directly.
No alias, shim, new representation, or cache is added. The regular-range method
retains its cancelled one-sided tangent and certified interior orientation.

The regression uses alpha^65=1/2 and 4 beta^2=alpha, 0<beta<1/2. For P(t)=(t,t^2),
the forward tangent is (1,2 beta), and the reversed source tangent at beta is
(-1,-2(1-beta)). Rational interval inequalities determine the vector cross/dot
and linear-combination signs independently. Signed offset distances -10,0,10
cover positive and negative parallel derivative scale: on t in (0,1), speed
squared is at most 5, so 20^2 exceeds speed^6 and distance +10 reverses it.
The source-only linear form intentionally omits that parallel scale. Both
selected-fiber and recursive projective parameters are exercised; a cold query
runs before optional projective import and no projected Bezier cache is allowed.
A separate cubic P(t)=(t^3,0) checks selected stationary endpoints through the
regularized tangent in both traversal directions and policy modes.

Both Clippy configurations and all eight focused cases pass. The broader
selection contains 991 cases, including all 121 public integration cases.
The companion fillet, chord/selected-circle fillet and recursive chamfer pass
in isolation within their existing deadlines. After those cases, the initial
driver exited because its list named `concurrent` shadowed Python's imported
module, before creating the worker pool. Session 58957 was reaped with exit 1.
The separate resume driver consumes the same source and executable hashes,
reuses those eleven completed receipts (eight focused, three isolated), and
runs the remaining 980 cases with two workers. Source and executable bytes
are unchanged. Broad qualification remains pending.

Qualification is complete and committed as
c13778eb15662d3b12093a54e518e9fb137c39d1. A final selection audit adds twelve
corner/endpoint-image cases omitted by the broad name filter, using the same
frozen executable. Across 1,003 attempted cases, 987 pass, six are ignored,
and the same ten prior nonpasses remain (eight bounded timeouts and two
obsolete layout assertions). All 121 public integration cases pass. Both
Clippy configurations and formatting pass. The qualification, staged and
post-commit records bind all 2,044 inputs and the committed bytes. All owned
processes are reaped, all thirty repositories are clean, and nothing was pushed.

Native bivariate tangent pairs and duplicate CurveTangent2 parameter storage
are separate follow-ups; this candidate does not claim to have removed their
remaining restrictions.
