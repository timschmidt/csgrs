# First candidate findings

The frozen candidate compiled and ran all eight tests: three passed, five
failed. Preserve `candidate-v1` and its artifacts unchanged.

Native authoritative clipping passed the algebraic endpoint/contact replay
case, original full grid coverage and approximate-terminal receipt checks.
The linear fixtures reached the expected authoritative directed intervals, but
the geometry-only adapter rejected `Curve2` range wrappers whose endpoints
were already represented `Real` values. The adapter must use those exact
endpoints through `LineSeg2::try_new`, preserving typed construction failure.
Every source here is a generated line; no line recognition or approximation
algorithm belongs in CSGRS. Nonrepresented selected endpoints remain in the
authoritative native records and produce Unsupported only in the string view.

The symbolic affine fixture's expected and actual expressions had different
structure. Its first test incorrectly used structural `PartialEq` to demand
value equality. Use certified scalar equality for expected coordinates,
connectivity and collinearity. Preserve the exact fixture, strict policy and
directed interval expectations; an Unknown comparison still fails the test.
This corrects the equality criterion required by the original contract.
