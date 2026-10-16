# Reproduced construction failures

`reproduction-v1` ran against retained libraries from `67a3daae8`. All three
commands passed; the diagnostic deliberately prints observed errors and wrong
successes. Its zero exit is not an acceptance claim for those cases.

The CCW L ring `(3,1),(1,1),(1,3),(0,3),(0,0),(3,0)` has area 5. Its first
non-collinear triple has the opposite orientation. A single polyhedron face
returns signed area -5. Lofting to z=2 returns a boundary with 12 unbalanced
edges and signed volume -10/3 instead of 10. Across all twelve cyclic/reversed
orders, two lofts and four authored polyhedron prisms fail native PWN validation.
Empty union rejects these outputs. Membership alone fails to detect the defect.

Zero-area and nonzero-area crossed rings, and a nonadjacent repeated vertex,
publish successful incomplete or ambiguous faces. The nonzero crossing's
triangles have signed area 5 against boundary shoelace area 1. The repeated-point
loft even passes PWN validation: topology alone cannot establish face fidelity.
Repeated square sections at the beginning, middle and end produce degenerate
triangles. Unresolved support-triple predicates lose their native cause in
`ValidationError::Geometry` rather than retaining an undecided error.

Independent open follow-up: Minkowski of two separated unit boxes with a unit
box fills the known gap, returning volume 24 rather than 16. An unused input
point also changes the result. Empty paired with malformed indices hides the
invalid operand. Degenerate and undecided hull causes become strings. This is
a vertex-sum convex-hull implementation, not a general represented-set sum.
It needs a separate native capability/migration; the face/loft repair does not
resolve it or establish that the whole properties plan is complete.

Original tetrahedron (volume 4/3), square loft (volume 896/3), and cube sum
(volume 125) all succeed strictly and pass PWN validation. Their full ordered
coordinate/index artifacts are retained under the reproduction target. They
remain acceptance controls, alongside the original benchmark and render inputs.
