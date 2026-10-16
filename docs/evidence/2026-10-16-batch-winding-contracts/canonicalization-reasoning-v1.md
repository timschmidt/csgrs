# Why the canonical edge selection is independent of the starting triangulation

This is the reasoning for the Hypertri implementation in this change, not a
formal-verification result or an additional acceptance policy. The normative
requirements remain `properties.md` and `docs/admission.md`.

The checked topology entry point supplies unique ordered points, already split
noncrossing constraints, and a complete positively oriented triangulation of
their convex hull. Every input vertex participates. The existing constraint
recovery path preserves those vertices and any protected edges. Its postconditions
remain checked by the existing convex-hull topology validator.

The new pass begins with authored constraints and the subdivided hull edges
protected. It visits every possible endpoint pair in lexicographic order. An
edge already present in the current valid triangulation has an incidence proof:
it cannot cross a protected edge or contain another input vertex in its interior.
For an absent edge, exact predicates check those two conditions before the same
native constraint-recovery path inserts it. The optional rational bounding-box
filters only reject provably disjoint candidates; authoritative coordinates
remain unchanged.

These checks also exclude overlapping protected segments and a candidate passing
through a protected endpoint. A candidate endpoint cannot lie inside a protected
segment: initial constraints already exclude interior input vertices, and every
new protected edge has the same obligation. Thus accepting a candidate preserves
a planar straight-line graph without adding coordinates or discarding contacts.

Every accepted edge is immediately protected. Consequently the decision for a
pair depends only on the points, original constraints and earlier accepted pairs.
It cannot depend on the starting triangulation. The decreasing-flip pass remains
a preparation step: accepting an already present edge uses a valid incidence
proof, rather than making a different geometric choice. Constraint recovery is
needed only when that prepared triangulation omitted an admissible earlier edge.

A complete triangulation of the fixed vertices and fixed subdivided hull has a
fixed edge count. Protected edges are always a subset of the current
triangulation. Once their counts agree, every edge is protected; no further
noncrossing diagonal can be added. Conversely, after all pairs are considered,
an unprotected current edge would be impossible. It was either accepted earlier,
or rejected because it crossed an already protected edge or contained a fixed
input vertex. Both rejection witnesses persist, contradicting its presence in
the current valid triangulation. This supplies finite termination and uniqueness
of the selected edge set, assuming the checked predicate and recovery contracts.
It does not promise an identical triangle storage order across different source
faces.

For an identical protected cell with the same point set, internal constraints
and relative vertex order, an edge from outside cannot cross its protected
boundary. Exterior choices therefore cannot obstruct an interior candidate.
Applying the same ordered selection inside both copies gives the same geometric
triangle set. Hypermesh can then bundle coincident source sheets by their shared
vertex triples without changing its winding transitions or relaxing a consistency
check.

The retained five-point counterexample has P=(120,60), T=(120,30), Q=(90,30),
R=(120,40), S=(102,48), ordered P,T,Q,R,S, with boundary P-R-T-Q-S-P.
The S-centered triangulation is a local minimum for the old single-flip rule:
R-S cannot flip to P-T through R, while T-S to Q-R increases its endpoint pair.
The Q-centered triangulation is another local minimum. The new pass admits P-Q
and recovers its whole corridor, preserving the mandatory collinear vertex R.
The three nondegenerate combinatorial triangulations form an independent finite
oracle. Native tests enumerate their lexicographic minimum under every vertex
order, shear/translation, reflection/scaling, and both predicate policies. The
three larger captured face inputs test independence from surrounding constraints.
The original 26-box workload remains a separate required regression.

This argument trusts the existing exact predicate implementations and checked
constraint recovery; it is not a machine-checked connection to executable code.
An undecided predicate remains a typed error. Consuming approximate evidence
remains qualified by the evaluator receipt; selecting an approximate policy does
not itself establish or destroy certification. No claim is made that every
opaque Real expression is decidable.

At most n(n-1)/2 pairs are considered and protection grows monotonically. These
finite combinatorial bounds do not bound scalar coefficient growth, total CPU
work or heap bytes. The pass retains edge/topology workspaces and reuses native
recovery. Complete internal work, retained-memory and cancellation contracts
remain open; the diagnostic and qualification commands use the separately
recorded external containment. Resource comparisons must report the same
completed controls, all samples, output artifacts and any increased costs.
