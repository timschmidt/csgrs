# Common-cell triangulation refinement of the acceptance contract

The completed `diagnostic-v2` phase (6/6, unchanged sources) captures the exact
projected points, protected edges and output triangles supplied by three source
faces. `extract_cells_v1.py` checks that their common five-vertex cell has the
same boundary and vertex order, then retains the inputs and outputs in
`fixtures/common-cell-v1.json`. Its x=3 projection is scaled by 30 to give exact
integer test coordinates. The original 26-box and deletion-minimal five-box
failures remain required; these smaller planar cases supplement them.

Hypertri owns this newly localized invariant: canonical topology-only
triangulation must give the same triangles within an identical protected cell
when the relative order of its vertices agrees. Hypermesh relies on that property
to bundle coincident source sheets before radial topology. The public Hypertri
entry point and both its scan/retained-adjacency paths are affected. This extends
the owning-layer identification in `contract-v1.md`; all its properties, consumers,
preservation requirements and limitations still apply.

For P=(120,60), T=(120,30), Q=(90,30), R=(120,40), S=(102,48), with the
vertex order P,T,Q,R,S, the protected boundary is P-R-T-Q-S-P. Two valid
triangulations are stable under decreasing single-diagonal flips. In the
S-centered triangulation, flipping R-S would create the inadmissible edge P-T
through R; flipping T-S to Q-R temporarily increases the endpoint pair.
Consequently local descent is not a canonicalization proof. The earlier incircle
calculation is irrelevant: this API deliberately promises topology without
Delaunay quality. Do not add an empty-circle decision requirement or weaken the
radial winding consistency check to repair this failure.

Before an algorithm edit, reproduce the native public common-cell test and the
five-point private canonicalization test on the existing implementation. Require
strict Certified output, all original protected subsegments, positive triangles,
and independently computed integer double areas (1080 for the common cell).
Constraint order changes must not change this common-cell subdivision. Final
native Boolean tests must establish volume 46 for the reduced boxes and volume
124 with its unit cavity for all 26 original boxes, including downstream use.

A candidate may use existing exact constraint recovery to establish a globally
ordered noncrossing edge selection. Every accepted edge must avoid protected
edges and interior input vertices; ordinary inability to prove a predicate stays
typed. Record any necessary triangulation/artifact migration separately from
semantic correctness and retain unfavorable costs. All temporary tracing has
been removed before the native regression baseline. No implementation repair or
performance claim is established by this diagnosis alone.
