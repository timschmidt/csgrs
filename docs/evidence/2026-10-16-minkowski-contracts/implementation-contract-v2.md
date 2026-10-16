# Final implementation and acceptance scope

The represented-set domain, typed failures and resource units in `contract-v1.md`
remain. Hypermesh owns decomposition, signed winding, convex pair hulls and their
union; Hyperlattice owns the exact projective edge/plane intersection; CSGRS and
WASM expose the native operation and its receipt. The ray/triangle implementation
is shared with the existing surface arrangement. Unused positions are removed
by index usage before native validation. Both operands are validated before a
successful empty identity. A separate affine-rank proof regularizes a planar
closed-PWN input to empty; a failed hull is never interpreted as empty.

The initial proposal's single final multi-operand union is superseded by
incremental binary native unions. `union-baseline-v1` independently reproduces
the existing 26-box failure using the pre-edit libraries: both a 26-input union
and a merged-shell union fail winding propagation, while the binary accumulation
certifies volume 124 and closed-PWN topology. The same complete hollow fixture
remains required here. This changes the composition strategy, not the represented
set, policy, fixture or kernel. The native batch-arrangement failure remains an
open regression; this patch does not claim to repair it or improve P21's reuse.

The unchanged original 16×8 sphere example requires two scalar proofs detailed
in `root-square-contract-v1.md` and `eighth-angle-contract-v1.md`. Hyperreal owns
both. The former adds a separately bounded, domain-checked constant-square
projection after previous normal forms decline. The latter recognizes two exact
selected radical leaves inside the existing tower. Neither changes authoritative
symbolic coordinates, scalar serialization or predicate policy. The full sphere
test checks strict completion, topology, exact bounds, and independent interior
and exterior witnesses in addition to the other exact-volume/set regressions.

`minkowski_sum` deliberately migrates from the hull of all stored vertex sums to
the regularized sum of the represented nonzero-winding solids. `minkowski_hull`
explicitly preserves the old point-set intent. Root and JavaScript errors retain
typed categories and native causes where supported. Bare meshes still do not
carry inherited construction provenance; local certification is not a repair of
that broader gap. Collection limits and cooperative cancellation do not bound
primitive work, scalar growth, heap bytes or elapsed time.

The migration also permits the decomposition and union to select a different
triangulation or vertex order for the same set. Determinism is checked separately
from set equality. The original cube(2)+cube(3) ordered artifact has its own
preservation requirement; this is not an assertion that every previous vertex-hull
artifact is identical under the new solid operation.

Final qualification must include scalar/default/all-feature and downstream
predicate, lattice, solver, triangulation, mesh, curve and root tests; relevant
Clippy/docs; original benchmark consumers; and actual affected bindings. The
pre-edit cube cost driver remains fixed and requires identical ordered output.
Additional scalar probes are retrospective comparisons against immutable pinned
pre-edit libraries, not a pre-edit optimization baseline. New general-sum cost
rows have certified semantics and no equivalent correct old workload. Report
completion separately from cost and retain unfavorable measurements. Full CI,
clean external release, provenance, complete internal bounds and the entire
properties plan remain open.
