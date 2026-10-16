# Represented-set Minkowski acceptance contract

Target revision: `adef9a64a91a7524006ba41527e427e2edb79fbe`, including the
unchanged 35 pending native files. The prior face/loft milestone is committed
and qualified. The full properties goal remains active.

Hypermesh owns closed-PWN validation, convex decomposition, exact intersections,
winding and Boolean union. CSGRS owns the canonical-empty bridge and public
modeling vocabulary. Hyperlattice and Hyperlimit retain scalar construction and
predicate ownership. Relevant properties: P1–P11, P17–P27, P30, P35–P39.

The intended operation is the regularized set sum `{a+b | a in A, b in B}` of
bounded, closed-PWN represented solids, with nonzero winding defining material.
Disconnected components, concavity, holes, reversed shells, winding multiplicity,
canceling shells, unused stored positions and canonical empty results are in
scope. Invalid indices, open or degenerate triangles, and unresolved predicates
must retain typed causes. Validate both operands before an empty shortcut.
Sampled polyhedra remain sampled polyhedra; no analytic-surface claim is made.

Reproduce the existing disconnected, unused-point and empty/malformed failures
on the pinned target before production edits. Preserve original cube(2)+cube(3)
ordered output and benchmark inputs. Explicitly migrate the old vertex-hull
interpretation to a separately named point-set hull operation; preserve its
terminal-equality fixture and add strict represented-set success with the same
unused symbolic position. Do not silently change tests or supported policies.

The decomposition identity is `A+B = union(A_i+B_j)` for convex covers of A and B.
This identity is also the method described in the
[CGAL manual](https://doc.cgal.org/latest/Minkowski_sum_3/index.html#Minkowski_sum_3Decomposition).
No CGAL implementation is copied. The proposed native construction refines the
convex source bounds by all source supporting planes, then selects full-dimensional
cells by the existing signed winding ray predicates. Within every open cell no
source triangle is crossed, so winding is constant. Convex-cell splitting and
pair sums reuse exact native intersections and convex hulls. Final union reuses
the single native arrangement. Convex source certification may avoid subdivision
but must prove the same represented-set claim.

Acceptance uses independently derived exact volumes and membership for separated
boxes, an L prism, a hollow body, nested/overlapping/canceling shells, transforms,
both policies and repeated composition. Validate output topology separately and
consume results in Booleans. Tests must detect convexification and parity errors;
wrapper agreement and triangle counts are not semantic oracles.

Explicit limits on decomposition cells, pair sums and generated point counts,
plus cooperative cancellation, must fail before complete output publication.
These counts are not total work, heap-byte, coefficient-growth or time bounds;
native validation/hull/arrangement calls remain externally contained. Small
limits must exhaust, release temporary state and permit subsequent recovery.
Default limits and any migration are documented. No provisional mesh is returned
as a completed sum.

Retain a frozen baseline and candidate cube-sum cost harness: three fresh timing
children and one allocation child at each cold/warm state. Include input
construction and output materialization, including internal validation, inside
timing; separate independent topology/volume/serialization checks occur outside.
Keep exact output rows, raw unfavorable costs, source/toolchain/dependency pins
and process resources. This is a correctness repair, not an optimization claim.

Qualify affected native/Rust/JS consumers, warning-denied Clippy, docs and original
benchmarks after the final edit. Preserve all prior artifacts and pending hashes.
Only the ledger's raw `current_work` value may change. No push or full-plan
completion claim follows from this milestone.
