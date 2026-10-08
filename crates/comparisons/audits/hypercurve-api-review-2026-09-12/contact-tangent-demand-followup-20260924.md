# Contact tangent demand (read-only follow-up)

BezierAlgebraicChordParallelContact2 eagerly stores both cross and dot signs.
RegionBooleanContext::algebraic_chord_parallel_pair_result consumes only the
cross sign and contact/parameter identity. Curve2 fillet-center construction
consumes both, with the source-derivative orientation adjustment retained.
The dot is mathematically meaningful even at a transverse contact, but it need
not be established before publishing an arrangement contact that never asks
for it. Other contact carriers already keep optional dot certificates.

A future consolidation should retain the common point/parameter certificate
and compute tangent relations only when the consuming operation requires
them. It must preserve fillet frame and traversal semantics and the existing
exact dot/cross regression oracles; deleting the dot proof or substituting an
unknown with zero is not valid. The current local-chord candidate keeps the
contract intact and fixes its native-parameter support-line dispatch instead.
No production change for this follow-up has been made.
