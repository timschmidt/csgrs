# Distribution and concatenation migration

`solid::merge`, `distribute_linear`, `distribute_grid`, and `distribute_arc`
return `Result<TriangleMesh, ValidationError>`. Callers must propagate or handle
failure. Existing context forms continue to return a certainty-bearing result.
Native Boolean errors remain `ValidationError::Mesh`, preserving their cause
and category. There is no new implicit lossy fallback.

Distributions accept canonical empty geometry and nonempty closed PWN sources.
Malformed buffers, open surfaces, degenerate faces and unbalanced edge winding
fail before zero/one-copy and disjoint shortcuts. Zero-copy linear layouts also
require a provably nonzero direction. This intentionally rejects earlier
shortcut successes outside the declared set-modeling domain. Empty sources
need no copy storage; signed/zero spacing, signed radius and the one-copy arc
midpoint retain their meanings. Counts, aggregate position/triangle byte sizes
and copy-vector reservation are checked before copy construction.

`TriangleMesh::validate_closed_pwn` shares the native Boolean input validator.
It checks topology without computing bounds or materializing a polygon soup,
and retains the combined certainty of nondegenerate faces and balanced edges.
This does not require manifoldness, regularize winding multiplicity, prove a
nonempty represented set, or certify inherited construction intent.

`TriangleMesh::try_concatenate` validates indices before rebasing and uses
checked count/byte arithmetic and fallible vector reservations. Open surfaces,
unused positions, order, coincident triangles and winding multiplicity remain
supported. Existing PWN facts are transferred only for matching source buffers,
with the weakest certainty. A strict request can upgrade an approximate fact
only through new native proof. Mutation invalidates evidence through the
existing buffer-identity mechanism. Successful modeling results alone enter
caches; failure never enters the next operation as geometry.

The implementation changes error precedence for invalid sources before layout
shortcuts and moves native edge validation before polygon assembly. Exact
successful coordinates and triangle rows are required to remain unchanged on
the retained original corpus. The caller and README migrations are explicit.
No binding exposes these distribution methods directly; shared native Boolean
validation still requires actual binding-consumer checks.

Vector checks are not a total-work, expression-growth, heap-byte, cache-byte or
cancellation budget. Scalar clones and shared-buffer conversions retain their
existing infallible allocation behavior. These limits and missing end-to-end
provenance remain open. This patch does not claim release readiness, full CI,
formal coverage, fuzz-campaign completion, or admission of the 35 pending native
Hypercurve/Hyperdrc files.
