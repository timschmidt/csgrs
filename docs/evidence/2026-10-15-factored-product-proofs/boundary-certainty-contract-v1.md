# Boundary certainty acceptance

Owning layers: Hyperreal supplies the stronger exact facts; the CSGRS scalar
adapter, VRML importer and JavaScript binding must report the certainty actually
consumed. Relevant properties: P4, P8, P9, P18, P25, P27, P30, P35 and P37.

`downstream-frontier-v1` reproduced three stale expectations on parent
045621eba plus the recorded E12 patch: the unchanged 16-segment through-drill
now succeeds strictly through both adapters, and the unchanged rotated concave
VRML polygon now imports strictly. All failures remain recorded. These findings
are not attributed solely to E12; the cumulative scalar extensions matter.
`adapter-certainty-v1` independently checks the same drill on both committed
045621eba and E12, with closed manifold geometry and independent inside/hole/
outside points. Both certify under both policies. Its additional translated-cube
fixture retains strict failure, qualified approximate empty, then strict failure.

Retain every original input, including 16 drill segments and the VRML polygon
and rotation. Require explicit certified receipts under strict and approximate
policies, with independent point membership for the drill. Retain the existing
144-triangle binding artifact check separately from semantic checks. For VRML,
require three triangles and no ignored degeneracies, and independently check
that total unsigned area lies within 1/10^12 of 3 (source shoelace area, preserved
by a rigid rotation). This is a certified enclosure, not an exact area-equality
claim or a closed-solid topology claim.

Keep genuinely unresolved trigonometric identity cases as additional policy
coverage. No approximate evidence may satisfy a later strict request. Run the
affected root feature matrix, Clippy, and the real generated Node/WASM and
TypeScript consumers after edits. The complete native scalar/foundation/mesh/
curve matrix will also be repeated on the final source. This record defines
acceptance; it does not establish that pending edits or checks have passed.
