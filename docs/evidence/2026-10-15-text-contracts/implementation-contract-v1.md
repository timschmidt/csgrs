# Implementation decision after reproduction

`native-outline-v1` passes all four commands on the unchanged target revision.
The original README and benchmark strings produce 151 and 662 native source
curves respectively, regularize strictly to 3 and 19 boundary loops (including holes), and survive
strict translation and union with empty. This advance-only feasibility probe
omits kerning, so it does not admit final layout behavior.

Implement authoritative TrueType outlines using the existing native line,
quadratic/cubic Bézier, path and NonZero-region machinery. Preserve the caller's
`Real` scale and layout arithmetic. Use the exact point-to-millimeter ratio
`127/360`, replacing the old binary64 `0.3527777`. This is an explicit geometric
and serialization migration from eight-step sampled polygons, not a performance
optimization. The primary API returns `Result`; its context form carries local
certainty. No implicit legacy sampled fallback remains.

The parser boundary is explicit: ttf-parser supplies f32 outline coordinates;
those represented values are promoted exactly. Native geometry retains their
curves and exact requested scaling, but does not certify preservation of
pre-parser coordinates for composite, variable or CFF fonts. The layout remains
the documented simple horizontal mapping with advances, pair adjustments and
line/tab controls, not a shaping or bidirectional-text engine. Preserve native
parse causes and identify missing glyphs, malformed/unsupported outlines and
failed geometry separately from successful blank text. Never publish the
partial callbacks produced by a malformed glyph.

Hershey failure propagation belongs in Hypercurve. Make native `strings`
fallible, validate compiled records and nonzero size before publishing text,
and retain decode/curve/resource diagnostics. Negative sizes remain supported.
CSGRS uses that implementation and performs its Cartesian reflection under the
selected policy, preserving typed errors and local certainty. Check original
Hershey coordinates byte for byte; no representation migration is intended.

Dimension annotation must propagate text-construction failures before mutating
the drawing. Its existing display-only numeric formatting does not become
authoritative geometry; the measured length remains `Real`. Update affected
examples, tests, native docs and benchmark callers rather than adding a silent
empty-on-error adapter.

Acceptance includes synthetic line/quadratic font geometry with independent
exact area/membership, scales `1` and `1 + 2^-100`, supported symbolic scale,
blank/malformed/missing-glyph cases, native downstream composition, both original
full-size text workloads, and native Hershey/error/annotation consumers. Record
costs against the preserved source without treating changed TrueType geometry
as an optimization or demanding obsolete sampled-coordinate equality.
