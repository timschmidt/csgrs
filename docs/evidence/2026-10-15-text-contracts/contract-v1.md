# Text construction acceptance contract

Target revision: `72cd216cc`, including the unchanged 35 pending native files.
The previous goal turn made verified progress on sampled mechanical failures;
it did not admit those native files or complete the properties plan.

Owning layers: Hypercurve owns decoding Hershey records and constructing their
native strokes. CSGRS owns Cartesian text layout, font-format conversion and
public modeling composition. Hyperreal owns scalar values; Hyperlimit owns
predicate evidence; native Hypercurve paths and regions own topology. Reuse
those implementations rather than adding an adapter kernel.

Relevant properties: P1, P4, P5, P6, P8, P9, P12–P20, P25–P27, P29, P35–P39.
The missing acceptance cases are invalid font data, unsupported/malformed
glyphs, degenerate strokes, undecided scale, successful blank text, and
preservation of exact requested scale and consumed certainty.

First reproduce current behavior before implementation. Retain full original
README inputs (TrueType `CSG` at 72 and ROWMANS `CSG` at 1), benchmark inputs
(TrueType `csgrs benchmark` at 24 and FUTURAL `CSGRS` at 2), and independent
custom Hershey strokes. Pin all represented output coordinates and diagnostics.
Also compare distinct positive exact TrueType scales that round to the same
binary64 value; agreement there is a defect, not a preservation requirement.

Primary construction must return typed errors, never successful empty or
partial text on failure. Preserve native causes and policy receipts. Empty
text, whitespace and explicitly blank glyphs are successful empty geometry
after relevant input validation. Missing glyphs and malformed records are
different from blank glyphs. Preserve existing supported invertible Hershey
scales, including negative scales; invalid zero and undecided scale require
explicit diagnosis. TrueType's declared positive scale is not a float range.

TrueType numerical certainty, topology and fidelity must be separate claims.
The current conversion rounds requested `Real` scale through binary64 and
flattens every font curve into eight segments. Do not silently promote that
sampled result into an exact font outline. Investigate native exact outline
construction on the pinned parser output and document any parser-boundary
quantization. If representation, units or serialization must migrate, identify
the migration explicitly and preserve all original full-size completion cases.
Any retained lossy convenience must be explicitly named and intentionally used.

Use independent exact stroke coordinates, advances and axis orientation, and
independently derived synthetic font geometry where appropriate. Test blank
composition, exact transforms, further native operations, and strict →
approximate → strict evidence use. Matching two wrappers or checksums is a
preservation check, not an independent semantic oracle. Do not discard hard
glyphs, reduce original text/resolution, switch policy or accept new Unknown
outcomes to pass a previously supported workload.

Use checked capacities and reservations where this change owns allocation;
disclose absent internal work/heap-byte/cancellation bounds. Dangerous probes
use external 1800-second/8-GiB containment. Retain all trials and cost changes.
Run affected native and modeling tests, downstream dimension annotations,
feature combinations, lint, docs, examples and benchmark consumers after the
final implementation edit. External release validation and the full plan remain
open; only `current_work` in the protected properties ledger may change.
