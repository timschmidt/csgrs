# Exact native SVG primitive coordinates — 9 November 2026

Hypercurve owns line-attribute and point-list numeric ingress, P1, P6, P12,
P17–P19, P25, P27 and P39. Three maintained witnesses fail at runtime against
unchanged production `a8b2392180`: line and polyline endpoints differ from
independent rational values, and filled point-list input differs from an
independently constructed exact rectangle. The nine original decimal/transform
tests and the new malformed-input controls pass on that baseline. Red means
these actual assertion failures, not a compiler failure or timeout.

Native line attributes now reuse the existing numeric attribute validator and
the shared exact token cursor. Rounded validator output is discarded for an
authored value. An omitted internal default retains the existing exact binary
conversion; current line defaults are zero. Point lists retain the complete
existing grammar/finite check and pair-count validation before exact scalar
conversion. Their temporary floating values supply only the token count; the
original spans supply both point coordinates. There is no second decimal lexer
or temporary vector of exact scalars.

Primitive construction, paint selection, native family choices and region
arrangement are unchanged. No curve fitting, tolerance merge or sampling
substitutes for native coordinates. Public signatures and codec versions remain
stable. A tiny exact line survives native replay even when its display endpoints
round together; display data never becomes the authoritative operand.

All 13 focused test reports pass under the maintained both-policy loops. The
nine original test bodies remain byte-identical; four appended witnesses cover
exact line attributes, point-list coordinates, filled-set continuation and typed
invalid/degenerate input. Expectations use independent integer ratios, adjacent
large integer literals and `1 / 10^400`, rather than a second import of the same
token. The filled polygon/polyline witness uses a complete XOR against an
independent rational rectangle, four verified fresh native test processes,
separate deterministic re-export, reflection, later difference/union and empty
recovery. Invalid input with disabled fill/stroke remains an error; valid empty
selection remains successful geometry.

All ten affected Hypercurve SVG harnesses pass 112 reports, with no failures
or ignores. The 17 runner tests and four selected capability rows pass;
this does not qualify all 443 catalog rows. Strict all-target/all-feature owner
Clippy, repository formatting, source budget and whitespace checks pass.

Three unchanged path-command controls run under both policies against the
unchanged committed baseline and candidate. Complete nonempty dispatch vectors,
rational counters, certainty and all eight SVG strings agree exactly. These
controls do not exercise the intentionally corrected native attributes; those
use the independent coordinate/set expectations above. No speedup or universal
equivalence claim follows from this finite control comparison.

Rust 1.95.0 passes all thirteen maintained bodies and strict owner SVG lint.
Those same thirteen bodies execute in three fresh pure Rust WASM instances
with zero host imports and the original compiled stack configuration. Original
native child-process guards remain; WASM still executes full native metadata
and region continuation. Module/memory observations do not establish a public
ABI or a total resource bound.

CSGRS/BRep downstream checks, mixed paint/native integration, immutable evidence
closeout and a new complete publication gate remain pending. Focused raw logs,
manifests and preserved original fixtures currently live locally under
`target/d03-native-corpus-20261108/svg-native-attributes-admission-20261109`.
They have not been uploaded or counted as complete qualification. This branch
is independent of the separately qualified paint correction; their combined
source still needs mixed-input and final downstream verification.

Rectangle coordinates/dimensions, sampled circle/ellipse/rounded-rectangle
fidelity, broader SVG/OpenSCAD source conformance, arbitrary algebraic operation
closure and whole-operation work/heap/cancellation control remain open. The
finite grammar envelope is preserved; no uniform practical completion bound is
claimed for arbitrarily large scalar descriptions.
