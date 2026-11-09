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

The separate parent branch's raw logs, manifests and preserved original
fixtures remain local under
`target/d03-native-corpus-20261108/svg-native-attributes-admission-20261109`.
The [combined-source qualification](svg-exact-paint-2026-11-09.md#combined-native-attribute-and-hidden-input-qualification)
subsequently adds an independent mixed CSGRS witness and a narrow hidden-native
validation correction. Committed `2639c68fee` passes 118 owner reports, six root
region reports, ten selected rows, matching dispatch controls, nineteen MSRV
bodies and 57 WASM executions. Its seven root SVG harnesses pass sixteen reports,
and full BRep/IO release suites pass 996. Strict owner/root lint passes. These
are actual combined-source receipts, not attribution of parent-only checks.
Immutable evidence closeout and a complete new publication gate remain pending;
no evidence has been uploaded.

The continuation below addresses unrounded rectangle coordinates/dimensions
beyond that initial line/point-list increment. Sampled circle/ellipse/rounded
fidelity, broader SVG/OpenSCAD source conformance, arbitrary algebraic operation
closure and whole-operation work/heap/cancellation control remain open. The
finite grammar envelope is preserved; no uniform practical completion bound is
claimed for arbitrarily large scalar descriptions.

## Exact unrounded rectangle attributes

The rectangle continuation is local correctness work in Hypercurve, P1, P6,
P8–P9, P12, P17–P19, P25, P27 and P39. Its domain is the existing finite numeric
attribute syntax, positive exact dimensions and nonnegative corner radii. An
absent or exactly zero effective radius selects an unrounded rectangle.
`CurveRegion2` and its existing arrangement remain the filled-set authority.
Public signatures and native codec versions do not change.

On acceptance-only `7b4782d7bb`, the original six witnesses compile and execute:
five fail assertions and one valid-hidden control passes. Failures reproduce
rounded decimal sets, negative-underflow radii, replay/transform disagreement
and overflow of a finite-source coordinate sum. The seven-case continuation
retains those six bodies as a byte-identical prefix. A seventh witness failed
on the first correction: valid hidden rounded input with positive `1e-400`
dimensions was still rejected by sampled materialization. It now validates the
exact fields and selects empty before that optional work.

Authored fields reuse `exact_number` and the existing exact scalar sign owner.
Native unrounded endpoints use exact addition. Rounded visible geometry keeps
its existing sampled calculation. Valid hidden/unpainted geometry retains
successful empty selection; invalid fields still fail typed before selection.
No approximation, fitting or second arrangement replaces the exact lines.

The seven owner tests pass under both explicit policies, with two verified
fresh native child replays and independent rational-boundary XOR expectations.
They cover decimal/scientific fields, adjacent large integers, tiny extents,
zero-radius conventions, exact paint/transform composition, native replay,
reflection, Boolean/empty recovery, typed invalid fields and valid hidden
rounded attributes. The finite-source `1e308 + 1e308` witness qualifies native
construction and Booleans only: native SVG export still has its finite display
projection restriction.

Eight visible rounded controls (four cases under both policies) retain identical
serialized sampled boundaries and certainty against the unchanged baseline.
Exact parsing and four new sign guards add measured dispatch/rational work;
all complete vectors are retained. The second correction retains exactly the
first correction's visible control vectors and rational counters. These are
correctness changes, not a behavior-preserving refactor or a speedup claim.
They do not establish analytic rounded-source fidelity.

Seven rows are registered alongside the unchanged 449 original catalog objects.
The root adapter adds an independent exact rectangle/paint/transform witness.
On the bytes committed as `0963cec833`, the twelve owner SVG harnesses and root
adapter harness pass 132 parent reports without failures or ignores. Seventeen
selected capability rows pass; this does not qualify all 456 catalog rows.
Strict owner all-target/all-feature Clippy, formatting and whitespace checks
pass. An earlier whole-private-helper include failed the final formatter check;
its raw attempt remains. The root shares the existing literal macros and uses
the public policy/receipt API, leaving the owning helper unchanged, and the
entire affected run was repeated successfully.

Rust 1.95.0 passes all 26 maintained decimal, paint and rectangle bodies and
strict owner SVG Clippy. Those exact bodies execute 78 times across three fresh
pure Rust WASM instances, with zero host imports and the original compiled
stack. Native child-process guards remain unchanged. The module is 15,197,186
bytes (SHA-256 `fc571144e6c80808615e152867380c9362c0a3840ef7cb2d978785a2aeffbbbe`).
These observations do not establish a public ABI or a total memory bound.

The seven root SVG harnesses and 110 BRep/IO harnesses pass 1,013 parent reports
without failures or ignores; strict root all-target/all-feature Clippy also
passes. Portability and downstream commands execute on the immutable commit
with all 3,839 source inputs unchanged. The initial owner run is tied to that
commit by an exact checked-source byte comparison. Raw command/output digests,
source/fixture proofs, full rounded-control vectors and the original red cases
are retained locally under
`target/d03-native-corpus-20261108/svg-rectangle-admission-20261109`.

Archive/index closeout and a fresh complete pre-push gate on the eventual
integrated source remain pending. Local qualification does not establish broad
SVG/OpenSCAD source conformance, analytic sampled-shape fidelity, arbitrary
algebraic operation closure or total work/heap/cancellation control.

### Integrated rounded sampler limit classification

On integrated `7841be10b8`, two new owner assertions and one root assertion
reproduce valid visible rounded input being reported as malformed. Positive
`1e-400` dimensions underflow the sampler; finite `x = width = 1e308` overflow
its computed bound. The owner is the existing Hypercurve sampled representation
boundary (P6, P14, P16, P18, P25, P27, P39). Exact validation already establishes
that these source descriptions are valid. The correction returns
`SvgError::Unsupported` before sampled construction; the public root adapter
preserves that category. This is a declared implementation gap, not exact
rounded completion, invalid geometry or exhausted resources.

Two additional catalog rows assert unsupported outcomes, preserving all 456
previous objects. The seven qualified rectangle and seven root test bodies
remain byte-identical prefixes. Integrated-source owner, portability, downstream
and publication qualification are pending; the earlier committed receipts above
retain their original source attribution.
