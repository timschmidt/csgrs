# Exact SVG paint selection — 9 November 2026

Hypercurve owns numeric SVG paint ingress and geometry selection. The affected
contracts are P1, P6, P8–P9, P12, P17–P19, P25, P27 and P39. This is a
correctness extension; the existing curve, arrangement and native replay
authorities remain responsible for geometry.

On production `a8b2392180`, four new maintained tests fail at runtime: tiny
positive opacity disappears, inherited positive factors underflow, invalid
negative factors become successful empty geometry, and the resulting empty set
cannot satisfy independent native-replay/Boolean expectations. A separate
bounded baseline observes the same sign failure with 400-digit exponents under
both policies. No compiler failure or external timeout establishes these gaps.

Exact scalar conversion and the existing STRICT sign/order predicates now
validate each explicit paint value. The previous floating parser remains only
the finite syntax/range guard. Its rounded value never supplies an authoritative
sign or bound. Percentage values are compared with 100 before normalization;
this preserves the equivalent interval and avoids materializing an unused
quotient. Scalar conversion errors and undecided required predicates remain
typed. Proved invalid ranges produce `SvgError::MalformedInput`.

The importer uses paint values only to select topology; it does not retain
appearance or construct stroke thickness. After validating nonnegative values,
exact positivity is sufficient. A finite product of nonnegative opacity factors
is positive exactly when every factor is positive, so accumulated opacity keeps
that fact through conjunction. Individual fill/stroke opacity and stroke-width
properties retain their previous inheritance and override rules. Property order,
inline-style precedence, display and visibility behavior remain explicit. The
default context requires no new scalar decision.

The original four test bodies pass unchanged. A fifth adds positive, zero and
negative 400-digit scientific exponents, with and without percentages, using
the scalar owner's lazy representation. Filled selection is checked against
an independently constructed exact unit-square boundary. The replay witness
verifies two fresh native test processes, separate deterministic re-export,
reflection, later difference/union/XOR and canonical empty recovery. The five
tests keep STRICT and APPROXIMATE_512 certainty receipts.

Three unchanged controls run under both policies against committed main and
the candidate. Complete nonempty dispatch vectors, rational counters, certainty
and all eight serialized SVG strings agree exactly. These controls contain no
explicit numeric paint values; the corrected paint inputs intentionally change
former rounded outcomes. This is finite functional evidence, not a controlled
performance or universal equivalence claim.

All 11 affected Hypercurve SVG harnesses pass 113 test reports, with no failures
or ignores. The 17 capability-runner tests and five selected witness rows pass;
this does not qualify all 444 catalog rows. Seven existing CSGRS SVG harnesses
pass 14 reports, and the complete BRep/IO release suites pass 996 reports across
110 harnesses, with no failures or ignores. Owner and root strict all-target,
all-feature Clippy and repository formatting pass.

A direct CSGRS paint witness was then appended without changing the original
four root region test bodies. All five tests in that harness pass, followed
by strict root Clippy and formatting. The new witness checks inherited tiny
paint and an exact affine map against an independent rational rectangle,
native replay, separate deterministic re-export, further Booleans, valid zero
selection, exact stroke endpoints/replay and typed invalid ranges. Production
and all five owner acceptance bodies remain byte-identical throughout this
strengthening.

Rust 1.95.0 passes all five maintained paint tests and strict owner lint.
The same five bodies run in three fresh pure Rust WASM instances with zero
host imports and the original compiled stack configuration. Native subprocess
replay retains its original target guard; WASM still exercises the complete
region continuation. Module size and memory observations do not establish a
public ABI or a total heap/work bound.

A new full pre-push gate remains required before publication of this code.
The separate `a8b2392180` transform gate does not qualify the paint candidate.
Raw logs, manifests, red/green receipts, unchanged-body proofs and control
artifacts currently live under
`target/d03-native-corpus-20261108/svg-paint-admission-20261109`. Evidence is
local and unuploaded; immutable archive/index closeout remains pending.

Public signatures and native metadata versions are unchanged. At this separate
paint checkpoint, primitive coordinates and point lists remained outside its
scope. Sampled circles/ellipses/rounded rectangles,
general SVG/OpenSCAD source conformance and arbitrary algebraic operation
closure remain open. No total arithmetic-work, retained-memory or cancellation
bound is added. External diagnostic containment is not kernel exhaustion or a
completion guarantee.

## Combined native attribute and hidden-input qualification

The independent paint (`0e840c6bf3`) and native-attribute (`16a4a8f65d`)
commits combine at `599619c175`. The original five paint bodies, thirteen
native decimal/transform bodies and five root region bodies are retained.
A sixth CSGRS region witness combines exact line/polygon/polyline coordinates,
tiny positive paint, inherited opacity and exact transforms. Independent
rational endpoints and full rectangle XOR, native replay, deterministic
re-export, later Booleans and empty recovery establish its expectations.

The initial combined run passes the original suites but fails this new witness:
an odd polygon point list under zero ancestor opacity becomes successful empty
geometry. A separate owner witness reproduces malformed hidden line admission
on unchanged production. Neither failure is a compiler error or containment
limit. The correction at `2639c68fee` lets existing line/polygon/polyline
constructors validate hidden native input before selection. Shared geometry
conversion returns empty before transforming or arranging valid hidden input.
Other SVG families retain their existing visibility handling. The new sixth
paint body pairs malformed native coordinates, arity and degeneracy under
zero opacity/display/visibility with valid-empty continuation controls. The
committed failing CSGRS assertion is unchanged; no parent fixture is weakened.

Actual qualification of committed `2639c68fee`, with all 3,838 tracked inputs
unchanged through terminal completion:

- Eleven owner SVG harnesses: 118 passing reports; root region replay: six.
  There are no failures or ignored tests. All ten selected capability rows
  and 17 capability-runner tests pass. This does not qualify all 449 rows.
- Three unchanged controls under both policies: complete dispatch vectors,
  rational counters, certainty and all eight SVG strings match actual `a8b2392180`.
- Rust 1.95.0: all nineteen maintained owner bodies and strict owner SVG lint
  pass. The same nineteen bodies execute 57 times in three fresh WASM
  instances, with zero imports and the original compiled stack. Native-only
  process guards are retained.
- Seven root SVG harnesses: sixteen passing reports; full BRep/IO release
  suites: 996 reports across 110 harnesses. Owner/root all-feature/all-target
  strict Clippy, formatting and whitespace checks pass.

Raw red/green receipts, exact source maps, unchanged-fixture proofs, complete
control outputs, portability sources and downstream logs are retained locally
under `target/d03-native-corpus-20261108/svg-numeric-integration-admission-20261109`
and the named runs under `target/brep-admission-20261105`. The two independent
parent sources are preserved from immutable Git blobs; the current integration
worktree is not substituted for their previously qualified input. These are
functional completion receipts, not performance or general closure evidence.
Immutable evidence/index closeout and a complete new publication gate remain
pending. The separate transform gate cannot qualify this combined source.
