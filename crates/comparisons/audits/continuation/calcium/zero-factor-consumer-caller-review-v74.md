# Zero-factor consumer call-path review — checkpoint 74

This is a selected-range re-read of already bound Hyper sources, not new donor
coverage, a complete new Hypercurve read, or runtime branch instrumentation.
The source maps in checkpoint 70/current retained bindings and the consumer
binding verify the complete file identities; only the ranges below were read
for this review. The candidate leaves these callers byte-identical.

| Frozen file | Reviewed inclusive lines |
| --- | --- |
| point-demand-candidate/hypersolve/src/algebraic.rs | 1100–1345; 1820–1935 |
| point-demand-candidate/hypersolve/src/algebraic_rational_image.rs | 590–720 |
| point-demand-consumer-v66/hypercurve/src/bezier_algebraic_image.rs | 155–205; 325–405; 780–855; 2160–2255 |
| point-demand-consumer-v66/hypercurve/src/bezier_offset.rs | 65480–65635 |
| point-demand-consumer-v66/hypercurve/src/bezier_arrangement.rs | 1580–1630 |
| point-demand-consumer-v66/hypercurve/src/bezier_tangent_order.rs | 1160–1275 |
| point-demand-consumer-v66/hypercurve/examples/basic.rs | 1–29, complete |
| point-demand-consumer-v66/hypercurve/examples/arrangement.rs | 1–44, complete |

The public represented-root arithmetic dispatcher prefers exact point/scalar
and same-representation paths before independent resultant construction. The
independent path rejects point-witness operands, invokes the binary constructor
for add/subtract/multiply/divide, maps Transformed to ComputedRepresentation,
and preserves explicit Undecided/error outcomes. Therefore testing a rational
point through the dispatcher does not automatically exercise divisor deflation.
The existing independent binary-constructor corpus is direct qualification of
the modified public function, not evidence that every dispatcher route uses it.

The rational-image fallback transforms numerator and denominator polynomials,
then calls represented-root arithmetic with Divide. It accepts a valid computed
representation or builds a constant representation from an exact witness; other
outcomes remain explicit. This is an indirect consumer route to the changed
constructor, not a claim that all rational maps reach the fallback.

Hypercurve's arithmetic adapter calls STRICT first and only tries the explicitly
permitted APPROXIMATE_512 terminal when appropriate. It records a successful
approximate terminal in the context. Geometry rational coordinate/projection
paths likewise use strict rational-image construction or retain certified
denominator-sign evidence; an approximate scalar match does not silently create
an exact geometric proof. The reviewed tangent/arrangement callers request
negation, multiplication, addition or subtraction. The selected offset relation
helper supports sum/product and separately compares the resulting represented
root to the selected result, avoiding a foreign-conjugate equality claim.

The basic example builds a quadratic and a square region and checks a decided
point classification. Arrangement builds four straight edges, checks exact
native arrangement status and interior classification. Their successful runs
qualify those representative integration paths and linked sizes. They are not
direct tests of unused divisor-zero-factor recovery; no such coverage is credited.

No consumer algorithm, policy, public API or validation contract is changed.
The broad consumer regression suite is a separate compatibility gate from the
independently proved binary mathematics and matched native/WASM cost corpora.
Remaining ecosystem source reads and unresolved transfers remain open.
