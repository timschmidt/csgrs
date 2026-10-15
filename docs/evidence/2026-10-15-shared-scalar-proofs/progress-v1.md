# Shared proof reuse and strict drilling progress

Baseline `df2dfac1d` retains the failing shared-expression regression against
`abe117261`: a balanced seven-addition DAG for `128*d/d - 128`, with d strictly
positive, exceeds the 512-node limit when unfolded. The candidate reuses only
completed quotient projections by immutable node identity within one query.
The borrowed root keeps every key alive; no entry survives the attempt.

Each hit consumes a work charge and checks cancellation before cloning the
projection. It performs no further node expansion. Misses keep the existing
admission, depth, node, term, degree, coefficient and work limits. At most 512
temporary projections can be retained, each with admitted numerator and
denominator polynomials. This is an entry-count limit, not a heap-byte bound.
Existing opaque structural comparisons and earlier recognizers still preclude
a whole-query work or memory guarantee. No negative result is cached and the
authoritative expression is unchanged. Laurent and polynomial proofs remain
preferred. No native production file changed.

[Scalar-and-drill qualification](scalar-and-drill-v1/result.json) passes all
15 commands: formatting, 908 default scalar tests, 1015 all-feature scalar tests,
all-feature/all-target scalar Clippy with warnings denied, the CSGRS curve
library build, eight full-resolution drill observations and 28 scalar doctests
with warnings denied. Counts overlap across configurations. Controls include
work exhaustion and cancellation on an already cached projection, recovery,
depth limits, exact denominator evidence and concurrent readers.
The final model tests also pass all-target CSGRS Clippy with the curve feature
and warnings denied in a fresh isolated target.

All original 8/16/24/32-segment radius-2 polygonal drills through the 12-by-12-by-4
body now complete with `Certified` outcomes under both strict and approximate
contexts. Each separately passes certified closed-manifold validation. The
16-segment context test's previous expectation of strict failure was reproduced
and then strengthened to require certified completion, including the later
strict call. Its geometry and original volume assertion remain.

The new full-resolution semantic test checks independent interior, exterior
and hole-boundary samples, including exact displacements of `1/10^12`, and
certifies `|6V - (3456 - 48*n*sin(2*pi/n))| < 1/10^24`. This is an independently
derived polygonal-volume enclosure, not a claim that exact volume equality was
decided. Exact equality for the large 16-segment volume expression still returns
`Unknown`. No analytic-cylinder fidelity claim is made.

Acceptance remains incomplete. The final
[model-acceptance-v3 trial](model-acceptance-v3/result.json) retains a strict
composition test for each resolution: accumulate the drill result from empty,
subtract the original body to remove everything, then union the empty result
with that body. The 16/32 cases fail at empty accumulation with typed
`PredicateUndecided("Real sign classification")`. The tests continue to require
success; no fixture, policy, validation or assertion was weakened.
The 8/24 cases pass the complete chain. This trial records two passed and two
failed composition cases, alongside the passed context and semantic tests.

[Native diagnostic observations](diagnostic-v1/result.json) use a source copy
and the exact Cargo dependency graph. The 8/24 cases certify both empty union
and subtraction by the body; 16/32 fail both. Empty union stops in coplanar
overlap classification during a new arrangement. Direct subtraction stops in
deferred-span comparison. A diagnostic exit of zero records these outcomes and
does not mean the operations succeeded.

[Independent conditional expansion](composition-algebra-v3.json) gives zero
numerators for all four captured failures. After reuse of structurally identical
subexpressions and common denominators, it encounters intermediate degrees 9
and 10, above the production degree cap of 8. There are three opaque atoms and
one distinct denominator obligation, `144*(sin(alpha)-sin(beta))`. This proves
an algebraic identity conditional on nonzero denominators, not a production
certificate. It identifies a degree obstacle, but does not instrument which
production limit declines first. Next confirm that cause and investigate exact
factor cancellation or retained construction evidence without increasing limits
or bypassing operand validation. Preserve the failing full composition tests.

All closed trials remain intact. `context-baseline-v1` failed to compile because
the direct test command omitted Hyperlattice; v2 pins that dependency and
reproduces the obsolete failure expectation. `model-acceptance-v1` caught two
unsupported usize conversions and a misspelled `ZeroKnowledge` variant in the
new test; v2 corrects only those API uses and the direct test arguments, then
exposes the composition failure. V3 splits the same composition assertions into
four named cases so one failure cannot prevent the other resolutions running.
`analysis-v1` exhausts the independent expander's 4096-visit cap. V2 reuses
completed structural expansions; v3 also shares common denominators and records
degree/term/product observations. No production limit changed for these trials.

This remains a candidate, with broad downstream qualification, matched cost and
allocation evidence, final retention, full composition and release/user-path
validation open. The 35 pending native curve/DRC files remain byte-identical and
unadmitted. Full CI/pre-push, bindings, MSRV, sanitizer, formal verification and
the complete properties plan remain open. No push was performed.
