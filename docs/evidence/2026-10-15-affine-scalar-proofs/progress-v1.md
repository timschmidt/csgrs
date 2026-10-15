# Polynomial proof progress; full admission remains open

Baseline `c35179831` records four failing public proof regressions and the
passing unsupported-trigonometric control. The first baseline compilation
mistakenly used an `i64` denominator literal for `Rational::fraction`; the
corrected `u64` has the same value. Both trials remain preserved.

The candidate adds a shared Hyperreal rational-polynomial projection after
the existing successful Laurent path. The projection treats roots, transcendental
values and inverse expressions as opaque atoms. Structural equality is the only
atom equivalence. The proof uses rational ring arithmetic and publishes a sign
only after every nonconstant monomial cancels. It adds no native tolerance,
policy substitution, expression rewrite or persistent proof-payload cache.

Expansion permits 512 node visits, depth 64, 32 atoms, 64 monomials, degree 8,
4096 stored coefficient bits and 8192 charged operations. Temporary coefficient
arithmetic can exceed the stored coefficient limit before rejection. The charges
count operations, not all work in the existing structural DAG matcher. Neither
this fallback nor the earlier normalizer establishes a whole-query time or
heap-byte bound. The new attempt checks cancellation and does not cache declines.

The final-source [scalar-and-drill-v2 trial](scalar-and-drill-v2/result.json)
passes all 15 commands: workspace formatting, 895 default scalar results,
1001 all-feature scalar results, all-target/all-feature scalar Clippy with
warnings denied, the CSGRS curve library build, all eight original drilling
observations and 28 scalar doctests with warnings denied. Test counts overlap
across configurations. The probe exit status means that its observations were
recorded, not that the Boolean operation completed strictly.

Tests include independent distributivity and affine identities, exact positive
and negative `1/10^12` displacements, preserved unsupported identities, proof
reuse across precision changes and concurrent readers, unchanged authoritative
serialization, separate recognition limits, cancellation and subsequent recovery.
The first broad scalar trial passed 784 library tests and failed a new test that
incorrectly assumed the process-wide pi cache was empty. The correction retains
the pi resource case and verifies no approximation on a fresh private sine node.
No production code, original fixture, limit or mathematical assertion changed
for that retry. Mutable seeds have independent inodes, including internal aliases.

The full unchanged drilling matrix preserves the certified 8/24-segment cases
and qualified approximate 16/32-segment cases. The 16/32 strict failures advance
from plane coplanarity to typed `PredicateUndecided("3D coordinate equality")`
during arrangement-point interning. They are still failed strict workflows;
volume and composition acceptance are still required. The native diagnostic
uses a source copy and the exact Cargo dependency fingerprints; live native
production files are unchanged.

[Independent expansion](coordinate-algebra-v1.json) covers the four captured
coordinate differences: each has 303 arithmetic visits, three opaque atoms and
two inverse occurrences. Each resulting numerator is exactly zero. Each
denominator obligation normalizes to `144*(sin(alpha)-sin(beta))`, where the
captured angles satisfy `0 < beta < alpha < pi/2`. This is nonzero mathematically,
but the diagnostic is conditional algebra, not a production nonzero certificate.
No trigonometric equality is needed for the numerator cancellation.

The next implementation must handle quotient identities only with admissible
nonzero-denominator evidence, preserve the already successful polynomial and
Laurent domains, and add zero/unknown-denominator and near-nonzero controls.
It must then rerun the original full workloads and semantic/topology checks.
Do not add a point-interner-specific formula or infer equality from refinement
failure. This scalar candidate is committed as work in progress; broad downstream
qualification, matched completed-contract costs and final retention are pending.
The 35 preexisting native curve/DRC files remain byte-identical and unadmitted.
Full CI/pre-push, release, bindings, MSRV, sanitizer, formal verification and the
complete properties plan remain open. No push was performed.
