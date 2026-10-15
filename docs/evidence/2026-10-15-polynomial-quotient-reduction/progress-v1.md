# Polynomial quotient recovery candidate

The previous goal turn made progress: zero-quotient normalization restored
certified empty accumulation at every original drill resolution. This change
extends that work in Hyperreal, the owner of scalar polynomial identities.
It addresses P2–P5, P7–P9 and the downstream composition and completion
requirements without changing policies, fixtures or recognition limits.

After ordinary quotient arithmetic declines, the recognizer can divide a
completed child's numerator by its complete denominator. Exact multivariate
division returns a polynomial only when the remainder is zero. Every divisor
has already passed the existing strict nonzero-domain guard. Arithmetic is
retried using the same atom identities and remaining work budget. Successful
existing expansions do not perform this factor search. This introduces no
persistent cache, additional recognition attempt or authoritative tree rewrite.

Division charges leading-term scans, monomial comparisons, coefficient
arithmetic and insertions. The existing node, depth, atom, term, degree,
coefficient and work limits remain unchanged. Tests cover negative divisors,
strict witness warming, tiny signed remainders and displacements, serialization,
cached depth and node checks, oversized coefficients, work exhaustion,
cancellation and recovery. External per-process address-space and wall limits
remain containment, not an internal bound on complete-query work or heap bytes.

`factor-candidate-v1` failed to compile an added coefficient-limit test because
it used an unsupported `Rational::from(BigInt)` conversion. The corrected
`factor-candidate-v2` passes all 800 scalar unit tests, both new public tests,
the seven preceding quotient tests and four context tests. Its independent
polygonal membership, volume-enclosure and topology checks retain all original
8/16/24/32 fixtures and pass. The full composition tests now pass for 8, 16 and
24 segments. The unchanged 32-segment test still fails at complete subtraction
with typed `PredicateUndecided`; it is not admitted as completed geometry.

`factor-qualification-v1` passes all six commands: workspace formatting,
919 default-feature and 1,026 all-feature scalar tests, 28 all-feature doctests,
scalar all-feature/all-target Clippy and root curve/all-target Clippy, with
warnings denied. Counts overlap with the direct tests. Sources stayed frozen
through each run. The lost temporary Cargo cache was replaced by the existing
offline cache at `/home/tim/.cargo`; the new mutable target uses independent
inodes copied from a frozen seed.

The fresh copied-source `diagnostic-v3` confirms certified full 8/16/24 chains
and the remaining 32-segment failure. Both direct and chained subtraction now
reach a different coplanar clipping predicate: the 64-term limit is hit while
459 node visits and 7,356 work charges remain. There are four opaque sine
atoms and no inverse operands. Independent rational polynomial expansion
(`analysis-v2`, `remaining-algebra-v1.json`) proves the captured expression
identically zero but materializes 145 terms of degree eight. Exact polynomial
division does not help this purely polynomial expression.

The captured expression has common product factors. In the recorded DAG,
node 38 equals twelve times node 36; node 47 is their opposite scaled products
with common factors, and is zero. The next implementation investigation is
bounded common-factor cancellation before multiplying the expanded products.
It must preserve all operand-domain, depth, node, degree and work obligations
and the original full-scale 32-segment test. This algebraic observation is not
yet a native proof or a completed workflow.

The closed-artifact consolidation verifies 3,275 library paths and replaces
2,864 duplicate copies with links to identical closed non-executable files.
All recorded bytes and paths survive; no unique artifact or executable was
removed. The journal records inode and metadata changes. This recovered about
3.6 GB for new runtime targets. These targets remain closed to future builds;
any mutable seed must again use independent inodes.

The external `/tmp` loss described in `baseline-v1.md` remains unresolved.
The closure audit keeps every inherited expected hash, separates verified
survivors from missing artifacts, and includes the lost diagnostic source
copies. It does not claim complete historical preservation. No recovery copy
has been made. The compact artifact audit is compressed without dropping
records. Both the historical loss and the failed oversized recovery inventory
remain in the evidence.

This is a tested capability candidate, not full admission. Broad downstream
qualification, matched cost/allocation/artifact comparisons, complete external
consumer and release validation, and the 35 pending native Hypercurve/HyperDRC
files remain open. No speedup, bounded whole-operation memory, full CI success,
or completion of the P1–P39 plan is claimed. Nothing has been pushed.
