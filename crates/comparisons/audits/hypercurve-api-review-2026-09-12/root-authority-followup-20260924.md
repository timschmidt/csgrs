# Selected root authority follow-up (read-only design)

The rational-polynomial work demonstrates two different reuse needs. Fresh
wide evaluations benefit from integer accumulation, while repeating exactly
the same argument can favor retained scalar arithmetic. A global polynomial
cache in Real::eval_poly would put query identity in the wrong owner. The
selected root or field already knows the fixed defining relation, branch and
refinement history and is the appropriate owner for reusable preparation.

Current concrete costs: AlgebraicRootRepresentation exposes a Vec<Real>, an
IsolatedRootInterval and a structural validation status. Hypercurve's
refined_represented_root clones that full vector, refines from the supplied
interval with STRICT policy, then repeats structural validation. Shared-source
bounds now does this once per source per query, but it does not retain progress
between queries. Native Bezier algebraic parameters already retain their Sturm
sequence through shared data; generic represented roots lack the same authority.

The next coherent migration should replace the publicly forgeable combination
with a private shared selected-root definition and construction certificate.
Isolation, projection and validated transforms should issue it once. A refined
enclosure must preserve that definition and selected branch without replaying
validation or copying coefficients. Cache only certified exact refinement and
preparation; a policy-dependent uncertainty must never become cached geometric
truth. Preserve point witnesses with arbitrary exact Real payloads, not only
rational payloads. Keep the distinction between an owned half-open isolator and
an exact point witness, including repeated roots and excluded endpoint roots.

This requires migrating constructors and all callers directly, not adding an
alternate root wrapper beside the old public fields. Existing geometric
existence/uniqueness proofs in Hypercurve must have a solver-owned certificate
route; replacing every such proof with a fresh Sturm construction would discard
the local performance work. Explicit rational reconstruction guards remain
necessary only for theorems that consume genuine rational coefficients or
coordinates. Root/fiber arithmetic belongs in Hypersolve, geometric incidence
and source-domain ownership remain in Hypercurve, and exact scalar values and
intrinsic arithmetic facts remain in Hyperreal. No source is changed here.

Additional migration constraints from the current callers: root equality is
used to match tensor source axes and to recognize an existing selected branch.
Certified refinement must not change that selection identity. Keep mutable
refinement/preparation outside equality and bounded diagnostics. A cheap shared
selection check is distinct from mathematical equality of roots from different
definitions; the latter still requires the existing exact comparison authority.
Changing every equality to a forced scalar/compositum comparison would undo
the intended scheduling improvement.

One concrete geometric certificate producer is the represented-coordinate
represented_univariate_coordinate path in bezier_offset.rs. It combines a
retained point's existence proof with a strictly signed derivative enclosure
to prove that an image eliminant has exactly one selected coordinate root in
the box. Exact point intervals first prove the defining residual is zero.
Its current public struct construction and structural validation must be
replaced by a solver-issued certificate route that preserves this local proof.
Simply wrapping an arbitrary public polynomial/count/interval tuple in Arc
would share allocations but would not establish evidentiary closure.

The retained-coordinate V4 candidate demonstrates another authority boundary:
an older exact point can retain its Hypersolve root and coordinate polynomials
without a BezierAlgebraicParameter2 wrapper. The root identity proof must consume
that retained solver evidence directly. Moving reuse into polynomial signs
resolves both materialized and selected closed-cubic trim-only chamfers in the
focused run; full qualification is pending. The private helper returns the
proved embedding so comparison and predicate replay share one certificate.

A related existing scheduling detail deserves attention during source-identity
normalization: recursive_quadratic_source_union canonicalizes and deduplicates
both input lists, while the native comparison's membership shortcut currently
compares the resulting count with the raw base count. A base containing sources
that collapse during canonicalization can make that count an imperfect test
of pre-existing axis membership. The residual and singleton interval still
prove any returned equality, but matching axis maps is a more precise work
bound. The new sign replay only proposes roots drawn directly from the base,
so it never proposes an unrelated generator.

## Native chart identity at a coefficient-root comparison

Read-only follow-up: BezierAlgebraicParameter2 already retains a composed
projective_source and exposes projective_map_from(other), including independently
issued nested singleton certificates. Recursive coefficient_root_embedding
accepts only AlgebraicRootRepresentation, which erases that chart provenance.
An affine image of an existing coefficient root may therefore miss generator
identity even though its original root is present. The current guard also
compares deduplicated union length with raw base length; membership should use
the returned axis maps when bases contain duplicates.

If further profiling demonstrates this case, consume the native parameter's
existing chart certificate before converting it to a bare root tuple. An affine
map into an existing coefficient axis can evaluate through that same field. A
general Mobius map additionally requires a nonzero denominator and correctly
oriented homogeneous sign replay. Neither overlapping intervals nor an affine
polynomial resemblance establishes selection identity. This is a hypothesis
from caller/source inspection, not yet a measured quartic diagnosis or an
implemented change.

## Refinement ownership at the represented-root boundary

Read-only review during V18 finds `refined_represented_root` in Hypercurve rebuilding refinement from `AlgebraicRootRepresentation.polynomial_coefficients` and `.interval`, then reconstructing and validating another public tuple. Repeated calls from coefficient interval evaluation do not carry the native Bezier shared Sturm/chart authority through this boundary. This is a concrete ownership boundary for the existing Hypersolve root-certificate extraction, not justification for a new global cache or an Arc wrapper around still-forgeable public fields. A shared selected-root authority should retain its polynomial preparation, unique root selection and compatible refinements together. The exact local authority must remain valid for nonrational Real coefficients and for finite projective charts; overlapping brackets alone are not identity evidence. The V15 sample showed tensor interval arithmetic, so this follow-up is still source-inspection evidence rather than a measured claim that root re-preparation dominates the current candidate.

## V34 measured preparation and boundary audit

The V34 50-second quartic stack directly sampled represented-root refinement inside local ordered-field Bernstein sign queries. A second GDB diagnostic counted exactly two public chamfer entries (first TrimOnly then first TrimOrExtend) and sampled primitive integer Sturm construction in that same refinement helper. Both owned probes reaped and preserved all source/executable hashes. This establishes repeated preparation on the first extension workload; it does not measure a whole-run attribution percentage.

Inspection of the reusable UnivariateSturmSequence also found a potential half-open endpoint defect for repeated polynomials: every raw sequence member shares the terminal GCD and can vanish at a repeated root, so skipping zero members loses the limiting variation count. The independent counterexample is (x²-1)² over (-2,-1], whose distinct root count must be one. A V35 regression tests all integer endpoint pairs in [-2,2], symmetric/asymmetric multiplicities, signed rational and irrational gauges, both policies, root classification, and preservation of the original GCD evidence. It is run against the existing implementation before changing the authority.

V35 confirms the defect: both Clippy configurations passed, then the new test failed at (-2,-1], returning Some(2) instead of Some(1). Exact outer handle 48721 reaped with exit 1. V36 corrects common-root evaluation by taking the first nonzero right-hand Taylor sign of each chain member only when every original member vanishes. This preserves half-open endpoint ownership without changing the chain or its terminal GCD. Ordinary point evaluation keeps the previous path. General interval refinement now constructs one chain on the original polynomial, reuses its terminal GCD to obtain the square-free bisection polynomial by checked exact division, and retains all witness/count/bracket validation. Qualification pending.
