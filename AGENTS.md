# Repository Agent Rules

## Change Acceptance

Apply the acceptance contract in [`properties.md`](properties.md) and the
normative protocol in [`docs/admission.md`](docs/admission.md) to every change.

Before changing code, identify the owning layer, the relevant property, its
input domain, and a failing or missing acceptance test. Reproduce review
findings on the target revision; do not assume an earlier assessment is still
current.

Make the smallest coherent change that restores or extends the contract.
Preserve exact meaning, supported composition, provenance, certainty, and
explicit failure. Do not weaken tests, policies, fixtures, or validation to
obtain a passing result.

After the final edit, run the affected local and downstream checks. Record
what was executed, what passed, what remains blocked, and the evidence
supporting any correctness or performance claim. Updating a specification or
ledger does not establish that the implementation satisfies it.

## Hypercurve: exact curve and operation closure

This section defines the required support envelope for Hypercurve and its
consumers. It refines P12–P20 in [`properties.md`](properties.md) and the
[`admission protocol`](docs/admission.md). It is a target contract, not a claim
that every requirement is already implemented. Reproduce capability claims on
the revision being changed. Record current gaps and measurements in the
capability records and work ledgers, rather than weakening this contract or
embedding a transient implementation inventory here.

**A generated exact curve is a first-class geometric object. An operation is
not complete merely because its result can be constructed, evaluated, or drawn:
the next applicable operation must be able to consume that result without
changing its meaning, losing required evidence, or escaping the declared
resource contract.**

### Required domain and operations

The required complete mathematical domain is finite descriptions of planar
algebraic curves and bounded regularized regions with finitely many boundary
pieces, using rational or real-algebraic geometric data. This includes arbitrary
finite polynomial degree and algebraic coefficients; rational coordinates,
radical formulas, and small algebraic degree must not be prerequisites.
Existing supported symbolic inputs remain supported. Broader symbolic or
transcendental inputs require explicit decision guarantees; the algebraic
contract does not imply universally decidable arithmetic for arbitrary
computable real expressions.

The operation contract includes:

- Regularized union, intersection, difference, and symmetric difference.
- Constant-distance Euclidean offsets, including explicitly specified disk
  dilation/erosion and round, bevel, or miter boundary conventions. These
  conventions must not be silently treated as equivalent.
- Constant-radius circular fillets and straight chamfers with Euclidean chord
  setbacks from the original corner. State trimming, extension, side, and
  solution-selection rules.
- Finite compositions, including exact restriction, subdivision, reversal,
  supported transforms, boundary reconstruction, and native exact interchange
  wherever these occur between modeling operations.

All applicable operations must accept authored and generated geometry, mixed
families, algebraic endpoints and centers, holes, disconnected components, and
canonical empty results. Mathematical preconditions still apply: a specified
fillet need not exist or be unique. Selection among solutions must have an
explicit exact contract. Within the algebraic support envelope, transform
coefficients and selection constraints must be algebraic or semialgebraic over
the admitted data. Other choices require a separately declared scalar and
decision contract.

`CurveRegion2` remains the sole filled-region carrier. Specializations must
share its region semantics and arrangement authority. Fix scalar, predicate,
incidence, and topology invariants in their lowest sufficient owning layers;
adapters must not create a second geometry kernel.

### Minimum sufficient functional representations

Distinguish geometric families from storage variants. Minimize duplicated
mathematical authority and avoidable growth, not merely the number of enum
variants. There is no uniquely smallest type vocabulary or mathematically
proved universally fastest representation.

For rational seeds and the named operations, with similarities where transforms
are interleaved, the sufficient compact basis is:

1. Rational source spans, including line, circle, polynomial/rational Bézier,
   polynomial B-spline, and NURBS spans.
2. Exact signed normal parallels of those spans.
3. Exact selected points, parameter domains, traversal, and finite boundary
   topology needed to select and assemble the retained pieces.

Here, *rational* means rational functions of the parameter. Coefficients and
trim endpoints may be real algebraic. Lines and circles are rational
specializations; splines supply finitely many rational spans. A whole circle
may require several finite rational charts.

For arbitrary algebraic seeds, the sufficient general carrier is a selected
real algebraic arc: a defining polynomial or algebraic system, the intended
real branch, a valid chart and domain, exact endpoints, traversal, and
singular-event incidence. At the filled-set level the closed class is
semialgebraic regions. A polynomial equation alone is not a selected curve.

Exact points, centers, controls, and coefficients may require field extensions
or jointly selected solution fibers. Preserve their defining relations,
isolators/selectors, and shared dependencies; independently materialized scalar
coordinates are not a prerequisite for consuming an exact construction.

General offsets can be nonrational. Polynomial curves, NURBS, and fixed-degree
rational families are not universal exact fallbacks. Do not force a generated
curve into an authored family by fitting, sampling, rounding, or dropping its
branch restrictions. A finite catalog of named low-degree curves is not the
complete support envelope.

### Minimum practical working set

Maintain an efficient implementation of the following functional roles. This
table specifies capabilities and preserved structure, not mandatory public
types or one implementation per row.

| Working representation | Required purpose |
| --- | --- |
| Exact lines and circular arcs | Cheap primitive predicates and constructions, including generated objects with algebraic endpoints or centers |
| Polynomial and rational spans | Preserve low-degree structure where useful; support arbitrary finite-degree spans and exact spline decomposition |
| Source-preserving normal parallels | Represent offsets compactly and reuse source domains and certified distance composition |
| Selected algebraic branches and systems | Cover nonrational and arbitrary implicit geometry without mandatory rational conversion |
| Shared restrictions, selected points, and incidence records | Preserve exact trims, correlated coordinates, contacts, and reusable proofs across all families |

Keep useful native specializations and compact procedural definitions. Do not
make implicitization, high-degree eliminants, or coordinate materialization
the default when an exact source-preserving construction suffices. Conversely,
special-case recognizers must not become a substitute for completing the
general admitted domain.

The "minimum performant set" is the smallest coherent implementation set that
meets the declared completed-workload envelope with evidence. Add, merge, or
remove specializations only with correctness, composition, and cost evidence.
An alternative to the baseline above is acceptable when it preserves those
contracts and demonstrates comparable or better practical behavior. Native and
general paths must agree on geometry, topology, certainty, and provenance.

### Exhaustive generation rules

| Operation | Permitted new supporting curves |
| --- | --- |
| Regularized Boolean | None beyond operand supports; select, split, trim, reverse, and assemble their subarcs |
| Constant offset | Normal parallels; circles for round joins or point features; lines for bevel or bounded-miter connectors |
| Circular fillet | A circle, with retained portions of the incident supports |
| Straight chamfer | A line, with retained portions of the incident supports |
| Finite composition | Reapplication of these rules, retaining exact definitions and selections |

Trimming may increase endpoint or coefficient complexity without changing the
supporting family. A generated chord is still a line; a fillet with a
high-degree algebraic center is still a circle. Do not confuse support degree,
parameter-root degree, coefficient-field degree, expression size, or number of
boundary pieces. Each can affect cost differently.

### Exactness, identity, and topology invariants

Every operation must preserve, transport with proof, or re-establish:

- The exact represented image, selected branch, and domain, including endpoint
  inclusion and traversal where relevant.
- Point-on-support relations, endpoint identity, contact and tangency
  relations, overlap correspondences, and parameter/chart mappings needed by
  subsequent operations.
- Source and selected-root/fiber identities, including the base selections on
  which dependent coordinates rely.
- Connectivity, winding, boundary ownership, material side, and the declared
  fill rule and regularization semantics.
- Numerical certainty, topological validity, and fidelity to the intended
  source as separate claims. Preserve operation-local certainty and inherited
  qualification.

Distinguish structural identity, equality of scalar values, equality of curve
images, parameter correspondence, and equality of filled sets. Allocation
identity may accelerate a proof; it must not define mathematical equality.
Mutation or a changed transform must invalidate affected evidence unless
certified proof transport makes its reuse valid.

Do not discard known construction relations and force later operations to
rediscover them through rounded coordinates or repeated global elimination.
Do not merge nearby but distinct features by tolerance. Candidate roots from
squaring or elimination must be checked against original equations, source
domains, nonzero denominators, selected branches, and normal-side conditions.

Material and holes must follow the arrangement and declared winding/fill
semantics. Simplifying a boundary must preserve the selected set and all
required incidence and multiplicity information. Matching endpoints or a
geometric fit is insufficient evidence of equivalent oriented arcs.

### Repeated offsets, singularities, and transforms

Normalize repeated constant normal offsets when certified. For an arc-length
parameterized source with left normal `N`, use `dT/ds = k N` and
`dN/ds = -k T`. On a regular connected interval:

```text
C_d = C + d N
dC_d/ds = (1 - d k) T
epsilon = sign(1 - d k)
Offset_e(Offset_d(C)) = Offset_(d + epsilon e)(C)
```

Split at singularities and orientation changes, account for traversal
reversal, and classify wholly collapsed spans. This is a local support
identity. It does not imply that erosion reverses dilation, that removed
features reappear, or that intermediate region regularization can be skipped.
At zero distance, retain the source without unnecessarily requiring a normal.

Handle poles, removable factors, stationary parameters, cusp events, singular
branches, tangencies, coincident supports, and partial overlaps explicitly.
Detect removable rational factors before classifying a genuine pole. Algebraic
cancellation must preserve the authored domain and original denominator guards.
Admitting an excluded parameter requires an explicit extension contract and
proof of the finite branch limit. Preserve one-sided branch limits or
alternative charts where regular formulas fail. Repeated implicit factors may
share a geometric zero set while carrying different multiplicity information;
do not discard that information indiscriminately.

Not every solution set is a finite list of isolated points. Represent overlaps
as correspondences, and handle multiple or continuous fillet families using
explicit selection constraints or an honest underconstrained outcome.
Lower-dimensional remnants follow the region's regularization contract;
successful empty geometry remains a valid subsequent operand.

Similarities preserve the compact rational-source/normal-parallel basis, with
correct scale and orientation handling. Nonsingular affine transforms with
algebraic coefficients preserve the declared algebraic domain but do not
generally commute with Euclidean offsets. Interleaving nonuniform affine maps
and offsets may require a broader algebraic carrier or an exact retained
transformed definition. Singular maps require separate collapse semantics.
Never apply an offset/transform commutation rule without its mathematical
preconditions.

### Resource and performance invariants

Use the governing order: exactness > completeness > performance > memory >
binary size > source size. Preserve compact shared source and proof structures;
avoid duplicating a construction graph into an expression tree. Measure
unavoidable algebraic complexity separately from avoidable expansion,
reconstruction, repeated promotion, and discarded proof work.

Resource contracts must distinguish precision, total work, coefficient growth,
expression/system size, temporary and retained memory, and cancellation.
Account across nested calls and workers where applicable. A precision limit,
refinement counter, or cache-entry count alone is not a total resource bound.

Exhaustion must be typed before publishing a completed result. Provisional
partial output requires a separate explicit contract. Cancellation must clean
up owned temporary state. An exhausted attempt must not poison a later
larger-budget retry or become permanent evidence of undecidability.

No uniform practical bound is promised for arbitrary algebraic degree,
coefficient complexity, or operation depth. Publish a measured ordinary
completion envelope and a stress/resource envelope. Configured degree or work
limits are resource or implementation restrictions, not mathematical
completeness bounds.

### Exceptions and prohibited shortcuts

Distinguish successful geometry, successful empty geometry, proved no solution,
invalid or inapplicable input, ambiguous/underconstrained construction,
unsupported representation, undecided predicate, exhausted resources, and
unbounded output. Do not replace any failure with zero coordinates, a false
classification, an unchanged input, or an incomplete success. Resource
exhaustion is valid only when a declared resource limit is actually reached.

Within the target domain, a missing algorithm, unsupported representation, or
lost relation is an implementation gap. A typed blocker is honest reporting,
not evidence of completed support. Do not narrow a previously supported domain,
relabel a blocker, or weaken a fixture to present a regression as compliance.
Intentional contract changes require the repository's explicit migration
process.

True arc-length setbacks, variable-radius or general curvature-continuous
blends, transcendental source curves, and other operations outside the declared
algebraic grammar require their own closure and decision contracts. Do not
silently interpret them as chord setbacks, circular fillets, or algebraic
special cases. Preserve any existing supported behavior while extending the
contract.

Geometric approximation or demotion of authoritative data requires an
explicitly requested, documented approximation contract with its loss model
and claimed error/topology guarantees. Certified internal numerical filters and
enclosures remain permitted when their bounds justify consumed decisions and
the exact object remains authoritative. A policy-authorized approximate
terminal must retain its certainty receipt and cannot count as exact success.

Display chords, tessellation, sampled paths, and quantized interchange must
never substitute for the original exact source in an exact decision. If such
output is explicitly adopted as a new model, subsequent exactness is relative
to that model; approximation provenance and loss of source fidelity remain
explicit. Exact native replay must retain and validate defining relations and
selections; serialized flags and live caches are not proof. Local validation
after lossy reimport does not certify preservation of the original source.

### Admission and regression obligations

Before changing a family, representation, operation, predicate, conversion, or
consumer, identify the owning layer, affected closure relationships, and a
failing or missing acceptance witness. Maintain a capability matrix separating
representation, evaluation/differentials, incidence/order, region admission,
Booleans, repeated offsets, corner edits, reconstruction, and exact replay.
Keep target coverage and demonstrated implementation coverage distinct.

Exercise output as subsequent input. Relevant changes must cover mixed authored
and generated families and finite chains of Boolean, offset, fillet/chamfer,
reconstruction, supported transform, native save/reload, and further operation.
Do not require every unrelated change to rerun the entire matrix; apply the
existing admission and downstream-validation rules to the affected contract.

The maintained corpus must cover:

- Algebraically trimmed rational spans; generated lines and circles with
  selected algebraic endpoints or centers.
- Nonrational ellipse offsets, rationally reparameterizable parabola offsets,
  Pythagorean-hodograph specializations, and offsets with cusp/normal-reversal
  events.
- Coincident, partially overlapping, reversed, tangent, near-coincident but
  distinct, and singular geometry.
- Holes, nested/disconnected components, disappearing features, canonical
  empty-set identities, and invalid inputs paired with empty operands.
- Implicit branches with vertical, repeated, and boundary components;
  algebraic coefficients; nodes, cusps, and isolated events.
- Fresh-process exact replay, invalid selectors/source references, affected
  cache and policy transitions, small budgets, cancellation, and retry.

Use independent exact expectations, defining relations, topology invariants,
and representation-aware generators. Area, sampled membership, checksums, or
agreement between wrappers of one implementation cannot alone certify the
intended set. Verify semantic equality separately from deterministic
serialization. Preserve original hard cases alongside minimized regressions.

Conformance fixtures must achieve the advertised exact success within their
published resource envelope. Controlled exhaustion validates resource handling,
not geometric completion of those fixtures. Report certified completion,
qualified approximation, undecided, unsupported, invalid, exhausted, and
incorrect outcomes separately. Performance claims compare the same completed
contract, including required output materialization and evidence.

A finite test suite is not a universal closure proof. Support claims require
the declared domain, a sufficient representation and closure argument,
algorithms covering that domain and its degeneracies, implementation review,
and a maintained qualification corpus. Documentation alone does not establish
implementation completeness.

### Mathematical rationale

- [Sendra, *Brief Atlas of Offset Curves*](https://dialnet.unirioja.es/descarga/articulo/3217873.pdf): algebraic offset constructions and nonrational offsets.
- Patrikalakis, Maekawa, and Cho, *Shape Interrogation*: [differential behavior](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node219.html), [singularities](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node220.html), and [rational-offset limitations and exceptions](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node222.html).
- [Basu, *Algorithms in Real Algebraic Geometry: A Survey*](https://www.math.purdue.edu/~sbasu/raag_survey2011_final.pdf): semialgebraic closure and effective quantifier elimination.
- [Yap, *Theory of Real Computation according to EGC*](https://cs.nyu.edu/exact/doc/realtheory.pdf): limits of unrestricted exact decisions over computable reals.

## Pre-Push Verification

Before every `git push`, agents must:

1. Run `cargo fmt --all -- --check`.
2. Run Cargo Clippy with `-D warnings`, including every feature, target, and manifest combination exercised by this repository's GitHub Actions workflows.
3. Run every locally executable validation command from every file in `.github/workflows/`, including tests, checks, docs, benches, fuzz-target checks, release builds, and WASM/demo builds where configured.
4. Whenever CI configuration changes (`.github/workflows/**`, `.github/actions/**`, `.github/dependabot.yml`, or pre-commit configuration), run `zizmor --pedantic --strict-collection .` from the repository root and fix every finding.
5. Fix every failure or warning caused by the pending changes. Repeat the complete validation set until it passes.
6. Commit all intended fixes before pushing, then confirm the worktree contains no uncommitted changes from the task.

A push is not allowed while any required check is failing. Hosted deployment actions, secret-dependent publication, and packaging for operating systems unavailable on the current host may be excluded only when they cannot be reproduced locally; inspect those workflow steps for consistency and report the limitation explicitly.
