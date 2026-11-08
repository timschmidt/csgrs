# Selected polynomial guard components — 8 November 2026

This extension starts at `8040afd30ac7787499ae07383fd49a6614860e8d`.
Hypersolve owns checked common factors and root isolation; Hypercurve owns the
selected source branch, query guards and complete parameter loci. The affected
contracts are P12, P14–P20 and P35–P39. The algebraic closure target is unchanged.

The original selected graph `y=sqrt(x)`, `x in [0,1]`, queried through
`[2D,3D]/D` with `D=2x-1`, requires exactly `[0,1/2) union (1/2,1]`.
An independent selected query reproduced typed `Unsupported` on the parent.
The initial interval-box guard proof could not represent this punctured domain.
The original hard probe now completes with both pole endpoints excluded.

The existing zero-tangent-derivative identity proves a constant rational image
on the connected regular source graph wherever the extra map is defined.
Analytic continuation prevents different constants across isolated removable
poles; the authored guard still excludes those parameters. A complete guard
census supplies the domain, and an exact interior witness identifies the
constant. Neither a point sample alone nor denominator cancellation proves
the represented set.

The guard is pulled back into the original source coordinates. The shared
`bivariate_topology::bivariate_polynomial_gcd` exposes the existing checked
rational-coefficient factor reducer. A checked simple-root slab and exact
factor selection distinguish a selected common component from an unrelated
one. Isolated candidates from the residual support are checked against the
original selected branch, active trim, guard and pullback weight. The source
equation, defining multiplicities, root selections and original map definitions
remain authoritative. No fitted curve or second incidence kernel is introduced.

A nonzero guard polynomial vanishing on the whole selected source gives an
empty defined query domain. A globally zero denominator remains invalid.
Otherwise its exact zeros split the active interval into components with
explicit endpoint inclusion. `point_at_component` validates span zero, the
identity source chart and current membership, then evaluates the original
branch. Parameter metadata alone does not establish another query's incidence.

The maintained witnesses distinguish the following cases:

| Input | Independent expectation and continuation |
| --- | --- |
| Selected square-root graph, canceled interior pole | Two exact open-ended intervals; membership, component evaluation, intersection, reversal and trimming |
| Ordinate, mixed-coordinate and repeated guards | Punctures at `1/4`, the selected unit root of `t^2-3t+1`, both original endpoints, and one repeated zero at `1/2` |
| `(y^2-x)(y-2)`, selected positive square-root branch | Unselected common factor, a proper puncture, empty selected guard domain and invalid global zero denominator remain distinct |
| Degree-nine graph `y^9=x` | Guard `2y-1` excludes exactly `x=1/512`; actual zero work allowance fails and a larger retry succeeds |
| Constant selected ordinates from cubic and quintic fields | Independent irrational queries need no stored scalar coordinates; generated source points feed another exact location query; the negative ordinate is absent |
| Guarded identity, shear, reversal and exact trim | Fresh-process native SVG replay recomputes the locus, consumes a selected component point and performs another location/intersection; deterministic bytes are checked separately |

The original selector satisfies
`t^5+t-1 = (t^2-t+1)(t^3+t^2-1)`. Its quadratic factor has no real roots;
the cubic has no rational root, so the selected real value has field degree
three. Wording in the initial local evidence that described a degree-five
constant overstated this degree; the degree-five polynomial selection and all
geometric results remain valid. That record is preserved unchanged.

The regression retains the original case and adds
`f(t)=-1+2(t+t^2+t^3+t^4+t^5)`. Its reciprocal
`2+2s+2s^2+2s^3+2s^4-s^5` is Eisenstein at two, proving irreducibility and
field degree five. Its positive derivative and opposite endpoint signs give
one root in `[0,1]`. Both independent selections and the consuming chain are
tested without materializing that root as a scalar coordinate.

The functional source freeze contains 3,810 inputs and patch SHA-256
`b89ff705b269328ecd713f3334457141a0f99b4aa870a48bea538ec88a6c7606`.
Its full owner run records 1,368 successful Hypersolve harness reports with no
ignores and 3,185 Hypercurve reports with 37 existing ignores. Counts describe
harness reports, not distinct workflows. The later irrational-value regression
changes only its test module; all production bytes are identical. The affected
target then passes all 20 location tests, three GCD tests and strict Clippy for
Hypersolve, Hypercurve, BRep and BRep IO.

The downstream native BRep/IO command records 996 successful harness reports
and no ignores. Strict documentation builds for all four crates pass. Its
wrapper later fails an additional all-features WASM query that activates
Hypercurve's third-party comparison benchmarks. The identical query fails on
the unchanged parent through `curvo -> rand -> getrandom`; that broader feature
combination remains unqualified. This failure is retained separately from the
successful native and documentation commands. The combined production
Hypercurve features (`svg`, `triangulation`, `hershey`, `dispatch-trace`) and
the BRep/IO all-features WASM combination pass on Rust 1.99.1 and 1.95.0.
Native MSRV tests and strict linting also pass.

The initial five-function actual WASM probe executes the maintained pure acceptance
functions in three fresh module memories, with the default stack and no host
imports. All original assertions pass, including exact selected points,
component endpoints, shared factors, degree-nine input, budget/retry and the
independent irrational-value chain. The 12,017,011-byte module has SHA-256
`b634ca42e96b6b3e6dca6d1654863c641a48b363f56f96239de10b28477b5ff1`.
The native process-spawning SVG witness is exercised separately. This private
transport does not qualify a public ABI or universal native/WASM interchange.

After the degree correction, the same fifth pure function retains the cubic
case and also exercises the irreducible quintic and its consuming chain.
Native and Rust 1.95 location tests, strict linting, all 423 reviewed witnesses
and the actual WASM run pass with both selectors. The expanded 12,018,430-byte
module has SHA-256
`3b2d3a6482bbaf4498a1082fa0a4f5b1bfae9c32383f2927fb76b8d7e6d3dfcb`.
Production code, manifests and CI are unchanged from `2fd417079b`.

Fourteen unchanged rational-map acceptance functions complete on both
revisions under both policies. Median allocation calls and requested bytes match.
Three alternating fresh
process samples per policy give descriptive time ratios from 0.9464 to 1.0146.
Other validation runs concurrently, so no statistical speedup or performance
nonregression is claimed. The normal executable grows 58,840 bytes; its trace
build grows 33,792 bytes. This is a capability extension. Cumulative allocation
requests do not measure peak or retained heap.

**Trace erratum:** the historical report's 28 vectors were empty because
`reset()` did not enable recording; their equality does not establish dispatch
invariance. The original record remains immutable. The
[common-section follow-up](implicit-shared-sections-2026-11-08.md) provides new,
nonempty recorded comparisons against published main.

The unchanged nonconstant `(D,D^2)/1` census still completes its first four
cases and returns `Unsupported` for the unselected shared-component case.
General constancy recognition, nonconstant sections, algebraic guard coefficient
fields, filled operation closure, total work, coefficient/system growth and
memory/cancellation bounds remain open. Configured degree and counted-work
limits are resource restrictions. External process containment is neither geometric exhaustion nor
completion. All original hard inputs, failed prototypes and adverse samples
are retained with their actual outcomes.

The reviewed capability catalog adds six named fixtures while preserving all
417 prior rows and the target contract. All 423 witnesses pass their declared
assertions, including the existing explicit blocker rows. This is scoped
admission; publication requires a new full pre-push qualification for this source. The running
280-command qualification on spatial commit `827574f9aa` covers that revision.
The local record `docs/evidence/2026-11-08-implicit-guard-components` at
`682551f48eaf6ce880e4f2b762c771f99b7267b1`, branch
`research-implicit-guard-components-20261108`, contains 848 files and
23,553,657 bytes, SHA-256
`6405472723356308a0cd40b94fb7bc170299190bd2fe11b05144b91ed1e6ec1c`.
All 439 local records and four historical excluded-package digests verify;
the original pending payload is unchanged. The [archive index](../evidence-archive.md)
distinguishes local and published availability. Publication to the separate
evidence repository remains pending explicit authorization.

The degree correction is retained separately at
`docs/evidence/2026-11-08-implicit-quintic-degree-correction`, local commit
`5e076576f6e00d1a64ef5af24b320e9e59ec52a0`, branch
`research-implicit-quintic-correction-20261108`: 616 files, 7,882,237 bytes,
SHA-256 `fc7b576deb796a4f185288a42ef358975082e33ec324da13d80342f611efbb04`.
All 440 local records and four historical excluded-package digests verify.
The original 439 records, their raw degree wording and the pending publication
payload remain unchanged. This follow-up changes tests and attribution only.
