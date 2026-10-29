# Future work

## Recursive quadratic field contracts

Two contract gaps were found while writing the independent recursive quadratic
field tests (`tests/recursive_quadratic_field*.rs`). Neither gives a wrong
result for current callers.

- **`RecursiveQuadraticValue::is_stored_equivalent_to` field identity.** The
  documentation describes a comparison of representations in the same
  recursive field. Its exact-witness fallback, however, never checks field
  identity: the value `1 + 2√2 + 3√3 + 4√6`, built in two separately
  constructed but equivalent base fields, compares equivalent. Its two
  production callers, `joined_with` and `recursive_embed_foreign_field`, only
  compare values from one field. Either require the same field or document
  the cross-field case as a value comparison.
- **`embed_value` with equivalent-base embeddings.** Embeddings from
  `rebased_to_equivalent_base` or `extension_embeddings_to_equivalent_tower`
  map base-level values only when the base is the identical object. A value
  with base-level components therefore embeds as `None`. Downstream callers
  pair these embeddings with `RecursiveQuadraticValue::rebased_to_equivalent_base`
  instead. Composing them directly would remove that two-step requirement.

### Unused tensor generators (2026-10-29)

Three acceptance tests fail on `146cc0804f` because exact zero padding
requires a witness for an unused source axis. The evaluator now scans the
positive-power slices for stored exact zeros and evaluates the constant slice
when the axis is unused. It retains the full native field and only caches
successful exact values. Mixed axis orders and coefficients of `2^-80` check
that a genuine dependency is never discarded.

The original `2*sqrt(2)-sqrt(8)` fixtures now require a zero witness. A new
`2*c*sqrt(2)-c*sqrt(8)` fixture retains a genuinely used cubic source and
checks the exact sign fallback. All 26 local qualification groups pass,
including 106 native regression tests, full
Hypersolve (1,293 passed), Hypercurve
(2,990 passed), CSGRS
(691 passed), six downstream suites, strict MSRV
Clippy, minimal builds, docs and fuzz checks, allocation lifetime checks and
the existing capability matrix. Raw logs distinguish ignored tests.

Evidence: `docs/evidence/2026-10-29-hypercurve-closure-native-tensor-witnesses`, SHA-256 `147e1b34d3a35e74f3961c9f21a0a62fa3007cd4f9f8337aca568ed602e38c67`. Original failures, source patches,
commands and local cost samples are retained. No repeated-chamfer completion,
universal closure, total-work bound or publication is claimed by this record.

## Modular certificates for tower remainders

The H14 lead (a modular/CRT gcd over a selected field, with a maximality
certificate) was evaluated on the equality that dominated the chamfered
bevel stroke in Hypercurve. That proof is not a gcd: it is the
sign-preserving remainder of a degree-192 rational eliminant by a degree-6
relation over a two-source, one-extension tower, and the remainder is
identically zero. A modular image of a nonzero remainder proves only
nonzero, and a modular proof of zero would need a height bound for tower
coefficients after reduction modulo the source polynomials, which this crate
does not compute. The remainder schedule and the source-quotient reduction
were made cheaper instead (`PERFORMANCE.md`). A modular gcd over Q(alpha)
was also proposed for Hypercurve's remaining common-fiber stalls. The R16
captures on 2026-10-25 separate two cases: the degree-36/40 rational pair has
a verified degree-36 GCD that already finishes in about 26 diagnostic ms,
followed by an unfinished Sturm sequence. The other pair has degrees 4/40
over `Q(sqrt(89))`, a degree-10 carrier and selected rational degree 20;
its verified GCD has degree 4, and general coefficient arithmetic dominates.
The existing quotient-reduction API suffices for the first zero identity,
but both tested consumer variants remain unadopted. See
[Hypercurve's captured evidence](../hypercurve/FUTURE_WORK.md#3-selected-fiber-common-root-counting).
No modular/CRT implementation is justified by assuming these are the same
rational-field workload.

## Finite-fiber topology domains

`decompose_bivariate_curve` reproduces its documented blockers at
`61da737db3`. With the circle `F = 16x^2 + 16y^2 - 4` on `[-1, 1]^2`:

| Input | Result |
| --- | --- |
| `F`, `(x - 3)F`, `(x - 3)^2 F` | Topology; rootless `x`-content is already harmless |
| `(2y - 1)F` | Topology; the horizontal line is a graph component |
| `F^2`, `(2y - 1)^2 F` | `RepeatedComponent` |
| `(2x - 1)F`, `(x + 1)F` | `VerticalComponent`, also with the root on the boundary |
| `(y - 1)F` | `BoundaryComponent` |

Square-free reduction is exact and needs no selected field: divide the
`x`-content `c(x)` out first, then any common factor of the primitive part
`p` and `p_y` has every irreducible factor at least squared in `p`, so
dividing it out keeps the zero set. Dividing `F` by a common factor of `F` and
`F_y` instead would remove a simple vertical component `c(x)`. The reduction
was not adopted inside `decompose_bivariate_curve`. The topology is a set
package, and its consumers read multiplicity from the absence of the blocker:
HyperBRep's plane–tensor section expects `RepeatedComponent` for a plane
tangent along a line (`(y - 1/2)^2`), and admitting it as an ordinary section
would present a tangential contact as a crossing curve. Admission needs a
published multiplicity record for each branch and isolated point, and
consumers that read it. Vertical components need a topology carrier for
vertical segments and their adjacency to graph branches. Both are new public
API on a crate prepared for release, so they are deferred.

### Horizontal boundary components (2026-10-27)

The historical `BoundaryComponent` result above and the original `y+2=0`
fixture are retained in the qualification corpus. The extension on
`5de7f0cb05` keeps a horizontal boundary factor in the original equation,
closed fibers and selected graph. Its identically zero boundary specialization
has no isolated entry events; contacts with other branches already occur in
`resultant(F,F_y)`. Root-free separators and exact closed root counts retain
all incident arcs. No factor or multiplicity is discarded.

Ten focused solver witnesses cover both boundaries, distinct components,
parabola and circle tangencies, partial-domain crossings, algebraic event
parameters, `2^-80` separation, exact source replay and reached budget failure
with larger retry. Independent rational Sturm counts audit closed connection
bands. Hypercurve consumes the result through selected trims, overlaps, further
intersections, reversal and similarity transport. The original BRep boundary
section is now a positive native-replay witness. Its reversed restricted pcurve
exposed a separate directed-chart correspondence bug, fixed in BRep's shared
materialization owner without changing topology validation. A later plane contact also requires a
checked similarity view of the retained rational maps; the original definitions
and denominator guards remain authoritative.

The [Hypercurve qualification ledger](../hypercurve/FUTURE_WORK.md) records
commands, raw evidence and the remaining pre-push gate. Vertical and repeated
components, algebraic coefficient fields and total resource bounds remain open;
this does not adopt the multiplicity reduction discussed above.

## Represented-root completeness

`represented_strict_sign` refines a represented value's image interval at
most 16 times. A tiny but nonzero value, such as `p√3 - q` near `2^-78` for
consecutive Pell numbers, stays undecided. Nested and tensor ratios with such
a denominator then return `Predicate`, although the quotient is well defined.
The answer is honest but incomplete. `tests/represented_roots.rs`
(`nearly_vanishing_denominators_are_never_misclassified`) accepts either the
correct value or `Predicate`. A deeper refinement schedule, or an exact
algebraic sign for the denominator, would decide these cases.

## Selected-field chord composition (2026-10-29)

The original second polynomial chamfer exposed repeated quotient expansion and
a later selected-scalar enclosure stall. Unit quotients now use exact Bezout
inverses modulo retained relations; nonunit quotients keep their original local
representation. Optional parent images retain and validate the original
equations and selectors. Exact interval filters refine selected roots only
when endpoint signs certify a bracket, preserve denominator guards, and leave
zero or unresolved signs to the complete exact path. Rational sign queries
retain their earlier remainder path and resource behavior.

The maintained 405-node scalar capture replays in a fresh process and proves
`-1 < value < 0`; independent rational interval arithmetic checks that claim.
Zero-budget refusal permits a larger retry. Original six-refinement budget
regressions remain unchanged. The unused-tensor witness correction from local
`b6ec1de05f` is included with its original zero and nonzero dependency tests.

All four focused groups and all 27 affected qualification groups pass.
The latter include full Hypersolve (1,304 passed),
Hypercurve (3,031 passed), CSGRS
(693 passed), six downstream suites, strict MSRV
Clippy, minimal builds, strict docs and fuzz checks, allocation lifetime checks,
the capability matrix and hygiene. Logs retain ignored-test counts. Evidence:
`docs/evidence/2026-10-29-hypercurve-closure-selected-chord-composition`, SHA-256 `0e16db489d3a194f55c9d7763a8ab6ae21184fe6ce7d4f5be0969cc003967ca3`. These are local admission checks; full pre-push
qualification and package publication are separate.

Individual normalization and enclosure experiments did not complete the
original geometry. Failed attempts, invalid cache attribution and the repaired
filter-budget regression remain in the archive. New bisections use the shared
refinement counter; it is not a total arithmetic-work or memory bound.

## Repeated rational polynomial factors (2026-10-29)

Finite-fiber bivariate topology now reduces repeated generic factors with exact
GCDs of the polynomial and both partial derivatives. Axis content is retained;
checked reconstruction preserves the original rational unit and factor powers.
The original equation remains authoritative while geometric consumers use the
reduced support. Each selected event root keeps all factor memberships, and
each open-slab branch keeps its defining exponent.

The six original circle, parabola, node, cusp, isolated-point and mixed-power
witnesses failed with `RepeatedComponent` on clean `6296155cf7`. They now pass,
with independent Sturm fiber counts, exact incidence, factor reconstruction,
rootless axis factors, degree refusal, exhaustion during reduction and retry.
Original zero-polynomial and vertical-component rejections remain unchanged.

All eight focused groups and all 24 affected qualification groups pass.
The latter include full Hypersolve (1,312 passed),
Hypercurve (3,039 passed), CSGRS
(693 passed), six downstream suites, strict MSRV
Clippy, minimal builds, strict docs and fuzz-target checks, allocation lifetime
checks, the capability matrix and hygiene. Counts are harness-reported passes;
logs retain ignored counts. The matrix verifies 361
advertised outcomes, including explicit blocker witnesses. Evidence:
`docs/evidence/2026-10-29-hypercurve-closure-polynomial-multiplicities`, SHA-256 `b71cabb2d4267a8bfe032a6f09fe06b479ed2b5f90b40eb0a22cd2310c37e0b3`. These local admission checks do not establish
full pre-push qualification or complete general algebraic operation closure.

The bounded component path respects configured projection limits. Existing
work counters do not bound total arithmetic, coefficient growth, retained heap
or cancellation. Recorded process RSS and command timings include builds;
allocation lifetime checks are not a matched multiplicity benchmark. No
performance improvement is claimed. Arbitrary algebraic coefficients, vertical
components and the broader exact-closure target remain open.

## Empty first-coordinate polynomial topology (2026-10-29)

Removing an early nonconstant-axis rejection lets the existing exact projection
and closed-fiber census distinguish an empty rectangle from a live vertical
component. Every first-coordinate root in the closed domain enters that census;
an identically zero fiber still returns `VerticalComponent`. The original
equation, domain checks, root selection and resource controls remain authoritative.

On unchanged `d31ef199c3`, `x^2+1` and `x-3` incorrectly failed on
`[-1,1] x [-2,2]`. They now return empty topology and accept later fiber queries.
Seven owner tests include repeated and degree-six sources, algebraic live roots,
boundary roots, malformed domains, degree limits, zero-budget refusal and retry.
A one-promotion budget completes the rational census; a mistaken new exhaustion
expectation is recorded and corrected against the existing budget contract.

All eight focused groups and all 24 affected qualification groups pass.
The latter include full Hypersolve (1,311 passed),
Hypercurve (3,032 passed), CSGRS
(693 passed), six downstream suites, strict MSRV
Clippy, minimal builds, strict docs and fuzz-target checks, allocation lifetime
checks, the capability matrix and hygiene. Counts are harness-reported passes;
logs retain ignored counts. The matrix verifies 354
advertised outcomes, including explicit blocker witnesses. Evidence:
`docs/evidence/2026-10-29-hypercurve-closure-empty-axis-topology`, SHA-256 `87bc3fab2d2ee245cab8b2877f1966e8eeeb563b01c54c60d6679f86d79b447c`. These local admission checks do not establish
full pre-push qualification or complete general algebraic operation closure.

This is empty-support admission. Actual vertical components and algebraic
coefficient topology remain open. Existing work counters do not bound total
arithmetic, coefficient growth, retained memory or cancellation. Command timing
and process RSS include builds; no performance improvement is claimed.
