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
