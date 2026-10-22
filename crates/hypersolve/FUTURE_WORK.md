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

## Represented-root completeness

`represented_strict_sign` refines a represented value's image interval at
most 16 times. A tiny but nonzero value, such as `p√3 - q` near `2^-78` for
consecutive Pell numbers, stays undecided. Nested and tensor ratios with such
a denominator then return `Predicate`, although the quotient is well defined.
The answer is honest but incomplete. `tests/represented_roots.rs`
(`nearly_vanishing_denominators_are_never_misclassified`) accepts either the
correct value or `Predicate`. A deeper refinement schedule, or an exact
algebraic sign for the denominator, would decide these cases.
