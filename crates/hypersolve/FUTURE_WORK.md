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
