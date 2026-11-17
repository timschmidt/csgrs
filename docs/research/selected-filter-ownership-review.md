# Selected polynomial sign-filter ownership review

At `478795e48`, all 220 lines and eleven declarations in
`crates/hypersolve/src/selected_polynomial_field/filter.rs` were reviewed with
their immediate callers and interval, witness and resource dependencies. Three
lexical inventory rows are updated and eight declarations added; the other
1,582 rows are unchanged. The resulting 1,593-row workspace inventory remains
partial. This closes a declaration-mapping gap, not an implementation gap.

Hypersolve owns this category-3 exact proof adapter (plan section 7.1), under
P1, P4, P5, P8, P10, P19 and P20. Its interval endpoints are derived from
retained selected-field data. Generic arithmetic already goes through
`RealInterval`; selection authority, positive-denominator proofs and fallback
decisions must remain with the field implementation.

| Role | Required distinction |
| --- | --- |
| Coarse coefficient enclosure | A selected quotient keeps its denominator guard; a box crossing zero declines the filter. |
| Top-level sign filter | The field constructor already established a positive denominator. Only a definite nonzero numerator sign is consumed. |
| Rational-coefficient recognition | Preserve the cheaper exact remainder path; do not eagerly refine an otherwise rational query. |
| Refined field enclosure | A monic linear relation gives a parent image. Other relations bisect only after proving opposite endpoint signs; inability to separate a midpoint retains the safe bracket. |
| Memoization | Allocation identity reuses bounds for the same immutable graph during one call. It does not define mathematical equality, and these memo tables do not survive a retry. |
| Horner helpers | `polynomial` interleaves selected coefficient decisions with multiplication; `interval_polynomial` consumes precomputed coefficient intervals. Merging them can change proof work and its order. |

The selected-value caller checks exhaustion and structural zero, tries coarse
bounds, handles a constant numerator, then conditionally tries refinement steps
4, 8, 16 and 32 before the complete selected-root sign path. It checks exhaustion
after each optional refinement. The ordered-field context also uses coarse
separation, with structural zero handled separately. Zero-containing bounds are
never a proof of zero. This audit changes none of those dispatch sequences.

Nested requests double the parent refinement count up to 256, and performed
bisections charge shared refinement units. Call-local maps and coefficient
vectors allocate; neither those step limits nor cache entry counts establish
total work, coefficient-size, retained/temporary-memory or stack bounds. There
is no cache serialization or independently trusted replay flag. The existing
strict `RealInterval` behavior, including conservative product bounds, remains
the arithmetic contract described in the earlier symbolic-adapter audit.

The frozen full-owner receipt for `f65589b44` contains 14 passing selected-field
tests, including repeated-factor zero, nearby nonzero signs, original denominator
guards and budget refusal/retry without approximation. The reviewed module and
eight relevant source/test files are identical at that revision and `478795e48`.
This reuses an existing receipt; no Rust test or benchmark was run for this
documentation-only review, and no focused filter cost or dispatch-count result
is inferred from it.

Static declaration coverage, unchanged unrelated rows and source bytes, spelling,
whitespace, archive identity and repository-budget checks qualify the inventory
change. Production, resource accounts and pending runtime gates are unchanged.
No migration, performance improvement or phase completion is admitted.

Frozen review evidence: `docs/evidence/2026-11-17-interval-selected-filter-review`, local commit
`b62e9eb5ae819cd788a7b379f16ad52147bd950c`; 12 files / 1,539,709 bytes, SHA-256
`071764eecb4a080fbdaafd15db2d365f14c8066fce643f711f39d04aab877b01`. Prior archive records and manifest rows are preserved.
No push.
