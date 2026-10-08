

## 2026-09-23 — scalar projection names match arbitrary exact values

Committed Hypercurve `44ad8df86e7b94efdbeb58e4ba4a5301a59ac8b4`. The private mapped-circle cut query is now `scalar_value`; its two map helpers, incidence projection and scalar cache entries use the same contract. All callers were updated and the old names removed. Inline values, inverse-map witnesses and chamfer images already accepted arbitrary exact Real values, so the former rational-only wording was false. The query is optional scalar projection, not a rationality proof. Genuine rational-payload guards elsewhere are unaffected.

This changes only identifiers, comments, diagnostic strings and formatting; algorithms, branch checks, tests and cache ownership are unchanged. Both all-target feature configurations pass without warnings. All 368 Hypercurve inputs match the immutable snapshot, with source/staged/HEAD hashes recorded in `mapped-cut-scalars-20260923-{qualification,staged,post-commit}`. No new tests or repeat of the full unchanged-behavior sweep was needed. All owned processes were reaped before commit. The full goal remains active.
