# Containment source-validation migration

This supplements the preregistered `prototype-v1/contract.json` after the
baseline reproductions. It does not change the fixtures, assertions, policies,
query algorithm, or acceptance commands.

The source of a nonempty solid-containment query must now satisfy the existing
native closed-PWN validation contract under the selected policy **before any
query shortcut answers**. Canonical empty geometry has both buffers empty and
still yields a decided, Certified `false`.

This is an intentional tightening of the input/evidence contract:

- Invalid, open, degenerate, unbalanced, or partially empty geometry that
  previously reached a successful shortcut now returns its native typed error.
- If closed-PWN validation cannot be certified under the strict policy, the
  query returns `PredicateUndecided` even when its bounds alone could exclude
  the query point. The WASM context wrapper reports its existing `Unknown`
  status, absent value and absent certainty; it must not return a decided false.
- If approximate validation is consumed, its qualification accompanies the
  query. A later strict query must establish admissible source validity again.
  The baseline's out-of-bounds strict success after approximate-only validation
  is deliberately removed; it is not counted as a preserved certified success.

The new regression's symbolic duplicated-corner fixture proves this transition:
explicit strict validation is undecided, approximate validation succeeds with
`Approximate512Consumed`, and the unchanged baseline query nonetheless returns
a successful strict bool. The candidate must refuse that promotion. This is a
correctness/evidence migration, not an optimization or expanded strict-domain
completion claim. Existing certified-success tests remain intact.

This contract establishes source validity and preserves local query certainty.
It does not establish manifoldness, analytic-source fidelity, or end-to-end
construction provenance. The retained parity diagnostic also shows that this
query still disagrees with nonzero-winding solid semantics for nested outward
shells and coincident opposite shells. Those failures remain open and must not
be represented as certified set-membership correctness by this admission.

Native validation has no total-work, heap-byte, or cancellation budget here.
The harness supplies external elapsed/address-space containment. Its first
WASM invocation under the native 8-GiB address-space limit failed before test
execution; the separately retained 64-GiB run executed all 13 tests and
reproduced exactly three failed invalid-input assertions with ten passing
controls. The infrastructure failure is not acceptance-test evidence.
