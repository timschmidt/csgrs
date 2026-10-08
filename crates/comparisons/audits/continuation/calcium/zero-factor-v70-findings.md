# Certified divisor zero-factor trial — checkpoint 70

Status: isolated numerical qualification, **not retained**. Six continuation
transfers remain retained. The full ecosystem inventory is still open; this
checkpoint adds no donor-line credit.

## Change and proof boundary

An exclusive 175-file / 6,094,998-byte solver copy shares the existing frozen
scalar dependencies and build cache. It starts from the retained point-witness
baseline, not the separate power-sum candidate. Only algebraic_binary.rs changes
(56 added / one removed line, including documentation/test-module declaration),
plus a new 260-line module containing seven tests. Current live 956-file and
separate power-candidate identities recheck unchanged.

For division, after the existing oversized square-free reduction, write the
divisor carrier as Q(x)=x^k R(x). Exact zero coefficients and a STRICT proof that
the divisor interval excludes zero permit using R internally. R remains
nonconstant. The ordinary path borrows its coefficient suffix; the oversized
path drains the owned prefix without another vector allocation. Original
source evidence, interval construction, witness recovery and all downstream
containment/vanishing/uniqueness checks remain unchanged. Approximate policy
does not supply the nonzero proof for this reduction.

For already nonzero raw resultants, preserve signed primitive orientation:
Res(P,x^k S)=((-1)^deg(P) P(0))^k Res(P,S). Positive content normalization
removes magnitude, not sign. A formerly identically zero resultant has no
orientation to preserve. Source payload validation rejects a zero leading
coefficient, so stored degrees are valid. Keeping square-free reduction before
deflation avoids treating originally shared but differently reduced carriers
as equal. Literal zero divisors and residual over-cap degrees remain guarded.

## Numerical results

| Corpus | Paired queries | Unchanged full records | Newly Transformed |
|---|---:|---:|---:|
| Earlier rational/interval, STRICT | 6,440 | 6,388 | 40 Undecided + 12 degree-rejected |
| Same corpus, approximate policy | 6,440 | 6,388 | 40 Undecided + 12 degree-rejected |
| Wide coefficients, both policies | 3,680 | 3,620 | 60 Undecided |
| New degree/sign/shared controls | 132 | 112 | 12 Undecided + 8 degree-rejected |

Total: 16,692 paired queries, 184 new answers and 16,508 unchanged full records.
Repeated policies/scales/carriers are not distinct defects. The independent
rational Euclidean-GCD and polynomial-ring determinant checker passes 128,652
assertions / 6,441 distinct determinant constructions. All 14,168 checked
successful polynomials agree in full signed coefficients, including existing
multiplicities, not merely root sets. Exact point values/witnesses, source and
result metadata, interval containment/ownership and root uniqueness are checked.
New degree cases include x(x-1)^8 divided by itself, degree admission 3×4→3×3,
still-rejected 4×4→4×3, signs and odd/even removed multiplicities.

The wide corpus retains its 23 admitted ordered degree pairs, four families,
five construction heights and input numerators up to 2,577 bits. It is not
exhaustive over rationals or arbitrary intervals. Another 384 extended
nonrational/history/policy records stay byte-identical and pass 11,248 independent
checks: 316 Transformed, 24 denominator guards, six Undecided, six nonisolating
and 32 InvalidEvidence. Do not add those unchanged records to the 184 recoveries.

## Tests, memory and reproducibility

- Fresh default tests: 817 passed. Fresh all-feature debug and release: 818
  passed each, eight suites per configuration, zero failed/ignored. All 811
  retained all-feature test names remain, with seven additions. The single
  default/all-feature difference is the existing tensor-resultant opaque-zero
  pruning test, not a missing regression.
- Solver all-target/all-feature and collector Clippy pass with warnings denied;
  formatting and release all-feature WASM compilation pass. WASM was not executed.
- Dependency metadata: 33 packages/nodes; all 32 dependency packages/nodes and
  root dependency edges agree after the solver-path normalization. Collector
  package name/targets differ intentionally; locks agree after its name change.
- Three serial native Memcheck runs (public, extended, and new degree corpus)
  have zero errors and zero definite/indirect/possible loss. All full outputs
  match normal execution. Still reachable bytes are respectively 1,843,616 /
  48,648 / 28,424; these process totals include setup, caches and collection,
  and establish neither matched memory savings nor peak/RSS or boundedness.
- Four new executables total 11,393,464 bytes under
  /tmp/calcium-zero-factor.1YCn5a; one old baseline executable is reused. No
  scalar tree duplication. The post-build snapshot records 15,054,901,248 bytes
  available on /tmp; shared-cache growth is additional. No deletion/commit/push.

The first unit run was six-pass/one-fail: its new-result expectation incorrectly
forced a positive leading sign after a negative input scale. The existing
square-free routine divides by a monic GCD and preserves that input sign.
An independent determinant confirms both signs; only the test expectation was
corrected. A subsequent full gate passed tests and Clippy but failed formatting
on the corrected assertion. Both test versions and all three failed captures
(unit, formatting, enclosing gate) remain preserved. Production algorithm bytes
did not change after the initial trial. Fresh final gates finish at
2026-09-11T19:07:41.047Z, numerical oracle at 19:07:59.070Z, extra gates at
19:08:36.656Z and evidence assembly at 19:10:45.469Z. Elapsed gate times are not
benchmarks.

Current source check: `node zero-factor-final-sources-v70.mjs`.
The initial and signed-only source entry points intentionally describe earlier
test states. Final evidence entry point: `node verify-zero-factor-v70.mjs`.
[Evidence assembly](results/zero-factor-evidence-v70.json),
[final verification](results/zero-factor-verify-v70.json).

## Remaining decision

Run matched native/WASM CPU and separate per-query allocation measurements,
including no-zero bypasses, signed unchanged results, new-answer work, and
oversized/shared carriers. Preserve full numerical checks before timing,
balanced order, raw observations, source/executable bindings and corrected
statistics. Qualify representative consumer tests and stripped sizes before
deciding retention. No current throughput, binary-size or memory improvement
is claimed. The power-sum trial and remaining donor/reference reads remain open.
