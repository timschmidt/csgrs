# Roots of unity and forward rational-angle functions — checkpoint 76

Read 32 previously unread files in full: eight qqbar implementations and their
eight tests in each pinned repository. Archived Calcium adds 16 files / 1,015
lines; current FLINT adds 16 / 949. Coverage is now 568 complete archived files /
61,301 lines and 879 complete + 20 partial current files / 124,316 lines:
1,447 complete, 20 partial, 185,617 uniquely read lines combined. Manuals were
re-read without extra line credit. This does not complete qqbar or the ecosystem.

## Source findings and Hyper comparison

Root-of-unity construction normalizes the rational turn, chooses its cyclotomic
minimal polynomial and validates the selected complex embedding. Recognition
first proves the polynomial cyclotomic, then recovers the numerator from an
argument enclosure at fixed 64-bit precision (except small orders). Cosine
constructs the polynomial for twice the cosine and certifies its root before
dyadic rescaling. Sine reuses complementary cosine; tangent has small-denominator
constants/poles and a generic root-of-unity reciprocal expression. Cotangent,
secant and cosecant use checked poles and inversion.

Both manuals explicitly restrict inputs away from word boundaries, citing
intermediate arithmetic and degree growth. The initial doubled-word concern
is therefore a documented representational limit, not a demonstrated
in-contract defect. No excluded-boundary or huge-degree probes were run.

The sixteen upstream tests use small random inputs and shared-backend overlap
or root reconstruction. The reciprocal-trig tests skip nonfinite approximation
cases and do not directly assert the pole-return flag. All sixteen tests were
read, not newly executed. The independent corpus below checks these contracts
directly, without treating overlap or the upstream identities as its oracle.

Selected live Hyper ranges show that arbitrary-precision rational reduction,
small exact tables, canonical SinPi/TanPi certificates, exact pole rejection and
bounded certified predicates already implement the useful basic architecture.
General rational angles remain compact and demand-driven; adopting an eager
degree-O(q) algebraic object for every angle is not justified by this comparison.
PartialEq is structural; the capability probe uses certified_eq_until instead.

## Independent native qualification

The in-contract corpus uses q = 1,2,3,4,6,12; p from -2q through 2q; unreduced
scales 1 and 3; all eight forward functions; initial and 256-bit cached states.
It contains 3,776 angle records, ten non-root controls and one terminal record.
There are 3,408 values, 368 pole observations and 944 positive root-recognition
observations. Repetition across signs/scales/cache states is not independent bugs.

An independent exact Q(sqrt(2),sqrt(3)) oracle uses authored pi/12 radicals and
quadrant identities. Distinct Galois images determine each real minimal
polynomial; exact division of X^n-1 by lower cyclotomic factors determines the
complex root-of-unity polynomials. Exact algebraic sign comparisons check both
dyadic enclosure components, not approximate overlap. Pole flags, canonical
recognition numerators/denominators, optional NULL outputs, ordered endpoints
and absolute widths <=2^-96 are checked. No FLINT/Arb approximation is the oracle.

All 28,586 checks pass. All ten deliberate record/stream corruptions are rejected.
Native and Memcheck outputs are byte-identical, 910,636 bytes each. Memcheck
reports zero errors and zero live blocks: 397,808 allocations/frees and
41,257,680 cumulative requested bytes, including setup/caching/output. These
are not operation-only allocation, peak/RSS or performance measurements.
Only current FLINT is executed; archived source similarity is not execution credit.

## Hyper completeness observation

The unchanged current scalar is probed at all 24 pi/12 residues for sine, cosine,
tangent and cotangent. Exact period offsets use 0, 5 and 2^1024; two certified
equality budgets (-64 and -256) compare radical expressions, direct periodic
equivalents and radical expressions perturbed by 1/1024. Debug/release outputs
are byte-identical, 1,728 observations each:

- 360 radical equalities are certified; 192 radical comparisons remain Unknown.
- All 552 periodic equivalents are Equal and all 552 unequal controls NotEqual.
- All 72 repeated pole observations return NotANumber as required.
- The Unknown set is exactly 32 function/residue identities: sin/cos/tan/cot
  at residues 1,5,7,11,13,17,19,23 modulo 24, repeated over shifts/budgets.
- Six output/order corruptions are rejected. Formatting, warnings-denied
  all-target/all-feature Clippy for the audit package, and the ten-package/node
  dependency graph checks pass. Only the current Hyperreal is in that graph.

This supports a concrete completeness candidate: certify twelfth-turn radical
relations while preserving the existing SinPi/TanPi value, cache and inverse
function information. It is not yet implemented, timed or selected. Unknown
is a correct partial outcome, not a false inequality. The probe is not full
Hyper regression, all-feature runtime or arbitrary algebraic-relation coverage.

## Evidence, failures and resource use

Source/library/configuration identities are bound before the native build. The
957-file retained live map remains unchanged. Both Hyper probe versions and
their pre-run origins are preserved. Initial rustfmt failure and an i32-to-i64
probe argument error (plus its failed outer runner) remain recorded; the latter
ran no queries. Only formatting and the exact widening conversion were fixed.
The first combined verifier wrongly treated rustfmt's removal of one optional
trailing call comma as an algorithm change. Its failed capture/original script
remain; the corrected verifier permits only that exact formatting difference.
An uncaptured follow-up tried to parse that failed verifier's empty stdout and
failed before doing any work. No mathematical outputs or live source were changed.

The corrected combined evidence check passes 2026-09-11T22:35:44.525Z.
Final sealed verification is recorded separately in results/qqbar-trig-verify-v76.*.
It rechecks source/coverage, mathematical results, negative controls, archived
failures and Hyper capability evidence rather than only trusting successful exits.

One dedicated C executable is 13,872 bytes in /tmp/calcium-qqbar-trig-v76.xvGytz.
The two Rust probe products (3,138,880 debug / 1,943,208 release bytes) stay in the
existing shared target; no duplicate snapshots or full source trees were copied.
Recorded /tmp availability after the Hyper gates is 14,025,854,976 bytes. Shared
cache/filesystem changes are not a workload memory measurement. No native library
rebuild, production/donor edit, cleanup, external report, commit or push.

Current entry point from this directory:

```sh
node verify-qqbar-trig-v76.mjs
```

Seven continuation transfers remain retained. Continue the certificate candidate,
remaining algebraic/inverse-trig/support reads, other formal/symbolic/historical
references, the separate power-sum work and full inventory reconciliation.
