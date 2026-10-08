# Checkpoint 69 — wide coefficients pass; zero-factor division is next

The isolated power-sum constructor matches the retained baseline throughout the
new wide-coefficient corpus. A separately checked mathematical opportunity
addresses the known zero-resultant division gap and takes priority over timing.
No production/donor source or retained-transfer count changes.

## Wider exact qualification

The authored corpus covers all 23 ordered degree pairs with product at most nine,
five construction-height parameters (1, 31, 65, 129, 257), and distinct, repeated,
unused-zero and complex-conjugate carrier families. Every selected root is an
exact rational point. Common signed rational coefficient scales exercise
normalization; repeated equal carriers also exercise sharing. This is not a
random or exhaustive sample of rationals, arbitrary intervals or all histories.

There are 1,840 arithmetic cases and two policies, STRICT and APPROXIMATE_512:
3,680 query records plus a terminal record per variant. Both full outputs are
byte-identical. Each variant returns 3,540 Transformed, 80 denominator guards and
60 Undecided zero-resultant controls. Neither loses or gains a public answer.

The independent polynomial-ring Sylvester oracle checks complete signed
polynomials, including multiplicities, not only their root sets. Exact rational
arithmetic separately checks source roots, selected quotient/sum/product values,
point endpoints, witnesses, vanishing and metadata. All 81,610 assertions pass,
using 1,791 distinct determinant constructions. Repeated policies/scales are
not independent defect counts or statistical observations.

Maximum observed widths are 2,577-bit input numerators, 258-bit denominators,
2,320-bit primitive input coefficients and 4,639-bit resultant coefficients.
The construction-height parameter is not the final coefficient bit width.

Both release harness builds/runs and warnings-denied Clippy gates pass. Complete
dependency metadata agrees after only intended path normalization: 33 packages/
nodes, one of each relevant Hyper crate and identical lockfiles. No new solver
test suite, Memcheck, WASM execution, consumer run or timing campaign is claimed;
checkpoint 68's unchanged-source qualification remains separate.

## A completeness opportunity, not yet an implementation

Every new Undecided result is the known division-resultant collapse: both
carriers contain zero as a root, although the selected divisor is nonzero.
The raw quotient construction retains a common x factor, so its resultant
vanishes identically. This is lost completeness, not an incorrect returned value.

An independent probe checks all forty earlier public controls and thirty new
wide cases before policy duplication: seventy records / forty distinct
normalized carrier pairs. All divisor intervals exclude zero exactly. If
Q(x)=x^k R(x), then Q(beta)=0 and beta!=0 imply R(beta)=0. The probe verifies the
factor identity, retained divisor-root isolation, a nonzero deflated resultant,
and exactly one root in every authored quotient image. All 810 assertions pass.
For example, zero divided by the selected root 1 of x(x-1) currently returns
Undecided; removing the unused divisor zero gives a nonzero carrier for zero.

This does not implement a Hyper fix or prove a total division algorithm. Before
implementation, preserve STRICT nonzero evidence, original validation and the
unchanged interval/polynomial/witness replay. Two architecture interactions need
explicit coverage:

- Deflation changes the internal right carrier and potentially degree admission.
  The oversized-carrier sharing shortcut currently compares the original source
  polynomials. It must not reuse an unmodified left carrier for a transformed
  right carrier merely because the original inputs matched.
- For an already nonzero resultant, dropping x^k changes the resultant by a
  scalar factor, whose sign can change primitive orientation. Existing full
  output orientation must be preserved or any change explicitly justified;
  do not reduce the oracle to root-set agreement to hide such differences.

Pursue this as a separate completeness transfer before benchmarking power sums.
The optimized constructor stays isolated, and the earlier forty controls are
not relabelled as newly discovered bugs.

## Evidence and resources

Wide execution gates finish at 2026-09-11T17:11:25.768Z; the independent wide
check at 17:14:48.039Z. The separate deflation probe finishes at 17:55:16.338Z.
Those elapsed times and the gap are not benchmark evidence. Sources are
rechecked against the unchanged 956-file live and 176-file candidate maps.

Evidence assembly initially fails because an absent optional metadata field is
undefined in memory but null after JSON array serialization. The initial evidence
and verification captures remain exit one; an uncaptured record attempt hits the
same assertion and writes no manifest. The initial probe/evidence/verifier code
is archived. Making those forty metadata slots explicitly null leaves serialized
probe output byte-identical. Fresh recomputation compares the original and
corrected objects after serialization and preserves every polynomial, root,
input and mathematical assertion. This is an audit-metadata correction, not a
Hyper or mathematical failure. Its fresh probe passes at 18:14:54.114Z.

No solver/scalar tree was copied. Two dedicated executables total 5,616,216
bytes under /tmp/calcium-power-wide.ijyxgC, using the same two-job nonincremental
Cargo cache. Input data occupies 2,838,932 workspace bytes; paired full output
28,016,100 bytes. Cache growth is additional. Nothing was deleted, committed
or pushed; six retained continuation transfers and donor coverage remain
unchanged (1,415 complete files, twenty partial, 183,653 uniquely read lines).

Current isolated evidence verification, from this directory:

```sh
node verify-power-wide-v69.mjs
```

Full inventory/reference reconciliation, donor reads, separate completeness
qualification, matched native/WASM costs, allocation and consumer/size gates
remain open. The full ecosystem objective is not complete.
