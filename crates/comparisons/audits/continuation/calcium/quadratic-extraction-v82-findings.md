# Checkpoint 82 — quadratic extraction and normalization

Four new donor files / 644 lines are read completely. Native selected-root and
normalization checks pass; Hyper proves every tested identity and ordering.
No production change is retained. Seven continuation transfers remain retained;
the full ecosystem audit is still open.

## Source coverage and architecture

| Pinned file | Archived lines | Current lines |
| --- | ---: | ---: |
| `qqbar/get_quadratic.c` | 226 | 226 |
| `qqbar/test/t-get_quadratic.c` | 100 | 92 |

Current paths have the `src/` prefix. Pins remain Calcium
`8dbb16fc4fe92eaf3ebbc7478d629e994d39f944` and FLINT
`e269d38061d7a42070ddcffe6eb114466ed4aa7e`. Every selected line was read; no called
factorization/interval implementation is newly credited from these wrappers.
The manuals and Hyper excerpts recorded in the origins are contextual rereads.
Coverage becomes 600 archived complete files / 63,484 lines, and 912 current
complete + 20 partial files / 126,417 lines: 1,512 complete / 20 partial /
189,901 unique lines in this continuation, not the entire ecosystem.

The extraction contract is `(a+b*sqrt(c))/q`, positive q and joint content one.
Degree-one values return b=c=0. Degree other than one or two aborts in the archive
and throws in current FLINT; callers do not receive a general partial conversion.
Degree two uses the primitive polynomial and selected-root enclosure, not just
the polynomial. Gaussian rationals get c=-1 without general factoring. Otherwise
mode 0 removes powers of two, mode 1 factors fully, mode 2 uses a smooth-factor
heuristic. Reduced coefficients are canonical relative to a chosen radicand;
mode 0/2 do not promise the same radicand as full factorization.

Complex conjugates are selected by imaginary sign; pure real radicals by real
sign. General real quadratics approximate both formula branches with doubling
precision and require exactly one to overlap the stored selected-root enclosure.
The branch-selection loop refines formula enclosures, not the stored input
enclosure. This relies on qqbar's valid isolating-enclosure invariant. No generic
computable-real total comparison or guaranteed cheap branch selection is inferred.
All outputs in this probe are distinct; undocumented output aliasing is untested.

Two implementation comments have notation slips: the initial variable is
4ac-b^2, so its square test recognizes Q(i), despite the comment referring to
its negative; the factor-helper contract is `|D|=A*B^2`, not
`sqrt(|D|)=A*B^2`. Actual arithmetic is consistent and qualified independently.
The archive manual also presents q as const in its signature although the
implementation writes it; current documentation corrects that signature.
These are documentation issues, not demonstrated numerical defects.

Upstream randomized tests reconstruct the value and check normalization, using
different operand-size ranges for the three policies. Archived default count is
10,000 times its multiplier; current count is 1,000 times its multiplier. Both
test files are read, not newly executed. No coverage equivalence is inferred from
their shared structure or from this separate deterministic collector.

## Independent native qualification

775 authored inputs yield 4,650 rows plus terminal, 22,104,752 bytes. Each is
queried under three factorization policies and two cache histories. There are
4,320 grid rows, 192 large rows, 48 close-conjugate rows and 90 rational rows.
The grid covers real and complex squarefree bases, signs, denominators and square
factors through 65,537. Focused cases reach +/-2^256 and denominator 2^1024.

99,587 independent checks validate exact coefficient identities, content,
positive denominators, radical sign/magnitude and selected sign; original and
reconstructed primitive polynomials; and both components of both values using
exact quadratic endpoint signs and width <=2^-96. Twenty corruptions are rejected.
The mathematical checker does not use donor equality as its oracle.

Mode 0 leaves a nonminimal radicand for 412 of 775 inputs, as permitted. Full
factorization gives every expected minimal radicand; smooth factoring also does
so on this corpus, without implying a general guarantee. Coefficients are stable
on repeat. The second cache history follows a 2048-bit output request, so it is
not a calibrated cold/warm performance comparison.

Native and Memcheck streams match byte-for-byte. Memcheck reports zero errors and
zero live blocks: 1,276,837 allocations/frees, 280,998,786 cumulative requested
bytes for the entire collector. This includes construction, extraction,
reconstruction, enclosures and output; it is neither peak memory nor per-operation
cost. Only the current pinned library is executed. Both bounded commands finish.

## Hyper comparison and transfer decision

Hyper already implements bounded small-square stripping, exact square rejection
filters, rationalized residual denominators and retained reductions. Its bounded
256-node quadratic-surd parser uses a small lazy memo vector; supported radicals
feed exact signs and square-norm recovery. This is not a general algebraic-number
coefficient extraction API. Expensive global discriminant factorization is not
needed to preserve an exact lazy value.

The public probe covers 467 nonnegative-radicand inputs at floors -64 and -256:
934 rows plus terminal, 303,998 bytes per profile. Full debug/release streams
match, including certificates. All 934 normalized-form equalities, 934 squared
identities and 934 orderings are correct. Ordering counts: 444 Less, 54 Equal,
436 Greater. There are no Unknown results in this bounded corpus.

72 rows retain a nonminimal residual (36 inputs, two precision policies). These
use base 23 with square factors 19, 257 or 65,537; all normalized identities still
prove equality through bounded refinement. Other equality routes are 822
StructuralEquality and 40 DifferenceStructuralFacts. Squared identities use 510
StructuralEquality and 424 DifferenceStructuralFacts. Nonminimal representation
is therefore not a demonstrated completeness defect in these cases. Thirteen
Hyper corruption controls pass. Fmt, locked offline 21-package metadata, both
profiles and warning-denied all-target/all-feature Clippy pass for the audit crate.

Retain Hyper's bounded normalization policy. Broader factor stripping might trade
up-front work for fewer later proof queries, but that is only an unmeasured
workload-specific hypothesis. No new production edit, matched benchmark,
full-stack regression, WASM runtime or representative linked-size claim is made.
The prior two-candidate rounding and proof-scheduling ideas remain open.

## Provenance, storage and limitations

The unchanged 957-file live map and 184-file isolated candidate are checked.
Tracked donor worktrees remain clean and HEADs match their inventory pins.
Two sandboxed zero-exit captures have empty result logs and are preserved as
unusable: preparation and the first driver. Preparation had written its exclusive
files, so it was not replayed; a separate approved confirmation checks all their
hashes, input content, oracle and timestamps. The approved driver confirms the
successful existing native gates. Native outputs themselves were not empty.

One C executable is 14,024 bytes; shared-target Rust executables are 3,522,104
and 2,035,880 bytes, total new executable products 5,572,008 bytes. Existing
libraries and the shared build target are reused. No source tree copy, library
rebuild or cleanup. The post-build snapshot has 13,910,568,960 bytes available on
/tmp; this is capacity, not attributable peak consumption. Raw enclosures are
stored in the workspace as reproducible evidence.

Combined evidence, sealing and final verification are separate gates. Once sealed,
run `node verify-quadratic-extraction-v82.mjs`. Remaining qqbar expression and
complex-support files, other ecosystem references and full inventory
reconciliation remain open. No external report, commit or push was performed.
