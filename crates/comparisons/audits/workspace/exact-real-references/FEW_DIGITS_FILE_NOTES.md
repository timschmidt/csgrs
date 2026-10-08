# Few Digits 0.5.0 — source and targeted transfer pass complete

Official archive: https://r6.ca/FewDigits/FewDigits.tar.gz

SHA-256: 78ac940fdc86ce546bddfbb2ff45bd14d34c812e3ee1b0079f42aad394a837cf.
All 11 regular source-package files / 1,037 physical lines have been read.
The archive has no symlinks or unsafe paths. Inventory and full read ranges
are in FEW_DIGITS_FILE_INVENTORY.tsv and FEW_DIGITS_READ_COVERAGE.tsv.
This is the published 0.5.0 archive, not the complete Darcs development history.

| File | Read findings / comparison candidates |
| --- | --- |
| Combinatorics.hs | Shared factorial stream and multiplicative choose with factorial indexing. Exact for intended nonnegative indices; list indexing and retained factorial growth are performance considerations. |
| Data/Interval.hs | Exact Rational endpoints, sign-split multiplication, reciprocal excludes zero, even/odd power bounds, interval union and QuickCheck generator. Power zero over a crossing interval yields a loose [0,1], still enclosing 1. Ordering invariants are caller conventions, not validated by the exposed constructor. |
| Data/Multinomial.hs | Dense nested coefficient lists, Horner evaluation/composition, naive multiplication, normalization, tensor Bernstein convex-hull bounds, type-indexed derivatives. `degree` returns normalized coefficient count. Empty zero derivative and ordinary multivariate dy/dxy throw tail []; independently reproduced below. Rational-to-Double VectorQ is explicitly approximate. Bernstein bounded derivatives are relevant to Hypersolve/Hypercurve, but those already use exact Bernstein and specialized shared-scale representations. |
| Data/Real/Base.hs | Rational base, dyadic approximation using integer bit-length search, lazy unreduced rational powers, and a balanced-LCM common-denominator sum. Compression and sum policies are test/benchmark candidates; compare Hyper's existing integer approximants and exact rational accumulation. |
| Data/Real/Complete.hs | Completion as positive-gauge function; unit, epsilon-halving join, lifted unary/binary maps, explicit continuity modulus and composition. Constant maps deliberately avoid evaluating undefined moduli/inputs. Semantic validity relies on regularity and actual continuity promises, neither checked by the type. |
| Data/Real/Gauge.hs | Rational alias with strictly-positive intention only. |
| Data/Real/CReal.hs | Gauge-to-Rational closure, no finest-approximation cache; compress at lift boundaries; approximate integer bound plus clipping; partial nonzero/order witnesses; asymmetric product modulus; Bernstein derivative bounds; Taylor functions with repeated reduction to 2^-51; Machin pi; Wolfram digit-recurrence sqrt; equal-share batch sum. sqrt zero loops in scaling; both exact-zero and valid negative-sided regular-zero representatives time out independently. Endpoint inverse functions use divisions by zero without special cases; asin(1)/acos(1) time out. |
| Data/Real/ICReal.hs | Exact rational variant plus a constructive closure carrying cached coarse bounds. Non-rational cos dispatches to sin; intervalSqrt's zero-lower case returns input upper bound, which is not a sqrt upper bound below 1. Test both raw result and propagation through clipping consumers. Rational dispatch resembles Hyper's richer exact classes; promises in witness constructors remain caller responsibilities. |
| FewDigits.cabal | Version0.5.0, no build-depends/test suite, four exposed modules. License filename is misspelled Licence.txt versus actual License.txt. Native configure passes after selecting GHC; build fails because base/QuickCheck are undeclared. |
| License.txt | Two-condition BSD-style license text (manifest labels BSD3); no donor code copied into production. |
| Setup.hs | Standard Cabal Simple entry point. |

## Build and supporting paper

Artifacts: /tmp/fewdigits-audit.hKnqR2. Original Cabal configure succeeds with
GHC9.6.7 in PATH; original build fails on missing dependencies. Direct original
GHC checking with installed QuickCheck2.14.3 also fails on old Ratio import and
the old coarbitrary member of Arbitrary. These are not native numerical passes.

The isolated compatibility copy changes only Ratio/Maybe import locations,
Prelude <*> hiding, the modern CoArbitrary instance split, the Eq constraint
on Polynomial, and an identifier `rec` newly reserved by enabled extensions.
With -XUndecidableInstances, all eight modules typecheck. Numerical formulas,
special cases, approximation policies and bounds are unchanged. Deprecated
extension/style warnings remain. /tmp/fewdigits-compat.diff records the edits.

The linked preprint, A Monadic, Functional Implementation of Real Numbers,
has been read visually on ALL21 pages, including every proof, the timing table,
bibliography and five-page source appendix. Its PDF text encoding is broken
in both browser extraction and pdftotext; no garbled extraction is counted as
read. The project PDF and arXiv cs/0605058 download are byte-identical:
ec42312a5dc8746d6b2d5943409fa6d5daa105c9f0c4435b483ae9014cb7a108.
Rendered pages are under the temporary artifact directory (page1 separately
at /tmp/fewdigits-paper-page1.png).

The paper proves a completion monad on prelength spaces and explicitly
distinguishes function representatives from real-number equivalence. It
explains asymmetric product bounds, compression and balanced/equal-share
sums. Its prototype is older than the downloaded package: the appendix uses
approxRational and Newton sqrt, while 0.5.0 uses dyadic compression and Wolfram
sqrt. The theorem's sqrt-zero case is present in prose but missing from the
0.5.0 implementation. Paper timing claims are historical, not current results.
Formalization is future work there; this Haskell archive is not a verified
implementation merely because the paper contains mathematical proofs.

## Numerical qualification (compatibility build)

FewProbe passes 655,415 exact-Rational controls: dyadic compression/error,
unreduced powers and sumBase, interval arithmetic, regular-function arithmetic,
positive rational sqrt, polynomial evaluation/derivatives, sampled Bernstein
bounds, continuity moduli and constant-map laziness. Regular inputs q+s*epsilon
with s in {-1,0,1} satisfy the advertised approximation contract; no malformed
input oracle is needed for the failures below. Counts and domains are in the
probe source and /tmp/fewdigits-probe.log. A reporting probe's exit zero does
not mean its reported failure count is zero.

FewSemantic passes 28/40 exact identity checks. Non-rational ICReal cosine of
a valid opaque zero returns zero instead of one at 8/16/32/64 bits. For the
valid value (opaque(5/4)+1)/16 = 9/64, sqrt itself approximates 3/8 correctly,
but intervalSqrt supplies [0,3/16]. Downstream clipping makes its square 9/256
instead of 9/64, and multiplication by opaque one gives 3/16 instead of 3/8.
These are result-changing enclosure/dispatch bugs, not just loose metadata.

Generic sqrt(0), sqrt of the valid regular zero representative -epsilon,
asin(1) and acos(1) each time out at three seconds (exit124). The empty
polynomial derivative and its lifted zero polynomial each throw tail [] (exit1).
The unchanged embedded QuickCheck properties independently reproduce ordinary
multivariate dy/dxy exceptions, including a polynomial with zero inner
coefficients. The intervalSqrt property fails the explicit [1/16,3/16], 3/8
witness; a random run terminates with stack overflow, not a test pass. Base
approximation and first bitLength property each pass 1,000 cases; bitLength2's
own shiftL boundary expression overflows on input0. Both Bernstein-bound
properties pass 300 cases. This is a property defect, not evidence that the
bitLength implementation failed the independently passing approximation grid.

All 216 elementary-function comparisons pass against independent directed
1024-bit MPFR bounds at 16/64/128 requested bits: sqrt, exp, ln, sin/cos,
asin/acos/atan, sinh/cosh, asinh/acosh/atanh and erf. Inputs are exact dyadics;
the known nonterminating boundary cases are excluded and reported separately.
This finite interior grid does not certify complete elementary closure.

## Performance and matching Hyper controls

All probe sources, the compatibility copy/diff, raw tables, commands and logs
are now preserved in few-digits-qualification/. Frozen executables remain at
/tmp/fewdigits-audit.hKnqR2; the README states reproducible compiler settings,
column units, failure exit statuses and policy differences. Original inventory
hashes reconcile after qualification: 11 files / 1,037 lines remain unchanged.

180 CPU6-pinned donor summation observations compare original balanced-LCM
sumBase, sequential LCM, strict left addition and balanced rational addition.
Every result passes an exact Rational oracle. Balanced LCM loses to sequential
LCM on shared/nested/dyadic families and small independent inputs. At512 terms /
512-bit parameter, mixed-family medians are 50.03ms balanced LCM, 63.48ms
sequential LCM, 2.132s left addition and 37.55ms balanced addition. Balanced
addition also uses 2.42MB managed allocation versus balanced LCM's 50.56MB.
These are process CPU/managed-allocation measurements, not cross-language
performance ratios or peak-memory figures. Individual canonical reductions
can turn nominally equal denominators into proper divisors.

180 matching Hyper schedule observations all pass independent GMP rational
oracles. External sequential/balanced LCM controls use the same public Hyper
wide-GCD primitive; public mean_refs additionally retains its private exact
structural/mixed-width shortcuts. Their final normalization is included. The
first harness falsely interpreted Hyper mixed-fraction Display with GMP's
whitespace-insensitive parser; direct signed numerator/denominator interchange
corrected the HARNESS and every final row was rerun. No Hyper arithmetic
failure is claimed from that superseded partial table.

48 additional costly-case timings disable allocation counter increments.
At128 terms /128-bit parameter, mixed-family medians are 3.236ms current mean,
3.399ms sequential LCM control, 4.786ms balanced LCM and 4.076ms balanced add.
Independent-family medians are 2.181/2.258/4.308/3.520ms respectively. At256
terms /512-bit parameter, balanced LCM remains 3.5--8.7% slower than current,
with greater cumulative allocation demand. Balancing denominator merges is
not a worthwhile Hyper transfer here; backend GCD and final reduction costs
matter more than donor-language asymptotics. This is a schedule experiment,
not a production A/B patch or evidence that every possible threshold was tuned.

54 paired Fibonacci-ratio sine observations compare unchanged dyadic
compression with an isolated reconstruction of the paper's approxRational
compression. All54 pass independent directed8192-bit MPFR checks, reserving
the additional compact-report rounding error from the donor error budget.
Dyadic compression is about1.1--5.6x faster; at512bits it uses about72--80%
less cumulative managed allocation. Its output denominator has more decimal
digits but is a power of two, illustrating why payload size alone does not
predict arithmetic cost. The rational-compressed linked probe adds16,512 text
and640 data bytes. Removing compression entirely did not finish the2395/2396
Fibonacci-ratio sine /128bit request in10 seconds; no finite achieved-accuracy
or speed result is claimed for that unfinished variant.

Unchanged Hyperreal21e76ea passes920 matching scalar checks:864 directed-MPFR
interior/history,24 endpoint/history and32 exact wrapper-witness/history.
Computable sinh/cosh checks use defining exp formulas (there are no named
methods at that layer), not Real's separate optimized hyperbolic dispatcher.
The unchanged Hypersolve symbolic module passes486 derivative/zero controls,
including x*x*y, dy, dxy, dyy and canceled polynomials. This is a scoped source
module build against frozen Hyperreal, not a full Hypersolve integration gate.
The donor semantic workload finishes Memcheck with zero memory errors, despite
its numerical failures. It is not a full leak or legacy-runtime qualification.

## Final transfer disposition

- Keep demand-bounded approximation payloads, not simplest-rational outputs:
  the donor experiment supports Hyper's existing implicit-error single-BigInt
  approximants. Do not apply approximation compression to retained exact
  Rational inputs or symbolic proof data. No representation replacement needed.
- Reject balanced-LCM import: it fails the tested Hyper performance/memory
  tradeoff. Hyper already has one-final-reduction mean/product schedules and
  specialized dyadic/equal-denominator paths in aggregate_products.rs.
- Equal-share sumRealList avoids linearly accumulating error budgets, but
  Hyper already has size-hint-gated, order-preserving binary-carry Real sums
  (add_sub.rs:230 onward), homogeneous symbolic collapse and the retained
  scale-aware shared-DAG Add scheduling. The earlier ireal paired qualification
  in the root ledger covers independent versus shared-prefix tradeoffs; an
  additional n-ary node/cache/serialization variant is not justified here.
- Keep certified bounds as proof obligations: Few Digits' stale sqrt bound
  demonstrates that clipping is only sound with an actual enclosure. Hyper's
  structural facts and domain-aware constructors are richer; matching scalar
  controls expose no analogous defect. No unchecked interval promise imported.
- Bernstein derivative bounds and constant/zero handling are already present
  in Hypersolve/Hypercurve, including earlier shared-scale subdivision work.
  The donor's compact nested-list implementation is not a completeness upgrade.
- The completion monad and caller-supplied continuity moduli are a genuine
  more-general API capability, not falsely called identical to Hyper's
  elementary tower. A checked convergence/modulus/domain, cancellation,
  serialization and exact-fact contract plus an actual consumer is required
  before exposing arbitrary closures in the scalar core. No such expansion
  is selected in this pass. Partial equality is intentionally partial, not a
  defect to fix by claiming undecidable general equality.

No new Hyper production change is retained. This closes the published archive,
supporting-paper review, scoped qualification and targeted transfer comparison;
it does not certify this donor as generally correct or cover inaccessible Darcs
history. Remaining ecosystem references are independently pending.
