# Ruffini polynomial/dependent checkpoint — partial, 2026-09-06

Donor pin82d552fee22d92e493936183fab8672517694e56 remains unchanged. This slice
fully reads all29polynomial files/2,585lines and fourclass-group files/423lines,
bringing explicit source coverage to181/245files,11,130/26,234physical lines.
The64remaining files stay UNREAD/in scope. Complete per-file notes are in
`../RUFFINI_FILE_NOTES.md`; inventory fingerprints are not a substitute for reads.

## Native execution and provenance

`run-polynomial.mjs` directly compiles unchanged polynomial, parser, permutation
and finite-field main/test files, IntegerPolynomial, and PolynomialContracts
against the already pinned common/reals/integer classes and dependency jars.
New classes are isolated in `.audit-ruffini-build.LmZgYM/polynomial`.
javac21.0.11 uses --release16 and UTF-8; OpenJDK25.0.4 executes the code. This
does not qualify Maven's lifecycle or a Java16 runtime. All new source/class
hashes are in `polynomial-build-manifest.json`; prior manifests remain intact.

Each default JIT and -Xint process has assertions,512MiB heap,1MiB stack and
common-pool parallelism1. Ordinary cases have120second external caps; Gaussian,
quadratic and two suspected non-progressing divisions have2second caps. All
eight completing probe families finish in both modes. Each of the two division
processes hits its cap and is SIGKILLed in each mode; no process is left running.
The10original JUnit tests per mode pass:7polynomial,1parser,1permutation,1finite
field. Some original polynomial tests only print the result or compare the same
implementation, so that result is not broad numerical correctness evidence.

## Independent arithmetic checks

The main arithmetic corpus has2,630probes, all passing in each mode:

| Operation | Probes per mode | Domain/oracle |
| --- | ---: | --- |
| Ordinary product | 640 | 8×8 length pairs,5seeds, dense and sparse maps; exact BigInteger convolution |
| Karatsuba product | 640 | Identical inputs and independent convolution |
| Evaluation | 420 | 12lengths,5seeds,7signed arguments; independent integer Horner |
| Truncated inverse | 480 | 8lengths,5seeds,12orders; f(0)=1 and every low product coefficient checked |
| Ordinary monic division | 225 | Constructed a=bq; independently known q and zero remainder |
| Fast monic division | 225 | Same known quotient/remainder, not merely a=bq+r reconstruction |

Lengths for multiplication are1,2,3,4,5,8,9,16. Missing coefficients are treated
as exact zero by the oracle, not by donor polynomial equality. All probes use
small exact signed source coefficients; larger coefficients and degrees occur
in the separate fully result-checked performance experiment. The FastDivision
grid's quotient constants are nonzero; dedicated monomial tests below cover
the reversal-width boundary that this grid intentionally does not prove.

## Boundary and dependent results

There are2,960completing probes per mode including the arithmetic corpus; outputs
are byte-identical across modes. Complete statuses are in polynomial-analysis.json.

- Sparse equality rejects x=0+x and accepts x=1+x. Explicit trailing-zero
  degree breaks equality too. Constant differentiation and an empty polynomial
  cannot be recognized as zero because degree() calls lastKey on an empty map.
- Builder and copy-constructor values change when the original builder changes.
  The multivariate builder, monomial exponent array and recursive addition have
  independently checked aliasing failures.
- FastDivision(x²,x) returns a wrong quotient because reverse uses the actual
  degree instead of preserving the requested width. Nonmonic field division
  through PolynomialRingKaratsuba fails; ordinary PolynomialRing passes its
  corresponding control. No unqualified algorithm is imported into Hyper.
- Batch sparse x² evaluation throws at1,2,4,8points. Tree interpolation of square
  values passes at1,2points but throws at4,8; direct Lagrange controls pass.
  Three-point tree input is explicitly unsupported. Constant-zero batch
  controls at power-of-two sizes pass; do not generalize the sparse failure to
  all batch values.
- Fixed-size Fourier multiplication is cyclic once degree reaches n, not full
  K[x] multiplication. Oversized import truncates rather than consistently
  reducing modulo x^n-1. Transforming zero back leaves an empty polynomial whose
  zero predicate throws. Cyclic convolution itself is not classified as wrong;
  its restricted contract and conversion boundary are the issue.
- Multivariate parameterless leadingCoefficient selects the lowest stored
  monomial rather than the highest. A same-support equality control passes;
  different-support equality's source-derived null issue remains unqualified.
- For f(t)=t², parser f(2)+3 gives25 and f(2)*3 gives36. 1*-2 is rejected while
  parenthesized negation works. 2^3^2 gives64: left-associativity differs from
  conventional exponent notation, without an explicit documented associativity
  guarantee. Unmatched close/comma throw NPE; unmatched open becomes a checked
  evaluation error. IntegerPolynomial.parse(x+x) overwrites a term; parse(-x)
  throws NumberFormatException.
- Exhaustive uniform draw enumeration, not Monte Carlo, yields only4,11,40,162,
  788distinct permutations from6,24,120,720,5040equiprobable paths at sizes3..7.
  Size2passes. Size7multiplicities range1..34. Constructor-array aliasing also
  fails; RandomDerangement's remaining boundary/resource questions are open.
- Valid degree-one extensions Fp[x]/x expose missing gcd normalization: only
  constant1passes (6/68int and5/66BigInteger inverse cases). All68base-prime and
  all66normalized AlgebraicFieldExtension controls pass. These tests do not
  establish correctness of every nonconstant extension element.
- Squaring Gaussian/quadratic generators throws ArithmeticException because
  their defining polynomial has a linear coefficient represented as0/0.
  Nonconstant extension inverse, large-shift Tonelli-Shanks and Berlekamp-Rabin
  questions remain open; the single original square-root test passes.

## Verifying non-progress independently of timeouts

The uninstrumented trailing-zero and multivariate division calls each hit a
2second process cap. PolynomialProgress then subclasses only the supplied ring
callbacks, preserving every arithmetic result before deliberately throwing a
private budget exception. Both modes verify1,025observations per path:

- Univariate dividend1+0x and divisor1 are unchanged on each loop condition;
  the stored leading zero skips every update. No internal changing counter is
  involved, so the source transition explains the nonterminal state.
- Multivariate x/x overF5 repeatedly adds rather than subtracts its leading
  multiple; coefficients cycle1,2,4,3. The callback asserts both input
  coefficients at every step. It is not a floating-point convergence issue.

Both prefix-check processes exit0 at their explicit budget. Source/class hashes
are in polynomial-progress-build-manifest.json; identical output SHA256 is
`d3eccd50fb7e86c1c9ad06e7a65cdbd2fc25b0ab8156b333e21447fb139f8092`.
Timeouts alone are not used as a proof of nontermination.

## Current Hyper comparison

The isolated publish=false polynomial-hyper harness builds offline against
current Hyperreal/Hypersolve and their local dependencies. It snapshots201source/
manifest files in Hyperreal, Hyperlattice, Hyperlimit and Hypersolve before and
after execution; hashes remain stable. Debug and release each pass1,520checks:

- 420 exact integer evaluations.
- 675 exact monic/nonmonic divisibility cases using leading scales -3,1,2.
- 117 sparse monomial divisions through degree16.
- 49 monic gcd-normalization cases.
- 243 bivariate exact divisions against independently constructed coefficient grids.
- 16 empty/trailing-zero/domain and radical-identity controls.

Output SHA256 `93f9b74fa61caa4eb7789da201fcff4f776015989bcb6c96f7740a8c6ab0d545`.
Harness source `8b47ccec76324c2f439b51cb5dba1f3a64d1dd7cd411b8db5ffda7e92dc2bbe4`.
Debug binary `21b87bc5d7808e6c28c3f026004eeb4d6ed025460687240fec0347cf25da245c`.
Release binary `1f3f0b3ab035bce291961917db0949fc7bbad982ab3ec1b62aafb966a95f07bf`.

Source inspection confirms strict trimming and explicit rejection of undecidable
coefficients/zero divisors, certified leading-term cancellation, retained
divisor inverses, rational Horner and balanced long nonrational evaluation.
Those are existing Hyper capabilities, not new changes. Characteristic-p fields,
text parsers and random permutations do not have a claimed matching Hyper API
in this harness. The earlier full downstream suites were not rerun in this
no-production-change slice; older Hypercurve results retain their snapshot caveat.

## Performance experiment

The initial PolynomialBench run completed the16/32dense family, then failed
the sparse family's1,048,576-loop calibration allowance before any measured
sparse batch. Its stdout, stderr, source, class and manifest remain preserved;
the failure is explicitly `AssertionError: calibration cap`, not a wrong product.

PolynomialBenchExtended keeps the same input generator, verification, timer and
workload helper, changing only calibration/warmup loop allowances to16,777,216.
It reruns the full12families independently: lengths16,64,128; coefficient
parameters32,2048bits; dense and stride-eight sparse maps (last coefficient
also retained). Coefficient parameters shift small signed integers and add
small offsets; they do not mean every coefficient has exactly that bit length.

Each family uses a fresh JVM, CPU6affinity, common-pool parallelism1, assertions,
256–512MiB heap and1MiB stack. Each algorithm warms for≥0.5sprocess CPU and
calibrates a fixed batch for≥1sCPU. Three rotated warmup rounds precede nine
rotated measured pairs. Every measured batch must use≥0.5sCPU or fail the
entire process. Every product is checked coefficient-by-coefficient inside the
timer against a separately prepared exact BigInteger convolution; no failed
result is accepted as fast. CPU and wall times are both preserved.

Shared host, not an exclusively reserved CPU. No other heavy work was launched
by this audit during measurement. Nine within-JVM pairs support descriptive
20,000-resample fixed-seed bootstrap intervals, not independent-process
reproducibility guarantees. Ratios compare Karatsuba/ordinary donor product
costs including full verification; they are not Hyper speedups.

The extended benchmark completes all12processes:288observations,216measured
batches,108paired rounds and27,753,408exactly checked measured products. The
shortest measured batch uses1.15sCPU; none is rejected for short duration or
wrong arithmetic. Initial failed-cap artifacts are validated separately and
are not merged into these measurements. CPU ratios below are Karatsuba/ordinary;
full raw ratios and descriptive bootstrap intervals are in polynomial-bench-analysis.json.

| Length | Coefficient parameter | Dense CPU ratio [95% interval] | Sparse CPU ratio [interval] |
| ---: | ---: | ---: | ---: |
| 16 | 32 | 5.38 [5.25,5.47] | 9.43 [9.29,9.58] |
| 16 | 2048 | 0.960 [0.952,0.969] | 2.21 [2.16,2.24] |
| 64 | 32 | 2.24 [2.18,2.29] | 26.47 [26.14,26.51] |
| 64 | 2048 | 0.596 [0.579,0.607] | 3.76 [3.71,3.82] |
| 128 | 32 | 1.62 [1.56,1.63] | 28.34 [27.94,28.39] |
| 128 | 2048 | 0.447 [0.441,0.458] | 4.14 [3.87,4.23] |

Wall medians corroborate the dense64/2048and128/2048wins (0.595,0.451) and the
large sparse regressions. This warrants a **future isolated Hyper experiment**
gated by coefficient cost and density, not blind replacement of polynomial
multiplication. Hyper's rational backend already uses integer Karatsuba/Toom
dispatch, but that is different from polynomial-level Karatsuba. Any candidate
must compare against the actual current Hyper polynomial callers, include small,
sparse, unbalanced and symbolic coefficients, verify exact remainder/root
behavior, and measure allocations/code-size before retention. The donor timing
does not establish a Hyper crossover or authorize a lower-priority regression.
No production implementation of that candidate has been made in this slice.

## Class-group native follow-up

After all polynomial timing processes finished, run-classgroup.mjs compiled the
two unchanged class-group main files and a separate harness. Direct compilation
of the unchanged original test file fails on its obsolete
finitefields.quadraticform imports. The original log and source are preserved;
the tests are also outside the standard Maven source tree. No claim that those
five JUnit tests ran unchanged is made.

Both execution modes produce146identical probe rows:80pass,66wrong. In a
negative-discriminant grid-1..-128, all64valid-congruence controls construct the
right principal discriminant, while all64invalid-congruence inputs are accepted
instead of rejected. Object.equals is true before either discriminant cache is
warm, false after only one is warmed, and true after both are warm. Equal
objects have differing inherited identity hashes in both runs. The Group's
coefficient-based equality remains stable; it must not be conflated with the
object failure. Reduction, inverse/product, and the supplied order3/order7
coefficient controls pass. Sampling and general composition remain unqualified.

Output SHA256 `906dc78d306a1454f9b929d0b824a06c0bb5113dbaf0f6e9156686f655d89c0a`.
All class-group processes terminate normally; source/classes are pinned in
classgroup-build-manifest.json and checked by analyze-classgroup.mjs.

## Scope and retention

No new production change is retained so far. Read coverage does not claim that
all donor boundary hypotheses are tested; outstanding ones are explicitly in
the file notes. The remaining64files include elliptic code, demos/constant data
and the root SVG. Full Ruffini and ecosystem goal remain ACTIVE/OPEN, not
complete or blocked. No agents, donor edits, commits, pushes or deletions.

Final aggregate `verify-checkpoint.mjs` passes for all earlier scalar/shared,
matrix and new polynomial/class-group artifacts, including the preserved failed
calibration attempt. Current Hyperreal HEAD remains bd92d87; prior retained
repair hashes match and git diff --check passes. Hyperlattice/Hyperlimit/
Hypersolve and donor checkout are clean. All owned processes are terminal.
The next concrete action is an isolated Hyper polynomial Karatsuba experiment;
production retention requires its own correctness, representative performance,
memory and size evidence. Remaining source and other referenced systems are
not removed from scope by that experiment.
