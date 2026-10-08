# Escardó signed-binary reals — source, qualification and transfer closed

Source: https://github.com/martinescardo/ExactRealNumberComputationInHaskell
Snapshot: 971ff20d35a4af4be595364c704ad118c4bf3c37 (2025-11-14).
All five tracked regular text files / 1,995 physical lines read. No symlinks,
submodules, binaries or repository AGENTS instructions are present. Inventory
and read ranges are in ESCARDO_FILE_INVENTORY.tsv / ESCARDO_READ_COVERAGE.tsv.

| File | File/line audit findings |
| --- | --- |
| .gitignore | Six editor/compiler/executable exclusions. |
| LICENSE | CC0 1.0 text; no donor code imported into Hyper production. |
| mdtohs.hs | 36-line fenced-Haskell extractor; only haskell fences copied, not hs fences. Indentation removed from copied blocks. No parser/build framework beyond line-prefix recognition. |
| README.md | All 1,291 lines read, including code, proofs/explanations, examples and historical tables. Literate source of the generated Haskell. Covers signed redundant streams on [-1,1], productive bigMid, clipped arithmetic, four multiplication forms, specialized square, logistic iteration, scaled elementary series, searchable streams, modulus, extrema, integration, partial comparison, overlapping-root search and continuous branch gluing. General R is only a mantissa/exponent alias; arithmetic on general R is explicitly not implemented. |
| fun2011.hs | All 541 generated lines read independently. Lazy Int lists share prefixes but have no precision-indexed finest-integer cache. Zero/endpoint divideBy cases reduce lookahead; specialized multiply/square exploit leading-zero digits. bigMid uses a wider digit alphabet to break unproductive recursive-mid dependencies. General Int scale/series indices are unchecked machine integers. fromDouble terminates after55 digits, suspicious for retained exact tiny binary inputs. squareRoot uses inverse sqr over [-1,1], although sqr is monotone only on [0,1]; zero boundary needs independent qualification. Deliberate buggyMul is a documented negative control, not an accidental new finding. |

Semantic cautions: digit alphabets, infinite-list productivity, positivity and
function-domain promises are aliases/conventions, not enforced types. Negative
comparison of zero and nonnegative decimal output at decimal boundaries are
explicitly partial; do not label those declared limitations implementation
bugs. forEveryI searches only +/-1 representatives, whereas forEveryI' covers
all signed representatives. Predicates must be total/continuous on the searched
stream domain, and representation-invariant when interpreted as real-number
properties. The text's infinitely weighted-sum exponent has a prose typo;
implementation denotation is sum x_n / 2^(n+1).

Unchanged GHC9.6.7 -O2 build passes and prints the advertised 500-character pi
string. GHC reports an unrecognized warning-disable flag, not a build failure.
The generator completes: its output differs only in four indentation lines
inside the deliberately broken multiplication example. The numerical probes
use a separate copy with ONLY `module Escardo where` added, because implicit
Main exports only main; no numerical compatibility changes are required.

## Independent exact-prefix qualification

A proper n-digit signed-binary prefix encloses its denotation within 2^-n.
The oracle uses exact Rational arithmetic and also checks the digit alphabet.
The ordinary grid tests all 27 length-three prefixes with each of three
constant tails (81 valid, deliberately redundant representations), at multiple
output lengths. It runs 185,061 checks: 2,700 fail, so the donor is NOT qualified
as a correct scalar implementation. Failures are not the intentional buggyMul.

- Selected mul_version2 fails 1,554/19,683 product controls. In its `(a0:0:x)` /
  `(b0:1:y)` specialization, `p' = 0:0:x` drops the a0 contribution. Half times
  three quarters emits one quarter instead of three eighths. Versions0,1,3
  each pass the same 19,683 controls; specialized sqr passes243 controls.
- imin and imax each fail540/19,683 controls. The mixed -1/0-leading-digit
  cases of imin use oneMinus where the residual comparison needs addOne.
  min(-1/4,-3/8) emits -1/4. The continuous-gluing pmax alternative passes.
- fromDouble fails66/120 exact-import controls. Its fixed55-digit conversion
  is approximate even for simple representable doubles: zero becomes2^-55,
  one becomes1-2^-55. This is a fixed-precision conversion limitation, not an
  arbitrarily accurate import contract; using it to supply exact inputs would
  contaminate tests. Other numerical probes construct Rational streams directly.
- squareRoot fails9/18 independent nonnegative root-enclosure checks. Both
  zero and1/16 emit an all-minus-one prefix, approaching -1, instead of0 and1/4.
  inverse requires an increasing bijection on[-1,1], violated by sqr. The text
  observes sqr is bijective only on[0,1], but the implementation does not restrict
  the inverse's search domain. This is an actual incorrect output, not merely
  declared nontermination at a comparison boundary.
- The remaining ordinary unary, division, integer-multiple and periodic bigMid
  controls pass. In a separate suite,72 simple extremum/integration controls,
  48 finite-prefix quantifier controls and36 affine trisection controls pass.
  These small functional tests do not override the concrete min/max defects.

## Supporting papers (entire texts read)

Escardo-Simpson-interval.pdf, official author-hosted 27-page paper, has1,641
extracted lines; all were read, including all appendices. Printed equations on
pages9 and21 were additionally visually checked. The midpoint-algebra/free
interval object explains productive bigMid, cancellation, affine maps and
bounded convexity. It distinguishes constructive Cauchy/Euclidean/Dedekind
completions instead of treating arbitrary approximation closures as interchangeable.
There are printed formula defects: page9's rescaling uses a negative exponent
where page20 uses the required positive one; page21's displayed Cauchy-limit
formula yields c/2 for the constant sequence c. Do not transplant these formulas
as certified numerical convergence algorithms. This observation is not a claim
that the paper's foundational characterization as a whole is false.

Simpson-1998-functional-integration.pdf, official Edinburgh alternate host, has
461 extracted lines and10 PDF pages including the university cover. All lines
and all nine paper pages were read visually (text extraction corrupts symbols).
It distinguishes total stream functions from representation-respecting real
functions, intensional prefix demand from extensional continuity, and makes
the one-digit-lookahead extremum recurrence explicit. Its maximum recurrence
uses subOne, supporting the mixed-prefix min diagnosis above. Integration uses
a different dyadic-stream representation for componentwise averaging with
one-digit lookahead. The paper explicitly reports poor practical integration
performance; historical observations are not current measurements. Proofs are
mathematical/informal, not a machine-checked guarantee for the 2025 tutorial.

## Extended qualification and measurements

- Native directed8192-bit MPFR grid:455 outputs,115 failures. Of55 cases per
  scaled operation, failures are exp15, sin10, cos26, atan10, asin7, log14,
  log(x+1)/x14, reciprocal15. Takano pi/4 fails4/7, BBP pi/32 passes8/8.
  All498 printed native decimal digits of BBP pi pass an independent directed
  MPFR floor-of-scaled-pi oracle. A2048-bit Takano request exceeded90 seconds;
  the final bounded table omits that unachieved request and is byte-identical
  to the455 completed rows preserved from the initial run.
- Hyper21e76ea passes1,820 matching elementary/history checks plus238 exact
  product/root/float/history controls and two resolved min/max controls.
  Hyper's borrowing min/max explicitly use certified partial-order selection;
  they are NOT claimed to be total continuous extrema on unresolved inputs.
- Extra exact controls: corrected multiplication26,244 pass; wider-alphabet
  normalization41,148 pass; domain-restricted square root35 pass; continuous
  gluing at undecided zero/equal real branches9,807 pass; old two-digit
  normalization1,125 pass. Four explicit lookahead boundary controls pass.
  Native machine-width divisor tests fail6/36 and general normalization fails
  9/12 near Int maximum: valid large Int parameters overflow intermediate
  a+2s /2a+b arithmetic. These are separate from the small-alphabet successes.
- negative zero, decimal half, dyadic-root bisection and naive recursive bigMid
  each time out at3 seconds, consistent with their stated partiality or the
  tutorial's productivity warning. The bounded Memcheck ordinary workload
  completes with zero memory errors; numerical failures still reproduce.
- 243 CPU6-pinned process-CPU/managed-allocation observations force complete
  outputs and validate every result against an exact Rational enclosure. The
  audit-only corrected zero-prefix mul2 improves512-bit dense products from
  11.21ms to10.45ms, zero-prefixed from9.02ms to3.27ms, terminating from5.97ms
  to0.595ms versus native mul1. Specialized square roughly halves allocation
  versus generic self multiplication. These are within-GHC kernel comparisons,
  not cross-language Hyper speed claims, and broken native mul2 is excluded.
- Another162 observations repeat normalization with larger batches/nine
  seeds. Native one-digit fast paths reduce managed allocation, but runtime
  is mixed: dense32-bit medians3.32us vs2.85us for always-two-digit, endpoint
  512-bit31.53us vs34.39us. The robust benefit is productivity/lookahead, not
  a universal speed claim. Every row has a unique variant/family/bits/seed key.

Hyper already specializes square/zero/magnitude demand and stores dyadic scales
as offsets. Productive stream bigMid and quantification over all continuous
closures remain broader capabilities, not falsely claimed subsumed by finite
Real sums or partial predicate reports. No demonstrated current consumer
justifies replacing Hyper's scalar representation with lazy Int lists.

RETAINED Hyperreal bd92d87: directly approximate sqrt(Square(x)) as abs(approx(x,p))
when construction cannot prove x's sign. The1-Lipschitz inequality preserves
the existing one-unit integer approximation contract without a new node or
arbitrary predicate closure. This is inspired by the continuous-absolute-value
comparison, not copied donor code. The first build exposed a private-field
boundary; an internal square-operand query keeps that boundary intact. Original
21e76ea and the isolated candidate each pass966 independent exact/MPFR/history/
serde controls plus one public Real::abs lossy-zero boundary. New production
regressions check385 raw-node rational precision/history cases and explicit
unresolved-sign, no-intermediate-cache and abort-without-cache-publication paths.

The final480-row A/B has432 CPU6-pinned timing observations (nine alternating
seeds, freshly built graphs) and48 separate allocation-count observations.
Ordinary unresolved positive/negative absolute values improve2.68--3.70x and
reduce cumulative allocated bytes49--78%. Opaque trigonometric cancellation
plus/minus2^-256 improves2.01--2.69x once the demand resolves the perturbation.
Exact zero is effectively unchanged at2048bits. Ordinary sqrt controls have
identical allocation and remain within1.5% CPU; the noisy short336-row pilot
is preserved but excluded from conclusions. All timed outputs are forced and
checked independently after timing, not accepted through mutual agreement.

Production all-feature debug/release gates pass826 tests each; all-target/
all-feature Clippy -D warnings, cargo fmt and git diff checks pass. Default
downstream library-only gates pass Hyperlattice19, Hyperlimit242, Hypertri3,
Hypersolve429, after the original native-cache sandbox failure was retried
with approval and CCACHE_DISABLE=1. These693 are scoped library controls,
not the complete downstream integration/feature matrix. The candidate oracle
Memcheck workload completes with zero memory errors; no full leak claim.

The linked serde/oracle driver changes text by-368 bytes, data by0 and BSS
by+384; scalar layouts and wire variants are unchanged. Four production files
are committed:14 runtime/helper lines,52 regression lines and26 performance
documentation lines. Hyperreal is clean after bd92d87; no donor code copied.
Native/source/targeted transfer qualification is closed. General searchable
function spaces, continuous branch-closure nodes and streaming replacement
representations are not selected; their wider contracts are explicitly noted
above rather than claimed subsumed. Remaining external donors remain pending.

Audit data/prototype moved from /tmp/escardo-audit.A6Oxth to workspace
.audit-escardo.A6Oxth after the user's51,490MB tmpfs quota filled; a symlink
preserves all original paths. No evidence was deleted. Normal build temporary
files now use the workspace directory. Original donor files remain unchanged.
