# Plume source audit

Source, scoped numerical qualification and targeted Hyper transfer comparison
complete, with generated-container/native-build limitations stated below.
The authoritative source ranges are in PLUME_READ_COVERAGE.tsv, with hashes
in PLUME_FILE_INVENTORY.tsv; report coverage is tracked separately.
Priority: exactness, completeness, performance, memory, binary size, source size.

## Provenance and scope

The Edinburgh 1998 report survives at https://www.dcs.ed.ac.uk/home/mhe/plume/.
Original archives are preserved in the supervisor's public repository,
https://github.com/martinescardo/martinescardo.github.io/tree/2f2aa72457d14f2363a741af1ebb52932a48d6ec/dbp.
The commit is2f2aa72457d14f2363a741af1ebb52932a48d6ec;
its tree is06dc5f3ceea059320cc8a58f3cb0afb1c0fb2c62.
Downloaded archives match the Git blob hashes from the public tree.

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| v1.2.tar.gz | 77791 | 67ff10734916f9fd433aeb4d9a51c0f6ba6fde4b86bcebc648b45c166b648716 |
| cgi_source.tar.gz | 58786 | ce1d96a3cc3705debac9357958411daa4751ee45b8a3c0a0a6ca385f7309c777 |
| performance.tar.gz | 29446 | 74d48a41012a1d2ed5042f6cf690b9ba1bbba7a85b90f43729f9731eb2c37f95 |
| solaris_demo.tar.gz | 1457472 | e28980237f928beba4b0e3ce48b9722c12ab846610d586d626ca963f10b09358 |
| references.bib | 9112 | 97a4b5aa07fd6d5f8515b6f86cf7cdbc18d73ccf1f4a993d89eb355670631eaa |

All archive member listings were read before extraction: regular files and
relative directories only, no links or traversal paths. The four executables
are32-bit big-endian SPARC ELF files and have not been executed. There are174
extracted/support files,170 text files/24,049 lines,102 unique text contents.
Copies are not automatically credited as independently read. Generated and
vendored files will be classified explicitly under the root scope convention.
No license grant was found in the two README files; do not copy donor code.

The official report.ps.gz SHA-256 is
a9c94e9257d45622c5dd42f7349dce42cbb2232b02881dc2cc0b3a4519ea11d2;
decompressed report.ps is
a6faa2e4a5d979f8e2bd5b8e77280fb21d2d3597d6a7967418c1f99e64b0bc2d.
Local Ghostscript conversion gives133 pages; pdftotext -layout gives4,871
lines. All report.txt1-4871 and all133 primary report pages have now been read.
Mathematical extraction has known layout/glyph losses; discrepancies are
recorded below after visual checking. The independently
published author-report.pdf is also preserved, not byte-identical to the
local conversion. Report HTML node91(80),node97(82),node147(63) is fully read;
index.html1-618 is now also fully read as raw HTML. node91's diagram img216.gif
was independently acquired and visually read during report closure.
dbp-index.html is a404 response, excluded from donor coverage.
Initial physical PDF pages35 and40 (printed27,32) were visually read; the
expanded selected-page list is in PLUME_REPORT_COVERAGE.tsv. The affine
rescaling formula on27 is correct; text extraction was misleading. The
conversion formula on32 genuinely subtracts/adds1/4 instead of1/2. On input
(1:0:0:...), the printed formula continually emits1 and maps1/2 to1. The
implementation uses1/2 and passes the independent conversion grids.

## Read file notes — main v1.2

- README/README~: module inventory and historical file listing; Main/IMain
  claim version1.0 although archive is v1.2. Main and IMain are separate Main
  modules and must be built separately. Interact-based calculator state.
- Makefile: old GHC/Happy/Alex flags and generated parser dependencies. Do not
  run its destructive clean targets; use isolated output directories.
- Utils: generic take/drop use obsolete toInt conversion without an explicit
  overflow check; finite-list head and predicates are intentionally partial.
- DyDigit: pair(a,b) represents a/2^b. Most operations assume canonical pairs;
  public dyd checks range but does not normalize, while dydEq compares pairs
  structurally. dydShr can produce noncanonical zero. Normalize removes factors
  of two. AddRC/SubRC clip a first component to[-1,1] and return a residual.
- DyStream: static defects awaiting oracle qualification: dys_minusThird has
  the same positive stream as dys_Third; the lowercase pattern dyd_Zero in
  dysDigAdd is a fresh variable and matches all digits, dropping every carry;
  GHC confirms the next clause is redundant. dysNormMax recursion does not
  decrement max. dysP also appears to drop a nonzero first digit when its
  clipped residual is zero. Componentwise averaging is one-lookahead but
  dyadic mantissa sizes can grow; multiplication expands three recursive paths.
- SBinStream: signed digits are Int; zero-prefix multiplication and prefix
  recoding preserve scale without first deciding sign. Optimized two-digit
  multiplication shares xavy; unused sbMul2 is a general recurrence. NormExtra
  only uses the extra recoding at the first step, then calls plain Norm.
  sbAbs is productive at zero; sbAbsSign's sign projection is not. Native Int
  integer-division recurrence risks overflow and handles negative divisors
  other than-1 without sign normalization. sbGTE's swapped fallback omits
  inversion; exact equality is deliberately partial. Test before accepting any
  arithmetic or min/max/comparison claim.
- SBinFloat: Integer exponent avoids fixed exponent overflow but operations
  still use Int for some counters/conversions. sbfFour equals sbfTwo in source.
  Multiplication switches to unbounded normalization at exponent sum>=500,
  risking a productive-zero regression. Add/sub align mantissas and increase
  exponent by one. Normalization is not an exact-zero decision.
- DyFloat: analogous exponent alignment; quarter constants equal half
  constants. Carries and normalization inherit the underlying stream defects.
- Convert: signed digits inject directly into dyadic pairs; dyadic-to-signed
  conversion examines two digits and emits with an overlapping threshold,
  retaining only a bounded residual. The old alternate multiply is commented
  out and not an active implementation.
- IReal: recursively nested interval endpoint streams use a continuous
  banana-bracket mediator. Initial exponent normalization is capped at500,
  unlike sbfBetween's unbounded upper-endpoint normalization. Domain requires
  ordered nested enclosures; no certification object checks that contract.
  An explicit modulus is not required for productivity of a valid shrinking
  interval stream, but arbitrary input intervals cannot be assumed valid.
- Functions: all465 lines read. Square root uses a25-digit Double seed followed
  by Newton enclosures; zero forces an undefined quotient/normalization path.
  Log attempts to reduce to a bounded ln1p domain, but its exponent1 branch
  accepts all positive-leading mantissas, including values outside the stated
  convergence range. Subsequent bit-count truncation is not a valid remainder
  bound there. ln(2) depends on representation: the endpoint stream tends to
  0.625 while decimal/terminating inputs produce the right value. ln(8) also
  fails at larger demands. Exponential uses sign-aware interval streams, with
  an unknown-sign branch near zero. Trig series have no efficient large-argument
  reduction. The active pi formula uses atan(1/8),atan(1/57),atan(1/239);
  it passes the scoped MPFR probes. Old unused series variants remain in file.
- X_Repn: division first removes leading-zero/canceling prefixes from the
  denominator and emits dyadic coefficients in{0,+/-1/4,+/-1/2,+/-1}; output
  scale compensates by4. Two invalid-denominator guards are unreachable after
  earlier broad patterns, confirmed by GHC. The public sbfDiv corrects inputs;
  direct sbDiv retains a narrower domain contract. Separate xsbMul avoids the
  defective dysAdd/shift helpers and passes numerical checks.
- SBinDec: exact finite decimal import uses Horner forms and integer division;
  output converts finite signed-decimal prefixes and is an approximation, not
  certified correctly rounded text. Machine-Int exponent/count conversions,
  carry conventions, and unvalidated direct strings need care. The focused195
  valid-decimal imports pass. Broader malformed/rounding cases remain open.
- Integr: searchable binary-prefix witnesses find common output digits;
  integrate uses dyadic averages of all branch outcomes, then converts. Whole-
  line function wrappers search for a uniform exponent on the parameter domain.
  The unused sfnmind alternate calls sfnmaxd on one branch, a static typo;
  public fnmin uses sfnmin1, not that alternate. Partial/noncontinuous closures
  do not satisfy the compact-search contract. Of270 scoped constant/affine/
  quadratic/degenerate/reversed-bound controls,236 pass;34 hit a1s request cap,
  mostly integration. No finite wrong functional result observed; timeouts
  are not proofs of nontermination.
- Calc/ICalc: expression trees are evaluated recursively using Maybe for missing
  variables. Definitions are stored as ASTs, so later variable reassignment
  changes existing dependent definitions; it is not an immutable value DAG.
  ICalc's higher-order evaluator has no nested-functional cases. Scalar abs is
  not in the grammar. Parser/lexer specifications and both AST modules are read;
  four generated .hs tables are now independently line-read (2187 lines total),
  as well as compiled; they match the CGI copies byte-for-byte. Ordinary calculator
  smoke checks cover arithmetic precedence, signs, functions, variables and
  lexical errors. Both calculator entry points compatibility-build.
- Database/IDatabase: linear association list, remove-on-store filtering; the
  integration database throws on a missing variable, unlike ordinary NoVar.
  dbDigits is explicitly undefined. Interact is a pure String transformer with
  line editing; EOF without a newline has a partial tail. Alex runtime379 lines
  is read and recognized as vendored scanner support, not novel scalar code.
- Demos/Tests/Test/demo1: logistic-map streams and print/tabulate helpers, not
  an assertion-based numerical suite. Signed, cross-representation and dyadic
  variants have different normalization/shift behavior and cannot be assumed
  equivalent. Higher-iteration logistic-map qualification remains pending.

## Build evidence

Unmodified modern GHC9.6.7 build of Main stops on the old Array module import
and missing toInt in Utils. The log is /tmp/plume-native-build.log; no runtime
success is claimed. DyDigit, DyStream and DyFloat compile before that failure.
The isolated copy at workspace .audit-plume.SZHOs8/compat adds only old Prelude
conversion aliases, Data.Array/fmap and Nothing for obsolete Maybe zero. The
exact diff is plume-qualification/compatibility.patch. Numerical formulas remain
byte-identical. Both calculators and the numerical probe build; the original
snapshot is still unmodified and still does not build natively on modern GHC.
No Hyper source was changed for Plume.

## Performance variant

versioned/perform/README: digit-lookahead instrumentation and Perl graph tools;
explicitly described as incompletely tested. LookInt214 lines and sbMul.fix67
lines are also read. LookInt attaches a maximum input-index tag to each digit;
Boolean comparisons discard the tag, so branch-dependent result constructors
must explicitly restore it. Its compare subtracts Int values and can overflow.
sbMul.fix manually carries the maximum first-four-digit tag across its main
cases but uses fresh zero-tag literals in prefix rewrites. The later active
SBinStreamM rewrite uses tagged literals, but still misses control dependencies.
All36 versioned/perform files/4365 physical lines are now read independently;
the separate performance.tar.gz copies were subsequently independently read
in full (all36 files/4365 lines). They are byte-identical to these counterparts;
source-copy-map.json records every pairing. The earlier qualification applies
by identity, not by new measurements. Figure text was reread; no second visual
render is claimed for these identical copies.

File-level comparison and dispositions:

- ConvertM.hs1-84: paired-dyadic conversion preserves the same correct residual
  formulas as the main copy. Uses the hidden Lookint constructor, requiring an
  export bridge on modern GHC. All tested roundtrips and cross-products pass;
  conversion forces two extra input digits in the finite demand grid.
- DyDigitM.hs1-288: dyadic numerator/exponent plus maximum tag; exact arithmetic
  still assumes canonical inputs. Boolean comparisons discard dependencies.
  DyStreamM.hs1-167 repeats the shadowed carry pattern and broken subtraction,
  positive minus-third constant and nondecrementing normalization cap. These
  are not safer variants of the main implementation.
- DyFloatM.hs1-160: exponent alignment/scaled wrappers; unlike the stream
  logistic map, its floating map includes the factor4 and passes102 controls.
- SBinStreamM.hs1-489: useful zero-prefix multiplication and parity-sensitive
  average; alternative carry average consumes one extra digit. The helper
  mone_plus_twoposx calls the opposite zero recurrence, breaking21/243 capped
  double prefixes. Negative sbIntDiv fails967/972; unequal sbGTE retains the
  swap-without-negation defect (1980/6094). Ordinary arithmetic grids pass.
  Output tags from zero-prefix multiplication can omit previously inspected
  opposite-operand/control digits;486/19,683 multiplication demand rows are low.
  sbIntDiv also drops a tagged divisor: a forced divisor tagged1000 yields
  correctly valued output tagged only1..8. Tags are not dependency certificates.
- SBinFloatM.hs1-237: drops the main zero-divisor guard; zero recurses through
  even-factor stripping until the256MB heap limit. The exponent500 unbounded
  normalization switch remains; no portable scheduling gain is established.
- X_RepnM.hs1-90: moves invalid-prefix guards before general patterns and
  changes remainder recurrences. The negative remainder branch subtracts the
  denominator where the main implementation adds it. Canonical scaled540
  cases pass, but870/17,982 representation-sensitive divisions fail, including
  -1/(1/8) tending to-16 instead of-8. Do not trust only canonical test inputs.
- IRealM.hs1-117: chooses only an unboundedly normalized first upper endpoint;
  valid negative nested intervals can be outside the chosen mantissa range.
  A limit of-3/4 with first interval[-5/4,-1/4] tends to-1/2. First upper0
  exhausts256MB. Banana-bracket output tags omit some upper-bound control
  dependencies (source observation, not a complete dynamic interval-tag audit).
  Older untagged IReal.hs1-130 has the same exponent selection and repeats the
  three negative-limit failures when supplied main v1.2 stream dependencies.
- FunctionsM.hs1-397: sin and atan use ordered limits rather than main between;
  nine returned sine and six atan prefixes fail directed MPFR. The five log
  failures persist; exp27, cos21 and pi3 returned-prefix checks pass. Six zero
  requests hit256MB and two atan(+1) requests hit3s. No sqrt is exported here.
- SBinDecM.hs1-348: declares SBinDec, and raw-output formatting passes Int
  digits to Lookint output. Names/type bridges enable qualification without
  repairing numerical formulas. All195 imports pass, but96/585 outputs fail;
  exact2 can print1, -2 can print0, -32 can print-30. Default import generators
  use zero-tag digits, not the commented lookahead-retagging wrappers.
- TestsM.hs1-90: signed-stream and signed-/dyadic-float logistic maps include
  factor4; dyadic-stream map omits it and also uses broken subtraction. That
  stream fails90/102 true-logistic checks, and74/102 checks against even the
  no-factor4 recurrence. Testing.hs1-23 uses the parity average rather than
  alternate average in its signed logistic map; both signed variants pass102.
  Both original files declare Tests; Testing exports nothing. Audit-only name/
  export bridges permit calls; recursive bodies remain unchanged.
- Utils.hs1-66: old Prelude conversion names require aliases; no numerical
  alternative. LookInt.hs1-214 has three wrong machine-boundary compare
  results in four controls. README1-11 explicitly warns that this is unfinished
  performance instrumentation. sbMul.fix1-67 is not the active tested routine.
- plot_look1-12 and plotf1-10 use numeric empty-string comparison, dropping zero
  values, and start indices at1 versus0. plotfig1-54 emits raw debug rows into
  FIG coordinate data. res1/136 rows and res2/25 rows contain numerical data;
  res2 is the first25 rows of res1. xplot/28 rows is data, not an executable.
- Every coordinate/label and EPS procedure in the thirteen figure artifacts
  is read (exact ranges in coverage). abyb/abyb2/thirdbyone compare digit demand
  with n..4n, not wall time; form/form2 are plotting fragments. real_sbmul and
  its backup label25 as35. Both original EPS files render and were visually
  read. Modern fig2dev rejects negative transparent-color headers; test2 with
  only that header bridged renders but incorporates debug pairs as coordinates
  and truncates the intended curve. A successful renderer is not graph proof.

Qualified performance-variant evidence is separate from the main checkpoint:
instrumented-qualification-summary.json, instrumented-README.md and the raw
tables under plume-qualification/. Optimized/unoptimized grids, actual-demand
rows and examples are byte-identical. Benchmarks compare only the two valid
averaging kernels:192 final paired-order observations from216 including warmup.
On dense inputs the lower-lookahead average has1.18--1.30x median CPU and
1.35--1.43x cumulative allocation of the carry average. Sparse/terminating cases
are mixed; lower demand can still help when input evaluation is expensive.
These are instrumented Haskell kernels with pre-forced inputs, not a Hyper
speed claim. Submillisecond timings and host variability are explicitly retained
in per-group min/max and paired statistics.

Current Hyper comparison: computable/format.rs1-577 uses integer magnitude,
explicit sign and decimal carry handling; it does not re-use signed-stream
decimal logic. Extra195-input/three-place/two-history formatting controls pass
1170 exact GMP checks. Its format tests23 and linear-demand tests5 pass in both
debug and release. Read node/linear_demand_tests.rs1-226,
node/representation.rs1-82 and node/approximation_queries.rs135-198: the
saturating demand hint changes operand order only, never requested precision
or enclosure validity, and is excluded from serialization. That is distinct
from claiming the hint is a measured complete dependency trace. No per-digit
metadata, stream representation or alternate kernel is selected for Hyper.

## CGI archive closure: source reading, not web deployment

All57 files/8110 physical lines are now independently read, including the31
byte-identical main v1.2 files and26 different/new files. The comparison map
verifies every original hash. Lexer.hs169,ILexer.hs189,Parser.hs834,IParser.hs995
were read literally including all DFA transitions, Happy state actions,
semantic reductions and stack/error-recovery code. One oversized combined
lexer output was truncated; missing ranges were reread before granting credit.
Main v1.2 generated copies are also now independently read in full; their
byte identity transfers the syntax checks without inventing another test run.

- CGI.hs33: request-to-worker-to-response orchestration; tryRead requires an
  empty remainder. No scalar state or arithmetic implementation.
- CGISystem.hs54,EnvPassed.hs24: legacy System/catch APIs and fixed CGI variable
  list; environment/error-log display scaffolding is not a scalar idea. No
  actual environment display or hardcoded log-writing path was executed.
- Request.hs39,Response.hs55,Mime.hs60: method/MIME wrappers and output headers;
  unsupported methods/types have fallback behavior, not explicit diagnostics.
  UrlEncoded output uses Haskell show rather than a form encoder. No HTTP
  service or protocol/vulnerability test was performed for this scalar audit.
- URL.hs3,UrlEncoded.hs76: String alias and first-prefix parse/query lookup;
  repeated keys choose the first entry and missing keys become empty strings.
  Parsing.hs191 uses list-valued nondeterministic parser results, old overloaded
  monad choice and first-result choice; whole-input consumption is not implicit.
  These contracts differ from CGI.tryRead's explicit empty-remainder check.
- HTML.hs59,HTMLWizard.hs371: AST and attribute-replacement combinators; image
  and unordered-list combinators emit IMAGE and OL respectively. They are not
  relevant scalar dispatch or compact numerical-formatting machinery.
- Pretty.hs242,PrettyHTML.hs45: cached document lengths, alternative layouts
  and ShowS builders; old strict/operator assumptions require compatibility.
  HTML text is passed through as text and attributes use Haskell show. No
  web correctness/security or full renderer qualification is claimed. Hyper
  already writes numerical formatting through fmt rather than this Doc tree.
- WebSite.hs82: flattening uses depth/sibling-index pairs, not full ancestry;
  these are not unique identities in general deeper trees. Reject as an
  identity model for Hyper DAG/provenance work; no implementation is copied.
- Counter.hs43,Demo.hs42,Hello.hs19,hellodave.hs19,EnvPassed.hs24: application
  demonstrations, hidden form counter and environment display. Hello and
  hellodave differ in the selected host field. Not numerical alternatives.
- Test.hs57: expression CGI worker calls calc with emptyDB and ten digits,
  discarding returned state/digit settings on each request. Test2.hs64 is a
  registration/log/redirect demo; it was only read, never executed. demo.hs21
  places the calculator in a one-page site; lowercase-module hello.hs25 is an
  unfinished/noncompiling experiment, not silently repaired.
- Makefile341: legacy compiler flags, CGI link targets and duplicate Test.o
  rules alongside copied calculator rules. No numerical optimization beyond
  ordinary compiler selection; destructive clean targets were not run.
- demo.cgi1,trial.cgi1: hardcoded Hugs commands; world.pl6: static greeting.
  No archived launcher or registration-side-effect program was run.
- All remaining31 files carry the corresponding main numerical/disposition
  notes above by exact identity. The generated ordinary parser contains58
  states and31 semantic rules; the integrated parser contains71 states and34
  rules. EOF acceptance and error recovery are explicit. The main power grammar
  accepts NFactor^NFactor, not arbitrary chains or unparenthesized power on the
  right of multiplication/division; the integrated lexer has no power token.

Native compile-only hellodave check fails (System,Char,Prelude <> ambiguity).
The isolated pure ParserProbe uses only the existing Data.Array/fmap bridge in
Alex; no CGI I/O modules are needed. O0/O2 both pass33938 independent lexer and
88 typed-AST/rejection checks. Checks cover all ASCII inputs through length2,
longest keyword/identifier/decimal boundaries and source positions, not an
exhaustive language proof. Source/read/qualification details are separate from
main numerical benchmarks. No parser benchmark or Hyper speed claim is needed:
there is no selected replacement or new performance candidate here.

Current Hyper real/arithmetic/format_parse.rs1-106 and rational/arithmetic/
format_parse.rs1-458 are read. Their scalar literal contract is not a general
expression parser. They already have small-word parsing, backend parsing,
large-input divide/conquer and bounded scientific expansion with Real's lazy
scale fallback. No generated-language layer or request-local AST database is
selected for Hyper. This syntax audit does not prove every accepted Plume AST
can evaluate: ICalc's nested-functional cases remain absent as noted above.

## Qualified numerical results and initial Hyper comparison

- Main exact grid212,860:24,305 failures, no exceptions. Extra grid39,465:
  107 machine-Int division failures; ordinary dyadic averages/multiplication,
  conversion, digit multiplication and195 valid decimal imports all pass.
-129 elementary requests:116 returned prefixes, nine256MB heap exhaustion
  results at zero and four3s atan(+/-1) timeouts. Directed8192-bit MPFR finds
  five bad returned log prefixes. A further24 logarithm representation controls
  find14 bad prefixes; raw rows and source representation are recorded jointly.
  The finite-prefix test uses exact dyadic inputs and inward output-interval
  rounding against outward MPFR results; it does not assume Double accuracy.
- Initial sandbox child-process permission errors were reproduced outside the
  sandbox with approval. All116 finite prefixes match, and the nine heap/four
  timeout classifications persist with no EPERM. Original logs remain under
  plume-qualification/sandbox-run/. UI likewise has an approved clean retry.
- Hyperreal bd92d87 passes all129 requested operations at four precision/history
  points (516 checks), including requests that Plume did not complete. It also
  passes26,232 exact Rational controls, including large integer divisors and
  zero products across the donor's exponent500 threshold. These public-control
  zero products may simplify; no raw-node coverage is claimed for them.
- Memcheck on the extra-grid workload completes with zero memory errors. Its
  numerical107 failures remain failures; no full leak/runtime qualification.
- Within-GHC multiply comparisons force fresh outputs from equally pre-forced
  inputs and independently verify every result using Rational arithmetic.
  CPU/allocation data and exact commands are preserved. There is no cross-
  language scalar-speed or binary-size claim. The final benchmark source is
  frozen in KernelProbe.bench-final.hs; preliminary observations are retained
  separately in pilot-bench/ and are not mixed into final statistics.
  Final approved run has216 observations,192 after excluding round0 warmup,
  and zero child-process errors. Dense dyadic multiply takes8.3--24.4x signed
  kernel CPU and10.5--14.1x cumulative allocation. Submillisecond timings have
  limited resolution; per-group min/max and all raw rows are preserved.
- Hyper already uses bounded magnitude-aware multiplication, direct Offset
  demand shifts, specialized squaring, exact rational/dyadic kernels and one
  synchronized finest integer approximation. Plume's useful zero-prefix and
  digit-swell lessons support those existing choices. Relevant current code:
  computable/approximation/arithmetic_kernels.rs1-255,
  approximation/logarithms.rs1-125, node/logarithms.rs165-320 and
  node/representation.rs165-222. The log precondition is explicitly checked by
  Hyper's construction/range reduction and its small-argument atanh kernel.
  Generic compact-domain functional closures and arbitrary nested-interval
  limits remain broader API ideas, not falsely claimed to exist in Hyper.
  No additional scalar change is selected from this slice.

## Remaining source-container support files

- references.bib1-375: historical citation metadata (digit arithmetic,
  constructive representations, functionals, Haskell/profiling and chaos),
  not another implementation. Reading these citations does not credit their
  referenced works as read or verify current link availability.
- solaris/test1.era1-6: thirty-digit cancellation example at x=77617,y=33096;
  test1.mws1-28 contains the same polynomial with decimal coefficients and an
  exact rational-coefficient control under several Maple Digits settings.
  No Maple execution or original SPARC executable run is claimed.
- solaris/test2.mws1-115: all serialized worksheet text/plot data read, not
  rendered. Examples are min(x*x-x) on [0,1] and the interior maximum of
  30-1/x-60*x near 1/sqrt(60), with an archived signed-prefix enclosure.
  Its [0,1] plot includes a singular endpoint; the explicit fmax interval
  [0.129099,0.1291] avoids zero. Encoded plots are not certified numerical data.

The entire 174-file source-container inventory is now accounted for:170 text
files/24049 physical lines independently read and4 SPARC binaries identified,
not executed. source-parser-summary.json validates the original hashes and
coverage plus the isolated parser results. The separately inventoried report
is still incomplete; this is not whole-Plume or ecosystem audit closure.

## Report continuation: text complete, selected visual verification

All report.txt1631-4871 is now independently read, completing4871 extracted
lines. Appendix Haskell, bibliography, sample digits and the long dyadic
numerator are included. The report is not a byte-equivalent executable source;
several printed pseudocode formulas disagree with the actual qualified source.
The following discrepancies were checked on rendered pages, not inferred only
from text extraction (printed page numbers below):

- p45: dyadic multiplication's displayed expansion has the right cross terms,
  but its following pseudocode repeats b*x twice. Independent GMP checks on
  all9^4=6561 quarter-grid tuples validate the correct identity;6016 reject
  the duplicated-cross-term transcription. This is not a new source failure.
- p47: moving -4e out of the middle expression requires +4e on both bounds,
  not -4e. All189 legal signed-digit/carry tuples pass the actual carry
  invariant and corrected inequality;88 reject the printed inequality.
- p48: the displayed average recursion drops the already-peeked a1/b1 digits
  and has inconsistent carry notation. The p86 Haskell retains those digits
  and computes c'=d'-2e. p49's multiplier pseudocode changes its q variables
  to undeclared a,b,c,d; its preceding semantic expansion uses a0,a1,b0,b1.
- p51 (repeated p84): the auxiliary integer-divider signature omits its s
  parameter, while recursive calls use the outer name with three arguments.
  The Haskell retains the auxiliary state; its negative-divisor/overflow
  defects are independently qualified above, not caused by this notation.
- p54 says4x/y, while the bounded mantissa kernel computes x/(4y). p56 writes
  r=y-x despite the prose and subsequent positive-case derivation needing x-y.
  Source X_Repn uses x-y and the scale offset+2. Do not port those printed
  expressions literally or infer a second numerical source defect.
- p65: initial sig is unspecified; the negative-output test uses the lower
  bound instead of the upper. The zero-output guard also uses an unprimed
  upper quantity. Appendix/source sbBB has explicit finite sig and correct
  output-side guards. The three-digit interval-width argument is useful under
  ordered valid shrinking enclosures, not a validator of arbitrary limits.
- p68: the sine recurrence has the wrong factorial factor and a repeated
  sign factor; the cosine sum mixes i/n indices and has power2n-1 with no
  constant term. The executable sin/cos recurrences use the appropriate
  factorial factors and initial values. p71's log sum starts with division
  by zero and omits alternating signs; source alternates correctly, but its
  independently demonstrated range/remainder faults still remain.
- pp116-117: the general dyadic-average denominator uses b+c+1, not b+d+1;
  the derived unequal-exponent rules use the correct alignment. The optional
  split-sum formula has overlapping conditions and reversed residual signs.
  Treat neither as a validated carry algorithm; source AddRC/SubRC controls
  are recorded separately. Canonical odd-numerator multiplication and direct
  exponent shifts are ordinary existing Hyper arithmetic, not a new kernel.
- Figure5.1/p63 is visually read: ordered lower/upper information may force
  a value even when individual representation ranges overlap. Figure7.1/p91
  shows recursive average branching/two-digit productivity; figures7.2-7.3
  pp99-100 repeat the lookahead graphs, including the25 tick mislabeled35.
  Figure7.4/p102 is the historical heap-residency profile for ln(2), not
  cumulative allocation or current Hyper evidence. FigureC.1/p123 says digits
  and bye, whereas actual lexer keywords are Digits and exit. Its restricted
  power grammar matches the qualified source. Remaining visual pages open.

Chapter5's general compact-domain functional closures and bound-driven limits
remain genuine completeness ideas. They require total/continuous closures and
valid shrinking enclosures; the calculator's parser accepting such syntax does
not provide those guarantees. No arbitrary exact equality test is introduced.
Chapter6's fixed integers must be justified per counter, not applied to scalar
values; the donor's actual Int boundary failures demonstrate the distinction.
The proposed finite numeral terminator is already subsumed for rational values
by Hyper's exact Rational/classification layer; a general lazy-zero decision is
not available. Larger radix/word kernels likewise already exist in Hyper.

Chapter7's digit-demand, branching and digit-swell distinctions are supported
by the existing paired benchmarks and actual-demand qualification, with the
instrumentation omissions explicitly retained. Its approximate SPARC timings
include process start/decimal formatting and are not comparable to Hyper's
kernel timings. Periodic two-input experiments are not universal demand proofs.
The ten-iteration dyadic logistic numerator remains an unqualified historical
example until a matching valid recurrence/input representation is established.

For its expression-tree recommendation, reread current Hyper
real/arithmetic/add_sub.rs210-345, rational/arithmetic/ops.rs1420-1565 and
real/arithmetic/mul_div.rs465-530. Real Sum balances proven-long heterogeneous
iterators at256 with streaming carry slots, preserving homogeneous symbolic
folding. Rational Product balances512-factor chunks with logarithmic partials,
zero/one handling and iterator exhaustion. Real Product uses this for its exact
prefix and intentionally preserves mixed symbolic order. These targeted choices
are not a claim that every arbitrary binary expression is globally optimized.
No unconditional reassociation or generic optimization compiler is selected.

## Report/fixture numerical controls

The modern compatibility calculator runs the original solaris/test1.era input
to30 decimal places and exits0. Its output is within10^-30 of the independent
exact result -54767/66192. The observed wall/CPU/RSS in its single command log
are diagnostics only, not benchmark evidence or a cross-language speed ratio.
Maple and SPARC binaries remain unexecuted. The report fixture Rust harness
checks100 neighboring/sign-varied input pairs and two coefficient decompositions:
200 exact Real equalities and1200 Computable precision/history enclosures pass
in release and debug. Public exact folding is allowed; no opaque/raw-node test
claim. Its first two compile failures were audit-only Rug expression completion/
type-annotation errors, preserved separately and fixed before successful runs.

For test2.mws, exact squared inequalities prove the critical point1/sqrt(60)
lies in[0.129099,0.1291], and prove that its value30-2*sqrt(60) is enclosed by
the archived midpoint1857/128 with radius1/256. This qualifies the recorded
prefix, not the encoded Maple plot or the runtime completeness of fnmax.
The same executable checks the6561 dyadic identities and189 carry invariants
described above, recording rejected printed-formula counts separately.
Commands/logs: plume-qualification/report-fixture-README.md and
report-fixture-controls{,-debug}.log. No new Hyper source is retained.

## Later functional/logistic qualification

See plume-qualification/late-README.md for commands, exact bridges, frozen
sources and numerical/performance limitations. All original174 hashes still
match; only the scratch Tests export list and one final blank line differ.
Main numerical modules and earlier frozen probes are unchanged.

All392 short logistic requests run in both O0/O2;350 prefixes return and are
byte-identical, with42 observations hitting750ms caps. Of the returned values,
28 dyadic-stream outputs fail the independent oracle; the other six variants'
returned outputs pass. This distinguishes the unbounded-normalization wrappers
from the demo's unnormalized step and tests greedy/delayed/decimal inputs.
The broader42 requests at10/40/60 iterations produce34 prefixes, of which two
dyadic-stream values are wrong; eight dyadic requests reach2s caps. The report's
large first digit is not reproduced: the greedy half representation has exponent
2047, not2558, at iteration10; delayed representation gives zero. This active
dyadic-stream map is wrong, so its digit growth is not reliable logistic evidence.

For the four selected functional fixture families, O2 returns12/16 and O0
10/16 before3s caps; all finite results and common prefixes qualify. This now
reproduces the reciprocal maximum worksheet enclosure at8 fractional bits.
No generic total-function runtime/completeness assertion follows from those tests.

The independent4096-bit outward interval oracle passes91 controls against exact
rational recurrence through12 iterations. Hyper's public Computable version
passes160 precision/history checks in release/debug, including60 iterations.
Memcheck has zero memory errors, not a leak claim. Reread node/algebra.rs
1160-1450,1625-1730 and structural_analysis.rs1540-1574: although exact leaves
fold, multiplying by4 creates an Offset, so this workload does not suffer the
initially suspected eager rational expansion. Do not implement a policy change
on the basis of that disproved concern. General huge pure-rational operations
are a distinct contract, not established defective by this experiment.

The correct signed and dyadic-float wrappers were benchmarked separately from
the faulty dyadic-stream code:108 CPU6-pinned, alternating-order, exact-qualified
observations;96 after warmups. At10 iterations, dyadic/signed median CPU ratios
are147.27 and124.42 on the two inputs, and allocation ratios90.65 and103.30.
The full construction/input-evaluation/prefix-forcing workload is measured,
excluding startup and oracle checking. Managed cumulative allocation is not RSS.
Earlier demand/swell lessons remain consistent, with no Hyper speed claim and
no new representation replacement selected. All rows and uncertainty summaries
are in late-summary.json. No new production change is retained.

index.html1-618 is completely read. The two report text extractions were compared
byte-for-byte and all five differing layout hunks read; author-report.txt is not
thereby independently credited read in full. Remaining report visuals and
alternate-container coverage remain explicit below.

## Still open

Finish remaining mathematical/figure visual checks;
review original author PDF/dbp alternate report containers as appropriate;
qualify remaining report/fixture claims and
remaining normalization/conversion/logistic-map/functional ideas. No whole
Plume audit or transfer closure is claimed at this checkpoint.

## Additional rendered-page checks

Physical pages49-64,71-80,99-110,123-133 are now visually read, alongside
previous pages35 and40 (51 of133). This completes the already-rendered batch,
not all report visuals. Text coverage is separate and already complete.
New printed-formula discrepancies, using printed page numbers:

- p41: the clipped subtract-one function's cases for first digit+1 and-1
  are swapped. The source mone_plus_posx uses the correct tail/constant cases.
- p43: p(0,1::x') and p(-1,0::x') display1-x' where the required clipped
  residual is1+x'. The actual source calls one_plus_negx, which computes
  min(1,1+x'). The prefix identity applies only when the requested prefix
  digit can represent the input; the clipped extension is not an identity
  outside that domain. Exact independent controls are being added separately.
- p55's final derivation writes divide(x'',4y), inconsistent with its own
  kernel convention divide(x,y)=x/(4y). Its preceding recurrence correctly
  uses divide(x'',y); do not insert another factor4 into the source.
- p70's displayed positive-exp tail implication needs n>=1: n=0 satisfies
  x^n/n!<=1 for every x but the resulting upper bound2 fails, e.g. at x=2.
  For n>=1, AM-GM gives x<=(n!)^(1/n)<=(n+1)/2 and the remaining terms
  are bounded by a geometric tail. The separately stated eventual-N condition
  is stronger than the isolated implication. No new source failure inferred.
- p115 says division is closed over dyadic rationals, which is false:
  1/(3/4)=4/3. Canonical dyadic pair equality on p118 is valid only under the
  stated canonical-form invariant; the public donor constructor does not
  enforce it, as already recorded. Hyper uses its exact Rational fallback.

The additional demand/branching tables, Appendix B division/limit code and
Appendix C transcript are now visually corroborated. The numerical and
performance limitations recorded above still apply; none is a new benchmark.

## Boundary qualification and retained Hyper Display improvement

See plume-qualification/boundary-README.md and boundary-summary.json. All174
donor originals remain unchanged. Main SBinDec was reread in full1-327;
SBinStream1-105/352-441, DyStream1-134 and normalization call sites were
revisited. New independent grids qualify5128 checks/build with17 dyadic cap
violations, not numerical value failures. Signed clipped helpers and bounded
normalization pass. Printed subtraction/residual errors are independently
rejected while actual source helpers pass.

The2080-case main decimal-output grid has128 returned values outside the
requested decimal error, all for positive exponents and positive place counts;
output is identical underO0/O2 and independently BigInt-checked. Ignoring
fractional fcarry loses integer units at endpoints. Direct literal-helper
behavior is separately recorded; do not confuse its partial domain with the
already-qualified calculator grammar.31 productivity requests/build give21
correct finite prefixes and10 bounded observations with no result. These
reproduce the dyadic cap bug, negative-cap normalization and multiplication's
productive-zero discontinuity at exponent sum500. None is a portable scheduler.

The analogous Hyper control revealed that fixed-decimal Display unnecessarily
searched for a leading nonzero bit of a nontrivial zero cancellation. The
retained two-line replacement bounds this search by existing guarded decimal
precision, without claiming a general equality decision or changing scientific
notation. Cancellation/tiny-value regressions and cache-demand bounds pass;
2188 decimal/history plus300 approximation checks pass in debug/release.
Full Hyperreal and downstream evidence is recorded in the boundary README.

Same-source paired before/after benchmarks have72 qualified CPU6 process rows,
64 after warmups. Tiny-radical formatting is140.918x faster by median CPU and
requests99.718% fewer allocation bytes. Ordinary-rational/radical intervals
include parity; no general speedup claim. Audit-binary loaded size is unchanged,
file size+64B. This is the first retained Hyper change from the Plume slice.
The remaining76 unviewed report pages and alternate report containers are not
closed by these numerical tests. Overall reference inventory remains ACTIVE.

Physical report65-70 were subsequently rendered and visually read, bringing
coverage to57/133. Positive quotient residual formulas on printed57-58 are
consistent with the kernel's x/(4y) convention; page60 is blank apart from its
header. Printed61-62 describes bound-driven digit emission and the requirement
that nested intervals actually converge to a singleton. This is not a general
unmodulated-Cauchy-sequence limit operator or an interval-contract validator.

## Remaining opening/representation report visuals

Physical1-34,36-39,41-48 were independently visually read, bringing primary
report coverage to103/133. Cover, blank pages, acknowledgements and contents
are included; none is an implementation or benchmark result. Printed1-8 gives
historical representation/language context. Printed9-16 distinguishes input
error, floating error, symbolic identities and productivity of redundant
representations. Historical timing/accuracy claims remain separate from
reproduced controls. In particular, printed11's logistic table starts at the
exactly binary-representable43/64; our earlier0.1/.5467 long controls do not
directly qualify that specific table. Its relative-error algebra does not
account for newly introduced machine-operation rounding factors.

Printed18's B-adic definition has a correct preceding error condition
|r-c_n*B^(-n)|<B^(-n), but its displayed limit divides c_n by B^(-n), reversing
the scale. For B=2,c_n=2^n,r=1 the error is zero while the printed limit's terms
are4^n. The consistent limit multiplies by B^(-n). Printed25 duplicates d_1
in its finite-prefix list and labels its range with x rather than x'; the
following prefix sum/radius and diagram are consistent. Printed36 repeats
U_-2 where the third decimal digit should be U_-3. These are report defects,
not inferred source defects or reasons to port a new kernel.

Printed19-20's fractional-transformation efficiency statement supplies no
reproducible performance evidence. Printed28-31's singleton nested intervals,
prefix range analysis and cons/average identities are consistent with the
previously qualified scoped source. Printed33-37 explicitly makes decimal
output a finite approximation and requires carrying fractional overflow to
the integer part: the source's lost fcarry contradicts that requirement and
was already independently reproduced in the boundary grid. Printed39-40
confirms the signed-stream main path and dyadic intermediate roles. No further
Hyper production edit selected by this visual slice.

Physical81-98 and111-122 were subsequently visually read, completing all133
primary report pages. Figure6.1's module/dependency layout is now corroborated;
printed73-74 makes functional continuity/closed-interval and scaling contracts
explicit. Printed75-86 covers UI/core separation, generated lexer/parser,
integer fast-path opportunities, language/tool history and actual average/
integer-division code. The negative-divisor/overflow and average-pseudocode
discrepancies are already recorded, not new failures. Printed87-90 separates
lookahead, branching and dyadic-digit swell: fewer input digits need not mean
less CPU/allocation, as the already qualified benchmark demonstrates.

Printed103's timing table is read but not accepted as a modern reproduction:
ln(2)'s source can return a wrong finite value and the archived logistic
variants differ. Printed104-110's proposed finite-number terminators,
word-sized digits and symbolic expression simplification are substantially
covered by Hyper's existing exact Rational/symbolic/approximation architecture.
Parallel stream evaluation is only a proposed direction, not an implemented
or qualified donor optimization. These pages do not justify another Hyper
change. Printed111-114's bibliography and blank closing page are read;
citations do not grant read credit to their linked external documents.
Alternate author-report and remaining linked HTML containers remain open.

The printed11 logistic table's ten correct-result entries have now been
independently qualified for input43/64 through60 iterations: all ten pass the
existing4096-bit outward MPFR oracle, and all30 checks pass an independent
integer-only outward interval implementation at1024/2048/4096 bits. The latter
passes39 unrounded exact-rational self-checks through12 iterations. The entire
enclosure is strictly inside each printed six-decimal rounding cell, so no
tie rule is assumed. Historical single/double columns and timing claims are
not reproduced.36 exact B-adic counterexamples qualify the printed scale
reversal. See plume-qualification/report-visual-README.md for commands,
hashes, historical-binary limitation and analysis.

The linked img216.gif was acquired directly from the Edinburgh project page,
585x256 GIF89a, SHA-256
12465a21095f3adfb0d64edb908c652f4c700cfe028e29112d3438e621186fe6.
All labels/arrows were visually read and agree with primary report figure6.1:
UI/core separation and mixed-representation consumers, not a different kernel.
This closes node91's missing content diagram, not unseen linked HTML pages.

## Alternate report and final disposition

All4871 lines of author-report.txt were independently read. SHA-256:
65e49cad5a4e3530429276313f2def4f2d8ade04d4673e5c77e641513c859fab.
Complete byte comparison finds ten differing lines in five layout hunks at
564,642-646,2370,3323,4693-4696; no numerical/source revision is present.
Author PDF pages20-22,65,77-80,90,104,129 were independently viewed (11 pages),
including all five difference-hunk pages. The other122 alternate visual pages
are not credited. The primary report remains fully visually read133/133.

Two additional printed errors are confirmed visually and by exact controls:

- Printed69 claims100 correct digits from10^50 Gregory-Leibniz terms. For
  S_N=sum(k=0..N-1,(-1)^k/(2k+1)), the exact remainder magnitude of4*S_N
  is4*integral_0^1 t^(2N)/(1+t^2)dt, strictly between2/(2N+1) and4/(2N+1).
  Thus10^50 terms leave error on the order of10^-50, not10^-100. The finite
  geometric identity yielding this remainder passes1056 exact rational-grid
  checks. No astronomically long computation is executed or timed.
- Printed96 gives ceiling(ln(x)) for the depth of a balanced binary expression
  whose linear analogue has n leaves. The relevant depth is ceiling(log2(n));
  16 leaves have depth4, whereas ceiling(ln(16)) is3.512 exact recursive-depth
  checks corroborate the corrected structural count. This is not a new
  empirical demand bound for arbitrary shared Hyper DAGs.

Printed72 also fails to distinguish the range-reduced x' in[1/2,3/2] from its
recentered x'-1 when describing the ln(1+u) kernel. The actual source and its
separate range/remainder faults were already read and qualified; neither this
wording nor the two new report errors justifies a Hyper arithmetic edit.

analyze-author-report.mjs checks hashes, exact controls and the five-hunk
comparison and generates PLUME_HTML_LINK_INVENTORY.tsv:147 generated section
links present in the fully read LaTeX2HTML index. Three section bodies were
independently read;144 remain indexed-only. This uses the root ledger's
existing generated-artifact convention, not a claim of reading those bodies
or of auditing every linked bibliography item. Both full report texts and all
primary visuals supply the algorithm/semantic documentation audit.

| Candidate | Disposition against Hyper |
| --- | --- |
| Bound normalization/sign search by requested output information | Retained in fixed-decimal Display; boundary-README.md contains full tests, paired benchmark, allocation and size evidence. Scientific formatting unchanged. |
| Digit streaming, larger-base digits and finite terminators | Existing batched dyadic approximations, exact Rational values and symbolic facts cover the relevant scalar benefits; no representation replacement justified. |
| Dyadic intermediate arithmetic with lower lookahead | Scoped numerical controls expose defects; qualified CPU/allocation comparisons show swell costs. Hyper already avoids unnecessary unbounded exact intermediates. |
| Bound-driven limits and total continuous scalar operations | Already represented by Hyper approximation/refinement and the preceding continuous-absolute-value improvement; no unmodulated arbitrary Cauchy limit or decidable equality claim. |
| Generic integration and function min/max | Require a certified continuous-function/compact-domain contract, not an arbitrary closure. Source returns valid finite fixtures but also caps; no present scalar or geometry consumer justifies this separate API from these algorithms. |
| Tagged lookahead, expression balancing and reuse | Tags can under-report demand; lower demand is not a timing oracle. Existing scale-aware scheduling, DAG sharing and exact aggregate kernels cover current consumers; paired experiments recorded above. |
| Native helper, parser and formatting implementations | Independent grids expose numeric/domain/overflow/productivity faults. No donor code copied; old SPARC executables remain unexecuted and CGI endpoints unlaunched. |

All174 original source-container artifacts remain unchanged;170 text files/
24049 physical lines were independently read. The sole retained Plume-derived
production edit is the qualified fixed-decimal Display change. Generated
document projections, obsolete native-runtime behavior and timeout observations
retain their explicit limitations. This closes the Plume source/targeted-transfer
slice, not the broad ecosystem goal or any other pending reference.
